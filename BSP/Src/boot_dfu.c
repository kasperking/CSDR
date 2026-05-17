/* USER CODE BEGIN Header */
/**
 * @file  boot_dfu.c
 * @brief Power-on USB DFU mode detection and STM32H750 ROM bootloader jump.
 *
 *  Designed to be called from MX_FMC_Init() USER CODE FMC_Init 2, immediately
 *  after LCD_Bus_Init() and before MX_USB_DEVICE_Init().  At that point:
 *    - GPIO is initialised  → buttons are readable
 *    - FMC + ST7796S is up  → LCD_FillRect / LCD_Clear are safe
 *    - USB stack NOT started → clean slate for ROM DFU enumeration
 *    - No DSP, no FreeRTOS, no audio DMA
 *
 *  Text is rendered with direct LCD_FillRect calls (no line buffer, no
 *  DMA_SRAM dependency) so the DFU path works before the SRAM1 MPU region
 *  has been granted full access.
 */
/* USER CODE END Header */

#include "boot_dfu.h"
#include "main.h"           /* PW_Pin / PW_GPIO_Port, ENC_SW_Pin / ENC_SW_GPIO_Port */
#include "stm32h7xx_hal.h"
#include "lcd_bus_fmc.h"    /* LCD_Clear, LCD_FillRect, LCD_W, LCD_H */
#include "lcd_render.h"     /* Font6x8 (extern const Font_t) */

/* ── STM32H750VB ROM bootloader base ──────────────────────────────────────────
 * System memory for STM32H750/H742/H743: 0x1FF09800.
 * Verify against ST AN2606 Table 8 when targeting a different H7 variant.
 */
#define DFU_BOOTLOADER_ADDR  0x1FF09800UL

/* ── Private: direct FillRect text engine (no line buffer) ───────────────── */

/* Draw one character at pixel position (px, py), scaling each font pixel to
 * a scale×scale square.  Background is assumed already cleared (black); only
 * foreground (lit) pixels are written to minimise FMC traffic. */
static void draw_char_scaled(char c, uint16_t px, uint16_t py,
                              uint8_t scale, uint16_t fg)
{
    /* Font covers ASCII 32–90; fold lowercase to uppercase */
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    if ((uint8_t)c < 32U || (uint8_t)c > 90U) return;

    const uint8_t *bmp = Font6x8.data + ((uint8_t)c - 32U) * Font6x8.width;

    for (uint8_t row = 0U; row < Font6x8.height; row++) {
        uint16_t y0 = (uint16_t)(py + (uint16_t)row * scale);
        for (uint8_t col = 0U; col < Font6x8.width; col++) {
            if ((bmp[col] >> row) & 1U) {
                uint16_t x0 = (uint16_t)(px + (uint16_t)col * scale);
                LCD_FillRect(x0, y0,
                             (uint16_t)(x0 + scale - 1U),
                             (uint16_t)(y0 + scale - 1U),
                             fg);
            }
        }
    }
}

/* Render a string horizontally centred on the 480-wide display at row y.
 * scale: pixel magnification (1–4).  fg: RGB565 foreground colour. */
static void draw_str_centered(const char *str, uint16_t y,
                               uint8_t scale, uint16_t fg)
{
    /* Count characters without pulling in <string.h> */
    uint16_t len = 0U;
    const char *p = str;
    while (*p++) len++;

    uint16_t total_w = (uint16_t)((uint16_t)(Font6x8.width * scale) * len);
    uint16_t x       = (uint16_t)((LCD_W - total_w) / 2U);

    while (*str) {
        draw_char_scaled(*str++, x, y, scale, fg);
        x = (uint16_t)(x + (uint16_t)(Font6x8.width * scale));
    }
}

/* ── Public API ───────────────────────────────────────────────────────────── */

bool boot_dfu_requested(void)
{
    /* Both pins have PULLUP; active-low → pressed == GPIO_PIN_RESET */
    return (HAL_GPIO_ReadPin(PW_GPIO_Port,     PW_Pin)     == GPIO_PIN_RESET) &&
           (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET);
}

void ui_show_dfu_screen(void)
{
    /* Black background */
    LCD_Clear(0x0000U);

    /* Layout for 480×320, Font6x8 (6×8 px):
     *   Title  "USB DFU MODE"     — scale 4 → 24×32 px/char
     *   gap                       — 16 px
     *   Subtitle "Connect to PC..." — scale 2 → 12×16 px/char
     *
     *   Total block height = 32 + 16 + 16 = 64 px
     *   title_y = (320 - 64) / 2 = 128
     *   sub_y   = 128 + 32 + 16  = 176
     */
    const uint16_t title_h = (uint16_t)(Font6x8.height * 4U);   /* 32 */
    const uint16_t sub_h   = (uint16_t)(Font6x8.height * 2U);   /* 16 */
    const uint16_t block_h = (uint16_t)(title_h + 16U + sub_h); /* 64 */
    const uint16_t title_y = (uint16_t)((LCD_H - block_h) / 2U);
    const uint16_t sub_y   = (uint16_t)(title_y + title_h + 16U);

    draw_str_centered("USB DFU MODE",     title_y, 4U, 0xFFFFU); /* white */
    draw_str_centered("Connect to PC...", sub_y,   2U, 0x07FFU); /* cyan  */
}

void boot_enter_dfu(void)
{
    typedef void (*BootFn)(void);

    /* ── 1. Halt all interrupt sources ───────────────────────────────────── */
    __disable_irq();
    SysTick->CTRL = 0U;     /* stop SysTick; prevent re-entry via tick handler */

    /* Clear all NVIC enables and pending bits (8 registers × 32 IRQs = 256) */
    for (uint32_t i = 0U; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;   /* disable */
        NVIC->ICPR[i] = 0xFFFFFFFFU;   /* clear pending */
    }

    /* ── 2. Clean up caches and memory protection ─────────────────────────
     * SCB_DisableDCache() flushes + invalidates before disabling.
     * The bootloader configures its own MPU regions.               */
    SCB_DisableDCache();
    SCB_DisableICache();
    HAL_MPU_Disable();

    /* ── 3. Reset RCC to HSI default ─────────────────────────────────────
     * HAL_RCC_DeInit() gates all peripheral clocks and returns the PLL
     * to reset state.  The ROM bootloader reconfigures clocks itself.  */
    HAL_RCC_DeInit();

    /* ── 4. Redirect vector table and jump ───────────────────────────────
     * The ROM vector table at DFU_BOOTLOADER_ADDR holds:
     *   [+0]  initial MSP value
     *   [+4]  reset handler (bootloader entry point)                    */
    SCB->VTOR = DFU_BOOTLOADER_ADDR;
    __DSB();

    __set_MSP(*(volatile uint32_t *)DFU_BOOTLOADER_ADDR);
    __ISB();

    BootFn bootloader = (BootFn)(*(volatile uint32_t *)(DFU_BOOTLOADER_ADDR + 4U));
    bootloader();

    /* Never reached */
    while (1) {}
}
