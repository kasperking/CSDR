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
/* lcd_render.h intentionally NOT included: Font6x8.data is NULL at DFU time
 * (loaded from SPI assets by LCD_Render_Init which never runs in DFU path).
 * The self-contained s_dfu_font_data array above replaces Font6x8 here.     */

/* ── STM32H750VB ROM bootloader base ──────────────────────────────────────────
 * System memory for STM32H750/H742/H743: 0x1FF09800.
 * Verify against ST AN2606 Table 8 when targeting a different H7 variant.
 */
#define DFU_BOOTLOADER_ADDR  0x1FF09800UL

/* ── Self-contained 6×8 font ─────────────────────────────────────────────────
 * Font6x8 (from lcd_render.c) is NULL until LCD_Render_Init() loads SPI assets.
 * LCD_Render_Init() is called from CSDR_Init(), which never runs in the DFU
 * path.  Reading from a NULL font pointer produces vector-table garbage on screen.
 *
 * This local copy is placed in internal flash and always accessible.
 * ASCII 32-90, column-major bitmap, LSB = top row, 6 bytes per glyph.        */
static const uint8_t s_dfu_font_data[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5F,0x00,0x00,0x00,
  0x00,0x07,0x00,0x07,0x00,0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,
  0x24,0x2A,0x7F,0x2A,0x12,0x00,0x23,0x13,0x08,0x64,0x62,0x00,
  0x36,0x49,0x55,0x22,0x50,0x00,0x00,0x05,0x03,0x00,0x00,0x00,
  0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x41,0x22,0x1C,0x00,0x00,
  0x14,0x08,0x3E,0x08,0x14,0x00,0x08,0x08,0x3E,0x08,0x08,0x00,
  0x00,0x50,0x30,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x00,
  0x00,0x60,0x60,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x00,
  0x3E,0x51,0x49,0x45,0x3E,0x00,0x00,0x42,0x7F,0x40,0x00,0x00,
  0x42,0x61,0x51,0x49,0x46,0x00,0x21,0x41,0x45,0x4B,0x31,0x00,
  0x18,0x14,0x12,0x7F,0x10,0x00,0x27,0x45,0x45,0x45,0x39,0x00,
  0x3C,0x4A,0x49,0x49,0x30,0x00,0x01,0x71,0x09,0x05,0x03,0x00,
  0x36,0x49,0x49,0x49,0x36,0x00,0x06,0x49,0x49,0x29,0x1E,0x00,
  0x00,0x36,0x36,0x00,0x00,0x00,0x00,0x56,0x36,0x00,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00,0x00,0x14,0x14,0x14,0x14,0x14,0x00,
  0x00,0x41,0x22,0x14,0x08,0x00,0x02,0x01,0x51,0x09,0x06,0x00,
  0x3E,0x41,0x5D,0x55,0x1E,0x00,0x7E,0x11,0x11,0x11,0x7E,0x00,
  0x7F,0x49,0x49,0x49,0x36,0x00,0x3E,0x41,0x41,0x41,0x22,0x00,
  0x7F,0x41,0x41,0x22,0x1C,0x00,0x7F,0x49,0x49,0x49,0x41,0x00,
  0x7F,0x09,0x09,0x09,0x01,0x00,0x3E,0x41,0x49,0x49,0x7A,0x00,
  0x7F,0x08,0x08,0x08,0x7F,0x00,0x00,0x41,0x7F,0x41,0x00,0x00,
  0x20,0x40,0x41,0x3F,0x01,0x00,0x7F,0x08,0x14,0x22,0x41,0x00,
  0x7F,0x40,0x40,0x40,0x40,0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,
  0x7F,0x04,0x08,0x10,0x7F,0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,
  0x7F,0x09,0x09,0x09,0x06,0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,
  0x7F,0x09,0x19,0x29,0x46,0x00,0x46,0x49,0x49,0x49,0x31,0x00,
  0x01,0x01,0x7F,0x01,0x01,0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,
  0x1F,0x20,0x40,0x20,0x1F,0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,
  0x63,0x14,0x08,0x14,0x63,0x00,0x07,0x08,0x70,0x08,0x07,0x00,
  0x61,0x51,0x49,0x45,0x43,0x00,
};
#define DFU_FONT_W  6U
#define DFU_FONT_H  8U

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

    const uint8_t *bmp = s_dfu_font_data + ((uint8_t)c - 32U) * DFU_FONT_W;

    for (uint8_t row = 0U; row < DFU_FONT_H; row++) {
        uint16_t y0 = (uint16_t)(py + (uint16_t)row * scale);
        for (uint8_t col = 0U; col < DFU_FONT_W; col++) {
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

    uint16_t total_w = (uint16_t)((uint16_t)(DFU_FONT_W * scale) * len);
    uint16_t x       = (uint16_t)((LCD_W - total_w) / 2U);

    while (*str) {
        draw_char_scaled(*str++, x, y, scale, fg);
        x = (uint16_t)(x + (uint16_t)(DFU_FONT_W * scale));
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
    const uint16_t title_h = (uint16_t)(DFU_FONT_H * 4U);   /* 32 */
    const uint16_t sub_h   = (uint16_t)(DFU_FONT_H * 2U);   /* 16 */
    const uint16_t block_h = (uint16_t)(title_h + 16U + sub_h); /* 64 */
    const uint16_t title_y = (uint16_t)((LCD_H - block_h) / 2U);
    const uint16_t sub_y   = (uint16_t)(title_y + title_h + 16U);

    draw_str_centered("USB DFU MODE",     title_y, 4U, 0xFFFFU); /* white */
    draw_str_centered("Connect to PC...", sub_y,   2U, 0x07FFU); /* cyan  */
}

void boot_enter_dfu(void)
{
    /* ── Signal ROM-bootloader intent via backup register + system reset ─────
     * A direct application-context jump requires manually undoing all clock
     * and peripheral state (HAL_RCC_DeInit, cache flush, etc.).  If any clock
     * is not in the exact state the ROM bootloader expects, USB DFU will not
     * enumerate — HAL_RCC_DeInit() disables HSI48 and PLLs, and there is no
     * guarantee the ROM bootloader re-enables them before trying to start USB.
     *
     * Reliable approach: write a magic value to RTC_BKP0R (a backup-domain
     * register that survives NVIC_SystemReset), then reset.  The early check
     * in main() [USER CODE BEGIN Init] detects the magic and jumps to the ROM
     * bootloader while the system is still in hardware-reset state — HSI 64 MHz,
     * no PLLs, no peripheral clocks, caches flushed+disabled.  The ROM
     * bootloader then initialises its own clocks and USB stack cleanly.       */
    /* Blank screen before reset.  NVIC_SystemReset() resets the FMC peripheral;
     * the brief bus transition can corrupt the frame buffer with garbage pixels.
     * Filling black first leaves the display cleanly off during DFU.           */
    LCD_Clear(0x0000U);

    __disable_irq();
    SysTick->CTRL = 0U;

    /* Enable RTC APB interface clock and backup-domain write access */
    SET_BIT(RCC->APB4ENR, RCC_APB4ENR_RTCAPBEN);
    SET_BIT(PWR->CR1, PWR_CR1_DBP);
    __DSB();

    RTC->BKP0R = 0xB007DEADUL;  /* magic; checked in main() USER CODE Init */
    __DSB();

    NVIC_SystemReset();  /* backup domain is preserved through system reset */
    while(1) {}          /* never reached */
}
