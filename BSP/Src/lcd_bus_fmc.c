/* USER CODE BEGIN Header */
/**
 * @file  lcd_bus_fmc.c
 * @brief FMC 8080-mode LCD bus driver – ST7796S 480×320 on STM32H750.
 *
 *  Memory map:
 *    0x60000000 → LCD command  (FMC A16=0, RS/DC LOW)
 *    0x60010000 → LCD data     (FMC A16=1, RS/DC HIGH)
 *
 *  MPU note:
 *    LCD_Bus_Init() installs MPU Region 1 covering 0x60000000–0x6001FFFF
 *    as Strongly-Ordered (TEX=0,C=0,B=0).  Without this the D-Cache
 *    may buffer or reorder writes, corrupting the LCD output even if
 *    FMC timing is correct.
 *
 *  Orientation: landscape 480×320, origin top-left, X increases right.
 *  MADCTL=0xE8 (MY|MX|MV|BGR).  Panel glass requires MY+MX; color filter is BGR.
 */
/* USER CODE END Header */

#include "lcd_bus_fmc.h"
#include "stm32h7xx_hal.h"
#include "main.h"      /* LCD_RESET_Pin / LCD_RESET_GPIO_Port (PD13 from .ioc) */

/* ── MPU region 1: FMC LCD space, Strongly-Ordered ──────────────────────── */
static void fmc_lcd_mpu_config(void)
{
    MPU_Region_InitTypeDef r = {0};

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HAL_MPU_Disable();

    r.Enable           = MPU_REGION_ENABLE;
    r.Number           = MPU_REGION_NUMBER1;
    r.BaseAddress      = 0x60000000UL;
    r.Size             = MPU_REGION_SIZE_128KB;    /* covers 0x0000 and 0x10000 offsets */
    r.SubRegionDisable = 0x00U;
    r.TypeExtField     = MPU_TEX_LEVEL0;            /* TEX=0 */
    r.AccessPermission = MPU_REGION_FULL_ACCESS;
    r.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    r.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;  /* S=0 */
    r.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;  /* C=0 */
    r.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE; /* B=0 → Strongly-Ordered */
    HAL_MPU_ConfigRegion(&r);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    if (!primask) __enable_irq();
}

/* ── Low-level bus primitives ────────────────────────────────────────────── */

void LCD_WriteCmd(uint8_t cmd)
{
    *LCD_FMC_CMD_ADDR = cmd;
}

void LCD_WriteData8(uint8_t data)
{
    *LCD_FMC_DATA_ADDR = data;
}

/* ── LCD_WriteDataBuffer ──────────────────────────────────────────────────────
 * Raw RGB565 buffer: sends MSB then LSB for each pixel.
 * 4× loop unroll reduces branch overhead for large spectrum/waterfall blasts.
 */
void LCD_WriteDataBuffer(const uint16_t *buf, uint32_t count)
{
    while (count >= 4U) {
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[0] >> 8); *LCD_FMC_DATA_ADDR = (uint8_t)(buf[0]);
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[1] >> 8); *LCD_FMC_DATA_ADDR = (uint8_t)(buf[1]);
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[2] >> 8); *LCD_FMC_DATA_ADDR = (uint8_t)(buf[2]);
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[3] >> 8); *LCD_FMC_DATA_ADDR = (uint8_t)(buf[3]);
        buf += 4U; count -= 4U;
    }
    while (count--) {
        *LCD_FMC_DATA_ADDR = (uint8_t)(*buf >> 8);
        *LCD_FMC_DATA_ADDR = (uint8_t)(*buf++);
    }
}

/* ── LCD_PushWindow ───────────────────────────────────────────────────────────
 * Sets the CASET/RASET window then writes a SWAP16-encoded pixel buffer.
 *
 * The UI rendering layer stores pixels byte-swapped (SWAP16 convention) so
 * that SPI-DMA byte order was correct on the wire.  For FMC the write order
 * is explicit: we must send the original MSB first.  After SWAP16, the
 * original MSB lands in the *low* byte of the stored uint16_t, so we emit
 * (uint8_t)(px) first then (uint8_t)(px >> 8).
 *
 * Example: red (0xF800) → SWAP16 → 0x00F8 stored in buffer.
 *   Emit: (uint8_t)(0x00F8) = 0xF8 [MSB, correct], (uint8_t)(0x00F8>>8) = 0x00 [LSB]
 *   LCD receives: 0xF8 0x00 → interprets as 0xF800 = red ✓
 *
 * 4× unrolled for spectrum (12 160 px) and waterfall (9 600 px) blasts.
 * No IRQ disable — FMC writes are synchronous memory-mapped operations.
 */
void LCD_PushWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                    const uint16_t *buf, uint32_t npix)
{
    LCD_SetWindow(x0, y0, x1, y1);

    while (npix >= 4U) {
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[0]);       *LCD_FMC_DATA_ADDR = (uint8_t)(buf[0] >> 8);
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[1]);       *LCD_FMC_DATA_ADDR = (uint8_t)(buf[1] >> 8);
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[2]);       *LCD_FMC_DATA_ADDR = (uint8_t)(buf[2] >> 8);
        *LCD_FMC_DATA_ADDR = (uint8_t)(buf[3]);       *LCD_FMC_DATA_ADDR = (uint8_t)(buf[3] >> 8);
        buf += 4U; npix -= 4U;
    }
    while (npix--) {
        *LCD_FMC_DATA_ADDR = (uint8_t)(*buf);
        *LCD_FMC_DATA_ADDR = (uint8_t)(*buf++ >> 8);
    }
}

/* ── Window / fill / pixel helpers ──────────────────────────────────────── */

void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    LCD_WriteCmd(ST7796_CASET);
    LCD_WriteData8((uint8_t)(x0 >> 8));
    LCD_WriteData8((uint8_t)(x0));
    LCD_WriteData8((uint8_t)(x1 >> 8));
    LCD_WriteData8((uint8_t)(x1));

    LCD_WriteCmd(ST7796_RASET);
    LCD_WriteData8((uint8_t)(y0 >> 8));
    LCD_WriteData8((uint8_t)(y0));
    LCD_WriteData8((uint8_t)(y1 >> 8));
    LCD_WriteData8((uint8_t)(y1));

    LCD_WriteCmd(ST7796_RAMWR);
}

void LCD_WritePixel(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_SetWindow(x, y, x, y);
    LCD_WriteData16(color);
}

void LCD_FillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    uint32_t npix = (uint32_t)(x1 - x0 + 1U) * (uint32_t)(y1 - y0 + 1U);
    uint8_t  hi   = (uint8_t)(color >> 8);
    uint8_t  lo   = (uint8_t)(color);

    LCD_SetWindow(x0, y0, x1, y1);

    while (npix--) {
        *LCD_FMC_DATA_ADDR = hi;
        *LCD_FMC_DATA_ADDR = lo;
    }
}

void LCD_Clear(uint16_t color)
{
    LCD_FillRect(0U, 0U, LCD_W - 1U, LCD_H - 1U, color);
}

/* ── ST7796S initialisation sequence ─────────────────────────────────────── */

static void lcd_init_sequence(void)
{
    /* Software reset */
    LCD_WriteCmd(ST7796_SWRESET);
    HAL_Delay(120U);

    /* Sleep out */
    LCD_WriteCmd(ST7796_SLPOUT);
    HAL_Delay(120U);

    /* Unlock extended command set (level 1 + 2) */
    LCD_WriteCmd(0xF0U); LCD_WriteData8(0xC3U);
    LCD_WriteCmd(0xF0U); LCD_WriteData8(0x96U);

    /* Memory access control: MY|MX|MV|BGR (0xE8).
     * MY+MX correct the panel glass orientation; MV selects landscape scan;
     * BGR matches the panel color filter order (bits[15:11]=B, bits[4:0]=R). */
    LCD_WriteCmd(ST7796_MADCTL);
    LCD_WriteData8(ST7796_MADCTL_LANDSCAPE);

    /* Pixel format: RGB565 (16 bpp) */
    LCD_WriteCmd(ST7796_COLMOD);
    LCD_WriteData8(ST7796_COLMOD_16BIT);

    /* Display function control */
    LCD_WriteCmd(0xB6U);
    LCD_WriteData8(0x80U);
    LCD_WriteData8(0x02U);
    LCD_WriteData8(0x3BU);

    /* Interface timing / digital control (DOCA) */
    LCD_WriteCmd(0xE8U);
    LCD_WriteData8(0x40U); LCD_WriteData8(0x8AU); LCD_WriteData8(0x00U); LCD_WriteData8(0x00U);
    LCD_WriteData8(0x29U); LCD_WriteData8(0x19U); LCD_WriteData8(0xA5U); LCD_WriteData8(0x33U);

    /* Power control 2 */
    LCD_WriteCmd(0xC1U); LCD_WriteData8(0x06U);
    /* Power control 3 (normal mode) */
    LCD_WriteCmd(0xC2U); LCD_WriteData8(0xA7U);
    /* VCOM */
    LCD_WriteCmd(0xC5U); LCD_WriteData8(0x18U);

    HAL_Delay(120U);

    /* Positive gamma curve */
    LCD_WriteCmd(0xE0U);
    LCD_WriteData8(0xF0U); LCD_WriteData8(0x09U); LCD_WriteData8(0x0BU);
    LCD_WriteData8(0x06U); LCD_WriteData8(0x04U); LCD_WriteData8(0x15U);
    LCD_WriteData8(0x2FU); LCD_WriteData8(0x54U); LCD_WriteData8(0x42U);
    LCD_WriteData8(0x3CU); LCD_WriteData8(0x17U); LCD_WriteData8(0x14U);
    LCD_WriteData8(0x18U); LCD_WriteData8(0x1BU);

    /* Negative gamma curve */
    LCD_WriteCmd(0xE1U);
    LCD_WriteData8(0xE0U); LCD_WriteData8(0x09U); LCD_WriteData8(0x0BU);
    LCD_WriteData8(0x06U); LCD_WriteData8(0x04U); LCD_WriteData8(0x03U);
    LCD_WriteData8(0x2BU); LCD_WriteData8(0x43U); LCD_WriteData8(0x42U);
    LCD_WriteData8(0x3BU); LCD_WriteData8(0x16U); LCD_WriteData8(0x14U);
    LCD_WriteData8(0x17U); LCD_WriteData8(0x1BU);

    /* Lock extended command set */
    LCD_WriteCmd(0xF0U); LCD_WriteData8(0x3CU);
    LCD_WriteCmd(0xF0U); LCD_WriteData8(0x69U);

    /* Normal display mode on */
    LCD_WriteCmd(ST7796_NORON);
    HAL_Delay(10U);

    /* Display on */
    LCD_WriteCmd(ST7796_DISPON);
    HAL_Delay(25U);
}

/* ── Public init ─────────────────────────────────────────────────────────── */

/*
 * WHY BOTH HARDWARE AND SOFTWARE RESET?
 *
 * Hardware reset (RESX pulse via PD13):
 *   Forces the ST7796 to its power-on state unconditionally — works even if
 *   the bus is in an unknown state from a previous partial init, watchdog
 *   restart, or warm reboot.  All internal registers return to factory
 *   defaults and the display controller halts any ongoing DBI transfer.
 *
 * Software reset (command 0x01, SWRESET) that follows in lcd_init_sequence():
 *   Provides an additional full reset of the ST7796 digital and analog
 *   sections after any startup oscillator transients have settled.  The
 *   datasheet recommends issuing SWRESET after RESX to guarantee a clean
 *   analog power-up baseline before programming gamma, VCOM, and power
 *   control registers.
 *
 * Together they give a deterministic, order-independent init regardless
 * of the MCU reset source (power-on, NRST pin, watchdog, software reset).
 *
 * Compile-time fallback:
 *   If LCD_RESET_Pin is not defined (board without dedicated reset pin),
 *   hardware reset is skipped — only software reset runs.  The #ifdef
 *   guard makes this safe without separate build variants.
 */
void LCD_Bus_Init(void)
{
    fmc_lcd_mpu_config();

#ifdef LCD_RESET_Pin
    /* Assert hardware reset: hold RESX low for 10 ms (≥1 ms per ST7796 spec) */
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10U);
    /* Deassert and wait ≥120 ms for display oscillator and analog supply to settle
     * before the first command (SWRESET in lcd_init_sequence).                    */
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(120U);
#endif

    lcd_init_sequence(); /* begins with SWRESET + 120 ms, then full register prog */
}
