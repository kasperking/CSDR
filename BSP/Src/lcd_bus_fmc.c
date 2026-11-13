/* USER CODE BEGIN Header */
/**
 * @file  lcd_bus_fmc.c
 * @brief FMC 8080-mode LCD bus driver – ST7796S / ILI9488 on STM32H750.
 *
 *  Controller-specific code is selected with LCD_CONTROLLER (the real chip).
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
#include "main.h"      /* LCD_RESET_Pin / LCD_RESET_GPIO_Port (PC8 from .ioc) */

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
#if HW_FMC_16BIT
    r.Size             = MPU_REGION_SIZE_256KB;    /* covers 0x0000 and 0x20000 offsets (16-bit addr shift) */
#else
    r.Size             = MPU_REGION_SIZE_128KB;    /* covers 0x0000 and 0x10000 offsets */
#endif
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
    *LCD_FMC_CMD_ADDR = (lcd_bus_t)cmd;
}

void LCD_WriteData8(uint8_t data)
{
    *LCD_FMC_DATA_ADDR = (lcd_bus_t)data;
}

/* ── LCD_WriteDataBuffer ──────────────────────────────────────────────────────
 * Raw RGB565 buffer (NOT SWAP16 encoded): sends each pixel to the LCD.
 * 8-bit mode : MSB then LSB per pixel (two byte-writes).
 * 16-bit mode: single halfword-write per pixel.
 * 4× loop unroll reduces branch overhead for large spectrum/waterfall blasts.
 */
void LCD_WriteDataBuffer(const uint16_t *buf, uint32_t count)
{
#if HW_FMC_16BIT
    while (count >= 4U) {
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)buf[0];
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)buf[1];
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)buf[2];
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)buf[3];
        buf += 4U; count -= 4U;
    }
    while (count--) { *LCD_FMC_DATA_ADDR = (lcd_bus_t)*buf++; }
#else
    while (count >= 4U) {
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[0] >> 8); *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[0]);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[1] >> 8); *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[1]);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[2] >> 8); *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[2]);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[3] >> 8); *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[3]);
        buf += 4U; count -= 4U;
    }
    while (count--) {
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(*buf >> 8);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(*buf++);
    }
#endif
}

/* ── LCD_PushWindow ───────────────────────────────────────────────────────────
 * Sets the CASET/RASET window then writes a SWAP16-encoded pixel buffer.
 *
 * SWAP16 convention: pixel P is stored as (P>>8)|(P<<8) so that the original
 * MSB sits in the low byte of the stored uint16_t.
 *
 * 8-bit mode:
 *   Emit low byte first (= original MSB), then high byte (= original LSB).
 *   Example: red 0xF800 → stored 0x00F8 → emit 0xF8, 0x00 → LCD gets 0xF800 ✓
 *
 * 16-bit mode:
 *   One halfword-write per pixel.  Writing the stored SWAP16 value directly
 *   would send bytes in the wrong order (D0-D7 = 0xF8, D8-D15 = 0x00 → 0x00F8).
 *   Apply __builtin_bswap16 to restore native byte order before writing:
 *   bswap16(0x00F8) = 0xF800 → D0-D7=0x00, D8-D15=0xF8 → LCD gets 0xF800 ✓
 *
 * 4× unrolled for spectrum (12 160 px) and waterfall (9 600 px) blasts.
 * No IRQ disable — FMC writes are synchronous memory-mapped operations.
 */
void LCD_PushWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                    const uint16_t *buf, uint32_t npix)
{
    LCD_SetWindow(x0, y0, x1, y1);

#if HW_FMC_16BIT
    while (npix >= 4U) {
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)__builtin_bswap16(buf[0]);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)__builtin_bswap16(buf[1]);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)__builtin_bswap16(buf[2]);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)__builtin_bswap16(buf[3]);
        buf += 4U; npix -= 4U;
    }
    while (npix--) { *LCD_FMC_DATA_ADDR = (lcd_bus_t)__builtin_bswap16(*buf++); }
#else
    while (npix >= 4U) {
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[0]);       *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[0] >> 8);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[1]);       *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[1] >> 8);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[2]);       *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[2] >> 8);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[3]);       *LCD_FMC_DATA_ADDR = (lcd_bus_t)(buf[3] >> 8);
        buf += 4U; npix -= 4U;
    }
    while (npix--) {
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(*buf);
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)(*buf++ >> 8);
    }
#endif
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

    LCD_SetWindow(x0, y0, x1, y1);

#if HW_FMC_16BIT
    while (npix--) { *LCD_FMC_DATA_ADDR = (lcd_bus_t)color; }
#else
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color);
    while (npix--) {
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)hi;
        *LCD_FMC_DATA_ADDR = (lcd_bus_t)lo;
    }
#endif
}

void LCD_Clear(uint16_t color)
{
    LCD_FillRect(0U, 0U, LCD_W - 1U, LCD_H - 1U, color);
}

/* ── ST7796S initialisation sequence ─────────────────────────────────────── */

#if LCD_CONTROLLER == LCD_PANEL_ST7796
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

    /* Memory access control — value from hardware profile (HW_LCD_MADCTL).
     * Default 0xE8 = MY|MX|MV|BGR: landscape scan, BGR filter.
     * Change orientation by updating the active hardware profile. */
    LCD_WriteCmd(ST7796_MADCTL);
    LCD_WriteData8(HW_LCD_MADCTL);

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
#endif /* LCD_PANEL_ST7796 */

/* ── ILI9488 initialisation sequence (landscape 480×320) ─────────────────────
 *
 *  Same 320×480 native glass and MIPI-DCS addressing as ST7796S, so the whole
 *  window / push / DMA path and the wide UI layout are shared.  Differences:
 *    - no F0h command-set unlock (ILI9488 extended registers are always open)
 *    - power / VCOM / gamma registers have different semantics → own values
 *    - COLMOD 0x55 (RGB565) is valid on the 8080 parallel bus used here
 *    - inversion is a module property (TN: INVOFF, some IPS: INVON); chosen by
 *      HW_LCD_INVERT from hw_config.py.  There is NO software pre-invert —
 *      INVON here exists to make an IPS glass look correct.
 *
 *  Register values: common ILI9488 vendor defaults for 3.5" 320×480 modules
 *  (gamma, VREG1OUT/VREG2OUT ±4.75 V, VCOM, 60 Hz frame rate).
 *  Order: SWRESET → registers → SLPOUT(120 ms) → DISPON.  Programming the
 *  analog registers before leaving sleep avoids a flash of uncalibrated gamma.
 */
#if LCD_CONTROLLER == LCD_PANEL_ILI9488
static void lcd_init_sequence(void)
{
    /* Software reset — wait 120 ms (worst case per datasheet) */
    LCD_WriteCmd(ILI9488_SWRESET);
    HAL_Delay(120U);

    /* Positive gamma */
    LCD_WriteCmd(ILI9488_PGAMCTRL);
    LCD_WriteData8(0x00U); LCD_WriteData8(0x03U); LCD_WriteData8(0x09U);
    LCD_WriteData8(0x08U); LCD_WriteData8(0x16U); LCD_WriteData8(0x0AU);
    LCD_WriteData8(0x3FU); LCD_WriteData8(0x78U); LCD_WriteData8(0x4CU);
    LCD_WriteData8(0x09U); LCD_WriteData8(0x0AU); LCD_WriteData8(0x08U);
    LCD_WriteData8(0x16U); LCD_WriteData8(0x1AU); LCD_WriteData8(0x0FU);

    /* Negative gamma */
    LCD_WriteCmd(ILI9488_NGAMCTRL);
    LCD_WriteData8(0x00U); LCD_WriteData8(0x16U); LCD_WriteData8(0x19U);
    LCD_WriteData8(0x03U); LCD_WriteData8(0x0FU); LCD_WriteData8(0x05U);
    LCD_WriteData8(0x32U); LCD_WriteData8(0x45U); LCD_WriteData8(0x46U);
    LCD_WriteData8(0x04U); LCD_WriteData8(0x0EU); LCD_WriteData8(0x0DU);
    LCD_WriteData8(0x35U); LCD_WriteData8(0x37U); LCD_WriteData8(0x0FU);

    /* Power control 1: VREG1OUT / VREG2OUT */
    LCD_WriteCmd(ILI9488_PWCTR1);
    LCD_WriteData8(0x17U); LCD_WriteData8(0x15U);

    /* Power control 2: VGH / VGL step-up factor */
    LCD_WriteCmd(ILI9488_PWCTR2);
    LCD_WriteData8(0x41U);

    /* VCOM control: NV memory not used, VCM_REG from register */
    LCD_WriteCmd(ILI9488_VMCTR1);
    LCD_WriteData8(0x00U); LCD_WriteData8(0x12U); LCD_WriteData8(0x80U);

    /* Memory access control — value from hardware profile (HW_LCD_MADCTL).
     * 0xE8 = MY|MX|MV|BGR landscape (same mounting as the ST7796 module);
     * 0x28 = MV|BGR is the 180°-rotated alternative. */
    LCD_WriteCmd(ILI9488_MADCTL);
    LCD_WriteData8(HW_LCD_MADCTL);

    /* Pixel format: RGB565 (DBI 16 bpp) — 2 WR/pixel on 8-bit, 1 on 16-bit */
    LCD_WriteCmd(ILI9488_COLMOD);
    LCD_WriteData8(ILI9488_COLMOD_16BIT);

    /* Interface mode control: default 8080 polarity */
    LCD_WriteCmd(ILI9488_IFMODE);
    LCD_WriteData8(0x00U);

    /* Frame rate: ~60 Hz */
    LCD_WriteCmd(ILI9488_FRMCTR1);
    LCD_WriteData8(0xA0U);

    /* Display inversion control: 2-dot inversion */
    LCD_WriteCmd(ILI9488_INVTR);
    LCD_WriteData8(0x02U);

    /* Display function control: GS=0 SS=0 (MADCTL owns orientation),
     * NL = 480 lines ((0x3B + 1) × 8 = 480) */
    LCD_WriteCmd(ILI9488_DFUNCTR);
    LCD_WriteData8(0x02U); LCD_WriteData8(0x02U); LCD_WriteData8(0x3BU);

    /* Entry mode set */
    LCD_WriteCmd(ILI9488_ETMOD);
    LCD_WriteData8(0xC6U);

    /* Set image function: 24-bit data bus disabled (DB_EN=0) */
    LCD_WriteCmd(ILI9488_SETIMAGE);
    LCD_WriteData8(0x00U);

    /* Adjust control 3: vendor-recommended value (harmless on DBI) */
    LCD_WriteCmd(ILI9488_ADJCTL3);
    LCD_WriteData8(0xA9U); LCD_WriteData8(0x51U);
    LCD_WriteData8(0x2CU); LCD_WriteData8(0x82U);

    /* Inversion per module type (hw_config.py → HW_LCD_INVERT) */
#if HW_LCD_INVERT
    LCD_WriteCmd(ILI9488_INVON);
#else
    LCD_WriteCmd(ILI9488_INVOFF);
#endif

    /* Sleep out — 120 ms before the next command */
    LCD_WriteCmd(ILI9488_SLPOUT);
    HAL_Delay(120U);

    /* Normal display mode on */
    LCD_WriteCmd(ILI9488_NORON);
    HAL_Delay(10U);

    /* Display on */
    LCD_WriteCmd(ILI9488_DISPON);
    HAL_Delay(25U);
}
#endif /* LCD_PANEL_ILI9488 */

#if (LCD_CONTROLLER != LCD_PANEL_ST7796) && (LCD_CONTROLLER != LCD_PANEL_ILI9488)
#  error "lcd_bus_fmc.c: no init sequence for this LCD_CONTROLLER (HW_LCD_PANEL)"
#endif

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
