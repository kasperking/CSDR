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
 *  MADCTL=0x60 (MX|MV, RGB).  Switch to 0x68 if R/B appear swapped.
 */
/* USER CODE END Header */

#include "lcd_bus_fmc.h"
#include "stm32h7xx_hal.h"

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

void LCD_WriteDataBuffer(const uint16_t *buf, uint32_t count)
{
    while (count--) {
        *LCD_FMC_DATA_ADDR = (uint8_t)(*buf >> 8);
        *LCD_FMC_DATA_ADDR = (uint8_t)(*buf);
        buf++;
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

    /* Memory access: landscape 480×320, MX|MV, RGB order.
     * Change to ST7796_MADCTL_LANDSCAPE_BGR (0x68) if red and blue appear swapped. */
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

void LCD_Bus_Init(void)
{
    fmc_lcd_mpu_config();
    lcd_init_sequence();
}
