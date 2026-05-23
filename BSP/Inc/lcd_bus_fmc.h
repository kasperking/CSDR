/* USER CODE BEGIN Header */
/**
 * @file  lcd_bus_fmc.h
 * @brief FMC 8080-mode LCD bus driver for STM32H750 – ST7796S 480×320.
 *
 *  FMC Bank1 NE1 memory-mapped access.  No SPI, no DMA — pure FMC.
 *  A16 (PD11) drives LCD RS/DC:
 *    write to LCD_FMC_CMD_ADDR  (0x60000000) → A16=0 → command
 *    write to LCD_FMC_DATA_ADDR (0x60010000) → A16=1 → data/pixel
 *
 *  Address mapping detail (8-bit mode):
 *    FMC Bank1 NE1 base = 0x60000000
 *    A16 = CPU address bit 16 = offset 0x10000
 *    CMD  = base + 0x00000 = 0x60000000  (A16=0 → RS/DC LOW  → command)
 *    DATA = base + 0x10000 = 0x60010000  (A16=1 → RS/DC HIGH → data)
 *    Any byte write to CMD_ADDR: FMC pulses NWE, keeps A16 LOW.
 *    Any byte write to DATA_ADDR: FMC pulses NWE, keeps A16 HIGH.
 *
 *  Bus width: 8-bit.  Each 16-bit pixel = 2 consecutive byte writes, MSB first.
 *
 *  IMPORTANT: LCD_Bus_Init() configures MPU Region 1 (0x60000000–0x6001FFFF)
 *  as Strongly-Ordered (TEX=0,C=0,B=0).  D-Cache must not buffer FMC writes.
 */
/* USER CODE END Header */

#ifndef LCD_BUS_FMC_H
#define LCD_BUS_FMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ── Bus width ───────────────────────────────────────────────────────────────
 * Only 8-bit implemented.  Reserve define for future 16-bit migration.
 */
#define LCD_BUS_WIDTH   8

/* ── FMC Bank1 NE1 address map ───────────────────────────────────────────────
 * CMD  → 0x60000000  (A16=0)
 * DATA → 0x60010000  (A16=1, offset = 1<<16)
 */
#define LCD_FMC_CMD_ADDR   ((volatile uint8_t *)0x60000000UL)
#define LCD_FMC_DATA_ADDR  ((volatile uint8_t *)0x60010000UL)

/* ── Screen geometry (ST7796S landscape 480×320) ─────────────────────────── */
#ifndef LCD_W
#define LCD_W   480U
#endif
#ifndef LCD_H
#define LCD_H   320U
#endif

/* ── ST7796S command set (subset for bring-up) ───────────────────────────── */
#define ST7796_SWRESET    0x01U   /* Software reset                          */
#define ST7796_SLPOUT     0x11U   /* Sleep out                               */
#define ST7796_NORON      0x13U   /* Normal display mode on                  */
#define ST7796_INVOFF     0x20U   /* Display inversion off                   */
#define ST7796_INVON      0x21U   /* Display inversion on                    */
#define ST7796_DISPON     0x29U   /* Display on                              */
#define ST7796_CASET      0x2AU   /* Column address set                      */
#define ST7796_RASET      0x2BU   /* Row address set                         */
#define ST7796_RAMWR      0x2CU   /* Memory write                            */
#define ST7796_MADCTL     0x36U   /* Memory data access control              */
#define ST7796_COLMOD     0x3AU   /* Interface pixel format                  */

/* ── MADCTL landscape configuration ─────────────────────────────────────────
 * ST7796S native resolution: 320 columns × 480 rows (portrait).
 *
 * MADCTL bit layout:  MY | MX | MV | ML | BGR | MH | 0 | 0
 *
 * This panel requires MY|MX|MV|BGR = 0xE8:
 *   MY + MX  correct glass orientation for this module; 0x60/0x68 (MX|MV only)
 *            produces a mirrored/flipped image on the physical panel
 *   MV       landscape scan (swap X↔Y addressing, 480×320)
 *   BGR      panel color filter is BGR order — bits[15:11] drive the B sub-pixel,
 *            bits[4:0] drive R.  Software colors must encode R and B swapped
 *            relative to their visual intent, or use the SWAP16 framebuffer
 *            convention (see LCD_PushWindow / DMA header comments).
 */
#define ST7796_MADCTL_LANDSCAPE  0xE8U   /* MY|MX|MV|BGR — confirmed for this panel */

/* COLMOD: 16-bit/pixel (RGB565) for both DPI and DBI */
#define ST7796_COLMOD_16BIT          0x55U

/* ── API ─────────────────────────────────────────────────────────────────── */

void LCD_Bus_Init(void);

/* Raw bus primitives */
void LCD_WriteCmd(uint8_t cmd);
void LCD_WriteData8(uint8_t data);

/* 16-bit pixel: two 8-bit writes, MSB first */
static inline void LCD_WriteData16(uint16_t data)
{
    *LCD_FMC_DATA_ADDR = (uint8_t)(data >> 8);
    *LCD_FMC_DATA_ADDR = (uint8_t)(data);
}

/* Core LCD operations — colors are raw RGB565 (no byte-swap) */
void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_WritePixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_FillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void LCD_Clear(uint16_t color);

/* Raw RGB565 buffer write — caller owns SetWindow/RAMWR context */
void LCD_WriteDataBuffer(const uint16_t *buf, uint32_t count);

/* ── UI push helpers ─────────────────────────────────────────────────────────
 * Zone buffers from sdr_ui.c use the SWAP16 byte-swap convention inherited
 * from the SPI-DMA era (pixel 0xABCD stored as 0xCDAB so DMA byte order is
 * correct).  LCD_PushWindow handles that convention internally: it writes the
 * low byte of each uint16_t first (= the original MSB after SWAP16), giving
 * the ST7796 the correct big-endian pixel stream.
 *
 *  LCD_PushWindow – SetWindow then burst-write a SWAP16-encoded pixel buffer.
 *    x0,y0: top-left; x1,y1: bottom-right (inclusive, screen coordinates).
 *    buf  : pointer to first pixel; npix = (x1-x0+1)*(y1-y0+1).
 */
void LCD_PushWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                    const uint16_t *buf_swap16, uint32_t npix);

#ifdef __cplusplus
}
#endif

#endif /* LCD_BUS_FMC_H */
