/* USER CODE BEGIN Header */
/**
 * @file  lcd_bus_fmc.h
 * @brief FMC 8080-mode LCD bus driver for STM32H750 – ST7796S/ST7789.
 *
 *  FMC Bank1 NE1 memory-mapped access.  No SPI — pure FMC.
 *  A16 (PD11) drives LCD RS/DC:
 *    write to LCD_FMC_CMD_ADDR  → A16=0 → command
 *    write to LCD_FMC_DATA_ADDR → A16=1 → data/pixel
 *
 *  Bus width is selected by hw_config.py (HW_FMC_8BIT / HW_FMC_16BIT):
 *
 *  8-bit mode:
 *    DATA = 0x60000000 + 0x10000  (A16 = CPU bit 16)
 *    lcd_bus_t = uint8_t.  Each RGB565 pixel = 2 byte-writes, MSB first.
 *    MPU Region 1 covers 128 KB (0x60000000–0x6001FFFF).
 *
 *  16-bit mode:
 *    DATA = 0x60000000 + 0x20000  (A16 = CPU bit 17; bus-width address shift)
 *    lcd_bus_t = uint16_t.  Each RGB565 pixel = 1 halfword-write.
 *    MPU Region 1 covers 256 KB (0x60000000–0x6003FFFF).
 *    LCD_WriteData8 casts to uint16_t (DB15-DB8 = 0x00 → don't-care for params).
 *    LCD_WriteData16 collapses to a single write.
 *    LCD_PushWindow un-does the SWAP16 buffer convention per pixel (bswap16).
 *    DMA pixel path falls back to CPU (see lcd_dma.c).
 *
 *  IMPORTANT: LCD_Bus_Init() configures MPU Region 1 as Strongly-Ordered
 *  (TEX=0,C=0,B=0).  D-Cache must not buffer FMC writes.
 */
/* USER CODE END Header */

#ifndef LCD_BUS_FMC_H
#define LCD_BUS_FMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "hw_config_active.h"

/* ── Bus width ───────────────────────────────────────────────────────────────
 * Derived from HW_FMC_8BIT / HW_FMC_16BIT in hw_config_active.h.
 *
 * 8-bit mode:  lcd_bus_t = uint8_t,  DATA offset = 0x10000 (A16 = CPU bit 16)
 * 16-bit mode: lcd_bus_t = uint16_t, DATA offset = 0x20000 (A16 = CPU bit 17,
 *              because each FMC address word is 2 bytes wide).
 *
 * lcd_bus_fmc.c casts writes to lcd_bus_t so both modes compile cleanly.
 * LCD_WriteData16 collapses to a single write in 16-bit mode (1 pixel per WR).
 * hw_config_active.h is included above so these macros are visible immediately. */
#if HW_FMC_16BIT
#  define LCD_BUS_WIDTH    16
   typedef volatile uint16_t lcd_bus_t;
#else
#  define LCD_BUS_WIDTH    8
   typedef volatile uint8_t lcd_bus_t;
#endif

/* ── FMC Bank1 NE1 address map ───────────────────────────────────────────────
 * CMD  → 0x60000000              (A16=0 → RS/DC LOW  → command)
 * DATA → 0x60000000 + DATA_OFFSET (A16=1 → RS/DC HIGH → data/pixel)
 *
 * HW_FMC_DATA_ADDR_OFFSET comes from hw_config_active.h:
 *   8-bit  → 0x10000  (offset = 1<<16)
 *   16-bit → 0x20000  (offset = 1<<17, due to bus-width address shift)
 */
#define LCD_FMC_CMD_ADDR   ((lcd_bus_t *)0x60000000UL)
#define LCD_FMC_DATA_ADDR  ((lcd_bus_t *)(0x60000000UL + HW_FMC_DATA_ADDR_OFFSET))

/* ── Screen geometry ──────────────────────────────────────────────────────── *
 * LCD_W and LCD_H are defined by lcd_panel_config.h.                         *
 * Fallback defaults retain the original ST7796 values so that files which    *
 * include lcd_bus_fmc.h without lcd_render.h still compile correctly.        */
#include "lcd_panel_config.h"

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

/* ── MADCTL landscape configuration (ST7796S) ───────────────────────────────
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
#define ST7796_COLMOD_16BIT      0x55U

/* ── ST7789V command set (portrait 240×320) ──────────────────────────────── *
 * Shared commands (CASET/RASET/RAMWR/MADCTL/COLMOD/SWRESET/SLPOUT/DISPON)   *
 * use the same opcodes as ST7796S — LCD_SetWindow works unchanged.           */
#define ST7789_SWRESET    0x01U
#define ST7789_SLPOUT     0x11U
#define ST7789_INVOFF     0x20U   /* Inversion off                            */
#define ST7789_INVON      0x21U   /* Inversion on — required for normal mode  */
#define ST7789_DISPON     0x29U
#define ST7789_CASET      0x2AU
#define ST7789_RASET      0x2BU
#define ST7789_RAMWR      0x2CU
#define ST7789_MADCTL     0x36U
#define ST7789_COLMOD     0x3AU

/* MADCTL portrait — no axis swap (MV=0), BGR color filter.
 * MY=0 MX=0 MV=0 ML=0 BGR=1 MH=0 → 0x08.
 * X-axis: 0..239 left→right.  Y-axis: 0..319 top→bottom. */
#define ST7789_MADCTL_PORTRAIT   0x08U   /* BGR — confirmed for this panel */

#define ST7789_COLMOD_16BIT      0x55U   /* RGB565 */

/* ── API ─────────────────────────────────────────────────────────────────── */

void LCD_Bus_Init(void);

/* Raw bus primitives */
void LCD_WriteCmd(uint8_t cmd);
void LCD_WriteData8(uint8_t data);

/* 16-bit pixel write.
 * 8-bit mode : two byte-writes, MSB first.
 * 16-bit mode: single halfword-write (one WR pulse = one pixel). */
static inline void LCD_WriteData16(uint16_t data)
{
#if HW_FMC_16BIT
    *LCD_FMC_DATA_ADDR = (lcd_bus_t)data;
#else
    *LCD_FMC_DATA_ADDR = (lcd_bus_t)(data >> 8);
    *LCD_FMC_DATA_ADDR = (lcd_bus_t)(data);
#endif
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
