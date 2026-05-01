/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st7789.h
  * @brief   ST7789VW – SPI/DMA driver, scanline primitives, font helpers
  *
  *  Pure hardware layer: init, window, push, font rendering.
  *  No UI layout, no colour palette, no application state.
  *  UI lives in sdr_ui.h / sdr_ui.c.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ST7789_H
#define __ST7789_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── ST7789 Commands ────────────────────────────────── */
#define ST7789_SWRESET    0x01U
#define ST7789_SLPOUT     0x11U
#define ST7789_NORON      0x13U
#define ST7789_INVON      0x21U
#define ST7789_DISPON     0x29U
#define ST7789_CASET      0x2AU
#define ST7789_RASET      0x2BU
#define ST7789_RAMWR      0x2CU
#define ST7789_MADCTL     0x36U
#define ST7789_COLMOD     0x3AU
#define ST7789_PORCTRL    0xB2U
#define ST7789_GCTRL      0xB7U
#define ST7789_VCOMS      0xBBU
#define ST7789_LCMCTRL    0xC0U
#define ST7789_VDVVRHEN   0xC2U
#define ST7789_VRHS       0xC3U
#define ST7789_VDVS       0xC4U
#define ST7789_FRCTRL2    0xC6U
#define ST7789_PWCTRL1    0xD0U
#define ST7789_PVGAMCTRL  0xE0U
#define ST7789_NVGAMCTRL  0xE1U

/* MADCTL: Landscape 320×240 (MX|MV|BGR) */
#define ST7789_MADCTL_LANDSCAPE   0x68U

/* ── Screen geometry ────────────────────────────────── */
#define LCD_W   320U
#define LCD_H   240U

/* ── Pixel byte-swap for SPI (big-endian on wire) ───── */
#define SWAP16(x)  (uint16_t)((((x) & 0x00FFU) << 8U) | (((x) >> 8U) & 0x00FFU))

/* ── Font ───────────────────────────────────────────── */
typedef struct {
  const uint8_t *data;
  uint8_t        width;
  uint8_t        height;
} Font_t;

extern const Font_t Font6x8;

/* ── LCD handle ─────────────────────────────────────── */
typedef struct {
  SPI_HandleTypeDef *hspi;
  DMA_HandleTypeDef *hdma_tx;
  GPIO_TypeDef  *cs_port;  uint16_t cs_pin;
  GPIO_TypeDef  *dc_port;  uint16_t dc_pin;
  GPIO_TypeDef  *rst_port; uint16_t rst_pin;
  GPIO_TypeDef  *bl_port;  uint16_t bl_pin;
  uint16_t       width;
  uint16_t       height;
  volatile bool  dma_busy;
  volatile bool  cs_held;
} ST7789_Handle_t;

/* ── Core API ───────────────────────────────────────── */
void ST7789_Init(ST7789_Handle_t *lcd);
void ST7789_SetBacklight(ST7789_Handle_t *lcd, bool on);
void ST7789_DMA_TxCpltCallback(ST7789_Handle_t *lcd);
void ST7789_SetWindow(ST7789_Handle_t *lcd,
                      uint16_t x0, uint16_t y0,
                      uint16_t x1, uint16_t y1);
void ST7789_FillScreen(ST7789_Handle_t *lcd, uint16_t color);

/* ── Push primitives ────────────────────────────────── */

/* Full-width scanline (0..LCD_W-1), fire-and-forget DMA.
 * Caller must dma_wait_pub() + cs_high_pub() before next command. */
void ST7789_PushScanline(ST7789_Handle_t *lcd,
                          uint16_t y, const uint16_t *line);

/* Full-width block (0..LCD_W-1, y0..y1), blocking. */
void ST7789_PushBlock(ST7789_Handle_t *lcd,
                      uint16_t y0, uint16_t y1, const uint16_t *buf);

/* Arbitrary sub-rectangle, blocking.
 * buf must hold (x1-x0+1)*(y1-y0+1) pixels in row-major order. */
void ST7789_PushWindow(ST7789_Handle_t *lcd,
                       uint16_t x0, uint16_t x1,
                       uint16_t y0, uint16_t y1,
                       const uint16_t *buf);

/* ── Line-buffer render helpers ─────────────────────── */
void LCD_LineFill(uint16_t *ln, uint16_t x0, uint16_t w, uint16_t color);
void LCD_LineChar(uint16_t *ln, uint16_t x, uint16_t frow,
                  char c, const Font_t *f, uint16_t fg, uint16_t bg);
void LCD_LineStr(uint16_t *ln, uint16_t x, uint16_t frow,
                 const char *s, const Font_t *f, uint16_t fg, uint16_t bg);
void LCD_LineRect(uint16_t *ln, uint16_t x0, uint16_t w,
                  uint16_t row, uint16_t total_h, uint16_t border,
                  uint16_t fill, uint16_t border_color);

/* ── Helpers exposed to UI layer ────────────────────── */
void      dma_wait_pub(ST7789_Handle_t *lcd);
void      cs_high_pub(ST7789_Handle_t *lcd);
uint16_t *ST7789_GetLineBuf(void);

#ifdef __cplusplus
}
#endif
#endif /* __ST7789_H */
