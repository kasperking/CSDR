/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st7789.h
  * @brief   LCD rendering primitives – font helpers, line buffer, SWAP16
  *
  *  Pure rendering layer.  No transport.  No UI layout.
  *  Transport: FMC 8080-mode via lcd_bus_fmc.h.
  *  UI lives in sdr_ui.h / sdr_ui.c.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ST7789_H
#define __ST7789_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Screen geometry ────────────────────────────────── */
#define LCD_W   320U
#define LCD_H   240U

/* ── Pixel byte-swap ────────────────────────────────── */
#define SWAP16(x)  (uint16_t)((((x) & 0x00FFU) << 8U) | (((x) >> 8U) & 0x00FFU))

/* ── Font ───────────────────────────────────────────── */
typedef struct {
  const uint8_t *data;
  uint8_t        width;
  uint8_t        height;
} Font_t;

extern const Font_t Font6x8;

/* ── Line-buffer render helpers ─────────────────────── */
void LCD_LineFill(uint16_t *ln, uint16_t x0, uint16_t w, uint16_t color);
void LCD_LineChar(uint16_t *ln, uint16_t x, uint16_t frow,
                  char c, const Font_t *f, uint16_t fg, uint16_t bg);
void LCD_LineStr(uint16_t *ln, uint16_t x, uint16_t frow,
                 const char *s, const Font_t *f, uint16_t fg, uint16_t bg);
void LCD_LineRect(uint16_t *ln, uint16_t x0, uint16_t w,
                  uint16_t row, uint16_t total_h, uint16_t border,
                  uint16_t fill, uint16_t border_color);

/* ── Line buffer (320 px, DMA_SRAM) ─────────────────── */
uint16_t *ST7789_GetLineBuf(void);

#ifdef __cplusplus
}
#endif
#endif /* __ST7789_H */
