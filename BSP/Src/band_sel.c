/* USER CODE BEGIN Header */
/**
  * @file band_sel.c
  * @brief Band quick-select overlay — 4×3 grid over SPEC+WF zone
  */
/* USER CODE END Header */

#include "band_sel.h"
#include "bpf_lpf.h"
#include "sdr_ui.h"
#include "lcd_render.h"
#include "lcd_bus_fmc.h"

/* ── Layout constants ─────────────────────────────────────────────────────── */
#define BS_MARGIN    8U
#define BS_COLS      4U
#define BS_ROWS_G    3U                                 /* 4×3 = 12 ≥ 10 bands */
#define BS_W         ((uint16_t)(LCD_W - 2U * BS_MARGIN))
#define BS_COL_W     ((uint16_t)(BS_W / BS_COLS))
#define BS_TITLE_H   16U
#define BS_ITEM_H    26U                                /* cell height          */

/* ── Colour palette ───────────────────────────────────────────────────────── */
#define BS_TITLE_BG  0x10A2U
#define BS_TITLE_FG  0xFFFFU
#define BS_HINT_FG   0x5ACBU
#define BS_BG        0x0843U
#define BS_EMPTY_BG  0x0421U   /* slightly darker than normal cell              */
#define BS_SEL_BG    0xF800U   /* red cursor                                    */
#define BS_ACT_BG    0x0360U   /* dark green — currently active band            */
#define BS_NAME_FG   0x3FE0U   /* yellow-green                                  */
#define BS_NAME_SEL  0xFFFFU
#define BS_NAME_ACT  0x07E0U

/* ── Module state ─────────────────────────────────────────────────────────── */
static bool    s_open   = false;
static uint8_t s_cursor = 0U;
static uint8_t s_active = 0U;

#define LN LCD_GetLineBuf()

static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

/* ── Helpers ──────────────────────────────────────────────────────────────── */

/* Band name char count (avoids string.h in inner loop) */
static inline uint16_t band_name_chars(uint8_t idx)
{
  return (idx == 0U) ? 4U : 3U;
}

/* ════ Public API ════════════════════════════════════════════════════════════ */

void BandSel_Open(uint8_t current_band, uint8_t active_band)
{
  s_cursor = current_band < BAND_COUNT ? current_band : 0U;
  s_active = active_band  < BAND_COUNT ? active_band  : 0U;
  s_open   = true;
}

void BandSel_Close(void) { s_open = false; }

bool BandSel_IsOpen(void) { return s_open; }

void BandSel_CursorUp(void)
{
  s_cursor = (s_cursor > 0U) ? (uint8_t)(s_cursor - 1U) : (uint8_t)(BAND_COUNT - 1U);
}

void BandSel_CursorDown(void)
{
  s_cursor = (uint8_t)((s_cursor + 1U) % BAND_COUNT);
}

uint8_t BandSel_Cursor(void) { return s_cursor; }

void BandSel_Render(void)
{
  uint16_t panel_y   = (uint16_t)ZONE_SPEC_Y;

  /* Pre-compute vertical text start inside a cell.
   * Each cell has a 1px gap at top and bottom (bg shows through as separator).
   * Text is vertically centred within the inner area (BS_ITEM_H - 2px). */
  uint16_t eff_cell_h = (uint16_t)(BS_ITEM_H - 2U);
  uint16_t txt_vstart = 1U + (eff_cell_h - (uint16_t)Font8x10.height) / 2U;
  uint16_t txt_vend   = txt_vstart + (uint16_t)Font8x10.height;

  /* ── Title bar ─────────────────────────────────────────────────────────── */
  for (uint16_t fr = 0U; fr < BS_TITLE_H; fr++) {
    uint16_t *ln = LN;
    bool top = (fr == 0U);
    bool bot = (fr == (uint16_t)BS_TITLE_H - 1U);

    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, (uint16_t)BS_MARGIN, BS_W,
                 (top || bot) ? BS_TITLE_BG : BS_TITLE_BG);

    if (!top && !bot) {
      ln[(uint16_t)BS_MARGIN]               = sw16(0x2965U);
      ln[(uint16_t)(BS_MARGIN + BS_W - 1U)] = sw16(0x2965U);
    }

    if (!top && !bot && fr >= 4U && fr < 4U + (uint16_t)Font5x8.height) {
      uint16_t tr = fr - 4U;
      LCD_LineStr(ln, (uint16_t)(BS_MARGIN + 4U), tr,
                  "SELECT BAND", &Font5x8, BS_TITLE_FG, BS_TITLE_BG);
#if LCD_W >= 400
      LCD_LineStr(ln, (uint16_t)(BS_MARGIN + BS_W - 72U), tr,
                  "F3=OK  F4=ESC", &Font5x8, BS_HINT_FG, BS_TITLE_BG);
#endif
    }

    LCD_PushWindow(0U, panel_y + fr, (uint16_t)(LCD_W - 1U), panel_y + fr, ln, LCD_W);
  }

  /* ── 4×3 band grid ─────────────────────────────────────────────────────── */
  uint16_t grid_y = panel_y + BS_TITLE_H;

  for (uint8_t row = 0U; row < BS_ROWS_G; row++) {
    uint16_t abs_y = grid_y + (uint16_t)row * BS_ITEM_H;

    for (uint16_t fr = 0U; fr < BS_ITEM_H; fr++) {
      uint16_t *ln = LN;
      LCD_LineFill(ln, 0U, LCD_W, UI_BG); /* 1px gaps between cells show as bg */

      for (uint8_t col = 0U; col < BS_COLS; col++) {
        uint8_t  band_idx = (uint8_t)(row * BS_COLS + col);
        uint16_t cell_x   = (uint16_t)(BS_MARGIN + (uint16_t)col * BS_COL_W);
        uint16_t inner_x  = cell_x + 1U;
        uint16_t inner_w  = BS_COL_W - 2U;
        bool     gap      = (fr == 0U || fr == (uint16_t)BS_ITEM_H - 1U);

        if (band_idx >= BAND_COUNT) {
          if (!gap) LCD_LineFill(ln, inner_x, inner_w, BS_EMPTY_BG);
          continue;
        }

        bool is_sel = (s_cursor == band_idx);
        bool is_act = (s_active == band_idx && !is_sel);
        uint16_t bg = is_sel ? BS_SEL_BG : is_act ? BS_ACT_BG : BS_BG;

        if (!gap) LCD_LineFill(ln, inner_x, inner_w, bg);

        /* Band name centred with Font8x10 */
        if (!gap && fr >= txt_vstart && fr < txt_vend) {
          uint16_t tr     = fr - txt_vstart;
          uint16_t nchars = band_name_chars(band_idx);
          uint16_t name_w = nchars * (uint16_t)Font8x10.width;
          uint16_t text_x = inner_x + (inner_w - name_w) / 2U;
          uint16_t clr    = is_sel ? BS_NAME_SEL : is_act ? BS_NAME_ACT : BS_NAME_FG;
          LCD_LineStrW(ln, text_x, tr, BPF_BandName(band_idx), &Font8x10, clr, bg);
        }
      }

      LCD_PushWindow(0U, abs_y + fr, (uint16_t)(LCD_W - 1U), abs_y + fr, ln, LCD_W);
    }
  }
}
