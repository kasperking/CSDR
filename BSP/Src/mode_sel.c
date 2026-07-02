/* USER CODE BEGIN Header */
/**
  * @file mode_sel.c
  * @brief Mode quick-select overlay — 4×2 grid over SPEC+WF zone
  */
/* USER CODE END Header */

#include "mode_sel.h"
#include "csdr_app.h"
#include "sdr_ui.h"
#include "lcd_render.h"
#include "lcd_bus_fmc.h"

/* ── Layout constants ─────────────────────────────────────────────────────── */
#define MS_MARGIN   8U
#define MS_COLS     4U
#define MS_ROWS     2U
#define MS_W        ((uint16_t)(LCD_W - 2U * MS_MARGIN))
#define MS_COL_W    ((uint16_t)(MS_W / MS_COLS))
#define MS_TITLE_H  16U
#define MS_ITEM_H   26U

/* ── Colour palette ───────────────────────────────────────────────────────── */
#define MS_TITLE_BG  0x10A2U
#define MS_TITLE_FG  0xFFFFU
#define MS_HINT_FG   0x5ACBU
#define MS_BG        0x0843U
#define MS_SEL_BG    0xF800U   /* red cursor                                    */
#define MS_ACT_BG    0x0360U   /* dark green — currently active mode            */

/* ── Mode metadata ────────────────────────────────────────────────────────── */
static const char *const s_mode_names[MODE_COUNT] = {
  "AM", "FM", "USB", "LSB", "CW", "DIGU", "DIGL", "FDV"
};

static const uint16_t s_mode_colors[MODE_COUNT] = {
  UI_MODE_AM, UI_MODE_FM, UI_MODE_USB, UI_MODE_LSB,
  UI_MODE_CW, UI_MODE_DIGU, UI_MODE_DIGL, UI_MODE_FREEDV
};

/* ── Module state ─────────────────────────────────────────────────────────── */
static bool    s_open   = false;
static uint8_t s_cursor = 0U;
static uint8_t s_active = 0U;

#define LN LCD_GetLineBuf()

static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

/* ════ Public API ════════════════════════════════════════════════════════════ */

void ModeSel_Open(uint8_t current_mode, uint8_t active_mode)
{
  s_cursor = current_mode < MODE_COUNT ? current_mode : 0U;
  s_active = active_mode  < MODE_COUNT ? active_mode  : 0U;
  s_open   = true;
}

void ModeSel_Close(void) { s_open = false; }

bool ModeSel_IsOpen(void) { return s_open; }

void ModeSel_CursorUp(void)
{
  s_cursor = (s_cursor > 0U) ? (uint8_t)(s_cursor - 1U)
                              : (uint8_t)(MODE_COUNT - 1U);
}

void ModeSel_CursorDown(void)
{
  s_cursor = (uint8_t)((s_cursor + 1U) % MODE_COUNT);
}

uint8_t ModeSel_Cursor(void) { return s_cursor; }

void ModeSel_Render(void)
{
  uint16_t panel_y = (uint16_t)ZONE_SPEC_Y;

  uint16_t eff_cell_h = (uint16_t)(MS_ITEM_H - 2U);
  uint16_t txt_vstart = 1U + (eff_cell_h - (uint16_t)Font8x10.height) / 2U;
  uint16_t txt_vend   = txt_vstart + (uint16_t)Font8x10.height;

  /* ── Title bar ─────────────────────────────────────────────────────────── */
  for (uint16_t fr = 0U; fr < MS_TITLE_H; fr++) {
    uint16_t *ln = LN;
    bool top = (fr == 0U);
    bool bot = (fr == (uint16_t)MS_TITLE_H - 1U);

    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, (uint16_t)MS_MARGIN, MS_W, MS_TITLE_BG);

    if (!top && !bot) {
      ln[(uint16_t)MS_MARGIN]               = sw16(0x2965U);
      ln[(uint16_t)(MS_MARGIN + MS_W - 1U)] = sw16(0x2965U);
    }

    if (!top && !bot && fr >= 4U && fr < 4U + (uint16_t)Font5x8.height) {
      uint16_t tr = fr - 4U;
      LCD_LineStr(ln, (uint16_t)(MS_MARGIN + 4U), tr,
                  "SELECT MODE", &Font5x8, MS_TITLE_FG, MS_TITLE_BG);
#if LCD_W >= 400
      LCD_LineStr(ln, (uint16_t)(MS_MARGIN + MS_W - 72U), tr,
                  "F3=OK  F4=ESC", &Font5x8, MS_HINT_FG, MS_TITLE_BG);
#endif
    }

    LCD_PushWindow(0U, panel_y + fr, (uint16_t)(LCD_W - 1U), panel_y + fr, ln, LCD_W);
  }

  /* ── 4×2 mode grid ─────────────────────────────────────────────────────── */
  uint16_t grid_y = panel_y + MS_TITLE_H;

  for (uint8_t row = 0U; row < MS_ROWS; row++) {
    uint16_t abs_y = grid_y + (uint16_t)row * MS_ITEM_H;

    for (uint16_t fr = 0U; fr < MS_ITEM_H; fr++) {
      uint16_t *ln = LN;
      LCD_LineFill(ln, 0U, LCD_W, UI_BG);

      for (uint8_t col = 0U; col < MS_COLS; col++) {
        uint8_t  mode_idx = (uint8_t)(row * MS_COLS + col);
        uint16_t cell_x   = (uint16_t)(MS_MARGIN + (uint16_t)col * MS_COL_W);
        uint16_t inner_x  = cell_x + 1U;
        uint16_t inner_w  = MS_COL_W - 2U;
        bool     gap      = (fr == 0U || fr == (uint16_t)MS_ITEM_H - 1U);

        if (mode_idx >= MODE_COUNT) continue;

        bool is_sel = (s_cursor == mode_idx);
        bool is_act = (s_active == mode_idx && !is_sel);
        uint16_t bg = is_sel ? MS_SEL_BG : is_act ? MS_ACT_BG : MS_BG;

        if (!gap) LCD_LineFill(ln, inner_x, inner_w, bg);

        if (!gap && fr >= txt_vstart && fr < txt_vend) {
          uint16_t tr     = fr - txt_vstart;
          const char *nm  = s_mode_names[mode_idx];
          uint16_t nchars = 0U;
          for (const char *p = nm; *p; p++) nchars++;
          uint16_t name_w = nchars * (uint16_t)Font8x10.width;
          uint16_t text_x = inner_x + (inner_w - name_w) / 2U;
          uint16_t clr    = is_sel ? 0xFFFFU
                          : is_act ? 0x07E0U
                          : s_mode_colors[mode_idx];
          LCD_LineStrW(ln, text_x, tr, nm, &Font8x10, clr, bg);
        }
      }

      LCD_PushWindow(0U, abs_y + fr, (uint16_t)(LCD_W - 1U), abs_y + fr, ln, LCD_W);
    }
  }
}
