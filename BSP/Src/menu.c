/* USER CODE BEGIN Header */
/**
  * @file menu.c
  * @brief SDR Menu System – scanline renderer overlay
  */
/* USER CODE END Header */

#include "menu.h"
#include "sdr_ui.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ── Private: SWAP16 (must be before any use) ── */
static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

/* ── Line buffer access ── */
/* LCD_GetLineBuf() returns the 320-pixel DMA line buffer from lcd_render.c */
#define LN  LCD_GetLineBuf()

/* ── Edit color (cyan = UI_FREQ_MHZ) ── */
#define MENU_EDIT_COLOR  UI_FREQ_MHZ

/* USER CODE BEGIN PV */
Menu_Handle_t g_menu;

static int32_t _agc_val, _nb_val, _nr_val, _rit_val;
static int32_t _vol_val, _mic_val, _sq_val, _step_val, _att_val;
static int32_t _band_val, _mode_val, _bl_val, _usb_val;
static int32_t _zoom_val;

static const char *agc_strs[]  = { "SLOW", "FAST" };
static const char *onoff_strs[]= { "OFF",  "ON"   };
static const char *step_strs[] = { "1Hz","10Hz","100Hz","1KHz","10KHz","100KHz" };
static const char *band_strs[] = { "160m","80m","60m","40m","30m",
                                    "20m","17m","15m","12m","10m","6m" };
static const char *mode_strs[] = { "AM","FM","USB","LSB","CW" };
static const char *usb_strs[]  = { "Off","CAT","Audio" };
static const char *zoom_strs[] = { "+/-24k","+/-12k","+/-6k","+/-3k" };

/* MenuApplyFn defined in menu.h */
static MenuApplyFn s_apply_cb = NULL;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

/* ── Push one scanline via FMC ── */
static void push_ln(uint16_t y)
{
  LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, LN, LCD_W);
}

/* ── Render one menu item (MENU_ITEM_H = 16 scanlines) ── */
static void render_item(Menu_Handle_t *m, uint8_t idx, uint16_t abs_y)
{
  MenuItem_t *it  = &m->items[idx];
  bool sel        = (idx == m->cursor);
  uint16_t bg     = sel ? MENU_SEL_COLOR : MENU_BG_COLOR;

  /* Build value string */
  char val[24];
  if (it->type == MENU_TYPE_ACTION) {
    snprintf(val, sizeof(val), ">> RUN");
  } else if (it->type == MENU_TYPE_INT) {
    snprintf(val, sizeof(val), "%ld", (long)*it->value_ptr);
  } else {
    int32_t vi = *it->value_ptr;
    if (vi < 0) vi = 0;
    if (vi >= (int32_t)it->enum_count) vi = (int32_t)it->enum_count - 1;
    snprintf(val, sizeof(val), "%s", it->enum_strs[vi]);
  }

  for (uint16_t fr = 0U; fr < (uint16_t)MENU_ITEM_H; fr++) {
    uint16_t *ln = LN;
    bool top = (fr == 0U);
    bool bot = (fr == (uint16_t)MENU_ITEM_H - 1U);

    /* Fill */
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, MENU_X, MENU_W, (top || bot) ? MENU_BORDER_COLOR : bg);

    /* Side borders */
    if (!top && !bot) {
      ln[MENU_X]                = sw16(MENU_BORDER_COLOR);
      ln[MENU_X + MENU_W - 1U] = sw16(MENU_BORDER_COLOR);
    }

    /* Text rows: fr 4..11 (Font6x8 height = 8) */
    if (!top && !bot && fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      uint16_t row = fr - 4U;
      /* Label */
      LCD_LineStr(ln, (uint16_t)(MENU_X + 4U), row,
                  it->label, &Font6x8, MENU_LBL_COLOR, bg);
      /* Value */
      uint16_t vcol = (sel && m->editing) ? MENU_EDIT_COLOR : MENU_VAL_COLOR;
      LCD_LineStr(ln, (uint16_t)(MENU_X + MENU_W - 80U), row,
                  val, &Font6x8, vcol, bg);
    }

    push_ln((uint16_t)(abs_y + fr));
  }
}

/* USER CODE END 0 */

/* ════ Menu_Init ════ */
void Menu_Init(Menu_Handle_t *m)
{
  /* USER CODE BEGIN Menu_Init_0 */
  memset(m, 0, sizeof(*m));
  m->item_count = MENU_ITEM_COUNT;
  _bl_val = 80;

  m->items[0]  = (MenuItem_t){ "AGC",      MENU_TYPE_ENUM, 0,0,0, &_agc_val,  agc_strs,  2U,  NULL };
  m->items[1]  = (MenuItem_t){ "NB",       MENU_TYPE_ENUM, 0,0,0, &_nb_val,   onoff_strs,2U,  NULL };
  m->items[2]  = (MenuItem_t){ "NR",       MENU_TYPE_ENUM, 0,0,0, &_nr_val,   onoff_strs,2U,  NULL };
  m->items[3]  = (MenuItem_t){ "RIT (Hz)", MENU_TYPE_INT, -999, 999, 1, &_rit_val, NULL, 0, NULL };
  m->items[4]  = (MenuItem_t){ "Volume",   MENU_TYPE_INT, 0, 100, 5, &_vol_val, NULL, 0, NULL };
  m->items[5]  = (MenuItem_t){ "Mic Gain", MENU_TYPE_INT, 0, 100, 1, &_mic_val, NULL, 0, NULL };
  m->items[6]  = (MenuItem_t){ "Squelch",  MENU_TYPE_INT, 0, 100, 1, &_sq_val,  NULL, 0, NULL };
  m->items[7]  = (MenuItem_t){ "Step",     MENU_TYPE_ENUM, 0,0,0, &_step_val, step_strs, 6U,  NULL };
  m->items[8]  = (MenuItem_t){ "ATT (dB)", MENU_TYPE_INT, 0,  31, 1, &_att_val, NULL, 0, NULL };
  m->items[9]  = (MenuItem_t){ "Band",     MENU_TYPE_ENUM, 0,0,0, &_band_val, band_strs, 11U, NULL };
  m->items[10] = (MenuItem_t){ "Mode",     MENU_TYPE_ENUM, 0,0,0, &_mode_val, mode_strs, 5U,  NULL };
  m->items[11] = (MenuItem_t){ "Backlight",MENU_TYPE_INT, 0, 100,10, &_bl_val,  NULL, 0, NULL };
  m->items[12] = (MenuItem_t){ "USB",      MENU_TYPE_ENUM, 0,0,0, &_usb_val,  usb_strs,  3U,  NULL };
  m->items[13] = (MenuItem_t){ "Span",     MENU_TYPE_ENUM, 0,0,0, &_zoom_val, zoom_strs, 4U,  NULL };
  m->items[14] = (MenuItem_t){ "Diagnostics",  MENU_TYPE_ACTION, 0,0,0, NULL, NULL, 0U, NULL };
  m->items[15] = (MenuItem_t){ "Calibration",  MENU_TYPE_ACTION, 0,0,0, NULL, NULL, 0U, NULL };
  m->items[16] = (MenuItem_t){ "SWR Scan",     MENU_TYPE_ACTION, 0,0,0, NULL, NULL, 0U, NULL };
  /* USER CODE END Menu_Init_0 */
}

/* ════ Navigation ════ */
void Menu_Toggle(Menu_Handle_t *m)
{ m->open = !m->open; m->editing = false; if (m->open) Menu_Render(m); }

static void clamp_scroll(Menu_Handle_t *m)
{
  if (m->cursor < m->scroll) m->scroll = m->cursor;
  if (m->cursor >= m->scroll + MENU_VISIBLE_ROWS)
    m->scroll = (uint8_t)(m->cursor - MENU_VISIBLE_ROWS + 1U);
}

static void change_val(Menu_Handle_t *m, int32_t d)
{
  MenuItem_t *it = &m->items[m->cursor];
  int32_t v = *it->value_ptr;
  if (it->type == MENU_TYPE_INT) {
    v += d * it->step;
    if (v < it->min) v = it->min;
    if (v > it->max) v = it->max;
  } else {
    v += d;
    if (v < 0) v = (int32_t)it->enum_count - 1;
    if (v >= (int32_t)it->enum_count) v = 0;
  }
  *it->value_ptr = v;
  if (it->on_change) it->on_change();
  if (s_apply_cb) s_apply_cb();
}

void Menu_Up(Menu_Handle_t *m)
{
  if (!m->open) return;
  if (m->editing) change_val(m, +1);
  else { if (m->cursor > 0U) m->cursor--; clamp_scroll(m); }
  Menu_Render(m);
}

void Menu_Down(Menu_Handle_t *m)
{
  if (!m->open) return;
  if (m->editing) change_val(m, -1);
  else { if (m->cursor < m->item_count - 1U) m->cursor++; clamp_scroll(m); }
  Menu_Render(m);
}

void Menu_Select(Menu_Handle_t *m)
{
  if (!m->open) return;
  m->editing = !m->editing;
  if (!m->editing && s_apply_cb) s_apply_cb();
  Menu_Render(m);
}

void Menu_Confirm(Menu_Handle_t *m)
{
  if (!m->open) return;
  m->editing = false;
  if (s_apply_cb) s_apply_cb();
  Menu_Render(m);
}

void Menu_Back(Menu_Handle_t *m)
{
  if (!m->open) return;
  if (m->editing) { m->editing = false; Menu_Render(m); }
  else             { m->open = false; }
}

void Menu_EncoderEdit(Menu_Handle_t *m, int32_t delta)
{ if (m->open && m->editing) { change_val(m, delta); Menu_Render(m); } }

/* ════ Menu_Render ════ */
void Menu_Render(Menu_Handle_t *m)
{
  /* USER CODE BEGIN Menu_Render_0 */
  if (!m->open) return;

  uint16_t y = (uint16_t)MENU_Y;

  /* Header (16px) */
  for (uint16_t fr = 0U; fr < 16U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, MENU_X, MENU_W,
                 (fr == 0U || fr == 15U) ? MENU_BORDER_COLOR : MENU_HEADER_BG);
    ln[MENU_X]                = sw16(MENU_BORDER_COLOR);
    ln[MENU_X + MENU_W - 1U] = sw16(MENU_BORDER_COLOR);

    if (fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      const char *t = m->editing ? " [ EDIT ] " : " -= MENU =- ";
      uint16_t tx = (uint16_t)(MENU_X + (MENU_W - (uint16_t)strlen(t) * Font6x8.width) / 2U);
      LCD_LineStr(ln, tx, fr - 4U, t, &Font6x8, 0xFFFFU, MENU_HEADER_BG);
    }
    push_ln(y++);
  }

  /* Items */
  for (uint8_t r = 0U; r < MENU_VISIBLE_ROWS; r++) {
    uint8_t idx = m->scroll + r;
    if (idx >= m->item_count) break;
    render_item(m, idx, y);
    y += (uint16_t)MENU_ITEM_H;
  }

  /* Hint row */
  {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    char cnt[10];
    snprintf(cnt, sizeof(cnt), "%d/%d", m->cursor + 1, m->item_count);
    LCD_LineStr(ln, (uint16_t)(MENU_X + MENU_W - 42U), 0U,
                cnt, &Font6x8, MENU_LBL_COLOR, UI_BG);
    LCD_LineStr(ln, (uint16_t)(MENU_X + 4U), 0U,
                "F1=UP F2=DN ENC=EDIT F4=EXIT",
                &Font6x8, MENU_LBL_COLOR, UI_BG);
    for (uint8_t fr = 0U; fr < (uint8_t)Font6x8.height; fr++)
      push_ln(y++);
  }
  /* USER CODE END Menu_Render_0 */
}

/* ════ Load / Save SDR state ════ */
void Menu_LoadFromSDR(Menu_Handle_t *m,
                       bool agc_fast, bool nb, bool nr, int16_t rit,
                       uint8_t vol, uint8_t mic_gain, uint8_t sq, uint32_t step,
                       uint8_t att, uint8_t band, uint8_t mode,
                       uint8_t usb_mode, uint8_t zoom, MenuApplyFn apply_cb)
{
  /* USER CODE BEGIN Menu_LoadFromSDR_0 */
  (void)m;
  static const uint32_t sv[6] = {1,10,100,1000,10000,100000};
  _agc_val  = agc_fast ? 1 : 0;
  _nb_val   = nb  ? 1 : 0;
  _nr_val   = nr  ? 1 : 0;
  _rit_val  = (int32_t)rit;
  _vol_val  = (int32_t)vol;
  _mic_val  = (int32_t)mic_gain;
  _sq_val   = (int32_t)sq;
  _step_val = 2;
  for (uint8_t i = 0; i < 6U; i++) if (step == sv[i]) { _step_val = (int32_t)i; break; }
  _att_val  = (int32_t)att;
  _band_val = (int32_t)band;
  _mode_val = (int32_t)mode;
  _usb_val  = (int32_t)usb_mode;
  _zoom_val = (int32_t)zoom;
  s_apply_cb = apply_cb;
  /* USER CODE END Menu_LoadFromSDR_0 */
}

void Menu_SaveToSDR(Menu_Handle_t *m,
                     bool *agc_fast, bool *nb, bool *nr, int16_t *rit,
                     uint8_t *vol, uint8_t *mic_gain, uint8_t *sq, uint32_t *step,
                     uint8_t *att, uint8_t *band, uint8_t *mode,
                     uint8_t *usb_mode, uint8_t *zoom)
{
  /* USER CODE BEGIN Menu_SaveToSDR_0 */
  (void)m;
  static const uint32_t sv[6] = {1,10,100,1000,10000,100000};
  *agc_fast = (_agc_val != 0);
  *nb       = (_nb_val  != 0);
  *nr       = (_nr_val  != 0);
  *rit      = (int16_t)_rit_val;
  *vol      = (uint8_t)_vol_val;
  *mic_gain = (uint8_t)_mic_val;
  *sq       = (uint8_t)_sq_val;
  *step     = sv[(_step_val >= 0 && _step_val < 6) ? _step_val : 2];
  *att      = (uint8_t)_att_val;
  *band     = (uint8_t)_band_val;
  *mode     = (uint8_t)_mode_val;
  *usb_mode = (uint8_t)_usb_val;
  *zoom     = (uint8_t)(_zoom_val >= 0 && _zoom_val < 4 ? _zoom_val : 0);
  /* USER CODE END Menu_SaveToSDR_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
