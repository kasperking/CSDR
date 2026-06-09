/* USER CODE BEGIN Header */
/**
  * @file menu.c
  * @brief SDR Menu System – 2-level hierarchical scanline renderer
  */
/* USER CODE END Header */

#include "menu.h"
#include "sdr_ui.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ── Private: SWAP16 ── */
static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

/* ── Line buffer access ── */
#define LN  LCD_GetLineBuf()

/* ── Edit color (cyan = UI_FREQ_MHZ) ── */
#define MENU_EDIT_COLOR  UI_FREQ_MHZ

/* USER CODE BEGIN PV */
Menu_Handle_t g_menu;

static int32_t _agc_val, _nb_val, _nr_val, _rit_val;
static int32_t _vol_val, _mic_val, _digi_val, _sq_val, _step_val, _att_val;
static int32_t _band_val, _mode_val, _bl_val, _usb_val;
static int32_t _zoom_val, _alc_val, _rfpwr_val;
static uint8_t s_pa_watts = 0U;

static int32_t _tx_low_val, _tx_high_val;
static int32_t _cw_pitch_val, _cw_wpm_val, _cw_keyer_val;
static int32_t _cw_rev_val, _cw_paddle_rev_val;
static int32_t _cw_stvol_val, _cw_bkin_val, _cw_bkdly_val, _cw_filter_val;

static const char *agc_strs[]    = { "SLOW", "FAST" };
static const char *onoff_strs[]  = { "OFF",  "ON"   };
static const char *step_strs[]   = { "1Hz","10Hz","100Hz","1KHz","10KHz","100KHz" };
static const char *band_strs[]   = { "160m","80m","60m","40m","30m",
                                     "20m","17m","15m","12m","10m","6m" };
static const char *mode_strs[]   = { "AM","FM","USB","LSB","CW" };
static const char *usb_strs[]    = { "Off", "CAT", "CAT+Audio" };
static const char *zoom_strs[]   = { "+/-24k","+/-12k","+/-6k","+/-3k" };
static const char *keyer_strs[]  = { "STRAIGHT", "IAMBIC-A", "IAMBIC-B" };
static const char *bkin_strs[]   = { "OFF", "SEMI", "FULL" };

static MenuApplyFn s_apply_cb = NULL;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

/* ── Push one scanline via FMC ── */
static void push_ln(uint16_t y)
{
  LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, LN, LCD_W);
}

/* ── Build view: collect all items whose parent == current_group ── */
static void Menu_BuildView(Menu_Handle_t *m)
{
  m->view_count = 0U;
  for (uint8_t i = 0U; i < m->item_count; i++) {
    if (m->items[i].parent == m->current_group) {
      m->view[m->view_count++] = i;
    }
  }
}

/* ── Render one menu item (MENU_ITEM_H scanlines starting at abs_y) ── */
static void render_item(Menu_Handle_t *m, uint8_t item_idx, bool sel, uint16_t abs_y)
{
  MenuItem_t *it = &m->items[item_idx];
  uint16_t bg    = sel ? MENU_SEL_COLOR : MENU_BG_COLOR;

  char val[24];
  if (it->type == MENU_TYPE_GROUP) {
    snprintf(val, sizeof(val), ">");
  } else if (it->type == MENU_TYPE_ACTION) {
    snprintf(val, sizeof(val), ">> RUN");
  } else if (it->type == MENU_TYPE_INT) {
    snprintf(val, sizeof(val), "%ld%s",
             (long)*it->value_ptr, it->suffix ? it->suffix : "");
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

    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, MENU_X, MENU_W, (top || bot) ? MENU_BORDER_COLOR : bg);

    if (!top && !bot) {
      ln[MENU_X]                = sw16(MENU_BORDER_COLOR);
      ln[MENU_X + MENU_W - 1U] = sw16(MENU_BORDER_COLOR);
    }

    if (!top && !bot && fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      uint16_t row = fr - 4U;
      uint16_t lbl_col = (it->type == MENU_TYPE_GROUP) ? MENU_GROUP_COLOR : MENU_LBL_COLOR;
      LCD_LineStr(ln, (uint16_t)(MENU_X + 4U), row, it->label, &Font6x8, lbl_col, bg);

      uint16_t vcol;
      if (it->type == MENU_TYPE_GROUP)
        vcol = MENU_GROUP_COLOR;
      else if (sel && m->editing)
        vcol = MENU_EDIT_COLOR;
      else
        vcol = MENU_VAL_COLOR;
      LCD_LineStr(ln, (uint16_t)(MENU_X + MENU_W - 80U), row, val, &Font6x8, vcol, bg);
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
  m->item_count    = MENU_ITEM_COUNT;
  m->current_group = -1;
  _bl_val          = 80;

  /* ── Root groups (parent = -1) ──────────────────────── idx 0..5 */
  m->items[0] = (MenuItem_t){ "RX",     MENU_TYPE_GROUP, 0,0,0, NULL,NULL,0U,NULL,NULL, -1 };
  m->items[1] = (MenuItem_t){ "Audio",  MENU_TYPE_GROUP, 0,0,0, NULL,NULL,0U,NULL,NULL, -1 };
  m->items[2] = (MenuItem_t){ "Tuning", MENU_TYPE_GROUP, 0,0,0, NULL,NULL,0U,NULL,NULL, -1 };
  m->items[3] = (MenuItem_t){ "TX",     MENU_TYPE_GROUP, 0,0,0, NULL,NULL,0U,NULL,NULL, -1 };
  m->items[4] = (MenuItem_t){ "CW",     MENU_TYPE_GROUP, 0,0,0, NULL,NULL,0U,NULL,NULL, -1 };
  m->items[5] = (MenuItem_t){ "System", MENU_TYPE_GROUP, 0,0,0, NULL,NULL,0U,NULL,NULL, -1 };

  /* ── RX (parent = 0) ─────────────────────────────────── idx 6..12 */
  m->items[6]  = (MenuItem_t){ "AGC",      MENU_TYPE_ENUM,   0,   0,  0, &_agc_val, agc_strs,  2U, NULL, NULL,  0 };
  m->items[7]  = (MenuItem_t){ "NB",       MENU_TYPE_ENUM,   0,   0,  0, &_nb_val,  onoff_strs,2U, NULL, NULL,  0 };
  m->items[8]  = (MenuItem_t){ "NR",       MENU_TYPE_ENUM,   0,   0,  0, &_nr_val,  onoff_strs,2U, NULL, NULL,  0 };
  m->items[9]  = (MenuItem_t){ "ATT (dB)", MENU_TYPE_INT,    0,  31,  1, &_att_val, NULL,      0U, NULL, NULL,  0 };
  m->items[10] = (MenuItem_t){ "Squelch",  MENU_TYPE_INT,    0, 100,  1, &_sq_val,  NULL,      0U, NULL, NULL,  0 };
  m->items[11] = (MenuItem_t){ "RIT (Hz)", MENU_TYPE_INT, -999, 999,  1, &_rit_val, NULL,      0U, NULL, NULL,  0 };
  m->items[12] = (MenuItem_t){ "Span",     MENU_TYPE_ENUM,   0,   0,  0, &_zoom_val,zoom_strs, 4U, NULL, NULL,  0 };

  /* ── Audio (parent = 1) ──────────────────────────────── idx 13..15 */
  m->items[13] = (MenuItem_t){ "Volume",     MENU_TYPE_INT, 0,100,5, &_vol_val,  NULL,0U,NULL,NULL,  1 };
  m->items[14] = (MenuItem_t){ "Mic Gain",   MENU_TYPE_INT, 0,100,1, &_mic_val,  NULL,0U,NULL,NULL,  1 };
  m->items[15] = (MenuItem_t){ "Digi Drive", MENU_TYPE_INT, 0,100,1, &_digi_val, NULL,0U,NULL,NULL,  1 };

  /* ── Tuning (parent = 2) ─────────────────────────────── idx 16..18 */
  m->items[16] = (MenuItem_t){ "Step", MENU_TYPE_ENUM, 0,0,0, &_step_val, step_strs, 6U, NULL, NULL, 2 };
  m->items[17] = (MenuItem_t){ "Band", MENU_TYPE_ENUM, 0,0,0, &_band_val, band_strs,11U, NULL, NULL, 2 };
  m->items[18] = (MenuItem_t){ "Mode", MENU_TYPE_ENUM, 0,0,0, &_mode_val, mode_strs, 5U, NULL, NULL, 2 };

  /* ── TX (parent = 3) ─────────────────────────────────── idx 19..22 */
  m->items[19] = (MenuItem_t){ "RF Power", MENU_TYPE_INT,  5,  100,  5, &_rfpwr_val,NULL,      0U,NULL, "%",  3 };
  m->items[20] = (MenuItem_t){ "Ext ALC",  MENU_TYPE_ENUM, 0,    0,  0, &_alc_val,  onoff_strs,2U,NULL,NULL,  3 };
  m->items[21] = (MenuItem_t){ "TX Low",   MENU_TYPE_INT, 100,  500, 50, &_tx_low_val, NULL,   0U,NULL,"Hz",  3 };
  m->items[22] = (MenuItem_t){ "TX High",  MENU_TYPE_INT,2200, 3500,100, &_tx_high_val,NULL,   0U,NULL,"Hz",  3 };

  /* ── CW (parent = 4) ─────────────────────────────────── idx 23..31 */
  m->items[23] = (MenuItem_t){ "CW Pitch",     MENU_TYPE_INT, 300, 900, 10,&_cw_pitch_val,     NULL,      0U,NULL,"Hz",  4 };
  m->items[24] = (MenuItem_t){ "CW Speed",     MENU_TYPE_INT,   5,  40,  1,&_cw_wpm_val,       NULL,      0U,NULL,"WPM", 4 };
  m->items[25] = (MenuItem_t){ "Keyer Mode",   MENU_TYPE_ENUM,  0,   0,  0,&_cw_keyer_val,     keyer_strs,3U,NULL,NULL,  4 };
  m->items[26] = (MenuItem_t){ "CW Reverse",   MENU_TYPE_ENUM,  0,   0,  0,&_cw_rev_val,       onoff_strs,2U,NULL,NULL,  4 };
  m->items[27] = (MenuItem_t){ "Paddle Rev",   MENU_TYPE_ENUM,  0,   0,  0,&_cw_paddle_rev_val,onoff_strs,2U,NULL,NULL,  4 };
  m->items[28] = (MenuItem_t){ "Sidetone Vol", MENU_TYPE_INT,   0, 100,  5,&_cw_stvol_val,     NULL,      0U,NULL,"%",   4 };
  m->items[29] = (MenuItem_t){ "BK-IN",        MENU_TYPE_ENUM,  0,   0,  0,&_cw_bkin_val,      bkin_strs, 3U,NULL,NULL,  4 };
  m->items[30] = (MenuItem_t){ "BK Delay",     MENU_TYPE_INT,  50,2000, 50,&_cw_bkdly_val,     NULL,      0U,NULL,"ms",  4 };
  m->items[31] = (MenuItem_t){ "CW Filter",    MENU_TYPE_INT, 100,2000, 50,&_cw_filter_val,    NULL,      0U,NULL,"Hz",  4 };

  /* ── System (parent = 5) ─────────────────────────────── idx 32..35 */
  m->items[32] = (MenuItem_t){ "Backlight",   MENU_TYPE_INT,    0,100,10, &_bl_val, NULL,     0U,NULL,NULL,  5 };
  m->items[33] = (MenuItem_t){ "USB",         MENU_TYPE_ENUM,   0,  0, 0, &_usb_val,usb_strs,3U,NULL,NULL,  5 };
  m->items[34] = (MenuItem_t){ "Calibration", MENU_TYPE_ACTION, 0,  0, 0, NULL,     NULL,     0U,NULL,NULL,  5 };
  m->items[35] = (MenuItem_t){ "SWR Scan",    MENU_TYPE_ACTION, 0,  0, 0, NULL,     NULL,     0U,NULL,NULL,  5 };

  Menu_BuildView(m);
  /* USER CODE END Menu_Init_0 */
}

/* ════ Navigation ════ */
void Menu_Toggle(Menu_Handle_t *m)
{
  m->open = !m->open;
  m->editing = false;
  if (m->open) {
    m->current_group = -1;
    m->cursor        = 0U;
    m->scroll        = 0U;
    Menu_BuildView(m);
    Menu_Render(m);
  }
}

static void clamp_scroll(Menu_Handle_t *m)
{
  if (m->cursor < m->scroll) m->scroll = m->cursor;
  if (m->cursor >= m->scroll + MENU_VISIBLE_ROWS)
    m->scroll = (uint8_t)(m->cursor - MENU_VISIBLE_ROWS + 1U);
}

static void change_val(Menu_Handle_t *m, int32_t d)
{
  MenuItem_t *it = &m->items[m->view[m->cursor]];
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
  else { if (m->cursor < m->view_count - 1U) m->cursor++; clamp_scroll(m); }
  Menu_Render(m);
}

void Menu_Select(Menu_Handle_t *m)
{
  if (!m->open) return;
  uint8_t idx = m->view[m->cursor];

  if (m->items[idx].type == MENU_TYPE_GROUP) {
    m->current_group = (int8_t)idx;
    Menu_BuildView(m);
    m->cursor  = 0U;
    m->scroll  = 0U;
    m->editing = false;
  } else if (m->items[idx].type == MENU_TYPE_ACTION) {
    if (m->items[idx].on_change) m->items[idx].on_change();
  } else {
    m->editing = !m->editing;
    if (!m->editing && s_apply_cb) s_apply_cb();
  }
  Menu_Render(m);
}

void Menu_Confirm(Menu_Handle_t *m)
{
  if (!m->open) return;
  if (m->editing) {
    m->editing = false;
    if (s_apply_cb) s_apply_cb();
    Menu_Render(m);
  } else {
    Menu_Back(m);
  }
}

void Menu_Back(Menu_Handle_t *m)
{
  if (!m->open) return;
  if (m->editing) {
    m->editing = false;
    Menu_Render(m);
  } else if (m->current_group >= 0) {
    uint8_t prev = (uint8_t)m->current_group;
    m->current_group = -1;
    Menu_BuildView(m);
    for (uint8_t i = 0U; i < m->view_count; i++) {
      if (m->view[i] == prev) { m->cursor = i; break; }
    }
    clamp_scroll(m);
    m->editing = false;
    Menu_Render(m);
  } else {
    m->open = false;
  }
}

void Menu_EncoderEdit(Menu_Handle_t *m, int32_t delta)
{ if (m->open && m->editing) { change_val(m, delta); Menu_Render(m); } }

/* ════ Menu_Render ════ */
void Menu_Render(Menu_Handle_t *m)
{
  /* USER CODE BEGIN Menu_Render_0 */
  if (!m->open) return;

  uint16_t y = (uint16_t)MENU_Y;

  /* ── Header (16px) ── */
  for (uint16_t fr = 0U; fr < 16U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, MENU_X, MENU_W,
                 (fr == 0U || fr == 15U) ? MENU_BORDER_COLOR : MENU_HEADER_BG);
    ln[MENU_X]                = sw16(MENU_BORDER_COLOR);
    ln[MENU_X + MENU_W - 1U] = sw16(MENU_BORDER_COLOR);

    if (fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      const char *t;
      if (m->editing)
        t = " [ EDIT ] ";
      else if (m->current_group >= 0)
        t = m->items[m->current_group].label;
      else
        t = " -= MENU =- ";
      uint16_t tx = (uint16_t)(MENU_X + (MENU_W - (uint16_t)strlen(t) * Font6x8.width) / 2U);
      LCD_LineStr(ln, tx, fr - 4U, t, &Font6x8, 0xFFFFU, MENU_HEADER_BG);
    }
    push_ln(y++);
  }

  /* ── Items (always render all MENU_VISIBLE_ROWS; clear unused rows) ── */
  for (uint8_t r = 0U; r < MENU_VISIBLE_ROWS; r++) {
    uint8_t vi = (uint8_t)(m->scroll + r);
    if (vi < m->view_count) {
      render_item(m, m->view[vi], (vi == m->cursor), y);
    } else {
      for (uint16_t fr = 0U; fr < (uint16_t)MENU_ITEM_H; fr++) {
        uint16_t *ln = LN;
        LCD_LineFill(ln, 0U, LCD_W, UI_BG);
        push_ln((uint16_t)(y + fr));
      }
    }
    y += (uint16_t)MENU_ITEM_H;
  }

  /* ── Hint row ── */
  {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    char cnt[10];
    snprintf(cnt, sizeof(cnt), "%d/%d", m->cursor + 1, m->view_count);
    LCD_LineStr(ln, (uint16_t)(MENU_X + MENU_W - 42U), 0U,
                cnt, &Font6x8, MENU_LBL_COLOR, UI_BG);
    const char *hint = (m->current_group >= 0)
                     ? "F1=UP F2=DN ENC=EDIT F4=BACK"
                     : "F1=UP F2=DN ENC=SEL  F4=EXIT";
    LCD_LineStr(ln, (uint16_t)(MENU_X + 4U), 0U, hint, &Font6x8, MENU_LBL_COLOR, UI_BG);
    for (uint8_t fr = 0U; fr < (uint8_t)Font6x8.height; fr++)
      push_ln(y++);
  }
  /* USER CODE END Menu_Render_0 */
}

/* ════ Load / Save SDR state ════ */
void Menu_LoadFromSDR(Menu_Handle_t *m,
                       bool agc_fast, bool nb, bool nr, int16_t rit,
                       uint8_t vol, uint8_t mic_gain, uint8_t digi_gain,
                       uint8_t sq, uint32_t step,
                       uint8_t att, uint8_t band, uint8_t mode,
                       uint8_t usb_mode, uint8_t zoom,
                       bool ext_alc, uint8_t rf_power_pct, uint8_t pa_watts,
                       uint8_t backlight,
                       uint16_t cw_pitch, uint8_t cw_wpm, uint8_t cw_keyer,
                       bool cw_rev, bool cw_paddle_rev,
                       uint8_t cw_st_vol, uint8_t cw_bkin, uint16_t cw_bk_delay,
                       uint16_t cw_filter,
                       uint16_t tx_audio_low_hz,
                       uint16_t tx_audio_high_hz,
                       MenuApplyFn apply_cb)
{
  /* USER CODE BEGIN Menu_LoadFromSDR_0 */
  static const uint32_t sv[6] = {1,10,100,1000,10000,100000};
  _agc_val  = agc_fast ? 1 : 0;
  _nb_val   = nb  ? 1 : 0;
  _nr_val   = nr  ? 1 : 0;
  _rit_val  = (int32_t)rit;
  _vol_val  = (int32_t)vol;
  _mic_val  = (int32_t)mic_gain;
  _digi_val = (int32_t)digi_gain;
  _sq_val   = (int32_t)sq;
  _step_val = 2;
  for (uint8_t i = 0; i < 6U; i++) if (step == sv[i]) { _step_val = (int32_t)i; break; }
  _att_val  = (int32_t)att;
  _band_val = (int32_t)band;
  _mode_val = (int32_t)mode;
  _usb_val  = (int32_t)usb_mode;
  _zoom_val = (int32_t)zoom;
  _alc_val  = ext_alc ? 1 : 0;
  _bl_val   = (backlight >= 10U && backlight <= 100U) ? (int32_t)backlight : 80;

  /* RF Power: show Watts when pa_watts configured, else percent */
  s_pa_watts = pa_watts;
  if (pa_watts > 0U) {
    uint8_t pct = (rf_power_pct > 0U && rf_power_pct <= 100U) ? rf_power_pct : 100U;
    int32_t w   = (int32_t)((uint32_t)pct * pa_watts / 100U);
    if (w < 1) w = 1;
    if (w > (int32_t)pa_watts) w = (int32_t)pa_watts;
    _rfpwr_val                              = w;
    m->items[MENU_IDX_RFPOWER].min          = 1;
    m->items[MENU_IDX_RFPOWER].max          = (int32_t)pa_watts;
    m->items[MENU_IDX_RFPOWER].step         = (pa_watts >= 50U) ? 5 : 1;
    m->items[MENU_IDX_RFPOWER].suffix       = "W";
  } else {
    _rfpwr_val                              = (rf_power_pct >= 5U && rf_power_pct <= 100U) ? (int32_t)rf_power_pct : 100;
    m->items[MENU_IDX_RFPOWER].min          = 5;
    m->items[MENU_IDX_RFPOWER].max          = 100;
    m->items[MENU_IDX_RFPOWER].step         = 5;
    m->items[MENU_IDX_RFPOWER].suffix       = "%";
  }

  /* CW */
  _cw_pitch_val      = (cw_pitch >= 300U && cw_pitch <= 900U) ? (int32_t)cw_pitch : 700;
  _cw_wpm_val        = (cw_wpm >= 5U && cw_wpm <= 40U)        ? (int32_t)cw_wpm   : 20;
  _cw_keyer_val      = (cw_keyer <= 2U)                        ? (int32_t)cw_keyer : 0;
  _cw_rev_val        = cw_rev        ? 1 : 0;
  _cw_paddle_rev_val = cw_paddle_rev ? 1 : 0;
  _cw_stvol_val      = (cw_st_vol <= 100U)                     ? (int32_t)cw_st_vol  : 50;
  _cw_bkin_val       = (cw_bkin <= 2U)                         ? (int32_t)cw_bkin    : 0;
  _cw_bkdly_val      = (cw_bk_delay >= 50U && cw_bk_delay <= 2000U) ? (int32_t)cw_bk_delay : 150;
  _cw_filter_val     = (cw_filter >= 100U && cw_filter <= 2000U)     ? (int32_t)cw_filter    : 500;

  _tx_low_val  = (tx_audio_low_hz  >= 100U && tx_audio_low_hz  <= 500U)  ? (int32_t)tx_audio_low_hz  : 200;
  _tx_high_val = (tx_audio_high_hz >= 2200U && tx_audio_high_hz <= 3500U) ? (int32_t)tx_audio_high_hz : 2800;

  s_apply_cb = apply_cb;
  /* USER CODE END Menu_LoadFromSDR_0 */
}

void Menu_SaveToSDR(Menu_Handle_t *m,
                     bool *agc_fast, bool *nb, bool *nr, int16_t *rit,
                     uint8_t *vol, uint8_t *mic_gain, uint8_t *digi_gain,
                     uint8_t *sq, uint32_t *step,
                     uint8_t *att, uint8_t *band, uint8_t *mode,
                     uint8_t *usb_mode, uint8_t *zoom,
                     bool *ext_alc, uint8_t *rf_power, uint8_t *backlight,
                     uint16_t *cw_pitch, uint8_t *cw_wpm, uint8_t *cw_keyer,
                     bool *cw_rev, bool *cw_paddle_rev,
                     uint8_t *cw_st_vol, uint8_t *cw_bkin, uint16_t *cw_bk_delay,
                     uint16_t *cw_filter,
                     uint16_t *tx_audio_low_hz,
                     uint16_t *tx_audio_high_hz)
{
  /* USER CODE BEGIN Menu_SaveToSDR_0 */
  (void)m;
  static const uint32_t sv[6] = {1,10,100,1000,10000,100000};
  *agc_fast  = (_agc_val  != 0);
  *nb        = (_nb_val   != 0);
  *nr        = (_nr_val   != 0);
  *rit       = (int16_t)_rit_val;
  *vol       = (uint8_t)_vol_val;
  *mic_gain  = (uint8_t)_mic_val;
  *digi_gain = (uint8_t)_digi_val;
  *sq        = (uint8_t)_sq_val;
  *step      = sv[(_step_val >= 0 && _step_val < 6) ? _step_val : 2];
  *att       = (uint8_t)_att_val;
  *band      = (uint8_t)_band_val;
  *mode      = (uint8_t)_mode_val;
  *usb_mode  = (uint8_t)_usb_val;
  *zoom      = (uint8_t)(_zoom_val >= 0 && _zoom_val < 4 ? _zoom_val : 0);
  *ext_alc   = (_alc_val != 0);
  if (s_pa_watts > 0U) {
    int32_t w   = (_rfpwr_val >= 1) ? _rfpwr_val : 1;
    uint32_t pct = (uint32_t)w * 100U / s_pa_watts;
    if (pct < 1U)   pct = 1U;
    if (pct > 100U) pct = 100U;
    *rf_power = (uint8_t)pct;
  } else {
    *rf_power = (uint8_t)(_rfpwr_val >= 5 && _rfpwr_val <= 100 ? _rfpwr_val : 100);
  }
  *backlight = (uint8_t)(_bl_val >= 10 && _bl_val <= 100 ? _bl_val : 80);

  /* CW */
  *cw_pitch    = (uint16_t)(_cw_pitch_val >= 300 && _cw_pitch_val <= 900 ? _cw_pitch_val : 700);
  *cw_wpm      = (uint8_t)(_cw_wpm_val >= 5 && _cw_wpm_val <= 40 ? _cw_wpm_val : 20);
  *cw_keyer    = (uint8_t)(_cw_keyer_val >= 0 && _cw_keyer_val <= 2 ? _cw_keyer_val : 0);
  *cw_rev      = (_cw_rev_val != 0);
  *cw_paddle_rev = (_cw_paddle_rev_val != 0);
  *cw_st_vol   = (uint8_t)(_cw_stvol_val >= 0 && _cw_stvol_val <= 100 ? _cw_stvol_val : 50);
  *cw_bkin     = (uint8_t)(_cw_bkin_val >= 0 && _cw_bkin_val <= 2 ? _cw_bkin_val : 0);
  *cw_bk_delay = (uint16_t)(_cw_bkdly_val >= 50 && _cw_bkdly_val <= 2000 ? _cw_bkdly_val : 150);
  *cw_filter        = (uint16_t)(_cw_filter_val >= 100 && _cw_filter_val <= 2000 ? _cw_filter_val : 500);
  *tx_audio_low_hz  = (uint16_t)(_tx_low_val  >= 100 && _tx_low_val  <= 500  ? _tx_low_val  : 200);
  *tx_audio_high_hz = (uint16_t)(_tx_high_val >= 2200 && _tx_high_val <= 3500 ? _tx_high_val : 2800);
  /* USER CODE END Menu_SaveToSDR_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
