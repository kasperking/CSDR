/* USER CODE BEGIN Header */
/**
  * @file menu.c
  * @brief SDR Menu System – 2-level hierarchical scanline renderer
  */
/* USER CODE END Header */

#include "menu.h"
#include "sdr_ui.h"
#include "build_info.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ── Private: SWAP16 ── */
static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

#define LN  LCD_GetLineBuf()
#define MENU_EDIT_COLOR  UI_FREQ_MHZ

/* USER CODE BEGIN PV */
Menu_Handle_t g_menu;

/* RX group */
static int32_t _agc_val, _nb_val, _nr_val, _rit_val;
static int32_t _att_val, _sq_val, _zoom_val, _bw_val;
static int32_t _rxshift_val, _notch_val, _notchhz_val;
static int32_t _cwdec_val;

/* CW group */
static int32_t _cw_pitch_val, _cw_wpm_val, _keyer_val, _paddlerev_val;
static int32_t _sidetone_val, _bkin_val, _bkdelay_val, _cwrev_val, _cwfilter_val;

/* Audio group */
static int32_t _vol_val, _mic_val, _digi_val;

/* Tuning group */
static int32_t _step_val, _band_val, _mode_val;

/* TX group */
static int32_t _rfpwr_val, _alc_val, _tx_low_val, _tx_high_val;
static int32_t _vox_val, _voxgain_val, _voxdelay_val;

/* System group */
static int32_t _bl_val, _usb_val, _iq_stream_val;

/* Misc */
static uint8_t s_pa_watts = 0U;

static const char *agc_strs[]   = { "SLOW", "FAST", "AUTO" };
static const char *onoff_strs[] = { "OFF",  "ON"   };
static const char *keyer_strs[] = { "STRAIGHT", "IAMBIC-A", "IAMBIC-B" };
static const char *bkin_strs[]  = { "OFF", "SEMI", "FULL" };
static const char *step_strs[] = { "1Hz","10Hz","100Hz","1KHz","10KHz","100KHz" };
static const char *band_strs[] = { "160m","80m","60m","40m","30m",
                                    "20m","17m","15m","12m","10m","6m" };
static const char *mode_strs[] = { "AM","FM","USB","LSB","CW","DIGU","DIGL","FDV" };
static const char *usb_strs[]      = { "Off","CAT","Audio" };
static const char *iq_stream_strs[] = { "IQ","Demod" };
static const char *zoom_strs[] = { "+/-24k","+/-12k","+/-6k","+/-3k" };

static MenuApplyFn s_apply_cb = NULL;

/* About info strings */
static const char *about_ver_strs[]  = { FW_VERSION_STR };
static const char *about_date_strs[] = { FW_BUILD_DATE };
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

/* ── Rebuild view[] for current_group ── */
static void Menu_BuildView(Menu_Handle_t *m)
{
  m->view_count = 0U;
  for (uint8_t i = 0U; i < m->item_count; i++) {
    if ((int8_t)m->items[i].parent == m->current_group)
      m->view[m->view_count++] = i;
  }
  if (m->cursor >= m->view_count)
    m->cursor = (m->view_count > 0U) ? (uint8_t)(m->view_count - 1U) : 0U;
  m->scroll = 0U;
}

/* ── Push one scanline via FMC ── */
static void push_ln(uint16_t y)
{
  LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, LN, LCD_W);
}

/* ── Render one row from view[vi] ── */
static void render_item(Menu_Handle_t *m, uint8_t vi, uint16_t abs_y)
{
  uint8_t item_idx = m->view[vi];
  MenuItem_t *it   = &m->items[item_idx];
  bool sel         = (vi == m->cursor);
  bool is_group    = (it->type == MENU_TYPE_GROUP);

  uint16_t bg = sel ? MENU_SEL_COLOR
              : is_group ? MENU_GROUP_COLOR
              : MENU_BG_COLOR;

  char val[24];
  if (is_group) {
    snprintf(val, sizeof(val), " >");
  } else if (it->type == MENU_TYPE_ACTION) {
    snprintf(val, sizeof(val), ">> RUN");
  } else if (it->type == MENU_TYPE_INFO) {
    snprintf(val, sizeof(val), "%s", (it->enum_strs && it->enum_strs[0]) ? it->enum_strs[0] : "");
  } else if (it->type == MENU_TYPE_INT) {
    snprintf(val, sizeof(val), "%ld%s",
             (long)*it->value_ptr, it->suffix ? it->suffix : "");
  } else {
    int32_t vi2 = *it->value_ptr;
    if (vi2 < 0) vi2 = 0;
    if (vi2 >= (int32_t)it->enum_count) vi2 = (int32_t)it->enum_count - 1;
    snprintf(val, sizeof(val), "%s", it->enum_strs[vi2]);
  }

  /* Right-align value with 4px margin inside the menu right border */
  uint16_t val_w = (uint16_t)(strlen(val) * (uint16_t)Font6x8.width);
  uint16_t val_x = (uint16_t)(MENU_X + MENU_W - 4U - val_w);
  if (val_x < (uint16_t)(MENU_X + 4U)) val_x = (uint16_t)(MENU_X + 4U);

  /* Clip label so it does not overlap the value column (keep 4px gap) */
  char lbl_buf[20];
  {
    uint16_t lbl_end = (val_x > 8U) ? (uint16_t)(val_x - 4U) : 0U;
    uint8_t  max_ch  = (lbl_end > (uint16_t)(MENU_X + 4U))
                     ? (uint8_t)((lbl_end - (uint16_t)(MENU_X + 4U)) / Font6x8.width)
                     : 0U;
    uint8_t  n = (uint8_t)strlen(it->label);
    if (n > max_ch && max_ch >= 1U) { memcpy(lbl_buf, it->label, max_ch); lbl_buf[max_ch] = '\0'; }
    else                             { memcpy(lbl_buf, it->label, n + 1U); }
  }

  uint16_t lbl_clr = is_group ? 0x0000U : MENU_LBL_COLOR;

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
      LCD_LineStr(ln, (uint16_t)(MENU_X + 4U), row, lbl_buf, &Font6x8, lbl_clr, bg);
      uint16_t vcol = (sel && m->editing)           ? MENU_EDIT_COLOR
                    : (it->type == MENU_TYPE_INFO)   ? MENU_LBL_COLOR
                    : MENU_VAL_COLOR;
      if (is_group) vcol = 0x0000U;
      LCD_LineStr(ln, val_x, row, val, &Font6x8, vcol, bg);
    }

    push_ln((uint16_t)(abs_y + fr));
  }
}

/* ── Render blank row (clear stale content) ── */
static void render_blank_row(uint16_t abs_y)
{
  for (uint16_t fr = 0U; fr < (uint16_t)MENU_ITEM_H; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
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
  m->prev_group    = -1;
  _bl_val          = 80;

  /* ── Root groups (parent = -1) — appear in index order in root view ── */
  m->items[0] = (MenuItem_t){ "RX",    MENU_TYPE_GROUP,0,0,0,NULL,NULL,0U,NULL,NULL,-1 };
  m->items[1] = (MenuItem_t){ "Audio", MENU_TYPE_GROUP,0,0,0,NULL,NULL,0U,NULL,NULL,-1 };
  m->items[2] = (MenuItem_t){ "Tuning",MENU_TYPE_GROUP,0,0,0,NULL,NULL,0U,NULL,NULL,-1 };
  m->items[3] = (MenuItem_t){ "TX",    MENU_TYPE_GROUP,0,0,0,NULL,NULL,0U,NULL,NULL,-1 };
  m->items[4] = (MenuItem_t){ "CW",    MENU_TYPE_GROUP,0,0,0,NULL,NULL,0U,NULL,NULL,-1 };
  m->items[5] = (MenuItem_t){ "System",MENU_TYPE_GROUP,0,0,0,NULL,NULL,0U,NULL,NULL,-1 };

  /* ── RX group (parent = 0) ──────────────────────────────── */
  m->items[6]  = (MenuItem_t){ "BW",      MENU_TYPE_INT,  100,24000,100, &_bw_val,     NULL,      0U, NULL,"Hz",0 };
  m->items[7]  = (MenuItem_t){ "AGC",     MENU_TYPE_ENUM, 0,0,0,         &_agc_val,    agc_strs,  3U, NULL,NULL,0 };
  m->items[8]  = (MenuItem_t){ "ATT(dB)", MENU_TYPE_INT,  0, 31, 1,      &_att_val,    NULL,      0U, NULL,"dB",0 };
  m->items[9]  = (MenuItem_t){ "Squelch", MENU_TYPE_INT,  0,100, 1,      &_sq_val,     NULL,      0U, NULL,NULL,0 };
  m->items[10] = (MenuItem_t){ "Span",    MENU_TYPE_ENUM, 0,0,0,         &_zoom_val,   zoom_strs, 4U, NULL,NULL,0 };
  m->items[11] = (MenuItem_t){ "NR",      MENU_TYPE_ENUM, 0,0,0,         &_nr_val,     onoff_strs,2U, NULL,NULL,0 };
  m->items[12] = (MenuItem_t){ "NB",      MENU_TYPE_ENUM, 0,0,0,         &_nb_val,     onoff_strs,2U, NULL,NULL,0 };
  m->items[13] = (MenuItem_t){ "Notch",   MENU_TYPE_ENUM, 0,0,0,         &_notch_val,  onoff_strs,2U, NULL,NULL,0 };
  m->items[14] = (MenuItem_t){ "Notch Hz",MENU_TYPE_INT,  100,4000,50,   &_notchhz_val,NULL,      0U, NULL,"Hz",0 };
  m->items[15] = (MenuItem_t){ "RIT(Hz)", MENU_TYPE_INT,  -999,999,1,    &_rit_val,    NULL,      0U, NULL,NULL,0 };
  m->items[16] = (MenuItem_t){ "RX Shift",MENU_TYPE_INT,  -2000,2000,50, &_rxshift_val,NULL,      0U, NULL,"Hz",0 };

  /* ── Audio group (parent = 1) ───────────────────────────── */
  m->items[17] = (MenuItem_t){ "Volume",    MENU_TYPE_INT, 0,100,5, &_vol_val, NULL,0U,NULL,NULL,1 };
  m->items[18] = (MenuItem_t){ "Mic Gain",  MENU_TYPE_INT, 0,100,1, &_mic_val, NULL,0U,NULL,NULL,1 };
  m->items[19] = (MenuItem_t){ "Digi Drive",MENU_TYPE_INT, 0,100,1, &_digi_val,NULL,0U,NULL,NULL,1 };

  /* ── Tuning group (parent = 2) ──────────────────────────── */
  m->items[20] = (MenuItem_t){ "Step",MENU_TYPE_ENUM,0,0,0,&_step_val,step_strs,6U, NULL,NULL,2 };
  m->items[21] = (MenuItem_t){ "Band",MENU_TYPE_ENUM,0,0,0,&_band_val,band_strs,11U,NULL,NULL,2 };
  m->items[22] = (MenuItem_t){ "Mode",MENU_TYPE_ENUM,0,0,0,&_mode_val,mode_strs,8U, NULL,NULL,2 };

  /* ── TX group (parent = 3) ──────────────────────────────── */
  m->items[23] = (MenuItem_t){ "RF Power", MENU_TYPE_INT,  5,100,5,    &_rfpwr_val,   NULL,      0U,NULL,"%", 3 };
  m->items[24] = (MenuItem_t){ "VOX",      MENU_TYPE_ENUM, 0,0,0,      &_vox_val,     onoff_strs,2U,NULL,NULL,3 };
  m->items[25] = (MenuItem_t){ "VOX Gain", MENU_TYPE_INT,  0,100,5,    &_voxgain_val, NULL,      0U,NULL,NULL,3 };
  m->items[26] = (MenuItem_t){ "VOX Delay",MENU_TYPE_INT,100,2000,100, &_voxdelay_val,NULL,      0U,NULL,"ms",3 };
  m->items[27] = (MenuItem_t){ "TX Low",   MENU_TYPE_INT, 100,500,50,  &_tx_low_val,  NULL,      0U,NULL,"Hz",3 };
  m->items[28] = (MenuItem_t){ "TX High",  MENU_TYPE_INT,2200,3500,100,&_tx_high_val, NULL,      0U,NULL,"Hz",3 };
  m->items[29] = (MenuItem_t){ "Ext ALC",  MENU_TYPE_ENUM, 0,0,0,      &_alc_val,     onoff_strs,2U,NULL,NULL,3 };

  /* ── CW group (parent = 4) ──────────────────────────────── */
  m->items[30] = (MenuItem_t){ "CW Decode",MENU_TYPE_ENUM, 0,0,0,    &_cwdec_val,    onoff_strs,2U, NULL,NULL,4 };
  m->items[31] = (MenuItem_t){ "Pitch",    MENU_TYPE_INT,  300,900,50,&_cw_pitch_val, NULL,      0U, NULL,"Hz",4 };
  m->items[32] = (MenuItem_t){ "Speed",    MENU_TYPE_INT,  5,  40, 1, &_cw_wpm_val,  NULL,      0U, NULL,"WPM",4 };
  m->items[33] = (MenuItem_t){ "Keyer",    MENU_TYPE_ENUM, 0,0,0,    &_keyer_val,    keyer_strs,3U, NULL,NULL,4 };
  m->items[34] = (MenuItem_t){ "Sidetone", MENU_TYPE_INT,  0,100,5,  &_sidetone_val, NULL,      0U, NULL,"%", 4 };
  m->items[35] = (MenuItem_t){ "BK-IN",    MENU_TYPE_ENUM, 0,0,0,    &_bkin_val,     bkin_strs, 3U, NULL,NULL,4 };
  m->items[36] = (MenuItem_t){ "BK Delay", MENU_TYPE_INT,  50,2000,50,&_bkdelay_val, NULL,      0U, NULL,"ms",4 };
  m->items[37] = (MenuItem_t){ "CW Rev",   MENU_TYPE_ENUM, 0,0,0,    &_cwrev_val,    onoff_strs,2U, NULL,NULL,4 };
  m->items[38] = (MenuItem_t){ "Paddle Rev",MENU_TYPE_ENUM,0,0,0,    &_paddlerev_val,onoff_strs,2U, NULL,NULL,4 };
  m->items[39] = (MenuItem_t){ "Filter",   MENU_TYPE_INT,  50,500,50,&_cwfilter_val, NULL,      0U, NULL,"Hz",4 };

  /* ── System group (parent = 5) ──────────────────────────── */
  m->items[40] = (MenuItem_t){ "Backlight",   MENU_TYPE_INT,   0,100,10,&_bl_val,        NULL,          0U,NULL,NULL,5 };
  m->items[41] = (MenuItem_t){ "USB",         MENU_TYPE_ENUM,  0,0,0,   &_usb_val,       usb_strs,      3U,NULL,NULL,5 };
  m->items[42] = (MenuItem_t){ "USB Stream",  MENU_TYPE_ENUM,  0,0,0,   &_iq_stream_val, iq_stream_strs,2U,NULL,NULL,5 };
  m->items[43] = (MenuItem_t){ "Calibration", MENU_TYPE_ACTION,0,0,0,   NULL,NULL,           0U,NULL,NULL,5 };
  m->items[44] = (MenuItem_t){ "Factory Reset",MENU_TYPE_ACTION,0,0,0,   NULL,NULL,           0U,NULL,NULL,5 };
  m->items[45] = (MenuItem_t){ "About",        MENU_TYPE_GROUP, 0,0,0,   NULL,NULL,           0U,NULL,NULL,5 };

  /* ── About sub-group (parent = 45) ──────────────────────── */
  m->items[46] = (MenuItem_t){ "Version",    MENU_TYPE_INFO,  0,0,0, NULL,about_ver_strs, 1U,NULL,NULL,45 };
  m->items[47] = (MenuItem_t){ "Build Date", MENU_TYPE_INFO,  0,0,0, NULL,about_date_strs,1U,NULL,NULL,45 };

  /* ── Root action (parent = -1) — last so it appears at end of root view ── */
  m->items[48] = (MenuItem_t){ "SWR Scan",  MENU_TYPE_ACTION,0,0,0, NULL,NULL,0U,NULL,NULL,-1 };

  Menu_BuildView(m);
  /* USER CODE END Menu_Init_0 */
}

/* ════ Navigation ════ */
void Menu_Toggle(Menu_Handle_t *m)
{
  m->open = !m->open;
  m->editing = false;
  m->current_group = -1;
  m->prev_group    = -1;
  Menu_BuildView(m);
  m->cursor = 0U;
  m->scroll = 0U;
  if (m->open) Menu_Render(m);
}

static void clamp_scroll(Menu_Handle_t *m)
{
  if (m->cursor < m->scroll) m->scroll = m->cursor;
  if (m->cursor >= m->scroll + MENU_VISIBLE_ROWS)
    m->scroll = (uint8_t)(m->cursor - MENU_VISIBLE_ROWS + 1U);
}

static void change_val(Menu_Handle_t *m, int32_t d)
{
  MenuItem_t *it = Menu_CurrentItem(m);
  if (!it) return;
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
  else {
    if (m->cursor + 1U < m->view_count) m->cursor++;
    clamp_scroll(m);
  }
  Menu_Render(m);
}

void Menu_Select(Menu_Handle_t *m)
{
  if (!m->open) return;
  MenuItem_t *it = Menu_CurrentItem(m);
  if (!it) return;
  if (it->type == MENU_TYPE_GROUP) {
    uint8_t gidx     = m->view[m->cursor];
    m->prev_group    = m->current_group;
    m->current_group = (int8_t)gidx;
    m->editing = false;
    Menu_BuildView(m);
    m->cursor = 0U;
    m->scroll = 0U;
  } else if (it->type != MENU_TYPE_ACTION && it->type != MENU_TYPE_INFO) {
    m->editing = !m->editing;
    if (!m->editing && s_apply_cb) s_apply_cb();
  }
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
  if (m->editing) {
    m->editing = false;
    Menu_Render(m);
  } else if (m->current_group >= 0) {
    int8_t was_group = m->current_group;
    m->current_group = m->prev_group;
    m->prev_group    = -1;
    m->editing = false;
    Menu_BuildView(m);
    for (uint8_t i = 0U; i < m->view_count; i++) {
      if ((int8_t)m->view[i] == was_group) { m->cursor = i; break; }
    }
    clamp_scroll(m);
    Menu_Render(m);
  } else {
    m->open = false;
  }
}

void Menu_EncoderEdit(Menu_Handle_t *m, int32_t delta)
{
  if (!m->open) return;
  int32_t d = (delta > 0) ? 1 : (delta < 0) ? -1 : 0;
  if (m->editing) {
    change_val(m, d);
  } else {
    if (d > 0 && m->cursor + 1U < m->view_count) m->cursor++;
    else if (d < 0 && m->cursor > 0U) m->cursor--;
    clamp_scroll(m);
  }
  Menu_Render(m);
}

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
      const char *t;
      if (m->editing)
        t = " [ EDIT ] ";
      else if (m->current_group >= 0)
        t = m->items[(uint8_t)m->current_group].label;
      else
        t = " -= MENU =- ";
      uint16_t tx = (uint16_t)(MENU_X + (MENU_W - (uint16_t)strlen(t) * Font6x8.width) / 2U);
      LCD_LineStr(ln, tx, fr - 4U, t, &Font6x8, 0xFFFFU, MENU_HEADER_BG);
    }
    push_ln(y++);
  }

  /* Items */
  for (uint8_t r = 0U; r < MENU_VISIBLE_ROWS; r++) {
    uint8_t vi = (uint8_t)(m->scroll + r);
    if (vi < m->view_count)
      render_item(m, vi, y);
    else
      render_blank_row(y);
    y += (uint16_t)MENU_ITEM_H;
  }

  /* Hint row — draw hint first, counter on top so it always wins on overlap */
  {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    const char *hint = m->editing
                     ? "ENC=CHANGE  BTN/F4=DONE"
                     : (m->current_group >= 0)
                     ? "ENC=MOVE BTN=EDIT F4=BACK"
                     : "ENC=MOVE BTN=SEL  F4=EXIT";
    LCD_LineStr(ln, (uint16_t)(MENU_X + 4U), 0U, hint, &Font6x8, MENU_LBL_COLOR, UI_BG);
    char cnt[12];
    snprintf(cnt, sizeof(cnt), "%d/%d", m->cursor + 1, m->view_count);
    uint16_t cnt_w = (uint16_t)(strlen(cnt) * (uint16_t)Font6x8.width);
    LCD_LineStr(ln, (uint16_t)(MENU_X + MENU_W - 4U - cnt_w), 0U,
                cnt, &Font6x8, MENU_LBL_COLOR, UI_BG);
    for (uint8_t fr = 0U; fr < (uint8_t)Font6x8.height; fr++)
      push_ln(y++);
  }
  /* USER CODE END Menu_Render_0 */
}

/* ════ Load / Save SDR state ════ */
void Menu_LoadFromSDR(Menu_Handle_t *m,
                       uint8_t agc_speed, bool nb, bool nr, int16_t rit,
                       uint8_t vol, uint8_t mic_gain, uint8_t digi_gain,
                       uint8_t sq, uint32_t step, uint32_t bw_hz,
                       uint8_t att, uint8_t band, uint8_t mode,
                       uint8_t usb_mode, uint8_t zoom,
                       bool ext_alc, uint8_t rf_power_pct, uint8_t pa_watts,
                       uint16_t tx_audio_low_hz, uint16_t tx_audio_high_hz,
                       int16_t rx_shift_hz,
                       bool notch_on, int16_t notch_hz,
                       bool vox_on, uint8_t vox_gain, uint16_t vox_delay,
                       bool cw_decode_on,
                       uint16_t cw_pitch_hz, uint8_t cw_wpm,
                       uint8_t keyer_mode, bool paddle_reverse,
                       uint8_t sidetone_vol, uint8_t cw_bkin,
                       uint16_t cw_bk_delay_ms, bool cw_reverse,
                       uint16_t cw_filter_hz,
                       bool usb_iq_stream,
                       MenuApplyFn apply_cb)
{
  /* USER CODE BEGIN Menu_LoadFromSDR_0 */
  static const uint32_t sv[6] = {1,10,100,1000,10000,100000};
  _agc_val  = (int32_t)(agc_speed <= 2U ? agc_speed : 1U);
  _nb_val   = nb  ? 1 : 0;
  _nr_val   = nr  ? 1 : 0;
  _rit_val  = (int32_t)rit;
  _rxshift_val = (int32_t)rx_shift_hz;
  _notch_val   = notch_on ? 1 : 0;
  _notchhz_val = (notch_hz >= 100 && notch_hz <= 4000) ? (int32_t)notch_hz : 1000;
  _vol_val  = (int32_t)vol;
  _mic_val  = (int32_t)mic_gain;
  _digi_val = (int32_t)digi_gain;
  _sq_val   = (int32_t)sq;
  _step_val = 2;
  for (uint8_t i = 0; i < 6U; i++) if (step == sv[i]) { _step_val = (int32_t)i; break; }
  _bw_val   = (bw_hz >= 100U && bw_hz <= 24000U) ? (int32_t)bw_hz : 3000;
  _att_val  = (int32_t)att;
  _band_val = (int32_t)band;
  _mode_val = (int32_t)mode;
  _usb_val       = (int32_t)usb_mode;
  _iq_stream_val = usb_iq_stream ? 1 : 0;
  _zoom_val      = (int32_t)zoom;
  _alc_val  = ext_alc ? 1 : 0;

  _tx_low_val  = (tx_audio_low_hz  >= 100U && tx_audio_low_hz  <= 500U)  ? (int32_t)tx_audio_low_hz  : 200;
  _tx_high_val = (tx_audio_high_hz >= 2200U && tx_audio_high_hz <= 3500U) ? (int32_t)tx_audio_high_hz : 2800;

  _vox_val      = vox_on ? 1 : 0;
  _voxgain_val  = (vox_gain  <= 100U) ? (int32_t)vox_gain  : 50;
  _voxdelay_val = (vox_delay >= 100U && vox_delay <= 2000U) ? (int32_t)vox_delay : 500;
  _cwdec_val    = cw_decode_on ? 1 : 0;

  /* CW settings */
  _cw_pitch_val  = (cw_pitch_hz  >= 300U && cw_pitch_hz  <= 900U)  ? (int32_t)cw_pitch_hz  : 700;
  _cw_wpm_val    = (cw_wpm       >= 5U   && cw_wpm       <= 40U)   ? (int32_t)cw_wpm        : 20;
  _keyer_val     = (keyer_mode   <= 2U)                             ? (int32_t)keyer_mode    : 0;
  _paddlerev_val = paddle_reverse ? 1 : 0;
  _sidetone_val  = (sidetone_vol <= 100U)                           ? (int32_t)sidetone_vol  : 50;
  _bkin_val      = (cw_bkin      <= 2U)                             ? (int32_t)cw_bkin       : 0;
  _bkdelay_val   = (cw_bk_delay_ms >= 50U && cw_bk_delay_ms <= 2000U) ? (int32_t)cw_bk_delay_ms : 200;
  _cwrev_val     = cw_reverse ? 1 : 0;
  _cwfilter_val  = (cw_filter_hz >= 50U && cw_filter_hz <= 500U)   ? (int32_t)cw_filter_hz  : 500;

  /* RF Power: Watts when PA configured, else percent */
  s_pa_watts = pa_watts;
  if (pa_watts > 0U) {
    uint8_t pct = (rf_power_pct > 0U && rf_power_pct <= 100U) ? rf_power_pct : 100U;
    int32_t w   = (int32_t)((uint32_t)pct * pa_watts / 100U);
    if (w < 1) w = 1;
    if (w > (int32_t)pa_watts) w = (int32_t)pa_watts;
    _rfpwr_val                    = w;
    m->items[MENU_IDX_RFPOWER].min    = 1;
    m->items[MENU_IDX_RFPOWER].max    = (int32_t)pa_watts;
    m->items[MENU_IDX_RFPOWER].step   = (pa_watts >= 50U) ? 5 : 1;
    m->items[MENU_IDX_RFPOWER].suffix = "W";
  } else {
    _rfpwr_val                    = (rf_power_pct >= 5U && rf_power_pct <= 100U) ? (int32_t)rf_power_pct : 100;
    m->items[MENU_IDX_RFPOWER].min    = 5;
    m->items[MENU_IDX_RFPOWER].max    = 100;
    m->items[MENU_IDX_RFPOWER].step   = 5;
    m->items[MENU_IDX_RFPOWER].suffix = "%";
  }

  s_apply_cb = apply_cb;
  /* USER CODE END Menu_LoadFromSDR_0 */
}

void Menu_SaveToSDR(Menu_Handle_t *m,
                     uint8_t *agc_speed, bool *nb, bool *nr, int16_t *rit,
                     uint8_t *vol, uint8_t *mic_gain, uint8_t *digi_gain,
                     uint8_t *sq, uint32_t *step, uint32_t *bw_hz,
                     uint8_t *att, uint8_t *band, uint8_t *mode,
                     uint8_t *usb_mode, uint8_t *zoom,
                     bool *ext_alc, uint8_t *rf_power,
                     uint16_t *tx_audio_low_hz, uint16_t *tx_audio_high_hz,
                     int16_t *rx_shift_hz,
                     bool *notch_on, int16_t *notch_hz,
                     bool *vox_on, uint8_t *vox_gain, uint16_t *vox_delay,
                     bool *cw_decode_on,
                     uint16_t *cw_pitch_hz, uint8_t *cw_wpm,
                     uint8_t *keyer_mode, bool *paddle_reverse,
                     uint8_t *sidetone_vol, uint8_t *cw_bkin,
                     uint16_t *cw_bk_delay_ms, bool *cw_reverse,
                     uint16_t *cw_filter_hz,
                     bool *usb_iq_stream)
{
  /* USER CODE BEGIN Menu_SaveToSDR_0 */
  (void)m;
  static const uint32_t sv[6] = {1,10,100,1000,10000,100000};
  *agc_speed = (uint8_t)_agc_val;
  *nb        = (_nb_val   != 0);
  *nr        = (_nr_val   != 0);
  *rit       = (int16_t)_rit_val;
  *rx_shift_hz = (int16_t)_rxshift_val;
  *notch_on    = (_notch_val != 0);
  *notch_hz    = (int16_t)_notchhz_val;
  *vox_on      = (_vox_val != 0);
  *vox_gain    = (uint8_t)(_voxgain_val  >= 0   && _voxgain_val  <= 100  ? _voxgain_val  : 50);
  *vox_delay   = (uint16_t)(_voxdelay_val >= 100 && _voxdelay_val <= 2000 ? _voxdelay_val : 500);
  *vol       = (uint8_t)_vol_val;
  *mic_gain  = (uint8_t)_mic_val;
  *digi_gain = (uint8_t)_digi_val;
  *sq        = (uint8_t)_sq_val;
  *step      = sv[(_step_val >= 0 && _step_val < 6) ? _step_val : 2];
  *bw_hz     = (uint32_t)(_bw_val >= 100 && _bw_val <= 24000 ? _bw_val : 3000);
  *att       = (uint8_t)_att_val;
  *band      = (uint8_t)_band_val;
  *mode      = (uint8_t)_mode_val;
  *usb_mode  = (uint8_t)_usb_val;
  *zoom      = (uint8_t)(_zoom_val >= 0 && _zoom_val < 4 ? _zoom_val : 0);
  *ext_alc   = (_alc_val != 0);
  if (s_pa_watts > 0U) {
    int32_t w = (_rfpwr_val >= 1) ? _rfpwr_val : 1;
    uint32_t pct = (uint32_t)w * 100U / s_pa_watts;
    if (pct < 1U)   pct = 1U;
    if (pct > 100U) pct = 100U;
    *rf_power = (uint8_t)pct;
  } else {
    *rf_power = (uint8_t)(_rfpwr_val >= 5 && _rfpwr_val <= 100 ? _rfpwr_val : 100);
  }
  *tx_audio_low_hz  = (uint16_t)_tx_low_val;
  *tx_audio_high_hz = (uint16_t)_tx_high_val;
  *cw_decode_on     = (_cwdec_val != 0);
  *cw_pitch_hz      = (uint16_t)(_cw_pitch_val >= 300 && _cw_pitch_val <= 900 ? _cw_pitch_val : 700);
  *cw_wpm           = (uint8_t) (_cw_wpm_val   >= 5   && _cw_wpm_val   <= 40  ? _cw_wpm_val   : 20);
  *keyer_mode       = (uint8_t) (_keyer_val     >= 0   && _keyer_val    <= 2   ? _keyer_val    : 0);
  *paddle_reverse   = (_paddlerev_val != 0);
  *sidetone_vol     = (uint8_t) (_sidetone_val  >= 0   && _sidetone_val <= 100 ? _sidetone_val : 50);
  *cw_bkin          = (uint8_t) (_bkin_val      >= 0   && _bkin_val     <= 2   ? _bkin_val     : 0);
  *cw_bk_delay_ms   = (uint16_t)(_bkdelay_val   >= 50  && _bkdelay_val  <= 2000 ? _bkdelay_val : 200);
  *cw_reverse       = (_cwrev_val != 0);
  *cw_filter_hz     = (uint16_t)(_cwfilter_val  >= 50  && _cwfilter_val <= 500  ? _cwfilter_val : 500);
  *usb_iq_stream    = (_iq_stream_val != 0);
  /* USER CODE END Menu_SaveToSDR_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
