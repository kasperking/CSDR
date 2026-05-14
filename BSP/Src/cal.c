/* USER CODE BEGIN Header */
/**
  * @file  cal.c
  * @brief Calibration overlay menu – two-level scanline renderer.
  *
  *  Navigation (same keys as main menu):
  *   ENC rotate  – scroll / change value
  *   ENC press   – enter section / begin edit
  *   F1          – value up  (when editing)
  *   F2          – value down (when editing)
  *   F4          – back / exit without save
  *   "Save Settings" item – exit and signal caller to persist
  */
/* USER CODE END Header */

#include "cal.h"
#include "sdr_ui.h"
#include "encoder.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ── Key sampling (active-low, same GPIO layout as csdr_app.c) ────────── */
extern TIM_HandleTypeDef htim1;   /* encoder timer (TIM1_CH1/CH2) */

#define KEY_ENC_SW_Port  GPIOA
#define KEY_ENC_SW_Pin   GPIO_PIN_2
#define KEY_F1_Port      GPIOE
#define KEY_F1_Pin       GPIO_PIN_10
#define KEY_F2_Port      GPIOE
#define KEY_F2_Pin       GPIO_PIN_11
#define KEY_F4_Port      GPIOE
#define KEY_F4_Pin       GPIO_PIN_13

/* Key_t instances are declared locally in each blocking loop. */

/* ── Scanline helpers ───────────────────────────────────────────────────── */
#define LN   ST7789_GetLineBuf()

#define CAL_X        10U
#define CAL_W       300U
#define CAL_Y        ZONE_SPEC_Y      /* overlay on spectrum zone */
#define CAL_ITEM_H   15U              /* px per row               */
#define CAL_VISIBLE   7U              /* rows visible: 16hdr+7*15=121px <= 130px avail */

#define CAL_BG        0x0843U   /* Dark blue-gray     */
#define CAL_SEL_BG    0xF800   /* Teal highlight     */
#define CAL_HDR_BG    0xF800   /* Dark blue-gray     */
#define CAL_SECT_BG   0x0208U
#define CAL_LBL       0x8410U
#define CAL_VAL       0xFFE0U
#define CAL_EDIT_VAL  0x07FFU
#define CAL_ACTION    0xF81FU
#define CAL_BORDER    0x10A2U   /* Dark subtle border */
#define CAL_SAVE_BG   0x0400U
#define CAL_SAVE_FG   0x07E0U

static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

static void push_ln(ST7789_Handle_t *lcd, uint16_t y)
{
  ST7789_PushScanline(lcd, y, LN);
  dma_wait_pub(lcd);
  cs_high_pub(lcd);
}

/* ── Data model ─────────────────────────────────────────────────────────── */

typedef enum {
  CAL_T_INT,      /* integer with min/max/step */
  CAL_T_ACTION,   /* immediate action          */
  CAL_T_BACK,     /* "Exit" within a section   */
} CalItemType_t;

typedef struct {
  const char   *label;
  CalItemType_t type;
  int32_t       min, max, step;
  int32_t      *val;        /* NULL for actions */
} CalItem_t;

typedef struct {
  const char   *title;
  const CalItem_t *items;
  uint8_t       count;
} CalSection_t;

/* ── Value storage ─ mirrors Cal_Params_t fields for live editing ───────── */
static int32_t v_xtal_ppm;
static int32_t v_iq_gain;
static int32_t v_iq_phase;
static int32_t v_dc_i;
static int32_t v_dc_q;
static int32_t v_audio_gain;
static int32_t v_mic_gain;
static int32_t v_smeter_off;
static int32_t v_lo_offset;

/* ── Section item tables ────────────────────────────────────────────────── */
static const CalItem_t items_freq[] = {
  { "XTAL PPM",      CAL_T_INT,  -200,   200,     1, &v_xtal_ppm  },
  { "Apply",         CAL_T_ACTION, 0,0,0,            NULL         },
  { "Exit",          CAL_T_BACK,   0,0,0,            NULL         },
};

static const CalItem_t items_iq[] = {
  { "IQ Gain",       CAL_T_INT,    -50,    50,     1, &v_iq_gain   },
  { "IQ Phase",      CAL_T_INT,    -50,    50,     1, &v_iq_phase  },
  { "Auto IQ Cal",   CAL_T_ACTION, 0,0,0,            NULL         },
  { "Exit",          CAL_T_BACK,   0,0,0,            NULL         },
};

static const CalItem_t items_dc[] = {
  { "DC I Offset",   CAL_T_INT, -2048,  2048,     1, &v_dc_i      },
  { "DC Q Offset",   CAL_T_INT, -2048,  2048,     1, &v_dc_q      },
  { "Auto DC Cal",   CAL_T_ACTION, 0,0,0,            NULL         },
  { "Exit",          CAL_T_BACK,   0,0,0,            NULL         },
};

static const CalItem_t items_audio[] = {
  { "Audio Gain dB", CAL_T_INT,    -20,    20,     1, &v_audio_gain},
  { "Mic Gain",      CAL_T_INT,      0,   100,     1, &v_mic_gain  },
  { "Exit",          CAL_T_BACK,   0,0,0,            NULL         },
};

static const CalItem_t items_rf[] = {
  { "S-Meter Offs",  CAL_T_INT,    -20,    20,     1, &v_smeter_off},
  { "LO Offset Hz",  CAL_T_INT,  10000, 25000,   100, &v_lo_offset },
  { "Exit",          CAL_T_BACK,   0,0,0,            NULL         },
};

static const CalSection_t s_sections[] = {
  { "Frequency Cal",   items_freq,  3U },
  { "IQ Calibration",  items_iq,    4U },
  { "DC Offset",       items_dc,    4U },
  { "Audio Cal",       items_audio, 3U },
  { "RF / Display Cal",items_rf,    3U },
};
#define SECTION_COUNT  5U

/* Top-level item types */
#define TOP_SECT   0   /* enter section submenu */
#define TOP_SAVE   1
#define TOP_LOAD   2
#define TOP_RESET  3
#define TOP_EXIT   4

typedef struct { const char *label; int kind; } TopItem_t;
static const TopItem_t s_top[] = {
  { "Frequency Cal",    TOP_SECT  },
  { "IQ Calibration",  TOP_SECT  },
  { "DC Offset",        TOP_SECT  },
  { "Audio Cal",        TOP_SECT  },
  { "RF / Display Cal", TOP_SECT  },
  { "Save Settings",    TOP_SAVE  },
  { "Load Settings",    TOP_LOAD  },
  { "Reset Default",    TOP_RESET },
  { "Exit Calibration", TOP_EXIT  },
};
#define TOP_COUNT  9U

/* ── Rendering ──────────────────────────────────────────────────────────── */

static void render_header(ST7789_Handle_t *lcd, const char *title, uint16_t *y)
{
  for (uint16_t fr = 0U; fr < 16U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, CAL_X, CAL_W,
                 (fr == 0U || fr == 15U) ? CAL_BORDER : CAL_HDR_BG);
    ln[CAL_X]            = sw16(CAL_BORDER);
    ln[CAL_X + CAL_W - 1U] = sw16(CAL_BORDER);
    if (fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      uint16_t tx = (uint16_t)(CAL_X + (CAL_W - (uint16_t)strlen(title) *
                                Font6x8.width) / 2U);
      LCD_LineStr(ln, tx, fr - 4U, title, &Font6x8, 0xFFFFU, CAL_HDR_BG);
    }
    push_ln(lcd, (*y)++);
  }
}

static void render_top_item(ST7789_Handle_t *lcd, uint8_t idx,
                             uint8_t cursor, uint16_t abs_y)
{
  const TopItem_t *it = &s_top[idx];
  bool sel = (idx == cursor);
  uint16_t bg = sel ? CAL_SEL_BG : CAL_BG;
  uint16_t fg = 0xFFFFU;
  if (it->kind == TOP_SAVE)  { bg = sel ? sw16(CAL_SAVE_BG) : CAL_BG; fg = sel ? CAL_SAVE_FG : CAL_ACTION; }
  if (it->kind == TOP_LOAD)  { fg = sel ? 0xFFFFU : CAL_ACTION; }
  if (it->kind == TOP_RESET) { fg = sel ? 0xFFFFU : 0xF800U; }
  if (it->kind == TOP_EXIT)  { fg = sel ? 0xFFFFU : CAL_LBL; }
  if (it->kind == TOP_SECT)  { fg = sel ? 0xFFFFU : CAL_VAL; }

  for (uint16_t fr = 0U; fr < CAL_ITEM_H; fr++) {
    uint16_t *ln = LN;
    bool top = (fr == 0U), bot = (fr == CAL_ITEM_H - 1U);
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, CAL_X, CAL_W, (top || bot) ? CAL_BORDER : bg);
    if (!top && !bot) {
      ln[CAL_X]              = sw16(CAL_BORDER);
      ln[CAL_X + CAL_W - 1U] = sw16(CAL_BORDER);
    }
    if (!top && !bot && fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      uint16_t row = fr - 4U;
      /* Indent section items */
      uint16_t lx = (uint16_t)(CAL_X + (it->kind == TOP_SECT ? 4U : 8U));
      LCD_LineStr(ln, lx, row, it->label, &Font6x8, fg, bg);
      if (it->kind == TOP_SECT) {
        LCD_LineStr(ln, (uint16_t)(CAL_X + CAL_W - 16U), row,
                    "->", &Font6x8, CAL_BORDER, bg);
      }
    }
    push_ln(lcd, abs_y + fr);
  }
}

static void render_sub_item(ST7789_Handle_t *lcd, const CalItem_t *it,
                             uint8_t idx, uint8_t cursor, bool editing,
                             uint16_t abs_y)
{
  bool sel  = (idx == cursor);
  bool edit = sel && editing && (it->type == CAL_T_INT);
  uint16_t bg  = sel ? CAL_SEL_BG : CAL_BG;
  if (it->type == CAL_T_BACK)   { bg = sel ? 0x8000U : CAL_BG; }
  if (it->type == CAL_T_ACTION) { bg = sel ? 0x0010U : CAL_BG; }

  char val_s[16] = "";
  if (it->type == CAL_T_INT && it->val)
    snprintf(val_s, sizeof(val_s), "%ld", (long)*it->val);
  else if (it->type == CAL_T_ACTION)
    snprintf(val_s, sizeof(val_s), ">> RUN");
  else if (it->type == CAL_T_BACK)
    snprintf(val_s, sizeof(val_s), "< Back");

  for (uint16_t fr = 0U; fr < CAL_ITEM_H; fr++) {
    uint16_t *ln = LN;
    bool top = (fr == 0U), bot = (fr == CAL_ITEM_H - 1U);
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, CAL_X, CAL_W, (top || bot) ? CAL_BORDER : bg);
    if (!top && !bot) {
      ln[CAL_X]              = sw16(CAL_BORDER);
      ln[CAL_X + CAL_W - 1U] = sw16(CAL_BORDER);
    }
    if (!top && !bot && fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      uint16_t row = fr - 4U;
      if (it->type == CAL_T_INT) {
        LCD_LineStr(ln, (uint16_t)(CAL_X + 4U), row,
                    it->label, &Font6x8, CAL_LBL, bg);
        uint16_t vc = edit ? CAL_EDIT_VAL : CAL_VAL;
        LCD_LineStr(ln, (uint16_t)(CAL_X + CAL_W - 56U), row,
                    val_s, &Font6x8, vc, bg);
      } else {
        uint16_t fc = (it->type == CAL_T_BACK) ? 0xFFFFU : CAL_ACTION;
        uint16_t tx = (uint16_t)(CAL_X + (CAL_W -
                       (uint16_t)strlen(val_s) * Font6x8.width) / 2U);
        LCD_LineStr(ln, tx, row, val_s, &Font6x8, fc, bg);
      }
    }
    push_ln(lcd, abs_y + fr);
  }
}

/* ── Auto-calibration stubs ─────────────────────────────────────────────── */
static void auto_iq_cal(ST7789_Handle_t *lcd)
{
  /* Stub: zero IQ errors */
  v_iq_gain  = 0;
  v_iq_phase = 0;

  uint16_t y = (uint16_t)(CAL_Y + 60U);
  for (uint16_t fr = 0U; fr < 20U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, CAL_X, CAL_W,
                 (fr == 0U || fr == 19U) ? CAL_BORDER : CAL_SEL_BG);
    if (fr >= 6U && fr < 6U + (uint16_t)Font6x8.height)
      LCD_LineStr(ln, (uint16_t)(CAL_X + 60U), fr - 6U,
                  "Auto IQ Cal: done", &Font6x8, 0xFFFFU, CAL_SEL_BG);
    push_ln(lcd, y + fr);
  }
  HAL_Delay(1200);
}

static void auto_dc_cal(ST7789_Handle_t *lcd)
{
  v_dc_i = 0;
  v_dc_q = 0;

  uint16_t y = (uint16_t)(CAL_Y + 60U);
  for (uint16_t fr = 0U; fr < 20U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, CAL_X, CAL_W,
                 (fr == 0U || fr == 19U) ? CAL_BORDER : CAL_SEL_BG);
    if (fr >= 6U && fr < 6U + (uint16_t)Font6x8.height)
      LCD_LineStr(ln, (uint16_t)(CAL_X + 60U), fr - 6U,
                  "Auto DC Cal: done", &Font6x8, 0xFFFFU, CAL_SEL_BG);
    push_ln(lcd, y + fr);
  }
  HAL_Delay(1000);
}

/* ── Encoder delta helper ────────────────────────────────────────────────── */
static int32_t enc_read_delta(void)
{
  static uint32_t s_last = 0U;
  uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim1);
  int32_t  d   = (int32_t)(cnt - s_last);
  if (d >  2) { s_last = cnt; return  1; }
  if (d < -2) { s_last = cnt; return -1; }
  return 0;
}

/* ── Sub-level loop ─────────────────────────────────────────────────────── */
static void render_sublevel(ST7789_Handle_t *lcd, uint8_t sect_idx,
                             uint8_t cursor, bool editing, uint8_t scroll)
{
  const CalSection_t *sec = &s_sections[sect_idx];
  uint16_t y = (uint16_t)CAL_Y;
  render_header(lcd, sec->title, &y);
  for (uint8_t r = 0U; r < CAL_VISIBLE; r++) {
    uint8_t idx = scroll + r;
    if (idx >= sec->count) break;
    render_sub_item(lcd, &sec->items[idx], idx, cursor, editing, y);
    y += CAL_ITEM_H;
  }
}

static void run_section(ST7789_Handle_t *lcd, uint8_t sect_idx)
{
  const CalSection_t *sec = &s_sections[sect_idx];
  uint8_t cursor  = 0U;
  uint8_t scroll  = 0U;
  bool    editing = false;

  Key_t k_enc = {0}, k_f1 = {0}, k_f2 = {0}, k_f4 = {0};
  Key_Init(&k_enc, KEY_ENC_SW_Port, KEY_ENC_SW_Pin);
  Key_Init(&k_f1,  KEY_F1_Port,     KEY_F1_Pin);
  Key_Init(&k_f2,  KEY_F2_Port,     KEY_F2_Pin);
  Key_Init(&k_f4,  KEY_F4_Port,     KEY_F4_Pin);

  render_sublevel(lcd, sect_idx, cursor, editing, scroll);

  for (;;) {
    Key_Poll(&k_enc); Key_Poll(&k_f1); Key_Poll(&k_f2); Key_Poll(&k_f4);

    /* Encoder rotation */
    int32_t d = enc_read_delta();
    if (d != 0) {
      if (editing && sec->items[cursor].type == CAL_T_INT) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v += d * it->step;
        if (*v < it->min) *v = it->min;
        if (*v > it->max) *v = it->max;
      } else {
        if (d > 0 && cursor < sec->count - 1U) { cursor++; }
        if (d < 0 && cursor > 0U)              { cursor--; }
        if (cursor < scroll)                   { scroll = cursor; }
        if (cursor >= scroll + CAL_VISIBLE)    { scroll = (uint8_t)(cursor - CAL_VISIBLE + 1U); }
      }
      render_sublevel(lcd, sect_idx, cursor, editing, scroll);
    }

    /* ENC press: toggle edit / confirm action / back */
    if (Key_Press(&k_enc)) {
      const CalItem_t *it = &sec->items[cursor];
      if (it->type == CAL_T_INT) {
        editing = !editing;
      } else if (it->type == CAL_T_BACK) {
        return;
      } else if (it->type == CAL_T_ACTION) {
        if (sect_idx == 1U && cursor == 2U) auto_iq_cal(lcd);
        if (sect_idx == 2U && cursor == 2U) auto_dc_cal(lcd);
      }
      render_sublevel(lcd, sect_idx, cursor, editing, scroll);
    }

    /* F1 = value up (hold-repeat while editing) */
    if (Key_PressOrRepeat(&k_f1)) {
      if (editing && sec->items[cursor].type == CAL_T_INT) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v += it->step; if (*v > it->max) *v = it->max;
        render_sublevel(lcd, sect_idx, cursor, editing, scroll);
      }
    }

    /* F2 = value down (hold-repeat while editing) */
    if (Key_PressOrRepeat(&k_f2)) {
      if (editing && sec->items[cursor].type == CAL_T_INT) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v -= it->step; if (*v < it->min) *v = it->min;
        render_sublevel(lcd, sect_idx, cursor, editing, scroll);
      }
    }

    /* F4 = exit edit mode / back */
    if (Key_Press(&k_f4)) {
      if (editing) { editing = false; render_sublevel(lcd, sect_idx, cursor, editing, scroll); }
      else         { return; }
    }
  }
}

/* ── Top-level loop ─────────────────────────────────────────────────────── */
static void render_toplevel(ST7789_Handle_t *lcd, uint8_t cursor, uint8_t scroll)
{
  uint16_t y = (uint16_t)CAL_Y;
  render_header(lcd, " -= CALIBRATION =- ", &y);
  for (uint8_t r = 0U; r < CAL_VISIBLE; r++) {
    uint8_t idx = scroll + r;
    if (idx >= TOP_COUNT) break;
    render_top_item(lcd, idx, cursor, y);
    y += CAL_ITEM_H;
  }
}

/* ── Cal_Run ────────────────────────────────────────────────────────────── */
bool Cal_Run(ST7789_Handle_t *lcd, Cal_Params_t *params)
{
  /* Copy params into working storage */
  v_xtal_ppm   = params->xtal_ppm;
  v_iq_gain    = (int32_t)params->iq_gain;
  v_iq_phase   = (int32_t)params->iq_phase;
  v_dc_i       = params->dc_i_offset;
  v_dc_q       = params->dc_q_offset;
  v_audio_gain = (int32_t)params->audio_gain_db;
  v_mic_gain   = (int32_t)params->mic_gain;
  v_smeter_off = (int32_t)params->smeter_offset_db;
  v_lo_offset  = (int32_t)params->lo_offset_hz;

  uint8_t cursor = 0U;
  uint8_t scroll = 0U;
  Key_t k_enc = {0}, k_f4 = {0};
  Key_Init(&k_enc, KEY_ENC_SW_Port, KEY_ENC_SW_Pin);
  Key_Init(&k_f4,  KEY_F4_Port,     KEY_F4_Pin);

  render_toplevel(lcd, cursor, scroll);

  for (;;) {
    Key_Poll(&k_enc); Key_Poll(&k_f4);

    int32_t d = enc_read_delta();
    if (d != 0) {
      if (d > 0 && cursor < TOP_COUNT - 1U) cursor++;
      if (d < 0 && cursor > 0U)             cursor--;
      if (cursor < scroll)                  scroll = cursor;
      if (cursor >= scroll + CAL_VISIBLE)   scroll = (uint8_t)(cursor - CAL_VISIBLE + 1U);
      render_toplevel(lcd, cursor, scroll);
    }

    if (Key_Press(&k_enc)) {
      const TopItem_t *it = &s_top[cursor];

      if (it->kind == TOP_SECT) {
        run_section(lcd, (uint8_t)cursor);
        render_toplevel(lcd, cursor, scroll);

      } else if (it->kind == TOP_SAVE) {
        params->xtal_ppm        = v_xtal_ppm;
        params->iq_gain         = (int16_t)v_iq_gain;
        params->iq_phase        = (int16_t)v_iq_phase;
        params->dc_i_offset     = v_dc_i;
        params->dc_q_offset     = v_dc_q;
        params->audio_gain_db   = (int16_t)v_audio_gain;
        params->mic_gain        = (int16_t)v_mic_gain;
        params->smeter_offset_db= (int16_t)v_smeter_off;
        params->lo_offset_hz    = (uint32_t)v_lo_offset;
        return true;

      } else if (it->kind == TOP_LOAD) {
        /* Restore caller-supplied values (reload from flash is caller's job) */
        v_xtal_ppm   = params->xtal_ppm;
        v_iq_gain    = (int32_t)params->iq_gain;
        v_iq_phase   = (int32_t)params->iq_phase;
        v_dc_i       = params->dc_i_offset;
        v_dc_q       = params->dc_q_offset;
        v_audio_gain = (int32_t)params->audio_gain_db;
        v_mic_gain   = (int32_t)params->mic_gain;
        v_smeter_off = (int32_t)params->smeter_offset_db;
        v_lo_offset  = (int32_t)params->lo_offset_hz;
        render_toplevel(lcd, cursor, scroll);

      } else if (it->kind == TOP_RESET) {
        Cal_Params_t def = CAL_PARAMS_DEFAULT;
        v_xtal_ppm   = def.xtal_ppm;
        v_iq_gain    = def.iq_gain;
        v_iq_phase   = def.iq_phase;
        v_dc_i       = def.dc_i_offset;
        v_dc_q       = def.dc_q_offset;
        v_audio_gain = def.audio_gain_db;
        v_mic_gain   = def.mic_gain;
        v_smeter_off = def.smeter_offset_db;
        v_lo_offset  = (int32_t)def.lo_offset_hz;
        render_toplevel(lcd, cursor, scroll);

      } else { /* TOP_EXIT */
        return false;
      }
    }

    if (Key_Press(&k_f4)) {
      return false;
    }
  }
}
