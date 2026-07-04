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
#include "input_scan.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "bpf_lpf.h"
#include "w25q.h"
#include "csdr_app.h"
#include "pe4302.h"     /* g_att — display path adds back front-end attenuation */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* ── Key sampling ────────────────────────────────────────────────────────
 * ENC_SW: direct MCU input (PB3) — use main.h macros.
 * F1/F2/F4: PCA9555 expander — use Key_InitPCA with g_pca9555_raw cache. */
extern TIM_HandleTypeDef htim3;   /* encoder timer (TIM3_CH1/CH2 = PB4/PB5) */

/* DSP pointer set by Cal_Run — used by all auto-cal routines */
static DSP_State_t *s_dsp;

/* Key_t instances are declared locally in each blocking loop. */

/* ── Scanline helpers ───────────────────────────────────────────────────── */
#define LN   LCD_GetLineBuf()

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

static void push_ln(uint16_t y)
{
  LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, LN, LCD_W);
}

/* ── Data model ─────────────────────────────────────────────────────────── */

typedef enum {
  CAL_T_INT,       /* integer with min/max/step                           */
  CAL_T_FLOAT10,   /* integer stored ×10, displayed as "X.YA" (no float) */
  CAL_T_ACTION,    /* immediate action                                    */
  CAL_T_BACK,      /* "Exit" within a section                             */
  CAL_T_ENUM,      /* cycles val in [0,max]; choices[val] displayed       */
} CalItemType_t;

typedef struct {
  const char        *label;
  CalItemType_t      type;
  int32_t            min, max, step;
  int32_t           *val;        /* NULL for actions */
  const char *const *choices;    /* CAL_T_ENUM: label per index; NULL otherwise */
} CalItem_t;

typedef struct {
  const char   *title;
  const CalItem_t *items;
  uint8_t       count;
} CalSection_t;

/* ── PA watts ↔ index conversion ───────────────────────────────────────── */
static const uint8_t pa_watts_table[] = { 0U, 20U, 45U, 100U };
static uint8_t pa_idx_to_watts(int32_t idx)
{
  if (idx < 0 || idx > 3) return 0U;
  return pa_watts_table[(uint8_t)idx];
}
static int32_t pa_watts_to_idx(uint8_t w)
{
  for (int32_t i = 0; i < 4; i++)
    if (pa_watts_table[i] == w) return i;
  return 0;
}

/* ── Band cal working storage ────────────────────────────────────────────── */
static int32_t v_band_rx_gain;
static int32_t v_band_nf_off;
static int32_t v_band_tx_drive;
static int32_t v_band_swr_scale;
static char    s_band_cal_title[24] = "Band Cal";

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
static int32_t v_pa_idx;
static int32_t v_oc_idx;    /* 0=2.0A  1=2.5A  2=3.0A  3=3.5A  4=4.0A */
static int32_t v_pwr_scale; /* tandem-match FWD power cal 50..200 %   */

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
  { "S-Meter Offs",  CAL_T_INT,    -60,    60,     1, &v_smeter_off},
  { "LO Offset Hz",  CAL_T_INT,      0, 25000,   500, &v_lo_offset },
  { "Auto S-Meter",  CAL_T_ACTION, 0,0,0,            NULL         },
  { "Auto Noise Flr",CAL_T_ACTION, 0,0,0,            NULL         },
  { "Auto AGC Ref",  CAL_T_ACTION, 0,0,0,            NULL         },
  { "Exit",          CAL_T_BACK,   0,0,0,            NULL         },
};

static const char *const pa_choices[] = { "None", "20W", "45W", "100W" };
static const CalItem_t items_hw[] = {
  { "PA Power",  CAL_T_ENUM,     0,   3, 1, &v_pa_idx,     pa_choices },
  { "OC Limit",  CAL_T_FLOAT10, 10, 200, 1, &v_oc_idx,     NULL       },
  { "PWR Scale %",CAL_T_INT,    50, 200, 1, &v_pwr_scale,  NULL       },
  { "Exit",      CAL_T_BACK,    0,   0, 0, NULL,           NULL       },
};

static const CalItem_t items_band[] = {
  { "RX Gain Trim",  CAL_T_INT,    -20,  20,  1, &v_band_rx_gain,   NULL },
  { "Noise Flr Off", CAL_T_INT,    -20,  20,  1, &v_band_nf_off,    NULL },
  { "TX Drive Trim", CAL_T_INT,    -50,  50,  1, &v_band_tx_drive,  NULL },
  { "SWR Scale %",   CAL_T_INT,     50, 200,  1, &v_band_swr_scale, NULL },
  { "Auto Noise",    CAL_T_ACTION,   0,   0,  0, NULL,              NULL },
  { "Save Band Cal", CAL_T_ACTION,   0,   0,  0, NULL,              NULL },
  { "Exit",          CAL_T_BACK,     0,   0,  0, NULL,              NULL },
};

/* s_sections is non-const so the band entry title can be updated at runtime */
static CalSection_t s_sections[] = {
  { "Frequency Cal",   items_freq,  3U },
  { "IQ Calibration",  items_iq,    4U },
  { "DC Offset",       items_dc,    4U },
  { "Audio Cal",       items_audio, 3U },
  { "RF / Display Cal",items_rf,    6U },
  { "PA Hardware",     items_hw,    4U },
  { s_band_cal_title,  items_band,  7U },
};
#define SECTION_COUNT  7U

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
  { "PA Hardware",      TOP_SECT  },
  { "Band Cal",         TOP_SECT  },
  { "Save Settings",    TOP_SAVE  },
  { "Load Settings",    TOP_LOAD  },
  { "Reset Default",    TOP_RESET },
  { "Exit Calibration", TOP_EXIT  },
};
#define TOP_COUNT  11U

/* ── Rendering ──────────────────────────────────────────────────────────── */

static void render_header(const char *title, uint16_t *y)
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
    push_ln((*y)++);
  }
}

static void render_top_item(uint8_t idx, uint8_t cursor, uint16_t abs_y)
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
    push_ln(abs_y + fr);
  }
}

static void render_sub_item(const CalItem_t *it, uint8_t idx,
                             uint8_t cursor, bool editing, uint16_t abs_y)
{
  bool sel  = (idx == cursor);
  bool edit = sel && editing && (it->type == CAL_T_INT || it->type == CAL_T_FLOAT10 ||
                                 it->type == CAL_T_ENUM);
  uint16_t bg  = sel ? CAL_SEL_BG : CAL_BG;
  if (it->type == CAL_T_BACK)   { bg = sel ? 0x8000U : CAL_BG; }
  if (it->type == CAL_T_ACTION) { bg = sel ? 0x0010U : CAL_BG; }

  char val_s[16] = "";
  if (it->type == CAL_T_INT && it->val)
    snprintf(val_s, sizeof(val_s), "%ld", (long)*it->val);
  else if (it->type == CAL_T_FLOAT10 && it->val) {
    int32_t v = *it->val;
    snprintf(val_s, sizeof(val_s), "%ld.%ldA", (long)(v / 10), (long)(v % 10));
  } else if (it->type == CAL_T_ENUM && it->val && it->choices)
    snprintf(val_s, sizeof(val_s), "%s", it->choices[*it->val]);
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
      if (it->type == CAL_T_INT || it->type == CAL_T_FLOAT10 || it->type == CAL_T_ENUM) {
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
    push_ln(abs_y + fr);
  }
}

/* ── Auto-calibration helpers ───────────────────────────────────────────── */

/* Progress bar toast: fill bar grows left-to-right over total_ms.
 * Call in a polling loop; re-renders at most every ~50 ms. */
static uint32_t s_last_render_tick = 0U;
static void render_cal_progress(const char *msg,
                                 uint32_t elapsed_ms, uint32_t total_ms)
{
  uint32_t now = HAL_GetTick();
  if (now - s_last_render_tick < 50U) return;
  s_last_render_tick = now;

  uint8_t pct = (uint8_t)(elapsed_ms >= total_ms ? 100U
                           : elapsed_ms * 100U / total_ms);
  uint16_t bar_w = (uint16_t)((uint32_t)pct * (uint32_t)(CAL_W - 20U) / 100U);
  uint16_t y = (uint16_t)(CAL_Y + 60U);
  for (uint16_t fr = 0U; fr < 28U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    bool edge = (fr == 0U || fr == 27U);
    LCD_LineFill(ln, CAL_X, CAL_W, edge ? CAL_BORDER : CAL_SEL_BG);
    if (!edge) {
      ln[CAL_X]              = sw16(CAL_BORDER);
      ln[CAL_X + CAL_W - 1U] = sw16(CAL_BORDER);
    }
    if (!edge && fr >= 4U && fr < 4U + (uint16_t)Font6x8.height)
      LCD_LineStr(ln, (uint16_t)(CAL_X + 8U), fr - 4U,
                  msg, &Font6x8, 0xFFFFU, CAL_SEL_BG);
    if (fr >= 17U && fr < 23U)   /* green progress bar */
      LCD_LineFill(ln, (uint16_t)(CAL_X + 10U), bar_w, 0x07E0U);
    push_ln(y + fr);
  }
}

/* Result toast: two text lines + 1.4 s pause. */
static void render_cal_result(const char *line1, const char *line2)
{
  uint16_t y = (uint16_t)(CAL_Y + 60U);
  for (uint16_t fr = 0U; fr < 28U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    bool edge = (fr == 0U || fr == 27U);
    LCD_LineFill(ln, CAL_X, CAL_W, edge ? CAL_BORDER : CAL_SAVE_BG);
    if (!edge) {
      ln[CAL_X]              = sw16(CAL_BORDER);
      ln[CAL_X + CAL_W - 1U] = sw16(CAL_BORDER);
    }
    if (!edge && fr >= 4U && fr < 4U + (uint16_t)Font6x8.height)
      LCD_LineStr(ln, (uint16_t)(CAL_X + 8U), fr - 4U,
                  line1, &Font6x8, CAL_SAVE_FG, CAL_SAVE_BG);
    if (!edge && fr >= 15U && fr < 15U + (uint16_t)Font6x8.height)
      LCD_LineStr(ln, (uint16_t)(CAL_X + 8U), fr - 15U,
                  line2, &Font6x8, 0xFFFFU, CAL_SAVE_BG);
    push_ln(y + fr);
  }
  HAL_Delay(1400U);
}

/* ── RX DC Offset auto-cal ──────────────────────────────────────────────── */
static void auto_dc_cal(void)
{
  if (!s_dsp) return;
  s_last_render_tick = 0U;

  /* 8192 samples ≈ 170 ms at 48 kHz — enough for a stable mean */
  DSP_CalStart(s_dsp, DSP_CAL_DC, 8192U);
  uint32_t t0 = HAL_GetTick();
  DSP_CalMeas_t res;
  bool timed_out = false;
  while (!DSP_CalPoll(s_dsp, &res)) {
    CSDR_ProcessAudioPending();
    uint32_t e = HAL_GetTick() - t0;
    render_cal_progress("DC Cal: measuring...", e, 200U);
    if (e > 3000U) { timed_out = true; break; }
  }

  if (timed_out) {
    render_cal_result("DC Cal: TIMEOUT", "Check SAI/DMA");
    return;
  }

  int32_t di = (int32_t)res.result_dc_i;
  int32_t dq = (int32_t)res.result_dc_q;
  if (di < -2048) di = -2048;
  if (di >  2048) di =  2048;
  if (dq < -2048) dq = -2048;
  if (dq >  2048) dq =  2048;

  v_dc_i = di;
  v_dc_q = dq;
  DSP_SetDCOffset(s_dsp, di, dq);

  char buf[40];
  snprintf(buf, sizeof(buf), "I=%+ld  Q=%+ld  ADC counts", (long)di, (long)dq);
  render_cal_result("DC Cal done:", buf);
}

/* ── RX IQ Balance auto-cal ─────────────────────────────────────────────── */
static void auto_iq_cal(void)
{
  if (!s_dsp) return;
  s_last_render_tick = 0U;

  /* Prompt: user must have a real signal present */
  render_cal_progress("IQ Cal: tune to signal", 0U, 1U);
  HAL_Delay(600U);

  /* 16384 samples ≈ 341 ms — enough to average out noise */
  DSP_CalStart(s_dsp, DSP_CAL_IQ, 16384U);
  uint32_t t0 = HAL_GetTick();
  DSP_CalMeas_t res;
  bool timed_out = false;
  while (!DSP_CalPoll(s_dsp, &res)) {
    CSDR_ProcessAudioPending();
    uint32_t e = HAL_GetTick() - t0;
    render_cal_progress("IQ Cal: measuring...", e, 380U);
    if (e > 3000U) { timed_out = true; break; }
  }

  if (timed_out) {
    render_cal_result("IQ Cal: TIMEOUT", "Check SAI/DMA");
    return;
  }

  /* Reject if no signal present (rms_i < -40 dBFS ≈ 0.01) */
  float cnt   = (float)res.n_count;
  float rms_i = (cnt > 0.0f) ? sqrtf(res.acc_ii / cnt) : 0.0f;
  if (rms_i < 0.01f) {
    render_cal_result("IQ Cal: no signal!", "Need signal > -40dBFS");
    return;
  }

  /* Clamp to ±50 milli / ±50 mrad working range */
  int32_t g = (int32_t)res.result_iq_gain;
  int32_t p = (int32_t)res.result_iq_phase;
  if (g < -50) g = -50;
  if (g >  50) g =  50;
  if (p < -50) p = -50;
  if (p >  50) p =  50;

  v_iq_gain  = g;
  v_iq_phase = p;
  DSP_SetIQCorr(s_dsp, (int16_t)v_iq_gain, (int16_t)v_iq_phase);

  char buf[40];
  snprintf(buf, sizeof(buf), "Gain=%+ld  Phase=%+ld mrad", (long)g, (long)p);
  render_cal_result("IQ Cal done:", buf);
}

/* ── Noise Floor measurement (display only) ─────────────────────────────── */
static void auto_noise_floor(void)
{
  if (!s_dsp) return;
  s_last_render_tick = 0U;

  /* Sample signal_power_db for 2 s (IIR α=0.1 per sample → well settled) */
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 2000U)
    render_cal_progress("Noise: sampling (no sig)", HAL_GetTick() - t0, 2000U);

  int16_t floor_i = (int16_t)s_dsp->signal_power_db;
  char buf[40];
  snprintf(buf, sizeof(buf), "Floor: %d dBFS", (int)floor_i);
  render_cal_result("Noise Floor:", buf);
}

/* ── S-meter Zero auto-cal ──────────────────────────────────────────────── */
static void auto_smeter_zero(void)
{
  if (!s_dsp) return;
  s_last_render_tick = 0U;

  /* Measure noise floor — disconnect antenna or use terminated input */
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 2000U)
    render_cal_progress("S-Meter: remove antenna", HAL_GetTick() - t0, 2000U);

  float floor_db = s_dsp->signal_power_db;

  /* Want noise to read S0 (bars=0).  Display formula (antenna-referenced):
   *   bars = (signal_db + att + smeter_offset_db + 73) / 3
   * For bars=0 at floor_db: offset = -(floor_db + att + 73).
   * The att term must match the display path or the offset absorbs
   * whatever attenuation happens to be engaged during calibration. */
  float att_db = (float)g_att.current_atten_x2 * 0.5f;
  int16_t offset = (int16_t)(-(floor_db + att_db + 73.0f));
  if (offset < -60) offset = -60;
  if (offset >  60) offset =  60;

  v_smeter_off = (int32_t)offset;

  char buf[40];
  snprintf(buf, sizeof(buf), "Flr=%d dBFS  Off=%+d dB",
           (int)(int16_t)floor_db, (int)offset);
  render_cal_result("S-Meter zeroed:", buf);
}

/* ── AGC Reference auto-cal ─────────────────────────────────────────────── */
static void auto_agc_ref(void)
{
  if (!s_dsp) return;
  s_last_render_tick = 0U;

  /* Measure noise floor (same 2 s settle) */
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 2000U)
    render_cal_progress("AGC: sampling floor...", HAL_GetTick() - t0, 2000U);

  float noise_db  = s_dsp->signal_power_db;
  float noise_rms = powf(10.0f, noise_db / 20.0f);

  /* Set max_gain so that noise × max_gain = AGC_target × 0.25 (−12 dB below
   * target).  This prevents AGC from pumping bare noise to near-full volume
   * while preserving sensitivity for signals above the noise floor. */
  float new_max = s_dsp->agc.target * 0.25f / (noise_rms + 1e-10f);
  if (new_max <  1.0f)   new_max =  1.0f;
  if (new_max > 200.0f)  new_max = 200.0f;
  s_dsp->agc.max_gain = new_max;

  char buf[40];
  snprintf(buf, sizeof(buf), "MaxGain=%d  Flr=%d dBFS",
           (int)new_max, (int)(int16_t)noise_db);
  render_cal_result("AGC Ref set:", buf);
}

/* ── Band-level noise floor auto-cal ────────────────────────────────────── */
static void auto_band_noise_floor(void)
{
  if (!s_dsp) return;
  s_last_render_tick = 0U;

  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 2000U)
    render_cal_progress("BandNF: no signal...", HAL_GetTick() - t0, 2000U);

  float floor_db = s_dsp->signal_power_db;
  /* Solve for noise_floor_off so that S-meter reads S0 at the noise floor:
   *   signal_power_db + att + smeter_offset + rx_gain_trim + noise_floor_off = -73
   *   noise_floor_off = -(floor_db + att + smeter_offset + rx_gain_trim + 73)  */
  int16_t off = (int16_t)(-(floor_db
                           + (float)g_att.current_atten_x2 * 0.5f
                           + (float)v_smeter_off
                           + (float)v_band_rx_gain
                           + 73.0f));
  if (off < -20) off = -20;
  if (off >  20) off =  20;
  v_band_nf_off = (int32_t)off;

  char buf[40];
  snprintf(buf, sizeof(buf), "Flr=%d dBFS  Off=%+d",
           (int)(int16_t)floor_db, (int)off);
  render_cal_result("Band NF zeroed:", buf);
}

/* ── Save per-band cal to flash ─────────────────────────────────────────── */
static void save_band_cal(void)
{
  uint8_t bi = g_sdr.band_idx;
  if (bi >= BAND_COUNT) return;
  g_band_cal[bi].rx_gain_trim    = (int16_t)v_band_rx_gain;
  g_band_cal[bi].noise_floor_off = (int16_t)v_band_nf_off;
  g_band_cal[bi].tx_drive_trim   = (int16_t)v_band_tx_drive;
  g_band_cal[bi].swr_scale       = (int16_t)v_band_swr_scale;
  Flash_SaveBandCal(&g_flash, g_band_cal);
  render_cal_result("Band Cal saved:", BPF_BandName(bi));
}

/* ── Encoder delta helper ────────────────────────────────────────────────── */
static uint32_t s_enc_last = 0U;

static int32_t enc_read_delta(void)
{
  uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
  int32_t  d   = (int32_t)(cnt - s_enc_last);
  if (d >  2) { s_enc_last = cnt; return  1; }
  if (d < -2) { s_enc_last = cnt; return -1; }
  return 0;
}

/* Discard accumulated counts: TIM3 keeps counting outside these loops (main
 * tuning), and pressing the ENC shaft button jiggles it by 1-2 counts. */
static void enc_flush(void)
{
  s_enc_last = __HAL_TIM_GET_COUNTER(&htim3);
}

/* ── Sub-level loop ─────────────────────────────────────────────────────── */
static void render_sublevel(uint8_t sect_idx, uint8_t cursor,
                             bool editing, uint8_t scroll)
{
  const CalSection_t *sec = &s_sections[sect_idx];
  uint16_t y = (uint16_t)CAL_Y;
  render_header(sec->title, &y);
  for (uint8_t r = 0U; r < CAL_VISIBLE; r++) {
    uint8_t idx = scroll + r;
    if (idx >= sec->count) break;
    render_sub_item(&sec->items[idx], idx, cursor, editing, y);
    y += CAL_ITEM_H;
  }
}

static void run_section(uint8_t sect_idx)
{
  const CalSection_t *sec = &s_sections[sect_idx];
  uint8_t cursor  = 0U;
  uint8_t scroll  = 0U;
  bool    editing = false;

  Key_t k_enc = {0}, k_f1 = {0}, k_f2 = {0}, k_f4 = {0};
  Key_Init   (&k_enc, ENC_SW_GPIO_Port,  ENC_SW_Pin);
  Key_InitPCA(&k_f1,  &g_pca9555_raw,   PCA_BIT_F1);
  Key_InitPCA(&k_f2,  &g_pca9555_raw,   PCA_BIT_F2);
  Key_InitPCA(&k_f4,  &g_pca9555_raw,   PCA_BIT_F4);
  /* The ENC press that opened this section is likely still held — swallow it
   * so it does not re-fire here and silently toggle edit mode on item 0. */
  Key_Sync(&k_enc); Key_Sync(&k_f1); Key_Sync(&k_f2); Key_Sync(&k_f4);
  enc_flush();

  render_sublevel(sect_idx, cursor, editing, scroll);

  for (;;) {
    Key_Poll(&k_enc); Key_Poll(&k_f1); Key_Poll(&k_f2); Key_Poll(&k_f4);

    /* While the ENC shaft button is physically down (press or release in
     * progress) the shaft jiggles the counter — discard those counts. */
    if (k_enc.state != KS_IDLE) { enc_flush(); }

    /* Encoder rotation */
    int32_t d = enc_read_delta();
    if (d != 0) {
      if (editing && (sec->items[cursor].type == CAL_T_INT ||
                      sec->items[cursor].type == CAL_T_FLOAT10)) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v += d * it->step;
        if (*v < it->min) *v = it->min;
        if (*v > it->max) *v = it->max;
      } else if (editing && sec->items[cursor].type == CAL_T_ENUM) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v += d;
        if (*v < it->min) *v = it->max;   /* wrap around */
        if (*v > it->max) *v = it->min;
      } else {
        if (d > 0 && cursor < sec->count - 1U) { cursor++; }
        if (d < 0 && cursor > 0U)              { cursor--; }
        if (cursor < scroll)                   { scroll = cursor; }
        if (cursor >= scroll + CAL_VISIBLE)    { scroll = (uint8_t)(cursor - CAL_VISIBLE + 1U); }
      }
      render_sublevel(sect_idx, cursor, editing, scroll);
    }

    /* ENC press: toggle edit / confirm action / back */
    if (Key_Press(&k_enc)) {
      const CalItem_t *it = &sec->items[cursor];
      if (it->type == CAL_T_INT || it->type == CAL_T_FLOAT10 || it->type == CAL_T_ENUM) {
        editing = !editing;
      } else if (it->type == CAL_T_BACK) {
        return;
      } else if (it->type == CAL_T_ACTION) {
        if (sect_idx == 1U && cursor == 2U) auto_iq_cal();
        if (sect_idx == 2U && cursor == 2U) auto_dc_cal();
        if (sect_idx == 4U && cursor == 2U) auto_smeter_zero();
        if (sect_idx == 4U && cursor == 3U) auto_noise_floor();
        if (sect_idx == 4U && cursor == 4U) auto_agc_ref();
        if (sect_idx == 6U && cursor == 4U) auto_band_noise_floor();
        if (sect_idx == 6U && cursor == 5U) save_band_cal();
        /* Actions block for seconds (measure + toast): drop any key press
         * or knob turn made during that time. */
        Key_Sync(&k_enc); Key_Sync(&k_f1); Key_Sync(&k_f2); Key_Sync(&k_f4);
        enc_flush();
      }
      render_sublevel(sect_idx, cursor, editing, scroll);
    }

    /* F1 = value up (hold-repeat while editing) */
    if (Key_PressOrRepeat(&k_f1)) {
      if (editing && (sec->items[cursor].type == CAL_T_INT ||
                      sec->items[cursor].type == CAL_T_FLOAT10)) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v += it->step; if (*v > it->max) *v = it->max;
        render_sublevel(sect_idx, cursor, editing, scroll);
      } else if (editing && sec->items[cursor].type == CAL_T_ENUM) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v += it->step; if (*v > it->max) *v = it->min;
        render_sublevel(sect_idx, cursor, editing, scroll);
      }
    }

    /* F2 = value down (hold-repeat while editing) */
    if (Key_PressOrRepeat(&k_f2)) {
      if (editing && (sec->items[cursor].type == CAL_T_INT ||
                      sec->items[cursor].type == CAL_T_FLOAT10)) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v -= it->step; if (*v < it->min) *v = it->min;
        render_sublevel(sect_idx, cursor, editing, scroll);
      } else if (editing && sec->items[cursor].type == CAL_T_ENUM) {
        int32_t *v = sec->items[cursor].val;
        const CalItem_t *it = &sec->items[cursor];
        *v -= it->step; if (*v < it->min) *v = it->max;
        render_sublevel(sect_idx, cursor, editing, scroll);
      }
    }

    /* F4 = exit edit mode / back */
    if (Key_Press(&k_f4)) {
      if (editing) { editing = false; render_sublevel(sect_idx, cursor, editing, scroll); }
      else         { return; }
    }
  }
}

/* ── Top-level loop ─────────────────────────────────────────────────────── */
static void render_toplevel(uint8_t cursor, uint8_t scroll)
{
  uint16_t y = (uint16_t)CAL_Y;
  render_header(" -= CALIBRATION =- ", &y);
  for (uint8_t r = 0U; r < CAL_VISIBLE; r++) {
    uint8_t idx = scroll + r;
    if (idx >= TOP_COUNT) break;
    render_top_item(idx, cursor, y);
    y += CAL_ITEM_H;
  }
}

/* ── Cal_Run ────────────────────────────────────────────────────────────── */
bool Cal_Run(Cal_Params_t *params, DSP_State_t *dsp)
{
  s_dsp = dsp;   /* expose to all auto-cal routines */

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
  v_pa_idx     = pa_watts_to_idx(params->pa_watts);
  v_oc_idx     = (params->pa_oc_limit_idx >= 10U && params->pa_oc_limit_idx <= 200U)
                 ? (int32_t)params->pa_oc_limit_idx : 100;
  v_pwr_scale  = (params->pwr_scale >= 50U && params->pwr_scale <= 200U)
                 ? (int32_t)params->pwr_scale : 100;

  uint8_t cursor = 0U;
  uint8_t scroll = 0U;
  Key_t k_enc = {0}, k_f4 = {0};
  Key_Init   (&k_enc, ENC_SW_GPIO_Port, ENC_SW_Pin);
  Key_InitPCA(&k_f4,  &g_pca9555_raw,  PCA_BIT_F4);
  /* The key press that opened the Cal menu may still be held — swallow it,
   * and drop TIM3 counts accumulated while the main app was tuning. */
  Key_Sync(&k_enc); Key_Sync(&k_f4);
  enc_flush();

  render_toplevel(cursor, scroll);

  for (;;) {
    Key_Poll(&k_enc); Key_Poll(&k_f4);

    if (k_enc.state != KS_IDLE) { enc_flush(); }

    int32_t d = enc_read_delta();
    if (d != 0) {
      if (d > 0 && cursor < TOP_COUNT - 1U) cursor++;
      if (d < 0 && cursor > 0U)             cursor--;
      if (cursor < scroll)                  scroll = cursor;
      if (cursor >= scroll + CAL_VISIBLE)   scroll = (uint8_t)(cursor - CAL_VISIBLE + 1U);
      render_toplevel(cursor, scroll);
    }

    if (Key_Press(&k_enc)) {
      const TopItem_t *it = &s_top[cursor];

      if (it->kind == TOP_SECT) {
        if (cursor == 6U) {
          /* Load current band's cal into working vars and update section title */
          uint8_t bi = g_sdr.band_idx;
          v_band_rx_gain   = (int32_t)g_band_cal[bi].rx_gain_trim;
          v_band_nf_off    = (int32_t)g_band_cal[bi].noise_floor_off;
          v_band_tx_drive  = (int32_t)g_band_cal[bi].tx_drive_trim;
          v_band_swr_scale = (int32_t)g_band_cal[bi].swr_scale;
          snprintf(s_band_cal_title, sizeof(s_band_cal_title),
                   "Band Cal [%s]", BPF_BandName(bi));
          s_sections[6U].title = s_band_cal_title;
        }
        run_section((uint8_t)cursor);
        /* F4/BACK that closed the section is likely still held — swallow it
         * so it does not re-fire here and exit the whole Cal menu. */
        Key_Sync(&k_enc); Key_Sync(&k_f4);
        enc_flush();
        render_toplevel(cursor, scroll);

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
        params->pa_watts        = pa_idx_to_watts(v_pa_idx);
        params->pa_oc_limit_idx = (uint8_t)v_oc_idx;
        params->pwr_scale       = (uint8_t)v_pwr_scale;
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
        v_pa_idx     = pa_watts_to_idx(params->pa_watts);
        v_oc_idx     = (params->pa_oc_limit_idx >= 10U && params->pa_oc_limit_idx <= 200U)
                 ? (int32_t)params->pa_oc_limit_idx : 100;
        v_pwr_scale  = (params->pwr_scale >= 50U && params->pwr_scale <= 200U)
                 ? (int32_t)params->pwr_scale : 100;
        render_toplevel(cursor, scroll);

      } else if (it->kind == TOP_RESET) {
        Cal_Params_t def = CAL_PARAMS_DEFAULT;
        params->xtal_ppm         = def.xtal_ppm;
        params->iq_gain          = def.iq_gain;
        params->iq_phase         = def.iq_phase;
        params->dc_i_offset      = def.dc_i_offset;
        params->dc_q_offset      = def.dc_q_offset;
        params->audio_gain_db    = def.audio_gain_db;
        params->mic_gain         = def.mic_gain;
        params->smeter_offset_db = def.smeter_offset_db;
        params->lo_offset_hz     = def.lo_offset_hz;
        params->pa_watts         = def.pa_watts;
        params->pa_oc_limit_idx  = def.pa_oc_limit_idx;
        params->pwr_scale        = def.pwr_scale;
        /* Sync working vars so UI reflects reset values on any re-entry */
        v_xtal_ppm   = def.xtal_ppm;
        v_iq_gain    = (int32_t)def.iq_gain;
        v_iq_phase   = (int32_t)def.iq_phase;
        v_dc_i       = def.dc_i_offset;
        v_dc_q       = def.dc_q_offset;
        v_audio_gain = (int32_t)def.audio_gain_db;
        v_mic_gain   = (int32_t)def.mic_gain;
        v_smeter_off = (int32_t)def.smeter_offset_db;
        v_lo_offset  = (int32_t)def.lo_offset_hz;
        v_pa_idx     = pa_watts_to_idx(def.pa_watts);
        v_oc_idx     = (int32_t)def.pa_oc_limit_idx;
        v_pwr_scale  = (int32_t)def.pwr_scale;
        /* Reset all per-band cal to defaults and save immediately */
        for (uint8_t bi = 0U; bi < BAND_COUNT; bi++) {
          g_band_cal[bi].rx_gain_trim    = 0;
          g_band_cal[bi].noise_floor_off = 0;
          g_band_cal[bi].tx_drive_trim   = 0;
          g_band_cal[bi].swr_scale       = 100;
        }
        Flash_SaveBandCal(&g_flash, g_band_cal);
        render_cal_result("Reset to defaults", "Cal saved to flash");
        return true;

      } else { /* TOP_EXIT */
        return false;
      }
    }

    if (Key_Press(&k_f4)) {
      return false;
    }
  }
}
