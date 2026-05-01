/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_ui.c
  * @brief   CSDR SDR UI – all SDR_UI_Draw* and SDR_UI_Update* functions
  *
  *  Communicates with the LCD only through the public st7789.h driver API.
  *  No direct SPI/DMA access; no hw_cmd / dma_push calls.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sdr_ui.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── Private defines ─────────────────────────────────────────────────────── */
#define PANEL_H   (ZONE_WF_Y2 - ZONE_TOPBAR_Y2)   /* 178 rows */

#define BIG_W     10U   /* 2× scaled digit width  */
#define BIG_H     16U   /* 2× scaled digit height */

#define WF_MIN_DB    (-120.0f)
#define WF_MAX_DB    ( -20.0f)
#define WF_RANGE_DB  (100.0f)
#define WF_INV_RANGE (255.0f / WF_RANGE_DB)
#define WF_DB_OFFSET  42.14f   /* 10*log10(N²*0.25) for N=256, Hann window */
#define WF_SMOOTH_ALPHA  0.72f
#define CROP_MARGIN      32U   /* bins trimmed from each edge */

/* ── DMA-accessible UI buffers ───────────────────────────────────────────── */
static uint16_t s_topbar[ZONE_TOPBAR_H * LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

static uint16_t s_panel[PANEL_H * PANEL_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

static uint16_t s_spec_buf[ZONE_SPEC_H][DISP_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

static uint16_t s_wf_buf[ZONE_WF_H][DISP_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* CPU-only (no DMA constraint) */
static float    s_wf_smooth[256];
static uint16_t s_wf_lut[256];

/* ── pwr_compress ────────────────────────────────────────────────────────── */
#define PWR_LOG2_FLOOR  (-26.6f)
#define PWR_LOG2_RANGE  ( 26.6f)
static inline float pwr_compress(float pwr)
{
  if (pwr < 1e-9f) return 0.0f;
  uint32_t u; memcpy(&u, &pwr, 4U);
  int32_t exp2 = (int32_t)(u >> 23U) - 127;
  u = (u & 0x007FFFFFU) | 0x3F800000U;
  float m; memcpy(&m, &u, 4U);
  float norm = ((float)exp2 + m - 1.0f - PWR_LOG2_FLOOR) * (1.0f / PWR_LOG2_RANGE);
  if (norm < 0.0f) return 0.0f;
  if (norm > 1.0f) return 1.0f;
  return norm;
}

/* ── spec_color ──────────────────────────────────────────────────────────── */
static uint16_t spec_color(float norm)
{
  uint8_t r, g, b;
  if (norm < 0.40f) {
    float t = norm / 0.40f;
    r = 0; g = (uint8_t)(t * 30.0f); b = (uint8_t)((1.0f - t * 0.5f) * 31.0f);
  } else if (norm < 0.75f) {
    float t = (norm - 0.40f) / 0.35f;
    r = (uint8_t)(t * 24.0f); g = (uint8_t)(30.0f + t * 33.0f); b = (uint8_t)((0.5f - t * 0.5f) * 31.0f);
  } else {
    float t = (norm - 0.75f) / 0.25f;
    r = 31U; g = (uint8_t)((1.0f - t) * 63.0f); b = 0;
  }
  return (uint16_t)((r << 11U) | (g << 5U) | b);
}

/* ── wf_lut_init ─────────────────────────────────────────────────────────── */
static void wf_lut_init(void)
{
  static const float sr[11] = { 0, 0, 0, 0, 0, 0,15,31,31,31,31};
  static const float sg[11] = { 0, 0,15,31,63,63,63,63,32, 0,63};
  static const float sb[11] = { 8,31,31,31,31, 0, 0, 0, 0, 0,31};
  float mr[11], mg[11], mb[11];
  mr[0] = sr[1]-sr[0]; mg[0] = sg[1]-sg[0]; mb[0] = sb[1]-sb[0];
  mr[10]= sr[10]-sr[9];mg[10]= sg[10]-sg[9];mb[10]= sb[10]-sb[9];
  for (int i = 1; i < 10; i++) {
    mr[i] = 0.5f*(sr[i+1]-sr[i-1]);
    mg[i] = 0.5f*(sg[i+1]-sg[i-1]);
    mb[i] = 0.5f*(sb[i+1]-sb[i-1]);
  }
  for (int i = 0; i <= 255; i++) {
    float n = (float)i / 255.0f;
    float ng = powf(n, 0.8f);
    float pos = ng * 10.0f;
    int lo = (int)pos;
    if (lo >= 10) {
      s_wf_lut[i] = SWAP16((uint16_t)((31U<<11)|(63U<<5)|31U));
      continue;
    }
    float t = pos - (float)lo, t2 = t*t, t3 = t2*t;
    float h00 = 2*t3 - 3*t2 + 1;
    float h10 = t3 - 2*t2 + t;
    float h01 = -2*t3 + 3*t2;
    float h11 = t3 - t2;
    int r = (int)(h00*sr[lo] + h10*mr[lo] + h01*sr[lo+1] + h11*mr[lo+1]);
    int g = (int)(h00*sg[lo] + h10*mg[lo] + h01*sg[lo+1] + h11*mg[lo+1]);
    int b = (int)(h00*sb[lo] + h10*mb[lo] + h01*sb[lo+1] + h11*mb[lo+1]);
    if (r < 0) {r = 0;} else if (r > 31) {r = 31;}
    if (g < 0) {g = 0;} else if (g > 63) {g = 63;}
    if (b < 0) {b = 0;} else if (b > 31) {b = 31;}
    s_wf_lut[i] = SWAP16((uint16_t)(((uint16_t)r<<11)|((uint16_t)g<<5)|(uint16_t)b));
  }
}

/* ── 2× scale big numerics ───────────────────────────────────────────────── */
static void ln_bigchar(uint16_t *ln, uint16_t x, uint16_t frow,
                        char c, uint16_t fg, uint16_t bg)
{
  if ((uint8_t)c < 32U || (uint8_t)c > 90U || frow >= BIG_H) return;
  const uint8_t *bmp = Font6x8.data + ((uint8_t)c - 32U) * 6U;
  uint16_t orig_row = frow >> 1U;
  for (uint16_t col = 0; col < 6U; col++) {
    uint8_t bit = (bmp[col] >> orig_row) & 1U;
    uint16_t pix = SWAP16(bit ? fg : bg);
    uint16_t px0 = (uint16_t)(x + col * 10U / 6U);
    uint16_t px1 = (uint16_t)(x + (col + 1U) * 10U / 6U);
    if (px1 > x + BIG_W) { px1 = x + BIG_W; }
    for (uint16_t px = px0; px < px1 && px < LCD_W; px++) { ln[px] = pix; }
  }
  uint16_t last = (uint16_t)(x + BIG_W); if (last > LCD_W) { last = LCD_W; }
  uint16_t filled = (uint16_t)(x + 6U * 10U / 6U);
  uint16_t sbg = SWAP16(bg);
  for (uint16_t px = filled; px < last; px++) { ln[px] = sbg; }
}

static void ln_bigstr(uint16_t *ln, uint16_t x, uint16_t frow,
                       const char *s, uint16_t fg, uint16_t bg)
{
  while (*s) {
    if (*s == '.') {
      if (frow >= BIG_H - 4U && frow < BIG_H - 2U) {
        uint16_t dx = (uint16_t)(x + 4U);
        if (dx < LCD_W) { ln[dx] = SWAP16(fg); }
        if (dx + 1U < LCD_W) { ln[dx + 1U] = SWAP16(fg); }
      }
      x += 4U;
    } else {
      ln_bigchar(ln, x, frow, *s, fg, bg);
      x += BIG_W;
    }
    s++;
    if (x >= LCD_W) { break; }
  }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

void SDR_UI_Init(void)
{
  memset(s_spec_buf,  0, sizeof(s_spec_buf));
  memset(s_wf_buf,    0, sizeof(s_wf_buf));
  memset(s_wf_smooth, 0, sizeof(s_wf_smooth));
  wf_lut_init();
}

/* ── SDR_UI_DrawFrame ────────────────────────────────────────────────────── */
void SDR_UI_DrawFrame(ST7789_Handle_t *lcd)
{
  ST7789_FillScreen(lcd, UI_BG);

  uint16_t *ln = ST7789_GetLineBuf();

  /* Top bar bottom border */
  LCD_LineFill(ln, 0, LCD_W, UI_DIVIDER);
  ST7789_PushWindow(lcd, 0, LCD_W - 1U,
                    ZONE_TOPBAR_Y2 - 1U, ZONE_TOPBAR_Y2 - 1U, ln);

  /* Panel / display background + vertical divider */
  for (uint16_t y = ZONE_TOPBAR_Y2; y < ZONE_WF_Y2; y++) {
    LCD_LineFill(ln, 0, LCD_W, UI_BG);
    ln[PANEL_DIV_X] = SWAP16(UI_BORDER);
    ST7789_PushWindow(lcd, 0, LCD_W - 1U, y, y, ln);
  }

  /* Spectrum / waterfall separator */
  LCD_LineFill(ln, 0, LCD_W, UI_DIVIDER);
  ST7789_PushWindow(lcd, 0, LCD_W - 1U,
                    ZONE_SPEC_Y2 - 1U, ZONE_SPEC_Y2 - 1U, ln);
}

/* ── SDR_UI_DrawTopBar ───────────────────────────────────────────────────── */
void SDR_UI_DrawTopBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  static const char *const mode_s[] = {"AM ","FM ","USB","LSB","CW "};
  static const char *const band_s[] = {"160","80m","60m","40m","30m",
                                        "20m","17m","15m","12m","10m","6m"};
  const char *mode_str = (ui->mode < 5U) ? mode_s[ui->mode] : "---";
  const char *band_str = (ui->band_idx < 11U) ? band_s[ui->band_idx] : "??m";

  uint32_t mhz = ui->freq_hz / 1000000UL;
  uint32_t khz = (ui->freq_hz % 1000000UL) / 1000UL;
  uint32_t hz_r = ui->freq_hz % 1000UL;
  char mhz_s[6], khz_s[8], hz_s[8];
  snprintf(mhz_s, sizeof(mhz_s), "%lu", mhz);
  snprintf(khz_s, sizeof(khz_s), "%03lu", khz);
  snprintf(hz_s,  sizeof(hz_s),  "%03lu", hz_r);

  int32_t bars = (int32_t)((ui->signal_db + 73.0f) / 3.0f);
  if (bars < 0) {bars = 0;} if (bars > SM_BARS) {bars = SM_BARS;}
  char s_str[8];
  if (bars <= 9) { snprintf(s_str, sizeof(s_str), "S%ld", (long)bars); }
  else           { snprintf(s_str, sizeof(s_str), "S9+%ld", (long)((bars - 9) * 3)); }

  bool tx = ui->tx_mode;
  uint16_t ind_bg = tx ? UI_TX_BG : UI_RX_BG;
  uint16_t ind_fg = tx ? UI_TX_FG : UI_RX_FG;
  const char *ind_str = tx ? "TX" : "RX";

  for (uint16_t row = 0; row < ZONE_TOPBAR_H; row++) {
    uint16_t *ln = s_topbar + (uint32_t)row * LCD_W;

    if (row == ZONE_TOPBAR_H - 1U) { LCD_LineFill(ln, 0, LCD_W, UI_DIVIDER); continue; }

    if (row < ZONE_TOPBAR_R1H) {
      LCD_LineFill(ln, 0, LCD_W, UI_TOPBAR_BG);
      bool top = (row == 0), bot = (row == ZONE_TOPBAR_R1H - 1U);
      if (bot) { LCD_LineFill(ln, 0, LCD_W, UI_BORDER); continue; }

      LCD_LineFill(ln, 0, 32U, top ? UI_BORDER : UI_MODE_BG);
      if (!top) {
        ln[0] = SWAP16(UI_BORDER); ln[31] = SWAP16(UI_BORDER);
        if (row >= 6U && (row - 6U) < BIG_H)
          ln_bigstr(ln, 2U, (uint16_t)(row - 6U), mode_str, UI_MODE_FG, UI_MODE_BG);
      }

      LCD_LineFill(ln, 33U, 41U, top ? UI_BORDER : UI_BAND_BG);
      if (!top) {
        ln[33] = SWAP16(UI_BORDER); ln[73] = SWAP16(UI_BORDER);
        if (row >= 6U && (row - 6U) < BIG_H)
          ln_bigstr(ln, 35U, (uint16_t)(row - 6U), band_str, UI_BAND_FG, UI_BAND_BG);
      }

      if (row >= 6U && (row - 6U) < BIG_H) {
        uint16_t fr = (uint16_t)(row - 6U);
        uint16_t fx = 76U;
        ln_bigstr(ln, fx, fr, mhz_s, UI_FREQ_MHZ, UI_TOPBAR_BG);
        fx = (uint16_t)(fx + (uint16_t)strlen(mhz_s) * BIG_W);
        ln_bigstr(ln, fx, fr, ".", UI_FREQ_DOT, UI_TOPBAR_BG); fx += 4U;
        ln_bigstr(ln, fx, fr, khz_s, UI_FREQ_KHZ, UI_TOPBAR_BG);
        fx = (uint16_t)(fx + 3U * BIG_W);
        ln_bigstr(ln, fx, fr, ".", UI_FREQ_DOT, UI_TOPBAR_BG); fx += 4U;
        ln_bigstr(ln, fx, fr, hz_s, UI_FREQ_HZ, UI_TOPBAR_BG);
      }
    } else {
      uint16_t r2 = (uint16_t)(row - ZONE_TOPBAR_R1H);
      LCD_LineFill(ln, 0, LCD_W, UI_TOPBAR_BG);

      if (r2 < Font6x8.height) {
        static const char *const slbls[] = {"S1","S3","S5","S7","S9","+30"};
        static const uint8_t spos[] = {0,2,4,6,8,10};
        for (uint8_t t = 0; t < 6U; t++) {
          uint16_t tx2 = (uint16_t)(SM_START_X + (uint16_t)spos[t] * (SM_BAR_W + SM_BAR_GAP));
          LCD_LineStr(ln, tx2, r2, slbls[t], &Font6x8, UI_STATUS_LBL, UI_TOPBAR_BG);
        }
      }

      if (r2 == 8U) {
        static const uint8_t tick_pos[] = {0,2,4,6,8,10};
        for (uint8_t t = 0; t < 6U; t++) {
          uint16_t tx2 = (uint16_t)(SM_START_X + (uint16_t)tick_pos[t] * (SM_BAR_W + SM_BAR_GAP) + SM_BAR_W / 2U);
          if (tx2 < RXTX_X) { ln[tx2] = SWAP16(UI_SMETER_TICK); }
        }
      }

      if (r2 >= SM_BAR_YOFF && r2 < (uint16_t)(SM_BAR_YOFF + SM_BAR_H)) {
        bool bar_top = (r2 == SM_BAR_YOFF);
        bool bar_bot = (r2 == (uint16_t)(SM_BAR_YOFF + SM_BAR_H - 1U));
        for (uint8_t b = 0; b < SM_BARS; b++) {
          uint16_t bx = (uint16_t)(SM_START_X + (uint16_t)b * (SM_BAR_W + SM_BAR_GAP));
          uint16_t bc_on;
          if (b < 6U) {bc_on = UI_S1_6;} else if (b < 9U) {bc_on = UI_S7_9;} else {bc_on = UI_S9P;}
          uint16_t fill = (b < (uint8_t)bars) ? bc_on : UI_SMETER_BG;
          for (uint16_t px = bx; px < bx + SM_BAR_W && px < RXTX_X; px++) {
            bool edge = (px == bx || px == bx + SM_BAR_W - 1U || bar_top || bar_bot);
            ln[px] = SWAP16(edge ? UI_BORDER : fill);
          }
          if (bx + SM_BAR_W < RXTX_X) { ln[bx + SM_BAR_W] = SWAP16(UI_TOPBAR_BG); }
        }
      }

      if (r2 >= 11U && r2 < 11U + Font6x8.height) {
        uint16_t frow = (uint16_t)(r2 - 11U);
        uint16_t vcol = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
        LCD_LineStr(ln, SM_SVAL_X, frow, s_str, &Font6x8, vcol, UI_TOPBAR_BG);
      }

      LCD_LineFill(ln, RXTX_X, RXTX_W, ind_bg);
      ln[RXTX_X] = SWAP16(UI_BORDER);
      ln[LCD_W - 1U] = SWAP16(UI_BORDER);
      if (r2 == 0 || r2 == ZONE_TOPBAR_R2H - 2U) { LCD_LineFill(ln, RXTX_X, RXTX_W, UI_BORDER); }

      if (r2 >= 13U && r2 < 13U + Font6x8.height)
        LCD_LineStr(ln, (uint16_t)(RXTX_X + 22U), (uint16_t)(r2 - 13U),
                    ind_str, &Font6x8, ind_fg, ind_bg);

      if (r2 >= 10U && r2 <= 12U) {
        uint16_t si_c = ui->si5351_ok ? SWAP16(UI_STATUS_ON) : SWAP16(UI_STATUS_OFF);
        ln[RXTX_X + 4U] = si_c; ln[RXTX_X + 5U] = si_c; ln[RXTX_X + 6U] = si_c;
      }
    }
  }

  ST7789_PushBlock(lcd, ZONE_TOPBAR_Y, (uint16_t)(ZONE_TOPBAR_Y2 - 1U), s_topbar);
}

/* ── SDR_UI_UpdateSMeter_SetTX / SDR_UI_UpdateSMeter ────────────────────── */
static uint16_t     s_smeter_ind_bg  = UI_RX_BG;
static uint16_t     s_smeter_ind_fg  = UI_RX_FG;
static const char  *s_smeter_ind_str = "RX";

void SDR_UI_UpdateSMeter_SetTX(bool tx)
{
  s_smeter_ind_bg  = tx ? UI_TX_BG : UI_RX_BG;
  s_smeter_ind_fg  = tx ? UI_TX_FG : UI_RX_FG;
  s_smeter_ind_str = tx ? "TX" : "RX";
}

void SDR_UI_UpdateSMeter(ST7789_Handle_t *lcd, float signal_db)
{
  int32_t bars = (int32_t)((signal_db + 73.0f) / 3.0f);
  if (bars < 0) {bars = 0;} if (bars > SM_BARS) {bars = SM_BARS;}

  char s_str[8];
  if (bars <= 9) { snprintf(s_str, sizeof(s_str), "S%ld", (long)bars); }
  else           { snprintf(s_str, sizeof(s_str), "S9+%ld", (long)((bars - 9) * 3)); }

  uint16_t abs_y0 = (uint16_t)(ZONE_TOPBAR_R1H + SM_BAR_YOFF);
  uint16_t *ln = ST7789_GetLineBuf();

  for (uint16_t r2 = (uint16_t)SM_BAR_YOFF;
       r2 < (uint16_t)(SM_BAR_YOFF + SM_BAR_H); r2++) {
    bool bar_top = (r2 == SM_BAR_YOFF);
    bool bar_bot = (r2 == (uint16_t)(SM_BAR_YOFF + SM_BAR_H - 1U));

    LCD_LineFill(ln, 0, LCD_W, UI_TOPBAR_BG);

    for (uint8_t b = 0; b < SM_BARS; b++) {
      uint16_t bx = (uint16_t)(SM_START_X + (uint16_t)b * (SM_BAR_W + SM_BAR_GAP));
      uint16_t bc_on;
      if (b < 6U) {bc_on = UI_S1_6;} else if (b < 9U) {bc_on = UI_S7_9;} else {bc_on = UI_S9P;}
      uint16_t fill = (b < (uint8_t)bars) ? bc_on : UI_SMETER_BG;
      for (uint16_t px = bx; px < bx + SM_BAR_W && px < RXTX_X; px++) {
        bool edge = (px == bx || px == bx + SM_BAR_W - 1U || bar_top || bar_bot);
        ln[px] = SWAP16(edge ? UI_BORDER : fill);
      }
    }

    if (!bar_top && !bar_bot) {
      uint16_t fr = (uint16_t)(r2 - SM_BAR_YOFF - 1U);
      if (fr < Font6x8.height) {
        uint16_t vcol = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
        LCD_LineStr(ln, SM_SVAL_X, fr, s_str, &Font6x8, vcol, UI_TOPBAR_BG);
      }
    }

    LCD_LineFill(ln, RXTX_X, RXTX_W, s_smeter_ind_bg);
    if (!bar_top && !bar_bot) {
      uint16_t fr2 = (uint16_t)(r2 - SM_BAR_YOFF);
      if (fr2 >= 7U && fr2 < 7U + Font6x8.height)
        LCD_LineStr(ln, (uint16_t)(RXTX_X + 22U), (uint16_t)(fr2 - 7U),
                    s_smeter_ind_str, &Font6x8, s_smeter_ind_fg, s_smeter_ind_bg);
    }
    ln[RXTX_X] = SWAP16(UI_BORDER);
    ln[LCD_W - 1U] = SWAP16(UI_BORDER);

    uint16_t abs_y = (uint16_t)(abs_y0 + (r2 - SM_BAR_YOFF));
    ST7789_PushWindow(lcd, 0, LCD_W - 1U, abs_y, abs_y, ln);
  }
}

/* ── SDR_UI_DrawStatusPanel ──────────────────────────────────────────────── */
void SDR_UI_DrawStatusPanel(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  char buf[10];
  const uint16_t H = (uint16_t)PANEL_H;

  for (uint32_t i = 0; i < (uint32_t)H * PANEL_W; i++) { s_panel[i] = SWAP16(UI_PANEL_BG); }
  for (uint16_t r = 0; r < H; r++) { s_panel[(uint32_t)r * PANEL_W + (PANEL_W - 1U)] = SWAP16(UI_BORDER); }
  uint16_t sep_step = (uint16_t)(H / 8U);
  for (uint8_t s = 1; s < 8U; s++) {
    uint16_t ry = (uint16_t)(s * sep_step);
    if (ry < H) {
      for (uint16_t x = 0; x < PANEL_W - 1U; x++) { s_panel[(uint32_t)ry * PANEL_W + x] = SWAP16(UI_BORDER); }
    }
  }

#define PI(r0,lbl,val_str,vc) do{ \
    uint16_t _vx=(uint16_t)(2U+(uint16_t)(strlen(lbl)*(uint16_t)Font6x8.width)+2U); \
    for(uint16_t _fr=0;_fr<Font6x8.height;_fr++){ \
      uint16_t _row=(uint16_t)((r0)+(_fr)+5U); if(_row>=H)break; \
      uint16_t *_ln=s_panel+(uint32_t)_row*PANEL_W; \
      LCD_LineStr(_ln,2U,_fr,(lbl),&Font6x8,UI_STATUS_LBL,UI_PANEL_BG); \
      LCD_LineStr(_ln,_vx,_fr,(val_str),&Font6x8,(vc),UI_PANEL_BG); \
    } }while(0)

  PI(0U,            "AGC", ui->agc_fast ? "FAST" : "SLOW",
     ui->agc_fast ? UI_STATUS_VAL : UI_STATUS_LBL);
  PI(sep_step * 1U, "NB",  ui->nb_on ? "ON" : "OFF",
     ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF);
  PI(sep_step * 2U, "NR",  ui->nr_on ? "ON" : "OFF",
     ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF);
  snprintf(buf, sizeof(buf), "%+05d", (int)ui->rit_hz);
  PI(sep_step * 3U, "RIT", buf, ui->rit_hz != 0 ? UI_FREQ_KHZ : UI_STATUS_LBL);
  snprintf(buf, sizeof(buf), "%3d%%", (int)ui->volume);
  PI(sep_step * 4U, "VOL", buf, UI_STATUS_VAL);
  snprintf(buf, sizeof(buf), "%3d", (int)ui->squelch);
  PI(sep_step * 5U, "SQ",  buf, UI_STATUS_VAL);
  uint32_t st = (uint32_t)ui->step;
  if      (st >= 100000U) { snprintf(buf, sizeof(buf), "100KHz"); }
  else if (st >= 10000U)  { snprintf(buf, sizeof(buf), "10KHz"); }
  else if (st >= 1000U)   { snprintf(buf, sizeof(buf), "1KHz"); }
  else if (st >= 100U)    { snprintf(buf, sizeof(buf), "100Hz"); }
  else if (st >= 10U)     { snprintf(buf, sizeof(buf), "10Hz"); }
  else                    { snprintf(buf, sizeof(buf), "1Hz"); }
  PI(sep_step * 6U, "STP", buf, UI_FREQ_KHZ);
  snprintf(buf, sizeof(buf), "%uHz", (unsigned)ui->bw_hz);
  PI(sep_step * 7U, "BW",  buf, UI_FREQ_KHZ);
#undef PI

  for (uint16_t r = 0; r < H; r++) {
    ST7789_PushWindow(lcd, PANEL_X, PANEL_W - 1U,
                      (uint16_t)(ZONE_TOPBAR_Y2 + r),
                      (uint16_t)(ZONE_TOPBAR_Y2 + r),
                      s_panel + (uint32_t)r * PANEL_W);
  }
}

/* ── SDR_UI_DrawSpectrum ─────────────────────────────────────────────────── */
void SDR_UI_DrawSpectrum(ST7789_Handle_t *lcd,
                          const float *fft_db, uint16_t bins,
                          float bw_lo_ratio, float bw_hi_ratio,
                          SDR_UI_State_t *ui)
{
  if (!bins) { return; }
  const uint16_t W = DISP_W;
  const uint16_t H = ZONE_SPEC_H;

  const uint16_t b0    = (bins > 2U * CROP_MARGIN) ? (uint16_t)CROP_MARGIN : 0U;
  const uint16_t n_vis = (bins > 2U * CROP_MARGIN) ? (uint16_t)(bins - 2U * CROP_MARGIN) : bins;
  const float bin_per_px = (float)n_vis / (float)W;
  const float crop_scale = (float)bins  / (float)n_vis;

  uint16_t bar_h[256U];
  if (W > 256U) { return; }
  for (uint16_t x = 0U; x < W; x++) {
    uint16_t bi = (uint16_t)((float)x * bin_per_px) + b0;
    if (bi >= bins) { bi = bins - 1U; }
    bar_h[x] = (uint16_t)(pwr_compress(fft_db[bi]) * (float)H);
  }

  uint16_t g20 = (uint16_t)(H - (uint16_t)(0.75f * (float)H));
  uint16_t g40 = (uint16_t)(H - (uint16_t)(0.50f * (float)H));
  uint16_t g60 = (uint16_t)(H - (uint16_t)(0.25f * (float)H));
  uint16_t cx_px = W / 2U;

  bool bw_lo_valid = (bw_lo_ratio > 0.0001f);
  bool bw_hi_valid = (bw_hi_ratio > 0.0001f);
  uint16_t bw_lo_px = 0U, bw_hi_px = 0U;
  if (bw_lo_valid) {
    uint16_t off = (uint16_t)(bw_lo_ratio * (float)W * crop_scale + 0.5f);
    if (off == 0U) { off = 1U; }
    bw_lo_px = (cx_px > off) ? (uint16_t)(cx_px - off) : 0U;
  }
  if (bw_hi_valid) {
    uint16_t off = (uint16_t)(bw_hi_ratio * (float)W * crop_scale + 0.5f);
    if (off == 0U) { off = 1U; }
    bw_hi_px = cx_px + off;
    if (bw_hi_px >= W) { bw_hi_px = W - 1U; }
  }

  for (uint16_t y = 0U; y < H; y++) {
    uint16_t *row = s_spec_buf[y];
    bool is_grid = (y == g20 || y == g40 || y == g60);
    uint16_t bg_sw = is_grid ? SWAP16(UI_SPEC_GRID) : SWAP16(UI_SPEC_BG);

    for (uint16_t x = 0U; x < W; x++) { row[x] = bg_sw; }

    if (!is_grid) {
      for (uint16_t gx = 0U; gx < W; gx += 40U) { row[gx] = SWAP16(UI_SPEC_GRID); }
    }

    if (bw_lo_valid && bw_lo_px < W) { row[bw_lo_px] = SWAP16(UI_SPEC_BW); }
    if (bw_hi_valid && bw_hi_px < W) { row[bw_hi_px] = SWAP16(UI_SPEC_BW); }
    if (cx_px < W)                   { row[cx_px]     = SWAP16(UI_SPEC_CENTER); }

    uint16_t y_from_bot = (uint16_t)(H - 1U - y);
    for (uint16_t x = 0U; x < W; x++) {
      if (bar_h[x] > 0U && y_from_bot < bar_h[x])
        row[x] = SWAP16(spec_color((float)y_from_bot / (float)H));
    }
  }

  for (uint16_t y = 0U; y < H; y++) {
    ST7789_PushWindow(lcd, DISP_X, LCD_W - 1U,
                      (uint16_t)(ZONE_SPEC_Y + y),
                      (uint16_t)(ZONE_SPEC_Y + y),
                      s_spec_buf[y]);
  }
  (void)ui;
}

/* ── SDR_UI_DrawWaterfall ────────────────────────────────────────────────── */
void SDR_UI_DrawWaterfall(ST7789_Handle_t *lcd,
                           const float *fft_db, uint16_t bins)
{
  if (!bins) { return; }

  uint16_t nb = (bins <= 256U) ? bins : 256U;
  for (uint16_t b = 0U; b < nb; b++) {
    s_wf_smooth[b] = WF_SMOOTH_ALPHA * s_wf_smooth[b]
                   + (1.0f - WF_SMOOTH_ALPHA) * fft_db[b];
  }

  const uint16_t b0    = (nb > 2U * CROP_MARGIN) ? (uint16_t)CROP_MARGIN : 0U;
  const uint16_t n_vis = (nb > 2U * CROP_MARGIN) ? (uint16_t)(nb - 2U * CROP_MARGIN) : nb;

  dma_wait_pub(lcd);
  memmove(&s_wf_buf[1][0], &s_wf_buf[0][0], (ZONE_WF_H - 1U) * DISP_W * 2U);

  const float xs = (float)DISP_W / (float)n_vis;
  for (uint16_t x = 0U; x < DISP_W; x++) {
    uint16_t bi = (uint16_t)((float)x / xs) + b0;
    if (bi >= nb) { bi = nb - 1U; }
    float pwr = s_wf_smooth[bi];
    float db  = (pwr > 1e-30f) ? (10.0f * log10f(pwr) - WF_DB_OFFSET) : WF_MIN_DB;
    int   idx = (int)((db - WF_MIN_DB) * WF_INV_RANGE);
    if (idx < 0) {idx = 0;} else if (idx > 255) {idx = 255;}
    s_wf_buf[0][x] = s_wf_lut[idx];
  }

  for (uint16_t r = 0U; r < ZONE_WF_H; r++) {
    ST7789_PushWindow(lcd, DISP_X, LCD_W - 1U,
                      (uint16_t)(ZONE_WF_Y + r),
                      (uint16_t)(ZONE_WF_Y + r),
                      s_wf_buf[r]);
  }
}

/* ── SDR_UI_DrawFuncBar (stub) ───────────────────────────────────────────── */
void SDR_UI_DrawFuncBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{ (void)lcd; (void)ui; }
