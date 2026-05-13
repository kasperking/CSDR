/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_ui.c
  * @brief   CSDR SDR UI – 8-zone layout, single-block SPI pushes
  *
  *  All zone buffers live in DMA_SRAM.
  *  Each Draw* function renders its buffer then pushes in ONE SPI transaction.
  *  Waterfall uses a 2-split ring push for zero-copy true scroll.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sdr_ui.h"
#include "runtime_diag.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── Private defines ─────────────────────────────────────────────────────── */
#define BIG_W   15U   /* 3× scaled digit width  (Font6x8 col × 15/6) */
#define BIG_H   24U   /* 3× scaled digit height (Font6x8 row × 3)    */

#define MED_W   12U   /* 2× scaled digit width  (Font6x8 col × 2)    */
#define MED_H   16U   /* 2× scaled digit height (Font6x8 row × 2)    */

#define WF_MIN_DB    (-120.0f)
#define WF_RANGE_DB  ( 100.0f)
#define WF_INV_RANGE (255.0f / WF_RANGE_DB)
#define WF_DB_OFFSET  30.00f
#define WF_SMOOTH_ALPHA  0.72f
/* (CROP_MARGIN removed – spectrum window is now zoom-derived) */

/* ── DMA-accessible zone buffers ─────────────────────────────────────────── */
static uint16_t s_hdr_buf[HDR_H  * LCD_W]   /* 11,520 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_sbl_buf[SBL_H  * SBL_W]   /*  9,840 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_sbr_buf[SBR_H  * SBR_W]   /*  9,840 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_vfo_buf[VFO_H  * VFO_W]   /* 17,600 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_mtr_buf[MTR_H  * MTR_W]   /* 15,200 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_spec_buf[SPEC_H][SPEC_W]  /* 24,320 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_wf_buf[WF_H][WF_W]        /* 19,200 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* WF pre-compute: two uint8_t line buffers (double-buffer for DSP/UI split) */
static uint8_t  s_wf_idx[2][WF_W];          /*    640 B */
static volatile uint8_t s_wf_fill = 0;

/* CPU-only: IIR smoother + colour LUT + ring head */
static float    s_wf_smooth[256];
static uint16_t s_wf_lut[256];
static uint8_t  s_wf_head = 0;
static float    s_smeter_voltage = 0.0f;

/* Spectrum delta-skip: previous column pixel rows; skips redraw when unchanged */
static uint16_t s_spec_py_prev[SPEC_W];
static bool     s_spec_py_valid = false;
static uint32_t s_spec_skip_hits = 0U;
static uint32_t s_spec_draw_hits = 0U;
static int32_t  s_rx_meter_bars = -1;
static bool     s_tx_meter_active = false;
static int32_t  s_tx_alc_bars = -1;
static int32_t  s_tx_alc_pct = -1;
static int32_t  s_tx_swr_x10 = -1;
/* Meter partial-push: static rows (labels, ticks) valid in buffer after first full draw */
static bool     s_mtr_static_valid = false;

/* VFO section-split cache: avoid pushing unchanged half-zones */
static struct {
  uint32_t freq_hz;
  uint32_t freq_b_hz;
  uint32_t step;
  uint32_t bw_hz;
  int16_t  rit_hz;
  bool     tx_mode;
  uint8_t  active_vfo;
  bool     valid;
} s_vfo_cache;

/* ── pwr_compress: fast log2-based amplitude normalise ───────────────────── */
#define PWR_LOG2_FLOOR  (-26.6f)
#define PWR_LOG2_RANGE  ( 26.6f)
static inline float pwr_compress(float pwr)
{
  if (pwr < 1e-9f) return 0.0f;
  uint32_t u; memcpy(&u, &pwr, 4U);
  int32_t e = (int32_t)(u >> 23U) - 127;
  u = (u & 0x007FFFFFU) | 0x3F800000U;
  float m; memcpy(&m, &u, 4U);
  float n = ((float)e + m - 1.0f - PWR_LOG2_FLOOR) * (1.0f / PWR_LOG2_RANGE);
  if (n < 0.0f) return 0.0f;
  if (n > 1.0f) return 1.0f;
  return n;
}

/* ── wf_lut_init: Hermite-spline thermal palette ─────────────────────────── */
static void wf_lut_init(void)
{
  static const float sr[11] = { 0, 0, 0, 0, 0, 0, 0,0,8,16,31};
  static const float sg[11] = { 0, 0,8,20,40,63,63,63,63,63,63};
  static const float sb[11] = { 0, 16,31,31,31,31,24,16,8,0,31};
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
    float ng = powf(n, 1.2f);
    float pos = ng * 10.0f;
    int lo = (int)pos;
    if (lo >= 10) {
      s_wf_lut[i] = SWAP16((uint16_t)((31U<<11)|(63U<<5)|31U));
      continue;
    }
    float t = pos-(float)lo, t2=t*t, t3=t2*t;
    float h00=2*t3-3*t2+1, h10=t3-2*t2+t, h01=-2*t3+3*t2, h11=t3-t2;
    int r=(int)(h00*sr[lo]+h10*mr[lo]+h01*sr[lo+1]+h11*mr[lo+1]);
    int g=(int)(h00*sg[lo]+h10*mg[lo]+h01*sg[lo+1]+h11*mg[lo+1]);
    int b=(int)(h00*sb[lo]+h10*mb[lo]+h01*sb[lo+1]+h11*mb[lo+1]);
    if (r<0)r=0; else if (r>31)r=31;
    if (g<0)g=0; else if (g>63)g=63;
    if (b<0)b=0; else if (b>31)b=31;
    s_wf_lut[i]=SWAP16((uint16_t)(((uint16_t)r<<11)|((uint16_t)g<<5)|(uint16_t)b));
  }
}

/* ── 3× scaled big digit rendering (onto a line-buffer slice) ────────────── */
static void ln_bigchar(uint16_t *ln, uint16_t x, uint16_t frow,
                        char c, uint16_t fg, uint16_t bg)
{
  if ((uint8_t)c < 32U || (uint8_t)c > 90U || frow >= BIG_H) return;
  const uint8_t *bmp = Font6x8.data + ((uint8_t)c - 32U) * 6U;
  uint16_t orig_row = frow / 3U;
  for (uint16_t col = 0; col < 6U; col++) {
    uint8_t bit = (bmp[col] >> orig_row) & 1U;
    uint16_t pix = SWAP16(bit ? fg : bg);
    uint16_t px0 = (uint16_t)(x + col * 15U / 6U);
    uint16_t px1 = (uint16_t)(x + (col + 1U) * 15U / 6U);
    if (px1 > x + BIG_W) px1 = x + BIG_W;
    for (uint16_t px = px0; px < px1; px++) ln[px] = pix;
  }
  uint16_t last   = (uint16_t)(x + BIG_W);
  uint16_t filled = (uint16_t)(x + 6U * 15U / 6U);
  uint16_t sbg    = SWAP16(bg);
  for (uint16_t px = filled; px < last; px++) ln[px] = sbg;
}

static void ln_bigstr(uint16_t *ln, uint16_t x, uint16_t frow,
                       const char *s, uint16_t fg, uint16_t bg)
{
  while (*s) {
    if (*s == '.') {
      /* 3-pixel dot centred in the 6-px dot slot, near baseline */
      if (frow >= BIG_H - 5U && frow < BIG_H - 2U) {
        ln[x + 2U] = SWAP16(fg);
        ln[x + 3U] = SWAP16(fg);
        ln[x + 4U] = SWAP16(fg);
      }
      x += 6U;
    } else {
      ln_bigchar(ln, x, frow, *s, fg, bg);
      x += BIG_W;
    }
    s++;
  }
}

/* ── 2× scaled medium digit rendering (onto a line-buffer slice) ─────────── */
static void ln_medchar(uint16_t *ln, uint16_t x, uint16_t frow,
                       char c, uint16_t fg, uint16_t bg)
{
  if ((uint8_t)c < 32U || (uint8_t)c > 90U || frow >= MED_H) return;
  const uint8_t *bmp = Font6x8.data + ((uint8_t)c - 32U) * 6U;
  uint16_t orig_row = frow / 2U;
  for (uint16_t col = 0; col < 6U; col++) {
    uint8_t  bit = (bmp[col] >> orig_row) & 1U;
    uint16_t pix = SWAP16(bit ? fg : bg);
    ln[x + col * 2U]      = pix;
    ln[x + col * 2U + 1U] = pix;
  }
}

static void ln_medstr(uint16_t *ln, uint16_t x, uint16_t frow,
                      const char *s, uint16_t fg, uint16_t bg)
{
  while (*s) {
    if (*s == '.') {
      /* 4-pixel dot near baseline, 6-px slot (matches big-font dot style) */
      if (frow >= MED_H - 4U && frow < MED_H - 2U) {
        ln[x + 1U] = SWAP16(fg); ln[x + 2U] = SWAP16(fg);
        ln[x + 3U] = SWAP16(fg); ln[x + 4U] = SWAP16(fg);
      }
      x += 6U;
    } else {
      ln_medchar(ln, x, frow, *s, fg, bg);
      x += MED_W;
    }
    s++;
  }
}

/* ── buf_fill ────────────────────────────────────────────────────────────── */
static inline void buf_fill(uint16_t *buf, uint32_t n, uint16_t color)
{
  uint16_t c = SWAP16(color);
  for (uint32_t i = 0; i < n; i++) buf[i] = c;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_Init
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_Init(void)
{
  memset(s_spec_buf,  0, sizeof(s_spec_buf));
  memset(s_wf_buf,    0, sizeof(s_wf_buf));
  memset(s_wf_smooth, 0, sizeof(s_wf_smooth));
  s_wf_head = 0; s_wf_fill = 0;
  s_spec_py_valid = false;
  s_mtr_static_valid = false;
  s_vfo_cache.valid  = false;
  wf_lut_init();
}

/* ── Spectrum zoom state ─────────────────────────────────────────────────────
 * zoom=0 → ±24kHz   half=128  (full FFT, 256 bins)
 * zoom=1 → ±18kHz   half= 96
 * zoom=2 → ±12kHz   half= 64
 * zoom=3 → ±6kHz    half= 32
 * zoom=4 → ±3kHz    half= 16
 * b0 = center - half,  n_vis = 2 * half
 * ─────────────────────────────────────────────────────────────────────────── */
static uint8_t  s_spec_zoom = 0U;
static uint32_t s_spec_sr   = 48000U;
static uint16_t s_spec_bins = 256U;

/* Half-span in bins (referenced to 256-bin FFT) for each zoom level */
static const uint8_t s_zoom_half[SPEC_ZOOM_COUNT] = { 128U, 96U, 64U, 32U, 16U };

static void spec_window(uint16_t bins, uint16_t *b0_out, uint16_t *n_vis_out)
{
  uint16_t center = bins >> 1U;
  uint16_t half   = (uint16_t)((uint32_t)s_zoom_half[s_spec_zoom] * bins / 256U);
  if (half < 1U) half = 1U;
  *b0_out    = (uint16_t)(center - half);
  *n_vis_out = (uint16_t)(half << 1U);
}

static uint32_t spec_half_span_hz(void)
{
  uint16_t b0, n_vis;
  spec_window(s_spec_bins, &b0, &n_vis);
  (void)b0;
  return (uint32_t)((uint32_t)(n_vis >> 1U) * s_spec_sr / s_spec_bins);
}

static void draw_footer_rows(ST7789_Handle_t *lcd, uint32_t half_hz)
{
  char lbuf[12] = "";
  char rbuf[12] = "";
  uint16_t rx_x = 0U;
  if (half_hz > 0U) {
    uint32_t bk = half_hz / 1000U;
    snprintf(lbuf, sizeof(lbuf), "-%luK", (unsigned long)bk);
    snprintf(rbuf, sizeof(rbuf), "+%luK", (unsigned long)bk);
    rx_x = (uint16_t)(LCD_W - (uint16_t)(strlen(rbuf) * Font6x8.width) - 2U);
  }
  uint16_t fh  = Font6x8.height;
  uint16_t pad = (FTR_H - fh) / 2U;
  for (uint16_t row = 0U; row < FTR_H; row++) {
    uint16_t *ln = ST7789_GetLineBuf();
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    if (half_hz > 0U && row >= pad && row < pad + fh) {
      uint16_t frow = row - pad;
      LCD_LineStr(ln, 2U, frow, lbuf, &Font6x8, UI_SPEC_GRID, UI_BG);
      LCD_LineStr(ln, (uint16_t)(LCD_W / 2U - Font6x8.width / 2U), frow,
                  "0", &Font6x8, UI_SPEC_GRID, UI_BG);
      LCD_LineStr(ln, rx_x, frow, rbuf, &Font6x8, UI_SPEC_GRID, UI_BG);
    }
    ST7789_PushScanline(lcd, (uint16_t)(FTR_Y + row), ln);
    dma_wait_pub(lcd);
    cs_high_pub(lcd);
  }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawFrame  – one-time skeleton
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawFrame(ST7789_Handle_t *lcd, uint32_t sample_rate, uint16_t fft_bins)
{
  s_spec_sr   = sample_rate ? sample_rate : 48000U;
  s_spec_bins = fft_bins    ? fft_bins    : 256U;

  ST7789_FillScreen(lcd, UI_BG);
  draw_footer_rows(lcd, spec_half_span_hz());
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawHeader  (HDR_H=18 rows × LCD_W=320 cols)
 *
 *  Row map:
 *   0:       top border
 *   1-16:    content
 *   17:      bottom border / divider
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawHeader(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  char att_str[8];
  snprintf(att_str, sizeof(att_str), "ATT:%u", ui->att_db);
  uint16_t att_x = 4U;

  char vstr[12];
  snprintf(vstr, sizeof(vstr), "%.1fV", ui->voltage);
  uint16_t vcol = (ui->voltage < 11.5f && ui->voltage > 0.5f)
                  ? UI_STATUS_OFF : UI_STATUS_VAL;
  uint16_t volt_x = (uint16_t)(LCD_W - (uint16_t)(strlen(vstr) * Font6x8.width) - 4U);

  uint16_t txt_y = (uint16_t)((HDR_H - Font6x8.height) / 2U);  /* = 5 */

  for (uint16_t row = 0; row < HDR_H; row++) {
    uint16_t *ln = s_hdr_buf + (uint32_t)row * LCD_W;

    if (row == HDR_H - 1U) {
      LCD_LineFill(ln, 0, LCD_W, UI_DIVIDER);
      continue;
    }
    LCD_LineFill(ln, 0, LCD_W, UI_HDR_BG);

    /* ATT */
    if (row >= txt_y && row < txt_y + Font6x8.height)
      LCD_LineStr(ln, att_x, row - txt_y, att_str, &Font6x8, UI_STATUS_VAL, UI_HDR_BG);

    /* Voltage */
    if (row >= txt_y && row < txt_y + Font6x8.height)
      LCD_LineStr(ln, volt_x, row - txt_y, vstr, &Font6x8, vcol, UI_HDR_BG);
  }

  ST7789_PushBlock(lcd, HDR_Y, HDR_Y2 - 1U, s_hdr_buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSidebarLeft  (SBL_W=60 × SBL_H=82)
 *
 *  5 items × 16px = 80px, 2px bottom border
 *  Items: Mode(badge) | VFO | NR·NB | VOL | SQL
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSidebarLeft(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  static const char *const mode_s[]  = {"AM","FM","USB","LSB","CW"};
  static const uint16_t    mode_bg[] = {UI_MODE_AM,UI_MODE_FM,UI_MODE_USB,
                                         UI_MODE_LSB,UI_MODE_CW};
  const char *mode_str = (ui->mode < 5U) ? mode_s[ui->mode]  : "---";
  uint16_t    mbg      = (ui->mode < 5U) ? mode_bg[ui->mode] : UI_STATUS_LBL;

  char vol_str[6]; snprintf(vol_str, sizeof(vol_str), "%u", ui->volume);
  char sql_str[6]; snprintf(sql_str, sizeof(sql_str), "%u", ui->squelch);

  buf_fill(s_sbl_buf, (uint32_t)SBL_H * SBL_W, UI_SBL_BG);

  uint16_t item_h = 16U;

  for (uint8_t i = 0; i < 5U; i++) {
    uint16_t y0 = (uint16_t)(i * item_h);

    /* Separator (except first) */
    if (i > 0U)
      for (uint16_t x = 0; x < SBL_W; x++)
        s_sbl_buf[(uint32_t)y0 * SBL_W + x] = SWAP16(UI_DIVIDER);

    uint16_t text_y = y0 + 4U;

    switch (i) {
      /* ── Mode: full-width colored badge ─────────────────── */
      case 0:
    for (uint16_t fr = 0; fr < Font6x8.height; fr++) {

        uint16_t r = text_y + fr;
        if (r >= 80U) break;

        uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;

        uint16_t cx =
            (uint16_t)(1U +
            (SBL_W - 1U -
            (uint16_t)(strlen(mode_str) * Font6x8.width)) / 2U);

        LCD_LineStr(ln, cx + 1u, fr, mode_str, &Font6x8, mbg, UI_SBL_BG);
    }
    break;

      /* ── VFO A/B – active letter highlighted ────────────── */
      case 1: {
        uint16_t col_a = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_LBL;
        uint16_t col_b = (ui->active_vfo == 1U) ? UI_STATUS_ON  : UI_STATUS_LBL;
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= 80U) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStr(ln, 2U,                        fr, "VFO", &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          LCD_LineStr(ln, 2U + 4U * Font6x8.width,  fr, "A",   &Font6x8, col_a, UI_SBL_BG);
          LCD_LineStr(ln, 2U + 5U * Font6x8.width,  fr, "/",   &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          LCD_LineStr(ln, 2U + 6U * Font6x8.width,  fr, "B",   &Font6x8, col_b, UI_SBL_BG);
        }
        break;
      }

      /* ── NR / NB toggle badges ───────────────────────────── */
      case 2: {
        uint16_t nr_bg = ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF;
        uint16_t nb_bg = ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF;
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= 80U) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          /* NR badge: x=1..27 */
          LCD_LineFill(ln, 1U, 27U, nr_bg);
          LCD_LineStr(ln, 8U, fr, "NR", &Font6x8, UI_BG, nr_bg);
          /* NB badge: x=30..57 */
          LCD_LineFill(ln, 30U, 27U, nb_bg);
          LCD_LineStr(ln, 37U, fr, "NB", &Font6x8, UI_BG, nb_bg);
        }
        break;
      }

      /* ── Volume ──────────────────────────────────────────── */
      case 3:
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= 80U) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStr(ln, 2U, fr, "VOL", &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          uint16_t vx = (uint16_t)(SBL_W - 1U - (uint16_t)(strlen(vol_str) * Font6x8.width) - 3U);
          LCD_LineStr(ln, vx, fr, vol_str, &Font6x8, UI_STATUS_VAL, UI_SBL_BG);
        }
        break;

      /* ── Squelch ─────────────────────────────────────────── */
      case 4:
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= 80U) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStr(ln, 2U, fr, "SQL", &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          uint16_t vx = (uint16_t)(SBL_W - 1U - (uint16_t)(strlen(sql_str) * Font6x8.width) - 3U);
          LCD_LineStr(ln, vx, fr, sql_str, &Font6x8, UI_STATUS_VAL, UI_SBL_BG);
        }
        break;

      default: break;
    }
  }

  ST7789_PushWindow(lcd, SBL_X, (uint16_t)(SBL_X + SBL_W - 1U),
                    SBL_Y, SBL_Y2 - 1U, s_sbl_buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSidebarRight  (SBR_W=60 × SBR_H=82)
 *
 *  4 items × 20px = 80px, 2px bottom border
 *  Items: RIT | MIC | DSP | BW
 *  Layout per item: rows 2-9 label (gray), rows 11-18 value (right-aligned)
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSidebarRight(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  char rit_str[8];
  snprintf(rit_str, sizeof(rit_str), ui->rit_hz ? "%+d" : "OFF", (int)ui->rit_hz);

  char mic_str[8]; snprintf(mic_str, sizeof(mic_str), "%d", (int)ui->mic_gain);
  char dsp_str[6]; snprintf(dsp_str, sizeof(dsp_str), "%u", ui->dsp_level);

  /* BW: ≥10k → "12k", ≥1k → "2.7k", <1k → "500Hz" */
  char bw_str[10];
  if (ui->bw_hz >= 10000U) {
    snprintf(bw_str, sizeof(bw_str), "%luk",
             (unsigned long)(ui->bw_hz / 1000U));
  } else if (ui->bw_hz >= 1000U) {
    snprintf(bw_str, sizeof(bw_str), "%lu.%luk",
             (unsigned long)(ui->bw_hz / 1000U),
             (unsigned long)((ui->bw_hz % 1000U) / 100U));
  } else {
    snprintf(bw_str, sizeof(bw_str), "%luHz", (unsigned long)ui->bw_hz);
  }

  buf_fill(s_sbr_buf, (uint32_t)SBR_H * SBR_W, UI_SBR_BG);

  uint16_t item_h = 20U;

  struct { const char *lbl; const char *val; uint16_t vc; } items[4] = {
    { "RIT", rit_str, ui->rit_hz ? UI_FREQ_KHZ : UI_STATUS_LBL },
    { "MIC", mic_str, UI_STATUS_VAL },
    { "DSP", dsp_str, UI_STATUS_VAL },
    { "BW",  bw_str,  UI_FREQ_KHZ  },
  };

  for (uint8_t i = 0; i < 4U; i++) {
    uint16_t y0 = (uint16_t)(i * item_h);

    if (i > 0U)
      for (uint16_t x = 0U; x < SBR_W; x++)
        s_sbr_buf[(uint32_t)y0 * SBR_W + x] = SWAP16(UI_DIVIDER);

    /* Label: rows y0+2 .. y0+9 */
    for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
      uint16_t r = y0 + 2U + fr;
      if (r >= 80U) break;
      uint16_t *ln = s_sbr_buf + (uint32_t)r * SBR_W;
      LCD_LineStr(ln, 2U, fr, items[i].lbl, &Font6x8, UI_STATUS_LBL, UI_SBR_BG);
    }

    /* Value: rows y0+11 .. y0+18, right-aligned */
    uint16_t val_len = (uint16_t)(strlen(items[i].val) * Font6x8.width);
    uint16_t val_x   = (uint16_t)(SBR_W - val_len - 3U);
    for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
      uint16_t r = y0 + 11U + fr;
      if (r >= 80U) break;
      uint16_t *ln = s_sbr_buf + (uint32_t)r * SBR_W;
      LCD_LineStr(ln, val_x, fr, items[i].val, &Font6x8, items[i].vc, UI_SBR_BG);
    }
  }

  ST7789_PushWindow(lcd, SBR_X, (uint16_t)(SBR_X + SBR_W - 1U),
                    SBR_Y, SBR_Y2 - 1U, s_sbr_buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawVFO  (VFO_W=200 × VFO_H=44)
 *
 *  Row map:
 *   0:       top border
 *   1-8:     step / active-VFO label (Font6x8, top corners)
 *   2-25:    big frequency digits (BIG_H=24, freq_top=2)
 *   27-42:   VFO B sub-freq at 2× scale (MED_H=16) or RIT at Font6x8
 *   43:      bottom gap
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawVFO(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  uint32_t mhz  = ui->freq_hz / 1000000UL;
  uint32_t khz  = (ui->freq_hz % 1000000UL) / 1000UL;
  uint32_t hz_r = ui->freq_hz % 1000UL;
  char mhz_s[6], khz_s[4], hz_s[4];
  snprintf(mhz_s, sizeof(mhz_s), "%lu",   (unsigned long)mhz);
  snprintf(khz_s, sizeof(khz_s), "%03lu", (unsigned long)khz);
  snprintf(hz_s,  sizeof(hz_s),  "%03lu", (unsigned long)hz_r);

  /* Step label */
  char step_str[10];
  uint32_t st = ui->step;
  if      (st >= 100000U) { snprintf(step_str, sizeof(step_str), "100k"); }
  else if (st >= 10000U)  { snprintf(step_str, sizeof(step_str), "10k"); }
  else if (st >= 1000U)   { snprintf(step_str, sizeof(step_str), "1k"); }
  else if (st >= 100U)    { snprintf(step_str, sizeof(step_str), "100"); }
  else if (st >= 10U)     { snprintf(step_str, sizeof(step_str), "10"); }
  else                    { snprintf(step_str, sizeof(step_str), "1"); }

  /* BW label (right of step) */
  char bw_str[10];
  if (ui->bw_hz >= 10000U) {
    snprintf(bw_str, sizeof(bw_str), "%luk", (unsigned long)(ui->bw_hz / 1000U));
  } else if (ui->bw_hz >= 1000U) {
    snprintf(bw_str, sizeof(bw_str), "%lu.%luk",
             (unsigned long)(ui->bw_hz / 1000U),
             (unsigned long)((ui->bw_hz % 1000U) / 100U));
  } else {
    snprintf(bw_str, sizeof(bw_str), "%luHz", (unsigned long)ui->bw_hz);
  }

  /* Sub-freq / RIT line – shows inactive VFO frequency */
  char sub_str[22] = "";
  if (ui->freq_b_hz > 0U) {
    uint32_t bm = ui->freq_b_hz / 1000000UL;
    uint32_t bk = (ui->freq_b_hz % 1000000UL) / 1000UL;
    uint32_t bh = ui->freq_b_hz % 1000UL;
    const char *pfx = (ui->active_vfo == 0U) ? "B:" : "A:";
    snprintf(sub_str, sizeof(sub_str), "%s%lu.%03lu.%03lu",
             pfx, (unsigned long)bm, (unsigned long)bk, (unsigned long)bh);
  } else if (ui->rit_hz != 0) {
    snprintf(sub_str, sizeof(sub_str), "RIT %+d Hz", (int)ui->rit_hz);
  }

  /* RX/TX indicator (this panel has swapped R/B: 0xF800=blue=RX, 0x001F=red=TX) */
  const char *rt_str   = ui->tx_mode ? "TX" : "RX";
  uint16_t    rt_color = ui->tx_mode ? 0x001FU : 0xF800U;

  /* Precompute MED-scale pixel width of sub_str so RX/TX can be placed right after */
  uint16_t sub_med_w = 0U;
  if (ui->freq_b_hz > 0U) {
    const char *p = sub_str;
    while (*p) { sub_med_w += (*p == '.') ? 6U : MED_W; p++; }
  }

  buf_fill(s_vfo_buf, (uint32_t)VFO_H * VFO_W, UI_VFO_BG);

  const uint16_t freq_top = 2U;
  const uint16_t step_y   = 1U;
  const uint16_t bw_y     = (uint16_t)(step_y + Font6x8.height + 1U);  /* 10 */
  const uint16_t sub_y    = (uint16_t)(freq_top + BIG_H + 1U);          /* 27 */
  uint16_t step_x = (uint16_t)(VFO_W - (uint16_t)(strlen(step_str) * Font6x8.width) - 3U);
  uint16_t bw_x   = (uint16_t)(VFO_W - (uint16_t)(strlen(bw_str)   * Font6x8.width) - 3U);

  for (uint16_t row = 0; row < VFO_H; row++) {
    uint16_t *ln = s_vfo_buf + (uint32_t)row * VFO_W;

    /* Big frequency – centred horizontally */
    if (row >= freq_top && (row - freq_top) < BIG_H) {
      uint16_t fr = row - freq_top;
      /* total width: mhz_digits + dot + 3 khz digits + dot + 3 hz digits */
      uint16_t total_w = (uint16_t)((strlen(mhz_s) + 6U) * BIG_W + 12U);
      uint16_t fx = (VFO_W > total_w) ? (uint16_t)((VFO_W - total_w) / 2U) : 2U;
      ln_bigstr(ln, fx, fr, mhz_s, UI_FREQ_MHZ, UI_VFO_BG);
      fx += (uint16_t)(strlen(mhz_s) * BIG_W);
      ln_bigstr(ln, fx, fr, ".", UI_FREQ_DOT, UI_VFO_BG); fx += 6U;
      ln_bigstr(ln, fx, fr, khz_s, UI_FREQ_KHZ, UI_VFO_BG); fx += 3U * BIG_W;
      ln_bigstr(ln, fx, fr, ".", UI_FREQ_DOT, UI_VFO_BG); fx += 6U;
      ln_bigstr(ln, fx, fr, hz_s, UI_FREQ_HZ, UI_VFO_BG);
    }

    /* Active VFO label: top-left, 2× scaled for clear identification */
    if (row >= step_y && (row - step_y) < MED_H) {
      const char *vl = (ui->active_vfo == 0U) ? "A" : "B";
      uint16_t    vc = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_ON;
      ln_medchar(ln, 2U, row - step_y, *vl, vc, UI_VFO_BG);
    }

    /* Step label + BW below: top-right corner */
    if (row >= step_y && (row - step_y) < Font6x8.height)
      LCD_LineStr(ln, step_x, row - step_y, step_str, &Font6x8, UI_FREQ_KHZ, UI_VFO_BG);
    if (row >= bw_y && (row - bw_y) < Font6x8.height)
      LCD_LineStr(ln, bw_x, row - bw_y, bw_str, &Font6x8, UI_STATUS_LBL, UI_VFO_BG);

    /* Sub-freq + RX/TX indicator */
    if (ui->freq_b_hz > 0U) {
      if (row >= sub_y && (row - sub_y) < MED_H) {
        uint16_t fr = row - sub_y;
        ln_medstr(ln, 4U, fr, sub_str, UI_FREQ_SUB, UI_VFO_BG);
        uint16_t rt_x = (uint16_t)(4U + sub_med_w + 6U);
        if (rt_x + 2U * MED_W <= VFO_W)
          ln_medstr(ln, rt_x, fr, rt_str, rt_color, UI_VFO_BG);
      }
    } else if (ui->rit_hz != 0) {
      if (row >= sub_y && (row - sub_y) < Font6x8.height) {
        uint16_t fr    = row - sub_y;
        LCD_LineStr(ln, 4U, fr, sub_str, &Font6x8, UI_FREQ_SUB, UI_VFO_BG);
        uint16_t sub_w = (uint16_t)(strlen(sub_str) * Font6x8.width);
        uint16_t rt_x  = (uint16_t)(4U + sub_w + 4U);
        if (rt_x + 2U * Font6x8.width <= VFO_W)
          LCD_LineStr(ln, rt_x, fr, rt_str, &Font6x8, rt_color, UI_VFO_BG);
      }
    } else {
      /* No secondary VFO, no RIT: show RX/TX standalone at sub_y */
      if (row >= sub_y && (row - sub_y) < MED_H)
        ln_medstr(ln, 4U, row - sub_y, rt_str, rt_color, UI_VFO_BG);
    }
  }

  /* Section-split push: upper = rows 0..26 (border+step+digits),
   * lower = rows 27..43 (sub-freq/RIT/RX-TX).  Push only changed section(s).
   * sub_y = freq_top + BIG_H + 1 = 27 → exact split point. */
  const uint16_t VFO_SPLIT = 27U;

  bool upper_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_hz    != ui->freq_hz
      || s_vfo_cache.step       != ui->step
      || s_vfo_cache.bw_hz      != ui->bw_hz
      || s_vfo_cache.active_vfo != ui->active_vfo;

  bool lower_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_b_hz  != ui->freq_b_hz
      || s_vfo_cache.rit_hz     != ui->rit_hz
      || s_vfo_cache.tx_mode    != ui->tx_mode
      || s_vfo_cache.active_vfo != ui->active_vfo;

  s_vfo_cache.freq_hz    = ui->freq_hz;
  s_vfo_cache.freq_b_hz  = ui->freq_b_hz;
  s_vfo_cache.step       = ui->step;
  s_vfo_cache.bw_hz      = ui->bw_hz;
  s_vfo_cache.rit_hz     = ui->rit_hz;
  s_vfo_cache.tx_mode    = ui->tx_mode;
  s_vfo_cache.active_vfo = ui->active_vfo;
  s_vfo_cache.valid      = true;

  if (upper_chg && lower_chg) {
    ST7789_PushWindow(lcd, VFO_X, (uint16_t)(VFO_X + VFO_W - 1U),
                      VFO_Y, VFO_Y2 - 1U, s_vfo_buf);
  } else if (upper_chg) {
    /* 27 rows × 200px = 10.8 kB (vs 17.6 kB full) */
    ST7789_PushWindow(lcd, VFO_X, (uint16_t)(VFO_X + VFO_W - 1U),
                      VFO_Y, (uint16_t)(VFO_Y + VFO_SPLIT - 1U), s_vfo_buf);
  } else if (lower_chg) {
    /* 17 rows × 200px = 6.8 kB */
    ST7789_PushWindow(lcd, VFO_X, (uint16_t)(VFO_X + VFO_W - 1U),
                      (uint16_t)(VFO_Y + VFO_SPLIT), VFO_Y2 - 1U,
                      s_vfo_buf + (uint32_t)VFO_SPLIT * VFO_W);
  }
  /* else: content unchanged – skip push */
}

/* ════════════════════════════════════════════════════════════════════════════
 *  draw_smeter_rows  – fills s_mtr_buf for RX S-meter
 *
 *  MTR_H=38 row map:
 *   0-7:       top gap
 *   8-15:      scale labels  ("S 1 3 5 7 9 +20 +40")
 *  16:         tick marks
 *  17-19:      3-px thin bar (smooth fill, colour-coded)
 *  22-29:      S-value text  ("S7", "S9+6")
 * ════════════════════════════════════════════════════════════════════════════ */
static void draw_smeter_rows(int32_t bars)
{
  char s_str[8];
  if (bars <= 9) snprintf(s_str, sizeof(s_str), "S%ld",   (long)bars);
  else           snprintf(s_str, sizeof(s_str), "S9+%ld", (long)((bars - 9) * 3));

  uint16_t fill_x = (uint16_t)(SM_START_X + (uint16_t)bars * (SM_BAR_W + SM_BAR_GAP));
  uint16_t x_end  = (uint16_t)(SM_START_X + SM_TOTAL_W);

  static const char *const slbls[] = {"S","1","3","5","7","9","+20","+40"};
  static const uint8_t     sbar[]  = { 0,  1,  3,  5,  7,  9,  10,   11 };

  for (uint16_t row = 0; row < MTR_H; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0, MTR_W, UI_MTR_BG);

    /* Scale labels: rows 8-15 */
    if (row >= 8U && row < 8U + Font6x8.height) {
      uint16_t fr = row - 8U;
      for (uint8_t t = 0; t < 8U; t++) {
        uint16_t lx  = (uint16_t)(SM_START_X + (uint16_t)sbar[t] * (SM_BAR_W + SM_BAR_GAP));
        /* "+20" (t=6) is 18 px wide but only 14 px before "+40":
         * shift 9 px left → right-align before "+40" tick with clear gap. */
        if (t == 6U) { lx -= 9U; }
        uint16_t col = (t < 6U) ? UI_SMETER_TICK : UI_S1_6;
        LCD_LineStr(ln, lx, fr, slbls[t], &Font6x8, col, UI_MTR_BG);
      }
    }

    /* Tick marks: row 16 */
    if (row == 16U) {
      for (uint8_t t = 0; t < 8U; t++) {
        uint16_t tx = (uint16_t)(SM_START_X + (uint16_t)sbar[t] * (SM_BAR_W + SM_BAR_GAP));
        if (tx < MTR_W) ln[tx] = SWAP16(UI_SMETER_TICK);
      }
    }

    /* Thin bar: rows 17-19 */
    if (row >= 17U && row < 20U) {
      for (uint16_t px = SM_START_X; px < x_end && px < MTR_W; px++) {
        if (px < fill_x) {
          uint16_t off = px - SM_START_X;
          uint16_t seg = off / (SM_BAR_W + SM_BAR_GAP);
          uint16_t col = (seg < 6U) ? UI_S1_6 : (seg < 9U) ? UI_S7_9 : UI_S9P;
          ln[px] = SWAP16(col);
        } else {
          ln[px] = SWAP16(UI_SMETER_BG);
        }
      }
    }

    /* S-value text: rows 22-29 */
    if (row >= 22U && (row - 22U) < Font6x8.height) {
      uint16_t fr   = row - 22U;
      uint16_t scol = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
      LCD_LineStr(ln, SM_START_X, fr, s_str, &Font6x8, scol, UI_MTR_BG);
    }
  }
}

void SDR_UI_DrawMeter(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  int32_t bars = (int32_t)((ui->signal_db + 73.0f) / 3.0f);
  if (bars < 0) bars = 0;
  if (bars > (int32_t)SM_BARS) bars = (int32_t)SM_BARS;
  s_rx_meter_bars = bars;
  draw_smeter_rows(bars);
  s_mtr_static_valid = true;   /* full render: static rows now valid in buffer */
  ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                    MTR_Y, MTR_Y2 - 1U, s_mtr_buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_UpdateSMeter  – fast meter refresh (10 Hz, RX)
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdateSMeter_SetTX(bool tx)
{
  (void)tx;
  s_tx_meter_active = false;
  s_tx_alc_bars = -1;
  s_tx_alc_pct = -1;
  s_tx_swr_x10 = -1;
  s_rx_meter_bars = -1;
  s_mtr_static_valid = false;  /* TX meter overwrites buffer; force full RX redraw on return */
}
void SDR_UI_UpdateSMeter_SetVoltage(float v) { s_smeter_voltage = v; }

void SDR_UI_UpdateSMeter(ST7789_Handle_t *lcd, float signal_db)
{
  int32_t bars = (int32_t)((signal_db + 73.0f) / 3.0f);
  if (bars < 0) bars = 0;
  if (bars > (int32_t)SM_BARS) bars = (int32_t)SM_BARS;
  if (bars == s_rx_meter_bars) return;
  s_rx_meter_bars = bars;

  if (!s_mtr_static_valid) {
    /* First call or after TX: full render to restore static rows, then full push */
    draw_smeter_rows(bars);
    s_mtr_static_valid = true;
    ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                      MTR_Y, MTR_Y2 - 1U, s_mtr_buf);
    return;
  }

  /* Fast path: re-render only the two dynamic row bands, push each as a small window.
   * Static rows (8-16: labels + tick) remain valid in the buffer from the last full draw.
   * Total pushed: 3 rows (bar) + 8 rows (text) = 4.4 kB vs 15.2 kB full zone. */

  /* Rows 17-19: thin bar – overwrite bar region only, leave BG pixels unchanged */
  uint16_t fill_x = (uint16_t)(SM_START_X + (uint16_t)bars * (SM_BAR_W + SM_BAR_GAP));
  uint16_t x_end  = (uint16_t)(SM_START_X + SM_TOTAL_W);
  for (uint16_t row = 17U; row < 20U; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    for (uint16_t px = SM_START_X; px < x_end && px < MTR_W; px++) {
      if (px < fill_x) {
        uint16_t off = (uint16_t)(px - SM_START_X);
        uint16_t seg = off / (SM_BAR_W + SM_BAR_GAP);
        uint16_t col = (seg < 6U) ? UI_S1_6 : (seg < 9U) ? UI_S7_9 : UI_S9P;
        ln[px] = SWAP16(col);
      } else {
        ln[px] = SWAP16(UI_SMETER_BG);
      }
    }
  }

  /* Rows 22-29: S-value text – clear full row then write text to handle length changes */
  char s_str[8];
  if (bars <= 9) snprintf(s_str, sizeof(s_str), "S%ld",   (long)bars);
  else           snprintf(s_str, sizeof(s_str), "S9+%ld", (long)((bars - 9) * 3));
  uint16_t scol = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
  for (uint16_t row = 22U; row < 30U; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0, MTR_W, UI_MTR_BG);
    uint16_t fr = row - 22U;
    if (fr < Font6x8.height)
      LCD_LineStr(ln, SM_START_X, fr, s_str, &Font6x8, scol, UI_MTR_BG);
  }

  /* Push bar rows (3 rows × 200px = 1.2 kB) */
  ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                    (uint16_t)(MTR_Y + 17U), (uint16_t)(MTR_Y + 19U),
                    s_mtr_buf + 17U * MTR_W);
  /* Push text rows (8 rows × 200px = 3.2 kB) */
  ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                    (uint16_t)(MTR_Y + 22U), (uint16_t)(MTR_Y + 29U),
                    s_mtr_buf + 22U * MTR_W);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  TX meter helpers – render only the row span that will be pushed
 * ════════════════════════════════════════════════════════════════════════════ */
static void tx_meter_render_rows(uint16_t row0, uint16_t row1,
                                 int32_t alc_b, int32_t alc_pct,
                                 int32_t swr_x10)
{
  uint16_t alc_fill = (uint16_t)(SM_START_X + (uint16_t)alc_b * (SM_BAR_W + SM_BAR_GAP));
  uint16_t x_end    = (uint16_t)(SM_START_X + SM_TOTAL_W);
  char alc_val[6]; snprintf(alc_val, sizeof(alc_val), "%ld%%", (long)alc_pct);
  char swr_val[6]; snprintf(swr_val, sizeof(swr_val), "%ld.%ld",
                            (long)(swr_x10 / 10), (long)(swr_x10 % 10));
  uint16_t swr_col = (swr_x10 >= 30) ? UI_S9P : (swr_x10 >= 20) ? UI_S7_9 : UI_S1_6;

  static const char *const slbls[] = {"0","25","50","75","100"};
  static const uint8_t     spos[]  = { 0,   3,   6,   9,  11 };

  if (row1 >= MTR_H) row1 = MTR_H - 1U;
  for (uint16_t row = row0; row <= row1; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0, MTR_W, UI_MTR_BG);

    /* Scale labels: rows 5-12 */
    if (row >= 5U && row < 5U + Font6x8.height) {
      uint16_t fr = row - 5U;
      for (uint8_t t = 0; t < 5U; t++) {
        uint16_t lx = (uint16_t)(SM_START_X + (uint16_t)spos[t] * (SM_BAR_W + SM_BAR_GAP));
        LCD_LineStr(ln, lx, fr, slbls[t], &Font6x8, UI_SMETER_TICK, UI_MTR_BG);
      }
    }

    /* Tick marks: row 13 */
    if (row == 13U) {
      for (uint8_t t = 0; t < 5U; t++) {
        uint16_t tx = (uint16_t)(SM_START_X + (uint16_t)spos[t] * (SM_BAR_W + SM_BAR_GAP));
        if (tx < MTR_W) ln[tx] = SWAP16(UI_SMETER_TICK);
      }
    }

    /* Thin ALC bar: rows 14-16 */
    if (row >= 14U && row < 17U) {
      for (uint16_t px = SM_START_X; px < x_end && px < MTR_W; px++) {
        if (px < alc_fill) {
          uint16_t off = px - SM_START_X;
          uint16_t seg = off / (SM_BAR_W + SM_BAR_GAP);
          uint16_t col = (seg < 6U) ? UI_S1_6 : (seg < 9U) ? UI_S7_9 : UI_S9P;
          ln[px] = SWAP16(col);
        } else {
          ln[px] = SWAP16(UI_SMETER_BG);
        }
      }
    }

    /* ALC value text: rows 18-25 */
    if (row >= 18U && (row - 18U) < Font6x8.height) {
      uint16_t fr = row - 18U;
      LCD_LineStr(ln, SM_START_X, fr, "ALC", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, SM_START_X + 4U * Font6x8.width, fr, alc_val,
                  &Font6x8, UI_STATUS_VAL, UI_MTR_BG);
    }

    /* SWR compact text: rows 28-35 – no bar, label + value only */
    if (row >= 28U && (row - 28U) < Font6x8.height) {
      uint16_t fr = row - 28U;
      LCD_LineStr(ln, SM_START_X, fr, "SWR", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, SM_START_X + 4U * Font6x8.width, fr, swr_val,
                  &Font6x8, swr_col, UI_MTR_BG);
    }
  }
}

static void tx_meter_push_rows(ST7789_Handle_t *lcd, uint16_t row0, uint16_t row1)
{
  ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                    (uint16_t)(MTR_Y + row0), (uint16_t)(MTR_Y + row1),
                    s_mtr_buf + (uint32_t)row0 * MTR_W);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_UpdateTXMeters  – dirty, row-bounded ALC/SWR refresh for TX
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdateTXMeters(ST7789_Handle_t *lcd, float alc_norm, float swr)
{
  int32_t alc_b = (int32_t)(alc_norm * (float)SM_BARS + 0.5f);
  if (alc_b < 0) alc_b = 0;
  if (alc_b > (int32_t)SM_BARS) alc_b = (int32_t)SM_BARS;

  int32_t alc_pct = (int32_t)(alc_norm * 100.0f + 0.5f);
  if (alc_pct < 0) alc_pct = 0;
  if (alc_pct > 999) alc_pct = 999;

  int32_t swr_x10 = (int32_t)(swr * 10.0f + 0.5f);
  if (swr_x10 < 0) swr_x10 = 0;
  if (swr_x10 > 999) swr_x10 = 999;

  /* First TX meter draw is intentionally split into small SPI windows.  The
   * old path pushed all 200x38 pixels (15.2 kB) synchronously; each dirty
   * region below is <= 3.2 kB (blank gaps split out separately), so one
   * LCD wait is much less likely to straddle a 5.33 ms audio deadline. */
  bool first = !s_tx_meter_active;
  if (first) s_tx_meter_active = true;

  if (first) {
    tx_meter_render_rows(0U, 4U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 0U, 4U);
    tx_meter_render_rows(5U, 12U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 5U, 12U);
    tx_meter_render_rows(13U, 13U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 13U, 13U);
    tx_meter_render_rows(26U, 27U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 26U, 27U);
    tx_meter_render_rows(36U, 37U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 36U, 37U);
  }

  if (first || alc_b != s_tx_alc_bars) {
    tx_meter_render_rows(14U, 16U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 14U, 16U);
    s_tx_alc_bars = alc_b;
  }

  if (first || alc_pct != s_tx_alc_pct) {
    tx_meter_render_rows(18U, 25U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 18U, 25U);
    s_tx_alc_pct = alc_pct;
  }

  if (first || swr_x10 != s_tx_swr_x10) {
    tx_meter_render_rows(28U, 35U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(lcd, 28U, 35U);
    s_tx_swr_x10 = swr_x10;
  }
}

/* ── Compat wrappers ─────────────────────────────────────────────────────── */
void SDR_UI_DrawTopBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  SDR_UI_DrawHeader(lcd, ui);
  SDR_UI_DrawVFO(lcd, ui);
  SDR_UI_DrawMeter(lcd, ui);
}

void SDR_UI_DrawStatusPanel(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{
  SDR_UI_DrawSidebarLeft(lcd, ui);
  SDR_UI_DrawSidebarRight(lcd, ui);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSpectrum  – single PushBlock (no per-row loop)
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSpectrum(ST7789_Handle_t *lcd,
                          const float *fft_db, uint16_t bins,
                          float bw_lo_ratio, float bw_hi_ratio,
                          SDR_UI_State_t *ui)
{
  if (!bins) return;
  uint16_t b0, n_vis;
  spec_window(bins, &b0, &n_vis);
  const float bpp    = (float)n_vis / (float)SPEC_W;
  const float cscale = (float)bins  / (float)n_vis;

  /* Step 1: linear interpolation between adjacent FFT bins */
  float yf[SPEC_W];
  for (uint16_t x = 0; x < SPEC_W; x++) {
    float    fbin = (float)b0 + (float)x * bpp;
    uint16_t bi0  = (uint16_t)fbin;
    if (bi0 >= bins) bi0 = (uint16_t)(bins - 1U);
    float    t    = fbin - (float)bi0;
    uint16_t bi1  = (bi0 + 1U < bins) ? (uint16_t)(bi0 + 1U) : bi0;
    yf[x] = pwr_compress(fft_db[bi0]) * (1.0f - t)
           + pwr_compress(fft_db[bi1]) * t;
  }

  /* Step 2: 3-tap symmetric smoothing (reads original neighbours, embedded-friendly) */
  {
    float prev = yf[0];
    for (uint16_t x = 1U; x < (uint16_t)(SPEC_W - 1U); x++) {
      float c = yf[x];
      yf[x]  = 0.25f * prev + 0.5f * c + 0.25f * yf[x + 1U];
      prev   = c;
    }
  }

  /* Step 3: amplitude [0,1] → pixel row [0, SPEC_H-2].
   * NO_SIG sentinel (= SPEC_H-1) marks zero-amplitude columns. */
  const uint16_t NO_SIG = (uint16_t)(SPEC_H - 1U);
  uint16_t py[SPEC_W];
  for (uint16_t x = 0; x < SPEC_W; x++) {
    uint16_t h = (uint16_t)(yf[x] * (float)(SPEC_H - 2U) + 0.5f);
    if (h > (uint16_t)(SPEC_H - 2U)) h = (uint16_t)(SPEC_H - 2U);
    py[x] = (h > 0U) ? (uint16_t)(SPEC_H - 1U - h) : NO_SIG;
  }

  /* Delta skip: if no column moved >= 2 pixels, the visual is unchanged – skip. */
  if (s_spec_py_valid) {
    uint16_t max_d = 0U;
    for (uint16_t x = 0U; x < SPEC_W; x++) {
      uint16_t d = (py[x] > s_spec_py_prev[x])
                 ? (uint16_t)(py[x] - s_spec_py_prev[x])
                 : (uint16_t)(s_spec_py_prev[x] - py[x]);
      if (d > max_d) { max_d = d; if (max_d >= 2U) break; }
    }
    if (max_d < 2U) { s_spec_skip_hits++; return; }
  }
  s_spec_draw_hits++;
  memcpy(s_spec_py_prev, py, sizeof(py));
  s_spec_py_valid = true;

  /* Step 4: per-column segment bounds for connected polyline (left-bridge).
   * Column x bridges vertically from py[x-1] to py[x], giving a thin
   * connected line with no gaps between adjacent columns. */
  uint16_t seg_top[SPEC_W], seg_bot[SPEC_W];
  seg_top[0] = seg_bot[0] = py[0];
  for (uint16_t x = 1U; x < SPEC_W; x++) {
    uint16_t cur  = py[x];
    uint16_t prev = py[x - 1U];
    if (cur >= NO_SIG) {
      seg_top[x] = seg_bot[x] = NO_SIG;       /* no signal this column */
    } else if (prev >= NO_SIG) {
      seg_top[x] = seg_bot[x] = cur;           /* left gap: single pixel */
    } else {
      seg_top[x] = (cur < prev) ? cur : prev;
      seg_bot[x] = (cur > prev) ? cur : prev;
    }
  }

  /* Grid rows and bandwidth markers */
  uint16_t g1 = (uint16_t)(SPEC_H - (uint16_t)(0.75f * (float)SPEC_H));
  uint16_t g2 = (uint16_t)(SPEC_H - (uint16_t)(0.50f * (float)SPEC_H));
  uint16_t g3 = (uint16_t)(SPEC_H - (uint16_t)(0.25f * (float)SPEC_H));
  uint16_t cx = SPEC_W / 2U;

  bool bw_lo_ok = (bw_lo_ratio > 0.0001f), bw_hi_ok = (bw_hi_ratio > 0.0001f);
  uint16_t bw_lo = 0U, bw_hi = 0U;
  if (bw_lo_ok) {
    uint16_t off = (uint16_t)(bw_lo_ratio * (float)SPEC_W * cscale + 0.5f);
    if (!off) off = 1U;
    bw_lo = (cx > off) ? (uint16_t)(cx - off) : 0U;
  }
  if (bw_hi_ok) {
    uint16_t off = (uint16_t)(bw_hi_ratio * (float)SPEC_W * cscale + 0.5f);
    if (!off) off = 1U;
    bw_hi = cx + off;
    if (bw_hi >= SPEC_W) bw_hi = SPEC_W - 1U;
  }

  /* Render rows 0..SPEC_H-2: background + grid + markers (no per-row polyline scan) */
  uint16_t spec_sw = SWAP16(0xFCC0U);          /* amber signal colour */
  uint16_t bw_sw   = SWAP16(UI_SPEC_BW);
  uint16_t cx_sw   = SWAP16(UI_SPEC_CENTER);
  uint16_t dot_sw  = SWAP16(UI_SPEC_GRID);
  for (uint16_t y = 0U; y < (uint16_t)(SPEC_H - 1U); y++) {
    uint16_t *row    = s_spec_buf[y];
    bool      is_grid = (y == g1 || y == g2 || y == g3);
    uint16_t  bg_sw  = is_grid ? SWAP16(UI_SPEC_GRID) : SWAP16(UI_SPEC_BG);

    for (uint16_t x = 0U; x < SPEC_W; x++) row[x] = bg_sw;
    if (!is_grid) {
      for (uint16_t gx = 0U; gx < SPEC_W; gx += 40U) row[gx] = dot_sw;
    }
    if (bw_lo_ok && bw_lo < SPEC_W) row[bw_lo] = bw_sw;
    if (bw_hi_ok && bw_hi < SPEC_W) row[bw_hi] = bw_sw;
    if (cx < SPEC_W)                 row[cx]     = cx_sw;
  }

  /* Column-first polyline pass: for each column write amber only to its row span.
   * Replaces O(H×W) per-row scan with O(W×avg_signal_height) stride writes. */
  for (uint16_t x = 0U; x < SPEC_W; x++) {
    uint16_t top = seg_top[x];
    uint16_t bot = seg_bot[x];
    if (top >= NO_SIG) continue;
    for (uint16_t yr = top; yr <= bot; yr++) {
      s_spec_buf[yr][x] = spec_sw;
    }
  }

  /* Last row: spectrum/waterfall divider */
  for (uint16_t x = 0; x < SPEC_W; x++)
    s_spec_buf[SPEC_H - 1U][x] = SWAP16(UI_DIVIDER);

  ST7789_PushBlock(lcd, SPEC_Y, SPEC_Y2 - 1U, &s_spec_buf[0][0]);
  (void)ui;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_WaterfallPrecompute
 *  Call from DSP task: IIR smooth + log10f + LUT index → s_wf_idx[fill]
 *  Returns buffer index for SDR_UI_WaterfallPush.
 * ════════════════════════════════════════════════════════════════════════════ */
uint8_t SDR_UI_WaterfallPrecompute(const float *fft_db, uint16_t bins)
{
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_WF_PRECOMPUTE);
  uint8_t fill = (uint8_t)(s_wf_fill ^ 1U);
  uint8_t *dst = s_wf_idx[fill];

  uint16_t nb = (bins <= 256U) ? bins : 256U;
  uint16_t b0, nvis;
  spec_window(nb, &b0, &nvis);
  float xs = (float)WF_W / (float)nvis;

  for (uint16_t b = 0; b < nb; b++)
    s_wf_smooth[b] = WF_SMOOTH_ALPHA * s_wf_smooth[b]
                   + (1.0f - WF_SMOOTH_ALPHA) * fft_db[b];

  for (uint16_t x = 0; x < WF_W; x++) {
    uint16_t bi  = (uint16_t)((float)x / xs) + b0;
    if (bi >= nb) bi = nb - 1U;
    float pwr = s_wf_smooth[bi];
    float db  = (pwr > 1e-30f) ? (10.0f * log10f(pwr) - WF_DB_OFFSET) : WF_MIN_DB;
    int   idx = (int)((db - WF_MIN_DB) * WF_INV_RANGE);
    if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
    dst[x] = (uint8_t)idx;
  }

  s_wf_fill = fill;
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_WF_PRECOMPUTE);
  return fill;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_WaterfallPush
 *  Call from UI task: LUT lookup → ring slot, then 2-split push for true scroll.
 *
 *  Head decrements before write so newest row always sits at s_wf_head:
 *   s_wf_buf[s_wf_head]     = newest → display top    (Block A start)
 *   s_wf_buf[s_wf_head-1..] = older  → display bottom (Block B)
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_WaterfallPush(ST7789_Handle_t *lcd, uint8_t buf_idx)
{
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_WF_SCROLL);
  /* Decrement head first so newest row sits at head → displayed at top */
  s_wf_head = (s_wf_head == 0U) ? (uint8_t)(WF_H - 1U) : (uint8_t)(s_wf_head - 1U);

  const uint8_t *src = s_wf_idx[buf_idx];
  uint16_t *row = s_wf_buf[s_wf_head];
  for (uint16_t x = 0; x < WF_W; x++) row[x] = s_wf_lut[src[x]];

  /* Block A: s_wf_buf[head .. WF_H-1]  → physical WF_Y .. WF_Y+a-1  (newest first) */
  uint16_t a = (uint16_t)(WF_H - s_wf_head);
  ST7789_PushWindow(lcd, WF_X, (uint16_t)(WF_X + WF_W - 1U),
                    WF_Y, (uint16_t)(WF_Y + a - 1U),
                    &s_wf_buf[s_wf_head][0]);

  /* Block B: s_wf_buf[0 .. head-1]     → physical WF_Y+a .. WF_Y2-1 (older) */
  if (s_wf_head > 0U)
    ST7789_PushWindow(lcd, WF_X, (uint16_t)(WF_X + WF_W - 1U),
                      (uint16_t)(WF_Y + a), WF_Y2 - 1U,
                      &s_wf_buf[0][0]);
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_WF_SCROLL);
}

/* ── Compat: Precompute + Push in one call ───────────────────────────────── */
void SDR_UI_DrawWaterfall(ST7789_Handle_t *lcd,
                           const float *fft_db, uint16_t bins)
{
  if (!bins) return;
  uint8_t idx = SDR_UI_WaterfallPrecompute(fft_db, bins);
  SDR_UI_WaterfallPush(lcd, idx);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Spectrum zoom control
 *  zoom=0 ±24kHz | zoom=1 ±18kHz | zoom=2 ±12kHz | zoom=3 ±6kHz | zoom=4 ±3kHz
 *  spec_window() re-derives b0/n_vis used by DrawSpectrum and
 *  WaterfallPrecompute, so both stay aligned automatically.
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_SetSpecZoom(ST7789_Handle_t *lcd, uint8_t zoom)
{
  if (zoom >= SPEC_ZOOM_COUNT) zoom = SPEC_ZOOM_COUNT - 1U;
  s_spec_zoom = zoom;
  draw_footer_rows(lcd, spec_half_span_hz());
}

uint8_t SDR_UI_GetSpecZoom(void) { return s_spec_zoom; }

void SDR_UI_RedrawFooter(ST7789_Handle_t *lcd)
{
  draw_footer_rows(lcd, spec_half_span_hz());
}

/* ── Spectrum skip statistics ─────────────────────────────────────────────── */
void SDR_UI_GetSpecSkipStats(uint32_t *skip_hits, uint32_t *draw_hits)
{
  if (skip_hits) *skip_hits = s_spec_skip_hits;
  if (draw_hits) *draw_hits = s_spec_draw_hits;
}

/* ── Stub ────────────────────────────────────────────────────────────────── */
void SDR_UI_DrawFuncBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{ (void)lcd; (void)ui; }
