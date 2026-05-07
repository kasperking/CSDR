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
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── Private defines ─────────────────────────────────────────────────────── */
#define BIG_W   15U   /* 3× scaled digit width  (Font6x8 col × 15/6) */
#define BIG_H   24U   /* 3× scaled digit height (Font6x8 row × 3)    */

#define WF_MIN_DB    (-120.0f)
#define WF_RANGE_DB  ( 100.0f)
#define WF_INV_RANGE (255.0f / WF_RANGE_DB)
#define WF_DB_OFFSET  42.14f
#define WF_SMOOTH_ALPHA  0.72f
/* (CROP_MARGIN removed – spectrum window is now zoom-derived) */

/* ── DMA-accessible zone buffers ─────────────────────────────────────────── */
static uint16_t s_hdr_buf[HDR_H  * LCD_W]   /* 11,520 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_sbl_buf[SBL_H  * SBL_W]   /*  9,840 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_sbr_buf[SBR_H  * SBR_W]   /*  9,840 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_vfo_buf[VFO_H  * VFO_W]   /* 14,400 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_mtr_buf[MTR_H  * MTR_W]   /* 18,400 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_spec_buf[SPEC_H][SPEC_W]  /* 43,520 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_wf_buf[WF_H][WF_W]        /* 39,680 B */
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* WF pre-compute: two uint8_t line buffers (double-buffer for DSP/UI split) */
static uint8_t  s_wf_idx[2][WF_W];          /*    640 B */
static volatile uint8_t s_wf_fill = 0;

/* CPU-only: IIR smoother + colour LUT + ring head */
static float    s_wf_smooth[256];
static uint16_t s_wf_lut[256];
static uint8_t  s_wf_head = 0;
static float    s_smeter_voltage = 0.0f;

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

/* ── spec_color: blue→green→yellow gradient ──────────────────────────────── */
static uint16_t spec_color(float norm)
{
  uint8_t r, g, b;
  if (norm < 0.40f) {
    float t = norm / 0.40f;
    r = 0; g = (uint8_t)(t * 30.0f); b = (uint8_t)((1.0f - t * 0.5f) * 31.0f);
  } else if (norm < 0.75f) {
    float t = (norm - 0.40f) / 0.35f;
    r = (uint8_t)(t * 24.0f);
    g = (uint8_t)(30.0f + t * 33.0f);
    b = (uint8_t)((0.5f - t * 0.5f) * 31.0f);
  } else {
    float t = (norm - 0.75f) / 0.25f;
    r = 31U; g = (uint8_t)((1.0f - t) * 63.0f); b = 0;
  }
  return (uint16_t)((r << 11U) | (g << 5U) | b);
}

/* ── wf_lut_init: Hermite-spline thermal palette ─────────────────────────── */
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
  wf_lut_init();
}

/* ── Spectrum zoom state ─────────────────────────────────────────────────────
 * zoom=0 → ±24kHz (full FFT)   center = bins/2 = 128
 * zoom=1 → ±12kHz              half   = 128 >> zoom
 * zoom=2 → ±6kHz               b0     = center - half
 * zoom=3 → ±3kHz               n_vis  = 2 * half
 * ─────────────────────────────────────────────────────────────────────────── */
static uint8_t  s_spec_zoom = 0U;
static uint32_t s_spec_sr   = 48000U;
static uint16_t s_spec_bins = 256U;

static void spec_window(uint16_t bins, uint16_t *b0_out, uint16_t *n_vis_out)
{
  uint16_t center = bins >> 1U;
  uint16_t half   = center >> s_spec_zoom;
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
  bool tx = ui->tx_mode;
  const char *rxtx_str = tx ? "TX" : "RX";
  uint16_t    rxtx_bg  = tx ? UI_TX_BG : UI_RX_BG;
  uint16_t    rxtx_fg  = tx ? UI_TX_FG : UI_RX_FG;

  uint16_t badge_w = (uint16_t)(Font6x8.width * 2U + 8U);
  uint16_t badge_x = 4U;

  char att_str[8];
  snprintf(att_str, sizeof(att_str), "ATT:%u", ui->att_db);
  uint16_t att_x = (uint16_t)(badge_x + badge_w + 8U);

  char vstr[8];
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

    /* TX/RX badge */
    LCD_LineFill(ln, badge_x, badge_w, rxtx_bg);
    if (row >= txt_y && row < txt_y + Font6x8.height) {
      uint16_t fr = row - txt_y;
      uint16_t tx_x = (uint16_t)(badge_x + (badge_w - (uint16_t)(strlen(rxtx_str) * Font6x8.width)) / 2U);
      LCD_LineStr(ln, tx_x, fr, rxtx_str, &Font6x8, rxtx_fg, rxtx_bg);
    }

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
          LCD_LineFill(ln, 1U, SBL_W - 1U, mbg);
          uint16_t cx = (uint16_t)(1U + (SBL_W - 1U - (uint16_t)(strlen(mode_str) * Font6x8.width)) / 2U);
          LCD_LineStr(ln, cx, fr, mode_str, &Font6x8, UI_MODE_FG, mbg);
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

  char mic_str[6]; snprintf(mic_str, sizeof(mic_str), "%d", (int)ui->mic_gain);
  char dsp_str[6]; snprintf(dsp_str, sizeof(dsp_str), "%u", ui->dsp_level);

  /* BW: ≥10k → "12k", ≥1k → "2.7k", <1k → "500Hz" */
  char bw_str[8];
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
 *  SDR_UI_DrawVFO  (VFO_W=200 × VFO_H=36)
 *
 *  Row map:
 *   0:       top border
 *   1-8:     step label (top-right, Font6x8)  ← same rows as big freq padding
 *   3-18:    big frequency digits (BIG_H=16, freq_top=3)
 *   20-27:   sub-freq or RIT label (Font6x8)
 *   35:      bottom border
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

  buf_fill(s_vfo_buf, (uint32_t)VFO_H * VFO_W, UI_VFO_BG);

  const uint16_t freq_top = 3U;
  const uint16_t step_y   = 1U;
  const uint16_t sub_y    = (uint16_t)(freq_top + BIG_H + 1U);  /* 28 */
  uint16_t step_x = (uint16_t)(VFO_W - (uint16_t)(strlen(step_str) * Font6x8.width) - 3U);

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

    /* Active VFO label: top-left corner ("A" or "B") */
    if (row >= step_y && (row - step_y) < Font6x8.height) {
      const char *vl = (ui->active_vfo == 0U) ? "A" : "B";
      uint16_t vc    = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_ON;
      LCD_LineStr(ln, 2U, row - step_y, vl, &Font6x8, vc, UI_VFO_BG);
    }

    /* Step label: top-right corner */
    if (row >= step_y && (row - step_y) < Font6x8.height)
      LCD_LineStr(ln, step_x, row - step_y, step_str, &Font6x8, UI_FREQ_KHZ, UI_VFO_BG);

    /* Sub-freq / RIT */
    if (sub_str[0] && row >= sub_y && (row - sub_y) < Font6x8.height)
      LCD_LineStr(ln, 4U, row - sub_y, sub_str, &Font6x8, UI_FREQ_SUB, UI_VFO_BG);
  }

  ST7789_PushWindow(lcd, VFO_X, (uint16_t)(VFO_X + VFO_W - 1U),
                    VFO_Y, VFO_Y2 - 1U, s_vfo_buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  draw_smeter_rows  – fills s_mtr_buf for RX S-meter
 *
 *  MTR_H=46 row map:
 *   0:         top border
 *   1-8:       scale labels  ("S 1 3 5 7 9 +20 +40")
 *   9-10:      tick marks
 *  11-30:      bar (SM_BAR_YOFF=11, SM_BAR_H=20)
 *  32-39:      value text "S7  –87 dBm" + voltage
 *  45:         bottom border / main-spectrum divider
 * ════════════════════════════════════════════════════════════════════════════ */
static void draw_smeter_rows(int32_t bars)
{
  /* Thin horizontal radio-style S-meter:
   *  rows  1-8 : scale labels
   *  row   9   : tick marks
   *  rows 10-12: 3-px thin bar (smooth fill, colour-coded)
   *  rows 14-21: value text
   */
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

    /* Scale labels: rows 1-8 */
    if (row >= 1U && row < 1U + Font6x8.height) {
      uint16_t fr = row - 1U;
      for (uint8_t t = 0; t < 8U; t++) {
        uint16_t lx  = (uint16_t)(SM_START_X + (uint16_t)sbar[t] * (SM_BAR_W + SM_BAR_GAP));
        /* "+20" (t=6) is 18 px wide but only 14 px before "+40" — shift left
         * 4 px so it ends at x=157, leaving a 1 px gap before "+40" at 158. */
        if (t == 6U) { lx -= 4U; }
        uint16_t col = (t < 6U) ? UI_SMETER_TICK : UI_S1_6;
        LCD_LineStr(ln, lx, fr, slbls[t], &Font6x8, col, UI_MTR_BG);
      }
    }

    /* Tick marks: row 9 */
    if (row == 9U) {
      for (uint8_t t = 0; t < 8U; t++) {
        uint16_t tx = (uint16_t)(SM_START_X + (uint16_t)sbar[t] * (SM_BAR_W + SM_BAR_GAP));
        if (tx < MTR_W) ln[tx] = SWAP16(UI_SMETER_TICK);
      }
    }

    /* Thin bar: rows 10-12 */
    if (row >= 10U && row < 13U) {
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

    /* Value text: rows 14-21 */
    if (row >= 14U && (row - 14U) < Font6x8.height) {
      uint16_t fr   = row - 14U;
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
  draw_smeter_rows(bars);
  ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                    MTR_Y, MTR_Y2 - 1U, s_mtr_buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_UpdateSMeter  – fast meter refresh (10 Hz, RX)
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdateSMeter_SetTX(bool tx)      { (void)tx; }
void SDR_UI_UpdateSMeter_SetVoltage(float v) { s_smeter_voltage = v; }

void SDR_UI_UpdateSMeter(ST7789_Handle_t *lcd, float signal_db)
{
  int32_t bars = (int32_t)((signal_db + 73.0f) / 3.0f);
  if (bars < 0) bars = 0;
  if (bars > (int32_t)SM_BARS) bars = (int32_t)SM_BARS;
  draw_smeter_rows(bars);
  ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                    MTR_Y, MTR_Y2 - 1U, s_mtr_buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_UpdateTXMeters  – ALC (top half) + SWR (bottom half)
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdateTXMeters(ST7789_Handle_t *lcd, float alc_norm, float swr)
{
  /* TX layout – ALC is the primary thin-bar element; SWR is compact text only.
   *  rows  1-8 : "ALC XX%" label
   *  row   9   : tick marks
   *  rows 10-12: 3-px thin ALC bar
   *  rows 14-21: "ALC XX%" value text
   *  rows 24-31: "SWR X.X" compact text, no bar
   */
  int32_t alc_b = (int32_t)(alc_norm * (float)SM_BARS + 0.5f);
  if (alc_b < 0) alc_b = 0;
  if (alc_b > (int32_t)SM_BARS) alc_b = (int32_t)SM_BARS;
  uint16_t alc_fill = (uint16_t)(SM_START_X + (uint16_t)alc_b * (SM_BAR_W + SM_BAR_GAP));
  uint16_t x_end    = (uint16_t)(SM_START_X + SM_TOTAL_W);

  char alc_val[6]; snprintf(alc_val, sizeof(alc_val), "%d%%", (int)(alc_norm * 100.0f + 0.5f));
  char swr_val[6]; snprintf(swr_val, sizeof(swr_val), "%.1f", swr);
  uint16_t swr_col = (swr >= 3.0f) ? UI_S9P : (swr >= 2.0f) ? UI_S7_9 : UI_S1_6;

  static const char *const slbls[] = {"0","25","50","75","100"};
  static const uint8_t     spos[]  = { 0,   3,   6,   9,  11 };

  for (uint16_t row = 0; row < MTR_H; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0, MTR_W, UI_MTR_BG);

    /* Scale labels: rows 1-8 */
    if (row >= 1U && row < 1U + Font6x8.height) {
      uint16_t fr = row - 1U;
      for (uint8_t t = 0; t < 5U; t++) {
        uint16_t lx = (uint16_t)(SM_START_X + (uint16_t)spos[t] * (SM_BAR_W + SM_BAR_GAP));
        LCD_LineStr(ln, lx, fr, slbls[t], &Font6x8, UI_SMETER_TICK, UI_MTR_BG);
      }
    }

    /* Tick marks: row 9 */
    if (row == 9U) {
      for (uint8_t t = 0; t < 5U; t++) {
        uint16_t tx = (uint16_t)(SM_START_X + (uint16_t)spos[t] * (SM_BAR_W + SM_BAR_GAP));
        if (tx < MTR_W) ln[tx] = SWAP16(UI_SMETER_TICK);
      }
    }

    /* Thin ALC bar: rows 10-12 */
    if (row >= 10U && row < 13U) {
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

    /* ALC value text: rows 14-21 */
    if (row >= 14U && (row - 14U) < Font6x8.height) {
      uint16_t fr = row - 14U;
      LCD_LineStr(ln, SM_START_X, fr, "ALC", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, SM_START_X + 4U * Font6x8.width, fr, alc_val,
                  &Font6x8, UI_STATUS_VAL, UI_MTR_BG);
    }

    /* SWR compact text: rows 24-31 – no bar, label + value only */
    if (row >= 24U && (row - 24U) < Font6x8.height) {
      uint16_t fr = row - 24U;
      LCD_LineStr(ln, SM_START_X, fr, "SWR", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, SM_START_X + 4U * Font6x8.width, fr, swr_val,
                  &Font6x8, swr_col, UI_MTR_BG);
    }
  }

  ST7789_PushWindow(lcd, MTR_X, (uint16_t)(MTR_X + MTR_W - 1U),
                    MTR_Y, MTR_Y2 - 1U, s_mtr_buf);
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

  uint16_t bar_h[SPEC_W];
  for (uint16_t x = 0; x < SPEC_W; x++) {
    uint16_t bi = (uint16_t)((float)x * bpp) + b0;
    if (bi >= bins) bi = bins - 1U;
    bar_h[x] = (uint16_t)(pwr_compress(fft_db[bi]) * (float)SPEC_H);
  }

  uint16_t g1  = (uint16_t)(SPEC_H - (uint16_t)(0.75f * (float)SPEC_H));
  uint16_t g2  = (uint16_t)(SPEC_H - (uint16_t)(0.50f * (float)SPEC_H));
  uint16_t g3  = (uint16_t)(SPEC_H - (uint16_t)(0.25f * (float)SPEC_H));
  uint16_t cx  = SPEC_W / 2U;

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

  for (uint16_t y = 0; y < SPEC_H; y++) {
    uint16_t *row     = s_spec_buf[y];
    bool      is_grid = (y == g1 || y == g2 || y == g3);
    uint16_t  bg_sw   = is_grid ? SWAP16(UI_SPEC_GRID) : SWAP16(UI_SPEC_BG);

    for (uint16_t x = 0; x < SPEC_W; x++) row[x] = bg_sw;
    if (!is_grid) for (uint16_t gx = 0; gx < SPEC_W; gx += 40U) row[gx] = SWAP16(UI_SPEC_GRID);

    if (bw_lo_ok && bw_lo < SPEC_W) row[bw_lo] = SWAP16(UI_SPEC_BW);
    if (bw_hi_ok && bw_hi < SPEC_W) row[bw_hi] = SWAP16(UI_SPEC_BW);
    if (cx < SPEC_W)                 row[cx]     = SWAP16(UI_SPEC_CENTER);

    uint16_t y_from_bot = (uint16_t)(SPEC_H - 1U - y);
    for (uint16_t x = 0; x < SPEC_W; x++)
      if (bar_h[x] > 0U && y_from_bot < bar_h[x])
        row[x] = SWAP16(spec_color((float)y_from_bot / (float)SPEC_H));
  }

  /* Last row is always the spectrum/waterfall divider */
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
 *  zoom=0 ±24kHz | zoom=1 ±12kHz | zoom=2 ±6kHz | zoom=3 ±3kHz
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

/* ── Stub ────────────────────────────────────────────────────────────────── */
void SDR_UI_DrawFuncBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui)
{ (void)lcd; (void)ui; }
