/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_ui.c
  * @brief   CSDR SDR UI – 9-zone layout, FMC burst pushes (480×320)
  *
  *  All zone buffers live in DMA_SRAM (RAM_D1, 512KB).  Cache-coherency is
  *  handled at the FMC MPU region level – Region 1 is Strongly-Ordered,
  *  so no D-Cache flush is needed before pushing.
  *
  *  Spectrum and waterfall zones use LCD_PushWindowAsync (DMA2 Stream0 M2M).
  *  CPU still handles CASET/RASET/RAMWR window setup synchronously, then DMA
  *  carries the pixel payload to the fixed FMC data address.
  *
  *  Async strip sequence: LCD_Wait() → LCD_PushWindowAsync() → MainLoopBeat().
  *  Audio/USB ISRs preempt freely during the cooperative NOP poll in LCD_Wait.
  *  Other zones (header, sidebar, VFO, meter) retain synchronous LCD_PushWindow.
  *
  *  Single-zone worst-case transfer times (8-bit FMC, 116.7 ns/byte):
  *    Spectrum  (480×72 = 69,120 px = 138,240 B): ~8.06 ms (9 × 8-row strips)
  *    Waterfall (480×1  =    480 px =     960 B): ~112 µs  (1-row ring push)
  *  Waterfall uses a single-row ring-slot push — no chunking needed.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sdr_ui.h"
#include "runtime_diag.h"
#include "lcd_dma.h"    /* LCD_Wait / LCD_PushWindowAsync / diagnostics */
#include "core_cm7.h"   /* DWT->CYCCNT for chunk render timing */
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── Private defines ─────────────────────────────────────────────────────── */
/* 7-segment VFO digit cell: 18×24 px.  Stroke 2 px, gap 1 px at corners.
 * Segments: a(top) b(UR) c(LR) d(bot) e(LL) f(UL) g(mid) */
#define BIG_W   18U   /* digit cell width  */
#define BIG_H   24U   /* digit cell height */

#define MED_W   12U   /* 2× scaled digit width  (6 × 2) */
#define MED_H   16U   /* 2× scaled digit height (8 × 2) */

#define WF_MIN_DB    (-120.0f)
#define WF_RANGE_DB  ( 100.0f)
#define WF_INV_RANGE (255.0f / WF_RANGE_DB)
#define WF_DB_OFFSET  30.0f
#define WF_SMOOTH_ALPHA  0.72f

/* ── Chunked LCD push parameters ────────────────────────────────────────────
 * SPEC_CHUNK_ROWS: spectrum is still pushed in 8-row strips with abort
 * capability.  At 8-bit FMC (116.7 ns/byte): 8 rows × 480 px × 2 B = 7,680 B
 * → ~896 µs per strip.
 *
 * WF_CHUNK_ROWS is no longer used: the waterfall now uses a single-row
 * ring-slot push (~112 µs total), so chunking and overload abort are
 * unnecessary for the waterfall path. */
#define SPEC_CHUNK_ROWS  8U

/* Debugger-visible LCD chunk statistics.
 * Also reported to runtime_diag snapshot via RuntimeDiag_LcdChunkReport(). */
static volatile uint32_t s_lcd_chunk_count       = 0U; /*!< Total strips pushed since boot  */
static volatile uint32_t s_lcd_chunk_abort_count = 0U; /*!< Waterfall renders aborted early */
static volatile uint32_t s_wf_partial_count      = 0U; /*!< Waterfall frames partially done */
static volatile uint32_t s_max_chunk_render_us   = 0U; /*!< Peak µs for a single strip      */

static inline uint32_t ui_cyc_to_us(uint32_t cycles)
{
  uint32_t mhz = SystemCoreClock / 1000000U;
  return (mhz > 0U) ? (cycles / mhz) : 0U;
}

/* ── DMA-accessible zone buffers (RAM_D1, 512 KB) ───────────────────────── */
/* Sizes at 480×320:
 *   s_hdr_buf  : 24 × 480 × 2 =  23,040 B
 *   s_sbl_buf  : 96 × 80  × 2 =  15,360 B
 *   s_sbr_buf  : 96 × 80  × 2 =  15,360 B
 *   s_vfo_buf  : 64 × 320 × 2 =  40,960 B
 *   s_mtr_buf  : 32 × 320 × 2 =  20,480 B
 *   s_spec_buf : 72 × 480 × 2 =  69,120 B
 *   s_wf_buf   : 72 × 480 × 2 =  69,120 B
 *   Total UI                  : ~253 KB  (fits in 512 KB RAM_D1)
 */
static uint16_t s_hdr_buf[HDR_H  * LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_sbl_buf[SBL_H  * SBL_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_sbr_buf[SBR_H  * SBR_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_vfo_buf[VFO_H  * VFO_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_mtr_buf[MTR_H  * MTR_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
/* Strip buffer for dirty-column segment push: SM_SEG_H × SM_RULER_W = 13 × 168 = 2,184 px */
static uint16_t s_mtr_strip[SM_SEG_H * SM_RULER_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_spec_buf[SPEC_H][SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_wf_buf[WF_H][WF_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* Spectrum compute arrays — static (avoid 4.8 KB stack frame per call) */
static float    s_spec_yf     [SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_spec_py     [SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
/* Strip buffer: one SPEC_CHUNK_ROWS-tall slice for partial-column FMC pushes.
 * Worst case: SPEC_CHUNK_ROWS × SPEC_W = 8 × 480 = 3,840 pixels = 7,680 B. */
static uint16_t s_spec_strip[SPEC_CHUNK_ROWS * SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* WF pre-compute: two uint8_t line buffers (double-buffer for DSP/UI split) */
static uint8_t  s_wf_idx[2][WF_W];   /* 2 × 480 B */
static volatile uint8_t s_wf_fill = 0;

/* Adaptive waterfall suppression (set by csdr_app under high audio load) */
static volatile bool s_wf_suppressed = false;

/* CPU-only: IIR smoother + colour LUT + ring head */
static float    s_wf_smooth[256];    /* indexed by FFT bin (max 256) */
static uint16_t s_wf_lut[256];
static uint8_t  s_wf_head = 0;
static float    s_smeter_voltage = 0.0f;

/* Spectrum delta-skip: previous column pixel rows */
static uint16_t s_spec_py_prev[SPEC_W];
static bool     s_spec_py_valid = false;

static uint32_t s_spec_skip_hits    = 0U;
static uint32_t s_spec_draw_hits    = 0U;
/* Spectrum partial-push diagnostics */
static uint32_t s_spec_partial_count   = 0U;
static uint32_t s_max_spec_partial_us  = 0U;
/* VFO glyph-level push diagnostics */
static uint32_t s_vfo_glyph_count  = 0U;
static uint32_t s_vfo_skip_count   = 0U;
static uint32_t s_max_vfo_us       = 0U;
/* S-meter label/tick positions: segment indices where markers appear */
static const uint8_t s_sbar[8] = { 0U, 1U, 3U, 5U, 7U, 9U, 10U, 11U };

static int32_t  s_rx_meter_bars = -1;
static bool     s_tx_meter_active = false;
static int32_t  s_tx_alc_bars = -1;
static int32_t  s_tx_alc_pct = -1;
static int32_t  s_tx_swr_x10 = -1;
static bool     s_mtr_static_valid = false;

/* VFO section-split + glyph-level cache */
static struct {
  uint32_t freq_hz;
  uint32_t freq_b_hz;
  int16_t  rit_hz;
  bool     tx_mode;
  uint8_t  active_vfo;
  bool     valid;
  /* Glyph-level cache for upper-section partial push */
  char     mhz_s[6];
  char     khz_s[4];
  char     hz_s[4];
  uint16_t fx_base;
} s_vfo_cache;

/* Right-sidebar value cache — skip push when nothing changed */
static struct {
  uint32_t bw_hz;
  uint32_t step;
  int16_t  mic_gain;
  uint8_t  att_db;
  uint8_t  dsp_level;
  bool     valid;
} s_sbr_cache;

/* Left-sidebar value cache */
static struct {
  uint8_t  mode;
  uint8_t  volume;
  uint8_t  squelch;
  bool     nr_on;
  bool     nb_on;
  uint8_t  active_vfo;
  bool     valid;
} s_sbl_cache;

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

/* ── wf_lut_init: Hermite-spline SDR waterfall palette ──────────────────── *
 *  black → dark-navy → blue → cyan → yellow → white, gamma=1.3             */
static void wf_lut_init(void)
{
  static const float sr[11] = { 0,  0,  0,  0,  0,  0,  0, 14, 31, 31, 31};
  static const float sg[11] = { 0,  0,  1,  4, 10, 38, 63, 63, 63, 63, 63};
  static const float sb[11] = { 0,  3, 12, 22, 31, 31, 26,  8,  0, 16, 31};
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
    float ng = powf(n, 1.3f);
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

/* ── 7-segment VFO digit rendering ──────────────────────────────────────────
 * Segment mask bits: 0=a(top) 1=b(UR) 2=c(LR) 3=d(bot) 4=e(LL) 5=f(UL) 6=g(mid)
 * Layout in BIG_W×BIG_H cell (18×24):
 *   rows  0- 1 : horizontal a (cols 2-15)
 *   rows  2-10 : verticals f (cols 0-1), b (cols 16-17)
 *   rows 11-12 : horizontal g (cols 2-15)
 *   rows 13-21 : verticals e (cols 0-1), c (cols 16-17)
 *   rows 22-23 : horizontal d (cols 2-15)
 */
static const uint8_t s_seg7[10] = {
    0x3F, /* 0 */  0x06, /* 1 */  0x5B, /* 2 */  0x4F, /* 3 */  0x66, /* 4 */
    0x6D, /* 5 */  0x7D, /* 6 */  0x07, /* 7 */  0x7F, /* 8 */  0x6F, /* 9 */
};

static void ln_segchar(uint16_t *ln, uint16_t x, uint16_t fr,
                       char c, uint16_t fg, uint16_t bg)
{
  uint16_t pfg = SWAP16(fg), pbg = SWAP16(bg);
  for (uint16_t i = 0; i < BIG_W; i++) ln[x + i] = pbg;
  if (c < '0' || c > '9') return;
  uint8_t seg = s_seg7[(uint8_t)(c - '0')];
  if (fr < 2U) {
    if ((seg >> 0) & 1U) for (uint16_t i = 2U; i < 16U; i++) ln[x + i] = pfg;
  } else if (fr < 11U) {
    if ((seg >> 5) & 1U) { ln[x + 0U]  = pfg; ln[x + 1U]  = pfg; }
    if ((seg >> 1) & 1U) { ln[x + 16U] = pfg; ln[x + 17U] = pfg; }
  } else if (fr < 13U) {
    if ((seg >> 6) & 1U) for (uint16_t i = 2U; i < 16U; i++) ln[x + i] = pfg;
  } else if (fr < 22U) {
    if ((seg >> 4) & 1U) { ln[x + 0U]  = pfg; ln[x + 1U]  = pfg; }
    if ((seg >> 2) & 1U) { ln[x + 16U] = pfg; ln[x + 17U] = pfg; }
  } else {
    if ((seg >> 3) & 1U) for (uint16_t i = 2U; i < 16U; i++) ln[x + i] = pfg;
  }
}

static void ln_segstr(uint16_t *ln, uint16_t x, uint16_t fr,
                      const char *s, uint16_t fg, uint16_t bg)
{
  uint16_t pfg = SWAP16(fg), pbg = SWAP16(bg);
  while (*s) {
    if (*s == '.') {
      /* Decimal separator: 4×4 px dot in bottom of 6px-wide cell */
      for (uint16_t i = 0; i < 6U; i++) ln[x + i] = pbg;
      if (fr >= BIG_H - 4U) {
        ln[x + 1U] = pfg; ln[x + 2U] = pfg;
        ln[x + 3U] = pfg; ln[x + 4U] = pfg;
      }
      x += 6U;
    } else {
      ln_segchar(ln, x, fr, *s, fg, bg);
      x += BIG_W;
    }
    s++;
  }
}

/* ── 2× scaled medium digit rendering ────────────────────────────────────── */
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
  s_wf_head = (uint8_t)(WF_H - 1U); s_wf_fill = 0; /* wraps to 0 on first push → top row */
  s_spec_py_valid      = false;
  s_spec_skip_hits     = 0U;
  s_spec_draw_hits     = 0U;
  s_spec_partial_count = 0U;
  s_max_spec_partial_us= 0U;
  s_vfo_glyph_count    = 0U;
  s_vfo_skip_count     = 0U;
  s_max_vfo_us         = 0U;
  s_mtr_static_valid   = false;
  s_vfo_cache.valid    = false;
  s_sbr_cache.valid    = false;
  s_sbl_cache.valid    = false;
  s_wf_suppressed      = false;
  wf_lut_init();
}

/* ── Waterfall suppression API ───────────────────────────────────────────── */
void SDR_UI_SetWaterfallSuppressed(bool suppressed)
{
  s_wf_suppressed = suppressed;
}

bool SDR_UI_GetWaterfallSuppressed(void)
{
  return s_wf_suppressed;
}

/* ── Spectrum zoom state ─────────────────────────────────────────────────────
 * zoom=0 → ±24kHz   half=128
 * zoom=1 → ±18kHz   half= 96
 * zoom=2 → ±12kHz   half= 64
 * zoom=3 → ±6kHz    half= 32
 * zoom=4 → ±3kHz    half= 16
 * ─────────────────────────────────────────────────────────────────────────── */
static uint8_t  s_spec_zoom = 0U;
static uint32_t s_spec_sr   = 48000U;
static uint16_t s_spec_bins = 256U;

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

/* ── draw_footer_rows ────────────────────────────────────────────────────────
 * Renders FTR_H scanlines into the shared line buffer (480px) and pushes each
 * as a 1-row window.  Footer is 32 rows at 480px = 30,720 bytes total,
 * which at 116.7 ns/byte = ~3.6 ms — well within the audio budget.
 * ─────────────────────────────────────────────────────────────────────────── */
static void draw_footer_rows(uint32_t half_hz)
{
  char lbuf[12] = "";
  char rbuf[12] = "";
  uint16_t rx_x = 0U;
  if (half_hz > 0U) {
    uint32_t bk = half_hz / 1000U;
    snprintf(lbuf, sizeof(lbuf), "-%luK", (unsigned long)bk);
    snprintf(rbuf, sizeof(rbuf), "+%luK", (unsigned long)bk);
    rx_x = (uint16_t)(LCD_W - (uint16_t)(strlen(rbuf) * Font6x8.width) - 4U);
  }
  uint16_t fh  = Font6x8.height;
  uint16_t pad = (FTR_H - fh) / 2U;

  for (uint16_t row = 0U; row < FTR_H; row++) {
    uint16_t *ln = LCD_GetLineBuf();
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    if (half_hz > 0U && row >= pad && row < pad + fh) {
      uint16_t frow = row - pad;
      LCD_LineStr(ln, 4U, frow, lbuf, &Font6x8, UI_SPEC_GRID, UI_BG);
      LCD_LineStr(ln, (uint16_t)(LCD_W / 2U - Font6x8.width / 2U), frow,
                  "0", &Font6x8, UI_SPEC_GRID, UI_BG);
      LCD_LineStr(ln, rx_x, frow, rbuf, &Font6x8, UI_SPEC_GRID, UI_BG);
    }
    LCD_PushWindow(0U, (uint16_t)(FTR_Y + row),
                   (uint16_t)(LCD_W - 1U), (uint16_t)(FTR_Y + row),
                   ln, LCD_W);
  }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawFrame  – one-time skeleton
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawFrame(uint32_t sample_rate, uint16_t fft_bins)
{
  s_spec_sr   = sample_rate ? sample_rate : 48000U;
  s_spec_bins = fft_bins    ? fft_bins    : 256U;

  LCD_Clear(UI_BG);
  draw_footer_rows(spec_half_span_hz());
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawHeader  (HDR_H=24 rows × LCD_W=480 cols)
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawHeader(const SDR_UI_State_t *ui)
{
  char vstr[12];
  snprintf(vstr, sizeof(vstr), "%.1fV", ui->voltage);
  uint16_t vcol    = (ui->voltage < 11.5f && ui->voltage > 0.5f)
                     ? UI_STATUS_OFF : UI_STATUS_VAL;
  uint16_t volt_x  = (uint16_t)(LCD_W - (uint16_t)(strlen(vstr) * Font6x8.width) - 4U);
  uint16_t txt_y   = (uint16_t)((HDR_H - Font6x8.height) / 2U);

  for (uint16_t row = 0; row < HDR_H; row++) {
    uint16_t *ln = s_hdr_buf + (uint32_t)row * LCD_W;
    if (row == HDR_H - 1U) {
      LCD_LineFill(ln, 0, LCD_W, UI_DIVIDER);
      continue;
    }
    LCD_LineFill(ln, 0, LCD_W, UI_HDR_BG);
    if (row >= txt_y && row < txt_y + Font6x8.height) {
      uint16_t fr = row - txt_y;
      LCD_LineStr(ln, volt_x, fr, vstr, &Font6x8, vcol, UI_HDR_BG);
    }
  }

  LCD_PushWindow(0U, HDR_Y, (uint16_t)(LCD_W - 1U), HDR_Y2 - 1U,
                 s_hdr_buf, (uint32_t)LCD_W * HDR_H);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSidebarLeft  (SBL_W=80 × SBL_H=96)
 *
 *  5 items × ~19 rows each:
 *   0: Mode        1: VFO A/B   2: NR/NB
 *   3: VOL         4: SQL
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSidebarLeft(const SDR_UI_State_t *ui)
{
  /* Cache guard — skip rebuild when nothing changed */
  if (s_sbl_cache.valid
      && s_sbl_cache.mode       == ui->mode
      && s_sbl_cache.volume     == ui->volume
      && s_sbl_cache.squelch    == ui->squelch
      && s_sbl_cache.nr_on      == ui->nr_on
      && s_sbl_cache.nb_on      == ui->nb_on
      && s_sbl_cache.active_vfo == ui->active_vfo) return;

  s_sbl_cache.mode       = ui->mode;
  s_sbl_cache.volume     = ui->volume;
  s_sbl_cache.squelch    = ui->squelch;
  s_sbl_cache.nr_on      = ui->nr_on;
  s_sbl_cache.nb_on      = ui->nb_on;
  s_sbl_cache.active_vfo = ui->active_vfo;
  s_sbl_cache.valid      = true;

  static const char *const mode_s[]  = {"AM","FM","USB","LSB","CW"};
  static const uint16_t    mode_bg[] = {UI_MODE_AM,UI_MODE_FM,UI_MODE_USB,
                                         UI_MODE_LSB,UI_MODE_CW};
  const char *mode_str = (ui->mode < 5U) ? mode_s[ui->mode]  : "---";
  uint16_t    mbg      = (ui->mode < 5U) ? mode_bg[ui->mode] : UI_STATUS_LBL;

  char vol_str[6]; snprintf(vol_str, sizeof(vol_str), "%u", ui->volume);
  char sql_str[6]; snprintf(sql_str, sizeof(sql_str), "%u", ui->squelch);

  buf_fill(s_sbl_buf, (uint32_t)SBL_H * SBL_W, UI_SBL_BG);

  const uint16_t item_h = 19U;   /* 5 items × 19 = 95, +1 row blank at top */

  for (uint8_t i = 0; i < 5U; i++) {
    uint16_t y0 = (uint16_t)(1U + (uint32_t)i * item_h);

    if (i > 0U)
      for (uint16_t x = 0; x < SBL_W; x++)
        s_sbl_buf[(uint32_t)y0 * SBL_W + x] = SWAP16(UI_DIVIDER);

    uint16_t text_y = y0 + 4U;

    switch (i) {
      case 0:
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          uint16_t cx = (uint16_t)(2U +
              (SBL_W - 2U - (uint16_t)(strlen(mode_str) * Font6x8.width)) / 2U);
          LCD_LineStr(ln, cx, fr, mode_str, &Font6x8, mbg, UI_SBL_BG);
        }
        break;

      case 1: {
        uint16_t col_a = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_LBL;
        uint16_t col_b = (ui->active_vfo == 1U) ? UI_STATUS_ON  : UI_STATUS_LBL;
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStr(ln, 2U,                       fr, "VFO", &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          LCD_LineStr(ln, 2U + 4U * Font6x8.width,  fr, "A",   &Font6x8, col_a, UI_SBL_BG);
          LCD_LineStr(ln, 2U + 5U * Font6x8.width,  fr, "/",   &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          LCD_LineStr(ln, 2U + 6U * Font6x8.width,  fr, "B",   &Font6x8, col_b, UI_SBL_BG);
        }
        break;
      }

      case 2: {
        uint16_t nr_bg = ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF;
        uint16_t nb_bg = ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF;
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineFill(ln,  2U, 34U, nr_bg);
          LCD_LineStr(ln,   9U, fr, "NR", &Font6x8, UI_BG, nr_bg);
          LCD_LineFill(ln, 40U, 34U, nb_bg);
          LCD_LineStr(ln,  47U, fr, "NB", &Font6x8, UI_BG, nb_bg);
        }
        break;
      }

      case 3:
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStr(ln, 2U, fr, "VOL", &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          uint16_t vx = (uint16_t)(SBL_W - 1U - (uint16_t)(strlen(vol_str) * Font6x8.width) - 3U);
          LCD_LineStr(ln, vx, fr, vol_str, &Font6x8, UI_STATUS_VAL, UI_SBL_BG);
        }
        break;

      case 4:
        for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStr(ln, 2U, fr, "SQL", &Font6x8, UI_STATUS_LBL, UI_SBL_BG);
          uint16_t vx = (uint16_t)(SBL_W - 1U - (uint16_t)(strlen(sql_str) * Font6x8.width) - 3U);
          LCD_LineStr(ln, vx, fr, sql_str, &Font6x8, UI_STATUS_VAL, UI_SBL_BG);
        }
        break;

      default: break;
    }
  }

  LCD_PushWindow(SBL_X, SBL_Y,
                 (uint16_t)(SBL_X + SBL_W - 1U), SBL_Y2 - 1U,
                 s_sbl_buf, (uint32_t)SBL_W * SBL_H);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSidebarRight  (SBR_W=80 × SBR_H=96)
 *
 *  3 paired rows — more compact than the old 5-row stack:
 *    Row 0: BW  <val>   |  ST  <val>
 *    Row 1: MIC <val>   |  AT  <val>
 *    Row 2: DSP <val>
 *
 *  Column geometry (both cols 37 px wide, 6 px gutter):
 *    Left  col: label at x=2,  value right-aligned to x=38
 *    Right col: label at x=42, value right-aligned to x=77
 *
 *  row_h=26: 3×26=78 px, 4 px top pad → 82 px used, 14 px headroom.
 *  Thin separator between rows (1 px at top of rows 1 and 2).
 *  Cache guard: skip buffer rebuild + push when values unchanged.
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSidebarRight(const SDR_UI_State_t *ui)
{
  /* Cache guard — avoid rebuild when nothing changed */
  if (s_sbr_cache.valid
      && s_sbr_cache.bw_hz    == ui->bw_hz
      && s_sbr_cache.step     == ui->step
      && s_sbr_cache.mic_gain == ui->mic_gain
      && s_sbr_cache.att_db   == ui->att_db
      && s_sbr_cache.dsp_level== ui->dsp_level) return;

  s_sbr_cache.bw_hz    = ui->bw_hz;
  s_sbr_cache.step     = ui->step;
  s_sbr_cache.mic_gain = ui->mic_gain;
  s_sbr_cache.att_db   = ui->att_db;
  s_sbr_cache.dsp_level= ui->dsp_level;
  s_sbr_cache.valid    = true;

  /* Format strings */
  char bw_str[10];
  if (ui->bw_hz >= 10000U)
    snprintf(bw_str, sizeof(bw_str), "%luk", (unsigned long)(ui->bw_hz / 1000U));
  else if (ui->bw_hz >= 1000U)
    snprintf(bw_str, sizeof(bw_str), "%lu.%luk",
             (unsigned long)(ui->bw_hz / 1000U),
             (unsigned long)((ui->bw_hz % 1000U) / 100U));
  else
    snprintf(bw_str, sizeof(bw_str), "%luHz", (unsigned long)ui->bw_hz);

  char step_str[8];
  uint32_t st = ui->step;
  if      (st >= 100000U) snprintf(step_str, sizeof(step_str), "100k");
  else if (st >=  10000U) snprintf(step_str, sizeof(step_str), "10k");
  else if (st >=   1000U) snprintf(step_str, sizeof(step_str), "1k");
  else if (st >=    100U) snprintf(step_str, sizeof(step_str), "100");
  else if (st >=     10U) snprintf(step_str, sizeof(step_str), "10");
  else                    snprintf(step_str, sizeof(step_str), "1");

  char mic_str[8]; snprintf(mic_str, sizeof(mic_str), "%d",  (int)ui->mic_gain);
  char dsp_str[6]; snprintf(dsp_str, sizeof(dsp_str), "%u",  ui->dsp_level);
  char att_str[8]; snprintf(att_str, sizeof(att_str), "%udB", ui->att_db);

  buf_fill(s_sbr_buf, (uint32_t)SBR_H * SBR_W, UI_SBR_BG);

  /*
   * Paired row layout:
   *   rows[3][2] — { {left_lbl,left_val,left_vc}, {right_lbl,right_val,right_vc} }
   *   right_lbl==NULL → single item spanning full row (row 2, DSP only)
   */
  struct { const char *lbl; const char *val; uint16_t vc; } rows[3][2] = {
    { { "BW",  bw_str,   UI_FREQ_KHZ   }, { "ST",  step_str, UI_FREQ_KHZ   } },
    { { "MIC", mic_str,  UI_STATUS_VAL }, { "AT",  att_str,  UI_STATUS_VAL } },
    { { "DSP", dsp_str,  UI_STATUS_VAL }, { NULL,  NULL,     0U            } },
  };

  const uint16_t row_h   = 26U;
  const uint16_t top_pad =  4U;
  /* text vertically centred within row_h: (26-8)/2=9 px from row top */
  const uint16_t txt_off =  9U;

  /* Column geometry */
  const uint16_t L_LBL_X = 2U;   /* left col label  */
  const uint16_t L_VAL_X = 38U;  /* left col val right-edge */
  const uint16_t R_LBL_X = 42U;  /* right col label */
  const uint16_t R_VAL_X = 77U;  /* right col val right-edge */

  for (uint8_t i = 0; i < 3U; i++) {
    uint16_t y0 = (uint16_t)(top_pad + (uint32_t)i * row_h);

    /* Thin separator above rows 1 and 2 */
    if (i > 0U) {
      for (uint16_t x = 3U; x < (uint16_t)(SBR_W - 3U); x++)
        s_sbr_buf[(uint32_t)y0 * SBR_W + x] = SWAP16(UI_DIVIDER);
    }

    uint16_t txt_y = (uint16_t)(y0 + txt_off);

    /* Left item */
    {
      const char *lbl = rows[i][0].lbl;
      const char *val = rows[i][0].val;
      uint16_t    vc  = rows[i][0].vc;
      uint16_t val_x  = (uint16_t)(L_VAL_X - (uint16_t)(strlen(val) * Font6x8.width));
      for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
        uint16_t r = (uint16_t)(txt_y + fr);
        if (r >= SBR_H) break;
        uint16_t *ln = s_sbr_buf + (uint32_t)r * SBR_W;
        LCD_LineStr(ln, L_LBL_X, fr, lbl, &Font6x8, UI_STATUS_LBL, UI_SBR_BG);
        LCD_LineStr(ln, val_x,   fr, val, &Font6x8, vc,             UI_SBR_BG);
      }
    }

    /* Right item (optional) */
    if (rows[i][1].lbl != NULL) {
      const char *lbl = rows[i][1].lbl;
      const char *val = rows[i][1].val;
      uint16_t    vc  = rows[i][1].vc;
      uint16_t val_x  = (uint16_t)(R_VAL_X - (uint16_t)(strlen(val) * Font6x8.width));
      for (uint16_t fr = 0; fr < Font6x8.height; fr++) {
        uint16_t r = (uint16_t)(txt_y + fr);
        if (r >= SBR_H) break;
        uint16_t *ln = s_sbr_buf + (uint32_t)r * SBR_W;
        LCD_LineStr(ln, R_LBL_X, fr, lbl, &Font6x8, UI_STATUS_LBL, UI_SBR_BG);
        LCD_LineStr(ln, val_x,   fr, val, &Font6x8, vc,             UI_SBR_BG);
      }
    }
  }

  LCD_PushWindow(SBR_X, SBR_Y,
                 (uint16_t)(SBR_X + SBR_W - 1U), SBR_Y2 - 1U,
                 s_sbr_buf, (uint32_t)SBR_W * SBR_H);
}

/* Forward declaration — defined before SDR_UI_DrawSpectrum further below */
static void vfo_push_x_band(uint16_t x_lo, uint16_t x_hi,
                             uint16_t row0, uint16_t row1);

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawVFO  (VFO_W=320 × VFO_H=64)
 *
 *  7-segment digits occupy rows 2..25 (BIG_H=24), single color UI_FREQ_FG.
 *  Sub-line (inactive VFO / RIT / RX-TX) at rows 28..43 (MED_H=16).
 *  VFO_SPLIT=28 separates upper (freq) from lower (sub/RIT/TX).
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawVFO(const SDR_UI_State_t *ui)
{
  uint32_t mhz  = ui->freq_hz / 1000000UL;
  uint32_t khz  = (ui->freq_hz % 1000000UL) / 1000UL;
  uint32_t hz_r = ui->freq_hz % 1000UL;
  char mhz_s[6], khz_s[4], hz_s[4];
  snprintf(mhz_s, sizeof(mhz_s), "%lu",   (unsigned long)mhz);
  snprintf(khz_s, sizeof(khz_s), "%03lu", (unsigned long)khz);
  snprintf(hz_s,  sizeof(hz_s),  "%03lu", (unsigned long)hz_r);

  char full_freq[20];
  snprintf(full_freq, sizeof(full_freq), "%s.%s.%s", mhz_s, khz_s, hz_s);

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

  /* RX = green (subtle), TX = red — per UI spec */
  static const char *const vfo_mode_s[] = {"AM","FM","USB","LSB","CW"};
  const char *vfo_mode_str = (ui->mode < 5U) ? vfo_mode_s[ui->mode] : "---";
  const char *rt_str       = ui->tx_mode ? "TX" : "RX";
  uint16_t    rt_color     = ui->tx_mode ? UI_TX_BG : UI_RX_BG;

  buf_fill(s_vfo_buf, (uint32_t)VFO_H * VFO_W, UI_VFO_BG);

  const uint16_t freq_top = 2U;
  const uint16_t vfoi_y   = 1U;
  const uint16_t sub_y    = (uint16_t)(freq_top + BIG_H + 2U);  /* row 28 */

  /* RX/TX badge: 20px tall (MED_H + 2px pad × 2), 30px wide (2 chars + 3px pad × 2), right-aligned */
  const uint16_t rt_bad_w = (uint16_t)(2U * MED_W + 6U);
  const uint16_t rt_bad_h = (uint16_t)(MED_H + 4U);
  const uint16_t rt_bx    = (uint16_t)(VFO_W - 2U - rt_bad_w);
  const uint16_t rt_by    = (uint16_t)(sub_y + ((uint16_t)(VFO_H - sub_y) - rt_bad_h) / 2U);
  const uint16_t rt_fg    = ui->tx_mode ? UI_TX_FG : UI_RX_FG;

  /* Frequency centering: each digit = BIG_W px, each '.' = 6 px */
  uint16_t total_w = 0U;
  for (const char *p = full_freq; *p; p++)
    total_w += (*p == '.') ? 6U : BIG_W;
  uint16_t fx_base = (VFO_W > total_w) ? (uint16_t)((VFO_W - total_w) / 2U) : 2U;

  for (uint16_t row = 0; row < VFO_H; row++) {
    uint16_t *ln = s_vfo_buf + (uint32_t)row * VFO_W;

    /* 7-segment frequency — single color */
    if (row >= freq_top && (row - freq_top) < BIG_H) {
      ln_segstr(ln, fx_base, row - freq_top, full_freq, UI_FREQ_FG, UI_VFO_BG);
    }

    /* Active VFO indicator (2× medium, top-left) */
    if (row >= vfoi_y && (row - vfoi_y) < MED_H) {
      const char *vl = (ui->active_vfo == 0U) ? "A" : "B";
      uint16_t    vc = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_ON;
      ln_medchar(ln, 2U, row - vfoi_y, *vl, vc, UI_VFO_BG);
    }

    /* Sub-line: secondary VFO freq | RIT offset | mode label */
    if (ui->freq_b_hz > 0U) {
      if (row >= sub_y && (row - sub_y) < MED_H)
        ln_medstr(ln, 4U, row - sub_y, sub_str, UI_FREQ_SUB, UI_VFO_BG);
    } else if (ui->rit_hz != 0) {
      if (row >= sub_y && (row - sub_y) < Font6x8.height)
        LCD_LineStr(ln, 4U, row - sub_y, sub_str, &Font6x8, UI_FREQ_SUB, UI_VFO_BG);
    } else {
      /* Mode label vertically aligned with badge */
      if (row >= rt_by && (row - rt_by) < rt_bad_h) {
        uint16_t br = row - rt_by;
        if (br >= 2U && br < 2U + MED_H)
          ln_medstr(ln, 4U, br - 2U, vfo_mode_str, UI_STATUS_LBL, UI_VFO_BG);
      }
    }

    /* RX/TX badge: solid rt_color fill, white medium text, right-aligned in sub-line zone */
    if (row >= rt_by && row < rt_by + rt_bad_h) {
      uint16_t br = row - rt_by;
      uint16_t px = SWAP16(rt_color);
      for (uint16_t i = 0U; i < rt_bad_w; i++) ln[rt_bx + i] = px;
      if (br >= 2U && br < 2U + MED_H)
        ln_medstr(ln, (uint16_t)(rt_bx + 3U), br - 2U, rt_str, rt_fg, rt_color);
    }
  }

  /* Section-split push: upper = rows 0..27, lower = rows 28..63. */
  const uint16_t VFO_SPLIT = 28U;

  bool upper_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_hz    != ui->freq_hz
      || s_vfo_cache.active_vfo != ui->active_vfo;

  bool lower_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_b_hz  != ui->freq_b_hz
      || s_vfo_cache.rit_hz     != ui->rit_hz
      || s_vfo_cache.tx_mode    != ui->tx_mode
      || s_vfo_cache.active_vfo != ui->active_vfo;

  /* Glyph-level dirty flags — compare against OLD cache values */
  bool mhz_g  = !s_vfo_cache.valid || strcmp(mhz_s, s_vfo_cache.mhz_s) != 0
                                    || fx_base != s_vfo_cache.fx_base;
  bool khz_g  = !s_vfo_cache.valid || strcmp(khz_s, s_vfo_cache.khz_s) != 0
                                    || fx_base != s_vfo_cache.fx_base;
  bool hz_g   = !s_vfo_cache.valid || strcmp(hz_s,  s_vfo_cache.hz_s)  != 0
                                    || fx_base != s_vfo_cache.fx_base;
  bool vfoi_g = !s_vfo_cache.valid || ui->active_vfo != s_vfo_cache.active_vfo;

  /* Update cache */
  s_vfo_cache.freq_hz    = ui->freq_hz;
  s_vfo_cache.freq_b_hz  = ui->freq_b_hz;
  s_vfo_cache.rit_hz     = ui->rit_hz;
  s_vfo_cache.tx_mode    = ui->tx_mode;
  s_vfo_cache.active_vfo = ui->active_vfo;
  strncpy(s_vfo_cache.mhz_s, mhz_s, sizeof(s_vfo_cache.mhz_s) - 1U);
  s_vfo_cache.mhz_s[sizeof(s_vfo_cache.mhz_s) - 1U] = '\0';
  strncpy(s_vfo_cache.khz_s, khz_s, sizeof(s_vfo_cache.khz_s) - 1U);
  s_vfo_cache.khz_s[sizeof(s_vfo_cache.khz_s) - 1U] = '\0';
  strncpy(s_vfo_cache.hz_s,  hz_s,  sizeof(s_vfo_cache.hz_s)  - 1U);
  s_vfo_cache.hz_s[sizeof(s_vfo_cache.hz_s) - 1U] = '\0';
  s_vfo_cache.fx_base = fx_base;
  s_vfo_cache.valid   = true;

  /* Push upper section with glyph-level granularity */
  if (upper_chg) {
    if (mhz_g) {
      /* MHz changed or first draw: centering may have shifted, push full upper */
      LCD_PushWindow(VFO_X, VFO_Y,
                     (uint16_t)(VFO_X + VFO_W - 1U), (uint16_t)(VFO_Y + VFO_SPLIT - 1U),
                     s_vfo_buf, (uint32_t)VFO_W * VFO_SPLIT);
      s_vfo_glyph_count++;
    } else {
      /* MHz unchanged: push only the sub-bands that changed */
      uint32_t cyc0 = DWT->CYCCNT;

      /* kHz/Hz digit band: dot1 through end of Hz digits */
      if (khz_g || hz_g) {
        uint16_t mhz_len = (uint16_t)strlen(mhz_s);
        uint16_t band_x0 = (uint16_t)(fx_base + (uint32_t)mhz_len * BIG_W);
        uint16_t band_x1 = (uint16_t)(band_x0 + 6U + 3U * BIG_W + 6U + 3U * BIG_W - 1U);
        if (band_x1 >= VFO_W) band_x1 = VFO_W - 1U;
        vfo_push_x_band(band_x0, band_x1, freq_top, (uint16_t)(freq_top + BIG_H - 1U));
        s_vfo_glyph_count++;
      }

      /* VFO A/B indicator (2× medium glyph, top-left) */
      if (vfoi_g) {
        uint16_t ind_x1 = (uint16_t)(2U + MED_W + 1U);
        uint16_t ind_y1 = (uint16_t)(vfoi_y + MED_H - 1U);
        if (ind_y1 >= VFO_SPLIT) ind_y1 = VFO_SPLIT - 1U;
        vfo_push_x_band(0U, ind_x1, vfoi_y, ind_y1);
        s_vfo_glyph_count++;
      }

      uint32_t us = ui_cyc_to_us(DWT->CYCCNT - cyc0);
      if (us > s_max_vfo_us) s_max_vfo_us = us;
    }
  } else {
    s_vfo_skip_count++;
  }

  /* Push lower section (sub-line / RIT / TX indicator) */
  if (lower_chg) {
    LCD_PushWindow(VFO_X, (uint16_t)(VFO_Y + VFO_SPLIT),
                   (uint16_t)(VFO_X + VFO_W - 1U), VFO_Y2 - 1U,
                   s_vfo_buf + (uint32_t)VFO_SPLIT * VFO_W,
                   (uint32_t)VFO_W * (VFO_H - VFO_SPLIT));
  }

  RuntimeDiag_VfoReport(s_vfo_glyph_count, s_vfo_skip_count, s_max_vfo_us);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  draw_smeter_rows  – full redraw of the 32-row MTR zone, segment bargraph
 *
 *  Layout (MTR_H=32):
 *    rows  2– 9  scale labels  (SM_LBL_ROW)
 *    row  10     tick marks    (SM_TICK_ROW)
 *    rows 11–23  segment bars  (SM_SEG_ROW, SM_SEG_H=13)
 *    rows 24–31  S-value text  (SM_VAL_ROW)
 *
 *  Each of SM_BARS=12 segments is SM_SEG_W=10 px wide with a 4-px gap.
 *  Green=S1–S6, yellow=S7–S9, red=S9+.
 * ════════════════════════════════════════════════════════════════════════════ */
static void draw_smeter_rows(int32_t bars)
{
  char s_str[8];
  if (bars <= 9) snprintf(s_str, sizeof(s_str), "S%ld",   (long)bars);
  else           snprintf(s_str, sizeof(s_str), "S9+%ld", (long)((bars - 9) * 3));

  static const char *const slbls[] = {"S","1","3","5","7","9","+20","+40"};

  for (uint16_t row = 0; row < MTR_H; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0, MTR_W, UI_MTR_BG);

    /* Scale labels */
    if (row >= SM_LBL_ROW && row < SM_LBL_ROW + Font6x8.height) {
      uint16_t fr = row - SM_LBL_ROW;
      for (uint8_t t = 0; t < 8U; t++) {
        uint16_t lx = (uint16_t)(SM_START_X + (uint16_t)s_sbar[t] * SM_UNIT_W);
        if (t == 6U) lx -= 9U;  /* centre "+20" over its tick */
        uint16_t col = (t < 6U) ? UI_SMETER_TICK : UI_S1_6;
        LCD_LineStr(ln, lx, fr, slbls[t], &Font6x8, col, UI_MTR_BG);
      }
    }

    /* Tick marks: single-pixel dot at each scale position */
    if (row == SM_TICK_ROW) {
      for (uint8_t t = 0; t < 8U; t++) {
        uint16_t tx = (uint16_t)(SM_START_X + (uint16_t)s_sbar[t] * SM_UNIT_W);
        if (tx < MTR_W) ln[tx] = SWAP16(UI_SMETER_TICK);
      }
    }

    /* Segment bars */
    if (row >= SM_SEG_ROW && row < SM_SEG_ROW + SM_SEG_H) {
      for (uint8_t seg = 0U; seg < SM_BARS; seg++) {
        if ((int32_t)seg < bars) {
          uint16_t col = (seg < 6U) ? UI_S1_6 : (seg < 9U) ? UI_S7_9 : UI_S9P;
          uint16_t x0  = (uint16_t)(SM_START_X + (uint16_t)seg * SM_UNIT_W);
          for (uint16_t px = x0; px < x0 + SM_SEG_W && px < MTR_W; px++)
            ln[px] = SWAP16(col);
        }
      }
    }

    /* S-value text */
    if (row >= SM_VAL_ROW && (row - SM_VAL_ROW) < Font6x8.height) {
      uint16_t fr   = row - SM_VAL_ROW;
      uint16_t scol = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
      LCD_LineStr(ln, SM_START_X, fr, s_str, &Font6x8, scol, UI_MTR_BG);
    }
  }
}

void SDR_UI_DrawMeter(const SDR_UI_State_t *ui)
{
  int32_t bars = (int32_t)((ui->signal_db + 73.0f) / 3.0f);
  if (bars < 0) bars = 0;
  if (bars > (int32_t)SM_BARS) bars = (int32_t)SM_BARS;
  s_rx_meter_bars = bars;
  draw_smeter_rows(bars);
  s_mtr_static_valid = true;
  LCD_PushWindow(MTR_X, MTR_Y,
                 (uint16_t)(MTR_X + MTR_W - 1U), MTR_Y2 - 1U,
                 s_mtr_buf, (uint32_t)MTR_W * MTR_H);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_UpdateSMeter  – fast RX meter refresh (10 Hz)
 *
 *  Fast path: pack and push only the dirty segment column range (the set of
 *  segments that changed state between old_bars and new bars) using s_mtr_strip.
 *  Static rows (labels, ticks) are never re-rendered after initial draw.
 *
 *  Segment push:  dirty_w × SM_SEG_H × 2 B  (min 364 B for 1 seg, max 4,368 B)
 *  Text push:     Font6x8.height × MTR_W × 2 B  = 5,120 B
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdateSMeter_SetTX(bool tx)
{
  (void)tx;
  s_tx_meter_active = false;
  s_tx_alc_bars = -1;
  s_tx_alc_pct = -1;
  s_tx_swr_x10 = -1;
  s_rx_meter_bars = -1;
  s_mtr_static_valid = false;
}
void SDR_UI_UpdateSMeter_SetVoltage(float v) { s_smeter_voltage = v; }

void SDR_UI_UpdateSMeter(float signal_db)
{
  int32_t bars = (int32_t)((signal_db + 73.0f) / 3.0f);
  if (bars < 0) bars = 0;
  if (bars > (int32_t)SM_BARS) bars = (int32_t)SM_BARS;
  if (bars == s_rx_meter_bars) return;

  int32_t old_bars = s_rx_meter_bars;
  s_rx_meter_bars  = bars;

  if (!s_mtr_static_valid) {
    draw_smeter_rows(bars);
    s_mtr_static_valid = true;
    LCD_PushWindow(MTR_X, MTR_Y,
                   (uint16_t)(MTR_X + MTR_W - 1U), MTR_Y2 - 1U,
                   s_mtr_buf, (uint32_t)MTR_W * MTR_H);
    return;
  }

  /* Fast path: dirty segment column range.
   * Only the segments that changed state need updating.
   * seg_lo = first changed segment, seg_hi = last changed segment. */
  {
    int32_t seg_lo   = (bars < old_bars) ? bars     : old_bars;
    int32_t seg_hi   = (bars > old_bars) ? bars - 1 : old_bars - 1;
    uint16_t dirty_w = (uint16_t)((seg_hi - seg_lo + 1) * (int32_t)SM_UNIT_W);
    uint16_t x_lo    = (uint16_t)(SM_START_X + (uint16_t)seg_lo * SM_UNIT_W);

    /* Build strip: dirty_w cols × SM_SEG_H rows */
    for (uint16_t r = 0U; r < SM_SEG_H; r++) {
      uint16_t *srow = s_mtr_strip + (uint32_t)r * dirty_w;
      uint16_t  c    = 0U;
      for (int32_t seg = seg_lo; seg <= seg_hi; seg++) {
        uint16_t col_lit = ((uint8_t)seg < 6U) ? UI_S1_6
                         : ((uint8_t)seg < 9U) ? UI_S7_9 : UI_S9P;
        uint16_t col_on  = ((int32_t)seg < bars) ? col_lit : UI_MTR_BG;
        for (uint16_t px = 0U; px < SM_UNIT_W; px++, c++) {
          srow[c] = SWAP16((px < SM_SEG_W) ? col_on : UI_MTR_BG);
        }
      }
    }
    LCD_PushWindow(
      (uint16_t)(MTR_X + x_lo),
      (uint16_t)(MTR_Y + SM_SEG_ROW),
      (uint16_t)(MTR_X + x_lo + dirty_w - 1U),
      (uint16_t)(MTR_Y + SM_SEG_ROW + SM_SEG_H - 1U),
      s_mtr_strip,
      (uint32_t)dirty_w * SM_SEG_H * 2U);
  }

  /* Text rows: re-render and push */
  char     s_str[8];
  if (bars <= 9) snprintf(s_str, sizeof(s_str), "S%ld",   (long)bars);
  else           snprintf(s_str, sizeof(s_str), "S9+%ld", (long)((bars - 9) * 3));
  uint16_t scol = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
  for (uint16_t row = SM_VAL_ROW; row < SM_VAL_ROW + Font6x8.height; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0, MTR_W, UI_MTR_BG);
    LCD_LineStr(ln, SM_START_X, row - SM_VAL_ROW, s_str, &Font6x8, scol, UI_MTR_BG);
  }
  LCD_PushWindow(MTR_X, (uint16_t)(MTR_Y + SM_VAL_ROW),
                 (uint16_t)(MTR_X + MTR_W - 1U),
                 (uint16_t)(MTR_Y + SM_VAL_ROW + Font6x8.height - 1U),
                 s_mtr_buf + (uint32_t)SM_VAL_ROW * MTR_W,
                 (uint32_t)MTR_W * Font6x8.height);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  TX meter helpers  (MTR_H=32)
 *
 *  Row layout:
 *    rows  1– 8: ALC scale labels
 *    row   9:    tick marks
 *    rows 10–12: ALC bar
 *    rows 13–20: ALC value text
 *    rows 21–28: SWR value text
 * ════════════════════════════════════════════════════════════════════════════ */
#define TX_LBL_ROW   1U
#define TX_TICK_ROW  9U
#define TX_BAR_ROW  10U
#define TX_ALC_ROW  13U
#define TX_SWR_ROW  21U

static void tx_meter_render_rows(uint16_t row0, uint16_t row1,
                                 int32_t alc_b, int32_t alc_pct,
                                 int32_t swr_x10)
{
  uint16_t alc_fill = (uint16_t)(SM_START_X + (uint16_t)alc_b * SM_UNIT_W);
  uint16_t x_end    = (uint16_t)(SM_START_X + SM_RULER_W);
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

    if (row >= TX_LBL_ROW && row < TX_LBL_ROW + Font6x8.height) {
      uint16_t fr = row - TX_LBL_ROW;
      for (uint8_t t = 0; t < 5U; t++) {
        uint16_t lx = (uint16_t)(SM_START_X + (uint16_t)spos[t] * SM_UNIT_W);
        LCD_LineStr(ln, lx, fr, slbls[t], &Font6x8, UI_SMETER_TICK, UI_MTR_BG);
      }
    }

    if (row == TX_TICK_ROW) {
      for (uint8_t t = 0; t < 5U; t++) {
        uint16_t tx = (uint16_t)(SM_START_X + (uint16_t)spos[t] * SM_UNIT_W);
        if (tx < MTR_W) ln[tx] = SWAP16(UI_SMETER_TICK);
      }
    }

    if (row >= TX_BAR_ROW && row < TX_BAR_ROW + 3U) {
      for (uint16_t px = SM_START_X; px < x_end && px < MTR_W; px++) {
        if (px < alc_fill) {
          uint16_t off = px - SM_START_X;
          uint16_t seg = off / SM_UNIT_W;
          uint16_t col = (seg < 6U) ? UI_S1_6 : (seg < 9U) ? UI_S7_9 : UI_S9P;
          ln[px] = SWAP16(col);
        } else {
          ln[px] = SWAP16(UI_SMETER_BG);
        }
      }
    }

    if (row >= TX_ALC_ROW && (row - TX_ALC_ROW) < Font6x8.height) {
      uint16_t fr = row - TX_ALC_ROW;
      LCD_LineStr(ln, SM_START_X, fr, "ALC", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, SM_START_X + 4U * Font6x8.width, fr, alc_val,
                  &Font6x8, UI_STATUS_VAL, UI_MTR_BG);
    }

    if (row >= TX_SWR_ROW && (row - TX_SWR_ROW) < Font6x8.height) {
      uint16_t fr = row - TX_SWR_ROW;
      LCD_LineStr(ln, SM_START_X, fr, "SWR", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, SM_START_X + 4U * Font6x8.width, fr, swr_val,
                  &Font6x8, swr_col, UI_MTR_BG);
    }
  }
}

static void tx_meter_push_rows(uint16_t row0, uint16_t row1)
{
  LCD_PushWindow(MTR_X, (uint16_t)(MTR_Y + row0),
                 (uint16_t)(MTR_X + MTR_W - 1U), (uint16_t)(MTR_Y + row1),
                 s_mtr_buf + (uint32_t)row0 * MTR_W,
                 (uint32_t)MTR_W * (row1 - row0 + 1U));
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_UpdateTXMeters
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdateTXMeters(float alc_norm, float swr)
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

  bool first = !s_tx_meter_active;
  if (first) s_tx_meter_active = true;

  if (first) {
    /* First TX render: push all static rows */
    tx_meter_render_rows(0U, TX_LBL_ROW - 1U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(0U, TX_LBL_ROW - 1U);
    tx_meter_render_rows(TX_LBL_ROW, TX_TICK_ROW, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(TX_LBL_ROW, TX_TICK_ROW);
  }

  if (first || alc_b != s_tx_alc_bars) {
    tx_meter_render_rows(TX_BAR_ROW, TX_BAR_ROW + 2U, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(TX_BAR_ROW, TX_BAR_ROW + 2U);
    s_tx_alc_bars = alc_b;
  }

  if (first || alc_pct != s_tx_alc_pct) {
    tx_meter_render_rows(TX_ALC_ROW, TX_ALC_ROW + (uint16_t)Font6x8.height - 1U,
                         alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(TX_ALC_ROW, TX_ALC_ROW + (uint16_t)Font6x8.height - 1U);
    s_tx_alc_pct = alc_pct;
  }

  if (first || swr_x10 != s_tx_swr_x10) {
    tx_meter_render_rows(TX_SWR_ROW, TX_SWR_ROW + (uint16_t)Font6x8.height - 1U,
                         alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(TX_SWR_ROW, TX_SWR_ROW + (uint16_t)Font6x8.height - 1U);
    s_tx_swr_x10 = swr_x10;
  }
}

/* ── Compat wrappers ─────────────────────────────────────────────────────── */
void SDR_UI_DrawTopBar(const SDR_UI_State_t *ui)
{
  SDR_UI_DrawHeader(ui);
  SDR_UI_DrawVFO(ui);
  SDR_UI_DrawMeter(ui);
}

void SDR_UI_DrawStatusPanel(const SDR_UI_State_t *ui)
{
  SDR_UI_DrawSidebarLeft(ui);
  SDR_UI_DrawSidebarRight(ui);
}

/* ── spec_push_partial ───────────────────────────────────────────────────────
 * Push only columns [x_lo..x_hi] of s_spec_buf to the LCD, chunked by
 * SPEC_CHUNK_ROWS rows.  For each strip, columns are packed into s_spec_strip
 * (a contiguous tile) and sent as a single LCD_PushWindow call, keeping the
 * window-command count the same as a full push while reducing data transfer
 * proportionally to the dirty column fraction.
 * Returns the elapsed µs for the full partial push. */
static uint32_t spec_push_partial(uint16_t x_lo, uint16_t x_hi)
{
  uint16_t w    = (uint16_t)(x_hi - x_lo + 1U);
  uint32_t cyc0 = DWT->CYCCNT;
  for (uint16_t strip = 0U; strip < SPEC_H; strip += SPEC_CHUNK_ROWS) {
    uint16_t rows = (uint16_t)(SPEC_H - strip);
    if (rows > SPEC_CHUNK_ROWS) rows = SPEC_CHUNK_ROWS;
    /* Wait for previous DMA to finish reading s_spec_strip before overwriting. */
    LCD_Wait();
    for (uint16_t r = 0U; r < rows; r++) {
      memcpy(&s_spec_strip[(uint32_t)r * w],
             &s_spec_buf[strip + r][x_lo],
             (uint32_t)w * 2U);
    }
    LCD_PushWindowAsync((uint16_t)(SPEC_X + x_lo), (uint16_t)(SPEC_Y + strip),
                        (uint16_t)(SPEC_X + x_hi),
                        (uint16_t)(SPEC_Y + strip + rows - 1U),
                        s_spec_strip, (uint32_t)w * rows * 2U);
    RuntimeDiag_MainLoopBeat();
  }
  /* Wait for the final strip DMA before returning so timing is accurate. */
  LCD_Wait();
  return ui_cyc_to_us(DWT->CYCCNT - cyc0);
}

/* ── vfo_push_x_band ─────────────────────────────────────────────────────────
 * Push a column sub-band of s_vfo_buf to the LCD.  Coordinates are
 * buffer-local (0-based within the VFO zone).  One LCD_PushWindow per row
 * avoids the need for a separate packing buffer while keeping data volume
 * proportional to the number of changed columns. */
static void vfo_push_x_band(uint16_t x_lo, uint16_t x_hi,
                             uint16_t row0, uint16_t row1)
{
  uint16_t w = (uint16_t)(x_hi - x_lo + 1U);
  for (uint16_t row = row0; row <= row1; row++) {
    LCD_PushWindow((uint16_t)(VFO_X + x_lo), (uint16_t)(VFO_Y + row),
                   (uint16_t)(VFO_X + x_hi), (uint16_t)(VFO_Y + row),
                   s_vfo_buf + (uint32_t)row * VFO_W + x_lo, w);
  }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSpectrum  – single FMC burst (480×72 = 69,120 px ≈ 8.1 ms)
 *
 *  Grid lines at 75%, 50%, 25% height.  Vertical dots every 40 pixels.
 *  BW markers and center-frequency line overlaid.
 *  Delta-skip: suppresses redraw when spectrum is visually unchanged (< 2px).
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSpectrum(const float *fft_db, uint16_t bins,
                          float bw_lo_ratio, float bw_hi_ratio,
                          SDR_UI_State_t *ui)
{
  if (!bins) return;
  uint16_t b0, n_vis;
  spec_window(bins, &b0, &n_vis);
  const float bpp    = (float)n_vis / (float)SPEC_W;
  const float cscale = (float)bins  / (float)n_vis;

  for (uint16_t x = 0; x < SPEC_W; x++) {
    float    fbin = (float)b0 + (float)x * bpp;
    uint16_t bi0  = (uint16_t)fbin;
    if (bi0 >= bins) bi0 = (uint16_t)(bins - 1U);
    float    t    = fbin - (float)bi0;
    uint16_t bi1  = (bi0 + 1U < bins) ? (uint16_t)(bi0 + 1U) : bi0;
    /* Bias toward the dominant bin (t < 0.5 → bi0 wins, t ≥ 0.5 → bi1 wins). */
    float tw = (t < 0.5f) ? (t * 0.4f) : (1.0f - (1.0f - t) * 0.4f);
    s_spec_yf[x] = pwr_compress(fft_db[bi0]) * (1.0f - tw)
                 + pwr_compress(fft_db[bi1]) * tw;
  }

  const uint16_t NO_SIG = (uint16_t)(SPEC_H - 1U);
  for (uint16_t x = 0; x < SPEC_W; x++) {
    uint16_t h = (uint16_t)(s_spec_yf[x] * (float)(SPEC_H - 2U) + 0.5f);
    if (h > (uint16_t)(SPEC_H - 2U)) h = (uint16_t)(SPEC_H - 2U);
    s_spec_py[x] = (h > 0U) ? (uint16_t)(SPEC_H - 1U - h) : NO_SIG;
  }

  /* One-pass delta scan: find max delta and dirty column bounding box.
   * Full scan (no early-break) lets us track x_lo/x_hi for partial push. */
  uint16_t dirty_x0 = SPEC_W, dirty_x1 = 0U;
  if (s_spec_py_valid) {
    uint16_t max_d = 0U;
    for (uint16_t x = 0U; x < SPEC_W; x++) {
      uint16_t d = (s_spec_py[x] > s_spec_py_prev[x])
                 ? (uint16_t)(s_spec_py[x] - s_spec_py_prev[x])
                 : (uint16_t)(s_spec_py_prev[x] - s_spec_py[x]);
      if (d > max_d) max_d = d;
      if (d >= 2U) {
        if (x < dirty_x0) dirty_x0 = x;
        dirty_x1 = x;
      }
    }
    if (max_d < 2U) {
      s_spec_skip_hits++;
      RuntimeDiag_SpecReport(s_spec_partial_count, s_spec_skip_hits,
                              s_max_spec_partial_us);
      return;
    }
  }
  s_spec_draw_hits++;
  memcpy(s_spec_py_prev, s_spec_py, sizeof(s_spec_py));
  s_spec_py_valid = true;

  /* Grid lines at 75%, 50%, 25% of height */
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

  uint16_t spec_sw      = SWAP16(0x041FU);   /* bright: peak edge       */
  uint16_t spec_fill_sw = SWAP16(0x0107U);   /* darker: fill body       */
  uint16_t bw_sw        = SWAP16(UI_SPEC_BW);
  /* Center marker colours: bright centre, black shadow for dark|white|dark */
  uint16_t cx_sw        = SWAP16(0xFFFFU);   /* bright white centre pixel     */
  uint16_t cx_shadow    = SWAP16(0x0000U);   /* black shadow ± 1 px           */
  uint16_t dot_sw       = SWAP16(UI_SPEC_GRID);

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
    /* Center marker: dark|white|dark — signal trace overlays it when present. */
    if (cx > 0U && cx < SPEC_W - 1U) {
      row[cx - 1U] = cx_shadow;
      row[cx]      = cx_sw;
      row[cx + 1U] = cx_shadow;
    } else if (cx < SPEC_W) {
      row[cx] = cx_sw;
    }
  }

  /* Draw filled area spectrum: bright peak edge, dim fill body to baseline. */
  const uint16_t fill_bot = (uint16_t)(SPEC_H - 2U);
  for (uint16_t x = 0U; x < SPEC_W; x++) {
    uint16_t peak = s_spec_py[x];
    if (peak >= NO_SIG) continue;
    s_spec_buf[peak][x] = spec_sw;
    for (uint16_t yr = (uint16_t)(peak + 1U); yr <= fill_bot; yr++) {
      s_spec_buf[yr][x] = spec_fill_sw;
    }
  }

  /* Bottom divider row */
  for (uint16_t x = 0; x < SPEC_W; x++)
    s_spec_buf[SPEC_H - 1U][x] = SWAP16(UI_DIVIDER);

  /* Choose partial or full push.
   * Partial: dirty column band < 75% of display width and cache was valid.
   * Full:    first draw, zoom change, or wide dirty range. */
  bool do_partial = (dirty_x0 <= dirty_x1)
                  && ((uint32_t)(dirty_x1 - dirty_x0 + 1U) < (SPEC_W * 3U / 4U));

  if (do_partial) {
    uint32_t us = spec_push_partial(dirty_x0, dirty_x1);
    s_spec_partial_count++;
    if (us > s_max_spec_partial_us) s_max_spec_partial_us = us;
  } else {
    /* Full chunked async push — 8-row strips, no abort path for spectrum.
     * LCD_Wait() before each strip ensures the previous DMA has finished
     * (source buffer safe to reuse) and gives audio ISRs a preemption window. */
    for (uint16_t strip = 0U; strip < SPEC_H; strip += SPEC_CHUNK_ROWS) {
      uint16_t rows   = (uint16_t)(SPEC_H - strip);
      if (rows > SPEC_CHUNK_ROWS) rows = SPEC_CHUNK_ROWS;
      uint16_t lcd_y0 = (uint16_t)(SPEC_Y + strip);
      uint16_t lcd_y1 = (uint16_t)(lcd_y0 + rows - 1U);
      LCD_Wait();
      LCD_PushWindowAsync(SPEC_X, lcd_y0,
                          (uint16_t)(SPEC_X + SPEC_W - 1U), lcd_y1,
                          &s_spec_buf[strip][0], (uint32_t)SPEC_W * rows * 2U);
      RuntimeDiag_MainLoopBeat();
    }
    /* Ensure the final strip DMA completes before leaving this function. */
    LCD_Wait();
  }

  RuntimeDiag_SpecReport(s_spec_partial_count, s_spec_skip_hits, s_max_spec_partial_us);
  (void)ui;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_WaterfallPrecompute  (call from DSP task)
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
    /* Fast log2-based compression (identical to pwr_compress used for spectrum).
     * Maps power [~1e-8 .. 1.0] → index [0..255] on a log scale.
     * Eliminates per-pixel log10f (~80 cycles on Cortex-M7) with no perceptible
     * change in dynamic range or color palette appearance. */
    int idx = (int)(pwr_compress(pwr) * 255.0f + 0.5f);
    if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
    dst[x] = (uint8_t)idx;
  }

  s_wf_fill = fill;
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_WF_PRECOMPUTE);
  return fill;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_WaterfallPush  (call from UI task)
 *
 *  Single-row ring push — Option B from the embedded scroll spec.
 *
 *  The ring head advances forward (0 → WF_H-1 → 0).  Each tick, exactly
 *  one new row is written into the ring slot and pushed to its fixed LCD Y
 *  coordinate.  All other rows are already resident in LCD RAM from previous
 *  pushes; the controller retains them without any CPU action.
 *
 *  LCD traffic per tick: 480 px × 1 row × 2 B = 960 B ≈ 112 µs.
 *  Previous full-frame 2-split push: 480 × 72 × 2 = 69,120 B ≈ 8.06 ms.
 *
 *  Visual result: rolling sweep waterfall — the write cursor descends one
 *  row per tick.  Rows above the cursor are the most recent history; rows
 *  below are from the previous sweep cycle.  No memmove, no full-frame copy,
 *  no multi-chunk loops, no overload-abort needed for a 112 µs push.
 *
 *  Skipped when s_wf_suppressed is set by adaptive load control.
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_WaterfallPush(uint8_t buf_idx)
{
  if (s_wf_suppressed) return;

  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_WF_SCROLL);

  /* Advance ring head to next slot (wraps 71 → 0) */
  s_wf_head = (s_wf_head >= (uint8_t)(WF_H - 1U)) ? 0U : (uint8_t)(s_wf_head + 1U);

  /* Apply colour LUT and write new row into ring slot */
  const uint8_t *src = s_wf_idx[buf_idx];
  uint16_t      *row = s_wf_buf[s_wf_head];
  for (uint16_t x = 0U; x < WF_W; x++) row[x] = s_wf_lut[src[x]];

  /* Async DMA push: wait for any previous waterfall row DMA, then launch new.
   * row_us measures only the wait + DMA-start latency, not transfer time.
   * Actual pixel transfer time is tracked in LCD_DMA_GetMaxLatencyUs(). */
  uint16_t lcd_y = (uint16_t)(WF_Y + s_wf_head);
  uint32_t cyc0  = DWT->CYCCNT;
  LCD_Wait();
  LCD_PushWindowAsync(WF_X, lcd_y,
                      (uint16_t)(WF_X + WF_W - 1U), lcd_y,
                      row, (uint32_t)WF_W * 2U);
  uint32_t row_us = ui_cyc_to_us(DWT->CYCCNT - cyc0);

  s_lcd_chunk_count++;
  if (row_us > s_max_chunk_render_us) s_max_chunk_render_us = row_us;

  RuntimeDiag_LcdChunkReport(s_lcd_chunk_count, s_lcd_chunk_abort_count,
                               s_wf_partial_count, s_max_chunk_render_us);
  RuntimeDiag_LcdDmaReport(LCD_DMA_GetMaxLatencyUs(), LCD_DMA_GetQueuedCount(),
                            LCD_IsBusy());

  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_WF_SCROLL);
}

/* ── Compat: Precompute + Push in one call ───────────────────────────────── */
void SDR_UI_DrawWaterfall(const float *fft_db, uint16_t bins)
{
  if (!bins) return;
  uint8_t idx = SDR_UI_WaterfallPrecompute(fft_db, bins);
  SDR_UI_WaterfallPush(idx);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Spectrum zoom control
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_SetSpecZoom(uint8_t zoom)
{
  if (zoom >= SPEC_ZOOM_COUNT) zoom = SPEC_ZOOM_COUNT - 1U;
  s_spec_zoom = zoom;
  draw_footer_rows(spec_half_span_hz());
}

uint8_t SDR_UI_GetSpecZoom(void) { return s_spec_zoom; }

void SDR_UI_RedrawFooter(void)
{
  draw_footer_rows(spec_half_span_hz());
}

/* ── Spectrum skip statistics ─────────────────────────────────────────────── */
void SDR_UI_GetSpecSkipStats(uint32_t *skip_hits, uint32_t *draw_hits)
{
  if (skip_hits) *skip_hits = s_spec_skip_hits;
  if (draw_hits) *draw_hits = s_spec_draw_hits;
}

/* ── Stub ────────────────────────────────────────────────────────────────── */
void SDR_UI_DrawFuncBar(const SDR_UI_State_t *ui)
{ (void)ui; }
