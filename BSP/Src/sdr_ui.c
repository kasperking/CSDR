/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_ui.c
  * @brief   CSDR SDR UI – zone layout, FMC burst pushes (ST7796 480×320 / ST7789 240×320)
  *
  *  Panel is selected at compile time via LCD_PANEL in lcd_panel_config.h.
  *
  *  ST7796 480×320: 9-zone layout with sidebars.  All zone buffers live in
  *  DMA_SRAM (RAM_D1).  Spectrum/waterfall use async DMA2 Stream0 M2M.
  *  Total RAM_D1 footprint: ~253 KB.
  *
  *  ST7789 240×320: compact layout — no sidebars, status bar replaces them.
  *  Smaller buffers (SPEC 240×76, WF 240×96 vs. 480×72 each).  Waterfall row
  *  push is 480 B (~56 µs) vs. 960 B (~112 µs) at the same FMC clock.
  *  Total RAM_D1 footprint: ~148 KB.
  *
  *  DrawSidebarLeft renders the compact STATUS zone on ST7789.
  *  DrawSidebarRight is a no-op on ST7789.
  *
  *  Single-zone transfer times (8-bit FMC, 116.7 ns/byte):
  *    ST7796  Spectrum  (480×72):   ~8.06 ms   (9 × 8-row strips)
  *    ST7796  WF row   (480×1):     ~112 µs
  *    ST7789  Spectrum  (240×76):   ~4.26 ms   (10 × 8-row strips)
  *    ST7789  WF row   (240×1):      ~56 µs
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sdr_ui.h"
#include "sdr_dsp.h"    /* DSP_FFT_SIZE */
#include "runtime_diag.h"
#include "lcd_dma.h"    /* LCD_Wait / LCD_PushWindowAsync / diagnostics */
#include "selftest.h"   /* g_selftest, SelfTest_AnyFail — top-bar HW warnings */
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
 * SPEC_CHUNK_ROWS: spectrum pushed in strips with abort capability.
 * Value comes from HW_DMA_CHUNK_ROWS in the active hardware profile.
 * At 8-bit FMC (116.7 ns/byte): 8 rows × 480 px × 2 B = 7,680 B → ~896 µs.
 *
 * WF uses the same SPEC_CHUNK_ROWS value for its full-frame memmove push. */
#define SPEC_CHUNK_ROWS  HW_DMA_CHUNK_ROWS

/* TX compact panel geometry — centred inside the SPEC and WF zones.
 * TX_PANEL_W: bounded width so bins map at ≤4 px/bin (near-native density).
 * TX_PANEL_X: left edge, derived at compile time from SPEC_W. */
#define TX_PANEL_W   128U
#define TX_PANEL_X   ((uint16_t)((SPEC_W - TX_PANEL_W) / 2U))

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

/* ── DMA-accessible zone buffers (RAM_D1, 512 KB) ───────────────────────── *
 *
 *  ST7796 480×320:                      ST7789 240×320:
 *   s_hdr_buf  : 24×480×2 =  23,040 B   16×240×2 =   7,680 B
 *   s_sbl_buf  : 96×80 ×2 =  15,360 B   — (not on ST7789)
 *   s_sbr_buf  : 96×80 ×2 =  15,360 B   — (not on ST7789)
 *   s_sts_buf  :     — (not on ST7796)   28×240×2 =  13,440 B
 *   s_vfo_buf  : 64×320×2 =  40,960 B   48×240×2 =  23,040 B
 *   s_mtr_buf  : 32×320×2 =  20,480 B   24×240×2 =  11,520 B
 *   s_spec_buf : 72×480×2 =  69,120 B   76×240×2 =  36,480 B
 *   s_wf_buf   : 72×480×2 =  69,120 B   96×240×2 =  46,080 B
 *   Total UI             : ~253 KB                 ~148 KB
 */
static uint16_t s_hdr_buf[HDR_H  * LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
#if LCD_PANEL == LCD_PANEL_ST7796
static uint16_t s_sbl_buf[SBL_H  * SBL_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_sbr_buf[SBR_H  * SBR_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
#else
/* On ST7789 the sidebar functions return before touching these buffers.
 * Declare 1-element dummies so the dead-code function bodies compile. */
static uint16_t s_sbl_buf[1U] __attribute__((section(".DMA_SRAM")));
static uint16_t s_sbr_buf[1U] __attribute__((section(".DMA_SRAM")));
/* Compact status bar — replaces sidebars on ST7789 */
static uint16_t s_sts_buf[STS_H  * LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
#endif
static uint16_t s_vfo_buf[VFO_H  * VFO_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_mtr_buf[MTR_H  * MTR_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_spec_buf[SPEC_H][SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_wf_buf[WF_H][WF_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* Spectrum compute arrays — static (avoid large stack frame per call) */
static float    s_spec_yf     [SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static uint16_t s_spec_py     [SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
/* Strip buffer: one SPEC_CHUNK_ROWS-tall slice for partial-column FMC pushes.
 * Worst case: SPEC_CHUNK_ROWS × SPEC_W = 8 × SPEC_W pixels. */
static uint16_t s_spec_strip[SPEC_CHUNK_ROWS * SPEC_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* WF pre-compute: two uint8_t line buffers (double-buffer for DSP/UI split) */
static uint8_t  s_wf_idx[2][WF_W];   /* 2 × WF_W bytes */
static volatile uint8_t s_wf_fill = 0;

/* Adaptive waterfall suppression (set by csdr_app under high audio load) */
static volatile bool s_wf_suppressed = false;

/* TX zone blank flag — cleared on TX→RX so next TX session re-blanks */
static bool s_tx_zone_blanked = false;

/* CPU-only: IIR smoother + colour LUT + ring head */
static float    s_wf_smooth[DSP_FFT_SIZE];
static uint16_t s_wf_lut[256];
static uint8_t  s_wf_head = 0;
static int16_t  s_smeter_voltage_x10 = 0;

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
  uint8_t  tx_power;
  uint8_t  pa_watts;
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
  int16_t  mic_gain;   /* voice gain OR digi_gain, depending on mode */
  uint8_t  att_db;
  int16_t  rit_hz;     /* passband graphic shifts with IF/RIT offset */
  uint8_t  mode;       /* needed to re-render when DG↔voice label flips */
  uint8_t  tx_power;
  uint8_t  pa_watts;
  bool     tx_mode;
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

/* ── fmt_1dp ─────────────────────────────────────────────────────────────── *
 * Format a scaled integer (value × 10) as "W.F[unit]" with no floats.
 * e.g. fmt_1dp(buf, 8, 132, 'V') → "13.2V"
 *      fmt_1dp(buf, 8,  15,  0 ) → "1.5"
 * buf must be at least 7 bytes.  Returns pointer to buf for convenience. */
static char *fmt_1dp(char *buf, uint8_t bufsz, int32_t val_x10, char unit)
{
  uint8_t n = 0;
  if (val_x10 < 0) {
    if (n < bufsz - 1U) buf[n++] = '-';
    val_x10 = -val_x10;
  }
  int32_t whole = val_x10 / 10;
  int32_t frac  = val_x10 % 10;
  if (whole >= 100) { if (n < bufsz-1U) buf[n++] = (char)('0' + whole / 100); }
  if (whole >=  10) { if (n < bufsz-1U) buf[n++] = (char)('0' + (whole / 10) % 10); }
  if              (n < bufsz-1U) buf[n++] = (char)('0' + whole % 10);
  if              (n < bufsz-1U) buf[n++] = '.';
  if              (n < bufsz-1U) buf[n++] = (char)('0' + frac);
  if (unit && n < bufsz-1U)      buf[n++] = unit;
  buf[n < bufsz ? n : bufsz - 1U] = '\0';
  return buf;
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

/* Forward declaration — defined later in this file */
static uint32_t spec_push_partial(uint16_t x_lo, uint16_t x_hi);

/* ── TX mode UI policy ───────────────────────────────────────────────────────
 * SetTXMode: called from csdr_apply_tx on every TX/RX transition.
 *   TX→RX: reset zone-blank flag so the next TX session re-blanks; also
 *           invalidate the RX spectrum prev-row cache so the first post-TX
 *           spectrum draw repaints from scratch rather than delta-skipping. */
void SDR_UI_SetTXMode(bool tx_active)
{
  if (!tx_active) {
    s_tx_zone_blanked = false;
    s_spec_py_valid   = false;
  }
}

/* SDR_UI_DrawTXSpectrum – compact 128 px audio-band spectrum for TX monitor.
 *
 * First call per TX session: lazily blanks the full SPEC and WF zones once,
 * then on every subsequent call redraws only the TX_PANEL_W (128 px) centred
 * strip — FMC traffic proportional to panel width, not full display width.
 *
 * Render path:
 *   – bpp = n_vis / TX_PANEL_W → near-native bin density, no stretching.
 *   – Audio-band window: ±3.5 kHz (USB/LSB/CW), ±6 kHz (AM), ±8 kHz (FM).
 *   – Renders into s_spec_buf columns [TX_PANEL_X .. TX_PANEL_X+TX_PANEL_W-1].
 *   – Pushed via spec_push_partial: ≈ 2.1 ms for 128 × SPEC_H px strip.
 *   – Appends one WF ring row at panel columns only: ≈ 30 µs.
 *   – Reuses s_spec_py, s_spec_buf, s_wf_smooth, s_wf_head (mutually exclusive
 *     with RX DrawSpectrum / WaterfallPush). */
void SDR_UI_DrawTXSpectrum(const float *fft_db, uint16_t bins,
                            uint8_t mode, uint32_t sr)
{
  if (!bins || !sr) return;

  /* ── Lazy zone blanking on first TX call ────────────────────────────────── */
  if (!s_tx_zone_blanked) {
    /* Blank SPEC zone — fill with UI_SPEC_BG and push all strips. */
    buf_fill(&s_spec_buf[0][0], (uint32_t)SPEC_H * SPEC_W, UI_SPEC_BG);
    for (uint16_t strip = 0U; strip < SPEC_H; strip += SPEC_CHUNK_ROWS) {
      uint16_t rows = (uint16_t)(SPEC_H - strip);
      if (rows > SPEC_CHUNK_ROWS) rows = SPEC_CHUNK_ROWS;
      LCD_Wait();
      LCD_PushWindowAsync(SPEC_X, (uint16_t)(SPEC_Y + strip),
                          (uint16_t)(SPEC_X + SPEC_W - 1U),
                          (uint16_t)(SPEC_Y + strip + rows - 1U),
                          &s_spec_buf[strip][0], (uint32_t)SPEC_W * rows * 2U);
    }
    /* Blank WF zone — zero the ring buffer and push all strips. */
    memset(s_wf_buf, 0, sizeof(s_wf_buf));
    for (uint16_t strip = 0U; strip < WF_H; strip += SPEC_CHUNK_ROWS) {
      uint16_t rows = (uint16_t)(WF_H - strip);
      if (rows > SPEC_CHUNK_ROWS) rows = SPEC_CHUNK_ROWS;
      LCD_Wait();
      LCD_PushWindowAsync(WF_X, (uint16_t)(WF_Y + strip),
                          (uint16_t)(WF_X + WF_W - 1U),
                          (uint16_t)(WF_Y + strip + rows - 1U),
                          &s_wf_buf[strip][0], (uint32_t)WF_W * rows * 2U);
    }
    LCD_Wait();
    s_tx_zone_blanked = true;
  }

  /* ── Audio band window ───────────────────────────────────────────────────── *
   * fft_db is linear power after fftshift: [0]=−Fs/2, [bins/2]=DC,
   * [bins−1]≈+Fs/2.  We show a ±half_hz window centred on DC.               */
  float    hz_per_bin = (float)sr / (float)bins;
  float    half_hz    = (mode == 0U) ? 6000.0f          /* AM   ±6 kHz  */
                      : (mode == 1U) ? 8000.0f           /* FM   ±8 kHz  */
                                     : 3500.0f;          /* SSB/CW ±3.5 kHz */
  uint16_t half_span  = (uint16_t)(half_hz / hz_per_bin + 0.5f);
  if (half_span < 4U) half_span = 4U;

  uint16_t b_center = (uint16_t)(bins >> 1U);
  uint16_t b_lo     = (b_center > half_span) ? (uint16_t)(b_center - half_span) : 0U;
  uint16_t b_hi     = (uint16_t)(b_center + half_span);
  if (b_hi >= bins) b_hi = (uint16_t)(bins - 1U);
  uint16_t n_vis    = (uint16_t)(b_hi - b_lo + 1U);
  const float bpp = (float)n_vis / (float)TX_PANEL_W;

  /* ── Column peak rows — reuse s_spec_py (mutually exclusive with RX path) ─ */
  const uint16_t NO_SIG   = (uint16_t)(SPEC_H - 1U);
  const uint16_t fill_bot = (uint16_t)(SPEC_H - 2U);
  for (uint16_t x = 0U; x < TX_PANEL_W; x++) {
    float    fbin = (float)b_lo + (float)x * bpp;
    uint16_t bi   = (uint16_t)(fbin + 0.5f);
    if (bi >= bins) bi = (uint16_t)(bins - 1U);
    float    p    = pwr_compress(fft_db[bi]);
    uint16_t h    = (uint16_t)(p * (float)(SPEC_H - 2U) + 0.5f);
    if (h > fill_bot) h = fill_bot;
    s_spec_py[x]  = (h > 0U) ? (uint16_t)(SPEC_H - 1U - h) : NO_SIG;
  }

  /* ── Pixel colors ───────────────────────────────────────────────────────── *
   * Amber-orange peak + dark rust body distinguishes TX spectrum from the
   * RX icy-blue palette at a glance.                                         */
  const uint16_t tx_peak_sw = SWAP16(0xFCA0U); /* amber-orange: R=31,G=37,B=0 */
  const uint16_t tx_body_sw = SWAP16(0x7800U); /* dark rust:    R=15,G=0, B=0 */
  const uint16_t bg_sw      = SWAP16(UI_SPEC_BG);
  const uint16_t grid_sw    = SWAP16(UI_SPEC_GRID);
  const uint16_t div_sw     = SWAP16(UI_DIVIDER);
  /* DC (0 Hz) reference column — panel-local */
  const uint16_t cx_panel   = (uint16_t)(TX_PANEL_W / 2U);

  /* ── Clear panel columns and render into s_spec_buf ────────────────────── */
  for (uint16_t y = 0U; y < (uint16_t)(SPEC_H - 1U); y++) {
    uint16_t *rp = &s_spec_buf[y][TX_PANEL_X];
    for (uint16_t x = 0U; x < TX_PANEL_W; x++) rp[x] = bg_sw;
    if ((y % 6U == 0U)) rp[cx_panel] = grid_sw;
  }

  /* Filled amber bars */
  for (uint16_t x = 0U; x < TX_PANEL_W; x++) {
    uint16_t peak = s_spec_py[x];
    if (peak >= NO_SIG) continue;
    s_spec_buf[peak][TX_PANEL_X + x] = tx_peak_sw;
    for (uint16_t yr = (uint16_t)(peak + 1U); yr <= fill_bot; yr++)
      s_spec_buf[yr][TX_PANEL_X + x] = tx_body_sw;
  }

  /* "TX" label — anchored to panel left edge */
  for (uint16_t fr = 0U; fr < (uint16_t)Font6x8.height; fr++) {
    uint16_t r = (uint16_t)(2U + fr);
    if (r < (uint16_t)(SPEC_H - 1U))
      LCD_LineStr(s_spec_buf[r], (uint16_t)(TX_PANEL_X + 4U), fr,
                  "TX", &Font6x8, UI_TX_BG, UI_SPEC_BG);
  }

  /* Bottom divider — panel columns only */
  uint16_t *div_row = &s_spec_buf[SPEC_H - 1U][TX_PANEL_X];
  for (uint16_t x = 0U; x < TX_PANEL_W; x++) div_row[x] = div_sw;

  /* ── Compact async SPEC push: TX_PANEL_W × SPEC_H px ≈ 2.1 ms ──────────── */
  (void)spec_push_partial(TX_PANEL_X, (uint16_t)(TX_PANEL_X + TX_PANEL_W - 1U));

  /* ── TX waterfall ring-row append ────────────────────────────────────────── *
   * IIR-smooth audio band bins, advance ring head, map TX_PANEL_W px via LUT,
   * then push a single partial WF row ≈ 30 µs of FMC traffic.               */
  for (uint16_t b = b_lo; b <= b_hi; b++)
    s_wf_smooth[b] = WF_SMOOTH_ALPHA * s_wf_smooth[b]
                   + (1.0f - WF_SMOOTH_ALPHA) * fft_db[b];

  s_wf_head = (s_wf_head >= (uint8_t)(WF_H - 1U)) ? 0U : (uint8_t)(s_wf_head + 1U);

  /* s_wf_buf must not be written while any previous WF DMA is still reading it. */
  LCD_Wait();
  uint16_t *wf_row = &s_wf_buf[s_wf_head][TX_PANEL_X];
  for (uint16_t x = 0U; x < TX_PANEL_W; x++) {
    float    fbin = (float)b_lo + (float)x * bpp;
    uint16_t bi   = (uint16_t)(fbin + 0.5f);
    if (bi >= bins) bi = (uint16_t)(bins - 1U);
    int idx = (int)(pwr_compress(s_wf_smooth[bi]) * 255.0f + 0.5f);
    if (idx < 0) idx = 0; else if (idx > 255) idx = 255;
    wf_row[x] = s_wf_lut[(uint8_t)idx];
  }

  uint16_t wf_lcd_y = (uint16_t)(WF_Y + s_wf_head);
  LCD_PushWindowAsync((uint16_t)(WF_X + TX_PANEL_X), wf_lcd_y,
                      (uint16_t)(WF_X + TX_PANEL_X + TX_PANEL_W - 1U), wf_lcd_y,
                      wf_row, (uint32_t)TX_PANEL_W * 2U);
  /* Barrier: do not exit with DMA in flight.  csdr_refresh_display fires every
   * 1000 ms in TX mode — same loop iteration as t_tx_spec at LCM(200,1000) —
   * and calls synchronous LCD_PushWindow.  Without this wait the two paths
   * write to LCD_FMC_DATA_ADDR concurrently, corrupting the command stream. */
  LCD_Wait();
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
  char lm_buf[6] = "";
  char rm_buf[6] = "";
  uint16_t rx_x  = 0U;
  uint16_t lm_lx = 0U;   /* -12K label left edge */
  uint16_t rm_lx = 0U;   /* +12K label left edge */
  uint16_t lm_px = 0U;   /* -12K marker pixel (for tick) */
  uint16_t rm_px = 0U;   /* +12K marker pixel (for tick) */
  bool show_mid = false;

  if (half_hz > 0U) {
    uint32_t bk = half_hz / 1000U;
    snprintf(lbuf, sizeof(lbuf), "-%luK", (unsigned long)bk);
    snprintf(rbuf, sizeof(rbuf), "+%luK", (unsigned long)bk);
    rx_x = (uint16_t)(LCD_W - (uint16_t)(strlen(rbuf) * Font6x8.width) - 4U);
  }

  /* ±12k intermediate markers — only rendered when span > ±12kHz */
  if (half_hz > 12000U) {
    uint16_t cx  = (uint16_t)(LCD_W / 2U);
    uint16_t off = (uint16_t)((uint32_t)12000U * (uint32_t)(LCD_W / 2U) / half_hz);
    lm_px = (uint16_t)(cx - off);
    rm_px = (uint16_t)(cx + off);
    snprintf(lm_buf, sizeof(lm_buf), "-12K");
    snprintf(rm_buf, sizeof(rm_buf), "+12K");
    uint16_t lw = (uint16_t)(4U * (uint16_t)Font6x8.width);  /* 4 chars wide */
    lm_lx = (lm_px >= lw / 2U) ? (uint16_t)(lm_px - lw / 2U) : 0U;
    rm_lx = (rm_px >= lw / 2U) ? (uint16_t)(rm_px - lw / 2U) : 0U;
    if (rm_lx + lw > LCD_W) rm_lx = (uint16_t)(LCD_W - lw);
    show_mid = true;
  }

  uint16_t fh    = Font6x8.height;
  uint16_t pad   = (FTR_H - fh) / 2U;
  uint16_t cx_px = (uint16_t)(LCD_W / 2U);

  /* Tick marks sit immediately above labels.
   * Major (0 kHz): 4-row tall, bright.  Medium (±12k): 2-row tall, dimmer. */
  uint16_t tmaj0    = (pad >= 4U) ? (uint16_t)(pad - 4U) : 0U;
  uint16_t tmed0    = (pad >= 2U) ? (uint16_t)(pad - 2U) : 0U;
  uint16_t tick_maj = SWAP16(UI_STATUS_VAL);   /* bright white */
  uint16_t tick_med = SWAP16(UI_SMETER_TICK);  /* medium gray  */

  for (uint16_t row = 0U; row < FTR_H; row++) {
    uint16_t *ln = LCD_GetLineBuf();
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);

    /* 4-row center tick */
    if (row >= tmaj0 && row < pad && cx_px < LCD_W)
      ln[cx_px] = tick_maj;

    /* 2-row ±12k ticks */
    if (show_mid && row >= tmed0 && row < pad) {
      if (lm_px < LCD_W) ln[lm_px] = tick_med;
      if (rm_px < LCD_W) ln[rm_px] = tick_med;
    }

    if (half_hz > 0U && row >= pad && row < pad + fh) {
      uint16_t frow = row - pad;
      LCD_LineStr(ln, 4U,   frow, lbuf,  &Font6x8, UI_SMETER_TICK, UI_BG);
      LCD_LineStr(ln, rx_x, frow, rbuf,  &Font6x8, UI_SMETER_TICK, UI_BG);
      LCD_LineStr(ln, (uint16_t)(cx_px - Font6x8.width / 2U), frow,
                  "0", &Font6x8, UI_STATUS_VAL, UI_BG);
      if (show_mid) {
        LCD_LineStr(ln, lm_lx, frow, lm_buf, &Font6x8, UI_SMETER_TICK, UI_BG);
        LCD_LineStr(ln, rm_lx, frow, rm_buf, &Font6x8, UI_SMETER_TICK, UI_BG);
      }
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
  fmt_1dp(vstr, sizeof(vstr), ui->voltage_x10, 'V');
  uint16_t vcol    = (ui->voltage_x10 < 115 && ui->voltage_x10 > 5)
                     ? UI_STATUS_OFF : UI_STATUS_VAL;
  uint16_t volt_x  = (uint16_t)(LCD_W - (uint16_t)(strlen(vstr) * Font6x8.width) - 4U);
  uint16_t txt_y   = (uint16_t)((HDR_H - Font6x8.height) / 2U);
  const char *agc_str = ui->agc_fast ? "AGC-F" : "AGC-S";

  /* ── Hardware warning: "! CODEC PLL" etc. centred between AGC and voltage ─ */
  char     warn_str[32] = {0};
  uint16_t warn_x       = 0U;
  if (SelfTest_AnyFail()) {
    uint8_t pos = 0U;
    warn_str[pos++] = '!';
    for (uint8_t i = 0U; i < SELFTEST_COUNT; i++) {
      if (!g_selftest.items[i].ok) {
        warn_str[pos++] = ' ';
        for (const char *c = g_selftest.items[i].id;
             *c && pos < (uint8_t)(sizeof(warn_str) - 1U); c++) {
          warn_str[pos++] = *c;
        }
      }
    }
    warn_str[pos] = '\0';
    /* Centre between right edge of AGC label and left edge of voltage */
    uint16_t agc_end = (uint16_t)(4U + (uint16_t)(strlen(agc_str) * Font6x8.width) + 6U);
    uint16_t warn_w  = (uint16_t)((uint16_t)strlen(warn_str) * Font5x8.width);
    uint16_t avail   = (volt_x > agc_end) ? (uint16_t)(volt_x - agc_end) : 0U;
    warn_x = (avail > warn_w)
             ? (uint16_t)(agc_end + (avail - warn_w) / 2U)
             : agc_end;
  }

  for (uint16_t row = 0; row < HDR_H; row++) {
    uint16_t *ln = s_hdr_buf + (uint32_t)row * LCD_W;
    if (row == HDR_H - 1U) {
      LCD_LineFill(ln, 0, LCD_W, UI_DIVIDER);
      continue;
    }
    LCD_LineFill(ln, 0, LCD_W, UI_HDR_BG);
    if (row >= txt_y && row < txt_y + Font6x8.height) {
      uint16_t fr = row - txt_y;
      LCD_LineStr(ln, 4U,     fr, agc_str,  &Font6x8, UI_STATUS_LBL, UI_HDR_BG);
      LCD_LineStr(ln, volt_x, fr, vstr,     &Font6x8, vcol,          UI_HDR_BG);
      if (warn_str[0]) {
        LCD_LineStr(ln, warn_x, fr, warn_str, &Font5x8, UI_STATUS_WARN, UI_HDR_BG);
      }
    }
  }

  LCD_PushWindow(0U, HDR_Y, (uint16_t)(LCD_W - 1U), HDR_Y2 - 1U,
                 s_hdr_buf, (uint32_t)LCD_W * HDR_H);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  draw_compact_status  (ST7789 only)  STS_H=28 × LCD_W=240
 *
 *  Two-row status bar that replaces the sidebars on the 240×320 compact layout.
 *
 *  Row 0 (y≈5): [Mode]  VOL:xx  SQL:xx
 *  Row 1 (y≈16): BW:xxx  ST:xxx  [NR] [NB]
 *
 *  NR/NB drawn as coloured badges (green=ON, red=OFF) matching sidebar style.
 * ════════════════════════════════════════════════════════════════════════════ */
#if LCD_PANEL == LCD_PANEL_ST7789
static void draw_compact_status(const SDR_UI_State_t *ui)
{
  /* Cache guard */
  if (s_sbl_cache.valid
      && s_sbl_cache.mode       == ui->mode
      && s_sbl_cache.volume     == ui->volume
      && s_sbl_cache.squelch    == ui->squelch
      && s_sbl_cache.nr_on      == ui->nr_on
      && s_sbl_cache.nb_on      == ui->nb_on
      && s_sbr_cache.bw_hz      == ui->bw_hz
      && s_sbr_cache.step       == ui->step) return;

  s_sbl_cache.mode       = ui->mode;
  s_sbl_cache.volume     = ui->volume;
  s_sbl_cache.squelch    = ui->squelch;
  s_sbl_cache.nr_on      = ui->nr_on;
  s_sbl_cache.nb_on      = ui->nb_on;
  s_sbl_cache.active_vfo = ui->active_vfo;
  s_sbl_cache.valid      = true;
  s_sbr_cache.bw_hz      = ui->bw_hz;
  s_sbr_cache.step       = ui->step;
  s_sbr_cache.valid      = true;

  static const char *const mode_s[] = {"AM","FM","USB","LSB","CW","DIGU","DIGL"};
  const char *mode_str = (ui->mode < 7U) ? mode_s[ui->mode] : "---";

  char vol_str[8]; snprintf(vol_str, sizeof(vol_str), "VOL:%u",  ui->volume);
  char sql_str[8]; snprintf(sql_str, sizeof(sql_str), "SQL:%u",  ui->squelch);

  char bw_str[10];
  if (ui->bw_hz >= 10000U)
    snprintf(bw_str, sizeof(bw_str), "BW:%luk", (unsigned long)(ui->bw_hz / 1000U));
  else if (ui->bw_hz >= 1000U)
    snprintf(bw_str, sizeof(bw_str), "BW:%lu.%luk",
             (unsigned long)(ui->bw_hz / 1000U),
             (unsigned long)((ui->bw_hz % 1000U) / 100U));
  else
    snprintf(bw_str, sizeof(bw_str), "BW:%luHz", (unsigned long)ui->bw_hz);

  char step_str[12];
  uint32_t st = ui->step;
  if      (st >= 100000U) snprintf(step_str, sizeof(step_str), "ST:100k");
  else if (st >=  10000U) snprintf(step_str, sizeof(step_str), "ST:10k");
  else if (st >=   1000U) snprintf(step_str, sizeof(step_str), "ST:1k");
  else if (st >=    100U) snprintf(step_str, sizeof(step_str), "ST:100");
  else if (st >=     10U) snprintf(step_str, sizeof(step_str), "ST:10");
  else                    snprintf(step_str, sizeof(step_str), "ST:1");

  uint16_t nr_bg = ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF;
  uint16_t nb_bg = ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF;

  buf_fill(s_sts_buf, (uint32_t)STS_H * LCD_W, UI_BG);

  /* Vertical placement: two Font8x10 rows in STS_H=28 px.
   * Content height: 10 + 4 + 10 = 24 px.  Top margin: 4, gap: 4. */
  const uint16_t row0_y = 4U;          /* row 0: mode / vol / sql */
  const uint16_t row1_y = (uint16_t)(row0_y + Font8x10.height + 4U);  /* = 18 */

  /* Thin top border */
  for (uint16_t x = 0U; x < LCD_W; x++)
    s_sts_buf[x] = SWAP16(UI_DIVIDER);

  uint16_t fh = Font8x10.height;
  for (uint16_t fr = 0U; fr < fh; fr++) {
    /* Row 0: mode  vol_str  sql_str */
    {
      uint16_t r = (uint16_t)(row0_y + fr);
      if (r < STS_H) {
        uint16_t *ln = s_sts_buf + (uint32_t)r * LCD_W;
        uint16_t col_mode = (ui->mode < 7U) ? UI_STATUS_VAL : UI_STATUS_LBL;
        LCD_LineStrW(ln, 4U,  fr, mode_str, &Font8x10, col_mode, UI_BG);
        /* vol and sql left-placed with gap after mode field */
        uint16_t vx = (uint16_t)(4U + 6U * Font8x10.width);  /* after "USB " gap */
        LCD_LineStrW(ln, vx,  fr, vol_str, &Font8x10, UI_STATUS_VAL, UI_BG);
        uint16_t sx = (uint16_t)(vx + (uint16_t)(strlen(vol_str) + 2U) * Font8x10.width);
        LCD_LineStrW(ln, sx,  fr, sql_str, &Font8x10, UI_STATUS_VAL, UI_BG);
      }
    }
    /* Row 1: bw_str  step_str  [NR] [NB] */
    {
      uint16_t r = (uint16_t)(row1_y + fr);
      if (r < STS_H) {
        uint16_t *ln = s_sts_buf + (uint32_t)r * LCD_W;
        LCD_LineStrW(ln, 4U, fr, bw_str,   &Font8x10, UI_FREQ_KHZ, UI_BG);
        uint16_t bx = (uint16_t)(4U + (uint16_t)(strlen(bw_str) + 2U) * Font8x10.width);
        LCD_LineStrW(ln, bx, fr, step_str, &Font8x10, UI_FREQ_KHZ, UI_BG);
        /* NR badge */
        uint16_t nr_x = (uint16_t)(LCD_W - 50U);
        uint16_t nb_x = (uint16_t)(LCD_W - 26U);
        LCD_LineFill(ln, nr_x, 22U, nr_bg);
        LCD_LineStrW(ln, (uint16_t)(nr_x + 4U), fr, "NR", &Font8x10, UI_BG, nr_bg);
        LCD_LineFill(ln, nb_x, 22U, nb_bg);
        LCD_LineStrW(ln, (uint16_t)(nb_x + 4U), fr, "NB", &Font8x10, UI_BG, nb_bg);
      }
    }
  }

  LCD_PushWindow(0U, STS_Y,
                 (uint16_t)(LCD_W - 1U), (uint16_t)(STS_Y2 - 1U),
                 s_sts_buf, (uint32_t)LCD_W * STS_H);
}
#endif /* LCD_PANEL_ST7789 */

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSidebarLeft  (SBL_W=80 × SBL_H=96)
 *
 *  5 items × ~19 rows each:
 *   0: Mode        1: VFO A/B   2: NR/NB
 *   3: VOL         4: SQL
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSidebarLeft(const SDR_UI_State_t *ui)
{
#if LCD_PANEL == LCD_PANEL_ST7789
  /* Compact layout: render the status bar instead of a sidebar */
  draw_compact_status(ui);
  return;
#endif /* LCD_PANEL_ST7789 */

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

  static const char *const mode_s[]  = {"AM","FM","USB","LSB","CW","DIGU","DIGL"};
  static const uint16_t    mode_bg[] = {UI_MODE_AM, UI_MODE_FM,  UI_MODE_USB,
                                        UI_MODE_LSB, UI_MODE_CW, UI_MODE_DIGU,
                                        UI_MODE_DIGL};
  const char *mode_str = (ui->mode < 7U) ? mode_s[ui->mode]  : "---";
  uint16_t    mbg      = (ui->mode < 7U) ? mode_bg[ui->mode] : UI_STATUS_LBL;

  char vol_str[6]; snprintf(vol_str, sizeof(vol_str), "%u", ui->volume);
  char sql_str[6]; snprintf(sql_str, sizeof(sql_str), "%u", ui->squelch);

  buf_fill(s_sbl_buf, (uint32_t)SBL_H * SBL_W, UI_SBL_BG);

  const uint16_t item_h = 19U;   /* 5 items × 19 = 95, +1 row blank at top */

  for (uint8_t i = 0; i < 5U; i++) {
    uint16_t y0 = (uint16_t)(1U + (uint32_t)i * item_h);

    if (i > 0U)
      for (uint16_t x = 0; x < SBL_W; x++)
        s_sbl_buf[(uint32_t)y0 * SBL_W + x] = SWAP16(UI_DIVIDER);

    uint16_t text_y = y0 + 6U;

    switch (i) {
      case 0:
        for (uint16_t fr = 0; fr < Font8x10.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          uint16_t cx = (uint16_t)(2U +
              (SBL_W - 2U - (uint16_t)(strlen(mode_str) * Font8x10.width)) / 2U);
          LCD_LineStrW(ln, cx, fr, mode_str, &Font8x10, mbg, UI_SBL_BG);
        }
        break;

      case 1: {
        uint16_t col_a = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_LBL;
        uint16_t col_b = (ui->active_vfo == 1U) ? UI_STATUS_ON  : UI_STATUS_LBL;
        for (uint16_t fr = 0; fr < Font8x10.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStrW(ln, 2U,                        fr, "VFO", &Font8x10, UI_STATUS_LBL, UI_SBL_BG);
          LCD_LineStrW(ln, 2U + 4U * Font8x10.width,  fr, "A",   &Font8x10, col_a, UI_SBL_BG);
          LCD_LineStrW(ln, 2U + 5U * Font8x10.width,  fr, "/",   &Font8x10, UI_STATUS_LBL, UI_SBL_BG);
          LCD_LineStrW(ln, 2U + 6U * Font8x10.width,  fr, "B",   &Font8x10, col_b, UI_SBL_BG);
        }
        break;
      }

      case 2: {
        uint16_t nr_bg = ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF;
        uint16_t nb_bg = ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF;
        for (uint16_t fr = 0; fr < Font8x10.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineFill(ln,  2U, 34U, nr_bg);
          LCD_LineStrW(ln,  13U, fr, "NR", &Font8x10, UI_BG, nr_bg);
          LCD_LineFill(ln, 40U, 34U, nb_bg);
          LCD_LineStrW(ln,  51U, fr, "NB", &Font8x10, UI_BG, nb_bg);
        }
        break;
      }

      case 3:
        for (uint16_t fr = 0; fr < Font8x10.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStrW(ln, 2U, fr, "VOL", &Font8x10, UI_STATUS_LBL, UI_SBL_BG);
          uint16_t vx = (uint16_t)(SBL_W - 1U - (uint16_t)(strlen(vol_str) * Font8x10.width) - 3U);
          LCD_LineStrW(ln, vx, fr, vol_str, &Font8x10, UI_STATUS_VAL, UI_SBL_BG);
        }
        break;

      case 4:
        for (uint16_t fr = 0; fr < Font8x10.height; fr++) {
          uint16_t r = text_y + fr;
          if (r >= SBL_H) break;
          uint16_t *ln = s_sbl_buf + (uint32_t)r * SBL_W;
          LCD_LineStrW(ln, 2U, fr, "SQL", &Font8x10, UI_STATUS_LBL, UI_SBL_BG);
          uint16_t vx = (uint16_t)(SBL_W - 1U - (uint16_t)(strlen(sql_str) * Font8x10.width) - 3U);
          LCD_LineStrW(ln, vx, fr, sql_str, &Font8x10, UI_STATUS_VAL, UI_SBL_BG);
        }
        break;

      default: break;
    }
  }

  LCD_PushWindow(SBL_X, SBL_Y,
                 (uint16_t)(SBL_X + SBL_W - 1U), SBL_Y2 - 1U,
                 s_sbl_buf, (uint32_t)SBL_W * SBL_H);
}

/* ── sbr_draw_passband ──────────────────────────────────────────────────────
 * Xiegu-style passband trapezoid in the lower portion of the right sidebar.
 * Draws BW label (Font5x8, centred) above a trapezoidal passband shape.
 * y0/h: zone origin and height in s_sbr_buf.
 * bw_hz: passband width; rit_hz: IF/RIT offset shifts the center left/right. */
#if LCD_PANEL == LCD_PANEL_ST7796
static void sbr_draw_passband(uint16_t y0, uint16_t h,
                              uint32_t bw_hz, int16_t rit_hz)
{
  char bw_str[10];
  if (bw_hz >= 10000U)
    snprintf(bw_str, sizeof(bw_str), "%luk", (unsigned long)(bw_hz / 1000U));
  else if (bw_hz >= 1000U)
    snprintf(bw_str, sizeof(bw_str), "%lu.%luk",
             (unsigned long)(bw_hz / 1000U),
             (unsigned long)((bw_hz % 1000U) / 100U));
  else
    snprintf(bw_str, sizeof(bw_str), "%luHz", (unsigned long)bw_hz);

  /* Layout: [2px pad][Font5x8 label][2px gap][shape fills rest] */
  const uint16_t lbl_off = 2U;
  const uint16_t shp_off = (uint16_t)(lbl_off + Font5x8.height + 2U);
  const uint16_t shp_h   = (h > shp_off + 4U) ? (uint16_t)(h - shp_off - 1U) : 4U;

  /* Passband half-width in pixels (BW 0–8 kHz maps to 0–36 px) */
  const uint32_t bw_max = 8000U;
  const uint16_t hw_max = 72U;
  uint32_t bw_c = (bw_hz > bw_max) ? bw_max : bw_hz;
  uint16_t hw   = (uint16_t)((uint32_t)bw_c * hw_max / bw_max);
  if (hw < 3U) hw = 3U;

  /* RIT/IF shift → center offset, saturated at ±12 px */
  int16_t shift = 0;
  if (bw_hz > 200U) {
    int32_t s = (int32_t)rit_hz * 12 / ((int32_t)(bw_hz / 2U) + 1);
    shift = (s > 12) ? 12 : (s < -12) ? -12 : (int16_t)s;
  }
  int16_t cx = (int16_t)(SBR_W / 2U) + shift;

  /* Trapezoid: top = passband width, bottom flares by slope px each side */
  const int16_t slope = (hw > 4U) ? 3 : 1;
  int16_t xl_t = cx - (int16_t)hw;
  int16_t xr_t = cx + (int16_t)hw;
  int16_t xl_b = xl_t - slope;
  int16_t xr_b = xr_t + slope;

  uint16_t pw = SWAP16(UI_SPEC_BW);      /* cyan outline        */
  uint16_t cm = SWAP16(UI_STATUS_VAL);   /* white center marker */
  uint16_t fi = SWAP16(UI_SMETER_BG);    /* dim interior fill   */

  for (uint16_t r = 0U; r < h; r++) {
    uint16_t y = (uint16_t)(y0 + r);
    if (y >= SBR_H) break;
    uint16_t *ln = s_sbr_buf + (uint32_t)y * SBR_W;

    if (r >= lbl_off && r < (uint16_t)(lbl_off + Font5x8.height)) {
      uint16_t fr = (uint16_t)(r - lbl_off);
      uint16_t lw = (uint16_t)(strlen(bw_str) * Font5x8.width);
      uint16_t lx = (lw < SBR_W) ? (uint16_t)((SBR_W - lw) / 2U) : 0U;
      LCD_LineStr(ln, lx, fr, bw_str, &Font5x8, UI_FREQ_KHZ, UI_SBR_BG);
    }

    if (r < shp_off) continue;
    uint16_t sr = (uint16_t)(r - shp_off);
    if (sr >= shp_h) continue;

    /* Interpolate left/right edges linearly from top to bottom */
    int16_t denom = (int16_t)(shp_h > 1U ? shp_h - 1U : 1U);
    int16_t xl_e  = xl_t + (int16_t)((int32_t)(xl_b - xl_t) * (int32_t)sr / denom);
    int16_t xr_e  = xr_t + (int16_t)((int32_t)(xr_b - xr_t) * (int32_t)sr / denom);
    if (xl_e < 0)                 xl_e = 0;
    if (xr_e < 0)                 xr_e = 0;
    if (xl_e >= (int16_t)SBR_W)  xl_e = (int16_t)(SBR_W - 1U);
    if (xr_e >= (int16_t)SBR_W)  xr_e = (int16_t)(SBR_W - 1U);

    if (sr == 0U || sr == (uint16_t)(shp_h - 1U)) {
      /* Top and bottom rails: full horizontal line */
      for (int16_t x = xl_e; x <= xr_e; x++)
        ln[(uint16_t)x] = (x == cx) ? cm : pw;
    } else {
      /* Interior: left/right outline pixels + dim fill + center marker */
      ln[(uint16_t)xl_e] = pw;
      ln[(uint16_t)xr_e] = pw;
      for (int16_t x = (int16_t)(xl_e + 1); x < xr_e; x++) {
        if (x < 0 || x >= (int16_t)SBR_W) continue;
        ln[(uint16_t)x] = (x == cx) ? cm : fi;
      }
    }
  }
}
#endif /* LCD_PANEL_ST7796 */

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawSidebarRight  (SBR_W=80 × SBR_H=96)
 *
 *  2 paired text rows + Xiegu-style passband graphic below:
 *    Row 0: BW  <val>   |  ST  <val>
 *    Row 1: MIC <val>   |  AT  <val>
 *    Passband zone (rows 61-95): BW label + trapezoid indicator
 *
 *  Column geometry (left 36 px, right 35 px, 6 px gutter):
 *    Left  col: label at x=2,  value right-aligned to x=38
 *    Right col: label at x=44, value right-aligned to x=79
 *
 *  row_h=26: 2×26=52 px + 9 px top pad = 61 px text area.
 *  Thin separator between rows (1 px at top of row 1).
 *  Cache guard: skip buffer rebuild + push when values unchanged.
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_DrawSidebarRight(const SDR_UI_State_t *ui)
{
#if LCD_PANEL == LCD_PANEL_ST7789
  (void)ui;  /* no sidebar on compact layout */
  return;
#endif /* LCD_PANEL_ST7789 */

  /* Cache guard — avoid rebuild when nothing changed */
  if (s_sbr_cache.valid
      && s_sbr_cache.bw_hz    == ui->bw_hz
      && s_sbr_cache.step     == ui->step
      && s_sbr_cache.mic_gain == ui->mic_gain
      && s_sbr_cache.att_db   == ui->att_db
      && s_sbr_cache.rit_hz   == ui->rit_hz
      && s_sbr_cache.mode     == ui->mode) return;

  s_sbr_cache.bw_hz    = ui->bw_hz;
  s_sbr_cache.step     = ui->step;
  s_sbr_cache.mic_gain = ui->mic_gain;
  s_sbr_cache.att_db   = ui->att_db;
  s_sbr_cache.rit_hz   = ui->rit_hz;
  s_sbr_cache.mode     = ui->mode;
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

  /* Row 1 left: MIC for voice modes, DG for digital modes.
   * csdr_app passes digi_gain (not mic_gain) in ui->mic_gain when in DIGU/DIGL. */
  bool digi_mode = (ui->mode == (uint8_t)5U || ui->mode == (uint8_t)6U); /* DIGU=5, DIGL=6 */
  char mic_str[8]; snprintf(mic_str, sizeof(mic_str), "%d", (int)ui->mic_gain);
  const char *mic_lbl = digi_mode ? "DG" : "MIC";
  uint16_t    mic_vc  = digi_mode ? UI_STATUS_ON : UI_STATUS_VAL;

  /* AT: show 0.5 dB precision from PE4302 raw register value.
   * When RF AGC is active the value colour changes to green (UI_STATUS_ON). */
  char att_str[8];
  if (ui->att_x2 & 1U)
    snprintf(att_str, sizeof(att_str), "%u.5", ui->att_x2 / 2U);
  else
    snprintf(att_str, sizeof(att_str), "%u dB", ui->att_x2 / 2U);
  uint16_t att_vc = ui->rf_agc_on ? UI_STATUS_ON : UI_STATUS_VAL;

  buf_fill(s_sbr_buf, (uint32_t)SBR_H * SBR_W, UI_SBR_BG);

  /* 2 paired text rows; passband graphic fills the space below */
  struct { const char *lbl; const char *val; uint16_t vc; } rows[2][2] = {
    { { "BW",  bw_str,  UI_FREQ_KHZ }, { "ST",  step_str, UI_FREQ_KHZ } },
    { { mic_lbl, mic_str, mic_vc    }, { "AT",  att_str,  att_vc      } },
  };

  const uint16_t row_h   = 26U;
  const uint16_t top_pad =  9U;
  const uint16_t txt_off =  9U;

  /* Column geometry — left col 36 px, right col 35 px, 6 px gutter */
  const uint16_t L_LBL_X = 2U;   /* left col label  */
  const uint16_t L_VAL_X = 38U;  /* left col val right-edge */
  const uint16_t R_LBL_X = 44U;  /* right col label (gutter: 44-38=6 px) */
  const uint16_t R_VAL_X = 79U;  /* right col val right-edge */

  for (uint8_t i = 0; i < 2U; i++) {
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
      uint16_t val_x  = (uint16_t)(L_VAL_X - (uint16_t)(strlen(val) * Font5x8.width));
      for (uint16_t fr = 0; fr < Font5x8.height; fr++) {
        uint16_t r = (uint16_t)(txt_y + fr);
        if (r >= SBR_H) break;
        uint16_t *ln = s_sbr_buf + (uint32_t)r * SBR_W;
        LCD_LineStr(ln, L_LBL_X, fr, lbl, &Font5x8, UI_STATUS_LBL, UI_SBR_BG);
        LCD_LineStr(ln, val_x,   fr, val, &Font5x8, vc,             UI_SBR_BG);
      }
    }

    /* Right item (optional) */
    if (rows[i][1].lbl != NULL) {
      const char *lbl = rows[i][1].lbl;
      const char *val = rows[i][1].val;
      uint16_t    vc  = rows[i][1].vc;
      uint16_t val_x  = (uint16_t)(R_VAL_X - (uint16_t)(strlen(val) * Font5x8.width));
      for (uint16_t fr = 0; fr < Font5x8.height; fr++) {
        uint16_t r = (uint16_t)(txt_y + fr);
        if (r >= SBR_H) break;
        uint16_t *ln = s_sbr_buf + (uint32_t)r * SBR_W;
        LCD_LineStr(ln, R_LBL_X, fr, lbl, &Font5x8, UI_STATUS_LBL, UI_SBR_BG);
        LCD_LineStr(ln, val_x,   fr, val, &Font5x8, vc,             UI_SBR_BG);
      }
    }
  }

  /* Passband graphic fills the zone below the 2 text rows */
  {
    uint16_t pb_y0 = (uint16_t)(top_pad + 2U * row_h);
    uint16_t pb_h  = (uint16_t)(SBR_H - pb_y0);
    sbr_draw_passband(pb_y0, pb_h, ui->bw_hz, ui->rit_hz);
  }

  LCD_PushWindow(SBR_X, SBR_Y,
                 (uint16_t)(SBR_X + SBR_W - 1U), SBR_Y2 - 1U,
                 s_sbr_buf, (uint32_t)SBR_W * SBR_H);
}

/* Forward declaration — defined later in this file */
static void vfo_push_x_band(uint16_t x_lo, uint16_t x_hi,
                             uint16_t row0, uint16_t row1);

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawVFO  (VFO_W × VFO_H)
 *
 *  ST7796 (64 px): 7-segment digits rows 2..25, 15-px gap, thin divider row 33,
 *  secondary VFO sub-line rows 41..50 (Font8x10).  VFO_SPLIT=28.
 *  ST7789 (48 px): 7-segment digits rows 2..25, 12-px gap, sub-line rows 38..47 (Font8x10).
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
  static const char *const vfo_mode_s[] = {"AM","FM","USB","LSB","CW","DIGU","DIGL"};
  const char *vfo_mode_str = (ui->mode < 7U) ? vfo_mode_s[ui->mode] : "---";
  const char *rt_str       = ui->tx_mode ? "TX" : "RX";
  uint16_t    rt_color     = ui->tx_mode ? UI_TX_BG : UI_RX_BG;

  /* PW badge (ST7796 only — no vertical room on ST7789) */
#if LCD_PANEL != LCD_PANEL_ST7789
  char pw_str[8] = "";
  if (ui->tx_mode) {
    uint16_t actual_w = (ui->pa_watts > 0U)
      ? (uint16_t)((uint32_t)ui->pa_watts * ui->tx_power / 100U)
      : (uint16_t)ui->tx_power;
    snprintf(pw_str, sizeof(pw_str), "%dW", (int)actual_w);
  }
#endif

  buf_fill(s_vfo_buf, (uint32_t)VFO_H * VFO_W, UI_VFO_BG);

  const uint16_t freq_top = 2U;
  const uint16_t vfoi_y   = 1U;

  /* Gap between primary and secondary VFO.  Secondary uses Font8x10 (8×10 px). */
#if LCD_PANEL == LCD_PANEL_ST7789
  const uint16_t sub_y  = (uint16_t)(freq_top + BIG_H + 12U);  /* row 38: 12-px gap, Font8x10 fills rows 38-47 */
  const uint16_t div_y  = 0xFFFFU;  /* no room for divider on compact panel */
#else
  const uint16_t sub_y  = (uint16_t)(freq_top + BIG_H + 15U);  /* row 41: 15-px gap */
  const uint16_t div_y  = (uint16_t)(freq_top + BIG_H + 7U);   /* row 33, centred in gap */
#endif
  const uint16_t badge_y = (uint16_t)(freq_top + BIG_H + 2U);  /* row 28, mode/RX/TX base */

  /* RX/TX text-only badge: no filled background; colour identifies mode.
   * In TX mode the badge is pinned to badge_y (no centering) to free the
   * lower rows for the watts readout. */
  const uint16_t rt_bad_w = (uint16_t)(2U * MED_W + 6U);
  const uint16_t rt_bad_h = (uint16_t)(MED_H + 4U);
  const uint16_t rt_bx    = (uint16_t)(VFO_W - 2U - rt_bad_w);
  const uint16_t rt_by    = ui->tx_mode
      ? badge_y
      : (uint16_t)(badge_y +
          ((VFO_H > badge_y + rt_bad_h) ? (VFO_H - badge_y - rt_bad_h) / 2U : 0U));

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
      if (row >= sub_y && (row - sub_y) < Font8x10.height)
        LCD_LineStrW(ln, 4U, row - sub_y, sub_str, &Font8x10, UI_FREQ_SUB, UI_VFO_BG);
    } else if (ui->rit_hz != 0) {
      if (row >= sub_y && (row - sub_y) < Font8x10.height)
        LCD_LineStrW(ln, 4U, row - sub_y, sub_str, &Font8x10, UI_FREQ_SUB, UI_VFO_BG);
    } else {
      /* Mode label vertically aligned with badge */
      if (row >= rt_by && (row - rt_by) < rt_bad_h) {
        uint16_t br = row - rt_by;
        if (br >= 2U && br < 2U + MED_H)
          ln_medstr(ln, 4U, br - 2U, vfo_mode_str, UI_STATUS_LBL, UI_VFO_BG);
      }
    }

    /* RX/TX: colored text only, transparent background */
    if (row >= rt_by && row < rt_by + rt_bad_h) {
      uint16_t br = row - rt_by;
      if (br >= 2U && br < 2U + MED_H)
        ln_medstr(ln, (uint16_t)(rt_bx + 3U), br - 2U, rt_str, rt_color, UI_VFO_BG);
    }

#if LCD_PANEL != LCD_PANEL_ST7789
    /* Watts readout — MED font, right-aligned under TX badge, TX mode only */
    if (ui->tx_mode && pw_str[0] != '\0') {
      uint16_t pw_by = (uint16_t)(badge_y + rt_bad_h);  /* row 48 */
      if (row >= pw_by && (row - pw_by) < MED_H) {
        uint16_t med_row = row - pw_by;
        uint16_t pw_len  = (uint16_t)(strlen(pw_str) * MED_W);
        uint16_t pw_x    = (uint16_t)(rt_bx + rt_bad_w - pw_len);
        ln_medstr(ln, pw_x, med_row, pw_str, UI_TX_BG, UI_VFO_BG);
      }
    }
#endif

    /* Thin centred divider between primary and secondary VFO (only when sub-freq shown) */
    if (row == div_y && ui->freq_b_hz > 0U) {
      uint16_t dx0 = (uint16_t)(VFO_W / 5U);
      uint16_t dx1 = (uint16_t)(4U * VFO_W / 5U);
      for (uint16_t x = dx0; x < dx1; x++)
        ln[x] = UI_BORDER;
    }
  }

  /* Section-split push: upper = rows 0..27 (main digits + gap/divider),
   * lower = rows 28..VFO_H-1 (sub-line or mode/badge).  Split kept at 28
   * so badge at row 28 always lands in the lower section on both panels. */
  const uint16_t VFO_SPLIT = 28U;

  bool upper_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_hz    != ui->freq_hz
      || s_vfo_cache.active_vfo != ui->active_vfo;

  bool lower_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_b_hz  != ui->freq_b_hz
      || s_vfo_cache.rit_hz     != ui->rit_hz
      || s_vfo_cache.tx_mode    != ui->tx_mode
      || s_vfo_cache.active_vfo != ui->active_vfo
      || s_vfo_cache.tx_power   != ui->tx_power
      || s_vfo_cache.pa_watts   != ui->pa_watts;

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
  s_vfo_cache.tx_power   = ui->tx_power;
  s_vfo_cache.pa_watts   = ui->pa_watts;
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

/* ── UHSDR-style calibrated RF ruler — geometry constants ────────────────── *
 *
 *  Row map (MTR zone):
 *    rows  0–9          scale labels + inline S-value (Font8x10)
 *    rows 10..TICK_END  major ticks (SM_TICK_H_MAJ px), minor ticks (SM_TICK_H_MIN px)
 *    row  SM_RAIL_TOP_R top rail (1-px horizontal line)
 *    rows SM_RAIL_TOP_R+1..SM_RAIL_BOT_R-1   cursor travel zone
 *    row  SM_RAIL_BOT_R bottom rail (1-px horizontal line)
 *    rows SM_RAIL_BOT_R+1..MTR_H-1   TX meter / unused
 *
 *  Moving indicator: SM_LINE_H-px horizontal fill from ruler left edge to
 *  sm_mark_x(bars), centred between rails (rows SM_LINE_R0 .. SM_LINE_R0+SM_LINE_H-1).
 *  Fast path extends or shrinks only the delta columns.
 */
#define SM_LBL_R0     0U    /* label band start (rows 0-9, Font8x10 height) */
#define SM_TICK_R0   10U    /* first tick row */
#if LCD_PANEL == LCD_PANEL_ST7796
#  define SM_TICK_H_MAJ  4U   /* major tick height: rows 10-13 */
#  define SM_TICK_H_MIN  2U   /* minor tick height: rows 12-13 */
#  define SM_RAIL_TOP_R 14U   /* top rail row */
#  define SM_RAIL_BOT_R 22U   /* bottom rail row */
#else /* ST7789 — 24 rows */
#  define SM_TICK_H_MAJ  2U   /* major tick height: rows 10-11 */
#  define SM_TICK_H_MIN  1U   /* minor tick height: row 11 */
#  define SM_RAIL_TOP_R 12U   /* top rail row */
#  define SM_RAIL_BOT_R 18U   /* bottom rail row */
#endif
#define SM_LINE_H   4U        /* signal line height in rows */
#define SM_LINE_R0  ((SM_RAIL_TOP_R + SM_RAIL_BOT_R + 1U - SM_LINE_H) / 2U)  /* centred between rails */
#define SM_VAL_CLR_W  48U     /* pixel columns cleared per row for S-value update */

/* Calibrated signal column — maps bars (0-SM_BARS) to ruler pixel X. */
static inline uint16_t sm_mark_x(int32_t bars)
{
  if (bars <= 0)
    return (uint16_t)SM_START_X;
  uint16_t x  = (uint16_t)(SM_START_X + (uint32_t)(uint16_t)bars * SM_RULER_W / SM_BARS);
  uint16_t hi = (uint16_t)(SM_START_X + SM_RULER_W - 1U);
  return (x < hi) ? x : hi;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  draw_smeter_rows  — full MTR zone redraw, calibrated ruler + signal line
 *
 *  Scale plate: labels, ticks, top rail, bottom rail — all static.
 *  Moving indicator: SM_LINE_H-row horizontal fill from SM_START_X to sm_mark_x(bars).
 * ════════════════════════════════════════════════════════════════════════════ */
static void draw_smeter_rows(int32_t bars)
{
  /* S-value string (inline right of ruler in label band) */
  char s_str[8];
  if (bars <= 9) snprintf(s_str, sizeof(s_str), "S%ld", (long)bars);
  else           snprintf(s_str, sizeof(s_str), "+%ld",  (long)((bars - 9) * 3));

  uint16_t mk_col    = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
  uint16_t ruler_end = (uint16_t)(SM_START_X + SM_RULER_W);
  uint16_t val_x     = (uint16_t)(ruler_end + 4U);
  uint16_t ndx       = sm_mark_x(bars);
  uint16_t tick_c    = SWAP16(UI_SMETER_TICK);
  uint16_t minor_c   = SWAP16(UI_DIVIDER);
  uint16_t cur_c     = SWAP16(UI_SMETER_ACT);

  /* Major tick positions: S(0),S1,S3,S5,S7,S9,+20,+40 at ruler segments */
  static const uint8_t     lbl_seg[8] = { 0U,1U,3U,5U,7U,9U,10U,11U };
  static const char *const lbl_str[8] = { "S","1","3","5","7","9","20","40" };
  /* Minor (unlabeled) tick positions: segments 2, 4, 6, 8 */
  static const uint8_t     min_seg[4] = { 2U, 4U, 6U, 8U };

  uint16_t maj_tx[8], min_tx[4];
  for (uint8_t t = 0U; t < 8U; t++)
    maj_tx[t] = (uint16_t)(SM_START_X + (uint16_t)lbl_seg[t] * SM_UNIT_W);
  for (uint8_t t = 0U; t < 4U; t++)
    min_tx[t] = (uint16_t)(SM_START_X + (uint16_t)min_seg[t] * SM_UNIT_W);

  for (uint16_t row = 0U; row < MTR_H; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0U, MTR_W, UI_MTR_BG);

    /* ── Scale labels + inline S-value (label band, rows 0-9) ── */
    if (row >= SM_LBL_R0 && row < SM_LBL_R0 + (uint16_t)Font8x10.height) {
      uint16_t fr = row - SM_LBL_R0;
      for (uint8_t t = 0U; t < 8U; t++) {
        uint16_t half_w = (uint16_t)(strlen(lbl_str[t]) * Font8x10.width / 2U);
        uint16_t lx     = (maj_tx[t] >= half_w) ? (maj_tx[t] - half_w) : 0U;
        LCD_LineStrW(ln, lx, fr, lbl_str[t], &Font8x10, UI_SMETER_TICK, UI_MTR_BG);
      }
      if (val_x < MTR_W)
        LCD_LineStrW(ln, val_x, fr, s_str, &Font8x10, mk_col, UI_MTR_BG);
    }

    /* ── Major ticks: SM_TICK_H_MAJ rows tall, all 8 labeled positions ── */
    if (row >= SM_TICK_R0 && row < (uint16_t)(SM_TICK_R0 + SM_TICK_H_MAJ)) {
      for (uint8_t t = 0U; t < 8U; t++)
        if (maj_tx[t] < MTR_W) ln[maj_tx[t]] = tick_c;
    }

    /* ── Minor ticks: SM_TICK_H_MIN rows tall (bottom of tick band) ── */
    if (row >= (uint16_t)(SM_TICK_R0 + SM_TICK_H_MAJ - SM_TICK_H_MIN) &&
        row <  (uint16_t)(SM_TICK_R0 + SM_TICK_H_MAJ)) {
      for (uint8_t t = 0U; t < 4U; t++)
        if (min_tx[t] < MTR_W) ln[min_tx[t]] = minor_c;
    }

    /* ── Top rail: 1-px horizontal line spanning ruler width ── */
    if (row == SM_RAIL_TOP_R) {
      for (uint16_t x = SM_START_X; x < ruler_end && x < MTR_W; x++)
        ln[x] = tick_c;
    }

    /* ── Bottom rail: 1-px horizontal line spanning ruler width ── */
    if (row == SM_RAIL_BOT_R && SM_RAIL_BOT_R < MTR_H) {
      for (uint16_t x = SM_START_X; x < ruler_end && x < MTR_W; x++)
        ln[x] = tick_c;
    }

    /* ── Signal line: SM_LINE_H-px fill from ruler left to calibrated column ── */
    if (row >= SM_LINE_R0 && row < (uint16_t)(SM_LINE_R0 + SM_LINE_H)) {
      uint16_t end_x = (ndx < (uint16_t)(MTR_W - 1U)) ? ndx : (uint16_t)(MTR_W - 1U);
      for (uint16_t px = (uint16_t)SM_START_X; px <= end_x; px++)
        ln[px] = cur_c;
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
 *  Only two regions of s_mtr_buf are dynamic:
 *    • Signal line rows SM_LINE_R0..+SM_LINE_H-1   (extend or shrink delta columns)
 *    • Label rows SM_LBL_R0..+Font8x10.height-1    (S-value text)
 *
 *  All other rows (ticks, rails, background) are static and never retransmitted.
 *
 *  Two targeted pushes per tick:
 *    Push A: SM_LINE_H × MTR_W × 2 B   (signal line rows)
 *    Push B: Font8x10.height × MTR_W × 2 B  (label rows)
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
void SDR_UI_UpdateSMeter_SetVoltage(int16_t v_x10) { s_smeter_voltage_x10 = v_x10; }

void SDR_UI_UpdateSMeter(float signal_db)
{
  int32_t bars = (int32_t)((signal_db + 73.0f) / 3.0f);
  if (bars < 0) bars = 0;
  if (bars > (int32_t)SM_BARS) bars = (int32_t)SM_BARS;
  int32_t old_bars = s_rx_meter_bars;
  if (bars == old_bars) return;

  s_rx_meter_bars = bars;

  if (!s_mtr_static_valid) {
    draw_smeter_rows(bars);
    s_mtr_static_valid = true;
    LCD_PushWindow(MTR_X, MTR_Y,
                   (uint16_t)(MTR_X + MTR_W - 1U), MTR_Y2 - 1U,
                   s_mtr_buf, (uint32_t)MTR_W * MTR_H);
    return;
  }

  uint16_t mk_col    = (bars > 9) ? UI_S9P : (bars > 5) ? UI_S7_9 : UI_S1_6;
  uint16_t ruler_end = (uint16_t)(SM_START_X + SM_RULER_W);
  uint16_t val_x     = (uint16_t)(ruler_end + 4U);
  uint16_t bg     = SWAP16(UI_MTR_BG);
  uint16_t cur_c  = SWAP16(UI_SMETER_ACT);

  /* 1. Extend or shrink signal line by delta columns only. */
  {
    uint16_t old_ndx = sm_mark_x(old_bars);
    uint16_t new_ndx = sm_mark_x(bars);

    if (new_ndx > old_ndx) {
      /* Extend: fill new_ndx+1..old_ndx columns with signal colour */
      for (uint16_t row = SM_LINE_R0; row < (uint16_t)(SM_LINE_R0 + SM_LINE_H); row++) {
        if (row >= MTR_H) break;
        uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
        for (uint16_t px = (uint16_t)(old_ndx + 1U); px <= new_ndx && px < MTR_W; px++)
          ln[px] = cur_c;
      }
    } else if (new_ndx < old_ndx) {
      /* Shrink: erase new_ndx+1..old_ndx columns back to background */
      for (uint16_t row = SM_LINE_R0; row < (uint16_t)(SM_LINE_R0 + SM_LINE_H); row++) {
        if (row >= MTR_H) break;
        uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
        for (uint16_t px = (uint16_t)(new_ndx + 1U); px <= old_ndx && px < MTR_W; px++)
          ln[px] = bg;
      }
    }
  }

  /* 2. Update S-value text in label row band */
  {
    char s_str[8];
    if (bars <= 9) snprintf(s_str, sizeof(s_str), "S%ld", (long)bars);
    else           snprintf(s_str, sizeof(s_str), "+%ld",  (long)((bars - 9) * 3));
    uint16_t row_top = SM_LBL_R0;
    uint16_t row_end = (uint16_t)(SM_LBL_R0 + (uint16_t)Font8x10.height);
    if (row_end > MTR_H) row_end = MTR_H;
    for (uint16_t row = row_top; row < row_end; row++) {
      uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
      uint16_t  fr = row - row_top;
      for (uint16_t x = val_x; x < val_x + SM_VAL_CLR_W && x < MTR_W; x++)
        ln[x] = SWAP16(UI_MTR_BG);
      if (val_x < MTR_W)
        LCD_LineStrW(ln, val_x, fr, s_str, &Font8x10, mk_col, UI_MTR_BG);
    }
  }

  /* 3. Two targeted pushes — only dynamic rows retransmitted */
  /* Push A: signal line (SM_LINE_H rows) */
  {
    uint16_t r0 = SM_LINE_R0;
    uint16_t r1 = (uint16_t)(SM_LINE_R0 + SM_LINE_H - 1U);
    if (r1 >= MTR_H) r1 = MTR_H - 1U;
    LCD_PushWindow(MTR_X, (uint16_t)(MTR_Y + r0),
                   (uint16_t)(MTR_X + MTR_W - 1U), (uint16_t)(MTR_Y + r1),
                   s_mtr_buf + (uint32_t)r0 * MTR_W,
                   (uint32_t)MTR_W * (r1 - r0 + 1U));
  }
  /* Push B: label rows (Font8x10.height = 10 rows, S-value updated) */
  {
    uint16_t r0 = SM_LBL_R0;
    uint16_t r1 = (uint16_t)(SM_LBL_R0 + (uint16_t)Font8x10.height - 1U);
    if (r1 >= MTR_H) r1 = MTR_H - 1U;
    LCD_PushWindow(MTR_X, (uint16_t)(MTR_Y + r0),
                   (uint16_t)(MTR_X + MTR_W - 1U), (uint16_t)(MTR_Y + r1),
                   s_mtr_buf + (uint32_t)r0 * MTR_W,
                   (uint32_t)MTR_W * (r1 - r0 + 1U));
  }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  TX meter — ruler + marker (same geometry as RX, different scale labels)
 *
 *  Shares SM_LBL_R0, SM_TICK_R0, SM_RAIL_TOP_R, SM_RAIL_BOT_R.
 *  ALC scale: "0  25  50  75  100"  at segments {0,3,6,9,11}.
 *  Marker: sm_mark_x(alc_b) — no fill bar.
 *  Value text: one Font5x8 line (8 px) below bottom rail: "ALC XX%  SWR X.X"
 * ════════════════════════════════════════════════════════════════════════════ */
#if LCD_PANEL == LCD_PANEL_ST7796
#  define TX_VAL_R0  ((uint16_t)(SM_RAIL_BOT_R + 3U))   /* 2-row gap on ST7796 */
#else
#  define TX_VAL_R0  ((uint16_t)(SM_RAIL_BOT_R + 2U))   /* 1-row gap on ST7789 */
#endif

static void tx_meter_render_rows(uint16_t row0, uint16_t row1,
                                 int32_t alc_b, int32_t alc_pct,
                                 int32_t swr_x10)
{
  /* ALC% string: "75%" or "100%" */
  char alc_val[6];
  {
    uint8_t n = 0;
    int32_t v = alc_pct;
    if (v >= 100) { alc_val[n++] = '1'; alc_val[n++] = '0'; alc_val[n++] = '0'; }
    else if (v >= 10) { alc_val[n++] = (char)('0' + v / 10); alc_val[n++] = (char)('0' + v % 10); }
    else { alc_val[n++] = (char)('0' + v); }
    alc_val[n++] = '%';
    alc_val[n]   = '\0';
  }
  char swr_val[8];
  fmt_1dp(swr_val, sizeof(swr_val), swr_x10, '\0');
  uint16_t swr_col  = (swr_x10 >= 30) ? UI_S9P : (swr_x10 >= 20) ? UI_S7_9 : UI_S1_6;
  uint16_t alc_col  = (alc_b > 9) ? UI_S9P : (alc_b > 6) ? UI_S7_9 : UI_S1_6;
  uint16_t ruler_end = (uint16_t)(SM_START_X + SM_RULER_W);
  uint16_t ndx       = sm_mark_x(alc_b);
  uint16_t pm        = SWAP16(alc_col);
  uint16_t tick_c    = SWAP16(UI_SMETER_TICK);

  /* ALC scale labels: "0", "25", "50", "75", "100" at ruler segments */
  static const char *const slbls[] = { "0", "25", "50", "75", "100" };
  static const uint8_t     spos[]  = { 0U,   3U,   6U,   9U,   11U };
  uint16_t stx[5];
  for (uint8_t t = 0U; t < 5U; t++)
    stx[t] = (uint16_t)(SM_START_X + (uint16_t)spos[t] * SM_UNIT_W);

  /* Value-line x positions (below bottom rail) */
  uint16_t alc_lbl_x = SM_START_X;
  uint16_t alc_val_x = (uint16_t)(SM_START_X + 4U * Font6x8.width);  /* "ALC " = 4 chars */
  uint16_t swr_lbl_x = (uint16_t)(SM_START_X + 9U * Font6x8.width);  /* after "ALC XXX%" */
  uint16_t swr_val_x = (uint16_t)(SM_START_X + 13U * Font6x8.width);

  if (row1 >= MTR_H) row1 = MTR_H - 1U;
  for (uint16_t row = row0; row <= row1; row++) {
    uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
    LCD_LineFill(ln, 0U, MTR_W, UI_MTR_BG);

    /* ── ALC scale labels (same band as RX labels) ── */
    if (row >= SM_LBL_R0 && row < (uint16_t)(SM_LBL_R0 + Font8x10.height)) {
      uint16_t fr = row - SM_LBL_R0;
      for (uint8_t t = 0U; t < 5U; t++) {
        uint16_t half_w = (uint16_t)(strlen(slbls[t]) * Font8x10.width / 2U);
        uint16_t lx     = (stx[t] >= half_w) ? (stx[t] - half_w) : 0U;
        LCD_LineStrW(ln, lx, fr, slbls[t], &Font8x10, UI_SMETER_TICK, UI_MTR_BG);
      }
      /* ALC% inline right of ruler */
      uint16_t vx = (uint16_t)(ruler_end + 4U);
      if (vx < MTR_W)
        LCD_LineStrW(ln, vx, fr, alc_val, &Font8x10, alc_col, UI_MTR_BG);
    }

    /* ── Major ticks: SM_TICK_H_MAJ rows, 5 positions ── */
    if (row >= SM_TICK_R0 && row < (uint16_t)(SM_TICK_R0 + SM_TICK_H_MAJ)) {
      for (uint8_t t = 0U; t < 5U; t++)
        if (stx[t] < MTR_W) ln[stx[t]] = tick_c;
    }

    /* ── Top rail ── */
    if (row == SM_RAIL_TOP_R) {
      for (uint16_t x = SM_START_X; x < ruler_end && x < MTR_W; x++)
        ln[x] = tick_c;
    }

    /* ── Bottom rail ── */
    if (row == SM_RAIL_BOT_R && SM_RAIL_BOT_R < MTR_H) {
      for (uint16_t x = SM_START_X; x < ruler_end && x < MTR_W; x++)
        ln[x] = tick_c;
    }

    /* ── ALC signal line: SM_LINE_H-px centered fill, ruler left to calibrated column ── */
    if (row >= SM_LINE_R0 && row < (uint16_t)(SM_LINE_R0 + SM_LINE_H)) {
      uint16_t end_x = (ndx < (uint16_t)(MTR_W - 1U)) ? ndx : (uint16_t)(MTR_W - 1U);
      for (uint16_t px = (uint16_t)SM_START_X; px <= end_x; px++)
        ln[px] = pm;
    }

    /* ── Value line: "ALC XX%   SWR X.X" below bottom rail ── */
    if (row >= TX_VAL_R0 && (row - TX_VAL_R0) < (uint16_t)Font6x8.height) {
      uint16_t fr = row - TX_VAL_R0;
      LCD_LineStr(ln, alc_lbl_x, fr, "ALC", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, alc_val_x, fr, alc_val, &Font6x8, alc_col, UI_MTR_BG);
      LCD_LineStr(ln, swr_lbl_x, fr, "SWR", &Font6x8, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStr(ln, swr_val_x, fr, swr_val, &Font6x8, swr_col, UI_MTR_BG);
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
 *
 *  Static rows (labels, ticks, rails): pushed once on first TX frame.
 *  Dynamic rows: cursor zone (SM_RAIL_TOP_R..SM_RAIL_BOT_R) + value line.
 *  Both dynamic zones pushed independently when their value changes.
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdateTXMeters(int32_t alc_pct, int32_t swr_x10)
{
  int32_t alc_b = (alc_pct * (int32_t)SM_BARS + 50) / 100;
  if (alc_b < 0) alc_b = 0;
  if (alc_b > (int32_t)SM_BARS) alc_b = (int32_t)SM_BARS;
  if (alc_pct < 0) alc_pct = 0;
  if (alc_pct > 100) alc_pct = 100;
  if (swr_x10 < 0) swr_x10 = 0;
  if (swr_x10 > 999) swr_x10 = 999;

  bool first = !s_tx_meter_active;
  if (first) s_tx_meter_active = true;

  /* Static content: label band, ticks, rails — render + push on first frame */
  if (first) {
    uint16_t static_end = (TX_VAL_R0 - 1U < MTR_H) ? (TX_VAL_R0 - 1U) : (uint16_t)(MTR_H - 1U);
    tx_meter_render_rows(0U, static_end, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(0U, static_end);
  }

  /* Dynamic: cursor zone — push when ALC bar position changes */
  if (first || alc_b != s_tx_alc_bars) {
    uint16_t r0 = SM_RAIL_TOP_R;
    uint16_t r1 = SM_RAIL_BOT_R;
    if (r1 >= MTR_H) r1 = MTR_H - 1U;
    tx_meter_render_rows(r0, r1, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(r0, r1);
    s_tx_alc_bars = alc_b;
  }

  /* Dynamic: value line (ALC% + SWR) — push when either value changes */
  if (first || alc_pct != s_tx_alc_pct || swr_x10 != s_tx_swr_x10) {
    uint16_t r0 = TX_VAL_R0;
    uint16_t r1 = (uint16_t)(TX_VAL_R0 + (uint16_t)Font6x8.height - 1U);
    if (r0 < MTR_H) {
      if (r1 >= MTR_H) r1 = MTR_H - 1U;
      tx_meter_render_rows(r0, r1, alc_b, alc_pct, swr_x10);
      tx_meter_push_rows(r0, r1);
    }
    s_tx_alc_pct = alc_pct;
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
    uint16_t bi   = (uint16_t)(fbin + 0.5f);
    if (bi >= bins) bi = (uint16_t)(bins - 1U);
    s_spec_yf[x] = pwr_compress(fft_db[bi]);
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

  uint16_t spec_sw      = SWAP16(0xC7FFU);   /* icy white-blue: top     */
  uint16_t spec_fill_sw = SWAP16(0x3D7FU);   /* muted cold cyan: body   */
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

  /* Draw filled cyan/teal spectrum columns — bright top pixel, darker body. */
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

  uint16_t nb = bins;
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
 *  memmove scroll: shift all rows down by one, write newest at s_wf_buf[0],
 *  then push the full WF zone in SPEC_CHUNK_ROWS strips.  Newest data always
 *  appears at WF_Y (top); history scrolls toward WF_Y2 (bottom).
 *
 *  CPU: memmove (WF_H-1) × WF_W × 2 B ≈ 230 µs on D1 AXI.
 *  DMA: WF_W × WF_H × 2 B = 69,120 B ≈ 8.1 ms over 8-bit FMC.
 *  At 75 ms frame period (13 fps): ~8.3 ms / 75 ms = 11 % FMC time.
 *
 *  Synchronisation invariant:
 *    s_wf_buf must not be modified (memmove or row write) while LCD DMA is
 *    actively reading from it.  Two explicit LCD_Wait() barriers enforce this:
 *      1. Before memmove — guards the entire buffer rewrite.
 *      2. After the final strip push — ensures the function never exits with
 *         an in-flight DMA on s_wf_buf; any subsequent buffer touch is safe.
 *
 *  Skipped when s_wf_suppressed is set by adaptive load control.
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_WaterfallPush(uint8_t buf_idx)
{
  if (s_wf_suppressed) return;

  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_WF_SCROLL);

  /* Wait for any in-flight spec DMA before memmove touches s_wf_buf. */
  LCD_Wait();

  memmove(&s_wf_buf[1][0], &s_wf_buf[0][0],
          (WF_H - 1U) * WF_W * sizeof(uint16_t));
  const uint8_t *src = s_wf_idx[buf_idx];
  uint16_t      *row = s_wf_buf[0];
  for (uint16_t x = 0U; x < WF_W; x++) row[x] = s_wf_lut[src[x]];

  /* Synchronous CPU push — no DMA.
   * The ring-buffer code (1 row/frame, async DMA) never produced white screen.
   * The memmove code (9 strips/frame, async DMA) does.  Root-cause hypothesis:
   * DMA2 M2M FIFO (FULL threshold) occasionally generates a spurious extra byte
   * to the FMC data register under SAI DMA bus contention.  At 9 strips × 13fps
   * the FIFO runs ~72× more operations per second than the old ring path; the
   * extra byte shifts the LCD RAMWR counter, eventually desyncing the command
   * stream and producing a fully-white display.  CPU writes are byte-exact. */
  uint32_t cyc0 = DWT->CYCCNT;
  LCD_PushWindow(WF_X, WF_Y,
                 (uint16_t)(WF_X + WF_W - 1U), (uint16_t)(WF_Y + WF_H - 1U),
                 &s_wf_buf[0][0], (uint32_t)WF_W * WF_H);
  uint32_t row_us = ui_cyc_to_us(DWT->CYCCNT - cyc0);

  s_lcd_chunk_count++;
  if (row_us > s_max_chunk_render_us) s_max_chunk_render_us = row_us;

  RuntimeDiag_LcdChunkReport(s_lcd_chunk_count, s_lcd_chunk_abort_count,
                               s_wf_partial_count, s_max_chunk_render_us);

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
