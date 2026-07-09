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
#include "pa_protect.h" /* PA_State_t / PA_Fault_t for TX warning overlay */
#include "rtc_clock.h"  /* RTC_Clock_GetTime / SetTime */
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
 *   s_sbr_buf  : 96×88 ×2 =  16,896 B   — (not on ST7789)
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
/* Compact status bar: only allocated when STS_H > 0 */
#if STS_H > 0U
static uint16_t s_sts_buf[STS_H * LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
#endif
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

/* PA warn zone: full INFO-zone buffer (ST7796 only: 24×480×2 = 23 KB).
 * Not allocated on ST7789 (INFO_H = 0). */
#if INFO_H > 0
static uint16_t s_info_buf[INFO_H][LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));
#endif

/* WF pre-compute: two uint8_t line buffers (double-buffer for DSP/UI split) */
static uint8_t  s_wf_idx[2][WF_W];   /* 2 × WF_W bytes */
static volatile uint8_t s_wf_fill = 0;

/* Adaptive waterfall suppression (set by csdr_app under high audio load) */
static volatile bool s_wf_suppressed = false;

/* TX zone blank flag — cleared on TX→RX so next TX session re-blanks */
static bool s_tx_zone_blanked = false;

/* PA warn INFO-zone cache */
static PA_State_t s_pa_warn_drawn  = (PA_State_t)0xFFU;
static PA_Fault_t s_pa_wflt_drawn  = (PA_Fault_t)0xFFU;

/* Clock display is driven by RTC hardware (rtc_clock.h) — no SRAM state needed */

/* CPU-only: IIR smoother + colour LUT + ring head */
static float    s_wf_smooth[DSP_FFT_SIZE];
static uint16_t s_wf_lut[256];
static uint8_t  s_wf_head = 0;
static int16_t  s_smeter_voltage_x10 = 0;
static int16_t  s_rssi_db            = -200;

/* Spectrum delta-skip: previous column pixel rows */
static uint16_t s_spec_py_prev[SPEC_W];
static bool     s_spec_py_valid = false;
static int32_t  s_spec_marker_hz = 0;   /* Track-mode demod offset from center, Hz */

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
  uint32_t fwd_power_mw;
  /* Inline-params fields (ST7789 portrait right panel) */
  uint8_t  mode;
  uint8_t  volume;
  uint8_t  squelch;
  uint32_t step;
  uint32_t bw_hz;
  bool     nb_on;
  bool     nr_on;
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
    float ng = (n > 0.0f) ? expf(1.3f * logf(n)) : 0.0f;
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

static uint16_t med_str_w(const char *s)
{
  uint16_t w = 0U;
  while (*s) { w += (*s++ == '.') ? 6U : MED_W; }
  return w;
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

/* ── Partial-redraw cache invalidation ───────────────────────────────────────
 * Call after a full-screen overlay (e.g. SWR scan) has painted over the normal
 * UI zones.  Every partial-redraw cache (VFO glyph, sidebar values, meter
 * statics, spectrum delta-skip, RSSI) believes the LCD still shows its last
 * push; without invalidation the next DIRTY_ALL refresh skips those zones and
 * the overlay's pixels stay on screen. */
void SDR_UI_InvalidateCaches(void)
{
  s_vfo_cache.valid    = false;
  s_sbl_cache.valid    = false;
  s_sbr_cache.valid    = false;
  s_spec_py_valid      = false;
  s_mtr_static_valid   = false;
  s_rx_meter_bars      = -1;     /* force full meter redraw on next tick   */
  s_rssi_db            = -200;   /* force INFO-strip RSSI redraw           */
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
 *   RX→TX: blank SPEC and WF zones immediately so the display switches
 *           without waiting for the first FFT frame.
 *   TX→RX: reset zone-blank flag so the next TX session re-blanks; also
 *           invalidate the RX spectrum prev-row cache so the first post-TX
 *           spectrum draw repaints from scratch rather than delta-skipping. */
bool SDR_UI_IsTXZoneBlanked(void) { return s_tx_zone_blanked; }

void SDR_UI_SetTXMode(bool tx_active)
{
  if (tx_active) {
    if (!s_tx_zone_blanked) {
      /* Blank SPEC zone — fill and push all strips */
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
      /* Blank WF zone */
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
      /* Invalidate delta-skip cache now: even if do_trip() fires before
       * SetTXMode(false) is called (race via cat_tx_dirty, up to 10 ms),
       * the first post-TX DrawSpectrum will do a full redraw rather than
       * comparing new RX peaks against stale pre-TX peaks and delta-skipping. */
      s_spec_py_valid   = false;
    }
  } else {
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
static uint8_t  s_spec_zoom     = 0U;
static uint32_t s_spec_sr       = 48000U;
static uint32_t s_spec_orig_sr  = 48000U;   /* SR at DrawFrame (never changes with zoom) */
static uint16_t s_spec_bins     = 256U;
static uint32_t s_footer_freq_hz = 0U;      /* VFO frequency for absolute ruler labels */
static uint32_t s_footer_step_hz = 0U;      /* Tuning step for grid spacing */

#define FMARK_MAX 16U
typedef struct {
  uint16_t px;
  uint16_t lbl_lx;   /* 0xFFFFU = label suppressed */
  char     lbl[10];
} FootMark_t;
static FootMark_t s_fmarks[FMARK_MAX];
static uint8_t    s_n_fmarks = 0U;

static uint32_t pick_grid_hz(uint32_t step_hz, uint32_t half_hz)
{
  /* Target ≤8 intermediate marks per half-span: min grid = half_hz/4.
   * Round up to next multiple of step_hz so marks land on tuning positions. */
  if (step_hz == 0U) return 0U;
  uint32_t min_g = half_hz / 4U;
  if (min_g < step_hz) min_g = step_hz;
  return ((min_g + step_hz - 1U) / step_hz) * step_hz;
}

/* Display-crop table (zoom 0 only, others use full decimated FFT) */
static const uint8_t s_zoom_half[SPEC_ZOOM_COUNT] = { 128U, 128U, 128U, 128U };

/* Per-zoom effective sample rate and half-span override for decimation.
 * zoom 0: decimate×1, full ±24kHz FFT (half_ovr=0, uses s_zoom_half[0]=128).
 * zoom 1/2/3: decimate×2/4/8; show full decimated FFT (half_ovr=128). */
static const uint32_t s_zoom_sr_tbl  [SPEC_ZOOM_COUNT] = {48000U, 24000U, 12000U, 6000U};
static const uint8_t  s_zoom_half_ovr[SPEC_ZOOM_COUNT] = {0U, 128U, 128U, 128U};
static uint8_t s_spec_half_ovr = 0U;  /* 0 = use s_zoom_half table; 128 = full FFT */

static void spec_window(uint16_t bins, uint16_t *b0_out, uint16_t *n_vis_out)
{
  uint16_t center = bins >> 1U;
  uint8_t  zh     = s_spec_half_ovr ? s_spec_half_ovr : s_zoom_half[s_spec_zoom];
  uint16_t half   = (uint16_t)((uint32_t)zh * bins / 256U);
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

/* Format Hz as "MHz.kkk" (e.g. 14225000 → "14.225") using integer only. */
static void fmt_mhz(char *buf, size_t sz, uint32_t hz)
{
  snprintf(buf, sz, "%lu.%03lu",
           (unsigned long)(hz / 1000000U),
           (unsigned long)((hz % 1000000U) / 1000U));
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
  uint16_t rx_x  = 0U;
  uint16_t cx_px = (uint16_t)(LCD_W / 2U);

  /* ── Left / right edge labels ────────────────────────────────────────────── */
  if (half_hz > 0U) {
    if (s_footer_freq_hz > 0U) {
      uint32_t f_lo = (s_footer_freq_hz > half_hz) ? (s_footer_freq_hz - half_hz) : 0U;
      fmt_mhz(lbuf, sizeof(lbuf), f_lo);
      fmt_mhz(rbuf, sizeof(rbuf), s_footer_freq_hz + half_hz);
    } else {
      uint32_t bk = half_hz / 1000U;
      snprintf(lbuf, sizeof(lbuf), "-%luK", (unsigned long)bk);
      snprintf(rbuf, sizeof(rbuf), "+%luK", (unsigned long)bk);
    }
    rx_x = (uint16_t)(LCD_W - (uint16_t)(strlen(rbuf) * Font6x8.width) - 4U);
  }

  /* ── Step-based intermediate grid marks ─────────────────────────────────── */
  s_n_fmarks = 0U;
  if (s_footer_freq_hz > 0U && s_footer_step_hz > 0U && half_hz > 0U) {
    uint32_t grid_hz = pick_grid_hz(s_footer_step_hz, half_hz);
    uint32_t f_left  = (s_footer_freq_hz > half_hz) ? (s_footer_freq_hz - half_hz) : 0U;
    uint32_t f_right = s_footer_freq_hz + half_hz;
    uint32_t span_hz = 2U * half_hz;
    /* edge guard: labels close to the screen edge would overlap the edge labels */
    uint16_t left_guard  = (uint16_t)(strlen(lbuf) * Font6x8.width + 8U);
    uint16_t right_guard = (rx_x > 4U) ? (uint16_t)(rx_x - 4U) : 0U;
    /* first grid mark at or above f_left */
    uint32_t f_mark = (grid_hz > 0U) ? ((f_left / grid_hz) * grid_hz) : f_left;
    if (f_mark < f_left) f_mark += grid_hz;
    for (; f_mark <= f_right && s_n_fmarks < FMARK_MAX; f_mark += grid_hz) {
      if (f_mark == s_footer_freq_hz) continue;  /* center drawn separately */
      uint16_t px = (uint16_t)((uint32_t)(f_mark - f_left) * LCD_W / span_hz);
      if (px < 2U || px >= (uint16_t)(LCD_W - 1U)) continue;
      s_fmarks[s_n_fmarks].px = px;
      fmt_mhz(s_fmarks[s_n_fmarks].lbl, sizeof(s_fmarks[0].lbl), f_mark);
      uint16_t lw = (uint16_t)(strlen(s_fmarks[s_n_fmarks].lbl) * Font6x8.width);
      uint16_t lx = (px > lw / 2U) ? (uint16_t)(px - lw / 2U) : 0U;
      if (lx + lw > LCD_W) lx = (uint16_t)(LCD_W - lw);
      /* suppress if would overlap the edge labels */
      s_fmarks[s_n_fmarks].lbl_lx =
        (lx >= left_guard && lx + lw <= right_guard) ? lx : 0xFFFFU;
      s_n_fmarks++;
    }
    /* suppress labels that overlap each other (left-to-right sweep) */
    uint16_t last_rx = 0U;
    for (uint8_t i = 0U; i < s_n_fmarks; i++) {
      if (s_fmarks[i].lbl_lx == 0xFFFFU) continue;
      uint16_t lbl_w = (uint16_t)(strlen(s_fmarks[i].lbl) * Font6x8.width);
      if (s_fmarks[i].lbl_lx < last_rx + 4U) {
        s_fmarks[i].lbl_lx = 0xFFFFU;
      } else {
        last_rx = (uint16_t)(s_fmarks[i].lbl_lx + lbl_w);
      }
    }
  }

  /* ── Tick geometry ───────────────────────────────────────────────────────── */
  uint16_t fh       = Font6x8.height;
  uint16_t pad      = (FTR_H - fh) / 2U;
  uint16_t tick_end = (pad >= 2U) ? (uint16_t)(pad - 2U) : 0U;
  uint16_t tmaj0    = (tick_end >= 4U) ? (uint16_t)(tick_end - 4U) : 0U;
  uint16_t tmed0    = (tick_end >= 2U) ? (uint16_t)(tick_end - 2U) : 0U;
  uint16_t tick_maj = SWAP16(UI_STATUS_VAL);
  uint16_t tick_med = SWAP16(UI_SMETER_TICK);

  for (uint16_t row = 0U; row < FTR_H; row++) {
    uint16_t *ln = LCD_GetLineBuf();
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);

    /* center tick: 4-row, bright */
    if (row >= tmaj0 && row < tick_end && cx_px < LCD_W)
      ln[cx_px] = tick_maj;

    /* grid ticks: 2-row, dim */
    if (row >= tmed0 && row < tick_end) {
      for (uint8_t i = 0U; i < s_n_fmarks; i++) {
        if (s_fmarks[i].px < LCD_W) ln[s_fmarks[i].px] = tick_med;
      }
    }

    if (half_hz > 0U && row >= pad && row < pad + fh) {
      uint16_t frow = row - pad;
      /* edge labels */
      LCD_LineStr(ln, 4U,   frow, lbuf, &Font6x8, UI_SMETER_TICK, UI_BG);
      LCD_LineStr(ln, rx_x, frow, rbuf, &Font6x8, UI_SMETER_TICK, UI_BG);
      /* grid mark labels */
      for (uint8_t i = 0U; i < s_n_fmarks; i++) {
        if (s_fmarks[i].lbl_lx != 0xFFFFU)
          LCD_LineStr(ln, s_fmarks[i].lbl_lx, frow,
                      s_fmarks[i].lbl, &Font6x8, UI_SMETER_TICK, UI_BG);
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
  s_spec_sr      = sample_rate ? sample_rate : 48000U;
  s_spec_orig_sr = s_spec_sr;
  s_spec_bins    = fft_bins    ? fft_bins    : 256U;

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
  uint16_t txt_y   = (uint16_t)((HDR_H - Font8x10.height) / 2U);
  static const char *agc_labels[3] = { "AGC:SLOW", "AGC:FAST", "AGC:AUTO" };
  const char *agc_str = (ui->agc_speed < 3U) ? agc_labels[ui->agc_speed] : "AGC:SLOW";

  char att_hdr[16];
  if (ui->att_x2 & 1U)
    snprintf(att_hdr, sizeof(att_hdr), "ATT:%u.5DB", ui->att_x2 / 2U);
  else
    snprintf(att_hdr, sizeof(att_hdr), "ATT:%uDB", ui->att_x2 / 2U);
  uint16_t att_vc  = ui->rf_agc_on ? UI_STATUS_ON : UI_STATUS_VAL;
  uint16_t sep1_x  = (uint16_t)(4U + (uint16_t)(strlen(agc_str) * Font8x10.width) + 4U);
  uint16_t att_x   = (uint16_t)(sep1_x + (uint16_t)Font8x10.width + 4U);
  uint16_t left_end = (uint16_t)(att_x + (uint16_t)(strlen(att_hdr) * Font8x10.width) + 4U);
  uint16_t volt_x  = (uint16_t)(LCD_W - (uint16_t)(strlen(vstr) * Font8x10.width) - 4U);

  /* ── RTC clock: HH:MM:SS ────────────────────────────────────────────────── */
  uint8_t  clk_h, clk_m, clk_s;
  RTC_Clock_GetTime(&clk_h, &clk_m, &clk_s);
  char     clk_str[9];
  /* Range-clamp keeps each field at 2 digits — guards a garbage RTC read and
   * lets the compiler prove the buffer fits (silences -Wformat-truncation) */
  snprintf(clk_str, sizeof(clk_str), "%02u:%02u:%02u",
           (unsigned)(clk_h % 24U), (unsigned)(clk_m % 60U), (unsigned)(clk_s % 60U));
  uint16_t clk_w      = (uint16_t)(8U * (uint16_t)Font8x10.width);
  uint16_t sep_clk_x  = (uint16_t)(volt_x - (uint16_t)Font8x10.width - 4U);
  uint16_t clock_x    = (uint16_t)(sep_clk_x - clk_w - 4U);
  /* Use clock slot only when it doesn't overrun the ATT label */
  bool     clk_fits   = (clock_x > left_end + 8U);
  uint16_t sep2_x     = clk_fits
                        ? (uint16_t)(clock_x - (uint16_t)Font8x10.width - 4U)
                        : (uint16_t)(volt_x   - (uint16_t)Font8x10.width - 4U);

  /* ── Hardware warning: centred between ATT label and sep2 ─ */
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
    uint16_t warn_w = (uint16_t)((uint16_t)strlen(warn_str) * Font8x10.width);
    uint16_t avail  = (sep2_x > left_end + 4U) ? (uint16_t)(sep2_x - left_end - 4U) : 0U;
    warn_x = (avail > warn_w)
             ? (uint16_t)(left_end + (avail - warn_w) / 2U)
             : left_end;
  }

  for (uint16_t row = 0; row < HDR_H; row++) {
    uint16_t *ln = s_hdr_buf + (uint32_t)row * LCD_W;
    if (row == HDR_H - 1U) {
      LCD_LineFill(ln, 0, LCD_W, UI_DIVIDER);
      continue;
    }
    LCD_LineFill(ln, 0, LCD_W, UI_HDR_BG);
    LCD_LineFill(ln, sep1_x, 1U, UI_DIVIDER);
    LCD_LineFill(ln, sep2_x, 1U, UI_DIVIDER);
    if (clk_fits) {
      LCD_LineFill(ln, sep_clk_x, 1U, UI_DIVIDER);
    }
    if (row >= txt_y && row < txt_y + Font8x10.height) {
      uint16_t fr = row - txt_y;
      LCD_LineStrW(ln, 4U,     fr, agc_str,  &Font8x10, UI_STATUS_LBL, UI_HDR_BG);
      LCD_LineStrW(ln, att_x,  fr, att_hdr,  &Font8x10, att_vc,        UI_HDR_BG);
      if (clk_fits) {
        LCD_LineStrW(ln, clock_x, fr, clk_str, &Font8x10, UI_STATUS_HINT, UI_HDR_BG);
      }
      LCD_LineStrW(ln, volt_x, fr, vstr,     &Font8x10, vcol,          UI_HDR_BG);
      if (warn_str[0]) {
        LCD_LineStrW(ln, warn_x, fr, warn_str, &Font8x10, UI_STATUS_WARN, UI_HDR_BG);
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
#if LCD_PANEL == LCD_PANEL_ST7789 && STS_H > 0U
static void draw_compact_status(const SDR_UI_State_t *ui)
{
#if STS_H == 0U
  /* New portrait layout: status info is embedded in the VFO zone — no-op here */
  (void)ui;
  return;
#endif /* STS_H == 0U */
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

  static const char *const mode_s[] = {"AM","FM","USB","LSB","CW","DIGU","DIGL","FDV"};
  const char *mode_str = (ui->mode < 8U) ? mode_s[ui->mode] : "---";

  char vol_str[8]; snprintf(vol_str, sizeof(vol_str), "VOL:%u",  ui->volume);
  char sql_str[8]; snprintf(sql_str, sizeof(sql_str), "SQL:%u",  ui->squelch);

  /* uint32_t = long unsigned int on this toolchain → %lu.
   * Decimal branch avoids snprintf: two %lu would need a 26-byte buffer.
   * Integer split used directly (same pattern as fmt_1dp / no-float rule). */
  char bw_str[16];
  if (ui->bw_hz >= 10000U)
    snprintf(bw_str, sizeof(bw_str), "BW:%luk",
             (unsigned long)(ui->bw_hz / 1000U));
  else if (ui->bw_hz >= 1000U) {
    uint32_t k = ui->bw_hz / 1000U;           /* 1-9 */
    uint32_t f = (ui->bw_hz % 1000U) / 100U;  /* 0-9 */
    bw_str[0]='B'; bw_str[1]='W'; bw_str[2]=':';
    bw_str[3]=(char)('0'+(int)k); bw_str[4]='.';
    bw_str[5]=(char)('0'+(int)f); bw_str[6]='k'; bw_str[7]='\0';
  } else
    snprintf(bw_str, sizeof(bw_str), "BW:%luHz",
             (unsigned long)ui->bw_hz);

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

  /* Vertical placement: two Font8x10 rows (height=10 each).
   * Landscape STS_H=24: margin 2, gap 2 — row0 rows 2-11, row1 rows 14-23.
   * Portrait  STS_H=28: margin 4, gap 4 — row0 rows 4-13, row1 rows 18-27. */
#if LCD_W > LCD_H
  const uint16_t row0_y = 2U;
  const uint16_t row1_y = (uint16_t)(row0_y + Font8x10.height + 2U);  /* = 14 */
#else
  const uint16_t row0_y = 4U;
  const uint16_t row1_y = (uint16_t)(row0_y + Font8x10.height + 4U);  /* = 18 */
#endif

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
#endif /* LCD_PANEL_ST7789 && STS_H > 0 */

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
#if STS_H == 0U
  /* New portrait layout: params are in the VFO zone — delegate so DIRTY_SBL
   * triggers a VFO re-render when vol/sql/nr/nb change */
  SDR_UI_DrawVFO(ui);
  return;
#else
  draw_compact_status(ui);
  return;
#endif /* STS_H */
#endif /* LCD_PANEL_ST7789 */

  /* Cache guard — skip rebuild when nothing changed */
  if (s_sbl_cache.valid
      && s_sbl_cache.volume     == ui->volume
      && s_sbl_cache.squelch    == ui->squelch
      && s_sbl_cache.nr_on      == ui->nr_on
      && s_sbl_cache.nb_on      == ui->nb_on
      && s_sbl_cache.active_vfo == ui->active_vfo) return;

  s_sbl_cache.volume     = ui->volume;
  s_sbl_cache.squelch    = ui->squelch;
  s_sbl_cache.nr_on      = ui->nr_on;
  s_sbl_cache.nb_on      = ui->nb_on;
  s_sbl_cache.active_vfo = ui->active_vfo;
  s_sbl_cache.valid      = true;

  char vol_str[6]; snprintf(vol_str, sizeof(vol_str), "%u", ui->volume);
  char sql_str[6]; snprintf(sql_str, sizeof(sql_str), "%u", ui->squelch);

  buf_fill(s_sbl_buf, (uint32_t)SBL_H * SBL_W, UI_SBL_BG);

  /* 4 items × 24 rows = 96 rows (SBL_H).
   * All text in MED (12×16): label left x=2, value right-aligned — same row.
   * val_off=4 → MED rows 4..19 within each 24-px slot.
   * Width budget: "VOL"(36)+"100"(36)+4gap+2+2 = 80px exactly. */
  const uint16_t item_h  = 24U;
  const uint16_t val_off =  4U;

  uint16_t col_a = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_LBL;
  uint16_t col_b = (ui->active_vfo == 1U) ? UI_STATUS_ON  : UI_STATUS_LBL;
  uint16_t nr_bg = ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF;
  uint16_t nb_bg = ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF;
  uint16_t vol_x = (uint16_t)(SBL_W - 2U - med_str_w(vol_str));
  uint16_t sql_x = (uint16_t)(SBL_W - 2U - med_str_w(sql_str));

  for (uint8_t i = 0; i < 4U; i++) {
    uint16_t y0 = (uint16_t)(i * item_h);

    for (uint16_t row = 0; row < item_h; row++) {
      uint16_t abs_r = y0 + row;
      if (abs_r >= SBL_H) break;
      uint16_t *ln = s_sbl_buf + (uint32_t)abs_r * SBL_W;

      if (i > 0U && row == 0U) { LCD_LineFill(ln, 0, SBL_W, UI_DIVIDER); continue; }

      switch (i) {
        case 0:  /* VFO — "VFO" left, A/B right, all MED */
          if (row >= val_off && row < val_off + MED_H) {
            uint16_t fr = row - val_off;
            ln_medstr(ln,  2U, fr, "VFO", UI_STATUS_LBL, UI_SBL_BG);
            ln_medchar(ln, 44U, fr, 'A',  col_a,         UI_SBL_BG);
            ln_medchar(ln, 56U, fr, '/',  UI_STATUS_LBL, UI_SBL_BG);
            ln_medchar(ln, 68U, fr, 'B',  col_b,         UI_SBL_BG);
          }
          break;

        case 1:  /* NR / NB color badges */
          if (row >= val_off && row < val_off + MED_H + 4U) {
            LCD_LineFill(ln,  2U, 34U, nr_bg);
            LCD_LineFill(ln, 44U, 34U, nb_bg);
            if (row >= val_off + 2U && row < val_off + 2U + MED_H) {
              uint16_t fr = row - (val_off + 2U);
              ln_medstr(ln,  7U, fr, "NR", UI_BG, nr_bg);
              ln_medstr(ln, 49U, fr, "NB", UI_BG, nb_bg);
            }
          }
          break;

        case 2:  /* VOL — "VOL" left, value right, all MED */
          if (row >= val_off && row < val_off + MED_H) {
            uint16_t fr = row - val_off;
            ln_medstr(ln,  2U,   fr, "VOL", UI_STATUS_LBL, UI_SBL_BG);
            ln_medstr(ln, vol_x, fr, vol_str, UI_STATUS_VAL, UI_SBL_BG);
          }
          break;

        case 3:  /* SQL — "SQL" left, value right, all MED */
          if (row >= val_off && row < val_off + MED_H) {
            uint16_t fr = row - val_off;
            ln_medstr(ln,  2U,   fr, "SQL", UI_STATUS_LBL, UI_SBL_BG);
            ln_medstr(ln, sql_x, fr, sql_str, UI_STATUS_VAL, UI_SBL_BG);
          }
          break;

        default: break;
      }
    }
  }

  LCD_PushWindow(SBL_X, SBL_Y,
                 (uint16_t)(SBL_X + SBL_W - 1U), SBL_Y2 - 1U,
                 s_sbl_buf, (uint32_t)SBL_W * SBL_H);
}


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
#if STS_H == 0U
  /* New portrait layout: BW/STP are in VFO zone — delegate so DIRTY_SBR
   * triggers a VFO re-render when bw/step/mode-dependent params change */
  SDR_UI_DrawVFO(ui);
  return;
#else
  (void)ui;  /* landscape/old portrait: no right sidebar */
  return;
#endif /* STS_H */
#endif /* LCD_PANEL_ST7789 */

  /* Cache guard — avoid rebuild when nothing changed */
  if (s_sbr_cache.valid
      && s_sbr_cache.bw_hz    == ui->bw_hz
      && s_sbr_cache.step     == ui->step
      && s_sbr_cache.mic_gain == ui->mic_gain
      && s_sbr_cache.rit_hz   == ui->rit_hz
      && s_sbr_cache.mode     == ui->mode) return;

  s_sbr_cache.bw_hz    = ui->bw_hz;
  s_sbr_cache.step     = ui->step;
  s_sbr_cache.mic_gain = ui->mic_gain;
  s_sbr_cache.rit_hz   = ui->rit_hz;
  s_sbr_cache.mode     = ui->mode;
  s_sbr_cache.valid    = true;

  /* Format strings */
  char bw_str[10];
  if (ui->bw_hz >= 10000U) {
    snprintf(bw_str, sizeof(bw_str), "%luK", (unsigned long)(ui->bw_hz / 1000U));
  } else if (ui->bw_hz >= 1000U) {
    uint32_t frac = (ui->bw_hz % 1000U) / 100U;
    if (frac)
      snprintf(bw_str, sizeof(bw_str), "%lu.%luK",
               (unsigned long)(ui->bw_hz / 1000U), (unsigned long)frac);
    else
      snprintf(bw_str, sizeof(bw_str), "%luK", (unsigned long)(ui->bw_hz / 1000U));
  } else {
    snprintf(bw_str, sizeof(bw_str), "%lu", (unsigned long)ui->bw_hz);
  }

  char step_str[8];
  uint32_t st = ui->step;
  if      (st >= 100000U) snprintf(step_str, sizeof(step_str), "100K");
  else if (st >=  10000U) snprintf(step_str, sizeof(step_str), "10K");
  else if (st >=   1000U) snprintf(step_str, sizeof(step_str), "1K");
  else if (st >=    100U) snprintf(step_str, sizeof(step_str), "100");
  else if (st >=     10U) snprintf(step_str, sizeof(step_str), "10");
  else                    snprintf(step_str, sizeof(step_str), "1");

  /* Row 1 left: MIC for voice modes, DG for digital modes.
   * csdr_app passes digi_gain (not mic_gain) in ui->mic_gain when in DIGU/DIGL. */
  bool digi_mode = (ui->mode == (uint8_t)5U || ui->mode == (uint8_t)6U); /* DIGU=5, DIGL=6 */
  char mic_str[8]; snprintf(mic_str, sizeof(mic_str), "%d", (int)ui->mic_gain);
  uint16_t    mic_vc  = digi_mode ? UI_STATUS_ON : UI_STATUS_VAL;

  buf_fill(s_sbr_buf, (uint32_t)SBR_H * SBR_W, UI_SBR_BG);

  /* 4 items × 24 rows = 96 rows (SBR_H), mirroring SBL layout.
   * All text in MED (12×16): label left, value right — same row. */
  const uint16_t item_h  = 24U;
  const uint16_t val_off =  4U;

  /* RIT: k-suffix for |rit_hz| >= 1000 keeps string ≤ 4 chars ("+9k").
   * Budget (SBR_W=88): label(36)+"+ 500"(48)=84px exactly ✓           */
  char rit_str[8];
  if (ui->rit_hz == 0) {
    snprintf(rit_str, sizeof(rit_str), "0");
  } else {
    int rv = (int)ui->rit_hz;
    int av = (rv < 0) ? -rv : rv;
    if (av >= 1000)
      snprintf(rit_str, sizeof(rit_str), "%+dK", rv / 1000);
    else
      snprintf(rit_str, sizeof(rit_str), "%+d", rv);
  }
  uint16_t rit_vc = (ui->rit_hz != 0) ? UI_STATUS_ON : UI_STATUS_LBL;

  /* SBR_W=88 → budget 84px: "STP"(36)+"100K"(48)=84 ✓  "RIT"(36)+"+500"(48)=84 ✓ */
  const char *mic_lbl_med = digi_mode ? "DG" : "MIC";
  struct { const char *lbl; const char *val; uint16_t vc; } items[4] = {
    { "BW",        bw_str,   UI_FREQ_KHZ },
    { "STP",       step_str, UI_FREQ_KHZ },
    { mic_lbl_med, mic_str,  mic_vc      },
    { "RIT",       rit_str,  rit_vc      },
  };

  for (uint8_t i = 0; i < 4U; i++) {
    uint16_t y0    = (uint16_t)(i * item_h);
    uint16_t val_x = (uint16_t)(SBR_W - 2U - med_str_w(items[i].val));

    for (uint16_t row = 0; row < item_h; row++) {
      uint16_t abs_r = y0 + row;
      if (abs_r >= SBR_H) break;
      uint16_t *ln = s_sbr_buf + (uint32_t)abs_r * SBR_W;

      if (i > 0U && row == 0U) { LCD_LineFill(ln, 0, SBR_W, UI_DIVIDER); continue; }

      if (row >= val_off && row < val_off + MED_H) {
        uint16_t fr = row - val_off;
        ln_medstr(ln, 2U,    fr, items[i].lbl, UI_STATUS_LBL,  UI_SBR_BG);
        ln_medstr(ln, val_x, fr, items[i].val, items[i].vc,   UI_SBR_BG);
      }
    }
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
 *  ST7796  (64 px): 7-seg digits rows 2..25, gap, divider row 33,
 *                   sub-line rows 41..50 (Font8x10).  VFO_SPLIT=28.
 *  ST7789 landscape (44 px): digits rows 2..25, sub-line rows 34..43.
 *  ST7789 portrait  (72 px): digits rows 2..25; right panel (x=192..239):
 *                   mode rows 2..17, RX/TX rows 20..35; divider row 38;
 *                   params row A rows 40..47, params row B rows 49..56;
 *                   sub-line rows 60..69.  VFO_SPLIT=36.
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
  static const char *const vfo_mode_s[] = {"AM","FM","USB","LSB","CW","DIGU","DIGL","FDV"};
  const char *vfo_mode_str = (ui->mode < 8U) ? vfo_mode_s[ui->mode] : "---";
  const char *rt_str       = ui->tx_mode ? "TX" : "RX";
  uint16_t    rt_color     = ui->tx_mode ? UI_TX_BG : UI_RX_BG;

  /* PW badge (ST7796 only — no vertical room on ST7789) */
#if LCD_PANEL != LCD_PANEL_ST7789
  char pw_str[8] = "";
  if (ui->tx_mode) {
    /* Measured fwd power (tandem match) when a PA is fitted; falls back to
     * the commanded setpoint while the closed-loop envelope is still settling
     * (fwd_power_mw reads 0 for the first ~100 ms of TX) or with no PA fitted. */
    uint16_t actual_w = (ui->pa_watts > 0U && ui->fwd_power_mw > 0U)
      ? (uint16_t)((ui->fwd_power_mw + 500U) / 1000U)
      : (ui->pa_watts > 0U)
        ? (uint16_t)((uint32_t)ui->pa_watts * ui->tx_power / 100U)
        : (uint16_t)ui->tx_power;
    snprintf(pw_str, sizeof(pw_str), "%dW", (int)actual_w);
  }
#endif

  buf_fill(s_vfo_buf, (uint32_t)VFO_H * VFO_W, UI_VFO_BG);

  /* ── ST7789 landscape: pre-compute right-panel strings + colors ──────────── */
#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
  char rp_step_s[12], rp_vol_s[10], rp_bw_s[14], rp_sql_s[10];
  {
    uint32_t st = ui->step;
    if      (st >= 100000U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:100k");
    else if (st >=  10000U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:10k");
    else if (st >=   1000U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:1k");
    else if (st >=    100U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:100");
    else if (st >=     10U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:10");
    else                    snprintf(rp_step_s, sizeof(rp_step_s), "STP:1");

    snprintf(rp_vol_s,  sizeof(rp_vol_s),  "VOL:%u",  ui->volume);
    snprintf(rp_sql_s,  sizeof(rp_sql_s),  "SQL:%u",  ui->squelch);

    if (ui->bw_hz >= 10000U)
      snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%luk", (unsigned long)(ui->bw_hz / 1000U));
    else if (ui->bw_hz >= 1000U) {
      uint32_t bk = ui->bw_hz / 1000U;
      uint32_t bf = (ui->bw_hz % 1000U) / 100U;
      if (bf) snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%lu.%luk", (unsigned long)bk, (unsigned long)bf);
      else    snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%luk", (unsigned long)bk);
    } else
      snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%lu", (unsigned long)ui->bw_hz);
  }
  static const uint16_t rp_mode_col_ls[8] = {
    UI_MODE_AM, UI_MODE_FM, UI_MODE_USB, UI_MODE_LSB,
    UI_MODE_CW, UI_MODE_DIGU, UI_MODE_DIGL, UI_MODE_FREEDV
  };
  uint16_t rp_mc = (ui->mode < 8U) ? rp_mode_col_ls[ui->mode] : UI_STATUS_LBL;
  uint16_t rp_nb = ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF;
  uint16_t rp_nr = ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF;
#endif /* ST7789 landscape pre-compute */

  /* ── ST7789 portrait: pre-compute right-panel strings + colors ───────────── */
#if LCD_PANEL == LCD_PANEL_ST7789 && !(LCD_W > LCD_H)
  char rp_step_s[12], rp_vol_s[10], rp_bw_s[14], rp_sql_s[10];
  {
    uint32_t st = ui->step;
    if      (st >= 100000U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:100k");
    else if (st >=  10000U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:10k");
    else if (st >=   1000U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:1k");
    else if (st >=    100U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:100");
    else if (st >=     10U) snprintf(rp_step_s, sizeof(rp_step_s), "STP:10");
    else                    snprintf(rp_step_s, sizeof(rp_step_s), "STP:1");

    snprintf(rp_vol_s,  sizeof(rp_vol_s),  "VOL:%u",  ui->volume);
    snprintf(rp_sql_s,  sizeof(rp_sql_s),  "SQL:%u",  ui->squelch);

    if (ui->bw_hz >= 10000U)
      snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%luk", (unsigned long)(ui->bw_hz / 1000U));
    else if (ui->bw_hz >= 1000U) {
      uint32_t bk = ui->bw_hz / 1000U;
      uint32_t bf = (ui->bw_hz % 1000U) / 100U;
      if (bf) snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%lu.%luk", (unsigned long)bk, (unsigned long)bf);
      else    snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%luk", (unsigned long)bk);
    } else
      snprintf(rp_bw_s, sizeof(rp_bw_s), "BW:%lu", (unsigned long)ui->bw_hz);
  }
  static const uint16_t rp_mode_col[8] = {
    UI_MODE_AM, UI_MODE_FM, UI_MODE_USB, UI_MODE_LSB,
    UI_MODE_CW, UI_MODE_DIGU, UI_MODE_DIGL, UI_MODE_FREEDV
  };
  uint16_t rp_mc = (ui->mode < 8U) ? rp_mode_col[ui->mode] : UI_STATUS_LBL;
  uint16_t rp_nb = ui->nb_on ? UI_STATUS_ON : UI_STATUS_OFF;
  uint16_t rp_nr = ui->nr_on ? UI_STATUS_ON : UI_STATUS_OFF;
#endif /* ST7789 portrait pre-compute */

  const uint16_t freq_top = 2U;
  const uint16_t vfoi_y   = 1U;

  /* Gap between primary and secondary VFO.  Secondary uses MED (12x16 px). */
#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
  /* Landscape 320x240: sub-VFO below main digits (rows 34..49) */
  const uint16_t sub_y  = 34U;
  const uint16_t div_y  = 0xFFFFU;
#elif LCD_PANEL == LCD_PANEL_ST7789
  /* Portrait 240x320: sub-VFO rows 28..43 (MED 12x16); divider row 38 within it;
   * params row A at 46 → 2-row gap after sub-VFO; no overlap. */
  const uint16_t sub_y  = 28U;
  const uint16_t div_y  = 0xFFFFU;
#else
  const uint16_t sub_y  = (uint16_t)(freq_top + BIG_H + 10U);  /* row 36: below divider */
  const uint16_t div_y  = (uint16_t)(freq_top + BIG_H + 7U);   /* row 33, centred in gap */
#endif
  /* RX/TX text-only badge (ST7796 only) — ST7789 uses right panel instead.
   * rt_by chosen so text rows (rt_by+2 .. rt_by+2+MED_H) coincide with
   * mode_y (= freq_top + (BIG_H-MED_H)/2), giving side-by-side alignment. */
#if LCD_PANEL != LCD_PANEL_ST7789
  const uint16_t rt_bad_w = (uint16_t)(2U * MED_W + 6U);
  const uint16_t rt_bad_h = (uint16_t)(MED_H + 4U);
  uint16_t rt_bx    = (uint16_t)(VFO_W - 22U - rt_bad_w);
  const uint16_t rt_by    = (uint16_t)(freq_top + (BIG_H - MED_H) / 2U - 2U);
#else
  const uint16_t rt_bad_h = (uint16_t)(MED_H + 4U);
  const uint16_t rt_bx    = VFO_W;   /* out-of-range: disables legacy badge */
  const uint16_t rt_by    = VFO_H;
#endif

  /* Frequency centering: each digit = BIG_W px, each '.' = 6 px */
  uint16_t total_w = 0U;
  for (const char *p = full_freq; *p; p++)
    total_w += (*p == '.') ? 6U : BIG_W;
  /* ST7789: right portion reserved for mode/RT panel — center digits in left area only */
#if LCD_PANEL == LCD_PANEL_ST7789
  uint16_t dig_area = VFO_RIGHT_X;
#else
  uint16_t dig_area = VFO_W;
#endif
  uint16_t fx_base = (dig_area > total_w) ? (uint16_t)((dig_area - total_w) / 2U) : 2U;
#if LCD_PANEL == LCD_PANEL_ST7796
  /* Right-align digits: units column always at x=180 (= xx.xxx.xxx right edge) */
  const uint16_t vfo_right_edge = 2U + MED_W + 10U + 8U * BIG_W + 2U * 6U;  /* 180 */
  fx_base = (uint16_t)(vfo_right_edge - total_w);
  const uint16_t mode_x = (uint16_t)(vfo_right_edge + 10U);  /* 190, fixed */
  const uint16_t mode_y = (uint16_t)(freq_top + (BIG_H - MED_H) / 2U);
  static const uint16_t s_mode_col[8] = {
      UI_MODE_AM, UI_MODE_FM,  UI_MODE_USB, UI_MODE_LSB,
      UI_MODE_CW, UI_MODE_DIGU, UI_MODE_DIGL, UI_MODE_FREEDV
  };
  uint16_t mode_color = (ui->mode < 8U) ? s_mode_col[ui->mode] : UI_STATUS_LBL;
  rt_bx = (uint16_t)(mode_x + (uint16_t)(strlen(vfo_mode_str) * MED_W) + 12U);
#else
  fx_base = (fx_base >= 14U) ? (uint16_t)(fx_base - 14U) : 0U;
#endif

  /* VFO_SPLIT defined here (before the row loop) so pw_by can reference it.
   * PW badge starts exactly at the split → always lands in the lower section,
   * so lower_chg alone keeps it current without needing an upper-section push. */
#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
  const uint16_t VFO_SPLIT = 27U;
#elif LCD_PANEL == LCD_PANEL_ST7789
  const uint16_t VFO_SPLIT = 26U;
#else
  const uint16_t VFO_SPLIT = 28U;
#endif

  for (uint16_t row = 0; row < VFO_H; row++) {
    uint16_t *ln = s_vfo_buf + (uint32_t)row * VFO_W;

    /* 7-segment frequency — single color */
    if (row >= freq_top && (row - freq_top) < BIG_H) {
      ln_segstr(ln, fx_base, row - freq_top, full_freq, UI_FREQ_FG, UI_VFO_BG);
    }

#if LCD_PANEL == LCD_PANEL_ST7796
    if (row >= mode_y && (row - mode_y) < MED_H)
      ln_medstr(ln, mode_x, row - mode_y, vfo_mode_str, mode_color, UI_VFO_BG);
#endif

    /* Active VFO indicator (2× medium, top-left) */
    if (row >= vfoi_y && (row - vfoi_y) < MED_H) {
      const char *vl = (ui->active_vfo == 0U) ? "A" : "B";
      uint16_t    vc = (ui->active_vfo == 0U) ? UI_STATUS_VAL : UI_STATUS_ON;
      ln_medchar(ln, 2U, row - vfoi_y, *vl, vc, UI_VFO_BG);
    }

    /* Sub-line: secondary VFO freq | RIT offset — MED (12×16) for visibility */
    if (ui->freq_b_hz > 0U) {
      if (row >= sub_y && (row - sub_y) < MED_H)
        ln_medstr(ln, 4U, row - sub_y, sub_str, UI_FREQ_SUB, UI_VFO_BG);
    } else if (ui->rit_hz != 0) {
      if (row >= sub_y && (row - sub_y) < MED_H)
        ln_medstr(ln, 4U, row - sub_y, sub_str, UI_FREQ_SUB, UI_VFO_BG);
    }

    /* RX/TX: colored text only, transparent background */
    if (row >= rt_by && row < rt_by + rt_bad_h) {
      uint16_t br = row - rt_by;
      if (br >= 2U && br < 2U + MED_H)
        ln_medstr(ln, (uint16_t)(rt_bx + 3U), br - 2U, rt_str, rt_color, UI_VFO_BG);
    }

#if LCD_PANEL != LCD_PANEL_ST7789
    /* Watts readout — MED font, right-aligned under TX badge, TX mode only.
     * pw_by = VFO_SPLIT so the wattage text lives entirely in the lower section;
     * lower_chg triggers on tx_power/pa_watts changes → no stale upper rows. */
    if (ui->tx_mode && pw_str[0] != '\0') {
      const uint16_t pw_by = VFO_SPLIT;
      if (row >= pw_by && (row - pw_by) < MED_H) {
        uint16_t med_row = row - pw_by;
        uint16_t pw_len  = (uint16_t)(strlen(pw_str) * MED_W);
        uint16_t pw_x    = (uint16_t)(rt_bx + rt_bad_w - pw_len);
        ln_medstr(ln, pw_x, med_row, pw_str, UI_TX_BG, UI_VFO_BG);
      }
    }
#endif

    (void)div_y;

#if LCD_PANEL == LCD_PANEL_ST7789 && !(LCD_W > LCD_H)
    /* ── ST7789 portrait: right panel (x=VFO_RIGHT_X..239) ──────────────── */
    /* Mode label rows 2..17 */
    if (row >= 2U && (row - 2U) < MED_H) {
      ln_medstr(ln, (uint16_t)(VFO_RIGHT_X + 4U), row - 2U,
                vfo_mode_str, rp_mc, UI_VFO_BG);
    }
    /* RX/TX label rows 20..35 */
    if (row >= 20U && (row - 20U) < MED_H) {
      ln_medstr(ln, (uint16_t)(VFO_RIGHT_X + 4U), row - 20U,
                rt_str, rt_color, UI_VFO_BG);
    }
    /* Full-width horizontal divider at row 38 */
    if (row == 38U) {
      for (uint16_t x = 0U; x < VFO_W; x++)
        ln[x] = SWAP16(UI_DIVIDER);
    }
    /* Params row A: STP / VOL / NB badge */
    if (row >= VFO_PARAMS_Y && row < VFO_PARAMS_Y + Font5x8.height) {
      uint16_t fr = row - VFO_PARAMS_Y;
      LCD_LineStr(ln,  4U, fr, rp_step_s, &Font5x8, UI_FREQ_KHZ,   UI_VFO_BG);
      LCD_LineStr(ln, 64U, fr, rp_vol_s,  &Font5x8, UI_STATUS_VAL, UI_VFO_BG);
      LCD_LineFill(ln, (uint16_t)(LCD_W - 20U), 16U, rp_nb);
      LCD_LineStr(ln,  (uint16_t)(LCD_W - 17U), fr, "NB", &Font5x8, UI_BG, rp_nb);
    }
    /* Params row B: BW / SQL / NR badge */
    if (row >= VFO_PARAMS_B_Y && row < VFO_PARAMS_B_Y + Font5x8.height) {
      uint16_t fr = row - VFO_PARAMS_B_Y;
      LCD_LineStr(ln,  4U, fr, rp_bw_s,  &Font5x8, UI_FREQ_KHZ,   UI_VFO_BG);
      LCD_LineStr(ln, 64U, fr, rp_sql_s, &Font5x8, UI_STATUS_VAL, UI_VFO_BG);
      LCD_LineFill(ln, (uint16_t)(LCD_W - 20U), 16U, rp_nr);
      LCD_LineStr(ln,  (uint16_t)(LCD_W - 17U), fr, "NR", &Font5x8, UI_BG, rp_nr);
    }
#endif /* ST7789 portrait row rendering */

#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
    /* ── ST7789 landscape: 4-row right panel, evenly distributed ───────────
     * Row 1 rows  1..16  MED_H   : Mode  |  RX/TX
     * Row 2 rows 19..28  Font8x10: STP   |  BW
     * Row 3 rows 31..40  Font8x10: VOL   |  SQL
     * Row 4 rows 43..52  Font8x10: NB    |  NR  (colored: green=ON red=OFF)
     * ─────────────────────────────────────────────────────────────────────── */
    {
      uint16_t bg16 = SWAP16(UI_VFO_BG);
      for (uint16_t x = VFO_RIGHT_X; x < VFO_W; x++) ln[x] = bg16;
    }
    /* Row 1 — Mode (MED_H) + RX/TX (MED_H) */
    if (row >= 1U && (row - 1U) < MED_H) {
      uint16_t fr = row - 1U;
      ln_medstr(ln, (uint16_t)(VFO_RIGHT_X + 3U),  fr, vfo_mode_str, rp_mc,    UI_VFO_BG);
      ln_medstr(ln, (uint16_t)(VFO_RIGHT_X + 57U), fr, rt_str,       rt_color, UI_VFO_BG);
    }
    /* Row 2 — STP (Font8x10) + BW (Font8x10) */
    if (row >= 19U && (row - 19U) < (uint16_t)Font8x10.height) {
      uint16_t fr = row - 19U;
      LCD_LineStrW(ln, (uint16_t)(VFO_RIGHT_X + 3U),  fr, rp_step_s, &Font8x10, UI_FREQ_KHZ, UI_VFO_BG);
      LCD_LineStrW(ln, (uint16_t)(VFO_RIGHT_X + 75U), fr, rp_bw_s,   &Font8x10, UI_FREQ_KHZ, UI_VFO_BG);
    }
    /* Row 3 — VOL (Font8x10) + SQL (Font8x10) */
    if (row >= 31U && (row - 31U) < (uint16_t)Font8x10.height) {
      uint16_t fr = row - 31U;
      LCD_LineStrW(ln, (uint16_t)(VFO_RIGHT_X + 3U),  fr, rp_vol_s,  &Font8x10, UI_STATUS_VAL, UI_VFO_BG);
      LCD_LineStrW(ln, (uint16_t)(VFO_RIGHT_X + 75U), fr, rp_sql_s,  &Font8x10, UI_STATUS_VAL, UI_VFO_BG);
    }
    /* Row 4 — NB + NR: small badge (Font8x10 height), colored fill + black text */
    if (row >= 43U && (row - 43U) < (uint16_t)Font8x10.height) {
      uint16_t fr = row - 43U;
      LCD_LineFill(ln, (uint16_t)(VFO_RIGHT_X +  3U), 20U, rp_nb);
      LCD_LineStrW(ln, (uint16_t)(VFO_RIGHT_X +  5U), fr, "NB", &Font8x10, UI_BG, rp_nb);
      LCD_LineFill(ln, (uint16_t)(VFO_RIGHT_X + 75U), 20U, rp_nr);
      LCD_LineStrW(ln, (uint16_t)(VFO_RIGHT_X + 77U), fr, "NR", &Font8x10, UI_BG, rp_nr);
    }
#endif /* ST7789 landscape row rendering */
  }

  /* Right-panel changed: forces redraw of both sections on ST7789.
   * Landscape: rp_chg propagates to lower_chg so params (rows 19..58) all refresh.
   * Portrait: params are in lower section → only mode/tx_mode need upper push.
   * ST7796: mode text and RT badge ("TX"/"RX") both sit in the upper section;
   *         add tx_mode so the upper section is pushed on every TX key/unkey. */
  bool rp_chg = false;
#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
  rp_chg = !s_vfo_cache.valid
          || s_vfo_cache.mode    != ui->mode
          || s_vfo_cache.tx_mode != ui->tx_mode
          || s_vfo_cache.volume  != ui->volume
          || s_vfo_cache.squelch != ui->squelch
          || s_vfo_cache.step    != ui->step
          || s_vfo_cache.bw_hz   != ui->bw_hz
          || s_vfo_cache.nb_on   != ui->nb_on
          || s_vfo_cache.nr_on   != ui->nr_on;
#elif LCD_PANEL == LCD_PANEL_ST7789
  rp_chg = !s_vfo_cache.valid
          || s_vfo_cache.mode    != ui->mode
          || s_vfo_cache.tx_mode != ui->tx_mode;
#else /* ST7796 */
  rp_chg = !s_vfo_cache.valid
          || s_vfo_cache.mode    != ui->mode
          || s_vfo_cache.tx_mode != ui->tx_mode;  /* RT badge rows 6..21 in upper section */
#endif

  bool upper_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_hz    != ui->freq_hz
      || s_vfo_cache.active_vfo != ui->active_vfo
#if LCD_PANEL == LCD_PANEL_ST7796
      || s_vfo_cache.mode       != ui->mode
#endif
      || rp_chg;

  bool lower_chg = !s_vfo_cache.valid
      || s_vfo_cache.freq_b_hz  != ui->freq_b_hz
      || s_vfo_cache.rit_hz     != ui->rit_hz
      || s_vfo_cache.tx_mode    != ui->tx_mode
      || s_vfo_cache.active_vfo != ui->active_vfo
      || s_vfo_cache.tx_power   != ui->tx_power
      || s_vfo_cache.pa_watts   != ui->pa_watts
      || s_vfo_cache.fwd_power_mw != ui->fwd_power_mw
#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
      /* Landscape: NB/NR (row 4, rows 43..58) are in lower section → push on any rp change */
      || rp_chg
#elif LCD_PANEL == LCD_PANEL_ST7789
      /* Portrait: params rows are in lower section */
      || s_vfo_cache.mode    != ui->mode
      || s_vfo_cache.volume  != ui->volume
      || s_vfo_cache.squelch != ui->squelch
      || s_vfo_cache.step    != ui->step
      || s_vfo_cache.bw_hz   != ui->bw_hz
      || s_vfo_cache.nb_on   != ui->nb_on
      || s_vfo_cache.nr_on   != ui->nr_on
#endif
      ;

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
  s_vfo_cache.fwd_power_mw = ui->fwd_power_mw;
#if LCD_PANEL == LCD_PANEL_ST7796
  s_vfo_cache.mode    = ui->mode;
#endif
#if LCD_PANEL == LCD_PANEL_ST7789
  s_vfo_cache.mode    = ui->mode;
  s_vfo_cache.volume  = ui->volume;
  s_vfo_cache.squelch = ui->squelch;
  s_vfo_cache.step    = ui->step;
  s_vfo_cache.bw_hz   = ui->bw_hz;
  s_vfo_cache.nb_on   = ui->nb_on;
  s_vfo_cache.nr_on   = ui->nr_on;
#endif
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
    if (mhz_g || rp_chg) {
      /* MHz changed, first draw, or right-panel changed → push full upper section */
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

/* RSSI label geometry — panel-specific.
 *
 * ST7796 (MTR_W=320): RSSI sits RIGHT of S-value.
 *   S-value at val_x=222, clears [222..269].
 *   RSSI at [272..307] (2px gap); max "-120dB"=6×6=36px = RSSI_CLR_W.
 *   Clamped to [-120,0] so text never overruns.
 *
 * ST7789 landscape (MTR_W=320): matches ST7796 style.
 *   S-value at val_x=162 (Font8x10, SM_VAL_CLR_W=48px), RSSI right after.
 *   RSSI at 212 (val_x+SM_VAL_CLR_W+2); RSSI_CLR_W=48; max "-120dB"=6ch×6px=36px.
 *
 * ST7789 portrait (MTR_W=240): RSSI REPLACES S-value text at val_x=222.
 *   Available [222..239] = 18px = 3 chars × Font5x8.width(6).
 *   Clamped to [-99,0]; "-99"=3×6=18px fits exactly.
 *   mk_col used so colour still tracks signal level. */
#if LCD_PANEL == LCD_PANEL_ST7796
#  define RSSI_X       272U
#  define RSSI_CLR_W    36U
#  define RSSI_DB_MIN  (-99)   /* max "-99dBm"=6ch×6px=36px fits RSSI_CLR_W exactly */
#  define RSSI_FMT    "%ddBm"
#elif LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
   /* Landscape 320×20: S-value at val_x=162 (Font8x10), RSSI right after with "dBm" unit.
    * RSSI_X = ruler_end(158)+4+SM_VAL_CLR_W(48)+2 = 212; "-120dBm"=7ch×6px=42px < CLR_W=48. */
#  define RSSI_X       (SM_START_X + SM_RULER_W + 4U + SM_VAL_CLR_W + 2U)
#  define RSSI_CLR_W    48U
#  define RSSI_DB_MIN (-120)
#  define RSSI_FMT    "%ddBm"
#else
#  define RSSI_X       222U   /* = val_x (portrait 240px) */
#  define RSSI_CLR_W    18U
#  define RSSI_DB_MIN  (-99)
#  define RSSI_FMT    "%d"
#endif

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
        uint16_t lx;
#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
        /* Landscape: last label "40" left-aligned at tick — avoids overlap with "20" */
        lx = (t == 7U) ? maj_tx[t] : ((maj_tx[t] >= half_w) ? (maj_tx[t] - half_w) : 0U);
#else
        lx = (maj_tx[t] >= half_w) ? (maj_tx[t] - half_w) : 0U;
#endif
        LCD_LineStrW(ln, lx, fr, lbl_str[t], &Font8x10, UI_SMETER_TICK, UI_MTR_BG);
      }
#if LCD_PANEL == LCD_PANEL_ST7796 || (LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H)
      /* ST7796 + ST7789 landscape: S-value to the left of RSSI */
      if (val_x < MTR_W)
        LCD_LineStrW(ln, val_x, fr, s_str, &Font8x10, mk_col, UI_MTR_BG);
#endif
      /* RSSI: ST7796 shows it in INFO zone; other panels show inline here */
#if LCD_PANEL != LCD_PANEL_ST7796
      if (fr < Font5x8.height && s_rssi_db > RSSI_DB_MIN - 1) {
        char rbuf[8];
        int rv = (int)s_rssi_db;
        if (rv < RSSI_DB_MIN) rv = RSSI_DB_MIN;
        if (rv > 0)           rv = 0;
        snprintf(rbuf, sizeof(rbuf), RSSI_FMT, rv);
        LCD_LineStr(ln, RSSI_X, fr, rbuf, &Font5x8, mk_col, UI_MTR_BG);
      }
#endif
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
  s_rssi_db = (int16_t)ui->signal_db;
  draw_smeter_rows(bars);
  s_mtr_static_valid = true;
  LCD_PushWindow(MTR_X, MTR_Y,
                 (uint16_t)(MTR_X + MTR_W - 1U), MTR_Y2 - 1U,
                 s_mtr_buf, (uint32_t)MTR_W * MTR_H);
}

#if LCD_PANEL == LCD_PANEL_ST7796
static void rssi_info_draw(void);  /* defined near CW text section below */
#endif

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
  s_rssi_db = -200;
  s_mtr_static_valid = false;
}
void SDR_UI_UpdateSMeter_SetVoltage(int16_t v_x10) { s_smeter_voltage_x10 = v_x10; }

void SDR_UI_UpdateSMeter(float signal_db)
{
  int32_t bars = (int32_t)((signal_db + 73.0f) / 3.0f);
  if (bars < 0) bars = 0;
  if (bars > (int32_t)SM_BARS) bars = (int32_t)SM_BARS;
  int32_t old_bars = s_rx_meter_bars;

  int16_t rssi_db = (int16_t)signal_db;
  bool rssi_changed = (rssi_db != s_rssi_db);
  bool bars_changed = (bars != old_bars);
  if (!bars_changed && !rssi_changed) return;

  s_rx_meter_bars = bars;
  s_rssi_db = rssi_db;

#if LCD_PANEL == LCD_PANEL_ST7796
  if (rssi_changed) rssi_info_draw();
#endif

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
  if (bars_changed) {
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

  /* 2. Update S-value text and/or RSSI in label row band */
  {
    char s_str[8];
    if (bars <= 9) snprintf(s_str, sizeof(s_str), "S%ld", (long)bars);
    else           snprintf(s_str, sizeof(s_str), "+%ld",  (long)((bars - 9) * 3));
    char rssi_str[8];
    { int rv = (int)s_rssi_db;
      if (rv < RSSI_DB_MIN) rv = RSSI_DB_MIN;
      if (rv > 0)           rv = 0;
      snprintf(rssi_str, sizeof(rssi_str), RSSI_FMT, rv); }
    uint16_t row_top = SM_LBL_R0;
    uint16_t row_end = (uint16_t)(SM_LBL_R0 + (uint16_t)Font8x10.height);
    if (row_end > MTR_H) row_end = MTR_H;
    for (uint16_t row = row_top; row < row_end; row++) {
      uint16_t *ln = s_mtr_buf + (uint32_t)row * MTR_W;
      uint16_t  fr = row - row_top;
#if LCD_PANEL == LCD_PANEL_ST7796 || (LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H)
      /* ST7796: S-value only (RSSI moved to INFO zone). ST7789 landscape: S-value + RSSI. */
      if (bars_changed) {
        for (uint16_t x = val_x; x < val_x + SM_VAL_CLR_W && x < MTR_W; x++)
          ln[x] = SWAP16(UI_MTR_BG);
        if (val_x < MTR_W)
          LCD_LineStrW(ln, val_x, fr, s_str, &Font8x10, mk_col, UI_MTR_BG);
      }
#if LCD_PANEL != LCD_PANEL_ST7796
      if (rssi_changed) {
        for (uint16_t x = RSSI_X; x < (uint16_t)(RSSI_X + RSSI_CLR_W) && x < MTR_W; x++)
          ln[x] = SWAP16(UI_MTR_BG);
        if (fr < Font5x8.height)
          LCD_LineStr(ln, RSSI_X, fr, rssi_str, &Font5x8, UI_SMETER_TICK, UI_MTR_BG);
      }
#endif
#else
      /* ST7789 portrait: RSSI at val_x replaces S-value */
      if (rssi_changed) {
        for (uint16_t x = RSSI_X; x < (uint16_t)(RSSI_X + RSSI_CLR_W) && x < MTR_W; x++)
          ln[x] = SWAP16(UI_MTR_BG);
        if (fr < Font5x8.height)
          LCD_LineStr(ln, RSSI_X, fr, rssi_str, &Font5x8, mk_col, UI_MTR_BG);
      }
#endif
    }
  }

  /* 3. Two targeted pushes — only dynamic rows retransmitted */
  /* Push A: signal line (SM_LINE_H rows) — only when bars moved */
  if (bars_changed) {
    uint16_t r0 = SM_LINE_R0;
    uint16_t r1 = (uint16_t)(SM_LINE_R0 + SM_LINE_H - 1U);
    if (r1 >= MTR_H) r1 = MTR_H - 1U;
    LCD_PushWindow(MTR_X, (uint16_t)(MTR_Y + r0),
                   (uint16_t)(MTR_X + MTR_W - 1U), (uint16_t)(MTR_Y + r1),
                   s_mtr_buf + (uint32_t)r0 * MTR_W,
                   (uint32_t)MTR_W * (r1 - r0 + 1U));
  }
  /* Push B: label rows.
   * ST7796 + ST7789 landscape: push when bars or RSSI changed (both in label band).
   * ST7789 portrait: push only when RSSI changed (no separate S-value text). */
#if LCD_PANEL == LCD_PANEL_ST7789 && !(LCD_W > LCD_H)
  if (rssi_changed)
#endif
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
/* TX_SWR_R0 (ST7796 only): SWR text placed in ruler right column (x=ruler_end+4=222),
 * rows TX_SWR_R0..TX_SWR_R0+Font8x10.height-1 = 12..21.
 * Ruler spans x=2..217; x=222+ is background in every ruler row — no overlap.
 * 10/10 rows visible, zero clip. */
#if LCD_PANEL == LCD_PANEL_ST7796
#  define TX_SWR_R0  12U
#else
/* ST7789: SWR is suppressed (MTR too short). TX_VAL_R0 sets static_end only. */
#  define TX_VAL_R0  ((uint16_t)(SM_RAIL_BOT_R + 2U))
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

#if LCD_PANEL == LCD_PANEL_ST7796
    /* SWR text: right-column overlay (x=ruler_end+4=222), rows TX_SWR_R0..TX_SWR_R0+9.
     * Ruler content (ticks/rails/bar) ends at x=217; x=222+ is always background. */
    if (row >= (uint16_t)TX_SWR_R0 && (row - (uint16_t)TX_SWR_R0) < (uint16_t)Font8x10.height) {
      uint16_t fr  = row - (uint16_t)TX_SWR_R0;
      uint16_t vx  = (uint16_t)(ruler_end + 4U);
      uint16_t svx = (uint16_t)(vx + 4U * (uint16_t)Font8x10.width);
      LCD_LineStrW(ln, vx,  fr, "SWR", &Font8x10, UI_STATUS_LBL, UI_MTR_BG);
      LCD_LineStrW(ln, svx, fr, swr_val, &Font8x10, swr_col, UI_MTR_BG);
    }
#endif
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

  /* Static content: label band (0..9), ticks (10..13), rails (14,22), SWR zone (12..21).
   * ST7796: static_end=SM_RAIL_BOT_R=22 covers rows 0..22, which includes SWR rows 12..21.
   * ST7789: static_end=TX_VAL_R0-1=19 (SWR suppressed). */
  if (first) {
#if LCD_PANEL == LCD_PANEL_ST7796
    uint16_t static_end = (uint16_t)SM_RAIL_BOT_R;   /* 22 */
#else
    uint16_t static_end = (TX_VAL_R0 - 1U < (uint16_t)MTR_H) ? (uint16_t)(TX_VAL_R0 - 1U) : (uint16_t)(MTR_H - 1U);
#endif
    tx_meter_render_rows(0U, static_end, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(0U, static_end);
  }

  /* Dynamic: cursor zone (SM_RAIL_TOP_R..SM_RAIL_BOT_R) — push when ALC bar moves.
   * On ST7796 this also re-renders SWR rows 14..21 (within the cursor zone range). */
  if (first || alc_b != s_tx_alc_bars) {
    uint16_t r0 = SM_RAIL_TOP_R;
    uint16_t r1 = SM_RAIL_BOT_R;
    if (r1 >= MTR_H) r1 = MTR_H - 1U;
    tx_meter_render_rows(r0, r1, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(r0, r1);
    s_tx_alc_bars = alc_b;
  }

  /* Dynamic: SWR text update.
   * ST7796: push TX_SWR_R0..TX_SWR_R0+9 (rows 12..21, full 10/10 rows, no clip).
   *         Rows 12..13 (above cursor zone) only updated here, not in cursor push.
   * ST7789: suppressed (zone too short for readable text). */
  if (first || swr_x10 != s_tx_swr_x10) {
#if LCD_PANEL == LCD_PANEL_ST7796
    uint16_t r0 = (uint16_t)TX_SWR_R0;
    uint16_t r1 = (uint16_t)(TX_SWR_R0 + (uint16_t)Font8x10.height - 1U);
    tx_meter_render_rows(r0, r1, alc_b, alc_pct, swr_x10);
    tx_meter_push_rows(r0, r1);
#endif
    s_tx_alc_pct = alc_pct;
    s_tx_swr_x10 = swr_x10;
  }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_UpdatePAWarn — persistent PA protection warning in the INFO zone.
 *
 *  ST7796: INFO zone (Y=120..144, 24 px) sits between the S-meter and spectrum.
 *  This zone is NOT touched by any dirty-flag redraw path, so the warning
 *  persists across TX→RX transitions, including PA TRIP.
 *
 *  When PA state is NORMAL the zone is cleared once and then left alone.
 *  When PA state is not NORMAL the zone is redrawn on every call so it
 *  survives SDR_UI_DrawCWText overwriting it.
 *
 *  ST7789 (INFO_H = 0): no-op — warning already appears in the left SPEC
 *  strip via pa_warn_render_push() inside DrawTXSpectrum.
 * ════════════════════════════════════════════════════════════════════════════ */
void SDR_UI_UpdatePAWarn(PA_State_t state, PA_Fault_t fault)
{
#if INFO_H > 0
  bool active     = (state != PA_STATE_NORMAL);
  bool was_active = (s_pa_warn_drawn != PA_STATE_NORMAL &&
                     s_pa_warn_drawn != (PA_State_t)0xFFU);

  /* No active warning and nothing to clear: nothing to do. */
  if (!active && !was_active) return;

  /* Active warning and no change: still redraw to survive DrawCWText overwrites. */

  s_pa_warn_drawn = state;
  s_pa_wflt_drawn = fault;

  const char *fault_s = (fault == PA_FAULT_OVERCURRENT) ? "! OVERCURRENT"
                      : (fault == PA_FAULT_OVERTEMP)    ? "! OVERTEMP"
                      : (fault == PA_FAULT_HIGH_SWR)    ? "! HIGH SWR"
                      :                                    "! PA PROTECT";
  const char *state_s = (state == PA_STATE_FOLDBACK)    ? "FOLDBACK"
                      : (state == PA_STATE_LIMIT)        ? "POWER LIMIT"
                      : (state == PA_STATE_TRIP)         ? "TX TRIPPED"
                      : (state == PA_STATE_COOLDOWN)     ? "COOLDOWN"
                      :                                    "";
  uint16_t fg = (state >= PA_STATE_TRIP) ? UI_STATUS_OFF : UI_STATUS_WARN;

  /* Two Font5x8 lines inside INFO_H (24 px on ST7796):
   *   line 1 — fault : rows 2..9
   *   line 2 — state : rows 11..18   (4 px gap at bottom)
   *
   * Only columns [0..SBR_X-1] are written — RSSI lives at [SBR_X..LCD_W-1]
   * and is managed independently by rssi_info_draw(); never overwrite it. */
  const uint16_t WARN_W = (uint16_t)SBR_X;   /* 392 px for ST7796 */
  const uint16_t L1 = 2U;
  const uint16_t L2 = 11U;
  const uint16_t FH = (uint16_t)Font5x8.height;  /* = 8 */

  for (uint16_t r = 0U; r < (uint16_t)INFO_H; r++) {
    uint16_t *ln = s_info_buf[r];
    LCD_LineFill(ln, 0U, WARN_W, UI_MTR_BG);
    if (active) {
      if (r >= L1 && r < L1 + FH)
        LCD_LineStr(ln, 8U, r - L1, fault_s, &Font5x8, fg, UI_MTR_BG);
      if (r >= L2 && r < L2 + FH)
        LCD_LineStr(ln, 8U, r - L2, state_s, &Font5x8, fg, UI_MTR_BG);
    }
    LCD_PushWindow(0U, (uint16_t)(INFO_Y + r),
                   (uint16_t)(WARN_W - 1U), (uint16_t)(INFO_Y + r),
                   ln, WARN_W);
  }
#else
  (void)state; (void)fault;
#endif
}

/* ── RTC clock wrappers ───────────────────────────────────────────────────── */
void SDR_UI_SetClock(uint8_t h, uint8_t m, uint8_t s)
{
  RTC_Clock_SetTime(h, m, s);
}

void SDR_UI_GetClock(uint8_t *h, uint8_t *m, uint8_t *s)
{
  RTC_Clock_GetTime(h, m, s);
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
 *  Xiegu-style passband shaded region + bright edge lines + center marker.
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
    uint16_t b_lo = (uint16_t)fbin;
    float    t    = fbin - (float)b_lo;
    if (b_lo >= bins) b_lo = (uint16_t)(bins - 1U);
    uint16_t b_hi = (uint16_t)(b_lo + 1U);
    if (b_hi >= bins) b_hi = b_lo;
    float val = (1.0f - t) * fft_db[b_lo] + t * fft_db[b_hi];
    s_spec_yf[x] = pwr_compress(val);
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

  /* zoom_corr scales the ratio (computed against ADC sample rate) to the
   * effective display span — without it the passband region stays fixed-width
   * when zooming in even though the displayed span narrows. */
  float zoom_corr = (s_spec_sr > 0U) ? ((float)s_spec_orig_sr / (float)s_spec_sr) : 1.0f;

  /* Track-mode marker: shift the carrier column off center.  Same Hz→px
   * mapping as the bw ratios below (vs ADC rate, zoom-corrected). */
  if (s_spec_marker_hz != 0 && s_spec_orig_sr > 0U) {
    float mpx_f = (float)s_spec_marker_hz / (float)s_spec_orig_sr
                * (float)SPEC_W * cscale * zoom_corr;
    int32_t mpx = (int32_t)(SPEC_W / 2U)
                + (int32_t)(mpx_f >= 0.0f ? mpx_f + 0.5f : mpx_f - 0.5f);
    if (mpx < 1) mpx = 1;
    if (mpx > (int32_t)(SPEC_W - 2U)) mpx = (int32_t)(SPEC_W - 2U);
    cx = (uint16_t)mpx;
  }

  /* Ratios are signed extents from the carrier column (bw_lo = LEFT extent,
   * bw_hi = RIGHT extent); a negative extent puts that edge on the opposite
   * side of the carrier, so an offset passband (CW: pitch ± bw/2) works. */
  bool bw_lo_ok = (bw_lo_ratio > 0.0001f) || (bw_lo_ratio < -0.0001f);
  bool bw_hi_ok = (bw_hi_ratio > 0.0001f) || (bw_hi_ratio < -0.0001f);
  int32_t pb_l = (int32_t)cx, pb_r = (int32_t)cx;
  {
    float scale = (float)SPEC_W * cscale * zoom_corr;
    float lo_f  = bw_lo_ratio * scale;
    float hi_f  = bw_hi_ratio * scale;
    if (bw_lo_ok)
      pb_l = (int32_t)cx - (int32_t)(lo_f >= 0.0f ? lo_f + 0.5f : lo_f - 0.5f);
    if (bw_hi_ok)
      pb_r = (int32_t)cx + (int32_t)(hi_f >= 0.0f ? hi_f + 0.5f : hi_f - 0.5f);
    if (pb_l < 0) pb_l = 0;
    if (pb_l > (int32_t)(SPEC_W - 1U)) pb_l = (int32_t)(SPEC_W - 1U);
    if (pb_r < 0) pb_r = 0;
    if (pb_r > (int32_t)(SPEC_W - 1U)) pb_r = (int32_t)(SPEC_W - 1U);
  }

  uint16_t spec_sw      = SWAP16(0xC7FFU);   /* icy white-blue: top     */
  uint16_t spec_fill_sw = SWAP16(0x3D7FU);   /* muted cold cyan: body   */
  uint16_t pb_sw        = SWAP16(UI_SPEC_PASS);
  uint16_t cx_sw        = SWAP16(0xFFFFU);   /* bright white centre pixel     */
  uint16_t dot_sw       = SWAP16(UI_SPEC_GRID);

  /* Passband shaded region: pb_l..pb_r as computed above.
   * USB: center→right | LSB: left→center | AM/FM: left→right
   * CW: offset span at pitch ± bw/2 (both edges may sit on one side). */
  uint16_t pb_x0 = (uint16_t)pb_l, pb_x1 = (uint16_t)pb_r;
  bool do_shade = (bw_lo_ok || bw_hi_ok) && (pb_r > pb_l);

  for (uint16_t y = 0U; y < (uint16_t)(SPEC_H - 1U); y++) {
    uint16_t *row    = s_spec_buf[y];
    bool      is_grid = (y == g1 || y == g2 || y == g3);
    uint16_t  bg_sw  = is_grid ? SWAP16(UI_SPEC_GRID) : SWAP16(UI_SPEC_BG);

    for (uint16_t x = 0U; x < SPEC_W; x++) row[x] = bg_sw;
    if (!is_grid) {
      for (uint16_t gx = 0U; gx < SPEC_W; gx += 40U) row[gx] = dot_sw;
    }
    /* Passband shaded fill — Xiegu-style; spectrum trace draws on top */
    if (do_shade) {
      for (uint16_t bx = pb_x0; bx <= pb_x1 && bx < SPEC_W; bx++)
        row[bx] = pb_sw;
    }
    /* Center carrier marker: single white pixel. No black flanks — they cause
     * a dark slot across the empty spectrum portion above the noise floor. */
    if (cx < SPEC_W) row[cx] = cx_sw;
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
    float    fbin = (float)x / xs + (float)b0;
    uint16_t b_lo = (uint16_t)fbin;
    float    t    = fbin - (float)b_lo;
    if (b_lo >= nb) b_lo = (uint16_t)(nb - 1U);
    uint16_t b_hi = (uint16_t)(b_lo + 1U);
    if (b_hi >= nb) b_hi = b_lo;
    float pwr = (1.0f - t) * s_wf_smooth[b_lo] + t * s_wf_smooth[b_hi];
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
  s_spec_zoom      = zoom;
  s_spec_sr        = s_zoom_sr_tbl[zoom];
  s_spec_half_ovr  = s_zoom_half_ovr[zoom];
  s_spec_py_valid  = false;   /* force full redraw; old cache mismatches new decimation */
  draw_footer_rows(spec_half_span_hz());
}

uint8_t SDR_UI_GetSpecZoom(void) { return s_spec_zoom; }

void SDR_UI_RedrawFooter(void)
{
  draw_footer_rows(spec_half_span_hz());
}

void SDR_UI_SetFooterFreq(uint32_t freq_hz, uint32_t step_hz)
{
  if (freq_hz == s_footer_freq_hz && step_hz == s_footer_step_hz) return;
  s_footer_freq_hz = freq_hz;
  s_footer_step_hz = step_hz;
  draw_footer_rows(spec_half_span_hz());
}

void SDR_UI_SetSpecMarker(int32_t offset_hz)
{
  if (offset_hz == s_spec_marker_hz) return;
  s_spec_marker_hz = offset_hz;
  /* Force a full spectrum redraw so the marker column moves even when the
   * spectrum content itself is below the delta-skip threshold. */
  s_spec_py_valid = false;
}

/* ── Spectrum skip statistics ─────────────────────────────────────────────── */
void SDR_UI_GetSpecSkipStats(uint32_t *skip_hits, uint32_t *draw_hits)
{
  if (skip_hits) *skip_hits = s_spec_skip_hits;
  if (draw_hits) *draw_hits = s_spec_draw_hits;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SDR_UI_DrawCWText  –  INFO strip (Y=120..144, 24 px × LCD_W)
 *
 *  Renders decoded CW text in amber (UI_MODE_CW = 0x07FF) centred vertically
 *  in the 24-row INFO zone, using Font6x8 (6×8 px).  Right-fills with spaces
 *  so leftover characters from a previous longer string are erased.
 *
 *  Uses the shared line buffer (LCD_GetLineBuf) to avoid a dedicated 23 KB
 *  static buffer for a 24-row zone.
 * ════════════════════════════════════════════════════════════════════════════ */
#if INFO_H > 0
static bool s_cw_dec_active = false;

static void cw_text_draw_rows(const char *text)
{
  uint16_t *ln    = LCD_GetLineBuf();
  uint16_t  txt_y = (uint16_t)((INFO_H - Font6x8.height) / 2U);
  /* When decode is armed but no text yet, show dim "[DEC]" placeholder */
  const char *render = text ? text : (s_cw_dec_active ? "[DEC]" : NULL);
  uint16_t    color  = text ? UI_MODE_CW : UI_STATUS_LBL;

  for (uint16_t row = 0U; row < INFO_H; row++) {
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    if (render && row >= txt_y && row < txt_y + Font6x8.height) {
      uint16_t fr = row - txt_y;
      uint16_t x  = 4U;
      for (const char *p = render; *p && x + Font6x8.width <= LCD_W; p++) {
        LCD_LineChar(ln, x, fr, *p, &Font6x8, color, UI_BG);
        x = (uint16_t)(x + Font6x8.width);
      }
    }
    LCD_PushWindow(0U, (uint16_t)(INFO_Y + row),
                   (uint16_t)(LCD_W - 1U), (uint16_t)(INFO_Y + row),
                   ln, LCD_W);
  }
}

void SDR_UI_DrawCWText(const char *text)  { cw_text_draw_rows(text); }
void SDR_UI_ClearCWText(void)             { cw_text_draw_rows(NULL);  }
void SDR_UI_SetCWDecActive(bool on)       { s_cw_dec_active = on; cw_text_draw_rows(NULL); }

/* RSSI display in the right column of INFO zone (below SBR, above spectrum).
 * Writes only [SBR_X .. SBR_X+SBR_W-1] per row — CW text in the left portion is unaffected. */
#if LCD_PANEL == LCD_PANEL_ST7796
static void rssi_info_draw(void)
{
  /* number (Font8x10, uppercase-safe) + unit "dBm" (Font6x8, full lowercase) */
  char numstr[8];
  int rv = (int)s_rssi_db;
  if (rv < -99) rv = -99;
  if (rv >   0) rv =   0;
  snprintf(numstr, sizeof(numstr), "%d", rv);

  static const char unit[] = "dBm";
  uint16_t num_w  = (uint16_t)(strlen(numstr) * (uint16_t)Font8x10.width);
  uint16_t unit_w = (uint16_t)((sizeof(unit) - 1U) * (uint16_t)Font6x8.width);
  uint16_t start_x = (uint16_t)(SBR_X + SBR_W - 2U - num_w - unit_w);
  uint16_t unit_x  = start_x + num_w;

  uint16_t num_y  = (uint16_t)((INFO_H - (uint16_t)Font8x10.height) / 2U);
  uint16_t unit_y = (uint16_t)((INFO_H - (uint16_t)Font6x8.height)  / 2U);
  uint16_t *ln    = LCD_GetLineBuf();

  for (uint16_t row = 0U; row < INFO_H; row++) {
    LCD_LineFill(ln, SBR_X, SBR_W, UI_BG);
    if (row >= num_y && row < num_y + (uint16_t)Font8x10.height)
      LCD_LineStrW(ln, start_x, row - num_y, numstr, &Font8x10, UI_STATUS_VAL, UI_BG);
    if (row >= unit_y && row < unit_y + (uint16_t)Font6x8.height)
      LCD_LineStr(ln, unit_x, row - unit_y, unit, &Font6x8, UI_STATUS_LBL, UI_BG);
    LCD_PushWindow(SBR_X, (uint16_t)(INFO_Y + row),
                   (uint16_t)(SBR_X + SBR_W - 1U), (uint16_t)(INFO_Y + row),
                   ln + SBR_X, SBR_W);
  }
}
#endif /* LCD_PANEL_ST7796 */

#else
void SDR_UI_DrawCWText(const char *text)  { (void)text; }
void SDR_UI_ClearCWText(void)             {}
void SDR_UI_SetCWDecActive(bool on)       { (void)on; }
#endif
