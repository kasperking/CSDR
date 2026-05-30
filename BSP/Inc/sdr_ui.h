/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_ui.h
  * @brief   CSDR SDR UI – zone layout over FMC LCD (ST7796 480×320 or ST7789 240×320)
  *
  *  ── ST7796 480×320 (LCD_PANEL_ST7796, default) ─────────────────────────────
  *  ┌─────────────────────────────────────────────────────────────┐  Y=0
  *  │  HEADER  480×24                              13.9V          │
  *  ├─────────┬────────────────────────────────────┬─────────────┤  Y=24
  *  │ SBL 80  │  VFO  320×64                       │ SBR 80      │
  *  │ Mode    │  14.200.000                         │BW 2.7k ST1k│
  *  │ VFO A/B │  A  USB  RX                         │MIC 25  AT6d│
  *  │ NR  NB  ├────────────────────────────────────┤DSP  1      │
  *  │ VOL 78  │  METER  320×32  2-px ruler style    │            │
  *  │ SQL  0  │  S 1   3   5   7   9  +20 +40      │            │
  *  │         │  |---|---|---|===|===|  ← 2 px act  │            │
  *  ├─────────┴────────────────────────────────────┴─────────────┤  Y=120
  *  │  INFO STRIP  480×24  (function key labels / status text)   │
  *  ├─────────────────────────────────────────────────────────────┤  Y=144
  *  │  SPECTRUM  480×72                                          │
  *  ├─────────────────────────────────────────────────────────────┤  Y=216
  *  │  WATERFALL  480×72                                         │
  *  ├─────────────────────────────────────────────────────────────┤  Y=288
  *  │  FOOTER  480×32   -24k      0      +24k                    │
  *  └─────────────────────────────────────────────────────────────┘  Y=320
  *
  *  ── ST7789 240×320 (LCD_PANEL_ST7789, compact) ─────────────────────────────
  *  ┌───────────────────────┐  Y=0
  *  │  HEADER  240×16       │  voltage
  *  ├───────────────────────┤  Y=16
  *  │  VFO  240×48          │  14.200.000  A  USB  RX
  *  ├───────────────────────┤  Y=64
  *  │  METER  240×24        │  S 1  3  5  7  9  +20 +40
  *  ├───────────────────────┤  Y=88
  *  │  SPECTRUM  240×76     │
  *  ├───────────────────────┤  Y=164
  *  │  WATERFALL  240×96    │
  *  ├───────────────────────┤  Y=260
  *  │  STATUS  240×28       │  USB  VOL:78  SQL:0 / BW:2.7k  NR NB
  *  ├───────────────────────┤  Y=288
  *  │  FOOTER  240×32       │  -24k   0   +24k
  *  └───────────────────────┘  Y=320
  *
  *  Panel is selected in lcd_panel_config.h (LCD_PANEL define).
  *  Transport: FMC 8080-mode via LCD_PushWindow / LCD_Clear.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SDR_UI_H
#define __SDR_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lcd_render.h"    /* SWAP16, Font6x8, LCD_LineFill/Str helpers, LCD_W  */
#include "lcd_bus_fmc.h"   /* LCD_PushWindow, LCD_Clear, LCD_FillRect            */

/* ════════════════════════════════════════════════════════════════════════════
 *  Zone geometry — two conditional layouts selected by lcd_panel_config.h
 * ════════════════════════════════════════════════════════════════════════════ */

#if LCD_PANEL == LCD_PANEL_ST7796
/* ── ST7796 480×320 layout (9-zone with sidebars) ────────────────────────── */

#define HDR_Y    0U
#define HDR_H   24U
#define HDR_Y2  24U

#define SBL_X    0U
#define SBL_W   80U
#define SBL_Y   24U
#define SBL_H   96U
#define SBL_Y2 120U

#define SBR_W   80U
#define SBR_X  400U          /* LCD_W - SBR_W = 480 - 80 */
#define SBR_Y   24U
#define SBR_H   96U
#define SBR_Y2 120U

#define VFO_X   80U
#define VFO_W  320U
#define VFO_Y   24U
#define VFO_H   64U
#define VFO_Y2  88U

#define MTR_X   80U
#define MTR_W  320U
#define MTR_Y   88U
#define MTR_H   32U
#define MTR_Y2 120U

/* Info strip – spacer between top panel and spectrum */
#define INFO_Y  120U
#define INFO_H   24U
#define INFO_Y2 144U

#define SPEC_X    0U
#define SPEC_W  480U
#define SPEC_Y  144U
#define SPEC_H   72U
#define SPEC_Y2 216U

#define WF_X     0U
#define WF_W   480U
#define WF_Y   216U
#define WF_H    72U
#define WF_Y2  288U

#define FTR_Y  288U
#define FTR_H   32U
#define FTR_Y2 320U

/* S-meter ruler (ST7796 32-row MTR zone):
 *   rows  1– 8: scale labels + inline S-value (Font5x8)
 *   rows 10–13: major ticks (4 px), rows 12–13: minor ticks (2 px)
 *   row  14:    top rail (1-px horizontal line)
 *   rows 17–18: 2-px continuous signal line (SM_LINE_R0, SM_LINE_H)
 *   row  22:    bottom rail; rows 23+: TX meter / unused           */

/* No compact STATUS zone on ST7796 */
#define STS_Y   FTR_Y
#define STS_H    0U
#define STS_Y2  FTR_Y

#elif LCD_PANEL == LCD_PANEL_ST7789
/* ── ST7789 240×320 compact layout (no sidebars) ────────────────────────── *
 *
 *  Total: 16+48+24+76+96+28+32 = 320 px ✓
 *
 *  SBL/SBR are defined as zero-size so callers compile cleanly.
 *  DrawSidebarLeft renders the STATUS zone; DrawSidebarRight is a no-op.
 */

#define HDR_Y    0U
#define HDR_H   16U      /* compact header */
#define HDR_Y2  16U

#define SBL_X    0U
#define SBL_W    0U      /* not rendered */
#define SBL_H    0U
#define SBL_Y   HDR_Y2
#define SBL_Y2  HDR_Y2

#define SBR_W    0U      /* not rendered */
#define SBR_X   LCD_W    /* off-screen origin (unused) */
#define SBR_Y   HDR_Y2
#define SBR_H    0U
#define SBR_Y2  HDR_Y2

#define VFO_X    0U
#define VFO_W   LCD_W    /* full width — no sidebars */
#define VFO_Y   HDR_Y2   /* = 16 */
#define VFO_H   48U
#define VFO_Y2  64U      /* HDR_Y2 + VFO_H */

#define MTR_X    0U
#define MTR_W   LCD_W
#define MTR_Y   VFO_Y2   /* = 64 */
#define MTR_H   24U
#define MTR_Y2  88U      /* MTR_Y + MTR_H */

/* No info strip on compact */
#define INFO_Y  MTR_Y2
#define INFO_H    0U
#define INFO_Y2 MTR_Y2

#define SPEC_X    0U
#define SPEC_W   LCD_W
#define SPEC_Y   MTR_Y2  /* = 88 */
#define SPEC_H   76U
#define SPEC_Y2 164U     /* SPEC_Y + SPEC_H */

#define WF_X     0U
#define WF_W    LCD_W
#define WF_Y    SPEC_Y2  /* = 164 */
#define WF_H    96U
#define WF_Y2  260U      /* WF_Y + WF_H */

/* Compact status bar — replaces sidebars (mode/vol/sq/bw/step/NR/NB) */
#define STS_Y   WF_Y2    /* = 260 */
#define STS_H   28U
#define STS_Y2 288U      /* STS_Y + STS_H */

#define FTR_Y   STS_Y2   /* = 288 */
#define FTR_H   32U
#define FTR_Y2  LCD_H    /* = 320 */

/* S-meter ruler (ST7789 24-row MTR zone):
 *   rows  1– 8: scale labels + inline S-value (Font5x8)
 *   rows 10–11: major ticks (2 px), row 11: minor ticks (1 px)
 *   row  12:    top rail (1-px horizontal line)
 *   rows 14–15: 2-px continuous signal line (SM_LINE_R0, SM_LINE_H)
 *   row  18:    bottom rail; rows 19+: TX meter / unused          */

#else
#  error "Unknown LCD_PANEL in sdr_ui.h — check lcd_panel_config.h"
#endif /* LCD_PANEL */

/* ── Zoom levels: 0=±24k  1=±18k  2=±12k  3=±6k  4=±3k ─────────────────── */
#define SPEC_ZOOM_COUNT  5U

/* ── S-meter ruler geometry (shared, fits both MTR widths) ──────────────── *
 *  SM_UNIT_W  : tick pitch (18 px); 12 × 18 = 216 px ruler
 *  SM_RULER_W : derived — total ruler span in pixels
 *  SM_START_X : left margin before first tick
 *
 *  Fit check (tightest panel: ST7789 MTR_W=240):
 *    ruler_end = SM_START_X + SM_RULER_W = 218
 *    val_x     = ruler_end + 4 = 222
 *    max label "+40" = 3 × 6 px = 18 px → ends at 240 = MTR_W  ✓
 * ─────────────────────────────────────────────────────────────────────────── */
#define SM_BARS      12U
#define SM_UNIT_W    18U
#define SM_START_X    2U
#define SM_RULER_W   (SM_BARS * SM_UNIT_W)   /* 216 px */

/* ── Legacy aliases (used by menu.c / sdr_scan.c) ──── */
#define ZONE_SPEC_Y   SPEC_Y
#define ZONE_SPEC_H   SPEC_H
#define ZONE_SPEC_Y2  SPEC_Y2
#define ZONE_WF_Y     WF_Y
#define ZONE_WF_H     WF_H
#define ZONE_WF_Y2    WF_Y2

/* ── Dirty-zone bitmask ─────────────────────────────── */
#define DIRTY_HDR   0x01U
#define DIRTY_SBL   0x02U
#define DIRTY_VFO   0x04U
#define DIRTY_SBR   0x08U
#define DIRTY_MTR   0x10U
#define DIRTY_ALL   0x1FU

/* ── Colour palette (RGB565) ────────────────────────── */
#define UI_BG             0x0000U
#define UI_HDR_BG         0x0000U
#define UI_SBL_BG         0x0000U
#define UI_SBR_BG         0x0000U
#define UI_VFO_BG         0x0000U
#define UI_MTR_BG         0x0000U
#define UI_BORDER         0x18C6U
#define UI_DIVIDER        0x4208U   /* zone dividers & minor ticks — dark gray  */

#define UI_FREQ_MHZ       0x07FFU  /* menu edit-mode highlight */
#define UI_FREQ_KHZ       0x3FE0U  /* BW/step sidebar values  */
#define UI_FREQ_FG        0xFFFFU  /* main VFO frequency       */
#define UI_FREQ_SUB       0x2945U  /* inactive-VFO sub-line (dimmed ~16%)  */

#define UI_MODE_AM        0xFFFFU
#define UI_MODE_FM        0x07E0U
#define UI_MODE_USB       0xF800U
#define UI_MODE_LSB       0xFFE0U
#define UI_MODE_CW        0x07FFU
#define UI_MODE_DIGU      0xFD20U  /* orange — digital USB (WSJT-X/FT8) */
#define UI_MODE_DIGL      0xFCC0U  /* amber  — digital LSB               */
#define UI_MODE_FG        0xFFFFU

#define UI_S1_6           0x07E0U
#define UI_S7_9           0xFFE0U
#define UI_S9P            0xF800U
#define UI_SMETER_BG      0x1082U
#define UI_SMETER_TICK    0xC618U   /* scale labels, ticks, rails — bright gray  */
#define UI_SMETER_ACT     0x0720U   /* active signal line — RF green (≈231/255 G, classic meter) */

#define UI_STATUS_LBL     0x3433U   /* dimmed: subdues sidebar labels vs. values */
#define UI_STATUS_VAL     0xFFFFU
#define UI_STATUS_ON      0x07E0U
#define UI_STATUS_OFF     0xF800U

#define UI_TX_BG          0xF800U   /* TX = red                      */
#define UI_TX_FG          0xFFFFU
#define UI_RX_BG          0x07E0U   /* RX = pure green (matches TX visual weight) */
#define UI_RX_FG          0xFFFFU

#define UI_SPEC_BG        0x0843U
#define UI_SPEC_GRID      0x18C6U
#define UI_SPEC_CENTER    0xF81FU
#define UI_SPEC_BW        0x07FFU

/* ── SDR UI state ───────────────────────────────────── */
typedef struct {
  uint32_t  freq_hz;
  uint32_t  freq_b_hz;     /*!< Inactive VFO frequency for sub-line display */
  uint8_t   mode;
  uint8_t   band_idx;
  float     signal_db;
  uint8_t   volume;
  uint8_t   squelch;
  uint32_t  step;
  bool      agc_fast;
  bool      nb_on;
  bool      nr_on;
  int16_t   rit_hz;
  bool      tx_mode;
  bool      si5351_ok;
  uint32_t  bw_hz;
  int16_t   voltage_x10;  /*!< Supply voltage × 10, e.g. 132 = 13.2 V     */
  uint8_t   att_db;
  uint8_t   att_x2;       /*!< PE4302 raw value (0–63, 0.5 dB/step); overrides att_db in display */
  bool      rf_agc_on;    /*!< RF AGC active — colours AT label green */
  int16_t   mic_gain;
  uint8_t   tx_power;     /*!< TX output power 0-100% for sidebar display */
  uint8_t   pa_watts;     /*!< PA rating 0/20/45/100 W; 0 = no PA */
  uint16_t  filter_len;
  uint8_t   dsp_level;
  uint8_t   active_vfo;    /*!< 0 = VFO A active, 1 = VFO B active */
} SDR_UI_State_t;

/* ── API ─────────────────────────────────────────────────────────────────────
 *  No lcd handle — FMC is memory-mapped and stateless.
 * ────────────────────────────────────────────────────────────────────────── */

void SDR_UI_Init(void);

/* One-time skeleton + footer; call before any zone draws */
void SDR_UI_DrawFrame(uint32_t sample_rate, uint16_t fft_bins);

/* Spectrum zoom (display-only, no DSP change).  Redraws footer immediately. */
void    SDR_UI_SetSpecZoom(uint8_t zoom);
uint8_t SDR_UI_GetSpecZoom(void);

/* Zone draws – each renders its buffer then pushes via FMC in one burst */
void SDR_UI_DrawHeader(const SDR_UI_State_t *ui);
void SDR_UI_DrawSidebarLeft(const SDR_UI_State_t *ui);
void SDR_UI_DrawVFO(const SDR_UI_State_t *ui);
void SDR_UI_DrawSidebarRight(const SDR_UI_State_t *ui);
void SDR_UI_DrawMeter(const SDR_UI_State_t *ui);

/* Convenience wrappers (backward compat) */
void SDR_UI_DrawTopBar(const SDR_UI_State_t *ui);
void SDR_UI_DrawStatusPanel(const SDR_UI_State_t *ui);

/* Spectrum: full redraw, single FMC burst */
void SDR_UI_DrawSpectrum(const float *fft_db, uint16_t bins,
                         float bw_lo_ratio, float bw_hi_ratio,
                         SDR_UI_State_t *ui);

/* Waterfall: IIR + LUT precompute (call from DSP task) */
uint8_t SDR_UI_WaterfallPrecompute(const float *fft_db, uint16_t bins);

/* Waterfall: 2-split ring push for true scroll (call from UI task) */
void SDR_UI_WaterfallPush(uint8_t buf_idx);

/* Compat: combines Precompute + Push in one call */
void SDR_UI_DrawWaterfall(const float *fft_db, uint16_t bins);

/* Meter fast-update (10 Hz) */
void SDR_UI_UpdateSMeter(float signal_db);
void SDR_UI_UpdateSMeter_SetTX(bool tx);
void SDR_UI_UpdateSMeter_SetVoltage(int16_t v_x10);  /*!< v × 10, e.g. 132 = 13.2 V */
void SDR_UI_UpdateTXMeters(int32_t alc_pct, int32_t swr_x10); /*!< alc 0-100 %, swr × 10 */

void SDR_UI_DrawFuncBar(const SDR_UI_State_t *ui);

/* Redraw footer (frequency scale labels) without changing zoom state. */
void SDR_UI_RedrawFooter(void);

/* Spectrum delta-skip counters */
void SDR_UI_GetSpecSkipStats(uint32_t *skip_hits, uint32_t *draw_hits);

/* Waterfall adaptive-skip control (called by csdr_app) */
void SDR_UI_SetWaterfallSuppressed(bool suppressed);
bool SDR_UI_GetWaterfallSuppressed(void);

/* TX mode UI policy.
 *   SetTXMode(true)  — no-op; SPEC+WF blanking deferred to first DrawTXSpectrum call.
 *   SetTXMode(false) — resets blanking flag and invalidates RX spec cache on TX→RX.
 *   DrawTXSpectrum   — compact audio-band mic spectrum in the SPEC zone (~5 fps in TX).
 *                      fft_db : linear power after fftshift (fft_db[bins/2]=DC).
 *                      mode   : UI mode byte (0=AM,1=FM,2=USB,3=LSB,4=CW).
 *                      sr     : audio sample rate (e.g. 48000). */
void SDR_UI_SetTXMode(bool tx_active);
void SDR_UI_DrawTXSpectrum(const float *fft_db, uint16_t bins,
                            uint8_t mode, uint32_t sr);

#ifdef __cplusplus
}
#endif
#endif /* __SDR_UI_H */
