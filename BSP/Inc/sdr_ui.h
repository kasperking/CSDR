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
#include "pa_protect.h"    /* PA_State_t, PA_Fault_t for TX warning overlay      */

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

#define SBR_W   88U
#define SBR_X  392U          /* LCD_W - SBR_W = 480 - 88 */
#define SBR_Y   24U
#define SBR_H   96U
#define SBR_Y2 120U

#define VFO_X   80U
#define VFO_W  312U
#define VFO_Y   24U
#define VFO_H   64U
#define VFO_Y2  88U

#define MTR_X   80U
#define MTR_W  312U
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
/* ── ST7789: orientation-split layout ───────────────────────────────────── *
 *
 *  Landscape (320x240): 12+44+20+56+60+24+24 = 240 px
 *  Portrait  (240x320): 16+48+24+76+96+28+32 = 320 px
 *
 *  SBL/SBR are zero-size on both orientations — DrawSidebarLeft renders the
 *  STATUS bar; DrawSidebarRight is a no-op.
 */
#if LCD_W > LCD_H
/* ── ST7789 320×240 landscape ──────────────────────────────────────────── *
 *
 *  New layout — STS zone eliminated, params merged into VFO zone:
 *
 *  ┌──────────────────────────────────────────────────────────────────┐  Y=0
 *  │  HEADER  320×12  AGC-F                              13.8V       │
 *  ├──────────────────────────────────────────────────────────────────┤  Y=12
 *  │  VFO  320×60   [A] 14.200.000    │ USB ← right 120px           │
 *  │                                  │ RX                           │
 *  │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ (divider row 36)            │
 *  │  STP:100  VOL:70  [NB]                                          │
 *  │  BW:3000  SQL:0   [NR]                                          │
 *  ├──────────────────────────────────────────────────────────────────┤  Y=72
 *  │  METER  320×20   S-meter ruler                                  │
 *  ├──────────────────────────────────────────────────────────────────┤  Y=92
 *  │  SPECTRUM  320×60                                               │
 *  ├──────────────────────────────────────────────────────────────────┤  Y=152
 *  │  WATERFALL  320×64                                              │
 *  ├──────────────────────────────────────────────────────────────────┤  Y=216
 *  │  FOOTER  320×24  -24k        0        +24k                      │
 *  └──────────────────────────────────────────────────────────────────┘  Y=240
 *
 *  Total: 12+60+20+60+64+24 = 240 ✓
 */

#define HDR_Y    0U
#define HDR_H   12U
#define HDR_Y2  12U

#define SBL_X    0U
#define SBL_W    0U
#define SBL_H    0U
#define SBL_Y   HDR_Y2
#define SBL_Y2  HDR_Y2

#define SBR_W    0U
#define SBR_X   LCD_W
#define SBR_Y   HDR_Y2
#define SBR_H    0U
#define SBR_Y2  HDR_Y2

#define VFO_X    0U
#define VFO_W   LCD_W
#define VFO_Y   HDR_Y2   /* = 12 */
#define VFO_H   60U
#define VFO_Y2  72U

/* Right panel split: digits centered in left 180px; right 140px for status inline */
#define VFO_RIGHT_X    180U

#define MTR_X    0U
#define MTR_W   LCD_W
#define MTR_Y   VFO_Y2   /* = 72 */
#define MTR_H   20U
#define MTR_Y2  92U

#define INFO_Y  MTR_Y2
#define INFO_H    0U
#define INFO_Y2 MTR_Y2

#define SPEC_X    0U
#define SPEC_W   LCD_W
#define SPEC_Y   MTR_Y2  /* = 92 */
#define SPEC_H   60U
#define SPEC_Y2 152U

#define WF_X     0U
#define WF_W    LCD_W
#define WF_Y    SPEC_Y2  /* = 152 */
#define WF_H    64U
#define WF_Y2  216U

#define STS_Y   WF_Y2    /* = 216 */
#define STS_H    0U
#define STS_Y2  216U

#define FTR_Y   STS_Y2   /* = 216 */
#define FTR_H   24U
#define FTR_Y2  LCD_H    /* = 240 */

/* S-meter ruler (ST7789 landscape 20-row MTR zone):
 *   rows  0– 9: scale labels + inline S-value (Font8x10)
 *   rows 10–11: major ticks (2 px), row 11: minor ticks (1 px)
 *   row  12:    top rail (1-px horizontal line)
 *   rows 13–16: 4-px signal line (SM_LINE_R0..+SM_LINE_H-1)
 *   row  18:    bottom rail; TX ALC/SWR text omitted (MTR too short) */

#else
/* ── ST7789 240×320 portrait ───────────────────────────────────────────── *
 *
 *  New layout — STS zone eliminated, params merged into VFO zone right panel:
 *
 *  ┌───────────────────────┐  Y=0
 *  │  HEADER  240×16       │  AGC-F / 13.8V
 *  ├───────────────────────┤  Y=16
 *  │  VFO  240×72          │  [A] 7.100.000 │ USB  ← right panel 48px
 *  │                       │                │ RX
 *  │  B 14.200.000         │  sub-VFO MED 12x16 (row 28..43)
 *  │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ │  (divider @ row 38 within VFO)
 *  │  STP:xxx  VOL:xx  NB  │  full-width params row A (row 46)
 *  │  BW:xxx   SQL:x   NR  │  full-width params row B (row 55)
 *  ├───────────────────────┤  Y=88
 *  │  METER  240×24        │  S-meter ruler
 *  ├───────────────────────┤  Y=112
 *  │  SPECTRUM  240×88     │
 *  ├───────────────────────┤  Y=200
 *  │  WATERFALL  240×88    │
 *  ├───────────────────────┤  Y=288
 *  │  FOOTER  240×32       │  -24k   0   +24k
 *  └───────────────────────┘  Y=320
 *
 *  Total: 16+72+24+88+88+32 = 320 ✓
 */

#define HDR_Y    0U
#define HDR_H   16U
#define HDR_Y2  16U

#define SBL_X    0U
#define SBL_W    0U
#define SBL_H    0U
#define SBL_Y   HDR_Y2
#define SBL_Y2  HDR_Y2

#define SBR_W    0U
#define SBR_X   LCD_W
#define SBR_Y   HDR_Y2
#define SBR_H    0U
#define SBR_Y2  HDR_Y2

#define VFO_X    0U
#define VFO_W   LCD_W
#define VFO_Y   HDR_Y2   /* = 16 */
#define VFO_H   72U       /* expanded: right panel + params rows inline */
#define VFO_Y2  88U

/* Right panel split: left 192px = digit area; right 48px = mode/RX labels */
#define VFO_RIGHT_X    192U
/* Params rows within s_vfo_buf (below digit/badge area, full-width) */
#define VFO_PARAMS_Y    46U   /* row for params row A (STP/VOL/NB) -- below sub-VFO */
#define VFO_PARAMS_B_Y  55U   /* row for params row B (BW/SQL/NR)  */

#define MTR_X    0U
#define MTR_W   LCD_W
#define MTR_Y   VFO_Y2   /* = 88 */
#define MTR_H   24U
#define MTR_Y2  112U

#define INFO_Y  MTR_Y2
#define INFO_H    0U
#define INFO_Y2 MTR_Y2

#define SPEC_X    0U
#define SPEC_W   LCD_W
#define SPEC_Y   MTR_Y2  /* = 112 */
#define SPEC_H   88U
#define SPEC_Y2 200U

#define WF_X     0U
#define WF_W    LCD_W
#define WF_Y    SPEC_Y2  /* = 200 */
#define WF_H    88U
#define WF_Y2  288U

/* STS zone eliminated — status info merged into VFO zone right panel */
#define STS_Y   WF_Y2    /* = 288 */
#define STS_H    0U
#define STS_Y2  288U

#define FTR_Y   STS_Y2   /* = 288 */
#define FTR_H   32U
#define FTR_Y2  LCD_H    /* = 320 */

/* S-meter ruler (ST7789 portrait 24-row MTR zone):
 *   rows  0– 9: scale labels + inline S-value (Font8x10)
 *   rows 10–11: major ticks (2 px), row 11: minor ticks (1 px)
 *   row  12:    top rail (1-px horizontal line)
 *   rows 13–16: 4-px signal line (SM_LINE_R0..+SM_LINE_H-1)
 *   row  18:    bottom rail; rows 19+: TX meter / unused          */

#endif /* LCD_W > LCD_H */

#else
#  error "Unknown LCD_PANEL in sdr_ui.h — check lcd_panel_config.h"
#endif /* LCD_PANEL */

/* ── Zoom levels: 0=±24k  1=±12k  2=±6k  3=±3k ─────────────────────────── */
#define SPEC_ZOOM_COUNT  4U

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
/* Landscape ST7789 (320×240): narrower tick pitch so ruler ≈ 50% of screen */
#if LCD_PANEL == LCD_PANEL_ST7789 && LCD_W > LCD_H
#undef  SM_UNIT_W
#define SM_UNIT_W   13U
#undef  SM_RULER_W
#define SM_RULER_W  (SM_BARS * SM_UNIT_W)   /* 156 px */
#endif

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
#define UI_MODE_FREEDV    0x07F0U  /* cyan-green — FreeDV NBUSB          */
#define UI_MODE_FG        0xFFFFU

#define UI_S1_6           0x07E0U
#define UI_S7_9           0xFFE0U
#define UI_S9P            0xF800U
#define UI_SMETER_BG      0x1082U
#define UI_SMETER_TICK    0xC618U   /* scale labels, ticks, rails — bright gray  */
#define UI_SMETER_ACT     0x0720U   /* active signal line — RF green (≈231/255 G, classic meter) */

#define UI_STATUS_LBL     0x3433U   /* dimmed label — note: has slight blue tint (B>R) */
#define UI_STATUS_VAL     0xFFFFU
#define UI_STATUS_ON      0x07E0U
#define UI_STATUS_OFF     0xF800U
#define UI_STATUS_WARN    0xFD00U   /* amber — hardware missing warning */
#define UI_STATUS_HINT    0x5ACBU   /* neutral gray: R=11 G=22 B=11 → R≈G≈B≈89/255 */

#define UI_TX_BG          0xF800U   /* TX = red                      */
#define UI_TX_FG          0xFFFFU
#define UI_RX_BG          0x07E0U   /* RX = pure green (matches TX visual weight) */
#define UI_RX_FG          0xFFFFU

#define UI_SPEC_BG        0x0843U
#define UI_SPEC_GRID      0x18C6U
#define UI_SPEC_CENTER    0xF81FU
#define UI_SPEC_BW        0x07FFU
#define UI_SPEC_PASS      0x0929U   /* passband shaded region — dim teal fill */

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
  uint8_t   agc_speed;   /*!< 0=SLOW 1=FAST 2=AUTO */
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
  uint32_t  fwd_power_mw; /*!< Measured fwd power envelope (mW), TX only; PW badge source */
  uint16_t  filter_len;
  uint8_t   dsp_level;
  uint8_t   active_vfo;    /*!< 0 = VFO A active, 1 = VFO B active */
} SDR_UI_State_t;

/* ── API ─────────────────────────────────────────────────────────────────────
 *  No lcd handle — FMC is memory-mapped and stateless.
 * ────────────────────────────────────────────────────────────────────────── */

void SDR_UI_Init(void);

/* Invalidate all partial-redraw caches (VFO glyph, sidebars, meter statics,
 * spectrum delta-skip, RSSI).  Call after a full-screen overlay (SWR scan)
 * painted over the UI zones so the next DIRTY_ALL refresh repaints them all
 * instead of cache-skipping. */
void SDR_UI_InvalidateCaches(void);

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

/* CW decoder text strip (INFO zone, Y=120..144, 24 px).
 * text: null-terminated string of decoded chars; drawn amber in CW mode.
 * Call SDR_UI_ClearCWText() when leaving CW mode to restore INFO to blank.
 * Call SDR_UI_SetCWDecActive(true) when decode is enabled — shows dim [DEC]
 * placeholder until first decoded char arrives; false clears it. */
void SDR_UI_DrawCWText(const char *text);
void SDR_UI_ClearCWText(void);
void SDR_UI_SetCWDecActive(bool on);

/* Meter fast-update (10 Hz) */
void SDR_UI_UpdateSMeter(float signal_db);
void SDR_UI_UpdateSMeter_SetTX(bool tx);
void SDR_UI_UpdateSMeter_SetVoltage(int16_t v_x10);  /*!< v × 10, e.g. 132 = 13.2 V */
void SDR_UI_UpdateTXMeters(int32_t alc_pct, int32_t swr_x10); /*!< alc 0-100 %, swr × 10 */

/* Redraw footer (frequency scale labels) without changing zoom state. */
void SDR_UI_RedrawFooter(void);

/* Update VFO frequency and tuning step for the frequency ruler.
 * Grid marks are placed at multiples of step_hz; redraws only when changed. */
void SDR_UI_SetFooterFreq(uint32_t freq_hz, uint32_t step_hz);

/* Track-mode spectrum marker: demod offset (Hz) from the spectrum center.
 * 0 = centered (Fix mode). Passband shading + carrier marker follow it. */
void SDR_UI_SetSpecMarker(int32_t offset_hz);

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
 *                      sr     : audio sample rate (e.g. 48000).
 */
void SDR_UI_SetTXMode(bool tx_active);
bool SDR_UI_IsTXZoneBlanked(void);
void SDR_UI_DrawTXSpectrum(const float *fft_db, uint16_t bins,
                            uint8_t mode, uint32_t sr);

/* Software UTC clock displayed in the header topbar.
 * Call once after boot (or after NTP/CAT sync) to set HH:MM:SS.
 * Counts forward using HAL_GetTick(); wraps at 24 h. */
void SDR_UI_SetClock(uint8_t h, uint8_t m, uint8_t s);
void SDR_UI_GetClock(uint8_t *h, uint8_t *m, uint8_t *s);  /*!< Read current HH:MM:SS from the running clock */

/* Persistent PA fault warning in the INFO zone (between S-meter and spectrum).
 * Safe to call at any rate; redraws only when state changes (or always when
 * active to survive DrawCWText overwrites).
 * Call from csdr_refresh_display() and from a periodic 500 ms timer. */
void SDR_UI_UpdatePAWarn(PA_State_t state, PA_Fault_t fault);

#ifdef __cplusplus
}
#endif
#endif /* __SDR_UI_H */
