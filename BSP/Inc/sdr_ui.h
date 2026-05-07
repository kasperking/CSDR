/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_ui.h
  * @brief   CSDR SDR UI – 8-zone layout 320×240
  *
  *  ┌──────────────────────────────────────────────────┐  Y=0
  *  │  HEADER  320×18    [RX] ATT:6  13.9V            │
  *  ├──────────┬─────────────────────────┬─────────────┤  Y=18
  *  │ SBL 60   │  VFO  200×44            │ SBR 60      │
  *  │ Mode     │  28.564.000        1kHz  │ RIT  +150   │
  *  │ VFO A/B  │ A  B:28.564.000   2.7k  │ MIC  21     │
  *  │ NR  NB   ├─────────────────────────┤ DSP  6      │
  *  │ VOL  78  │  METER  200×38          │ LEN  45     │
  *  │ SQL   0  │  S▐▐▐▐▐▐░░░  S7        │             │
  *  │          │                         │             │
  *  ├──────────┴─────────────────────────┴─────────────┤  Y=100
  *  │  SPECTRUM  320×68                                │
  *  ├──────────────────────────────────────────────────┤  Y=168
  *  │  WATERFALL  320×62                               │
  *  ├──────────────────────────────────────────────────┤  Y=230
  *  │  FOOTER  320×10  -24k    0    +24k               │
  *  └──────────────────────────────────────────────────┘  Y=240
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SDR_UI_H
#define __SDR_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "st7789.h"

/* ── Zone geometry ──────────────────────────────────── */

#define HDR_Y    0U
#define HDR_H   18U
#define HDR_Y2  18U

#define SBL_X    0U
#define SBL_W   60U
#define SBL_Y   18U
#define SBL_H   82U
#define SBL_Y2 100U

#define SBR_W   60U
#define SBR_X  260U          /* LCD_W - SBR_W */
#define SBR_Y   18U
#define SBR_H   82U
#define SBR_Y2 100U

#define VFO_X   60U
#define VFO_W  200U
#define VFO_Y   18U
#define VFO_H   44U
#define VFO_Y2  62U

#define MTR_X   60U
#define MTR_W  200U
#define MTR_Y   62U
#define MTR_H   38U
#define MTR_Y2 100U

#define SPEC_X    0U
#define SPEC_W  320U
#define SPEC_Y  100U
#define SPEC_H   68U
#define SPEC_Y2 168U
/* Zoom levels: 0=±24k  1=±18k  2=±12k  3=±6k  4=±3k  (display-only, DSP unchanged) */
#define SPEC_ZOOM_COUNT  5U

#define WF_X     0U
#define WF_W   320U
#define WF_Y   168U
#define WF_H    62U
#define WF_Y2  230U

#define FTR_Y  230U
#define FTR_H   10U
#define FTR_Y2 240U

/* ── S-meter bar geometry (inside MTR) ──────────────── */
#define SM_BARS      12U
#define SM_BAR_W     13U
#define SM_BAR_GAP    1U
#define SM_BAR_H     20U
#define SM_BAR_YOFF  11U
#define SM_START_X    4U
#define SM_TOTAL_W   (SM_BARS * (SM_BAR_W + SM_BAR_GAP))  /* 168 */

/* ── Dirty-zone bitmask ─────────────────────────────── */
#define DIRTY_HDR   0x01U
#define DIRTY_SBL   0x02U
#define DIRTY_VFO   0x04U
#define DIRTY_SBR   0x08U
#define DIRTY_MTR   0x10U

/* ── Legacy aliases (used by menu.c) ────────────────── */
#define ZONE_SPEC_Y   SPEC_Y
#define ZONE_SPEC_H   SPEC_H
#define ZONE_SPEC_Y2  SPEC_Y2
#define ZONE_WF_Y     WF_Y
#define ZONE_WF_H     WF_H
#define ZONE_WF_Y2    WF_Y2

/* ── Colour palette (RGB565) ────────────────────────── */
#define UI_BG             0x0000U
#define UI_HDR_BG         0x0000U
#define UI_SBL_BG         0x0000U
#define UI_SBR_BG         0x0000U
#define UI_VFO_BG         0x0000U
#define UI_MTR_BG         0x0000U
#define UI_BORDER         0x18C6U
#define UI_DIVIDER        0x18C6U

#define UI_FREQ_MHZ       0x07FFU
#define UI_FREQ_KHZ       0x3FE0U
#define UI_FREQ_HZ        0x2D65U
#define UI_FREQ_DOT       0x07FFU
#define UI_FREQ_SUB       0x528AU

#define UI_MODE_AM        0x7800U
#define UI_MODE_FM        0x0380U
#define UI_MODE_USB       0x001FU
#define UI_MODE_LSB       0x600FU
#define UI_MODE_CW        0xCA00U
#define UI_MODE_FG        0xFFE0U

#define UI_S1_6           0x07E0U
#define UI_S7_9           0xFFE0U
#define UI_S9P            0xF800U
#define UI_SMETER_BG      0x1082U
#define UI_SMETER_TICK    0x5AEBU

#define UI_STATUS_LBL     0x8410U
#define UI_STATUS_VAL     0xFFFFU
#define UI_STATUS_ON      0x07E0U
#define UI_STATUS_OFF     0xF800U

#define UI_TX_BG          0xF800U
#define UI_TX_FG          0xFFFFU
#define UI_RX_BG          0x0400U
#define UI_RX_FG          0x07E0U

#define UI_SPEC_BG        0x0000U
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
  float     voltage;
  uint8_t   att_db;
  int16_t   mic_gain;
  uint16_t  filter_len;
  uint8_t   dsp_level;
  uint8_t   active_vfo;    /*!< 0 = VFO A active, 1 = VFO B active */
} SDR_UI_State_t;

/* ── API ────────────────────────────────────────────── */

void SDR_UI_Init(void);

/* One-time skeleton + footer; call before any zone draws */
void SDR_UI_DrawFrame(ST7789_Handle_t *lcd, uint32_t sample_rate, uint16_t fft_bins);

/* Spectrum zoom (display-only, no DSP change).  Redraws footer immediately. */
void    SDR_UI_SetSpecZoom(ST7789_Handle_t *lcd, uint8_t zoom);
uint8_t SDR_UI_GetSpecZoom(void);

/* Zone draws – each renders its buffer then pushes in a single SPI block */
void SDR_UI_DrawHeader(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_DrawSidebarLeft(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_DrawVFO(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_DrawSidebarRight(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_DrawMeter(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);

/* Convenience wrappers (backward compat) */
void SDR_UI_DrawTopBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_DrawStatusPanel(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);

/* Spectrum: full redraw, single PushBlock */
void SDR_UI_DrawSpectrum(ST7789_Handle_t *lcd,
                         const float *fft_db, uint16_t bins,
                         float bw_lo_ratio, float bw_hi_ratio,
                         SDR_UI_State_t *ui);

/* Waterfall: IIR + LUT precompute (call from DSP task) */
uint8_t SDR_UI_WaterfallPrecompute(const float *fft_db, uint16_t bins);

/* Waterfall: 2-split ring push for true scroll (call from UI task) */
void SDR_UI_WaterfallPush(ST7789_Handle_t *lcd, uint8_t buf_idx);

/* Compat: combines Precompute + Push in one call */
void SDR_UI_DrawWaterfall(ST7789_Handle_t *lcd,
                          const float *fft_db, uint16_t bins);

/* Meter fast-update (10 Hz) */
void SDR_UI_UpdateSMeter(ST7789_Handle_t *lcd, float signal_db);
void SDR_UI_UpdateSMeter_SetTX(bool tx);
void SDR_UI_UpdateSMeter_SetVoltage(float v);
void SDR_UI_UpdateTXMeters(ST7789_Handle_t *lcd, float alc_norm, float swr);

void SDR_UI_DrawFuncBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);

#ifdef __cplusplus
}
#endif
#endif /* __SDR_UI_H */
