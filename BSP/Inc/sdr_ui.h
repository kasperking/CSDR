/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_ui.h
  * @brief   CSDR SDR UI layer
  *
  *  Layout 320×240 landscape:
  *  ┌─────────────────────────────────────────────────┐ Y=0
  *  │ [USB] [40m]   7.100.000   S▓▓▓▓░░ S7   [TX]   │ 62px
  *  ├────────╂────────────────────────────────────────┤ Y=62
  *  │ AGC    ┃                                        │
  *  │ NB     ┃           SPECTRUM  241×108px          │ 108px
  *  │ NR     ┃                                        │
  *  ├────────╂────────────────────────────────────────┤ Y=170
  *  │ RIT    ┃                                        │
  *  │ VOL    ┃           WATERFALL 241×70px           │ 70px
  *  │ ...    ┃                                        │
  *  └────────┴────────────────────────────────────────┘ Y=240
  *
  *  Calls only the public st7789.h driver API.
  *  Driver never includes this file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SDR_UI_H
#define __SDR_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "st7789.h"

/* ── Layout geometry ────────────────────────────────── */
#define PANEL_W       78U
#define PANEL_X       0U
#define PANEL_DIV_X   (PANEL_W)

#define DISP_X        (PANEL_W + 1U)
#define DISP_W        (LCD_W - DISP_X)   /* 241px */

#define ZONE_TOPBAR_Y    0U
#define ZONE_TOPBAR_H    62U
#define ZONE_TOPBAR_R1H  28U
#define ZONE_TOPBAR_R2H  34U
#define ZONE_TOPBAR_Y2   (ZONE_TOPBAR_Y + ZONE_TOPBAR_H)

#define ZONE_SPEC_Y      (ZONE_TOPBAR_Y2)
#define ZONE_SPEC_H      108U
#define ZONE_SPEC_Y2     (ZONE_SPEC_Y + ZONE_SPEC_H)

#define ZONE_WF_Y        (ZONE_SPEC_Y2)
#define ZONE_WF_H        70U
#define ZONE_WF_Y2       (ZONE_WF_Y + ZONE_WF_H)

/* S-meter geometry */
#define SM_BARS     12U
#define SM_BAR_W    13U
#define SM_BAR_GAP  1U
#define SM_BAR_H    22U
#define SM_BAR_YOFF 9U
#define SM_START_X  10U
#define SM_TOTAL_W  (SM_BARS * (SM_BAR_W + SM_BAR_GAP))
#define SM_SVAL_X   (SM_START_X + SM_TOTAL_W + 4U)
#define RXTX_X      (LCD_W - 56U)
#define RXTX_W      56U

/* Function buttons */
#define BTN_COUNT   7U
#define BTN_W       (LCD_W / BTN_COUNT)

/* ── Colour palette (RGB565) ────────────────────────── */
#define UI_BG             0x0000U
#define UI_PANEL_BG       0x0841U
#define UI_TOPBAR_BG      0x0208U
#define UI_BORDER         0x2945U
#define UI_DIVIDER        0x31A6U
#define UI_FREQ_MHZ       0x07FFU
#define UI_FREQ_KHZ       0x3FE0U
#define UI_FREQ_HZ        0x2D65U
#define UI_FREQ_DOT       0x07FFU
#define UI_MODE_BG        0x000FU
#define UI_MODE_FG        0xFFE0U
#define UI_BAND_BG        0x3800U
#define UI_BAND_FG        0xFD20U
#define UI_SMETER_BG      0x1082U
#define UI_S1_6           0x07E0U
#define UI_S7_9           0xFFE0U
#define UI_S9P            0xF800U
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
#define UI_SPEC_LOW       0x001FU
#define UI_SPEC_MID       0x07E0U
#define UI_SPEC_HIGH      0xFD20U
#define UI_SPEC_PEAK      0xFFFFU
#define UI_SPEC_CENTER    0xF81FU
#define UI_SPEC_BW        0x07FFU
#define UI_BTN_BG         0x0841U
#define UI_BTN_BORDER     0x2945U
#define UI_BTN_FG         0xFFE0U
#define UI_BTN_ACTIVE_BG  0x0007U
#define UI_BTN_ACTIVE_FG  0x07FFU

/* ── SDR UI state ───────────────────────────────────── */
typedef struct {
  uint32_t  freq_hz;
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
  bool      qse_on;
  uint8_t   active_btn;
  uint32_t  bw_hz;
} SDR_UI_State_t;

/* ── API ────────────────────────────────────────────── */

/* Call once after ST7789_Init() to zero UI buffers and build LUT. */
void SDR_UI_Init(void);

void SDR_UI_DrawFrame(ST7789_Handle_t *lcd);
void SDR_UI_DrawTopBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_DrawStatusPanel(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_DrawSpectrum(ST7789_Handle_t *lcd,
                         const float *fft_db, uint16_t bins,
                         float bw_lo_ratio, float bw_hi_ratio,
                         SDR_UI_State_t *ui);
void SDR_UI_DrawWaterfall(ST7789_Handle_t *lcd,
                          const float *fft_db, uint16_t bins);
void SDR_UI_DrawFuncBar(ST7789_Handle_t *lcd, const SDR_UI_State_t *ui);
void SDR_UI_UpdateSMeter(ST7789_Handle_t *lcd, float signal_db);
void SDR_UI_UpdateSMeter_SetTX(bool tx);

#ifdef __cplusplus
}
#endif
#endif /* __SDR_UI_H */
