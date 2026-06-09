/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    band_sel.h
  * @brief   Band quick-select overlay — hold BAND key to open
  *
  *  Navigation:
  *   Hold BAND     → Open overlay (cursor starts at current band)
  *   F1 / Encoder↑ → Cursor up
  *   F2 / Encoder↓ → Cursor down
  *   F3 / ENC_SW   → Confirm — apply selected band, close overlay
  *   F4 / BAND     → Cancel — close without changing band
  *
  *  Layout: 2-column grid over the SPEC+WF zone
  *   Left  col: 160m 80m 60m 40m 30m 20m
  *   Right col: 17m  15m 12m 10m 6m
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BAND_SEL_H
#define __BAND_SEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void    BandSel_Open(uint8_t current_band, uint8_t active_band);
void    BandSel_Close(void);
bool    BandSel_IsOpen(void);
void    BandSel_CursorUp(void);
void    BandSel_CursorDown(void);
uint8_t BandSel_Cursor(void);
void    BandSel_Render(void);

#ifdef __cplusplus
}
#endif
#endif /* __BAND_SEL_H */
