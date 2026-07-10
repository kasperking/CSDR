/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mode_sel.h
  * @brief   Mode quick-select overlay — hold MODE key to open
  *
  *  Navigation:
  *   Hold MODE    → Open overlay (cursor starts at current mode)
  *   F1 / Encoder↑ → Cursor up
  *   F2 / Encoder↓ → Cursor down
  *   F3 / ENC_SW   → Confirm — apply selected mode, close overlay
  *   F4 / MODE     → Cancel — close without changing mode
  *
  *  Layout: 4-column × 2-row grid over the SPEC+WF zone
  *   Row 0: AM  FM  USB  LSB
  *   Row 1: CW DIGU DIGL
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MODE_SEL_H
#define __MODE_SEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void    ModeSel_Open(uint8_t current_mode, uint8_t active_mode);
void    ModeSel_Close(void);
bool    ModeSel_IsOpen(void);
void    ModeSel_CursorUp(void);
void    ModeSel_CursorDown(void);
uint8_t ModeSel_Cursor(void);
void    ModeSel_Render(void);

#ifdef __cplusplus
}
#endif
#endif /* __MODE_SEL_H */
