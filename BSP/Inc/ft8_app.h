/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ft8_app.h
  * @brief   Full-screen FT8 monitor app (SWR-Scan-style overlay)
  *
  *  Launched from menu root ACTION "FT8".  Blocking, but keeps the radio
  *  fully alive: every loop iteration pumps CSDR_ProcessAudioPending(),
  *  ticks FT8_Poll() and refreshes the watchdog.  Shows a WSJT-X-style
  *  scrolling decode list (UTC time, SNR, DT, freq, message) that persists
  *  across app sessions.  Exit with MENU or F4.
  *
  *  Caller contract (see csdr_app.c ACTION dispatch):
  *   - mode must already be DIGU/USB (launcher forces DIGU),
  *   - after return: discard stale encoder delta + set DIRTY_ALL.
  *  The app itself restores the FT8 engine to the menu-toggle state and
  *  calls SDR_UI_InvalidateCaches()/ClearCWText() before returning
  *  (full-screen overlay exit rule).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __FT8_APP_H
#define __FT8_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void FT8_App_Run(void);

#ifdef __cplusplus
}
#endif
#endif /* __FT8_APP_H */
