/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    terms.h
  * @brief   Full-screen "Terms of Use / Disclaimer" text viewer
  *          (System -> About -> "Terms & Disclaimer").
  *
  *  Read-only, scrollable notice covering: amateur-license requirement,
  *  no-warranty / experimental-firmware disclaimer, and an explicit callout
  *  of the TX Band Unlock feature's legal risk (see tx_unlock.h). Text lives
  *  in terms.c as a fixed line array — no persistence, no state.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TERMS_H
#define __TERMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Full-screen blocking text viewer. Scroll with ENC/F1/F2, exit with F4.
 * Pumps CSDR_ProcessAudioPending() every iteration (same pattern as
 * Cal_Run / SWR_Scan_Run / TxUnlock_Run) and calls SDR_UI_InvalidateCaches()
 * + SDR_UI_ClearCWText() on exit (full-screen overlay rule). */
void Terms_ShowDisclaimer(void);

#ifdef __cplusplus
}
#endif
#endif /* __TERMS_H */
