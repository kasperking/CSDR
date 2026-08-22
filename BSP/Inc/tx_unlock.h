/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tx_unlock.h
  * @brief   Out-of-band TX limiter + secret-code unlock (MARS/CAP-style)
  *
  *  Policy
  *  ------
  *  By default the transmitter is gated to the amateur band edges declared in
  *  bpf_lpf.h (BAND_FREQ_MIN / BAND_FREQ_MAX): a new TX keying whose carrier
  *  falls outside every ham band is refused.  An operator who is authorised to
  *  transmit outside the ham bands (e.g. MARS / CAP / service test) can lift
  *  the limit by entering a secret code on the front panel
  *  (System -> "TX Band Unlock").
  *
  *  Audit trail ("leaves a trace")
  *  ------------------------------
  *  Every unlock, re-lock, wrong-code attempt, and out-of-band transmission is
  *  recorded in a dedicated flash sector (FLASH_ADDR_TX_UNLOCK) together with
  *  the RTC time of the last unlock.  That sector is NOT part of the settings
  *  region, so a Factory Reset does not erase it — the record cannot be cleared
  *  from the normal UI.  The unlocked state also raises a persistent amber
  *  "OOB TX" tag in the header while it is active.
  *
  *  The secret code lives only in the compiled firmware (TX_UNLOCK_SECRET in
  *  tx_unlock.c) — it is never shown on screen nor reported over CAT.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TX_UNLOCK_H
#define __TX_UNLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "w25q.h"          /* W25Q_Handle_t */
#include <stdint.h>
#include <stdbool.h>

/* Persistent out-of-band TX unlock audit record.
 * Stored on its own flash sector so it survives a settings Factory Reset.
 * Layout: 4-byte fields first, 1-byte flags, crc32 last (covers all bytes
 * before itself). */
typedef struct {
  uint32_t magic;          /* TX_UNLOCK_MAGIC when valid                      */
  uint32_t unlock_count;   /* successful code entries that engaged the unlock */
  uint32_t relock_count;   /* manual re-locks from the unlock screen          */
  uint32_t fail_count;     /* wrong-code attempts                             */
  uint32_t oob_tx_count;   /* out-of-band TX keyings performed while unlocked */
  uint32_t last_uptime_s;  /* HAL uptime (s) at the last successful unlock    */
  uint8_t  unlocked;       /* 1 = out-of-band TX currently permitted          */
  uint8_t  last_hh;        /* RTC time-of-day of the last unlock (if RTC set) */
  uint8_t  last_mm;
  uint8_t  last_ss;
  uint8_t  last_time_valid;/* 1 = RTC was set when the last unlock happened   */
  uint8_t  pad[3];         /* keep crc32 4-byte aligned; memset to 0          */
  uint32_t crc32;
} TxUnlockLog_t;

#define TX_UNLOCK_MAGIC  0x5701C0DEUL

/* Load the audit record from flash into RAM. Call once at boot after the W25Q
 * driver is up. A blank/invalid sector leaves the record zeroed (locked) and
 * is NOT written back — the first real event writes it, so boot stays fast. */
void TxUnlock_Init(W25Q_Handle_t *dev);

/* True while out-of-band TX is permitted (persisted across power cycles). */
bool TxUnlock_IsUnlocked(void);

/* TX policy gate: true when tx_hz sits inside a ham band OR the unlock is
 * active. Cheap (table lookup + flag) — safe to call from csdr_apply_tx. */
bool TxUnlock_FreqAllowed(uint32_t tx_hz);

/* Count one out-of-band transmission (RAM only; flushed to flash on the next
 * unlock/relock event and at shutdown — never writes flash in the hot path). */
void TxUnlock_NoteOobTx(void);

/* Read-only access to the live audit record (UI / diagnostics). */
const TxUnlockLog_t *TxUnlock_GetLog(void);

/* Flush the RAM audit record (incl. oob_tx_count) to flash. Blocking flash
 * erase+program — call only from shutdown or a full-screen config context. */
void TxUnlock_PersistCounters(void);

/* Full-screen blocking unlock/status screen (System -> "TX Band Unlock").
 * Shows the audit trace, accepts the secret code, and engages or re-locks the
 * out-of-band TX permission. Pumps CSDR_ProcessAudioPending() every iteration
 * (same pattern as Cal_Run / SWR_Scan_Run) and calls SDR_UI_InvalidateCaches()
 * + SDR_UI_ClearCWText() on exit (full-screen overlay rule). */
void TxUnlock_Run(void);

#ifdef __cplusplus
}
#endif
#endif /* __TX_UNLOCK_H */
