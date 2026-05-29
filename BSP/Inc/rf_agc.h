/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rf_agc.h
  * @brief   RF Front-end AGC using PE4302 Digital Step Attenuator
  *
  *  Purpose: overload prevention and dynamic-range extension ONLY.
  *  This is NOT audio-level levelling — the DSP/audio AGC in sdr_dsp.c
  *  is separate and unchanged.
  *
  *  Signal source: g_dsp.signal_power_db (baseband IQ RMS, post-FIR,
  *  pre-audio-AGC).  This reflects the actual RF input level through the
  *  QSD mixer and WM8731 ADC.
  *
  *  Attack/Release strategy (HF SDR):
  *   Attack  – fast enough to prevent ADC saturation on sudden strong signals
  *   Release – much slower (200 ms hold + gradual step) to avoid pumping
  *             on band-noise transitions and CW QSB
  *
  *  Manual override: any CAT/menu att change starts a cooldown period
  *  (RFAGC_MANUAL_COOLDOWN_MS) during which the RF AGC is frozen.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __RF_AGC_H
#define __RF_AGC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Threshold defaults (dBFS, post-FIR baseband IQ power) ──────────────────
 *  The signal scale: S9 ≈ −46 dBFS, S9+20 ≈ −26 dBFS, S9+40 ≈ −6 dBFS.
 *  HIGH_THRESH: attack region — signals this strong risk ADC saturation.
 *  LOW_THRESH:  release region — below this there is no front-end overload risk.
 *  The gap between the two thresholds is the hysteresis band.
 * ─────────────────────────────────────────────────────────────────────────── */
#define RFAGC_HIGH_THRESH_DB    (-10.0f)   /* ≈ S9+40 dB → increase attenuation */
#define RFAGC_LOW_THRESH_DB     (-30.0f)   /* ≈ S9+16 dB → release attenuation  */

/* ── Step sizes (PE4302 register units: 1 unit = 0.5 dB) ───────────────────
 *  Attack 2 = 1.0 dB/tick, Release 1 = 0.5 dB/tick
 *  At RFAGC_INTERVAL_MS = 20 ms: attack rate = 50 dB/s, release rate = 25 dB/s
 *                                (release rate limited further by HOLD below) */
#define RFAGC_ATTACK_STEP_X2    2U         /* 1.0 dB per attack tick  */
#define RFAGC_RELEASE_STEP_X2   1U         /* 0.5 dB per release tick */

/* ── Timing ─────────────────────────────────────────────────────────────────
 *  INTERVAL:     main loop call period; set timer in CSDR_Loop accordingly.
 *  RELEASE_HOLD: minimum time after last attack before any release step.
 *                Prevents pumping — a brief strong signal causes the
 *                attenuator to hold for at least this long before releasing. */
#define RFAGC_INTERVAL_MS       20U        /* update period (ms)                */
#define RFAGC_RELEASE_HOLD_MS   200U       /* hold after attack before release  */

/* ── Manual-override cooldown ───────────────────────────────────────────────
 *  After any CAT/menu manual att change the RF AGC freezes for this duration
 *  so the user's setting is respected without fighting the feedback loop. */
#define RFAGC_MANUAL_COOLDOWN_MS  5000U    /* 5 seconds                         */

/* ── Return sentinel ────────────────────────────────────────────────────────
 *  RFAGC_Update returns this when attenuation is unchanged (no SPI write). */
#define RFAGC_NO_CHANGE         0xFFU

/* ── State ──────────────────────────────────────────────────────────────────*/
typedef struct {
    bool     enabled;            /*!< RF AGC active flag                      */
    uint8_t  target_x2;         /*!< Tracked PE4302 register value (0–63)    */
    uint32_t t_last_attack_ms;  /*!< Tick of last attack step (for hold timer)*/
    uint32_t t_manual_ms;       /*!< Tick of last manual override             */
    float    high_thresh_db;    /*!< Attack threshold (dBFS)                  */
    float    low_thresh_db;     /*!< Release threshold (dBFS)                 */
} RF_AGC_t;

extern RF_AGC_t g_rfagc;

/* ── API ─────────────────────────────────────────────────────────────────── */

/** Initialise RF AGC state. Call once in CSDR_Init after PE4302_Init. */
void    RFAGC_Init(RF_AGC_t *agc);

/**
  * @brief  Main update — call from CSDR_Loop RX periodic task every
  *         RFAGC_INTERVAL_MS milliseconds.  Never call from an ISR.
  * @param  agc        RF AGC state handle
  * @param  signal_db  Current DSP baseband power (g_dsp.signal_power_db)
  * @param  tx_mode    True when transmitting — freezes AGC during TX
  * @return New att_x2 value (0–63) if attenuation changed, RFAGC_NO_CHANGE
  *         if unchanged.  Caller writes the returned value to PE4302_SetAttn_Raw.
  */
uint8_t RFAGC_Update(RF_AGC_t *agc, float signal_db, bool tx_mode);

/**
  * @brief  Notify RF AGC that a manual attenuation change was applied.
  *         Syncs internal tracked value and starts the manual-override cooldown.
  *         Call whenever CAT/menu sets g_sdr.att_db directly.
  * @param  att_x2  New PE4302 register value (= att_dB × 2)
  */
void    RFAGC_NotifyManual(RF_AGC_t *agc, uint8_t att_x2);

/**
  * @brief  Enable or disable RF AGC.
  *         On disable, syncs tracked target to current hardware value so
  *         re-enabling starts smoothly without a step change.
  * @param  current_att_x2  Current PE4302 register value (g_att.current_atten_x2)
  */
void    RFAGC_SetEnabled(RF_AGC_t *agc, bool en, uint8_t current_att_x2);

/** Override attack/release thresholds at runtime. */
void    RFAGC_SetThresholds(RF_AGC_t *agc, float high_db, float low_db);

#ifdef __cplusplus
}
#endif
#endif /* __RF_AGC_H */
