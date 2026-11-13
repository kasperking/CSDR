/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pa_protect.h
  * @brief   Centralized PA Protection Manager
  *
  *  Architecture:
  *
  *    Sensors (INA226 / NTC / SWR)
  *      → IIR filtering   (α=0.1 SWR+temp, α=0.3 current)
  *      → State machine    (NORMAL → FOLDBACK → LIMIT → TRIP → COOLDOWN)
  *      → Power policy     (drive_limit 100/75/50/25/0 %)
  *      → TX drive control (multiplied into g_dsp.tx.drive_gain via csdr_apply_tx)
  *
  *  External ALC (PA0 / ADC1_INP16 — PA feedback voltage):
  *      PA_Protect_FastAlcTick(), 1 ms: fast-attack / slow-release envelope
  *      → continuous proportional reduction above ALC_THRESHOLD_PCT input
  *      → PA_Protect_GetALCDrive() 0-100 multiplier, forwarded straight to
  *        g_dsp.tx.ext_alc_gain (NOT via csdr_apply_tx)
  *      → enabled by g_sdr.ext_alc_on AND g_sdr.ext_pa_on; fixed profile:
  *        Attack  τ ≈ 1.4 ms (α=0.5,   1 ms sample)
  *        Release τ ≈ 500 ms (α=0.002, 1 ms sample)
  *        Threshold: 10% input  →  drive starts reducing
  *                   (−0.44 V amp ALC with R108 = 150k, see pa_protect.c)
  *        Maximum reduction:    →  30% minimum drive
  *
  *  Power ALC (tandem-match forward power — closed loop; only CW / FM /
  *  DIGU / DIGL / TUNE — SSB/AM hold 100 %, see power_alc_mode_ok()):
  *      target_mw = pa_watts × tx_power%   (same watt figure as RF Power menu)
  *      peak-hold envelope (fast-attack/slow-release) of g_analog.fwd_power_mw
  *      → slow proportional corrector (±1 %-pt / 20 ms tick, clamped 50..150%)
  *      → PA_Protect_GetPowerALCDrive() multiplier (multiplied independently
  *        of stepped foldback and external ALC in csdr_apply_tx)
  *      Compensates band-to-band PA gain variation and supply-voltage sag so
  *      the configured watt figure is what actually leaves the antenna jack.
  *      Resets to 100% at the start of every transmission; holds (does not
  *      drift) during SSB syllable gaps; no separate enable flag — runs
  *      whenever pa_watts > 0 since it reuses the existing SWR sensor.
  *      External-ALC precedence: while the external ALC is cutting drive
  *      (GetALCDrive() < 100) the corrector may hold or step down but never
  *      boost — low forward power is then intentional, and compensating it
  *      would fight the external amplifier's ALC loop.
  *
  *  This module owns the protection decision only.
  *  It does NOT touch hardware directly — it sets flags consumed by csdr_apply_tx().
  *
  *  Integration checklist:
  *    1. PA_Protect_Init()     — call once in CSDR_Init(), after PA_OC_Init().
  *    2. PA_Protect_Update()   — call every 20 ms in CSDR_Loop (dedicated timer).
  *    3. csdr_apply_tx()       — guard + PA_Protect_OnTxStart/Stop + drive limits.
  *
  *  Protection does NOT depend on UI refresh rate.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PA_PROTECT_H
#define __PA_PROTECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ─── State machine ──────────────────────────────────────────────────────── */

typedef enum {
    PA_STATE_NORMAL,    /*!< Full TX power allowed                             */
    PA_STATE_FOLDBACK,  /*!< Soft power reduction in progress: 75% → 50%      */
    PA_STATE_LIMIT,     /*!< Maximum foldback sustained: 25%                   */
    PA_STATE_TRIP,      /*!< TX disabled, fault latched (transient; ~20 ms)   */
    PA_STATE_COOLDOWN   /*!< TX blocked; waiting for conditions to recover     */
} PA_State_t;

/* ─── Fault reason (priority-ordered: higher = worse) ───────────────────── */

typedef enum {
    PA_FAULT_NONE,
    PA_FAULT_OVERCURRENT,   /*!< INA226 filtered current exceeded threshold   */
    PA_FAULT_OVERTEMP,      /*!< NTC temperature exceeded threshold            */
    PA_FAULT_HIGH_SWR       /*!< Reflected power / antenna mismatch           */
} PA_Fault_t;

/* ─── Configurable thresholds ────────────────────────────────────────────── */

typedef struct {
    float    swr_warn;           /*!< SWR warning level        (default 2.0)  */
    float    swr_trip;           /*!< SWR hard trip level       (default 4.0)  */
    int16_t  temp_warn_c10;      /*!< Warn temp °C×10          (default 750)   */
    int16_t  temp_trip_c10;      /*!< Trip temp °C×10          (default 900)   */
    int16_t  temp_recover_c10;   /*!< Cooldown recovery °C×10  (default 700)   */
    float    current_warn_a;     /*!< Soft warn current A      (default 3.0)   */
    float    current_trip_a;     /*!< Hard trip current A      (default 4.0)   */
    uint32_t cooldown_ms;        /*!< Max cooldown timeout ms  (default 30000) */
} PA_Protect_Config_t;

extern PA_Protect_Config_t g_pa_cfg;

/* ─── API ────────────────────────────────────────────────────────────────── */

/** @brief One-time init. Call in CSDR_Init() after PA_OC_Init(). */
void PA_Protect_Init(void);

/**
  * @brief Periodic protection update. Call every 20 ms from CSDR_Loop.
  *        Runs IIR filters, evaluates thresholds, drives state machine.
  *        Must NOT be called from an ISR.
  */
void PA_Protect_Update(void);

/**
  * @brief External-amp ALC fast path: sample PA0 + attack/release envelope.
  *        Call every 1 ms from the main loop (csdr_fast_alc_tick does this
  *        and forwards GetALCDrive() into g_dsp.tx.ext_alc_gain).
  *        Active only when ext_alc_on && ext_pa_on; ADC read only while TX.
  */
void PA_Protect_FastAlcTick(void);

/**
  * @brief Notify that TX is starting.
  *        Seeds IIR filters from current readings to suppress TX-attack transient.
  *        Call from csdr_apply_tx() on the TX path.
  */
void PA_Protect_OnTxStart(void);

/**
  * @brief Notify that TX has stopped.
  *        Resets foldback state on voluntary TX→RX transition.
  *        Call from csdr_apply_tx() on the RX path.
  */
void PA_Protect_OnTxStop(void);

/**
  * @brief Returns false during TRIP or COOLDOWN — TX must be refused.
  *        Call in csdr_apply_tx() as a gate before enabling the T/R relay.
  */
bool PA_Protect_IsTxAllowed(void);

/**
  * @brief Stepped drive limit from PA-protect state machine.
  *        Returns 100 in NORMAL, 75/50/25 in FOLDBACK/LIMIT, 0 in TRIP/COOLDOWN.
  *        Multiply this into the computed audio_gain in csdr_apply_tx().
  */
uint8_t PA_Protect_GetDriveLimit(void);

/**
  * @brief Continuous ALC drive limit from external PA feedback (PA0/ADC1_INP16).
  *        Returns 100 when g_sdr.ext_alc_on=false or ALC input < ALC_THRESHOLD_PCT (10%).
  *        Returns 30..100 proportional reduction when input exceeds threshold.
  *        Multiply independently into csdr_apply_tx() alongside GetDriveLimit().
  */
uint8_t PA_Protect_GetALCDrive(void);

/**
  * @brief Continuous closed-loop drive corrector from tandem-match forward
  *        power (g_analog.fwd_power_mw) vs target = pa_watts × tx_power%.
  *        Returns 100 at rest; 50..150 while actively correcting under/over
  *        drive so measured output tracks the configured watt figure.
  *        Always active when g_sdr.pa_watts > 0 — no enable flag, reuses
  *        the SWR sensor hardware. Multiply independently into
  *        csdr_apply_tx() alongside the other drive multipliers.
  */
uint8_t PA_Protect_GetPowerALCDrive(void);

/**
  * @brief Smoothed (peak-hold envelope) forward power in milliwatts — the
  *        same signal the Power ALC corrector uses internally. For UI
  *        display of actual transmitted power (vs the configured target).
  *        Returns 0 when g_sdr.pa_watts == 0 (no PA fitted) or not transmitting.
  */
uint32_t PA_Protect_GetFwdPowerEnvelope_mW(void);

/**
  * @brief IIR-filtered SWR ×100 (100 = 1.00, clamped 100-2000) — the same
  *        figure the protection state machine acts on.  Returns 100 when not
  *        transmitting, no PA fitted, or the PA sensor is missing, so meter
  *        consumers (CAT RM1) rest at zero deflection.  Read-only; safe to
  *        call from the main loop (same context as PA_Protect_Update).
  */
uint16_t PA_Protect_GetSwrX100(void);

/**
  * @brief External-ALC drive reduction in %-points (0 = ALC idle, 70 = max
  *        reduction, i.e. 100 − GetALCDrive()).  Returns 0 when not
  *        transmitting or g_sdr.ext_alc_on is off, so meter consumers
  *        (CAT RM3) rest at zero deflection.  Read-only, main-loop safe.
  */
uint8_t PA_Protect_GetAlcReductionPct(void);

/** @brief Current protection state (for UI display). */
PA_State_t PA_Protect_GetState(void);

/** @brief Active fault reason (PA_FAULT_NONE when healthy). */
PA_Fault_t PA_Protect_GetFault(void);

/**
  * @brief Force exit from COOLDOWN (e.g. long-press to acknowledge).
  *        Only acts when TX is off and state is COOLDOWN.
  */
void PA_Protect_ManualReset(void);

#ifdef __cplusplus
}
#endif
#endif /* __PA_PROTECT_H */
