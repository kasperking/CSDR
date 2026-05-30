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
  *      → TX drive control (multiplied into g_dsp.tx.audio_gain via csdr_apply_tx)
  *
  *  This module owns the protection decision only.
  *  It does NOT touch hardware directly — it sets flags consumed by csdr_apply_tx().
  *
  *  Integration checklist:
  *    1. PA_Protect_Init()     — call once in CSDR_Init(), after PA_OC_Init().
  *    2. PA_Protect_Update()   — call every 20 ms in CSDR_Loop (dedicated timer).
  *    3. csdr_apply_tx()       — guard + PA_Protect_OnTxStart/Stop + drive limit.
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
  * @brief Current drive limit percentage to apply to audio gain.
  *        Returns 100 in NORMAL, 75/50/25 in FOLDBACK/LIMIT, 0 in TRIP/COOLDOWN.
  *        Multiply this into the computed audio_gain in csdr_apply_tx().
  */
uint8_t PA_Protect_GetDriveLimit(void);

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
