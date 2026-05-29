/* USER CODE BEGIN Header */
/**
  * @file rf_agc.c
  * @brief RF Front-end AGC — PE4302 digital step attenuator control loop.
  *
  * Strategy (HF SDR overload protection):
  *
  *   This module controls the PE4302 RF attenuator placed before the QSD
  *   mixer to prevent ADC saturation from strong nearby signals.  It is
  *   intentionally conservative: it reacts to sustained overload, not to
  *   momentary peaks, and releases slowly to avoid audible pumping on
  *   band-noise variations and CW QSB.
  *
  *   The DSP audio AGC in sdr_dsp.c operates independently after the ADC
  *   and is not modified by this module.
  *
  * Signal measurement:
  *   g_dsp.signal_power_db — baseband IQ RMS (post-FIR, pre-audio-AGC).
  *   Updated every 256-sample DSP batch (~5.3 ms at 48 kHz) with a 0.9/0.1
  *   exponential smoother, providing ~50 ms effective time constant.
  *   This is an appropriate measure of front-end input level.
  *
  * Timing (called from CSDR_Loop, never from ISR):
  *   Update interval : RFAGC_INTERVAL_MS (20 ms)
  *   Attack rate     : RFAGC_ATTACK_STEP_X2 × 0.5 dB per tick = 1.0 dB / 20 ms
  *   Release rate    : RFAGC_RELEASE_STEP_X2 × 0.5 dB per tick = 0.5 dB / 20 ms
  *                     gated by RFAGC_RELEASE_HOLD_MS minimum hold after attack
  *
  * Manual override:
  *   Any CAT or menu-driven manual att change calls RFAGC_NotifyManual(),
  *   which freezes the feedback loop for RFAGC_MANUAL_COOLDOWN_MS (5 s).
  */
/* USER CODE END Header */

#include "rf_agc.h"
#include "pe4302.h"          /* PE4302_MAX_ATTN_X2 */
#include "stm32h7xx_hal.h"   /* HAL_GetTick */

RF_AGC_t g_rfagc;

void RFAGC_Init(RF_AGC_t *agc)
{
    agc->enabled          = false;
    agc->target_x2        = 0U;
    agc->t_last_attack_ms = 0U;
    agc->t_manual_ms      = 0U;     /* no cooldown at boot */
    agc->high_thresh_db   = RFAGC_HIGH_THRESH_DB;
    agc->low_thresh_db    = RFAGC_LOW_THRESH_DB;
}

uint8_t RFAGC_Update(RF_AGC_t *agc, float signal_db, bool tx_mode)
{
    /* Frozen conditions: disabled, transmitting, or in manual-override cooldown */
    if (!agc->enabled || tx_mode) return RFAGC_NO_CHANGE;

    uint32_t now = HAL_GetTick();
    if ((now - agc->t_manual_ms) < RFAGC_MANUAL_COOLDOWN_MS) return RFAGC_NO_CHANGE;

    uint8_t new_x2 = agc->target_x2;

    if (signal_db > agc->high_thresh_db) {
        /* Attack: signal above high threshold → increase attenuation immediately.
         * Step one ATTACK_STEP_X2 per call (caller paces to RFAGC_INTERVAL_MS). */
        uint16_t next = (uint16_t)new_x2 + (uint16_t)RFAGC_ATTACK_STEP_X2;
        new_x2 = (next > (uint16_t)PE4302_MAX_ATTN_X2)
                   ? PE4302_MAX_ATTN_X2
                   : (uint8_t)next;
        agc->t_last_attack_ms = now;

    } else if (signal_db < agc->low_thresh_db) {
        /* Release: signal below low threshold AND hold timer expired.
         * The hold prevents pumping on brief strong signals or CW peaks. */
        if (new_x2 > 0U &&
            (now - agc->t_last_attack_ms) >= RFAGC_RELEASE_HOLD_MS) {
            new_x2 = (new_x2 <= RFAGC_RELEASE_STEP_X2)
                       ? 0U
                       : (uint8_t)(new_x2 - RFAGC_RELEASE_STEP_X2);
        }
    }
    /* Hysteresis: if signal is between thresholds, hold current attenuation. */

    if (new_x2 == agc->target_x2) return RFAGC_NO_CHANGE;
    agc->target_x2 = new_x2;
    return new_x2;
}

void RFAGC_NotifyManual(RF_AGC_t *agc, uint8_t att_x2)
{
    agc->target_x2        = att_x2;
    agc->t_manual_ms      = HAL_GetTick();  /* start cooldown */
    agc->t_last_attack_ms = 0U;             /* reset release hold */
}

void RFAGC_SetEnabled(RF_AGC_t *agc, bool en, uint8_t current_att_x2)
{
    if (en && !agc->enabled) {
        /* Sync target to current hardware value so the first update
         * does not apply a step change from a stale tracked value. */
        agc->target_x2        = current_att_x2;
        agc->t_last_attack_ms = 0U;
    }
    agc->enabled = en;
}

void RFAGC_SetThresholds(RF_AGC_t *agc, float high_db, float low_db)
{
    agc->high_thresh_db = high_db;
    agc->low_thresh_db  = low_db;
}
