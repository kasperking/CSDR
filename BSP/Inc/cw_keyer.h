/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cw_keyer.h
  * @brief   CW TX keyer — straight key / iambic A / iambic B + sidetone NCO
  *
  *  Hardware:  DIT = PB13 (active-low), DAH = PB14 (active-low)
  *
  *  BK-IN modes:
  *   Off  – keyer generates sidetone only; TX controlled by PTT/CAT.
  *   Semi – key-down asserts TX; key-up + bk_delay_ms → RX.
  *   Full – key-down asserts TX; key-up + 1 dit → RX (fast break-in).
  *
  *  Sidetone: NCO at pitch_hz injected into the audio output buffer.
  *            Amplitude ramps up/down over ~3 ms to suppress key clicks.
  *
  *  Integration:
  *   1. Call CW_Keyer_Init() once in CSDR_Init, passing GPIO and set_tx callback.
  *   2. Call CW_Keyer_SetConfig() whenever settings change.
  *   3. Call CW_Keyer_Update()  every CSDR_Loop iteration (main-loop context).
  *   4. Call CW_Keyer_Sidetone() after each DSP_Process block to mix sidetone
  *      into the audio output buffer (int32_t, 24-bit I2S samples).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CW_KEYER_H
#define __CW_KEYER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Keyer mode ──────────────────────────────────────────────────────────── */
typedef enum {
    CW_KEYER_STRAIGHT = 0,   /*!< Straight key on DIT pin        */
    CW_KEYER_IAMBIC_A = 1,   /*!< Iambic mode A (no memory)      */
    CW_KEYER_IAMBIC_B = 2,   /*!< Iambic mode B (Curtis, memory) */
} CW_KeyerMode_t;

/* ── BK-IN mode ──────────────────────────────────────────────────────────── */
typedef enum {
    CW_BKIN_OFF  = 0,   /*!< No auto T/R; PTT/CAT only      */
    CW_BKIN_SEMI = 1,   /*!< TX on key-down; RX after delay */
    CW_BKIN_FULL = 2,   /*!< Fast break-in (1-dit delay)    */
} CW_BKInMode_t;

/* ── Internal iambic states (not public API) ─────────────────────────────── */
typedef enum {
    IK_IDLE      = 0,
    IK_DIT       = 1,
    IK_DIT_SPACE = 2,
    IK_DAH       = 3,
    IK_DAH_SPACE = 4,
} IambicState_t;

/* ── Keyer instance ──────────────────────────────────────────────────────── */
typedef struct {
    /* ── Config ──────────────────────────────────────────────────────────── */
    CW_KeyerMode_t  mode;
    bool            paddle_rev;      /*!< Swap DIT↔DAH paddles            */
    uint32_t        dit_ms;          /*!< Dit duration ms = 1200/wpm      */
    uint16_t        pitch_hz;        /*!< Sidetone pitch Hz (300-900)     */
    uint8_t         sidetone_vol;    /*!< Sidetone volume 0-100           */
    CW_BKInMode_t   bkin_mode;
    uint16_t        bk_delay_ms;     /*!< Delay before auto-RX (Semi/Full)*/

    /* ── GPIO ────────────────────────────────────────────────────────────── */
    GPIO_TypeDef   *dit_port;
    uint16_t        dit_pin;
    GPIO_TypeDef   *dah_port;
    uint16_t        dah_pin;

    /* ── Keying output ───────────────────────────────────────────────────── */
    bool            key_down;        /*!< Current RF key state            */
    bool            tx_active;       /*!< TX relay is currently engaged   */

    /* ── Straight key state ─────────────────────────────────────────────── */
    bool            sk_prev;

    /* ── Iambic state machine ───────────────────────────────────────────── */
    IambicState_t   ik_state;
    uint32_t        ik_end_ms;       /*!< Tick when current element ends  */
    bool            dit_mem;         /*!< Queued dit (Iambic B memory)    */
    bool            dah_mem;         /*!< Queued dah (Iambic B memory)    */

    /* ── BK-IN timing ───────────────────────────────────────────────────── */
    uint32_t        last_key_ms;     /*!< Tick of last key-up event       */
    bool            bkin_tx_pending; /*!< TX was asserted by keyer        */

    /* ── Sidetone NCO ───────────────────────────────────────────────────── */
    uint32_t        st_phase_acc;
    uint32_t        st_phase_inc;    /*!< Recomputed by SetConfig         */
    float           st_env;          /*!< Envelope smoother (click-free)  */
    float           st_alpha_on;     /*!< Attack coeff  (~3 ms)           */
    float           st_alpha_off;    /*!< Release coeff (~5 ms)           */

    /* ── TX callback ────────────────────────────────────────────────────── */
    void           (*set_tx)(bool tx);
} CW_Keyer_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * @brief  One-time init.  GPIO pins must already be configured as inputs
 *         with pull-up (active-low).  set_tx is called whenever BK-IN logic
 *         requests a T/R state change; pass NULL to disable auto T/R.
 */
void CW_Keyer_Init(CW_Keyer_t *k,
                   GPIO_TypeDef *dit_port, uint16_t dit_pin,
                   GPIO_TypeDef *dah_port, uint16_t dah_pin,
                   void (*set_tx)(bool tx));

/**
 * @brief  Apply new settings.  Call whenever g_sdr CW fields change.
 * @param  sample_rate  Audio sample rate (used to compute sidetone NCO coeff)
 */
void CW_Keyer_SetConfig(CW_Keyer_t *k,
                        CW_KeyerMode_t mode, bool paddle_rev,
                        uint8_t wpm, uint16_t pitch_hz,
                        uint8_t sidetone_vol,
                        CW_BKInMode_t bkin_mode, uint16_t bk_delay_ms,
                        uint32_t sample_rate);

/**
 * @brief  Main-loop update.  Polls GPIO, advances state machine, handles BK-IN.
 *         Call every CSDR_Loop iteration.
 */
void CW_Keyer_Update(CW_Keyer_t *k);

/**
 * @brief  Mix sidetone into audio output buffer.
 *         Call after DSP_Process; adds NCO tone at pitch_hz (scaled by
 *         sidetone_vol) to each sample in buf[0..len-1] when key is down.
 * @param  buf         int32_t audio buffer (24-bit I2S samples, one channel)
 * @param  len         Number of samples
 * @param  sample_rate Current audio sample rate
 */
void CW_Keyer_Sidetone(CW_Keyer_t *k, int32_t *buf, uint32_t len,
                       uint32_t sample_rate);

#ifdef __cplusplus
}
#endif
#endif /* __CW_KEYER_H */
