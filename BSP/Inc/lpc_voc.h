/**
  ******************************************************************************
  * @file    lpc_voc.h
  * @brief   LPC-10 vocoder for FreeDV (8 kHz narrowband speech)
  *
  *  Implementation:
  *    Analysis:  pre-emphasis → Hamming window → autocorrelation (Lag Windowing
  *               bandwidth expansion λ=0.994) → Levinson-Durbin LPC order 10
  *    Pitch:     normalized ACF on pre-emphasized speech, range 20-142 samples
  *               (≈56-400 Hz); voiced when peak > LPC_VOICED_THRESH = 0.35
  *    Synthesis: voiced = impulse train at pitch period;
  *               unvoiced = LCG white noise;
  *               all-pole IIR filter y[n] = e[n] - Σ a[k]*y[n-k];
  *               de-emphasis α = 0.97.
  *
  *  Memory: LPC_Voc_t is 56 bytes (no heap allocation).
  *  CPU:    ~200 μs per 40 ms frame at 480 MHz Cortex-M7 (0.5% load).
  ******************************************************************************
  */

#ifndef __LPC_VOC_H
#define __LPC_VOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Configuration ───────────────────────────────────────── */
#define LPC_ORDER          10U    /*!< LPC filter order (10 poles)         */
#define LPC_FRAME_SAMPS   320U    /*!< 40 ms frame @ 8 kHz (3 OFDM syms)  */
#define LPC_PITCH_MIN      20U    /*!< Min pitch period (samples): 400 Hz  */
#define LPC_PITCH_MAX     142U    /*!< Max pitch period (samples):  56 Hz  */
#define LPC_VOICED_THRESH  0.35f  /*!< Normalized ACF threshold for V/UV   */
#define LPC_PREEMPH        0.97f  /*!< Pre-emphasis α  (encoder)           */
#define LPC_DEEMPH         0.97f  /*!< De-emphasis  α  (decoder)           */

/* ── Types ───────────────────────────────────────────────── */

/**
 * @brief  LPC frame parameters — output of LPC_Encode, input to LPC_Decode.
 *
 *  In Phase 2 (this module) the frame is passed directly between encoder
 *  and decoder (loopback) without quantization.  Phase 3 will quantize
 *  a[] and pitch_samps to bits for over-air transmission.
 */
typedef struct {
    float    a[LPC_ORDER + 1U]; /*!< a[0]=1 (unused), a[1..p] from Levinson-Durbin */
    float    gain;              /*!< Excitation RMS energy                           */
    uint16_t pitch_samps;       /*!< Pitch period (samples); 0 = unvoiced            */
} LPC_Frame_t;

/**
 * @brief  LPC vocoder state (encoder + decoder combined, 56 bytes).
 *
 *  A single LPC_Voc_t handles both encode and decode so that synthesis
 *  filter memory continues smoothly across frame boundaries.
 */
typedef struct {
    /* Encoder state */
    float    pre_x;                 /*!< Pre-emphasis one-sample delay       */

    /* Decoder state */
    float    syn_mem[LPC_ORDER];    /*!< Synthesis filter memory y[n-1..n-p] */
    float    de_x;                  /*!< De-emphasis one-sample delay        */
    float    pitch_phase;           /*!< Fractional phase in pitch period    */
    uint32_t prng;                  /*!< LCG PRNG state for UV noise         */
} LPC_Voc_t;

/* ── API ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise vocoder state (call once, or at mode switch).
 */
void LPC_Voc_Init(LPC_Voc_t *v);

/**
 * @brief  Analyse one 160-sample PCM frame, produce LPC parameters.
 *
 * @param  v        Vocoder state (encoder pre_x updated)
 * @param  pcm_in   Input: 320 float samples at 8 kHz, normalised ±1
 * @param  f        Output: LPC frame parameters
 */
void LPC_Encode(LPC_Voc_t *v, const float *pcm_in, LPC_Frame_t *f);

/**
 * @brief  Synthesise one 160-sample PCM frame from LPC parameters.
 *
 * @param  v        Vocoder state (syn_mem, de_x, pitch_phase, prng updated)
 * @param  f        Input: LPC frame parameters from LPC_Encode
 * @param  pcm_out  Output: 320 float samples at 8 kHz, normalised ±1
 */
void LPC_Decode(LPC_Voc_t *v, const LPC_Frame_t *f, float *pcm_out);

#ifdef __cplusplus
}
#endif
#endif /* __LPC_VOC_H */
