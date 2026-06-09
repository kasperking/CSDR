#ifndef __CW_DECODE_H
#define __CW_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Maximum decoded characters held in the ring buffer */
#define CWDEC_TEXT_LEN  64U

/* ── CWEnv_t – per-sample envelope state (lives in DSP_State_t) ─────────────
 * Updated inside DSP_Process (main-loop context); `keyed` read by CWDec_Update.
 * Tap point: I²+Q² magnitude-squared of the band-limited IQ signal, pre-AGC.
 * No sqrt required — thresholding is done in the squared domain.
 */
typedef struct {
    float    env_sq;        /* IIR envelope power (asymmetric attack/release) */
    float    floor_sq;      /* Adaptive noise floor (fast-down / slow-up IIR) */
    float    alpha_r;       /* Attack coeff  (~0.5 ms)                        */
    float    alpha_f;       /* Release coeff (~5 ms)                          */
    float    alpha_nd;      /* Floor fall coeff (~20 ms)                      */
    float    alpha_nu;      /* Floor rise coeff (~400 ms)                     */
    volatile uint8_t keyed; /* Current keying state 0/1 (written by DSP_Process) */
} CWEnv_t;

/* ── CWDec_t – main-loop timing decoder state ───────────────────────────── */
typedef struct {
    /* Timing */
    bool     prev_keyed;    /* Keying state at last CWDec_Update call        */
    uint32_t edge_ms;       /* HAL_GetTick() at last edge                    */
    uint32_t dit_ms;        /* Adaptive dit estimate (ms); init = 60 = 20WPM */

    /* Morse accumulator */
    uint8_t  mbits;         /* Sentinel (1) + element bits; index 1..63      */
    bool     char_decoded;  /* One-shot: char already emitted for this space  */
    bool     word_done;     /* One-shot: word-space already emitted           */

    /* Text ring buffer */
    char     text[CWDEC_TEXT_LEN];
    uint8_t  head;          /* Write index (wraps at CWDEC_TEXT_LEN)         */
    uint8_t  count;         /* Total chars written so far                    */

    /* Debug counters */
    uint32_t n_chars;
    uint32_t n_unknown;
} CWDec_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/* Initialise CWEnv_t coefficients for a given sample rate */
void CWEnv_Init(CWEnv_t *env, uint32_t sample_rate);

/* Initialise decoder state; call once and on mode reset */
void CWDec_Init(CWDec_t *cw);

/* Set decoder speed from WPM; updates dit_ms = 1200/wpm (standard formula).
 * Does not reset the timing state so a speed change mid-QSO is smooth. */
void CWDec_SetWPM(CWDec_t *cw, uint8_t wpm);

/* Reset only the timing/accumulator state (text ring is preserved) */
void CWDec_Reset(CWDec_t *cw);

/* Main-loop update: feed current keyed state, advance timing state machine.
 * Returns true when the text buffer changed (new char or word space added). */
bool CWDec_Update(CWDec_t *cw, uint8_t keyed);

/* Copy the last max_chars decoded characters (null-terminated) into buf.
 * Characters are ordered oldest-to-newest.                              */
void CWDec_GetText(const CWDec_t *cw, char *buf, uint8_t max_chars);

#ifdef __cplusplus
}
#endif
#endif /* __CW_DECODE_H */
