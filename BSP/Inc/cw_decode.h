#ifndef __CW_DECODE_H
#define __CW_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Maximum decoded characters held in the ring buffer */
#define CWDEC_TEXT_LEN  64U

/* Edge-event ring depth (power of two).  Sized for the worst main-loop
 * stall: ~100 ms EEPROM save at 40 WPM ≈ 4 edges; 16 leaves 4× margin. */
#define CWDEC_EDGE_RING 16U

/* ── CWEnv_t – per-sample envelope state (lives in DSP_State_t) ─────────────
 * Updated inside DSP_Process; consumed by CWDec_Update.  Both run in
 * main-loop context (csdr_process_audio_pending / CSDR_Loop) — no ISR is
 * involved, so the edge ring needs no locking.
 * Tap point: I²+Q² magnitude-squared of the band-limited IQ signal, pre-AGC.
 * No sqrt required — thresholding is done in the squared domain.
 *
 * Keying edges are timestamped with a 48 kHz sample counter and queued in a
 * small ring, so element/space durations are exact even when the main loop
 * stalls (EEPROM save, menu redraw) and several audio blocks are processed
 * in one burst.
 */
typedef struct {
    float    env_sq;        /* IIR envelope power (asymmetric attack/release) */
    float    floor_sq;      /* Adaptive noise floor (fast-down / slow-up IIR) */
    float    alpha_r;       /* Attack coeff  (~0.5 ms)                        */
    float    alpha_f;       /* Release coeff (~5 ms)                          */
    float    alpha_nd;      /* Floor fall coeff (~20 ms)                      */
    float    alpha_nu;      /* Floor rise coeff (~400 ms, key up)             */
    float    alpha_nk;      /* Floor rise coeff (~2 s, key DOWN — the floor
                             * must track noise, not learn the signal)        */
    uint8_t  keyed;         /* Current debounced keying state 0/1             */

    /* Debounce: a threshold crossing must persist ~3 ms before it becomes
     * an edge, killing both ON-blips and mid-element dropouts at the
     * source.  Committed edges are back-dated so durations stay exact. */
    uint16_t dbn_len;       /* Samples required to accept a new state         */
    uint16_t dbn_ctr;       /* Consecutive samples in the candidate state     */

    /* Sample-domain clock + edge-event ring */
    uint32_t samps_per_ms;  /* sample_rate / 1000 (48 at 48 kHz)              */
    uint32_t sample_clock;  /* Samples processed in CW mode (wraps ~24 h)     */
    uint32_t edge_time[CWDEC_EDGE_RING];   /* sample_clock at transition      */
    uint8_t  edge_state[CWDEC_EDGE_RING];  /* new keyed state 0/1             */
    uint8_t  edge_wr;       /* Producer index (DSP_Process)                   */
    uint8_t  edge_rd;       /* Consumer index (CWDec_Update)                  */
} CWEnv_t;

/* Producer side: record a keying transition at sample time t.  Call from
 * the DSP sample loop once a debounced transition is accepted.  When the
 * ring is full the event is dropped; CWDec_Update resynchronises from
 * env->keyed. */
static inline void CWEnv_PushEdge(CWEnv_t *env, uint8_t new_state, uint32_t t)
{
    uint8_t nxt = (uint8_t)((env->edge_wr + 1U) & (CWDEC_EDGE_RING - 1U));
    if (nxt != env->edge_rd) {
        env->edge_time[env->edge_wr]  = t;
        env->edge_state[env->edge_wr] = new_state;
        env->edge_wr = nxt;
    }
    env->keyed = new_state;
}

/* ── CWDec_t – main-loop timing decoder state ───────────────────────────── */
typedef struct {
    /* Timing (sample-domain, see CWEnv_t.sample_clock) */
    bool     prev_keyed;    /* Keying state after last consumed edge         */
    uint32_t rise_samp;     /* sample_clock at last rising edge              */
    uint32_t fall_samp;     /* sample_clock at end of last accepted element  */
    uint32_t dit_ms;        /* Adaptive dit estimate (ms); init = 60 = 20WPM */

    /* Deferred element: a falling edge is committed only once the gap that
     * follows proves real (≥ dit/4).  A shorter gap is a QSB dropout — the
     * next rising edge re-opens the same element instead of splitting it. */
    bool     el_pending;    /* Uncommitted element exists                     */
    uint32_t pend_rise;     /* Its start (sample_clock)                       */
    uint32_t pend_fall;     /* Its end   (sample_clock)                       */

    /* Morse accumulator */
    uint8_t  mbits;         /* Sentinel (1) + element bits; index 1..63      */
    bool     char_decoded;  /* One-shot: char already emitted for this space  */
    bool     word_done;     /* One-shot: word-space already emitted           */

    /* Text ring buffer */
    char     text[CWDEC_TEXT_LEN];
    uint8_t  head;          /* Write index (wraps at CWDEC_TEXT_LEN)         */
    uint8_t  count;         /* Chars in ring; saturates at CWDEC_TEXT_LEN    */

    /* Debug counters */
    uint32_t n_chars;
    uint32_t n_unknown;
} CWDec_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/* Initialise CWEnv_t coefficients for a given sample rate */
void CWEnv_Init(CWEnv_t *env, uint32_t sample_rate);

/* Initialise decoder state; call once and on mode reset */
void CWDec_Init(CWDec_t *cw);

/* Reset the timing/accumulator state and drain queued edges (text ring and
 * the adaptive dit estimate are preserved; re-seed dit_ms explicitly when
 * the menu WPM changes). */
void CWDec_Reset(CWDec_t *cw, CWEnv_t *env);

/* Main-loop update: consume queued keying edges, advance timing state.
 * Returns true when the text buffer changed (new char or word space added). */
bool CWDec_Update(CWDec_t *cw, CWEnv_t *env);

/* Copy the last max_chars decoded characters (null-terminated) into buf.
 * Characters are ordered oldest-to-newest.                              */
void CWDec_GetText(const CWDec_t *cw, char *buf, uint8_t max_chars);

#ifdef __cplusplus
}
#endif
#endif /* __CW_DECODE_H */
