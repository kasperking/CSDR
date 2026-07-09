#include "cw_decode.h"
#include <math.h>
#include <string.h>

/* ── Morse code lookup table ────────────────────────────────────────────────
 * Index = sentinel bit (1) followed by element bits: dit=0, dah=1.
 * Example: 'A' = .-  →  bits 01  →  index = 0b101 = 5
 * Valid range: indices 2..63 (1- to 5-element codes).
 * 0 = unknown/unused code.
 */
static const char cw_table[64] = {
    /*  0      1  */
    0,    0,
    /* 2-element (1 bit) */
    'E',  'T',
    /* 4-element (2 bits) */
    'I',  'A',  'N',  'M',
    /* 8-element (3 bits) */
    'S',  'U',  'R',  'W',  'D',  'K',  'G',  'O',
    /* 16-element (4 bits) */
    'H',  'V',  'F',   0,   'L',   0,   'P',  'J',
    'B',  'X',  'C',  'Y',  'Z',  'Q',   0,    0,
    /* 32-element (5 bits) */
    '5',  '4',   0,   '3',   0,    0,    0,   '2',
     0,    0,   '+',   0,    0,    0,    0,   '1',
    '6',  '=',  '/',   0,    0,    0,    0,    0,
    '7',   0,    0,    0,   '8',   0,   '9',  '0',
};

/* Coefficients: α = exp(-1 / (τ_samples)) ≈ exp(-1 / (τ_ms * Fs / 1000)) */
void CWEnv_Init(CWEnv_t *env, uint32_t sample_rate)
{
    float sr = (float)sample_rate;
    env->alpha_r  = expf(-1.0f / (0.0005f * sr));  /* 0.5 ms attack  */
    env->alpha_f  = expf(-1.0f / (0.005f  * sr));  /* 5 ms   release */
    env->alpha_nd = expf(-1.0f / (0.020f  * sr));  /* 20 ms  floor fall */
    env->alpha_nu = expf(-1.0f / (0.400f  * sr));  /* 400 ms floor rise (key up) */
    env->alpha_nk = expf(-1.0f / (2.000f  * sr));  /* 2 s    floor rise (key down) */
    env->env_sq   = 0.0f;
    /* Start HIGH and settle downward (20 ms fall): keyed stays 0 while the
     * floor converges onto the true band noise, so mode entry is silent
     * instead of spraying garbage until the floor climbs up. */
    env->floor_sq = 1.0f;
    env->keyed    = 0U;
    env->samps_per_ms = (sample_rate >= 1000U) ? (sample_rate / 1000U) : 1U;
    env->dbn_len  = (uint16_t)(3U * env->samps_per_ms);   /* 3 ms */
    env->dbn_ctr  = 0U;
    env->sample_clock = 0U;
    env->edge_wr  = 0U;
    env->edge_rd  = 0U;
}

void CWDec_Init(CWDec_t *cw)
{
    memset(cw, 0, sizeof(CWDec_t));
    cw->dit_ms = 60U;   /* 20 WPM default */
    cw->mbits  = 1U;    /* sentinel */
}

void CWDec_Reset(CWDec_t *cw, CWEnv_t *env)
{
    env->edge_rd     = env->edge_wr;   /* drop queued edges */
    cw->prev_keyed   = false;
    cw->rise_samp    = env->sample_clock;
    cw->fall_samp    = env->sample_clock;
    cw->el_pending   = false;
    cw->mbits        = 1U;
    cw->char_decoded = false;
    cw->word_done    = false;
    /* dit_ms deliberately preserved: callers re-seed from menu WPM where
     * appropriate, and a mid-session toggle keeps the adapted estimate. */
}

/* Append a character to the ring buffer */
static void cwdec_emit(CWDec_t *cw, char c)
{
    cw->text[cw->head] = c;
    cw->head = (uint8_t)((cw->head + 1U) % CWDEC_TEXT_LEN);
    /* Saturate: once the ring is full it stays full (uint8_t would wrap
     * at 256 and make GetText return an empty string). */
    if (cw->count < (uint8_t)CWDEC_TEXT_LEN) cw->count++;
}

/* Decode mbits and emit decoded char (or '?' for unknown) */
static void cwdec_emit_char(CWDec_t *cw)
{
    if (cw->mbits < 2U) return;   /* only sentinel, no elements */
    char c = 0;
    if (cw->mbits < 64U) c = cw_table[cw->mbits];
    if (c) {
        cwdec_emit(cw, c);
        cw->n_chars++;
    } else {
        cwdec_emit(cw, '?');
        cw->n_unknown++;
    }
    cw->mbits = 1U;   /* reset to sentinel */
}

/* Add one element (dit or dah) to the accumulator.
 * Classify at 2× dit (midpoint of the 1:3 dit/dah ratio).  The adaptive dit
 * estimate learns from BOTH element kinds (dah contributes dur/3) with an
 * asymmetric rate: fast follow-down (1/2), slow follow-up (1/8).  Real
 * elements are never shorter than the true dit, so the estimate converges
 * onto the fastest sender within a few elements, while a misclassified dah
 * can only nudge it up slowly — the old 2.5× + dit-only update ran away
 * upward for anything ≳1.2× faster than the seeded WPM.
 */
static void cwdec_add_element(CWDec_t *cw, uint32_t dur_ms)
{
    uint8_t  is_dah = (dur_ms > (cw->dit_ms * 2U)) ? 1U : 0U;
    uint32_t est    = is_dah ? (dur_ms / 3U) : dur_ms;

    if (est < cw->dit_ms) {
        cw->dit_ms = (cw->dit_ms + est) / 2U;          /* fast follow-down */
    } else {
        cw->dit_ms = (cw->dit_ms * 7U + est) / 8U;     /* slow follow-up  */
    }
    /* Clamp to 5..80 WPM */
    if (cw->dit_ms < 15U)  cw->dit_ms = 15U;
    if (cw->dit_ms > 240U) cw->dit_ms = 240U;

    /* Accumulate into morse bit tree */
    if (cw->mbits < 32U) {   /* max 5 elements (index < 64) */
        cw->mbits = (uint8_t)((cw->mbits << 1U) | is_dah);
    } else {
        /* Overflow: too many elements — reset */
        cw->mbits = 1U;
    }
}

/* Char/word boundary detection for a given OFF duration.  Called both when
 * a rising edge closes a gap (the whole gap can sit inside one processing
 * burst after a main-loop stall, invisible to the poll-time check) and at
 * poll time for a still-open trailing gap. */
static bool cwdec_check_space(CWDec_t *cw, uint32_t space_ms)
{
    bool changed = false;

    /* Character end: space ≥ 2× dit (midpoint of the 1:3 gap ratio)
     * AND we have accumulated elements */
    if (!cw->char_decoded && cw->mbits > 1U &&
        space_ms >= (cw->dit_ms * 2U)) {
        cwdec_emit_char(cw);
        cw->char_decoded = true;
        changed = true;
    }

    /* Word space: space ≥ 5× dit after a character */
    if (cw->char_decoded && !cw->word_done &&
        space_ms >= (cw->dit_ms * 5U)) {
        cwdec_emit(cw, ' ');
        cw->word_done = true;
        changed = true;
    }

    return changed;
}

/* Commit the deferred element once the gap after it has proven real.
 * Applies the noise-blip (< dit/4) and steady-carrier (> 8× dit) filters. */
static void cwdec_commit_pending(CWDec_t *cw, uint32_t spms)
{
    uint32_t dur_ms = (cw->pend_fall - cw->pend_rise) / spms;
    cw->el_pending  = false;

    if (dur_ms < (cw->dit_ms / 4U)) {
        /* Noise blip: ignore entirely — the current inter-element space
         * keeps running from the last real element. */
    } else if (dur_ms > (cw->dit_ms * 8U)) {
        /* Steady carrier (tune-up): discard accumulated elements and
         * suppress the char/word timeouts for this space. */
        cw->mbits        = 1U;
        cw->fall_samp    = cw->pend_fall;
        cw->char_decoded = true;
        cw->word_done    = true;
    } else {
        cwdec_add_element(cw, dur_ms);
        cw->fall_samp    = cw->pend_fall;
        cw->char_decoded = false;
        cw->word_done    = false;
    }
}

bool CWDec_Update(CWDec_t *cw, CWEnv_t *env)
{
    bool     changed = false;
    uint32_t spms    = env->samps_per_ms ? env->samps_per_ms : 48U;

    /* Consume queued keying edges (sample-domain timestamps, exact even
     * when the main loop stalled and several audio blocks were processed
     * in one burst). */
    while (env->edge_rd != env->edge_wr) {
        uint8_t  slot  = env->edge_rd;
        uint32_t t     = env->edge_time[slot];
        bool     state = (env->edge_state[slot] != 0U);
        env->edge_rd   = (uint8_t)((slot + 1U) & (CWDEC_EDGE_RING - 1U));

        if (state == cw->prev_keyed) {
            /* Duplicate state after a ring overflow: resync timestamps only */
            if (state) cw->rise_samp = t; else cw->fall_samp = t;
            continue;
        }
        if (state) {
            /* Rising edge. */
            if (cw->el_pending &&
                ((t - cw->pend_fall) / spms) < (cw->dit_ms / 4U)) {
                /* QSB dropout: the OFF was too short to be a real gap —
                 * re-open the pending element instead of splitting it. */
                cw->rise_samp  = cw->pend_rise;
                cw->el_pending = false;
            } else {
                if (cw->el_pending) cwdec_commit_pending(cw, spms);
                /* Close the preceding gap — it may contain a char/word
                 * boundary that the poll-time check never saw. */
                changed |= cwdec_check_space(cw, (t - cw->fall_samp) / spms);
                cw->rise_samp = t;
            }
        } else {
            /* Falling edge: defer the element until the gap proves real */
            cw->pend_rise  = cw->rise_samp;
            cw->pend_fall  = t;
            cw->el_pending = true;
        }
        cw->prev_keyed = state;
    }

    /* While key is OFF: commit a pending element once its trailing gap
     * exceeds dit/4, then check the still-open gap for char/word ends.
     * All spans are measured on the same sample clock the edges use. */
    if (!cw->prev_keyed) {
        if (cw->el_pending &&
            ((env->sample_clock - cw->pend_fall) / spms) >= (cw->dit_ms / 4U)) {
            cwdec_commit_pending(cw, spms);
        }
        if (!cw->el_pending) {
            changed |= cwdec_check_space(cw,
                           (env->sample_clock - cw->fall_samp) / spms);
        }
    }

    return changed;
}

void CWDec_GetText(const CWDec_t *cw, char *buf, uint8_t max_chars)
{
    if (!buf || max_chars == 0U) return;

    uint8_t avail = (cw->count < (uint32_t)CWDEC_TEXT_LEN)
                    ? (uint8_t)cw->count
                    : (uint8_t)CWDEC_TEXT_LEN;

    uint8_t n = (avail < max_chars) ? avail : (uint8_t)(max_chars - 1U);

    /* Find start: head - n (wrapping) */
    uint8_t start = (uint8_t)((cw->head + CWDEC_TEXT_LEN - n) % CWDEC_TEXT_LEN);
    for (uint8_t i = 0U; i < n; i++) {
        buf[i] = cw->text[(start + i) % CWDEC_TEXT_LEN];
    }
    buf[n] = '\0';
}
