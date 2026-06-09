#include "cw_decode.h"
#include "stm32h7xx_hal.h"
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
    '7',   0,    0,   '8',   0,   '9',  '0',   0,
};

/* Coefficients: α = exp(-1 / (τ_samples)) ≈ exp(-1 / (τ_ms * Fs / 1000)) */
void CWEnv_Init(CWEnv_t *env, uint32_t sample_rate)
{
    float sr = (float)sample_rate;
    env->alpha_r  = expf(-1.0f / (0.0005f * sr));  /* 0.5 ms attack  */
    env->alpha_f  = expf(-1.0f / (0.005f  * sr));  /* 5 ms   release */
    env->alpha_nd = expf(-1.0f / (0.020f  * sr));  /* 20 ms  floor fall */
    env->alpha_nu = expf(-1.0f / (0.400f  * sr));  /* 400 ms floor rise */
    env->env_sq   = 0.0f;
    env->floor_sq = 1e-8f;  /* small non-zero to avoid threshold = 0 at startup */
    env->keyed    = 0U;
}

void CWDec_Init(CWDec_t *cw)
{
    memset(cw, 0, sizeof(CWDec_t));
    cw->dit_ms = 60U;   /* 20 WPM default */
    cw->mbits  = 1U;    /* sentinel */
}

void CWDec_SetWPM(CWDec_t *cw, uint8_t wpm)
{
    if (wpm < 5U)  wpm = 5U;
    if (wpm > 60U) wpm = 60U;
    cw->dit_ms = 1200U / wpm;
}

void CWDec_Reset(CWDec_t *cw)
{
    cw->prev_keyed  = false;
    cw->edge_ms     = HAL_GetTick();
    cw->dit_ms      = 60U;
    cw->mbits       = 1U;
    cw->char_decoded = false;
    cw->word_done   = false;
}

/* Append a character to the ring buffer */
static void cwdec_emit(CWDec_t *cw, char c)
{
    cw->text[cw->head] = c;
    cw->head = (uint8_t)((cw->head + 1U) % CWDEC_TEXT_LEN);
    cw->count++;
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
 * Also updates the adaptive dit estimate from dit durations.
 */
static void cwdec_add_element(CWDec_t *cw, uint32_t dur_ms)
{
    /* Classify: duration ≤ 2.5× dit = dit, otherwise dah */
    uint8_t is_dah = (dur_ms > (cw->dit_ms * 5U / 2U)) ? 1U : 0U;

    /* Update dit estimate from short elements (EWMa with 1/8 weight) */
    if (!is_dah && dur_ms >= (cw->dit_ms / 4U)) {
        cw->dit_ms = (cw->dit_ms * 7U + dur_ms) / 8U;
    }

    /* Accumulate into morse bit tree */
    if (cw->mbits < 32U) {   /* max 5 elements (index < 64) */
        cw->mbits = (uint8_t)((cw->mbits << 1U) | is_dah);
    } else {
        /* Overflow: too many elements — reset */
        cw->mbits = 1U;
    }
}

bool CWDec_Update(CWDec_t *cw, uint8_t keyed)
{
    uint32_t now = HAL_GetTick();
    bool changed = false;

    /* Edge detection */
    if ((uint8_t)(keyed != 0U) != (uint8_t)(cw->prev_keyed)) {
        uint32_t dur = now - cw->edge_ms;
        cw->edge_ms  = now;

        if (cw->prev_keyed) {
            /* Falling edge: key was ON for dur ms → classify element */
            cwdec_add_element(cw, dur);
            cw->char_decoded = false;
            cw->word_done    = false;
        }
        /* Rising edge: key is now ON — no action needed here */

        cw->prev_keyed = (keyed != 0U);
    }

    /* While key is OFF: check for character and word-space timeouts */
    if (!cw->prev_keyed) {
        uint32_t space = now - cw->edge_ms;

        /* Character end: space ≥ 1.5× dit AND we have accumulated elements */
        if (!cw->char_decoded && cw->mbits > 1U &&
            space >= (cw->dit_ms * 3U / 2U)) {
            cwdec_emit_char(cw);
            cw->char_decoded = true;
            changed = true;
        }

        /* Word space: space ≥ 5× dit after a character */
        if (cw->char_decoded && !cw->word_done &&
            space >= (cw->dit_ms * 5U)) {
            cwdec_emit(cw, ' ');
            cw->word_done = true;
            changed = true;
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
