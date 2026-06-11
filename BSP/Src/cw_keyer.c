#include "cw_keyer.h"
#include "main.h"
#include "stm32h7xx_hal.h"

/* ── GPIO helpers ───────────────────────────────────────────── */

static inline bool _read_dit(const CWKeyer_t *k)
{
    bool raw_dit = (HAL_GPIO_ReadPin(DIT_GPIO_Port, DIT_Pin) == GPIO_PIN_RESET);
    bool raw_dah = (HAL_GPIO_ReadPin(DAH_GPIO_Port, DAH_Pin) == GPIO_PIN_RESET);
    return k->paddle_reverse ? raw_dah : raw_dit;
}

static inline bool _read_dah(const CWKeyer_t *k)
{
    bool raw_dit = (HAL_GPIO_ReadPin(DIT_GPIO_Port, DIT_Pin) == GPIO_PIN_RESET);
    bool raw_dah = (HAL_GPIO_ReadPin(DAH_GPIO_Port, DAH_Pin) == GPIO_PIN_RESET);
    return k->paddle_reverse ? raw_dit : raw_dah;
}

/* ── Init / config ──────────────────────────────────────────── */

void CWKeyer_Init(CWKeyer_t *k)
{
    k->key_down     = false;
    k->ptt_out      = false;
    k->ky_state     = KY_IDLE;
    k->mem_dit      = false;
    k->mem_dah      = false;
    k->prev_was_dit = false;
    k->ever_keyed   = false;
    k->t_elem_start = 0U;
    k->t_last_up    = 0U;
    CWKeyer_SetWPM(k, k->wpm > 0U ? k->wpm : 20U);
}

void CWKeyer_SetWPM(CWKeyer_t *k, uint8_t wpm)
{
    if (wpm < 5U)  wpm = 5U;
    if (wpm > 40U) wpm = 40U;
    k->wpm    = wpm;
    k->dit_ms = 1200U / (uint32_t)wpm;
}

/* ── BK-IN PTT management (shared by straight + iambic) ─────── */

static void _bkin_update(CWKeyer_t *k, bool is_active, uint32_t now)
{
    switch (k->bkin) {
    case BKIN_OFF:
        k->ptt_out = false;
        break;

    case BKIN_SEMI:
        if (is_active) {
            k->ptt_out    = true;
            k->ever_keyed = true;
            k->t_last_up  = now;  /* keep resetting while active */
        } else if (k->ever_keyed) {
            /* idle: release after hang time */
            if ((now - k->t_last_up) >= (uint32_t)k->bk_delay_ms) {
                k->ptt_out    = false;
                k->ever_keyed = false;
            }
        }
        break;

    case BKIN_FULL:
        /* PTT follows key_down + 1-dit holdoff */
        if (is_active) {
            k->ptt_out   = true;
            k->t_last_up = now;
        } else if (k->ptt_out) {
            if ((now - k->t_last_up) >= k->dit_ms) {
                k->ptt_out = false;
            }
        }
        break;
    }
}

/* ── Main update ────────────────────────────────────────────── */

void CWKeyer_Update(CWKeyer_t *k)
{
    uint32_t now     = HAL_GetTick();
    bool     dit     = _read_dit(k);
    bool     dah     = _read_dah(k);
    uint32_t elapsed = now - k->t_elem_start;

    /* ── Straight key ── */
    if (k->mode == KEYER_STRAIGHT) {
        bool key = dit || dah;
        if (k->key_down && !key) k->t_last_up = now;
        k->key_down = key;
        _bkin_update(k, key, now);
        return;
    }

    /* ── Iambic A/B state machine ── */
    switch (k->ky_state) {

    case KY_IDLE:
        k->key_down = false;
        k->mem_dit  = false;
        k->mem_dah  = false;
        if (dit) {
            k->ky_state     = KY_DIT_ON;
            k->t_elem_start = now;
            k->key_down     = true;
        } else if (dah) {
            k->ky_state     = KY_DAH_ON;
            k->t_elem_start = now;
            k->key_down     = true;
        }
        break;

    case KY_DIT_ON:
        k->key_down = true;
        if (k->mode == KEYER_IAMBIC_B && dah) k->mem_dah = true;
        if (elapsed >= k->dit_ms) {
            k->key_down     = false;
            k->prev_was_dit = true;
            k->t_last_up    = now;
            k->ky_state     = KY_INTER;
            k->t_elem_start = now;
        }
        break;

    case KY_DAH_ON:
        k->key_down = true;
        if (k->mode == KEYER_IAMBIC_B && dit) k->mem_dit = true;
        if (elapsed >= k->dit_ms * 3U) {
            k->key_down     = false;
            k->prev_was_dit = false;
            k->t_last_up    = now;
            k->ky_state     = KY_INTER;
            k->t_elem_start = now;
        }
        break;

    case KY_INTER:
        k->key_down = false;
        if (elapsed >= k->dit_ms) {
            /* decide next element */
            if (k->prev_was_dit) {
                bool next_dah = dah || k->mem_dah;
                k->mem_dah = false;
                if (next_dah) {
                    k->ky_state = KY_DAH_ON; k->t_elem_start = now; k->key_down = true;
                } else if (dit) {
                    k->ky_state = KY_DIT_ON; k->t_elem_start = now; k->key_down = true;
                } else {
                    k->ky_state = KY_IDLE;
                }
            } else {
                bool next_dit = dit || k->mem_dit;
                k->mem_dit = false;
                if (next_dit) {
                    k->ky_state = KY_DIT_ON; k->t_elem_start = now; k->key_down = true;
                } else if (dah) {
                    k->ky_state = KY_DAH_ON; k->t_elem_start = now; k->key_down = true;
                } else {
                    k->ky_state = KY_IDLE;
                }
            }
        }
        break;

    default:
        k->ky_state = KY_IDLE;
        break;
    }

    /* BK-IN PTT: "active" = element on OR still in INTER (short space between elements) */
    bool active = (k->ky_state == KY_DIT_ON || k->ky_state == KY_DAH_ON || k->ky_state == KY_INTER);
    _bkin_update(k, active, now);
}
