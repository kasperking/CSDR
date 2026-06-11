#ifndef __CW_KEYER_H
#define __CW_KEYER_H

#include <stdint.h>
#include <stdbool.h>

/* ── Keyer mode ─────────────────────────────────────────────── */
typedef enum {
    KEYER_STRAIGHT = 0,
    KEYER_IAMBIC_A = 1,  /* no memory: checks paddles at element end */
    KEYER_IAMBIC_B = 2,  /* Curtis B: memory captured during element  */
} CWKeyerMode_t;

/* ── Break-in mode ──────────────────────────────────────────── */
typedef enum {
    BKIN_OFF  = 0,  /* PTT not driven by keyer (manual only)          */
    BKIN_SEMI = 1,  /* PTT on first keydown; release after bk_delay   */
    BKIN_FULL = 2,  /* PTT per element + 1-dit holdoff after last key  */
} CWBkin_t;

/* ── Internal state IDs ─────────────────────────────────────── */
#define KY_IDLE    0U
#define KY_DIT_ON  1U
#define KY_DAH_ON  2U
#define KY_INTER   3U

typedef struct {
    /* Config — caller fills before first CWKeyer_Update call */
    CWKeyerMode_t mode;
    bool          paddle_reverse;  /* swap DIT/DAH pins            */
    uint8_t       wpm;             /* 5-40 WPM                     */
    CWBkin_t      bkin;
    uint16_t      bk_delay_ms;    /* BK-IN Semi hang time (ms)    */

    /* Outputs — read by main loop after CWKeyer_Update */
    bool          key_down;  /* element active (sidetone + RF on) */
    bool          ptt_out;   /* PTT should be asserted            */

    /* Internal */
    uint32_t      dit_ms;          /* 1200 / wpm                   */
    uint8_t       ky_state;        /* KY_IDLE / DIT_ON / DAH_ON / INTER */
    bool          mem_dit;         /* Iambic-B DIT memory          */
    bool          mem_dah;         /* Iambic-B DAH memory          */
    bool          prev_was_dit;    /* last element type            */
    bool          ever_keyed;      /* at least one element sent since PTT */
    uint32_t      t_elem_start;    /* HAL_GetTick at element start */
    uint32_t      t_last_up;       /* HAL_GetTick at last key-up   */
} CWKeyer_t;

void CWKeyer_Init(CWKeyer_t *k);
void CWKeyer_SetWPM(CWKeyer_t *k, uint8_t wpm);
void CWKeyer_Update(CWKeyer_t *k);  /* call every main-loop iteration */

#endif /* __CW_KEYER_H */
