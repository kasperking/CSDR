#include "cw_keyer.h"
#include <math.h>
#include <string.h>

/* NCO lookup table size (must match sdr_dsp.c) */
#define ST_LUT_BITS  10U
#define ST_LUT_SIZE  (1U << ST_LUT_BITS)
#define ST_LUT_MASK  (ST_LUT_SIZE - 1U)

/* 1024-entry sine LUT, range ±1.0 */
static float s_st_lut[ST_LUT_SIZE];
static bool  s_lut_ready = false;

static void lut_init(void)
{
    if (s_lut_ready) return;
    for (uint32_t i = 0U; i < ST_LUT_SIZE; i++)
        s_st_lut[i] = sinf(2.0f * 3.14159265f * (float)i / (float)ST_LUT_SIZE);
    s_lut_ready = true;
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/* Active-low GPIO read — returns true when pin is pressed */
static inline bool pin_pressed(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

/* Read effective dit and dah paddle states, accounting for paddle_rev */
static inline void read_paddles(const CW_Keyer_t *k, bool *dit, bool *dah)
{
    bool raw_dit = pin_pressed(k->dit_port, k->dit_pin);
    bool raw_dah = pin_pressed(k->dah_port, k->dah_pin);
    if (k->paddle_rev) { *dit = raw_dah; *dah = raw_dit; }
    else               { *dit = raw_dit; *dah = raw_dah; }
}

/* ── Straight key update ─────────────────────────────────────────────────── */

static void sk_update(CW_Keyer_t *k)
{
    bool dit, dah;
    read_paddles(k, &dit, &dah);
    bool pressed = dit;   /* straight key wired to DIT pin */

    if (pressed != k->sk_prev) {
        k->sk_prev = pressed;
        k->key_down = pressed;
        if (pressed)
            k->last_key_ms = 0U;   /* reset; key is still down */
        else
            k->last_key_ms = HAL_GetTick();
    }
}

/* ── Iambic update ───────────────────────────────────────────────────────── */

static void ik_start_dit(CW_Keyer_t *k, uint32_t now)
{
    k->ik_state  = IK_DIT;
    k->ik_end_ms = now + k->dit_ms;
    k->key_down  = true;
    k->dit_mem   = false;  /* clear own memory when element starts */
}

static void ik_start_dah(CW_Keyer_t *k, uint32_t now)
{
    k->ik_state  = IK_DAH;
    k->ik_end_ms = now + k->dit_ms * 3U;
    k->key_down  = true;
    k->dah_mem   = false;
}

static void ik_start_space(CW_Keyer_t *k, uint32_t now, IambicState_t space_state)
{
    k->ik_state  = space_state;
    k->ik_end_ms = now + k->dit_ms;  /* inter-element gap = 1 dit */
    k->key_down  = false;
    k->last_key_ms = now;
}

static void ik_update(CW_Keyer_t *k)
{
    uint32_t now = HAL_GetTick();
    bool dit_p, dah_p;
    read_paddles(k, &dit_p, &dah_p);

    switch (k->ik_state) {

    case IK_IDLE:
        if (dit_p && dah_p) {
            /* Both pressed from idle: prefer DIT (A), or DAH (B) */
            if (k->mode == CW_KEYER_IAMBIC_B)
                ik_start_dah(k, now);
            else
                ik_start_dit(k, now);
        } else if (dit_p) {
            ik_start_dit(k, now);
        } else if (dah_p) {
            ik_start_dah(k, now);
        }
        break;

    case IK_DIT:
        /* Memory update during dit (Iambic B only) */
        if (k->mode == CW_KEYER_IAMBIC_B && dah_p)
            k->dah_mem = true;
        if ((int32_t)(now - k->ik_end_ms) >= 0)
            ik_start_space(k, now, IK_DIT_SPACE);
        break;

    case IK_DIT_SPACE:
        if ((int32_t)(now - k->ik_end_ms) >= 0) {
            /* Decide next element */
            if (k->mode == CW_KEYER_IAMBIC_B && k->dah_mem)
                ik_start_dah(k, now);
            else if (dah_p)
                ik_start_dah(k, now);
            else if (k->mode == CW_KEYER_IAMBIC_B && k->dit_mem)
                ik_start_dit(k, now);
            else if (dit_p)
                ik_start_dit(k, now);
            else
                k->ik_state = IK_IDLE;
        }
        break;

    case IK_DAH:
        if (k->mode == CW_KEYER_IAMBIC_B && dit_p)
            k->dit_mem = true;
        if ((int32_t)(now - k->ik_end_ms) >= 0)
            ik_start_space(k, now, IK_DAH_SPACE);
        break;

    case IK_DAH_SPACE:
        if ((int32_t)(now - k->ik_end_ms) >= 0) {
            if (k->mode == CW_KEYER_IAMBIC_B && k->dit_mem)
                ik_start_dit(k, now);
            else if (dit_p)
                ik_start_dit(k, now);
            else if (k->mode == CW_KEYER_IAMBIC_B && k->dah_mem)
                ik_start_dah(k, now);
            else if (dah_p)
                ik_start_dah(k, now);
            else
                k->ik_state = IK_IDLE;
        }
        break;

    default:
        k->ik_state = IK_IDLE;
        break;
    }
}

/* ── BK-IN T/R management ────────────────────────────────────────────────── */

static void bkin_update(CW_Keyer_t *k)
{
    if (k->bkin_mode == CW_BKIN_OFF || k->set_tx == NULL)
        return;

    uint32_t now = HAL_GetTick();

    if (k->key_down) {
        /* Assert TX on first key-down */
        if (!k->tx_active) {
            k->set_tx(true);
            k->tx_active = true;
            k->bkin_tx_pending = true;
        }
        k->last_key_ms = now;
    } else if (k->bkin_tx_pending) {
        /* Compute delay: Semi uses bk_delay_ms; Full uses 1 dit */
        uint32_t delay = (k->bkin_mode == CW_BKIN_FULL)
                         ? k->dit_ms
                         : (uint32_t)k->bk_delay_ms;
        if ((now - k->last_key_ms) >= delay) {
            k->set_tx(false);
            k->tx_active       = false;
            k->bkin_tx_pending = false;
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void CW_Keyer_Init(CW_Keyer_t *k,
                   GPIO_TypeDef *dit_port, uint16_t dit_pin,
                   GPIO_TypeDef *dah_port, uint16_t dah_pin,
                   void (*set_tx)(bool tx))
{
    lut_init();
    memset(k, 0, sizeof(*k));

    k->dit_port = dit_port;
    k->dit_pin  = dit_pin;
    k->dah_port = dah_port;
    k->dah_pin  = dah_pin;
    k->set_tx   = set_tx;

    /* Default config — overwritten by CW_Keyer_SetConfig on init */
    k->mode           = CW_KEYER_STRAIGHT;
    k->dit_ms         = 60U;   /* 20 WPM */
    k->pitch_hz       = 700U;
    k->sidetone_vol   = 50U;
    k->bkin_mode      = CW_BKIN_OFF;
    k->bk_delay_ms    = 150U;

    /* Envelope smoother at default 48 kHz — updated by SetConfig */
    k->st_alpha_on  = expf(-1.0f / (0.003f * 48000.0f));
    k->st_alpha_off = expf(-1.0f / (0.005f * 48000.0f));
    k->st_phase_inc = (uint32_t)((int64_t)700 * (int64_t)4294967296LL / 48000LL);
}

void CW_Keyer_SetConfig(CW_Keyer_t *k,
                        CW_KeyerMode_t mode, bool paddle_rev,
                        uint8_t wpm, uint16_t pitch_hz,
                        uint8_t sidetone_vol,
                        CW_BKInMode_t bkin_mode, uint16_t bk_delay_ms,
                        uint32_t sample_rate)
{
    k->mode         = mode;
    k->paddle_rev   = paddle_rev;
    k->dit_ms       = (wpm > 0U) ? (1200U / wpm) : 60U;
    k->pitch_hz     = pitch_hz;
    k->sidetone_vol = sidetone_vol;
    k->bkin_mode    = bkin_mode;
    k->bk_delay_ms  = bk_delay_ms;

    if (sample_rate > 0U) {
        k->st_phase_inc = (uint32_t)((int64_t)pitch_hz
                           * (int64_t)4294967296LL / (int64_t)sample_rate);
        k->st_alpha_on  = expf(-1.0f / (0.003f * (float)sample_rate));
        k->st_alpha_off = expf(-1.0f / (0.005f * (float)sample_rate));
    }

    /* When switching keyer mode, reset iambic state to avoid stuck elements */
    k->ik_state = IK_IDLE;
    k->dit_mem  = false;
    k->dah_mem  = false;
}

void CW_Keyer_Update(CW_Keyer_t *k)
{
    if (k->mode == CW_KEYER_STRAIGHT)
        sk_update(k);
    else
        ik_update(k);

    bkin_update(k);
}

void CW_Keyer_Sidetone(CW_Keyer_t *k, int32_t *buf, uint32_t len,
                       uint32_t sample_rate)
{
    (void)sample_rate;   /* coefficients already set in SetConfig */

    if (k->sidetone_vol == 0U) return;

    /* Peak amplitude: 24-bit I2S → 8 388 608; scale by sidetone_vol/100 */
    float peak = (float)k->sidetone_vol * 83886.0f;  /* vol=100 → ~8.4M */

    for (uint32_t i = 0U; i < len; i++) {
        /* Envelope: ramp toward 1.0 when key down, toward 0.0 when up */
        float target = k->key_down ? 1.0f : 0.0f;
        float alpha  = k->key_down ? k->st_alpha_on : k->st_alpha_off;
        k->st_env    = alpha * k->st_env + (1.0f - alpha) * target;

        if (k->st_env < 1e-5f) continue;   /* below noise floor — skip */

        k->st_phase_acc += k->st_phase_inc;
        uint32_t idx = k->st_phase_acc >> (32U - ST_LUT_BITS);
        float tone   = s_st_lut[idx & ST_LUT_MASK];

        buf[i] += (int32_t)(tone * peak * k->st_env);
    }
}
