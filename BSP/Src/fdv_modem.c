/**
  ******************************************************************************
  * @file    fdv_modem.c
  * @brief   FreeDV narrowband OFDM modem — Phase 3 TX
  *
  *  NCO approach: 32-bit phase accumulator per carrier; cosine read from
  *  a 128-entry LUT using the upper 7 bits (phase >> 25).
  *  Phase resolution: 360°/2^32 ≈ 0.000000084°; LUT quantisation ≈ 2.8°.
  *  QPSK constellation points are at 0°/90°/180°/270° (margins ±45°), so
  *  2.8° LUT error is well within the decision boundary.
  *
  *  DQPSK: phase offsets are reset to 0 each super-frame so the pilot
  *  symbol always provides a fresh reference.  Each data symbol then adds
  *  the per-carrier differential phase (0 / π/2 / π / 3π/2) into phase_off.
  ******************************************************************************
  */

#include "fdv_modem.h"
#include <string.h>
#include <math.h>

/* ── 128-entry cosine LUT (initialised once) ───────────────────────────────
 *  s_cos_lut[i] = cos(2π × i / 128)
 *  Index from 32-bit phase: upper 7 bits → phase >> 25
 * ────────────────────────────────────────────────────────────────────────── */
static float   s_cos_lut[128];
static uint8_t s_cos_lut_ready = 0U;

static void ensure_cos_lut(void)
{
    if (s_cos_lut_ready) return;
    for (int i = 0; i < 128; i++)
        s_cos_lut[i] = cosf(6.28318530f * (float)i / 128.0f);
    s_cos_lut_ready = 1U;
}

/* ── Generate n_samps samples from current NCO + phase_off state ────────── */
static void fdv_gen_samples(FdvModem_t *m, uint32_t n_samps, float *out)
{
    for (uint32_t n = 0U; n < n_samps; n++) {
        float y = 0.0f;
        for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
            uint32_t ph = m->phase_acc[k] + m->phase_off[k];
            y += s_cos_lut[ph >> 25];      /* upper 7 bits select LUT entry */
            m->phase_acc[k] += m->phase_inc[k];
        }
        out[n] = y * m->amp;
    }
}

/* ── Apply 32-bit chunk of DQPSK data (16 carriers × 2 bits) ─────────────
 *
 *  bytes[0..3] hold 32 bits MSB-first.
 *  Carrier k uses bits at positions 2k and 2k+1 (k = 0..15).
 *  QPSK map: 00→0, 01→π/2, 10→π, 11→3π/2  (binary, not Gray coded)
 *  In 32-bit phase: π/2 = 0x40000000.
 * ────────────────────────────────────────────────────────────────────────── */
static void fdv_apply_bits(FdvModem_t *m, const uint8_t *bytes)
{
    for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
        uint32_t bit_pos = k * 2U;
        uint32_t byte_i  = bit_pos / 8U;
        uint32_t shift   = 6U - (bit_pos % 8U);   /* MSB first within byte */
        uint8_t  b2      = (bytes[byte_i] >> shift) & 0x3U;
        /* b2 ∈ {0,1,2,3} → Δφ = 0, π/2, π, 3π/2 */
        m->phase_off[k] += (uint32_t)b2 << 30;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void FdvModem_Init(FdvModem_t *m)
{
    ensure_cos_lut();
    memset(m, 0, sizeof(*m));
    m->amp = FDV_AMP;

    /* Carrier frequencies: 450 + k × 75 Hz, phase_inc = f/8000 × 2^32 */
    for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
        float freq      = 450.0f + (float)k * 75.0f;
        m->phase_inc[k] = (uint32_t)(freq / 8000.0f * 4294967296.0f);
    }
}

void FdvModem_EncodeSuperFrame(FdvModem_t *m,
                               const uint8_t bits[8],
                               float out[FDV_FRAME_SAMPS])
{
    /* Reset DQPSK offsets: pilot always transmitted at reference phase 0 */
    for (uint32_t k = 0U; k < FDV_CARRIERS; k++)
        m->phase_off[k] = 0U;

    /* Symbol 0 — PILOT (no phase update before generation) */
    fdv_gen_samples(m, FDV_SYM0_SAMPS, out);

    /* Symbol 1 — DATA1: bits[0..3] = bits 0-31 */
    fdv_apply_bits(m, bits);
    fdv_gen_samples(m, FDV_SYM1_SAMPS, out + FDV_SYM0_SAMPS);

    /* Symbol 2 — DATA2: bits[4..7] = bits 32-63 */
    fdv_apply_bits(m, bits + 4U);
    fdv_gen_samples(m, FDV_SYM2_SAMPS, out + FDV_SYM0_SAMPS + FDV_SYM1_SAMPS);
}
