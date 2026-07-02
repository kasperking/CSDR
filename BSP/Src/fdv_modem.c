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

/* ── DQPSK differential decoder ────────────────────────────────────────────
 *
 *  Computes phase difference between two correlator snapshots using the
 *  complex product  sig × conj(ref) = e^{j*(phase_sig - phase_ref)}.
 *  The 2-bit QPSK symbol is recovered from the quadrant of the result.
 *
 *  Bit packing follows the TX convention: carrier k carries bits at
 *  positions 2k, 2k+1 (MSB first within each byte).
 *  byte_start: 0 for DATA1 (bits 0-31), 4 for DATA2 (bits 32-63).
 * ────────────────────────────────────────────────────────────────────────── */
static void fdv_dqpsk_decode(const float *ref_i, const float *ref_q,
                              const float *sig_i, const float *sig_q,
                              uint8_t *out_bits, uint32_t byte_start)
{
    memset(out_bits + byte_start, 0U, 4U);
    for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
        /* Complex product: sig × conj(ref) */
        float re = sig_i[k] * ref_i[k] + sig_q[k] * ref_q[k];
        float im = sig_q[k] * ref_i[k] - sig_i[k] * ref_q[k];
        /* Phase angle in [0, 2π) */
        float dph = atan2f(im, re);
        if (dph < 0.0f) dph += 6.28318530f;
        /* Map to QPSK symbol: 0°→00, 90°→01, 180°→10, 270°→11 */
        uint8_t b2 = (uint8_t)((uint32_t)(dph * (2.0f / 3.14159265f) + 0.5f) & 3U);
        /* Pack: same layout as fdv_apply_bits in TX */
        uint32_t bit_pos = k * 2U;
        uint32_t byte_i  = byte_start + bit_pos / 8U;
        uint32_t shift   = 6U - (bit_pos % 8U);
        out_bits[byte_i] |= (uint8_t)(b2 << shift);
    }
}

void FdvModem_RxInit(FdvModemRx_t *rx)
{
    ensure_cos_lut();
    memset(rx, 0, sizeof(*rx));
    for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
        float freq      = 450.0f + (float)k * 75.0f;
        rx->phase_inc[k] = (uint32_t)(freq / 8000.0f * 4294967296.0f);
    }
}

/* ── FdvModem_RxPush ────────────────────────────────────────────────────────
 *
 *  Per-sample OFDM demodulator: accumulates correlators across each symbol,
 *  snapshots at symbol boundaries, then DQPSK-decodes at frame end.
 *
 *  The NCO runs continuously (no reset at frame/symbol boundaries) so that
 *  the common initial phase error between TX and RX NCOs cancels in the
 *  differential operation — the DQPSK property.
 *
 *  Sin from the cosine LUT: sin(φ) = cos(φ − π/2).
 *  In the 128-entry LUT: -π/2 ↔ -32 steps ↔ +96 steps (mod 128).
 * ────────────────────────────────────────────────────────────────────────── */
void FdvModem_RxPush(FdvModemRx_t *rx, float s)
{
    for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
        uint8_t ci = (uint8_t)(rx->phase_acc[k] >> 25);
        uint8_t si = (uint8_t)((ci + 96U) & 127U);
        rx->corr_i[k] += s * s_cos_lut[ci];
        rx->corr_q[k] += s * s_cos_lut[si];
        rx->phase_acc[k] += rx->phase_inc[k];
    }
    rx->samp_cnt++;

    if (rx->samp_cnt == FDV_SYM0_SAMPS) {
        /* End of PILOT: snapshot correlators */
        for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
            rx->pilot_i[k] = rx->corr_i[k];
            rx->pilot_q[k] = rx->corr_q[k];
            rx->corr_i[k]  = 0.0f;
            rx->corr_q[k]  = 0.0f;
        }
    } else if (rx->samp_cnt == FDV_SYM0_SAMPS + FDV_SYM1_SAMPS) {
        /* End of DATA1: decode bits 0-31 vs PILOT */
        for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
            rx->sym1_i[k] = rx->corr_i[k];
            rx->sym1_q[k] = rx->corr_q[k];
            rx->corr_i[k] = 0.0f;
            rx->corr_q[k] = 0.0f;
        }
        fdv_dqpsk_decode(rx->pilot_i, rx->pilot_q,
                         rx->sym1_i,  rx->sym1_q,
                         rx->rx_bits, 0U);
    } else if (rx->samp_cnt >= FDV_FRAME_SAMPS) {
        /* End of DATA2: decode bits 32-63 vs DATA1, signal frame ready */
        fdv_dqpsk_decode(rx->sym1_i, rx->sym1_q,
                         rx->corr_i, rx->corr_q,
                         rx->rx_bits, 4U);
        rx->frame_ready = true;
        rx->samp_cnt    = 0U;
        for (uint32_t k = 0U; k < FDV_CARRIERS; k++) {
            rx->corr_i[k] = 0.0f;
            rx->corr_q[k] = 0.0f;
        }
    }
}

/* ── TX encoder ─────────────────────────────────────────────────────────── */

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
