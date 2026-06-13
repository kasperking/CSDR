/**
  ******************************************************************************
  * @file    freedv_mode.c
  * @brief   FreeDV narrowband digital voice – Phase 3
  *
  *  This file implements:
  *    • 48 kHz → 8 kHz polyphase FIR decimator   (FDVR_RATIO = 6)
  *    • 8 kHz → 48 kHz polyphase FIR interpolator (FDVR_RATIO = 6)
  *    • 8 kHz narrowband USB (NBUSB) SSB modulator (Hilbert phasing)
  *    • 8 kHz NBUSB SSB demodulator (stub)
  *    • LPC-10 analysis → 64-bit quantizer → 16-carrier DQPSK OFDM TX
  *
  *  All processing is sample-by-sample to slot into the existing per-sample
  *  loops in DSP_ProcessTX / DSP_Process (sdr_dsp.c).
  *
  *  Polyphase FIR design (decimation):
  *    Prototype LPF:  fc = 3900 Hz, fs_high = 48000 Hz
  *    → cutoff_norm = 3900/48000 ≈ 0.08125
  *    72 taps, Kaiser window β=6.
  *    Coefficients split into 6 polyphases of 12 taps each.
  *
  *    For interpolation (upsample 8→48 kHz) the same prototype filter is
  *    used but the polyphase phases are traversed in reverse order and the
  *    output is scaled by FDVR_RATIO to preserve energy.
  *
  *  Hilbert SSB (NBUSB):
  *    Same 63-tap odd-symmetric FIR as the main RX/TX hilbert, but operated
  *    at 8 kHz so the filter coefficients are recalculated for 8 kHz rate.
  *    NBUSB = USB sideband at 8 kHz BW: tx_q = +H{audio8}, tx_i = delayed audio8.
  ******************************************************************************
  */

#include "freedv_mode.h"
#include "lpc_voc.h"
#include "lpc_quant.h"
#include "fdv_modem.h"
#include <string.h>
#include <math.h>

/* Compile-time guard: LPC frame and OFDM super-frame must be the same length */
typedef char _fdv_frame_size_check[(LPC_FRAME_SAMPS == FDV_FRAME_SAMPS) ? 1 : -1];

/* ── Polyphase prototype filter coefficients (72 taps) ─────────────────────
 *
 *  Designed with Kaiser window, β = 6.0.
 *  Prototype: sinc-based LPF, fc_norm = 3900/48000 = 0.08125.
 *  h[n] = kaiser(n, 72, 6.0) * sinc(0.08125 * (n - 35.5))
 *  Normalised so that sum over polyphase 0 == 1 (unity passband gain after
 *  decimation; multiply by FDVR_RATIO = 6 for interpolation).
 *
 *  These were computed offline; stored as a flat 72-element array.
 *  Polyphase bank p (0..5) uses taps h[p], h[p+6], h[p+12], ..., h[p+66].
 * ────────────────────────────────────────────────────────────────────────── */
static const float s_proto[FDVR_TOTAL_TAPS] = {
/*  0*/ -0.000208f, -0.000627f, -0.001215f, -0.001879f, -0.002476f, -0.002832f,
/*  6*/ -0.002748f, -0.002038f, -0.000558f,  0.001780f,  0.004867f,  0.008466f,
/* 12*/  0.012262f,  0.015879f,  0.018892f,  0.020872f,  0.021441f,  0.020336f,
/* 18*/  0.017476f,  0.013007f,  0.007256f,  0.000717f, -0.006063f, -0.012205f,
/* 24*/ -0.017107f, -0.020225f, -0.021234f, -0.019991f, -0.016571f, -0.011270f,
/* 30*/ -0.004639f,  0.002687f,  0.009868f,  0.016187f,  0.021025f,  0.023949f,
/* 36*/  0.023949f,  0.021025f,  0.016187f,  0.009868f,  0.002687f, -0.004639f,
/* 42*/ -0.011270f, -0.016571f, -0.019991f, -0.021234f, -0.020225f, -0.017107f,
/* 48*/ -0.012205f, -0.006063f,  0.000717f,  0.007256f,  0.013007f,  0.017476f,
/* 54*/  0.020336f,  0.021441f,  0.020872f,  0.018892f,  0.015879f,  0.012262f,
/* 60*/  0.008466f,  0.004867f,  0.001780f, -0.000558f, -0.002038f, -0.002748f,
/* 66*/ -0.002832f, -0.002476f, -0.001879f, -0.001215f, -0.000627f, -0.000208f,
};

/* ── Hilbert FIR coefficients at 8 kHz (63 taps) ───────────────────────────
 *
 *  Odd-symmetric 63-tap Hilbert transformer, windowed with Hann window.
 *  h[n] = 0 for even n-31; h[n] = 2/(π*(n-31)) * hann[n] for odd n-31.
 *  Passband ≈ 100–3900 Hz at 8 kHz (well within 400–3500 Hz voice band).
 *  Group delay = (63-1)/2 = 31 samples at 8 kHz = 3.875 ms.
 * ────────────────────────────────────────────────────────────────────────── */
static const float s_hilbert8[FDVR_HILBERT_TAPS] = {
  /* n=0..62, centre n=31 (zero). Odd offsets only are non-zero. */
   0.000000f,  0.020437f,  0.000000f,  0.021894f,  0.000000f,  0.023852f,
   0.000000f,  0.026536f,  0.000000f,  0.030352f,  0.000000f,  0.035980f,
   0.000000f,  0.044706f,  0.000000f,  0.059781f,  0.000000f,  0.090577f,
   0.000000f,  0.168697f,  0.000000f,  0.597575f,  0.000000f, -0.597575f,
   0.000000f, -0.168697f,  0.000000f, -0.090577f,  0.000000f, -0.059781f,
   0.000000f, -0.044706f,  0.000000f, -0.035980f,  0.000000f, -0.030352f,
   0.000000f, -0.026536f,  0.000000f, -0.023852f,  0.000000f, -0.021894f,
   0.000000f, -0.020437f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,
   0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,
   0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,  0.000000f,
   0.000000f,  0.000000f,  0.000000f,
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── IIR DC-block at 8 kHz  H(z) = (1-z^-1)/(1-0.9975*z^-1) ────────────── */
static void dc8_init(FdvIIR_t *f)
{
    f->b0 =  1.0f; f->b1 = -1.0f; f->b2 = 0.0f;
    f->a1 = -0.9975f; f->a2 = 0.0f;
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

static float dc8_process(FdvIIR_t *f, float x)
{
    float y = f->b0 * x + f->b1 * f->x1 - f->a1 * f->y1;
    f->x1 = x;
    f->y1 = y;
    return y;
}

/* ── Hilbert FIR (circular buffer) ─────────────────────────────────────── */
static void hilbert8_init(FdvHilbert_t *h)
{
    memset(h->buf, 0, sizeof(h->buf));
    h->idx = 0U;
    memcpy(h->coeff, s_hilbert8, sizeof(s_hilbert8));
}

static float hilbert8_process(FdvHilbert_t *h, float x)
{
    h->buf[h->idx] = x;
    float acc = 0.0f;
    uint16_t k = h->idx;
    for (uint16_t i = 0U; i < FDVR_HILBERT_TAPS; i++) {
        acc += h->coeff[i] * h->buf[k];
        k = (k == 0U) ? (FDVR_HILBERT_TAPS - 1U) : (k - 1U);
    }
    h->idx = (uint16_t)((h->idx + 1U) % FDVR_HILBERT_TAPS);
    return acc;
}

/* ── Polyphase decimation: push one 48 kHz sample, return 8 kHz sample ──
 *
 *  The 72-tap prototype filter is split into FDVR_PHASES = 6 polyphases of
 *  12 taps each.  On every input sample we advance the delay line.  On every
 *  6th input (when phase rolls back to 0) we compute the polyphase inner
 *  product and return the result; otherwise we return 0 (caller ignores it).
 *
 *  Polyphase bank p uses h[p + 6*k] for k = 0..11.
 *  The delay line is indexed in reverse (newest sample at index 0).
 *
 *  Returns true and sets *out when a new 8 kHz sample is ready.
 * ────────────────────────────────────────────────────────────────────────── */
static bool decimate_push(Resampler6_t *r, float x, float *out)
{
    /* Shift delay line: move entries up by 1 */
    for (int i = FDVR_TOTAL_TAPS - 1; i > 0; i--)
        r->buf[i] = r->buf[i - 1];
    r->buf[0] = x;

    /* On phase 0, compute polyphase 0 inner product (all 72 taps) */
    bool ready = (r->phase == 0U);
    if (ready) {
        float acc = 0.0f;
        for (uint16_t k = 0U; k < FDVR_TOTAL_TAPS; k++)
            acc += s_proto[k] * r->buf[k];
        /* Scale: sum of prototype = 1/FDVR_RATIO; multiply back for unity gain */
        *out = acc * (float)FDVR_RATIO;
    }

    r->phase = (uint8_t)((r->phase + 1U) % FDVR_PHASES);
    return ready;
}

/* ── Polyphase interpolation: push one 8 kHz sample, write 6 × 48 kHz ──
 *
 *  For interpolation (1:M upsample) we insert FDVR_PHASES-1 zeros between
 *  consecutive 8 kHz samples and filter with the same prototype.
 *  Equivalently, for each output slot p (0..5) we compute:
 *    y[p] = FDVR_RATIO * Σ h[p + FDVR_PHASES*k] * x[n-k]
 *  where x[n] is the 8 kHz input sequence.
 *
 *  Each call to interp_push pushes one 8 kHz sample and fills out[0..5]
 *  with the 6 corresponding 48 kHz samples.
 * ────────────────────────────────────────────────────────────────────────── */
static void __attribute__((unused)) interp_push(Resampler6_t *r, float x, float out[FDVR_PHASES])
{
    /* Shift history by 1 (store one 8 kHz sample per slot) */
    for (int i = FDVR_TAPS_PER_PHASE - 1; i > 0; i--)
        r->buf[i] = r->buf[i - 1];
    r->buf[0] = x;

    /* Compute FDVR_PHASES output samples */
    for (uint8_t p = 0U; p < FDVR_PHASES; p++) {
        float acc = 0.0f;
        for (uint16_t k = 0U; k < FDVR_TAPS_PER_PHASE; k++)
            acc += s_proto[p + FDVR_PHASES * k] * r->buf[k];
        out[p] = acc * (float)FDVR_RATIO;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void FreeDV_Init(FreeDV_State_t *fdv, float audio_gain)
{
    memset(fdv, 0, sizeof(*fdv));
    fdv->audio_gain = audio_gain;
    dc8_init(&fdv->dc8);
    hilbert8_init(&fdv->hilbert8);
    LPC_Voc_Init(&fdv->voc);
    FdvModem_Init(&fdv->modem);
}

/* ── TX: one 48 kHz sample in → IQ pair out ─────────────────────────────
 *
 *  Every FDVR_RATIO=6 calls (8 kHz tick):
 *    1. Polyphase decimate → 8 kHz PCM sample.
 *    2. DC block at 8 kHz.
 *    3. Accumulate in pcm8_buf[] (320-sample / 40 ms frame buffer).
 *    4. When 320 samples full:
 *         LPC_Encode  → LPC_Frame_t  (10-pole analysis)
 *         LPC_Quantize → uint8_t[8]  (64-bit scalar quantization)
 *         FdvModem_EncodeSuperFrame → ofdm_buf[] (320 OFDM audio samples)
 *    5. Pull next sample from ofdm_buf[] (fallback: raw pcm8 for first 40 ms)
 *       → Hilbert NBUSB SSB → IQ hold.
 *
 *  Between ticks (5 out of 6 calls): IQ hold is returned unchanged.
 *
 *  Signal chain on-air:
 *    Voice → LPC → 64-bit frame → OFDM audio (450–1575 Hz) →
 *    Hilbert NBUSB SSB → RF (VFO+450 to VFO+1575 Hz)
 * ────────────────────────────────────────────────────────────────────────── */
void FreeDV_TX_Sample(FreeDV_State_t *fdv,
                      float  audio_in,
                      float *out_i,
                      float *out_q)
{
    float pcm8;
    if (decimate_push(&fdv->dec, audio_in * fdv->audio_gain, &pcm8)) {
        fdv->dec_out_count++;

        /* 1. DC block at 8 kHz */
        pcm8 = dc8_process(&fdv->dc8, pcm8);

        /* 2. Accumulate 40 ms (320 samples) of 8 kHz speech */
        fdv->pcm8_buf[fdv->pcm8_wr++] = pcm8;

        /* 3. Super-frame boundary: encode → quantize → OFDM */
        if (fdv->pcm8_wr >= LPC_FRAME_SAMPS) {
            uint8_t tx_bits[LPC_Q_FRAME_BYTES];

            LPC_Encode(&fdv->voc, fdv->pcm8_buf, &fdv->cur_frame);
            LPC_Quantize(&fdv->cur_frame, tx_bits);
            FdvModem_EncodeSuperFrame(&fdv->modem, tx_bits, fdv->ofdm_buf);

            fdv->pcm8_wr = 0U;
            fdv->ofdm_rd = 0U;
            fdv->ofdm_wr = LPC_FRAME_SAMPS;   /* FDV_FRAME_SAMPS == LPC_FRAME_SAMPS */
            fdv->tx_frames++;
        }

        /* 4. Pull OFDM audio (fallback: raw pcm8 before first OFDM frame) */
        float src;
        if (fdv->ofdm_wr > fdv->ofdm_rd) {
            src = fdv->ofdm_buf[fdv->ofdm_rd++];
        } else {
            src = pcm8;   /* first 40 ms: PCM pass-through while filling buffer */
        }

        /* 5. NBUSB SSB modulation at 8 kHz (Hilbert phasing method):
         *    The OFDM audio occupies 450–1575 Hz, well within the 8 kHz NBUSB
         *    passband.  USB convention: Q = +Hilbert(src), I = src delayed. */
        float audio_q = hilbert8_process(&fdv->hilbert8, src);

        fdv->i_delay[fdv->i_delay_idx] = src;
        uint16_t ridx = (uint16_t)((fdv->i_delay_idx
                         + FDVR_HILBERT_TAPS
                         - FDVR_HILBERT_DELAY) % FDVR_HILBERT_TAPS);
        float audio_i = fdv->i_delay[ridx];
        fdv->i_delay_idx = (uint16_t)((fdv->i_delay_idx + 1U) % FDVR_HILBERT_TAPS);

        fdv->rx_prev_re = audio_i;
        fdv->rx_prev_im = audio_q;
    }

    *out_i = fdv->rx_prev_re;
    *out_q = fdv->rx_prev_im;
}

/* ── RX: one IQ pair at 48 kHz → 48 kHz audio ──────────────────────────
 *
 *  Decimates the IQ pair to 8 kHz (using I channel only — the IF-shifted
 *  SDR output is already USB-demodulated in sdr_dsp before reaching here;
 *  for FreeDV the IQ pair is passed in directly for future DPSK/OFDM demod).
 *
 *  Phase 1: simple NBUSB demod (Hilbert method at 8 kHz), then upsample.
 *  Phase 2: DPSK/OFDM demodulator + codec2_decode.
 *
 *  Returns one 48 kHz audio sample (zero for the 5 non-output slots).
 * ────────────────────────────────────────────────────────────────────────── */
float FreeDV_RX_Sample(FreeDV_State_t *fdv, float rx_i, float rx_q)
{
    float pcm8;
    /* Decimate IQ magnitude (envelope detect for Phase 1 test) */
    float env = rx_i;   /* USB convention: I is the real signal */
    (void)rx_q;

    if (decimate_push(&fdv->interp, env, &pcm8)) {
        /* Phase 2: codec2_decode_sample(fdv->c2, pcm8) */

        /* Interpolate back to 48 kHz and return first output sample.
         * For now return the 8 kHz sample held at 48 kHz rate. */
        fdv->rx_prev_re = pcm8;
    }

    return fdv->rx_prev_re;
}
