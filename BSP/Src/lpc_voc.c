/**
  ******************************************************************************
  * @file    lpc_voc.c
  * @brief   LPC-10 vocoder implementation for FreeDV
  *
  *  All processing at 8 kHz sample rate, 40 ms (320-sample) frames.
  *
  *  Analysis pipeline per frame:
  *    pre-emphasis → Hamming window → autocorrelation (72-lag) →
  *    lag-window bandwidth expansion → Levinson-Durbin LPC order 10 →
  *    gain = sqrt(prediction error / N) →
  *    pitch via normalized ACF of pre-emphasized speech (lag 20-142)
  *
  *  Synthesis pipeline per sample:
  *    excitation (impulse train or white noise) →
  *    all-pole LPC filter y[n] = e[n] − Σ a[k]·y[n−k] → de-emphasis
  *
  *  Key design choices:
  *    - Lag-window (bandwidth expansion, λ=0.9946) prevents poles from
  *      parking too close to the unit circle → stable synthesis filter.
  *    - Pitch detection on pre-emphasized speech (not residual) avoids the
  *      need for LPC analysis history across frame boundaries.
  *    - Impulse amplitude = gain × √(pitch_samps) so voiced frames have
  *      the same average power per sample as the prediction-error energy.
  *    - De-emphasis IIR is a first-order all-pass shelf (not biquad) so
  *      it cannot become unstable.
  ******************************************************************************
  */

#include "lpc_voc.h"
#include <string.h>
#include <math.h>

/* ── Module-level Hamming window (pre-computed once, shared across calls) ── */
static float   s_hamming[LPC_FRAME_SAMPS];
static uint8_t s_hamming_ready = 0U;

/* ── Static analysis scratch buffers (avoid 2.5 KB ISR stack with 40ms frames) */
static float   s_pe [LPC_FRAME_SAMPS];   /* pre-emphasised speech     */
static float   s_win[LPC_FRAME_SAMPS];   /* pre-emphasised + windowed */

static void ensure_hamming(void)
{
    if (s_hamming_ready) return;
    const float two_pi = 6.28318530717958647f;
    for (uint16_t n = 0U; n < LPC_FRAME_SAMPS; n++) {
        float ph = two_pi * (float)n / (float)(LPC_FRAME_SAMPS - 1U);
        s_hamming[n] = 0.54f - 0.46f * cosf(ph);
    }
    s_hamming_ready = 1U;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal DSP helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Windowed autocorrelation with lag-window bandwidth expansion ────────
 *
 *  Computes R[k] = Σ_{n=k}^{N-1} s[n]·s[n-k]  for k = 0..LPC_ORDER.
 *  Applies lag-window:  R[k] *= λ^k  (λ = 0.9946).
 *  This slightly broadens spectral peaks → poles stay away from unit circle.
 * ────────────────────────────────────────────────────────────────────────── */
static void autocorr_lw(const float *s, float *R)
{
    const float lambda = 0.9946f;

    for (uint16_t k = 0U; k <= LPC_ORDER; k++) {
        float acc = 0.0f;
        for (uint16_t n = k; n < LPC_FRAME_SAMPS; n++)
            acc += s[n] * s[n - k];
        R[k] = acc;
    }

    /* Apply lag-window: R[k] *= λ^k  (R[0] unchanged) */
    float lam = 1.0f;
    for (uint16_t k = 1U; k <= LPC_ORDER; k++) {
        lam *= lambda;
        R[k] *= lam;
    }
}

/* ── Levinson-Durbin recursion ───────────────────────────────────────────
 *
 *  Solves the Yule-Walker equations for the LPC coefficients a[1..p].
 *  Convention: a[0] = 1 (not stored in the reflection-coefficient update;
 *              caller sets frame->a[0] = 1 after this call).
 *
 *  @param  R        Autocorrelation vector R[0..LPC_ORDER]
 *  @param  a        Output: LPC coefficients a[0..LPC_ORDER] (a[0] stays 0 here)
 *  @param  err_out  Output: final prediction error energy (used for gain)
 * ────────────────────────────────────────────────────────────────────────── */
static void levinson_durbin(const float *R, float *a, float *err_out)
{
    float err = R[0] + 1e-8f;   /* white-noise floor prevents /0 on silence */
    float tmp[LPC_ORDER + 1U];
    float k;
    uint16_t m, j;

    for (j = 0U; j <= LPC_ORDER; j++) a[j] = 0.0f;

    for (m = 1U; m <= LPC_ORDER; m++) {
        /* Reflection coefficient k_m */
        float num = R[m];
        for (j = 1U; j < m; j++) num += a[j] * R[m - j];
        k = -num / err;

        /* Hard-clip to unit circle (catches numerical edge cases) */
        if (k >  0.9999f) k =  0.9999f;
        if (k < -0.9999f) k = -0.9999f;

        /* Save current a[1..m-1] into tmp */
        for (j = 1U; j <= LPC_ORDER; j++) tmp[j] = a[j];

        /* Update a[1..m-1] and set a[m] */
        for (j = 1U; j < m; j++) a[j] = tmp[j] + k * tmp[m - j];
        a[m] = k;

        /* Update prediction error */
        err *= (1.0f - k * k);
        if (err < 1e-12f) { err = 1e-12f; break; }
    }

    if (err_out) *err_out = err;
}

/* ── Normalized ACF pitch detector ──────────────────────────────────────
 *
 *  Searches for the lag in [LPC_PITCH_MIN, LPC_PITCH_MAX] that maximizes
 *  the normalized cross-correlation:
 *
 *      r(k) = Σ s[n]·s[n+k]  /  (R[0] + ε)
 *
 *  Note: this is unnormalized by the lagged energy; it biases slightly
 *  toward shorter lags (higher pitch) but works reliably for voiced speech.
 *  True voiced frames have r(T0) > LPC_VOICED_THRESH ≈ 0.35.
 *
 *  @param  s          Pre-emphasized (NOT windowed) speech frame
 *  @param  r0         R[0] from the analysis frame (normalizer)
 *  @param  pitch_out  Best pitch period in samples (LPC_PITCH_MIN..MAX)
 *  @param  corr_out   Normalized peak correlation value
 * ────────────────────────────────────────────────────────────────────────── */
static void pitch_detect(const float *s, float r0,
                         uint16_t *pitch_out, float *corr_out)
{
    const float norm = 1.0f / (r0 + 1e-8f);
    float best_r = -1.0f;
    uint16_t best_lag = LPC_PITCH_MAX;

    for (uint16_t lag = LPC_PITCH_MIN; lag <= LPC_PITCH_MAX; lag++) {
        float acc = 0.0f;
        uint16_t n_valid = LPC_FRAME_SAMPS - lag;
        for (uint16_t n = 0U; n < n_valid; n++)
            acc += s[n] * s[n + lag];
        float r = acc * norm;
        if (r > best_r) {
            best_r    = r;
            best_lag  = lag;
        }
    }

    *pitch_out = best_lag;
    *corr_out  = best_r;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void LPC_Voc_Init(LPC_Voc_t *v)
{
    ensure_hamming();
    memset(v, 0, sizeof(*v));
    v->prng = 0x12345678U;
}

/* ── LPC_Encode ──────────────────────────────────────────────────────────
 *
 *  Fills frame->a[0..10], frame->gain, frame->pitch_samps from 320 PCM
 *  samples at 8 kHz.
 *
 *  Steps:
 *    1. Pre-emphasise (update v->pre_x across frames for continuity).
 *    2. Apply Hamming window.
 *    3. Autocorrelation + lag-window bandwidth expansion.
 *    4. Levinson-Durbin LPC order 10.
 *    5. Gain = sqrt(prediction_error / LPC_FRAME_SAMPS).
 *    6. Pitch detection on pre-emphasised (un-windowed) speech.
 * ────────────────────────────────────────────────────────────────────────── */
void LPC_Encode(LPC_Voc_t *v, const float *pcm_in, LPC_Frame_t *f)
{
    /* s_pe / s_win: module-level statics (avoids 2.5 KB stack with 40ms frames) */
    float R[LPC_ORDER + 1U];
    float err;
    uint16_t pitch;
    float corr;

    /* 1. Pre-emphasis: y[n] = x[n] - α·x[n-1] */
    for (uint16_t n = 0U; n < LPC_FRAME_SAMPS; n++) {
        s_pe[n]   = pcm_in[n] - LPC_PREEMPH * v->pre_x;
        v->pre_x  = pcm_in[n];
    }

    /* 2. Hamming window (analysis only, not for pitch) */
    for (uint16_t n = 0U; n < LPC_FRAME_SAMPS; n++)
        s_win[n] = s_pe[n] * s_hamming[n];

    /* 3. Windowed autocorrelation with lag-window */
    autocorr_lw(s_win, R);

    /* 4. Levinson-Durbin */
    levinson_durbin(R, f->a, &err);
    f->a[0] = 1.0f;

    /* 5. Gain from prediction error energy */
    f->gain = sqrtf(err / (float)LPC_FRAME_SAMPS);

    /* 6. Pitch detection on pre-emphasised (un-windowed) speech */
    pitch_detect(s_pe, R[0], &pitch, &corr);

    if (corr > LPC_VOICED_THRESH) {
        f->pitch_samps = pitch;
    } else {
        f->pitch_samps = 0U;   /* unvoiced */
    }
}

/* ── LPC_Decode ──────────────────────────────────────────────────────────
 *
 *  Synthesises 320 PCM samples from LPC parameters.
 *
 *  Steps per sample:
 *    1. Generate excitation e[n]:
 *         voiced:   impulse train at pitch period, amplitude = gain·√(pitch)
 *                   (ensures average power per sample ≈ gain²)
 *         unvoiced: LCG white noise × gain
 *    2. All-pole synthesis:  y[n] = e[n] − Σ_{k=1}^{p} a[k]·y[n-k]
 *    3. De-emphasis:         s[n] = y[n] + α·s[n-1]
 *         (first-order all-pole, inverse of pre-emphasis)
 * ────────────────────────────────────────────────────────────────────────── */
void LPC_Decode(LPC_Voc_t *v, const LPC_Frame_t *f, float *pcm_out)
{
    const bool voiced = (f->pitch_samps > 0U);

    /* Pre-compute voiced impulse amplitude for constant average power */
    float imp_amp = 0.0f;
    if (voiced)
        imp_amp = f->gain * sqrtf((float)f->pitch_samps);

    for (uint16_t n = 0U; n < LPC_FRAME_SAMPS; n++) {

        /* ── Step 1: Excitation ─────────────────────────────────────── */
        float e;
        if (voiced) {
            v->pitch_phase += 1.0f;
            if (v->pitch_phase >= (float)f->pitch_samps) {
                v->pitch_phase -= (float)f->pitch_samps;
                e = imp_amp;     /* impulse at pitch period */
            } else {
                e = 0.0f;
            }
        } else {
            /* LCG PRNG: Park-Miller (Numerical Recipes variant) */
            v->prng = v->prng * 1664525U + 1013904223U;
            /* Map uint32 to ±1: divide by 2^31 */
            e = (float)(int32_t)v->prng * (1.0f / 2147483648.0f) * f->gain;
        }

        /* ── Step 2: All-pole synthesis filter ─────────────────────── */
        /* y[n] = e[n] − Σ_{k=1}^{p} a[k]·y[n-k]
         * syn_mem[0] = y[n-1], syn_mem[1] = y[n-2], ..., syn_mem[p-1] = y[n-p] */
        float y = e;
        for (uint16_t k = 0U; k < LPC_ORDER; k++)
            y -= f->a[k + 1U] * v->syn_mem[k];

        /* Shift synthesis memory (oldest drops off) */
        for (uint16_t k = LPC_ORDER - 1U; k > 0U; k--)
            v->syn_mem[k] = v->syn_mem[k - 1U];
        v->syn_mem[0] = y;

        /* ── Step 3: De-emphasis ────────────────────────────────────── */
        /* s[n] = y[n] + α·s[n-1]   (inverse of pre-emphasis) */
        float s = y + LPC_DEEMPH * v->de_x;
        v->de_x = s;

        pcm_out[n] = s;
    }
}
