/**
  ******************************************************************************
  * @file    nr_spec.c
  * @brief   NR2 — spectral noise reduction
  *
  *  Algorithm (Ephraim-Malah decision-directed + Wiener gain):
  *   512-pt FFT frames, 50 % overlap, sqrt-Hann WOLA (perfect reconstruction).
  *   Per bin k:
  *     P      = |Y_k|²
  *     λ_k    : noise PSD — adapts down fast when P < 3λ (noise-only
  *              decision), drifts up slowly otherwise so a raised floor is
  *              re-acquired in ~1 s without eating speech.
  *     γ      = P/λ                       (a-posteriori SNR)
  *     ξ      = α·Â²_prev/λ + (1−α)·max(γ−1, 0),  α = 0.98
  *              (decision-directed a-priori SNR — the smoothing that kills
  *               musical noise)
  *     G      = max(ξ/(1+ξ), floor)      (Wiener gain, floored by level)
  *   Gains get a 1-2-1 smoothing across frequency, then are applied
  *   symmetrically to both half-spectra before the inverse FFT.
  *
  *  Cost: one frame burst (fwd+inv FFT + 257-bin update ≈ 0.2 ms on the
  *  480 MHz M7) every 256 samples (5.3 ms) inside CSDR_Loop; ~17 KB of
  *  static state in .bss (DTCM).  No blocking calls, no ISR interaction.
  ******************************************************************************
  */
#include "nr_spec.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define NRS_N      512U
#define NRS_HOP    256U               /* 50 % overlap                */
#define NRS_BINS   (NRS_N / 2U + 1U)  /* real-signal half-spectrum   */
#define NRS_MASK   (NRS_N - 1U)
#define NRS_LOG2N  9U

#define NRS_ALPHA     0.98f   /* decision-directed weight                    */
#define NRS_NOISE_DN  0.05f   /* λ step toward P on noise-only frames        */
#define NRS_NOISE_UP  1.004f  /* λ up-drift per frame ≈ ×2 per second        */
#define NRS_GAMMA_TH  3.0f    /* P < TH·λ ⇒ treat frame bin as noise-only    */

/* Tables (built once in NRSpec_Init) */
static float    s_win[NRS_N];           /* sqrt-Hann analysis = synthesis    */
static float    s_twr[NRS_N / 2U];      /* twiddle cos(2πk/N)                */
static float    s_twi[NRS_N / 2U];      /* twiddle −sin(2πk/N) (forward)     */
static uint16_t s_brev[NRS_N];

/* Signal state */
static float    s_inring[NRS_N];        /* last N input samples              */
static float    s_re[NRS_N], s_im[NRS_N];
static float    s_ola[NRS_N];           /* overlap-add accumulator           */
static float    s_fifo[NRS_HOP];        /* output samples for current hop    */
static float    s_lambda[NRS_BINS];     /* noise PSD estimate per bin        */
static float    s_a2prev[NRS_BINS];     /* Â² = (G·|Y|)² from previous frame */
static float    s_gain[NRS_BINS];
static uint16_t s_widx;
static uint16_t s_fpos;
static float    s_floor = 0.251f;       /* level 50 → −12 dB                 */

void NRSpec_SetLevel(uint8_t level)
{
  if (level > 100U) level = 100U;
  /* 0 → 0 dB (no reduction), 50 → −12 dB, 100 → −24 dB */
  s_floor = powf(10.0f, -0.012f * (float)level);
}

void NRSpec_Reset(void)
{
  memset(s_inring, 0, sizeof(s_inring));
  memset(s_ola,    0, sizeof(s_ola));
  memset(s_fifo,   0, sizeof(s_fifo));
  memset(s_a2prev, 0, sizeof(s_a2prev));
  /* Start λ high: converging down (fast path) beats converging up (slow) —
   * full sensitivity after ~0.3 s instead of eating speech for seconds. */
  for (uint32_t k = 0U; k < NRS_BINS; k++) s_lambda[k] = 1e-2f;
  s_widx = 0U;
  s_fpos = 0U;
}

void NRSpec_Init(void)
{
  for (uint32_t i = 0U; i < NRS_N; i++) {
    float h = 0.5f - 0.5f * cosf(6.28318531f * (float)i / (float)NRS_N);
    s_win[i] = sqrtf(h);
  }
  for (uint32_t k = 0U; k < NRS_N / 2U; k++) {
    float a = 6.28318531f * (float)k / (float)NRS_N;
    s_twr[k] =  cosf(a);
    s_twi[k] = -sinf(a);
  }
  for (uint32_t i = 0U; i < NRS_N; i++) {
    uint32_t r = 0U, v = i;
    for (uint32_t b = 0U; b < NRS_LOG2N; b++) { r = (r << 1) | (v & 1U); v >>= 1; }
    s_brev[i] = (uint16_t)r;
  }
  NRSpec_Reset();
}

/* In-place radix-2 DIT complex FFT, N=512.  inverse=true conjugates the
 * twiddles and scales by 1/N. */
static void nrs_fft(float *re, float *im, bool inverse)
{
  for (uint32_t i = 0U; i < NRS_N; i++) {
    uint32_t j = s_brev[i];
    if (j > i) {
      float t;
      t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (uint32_t len = 2U; len <= NRS_N; len <<= 1) {
    uint32_t half = len >> 1;
    uint32_t step = NRS_N / len;
    for (uint32_t base = 0U; base < NRS_N; base += len) {
      uint32_t ti = 0U;
      for (uint32_t j = 0U; j < half; j++) {
        float wr = s_twr[ti];
        float wi = inverse ? -s_twi[ti] : s_twi[ti];
        ti += step;
        uint32_t a = base + j, b = a + half;
        float tr = re[b] * wr - im[b] * wi;
        float tq = re[b] * wi + im[b] * wr;
        re[b] = re[a] - tr;
        im[b] = im[a] - tq;
        re[a] += tr;
        im[a] += tq;
      }
    }
  }
  if (inverse) {
    const float s = 1.0f / (float)NRS_N;
    for (uint32_t i = 0U; i < NRS_N; i++) { re[i] *= s; im[i] *= s; }
  }
}

static void nrs_frame(void)
{
  /* 1. Windowed frame from the last N samples (oldest first) */
  for (uint32_t i = 0U; i < NRS_N; i++) {
    s_re[i] = s_inring[(s_widx + i) & NRS_MASK] * s_win[i];
    s_im[i] = 0.0f;
  }

  nrs_fft(s_re, s_im, false);

  /* 2. Per-bin noise tracking, decision-directed SNR, Wiener gain */
  for (uint32_t k = 0U; k < NRS_BINS; k++) {
    float P = s_re[k] * s_re[k] + s_im[k] * s_im[k];
    float lam = s_lambda[k];
    if (P < NRS_GAMMA_TH * lam) lam += NRS_NOISE_DN * (P - lam);
    else                        lam *= NRS_NOISE_UP;
    s_lambda[k] = lam;

    float inv_lam = 1.0f / (lam + 1e-12f);
    float gamma   = P * inv_lam;
    float gm1     = gamma - 1.0f;
    if (gm1 < 0.0f) gm1 = 0.0f;
    float xi = NRS_ALPHA * s_a2prev[k] * inv_lam + (1.0f - NRS_ALPHA) * gm1;
    float g  = xi / (1.0f + xi);
    if (g < s_floor) g = s_floor;
    s_gain[k]   = g;
    s_a2prev[k] = g * g * P;
  }

  /* 3. 1-2-1 smoothing across frequency (reduces isolated-bin flutter) */
  float prev = s_gain[0];
  for (uint32_t k = 1U; k < NRS_BINS - 1U; k++) {
    float g = 0.25f * prev + 0.5f * s_gain[k] + 0.25f * s_gain[k + 1U];
    prev = s_gain[k];
    s_gain[k] = g;
  }

  /* 4. Apply gains symmetrically (conjugate half mirrors bins 1..N/2-1) */
  s_re[0]         *= s_gain[0];
  s_im[0]         *= s_gain[0];
  s_re[NRS_N / 2] *= s_gain[NRS_N / 2];
  s_im[NRS_N / 2] *= s_gain[NRS_N / 2];
  for (uint32_t k = 1U; k < NRS_N / 2U; k++) {
    float g = s_gain[k];
    s_re[k] *= g;          s_im[k] *= g;
    s_re[NRS_N - k] *= g;  s_im[NRS_N - k] *= g;
  }

  nrs_fft(s_re, s_im, true);

  /* 5. WOLA synthesis: overlap-add with the same sqrt-Hann, emit one hop */
  for (uint32_t i = 0U; i < NRS_N; i++)
    s_ola[i] += s_re[i] * s_win[i];
  memcpy(s_fifo, s_ola, NRS_HOP * sizeof(float));
  memmove(s_ola, s_ola + NRS_HOP, (NRS_N - NRS_HOP) * sizeof(float));
  memset(s_ola + (NRS_N - NRS_HOP), 0, NRS_HOP * sizeof(float));
}

float NRSpec_Process(float in)
{
  s_inring[s_widx] = in;
  s_widx = (uint16_t)((s_widx + 1U) & NRS_MASK);
  float out = s_fifo[s_fpos];
  if (++s_fpos >= NRS_HOP) {
    nrs_frame();
    s_fpos = 0U;
  }
  return out;
}
