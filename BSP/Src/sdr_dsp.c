/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_dsp.c
  * @brief   SDR DSP Engine
  *
  *  Module:
  *   - NCO   : Numerically Controlled Oscillator (32-bit acc, 1024-entry LUT)
  *   - FIR   : Lowpass filter, Hann-windowed sinc, circular buffer, 64 taps
  *   - IIR   : DC blocker  H(z)=(1-z^-1)/(1-0.995*z^-1)
  *   - AGC   : Peak-hold + hang AGC, 1ms attack, 0.2ms env smoother; SLOW/FAST/AUTO (adaptive); mode presets: SSB 300ms–1s, CW 150ms–400ms, AM 800ms–2s; FM/DIGI bypass
  *   - FFT   : Radix-2 DIT, N=512, Hann window
  *   - DEMOD : AM, FM (atan2 differentiator), USB, LSB, CW (pitch-centred passband, product detector)
  *
  *  RX pipeline per sample:
  *   pre-DC → NCO mix (LO offset) → post-DC → FFT feed → NCO mix (IF shift) → FIR LPF → [S-meter] → Demod → audio LPF → AGC → out
  *
  *  TX pipeline per sample:
  *   USB pull → audio DC → gain → audio FIR LPF → compressor → mod → FFT feed → DAC out
  *
  *  SAI DMA format (STM32H7, DataSize=16 in SlotSize=32):
  *   RX read:  (int16_t)(uint16_t)word          — right-justified, data in bits[15:0]
  *   TX write: (int32_t)(int16_t)sample         — right-justified, data in bits[15:0]
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sdr_dsp.h"
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "runtime_diag.h"
#include "spi_assets.h"
#include "nr_spec.h"
#include "ft8_mode.h"
/* USER CODE END Includes */

/* ── FFT backend selection ──────────────────────────────────────────────────
 * Default: CMSIS-DSP arm_cfft_f32 (SDR_USE_CMSIS_FFT defined below).
 * Fallback: comment out SDR_USE_CMSIS_FFT to revert to the custom radix-2
 *           twiddle-LUT FFT if the CMSIS build proves unstable.
 *
 * CMSIS build requirements (link prebuilt library OR add source files):
 *   ARM_MATH_CM7 preprocessor define (STM32CubeIDE: C/C++ Build → Settings
 *     → MCU GCC Compiler → Preprocessor), or defined here below.
 *   Link: Middlewares/ST/ARM/DSP/Lib/arm_cortexM7lfsp_math.lib
 *   OR add: arm_cfft_f32.c, arm_cfft_radix8_f32.c,
 *           arm_const_structs.c, arm_common_tables.c
 *
 * Complex_f{float re; float im;} is bit-identical to CMSIS float32_t
 * interleaved [Re,Im,...] — arm_cfft_f32 operates in-place, no extra copy.
 * Both backends produce the same natural DFT bin ordering; fftshift semantics
 * in FFT_ComputeMag_dB and spectrum/waterfall display are unchanged. */
#define SDR_USE_CMSIS_FFT

#ifdef SDR_USE_CMSIS_FFT
#ifndef ARM_MATH_CM7
#define ARM_MATH_CM7
#endif
#include "arm_math.h"
#include "arm_const_structs.h"
static const arm_cfft_instance_f32 *s_cfft_inst;
static arm_cfft_instance_f32        s_cfft_ram_inst; /* filled from SPI flash tables */
#endif /* SDR_USE_CMSIS_FFT */

/* USER CODE BEGIN PD */
#define DSP_TWO_PI      6.28318530717958647692f
#define DSP_INV_32767   (1.0f / 32767.0f)
#define DSP_INV_4G      (1.0f / 4294967296.0f)

/* NCO LUT – 1024-entry full-cycle sin table, 4 KB RAM, built once in DSP_Init */
#define NCO_LUT_BITS    10U
#define NCO_LUT_SIZE    (1U << NCO_LUT_BITS)
#define NCO_LUT_MASK    (NCO_LUT_SIZE - 1U)

/* FM constants.
 * TX: ±5 kHz peak deviation at full-scale audio — NBFM wide (25 kHz spacing),
 *     matches TS-2000 FM wide; fits the default 15 kHz RX passband (Carson:
 *     2×(5000+2800) = 15.6 kHz).
 * RX: makeup gain reference — ±6 kHz deviation demodulates to ±1.0 audio.
 *     AGC is bypassed in FM; without makeup, NBFM (±2.5–5 kHz dev) sits
 *     14–20 dB below the AGC-normalised SSB/AM audio level.
 * TX carrier amplitude 0.7 = the "100%" envelope convention of this codebase
 * (see cw_sidetone_amp: "0.35 of 0.7 max"). */
#define FM_TX_DEV_HZ     5000.0f
#define FM_RX_DEV_FS_HZ  6000.0f
#define FM_TX_AMP        0.7f
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static float   s_nco_sin_lut[NCO_LUT_SIZE];
static uint8_t s_nco_lut_init = 0U;

/* Custom twiddle factors — only needed by the fallback radix-2 path.
 * When CMSIS FFT is active these arrays are never read; guard them to
 * recover 2 KB of BSS. */
#ifndef SDR_USE_CMSIS_FFT
static float   s_fft_tw_cos[DSP_FFT_SIZE / 2U];
static float   s_fft_tw_sin[DSP_FFT_SIZE / 2U];
static uint8_t s_fft_tw_init = 0U;
#endif

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static void bit_reverse(Complex_f *buf, uint16_t n);
static void FFT_Precomp(Complex_f *buf, uint16_t n);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void bit_reverse(Complex_f *buf, uint16_t n)
{
  uint16_t i, j = 0U;
  for (i = 1U; i < n; i++)
  {
    uint16_t bit = n >> 1U;
    for (; j & bit; bit >>= 1U) { j ^= bit; }
    j ^= bit;
    if (i < j) { Complex_f tmp = buf[i]; buf[i] = buf[j]; buf[j] = tmp; }
  }
}
/* FFT with precomputed twiddle table — eliminates per-stage sinf/cosf and
 * per-butterfly incremental twiddle rotation (saves ~4 FP mults per butterfly).
 * Requires s_fft_tw_cos/sin built by DSP_Init before first call.
 *
 * When SDR_USE_CMSIS_FFT is defined this function delegates to arm_cfft_f32
 * instead.  Complex_f{float re; float im;} is bit-identical to CMSIS
 * float32_t interleaved [Re,Im,...] so no copy is needed.  The CMSIS call
 * with bitReverseFlag=1 performs bit-reversal internally, so our own
 * bit_reverse() call is skipped on that path.  Output order and fftshift
 * semantics in FFT_ComputeMag_dB are unchanged. */
static void FFT_Precomp(Complex_f *buf, uint16_t n)
{
#ifdef SDR_USE_CMSIS_FFT
  /* twiddleCoef_N[] stores e^{+j2πk/N} (positive exponent, verified: k=1 Im > 0).
   * arm_cfft_f32 forward (ifftFlag=0) therefore computes X[k]=Σx[n]·e^{+j2πkn/N}
   * (IDFT convention), which places a +f₀ tone at bin N−f₀ → mirrored spectrum.
   * Pre-conjugating the input (negate all Im) makes the butterfly compute the
   * standard DFT X[k]=Σx[n]·e^{−j2πkn/N}.  Power |X[k]|² is unchanged by
   * conjugation, so mag_db[] is unaffected except the mirror is corrected. */
  for (uint16_t i = 0U; i < (uint16_t)n; i++) buf[i].im = -buf[i].im;
  arm_cfft_f32(s_cfft_inst, (float32_t *)buf, 0U /*forward*/, 1U /*bitReverse*/);
#else
  bit_reverse(buf, n);
  for (uint16_t len = 2U; len <= n; len <<= 1U) {
    uint16_t step = n / len;          /* twiddle index step into W_N table */
    for (uint16_t i = 0U; i < n; i += len) {
      uint16_t half = len >> 1U;
      for (uint16_t j = 0U; j < half; j++) {
        uint16_t tw   = (uint16_t)(j * step);
        uint16_t even = i + j;
        uint16_t odd  = i + j + half;
        Complex_f u   = buf[even];
        float vr = buf[odd].re * s_fft_tw_cos[tw] - buf[odd].im * s_fft_tw_sin[tw];
        float vi = buf[odd].re * s_fft_tw_sin[tw] + buf[odd].im * s_fft_tw_cos[tw];
        buf[even].re = u.re + vr;  buf[even].im = u.im + vi;
        buf[odd].re  = u.re - vr;  buf[odd].im  = u.im - vi;
      }
    }
  }
#endif /* SDR_USE_CMSIS_FFT */
}
/* USER CODE END 0 */

/* ============================================================
 *  NCO  (LUT-based, no per-sample cosf/sinf)
 * ============================================================ */
void NCO_SetFrequency(NCO_t *nco, int32_t freq_hz, uint32_t sample_rate)
{
  /* USER CODE BEGIN NCO_SetFrequency_0 */
  /* Multiply-before-divide avoids double precision: freq_hz*2^32 stays within
   * int64_t for any freq_hz within Nyquist (max ~24000 * 4294967296 ≈ 1e14).
   * int64_t intermediate gives correct two's-complement for negative frequencies. */
  int64_t inc64 = (int64_t)freq_hz * (int64_t)4294967296LL / (int64_t)sample_rate;
  nco->phase_inc = (uint32_t)inc64;
  nco->phase_acc = 0U;
  nco->cos_val   = 1.0f;
  nco->sin_val   = 0.0f;
  /* USER CODE END NCO_SetFrequency_0 */
}

void NCO_Step(NCO_t *nco)
{
  /* USER CODE BEGIN NCO_Step_0 */
  nco->phase_acc += nco->phase_inc;
  uint32_t idx    = nco->phase_acc >> (32U - NCO_LUT_BITS);
  /* cos(φ) = sin(φ + 90°) → offset LUT index by N/4 */
  nco->cos_val    =  s_nco_sin_lut[(idx + (NCO_LUT_SIZE / 4U)) & NCO_LUT_MASK];
  nco->sin_val    = -s_nco_sin_lut[idx & NCO_LUT_MASK]; /* negative for down-mix */
  /* USER CODE END NCO_Step_0 */
}

/* ============================================================
 *  FIR Lowpass (Hann-windowed sinc)
 * ============================================================ */
void FIR_Init_LPF(FIR_Filter_t *fir, float cutoff_norm, uint16_t taps)
{
  /* USER CODE BEGIN FIR_Init_LPF_0 */
  if (taps > FIR_MAX_TAPS) { taps = FIR_MAX_TAPS; }
  fir->taps = taps;
  fir->idx  = 0U;
  memset(fir->buf, 0, sizeof(fir->buf));

  /* Linear-phase FIR requires an odd tap count (Type I).  With even taps the
   * Hann window zeros both endpoints, leaving one unmatched near-edge tap that
   * breaks perfect symmetry.  Forcing odd ensures n = −M…0…+M with a true
   * centre tap and symmetric coefficient pairs — exact linear phase at the cost
   * of one unused array slot.  Group delay = (taps−1)/2 samples. */
  if ((taps & 1U) == 0U) { taps--; }
  int32_t half = (int32_t)taps / 2;
  float   sum  = 0.0f;
  for (uint16_t i = 0U; i < taps; i++)
  {
    int32_t n = (int32_t)i - half;
    float h = (n == 0) ? (2.0f * cutoff_norm)
                       : (sinf(DSP_TWO_PI * cutoff_norm * (float)n)
                          / (3.14159265f * (float)n));
    float w    = 0.5f - 0.5f * cosf(DSP_TWO_PI * (float)i / (float)(taps - 1U));
    fir->coeff[i] = h * w;
    sum += fir->coeff[i];
  }
  if (fabsf(sum) > 1e-10f) {
    for (uint16_t i = 0U; i < taps; i++) { fir->coeff[i] /= sum; }
  }
  /* USER CODE END FIR_Init_LPF_0 */
}

float FIR_Process(FIR_Filter_t *fir, float x)
{
  /* USER CODE BEGIN FIR_Process_0 */
  fir->buf[fir->idx] = x;
  float    acc = 0.0f;
  uint16_t k   = fir->idx;
  for (uint16_t i = 0U; i < fir->taps; i++)
  {
    acc += fir->coeff[i] * fir->buf[k];
    k    = (k == 0U) ? (fir->taps - 1U) : (k - 1U);
  }
  fir->idx = (uint16_t)(fir->idx + 1U) % fir->taps;
  return acc;
  /* USER CODE END FIR_Process_0 */
}

/* ============================================================
 *  IIR DC Block  H(z) = (1-z^-1)/(1-0.995*z^-1)
 * ============================================================ */
void IIR_DCBlock_Init(IIR_Biquad_t *f)
{
  /* USER CODE BEGIN IIR_DCBlock_Init_0 */
  f->b0 = 1.0f; f->b1 = -1.0f; f->b2 = 0.0f;
  f->a1 = -0.995f; f->a2 = 0.0f;
  f->x1 = 0.0f; f->x2 = 0.0f;
  f->y1 = 0.0f; f->y2 = 0.0f;
  /* USER CODE END IIR_DCBlock_Init_0 */
}

float IIR_DCBlock_Process(IIR_Biquad_t *f, float x)
{
  /* USER CODE BEGIN IIR_DCBlock_Process_0 */
  float y = f->b0 * x + f->b1 * f->x1
            - f->a1 * f->y1;
  f->x1 = x;
  f->y1 = y;
  return y;
  /* USER CODE END IIR_DCBlock_Process_0 */
}

void IIR_HP1_Init(IIR_Biquad_t *f, float fc_hz, uint32_t sample_rate)
{
  float K     = tanf(3.14159265f * fc_hz / (float)sample_rate);
  float alpha = 1.0f / (1.0f + K);
  f->b0 = alpha;  f->b1 = -alpha;  f->b2 = 0.0f;
  f->a1 = (K - 1.0f) * alpha;  f->a2 = 0.0f;
  f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

float IIR_Biquad_Process(IIR_Biquad_t *f, float x)
{
  float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2
           - f->a1*f->y1 - f->a2*f->y2;
  f->x2 = f->x1; f->x1 = x;
  f->y2 = f->y1; f->y1 = y;
  return y;
}

void Notch_Init(IIR_Biquad_t *f, float fc_hz, uint32_t sample_rate)
{
  /* Standard biquad notch: zeros at ±jω₀, poles at r·e^±jω₀ (r=0.97 → narrow) */
  float r  = 0.97f;
  float w0 = 2.0f * 3.14159265f * fc_hz / (float)sample_rate;
  float c  = cosf(w0);
  f->b0 = 1.0f;  f->b1 = -2.0f * c;  f->b2 = 1.0f;
  f->a1 = -2.0f * r * c;              f->a2 = r * r;
  f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

/* RBJ audio-EQ-cookbook low/high shelf, shelf slope S=1 (max slope, no
 * passband ripple). gain_db=0 collapses to an exact unity pass-through. */
static void Shelf_Init(IIR_Biquad_t *f, float fc_hz, float gain_db,
                        uint32_t sample_rate, bool high_shelf)
{
  float A      = powf(10.0f, gain_db / 40.0f);
  float w0     = 2.0f * 3.14159265f * fc_hz / (float)sample_rate;
  float cosw0  = cosf(w0);
  float alpha  = sinf(w0) * 0.70710678f;   /* sin(w0)/2 * sqrt(2), S=1 */
  float twoSqrtAalpha = 2.0f * sqrtf(A) * alpha;
  float b0, b1, b2, a0, a1, a2;

  if (high_shelf) {
    b0 =      A * ((A + 1.0f) + (A - 1.0f) * cosw0 + twoSqrtAalpha);
    b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosw0);
    b2 =      A * ((A + 1.0f) + (A - 1.0f) * cosw0 - twoSqrtAalpha);
    a0 =           (A + 1.0f) - (A - 1.0f) * cosw0 + twoSqrtAalpha;
    a1 =    2.0f * ((A - 1.0f) - (A + 1.0f) * cosw0);
    a2 =           (A + 1.0f) - (A - 1.0f) * cosw0 - twoSqrtAalpha;
  } else {
    b0 =      A * ((A + 1.0f) - (A - 1.0f) * cosw0 + twoSqrtAalpha);
    b1 =  2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosw0);
    b2 =      A * ((A + 1.0f) - (A - 1.0f) * cosw0 - twoSqrtAalpha);
    a0 =           (A + 1.0f) + (A - 1.0f) * cosw0 + twoSqrtAalpha;
    a1 =   -2.0f * ((A - 1.0f) + (A + 1.0f) * cosw0);
    a2 =           (A + 1.0f) + (A - 1.0f) * cosw0 - twoSqrtAalpha;
  }

  f->b0 = b0 / a0; f->b1 = b1 / a0; f->b2 = b2 / a0;
  f->a1 = a1 / a0; f->a2 = a2 / a0;
  f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

/* Bass corner ~300 Hz (below voice fundamentals), treble corner ~2.5 kHz
 * (presence/consonant range) — typical comms-radio tone-control points. */
#define DSP_BASS_SHELF_HZ    300.0f
#define DSP_TREBLE_SHELF_HZ 2500.0f

void DSP_SetTone(DSP_State_t *dsp, int8_t bass_db, int8_t treble_db)
{
  if (bass_db   < -10) bass_db   = -10;
  if (bass_db   >  10) bass_db   =  10;
  if (treble_db < -10) treble_db = -10;
  if (treble_db >  10) treble_db =  10;
  uint32_t sr = dsp->sample_rate ? dsp->sample_rate : 48000U;
  Shelf_Init(&dsp->bass_shelf,   DSP_BASS_SHELF_HZ,   (float)bass_db,   sr, false);
  Shelf_Init(&dsp->treble_shelf, DSP_TREBLE_SHELF_HZ, (float)treble_db, sr, true);
}

/* ============================================================
 *  AGC
 * ============================================================ */
void AGC_Init(AGC_t *agc, uint32_t sample_rate)
{
  /* USER CODE BEGIN AGC_Init_0 */
  agc->gain       = 1.0f;
  agc->target     = 0.15f;
  agc->max_gain   = 64.0f;
  agc->min_gain   = 0.01f;
  agc->level      = 0.0f;
  agc->env_smooth = 0.0f;
  agc->hang_timer = 0U;
  agc->bypass     = false;
  agc->auto_mode  = false;
  agc->prev_level = 0.0f;
  agc->drate      = 0.0f;
  /* Default to SSB-slow constants; caller overrides via AGC_SetMode() */
  agc->attack     = expf(-1.0f / (0.001f * (float)sample_rate));  /* 1 ms   */
  agc->decay      = expf(-1.0f / (1.000f * (float)sample_rate));  /* 1 s    */
  agc->hang_time  = (uint32_t)(0.200f * (float)sample_rate);      /* 200 ms */
  agc->decay_fast = agc->decay;
  agc->decay_slow = agc->decay;
  agc->hang_fast  = agc->hang_time;
  agc->hang_slow  = agc->hang_time;
  /* USER CODE END AGC_Init_0 */
}

/* ============================================================
 *  Noise Blanker
 *
 *  Configure the HF impulse noise blanker.  Safe to call from the main loop
 *  while DSP_Process() is not running (both share the same CSDR_Loop context).
 *
 *  level 0-100 controls threshold aggressiveness:
 *    0  → ratio=32 amplitude (~30 dB above noise floor), only ADC-saturating spikes
 *    50 → ratio=18           (~25 dB), typical PSU / ignition noise   [default]
 *   100 → ratio=4            (~12 dB), aggressive – may false-trigger on strong SSB
 *
 *  blank_width: 2 samples at low levels, 6 at max (~42-125 µs at 48 kHz).
 *  Blanked samples are ramped between the pre/post-impulse anchors (see
 *  NoiseBlanker_t doc) rather than zeroed, so no FIR smoothing is needed.
 *
 *  On first enable, floor_sq is seeded from agc.level² so the threshold
 *  starts conservatively high and descends to the real noise floor over ~200 ms.
 *  The interpolation delay line is also re-primed (see prime_ctr) so it
 *  never plays out stale ring contents from a previous enable.
 * ============================================================ */
void DSP_NB_Set(DSP_State_t *dsp, bool enabled, uint8_t level)
{
  if (level > 100U) { level = 100U; }

  /* Amplitude ratio: trigger when instantaneous IQ magnitude exceeds
   * ratio × estimated noise floor.  Precomputed as ratio² to avoid sqrtf
   * in the per-sample hot path.  Linear interpolation in R gives a
   * perceptually even response across the level knob. */
  float r = 32.0f - (float)level * 0.28f;  /* 32.0 at 0, 4.0 at 100 */
  if (r < 2.0f) { r = 2.0f; }
  dsp->nb.threshold_ratio_sq = r * r;

  /* Blanking window width in samples (precomputed, not per-sample) */
  uint8_t w = (uint8_t)(2U + (uint32_t)level * 4U / 100U);
  dsp->nb.blank_width = (w > 6U) ? 6U : w;

  if (enabled && !dsp->nb.enabled) {
    /* Seed floor_sq from agc.level (demodulated signal envelope proxy) so the
     * initial threshold is well above the current signal.  This prevents false
     * blanking during the ~200 ms IIR settling period after enable. */
    float a = dsp->agc.level;
    dsp->nb.floor_sq        = (a > 0.0f) ? (a * a) : 1e-6f;
    dsp->nb.blank_ctr       = 0U;
    dsp->nb.awaiting_anchor = false;
    dsp->nb.widx            = 0U;
    dsp->nb.prime_ctr       = NB_RING_LEN;  /* re-fill delay line before use */
  } else if (!enabled) {
    dsp->nb.blank_ctr       = 0U;
    dsp->nb.awaiting_anchor = false;
  }

  dsp->nb.level   = level;
  dsp->nb.enabled = enabled;
}

/* ── NLMS adaptive-filter engine (shared by NR1 and BC) ────────────────────
 *
 *  Adaptive 32-tap predictor: estimates the current sample from samples
 *  Δ..Δ+31 in the past.  With Δ past the noise correlation time, only
 *  long-correlation content (voice pitch, steady carriers) is predictable →
 *  lands in prediction y; band-limited noise stays in error e = in − y.
 *  The decorrelation delay Δ is essential: predicting from x[n] itself would
 *  let the filter converge to the identity (w[0]=1) and stop separating.
 *
 *  NLMS: step scaled by µ / (TAPS·P̂ + ε) where P̂ is an IIR power estimate,
 *  so convergence is level-independent.  Tiny leakage on w prevents weight
 *  drift during long silences.
 *
 *  Returns error e; prediction y via *pred.  NR1 blends in→y by wet;
 *  BC uses e directly (predictable tones subtracted = auto-notch).
 * ─────────────────────────────────────────────────────────────────────────*/
static float lms_run(LMS_t *s, float in, float *pred)
{
  const uint16_t mask = LMS_LINE_LEN - 1U;
  s->x[s->widx] = in;

  /* Reference vector = x[n-Δ], x[n-Δ-1], … (newest-first from Δ back) */
  uint16_t base = (uint16_t)((s->widx + LMS_LINE_LEN - s->delay) & mask);
  float y = 0.0f;
  uint16_t idx = base;
  for (uint32_t i = 0U; i < LMS_TAPS; i++) {
    y += s->w[i] * s->x[idx];
    idx = (uint16_t)((idx + mask) & mask);   /* idx-- mod LINE_LEN */
  }

  float e = in - y;
  s->pow_est += 0.01f * (in * in - s->pow_est);
  float mu_e = e * (s->mu / ((float)LMS_TAPS * s->pow_est + 1e-9f));

  idx = base;
  for (uint32_t i = 0U; i < LMS_TAPS; i++) {
    s->w[i] = s->w[i] * (1.0f - 1e-5f) + mu_e * s->x[idx];
    idx = (uint16_t)((idx + mask) & mask);
  }

  s->widx = (uint16_t)((s->widx + 1U) & mask);
  *pred = y;
  return e;
}

static void lms_reset(LMS_t *s)
{
  memset(s->w, 0, sizeof(s->w));
  memset(s->x, 0, sizeof(s->x));
  s->widx    = 0U;
  s->pow_est = 1e-4f;  /* non-zero so the NLMS denominator is sane at start */
}

/* mode 0=off, 1=NR1 (LMS line enhancer), 2=NR2 (spectral, see nr_spec.c).
 * level 0-100: NR1 dry/wet mix (100 = pure prediction); NR2 suppression
 * depth (100 = −24 dB floor).  Scalar stores only; safe from the main loop. */
void DSP_NR_Set(DSP_State_t *dsp, uint8_t mode, uint8_t level)
{
  if (mode  > 2U)   mode  = 2U;
  if (level > 100U) level = 100U;
  dsp->nr.wet   = (float)level * 0.01f;
  dsp->nr.mu    = 0.05f;
  dsp->nr.delay = 32U;   /* ≈ fs/BW for 1.5-3 kHz noise at 48 kHz */
  NRSpec_SetLevel(level);
  if (mode != dsp->nr_mode) {
    /* Clean start on any mode change so filters reconverge from scratch */
    lms_reset(&dsp->nr);
    if (mode == 2U) NRSpec_Reset();
  }
  dsp->nr.enabled = (mode == 1U);
  dsp->nr_mode    = mode;
}

/* Beat canceller: mode 0=off, 1/2=on (TS-2000 BC1/BC2 — same engine here) */
void DSP_BC_Set(DSP_State_t *dsp, uint8_t mode)
{
  bool en = (mode > 0U);
  dsp->bc.mu    = 0.15f;  /* faster than NR1 — track drifting carriers */
  dsp->bc.delay = 96U;    /* 2 ms: speech decorrelated, steady tones remain */
  if (en && !dsp->bc.enabled)
    lms_reset(&dsp->bc);
  dsp->bc.enabled = en;
}

/* Set mode-specific AGC timing and bypass.  Primary configuration entry point.
 * speed: 0=SLOW, 1=FAST, 2=AUTO (adaptive).
 *
 * SSB (USB/LSB):  1 ms attack, 200 ms hang (slow) / 100 ms (fast), 1 s / 300 ms decay.
 * CW:             1 ms attack,  75 ms hang (slow) /  25 ms (fast), 400 ms / 150 ms decay.
 * AM:             1 ms attack, 500 ms hang (slow) / 300 ms (fast), 2 s / 800 ms decay.
 * AUTO:           blend between fast/slow constants based on |d(level)/dt|, τ≈70ms.
 * FM / DIGI:      bypass=true, gain=1.0 — constant amplitude, AGC causes pumping.
 */
void AGC_SetMode(AGC_t *agc, SDR_Mode_t mode, uint8_t speed, uint32_t sample_rate)
{
  float sr = (float)sample_rate;
  agc->attack    = expf(-1.0f / (0.001f * sr));   /* 1 ms always */
  agc->auto_mode = (speed == 2U);

  switch (mode)
  {
    case MODE_FM:
    case MODE_DIGU:
    case MODE_DIGL:
      agc->bypass     = true;
      agc->gain       = 1.0f;
      agc->hang_timer = 0U;
      agc->auto_mode  = false;
      return;

    case MODE_CW:
      agc->decay_fast = expf(-1.0f / (0.150f * sr));
      agc->decay_slow = expf(-1.0f / (0.400f * sr));
      agc->hang_fast  = (uint32_t)(0.025f * sr);
      agc->hang_slow  = (uint32_t)(0.075f * sr);
      break;

    case MODE_AM:
      agc->decay_fast = expf(-1.0f / (0.800f * sr));
      agc->decay_slow = expf(-1.0f / (2.000f * sr));
      agc->hang_fast  = (uint32_t)(0.300f * sr);
      agc->hang_slow  = (uint32_t)(0.500f * sr);
      break;

    case MODE_USB:
    case MODE_LSB:
    default:
      agc->decay_fast = expf(-1.0f / (0.300f * sr));
      agc->decay_slow = expf(-1.0f / (1.000f * sr));
      agc->hang_fast  = (uint32_t)(0.100f * sr);
      agc->hang_slow  = (uint32_t)(0.200f * sr);
      break;
  }
  agc->bypass = false;
  if (agc->auto_mode) {
    /* Start at slow; process loop adapts from here */
    agc->decay     = agc->decay_slow;
    agc->hang_time = agc->hang_slow;
    agc->drate     = 0.0f;
    agc->prev_level = 0.0f;
  } else {
    bool fast      = (speed == 1U);
    agc->decay     = fast ? agc->decay_fast : agc->decay_slow;
    agc->hang_time = fast ? agc->hang_fast  : agc->hang_slow;
  }
}

/* Compatibility wrapper — applies SSB-equivalent timing when mode is not known.
 * Prefer AGC_SetMode() for all new call sites. */
void AGC_SetSpeed(AGC_t *agc, uint8_t speed, uint32_t sample_rate)
{
  AGC_SetMode(agc, MODE_USB, speed, sample_rate);
}

float AGC_Process(AGC_t *agc, float x)
{
  /* USER CODE BEGIN AGC_Process_0 */
  /* FM and DIGI: bypass completely – AGC pumping on constant-amplitude
   * or narrow digital signals causes noise breathing / amplitude wander. */
  if (agc->bypass) { return x; }

  /* Smooth the instantaneous magnitude with a short IIR (τ ≈ 0.2 ms at 48 kHz).
   * α=0.9 → single-sample click only moves env_smooth to 10% of its amplitude,
   * preventing a one-sample transient from triggering a full AGC attack cycle. */
  agc->env_smooth = 0.9f * agc->env_smooth + 0.1f * fabsf(x);
  float env = agc->env_smooth;

  if (env >= agc->level) {
    /* Attack: signal rising – fast approach, reset hang timer */
    agc->level      = agc->attack * agc->level + (1.0f - agc->attack) * env;
    agc->hang_timer = agc->hang_time;
  } else if (agc->hang_timer > 0U) {
    /* Hang: hold level constant – no pumping between syllables */
    agc->hang_timer--;
  } else {
    /* Decay: slow release after hang expires */
    agc->level = agc->decay * agc->level;
  }

  /* AUTO mode: blend decay/hang_time by rate-of-change of level.
   * drate tracks |d(level)/dt| with τ≈70ms; high drate → fast signal → faster release. */
  if (agc->auto_mode) {
    float dlevel = fabsf(agc->level - agc->prev_level);
    agc->drate = 0.9997f * agc->drate + 0.0003f * dlevel;   /* τ ≈ 70 ms at 48 kHz */
    float blend = agc->drate / (agc->drate + 0.0008f);       /* 0=slow … 1=fast     */
    agc->decay = agc->decay_slow + blend * (agc->decay_fast - agc->decay_slow);
    agc->hang_time = agc->hang_fast +
                     (uint32_t)((1.0f - blend) * (float)(agc->hang_slow - agc->hang_fast));
  }
  agc->prev_level = agc->level;

  if (agc->level > 1e-10f) { agc->gain = agc->target / agc->level; }
  if (agc->gain > agc->max_gain) { agc->gain = agc->max_gain; }
  if (agc->gain < agc->min_gain) { agc->gain = agc->min_gain; }
  return x * agc->gain;
  /* USER CODE END AGC_Process_0 */
}

/* ============================================================
 *  FFT Radix-2 DIT
 * ============================================================ */
void FFT_Hann_Window(float *w, uint16_t n)
{
  /* USER CODE BEGIN FFT_Hann_Window_0 */
  for (uint16_t i = 0U; i < n; i++)
    w[i] = 0.5f - 0.5f * cosf(DSP_TWO_PI * (float)i / (float)(n - 1U));
  /* USER CODE END FFT_Hann_Window_0 */
}

void FFT_Radix2(Complex_f *buf, uint16_t n)
{
  /* USER CODE BEGIN FFT_Radix2_0 */
  bit_reverse(buf, n);
  for (uint16_t len = 2U; len <= n; len <<= 1U)
  {
    float ang = -DSP_TWO_PI / (float)len;
    float wr  = cosf(ang);
    float wi  = sinf(ang);
    for (uint16_t i = 0U; i < n; i += len)
    {
      float cr = 1.0f, ci = 0.0f;
      for (uint16_t j = 0U; j < (len >> 1U); j++)
      {
        uint16_t  even = i + j;
        uint16_t  odd  = i + j + (len >> 1U);
        Complex_f u    = buf[even];
        Complex_f v;
        v.re = buf[odd].re * cr - buf[odd].im * ci;
        v.im = buf[odd].re * ci + buf[odd].im * cr;
        buf[even].re = u.re + v.re; buf[even].im = u.im + v.im;
        buf[odd].re  = u.re - v.re; buf[odd].im  = u.im - v.im;
        float new_cr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = new_cr;
      }
    }
  }
  /* USER CODE END FFT_Radix2_0 */
}

void FFT_ComputeMag_dB(const Complex_f *buf, float *mag_db,
                        uint16_t half_n, float *peak_db)
{
  /* USER CODE BEGIN FFT_ComputeMag_0 */
  /* Store raw power (re²+im²) — no sqrt, no log.
   * Compression to log-like display range is done by pwr_compress() in sdr_ui.c
   * at render time (~13 fps × SPEC_W px) rather than here (~94 fps × 512 bins). */
  const uint16_t n = half_n;
  const uint16_t h = n / 2U;
  *peak_db = 0.0f;
  /* fftshift: negative half buf[h..n-1] → mag_db[0..h-1] */
  for (uint16_t i = 0U; i < h; i++)
  {
    float pwr = buf[h+i].re * buf[h+i].re + buf[h+i].im * buf[h+i].im;
    mag_db[i] = pwr;
    if (pwr > *peak_db) { *peak_db = pwr; }
  }
  /* positive half buf[0..h-1] → mag_db[h..n-1] */
  for (uint16_t i = 0U; i < h; i++)
  {
    float pwr = buf[i].re * buf[i].re + buf[i].im * buf[i].im;
    mag_db[h+i] = pwr;
    if (pwr > *peak_db) { *peak_db = pwr; }
  }
  /* USER CODE END FFT_ComputeMag_0 */
}

/* ============================================================
 *  Demodulators
 * ============================================================ */
float Demod_AM(float i, float q)
{
  /* USER CODE BEGIN Demod_AM_0 */
  return sqrtf(i * i + q * q);
  /* USER CODE END Demod_AM_0 */
}

float Demod_FM(FM_Demod_t *fm, float i, float q)
{
  /* USER CODE BEGIN Demod_FM_0 */
  float re = i * fm->prev_re + q * fm->prev_im;
  float im = q * fm->prev_re - i * fm->prev_im;
  fm->prev_re = i;
  fm->prev_im = q;
  /* scale (set in DSP_Init) = -fs/(2π·FM_RX_DEV_FS_HZ):
   *   sign  — hardware QSD gives Q=-sin (baseband exp(-jωt)), so atan2 yields
   *           -Δφ; negation restores true frequency polarity (FSK data safe).
   *   magnitude — ±FM_RX_DEV_FS_HZ deviation → ±1.0 audio makeup gain,
   *           since AGC is bypassed in FM. */
  float d = atan2f(im, re) * fm->scale;
  float y = fm->de_emph.b0 * d - fm->de_emph.a1 * fm->de_emph.y1;
  fm->de_emph.y1 = y;
  return y;
  /* USER CODE END Demod_FM_0 */
}

/* Both functions receive hq = Hilbert{-Q} (Q negated before Hilbert call).
 * Hardware QSD outputs Q=-sin for +f; negation restores +sin convention.
 * For a USB signal  (+f): H{-Q}=H{+sin}=−cos=−I → (I−(−I))×0.5 = I  ✓, LSB = 0 ✓
 * For an LSB signal (−f): H{-Q}=H{−sin}=+cos=+I → (I−(+I))×0.5 = 0  ✓, LSB = I ✓ */
float Demod_USB(float i, float hq) { return (i - hq) * 0.5f; }
float Demod_LSB(float i, float hq) { return (i + hq) * 0.5f; }

/* ============================================================
 *  DSP Engine – init / config
 * ============================================================ */
void DSP_Init(DSP_State_t *dsp, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_Init_0 */
  memset(dsp, 0, sizeof(DSP_State_t));
  dsp->mode        = MODE_AM;
  dsp->sample_rate = sample_rate;

  /* Build NCO LUT once (shared across all DSP instances via static) */
  if (!s_nco_lut_init) {
    for (uint32_t i = 0U; i < NCO_LUT_SIZE; i++)
      s_nco_sin_lut[i] = sinf(DSP_TWO_PI * (float)i / (float)NCO_LUT_SIZE);
    s_nco_lut_init = 1U;
  }

#ifndef SDR_USE_CMSIS_FFT
  if (!s_fft_tw_init) {
    for (uint16_t k = 0U; k < (DSP_FFT_SIZE / 2U); k++) {
      float ang = -DSP_TWO_PI * (float)k / (float)DSP_FFT_SIZE;
      s_fft_tw_cos[k] = cosf(ang);
      s_fft_tw_sin[k] = sinf(ang);
    }
    s_fft_tw_init = 1U;
  }
#endif

#ifdef SDR_USE_CMSIS_FFT
  /* Use twiddle/bitrev tables loaded from SPI flash into RAM when available.
   * This makes arm_cfft_sR_f32_len512 (and its ~5 KB of rodata) unreferenced
   * once SPI_ASSETS_FFT_FALLBACK is set to 0 in spi_assets.h. */
  if (SPI_Assets_IsLoaded(SPI_ASSET_FFT_TWIDDLE) &&
      SPI_Assets_IsLoaded(SPI_ASSET_FFT_BITREV)) {
    s_cfft_ram_inst.fftLen       = DSP_FFT_SIZE;
    s_cfft_ram_inst.pTwiddle     = (const float32_t *)SPI_Assets_GetBuf(SPI_ASSET_FFT_TWIDDLE);
    s_cfft_ram_inst.pBitRevTable = (const uint16_t  *)SPI_Assets_GetBuf(SPI_ASSET_FFT_BITREV);
    s_cfft_ram_inst.bitRevLength  = SPI_FFT_BITREV_LEN;
    s_cfft_inst = &s_cfft_ram_inst;
  } else {
#if SPI_ASSETS_FFT_FALLBACK
    s_cfft_inst = &arm_cfft_sR_f32_len512;
#endif
  }
#endif

  NCO_SetFrequency(&dsp->nco,    0, sample_rate);
  NCO_SetFrequency(&dsp->nco_if, 0, sample_rate);
  /* CW post-FIR re-centre shift: +pitch (CW-N).  Kept in sync by
   * DSP_SetCWPitch / DSP_SetCWReverse; memset above cleared cw_reverse. */
  dsp->cw_pitch_hz = 700U;
  NCO_SetFrequency(&dsp->nco_cw_shift, 700, sample_rate);

  float bw = 6000.0f / (float)sample_rate;
  FIR_Init_LPF(&dsp->fir_i,     bw, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q,     bw, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_audio, 4000.0f / (float)sample_rate, 32U);

  IIR_DCBlock_Init(&dsp->dc_block_audio);
  IIR_DCBlock_Init(&dsp->dc_postmix_i);
  IIR_DCBlock_Init(&dsp->dc_postmix_q);

  /* IQ correction: identity until DSP_SetIQCorr() is called */
  dsp->iq_g_inv = 1.0f;
  dsp->iq_p     = 0.0f;

  /* Static DC offset: 0 until DSP_SetDCOffset() is called */
  dsp->dc_i_static       = 0.0f;
  dsp->dc_q_static       = 0.0f;
  dsp->cal_meas.mode     = DSP_CAL_IDLE;
  dsp->cal_meas.done     = false;

  AGC_Init(&dsp->agc, sample_rate);
  FFT_Hann_Window(dsp->fft_window, DSP_FFT_SIZE);

  for (int s = 0; s < 3; s++) {
    FIR_Init_LPF(&dsp->spec_dec_i[s], 0.25f, 31U);
    FIR_Init_LPF(&dsp->spec_dec_q[s], 0.25f, 31U);
    dsp->spec_decim_cnt[s] = 0U;
  }
  dsp->spec_decim       = 1U;
  dsp->spec_decim_flush = 0U;

  {
    float de_fc          = 2122.0f / (float)sample_rate;
    dsp->fm.de_emph.b0   = 1.0f - expf(-DSP_TWO_PI * de_fc);
    dsp->fm.de_emph.a1   = -(1.0f - dsp->fm.de_emph.b0);
    dsp->fm.de_emph.y1   = 0.0f;
    dsp->fm.prev_re      = 0.0f;
    dsp->fm.prev_im      = 0.0f;
    /* Polarity fix + makeup gain — see Demod_FM */
    dsp->fm.scale        = -(float)sample_rate / (DSP_TWO_PI * FM_RX_DEV_FS_HZ);
    /* TX pre-emphasis = exact inverse of the RX de-emphasis one-pole:
     * H⁻¹(z) = (1 - (1-b0)·z⁻¹)/b0 — TX→RX audio round trip is flat. */
    dsp->tx.fm_pre_k     = 1.0f - dsp->fm.de_emph.b0;
    dsp->tx.fm_pre_gain  = 1.0f / dsp->fm.de_emph.b0;
  }

  Hilbert_Init(&dsp->rx_hilbert);
  memset(dsp->rx_i_delay, 0, sizeof(dsp->rx_i_delay));
  dsp->rx_delay_idx = 0U;

  Hilbert_Init(&dsp->tx.hilbert);
  memset(dsp->tx.audio_delay, 0, sizeof(dsp->tx.audio_delay));
  dsp->tx.delay_idx    = 0;
  dsp->tx.fm_phase_acc = 0U;
  dsp->tx.fm_pre_x1    = 0.0f;
  /* Full-scale audio (±1.0) → NCO increment for ±FM_TX_DEV_HZ deviation */
  dsp->tx.fm_dev_scale = FM_TX_DEV_HZ * 4294967296.0f / (float)sample_rate;
  dsp->tx.drive_gain   = 0.0f;
  dsp->tx.cw_phase_acc    = 0;
  dsp->tx.cw_bfo_inc      = (uint32_t)((int64_t)700 * (int64_t)4294967296LL / (int64_t)sample_rate);
  dsp->tx.cw_sidetone_amp = 0.35f;  /* 50% vol default → 0.35 of 0.7 max */
  dsp->tx.cw_env_amp      = 0.0f;
  dsp->tx.audio_gain   = 1.0f;
  dsp->tx.tx_lp_hz     = 2800.0f;
  dsp->tx.tx_hp_hz     = 200.0f;
  FIR_Init_LPF(&dsp->tx.fir_audio, 2800.0f / (float)sample_rate, 32U);
  IIR_DCBlock_Init(&dsp->tx.dc_block);
  IIR_HP1_Init(&dsp->tx.hp_audio, 200.0f, sample_rate);
  dsp->tx.comp_env    = 0.0f;
  dsp->tx.comp_attack = expf(-1.0f / (0.001f * (float)sample_rate)); /* 1 ms  */
  dsp->tx.comp_decay  = expf(-1.0f / (0.050f * (float)sample_rate)); /* 50 ms */

  CWEnv_Init(&dsp->cw_env, sample_rate);

  dsp->signal_power_db      = -120.0f;
  dsp->squelch_threshold_db = -200.0f;
  dsp->squelch_open         = true;
  dsp->notch_on = false;
  dsp->notch_hz = 1000.0f;
  Notch_Init(&dsp->notch, 1000.0f, sample_rate);
  DSP_SetTone(dsp, 0, 0);   /* flat by default */
  dsp->rx_volume_scale = 1.0f;
  /* USER CODE END DSP_Init_0 */
}

void DSP_SetFrequency(DSP_State_t *dsp, uint32_t lo_offset_hz, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_SetFrequency_0 */
  /* LO = VFO + lo_offset_hz, so the ADC sees the signal at −lo_offset_hz from DC.
   * Shift it to 0 by rotating at +lo_offset_hz, i.e. NCO at −lo_offset_hz
   * (because NCO_Step applies exp(−j·ω·t)). */
  NCO_SetFrequency(&dsp->nco, -(int32_t)lo_offset_hz, sample_rate);
  /* USER CODE END DSP_SetFrequency_0 */
}

/* sq=0 → disabled; sq=1-100 → threshold -70 to 0 dBFS (S1 to clipping) */
void DSP_SetSquelch(DSP_State_t *dsp, uint8_t sq)
{
  if (sq == 0U) {
    dsp->squelch_threshold_db = -200.0f;
    dsp->squelch_open         = true;
  } else {
    dsp->squelch_threshold_db = -70.0f + (float)(sq - 1U) * (70.0f / 99.0f);
  }
}

void DSP_SetIFShift(DSP_State_t *dsp, int32_t if_shift_hz, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_SetIFShift_0 */
  NCO_SetFrequency(&dsp->nco_if, if_shift_hz, sample_rate);
  /* USER CODE END DSP_SetIFShift_0 */
}

void DSP_SetMode(DSP_State_t *dsp, SDR_Mode_t mode, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_SetMode_0 */
  dsp->mode        = mode;
  dsp->sample_rate = sample_rate;

  /* Reset compressor when entering a voice mode from a digital mode so it
   * re-tracks from zero rather than inheriting the settled tone level. */
  if (mode != MODE_DIGU && mode != MODE_DIGL)
    dsp->tx.comp_env = 0.0f;

  float mode_bw_hz;
  switch (mode)
  {
    case MODE_AM:     mode_bw_hz = 6000.0f;  break;
    case MODE_FM:     mode_bw_hz = 15000.0f; break;
    case MODE_USB:
    case MODE_LSB:
    case MODE_DIGU:
    case MODE_DIGL:   mode_bw_hz = 3000.0f;  break;
    case MODE_CW:     mode_bw_hz = 500.0f;   break;
    default:          mode_bw_hz = 4000.0f;  break;
  }
  /* Never overwrite a caller-set bw_hz — only seed it on first init (bw_hz==0).
   * The caller is responsible for calling DSP_SetBW() with the desired BW after
   * this function; that is the sole place that updates dsp->bw_hz. */
  if (dsp->bw_hz <= 0.0f) dsp->bw_hz = mode_bw_hz;

  /* Same CW half-width rule as DSP_SetBW (bw_hz = full width around pitch) */
  float cutoff  = (mode == MODE_CW) ? (dsp->bw_hz * 0.5f) : dsp->bw_hz;
  float bw_norm = cutoff / (float)sample_rate;
  FIR_Init_LPF(&dsp->fir_i, bw_norm, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q, bw_norm, FIR_MAX_TAPS);

  /* Reset all DC blockers on mode switch to avoid residue from previous mode */
  IIR_DCBlock_Init(&dsp->dc_postmix_i);
  IIR_DCBlock_Init(&dsp->dc_postmix_q);
  IIR_DCBlock_Init(&dsp->dc_block_audio);

  /* Reset FM differentiator and de-emphasis to avoid stale IQ leaking into FM */
  dsp->fm.prev_re    = 0.0f;
  dsp->fm.prev_im    = 0.0f;
  dsp->fm.de_emph.y1 = 0.0f;

  /* Reset TX FM modulator: pre-emphasis state and NCO phase (phase itself is
   * arbitrary, but a defined start keeps loopback tests deterministic) */
  dsp->tx.fm_phase_acc = 0U;
  dsp->tx.fm_pre_x1    = 0.0f;

  /* Flush audio FIR buffer to prevent cross-mode transient (up to 64/4kHz ~16ms) */
  memset(dsp->fir_audio.buf, 0, sizeof(dsp->fir_audio.buf));
  dsp->fir_audio.idx = 0U;

  /* Per-mode audio LPF cutoff: CW audio now sits at pitch ± bw/2 (up to
   * 900 + 250 = 1150 Hz), so use 1.5 kHz — selectivity is provided by the
   * pitch-centred IF LPF, this only trims wideband noise.  Fixed (not
   * pitch-coupled) so pitch changes never re-init/flush this FIR mid-RX.
   * All other modes use 4 kHz. */
  {
    float audio_lp_hz = (mode == MODE_CW) ? 1500.0f : 4000.0f;
    FIR_Init_LPF(&dsp->fir_audio, audio_lp_hz / (float)sample_rate, 32U);
  }

  /* Reset RX Hilbert FIR and I delay line: 31-sample startup transient is
   * inaudible (~0.65 ms) and prevents stale data leaking across modes. */
  memset(dsp->rx_hilbert.buf, 0, sizeof(dsp->rx_hilbert.buf));
  dsp->rx_hilbert.idx = 0U;
  memset(dsp->rx_i_delay, 0, sizeof(dsp->rx_i_delay));
  dsp->rx_delay_idx   = 0U;

  /* Reset AGC runtime: inherited level/gain from previous mode can cause
   * an initial burst (if previous mode was loud) or silence (if quiet).
   * env_smooth is also cleared so the smoothed envelope starts from zero.
   * bypass and timing constants are set by AGC_SetMode() after this call. */
  dsp->agc.level      = 0.0f;
  dsp->agc.env_smooth = 0.0f;
  dsp->agc.gain       = 1.0f;
  dsp->agc.hang_timer = 0U;
  dsp->agc.prev_level = 0.0f;
  dsp->agc.drate      = 0.0f;

  /* Reset CW keying envelope so the decoder starts fresh after a mode change.
   * floor_sq restarts HIGH and settles down (20 ms fall) so keyed stays 0
   * while it converges — silent entry instead of a garbage burst. */
  dsp->cw_env.env_sq   = 0.0f;
  dsp->cw_env.floor_sq = 1.0f;
  dsp->cw_env.keyed    = 0U;
  dsp->cw_env.dbn_ctr  = 0U;
  dsp->cw_env.edge_rd  = dsp->cw_env.edge_wr;  /* drop queued edges */
  /* USER CODE END DSP_SetMode_0 */
}

void DSP_SetBW(DSP_State_t *dsp, float bw_hz)
{
  if (bw_hz < 100.0f)   bw_hz = 100.0f;
  if (bw_hz > 24000.0f) bw_hz = 24000.0f;
  uint32_t sr = dsp->sample_rate ? dsp->sample_rate : 48000U;
  dsp->bw_hz = bw_hz;
  /* CW: bw_hz is the full passband width centred on the tuned signal (the
   * −pitch pre-shift in eff_if puts it at 0 Hz here), so the one-sided LPF
   * cutoff is half of it.  Other modes: sideband selection happens in the
   * phasing demod, so the cutoff equals the full audio bandwidth. */
  float cutoff  = (dsp->mode == MODE_CW) ? (bw_hz * 0.5f) : bw_hz;
  float bw_norm = cutoff / (float)sr;
  FIR_Init_LPF(&dsp->fir_i, bw_norm, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q, bw_norm, FIR_MAX_TAPS);
}

void DSP_SetNotch(DSP_State_t *dsp, bool on, float hz)
{
  if (hz < 100.0f)  hz = 100.0f;
  if (hz > 4000.0f) hz = 4000.0f;
  dsp->notch_on = on;
  dsp->notch_hz = hz;
  uint32_t sr = dsp->sample_rate ? dsp->sample_rate : 48000U;
  Notch_Init(&dsp->notch, hz, sr);
}

void DSP_SetTxPassband(DSP_State_t *dsp, float hp_hz, float lp_hz)
{
  if (hp_hz < 100.0f) hp_hz = 100.0f;
  if (hp_hz > 500.0f) hp_hz = 500.0f;
  if (lp_hz < 2000.0f) lp_hz = 2000.0f;
  if (lp_hz > 4000.0f) lp_hz = 4000.0f;
  dsp->tx.tx_hp_hz = hp_hz;
  dsp->tx.tx_lp_hz = lp_hz;
  uint32_t sr = dsp->sample_rate ? dsp->sample_rate : 48000U;
  IIR_HP1_Init(&dsp->tx.hp_audio, hp_hz, sr);
  FIR_Init_LPF(&dsp->tx.fir_audio, lp_hz / (float)sr, 32U);
}

/* ── CW pitch / reverse / sidetone ─────────────────────────────────────────
 * Pitch drives (a) the TX carrier/sidetone NCO and (b) the RX post-FIR
 * re-centre shift (nco_cw_shift, step 5d in DSP_Process).  The matching
 * −pitch pre-shift lives in eff_if (csdr_apply_nco_if), so both must be
 * updated together when pitch or reverse changes. */

static void cw_program_shift_nco(DSP_State_t *dsp, uint32_t sample_rate)
{
  int32_t f = dsp->cw_reverse ? -(int32_t)dsp->cw_pitch_hz
                              :  (int32_t)dsp->cw_pitch_hz;
  NCO_SetFrequency(&dsp->nco_cw_shift, f, sample_rate);
}

void DSP_SetCWPitch(DSP_State_t *dsp, uint16_t pitch_hz, uint32_t sample_rate)
{
  if (pitch_hz < 300U) pitch_hz = 300U;
  if (pitch_hz > 900U) pitch_hz = 900U;
  uint32_t inc = (uint32_t)((int64_t)pitch_hz * (int64_t)4294967296LL / (int64_t)sample_rate);
  dsp->tx.cw_bfo_inc = inc;
  dsp->cw_pitch_hz   = pitch_hz;
  cw_program_shift_nco(dsp, sample_rate);
}

void DSP_SetCWReverse(DSP_State_t *dsp, bool reverse)
{
  dsp->cw_reverse = reverse;
  cw_program_shift_nco(dsp, dsp->sample_rate ? dsp->sample_rate : 48000U);
}

void DSP_SetSidetoneVol(DSP_State_t *dsp, uint8_t vol_pct)
{
  if (vol_pct > 100U) vol_pct = 100U;
  dsp->tx.cw_sidetone_amp = (float)vol_pct * (0.7f / 100.0f);
}

/* ── Static DC offset ───────────────────────────────────────────────────────
 * Call after loading cal from flash.  dc_i / dc_q are in ADC count units
 * (same units as the int32_t dc_i_offset stored in Flash_Settings_t). */
void DSP_SetDCOffset(DSP_State_t *dsp, int32_t dc_i, int32_t dc_q)
{
  dsp->dc_i_static = (float)dc_i;
  dsp->dc_q_static = (float)dc_q;
}

/* ── Spectrum decimation ────────────────────────────────────────────────────
 * factor: 1=±24kHz (pass-through), 2=±12kHz, 4=±6kHz, 8=±3kHz.
 * Resets filter delay lines and discards the first 2 FFT frames so the
 * display doesn't show transient artefacts while the FIRs settle. */
void DSP_SetSpecDecim(DSP_State_t *dsp, uint8_t factor)
{
  if (factor != 2U && factor != 4U && factor != 8U) factor = 1U;
  dsp->spec_decim = factor;
  for (int s = 0; s < 3; s++) {
    memset(dsp->spec_dec_i[s].buf, 0, sizeof(dsp->spec_dec_i[s].buf));
    memset(dsp->spec_dec_q[s].buf, 0, sizeof(dsp->spec_dec_q[s].buf));
    dsp->spec_dec_i[s].idx  = 0U;
    dsp->spec_dec_q[s].idx  = 0U;
    dsp->spec_decim_cnt[s]  = 0U;
  }
  dsp->fft_fill         = 0U;
  dsp->spec_decim_flush = 2U;
}

/* ── Cal measurement ────────────────────────────────────────────────────────
 * DSP_CalStart: arm accumulator (DSP fills it from ISR context).
 * DSP_CalPoll : returns true + copies result when measurement is complete.
 *              Clears cal_meas.mode so DSP stops accumulating after poll. */
void DSP_CalStart(DSP_State_t *dsp, DSP_CalMode_t mode, uint32_t n_samples)
{
  dsp->cal_meas.mode     = DSP_CAL_IDLE;  /* disarm first */
  dsp->cal_meas.n_target = n_samples;
  dsp->cal_meas.n_count  = 0U;
  dsp->cal_meas.acc_i    = 0.0f;
  dsp->cal_meas.acc_q    = 0.0f;
  dsp->cal_meas.acc_ii   = 0.0f;
  dsp->cal_meas.acc_qq   = 0.0f;
  dsp->cal_meas.acc_iq   = 0.0f;
  dsp->cal_meas.done     = false;
  __DMB();
  dsp->cal_meas.mode     = mode;  /* arm: DSP begins accumulating next call */
}

bool DSP_CalPoll(DSP_State_t *dsp, DSP_CalMeas_t *out)
{
  if (!dsp->cal_meas.done) return false;
  __DMB();
  if (out) *out = dsp->cal_meas;
  dsp->cal_meas.mode = DSP_CAL_IDLE;
  return true;
}

/* Load IQ calibration values into DSP state.
 * gain_millis: Q amplitude error × 1000  (-50 → Q is 5% weak, +50 → Q is 5% strong)
 * phase_mrad : phase error in milliradians (-50..+50, ≈ ±2.9°)
 * Transparent when both are zero (identity correction). */
void DSP_SetIQCorr(DSP_State_t *dsp, int16_t gain_millis, int16_t phase_mrad)
{
  /* USER CODE BEGIN DSP_SetIQCorr_0 */
  float g      = (float)gain_millis * 0.001f;    /* fractional gain error  */
  float p      = (float)phase_mrad  * 0.001f;    /* phase error, radians   */
  dsp->iq_g_inv = 1.0f / (1.0f + g);             /* Q amplitude correction */
  dsp->iq_p     = p;                              /* Q phase correction     */
  /* USER CODE END DSP_SetIQCorr_0 */
}

/* ============================================================
 *  DSP_Process – RX demodulation pipeline
 *
 *  iq_in[]  : int32 interleaved [I0,Q0, I1,Q1, ...]
 *             SAI DMA format: 16-bit sample right-justified in bits[15:0] (RX).
 *             Read: (int16_t)(uint16_t)word
 *
 *  audio_out[]: int32 stereo [L0,R0, L1,R1, ...]
 *             SAI TX non-pack: 16-bit data right-justified in bits[15:0].
 *             Write: (int32_t)(int16_t)sample
 * ============================================================ */
void DSP_Process(DSP_State_t *dsp,
                  const int32_t *iq_in,
                  int32_t       *audio_out,
                  uint32_t       len)
{
  /* USER CODE BEGIN DSP_Process_0 */
  float power_acc = 0.0f;

  for (uint32_t n = 0U; n < len; n++)
  {
    /* ── 1. Read: SAI RX stores 16-bit data right-justified in bits[15:0].
     *           (STM32H7 SAI: DataSize < SlotSize → RX right-justified, TX left-justified)
     *           Standard IQ convention: Q = +sin(2πft) for a signal at +f (USB side). */
    int16_t adc_i = (int16_t)(uint16_t)iq_in[n * 2U + 0U];
    int16_t adc_q = (int16_t)(uint16_t)iq_in[n * 2U + 1U];

    /* ── 1b. Cal: DC measurement — raw ADC counts before any processing.
     *         Accumulates the static ADC bias so auto_dc_cal can measure it. */
    if (dsp->cal_meas.mode == DSP_CAL_DC && !dsp->cal_meas.done) {
      dsp->cal_meas.acc_i += (float)adc_i;
      dsp->cal_meas.acc_q += (float)adc_q;
      if (++dsp->cal_meas.n_count >= dsp->cal_meas.n_target) {
        float cnt = (float)dsp->cal_meas.n_count;
        dsp->cal_meas.result_dc_i = dsp->cal_meas.acc_i / cnt;
        dsp->cal_meas.result_dc_q = dsp->cal_meas.acc_q / cnt;
        dsp->cal_meas.done = true;
      }
    }

    /* ── 1c. Static DC offset subtraction + normalize to ±1.0 */
    float raw_i = ((float)adc_i - dsp->dc_i_static) * DSP_INV_32767;
    float raw_q = ((float)adc_q - dsp->dc_q_static) * DSP_INV_32767;

    /* ── 3. NCO down-mix to baseband
     *       [mix_i]   [ cos  sin] [raw_i]
     *       [mix_q] = [-sin  cos] [raw_q]  (nco.sin_val is stored negated) */
    NCO_Step(&dsp->nco);
    float mix_i = raw_i * dsp->nco.cos_val - raw_q * dsp->nco.sin_val;
    float mix_q = raw_i * dsp->nco.sin_val + raw_q * dsp->nco.cos_val;

    /* ── 3b. Post-mix DC removal – kills residual LO leakage / QSD imbalance
     *        that survives the pre-mix blocker and would appear as center spike */
    /* ── 3b+. Cal: IQ mismatch measurement — post-DC-block, pre-correction.
     *          Accumulates I²/Q²/I·Q to compute gain and phase imbalance.
     *          A real signal must be present for a meaningful result. */
    if (dsp->cal_meas.mode == DSP_CAL_IQ && !dsp->cal_meas.done) {
      dsp->cal_meas.acc_ii += mix_i * mix_i;
      dsp->cal_meas.acc_qq += mix_q * mix_q;
      dsp->cal_meas.acc_iq += mix_i * mix_q;
      if (++dsp->cal_meas.n_count >= dsp->cal_meas.n_target) {
        float cnt   = (float)dsp->cal_meas.n_count;
        float rms_i = sqrtf(dsp->cal_meas.acc_ii / cnt);
        float rms_q = sqrtf(dsp->cal_meas.acc_qq / cnt);
        float cross  = dsp->cal_meas.acc_iq / cnt;
        float denom  = rms_i * rms_q + 1e-10f;
        /* Gain error: (Q_rms / I_rms - 1) × 1000 */
        dsp->cal_meas.result_iq_gain  = (rms_q / (rms_i + 1e-10f) - 1.0f) * 1000.0f;
        /* Phase error: asin(E[I·Q] / (I_rms × Q_rms)) × 1000 (milliradians) */
        float pa = cross / denom;
        if (pa >  0.9999f) pa =  0.9999f;
        if (pa < -0.9999f) pa = -0.9999f;
        dsp->cal_meas.result_iq_phase = asinf(pa) * 1000.0f;
        dsp->cal_meas.done = true;
      }
    }

    /* ── 3c. IQ mismatch correction (Gram-Schmidt orthogonalization).
     *        Corrects QSD amplitude and phase imbalance to improve image rejection.
     *        Identity when iq_g_inv=1.0 and iq_p=0.0 (uncalibrated default).
     *        Cost: 2 multiplies + 1 subtract per sample. */
    mix_q = (mix_q - dsp->iq_p * mix_i) * dsp->iq_g_inv;

    /* ── 3d. HF Noise Blanker — impulse detection + interpolating (soft) blank.
     *
     *  Placement rationale:
     *    • Full ±Fs/2 = ±24 kHz bandwidth here maximises impulse detectability.
     *    • AGC after the blank avoids hang-time pumping from blanked samples.
     *    • FFT receives the (already-repaired) delayed IQ; a 4/512-sample
     *      blank is ~0.8% duty and produces negligible spectral artefact.
     *
     *  Algorithm:
     *    floor_sq : IIR power estimate (α=0.9999 → τ≈208 ms at 48 kHz).
     *               Frozen during active blanking so spikes do not bias the floor.
     *    Trigger  : I²+Q² > floor_sq × threshold_ratio_sq → start blank window.
     *    Blank    : instead of zeroing, the blanked samples are linearly
     *               ramped between the last good sample before the impulse
     *               (anchor_i/q) and the first good sample after it. Since
     *               the "after" endpoint is not known until blank_width
     *               samples later, IQ is run through a small fixed
     *               NB_RING_LEN-sample delay line — the ramp is written
     *               into the ring before those slots are read back out, so
     *               downstream stages never see a raw zero or the endpoint
     *               guess is wrong. NB disabled = zero added latency.
     *
     *  CPU cost: ~0.15% at 48 kHz / 400 MHz (ring copy + occasional ramp fill).
     *  Disabled by default; enable via DSP_NB_Set() + g_sdr.nb_on flag. */
    if (dsp->nb.enabled) {
      if (dsp->nb.prime_ctr > 0U) {
        /* Delay line not yet full of real samples since enable — bypass
         * (no delay) while filling it so stale/garbage data is never output. */
        dsp->nb.ring_i[dsp->nb.widx] = mix_i;
        dsp->nb.ring_q[dsp->nb.widx] = mix_q;
        dsp->nb.widx = (uint8_t)((dsp->nb.widx + 1U) & (NB_RING_LEN - 1U));
        dsp->nb.prime_ctr--;
      } else {
        float nb_mag_sq = mix_i * mix_i + mix_q * mix_q;

        /* Update background floor only outside blanking window */
        if (dsp->nb.blank_ctr == 0U && !dsp->nb.awaiting_anchor) {
          dsp->nb.floor_sq = 0.9999f * dsp->nb.floor_sq
                           + 0.0001f * nb_mag_sq;
        }

        float nb_thr = dsp->nb.floor_sq * dsp->nb.threshold_ratio_sq;
        dsp->nb.current_threshold_sq = nb_thr;

        if (dsp->nb.awaiting_anchor) {
          /* This sample is the first good one after the impulse — the
           * "after" endpoint is now known, so retroactively fill the
           * blanked ring slots with a linear ramp anchor -> this sample. */
          uint8_t width = dsp->nb.blank_width;
          for (uint8_t k = 0U; k < width; k++) {
            float t   = (float)(k + 1U) / (float)(width + 1U);
            uint8_t pos = (uint8_t)((dsp->nb.blank_start_widx + k) & (NB_RING_LEN - 1U));
            dsp->nb.ring_i[pos] = dsp->nb.anchor_i + t * (mix_i - dsp->nb.anchor_i);
            dsp->nb.ring_q[pos] = dsp->nb.anchor_q + t * (mix_q - dsp->nb.anchor_q);
          }
          dsp->nb.awaiting_anchor = false;
        } else if (dsp->nb.blank_ctr == 0U && nb_mag_sq > nb_thr) {
          /* Trigger: start a new blank window on the rising edge of an impulse */
          dsp->nb.blank_ctr        = dsp->nb.blank_width;
          dsp->nb.blank_start_widx = dsp->nb.widx;
          uint8_t prev = (uint8_t)((dsp->nb.widx + NB_RING_LEN - 1U) & (NB_RING_LEN - 1U));
          dsp->nb.anchor_i = dsp->nb.ring_i[prev];
          dsp->nb.anchor_q = dsp->nb.ring_q[prev];
          dsp->nb.trig_count++;
          if (nb_mag_sq > dsp->nb.peak_mag_sq) {
            dsp->nb.peak_mag_sq = nb_mag_sq;
          }
        }

        /* Buffer this tick's raw sample (bad samples get ramp-repaired by
         * the awaiting_anchor branch above before their slot is ever read). */
        dsp->nb.ring_i[dsp->nb.widx] = mix_i;
        dsp->nb.ring_q[dsp->nb.widx] = mix_q;

        if (dsp->nb.blank_ctr > 0U) {
          dsp->nb.blank_ctr--;
          if (dsp->nb.blank_ctr == 0U) { dsp->nb.awaiting_anchor = true; }
        }

        dsp->nb.widx = (uint8_t)((dsp->nb.widx + 1U) & (NB_RING_LEN - 1U));

        /* Output = oldest ring slot → fixed NB_RING_LEN-sample delay */
        mix_i = dsp->nb.ring_i[dsp->nb.widx];
        mix_q = dsp->nb.ring_q[dsp->nb.widx];
      }
    }

    /* ── 4. FFT feed with optional decimation (spec bandscope).
     *       Tap point: after NB, before audio FIR.  Decimation factor set by
     *       DSP_SetSpecDecim() when the UI zoom level changes.
     *       decim=1 → pass-through (full ±24 kHz), =2/4/8 → ±12/6/3 kHz. */
    {
      float di = mix_i, dq = mix_q;
      bool  emit = true;

      if (dsp->spec_decim >= 2U) {
        di = FIR_Process(&dsp->spec_dec_i[0], di);
        dq = FIR_Process(&dsp->spec_dec_q[0], dq);
        dsp->spec_decim_cnt[0]++;
        emit = (dsp->spec_decim_cnt[0] & 1U) == 0U;
      }
      if (emit && dsp->spec_decim >= 4U) {
        di = FIR_Process(&dsp->spec_dec_i[1], di);
        dq = FIR_Process(&dsp->spec_dec_q[1], dq);
        dsp->spec_decim_cnt[1]++;
        emit = (dsp->spec_decim_cnt[1] & 1U) == 0U;
      }
      if (emit && dsp->spec_decim >= 8U) {
        di = FIR_Process(&dsp->spec_dec_i[2], di);
        dq = FIR_Process(&dsp->spec_dec_q[2], dq);
        dsp->spec_decim_cnt[2]++;
        emit = (dsp->spec_decim_cnt[2] & 1U) == 0U;
      }

      if (emit && dsp->fft_fill < DSP_FFT_SIZE) {
        float w = dsp->fft_window[dsp->fft_fill];
        dsp->fft_buf[dsp->fft_fill].re = di * w;
        dsp->fft_buf[dsp->fft_fill].im = dq * w;
        dsp->fft_fill++;
        if (dsp->fft_fill >= DSP_FFT_SIZE) {
          float peak;
          { uint32_t fft_t0 = DWT->CYCCNT;
            FFT_Precomp(dsp->fft_buf, DSP_FFT_SIZE);
            RuntimeDiag_FftReport((uint32_t)(DWT->CYCCNT - fft_t0)); }
          FFT_ComputeMag_dB(dsp->fft_buf, dsp->fft_mag_db, DSP_FFT_SIZE, &peak);
          if (dsp->spec_decim_flush > 0U) {
            dsp->spec_decim_flush--;
          } else {
            dsp->fft_ready = true;
            if (dsp->wf_lines < 255U) dsp->wf_lines++;
          }
          dsp->fft_fill = 0U;
        }
      }
    }

    /* ── 4b. IF shift – independent passband tuning, applied after FFT so the
     *         waterfall/spectrum always centres on the VFO frequency. */
    NCO_Step(&dsp->nco_if);
    float shift_i = mix_i * dsp->nco_if.cos_val - mix_q * dsp->nco_if.sin_val;
    float shift_q = mix_i * dsp->nco_if.sin_val + mix_q * dsp->nco_if.cos_val;

    /* ── 5. FIR LPF I and Q (band-limit for demod) */
    float filt_i = FIR_Process(&dsp->fir_i, shift_i);
    float filt_q = FIR_Process(&dsp->fir_q, shift_q);

    /* ── 5b. S-meter: band-limited IQ power before AGC.
     *        filt_i²+filt_q² is the instantaneous complex baseband power,
     *        band-limited to the selected passband, independent of demod mode.
     *        Measuring post-AGC audio (as previously done) produced a nearly
     *        constant value (~AGC target²) regardless of signal strength. */
    float mag_sq = filt_i * filt_i + filt_q * filt_q;
    power_acc += mag_sq;

    /* ── 5c. CW keying envelope tap – pre-AGC, no BFO ripple, no sqrt.
     *        Asymmetric IIR on mag² + adaptive floor tracker.
     *        Keying uses a hysteresis comparator (ON above 10 dB over the
     *        floor, OFF below 6 dB) plus a 3 ms debounce so envelope ripple
     *        near the threshold cannot spray edges.  The floor rises only
     *        slowly (~2 s) while keyed — it must track the band noise, not
     *        learn the signal and erode the detection margin mid-dah.
     *        Accepted transitions are back-dated by the debounce length and
     *        queued in cw_env's edge ring, so CWDec_Update gets exact
     *        durations regardless of main-loop jitter.
     *        Only updated in CW mode to save cycles in other modes. */
    if (dsp->mode == MODE_CW) {
      float alpha = (mag_sq > dsp->cw_env.env_sq)
                    ? dsp->cw_env.alpha_r : dsp->cw_env.alpha_f;
      dsp->cw_env.env_sq = alpha * dsp->cw_env.env_sq
                           + (1.0f - alpha) * mag_sq;
      float fa = (dsp->cw_env.env_sq < dsp->cw_env.floor_sq)
                 ? dsp->cw_env.alpha_nd
                 : (dsp->cw_env.keyed ? dsp->cw_env.alpha_nk
                                      : dsp->cw_env.alpha_nu);
      dsp->cw_env.floor_sq = fa * dsp->cw_env.floor_sq
                             + (1.0f - fa) * dsp->cw_env.env_sq;
      dsp->cw_env.sample_clock++;
      float thr = dsp->cw_env.keyed ? 4.0f : 10.0f;   /* 6 dB / 10 dB */
      uint8_t raw = (dsp->cw_env.env_sq > dsp->cw_env.floor_sq * thr)
                    ? 1U : 0U;
      if (raw != dsp->cw_env.keyed) {
        if (++dsp->cw_env.dbn_ctr >= dsp->cw_env.dbn_len) {
          uint32_t t = (dsp->cw_env.sample_clock >= dsp->cw_env.dbn_len)
                       ? dsp->cw_env.sample_clock - dsp->cw_env.dbn_len : 0U;
          CWEnv_PushEdge(&dsp->cw_env, raw, t);
          dsp->cw_env.dbn_ctr = 0U;
        }
      } else {
        dsp->cw_env.dbn_ctr = 0U;
      }
    }

    /* ── 5d. CW passband re-centre — eff_if carries an extra −pitch (CW-N,
     *        +pitch for CW-R, see csdr_apply_nco_if) so the IF LPF above is
     *        centred on the correctly-tuned signal.  Shift the filtered
     *        baseband back by ±pitch so the product detector below yields
     *        the sidetone pitch again: the net audio mapping (tone = offset
     *        from dial) is identical to the unshifted chain, only the filter
     *        window moved from [0, bw] to pitch ± bw/2.
     *        Residual LO/DC leakage sits at −pitch here — outside the LPF
     *        passband — and lands on 0 Hz audio after this shift, where the
     *        post-demod DC blocker (7b) removes it: it can never appear as
     *        a tone at the pitch frequency (the old envelope+BFO CW demod
     *        failure mode).  Taps 5b/5c above use |·|², which is invariant
     *        under this rotation, so they stay pre-shift. */
    if (dsp->mode == MODE_CW) {
      NCO_Step(&dsp->nco_cw_shift);
      float cwr_i = filt_i * dsp->nco_cw_shift.cos_val - filt_q * dsp->nco_cw_shift.sin_val;
      float cwr_q = filt_i * dsp->nco_cw_shift.sin_val + filt_q * dsp->nco_cw_shift.cos_val;
      filt_i = cwr_i;
      filt_q = cwr_q;
    }

    /* ── 5e. Hilbert FIR on Q + matched I delay for USB/LSB phasing demod.
     *
     *  Demod_USB/LSB expect H{Q} (Hilbert-transformed Q) as second argument,
     *  not raw Q.  The Hilbert FIR has group delay = HILBERT_DELAY = 31 samples;
     *  I is delayed through a ring buffer of size (HILBERT_DELAY+1) = 32 to
     *  keep I and H{Q} time-aligned.
     *
     *  Only active in USB/LSB/CW modes to save computation.  The Hilbert FIR
     *  and delay buffer are reset in DSP_SetMode so the first 31 startup
     *  samples output zero (inaudible at 48 kHz).
     *
     *  CW reuses the same phasing product detector as USB/LSB (picked via
     *  cw_reverse below) instead of an envelope detector with a reinserted
     *  BFO tone: any residual DC/LO-leakage in the baseband then behaves
     *  exactly as it does in SSB (stays near 0 Hz, removed by the ordinary
     *  post-demod DC blocker) instead of being frequency-shifted up into an
     *  audible, unblockable tone at the pitch frequency. */
    float filt_i_d = filt_i;   /* aligned I: delayed for USB/LSB/DIGU/DIGL/CW, direct otherwise */
    float filt_q_h = filt_q;   /* hq arg:    H{Q} for SSB/CW modes, raw Q for others             */
    if (dsp->mode == MODE_USB    || dsp->mode == MODE_LSB  ||
        dsp->mode == MODE_DIGU   || dsp->mode == MODE_DIGL ||
        dsp->mode == MODE_CW)
    {
      /* Hardware QSD produces Q = -sin for a signal at +f (USB side), i.e. the
       * complex baseband is exp(-jωt).  FFT_Precomp negates Im to fix the
       * mirrored spectrum display, but the demodulator works on raw filt_q.
       * Negate Q here so the Hilbert sees +sin convention that Demod_USB/LSB
       * expect: H{+sin}=-cos for USB, H{-sin}=+cos for LSB. */
      filt_q_h = Hilbert_Process(&dsp->rx_hilbert, -filt_q);
      /* Ring-buffer I delay: write current, read oldest (31 samples ago) */
      dsp->rx_i_delay[dsp->rx_delay_idx] = filt_i;
      uint16_t rd = (uint16_t)((dsp->rx_delay_idx + 1U) % (HILBERT_DELAY + 1U));
      filt_i_d     = dsp->rx_i_delay[rd];
      dsp->rx_delay_idx = rd;
    }

    /* ── 6. Demodulate */
    float audio;
    switch (dsp->mode)
    {
      case MODE_AM:   audio = Demod_AM(filt_i, filt_q);                          break;
      case MODE_FM:   audio = Demod_FM(&dsp->fm, filt_i, filt_q);               break;
      case MODE_USB:
      case MODE_DIGU: audio = Demod_USB(filt_i_d, filt_q_h);                    break;
      case MODE_LSB:
      case MODE_DIGL: audio = Demod_LSB(filt_i_d, filt_q_h);                    break;
      case MODE_CW:   audio = dsp->cw_reverse ? Demod_LSB(filt_i_d, filt_q_h)
                                               : Demod_USB(filt_i_d, filt_q_h); break;
      default:        audio = filt_i;                                             break;
    }

    /* ── 7. Audio LPF */
    audio = FIR_Process(&dsp->fir_audio, audio);

    /* ── 7b. Audio DC block – removes DC bias from AM envelope demodulation.
     *        Without this, AM demod (always-positive envelope) produces a DC
     *        signal that is inaudible through AC-coupled headphone outputs. */
    audio = IIR_DCBlock_Process(&dsp->dc_block_audio, audio);

    /* ── 7b2. FT8 decoder tap — post-demod/LPF/DC-block, pre-notch/NR/AGC so
     *         decode is independent of audio shaping, and pre-squelch (the
     *         squelch gate only zeroes audio_out).  Gate flag is owned by
     *         FT8_Poll; both run in CSDR_Loop context. */
    if (g_ft8_tap_enable)
      FT8_FeedAudio(audio);

    /* ── 7c. Notch filter – narrow-band rejection before AGC */
    if (dsp->notch_on)
      audio = IIR_Biquad_Process(&dsp->notch, audio);

    /* ── 7c2. Beat canceller – LMS auto-notch, removes steady carriers so the
     *         NR stage below sees voice+noise only */
    if (dsp->bc.enabled) {
      float bc_pred;
      audio = lms_run(&dsp->bc, audio, &bc_pred);
    }

    /* ── 7d. Noise Reduction – NR1 LMS line enhancer / NR2 spectral Wiener */
    if (dsp->nr_mode == 1U) {
      float pred;
      (void)lms_run(&dsp->nr, audio, &pred);
      audio = audio + (pred - audio) * dsp->nr.wet;
    } else if (dsp->nr_mode == 2U) {
      audio = NRSpec_Process(audio);
    }

    /* ── 8. AGC */
    audio = AGC_Process(&dsp->agc, audio);

    /* ── 8b. RX tone control – bass/treble shelf, final tone shaping only.
     *        Placed after AGC so it never feeds back into level detection or
     *        squelch (signal_power_db is measured pre-AGC, step 5b above). */
    audio = IIR_Biquad_Process(&dsp->bass_shelf,   audio);
    audio = IIR_Biquad_Process(&dsp->treble_shelf, audio);

    /* ── 9. Write: 16-bit sample right-justified in bits[15:0] of 32-bit DMA word.
     *           STM32H7 SAI non-pack: DataSize=16 in bits[15:0], bits[31:16] ignored by SAI. */
    int32_t out_val = (int32_t)(audio * dsp->rx_volume_scale * 32767.0f);
    if (out_val >  32767)  out_val =  32767;
    if (out_val < -32768)  out_val = -32768;
    int32_t dac_word = (int32_t)(int16_t)out_val;
    audio_out[n * 2U + 0U] = dac_word;  /* L */
    audio_out[n * 2U + 1U] = dac_word;  /* R */
  }

  if (len > 0U) {
    float rms_db = 20.0f * log10f(sqrtf(power_acc / (float)len) + 1e-10f);
    dsp->signal_power_db = 0.9f * dsp->signal_power_db + 0.1f * rms_db;
  }

  /* Squelch gate — 3 dB hysteresis to prevent chatter */
  if (dsp->squelch_threshold_db > -199.0f) {
    if      (dsp->signal_power_db >= dsp->squelch_threshold_db + 3.0f) dsp->squelch_open = true;
    else if (dsp->signal_power_db <  dsp->squelch_threshold_db - 3.0f) dsp->squelch_open = false;
    if (!dsp->squelch_open)
      memset(audio_out, 0, len * 2U * sizeof(int32_t));
  }

  dsp->sample_count += len;
  /* USER CODE END DSP_Process_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/* ============================================================
 *  Hilbert Transform FIR (90° phase shift for SSB TX)
 *
 *  h[n] = 0                         if (n-M) even
 *  h[n] = (2 / (pi*(n-M))) * w[n]   if (n-M) odd
 *  Hamming window, N = HILBERT_TAPS (63), M = 31.
 *
 *  After computing coefficients, normalize passband gain to 1.0 at Fs/8
 *  (≈6kHz @ 48kHz) so that the Hilbert Q-branch and the linear delay
 *  I-branch have equal amplitude → good carrier suppression in SSB.
 * ============================================================ */
void Hilbert_Init(Hilbert_t *h)
{
  const int N = HILBERT_TAPS;
  const int M = (N - 1) / 2;

  for (int n = 0; n < N; n++) {
    int k = n - M;
    float c = 0.0f;
    if ((k & 1) != 0) {
      c = 2.0f / (3.14159265358979f * (float)k);
      float w = 0.54f - 0.46f * cosf(2.0f * 3.14159265358979f
                                      * (float)n / (float)(N - 1));
      c *= w;
    }
    h->coeff[n] = c;
    h->buf[n]   = 0.0f;
  }
  h->idx = 0;

  /* Normalize passband gain to 1.0 at f = Fs/8.
   * Hamming windowing reduces the ideal-Hilbert unity gain to ~0.94;
   * without correction the Q branch is weaker than the I (delay) branch,
   * leaving an uncanncelled carrier residue in the SSB output. */
  float re = 0.0f, im = 0.0f;
  const float omega = 2.0f * 3.14159265358979f / 8.0f; /* 2pi * (Fs/8)/Fs */
  for (int n = 0; n < N; n++) {
    float phase = omega * (float)(n - M);
    re += h->coeff[n] * cosf(phase);
    im += h->coeff[n] * sinf(phase);
  }
  float gain = sqrtf(re * re + im * im);
  if (gain > 0.01f) {
    float inv = 1.0f / gain;
    for (int n = 0; n < N; n++) h->coeff[n] *= inv;
  }
}

float Hilbert_Process(Hilbert_t *h, float x)
{
  const int N = HILBERT_TAPS;
  h->buf[h->idx] = x;
  float acc = 0.0f;
  int   k   = h->idx;
  for (int n = 0; n < N; n++) {
    acc += h->coeff[n] * h->buf[k];
    k    = (k == 0) ? (N - 1) : (k - 1);
  }
  h->idx = (uint16_t)((h->idx + 1U) % N);
  return acc;
}

/* ============================================================
 *  DSP_ProcessTX – TX DSP pipeline
 *
 *  Audio source : USB Audio OUT ring (PC → radio).
 *  FIR LPF is applied to the audio signal BEFORE modulation so the
 *  filter is on a real mono signal (not complex IQ); filtering after
 *  SSB modulation would introduce differential phase/amplitude shift
 *  between I and Q branches, degrading carrier suppression.
 *
 *  Output: int32 MSB-aligned interleaved [I0,Q0, I1,Q1, ...]
 * ============================================================ */
#include "usb_audio.h"
extern USB_Audio_Handle_t g_usb_audio;

void DSP_ProcessTX(DSP_State_t *dsp, int32_t *iq_out, uint32_t len)
{
  /* USER CODE BEGIN DSP_ProcessTX_0 */

  /* Snapshot tx_count once under a brief critical section.
   * USB_Audio_WriteTX (USB IRQ) increments tx_count concurrently.  We read a
   * stable value here, compute how many complete stereo samples (4 bytes each)
   * we can consume, then do all reads without IRQ protection — tx_rd is
   * exclusively written by this function (main-loop context only).
   * After the loop we subtract the consumed byte count in a single CS so the
   * IRQ's concurrent additions are preserved correctly. */
  __disable_irq();
  uint16_t tx_avail = g_usb_audio.tx_count;
  __enable_irq();

  /* Each USB stereo frame = int16 L + int16 R = 4 bytes.  DSP uses L only. */
  uint16_t samples_avail = tx_avail / 4U;
  if (samples_avail > (uint16_t)len) { samples_avail = (uint16_t)len; }
  uint16_t bytes_consumed = (uint16_t)(samples_avail * 4U);

  for (uint32_t n = 0U; n < len; n++)
  {
    /* ── 1. Pull mono audio from selected TX source.
     *   MIC mode: SAI RX (WM8731 ADC mic_buf), 16-bit right-justified stereo,
     *             left channel (index 2n) = mic audio.
     *   USB mode: USB TX ring, int16 LE stereo, L channel only. */
    float audio = 0.0f;
    if (dsp->mic_buf != NULL) {
      /* MIC IN path: s_rx_buf[2n] = L channel, 16-bit right-justified in int32 */
      int16_t s = (int16_t)(dsp->mic_buf[n * 2U] & 0xFFFFU);
      audio = (float)s * DSP_INV_32767;
    } else if (n < (uint32_t)samples_avail) {
      uint8_t lo = g_usb_audio.tx_ring[g_usb_audio.tx_rd];
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 1U) % USB_AUDIO_RING_SIZE);
      uint8_t hi = g_usb_audio.tx_ring[g_usb_audio.tx_rd];
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 1U) % USB_AUDIO_RING_SIZE);
      int16_t s = (int16_t)((uint16_t)hi << 8 | lo);
      audio = (float)s * DSP_INV_32767;
      /* Discard R channel (2 bytes) */
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 2U) % USB_AUDIO_RING_SIZE);
    }

    /* ── 1b. FT8 beacon tone — replaces the mic/USB source while active.
     *        Injected pre-gain so digi drive, tx_power, PA foldback and ALC
     *        all apply exactly as for WSJT-X audio (DIGU linear path). */
    if (g_ft8_tx_tone)
      audio = FT8_TxSample();

    /* ── 2. Audio DC block – remove mic/line DC offset before Hilbert.
     *       DC in audio produces a carrier tone at the TX LO frequency. */
    audio = IIR_DCBlock_Process(&dsp->tx.dc_block, audio);
    audio = IIR_DCBlock_Process(&dsp->tx.hp_audio, audio);  /* TX Low-cut HPF */

    /* ── 3. TX audio gain.
     * FM/AM: modulation depth (deviation / mod. index) must track mic_gain
     * only — drive_gain (tx_power × PA foldback × ALC × trim) is applied
     * separately to carrier amplitude further down (fm_amp / AM carrier).
     * audio_gain = drive_gain × gain_src_pct/100 (see csdr_apply_tx), so
     * dividing it back out here recovers the pure gain_src_pct/100 factor —
     * otherwise lowering tx_power or PA foldback would also shrink FM
     * deviation / AM modulation index instead of just RF carrier power. */
    if (dsp->mode == MODE_FM || dsp->mode == MODE_AM) {
      audio *= (dsp->tx.drive_gain > 0.0001f)
                 ? (dsp->tx.audio_gain / dsp->tx.drive_gain) : 0.0f;
    } else {
      audio *= dsp->tx.audio_gain;
    }

    /* ── 4. Audio FIR LPF BEFORE modulation.
     *       Filtering the real audio signal band-limits and ensures
     *       equal spectral amplitude entering both I (delay) and Q (Hilbert)
     *       branches. Do NOT add another filter after the modulator. */
    float audio_lp = FIR_Process(&dsp->tx.fir_audio, audio);

    /* ── 4b. TX compressor + soft limiter.
     *
     *   Voice modes (USB/LSB/AM/FM/CW):
     *     Stage 1 – Peak-tracking compressor (4:1 above 0.70 threshold).
     *               Attack 1 ms / release 50 ms.  Transparent below threshold,
     *               gently reduces gain when speech peaks exceed it.
     *     Stage 2 – C1-smooth soft limiter for residual peaks.
     *               Linear to 0.95; above that asymptotically approaches 1.0.
     *               Formula:  y = 1 − 0.0025 / (|x| − 0.90)   for |x| > 0.95
     *
     *   Digital modes (DIGU/DIGL):
     *     Both stages bypassed.  Pure linear pass — no gain riding, no
     *     harmonic generation.  audio_gain (from digi_gain) is the only
     *     amplitude scaling.  Preserves tone purity for WSJT-X/FT8/Tune. */
    if (dsp->mode != MODE_DIGU && dsp->mode != MODE_DIGL) {
      float env = fabsf(audio_lp);
      if (env > dsp->tx.comp_env)
        dsp->tx.comp_env = dsp->tx.comp_attack * dsp->tx.comp_env
                          + (1.0f - dsp->tx.comp_attack) * env;
      else
        dsp->tx.comp_env = dsp->tx.comp_decay  * dsp->tx.comp_env
                          + (1.0f - dsp->tx.comp_decay)  * env;

      /* 4:1 compression above 0.70 */
      if (dsp->tx.comp_env > 0.70f) {
        float excess     = dsp->tx.comp_env - 0.70f;
        float compressed = 0.70f + excess * 0.25f;
        audio_lp *= compressed / dsp->tx.comp_env;
      }

      /* Soft limiter for residual peaks: C1-continuous at knee = 0.95 */
      float a = fabsf(audio_lp);
      if (a > 0.95f) {
        float s  = (audio_lp >= 0.0f) ? 1.0f : -1.0f;
        audio_lp = s * (1.0f - 0.0025f / (a - 0.90f));
      }
    }

    /* ── 5. Modulate */
    float tx_i = 0.0f, tx_q = 0.0f;
    /* TUNE: force the CW carrier branch regardless of the displayed operating
     * mode — reuses the same envelope/oscillator so PA_Protect's drive limit
     * (audio_gain) still applies (see csdr_app.c / pa_protect.c). */
    SDR_Mode_t mod_sel = dsp->tx.tune_active ? MODE_CW : dsp->mode;
    switch (mod_sel) {
      case MODE_USB:
      case MODE_LSB:
      case MODE_DIGU:
      case MODE_DIGL: {
        /* Phasing method: delay I by Hilbert group delay = (N-1)/2 samples.
         * DIGU uses USB polarity (Q=+H), DIGL uses LSB polarity (Q=-H). */
        const uint16_t M = (HILBERT_TAPS - 1U) / 2U;
        dsp->tx.audio_delay[dsp->tx.delay_idx] = audio_lp;
        uint16_t ridx = (uint16_t)((dsp->tx.delay_idx + HILBERT_TAPS - M) % HILBERT_TAPS);
        float audio_d = dsp->tx.audio_delay[ridx];
        dsp->tx.delay_idx = (uint16_t)((dsp->tx.delay_idx + 1U) % HILBERT_TAPS);
        float audio_h = Hilbert_Process(&dsp->tx.hilbert, audio_lp);
        tx_i = audio_d;
        tx_q = (dsp->mode == MODE_USB || dsp->mode == MODE_DIGU) ? +audio_h : -audio_h;
        break;
      }
      case MODE_AM:
        /* Carrier ← drive_gain (tx_power × PA foldback × ALC × trim), same
         * role as FM's fm_amp below, so PA protection/TRIP and tx_power act
         * on real RF power; audio_lp (mic_gain only, stage 3) sets modulation
         * index independent of drive. */
        tx_i = dsp->tx.drive_gain * (0.5f + 0.5f * audio_lp);
        tx_q = 0.0f;
        break;
      case MODE_CW: {
        /* Keyed CW carrier at LO+pitch Hz (USB sideband) with click-free envelope.
         * cw_key_out is written by the main-loop keyer; volatile for ISR safety.
         * Scaled by audio_gain so PA_Protect foldback/trip actually reduces the
         * carrier (previously the CW path bypassed audio_gain entirely). */
        float env_tgt = dsp->cw_key_out ? (dsp->tx.cw_sidetone_amp * dsp->tx.audio_gain) : 0.0f;
        /* Attack ~3 ms, release ~6 ms at 48 kHz (1 sample ≈ 0.021 ms) */
        float coeff = (env_tgt > dsp->tx.cw_env_amp) ? 0.9994f : 0.9988f;
        dsp->tx.cw_env_amp = dsp->tx.cw_env_amp * coeff + env_tgt * (1.0f - coeff);
        dsp->tx.cw_phase_acc += dsp->tx.cw_bfo_inc;
        uint32_t cw_idx = dsp->tx.cw_phase_acc >> (32U - NCO_LUT_BITS);
        float cw_cos = s_nco_sin_lut[(cw_idx + (NCO_LUT_SIZE / 4U)) & NCO_LUT_MASK];
        float cw_sin = s_nco_sin_lut[cw_idx & NCO_LUT_MASK];
        /* CW-R: conjugate → LSB-side carrier at LO−pitch, so a station tuned
         * to the sidetone pitch on the reverse sideband is answered on its
         * own frequency (mirrors the RX ±pitch convention above).  Sidetone
         * monitor taps the I channel only, so it is unaffected. */
        if (dsp->cw_reverse) cw_sin = -cw_sin;
        tx_i = cw_cos * dsp->tx.cw_env_amp;
        tx_q = cw_sin * dsp->tx.cw_env_amp;
        break;
      }
      case MODE_FM: {
        /* Constant-envelope FM: NCO phase accumulator, same LUT as the CW branch.
         *
         * Pre-emphasis (inverse of the RX 75 µs de-emphasis, coeffs from
         * DSP_Init) is applied to audio_lp, then hard-clamped to ±1.0 so the
         * high-frequency boost cannot push deviation past FM_TX_DEV_HZ.
         *
         * Gain split — audio_gain would be wrong for both roles here:
         *   deviation ← audio_lp (mic_gain baked in at stage 3: mic gain
         *               controls deviation, as on any FM rig);
         *   carrier   ← drive_gain (tx_power × PA foldback × ALC × trim,
         *               WITHOUT mic gain) so PA protection and tx_power act on
         *               real RF power — audio_gain alone would let mic gain
         *               couple into carrier amplitude. */
        float x = (audio_lp - dsp->tx.fm_pre_k * dsp->tx.fm_pre_x1)
                  * dsp->tx.fm_pre_gain;
        dsp->tx.fm_pre_x1 = audio_lp;
        if (x >  1.0f) x =  1.0f;
        if (x < -1.0f) x = -1.0f;
        dsp->tx.fm_phase_acc += (uint32_t)(int32_t)(x * dsp->tx.fm_dev_scale);
        uint32_t fm_idx = dsp->tx.fm_phase_acc >> (32U - NCO_LUT_BITS);
        float fm_amp = FM_TX_AMP * dsp->tx.drive_gain;
        /* Q=+sin convention (positive audio → +f), consistent with the SSB
         * branch above; FFT feed negates Q for correct display. */
        tx_i = s_nco_sin_lut[(fm_idx + (NCO_LUT_SIZE / 4U)) & NCO_LUT_MASK] * fm_amp;
        tx_q = s_nco_sin_lut[fm_idx & NCO_LUT_MASK] * fm_amp;
        break;
      }
      default:
        tx_i = audio_lp * 0.7f;
        tx_q = 0.0f;
        break;
    }

    /* ── 6. FFT feed for TX spectrum display.
     *       TX generates Q = +sin (correct convention).  FFT_Precomp negates Im
     *       to correct for RX hardware Q = -sin; negate here so the double-
     *       negation restores correct sign and the display is not mirrored. */
    if (dsp->fft_fill < DSP_FFT_SIZE)
    {
      float w = dsp->fft_window[dsp->fft_fill];
      dsp->fft_buf[dsp->fft_fill].re =  tx_i * w;
      dsp->fft_buf[dsp->fft_fill].im = -tx_q * w;
      dsp->fft_fill++;
      if (dsp->fft_fill >= DSP_FFT_SIZE)
      {
        float peak;
        { uint32_t fft_t0 = DWT->CYCCNT;
          FFT_Precomp(dsp->fft_buf, DSP_FFT_SIZE);
          RuntimeDiag_FftReport((uint32_t)(DWT->CYCCNT - fft_t0)); }
        FFT_ComputeMag_dB(dsp->fft_buf, dsp->fft_mag_db, DSP_FFT_SIZE, &peak);
        dsp->fft_ready = true;
        dsp->fft_fill  = 0U;
      }
    }

    /* ── 7. Write to SAI TX DMA: 16-bit sample right-justified in bits[15:0] */
    int32_t i_val = (int32_t)(tx_i * 32767.0f);
    int32_t q_val = (int32_t)(tx_q * 32767.0f);
    if (i_val >  32767)  i_val =  32767;
    if (i_val < -32768)  i_val = -32768;
    if (q_val >  32767)  q_val =  32767;
    if (q_val < -32768)  q_val = -32768;
    iq_out[n * 2U + 0U] = (int32_t)(int16_t)i_val;
    iq_out[n * 2U + 1U] = (int32_t)(int16_t)q_val;
  }

  /* Single atomic decrement: subtract only what was consumed.  Any bytes
   * added by USB_Audio_WriteTX (USB IRQ) after the snapshot are preserved
   * because the IRQ only adds (never subtracts) and we subtract only the
   * snapshotted amount.  Clamp defensively: bytes_consumed <= tx_avail <=
   * tx_count_at_subtract by design, but underflow would be catastrophic
   * (uint16_t wrap makes ring appear full). */
  if (bytes_consumed > 0U) {
    __disable_irq();
    if (g_usb_audio.tx_count >= bytes_consumed)
      g_usb_audio.tx_count = (uint16_t)(g_usb_audio.tx_count - bytes_consumed);
    else
      g_usb_audio.tx_count = 0U;   /* should not happen; guard against wrap */
    __enable_irq();
  }
  /* USER CODE END DSP_ProcessTX_0 */
}
