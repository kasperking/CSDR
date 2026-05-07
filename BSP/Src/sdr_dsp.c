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
  *   - AGC   : Peak-hold + hang AGC, 1ms attack; slow=500ms hang/1.5s decay, fast=100ms hang/300ms decay
  *   - FFT   : Radix-2 DIT, N=256, Hann window
  *   - DEMOD : AM, FM (atan2 differentiator), USB, LSB, CW (BFO 700Hz)
  *
  *  RX pipeline per sample:
  *   pre-DC → NCO mix → post-DC → FFT feed → FIR LPF → Demod → audio LPF → AGC → out
  *
  *  TX pipeline per sample:
  *   USB pull → audio DC → gain → audio FIR LPF → SSB mod → FFT feed → DAC out
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
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define DSP_TWO_PI      6.28318530717958647692f
#define DSP_INV_32767   (1.0f / 32767.0f)
#define DSP_INV_4G      (1.0f / 4294967296.0f)

/* NCO LUT – 1024-entry full-cycle sin table, 4 KB RAM, built once in DSP_Init */
#define NCO_LUT_BITS    10U
#define NCO_LUT_SIZE    (1U << NCO_LUT_BITS)
#define NCO_LUT_MASK    (NCO_LUT_SIZE - 1U)
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static float   s_nco_sin_lut[NCO_LUT_SIZE];
static uint8_t s_nco_lut_init = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static void bit_reverse(Complex_f *buf, uint16_t n);
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
/* USER CODE END 0 */

/* ============================================================
 *  NCO  (LUT-based, no per-sample cosf/sinf)
 * ============================================================ */
void NCO_SetFrequency(NCO_t *nco, int32_t freq_hz, uint32_t sample_rate)
{
  /* USER CODE BEGIN NCO_SetFrequency_0 */
  /* Use signed 64-bit intermediate: casting a negative double directly to
   * uint32_t is undefined behaviour.  int64_t conversion is defined, and the
   * subsequent truncation to uint32_t gives the correct two's-complement
   * phase increment for negative frequencies. */
  int64_t inc64 = (int64_t)((double)freq_hz / (double)sample_rate * 4294967296.0);
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

/* ============================================================
 *  AGC
 * ============================================================ */
void AGC_Init(AGC_t *agc, uint32_t sample_rate)
{
  /* USER CODE BEGIN AGC_Init_0 */
  agc->gain       = 1.0f;
  agc->target     = 0.5f;
  agc->max_gain   = 1000.0f;
  agc->min_gain   = 0.001f;
  agc->level      = 0.0f;
  agc->hang_timer = 0U;
  /* Default to slow constants; caller overrides via AGC_SetSpeed() */
  agc->attack    = expf(-1.0f / (0.001f * (float)sample_rate));  /* 1 ms   */
  agc->decay     = expf(-1.0f / (1.500f * (float)sample_rate));  /* 1.5 s  */
  agc->hang_time = (uint32_t)(0.500f * (float)sample_rate);      /* 500 ms */
  /* USER CODE END AGC_Init_0 */
}

/* Set AGC time constants.  Call once after AGC_Init and whenever agc_fast changes. */
void AGC_SetSpeed(AGC_t *agc, bool fast, uint32_t sample_rate)
{
  /* USER CODE BEGIN AGC_SetSpeed_0 */
  agc->attack = expf(-1.0f / (0.001f * (float)sample_rate));   /* 1 ms always */
  if (fast) {
    agc->decay     = expf(-1.0f / (0.300f * (float)sample_rate)); /* 300 ms */
    agc->hang_time = (uint32_t)(0.100f * (float)sample_rate);     /* 100 ms */
  } else {
    agc->decay     = expf(-1.0f / (1.500f * (float)sample_rate)); /* 1.5 s  */
    agc->hang_time = (uint32_t)(0.500f * (float)sample_rate);     /* 500 ms */
  }
  /* USER CODE END AGC_SetSpeed_0 */
}

float AGC_Process(AGC_t *agc, float x)
{
  /* USER CODE BEGIN AGC_Process_0 */
  float env = fabsf(x);
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
   * Compression to log-like display range is done by pwr_compress() in st7789.c
   * at render time (25 fps × 240 px) rather than here (188 fps × 256 bins). */
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
  float d = atan2f(im, re) * (1.0f / 3.14159265f);
  float y = fm->de_emph.b0 * d - fm->de_emph.a1 * fm->de_emph.y1;
  fm->de_emph.y1 = y;
  return y;
  /* USER CODE END Demod_FM_0 */
}

float Demod_USB(float i, float q) { return (i + q) * 0.5f; }
float Demod_LSB(float i, float q) { return (i - q) * 0.5f; }

float Demod_CW(float i, float q, uint32_t *phase_acc, uint32_t phase_inc)
{
  /* USER CODE BEGIN Demod_CW_0 */
  float amp = Demod_AM(i, q);
  *phase_acc += phase_inc;
  uint32_t idx = *phase_acc >> (32U - NCO_LUT_BITS);
  float bfo = s_nco_sin_lut[(idx + (NCO_LUT_SIZE / 4U)) & NCO_LUT_MASK];
  return amp * bfo;
  /* USER CODE END Demod_CW_0 */
}

/* ============================================================
 *  DSP Engine – init / config
 * ============================================================ */
void DSP_Init(DSP_State_t *dsp, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_Init_0 */
  memset(dsp, 0, sizeof(DSP_State_t));
  dsp->mode = MODE_AM;

  /* Build NCO LUT once (shared across all DSP instances via static) */
  if (!s_nco_lut_init) {
    for (uint32_t i = 0U; i < NCO_LUT_SIZE; i++)
      s_nco_sin_lut[i] = sinf(DSP_TWO_PI * (float)i / (float)NCO_LUT_SIZE);
    s_nco_lut_init = 1U;
  }

  NCO_SetFrequency(&dsp->nco, 0, sample_rate);

  float bw = 6000.0f / (float)sample_rate;
  FIR_Init_LPF(&dsp->fir_i,     bw, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q,     bw, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_audio, 4000.0f / (float)sample_rate, 32U);

  IIR_DCBlock_Init(&dsp->dc_block_i);
  IIR_DCBlock_Init(&dsp->dc_block_q);
  IIR_DCBlock_Init(&dsp->dc_block_audio);
  IIR_DCBlock_Init(&dsp->dc_postmix_i);
  IIR_DCBlock_Init(&dsp->dc_postmix_q);

  /* IQ correction: identity until DSP_SetIQCorr() is called */
  dsp->iq_g_inv = 1.0f;
  dsp->iq_p     = 0.0f;

  AGC_Init(&dsp->agc, sample_rate);
  FFT_Hann_Window(dsp->fft_window, DSP_FFT_SIZE);

  {
    float de_fc          = 2122.0f / (float)sample_rate;
    dsp->fm.de_emph.b0   = 1.0f - expf(-DSP_TWO_PI * de_fc);
    dsp->fm.de_emph.a1   = -(1.0f - dsp->fm.de_emph.b0);
    dsp->fm.de_emph.y1   = 0.0f;
    dsp->fm.prev_re      = 0.0f;
    dsp->fm.prev_im      = 0.0f;
  }

  Hilbert_Init(&dsp->tx.hilbert);
  memset(dsp->tx.audio_delay, 0, sizeof(dsp->tx.audio_delay));
  dsp->tx.delay_idx    = 0;
  dsp->tx.fm_phase     = 0.0f;
  dsp->tx.cw_phase_acc = 0;
  dsp->tx.audio_gain   = 1.0f;
  FIR_Init_LPF(&dsp->tx.fir_audio, 4000.0f / (float)sample_rate, 32U);
  IIR_DCBlock_Init(&dsp->tx.dc_block);
  dsp->tx.comp_env    = 0.0f;
  dsp->tx.comp_attack = expf(-1.0f / (0.001f * (float)sample_rate)); /* 1 ms  */
  dsp->tx.comp_decay  = expf(-1.0f / (0.050f * (float)sample_rate)); /* 50 ms */

  /* CW BFO phase increment: 700 Hz, sample-rate-derived */
  dsp->cw_bfo_inc  = (uint32_t)(int64_t)(700.0 / (double)sample_rate * 4294967296.0);
  dsp->cw_phase_acc = 0U;

  dsp->signal_power_db = -120.0f;
  /* USER CODE END DSP_Init_0 */
}

void DSP_SetFrequency(DSP_State_t *dsp, uint32_t freq_hz,
                       uint32_t if_hz, uint32_t lo_offset_hz,
                       uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_SetFrequency_0 */
  int32_t delta = (int32_t)freq_hz - ((int32_t)if_hz + (int32_t)lo_offset_hz);
  NCO_SetFrequency(&dsp->nco, delta, sample_rate);
  /* USER CODE END DSP_SetFrequency_0 */
}

void DSP_SetMode(DSP_State_t *dsp, SDR_Mode_t mode, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_SetMode_0 */
  dsp->mode        = mode;
  dsp->sample_rate = sample_rate;

  float mode_bw_hz;
  switch (mode)
  {
    case MODE_AM:  mode_bw_hz = 6000.0f;  break;
    case MODE_FM:  mode_bw_hz = 15000.0f; break;
    case MODE_USB:
    case MODE_LSB: mode_bw_hz = 3000.0f;  break;
    case MODE_CW:  mode_bw_hz = 500.0f;   break;
    default:       mode_bw_hz = 4000.0f;  break;
  }
  /* Never overwrite a caller-set bw_hz — only seed it on first init (bw_hz==0).
   * The caller is responsible for calling DSP_SetBW() with the desired BW after
   * this function; that is the sole place that updates dsp->bw_hz. */
  if (dsp->bw_hz <= 0.0f) dsp->bw_hz = mode_bw_hz;

  float bw_norm = dsp->bw_hz / (float)sample_rate;
  FIR_Init_LPF(&dsp->fir_i, bw_norm, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q, bw_norm, FIR_MAX_TAPS);

  /* Reset all DC blockers on mode switch to avoid residue from previous mode */
  IIR_DCBlock_Init(&dsp->dc_block_i);
  IIR_DCBlock_Init(&dsp->dc_block_q);
  IIR_DCBlock_Init(&dsp->dc_postmix_i);
  IIR_DCBlock_Init(&dsp->dc_postmix_q);

  /* Recompute CW BFO increment for the current sample rate */
  dsp->cw_bfo_inc = (uint32_t)(int64_t)(700.0 / (double)sample_rate * 4294967296.0);
  /* USER CODE END DSP_SetMode_0 */
}

void DSP_SetBW(DSP_State_t *dsp, float bw_hz)
{
  if (bw_hz < 100.0f)   bw_hz = 100.0f;
  if (bw_hz > 24000.0f) bw_hz = 24000.0f;
  uint32_t sr = dsp->sample_rate ? dsp->sample_rate : 48000U;
  dsp->bw_hz = bw_hz;
  float bw_norm = bw_hz / (float)sr;
  FIR_Init_LPF(&dsp->fir_i, bw_norm, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q, bw_norm, FIR_MAX_TAPS);
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
 *             Same MSB-aligned format for DAC output.
 *             Write: (uint32_t)(uint16_t)(int16_t)sample << 16
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
     *           (STM32H7 SAI: DataSize < SlotSize → RX right-justified, TX left-justified) */
    float raw_i = (float)((int16_t)(uint16_t)iq_in[n * 2U + 0U]) * DSP_INV_32767;
    float raw_q = (float)((int16_t)(uint16_t)iq_in[n * 2U + 1U]) * DSP_INV_32767;

    /* ── 2. Pre-mix DC block (ADC DC offset) */
    raw_i = IIR_DCBlock_Process(&dsp->dc_block_i, raw_i);
    raw_q = IIR_DCBlock_Process(&dsp->dc_block_q, raw_q);

    /* ── 3. NCO down-mix to baseband
     *       [mix_i]   [ cos  sin] [raw_i]
     *       [mix_q] = [-sin  cos] [raw_q]  (nco.sin_val is stored negated) */
    NCO_Step(&dsp->nco);
    float mix_i = raw_i * dsp->nco.cos_val - raw_q * dsp->nco.sin_val;
    float mix_q = raw_i * dsp->nco.sin_val + raw_q * dsp->nco.cos_val;

    /* ── 3b. Post-mix DC removal – kills residual LO leakage / QSD imbalance
     *        that survives the pre-mix blocker and would appear as center spike */
    mix_i = IIR_DCBlock_Process(&dsp->dc_postmix_i, mix_i);
    mix_q = IIR_DCBlock_Process(&dsp->dc_postmix_q, mix_q);

    /* ── 3c. IQ mismatch correction (Gram-Schmidt orthogonalization).
     *        Corrects QSD amplitude and phase imbalance to improve image rejection.
     *        Identity when iq_g_inv=1.0 and iq_p=0.0 (uncalibrated default).
     *        Cost: 2 multiplies + 1 subtract per sample. */
    mix_q = (mix_q - dsp->iq_p * mix_i) * dsp->iq_g_inv;

    /* ── 4. FFT feed BEFORE FIR – full ±Fs/2 = ±24kHz bandscope */
    if (dsp->fft_fill < DSP_FFT_SIZE)
    {
      float w = dsp->fft_window[dsp->fft_fill];
      dsp->fft_buf[dsp->fft_fill].re = mix_i * w;
      dsp->fft_buf[dsp->fft_fill].im = mix_q * w;
      dsp->fft_fill++;
      if (dsp->fft_fill >= DSP_FFT_SIZE)
      {
        float peak;
        FFT_Radix2(dsp->fft_buf, DSP_FFT_SIZE);
        FFT_ComputeMag_dB(dsp->fft_buf, dsp->fft_mag_db, DSP_FFT_SIZE, &peak);
        dsp->fft_ready = true;
        if (dsp->wf_lines < 8U) dsp->wf_lines++;
        dsp->fft_fill  = 0U;
      }
    }

    /* ── 5. FIR LPF I and Q (band-limit for demod) */
    float filt_i = FIR_Process(&dsp->fir_i, mix_i);
    float filt_q = FIR_Process(&dsp->fir_q, mix_q);

    /* ── 6. Demodulate */
    float audio;
    switch (dsp->mode)
    {
      case MODE_AM:  audio = Demod_AM(filt_i, filt_q);             break;
      case MODE_FM:  audio = Demod_FM(&dsp->fm, filt_i, filt_q);   break;
      case MODE_USB: audio = Demod_USB(filt_i, filt_q);            break;
      case MODE_LSB: audio = Demod_LSB(filt_i, filt_q);            break;
      case MODE_CW:  audio = Demod_CW(filt_i, filt_q, &dsp->cw_phase_acc, dsp->cw_bfo_inc);  break;
      default:       audio = filt_i;                                 break;
    }

    /* ── 7. Audio LPF */
    audio = FIR_Process(&dsp->fir_audio, audio);

    /* ── 7b. Audio DC block – removes DC bias from AM envelope demodulation.
     *        Without this, AM demod (always-positive envelope) produces a DC
     *        signal that is inaudible through AC-coupled headphone outputs. */
    audio = IIR_DCBlock_Process(&dsp->dc_block_audio, audio);

    /* ── 8. AGC */
    audio = AGC_Process(&dsp->agc, audio);

    /* ── 9. Power for S-meter */
    power_acc += audio * audio;

    /* ── 10. Write: MSB-align 16-bit sample into 32-bit DAC word [31:16] */
    int32_t out_val = (int32_t)(audio * 32767.0f);
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
  for (uint32_t n = 0U; n < len; n++)
  {
    /* ── 1. Pull mono audio from USB TX ring (L channel, int16 LE).
     *       Silence (0.0f) when the ring is empty. */
    float audio = 0.0f;
    if (g_usb_audio.tx_count >= 4U) {
      uint8_t lo = g_usb_audio.tx_ring[g_usb_audio.tx_rd];
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 1U) % USB_AUDIO_RING_SIZE);
      uint8_t hi = g_usb_audio.tx_ring[g_usb_audio.tx_rd];
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 1U) % USB_AUDIO_RING_SIZE);
      int16_t s = (int16_t)((uint16_t)hi << 8 | lo);
      audio = (float)s * DSP_INV_32767;
      /* Discard R channel (2 bytes) */
      g_usb_audio.tx_rd    = (uint16_t)((g_usb_audio.tx_rd + 2U) % USB_AUDIO_RING_SIZE);
      g_usb_audio.tx_count = (uint16_t)(g_usb_audio.tx_count - 4U);
    }

    /* ── 2. Audio DC block – remove mic/line DC offset before Hilbert.
     *       DC in audio produces a carrier tone at the TX LO frequency. */
    audio = IIR_DCBlock_Process(&dsp->tx.dc_block, audio);

    /* ── 3. TX audio gain */
    audio *= dsp->tx.audio_gain;

    /* ── 4. Audio FIR LPF BEFORE modulation.
     *       Filtering the real audio signal band-limits and ensures
     *       equal spectral amplitude entering both I (delay) and Q (Hilbert)
     *       branches. Do NOT add another filter after the modulator. */
    float audio_lp = FIR_Process(&dsp->tx.fir_audio, audio);

    /* ── 4b. TX compressor + soft limiter (prevent overdrive and splatter).
     *
     *   Stage 1 – Peak-tracking compressor (4:1 above 0.70 threshold).
     *             Attack 1 ms / release 50 ms.  Transparent below threshold,
     *             gently reduces gain when speech peaks exceed it.
     *
     *   Stage 2 – C1-smooth soft limiter for residual peaks.
     *             Linear to 0.95; above that asymptotically approaches 1.0
     *             with continuous slope (no hard-clip distortion on the knee).
     *             Formula:  y = 1 − 0.0025 / (|x| − 0.90)   for |x| > 0.95
     *             Verified C1 at knee: dy/da|_{a=0.95} = 1.0 = incoming slope. */
    {
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
    switch (dsp->mode) {
      case MODE_USB:
      case MODE_LSB: {
        /* Phasing method: delay I by Hilbert group delay = (N-1)/2 samples */
        const uint16_t M = (HILBERT_TAPS - 1U) / 2U;
        dsp->tx.audio_delay[dsp->tx.delay_idx] = audio_lp;
        uint16_t ridx = (uint16_t)((dsp->tx.delay_idx + HILBERT_TAPS - M) % HILBERT_TAPS);
        float audio_d = dsp->tx.audio_delay[ridx];
        dsp->tx.delay_idx = (uint16_t)((dsp->tx.delay_idx + 1U) % HILBERT_TAPS);
        float audio_h = Hilbert_Process(&dsp->tx.hilbert, audio_lp);
        tx_i = audio_d;
        tx_q = (dsp->mode == MODE_USB) ? +audio_h : -audio_h;
        break;
      }
      case MODE_AM:
        tx_i = 0.5f + 0.5f * audio_lp;   /* carrier + DSB-AM */
        tx_q = 0.0f;
        break;
      case MODE_FM:
      case MODE_CW:
      default:
        tx_i = audio_lp * 0.7f;
        tx_q = 0.0f;
        break;
    }

    /* ── 6. FFT feed for TX spectrum display */
    if (dsp->fft_fill < DSP_FFT_SIZE)
    {
      float w = dsp->fft_window[dsp->fft_fill];
      dsp->fft_buf[dsp->fft_fill].re = tx_i * w;
      dsp->fft_buf[dsp->fft_fill].im = tx_q * w;
      dsp->fft_fill++;
      if (dsp->fft_fill >= DSP_FFT_SIZE)
      {
        float peak;
        FFT_Radix2(dsp->fft_buf, DSP_FFT_SIZE);
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
  /* USER CODE END DSP_ProcessTX_0 */
}
