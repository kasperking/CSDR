/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_dsp.c
  * @brief   SDR DSP Engine
  *
  *  Module:
  *   - NCO   : Numerically Controlled Oscillator (32-bit phase acc, FPU cos/sin)
  *   - FIR   : Lowpass filter, Hann-windowed sinc, circular buffer, 64 taps
  *   - IIR   : DC blocker  H(z)=(1-z⁻¹)/(1-0.995z⁻¹)
  *   - AGC   : Envelope detector + gain control, attack 10ms / decay 100ms
  *   - FFT   : Radix-2 DIT, N=256, cửa sổ Hann
  *   - DEMOD : AM, FM (atan2 differentiator), USB, LSB, CW (BFO 700Hz)
  *
  *  Luồng xử lý mỗi sample:
  *   DC block → NCO mix → FIR LPF (I+Q) → Demod → Audio LPF → AGC → out
  *   Đồng thời: feed FFT buffer → khi đầy chạy FFT → cập nhật fft_mag_db
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sdr_dsp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DSP_TWO_PI      6.28318530717958647692f
#define DSP_INV_32767   (1.0f / 32767.0f)
#define DSP_INV_4G      (1.0f / 4294967296.0f)   /* 1/2^32 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void bit_reverse(Complex_f *buf, uint16_t n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 *  Nội bộ: bit-reversal permutation cho FFT radix-2
 * ============================================================ */
static void bit_reverse(Complex_f *buf, uint16_t n)
{
  uint16_t i, j = 0U;
  for (i = 1U; i < n; i++)
  {
    uint16_t bit = n >> 1U;
    for (; j & bit; bit >>= 1U) { j ^= bit; }
    j ^= bit;
    if (i < j)
    {
      Complex_f tmp = buf[i];
      buf[i] = buf[j];
      buf[j] = tmp;
    }
  }
}

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/* ============================================================
 *  NCO
 * ============================================================ */

/**
  * @brief  Cài tần số NCO.
  *         phase_inc = freq_hz × 2³² / sample_rate
  * @param  nco          NCO handle
  * @param  freq_hz      Tần số lệch (có dấu, Hz)
  * @param  sample_rate  Tần số lấy mẫu (Hz)
  */
void NCO_SetFrequency(NCO_t *nco, int32_t freq_hz, uint32_t sample_rate)
{
  /* USER CODE BEGIN NCO_SetFrequency_0 */
  nco->phase_inc = (uint32_t)((double)freq_hz /
                               (double)sample_rate * 4294967296.0);
  nco->phase_acc = 0U;
  nco->cos_val   = 1.0f;
  nco->sin_val   = 0.0f;
  /* USER CODE END NCO_SetFrequency_0 */
}

/**
  * @brief  Tiến một bước NCO – tính cos/sin qua FPU Cortex-M7.
  * @param  nco  NCO handle
  */
void NCO_Step(NCO_t *nco)
{
  /* USER CODE BEGIN NCO_Step_0 */
  nco->phase_acc += nco->phase_inc;
  float phi       = (float)nco->phase_acc * (DSP_TWO_PI * DSP_INV_4G);
  nco->cos_val    =  cosf(phi);
  nco->sin_val    = -sinf(phi);   /* âm để xuống tần (down-mix) */
  /* USER CODE END NCO_Step_0 */
}

/* ============================================================
 *  FIR Lowpass Filter  (Hann-windowed sinc)
 * ============================================================ */

/**
  * @brief  Khởi tạo FIR lowpass.
  * @param  fir          FIR handle
  * @param  cutoff_norm  Tần số cắt chuẩn hóa (0..0.5)  = fc/Fs
  * @param  taps         Số taps (max FIR_MAX_TAPS = 64)
  */
void FIR_Init_LPF(FIR_Filter_t *fir, float cutoff_norm, uint16_t taps)
{
  /* USER CODE BEGIN FIR_Init_LPF_0 */
  if (taps > FIR_MAX_TAPS) { taps = FIR_MAX_TAPS; }
  fir->taps = taps;
  fir->idx  = 0U;
  (void)memset(fir->buf, 0, sizeof(fir->buf));

  int32_t half = (int32_t)taps / 2;
  float   sum  = 0.0f;

  for (uint16_t i = 0U; i < taps; i++)
  {
    int32_t n = (int32_t)i - half;
    float h;
    if (n == 0)
    {
      h = 2.0f * cutoff_norm;
    }
    else
    {
      h = sinf(DSP_TWO_PI * cutoff_norm * (float)n) /
          (3.14159265f * (float)n);
    }
    /* Cửa sổ Hann */
    float w    = 0.5f - 0.5f * cosf(DSP_TWO_PI * (float)i / (float)(taps - 1U));
    fir->coeff[i] = h * w;
    sum += fir->coeff[i];
  }

  /* Normalize → DC gain = 1.0 */
  if (fabsf(sum) > 1e-10f)
  {
    for (uint16_t i = 0U; i < taps; i++) { fir->coeff[i] /= sum; }
  }
  /* USER CODE END FIR_Init_LPF_0 */
}

/**
  * @brief  Xử lý một sample qua FIR (vòng tròn buffer).
  * @param  fir  FIR handle
  * @param  x    Sample đầu vào
  * @retval float  Sample đầu ra
  */
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
 *  IIR DC Block
 * ============================================================ */

/**
  * @brief  Khởi tạo bộ lọc DC block bậc 1.
  *         H(z) = (1 − z⁻¹) / (1 − 0.995·z⁻¹)
  */
void IIR_DCBlock_Init(IIR_Biquad_t *f)
{
  /* USER CODE BEGIN IIR_DCBlock_Init_0 */
  f->b0 = 1.0f;  f->b1 = -1.0f; f->b2 = 0.0f;
  f->a1 = -0.995f; f->a2 = 0.0f;
  f->x1 = 0.0f; f->x2 = 0.0f;
  f->y1 = 0.0f; f->y2 = 0.0f;
  /* USER CODE END IIR_DCBlock_Init_0 */
}

/**
  * @brief  Xử lý một sample qua IIR biquad.
  */
float IIR_DCBlock_Process(IIR_Biquad_t *f, float x)
{
  /* USER CODE BEGIN IIR_DCBlock_Process_0 */
  float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
            - f->a1 * f->y1 - f->a2 * f->y2;
  f->x2 = f->x1; f->x1 = x;
  f->y2 = f->y1; f->y1 = y;
  return y;
  /* USER CODE END IIR_DCBlock_Process_0 */
}

/* ============================================================
 *  AGC
 * ============================================================ */

/**
  * @brief  Khởi tạo AGC.
  *         Attack 10ms, Decay 100ms, Target 0.5, Gain range 0.001..1000
  */
void AGC_Init(AGC_t *agc, uint32_t sample_rate)
{
  /* USER CODE BEGIN AGC_Init_0 */
  agc->gain      = 1.0f;
  agc->target    = 0.5f;
  agc->max_gain  = 1000.0f;
  agc->min_gain  = 0.001f;
  agc->level     = 0.0f;
  agc->attack    = expf(-1.0f / (0.010f * (float)sample_rate));  /* 10ms */
  agc->decay     = expf(-1.0f / (0.100f * (float)sample_rate));  /* 100ms */
  /* USER CODE END AGC_Init_0 */
}

/**
  * @brief  Xử lý AGC: envelope detect → gain adjust → apply.
  * @param  agc  AGC handle
  * @param  x    Sample vào
  * @retval float  Sample đã khuếch đại
  */
float AGC_Process(AGC_t *agc, float x)
{
  /* USER CODE BEGIN AGC_Process_0 */
  float env = fabsf(x);

  /* Envelope follower: attack nhanh, decay chậm */
  if (env > agc->level)
    { agc->level = agc->attack  * agc->level + (1.0f - agc->attack)  * env; }
  else
    { agc->level = agc->decay   * agc->level + (1.0f - agc->decay)   * env; }

  /* Tính gain */
  if (agc->level > 1e-10f) { agc->gain = agc->target / agc->level; }
  if (agc->gain > agc->max_gain) { agc->gain = agc->max_gain; }
  if (agc->gain < agc->min_gain) { agc->gain = agc->min_gain; }

  return x * agc->gain;
  /* USER CODE END AGC_Process_0 */
}

/* ============================================================
 *  FFT Radix-2 DIT (Cooley-Tukey)
 * ============================================================ */

/**
  * @brief  Tạo cửa sổ Hann cho N điểm.
  * @param  w  Mảng đầu ra (N phần tử)
  * @param  n  Kích thước FFT
  */
void FFT_Hann_Window(float *w, uint16_t n)
{
  /* USER CODE BEGIN FFT_Hann_Window_0 */
  for (uint16_t i = 0U; i < n; i++)
  {
    w[i] = 0.5f - 0.5f * cosf(DSP_TWO_PI * (float)i / (float)(n - 1U));
  }
  /* USER CODE END FFT_Hann_Window_0 */
}

/**
  * @brief  FFT Radix-2 DIT in-place (N phải là lũy thừa 2).
  * @param  buf  Mảng Complex_f kích thước n (input + output)
  * @param  n    Kích thước FFT (= DSP_FFT_SIZE = 256)
  */
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

        buf[even].re = u.re + v.re;
        buf[even].im = u.im + v.im;
        buf[odd].re  = u.re - v.re;
        buf[odd].im  = u.im - v.im;

        float new_cr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = new_cr;
      }
    }
  }
  /* USER CODE END FFT_Radix2_0 */
}

/**
  * @brief  Tính độ lớn dB từ kết quả FFT phức.
  * @param  buf      Mảng Complex_f sau FFT_Radix2
  * @param  mag_db   Mảng đầu ra (half_n phần tử)
  * @param  half_n   DSP_FFT_HALF = 128
  * @param  peak_db  Con trỏ nhận giá trị đỉnh dB
  */
void FFT_ComputeMag_dB(const Complex_f *buf, float *mag_db,
                        uint16_t half_n, float *peak_db)
{
  /* USER CODE BEGIN FFT_ComputeMag_0 */
  /* Full-size complex-FFT với fftshift:
   *   Input:  buf[0..N-1] - FFT complex output (N = DSP_FFT_SIZE)
   *   Output: mag_db[0..N-1] re-arranged:
   *     mag_db[0..N/2-1]   = buf[N/2..N-1]  (negative frequencies -Fs/2..-1)
   *     mag_db[N/2..N-1]   = buf[0..N/2-1]  (positive frequencies  0  ..+Fs/2)
   *   Center (DC, 0 Hz) nằm tại mag_db[N/2]
   *
   *   Tham số half_n vẫn giữ tên nhưng dùng làm full_n (để tương thích).
   *   Caller truyền vào DSP_FFT_SIZE = 256. */
  const uint16_t n = half_n;   /* thực ra là full size = DSP_FFT_SIZE */
  const uint16_t h = n / 2U;
  *peak_db = -120.0f;

  /* Negative half: buf[h..n-1] → mag_db[0..h-1] */
  for (uint16_t i = 0U; i < h; i++)
  {
    float mag = sqrtf(buf[h + i].re * buf[h + i].re
                       + buf[h + i].im * buf[h + i].im);
    float db  = 20.0f * log10f(mag + 1e-10f);
    mag_db[i] = db;
    if (db > *peak_db) { *peak_db = db; }
  }
  /* Positive half: buf[0..h-1] → mag_db[h..n-1] */
  for (uint16_t i = 0U; i < h; i++)
  {
    float mag = sqrtf(buf[i].re * buf[i].re + buf[i].im * buf[i].im);
    float db  = 20.0f * log10f(mag + 1e-10f);
    mag_db[h + i] = db;
    if (db > *peak_db) { *peak_db = db; }
  }
  /* USER CODE END FFT_ComputeMag_0 */
}

/* ============================================================
 *  Demodulators
 * ============================================================ */

/**
  * @brief  AM: biên độ bao (envelope).
  *         audio = sqrt(I² + Q²)
  */
float Demod_AM(float i, float q)
{
  /* USER CODE BEGIN Demod_AM_0 */
  return sqrtf(i * i + q * q);
  /* USER CODE END Demod_AM_0 */
}

/**
  * @brief  FM: phân biệt pha tức thời (atan2 differentiator).
  *         Δφ = arg( z[n] · conj(z[n-1]) ) / π  → [-1, +1]
  *         Sau đó qua de-emphasis 75µs.
  */
float Demod_FM(FM_Demod_t *fm, float i, float q)
{
  /* USER CODE BEGIN Demod_FM_0 */
  float re = i * fm->prev_re + q * fm->prev_im;
  float im = q * fm->prev_re - i * fm->prev_im;
  fm->prev_re = i;
  fm->prev_im = q;
  float d = atan2f(im, re) * (1.0f / 3.14159265f);   /* normalized -1..+1 */

  /* De-emphasis: single-pole IIR lowpass */
  float y = fm->de_emph.b0 * d - fm->de_emph.a1 * fm->de_emph.y1;
  fm->de_emph.y1 = y;
  return y;
  /* USER CODE END Demod_FM_0 */
}

/**
  * @brief  USB (Upper Sideband): phasing method đơn giản.
  *         audio = (I + Q) / 2
  * @note   Để chất lượng cao hơn, dùng Hilbert FIR 64-tap.
  */
float Demod_USB(float i, float q)
{
  /* USER CODE BEGIN Demod_USB_0 */
  return (i + q) * 0.5f;
  /* USER CODE END Demod_USB_0 */
}

/**
  * @brief  LSB (Lower Sideband): phasing method.
  *         audio = (I − Q) / 2
  */
float Demod_LSB(float i, float q)
{
  /* USER CODE BEGIN Demod_LSB_0 */
  return (i - q) * 0.5f;
  /* USER CODE END Demod_LSB_0 */
}

/**
  * @brief  CW: AM envelope × BFO 700Hz tone.
  *         Tạo âm thanh từ Morse carrier.
  * @param  i          Kênh I sau lọc
  * @param  q          Kênh Q sau lọc
  * @param  phase_acc  Con trỏ tới tích lũy pha BFO (duy trì giữa các call)
  */
float Demod_CW(float i, float q, uint32_t *phase_acc)
{
  /* USER CODE BEGIN Demod_CW_0 */
  float amp = Demod_AM(i, q);
  /* BFO 700Hz NCO: phase_inc = 700/48000 × 2^32 */
  *phase_acc += 62500000UL;   /* ≈ 700/48000 × 4294967296 */
  float bfo   = cosf((float)(*phase_acc) * (DSP_TWO_PI * DSP_INV_4G));
  return amp * bfo;
  /* USER CODE END Demod_CW_0 */
}

/* ============================================================
 *  DSP Engine – API công khai
 * ============================================================ */

/**
  * @brief  Khởi tạo toàn bộ DSP engine.
  * @param  dsp          DSP state
  * @param  sample_rate  Tần số lấy mẫu (Hz), thường 48000
  */
void DSP_Init(DSP_State_t *dsp, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_Init_0 */
  (void)memset(dsp, 0, sizeof(DSP_State_t));
  dsp->mode = MODE_AM;

  /* NCO mặc định tại 0Hz (không dịch tần) */
  NCO_SetFrequency(&dsp->nco, 0, sample_rate);

  /* FIR LPF cắt 6kHz (AM bandwidth) */
  float bw = 6000.0f / (float)sample_rate;
  FIR_Init_LPF(&dsp->fir_i,     bw, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q,     bw, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_audio, 4000.0f / (float)sample_rate, 32U);

  /* DC block I và Q */
  IIR_DCBlock_Init(&dsp->dc_block_i);
  IIR_DCBlock_Init(&dsp->dc_block_q);

  /* AGC */
  AGC_Init(&dsp->agc, sample_rate);

  /* Cửa sổ Hann cho FFT */
  FFT_Hann_Window(dsp->fft_window, DSP_FFT_SIZE);

  /* FM de-emphasis: pole = exp(-2π·fc/Fs), fc = 1/(2π·75µs) ≈ 2122Hz */
  {
    float de_fc          = 2122.0f / (float)sample_rate;
    dsp->fm.de_emph.b0   = 1.0f - expf(-DSP_TWO_PI * de_fc);
    dsp->fm.de_emph.a1   = -(1.0f - dsp->fm.de_emph.b0);
    dsp->fm.de_emph.y1   = 0.0f;
    dsp->fm.prev_re      = 0.0f;
    dsp->fm.prev_im      = 0.0f;
  }

  /* TX state init */
  Hilbert_Init(&dsp->tx.hilbert);
  for (int i = 0; i < HILBERT_TAPS; i++) dsp->tx.audio_delay[i] = 0.0f;
  dsp->tx.delay_idx    = 0;
  dsp->tx.fm_phase     = 0.0f;
  dsp->tx.cw_phase_acc = 0;
  dsp->tx.audio_gain   = 1.0f;

  dsp->signal_power_db = -120.0f;
  /* USER CODE END DSP_Init_0 */
}

/**
  * @brief  Cập nhật NCO khi thay đổi tần số thu.
  * @param  dsp          DSP state
  * @param  freq_hz      Tần số thu (Hz)
  * @param  if_hz        Tần số IF của front-end (Hz, 0 nếu direct-conversion)
  * @param  sample_rate  Fs (Hz)
  */
void DSP_SetFrequency(DSP_State_t *dsp, uint32_t freq_hz,
                       uint32_t if_hz, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_SetFrequency_0 */
  int32_t delta = (int32_t)freq_hz - (int32_t)if_hz;
  NCO_SetFrequency(&dsp->nco, delta, sample_rate);
  /* USER CODE END DSP_SetFrequency_0 */
}

/**
  * @brief  Thay đổi mode điều chế – cập nhật băng thông FIR.
  * @param  dsp          DSP state
  * @param  mode         SDR_Mode_t mới
  * @param  sample_rate  Fs (Hz)
  */
void DSP_SetMode(DSP_State_t *dsp, SDR_Mode_t mode, uint32_t sample_rate)
{
  /* USER CODE BEGIN DSP_SetMode_0 */
  dsp->mode        = mode;
  dsp->sample_rate = sample_rate;

  float bw_hz;
  switch (mode)
  {
    case MODE_AM:  bw_hz = 6000.0f;  break;
    case MODE_FM:  bw_hz = 15000.0f; break;
    case MODE_USB:
    case MODE_LSB: bw_hz = 3000.0f;  break;
    case MODE_CW:  bw_hz = 500.0f;   break;
    default:       bw_hz = 4000.0f;  break;
  }
  dsp->bw_hz = bw_hz;

  float bw_norm = bw_hz / (float)sample_rate;
  FIR_Init_LPF(&dsp->fir_i, bw_norm, FIR_MAX_TAPS);
  FIR_Init_LPF(&dsp->fir_q, bw_norm, FIR_MAX_TAPS);

  /* Reset DC block state khi mode switch - tránh residue từ mode trước */
  IIR_DCBlock_Init(&dsp->dc_block_i);
  IIR_DCBlock_Init(&dsp->dc_block_q);
  /* USER CODE END DSP_SetMode_0 */
}

/**
  * @brief  Xử lý một block audio I/Q (gọi từ DMA callback).
  *
  *  Input  iq_in[]:  int32 interleaved [I0,Q0, I1,Q1, ...]
  *                   Mỗi giá trị là sample 16-bit nằm ở bit [31:16]
  *                   (SAI1 DMA word-aligned, 16-bit slot trong 32-bit)
  *  Output audio_out[]: int32 stereo interleaved [L0,R0, L1,R1, ...]
  *                   Ghi lại SAI1_A TX DMA để DAC phát âm thanh.
  *
  * @param  dsp       DSP state
  * @param  iq_in     Buffer RX từ DMA (int32, interleaved I/Q)
  * @param  audio_out Buffer TX tới DMA (int32, interleaved stereo)
  * @param  len       Số cặp I/Q (= AUDIO_BLOCK_SIZE)
  */
void DSP_Process(DSP_State_t *dsp,
                  const int32_t *iq_in,
                  int32_t       *audio_out,
                  uint32_t       len)
{
  /* USER CODE BEGIN DSP_Process_0 */
  static uint32_t cw_phase = 0U;
  float power_acc = 0.0f;

  for (uint32_t n = 0U; n < len; n++)
  {
    /* ── 1. Đọc I/Q từ SAI DMA buffer ─────────────────────────
     *      SAI1 16-bit slot trong 32-bit word: sample ở [31:16]
     *      Shift phải 16 để lấy signed 16-bit, rồi normalize   */
    /* SAI slot 32-bit with 16-bit data: STM32H7 LSB-aligns data in slot.
     * Cast to int16_t extracts the lower 16 bits as signed value. */
    float raw_i = (float)((int16_t)iq_in[n * 2U + 0U]) * DSP_INV_32767;
    float raw_q = (float)((int16_t)iq_in[n * 2U + 1U]) * DSP_INV_32767;

    /* ── 2. DC block (loại bỏ offset DC của ADC) ────────────── */
    raw_i = IIR_DCBlock_Process(&dsp->dc_block_i, raw_i);
    raw_q = IIR_DCBlock_Process(&dsp->dc_block_q, raw_q);

    /* ── 3. Trộn với NCO (xuống tần về baseband) ─────────────
     *      [mix_i]   [cos  -sin] [raw_i]
     *      [mix_q] = [sin   cos] [raw_q]                       */
    NCO_Step(&dsp->nco);
    float mix_i = raw_i * dsp->nco.cos_val - raw_q * dsp->nco.sin_val;
    float mix_q = raw_i * dsp->nco.sin_val + raw_q * dsp->nco.cos_val;

    /* ── 4. FFT feed TRƯỚC khi FIR cắt - hiển thị TOÀN DẢI ───
     *      Lấy mix_i/mix_q (đã shift xuống baseband qua NCO) vào FFT buf.
     *      Nhờ đó bandscope thấy toàn bộ ±Fs/2 = ±24kHz quanh tần số tuning,
     *      các tín hiệu off-channel vẫn hiện để scanning. */
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
        dsp->fft_fill  = 0U;
      }
    }

    /* ── 5. FIR Lowpass lọc I và Q cho demod (band-limit audio path) ─ */
    float filt_i = FIR_Process(&dsp->fir_i, mix_i);
    float filt_q = FIR_Process(&dsp->fir_q, mix_q);

    /* ── 6. Demodulate ─────────────────────────────────────── */
    float audio;
    switch (dsp->mode)
    {
      case MODE_AM:  audio = Demod_AM(filt_i, filt_q);               break;
      case MODE_FM:  audio = Demod_FM(&dsp->fm, filt_i, filt_q);     break;
      case MODE_USB: audio = Demod_USB(filt_i, filt_q);              break;
      case MODE_LSB: audio = Demod_LSB(filt_i, filt_q);              break;
      case MODE_CW:  audio = Demod_CW(filt_i, filt_q, &cw_phase);    break;
      default:       audio = filt_i;                                   break;
    }

    /* ── 7. Audio LPF ──────────────────────────────────────── */
    audio = FIR_Process(&dsp->fir_audio, audio);

    /* ── 8. AGC ─────────────────────────────────────────────── */
    audio = AGC_Process(&dsp->agc, audio);

    /* ── 9. Power accumulate cho S-meter ────────────────────── */
    power_acc += audio * audio;

    /* ── 10. Scale → int32 DAC output ───────────────────────
     *       SAI1_A slot 32-bit, data 16-bit: STM32H7 LSB-align.
     *       Data phải ở bit [15:0], bit [31:16] padding 0.
     *       Symmetric với RX reading (int16_t cast).              */
    int32_t out_val = (int32_t)(audio * 32767.0f);
    out_val = (out_val > 32767)  ?  32767  : out_val;
    out_val = (out_val < -32768) ? -32768  : out_val;
    /* Store 16-bit value in LSB of 32-bit word (no shift). Upper bits = 0. */
    int32_t dac_word = out_val & 0x0000FFFF;
    audio_out[n * 2U + 0U] = dac_word;  /* L */
    audio_out[n * 2U + 1U] = dac_word;  /* R */
  }

  /* Cập nhật công suất trung bình → S-meter (EMA) */
  if (len > 0U)
  {
    float rms_db = 20.0f * log10f(sqrtf(power_acc / (float)len) + 1e-10f);
    dsp->signal_power_db = 0.9f * dsp->signal_power_db + 0.1f * rms_db;
  }

  dsp->sample_count += len;
  /* USER CODE END DSP_Process_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */


/* ════════════════════════════════════════════════════════════════
 *  Hilbert Transform FIR (90° phase shift)
 *
 *  Coefficients h[n] for n=0..N-1, N odd:
 *    h[n] = 0                        if (n-M) even
 *    h[n] = 2/(pi*(n-M)) * window    if (n-M) odd
 *  where M = (N-1)/2 is center tap.
 *  Windowed with Hamming to reduce sidelobes.
 *
 *  Group delay = M samples → need matched delay line for I channel
 *  Passband: ~1/N to ~1-1/N of Nyquist (good 500Hz..23kHz @ 48kHz)
 * ════════════════════════════════════════════════════════════════ */

void Hilbert_Init(Hilbert_t *h)
{
  const int N = HILBERT_TAPS;
  const int M = (N - 1) / 2;

  for (int n = 0; n < N; n++) {
    int k = n - M;
    float c;
    if ((k & 1) == 0) {
      c = 0.0f;                      /* even samples = 0 */
    } else {
      c = 2.0f / (3.14159265358979f * (float)k);
      /* Hamming window */
      float w = 0.54f - 0.46f * cosf(2.0f * 3.14159265358979f * (float)n
                                      / (float)(N - 1));
      c *= w;
    }
    h->coeff[n] = c;
    h->buf[n]   = 0.0f;
  }
  h->idx = 0;
}

float Hilbert_Process(Hilbert_t *h, float x)
{
  const int N = HILBERT_TAPS;
  h->buf[h->idx] = x;

  float acc = 0.0f;
  int k = h->idx;
  for (int n = 0; n < N; n++) {
    acc += h->coeff[n] * h->buf[k];
    k = (k == 0) ? (N - 1) : (k - 1);
  }
  h->idx = (uint16_t)((h->idx + 1U) % N);
  return acc;
}

/* ════════════════════════════════════════════════════════════════
 *  DSP_ProcessTX – TX DSP pipeline
 *
 *  Input source: USB Audio OUT ring (g_usb_audio.tx_ring).
 *    Format: L=audio_L, R=audio_R int16 interleaved LE.
 *    Mono-ize: use L channel only (PC radio apps typically send mono SSB audio).
 *
 *  Modulation by mode:
 *    USB: I = audio_delayed, Q = +Hilbert(audio)
 *    LSB: I = audio_delayed, Q = -Hilbert(audio)
 *    AM:  I = (0.5 + 0.5*audio), Q = 0
 *    FM:  phase += audio*dev → I = cos(phase), Q = sin(phase)
 *    CW:  I = key*cos(700Hz), Q = key*sin(700Hz) [key = TX audio amplitude]
 *
 *  FFT feed: IQ output đã modulate → FFT thấy spectrum SSB
 *
 *  Output: int32 LSB-aligned interleaved [I0, Q0, I1, Q1, ...]
 * ════════════════════════════════════════════════════════════════ */

/* External USB audio ring access */
#include "usb_audio.h"
extern USB_Audio_Handle_t g_usb_audio;

void DSP_ProcessTX(DSP_State_t *dsp, int32_t *iq_out, uint32_t len)
{
  /* USER CODE BEGIN DSP_ProcessTX_0 */

  /* TX DEBUG: nếu không có USB audio, dùng internal 1kHz square wave
   * để test TX pipeline (xác nhận FFT/modulator/DAC chain work).
   * Sau khi confirm USB audio work thì bỏ. */
  static uint16_t s_test_tone_cnt = 0;

  for (uint32_t n = 0U; n < len; n++)
  {
    /* ── 1. Pull mono audio sample từ USB TX ring ─────────────
     *      USB ring format: [L_lo, L_hi, R_lo, R_hi] per sample
     *      4 bytes = 1 stereo pair. Ưu tiên L channel (mono SSB). */
    float audio = 0.0f;
    bool got_usb = false;
    if (g_usb_audio.tx_count >= 4U) {
      uint8_t lo = g_usb_audio.tx_ring[g_usb_audio.tx_rd];
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 1U) % USB_AUDIO_RING_SIZE);
      uint8_t hi = g_usb_audio.tx_ring[g_usb_audio.tx_rd];
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 1U) % USB_AUDIO_RING_SIZE);
      int16_t s = (int16_t)((uint16_t)hi << 8 | lo);
      audio = (float)s * DSP_INV_32767;

      /* Skip R channel (2 bytes) */
      g_usb_audio.tx_rd = (uint16_t)((g_usb_audio.tx_rd + 2U) % USB_AUDIO_RING_SIZE);
      g_usb_audio.tx_count = (uint16_t)(g_usb_audio.tx_count - 4U);
      got_usb = true;
    }

    if (!got_usb) {
      /* Internal 1kHz square wave: 48kHz/48samples = 1kHz period */
      s_test_tone_cnt = (uint16_t)((s_test_tone_cnt + 1U) % 48U);
      audio = (s_test_tone_cnt < 24U) ? 0.3f : -0.3f;
    }

    /* TX audio gain (mặc định 1.0, có thể scale) */
    audio *= dsp->tx.audio_gain;

    /* ── 2. Modulate theo mode ──────────────────────────────── */
    float tx_i = 0.0f, tx_q = 0.0f;

    switch (dsp->mode) {
      case MODE_USB:
      case MODE_LSB: {
        /* Phasing method: delay I by Hilbert group delay = M = (N-1)/2 = 15
         * Q = Hilbert(audio) (90° shifted) */
        const uint16_t M = (HILBERT_TAPS - 1U) / 2U;
        /* Write audio vào delay line */
        dsp->tx.audio_delay[dsp->tx.delay_idx] = audio;
        /* Read delayed audio from M samples ago */
        uint16_t read_idx = (uint16_t)((dsp->tx.delay_idx + HILBERT_TAPS - M)
                                         % HILBERT_TAPS);
        float audio_d = dsp->tx.audio_delay[read_idx];
        dsp->tx.delay_idx = (uint16_t)((dsp->tx.delay_idx + 1U) % HILBERT_TAPS);

        float audio_h = Hilbert_Process(&dsp->tx.hilbert, audio);

        if (dsp->mode == MODE_USB) {
          tx_i = audio_d;
          tx_q = audio_h;         /* +Hilbert for USB */
        } else {
          tx_i = audio_d;
          tx_q = -audio_h;        /* -Hilbert for LSB */
        }
        break;
      }

      case MODE_AM: {
        /* AM: carrier + modulated envelope
         *   I = (0.5 + 0.5*audio), Q = 0
         *   DC offset = 0.5 → carrier, audio modulates ±0.5 */
        tx_i = 0.5f + 0.5f * audio;
        tx_q = 0.0f;
        break;
      }

      case MODE_FM:
      case MODE_CW:
      default:
        /* FM/CW TX not yet implemented (would need cosf/sinf). AM fallback. */
        tx_i = audio * 0.7f;
        tx_q = 0.0f;
        break;
    }

    /* ── 3. FIR LPF band-limit (optional, dùng chung fir_i/fir_q) ─ */
    tx_i = FIR_Process(&dsp->fir_i, tx_i);
    tx_q = FIR_Process(&dsp->fir_q, tx_q);

    /* ── 3b. DC block để loại bỏ carrier leakage ở center ─── */
    tx_i = IIR_DCBlock_Process(&dsp->dc_block_i, tx_i);
    tx_q = IIR_DCBlock_Process(&dsp->dc_block_q, tx_q);

    /* ── 4. Feed FFT buffer để display TX spectrum ──────────── */
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

    /* ── 5. Scale → int32 DAC output LSB-aligned ────────────── */
    int32_t i_val = (int32_t)(tx_i * 32767.0f);
    int32_t q_val = (int32_t)(tx_q * 32767.0f);
    if (i_val >  32767)  i_val =  32767;
    if (i_val < -32768)  i_val = -32768;
    if (q_val >  32767)  q_val =  32767;
    if (q_val < -32768)  q_val = -32768;

    iq_out[n * 2U + 0U] = (int32_t)((uint32_t)(uint16_t)i_val & 0x0000FFFFU);
    iq_out[n * 2U + 1U] = (int32_t)((uint32_t)(uint16_t)q_val & 0x0000FFFFU);
  }
  /* USER CODE END DSP_ProcessTX_0 */
}
