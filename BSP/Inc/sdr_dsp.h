/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_dsp.h
  * @brief   SDR DSP Engine – NCO · FIR · AGC · FFT · Demodulators
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SDR_DSP_H
#define __SDR_DSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "csdr_app.h"
#include "cw_decode.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define DSP_FFT_SIZE    512U
#define DSP_FFT_HALF    (DSP_FFT_SIZE / 2U)
#define FIR_MAX_TAPS    64U

/* Exported types ------------------------------------------------------------*/

/** Số phức floating-point */
typedef struct { float re; float im; } Complex_f;

/** NCO – Numerically Controlled Oscillator */
typedef struct {
  uint32_t phase_acc;   /*!< Bộ tích lũy pha 32-bit      */
  uint32_t phase_inc;   /*!< Bước pha = f·2³²/Fs         */
  float    cos_val;     /*!< cos(ωt) – kênh I             */
  float    sin_val;     /*!< -sin(ωt) – kênh Q (xuống tần)*/
} NCO_t;

/** FIR Filter (vòng tròn) */
typedef struct {
  float    coeff[FIR_MAX_TAPS];  /*!< Hệ số lọc     */
  float    buf[FIR_MAX_TAPS];    /*!< Circular buffer*/
  uint16_t taps;                 /*!< Số taps        */
  uint16_t idx;                  /*!< Con trỏ ghi    */
} FIR_Filter_t;

/** IIR Biquad (DC block hoặc de-emphasis) */
typedef struct {
  float b0, b1, b2;
  float a1, a2;
  float x1, x2, y1, y2;
} IIR_Biquad_t;

/** AGC – Automatic Gain Control (peak-hold + hang) */
typedef struct {
  float    gain;        /*!< Current gain                       */
  float    attack;      /*!< Attack coeff  (~1 ms)              */
  float    decay;       /*!< Release coeff (mode-dependent)     */
  float    target;      /*!< Target output level (0–1)          */
  float    max_gain;
  float    min_gain;
  float    level;       /*!< Peak envelope estimate             */
  float    env_smooth;  /*!< Pre-smoothed envelope (α=0.9, ~0.2 ms) */
  uint32_t hang_timer;  /*!< Samples remaining in hang hold     */
  uint32_t hang_time;   /*!< Hang duration (samples)            */
  bool     bypass;      /*!< true = FM/DIGI: unity gain, no AGC */
  /* AUTO mode (speed=2): adapt decay/hang by signal rate-of-change */
  bool     auto_mode;   /*!< Adaptive timing active             */
  float    decay_fast;  /*!< Fast release coeff (blend ref)     */
  float    decay_slow;  /*!< Slow release coeff (blend ref)     */
  uint32_t hang_fast;   /*!< Fast hang samples  (blend ref)     */
  uint32_t hang_slow;   /*!< Slow hang samples  (blend ref)     */
  float    prev_level;  /*!< Level from previous sample         */
  float    drate;       /*!< IIR |d(level)/dt|, τ≈70ms         */
} AGC_t;

/* ── Calibration measurement accumulator ──────────────────────────────────── */
typedef enum {
  DSP_CAL_IDLE = 0,
  DSP_CAL_DC,   /*!< Accumulate raw ADC mean (static I/Q bias)        */
  DSP_CAL_IQ,   /*!< Accumulate I²/Q²/IQ cross for mismatch estimate  */
} DSP_CalMode_t;

typedef struct {
  DSP_CalMode_t     mode;
  uint32_t          n_target;
  volatile uint32_t n_count;
  float             acc_i,  acc_q;           /*!< DC: raw ADC count sums  */
  float             acc_ii, acc_qq, acc_iq;  /*!< IQ: power + cross-term  */
  float             result_dc_i;             /*!< ADC count units          */
  float             result_dc_q;
  float             result_iq_gain;          /*!< × 1000 (millis)          */
  float             result_iq_phase;         /*!< milliradians             */
  volatile bool     done;
} DSP_CalMeas_t;

/** Noise Blanker – time-domain impulse suppressor for HF (PSU spikes, ignition).
 *  Blanked samples are replaced with a linear ramp between the last good
 *  sample before the impulse and the first good sample after it (soft /
 *  interpolating blanker), instead of hard-zeroing.  This needs the "after"
 *  endpoint before the blanked window is read back out, so IQ runs through
 *  a small fixed delay line (NB_RING_LEN samples, ~167 us at 48 kHz) while
 *  NB is enabled; NB disabled adds zero latency. */
#define NB_RING_LEN 8U  /* power-of-2, > max blank_width (6) so the "after" anchor is always captured before its ring slot is read out */
typedef struct {
  bool     enabled;               /*!< Runtime on/off toggle (disabled by default) */
  uint8_t  level;                 /*!< Intensity 0-100; higher = lower threshold   */
  float    floor_sq;              /*!< IIR-tracked background power (I²+Q²), τ≈208ms */
  uint8_t  blank_ctr;            /*!< Samples remaining in current blank window   */
  uint8_t  blank_width;          /*!< Precomputed blanking window width (samples)  */
  float    threshold_ratio_sq;   /*!< Precomputed ratio²; trigger when mag²>floor²×this */
  /* Interpolating-blanker delay line */
  float    ring_i[NB_RING_LEN];  /*!< IQ delay-line ring buffer                    */
  float    ring_q[NB_RING_LEN];
  uint8_t  widx;                 /*!< Ring write index                             */
  uint8_t  blank_start_widx;     /*!< Ring index of the first blanked sample       */
  float    anchor_i, anchor_q;   /*!< Last good IQ sample before the impulse       */
  bool     awaiting_anchor;      /*!< True for one tick after blank_ctr hits 0, until the next live sample supplies the "after" anchor */
  uint8_t  prime_ctr;            /*!< Ring fill countdown after enable; bypass (no delay) while priming so stale ring data is never output */
  /* Debug counters – read in debugger, no UI needed */
  uint32_t trig_count;           /*!< Total blanking events since last init/enable */
  float    peak_mag_sq;          /*!< Largest I²+Q² seen (max impulse power)      */
  float    current_threshold_sq; /*!< floor_sq × threshold_ratio_sq, last sample  */
} NoiseBlanker_t;

/** LMS Predictive Noise Reduction (audio domain) */
#define NR_TAPS  32U
typedef struct {
  float   w[NR_TAPS];   /*!< LMS weight vector (adapts to signal) */
  float   x[NR_TAPS];   /*!< Input delay line                     */
  bool    enabled;
} NR_t;

/** FM Demodulator (phân biệt pha tức thời) */
typedef struct {
  float        prev_re;
  float        prev_im;
  IIR_Biquad_t de_emph;  /*!< De-emphasis 75µs */
} FM_Demod_t;

/* Hilbert FIR cho SSB modulate/demodulate - 63 taps odd-symmetric */
#define HILBERT_TAPS    63U
/* Group delay of the Hilbert FIR = (HILBERT_TAPS-1)/2 samples */
#define HILBERT_DELAY   ((HILBERT_TAPS - 1U) / 2U)   /* 31 samples */

typedef struct {
  float    coeff[HILBERT_TAPS];
  float    buf  [HILBERT_TAPS];
  uint16_t idx;
} Hilbert_t;

/* TX state */
typedef struct {
  Hilbert_t    hilbert;                    /*!< 90° phase shift for Q channel */
  float        audio_delay[HILBERT_TAPS];  /*!< Match delay cho I channel */
  uint16_t     delay_idx;
  float        fm_phase;                   /*!< FM modulator phase accumulator */
  uint32_t     cw_phase_acc;               /*!< CW tone NCO phase accumulator       */
  uint32_t     cw_bfo_inc;                 /*!< CW sidetone NCO increment            */
  float        cw_sidetone_amp;            /*!< Target sidetone amplitude (0..0.7)   */
  float        cw_env_amp;                 /*!< Smoothed keying envelope (click-free)*/
  float        audio_gain;                 /*!< TX audio gain (0..1) */
  bool         tune_active;                /*!< TUNE button held: forces a continuous
                                                  CW-style carrier regardless of mode,
                                                  scaled by audio_gain (see csdr_app.c) */
  float        tx_lp_hz;                  /*!< TX High-cut (LPF) Hz */
  float        tx_hp_hz;                  /*!< TX Low-cut (HPF) Hz  */
  FIR_Filter_t fir_audio;                 /*!< TX-private audio LPF (separate from RX) */
  IIR_Biquad_t dc_block;                  /*!< TX-private audio DC blocker (separate from RX) */
  IIR_Biquad_t hp_audio;                  /*!< TX Low-cut HPF (1st-order Butterworth) */
  /* Compressor/limiter – applied after FIR LPF, before modulator.
   * Bypassed automatically when dsp->mode is MODE_DIGU or MODE_DIGL. */
  float        comp_env;     /*!< Compressor peak envelope            */
  float        comp_attack;  /*!< Attack coeff  (~1 ms)               */
  float        comp_decay;   /*!< Release coeff (~50 ms)              */
} TX_State_t;

/** Trạng thái DSP toàn bộ */
typedef struct {
  NCO_t        nco;        /*!< Stage 1: LO offset correction (−lo_offset_hz) */
  NCO_t        nco_if;     /*!< Stage 2: IF shift (independent passband tuning) */
  FIR_Filter_t fir_i;
  FIR_Filter_t fir_q;
  FIR_Filter_t fir_audio;
  IIR_Biquad_t dc_block_i;
  IIR_Biquad_t dc_block_q;
  IIR_Biquad_t dc_block_audio;
  IIR_Biquad_t dc_postmix_i;   /*!< Post-mix DC removal (LO leakage) */
  IIR_Biquad_t dc_postmix_q;
  /* IQ mismatch correction (Gram-Schmidt, applied after post-mix DC block) */
  float        iq_g_inv;       /*!< 1/(1+gain_err): Q amplitude scale  */
  float        iq_p;           /*!< phase_err (rad): Q -= iq_p * I     */
  FM_Demod_t   fm;
  AGC_t        agc;
  NoiseBlanker_t nb;  /*!< HF impulse noise blanker (disabled by default) */
  NR_t           nr;  /*!< LMS predictive noise reduction (audio domain)  */

  /* Static ADC DC offset (from auto-cal, applied pre-IIR in DSP_Process) */
  float         dc_i_static;   /*!< ADC count units subtracted from raw I */
  float         dc_q_static;
  /* Cal measurement accumulator (armed by DSP_CalStart, polled by DSP_CalPoll) */
  DSP_CalMeas_t cal_meas;

  /* Current bandwidth (Hz) và sample rate - dùng cho UI hiển thị BW marker */
  float     bw_hz;
  uint32_t  sample_rate;

  /* FFT */
  Complex_f fft_buf[DSP_FFT_SIZE];
  float     fft_mag_db[DSP_FFT_SIZE];   /* full 512 bins: [-Fs/2 .. +Fs/2] sau fftshift */
  float     fft_window[DSP_FFT_SIZE];
  uint16_t  fft_fill;
  bool      fft_ready;
  uint8_t   wf_lines;   /* pending waterfall lines since last display read */

  /* Spectrum decimation – 3-stage ÷2 cascade, tapped after Noise Blanker */
  FIR_Filter_t spec_dec_i[3];    /*!< Half-band anti-alias FIR I, stages 0-2 */
  FIR_Filter_t spec_dec_q[3];    /*!< Half-band anti-alias FIR Q, stages 0-2 */
  uint8_t      spec_decim;        /*!< Active factor: 1, 2, 4, or 8            */
  uint8_t      spec_decim_cnt[3]; /*!< Subsample counter per stage             */
  uint8_t      spec_decim_flush;  /*!< FFT frames to discard after zoom change */

  /* Trạng thái */
  SDR_Mode_t mode;
  float      signal_power_db;
  uint32_t   sample_count;

  /* Squelch */
  float      squelch_threshold_db;  /* -200 = disabled */
  bool       squelch_open;          /* current gate state */

  /* Software volume scale: 0.0 (silent) .. 1.0 (full). Applied to audio
   * samples before DAC output so volume works regardless of codec path. */
  float      rx_volume_scale;

  /* Notch filter (audio-domain, post-demod, pre-AGC) */
  IIR_Biquad_t notch;
  bool         notch_on;
  float        notch_hz;

  /* CW BFO – RX demodulator */
  uint32_t      cw_phase_acc;    /*!< RX CW BFO phase accumulator                    */
  uint32_t      cw_bfo_inc;      /*!< RX CW BFO phase increment (sample-rate-derived) */
  bool          cw_reverse;      /*!< Negate Q before CW demod (spectrum mirror)      */
  volatile bool cw_key_out;      /*!< Written by keyer (main loop), read by DSP ISR   */

  /* CW keying envelope – pre-AGC tap for decoder */
  CWEnv_t    cw_env;

  /* RX phasing SSB demodulator – Hilbert FIR on Q + matched I delay.
   * Hilbert FIR shifts Q by 90° so that (I ± H{Q})*0.5 gives perfect
   * single-sideband extraction.  I is delayed by HILBERT_DELAY = 31 samples
   * to compensate the FIR group delay.  Only active in USB/LSB modes. */
  Hilbert_t  rx_hilbert;
  float      rx_i_delay[HILBERT_DELAY + 1U];  /* ring buffer, size 32 */
  uint16_t   rx_delay_idx;

  /* TX state */
  TX_State_t tx;

  /* TX MIC input: set to s_rx_buf before DSP_ProcessTX when tx_src==MIC.
   * NULL = use USB audio ring (default). */
  const int32_t *mic_buf;
} DSP_State_t;

/* Exported functions prototypes ---------------------------------------------*/
void DSP_Init(DSP_State_t *dsp, uint32_t sample_rate);
void DSP_SetSquelch(DSP_State_t *dsp, uint8_t sq);
void DSP_SetFrequency(DSP_State_t *dsp, uint32_t lo_offset_hz, uint32_t sample_rate);
void DSP_SetIFShift(DSP_State_t *dsp, int32_t if_shift_hz, uint32_t sample_rate);
void DSP_SetMode(DSP_State_t *dsp, SDR_Mode_t mode, uint32_t sample_rate);
void DSP_SetBW(DSP_State_t *dsp, float bw_hz);
void DSP_SetCWPitch(DSP_State_t *dsp, uint16_t pitch_hz, uint32_t sample_rate);
void DSP_SetCWReverse(DSP_State_t *dsp, bool reverse);
void DSP_SetSidetoneVol(DSP_State_t *dsp, uint8_t vol_pct);
void DSP_Process(DSP_State_t *dsp,
                  const int32_t *iq_in,
                  int32_t       *audio_out,
                  uint32_t       len);

/**
 * @brief  TX DSP pipeline: audio source → SSB modulate → I/Q → DAC output.
 *         Audio source = USB Audio OUT ring (PC tới radio).
 *         Output = IQ int32 LSB-aligned tới SAI TX DMA (WM8731 → QSE mixer).
 *         Also feeds FFT for display spectrum during TX.
 * @param  dsp       DSP state
 * @param  iq_out    Output buffer (int32 interleaved [I0,Q0, I1,Q1, ...])
 * @param  len       Số cặp I/Q cần tạo
 */
void DSP_ProcessTX(DSP_State_t *dsp, int32_t *iq_out, uint32_t len);

/* Hilbert FIR 90° phase shift */
void  Hilbert_Init(Hilbert_t *h);
float Hilbert_Process(Hilbert_t *h, float x);

/* NCO */
void  NCO_SetFrequency(NCO_t *nco, int32_t freq_hz, uint32_t sample_rate);
void  NCO_Step(NCO_t *nco);

/* FIR */
void  FIR_Init_LPF(FIR_Filter_t *fir, float cutoff_norm, uint16_t taps);
float FIR_Process(FIR_Filter_t *fir, float x);

/* IIR */
void  IIR_DCBlock_Init(IIR_Biquad_t *f);
float IIR_DCBlock_Process(IIR_Biquad_t *f, float x);
void  IIR_HP1_Init(IIR_Biquad_t *f, float fc_hz, uint32_t sample_rate);
float IIR_Biquad_Process(IIR_Biquad_t *f, float x);
void  Notch_Init(IIR_Biquad_t *f, float fc_hz, uint32_t sample_rate);

/* AGC */
void  AGC_Init(AGC_t *agc, uint32_t sample_rate);
/* speed: 0=SLOW, 1=FAST, 2=AUTO */
void  AGC_SetMode(AGC_t *agc, SDR_Mode_t mode, uint8_t speed, uint32_t sample_rate);
void  AGC_SetSpeed(AGC_t *agc, uint8_t speed, uint32_t sample_rate);
float AGC_Process(AGC_t *agc, float x);

/* TX passband */
void  DSP_SetTxPassband(DSP_State_t *dsp, float hp_hz, float lp_hz);

/* Notch filter */
void  DSP_SetNotch(DSP_State_t *dsp, bool on, float hz);

/* Noise Blanker */
void  DSP_NB_Set(DSP_State_t *dsp, bool enabled, uint8_t level);
void  DSP_NR_Set(DSP_State_t *dsp, bool enabled, uint8_t level);

/* IQ correction */
void  DSP_SetIQCorr(DSP_State_t *dsp, int16_t gain_millis, int16_t phase_mrad);

/* Static DC offset (call after loading cal from flash) */
void  DSP_SetDCOffset(DSP_State_t *dsp, int32_t dc_i, int32_t dc_q);

/* Cal measurement: arm accumulator, poll for completion */
void  DSP_CalStart(DSP_State_t *dsp, DSP_CalMode_t mode, uint32_t n_samples);
bool  DSP_CalPoll(DSP_State_t *dsp, DSP_CalMeas_t *out);

/* FFT */
void  FFT_Hann_Window(float *w, uint16_t n);
/* Spectrum decimation */
void  DSP_SetSpecDecim(DSP_State_t *dsp, uint8_t factor);
void  FFT_Radix2(Complex_f *buf, uint16_t n);
void  FFT_ComputeMag_dB(const Complex_f *buf, float *mag_db,
                         uint16_t half_n, float *peak_db);

/* Demodulators */
float Demod_AM(float i, float q);
float Demod_FM(FM_Demod_t *fm, float i, float q);
float Demod_USB(float i, float q);
float Demod_LSB(float i, float q);
float Demod_CW(float i, float q, uint32_t *phase_acc, uint32_t phase_inc, bool reverse);

#ifdef __cplusplus
}
#endif
#endif /* __SDR_DSP_H */
