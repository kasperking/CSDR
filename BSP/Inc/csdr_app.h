/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    csdr_app.h
  * @brief   CSDR Application Layer
  *
  *  All GPIO pin definitions come from Core/Inc/main.h (CubeMX generated).
  *  This file adds only application-level types, constants, and API.
  *
  *  Usage in main.c (CubeMX generated):
  *  // USER CODE BEGIN Includes
  *  #include "csdr_app.h"
  *  // USER CODE END Includes
  *
  *  // USER CODE BEGIN 2
  *  CSDR_Init();
  *  // USER CODE END 2
  *
  *  // USER CODE BEGIN 3 (inside while(1))
  *  CSDR_Loop();
  *  // USER CODE END 3
  *
  *  In stm32h7xx_it.c – add to SysTick_Handler:
  *  // USER CODE BEGIN SysTick_IRQn 1
  *  CSDR_SysTickCallback();
  *  // USER CODE END SysTick_IRQn 1
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CSDR_APP_H
#define __CSDR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "main.h"          /* authoritative GPIO pin defines (CubeMX generated) */
#include "w25q.h"          /* BandCal_t, BAND_COUNT */
#include <stdint.h>
#include <stdbool.h>

/* Default LO offset used to initialise g_sdr.lo_offset_hz at boot. */
#define LO_OFFSET_DEFAULT  0U

/* AM RX low-IF: the LO parks AM_LOW_IF_HZ above the tuned frequency and the
 * RX NCO mixes it back to baseband, so the AM carrier never sits in the
 * analog DC notch (AC-coupled QSD→codec path + WM8731 ADC HPF) that destroys
 * envelope demodulation at zero-IF.  TX always runs LO = carrier — the TX IQ
 * path has no offset compensation (see the LO hop in csdr_apply_tx). */
#define AM_LOW_IF_HZ  12000U

/* ── SDR State ───────────────────────────────────────────── */
typedef enum {
  MODE_AM   = 0,
  MODE_FM   = 1,
  MODE_USB  = 2,   /* voice USB SSB          */
  MODE_LSB  = 3,   /* voice LSB SSB          */
  MODE_CW   = 4,
  MODE_DIGU   = 5,   /* digital USB (WSJT-X/FT8/DATA-USB) — linear TX path */
  MODE_DIGL   = 6,   /* digital LSB (DATA-LSB)             — linear TX path */
  MODE_COUNT  = 7
} SDR_Mode_t;

typedef enum {
  STEP_1=1, STEP_10=10, STEP_100=100,
  STEP_1K=1000, STEP_10K=10000, STEP_100K=100000
} FreqStep_t;

typedef struct {
  uint32_t   freq_hz;
  SDR_Mode_t mode;
  uint8_t    band_idx;
  FreqStep_t step;
  int16_t    rit_hz;
  uint32_t   bw_hz;       /*!< SH high-cut edge in Hz (= DSP LPF cutoff)        */
  uint32_t   sl_hz;       /*!< SL low-cut edge in Hz (= DSP IF shift magnitude)  */
  int16_t    if_shift_hz; /*!< IS: IF shift Hz — per-VFO, survives VFO swap      */
} VFO_State_t;

typedef struct {
  uint32_t    freq_hz;
  SDR_Mode_t  mode;
  uint8_t     band_idx;
  uint8_t     volume;
  uint8_t     squelch;
  FreqStep_t  step;
  bool        mute;
  uint8_t     agc_speed;   /*!< 0=SLOW 1=FAST 2=AUTO */
  bool        nb_on;
  uint8_t     nb_level;  /*!< NB intensity 0-100, default 50; RX menu → "NB Level" */
  uint8_t     nr_mode;   /*!< 0=off 1=NR1 (LMS) 2=NR2 (spectral); CAT NR0/1/2 */
  uint8_t     nr_level;  /*!< NR strength 0-100 (NR1 wet / NR2 depth); RX menu → "NR Level" */
  uint8_t     bc_mode;   /*!< Beat canceller: 0=off 1/2=on; CAT BC0/1/2 */
  int16_t     rit_hz;
  bool        tx_mode;
  bool        tune_mode;     /*!< TUNE button held: fixed low-power carrier, PA_Protect
                                   ignores SWR warn/trip (see pa_protect.c)             */
  bool        si5351_ok;
  uint8_t     att_db;
  bool        pwr_hold;
  uint32_t    bw_hz;       /*!< SH high-cut edge in Hz (= DSP LPF cutoff)        */
  uint32_t    sl_hz;       /*!< SL low-cut edge in Hz (= DSP IF shift magnitude)  */
  int16_t     if_shift_hz;
  /* Spectrum marker (Track mode): freq_hz stays the listening frequency;
   * LO center = freq_hz - marker_offset_hz.  Fix mode keeps offset at 0. */
  bool        marker_track;      /*!< false = Fix (marker at center), true = Track  */
  int32_t     marker_offset_hz;  /*!< demod offset from LO center; not persisted    */
  uint8_t     display_dirty;
  uint8_t     usb_mode;
  bool        usb_iq_stream;  /*!< true = raw IQ to USB audio; false = demodulated audio */
  /* Calibration */
  int32_t     xtal_ppb;   /*!< SI5351 XTAL correction ppb (GPS cal ghi trực tiếp) */
  int16_t     iq_gain;
  int16_t     iq_phase;
  int32_t     dc_i_offset;
  int32_t     dc_q_offset;
  int16_t     audio_gain_db;
  int16_t     mic_gain;   /*!< Voice TX drive 0-100 (default 50)            */
  int16_t     digi_gain;  /*!< DIGU/DIGL TX drive 0-100 (default 70)        */
  uint8_t     tx_power;         /*!< TX output power 0-100% (default 100); PC cmd */
  uint8_t     pa_watts;         /*!< PA hardware rating: 0=None, 20, 45, 100 W    */
  uint8_t     pa_oc_limit_idx;  /*!< OC limit ×10 A — 10..200 = 1.0..20.0 A, step 0.1 A */
  uint8_t     pwr_scale;        /*!< Tandem-match FWD power cal 50..200 % (default 100) */
  int16_t     smeter_offset_db;
  uint32_t    lo_offset_hz;
  /* Dual VFO */
  VFO_State_t vfo_b;
  uint8_t     active_vfo;
  /* CAT deferred-hardware flags — set by CAT handlers, cleared by CSDR_Loop */
  bool        cat_freq_dirty; /* FA SET: apply SI5351 + DSP NCO outside CAT context */
  bool        cat_vol_dirty;  /* AG SET: apply WM8731 volume outside CAT context    */
  bool        cat_mode_dirty; /* MD SET: apply DSP mode/BW outside CAT context      */
  bool        cat_tx_dirty;   /* TX/RX: apply T/R relay + codec outside CAT context */
  bool        cat_att_dirty;  /* RA SET: apply PE4302 attenuator outside CAT context */
  bool        cat_rit_dirty;  /* RT/RC/RU/RD/IS: recompute nco_if = if_shift_hz + (rit_on ? rit_hz : 0) */
  /* RF front-end AGC (PE4302) */
  bool        rf_agc_on;     /*!< Automatic PE4302 RF attenuator control (overload prevention) */
  /* External PA ALC feedback (PC1 / ADC2_INP11) */
  bool        ext_alc_on;   /*!< Enable external ALC drive reduction from PA feedback voltage  */
  /* External PA (keyed from T_R_SW via optocoupler) */
  bool        ext_pa_on;        /*!< Ext PA fitted: arms TX keying gate + drive cap            */
  uint8_t     ext_pa_delay_ms;  /*!< RF hold-off after T/R asserts, 0-50 ms (amp relay settle) */
  uint8_t     ext_pa_max_drive; /*!< tx_power cap % while ext_pa_on (amp input protection), 5-100 */
  /* PA gate bias (pa_bias.h — fixed trimmer or MCP4822 DAC on SPI3) */
  uint8_t     pa_bias_src;      /*!< 0=FIXED (trimmer), 1=DAC (MCP4822)       */
  uint8_t     pa_bias1;         /*!< DAC ch A (final)  0-200 = 0-100% FS      */
  uint8_t     pa_bias2;         /*!< DAC ch B (driver) 0-200 = 0-100% FS      */
  uint16_t    pa_idq_ma;        /*!< Idq đích cho auto-cal Bias 1: 50-2000 mA */
  /* TX audio passband */
  uint16_t    tx_audio_low_hz;   /*!< TX Low-cut (HPF) Hz: 100-500  */
  uint16_t    tx_audio_high_hz;  /*!< TX High-cut (LPF) Hz: 2200-3500 */
  /* Notch filter */
  bool        notch_on;          /*!< Audio notch filter enable */
  int16_t     notch_hz;          /*!< Notch center frequency Hz: 100-4000 */
  /* RX tone control */
  int8_t      bass_db;           /*!< Bass shelf gain dB: -10..+10, default 0 (flat) */
  int8_t      treble_db;         /*!< Treble shelf gain dB: -10..+10, default 0 (flat) */
  /* VOX */
  bool        vox_on;            /*!< VOX enable */
  uint8_t     vox_gain;          /*!< VOX sensitivity 0-100 (100=most sensitive) */
  uint16_t    vox_delay;         /*!< VOX hang time ms: 100-2000 */
  /* CW decode */
  bool        cw_decode_on;      /*!< CW decoder active (RX, CW mode only)         */
  /* RTTY decode */
  bool        rtty_decode_on;    /*!< RTTY decoder active (RX, USB/LSB/DIGU/DIGL)  */
  uint8_t     rtty_baud_idx;     /*!< g_rtty_baud_x100 index: 0=45.45 1=50 2=75 Bd */
  uint8_t     rtty_shift_idx;    /*!< g_rtty_shift_hz index: 0=170 1=425 2=850 Hz  */
  /* FT8 station (decode runs only inside the full-screen FT8 app) */
  char        ft8_call[12];      /*!< Station callsign for FT8 TX (empty = unset)  */
  char        ft8_grid[5];       /*!< 4-char Maidenhead grid for FT8 TX            */
  /* CW keyer / TX */
  uint16_t    cw_pitch_hz;       /*!< BFO / sidetone pitch Hz: 300-900, default 700 */
  uint8_t     cw_wpm;            /*!< Keyer speed WPM: 5-40, default 20             */
  uint8_t     keyer_mode;        /*!< 0=Straight 1=Iambic-A 2=Iambic-B             */
  bool        paddle_reverse;    /*!< Swap DIT/DAH paddles                          */
  uint8_t     sidetone_vol;      /*!< Sidetone volume 0-100%, default 50            */
  uint8_t     cw_bkin;           /*!< 0=Off 1=Semi 2=Full break-in                  */
  uint16_t    cw_bk_delay_ms;    /*!< BK-IN hang delay ms: 50-2000, default 200     */
  bool        cw_reverse;        /*!< CW reverse sideband selection                 */
  uint16_t    cw_filter_hz;      /*!< CW filter bandwidth Hz: 50-500, default 500   */
  uint8_t     tx_src;            /*!< TX audio source: 0=USB 1=MIC (hand mic)       */
} SDR_State_t;

extern SDR_State_t  g_sdr;
extern BandCal_t    g_band_cal[BAND_COUNT];   /* per-band cal; loaded from flash at boot */

/* ── Application constants ───────────────────────────────── */
#define CSDR_FREQ_MIN_HZ       100000UL
#define CSDR_FREQ_MAX_HZ     30000000UL
#define CSDR_FREQ_DEFAULT_HZ   7100000UL

#define CSDR_AUDIO_SAMPLE_RATE  48000UL
#define CSDR_AUDIO_BLOCK_SIZE     256U
#define CSDR_AUDIO_BUF_TOTAL   (CSDR_AUDIO_BLOCK_SIZE * 2U)

/* I2C addresses */
#define WM8731_I2C_ADDR     (0x1AU << 1U)
#define SI5351_I2C_ADDR     (0x60U << 1U)
#define SI5351_XTAL_HZ      25000000UL

/* LCD geometry: defined in lcd_render.h (480×320), do not redefine here. */

/* NTC / ADC */
#define ADC_VREF_MV       3300U
#define ADC_FULL_SCALE    65536U
#define VOLT_DIV_RATIO    4U
#define NTC_R_SERIES_OHM  10000U
#define NTC_R_25C_OHM     10000U
#define NTC_BETA          3950U
#define FAN_TEMP_START_C  40U
#define FAN_TEMP_FULL_C   65U
#define FAN_PWM_MIN       200U
#define FAN_PWM_MAX       999U
#define SWR_WARN_THRESH   200U
#define POWER_OFF_HOLD_MS 3000U

/* ── API ─────────────────────────────────────────────────── */

void CSDR_Init(void);

int32_t *CSDR_GetTxBuf(void);
int32_t *CSDR_GetRxBuf(void);
void     CSDR_ClearDspFlags(void);

/* Drain pending SAI DMA halves into the USB ring.
 * ONLY call from top-level CSDR_Loop context.
 * DO NOT call from inside LCD/FMC strip loops — causes USB ring throttle
 * runaway due to BASEPRI/DMA TC ISR interaction (confirmed by testing). */
void CSDR_ProcessAudioPending(void);

void CSDR_Loop(void);

/* Immediate TX/RX transition for full-screen apps (ft8_app beacon).
 * Call ONLY from CSDR_Loop-equivalent context — runs the full apply chain
 * (relays, codec, SI5351, PA_Protect OnTxStart/Stop). */
void CSDR_RequestTX(bool tx);

/* Persist g_sdr to flash (same serialiser as the menu path).  For apps that
 * edit persisted fields outside the menu (ft8_app station setup). */
void CSDR_SaveSettings(void);

/* T/R sequencing poll (ext-PA keying gate, TX→RX drain, deferred gain
 * reapply) — call every few ms from any app loop that drives TX via
 * CSDR_RequestTX while CSDR_Loop is not running. */
void CSDR_PollTxSequencing(void);

/* Effective RX LO offset: user cal offset (g_sdr.lo_offset_hz) plus the AM
 * low-IF when mode == MODE_AM.  Use for every RX-side SI5351 retune and every
 * DSP_SetFrequency call; TX LO programming uses the plain cal offset. */
uint32_t CSDR_RxLoOffset(void);

void CSDR_SysTickCallback(void);

void CSDR_CDC_Receive(uint8_t *buf, uint32_t len);
void CSDR_CDC_ResetCAT(void);

void CSDR_PrepareShutdown(void);   /*!< Save settings + power-off screen, then cut power */
void CSDR_VerifyIRQConfig(void);   /*!< IRQ priority sanity-check; call after MX_USB_DEVICE_Init() */

#ifdef __cplusplus
}
#endif
#endif /* __CSDR_APP_H */
