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
#include <stdint.h>
#include <stdbool.h>

/* Default LO offset used to initialise g_sdr.lo_offset_hz at boot. */
#define LO_OFFSET_DEFAULT  00000U

/* ── SDR State ───────────────────────────────────────────── */
typedef enum {
  MODE_AM = 0, MODE_FM, MODE_USB, MODE_LSB, MODE_CW, MODE_COUNT
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
  uint32_t   bw_hz;
} VFO_State_t;

typedef struct {
  uint32_t    freq_hz;
  SDR_Mode_t  mode;
  uint8_t     band_idx;
  uint8_t     volume;
  uint8_t     squelch;
  FreqStep_t  step;
  bool        mute;
  bool        agc_fast;
  bool        nb_on;
  bool        nr_on;
  int16_t     rit_hz;
  bool        tx_mode;
  bool        si5351_ok;
  uint8_t     att_db;
  bool        pwr_hold;
  uint32_t    bw_hz;
  int16_t     if_shift_hz;
  uint8_t     display_dirty;
  uint8_t     usb_mode;
  /* Calibration */
  int32_t     xtal_ppm;
  int16_t     iq_gain;
  int16_t     iq_phase;
  int32_t     dc_i_offset;
  int32_t     dc_q_offset;
  int16_t     audio_gain_db;
  int16_t     mic_gain;
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
} SDR_State_t;

extern SDR_State_t g_sdr;

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

/* LCD (FMC parallel, ST7796S) */
#define LCD_W  320U
#define LCD_H  240U

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

void CSDR_Loop(void);

void CSDR_SysTickCallback(void);

void CSDR_CDC_Receive(uint8_t *buf, uint32_t len);
void CSDR_CDC_ResetCAT(void);

#ifdef __cplusplus
}
#endif
#endif /* __CSDR_APP_H */
