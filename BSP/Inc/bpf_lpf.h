/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bpf_lpf.h
  * @brief   Band-Pass Filter (FST3253) + Low-Pass Filter (74HC238) Driver
  *
  *  ── BPF (FST3253 Analog Multiplexer) ────────────────────
  *  PD11 BPF_OE  – Output Enable (active low) – read-back as Input
  *  PD12 BPF_S0  – Select bit 0
  *  PD13 BPF_S1  – Select bit 1
  *
  *  FST3253: 2× dual 4:1 mux. Dùng 2 chân select → 4 vị trí:
  *   S1:S0 = 00 → BPF channel 0: 160m/80m  (1.8-4.0 MHz)
  *   S1:S0 = 01 → BPF channel 1: 40m/30m   (4.0-11.0 MHz)
  *   S1:S0 = 10 → BPF channel 2: 20m/15m   (11.0-22.0 MHz)
  *   S1:S0 = 11 → BPF channel 3: 10m/6m    (22.0-55.0 MHz)
  *
  *  ── LPF (74HC238 3-to-8 Decoder) ────────────────────────
  *  LPF_A0, LPF_A1, LPF_A2 – 3-bit address → chọn 1 trong 8 LPF TX
  *
  *  74HC238 outputs (Y0-Y7): chọn cuộn lọc thấp TX theo band
  *   A2:A1:A0 = 000 → LPF TX 160m  (cutoff 2.5 MHz)
  *   A2:A1:A0 = 001 → LPF TX 80m   (cutoff 5.0 MHz)
  *   A2:A1:A0 = 010 → LPF TX 40m   (cutoff 9.0 MHz)
  *   A2:A1:A0 = 011 → LPF TX 20m   (cutoff 16.0 MHz)
  *   A2:A1:A0 = 100 → LPF TX 15m   (cutoff 22.0 MHz)
  *   A2:A1:A0 = 101 → LPF TX 10m   (cutoff 35.0 MHz)
  *   A2:A1:A0 = 110 → LPF TX 6m    (cutoff 60.0 MHz)
  *   A2:A1:A0 = 111 → Không dùng / All off
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BPF_LPF_H
#define __BPF_LPF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* Band indices (khớp với SDR_State_t.band_idx) */
#define BAND_160M    0U
#define BAND_80M     1U
#define BAND_60M     2U
#define BAND_40M     3U
#define BAND_30M     4U
#define BAND_20M     5U
#define BAND_17M     6U
#define BAND_15M     7U
#define BAND_12M     8U
#define BAND_10M     9U
#define BAND_6M      10U
#define BAND_COUNT   11U

/* Band frequency edges (Hz) */
static const uint32_t BAND_FREQ_MIN[BAND_COUNT] = {
  1800000UL,  3500000UL,  5330000UL,  7000000UL, 10100000UL,
 14000000UL, 18068000UL, 21000000UL, 24890000UL, 28000000UL, 50000000UL
};
static const uint32_t BAND_FREQ_MAX[BAND_COUNT] = {
  2000000UL,  4000000UL,  5410000UL,  7300000UL, 10150000UL,
 14350000UL, 18168000UL, 21450000UL, 24990000UL, 29700000UL, 54000000UL
};

/* BPF channel mapping (S1:S0) per band */
#define BPF_CH_160_80M   0U  /* 1.8-4.0 MHz  */
#define BPF_CH_40_30M    1U  /* 4.0-11.0 MHz */
#define BPF_CH_20_15M    2U  /* 11-22 MHz    */
#define BPF_CH_10_6M     3U  /* 22-55 MHz    */

/* LPF channel mapping (A2:A1:A0) per band */
#define LPF_CH_160M  0U
#define LPF_CH_80M   1U
#define LPF_CH_40M   2U
#define LPF_CH_20M   3U
#define LPF_CH_15M   4U
#define LPF_CH_10M   5U
#define LPF_CH_6M    6U
#define LPF_CH_NONE  7U

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Khởi tạo BPF và LPF GPIO.
  *         BPF_OE = HIGH (disabled khi init).
  *         LPF_A* = 111 (none).
  */
void BPF_LPF_Init(void);

/**
  * @brief  Chọn BPF channel theo band index.
  *         Tự động ánh xạ band → S1:S0 của FST3253.
  * @param  band_idx  0..BAND_COUNT-1
  */
void BPF_SetBand(uint8_t band_idx);

/**
  * @brief  Chọn LPF TX channel theo band index.
  *         Tự động ánh xạ band → A2:A1:A0 của 74HC238.
  * @param  band_idx  0..BAND_COUNT-1
  */
void LPF_SetBand(uint8_t band_idx);

/**
  * @brief  Tự động chọn BPF + LPF dựa vào tần số.
  * @param  freq_hz  Tần số thu/phát (Hz)
  */
void BPF_LPF_SetFrequency(uint32_t freq_hz);

/**
  * @brief  Enable / disable BPF output (BPF_OE, active low).
  *         Nếu TX mode: thường disable BPF, enable LPF path.
  */
void BPF_Enable(bool enable);

/**
  * @brief  Lấy band index từ tần số.
  * @retval 0xFF nếu không thuộc band nào
  */
uint8_t BPF_FreqToBand(uint32_t freq_hz);

/**
  * @brief  Ánh xạ band index → tần số mặc định của band đó.
  */
uint32_t BPF_BandToFreq(uint8_t band_idx);

/**
  * @brief  Band lên/xuống với wrap-around.
  */
uint8_t BPF_BandUp(uint8_t current);
uint8_t BPF_BandDown(uint8_t current);

#ifdef __cplusplus
}
#endif
#endif /* __BPF_LPF_H */
