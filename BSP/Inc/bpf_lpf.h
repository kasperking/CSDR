/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bpf_lpf.h
  * @brief   Band-Pass Filter (FST3253) + Low-Pass Filter (74HC238) Driver
  *
  *  ── BPF (FST3253 Analog Multiplexer) ────────────────────
  *  PA4 BPF_S1  – relay select bit 0 (S0)
  *  PA5 BPF_S2  – relay select bit 1 (S1)
  *  PA6 BPF_OE1 – active-HIGH enable, TX relay bank (2B1..2B4)
  *  PA7 BPF_OE2 – active-HIGH enable, RX relay bank (1B1..1B4)
  *  OE1 and OE2 are ALWAYS complementary — never both HIGH.
  *
  *  S1:S0 = 00 → filter 0: 20/30m
  *  S1:S0 = 01 → filter 1: 40m
  *  S1:S0 = 10 → filter 2: 15-10m
  *  S1:S0 = 11 → filter 3: 80m
  *
  *  ── LPF (74HC238 3-to-8 Decoder) ────────────────────────
  *  PA0 LPF_A0, PA1 LPF_A1, PA2 LPF_A2 – decoder address (A2:A1:A0)
  *  Decoder enable pins E1/E2/E3 are hardwired on PCB (always enabled).
  *  Enum value = decoder address: (uint8_t)lpf_band_t → A2:A1:A0 directly.
  *
  *  A2:A1:A0 = 000 → Y0  LPF_1M8    — 1.8 MHz  (160m TX)
  *  A2:A1:A0 = 001 → Y1  LPF_3M5    — 3.5 MHz  (80m TX)
  *  A2:A1:A0 = 010 → Y2  LPF_5M8    — 5.8 MHz  (60m TX)
  *  A2:A1:A0 = 011 → Y3  LPF_8_17M  — 8-17 MHz (40m/30m/20m TX)
  *  A2:A1:A0 = 100 → Y4  LPF_17_32M — 17-32 MHz(17m..6m TX)
  *  A2:A1:A0 = 101 → Y5  (unused, unconnected)
  *  A2:A1:A0 = 110 → Y6  (unused, unconnected)
  *  A2:A1:A0 = 111 → Y7  LPF_OFF    — all relay coils released
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

/* BPF filter select channel (S1:S0) — matches truth table exactly.
 * ch = (S1 << 1) | S0; written to BPF_S2_Pin (bit1) / BPF_S1_Pin (bit0). */
#define BPF_CH_20_30M   0U  /* S1=0 S0=0 */
#define BPF_CH_40M      1U  /* S1=0 S0=1 */
#define BPF_CH_15_10M   2U  /* S1=1 S0=0 */
#define BPF_CH_80M      3U  /* S1=1 S0=1 */

/* ── RF mode / filter enums ─────────────────────────────── */

typedef enum {
  BPF_20_30M  = 0U,   /* ch0: S1=0 S0=0 — 14/10 MHz  */
  BPF_40M     = 1U,   /* ch1: S1=0 S0=1 — 7 MHz       */
  BPF_15_10M  = 2U,   /* ch2: S1=1 S0=0 — 21-28 MHz   */
  BPF_80M     = 3U,   /* ch3: S1=1 S0=1 — 3.5 MHz     */
} bpf_filter_t;

typedef enum {
  RF_MODE_RX = 0U,    /* OE1=0, OE2=1 (RX relay bank active)  */
  RF_MODE_TX = 1U,    /* OE1=1, OE2=0 (TX relay bank active)  */
} rf_mode_t;

/* ── LPF decoder band enum ──────────────────────────────────
 * Enum value = 74HC238 address (A2:A1:A0) written to MCU GPIO.
 * Cast to uint8_t to get the 3-bit decoder address directly.
 * Y5 and Y6 are unused/unconnected on the PCB.
 * LPF_OFF selects Y7 (unconnected) — all relay coils released.  */
typedef enum {
  LPF_1M8    = 0U,   /* A2:A1:A0=000 → Y0 — 1.8 MHz  (160m) */
  LPF_3M5    = 1U,   /* A2:A1:A0=001 → Y1 — 3.5 MHz  (80m)  */
  LPF_5M8    = 2U,   /* A2:A1:A0=010 → Y2 — 5.8 MHz  (60m)  */
  LPF_8_17M  = 3U,   /* A2:A1:A0=011 → Y3 — 8-17 MHz (40m/30m/20m) */
  LPF_17_32M = 4U,   /* A2:A1:A0=100 → Y4 — 17-32 MHz(17m..6m)     */
  LPF_OFF    = 7U,   /* A2:A1:A0=111 → Y7 — all relays released     */
} lpf_band_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialise BPF and LPF GPIO.
  *         Default: RF_MODE_RX, BPF_20_30M filter.
  *         OE1=0 (TX bank off), OE2=1 (RX bank on), T_R_SW=LOW.
  */
void BPF_LPF_Init(void);

/**
  * @brief  Central BPF relay control — the ONLY place OE1/OE2 are written.
  *
  *  Glitch-free sequence:
  *   1. Disable both OEs (both relay banks released).
  *   2. HAL_Delay(2 ms) — relay release time.
  *   3. Set S1:S0 select bits for requested filter.
  *   4. Assert OE1 (TX) or OE2 (RX) — never both.
  *
  *  Truth table enforced:
  *   TX: OE1=1, OE2=0 — engages TX relay bank (2B1..2B4).
  *   RX: OE1=0, OE2=1 — engages RX relay bank (1B1..1B4).
  *
  * @param  mode    RF_MODE_TX or RF_MODE_RX
  * @param  filter  BPF_20_30M / BPF_40M / BPF_15_10M / BPF_80M
  */
void BPF_Set(rf_mode_t mode, bpf_filter_t filter);

/**
  * @brief  Switch RF mode, keep current filter.
  *         Calls BPF_Set() with the cached filter value.
  */
void BPF_SetMode(rf_mode_t mode);

/**
  * @brief  Select filter for current RF mode from band index.
  *         Maps band_idx → bpf_filter_t, then calls BPF_Set().
  * @param  band_idx  0..BAND_COUNT-1
  */
void BPF_SetBand(uint8_t band_idx);

/**
  * @brief  Central LPF relay control — the ONLY place A0/A1/A2 are written.
  *
  *  Glitch-free sequence (encoder enable pins hardwired, always active):
  *   1. Write A2:A1:A0 = 111 → Y7 (LPF_OFF, unconnected) — all coils released.
  *   2. HAL_Delay(1 ms) — relay armature release time.
  *   3. Write target address → energise exactly one relay coil (or stay at Y7).
  *
  *  Going via Y7 prevents multi-bit address transitions from briefly
  *  energising an intermediate relay during GPIO bit updates.
  *
  *  No-op if the requested band is already active (cached state).
  *  Called from main-loop context only — HAL_Delay() is safe here.
  *
  * @param  band  lpf_band_t value; enum value == decoder A2:A1:A0 address
  */
void LPF_Set(lpf_band_t band);

/**
  * @brief  Select LPF from band index.
  *         Maps band_idx → lpf_band_t, then calls LPF_Set().
  * @param  band_idx  0..BAND_COUNT-1
  */
void LPF_SetBand(uint8_t band_idx);

/**
  * @brief  Select BPF + LPF from frequency.
  * @param  freq_hz  Operating frequency in Hz
  */
void BPF_LPF_SetFrequency(uint32_t freq_hz);

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
