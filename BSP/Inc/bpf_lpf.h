/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bpf_lpf.h
  * @brief   Band-Pass Filter (4× SN74CBT3253 via 74AHC595) + Low-Pass Filter
  *          (74HC238) Driver
  *
  *  ── BPF (4× SN74CBT3253 dual 4:1 FET mux, 2 banks of {QSD-side, ant-side}
  *         pair, driven by one 74AHC595 shift register — NOT direct GPIO) ──
  *  PA4 BPF_SRCLK – 595 shift clock
  *  PA5 BPF_RCLK  – 595 storage-register latch (pulse HIGH then LOW to commit)
  *  PA6 BPF_OE    – 595 chip output-enable, active-LOW. Tri-states Q0..Q7
  *                  (all mux OE pins float) when HIGH. Pull-ups on the 3253
  *                  OE nets hold every mux side OFF while floating, so this
  *                  pin must be driven HIGH by the time the MCU takes over
  *                  and must stay HIGH until the first valid word has been
  *                  shifted + latched — see BPF_LPF_Init().
  *  PA7 BPF_SER   – 595 serial data in
  *
  *  595 output bit → function (see bpf_lpf.c bpf595_shift() / BPF595_BIT_*):
  *    QA (bit0) = S0   — filter channel select bit 0, shared by all 4 chips
  *    QB (bit1) = S1   — filter channel select bit 1, shared by all 4 chips
  *    QC (bit2) = OE_1.1 — bank A (chip pair 1) RX side enable, active-LOW
  *    QD (bit3) = OE_1.2 — bank A (chip pair 1) TX side enable, active-LOW
  *    QE (bit4) = OE_2.1 — bank B (chip pair 2) RX side enable, active-LOW
  *    QF (bit5) = OE_2.2 — bank B (chip pair 2) TX side enable, active-LOW
  *    QG, QH    = unused (reserved for future expansion)
  *  Exactly one of OE_1.1/OE_1.2/OE_2.1/OE_2.2 may be LOW at a time — never
  *  two at once (would join TX/RX paths, or join bank A/B, through the
  *  filters). BPF_Set() enforces this with a break-before-make shift+latch.
  *
  *  channel = (bank << 2) | (S1 << 1) | S0 — 7 of 8 populated, redesigned
  *  2026-08-05 to give each band its own (or near-own) filter:
  *   ch0  bank A S1=0 S0=0 — 1.5-2.5 MHz   (160m)
  *   ch1  bank A S1=0 S0=1 — 2.4-4.5 MHz   (80m)
  *   ch2  bank A S1=1 S0=0 — 4.6-7.5 MHz   (60m, 40m)
  *   ch3  bank A S1=1 S0=1 — 7.5-12.3 MHz  (30m)
  *   ch4  bank B S1=0 S0=0 — 11-14.8 MHz   (20m)
  *   ch5  bank B S1=0 S0=1 — 13.8-22.1 MHz (17m)
  *   ch6  bank B S1=1 S0=0 — 19-32 MHz     (15m, 12m, 10m)
  *   ch7  bank B S1=1 S0=1 — not populated (spare)
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
/* 6m removed: LO ×4 architecture caps CLK0 at 150 MHz → LO ≤ 37.5 MHz
 * (SI5351_CalcMSDiv ms_div ≥ 6).  EEPROM BandCalBlock_t keeps 11 slots
 * (BAND_CAL_SLOTS in w25q.h) so stored per-band cal stays valid. */
#define BAND_COUNT   10U

/* Band frequency edges (Hz) */
static const uint32_t BAND_FREQ_MIN[BAND_COUNT] = {
  1800000UL,  3500000UL,  5330000UL,  7000000UL, 10100000UL,
 14000000UL, 18068000UL, 21000000UL, 24890000UL, 28000000UL
};
static const uint32_t BAND_FREQ_MAX[BAND_COUNT] = {
  2000000UL,  4000000UL,  5410000UL,  7300000UL, 10150000UL,
 14350000UL, 18168000UL, 21450000UL, 24990000UL, 29700000UL
};

/* ── RF mode / filter enums ─────────────────────────────── */

/* bpf_filter_t value = (bank << 2) | (S1 << 1) | S0 — see header comment
 * above for the full ch0-ch7 frequency table. ch7 not populated. */
typedef enum {
  BPF_160M   = 0U,   /* bankA ch0: S1=0 S0=0 — 1.5-2.5 MHz   (160m) */
  BPF_80M    = 1U,   /* bankA ch1: S1=0 S0=1 — 2.4-4.5 MHz   (80m)  */
  BPF_40M    = 2U,   /* bankA ch2: S1=1 S0=0 — 4.6-7.5 MHz   (60m, 40m) */
  BPF_30M    = 3U,   /* bankA ch3: S1=1 S0=1 — 7.5-12.3 MHz  (30m)  */
  BPF_20M    = 4U,   /* bankB ch0: S1=0 S0=0 — 11-14.8 MHz   (20m)  */
  BPF_17M    = 5U,   /* bankB ch1: S1=0 S0=1 — 13.8-22.1 MHz (17m)  */
  BPF_15_10M = 6U,   /* bankB ch2: S1=1 S0=0 — 19-32 MHz (15m, 12m, 10m) */
} bpf_filter_t;

typedef enum {
  RF_MODE_RX = 0U,    /* enables the bank's side-1 (RX) OE, active-LOW */
  RF_MODE_TX = 1U,    /* enables the bank's side-2 (TX) OE, active-LOW */
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
  * @brief  Initialise BPF and LPF GPIO, and bring up the 595 shift chain.
  *         Default: RF_MODE_RX, BPF_160M filter.
  *         Shifts the RX/filter-0 word into the 595 storage register while
  *         BPF_OE is still HIGH (595 outputs Hi-Z), then pulls BPF_OE LOW —
  *         the 3253 mux sides never see anything but the intended word.
  *         T_R_SW=LOW.
  */
void BPF_LPF_Init(void);

/**
  * @brief  Central BPF mux control — the ONLY place the 595 word is built
  *         and shifted out.
  *
  *  Break-before-make sequence, done as two shift+latch cycles:
  *   1. Shift+latch a word with all 4 OE bits HIGH — every mux side off,
  *      filters isolated (bank A and bank B, RX and TX, all disabled).
  *   2. Shift+latch a word with S1:S0 set for the requested filter and
  *      exactly one OE bit LOW (the bank/side being requested).
  *  No settle delay needed between the two latches — CBT3253 FET switches
  *  in nanoseconds and the 595 storage register updates atomically on RCLK.
  *
  * @param  mode    RF_MODE_TX or RF_MODE_RX
  * @param  filter  BPF_160M / BPF_80M / BPF_40M / BPF_30M / BPF_20M /
  *                 BPF_17M / BPF_15_10M
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

/**
  * @brief  Return short band name string (e.g. "20m").
  * @retval Pointer to a static string; "??m" for out-of-range index.
  */
const char *BPF_BandName(uint8_t idx);

#ifdef __cplusplus
}
#endif
#endif /* __BPF_LPF_H */
