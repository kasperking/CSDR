/* USER CODE BEGIN Header */
/**
  * @file bpf_lpf.c
  * @brief SN74CBT3253 BPF mux (via 74AHC595 shift register) + 74HC238 LPF
  *        band switching
  */
/* USER CODE END Header */
#include "bpf_lpf.h"
#include "csdr_app.h"

/* USER CODE BEGIN 0 */
/* All pins from Core/Inc/main.h (CubeMX generated):
 *  BPF: BPF_SRCLK=PA4, BPF_RCLK=PA5, BPF_SER=PA7 drive a 74AHC595 whose 6
 *       used outputs fan out to 4× SN74CBT3253 (2 banks of QSD-side/ant-side
 *       chip pairs) — see bpf_lpf.h header comment for the full bit table.
 *       BPF_OE=PA6 is the 595's OWN output-enable (active-LOW, tri-states
 *       Q0..Q7), not a 3253 OE — see BPF_LPF_Init() for why it must stay
 *       HIGH until the first word is shifted+latched.
 *       Exactly one of the 4 mux-side OE bits may be LOW at a time (never
 *       two — would join TX/RX paths, or bank A/B, through the filters).
 *  LPF: LPF_A0=PA0, LPF_A1=PA1, LPF_A2=PA2 (74HC238 address)
 *  T/R: T_R_SW=PB2 (relay control) */

/* 74AHC595 output bit positions (QA=bit0 .. QF=bit5); see bpf_lpf.h. */
#define BPF595_BIT_S0     0U
#define BPF595_BIT_S1     1U
#define BPF595_BIT_OE1_1  2U  /* bank A (chip pair 1) RX enable, active-LOW */
#define BPF595_BIT_OE1_2  3U  /* bank A (chip pair 1) TX enable, active-LOW */
#define BPF595_BIT_OE2_1  4U  /* bank B (chip pair 2) RX enable, active-LOW */
#define BPF595_BIT_OE2_2  5U  /* bank B (chip pair 2) TX enable, active-LOW */

/* Break word: all 4 OE bits HIGH (every mux side off). S0/S1 don't matter
 * while every side is disabled, so they're left 0 here. */
#define BPF595_ALL_OFF    ((1U << BPF595_BIT_OE1_1) | (1U << BPF595_BIT_OE1_2) | \
                            (1U << BPF595_BIT_OE2_1) | (1U << BPF595_BIT_OE2_2))

/* Cached OE/filter state — source of truth for BPF_SetMode / BPF_SetBand */
static rf_mode_t   s_rf_mode   = RF_MODE_RX;
static bpf_filter_t s_bpf_filter = BPF_160M;

/* BPF filter per band — maps band_idx → bpf_filter_t (truth table).
 * Redesigned 2026-08-05: every band now has its own or a tightly-shared
 * filter (only 60m/40m and 15m/12m/10m still share) — see bpf_lpf.h header
 * comment for the ch0-ch6 frequency table. */
static const bpf_filter_t bpf_map[BAND_COUNT] = {
  BPF_160M,     /* 160m — 1.8 MHz  */
  BPF_80M,      /* 80m  — 3.5 MHz  */
  BPF_40M,      /* 60m  — 5.3 MHz  */
  BPF_40M,      /* 40m  — 7.0 MHz  */
  BPF_30M,      /* 30m  — 10.1 MHz */
  BPF_20M,      /* 20m  — 14.0 MHz */
  BPF_17M,      /* 17m  — 18.1 MHz */
  BPF_15_10M,   /* 15m  — 21.0 MHz */
  BPF_15_10M,   /* 12m  — 24.9 MHz */
  BPF_15_10M,   /* 10m  — 28.0 MHz */
};

/* Cached LPF state — skip redundant relay cycling */
static lpf_band_t s_lpf_band = LPF_OFF;

/* LPF per band: band_idx → lpf_band_t (= 74HC238 A2:A1:A0 address) */
static const lpf_band_t lpf_map[BAND_COUNT] = {
  LPF_1M8,     /* 160m — 1.8 MHz  → Y0 */
  LPF_3M5,     /* 80m  — 3.5 MHz  → Y1 */
  LPF_5M8,     /* 60m  — 5.3 MHz  → Y2 */
  LPF_8_17M,   /* 40m  — 7.0 MHz  → Y3 */
  LPF_8_17M,   /* 30m  — 10.1 MHz → Y3 */
  LPF_8_17M,   /* 20m  — 14.0 MHz → Y3 */
  LPF_17_32M,  /* 17m  — 18.1 MHz → Y4 */
  LPF_17_32M,  /* 15m  — 21.0 MHz → Y4 */
  LPF_17_32M,  /* 12m  — 24.9 MHz → Y4 */
  LPF_17_32M,  /* 10m  — 28.0 MHz → Y4 */
};

/* Default frequency per band */
static const uint32_t band_default_freq[BAND_COUNT] = {
  1825000UL, 3650000UL, 5357500UL, 7100000UL, 10125000UL,
  14200000UL,18100000UL,21200000UL,24940000UL, 28500000UL
};

static const char *const band_names[BAND_COUNT] = {
  "160m","80m","60m","40m","30m","20m","17m","15m","12m","10m"
};

const char *BPF_BandName(uint8_t idx) {
  return (idx < BAND_COUNT) ? band_names[idx] : "??m";
}

/* Shift one byte MSB-first into the 595 (bit7 ends up on QH .. bit0 on QA)
 * and pulse RCLK to commit it to the storage register in one atomic step.
 * No inter-bit delay needed — 74AHC595 clocks well past 100 MHz, far
 * faster than HAL_GPIO_WritePin() can toggle. */
static void bpf595_shift(uint8_t word)
{
  for (int8_t i = 7; i >= 0; i--) {
    HAL_GPIO_WritePin(BPF_SER_GPIO_Port, BPF_SER_Pin,
                      (word & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BPF_SRCLK_GPIO_Port, BPF_SRCLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BPF_SRCLK_GPIO_Port, BPF_SRCLK_Pin, GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(BPF_RCLK_GPIO_Port, BPF_RCLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BPF_RCLK_GPIO_Port, BPF_RCLK_Pin, GPIO_PIN_RESET);
}

/* Build the "make" word for (mode, filter): S1:S0 set for the requested
 * channel, exactly one OE bit LOW — the bank (from filter's bit 2) and
 * side (RX/TX from mode) being requested. */
static uint8_t bpf595_word(rf_mode_t mode, bpf_filter_t filter)
{
  uint8_t bank   = ((uint8_t)filter >> 2) & 0x01U;
  uint8_t ch     = (uint8_t)filter & 0x03U;
  uint8_t oe_bit = bank
                 ? (mode == RF_MODE_TX ? BPF595_BIT_OE2_2 : BPF595_BIT_OE2_1)
                 : (mode == RF_MODE_TX ? BPF595_BIT_OE1_2 : BPF595_BIT_OE1_1);
  uint8_t word = BPF595_ALL_OFF & (uint8_t)~(1U << oe_bit);
  if (ch & 0x01U) { word |= (1U << BPF595_BIT_S0); }
  if (ch & 0x02U) { word |= (1U << BPF595_BIT_S1); }
  return word;
}

static void set_lpf_ch(uint8_t ch)
{
  HAL_GPIO_WritePin(LPF_A0_GPIO_Port, LPF_A0_Pin,
                    (ch & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LPF_A1_GPIO_Port, LPF_A1_Pin,
                    (ch & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LPF_A2_GPIO_Port, LPF_A2_Pin,
                    (ch & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* USER CODE END 0 */

void BPF_LPF_Init(void)
{
  /* USER CODE BEGIN BPF_LPF_Init_0 */
  /* BPF: 595 outputs are still Hi-Z here (BPF_OE parked HIGH by
   * MX_GPIO_Init) — shift the RX/filter-0 word in FIRST, then pull BPF_OE
   * LOW to enable. This way the 3253 muxes never see anything but the
   * intended word; there is no window where a stale/garbage shift state
   * is briefly exposed on the OE pins. */
  s_bpf_filter = BPF_160M;
  s_rf_mode    = RF_MODE_RX;
  bpf595_shift(bpf595_word(s_rf_mode, s_bpf_filter));
  HAL_GPIO_WritePin(BPF_OE_GPIO_Port, BPF_OE_Pin, GPIO_PIN_RESET);   /* 595 outputs live */

  /* LPF: select Y7 (LPF_OFF) — all relay coils released at power-on */
  s_lpf_band = LPF_OFF;
  set_lpf_ch((uint8_t)LPF_OFF);

  /* T/R switch: default RX (LOW) */
  HAL_GPIO_WritePin(T_R_SW_GPIO_Port, T_R_SW_Pin, GPIO_PIN_RESET);
  /* USER CODE END BPF_LPF_Init_0 */
}

void BPF_Set(rf_mode_t mode, bpf_filter_t filter)
{
  /* USER CODE BEGIN BPF_Set_0 */
  /* Step 1: break — shift+latch the all-OE-off word. Every mux side (both
   * banks, both RX/TX) is disabled before the select bits change, so the
   * step-2 shift below cannot momentarily route a wrong/joined channel. */
  bpf595_shift(BPF595_ALL_OFF);

  /* Step 2: make — shift+latch S1:S0 for the requested filter plus exactly
   * one OE bit low (the requested bank/side). No settle delay needed
   * between the two latches — CBT3253 FET switches in nanoseconds and the
   * 595 storage register updates atomically on RCLK. */
  bpf595_shift(bpf595_word(mode, filter));

  s_bpf_filter = filter;
  s_rf_mode    = mode;
  /* USER CODE END BPF_Set_0 */
}

void BPF_SetMode(rf_mode_t mode)
{
  /* USER CODE BEGIN BPF_SetMode_0 */
  BPF_Set(mode, s_bpf_filter);
  /* USER CODE END BPF_SetMode_0 */
}

void BPF_SetBand(uint8_t band_idx)
{
  /* USER CODE BEGIN BPF_SetBand_0 */
  if (band_idx >= BAND_COUNT) { band_idx = 0U; }
  BPF_Set(s_rf_mode, bpf_map[band_idx]);
  /* USER CODE END BPF_SetBand_0 */
}

void LPF_Set(lpf_band_t band)
{
  /* USER CODE BEGIN LPF_Set_0 */
  if (band == s_lpf_band) { return; }   /* already active — skip relay cycle */

  /* Step 1: route decoder to Y7 (unconnected) — release current relay coil.
   * This prevents multi-bit GPIO transitions from briefly activating
   * an intermediate decoder output. */
  set_lpf_ch((uint8_t)LPF_OFF);

  /* Step 2: allow relay armature to release before energising next coil */
  HAL_Delay(1U);

  /* Step 3: select target filter (skip write if turning off — Y7 already set) */
  if (band != LPF_OFF) {
    set_lpf_ch((uint8_t)band);
  }
  s_lpf_band = band;
  /* USER CODE END LPF_Set_0 */
}

void LPF_SetBand(uint8_t band_idx)
{
  /* USER CODE BEGIN LPF_SetBand_0 */
  if (band_idx >= BAND_COUNT) { band_idx = 0U; }
  LPF_Set(lpf_map[band_idx]);
  /* USER CODE END LPF_SetBand_0 */
}

void BPF_LPF_SetFrequency(uint32_t freq_hz)
{
  /* USER CODE BEGIN BPF_LPF_SetFrequency_0 */
  uint8_t band = BPF_FreqToBand(freq_hz);
  if (band == 0xFFU) { band = 0U; }
  BPF_SetBand(band);
  LPF_SetBand(band);
  /* USER CODE END BPF_LPF_SetFrequency_0 */
}


uint8_t BPF_FreqToBand(uint32_t freq_hz)
{
  /* USER CODE BEGIN BPF_FreqToBand_0 */
  for (uint8_t i = 0U; i < BAND_COUNT; i++) {
    if (freq_hz >= BAND_FREQ_MIN[i] && freq_hz <= BAND_FREQ_MAX[i])
      return i;
  }
  return 0xFFU;
  /* USER CODE END BPF_FreqToBand_0 */
}

uint32_t BPF_BandToFreq(uint8_t band_idx)
{
  if (band_idx >= BAND_COUNT) return 7100000UL;
  return band_default_freq[band_idx];
}

uint8_t BPF_BandUp(uint8_t current)
{
  return (current + 1U) % BAND_COUNT;
}

uint8_t BPF_BandDown(uint8_t current)
{
  return (current == 0U) ? (BAND_COUNT - 1U) : (current - 1U);
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
