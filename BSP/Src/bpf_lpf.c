/* USER CODE BEGIN Header */
/**
  * @file bpf_lpf.c
  * @brief FST3253 BPF + 74HC238 LPF band switching
  */
/* USER CODE END Header */
#include "bpf_lpf.h"
#include "csdr_app.h"

/* USER CODE BEGIN 0 */
/* All pins from Core/Inc/main.h (CubeMX generated):
 *  BPF: BPF_S0=PA4 (S0/bit0), BPF_S1=PA5 (S1/bit1) — relay select
 *       BPF_OE1=PA6 — active-HIGH, enables TX relay bank (2B1..2B4)
 *       BPF_OE2=PA7 — active-HIGH, enables RX relay bank (1B1..1B4)
 *       OE1 and OE2 must ALWAYS be complementary (never both HIGH).
 *  LPF: LPF_A0=PA0, LPF_A1=PA1, LPF_A2=PA2 (74HC238 address)
 *  T/R: T_R_SW=PB2 (relay control) */

/* Cached OE/filter state — source of truth for BPF_SetMode / BPF_SetBand */
static rf_mode_t   s_rf_mode   = RF_MODE_RX;
static bpf_filter_t s_bpf_filter = BPF_20_30M;

/* BPF filter per band — maps band_idx → bpf_filter_t (truth table) */
static const bpf_filter_t bpf_map[BAND_COUNT] = {
  BPF_80M,      /* 160m — 1.8 MHz, best available is 80m filter */
  BPF_80M,      /* 80m  — 3.5 MHz */
  BPF_40M,      /* 60m  — 5.3 MHz */
  BPF_40M,      /* 40m  — 7.0 MHz */
  BPF_20_30M,   /* 30m  — 10.1 MHz */
  BPF_20_30M,   /* 20m  — 14.0 MHz */
  BPF_20_30M,   /* 17m  — 18.1 MHz */
  BPF_15_10M,   /* 15m  — 21.0 MHz */
  BPF_15_10M,   /* 12m  — 24.9 MHz */
  BPF_15_10M,   /* 10m  — 28.0 MHz */
  BPF_15_10M,   /* 6m   — 50.0 MHz */
};

/* Cached LPF state — skip redundant relay cycling */
static lpf_band_t s_lpf_band = LPF_OFF;

/* LPF per band: band_idx → lpf_band_t (= 74HC238 A2:A1:A0 address)
 * 6m (50 MHz) has no dedicated filter; LPF_17_32M is the closest available. */
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
  LPF_17_32M,  /* 6m   — 50.0 MHz → Y4 (no adequate filter; hardware limit) */
};

/* Default frequency per band */
static const uint32_t band_default_freq[BAND_COUNT] = {
  1825000UL, 3650000UL, 5357500UL, 7100000UL, 10125000UL,
  14200000UL,18100000UL,21200000UL,24940000UL, 28500000UL, 50200000UL
};

static const char *const band_names[BAND_COUNT] = {
  "160m","80m","60m","40m","30m","20m","17m","15m","12m","10m","6m"
};

const char *BPF_BandName(uint8_t idx) {
  return (idx < BAND_COUNT) ? band_names[idx] : "??m";
}

static void set_bpf_ch(uint8_t ch)
{
  /* BPF_S1_Pin (PA4) = relay select bit 0; BPF_S2_Pin (PA5) = relay select bit 1.
   * CubeMX names these S1/S2 but the hardware truth table calls them S0/S1.
   * Do NOT revert to BPF_S0_Pin — that define was removed when GPIOD was
   * reclaimed for FMC; BPF_S0/S1 are now PA4/PA5 named BPF_S1/BPF_S2 in main.h. */
  HAL_GPIO_WritePin(BPF_S1_GPIO_Port, BPF_S1_Pin,
                    (ch & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BPF_S2_GPIO_Port, BPF_S2_Pin,
                    (ch & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  /* BPF: start in RX mode, first filter.
   * OE1=LOW (TX bank off), OE2=HIGH (RX bank on). */
  s_bpf_filter = BPF_20_30M;
  s_rf_mode    = RF_MODE_RX;
  set_bpf_ch((uint8_t)BPF_20_30M);
  HAL_GPIO_WritePin(BPF_OE1_GPIO_Port, BPF_OE1_Pin, GPIO_PIN_RESET); /* TX bank off */
  HAL_GPIO_WritePin(BPF_OE2_GPIO_Port, BPF_OE2_Pin, GPIO_PIN_SET);   /* RX bank on  */

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
  /* Step 1: de-energise both relay banks to prevent crowbar during switching */
  HAL_GPIO_WritePin(BPF_OE1_GPIO_Port, BPF_OE1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BPF_OE2_GPIO_Port, BPF_OE2_Pin, GPIO_PIN_RESET);

  /* Step 2: wait for relay armatures to release */
  HAL_Delay(2U);

  /* Step 3: set filter select bits before energising */
  set_bpf_ch((uint8_t)filter);
  s_bpf_filter = filter;

  /* Step 4: assert exactly one OE — TX→OE1=HIGH, RX→OE2=HIGH */
  if (mode == RF_MODE_TX) {
    HAL_GPIO_WritePin(BPF_OE1_GPIO_Port, BPF_OE1_Pin, GPIO_PIN_SET);
    /* OE2 remains RESET */
  } else {
    HAL_GPIO_WritePin(BPF_OE2_GPIO_Port, BPF_OE2_Pin, GPIO_PIN_SET);
    /* OE1 remains RESET */
  }
  s_rf_mode = mode;
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
