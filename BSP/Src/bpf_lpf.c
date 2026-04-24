/* USER CODE BEGIN Header */
/**
  * @file bpf_lpf.c
  * @brief FST3253 BPF + 74HC238 LPF band switching
  */
/* USER CODE END Header */
#include "bpf_lpf.h"
#include "csdr_app.h"

/* USER CODE BEGIN 0 */
/* LPF pins from CSDR.ioc: PC8=LPF_A0, PC9=LPF_A1, PA8=LPF_A2
 * Defined in Core/Inc/main.h as LPF_A0/A1/A2_Pin/_GPIO_Port */

/* BPF channel per band: S1:S0 */
static const uint8_t bpf_map[BAND_COUNT] = {
  BPF_CH_160_80M,  /* 160m */
  BPF_CH_160_80M,  /* 80m  */
  BPF_CH_40_30M,   /* 60m  */
  BPF_CH_40_30M,   /* 40m  */
  BPF_CH_40_30M,   /* 30m  */
  BPF_CH_20_15M,   /* 20m  */
  BPF_CH_20_15M,   /* 17m  */
  BPF_CH_20_15M,   /* 15m  */
  BPF_CH_10_6M,    /* 12m  */
  BPF_CH_10_6M,    /* 10m  */
  BPF_CH_10_6M,    /* 6m   */
};

/* LPF channel per band: A2:A1:A0 */
static const uint8_t lpf_map[BAND_COUNT] = {
  LPF_CH_160M, LPF_CH_80M, LPF_CH_80M, LPF_CH_40M, LPF_CH_40M,
  LPF_CH_20M,  LPF_CH_20M, LPF_CH_15M, LPF_CH_15M, LPF_CH_10M,
  LPF_CH_6M,
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
  HAL_GPIO_WritePin(BPF_S0_GPIO_Port, BPF_S0_Pin,
                    (ch & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BPF_S1_GPIO_Port, BPF_S1_Pin,
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
  /* BPF: S0=S1=0 (channel 0), OE disabled */
  set_bpf_ch(0U);
  HAL_GPIO_WritePin(BPF_OE_GPIO_Port, BPF_OE_Pin, GPIO_PIN_SET);

  /* LPF: A=111 (none) */
  set_lpf_ch(LPF_CH_NONE);

  /* T/R switch: default RX */
  HAL_GPIO_WritePin(TR_SW_GPIO_Port, TR_SW_Pin, GPIO_PIN_RESET);
  /* USER CODE END BPF_LPF_Init_0 */
}

void BPF_SetBand(uint8_t band_idx)
{
  /* USER CODE BEGIN BPF_SetBand_0 */
  if (band_idx >= BAND_COUNT) { band_idx = 0U; }
  set_bpf_ch(bpf_map[band_idx]);
  /* USER CODE END BPF_SetBand_0 */
}

void LPF_SetBand(uint8_t band_idx)
{
  /* USER CODE BEGIN LPF_SetBand_0 */
  if (band_idx >= BAND_COUNT) { band_idx = 0U; }
  set_lpf_ch(lpf_map[band_idx]);
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

void BPF_Enable(bool enable)
{
  /* USER CODE BEGIN BPF_Enable_0 */
  /* BPF_OE là input readback trong .ioc, nhưng thực tế là output active-low */
  /* Nếu cần drive: reconfigure as output */
  (void)enable;
  /* USER CODE END BPF_Enable_0 */
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
