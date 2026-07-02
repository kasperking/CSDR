/* USER CODE BEGIN Header */
/**
  * @file fsdr_analog.c
  * @brief FSDR Analog: Power, SWR/ALC/Voltage, Fan/NTC
  */
/* USER CODE END Header */

#include "fsdr_analog.h"
#include "csdr_app.h"
#include <math.h>
#include <string.h>

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim17;   /* Fan — TIM17_CH1 = PB9 */
FSDR_Analog_t g_analog = {0};

static uint8_t  s_fan_pct   = 0U;
static bool     s_pwr_held  = false;
static uint32_t s_btn_down_tick = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

/* ── SWR Tandem Match coupler constants ──────────────────────────────────
 *  Core   : BN-61-202 binocular, N = 10 turns secondary, 1 turn primary
 *  Diode  : BAT46JFILM Schottky — Vf ≈ 0.22V at ~2mA detection current
 *  Load   : R36/R54 = 510Ω || C113/C117 = 100nF → peak detector
 *  Ref    : STM32H7 16-bit ADC, Vref = 3.3V
 *
 *  Transfer function (N=10, Z0=50Ω):
 *    V_secondary_peak = sqrt(P_fwd_watts)        [exact for Z0=50Ω, N=10]
 *    V_ADC            = (V_secondary_peak − Vf) × k
 *    V_secondary_peak = V_ADC/k + Vf
 *    P_mW             = (V_ADC/k + Vf)² × 1000
 *
 *  ADC input divider k derived from PA hardware rating (g_sdr.pa_watts):
 *    pa_watts =   0 → R49=0Ω   (k=1.000, P_max≈12W)
 *    pa_watts =  20 → R49=150Ω (k≈0.776, P_max≈20W)
 *    pa_watts =  45 → R49=510Ω (k≈0.509, P_max≈45W)
 *    pa_watts = 100 → R49=1kΩ  (k≈0.338, P_max≈100W)
 *  Formula: k = 3.3 / (sqrt(pa_watts) − Vf), clamped to 1.0
 * ─────────────────────────────────────────────────────────────────────── */
#define SWR_N           10U                /* Secondary turns (BN-61-202)  */
#define SWR_Z0          50U                /* System impedance (Ω)         */
#define SWR_DIODE_VF    0.22f              /* BAT46JFILM Vf at 2mA, 25°C  */
#define SWR_ADC_SCALE   (3.3f / 65535.0f) /* 16-bit ADC, 3.3V Vref       */
#define SWR_MIN_VADC    0.05f              /* Below this → noise floor, P=0 */
/* P_mW = (V_ADC/k + Vf)^2 × N^2/(2×Z0) × 1000                           */
#define SWR_PWR_COEFF   ((float)(SWR_N * SWR_N) * 1000.0f / (2.0f * (float)SWR_Z0))

/* Divider ratio k derived from calibrated PA watts.
 * Returns 1.0 when pa_watts=0 (no external PA / P_max≤12W). */
static inline float swr_k(void)
{
    uint8_t w = g_sdr.pa_watts;
    if (w <= 12U) return 1.0f;
    float k = 3.3f / (sqrtf((float)w) - SWR_DIODE_VF);
    return (k > 1.0f) ? 1.0f : k;
}

/* ── ADC single-shot ─────────────────────────────────────── */
static uint16_t adc_read(ADC_HandleTypeDef *hadc)
{
  HAL_ADC_Start(hadc);
  if (HAL_ADC_PollForConversion(hadc, 10U) == HAL_OK)
    return (uint16_t)HAL_ADC_GetValue(hadc);
  return 0U;
}

/* ── ADC single-shot with explicit channel select ────────── */
static uint16_t adc_read_ch(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_ChannelConfTypeDef cfg = {0};
  cfg.Channel      = channel;
  cfg.Rank         = ADC_REGULAR_RANK_1;
  cfg.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  cfg.SingleDiff   = ADC_SINGLE_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset       = 0;
  if (HAL_ADC_ConfigChannel(hadc, &cfg) != HAL_OK) return 0U;
  return adc_read(hadc);
}

/* USER CODE END 0 */

/* ══════════════════════════════════════════════════════════
 *  POWER MANAGEMENT
 * ══════════════════════════════════════════════════════════ */

void PWR_Init(void)
{
  /* USER CODE BEGIN PWR_Init_0 */
  /* PW_HOLD (PB1, GPIO_Output): already driven HIGH in MX_GPIO_Init_2
   * USER CODE to prevent latch dropout; assert again here as belt-and-suspenders. */
  HAL_GPIO_WritePin(PW_HOLD_GPIO_Port, PW_HOLD_Pin, GPIO_PIN_SET);
  s_pwr_held = true;
  /* USER CODE END PWR_Init_0 */
}

void PWR_Hold(void)
{
  /* USER CODE BEGIN PWR_Hold_0 */
  HAL_GPIO_WritePin(PW_HOLD_GPIO_Port, PW_HOLD_Pin, GPIO_PIN_SET);
  s_pwr_held = true;
  /* USER CODE END PWR_Hold_0 */
}

void PWR_Shutdown(void)
{
  /* USER CODE BEGIN PWR_Shutdown_0 */
  HAL_GPIO_WritePin(PW_HOLD_GPIO_Port, PW_HOLD_Pin, GPIO_PIN_RESET);
  s_pwr_held = false;
  while (1) { __WFI(); }
  /* USER CODE END PWR_Shutdown_0 */
}

bool PWR_IsHeld(void)
{
  return s_pwr_held;
}

/**
  * @brief  Gọi mỗi 100ms: phát hiện nhấn giữ PW để tắt nguồn.
  *         Nhấn giữ > POWER_OFF_HOLD_MS → kích hoạt shutdown sequence.
  */
void PWR_Poll(void)
{
  /* USER CODE BEGIN PWR_Poll_0 */
  bool btn_pressed = (HAL_GPIO_ReadPin(PW_GPIO_Port, PW_Pin) == GPIO_PIN_RESET);
  uint32_t now = HAL_GetTick();

  if (btn_pressed) {
    if (s_btn_down_tick == 0U) { s_btn_down_tick = now; }
    else if ((now - s_btn_down_tick) >= POWER_OFF_HOLD_MS) {
      CSDR_PrepareShutdown();
    }
  } else {
    s_btn_down_tick = 0U;
  }
  /* USER CODE END PWR_Poll_0 */
}

/* ══════════════════════════════════════════════════════════
 *  ANALOG MEASUREMENTS
 * ══════════════════════════════════════════════════════════ */

void Analog_Init(void)
{
  /* USER CODE BEGIN Analog_Init_0 */
  memset(&g_analog, 0, sizeof(g_analog));
  /* USER CODE END Analog_Init_0 */
}

uint16_t Analog_ReadNTC_Raw(void)
{
  return adc_read_ch(&hadc1, ADC_CHANNEL_10);   /* PC0 */
}

uint16_t Analog_ReadVoltage_Raw(void)
{
  return adc_read_ch(&hadc1, ADC_CHANNEL_15);   /* PA3 */
}

uint16_t Analog_ReadALC_Raw(void)
{
  return adc_read(&hadc2);   /* ADC2_INP11 = PC1 */
}

uint16_t Analog_ReadSWR_For_Raw(void)
{
  return adc_read_ch(&hadc3, ADC_CHANNEL_0);   /* ADC3_INP0 = PC2_C */
}

uint16_t Analog_ReadSWR_Ref_Raw(void)
{
  return adc_read_ch(&hadc3, ADC_CHANNEL_1);   /* ADC3_INP1 = PC3_C */
}

/**
  * @brief  NTC → Temperature (°C × 10), Steinhart-Hart simplified.
  *         Vcc - NTC_SERIES - ADC - GND
  *         V_adc = Vcc × R_ntc / (R_series + R_ntc)
  *         R_ntc = R_series × V_adc / (Vcc - V_adc)
  *         1/T = 1/T0 + (1/Beta) × ln(R/R0)
  */
int16_t Analog_NTC_to_Temp_C10(uint16_t adc_raw)
{
  /* USER CODE BEGIN Analog_NTC_to_Temp_C10_0 */
  if (adc_raw == 0U) return 250;   /* 25.0°C fallback */

  float v_ratio = (float)adc_raw / 65535.0f;          /* 0..1 */
  float r_ntc   = (float)NTC_R_SERIES_OHM * v_ratio   /* voltage divider */
                  / (1.0f - v_ratio + 1e-6f);

  /* Steinhart-Hart one-term: 1/T = 1/T0 + (1/B)*ln(R/R0) */
  float T0    = 298.15f;  /* 25°C in Kelvin */
  float ratio = r_ntc / (float)NTC_R_25C_OHM;
  float inv_T = (1.0f / T0) + (1.0f / (float)NTC_BETA) * logf(ratio);
  float T_K   = 1.0f / (inv_T + 1e-10f);
  float T_C   = T_K - 273.15f;

  return (int16_t)(T_C * 10.0f);
  /* USER CODE END Analog_NTC_to_Temp_C10_0 */
}

/**
  * @brief  ADC raw → millivolts (với hệ số voltage divider).
  * @param  gain_x1  Hệ số khuếch đại (1 nếu không có, VOLT_DIV_RATIO cho nguồn)
  */
uint16_t Analog_ADC_to_mV(uint16_t adc_raw, uint16_t gain_x1)
{
  /* USER CODE BEGIN Analog_ADC_to_mV_0 */
  uint32_t mv = ((uint32_t)adc_raw * ADC_VREF_MV) / ADC_FULL_SCALE;
  return (uint16_t)(mv * gain_x1);
  /* USER CODE END Analog_ADC_to_mV_0 */
}

/**
  * @brief  Tính SWR từ điện áp forward và reflected.
  *         SWR = (Vf + Vr) / (Vf - Vr)   → × 100 để tránh float
  * @retval SWR × 100 (ví dụ: 150 = SWR 1.50)
  */
uint16_t Analog_Calc_SWR_x100(uint16_t vfor, uint16_t vref)
{
  /* USER CODE BEGIN Analog_Calc_SWR_x100_0 */
  float vf_v = (float)vfor * SWR_ADC_SCALE;
  if (vf_v < SWR_MIN_VADC) return 100U;   /* no TX signal */
  /* Gamma = amplitude reflection coefficient = V_ref_wave / V_fwd_wave.
   * Both voltages are peak-detected secondary voltages minus Vf; the Vf
   * offset must be restored before taking the ratio so that the formula
   * stays linear (Γ ≠ raw_vref / raw_vfor when Vf is significant). */
  float k       = swr_k();
  float vf_comp = vf_v / k + SWR_DIODE_VF;
  float vr_comp = (float)vref * SWR_ADC_SCALE / k + SWR_DIODE_VF;
  if (vr_comp >= vf_comp) return 9999U;    /* open/short antenna */
  float gamma = vr_comp / vf_comp;
  float swr   = (1.0f + gamma) / (1.0f - gamma + 1e-6f);
  return (uint16_t)(swr * 100.0f);
  /* USER CODE END Analog_Calc_SWR_x100_0 */
}

/**
  * @brief  Đọc và cập nhật tất cả kênh analog.
  *         Gọi mỗi 100ms từ main loop.
  */
void Analog_Update(void)
{
  /* USER CODE BEGIN Analog_Update_0 */

  /* NTC temperature */
  g_analog.adc_ntc   = Analog_ReadNTC_Raw();
  g_analog.temp_c    = Analog_NTC_to_Temp_C10(g_analog.adc_ntc);
  g_analog.temp_alarm = (g_analog.temp_c >= 700);  /* 70.0°C */

  /* Supply voltage (qua voltage divider 1:4) */
  g_analog.adc_voltage  = Analog_ReadVoltage_Raw();
  g_analog.voltage_mv   = Analog_ADC_to_mV(g_analog.adc_voltage, VOLT_DIV_RATIO);
  g_analog.voltage_low  = (g_analog.voltage_mv < 11500U); /* < 11.5V */

  /* ALC */
  g_analog.adc_alc      = Analog_ReadALC_Raw();
  g_analog.alc_percent  = (uint16_t)((uint32_t)g_analog.adc_alc * 100U / 65535U);

  /* SWR */
  g_analog.adc_swr_for  = Analog_ReadSWR_For_Raw();
  g_analog.adc_swr_ref  = Analog_ReadSWR_Ref_Raw();
  g_analog.swr_x100     = Analog_Calc_SWR_x100(g_analog.adc_swr_for,
                                                  g_analog.adc_swr_ref);

  /* Forward/reflected power (N=10, Z0=50Ω → P_mW = (V_ADC + Vf)² × 1000) */
  float vf_v = (float)g_analog.adc_swr_for * SWR_ADC_SCALE;
  float vr_v = (float)g_analog.adc_swr_ref * SWR_ADC_SCALE;
  /* Undo voltage divider (k derived from pa_watts) then restore Vf */
  float k       = swr_k();
  float vf_comp = (vf_v >= SWR_MIN_VADC) ? (vf_v / k + SWR_DIODE_VF) : 0.0f;
  float vr_comp = (vr_v >= SWR_MIN_VADC) ? (vr_v / k + SWR_DIODE_VF) : 0.0f;
  g_analog.fwd_power_mw = (uint16_t)(vf_comp * vf_comp * SWR_PWR_COEFF);
  g_analog.ref_power_mw = (uint16_t)(vr_comp * vr_comp * SWR_PWR_COEFF);
  g_analog.swr_alarm    = (g_analog.swr_x100 > SWR_WARN_THRESH) &&
                           (g_analog.fwd_power_mw > 100U);

  /* USER CODE END Analog_Update_0 */
}

/* ══════════════════════════════════════════════════════════
 *  FAN CONTROL (TIM17_CH1 = PB9)
 * ══════════════════════════════════════════════════════════ */

void Fan_Init(void)
{
  /* USER CODE BEGIN Fan_Init_0 */
  s_fan_pct = 0U;
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0U);
  /* USER CODE END Fan_Init_0 */
}

void Fan_SetPercent(uint8_t pct)
{
  /* USER CODE BEGIN Fan_SetPercent_0 */
  if (pct > 100U) { pct = 100U; }
  s_fan_pct = pct;
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim17);
  uint32_t duty   = 0U;
  if (pct > 0U) {
    duty = FAN_PWM_MIN + (uint32_t)(pct) * (FAN_PWM_MAX - FAN_PWM_MIN) / 100U;
    if (duty > period) duty = period;
  }
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, duty);
  /* USER CODE END Fan_SetPercent_0 */
}

void Fan_Stop(void)
{
  Fan_SetPercent(0U);
}

uint8_t Fan_GetPercent(void)
{
  return s_fan_pct;
}

/**
  * @brief  Điều chỉnh quạt tuyến tính theo nhiệt độ.
  *         temp_c10 < 400  → off
  *         temp_c10 = 400  → minimum speed
  *         temp_c10 >= 650 → full speed
  */
void Fan_Update(int16_t temp_c10)
{
  /* USER CODE BEGIN Fan_Update_0 */
  int16_t t_start = (int16_t)(FAN_TEMP_START_C * 10);
  int16_t t_full  = (int16_t)(FAN_TEMP_FULL_C  * 10);

  if (temp_c10 < t_start) {
    Fan_SetPercent(0U);
  } else if (temp_c10 >= t_full) {
    Fan_SetPercent(100U);
  } else {
    /* Tuyến tính: pct = (T - T_start) / (T_full - T_start) × 100 */
    uint8_t pct = (uint8_t)(((int32_t)(temp_c10 - t_start) * 100)
                             / (int32_t)(t_full - t_start));
    Fan_SetPercent(pct);
  }
  /* USER CODE END Fan_Update_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
