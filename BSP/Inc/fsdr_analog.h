/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fsdr_analog.h
  * @brief   FSDR Analog subsystems:
  *          - Power Management (PB12/PB13 soft on/off)
  *          - SWR / ALC / Voltage measurement (ADC1/2/3)
  *          - Fan control + NTC temperature (TIM3_CH1 + ADC1_INP10)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __FSDR_ANALOG_H
#define __FSDR_ANALOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ══════════════════════════════════════════════════════════
 *  POWER MANAGEMENT
 *  PD12 PW      – Power latch (drive HIGH = keep ON)
 *  PD13 PW_HOLD – Power-hold signal (drive HIGH = MCU running)
 *
 *  IMPORTANT: IOC currently configures PD12/PD13 as GPIO_Input (PULLUP).
 *  If the power latch requires the MCU to actively drive these pins,
 *  reconfigure them as GPIO_Output in CubeMX and regenerate.
 *
 *  Sequence bật nguồn:
 *   1. Nguồn vào → PW bị pull-up → MCU bắt đầu chạy
 *   2. MCU set PW=HIGH (giữ nguồn qua transistor)
 *   3. MCU init xong → PW_HOLD=HIGH
 *
 *  Sequence tắt nguồn mềm (nhấn giữ ENC_SW):
 *   1. PW_HOLD=LOW
 *   2. PW=LOW → nguồn tắt
 * ══════════════════════════════════════════════════════════ */

void PWR_Init(void);                     /* Set PW=HIGH, PW_HOLD=HIGH         */
void PWR_Hold(void);                     /* Giữ nguồn ON                       */
void PWR_Shutdown(void);                 /* Tắt nguồn mềm                      */
bool PWR_IsHeld(void);                   /* Kiểm tra PW đang HIGH              */
void PWR_Poll(void);                     /* Gọi mỗi 100ms: kiểm tra nút tắt   */

/* ══════════════════════════════════════════════════════════
 *  SWR / ALC / VOLTAGE MEASUREMENT
 *
 *  ADC channels (từ FSDR.ioc):
 *   ADC1_INP15 → PA3  VOLTAGE_IN_METER  (điện áp nguồn qua divider)
 *   ADC2_INP11 → PC1  ALC               (ALC/Mic level)
 *   ADC3_INP0  → PC2_C SWR_FOR          (forward power)
 *   ADC3_INP1  → PC3_C SWR_REF         (reflected power)
 *   ADC1_INP10 → PC0  NTC               (nhiệt độ)
 *
 *  SWR = (Vfor + Vref) / (Vfor - Vref)
 *  Power_W = (Vfor^2) / R_coupler (thường R=50Ω, hệ số cần hiệu chỉnh)
 * ══════════════════════════════════════════════════════════ */

typedef struct {
  /* Raw ADC (16-bit) */
  uint16_t adc_ntc;          /* NTC thermistor        */
  uint16_t adc_voltage;      /* Điện áp nguồn         */
  uint16_t adc_alc;          /* ALC meter             */
  uint16_t adc_swr_for;      /* SWR forward           */
  uint16_t adc_swr_ref;      /* SWR reflected         */

  /* Calculated */
  int16_t  temp_c;           /* Nhiệt độ °C (×10)     */
  uint16_t voltage_mv;       /* Điện áp nguồn (mV)    */
  uint16_t alc_percent;      /* ALC 0-100%            */
  uint16_t swr_x100;         /* SWR × 100 (e.g. 150=1.50) */
  uint16_t fwd_power_mw;     /* Forward power (mW)    */
  uint16_t ref_power_mw;     /* Reflected power (mW)  */

  /* Flags */
  bool swr_alarm;            /* SWR > threshold       */
  bool temp_alarm;           /* Temp > 70°C           */
  bool voltage_low;          /* Voltage < 11.5V       */
} FSDR_Analog_t;

extern FSDR_Analog_t g_analog;

/* ADC handles (extern từ main.c) */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

void Analog_Init(void);

/**
  * @brief  Đọc tất cả kênh ADC và tính toán giá trị.
  *         Gọi mỗi 100ms từ vòng lặp chính.
  */
void Analog_Update(void);

/* Individual reads */
uint16_t Analog_ReadNTC_Raw(void);
uint16_t Analog_ReadVoltage_Raw(void);
uint16_t Analog_ReadALC_Raw(void);
uint16_t Analog_ReadSWR_For_Raw(void);
uint16_t Analog_ReadSWR_Ref_Raw(void);

/* Conversions */
int16_t  Analog_NTC_to_Temp_C10(uint16_t adc_raw);   /* °C × 10 */
uint16_t Analog_ADC_to_mV(uint16_t adc_raw, uint16_t gain_x1);
uint16_t Analog_Calc_SWR_x100(uint16_t vfor, uint16_t vref);

/* ══════════════════════════════════════════════════════════
 *  FAN CONTROL
 *  PB1 → TIM3_CH4 PWM quạt làm mát
 *
 *  Logic:
 *   Temp < FAN_TEMP_START_C  → duty = 0 (tắt)
 *   Temp >= FAN_TEMP_START_C → duty tuyến tính từ MIN→MAX
 *   Temp >= FAN_TEMP_FULL_C  → duty = MAX (full speed)
 *   Temp >= 75°C             → cảnh báo, giảm công suất TX
 *
 *  PWM: TIM3_CH1, period=999 (1kHz @ APB1=120MHz, PSC=119)
 * ══════════════════════════════════════════════════════════ */

extern TIM_HandleTypeDef htim3;

void Fan_Init(void);

/**
  * @brief  Cập nhật tốc độ quạt theo nhiệt độ.
  *         Gọi mỗi 1 giây.
  * @param  temp_c10  Nhiệt độ °C × 10 (từ Analog_NTC_to_Temp_C10)
  */
void Fan_Update(int16_t temp_c10);

/**
  * @brief  Đặt tốc độ quạt thủ công (0-100%).
  */
void Fan_SetPercent(uint8_t pct);

/**
  * @brief  Dừng quạt hoàn toàn.
  */
void Fan_Stop(void);

/**
  * @brief  Lấy tốc độ quạt hiện tại (0-100%).
  */
uint8_t Fan_GetPercent(void);

#ifdef __cplusplus
}
#endif
#endif /* __FSDR_ANALOG_H */
