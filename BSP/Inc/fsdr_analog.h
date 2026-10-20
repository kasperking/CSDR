/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fsdr_analog.h
  * @brief   FSDR Analog subsystems:
  *          - Power Management (PB12 nút PW + PB2 PW_HOLD soft on/off)
  *          - SWR / ALC / Voltage measurement (ADC1/2/3)
  *          - Fan control + NTC temperature (TIM17_CH1 + ADC1_INP10)
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
 *  PB12 PW      – Power button input (INPUT_PULLUP; LOW = button held)
 *  PB2  PW_HOLD – Power latch output (drive HIGH = keep ON, LOW = cut power)
 *
 *  Sequence bật nguồn:
 *   1. User nhấn PW → nguồn vào → MCU bắt đầu chạy
 *   2. MCU set PW_HOLD=HIGH → latch giữ nguồn dù nhả nút
 *
 *  Sequence tắt nguồn mềm (nhấn giữ PW > POWER_OFF_HOLD_MS):
 *   1. PW_HOLD=LOW → nguồn tắt
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
 *   ADC1_INP16 → PA0  ALC               (ALC/Mic level)
 *   ADC1_INP17 → PA1  SWR_FOR           (forward power)
 *   ADC2_INP14 → PA2  SWR_REF           (reflected power)
 *   ADC1_INP15 → PA3  VOLTAGE_IN_METER  (điện áp nguồn qua divider)
 *   ADC1_INP18 → PA4  NTC               (nhiệt độ)
 *  Cả 5 chân nằm ở góc dưới-phải LQFP100 (pin 22-28), quay về phía PA/J4.
 *  FOR (ADC1) và REF (ADC2) ở hai ADC khác nhau → có thể chuyển sang dual
 *  regular-simultaneous sau này để lấy mẫu FOR/REF cùng thời điểm.
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
 *  PB9 → TIM17_CH1 PWM quạt làm mát
 *
 *  Logic:
 *   Temp < FAN_TEMP_START_C  → duty = 0 (tắt)
 *   Temp >= FAN_TEMP_START_C → duty tuyến tính từ MIN→MAX
 *   Temp >= FAN_TEMP_FULL_C  → duty = MAX (full speed)
 *   Temp >= 75°C             → cảnh báo, giảm công suất TX
 * ══════════════════════════════════════════════════════════ */

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
