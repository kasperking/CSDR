/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pa_overcurrent.h
  * @brief   PA Overcurrent Protection — policy layer trên INA226 driver
  *
  *  Module này chứa logic bảo vệ PA: ngưỡng dòng, EXTI ISR, xử lý lỗi trong
  *  main loop.  Mọi giao tiếp thanh ghi với INA226 được ủy quyền cho ina226.h.
  *
  *  Sơ đồ phần cứng:
  *
  *    +VDD_BIAS ─── Drain(Q1) ─── PA Gate/Base Bias output
  *                  Source(Q1) ── GND
  *                  Gate(Q1):
  *                      ├── 10kΩ → +3.3V          (giữ ON mặc định)
  *                      ├── ALERT (INA226) ──100Ω──┤  (override cứng, active-LOW)
  *                      └── MCU PA_BIAS_EN  ──1kΩ──┤  (điều khiển PTT)
  *
  *    ALERT LOW (quá dòng) → Vgate ≈ 0.3V → Q1 OFF → bias cắt trong ~140 µs
  *
  *  ALERT GPIO: PC6 (NC_PC6, pin không dùng trên board)
  *    CubeMX: GPIO_Input, Pull-up, EXTI Falling Edge, NVIC priority 5
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PA_OVERCURRENT_H
#define __PA_OVERCURRENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ina226.h"
#include <stdint.h>
#include <stdbool.h>

/* ─── PA-specific INA226 configuration ──────────────────────────────────────
 * Thay đổi INA226_ADDR_7BIT nếu chân A0/A1 được nối khác. */
#define PA_OC_INA226_ADDR_7BIT  0x40U       /*!< A0=GND, A1=GND                */
#define PA_OC_SHUNT_OHM         0.02f       /*!< Trở shunt 20mΩ, loại 2512 1%  */

/* ─── Default và giới hạn ngưỡng bảo vệ ────────────────────────────────────
 * Giới hạn vật lý = 0x7FFF × 2.5µV / 0.02Ω ≈ 4.096 A */
#define PA_OC_LIMIT_DEFAULT_A   3.5f
#define PA_OC_LIMIT_MAX_A       4.09f

/* ─── ALERT GPIO ─────────────────────────────────────────────────────────────
 * PC6 → EXTI line 6 → nhóm EXTI9_5_IRQn (STM32 gộp lines 5-9 chung 1 vector).
 * IOC: PC6.Signal=GPXTI6, GPIO_MODE_IT_FALLING, GPIO_PULLUP.                */
#define PA_OC_ALERT_GPIO_PORT   GPIOC
#define PA_OC_ALERT_GPIO_PIN    GPIO_PIN_6
#define PA_OC_ALERT_EXTI_IRQn   EXTI9_5_IRQn

/* ─── State ──────────────────────────────────────────────────────────────── */
typedef struct {
    INA226_Handle_t ina;            /*!< INA226 driver handle                  */
    bool     fault_latched;         /*!< Alert latch chưa được xóa             */
    bool     fault_pending;         /*!< Cờ ISR → main loop chưa xử lý        */
    uint32_t fault_count;           /*!< Tổng số lần lỗi từ boot               */
    uint32_t fault_tick_ms;         /*!< Tick lúc lỗi xảy ra (HAL_GetTick)    */
    float    limit_a;               /*!< Ngưỡng hiện tại (A)                   */
    bool     ina_ok;                /*!< false nếu INA226 không phản hồi qua I2C */
    uint32_t ina_retry_ms;          /*!< HAL_GetTick() khi cho phép thử lại I2C  */
} PA_OC_State_t;

extern PA_OC_State_t g_pa_oc;

/* ─── API ─────────────────────────────────────────────────────────────────── */

/**
  * @brief  Khởi tạo INA226 với cấu hình bảo vệ PA: SOL latch, 140 µs, ngưỡng mặc định.
  *         Gọi một lần trong CSDR_Init sau khi I2C2 sẵn sàng.
  */
void PA_OC_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief  Thay đổi ngưỡng dòng bảo vệ tại runtime.
  *         Gọi khi vào TX để nạp ngưỡng theo chế độ (CW/SSB/DIGI).
  * @param  current_a  Giới hạn (A), tối đa PA_OC_LIMIT_MAX_A
  */
void PA_OC_SetCurrentLimit(float current_a);

/**
  * @brief  Đọc dòng PA tức thời (A). Chỉ gọi từ main loop.
  */
float PA_OC_ReadCurrent(void);

/**
  * @brief  Gọi từ EXTI9_5_IRQHandler khi chân ALERT xuống LOW.
  *         Chỉ đặt cờ — không gọi I2C, không gọi HAL.
  */
void PA_OC_AlertISR(void);

/**
  * @brief  Gọi từ CSDR_Loop mỗi vòng lặp để xử lý fault_pending:
  *         chờ 200ms, xóa INA226 latch, cập nhật UI.
  * @return true nếu đang xử lý lỗi (kể cả khi đang chờ 200ms)
  */
bool PA_OC_HandleFaultInLoop(void);

#ifdef __cplusplus
}
#endif
#endif /* __PA_OVERCURRENT_H */
