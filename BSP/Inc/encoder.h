/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Encoder BSP – TIM2 hardware quadrature interface
  *
  *  Khác với phiên bản EXTI:
  *   - Dùng TIM2->CNT (hardware đếm xung A/B tự động)
  *   - Không cần EXTI handler
  *   - Gia tốc tính theo delta giữa 2 lần đọc CNT
  *   - Nút nhấn PA2 (encoder_sw): polling với debounce
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
  TIM_HandleTypeDef *htim;          /*!< TIM2 handle (hardware quadrature) */
  uint16_t           cnt_prev;      /*!< Giá trị CNT lần đọc trước         */
  int32_t            delta;         /*!< Delta tích lũy kể từ GetDelta()   */
  uint32_t           accel_count;   /*!< Bộ đếm gia tốc                    */
  int32_t            accel_mult;    /*!< Hệ số nhân                         */
  uint32_t           last_tick;     /*!< Tick lần đọc trước                 */
  /* Nút nhấn (polling PA2) */
  bool               btn_pressed;   /*!< Nhấn ngắn pending                  */
  bool               btn_long;      /*!< Nhấn dài pending                   */
  volatile uint32_t  btn_down_tick; /*!< Tick lúc bắt đầu nhấn             */
  bool               btn_prev_state;/*!< Trạng thái nút lần poll trước     */
  uint32_t           debounce_ms;
  uint32_t           long_press_ms;
} Encoder_t;

/* Exported variables --------------------------------------------------------*/
extern Encoder_t g_encoder;

/* Exported functions prototypes ---------------------------------------------*/
void    Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim);
void    Encoder_Poll(Encoder_t *enc);   /*!< Gọi mỗi 1ms từ SysTick */
int32_t Encoder_GetDelta(Encoder_t *enc);
bool    Encoder_GetButton(Encoder_t *enc);
bool    Encoder_GetLongPress(Encoder_t *enc);

/* Compatibility stubs (không dùng với TIM2 quadrature) */
static inline void Encoder_IRQ_Handler(Encoder_t *enc) { (void)enc; }
static inline void Encoder_Btn_IRQ_Handler(Encoder_t *enc) { (void)enc; }

#ifdef __cplusplus
}
#endif
#endif /* __ENCODER_H */
