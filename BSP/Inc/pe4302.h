/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pe4302.h
  * @brief   PE4302 6-bit Digital Step Attenuator BSP Driver
  *
  *  Giao tiếp: Bit-bang SPI (DATA, CLK, LE/LATCH)
  *   PD8  ATT_DAT  → DATA pin
  *   PD9  ATT_CLK  → CLK  pin
  *   PD10 ATT_LATCH→ LE   pin (active high latch)
  *
  *  Protocol:
  *   - MSB first, 6-bit word: [D5 D4 D3 D2 D1 D0]
  *   - Giá trị: 0x00=0dB, 0x01=0.5dB, ... 0x3F=31.5dB
  *   - Bước: 0.5dB/LSB → dải 0..31.5dB
  *   - Latch: clock 6 bits vào, sau đó pulse LE HIGH→LOW
  *
  *  Trong SDR receiver: dùng để điều chỉnh gain đầu vào
  *  trước mixer QSD. Tăng att khi tín hiệu mạnh (giảm IMD).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PE4302_H
#define __PE4302_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define PE4302_MAX_ATTN_X2   63U    /*!< 63 × 0.5dB = 31.5dB  */
#define PE4302_STEP_0_5DB    1U     /*!< 1 LSB = 0.5dB         */

/* Exported types ------------------------------------------------------------*/
typedef struct {
  GPIO_TypeDef *dat_port;  uint16_t dat_pin;   /* PD8  ATT_DAT  */
  GPIO_TypeDef *clk_port;  uint16_t clk_pin;   /* PD9  ATT_CLK  */
  GPIO_TypeDef *le_port;   uint16_t le_pin;    /* PD10 ATT_LATCH*/
  uint8_t current_atten_db;   /*!< Giá trị dB hiện tại (0-31)   */
  uint8_t current_atten_x2;   /*!< Raw register value (0-63)    */
} PE4302_Handle_t;

/* Exported variables --------------------------------------------------------*/
extern PE4302_Handle_t g_att;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Khởi tạo PE4302 – set tất cả chân output, đặt attenuation=0
  */
void PE4302_Init(PE4302_Handle_t *att);

/**
  * @brief  Đặt attenuation theo dB nguyên (0..31 dB).
  *         Tự động làm tròn về 0.5dB gần nhất.
  * @param  att    Handle
  * @param  db     Giá trị attenuation 0-31 dB
  */
void PE4302_SetAttn_dB(PE4302_Handle_t *att, uint8_t db);

/**
  * @brief  Đặt attenuation theo 0.5dB bước (0..63).
  *         reg_val = db × 2 → 0x00=0dB, 0x3F=31.5dB
  * @param  att     Handle
  * @param  val_x2  0..63
  */
void PE4302_SetAttn_Raw(PE4302_Handle_t *att, uint8_t val_x2);

/**
  * @brief  Tăng attenuation thêm 1dB (2 bước 0.5dB).
  */
void PE4302_IncAttn(PE4302_Handle_t *att);

/**
  * @brief  Giảm attenuation 1dB.
  */
void PE4302_DecAttn(PE4302_Handle_t *att);

/**
  * @brief  Đặt attenuation = 0 (bypass).
  */
void PE4302_Bypass(PE4302_Handle_t *att);

#ifdef __cplusplus
}
#endif
#endif /* __PE4302_H */
