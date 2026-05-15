/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pca9555.h
  * @brief   PCA9555 16-bit I2C GPIO expander – input-only driver
  *
  *  All 16 pins are configured as inputs (Config reg = 0xFF/0xFF).
  *  Raw state is cached in dev->raw after each PCA9555_ReadInputs() call.
  *  Bit = 0 means pin is low (active-low pressed), bit = 1 means high (released).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PCA9555_H
#define __PCA9555_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* PCA9555 register addresses */
#define PCA9555_REG_INPUT0   0x00U   /*!< Input port 0 (bits 7:0)  */
#define PCA9555_REG_INPUT1   0x01U   /*!< Input port 1 (bits 15:8) */
#define PCA9555_REG_OUTPUT0  0x02U
#define PCA9555_REG_OUTPUT1  0x03U
#define PCA9555_REG_CONFIG0  0x06U   /*!< Direction: 1=input, 0=output */
#define PCA9555_REG_CONFIG1  0x07U

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t           addr;   /*!< 8-bit HAL I2C address (7-bit addr << 1) */
  uint16_t           raw;    /*!< Last successful read: Port1[15:8] | Port0[7:0] */
  bool               ok;     /*!< true if last communication succeeded */
} PCA9555_t;

HAL_StatusTypeDef PCA9555_Init(PCA9555_t *dev, I2C_HandleTypeDef *hi2c, uint16_t addr);
HAL_StatusTypeDef PCA9555_ReadInputs(PCA9555_t *dev);

#ifdef __cplusplus
}
#endif
#endif /* __PCA9555_H */
