/* USER CODE BEGIN Header */
/**
  * @file  pca9555.c
  * @brief PCA9555 16-bit I2C GPIO expander – input-only driver
  */
/* USER CODE END Header */

#include "pca9555.h"

#define PCA9555_TIMEOUT_MS  5U

/**
  * @brief  Configure all 16 PCA9555 pins as inputs.
  * @param  dev   Driver handle
  * @param  hi2c  I2C peripheral (I2C2)
  * @param  addr  8-bit HAL address (7-bit device address << 1)
  */
HAL_StatusTypeDef PCA9555_Init(PCA9555_t *dev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
  dev->hi2c = hi2c;
  dev->addr = addr;
  dev->raw  = 0xFFFFU;   /* default: all high = all released */
  dev->ok   = false;

  /* Write Config0 + Config1: all 1s = all inputs */
  uint8_t cfg[3] = { PCA9555_REG_CONFIG0, 0xFFU, 0xFFU };
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, addr, cfg, 3U, PCA9555_TIMEOUT_MS);
  dev->ok = (ret == HAL_OK);
  return ret;
}

/**
  * @brief  Read both input ports into dev->raw.
  *         On I2C error the previous raw value is preserved.
  * @retval HAL_OK on success
  */
HAL_StatusTypeDef PCA9555_ReadInputs(PCA9555_t *dev)
{
  uint8_t reg = PCA9555_REG_INPUT0;
  uint8_t buf[2];

  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
      dev->hi2c, dev->addr, &reg, 1U, PCA9555_TIMEOUT_MS);
  if (ret != HAL_OK) {
    dev->ok = false;
    return ret;
  }

  ret = HAL_I2C_Master_Receive(
      dev->hi2c, dev->addr, buf, 2U, PCA9555_TIMEOUT_MS);
  if (ret == HAL_OK) {
    dev->raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8U);
    dev->ok  = true;
  } else {
    dev->ok = false;
  }
  return ret;
}
