/* USER CODE BEGIN Header */
/**
  * @file  pca9555.c
  * @brief PCA9555 16-bit I2C GPIO expander – input-only driver
  */
/* USER CODE END Header */

#include "pca9555.h"
#include "input_scan.h"   /* for HAS_PCA9555 */

#if HAS_PCA9555

/* 10 ms: một transaction 3 byte @100 kHz chỉ ~0.3 ms, nhưng timeout HAL có
 * độ phân giải 1 ms tick và main loop bị USB OTG/SAI/LCD ISR chen — 5 ms
 * từng bị vượt lúc stream USB audio, kéo theo blackout phím ở input_scan. */
#define PCA9555_TIMEOUT_MS  10U

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
  dev->raw  = 0xFFFFU;
  dev->dir0 = 0xFFU;   /* all inputs */
  dev->dir1 = 0xFFU;
  dev->out0 = 0x00U;
  dev->out1 = 0x00U;
  dev->ok   = false;

  uint8_t cfg[3] = { PCA9555_REG_CONFIG0, 0xFFU, 0xFFU };
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, addr, cfg, 3U, PCA9555_TIMEOUT_MS);
  dev->ok = (ret == HAL_OK);
  return ret;
}

HAL_StatusTypeDef PCA9555_ConfigDir(PCA9555_t *dev, uint8_t dir0, uint8_t dir1)
{
  dev->dir0 = dir0;
  dev->dir1 = dir1;
  uint8_t cfg[3] = { PCA9555_REG_CONFIG0, dir0, dir1 };
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, cfg, 3U, PCA9555_TIMEOUT_MS);
  dev->ok = (ret == HAL_OK);
  return ret;
}

HAL_StatusTypeDef PCA9555_WritePort(PCA9555_t *dev, uint8_t port, uint8_t val)
{
  uint8_t reg = (port == 0U) ? PCA9555_REG_OUTPUT0 : PCA9555_REG_OUTPUT1;
  if (port == 0U) dev->out0 = val; else dev->out1 = val;
  uint8_t buf[2] = { reg, val };
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, buf, 2U, PCA9555_TIMEOUT_MS);
  if (ret != HAL_OK) dev->ok = false;
  return ret;
}

HAL_StatusTypeDef PCA9555_SetPin(PCA9555_t *dev, uint8_t port, uint8_t pin, uint8_t val)
{
  uint8_t shadow = (port == 0U) ? dev->out0 : dev->out1;
  if (val) shadow |=  (uint8_t)(1U << pin);
  else     shadow &= ~(uint8_t)(1U << pin);
  return PCA9555_WritePort(dev, port, shadow);
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

#endif /* HAS_PCA9555 */
