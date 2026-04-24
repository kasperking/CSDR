/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wm8731.h
  * @brief   WM8731 Audio Codec BSP Driver
  *          Giao tiếp: I2C control (9-bit regs) + SAI1 I2S audio
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __WM8731_H
#define __WM8731_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  WM8731 initialization structure
  */
typedef struct
{
  I2C_HandleTypeDef *hi2c;          /*!< I2C handle   */
  uint8_t            i2c_addr;      /*!< 7-bit addr shifted (0x1A<<1) */
  uint32_t           sample_rate;   /*!< Target Fs (Hz) */
  uint8_t            input_volume;  /*!< LINE IN gain 0..31 (0x17=0dB) */
  uint8_t            output_volume; /*!< HP output 0..127 (0x79=0dB)   */
  bool               line_in;       /*!< true=LINE IN, false=MIC       */
} WM8731_Config_t;

/* Exported constants --------------------------------------------------------*/

/** @defgroup WM8731_Registers Register map */
#define WM8731_REG_LLINE_IN      0x00U
#define WM8731_REG_RLINE_IN      0x01U
#define WM8731_REG_LHPOUT        0x02U
#define WM8731_REG_RHPOUT        0x03U
#define WM8731_REG_ANALOG_PATH   0x04U
#define WM8731_REG_DIGITAL_PATH  0x05U
#define WM8731_REG_POWER_DOWN    0x06U
#define WM8731_REG_DIGITAL_IF    0x07U
#define WM8731_REG_SAMPLING      0x08U
#define WM8731_REG_ACTIVE        0x09U
#define WM8731_REG_RESET         0x0FU

/** @defgroup WM8731_AnalogPath REG_ANALOG_PATH bits */
#define WM8731_MICBOOST          (1U << 0)
#define WM8731_MUTEMIC           (1U << 1)
#define WM8731_INSEL_LINE        (0U << 2)
#define WM8731_INSEL_MIC         (1U << 2)
#define WM8731_BYPASS            (1U << 3)
#define WM8731_DACSEL            (1U << 4)
#define WM8731_SIDETONE          (1U << 5)

/** @defgroup WM8731_DigitalPath REG_DIGITAL_PATH bits */
#define WM8731_ADCHPD            (1U << 0)  /*!< HPF disable */
#define WM8731_DEEMP_NONE        (0U << 1)
#define WM8731_DEEMP_32K         (1U << 1)
#define WM8731_DEEMP_44K1        (2U << 1)
#define WM8731_DEEMP_48K         (3U << 1)
#define WM8731_DACMU             (1U << 3)  /*!< DAC soft mute */

/** @defgroup WM8731_PowerDown REG_POWER_DOWN bits */
#define WM8731_LINEINPD          (1U << 0)
#define WM8731_MICPD             (1U << 1)
#define WM8731_ADCPD             (1U << 2)
#define WM8731_DACPD             (1U << 3)
#define WM8731_OUTPD             (1U << 4)
#define WM8731_OSCPD             (1U << 5)
#define WM8731_CLKOUTPD          (1U << 6)
#define WM8731_POWEROFF          (1U << 7)

/** @defgroup WM8731_DigitalIF REG_DIGITAL_IF bits */
#define WM8731_FORMAT_I2S        (2U << 0)
#define WM8731_IWL_16BIT         (0U << 2)
#define WM8731_IWL_24BIT         (2U << 2)
#define WM8731_IWL_32BIT         (3U << 2)
#define WM8731_MS_SLAVE          (0U << 6)
#define WM8731_MS_MASTER         (1U << 6)

/** @defgroup WM8731_Sampling REG_SAMPLING bits (USB mode 12.288MHz) */
#define WM8731_USB_MODE          (1U << 0)
#define WM8731_BOSR_250_256      (0U << 1)
#define WM8731_SR_48K_48K        (0x00U << 2)  /*!< ADC 48k / DAC 48k */
#define WM8731_CLKIDIV2          (0U << 6)
#define WM8731_CLKODIV2          (0U << 7)

/** @defgroup WM8731_LineIn REG_LLINE_IN bits */
#define WM8731_LINVOL_MASK       0x1FU
#define WM8731_LINMUTE           (1U << 7)
#define WM8731_LRINBOTH          (1U << 8)

/** @defgroup WM8731_HPOut REG_LHPOUT bits */
#define WM8731_LHPVOL_MASK       0x7FU
#define WM8731_LZCEN             (1U << 7)
#define WM8731_RLHPBOTH          (1U << 8)

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef WM8731_Init(const WM8731_Config_t *cfg);
HAL_StatusTypeDef WM8731_Reset(I2C_HandleTypeDef *hi2c, uint8_t addr);
HAL_StatusTypeDef WM8731_SetVolume(I2C_HandleTypeDef *hi2c, uint8_t addr,
                                    uint8_t left_vol, uint8_t right_vol);
HAL_StatusTypeDef WM8731_SetMute(I2C_HandleTypeDef *hi2c, uint8_t addr,
                                  bool mute);
HAL_StatusTypeDef WM8731_SetInputGain(I2C_HandleTypeDef *hi2c, uint8_t addr,
                                       uint8_t gain);
HAL_StatusTypeDef WM8731_PowerDown(I2C_HandleTypeDef *hi2c, uint8_t addr);

#ifdef __cplusplus
}
#endif
#endif /* __WM8731_H */
