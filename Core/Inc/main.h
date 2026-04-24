/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NTC_Pin GPIO_PIN_0
#define NTC_GPIO_Port GPIOC
#define ALC_Pin GPIO_PIN_1
#define ALC_GPIO_Port GPIOC
#define SWR_FOR_Pin GPIO_PIN_2
#define SWR_FOR_GPIO_Port GPIOC
#define SWR_REF_Pin GPIO_PIN_3
#define SWR_REF_GPIO_Port GPIOC
#define ENC_CH1_Pin GPIO_PIN_0
#define ENC_CH1_GPIO_Port GPIOA
#define ENC_CH2_Pin GPIO_PIN_1
#define ENC_CH2_GPIO_Port GPIOA
#define ENC_SW_Pin GPIO_PIN_2
#define ENC_SW_GPIO_Port GPIOA
#define VOLTAGE_IN_METER_Pin GPIO_PIN_3
#define VOLTAGE_IN_METER_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LCD_SCL_Pin GPIO_PIN_5
#define LCD_SCL_GPIO_Port GPIOA
#define LCD_SDA_Pin GPIO_PIN_7
#define LCD_SDA_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_0
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOB
#define MENU_KEY_Pin GPIO_PIN_7
#define MENU_KEY_GPIO_Port GPIOE
#define BAND_KEY_Pin GPIO_PIN_8
#define BAND_KEY_GPIO_Port GPIOE
#define MODE_KEY_Pin GPIO_PIN_9
#define MODE_KEY_GPIO_Port GPIOE
#define F1_KEY_Pin GPIO_PIN_10
#define F1_KEY_GPIO_Port GPIOE
#define F2_KEY_Pin GPIO_PIN_11
#define F2_KEY_GPIO_Port GPIOE
#define F3_KEY_Pin GPIO_PIN_12
#define F3_KEY_GPIO_Port GPIOE
#define F4_KEY_Pin GPIO_PIN_13
#define F4_KEY_GPIO_Port GPIOE
#define PTT_Pin GPIO_PIN_14
#define PTT_GPIO_Port GPIOE
#define DAH_KEY_Pin GPIO_PIN_15
#define DAH_KEY_GPIO_Port GPIOE
#define DIT_KEY_Pin GPIO_PIN_10
#define DIT_KEY_GPIO_Port GPIOB
#define CPU_PW_Pin GPIO_PIN_12
#define CPU_PW_GPIO_Port GPIOB
#define CPU_PW_HOLD_Pin GPIO_PIN_13
#define CPU_PW_HOLD_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_15
#define LCD_BL_GPIO_Port GPIOB
#define ATT_DAT_Pin GPIO_PIN_8
#define ATT_DAT_GPIO_Port GPIOD
#define ATT_CLK_Pin GPIO_PIN_9
#define ATT_CLK_GPIO_Port GPIOD
#define ATT_LATCH_Pin GPIO_PIN_10
#define ATT_LATCH_GPIO_Port GPIOD
#define BPF_OE_Pin GPIO_PIN_11
#define BPF_OE_GPIO_Port GPIOD
#define BPF_S0_Pin GPIO_PIN_12
#define BPF_S0_GPIO_Port GPIOD
#define BPF_S1_Pin GPIO_PIN_13
#define BPF_S1_GPIO_Port GPIOD
#define T_R_SW_Pin GPIO_PIN_14
#define T_R_SW_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_6
#define FAN_GPIO_Port GPIOC
#define LPF_A0_Pin GPIO_PIN_8
#define LPF_A0_GPIO_Port GPIOC
#define LPF_A1_Pin GPIO_PIN_9
#define LPF_A1_GPIO_Port GPIOC
#define LPF_A2_Pin GPIO_PIN_8
#define LPF_A2_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_10
#define FLASH_SCK_GPIO_Port GPIOC
#define FLASH_MISO_Pin GPIO_PIN_11
#define FLASH_MISO_GPIO_Port GPIOC
#define FLASH_MOSI_Pin GPIO_PIN_12
#define FLASH_MOSI_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
