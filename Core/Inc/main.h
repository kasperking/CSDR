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
#define LPF_A0_Pin GPIO_PIN_0
#define LPF_A0_GPIO_Port GPIOA
#define LPF_A1_Pin GPIO_PIN_1
#define LPF_A1_GPIO_Port GPIOA
#define LPF_A2_Pin GPIO_PIN_2
#define LPF_A2_GPIO_Port GPIOA
#define VOLTAGE_IN_METER_Pin GPIO_PIN_3
#define VOLTAGE_IN_METER_GPIO_Port GPIOA
#define BPF_S1_Pin GPIO_PIN_4
#define BPF_S1_GPIO_Port GPIOA
#define BPF_S2_Pin GPIO_PIN_5
#define BPF_S2_GPIO_Port GPIOA
#define BPF_OE1_Pin GPIO_PIN_6
#define BPF_OE1_GPIO_Port GPIOA
#define BPF_OE2_Pin GPIO_PIN_7
#define BPF_OE2_GPIO_Port GPIOA
#define ATT_DAT_Pin GPIO_PIN_4
#define ATT_DAT_GPIO_Port GPIOC
#define ATT_CLK_Pin GPIO_PIN_5
#define ATT_CLK_GPIO_Port GPIOC
#define ATT_LATCH_Pin GPIO_PIN_0
#define ATT_LATCH_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_9
#define FAN_GPIO_Port GPIOB
#define T_R_SW_Pin GPIO_PIN_2
#define T_R_SW_GPIO_Port GPIOB
#define PTT_Pin GPIO_PIN_12
#define PTT_GPIO_Port GPIOB
#define DIT_Pin GPIO_PIN_13
#define DIT_GPIO_Port GPIOB
#define DAH_Pin GPIO_PIN_14
#define DAH_GPIO_Port GPIOB
#define PW_Pin GPIO_PIN_12
#define PW_GPIO_Port GPIOD
#define PW_HOLD_Pin GPIO_PIN_1
#define PW_HOLD_GPIO_Port GPIOB
/* PC6 was the legacy SPI-LCD RS/DC signal. RS/DC is now FMC_A16 (PD11).
 * PC6 is unused on the board; kept as a driven-low output to avoid floating. */
#define NC_PC6_Pin GPIO_PIN_6
#define NC_PC6_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_9
#define LCD_BL_GPIO_Port GPIOC
#define ENC_CH1_Pin GPIO_PIN_4
#define ENC_CH1_GPIO_Port GPIOB
#define ENC_CH2_Pin GPIO_PIN_5
#define ENC_CH2_GPIO_Port GPIOB
#define ENC_SW_Pin GPIO_PIN_10
#define ENC_SW_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_15
#define FLASH_CS_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_10
#define FLASH_SCK_GPIO_Port GPIOC
#define FLASH_MISO_Pin GPIO_PIN_11
#define FLASH_MISO_GPIO_Port GPIOC
#define FLASH_MOSI_Pin GPIO_PIN_12
#define FLASH_MOSI_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
