/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    input_scan.h
  * @brief   Centralized input scan – PCA9555 managed path
  *
  *  Architecture:
  *    - Encoder (ENC_SW, TIM1 quadrature): direct MCU, handled by encoder.c
  *    - All function buttons: PCA9555 on I2C2, managed here
  *
  *  Call order each main loop iteration:
  *    Input_Scan()           <- refresh g_pca9555_raw from PCA9555 once
  *    csdr_handle_encoder()  <- reads TIM1 + ENC_SW direct GPIO
  *    csdr_handle_keys()     <- calls Key_Poll() which reads g_pca9555_raw
  *
  *  Input_F4_IsPressed() does a live I2C read and is used only by the
  *  blocking sweep-abort loops in sdr_scan.c.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __INPUT_SCAN_H
#define __INPUT_SCAN_H

/* 0 = PCA9555 not installed; all expander I/O is disabled at compile time */
#define HAS_PCA9555  0

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── PCA9555 configuration ──────────────────────────────────────────────── */

/* I2C address: A0=A1=A2=GND → 7-bit 0x20 → 8-bit HAL 0x40 */
#define INPUT_PCA9555_ADDR   0x40U

/* PCA9555 bit assignments — Port 0 (bits 6:0).
 * PTT/DIT/DAH are direct MCU (PB12/PB13/PB14) — NOT on PCA9555.
 * Port 1 (bits 15:8) is reserved for future expansion. */
#define PCA_BIT_MENU    0U   /*!< MENU key */
#define PCA_BIT_BAND    1U   /*!< BAND key */
#define PCA_BIT_MODE    2U   /*!< MODE key */
#define PCA_BIT_F1      3U   /*!< F1 key   */
#define PCA_BIT_F2      4U   /*!< F2 key   */
#define PCA_BIT_F3      5U   /*!< F3 key   */
#define PCA_BIT_F4      6U   /*!< F4 key   */
/* bit 7 reserved */

/* ── Shared state ───────────────────────────────────────────────────────── */

/**
  * @brief  Cached PCA9555 input register (Port1:Port0), updated by Input_Scan().
  *         Bit = 0 → pin low → button pressed (active-low).
  *         Bit = 1 → pin high → button released.
  *         Initialized to 0xFFFF (all released) before first scan.
  */
extern uint16_t g_pca9555_raw;

/* ── API ────────────────────────────────────────────────────────────────── */

void Input_Init(void);

/**
  * @brief  Read PCA9555 inputs into g_pca9555_raw.
  *         Call once per main loop before Key_Poll().
  *         On I2C error the previous value is preserved.
  */
void Input_Scan(void);

/**
  * @brief  Immediate PCA9555 read — for blocking scan-abort loops only.
  *         Bypasses the Key_t debounce state machine.
  * @retval true if F4 is currently pressed
  */
bool Input_F4_IsPressed(void);

#ifdef __cplusplus
}
#endif
#endif /* __INPUT_SCAN_H */
