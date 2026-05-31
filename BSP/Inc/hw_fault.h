/* USER CODE BEGIN Header */
/**
  * @file  hw_fault.h
  * @brief Centralized hardware fault registry.
  *
  *  Any module that detects a persistent hardware absence sets a bit here.
  *  CSDR_Loop reads HW_Fault_Any() and replaces the spectrum/WF zone with a
  *  warning overlay listing every failed component.
  *
  *  Sources:
  *    Boot   — CSDR_Init maps g_selftest failures → HW_Fault_Set after SelfTest_Run
  *    Runtime — Input_Scan (PCA9555 retry ×3) → HW_FAULT_KEYS
  *             PA_OC_ReadCurrent (INA226 retry ×3) → HW_FAULT_INA226
  *
  *  Enable:
  *    Set HW_FAULT_WARN to 1 once hardware population is complete.
  *    At 0 the entire overlay path is compiled out with no runtime cost.
  */
/* USER CODE END Header */

#ifndef __HW_FAULT_H
#define __HW_FAULT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Set to 1 after development phase (all hardware populated).
 * At 0 every HW_Fault_* call and the overlay render are compiled out. */
#define HW_FAULT_WARN  0

/* ── Fault bits — one per hardware component ─────────────────────────── */
#define HW_FAULT_FLASH   (1UL << 0)   /*!< W25Q128 SPI flash — settings persistence  */
#define HW_FAULT_CODEC   (1UL << 1)   /*!< WM8731 audio codec — RX/TX audio          */
#define HW_FAULT_PLL     (1UL << 2)   /*!< SI5351 oscillator — VFO / LO synthesis    */
#define HW_FAULT_INA226  (1UL << 3)   /*!< INA226 current sensor — PA protection     */
#define HW_FAULT_SAI     (1UL << 4)   /*!< SAI audio DMA — I2S hardware interface    */
#define HW_FAULT_KEYS    (1UL << 5)   /*!< PCA9555 key expander — function keys      */

/* ── Global fault mask ──────────────────────────────────────────────── */
extern volatile uint32_t g_hw_fault_mask;

/* ── Inline helpers ─────────────────────────────────────────────────── */

/** @brief  Assert one or more fault bits (OR-assign, idempotent). */
static inline void HW_Fault_Set(uint32_t bits) { g_hw_fault_mask |= bits; }

/** @brief  true when at least one hardware component is flagged faulty. */
static inline bool HW_Fault_Any(void) { return g_hw_fault_mask != 0U; }

#ifdef __cplusplus
}
#endif
#endif /* __HW_FAULT_H */
