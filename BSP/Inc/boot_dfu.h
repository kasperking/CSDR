/* USER CODE BEGIN Header */
/**
 * @file  boot_dfu.h
 * @brief Power-on USB DFU mode detection and ROM bootloader jump.
 *
 *  Call sequence (inside MX_FMC_Init USER CODE FMC_Init 2, after LCD_Bus_Init):
 *    HAL_Delay(50);          -- debounce
 *    if (boot_dfu_requested()) {
 *        ui_show_dfu_screen();
 *        HAL_Delay(1000);
 *        boot_enter_dfu();
 *    }
 *
 *  At that call site GPIO and FMC/LCD are ready; USB stack has NOT started.
 */
/* USER CODE END Header */

#ifndef BOOT_DFU_H
#define BOOT_DFU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* Returns true when PW_KEY and ENC_SW are both held at power-on. */
bool boot_dfu_requested(void);

/* Clears the display and shows a DFU splash (black bg, white title, cyan sub). */
void ui_show_dfu_screen(void);

/* Tears down all peripherals and jumps to the STM32H750 ROM USB-DFU bootloader.
 * Never returns. */
void boot_enter_dfu(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_DFU_H */
