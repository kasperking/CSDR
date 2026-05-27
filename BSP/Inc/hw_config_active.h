/* hw_config_active.h -- CSDR Hardware Configuration (auto-generated)
 * DO NOT EDIT -- regenerate with:  python tools/hw_config.py
 *
 * Generated  : 2026-05-27 16:30:01
 * Controller : ST7796
 * Orientation: Landscape BGR
 * FMC width  : 8-bit
 * GPIO speed : MEDIUM
 * Board      : Test board
 * HSE        : 25.000 MHz  CRYSTAL
 * SYSCLK     : 480 MHz  (PLL1 M=5 N=192 P=2)
 * SAI1       : 12.2881 MHz  (PLL2 M=2 N=58 P=59)
 */

#ifndef HW_CONFIG_ACTIVE_H
#define HW_CONFIG_ACTIVE_H

/* -- Panel ---------------------------------------------------------------
 * HW_LCD_PANEL is read by lcd_panel_config.h to select the driver path.
 * LCD_W / LCD_H are provided here so portrait and landscape both resolve
 * correctly without editing lcd_panel_config.h.                          */
#define HW_LCD_PANEL        1   /* ST7796 */
#define LCD_W               480U
#define LCD_H               320U

/* -- MADCTL (register 0x36) ----------------------------------------------
 * MY|MX|MV|BGR  landscape, BGR filter                                 */
#define HW_LCD_MADCTL       0xE8U

/* -- FMC SRAM timing (AHB cycles, asynchronous mode A) ------------------
 * Limits per STM32H7 RM0433: ADDR_SETUP 0-15, ADDR_HOLD 1-15,
 * DATA_SETUP 1-255, BUS_TURN 0-15, CLK_DIV 2-16, DATA_LATENCY 2-17.   */
#define HW_FMC_ADDR_SETUP   2U
#define HW_FMC_ADDR_HOLD    15U
#define HW_FMC_DATA_SETUP   10U
#define HW_FMC_BUS_TURN     15U
#define HW_FMC_CLK_DIV      16U
#define HW_FMC_DATA_LATENCY 17U

/* -- FMC GPIO drive strength ---------------------------------------------
 * One of GPIO_SPEED_FREQ_LOW / MEDIUM / HIGH / VERY_HIGH               */
#define HW_FMC_GPIO_SPEED   GPIO_SPEED_FREQ_MEDIUM

/* -- LCD DMA push chunk size (spectrum strip height) --------------------
 * Must be <= SPEC_H (72 for ST7796, 76 for ST7789). Valid range 1-64.  */
#define HW_DMA_CHUNK_ROWS   8U

/* -- HSE clock source and PLL configuration -----------------------------
 * hw_config.py also patches Core/Src/main.c (PLL1/PLL2 dividers,
 * HSEState) and HSE_VALUE in stm32h7xx_hal_conf.h / system_stm32h7xx.c.
 * These macros mirror those patched values for reference / static assert.*/
#define HW_HSE_FREQ_HZ      25000000UL
#define HW_HSE_RCC_MODE     RCC_HSE_ON

#define HW_PLL1_M           5U
#define HW_PLL1_N           192U
#define HW_PLL1_P           2U
#define HW_PLL1_VCIRANGE    RCC_PLL1VCIRANGE_2

#define HW_PLL2_M           2U
#define HW_PLL2_N           58U
#define HW_PLL2_P           59U
#define HW_PLL2_VCIRANGE    RCC_PLL2VCIRANGE_3

#endif /* HW_CONFIG_ACTIVE_H */
