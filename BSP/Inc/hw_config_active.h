/* hw_config_active.h -- CSDR Hardware Configuration (auto-generated)
 * DO NOT EDIT -- regenerate with:  python tools/hw_config.py
 *
 * Generated  : 2026-05-29 10:59:36
 * Controller : ST7796
 * Orientation: Landscape BGR
 * FMC width  : 8-bit
 * GPIO speed : MEDIUM
 * Board      : Test board
 * HSE        : 25.000 MHz  CRYSTAL
 * SYSCLK     : 480 MHz  (PLL1 M=5 N=192 P=2)
 * SAI1       : 12.2881 MHz  (PLL2 M=2 N=58 P=59)
 * Storage    : W25Q NOR  W25Q128  128 Mbit
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

/* -- External NVM storage -----------------------------------------------
 * Selected : W25Q NOR Flash  W25Q128  128 Mbit (16 MB)
 *
 * Use #if HW_STORAGE_W25Q / HW_STORAGE_NONE etc. for conditional
 * compilation.  Future variants (FRAM, QSPI_NOR, NAND, SD) will use
 * the same flag pattern with type IDs 4-7.
 *
 * HW_HAS_PERSISTENT_STORAGE : any writable NVM is fitted
 * HW_HAS_LARGE_NVM          : >= 8 Mbit fitted (suitable for IQ/WF buffering)
 * HW_SUPPORTS_WATERFALL_CACHE: large NVM available for waterfall snapshots  */

/* Storage type flags (exactly one equals 1) */
#define HW_STORAGE_NONE              0
#define HW_STORAGE_I2C_EE            0
#define HW_STORAGE_SPI_EE            0
#define HW_STORAGE_W25Q              1
#define HW_STORAGE_FRAM              0   /* future */
#define HW_STORAGE_QSPI_NOR          0  /* future */
#define HW_STORAGE_NAND              0      /* future */
#define HW_STORAGE_SD                0        /* future */

/* Storage type ID  (0=none 1=i2c_ee 2=spi_ee 3=w25q 4-7=future) */
#define HW_STORAGE_TYPE              3

/* Capability flags */
#define HW_HAS_PERSISTENT_STORAGE    1
#define HW_HAS_LARGE_NVM             1
#define HW_SUPPORTS_WATERFALL_CACHE  1

/* W25Q geometry  (valid only when HW_STORAGE_W25Q == 1) */
#define HW_W25Q_CAPACITY_MBIT        128U
#define HW_W25Q_CAPACITY_BYTES       (128UL * 131072UL)
#define HW_W25Q_PAGE_SIZE            256U
#define HW_W25Q_SECTOR_SIZE          4096U
#define HW_W25Q_BLOCK32_SIZE         32768U
#define HW_W25Q_BLOCK64_SIZE         65536U

#endif /* HW_CONFIG_ACTIVE_H */
