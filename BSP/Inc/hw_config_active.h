/* AUTO-GENERATED — do not edit manually.
 * Run: python tools/select_hw_profile.py hw_test_fmc
 * Source: config/hw_profiles/hw_test_fmc.h
 */

#ifndef HW_CONFIG_ACTIVE_H
#define HW_CONFIG_ACTIVE_H

/*
 * hw_test_fmc.h — Hardware profile: test/development board, ST7796 480x320.
 *
 * Target   : development or test board with short FPC cable.
 * Panel    : ST7796S 480x320 landscape (MY|MX|MV|BGR = 0xE8).
 * FMC      : validated timing at 200 MHz HCLK, short FPC on test bench.
 * GPIO     : MEDIUM drive — conservative for breadboard/test wiring.
 *
 * NOT for direct #include — selected by tools/select_hw_profile.py.
 * See HARDWARE_PROFILES.md for usage.
 */

#define HW_PROFILE_NAME    "hw_test_fmc"

/* ── LCD panel ──────────────────────────────────────────────────────────────
 * 1 = ST7796S 480x320 landscape
 * 2 = ST7789V 240x320 portrait                                              */
#define HW_LCD_PANEL       1

/* MADCTL byte written to register 0x36.
 * 0xE8 = MY|MX|MV|BGR: corrects panel glass orientation, landscape scan,
 * BGR color filter.  See lcd_bus_fmc.h for full bit-field description.      */
#define HW_LCD_MADCTL      0xE8U

/* ── FMC SRAM timing (AHB clock cycles, mode A asynchronous) ───────────────
 * Limits per STM32H7 reference manual (RM0433):
 *   AddressSetupTime     : 0..15
 *   AddressHoldTime      : 1..15
 *   DataSetupTime        : 1..255
 *   BusTurnAroundDuration: 0..15
 *   CLKDivision          : 2..16
 *   DataLatency          : 2..17                                             */
#define HW_FMC_ADDR_SETUP       2U
#define HW_FMC_ADDR_HOLD        15U
#define HW_FMC_DATA_SETUP       10U
#define HW_FMC_BUS_TURN         15U
#define HW_FMC_CLK_DIV          16U
#define HW_FMC_DATA_LATENCY     17U

/* ── FMC GPIO drive strength ────────────────────────────────────────────────
 * One of: GPIO_SPEED_FREQ_LOW / MEDIUM / HIGH / VERY_HIGH                   */
#define HW_FMC_GPIO_SPEED  GPIO_SPEED_FREQ_MEDIUM

/* ── LCD DMA push chunk size (rows per async strip) ────────────────────────
 * Must divide evenly into SPEC_H (72) or be <= SPEC_H.  Valid range: 1..64. */
#define HW_DMA_CHUNK_ROWS  8U

#endif /* HW_CONFIG_ACTIVE_H */
