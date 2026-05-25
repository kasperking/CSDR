/*
 * hw_long_fpc_debug.h — Hardware profile: long FPC extension cable, relaxed FMC.
 *
 * Target   : scope-probing / debugging with a >100 mm FPC extension cable.
 * Panel    : ST7796S 480x320 landscape.
 * FMC      : relaxed timing (+2 cycles on address/data setup) to absorb cable
 *            propagation delay and capacitive loading.
 * GPIO     : MEDIUM drive — reduces ringing on long, unmatched FPC extension.
 *
 * NOT for direct #include — selected by tools/select_hw_profile.py.
 * See HARDWARE_PROFILES.md for usage.
 */

#define HW_PROFILE_NAME    "hw_long_fpc_debug"

/* ── LCD panel ──────────────────────────────────────────────────────────────
 * 1 = ST7796S 480x320 landscape
 * 2 = ST7789V 240x320 portrait                                              */
#define HW_LCD_PANEL       1

/* MY|MX|MV|BGR landscape.                                                   */
#define HW_LCD_MADCTL      0xE8U

/* ── FMC SRAM timing ────────────────────────────────────────────────────────
 * AddressSetup=4, DataSetup=14: +2 cycles vs standard to cover cable delay.
 * BusTurnAround=15: maximum hold, avoids bus contention on long cable.      */
#define HW_FMC_ADDR_SETUP       4U
#define HW_FMC_ADDR_HOLD        15U
#define HW_FMC_DATA_SETUP       14U
#define HW_FMC_BUS_TURN         15U
#define HW_FMC_CLK_DIV          16U
#define HW_FMC_DATA_LATENCY     17U

/* ── FMC GPIO drive strength ────────────────────────────────────────────── */
#define HW_FMC_GPIO_SPEED  GPIO_SPEED_FREQ_MEDIUM

/* ── LCD DMA push chunk size ────────────────────────────────────────────── */
#define HW_DMA_CHUNK_ROWS  8U
