/*
 * hw_prod_v1.h — Hardware profile: production board v1, ST7796 480x320.
 *
 * Target   : production PCB with short, impedance-controlled FPC to ST7796.
 * Panel    : ST7796S 480x320 landscape (same MADCTL as test board).
 * FMC      : tighter read timing (DataSetup=8 vs 10) — validated on prod PCB.
 * GPIO     : VERY_HIGH drive — production board has matched-length FMC traces.
 *
 * NOT for direct #include — selected by tools/select_hw_profile.py.
 * See HARDWARE_PROFILES.md for usage.
 */

#define HW_PROFILE_NAME    "hw_prod_v1"

/* ── LCD panel ──────────────────────────────────────────────────────────────
 * 1 = ST7796S 480x320 landscape
 * 2 = ST7789V 240x320 portrait                                              */
#define HW_LCD_PANEL       1

/* MY|MX|MV|BGR landscape — same orientation as test board.                  */
#define HW_LCD_MADCTL      0xE8U

/* ── FMC SRAM timing ────────────────────────────────────────────────────────
 * AddressSetup reduced to 1, DataSetup reduced to 8 vs test board.
 * Validated at 200 MHz HCLK on production PCB with 50 mm FPC.              */
#define HW_FMC_ADDR_SETUP       1U
#define HW_FMC_ADDR_HOLD        15U
#define HW_FMC_DATA_SETUP       8U
#define HW_FMC_BUS_TURN         15U
#define HW_FMC_CLK_DIV          16U
#define HW_FMC_DATA_LATENCY     17U

/* ── FMC GPIO drive strength ────────────────────────────────────────────────
 * VERY_HIGH: production PCB has controlled-impedance FMC traces.
 * Do NOT use on test bench — ringing on unmatched wiring causes glitches.   */
#define HW_FMC_GPIO_SPEED  GPIO_SPEED_FREQ_VERY_HIGH

/* ── LCD DMA push chunk size ────────────────────────────────────────────── */
#define HW_DMA_CHUNK_ROWS  8U
