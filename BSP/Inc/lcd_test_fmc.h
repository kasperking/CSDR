/* USER CODE BEGIN Header */
/**
 * @file  lcd_test_fmc.h
 * @brief ST7796S FMC LCD validation tests and throughput benchmarks.
 *
 *  Call LCD_FMC_RunTest() once after LCD_Bus_Init() to:
 *    – verify RGB565 color mapping (correct R/G/B channels)
 *    – validate landscape orientation (origin, axis directions)
 *    – confirm command/data bus separation (no corrupt pixels)
 *    – animate a moving rectangle (validates partial window writes)
 *    – measure fill throughput (ms/frame and FPS)
 *
 *  Results land in g_lcd_fmc_bench.  Inspect via debugger Live Expressions.
 *  All tests are blocking; no DMA, no RTOS dependency.
 */
/* USER CODE END Header */

#ifndef LCD_TEST_FMC_H
#define LCD_TEST_FMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Benchmark result record ─────────────────────────────────────────────── */
typedef struct {
    uint32_t full_clear_ms;       /* HAL_GetTick(): black screen fill time   */
    uint32_t full_fill_ms[3];     /* fill with red, green, blue              */
    uint32_t rect_100x100_us;     /* 100×100 single fill in microseconds     */
    uint32_t fps_estimate;        /* theoretical max FPS from full_clear_ms  */
    uint32_t bytes_per_sec;       /* raw write throughput estimate           */
    uint32_t frame_bytes;         /* 480×320×2 = 307200                      */
} LcdFmcBenchResult_t;

extern LcdFmcBenchResult_t g_lcd_fmc_bench;

/* ── API ─────────────────────────────────────────────────────────────────── */

/* Full test sequence: orientation → RGB565 → solid fills → moving rect →
 *                     random tiles → benchmarks                            */
void LCD_FMC_RunTest(void);

/* Individual stages (call separately during debug) */
void LCD_FMC_TestOrientation(void);     /* corner markers: verify X/Y/RGB    */
void LCD_FMC_TestRGB565Colors(void);    /* 8-stripe palette: verify channels  */
void LCD_FMC_TestSolidColors(void);     /* full-screen single-color cycles    */
void LCD_FMC_TestMovingRect(void);      /* animated sweep: verify window regs */
void LCD_FMC_TestRandomColors(void);    /* random 40×40 tiles: bus stability  */
void LCD_FMC_RunBenchmarks(void);       /* fill timing, FPS, throughput       */

#ifdef __cplusplus
}
#endif

#endif /* LCD_TEST_FMC_H */
