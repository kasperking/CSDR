/* USER CODE BEGIN Header */
/**
 * @file  lcd_test_fmc.c
 * @brief ST7796S FMC LCD validation tests and throughput benchmarks.
 *
 *  Validates:
 *    ✓ Landscape orientation (origin top-left, X right, Y down)
 *    ✓ RGB565 color mapping (correct R/G/B channel order)
 *    ✓ FMC command/data address separation (RS/DC via A16)
 *    ✓ Partial window writes (CASET/RASET register correctness)
 *    ✓ Write throughput (bytes/s and FPS)
 *
 *  Timing:
 *    – HAL_GetTick()  → ms resolution, full-screen fills
 *    – DWT CYCCNT     → ~2 ns at 480 MHz, small-rect microsecond timing
 *
 *  Results: g_lcd_fmc_bench — inspect via debugger Live Expressions.
 */
/* USER CODE END Header */

#include "lcd_test_fmc.h"
#include "lcd_bus_fmc.h"
#include "stm32h7xx_hal.h"

/* ── DWT cycle counter ───────────────────────────────────────────────────── */

#define DWT_CPU_MHZ  480U   /* adjust if CPU clock differs */

static void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0U;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t dwt_us(uint32_t start_cycles)
{
    return (DWT->CYCCNT - start_cycles) / DWT_CPU_MHZ;
}

/* ── Global result store ─────────────────────────────────────────────────── */
LcdFmcBenchResult_t g_lcd_fmc_bench = {0};

/* ── Colour palettes ─────────────────────────────────────────────────────── */

/* Full-colour palette for solid-fill and stripe tests (RGB565) */
static const uint16_t k_palette[] = {
    0xF800U,  /* red     */
    0x07E0U,  /* green   */
    0x001FU,  /* blue    */
    0xFFE0U,  /* yellow  */
    0xF81FU,  /* magenta */
    0x07FFU,  /* cyan    */
    0xFFFFU,  /* white   */
    0x0000U,  /* black   */
};
#define N_PALETTE  (sizeof(k_palette) / sizeof(k_palette[0]))

/* Pseudo-random for random-tile test */
static uint32_t s_rng = 0xDEADBEEFUL;

static uint16_t rand_color(void)
{
    s_rng ^= s_rng << 13U;
    s_rng ^= s_rng >> 17U;
    s_rng ^= s_rng << 5U;
    return (uint16_t)(s_rng & 0xFFFFU);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Orientation test
 *
 *  Draws unambiguous corner markers on a black background.
 *  Expected result if orientation and RGB order are correct:
 *    Top-left  corner (8×8):  RED
 *    Top-right corner (8×8):  GREEN
 *    Bottom-left  (8×8):      BLUE
 *    Bottom-right (8×8):      YELLOW
 *    Centre crosshair (3 px): WHITE
 *
 *  Fault interpretation:
 *    • X/Y swapped     → portrait image on landscape screen (MADCTL MV wrong)
 *    • Mirrored X      → red/green and blue/yellow swap sides (MADCTL MX wrong)
 *    • Mirrored Y      → top/bottom markers swap (MADCTL MY wrong)
 *    • R/B channels swapped → red corner appears blue (change MADCTL BGR bit:
 *                             0x60 → 0x68)
 * ══════════════════════════════════════════════════════════════════════════ */

void LCD_FMC_TestOrientation(void)
{
    const uint16_t M = 16U;   /* marker size in pixels */

    LCD_Clear(0x0000U);   /* black background */

    /* Top-left: RED */
    LCD_FillRect(0U, 0U, M - 1U, M - 1U, 0xF800U);

    /* Top-right: GREEN */
    LCD_FillRect(LCD_W - M, 0U, LCD_W - 1U, M - 1U, 0x07E0U);

    /* Bottom-left: BLUE */
    LCD_FillRect(0U, LCD_H - M, M - 1U, LCD_H - 1U, 0x001FU);

    /* Bottom-right: YELLOW */
    LCD_FillRect(LCD_W - M, LCD_H - M, LCD_W - 1U, LCD_H - 1U, 0xFFE0U);

    /* Centre crosshair: WHITE (3 px thick, 40 px long) */
    uint16_t cx = LCD_W / 2U;
    uint16_t cy = LCD_H / 2U;
    LCD_FillRect(cx - 1U, cy - 20U, cx + 1U, cy + 20U, 0xFFFFU);  /* vertical */
    LCD_FillRect(cx - 20U, cy - 1U, cx + 20U, cy + 1U, 0xFFFFU);  /* horizontal */

    HAL_Delay(3000U);   /* hold 3 s for visual inspection */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  RGB565 stripe test
 *
 *  Divides the screen into 8 equal vertical stripes, one per palette colour.
 *  Verifies that the controller receives correct 16-bit colour data and that
 *  R/G/B channels map to the expected display colours.
 *
 *  Stripe order (L→R): red | green | blue | yellow | magenta | cyan | white | black
 * ══════════════════════════════════════════════════════════════════════════ */

void LCD_FMC_TestRGB565Colors(void)
{
    const uint16_t stripe_w = LCD_W / (uint16_t)N_PALETTE;

    for (uint32_t i = 0U; i < N_PALETTE; i++) {
        uint16_t x0 = (uint16_t)(i * stripe_w);
        uint16_t x1 = (i == N_PALETTE - 1U) ? (LCD_W - 1U)
                                             : (uint16_t)(x0 + stripe_w - 1U);
        LCD_FillRect(x0, 0U, x1, LCD_H - 1U, k_palette[i]);
    }

    HAL_Delay(2000U);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Solid colour cycle
 * ══════════════════════════════════════════════════════════════════════════ */

void LCD_FMC_TestSolidColors(void)
{
    for (uint32_t i = 0U; i < N_PALETTE; i++) {
        LCD_Clear(k_palette[i]);
        HAL_Delay(400U);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Moving rectangle animation
 *
 *  A 60×80 yellow rectangle sweeps left→right and back on a dark-blue
 *  background.  Validates that LCD_SetWindow() correctly programs CASET/RASET
 *  for partial updates at every horizontal position.
 * ══════════════════════════════════════════════════════════════════════════ */

void LCD_FMC_TestMovingRect(void)
{
    const uint16_t RW  = 60U;
    const uint16_t RH  = 80U;
    const uint16_t Y0  = (LCD_H - RH) / 2U;
    const uint16_t Y1  = Y0 + RH - 1U;
    const uint16_t BG  = 0x000FU;   /* dark blue  */
    const uint16_t FG  = 0xFFE0U;   /* yellow     */

    LCD_Clear(BG);

    /* Sweep left → right */
    for (uint16_t x = 0U; x + RW < LCD_W; x += 4U) {
        if (x > 0U) {
            LCD_FillRect(x - 4U, Y0, x - 1U, Y1, BG);
        }
        LCD_FillRect(x, Y0, x + RW - 1U, Y1, FG);
        HAL_Delay(12U);
    }

    /* Sweep right → left */
    for (int16_t x = (int16_t)(LCD_W - RW); x >= 0; x -= 4) {
        LCD_FillRect((uint16_t)(x + RW - 4U), Y0,
                     (uint16_t)(x + RW - 1U), Y1, BG);
        LCD_FillRect((uint16_t)x, Y0, (uint16_t)(x + RW - 1U), Y1, FG);
        HAL_Delay(12U);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Random colour tile test
 *
 *  Fills 40×40 tiles with random RGB565 values.  Any FMC timing corruption
 *  appears as off-colour or partial tiles.  480×320 gives 12×8 = 96 tiles.
 * ══════════════════════════════════════════════════════════════════════════ */

void LCD_FMC_TestRandomColors(void)
{
    const uint16_t TW = 40U;
    const uint16_t TH = 40U;

    for (uint16_t y = 0U; y + TH <= LCD_H; y += TH) {
        for (uint16_t x = 0U; x + TW <= LCD_W; x += TW) {
            LCD_FillRect(x, y, x + TW - 1U, y + TH - 1U, rand_color());
        }
    }
    HAL_Delay(1000U);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Throughput benchmarks
 *
 *  g_lcd_fmc_bench fields after this function:
 *    full_clear_ms     — time to fill 480×320 with one colour
 *    fps_estimate      — 1000 / full_clear_ms
 *    bytes_per_sec     — frame_bytes * 1000 / full_clear_ms
 *    full_fill_ms[0-2] — fill time for red, green, blue
 *    rect_100x100_us   — DWT-measured 100×100 fill in µs
 *    frame_bytes       — 480×320×2 = 307200
 * ══════════════════════════════════════════════════════════════════════════ */

void LCD_FMC_RunBenchmarks(void)
{
    LcdFmcBenchResult_t *r = &g_lcd_fmc_bench;
    uint32_t t0, t1, cyc0;

    dwt_enable();

    r->frame_bytes = (uint32_t)LCD_W * (uint32_t)LCD_H * 2UL;   /* 307200 */

    /* ── 1. Full-screen clear (black) ── */
    t0 = HAL_GetTick();
    LCD_Clear(0x0000U);
    t1 = HAL_GetTick();
    r->full_clear_ms = t1 - t0;
    r->fps_estimate  = (r->full_clear_ms > 0U) ? (1000U / r->full_clear_ms) : 0U;
    r->bytes_per_sec = (r->full_clear_ms > 0U)
                     ? (r->frame_bytes * 1000U / r->full_clear_ms) : 0U;

    /* ── 2. Full fills: red, green, blue ── */
    const uint16_t bench_colors[3] = {0xF800U, 0x07E0U, 0x001FU};
    for (uint32_t i = 0U; i < 3U; i++) {
        t0 = HAL_GetTick();
        LCD_Clear(bench_colors[i]);
        t1 = HAL_GetTick();
        r->full_fill_ms[i] = t1 - t0;
        HAL_Delay(200U);
    }

    /* ── 3. 100×100 fill: DWT microsecond measurement ── */
    /* Centre the rect: (480-100)/2=190, (320-100)/2=110 */
    LCD_Clear(0x0000U);
    cyc0 = DWT->CYCCNT;
    LCD_FillRect(190U, 110U, 289U, 209U, 0xFFFFU);   /* 100×100 white */
    r->rect_100x100_us = dwt_us(cyc0);

    HAL_Delay(1500U);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Master test entry
 * ══════════════════════════════════════════════════════════════════════════ */

void LCD_FMC_RunTest(void)
{
    /* Stage 1: orientation markers — inspect visually before other tests */
    LCD_FMC_TestOrientation();

    /* Stage 2: 8-colour RGB565 stripes — verify channel mapping */
    LCD_FMC_TestRGB565Colors();

    /* Stage 3: solid colour cycle */
    LCD_FMC_TestSolidColors();

    /* Stage 4: moving rectangle — partial window validation */
    LCD_FMC_TestMovingRect();

    /* Stage 5: random tile fill — bus stability check */
    LCD_FMC_TestRandomColors();

    /* Stage 6: throughput benchmarks — results in g_lcd_fmc_bench */
    LCD_FMC_RunBenchmarks();

    /* Final: white screen — confirms test completed cleanly */
    LCD_Clear(0xFFFFU);
}
