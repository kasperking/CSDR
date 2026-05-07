/* USER CODE BEGIN Header */
/**
  * @file  sdr_scan.c
  * @brief SWR Scan mode – blocking sweep + plot
  *
  *  Layout (Y=62..239, full 320px width):
  *   Y=62..75   Title bar  (14px)
  *   Y=76..219  SWR plot   (144px)  SWR 1.0 top → 5.0 bottom
  *   Y=220..229 Freq axis  (10px)
  *   Y=230..239 Hint/result bar (10px)
  */
/* USER CODE END Header */

#include "sdr_scan.h"
#include "csdr_app.h"
#include "si5351.h"
#include "fsdr_analog.h"
#include "sdr_ui.h"
#include "bpf_lpf.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── extern handles (defined in si5351.c / fsdr_analog.c) ── */
extern SI5351_Handle_t g_si5351;

/* ── byte-swap helper (LCD SPI is big-endian) ── */
static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

/* ── DMA scanline push (same pattern as menu.c) ── */
#define LN  ST7789_GetLineBuf()
static void push_ln(ST7789_Handle_t *lcd, uint16_t y)
{
    ST7789_PushScanline(lcd, y, LN);
    dma_wait_pub(lcd);
    cs_high_pub(lcd);
}

/* ════════════════════════════════════════════════
 *  Layout constants
 * ════════════════════════════════════════════════ */
#define SC_Y0         62U          /* top of scan zone (below topbar)    */
#define SC_TITLE_H    14U          /* title bar rows                     */
#define SC_FREQ_H     10U          /* frequency axis rows below plot     */
#define SC_HINT_H     10U          /* hint / result bar rows at bottom   */
/* Total zone ends at FTR_Y-1 (Y=62..229) so the footer is never overwritten */
#define SC_PLT_Y      (SC_Y0 + SC_TITLE_H)                                    /* 76  */
#define SC_PLT_H      (FTR_Y - SC_PLT_Y - SC_FREQ_H - SC_HINT_H)             /* 134 */
#define SC_PLT_Y2     (SC_PLT_Y + SC_PLT_H)                                   /* 210 */
#define SC_FREQ_Y     SC_PLT_Y2                                                /* 210 */
#define SC_HINT_Y     (SC_FREQ_Y + SC_FREQ_H)                                 /* 220 */
#define SC_LBL_W      26U          /* left margin for Y-axis labels      */
#define SC_PLT_X      SC_LBL_W    /* plot left edge (px)                */
#define SC_PLT_W      (LCD_W - SC_LBL_W)   /* plot width = 294px        */

/* ── SWR axis: 1.0 (top=good) → 5.0 (bottom=bad) ── */
#define SC_SWR_MIN    1.0f
#define SC_SWR_MAX    5.0f

/* ════════════════════════════════════════════════
 *  Colour palette (RGB565)
 * ════════════════════════════════════════════════ */
#define SC_BG        0x0000U   /* black background              */
#define SC_TITLE_BG  0x000FU   /* dark blue title               */
#define SC_TITLE_FG  0xFFFFU   /* white title text              */
#define SC_GRID      0x2945U   /* dark gray grid lines          */
#define SC_AXIS_FG   0x8410U   /* gray axis labels              */
#define SC_GOOD      0x07E0U   /* green  SWR < 1.5              */
#define SC_WARN      0xFFE0U   /* yellow SWR 1.5–2.0            */
#define SC_BAD       0xFD20U   /* orange SWR 2.0–3.0            */
#define SC_ALARM     0xF800U   /* red    SWR >= 3.0             */
#define SC_MIN_MRK   0x07FFU   /* cyan   minimum SWR marker     */
#define SC_PROG_BG   0x18C6U   /* progress bar track            */
#define SC_PROG_FG   0x07E0U   /* progress bar fill             */

/* ════════════════════════════════════════════════
 *  Scan result buffer (static – module-local)
 * ════════════════════════════════════════════════ */
static uint16_t s_swr[SCAN_MAX_POINTS];   /* SWR × 100 per step, 0 = unmeasured */

/* ════════════════════════════════════════════════
 *  Helpers
 * ════════════════════════════════════════════════ */
static uint16_t swr_color(uint16_t swr_x100)
{
    if (swr_x100 < 150U) return SC_GOOD;
    if (swr_x100 < 200U) return SC_WARN;
    if (swr_x100 < 300U) return SC_BAD;
    return SC_ALARM;
}

/* Map SWR value → Y pixel in plot area.  1.0=bottom, 5.0=top. */
static uint16_t swr_to_y(uint16_t swr_x100)
{
    float swr = (float)swr_x100 * 0.01f;
    if (swr < SC_SWR_MIN) swr = SC_SWR_MIN;
    if (swr > SC_SWR_MAX) swr = SC_SWR_MAX;
    float norm = (swr - SC_SWR_MIN) / (SC_SWR_MAX - SC_SWR_MIN);
    return (uint16_t)(SC_PLT_Y2 - 1U - (uint16_t)(norm * (float)(SC_PLT_H - 1U)));
}

/* ════════════════════════════════════════════════
 *  TX control (direct hardware, no side-effects on g_sdr.tx_mode)
 * ════════════════════════════════════════════════ */
static void scan_tx_on(uint32_t freq_hz)
{
    HAL_Delay(2U);
    if (g_sdr.si5351_ok)
        SI5351_SetQSEFrequency(&g_si5351, freq_hz);
    HAL_GPIO_WritePin(TR_SW_GPIO_Port, TR_SW_Pin, GPIO_PIN_SET);
}

static void scan_tx_off(uint32_t restore_rx_hz)
{
    HAL_GPIO_WritePin(TR_SW_GPIO_Port, TR_SW_Pin, GPIO_PIN_RESET);
    HAL_Delay(2U);
    if (g_sdr.si5351_ok) {
        SI5351_EnableOutput(&g_si5351, 2U, false);
        SI5351_SetQSDFrequency(&g_si5351, restore_rx_hz + g_sdr.lo_offset_hz);
    }
}

/* ════════════════════════════════════════════════
 *  scan_draw_zone – render entire scan display
 *  cur_pt : number of measured points so far
 *  npts   : total points planned
 *  done   : true = show final result, false = show progress
 * ════════════════════════════════════════════════ */
static void scan_draw_zone(ST7789_Handle_t *lcd,
                            uint32_t start_hz, uint32_t stop_hz,
                            uint32_t cur_pt, uint32_t npts, bool done)
{
    char buf[48];

    /* ── Precompute per-pixel-column SWR Y and color ── */
    static uint16_t col_y[SC_PLT_W];
    static uint16_t col_col[SC_PLT_W];

    /* Find minimum SWR among measured points */
    uint32_t min_idx = 0U;
    uint16_t min_swr = 0xFFFFU;
    for (uint32_t i = 0U; i < cur_pt; i++) {
        if (s_swr[i] != 0U && s_swr[i] < min_swr) {
            min_swr = s_swr[i];
            min_idx = i;
        }
    }

    for (uint16_t px = 0U; px < SC_PLT_W; px++) {
        if (cur_pt == 0U || npts == 0U) {
            col_y[px]   = SC_PLT_Y2;   /* off-screen = not drawn */
            col_col[px] = SC_BG;
            continue;
        }
        uint32_t pt = (npts > 1U)
            ? ((uint32_t)px * (npts - 1U) / (SC_PLT_W - 1U))
            : 0U;
        if (pt >= cur_pt || s_swr[pt] == 0U) {
            col_y[px]   = SC_PLT_Y2;
            col_col[px] = SC_BG;
        } else {
            col_y[px]   = swr_to_y(s_swr[pt]);
            col_col[px] = (done && pt == min_idx) ? SC_MIN_MRK
                                                  : swr_color(s_swr[pt]);
        }
    }

    /* ── Grid Y positions for SWR=1.5,2.0,3.0,4.0 ── */
    static const float    grid_swr[4] = {1.5f, 2.0f, 3.0f, 4.0f};
    static const char    *grid_lbl[4] = {"1.5", "2.0", "3.0", "4.0"};
    static const uint16_t grid_col[4] = {SC_WARN, SC_WARN, SC_BAD, SC_ALARM};
    uint16_t grid_y[4];
    for (int g = 0; g < 4; g++)
        grid_y[g] = swr_to_y((uint16_t)(grid_swr[g] * 100.0f + 0.5f));

    /* Y-axis label vertical start positions, centered on grid line */
    /* Index 0=1.0, 1=1.5, 2=2.0, 3=3.0, 4=4.0, 5=5.0 */
    uint16_t    lbl_y0[6];
    const char *lbl_str[6] = {"1.0", "1.5", "2.0", "3.0", "4.0", "5.0"};
    uint16_t    lbl_col[6] = {SC_GOOD, SC_WARN, SC_WARN, SC_BAD, SC_ALARM, SC_ALARM};
    uint16_t    lbl_gy[6]  = {SC_PLT_Y2 - 1U, grid_y[0], grid_y[1],
                               grid_y[2], grid_y[3], SC_PLT_Y};
    for (int l = 0; l < 6; l++) {
        int16_t y0 = (int16_t)lbl_gy[l] - (int16_t)(Font6x8.height / 2U);
        if (y0 < (int16_t)SC_PLT_Y)                         y0 = (int16_t)SC_PLT_Y;
        if (y0 + (int16_t)Font6x8.height > (int16_t)SC_PLT_Y2)
            y0 = (int16_t)SC_PLT_Y2 - (int16_t)Font6x8.height;
        lbl_y0[l] = (uint16_t)y0;
    }

    /* ══ Title bar ══ */
    {
        uint32_t ctr_hz   = (start_hz + stop_hz) / 2U;
        uint32_t ctr_mhz  = ctr_hz / 1000000U;
        uint32_t ctr_khz  = (ctr_hz % 1000000U) / 1000U;
        uint32_t half_khz = (stop_hz - start_hz) / 2000U;
        snprintf(buf, sizeof(buf), " SWR SCAN  %lu.%03lu MHz  +/-%lu kHz",
                 (unsigned long)ctr_mhz, (unsigned long)ctr_khz,
                 (unsigned long)half_khz);

        for (uint16_t fr = 0U; fr < SC_TITLE_H; fr++) {
            uint16_t *ln = LN;
            bool edge = (fr == 0U || fr == SC_TITLE_H - 1U);
            LCD_LineFill(ln, 0U, LCD_W, edge ? UI_BORDER : SC_TITLE_BG);
            if (!edge && fr >= 3U && fr < 3U + Font6x8.height) {
                LCD_LineStr(ln, 0U, fr - 3U, buf, &Font6x8, SC_TITLE_FG, SC_TITLE_BG);
            }
            push_ln(lcd, (uint16_t)(SC_Y0 + fr));
        }
    }

    /* ══ Plot area ══ */
    for (uint16_t y = SC_PLT_Y; y < SC_PLT_Y2; y++) {
        uint16_t *ln = LN;
        LCD_LineFill(ln, 0U, LCD_W, SC_BG);

        /* Y-axis labels in left margin (one label occupies Font6x8.height rows) */
        for (int l = 0; l < 6; l++) {
            if (y >= lbl_y0[l] && y < lbl_y0[l] + Font6x8.height) {
                LCD_LineStr(ln, 1U, y - lbl_y0[l],
                            lbl_str[l], &Font6x8, lbl_col[l], SC_BG);
                break;
            }
        }

        /* Grid lines (horizontal) */
        if (y == SC_PLT_Y || y == SC_PLT_Y2 - 1U) {
            LCD_LineFill(ln, SC_PLT_X, SC_PLT_W, SC_GRID);
        } else {
            for (int g = 0; g < 4; g++) {
                if (y == grid_y[g]) {
                    LCD_LineFill(ln, SC_PLT_X, SC_PLT_W, SC_GRID);
                    break;
                }
            }
        }

        /* Data: bright dot at SWR level, dim fill below */
        for (uint16_t px = 0U; px < SC_PLT_W; px++) {
            if (col_col[px] == SC_BG) continue;
            if (y == col_y[px]) {
                /* Peak pixel – bright */
                ln[SC_PLT_X + px] = sw16(col_col[px]);
            } else if (y > col_y[px]) {
                /* Below peak – quarter-brightness fill */
                uint16_t dim = (uint16_t)(((col_col[px] & 0xF7DEU) >> 1U)
                                          & 0xF7DEU) >> 1U;
                ln[SC_PLT_X + px] = sw16(dim);
            }
        }

        push_ln(lcd, y);
    }

    /* ══ Frequency axis ══ */
    {
        uint32_t sk = start_hz / 1000U;
        uint32_t ek = stop_hz  / 1000U;
        uint32_t ck = ((start_hz + stop_hz) / 2U) / 1000U;
        char s_str[14], e_str[14], c_str[14];
        snprintf(s_str, sizeof(s_str), "%lu.%03lu",
                 (unsigned long)(sk / 1000U), (unsigned long)(sk % 1000U));
        snprintf(e_str, sizeof(e_str), "%lu.%03lu",
                 (unsigned long)(ek / 1000U), (unsigned long)(ek % 1000U));
        snprintf(c_str, sizeof(c_str), "%lu.%03lu",
                 (unsigned long)(ck / 1000U), (unsigned long)(ck % 1000U));
        uint16_t e_x = (uint16_t)(LCD_W - (uint16_t)(strlen(e_str)) * Font6x8.width - 2U);
        uint16_t c_x = (uint16_t)(SC_PLT_X + SC_PLT_W / 2U
                        - (uint16_t)(strlen(c_str)) * Font6x8.width / 2U);

        for (uint16_t fr = 0U; fr < SC_FREQ_H; fr++) {
            uint16_t *ln = LN;
            LCD_LineFill(ln, 0U, LCD_W, SC_BG);
            if (fr == 0U) {
                LCD_LineFill(ln, SC_PLT_X, SC_PLT_W, SC_GRID);
            } else if (fr >= 1U && fr < 1U + Font6x8.height) {
                uint16_t frow = fr - 1U;
                LCD_LineStr(ln, SC_PLT_X, frow, s_str, &Font6x8, SC_AXIS_FG, SC_BG);
                LCD_LineStr(ln, e_x,      frow, e_str, &Font6x8, SC_AXIS_FG, SC_BG);
                LCD_LineStr(ln, c_x,      frow, c_str, &Font6x8, SC_AXIS_FG, SC_BG);
            }
            push_ln(lcd, (uint16_t)(SC_FREQ_Y + fr));
        }
    }

    /* ══ Hint / result bar ══ */
    for (uint16_t fr = 0U; fr < SC_HINT_H; fr++) {
        uint16_t *ln = LN;
        LCD_LineFill(ln, 0U, LCD_W, SC_BG);

        if (fr < Font6x8.height) {
            if (done) {
                if (min_swr != 0xFFFFU) {
                    uint32_t min_hz  = start_hz + min_idx * SCAN_STEP_HZ;
                    uint32_t min_mhz = min_hz / 1000000U;
                    uint32_t min_khz = (min_hz % 1000000U) / 1000U;
                    snprintf(buf, sizeof(buf),
                             " MIN %.2f @ %lu.%03lu MHz  [F4=EXIT]",
                             (double)min_swr * 0.01,
                             (unsigned long)min_mhz, (unsigned long)min_khz);
                } else {
                    snprintf(buf, sizeof(buf), " No data  [F4=EXIT]");
                }
                LCD_LineStr(ln, 0U, fr, buf, &Font6x8, SC_MIN_MRK, SC_BG);
            } else {
                snprintf(buf, sizeof(buf), " %lu/%lu  [F4=ABORT]",
                         (unsigned long)cur_pt, (unsigned long)npts);
                LCD_LineStr(ln, 0U, fr, buf, &Font6x8, SC_AXIS_FG, SC_BG);
                /* Progress bar on the right */
                if (npts > 0U) {
                    uint16_t bar_x = (uint16_t)(LCD_W - 152U);
                    uint16_t bar_w = (uint16_t)((uint32_t)cur_pt * 150UL / npts);
                    LCD_LineFill(ln, bar_x, 150U, SC_PROG_BG);
                    if (bar_w > 0U) LCD_LineFill(ln, bar_x, bar_w, SC_PROG_FG);
                }
            }
        }
        push_ln(lcd, (uint16_t)(SC_HINT_Y + fr));
    }
}

/* ════════════════════════════════════════════════
 *  SWR_Scan_Run – public entry point
 * ════════════════════════════════════════════════ */
void SWR_Scan_Run(ST7789_Handle_t *lcd)
{
    uint32_t center_hz = g_sdr.freq_hz;

    /* Clamp scan range to valid frequency bounds */
    uint32_t start_hz = (center_hz > SCAN_SPAN_HZ + CSDR_FREQ_MIN_HZ)
                        ? (center_hz - SCAN_SPAN_HZ) : CSDR_FREQ_MIN_HZ;
    uint32_t stop_hz  = (center_hz + SCAN_SPAN_HZ < CSDR_FREQ_MAX_HZ)
                        ? (center_hz + SCAN_SPAN_HZ) : CSDR_FREQ_MAX_HZ;

    uint32_t npts = (stop_hz - start_hz) / SCAN_STEP_HZ + 1U;
    if (npts > SCAN_MAX_POINTS) npts = SCAN_MAX_POINTS;

    memset(s_swr, 0, sizeof(s_swr));

    /* Initial display (empty plot, 0/N progress) */
    scan_draw_zone(lcd, start_hz, stop_hz, 0U, npts, false);

    bool aborted = false;

    /* ── Sweep loop ── */
    for (uint32_t i = 0U; i < npts; i++) {
        uint32_t freq = start_hz + i * SCAN_STEP_HZ;

        /* Abort: F4 held down */
        if (HAL_GPIO_ReadPin(F4_KEY_GPIO_Port, F4_KEY_Pin) == GPIO_PIN_RESET) {
            aborted = true;
            break;
        }

        /* Switch BPF + TX LPF for this frequency */
        uint8_t band = BPF_FreqToBand(freq);
        if (band != 0xFFU) {
            BPF_SetBand(band);
            LPF_SetBand(band);
        }

        /* Short TX burst */
        scan_tx_on(freq);
        HAL_Delay(SCAN_TX_SETTLE_MS);

        /* Read SWR: two updates for stability */
        Analog_Update();
        HAL_Delay(5U);
        Analog_Update();
        uint16_t swr = g_analog.swr_x100;
        if (swr < 100U) swr = 100U;   /* floor at 1.00 */
        s_swr[i] = swr;

        /* Return to RX at original frequency */
        scan_tx_off(center_hz);
        HAL_Delay(10U);

        /* Incremental plot update */
        scan_draw_zone(lcd, start_hz, stop_hz, i + 1U, npts, false);
    }

    /* Restore BPF/LPF to original band */
    BPF_SetBand(g_sdr.band_idx);
    LPF_SetBand(g_sdr.band_idx);

    /* Restore QSD (RX) frequency */
    if (g_sdr.si5351_ok)
        SI5351_SetQSDFrequency(&g_si5351, center_hz + g_sdr.lo_offset_hz);
    g_sdr.freq_hz = center_hz;

    /* Count actually measured points (needed when aborted) */
    uint32_t done_pts = npts;
    if (aborted) {
        done_pts = 0U;
        while (done_pts < npts && s_swr[done_pts] != 0U) done_pts++;
    }

    /* Final result display */
    scan_draw_zone(lcd, start_hz, stop_hz, done_pts, npts, true);

    /* Wait: release held F4 (from abort), then wait for a fresh press+release */
    while (HAL_GPIO_ReadPin(F4_KEY_GPIO_Port, F4_KEY_Pin) == GPIO_PIN_RESET)
        HAL_Delay(5U);
    HAL_Delay(60U);   /* debounce */
    while (HAL_GPIO_ReadPin(F4_KEY_GPIO_Port, F4_KEY_Pin) != GPIO_PIN_RESET)
        HAL_Delay(10U);
    while (HAL_GPIO_ReadPin(F4_KEY_GPIO_Port, F4_KEY_Pin) == GPIO_PIN_RESET)
        HAL_Delay(5U);

    /* Signal main loop to redraw everything */
    g_sdr.display_dirty = true;
}
