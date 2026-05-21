/* USER CODE BEGIN Header */
/**
 * @file  lcd_dma.h
 * @brief Asynchronous LCD DMA flush layer – DMA2 Stream0, M2M byte-to-FMC.
 *
 *  CPU performs CASET/RASET/RAMWR window setup synchronously (fast FMC command
 *  writes).  DMA2_Stream0 then delivers the pixel payload byte-by-byte to the
 *  fixed LCD FMC data address without further CPU involvement.
 *
 *  Why DMA priority is MEDIUM:
 *    Audio DMA (DMA1_Stream0/1) is VERY_HIGH.  USB IRQ is preempt-priority 2.
 *    LCD pixels are cosmetic — a missed strip at most causes a one-frame
 *    artifact, never data loss.  MEDIUM ensures audio and USB always win
 *    AHB bus arbitration during simultaneous traffic.
 *
 *  Why chunked (8-row strips, not full-frame DMA):
 *    Each strip is ~900 µs.  Checking RuntimeDiag_IsUiOverload() between
 *    strips lets the waterfall abort early under load.  It also caps the
 *    maximum time audio ISR latency can compound: a single full-frame DMA
 *    (8 ms) is replaced by nine independently-abort-able 900 µs strips.
 *
 *  Cache coherency:
 *    Source buffers are in .DMA_SRAM (RAM_D1, 0x24000000), covered by MPU
 *    Region 0 as Non-Cacheable/Non-Bufferable (TEX=1,C=0,B=0).  No
 *    SCB_CleanDCache() is needed before starting DMA.
 *    The FMC LCD address (0x60010000) is Strongly-Ordered (MPU Region 1);
 *    DMA bypasses the MPU and writes directly via the AHB bus matrix.
 */
/* USER CODE END Header */

#ifndef LCD_DMA_H
#define LCD_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/* ── Init ────────────────────────────────────────────────────────────────── */

/* Configure DMA2_Stream0 for M2M BYTE transfers.
 * Call once at startup, after LCD_Bus_Init() (inside MX_FMC_Init USER CODE). */
void LCD_DMA_Init(void);

/* ── IRQ entry point ─────────────────────────────────────────────────────── */

/* Forward HAL IRQ handling from DMA2_Stream0_IRQHandler in stm32h7xx_it.c. */
void LCD_DMA_IRQHandler(void);

/* ── State query ─────────────────────────────────────────────────────────── */

bool LCD_IsBusy(void);

/* ── Cooperative wait ────────────────────────────────────────────────────── */

/* Poll s_dma_busy in a cheap NOP loop.  Audio/USB ISRs remain live.
 * Typical wait: ≤ 900 µs per 8-row strip, ≤ 112 µs per waterfall row. */
void LCD_Wait(void);

/* ── Async pixel payload push ────────────────────────────────────────────── */

/* Set CASET/RASET/RAMWR window (CPU), then start DMA for `len` bytes of
 * SWAP16-encoded pixel data from `buf` to the fixed FMC data address.
 *
 * `len` is in BYTES (npixels × 2).  The buffer must remain valid until
 * LCD_IsBusy() returns false or LCD_Wait() returns.
 *
 * Returns false if the DMA is still busy (call LCD_Wait first) or if the
 * HAL start fails.  On false the window has already been set — do not
 * issue another window command without an LCD_Wait. */
bool LCD_PushWindowAsync(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                         const void *buf, uint32_t len);

/* ── Diagnostics ─────────────────────────────────────────────────────────── */

uint32_t LCD_DMA_GetMaxLatencyUs(void); /*!< Peak µs DMA start → TC callback  */
uint32_t LCD_DMA_GetQueuedCount(void);  /*!< Total DMA chunk launches since boot */

#ifdef __cplusplus
}
#endif

#endif /* LCD_DMA_H */
