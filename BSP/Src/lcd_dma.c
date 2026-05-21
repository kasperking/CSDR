/* USER CODE BEGIN Header */
/**
 * @file  lcd_dma.c
 * @brief Asynchronous LCD DMA flush – DMA2 Stream0, M2M byte-to-FMC.
 *
 *  Pixel buffers stored in .DMA_SRAM (RAM_D1) use the SWAP16 convention:
 *  for pixel RGB565 value P, the stored uint16_t is SWAP16(P) = (P>>8)|(P<<8).
 *  In little-endian memory the low byte of SWAP16(P) is the original MSB of P,
 *  which the ST7796 8080 bus must receive first.  When DMA reads the buffer
 *  byte-by-byte in address order it naturally produces the correct wire sequence,
 *  identical to what the synchronous LCD_PushWindow loop does.
 */
/* USER CODE END Header */

#include "lcd_dma.h"
#include "lcd_bus_fmc.h"
#include "core_cm7.h"   /* DWT->CYCCNT */
#include "main.h"       /* SystemCoreClock */

/* ── DMA handle and state ────────────────────────────────────────────────── */

/* DMA2 Stream0: M2M, intentionally MEDIUM priority.
 * Audio DMA (DMA1 Stream0/1) is VERY_HIGH; USB is preempt-priority 2.
 * LCD pixels are cosmetic — MEDIUM ensures audio and USB always win
 * AHB bus arbitration during simultaneous traffic bursts. */
static DMA_HandleTypeDef s_hdma_lcd;

/* s_dma_busy: written in ISR (TC/TE callbacks), read in main thread.
 * volatile prevents compiler from caching the value in a register across
 * the LCD_Wait() poll loop. */
static volatile bool     s_dma_busy     = false;
static volatile uint32_t s_cyc_start    = 0U;
static volatile uint32_t s_max_lat_us   = 0U;
static volatile uint32_t s_queued_count = 0U;

/* ── Transfer-Complete callback ──────────────────────────────────────────── */

static void lcd_dma_tc_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    uint32_t mhz = SystemCoreClock / 1000000U;
    uint32_t elapsed = (mhz > 0U) ? ((DWT->CYCCNT - s_cyc_start) / mhz) : 0U;
    if (elapsed > s_max_lat_us) s_max_lat_us = elapsed;
    s_dma_busy = false;
}

/* ── Transfer-Error callback ─────────────────────────────────────────────── */

/* Release the busy lock on DMA error so subsequent pushes are not blocked. */
static void lcd_dma_err_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    s_dma_busy = false;
}

/* ── LCD_DMA_Init ─────────────────────────────────────────────────────────── */

void LCD_DMA_Init(void)
{
    /* DMA2 clock: DMA1 is already enabled in MX_DMA_Init for audio streams. */
    __HAL_RCC_DMA2_CLK_ENABLE();

    s_hdma_lcd.Instance = DMA2_Stream0;

    /* M2M with software-trigger (DMAMUX request 0 = no HW trigger). */
    s_hdma_lcd.Init.Request             = DMA_REQUEST_MEM2MEM;
    s_hdma_lcd.Init.Direction           = DMA_MEMORY_TO_MEMORY;

    /* Source (pixel buffer): address increments byte-by-byte through the strip. */
    s_hdma_lcd.Init.PeriphInc           = DMA_PINC_ENABLE;
    /* Destination (FMC data register): fixed address, every byte goes here. */
    s_hdma_lcd.Init.MemInc              = DMA_MINC_DISABLE;

    s_hdma_lcd.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_lcd.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    s_hdma_lcd.Init.Mode                = DMA_NORMAL;

    /* MEDIUM priority: LCD DMA yields to audio (VERY_HIGH) and USB (priority 2)
     * on the AHB bus.  A brief bus stall only delays pixel output, never audio. */
    s_hdma_lcd.Init.Priority            = DMA_PRIORITY_MEDIUM;

    /* FIFO required for M2M on STM32H7 DMA1/DMA2.  FULL threshold amortises
     * AHB overhead; SINGLE burst keeps each FMC destination write to one byte. */
    s_hdma_lcd.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    s_hdma_lcd.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    s_hdma_lcd.Init.MemBurst            = DMA_MBURST_SINGLE;
    s_hdma_lcd.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    HAL_DMA_Init(&s_hdma_lcd);
    HAL_DMA_RegisterCallback(&s_hdma_lcd, HAL_DMA_XFER_CPLT_CB_ID, lcd_dma_tc_callback);
    HAL_DMA_RegisterCallback(&s_hdma_lcd, HAL_DMA_XFER_ERROR_CB_ID, lcd_dma_err_callback);

    /* Priority 5: TC callback only clears one flag.  Well below audio (0) and
     * USB (2); well above SysTick (15) so the flag is cleared promptly. */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* ── LCD_DMA_IRQHandler ──────────────────────────────────────────────────── */

void LCD_DMA_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_lcd);
}

/* ── LCD_IsBusy ──────────────────────────────────────────────────────────── */

bool LCD_IsBusy(void)
{
    return s_dma_busy;
}

/* ── LCD_Wait ────────────────────────────────────────────────────────────── */

void LCD_Wait(void)
{
    /* Cooperative poll: cheap NOP loop.  Audio and USB ISRs preempt freely at
     * their higher priorities throughout the wait.  Typical duration:
     *   Waterfall row  : ≤ 112 µs  (480 px × 2 B × 117 ns)
     *   Spectrum strip : ≤ 900 µs  (8 rows × 480 px × 2 B × 117 ns) */
    while (s_dma_busy) { __NOP(); }
}

/* ── LCD_PushWindowAsync ─────────────────────────────────────────────────── */

bool LCD_PushWindowAsync(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                         const void *buf, uint32_t len)
{
    if (len == 0U)   return false;
    if (s_dma_busy)  return false;  /* caller must LCD_Wait() before calling */

    /* Window setup: CASET/RASET/RAMWR via synchronous CPU FMC writes (~1 µs).
     * DMA then delivers only the pixel payload bytes to the data address.
     * The ST7796 holds the RAMWR state until a new command is issued, so
     * the gap between RAMWR and the first DMA byte is safe. */
    LCD_SetWindow(x0, y0, x1, y1);

    s_dma_busy    = true;
    s_cyc_start   = DWT->CYCCNT;
    s_queued_count++;

    /* SrcAddress  = pixel buffer, bytes in FMC wire order (SWAP16 layout).
     * DstAddress  = LCD FMC data register: fixed, A16=1, RS/DC HIGH.
     * DataLength  = bytes to transfer (npixels × 2).
     * PeriphInc=ENABLE  → source address advances each beat.
     * MemInc=DISABLE    → destination address stays at LCD_FMC_DATA_ADDR. */
    HAL_StatusTypeDef st = HAL_DMA_Start_IT(
        &s_hdma_lcd,
        (uint32_t)buf,
        (uint32_t)LCD_FMC_DATA_ADDR,
        len
    );

    if (st != HAL_OK) {
        s_dma_busy = false;
        return false;
    }
    return true;
}

/* ── Diagnostics ─────────────────────────────────────────────────────────── */

uint32_t LCD_DMA_GetMaxLatencyUs(void) { return s_max_lat_us;    }
uint32_t LCD_DMA_GetQueuedCount(void)  { return s_queued_count;  }
