/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi_assets.c
  * @brief   SPI flash asset loader implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "spi_assets.h"
#include <string.h>

/* ── RAM buffers (AXI-SRAM, .bss — H750 has 512 KB) ────────────────────── */
static uint8_t   s_font_buf   [SPI_FONT_BLOB_SIZE]   __attribute__((aligned(4)));
static float     s_fft_twiddle[SPI_FFT_TWIDDLE_LEN]  __attribute__((aligned(4)));
static uint16_t  s_fft_bitrev [SPI_FFT_BITREV_LEN]   __attribute__((aligned(4)));

/* ── Asset registry ─────────────────────────────────────────────────────── */
typedef struct {
    uint32_t  flash_addr;
    uint32_t  byte_size;
    void     *ram_buf;
    bool      loaded;
} AssetEntry_t;

static AssetEntry_t s_assets[SPI_ASSET_COUNT] = {
    [SPI_ASSET_FONTS]       = { FLASH_ADDR_FONT_DATA,   SPI_FONT_BLOB_SIZE,   (void *)s_font_buf,    false },
    [SPI_ASSET_FFT_TWIDDLE] = { FLASH_ADDR_FFT_TWIDDLE, SPI_FFT_TWIDDLE_SIZE, (void *)s_fft_twiddle, false },
    [SPI_ASSET_FFT_BITREV]  = { FLASH_ADDR_FFT_BITREV,  SPI_FFT_BITREV_SIZE,  (void *)s_fft_bitrev,  false },
};

/* ── Debug diagnostics (inspect in debugger) ────────────────────────────── */
uint32_t dbg_spi_assets_loaded;                  /* bitmask: bit N = asset N loaded */
uint32_t dbg_spi_assets_probe[SPI_ASSET_COUNT];  /* raw probe word read per sector  */

/* ── Internal: paged read ───────────────────────────────────────────────── *
 * W25Q_Read uses HAL_SPI_Receive with a fixed 100 ms timeout.  A single     *
 * call for 4096 bytes would timeout at SPI clocks below ~320 KHz.           *
 * Splitting into 256-byte pages keeps each transaction ≤ 2 ms at 1 MHz.    */
static HAL_StatusTypeDef asset_read(W25Q_Handle_t *dev, uint32_t addr,
                                     uint8_t *buf,  uint32_t size)
{
    while (size > 0U) {
        uint32_t chunk = (size > W25Q_PAGE_SIZE) ? (uint32_t)W25Q_PAGE_SIZE : size;
        if (W25Q_Read(dev, addr, buf, chunk) != HAL_OK) return HAL_ERROR;
        addr += chunk;
        buf  += chunk;
        size -= chunk;
    }
    return HAL_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef SPI_Assets_LoadAll(W25Q_Handle_t *dev)
{
    if (!dev || !dev->present) return HAL_ERROR;

    dbg_spi_assets_loaded = 0U;

    HAL_StatusTypeDef overall = HAL_OK;
    for (uint8_t i = 0U; i < (uint8_t)SPI_ASSET_COUNT; i++) {
        AssetEntry_t *a = &s_assets[i];

        /* Probe: read first 4 bytes.  All 0xFF = sector blank (never written). */
        uint32_t probe = 0U;
        if (W25Q_Read(dev, a->flash_addr, (uint8_t *)&probe, 4U) != HAL_OK) {
            dbg_spi_assets_probe[i] = 0xDEAD0000UL | i;
            overall = HAL_ERROR;
            continue;
        }
        dbg_spi_assets_probe[i] = probe;

        if (probe == 0xFFFFFFFFUL) continue;   /* sector blank → skip */

        if (asset_read(dev, a->flash_addr, (uint8_t *)a->ram_buf, a->byte_size)
                == HAL_OK) {
            a->loaded = true;
            dbg_spi_assets_loaded |= (1UL << i);
        } else {
            overall = HAL_ERROR;
        }
    }
    return overall;
}

bool SPI_Assets_IsLoaded(SpiAssetId_t id)
{
    return ((uint8_t)id < (uint8_t)SPI_ASSET_COUNT) && s_assets[id].loaded;
}

void *SPI_Assets_GetBuf(SpiAssetId_t id)
{
    return ((uint8_t)id < (uint8_t)SPI_ASSET_COUNT && s_assets[id].loaded)
           ? s_assets[id].ram_buf : NULL;
}
