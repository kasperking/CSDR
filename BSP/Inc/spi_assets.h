/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi_assets.h
  * @brief   SPI flash asset loader — moves large const tables from internal
  *          flash to W25Q SPI NOR, loading them into AXI-SRAM at boot.
  *
  *  Asset map (mirrors FLASH_ADDR_* in w25q128.h):
  *   SPI_ASSET_FONTS       @ 0x02C000  ~1652 B  Font6x8 + Font5x8 + Font8x10
  *   SPI_ASSET_FFT_TWIDDLE @ 0x02D000  4096 B   twiddleCoef_512[1024]
  *   SPI_ASSET_FFT_BITREV  @ 0x02E000   896 B   armBitRevIndexTable512[448]
  *
  *  Fallback flags (set to 0 once SPI flash is programmed to reclaim space):
  *   SPI_ASSETS_FONT_FALLBACK  — keep font const arrays in internal flash
  *   SPI_ASSETS_FFT_FALLBACK   — keep arm_cfft_sR_f32_len512 in internal flash
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SPI_ASSETS_H
#define __SPI_ASSETS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "w25q128.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Asset IDs ─────────────────────────────────────────────────────────── */
typedef enum {
    SPI_ASSET_FONTS       = 0,   /* Font6x8 + Font5x8 + Font8x10 bitmaps */
    SPI_ASSET_FFT_TWIDDLE = 1,   /* twiddleCoef_512[1024] float32         */
    SPI_ASSET_FFT_BITREV  = 2,   /* armBitRevIndexTable512[448] uint16_t  */
    SPI_ASSET_COUNT
} SpiAssetId_t;

/* ── Font blob layout ───────────────────────────────────────────────────── *
 * Raw concatenation stored at FLASH_ADDR_FONT_DATA:                        *
 *   [Font6x8 : 354 B][Font5x8 : 354 B][Font8x10 : 944 B]  = 1652 B total  */
#define SPI_FONT_GLYPH_COUNT   59U                                   /* ASCII 32..90 */
#define SPI_FONT_F6X8_SIZE     (SPI_FONT_GLYPH_COUNT * 6U)          /* 354 bytes  */
#define SPI_FONT_F5X8_SIZE     (SPI_FONT_GLYPH_COUNT * 6U)          /* 354 bytes  */
#define SPI_FONT_F8X10_SIZE    (SPI_FONT_GLYPH_COUNT * 8U * 2U)     /* 944 bytes  */
#define SPI_FONT_BLOB_SIZE     (SPI_FONT_F6X8_SIZE + SPI_FONT_F5X8_SIZE + SPI_FONT_F8X10_SIZE)
#define SPI_FONT_F6X8_OFFSET   0U
#define SPI_FONT_F5X8_OFFSET   SPI_FONT_F6X8_SIZE
#define SPI_FONT_F8X10_OFFSET  (SPI_FONT_F6X8_SIZE + SPI_FONT_F5X8_SIZE)

/* ── FFT table sizes ────────────────────────────────────────────────────── */
#define SPI_FFT_TWIDDLE_LEN    1024U
#define SPI_FFT_BITREV_LEN     448U
#define SPI_FFT_TWIDDLE_SIZE   (SPI_FFT_TWIDDLE_LEN * 4U)  /* 4096 bytes */
#define SPI_FFT_BITREV_SIZE    (SPI_FFT_BITREV_LEN  * 2U)  /*  896 bytes */

/* ── Fallback compile-time gates ────────────────────────────────────────── *
 * 1 = keep embedded fallback in internal flash (safe while SPI not yet     *
 *     programmed).  Set to 0 after write-assets to reclaim ~6.7 KB.        */
#ifndef SPI_ASSETS_FONT_FALLBACK
#  define SPI_ASSETS_FONT_FALLBACK  1
#endif
#ifndef SPI_ASSETS_FFT_FALLBACK
#  define SPI_ASSETS_FFT_FALLBACK   1
#endif

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * Load all assets from SPI flash into RAM buffers.
 * Skips sectors whose first 4 bytes are 0xFFFFFFFF (not yet programmed).
 * Call after W25Q_Init(), before DSP_Init() and LCD_Render_Init().
 */
HAL_StatusTypeDef SPI_Assets_LoadAll(W25Q_Handle_t *dev);

/** Returns true if asset was successfully loaded from SPI flash. */
bool              SPI_Assets_IsLoaded(SpiAssetId_t id);

/** Returns pointer to RAM buffer for a loaded asset, NULL if not loaded. */
void             *SPI_Assets_GetBuf(SpiAssetId_t id);

#ifdef __cplusplus
}
#endif
#endif /* __SPI_ASSETS_H */
