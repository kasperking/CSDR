/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    w25q128.h
  * @brief   W25Q128JV SPI NOR Flash BSP Driver
  *
  *  Hardware: SPI3  (PC10=SCK, PC11=MISO, PC12=MOSI, PD0=CS)
  *  Capacity: 128Mbit = 16MB
  *  Sector  : 4KB (4096 byte) → erase unit nhỏ nhất
  *  Page    : 256 byte → write unit
  *  Block   : 64KB (65536 byte)
  *
  *  Layout flash (ví dụ):
  *   Sector 0   (0x000000..0x000FFF): Settings / config (4KB)
  *   Sector 1-2 (0x001000..0x002FFF): SI5351 cal + band settings (8KB)
  *   Sector 3-N (0x003000..        ): Boot logo bitmap 320×240×2B = 153600B
  *                                   cần 38 sector = 152KB
  *                                   → 0x003000..0x028FFF
  *
  *  Hỗ trợ: Read, Page Program, Sector/Block/Chip Erase
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __W25Q128_H
#define __W25Q128_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* W25Q128 Commands */
#define W25Q_CMD_WRITE_ENABLE    0x06U
#define W25Q_CMD_WRITE_DISABLE   0x04U
#define W25Q_CMD_READ_STATUS1    0x05U
#define W25Q_CMD_READ_STATUS2    0x35U
#define W25Q_CMD_WRITE_STATUS    0x01U
#define W25Q_CMD_READ_DATA       0x03U
#define W25Q_CMD_FAST_READ       0x0BU
#define W25Q_CMD_PAGE_PROGRAM    0x02U
#define W25Q_CMD_SECTOR_ERASE    0x20U   /* 4KB  */
#define W25Q_CMD_BLOCK_ERASE_32K 0x52U   /* 32KB */
#define W25Q_CMD_BLOCK_ERASE_64K 0xD8U   /* 64KB */
#define W25Q_CMD_CHIP_ERASE      0xC7U
#define W25Q_CMD_POWER_DOWN      0xB9U
#define W25Q_CMD_RELEASE_PD      0xABU
#define W25Q_CMD_READ_JEDEC_ID   0x9FU
#define W25Q_CMD_READ_UID        0x4BU

/* Status register bits */
#define W25Q_SR1_BUSY            0x01U
#define W25Q_SR1_WEL             0x02U

/* Geometry */
#define W25Q_PAGE_SIZE           256U
#define W25Q_SECTOR_SIZE         4096U
#define W25Q_BLOCK32_SIZE        32768U
#define W25Q_BLOCK64_SIZE        65536U
#define W25Q_TOTAL_SIZE          (16U * 1024U * 1024U)   /* 16MB */

/* Flash layout addresses */
#define FLASH_ADDR_SETTINGS      0x000000UL   /* 4KB: cài đặt hệ thống */
#define FLASH_ADDR_BAND_CAL      0x001000UL   /* 4KB: band / SI5351 cal */
#define FLASH_ADDR_LOGO          0x003000UL   /* 153600B: boot logo      */
#define FLASH_ADDR_FREE          0x02C000UL   /* Free area               */

/* Timeouts */
#define W25Q_TIMEOUT_SECTOR_MS   400U
#define W25Q_TIMEOUT_BLOCK_MS    2000U
#define W25Q_TIMEOUT_CHIP_MS     60000U
#define W25Q_TIMEOUT_WRITE_MS    10U
#define W25Q_SPI_TIMEOUT_MS      100U

/* Exported types ------------------------------------------------------------*/
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
  uint32_t           jedec_id;   /* Manufacturer + Device ID */
  bool               present;    /* true nếu chip phát hiện  */
} W25Q_Handle_t;

/* Settings structure lưu trong flash */
typedef struct {
  uint32_t   magic;              /* 0xFADEFADE → valid         */
  uint32_t   freq_hz;
  uint8_t    mode;
  uint8_t    band_idx;
  uint8_t    volume;
  uint8_t    squelch;
  uint8_t    att_db;
  bool       agc_fast;
  bool       nb_on;
  bool       nr_on;
  uint32_t   step;
  uint8_t    si5351_cal[32];     /* SI5351 crystal correction  */
  uint8_t    reserved[64];
  uint32_t   crc32;             /* CRC cuối cùng              */
} Flash_Settings_t;

#define FLASH_SETTINGS_MAGIC     0xFADEFADEUL

/* Exported variables --------------------------------------------------------*/
extern W25Q_Handle_t g_flash;

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef W25Q_Init(W25Q_Handle_t *dev, SPI_HandleTypeDef *hspi,
                             GPIO_TypeDef *cs_port, uint16_t cs_pin);
HAL_StatusTypeDef W25Q_ReadID(W25Q_Handle_t *dev, uint32_t *jedec_id);
HAL_StatusTypeDef W25Q_Read(W25Q_Handle_t *dev, uint32_t addr,
                             uint8_t *buf, uint32_t len);
HAL_StatusTypeDef W25Q_PageProgram(W25Q_Handle_t *dev, uint32_t addr,
                                    const uint8_t *buf, uint16_t len);
HAL_StatusTypeDef W25Q_SectorErase(W25Q_Handle_t *dev, uint32_t addr);
HAL_StatusTypeDef W25Q_BlockErase64K(W25Q_Handle_t *dev, uint32_t addr);
HAL_StatusTypeDef W25Q_ChipErase(W25Q_Handle_t *dev);
HAL_StatusTypeDef W25Q_Write(W25Q_Handle_t *dev, uint32_t addr,
                              const uint8_t *buf, uint32_t len);
HAL_StatusTypeDef W25Q_WaitBusy(W25Q_Handle_t *dev, uint32_t timeout_ms);

/* Settings API */
HAL_StatusTypeDef Flash_SaveSettings(W25Q_Handle_t *dev,
                                      const Flash_Settings_t *s);
HAL_StatusTypeDef Flash_LoadSettings(W25Q_Handle_t *dev,
                                      Flash_Settings_t *s);

/* Boot logo */
HAL_StatusTypeDef Flash_WriteLogo(W25Q_Handle_t *dev,
                                   const uint8_t *rgb565_data, uint32_t len);
HAL_StatusTypeDef Flash_ReadLogoScanline(W25Q_Handle_t *dev,
                                          uint16_t y, uint16_t *line_buf);

#ifdef __cplusplus
}
#endif
#endif /* __W25Q128_H */
