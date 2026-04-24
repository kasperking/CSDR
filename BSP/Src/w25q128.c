/* USER CODE BEGIN Header */
/**
  * @file w25q128.c
  * @brief W25Q128JV SPI Flash Driver
  */
/* USER CODE END Header */

#include "w25q128.h"
#include <string.h>

/* USER CODE BEGIN PV */
W25Q_Handle_t g_flash;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

#define _CS_L(d) HAL_GPIO_WritePin((d)->cs_port,(d)->cs_pin,GPIO_PIN_RESET)
#define _CS_H(d) HAL_GPIO_WritePin((d)->cs_port,(d)->cs_pin,GPIO_PIN_SET)

static HAL_StatusTypeDef spi_tx(W25Q_Handle_t *d, const uint8_t *b, uint16_t n)
{ return HAL_SPI_Transmit(d->hspi,(uint8_t*)b,n,W25Q_SPI_TIMEOUT_MS); }

static HAL_StatusTypeDef spi_rx(W25Q_Handle_t *d, uint8_t *b, uint16_t n)
{ return HAL_SPI_Receive(d->hspi,b,n,W25Q_SPI_TIMEOUT_MS); }

static HAL_StatusTypeDef __attribute__((unused)) spi_txrx(W25Q_Handle_t *d,
  const uint8_t *tx, uint8_t *rx, uint16_t n)
{ return HAL_SPI_TransmitReceive(d->hspi,(uint8_t*)tx,rx,n,W25Q_SPI_TIMEOUT_MS); }

static HAL_StatusTypeDef w25q_cmd(W25Q_Handle_t *d, uint8_t cmd)
{ _CS_L(d); HAL_StatusTypeDef r=spi_tx(d,&cmd,1); _CS_H(d); return r; }

static HAL_StatusTypeDef w25q_write_enable(W25Q_Handle_t *d)
{ return w25q_cmd(d,W25Q_CMD_WRITE_ENABLE); }

static HAL_StatusTypeDef __attribute__((unused)) w25q_addr_cmd(W25Q_Handle_t *d, uint8_t cmd, uint32_t addr)
{ uint8_t b[4]={cmd,(uint8_t)(addr>>16),(uint8_t)(addr>>8),(uint8_t)addr};
  _CS_L(d); HAL_StatusTypeDef r=spi_tx(d,b,4); _CS_H(d); return r; }

/* Simple CRC32 */
static uint32_t crc32_simple(const uint8_t *data, uint32_t len)
{ uint32_t crc=0xFFFFFFFFUL;
  for(uint32_t i=0;i<len;i++){
    crc^=(uint32_t)data[i];
    for(uint8_t b=0;b<8;b++)
      crc=(crc&1)?((crc>>1)^0xEDB88320UL):(crc>>1);
  }
  return ~crc;
}

/* USER CODE END 0 */

HAL_StatusTypeDef W25Q_Init(W25Q_Handle_t *dev, SPI_HandleTypeDef *hspi,
                             GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  /* USER CODE BEGIN W25Q_Init_0 */
  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin  = cs_pin;
  dev->present = false;
  dev->jedec_id = 0U;

  _CS_H(dev);
  HAL_Delay(5U);

  /* Release from power-down */
  w25q_cmd(dev, W25Q_CMD_RELEASE_PD);
  HAL_Delay(1U);

  /* Read JEDEC ID */
  uint32_t id = 0U;
  if (W25Q_ReadID(dev, &id) == HAL_OK) {
    dev->jedec_id = id;
    /* W25Q128: 0xEF4018 */
    dev->present = ((id & 0xFFFF00U) == 0xEF4000U);
  }
  return dev->present ? HAL_OK : HAL_ERROR;
  /* USER CODE END W25Q_Init_0 */
}

HAL_StatusTypeDef W25Q_ReadID(W25Q_Handle_t *dev, uint32_t *jedec_id)
{
  /* USER CODE BEGIN W25Q_ReadID_0 */
  uint8_t cmd = W25Q_CMD_READ_JEDEC_ID;
  uint8_t buf[3] = {0};
  _CS_L(dev);
  HAL_StatusTypeDef r = spi_tx(dev, &cmd, 1U);
  if (r == HAL_OK) r = spi_rx(dev, buf, 3U);
  _CS_H(dev);
  *jedec_id = ((uint32_t)buf[0] << 16U) | ((uint32_t)buf[1] << 8U) | buf[2];
  return r;
  /* USER CODE END W25Q_ReadID_0 */
}

HAL_StatusTypeDef W25Q_WaitBusy(W25Q_Handle_t *dev, uint32_t timeout_ms)
{
  /* USER CODE BEGIN W25Q_WaitBusy_0 */
  uint32_t t0 = HAL_GetTick();
  uint8_t cmd = W25Q_CMD_READ_STATUS1, sr = 0U;
  while (1) {
    _CS_L(dev);
    spi_tx(dev, &cmd, 1U);
    spi_rx(dev, &sr,  1U);
    _CS_H(dev);
    if (!(sr & W25Q_SR1_BUSY)) return HAL_OK;
    if ((HAL_GetTick() - t0) > timeout_ms) return HAL_TIMEOUT;
    HAL_Delay(1U);
  }
  /* USER CODE END W25Q_WaitBusy_0 */
}

HAL_StatusTypeDef W25Q_Read(W25Q_Handle_t *dev, uint32_t addr,
                             uint8_t *buf, uint32_t len)
{
  /* USER CODE BEGIN W25Q_Read_0 */
  uint8_t hdr[4] = { W25Q_CMD_READ_DATA,
                     (uint8_t)(addr >> 16U),
                     (uint8_t)(addr >> 8U),
                     (uint8_t)(addr) };
  _CS_L(dev);
  HAL_StatusTypeDef r = spi_tx(dev, hdr, 4U);
  if (r == HAL_OK) r = spi_rx(dev, buf, (uint16_t)(len & 0xFFFFU));
  _CS_H(dev);
  return r;
  /* USER CODE END W25Q_Read_0 */
}

HAL_StatusTypeDef W25Q_PageProgram(W25Q_Handle_t *dev, uint32_t addr,
                                    const uint8_t *buf, uint16_t len)
{
  /* USER CODE BEGIN W25Q_PageProgram_0 */
  if (len > W25Q_PAGE_SIZE) len = W25Q_PAGE_SIZE;
  HAL_StatusTypeDef r;
  r = W25Q_WaitBusy(dev, W25Q_TIMEOUT_WRITE_MS);
  if (r != HAL_OK) return r;
  r = w25q_write_enable(dev);
  if (r != HAL_OK) return r;

  uint8_t hdr[4] = { W25Q_CMD_PAGE_PROGRAM,
                     (uint8_t)(addr >> 16U),
                     (uint8_t)(addr >> 8U),
                     (uint8_t)(addr) };
  _CS_L(dev);
  spi_tx(dev, hdr, 4U);
  spi_tx(dev, buf, len);
  _CS_H(dev);
  return W25Q_WaitBusy(dev, W25Q_TIMEOUT_WRITE_MS);
  /* USER CODE END W25Q_PageProgram_0 */
}

HAL_StatusTypeDef W25Q_SectorErase(W25Q_Handle_t *dev, uint32_t addr)
{
  /* USER CODE BEGIN W25Q_SectorErase_0 */
  HAL_StatusTypeDef r = W25Q_WaitBusy(dev, W25Q_TIMEOUT_SECTOR_MS);
  if (r != HAL_OK) return r;
  r = w25q_write_enable(dev);
  if (r != HAL_OK) return r;
  uint8_t b[4] = { W25Q_CMD_SECTOR_ERASE,
                   (uint8_t)(addr>>16),(uint8_t)(addr>>8),(uint8_t)addr };
  _CS_L(dev); spi_tx(dev,b,4); _CS_H(dev);
  return W25Q_WaitBusy(dev, W25Q_TIMEOUT_SECTOR_MS);
  /* USER CODE END W25Q_SectorErase_0 */
}

HAL_StatusTypeDef W25Q_BlockErase64K(W25Q_Handle_t *dev, uint32_t addr)
{
  /* USER CODE BEGIN W25Q_BlockErase64K_0 */
  HAL_StatusTypeDef r = W25Q_WaitBusy(dev, W25Q_TIMEOUT_BLOCK_MS);
  if (r != HAL_OK) return r;
  r = w25q_write_enable(dev);
  if (r != HAL_OK) return r;
  uint8_t b[4] = { W25Q_CMD_BLOCK_ERASE_64K,
                   (uint8_t)(addr>>16),(uint8_t)(addr>>8),(uint8_t)addr };
  _CS_L(dev); spi_tx(dev,b,4); _CS_H(dev);
  return W25Q_WaitBusy(dev, W25Q_TIMEOUT_BLOCK_MS);
  /* USER CODE END W25Q_BlockErase64K_0 */
}

HAL_StatusTypeDef W25Q_ChipErase(W25Q_Handle_t *dev)
{
  /* USER CODE BEGIN W25Q_ChipErase_0 */
  HAL_StatusTypeDef r = w25q_write_enable(dev);
  if (r != HAL_OK) return r;
  w25q_cmd(dev, W25Q_CMD_CHIP_ERASE);
  return W25Q_WaitBusy(dev, W25Q_TIMEOUT_CHIP_MS);
  /* USER CODE END W25Q_ChipErase_0 */
}

/**
  * @brief  Ghi bất kỳ số byte (tự động chia page, erase sector nếu cần).
  */
HAL_StatusTypeDef W25Q_Write(W25Q_Handle_t *dev, uint32_t addr,
                              const uint8_t *buf, uint32_t len)
{
  /* USER CODE BEGIN W25Q_Write_0 */
  HAL_StatusTypeDef r;
  while (len > 0U) {
    uint32_t page_off = addr % W25Q_PAGE_SIZE;
    uint32_t chunk    = W25Q_PAGE_SIZE - page_off;
    if (chunk > len) chunk = len;
    r = W25Q_PageProgram(dev, addr, buf, (uint16_t)chunk);
    if (r != HAL_OK) return r;
    addr += chunk; buf += chunk; len -= chunk;
  }
  return HAL_OK;
  /* USER CODE END W25Q_Write_0 */
}

HAL_StatusTypeDef Flash_SaveSettings(W25Q_Handle_t *dev,
                                      const Flash_Settings_t *s)
{
  /* USER CODE BEGIN Flash_SaveSettings_0 */
  Flash_Settings_t tmp;
  memcpy(&tmp, s, sizeof(tmp));
  tmp.magic = FLASH_SETTINGS_MAGIC;
  tmp.crc32 = crc32_simple((const uint8_t*)&tmp,
                             sizeof(tmp) - sizeof(tmp.crc32));
  HAL_StatusTypeDef r = W25Q_SectorErase(dev, FLASH_ADDR_SETTINGS);
  if (r != HAL_OK) return r;
  return W25Q_Write(dev, FLASH_ADDR_SETTINGS,
                    (const uint8_t*)&tmp, sizeof(tmp));
  /* USER CODE END Flash_SaveSettings_0 */
}

HAL_StatusTypeDef Flash_LoadSettings(W25Q_Handle_t *dev,
                                      Flash_Settings_t *s)
{
  /* USER CODE BEGIN Flash_LoadSettings_0 */
  HAL_StatusTypeDef r = W25Q_Read(dev, FLASH_ADDR_SETTINGS,
                                   (uint8_t*)s, sizeof(*s));
  if (r != HAL_OK) return r;
  if (s->magic != FLASH_SETTINGS_MAGIC) return HAL_ERROR;

  uint32_t crc_calc = crc32_simple((const uint8_t*)s,
                                    sizeof(*s) - sizeof(s->crc32));
  return (crc_calc == s->crc32) ? HAL_OK : HAL_ERROR;
  /* USER CODE END Flash_LoadSettings_0 */
}

HAL_StatusTypeDef Flash_ReadLogoScanline(W25Q_Handle_t *dev,
                                          uint16_t y, uint16_t *line_buf)
{
  /* USER CODE BEGIN Flash_ReadLogoScanline_0 */
  /* Logo là RGB565, 320px × 240 dòng → mỗi dòng 640 byte */
  uint32_t offset = FLASH_ADDR_LOGO + (uint32_t)y * 320U * 2U;
  return W25Q_Read(dev, offset, (uint8_t*)line_buf, 320U * 2U);
  /* USER CODE END Flash_ReadLogoScanline_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
