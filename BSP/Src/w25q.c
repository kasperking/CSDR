/* USER CODE BEGIN Header */
/**
  * @file w25q.c
  * @brief W25Q SPI NOR Flash Driver
  */
/* USER CODE END Header */

#include "w25q.h"
#include <string.h>

/* USER CODE BEGIN PV */
W25Q_Handle_t g_flash;   /* zero-init → present=false; real init only when HW_STORAGE_W25Q==1 */
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

#define _CS_L(d) HAL_GPIO_WritePin((d)->cs_port,(d)->cs_pin,GPIO_PIN_RESET)
#define _CS_H(d) HAL_GPIO_WritePin((d)->cs_port,(d)->cs_pin,GPIO_PIN_SET)

static HAL_StatusTypeDef spi_tx(W25Q_Handle_t *d, const uint8_t *b, uint16_t n)
{ return HAL_SPI_Transmit(d->hspi,(uint8_t*)b,n,W25Q_SPI_TIMEOUT_MS); }

/* STM32H7 SPI in 2LINES full-duplex: HAL_SPI_Transmit accumulates one dirty
 * byte in RXFIFO per call (MISO captured during TX is not discarded by HAL).
 * Drain before every HAL_SPI_Receive so we read real flash data, not stale
 * RXFIFO bytes from prior transmits. */
static inline void spi_rx_flush(W25Q_Handle_t *d)
{
    while ((d->hspi->Instance->SR & SPI_SR_RXP) != 0U) {
        (void)(*(volatile uint8_t *)&d->hspi->Instance->RXDR);
    }
}

static HAL_StatusTypeDef spi_rx(W25Q_Handle_t *d, uint8_t *b, uint16_t n)
{
    spi_rx_flush(d);
    return HAL_SPI_Receive(d->hspi,b,n,W25Q_SPI_TIMEOUT_MS);
}

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
#if !HW_STORAGE_W25Q
  /* No W25Q fitted per hw_config (HW_STORAGE_W25Q == 0). */
  (void)hspi; (void)cs_port; (void)cs_pin;
  dev->present = false;
  return HAL_ERROR;
#endif
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

  /* Read JEDEC ID — accept any non-trivial response (0x000000 / 0xFFFFFF = no chip) */
  uint32_t id = 0U;
  if (W25Q_ReadID(dev, &id) == HAL_OK) {
    dev->jedec_id = id;
    dev->present = (id != 0x000000U && id != 0xFFFFFFU);
  }

  if (!dev->present) return HAL_ERROR;

  /* Clear status register to remove any write protection (BP bits) */
  W25Q_WriteSR1(dev, 0x00U);

  return HAL_OK;
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
                                          uint16_t y, uint16_t width, uint16_t *line_buf)
{
  /* USER CODE BEGIN Flash_ReadLogoScanline_0 */
  uint32_t offset = FLASH_ADDR_LOGO + (uint32_t)y * width * 2U;
  return W25Q_Read(dev, offset, (uint8_t*)line_buf, (uint32_t)width * 2U);
  /* USER CODE END Flash_ReadLogoScanline_0 */
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef W25Q_ReadSR1(W25Q_Handle_t *dev, uint8_t *sr)
{
  uint8_t cmd = W25Q_CMD_READ_STATUS1;
  *sr = 0U;
  _CS_L(dev);
  HAL_StatusTypeDef r = spi_tx(dev, &cmd, 1U);
  if (r == HAL_OK) r = spi_rx(dev, sr, 1U);
  _CS_H(dev);
  return r;
}

HAL_StatusTypeDef W25Q_WriteSR1(W25Q_Handle_t *dev, uint8_t new_sr)
{
  HAL_StatusTypeDef r = W25Q_WaitBusy(dev, 20U);
  if (r != HAL_OK) return r;
  r = w25q_write_enable(dev);
  if (r != HAL_OK) return r;
  uint8_t cmd[2] = { W25Q_CMD_WRITE_STATUS, new_sr };
  _CS_L(dev); spi_tx(dev, cmd, 2U); _CS_H(dev);
  return W25Q_WaitBusy(dev, 50U);
}

HAL_StatusTypeDef Flash_LoadBandCal(W25Q_Handle_t *dev,
                                     BandCal_t band[BAND_COUNT])
{
  BandCalBlock_t blk;
  bool ok = (W25Q_Read(dev, FLASH_ADDR_BAND_CAL,
                        (uint8_t*)&blk, sizeof(blk)) == HAL_OK)
            && (blk.magic == BAND_CAL_MAGIC)
            && (crc32_simple((const uint8_t*)&blk,
                              sizeof(blk) - sizeof(blk.crc32)) == blk.crc32);
  if (ok) {
    memcpy(band, blk.band, sizeof(blk.band));
  } else {
    /* Defaults: no gain trim, swr_scale=100 (×1.0 = no scaling) */
    for (uint8_t i = 0; i < BAND_COUNT; i++) {
      band[i].rx_gain_trim    = 0;
      band[i].noise_floor_off = 0;
      band[i].tx_drive_trim   = 0;
      band[i].swr_scale       = 100;
    }
  }
  return ok ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef Flash_SaveBandCal(W25Q_Handle_t *dev,
                                     const BandCal_t band[BAND_COUNT])
{
  BandCalBlock_t blk;
  blk.magic = BAND_CAL_MAGIC;
  memcpy(blk.band, band, sizeof(blk.band));
  blk.crc32 = crc32_simple((const uint8_t*)&blk,
                             sizeof(blk) - sizeof(blk.crc32));
  HAL_StatusTypeDef r = W25Q_SectorErase(dev, FLASH_ADDR_BAND_CAL);
  if (r != HAL_OK) return r;
  return W25Q_Write(dev, FLASH_ADDR_BAND_CAL,
                    (const uint8_t*)&blk, sizeof(blk));
}

/* ── Async settings save ──────────────────────────────────────────────────
 * State machine driven by Flash_SaveTick() from the main loop.  Each tick
 * costs one SR1 read (2-byte SPI transaction) while the flash is BUSY; the
 * erase command and each page program are issued without the internal
 * WaitBusy of the synchronous API, so the main loop never blocks on tERASE
 * (45–400 ms) or tPP (0.7–3 ms).
 *
 * Interleaving with the synchronous W25Q API is safe by construction: every
 * sync op begins with W25Q_WaitBusy, so it simply waits out an in-flight
 * async step (old blocking behaviour, no corruption).  usb_flash_proto is
 * the one caller that must not slip a command between async steps on the
 * same sector — it defers on Flash_SaveBusy(). */
typedef enum {
  FSAVE_IDLE = 0,
  FSAVE_ERASE,      /* sector erase issued, waiting for BUSY to clear     */
  FSAVE_PROGRAM,    /* page program issued, waiting; s_fsave_off advanced */
} FSave_State_t;

static FSave_State_t    s_fsave_state = FSAVE_IDLE;
static Flash_Settings_t s_fsave_blob;        /* blob being written        */
static Flash_Settings_t s_fsave_next;        /* queued request (latest)   */
static bool             s_fsave_have_next;
static uint32_t         s_fsave_off;         /* bytes programmed so far   */
static uint32_t         s_fsave_t0;          /* per-step timeout anchor   */

volatile uint32_t dbg_fsave_started  = 0U;
volatile uint32_t dbg_fsave_done     = 0U;
volatile uint32_t dbg_fsave_errors   = 0U;
volatile uint32_t dbg_fsave_queued   = 0U;

/* Issue the sector erase without waiting for completion. */
static HAL_StatusTypeDef fsave_start_erase(W25Q_Handle_t *dev)
{
  HAL_StatusTypeDef r = w25q_write_enable(dev);
  if (r != HAL_OK) return r;
  uint8_t b[4] = { W25Q_CMD_SECTOR_ERASE,
                   (uint8_t)(FLASH_ADDR_SETTINGS >> 16),
                   (uint8_t)(FLASH_ADDR_SETTINGS >> 8),
                   (uint8_t)FLASH_ADDR_SETTINGS };
  _CS_L(dev); r = spi_tx(dev, b, 4U); _CS_H(dev);
  s_fsave_t0 = HAL_GetTick();
  return r;
}

/* Program the next page-bounded chunk without waiting for completion.
 * The SPI transfer itself (≤260 B) is synchronous but takes only tens of µs. */
static HAL_StatusTypeDef fsave_program_chunk(W25Q_Handle_t *dev)
{
  uint32_t addr  = FLASH_ADDR_SETTINGS + s_fsave_off;
  uint32_t chunk = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);
  uint32_t left  = sizeof(s_fsave_blob) - s_fsave_off;
  if (chunk > left) chunk = left;

  HAL_StatusTypeDef r = w25q_write_enable(dev);
  if (r != HAL_OK) return r;
  uint8_t hdr[4] = { W25Q_CMD_PAGE_PROGRAM,
                     (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
  _CS_L(dev);
  r = spi_tx(dev, hdr, 4U);
  if (r == HAL_OK)
    r = spi_tx(dev, (const uint8_t *)&s_fsave_blob + s_fsave_off, (uint16_t)chunk);
  _CS_H(dev);
  if (r == HAL_OK) s_fsave_off += chunk;
  s_fsave_t0 = HAL_GetTick();
  return r;
}

static void fsave_abort(void)
{
  dbg_fsave_errors++;
  s_fsave_state     = FSAVE_IDLE;
  s_fsave_have_next = false;   /* sector state unknown — drop queued blob too;
                                  the next save request rewrites everything */
}

HAL_StatusTypeDef Flash_SaveSettingsAsync(W25Q_Handle_t *dev,
                                           const Flash_Settings_t *s)
{
  if (!dev->present) return HAL_ERROR;

  Flash_Settings_t tmp;
  memcpy(&tmp, s, sizeof(tmp));
  tmp.magic = FLASH_SETTINGS_MAGIC;
  tmp.crc32 = crc32_simple((const uint8_t *)&tmp,
                           sizeof(tmp) - sizeof(tmp.crc32));

  if (s_fsave_state != FSAVE_IDLE) {
    /* Save in flight: queue this snapshot; latest request wins.  Never touch
     * s_fsave_blob mid-write — pages already programmed came from it. */
    memcpy(&s_fsave_next, &tmp, sizeof(tmp));
    s_fsave_have_next = true;
    dbg_fsave_queued++;
    return HAL_OK;
  }

  memcpy(&s_fsave_blob, &tmp, sizeof(tmp));
  s_fsave_off = 0U;
  if (fsave_start_erase(dev) != HAL_OK) { fsave_abort(); return HAL_ERROR; }
  s_fsave_state = FSAVE_ERASE;
  dbg_fsave_started++;
  return HAL_OK;
}

void Flash_SaveTick(W25Q_Handle_t *dev)
{
  if (s_fsave_state == FSAVE_IDLE) return;

  uint8_t sr = 0U;
  if (W25Q_ReadSR1(dev, &sr) != HAL_OK) { fsave_abort(); return; }
  if (sr & W25Q_SR1_BUSY) {
    /* Still working — bail with a per-step timeout guard.  Erase max 400 ms,
     * page program max ~3 ms; a stuck BUSY beyond that means a wedged chip. */
    uint32_t lim = (s_fsave_state == FSAVE_ERASE) ? 500U : 20U;
    if ((HAL_GetTick() - s_fsave_t0) > lim) fsave_abort();
    return;
  }

  /* BUSY clear: erase finished (state ERASE) or previous page finished
   * (state PROGRAM) — issue the next page, or finish. */
  if (s_fsave_off < sizeof(s_fsave_blob)) {
    if (fsave_program_chunk(dev) != HAL_OK) { fsave_abort(); return; }
    s_fsave_state = FSAVE_PROGRAM;
    return;
  }

  /* Last page confirmed done (BUSY clear, nothing left to program). */
  dbg_fsave_done++;
  s_fsave_state = FSAVE_IDLE;
  if (s_fsave_have_next) {
    s_fsave_have_next = false;
    memcpy(&s_fsave_blob, &s_fsave_next, sizeof(s_fsave_blob));
    s_fsave_off = 0U;
    if (fsave_start_erase(dev) != HAL_OK) { fsave_abort(); return; }
    s_fsave_state = FSAVE_ERASE;
    dbg_fsave_started++;
  }
}

void Flash_SaveFlush(W25Q_Handle_t *dev)
{
  /* Shutdown path only: drive the state machine to completion.  Bounded by
   * the per-step timeouts in Flash_SaveTick (worst ≈ 2 saves ≈ 1 s). */
  while (s_fsave_state != FSAVE_IDLE) {
    Flash_SaveTick(dev);
    if (s_fsave_state != FSAVE_IDLE) HAL_Delay(1U);
  }
}

bool Flash_SaveBusy(void)
{
  return s_fsave_state != FSAVE_IDLE;
}

/* USER CODE END 1 */
