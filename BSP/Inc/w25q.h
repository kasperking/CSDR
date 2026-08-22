/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    w25q.h
  * @brief   W25Q SPI NOR Flash BSP Driver (W25Q16 .. W25Q128)
  *
  *  Hardware: SPI3  (PC10=SCK, PC11=MISO, PC12=MOSI, PD0=CS)
  *  Capacity: configured by hw_config.py (HW_W25Q_CAPACITY_BYTES)
  *  Sector  : 4KB (4096 byte) → erase unit nhỏ nhất
  *  Page    : 256 byte → write unit
  *  Block   : 64KB (65536 byte)
  *
  *  Layout flash:
  *   0x000000-0x003FFF  Settings    16KB
  *   0x004000-0x007FFF  Band cal    16KB
  *   0x008000-0x052FFF  Logo        300KB max (480x320x2 = 307200 B)
  *   0x058000-0x05FFFF  Font data   32KB
  *   0x060000-0x063FFF  FFT twiddle 16KB
  *   0x064000-0x067FFF  FFT bitrev  16KB
  *
  *  Hỗ trợ: Read, Page Program, Sector/Block/Chip Erase
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __W25Q_H
#define __W25Q_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "hw_config_active.h"
#include "bpf_lpf.h"   /* BAND_COUNT */
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

/* Geometry — sourced from hw_config_active.h when W25Q is fitted.
 * hw_config.py emits HW_W25Q_* only when storage_type == W25Q_NOR.
 * When HW_STORAGE_W25Q == 0 these constants are not used at runtime
 * (W25Q_Init returns HAL_ERROR immediately), but safe fallbacks are
 * provided so the driver file compiles in all configurations. */
#if HW_STORAGE_W25Q
#  define W25Q_PAGE_SIZE    HW_W25Q_PAGE_SIZE      /* from hw_config (256 for all W25Q) */
#  define W25Q_SECTOR_SIZE  HW_W25Q_SECTOR_SIZE    /* from hw_config (4096)             */
#  define W25Q_BLOCK32_SIZE HW_W25Q_BLOCK32_SIZE   /* from hw_config (32768)            */
#  define W25Q_BLOCK64_SIZE HW_W25Q_BLOCK64_SIZE   /* from hw_config (65536)            */
#  define W25Q_TOTAL_SIZE   HW_W25Q_CAPACITY_BYTES /* from hw_config (model-specific)   */
#else
#  define W25Q_PAGE_SIZE    256U
#  define W25Q_SECTOR_SIZE  4096U
#  define W25Q_BLOCK32_SIZE 32768U
#  define W25Q_BLOCK64_SIZE 65536U
#  define W25Q_TOTAL_SIZE   0U
#endif

/* Flash layout addresses */
/* Flash layout — DO NOT overlap regions.
 *  0x000000–0x003FFF  Settings   16KB
 *  0x004000–0x007FFF  Band cal   16KB
 *  0x008000–0x052FFF  Logo       300KB max (480×320×2 = 307200 B)
 *  0x053000–0x057FFF  [gap 20KB]
 *  0x058000–0x05FFFF  Font data  32KB  zone
 *  0x060000–0x063FFF  FFT twiddle 16KB zone
 *  0x064000–0x067FFF  FFT bitrev  16KB zone
 *  0x068000–0xFFFFFF  free (15.6 MB on W25Q128)                    */
#define FLASH_ADDR_SETTINGS      0x000000UL   /* 16KB: system settings  */
#define FLASH_ADDR_BAND_CAL      0x004000UL   /* 16KB: band / SI5351 cal */
#define FLASH_ADDR_LOGO          0x008000UL   /* 300KB max: boot logo up to 480×320 RGB565 */
#define FLASH_ADDR_LOGO_END      0x053000UL   /* first byte AFTER max logo area */
#define FLASH_ADDR_FONT_DATA     0x058000UL   /* 32KB zone (0x058000–0x05FFFF): font bitmaps  */
#define FLASH_ADDR_FFT_TWIDDLE   0x060000UL   /* 16KB zone (0x060000–0x063FFF): twiddle tables */
#define FLASH_ADDR_FFT_BITREV    0x064000UL   /* 16KB zone (0x064000–0x067FFF): bitrev tables  */
#define FLASH_ADDR_TX_UNLOCK     0x068000UL   /* 4KB: out-of-band TX unlock audit log
                                               * (tx_unlock.c). Separate from Settings so a
                                               * Factory Reset cannot wipe the trace.        */
/* 0x069000 and beyond: free (15.6 MB remaining on W25Q128) */

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

/* Settings structure stored in flash sector 0.
 * Layout: 4-byte fields first, 2-byte next, 1-byte/bool last.  crc32 covers
 * all bytes except itself (last 4) — including any implicit padding, which is
 * always zero because csdr_save_settings() memsets the struct before filling.
 * Struct size = 152 B (1-byte group ends at offset 147 → pad to crc32 @ 148).
 * Changing field order/offsets breaks backward compat (CRC mismatch →
 * defaults loaded); new 1-byte fields may consume the padding before crc32
 * without breaking old blobs (they read back 0 → loader must map 0 to the
 * field's default). */
typedef struct {
  /* ── always first ───────────────────────────────────────────── */
  uint32_t   magic;              /* 0xFADEFADE → valid           */

  /* ── VFO A — 4-byte ─────────────────────────────────────────── */
  uint32_t   freq_hz;
  uint32_t   step;               /* FreqStep_t cast to u32       */
  uint32_t   bw_hz;              /* high-cut filter edge Hz      */
  uint32_t   sl_hz;              /* low-cut filter edge Hz       */

  /* ── Calibration — 4-byte ───────────────────────────────────── */
  int32_t    xtal_ppm;           /* SI5351 crystal correction — đơn vị
                                    theo xtal_cal_unit (0=ppm, 1=ppb) */
  int32_t    dc_i_offset;        /* DSP DC-I bias                */
  int32_t    dc_q_offset;        /* DSP DC-Q bias                */
  uint32_t   lo_offset_hz;       /* LO tuning offset             */

  /* ── VFO B — 4-byte ─────────────────────────────────────────── */
  uint32_t   vfo_b_freq_hz;
  uint32_t   vfo_b_step;
  uint32_t   vfo_b_bw_hz;
  uint32_t   vfo_b_sl_hz;

  /* ── VFO A / TX / audio — 2-byte ────────────────────────────── */
  int16_t    if_shift_hz;
  int16_t    mic_gain;           /* voice TX drive 0-100         */
  int16_t    digi_gain;          /* digi TX drive 0-100          */
  int16_t    audio_gain_db;

  /* ── Calibration — 2-byte ───────────────────────────────────── */
  int16_t    smeter_offset_db;
  int16_t    iq_gain;
  int16_t    iq_phase;

  /* ── VFO B — 2-byte ─────────────────────────────────────────── */
  int16_t    vfo_b_if_shift_hz;

  /* ── VFO A — 1-byte ─────────────────────────────────────────── */
  uint8_t    mode;               /* SDR_Mode_t cast to u8        */
  uint8_t    band_idx;
  uint8_t    volume;
  uint8_t    squelch;
  uint8_t    att_db;
  uint8_t    nb_level;           /* noise blanker intensity 0-100 */
  uint8_t    active_vfo;         /* 0 = A, 1 = B                 */
  uint8_t    tx_power;           /* TX output power 0-100%       */
  uint8_t    pa_watts;           /* PA rating: 0/20/45/100 W     */
  uint8_t    pa_oc_limit_idx;    /* OC limit ×10 A — 10..200 = 1.0..20.0 A */

  /* ── VFO B — 1-byte ─────────────────────────────────────────── */
  uint8_t    vfo_b_mode;
  uint8_t    vfo_b_band_idx;

  /* ── Bool flags ─────────────────────────────────────────────── */
  uint8_t    agc_speed;    /*!< 0=SLOW 1=FAST 2=AUTO */
  bool       nb_on;
  uint8_t    nr_mode;      /*!< 0=off 1=NR1 (LMS) 2=NR2 (spectral) */
  bool       rf_agc_on;
  uint8_t    usb_mode;     /*!< 0=Off 1=CAT 2=Audio                 */
  bool       usb_iq_stream;/*!< true=raw IQ false=demodulated audio  */

  /* ── SI5351 per-band calibration (future) — tail carved for Ext PA
   *    (array was [32], never written → old blobs read 0 in the new
   *    fields; offsets of everything after are unchanged) ─────────── */
  uint8_t    si5351_cal[21];
  uint8_t    pa_idq_t10;         /* Idq target ÷10 mA: 5..200 = 50..2000 mA;
                                    0 = blob cũ → default 50 (500 mA).
                                    Carved từ si5351_cal tail               */
  uint8_t    pa_bias_src;        /* PA bias source: 0=FIXED (trimmer, cũng là
                                    blob cũ), 1=DAC (MCP4822).  Carved từ
                                    si5351_cal tail                       */
  uint8_t    pa_bias1_p1;        /* DAC bias A (final), lưu level+1: 1..201 =
                                    0..200; 0 = blob cũ → default 0 (OFF) */
  uint8_t    pa_bias2_p1;        /* DAC bias B (driver), cùng encoding    */
  uint8_t    rtty_baud_idx;      /* g_rtty_baud_x100 index (0=45.45 Bd); carved
                                    from si5351_cal tail, old blobs read 0    */
  uint8_t    rtty_shift_idx;     /* g_rtty_shift_hz index (0=170 Hz); same    */
  int8_t     utc_offset_h;       /* Múi giờ hiển thị: RTC = UTC + offset giờ,
                                    -12..+14; 0 cũng = blob cũ → UTC+0.
                                    Carved thêm từ si5351_cal tail        */
  uint8_t    xtal_cal_unit;      /* Đơn vị của field xtal_ppm phía trên:
                                    0 = ppm (blob cũ), 1 = ppb (GPS cal).
                                    Carved thêm từ si5351_cal tail        */
  uint8_t    ext_pa_on;          /* External PA fitted: 0=off 1=on;
                                    0 also = pre-ext-pa blob padding      */
  uint8_t    ext_pa_delay_p1;    /* TX keying delay, stored ms+1: 1..51 =
                                    0..50 ms; 0 = old blob → default 25   */
  uint8_t    ext_pa_drv;         /* Drive cap % while ext_pa_on: 5..100;
                                    0 = old blob → default 50             */

  /* ── TX audio passband + notch + extended flags (carved from reserved) ── */
  uint16_t   tx_audio_low_hz;    /* TX Low-cut HPF Hz: 100-500; 0=default(200)   */
  uint16_t   tx_audio_high_hz;   /* TX High-cut LPF Hz: 2200-3500; 0=default(2800) */
  int16_t    notch_hz;           /* Notch center Hz: 100-4000; 0=default(1000)   */
  uint8_t    ext_alc_on;         /* External ALC enable: 0=off 1=on      */
  uint8_t    notch_on;           /* Audio notch filter: 0=off 1=on       */
  uint8_t    vox_on;             /* VOX enable: 0=off 1=on               */
  uint8_t    vox_gain;           /* VOX sensitivity 0-100 (100=hot)      */
  uint16_t   vox_delay_ms;       /* VOX hang time 100-2000 ms            */

  /* ── CW keyer / TX settings ─────────────────────────────────── */
  uint16_t   cw_pitch_hz;        /* BFO/sidetone Hz: 300-900, def 700    */
  uint16_t   cw_bk_delay_ms;     /* BK-IN hang delay ms: 50-2000, def 200*/
  uint16_t   cw_filter_hz;       /* CW filter BW Hz: 50-500, def 500     */
  uint8_t    cw_wpm;             /* Keyer speed WPM: 5-40, def 20        */
  uint8_t    keyer_mode;         /* 0=Straight 1=IambicA 2=IambicB       */
  uint8_t    sidetone_vol;       /* Sidetone volume 0-100%, def 50       */
  uint8_t    cw_bkin;            /* 0=Off 1=Semi 2=Full break-in         */
  bool       paddle_reverse;     /* Swap DIT/DAH paddles                 */
  bool       cw_reverse;         /* CW reverse sideband flag             */
  bool       cw_decode_on;       /* CW decoder enable                    */
  uint8_t    tx_src;             /* TX audio source: 0=USB 1=MIC         */
  uint8_t    nr_level;           /* NR strength 0-100, def 50            */
  uint8_t    bc_mode;            /* Beat canceller: 0=off 1/2=on         */
  uint8_t    pwr_scale;          /* Tandem-match FWD power cal 50..200 %;
                                    0 = pre-pwr_scale blob → default 100 */
  uint8_t    marker_track;       /* Spectrum marker: 0=Fix (default), 1=Track;
                                    0 also = pre-marker blob padding      */
  int8_t     bass_db;            /* RX bass shelf gain dB: -10..+10; also the
                                    pre-tone-control blob padding value (0=flat) */
  int8_t     treble_db;          /* RX treble shelf gain dB: -10..+10; same padding */
  uint8_t    rtty_decode_on;     /* RTTY decoder enable: 1=on, 0=off.  Reuses the
                                    ft8_reserved byte (originally ft8_decode_on,
                                    then dead padding) — every old blob wrote 0
                                    here, so the layout AND semantics stay
                                    backward-compatible with no settings reset  */
  char       ft8_call[12];       /* FT8 TX callsign, NUL-terminated; [0]=0 = unset */
  char       ft8_grid[5];        /* FT8 TX 4-char grid, NUL-terminated             */

  /* ── always last ────────────────────────────────────────────── */
  uint32_t   crc32;
} Flash_Settings_t;

#define FLASH_SETTINGS_MAGIC     0xFADEFADEUL

/* Per-band calibration block stored at FLASH_ADDR_BAND_CAL.
 * rx_gain_trim    : int16, dB,  -20..+20  — hardware RX gain correction per band
 * noise_floor_off : int16, dB,  -20..+20  — per-band S-meter noise floor trim
 * tx_drive_trim   : int16, %, -50..+50   — TX audio gain: g *= (100+trim)/100
 * swr_scale       : int16,    50..200    — SWR scale: swr_x100 *= swr_scale/100
 * Default: rx_gain_trim=0, noise_floor_off=0, tx_drive_trim=0, swr_scale=100 */
typedef struct {
  int16_t rx_gain_trim;
  int16_t noise_floor_off;
  int16_t tx_drive_trim;
  int16_t swr_scale;          /* ×0.01 unit: 100 = ×1.0 (no scaling) */
} BandCal_t;                  /* 8 bytes */

/* EEPROM layout FROZEN at 11 slots (slot 10 = former 6m band, now unused).
 * Keeping the stored block at 11 entries preserves the size and CRC of
 * every existing blob when BAND_COUNT shrinks — no cal reset on upgrade.
 * Must always be ≥ BAND_COUNT; the RAM array g_band_cal[] stays BAND_COUNT. */
#define BAND_CAL_SLOTS  11U

typedef struct {
  uint32_t  magic;                 /* BAND_CAL_MAGIC when valid            */
  BandCal_t band[BAND_CAL_SLOTS];  /* BAND_CAL_SLOTS×8 = 88 bytes          */
  uint32_t  crc32;                 /* CRC covers all bytes before this     */
} BandCalBlock_t;                  /* 4 + 88 + 4 = 96 bytes                */

#define BAND_CAL_MAGIC  0xCA1BCA1BUL

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
HAL_StatusTypeDef W25Q_ReadSR1(W25Q_Handle_t *dev, uint8_t *sr);
HAL_StatusTypeDef W25Q_WriteSR1(W25Q_Handle_t *dev, uint8_t new_sr);

/* Settings API */
HAL_StatusTypeDef Flash_SaveSettings(W25Q_Handle_t *dev,
                                      const Flash_Settings_t *s);
HAL_StatusTypeDef Flash_LoadSettings(W25Q_Handle_t *dev,
                                      Flash_Settings_t *s);

/* Async settings save — main-loop friendly replacement for Flash_SaveSettings.
 * The synchronous path blocks 45–400 ms on the sector erase (audio/CAT/key
 * stall); this engine issues each flash op without waiting and advances one
 * step per Flash_SaveTick() call, polling BUSY with a single SR1 read (~µs).
 *   Flash_SaveSettingsAsync: snapshot *s (magic+CRC applied) and start.
 *     A request while a save is in flight is queued (latest wins) and starts
 *     as soon as the current one finishes.
 *   Flash_SaveTick : call once per main-loop iteration; no-op when idle.
 *   Flash_SaveFlush: block until idle (shutdown path only).
 *   Flash_SaveBusy : true while a save is erasing/programming/queued —
 *     other W25Q users that must not interleave (usb_flash_proto) check this. */
HAL_StatusTypeDef Flash_SaveSettingsAsync(W25Q_Handle_t *dev,
                                           const Flash_Settings_t *s);
void Flash_SaveTick(W25Q_Handle_t *dev);
void Flash_SaveFlush(W25Q_Handle_t *dev);
bool Flash_SaveBusy(void);

/* Per-band calibration API — reads/writes FLASH_ADDR_BAND_CAL.
 * Flash_LoadBandCal: on magic/CRC failure, writes safe defaults (swr_scale=100)
 *                    into band[] and returns HAL_ERROR.
 * Flash_SaveBandCal: sector-erase + page-write of BandCalBlock_t (96 bytes). */
HAL_StatusTypeDef Flash_LoadBandCal(W25Q_Handle_t *dev,
                                     BandCal_t band[BAND_COUNT]);
HAL_StatusTypeDef Flash_SaveBandCal(W25Q_Handle_t *dev,
                                     const BandCal_t band[BAND_COUNT]);

/* Boot logo */
HAL_StatusTypeDef Flash_WriteLogo(W25Q_Handle_t *dev,
                                   const uint8_t *rgb565_data, uint32_t len);
HAL_StatusTypeDef Flash_ReadLogoScanline(W25Q_Handle_t *dev,
                                          uint16_t y, uint16_t width, uint16_t *line_buf);

#ifdef __cplusplus
}
#endif
#endif /* __W25Q_H */
