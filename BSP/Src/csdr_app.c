/* USER CODE BEGIN Header */
/**
  * @file  csdr_app.c
  * @brief CSDR Application – dùng handle extern từ CubeMX-generated main.c
  *
  *  Tất cả logic ứng dụng nằm ở đây.
  *  Không cần sửa bất kỳ file nào CubeMX generate.
  */
/* USER CODE END Header */

#include "csdr_app.h"
#include "wm8731.h"
#include "ui.h"
#include "sdr_dsp.h"
#include "encoder.h"
#include "si5351.h"
#include "pe4302.h"
#include "w25q128.h"
#include "bpf_lpf.h"
#include "fsdr_analog.h"
#include "usb_cat.h"
#include "usb_audio.h"
#include "menu.h"
#include "diag.h"
#include "cal.h"
#include <string.h>
#include <math.h>

/* ══════════════════════════════════════════════════════════
 *  Extern HAL handles – được define trong main.c do CubeMX generate
 * ══════════════════════════════════════════════════════════ */
extern SPI_HandleTypeDef  hspi1;    /* LCD */
extern SPI_HandleTypeDef  hspi3;    /* Flash */
extern SAI_HandleTypeDef  hsai_BlockA1;
extern SAI_HandleTypeDef  hsai_BlockB1;
extern DMA_HandleTypeDef  hdma_spi1_tx;
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim1;    /* LCD BL */
extern TIM_HandleTypeDef  htim2;    /* Encoder */
extern TIM_HandleTypeDef  htim3;    /* Fan */
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern ADC_HandleTypeDef  hadc3;

/* ══════════════════════════════════════════════════════════
 *  Private state
 * ══════════════════════════════════════════════════════════ */
static ST7789_Handle_t  g_lcd;
static DSP_State_t      g_dsp;

SDR_State_t g_sdr = {
  .freq_hz       = CSDR_FREQ_DEFAULT_HZ,
  .mode          = MODE_USB,
  .band_idx      = 3U,
  .volume        = 200U,
  .squelch       = 0U,
  .step          = STEP_100,
  .agc_fast      = true,
  .display_dirty = true,
};


/* Audio buffers in SRAM1 (DMA accessible) */
static int32_t s_rx_buf[CSDR_AUDIO_BUF_TOTAL * 2U]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static int32_t s_tx_buf[CSDR_AUDIO_BUF_TOTAL * 2U]
    __attribute__((aligned(32), section(".DMA_SRAM")));

static volatile uint8_t s_ping = 0, s_pong = 0;

/* ════ DEBUG COUNTERS - xóa sau khi FFT chạy ════ */
static volatile uint32_t dbg_sai_init_ret    = 0xFFFF;  /* HAL return of SAI DMA start */
static volatile uint32_t dbg_cb_half_count   = 0;       /* HalfCplt callback fire count */
static volatile uint32_t dbg_cb_full_count   = 0;       /* Cplt callback fire count */
static volatile uint32_t dbg_dsp_process_cnt = 0;       /* DSP_Process call count */
static volatile uint32_t dbg_fft_ready_cnt   = 0;       /* FFT completed count */
static volatile int32_t  dbg_rx_sample_0     = 0;       /* Last sample [0] from DMA */
static volatile int32_t  dbg_rx_sample_1     = 0;       /* Last sample [1] from DMA */
static volatile int32_t  dbg_tx_sample_0     = 0;       /* DSP output to DAC [0] */
static volatile int32_t  dbg_tx_sample_1     = 0;       /* DSP output to DAC [1] */
static volatile uint8_t  dbg_force_tone      = 0;       /* set=1 in debugger → 1kHz test tone direct to DAC */
static volatile uint32_t dbg_wm8731_ok       = 0;       /* WM8731 init result */
static volatile uint32_t dbg_si5351_ok       = 0;       /* SI5351 init result */
static volatile uint32_t dbg_sai_error_cnt  = 0;       /* SAI error callback fires */
/* I2C scanner results - xem trong debugger để biết WM8731 thật ở địa chỉ nào */
static volatile uint8_t dbg_i2c_devices[128] = {0};
static volatile uint8_t dbg_i2c_found_count  = 0;



/* ── CAT callbacks ── */
static void     cat_set_freq(uint32_t f);
static void     cat_set_mode(uint8_t m);
static void     cat_set_tx(bool tx);
static void     cat_set_att(uint8_t lv);
static void     cat_set_volume(uint8_t vol);
static void     cat_set_nr(bool on);
static void     cat_set_nb(bool on);
static void     cat_set_bw(uint32_t hz);
static void     cat_set_agc_fast(bool fast);
static void     cat_set_squelch(uint8_t sq);
static uint32_t cat_get_freq(void);
static uint8_t  cat_get_mode(void);
static bool     cat_get_tx(void);
static float    cat_get_signal(void);
static uint8_t  cat_get_att(void);
static uint8_t  cat_get_volume(void);
static bool     cat_get_nr(void);
static bool     cat_get_nb(void);
static uint32_t cat_get_bw(void);
static bool     cat_get_agc_fast(void);
static uint8_t  cat_get_squelch(void);

static void csdr_apply_band(uint8_t band);
static void csdr_handle_encoder(void);
static void csdr_handle_keys(void);
static void csdr_refresh_display(void);
static void menu_apply_cb(void);

/* ══════════════════════════════════════════════════════════
 *  CSDR_Init  – gọi từ main.c USER CODE BEGIN 2
 * ══════════════════════════════════════════════════════════ */

void CSDR_Init(void)
{
  /* Power hold */
  PWR_Init();

  /* ═══ I2C BUS SCANNER - find devices on I2C1 ═══
   * Kết quả trong dbg_i2c_devices[addr7bit] = 1 nếu device có mặt.
   * WM8731 thường ở 0x1A (CSB=0) hoặc 0x1B (CSB=1).
   * SI5351 thường ở 0x60.
   * Xem trong debugger: expand dbg_i2c_devices[] để thấy slot nào = 1 */
  for (uint8_t a = 1; a < 128; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 10) == HAL_OK) {
      dbg_i2c_devices[a] = 1;
      dbg_i2c_found_count++;
    }
  }

  /* LCD */
  g_lcd.hspi     = &hspi1;
  g_lcd.hdma_tx  = &hdma_spi1_tx;
  g_lcd.cs_port  = LCD_CS_GPIO_Port;  g_lcd.cs_pin  = LCD_CS_Pin;
  g_lcd.dc_port  = LCD_DC_GPIO_Port;  g_lcd.dc_pin  = LCD_DC_Pin;
  g_lcd.rst_port = LCD_RST_GPIO_Port; g_lcd.rst_pin = LCD_RST_Pin;
  g_lcd.bl_port  = NULL;
  g_lcd.width    = LCD_W;
  g_lcd.height   = LCD_H;
  ST7789_Init(&g_lcd);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 800U);

  /* Flash + boot logo */
  if (W25Q_Init(&g_flash, &hspi3, FLASH_CS_GPIO_Port, FLASH_CS_Pin) == HAL_OK) {
    Flash_Settings_t fs;
    if (Flash_LoadSettings(&g_flash, &fs) == HAL_OK) {
      g_sdr.freq_hz  = fs.freq_hz;
      g_sdr.mode     = (SDR_Mode_t)fs.mode;
      g_sdr.band_idx = fs.band_idx;
      g_sdr.volume   = fs.volume;
      g_sdr.squelch  = fs.squelch;
      g_sdr.att_db   = fs.att_db;
      g_sdr.agc_fast = fs.agc_fast;
      g_sdr.step     = (FreqStep_t)fs.step;
    }
    for (uint16_t y = 0; y < LCD_H; y++) {
      uint16_t *ln = (uint16_t*)s_rx_buf;
      if (Flash_ReadLogoScanline(&g_flash, y, ln) == HAL_OK)
        ST7789_PushScanline(&g_lcd, y, ln);
    }
    HAL_Delay(1200U);
  }
  McHF_DrawFrame(&g_lcd);

  /* Delay nhỏ trước I2C để bus settle sau power-on */
  HAL_Delay(10);

  /* WM8731 */
  /* Compute WM8731 HP volume from g_sdr.volume using same formula as
   * cat_set_volume: range 90-121 (−31 dB to 0 dB), 0 → hardware mute.
   * g_sdr.volume may have been overridden by Flash_LoadSettings above. */
  uint8_t wm_out_vol = (g_sdr.volume == 0U) ? 0x2FU
                       : (uint8_t)(90U + ((uint16_t)g_sdr.volume * 31U / 255U));
  WM8731_Config_t wm = {
    .hi2c = &hi2c1, .i2c_addr = WM8731_I2C_ADDR,
    .sample_rate = CSDR_AUDIO_SAMPLE_RATE,
    .input_volume = 23U, .output_volume = wm_out_vol, .line_in = true
  };
  dbg_wm8731_ok = (uint32_t)WM8731_Init(&wm);  /* 0 = HAL_OK */

  /* SI5351 */
  dbg_si5351_ok = (uint32_t)SI5351_Init(&g_si5351, &hi2c1, SI5351_I2C_ADDR, SI5351_XTAL_HZ);
  if (dbg_si5351_ok == HAL_OK) {
    g_sdr.si5351_ok = true;
    SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz);
  }

  /* PE4302 Attenuator */
  PE4302_Init(&g_att);
  PE4302_SetAttn_dB(&g_att, g_sdr.att_db);

  /* BPF + LPF */
  BPF_LPF_Init();
  csdr_apply_band(g_sdr.band_idx);

  /* DSP */
  DSP_Init(&g_dsp, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetFrequency(&g_dsp, g_sdr.freq_hz, g_sdr.freq_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);

  /* Encoder */
  Encoder_Init(&g_encoder, &htim2);

  /* Analog subsystem */
  Analog_Init();
  Fan_Init();

  /* USB CAT */
  CAT_Callbacks_t cb = {
    .set_freq      = cat_set_freq,
    .set_mode      = cat_set_mode,
    .set_tx        = cat_set_tx,
    .set_att       = cat_set_att,
    .set_volume    = cat_set_volume,
    .set_nr        = cat_set_nr,
    .set_nb        = cat_set_nb,
    .set_bw        = cat_set_bw,
    .set_agc_fast  = cat_set_agc_fast,
    .set_squelch   = cat_set_squelch,
    .get_freq      = cat_get_freq,
    .get_mode      = cat_get_mode,
    .get_tx        = cat_get_tx,
    .get_signal_db = cat_get_signal,
    .get_att       = cat_get_att,
    .get_volume    = cat_get_volume,
    .get_nr        = cat_get_nr,
    .get_nb        = cat_get_nb,
    .get_bw        = cat_get_bw,
    .get_agc_fast  = cat_get_agc_fast,
    .get_squelch   = cat_get_squelch,
  };
  CAT_Init(&g_cat, &cb);

  /* USB Audio */
  USB_Audio_Init(&g_usb_audio);

  /* Menu */
  Menu_Init(&g_menu, &g_lcd);

  /* Initial UI */
  McHF_UI_State_t ui = {0};
  ui.freq_hz = g_sdr.freq_hz; ui.mode = (uint8_t)g_sdr.mode;
  ui.band_idx = g_sdr.band_idx; ui.volume = g_sdr.volume;
  ui.step = (uint32_t)g_sdr.step; ui.agc_fast = g_sdr.agc_fast;
  ui.si5351_ok = g_sdr.si5351_ok;
  McHF_DrawTopBar(&g_lcd, &ui);
  McHF_DrawStatusPanel(&g_lcd, &ui);

  /* Start SAI DMA (provides BCLK/LRCK to WM8731) */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* Start TX first - it's MASTER and generates BCLK/LRCK for RX */
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)s_tx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);
  HAL_Delay(10U);
  dbg_sai_init_ret = HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)s_rx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);
  HAL_Delay(10U);

  /* Re-activate WM8731 now that SAI clocks are running.
   * First activate may have failed because BCLK/LRCK weren't running.
   * Register 0x09 (Active) = 0x0001 → I2C write:
   *   byte 0: (0x09 << 1) | (0x0001 >> 8) = 0x12 | 0x00 = 0x12
   *   byte 1: 0x0001 & 0xFF = 0x01 */
  {
    uint8_t wm_activate[2] = { 0x12U, 0x01U };
    HAL_I2C_Master_Transmit(&hi2c1, WM8731_I2C_ADDR,
                             wm_activate, 2U, 100U);
  }

  g_sdr.pwr_hold = true;
}

/* ══════════════════════════════════════════════════════════
 *  CSDR_Loop  – gọi trong while(1) của main.c
 * ══════════════════════════════════════════════════════════ */
void CSDR_Loop(void)
{
  /* DSP ping/pong */
  if (s_ping) {
    s_ping = 0;
    SCB_InvalidateDCache_by_Addr((uint32_t*)s_rx_buf,
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
    dbg_rx_sample_0 = s_rx_buf[0];
    dbg_rx_sample_1 = s_rx_buf[1];
    if (g_sdr.tx_mode) {
      DSP_ProcessTX(&g_dsp, s_tx_buf, CSDR_AUDIO_BLOCK_SIZE);
    } else {
      DSP_Process(&g_dsp, s_rx_buf, s_tx_buf, CSDR_AUDIO_BLOCK_SIZE);
    }
    dbg_dsp_process_cnt++;
    /* Force 1kHz test tone directly into TX buffer when dbg_force_tone=1.
     * Set this flag in debugger to verify SAI TX → WM8731 DAC path. */
    if (dbg_force_tone) {
      static uint32_t s_tone_ph = 0U;
      for (uint16_t i = 0U; i < CSDR_AUDIO_BLOCK_SIZE; i++) {
        s_tone_ph += 89478485U;  /* 1000/48000 * 2^32 = 89478485 → 1kHz */
        float sv = sinf(6.28318530f * (float)s_tone_ph * (1.0f/4294967296.0f));
        int32_t w = (int32_t)((uint32_t)(uint16_t)(int16_t)(int32_t)(sv * 16383.0f) << 16U);
        s_tx_buf[i * 2U + 0U] = w;
        s_tx_buf[i * 2U + 1U] = w;
      }
    }
    dbg_tx_sample_0 = s_tx_buf[0];
    dbg_tx_sample_1 = s_tx_buf[1];
    SCB_CleanDCache_by_Addr((uint32_t*)s_tx_buf,
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
  }
  if (s_pong) {
    s_pong = 0;
    SCB_InvalidateDCache_by_Addr((uint32_t*)(s_rx_buf + CSDR_AUDIO_BLOCK_SIZE*2),
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
    if (g_sdr.tx_mode) {
      DSP_ProcessTX(&g_dsp, s_tx_buf + CSDR_AUDIO_BLOCK_SIZE*2,
                     CSDR_AUDIO_BLOCK_SIZE);
    } else {
      DSP_Process(&g_dsp, s_rx_buf + CSDR_AUDIO_BLOCK_SIZE*2,
                   s_tx_buf + CSDR_AUDIO_BLOCK_SIZE*2, CSDR_AUDIO_BLOCK_SIZE);
    }
    SCB_CleanDCache_by_Addr((uint32_t*)(s_tx_buf + CSDR_AUDIO_BLOCK_SIZE*2),
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
  }

  /* Input */
  csdr_handle_encoder();
  csdr_handle_keys();

  /* Timed tasks */
  static uint32_t t_analog=0, t_fan=0, t_pwr=0, t_disp=0, t_cat=0;
  uint32_t now = HAL_GetTick();

  if (now - t_analog >= 100U) { t_analog = now; Analog_Update(); }
  if (now - t_fan    >= 1000U){ t_fan    = now; Fan_Update(g_analog.temp_c); }
  if (now - t_pwr    >= 100U) { t_pwr    = now; PWR_Poll(); }
  if (now - t_disp   >= 200U) { t_disp   = now; csdr_refresh_display(); }
  /* CAT flush: gọi thường xuyên để response đến host trước timeout */
  CAT_FlushTX(&g_cat);

  if (now - t_cat    >= 10U) {
    t_cat = now;
    CAT_Process(&g_cat);
    USB_Audio_Process(&g_usb_audio);
    /* Discard buffered PC TX audio when not transmitting.
     * Without this the ring fills to 6144 bytes and stays there,
     * causing every subsequent USB OUT packet to hit the overrun path. */
    if (!g_sdr.tx_mode && g_usb_audio.tx_count > 0U) {
      g_usb_audio.tx_rd    = g_usb_audio.tx_wr;
      g_usb_audio.tx_count = 0U;
    }
  }
}

/* ══════════════════════════════════════════════════════════
 *  CSDR_SysTickCallback – gọi từ SysTick_Handler
 * ══════════════════════════════════════════════════════════ */
void CSDR_SysTickCallback(void)
{
  /* Guard: chỉ gọi sau khi CSDR_Init() hoàn thành.
   * Tránh HardFault khi HAL_Delay() được gọi trong quá trình
   * USB init (HAL_PCD_Init) trước khi g_encoder được khởi tạo. */
  if (g_sdr.pwr_hold)
    Encoder_Poll(&g_encoder);
}

/* ══════════════════════════════════════════════════════════
 *  CSDR_CDC_Receive – gọi từ usbd_cdc_if.c CDC_Receive_FS
 * ══════════════════════════════════════════════════════════ */
void CSDR_CDC_Receive(uint8_t *buf, uint32_t len)
{
  CAT_Receive(&g_cat, buf, (uint16_t)len);
}

/* ══════════════════════════════════════════════════════════
 *  HAL Callbacks
 *  (ghi đè weak symbols từ HAL – không conflict với CubeMX)
 * ══════════════════════════════════════════════════════════ */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *h)
{
  if (h->Instance == SAI1_Block_B) {
    s_ping = 1;
    dbg_cb_half_count++;
    /* NOTE: s_rx_buf is DMA/cached – read AFTER DCache invalidate in CSDR_Loop */
    if (g_usb_audio.usb_streaming)
      USB_Audio_WriteRX(&g_usb_audio, s_rx_buf, CSDR_AUDIO_BLOCK_SIZE);
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *h)
{
  if (h->Instance == SAI1_Block_B) {
    s_pong = 1;
    dbg_cb_full_count++;
    if (g_usb_audio.usb_streaming)
      USB_Audio_WriteRX(&g_usb_audio,
                         s_rx_buf + CSDR_AUDIO_BLOCK_SIZE*2,
                         CSDR_AUDIO_BLOCK_SIZE);
  }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *h)
{
  (void)h;
  dbg_sai_error_cnt++;
  /* Stop both blocks */
  HAL_SAI_DMAStop(&hsai_BlockA1);   /* TX master first */
  HAL_SAI_DMAStop(&hsai_BlockB1);   /* then RX slave  */
  /* Restart TX master first – it generates BCLK/LRCK for the slave */
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)s_tx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);
  /* Small wait for BCLK/LRCK to stabilise (~10 µs @ 480 MHz) */
  for (volatile uint32_t i = 0U; i < 4800U; i++) { __NOP(); }
  /* Then restart RX slave */
  dbg_sai_init_ret = HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)s_rx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h)
{
  if (h->Instance == SPI1) ST7789_DMA_TxCpltCallback(&g_lcd);
}

/* ══════════════════════════════════════════════════════════
 *  Private functions
 * ══════════════════════════════════════════════════════════ */
static void csdr_apply_band(uint8_t band)
{
  BPF_SetBand(band); LPF_SetBand(band);
  uint32_t f = BPF_BandToFreq(band);
  g_sdr.freq_hz = f; g_sdr.band_idx = band;
  DSP_SetFrequency(&g_dsp, f, f, CSDR_AUDIO_SAMPLE_RATE);
  if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, f);
  g_sdr.display_dirty = true;
}

static void csdr_handle_encoder(void)
{
  int32_t delta = Encoder_GetDelta(&g_encoder);
  if (delta != 0) {
    if (Menu_IsOpen(&g_menu)) { Menu_EncoderEdit(&g_menu, delta); return; }
    int64_t f = (int64_t)g_sdr.freq_hz + (int64_t)delta*(int64_t)g_sdr.step;
    if (f < CSDR_FREQ_MIN_HZ) f = CSDR_FREQ_MIN_HZ;
    if (f > CSDR_FREQ_MAX_HZ) f = CSDR_FREQ_MAX_HZ;
    g_sdr.freq_hz = (uint32_t)f;
    DSP_SetFrequency(&g_dsp, g_sdr.freq_hz, g_sdr.freq_hz, CSDR_AUDIO_SAMPLE_RATE);
    if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz);
    uint8_t b = BPF_FreqToBand(g_sdr.freq_hz);
    if (b != 0xFFU && b != g_sdr.band_idx) { BPF_SetBand(b); g_sdr.band_idx = b; }
    g_sdr.display_dirty = true;
  }
  if (Encoder_GetButton(&g_encoder)) {
    if (Menu_IsOpen(&g_menu)) {
      /* Check if selected item is ACTION type (e.g. Diagnostics) */
      if (g_menu.cursor < g_menu.item_count &&
          g_menu.items[g_menu.cursor].type == MENU_TYPE_ACTION) {
        const char *name = g_menu.items[g_menu.cursor].label;
        Menu_Toggle(&g_menu);
        g_sdr.display_dirty = true;
        if (strcmp(name, "Diagnostics") == 0) {
          Diag_Run(&g_lcd);
        } else if (strcmp(name, "Calibration") == 0) {
          Cal_Params_t cp = {
            .xtal_ppm       = g_sdr.xtal_ppm,
            .iq_gain_tenth  = g_sdr.iq_gain_tenth,
            .iq_phase_tenth = g_sdr.iq_phase_tenth,
            .audio_cal_db   = g_sdr.audio_cal_db,
          };
          if (Cal_Run(&g_lcd, &cp)) {
            g_sdr.xtal_ppm       = cp.xtal_ppm;
            g_sdr.iq_gain_tenth  = cp.iq_gain_tenth;
            g_sdr.iq_phase_tenth = cp.iq_phase_tenth;
            g_sdr.audio_cal_db   = cp.audio_cal_db;
            if (g_sdr.si5351_ok) {
              uint32_t xtal_corr = (uint32_t)((int32_t)SI5351_XTAL_HZ +
                SI5351_XTAL_HZ / 1000000L * g_sdr.xtal_ppm);
              g_si5351.xtal_hz = xtal_corr;
              SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz);
            }
          }
        }
        g_sdr.display_dirty = true;
      } else {
        Menu_Select(&g_menu);
      }
      return;
    }
    g_sdr.mode = (SDR_Mode_t)((g_sdr.mode + 1U) % MODE_COUNT);
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
    g_sdr.display_dirty = true;
  }
  if (Encoder_GetLongPress(&g_encoder)) {
    /* Long press: reserved for future use (QSE now tracks TX state) */
  }
}

static void csdr_handle_keys(void)
{
  /* Debounce timestamps – ignores bounces for 50ms after each press */
  static uint32_t db_menu=0, db_band=0, db_mode=0;
  static uint32_t db_f1=0, db_f2=0, db_f3=0, db_f4=0, db_ptt=0;
  /* Key state: true=currently pressed (for long-press detection) */
  static bool st_menu=false, st_band=false, st_mode=false;
  static bool st_f1=false, st_f2=false, st_f3=false;
  static bool st_f4=false, st_ptt=false;

#define KEY_DEBOUNCE_MS  50U
/* KEY_FELL: true only on first press after debounce period */
#define KEY_FELL(port, pin, last_t, state)   ({ bool _now = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);      bool _fire = false;      if (_now && !(state) && (HAL_GetTick()-(last_t)) >= KEY_DEBOUNCE_MS)        { _fire=true; (last_t)=HAL_GetTick(); }      (state) = _now;      _fire; })

  /* Macro: true on falling edge (button press) */

  if (KEY_FELL(MENU_KEY_GPIO_Port, MENU_KEY_Pin, db_menu, st_menu)) {
    g_sdr.display_dirty = false;  /* prevent status panel overwriting menu */
    if (!Menu_IsOpen(&g_menu))
      Menu_LoadFromSDR(&g_menu,
        g_sdr.agc_fast, g_sdr.nb_on, g_sdr.nr_on, g_sdr.rit_hz,
        g_sdr.volume, g_sdr.squelch, (uint32_t)g_sdr.step,
        g_sdr.att_db, g_sdr.band_idx, (uint8_t)g_sdr.mode,
        g_sdr.usb_mode, menu_apply_cb);
    Menu_Toggle(&g_menu);
    if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty = true;
  }

  /* F1: menu UP / ATT+ */
  if (KEY_FELL(F1_KEY_GPIO_Port, F1_KEY_Pin, db_f1, st_f1)) {
    if (Menu_IsOpen(&g_menu)) Menu_Up(&g_menu);
    else {
      PE4302_IncAttn(&g_att);
      g_sdr.att_db = g_att.current_atten_db;
      g_sdr.display_dirty = true;
    }
  }

  /* F2: menu DOWN / ATT- */
  if (KEY_FELL(F2_KEY_GPIO_Port, F2_KEY_Pin, db_f2, st_f2)) {
    if (Menu_IsOpen(&g_menu)) Menu_Down(&g_menu);
    else {
      PE4302_DecAttn(&g_att);
      g_sdr.att_db = g_att.current_atten_db;
      g_sdr.display_dirty = true;
    }
  }

  if (KEY_FELL(F3_KEY_GPIO_Port, F3_KEY_Pin, db_f3, st_f3)) {
    if (Menu_IsOpen(&g_menu)) {
      if (g_menu.cursor < g_menu.item_count &&
          g_menu.items[g_menu.cursor].type == MENU_TYPE_ACTION) {
        const char *name = g_menu.items[g_menu.cursor].label;
        Menu_Toggle(&g_menu);
        g_sdr.display_dirty = true;
        if (strcmp(name, "Diagnostics") == 0) {
          Diag_Run(&g_lcd);
        } else if (strcmp(name, "Calibration") == 0) {
          Cal_Params_t cp = {
            .xtal_ppm=g_sdr.xtal_ppm, .iq_gain_tenth=g_sdr.iq_gain_tenth,
            .iq_phase_tenth=g_sdr.iq_phase_tenth, .audio_cal_db=g_sdr.audio_cal_db
          };
          if (Cal_Run(&g_lcd, &cp)) {
            g_sdr.xtal_ppm=cp.xtal_ppm; g_sdr.iq_gain_tenth=cp.iq_gain_tenth;
            g_sdr.iq_phase_tenth=cp.iq_phase_tenth; g_sdr.audio_cal_db=cp.audio_cal_db;
            if (g_sdr.si5351_ok) {
              g_si5351.xtal_hz=(uint32_t)((int32_t)SI5351_XTAL_HZ+
                SI5351_XTAL_HZ/1000000L*g_sdr.xtal_ppm);
              SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz);
            }
          }
        }
        g_sdr.display_dirty = true;
      } else {
        Menu_Confirm(&g_menu);
      }
    }
  }

  /* F4: Back / Exit menu */
  if (KEY_FELL(F4_KEY_GPIO_Port, F4_KEY_Pin, db_f4, st_f4)) {
    if (Menu_IsOpen(&g_menu)) {
      Menu_Back(&g_menu);
      if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty = true;
    }
  }

  if (KEY_FELL(BAND_KEY_GPIO_Port, BAND_KEY_Pin, db_band, st_band))
    csdr_apply_band(BPF_BandUp(g_sdr.band_idx));

  if (KEY_FELL(MODE_KEY_GPIO_Port, MODE_KEY_Pin, db_mode, st_mode)) {
    g_sdr.mode = (SDR_Mode_t)((g_sdr.mode+1U) % MODE_COUNT);
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
    g_sdr.display_dirty = true;
  }

  if (KEY_FELL(PTT_GPIO_Port, PTT_Pin, db_ptt, st_ptt)) {
    cat_set_tx(!g_sdr.tx_mode);  /* reuse same TX sequencing */
  }
  #undef KEY_FELL
#undef KEY_DEBOUNCE_MS
}

static void csdr_refresh_display(void)
{
  bool menu_open = Menu_IsOpen(&g_menu);

  /* Spectrum + waterfall: only when menu closed */
  if (g_dsp.fft_ready && !menu_open) {
    g_dsp.fft_ready = false;
    /* Tính bandwidth marker ratios theo mode:
     *   LSB:   1 đường bên TRÁI (lower sideband, -BW..0)
     *   USB/CW: 1 đường bên PHẢI (upper sideband, 0..+BW)
     *   AM/FM: 2 đường đối xứng tại ±BW/2 (double sideband)
     *   ratio đơn vị: fraction của full spectrum span (Fs) */
    float bw_lo_ratio = 0.0f, bw_hi_ratio = 0.0f;
    if (g_dsp.sample_rate > 0U && g_dsp.bw_hz > 0.0f)
    {
      float full = g_dsp.bw_hz / (float)g_dsp.sample_rate;  /* BW / Fs */
      float half = full * 0.5f;                              /* BW/2 / Fs */
      switch (g_sdr.mode)
      {
        case MODE_LSB: bw_lo_ratio = full; bw_hi_ratio = 0.0f; break;
        case MODE_USB:
        case MODE_CW:  bw_lo_ratio = 0.0f; bw_hi_ratio = full; break;
        case MODE_AM:
        case MODE_FM:
        default:       bw_lo_ratio = half; bw_hi_ratio = half; break;
      }
    }
    McHF_DrawSpectrum(&g_lcd, g_dsp.fft_mag_db, DSP_FFT_SIZE,
                      bw_lo_ratio, bw_hi_ratio, NULL);
    McHF_DrawWaterfall(&g_lcd, g_dsp.fft_mag_db, DSP_FFT_SIZE);
  }

  if (g_sdr.display_dirty) {
    g_sdr.display_dirty = false;
    McHF_UI_State_t ui = {0};
    ui.freq_hz  = g_sdr.freq_hz;  ui.mode      = (uint8_t)g_sdr.mode;
    ui.band_idx = g_sdr.band_idx; ui.volume     = g_sdr.volume;
    ui.squelch  = g_sdr.squelch;  ui.step       = (uint32_t)g_sdr.step;
    ui.agc_fast = g_sdr.agc_fast; ui.nb_on      = g_sdr.nb_on;
    ui.nr_on    = g_sdr.nr_on;    ui.rit_hz     = g_sdr.rit_hz;
    ui.tx_mode  = g_sdr.tx_mode;  ui.si5351_ok  = g_sdr.si5351_ok;
    ui.qse_on   = g_sdr.qse_on;   ui.signal_db  = g_dsp.signal_power_db;

    /* TopBar (y=0..61) luôn cập nhật */
    McHF_DrawTopBar(&g_lcd, &ui);

    /* StatusPanel (y=62+) và S-meter chỉ khi menu ĐÓNG
     * Nếu menu đang mở: tuyệt đối không ghi đè vùng y=62.. */
    if (!menu_open) {
      McHF_DrawStatusPanel(&g_lcd, &ui);
    } else {
      /* Menu đang mở: re-render để đảm bảo không bị xóa */
      Menu_Render(&g_menu);
    }
  } else if (!menu_open) {
    /* Cập nhật S-meter nhẹ – chỉ khi menu đóng */
    McHF_UpdateSMeter(&g_lcd, g_dsp.signal_power_db);
  }
}

static void menu_apply_cb(void)
{
  bool agc, nb, nr; int16_t rit;
  uint8_t vol, sq, att, band, mode, usb; uint32_t step;
  Menu_SaveToSDR(&g_menu, &agc, &nb, &nr, &rit,
                  &vol, &sq, &step, &att, &band, &mode, &usb);
  g_sdr.agc_fast = agc; g_sdr.nb_on = nb; g_sdr.nr_on = nr;
  g_sdr.rit_hz = rit;   g_sdr.volume = vol; g_sdr.squelch = sq;
  g_sdr.step = (FreqStep_t)step;
  if (att != g_sdr.att_db) { PE4302_SetAttn_dB(&g_att, att); g_sdr.att_db = att; }
  if (band != g_sdr.band_idx) csdr_apply_band(band);
  if (mode != (uint8_t)g_sdr.mode) {
    g_sdr.mode = (SDR_Mode_t)mode;
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
  }
  g_sdr.usb_mode = usb;
  g_sdr.display_dirty = true;
}

/* CAT callbacks */
static void     cat_set_freq(uint32_t f) { g_sdr.freq_hz=f; DSP_SetFrequency(&g_dsp,f,f,CSDR_AUDIO_SAMPLE_RATE); if(g_sdr.si5351_ok)SI5351_SetQSDFrequency(&g_si5351,f); g_sdr.display_dirty=true; }
static void     cat_set_mode(uint8_t m)  { g_sdr.mode=(SDR_Mode_t)m; DSP_SetMode(&g_dsp,g_sdr.mode,CSDR_AUDIO_SAMPLE_RATE); g_sdr.display_dirty=true; }
static void cat_set_tx(bool tx)
{
  if (tx == g_sdr.tx_mode) return;  /* no change */
  g_sdr.tx_mode = tx;
  if (tx) {
    /* TX sequence: enable TX LO → switch T/R → enable PA
     * Keep USB streaming ON so PC audio can flow through tx_ring. */
    HAL_Delay(2);                                  /* 2ms settle   */
    if (g_sdr.si5351_ok) {
      SI5351_SetQSEFrequency(&g_si5351, g_sdr.freq_hz);  /* CLK2 ON */
      g_sdr.qse_on = true;
    }
    HAL_GPIO_WritePin(TR_SW_GPIO_Port, TR_SW_Pin, GPIO_PIN_SET);
  } else {
    /* RX sequence: disable PA → switch T/R → disable TX LO → restore QSD */
    HAL_GPIO_WritePin(TR_SW_GPIO_Port, TR_SW_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    if (g_sdr.si5351_ok) {
      SI5351_EnableOutput(&g_si5351, 2U, false);     /* CLK2 OFF */
      g_sdr.qse_on = false;
      /* SI5351_SetQSEFrequency reprograms PLL_A (shared with QSD CLK0/CLK1).
       * Re-apply QSD frequency to restore CLK0/CLK1 to correct output. */
      SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz);
    }
    WM8731_SetMute(&hi2c1, WM8731_I2C_ADDR, false);  /* Ensure HP unmuted */
  }
  McHF_UpdateSMeter_SetTX(tx);
  g_sdr.display_dirty = true;
}
static void     cat_set_att(uint8_t lv)  { static const uint8_t m[]={0,6,12,18}; PE4302_SetAttn_dB(&g_att,(lv<4)?m[lv]:0); g_sdr.att_db=g_att.current_atten_db; }

/* Scale 0-255 CAT volume → WM8731 HP register.
 * Range 90-121 (−31 dB to 0 dB) so 100% slider = 0 dB reference (not +6 dB).
 * vol==0 → hardware mute (register < 0x30). */
static void cat_set_volume(uint8_t vol)
{
  g_sdr.volume = vol;
  uint8_t wm_vol = (vol == 0U) ? 0x2FU
                                : (uint8_t)(90U + ((uint16_t)vol * 31U / 255U));
  WM8731_SetVolume(&hi2c1, WM8731_I2C_ADDR, wm_vol, wm_vol);
  g_sdr.display_dirty = true;
}
static void     cat_set_nr(bool on)       { g_sdr.nr_on = on; g_sdr.display_dirty = true; }
static void     cat_set_nb(bool on)       { g_sdr.nb_on = on; g_sdr.display_dirty = true; }
static void     cat_set_bw(uint32_t hz)   { DSP_SetBW(&g_dsp, (float)hz); g_sdr.display_dirty = true; }
static void     cat_set_agc_fast(bool f)  { g_sdr.agc_fast = f; g_sdr.display_dirty = true; }
static void     cat_set_squelch(uint8_t s){ g_sdr.squelch = s; g_sdr.display_dirty = true; }

static uint32_t cat_get_freq(void)        { return g_sdr.freq_hz; }
static uint8_t  cat_get_mode(void)        { return (uint8_t)g_sdr.mode; }
static bool     cat_get_tx(void)          { return g_sdr.tx_mode; }
static float    cat_get_signal(void)      { return g_dsp.signal_power_db; }
static uint8_t  cat_get_att(void)         { return g_sdr.att_db; }
static uint8_t  cat_get_volume(void)      { return g_sdr.volume; }
static bool     cat_get_nr(void)          { return g_sdr.nr_on; }
static bool     cat_get_nb(void)          { return g_sdr.nb_on; }
static uint32_t cat_get_bw(void)          { return (uint32_t)g_dsp.bw_hz; }
static bool     cat_get_agc_fast(void)    { return g_sdr.agc_fast; }
static uint8_t  cat_get_squelch(void)     { return g_sdr.squelch; }
/* ── Audio buffer accessors (expose main.c static buffers) ── */
extern int32_t tx_buf[];
extern int32_t rx_buf[];
extern volatile uint8_t dsp_ping;
extern volatile uint8_t dsp_pong;

int32_t *CSDR_GetTxBuf(void) { return tx_buf; }
int32_t *CSDR_GetRxBuf(void) { return rx_buf; }
void CSDR_ClearDspFlags(void) { dsp_ping = 0; dsp_pong = 0; }
