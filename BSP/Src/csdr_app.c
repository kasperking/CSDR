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
#include "sdr_ui.h"
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
#include "sdr_scan.h"
#include "runtime_diag.h"
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
  .volume        = 78U,
  .bw_hz         = 3000U,
  .if_shift_hz   = 0,
  .squelch       = 0U,
  .step          = STEP_100,
  .agc_fast      = true,
  .display_dirty = true,
  .lo_offset_hz  = LO_OFFSET_DEFAULT,
  .mic_gain      = 50,
  .vfo_b         = {
    .freq_hz     = 14200000UL,   /* VFO B default: 20m */
    .mode        = MODE_USB,
    .band_idx    = 5U,
    .step        = STEP_100,
    .rit_hz      = 0,
    .bw_hz       = 3000U,
  },
  .active_vfo    = 0U,
};


/* Audio buffers in SRAM1 (DMA accessible) */
static int32_t s_rx_buf[CSDR_AUDIO_BUF_TOTAL * 2U]
    __attribute__((aligned(32), section(".DMA_SRAM")));
static int32_t s_tx_buf[CSDR_AUDIO_BUF_TOTAL * 2U]
    __attribute__((aligned(32), section(".DMA_SRAM")));

static volatile uint32_t s_rx_ready_seq[2] = { 0U, 0U };
static uint32_t s_rx_done_seq[2] = { 0U, 0U };

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

/* USB watchdog counters – defined in usb_composite.c, watched here via extern */
extern volatile uint32_t dbg_usb_sof_cnt;     /* SOF callbacks received from host      */
extern volatile uint32_t dbg_usb_iso_in_cnt;  /* ISO IN packets successfully queued     */
extern volatile uint32_t dbg_usb_stall_cnt;   /* ISO IN queue failures (stall/not-ready)*/

/* Set to 1 in debugger to skip waterfall/LCD DMA while diagnosing USB audio. */
static volatile uint8_t dbg_disable_lcd_dma = 0;

/* UI refresh caps.  RX values preserve the existing cadence; TX values keep
 * long SPI LCD transfers away from the 5.33 ms audio half-buffer deadline. */
#define CSDR_UI_WF_RX_PERIOD_MS       40U   /* 25 fps waterfall in RX */
#define CSDR_UI_DISPLAY_RX_PERIOD_MS 200U   /* 5 Hz spectrum/meter in RX */
#define CSDR_UI_DISPLAY_TX_PERIOD_MS 1000U  /* 1 Hz compact TX meter refresh */
#define CSDR_UI_TX_DIRTY_MIN_MS      1000U  /* defer knob/menu redraws while TX audio is time-critical */


/* ── Function key state machines ── */
static Key_t k_menu, k_f1, k_f2, k_f3, k_f4, k_band, k_mode, k_ptt;

/* ── CAT callbacks ── */
static void     cat_set_freq(uint32_t f);
static inline uint32_t csdr_nco_freq(void)
{
    return (uint32_t)((int32_t)g_sdr.freq_hz + (int32_t)g_sdr.if_shift_hz);
}

static void     cat_set_mode(uint8_t m);
static void     cat_set_tx(bool tx);
static void     cat_set_att(uint8_t lv);
static void     cat_set_volume(uint8_t vol);
static void     cat_set_nr(bool on);
static void     cat_set_nb(bool on);
static void     cat_set_bw(uint32_t hz);
static void     cat_set_agc_fast(bool fast);
static void     cat_set_squelch(uint8_t sq);
static void     cat_set_rit_hz(int32_t hz);
static void     cat_set_step(uint32_t hz);
static void     cat_set_if_shift(int32_t hz);
static int32_t  cat_get_if_shift(void);
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
static int32_t  cat_get_rit_hz(void);
static uint32_t cat_get_step(void);

static void csdr_apply_band(uint8_t band);
static void csdr_handle_encoder(void);
static void csdr_handle_keys(void);
static void csdr_process_audio_pending(void);
static void csdr_update_waterfall(void);
static void csdr_refresh_display(void);
static void menu_apply_cb(void);
static uint8_t  default_zoom_for_mode(SDR_Mode_t m);
static uint32_t default_bw_for_mode(SDR_Mode_t m);
static void csdr_vfo_swap(void);
static void csdr_vfo_copy_to_b(void);
/* CAT VFO-B callbacks */
static void     cat_set_vfo_b_freq(uint32_t hz);
static void     cat_set_vfo_b_mode(uint8_t m);
static void     cat_set_vfo_b_bw(uint32_t hz);
static uint32_t cat_get_vfo_b_freq(void);
static uint8_t  cat_get_vfo_b_mode(void);
static uint32_t cat_get_vfo_b_bw(void);
static void     cat_set_active_vfo(uint8_t vfo);
static uint8_t  cat_get_active_vfo(void);

/* ══════════════════════════════════════════════════════════
 *  CSDR_Init  – gọi từ main.c USER CODE BEGIN 2
 * ══════════════════════════════════════════════════════════ */

void CSDR_Init(void)
{
  RuntimeDiag_Init();

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
  /* Keep LCD completion IRQs below USB/SAI audio IRQs.  SPI DMA can occupy the
   * bus for multi-kilobyte UI pushes, but its ISR must not preempt the audio
   * half-buffer producer/consumer callbacks. */
  HAL_NVIC_SetPriority(SPI1_IRQn, 5U, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5U, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0U, 0U);

  g_lcd.hspi     = &hspi1;
  g_lcd.hdma_tx  = &hdma_spi1_tx;
  g_lcd.cs_port  = LCD_CS_GPIO_Port;  g_lcd.cs_pin  = LCD_CS_Pin;
  g_lcd.dc_port  = LCD_DC_GPIO_Port;  g_lcd.dc_pin  = LCD_DC_Pin;
  g_lcd.rst_port = LCD_RST_GPIO_Port; g_lcd.rst_pin = LCD_RST_Pin;
  g_lcd.bl_port  = NULL;
  g_lcd.width    = LCD_W;
  g_lcd.height   = LCD_H;
  ST7789_Init(&g_lcd);
  SDR_UI_Init();
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
  SDR_UI_DrawFrame(&g_lcd, CSDR_AUDIO_SAMPLE_RATE, DSP_FFT_SIZE);

  /* Delay nhỏ trước I2C để bus settle sau power-on */
  HAL_Delay(10);

  /* WM8731 */
  /* Compute WM8731 HP volume from g_sdr.volume using same formula as
   * cat_set_volume: range 90-121 (−31 dB to 0 dB), 0 → hardware mute.
   * g_sdr.volume may have been overridden by Flash_LoadSettings above. */
  uint8_t wm_out_vol = (g_sdr.volume == 0U) ? 0x2FU
                       : (uint8_t)(90U + ((uint16_t)g_sdr.volume * 31U / 100U));
  WM8731_Config_t wm = {
    .hi2c = &hi2c1, .i2c_addr = WM8731_I2C_ADDR,
    .sample_rate = CSDR_AUDIO_SAMPLE_RATE,
    .input_volume = 23U, .output_volume = wm_out_vol, .line_in = true
  };
  dbg_wm8731_ok = (uint32_t)WM8731_Init(&wm);  /* 0 = HAL_OK */
  if (dbg_wm8731_ok != HAL_OK) RuntimeDiag_SetFault(FAULT_CODEC);

  /* SI5351 */
  dbg_si5351_ok = (uint32_t)SI5351_Init(&g_si5351, &hi2c1, SI5351_I2C_ADDR, SI5351_XTAL_HZ);
  if (dbg_si5351_ok == HAL_OK) {
    g_sdr.si5351_ok = true;
    SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
  } else {
    RuntimeDiag_SetFault(FAULT_PLL);
  }

  /* PE4302 Attenuator */
  PE4302_Init(&g_att);
  PE4302_SetAttn_dB(&g_att, g_sdr.att_db);

  /* BPF + LPF */
  BPF_LPF_Init();
  csdr_apply_band(g_sdr.band_idx);

  /* DSP */
  DSP_Init(&g_dsp, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetFrequency(&g_dsp, csdr_nco_freq(), g_sdr.freq_hz, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
  DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
  AGC_SetSpeed(&g_dsp.agc, g_sdr.agc_fast, CSDR_AUDIO_SAMPLE_RATE);

  /* Encoder */
  Encoder_Init(&g_encoder, &htim2);

  /* Function keys */
  Key_Init(&k_menu, MENU_KEY_GPIO_Port, MENU_KEY_Pin);
  Key_Init(&k_f1,   F1_KEY_GPIO_Port,   F1_KEY_Pin);
  Key_Init(&k_f2,   F2_KEY_GPIO_Port,   F2_KEY_Pin);
  Key_Init(&k_f3,   F3_KEY_GPIO_Port,   F3_KEY_Pin);
  Key_Init(&k_f4,   F4_KEY_GPIO_Port,   F4_KEY_Pin);
  Key_Init(&k_band, BAND_KEY_GPIO_Port, BAND_KEY_Pin);
  Key_Init(&k_mode, MODE_KEY_GPIO_Port, MODE_KEY_Pin);
  Key_Init(&k_ptt,  PTT_GPIO_Port,      PTT_Pin);

  /* Analog subsystem */
  Analog_Init();
  Fan_Init();

  /* USB CAT */
  CAT_Callbacks_t cb = {
    .set_freq        = cat_set_freq,
    .set_mode        = cat_set_mode,
    .set_tx          = cat_set_tx,
    .set_att         = cat_set_att,
    .set_volume      = cat_set_volume,
    .set_nr          = cat_set_nr,
    .set_nb          = cat_set_nb,
    .set_bw          = cat_set_bw,
    .set_agc_fast    = cat_set_agc_fast,
    .set_squelch     = cat_set_squelch,
    .set_rit_hz      = cat_set_rit_hz,
    .set_step        = cat_set_step,
    .get_freq        = cat_get_freq,
    .get_mode        = cat_get_mode,
    .get_tx          = cat_get_tx,
    .get_signal_db   = cat_get_signal,
    .get_att         = cat_get_att,
    .get_volume      = cat_get_volume,
    .get_nr          = cat_get_nr,
    .get_nb          = cat_get_nb,
    .get_bw          = cat_get_bw,
    .get_agc_fast    = cat_get_agc_fast,
    .get_squelch     = cat_get_squelch,
    .get_rit_hz      = cat_get_rit_hz,
    .get_step        = cat_get_step,
    .set_if_shift    = cat_set_if_shift,
    .get_if_shift    = cat_get_if_shift,
    /* VFO B + active-VFO selection */
    .set_vfo_b_freq  = cat_set_vfo_b_freq,
    .set_vfo_b_mode  = cat_set_vfo_b_mode,
    .set_vfo_b_bw    = cat_set_vfo_b_bw,
    .get_vfo_b_freq  = cat_get_vfo_b_freq,
    .get_vfo_b_mode  = cat_get_vfo_b_mode,
    .get_vfo_b_bw    = cat_get_vfo_b_bw,
    .set_active_vfo  = cat_set_active_vfo,
    .get_active_vfo  = cat_get_active_vfo,
  };
  CAT_Init(&g_cat, &cb);

  /* USB Audio */
  USB_Audio_Init(&g_usb_audio);

  /* Menu */
  Menu_Init(&g_menu, &g_lcd);

  /* Initial UI */
  SDR_UI_State_t ui = {0};
  ui.freq_hz   = g_sdr.freq_hz;        ui.mode      = (uint8_t)g_sdr.mode;
  ui.band_idx  = g_sdr.band_idx;       ui.volume    = g_sdr.volume;
  ui.step      = (uint32_t)g_sdr.step; ui.agc_fast  = g_sdr.agc_fast;
  ui.si5351_ok = g_sdr.si5351_ok;      ui.bw_hz     = g_sdr.bw_hz;
  ui.freq_b_hz = g_sdr.vfo_b.freq_hz;  ui.active_vfo = g_sdr.active_vfo;
  SDR_UI_DrawTopBar(&g_lcd, &ui);
  SDR_UI_DrawStatusPanel(&g_lcd, &ui);

  /* Start SAI DMA (provides BCLK/LRCK to WM8731) */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* Start TX first - it's MASTER and generates BCLK/LRCK for RX */
  RuntimeDiag_TxHalfFilled(0U);
  RuntimeDiag_TxHalfFilled(1U);
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
static void csdr_process_audio_pending(void)
{
  /* DSP ping/pong */
  uint32_t rx_seq = s_rx_ready_seq[0];
  if (rx_seq != s_rx_done_seq[0]) {
    /* s_rx_buf is in .DMA_SRAM (0x24000000, MPU: NON_CACHEABLE) so the
     * invalidate below is a no-op.  Keep it for safety if MPU config ever
     * changes.  Address and size are both 32-byte aligned:
     *   addr  = s_rx_buf (aligned(32) attribute)
     *   size  = 256 × 2 × 4 = 2048 B  (2048 / 32 = 64, exact multiple) */
    SCB_InvalidateDCache_by_Addr((uint32_t*)s_rx_buf,
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
    /* Feed USB ring from main-loop context, not ISR, to avoid starving the
     * USB OTG interrupt handler when the host opens the audio stream. */
    if (g_usb_audio.usb_streaming)
      USB_Audio_WriteRX(&g_usb_audio, s_rx_buf, CSDR_AUDIO_BLOCK_SIZE);
    dbg_rx_sample_0 = s_rx_buf[0];
    dbg_rx_sample_1 = s_rx_buf[1];
    RuntimeDiag_AudioBlockBegin();
    if (g_sdr.tx_mode) {
      DSP_ProcessTX(&g_dsp, s_tx_buf, CSDR_AUDIO_BLOCK_SIZE);
    } else {
      DSP_Process(&g_dsp, s_rx_buf, s_tx_buf, CSDR_AUDIO_BLOCK_SIZE);
    }
    RuntimeDiag_AudioBlockEnd();
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
    /* s_tx_buf: aligned(32), size 2048 B — 32-byte aligned ✓ */
    SCB_CleanDCache_by_Addr((uint32_t*)s_tx_buf,
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
    RuntimeDiag_TxHalfFilled(0U);
    RuntimeDiag_RxHalfConsumed(0U, rx_seq);
    s_rx_done_seq[0] = rx_seq;
  }
  rx_seq = s_rx_ready_seq[1];
  if (rx_seq != s_rx_done_seq[1]) {
    /* Pong half: byte offset = 256×2×4 = 2048 B from base → still 32-byte aligned.
     * Size 2048 B — 32-byte aligned ✓ */
    SCB_InvalidateDCache_by_Addr((uint32_t*)(s_rx_buf + CSDR_AUDIO_BLOCK_SIZE*2),
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
    if (g_usb_audio.usb_streaming)
      USB_Audio_WriteRX(&g_usb_audio,
                         s_rx_buf + CSDR_AUDIO_BLOCK_SIZE*2,
                         CSDR_AUDIO_BLOCK_SIZE);
    RuntimeDiag_AudioBlockBegin();
    if (g_sdr.tx_mode) {
      DSP_ProcessTX(&g_dsp, s_tx_buf + CSDR_AUDIO_BLOCK_SIZE*2,
                     CSDR_AUDIO_BLOCK_SIZE);
    } else {
      DSP_Process(&g_dsp, s_rx_buf + CSDR_AUDIO_BLOCK_SIZE*2,
                   s_tx_buf + CSDR_AUDIO_BLOCK_SIZE*2, CSDR_AUDIO_BLOCK_SIZE);
    }
    RuntimeDiag_AudioBlockEnd();
    /* s_tx_buf pong: byte offset 2048, size 2048 — 32-byte aligned ✓ */
    SCB_CleanDCache_by_Addr((uint32_t*)(s_tx_buf + CSDR_AUDIO_BLOCK_SIZE*2),
        CSDR_AUDIO_BLOCK_SIZE * 2 * (int32_t)sizeof(int32_t));
    RuntimeDiag_TxHalfFilled(1U);
    RuntimeDiag_RxHalfConsumed(1U, rx_seq);
    s_rx_done_seq[1] = rx_seq;
  }

}

void CSDR_Loop(void)
{
  RuntimeDiag_MainLoopBeat();

  csdr_process_audio_pending();

  /* Input */
  csdr_handle_encoder();
  csdr_handle_keys();

  /* Timed tasks */
  static uint32_t t_analog=0, t_fan=0, t_pwr=0, t_disp=0, t_cat=0, t_wf=0;
  uint32_t now = HAL_GetTick();

  if (now - t_analog >= 100U) {
    t_analog = now;
    Analog_Update();
    SDR_UI_UpdateSMeter_SetVoltage((float)g_analog.voltage_mv * 0.001f);
  }
  if (now - t_fan    >= 1000U){ t_fan    = now; Fan_Update(g_analog.temp_c); }
  if (now - t_pwr    >= 100U) { t_pwr    = now; PWR_Poll(); }
  /* Waterfall: 25 fps cap in RX.  Freeze it in TX so the 320x62 two-split
   * SPI push (~39.7 kB plus cache clean/DMA wait) cannot periodically block
   * TX audio generation at the same cadence as WSJT-X audio deadlines. */
  if (!g_sdr.tx_mode && (now - t_wf >= CSDR_UI_WF_RX_PERIOD_MS)) {
    t_wf = now;
    if (!dbg_disable_lcd_dma && !Diag_IsActive()) {
      csdr_process_audio_pending();
      RuntimeDiag_UiRenderBegin();
      csdr_update_waterfall();
      RuntimeDiag_UiRenderEnd();
    }
  } else if (g_sdr.tx_mode) {
    t_wf = now;
  }

  uint32_t disp_period = g_sdr.tx_mode ? CSDR_UI_DISPLAY_TX_PERIOD_MS
                                       : CSDR_UI_DISPLAY_RX_PERIOD_MS;
  bool disp_due = (now - t_disp >= disp_period);
  bool dirty_due = g_sdr.display_dirty && !g_sdr.tx_mode;
  if (g_sdr.display_dirty && g_sdr.tx_mode &&
      (now - t_disp >= CSDR_UI_TX_DIRTY_MIN_MS)) {
    dirty_due = true;
  }
  if (disp_due || dirty_due) {
    t_disp = now;
    if (!dbg_disable_lcd_dma && !Diag_IsActive()) {
      csdr_process_audio_pending();
      RuntimeDiag_UiRenderBegin();
      csdr_refresh_display();
      RuntimeDiag_UiRenderEnd();
    }
  }
  RuntimeDiag_ServiceSlow(now);
  Diag_Process();
  RuntimeDiag_WatchdogRefreshIfHealthy(now);
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
    s_rx_ready_seq[0] = RuntimeDiag_RxHalfIsr(0U);
    dbg_cb_half_count++;
    /* USB_Audio_WriteRX is intentionally NOT called here.
     * Calling USB functions from a DMA ISR starves the USB OTG IRQ when the
     * host opens a stream, causing the USB audio endpoint to hard-lock.
     * The main loop (CSDR_Loop) calls WriteRX after the RX sequence advances. */
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *h)
{
  if (h->Instance == SAI1_Block_B) {
    s_rx_ready_seq[1] = RuntimeDiag_RxHalfIsr(1U);
    dbg_cb_full_count++;
    /* USB_Audio_WriteRX moved to CSDR_Loop – see HAL_SAI_RxHalfCpltCallback. */
  }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *h)
{
  if (h->Instance == SAI1_Block_A) {
    RuntimeDiag_TxHalfConsumedIsr(0U, g_sdr.tx_mode);
  }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *h)
{
  if (h->Instance == SAI1_Block_A) {
    RuntimeDiag_TxHalfConsumedIsr(1U, g_sdr.tx_mode);
  }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *h)
{
  (void)h;
  dbg_sai_error_cnt++;
  RuntimeDiag_SetFault(FAULT_CODEC);
  /* Stop both blocks */
  HAL_SAI_DMAStop(&hsai_BlockA1);   /* TX master first */
  HAL_SAI_DMAStop(&hsai_BlockB1);   /* then RX slave  */
  /* Restart TX master first – it generates BCLK/LRCK for the slave */
  RuntimeDiag_TxHalfFilled(0U);
  RuntimeDiag_TxHalfFilled(1U);
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
  DSP_SetFrequency(&g_dsp, csdr_nco_freq(), f, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, f + g_sdr.lo_offset_hz);
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
    DSP_SetFrequency(&g_dsp, csdr_nco_freq(), g_sdr.freq_hz, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
    if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
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
            .xtal_ppm        = g_sdr.xtal_ppm,
            .iq_gain         = g_sdr.iq_gain,
            .iq_phase        = g_sdr.iq_phase,
            .dc_i_offset     = g_sdr.dc_i_offset,
            .dc_q_offset     = g_sdr.dc_q_offset,
            .audio_gain_db   = g_sdr.audio_gain_db,
            .mic_gain        = g_sdr.mic_gain,
            .smeter_offset_db= g_sdr.smeter_offset_db,
            .lo_offset_hz    = g_sdr.lo_offset_hz,
          };
          if (Cal_Run(&g_lcd, &cp)) {
            g_sdr.xtal_ppm        = cp.xtal_ppm;
            g_sdr.iq_gain         = cp.iq_gain;
            g_sdr.iq_phase        = cp.iq_phase;
            g_sdr.dc_i_offset     = cp.dc_i_offset;
            g_sdr.dc_q_offset     = cp.dc_q_offset;
            g_sdr.audio_gain_db   = cp.audio_gain_db;
            g_sdr.mic_gain        = cp.mic_gain;
            g_sdr.smeter_offset_db= cp.smeter_offset_db;
            g_sdr.lo_offset_hz    = cp.lo_offset_hz;
            DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
            if (g_sdr.si5351_ok) {
              g_si5351.xtal_hz = (uint32_t)((int32_t)SI5351_XTAL_HZ +
                SI5351_XTAL_HZ / 1000000L * g_sdr.xtal_ppm);
              SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
            }
          }
        } else if (strcmp(name, "SWR Scan") == 0) {
          SWR_Scan_Run(&g_lcd);
        }
        g_sdr.display_dirty = true;
      } else {
        Menu_Select(&g_menu);
      }
      return;
    }
    g_sdr.mode   = (SDR_Mode_t)((g_sdr.mode + 1U) % MODE_COUNT);
    g_sdr.bw_hz  = default_bw_for_mode(g_sdr.mode);
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
    DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
    SDR_UI_SetSpecZoom(&g_lcd, default_zoom_for_mode(g_sdr.mode));
    g_sdr.display_dirty = true;
  }
  if (Encoder_GetLongPress(&g_encoder)) {
    /* Long press: cycle spectrum zoom ±24k → ±18k → ±12k → ±6k → ±3k → ±24k */
    uint8_t z = (uint8_t)((SDR_UI_GetSpecZoom() + 1U) % SPEC_ZOOM_COUNT);
    SDR_UI_SetSpecZoom(&g_lcd, z);
  }
}

static void csdr_handle_keys(void)
{
  Key_Poll(&k_menu); Key_Poll(&k_f1);   Key_Poll(&k_f2); Key_Poll(&k_f3);
  Key_Poll(&k_f4);   Key_Poll(&k_band); Key_Poll(&k_mode); Key_Poll(&k_ptt);

  if (Key_Press(&k_menu)) {
    g_sdr.display_dirty = false;  /* prevent status panel overwriting menu */
    if (!Menu_IsOpen(&g_menu))
      Menu_LoadFromSDR(&g_menu,
        g_sdr.agc_fast, g_sdr.nb_on, g_sdr.nr_on, g_sdr.rit_hz,
        g_sdr.volume, g_sdr.squelch, (uint32_t)g_sdr.step,
        g_sdr.att_db, g_sdr.band_idx, (uint8_t)g_sdr.mode,
        g_sdr.usb_mode, SDR_UI_GetSpecZoom(), menu_apply_cb);
    Menu_Toggle(&g_menu);
    if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty = true;
  }

  /* F1: menu UP / Volume Down (hold-repeat while held) */
  if (Key_PressOrRepeat(&k_f1)) {
    if (Menu_IsOpen(&g_menu)) Menu_Up(&g_menu);
    else {
      uint8_t v = (g_sdr.volume >= 2U) ? (g_sdr.volume - 2U) : 0U;
      cat_set_volume(v);
    }
  }

  /* F2: menu DOWN / Volume Up (hold-repeat while held) */
  if (Key_PressOrRepeat(&k_f2)) {
    if (Menu_IsOpen(&g_menu)) Menu_Down(&g_menu);
    else {
      uint8_t v = (g_sdr.volume <= 98U) ? (g_sdr.volume + 2U) : 100U;
      cat_set_volume(v);
    }
  }

  /* F3: menu confirm / VFO A↔B swap */
  if (Key_Press(&k_f3)) {
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
            .xtal_ppm        = g_sdr.xtal_ppm,
            .iq_gain         = g_sdr.iq_gain,
            .iq_phase        = g_sdr.iq_phase,
            .dc_i_offset     = g_sdr.dc_i_offset,
            .dc_q_offset     = g_sdr.dc_q_offset,
            .audio_gain_db   = g_sdr.audio_gain_db,
            .mic_gain        = g_sdr.mic_gain,
            .smeter_offset_db= g_sdr.smeter_offset_db,
            .lo_offset_hz    = g_sdr.lo_offset_hz,
          };
          if (Cal_Run(&g_lcd, &cp)) {
            g_sdr.xtal_ppm        = cp.xtal_ppm;
            g_sdr.iq_gain         = cp.iq_gain;
            g_sdr.iq_phase        = cp.iq_phase;
            g_sdr.dc_i_offset     = cp.dc_i_offset;
            g_sdr.dc_q_offset     = cp.dc_q_offset;
            g_sdr.audio_gain_db   = cp.audio_gain_db;
            g_sdr.mic_gain        = cp.mic_gain;
            g_sdr.smeter_offset_db= cp.smeter_offset_db;
            g_sdr.lo_offset_hz    = cp.lo_offset_hz;
            DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
            if (g_sdr.si5351_ok) {
              g_si5351.xtal_hz = (uint32_t)((int32_t)SI5351_XTAL_HZ +
                SI5351_XTAL_HZ / 1000000L * g_sdr.xtal_ppm);
              SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
            }
          }
        } else if (strcmp(name, "SWR Scan") == 0) {
          SWR_Scan_Run(&g_lcd);
        }
        g_sdr.display_dirty = true;
      } else {
        Menu_Confirm(&g_menu);
      }
    } else {
      csdr_vfo_swap();   /* F3 outside menu: swap VFO A↔B */
    }
  }

  /* F4: Back / Exit menu  –or–  copy active VFO to inactive */
  if (Key_Press(&k_f4)) {
    if (Menu_IsOpen(&g_menu)) {
      Menu_Back(&g_menu);
      if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty = true;
    } else {
      csdr_vfo_copy_to_b();  /* F4 outside menu: copy active → inactive VFO */
    }
  }

  if (Key_Press(&k_band))
    csdr_apply_band(BPF_BandUp(g_sdr.band_idx));

  if (Key_Press(&k_mode)) {
    g_sdr.mode  = (SDR_Mode_t)((g_sdr.mode + 1U) % MODE_COUNT);
    g_sdr.bw_hz = default_bw_for_mode(g_sdr.mode);
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
    DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
    SDR_UI_SetSpecZoom(&g_lcd, default_zoom_for_mode(g_sdr.mode));
    g_sdr.display_dirty = true;
  }

  if (Key_Press(&k_ptt)) {
    cat_set_tx(!g_sdr.tx_mode);
  }
}

static void csdr_update_waterfall(void)
{
  /* UI path only — never called from DSP ping/pong.
   * Consume all pending lines, draw exactly 1 row.  Non-blocking: DMA runs
   * in background, CS deasserted from HAL_SPI_TxCpltCallback. */
  if (g_sdr.tx_mode || g_dsp.wf_lines == 0U || Menu_IsOpen(&g_menu)) return;
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_WATERFALL);
  g_dsp.wf_lines = 0U;                   /* drop extras accumulated since last tick */
  SDR_UI_DrawWaterfall(&g_lcd, g_dsp.fft_mag_db, DSP_FFT_SIZE);
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_WATERFALL);
}

static void csdr_refresh_display(void)
{
  bool menu_open = Menu_IsOpen(&g_menu);

  /* Spectrum: RX only.  During TX the spectrum/waterfall region is the largest
   * LCD workload (68 full-width rows), so leave it frozen and reserve main-loop
   * time for filling the next SAI TX half-buffer. */
  if (!g_sdr.tx_mode && g_dsp.fft_ready && !menu_open) {
    g_dsp.fft_ready = false;

    float bw_lo_ratio = 0.0f, bw_hi_ratio = 0.0f;
    if (g_dsp.sample_rate > 0U && g_sdr.bw_hz > 0U)
    {
      float full = (float)g_sdr.bw_hz / (float)g_dsp.sample_rate;
      float half = full * 0.5f;
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
    RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_SPECTRUM);
    SDR_UI_DrawSpectrum(&g_lcd, g_dsp.fft_mag_db, DSP_FFT_SIZE,
                        bw_lo_ratio, bw_hi_ratio, NULL);
    RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_SPECTRUM);
  }

  if (g_sdr.display_dirty) {
    g_sdr.display_dirty = false;
    SDR_UI_State_t ui = {0};
    ui.freq_hz   = g_sdr.freq_hz;       ui.mode      = (uint8_t)g_sdr.mode;
    ui.band_idx  = g_sdr.band_idx;      ui.volume    = g_sdr.volume;
    ui.squelch   = g_sdr.squelch;       ui.step      = (uint32_t)g_sdr.step;
    ui.agc_fast  = g_sdr.agc_fast;      ui.nb_on     = g_sdr.nb_on;
    ui.nr_on     = g_sdr.nr_on;         ui.rit_hz    = g_sdr.rit_hz;
    ui.tx_mode   = g_sdr.tx_mode;       ui.si5351_ok = g_sdr.si5351_ok;
    ui.signal_db = g_dsp.signal_power_db;
    ui.bw_hz     = g_sdr.bw_hz;         ui.voltage   = (float)g_analog.voltage_mv * 0.001f;
    ui.att_db    = g_sdr.att_db;        ui.mic_gain  = g_sdr.mic_gain;
    ui.freq_b_hz = g_sdr.vfo_b.freq_hz; /* inactive VFO shown in sub-line */
    ui.active_vfo = g_sdr.active_vfo;

    /* During TX, keep dirty redraws to the smallest safe region.  A full
     * top bar is three SPI windows (~41 kB) and knob events can otherwise
     * land inside the 5.33 ms TX half-buffer deadline. */
    if (g_sdr.tx_mode) {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
      SDR_UI_DrawHeader(&g_lcd, &ui);
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
    } else {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_STATUS_BAR);
      SDR_UI_DrawTopBar(&g_lcd, &ui);
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_STATUS_BAR);
    }

    /* StatusPanel (y=62+) và S-meter chỉ khi menu ĐÓNG
     * Nếu menu đang mở: tuyệt đối không ghi đè vùng y=62..
     * In TX, avoid repainting sidebars on the dirty transition; they are
     * cosmetic and add two extra 60x82 SPI pushes while audio timing is tight. */
    if (!menu_open && !g_sdr.tx_mode) {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_STATUS_BAR);
      SDR_UI_DrawStatusPanel(&g_lcd, &ui);
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_STATUS_BAR);
    } else if (menu_open) {
      /* Menu đang mở: re-render để đảm bảo không bị xóa */
      Menu_Render(&g_menu);
    }
  } else if (!menu_open) {
    if (g_sdr.tx_mode) {
      float alc_norm = (float)g_analog.alc_percent * (1.0f / 100.0f);
      float swr      = (float)g_analog.swr_x100    * (1.0f / 100.0f);
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
      SDR_UI_UpdateTXMeters(&g_lcd, alc_norm, swr);
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
    } else {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
      SDR_UI_UpdateSMeter(&g_lcd, g_dsp.signal_power_db);
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
    }
  }
}

static uint8_t default_zoom_for_mode(SDR_Mode_t m)
{
  switch (m) {
    case MODE_CW:            return 4U;  /* ±3k */
    case MODE_USB:
    case MODE_LSB:           return 3U;  /* ±6k */
    case MODE_AM:
    case MODE_FM:
    default:                 return 0U;  /* ±24k */
  }
}

static uint32_t default_bw_for_mode(SDR_Mode_t m)
{
  switch (m) {
    case MODE_AM:  return 6000U;
    case MODE_FM:  return 15000U;
    case MODE_USB:
    case MODE_LSB: return 3000U;
    case MODE_CW:  return 500U;
    default:       return 4000U;
  }
}

static void menu_apply_cb(void)
{
  bool agc, nb, nr; int16_t rit;
  uint8_t vol, sq, att, band, mode, usb, zoom; uint32_t step;
  Menu_SaveToSDR(&g_menu, &agc, &nb, &nr, &rit,
                  &vol, &sq, &step, &att, &band, &mode, &usb, &zoom);
  g_sdr.agc_fast = agc; g_sdr.nb_on = nb; g_sdr.nr_on = nr;
  AGC_SetSpeed(&g_dsp.agc, agc, CSDR_AUDIO_SAMPLE_RATE);
  g_sdr.rit_hz = rit;   cat_set_volume(vol); g_sdr.squelch = sq;
  g_sdr.step = (FreqStep_t)step;
  if (att != g_sdr.att_db) { PE4302_SetAttn_dB(&g_att, att); g_sdr.att_db = att; }
  if (band != g_sdr.band_idx) csdr_apply_band(band);
  if (mode != (uint8_t)g_sdr.mode) {
    g_sdr.mode  = (SDR_Mode_t)mode;
    g_sdr.bw_hz = default_bw_for_mode(g_sdr.mode);
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
    DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
  }
  g_sdr.usb_mode = usb;
  if (zoom != SDR_UI_GetSpecZoom()) SDR_UI_SetSpecZoom(&g_lcd, zoom);
  g_sdr.display_dirty = true;
}

/* ══════════════════════════════════════════════════════════
 *  Dual-VFO helpers
 * ══════════════════════════════════════════════════════════ */

/* Swap active ↔ inactive VFO and apply new active state to hardware */
static void csdr_vfo_swap(void)
{
  VFO_State_t tmp = {
    .freq_hz  = g_sdr.freq_hz,
    .mode     = g_sdr.mode,
    .band_idx = g_sdr.band_idx,
    .step     = g_sdr.step,
    .rit_hz   = g_sdr.rit_hz,
    .bw_hz    = g_sdr.bw_hz,
  };

  g_sdr.freq_hz  = g_sdr.vfo_b.freq_hz;
  g_sdr.mode     = g_sdr.vfo_b.mode;
  g_sdr.band_idx = g_sdr.vfo_b.band_idx;
  g_sdr.step     = g_sdr.vfo_b.step;
  g_sdr.rit_hz   = g_sdr.vfo_b.rit_hz;
  g_sdr.bw_hz    = g_sdr.vfo_b.bw_hz;

  g_sdr.vfo_b    = tmp;
  g_sdr.active_vfo ^= 1U;
  /* Keep CAT routing state in sync so VS; query and AI IF-frame reflect reality */
  g_cat.active_vfo = g_sdr.active_vfo;

  BPF_SetBand(g_sdr.band_idx);
  LPF_SetBand(g_sdr.band_idx);
  DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
  DSP_SetFrequency(&g_dsp, csdr_nco_freq(), g_sdr.freq_hz, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
  SDR_UI_SetSpecZoom(&g_lcd, default_zoom_for_mode(g_sdr.mode));
  g_sdr.display_dirty = true;
}

/* Copy active VFO state into the inactive VFO slot (A→B when A active, B→A when B active) */
static void csdr_vfo_copy_to_b(void)
{
  g_sdr.vfo_b.freq_hz  = g_sdr.freq_hz;
  g_sdr.vfo_b.mode     = g_sdr.mode;
  g_sdr.vfo_b.band_idx = g_sdr.band_idx;
  g_sdr.vfo_b.step     = g_sdr.step;
  g_sdr.vfo_b.rit_hz   = g_sdr.rit_hz;
  g_sdr.vfo_b.bw_hz    = g_sdr.bw_hz;
  g_sdr.display_dirty  = true;
}

/* ── VFO-B + active-VFO CAT callbacks ──────────────────────────────────────
 * These keep g_sdr.vfo_b and g_sdr.active_vfo in sync with CAT commands
 * so UI state and Hamlib/flrig/WSJT-X always agree.
 * ─────────────────────────────────────────────────────────────────────────*/
static void     cat_set_vfo_b_freq(uint32_t hz) { g_sdr.vfo_b.freq_hz = hz; g_sdr.display_dirty = true; }
static void     cat_set_vfo_b_mode(uint8_t m)   { g_sdr.vfo_b.mode = (SDR_Mode_t)m; g_sdr.display_dirty = true; }
static void     cat_set_vfo_b_bw(uint32_t hz)   { g_sdr.vfo_b.bw_hz = hz; g_sdr.display_dirty = true; }
static uint32_t cat_get_vfo_b_freq(void)        { return g_sdr.vfo_b.freq_hz; }
static uint8_t  cat_get_vfo_b_mode(void)        { return (uint8_t)g_sdr.vfo_b.mode; }
static uint32_t cat_get_vfo_b_bw(void)          { return g_sdr.vfo_b.bw_hz; }

/* VS/FR/DC handler: triggers a hardware VFO swap only when the selection differs
 * from the current radio state, so VS1;VS1; is idempotent. */
static void cat_set_active_vfo(uint8_t vfo)
{
  if (vfo != g_sdr.active_vfo) csdr_vfo_swap();
}
static uint8_t cat_get_active_vfo(void) { return g_sdr.active_vfo; }

/* CAT callbacks */
static void     cat_set_freq(uint32_t f) { g_sdr.freq_hz=f; DSP_SetFrequency(&g_dsp,csdr_nco_freq(),f,g_sdr.lo_offset_hz,CSDR_AUDIO_SAMPLE_RATE); if(g_sdr.si5351_ok)SI5351_SetQSDFrequency(&g_si5351,f+g_sdr.lo_offset_hz); g_sdr.display_dirty=true; }
static void     cat_set_mode(uint8_t m)  { g_sdr.mode=(SDR_Mode_t)m; DSP_SetMode(&g_dsp,g_sdr.mode,CSDR_AUDIO_SAMPLE_RATE); DSP_SetBW(&g_dsp,(float)g_sdr.bw_hz); SDR_UI_SetSpecZoom(&g_lcd,default_zoom_for_mode(g_sdr.mode)); g_sdr.display_dirty=true; }
static void cat_set_tx(bool tx)
{
  if (tx == g_sdr.tx_mode) return;  /* no change */
  g_sdr.tx_mode = tx;
  if (tx) {
    /* TX sequence: switch T/R relay.
     * Shared CLK0 LO (4× RF) already running — no Si5351 change needed. */
    HAL_Delay(2);                                  /* 2ms settle   */
    HAL_GPIO_WritePin(TR_SW_GPIO_Port, TR_SW_Pin, GPIO_PIN_SET);
  } else {
    /* RX sequence: switch T/R relay back.
     * Shared LO was never changed during TX — no Si5351 restore needed. */
    HAL_GPIO_WritePin(TR_SW_GPIO_Port, TR_SW_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    WM8731_SetMute(&hi2c1, WM8731_I2C_ADDR, false);  /* Ensure HP unmuted */
  }
  SDR_UI_UpdateSMeter_SetTX(tx);
  g_sdr.display_dirty = true;
}
static void     cat_set_att(uint8_t lv)  { static const uint8_t m[]={0,6,12,18}; PE4302_SetAttn_dB(&g_att,(lv<4)?m[lv]:0); g_sdr.att_db=g_att.current_atten_db; }

/* Scale 0-100 internal volume → WM8731 HP register.
 * Range 90-121 (−31 dB to 0 dB) so 100% = 0 dB reference.
 * vol==0 → hardware mute (register < 0x30). */
static void cat_set_volume(uint8_t vol)
{
  if (vol > 100U) vol = 100U;
  g_sdr.volume = vol;
  uint8_t wm_vol = (vol == 0U) ? 0x2FU
                                : (uint8_t)(90U + ((uint16_t)vol * 31U / 100U));
  WM8731_SetVolume(&hi2c1, WM8731_I2C_ADDR, wm_vol, wm_vol);
  g_sdr.display_dirty = true;
}
static void     cat_set_nr(bool on)       { g_sdr.nr_on = on; g_sdr.display_dirty = true; }
static void     cat_set_nb(bool on)       { g_sdr.nb_on = on; g_sdr.display_dirty = true; }
static void cat_set_bw(uint32_t hz)
{
    if (hz < 100U)   hz = 100U;
    if (hz > 24000U) hz = 24000U;
    g_sdr.bw_hz = hz;
    DSP_SetBW(&g_dsp, (float)hz);
    g_sdr.display_dirty = true;
}
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
static uint32_t cat_get_bw(void)          { return g_sdr.bw_hz; }
static bool     cat_get_agc_fast(void)    { return g_sdr.agc_fast; }
static uint8_t  cat_get_squelch(void)     { return g_sdr.squelch; }
static int32_t  cat_get_rit_hz(void)      { return (int32_t)g_sdr.rit_hz; }
static uint32_t cat_get_step(void)        { return (uint32_t)g_sdr.step; }
static void cat_set_rit_hz(int32_t hz)
{
  if (hz > 9999)  hz = 9999;
  if (hz < -9999) hz = -9999;
  g_sdr.rit_hz = (int16_t)hz;
  g_sdr.display_dirty = true;
}
static void cat_set_step(uint32_t hz)
{
  /* Map to nearest valid FreqStep_t value */
  FreqStep_t st;
  if      (hz >= 100000U) st = STEP_100K;
  else if (hz >=  10000U) st = STEP_10K;
  else if (hz >=   1000U) st = STEP_1K;
  else if (hz >=    100U) st = STEP_100;
  else if (hz >=     10U) st = STEP_10;
  else                    st = STEP_1;
  g_sdr.step = st;
  g_sdr.display_dirty = true;
}

static void cat_set_if_shift(int32_t hz)
{
  if (hz >  9999) hz =  9999;
  if (hz < -9999) hz = -9999;
  g_sdr.if_shift_hz = (int16_t)hz;
  DSP_SetFrequency(&g_dsp, csdr_nco_freq(), g_sdr.freq_hz, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  g_sdr.display_dirty = true;
}

static int32_t cat_get_if_shift(void) { return (int32_t)g_sdr.if_shift_hz; }
/* ── Audio buffer accessors (expose main.c static buffers) ── */
extern int32_t tx_buf[];
extern int32_t rx_buf[];
extern volatile uint8_t dsp_ping;
extern volatile uint8_t dsp_pong;

int32_t *CSDR_GetTxBuf(void) { return tx_buf; }
int32_t *CSDR_GetRxBuf(void) { return rx_buf; }
void CSDR_ClearDspFlags(void) { dsp_ping = 0; dsp_pong = 0; }
