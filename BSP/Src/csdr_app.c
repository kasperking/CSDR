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
#include "input_scan.h"
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
extern SPI_HandleTypeDef  hspi3;    /* Flash */
extern SAI_HandleTypeDef  hsai_BlockA1;
extern SAI_HandleTypeDef  hsai_BlockB1;
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c2;    /* PCA9555 button expander (I2C2: PB10/PB11) */
extern TIM_HandleTypeDef  htim3;    /* Encoder — TIM3 CH1=PB4, CH2=PB5 */
extern TIM_HandleTypeDef  htim8;    /* Backlight — TIM8_CH4 = PC9 */
extern TIM_HandleTypeDef  htim17;   /* Fan — TIM17_CH1 = PB9 */
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern ADC_HandleTypeDef  hadc3;

/* ══════════════════════════════════════════════════════════
 *  Private state
 * ══════════════════════════════════════════════════════════ */
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
  .nb_on         = false,   /* NB disabled by default */
  .nb_level      = 50U,     /* moderate intensity; raise for more aggressive blanking */
  .display_dirty = DIRTY_ALL,
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
extern volatile uint32_t dbg_usb_reset_cnt;   /* Bus reset count (1=normal attach)      */
extern volatile uint32_t dbg_usb_suspend_cnt; /* Host sent SUSPEND (PC sleep/idle)      */
extern volatile uint32_t dbg_usb_resume_cnt;  /* Host sent RESUME (should pair suspend) */
extern volatile uint32_t dbg_cdc_rx_pkts;     /* CDC OUT packets received from host     */
extern volatile uint32_t dbg_cdc_tx_pkts;     /* CDC IN packets queued to host          */
extern volatile uint32_t dbg_cdc_tx_drop;     /* CDC TX returned USBD_BUSY — deferred   */
extern volatile uint32_t dbg_cdc_busy_max;    /* Max consecutive BUSY run length        */
extern volatile uint32_t dbg_cdc_fail_cnt;    /* USBD_LL_Transmit returned FAIL (not BUSY) */
extern volatile uint32_t dbg_comp_datain_cnt; /* CDC DataIn fire count (must track tx_pkts) */
/* CAT transport counters – defined in usb_cat.c */
extern volatile uint32_t dbg_cat_rx_bytes;         /* Total bytes into CAT_Receive from ISR      */
extern volatile uint32_t dbg_cat_tx_bytes;         /* Total bytes sent to CDC_Transmit_FS        */
extern volatile uint32_t dbg_cat_fifo_drop;        /* Chars dropped: TX FIFO full (512B max)     */
extern volatile uint32_t dbg_cat_parse_calls;      /* CAT_Process calls (≈ 100/s expected)       */
/* CAT protocol diagnostics */
extern volatile uint32_t dbg_cat_unknown_cmds;     /* ?; responses — unrecognised opcode         */
extern volatile uint32_t dbg_cat_malformed_frames; /* ?; responses — known opcode, bad params    */
extern volatile uint32_t dbg_cat_blocked_updates;  /* SET while previous dirty flag still pending*/
extern volatile uint32_t dbg_cat_partial_timeouts; /* partial cmds discarded after 200ms idle    */
extern volatile uint32_t dbg_cat_max_cmd_len;      /* peak parser_len (overflow risk if → 127)   */
extern volatile uint32_t dbg_cat_parse_latency_us; /* last CAT_Process duration µs (DWT-based)   */
extern char              dbg_cat_last_cmd[];        /* last complete command, NUL-terminated       */
extern char              dbg_cat_last_resp[];       /* last non-empty response enqueued            */

/* Waterfall adaptive-skip counters */
static volatile uint32_t dbg_wf_skip_count  = 0U; /* waterfall frames suppressed by overload-hysteresis */
static volatile uint8_t  dbg_ui_load_high   = 0U; /* 1 = system load too high for waterfall */
/* Spectrum adaptive-skip counters */
static volatile uint32_t dbg_spec_skip_count = 0U; /* spectrum frames suppressed by overload-hysteresis */

/* USB/SAI drift mitigation */
static volatile uint32_t dbg_drift_corrections       = 0U; /* TX ring resets applied for chronic underrun */
static volatile uint32_t dbg_rx_drift_corrections    = 0U; /* RX ring trims applied for chronic overrun drift */
static volatile uint32_t dbg_rx_underdrift_corrections = 0U; /* RX ring refills applied for chronic underflow drift */

/* Set to 1 in debugger to skip waterfall/LCD DMA while diagnosing USB audio. */
static volatile uint8_t dbg_disable_lcd_dma = 0;

/* Forward declarations for functions used before their definition */
static void csdr_apply_volume(uint8_t vol);
static void csdr_apply_tx(void);

#define CSDR_UI_SPEC_RX_PERIOD_MS    75U    /* ~13 fps spectrum in RX — decoupled from waterfall */
#define CSDR_UI_WF_RX_PERIOD_MS      75U    /* ~13 fps waterfall in RX */
#define UI_OVERLOAD_DECAY_MS         150U   /* suppress waterfall; 2 ticks to enter, 2 to recover */
#define CSDR_UI_DISPLAY_RX_PERIOD_MS 100U   /* 10 fps dirty-zone + meter refresh */
#define CSDR_UI_DISPLAY_TX_PERIOD_MS 1000U  /* 1 Hz compact TX meter refresh */
#define CSDR_UI_TX_DIRTY_MIN_MS      1000U  /* defer knob/menu redraws while TX audio is time-critical */


/* ── Function key state machines ── */
static Key_t k_menu, k_f1, k_f2, k_f3, k_f4, k_band, k_mode, k_ptt;

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
static void csdr_update_spectrum(void);
static void csdr_update_waterfall(void);
static void csdr_refresh_display(void);
static void menu_apply_cb(void);
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

  /* Backlight: TIM8_CH4 (PC9).  SAI DMA IRQs: highest priority. */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0U, 0U);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 800U);

  /* Flash: load settings */
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
  }
  SDR_UI_Init();
  SDR_UI_DrawFrame(CSDR_AUDIO_SAMPLE_RATE, DSP_FFT_SIZE);

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
  DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetIFShift(&g_dsp, (int32_t)g_sdr.if_shift_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
  DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
  AGC_SetSpeed(&g_dsp.agc, g_sdr.agc_fast, CSDR_AUDIO_SAMPLE_RATE);
  DSP_NB_Set(&g_dsp, g_sdr.nb_on, g_sdr.nb_level);

  /* Encoder – TIM3 quadrature (PB4/PB5), initialised as encoder in MX_TIM3_Init */
  Encoder_Init(&g_encoder, &htim3);

  /* PCA9555 button expander – all function keys on I2C2 */
  Input_Init();
  Key_InitPCA(&k_menu, &g_pca9555_raw, PCA_BIT_MENU);
  Key_InitPCA(&k_f1,   &g_pca9555_raw, PCA_BIT_F1);
  Key_InitPCA(&k_f2,   &g_pca9555_raw, PCA_BIT_F2);
  Key_InitPCA(&k_f3,   &g_pca9555_raw, PCA_BIT_F3);
  Key_InitPCA(&k_f4,   &g_pca9555_raw, PCA_BIT_F4);
  Key_InitPCA(&k_band, &g_pca9555_raw, PCA_BIT_BAND);
  Key_InitPCA(&k_mode, &g_pca9555_raw, PCA_BIT_MODE);
  Key_Init(&k_ptt, PTT_GPIO_Port, PTT_Pin);   /* PB12 – direct MCU */

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
  Menu_Init(&g_menu);

  /* Start SAI DMA (provides BCLK/LRCK to WM8731) */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* Start TX first - it's MASTER and generates BCLK/LRCK for RX */
  RuntimeDiag_TxHalfFilled(0U);
  RuntimeDiag_TxHalfFilled(1U);
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)s_tx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);
  HAL_Delay(10U);
  dbg_sai_init_ret = HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)s_rx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);
  HAL_Delay(10U);

  /* ── E. Boot-time IRQ priority safety check ───────────────────────────────
   * Verifies that SAI and DMA audio IRQs are strictly higher priority
   * (lower number) than USB OTG.  CubeMX regeneration reverts the OTG_FS
   * priority to 0, putting it on par with SAI — the most common regen bug.
   *
   * If the hierarchy is wrong, the FAULT_IRQ_CFG bit is set in the runtime
   * fault register so it appears in the diagnostic snapshot.  In debug builds
   * (NDEBUG not defined) we halt in the debugger so the violation is
   * impossible to miss; a watchdog reset is the release-build recovery.
   *
   * Priority encoding (STM32H7, NVIC_PRIORITYGROUP_4, __NVIC_PRIO_BITS=4):
   *   HAL_NVIC_SetPriority(x, N, 0) stores N<<4 in the IPR register.
   *   Numerically lower IPR value = higher urgency = must preempt USB. */
  {
    uint32_t sai_prio  = NVIC_GetPriority(SAI1_IRQn);
    uint32_t dma0_prio = NVIC_GetPriority(DMA1_Stream0_IRQn);
    uint32_t dma1_prio = NVIC_GetPriority(DMA1_Stream1_IRQn);
    uint32_t otg_prio  = NVIC_GetPriority(OTG_FS_IRQn);
    if (sai_prio >= otg_prio || dma0_prio >= otg_prio || dma1_prio >= otg_prio) {
      RuntimeDiag_SetFault(FAULT_IRQ_CFG);
#if !defined(NDEBUG)
      /* Halt in debugger.  Check usbd_conf.c USER CODE USB_OTG_FS_MspInit 1
       * for the HAL_NVIC_SetPriority(OTG_FS_IRQn, 2, 0) override — it is
       * the first thing CubeMX regen silently removes. */
      while ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) { __BKPT(0); }
#endif
    }
  }

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

/* CSDR_ProcessAudioPending — public wrapper around csdr_process_audio_pending.
 *
 * SAFE call sites: top-level CSDR_Loop body, or equivalent main-loop context
 *   where no LCD/FMC strip DMA is in flight and no outer audio call is active.
 *
 * DO NOT call from inside LCD strip loops (spec_push_partial, SDR_UI_DrawSpectrum
 * full-push loop, or any tight render loop).  Doing so caused a runaway overrun
 * explosion in testing: USB_Audio_WriteRX's BASEPRI critical section interacts
 * badly with the LCD DMA TC ISR timing, and repeated calls within a single
 * render trip the USB ring throttle path hundreds of times per second. */
void CSDR_ProcessAudioPending(void)
{
  csdr_process_audio_pending();
}

void CSDR_Loop(void)
{
  RuntimeDiag_MainLoopBeat();

  csdr_process_audio_pending();

  /* Input: refresh PCA9555 once, then dispatch encoder + keys */
  Input_Scan();
  csdr_handle_encoder();
  csdr_handle_keys();

  /* Timed tasks */
  static uint32_t t_analog=0, t_fan=0, t_pwr=0, t_disp=0, t_cat=0, t_wf=0, t_spec=0;
  uint32_t now = HAL_GetTick();

  if (now - t_analog >= 100U) {
    t_analog = now;
    Analog_Update();
    SDR_UI_UpdateSMeter_SetVoltage((int16_t)(g_analog.voltage_mv / 100));
  }
  if (now - t_fan    >= 1000U){ t_fan    = now; Fan_Update(g_analog.temp_c); }
  if (now - t_pwr    >= 100U) { t_pwr    = now; PWR_Poll(); }

  /* Spectrum: ~15 fps in RX.  Independent timer from waterfall — allows fast
   * peak-trace responsiveness without pulling the slower waterfall scroll rate.
   * Adaptive suppression mirrors the waterfall policy:
   *  a) rx_overrun_pending — ring overflow flagged this tick (read, not cleared —
   *     the waterfall block owns the clear on its 75 ms tick)
   *  b) rx ring occupancy > 75% — main-loop is falling behind
   *  c) RuntimeDiag UI-overload flag (set by waterfall hysteresis)
   * Hysteresis: once triggered, suppression holds for UI_OVERLOAD_DECAY_MS. */
  if (!g_sdr.tx_mode && (now - t_spec >= CSDR_UI_SPEC_RX_PERIOD_MS)) {
    t_spec = now;
    if (!dbg_disable_lcd_dma && !Diag_IsActive()) {
      static uint32_t s_spec_overload_ms = 0U;
      bool spec_ring_pressure = g_usb_audio.rx_overrun_pending ||
                                (g_usb_audio.rx_count > USB_AUDIO_OVERRUN_BYTES);
      if (spec_ring_pressure || RuntimeDiag_IsUiOverload()) {
        s_spec_overload_ms = UI_OVERLOAD_DECAY_MS;
      } else if (s_spec_overload_ms > CSDR_UI_SPEC_RX_PERIOD_MS) {
        s_spec_overload_ms -= CSDR_UI_SPEC_RX_PERIOD_MS;
      } else {
        s_spec_overload_ms = 0U;
      }
      if (s_spec_overload_ms > 0U) {
        /* Suppressed: yield audio but skip the 8 ms FMC render. */
        csdr_process_audio_pending();
        dbg_spec_skip_count++;
      } else {
        csdr_process_audio_pending();
        RuntimeDiag_UiRenderBegin();
        csdr_update_spectrum();
        RuntimeDiag_UiRenderEnd();
      }
    }
  } else if (g_sdr.tx_mode) {
    t_spec = now;
  }

  /* Waterfall: ~13 fps in RX.  Frozen in TX to avoid the 480×72 FMC push
   * (8.06ms) competing with SAI TX DMA fill at WSJT-X audio deadlines. */
  if (!g_sdr.tx_mode && (now - t_wf >= CSDR_UI_WF_RX_PERIOD_MS)) {
    t_wf = now;
    /* Adaptive waterfall skip: suppress FMC push only under sustained load.
     *
     * Entry requires 2 consecutive high-pressure ticks (≥150 ms) to avoid
     * suppressing on isolated overruns that are harmless on FMC 480×72.
     * Recovery is fast: 150 ms (2 ticks) after pressure clears.
     *
     * Trigger conditions:
     *  a) rx ring occupancy > 75% — main-loop audio falling behind (sustained)
     *  b) UI render time consistently excessive (>15 ms)
     *  c) loop stall >20 ms
     *  d) sustained overrun/underrun rate ≥3/s (isolated events are ignored)
     *
     * rx_overrun_pending is consumed here to prevent stale flag accumulation
     * but is NOT used as a direct suppression trigger (isolated overruns are
     * harmless on FMC hardware and were falsely killing the waterfall). */
    {
      static uint32_t s_wf_overload_ms   = 0U;
      static uint8_t  s_wf_overload_hits = 0U;  /* consecutive high-pressure ticks */

      /* a: ring occupancy — sustained audio pressure */
      bool ring_pressure = (g_usb_audio.rx_count > USB_AUDIO_OVERRUN_BYTES);
      if (g_usb_audio.rx_overrun_pending) g_usb_audio.rx_overrun_pending = false;

      /* b+c+d: slow-path diagnostics; isolated overruns not sufficient */
      RuntimeDiag_Snapshot_t snap;
      RuntimeDiag_GetSnapshot(&snap);
      bool diag_pressure = (snap.max_ui_us > 15000U ||
                            snap.max_loop_stall_us > 20000U ||
                            snap.rx_overrun_per_sec >= 3U ||
                            snap.tx_underrun_per_sec >= 3U);

      if (ring_pressure || diag_pressure) {
        if (s_wf_overload_hits < 2U) s_wf_overload_hits++;
        if (s_wf_overload_hits >= 2U) {
          s_wf_overload_ms = UI_OVERLOAD_DECAY_MS;
          dbg_ui_load_high = 1U;
        }
      } else {
        s_wf_overload_hits = 0U;
        if (s_wf_overload_ms > CSDR_UI_WF_RX_PERIOD_MS) {
          s_wf_overload_ms -= CSDR_UI_WF_RX_PERIOD_MS;
        } else {
          s_wf_overload_ms = 0U;
          dbg_ui_load_high = 0U;
        }
      }
      bool wf_suppressed = (s_wf_overload_ms > 0U);
      if (wf_suppressed) dbg_wf_skip_count++;
      SDR_UI_SetWaterfallSuppressed(wf_suppressed);
      RuntimeDiag_WfSkipReport(dbg_wf_skip_count, wf_suppressed);
    }
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
  /* ── D. USB/SAI clock drift mitigation ───────────────────────────────────
   * USB ISO delivers 1 packet/ms from the host 48 MHz PLL; SAI DMA consumes
   * samples from the MCO/PLL2 audio clock.  Small frequency differences
   * accumulate over minutes, pushing the TX ring toward chronic underrun.
   *
   * Detection: once per second, check whether USB is actively delivering
   * (usb_rx_frames ≥ 900) while the TX ring is consistently near-empty
   * (< 1 packet).  Three consecutive seconds of this pattern indicates
   * structural drift rather than a burst starve.
   *
   * Correction: reset tx_rd = tx_wr (ring appears empty) so the next USB
   * OUT packets fill a clean ring.  Penalty: ≤4 ms of DAC silence, which
   * is identical to the silence already produced by the underrun path.
   * After correction, the ring stabilises at its natural fill level.
   *
   * Uses BASEPRI = 0x20 (same as the TX-ring drain above) to protect
   * tx_count + tx_rd from USB_Audio_WriteTX running in USB IRQ. */
  {
    static uint32_t s_drift_check_ms        = 0U;
    static uint32_t s_drift_last_rx_frames  = 0U;
    static uint8_t  s_drift_underrun_streak = 0U;

    if (g_usb_audio.usb_streaming && now - s_drift_check_ms >= 1000U) {
      s_drift_check_ms = now;
      uint32_t rx_frames_this_sec = g_usb_audio.usb_rx_frames - s_drift_last_rx_frames;
      s_drift_last_rx_frames      = g_usb_audio.usb_rx_frames;

      if (rx_frames_this_sec >= 900U &&
          g_usb_audio.tx_count < USB_AUDIO_BYTES_PER_FRAME) {
        if (++s_drift_underrun_streak >= 3U) {
          __set_BASEPRI(0x20U);
          g_usb_audio.tx_rd    = g_usb_audio.tx_wr;
          g_usb_audio.tx_count = 0U;
          __set_BASEPRI(0U);
          dbg_drift_corrections++;
          s_drift_underrun_streak = 0U;
        }
      } else {
        s_drift_underrun_streak = 0U;
      }
    }
  }

  /* ── E2. USB/SAI RX ring drift correction ────────────────────────────────
   * The RX ring (SAI→USB host) fills slightly faster than the host consumes
   * when the MCU PLL2 audio clock runs above the USB host's reference.
   * At ±50 ppm mismatch: ~9.6 bytes/sec drift → ring reaches the 40-packet
   * overrun threshold in ~12 minutes, causing sustained rx_overrun events.
   *
   * Detection: rx_count above 12 packets (2304 B) for 2 consecutive seconds
   *   while the USB host is actively receiving (usb_tx_frames ≥ 900/sec).
   * Correction: under BASEPRI = 0x20, advance rx_rd to discard the excess
   *   and restore rx_count to 8 packets (1536 B).  Eight packets keeps the
   *   ring minimum (≈ 512 B before each WriteRX) above the 192-B underrun
   *   threshold, preventing the systematic ~166 Hz underrun pattern.
   * Penalty: < 0.8 ms of IQ data trimmed — imperceptible to SDR software.
   *
   * rx_rd is ordinarily owned by the USB OTG IRQ (ReadRXPacket).  Writing it
   * here under BASEPRI = 0x20 is safe: that IRQ (priority 2, encoded 0x20)
   * is masked for the duration of the update.  SAI/DMA (priority 0) never
   * touch rx_rd.  re-check rx_count inside the CS to avoid uint16 underflow
   * if the IRQ drained the ring between the outer check and the CS entry. */
  {
    static uint32_t s_rx_drift_check_ms        = 0U;
    static uint32_t s_rx_drift_last_tx_frames  = 0U;
    static uint8_t  s_rx_drift_high_streak     = 0U;

    if (g_usb_audio.usb_streaming && now - s_rx_drift_check_ms >= 1000U) {
      s_rx_drift_check_ms = now;
      uint32_t tx_frames_this_sec = g_usb_audio.usb_tx_frames - s_rx_drift_last_tx_frames;
      s_rx_drift_last_tx_frames   = g_usb_audio.usb_tx_frames;

      if (tx_frames_this_sec >= 900U &&
          g_usb_audio.rx_count > (12U * USB_AUDIO_BYTES_PER_FRAME)) {
        if (++s_rx_drift_high_streak >= 2U) {
          __set_BASEPRI(0x20U);
          if (g_usb_audio.rx_count > (8U * USB_AUDIO_BYTES_PER_FRAME)) {
            uint16_t excess =
              (uint16_t)(g_usb_audio.rx_count - 8U * USB_AUDIO_BYTES_PER_FRAME);
            g_usb_audio.rx_rd =
              (uint16_t)((g_usb_audio.rx_rd + excess) % USB_AUDIO_RING_SIZE);
            g_usb_audio.rx_count = (uint16_t)(8U * USB_AUDIO_BYTES_PER_FRAME);
            dbg_rx_drift_corrections++;
          }
          __set_BASEPRI(0U);
          s_rx_drift_high_streak = 0U;
        }
      } else {
        s_rx_drift_high_streak = 0U;
      }
    }
  }

  /* ── E3. RX ring underflow drift correction ──────────────────────────────
   * Mirror of E2 for the opposite drift direction: when the SAI PLL2 runs
   * slightly slower than the USB host's reference, rx_count drains to zero
   * and rx_underrun fires at ~1000 Hz indefinitely.
   *
   * Detection: rx_count < 2 packets (384 B) for 2 consecutive 1-second
   *   windows while the USB host is actively receiving (usb_tx_frames ≥ 900/sec).
   *   The 2-second streak avoids false triggers during normal startup or a
   *   momentary main-loop stall.
   *
   * Correction: under BASEPRI = 0x20, zero-fill the first 8 packets of the
   *   ring and reset rx_wr / rx_rd / rx_count to the same state as the
   *   SetStreaming prefill.  The 1536-B silent cushion keeps rx_count above
   *   the 192-B underrun threshold until the next WriteRX block arrives.
   *
   * Why reset to offset 0: the ring is nearly empty so there is nothing
   * worth preserving; starting at 0 avoids any partial-overlap with stale
   * data past the old rx_wr. */
  {
    static uint32_t s_rx_underdrift_check_ms     = 0U;
    static uint32_t s_rx_underdrift_tx_frames    = 0U;
    static uint8_t  s_rx_underdrift_low_streak   = 0U;

    if (g_usb_audio.usb_streaming && now - s_rx_underdrift_check_ms >= 1000U) {
      s_rx_underdrift_check_ms = now;
      uint32_t tx_frames_this_sec = g_usb_audio.usb_tx_frames - s_rx_underdrift_tx_frames;
      s_rx_underdrift_tx_frames   = g_usb_audio.usb_tx_frames;

      if (tx_frames_this_sec >= 900U &&
          g_usb_audio.rx_count < (2U * USB_AUDIO_BYTES_PER_FRAME)) {
        if (++s_rx_underdrift_low_streak >= 2U) {
          __set_BASEPRI(0x20U);
          if (g_usb_audio.rx_count < (2U * USB_AUDIO_BYTES_PER_FRAME)) {
            const uint16_t prefill = (uint16_t)(8U * USB_AUDIO_BYTES_PER_FRAME);
            memset(g_usb_audio.rx_ring, 0, prefill);
            g_usb_audio.rx_wr    = prefill;
            g_usb_audio.rx_rd    = 0U;
            g_usb_audio.rx_count = prefill;
            dbg_rx_underdrift_corrections++;
          }
          __set_BASEPRI(0U);
          s_rx_underdrift_low_streak = 0U;
        }
      } else {
        s_rx_underdrift_low_streak = 0U;
      }
    }
  }

  RuntimeDiag_ServiceSlow(now);
  Diag_Process();
  RuntimeDiag_WatchdogRefreshIfHealthy(now);

  if (now - t_cat    >= 10U) {
    t_cat = now;
    CAT_Process(&g_cat);
    /* Apply hardware changes deferred by CAT handlers — all blocking I2C/SPI/GPIO
     * happens here in main-loop context, never inside the CAT parser. */
    if (g_sdr.cat_freq_dirty) {
      g_sdr.cat_freq_dirty = false;
      DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
      if (g_sdr.si5351_ok)
        SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
      uint8_t _b = BPF_FreqToBand(g_sdr.freq_hz);
      if (_b != 0xFFU && _b != g_sdr.band_idx) {
        BPF_SetBand(_b); LPF_SetBand(_b); g_sdr.band_idx = _b;
        g_sdr.display_dirty |= DIRTY_SBL;
      }
    }
    if (g_sdr.cat_vol_dirty) {
      g_sdr.cat_vol_dirty = false;
      uint8_t _wv = (g_sdr.volume == 0U) ? 0x2FU
                  : (uint8_t)(90U + ((uint16_t)g_sdr.volume * 31U / 100U));
      WM8731_SetVolume(&hi2c1, WM8731_I2C_ADDR, _wv, _wv);
    }
    if (g_sdr.cat_mode_dirty) {
      g_sdr.cat_mode_dirty = false;
      DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
      DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
    }
    if (g_sdr.cat_tx_dirty) {
      g_sdr.cat_tx_dirty = false;
      csdr_apply_tx();
    }
    if (g_sdr.cat_att_dirty) {
      g_sdr.cat_att_dirty = false;
      PE4302_SetAttn_dB(&g_att, g_sdr.att_db);
      g_sdr.att_db = g_att.current_atten_db;
    }
    if (g_sdr.cat_rit_dirty) {
      g_sdr.cat_rit_dirty = false;
      /* nco_if = IF shift + RIT offset (only when RIT is on).
       * RIT shifts the DSP passband without changing the displayed VFO frequency
       * or the SI5351 LO — the received signal moves in the audio chain only. */
      int32_t eff_if = (int32_t)g_sdr.if_shift_hz
                     + (g_cat.rit_on ? (int32_t)g_sdr.rit_hz : 0);
      DSP_SetIFShift(&g_dsp, eff_if, CSDR_AUDIO_SAMPLE_RATE);
    }
    USB_Audio_Process(&g_usb_audio);
    /* Discard buffered PC TX audio when not transmitting.
     * Without this the ring fills to 9216 bytes and stays there,
     * causing every subsequent USB OUT packet to hit the overrun path.
     * BASEPRI = 0x20: masks USB OTG IRQ (priority 2 → 0x20) that writes
     * tx_wr via USB_Audio_WriteTX; SAI/DMA (priority 0) unmasked. */
    if (!g_sdr.tx_mode && g_usb_audio.tx_count > 0U) {
      __set_BASEPRI(0x20U);
      g_usb_audio.tx_rd    = g_usb_audio.tx_wr;
      g_usb_audio.tx_count = 0U;
      __set_BASEPRI(0U);
    }
  }

  /* CAT TX flush — single call site, runs every main-loop tick (~1 ms).
   * Positioned AFTER CAT_Process so new responses are drained on the same
   * tick they are enqueued.  On non-Process ticks this retries any remaining
   * FIFO bytes that were deferred by a previous busy-guard hit. */
  CAT_FlushTX(&g_cat);
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

/* Called from CDC_Close (USB disconnect) to flush stale parser state so the
 * reconnecting host does not see partial commands or queued responses from
 * the previous session. */
void CSDR_CDC_ResetCAT(void)
{
  /* Snapshot callbacks before CAT_Init wipes them via memset.
   * CAT_Init(&g_cat, &g_cat.cb) would zero g_cat.cb first and then copy
   * the already-zeroed struct back — losing all function pointers.
   * BASEPRI = 0x20: masks USB OTG IRQ (CAT_Receive appends to rx_len);
   * SAI/DMA (priority 0) unmasked — audio continues during this copy. */
  CAT_Callbacks_t saved_cb;
  __set_BASEPRI(0x20U);
  saved_cb     = g_cat.cb;
  g_cat.rx_len = 0U;
  __set_BASEPRI(0U);
  CAT_Init(&g_cat, &saved_cb);
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
  (void)h;
}

/* ══════════════════════════════════════════════════════════
 *  Private functions
 * ══════════════════════════════════════════════════════════ */
static void csdr_apply_band(uint8_t band)
{
  BPF_SetBand(band); LPF_SetBand(band);
  uint32_t f = BPF_BandToFreq(band);
  g_sdr.freq_hz = f; g_sdr.band_idx = band;
  DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, f + g_sdr.lo_offset_hz);
  g_sdr.display_dirty |= (DIRTY_HDR | DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
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
    DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
    if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
    uint8_t b = BPF_FreqToBand(g_sdr.freq_hz);
    if (b != 0xFFU && b != g_sdr.band_idx) { BPF_SetBand(b); g_sdr.band_idx = b; }
    g_sdr.display_dirty |= DIRTY_VFO;
  }
  if (Encoder_GetButton(&g_encoder)) {
    if (Menu_IsOpen(&g_menu)) {
      /* Check if selected item is ACTION type (e.g. Diagnostics) */
      if (g_menu.cursor < g_menu.item_count &&
          g_menu.items[g_menu.cursor].type == MENU_TYPE_ACTION) {
        const char *name = g_menu.items[g_menu.cursor].label;
        Menu_Toggle(&g_menu);
        g_sdr.display_dirty |= DIRTY_ALL;
        if (strcmp(name, "Diagnostics") == 0) {
          Diag_Run();
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
          if (Cal_Run(&cp)) {
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
            DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
            if (g_sdr.si5351_ok) {
              g_si5351.xtal_hz = (uint32_t)((int32_t)SI5351_XTAL_HZ +
                SI5351_XTAL_HZ / 1000000L * g_sdr.xtal_ppm);
              SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
            }
          }
        } else if (strcmp(name, "SWR Scan") == 0) {
          SWR_Scan_Run();
        }
        g_sdr.display_dirty |= DIRTY_ALL;
      } else {
        Menu_Select(&g_menu);
      }
      return;
    }
    g_sdr.mode   = (SDR_Mode_t)((g_sdr.mode + 1U) % MODE_COUNT);
    g_sdr.bw_hz  = default_bw_for_mode(g_sdr.mode);
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
    DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
    g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL);
  }
  if (Encoder_GetLongPress(&g_encoder)) {
    /* Long press: cycle spectrum zoom ±24k → ±18k → ±12k → ±6k → ±3k → ±24k */
    uint8_t z = (uint8_t)((SDR_UI_GetSpecZoom() + 1U) % SPEC_ZOOM_COUNT);
    SDR_UI_SetSpecZoom(z);
  }
}

static void csdr_handle_keys(void)
{
  Key_Poll(&k_menu); Key_Poll(&k_f1);   Key_Poll(&k_f2); Key_Poll(&k_f3);
  Key_Poll(&k_f4);   Key_Poll(&k_band); Key_Poll(&k_mode); Key_Poll(&k_ptt);

  if (Key_Press(&k_menu)) {
    if (Diag_IsActive()) { Diag_Run(); return; }
    g_sdr.display_dirty = 0U;  /* prevent status panel overwriting menu */
    if (!Menu_IsOpen(&g_menu))
      Menu_LoadFromSDR(&g_menu,
        g_sdr.agc_fast, g_sdr.nb_on, g_sdr.nr_on, g_sdr.rit_hz,
        g_sdr.volume, g_sdr.squelch, (uint32_t)g_sdr.step,
        g_sdr.att_db, g_sdr.band_idx, (uint8_t)g_sdr.mode,
        g_sdr.usb_mode, SDR_UI_GetSpecZoom(), menu_apply_cb);
    Menu_Toggle(&g_menu);
    if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty |= DIRTY_ALL;
  }

  /* F1: reset diag peaks (DIAG active) / menu UP / Volume Down */
  if (Key_PressOrRepeat(&k_f1)) {
    if (Diag_IsActive()) {
      Diag_ResetPeaks();
    } else if (Menu_IsOpen(&g_menu)) {
      Menu_Up(&g_menu);
    } else {
      uint8_t v = (g_sdr.volume >= 2U) ? (g_sdr.volume - 2U) : 0U;
      csdr_apply_volume(v);
    }
  }

  /* F2: menu DOWN / Volume Up (hold-repeat while held) */
  if (Key_PressOrRepeat(&k_f2)) {
    if (Menu_IsOpen(&g_menu)) Menu_Down(&g_menu);
    else {
      uint8_t v = (g_sdr.volume <= 98U) ? (g_sdr.volume + 2U) : 100U;
      csdr_apply_volume(v);
    }
  }

  /* F3: menu confirm / VFO A↔B swap */
  if (Key_Press(&k_f3)) {
    if (Menu_IsOpen(&g_menu)) {
      if (g_menu.cursor < g_menu.item_count &&
          g_menu.items[g_menu.cursor].type == MENU_TYPE_ACTION) {
        const char *name = g_menu.items[g_menu.cursor].label;
        Menu_Toggle(&g_menu);
        g_sdr.display_dirty |= DIRTY_ALL;
        if (strcmp(name, "Diagnostics") == 0) {
          Diag_Run();
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
          if (Cal_Run(&cp)) {
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
            DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
            if (g_sdr.si5351_ok) {
              g_si5351.xtal_hz = (uint32_t)((int32_t)SI5351_XTAL_HZ +
                SI5351_XTAL_HZ / 1000000L * g_sdr.xtal_ppm);
              SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
            }
          }
        } else if (strcmp(name, "SWR Scan") == 0) {
          SWR_Scan_Run();
        }
        g_sdr.display_dirty |= DIRTY_ALL;
      } else {
        Menu_Confirm(&g_menu);
      }
    } else {
      csdr_vfo_swap();   /* F3 outside menu: swap VFO A↔B */
    }
  }

  /* F4: Exit DIAG  –or–  Back / Exit menu  –or–  copy active VFO to inactive */
  if (Key_Press(&k_f4)) {
    if (Diag_IsActive()) { Diag_Run(); return; }
    if (Menu_IsOpen(&g_menu)) {
      Menu_Back(&g_menu);
      if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty |= DIRTY_ALL;
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
    g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL);
  }

  if (Key_Press(&k_ptt)) {
    bool _tx = !g_sdr.tx_mode;
    g_sdr.tx_mode = _tx;
    g_sdr.display_dirty |= _tx ? DIRTY_HDR : (uint8_t)DIRTY_ALL;
    csdr_apply_tx();   /* immediate for physical PTT — no deferred path */
  }
}

static void csdr_update_spectrum(void)
{
  if (g_sdr.tx_mode || !g_dsp.fft_ready || Menu_IsOpen(&g_menu)) return;
  g_dsp.fft_ready = false;

  float bw_lo_ratio = 0.0f, bw_hi_ratio = 0.0f;
  if (g_dsp.sample_rate > 0U && g_sdr.bw_hz > 0U) {
    float full = (float)g_sdr.bw_hz / (float)g_dsp.sample_rate;
    float half = full * 0.5f;
    switch (g_sdr.mode) {
      case MODE_LSB: bw_lo_ratio = full; bw_hi_ratio = 0.0f; break;
      case MODE_USB:
      case MODE_CW:  bw_lo_ratio = 0.0f; bw_hi_ratio = full; break;
      default:       bw_lo_ratio = half; bw_hi_ratio = half; break;
    }
  }
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_SPECTRUM);
  SDR_UI_DrawSpectrum(g_dsp.fft_mag_db, DSP_FFT_SIZE, bw_lo_ratio, bw_hi_ratio, NULL);
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_SPECTRUM);
}

static void csdr_update_waterfall(void)
{
  if (g_sdr.tx_mode || Menu_IsOpen(&g_menu)) return;
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_WATERFALL);
  g_dsp.wf_lines = 0U;                   /* consume pending lines; render latest FFT frame */
  SDR_UI_DrawWaterfall(g_dsp.fft_mag_db, DSP_FFT_SIZE);
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_WATERFALL);
}

static void csdr_refresh_display(void)
{
  bool menu_open = Menu_IsOpen(&g_menu);

  uint8_t dirty = g_sdr.display_dirty;
  if (dirty != 0U) {
    g_sdr.display_dirty = 0U;
    SDR_UI_State_t ui = {0};
    ui.freq_hz   = g_sdr.freq_hz;       ui.mode      = (uint8_t)g_sdr.mode;
    ui.band_idx  = g_sdr.band_idx;      ui.volume    = g_sdr.volume;
    ui.squelch   = g_sdr.squelch;       ui.step      = (uint32_t)g_sdr.step;
    ui.agc_fast  = g_sdr.agc_fast;      ui.nb_on     = g_sdr.nb_on;
    ui.nr_on     = g_sdr.nr_on;         ui.rit_hz    = g_sdr.rit_hz;
    ui.tx_mode   = g_sdr.tx_mode;       ui.si5351_ok = g_sdr.si5351_ok;
    ui.signal_db = g_dsp.signal_power_db;
    ui.bw_hz     = g_sdr.bw_hz;         ui.voltage_x10 = (int16_t)(g_analog.voltage_mv / 100);
    ui.att_db    = g_sdr.att_db;        ui.mic_gain  = g_sdr.mic_gain;
    ui.freq_b_hz = g_sdr.vfo_b.freq_hz; /* inactive VFO shown in sub-line */
    ui.active_vfo = g_sdr.active_vfo;

    if (menu_open) {
      /* Menu đang mở: re-render để đảm bảo không bị xóa */
      Menu_Render(&g_menu);
    } else if (g_sdr.tx_mode) {
      /* TX: only redraw Header – meter is handled by UpdateTXMeters below */
      if (dirty & DIRTY_HDR) {
        RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
        SDR_UI_DrawHeader(&ui);
        RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
      }
    } else {
      /* RX: redraw only zones flagged dirty */
      if (dirty & DIRTY_HDR) {
        RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_STATUS_BAR);
        SDR_UI_DrawHeader(&ui);
        RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_STATUS_BAR);
      }
      if (dirty & DIRTY_VFO) {
        RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_STATUS_BAR);
        SDR_UI_DrawVFO(&ui);
        RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_STATUS_BAR);
      }
      if (dirty & DIRTY_MTR) {
        RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_STATUS_BAR);
        SDR_UI_DrawMeter(&ui);
        RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_STATUS_BAR);
      }
      if (dirty & DIRTY_SBL) {
        RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_STATUS_BAR);
        SDR_UI_DrawSidebarLeft(&ui);
        RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_STATUS_BAR);
      }
      if (dirty & DIRTY_SBR) {
        RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_STATUS_BAR);
        SDR_UI_DrawSidebarRight(&ui);
        RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_STATUS_BAR);
      }
    }
  } else if (!menu_open) {
    if (g_sdr.tx_mode) {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
      SDR_UI_UpdateTXMeters((int32_t)g_analog.alc_percent,
                            (int32_t)(g_analog.swr_x100 / 10));
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
    } else {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
      SDR_UI_UpdateSMeter(g_dsp.signal_power_db);
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
    }
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
  DSP_NB_Set(&g_dsp, nb, g_sdr.nb_level);
  g_sdr.rit_hz = rit;
  g_sdr.cat_rit_dirty = true;  /* apply new RIT offset to nco_if via CSDR_Loop */
  csdr_apply_volume(vol); g_sdr.squelch = sq;
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
  if (zoom != SDR_UI_GetSpecZoom()) SDR_UI_SetSpecZoom(zoom);
  g_sdr.display_dirty |= DIRTY_ALL;
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
  DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
  g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
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
  g_sdr.display_dirty  |= DIRTY_VFO;
}

/* ── VFO-B + active-VFO CAT callbacks ──────────────────────────────────────
 * These keep g_sdr.vfo_b and g_sdr.active_vfo in sync with CAT commands
 * so UI state and Hamlib/flrig/WSJT-X always agree.
 * ─────────────────────────────────────────────────────────────────────────*/
static void     cat_set_vfo_b_freq(uint32_t hz) { g_sdr.vfo_b.freq_hz = hz; g_sdr.display_dirty |= DIRTY_VFO; }
static void     cat_set_vfo_b_mode(uint8_t m)   { g_sdr.vfo_b.mode = (SDR_Mode_t)m; g_sdr.display_dirty |= DIRTY_VFO; }
static void     cat_set_vfo_b_bw(uint32_t hz)   { g_sdr.vfo_b.bw_hz = hz; g_sdr.display_dirty |= DIRTY_VFO; }
static uint32_t cat_get_vfo_b_freq(void)        { return g_sdr.vfo_b.freq_hz; }
static uint8_t  cat_get_vfo_b_mode(void)        { return (uint8_t)g_sdr.vfo_b.mode; }
static uint32_t cat_get_vfo_b_bw(void)          { return g_sdr.vfo_b.bw_hz; }

/* VS/FR/DC handler: real VFO swap with deferred hardware update.
 * Same logic as csdr_vfo_swap() but non-blocking (dirty flags, not direct I2C).
 * In split mode the LO must stay on the RX VFO, so only update the tracking
 * label — the user tunes VFO B via FB SET without retuning the receiver. */
static void cat_set_active_vfo(uint8_t vfo)
{
  if (vfo == g_sdr.active_vfo) return;
  if (g_cat.split_on) {
    /* Split: don't retune — only update display label */
    g_sdr.active_vfo  = vfo;
    g_cat.active_vfo  = vfo;
    g_sdr.display_dirty |= DIRTY_VFO;
    return;
  }
  /* Full swap: exchange active ↔ inactive VFO storage, then defer hardware */
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
  g_cat.active_vfo   = g_sdr.active_vfo;

  g_sdr.cat_freq_dirty = true;   /* retune SI5351 to new freq_hz */
  g_sdr.cat_mode_dirty = true;   /* reapply DSP mode + BW */
  g_sdr.display_dirty  |= (DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
}
static uint8_t cat_get_active_vfo(void) { return g_sdr.active_vfo; }

/* CAT callbacks */
static void     cat_set_freq(uint32_t f)
{
  if (g_sdr.cat_freq_dirty) dbg_cat_blocked_updates++;
  g_sdr.freq_hz = f;
  g_sdr.cat_freq_dirty = true; /* SI5351 + DSP NCO applied by CSDR_Loop */
  g_sdr.display_dirty |= DIRTY_VFO;
}
static void     cat_set_mode(uint8_t m)
{
  if (g_sdr.cat_mode_dirty) dbg_cat_blocked_updates++;
  g_sdr.mode  = (SDR_Mode_t)m;
  g_sdr.bw_hz = default_bw_for_mode(g_sdr.mode);
  g_sdr.cat_mode_dirty = true; /* DSP mode + BW applied by CSDR_Loop */
  g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL);
}
/* CAT callback — deferred: sets dirty flag, hardware applied by CSDR_Loop.
 * Physical PTT key goes through csdr_apply_tx() for immediate switching. */
static void cat_set_tx(bool tx)
{
  if (tx == g_sdr.tx_mode) return;
  if (g_sdr.cat_tx_dirty) dbg_cat_blocked_updates++;
  g_sdr.tx_mode    = tx;
  g_sdr.cat_tx_dirty = true;
  g_sdr.display_dirty |= tx ? DIRTY_HDR : (uint8_t)DIRTY_ALL;
}
static void     cat_set_att(uint8_t lv)
{
  if (g_sdr.cat_att_dirty) dbg_cat_blocked_updates++;
  static const uint8_t m[] = {0, 6, 12, 18};
  g_sdr.att_db = (lv < 4U) ? m[lv] : 0U;
  g_sdr.cat_att_dirty = true; /* PE4302 applied by CSDR_Loop */
  g_sdr.display_dirty |= DIRTY_HDR;
}

/* Immediate volume apply — WM8731 I2C updated synchronously.
 * Called from key handler and menu; safe in main-loop context. */
static void csdr_apply_volume(uint8_t vol)
{
  if (vol > 100U) vol = 100U;
  g_sdr.volume = vol;
  uint8_t wm_vol = (vol == 0U) ? 0x2FU
                                : (uint8_t)(90U + ((uint16_t)vol * 31U / 100U));
  WM8731_SetVolume(&hi2c1, WM8731_I2C_ADDR, wm_vol, wm_vol);
  g_sdr.display_dirty |= DIRTY_SBL;
}

/* Immediate TX/RX apply — hardware switched synchronously.
 * Called from physical PTT key handler and from the cat_tx_dirty path in CSDR_Loop.
 * Caller must have already set g_sdr.tx_mode to the desired state.
 *
 * Split operation: inactive VFO (vfo_b.freq_hz) is always the TX VFO.
 * On TX: SI5351 switches to vfo_b.freq_hz so the QSD mixer is on the TX frequency.
 * On RX: SI5351 returns to freq_hz (active/RX VFO).
 * The SI5351 call here is synchronous and safe — this function runs in main-loop
 * context (cat_tx_dirty path) or from the physical PTT key, never from an ISR. */
static void csdr_apply_tx(void)
{
  if (g_sdr.tx_mode) {
    /* RX → TX: switch BPF relay bank first (includes 2 ms relay gap), then close T/R. */
    BPF_SetMode(RF_MODE_TX);
    /* Split: retune LO to TX VFO (inactive slot) before gating RF */
    if (g_cat.split_on && g_sdr.si5351_ok)
      SI5351_SetQSDFrequency(&g_si5351, g_sdr.vfo_b.freq_hz + g_sdr.lo_offset_hz);
    HAL_GPIO_WritePin(T_R_SW_GPIO_Port, T_R_SW_Pin, GPIO_PIN_SET);
  } else {
    /* TX → RX: open T/R relay first, then switch BPF bank back, then unmute codec. */
    HAL_GPIO_WritePin(T_R_SW_GPIO_Port, T_R_SW_Pin, GPIO_PIN_RESET);
    BPF_SetMode(RF_MODE_RX);
    /* Split: restore LO to RX VFO (active slot) */
    if (g_cat.split_on && g_sdr.si5351_ok)
      SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
    WM8731_SetMute(&hi2c1, WM8731_I2C_ADDR, false);
  }
  SDR_UI_UpdateSMeter_SetTX(g_sdr.tx_mode);
}

/* CAT callback — deferred: sets dirty flag, WM8731 applied by CSDR_Loop. */
static void cat_set_volume(uint8_t vol)
{
  if (vol > 100U) vol = 100U;
  g_sdr.volume = vol;
  g_sdr.cat_vol_dirty = true;
  g_sdr.display_dirty |= DIRTY_SBL;
}
static void     cat_set_nr(bool on)       { g_sdr.nr_on = on; g_sdr.display_dirty |= DIRTY_SBL; }
static void     cat_set_nb(bool on)       { g_sdr.nb_on = on; DSP_NB_Set(&g_dsp, on, g_sdr.nb_level); g_sdr.display_dirty |= DIRTY_SBL; }
static void cat_set_bw(uint32_t hz)
{
    if (hz < 100U)   hz = 100U;
    if (hz > 24000U) hz = 24000U;
    g_sdr.bw_hz = hz;
    DSP_SetBW(&g_dsp, (float)hz);
    g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBR);
}
static void     cat_set_agc_fast(bool f)  { g_sdr.agc_fast = f; g_sdr.display_dirty |= DIRTY_SBL; }
static void     cat_set_squelch(uint8_t s){ g_sdr.squelch = s; g_sdr.display_dirty |= DIRTY_SBL; }

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
  if (g_sdr.cat_rit_dirty) dbg_cat_blocked_updates++;
  if (hz > 9999)  hz = 9999;
  if (hz < -9999) hz = -9999;
  g_sdr.rit_hz = (int16_t)hz;
  g_sdr.cat_rit_dirty = true;  /* recompute nco_if = if_shift_hz + (rit_on ? rit_hz : 0) */
  g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBR);
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
  g_sdr.display_dirty |= DIRTY_VFO;
}

static void cat_set_if_shift(int32_t hz)
{
  if (hz >  9999) hz =  9999;
  if (hz < -9999) hz = -9999;
  g_sdr.if_shift_hz = (int16_t)hz;
  g_sdr.cat_rit_dirty = true;  /* nco_if = if_shift_hz + (rit_on ? rit_hz : 0) */
  g_sdr.display_dirty |= DIRTY_VFO;
}

static int32_t cat_get_if_shift(void) { return (int32_t)g_sdr.if_shift_hz; }
int32_t *CSDR_GetTxBuf(void) { return s_tx_buf; }
int32_t *CSDR_GetRxBuf(void) { return s_rx_buf; }
void CSDR_ClearDspFlags(void) { s_rx_ready_seq[0] = s_rx_done_seq[0]; s_rx_ready_seq[1] = s_rx_done_seq[1]; }
