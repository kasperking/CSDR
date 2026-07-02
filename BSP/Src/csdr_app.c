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
#include "nr_spec.h"
#include "encoder.h"
#include "input_scan.h"
#include "si5351.h"
#include "pe4302.h"
#include "w25q.h"
#include "bpf_lpf.h"
#include "fsdr_analog.h"
#include "usb_cat.h"
#include "usb_audio.h"
#include "menu.h"
#include "band_sel.h"
#include "mode_sel.h"
#include "cal.h"
#include "sdr_scan.h"
#include "runtime_diag.h"
#include "rf_agc.h"
#include "pa_overcurrent.h"
#include "pa_protect.h"
#include "usb_flash_proto.h"
#include "selftest.h"
#include "hw_fault.h"
#include "spi_assets.h"
#include "cw_decode.h"
#include "cw_keyer.h"
#include "rtc_clock.h"
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
static CWDec_t          g_cw_dec;
static CWKeyer_t        g_cw_keyer;

BandCal_t g_band_cal[BAND_COUNT] = {
  {0,0,0,100},{0,0,0,100},{0,0,0,100},{0,0,0,100},{0,0,0,100},
  {0,0,0,100},{0,0,0,100},{0,0,0,100},{0,0,0,100},{0,0,0,100},{0,0,0,100}
};

SDR_State_t g_sdr = {
  .freq_hz       = CSDR_FREQ_DEFAULT_HZ,
  .mode          = MODE_USB,
  .band_idx      = 3U,
  .volume        = 20U,
  .bw_hz         = 3000U,
  .if_shift_hz   = 0,
  .squelch       = 0U,
  .step          = STEP_100,
  .agc_speed     = 1U,     /* default FAST */
  .nb_on         = false,   /* NB disabled by default */
  .nb_level      = 50U,     /* moderate intensity; raise for more aggressive blanking */
  .nr_level      = 50U,     /* half dry/wet mix; 100 = pure LMS prediction */
  .display_dirty = DIRTY_ALL,
  .lo_offset_hz  = LO_OFFSET_DEFAULT,
  .mic_gain      = 50,
  .digi_gain     = 70,
  .tx_power      = 100,
  .pa_watts      = 0,
  .pa_oc_limit_idx = 100U,     /* 10.0A default (stored ×10) */
  .tx_audio_low_hz  = 200U,
  .tx_audio_high_hz = 2800U,
  .notch_on  = false,
  .notch_hz  = 1000,
  .vox_on    = false,
  .vox_gain  = 50U,
  .vox_delay = 500U,
  .cw_decode_on   = false,
  .cw_pitch_hz    = 700U,
  .cw_wpm         = 20U,
  .keyer_mode     = 0U,   /* Straight */
  .paddle_reverse = false,
  .sidetone_vol   = 50U,
  .cw_bkin        = 0U,   /* BK-IN Off */
  .cw_bk_delay_ms = 200U,
  .cw_reverse     = false,
  .cw_filter_hz   = 500U,
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
static volatile int32_t  dbg_tx_peak         = 0;       /* Peak |sample| in last block (proportional to vol) */
static volatile uint32_t dbg_vol_scale_q16   = 0;       /* g_dsp.rx_volume_scale × 65536 (integer view) */
static volatile uint8_t  dbg_force_tone      = 0;       /* set=1 in debugger → 1kHz test tone direct to DAC */
static volatile uint32_t dbg_wm8731_ok         = 0;   /* WM8731 init result */
static volatile uint32_t dbg_wm8731_vol_err    = 0;   /* WM8731_SetVolume fail count */
static volatile uint32_t dbg_wm8731_i2c_err    = 0;   /* hi2c1.ErrorCode at last SetVolume fail */
static volatile uint32_t dbg_wm8731_i2c_state  = 0;   /* hi2c1.State at last SetVolume fail */
static volatile uint32_t dbg_wm8731_postinit_ok = 0;  /* post-SAI re-write: 0=all OK, else bitmask of failed steps */
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

/* Debounced freq save: updated on any freq change; csdr_save_settings fires 3s later.
 * s_save_on_disconnect: set by CSDR_CDC_ResetCAT so CSDR_Loop flushes immediately. */
static uint32_t s_freq_save_tick      = 0U;
static volatile bool s_save_on_disconnect = false;
static uint32_t s_tune_start_ms       = 0U;   /* HAL_GetTick() at TUNE start; TUNE_MAX_MS safety */
/* Suppress SAI stop/start volume apply for 500ms after USB connect so flrig
 * init sequence completes before the ~15ms audio gap. 0 = no active window. */
static volatile uint32_t s_usb_connect_tick = 0U;

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
static void csdr_factory_reset(void);
static void csdr_tune_start(void);
static void csdr_tune_stop(void);


#define CSDR_UI_SPEC_RX_PERIOD_MS    75U    /* ~13 fps spectrum in RX — decoupled from waterfall */
#define CSDR_UI_WF_RX_PERIOD_MS      75U    /* ~13 fps waterfall in RX */
#define UI_OVERLOAD_DECAY_MS         150U   /* suppress waterfall; 2 ticks to enter, 2 to recover */
#define CSDR_UI_DISPLAY_RX_PERIOD_MS 100U   /* 10 fps dirty-zone + meter refresh */
#define CSDR_UI_DISPLAY_TX_PERIOD_MS 1000U  /* 1 Hz compact TX meter refresh */
#define CSDR_UI_TX_DIRTY_MIN_MS      1000U  /* defer knob/menu redraws while TX audio is time-critical */
#define CSDR_UI_TX_SPEC_PERIOD_MS     200U  /* TX mic spectrum ~5 fps */

#define TUNE_POWER_PCT  25U     /* fixed drive cap while TUNE is active, independent of tx_power */
#define TUNE_MAX_MS     30000U  /* safety auto-stop in case the TUNE button sticks */


/* ── Function key state machines ── */
static Key_t k_menu, k_f1, k_f2, k_f3, k_f4, k_band, k_mode, k_ptt, k_tune;

/* ── CAT callbacks ── */
static void     cat_set_freq(uint32_t f);
static void     cat_set_mode(uint8_t m);
static void     cat_set_tx(bool tx);
static void     cat_set_att(uint8_t lv);
static void     cat_set_volume(uint8_t vol);
static void     cat_set_nr(uint8_t mode);
static void     cat_set_nr_level(uint8_t level);
static void     cat_set_bc(uint8_t mode);
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
static uint8_t  cat_get_nr(void);
static uint8_t  cat_get_nr_level(void);
static uint8_t  cat_get_bc(void);
static bool     cat_get_nb(void);
static uint32_t cat_get_bw(void);
static bool     cat_get_agc_fast(void);
static uint8_t  cat_get_squelch(void);
static int32_t  cat_get_rit_hz(void);
static uint32_t cat_get_step(void);

static void csdr_apply_mode_idx(uint8_t m);
static void csdr_apply_band(uint8_t band);
static void csdr_handle_encoder(void);
static void csdr_handle_keys(void);
static void csdr_process_audio_pending(void);
static void csdr_update_tx_spectrum(void);
static void csdr_update_spectrum(void);
static void csdr_update_waterfall(void);
static void csdr_refresh_display(void);
static void menu_apply_cb(void);
static void csdr_vox_poll(void);
static uint32_t default_bw_for_mode(SDR_Mode_t m);
static void csdr_apply_nco_if(void);
static void csdr_vfo_swap(void);
static void csdr_on_mode_changed(SDR_Mode_t old_mode);
/* CAT VFO-B callbacks */
static void     cat_set_lo_cut(uint32_t hz);
static uint32_t cat_get_lo_cut(void);
static void     cat_set_vfo_b_freq(uint32_t hz);
static void     cat_set_vfo_b_mode(uint8_t m);
static void     cat_set_vfo_b_bw(uint32_t hz);
static void     cat_set_vfo_b_lo_cut(uint32_t hz);
static uint32_t cat_get_vfo_b_freq(void);
static uint8_t  cat_get_vfo_b_mode(void);
static uint32_t cat_get_vfo_b_bw(void);
static uint32_t cat_get_vfo_b_lo_cut(void);
static void     cat_set_active_vfo(uint8_t vfo);
static uint8_t  cat_get_active_vfo(void);
static void     cat_set_rf_agc(bool on);
static bool     cat_get_rf_agc(void);
static void     cat_set_tx_power(uint8_t pct);
static uint8_t  cat_get_tx_power(void);
static uint16_t cat_get_swr_x100(void);
static uint8_t  cat_get_alc_reduction_pct(void);
static void     cat_set_cw_wpm(uint8_t wpm);
static uint8_t  cat_get_cw_wpm(void);
static void     cat_set_usb_stream(uint8_t m);
static uint8_t  cat_get_usb_stream(void);
static void     cat_set_tune(bool on);
static bool     cat_get_tune(void);

/* ── Settings persistence helper ──────────────────────────
 * Serialises g_sdr to Flash_Settings_t and calls Flash_SaveSettings.
 * Call from menu_apply_cb and rf_agc_on toggle. */
static void csdr_save_settings(void)
{
  Flash_Settings_t fs;
  memset(&fs, 0, sizeof(fs));

  /* VFO A */
  fs.freq_hz         = g_sdr.freq_hz;
  fs.step            = (uint32_t)g_sdr.step;
  fs.bw_hz           = g_sdr.bw_hz;
  fs.sl_hz           = g_sdr.sl_hz;
  fs.mode            = (uint8_t)g_sdr.mode;
  fs.band_idx        = g_sdr.band_idx;
  fs.volume          = g_sdr.volume;
  fs.squelch         = g_sdr.squelch;
  fs.att_db          = g_sdr.att_db;
  fs.if_shift_hz     = g_sdr.if_shift_hz;

  /* Flags */
  fs.agc_speed       = g_sdr.agc_speed;
  fs.nb_on           = g_sdr.nb_on;
  fs.nr_mode         = g_sdr.nr_mode;
  fs.nb_level        = g_sdr.nb_level;
  fs.nr_level        = g_sdr.nr_level;
  fs.bc_mode         = g_sdr.bc_mode;
  fs.rf_agc_on       = g_sdr.rf_agc_on;
  fs.usb_mode        = g_sdr.usb_mode;
  fs.usb_iq_stream   = g_sdr.usb_iq_stream;
  fs.ext_alc_on      = g_sdr.ext_alc_on;

  /* TX / audio */
  fs.mic_gain        = g_sdr.mic_gain;
  fs.digi_gain       = g_sdr.digi_gain;
  fs.tx_power           = g_sdr.tx_power;
  fs.pa_watts           = g_sdr.pa_watts;
  fs.tx_audio_low_hz    = g_sdr.tx_audio_low_hz;
  fs.tx_audio_high_hz   = g_sdr.tx_audio_high_hz;
  fs.notch_on           = g_sdr.notch_on ? 1U : 0U;
  fs.notch_hz           = g_sdr.notch_hz;
  fs.vox_on             = g_sdr.vox_on ? 1U : 0U;
  fs.vox_gain           = g_sdr.vox_gain;
  fs.vox_delay_ms       = g_sdr.vox_delay;
  fs.pa_oc_limit_idx = g_sdr.pa_oc_limit_idx;
  /* CW settings */
  fs.cw_pitch_hz    = g_sdr.cw_pitch_hz;
  fs.cw_wpm         = g_sdr.cw_wpm;
  fs.keyer_mode     = g_sdr.keyer_mode;
  fs.paddle_reverse = g_sdr.paddle_reverse ? 1U : 0U;
  fs.sidetone_vol   = g_sdr.sidetone_vol;
  fs.cw_bkin        = g_sdr.cw_bkin;
  fs.cw_bk_delay_ms = g_sdr.cw_bk_delay_ms;
  fs.cw_reverse     = g_sdr.cw_reverse ? 1U : 0U;
  fs.cw_filter_hz   = g_sdr.cw_filter_hz;
  fs.cw_decode_on   = g_sdr.cw_decode_on ? 1U : 0U;
  fs.tx_src         = g_sdr.tx_src;
  fs.audio_gain_db   = g_sdr.audio_gain_db;

  /* Calibration */
  fs.xtal_ppm        = g_sdr.xtal_ppm;
  fs.dc_i_offset     = g_sdr.dc_i_offset;
  fs.dc_q_offset     = g_sdr.dc_q_offset;
  fs.lo_offset_hz    = g_sdr.lo_offset_hz;
  fs.smeter_offset_db = g_sdr.smeter_offset_db;
  fs.iq_gain         = g_sdr.iq_gain;
  fs.iq_phase        = g_sdr.iq_phase;

  /* VFO B */
  fs.vfo_b_freq_hz     = g_sdr.vfo_b.freq_hz;
  fs.vfo_b_step        = (uint32_t)g_sdr.vfo_b.step;
  fs.vfo_b_bw_hz       = g_sdr.vfo_b.bw_hz;
  fs.vfo_b_sl_hz       = g_sdr.vfo_b.sl_hz;
  fs.vfo_b_mode        = (uint8_t)g_sdr.vfo_b.mode;
  fs.vfo_b_band_idx    = g_sdr.vfo_b.band_idx;
  fs.vfo_b_if_shift_hz = g_sdr.vfo_b.if_shift_hz;
  fs.active_vfo        = g_sdr.active_vfo;

  Flash_SaveSettings(&g_flash, &fs);
}

/* ══════════════════════════════════════════════════════════
 *  CSDR_Init  – gọi từ main.c USER CODE BEGIN 2
 * ══════════════════════════════════════════════════════════ */

void CSDR_Init(void)
{
  RuntimeDiag_Init();

  /* Power hold */
  PWR_Init();

  /* RTC — must be before flash restore so IsSet() works */
  RTC_Clock_Init();

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

  /* Default: IQ stream (preserves pre-existing behavior; not persisted in Flash) */
  g_sdr.usb_iq_stream = true;

  /* Flash: load settings */
  bool boot_flash_ok = (W25Q_Init(&g_flash, &hspi3, FLASH_CS_GPIO_Port, FLASH_CS_Pin) == HAL_OK);
  if (boot_flash_ok) {
    Flash_Settings_t fs;
    if (Flash_LoadSettings(&g_flash, &fs) == HAL_OK) {
      /* VFO A */
      g_sdr.freq_hz       = fs.freq_hz;
      g_sdr.mode          = (SDR_Mode_t)fs.mode;
      g_sdr.band_idx      = fs.band_idx;
      g_sdr.step          = (FreqStep_t)fs.step;
      g_sdr.bw_hz         = fs.bw_hz;
      g_sdr.sl_hz         = fs.sl_hz;
      g_sdr.if_shift_hz   = fs.if_shift_hz;
      /* Operating controls */
      g_sdr.volume        = fs.volume;
      g_sdr.squelch       = fs.squelch;
      g_sdr.att_db        = fs.att_db;
      g_sdr.agc_speed     = (fs.agc_speed <= 2U) ? fs.agc_speed : 1U;
      g_sdr.nb_on         = fs.nb_on;
      g_sdr.nr_mode       = (fs.nr_mode <= 2U) ? fs.nr_mode : 0U;
      g_sdr.nb_level      = fs.nb_level;
      g_sdr.nr_level      = (fs.nr_level <= 100U) ? fs.nr_level : 50U;
      g_sdr.bc_mode       = (fs.bc_mode <= 2U) ? fs.bc_mode : 0U;
      g_sdr.rf_agc_on     = fs.rf_agc_on;
      g_sdr.usb_mode      = (fs.usb_mode <= 1U) ? fs.usb_mode : 1U;
      g_sdr.usb_iq_stream = fs.usb_iq_stream;
      g_sdr.ext_alc_on    = fs.ext_alc_on;
      /* TX / audio */
      g_sdr.mic_gain      = fs.mic_gain;
      g_sdr.digi_gain     = fs.digi_gain;
      g_sdr.tx_power        = fs.tx_power ? fs.tx_power : 100U; /* default 100 for old EEPROM */
      g_sdr.pa_watts        = fs.pa_watts;
      g_sdr.pa_oc_limit_idx = (fs.pa_oc_limit_idx >= 10U && fs.pa_oc_limit_idx <= 200U)
                               ? fs.pa_oc_limit_idx : 100U;
      g_sdr.tx_audio_low_hz  = (fs.tx_audio_low_hz  >= 100U && fs.tx_audio_low_hz  <= 500U)
                               ? fs.tx_audio_low_hz  : 200U;
      g_sdr.tx_audio_high_hz = (fs.tx_audio_high_hz >= 2200U && fs.tx_audio_high_hz <= 3500U)
                               ? fs.tx_audio_high_hz : 2800U;
      g_sdr.notch_on  = (fs.notch_on != 0U);
      g_sdr.notch_hz  = (fs.notch_hz >= 100 && fs.notch_hz <= 4000) ? fs.notch_hz : 1000;
      g_sdr.vox_on    = (fs.vox_on != 0U);
      g_sdr.vox_gain  = (fs.vox_gain  <= 100U) ? fs.vox_gain  : 50U;
      g_sdr.vox_delay = (fs.vox_delay_ms >= 100U && fs.vox_delay_ms <= 2000U) ? fs.vox_delay_ms : 500U;
      g_sdr.audio_gain_db = fs.audio_gain_db;
      /* CW settings */
      g_sdr.cw_pitch_hz    = (fs.cw_pitch_hz  >= 300U && fs.cw_pitch_hz  <= 900U)  ? fs.cw_pitch_hz    : 700U;
      g_sdr.cw_wpm         = (fs.cw_wpm        >= 5U   && fs.cw_wpm        <= 40U)  ? fs.cw_wpm          : 20U;
      g_sdr.keyer_mode     = (fs.keyer_mode    <= 2U)                               ? fs.keyer_mode      : 0U;
      g_sdr.paddle_reverse = (fs.paddle_reverse != 0U);
      g_sdr.sidetone_vol   = (fs.sidetone_vol  <= 100U)                             ? fs.sidetone_vol    : 50U;
      g_sdr.cw_bkin        = (fs.cw_bkin       <= 2U)                               ? fs.cw_bkin         : 0U;
      g_sdr.cw_bk_delay_ms = (fs.cw_bk_delay_ms >= 50U && fs.cw_bk_delay_ms <= 2000U) ? fs.cw_bk_delay_ms : 200U;
      g_sdr.cw_reverse     = (fs.cw_reverse    != 0U);
      g_sdr.cw_filter_hz   = (fs.cw_filter_hz  >= 50U  && fs.cw_filter_hz  <= 500U) ? fs.cw_filter_hz   : 500U;
      g_sdr.cw_decode_on   = (fs.cw_decode_on  != 0U);
      g_sdr.tx_src         = (fs.tx_src <= 1U) ? fs.tx_src : 0U;
      /* Calibration */
      g_sdr.xtal_ppm         = fs.xtal_ppm;
      g_sdr.dc_i_offset      = fs.dc_i_offset;
      g_sdr.dc_q_offset      = fs.dc_q_offset;
      g_sdr.lo_offset_hz     = fs.lo_offset_hz;
      g_sdr.smeter_offset_db = fs.smeter_offset_db;
      g_sdr.iq_gain          = fs.iq_gain;
      g_sdr.iq_phase         = fs.iq_phase;
      /* VFO B */
      g_sdr.vfo_b.freq_hz     = fs.vfo_b_freq_hz;
      g_sdr.vfo_b.mode        = (SDR_Mode_t)fs.vfo_b_mode;
      g_sdr.vfo_b.band_idx    = fs.vfo_b_band_idx;
      g_sdr.vfo_b.step        = (FreqStep_t)fs.vfo_b_step;
      g_sdr.vfo_b.bw_hz       = fs.vfo_b_bw_hz;
      g_sdr.vfo_b.sl_hz       = fs.vfo_b_sl_hz;
      g_sdr.vfo_b.if_shift_hz = fs.vfo_b_if_shift_hz;
      g_sdr.active_vfo        = fs.active_vfo;
    }
    SPI_Assets_LoadAll(&g_flash);
    Flash_LoadBandCal(&g_flash, g_band_cal);  /* defaults kept on failure */
  }
  LCD_Render_Init();
  SDR_UI_Init();

  /* Boot logo — read from SPI flash, center on display, hold 1.5 s.
   * Skipped if flash failed or sector is erased (first pixel == 0xFFFF). */
  if (boot_flash_ok) {
    static uint16_t s_logo_raw[LCD_W];
    if (Flash_ReadLogoScanline(&g_flash, 0U, LCD_W, s_logo_raw) == HAL_OK
        && s_logo_raw[0] != 0xFFFFU) {
      uint16_t *ln = LCD_GetLineBuf();
      LCD_Clear(0x0000U);
      for (uint16_t y = 0U; y < LCD_H; y++) {
        if (y > 0U && Flash_ReadLogoScanline(&g_flash, y, LCD_W, s_logo_raw) != HAL_OK) break;
        for (uint16_t x = 0U; x < LCD_W; x++) ln[x] = SWAP16(s_logo_raw[x]);
        LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, ln, LCD_W);
      }
      HAL_Delay(1500U);
    }
  }

  SDR_UI_DrawFrame(CSDR_AUDIO_SAMPLE_RATE, DSP_FFT_SIZE);
  SDR_UI_SetFooterFreq(g_sdr.freq_hz, (uint32_t)g_sdr.step);

  /* Delay nhỏ trước I2C để bus settle sau power-on */
  HAL_Delay(10);

  /* WM8731 */
  /* Compute LHPVOL from flash-loaded volume.  Same formula as csdr_apply_volume.
   * Boot recovery below guards vol=0; wm_out_vol is reused in the BYPASS-fix WM8731_Init. */
  if (g_sdr.volume == 0U) g_sdr.volume = 20U;  /* recover vol=0 before formula */
  uint8_t wm_out_vol = (uint8_t)(90U + ((uint16_t)g_sdr.volume * 31U / 100U));
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

  /* RF AGC — syncs tracked target to the hardware value set above */
  RFAGC_Init(&g_rfagc);
  g_rfagc.target_x2 = g_att.current_atten_x2;   /* no step-change on first update */
  RFAGC_SetEnabled(&g_rfagc, g_sdr.rf_agc_on, g_att.current_atten_x2);

  /* PA overcurrent protection: INA226 trên I2C2, ALERT hardware-gate bias line */
  PA_OC_Init(&hi2c2);
  { float _lim = (float)g_sdr.pa_oc_limit_idx * 0.1f;
    PA_OC_SetCurrentLimit(_lim);
    g_pa_cfg.current_warn_a = _lim * 0.75f;
    g_pa_cfg.current_trip_a = _lim * 0.90f; }

  /* PA protection manager: centralized state machine (foldback, trip, cooldown) */
  PA_Protect_Init();

  /* BPF + LPF */
  BPF_LPF_Init();
  csdr_apply_band(g_sdr.band_idx);

  /* DSP */
  DSP_Init(&g_dsp, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetTxPassband(&g_dsp, (float)g_sdr.tx_audio_low_hz, (float)g_sdr.tx_audio_high_hz);
  DSP_SetNotch(&g_dsp, g_sdr.notch_on, (float)g_sdr.notch_hz);
  DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetIFShift(&g_dsp, (int32_t)g_sdr.if_shift_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
  DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
  DSP_SetDCOffset(&g_dsp, g_sdr.dc_i_offset, g_sdr.dc_q_offset);
  AGC_SetMode(&g_dsp.agc, g_sdr.mode, g_sdr.agc_speed, CSDR_AUDIO_SAMPLE_RATE);
  DSP_NB_Set(&g_dsp, g_sdr.nb_on, g_sdr.nb_level);
  NRSpec_Init();  /* build NR2 FFT/window tables before any DSP_NR_Set */
  DSP_NR_Set(&g_dsp, g_sdr.nr_mode, g_sdr.nr_level);
  DSP_BC_Set(&g_dsp, g_sdr.bc_mode);
  DSP_SetSquelch(&g_dsp, g_sdr.squelch);
  /* LHPVOL is set by wm_out_vol in the WM8731_Init BYPASS-fix block below (after SAI start).
   * csdr_apply_volume is NOT called here — SAI has not started yet. */
  g_dsp.rx_volume_scale = (g_sdr.volume == 0U) ? 0.0f : 1.0f;
  /* CW-specific DSP init (after DSP_Init which seeds 700 Hz) */
  DSP_SetCWPitch(&g_dsp, g_sdr.cw_pitch_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetCWReverse(&g_dsp, g_sdr.cw_reverse);
  DSP_SetSidetoneVol(&g_dsp, g_sdr.sidetone_vol);
  if (g_sdr.mode == MODE_CW)
    DSP_SetBW(&g_dsp, (float)g_sdr.cw_filter_hz);
  CWDec_Init(&g_cw_dec);
  g_cw_dec.dit_ms = 1200U / (uint32_t)g_sdr.cw_wpm;
  /* Keyer init */
  g_cw_keyer.mode          = (CWKeyerMode_t)g_sdr.keyer_mode;
  g_cw_keyer.paddle_reverse= g_sdr.paddle_reverse;
  g_cw_keyer.wpm           = g_sdr.cw_wpm;
  g_cw_keyer.bkin          = (CWBkin_t)g_sdr.cw_bkin;
  g_cw_keyer.bk_delay_ms   = g_sdr.cw_bk_delay_ms;
  CWKeyer_Init(&g_cw_keyer);

  /* Encoder – TIM3 quadrature (PB4/PB5), initialised as encoder in MX_TIM3_Init */
  Encoder_Init(&g_encoder, &htim3);

  /* PCA9555 button expander – all function keys on I2C2. */
  Input_Init();
  /* Selftest: when HAS_PCA9555=1 the init result is captured in the dbg counters;
   * when HAS_PCA9555=0 the driver is a no-op so probe directly to detect presence. */
#if HAS_PCA9555
  bool boot_keys_ok = (dbg_pca_timeout_count == 0U && dbg_pca_init_attempts > 0U);
#else
  bool boot_keys_ok = (HAL_I2C_IsDeviceReady(&hi2c2,
                           (uint16_t)INPUT_PCA9555_ADDR, 2U, 5U) == HAL_OK);
#endif
  Key_InitPCA(&k_menu, &g_pca9555_raw, PCA_BIT_MENU);
  Key_InitPCA(&k_f1,   &g_pca9555_raw, PCA_BIT_F1);
  Key_InitPCA(&k_f2,   &g_pca9555_raw, PCA_BIT_F2);
  Key_InitPCA(&k_f3,   &g_pca9555_raw, PCA_BIT_F3);
  Key_InitPCA(&k_f4,   &g_pca9555_raw, PCA_BIT_F4);
  Key_InitPCA(&k_band, &g_pca9555_raw, PCA_BIT_BAND);
  Key_InitPCA(&k_mode, &g_pca9555_raw, PCA_BIT_MODE);
  Key_InitPCA(&k_tune, &g_pca9555_raw, PCA_BIT_TUNE);
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
    .set_nr_level    = cat_set_nr_level,
    .set_bc          = cat_set_bc,
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
    .get_nr_level    = cat_get_nr_level,
    .get_bc          = cat_get_bc,
    .get_nb          = cat_get_nb,
    .get_bw          = cat_get_bw,
    .get_agc_fast    = cat_get_agc_fast,
    .get_squelch     = cat_get_squelch,
    .get_rit_hz      = cat_get_rit_hz,
    .get_step        = cat_get_step,
    .set_if_shift    = cat_set_if_shift,
    .get_if_shift    = cat_get_if_shift,
    .set_lo_cut      = cat_set_lo_cut,
    .get_lo_cut      = cat_get_lo_cut,
    /* VFO B + active-VFO selection */
    .set_vfo_b_freq  = cat_set_vfo_b_freq,
    .set_vfo_b_mode  = cat_set_vfo_b_mode,
    .set_vfo_b_bw    = cat_set_vfo_b_bw,
    .set_vfo_b_lo_cut = cat_set_vfo_b_lo_cut,
    .get_vfo_b_freq  = cat_get_vfo_b_freq,
    .get_vfo_b_mode  = cat_get_vfo_b_mode,
    .get_vfo_b_bw    = cat_get_vfo_b_bw,
    .get_vfo_b_lo_cut = cat_get_vfo_b_lo_cut,
    .set_active_vfo  = cat_set_active_vfo,
    .get_active_vfo  = cat_get_active_vfo,
    .set_rf_agc      = cat_set_rf_agc,
    .get_rf_agc      = cat_get_rf_agc,
    .set_tx_power    = cat_set_tx_power,
    .get_tx_power    = cat_get_tx_power,
    .get_swr_x100    = cat_get_swr_x100,
    .get_alc_reduction_pct = cat_get_alc_reduction_pct,
    .set_cw_wpm      = cat_set_cw_wpm,
    .get_cw_wpm      = cat_get_cw_wpm,
    .set_usb_stream  = cat_set_usb_stream,
    .get_usb_stream  = cat_get_usb_stream,
    .set_tune        = cat_set_tune,
    .get_tune        = cat_get_tune,
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

  /* WM8731 BYPASS fix: WM8731 resets ALL registers (incl BYPASS=1, DACPD, IWL) on first MCLK.
   * WM8731 NACKs all I2C while MCLK is active (confirmed: disconnect MCLK → writes succeed).
   * Fix: stop SAI (MCLK off) → full WM8731_Init (all 9 registers) → restart SAI.
   * Second MCLK start retains the registers written during MCLK-off. */
  {
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_DMAStop(&hsai_BlockB1);
    HAL_Delay(5U);  /* let MCLK line discharge */

    WM8731_Config_t wm_fix = {
      .hi2c         = &hi2c1,
      .i2c_addr     = WM8731_I2C_ADDR,
      .sample_rate  = CSDR_AUDIO_SAMPLE_RATE,
      .input_volume = 23U,
      .output_volume = wm_out_vol,
      .line_in      = true
    };
    dbg_wm8731_postinit_ok = (uint32_t)WM8731_Init(&wm_fix);  /* 0=HAL_OK */

    /* Restart SAI — TX master first, then RX slave */
    RuntimeDiag_TxHalfFilled(0U);
    RuntimeDiag_TxHalfFilled(1U);
    HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)s_tx_buf,
        CSDR_AUDIO_BUF_TOTAL * 2U);
    HAL_Delay(10U);
    dbg_sai_init_ret = HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)s_rx_buf,
        CSDR_AUDIO_BUF_TOTAL * 2U);
  }

  /* ── Self-test: record hardware presence, trigger top-bar warning if any fail ──
   * Results come from init return values already captured above — no extra I/O.
   * Boot continues normally regardless; no peripheral is disabled or retried. */
  SelfTest_Run(
    boot_flash_ok,
    (dbg_wm8731_ok    == (uint32_t)HAL_OK),
    g_sdr.si5351_ok,
    g_pa_oc.ina_ok,
    (dbg_sai_init_ret == (uint32_t)HAL_OK),
    boot_keys_ok
  );
  /* ── Critical fault reinit: one attempt for CODEC, PLL, INA ─────────────
   * These three make the radio non-functional or unsafe without them.
   * Runs unconditionally — independent of HW_FAULT_WARN.
   * SAI DMA is already running; all reinits are pure I2C, no audio impact.
   * On persistent failure HW_FAULT_* is set; CSDR_Loop will halt audio/RF. */
  {
    /* CODEC (WM8731) — audio codec */
    if (!g_selftest.items[1].ok) {
      uint8_t wm_vol2 = (g_sdr.volume == 0U) ? 0x2FU
                        : (uint8_t)(90U + ((uint16_t)g_sdr.volume * 31U / 100U));
      WM8731_Config_t wm2 = {
        .hi2c = &hi2c1, .i2c_addr = WM8731_I2C_ADDR,
        .sample_rate = CSDR_AUDIO_SAMPLE_RATE,
        .input_volume = 23U, .output_volume = wm_vol2, .line_in = true
      };
      dbg_wm8731_ok = (uint32_t)WM8731_Init(&wm2);
      if (dbg_wm8731_ok == HAL_OK) {
        uint8_t act[2] = { 0x12U, 0x01U };
        HAL_I2C_Master_Transmit(&hi2c1, WM8731_I2C_ADDR, act, 2U, 100U);
        g_selftest.items[1].ok = true;
      } else {
        HW_Fault_Set(HW_FAULT_CODEC);
      }
    }

    /* PLL (SI5351) — VFO / LO synthesis */
    if (!g_selftest.items[2].ok) {
      dbg_si5351_ok = (uint32_t)SI5351_Init(&g_si5351, &hi2c1,
                                              SI5351_I2C_ADDR, SI5351_XTAL_HZ);
      if (dbg_si5351_ok == HAL_OK) {
        g_sdr.si5351_ok = true;
        SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
        g_selftest.items[2].ok = true;
      } else {
        HW_Fault_Set(HW_FAULT_PLL);
      }
    }

  }

  /* SAI absent → no DMA audio stream → RX non-functional: treat as critical.
   * Set unconditionally so HW_Fault_IsCritical() always catches it. */
  if (!g_selftest.items[4].ok) HW_Fault_Set(HW_FAULT_SAI);

  /* INA226 absent → RX OK, TX blocked via PA_Protect_IsTxAllowed().
   * Set unconditionally so the TX guard fires regardless of HW_FAULT_WARN. */
  if (!g_selftest.items[3].ok) HW_Fault_Set(HW_FAULT_INA226);

#if HW_FAULT_WARN
  /* Truly non-critical: RX unaffected, warning header only. */
  if (!g_selftest.items[0].ok) HW_Fault_Set(HW_FAULT_FLASH);   /* FLASH */
  if (!g_selftest.items[5].ok) HW_Fault_Set(HW_FAULT_KEYS);    /* KEYS  */
#endif
  if (SelfTest_AnyFail()) g_sdr.display_dirty |= DIRTY_HDR;

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
    if (g_usb_audio.usb_streaming && g_sdr.usb_iq_stream && g_sdr.usb_mode != 0U)
      USB_Audio_WriteRX(&g_usb_audio, s_rx_buf, CSDR_AUDIO_BLOCK_SIZE);
    dbg_rx_sample_0 = s_rx_buf[0];
    dbg_rx_sample_1 = s_rx_buf[1];
    RuntimeDiag_AudioBlockBegin();
    if (g_sdr.tx_mode) {
      DSP_ProcessTX(&g_dsp, s_tx_buf, CSDR_AUDIO_BLOCK_SIZE);
    } else {
      DSP_Process(&g_dsp, s_rx_buf, s_tx_buf, CSDR_AUDIO_BLOCK_SIZE);
    }
    if (g_usb_audio.usb_streaming && !g_sdr.usb_iq_stream && !g_sdr.tx_mode && g_sdr.usb_mode != 0U)
      USB_Audio_WriteRX(&g_usb_audio, s_tx_buf, CSDR_AUDIO_BLOCK_SIZE);
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
    dbg_vol_scale_q16 = (uint32_t)(g_dsp.rx_volume_scale * 65536.0f);
    {
      int32_t pk = 0;
      for (uint32_t _i = 0; _i < CSDR_AUDIO_BLOCK_SIZE * 2U; _i++) {
        int32_t v = s_tx_buf[_i]; if (v < 0) v = -v;
        if (v > pk) pk = v;
      }
      dbg_tx_peak = pk;
    }
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
    if (g_usb_audio.usb_streaming && g_sdr.usb_iq_stream && g_sdr.usb_mode != 0U)
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
    if (g_usb_audio.usb_streaming && !g_sdr.usb_iq_stream && !g_sdr.tx_mode && g_sdr.usb_mode != 0U)
      USB_Audio_WriteRX(&g_usb_audio,
                         s_tx_buf + CSDR_AUDIO_BLOCK_SIZE*2,
                         CSDR_AUDIO_BLOCK_SIZE);
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

/* Draw a one-shot hardware-missing warning covering the SPEC+WF zone.
 * Builds the component list dynamically from g_hw_fault_mask so any
 * combination of failures is shown correctly.  Subsequent calls are
 * no-ops (s_drawn flag); display persists until reboot.
 *
 * Always compiled — called unconditionally from the critical-halt path in
 * CSDR_Loop regardless of HW_FAULT_WARN.  The non-critical call site in the
 * spectrum timer block is still gated by #if HW_FAULT_WARN. */
static void csdr_draw_hw_fault_warning(void)
{
  static bool s_drawn = false;
  if (s_drawn) return;
  s_drawn = true;

  LCD_FillRect(SPEC_X, SPEC_Y,
               (uint16_t)(SPEC_X + SPEC_W - 1U),
               (uint16_t)(WF_Y  + WF_H  - 1U),
               0x0000U);

  /* Build "FLASH CODEC PLL INA226 SAI KEYS" from active fault bits */
  char components[48] = "";
  if (g_hw_fault_mask & HW_FAULT_FLASH)  { strcat(components, "FLASH ");  }
  if (g_hw_fault_mask & HW_FAULT_CODEC)  { strcat(components, "CODEC ");  }
  if (g_hw_fault_mask & HW_FAULT_PLL)    { strcat(components, "PLL ");    }
  if (g_hw_fault_mask & HW_FAULT_INA226) { strcat(components, "INA226 "); }
  if (g_hw_fault_mask & HW_FAULT_SAI)    { strcat(components, "SAI ");    }
  if (g_hw_fault_mask & HW_FAULT_KEYS)   { strcat(components, "KEYS ");   }
  /* trim trailing space */
  size_t clen = strlen(components);
  if (clen > 0U) components[clen - 1U] = '\0';

  static uint16_t s_ln[LCD_W];
  /* Raw RGB565 — LCD_LineStr/LCD_LineFill apply SWAP16 internally.
   * Do NOT pre-wrap with SWAP16 here; double-SWAP produces wrong colours
   * on ST7789 where SWAP16 includes colour inversion (not pure byte-swap). */
  const uint16_t FG1 = 0xF800U;   /* red      */
  const uint16_t FG2 = 0xF040U;   /* dark red */
  const uint16_t BG  = 0x0000U;

  const char *line1 = "! HARDWARE NOT FOUND";
  const char *line2 = components;
  const char *line3 = "TRY REBOOT TO FIX";
  const char *line4 = "IF NOT DONE CHECK HARDWARE";

  uint16_t tw1 = (uint16_t)(strlen(line1) * Font6x8.width);
  uint16_t tw2 = (uint16_t)(strlen(line2) * Font5x8.width);
  uint16_t tw3 = (uint16_t)(strlen(line3) * Font5x8.width);
  uint16_t tw4 = (uint16_t)(strlen(line4) * Font5x8.width);
  uint16_t x1  = tw1 < SPEC_W ? (uint16_t)((SPEC_W - tw1) / 2U) : 0U;
  uint16_t x2  = tw2 < SPEC_W ? (uint16_t)((SPEC_W - tw2) / 2U) : 0U;
  uint16_t x3  = tw3 < SPEC_W ? (uint16_t)((SPEC_W - tw3) / 2U) : 0U;
  uint16_t x4  = tw4 < SPEC_W ? (uint16_t)((SPEC_W - tw4) / 2U) : 0U;

  uint16_t total_h = Font6x8.height + 4U
                   + Font5x8.height + 4U
                   + Font5x8.height + 4U
                   + Font5x8.height;
  uint16_t y0 = (uint16_t)(SPEC_Y + (SPEC_H + WF_H - total_h) / 2U);
  uint16_t y1 = y0 + Font6x8.height + 4U;
  uint16_t y2 = y1 + Font5x8.height + 4U;
  uint16_t y3 = y2 + Font5x8.height + 4U;

  for (uint16_t r = 0U; r < Font6x8.height; r++) {
    LCD_LineFill(s_ln, 0U, SPEC_W, BG);
    LCD_LineStr(s_ln, x1, r, line1, &Font6x8, FG1, BG);
    LCD_PushWindow(SPEC_X, y0 + r,
                   (uint16_t)(SPEC_X + SPEC_W - 1U), y0 + r,
                   s_ln, SPEC_W);
  }
  for (uint16_t r = 0U; r < Font5x8.height; r++) {
    LCD_LineFill(s_ln, 0U, SPEC_W, BG);
    LCD_LineStr(s_ln, x2, r, line2, &Font5x8, FG2, BG);
    LCD_PushWindow(SPEC_X, y1 + r,
                   (uint16_t)(SPEC_X + SPEC_W - 1U), y1 + r,
                   s_ln, SPEC_W);
  }
  for (uint16_t r = 0U; r < Font5x8.height; r++) {
    LCD_LineFill(s_ln, 0U, SPEC_W, BG);
    LCD_LineStr(s_ln, x3, r, line3, &Font5x8, UI_STATUS_LBL, BG);
    LCD_PushWindow(SPEC_X, y2 + r,
                   (uint16_t)(SPEC_X + SPEC_W - 1U), y2 + r,
                   s_ln, SPEC_W);
  }
  for (uint16_t r = 0U; r < Font5x8.height; r++) {
    LCD_LineFill(s_ln, 0U, SPEC_W, BG);
    LCD_LineStr(s_ln, x4, r, line4, &Font5x8, UI_STATUS_LBL, BG);
    LCD_PushWindow(SPEC_X, y3 + r,
                   (uint16_t)(SPEC_X + SPEC_W - 1U), y3 + r,
                   s_ln, SPEC_W);
  }
}

void CSDR_PrepareShutdown(void)
{
  csdr_save_settings();

  LCD_Clear(0x0000U);

  static uint16_t s_ln[LCD_W];
  const uint16_t FG  = 0xF800U;   /* red */
  const uint16_t DIM = 0x4A49U;   /* dim grey */
  const uint16_t BG  = 0x0000U;

  const char *title = "POWERING OFF";
  uint16_t tw = (uint16_t)(strlen(title) * Font6x8.width);
  uint16_t x0 = tw < LCD_W ? (uint16_t)((LCD_W - tw) / 2U) : 0U;

  uint16_t total_h = Font6x8.height + 8U + Font6x8.height;
  uint16_t y0 = (uint16_t)((LCD_H - total_h) / 2U);
  uint16_t y1 = y0 + Font6x8.height + 8U;

  for (uint16_t r = 0U; r < Font6x8.height; r++) {
    LCD_LineFill(s_ln, 0U, LCD_W, BG);
    LCD_LineStr(s_ln, x0, r, title, &Font6x8, FG, BG);
    LCD_PushWindow(0U, y0 + r, (uint16_t)(LCD_W - 1U), y0 + r, s_ln, LCD_W);
  }

  static const char *const dots[] = { "...", "..", "." };
  for (uint8_t i = 0U; i < 3U; i++) {
    uint16_t dw = (uint16_t)(strlen(dots[i]) * Font6x8.width);
    uint16_t xd = dw < LCD_W ? (uint16_t)((LCD_W - dw) / 2U) : 0U;
    for (uint16_t r = 0U; r < Font6x8.height; r++) {
      LCD_LineFill(s_ln, 0U, LCD_W, BG);
      LCD_LineStr(s_ln, xd, r, dots[i], &Font6x8, DIM, BG);
      LCD_PushWindow(0U, y1 + r, (uint16_t)(LCD_W - 1U), y1 + r, s_ln, LCD_W);
    }
    HAL_Delay(270U);
  }

  PWR_Shutdown();
}

/* Called after any DSP_SetMode to sync CW decoder + UI strip */
static void csdr_on_mode_changed(SDR_Mode_t old_mode)
{
    CWDec_Reset(&g_cw_dec);
    if (g_sdr.mode == MODE_CW) {
        /* Entering CW: apply CW-specific filter and pitch */
        DSP_SetBW(&g_dsp, (float)g_sdr.cw_filter_hz);
        g_sdr.bw_hz = (float)g_sdr.cw_filter_hz;
        DSP_SetCWPitch(&g_dsp, g_sdr.cw_pitch_hz, CSDR_AUDIO_SAMPLE_RATE);
        /* Seed the decoder WPM */
        g_cw_dec.dit_ms = 1200U / (uint32_t)g_sdr.cw_wpm;
        /* Reset keyer — avoids phantom key-down from previous mode */
        g_cw_keyer.ky_state = KY_IDLE;
        g_cw_keyer.key_down = false;
        g_cw_keyer.ptt_out  = false;
        g_dsp.cw_key_out    = false;
    } else {
        g_dsp.cw_key_out = false;
    }
    if (g_sdr.mode == MODE_CW) {
        SDR_UI_SetCWDecActive(g_sdr.cw_decode_on);
    }
    if (old_mode == MODE_CW && g_sdr.mode != MODE_CW) {
        g_sdr.cw_decode_on = false;
        SDR_UI_SetCWDecActive(false);
        SDR_UI_ClearCWText();
    }
}

void CSDR_Loop(void)
{
  RuntimeDiag_MainLoopBeat();

  /* PA overcurrent: xử lý fault từ EXTI ISR (tắt TX, báo lỗi UI, xóa INA226 latch) */
  PA_OC_HandleFaultInLoop();

  /* ── Critical hardware fault: CODEC / PLL / SAI absent after reinit ──
   * Audio and RF are non-functional.  Skip the DSP/audio/RF pipeline;
   * keep fault overlay, watchdog, power management, and CAT/CDC alive so
   * the flash tool and remote diagnostics still work over USB. */
  if (HW_Fault_IsCritical()) {
    static uint32_t s_crit_disp_ms = 0U;
    static uint32_t s_crit_cat_ms  = 0U;
    uint32_t now_c = HAL_GetTick();
    if ((now_c - s_crit_disp_ms) >= 500U) {
      s_crit_disp_ms = now_c;
      csdr_draw_hw_fault_warning();
    }
    RuntimeDiag_ServiceSlow(now_c);
    RuntimeDiag_WatchdogRefreshIfHealthy(now_c);
    PWR_Poll();
    FlashProto_Process();
    if (g_sdr.usb_mode != 0U) {
      if ((now_c - s_crit_cat_ms) >= 10U) {
        s_crit_cat_ms = now_c;
        CAT_Process(&g_cat);
      }
      CAT_FlushTX(&g_cat);
    }
    return;
  }

  csdr_process_audio_pending();

  /* Input: refresh PCA9555 once, then dispatch encoder + keys */
  Input_Scan();
  csdr_handle_encoder();
  csdr_handle_keys();

  /* CW keyer: update state machine, feed key_out to DSP (volatile write) */
  if (g_sdr.mode == MODE_CW) {
    CWKeyer_Update(&g_cw_keyer);
    g_dsp.cw_key_out = g_cw_keyer.key_down;
    /* BK-IN PTT: assert/deassert TX based on keyer ptt_out */
    if (g_sdr.cw_bkin != BKIN_OFF) {
      static bool s_keyer_ptt_prev = false;
      if (g_cw_keyer.ptt_out != s_keyer_ptt_prev) {
        s_keyer_ptt_prev = g_cw_keyer.ptt_out;
        /* Only take control when manual PTT is not already held */
        bool manual_ptt = (k_ptt.state == KS_PRESSED || k_ptt.state == KS_HELD);
        if (!manual_ptt) {
          bool want_tx = g_cw_keyer.ptt_out;
          if (want_tx != g_sdr.tx_mode) {
            g_sdr.tx_mode = want_tx;
            g_sdr.display_dirty |= want_tx
              ? (uint8_t)(DIRTY_HDR | DIRTY_VFO | DIRTY_SBR)
              : (uint8_t)DIRTY_ALL;
            csdr_apply_tx();
          }
        }
      }
    }
  } else {
    g_dsp.cw_key_out = false;
  }

  /* CW decoder – main-loop timing decoder, runs every loop iteration.
   * CWDec_Update is cheap (a few compares + HAL_GetTick); no timer guard needed.
   * Decoded text is pushed to the INFO strip when the buffer changes. */
  if (g_sdr.cw_decode_on && g_sdr.mode == MODE_CW && !g_sdr.tx_mode) {
    if (CWDec_Update(&g_cw_dec, g_dsp.cw_env.keyed)) {
      char cw_buf[CWDEC_TEXT_LEN + 1U];
      CWDec_GetText(&g_cw_dec, cw_buf, (uint8_t)(LCD_W / 6U));
      SDR_UI_DrawCWText(cw_buf);
    }
  }

  /* Timed tasks */
  static uint32_t t_analog=0, t_fan=0, t_pwr=0, t_disp=0, t_cat=0, t_wf=0, t_spec=0, t_tx_spec=0;
  static uint32_t t_rfagc = 0U;
  static uint32_t t_pa_prot = 0U;
  static uint32_t t_vox = 0U;
  static uint32_t t_tx_meter = 0U;
  static uint32_t t_pa_warn = 0U;
  uint32_t now = HAL_GetTick();

  if (now - t_analog >= 100U) {
    t_analog = now;
    Analog_Update();
    { /* Apply per-band SWR scale correction */
      int16_t sc = g_band_cal[g_sdr.band_idx].swr_scale;
      if (sc <  50) sc =  50;
      if (sc > 200) sc = 200;
      uint32_t swr_scaled = ((uint32_t)g_analog.swr_x100 * (uint32_t)(uint16_t)sc + 50U) / 100U;
      g_analog.swr_x100 = (uint16_t)(swr_scaled > 0xFFFFU ? 0xFFFFU : swr_scaled);
    }
    SDR_UI_UpdateSMeter_SetVoltage((int16_t)(g_analog.voltage_mv / 100));
  }
  if (now - t_fan    >= 1000U){ t_fan    = now; Fan_Update(g_analog.temp_c); }
  if (now - t_pwr    >= 100U) { t_pwr    = now; PWR_Poll(); }

  /* PA protection manager: runs every 20 ms, independent of UI/audio rate.
   * Evaluates IIR-filtered SWR + temperature + current, drives foldback/trip
   * state machine, and applies power limits via request_gain_reapply(). */
  if (now - t_pa_prot >= 20U) {
    t_pa_prot = now;
    PA_Protect_Update();
  }

  /* VOX poll: every 20 ms; non-destructive peek at USB TX ring level. */
  if (now - t_vox >= 20U) {
    t_vox = now;
    csdr_vox_poll();
  }

  /* RF AGC: update PE4302 attenuation from DSP signal level, RX only.
   * Runs at RFAGC_INTERVAL_MS (20 ms).  All SPI writes are bit-bang and
   * complete in < 2 µs — negligible main-loop impact. */
  if (!g_sdr.tx_mode && (now - t_rfagc >= RFAGC_INTERVAL_MS)) {
    t_rfagc = now;
    uint8_t new_x2 = RFAGC_Update(&g_rfagc, g_dsp.signal_power_db, g_sdr.tx_mode);
    if (new_x2 != RFAGC_NO_CHANGE) {
      PE4302_SetAttn_Raw(&g_att, new_x2);
      g_sdr.att_db = g_att.current_atten_db;  /* keep integer field in sync for CAT */
      g_sdr.display_dirty |= DIRTY_SBR | DIRTY_HDR;
    }
  }

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
#if HW_FAULT_WARN
    if (HW_Fault_Any()) { csdr_draw_hw_fault_warning(); } else
#endif
    if (!dbg_disable_lcd_dma) {
      static uint32_t s_spec_overload_ms = 0U;
      bool spec_ring_pressure = g_usb_audio.rx_overrun_pending ||
                                (g_usb_audio.rx_count > USB_AUDIO_OVERRUN_BYTES);
      /* Do NOT use RuntimeDiag_IsUiOverload() here: that flag reflects waterfall
       * suppression state and creates a circular feedback loop that permanently
       * kills spectrum whenever waterfall is suppressed (even spuriously). */
      if (spec_ring_pressure) {
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
    /* Do NOT reset t_spec here: let the RX spectrum timer run down so it fires
     * immediately (or nearly so) when TX ends, rather than waiting a full
     * 75 ms period after the transition. */
    if (now - t_tx_spec >= CSDR_UI_TX_SPEC_PERIOD_MS) {
      t_tx_spec = now;
      if (!dbg_disable_lcd_dma) {
        csdr_process_audio_pending();
        RuntimeDiag_UiRenderBegin();
        csdr_update_tx_spectrum();
        RuntimeDiag_UiRenderEnd();
      }
    }
    /* TX meter (ALC/SWR): update at 5 Hz — fast enough to track voice peaks. */
    if (now - t_tx_meter >= 200U) {
      t_tx_meter = now;
      SDR_UI_UpdateTXMeters((int32_t)g_analog.alc_percent,
                            (int32_t)(g_analog.swr_x100 / 10));
    }
  } else {
    t_tx_meter = now;  /* reset so meter fires immediately on next TX */
  }

  /* PA protection warning in the INFO zone: refresh at 500 ms.
   * Runs in both TX and RX mode so warning persists after a TRIP clears TX. */
  if (now - t_pa_warn >= 500U) {
    t_pa_warn = now;
    SDR_UI_UpdatePAWarn(PA_Protect_GetState(), PA_Protect_GetFault());
  }

  /* Waterfall: ~13 fps in RX.  Frozen in TX to avoid the 480×72 FMC push
   * (8.06ms) competing with SAI TX DMA fill at WSJT-X audio deadlines. */
  if (!g_sdr.tx_mode && (now - t_wf >= CSDR_UI_WF_RX_PERIOD_MS)) {
    t_wf = now;
    /* Adaptive waterfall skip: suppress FMC push only under sustained ring load.
     *
     * Sole trigger: rx ring occupancy > 75% (real-time, same 75 ms tick).
     * Entry requires 2 consecutive high-pressure ticks (≥150 ms) to filter
     * isolated spikes.  Recovery: 150 ms (2 ticks) after pressure clears.
     *
     * rx_overrun_per_sec was removed: it updates once per second, so a brief
     * burst causes up to 2 s of false suppression after the cause clears.
     * rx_overrun_pending is consumed here only to drain the flag. */
    {
      static uint32_t s_wf_overload_ms   = 0U;
      static uint8_t  s_wf_overload_hits = 0U;  /* consecutive high-pressure ticks */

      /* Ring occupancy: sole suppression trigger.
       * rx_overrun_per_sec was removed — it is updated once per second so
       * even a brief overrun burst keeps waterfall suppressed for an entire
       * second after the cause clears.  ring_pressure is instantaneous (same
       * 75 ms tick) and the 2-hit hysteresis below already filters spikes.
       * tx_underrun_per_sec is irrelevant: waterfall is frozen during TX. */
      bool ring_pressure = (g_usb_audio.rx_count > USB_AUDIO_OVERRUN_BYTES);
      if (g_usb_audio.rx_overrun_pending) g_usb_audio.rx_overrun_pending = false;

      if (ring_pressure) {
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
#if HW_FAULT_WARN
    if (!HW_Fault_Any())
#endif
    if (!dbg_disable_lcd_dma) {
      csdr_process_audio_pending();
      RuntimeDiag_UiRenderBegin();
      csdr_update_waterfall();
      RuntimeDiag_UiRenderEnd();
    }
  }
  /* Do NOT reset t_wf in TX mode: let the waterfall timer fire as soon as
   * TX ends rather than waiting another full 75 ms period. */

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
    /* Piggyback the PW badge (measured fwd power) on the existing 1 Hz TX
     * meter cadence — same budget already spent on SWR/ALC, no new timer. */
    if (disp_due && g_sdr.tx_mode) g_sdr.display_dirty |= (uint8_t)DIRTY_VFO;
    if (!dbg_disable_lcd_dma) {
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
  RuntimeDiag_WatchdogRefreshIfHealthy(now);

  if (now - t_cat    >= 10U) {
    t_cat = now;
    if (g_sdr.usb_mode != 0U) CAT_Process(&g_cat);
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
      /* Defer SAI stop/start for 500ms after USB connect — keeps CAT responses timely
       * during flrig init sequence; dirty flag stays set and fires after window closes. */
      if (!s_usb_connect_tick || (now - s_usb_connect_tick) >= 500U) {
        g_sdr.cat_vol_dirty = false;
        csdr_apply_volume(g_sdr.volume);
      }
    }
    if (g_sdr.cat_mode_dirty) {
      g_sdr.cat_mode_dirty = false;
      SDR_Mode_t old_mode = g_sdr.mode;
      DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
      DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
      AGC_SetMode(&g_dsp.agc, g_sdr.mode, g_sdr.agc_speed, CSDR_AUDIO_SAMPLE_RATE);
      csdr_on_mode_changed(old_mode);
      g_sdr.cat_rit_dirty = true;  /* recompute nco_if with updated sl_sign for new mode */
    }
    if (g_sdr.cat_tx_dirty) {
      g_sdr.cat_tx_dirty = false;
      csdr_apply_tx();
    }
    if (g_sdr.cat_att_dirty) {
      g_sdr.cat_att_dirty = false;
      PE4302_SetAttn_dB(&g_att, g_sdr.att_db);
      g_sdr.att_db = g_att.current_atten_db;
      /* Manual change: notify RF AGC so it respects the cooldown */
      RFAGC_NotifyManual(&g_rfagc, g_att.current_atten_x2);
      g_sdr.display_dirty |= DIRTY_SBR | DIRTY_HDR;
    }
    if (g_sdr.cat_rit_dirty) {
      g_sdr.cat_rit_dirty = false;
      /* nco_if = IF shift + RIT + SL contribution.
       * SL (low-cut) shifts the passband centre away from DC:
       *   USB/CW/DIGU: +sl_hz (passband starts above carrier)
       *   LSB/DIGL:    -sl_hz (passband starts below carrier)
       *   AM/FM:        0     (symmetric / fixed filter, no low cut) */
      int32_t sl_sign = 0;
      if (g_sdr.mode == MODE_USB || g_sdr.mode == MODE_CW ||
          g_sdr.mode == MODE_DIGU || g_sdr.mode == MODE_FREEDV)
          sl_sign = +1;
      else if (g_sdr.mode == MODE_LSB || g_sdr.mode == MODE_DIGL)
          sl_sign = -1;
      int32_t eff_if = (int32_t)g_sdr.if_shift_hz
                     + (g_cat.rit_on ? (int32_t)g_sdr.rit_hz : 0)
                     + sl_sign * (int32_t)g_sdr.sl_hz;
      DSP_SetIFShift(&g_dsp, eff_if, CSDR_AUDIO_SAMPLE_RATE);
    }
    /* Debounced freq save: write flash 3 s after the last freq change,
     * or immediately if flrig disconnected with a pending unsaved change. */
    if (s_save_on_disconnect || (s_freq_save_tick && (now - s_freq_save_tick) >= 3000U)) {
      s_freq_save_tick      = 0U;
      s_save_on_disconnect  = false;
      csdr_save_settings();
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

  /* Flash protocol: execute any pending command and send response.
   * Runs before CAT_FlushTX so the CDC TX is free when CAT needs it. */
  FlashProto_Process();

  /* CAT TX flush — single call site, runs every main-loop tick (~1 ms).
   * Positioned AFTER CAT_Process so new responses are drained on the same
   * tick they are enqueued.  On non-Process ticks this retries any remaining
   * FIFO bytes that were deferred by a previous busy-guard hit. */
  if (g_sdr.usb_mode != 0U) CAT_FlushTX(&g_cat);
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
  if (len > 0U && (buf[0] == FLASH_PROTO_MAGIC || FlashProto_IsActive())) {
      FlashProto_Receive(buf, (uint16_t)len);
  } else {
      CAT_Receive(&g_cat, buf, (uint16_t)len);
  }
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
  FlashProto_Init();
  CAT_Callbacks_t saved_cb;
  __set_BASEPRI(0x20U);
  saved_cb     = g_cat.cb;
  g_cat.rx_len = 0U;
  __set_BASEPRI(0U);
  CAT_Init(&g_cat, &saved_cb);
  /* If a freq change was pending (not yet flushed by debounce timer), save on next loop tick */
  if (s_freq_save_tick) s_save_on_disconnect = true;
  /* Open a 500ms window to suppress SAI stop/start volume apply during flrig init */
  s_usb_connect_tick = HAL_GetTick();
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
static void csdr_apply_mode_idx(uint8_t m)
{
  SDR_Mode_t new_mode = (m < MODE_COUNT) ? (SDR_Mode_t)m : g_sdr.mode;
  SDR_Mode_t old_mode = g_sdr.mode;
  if (new_mode == old_mode) { g_sdr.display_dirty |= DIRTY_ALL; return; }
  g_sdr.mode  = new_mode;
  g_sdr.bw_hz = default_bw_for_mode(new_mode);
  g_sdr.sl_hz = 0U;
  DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
  AGC_SetMode(&g_dsp.agc, g_sdr.mode, g_sdr.agc_speed, CSDR_AUDIO_SAMPLE_RATE);
  csdr_apply_nco_if();
  csdr_on_mode_changed(old_mode);
  g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
}

static void csdr_apply_band(uint8_t band)
{
  BPF_SetBand(band); LPF_SetBand(band);
  uint32_t f = BPF_BandToFreq(band);
  g_sdr.freq_hz = f; g_sdr.band_idx = band;
  DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, f + g_sdr.lo_offset_hz);
  g_sdr.display_dirty |= (DIRTY_HDR | DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
}

static void csdr_factory_reset(void)
{
  /* Preserve hardware calibration — these survive factory reset */
  int32_t  sv_xtal_ppm        = g_sdr.xtal_ppm;
  int32_t  sv_dc_i_offset     = g_sdr.dc_i_offset;
  int32_t  sv_dc_q_offset     = g_sdr.dc_q_offset;
  uint32_t sv_lo_offset_hz    = g_sdr.lo_offset_hz;
  int16_t  sv_smeter_offset   = g_sdr.smeter_offset_db;
  int16_t  sv_iq_gain         = g_sdr.iq_gain;
  int16_t  sv_iq_phase        = g_sdr.iq_phase;
  int16_t  sv_audio_gain_db   = g_sdr.audio_gain_db;
  uint8_t  sv_pa_watts        = g_sdr.pa_watts;
  uint8_t  sv_pa_oc_limit_idx = g_sdr.pa_oc_limit_idx;

  /* Reset operating state to factory defaults */
  g_sdr.freq_hz          = CSDR_FREQ_DEFAULT_HZ;
  g_sdr.mode             = MODE_USB;
  g_sdr.band_idx         = 3U;
  g_sdr.volume           = 20U;
  g_sdr.bw_hz            = 3000U;
  g_sdr.sl_hz            = 0U;
  g_sdr.if_shift_hz      = 0;
  g_sdr.squelch          = 0U;
  g_sdr.att_db           = 0U;
  g_sdr.step             = STEP_100;
  g_sdr.agc_speed        = 1U;
  g_sdr.nb_on            = false;
  g_sdr.nb_level         = 50U;
  g_sdr.nr_mode          = 0U;
  g_sdr.nr_level         = 50U;
  g_sdr.bc_mode          = 0U;
  g_sdr.rf_agc_on        = false;
  g_sdr.ext_alc_on       = false;
  g_sdr.mic_gain         = 50;
  g_sdr.digi_gain        = 70;
  g_sdr.tx_power         = 100U;
  g_sdr.tx_audio_low_hz  = 200U;
  g_sdr.tx_audio_high_hz = 2800U;
  g_sdr.notch_on         = false;
  g_sdr.notch_hz         = 1000;
  g_sdr.vox_on           = false;
  g_sdr.vox_gain         = 50U;
  g_sdr.vox_delay        = 500U;
  g_sdr.rit_hz           = 0;
  g_sdr.active_vfo       = 0U;
  g_sdr.vfo_b.freq_hz    = 14200000UL;
  g_sdr.vfo_b.mode       = MODE_USB;
  g_sdr.vfo_b.band_idx   = 5U;
  g_sdr.vfo_b.step       = STEP_100;
  g_sdr.vfo_b.rit_hz     = 0;
  g_sdr.vfo_b.bw_hz      = 3000U;
  g_sdr.vfo_b.sl_hz      = 0U;
  g_sdr.vfo_b.if_shift_hz = 0;
  g_sdr.usb_mode         = 1U;   /* On */
  g_sdr.usb_iq_stream    = false;
  g_sdr.tx_src           = 0U;   /* USB */
  g_sdr.cw_decode_on     = false;
  g_sdr.cw_pitch_hz      = 700U;
  g_sdr.cw_wpm           = 20U;
  g_sdr.keyer_mode       = 0U;
  g_sdr.paddle_reverse   = false;
  g_sdr.sidetone_vol     = 50U;
  g_sdr.cw_bkin          = 0U;
  g_sdr.cw_bk_delay_ms  = 200U;
  g_sdr.cw_reverse       = false;
  g_sdr.cw_filter_hz     = 500U;

  /* Restore calibration */
  g_sdr.xtal_ppm         = sv_xtal_ppm;
  g_sdr.dc_i_offset      = sv_dc_i_offset;
  g_sdr.dc_q_offset      = sv_dc_q_offset;
  g_sdr.lo_offset_hz     = sv_lo_offset_hz;
  g_sdr.smeter_offset_db = sv_smeter_offset;
  g_sdr.iq_gain          = sv_iq_gain;
  g_sdr.iq_phase         = sv_iq_phase;
  g_sdr.audio_gain_db    = sv_audio_gain_db;
  g_sdr.pa_watts         = sv_pa_watts;
  g_sdr.pa_oc_limit_idx  = sv_pa_oc_limit_idx;

  /* Apply to hardware and DSP */
  csdr_apply_band(3U);
  PE4302_SetAttn_dB(&g_att, 0U);
  RFAGC_SetEnabled(&g_rfagc, false, g_att.current_atten_x2);
  DSP_SetTxPassband(&g_dsp, 200.0f, 2800.0f);
  DSP_SetNotch(&g_dsp, false, 1000.0f);
  DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetIFShift(&g_dsp, 0, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetMode(&g_dsp, MODE_USB, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetBW(&g_dsp, 3000.0f);
  AGC_SetMode(&g_dsp.agc, MODE_USB, true, CSDR_AUDIO_SAMPLE_RATE);
  DSP_NB_Set(&g_dsp, false, 50U);
  DSP_NR_Set(&g_dsp, 0U, 50U);
  DSP_BC_Set(&g_dsp, 0U);
  DSP_SetSquelch(&g_dsp, 0U);
  DSP_SetCWPitch(&g_dsp, 700U, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetCWReverse(&g_dsp, false);
  DSP_SetSidetoneVol(&g_dsp, 50U);
  g_cw_keyer.mode           = (CWKeyerMode_t)0U;
  g_cw_keyer.paddle_reverse = false;
  g_cw_keyer.wpm            = 20U;
  g_cw_keyer.bkin           = (CWBkin_t)0U;
  g_cw_keyer.bk_delay_ms   = 200U;
  g_cw_dec.dit_ms           = 1200U / 20U;
  csdr_apply_volume(20U);
  csdr_save_settings();
  g_sdr.display_dirty |= DIRTY_ALL;
}

static void csdr_handle_encoder(void)
{
  int32_t delta = Encoder_GetDelta(&g_encoder);
  if (delta != 0) {
    if (BandSel_IsOpen()) {
      if (delta > 0) BandSel_CursorDown(); else BandSel_CursorUp();
      BandSel_Render();
      return;
    }
    if (ModeSel_IsOpen()) {
      if (delta > 0) ModeSel_CursorDown(); else ModeSel_CursorUp();
      ModeSel_Render();
      return;
    }
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
    s_freq_save_tick = HAL_GetTick();  /* arm debounced save */
  }
  if (Encoder_GetButton(&g_encoder)) {
    if (BandSel_IsOpen()) {
      uint8_t b = BandSel_Cursor();
      BandSel_Close();
      csdr_apply_band(b);
      g_sdr.display_dirty |= DIRTY_ALL;
      return;
    }
    if (ModeSel_IsOpen()) {
      uint8_t m = ModeSel_Cursor();
      ModeSel_Close();
      csdr_apply_mode_idx(m);
      g_sdr.display_dirty |= DIRTY_ALL;
      return;
    }
    if (Menu_IsOpen(&g_menu)) {
      /* Check if selected item is ACTION type (Calibration or SWR Scan) */
      { MenuItem_t *_cur = Menu_CurrentItem(&g_menu);
      if (_cur && _cur->type == MENU_TYPE_ACTION) {
        const char *name = _cur->label;
        Menu_Toggle(&g_menu);
        g_sdr.display_dirty |= DIRTY_ALL;
        if (strcmp(name, "Calibration") == 0) {
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
            .pa_watts        = g_sdr.pa_watts,
          };
          if (Cal_Run(&cp, &g_dsp)) {
            g_sdr.xtal_ppm        = cp.xtal_ppm;
            g_sdr.iq_gain         = cp.iq_gain;
            g_sdr.iq_phase        = cp.iq_phase;
            g_sdr.dc_i_offset     = cp.dc_i_offset;
            g_sdr.dc_q_offset     = cp.dc_q_offset;
            g_sdr.audio_gain_db   = cp.audio_gain_db;
            g_sdr.mic_gain        = cp.mic_gain;
            g_sdr.smeter_offset_db= cp.smeter_offset_db;
            g_sdr.lo_offset_hz    = cp.lo_offset_hz;
            g_sdr.pa_watts        = cp.pa_watts;
            g_sdr.pa_oc_limit_idx = cp.pa_oc_limit_idx;
            { float _lim = (float)cp.pa_oc_limit_idx * 0.1f;
              PA_OC_SetCurrentLimit(_lim);
              g_pa_cfg.current_warn_a = _lim * 0.75f;
              g_pa_cfg.current_trip_a = _lim * 0.90f; }
            DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
            DSP_SetDCOffset(&g_dsp, g_sdr.dc_i_offset, g_sdr.dc_q_offset);
            DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
            if (g_sdr.si5351_ok) {
              g_si5351.xtal_hz = (uint32_t)((int32_t)SI5351_XTAL_HZ +
                SI5351_XTAL_HZ / 1000000L * g_sdr.xtal_ppm);
              SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
            }
            csdr_save_settings();
          } else {
            /* Cancelled — restore live DSP state from saved settings */
            DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
            DSP_SetDCOffset(&g_dsp, g_sdr.dc_i_offset, g_sdr.dc_q_offset);
          }
        } else if (strcmp(name, "SWR Scan") == 0) {
          SWR_Scan_Run();
        } else if (strcmp(name, "Factory Reset") == 0) {
          csdr_factory_reset();
        }
        g_sdr.display_dirty |= DIRTY_ALL;
      } else {
        Menu_Select(&g_menu);
      }
      } /* end MenuItem_t *_cur block */
      return;
    }
    static const FreqStep_t step_cycle[] = {
      STEP_1, STEP_10, STEP_100, STEP_1K, STEP_10K, STEP_100K
    };
    uint8_t si = 0U;
    for (uint8_t i = 0U; i < 6U; i++) { if (step_cycle[i] == g_sdr.step) { si = i; break; } }
    g_sdr.step = step_cycle[(si + 1U) % 6U];
    g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
  }
  if (Encoder_GetLongPress(&g_encoder)) {
    /* Long press: cycle spectrum zoom ±24k → ±12k → ±6k → ±3k → ±24k */
    static const uint8_t c_zoom_decim[SPEC_ZOOM_COUNT] = {1U, 2U, 4U, 8U};
    uint8_t z = (uint8_t)((SDR_UI_GetSpecZoom() + 1U) % SPEC_ZOOM_COUNT);
    SDR_UI_SetSpecZoom(z);
    DSP_SetSpecDecim(&g_dsp, c_zoom_decim[z]);
  }
}

static void csdr_handle_keys(void)
{
  Key_Poll(&k_menu); Key_Poll(&k_f1);   Key_Poll(&k_f2); Key_Poll(&k_f3);
  Key_Poll(&k_f4);   Key_Poll(&k_band); Key_Poll(&k_mode); Key_Poll(&k_ptt);
  Key_Poll(&k_tune);

  if (Key_Press(&k_menu)) {
    g_sdr.display_dirty = 0U;  /* prevent status panel overwriting menu */
    if (!Menu_IsOpen(&g_menu))
      Menu_LoadFromSDR(&g_menu,
        g_sdr.agc_speed, g_sdr.nb_on, g_sdr.nr_mode, g_sdr.rit_hz,
        g_sdr.volume, (uint8_t)g_sdr.mic_gain, (uint8_t)g_sdr.digi_gain,
        g_sdr.squelch, (uint32_t)g_sdr.step, g_sdr.bw_hz,
        g_sdr.att_db, g_sdr.band_idx, (uint8_t)g_sdr.mode,
        g_sdr.usb_mode, SDR_UI_GetSpecZoom(),
        g_sdr.ext_alc_on, g_sdr.tx_power, g_sdr.pa_watts,
        g_sdr.tx_audio_low_hz, g_sdr.tx_audio_high_hz,
        g_sdr.if_shift_hz, g_sdr.notch_on, g_sdr.notch_hz,
        g_sdr.vox_on, g_sdr.vox_gain, g_sdr.vox_delay,
        g_sdr.cw_decode_on,
        g_sdr.cw_pitch_hz, g_sdr.cw_wpm,
        g_sdr.keyer_mode, g_sdr.paddle_reverse,
        g_sdr.sidetone_vol, g_sdr.cw_bkin,
        g_sdr.cw_bk_delay_ms, g_sdr.cw_reverse,
        g_sdr.cw_filter_hz,
        g_sdr.usb_iq_stream,
        g_sdr.tx_src,
        g_sdr.nb_level,
        g_sdr.nr_level,
        g_sdr.bc_mode,
        menu_apply_cb);
    Menu_Toggle(&g_menu);
    if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty |= DIRTY_ALL;
  }

  /* Band-sel overlay input — intercepts F1/F2/F3/F4 and band key */
  if (BandSel_IsOpen()) {
    if (Key_PressOrRepeat(&k_f1)) { BandSel_CursorUp();   BandSel_Render(); }
    if (Key_PressOrRepeat(&k_f2)) { BandSel_CursorDown(); BandSel_Render(); }
    if (Key_Press(&k_f3)) {
      uint8_t b = BandSel_Cursor();
      BandSel_Close();
      csdr_apply_band(b);
      g_sdr.display_dirty |= DIRTY_ALL;
    }
    if (Key_Press(&k_f4) || Key_Press(&k_band)) {
      BandSel_Close();
      g_sdr.display_dirty |= DIRTY_ALL;
    }
    return;  /* consume all remaining key processing this frame */
  }

  /* Mode-sel overlay input — intercepts F1/F2/F3/F4 and mode key */
  if (ModeSel_IsOpen()) {
    if (Key_PressOrRepeat(&k_f1)) { ModeSel_CursorUp();   ModeSel_Render(); }
    if (Key_PressOrRepeat(&k_f2)) { ModeSel_CursorDown(); ModeSel_Render(); }
    if (Key_Press(&k_f3)) {
      uint8_t m = ModeSel_Cursor();
      ModeSel_Close();
      csdr_apply_mode_idx(m);
      g_sdr.display_dirty |= DIRTY_ALL;
    }
    if (Key_Press(&k_f4) || Key_Press(&k_mode)) {
      ModeSel_Close();
      g_sdr.display_dirty |= DIRTY_ALL;
    }
    return;
  }

  /* F1: menu UP / Volume Down */
  if (Key_PressOrRepeat(&k_f1)) {
    if (Menu_IsOpen(&g_menu)) {
      Menu_Up(&g_menu);
    } else {
      uint8_t v = (g_sdr.volume >= 1U) ? (g_sdr.volume - 1U) : 0U;
      csdr_apply_volume(v);
    }
  }

  /* F2: menu DOWN / Volume Up (hold-repeat while held) */
  if (Key_PressOrRepeat(&k_f2)) {
    if (Menu_IsOpen(&g_menu)) Menu_Down(&g_menu);
    else {
      uint8_t v = (g_sdr.volume <= 99U) ? (g_sdr.volume + 1U) : 100U;
      csdr_apply_volume(v);
    }
  }

  /* F3: menu confirm / VFO A↔B swap (short) / CW decode toggle (hold, CW mode only)
   * Uses pending pattern: Press sets pending; Hold cancels pending and fires hold
   * action; Release fires swap only if pending (not consumed by hold). */
  { static bool s_f3_pending = false;
    if (Key_Press(&k_f3)) {
      if (Menu_IsOpen(&g_menu)) {
        MenuItem_t *_cur2 = Menu_CurrentItem(&g_menu);
        if (_cur2 && _cur2->type == MENU_TYPE_ACTION) {
          const char *name = _cur2->label;
          Menu_Toggle(&g_menu);
          g_sdr.display_dirty |= DIRTY_ALL;
          if (strcmp(name, "Calibration") == 0) {
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
              .pa_watts        = g_sdr.pa_watts,
            };
            if (Cal_Run(&cp, &g_dsp)) {
              g_sdr.xtal_ppm        = cp.xtal_ppm;
              g_sdr.iq_gain         = cp.iq_gain;
              g_sdr.iq_phase        = cp.iq_phase;
              g_sdr.dc_i_offset     = cp.dc_i_offset;
              g_sdr.dc_q_offset     = cp.dc_q_offset;
              g_sdr.audio_gain_db   = cp.audio_gain_db;
              g_sdr.mic_gain        = cp.mic_gain;
              g_sdr.smeter_offset_db= cp.smeter_offset_db;
              g_sdr.lo_offset_hz    = cp.lo_offset_hz;
              g_sdr.pa_watts        = cp.pa_watts;
              g_sdr.pa_oc_limit_idx = cp.pa_oc_limit_idx;
              { float _lim = (float)cp.pa_oc_limit_idx * 0.1f;
                PA_OC_SetCurrentLimit(_lim);
                g_pa_cfg.current_warn_a = _lim * 0.75f;
                g_pa_cfg.current_trip_a = _lim * 0.90f; }
              DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
              DSP_SetDCOffset(&g_dsp, g_sdr.dc_i_offset, g_sdr.dc_q_offset);
              DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
              if (g_sdr.si5351_ok) {
                g_si5351.xtal_hz = (uint32_t)((int32_t)SI5351_XTAL_HZ +
                  SI5351_XTAL_HZ / 1000000L * g_sdr.xtal_ppm);
                SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
              }
              csdr_save_settings();
            } else {
              DSP_SetIQCorr(&g_dsp, g_sdr.iq_gain, g_sdr.iq_phase);
              DSP_SetDCOffset(&g_dsp, g_sdr.dc_i_offset, g_sdr.dc_q_offset);
            }
          } else if (strcmp(name, "SWR Scan") == 0) {
            SWR_Scan_Run();
          } else if (strcmp(name, "Factory Reset") == 0) {
            csdr_factory_reset();
          }
          g_sdr.display_dirty |= DIRTY_ALL;
        } else {
          Menu_Confirm(&g_menu);
        }
      } else {
        s_f3_pending = true;  /* arm — wait for release or hold */
      }
    }
    if (!Menu_IsOpen(&g_menu) && Key_Hold(&k_f3)) {
      s_f3_pending = false;   /* hold consumed — cancel swap */
      if (g_sdr.mode == MODE_CW) {
        g_sdr.cw_decode_on = !g_sdr.cw_decode_on;
        SDR_UI_SetCWDecActive(g_sdr.cw_decode_on);
        if (!g_sdr.cw_decode_on) SDR_UI_ClearCWText();
        CWDec_Reset(&g_cw_dec);
        g_sdr.display_dirty |= DIRTY_ALL; /* refresh INFO: hints or [DEC] placeholder */
      }
    }
    if (!Menu_IsOpen(&g_menu) && Key_Release(&k_f3)) {
      if (s_f3_pending) { csdr_vfo_swap(); }
      s_f3_pending = false;
    }
  }

  /* F4: Back / Exit menu  –or–  quick SWR scan */
  if (Key_Press(&k_f4)) {
    if (Menu_IsOpen(&g_menu)) {
      Menu_Back(&g_menu);
      if (!Menu_IsOpen(&g_menu)) g_sdr.display_dirty |= DIRTY_ALL;
    } else {
      SWR_Scan_Run();
      g_sdr.display_dirty |= DIRTY_ALL;
    }
  }

  /* TUNE: dedicated momentary push-to-tune button on its own PCA9555 line
   * (PCA_BIT_TUNE) — continuous low-power carrier for as long as it's held.
   * See csdr_tune_start/csdr_tune_stop. Ignored while the menu is open. */
  if (!Menu_IsOpen(&g_menu)) {
    if (Key_Press(&k_tune))   { csdr_tune_start(); }
    if (Key_Release(&k_tune)) { csdr_tune_stop();  }
  }

  /* TUNE safety auto-stop: in case the button sticks or input glitches and
   * Key_Release never fires, force-stop after TUNE_MAX_MS regardless. */
  if (g_sdr.tune_mode && (HAL_GetTick() - s_tune_start_ms) >= TUNE_MAX_MS) {
    csdr_tune_stop();
  }


  /* BAND key: short press = BandUp on release; long hold = open overlay.
   * Key_Press fires ~20ms into a press; Key_Hold fires at 600ms.
   * Using pending flag so BandUp only applies if no hold fired (short press). */
  { static bool s_band_pending = false;
    if (Key_Press(&k_band))   { s_band_pending = true; }
    if (Key_Hold(&k_band))    { s_band_pending = false;
                                 g_sdr.display_dirty = 0U;
                                 BandSel_Open(g_sdr.band_idx, g_sdr.band_idx);
                                 BandSel_Render(); }
    if (Key_Release(&k_band)) { if (s_band_pending) {
                                   csdr_apply_band(BPF_BandUp(g_sdr.band_idx)); }
                                 s_band_pending = false; }
  }

  /* MODE key: short press = ModeUp on release; long hold = cancel.
   * Same pending-flag pattern as BAND key: action fires on release only
   * if no hold fired, preventing accidental mode change when holding. */
  { static bool s_mode_pending = false;
    if (Key_Press(&k_mode))   { s_mode_pending = true; }
    if (Key_Hold(&k_mode))    { s_mode_pending = false;
                                 g_sdr.display_dirty = 0U;
                                 ModeSel_Open(g_sdr.mode, g_sdr.mode);
                                 ModeSel_Render(); }
    if (Key_Release(&k_mode)) {
      if (s_mode_pending) {
        csdr_apply_mode_idx((uint8_t)((g_sdr.mode + 1U) % MODE_COUNT));
      }
      s_mode_pending = false;
    }
  }

  if (Key_Press(&k_ptt)) {
    g_sdr.tx_mode = true;
    g_sdr.display_dirty |= (uint8_t)(DIRTY_HDR | DIRTY_VFO | DIRTY_SBR);
    csdr_apply_tx();
  } else if (Key_Release(&k_ptt)) {
    g_sdr.tx_mode = false;
    g_sdr.display_dirty |= (uint8_t)DIRTY_ALL;
    csdr_apply_tx();
  }
}

static void csdr_update_tx_spectrum(void)
{
  if (!g_sdr.tx_mode || !g_dsp.fft_ready) return;
  g_dsp.fft_ready = false;
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_SPECTRUM);
  SDR_UI_DrawTXSpectrum(g_dsp.fft_mag_db, DSP_FFT_SIZE,
                        (uint8_t)g_sdr.mode, g_dsp.sample_rate);
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_SPECTRUM);
}

static void csdr_update_spectrum(void)
{
  if (g_sdr.tx_mode || !g_dsp.fft_ready || Menu_IsOpen(&g_menu) || BandSel_IsOpen() || ModeSel_IsOpen()) return;

  /* Race-window guard: do_trip() sets tx_mode=false immediately but
   * csdr_apply_tx() (which calls SetTXMode(false)) runs up to 10 ms later
   * via cat_tx_dirty.  If the SPEC zone is still in TX-blanked state while
   * we are already in RX mode, force SetTXMode(false) now so that the delta-
   * skip cache is cleared and the full RX spectrum is drawn rather than
   * comparing new peaks against stale pre-TX reference values. */
  if (SDR_UI_IsTXZoneBlanked()) {
    SDR_UI_SetTXMode(false);   /* idempotent if already called */
  }

  g_dsp.fft_ready = false;

  float bw_lo_ratio = 0.0f, bw_hi_ratio = 0.0f;
  if (g_dsp.sample_rate > 0U && g_sdr.bw_hz > 0U) {
    float full = (float)g_sdr.bw_hz / (float)g_dsp.sample_rate;
    float half = full * 0.5f;
    switch (g_sdr.mode) {
      case MODE_LSB:
      case MODE_DIGL: bw_lo_ratio = full; bw_hi_ratio = 0.0f; break;
      case MODE_USB:
      case MODE_DIGU:
      case MODE_CW:
      case MODE_FREEDV: bw_lo_ratio = 0.0f; bw_hi_ratio = full; break;
      default:        bw_lo_ratio = half; bw_hi_ratio = half;  break;
    }
  }
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_SPECTRUM);
  SDR_UI_DrawSpectrum(g_dsp.fft_mag_db, DSP_FFT_SIZE, bw_lo_ratio, bw_hi_ratio, NULL);
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_SPECTRUM);
}

static void csdr_update_waterfall(void)
{
  if (g_sdr.tx_mode || Menu_IsOpen(&g_menu) || BandSel_IsOpen() || ModeSel_IsOpen()) return;
  RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_WATERFALL);
  g_dsp.wf_lines = 0U;                   /* consume pending lines; render latest FFT frame */
  SDR_UI_DrawWaterfall(g_dsp.fft_mag_db, DSP_FFT_SIZE);
  RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_WATERFALL);
}

static void csdr_refresh_display(void)
{
  bool menu_open = Menu_IsOpen(&g_menu);

  /* Clock: force DIRTY_HDR once per second so HH:MM:SS ticks live */
  if (!menu_open) {
    static uint32_t s_clk_sec = UINT32_MAX;
    uint32_t sec = HAL_GetTick() / 1000U;
    if (sec != s_clk_sec) {
      s_clk_sec = sec;
      g_sdr.display_dirty |= DIRTY_HDR;
    }
  }

  uint8_t dirty = g_sdr.display_dirty;
  if (dirty != 0U) {
    g_sdr.display_dirty = 0U;
    SDR_UI_State_t ui = {0};
    ui.freq_hz   = g_sdr.freq_hz;       ui.mode      = (uint8_t)g_sdr.mode;
    ui.band_idx  = g_sdr.band_idx;      ui.volume    = g_sdr.volume;
    ui.squelch   = g_sdr.squelch;       ui.step      = (uint32_t)g_sdr.step;
    ui.agc_speed = g_sdr.agc_speed;     ui.nb_on     = g_sdr.nb_on;
    ui.nr_on     = (g_sdr.nr_mode > 0U); ui.rit_hz    = g_sdr.rit_hz;
    ui.tx_mode   = g_sdr.tx_mode;
    ui.si5351_ok = g_sdr.si5351_ok;
    ui.signal_db = g_dsp.signal_power_db
                 + (float)g_att.current_atten_x2 * 0.5f  /* antenna-referenced: add back front-end att */
                 + (float)g_sdr.smeter_offset_db
                 + (float)(g_band_cal[g_sdr.band_idx].rx_gain_trim
                          + g_band_cal[g_sdr.band_idx].noise_floor_off);
    ui.bw_hz     = g_sdr.bw_hz;         ui.voltage_x10 = (int16_t)(g_analog.voltage_mv / 100);
    ui.att_db    = g_sdr.att_db;
    ui.att_x2    = g_att.current_atten_x2;   /* 0.5 dB precision for sidebar display */
    ui.rf_agc_on = g_sdr.rf_agc_on;
    ui.mic_gain  = (g_sdr.mode == MODE_DIGU || g_sdr.mode == MODE_DIGL ||
                    g_sdr.mode == MODE_FREEDV)
                   ? g_sdr.digi_gain : g_sdr.mic_gain;
    ui.tx_power  = g_sdr.tx_power;
    ui.pa_watts  = g_sdr.pa_watts;
    ui.fwd_power_mw = PA_Protect_GetFwdPowerEnvelope_mW();
    ui.freq_b_hz = g_sdr.vfo_b.freq_hz; /* inactive VFO shown in sub-line */
    ui.active_vfo = g_sdr.active_vfo;

    if (BandSel_IsOpen()) {
      BandSel_Render();
    } else if (ModeSel_IsOpen()) {
      ModeSel_Render();
    } else if (menu_open) {
      /* Menu đang mở: re-render để đảm bảo không bị xóa */
      Menu_Render(&g_menu);
    } else if (g_sdr.tx_mode) {
      /* TX: redraw Header, VFO (PW badge lives here), and sidebar */
      if (dirty & DIRTY_HDR) {
        RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
        SDR_UI_DrawHeader(&ui);
        RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
      }
      if (dirty & DIRTY_VFO) {
        SDR_UI_SetFooterFreq(g_sdr.freq_hz, (uint32_t)g_sdr.step);
        SDR_UI_DrawVFO(&ui);
      }
      if (dirty & DIRTY_SBR) {
        SDR_UI_DrawSidebarRight(&ui);
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
        SDR_UI_SetFooterFreq(g_sdr.freq_hz, (uint32_t)g_sdr.step);
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
  } else if (!menu_open && !BandSel_IsOpen() && !ModeSel_IsOpen()) {
    if (g_sdr.tx_mode) {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
      SDR_UI_UpdateTXMeters((int32_t)g_analog.alc_percent,
                            (int32_t)(g_analog.swr_x100 / 10));
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
    } else {
      RuntimeDiag_UiSectionBegin(RUNTIME_DIAG_UI_VOLUME_MODE);
      SDR_UI_UpdateSMeter(g_dsp.signal_power_db
                        + (float)g_att.current_atten_x2 * 0.5f
                        + (float)g_sdr.smeter_offset_db
                        + (float)(g_band_cal[g_sdr.band_idx].rx_gain_trim
                                 + g_band_cal[g_sdr.band_idx].noise_floor_off));
      RuntimeDiag_UiSectionEnd(RUNTIME_DIAG_UI_VOLUME_MODE);
    }
  }

  /* PA warning: draw immediately after every refresh so it survives
   * dirty redraws (including the DIRTY_ALL triggered by PA TRIP). */
  SDR_UI_UpdatePAWarn(PA_Protect_GetState(), PA_Protect_GetFault());
}

static uint32_t default_bw_for_mode(SDR_Mode_t m)
{
  switch (m) {
    case MODE_AM:   return 6000U;
    case MODE_FM:   return 15000U;
    case MODE_USB:
    case MODE_LSB:
    case MODE_DIGU:
    case MODE_DIGL:   return 3000U;
    case MODE_FREEDV: return 4000U;
    case MODE_CW:     return 500U;
    default:          return 4000U;
  }
}

static void menu_apply_cb(void)
{
  uint8_t agc_speed; bool nb, ext_alc, notch_en, vox_en, cw_dec, paddle_rev, cw_rev, iq_stream; int16_t rit, rxshift, notch_f;
  uint8_t vol, mic, digi, sq, att, band, mode, usb, zoom, rfpwr, vox_gain, tx_src_new; uint32_t step, bw;
  uint16_t tx_low, tx_high, vox_delay;
  uint16_t cw_pitch, cw_filter, cw_bk_delay; uint8_t cw_wpm, keyer_mode, sidetone, cw_bkin;
  uint8_t nb_level, nr_level, nr, bc;
  Menu_SaveToSDR(&g_menu, &agc_speed, &nb, &nr, &rit,
                  &vol, &mic, &digi, &sq, &step, &bw, &att, &band, &mode, &usb, &zoom,
                  &ext_alc, &rfpwr, &tx_low, &tx_high, &rxshift, &notch_en, &notch_f,
                  &vox_en, &vox_gain, &vox_delay, &cw_dec,
                  &cw_pitch, &cw_wpm, &keyer_mode, &paddle_rev,
                  &sidetone, &cw_bkin, &cw_bk_delay, &cw_rev, &cw_filter, &iq_stream,
                  &tx_src_new, &nb_level, &nr_level, &bc);
  if (tx_low != g_sdr.tx_audio_low_hz || tx_high != g_sdr.tx_audio_high_hz) {
    g_sdr.tx_audio_low_hz  = tx_low;
    g_sdr.tx_audio_high_hz = tx_high;
    DSP_SetTxPassband(&g_dsp, (float)tx_low, (float)tx_high);
  }
  if (rxshift != g_sdr.if_shift_hz) {
    g_sdr.if_shift_hz = rxshift;
    csdr_apply_nco_if();
  }
  if (notch_en != g_sdr.notch_on || notch_f != g_sdr.notch_hz) {
    g_sdr.notch_on = notch_en;
    g_sdr.notch_hz = notch_f;
    DSP_SetNotch(&g_dsp, notch_en, (float)notch_f);
  }
  g_sdr.vox_on    = vox_en;
  g_sdr.vox_gain  = vox_gain;
  g_sdr.vox_delay = vox_delay;
  g_sdr.ext_alc_on = ext_alc;
  if (rfpwr != g_sdr.tx_power) {
    g_sdr.tx_power = rfpwr;
    g_sdr.display_dirty |= DIRTY_SBR;
    if (g_sdr.tx_mode) g_sdr.cat_tx_dirty = true;
  }
  g_sdr.mic_gain  = (int16_t)mic;
  g_sdr.digi_gain = (int16_t)digi;
  if (agc_speed != g_sdr.agc_speed)
    AGC_SetMode(&g_dsp.agc, g_sdr.mode, agc_speed, CSDR_AUDIO_SAMPLE_RATE);
  g_sdr.agc_speed = agc_speed; g_sdr.nb_on = nb; g_sdr.nr_mode = nr;
  g_sdr.nb_level  = nb_level;
  g_sdr.nr_level  = nr_level;
  g_sdr.bc_mode   = bc;
  DSP_NB_Set(&g_dsp, nb, nb_level);
  DSP_NR_Set(&g_dsp, nr, nr_level);
  DSP_BC_Set(&g_dsp, bc);
  g_sdr.rit_hz = rit;
  g_sdr.cat_rit_dirty = true;  /* apply new RIT offset to nco_if via CSDR_Loop */
  if (vol != g_sdr.volume) csdr_apply_volume(vol);
  g_sdr.squelch = sq; DSP_SetSquelch(&g_dsp, sq);
  g_sdr.step = (FreqStep_t)step;
  if (att != g_sdr.att_db) {
    PE4302_SetAttn_dB(&g_att, att);
    g_sdr.att_db = att;
    RFAGC_NotifyManual(&g_rfagc, g_att.current_atten_x2);  /* freeze RF AGC cooldown */
  }
  if (band != g_sdr.band_idx) csdr_apply_band(band);
  if (mode != (uint8_t)g_sdr.mode) {
    SDR_Mode_t old_mode_menu = g_sdr.mode;
    g_sdr.mode  = (SDR_Mode_t)mode;
    g_sdr.bw_hz = default_bw_for_mode(g_sdr.mode);
    g_sdr.sl_hz = 0U;
    DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
    DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
    AGC_SetMode(&g_dsp.agc, g_sdr.mode, g_sdr.agc_speed, CSDR_AUDIO_SAMPLE_RATE);
    csdr_apply_nco_if();
    csdr_on_mode_changed(old_mode_menu);
  } else if (g_sdr.mode != MODE_CW && bw != g_sdr.bw_hz) {
    g_sdr.bw_hz = bw;
    DSP_SetBW(&g_dsp, (float)bw);
    g_sdr.display_dirty |= DIRTY_SBR;
  }
  g_sdr.usb_mode      = usb;
  g_sdr.usb_iq_stream = iq_stream;
  if (tx_src_new != g_sdr.tx_src) {
    g_sdr.tx_src = tx_src_new;
    /* Nếu đang RX: cập nhật WM8731 input source ngay.
     * Nếu đang TX: csdr_apply_tx sẽ apply khi TX kết thúc hoặc bắt đầu lại. */
    if (!g_sdr.tx_mode)
      WM8731_SetInputSource(&hi2c1, WM8731_I2C_ADDR, false); /* về LINE IN khi RX */
  }
  if (zoom != SDR_UI_GetSpecZoom()) {
    static const uint8_t c_zoom_decim[SPEC_ZOOM_COUNT] = {1U, 2U, 4U, 8U};
    SDR_UI_SetSpecZoom(zoom);
    DSP_SetSpecDecim(&g_dsp, c_zoom_decim[zoom]);
  }
  if (cw_dec != g_sdr.cw_decode_on) {
    /* CW decode can only be enabled when in CW mode */
    bool effective = cw_dec && (g_sdr.mode == MODE_CW);
    g_sdr.cw_decode_on = effective;
    SDR_UI_SetCWDecActive(effective);
    if (!effective) SDR_UI_ClearCWText();
  }
  /* CW settings apply */
  if (cw_pitch != g_sdr.cw_pitch_hz) {
    g_sdr.cw_pitch_hz = cw_pitch;
    DSP_SetCWPitch(&g_dsp, cw_pitch, CSDR_AUDIO_SAMPLE_RATE);
  }
  if (cw_rev != g_sdr.cw_reverse) {
    g_sdr.cw_reverse = cw_rev;
    DSP_SetCWReverse(&g_dsp, cw_rev);
  }
  if (sidetone != g_sdr.sidetone_vol) {
    g_sdr.sidetone_vol = sidetone;
    DSP_SetSidetoneVol(&g_dsp, sidetone);
  }
  if (cw_filter != g_sdr.cw_filter_hz) {
    g_sdr.cw_filter_hz = cw_filter;
    if (g_sdr.mode == MODE_CW) DSP_SetBW(&g_dsp, (float)cw_filter);
  }
  if (cw_wpm != g_sdr.cw_wpm) {
    g_sdr.cw_wpm    = cw_wpm;
    g_cw_dec.dit_ms = 1200U / (uint32_t)cw_wpm;
    CWKeyer_SetWPM(&g_cw_keyer, cw_wpm);
  }
  g_sdr.keyer_mode      = keyer_mode;
  g_sdr.paddle_reverse  = paddle_rev;
  g_sdr.cw_bkin         = cw_bkin;
  g_sdr.cw_bk_delay_ms  = cw_bk_delay;
  g_cw_keyer.mode          = (CWKeyerMode_t)keyer_mode;
  g_cw_keyer.paddle_reverse= paddle_rev;
  g_cw_keyer.bkin          = (CWBkin_t)cw_bkin;
  g_cw_keyer.bk_delay_ms   = cw_bk_delay;
  g_sdr.display_dirty |= DIRTY_ALL;
  csdr_save_settings();
}

/* ══════════════════════════════════════════════════════════
 *  VOX (Voice-Operated eXchange)
 * ══════════════════════════════════════════════════════════ */
static bool     s_vox_active         = false;
static uint32_t s_vox_hang_remaining = 0U;

static float csdr_vox_peek_level(void)
{
  uint16_t avail = g_usb_audio.tx_count;
  if (avail < 4U) return 0.0f;
  uint32_t n_samp = avail / 4U;
  if (n_samp > 48U) n_samp = 48U;
  float peak = 0.0f;
  uint16_t rd = g_usb_audio.tx_rd;
  for (uint32_t i = 0U; i < n_samp; i++) {
    uint16_t pos = (uint16_t)((rd + i * 4U) % USB_AUDIO_RING_SIZE);
    uint16_t pos1 = (uint16_t)((pos + 1U) % USB_AUDIO_RING_SIZE);
    int16_t s = (int16_t)((uint16_t)g_usb_audio.tx_ring[pos]
                         | ((uint16_t)g_usb_audio.tx_ring[pos1] << 8U));
    float fs = (s < 0) ? -(float)s : (float)s;
    if (fs > peak) peak = fs;
  }
  return peak;
}

static void csdr_vox_poll(void)
{
  if (!g_sdr.vox_on) { s_vox_active = false; s_vox_hang_remaining = 0U; return; }
  float peak = csdr_vox_peek_level();
  float t = 1.0f - (float)g_sdr.vox_gain / 100.0f;
  float threshold = t * t * t * 32767.0f;
  if (peak >= threshold) {
    s_vox_hang_remaining = (uint32_t)g_sdr.vox_delay;
    if (!s_vox_active && !g_sdr.tx_mode) {
      s_vox_active = true;
      g_sdr.tx_mode = true;
      g_sdr.cat_tx_dirty = true;
    }
  } else if (s_vox_active) {
    if (s_vox_hang_remaining > 20U) {
      s_vox_hang_remaining -= 20U;
    } else {
      s_vox_hang_remaining = 0U;
      s_vox_active = false;
      g_sdr.tx_mode = false;
      g_sdr.cat_tx_dirty = true;
    }
  }
}

/* ══════════════════════════════════════════════════════════
 *  Dual-VFO helpers
 * ══════════════════════════════════════════════════════════ */

/* Recompute and apply DSP nco_if from current g_sdr state.
 * Call whenever mode, sl_hz, if_shift_hz, or rit changes via a direct (non-deferred)
 * path — e.g. physical key presses, VFO swap. CAT-deferred path uses cat_rit_dirty. */
static void csdr_apply_nco_if(void)
{
  int32_t sl_sign = 0;
  if (g_sdr.mode == MODE_USB || g_sdr.mode == MODE_CW ||
      g_sdr.mode == MODE_DIGU || g_sdr.mode == MODE_FREEDV)
      sl_sign = +1;
  else if (g_sdr.mode == MODE_LSB || g_sdr.mode == MODE_DIGL)
      sl_sign = -1;
  int32_t eff_if = (int32_t)g_sdr.if_shift_hz
                 + (g_cat.rit_on ? (int32_t)g_sdr.rit_hz : 0)
                 + sl_sign * (int32_t)g_sdr.sl_hz;
  DSP_SetIFShift(&g_dsp, eff_if, CSDR_AUDIO_SAMPLE_RATE);
}

/* Swap active ↔ inactive VFO and apply new active state to hardware */
static void csdr_vfo_swap(void)
{
  VFO_State_t tmp = {
    .freq_hz      = g_sdr.freq_hz,
    .mode         = g_sdr.mode,
    .band_idx     = g_sdr.band_idx,
    .step         = g_sdr.step,
    .rit_hz       = g_sdr.rit_hz,
    .bw_hz        = g_sdr.bw_hz,
    .sl_hz        = g_sdr.sl_hz,
    .if_shift_hz  = g_sdr.if_shift_hz,
  };

  g_sdr.freq_hz     = g_sdr.vfo_b.freq_hz;
  g_sdr.mode        = g_sdr.vfo_b.mode;
  g_sdr.band_idx    = g_sdr.vfo_b.band_idx;
  g_sdr.step        = g_sdr.vfo_b.step;
  g_sdr.rit_hz      = g_sdr.vfo_b.rit_hz;
  g_sdr.bw_hz       = g_sdr.vfo_b.bw_hz;
  g_sdr.sl_hz       = g_sdr.vfo_b.sl_hz;
  g_sdr.if_shift_hz = g_sdr.vfo_b.if_shift_hz;

  g_sdr.vfo_b    = tmp;
  g_sdr.active_vfo ^= 1U;
  /* Keep CAT routing state in sync so VS; query and AI IF-frame reflect reality */
  g_cat.active_vfo = g_sdr.active_vfo;

  BPF_SetBand(g_sdr.band_idx);
  LPF_SetBand(g_sdr.band_idx);
  DSP_SetMode(&g_dsp, g_sdr.mode, CSDR_AUDIO_SAMPLE_RATE);
  DSP_SetBW(&g_dsp, (float)g_sdr.bw_hz);
  AGC_SetMode(&g_dsp.agc, g_sdr.mode, g_sdr.agc_speed, CSDR_AUDIO_SAMPLE_RATE);
  csdr_apply_nco_if();
  DSP_SetFrequency(&g_dsp, g_sdr.lo_offset_hz, CSDR_AUDIO_SAMPLE_RATE);
  if (g_sdr.si5351_ok) SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
  g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
}

/* ── VFO-B + active-VFO CAT callbacks ──────────────────────────────────────
 * These keep g_sdr.vfo_b and g_sdr.active_vfo in sync with CAT commands
 * so UI state and Hamlib/flrig/WSJT-X always agree.
 * ─────────────────────────────────────────────────────────────────────────*/
static void     cat_set_vfo_b_freq(uint32_t hz) { g_sdr.vfo_b.freq_hz = hz; g_sdr.display_dirty |= DIRTY_VFO; }
static void     cat_set_vfo_b_mode(uint8_t m)
{
    g_sdr.vfo_b.mode  = (SDR_Mode_t)m;
    g_sdr.vfo_b.bw_hz = default_bw_for_mode((SDR_Mode_t)m);  /* reset stale BW from prior mode */
    g_sdr.vfo_b.sl_hz = 0U;                                   /* reset low-cut for new mode */
    g_sdr.display_dirty |= DIRTY_VFO;
}
static void     cat_set_vfo_b_bw(uint32_t hz)      { g_sdr.vfo_b.bw_hz = hz;    g_sdr.display_dirty |= DIRTY_VFO; }
static void     cat_set_vfo_b_lo_cut(uint32_t hz)  { g_sdr.vfo_b.sl_hz = hz;    g_sdr.display_dirty |= DIRTY_VFO; }
static uint32_t cat_get_vfo_b_freq(void)           { return g_sdr.vfo_b.freq_hz; }
static uint8_t  cat_get_vfo_b_mode(void)           { return (uint8_t)g_sdr.vfo_b.mode; }
static uint32_t cat_get_vfo_b_bw(void)             { return g_sdr.vfo_b.bw_hz; }
static uint32_t cat_get_vfo_b_lo_cut(void)         { return g_sdr.vfo_b.sl_hz; }

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
    .freq_hz      = g_sdr.freq_hz,
    .mode         = g_sdr.mode,
    .band_idx     = g_sdr.band_idx,
    .step         = g_sdr.step,
    .rit_hz       = g_sdr.rit_hz,
    .bw_hz        = g_sdr.bw_hz,
    .sl_hz        = g_sdr.sl_hz,
    .if_shift_hz  = g_sdr.if_shift_hz,
  };
  g_sdr.freq_hz     = g_sdr.vfo_b.freq_hz;
  g_sdr.mode        = g_sdr.vfo_b.mode;
  g_sdr.band_idx    = g_sdr.vfo_b.band_idx;
  g_sdr.step        = g_sdr.vfo_b.step;
  g_sdr.rit_hz      = g_sdr.vfo_b.rit_hz;
  g_sdr.bw_hz       = g_sdr.vfo_b.bw_hz;
  g_sdr.sl_hz       = g_sdr.vfo_b.sl_hz;
  g_sdr.if_shift_hz = g_sdr.vfo_b.if_shift_hz;
  g_sdr.vfo_b       = tmp;
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
  s_freq_save_tick = HAL_GetTick();  /* arm debounced save */
}
static void     cat_set_mode(uint8_t m)
{
  if (g_sdr.cat_mode_dirty) dbg_cat_blocked_updates++;
  SDR_Mode_t new_mode = (SDR_Mode_t)m;
  /* bw_hz: reset to default when mode changes OR when the new mode is NOT AM/FM.
   *
   * Asymmetric rule justified by two conflicting requirements:
   *   AM/FM: flrig polls MD5;/MD4; every cycle.  Resetting bw_hz on every poll
   *     would undo any custom bandwidth the user had set → only reset on actual
   *     mode change.
   *   USB/LSB/CW/DIGI: WSJT-X / Hamlib expects a predictable default passband when
   *     it asserts the mode.  A stale bw_hz (e.g. 2200 Hz from a prior flrig session)
   *     makes Hamlib report an unexpected width → WSJT-X triggers extra code paths
   *     that expose the Hamlib VFO_NONE bug (kenwood_set_freq: unsupported VFO None).
   *     Resetting to 3000 Hz on every MD2; is safe — it is what WSJT-X would set. */
  bool mode_changed = (new_mode != g_sdr.mode);
  bool is_am_fm     = (new_mode == MODE_AM || new_mode == MODE_FM);
  if (mode_changed || !is_am_fm) {
    g_sdr.bw_hz = default_bw_for_mode(new_mode);
  }
  /* sl_hz unconditional: stale lo-cut shifts eff_if and breaks FT8 decode. */
  g_sdr.sl_hz = 0U;
  g_sdr.mode = new_mode;
  g_sdr.cat_mode_dirty = true; /* DSP mode + BW applied by CSDR_Loop */
  g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
}
/* CAT callback — deferred: sets dirty flag, hardware applied by CSDR_Loop.
 * Physical PTT key goes through csdr_apply_tx() for immediate switching. */
static void cat_set_tx(bool tx)
{
  if (tx == g_sdr.tx_mode) return;
  /* RX; while TUNE carrier active (AC111 or physical key): drop the tune
   * state too — leaving tune_active/cw_key_out set would key a stray CW
   * carrier on the next PTT TX. */
  if (!tx && g_sdr.tune_mode) {
    g_sdr.tune_mode      = false;
    g_dsp.tx.tune_active = false;
    g_dsp.cw_key_out     = false;
  }
  if (g_sdr.cat_tx_dirty) dbg_cat_blocked_updates++;
  g_sdr.tx_mode    = tx;
  g_sdr.cat_tx_dirty = true;
  g_sdr.display_dirty |= tx ? (uint8_t)(DIRTY_HDR | DIRTY_VFO | DIRTY_SBR) : (uint8_t)DIRTY_ALL;
}
static void     cat_set_att(uint8_t lv)
{
  if (g_sdr.cat_att_dirty) dbg_cat_blocked_updates++;
  static const uint8_t m[] = {0, 6, 12, 18};
  g_sdr.att_db = (lv < 4U) ? m[lv] : 0U;
  g_sdr.cat_att_dirty = true; /* PE4302 applied by CSDR_Loop */
  g_sdr.display_dirty |= DIRTY_HDR;
}

/* Immediate volume apply — WM8731 LHPVOL updated via SAI stop/write/restart.
 * WM8731 NACKs all I2C when MCLK is active; must stop SAI to write LHPVOL.
 * Matches main-branch architecture: hardware LHPVOL controls volume, no rx_volume_scale. */
static void csdr_apply_volume(uint8_t vol)
{
  if (vol > 100U) vol = 100U;
  g_sdr.volume = vol;

  /* Soft-mute DSP immediately so SAI TX is silent during the stop/start gap */
  g_dsp.rx_volume_scale = 0.0f;

  /* Stop SAI — kills MCLK, WM8731 now accepts I2C */
  HAL_SAI_DMAStop(&hsai_BlockA1);
  HAL_SAI_DMAStop(&hsai_BlockB1);
  HAL_Delay(5U);

  /* Full WM8731_Init (not just SetVolume) — ensures BYPASS=0 / DACSEL=1 are
   * restored after SAI restart in case WM8731 reset its ANALOG_PATH register
   * when MCLK was re-applied.  Volume is embedded in the config struct. */
  uint8_t wm_vol = (vol == 0U) ? 0x2FU
                                : (uint8_t)(90U + ((uint16_t)vol * 31U / 100U));
  { WM8731_Config_t wm_v = {
      .hi2c = &hi2c1, .i2c_addr = WM8731_I2C_ADDR,
      .sample_rate = CSDR_AUDIO_SAMPLE_RATE,
      .input_volume = 23U, .output_volume = wm_vol, .line_in = true
    };
    WM8731_Init(&wm_v);
  }

  /* Restart SAI — TX master first, RX slave after */
  RuntimeDiag_TxHalfFilled(0U);
  RuntimeDiag_TxHalfFilled(1U);
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)s_tx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);
  HAL_Delay(10U);
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)s_rx_buf,
      CSDR_AUDIO_BUF_TOTAL * 2U);

  /* Restore DSP scale: 1.0 for vol>0 (LHPVOL handles attenuation), 0 for mute */
  g_dsp.rx_volume_scale = (vol == 0U) ? 0.0f : 1.0f;

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
  /* pa_watts=0 means no PA fitted; BPF and T/R relay must not switch.
   * All other features (VOX, CW keyer, TX UI, mic spectrum) run normally. */
  bool pa_fitted = (g_sdr.pa_watts != 0U);

  /* PA protection gate removed: TRIP/COOLDOWN zeroes drive via
   * PA_Protect_GetDriveLimit()=0 in the gain block below.
   * tx_mode stays true so TX UI (spectrum/meters) keeps running. */

  if (g_sdr.tx_mode) {
    PA_Protect_OnTxStart();
    /* Nạp ngưỡng INA226: base từ cài đặt, CW/DIGI thấp hơn 0.5/0.3A */
    { float base = (float)g_sdr.pa_oc_limit_idx * 0.1f;
      float lim  = (g_sdr.mode == MODE_CW)                                ? base - 0.5f :
                   (g_sdr.mode == MODE_DIGU || g_sdr.mode == MODE_DIGL ||
                    g_sdr.mode == MODE_FREEDV)                           ? base - 0.3f : base;
      if (lim < 1.0f) lim = 1.0f;
      PA_OC_SetCurrentLimit(lim); }
    /* RX → TX: switch WM8731 input nếu tx_src==MIC, mute HP và shutdown amp (non-CW). */
    bool use_mic_tx = (g_sdr.tx_src == 1U) && (g_sdr.mode != MODE_CW);
    WM8731_SetInputSource(&hi2c1, WM8731_I2C_ADDR, use_mic_tx);
    g_dsp.mic_buf = use_mic_tx ? s_rx_buf : NULL;
    if (g_sdr.mode != MODE_CW) {
      WM8731_SetMute(&hi2c1, WM8731_I2C_ADDR, true);
      HAL_GPIO_WritePin(AUDIO_SD_GPIO_Port, AUDIO_SD_Pin, GPIO_PIN_RESET); /* amp shutdown */
    }
    if (pa_fitted) {
      BPF_SetMode(RF_MODE_TX);
      /* Split: retune LO to TX VFO (inactive slot) before gating RF */
      if (g_cat.split_on && g_sdr.si5351_ok)
        SI5351_SetQSDFrequency(&g_si5351, g_sdr.vfo_b.freq_hz + g_sdr.lo_offset_hz);
      HAL_GPIO_WritePin(T_R_SW_GPIO_Port, T_R_SW_Pin, GPIO_PIN_SET);
    }
  } else {
    PA_Protect_OnTxStop();
    if (pa_fitted) {
      /* TX → RX: open T/R relay first, then switch BPF bank back, then unmute codec. */
      HAL_GPIO_WritePin(T_R_SW_GPIO_Port, T_R_SW_Pin, GPIO_PIN_RESET);
      BPF_SetMode(RF_MODE_RX);
      /* Split: restore LO to RX VFO (active slot) */
      if (g_cat.split_on && g_sdr.si5351_ok)
        SI5351_SetQSDFrequency(&g_si5351, g_sdr.freq_hz + g_sdr.lo_offset_hz);
    }
    g_dsp.mic_buf = NULL;  /* TX → RX: DSP_ProcessTX sẽ không được gọi, clear cho an toàn */
    HAL_GPIO_WritePin(AUDIO_SD_GPIO_Port, AUDIO_SD_Pin, GPIO_PIN_SET); /* amp enable */
    WM8731_SetMute(&hi2c1, WM8731_I2C_ADDR, false);
    WM8731_SetInputSource(&hi2c1, WM8731_I2C_ADDR, false); /* TX → RX: quay về LINE IN (QSD) */
  }
  /* Select gain source: digi_gain for digital modes, mic_gain for voice.
   * CW/TUNE use a fixed 100% base instead — carrier amplitude is governed by
   * sidetone volume (cw_sidetone_amp), not mic_gain, so mic_gain must not
   * couple into CW/TUNE drive.
   * Scale by tx_power (0-100%, clamped to TUNE_POWER_PCT while tuning), PA
   * protection stepped foldback (100/75/50/25/0 %), external ALC continuous
   * multiplier (30-100% when ext_alc_on), and the Power ALC closed-loop
   * corrector (50-150%, tandem-match forward power vs pa_watts × tx_power%
   * target — see pa_protect.h).
   * Clamped to [0.01, 1.0]. Zero when no PA fitted (pa_watts=0). */
  {
    bool digi    = (g_sdr.mode == MODE_DIGU || g_sdr.mode == MODE_DIGL ||
                    g_sdr.mode == MODE_FREEDV);
    bool cw_like = (g_sdr.mode == MODE_CW) || g_sdr.tune_mode;
    uint8_t gain_src_pct = cw_like ? 100U : (digi ? g_sdr.digi_gain : g_sdr.mic_gain);
    /* TUNE: fixed reduced drive (TUNE_POWER_PCT), independent of the normal
     * tx_power setting — minimizes PA stress while the antenna is mismatched. */
    uint8_t pwr_pct = g_sdr.tune_mode
                       ? ((g_sdr.tx_power < TUNE_POWER_PCT) ? g_sdr.tx_power : TUNE_POWER_PCT)
                       : g_sdr.tx_power;
    int16_t tx_trim = g_band_cal[g_sdr.band_idx].tx_drive_trim;
    float g = pa_fitted
              ? ((float)gain_src_pct                  * (1.0f / 100.0f)
                 * ((float)pwr_pct                      * (1.0f / 100.0f))
                 * ((float)PA_Protect_GetDriveLimit()    * (1.0f / 100.0f))
                 * ((float)PA_Protect_GetALCDrive()      * (1.0f / 100.0f))
                 * ((float)PA_Protect_GetPowerALCDrive() * (1.0f / 100.0f))
                 * ((float)(100 + tx_trim)               * (1.0f / 100.0f)))
              : 0.0f;
    if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    g_dsp.tx.audio_gain = g;
  }
  SDR_UI_UpdateSMeter_SetTX(g_sdr.tx_mode);
  SDR_UI_SetTXMode(g_sdr.tx_mode);
  /* Replace the S-meter immediately on TX start — don't wait for the 1 Hz
   * refresh tick.  UpdateTXMeters is idempotent and uses LCD_PushWindow
   * (synchronous), safe to call here in main-loop context. */
  if (g_sdr.tx_mode) {
    SDR_UI_UpdateTXMeters(0, (int32_t)(g_analog.swr_x100 / 10));
  }
}

/* TUNE: dedicated low-power continuous-carrier mode for external ATU tuning.
 * Forces a fixed reduced drive (TUNE_POWER_PCT, see csdr_apply_tx) and tells
 * PA_Protect to ignore SWR warn/trip (see pa_protect.c) — a bad match is the
 * expected condition while tuning. OC/thermal protection stays fully active.
 * Bound to F4 hold-to-tune in csdr_handle_keys(); auto-stops after
 * TUNE_MAX_MS in case the button sticks. */
static void csdr_tune_start(void)
{
  if (g_sdr.tx_mode) return;   /* already transmitting (PTT/keyer) — ignore */
  g_sdr.tune_mode      = true;
  g_dsp.tx.tune_active = true;
  g_dsp.cw_key_out     = true;
  g_sdr.tx_mode        = true;
  s_tune_start_ms      = HAL_GetTick();
  g_sdr.display_dirty |= (uint8_t)(DIRTY_HDR | DIRTY_VFO | DIRTY_SBR);
  csdr_apply_tx();
}

static void csdr_tune_stop(void)
{
  g_sdr.tune_mode      = false;
  g_dsp.tx.tune_active = false;
  g_dsp.cw_key_out     = false;
  g_sdr.tx_mode        = false;
  g_sdr.display_dirty |= DIRTY_ALL;
  csdr_apply_tx();
}

/* CAT callback — AC command (antenna tuner start/stop).  Same TUNE state as
 * csdr_tune_start/stop but deferred like cat_set_tx: csdr_apply_tx (I2C/GPIO)
 * runs in CSDR_Loop via cat_tx_dirty, never in the CAT handler.
 * The TUNE_MAX_MS safety auto-stop in csdr_handle_keys covers CAT-started
 * tuning too — s_tune_start_ms is armed here as well. */
static void cat_set_tune(bool on)
{
  if (on == g_sdr.tune_mode) return;
  if (on) {
    if (g_sdr.tx_mode) return;   /* already transmitting (PTT/keyer) — ignore */
    g_sdr.tune_mode      = true;
    g_dsp.tx.tune_active = true;
    g_dsp.cw_key_out     = true;
    g_sdr.tx_mode        = true;
    s_tune_start_ms      = HAL_GetTick();
    g_sdr.display_dirty |= (uint8_t)(DIRTY_HDR | DIRTY_VFO | DIRTY_SBR);
  } else {
    g_sdr.tune_mode      = false;
    g_dsp.tx.tune_active = false;
    g_dsp.cw_key_out     = false;
    g_sdr.tx_mode        = false;
    g_sdr.display_dirty |= DIRTY_ALL;
  }
  if (g_sdr.cat_tx_dirty) dbg_cat_blocked_updates++;
  g_sdr.cat_tx_dirty = true;
}

static bool cat_get_tune(void) { return g_sdr.tune_mode; }

/* CAT callback — deferred: sets dirty flag, WM8731 applied by CSDR_Loop. */
static void cat_set_volume(uint8_t vol)
{
  if (vol > 100U) vol = 100U;
  g_sdr.volume = vol;
  g_sdr.cat_vol_dirty = true;
  g_sdr.display_dirty |= DIRTY_SBL;
}
static void     cat_set_nr(uint8_t mode)
{
  if (mode > 2U) mode = 2U;
  g_sdr.nr_mode = mode;
  DSP_NR_Set(&g_dsp, mode, g_sdr.nr_level);
  g_sdr.display_dirty |= DIRTY_SBL;
}
static void     cat_set_nr_level(uint8_t level)
{
  if (level > 100U) level = 100U;
  g_sdr.nr_level = level;
  DSP_NR_Set(&g_dsp, g_sdr.nr_mode, level);
}
static void     cat_set_bc(uint8_t mode)
{
  if (mode > 2U) mode = 2U;
  g_sdr.bc_mode = mode;
  DSP_BC_Set(&g_dsp, mode);
}
static void     cat_set_nb(bool on)       { g_sdr.nb_on = on; DSP_NB_Set(&g_dsp, on, g_sdr.nb_level); g_sdr.display_dirty |= DIRTY_SBL; }
/* BW command diagnostics — watch in Live Expressions.
 *  dbg_last_bw_value:      raw Hz value received from CAT before mode-dependent clamping
 *  dbg_last_bw_mode:       g_sdr.mode (SDR enum) when the BW SET arrived
 *  dbg_last_bw_applied_hz: Hz actually written to DSP after clamping
 *  dbg_bw_invalid_count:   reserved (was FM-rejection counter; FM now accepts BW changes)
 *  dbg_bw_clamped_count:   SETs adjusted to mode bounds (FM 5000–15000, AM 1500–9000, SSB 100–24000) */
volatile uint32_t dbg_last_bw_value      = 0U;
volatile uint32_t dbg_last_bw_mode       = 0U;
volatile uint32_t dbg_last_bw_applied_hz = 0U;
volatile uint32_t dbg_bw_invalid_count   = 0U;
volatile uint32_t dbg_bw_clamped_count   = 0U;

static void cat_set_bw(uint32_t hz)
{
    dbg_last_bw_value = hz;
    dbg_last_bw_mode  = (uint32_t)g_sdr.mode;

    switch (g_sdr.mode) {
        case MODE_FM:
            /* FM accepts BW changes — map to practical [5000, 15000] Hz range.
             * FW GET returns min(bw_hz, 9999) for the 4-digit field; round-trip is
             * stable for any stored value ≤ 9999, and 15000 (default) reports as FW9999. */
            if (hz < 5000U)       { hz = 5000U;  dbg_bw_clamped_count++; }
            else if (hz > 15000U) { hz = 15000U; dbg_bw_clamped_count++; }
            break;
        case MODE_AM:
            /* AM bandwidth via FW only (SH is not applicable to AM in TS-2000).
             * Accepts [1500, 9000] Hz: covers narrow presets some clients send (~2400 Hz)
             * as well as Hamlib normal (9000 Hz) and narrow (6000 Hz) AM filters. */
            if (hz < 1500U)       { hz = 1500U;  dbg_bw_clamped_count++; }
            else if (hz > 9000U)  { hz = 9000U;  dbg_bw_clamped_count++; }
            break;
        default:
            if (hz < 100U)        { hz = 100U;   dbg_bw_clamped_count++; }
            else if (hz > 24000U) { hz = 24000U; dbg_bw_clamped_count++; }
            break;
    }

    dbg_last_bw_applied_hz = hz;
    g_sdr.bw_hz = hz;
    DSP_SetBW(&g_dsp, (float)hz);
    g_sdr.display_dirty |= (DIRTY_VFO | DIRTY_SBL | DIRTY_SBR);
}
static void     cat_set_agc_fast(bool f)  { g_sdr.agc_speed = f ? 1U : 0U; AGC_SetMode(&g_dsp.agc, g_sdr.mode, g_sdr.agc_speed, CSDR_AUDIO_SAMPLE_RATE); g_sdr.display_dirty |= DIRTY_SBL | DIRTY_HDR; }
static void     cat_set_squelch(uint8_t s){ g_sdr.squelch = s; DSP_SetSquelch(&g_dsp, s); g_sdr.display_dirty |= DIRTY_SBL; }

static uint32_t cat_get_freq(void)        { return g_sdr.freq_hz; }
static uint8_t  cat_get_mode(void)        { return (uint8_t)g_sdr.mode; }
static bool     cat_get_tx(void)          { return g_sdr.tx_mode; }
/* Antenna-referenced: add back front-end attenuation so SM does not drop
 * when the RF AGC / manual ATT engages.  Must NOT be fed to RFAGC_Update —
 * the RF AGC loop protects the ADC and needs the post-attenuator level. */
static float    cat_get_signal(void)      { return g_dsp.signal_power_db + (float)g_att.current_atten_x2 * 0.5f; }
static uint8_t  cat_get_att(void)         { return g_sdr.att_db; }
static uint8_t  cat_get_volume(void)      { return g_sdr.volume; }
static uint8_t  cat_get_nr(void)          { return g_sdr.nr_mode; }
static uint8_t  cat_get_nr_level(void)    { return g_sdr.nr_level; }
static uint8_t  cat_get_bc(void)          { return g_sdr.bc_mode; }
static bool     cat_get_nb(void)          { return g_sdr.nb_on; }
static uint32_t cat_get_bw(void)          { return g_sdr.bw_hz; }
static bool     cat_get_agc_fast(void)    { return (g_sdr.agc_speed >= 1U); }  /* FAST or AUTO → true */
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

static void cat_set_lo_cut(uint32_t hz)
{
  if (hz > 9999U) hz = 9999U;
  g_sdr.sl_hz = hz;
  g_sdr.cat_rit_dirty = true;  /* nco_if recomputed with updated sl_sign * sl_hz */
  g_sdr.display_dirty |= DIRTY_VFO;
}
static uint32_t cat_get_lo_cut(void) { return g_sdr.sl_hz; }

static void cat_set_rf_agc(bool on)
{
  RFAGC_SetEnabled(&g_rfagc, on, g_att.current_atten_x2);
  g_sdr.rf_agc_on = on;
  g_sdr.display_dirty |= DIRTY_SBR;
  csdr_save_settings();
}
static bool cat_get_rf_agc(void) { return g_sdr.rf_agc_on; }

static void cat_set_tx_power(uint8_t pct)
{
  if (pct > 100U) pct = 100U;
  g_sdr.tx_power = pct;
  g_sdr.display_dirty |= DIRTY_SBR;
  if (g_sdr.tx_mode) g_sdr.cat_tx_dirty = true; /* re-apply gain mid-TX */
  csdr_save_settings();
}
static uint8_t cat_get_tx_power(void) { return g_sdr.tx_power; }

/* RM1 SWR meter — pa_protect's filtered SWR, the same figure the protection
 * state machine acts on (returns 100 = 1.00 flat when not transmitting). */
static uint16_t cat_get_swr_x100(void) { return PA_Protect_GetSwrX100(); }

/* RM3 ALC meter — external-ALC drive reduction from pa_protect, the same
 * multiplier applied in csdr_apply_tx (0 when RX or ext_alc_on is off). */
static uint8_t cat_get_alc_reduction_pct(void) { return PA_Protect_GetAlcReductionPct(); }

static void cat_set_cw_wpm(uint8_t wpm)
{
  if (wpm < 5U)  wpm = 5U;
  if (wpm > 40U) wpm = 40U;
  g_sdr.cw_wpm = wpm;
  CWKeyer_SetWPM(&g_cw_keyer, wpm);
  g_cw_dec.dit_ms = 1200U / wpm;
  csdr_save_settings();
}
static uint8_t cat_get_cw_wpm(void) { return g_sdr.cw_wpm; }

static void    cat_set_usb_stream(uint8_t m) { g_sdr.usb_iq_stream = (m == 0U); g_sdr.display_dirty |= DIRTY_SBR; }
static uint8_t cat_get_usb_stream(void)      { return g_sdr.usb_iq_stream ? 0U : 1U; }

int32_t *CSDR_GetTxBuf(void) { return s_tx_buf; }
int32_t *CSDR_GetRxBuf(void) { return s_rx_buf; }

/* Called from main() AFTER MX_USB_DEVICE_Init() so OTG_FS priority is live.
 * Priority encoding (STM32H7, NVIC_PRIORITYGROUP_4):
 *   HAL_NVIC_SetPriority(x, N, 0) → N in IPR; lower N = higher urgency.
 *   Required: SAI1/DMA1_Str0-1 < OTG_FS (0 < 2). */
void CSDR_VerifyIRQConfig(void)
{
  uint32_t sai_prio  = NVIC_GetPriority(SAI1_IRQn);
  uint32_t dma0_prio = NVIC_GetPriority(DMA1_Stream0_IRQn);
  uint32_t dma1_prio = NVIC_GetPriority(DMA1_Stream1_IRQn);
  uint32_t otg_prio  = NVIC_GetPriority(OTG_FS_IRQn);
  if (sai_prio >= otg_prio || dma0_prio >= otg_prio || dma1_prio >= otg_prio) {
    RuntimeDiag_SetFault(FAULT_IRQ_CFG);
#if !defined(NDEBUG)
    /* Check usbd_conf.c USER CODE USB_OTG_FS_MspInit 1 for the
     * HAL_NVIC_SetPriority(OTG_FS_IRQn, 2, 0) override — first thing
     * CubeMX regen silently removes. */
    while ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) { __BKPT(0); }
#endif
  }
}
void CSDR_ClearDspFlags(void) { s_rx_ready_seq[0] = s_rx_done_seq[0]; s_rx_ready_seq[1] = s_rx_done_seq[1]; }
