#include "runtime_diag.h"
#include "core_cm7.h"

volatile uint32_t rx_overrun_count = 0U;
volatile uint32_t tx_underrun_count = 0U;
volatile uint32_t runtime_fault_flags = 0U;

static volatile uint32_t s_rx_dma_seq[2] = { 0U, 0U };
static volatile uint32_t s_rx_consumed_seq[2] = { 0U, 0U };
static volatile uint32_t s_tx_fill_seq[2] = { 1U, 1U };
static volatile uint32_t s_tx_dma_seq[2] = { 0U, 0U };
static uint32_t s_cpu_window_start_ms = 0U;
static uint32_t s_cpu_window_start_cyc = 0U;
static uint32_t s_cpu_active_cycles = 0U;
static uint32_t s_audio_block_start_cyc = 0U;
static uint32_t s_ui_render_start_cyc = 0U;
static uint32_t s_main_loop_last_cyc = 0U;
static uint32_t s_max_dsp_cycles = 0U;
static uint32_t s_max_ui_cycles = 0U;
static uint32_t s_max_loop_stall_cycles = 0U;
static uint32_t s_underrun_dsp_cycles = 0U;
static uint32_t s_underrun_ui_cycles = 0U;
static uint32_t s_underrun_loop_stall_cycles = 0U;
static uint32_t s_ui_section_start_cyc[RUNTIME_DIAG_UI_SECTION_COUNT];
static uint32_t s_ui_section_max_cyc[RUNTIME_DIAG_UI_SECTION_COUNT];
static uint8_t  s_cpu_load_percent = 0U;
static uint32_t s_diag_rate_window_start_ms = 0U;
static uint32_t s_diag_rate_rx_count = 0U;
static uint32_t s_diag_rate_tx_count = 0U;
static uint32_t s_rx_overrun_per_sec = 0U;
static uint32_t s_tx_underrun_per_sec = 0U;
static uint32_t s_max_rx_overrun_per_sec = 0U;
static uint32_t s_max_tx_underrun_per_sec = 0U;
static uint32_t s_last_audio_ok_ms = 0U;
static uint32_t s_last_watchdog_ms = 0U;
static uint32_t s_dsp_stack_words = 0U;
static uint32_t s_gui_stack_words = 0U;
static uint32_t s_cat_stack_words = 0U;
static uint32_t s_wf_skip_count          = 0U;
static uint8_t  s_ui_load_high           = 0U;
/* LCD chunk stats — written by RuntimeDiag_LcdChunkReport (called from sdr_ui) */
static uint32_t s_lcd_chunk_count        = 0U;
static uint32_t s_lcd_chunk_abort_count  = 0U;
static uint32_t s_wf_partial_count       = 0U;
static uint32_t s_max_chunk_render_us    = 0U;
/* FFT timing — written by RuntimeDiag_FftReport (called from sdr_dsp) */
static uint32_t s_max_fft_cycles         = 0U;
static uint32_t s_avg_fft_cycles         = 0U;  /* EMA, α = 1/64 */
/* Spectrum partial-redraw stats — written by RuntimeDiag_SpecReport (sdr_ui) */
static uint32_t s_spec_partial_count     = 0U;
static uint32_t s_spec_skip_count        = 0U;
static uint32_t s_max_spec_partial_us    = 0U;
/* VFO glyph-redraw stats — written by RuntimeDiag_VfoReport (sdr_ui) */
static uint32_t s_vfo_glyph_count        = 0U;
static uint32_t s_vfo_skip_count         = 0U;
static uint32_t s_max_vfo_us             = 0U;
/* Async LCD DMA stats — written by RuntimeDiag_LcdDmaReport (sdr_ui) */
static uint32_t s_lcd_dma_max_latency_us = 0U;
static uint32_t s_lcd_dma_queued_count   = 0U;
static uint8_t  s_lcd_dma_busy           = 0U;

#if RUNTIMEDIAG_FREERTOS_AVAILABLE
static TaskHandle_t s_dsp_task = NULL;
static TaskHandle_t s_gui_task = NULL;
static TaskHandle_t s_cat_task = NULL;
#endif

#if defined(HAL_IWDG_MODULE_ENABLED)
extern IWDG_HandleTypeDef hiwdg;
#endif

static void diag_enable_cycle_counter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void RuntimeDiag_Init(void)
{
  rx_overrun_count = 0U;
  tx_underrun_count = 0U;
  runtime_fault_flags = 0U;
  s_rx_dma_seq[0] = 0U;
  s_rx_dma_seq[1] = 0U;
  s_rx_consumed_seq[0] = 0U;
  s_rx_consumed_seq[1] = 0U;
  s_tx_fill_seq[0] = 1U;
  s_tx_fill_seq[1] = 1U;
  s_tx_dma_seq[0] = 0U;
  s_tx_dma_seq[1] = 0U;
  s_diag_rate_rx_count = 0U;
  s_diag_rate_tx_count = 0U;
  s_rx_overrun_per_sec = 0U;
  s_tx_underrun_per_sec = 0U;
  s_max_rx_overrun_per_sec = 0U;
  s_max_tx_underrun_per_sec = 0U;
  s_audio_block_start_cyc = 0U;
  s_ui_render_start_cyc = 0U;
  s_main_loop_last_cyc = 0U;
  s_max_dsp_cycles = 0U;
  s_max_ui_cycles = 0U;
  s_max_loop_stall_cycles = 0U;
  s_underrun_dsp_cycles = 0U;
  s_underrun_ui_cycles = 0U;
  s_underrun_loop_stall_cycles = 0U;
  for (uint8_t i = 0U; i < (uint8_t)RUNTIME_DIAG_UI_SECTION_COUNT; i++) {
    s_ui_section_start_cyc[i] = 0U;
    s_ui_section_max_cyc[i] = 0U;
  }
  s_max_fft_cycles = 0U;
  s_avg_fft_cycles = 0U;
  diag_enable_cycle_counter();
  s_cpu_window_start_ms = HAL_GetTick();
  s_cpu_window_start_cyc = DWT->CYCCNT;
  s_diag_rate_window_start_ms = s_cpu_window_start_ms;
  s_last_audio_ok_ms = s_cpu_window_start_ms;
}

void RuntimeDiag_SetFault(uint32_t fault_mask)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  runtime_fault_flags |= fault_mask;
  __set_PRIMASK(primask);
}

void RuntimeDiag_ClearFaults(uint32_t fault_mask)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  runtime_fault_flags &= ~fault_mask;
  __set_PRIMASK(primask);
}

uint32_t RuntimeDiag_GetFaults(void)
{
  return runtime_fault_flags;
}

void RuntimeDiag_AudioBlockBegin(void)
{
  s_audio_block_start_cyc = DWT->CYCCNT;
}

static uint32_t diag_cycles_to_us(uint32_t cycles)
{
  uint32_t hz_per_us = SystemCoreClock / 1000000U;
  if (hz_per_us == 0U) return 0U;
  return cycles / hz_per_us;
}

void RuntimeDiag_AudioBlockEnd(void)
{
  uint32_t cycles = (uint32_t)(DWT->CYCCNT - s_audio_block_start_cyc);
  s_cpu_active_cycles += cycles;
  if (cycles > s_max_dsp_cycles) s_max_dsp_cycles = cycles;
  RuntimeDiag_MarkAudioHealthy();
}

void RuntimeDiag_UiRenderBegin(void)
{
  s_ui_render_start_cyc = DWT->CYCCNT;
}

void RuntimeDiag_UiRenderEnd(void)
{
  uint32_t cycles = (uint32_t)(DWT->CYCCNT - s_ui_render_start_cyc);
  if (cycles > s_max_ui_cycles) s_max_ui_cycles = cycles;
}

void RuntimeDiag_UiSectionBegin(RuntimeDiag_UiSection_t section)
{
  if ((uint32_t)section >= (uint32_t)RUNTIME_DIAG_UI_SECTION_COUNT) return;
  s_ui_section_start_cyc[section] = DWT->CYCCNT;
}

void RuntimeDiag_UiSectionEnd(RuntimeDiag_UiSection_t section)
{
  if ((uint32_t)section >= (uint32_t)RUNTIME_DIAG_UI_SECTION_COUNT) return;
  uint32_t cycles = (uint32_t)(DWT->CYCCNT - s_ui_section_start_cyc[section]);
  if (cycles > s_ui_section_max_cyc[section]) s_ui_section_max_cyc[section] = cycles;
}

void RuntimeDiag_MainLoopBeat(void)
{
  uint32_t now_cyc = DWT->CYCCNT;
  if (s_main_loop_last_cyc != 0U) {
    uint32_t cycles = (uint32_t)(now_cyc - s_main_loop_last_cyc);
    if (cycles > s_max_loop_stall_cycles) s_max_loop_stall_cycles = cycles;
  }
  s_main_loop_last_cyc = now_cyc;
}

void RuntimeDiag_ServiceSlow(uint32_t now_ms)
{
  uint32_t elapsed_ms = now_ms - s_cpu_window_start_ms;
  if (elapsed_ms >= 500U) {
    uint32_t elapsed_cycles = (uint32_t)(DWT->CYCCNT - s_cpu_window_start_cyc);
    uint32_t pct = 0U;
    uint32_t one_percent_cycles = elapsed_cycles / 100U;
    if (one_percent_cycles != 0U) {
      pct = s_cpu_active_cycles / one_percent_cycles;
      if (pct > 100U) pct = 100U;
    }
    s_cpu_load_percent = (uint8_t)pct;
    s_cpu_active_cycles = 0U;
    s_cpu_window_start_cyc = DWT->CYCCNT;
    s_cpu_window_start_ms = now_ms;

#if RUNTIMEDIAG_FREERTOS_AVAILABLE
    if (s_dsp_task != NULL) s_dsp_stack_words = (uint32_t)uxTaskGetStackHighWaterMark(s_dsp_task);
    if (s_gui_task != NULL) s_gui_stack_words = (uint32_t)uxTaskGetStackHighWaterMark(s_gui_task);
    if (s_cat_task != NULL) s_cat_stack_words = (uint32_t)uxTaskGetStackHighWaterMark(s_cat_task);
#endif
  }

  uint32_t rate_elapsed_ms = now_ms - s_diag_rate_window_start_ms;
  if (rate_elapsed_ms >= 1000U) {
    uint32_t rx_now = rx_overrun_count;
    uint32_t tx_now = tx_underrun_count;
    s_rx_overrun_per_sec = ((rx_now - s_diag_rate_rx_count) * 1000U) / rate_elapsed_ms;
    s_tx_underrun_per_sec = ((tx_now - s_diag_rate_tx_count) * 1000U) / rate_elapsed_ms;
    if (s_rx_overrun_per_sec > s_max_rx_overrun_per_sec) s_max_rx_overrun_per_sec = s_rx_overrun_per_sec;
    if (s_tx_underrun_per_sec > s_max_tx_underrun_per_sec) s_max_tx_underrun_per_sec = s_tx_underrun_per_sec;
    s_diag_rate_rx_count = rx_now;
    s_diag_rate_tx_count = tx_now;
    s_diag_rate_window_start_ms = now_ms;
  }
}

void RuntimeDiag_GetSnapshot(RuntimeDiag_Snapshot_t *out)
{
  if (out == NULL) return;
  out->fault_flags = runtime_fault_flags;
  out->cpu_load_percent = s_cpu_load_percent;
  out->dsp_stack_words = s_dsp_stack_words;
  out->gui_stack_words = s_gui_stack_words;
  out->cat_stack_words = s_cat_stack_words;
  out->rx_overrun_total = rx_overrun_count;
  out->tx_underrun_total = tx_underrun_count;
  out->rx_overrun_per_sec = s_rx_overrun_per_sec;
  out->tx_underrun_per_sec = s_tx_underrun_per_sec;
  out->max_rx_overrun_per_sec = s_max_rx_overrun_per_sec;
  out->max_tx_underrun_per_sec = s_max_tx_underrun_per_sec;
  out->max_dsp_us = diag_cycles_to_us(s_max_dsp_cycles);
  out->max_ui_us = diag_cycles_to_us(s_max_ui_cycles);
  out->max_loop_stall_us = diag_cycles_to_us(s_max_loop_stall_cycles);
  out->underrun_dsp_us = diag_cycles_to_us(s_underrun_dsp_cycles);
  out->underrun_ui_us = diag_cycles_to_us(s_underrun_ui_cycles);
  out->underrun_loop_stall_us = diag_cycles_to_us(s_underrun_loop_stall_cycles);
  for (uint8_t i = 0U; i < (uint8_t)RUNTIME_DIAG_UI_SECTION_COUNT; i++) {
    out->ui_section_max_us[i] = diag_cycles_to_us(s_ui_section_max_cyc[i]);
  }
  out->wf_skip_count           = s_wf_skip_count;
  out->wf_render_max_us        = diag_cycles_to_us(s_ui_section_max_cyc[RUNTIME_DIAG_UI_WATERFALL]);
  out->spec_render_max_us      = diag_cycles_to_us(s_ui_section_max_cyc[RUNTIME_DIAG_UI_SPECTRUM]);
  out->ui_load_high            = (s_ui_load_high != 0U);
  out->lcd_chunk_count         = s_lcd_chunk_count;
  out->lcd_chunk_abort_count   = s_lcd_chunk_abort_count;
  out->wf_partial_render_count = s_wf_partial_count;
  out->max_chunk_render_us     = s_max_chunk_render_us;
  out->max_fft_us              = diag_cycles_to_us(s_max_fft_cycles);
  out->avg_fft_us              = diag_cycles_to_us(s_avg_fft_cycles);
  out->spec_partial_redraw_count = s_spec_partial_count;
  out->spec_skip_count           = s_spec_skip_count;
  out->max_spec_partial_us       = s_max_spec_partial_us;
  out->vfo_glyph_redraw_count    = s_vfo_glyph_count;
  out->vfo_skip_count            = s_vfo_skip_count;
  out->max_vfo_redraw_us         = s_max_vfo_us;
  out->lcd_dma_max_latency_us    = s_lcd_dma_max_latency_us;
  out->lcd_dma_queued_count      = s_lcd_dma_queued_count;
  out->lcd_dma_busy              = (s_lcd_dma_busy != 0U);
}

uint32_t RuntimeDiag_RxHalfIsr(uint8_t half_index)
{
  if (half_index >= 2U) return 0U;

  uint32_t prev_seq = s_rx_dma_seq[half_index];
  if (prev_seq != s_rx_consumed_seq[half_index]) {
    rx_overrun_count++;
    RuntimeDiag_SetFault(FAULT_DMA_RX_OVR);
  }

  prev_seq++;
  s_rx_dma_seq[half_index] = prev_seq;
  return prev_seq;
}

void RuntimeDiag_RxHalfConsumed(uint8_t half_index, uint32_t sequence)
{
  if (half_index >= 2U) return;
  s_rx_consumed_seq[half_index] = sequence;
}

void RuntimeDiag_TxHalfFilled(uint8_t half_index)
{
  if (half_index >= 2U) return;
  s_tx_fill_seq[half_index]++;
}

void RuntimeDiag_TxHalfConsumedIsr(uint8_t half_index, bool tx_active)
{
  if (half_index >= 2U) return;

  uint32_t fill_seq = s_tx_fill_seq[half_index];
  if (tx_active && fill_seq == s_tx_dma_seq[half_index]) {
    tx_underrun_count++;
    s_underrun_dsp_cycles = s_max_dsp_cycles;
    s_underrun_ui_cycles = s_max_ui_cycles;
    s_underrun_loop_stall_cycles = s_max_loop_stall_cycles;
    RuntimeDiag_SetFault(FAULT_I2S_TX_UND);
  }
  s_tx_dma_seq[half_index] = fill_seq;
}

void RuntimeDiag_MarkAudioHealthy(void)
{
  s_last_audio_ok_ms = HAL_GetTick();
}

void RuntimeDiag_ResetPeaks(void)
{
  s_max_dsp_cycles = 0U;
  s_max_ui_cycles = 0U;
  s_max_loop_stall_cycles = 0U;
  s_underrun_dsp_cycles = 0U;
  s_underrun_ui_cycles = 0U;
  s_underrun_loop_stall_cycles = 0U;
  for (uint8_t i = 0U; i < (uint8_t)RUNTIME_DIAG_UI_SECTION_COUNT; i++) {
    s_ui_section_max_cyc[i] = 0U;
  }
  s_max_rx_overrun_per_sec = 0U;
  s_max_tx_underrun_per_sec = 0U;
  s_wf_skip_count = 0U;
  s_max_chunk_render_us = 0U;  /* reset peak; cumulative counters are not reset */
  s_max_fft_cycles = 0U;
  s_max_spec_partial_us = 0U;
  s_max_vfo_us = 0U;
  s_lcd_dma_max_latency_us = 0U;
  /* rolling averages / cumulative totals not reset */
}

void RuntimeDiag_WfSkipReport(uint32_t skip_count, bool load_high)
{
  s_wf_skip_count = skip_count;
  s_ui_load_high  = load_high ? 1U : 0U;
}

bool RuntimeDiag_IsUiOverload(void)
{
  return (s_ui_load_high != 0U);
}

void RuntimeDiag_LcdChunkReport(uint32_t chunk_count, uint32_t abort_count,
                                 uint32_t partial_count, uint32_t max_chunk_us)
{
  s_lcd_chunk_count       = chunk_count;
  s_lcd_chunk_abort_count = abort_count;
  s_wf_partial_count      = partial_count;
  if (max_chunk_us > s_max_chunk_render_us) s_max_chunk_render_us = max_chunk_us;
}

void RuntimeDiag_FftReport(uint32_t cycles)
{
  /* FFT runs at ~187 Hz (256-sample frame / 48 kHz), making it the largest
   * single DSP cost.  Tracking peak and rolling average here lets the debugger
   * confirm that CMSIS arm_cfft_f32 beats the custom twiddle-LUT fallback and
   * catches any regression without external instrumentation.
   * EMA with α = 1/64 smooths transient spikes while converging in ~3 s. */
  if (cycles > s_max_fft_cycles) s_max_fft_cycles = cycles;
  s_avg_fft_cycles = (s_avg_fft_cycles * 63U + cycles) / 64U;
}

void RuntimeDiag_SpecReport(uint32_t partial_count, uint32_t skip_count,
                             uint32_t max_partial_us)
{
  s_spec_partial_count  = partial_count;
  s_spec_skip_count     = skip_count;
  if (max_partial_us > s_max_spec_partial_us)
    s_max_spec_partial_us = max_partial_us;
}

void RuntimeDiag_VfoReport(uint32_t glyph_count, uint32_t skip_count,
                            uint32_t max_redraw_us)
{
  s_vfo_glyph_count = glyph_count;
  s_vfo_skip_count  = skip_count;
  if (max_redraw_us > s_max_vfo_us) s_max_vfo_us = max_redraw_us;
}

void RuntimeDiag_LcdDmaReport(uint32_t max_latency_us, uint32_t queued_count,
                               bool busy)
{
  /* Peak latency is reported per-tick; keep the all-time maximum. */
  if (max_latency_us > s_lcd_dma_max_latency_us)
    s_lcd_dma_max_latency_us = max_latency_us;
  s_lcd_dma_queued_count = queued_count;
  s_lcd_dma_busy         = busy ? 1U : 0U;
}

void RuntimeDiag_WatchdogRefreshIfHealthy(uint32_t now_ms)
{
  if ((now_ms - s_last_audio_ok_ms) > 100U) return;
  if ((now_ms - s_last_watchdog_ms) < 25U) return;
  s_last_watchdog_ms = now_ms;
#if defined(HAL_IWDG_MODULE_ENABLED)
  HAL_IWDG_Refresh(&hiwdg);
#else
  (void)now_ms;
#endif
}

#if RUNTIMEDIAG_FREERTOS_AVAILABLE
void RuntimeDiag_RegisterTaskHandles(TaskHandle_t dsp_task,
                                     TaskHandle_t gui_task,
                                     TaskHandle_t cat_task)
{
  s_dsp_task = dsp_task;
  s_gui_task = gui_task;
  s_cat_task = cat_task;
}
#endif
