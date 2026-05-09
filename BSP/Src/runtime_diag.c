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
static uint8_t  s_cpu_load_percent = 0U;
static uint32_t s_diag_rate_window_start_ms = 0U;
static uint32_t s_diag_rate_rx_count = 0U;
static uint32_t s_diag_rate_tx_count = 0U;
static uint32_t s_rx_overrun_per_sec = 0U;
static uint32_t s_tx_underrun_per_sec = 0U;
static uint32_t s_last_audio_ok_ms = 0U;
static uint32_t s_last_watchdog_ms = 0U;
static uint32_t s_dsp_stack_words = 0U;
static uint32_t s_gui_stack_words = 0U;
static uint32_t s_cat_stack_words = 0U;

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
  s_audio_block_start_cyc = 0U;
  s_ui_render_start_cyc = 0U;
  s_main_loop_last_cyc = 0U;
  s_max_dsp_cycles = 0U;
  s_max_ui_cycles = 0U;
  s_max_loop_stall_cycles = 0U;
  s_underrun_dsp_cycles = 0U;
  s_underrun_ui_cycles = 0U;
  s_underrun_loop_stall_cycles = 0U;
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
  out->rx_overrun_per_sec = s_rx_overrun_per_sec;
  out->tx_underrun_per_sec = s_tx_underrun_per_sec;
  out->max_dsp_us = diag_cycles_to_us(s_max_dsp_cycles);
  out->max_ui_us = diag_cycles_to_us(s_max_ui_cycles);
  out->max_loop_stall_us = diag_cycles_to_us(s_max_loop_stall_cycles);
  out->underrun_dsp_us = diag_cycles_to_us(s_underrun_dsp_cycles);
  out->underrun_ui_us = diag_cycles_to_us(s_underrun_ui_cycles);
  out->underrun_loop_stall_us = diag_cycles_to_us(s_underrun_loop_stall_cycles);
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
