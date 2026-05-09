#include "runtime_diag.h"
#include "core_cm7.h"

volatile uint32_t rx_overrun_count = 0U;
volatile uint32_t tx_underrun_count = 0U;
volatile uint32_t runtime_fault_flags = 0U;

static volatile uint8_t s_tx_half_ready[2] = { 1U, 1U };
static volatile uint8_t s_rx_half_busy[2] = { 0U, 0U };
static uint32_t s_cpu_window_start_ms = 0U;
static uint32_t s_cpu_window_start_cyc = 0U;
static uint32_t s_cpu_active_cycles = 0U;
static uint32_t s_audio_block_start_cyc = 0U;
static uint8_t  s_cpu_load_percent = 0U;
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
  s_tx_half_ready[0] = 1U;
  s_tx_half_ready[1] = 1U;
  s_rx_half_busy[0] = 0U;
  s_rx_half_busy[1] = 0U;
  diag_enable_cycle_counter();
  s_cpu_window_start_ms = HAL_GetTick();
  s_cpu_window_start_cyc = DWT->CYCCNT;
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

void RuntimeDiag_AudioBlockEnd(void)
{
  s_cpu_active_cycles += (uint32_t)(DWT->CYCCNT - s_audio_block_start_cyc);
  RuntimeDiag_MarkAudioHealthy();
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
}

void RuntimeDiag_GetSnapshot(RuntimeDiag_Snapshot_t *out)
{
  if (out == NULL) return;
  out->fault_flags = runtime_fault_flags;
  out->cpu_load_percent = s_cpu_load_percent;
  out->dsp_stack_words = s_dsp_stack_words;
  out->gui_stack_words = s_gui_stack_words;
  out->cat_stack_words = s_cat_stack_words;
}

void RuntimeDiag_RxHalfIsr(uint8_t half_index, volatile uint8_t *pending_flag)
{
  if (pending_flag == NULL) return;
  if (*pending_flag != 0U || (half_index < 2U && s_rx_half_busy[half_index] != 0U)) {
    rx_overrun_count++;
    RuntimeDiag_SetFault(FAULT_DMA_RX_OVR);
  }
  if (half_index < 2U) s_rx_half_busy[half_index] = 1U;
  *pending_flag = 1U;
}

void RuntimeDiag_RxHalfConsumed(uint8_t half_index)
{
  if (half_index < 2U) s_rx_half_busy[half_index] = 0U;
}

void RuntimeDiag_TxHalfFilled(uint8_t half_index)
{
  if (half_index < 2U) s_tx_half_ready[half_index] = 1U;
}

void RuntimeDiag_TxHalfConsumedIsr(uint8_t half_index)
{
  if (half_index >= 2U) return;
  if (s_tx_half_ready[half_index] == 0U) {
    tx_underrun_count++;
    RuntimeDiag_SetFault(FAULT_I2S_TX_UND);
  }
  s_tx_half_ready[half_index] = 0U;
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
