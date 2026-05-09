#ifndef __RUNTIME_DIAG_H
#define __RUNTIME_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define FAULT_DMA_RX_OVR  (1UL << 0)
#define FAULT_I2S_TX_UND  (1UL << 1)
#define FAULT_CODEC       (1UL << 2)
#define FAULT_PLL         (1UL << 3)

typedef struct {
  uint32_t fault_flags;
  uint8_t  cpu_load_percent;
  uint32_t dsp_stack_words;
  uint32_t gui_stack_words;
  uint32_t cat_stack_words;
  uint32_t rx_overrun_per_sec;
  uint32_t tx_underrun_per_sec;
} RuntimeDiag_Snapshot_t;

extern volatile uint32_t rx_overrun_count;
extern volatile uint32_t tx_underrun_count;
extern volatile uint32_t runtime_fault_flags;

void RuntimeDiag_Init(void);
void RuntimeDiag_SetFault(uint32_t fault_mask);
void RuntimeDiag_ClearFaults(uint32_t fault_mask);
uint32_t RuntimeDiag_GetFaults(void);

void RuntimeDiag_AudioBlockBegin(void);
void RuntimeDiag_AudioBlockEnd(void);
void RuntimeDiag_ServiceSlow(uint32_t now_ms);
void RuntimeDiag_GetSnapshot(RuntimeDiag_Snapshot_t *out);

uint32_t RuntimeDiag_RxHalfIsr(uint8_t half_index);
void RuntimeDiag_RxHalfConsumed(uint8_t half_index, uint32_t sequence);
void RuntimeDiag_TxHalfFilled(uint8_t half_index);
void RuntimeDiag_TxHalfConsumedIsr(uint8_t half_index, bool tx_active);

void RuntimeDiag_MarkAudioHealthy(void);
void RuntimeDiag_WatchdogRefreshIfHealthy(uint32_t now_ms);

#if defined(__has_include)
# if __has_include("FreeRTOS.h") && __has_include("task.h")
#  include "FreeRTOS.h"
#  include "task.h"
#  define RUNTIMEDIAG_FREERTOS_AVAILABLE 1
void RuntimeDiag_RegisterTaskHandles(TaskHandle_t dsp_task,
                                     TaskHandle_t gui_task,
                                     TaskHandle_t cat_task);
# endif
#endif

#ifndef RUNTIMEDIAG_FREERTOS_AVAILABLE
#define RUNTIMEDIAG_FREERTOS_AVAILABLE 0
#endif

#ifdef __cplusplus
}
#endif

#endif /* __RUNTIME_DIAG_H */
