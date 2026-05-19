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

typedef enum {
  RUNTIME_DIAG_UI_WATERFALL = 0,
  RUNTIME_DIAG_UI_SPECTRUM,
  RUNTIME_DIAG_UI_LCD_FLUSH,
  RUNTIME_DIAG_UI_TEXT,
  RUNTIME_DIAG_UI_STATUS_BAR,
  RUNTIME_DIAG_UI_VOLUME_MODE,
  RUNTIME_DIAG_UI_SPI_TRANSFER,
  RUNTIME_DIAG_UI_WF_PRECOMPUTE,
  RUNTIME_DIAG_UI_WF_SCROLL,
  RUNTIME_DIAG_UI_SECTION_COUNT
} RuntimeDiag_UiSection_t;

typedef struct {
  uint32_t fault_flags;
  uint8_t  cpu_load_percent;
  uint32_t dsp_stack_words;
  uint32_t gui_stack_words;
  uint32_t cat_stack_words;
  uint32_t rx_overrun_total;
  uint32_t tx_underrun_total;
  uint32_t rx_overrun_per_sec;
  uint32_t tx_underrun_per_sec;
  uint32_t max_rx_overrun_per_sec;
  uint32_t max_tx_underrun_per_sec;
  uint32_t max_dsp_us;
  uint32_t max_ui_us;
  uint32_t max_loop_stall_us;
  uint32_t underrun_dsp_us;
  uint32_t underrun_ui_us;
  uint32_t underrun_loop_stall_us;
  uint32_t ui_section_max_us[RUNTIME_DIAG_UI_SECTION_COUNT];
  /* UI adaptive-skip state (reported by CSDR_Loop waterfall tick) */
  uint32_t wf_skip_count;      /*!< Waterfall frames suppressed by overload-hysteresis */
  uint32_t wf_render_max_us;   /*!< Alias: ui_section_max_us[RUNTIME_DIAG_UI_WATERFALL] */
  uint32_t spec_render_max_us; /*!< Alias: ui_section_max_us[RUNTIME_DIAG_UI_SPECTRUM]  */
  bool     ui_load_high;       /*!< true = system load too high for waterfall            */
  /* Chunked LCD push statistics (reported by sdr_ui via RuntimeDiag_LcdChunkReport) */
  uint32_t lcd_chunk_count;          /*!< Total 8-row LCD strips pushed since boot      */
  uint32_t lcd_chunk_abort_count;    /*!< Waterfall renders aborted mid-way by overload */
  uint32_t wf_partial_render_count;  /*!< Same as abort_count — partial waterfall frames*/
  uint32_t max_chunk_render_us;      /*!< Peak µs to push one 8-row LCD strip           */
  /* FFT execution timing — filled by RuntimeDiag_FftReport after each FFT call.
   * FFT runs at ~187 Hz (256 samples / 48 kHz); these counters let us compare
   * CMSIS arm_cfft_f32 against the custom fallback and detect regressions. */
  uint32_t max_fft_us;    /*!< Peak FFT execution time (µs)                  */
  uint32_t avg_fft_us;    /*!< Rolling-average FFT time (µs, EMA α=1/64)     */
  /* Spectrum partial-redraw statistics (reported by sdr_ui via RuntimeDiag_SpecReport) */
  uint32_t spec_partial_redraw_count; /*!< Spectrum frames pushed as partial column band */
  uint32_t spec_skip_count;           /*!< Spectrum frames skipped (delta < 2 px)        */
  uint32_t max_spec_partial_us;       /*!< Peak µs for a partial spectrum push            */
  /* VFO glyph-level redraw statistics (reported by sdr_ui via RuntimeDiag_VfoReport) */
  uint32_t vfo_glyph_redraw_count;    /*!< VFO partial glyph-band pushes since boot       */
  uint32_t vfo_skip_count;            /*!< VFO upper-section pushes skipped (unchanged)   */
  uint32_t max_vfo_redraw_us;         /*!< Peak µs for a glyph-level VFO push             */
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
void RuntimeDiag_UiRenderBegin(void);
void RuntimeDiag_UiRenderEnd(void);
void RuntimeDiag_UiSectionBegin(RuntimeDiag_UiSection_t section);
void RuntimeDiag_UiSectionEnd(RuntimeDiag_UiSection_t section);
void RuntimeDiag_MainLoopBeat(void);
void RuntimeDiag_ServiceSlow(uint32_t now_ms);
void RuntimeDiag_GetSnapshot(RuntimeDiag_Snapshot_t *out);
void RuntimeDiag_ResetPeaks(void);

/* Report waterfall adaptive-skip state from the main-loop waterfall tick */
void RuntimeDiag_WfSkipReport(uint32_t skip_count, bool load_high);

/* Quick overload query used by sdr_ui between LCD chunks.
 * Returns true when the adaptive waterfall hysteresis flag is raised,
 * signalling that the system is under enough load to warrant aborting
 * a partially-rendered waterfall strip rather than stalling further. */
bool RuntimeDiag_IsUiOverload(void);

/* Called by sdr_ui after every waterfall render to sync chunk counters
 * into the runtime-diag snapshot.  Passing the cumulative totals is safe
 * because this function simply copies them; it does not double-count. */
void RuntimeDiag_LcdChunkReport(uint32_t chunk_count, uint32_t abort_count,
                                 uint32_t partial_count, uint32_t max_chunk_us);

/* Called by sdr_dsp after each FFT_Precomp() invocation with the raw DWT
 * cycle count consumed.  Updates peak and rolling-average FFT timing so the
 * snapshot can compare CMSIS vs custom backend cost at any time. */
void RuntimeDiag_FftReport(uint32_t cycles);

/* Called by sdr_ui after each spectrum render with cumulative partial-push stats. */
void RuntimeDiag_SpecReport(uint32_t partial_count, uint32_t skip_count,
                             uint32_t max_partial_us);

/* Called by sdr_ui after each VFO render with cumulative glyph-push stats. */
void RuntimeDiag_VfoReport(uint32_t glyph_count, uint32_t skip_count,
                            uint32_t max_redraw_us);

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
