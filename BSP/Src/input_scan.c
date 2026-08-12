/* USER CODE BEGIN Header */
/**
  * @file  input_scan.c
  * @brief Centralized input scan – PCA9555 managed path
  *
  *  When HAS_PCA9555 == 0 (chip not installed) all functions become no-ops.
  *  g_pca9555_raw stays 0xFFFF (all buttons released) and the five
  *  dbg_pca_* counters let you confirm no I2C traffic is attempted.
  */
/* USER CODE END Header */

#include "input_scan.h"
#include "hw_fault.h"
#if HAS_PCA9555
#include "pca9555.h"
extern I2C_HandleTypeDef hi2c2;
static PCA9555_t s_pca;
#endif

/* ── Shared cache (extern-declared in input_scan.h) ─────────────────────── */
uint16_t g_pca9555_raw = 0xFFFFU;   /* all bits high = all keys released */

/* ── Diagnostic counters (readable via debugger / GDB) ──────────────────── */
uint32_t dbg_pca_init_attempts  = 0;
uint32_t dbg_pca_read_attempts  = 0;
uint32_t dbg_pca_write_attempts = 0;
uint32_t dbg_pca_timeout_count  = 0;
uint32_t dbg_pca_disabled_hits  = 0;

/* ── I2C skip / retry state ──────────────────────────────────────────────── */
#if HAS_PCA9555
/* Khi s_pca.ok=false, Input_Scan() bỏ qua I2C phím để tránh stall mỗi vòng
 * loop (2 HAL call × PCA9555_TIMEOUT_MS).  Lỗi thoáng qua (nhiễu RF lúc TX,
 * NACK đơn lẻ) chỉ bị phạt PCA_RETRY_FAST_MS rồi thử lại ngay; chỉ sau
 * PCA_FAIL_SLOW_THRESHOLD lần thất bại LIÊN TIẾP mới coi là chip vắng mặt
 * và giãn ra PCA_RETRY_MS để bắt hot-plug.  HW_FAULT_KEYS là latch một
 * chiều nên chỉ set khi lỗi kéo dài qua cả pha retry chậm
 * (PCA_FAIL_FAULT_THRESHOLD ≈ hơn 10 s), không set vì một burst nhiễu. */
#define PCA_RETRY_MS              5000U
#define PCA_RETRY_FAST_MS          100U
#define PCA_FAIL_SLOW_THRESHOLD      3U
#define PCA_FAIL_FAULT_THRESHOLD     5U
static uint32_t s_pca_retry_ms    = 0U;
static uint8_t  s_pca_fail_streak = 0U;  /* consecutive I2C failures (read or init) */

/* INT-driven flag: set by EXTI ISR via Input_SetIrqPending(), cleared here */
static volatile uint8_t s_pca_irq_pending = 0U;

void Input_SetIrqPending(void) { s_pca_irq_pending = 1U; }
#else
void Input_SetIrqPending(void) {}
#endif

/* ════════════════════════════════════════════════════════════════════════════
 *  Input_Init
 * ════════════════════════════════════════════════════════════════════════════ */

void Input_Init(void)
{
#if HAS_PCA9555
  dbg_pca_init_attempts++;
  if (PCA9555_Init(&s_pca, &hi2c2, INPUT_PCA9555_ADDR) == HAL_OK) {
    if (PCA9555_ReadInputs(&s_pca) == HAL_OK)
      g_pca9555_raw = s_pca.raw;
    else
      dbg_pca_timeout_count++;
  } else {
    dbg_pca_timeout_count++;
  }
#else
  dbg_pca_disabled_hits++;
  /* g_pca9555_raw stays 0xFFFFU — all buttons released, no I2C attempted */
#endif
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Input_Scan  — call once per main loop before Key_Poll()
 * ════════════════════════════════════════════════════════════════════════════ */

void Input_Scan(void)
{
#if HAS_PCA9555
  dbg_pca_read_attempts++;

  if (!s_pca.ok) {
    /* Device in fault state.  First PCA_FAIL_SLOW_THRESHOLD consecutive
     * failures are transient territory (EMI/NACK): retry after
     * PCA_RETRY_FAST_MS so keys come back within ~0.1 s.  Beyond that the
     * chip is treated as absent and probed every PCA_RETRY_MS. */
    uint32_t now = HAL_GetTick();
    uint32_t gap = (s_pca_fail_streak >= PCA_FAIL_SLOW_THRESHOLD)
                   ? PCA_RETRY_MS : PCA_RETRY_FAST_MS;
    if ((now - s_pca_retry_ms) < gap)
      return;
    s_pca_retry_ms = now;
    if (PCA9555_Init(&s_pca, s_pca.hi2c, s_pca.addr) != HAL_OK) {
      dbg_pca_timeout_count++;
      if (s_pca_fail_streak < 0xFFU) s_pca_fail_streak++;
#if HW_FAULT_WARN
      if (s_pca_fail_streak >= PCA_FAIL_FAULT_THRESHOLD)
        HW_Fault_Set(HW_FAULT_KEYS);
#endif
      return;
    }
    dbg_pca_init_attempts++;
    /* Fall through: read initial state after successful reinit. */
  } else {
    /* Device healthy: only read when INT fired (PB14 falling edge). */
    if (!s_pca_irq_pending)
      return;
  }

  s_pca_irq_pending = 0U;
  if (PCA9555_ReadInputs(&s_pca) == HAL_OK) {
    g_pca9555_raw     = s_pca.raw;
    s_pca_fail_streak = 0U;
  } else {
    dbg_pca_timeout_count++;
    if (s_pca_fail_streak < 0xFFU) s_pca_fail_streak++;
    s_pca_retry_ms = HAL_GetTick();
    /* Release every key in the cache: reads happen on INT edges, so a failed
     * read may have landed on a release edge and a stale "pressed" bit would
     * fire HOLD/REPEAT events for the whole retry window.  A key genuinely
     * still held re-registers on the recovery read (reinit fall-through
     * refreshes the full port state).  PTT is direct GPIO — unaffected. */
    g_pca9555_raw = 0xFFFFU;
  }
#else
  dbg_pca_disabled_hits++;
#endif
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Input_F4_IsPressed  — live read for blocking scan-abort loops
 * ════════════════════════════════════════════════════════════════════════════ */

bool Input_F4_IsPressed(void)
{
#if HAS_PCA9555
  /* Respect the fault latch: no I2C while s_pca.ok == false.  The blocking
   * sweep loops in sdr_scan.c call this every few ms; with a dead chip each
   * call would cost 2 HAL timeouts (~20 ms) and bypass the skip/retry
   * machinery above.  Input_Scan() re-probes the chip once the sweep
   * returns control to the main loop. */
  if (!s_pca.ok)
    return false;                    /* fail-safe: treat as released */
  dbg_pca_read_attempts++;
  if (PCA9555_ReadInputs(&s_pca) != HAL_OK) {
    dbg_pca_timeout_count++;
    /* Read failed (ok is now false).  s_pca.raw kept its previous value —
     * if the failure landed while F4 was held, that stale "pressed" bit
     * would wedge the wait-for-release loops in sdr_scan.c forever.
     * Report released; worst case is one missed abort press. */
    return false;
  }
  /* Active-low: bit = 0 means pressed */
  return !((s_pca.raw >> PCA_BIT_F4) & 1U);
#else
  dbg_pca_disabled_hits++;
  return false;   /* chip absent — F4 never pressed */
#endif
}
