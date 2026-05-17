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
  if (PCA9555_ReadInputs(&s_pca) == HAL_OK)
    g_pca9555_raw = s_pca.raw;
  else
    dbg_pca_timeout_count++;
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
  dbg_pca_read_attempts++;
  if (PCA9555_ReadInputs(&s_pca) != HAL_OK)
    dbg_pca_timeout_count++;
  /* Active-low: bit = 0 means pressed */
  return !((s_pca.raw >> PCA_BIT_F4) & 1U);
#else
  dbg_pca_disabled_hits++;
  return false;   /* chip absent — F4 never pressed */
#endif
}
