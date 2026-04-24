/* USER CODE BEGIN Header */
/**
  * @file  diag.c (STUB - flash optimization)
  * @brief Diagnostic screen stubbed out to save ~4-6KB flash.
  *
  *  Khi mày muốn khôi phục diag UI, replace file này với bản gốc.
  *  API (Diag_Run) giữ nguyên nên csdr_app.c không cần sửa.
  */
/* USER CODE END Header */

#include "diag.h"

void Diag_Run(ST7789_Handle_t *lcd)
{
  (void)lcd;
  /* No-op stub. Real diagnostic screen removed to fit FLASH. */
}
