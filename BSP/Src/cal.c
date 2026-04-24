/* USER CODE BEGIN Header */
/**
  * @file  cal.c (STUB - flash optimization)
  * @brief Calibration screen stubbed out to save ~3-5KB flash.
  *
  *  Khi mày muốn khôi phục calibration UI, replace file này với bản gốc.
  *  API (Cal_Run) giữ nguyên nên csdr_app.c không cần sửa.
  *
  *  Stub luôn trả về false (user huỷ, không lưu).
  *  Params được giữ nguyên (không bị modify).
  */
/* USER CODE END Header */

#include "cal.h"

bool Cal_Run(ST7789_Handle_t *lcd, Cal_Params_t *params)
{
  (void)lcd;
  (void)params;
  return false;   /* "user cancelled" — no parameter changes */
}
