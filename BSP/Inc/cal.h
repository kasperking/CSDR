#ifndef __CAL_H
#define __CAL_H
#ifdef __cplusplus
extern "C" {
#endif

#include "st7789.h"
#include <stdint.h>

/* cal.h – Calibration screen
 *
 *  Vào: chọn "Calibration" từ menu chính
 *  Ra:  F4 = lưu & thoát, MENU = thoát không lưu
 *
 *  Màn hình chia 4 tham số có thể chỉnh bằng encoder:
 *   1. XTAL correction (ppm): ±200ppm → bù drift tần số
 *   2. IQ Gain balance  (%):  ±20%    → cân bằng kênh I/Q
 *   3. IQ Phase balance (°):  ±20°    → loại bỏ image
 *   4. Audio level     (dB):  ±30dB   → điều chỉnh mức vào
 */

typedef struct {
  int32_t xtal_ppm;       /* XTAL correction */
  int16_t iq_gain_tenth;  /* IQ gain ×10 (150 = +15.0%) */
  int16_t iq_phase_tenth; /* IQ phase ×10 (0 = 0.0°) */
  int16_t audio_cal_db;   /* Audio level dB */
} Cal_Params_t;

/* Run calibration screen. Blocks until user exits.
 * params: current values (modified in-place if saved) */
bool Cal_Run(ST7789_Handle_t *lcd, Cal_Params_t *params);

#ifdef __cplusplus
}
#endif
#endif /* __CAL_H */
