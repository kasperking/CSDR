/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tx_dc_trim.h
  * @brief   QSE carrier-leak null — DC trim của driver OPA4350 bằng PWM TIM1
  *
  *  Carrier leak (LO) của QSE cần offset DC ở baseband, mà codec → driver đi
  *  qua tụ chặn C21/C22 nên DC cộng vào mẫu I/Q không tới được mixer.  Phần
  *  cứng (sheet 6, .epro 2026-10-25) bơm offset vào IN+ của tầng đảo đầu:
  *
  *    PA8 TIM1_CH1 (I_BIAS) ─10k─┬─100k─┬─ U13.5 INB+  (tầng B: OUTB = −I)
  *    PA9 TIM1_CH2 (Q_BIAS) ─10k─┤      ├─ U13.3 INA+  (tầng A: OUTA = −Q)
  *                              1µF    1µF ─10k─ TX_BIAS
  *
  *  Tầng C/D đảo lại từ OUTB/OUTA với IN+ = TX_BIAS → cặp nút mux nhận +δ/−δ
  *  vi sai, common-mode = 0 (không kéo TX_BIAS nguồn 5 kΩ).  Tụ 22 µF sau
  *  op-amp (C64/C66/C68/C71) PHẢI là 0 Ω — nếu lắp tụ, trim không có tác dụng.
  *
  *  δ mỗi nút ≈ (V_pwm − 2.5 V) × 10k/120k.  Zero ≈ duty 2.5/3.3 = code 6206
  *  trên ARR 8191 (TIM1 ≈ 240 MHz → PWM ≈ 29.3 kHz, lọc 2 cực ~16/18 Hz).
  *  1 code ≈ 0.4 mV PWM ≈ 34 µV tại nút mux.  Zero thật lệch theo Vos op-amp
  *  và tỉ lệ 3V3/5VA — cal per band bù hết.
  *
  *  Duty giữ CỐ ĐỊNH qua RX/TX (τ lọc ~18 ms > thời gian key — đổi lúc PTT
  *  sẽ thành click carrier); chỉ đổi khi đổi band hoặc khi chỉnh trong Cal.
  *  Thuần phần cứng timer, ghi CCR từ main loop — không ISR, không DMA.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TX_DC_TRIM_H
#define __TX_DC_TRIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TXDC_ARR        8191U   /* 13-bit PWM */
#define TXDC_ZERO_CODE  6206    /* 2.5 V / 3.3 V × 8192 — TX_BIAS danh định */
#define TXDC_TRIM_MAX   1985    /* ZERO + 1985 = ARR (3.3 V); đối xứng ± */

void TxDcTrim_Init(void);
/* Đặt trim (code lệch so với TXDC_ZERO_CODE, kẹp ±TXDC_TRIM_MAX). */
void TxDcTrim_Set(int16_t i, int16_t q);
/* Áp g_tx_iq_cal[band].dc_i/dc_q — chỉ ghi CCR khi giá trị đổi.  Gọi mỗi
 * vòng main loop được; band ngoài dải → bỏ qua. */
void TxDcTrim_FollowBand(uint8_t band);

#ifdef __cplusplus
}
#endif

#endif /* __TX_DC_TRIM_H */
