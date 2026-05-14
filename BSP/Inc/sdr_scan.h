/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdr_scan.h
  * @brief   SWR Scan mode – sweep frequency, measure SWR per step, plot result
  *
  *  Gọi SWR_Scan_Run() từ menu ACTION callback.
  *  Hàm blocking: chiếm toàn bộ vùng spectrum+waterfall (Y=62..239).
  *  Thoát bằng F4.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SDR_SCAN_H
#define __SDR_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Scan parameters */
#define SCAN_SPAN_HZ      200000UL   /* Half-span each side: ±200 kHz        */
#define SCAN_STEP_HZ       10000UL   /* Frequency step per point             */
#define SCAN_TX_SETTLE_MS     30U    /* Wait after TX on before reading SWR  */
#define SCAN_MAX_POINTS      201U    /* Max result buffer (400kHz/10kHz + 1) */

/**
  * @brief  Run SWR scan (blocking).
  *         Sweeps ±SCAN_SPAN_HZ around current frequency in SCAN_STEP_HZ steps.
  *         Keys TX briefly at each step, reads SWR, plots result.
  *         Returns when user presses F4.
  */
void SWR_Scan_Run(void);

#ifdef __cplusplus
}
#endif
#endif /* __SDR_SCAN_H */
