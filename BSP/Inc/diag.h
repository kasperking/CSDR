/* USER CODE BEGIN Header */
/**
  * @file  diag.h
  * @brief Diagnostic screen – kiểm tra các thành phần phần cứng
  *
  *  Gọi từ main loop khi nhấn giữ MENU + F4 cùng lúc (3 giây).
  *  Hiển thị trên LCD toàn bộ trạng thái hệ thống.
  */
/* USER CODE END Header */

#ifndef __DIAG_H
#define __DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "st7789.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Kết quả kiểm tra từng module ── */
typedef enum {
  DIAG_UNKNOWN = 0,
  DIAG_OK,
  DIAG_FAIL,
  DIAG_WARN,
} DiagStatus_t;

typedef struct {
  /* SI5351 VFO */
  DiagStatus_t si5351;
  uint32_t     si5351_freq_hz;       /* Tần số đang set */
  bool         si5351_clk0_on;
  bool         si5351_clk2_on;

  /* WM8731 Codec */
  DiagStatus_t wm8731;
  int16_t      codec_left_db;        /* Volume hiện tại */
  int16_t      codec_right_db;

  /* USB */
  DiagStatus_t usb_composite;
  bool         usb_cdc_active;       /* CDC COM port open */
  bool         usb_audio_streaming;  /* ISO IN đang chạy */

  /* W25Q128 Flash */
  DiagStatus_t flash;
  uint32_t     flash_jedec_id;       /* JEDEC ID đọc được */

  /* ADC / Analog */
  DiagStatus_t analog;
  float        temp_c;               /* NTC temperature */
  float        voltage_v;            /* Supply voltage */
  float        alc_mv;               /* ALC voltage */

  /* SAI Audio DMA */
  DiagStatus_t sai;
  uint32_t     sai_rx_frames;        /* Frames received */
  uint32_t     usb_audio_underrun;   /* Ring buffer underruns */

  /* BPF/LPF */
  DiagStatus_t bpf;
  uint8_t      bpf_band;
  uint8_t      lpf_band;
  uint8_t      att_db;
} Diag_Result_t;

/* ── API ── */
void Diag_Run(ST7789_Handle_t *lcd);
bool Diag_IsActive(void);
void Diag_Process(void);

#ifdef __cplusplus
}
#endif
#endif /* __DIAG_H */
