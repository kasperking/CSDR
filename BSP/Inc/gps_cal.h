/**
  ******************************************************************************
  * @file    gps_cal.h
  * @brief   GPS 1PPS frequency calibration — TIM3 reciprocal counter.
  *
  *  Đo sai số thạch anh SI5351 bằng cách đếm CLK2 (= XTAL passthrough,
  *  bật qua SI5351_SetCalOutput) giữa hai sườn lên 1PPS của GPS:
  *
  *    SI5351 CLK2 ──► PB5  (TIM3_CH2, AF2)  → external clock mode 1 (TI2FP2)
  *    GPS 1PPS    ──► PC6  (TIM3_CH1, AF2)  → input capture (latch phần cứng)
  *
  *  Counter 16-bit mở rộng 32-bit bằng update IRQ (~380 Hz @ 25 MHz).
  *  Capture latch bằng phần cứng nên jitter ISR không lọt vào kết quả;
  *  độ phân giải ±1 count/giây = 0.04 ppm, trung bình N giây → chia N.
  *
  *  ISR chỉ vài chục cycle, priority 12 — không ảnh hưởng audio/USB
  *  pipeline (priority 0).  Ngoài phiên đo TIM3 tắt hoàn toàn.
  ******************************************************************************
  */
#ifndef __GPS_CAL_H
#define __GPS_CAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  GPSCAL_IDLE = 0,     /*!< chưa Start                                  */
  GPSCAL_NO_CLK,       /*!< không thấy CLK2 vào PB5 (kiểm tra dây/CLK2) */
  GPSCAL_NO_PPS,       /*!< CLK chạy nhưng >2.5 s không có xung 1PPS    */
  GPSCAL_SETTLING,     /*!< có PPS, đang chờ interval hợp lệ đầu tiên   */
  GPSCAL_MEASURING     /*!< đang tích lũy — secs/ppb hợp lệ             */
} GPSCal_State_t;

typedef struct {
  GPSCal_State_t state;
  uint32_t       secs;        /*!< số interval 1 s đã tích lũy           */
  int32_t        ppb;         /*!< sai số xtal (dương = chạy nhanh)      */
  uint32_t       last_count;  /*!< count interval gần nhất (debug)       */
} GPSCal_Result_t;

/** Bật TIM3 + GPIO và bắt đầu đo.  nominal_hz = tần số danh định của
 *  tín hiệu đưa vào PB5 (= g_si5351.xtal_hz khi dùng CLK2 passthrough). */
void GPSCal_Start(uint32_t nominal_hz);

/** Dừng đo, tắt TIM3/IRQ, trả PB5/PC6 về trạng thái mặc định. */
void GPSCal_Stop(void);

/** Đọc trạng thái + kết quả hiện tại (gọi từ main loop, không chặn). */
void GPSCal_Read(GPSCal_Result_t *out);

#ifdef __cplusplus
}
#endif
#endif /* __GPS_CAL_H */
