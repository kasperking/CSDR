/**
  ******************************************************************************
  * @file    gps_nmea.h
  * @brief   GPS NMEA time sync — USART2 RX (PD6) → RTC.
  *
  *  Nhận câu NMEA từ module GPS (9600 8N1, chỉ cần dây TX của GPS):
  *
  *    GPS TX ──► PD6 (USART2_RX, AF7)
  *
  *  Parse $GxRMC (talker bất kỳ: GP/GN/BD/GA...), chỉ nhận khi status = 'A'
  *  (có fix).  Giờ UTC + utc_offset_h → so với RTC; lệch ≥ 2 s (hoặc RTC
  *  chưa từng được cài) thì RTC_Clock_SetTime.  Đồng hồ header và FT8 slot
  *  timing dùng thẳng RTC nên không cần wiring thêm.
  *
  *  ISR chỉ đẩy byte vào ring buffer (vài chục cycle, priority 12 — dưới
  *  audio/USB).  Toàn bộ parse chạy trong GPS_NMEA_Poll từ main loop.
  *  Không có GPS: PD6 pull-up giữ idle mức cao → không có byte, Poll rẻ.
  ******************************************************************************
  */
#ifndef __GPS_NMEA_H
#define __GPS_NMEA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  GPSNMEA_NO_DATA = 0,  /*!< >3 s không có byte nào (chưa cắm GPS)        */
  GPSNMEA_NO_FIX,       /*!< có câu NMEA nhưng RMC chưa có fix (status V) */
  GPSNMEA_OK            /*!< RMC hợp lệ trong 3 s gần nhất                */
} GPSNMEA_State_t;

typedef struct {
  GPSNMEA_State_t state;
  uint8_t   utc_h, utc_m, utc_s;  /*!< giờ UTC của RMC hợp lệ gần nhất     */
  uint32_t  sentences;            /*!< số câu qua được checksum            */
  uint32_t  cksum_err;            /*!< số câu hỏng checksum/format         */
  uint32_t  rtc_syncs;            /*!< số lần đã ghi RTC                   */
  int32_t   last_diff_s;          /*!< lệch GPS−RTC (giây, ±) lần so gần nhất */
} GPSNMEA_Status_t;

/** Bật USART2 RX trên PD6 + IRQ.  Gọi một lần lúc boot (sau RTC_Clock_Init). */
void GPS_NMEA_Init(void);

/** Bơm parser + auto-sync RTC.  Gọi mỗi vòng main loop / app loop. */
void GPS_NMEA_Poll(void);

/** Múi giờ hiển thị: RTC = UTC + offset (giờ nguyên, -12..+14, clamp).
 *  Đổi offset không ghi RTC ngay — câu RMC kế tiếp (~1 s) sẽ re-sync. */
void    GPS_NMEA_SetUtcOffset(int32_t hours);
int32_t GPS_NMEA_GetUtcOffset(void);

/** Trạng thái + counters (debug/UI). */
void GPS_NMEA_GetStatus(GPSNMEA_Status_t *out);

#ifdef __cplusplus
}
#endif
#endif /* __GPS_NMEA_H */
