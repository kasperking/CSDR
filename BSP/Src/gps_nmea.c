/**
  ******************************************************************************
  * @file    gps_nmea.c
  * @brief   GPS NMEA time sync — USART2 RX (PD6) → RTC.
  *
  *  ISR đẩy byte vào ring 256 B; GPS_NMEA_Poll (main-loop) ráp câu, kiểm
  *  checksum, parse $GxRMC.  Status 'A' (có fix) + giờ UTC hợp lệ →
  *  so sánh vòng tròn với RTC: lệch ≥ 2 s hoặc RTC chưa từng cài thì
  *  RTC_Clock_SetTime(UTC + utc_offset_h).
  *
  *  Sai số hệ thống: câu RMC được GPS phát ra ~0.1-0.5 s SAU sườn PPS của
  *  giây nó mô tả, nên RTC trễ hơn UTC thật tối đa ~0.5 s.  Đủ cho đồng hồ
  *  hiển thị; FT8 auto-DT tự bù phần dư sub-second.  Ngưỡng 2 s tránh
  *  re-sync dao động do độ trễ này (steady-state diff = 0..1 s).
  ******************************************************************************
  */
#include "gps_nmea.h"
#include "rtc_clock.h"
#include "stm32h7xx_hal.h"

#define GPSNMEA_IRQ_PRIO   12U        /* dưới audio DMA (0) — như gps_cal   */
#define GPSNMEA_BAUD       9600U      /* mặc định NEO-6M/7M/8M/M10          */
#define GPSNMEA_KERNEL_HZ  120000000U /* USART234578 = rcc_pclk1 (theo .ioc) */
#define GPSNMEA_LINE_MAX   90U        /* NMEA tối đa 82 ký tự               */
#define GPSNMEA_STALE_MS   3000U      /* không byte/fix > 3 s → NO_DATA/NO_FIX */
#define GPSNMEA_DIFF_MIN_S 2          /* lệch RTC tối thiểu mới ghi lại     */
#define GPSNMEA_SET_GAP_MS 5000U      /* hai lần ghi RTC cách nhau ≥ 5 s    */

/* ── RX ring (ISR ghi head, Poll ghi tail; index uint8_t wrap tự nhiên) ── */
static volatile uint8_t s_rb[256];
static volatile uint8_t s_head;
static volatile uint8_t s_tail;   /* ISR đọc để check full */

/* ── Parser / sync state (chỉ main-loop) ─────────────────────────────────── */
static char     s_line[GPSNMEA_LINE_MAX];
static uint8_t  s_len;
static bool     s_in_line;

static int32_t  s_utc_off_h;      /* RTC = UTC + offset (giờ nguyên)        */
static uint8_t  s_utc_h, s_utc_m, s_utc_s;
static uint32_t s_last_byte_ms, s_last_fix_ms, s_last_set_ms;
static bool     s_seen_byte, s_seen_fix;

/* Counters — volatile để xem trực tiếp trong debugger */
volatile uint32_t dbg_gps_nmea_sentences;
volatile uint32_t dbg_gps_nmea_cksum_err;
volatile uint32_t dbg_gps_nmea_syncs;
volatile int32_t  dbg_gps_nmea_diff_s;

/* ── ISR: chỉ buffer byte, lỗi line (ORE/FE/NE) xóa cờ rồi đọc tiếp ─────── */
void USART2_IRQHandler(void)
{
  uint32_t isr = USART2->ISR;
  if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF
                | USART_ICR_NECF  | USART_ICR_PECF;
  }
  while (USART2->ISR & USART_ISR_RXNE_RXFNE) {
    uint8_t b    = (uint8_t)USART2->RDR;
    uint8_t next = (uint8_t)(s_head + 1U);
    if (next != s_tail) {             /* đầy → drop byte, parser tự phục hồi */
      s_rb[s_head] = b;
      s_head = next;
    }
  }
}

/* ── API ─────────────────────────────────────────────────────────────────── */
void GPS_NMEA_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef gi = {0};
  gi.Pin       = GPIO_PIN_6;              /* USART2_RX ← GPS TX */
  gi.Mode      = GPIO_MODE_AF_PP;
  gi.Alternate = GPIO_AF7_USART2;
  gi.Speed     = GPIO_SPEED_FREQ_LOW;
  gi.Pull      = GPIO_PULLUP;             /* không GPS → idle cao, im lặng */
  HAL_GPIO_Init(GPIOD, &gi);

  __HAL_RCC_USART2_CLK_ENABLE();
  USART2->CR1 = 0U;                       /* UE=0 trước khi cấu hình        */
  USART2->CR2 = 0U;
  USART2->CR3 = 0U;
  USART2->BRR = GPSNMEA_KERNEL_HZ / GPSNMEA_BAUD;   /* 12500 — chia chẵn   */
  USART2->ICR = 0xFFFFFFFFU;
  USART2->CR1 = USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;

  HAL_NVIC_SetPriority(USART2_IRQn, GPSNMEA_IRQ_PRIO, 0U);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void GPS_NMEA_SetUtcOffset(int32_t hours)
{
  if (hours < -12) hours = -12;
  if (hours >  14) hours =  14;
  s_utc_off_h = hours;
}

int32_t GPS_NMEA_GetUtcOffset(void)
{
  return s_utc_off_h;
}

/* ── RTC sync: so sánh vòng tròn seconds-of-day, chỉ ghi khi đáng ─────────── */
static void nmea_try_sync(void)
{
  int32_t sod = (int32_t)s_utc_h * 3600 + (int32_t)s_utc_m * 60 + (int32_t)s_utc_s
              + s_utc_off_h * 3600;
  sod = ((sod % 86400) + 86400) % 86400;

  uint8_t rh, rm, rs;
  RTC_Clock_GetTime(&rh, &rm, &rs);
  int32_t diff = sod - ((int32_t)rh * 3600 + (int32_t)rm * 60 + (int32_t)rs);
  if (diff >  43200) diff -= 86400;
  if (diff < -43200) diff += 86400;
  dbg_gps_nmea_diff_s = diff;

  if (RTC_Clock_IsSet() && diff < GPSNMEA_DIFF_MIN_S && diff > -GPSNMEA_DIFF_MIN_S)
    return;
  uint32_t now = HAL_GetTick();
  if (s_last_set_ms != 0U && (now - s_last_set_ms) < GPSNMEA_SET_GAP_MS)
    return;

  RTC_Clock_SetTime((uint8_t)(sod / 3600),
                    (uint8_t)((sod % 3600) / 60),
                    (uint8_t)(sod % 60));
  s_last_set_ms = now;
  dbg_gps_nmea_syncs++;
}

/* ── Một câu hoàn chỉnh "$....*hh" (không CR/LF) ─────────────────────────── */
static void nmea_line(void)
{
  /* Checksum: XOR các byte giữa '$' và '*' */
  if (s_len < 9U || s_line[0] != '$') { dbg_gps_nmea_cksum_err++; return; }
  uint8_t star = 0U;
  for (uint8_t i = (uint8_t)(s_len - 1U); i > 0U; i--) {
    if (s_line[i] == '*') { star = i; break; }
  }
  if (star == 0U || (uint8_t)(s_len - star) != 3U) { dbg_gps_nmea_cksum_err++; return; }
  uint8_t cs = 0U;
  for (uint8_t i = 1U; i < star; i++) cs ^= (uint8_t)s_line[i];
  uint8_t rx = 0U;
  for (uint8_t i = 1U; i <= 2U; i++) {
    char c = s_line[star + i];
    rx <<= 4;
    if      (c >= '0' && c <= '9') rx |= (uint8_t)(c - '0');
    else if (c >= 'A' && c <= 'F') rx |= (uint8_t)(c - 'A' + 10);
    else { dbg_gps_nmea_cksum_err++; return; }
  }
  if (cs != rx) { dbg_gps_nmea_cksum_err++; return; }

  dbg_gps_nmea_sentences++;

  /* Chỉ quan tâm RMC (talker 2 ký tự bất kỳ): $GxRMC,hhmmss.ss,A,... */
  if (star < 6U || s_line[3] != 'R' || s_line[4] != 'M' || s_line[5] != 'C')
    return;

  /* Field 1 = time, field 2 = status; các field sau không cần */
  uint8_t f1 = 7U;                       /* sau "$GxRMC,"                   */
  if (s_line[6] != ',') return;
  /* hhmmss — 6 chữ số liền (phần .sss bỏ qua) */
  for (uint8_t i = 0U; i < 6U; i++) {
    if (f1 + i >= star || s_line[f1 + i] < '0' || s_line[f1 + i] > '9')
      return;                            /* time rỗng (chưa có giờ) → bỏ    */
  }
  uint8_t h = (uint8_t)((s_line[f1]     - '0') * 10 + (s_line[f1 + 1U] - '0'));
  uint8_t m = (uint8_t)((s_line[f1 + 2U] - '0') * 10 + (s_line[f1 + 3U] - '0'));
  uint8_t s = (uint8_t)((s_line[f1 + 4U] - '0') * 10 + (s_line[f1 + 5U] - '0'));
  if (h > 23U || m > 59U || s > 60U) return;

  /* Nhảy tới field 2 (status) */
  uint8_t i = f1;
  while (i < star && s_line[i] != ',') i++;
  if (i + 1U >= star || s_line[i + 1U] != 'A')
    return;                              /* 'V' = chưa fix → giờ chưa tin   */

  s_utc_h = h; s_utc_m = m; s_utc_s = s;
  s_seen_fix    = true;
  s_last_fix_ms = HAL_GetTick();
  nmea_try_sync();
}

void GPS_NMEA_Poll(void)
{
  if (s_tail == s_head) return;
  s_last_byte_ms = HAL_GetTick();
  s_seen_byte    = true;

  while (s_tail != s_head) {
    char b = (char)s_rb[s_tail];
    s_tail++;
    if (b == '$') {                      /* luôn restart tại '$'            */
      s_line[0] = '$';
      s_len     = 1U;
      s_in_line = true;
    } else if (!s_in_line) {
      /* rác giữa các câu — bỏ */
    } else if (b == '\r' || b == '\n') {
      s_in_line = false;
      if (s_len > 1U) nmea_line();
      s_len = 0U;
    } else if (s_len < GPSNMEA_LINE_MAX - 1U) {
      s_line[s_len++] = b;
    } else {                             /* câu quá dài → hỏng, chờ '$' mới */
      s_in_line = false;
      s_len     = 0U;
    }
  }
}

void GPS_NMEA_GetStatus(GPSNMEA_Status_t *out)
{
  uint32_t now = HAL_GetTick();
  out->utc_h       = s_utc_h;
  out->utc_m       = s_utc_m;
  out->utc_s       = s_utc_s;
  out->sentences   = dbg_gps_nmea_sentences;
  out->cksum_err   = dbg_gps_nmea_cksum_err;
  out->rtc_syncs   = dbg_gps_nmea_syncs;
  out->last_diff_s = dbg_gps_nmea_diff_s;
  if (!s_seen_byte || (now - s_last_byte_ms) > GPSNMEA_STALE_MS)
    out->state = GPSNMEA_NO_DATA;
  else if (!s_seen_fix || (now - s_last_fix_ms) > GPSNMEA_STALE_MS)
    out->state = GPSNMEA_NO_FIX;
  else
    out->state = GPSNMEA_OK;
}
