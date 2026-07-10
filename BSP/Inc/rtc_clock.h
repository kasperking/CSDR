#ifndef __RTC_CLOCK_H
#define __RTC_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * rtc_clock.h — bare-metal STM32H7 RTC driver (LSE, 24-hour BCD)
 *
 *  Clock source : LSE 32.768 kHz (already enabled in CubeMX SystemClock_Config)
 *  Prescalers   : PREDIV_A=127, PREDIV_S=255 → (128×256)=32768 → 1 Hz tick
 *  Persistence  : RTC keeps time across soft-reset and power-off when VBAT
 *                 is connected; BKP1R magic tracks whether time has been set.
 *
 *  Call RTC_Clock_Init() once at boot (before CSDR_Init flash restore).
 *  Call RTC_Clock_SetTime() to set time (also marks BKP1R as valid).
 *  Call RTC_Clock_GetTime() to read current HH:MM:SS from RTC counters.
 *  Call RTC_Clock_IsSet() to check if a user-set time is available.
 */

void RTC_Clock_Init(void);
void RTC_Clock_SetTime(uint8_t h, uint8_t m, uint8_t s);
void RTC_Clock_GetTime(uint8_t *h, uint8_t *m, uint8_t *s);
bool RTC_Clock_IsSet(void);

/* Shift the running calendar by shift_ms (positive = advance).  Whole-second
 * resolution: the new time is rounded to the nearest second (sub-second
 * counter restarts at 0), so the returned "applied" shift differs from the
 * request by up to ±500 ms — the caller keeps that residual if it matters
 * (FT8 auto-DT does).  Returns 0 when the RTC is not running.
 * Does NOT touch BKP1R (user-set flag stays as-is). */
int32_t RTC_Clock_ShiftMs(int32_t shift_ms);

#ifdef __cplusplus
}
#endif
#endif /* __RTC_CLOCK_H */
