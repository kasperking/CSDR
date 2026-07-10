#include "rtc_clock.h"
#include "stm32h7xx_hal.h"

/* BKP1R magic — indicates that SetTime has been called at least once */
#define RTC_CLOCK_MAGIC  0x52544300UL  /* "RTC\0" */

/* Track whether init succeeded (LSE started, RTC running) */
static bool s_rtc_ok = false;

/* ── helpers ─────────────────────────────────────────────────────────────── */

static void rtc_unlock(void)
{
  RTC->WPR = 0xCAU;
  RTC->WPR = 0x53U;
}

static void rtc_lock(void)
{
  RTC->WPR = 0xFFU;
}

/* Enter RTC init mode; returns false on timeout (LSE not running). */
static bool rtc_enter_init(void)
{
  SET_BIT(RTC->ISR, RTC_ISR_INIT);
  uint32_t t = 200000U;
  while (!READ_BIT(RTC->ISR, RTC_ISR_INITF) && --t) {}
  return (t != 0U);
}

static void rtc_exit_init(void)
{
  CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);
}

/* ── RTC_Clock_Init ──────────────────────────────────────────────────────── */
void RTC_Clock_Init(void)
{
  /* Backup domain write access — may already be set by main.c DFU check */
  SET_BIT(PWR->CR1, PWR_CR1_DBP);
  SET_BIT(RCC->APB4ENR, RCC_APB4ENR_RTCAPBEN);
  __DSB();

  /* Check if RTC is already correctly configured (LSE selected + running) */
  uint32_t bdcr    = RCC->BDCR;
  bool     lse_rdy = ((bdcr & RCC_BDCR_LSERDY) != 0U);
  bool     rtc_en  = ((bdcr & RCC_BDCR_RTCEN)  != 0U);
  uint32_t rtc_sel = (bdcr & RCC_BDCR_RTCSEL);

  if (lse_rdy && rtc_en && (rtc_sel == RCC_BDCR_RTCSEL_0)) {
    /* RTC already running — just enable bypass shadow for direct reads */
    rtc_unlock();
    SET_BIT(RTC->CR, RTC_CR_BYPSHAD);
    rtc_lock();
    s_rtc_ok = true;
    return;
  }

  /* ── First-time configuration ── */

  /* RTCSEL is write-once only after it has been written: reset value 00 means
   * it can still be set directly.  Only force a backup-domain reset when it
   * already holds a wrong non-zero source — that reset also stops LSE. */
  if ((rtc_sel != 0U) && (rtc_sel != RCC_BDCR_RTCSEL_0)) {
    SET_BIT(RCC->BDCR, RCC_BDCR_BDRST);
    __DSB();
    CLEAR_BIT(RCC->BDCR, RCC_BDCR_BDRST);
    __DSB();
  }

  /* LSE normally still runs from SystemClock_Config; it only needs a restart
   * after the backup-domain reset above.  LSE crystals take hundreds of ms
   * (up to seconds) to start, so wait on the HAL tick, not a spin count. */
  if (!READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY)) {
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
    uint32_t t0 = HAL_GetTick();
    while (!READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY)) {
      if ((HAL_GetTick() - t0) > 5000U)
        return;  /* LSE failed to start — leave s_rtc_ok = false */
    }
  }

  /* Select LSE as RTC clock source */
  MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL, RCC_BDCR_RTCSEL_0);

  /* Enable RTC */
  SET_BIT(RCC->BDCR, RCC_BDCR_RTCEN);
  __DSB();

  /* Unlock + init mode */
  rtc_unlock();
  if (!rtc_enter_init()) { rtc_lock(); return; }

  /* PREDIV_A=127, PREDIV_S=255 → (128×256)=32768 Hz → 1 Hz */
  RTC->PRER = (127U << 16U) | 255U;

  /* 24-hour format, bypass shadow registers */
  CLEAR_BIT(RTC->CR, RTC_CR_FMT);
  SET_BIT(RTC->CR,   RTC_CR_BYPSHAD);

  /* Start at 00:00:00, date 01-01-01 (arbitrary, unused) */
  RTC->TR = 0U;
  RTC->DR = 0x00210101U;

  rtc_exit_init();
  rtc_lock();

  /* BKP1R = 0: time not yet set by user */
  RTC->BKP1R = 0U;

  s_rtc_ok = true;
}

/* ── RTC_Clock_IsSet ─────────────────────────────────────────────────────── */
bool RTC_Clock_IsSet(void)
{
  return s_rtc_ok && (RTC->BKP1R == RTC_CLOCK_MAGIC);
}

/* ── RTC_Clock_SetTime ───────────────────────────────────────────────────── */
void RTC_Clock_SetTime(uint8_t h, uint8_t m, uint8_t s)
{
  if (!s_rtc_ok) return;

  /* Pack BCD time register:
   *   [22:20] HT  [19:16] HU  [14:12] MNT  [11:8] MNU  [6:4] ST  [3:0] SU */
  uint32_t tr = (((uint32_t)(h / 10U)) << 20U) | (((uint32_t)(h % 10U)) << 16U)
              | (((uint32_t)(m / 10U)) << 12U) | (((uint32_t)(m % 10U)) <<  8U)
              | (((uint32_t)(s / 10U)) <<  4U) | ((uint32_t)(s % 10U));

  rtc_unlock();
  if (!rtc_enter_init()) { rtc_lock(); return; }
  RTC->TR = tr;
  rtc_exit_init();
  rtc_lock();

  /* Mark as user-set (survives soft reset; cleared by backup domain reset) */
  RTC->BKP1R = RTC_CLOCK_MAGIC;
}

/* ── RTC_Clock_ShiftMs ───────────────────────────────────────────────────── */
int32_t RTC_Clock_ShiftMs(int32_t shift_ms)
{
  if (!s_rtc_ok || shift_ms == 0) return 0;

  /* Snapshot current time-of-day in ms (BYPSHAD: direct, re-read for
   * SSR/TR coherency; SSR is a 256 Hz down-counter, PREDIV_S=255) */
  uint32_t ss, tr;
  do {
    ss = RTC->SSR;
    tr = RTC->TR;
  } while (ss != RTC->SSR);
  (void)RTC->DR;
  uint32_t h = (((tr >> 20) & 0x3U) * 10U) + ((tr >> 16) & 0xFU);
  uint32_t m = (((tr >> 12) & 0x7U) * 10U) + ((tr >>  8) & 0xFU);
  uint32_t s = (((tr >>  4) & 0x7U) * 10U) + ( tr        & 0xFU);
  int64_t old_ms = ((int64_t)(h * 3600U + m * 60U + s) * 1000)
                 + (int64_t)(((255U - (ss & 0xFFU)) * 1000U) >> 8U);

  /* Target rounded to the nearest whole second (sub-seconds restart at 0
   * when leaving init mode, so only whole seconds can be written exactly) */
  int64_t target = old_ms + shift_ms;
  int64_t sec    = (target + 500) / 1000;          /* rounded             */
  int64_t new_ms = sec * 1000;
  int64_t sod    = ((sec % 86400) + 86400) % 86400; /* seconds-of-day     */

  uint32_t nh = (uint32_t)(sod / 3600);
  uint32_t nm = (uint32_t)((sod % 3600) / 60);
  uint32_t ns = (uint32_t)(sod % 60);
  uint32_t ntr = ((nh / 10U) << 20U) | ((nh % 10U) << 16U)
               | ((nm / 10U) << 12U) | ((nm % 10U) <<  8U)
               | ((ns / 10U) <<  4U) |  (ns % 10U);

  rtc_unlock();
  if (!rtc_enter_init()) { rtc_lock(); return 0; }
  RTC->TR = ntr;
  rtc_exit_init();
  rtc_lock();

  /* Applied shift = new − old (may differ from request by ±500 ms rounding) */
  return (int32_t)(new_ms - old_ms);
}

/* ── RTC_Clock_GetTime ───────────────────────────────────────────────────── */
void RTC_Clock_GetTime(uint8_t *h, uint8_t *m, uint8_t *s)
{
  if (!s_rtc_ok) { *h = 0U; *m = 0U; *s = 0U; return; }

  /* With BYPSHAD set, RTC->TR reads directly from the counters — no wait needed.
   * Reading TR first, then DR releases any potential double-read lock. */
  uint32_t tr = RTC->TR;
  (void)RTC->DR;

  *h = (uint8_t)(((tr >> 20U) & 0x3U) * 10U + ((tr >> 16U) & 0xFU));
  *m = (uint8_t)(((tr >> 12U) & 0x7U) * 10U + ((tr >>  8U) & 0xFU));
  *s = (uint8_t)(((tr >>  4U) & 0x7U) * 10U + ( tr         & 0xFU));
}
