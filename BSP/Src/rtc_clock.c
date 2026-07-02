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
  uint32_t bdcr = RCC->BDCR;
  bool lse_rdy  = ((bdcr & RCC_BDCR_LSERDY) != 0U);
  bool rtc_en   = ((bdcr & RCC_BDCR_RTCEN)  != 0U);
  bool sel_lse  = ((bdcr & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_0);

  if (lse_rdy && rtc_en && sel_lse) {
    /* RTC already running — just enable bypass shadow for direct reads */
    rtc_unlock();
    SET_BIT(RTC->CR, RTC_CR_BYPSHAD);
    rtc_lock();
    s_rtc_ok = true;
    return;
  }

  /* ── First-time configuration ── */

  /* Backup domain reset (required to change RTCSEL) */
  SET_BIT(RCC->BDCR, RCC_BDCR_BDRST);
  __DSB();
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_BDRST);
  __DSB();

  /* Re-enable LSE (backup domain reset turns it off) */
  SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
  uint32_t t = 1000000U;
  while (!READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY) && --t) {}
  if (t == 0U) return;  /* LSE failed to start — leave s_rtc_ok = false */

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
