/**
  ******************************************************************************
  * @file    gps_cal.c
  * @brief   GPS 1PPS frequency calibration — TIM3 reciprocal counter.
  *
  *  TIM3 đếm CLK2 (external clock mode 1 qua TI2FP2/PB5); mỗi sườn lên
  *  1PPS trên TI1/PC6 latch counter bằng phần cứng.  Hiệu hai lần latch
  *  liên tiếp = số chu kỳ CLK2 trong đúng 1 giây GPS.
  *
  *  Interval lệch quá ±0.5% danh định (mất PPS, mất CLK, glitch) làm
  *  reset bộ tích lũy — kết quả chỉ gồm các giây liên tục hợp lệ.
  ******************************************************************************
  */
#include "gps_cal.h"
#include "stm32h7xx_hal.h"

#define GPSCAL_IRQ_PRIO      12U    /* dưới audio DMA (0) và PA-OC EXTI (5) */
#define GPSCAL_PPS_TIMEOUT   2500U  /* ms không capture → NO_PPS            */

static volatile uint32_t s_ovf;         /* 16→32 bit extension               */
static volatile uint32_t s_prev_tick;   /* tick 32-bit của capture trước     */
static volatile bool     s_have_prev;
static volatile uint64_t s_sum;         /* tổng count các interval hợp lệ    */
static volatile uint32_t s_n;           /* số interval hợp lệ                */
static volatile uint32_t s_last_delta;  /* interval gần nhất (kể cả loại bỏ) */
static volatile uint32_t s_last_pps_ms; /* HAL_GetTick tại capture gần nhất  */

static uint32_t s_nom_hz;
static bool     s_running;
static uint32_t s_cnt_probe;            /* phát hiện CLK đứng (getter)       */

/* ── ISR ──────────────────────────────────────────────────────────────────
 * TIM3 chỉ có một vector chung UP+CC; xử lý UIF trước rồi mới capture, race
 * capture-sát-overflow xử lý bằng cờ UIF pending + nửa dải counter
 * (pattern chuẩn, giữ nguyên từ bản TIM1 hai vector). */
void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_UIF) {
    TIM3->SR = ~TIM_SR_UIF;
    s_ovf++;
  }

  if (TIM3->SR & TIM_SR_CC1IF) {
    uint16_t cc  = (uint16_t)TIM3->CCR1;      /* đọc CCR1 tự xóa CC1IF */
    uint32_t ovf = s_ovf;
    if ((TIM3->SR & TIM_SR_UIF) && cc < 0x8000U) { ovf++; }
    uint32_t tick = (ovf << 16) | cc;

    s_last_pps_ms = HAL_GetTick();

    if (!s_have_prev) {
      s_prev_tick = tick;
      s_have_prev = true;
      return;
    }

    uint32_t delta = tick - s_prev_tick;
    s_prev_tick  = tick;
    s_last_delta = delta;

    uint32_t tol = s_nom_hz / 200U;           /* ±0.5% */
    if (delta < s_nom_hz - tol || delta > s_nom_hz + tol) {
      s_sum = 0U;                             /* glitch/mất xung → làm lại */
      s_n   = 0U;
      return;
    }
    s_sum += delta;
    s_n++;
  }
}

/* ── API ─────────────────────────────────────────────────────────────────── */
void GPSCal_Start(uint32_t nominal_hz)
{
  if (s_running) { return; }
  s_nom_hz     = (nominal_hz != 0U) ? nominal_hz : 25000000UL;
  s_ovf        = 0U;
  s_have_prev  = false;
  s_sum        = 0U;
  s_n          = 0U;
  s_last_delta = 0U;
  s_last_pps_ms = HAL_GetTick();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gi = {0};
  gi.Mode      = GPIO_MODE_AF_PP;
  gi.Alternate = GPIO_AF2_TIM3;
  gi.Speed     = GPIO_SPEED_FREQ_LOW;
  gi.Pin       = GPIO_PIN_5;              /* PB5 = TIM3_CH2 ← SI5351 CLK2 */
  gi.Pull      = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gi);
  gi.Pin       = GPIO_PIN_6;              /* PC6 = TIM3_CH1 ← GPS 1PPS    */
  gi.Pull      = GPIO_PULLDOWN;           /* không GPS → không rung */
  HAL_GPIO_Init(GPIOC, &gi);

  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM3->CR1  = 0U;
  TIM3->CR2  = 0U;
  TIM3->PSC  = 0U;
  TIM3->ARR  = 0xFFFFU;
  /* CH2 = input TI2, không filter (25 MHz); CH1 = input TI1,
   * filter fDTS/32 N=8 (~1 µs) chống glitch trên đường PPS */
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC1F;
  TIM3->CCER  = TIM_CCER_CC1E;            /* capture sườn lên, CC2E off */
  /* External clock mode 1, nguồn TI2FP2 */
  TIM3->SMCR  = TIM_SMCR_TS_2 | TIM_SMCR_TS_1
              | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
  TIM3->CNT   = 0U;
  TIM3->SR    = 0U;
  TIM3->DIER  = TIM_DIER_CC1IE | TIM_DIER_UIE;

  HAL_NVIC_SetPriority(TIM3_IRQn, GPSCAL_IRQ_PRIO, 0U);
  HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  TIM3->CR1 = TIM_CR1_CEN;
  s_cnt_probe = TIM3->CNT;
  s_running   = true;
}

void GPSCal_Stop(void)
{
  if (!s_running) { return; }
  TIM3->CR1  = 0U;
  TIM3->DIER = 0U;
  TIM3->SR   = 0U;
  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  __HAL_RCC_TIM3_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);
  s_running = false;
}

void GPSCal_Read(GPSCal_Result_t *out)
{
  out->state      = GPSCAL_IDLE;
  out->secs       = 0U;
  out->ppb        = 0;
  out->last_count = 0U;
  if (!s_running) { return; }

  __disable_irq();
  uint64_t sum     = s_sum;
  uint32_t n       = s_n;
  uint32_t last    = s_last_delta;
  uint32_t pps_ms  = s_last_pps_ms;
  bool     got_pps = s_have_prev;
  __enable_irq();

  out->last_count = last;

  /* CLK2 sống = counter nhích giữa hai lần gọi (getter chạy mỗi ~100 ms
   * từ UI loop → 25 MHz phải nhích trừ khi đứng hẳn) */
  uint32_t cnt_now = TIM3->CNT;
  bool clk_ok = (cnt_now != s_cnt_probe);
  s_cnt_probe = cnt_now;

  if (!clk_ok) {
    out->state = GPSCAL_NO_CLK;
    return;
  }
  if (!got_pps || (HAL_GetTick() - pps_ms) > GPSCAL_PPS_TIMEOUT) {
    out->state = GPSCAL_NO_PPS;
    return;
  }
  if (n == 0U) {
    out->state = GPSCAL_SETTLING;
    return;
  }

  int64_t expected = (int64_t)n * (int64_t)s_nom_hz;
  int64_t err      = (int64_t)sum - expected;
  out->ppb   = (int32_t)((err * 1000000000LL) / expected);
  out->secs  = n;
  out->state = GPSCAL_MEASURING;
}
