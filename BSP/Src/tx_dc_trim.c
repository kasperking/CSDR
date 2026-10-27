/* QSE carrier-leak DC trim — PWM TIM1 CH1/CH2 (PA8/PA9). Xem tx_dc_trim.h. */
#include "tx_dc_trim.h"
#include "stm32h7xx_hal.h"
#include "csdr_app.h"     /* g_tx_iq_cal, BAND_COUNT */

static int16_t s_i = 0, s_q = 0;
static uint8_t s_ready = 0U;

static inline int16_t clamp_trim(int16_t v)
{
  if (v >  TXDC_TRIM_MAX) return  TXDC_TRIM_MAX;
  if (v < -TXDC_TRIM_MAX) return -TXDC_TRIM_MAX;
  return v;
}

void TxDcTrim_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* Timer chạy trước, rồi mới chuyển chân sang AF.  Trước đó PA8/PA9 ở
   * analog (reset H7) = hở → IN+ chỉ thấy TX_BIAS qua 10k, offset 0. */
  TIM1->CR1   = 0U;
  TIM1->PSC   = 0U;
  TIM1->ARR   = TXDC_ARR;
  TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |   /* PWM1 */
                (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCR1  = (uint32_t)TXDC_ZERO_CODE;
  TIM1->CCR2  = (uint32_t)TXDC_ZERO_CODE;
  TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM1->BDTR  = TIM_BDTR_MOE;          /* timer advanced: thiếu MOE = chân im */
  TIM1->EGR   = TIM_EGR_UG;            /* nạp PSC/ARR/CCR preload */
  TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

  GPIO_InitTypeDef gi = {
    .Pin       = GPIO_PIN_8 | GPIO_PIN_9,
    .Mode      = GPIO_MODE_AF_PP,
    .Pull      = GPIO_NOPULL,
    .Speed     = GPIO_SPEED_FREQ_LOW,  /* sườn chậm — chỉ cần mức DC */
    .Alternate = GPIO_AF1_TIM1,
  };
  HAL_GPIO_Init(GPIOA, &gi);

  s_i = 0; s_q = 0;
  s_ready = 1U;
}

void TxDcTrim_Set(int16_t i, int16_t q)
{
  if (!s_ready) return;
  i = clamp_trim(i);
  q = clamp_trim(q);
  if (i == s_i && q == s_q) return;
  s_i = i; s_q = q;
  TIM1->CCR1 = (uint32_t)(TXDC_ZERO_CODE + i);   /* I → INB+ */
  TIM1->CCR2 = (uint32_t)(TXDC_ZERO_CODE + q);   /* Q → INA+ */
}

void TxDcTrim_FollowBand(uint8_t band)
{
  if (band >= BAND_COUNT) return;
  TxDcTrim_Set(g_tx_iq_cal[band].dc_i, g_tx_iq_cal[band].dc_q);
}
