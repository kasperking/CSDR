/* USER CODE BEGIN Header */
/**
  * @file pe4302.c
  * @brief PE4302 Digital Step Attenuator Driver
  */
/* USER CODE END Header */
#include "pe4302.h"
#include "csdr_app.h"

/* USER CODE BEGIN PV */
PE4302_Handle_t g_att;
/* USER CODE END PV */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define _DAT_H(a) HAL_GPIO_WritePin((a)->dat_port,(a)->dat_pin,GPIO_PIN_SET)
#define _DAT_L(a) HAL_GPIO_WritePin((a)->dat_port,(a)->dat_pin,GPIO_PIN_RESET)
#define _CLK_H(a) HAL_GPIO_WritePin((a)->clk_port,(a)->clk_pin,GPIO_PIN_SET)
#define _CLK_L(a) HAL_GPIO_WritePin((a)->clk_port,(a)->clk_pin,GPIO_PIN_RESET)
#define _LE_H(a)  HAL_GPIO_WritePin((a)->le_port, (a)->le_pin, GPIO_PIN_SET)
#define _LE_L(a)  HAL_GPIO_WritePin((a)->le_port, (a)->le_pin, GPIO_PIN_RESET)

/**
  * @brief  Ghi 6-bit word vào PE4302 qua bit-bang SPI.
  *         MSB first. LE pulse sau khi hoàn thành.
  */
static void pe4302_write(PE4302_Handle_t *att, uint8_t val_x2)
{
  _LE_L(att);
  /* Clock 6 bits MSB first */
  for (int8_t bit = 5; bit >= 0; bit--)
  {
    if (val_x2 & (1U << (uint8_t)bit)) { _DAT_H(att); } else { _DAT_L(att); }
    _CLK_H(att);
    __NOP(); __NOP();   /* ~10ns setup */
    _CLK_L(att);
    __NOP(); __NOP();
  }
  /* Latch: LE HIGH pulse */
  _LE_H(att);
  __NOP(); __NOP(); __NOP(); __NOP();
  _LE_L(att);
  _DAT_L(att);
}

/* USER CODE END 0 */

void PE4302_Init(PE4302_Handle_t *att)
{
  /* USER CODE BEGIN PE4302_Init_0 */
  att->dat_port = ATT_DAT_GPIO_Port;  att->dat_pin = ATT_DAT_Pin;
  att->clk_port = ATT_CLK_GPIO_Port;  att->clk_pin = ATT_CLK_Pin;
  att->le_port  = ATT_LATCH_GPIO_Port; att->le_pin = ATT_LATCH_Pin;
  att->current_atten_db = 0U;
  att->current_atten_x2 = 0U;
  /* Set all LOW */
  _DAT_L(att); _CLK_L(att); _LE_L(att);
  /* Default: 0dB (bypass) */
  pe4302_write(att, 0x00U);
  /* USER CODE END PE4302_Init_0 */
}

void PE4302_SetAttn_Raw(PE4302_Handle_t *att, uint8_t val_x2)
{
  /* USER CODE BEGIN PE4302_SetAttn_Raw_0 */
  if (val_x2 > PE4302_MAX_ATTN_X2) { val_x2 = PE4302_MAX_ATTN_X2; }
  att->current_atten_x2 = val_x2;
  att->current_atten_db = val_x2 / 2U;
  pe4302_write(att, val_x2);
  /* USER CODE END PE4302_SetAttn_Raw_0 */
}

void PE4302_SetAttn_dB(PE4302_Handle_t *att, uint8_t db)
{
  /* USER CODE BEGIN PE4302_SetAttn_dB_0 */
  if (db > 31U) { db = 31U; }
  PE4302_SetAttn_Raw(att, (uint8_t)(db * 2U));
  /* USER CODE END PE4302_SetAttn_dB_0 */
}

void PE4302_IncAttn(PE4302_Handle_t *att)
{
  /* USER CODE BEGIN PE4302_IncAttn_0 */
  uint8_t next = att->current_atten_x2 + 2U;
  if (next > PE4302_MAX_ATTN_X2) { next = PE4302_MAX_ATTN_X2; }
  PE4302_SetAttn_Raw(att, next);
  /* USER CODE END PE4302_IncAttn_0 */
}

void PE4302_DecAttn(PE4302_Handle_t *att)
{
  /* USER CODE BEGIN PE4302_DecAttn_0 */
  if (att->current_atten_x2 < 2U) { PE4302_SetAttn_Raw(att, 0U); return; }
  PE4302_SetAttn_Raw(att, att->current_atten_x2 - 2U);
  /* USER CODE END PE4302_DecAttn_0 */
}

void PE4302_Bypass(PE4302_Handle_t *att)
{
  PE4302_SetAttn_Raw(att, 0x00U);
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
