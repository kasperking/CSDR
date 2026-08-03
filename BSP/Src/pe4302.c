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

/* PE4302 datasheet (Doc 70-0056-04) Table 8, Serial Interface AC Characteristics:
 *   tClkH / tClkL min = 30ns, tLEPW min = 30ns, tSDSUP / tSDHLD min = 10ns.
 * CPU core runs at 480MHz (Core/Src/main.c PLL config), so a single __NOP()
 * is only ~2ns and the Cortex-M7 can dual-issue back-to-back NOPs -- the old
 * 2-NOP gap (~4-8ns) did not reliably clear the 30ns minimums, and there was
 * no gap at all between writing DATA and raising CLK (tSDSUP).
 * A DWT->CYCCNT busy-wait was deliberately avoided here: that counter is not
 * guaranteed to be ticking (see the enable comment in usb_cat.c), and a wait
 * loop keyed off it would hang forever if it weren't -- unacceptable inside
 * the RF AGC path. Plain unrolled NOPs always take real, bounded time, so
 * this can never stall; it only costs a few hundred extra nanoseconds per
 * attenuator write, irrelevant against the ~20ms AGC update period.
 *
 * DATA (pin 3) also has a 10k series resistor on this board, required by the
 * datasheet to kill package resonance with the adjacent RF1 pin. That same
 * resistor forms an RC low-pass with the PE4302 input capacitance (not
 * specified in this datasheet, typically a few pF for an RF IC), which slows
 * the DATA edge before it crosses the input threshold -- on top of, not
 * covered by, the pure digital-logic margin above. Doubled the NOP count
 * to buy extra headroom for that; still negligible in absolute time.
 */
#define PE4302_BB_DELAY() \
  do { \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
  } while (0)

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
    PE4302_BB_DELAY();   /* tSDSUP: data setup before CLK rising edge */
    _CLK_H(att);
    PE4302_BB_DELAY();   /* tClkH: clock HIGH time */
    _CLK_L(att);
    PE4302_BB_DELAY();   /* tClkL: clock LOW time (also covers tSDHLD) */
  }
  /* Latch: LE HIGH pulse (gap since last CLK falling edge already covers tLESUP) */
  _LE_H(att);
  PE4302_BB_DELAY();     /* tLEPW: LE pulse width */
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
