/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Encoder BSP – TIM3 hardware quadrature (PB4=CH1, PB5=CH2)
  *
  *  Đọc TIM3->CNT mỗi 1ms (từ SysTick callback qua Encoder_Poll).
  *  Delta = CNT_now - CNT_prev (signed 16-bit → xử lý wrap-around).
  *  Gia tốc: đọc nhanh nhiều xung → nhân hệ số.
  *  Nút PB3 (ENC_SW): polling có debounce + long press.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "encoder.h"
#include "csdr_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENC_ACCEL_WINDOW_MS    50U   /* Cửa sổ thời gian tích xung gia tốc */
#define ENC_ACCEL_THRESH_HI   30U
#define ENC_ACCEL_THRESH_MED  15U
#define ENC_ACCEL_THRESH_LO    6U
/* USER CODE END PD */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
Encoder_t g_encoder;
/* USER CODE END EV */

/**
  * @brief  Khởi tạo encoder (gọi sau HAL_TIM_Encoder_Start).
  * @param  enc   Encoder handle
  * @param  htim  TIM3 handle (đã init bởi MX_TIM3_Init)
  */
void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Encoder_Init_0 */
  enc->htim           = htim;
  enc->cnt_prev       = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
  enc->delta          = 0;
  enc->accel_count    = 0U;
  enc->accel_mult     = 1;
  enc->last_tick      = HAL_GetTick();
  enc->btn_pressed    = false;
  enc->btn_long       = false;
  enc->btn_down_tick  = 0U;
  enc->btn_prev_state = true;   /* PULLUP → released = HIGH = true */
  enc->debounce_ms    = 20U;
  enc->long_press_ms  = 800U;
  /* USER CODE END Encoder_Init_0 */
}

/**
  * @brief  Polling encoder – gọi mỗi 1ms từ HAL_SYSTICK_Callback.
  *
  *  1. Đọc TIM3->CNT, tính delta (16-bit signed để xử lý wrap-around 0/65535).
  *  2. Tính gia tốc: |delta| lớn trong thời gian ngắn → nhân hệ số.
  *  3. Đọc PB3 (ENC_SW) với debounce & long press.
  *
  * @param  enc  Encoder handle
  */
void Encoder_Poll(Encoder_t *enc)
{
  /* USER CODE BEGIN Encoder_Poll_0 */

  /* ── 1. Đọc TIM3 CNT ─────────────────────────────────────── */
  uint16_t cnt_now = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
  int16_t  raw     = (int16_t)(cnt_now - enc->cnt_prev);
  enc->cnt_prev    = cnt_now;

  if (raw != 0)
  {
    /* ── 2. Gia tốc ──────────────────────────────────────────── */
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - enc->last_tick;
    enc->last_tick = now;

    /* Cộng dồn bộ đếm theo tốc độ xoay */
    uint32_t abs_raw = (raw < 0) ? (uint32_t)(-raw) : (uint32_t)raw;
    if (dt < ENC_ACCEL_WINDOW_MS) {
      enc->accel_count += abs_raw * 2U;
    } else {
      enc->accel_count = (enc->accel_count > abs_raw) ?
                          enc->accel_count - abs_raw : 0U;
    }
    if (enc->accel_count > 200U) { enc->accel_count = 200U; }

    int32_t mult;
    if      (enc->accel_count >= ENC_ACCEL_THRESH_HI)  { mult = 100; }
    else if (enc->accel_count >= ENC_ACCEL_THRESH_MED) { mult =  10; }
    else if (enc->accel_count >= ENC_ACCEL_THRESH_LO)  { mult =   5; }
    else                                                 { mult =   1; }
    enc->accel_mult = mult;

    enc->delta += (int32_t)raw * mult;
  }
  else
  {
    /* Không xoay → giảm dần gia tốc */
    if (enc->accel_count > 0U) { enc->accel_count--; }
  }

  /* ── 3. Nút nhấn PB3 (ENC_SW) – polling ─────────────────── */
  bool btn_now = (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_SET);
  uint32_t tick = HAL_GetTick();

  if (enc->btn_prev_state && !btn_now)
  {
    /* Cạnh xuống: bắt đầu nhấn */
    enc->btn_down_tick = tick;
  }
  else if (!enc->btn_prev_state && btn_now)
  {
    /* Cạnh lên: nhả nút */
    uint32_t held = tick - enc->btn_down_tick;
    if (held > enc->debounce_ms)
    {
      if (held >= enc->long_press_ms) { enc->btn_long    = true; }
      else                            { enc->btn_pressed = true; }
    }
  }
  enc->btn_prev_state = btn_now;

  /* USER CODE END Encoder_Poll_0 */
}

/**
  * @brief  Đọc và xóa delta tích lũy.
  * @retval int32_t delta (đã nhân gia tốc)
  */
int32_t Encoder_GetDelta(Encoder_t *enc)
{
  /* USER CODE BEGIN Encoder_GetDelta_0 */
  int32_t d = enc->delta;
  enc->delta = 0;
  return d;
  /* USER CODE END Encoder_GetDelta_0 */
}

/**
  * @brief  Đọc và xóa cờ nhấn ngắn.
  */
bool Encoder_GetButton(Encoder_t *enc)
{
  /* USER CODE BEGIN Encoder_GetButton_0 */
  if (enc->btn_pressed) { enc->btn_pressed = false; return true; }
  return false;
  /* USER CODE END Encoder_GetButton_0 */
}

/**
  * @brief  Đọc và xóa cờ nhấn dài.
  */
bool Encoder_GetLongPress(Encoder_t *enc)
{
  /* USER CODE BEGIN Encoder_GetLongPress_0 */
  if (enc->btn_long) { enc->btn_long = false; return true; }
  return false;
  /* USER CODE END Encoder_GetLongPress_0 */
}

/* USER CODE BEGIN 1 */

/* ── Key state machine implementation ─────────────────────────── */

void Key_Init(Key_t *k, GPIO_TypeDef *port, uint16_t pin)
{
  k->port       = port;
  k->pin        = pin;
  k->pca_cache  = NULL;
  k->pca_bit    = 0U;
  k->src        = KEY_SRC_GPIO;
  k->state      = KS_IDLE;
  k->raw_prev   = false;
  k->t_stable   = HAL_GetTick();
  k->t_press    = 0U;
  k->t_repeat   = 0U;
  k->evt_press   = false;
  k->evt_hold    = false;
  k->evt_repeat  = false;
  k->evt_release = false;
}

void Key_InitPCA(Key_t *k, const uint16_t *pca_cache, uint8_t pca_bit)
{
  k->port       = NULL;
  k->pin        = 0U;
  k->pca_cache  = pca_cache;
  k->pca_bit    = pca_bit;
  k->src        = KEY_SRC_PCA9555;
  k->state      = KS_IDLE;
  k->raw_prev   = false;
  k->t_stable   = HAL_GetTick();
  k->t_press    = 0U;
  k->t_repeat   = 0U;
  k->evt_press   = false;
  k->evt_hold    = false;
  k->evt_repeat  = false;
  k->evt_release = false;
}

void Key_Poll(Key_t *k)
{
  uint32_t now = HAL_GetTick();
  bool raw;
  if (k->src == KEY_SRC_PCA9555) {
    /* Active-low: bit = 0 means pressed */
    raw = !((uint8_t)((*k->pca_cache >> k->pca_bit) & 1U));
  } else {
    raw = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET);
  }

  if (raw != k->raw_prev) {
    k->raw_prev = raw;
    k->t_stable = now;
    return;
  }

  if ((now - k->t_stable) < KEY_DEBOUNCE_MS) { return; }

  switch (k->state) {
    case KS_IDLE:
      if (raw) {
        k->state     = KS_PRESSED;
        k->t_press   = now;
        k->evt_press = true;
      }
      break;

    case KS_PRESSED:
      if (!raw) {
        k->state       = KS_IDLE;
        k->evt_release = true;
      } else if ((now - k->t_press) >= KEY_HOLD_MS) {
        k->state      = KS_HELD;
        k->evt_hold   = true;
        k->t_repeat   = now + KEY_REPEAT_RATE_MS;
      }
      break;

    case KS_HELD:
      if (!raw) {
        k->state       = KS_IDLE;
        k->evt_release = true;
      } else if ((int32_t)(now - k->t_repeat) >= 0) {
        k->t_repeat   += KEY_REPEAT_RATE_MS;
        k->evt_repeat  = true;
      }
      break;

    default:
      k->state = KS_IDLE;
      break;
  }
}

bool Key_Press(Key_t *k)
  { bool e = k->evt_press;   k->evt_press   = false; return e; }
bool Key_Hold(Key_t *k)
  { bool e = k->evt_hold;    k->evt_hold    = false; return e; }
bool Key_Repeat(Key_t *k)
  { bool e = k->evt_repeat;  k->evt_repeat  = false; return e; }
bool Key_Release(Key_t *k)
  { bool e = k->evt_release; k->evt_release = false; return e; }
bool Key_PressOrRepeat(Key_t *k)
  { bool e = k->evt_press || k->evt_repeat;
    k->evt_press = false; k->evt_repeat = false; return e; }

/* USER CODE END 1 */
