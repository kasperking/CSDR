/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Encoder BSP – TIM3 hardware quadrature (PB4=CH1, PB5=CH2)
  *
  *  Đọc TIM3->CNT mỗi 1ms (từ SysTick callback qua Encoder_Poll).
  *  Delta = CNT_now - CNT_prev (signed 16-bit → xử lý wrap-around).
  *  Gia tốc: đọc nhanh nhiều xung → nhân hệ số.
  *  Nút ENC_SW (PB3): polling có debounce + long press.
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
/* EC11 in TIM3 X4 mode = 4 quadrature counts per physical detent.
 * Accumulate raw counts; only fire a step when full detent reached.
 * Bounce < 4 counts is silently discarded. Delta is scaled ×COUNTS_PER_STEP
 * so VFO sensitivity is unchanged vs the old direct-accumulation code. */
#define ENC_COUNTS_PER_STEP    4
/* After a step fires, block opposite-direction steps for this many ms.
 * EC11 bounce is typically 5–30 ms; intentional reversal takes > 100 ms. */
#define ENC_DIR_GUARD_MS      40U
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
  enc->raw_accum      = 0;
  enc->delta          = 0;
  enc->accel_count    = 0U;
  enc->guard_dir      = 0;
  enc->guard_expiry   = 0U;
  enc->accel_mult     = 1;
  enc->last_tick      = HAL_GetTick();
  enc->btn_pressed    = false;
  enc->btn_long       = false;
  enc->btn_down_tick  = 0U;
  enc->btn_prev_state = false;  /* PULLUP → released = not pressed = false */
  enc->btn_edge_tick  = 0U;
  enc->btn_in_press   = false;
  enc->debounce_ms    = 20U;
  enc->long_press_ms  = 800U;
  /* USER CODE END Encoder_Init_0 */
}

/**
  * @brief  Polling encoder – gọi mỗi 1ms từ HAL_SYSTICK_Callback.
  *
  *  1. Đọc TIM3->CNT, tính delta (16-bit signed để xử lý wrap-around 0/65535).
  *  2. Tính gia tốc: |delta| lớn trong thời gian ngắn → nhân hệ số.
  *  3. Đọc ENC_SW (PB3) với debounce & long press.
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

  /* ── Step 1: accumulate raw counts, threshold to full detents ───────── */
  enc->raw_accum += (int32_t)raw;

  int32_t steps = 0;
  if      (enc->raw_accum >=  ENC_COUNTS_PER_STEP) { steps = enc->raw_accum / ENC_COUNTS_PER_STEP; enc->raw_accum %= ENC_COUNTS_PER_STEP; }
  else if (enc->raw_accum <= -ENC_COUNTS_PER_STEP) { steps = enc->raw_accum / ENC_COUNTS_PER_STEP; enc->raw_accum %= ENC_COUNTS_PER_STEP; }

  /* ── Step 2: direction guard — discard opposite-direction steps within
   *    ENC_DIR_GUARD_MS after the last confirmed step (EC11 bounce filter) ─ */
  if (steps != 0)
  {
    uint32_t now = HAL_GetTick();
    int8_t   dir = (steps > 0) ? +1 : -1;

    if (enc->guard_dir != 0 && dir != enc->guard_dir &&
        (int32_t)(now - enc->guard_expiry) < 0)
    {
      /* Bounce: reverse step within guard window — discard and clear accum */
      enc->raw_accum = 0;
      steps = 0;
    }
    else
    {
      /* Valid step: arm guard in this direction */
      enc->guard_dir    = dir;
      enc->guard_expiry = now + ENC_DIR_GUARD_MS;

      /* ── Step 3: acceleration ────────────────────────────────────────── */
      uint32_t dt  = now - enc->last_tick;
      enc->last_tick = now;

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

      enc->delta += steps * mult;
    }
  }

  if (steps == 0) {
    /* No valid step this tick → decay acceleration */
    if (enc->accel_count > 0U) { enc->accel_count--; }
  }

  /* ── 3. Nút nhấn ENC_SW (PB3) – stability-window debounce ──────────
   * btn_now=true = pressed (active-low, GPIO_PIN_RESET).
   * Any edge resets the stability timer and un-confirms the press state.
   * Events fire only after the pin holds stable for debounce_ms, preventing
   * both mid-hold bounce double-fires and bouncy-release false triggers. */
  bool btn_now = (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET);
  uint32_t tick = HAL_GetTick();

  if (btn_now != enc->btn_prev_state) {
    enc->btn_prev_state = btn_now;
    enc->btn_edge_tick  = tick;
    /* btn_in_press intentionally NOT cleared here: the stability check below
     * handles all transitions, ensuring release events are never lost. */
  } else if ((tick - enc->btn_edge_tick) >= enc->debounce_ms) {
    if (btn_now && !enc->btn_in_press) {
      enc->btn_in_press  = true;
      enc->btn_down_tick = tick;   /* start of confirmed press */
    } else if (!btn_now && enc->btn_in_press) {
      enc->btn_in_press = false;
      uint32_t held = tick - enc->btn_down_tick;
      if (held >= enc->long_press_ms) { enc->btn_long    = true; }
      else                            { enc->btn_pressed = true; }
    }
  }

  /* USER CODE END Encoder_Poll_0 */
}

/**
  * @brief  Đọc và xóa delta tích lũy.
  * @retval int32_t delta (đã nhân gia tốc)
  */
int32_t Encoder_GetDelta(Encoder_t *enc)
{
  /* USER CODE BEGIN Encoder_GetDelta_0 */
  /* Encoder_Poll runs in the SysTick ISR: an increment landing between the
   * read and the clear below would be lost (missed detent).  Mask IRQs for
   * the two accesses — a handful of cycles once per main-loop tick. */
  __disable_irq();
  int32_t d = enc->delta;
  enc->delta = 0;
  __enable_irq();
  return d;
  /* USER CODE END Encoder_GetDelta_0 */
}

/**
  * @brief  Đọc và xóa cờ nhấn ngắn.
  */
bool Encoder_GetButton(Encoder_t *enc)
{
  /* USER CODE BEGIN Encoder_GetButton_0 */
  /* Same test-and-clear atomicity as Encoder_GetDelta. */
  bool hit = false;
  __disable_irq();
  if (enc->btn_pressed) { enc->btn_pressed = false; hit = true; }
  __enable_irq();
  return hit;
  /* USER CODE END Encoder_GetButton_0 */
}

/**
  * @brief  Đọc và xóa cờ nhấn dài.
  */
bool Encoder_GetLongPress(Encoder_t *enc)
{
  /* USER CODE BEGIN Encoder_GetLongPress_0 */
  bool hit = false;
  __disable_irq();
  if (enc->btn_long) { enc->btn_long = false; hit = true; }
  __enable_irq();
  return hit;
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

static bool key_read_raw(const Key_t *k)
{
  if (k->src == KEY_SRC_PCA9555) {
    /* Active-low: bit = 0 means pressed */
    return !((uint8_t)((*k->pca_cache >> k->pca_bit) & 1U));
  }
  return (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET);
}

void Key_Sync(Key_t *k)
{
  bool raw = key_read_raw(k);
  k->raw_prev    = raw;
  k->t_stable    = HAL_GetTick();
  k->state       = raw ? KS_WAIT_RELEASE : KS_IDLE;
  k->evt_press   = false;
  k->evt_hold    = false;
  k->evt_repeat  = false;
  k->evt_release = false;
}

void Key_Poll(Key_t *k)
{
  uint32_t now = HAL_GetTick();
  bool raw = key_read_raw(k);

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

    case KS_WAIT_RELEASE:      /* key was already down at Key_Sync: swallow it */
      if (!raw) { k->state = KS_IDLE; }
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
