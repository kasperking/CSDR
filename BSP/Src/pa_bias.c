/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pa_bias.c
  * @brief   PA gate-bias — MCP4822 trên SPI3 (share W25Q) / fixed trimmer
  *
  *  Mọi hàm ở đây chạy trong main-loop context (CSDR_Init/CSDR_Loop/
  *  csdr_apply_tx/PA_OC_HandleFaultInLoop) — cùng thread với W25Q driver và
  *  Flash_SaveTick, nên một transaction DAC không bao giờ chen vào giữa một
  *  transaction flash: không cần bus lock.  Giữa các bước save async, bus
  *  SPI3 rảnh (W25Q busy là busy NỘI BỘ chip, CS đã nhả).
  ******************************************************************************
  */
/* USER CODE END Header */

#include "pa_bias.h"
#include "pa_overcurrent.h"   /* g_pa_oc.ina_ok/fault, PA_OC_ReadCurrent    */
#include "fsdr_analog.h"      /* g_analog.temp_c (°C×10) — NTC bù nhiệt     */
#include "csdr_app.h"         /* g_sdr.tx_mode/pa_watts, CSDR_RequestTX     */

PA_Bias_State_t g_pa_bias;

static SPI_HandleTypeDef *s_spi = NULL;

/* Ramp: mỗi PA_BIAS_STEP_MS bước PA_BIAS_STEP_CODE về target →
 * full-scale 4095 hết ≈ 16 bước × 2 ms = 32 ms (trong blanking 100 ms). */
#define PA_BIAS_STEP_MS    2U
#define PA_BIAS_STEP_CODE  256U

/* ── MCP4822 write: 16-bit frame, [15]=B/A [13]=GA(1=×1) [12]=SHDN(1=on) ──
 * SPI3 kernel 120 MHz đang cấu hình /4 = 30 MHz cho W25Q — quá 20 MHz max
 * của MCP4822.  Hạ MBR xuống /16 (7.5 MHz) riêng cho transaction này rồi
 * trả lại: HAL giữ SPE=0 ngoài transfer nên sửa CFG1 tại đây là hợp lệ. */
static void bias_dac_write(uint8_t ch, uint16_t code)
{
  if (s_spi == NULL) return;
  uint16_t w = (uint16_t)(((ch != 0U) ? 0xB000U : 0x3000U) | (code & 0x0FFFU));
  uint8_t  f[2] = { (uint8_t)(w >> 8), (uint8_t)w };

  uint32_t cfg1 = s_spi->Instance->CFG1;
  MODIFY_REG(s_spi->Instance->CFG1, SPI_CFG1_MBR, SPI_BAUDRATEPRESCALER_16);
  HAL_GPIO_WritePin(PA_BIAS_CS_GPIO_PORT, PA_BIAS_CS_GPIO_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(s_spi, f, 2U, PA_BIAS_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(PA_BIAS_CS_GPIO_PORT, PA_BIAS_CS_GPIO_PIN, GPIO_PIN_SET);
  s_spi->Instance->CFG1 = cfg1;

  g_pa_bias.code[ch] = code;
  g_pa_bias.write_count++;
}

static inline uint16_t bias_level_to_code(uint8_t level)
{
  if (level > PA_BIAS_LEVEL_MAX) level = PA_BIAS_LEVEL_MAX;
  return (uint16_t)(((uint32_t)level * 4095U) / PA_BIAS_LEVEL_MAX);
}

/* Ghi 0 cả 2 kênh vô điều kiện — dùng cho boot/fault.  Ghi cả khi FIXED:
 * nếu DAC vắng mặt (DNP) thì frame rơi vào chân trống, vô hại; nếu DAC có
 * mặt nhưng jumper để FIXED thì output DAC không nối đi đâu, cũng vô hại. */
static void bias_force_zero(void)
{
  bias_dac_write(0U, 0U);
  bias_dac_write(1U, 0U);
  g_pa_bias.target[0] = 0U;
  g_pa_bias.target[1] = 0U;
  g_pa_bias.armed     = false;
}

/* Bù nhiệt NTC quanh 25 °C, lượng tử 4 code (5 mV gate) chống ghi lắt nhắt
 * khi nhiệt lững lờ quanh biên, clamp ±0.5 V gate. */
static int16_t bias_temp_comp_codes(void)
{
  int32_t c = -((int32_t)g_analog.temp_c - PA_BIAS_TC_REF_C10)
              * PA_BIAS_TC_CODE_X100_PER_C10 / 100;
  if (c >  PA_BIAS_TC_CLAMP_CODE) c =  PA_BIAS_TC_CLAMP_CODE;
  if (c < -PA_BIAS_TC_CLAMP_CODE) c = -PA_BIAS_TC_CLAMP_CODE;
  return (int16_t)((c / 4) * 4);
}

void PA_Bias_Init(SPI_HandleTypeDef *hspi)
{
  s_spi = hspi;

  /* PD3 không nằm trong .ioc — init tại đây.  Idle HIGH trước khi đổi mode
   * để không glitch CS xuống thấp lúc chuyển sang output. */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_GPIO_WritePin(PA_BIAS_CS_GPIO_PORT, PA_BIAS_CS_GPIO_PIN, GPIO_PIN_SET);
  GPIO_InitTypeDef gi = {
    .Pin   = PA_BIAS_CS_GPIO_PIN,
    .Mode  = GPIO_MODE_OUTPUT_PP,
    .Pull  = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(PA_BIAS_CS_GPIO_PORT, &gi);

  g_pa_bias.src      = PA_BIAS_SRC_FIXED;
  g_pa_bias.level[0] = 0U;
  g_pa_bias.level[1] = 0U;

  /* MCU soft-reset không power-cycle DAC — code cũ có thể còn giữ trong
   * thanh ghi.  Ép 0 tại boot, không tin POR. */
  bias_force_zero();
}

void PA_Bias_Configure(uint8_t src, uint8_t level_a, uint8_t level_b)
{
  bool was_dac = (g_pa_bias.src == PA_BIAS_SRC_DAC);
  g_pa_bias.src      = (src == PA_BIAS_SRC_DAC) ? PA_BIAS_SRC_DAC
                                                : PA_BIAS_SRC_FIXED;
  g_pa_bias.level[0] = (level_a <= PA_BIAS_LEVEL_MAX) ? level_a : 0U;
  g_pa_bias.level[1] = (level_b <= PA_BIAS_LEVEL_MAX) ? level_b : 0U;

  if (g_pa_bias.src == PA_BIAS_SRC_FIXED) {
    if (was_dac) bias_force_zero();   /* rời DAC mode: hạ output rồi im lặng */
    return;
  }
  /* DAC mode: level mới có hiệu lực ngay qua Tick (target tính lại từ
   * level+comp mỗi vòng khi armed) — chỉnh Idq sống giữa TX. */
}

void PA_Bias_OnTxStart(void)
{
  if (g_pa_bias.src != PA_BIAS_SRC_DAC) return;
  if (PA_BiasCal_Active()) return;   /* máy cal đang giữ DAC — không arm đè */
  g_pa_bias.armed        = true;
  g_pa_bias.last_step_ms = HAL_GetTick();
}

void PA_Bias_OnRxFinish(void)
{
  if (g_pa_bias.src != PA_BIAS_SRC_DAC) return;
  bias_force_zero();
}

void PA_Bias_OnFault(void)
{
  /* Vô điều kiện (kể cả FIXED) — xem bias_force_zero.  Chạy TRƯỚC
   * INA226_ReadAndClearLatch để lúc ALERT nhả Q1, DAC đã là 0. */
  bias_force_zero();
}

void PA_Bias_Tick(void)
{
  if (PA_BiasCal_Active()) return;   /* máy cal ghi DAC trực tiếp */
  if (g_pa_bias.src != PA_BIAS_SRC_DAC) return;

  /* Target = level + bù nhiệt, tính lại mỗi vòng khi armed: level đổi từ
   * menu/cal và nhiệt trôi đều hội tụ qua cùng đường ramp. */
  int16_t comp = g_pa_bias.armed ? bias_temp_comp_codes() : 0;
  g_pa_bias.comp_codes = comp;
  for (uint8_t ch = 0U; ch < 2U; ch++) {
    int32_t des = 0;
    if (g_pa_bias.armed && g_pa_bias.level[ch] > 0U) {
      des = (int32_t)bias_level_to_code(g_pa_bias.level[ch]) + comp;
      if (des < 0)    des = 0;
      if (des > 4095) des = 4095;
    }
    g_pa_bias.target[ch] = (uint16_t)des;
  }

  if (g_pa_bias.code[0] == g_pa_bias.target[0] &&
      g_pa_bias.code[1] == g_pa_bias.target[1]) return;

  uint32_t now = HAL_GetTick();
  if ((now - g_pa_bias.last_step_ms) < PA_BIAS_STEP_MS) return;
  g_pa_bias.last_step_ms = now;

  for (uint8_t ch = 0U; ch < 2U; ch++) {
    uint16_t cur = g_pa_bias.code[ch];
    uint16_t tgt = g_pa_bias.target[ch];
    if (cur == tgt) continue;
    uint16_t next;
    if (tgt > cur)
      next = ((uint16_t)(tgt - cur) > PA_BIAS_STEP_CODE)
             ? (uint16_t)(cur + PA_BIAS_STEP_CODE) : tgt;
    else
      next = ((uint16_t)(cur - tgt) > PA_BIAS_STEP_CODE)
             ? (uint16_t)(cur - PA_BIAS_STEP_CODE) : tgt;
    bias_dac_write(ch, next);
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Auto-cal Idq closed-loop — kênh A (Bias 1, tầng final)
 *
 *  Chạy toàn bộ trong main-loop (CSDR_Loop → PA_BiasCal_Tick).  TX được key
 *  qua CSDR_RequestTX với drive=0 (csdr_apply_tx ép RF zero khi Active).
 *  Trong lúc cal máy này ghi DAC trực tiếp — PA_Bias_Tick/OnTxStart đứng
 *  ngoài (armed không set), nên bù nhiệt không can thiệp giữa chừng; mức
 *  lưu cuối được quy về 25 °C bằng cách trừ comp tại nhiệt lúc cal.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define BCAL_SETTLE_TX_MS    200U   /* relay + keying gate + INA ổn định     */
#define BCAL_BASE_PERIOD_MS   20U   /* nhịp lấy mẫu baseline                 */
#define BCAL_BASE_SAMPLES      4U
#define BCAL_COARSE_MS        30U   /* nhịp bước thô                         */
#define BCAL_COARSE_CODE      20U   /* ≈25 mV gate/bước                      */
#define BCAL_FINE_MS          25U
#define BCAL_FINE_CODE         5U   /* ≈6 mV gate/bước                       */
#define BCAL_TIMEOUT_MS    10000U

enum { BCAL_IDLE = 0, BCAL_KEYING, BCAL_BASELINE, BCAL_COARSE,
       BCAL_FINE, BCAL_RESULT_OK, BCAL_RESULT_FAIL };

static struct {
  uint8_t  state;
  uint16_t code;         /* mã DAC A hiện tại của máy cal            */
  uint16_t idq_ma;       /* đích                                     */
  float    i_base;       /* dòng nền (A) — gồm Idq driver (kênh B)   */
  float    i_acc;
  uint8_t  n_acc;
  uint8_t  result_lvl;
  uint32_t t_start, t_step;
} s_cal;

static void bcal_abort(void)
{
  bias_force_zero();
  CSDR_RequestTX(false);
  s_cal.state = BCAL_RESULT_FAIL;
}

bool PA_BiasCal_Active(void)
{
  return (s_cal.state != BCAL_IDLE) &&
         (s_cal.state != BCAL_RESULT_OK) && (s_cal.state != BCAL_RESULT_FAIL);
}

bool PA_BiasCal_Start(uint16_t idq_target_ma)
{
  if (PA_BiasCal_Active())                  return false;
  if (g_pa_bias.src != PA_BIAS_SRC_DAC)     return false;
  if (g_sdr.pa_watts == 0U)                 return false;
  if (!g_pa_oc.ina_ok || g_pa_oc.fault_pending) return false;
  if (g_sdr.tx_mode)                        return false;

  s_cal.idq_ma  = (idq_target_ma >= 50U) ? idq_target_ma : 500U;
  s_cal.code    = 0U;
  s_cal.i_acc   = 0.0f;
  s_cal.n_acc   = 0U;
  s_cal.t_start = HAL_GetTick();
  s_cal.t_step  = s_cal.t_start;
  s_cal.state   = BCAL_KEYING;    /* Active TRƯỚC RequestTX: OnTxStart bỏ qua,
                                     csdr_apply_tx ép drive=0 ngay từ frame đầu */
  CSDR_RequestTX(true);
  return true;
}

void PA_BiasCal_Tick(void)
{
  if (!PA_BiasCal_Active()) return;
  uint32_t now = HAL_GetTick();

  /* Điều kiện sống còn — kiểm tra mọi state (src đổi FIXED giữa chừng từ
   * menu cũng phải dừng: cal ghi DAC trực tiếp, không qua Tick) */
  if (!g_sdr.tx_mode || !g_pa_oc.ina_ok || g_pa_oc.fault_pending ||
      g_pa_bias.src != PA_BIAS_SRC_DAC ||
      (now - s_cal.t_start) > BCAL_TIMEOUT_MS) {
    bcal_abort();
    return;
  }

  float tgt_a  = (float)s_cal.idq_ma * 0.001f;
  float oc_cap = g_pa_oc.limit_a * 0.8f;

  switch (s_cal.state) {

  case BCAL_KEYING:
    if ((now - s_cal.t_step) < BCAL_SETTLE_TX_MS) return;
    /* Driver (kênh B) lên mức cấu hình — dòng của nó nằm trong baseline;
     * final (kênh A) giữ 0. */
    bias_dac_write(1U, bias_level_to_code(g_pa_bias.level[1]));
    bias_dac_write(0U, 0U);
    s_cal.state  = BCAL_BASELINE;
    s_cal.t_step = now;
    return;

  case BCAL_BASELINE:
    if ((now - s_cal.t_step) < BCAL_BASE_PERIOD_MS) return;
    s_cal.t_step = now;
    s_cal.i_acc += PA_OC_ReadCurrent();
    if (++s_cal.n_acc < BCAL_BASE_SAMPLES) return;
    s_cal.i_base = s_cal.i_acc / (float)BCAL_BASE_SAMPLES;
    s_cal.state  = BCAL_COARSE;
    return;

  case BCAL_COARSE: {
    if ((now - s_cal.t_step) < BCAL_COARSE_MS) return;
    s_cal.t_step = now;
    float idq = PA_OC_ReadCurrent() - s_cal.i_base;
    /* FET dốc có thể nhảy quá 2×đích trong MỘT bước thô — đó là việc của
     * back-off, không abort; chỉ abort khi chạm trần OC. */
    if ((s_cal.i_base + idq) >= oc_cap) { bcal_abort(); return; }
    if (idq >= tgt_a) {
      /* Vượt đích bằng bước thô — lùi 3 bước rồi dò mịn lên lại */
      uint16_t back = 3U * BCAL_COARSE_CODE;
      s_cal.code   = (s_cal.code > back) ? (uint16_t)(s_cal.code - back) : 0U;
      bias_dac_write(0U, s_cal.code);
      s_cal.state  = BCAL_FINE;
      return;
    }
    if (s_cal.code >= 4095U) { bcal_abort(); return; }  /* hết thang — PA vắng? */
    s_cal.code = (uint16_t)((s_cal.code + BCAL_COARSE_CODE > 4095U)
                            ? 4095U : s_cal.code + BCAL_COARSE_CODE);
    bias_dac_write(0U, s_cal.code);
    return;
  }

  case BCAL_FINE: {
    if ((now - s_cal.t_step) < BCAL_FINE_MS) return;
    s_cal.t_step = now;
    float idq = PA_OC_ReadCurrent() - s_cal.i_base;
    if (idq >= 2.0f * tgt_a || (s_cal.i_base + idq) >= oc_cap) { bcal_abort(); return; }
    if (idq >= tgt_a) {
      /* Đạt đích — quy mức về 25 °C: code lưu = code hiện tại − comp(T_cal),
       * để Tick cộng lại comp(T) ở mọi nhiệt vẫn ra đúng Idq này. */
      int32_t base_code = (int32_t)s_cal.code - (int32_t)bias_temp_comp_codes();
      if (base_code < 0) base_code = 0;
      int32_t lvl = (base_code * (int32_t)PA_BIAS_LEVEL_MAX + 2047) / 4095;
      if (lvl > (int32_t)PA_BIAS_LEVEL_MAX) lvl = (int32_t)PA_BIAS_LEVEL_MAX;
      s_cal.result_lvl = (uint8_t)lvl;
      CSDR_RequestTX(false);   /* finish_rx_hw → OnRxFinish hạ DAC về 0 */
      s_cal.state = BCAL_RESULT_OK;
      return;
    }
    if (s_cal.code >= 4095U) { bcal_abort(); return; }
    s_cal.code = (uint16_t)((s_cal.code + BCAL_FINE_CODE > 4095U)
                            ? 4095U : s_cal.code + BCAL_FINE_CODE);
    bias_dac_write(0U, s_cal.code);
    return;
  }

  default:
    return;
  }
}

uint8_t PA_BiasCal_Poll(uint8_t *level_out)
{
  if (s_cal.state == BCAL_RESULT_OK) {
    s_cal.state = BCAL_IDLE;
    if (level_out) *level_out = s_cal.result_lvl;
    return 1U;
  }
  if (s_cal.state == BCAL_RESULT_FAIL) {
    s_cal.state = BCAL_IDLE;
    return 2U;
  }
  return 0U;
}
