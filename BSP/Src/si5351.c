/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    si5351.c
  * @brief   SI5351A Programmable Clock Generator BSP Driver
  *
  *  74LVC74 divide-by-4 quadrature architecture.
  *  Si5351 outputs a single clock at 4× LO frequency; external hardware
  *  performs the ÷4 and generates precise 0°/90° I/Q phases.
  *
  *  Frequency programming example (7.1 MHz RF → 28.4 MHz CLK0):
  *
  *  1. clk_freq = 7,100,000 × 4 = 28,400,000 Hz
  *  2. MS_div = 750 MHz / 28.4 MHz ≈ 26  (any integer, even not required)
  *  3. VCO_A = 28,400,000 × 26 = 738,400,000 Hz  ∈ [600,900] MHz
  *  4. CalcPLL: 738.4 MHz / 25 MHz = 29 + r/25 → P1, P2, P3
  *  5. Write PLL_A (Reg 26-33)
  *  6. Write MS0 (Reg 42-49): P1 = 128×26-512 = 2816, P2=0, P3=1
  *  7. CLK0_CTRL (Reg 16) = PLLA, MS src, 8mA (no phase offset)
  *  8. Reset PLL_A (Reg 177 |= 0x04)
  *  9. Enable CLK0 only (Reg 3 = 0xFE)  – CLK1 remains powered down
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "si5351.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SI5351_TIMEOUT_MS     50U
#define SI5351_INIT_DELAY_MS   10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
SI5351_Handle_t g_si5351;
/* USER CODE END EV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef SI5351_WriteReg(SI5351_Handle_t *si, uint8_t reg, uint8_t val);
static HAL_StatusTypeDef SI5351_ReadReg(SI5351_Handle_t *si, uint8_t reg, uint8_t *val);
static HAL_StatusTypeDef SI5351_WriteBurst(SI5351_Handle_t *si, uint8_t reg,
                                            const uint8_t *data, uint8_t len);
static HAL_StatusTypeDef SI5351_WriteMS(SI5351_Handle_t *si, uint8_t base_reg,
                                         const SI5351_MSParams_t *p, uint8_t r_div_code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Ghi một byte vào thanh ghi SI5351.
  */
static HAL_StatusTypeDef SI5351_WriteReg(SI5351_Handle_t *si, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { reg, val };
  return HAL_I2C_Master_Transmit(si->hi2c, si->i2c_addr, buf, 2U, SI5351_TIMEOUT_MS);
}

/**
  * @brief  Đọc một byte từ thanh ghi SI5351.
  */
static HAL_StatusTypeDef SI5351_ReadReg(SI5351_Handle_t *si, uint8_t reg, uint8_t *val)
{
  HAL_StatusTypeDef st;
  st = HAL_I2C_Master_Transmit(si->hi2c, si->i2c_addr, &reg, 1U, SI5351_TIMEOUT_MS);
  if (st != HAL_OK) { return st; }
  return HAL_I2C_Master_Receive(si->hi2c, si->i2c_addr, val, 1U, SI5351_TIMEOUT_MS);
}

/**
  * @brief  Ghi burst nhiều byte liên tiếp.
  */
static HAL_StatusTypeDef SI5351_WriteBurst(SI5351_Handle_t *si, uint8_t reg,
                                            const uint8_t *data, uint8_t len)
{
  uint8_t buf[16];
  if (len > 15U) { return HAL_ERROR; }
  buf[0] = reg;
  for (uint8_t i = 0U; i < len; i++) { buf[i + 1U] = data[i]; }
  return HAL_I2C_Master_Transmit(si->hi2c, si->i2c_addr, buf, (uint16_t)(len + 1U),
                                  SI5351_TIMEOUT_MS);
}

/**
  * @brief  Ghi 8 byte tham số MultiSynth (PLL hoặc Output divider).
  *
  *  Layout 8 byte (theo AN619):
  *   [0] P3[15:8]
  *   [1] P3[7:0]
  *   [2] bits[1:0]=P1[17:16], bits[7:4]=R_DIV[2:0] (chỉ cho output MS)
  *   [3] P1[15:8]
  *   [4] P1[7:0]
  *   [5] P3[19:16]<<4 | P2[19:16]
  *   [6] P2[15:8]
  *   [7] P2[7:0]
  *
  * @param  base_reg   Địa chỉ thanh ghi đầu (SI5351_REG_MSNA_BASE, v.v.)
  * @param  p          Tham số P1, P2, P3
  * @param  r_div_code Mã R_DIV (0=÷1 .. 7=÷128). Chỉ áp dụng cho output MS.
  *                    Với PLL (MSNA/MSNB), truyền 0.
  */
static HAL_StatusTypeDef SI5351_WriteMS(SI5351_Handle_t *si, uint8_t base_reg,
                                         const SI5351_MSParams_t *p, uint8_t r_div_code)
{
  uint8_t d[8];
  d[0] = (uint8_t)((p->p3 >> 8U)  & 0xFFU);
  d[1] = (uint8_t)( p->p3         & 0xFFU);
  d[2] = (uint8_t)(((r_div_code & 0x07U) << 4U) |
                   ((p->p1 >> 16U) & 0x03U));
  d[3] = (uint8_t)((p->p1 >> 8U)  & 0xFFU);
  d[4] = (uint8_t)( p->p1         & 0xFFU);
  d[5] = (uint8_t)((((p->p3 >> 12U) & 0xF0U)) |
                   (( p->p2 >> 16U) & 0x0FU));
  d[6] = (uint8_t)((p->p2 >> 8U)  & 0xFFU);
  d[7] = (uint8_t)( p->p2         & 0xFFU);
  return SI5351_WriteBurst(si, base_reg, d, 8U);
}

/* USER CODE END 0 */

/* ──────────────────────────────────────────────────────────────────── */
/*                   Exported utility functions                         */
/* ──────────────────────────────────────────────────────────────────── */

/**
  * @brief  Tính tham số PLL (MSNA hoặc MSNB) từ VCO target.
  *
  *  Công thức (AN619):
  *   f_vco = f_xtal × (a + b/c) = f_xtal × (P1 + 512 + P2/P3) / 128
  *
  *   a = floor(f_vco / f_xtal)
  *   b/c = (f_vco - a × f_xtal) / f_xtal = remainder/f_xtal
  *   Chọn c = f_xtal (cho độ chính xác tối đa), b = remainder
  *   Rồi rút gọn GCD(b, c).
  *
  *   P1 = 128×a + floor(128×b/c) - 512
  *   P2 = 128×b - c×floor(128×b/c)
  *   P3 = c
  *
  * @param  xtal_hz  Tần số XTAL (Hz)
  * @param  vco_hz   Tần số VCO mong muốn (Hz)
  * @param  params   Output tham số P1, P2, P3
  */
void SI5351_CalcPLL(uint32_t xtal_hz, uint32_t vco_hz, SI5351_MSParams_t *params)
{
  /* USER CODE BEGIN SI5351_CalcPLL_0 */
  uint32_t a = vco_hz / xtal_hz;
  uint32_t remainder = vco_hz - a * xtal_hz;

  /* Dùng denominator = xtal_hz để tối đa độ chính xác */
  uint32_t b = remainder;
  uint32_t c = xtal_hz;

  /* Rút gọn bằng GCD (Euclidean) */
  uint32_t tmp_b = b, tmp_c = c;
  while (tmp_c) { uint32_t t = tmp_c; tmp_c = tmp_b % tmp_c; tmp_b = t; }
  uint32_t gcd = (tmp_b > 0U) ? tmp_b : 1U;
  b /= gcd; c /= gcd;

  /* Giới hạn c ≤ 0xFFFFF (20-bit) */
  while (c > 0xFFFFFU) { b >>= 1U; c >>= 1U; }

  uint32_t floor_128bc = (c > 0U) ? (128U * b / c) : 0U;

  params->p3 = c;
  params->p1 = 128U * a + floor_128bc - 512U;
  params->p2 = 128U * b - c * floor_128bc;
  /* USER CODE END SI5351_CalcPLL_0 */
}

/**
  * @brief  Tính tham số Output MultiSynth từ integer divider.
  *
  *  Integer mode (P2=0, P3=1):
  *   P1 = 128 × div - 512
  *
  * @param  ms_div   Integer divider (phải là số chẵn cho quadrature)
  * @param  params   Output P1, P2, P3
  */
void SI5351_CalcMS(uint32_t ms_div, SI5351_MSParams_t *params)
{
  /* USER CODE BEGIN SI5351_CalcMS_0 */
  params->p1 = 128U * ms_div - 512U;
  params->p2 = 0U;
  params->p3 = 1U;
  /* USER CODE END SI5351_CalcMS_0 */
}

/**
  * @brief  Compute optimal MS_div and R_div for the requested output frequency.
  *
  *  Even-MS_div constraint removed: the external 74LVC74 ÷4 generates
  *  quadrature in hardware, so any integer MS_div is valid.
  *  Max MS_div raised to 2047 (18-bit P1 field limit of the Si5351).
  *
  * @param  freq_hz     Desired output frequency (Hz) – already ×4 for LO use
  * @param  xtal_hz     XTAL frequency (Hz)
  * @param  r_div_code  [out] R_DIV code (SI5351_R_DIV_x)
  * @param  vco_hz      [out] Actual VCO_A frequency to be programmed
  * @retval uint32_t    MS_div (0 if no valid combination found)
  */
uint32_t SI5351_CalcMSDiv(uint32_t freq_hz, uint32_t xtal_hz,
                            uint8_t *r_div_code, uint32_t *vco_hz)
{
  /* USER CODE BEGIN SI5351_CalcMSDiv_0 */
  static const uint32_t r_vals[8] = { 1U, 2U, 4U, 8U, 16U, 32U, 64U, 128U };

  for (uint8_t ri = 0U; ri < 8U; ri++)
  {
    uint32_t f_eff = freq_hz * r_vals[ri];
    if (f_eff == 0U) { continue; }

    /* MS_div nearest to VCO_TARGET */
    uint32_t ms = SI5351_VCO_TARGET_HZ / f_eff;

    /* Clamp to hardware limits (min=6, max=2047 for 18-bit integer P1) */
    if (ms < 6U)    { ms = 6U;    }
    if (ms > 2047U) { continue;   }

    uint32_t vco = f_eff * ms;
    if (vco < SI5351_VCO_MIN_HZ || vco > SI5351_VCO_MAX_HZ) { continue; }

    *r_div_code = ri;
    *vco_hz     = vco;
    return ms;
  }

  /* Fallback: R_div=128, MS_div=6 */
  *r_div_code = SI5351_R_DIV_128;
  *vco_hz     = freq_hz * 128U * 6U;
  return 6U;
  /* USER CODE END SI5351_CalcMSDiv_0 */
}

/* ──────────────────────────────────────────────────────────────────── */
/*                     Public API Functions                            */
/* ──────────────────────────────────────────────────────────────────── */

/**
  * @brief  Khởi tạo SI5351A.
  *
  *  Chuỗi:
  *   1. Chờ SYS_INIT bit = 0 (device ready)
  *   2. Cài crystal load (10pF mặc định)
  *   3. Tắt tất cả outputs (Reg3 = 0xFF)
  *   4. Power down tất cả CLK controls
  *   5. Xóa interrupt status
  *
  * @param  si          Handle
  * @param  hi2c        I2C handle (chia sẻ với WM8731)
  * @param  i2c_addr    8-bit I2C addr (0xC0)
  * @param  xtal_hz     XTAL frequency
  */
HAL_StatusTypeDef SI5351_Init(SI5351_Handle_t *si,
                               I2C_HandleTypeDef *hi2c,
                               uint8_t i2c_addr,
                               uint32_t xtal_hz)
{
  /* USER CODE BEGIN SI5351_Init_0 */
  HAL_StatusTypeDef ret;
  uint8_t status;

  si->hi2c         = hi2c;
  si->i2c_addr     = i2c_addr;
  si->xtal_hz      = xtal_hz;
  si->vco_a_hz     = 0U;
  si->freq_hz      = 0U;
  si->initialized  = false;

  for (uint8_t i = 0U; i < 3U; i++) {
    si->clk[i].enabled = false;
    si->clk[i].freq_hz = 0U;
  }

  /* 1. Chờ SYS_INIT = 0 (max 200ms) */
  uint32_t t0 = HAL_GetTick();
  do {
    ret = SI5351_ReadReg(si, SI5351_REG_DEV_STATUS, &status);
    if (ret != HAL_OK) { return ret; }
    if ((HAL_GetTick() - t0) > 200U) { return HAL_TIMEOUT; }
  } while (status & 0x80U);   /* bit7 = SYS_INIT */

  HAL_Delay(SI5351_INIT_DELAY_MS);

  /* 2. Crystal load = 10pF */
  ret = SI5351_WriteReg(si, SI5351_REG_CRYSTAL_LOAD, 0xD2U);  /* 0b11010010 */
  if (ret != HAL_OK) { return ret; }

  /* 3. Disable tất cả outputs */
  ret = SI5351_WriteReg(si, SI5351_REG_OUTPUT_EN_CTRL, 0xFFU);
  if (ret != HAL_OK) { return ret; }

  /* 4. Power down tất cả CLK */
  for (uint8_t i = 0U; i < 8U; i++) {
    ret = SI5351_WriteReg(si, (uint8_t)(SI5351_REG_CLK0_CTRL + i),
                           SI5351_CLK_PDN | SI5351_CLK_SRC_MS | SI5351_CLK_IDRV_8MA);
    if (ret != HAL_OK) { return ret; }
  }

  /* 5. Xóa interrupt status */
  ret = SI5351_WriteReg(si, SI5351_REG_INT_STATUS_STK, 0x00U);
  if (ret != HAL_OK) { return ret; }

  si->initialized = true;
  /* USER CODE END SI5351_Init_0 */
  return HAL_OK;
}

/**
  * @brief  Set RX LO frequency.
  *
  *  Programs CLK0 at freq_hz × 4 on PLL_A.
  *  CLK1 is not used (74LVC74 external hardware generates 90° phase).
  *  No phase-offset register is written.
  *
  * @param  si       Handle
  * @param  freq_hz  RF LO frequency (Hz) – function multiplies by 4 internally
  */
HAL_StatusTypeDef SI5351_SetQSDFrequency(SI5351_Handle_t *si, uint32_t freq_hz)
{
  /* USER CODE BEGIN SI5351_SetQSDFreq_0 */
  if (!si->initialized) { return HAL_ERROR; }

  HAL_StatusTypeDef ret;

  /* ── 1. CLK output frequency = RF LO × 4 ─────────────── */
  uint32_t clk_freq = freq_hz * 4U;

  /* ── 2. Compute PLL_A / MS0 parameters ───────────────── */
  uint8_t  r_div_code;
  uint32_t vco_hz;
  uint32_t ms_div = SI5351_CalcMSDiv(clk_freq, si->xtal_hz, &r_div_code, &vco_hz);
  if (ms_div == 0U) { return HAL_ERROR; }

  /* ── 3. Disable outputs while reprogramming ──────────── */
  ret = SI5351_WriteReg(si, SI5351_REG_OUTPUT_EN_CTRL, 0xFFU);
  if (ret != HAL_OK) { return ret; }

  /* ── 4. Write PLL_A ───────────────────────────────────── */
  SI5351_MSParams_t pll_params;
  SI5351_CalcPLL(si->xtal_hz, vco_hz, &pll_params);
  ret = SI5351_WriteMS(si, SI5351_REG_MSNA_BASE, &pll_params, 0U);
  if (ret != HAL_OK) { return ret; }

  /* ── 5. Write MS0 (CLK0 divider) ─────────────────────── */
  SI5351_MSParams_t ms_params;
  SI5351_CalcMS(ms_div, &ms_params);
  ret = SI5351_WriteMS(si, SI5351_REG_MS0_BASE, &ms_params, r_div_code);
  if (ret != HAL_OK) { return ret; }

  /* ── 6. CLK0 phase offset = 0 (no software quadrature) ── */
  ret = SI5351_WriteReg(si, SI5351_REG_CLK0_PHOFF, 0U);
  if (ret != HAL_OK) { return ret; }

  /* ── 7. CLK0 control: PLL_A source, MS divider, 8 mA ─── */
  ret = SI5351_WriteReg(si, SI5351_REG_CLK0_CTRL,
                         (uint8_t)(SI5351_CLK_SRC_MS | SI5351_CLK_IDRV_8MA));
  if (ret != HAL_OK) { return ret; }

  /* ── 8. Reset PLL_A ───────────────────────────────────── */
  ret = SI5351_WriteReg(si, SI5351_REG_PLL_RESET, SI5351_PLLA_RESET);
  if (ret != HAL_OK) { return ret; }

  HAL_Delay(1U);   /* Wait for PLL lock */

  /* ── 9. Enable CLK0 only; CLK1-7 disabled → 0xFE ───────
   *  Reg3: bit=0 enables, bit=1 disables.
   *  0xFE = 1111 1110: CLK0 on, CLK1-7 off.
   *  CLK2 (TX LO) is re-enabled by SI5351_SetQSEFrequency()
   *  and disabled by SI5351_EnableOutput() before this call. */
  ret = SI5351_WriteReg(si, SI5351_REG_OUTPUT_EN_CTRL, 0xFEU);
  if (ret != HAL_OK) { return ret; }

  /* Update state */
  si->vco_a_hz            = vco_hz;
  si->freq_hz             = freq_hz;
  si->clk[0].enabled      = true;
  si->clk[0].freq_hz      = freq_hz;   /* store RF freq, not ×4 */
  si->clk[0].phase_offset = 0U;
  si->clk[0].r_div_code   = r_div_code;
  si->clk[0].ms_div       = ms_div;
  si->clk[1].enabled      = false;     /* CLK1 unused */

  /* USER CODE END SI5351_SetQSDFreq_0 */
  return HAL_OK;
}

/**
  * @brief  Set TX LO frequency.
  *
  *  Programs CLK2 at freq_hz × 4 on PLL_B (independent of PLL_A / CLK0).
  *  External 74LVC74 divides by 4 and generates TX I/Q LO phases.
  *
  * @param  si       Handle
  * @param  freq_hz  TX LO frequency (Hz) – function multiplies by 4 internally
  */
HAL_StatusTypeDef SI5351_SetQSEFrequency(SI5351_Handle_t *si, uint32_t freq_hz)
{
  /* USER CODE BEGIN SI5351_SetQSEFreq_0 */
  if (!si->initialized) { return HAL_ERROR; }

  HAL_StatusTypeDef ret;

  /* ── 1. CLK2 output frequency = TX LO × 4 ─────────────── */
  uint32_t clk_freq = freq_hz * 4U;

  uint8_t  r_div_code;
  uint32_t vco_hz;
  uint32_t ms_div = SI5351_CalcMSDiv(clk_freq, si->xtal_hz, &r_div_code, &vco_hz);
  if (ms_div == 0U) { return HAL_ERROR; }

  /* ── 2. Write PLL_B ───────────────────────────────────── */
  SI5351_MSParams_t pll_params;
  SI5351_CalcPLL(si->xtal_hz, vco_hz, &pll_params);
  ret = SI5351_WriteMS(si, SI5351_REG_MSNB_BASE, &pll_params, 0U);
  if (ret != HAL_OK) { return ret; }

  /* ── 3. Write MS2 (CLK2 divider) ─────────────────────── */
  SI5351_MSParams_t ms_params;
  SI5351_CalcMS(ms_div, &ms_params);
  ret = SI5351_WriteMS(si, SI5351_REG_MS2_BASE, &ms_params, r_div_code);
  if (ret != HAL_OK) { return ret; }

  /* ── 4. CLK2 control: PLL_B source, MS divider, 8 mA ─── */
  ret = SI5351_WriteReg(si, SI5351_REG_CLK2_CTRL,
                         (uint8_t)(SI5351_CLK_MS_SRC_PLLB | SI5351_CLK_SRC_MS |
                                   SI5351_CLK_IDRV_8MA));
  if (ret != HAL_OK) { return ret; }

  /* ── 5. Reset PLL_B ───────────────────────────────────── */
  ret = SI5351_WriteReg(si, SI5351_REG_PLL_RESET, SI5351_PLLB_RESET);
  if (ret != HAL_OK) { return ret; }

  HAL_Delay(1U);

  /* ── 6. Enable CLK0 (if active) + CLK2; CLK1 stays off ─ */
  /* 0xFEU = only CLK0 on; clear bit2 to also enable CLK2   */
  uint8_t en_mask = si->clk[0].enabled ? 0xFEU : 0xFFU;
  en_mask &= ~(uint8_t)(1U << 2U);
  ret = SI5351_WriteReg(si, SI5351_REG_OUTPUT_EN_CTRL, en_mask);
  if (ret != HAL_OK) { return ret; }

  si->clk[2].enabled    = true;
  si->clk[2].freq_hz    = freq_hz;   /* store TX RF freq, not ×4 */
  si->clk[2].r_div_code = r_div_code;
  si->clk[2].ms_div     = ms_div;

  /* USER CODE END SI5351_SetQSEFreq_0 */
  return HAL_OK;
}

/**
  * @brief  Bật / tắt một CLK output.
  * @param  clk_num  0, 1, hoặc 2
  * @param  enable   true=bật, false=tắt
  */
HAL_StatusTypeDef SI5351_EnableOutput(SI5351_Handle_t *si, uint8_t clk_num, bool enable)
{
  /* USER CODE BEGIN SI5351_EnableOutput_0 */
  if (clk_num > 7U) { return HAL_ERROR; }
  uint8_t en_reg;
  HAL_StatusTypeDef ret = SI5351_ReadReg(si, SI5351_REG_OUTPUT_EN_CTRL, &en_reg);
  if (ret != HAL_OK) { return ret; }

  if (enable) { en_reg &= ~(uint8_t)(1U << clk_num); }
  else        { en_reg |=  (uint8_t)(1U << clk_num); }

  if (clk_num < 3U) { si->clk[clk_num].enabled = enable; }
  return SI5351_WriteReg(si, SI5351_REG_OUTPUT_EN_CTRL, en_reg);
  /* USER CODE END SI5351_EnableOutput_0 */
}

/**
  * @brief  Cài drive strength cho một CLK.
  * @param  drive_ma  SI5351_DRIVE_2MA .. SI5351_DRIVE_8MA
  */
HAL_StatusTypeDef SI5351_SetDrive(SI5351_Handle_t *si, uint8_t clk_num, uint8_t drive_ma)
{
  /* USER CODE BEGIN SI5351_SetDrive_0 */
  if (clk_num > 7U) { return HAL_ERROR; }
  uint8_t reg_addr = (uint8_t)(SI5351_REG_CLK0_CTRL + clk_num);
  uint8_t val;
  HAL_StatusTypeDef ret = SI5351_ReadReg(si, reg_addr, &val);
  if (ret != HAL_OK) { return ret; }
  val = (uint8_t)((val & 0xFCU) | (drive_ma & 0x03U));
  return SI5351_WriteReg(si, reg_addr, val);
  /* USER CODE END SI5351_SetDrive_0 */
}

/**
  * @brief  Reset PLL (gọi sau khi đã thay đổi VCO parameters).
  */
HAL_StatusTypeDef SI5351_ResetPLL(SI5351_Handle_t *si)
{
  /* USER CODE BEGIN SI5351_ResetPLL_0 */
  return SI5351_WriteReg(si, SI5351_REG_PLL_RESET,
                          SI5351_PLLA_RESET | SI5351_PLLB_RESET);
  /* USER CODE END SI5351_ResetPLL_0 */
}

/**
  * @brief  Đọc Device Status register.
  */
HAL_StatusTypeDef SI5351_GetStatus(SI5351_Handle_t *si, uint8_t *status)
{
  /* USER CODE BEGIN SI5351_GetStatus_0 */
  return SI5351_ReadReg(si, SI5351_REG_DEV_STATUS, status);
  /* USER CODE END SI5351_GetStatus_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
