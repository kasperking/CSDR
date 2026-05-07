/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    si5351.h
  * @brief   SI5351A Programmable Clock Generator BSP Driver
  *
  *  Hardware architecture (74LVC74 divide-by-4 quadrature):
  *   - CLK0  → 4 × LO → external 74LVC74 ÷4 → I and Q phases (0°/90°)
  *             shared by BOTH RX QSD and TX QSE
  *   - CLK1  → unused (powered down)
  *   - CLK2  → unused (powered down)
  *
  *  The 74LVC74 dual D flip-flop generates precise 90° quadrature from a
  *  single 4× clock with no software phase-offset registers involved.
  *  RX and TX are phase-coherent because they share the same LO source.
  *
  *  LO programming rule:
  *   SI5351_SetQSDFrequency(si, rf_hz)  → CLK0 = rf_hz × 4  (shared LO)
  *   Callers always pass the RF frequency; the ×4 is internal.
  *
  *  Constraints:
  *   • VCO_A in [600 MHz, 900 MHz]
  *   • MS_div: any integer ≥ 6, ≤ 2047 (even-only constraint removed)
  *   • R_div (÷1..÷128) used for very low CLK frequencies
  *
  *  I2C: 0x60 (7-bit), shared with WM8731
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SI5351_H
#define __SI5351_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* I2C address */
#define SI5351_I2C_ADDR_DEFAULT    (0x60U << 1U)   /*!< FS=GND → 0xC0 */

/* XTAL reference */
#define SI5351_XTAL_FREQ_HZ        27000000UL       /*!< 27 MHz onboard XTAL */

/* VCO range (PLL_A / PLL_B) */
#define SI5351_VCO_MIN_HZ          600000000UL
#define SI5351_VCO_MAX_HZ          900000000UL
#define SI5351_VCO_TARGET_HZ       750000000UL      /*!< Điểm giữa tối ưu */

/* Output drive strength */
#define SI5351_DRIVE_2MA           0x00U
#define SI5351_DRIVE_4MA           0x01U
#define SI5351_DRIVE_6MA           0x02U
#define SI5351_DRIVE_8MA           0x03U

/* ── Register map ──────────────────────────────────────── */
#define SI5351_REG_DEV_STATUS      0U
#define SI5351_REG_INT_STATUS_STK  1U
#define SI5351_REG_INT_STATUS_MASK 2U
#define SI5351_REG_OUTPUT_EN_CTRL  3U    /*!< Bit per CLK, 0=enable */
#define SI5351_REG_OEB_PIN_EN      9U
#define SI5351_REG_PLL_SRC         15U
#define SI5351_REG_CLK0_CTRL       16U
#define SI5351_REG_CLK1_CTRL       17U
#define SI5351_REG_CLK2_CTRL       18U
#define SI5351_REG_CLK3_CTRL       19U
#define SI5351_REG_CLK4_CTRL       20U
#define SI5351_REG_CLK5_CTRL       21U
#define SI5351_REG_CLK6_CTRL       22U
#define SI5351_REG_CLK7_CTRL       23U
#define SI5351_REG_CLK3_0_DISABLE  24U
#define SI5351_REG_CLK7_4_DISABLE  25U
/* PLL_A Multisynth (Reg 26-33) */
#define SI5351_REG_MSNA_BASE       26U
/* PLL_B Multisynth (Reg 34-41) */
#define SI5351_REG_MSNB_BASE       34U
/* MS0..MS7 Output dividers */
#define SI5351_REG_MS0_BASE        42U   /*!< CLK0: Reg 42-49 */
#define SI5351_REG_MS1_BASE        50U   /*!< CLK1: Reg 50-57 */
#define SI5351_REG_MS2_BASE        58U   /*!< CLK2: Reg 58-65 */
/* Spread spectrum, VCXO: 149-160 */
#define SI5351_REG_CLK0_PHOFF      165U  /*!< CLK0 phase offset [6:0] */
#define SI5351_REG_CLK1_PHOFF      166U  /*!< CLK1 phase offset [6:0] */
#define SI5351_REG_CLK2_PHOFF      167U  /*!< CLK2 phase offset [6:0] */
#define SI5351_REG_PLL_RESET       177U  /*!< bit5=PLLB_RST, bit2=PLLA_RST */
#define SI5351_REG_CRYSTAL_LOAD    183U

/* CLK_CTRL bit fields (Reg 16-23) */
#define SI5351_CLK_PDN             (1U << 7U)   /*!< Power down */
#define SI5351_CLK_MS_SRC_PLLB    (1U << 5U)   /*!< 0=PLLA, 1=PLLB */
#define SI5351_CLK_INV            (1U << 4U)   /*!< Invert output */
#define SI5351_CLK_SRC_MS          (3U << 2U)   /*!< CLK src = MultiSynth */
#define SI5351_CLK_SRC_XTAL        (0U << 2U)
#define SI5351_CLK_IDRV_8MA        (3U << 0U)
#define SI5351_CLK_IDRV_6MA        (2U << 0U)
#define SI5351_CLK_IDRV_4MA        (1U << 0U)
#define SI5351_CLK_IDRV_2MA        (0U << 0U)

/* PLL_RESET bits */
#define SI5351_PLLA_RESET          (1U << 2U)
#define SI5351_PLLB_RESET          (1U << 5U)

/* Crystal load capacitance */
#define SI5351_XTAL_LOAD_6PF       (1U << 6U)
#define SI5351_XTAL_LOAD_8PF       (2U << 6U)
#define SI5351_XTAL_LOAD_10PF      (3U << 6U)

/* R divider values (encoded in MS register byte 2 bits[6:4]) */
#define SI5351_R_DIV_1             0U
#define SI5351_R_DIV_2             1U
#define SI5351_R_DIV_4             2U
#define SI5351_R_DIV_8             3U
#define SI5351_R_DIV_16            4U
#define SI5351_R_DIV_32            5U
#define SI5351_R_DIV_64            6U
#define SI5351_R_DIV_128           7U

/* Exported types ------------------------------------------------------------*/

/** Kết quả tính toán tham số PLL */
typedef struct {
  uint32_t p1;   /*!< MS P1 parameter */
  uint32_t p2;   /*!< MS P2 parameter */
  uint32_t p3;   /*!< MS P3 parameter */
} SI5351_MSParams_t;

/** Trạng thái một CLK output */
typedef struct {
  bool     enabled;
  uint32_t freq_hz;
  uint8_t  phase_offset;  /*!< Thanh ghi phase offset [0..127] */
  uint8_t  r_div_code;    /*!< SI5351_R_DIV_x */
  uint32_t ms_div;        /*!< Integer MultiSynth divider */
} SI5351_CLK_State_t;

/** Handle chính của driver */
typedef struct {
  I2C_HandleTypeDef *hi2c;          /*!< I2C handle (chung với WM8731)   */
  uint8_t            i2c_addr;      /*!< 8-bit I2C addr (0xC0 mặc định) */
  uint32_t           xtal_hz;       /*!< XTAL frequency (Hz)             */
  uint32_t           vco_a_hz;      /*!< VCO_A frequency hiện tại        */
  uint32_t           freq_hz;       /*!< Tần số output hiện tại          */
  SI5351_CLK_State_t clk[3];        /*!< Trạng thái CLK0..CLK2           */
  bool               initialized;
} SI5351_Handle_t;

/* Exported functions prototypes ---------------------------------------------*/

/** Khởi tạo SI5351 */
HAL_StatusTypeDef SI5351_Init(SI5351_Handle_t *si,
                               I2C_HandleTypeDef *hi2c,
                               uint8_t i2c_addr,
                               uint32_t xtal_hz);

/** Set shared LO: programs CLK0 at freq_hz × 4 for 74LVC74 ÷4 quadrature.
 *  Used for both RX QSD and TX QSE — CLK1 and CLK2 remain powered down. */
HAL_StatusTypeDef SI5351_SetQSDFrequency(SI5351_Handle_t *si, uint32_t freq_hz);

/** Bật/tắt từng CLK output */
HAL_StatusTypeDef SI5351_EnableOutput(SI5351_Handle_t *si, uint8_t clk_num, bool enable);

/** Cài drive strength */
HAL_StatusTypeDef SI5351_SetDrive(SI5351_Handle_t *si, uint8_t clk_num, uint8_t drive_ma);

/** Reset PLL (cần sau khi thay đổi VCO) */
HAL_StatusTypeDef SI5351_ResetPLL(SI5351_Handle_t *si);

/** Đọc trạng thái device (kiểm tra SYS_INIT bit) */
HAL_StatusTypeDef SI5351_GetStatus(SI5351_Handle_t *si, uint8_t *status);

/** Tính tham số PLL/MS từ tần số (utility, public để test) */
void SI5351_CalcPLL(uint32_t xtal_hz, uint32_t vco_hz, SI5351_MSParams_t *params);
void SI5351_CalcMS(uint32_t ms_div, SI5351_MSParams_t *params);
uint32_t SI5351_CalcMSDiv(uint32_t freq_hz, uint32_t xtal_hz,
                            uint8_t *r_div_code, uint32_t *vco_hz);

/* Exported variables --------------------------------------------------------*/
extern SI5351_Handle_t g_si5351;

#ifdef __cplusplus
}
#endif
#endif /* __SI5351_H */
