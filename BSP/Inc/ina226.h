/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ina226.h
  * @brief   INA226 bidirectional current/power monitor driver (I2C)
  *
  *  Driver thuần phần cứng — không phụ thuộc vào bất kỳ logic ứng dụng nào.
  *  Có thể dùng lại cho bất kỳ mục đích đo dòng/áp nào trên bus I2C.
  *
  *  Địa chỉ I2C (7-bit):
  *    A1=GND A0=GND → 0x40   A1=GND A0=VS  → 0x41
  *    A1=GND A0=SDA → 0x42   A1=GND A0=SCL → 0x43
  *    A1=VS  A0=GND → 0x44   ...            → 0x45..0x4F
  *
  *  Thanh ghi chính:
  *    00h CONFIG    — averaging, conversion time, operating mode
  *    01h SHUNT_V   — shunt voltage (signed 16-bit, LSB = 2.5 µV)
  *    02h BUS_V     — bus voltage  (unsigned,       LSB = 1.25 mV)
  *    05h CALIB     — calibration (tuỳ chọn, không dùng cho alert-only)
  *    06h MASK_EN   — alert source select + AFF flag + latch control
  *    07h ALERT_LIM — alert threshold (cùng đơn vị với nguồn alert được chọn)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __INA226_H
#define __INA226_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ─── Register addresses ─────────────────────────────────────────────────── */
#define INA226_REG_CONFIG       0x00U
#define INA226_REG_SHUNT_V      0x01U
#define INA226_REG_BUS_V        0x02U
#define INA226_REG_POWER        0x03U
#define INA226_REG_CURRENT      0x04U
#define INA226_REG_CALIB        0x05U
#define INA226_REG_MASK_EN      0x06U
#define INA226_REG_ALERT_LIM    0x07U
#define INA226_REG_MFR_ID       0xFEU
#define INA226_REG_DIE_ID       0xFFU

/* ─── CONFIG register bit-fields (reg 00h) ───────────────────────────────────
 *
 *  Bit 15    : RST  — software reset (self-clearing)
 *  Bits 14-12: AVG  — number of samples to average
 *  Bits 11-9 : VBUS_CT — bus voltage conversion time
 *  Bits  8-6 : VSH_CT  — shunt voltage conversion time
 *  Bits  5-3 : reserved (always read as 100; write as 100 → bit 5 = 1)
 *  Bits  2-0 : MODE — operating mode
 *
 *  Conversion time options (both VBUS_CT and VSH_CT):
 *    000 = 140 µs    001 = 204 µs    010 = 332 µs    011 = 588 µs
 *    100 = 1.1 ms    101 = 2.116 ms  110 = 4.156 ms  111 = 8.244 ms
 *
 *  AVG options:
 *    000=1  001=4  010=16  011=64  100=128  101=256  110=512  111=1024
 *
 *  MODE options:
 *    000/100 = Power-Down   001 = Shunt Triggered    010 = Bus Triggered
 *    011 = Shunt+Bus Triggered    101 = Shunt Continuous
 *    110 = Bus Continuous         111 = Shunt+Bus Continuous  ← default
 * ─────────────────────────────────────────────────────────────────────────── */

/* Preset configurations — combine with INA226_MODE_* constants */
#define INA226_CFG_AVG_1        (0x0000U)   /*!< No averaging (1 sample)       */
#define INA226_CFG_AVG_4        (0x0200U)
#define INA226_CFG_AVG_16       (0x0400U)
#define INA226_CFG_AVG_64       (0x0600U)
#define INA226_CFG_CT_140US     (0x0000U)   /*!< 140 µs conversion time        */
#define INA226_CFG_CT_204US     (0x0040U)   /*!< VBUS_CT and VSH_CT combined   */
#define INA226_CFG_CT_332US     (0x0080U)
#define INA226_CFG_CT_588US     (0x00C0U)
#define INA226_CFG_CT_1100US    (0x0100U)
#define INA226_CFG_RSVD         (0x0020U)   /*!< Reserved bit 5 — always write 1 */
#define INA226_CFG_MODE_CONT_SB (0x0007U)   /*!< Shunt+Bus, Continuous          */
#define INA226_CFG_MODE_CONT_S  (0x0005U)   /*!< Shunt only, Continuous         */
#define INA226_CFG_MODE_PDWN    (0x0000U)   /*!< Power-down                     */

/* Fastest possible — no averaging, 140 µs each, continuous shunt+bus */
#define INA226_CFG_FASTEST  \
    (INA226_CFG_AVG_1 | INA226_CFG_CT_140US | \
     (INA226_CFG_CT_140US << 3) | INA226_CFG_RSVD | INA226_CFG_MODE_CONT_SB)
/* = 0x0000 | 0x0000 | 0x0000 | 0x0020 | 0x0007 = 0x0027 */

/* ─── MASK_EN register (reg 06h) ─────────────────────────────────────────────
 *  Bit 15: SOL  — Shunt Over-Limit      Bit 14: SUL — Shunt Under-Limit
 *  Bit 13: BOL  — Bus Over-Limit        Bit 12: BUL — Bus Under-Limit
 *  Bit 11: POL  — Power Over-Limit      Bit 10: CNVR — Conversion Ready
 *  Bit  4: AFF  — Alert Function Flag (read-only, cleared by reading reg 06h)
 *  Bit  3: CVRF — Conversion Ready Flag (read-only)
 *  Bit  2: OVF  — Math Overflow Flag   (read-only)
 *  Bit  1: APOL — Alert Polarity (0=active LOW, 1=active HIGH)
 *  Bit  0: LEN  — Alert Latch Enable (0=transparent, 1=latched)
 * ─────────────────────────────────────────────────────────────────────────── */
#define INA226_MASK_SOL         (1U << 15)  /*!< Shunt Over-Limit enable       */
#define INA226_MASK_SUL         (1U << 14)  /*!< Shunt Under-Limit enable      */
#define INA226_MASK_BOL         (1U << 13)  /*!< Bus Over-Limit enable         */
#define INA226_MASK_CNVR        (1U << 10)  /*!< Conversion Ready enable       */
#define INA226_MASK_AFF         (1U <<  4)  /*!< Alert Function Flag (r/o)     */
#define INA226_MASK_APOL        (1U <<  1)  /*!< Alert Polarity: 1=active HIGH */
#define INA226_MASK_LEN         (1U <<  0)  /*!< Latch Enable                  */

/* SOL alert, active-LOW, latched — đây là cấu hình dùng cho bảo vệ PA */
#define INA226_MASK_SOL_LATCH   (INA226_MASK_SOL | INA226_MASK_LEN)  /* 0x8001 */

/* ─── Physical constants ─────────────────────────────────────────────────── */
#define INA226_SHUNT_V_LSB_UV   2.5f    /*!< Shunt voltage LSB = 2.5 µV (fixed) */
#define INA226_BUS_V_LSB_MV     1.25f   /*!< Bus voltage LSB  = 1.25 mV (fixed) */

/* ─── Handle ─────────────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;    /*!< HAL I2C handle                            */
    uint16_t           dev_addr;/*!< 8-bit HAL address = (7-bit addr) << 1     */
    float              shunt_ohm;/*!< Shunt resistance (Ω) for current calc    */
} INA226_Handle_t;

/* ─── API ─────────────────────────────────────────────────────────────────── */

/**
  * @brief  Điền handle và soft-reset chip.
  *         Không cấu hình thanh ghi — gọi INA226_Configure() sau.
  * @param  h          Handle cần điền
  * @param  hi2c       HAL I2C handle
  * @param  addr_7bit  Địa chỉ 7-bit (ví dụ 0x40)
  * @param  shunt_ohm  Giá trị trở shunt (Ω)
  */
HAL_StatusTypeDef INA226_Init(INA226_Handle_t *h,
                               I2C_HandleTypeDef *hi2c,
                               uint8_t addr_7bit,
                               float shunt_ohm);

/** Ghi thẳng vào thanh ghi (big-endian, 16-bit). */
HAL_StatusTypeDef INA226_WriteReg(const INA226_Handle_t *h,
                                   uint8_t reg, uint16_t val);

/** Đọc thẳng từ thanh ghi (big-endian, 16-bit). */
HAL_StatusTypeDef INA226_ReadReg(const INA226_Handle_t *h,
                                  uint8_t reg, uint16_t *out);

/**
  * @brief  Cấu hình CONFIG và MASK_EN cùng lúc.
  * @param  cfg      Giá trị CONFIG (dùng INA226_CFG_* macros)
  * @param  mask_en  Giá trị MASK_EN (dùng INA226_MASK_* macros)
  */
HAL_StatusTypeDef INA226_Configure(const INA226_Handle_t *h,
                                    uint16_t cfg, uint16_t mask_en);

/**
  * @brief  Nạp Alert Limit register theo dòng điện (A).
  *         Alert_reg = (I × R_shunt × 1e6) / 2.5  [clamped to 0x7FFF]
  */
HAL_StatusTypeDef INA226_SetAlertLimitAmps(const INA226_Handle_t *h,
                                            float current_a);

/**
  * @brief  Đọc dòng điện hiện tại từ Shunt Voltage register (A).
  *         Trả về 0.0f nếu I2C lỗi.
  */
float INA226_ReadCurrentAmps(const INA226_Handle_t *h);

/**
  * @brief  Đọc điện áp bus (V).
  *         Trả về 0.0f nếu I2C lỗi.
  */
float INA226_ReadBusVolts(const INA226_Handle_t *h);

/**
  * @brief  Đọc Mask/Enable register để lấy trạng thái và xóa Alert Latch.
  *         Theo datasheet: đọc reg 06h tự động xóa AFF bit và nhả ALERT pin.
  * @param  aff_out  Nếu không NULL, nhận giá trị AFF (true = alert đã xảy ra)
  * @return HAL_OK hoặc mã lỗi I2C
  */
HAL_StatusTypeDef INA226_ReadAndClearLatch(const INA226_Handle_t *h,
                                            bool *aff_out);

#ifdef __cplusplus
}
#endif
#endif /* __INA226_H */
