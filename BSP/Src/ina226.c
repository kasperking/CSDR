/* USER CODE BEGIN Header */
/**
  * @file  ina226.c
  * @brief INA226 current/power monitor driver — low-level register access.
  *        Không phụ thuộc vào logic ứng dụng, có thể dùng lại tự do.
  */
/* USER CODE END Header */

#include "ina226.h"

/* ─── Internal helpers ───────────────────────────────────────────────────── */

/* HAL I2C timeout: worst-case for 3-byte I2C transaction at 100 kHz is ~270 µs.
 * 2 ms gives 7× headroom while keeping the main-loop stall negligible even if
 * the device is absent (worst case: 2 ms × 2 calls = 4 ms vs the old 20 ms). */
#define INA226_I2C_TIMEOUT_MS  2U

static HAL_StatusTypeDef write_reg(const INA226_Handle_t *h,
                                   uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFFU) };
    return HAL_I2C_Master_Transmit(h->hi2c, h->dev_addr, buf, 3, INA226_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_reg(const INA226_Handle_t *h,
                                  uint8_t reg, uint16_t *out)
{
    uint8_t rx[2];
    HAL_StatusTypeDef st;
    st = HAL_I2C_Master_Transmit(h->hi2c, h->dev_addr, &reg, 1, INA226_I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;
    st = HAL_I2C_Master_Receive(h->hi2c, h->dev_addr, rx, 2, INA226_I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;
    *out = ((uint16_t)rx[0] << 8) | rx[1];
    return HAL_OK;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef INA226_Init(INA226_Handle_t *h,
                               I2C_HandleTypeDef *hi2c,
                               uint8_t addr_7bit,
                               float shunt_ohm)
{
    h->hi2c      = hi2c;
    h->dev_addr  = (uint16_t)((uint16_t)addr_7bit << 1U);
    h->shunt_ohm = shunt_ohm;

    /* Soft reset: bit 15 của CONFIG, self-clearing sau 1 chu kỳ bus */
    HAL_StatusTypeDef st = write_reg(h, INA226_REG_CONFIG, 0x8000U);
    HAL_Delay(2);
    return st;
}

HAL_StatusTypeDef INA226_WriteReg(const INA226_Handle_t *h,
                                   uint8_t reg, uint16_t val)
{
    return write_reg(h, reg, val);
}

HAL_StatusTypeDef INA226_ReadReg(const INA226_Handle_t *h,
                                  uint8_t reg, uint16_t *out)
{
    return read_reg(h, reg, out);
}

HAL_StatusTypeDef INA226_Configure(const INA226_Handle_t *h,
                                    uint16_t cfg, uint16_t mask_en)
{
    HAL_StatusTypeDef st;
    st = write_reg(h, INA226_REG_CONFIG,  cfg);     if (st != HAL_OK) return st;
    st = write_reg(h, INA226_REG_MASK_EN, mask_en);
    return st;
}

HAL_StatusTypeDef INA226_SetAlertLimitAmps(const INA226_Handle_t *h,
                                            float current_a)
{
    /* Alert Limit register = V_shunt / LSB
     * V_shunt (µV) = I × R_shunt × 1e6
     * LSB = 2.5 µV
     *
     * Ví dụ: 3.5A × 0.02Ω = 70000 µV / 2.5 = 28000 = 0x6D60 */
    float v_uv = current_a * h->shunt_ohm * 1.0e6f;
    uint32_t raw = (uint32_t)(v_uv / INA226_SHUNT_V_LSB_UV + 0.5f);
    if (raw > 0x7FFFU) raw = 0x7FFFU;  /* clamp: signed 15-bit max */
    return write_reg(h, INA226_REG_ALERT_LIM, (uint16_t)raw);
}

float INA226_ReadCurrentAmps(const INA226_Handle_t *h)
{
    uint16_t raw = 0;
    if (read_reg(h, INA226_REG_SHUNT_V, &raw) != HAL_OK) return 0.0f;

    /* Shunt Voltage Register: signed 16-bit, LSB = 2.5 µV */
    int16_t signed_raw = (int16_t)raw;
    float v_shunt_v = (float)signed_raw * (INA226_SHUNT_V_LSB_UV * 1.0e-6f);
    return v_shunt_v / h->shunt_ohm;
}

float INA226_ReadBusVolts(const INA226_Handle_t *h)
{
    uint16_t raw = 0;
    if (read_reg(h, INA226_REG_BUS_V, &raw) != HAL_OK) return 0.0f;

    /* Bus Voltage Register: unsigned 16-bit, LSB = 1.25 mV */
    return (float)raw * (INA226_BUS_V_LSB_MV * 1.0e-3f);
}

HAL_StatusTypeDef INA226_ReadAndClearLatch(const INA226_Handle_t *h,
                                            bool *aff_out)
{
    uint16_t val = 0;
    HAL_StatusTypeDef st = read_reg(h, INA226_REG_MASK_EN, &val);
    if (st != HAL_OK) return st;

    /* Đọc reg 06h tự động xóa AFF bit và nhả ALERT pin khỏi trạng thái latch */
    if (aff_out) *aff_out = (val & INA226_MASK_AFF) != 0U;
    return HAL_OK;
}
