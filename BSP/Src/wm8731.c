/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wm8731.c
  * @brief   WM8731 Audio Codec BSP Driver
  *
  *  Giao tiếp:
  *   - Control : I2C (2-byte frame: reg[6:0]+val[8], val[7:0])
  *   - Audio   : SAI1 I2S 32-bit stereo slave 48kHz
  *
  *  Chuỗi init: Reset → Power → Analog path → Digital path →
  *              Digital IF → Sampling → Volume → Active
  ******************************************************************************
  * @note  CSB = GND  →  I2C addr 0x1A (7-bit) / 0x34 (8-bit write)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "wm8731.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WM8731_TIMEOUT_MS    100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef WM8731_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t addr, uint8_t reg,
                                          uint16_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Ghi thanh ghi 9-bit của WM8731 qua I2C.
  *         Frame 2 byte: [reg6:0 | val8] [val7:0]
  */
static HAL_StatusTypeDef WM8731_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t addr, uint8_t reg,
                                          uint16_t val)
{
  uint8_t buf[2];
  buf[0] = (uint8_t)(((reg & 0x7FU) << 1U) | ((val >> 8U) & 0x01U));
  buf[1] = (uint8_t)(val & 0xFFU);
  return HAL_I2C_Master_Transmit(hi2c, addr, buf, 2U, WM8731_TIMEOUT_MS);
}

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Software reset WM8731.
  * @param  hi2c  I2C handle
  * @param  addr  Địa chỉ 8-bit (0x34)
  */
HAL_StatusTypeDef WM8731_Reset(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
  /* USER CODE BEGIN WM8731_Reset_0 */
  /* USER CODE END WM8731_Reset_0 */
  return WM8731_WriteReg(hi2c, addr, WM8731_REG_RESET, 0x0000U);
  /* USER CODE BEGIN WM8731_Reset_1 */
  /* USER CODE END WM8731_Reset_1 */
}

/**
  * @brief  Khởi tạo đầy đủ WM8731 theo WM8731_Config_t.
  * @param  cfg  Con trỏ cấu hình (không NULL)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef WM8731_Init(const WM8731_Config_t *cfg)
{
  /* USER CODE BEGIN WM8731_Init_0 */
  HAL_StatusTypeDef ret;
  I2C_HandleTypeDef *hi2c = cfg->hi2c;
  uint8_t            addr = cfg->i2c_addr;
  /* USER CODE END WM8731_Init_0 */

  /* 1. Software Reset */
  ret = WM8731_Reset(hi2c, addr);
  if (ret != HAL_OK) { return ret; }
  HAL_Delay(10U);

  /* 2. Power Down: bật LINE IN / ADC / DAC / OUT, tắt MIC / OSC / CLKOUT */
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_POWER_DOWN,
                         WM8731_MICPD | WM8731_OSCPD | WM8731_CLKOUTPD);
  if (ret != HAL_OK) { return ret; }

  /* 3. Analog Path */
  {
    uint16_t ap = WM8731_DACSEL | WM8731_MUTEMIC;
    if (!cfg->line_in)
    {
      ap &= ~(uint16_t)WM8731_MUTEMIC;
      ap |= (uint16_t)(WM8731_INSEL_MIC | WM8731_MICBOOST);
    }
    ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_ANALOG_PATH, ap);
    if (ret != HAL_OK) { return ret; }
  }

  /* 4. Digital Path: HPF on, no de-emphasis, DAC unmute */
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_DIGITAL_PATH, WM8731_DEEMP_NONE);
  if (ret != HAL_OK) { return ret; }

  /* 5. Digital Interface: I2S 32-bit Slave */
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_DIGITAL_IF,
                         WM8731_FORMAT_I2S | WM8731_IWL_32BIT | WM8731_MS_SLAVE);
  if (ret != HAL_OK) { return ret; }

  /* 6. Sampling: USB mode, 48kHz ADC + 48kHz DAC (MCLK=12.288MHz) */
  /* Normal mode (không USB mode): MCLK từ SAI = 12.288MHz = 256×48kHz
   * USB mode (bit0=1) cần 12.000MHz MCLK với internal PLL → sai với SAI output
   * SR=0x00: ADC 48kHz / DAC 48kHz, BOSR=0: 256fs */
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_SAMPLING,
                         WM8731_BOSR_250_256 |
                         WM8731_SR_48K_48K | WM8731_CLKIDIV2 | WM8731_CLKODIV2);
  /* Note: WM8731_USB_MODE removed – using normal mode with 12.288MHz MCLK */
  if (ret != HAL_OK)
  { return ret; }

  /* 7a. Line IN gain (LRINBOTH → cập nhật cả 2 kênh) */
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_LLINE_IN,
                         ((uint16_t)(cfg->input_volume & WM8731_LINVOL_MASK))
                         | WM8731_LRINBOTH);
  if (ret != HAL_OK) { return ret; }

  /* 7b. Headphone volume (RLHPBOTH + zero-cross) */
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_LHPOUT,
                         ((uint16_t)(cfg->output_volume & WM8731_LHPVOL_MASK))
                         | WM8731_RLHPBOTH | WM8731_LZCEN);
  if (ret != HAL_OK) { return ret; }

  /* 8. Activate */
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_ACTIVE, 0x0001U);
  if (ret != HAL_OK) { return ret; }

  HAL_Delay(5U);

  /* USER CODE BEGIN WM8731_Init_1 */
  /* USER CODE END WM8731_Init_1 */
  return HAL_OK;
}

/**
  * @brief  Thay đổi âm lượng HP output (runtime).
  * @param  hi2c      I2C handle
  * @param  addr      Địa chỉ 8-bit
  * @param  left_vol  0..127  (0x79 ≈ 0dB)
  * @param  right_vol Không dùng (RLHPBOTH tự cập nhật cả hai)
  */
HAL_StatusTypeDef WM8731_SetVolume(I2C_HandleTypeDef *hi2c, uint8_t addr,
                                    uint8_t left_vol, uint8_t right_vol)
{
  /* USER CODE BEGIN WM8731_SetVolume_0 */
  (void)right_vol;
  /* USER CODE END WM8731_SetVolume_0 */
  return WM8731_WriteReg(hi2c, addr, WM8731_REG_LHPOUT,
                          ((uint16_t)(left_vol & WM8731_LHPVOL_MASK))
                          | WM8731_RLHPBOTH | WM8731_LZCEN);
}

/**
  * @brief  DAC soft mute bật / tắt.
  * @param  hi2c  I2C handle
  * @param  addr  Địa chỉ 8-bit
  * @param  mute  true = mute
  */
HAL_StatusTypeDef WM8731_SetMute(I2C_HandleTypeDef *hi2c, uint8_t addr, bool mute)
{
  /* USER CODE BEGIN WM8731_SetMute_0 */
  uint16_t dp = WM8731_DEEMP_NONE;
  if (mute) { dp |= WM8731_DACMU; }
  /* USER CODE END WM8731_SetMute_0 */
  return WM8731_WriteReg(hi2c, addr, WM8731_REG_DIGITAL_PATH, dp);
}

/**
  * @brief  Cài LINE IN gain (runtime).
  * @param  hi2c  I2C handle
  * @param  addr  Địa chỉ 8-bit
  * @param  gain  0..31
  */
HAL_StatusTypeDef WM8731_SetInputGain(I2C_HandleTypeDef *hi2c,
                                       uint8_t addr, uint8_t gain)
{
  /* USER CODE BEGIN WM8731_SetInputGain_0 */
  /* USER CODE END WM8731_SetInputGain_0 */
  return WM8731_WriteReg(hi2c, addr, WM8731_REG_LLINE_IN,
                          ((uint16_t)(gain & WM8731_LINVOL_MASK)) | WM8731_LRINBOTH);
}

/**
  * @brief  Power-off toàn bộ codec.
  * @param  hi2c  I2C handle
  * @param  addr  Địa chỉ 8-bit
  */
HAL_StatusTypeDef WM8731_PowerDown(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
  /* USER CODE BEGIN WM8731_PowerDown_0 */
  HAL_StatusTypeDef ret;
  ret = WM8731_WriteReg(hi2c, addr, WM8731_REG_ACTIVE, 0x0000U);
  if (ret != HAL_OK) { return ret; }
  HAL_Delay(1U);
  /* USER CODE END WM8731_PowerDown_0 */
  return WM8731_WriteReg(hi2c, addr, WM8731_REG_POWER_DOWN, WM8731_POWEROFF);
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
