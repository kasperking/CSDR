/* USER CODE BEGIN Header */
/**
  * @file  selftest.h
  * @brief Boot-time hardware selftest — presence check only.
  *
  *  Runs once at the end of CSDR_Init after all peripheral inits complete.
  *  Results are collected from init return values already captured by CSDR_Init;
  *  no additional I/O is performed here.
  *
  *  Boot always continues regardless of result — no peripheral is disabled,
  *  no init is retried.  Failed items are shown in the header top bar
  *  (SDR_UI_DrawHeader reads g_selftest directly).
  *
  *  Items tested:
  *    FLASH — W25Q128 SPI flash   (settings persistence)
  *    CODEC — WM8731 audio codec  (RX/TX audio)
  *    PLL   — SI5351 oscillator   (VFO / LO synthesis)
  *    INA   — INA226 PA current   (overcurrent protection)
  *    SAI   — SAI DMA audio       (I2S hardware interface)
  */
/* USER CODE END Header */

#ifndef __SELFTEST_H
#define __SELFTEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SELFTEST_COUNT  5U

typedef struct {
    const char *id;   /*!< Short label: "FLASH", "CODEC", "PLL", "INA", "SAI" */
    bool        ok;   /*!< true = detected and initialised correctly            */
} SelfTest_Entry_t;

typedef struct {
    SelfTest_Entry_t items[SELFTEST_COUNT];
    uint8_t          fail_count;
    bool             complete;   /*!< Set true after SelfTest_Run() */
} SelfTest_Result_t;

extern SelfTest_Result_t g_selftest;

/**
  * @brief  Record hardware presence results collected during CSDR_Init.
  *         Called once at end of boot. Never blocks or retries anything.
  *         Boot continues normally regardless of result.
  */
void SelfTest_Run(bool flash_ok, bool codec_ok, bool pll_ok,
                  bool ina_ok,   bool sai_ok);

/** @brief  true if at least one hardware item failed selftest. */
bool SelfTest_AnyFail(void);

#ifdef __cplusplus
}
#endif
#endif /* __SELFTEST_H */
