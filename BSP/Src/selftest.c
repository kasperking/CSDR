/* USER CODE BEGIN Header */
/**
  * @file  selftest.c
  * @brief Boot-time hardware selftest implementation.
  *        See selftest.h for architecture and integration notes.
  */
/* USER CODE END Header */

#include "selftest.h"

SelfTest_Result_t g_selftest = {0};

static const char *const k_ids[SELFTEST_COUNT] = {
    "FLASH", "CODEC", "PLL", "INA", "SAI", "KEYS"
};

void SelfTest_Run(bool flash_ok, bool codec_ok, bool pll_ok,
                  bool ina_ok,   bool sai_ok,   bool keys_ok)
{
    const bool ok[SELFTEST_COUNT] = { flash_ok, codec_ok, pll_ok, ina_ok, sai_ok, keys_ok };

    g_selftest.fail_count = 0U;
    for (uint8_t i = 0U; i < SELFTEST_COUNT; i++) {
        g_selftest.items[i].id = k_ids[i];
        g_selftest.items[i].ok = ok[i];
        if (!ok[i]) g_selftest.fail_count++;
    }
    g_selftest.complete = true;
}

bool SelfTest_AnyFail(void)
{
    return g_selftest.complete && (g_selftest.fail_count > 0U);
}
