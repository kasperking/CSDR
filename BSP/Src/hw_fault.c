/* USER CODE BEGIN Header */
/**
  * @file  hw_fault.c
  * @brief Centralized hardware fault registry — global storage.
  */
/* USER CODE END Header */

#include "hw_fault.h"

volatile uint32_t g_hw_fault_mask = 0U;
