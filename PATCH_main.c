/**
 * HƯỚNG DẪN: Paste các đoạn code dưới vào đúng vị trí trong main.c
 * (file do CubeMX generate – KHÔNG thay thế toàn bộ file)
 * ========================================================================
 */

/* ── 1. USER CODE BEGIN Includes ─────────────────────────────────────────
   Tìm dòng "USER CODE BEGIN Includes" trong main.c, paste vào giữa: */

/* USER CODE BEGIN Includes */
#include "csdr_app.h"
/* USER CODE END Includes */


/* ── 2. USER CODE BEGIN 2 ────────────────────────────────────────────────
   Tìm dòng "USER CODE BEGIN 2", paste vào giữa: */

/* USER CODE BEGIN 2 */
  CSDR_Init();
/* USER CODE END 2 */


/* ── 3. USER CODE BEGIN 3 ────────────────────────────────────────────────
   Tìm dòng "USER CODE BEGIN 3" bên trong while(1), paste vào giữa: */

  /* USER CODE BEGIN 3 */
    CSDR_Loop();
  /* USER CODE END 3 */


/**
 * HƯỚNG DẪN: Paste vào stm32h7xx_it.c
 * ========================================================================
 */

/* ── 4. SysTick_Handler trong stm32h7xx_it.c ────────────────────────────
   Tìm hàm SysTick_Handler, paste vào USER CODE BEGIN SysTick_IRQn 1: */

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  CSDR_SysTickCallback();
  /* USER CODE END SysTick_IRQn 1 */
}


/**
 * HƯỚNG DẪN: Paste vào USB_DEVICE/App/usbd_cdc_if.c
 * ========================================================================
 */

/* ── 5. CDC_Receive_FS trong usbd_cdc_if.c ──────────────────────────────
   Tìm hàm CDC_Receive_FS, paste vào USER CODE BEGIN 6: */

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  CSDR_CDC_Receive(Buf, *Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
