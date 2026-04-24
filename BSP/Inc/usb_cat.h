/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_cat.h
  * @brief   USB CAT Control – Kenwood TS-2000 Protocol over CDC VCP
  *
  *  Transport: USB CDC Virtual COM Port (EP1 IN/OUT, 64 byte packets)
  *  Protocol: Kenwood TS-2000 CAT commands (ASCII, semicolon terminated)
  *
  *  Lệnh hỗ trợ:
  *   FA;          → Đọc tần số VFO A       FA07100000;
  *   FA<10d>;     → Đặt tần số VFO A       FA00007100000;
  *   MD;          → Đọc mode              MD2;
  *   MD<1d>;      → Đặt mode              MD2; (1=LSB,2=USB,3=CW,4=FM,5=AM)
  *   TX;          → Đặt TX                TX1; (0=RX, 1=TX)
  *   RX;          → Đặt RX
  *   IF;          → Info (frequency+mode+etc)
  *   AI<1d>;      → Auto Info on/off
  *   SM<1d>;      → Đọc S-meter          SM00012;
  *   RA<02d>;     → RX attenuator        RA00; (0=off, 1=6dB, 2=12dB, 3=18dB)
  *   ID;          → Device ID             ID019; (TS-2000)
  *   PS;          → Power status          PS1;
  *   PC<3d>;      → TX power control (not impl – ACK only)
  *   ?;           → Error response        ?;
  *
  *  Auto Info (AI1):
  *   Khi tần số/mode thay đổi, device tự gửi IF; response.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USB_CAT_H
#define __USB_CAT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define CAT_BUF_SIZE      128U   /* Max command length */
#define CAT_TX_BUF_SIZE   128U   /* Response buffer    */
#define CAT_FIFO_SIZE      16U   /* Pending responses  */

/* Kenwood mode codes */
#define CAT_MODE_LSB    1U
#define CAT_MODE_USB    2U
#define CAT_MODE_CW     3U
#define CAT_MODE_FM     4U
#define CAT_MODE_AM     5U
#define CAT_MODE_FSK    6U
#define CAT_MODE_CWR    7U
#define CAT_MODE_FSKR   9U

/* Exported types ------------------------------------------------------------*/

/** Callback set: CAT driver calls these to apply commands to SDR state */
typedef struct {
  void (*set_freq)(uint32_t freq_hz);      /*!< Đặt tần số      */
  void (*set_mode)(uint8_t cat_mode);      /*!< Đặt mode        */
  void (*set_tx)(bool tx_on);              /*!< TX/RX switch    */
  void (*set_att)(uint8_t level_0_3);      /*!< Attenuator 0-3  */
  uint32_t (*get_freq)(void);              /*!< Đọc tần số      */
  uint8_t  (*get_mode)(void);              /*!< Đọc mode        */
  bool     (*get_tx)(void);               /*!< Đọc TX state    */
  float    (*get_signal_db)(void);         /*!< Đọc S-meter dBFS*/
  uint8_t  (*get_att)(void);              /*!< Đọc attenuation */
} CAT_Callbacks_t;

/** CAT driver state */
typedef struct {
  char     rx_buf[CAT_BUF_SIZE];   /*!< Accumulate incoming bytes */
  uint16_t rx_len;                  /*!< Current command length    */
  char     tx_buf[CAT_TX_BUF_SIZE]; /*!< Response to send          */
  bool     auto_info;               /*!< AI mode active            */
  uint32_t last_freq;               /*!< For change detection (AI) */
  uint8_t  last_mode;               /*!< For change detection (AI) */
  CAT_Callbacks_t cb;
  bool     initialized;
} CAT_Handle_t;

/* Exported variables --------------------------------------------------------*/
extern CAT_Handle_t g_cat;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Khởi tạo CAT driver với callbacks.
  */
void CAT_Init(CAT_Handle_t *cat, const CAT_Callbacks_t *cb);

/**
  * @brief  Nhận bytes từ USB CDC, parse và execute lệnh.
  *         Gọi từ USBD_CDC_Receive callback.
  * @param  cat   CAT handle
  * @param  data  Dữ liệu nhận từ USB
  * @param  len   Số byte
  */
void CAT_Receive(CAT_Handle_t *cat, const uint8_t *data, uint16_t len);

/**
  * @brief  Gọi trong main loop: kiểm tra thay đổi và gửi AI notifications.
  */
void CAT_Process(CAT_Handle_t *cat);
void CAT_FlushTX (CAT_Handle_t *cat);  /*!< Call from main loop to flush TX queue */

/**
  * @brief  Gửi chuỗi response qua USB CDC.
  *         Gọi nội bộ sau khi parse lệnh.
  */
void CAT_SendResponse(const char *resp);

/**
  * @brief  Tạo chuỗi IF (Information) response từ trạng thái SDR hiện tại.
  * @param  buf  Output buffer (≥100 byte)
  */
void CAT_BuildIF(CAT_Handle_t *cat, char *buf);

/**
  * @brief  Chuyển đổi mode SDR ↔ CAT mode code.
  */
uint8_t CAT_SDRModeToCat(uint8_t sdr_mode);
uint8_t CAT_CatModeToSDR(uint8_t cat_mode);

#ifdef __cplusplus
}
#endif
#endif /* __USB_CAT_H */
