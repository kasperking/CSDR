/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_cat.h
  * @brief   USB CAT Control – Kenwood TS-480 Protocol over CDC VCP
  *
  *  Transport: USB CDC Virtual COM Port (EP1 IN/OUT, 64 byte packets)
  *  Protocol: Kenwood TS-480 CAT commands (ASCII, semicolon terminated)
  *
  *  Lệnh hỗ trợ:
  *   FA/FB;        → Đọc/đặt tần số VFO A/B   FA00007100000;
  *   MD;           → Đọc/đặt mode             MD2; (1=LSB,2=USB,3=CW,4=FM,5=AM)
  *   TX/RX;        → Đặt TX/RX mode
  *   IF;           → Info frame (freq+mode+… 38 chars, TS-480 P15 shift byte)
  *   AI<0-2>;      → Auto Info: 0=off, 1=AI1, 2=AI2
  *   AG0<3d>;      → Audio gain 000-255        AG0127;
  *   NR<0-1>;      → Noise reduction on/off
  *   NB<0-1>;      → Noise blanker on/off
  *   SH<2d>;       → IF high-cut (00-11)       SH10; (≈3000 Hz)
  *   SL<2d>;       → IF low-cut  (00-11)       SL00;
  *   FW<4d>;       → Filter width Hz (legacy)  FW3000;
  *   GT0<0-2>;     → AGC: 0=fast,1=slow,2=off  GT00;
  *   SQ0<3d>;      → Squelch 000-255           SQ0000;
  *   SM0;          → Đọc S-meter              SM00012;
  *   RA<02d>;      → RX attenuator            RA00; (0=off,1=6dB,2=12dB,3=18dB)
  *   ID;           → Device ID                ID020; (TS-480)
  *   PS;           → Power status             PS1;
  *   PC<3d>;       → TX power (ACK only)
  *   ?;            → Error response
  *
  *  Auto Info (AI1/AI2):
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
  /* Setters */
  void (*set_freq)(uint32_t freq_hz);
  void (*set_mode)(uint8_t sdr_mode);
  void (*set_tx)(bool tx_on);
  void (*set_att)(uint8_t level_0_3);
  void (*set_volume)(uint8_t vol);         /*!< AG: 0-100       */
  void (*set_nr)(bool on);                 /*!< NR on/off       */
  void (*set_nb)(bool on);                 /*!< NB on/off       */
  void (*set_bw)(uint32_t hz);             /*!< FW: Hz          */
  void (*set_agc_fast)(bool fast);         /*!< GT: fast/slow   */
  void (*set_squelch)(uint8_t sq);         /*!< SQ: 0-255       */
  void (*set_rit_hz)(int32_t hz);          /*!< RIT offset Hz   */
  void (*set_step)(uint32_t hz);           /*!< Tuning step Hz  */
  void (*set_if_shift)(int32_t hz);        /*!< IS: IF shift Hz */
  /* Getters */
  uint32_t (*get_freq)(void);
  uint8_t  (*get_mode)(void);
  bool     (*get_tx)(void);
  float    (*get_signal_db)(void);
  uint8_t  (*get_att)(void);
  uint8_t  (*get_volume)(void);
  bool     (*get_nr)(void);
  bool     (*get_nb)(void);
  uint32_t (*get_bw)(void);
  bool     (*get_agc_fast)(void);
  uint8_t  (*get_squelch)(void);
  int32_t  (*get_rit_hz)(void);            /*!< RIT offset Hz   */
  uint32_t (*get_step)(void);              /*!< Tuning step Hz  */
  int32_t  (*get_if_shift)(void);          /*!< IS: IF shift Hz */
} CAT_Callbacks_t;

/** CAT driver state */
typedef struct {
  char     rx_buf[CAT_BUF_SIZE];
  uint16_t rx_len;
  char     tx_buf[CAT_TX_BUF_SIZE];
  uint8_t  ai_level;               /*!< 0=off, 1=AI1, 2=AI2      */
  uint32_t last_freq;
  uint8_t  last_mode;
  bool     last_tx;                /*!< TX state tracked for AI unsolicited IF */
  uint8_t  last_vfo;               /*!< active_vfo at last AI notification     */
  bool     last_split;             /*!< split_on at last AI notification       */
  bool     rit_on;                 /*!< RIT on/off                */
  int16_t  if_shift;               /*!< IS: IF shift Hz           */
  uint32_t vfo_b_freq;             /*!< VFO B stored frequency    */
  uint8_t  vfo_b_mode;             /*!< VFO B stored mode (CAT code) */
  uint32_t vfo_b_bw;               /*!< VFO B stored bandwidth Hz */
  uint8_t  active_vfo;             /*!< 0=VFO_A, 1=VFO_B (VS cmd) */
  bool     split_on;               /*!< Split: TX on VFO B        */
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
