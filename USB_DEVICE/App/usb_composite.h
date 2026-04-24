/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_composite.h
  * @brief   USB Composite Device Class: CDC VCP + UAC 1.0 Audio (Full-Duplex)
  *
  *  Interface layout:
  *    IAD 0  → Audio function (2 interfaces)
  *      IF 0  Audio Control (AC)
  *      IF 1  Audio Streaming OUT (PC → MCU TX), alt 0=zero-bw, alt 1=active
  *      IF 2  Audio Streaming IN  (MCU RX → PC), alt 0=zero-bw, alt 1=active
  *    IAD 1  → CDC function (2 interfaces)
  *      IF 3  CDC Control (ACM)
  *      IF 4  CDC Data
  *
  *  Endpoint map:
  *    EP0 IN/OUT  Control
  *    EP1 IN      CDC Data IN   (bulk 64B)
  *    EP1 OUT     CDC Data OUT  (bulk 64B)
  *    EP2 IN      CDC Notification (interrupt 8B)
  *    EP3 IN      Audio RX → PC (iso 192B/ms @ 48kHz s16 stereo)
  *    EP3 OUT     Audio PC → TX (iso 192B/ms @ 48kHz s16 stereo)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USB_COMPOSITE_H
#define __USB_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"
#include "usbd_cdc.h"

/* ───────────── Interface numbers ───────────── */
#define COMP_IF_AC          0x00U   /* Audio Control     */
#define COMP_IF_AS_OUT      0x01U   /* Audio Streaming OUT (PC→MCU TX) */
#define COMP_IF_AS_IN       0x02U   /* Audio Streaming IN  (MCU RX→PC) */
#define COMP_IF_CDC_CTRL    0x03U   /* CDC Control ACM   */
#define COMP_IF_CDC_DATA    0x04U   /* CDC Data          */
#define COMP_NUM_INTERFACES 5U

/* ───────────── Endpoint addresses ───────────── */
#define COMP_EP_CDC_IN      0x81U   /* bulk */
#define COMP_EP_CDC_OUT     0x01U   /* bulk */
#define COMP_EP_CDC_NOTIF   0x82U   /* interrupt */
#define COMP_EP_AUDIO_IN    0x83U   /* iso  MCU→PC */
#define COMP_EP_AUDIO_OUT   0x03U   /* iso  PC→MCU */

#define COMP_EP_CDC_SIZE        64U
#define COMP_EP_CDC_NOTIF_SIZE  8U
#define COMP_EP_AUDIO_SIZE      192U   /* 48 samples × 2ch × 2B @ 48kHz */

/* ───────────── Audio streaming state ───────────── */
#define COMP_AS_ALT_ZERO    0U
#define COMP_AS_ALT_ACTIVE  1U

/* ───────────── Config descriptor total length ─────────────
 *  AudioCtrl (IAD + IF + AC hdr + Input/Feature/Output × 2)
 *  AudioStream OUT (IF alt0 + IF alt1 + AS hdr + format + EP + ep-data-ep)
 *  AudioStream IN  (IF alt0 + IF alt1 + AS hdr + format + EP + ep-data-ep)
 *  CDC (IAD + IF ACM + hdr + call + acm + union + EP notif + IF Data + 2×EP)
 *  Config hdr: 9
 *  IAD Audio: 8
 *  IF AC:     9
 *  AC Hdr:   10  (wTotalLength + 2 IFs)
 *  Input T OUT:  12
 *  Feature OUT:   9  (2 channels = 3+2+ctl*2+7 = 10... giản: 9)
 *  Output T OUT: 9
 *  Input T IN:  12
 *  Feature IN:  9
 *  Output T IN: 9
 *  AS OUT: IF alt0 (9) + IF alt1 (9) + AS general (7) + AS format I (11) + EP std (9) + EP ISO spec (7) = 52
 *  AS IN:  same = 52
 *  IAD CDC:     8
 *  CDC ACM IF:  9
 *  CDC hdr:     5
 *  CDC call:    5
 *  CDC acm:     4
 *  CDC union:   5
 *  EP notif:    7
 *  CDC data IF: 9
 *  EP CDC IN:   7
 *  EP CDC OUT:  7
 *  = 9 + 8 + 9 + 10 + 12+9+9 + 12+9+9 + 52 + 52 + 8 + 9+5+5+4+5+7 + 9+7+7
 *  = 263 (ước tính; sẽ tính chính xác qua macro trong .c)
 */
#define COMP_CFG_DESC_SIZ   263U

/* ───────────── Exported class driver ───────────── */
extern USBD_ClassTypeDef USBD_Composite;

/* ───────────── API for app layer ───────────── */

/**
 * @brief  Transmit data over CDC IN endpoint.
 *         Thay thế cho USBD_CDC_TransmitPacket vì CDC class driver của ST
 *         hard-code EP address = 0x81/0x01, mình đã giữ nguyên nên OK.
 * @retval USBD_OK / USBD_BUSY
 */
uint8_t Composite_CDC_Transmit(USBD_HandleTypeDef *pdev,
                                uint8_t *buf, uint16_t len);

/**
 * @brief  Query if Audio IN streaming (MCU → PC) is active.
 */
uint8_t Composite_AudioIN_IsStreaming(void);

/**
 * @brief  Query if Audio OUT streaming (PC → MCU) is active.
 */
uint8_t Composite_AudioOUT_IsStreaming(void);

#ifdef __cplusplus
}
#endif
#endif /* __USB_COMPOSITE_H */
