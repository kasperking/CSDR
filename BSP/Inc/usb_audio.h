/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_audio.h
  * @brief   USB Audio Class 1.0 – IQ Streaming CSDR ↔ PC
  *
  *  Cấu hình: UAC 1.0, 48kHz, Stereo (L=I, R=Q), 16-bit
  *
  *  RX path (SDR → PC):
  *   SAI1_B ADC → s_iq_rx_buf (DMA) → USB Audio ISO IN (EP3)
  *   PC nhận IQ samples để xử lý bằng phần mềm (HDSDR, SDR#, etc.)
  *
  *  TX path (PC → SDR):
  *   USB Audio ISO OUT (EP3) → s_iq_tx_buf → SAI1_A DAC (TX audio)
  *   PC phát audio qua radio (SSB, AM, CW từ phần mềm)
  *
  *  Ring buffer: 4 × 192-byte packets (4ms latency)
  *   Packet = 48 samples/ms × 2ch × 2B = 192 byte/ms
  *
  *  IQ sample format trong USB packet (little-endian):
  *   offset 0: I sample int16_t (left channel)
  *   offset 2: Q sample int16_t (right channel)
  *   ... × 48 samples = 192 bytes per 1ms USB frame
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

/* Audio format (fixed) */
#define USB_AUDIO_SAMPLE_RATE    48000U
#define USB_AUDIO_CHANNELS       2U
#define USB_AUDIO_BITS           16U
#define USB_AUDIO_BYTES_PER_SAMPLE 2U
#define USB_AUDIO_BYTES_PER_FRAME  \
  (USB_AUDIO_SAMPLE_RATE / 1000U * USB_AUDIO_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE)
  /* = 48 × 2 × 2 = 192 byte/ms */

/* Ring buffer: 32 packets = 32ms (cần đủ lớn để chứa >= 2 SAI DMA block).
 * SAI block = 256 samples × 2ch × 2B = 1024B = ~5.3 packets/block
 * Min safe = 4×block ≈ 22 packets → round up to 32 for headroom. */
#define USB_AUDIO_RING_PACKETS   32U
#define USB_AUDIO_RING_SIZE      (USB_AUDIO_RING_PACKETS * USB_AUDIO_BYTES_PER_FRAME)
  /* = 32 × 192 = 6144 byte */

/* Thresholds */
#define USB_AUDIO_LATENCY_TARGET_MS  4U   /* Mục tiêu: 4ms latency */
#define USB_AUDIO_UNDERRUN_THR       2U   /* < 2 packets → underrun */
#define USB_AUDIO_OVERRUN_THR        7U   /* > 7 packets → overrun  */

/* Exported types ------------------------------------------------------------*/

typedef struct {
  /* RX ring buffer (SAI → USB) */
  uint8_t  rx_ring[USB_AUDIO_RING_SIZE]
      __attribute__((aligned(32)));
  uint16_t rx_wr;        /* Write pointer (SAI DMA writes here) */
  uint16_t rx_rd;        /* Read pointer  (USB reads here)      */
  uint16_t rx_count;     /* Bytes available                     */

  /* TX ring buffer (USB → SAI) */
  uint8_t  tx_ring[USB_AUDIO_RING_SIZE]
      __attribute__((aligned(32)));
  uint16_t tx_wr;
  uint16_t tx_rd;
  uint16_t tx_count;

  /* Stats */
  uint32_t rx_overrun;
  uint32_t rx_underrun;
  uint32_t tx_overrun;
  uint32_t tx_underrun;
  uint32_t usb_rx_frames;   /* Frames received from USB */
  uint32_t usb_tx_frames;   /* Frames sent to USB       */

  bool     usb_streaming;   /* True khi USB host đang stream */
} USB_Audio_Handle_t;

/* Exported variables --------------------------------------------------------*/
extern USB_Audio_Handle_t g_usb_audio;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Khởi tạo USB Audio ring buffers.
  */
void USB_Audio_Init(USB_Audio_Handle_t *au);

/**
  * @brief  Ghi IQ samples từ SAI DMA vào RX ring (→ USB IN).
  *         Gọi từ HAL_SAI_RxHalfCpltCallback / HAL_SAI_RxCpltCallback.
  * @param  au        Handle
  * @param  src       SAI DMA buffer (int32_t, 16-bit left-aligned)
  * @param  samples   Số sample pairs (I+Q)
  */
void USB_Audio_WriteRX(USB_Audio_Handle_t *au,
                        const int32_t *src, uint16_t samples);

/**
  * @brief  Đọc 1 packet (192 byte) từ RX ring để gửi qua USB ISO IN.
  *         Gọi từ USBD_AUDIO_DataIn callback mỗi 1ms.
  * @param  au     Handle
  * @param  dst    Output buffer (192 byte)
  * @retval Số byte thực sự copy (0 nếu underrun)
  */
uint16_t USB_Audio_ReadRXPacket(USB_Audio_Handle_t *au, uint8_t *dst);

/**
  * @brief  Ghi 1 packet nhận từ USB ISO OUT vào TX ring (→ SAI DAC).
  *         Gọi từ USBD_AUDIO_DataOut callback mỗi 1ms.
  * @param  au     Handle
  * @param  src    Packet data (192 byte)
  * @param  len    Thực tế là USB_AUDIO_BYTES_PER_FRAME
  */
void USB_Audio_WriteTX(USB_Audio_Handle_t *au,
                        const uint8_t *src, uint16_t len);

/**
  * @brief  Đọc samples từ TX ring để đẩy vào SAI DMA buffer.
  *         Gọi từ HAL_SAI_TxHalfCpltCallback / HAL_SAI_TxCpltCallback.
  * @param  au        Handle
  * @param  dst       SAI DMA buffer (int32_t)
  * @param  samples   Số sample pairs yêu cầu
  */
void USB_Audio_ReadTX(USB_Audio_Handle_t *au,
                       int32_t *dst, uint16_t samples);

/**
  * @brief  Gọi từ main loop: monitor streaming state.
  */
void USB_Audio_Process(USB_Audio_Handle_t *au);

/**
  * @brief  Bật/tắt USB Audio streaming.
  */
void USB_Audio_SetStreaming(USB_Audio_Handle_t *au, bool enable);

#ifdef __cplusplus
}
#endif
#endif /* __USB_AUDIO_H */
