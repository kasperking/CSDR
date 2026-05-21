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

/* Ring buffer: 48 packets = 48 ms latency capacity.
 * SAI block = 256 samples × 2ch × 2B = 1024B = ~5.3 packets/block.
 * RAM cost: 2 rings × 48 × 192 = 18432 B in DMA_SRAM. */
#define USB_AUDIO_RING_PACKETS   48U
#define USB_AUDIO_RING_SIZE      (USB_AUDIO_RING_PACKETS * USB_AUDIO_BYTES_PER_FRAME)
  /* = 48 × 192 = 9216 byte */

/* Overflow discard threshold (bytes): rx_count above this → drop one packet per SOF.
 *
 * Formula: (ring_packets − 8) × frame_size.  The 8-packet margin keeps the
 * threshold 1536 B below the WriteRX overrun guard (fires at ring_size − 1024 B),
 * so the discard path always fires before WriteRX silently drops a block.
 *
 * 75 % (= 6912 B, 36 pkts) created a stable limit-cycle with the 48-packet ring:
 * WriteRX adds one 1024-byte block every ~5.3 ms.  If rx_count was in the
 * [5888, 6912]-byte oscillation band, that block pushed rx_count back above the
 * threshold, making overflow fire on every SOF frame indefinitely.
 *
 * 83 % (= 7680 B, 40 pkts) raises the oscillation-zone floor to 6656 B ≈ 35 pkts,
 * unreachable from the natural operating fill of ~4–10 packets. */
#define USB_AUDIO_OVERRUN_BYTES \
  ((USB_AUDIO_RING_PACKETS - 8U) * USB_AUDIO_BYTES_PER_FRAME)
  /* 48 pkts → (48-8)×192 = 7680 B (83 %);  32 pkts → (32-8)×192 = 4608 B (75 %) */

#define USB_AUDIO_LATENCY_TARGET_MS  4U   /* target latency                          */
#define USB_AUDIO_UNDERRUN_THR       2U   /* < 2 packets → underrun (documentation)  */
#define USB_AUDIO_OVERRUN_THR       40U   /* threshold in packets; see OVERRUN_BYTES  */

/* Exported types ------------------------------------------------------------*/

typedef struct {
  /* RX ring buffer (SAI → USB) */
  uint8_t  rx_ring[USB_AUDIO_RING_SIZE]
      __attribute__((aligned(32)));
  volatile uint16_t rx_wr;      /* Write pointer – written only by main loop   */
  volatile uint16_t rx_rd;      /* Read pointer  – written only by USB IRQ     */
  volatile uint16_t rx_count;   /* Bytes available – written by both contexts  */

  /* TX ring buffer (USB → SAI) */
  uint8_t  tx_ring[USB_AUDIO_RING_SIZE]
      __attribute__((aligned(32)));
  volatile uint16_t tx_wr;      /* Write pointer – written only by USB IRQ     */
  volatile uint16_t tx_rd;      /* Read pointer  – written only by main loop   */
  volatile uint16_t tx_count;   /* Bytes available – written by both contexts  */

  /* Stats */
  uint32_t rx_overrun;       /* Total RX overflow events (both soft+hard paths)          */
  uint32_t rx_overrun_write; /* WriteRX hard-path drops only (ring + block > ring_size)  */
  uint32_t rx_underrun;      /* Total RX underrun events  (ring empty at USB ISO IN time)*/
  uint32_t tx_overrun;       /* Total TX overflow events  (USB OUT arrived, ring full)   */
  uint32_t tx_underrun;      /* Total TX underrun events  (SAI DMA fired, ring empty)    */
  uint32_t dropped_packets;  /* ReadRXPacket soft-path drops (ring > 83% at SOF time)    */
  uint32_t usb_rx_frames;    /* Frames received from USB host (USB OUT, PC→SAI)          */
  uint32_t usb_tx_frames;    /* Frames sent to USB host      (USB IN,  SAI→PC)           */
  uint16_t rx_count_peak;    /* Peak rx_count seen since session start (ST-Link watch)   */

  /* rx_overrun_pending: set by USB IRQ (ReadRXPacket) on every overflow event.
   * Cleared by main loop after detection.  Allows CSDR_Loop to react within
   * one waterfall tick (75 ms) rather than waiting for the 1-second rate window. */
  volatile bool rx_overrun_pending;

  volatile bool usb_streaming;   /* True khi USB host đang stream */
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
  *         Phải gọi từ main-loop context (CSDR_Loop), KHÔNG gọi từ DMA ISR.
  *         Gọi từ ISR sẽ chặn USB OTG IRQ và gây hard-lock khi host mở stream.
  * @param  au        Handle
  * @param  src       SAI DMA buffer (int32_t, 16-bit right-justified)
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
