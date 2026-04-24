/* USER CODE BEGIN Header */
/**
  * @file usb_audio.c
  * @brief USB Audio Class 1.0 IQ Streaming – Ring Buffer Engine
  */
/* USER CODE END Header */

#include "usb_audio.h"
#include <string.h>

/* USER CODE BEGIN PV */
USB_Audio_Handle_t g_usb_audio;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

void USB_Audio_Init(USB_Audio_Handle_t *au)
{
  /* USER CODE BEGIN USB_Audio_Init_0 */
  memset(au, 0, sizeof(*au));
  /* USER CODE END USB_Audio_Init_0 */
}

void USB_Audio_SetStreaming(USB_Audio_Handle_t *au, bool enable)
{
  /* USER CODE BEGIN USB_Audio_SetStreaming_0 */
  au->usb_streaming = enable;
  if (!enable) {
    /* Clear rings on stop */
    au->rx_wr = au->rx_rd = au->rx_count = 0U;
    au->tx_wr = au->tx_rd = au->tx_count = 0U;
  }
  /* USER CODE END USB_Audio_SetStreaming_0 */
}

/**
  * @brief  SAI DMA callback → ring buffer.
  *
  *  SAI format: int32_t left-aligned 16-bit (bits [31:16]).
  *  USB format: int16_t little-endian, L=I, R=Q interleaved.
  *
  *  Pipeline:
  *   SAI_RX: [I_hi16 I_lo16][Q_hi16 Q_lo16] → USB: [I16][Q16] × 48
  */
void USB_Audio_WriteRX(USB_Audio_Handle_t *au,
                        const int32_t *src, uint16_t samples)
{
  /* USER CODE BEGIN USB_Audio_WriteRX_0 */
  if (!au->usb_streaming) return;

  uint16_t bytes = (uint16_t)(samples * USB_AUDIO_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE);

  if (au->rx_count + bytes > USB_AUDIO_RING_SIZE) {
    au->rx_overrun++;
    return;
  }

  /* Pack SAI int32 → USB int16 little-endian */
  for (uint16_t i = 0U; i < samples; i++)
  {
    /* Left = I channel, Right = Q channel.
     * SAI format: int32_t với data 16-bit LSB-aligned (STM32H7 SAI
     * với SlotSize=32, DataSize=16 right-aligns data in slot). */
    int16_t i_samp = (int16_t)src[i * 2U];
    int16_t q_samp = (int16_t)src[i * 2U + 1U];

    /* Write I */
    uint16_t pos = au->rx_wr;
    au->rx_ring[pos]   = (uint8_t)( i_samp        & 0xFFU);
    au->rx_ring[(pos+1U) % USB_AUDIO_RING_SIZE] = (uint8_t)((i_samp >> 8U) & 0xFFU);
    au->rx_wr = (uint16_t)((pos + 2U) % USB_AUDIO_RING_SIZE);

    /* Write Q */
    pos = au->rx_wr;
    au->rx_ring[pos]   = (uint8_t)( q_samp        & 0xFFU);
    au->rx_ring[(pos+1U) % USB_AUDIO_RING_SIZE] = (uint8_t)((q_samp >> 8U) & 0xFFU);
    au->rx_wr = (uint16_t)((pos + 2U) % USB_AUDIO_RING_SIZE);
  }
  au->rx_count = (uint16_t)(au->rx_count + bytes);
  /* USER CODE END USB_Audio_WriteRX_0 */
}

/**
  * @brief  Đọc 1 USB packet (192 byte) từ RX ring.
  *         Nếu không đủ dữ liệu → zero-fill (underrun).
  */
uint16_t USB_Audio_ReadRXPacket(USB_Audio_Handle_t *au, uint8_t *dst)
{
  /* USER CODE BEGIN USB_Audio_ReadRXPacket_0 */
  const uint16_t pkt = USB_AUDIO_BYTES_PER_FRAME;

  if (au->rx_count < pkt) {
    au->rx_underrun++;
    memset(dst, 0, pkt);
    return pkt;
  }

  /* Linear copy with wrap-around */
  for (uint16_t i = 0U; i < pkt; i++) {
    dst[i] = au->rx_ring[au->rx_rd];
    au->rx_rd = (uint16_t)((au->rx_rd + 1U) % USB_AUDIO_RING_SIZE);
  }
  au->rx_count = (uint16_t)(au->rx_count - pkt);
  au->usb_tx_frames++;
  return pkt;
  /* USER CODE END USB_Audio_ReadRXPacket_0 */
}

/**
  * @brief  USB OUT packet → TX ring (PC → SAI DAC).
  */
void USB_Audio_WriteTX(USB_Audio_Handle_t *au,
                        const uint8_t *src, uint16_t len)
{
  /* USER CODE BEGIN USB_Audio_WriteTX_0 */
  if (!au->usb_streaming) return;

  if (au->tx_count + len > USB_AUDIO_RING_SIZE) {
    au->tx_overrun++;
    return;
  }
  for (uint16_t i = 0U; i < len; i++) {
    au->tx_ring[au->tx_wr] = src[i];
    au->tx_wr = (uint16_t)((au->tx_wr + 1U) % USB_AUDIO_RING_SIZE);
  }
  au->tx_count = (uint16_t)(au->tx_count + len);
  au->usb_rx_frames++;
  /* USER CODE END USB_Audio_WriteTX_0 */
}

/**
  * @brief  Đọc samples từ TX ring → SAI DMA buffer.
  *         USB int16 → SAI int32 left-aligned.
  *         Nếu ring trống → zero-fill (silence).
  */
void USB_Audio_ReadTX(USB_Audio_Handle_t *au,
                       int32_t *dst, uint16_t samples)
{
  /* USER CODE BEGIN USB_Audio_ReadTX_0 */
  uint16_t bytes = (uint16_t)(samples * USB_AUDIO_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE);

  if (au->tx_count < bytes) {
    au->tx_underrun++;
    memset(dst, 0, (size_t)samples * USB_AUDIO_CHANNELS * sizeof(int32_t));
    return;
  }

  for (uint16_t i = 0U; i < samples; i++)
  {
    /* Read I (left) */
    uint8_t lo = au->tx_ring[au->tx_rd];
    au->tx_rd = (uint16_t)((au->tx_rd + 1U) % USB_AUDIO_RING_SIZE);
    uint8_t hi = au->tx_ring[au->tx_rd];
    au->tx_rd = (uint16_t)((au->tx_rd + 1U) % USB_AUDIO_RING_SIZE);
    int16_t i_samp = (int16_t)((uint16_t)hi << 8U | lo);

    /* Read Q (right) */
    lo = au->tx_ring[au->tx_rd];
    au->tx_rd = (uint16_t)((au->tx_rd + 1U) % USB_AUDIO_RING_SIZE);
    hi = au->tx_ring[au->tx_rd];
    au->tx_rd = (uint16_t)((au->tx_rd + 1U) % USB_AUDIO_RING_SIZE);
    int16_t q_samp = (int16_t)((uint16_t)hi << 8U | lo);

    /* SAI: int32 with 16-bit data LSB-aligned (match SAI H7 format) */
    dst[i * 2U]       = (int32_t)((uint32_t)(uint16_t)i_samp & 0x0000FFFFU);
    dst[i * 2U + 1U]  = (int32_t)((uint32_t)(uint16_t)q_samp & 0x0000FFFFU);
  }
  au->tx_count = (uint16_t)(au->tx_count - bytes);
  /* USER CODE END USB_Audio_ReadTX_0 */
}

void USB_Audio_Process(USB_Audio_Handle_t *au)
{
  /* USER CODE BEGIN USB_Audio_Process_0 */
  /* Adaptive buffer management: nếu RX ring đầy >75% → drop oldest packet */
  if (au->rx_count > (USB_AUDIO_RING_SIZE * 3U / 4U)) {
    au->rx_rd = (uint16_t)((au->rx_rd + USB_AUDIO_BYTES_PER_FRAME)
                              % USB_AUDIO_RING_SIZE);
    if (au->rx_count >= USB_AUDIO_BYTES_PER_FRAME)
      au->rx_count = (uint16_t)(au->rx_count - USB_AUDIO_BYTES_PER_FRAME);
    au->rx_overrun++;
  }
  /* USER CODE END USB_Audio_Process_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
