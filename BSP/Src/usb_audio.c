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
    /* Stop: clear ring state so stale data is not replayed on reconnect.
     * Keep error counters alive until the next session start so they remain
     * visible in the diagnostic snapshot after the host disconnects. */
    au->rx_wr = au->rx_rd = au->rx_count = 0U;
    au->tx_wr = au->tx_rd = au->tx_count = 0U;
  } else {
    /* Start: reset per-session stats so counters reflect the new session only.
     * Pre-fill RX ring with 8 packets of silence when the ring is empty.
     * Without this, production (192.06 B/ms) ≈ consumption (192 B/ms) causes
     * the ring to bottom out at ~128 B before each WriteRX refill, firing
     * rx_underrun at ~166 Hz.  Eight silent packets give a 512-B minimum —
     * above the 192-B underrun threshold — so the ring never underruns unless
     * the main loop stalls for > 8 ms.
     * Guard: only pre-fill when rx_count == 0 (ring was cleared by Stop or
     * Init).  If SetStreaming(true) is called while already streaming (e.g.
     * host re-opens the interface), the ring already has live data — skip. */
    if (au->rx_count == 0U) {
      const uint16_t prefill = (uint16_t)(8U * USB_AUDIO_BYTES_PER_FRAME);  /* 1536 B */
      memset(au->rx_ring, 0, prefill);
      au->rx_wr    = prefill;
      au->rx_rd    = 0U;
      au->rx_count = prefill;
    }
    au->rx_overrun        = 0U;
    au->rx_overrun_write  = 0U;
    au->rx_underrun       = 0U;
    au->tx_overrun        = 0U;
    au->tx_underrun       = 0U;
    au->dropped_packets   = 0U;
    au->usb_rx_frames     = 0U;
    au->usb_tx_frames     = 0U;
    au->rx_count_peak     = 0U;
    au->rx_overrun_pending = false;
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

  /* Critical section: read rx_count + overrun check must be atomic.
   * USB IRQ (ReadRXPacket) can decrement rx_count at any time.  A stale
   * read here is conservative — the IRQ can only create MORE free space,
   * so if we see "no overrun" with a slightly high rx_count, that is safe.
   * BASEPRI = 0x20: masks USB OTG (priority 2 → 0x20) while leaving
   * SAI/DMA (priority 0 → 0x00) unmasked. */
  __set_BASEPRI(0x20U);
  if (au->rx_count + bytes > USB_AUDIO_RING_SIZE) {
    au->rx_overrun++;
    au->rx_overrun_write++;   /* hard-path: ring was genuinely full at produce time */
    __set_BASEPRI(0U);
    return;
  }
  __set_BASEPRI(0U);

  /* rx_wr is exclusively written by this function (main-loop context only).
   * USB IRQ never touches rx_wr, so the write loop needs no critical section. */
  for (uint16_t i = 0U; i < samples; i++)
  {
    /* SAI RX: 16-bit sample right-justified in bits[15:0]. */
    int16_t i_samp = (int16_t)(uint16_t)src[i * 2U];
    int16_t q_samp = (int16_t)(uint16_t)src[i * 2U + 1U];

    /* Write I */
    uint16_t pos = au->rx_wr;
    au->rx_ring[pos]                             = (uint8_t)( i_samp        & 0xFFU);
    au->rx_ring[(pos + 1U) % USB_AUDIO_RING_SIZE] = (uint8_t)((i_samp >> 8U) & 0xFFU);
    au->rx_wr = (uint16_t)((pos + 2U) % USB_AUDIO_RING_SIZE);

    /* Write Q */
    pos = au->rx_wr;
    au->rx_ring[pos]                             = (uint8_t)( q_samp        & 0xFFU);
    au->rx_ring[(pos + 1U) % USB_AUDIO_RING_SIZE] = (uint8_t)((q_samp >> 8U) & 0xFFU);
    au->rx_wr = (uint16_t)((pos + 2U) % USB_AUDIO_RING_SIZE);
  }

  /* Critical section: commit byte count.
   * USB IRQ may have decremented rx_count during the write loop — re-read it
   * inside the CS so the increment correctly accounts for that decrement.
   * If streaming was stopped (SetStreaming fired from IRQ) while we were
   * writing, the ring was already reset; discard this update to avoid
   * inflating rx_count against a zero'd ring.
   * BASEPRI = 0x20: masks USB OTG (priority 2 → 0x20); SAI/DMA unmasked. */
  __set_BASEPRI(0x20U);
  if (au->usb_streaming) {
    au->rx_count = (uint16_t)(au->rx_count + bytes);
    if (au->rx_count > au->rx_count_peak) au->rx_count_peak = au->rx_count;
  }
  __set_BASEPRI(0U);
  /* USER CODE END USB_Audio_WriteRX_0 */
}

/**
  * @brief  Đọc 1 USB packet (192 byte) từ RX ring.
  *         Nếu không đủ dữ liệu → zero-fill (underrun).
  *         Called exclusively from USB IRQ context (Comp_DataIn, Comp_SOF).
  *         rx_rd is written ONLY here — no concurrent modification, no CS needed.
  */
uint16_t USB_Audio_ReadRXPacket(USB_Audio_Handle_t *au, uint8_t *dst)
{
  /* USER CODE BEGIN USB_Audio_ReadRXPacket_0 */
  const uint16_t pkt = USB_AUDIO_BYTES_PER_FRAME;

  /* Overflow management: runs here (USB IRQ) so rx_rd has a single owner.
   * If the ring has grown beyond USB_AUDIO_OVERRUN_BYTES, discard the oldest
   * packet to catch up with the producer (WriteRX in the main loop).
   * This fires every 1 ms SOF frame, giving tighter rate control than the
   * former 10 ms poll in USB_Audio_Process. */
  if (au->rx_count > USB_AUDIO_OVERRUN_BYTES) {
    au->rx_rd    = (uint16_t)((au->rx_rd + pkt) % USB_AUDIO_RING_SIZE);
    au->rx_count = (uint16_t)(au->rx_count - pkt);
    au->rx_overrun++;
    au->dropped_packets++;
    /* Signal the main loop immediately — do not wait for the 1-second
     * rate window in RuntimeDiag_ServiceSlow.  The volatile write is a
     * single store instruction on Cortex-M7; no critical section needed. */
    au->rx_overrun_pending = true;
  }

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

    /* SAI TX: 16-bit data right-justified in bits[15:0]. */
    dst[i * 2U]       = (int32_t)(int16_t)i_samp;
    dst[i * 2U + 1U]  = (int32_t)(int16_t)q_samp;
  }
  au->tx_count = (uint16_t)(au->tx_count - bytes);
  /* USER CODE END USB_Audio_ReadTX_0 */
}

void USB_Audio_Process(USB_Audio_Handle_t *au)
{
  /* USER CODE BEGIN USB_Audio_Process_0 */
  /* Overflow management has been moved into USB_Audio_ReadRXPacket (USB IRQ
   * context) so that rx_rd has a single owner and no critical section is
   * needed there.  This function intentionally does nothing now; the call
   * site in CSDR_Loop may be left in place without harm. */
  (void)au;
  /* USER CODE END USB_Audio_Process_0 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
