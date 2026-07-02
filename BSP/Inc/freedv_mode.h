/**
  ******************************************************************************
  * @file    freedv_mode.h
  * @brief   FreeDV narrowband digital voice mode
  *
  *  Architecture:
  *   TX: USB audio 48 kHz  →  6:1 polyphase downsample  →  8 kHz PCM
  *       →  LPC-10 encode (LPC_Encode)                          [Phase 2]
  *       →  quantize LPC params to 64 bits (LPC_Quantize)       [Phase 3]
  *       →  OFDM DQPSK modulate (FdvModem_EncodeSuperFrame)     [Phase 3]
  *       →  NBUSB SSB modulate at 8 kHz BW  →  48 kHz IQ for RF
  *
  *   RX: 48 kHz IQ  →  SSB demod  →  8 kHz audio
  *       →  1:6 polyphase upsample  →  48 kHz audio out
  *       (OFDM demodulate + LPC_Dequantize planned for Phase 4)
  *
  *  Phase 1: resampler + NBUSB SSB — PCM pass-through vocoder stub.
  *  Phase 2: LPC-10 vocoder encode/decode loopback on TX path.
  *  Phase 3: LPC → quantize → OFDM DQPSK TX; RX stub (USB SSB demod).
  *
  *  Sample rates used in this module:
  *    FDVR_FS_HIGH  48000   (system rate, SAI DMA, USB audio)
  *    FDVR_FS_LOW    8000   (narrowband speech / codec)
  *    FDVR_RATIO        6   (decimation / interpolation factor)
  ******************************************************************************
  */

#ifndef __FREEDV_MODE_H
#define __FREEDV_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lpc_voc.h"    /* LPC_Voc_t, LPC_Frame_t  */
#include "lpc_quant.h"  /* LPC_Q_FRAME_BYTES        */
#include "fdv_modem.h"  /* FdvModem_t, FdvModemRx_t */

/* ── Sample-rate constants ────────────────────────────────── */
#define FDVR_FS_HIGH    48000U
#define FDVR_FS_LOW      8000U
#define FDVR_RATIO          6U

/* ── Polyphase FIR parameters ─────────────────────────────
 *  Anti-alias LPF at 3900 Hz (< Nyquist of 4000 Hz at 8 kHz).
 *  Total taps = FDVR_PHASES × FDVR_TAPS_PER_PHASE = 6 × 12 = 72.
 *  Window: Kaiser β=6; stop-band ≈ −72 dB.
 * ──────────────────────────────────────────────────────── */
#define FDVR_PHASES         6U
#define FDVR_TAPS_PER_PHASE 12U
#define FDVR_TOTAL_TAPS     (FDVR_PHASES * FDVR_TAPS_PER_PHASE)  /* 72 */

typedef struct {
    float    buf[FDVR_TOTAL_TAPS];  /*!< Delay-line history */
    uint8_t  phase;                 /*!< Current polyphase index (0…FDVR_PHASES-1) */
} Resampler6_t;

/* ── FreeDV Hilbert state for 8 kHz NBUSB TX ─────────────
 *  63-tap (same odd-tap rule as RX Hilbert).
 * ──────────────────────────────────────────────────────── */
#define FDVR_HILBERT_TAPS   63U
#define FDVR_HILBERT_DELAY  ((FDVR_HILBERT_TAPS - 1U) / 2U)  /* 31 */

typedef struct {
    float coeff[FDVR_HILBERT_TAPS];
    float buf  [FDVR_HILBERT_TAPS];
    uint16_t idx;
} FdvHilbert_t;

/* ── IIR DC-block (8 kHz rate) ───────────────────────────── */
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} FdvIIR_t;

/* ── Main FreeDV TX/RX state ─────────────────────────────── */
typedef struct {
    /* 48→8 kHz decimator (TX path) */
    Resampler6_t  dec;

    /* 8→48 kHz interpolator (RX path) */
    Resampler6_t  interp;

    /* DC block at 8 kHz (TX, pre-LPC) */
    FdvIIR_t      dc8;

    /* NBUSB SSB Hilbert at 8 kHz (TX modulation) */
    FdvHilbert_t  hilbert8;

    /* I-channel matched delay for NBUSB SSB */
    float         i_delay[FDVR_HILBERT_TAPS];
    uint16_t      i_delay_idx;

    /* Audio gain scaling (0..1) */
    float         audio_gain;

    /* IQ sample-hold — written at 8 kHz rate, read at 48 kHz */
    float         rx_prev_re;
    float         rx_prev_im;

    /* ── Phase 2/3: LPC vocoder + OFDM modem ───────────────────
     *  TX pipeline (Phase 3):
     *    1. Decimated 8 kHz samples accumulate in pcm8_buf[].
     *    2. When pcm8_wr reaches LPC_FRAME_SAMPS (320): LPC_Encode → cur_frame.
     *    3. LPC_Quantize(cur_frame) → 64-bit bitstream (8 bytes).
     *    4. FdvModem_EncodeSuperFrame(bits) → 320-sample OFDM audio → ofdm_buf[].
     *    5. Pull from ofdm_buf[] → Hilbert NBUSB SSB → 48 kHz IQ hold.
     *  Memory:
     *    pcm8_buf (320 floats = 1280 B) + ofdm_buf (320 floats = 1280 B)
     *    + LPC_Voc_t (56 B) + LPC_Frame_t (52 B) + FdvModem_t (~200 B)
     *    ≈ 2.9 KB additional in s_fdv.
     * ──────────────────────────────────────────────────────── */
    LPC_Voc_t     voc;                          /*!< LPC vocoder state (encode) */
    LPC_Frame_t   cur_frame;                    /*!< LPC params from last frame */
    FdvModem_t    modem;                        /*!< OFDM DQPSK TX modem       */

    float         pcm8_buf[LPC_FRAME_SAMPS];    /*!< Raw 8 kHz input buffer    */
    uint16_t      pcm8_wr;                      /*!< Write index into pcm8_buf  */

    float         ofdm_buf[LPC_FRAME_SAMPS];    /*!< OFDM-modulated 8 kHz out  */
    uint16_t      ofdm_rd;                      /*!< Read index into ofdm_buf   */
    uint16_t      ofdm_wr;                      /*!< Write fence (0 or SAMPS)   */

    /* Debug / diagnostic counters */
    uint32_t      dec_out_count;   /*!< 8 kHz samples produced by decimator */
    uint32_t      tx_frames;       /*!< LPC/OFDM super-frames since mode entry */

    /* ── RX pipeline ────────────────────────────────────────────────────────
     *  48 kHz IQ → USB demod → 48→8 kHz decimate (rx_dec) → OFDM demod
     *  → LPC_Dequantize + LPC_Decode → rx_pcm8_buf → 8→48 kHz interpolate
     *  (interp) → rx_out_buf → audio out.
     * ───────────────────────────────────────────────────────────────────── */
    Resampler6_t  rx_dec;                       /*!< 48→8 kHz decimator (RX)      */
    FdvIIR_t      rx_dc8;                       /*!< DC block at 8 kHz (RX)       */
    FdvModemRx_t  rx_demod;                     /*!< OFDM DQPSK demodulator       */
    LPC_Voc_t     rx_voc;                       /*!< LPC synthesis vocoder        */
    float         rx_pcm8_buf[LPC_FRAME_SAMPS]; /*!< Decoded 8 kHz PCM output     */
    uint16_t      rx_pcm8_rd;                   /*!< Read index into rx_pcm8_buf  */
    uint16_t      rx_pcm8_wr;                   /*!< Valid samples in buffer      */
    float         rx_out_buf[FDVR_PHASES];      /*!< Polyphase-interpolated output */
    uint8_t       rx_out_idx;                   /*!< Read index into rx_out_buf   */
    uint32_t      rx_frames;                    /*!< Decoded LPC frames (RX)      */
} FreeDV_State_t;

/* ── API ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise FreeDV state (call once at mode switch).
 */
void FreeDV_Init(FreeDV_State_t *fdv, float audio_gain);

/**
 * @brief  TX: push one 48 kHz audio sample, return IQ pair.
 *
 *  Every 6th call (8 kHz tick):
 *    1. Polyphase decimate → 8 kHz PCM sample.
 *    2. DC block at 8 kHz.
 *    3. Accumulate in pcm8_buf.
 *    4. When 320 samples full: LPC_Encode → LPC_Quantize → FdvModem_Encode
 *       → ofdm_buf[] (320-sample OFDM audio at 8 kHz).
 *    5. Pull from ofdm_buf[] → Hilbert NBUSB SSB → update IQ hold.
 *
 *  Between 8 kHz ticks: IQ is held (sample-hold, no gap).
 *
 * @param  fdv      FreeDV state
 * @param  audio    Input sample at 48 kHz (normalised ±1)
 * @param  out_i    Output: I sample
 * @param  out_q    Output: Q sample
 */
void FreeDV_TX_Sample(FreeDV_State_t *fdv,
                      float  audio,
                      float *out_i,
                      float *out_q);

/**
 * @brief  RX: accept one IQ pair at 48 kHz, return demodulated audio.
 */
float FreeDV_RX_Sample(FreeDV_State_t *fdv, float rx_i, float rx_q);

#ifdef __cplusplus
}
#endif
#endif /* __FREEDV_MODE_H */
