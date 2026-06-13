/**
  ******************************************************************************
  * @file    lpc_quant.h
  * @brief   LPC frame scalar quantizer / dequantizer (64 bits per frame)
  *
  *  Converts LPC_Frame_t to/from an 8-byte bitstream for over-air transmission
  *  in the FreeDV Phase 3 OFDM modem.
  *
  *  Bit layout (MSB-first, 64 bits total):
  *
  *    bits  0-11  SYNC word 0xA55 (12 bits, frame boundary marker)
  *    bits 12-15  gain  (4 bits, log2 scale, range [-10, 0])
  *    bit  16     voiced flag (1 = voiced)
  *    bits 17-23  pitch (7 bits; 0 when unvoiced; 1..123 → T0 = 20..142 smp)
  *    bits 24-27  a[1]  (4 bits, uniform range ±2.0)
  *    bits 28-31  a[2]
  *    bits 32-35  a[3]
  *    bits 36-39  a[4]
  *    bits 40-43  a[5]
  *    bits 44-47  a[6]
  *    bits 48-51  a[7]
  *    bits 52-55  a[8]
  *    bits 56-59  a[9]
  *    bits 60-63  a[10]
  *
  *  Rate: 64 bits / 40 ms (LPC_FRAME_SAMPS = 320 @ 8 kHz) = 1600 bps.
  ******************************************************************************
  */

#ifndef __LPC_QUANT_H
#define __LPC_QUANT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lpc_voc.h"

#define LPC_Q_FRAME_BYTES  8U        /*!< Quantized frame size in bytes     */
#define LPC_Q_SYNC         0xA55U    /*!< 12-bit sync pattern               */

/**
 * @brief  Quantize LPC_Frame_t to 64-bit bitstream.
 * @param  f     Input LPC parameters (a[0..10], gain, pitch_samps)
 * @param  bits  Output: 8 bytes, bit layout as above
 */
void LPC_Quantize(const LPC_Frame_t *f, uint8_t bits[LPC_Q_FRAME_BYTES]);

/**
 * @brief  Reconstruct LPC_Frame_t from 64-bit bitstream.
 * @param  bits  Input: 8 bytes previously produced by LPC_Quantize
 * @param  f     Output: LPC parameters (a[0]=1, a[1..10], gain, pitch_samps)
 */
void LPC_Dequantize(const uint8_t bits[LPC_Q_FRAME_BYTES], LPC_Frame_t *f);

#ifdef __cplusplus
}
#endif
#endif /* __LPC_QUANT_H */
