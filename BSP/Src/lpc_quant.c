/**
  ******************************************************************************
  * @file    lpc_quant.c
  * @brief   LPC frame scalar quantizer / dequantizer
  *
  *  Byte layout (8 bytes = 64 bits, MSB first):
  *    bits[0]        = sync[11:4]     = 0xA5
  *    bits[1][7:4]   = sync[3:0]      = 0x5
  *    bits[1][3:0]   = gain_q         (4 bits)
  *    bits[2][7]     = voiced
  *    bits[2][6:0]   = pitch_q        (7 bits)
  *    bits[3]        = a1_q[3:0] a2_q[3:0]
  *    bits[4]        = a3_q[3:0] a4_q[3:0]
  *    bits[5]        = a5_q[3:0] a6_q[3:0]
  *    bits[6]        = a7_q[3:0] a8_q[3:0]
  *    bits[7]        = a9_q[3:0] a10_q[3:0]
  ******************************************************************************
  */

#include "lpc_quant.h"
#include <string.h>
#include <math.h>

void LPC_Quantize(const LPC_Frame_t *f, uint8_t bits[LPC_Q_FRAME_BYTES])
{
    memset(bits, 0, LPC_Q_FRAME_BYTES);

    /* Sync 0xA55: top 8 bits → bits[0]; bottom 4 bits → bits[1] upper nibble */
    bits[0] = 0xA5U;
    bits[1] = 0x50U;

    /* Gain: log2 scale, 4 bits, range [-10, 0] → [0, 15] */
    float lg = log2f(f->gain > 1e-10f ? f->gain : 1e-10f);
    if (lg < -10.0f) lg = -10.0f;
    if (lg >   0.0f) lg =   0.0f;
    uint8_t qg = (uint8_t)((lg + 10.0f) * 15.0f / 10.0f + 0.5f);
    if (qg > 15U) qg = 15U;
    bits[1] |= qg;                  /* lower nibble of bits[1] */

    /* Voiced flag + pitch: bits[2] */
    uint8_t voiced = (f->pitch_samps > 0U) ? 1U : 0U;
    uint8_t qp     = 0U;
    if (voiced) {
        /* T0 range 20..142; store as 1..123 so 0 means unvoiced */
        int p = (int)f->pitch_samps - 19;
        if (p < 1)   p = 1;
        if (p > 123) p = 123;
        qp = (uint8_t)p;
    }
    bits[2] = (uint8_t)((voiced << 7) | (qp & 0x7FU));

    /* LPC coefficients a[1..10]: 4 bits each, uniform range [-2.0, +2.0] */
    for (int k = 1; k <= 10; k++) {
        float a = f->a[k];
        if (a < -2.0f) a = -2.0f;
        if (a >  2.0f) a =  2.0f;
        uint8_t qa = (uint8_t)((a + 2.0f) * 15.0f / 4.0f + 0.5f);
        if (qa > 15U) qa = 15U;
        int bi = 3 + (k - 1) / 2;          /* bytes 3..7 */
        if ((k - 1) % 2 == 0)
            bits[bi] |= (uint8_t)(qa << 4); /* upper nibble (odd k index) */
        else
            bits[bi] |= qa;                  /* lower nibble */
    }
}

void LPC_Dequantize(const uint8_t bits[LPC_Q_FRAME_BYTES], LPC_Frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->a[0] = 1.0f;

    /* Gain: lower nibble of bits[1] */
    uint8_t qg  = bits[1] & 0x0FU;
    float   lg  = (float)qg * 10.0f / 15.0f - 10.0f;
    f->gain     = exp2f(lg);

    /* Voiced flag + pitch */
    uint8_t voiced     = (bits[2] >> 7) & 1U;
    uint8_t qp         = bits[2] & 0x7FU;
    f->pitch_samps     = voiced ? (uint16_t)((uint16_t)qp + 19U) : 0U;

    /* LPC coefficients a[1..10] */
    for (int k = 1; k <= 10; k++) {
        int bi    = 3 + (k - 1) / 2;
        uint8_t qa;
        if ((k - 1) % 2 == 0)
            qa = (bits[bi] >> 4) & 0x0FU;
        else
            qa = bits[bi] & 0x0FU;
        f->a[k] = (float)qa * 4.0f / 15.0f - 2.0f;
    }
}
