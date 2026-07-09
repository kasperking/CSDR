/**
  ******************************************************************************
  * @file    fdv_modem.h
  * @brief   FreeDV narrowband OFDM modem — TX + RX with frame acquisition
  *
  *  16-carrier DQPSK OFDM at 8 kHz sample rate.
  *
  *  Carrier layout:
  *    f[k] = 450 + k × 75 Hz  for k = 0..15
  *    → 450, 525, 600, 675, 750, 825, 900, 975, 1050, 1125,
  *       1200, 1275, 1350, 1425, 1500, 1575 Hz
  *    Total occupied bandwidth: 450–1575 Hz (fits NBUSB passband).
  *
  *  Super-frame (40 ms = 320 samples at 8 kHz):
  *    Symbol 0  — PILOT   (107 smp): all carriers at reference phase
  *    Symbol 1  — DATA1   (107 smp): carries bits  0-31  (16 carriers × 2 QPSK)
  *    Symbol 2  — DATA2   (106 smp): carries bits 32-63
  *    Total: 107 + 107 + 106 = 320 samples  ✓
  *
  *  DQPSK phase step per symbol per carrier:
  *    bits 00 → Δφ = 0
  *    bits 01 → Δφ = π/2
  *    bits 10 → Δφ = π
  *    bits 11 → Δφ = 3π/2
  *  Phase offsets are reset to 0 at every super-frame start so the pilot
  *  always provides a fresh reference for the RX differential decoder.
  *
  *  Output amplitude: FDV_AMP = 0.125 per carrier.
  *    RMS ≈ 0.125 × √16 = 0.5  (normalised for NBUSB Hilbert SSB input)
  *    Peak ≤ 2.0  (worst-case all-in-phase, statistically rare for data)
  *
  *  CPU estimate: ~320 × 16 LUT lookups per 40 ms ≈ 0.05 ms  (< 0.2 % load)
  ******************************************************************************
  */

#ifndef __FDV_MODEM_H
#define __FDV_MODEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Constants ───────────────────────────────────────────── */
#define FDV_CARRIERS      16U    /*!< Number of DQPSK carriers             */
#define FDV_FRAME_SAMPS  320U    /*!< Samples per 40 ms super-frame @ 8kHz */
#define FDV_SYM0_SAMPS   107U   /*!< Pilot symbol length                   */
#define FDV_SYM1_SAMPS   107U   /*!< Data symbol 1 length                  */
#define FDV_SYM2_SAMPS   106U   /*!< Data symbol 2 length (320-107-107)    */
#define FDV_AMP          0.125f  /*!< Per-carrier amplitude                */

/* ── State ───────────────────────────────────────────────── */
typedef struct {
    uint32_t phase_acc[FDV_CARRIERS];  /*!< NCO accumulators (32-bit wrap) */
    uint32_t phase_inc[FDV_CARRIERS];  /*!< Fixed frequency increments      */
    uint32_t phase_off[FDV_CARRIERS];  /*!< DQPSK phase offsets             */
    float    amp;                       /*!< Per-carrier amplitude           */
} FdvModem_t;

/* ── API ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise modem state and precompute cosine LUT.
 */
void FdvModem_Init(FdvModem_t *m);

/**
 * @brief  Encode 64 bits into one 320-sample super-frame at 8 kHz.
 *
 *  Produces the OFDM baseband audio (450–1575 Hz) that feeds the
 *  Hilbert NBUSB SSB upconvertor → RF.
 *
 * @param  m     Modem state (phase_acc[k] advanced by 320 steps)
 * @param  bits  Input: 8 bytes of quantized LPC data (64 bits)
 * @param  out   Output: FDV_FRAME_SAMPS floats at 8 kHz, RMS ≈ 0.5
 */
void FdvModem_EncodeSuperFrame(FdvModem_t *m,
                               const uint8_t bits[8],
                               float out[FDV_FRAME_SAMPS]);

/* ── RX demodulator ──────────────────────────────────────── */

#define FDV_RXBITS_BYTES  8U   /*!< 64 bits = 8 bytes per LPC frame */

/**
 * @brief  Per-carrier DQPSK RX correlator state.
 *
 *  Each call to FdvModem_RxPush() accumulates one 8 kHz sample.
 *  At each symbol boundary the correlators are snapshotted and reset.
 *  After the third symbol (DATA2) the 64-bit LPC frame is recovered
 *  via differential phase detection and frame_ready is set.
 *
 *  Frame structure (320 samples @ 8 kHz):
 *    PILOT  : 107 samples — all carriers at reference phase
 *    DATA1  : 107 samples — DQPSK bits  0-31 vs PILOT
 *    DATA2  : 106 samples — DQPSK bits 32-63 vs DATA1
 */
typedef struct {
    uint32_t phase_inc[FDV_CARRIERS];    /*!< Fixed carrier increments         */
    uint32_t phase_acc[FDV_CARRIERS];    /*!< Continuous NCO accumulators      */
    float    corr_i[FDV_CARRIERS];       /*!< Current symbol I correlator      */
    float    corr_q[FDV_CARRIERS];       /*!< Current symbol Q correlator      */
    float    pilot_i[FDV_CARRIERS];      /*!< PILOT I snapshot                 */
    float    pilot_q[FDV_CARRIERS];      /*!< PILOT Q snapshot                 */
    float    sym1_i[FDV_CARRIERS];       /*!< DATA1 I snapshot                 */
    float    sym1_q[FDV_CARRIERS];       /*!< DATA1 Q snapshot                 */
    uint16_t samp_cnt;                   /*!< Samples within current frame     */
    uint8_t  rx_bits[FDV_RXBITS_BYTES];  /*!< Recovered 64-bit LPC frame       */
    bool     frame_ready;                /*!< True when rx_bits[] is valid     */
} FdvModemRx_t;

/**
 * @brief  Initialise RX demodulator state.
 */
void FdvModem_RxInit(FdvModemRx_t *rx);

/**
 * @brief  Push one 8 kHz baseband sample into the OFDM demodulator.
 *
 *  Call once per 8 kHz sample.  Sets rx->frame_ready when 320 samples
 *  have been processed and rx->rx_bits[] contains the decoded LPC frame.
 *  Caller must clear frame_ready after consuming the bits.
 */
void FdvModem_RxPush(FdvModemRx_t *rx, float s);

/* ── Frame acquisition ──────────────────────────────────────────────────
 *
 *  A lone FdvModemRx_t only decodes correctly if its internal super-frame
 *  boundary (samp_cnt == 0) already coincides with the incoming signal's —
 *  true only by coincidence, since TX and RX are independent radios keyed
 *  at arbitrary times with no shared time reference. There is no pilot
 *  search: a misaligned single instance will integrate across two
 *  different symbols and its sync word will (almost) never verify.
 *
 *  FdvModemBank_t runs FDV_SYNC_HYPS demodulator instances in parallel,
 *  each started at a different phase within one super-frame period
 *  (FDV_HYP_SPACING samples apart), so their frame boundaries are
 *  staggered across the full ambiguity window. Whichever hypothesis's
 *  boundary lands close enough to the true one will decode a valid sync
 *  word every super-frame; the caller (freedv_mode.c) picks that one and
 *  ignores the rest. No state reset is needed to "re-acquire" — every
 *  hypothesis keeps free-running, so if the signal reappears at a new
 *  offset (e.g. a fresh PTT), whichever hypothesis is now closest simply
 *  starts verifying again on its own.
 *
 *  RAM cost: FDV_SYNC_HYPS × sizeof(FdvModemRx_t) ≈ FDV_SYNC_HYPS × 530 B.
 * ────────────────────────────────────────────────────────────────────── */
#define FDV_SYNC_HYPS    8U
#define FDV_HYP_SPACING  (FDV_FRAME_SAMPS / FDV_SYNC_HYPS)  /*!< 40 samples */

typedef struct {
    FdvModemRx_t hyp[FDV_SYNC_HYPS];
} FdvModemBank_t;

/**
 * @brief  Initialise all hypotheses, staggering their starting phase.
 */
void FdvModemBank_Init(FdvModemBank_t *bank);

/**
 * @brief  Push one 8 kHz sample into every hypothesis in the bank.
 * @return Bitmask (bit h set for hypothesis h) of hypotheses whose
 *         rx_bits[] holds a freshly decoded 64-bit frame this call.
 *         0 if none completed a super-frame this sample. Each set
 *         hypothesis's frame_ready is cleared automatically.
 */
uint16_t FdvModemBank_Push(FdvModemBank_t *bank, float s);

#ifdef __cplusplus
}
#endif
#endif /* __FDV_MODEM_H */
