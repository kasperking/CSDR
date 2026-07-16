#ifndef __RTTY_DECODE_H
#define __RTTY_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── RTTY RX decoder (Baudot/ITA2, 45.45 Bd, 170 Hz shift) ──────────────────
 *
 * Audio-domain FSK decoder fed from the post-demod/pre-AGC tap in
 * DSP_Process (same tap point and gating pattern as FT8_FeedAudio).
 * Two tone bandpass filters (mark 2125 Hz / space 2295 Hz — the standard
 * LSB tone pair) feed an ATC slicer (automatic threshold correction): the
 * peaks/valleys of the envelope difference set the decision threshold, so
 * selective fading of one tone and cross-filter leakage cancel out.
 * Debounced mark/space transitions are timestamped
 * on a 48 kHz sample counter and queued in an edge ring — the same
 * main-loop-stall-proof scheme as the CW decoder (see cw_decode.h).
 *
 * RTTY_Poll (main loop) replays the edge timeline through a UART-style
 * framer: start bit validated at mid-bit, 5 data bits LSB-first, stop bit
 * must be mark (1.5-bit stop, sampled at 6.5 bit periods).  Framing errors
 * force a wait-for-mark resync, which suppresses noise garbage.  Decoded
 * Baudot codes pass through LTRS/FIGS shift state (USOS: space unshifts)
 * into a text ring read by RTTY_GetText.
 *
 * All entry points run in main-loop context (csdr_process_audio_pending /
 * CSDR_Loop / the FT8 app's audio pump) — no ISR involved, no locking.
 *
 * Sideband note: amateur RTTY is transmitted with mark on the higher RF
 * frequency.  Received in LSB/DIGL, mark lands on the LOWER audio tone
 * (2125 Hz) — the "normal" sense.  Received in USB/DIGU the audio spectrum
 * is inverted, so the caller must set RTTY_SetReverse(true) there.
 */

/* Decoded-text ring depth (chars), same as the CW decoder strip source */
#define RTTY_TEXT_LEN   64U

/* Mark audio tone (Hz): fixed at the amateur convention; the space tone sits
 * shift_hz above it.  Baud/shift are selected via RTTY_Configure from the
 * option tables below (menu indices map 1:1, persisted in EEPROM). */
#define RTTY_MARK_HZ    2125.0f
#define RTTY_TONE_BW_HZ 150.0f         /* per-tone BPF bandwidth */
#define RTTY_AFC_RANGE_HZ 60           /* decoder-side AFC correction limit */

#define RTTY_BAUD_COUNT  3U
#define RTTY_SHIFT_COUNT 3U
extern const uint16_t g_rtty_baud_x100[RTTY_BAUD_COUNT];  /* 4545 5000 7500 */
extern const uint16_t g_rtty_shift_hz[RTTY_SHIFT_COUNT];  /* 170 425 850    */

/* Producer gate — sdr_dsp.c calls RTTY_FeedAudio(audio) when set.  Owned by
 * CSDR_Loop (recomputed every iteration from rtty_decode_on + mode + RX). */
extern bool g_rtty_tap_enable;

/* One-time init (boot): build filter coefficients for the sample rate and
 * clear all state including the text ring.  Config defaults to 45.45 Bd /
 * 170 Hz; call RTTY_Configure after loading persisted settings. */
void RTTY_Init(uint32_t sample_rate);

/* Select modulation rate (baud × 100) and mark→space shift (Hz).  Rebuilds
 * filters and bit timing, zeroes the AFC offset and resets the framer (text
 * ring preserved).  Values come from g_rtty_baud_x100 / g_rtty_shift_hz. */
void RTTY_Configure(uint16_t baud_x100, uint16_t shift_hz);

/* Current decoder-side AFC correction (Hz, signed) — diagnostics/UI.  The
 * AFC tracks the tone pair within ±RTTY_AFC_RANGE_HZ by re-centring the
 * tone filters; it never touches the radio's tuning. */
int16_t RTTY_GetAfcHz(void);

/* Reset demod + framer state and drop queued edges.  The text ring and
 * debug counters are preserved (same semantics as CWDec_Reset).  Call on
 * mode change, decode toggle, and TX→RX transitions. */
void RTTY_Reset(void);

/* Audio-tone polarity: false = mark on 2125 Hz (LSB/DIGL convention),
 * true = sense swapped (USB/DIGU).  Call RTTY_Reset after changing. */
void RTTY_SetReverse(bool rev);

/* Per-sample feed from DSP_Process, post-demod pre-AGC, 48 kHz.  Cost when
 * enabled: 2 biquads + 2 envelope/peak IIRs + comparator (≈50 cycles). */
void RTTY_FeedAudio(float sample);

/* Main-loop framer: consume queued edges, assemble characters.  Returns
 * true when the text ring changed (new printable char or space). */
bool RTTY_Poll(void);

/* Copy the last max_chars decoded characters (null-terminated) into buf,
 * oldest-to-newest — same contract as CWDec_GetText. */
void RTTY_GetText(char *buf, uint8_t max_chars);

#ifdef __cplusplus
}
#endif
#endif /* __RTTY_DECODE_H */
