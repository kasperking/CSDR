/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ft8_mode.h
  * @brief   Standalone FT8 receive decoder (Phase 1: RX decode + display)
  *
  *  Decodes FT8 transmissions entirely on the MCU — no PC needed.
  *  Built on ft8_lib (Karlis Goba, YL3JG — MIT licence): LDPC(174,91)
  *  belief-propagation decoder, CRC-14, 77-bit message unpacker.
  *
  *  Architecture (everything runs in CSDR_Loop context — no ISR work):
  *    • DSP_Process RX tap (post-demod audio, pre-NR/AGC/squelch) feeds
  *      FT8_FeedAudio() at 48 kHz; a 3-stage half-band cascade decimates
  *      to 6 kHz and fills 80 ms sub-blocks.
  *    • FT8_Poll() consumes sub-blocks into an STFT waterfall (uint8 dB
  *      magnitudes, 93×2×2×449 ≈ 163 KB, placed in RAM_D2 via .ft8_ram),
  *      tracks the 15 s FT8 slot from the RTC (±auto-DT correction learnt
  *      from decoded signals), then runs candidate search + LDPC decode
  *      in ≤~1.5 ms slices so the main-loop audio pump is never starved.
  *    • Decoded messages rotate through the INFO text strip
  *      (SDR_UI_DrawCWText) — active in DIGU/USB mode during RX only.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __FT8_MODE_H
#define __FT8_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* One-time init: enables D2 SRAM clocks, builds FFT tables/window.
 * Call from CSDR_Init after settings restore (needs g_sdr defaults). */
void FT8_Init(void);

/* Menu toggle.  Enabling arms the decoder; it becomes active only while
 * mode is DIGU/USB and RX (see FT8_Poll).  Disabling clears the display. */
void FT8_SetEnabled(bool on);
bool FT8_GetEnabled(void);

/* Per-sample RX audio tap @48 kHz — called from DSP_Process (CSDR_Loop
 * context) when g_ft8_tap_enable is set.  Cost when capturing: one 31-tap
 * FIR at 48 k + half-rate stages (~2-3 % CPU). */
void FT8_FeedAudio(float sample);

/* Cooperative scheduler tick — call once per CSDR_Loop iteration.
 * Does at most ~1.5 ms of work (STFT / candidate search / LDPC decode). */
void FT8_Poll(void);

/* Fast gate read by DSP_Process; owned (written) by FT8_Poll.  Both run in
 * CSDR_Loop context, so no volatile/atomics needed. */
extern bool g_ft8_tap_enable;

/* ── App-facing status/results (consumed by ft8_app.c full-screen viewer) ── */

typedef struct {
  char    text[36];   /* decoded message (FTX_MAX_MESSAGE_LENGTH + NUL)      */
  int16_t dt_ms;      /* signal start vs nominal (+0.5 s) slot start         */
  int16_t freq_hz;    /* audio offset of tone 0                              */
  int8_t  snr_db;     /* crude estimate from sync score                      */
} FT8_Decode_t;

typedef enum {
  FT8_STATE_IDLE = 0, /* gated off (mode/TX) or disabled                     */
  FT8_STATE_WAIT,     /* waiting for next 15 s slot boundary                 */
  FT8_STATE_CAPTURE,  /* filling waterfall                                   */
  FT8_STATE_SEARCH,   /* candidate search                                    */
  FT8_STATE_DECODE,   /* LDPC decode burst                                   */
} FT8_State_t;

typedef struct {
  FT8_State_t state;
  int32_t     cycle_ms;     /* position inside the 15 s slot, 0..14999       */
  uint8_t     num_dec;      /* decodes stored for the last finalised slot    */
  uint32_t    slot_seq;     /* increments at every slot finalise             */
  int32_t     dt_offset_ms; /* current auto-DT/phase-scan correction         */
  uint8_t     num_blocks;   /* waterfall blocks captured this slot (0..85)   */
  uint16_t    num_cand;     /* candidates from the last completed search     */
  int16_t     best_score;   /* best sync score of the last search            */
  bool        init_ok;      /* false = FFT work area alloc failed at boot    */
} FT8_Status_t;

void FT8_GetStatus(FT8_Status_t *st);
/* Valid idx: 0..num_dec-1 of the last finalised slot; NULL out of range.
 * Entries stay valid until the next slot finalise (same CSDR_Loop context). */
const FT8_Decode_t *FT8_GetDecode(uint8_t idx);

/* false = a full-screen app owns the display: the engine stops drawing the
 * INFO strip (rotation + placeholders).  Default true. */
void FT8_SetStripUI(bool on);

/* Live position inside the 15 s FT8 slot (RTC + learnt corrections), in ms.
 * Unlike FT8_Status_t.cycle_ms this stays live while the decoder is gated
 * off (e.g. during our own TX) — used by ft8_app for TX slot scheduling. */
int32_t FT8_GetCycleMs(void);

/* ── TX beacon (Phase 2) ──────────────────────────────────────────────────
 * Usage (ft8_app): FT8_TxSetMessage("CQ MYCALL AB12") once, then per slot:
 * FT8_TxStart(FT8_GetCycleMs()); CSDR_RequestTX(true); ... poll FT8_TxDone()
 * → CSDR_RequestTX(false); FT8_TxStop().  FT8_TxSample() is called by
 * DSP_ProcessTX at 48 kHz while g_ft8_tx_tone is set; the tone is injected
 * pre-gain so digi drive / tx_power / PA protection apply unchanged. */
bool  FT8_TxSetMessage(const char *text);  /* pack77+LDPC; false = bad text */
void  FT8_TxStart(int32_t cycle_ms_now);   /* arm synth; silent until +0.5 s */
void  FT8_TxStop(void);
bool  FT8_TxDone(void);                    /* all 79 symbols emitted         */
float FT8_TxSample(void);                  /* 48 kHz sample, ±0.9 pre-gain   */

extern bool g_ft8_tx_tone;                 /* gate read by DSP_ProcessTX     */

#ifdef __cplusplus
}
#endif
#endif /* __FT8_MODE_H */
