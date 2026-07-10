/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ft8_mode.c
  * @brief   Standalone FT8 receive decoder engine (see ft8_mode.h)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "ft8_mode.h"
#include "csdr_app.h"
#include "sdr_dsp.h"
#include "sdr_ui.h"
#include "ft8_constants.h"
#include "ft8_decode.h"
#include "ft8_message.h"
#include "ft8_encode.h"
#include "kiss_fftr.h"
#include "rtc_clock.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── Decoder geometry ─────────────────────────────────────────────────────────
 * 48 kHz tap → ÷8 half-band cascade → 6 kHz mono audio.
 * FT8 symbol = 0.16 s = 960 samples; time_osr=2 → one 1920-pt real FFT per
 * 480-sample (80 ms) sub-block; freq_osr=2 → 3.125 Hz waterfall grid.
 * Decode band 100..2900 Hz (top edge limited by the ÷2 half-band roll-off). */
#define FT8M_SR             6000U
#define FT8M_BLOCK          960U                       /* samples / symbol      */
#define FT8M_TIME_OSR       2U
#define FT8M_FREQ_OSR       2U
#define FT8M_SUBBLOCK       (FT8M_BLOCK / FT8M_TIME_OSR)      /* 480 = 80 ms    */
#define FT8M_NFFT           (FT8M_BLOCK * FT8M_FREQ_OSR)      /* 1920           */
#define FT8M_MIN_BIN        16    /* 100 Hz  × 0.16 s                           */
#define FT8M_MAX_BIN        465   /* 2900 Hz × 0.16 s + 1                       */
#define FT8M_NUM_BINS       (FT8M_MAX_BIN - FT8M_MIN_BIN)     /* 449            */
#define FT8M_MAX_BLOCKS     93    /* 15 s / 0.16 s                              */
#define FT8M_DECODE_BLOCKS  85    /* start decoding at 13.6 s into the slot     */
#define FT8M_BLOCK_STRIDE   (FT8M_TIME_OSR * FT8M_FREQ_OSR * FT8M_NUM_BINS)

#define FT8M_SLOT_MS        15000
#define FT8M_MAX_CAND       140   /* heap saturated at 100 on a busy band   */
#define FT8M_MIN_SCORE      10
#define FT8M_LDPC_ITERS     20
#define FT8M_MAX_DECODES    20
#define FT8M_POLL_BUDGET_US 1500U  /* per-poll work cap; audio pump needs the
                                      loop back within ~5 ms (256-sample half) */
#define FT8M_UI_ROTATE_MS   2500U

/* ── Large buffers → RAM_D2 (0x30000000, 288 KB, otherwise unused).
 * CPU-only (no DMA), default MPU map (cacheable) — no coherency concerns.
 * Section is NOLOAD: startup code does not touch it; everything is
 * initialised explicitly in FT8_Init / capture start. */
#define FT8M_D2 __attribute__((section(".ft8_ram")))

static uint8_t      s_wf_mag[FT8M_MAX_BLOCKS * FT8M_BLOCK_STRIDE] FT8M_D2;  /* ~163 KB */
static float        s_window[FT8M_NFFT]      FT8M_D2;
static float        s_last_frame[FT8M_NFFT]  FT8M_D2;
static float        s_timedata[FT8M_NFFT]    FT8M_D2;
static kiss_fft_cpx s_freqdata[FT8M_NFFT / 2U + 1U] FT8M_D2;
static uint8_t      s_fft_work[24576] FT8M_D2 __attribute__((aligned(8)));

/* Sub-block hand-off ring: FeedAudio (48 kHz tap) → Poll (STFT).  4 slots =
 * 320 ms of slack against main-loop stalls (menu, overlays). */
#define FT8M_SB_RING 4U
static float s_sb_ring[FT8M_SB_RING][FT8M_SUBBLOCK] FT8M_D2;

/* ── Small state (DTCM .bss) ─────────────────────────────────────────────── */
bool g_ft8_tap_enable = false;

typedef enum {
  FT8_ST_IDLE = 0,   /* not active (mode/TX gate or disabled)                */
  FT8_ST_WAIT,       /* active, waiting for the next 15 s slot boundary      */
  FT8_ST_CAPTURE,    /* filling the waterfall                                */
  FT8_ST_SEARCH,     /* sliced Costas-sync candidate search                  */
  FT8_ST_DECODE,     /* sliced LDPC decode of ranked candidates              */
} ft8_state_t;

typedef struct {
  FT8_Decode_t pub;                              /* app-facing part          */
  uint8_t payload[FTX_PAYLOAD_LENGTH_BYTES];     /* for de-duplication       */
} ft8_decode_t;

static bool            s_init_ok   = false;
static bool            s_enabled   = false;
static ft8_state_t     s_state     = FT8_ST_IDLE;
static bool            s_capturing = false;
static bool            s_ui_drawn  = false;

static kiss_fftr_cfg   s_fft_cfg;
static ftx_waterfall_t s_wf;

/* ÷8 decimator: three ÷2 half-band stages (31-tap Hann-sinc, fc=0.23·Fs_in) */
static FIR_Filter_t s_dec_fir[3];
static uint8_t      s_dec_cnt[3];

static uint16_t s_sb_fill;      /* samples in the ring slot being written    */
static uint8_t  s_sb_wr, s_sb_rd, s_sb_pending;
static uint16_t s_subblocks_done;

/* Slot clock */
static int32_t  s_dt_offset_ms; /* auto-DT correction added to the RTC phase */
static int32_t  s_prev_cyc_ms;

/* Sliced search/decode context */
static ftx_candidate_t s_cand[FT8M_MAX_CAND];
static int      s_heap_size;
static int      s_search_cursor;   /* next time_offset to score (−10..20)    */
static int      s_cand_idx;

/* Results */
static ft8_decode_t s_dec[FT8M_MAX_DECODES];
static uint8_t   s_num_dec;
static uint32_t  s_slot_seq;    /* bumped at every slot finalise            */
static bool      s_strip_ui = true;  /* false = ft8_app owns the display    */
static uint8_t   s_ui_idx;
static uint32_t  s_ui_tick_ms;

/* Acquisition + per-slot diagnostics */
static uint8_t   s_dry_slots;   /* consecutive slots with zero decodes      */
static uint16_t  s_last_ncand;  /* candidates found in the last search      */
static int16_t   s_last_best;   /* best sync score in the last search       */
static bool      s_rtc_baked;   /* slot phase written back to RTC this boot */

/* Debug counters (inspect in debugger) */
volatile uint32_t dbg_ft8_slots;        /* capture cycles started             */
volatile uint32_t dbg_ft8_decodes;      /* total unique messages decoded      */
volatile uint32_t dbg_ft8_crc_pass;     /* candidates that passed LDPC+CRC    */
volatile uint32_t dbg_ft8_sb_drop;      /* sub-blocks dropped (loop stalled)  */
volatile uint32_t dbg_ft8_aborts;       /* decode bursts cut by slot boundary */
volatile uint32_t dbg_ft8_dt_adjust;    /* auto-DT corrections applied        */
volatile int32_t  dbg_ft8_dt_offset_ms; /* mirror of s_dt_offset_ms           */
volatile uint32_t dbg_ft8_init_fail;    /* FFT work area too small            */
volatile uint32_t dbg_ft8_phase_scan;   /* acquisition phase rotations        */
volatile uint32_t dbg_ft8_last_ncand;   /* candidates in last completed search*/
volatile int32_t  dbg_ft8_last_best;    /* best sync score in last search     */
volatile int32_t  dbg_ft8_rtc_bake_ms;  /* shift written into the RTC at lock */

/* ── Slot clock ───────────────────────────────────────────────────────────────
 * Position inside the 15 s FT8 cycle, in ms.  Primary source: RTC seconds
 * (BYPSHAD direct reads) + SSR sub-seconds (PREDIV_S=255 → 1/256 s).  60 s is
 * a multiple of 15 s, so only the seconds-of-minute field matters — the
 * decoder works even when the wall time is not set, and the auto-DT feedback
 * (learnt from decoded signals) pulls the phase onto the real FT8 slot grid.
 * Falls back to HAL_GetTick when the RTC/LSE is dead. */
static int32_t ft8_cycle_ms(void)
{
  int32_t base;
  if (READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY) && READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN)) {
    uint32_t ss, tr;
    do {                       /* BYPSHAD: re-read until SSR/TR are coherent */
      ss = RTC->SSR;
      tr = RTC->TR;
    } while (ss != RTC->SSR);
    uint32_t sec = (((tr >> 4U) & 0x7U) * 10U) + (tr & 0xFU);
    uint32_t ms  = ((255U - (ss & 0xFFU)) * 1000U) >> 8U;
    base = (int32_t)((sec % 15U) * 1000U + ms);
  } else {
    base = (int32_t)(HAL_GetTick() % (uint32_t)FT8M_SLOT_MS);
  }
  int32_t t = (base + s_dt_offset_ms) % FT8M_SLOT_MS;
  if (t < 0) t += FT8M_SLOT_MS;
  return t;
}

/* ── Init ─────────────────────────────────────────────────────────────────── */
void FT8_Init(void)
{
  /* D2 SRAM blocks are clock-gated out of reset — enable before first touch */
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();

  size_t fft_len = sizeof(s_fft_work);
  s_fft_cfg = kiss_fftr_alloc((int)FT8M_NFFT, 0, s_fft_work, &fft_len);
  if (s_fft_cfg == NULL) {
    dbg_ft8_init_fail = 1U;   /* work area too small — decoder stays off */
    s_init_ok = false;
    return;
  }

  /* Hann window scaled by kiss_fftr norm (2/N), as in ft8_lib's monitor */
  const float norm = 2.0f / (float)FT8M_NFFT;
  for (uint32_t i = 0U; i < FT8M_NFFT; i++) {
    float s = sinf(3.14159265358979f * (float)i / (float)FT8M_NFFT);
    s_window[i] = norm * s * s;
  }
  memset(s_last_frame, 0, sizeof(s_last_frame));

  for (int k = 0; k < 3; k++) {
    FIR_Init_LPF(&s_dec_fir[k], 0.23f, 31U);
    s_dec_cnt[k] = 0U;
  }

  s_wf.max_blocks   = FT8M_MAX_BLOCKS;
  s_wf.num_blocks   = 0;
  s_wf.num_bins     = FT8M_NUM_BINS;
  s_wf.time_osr     = FT8M_TIME_OSR;
  s_wf.freq_osr     = FT8M_FREQ_OSR;
  s_wf.block_stride = FT8M_BLOCK_STRIDE;
  s_wf.mag          = s_wf_mag;
  s_wf.protocol     = FTX_PROTOCOL_FT8;

  s_init_ok = true;
}

void FT8_SetEnabled(bool on)
{
  s_enabled = on;
  /* State/UI transitions are handled centrally in FT8_Poll */
}

bool FT8_GetEnabled(void) { return s_enabled; }

/* ── 48 kHz audio tap ────────────────────────────────────────────────────── */
void FT8_FeedAudio(float sample)
{
  /* Stages run even when not capturing so the filters stay warm and the
   * first sub-block after a slot boundary has no startup transient. */
  float y = FIR_Process(&s_dec_fir[0], sample);
  if ((++s_dec_cnt[0] & 1U) != 0U) return;
  y = FIR_Process(&s_dec_fir[1], y);
  if ((++s_dec_cnt[1] & 1U) != 0U) return;
  y = FIR_Process(&s_dec_fir[2], y);
  if ((++s_dec_cnt[2] & 1U) != 0U) return;

  if (!s_capturing) return;

  s_sb_ring[s_sb_wr][s_sb_fill++] = y;
  if (s_sb_fill >= FT8M_SUBBLOCK) {
    s_sb_fill = 0U;
    if (s_sb_pending >= (uint8_t)(FT8M_SB_RING - 1U)) {
      dbg_ft8_sb_drop++;        /* main loop stalled >240 ms — drop, resync next slot */
    } else {
      s_sb_wr = (uint8_t)((s_sb_wr + 1U) % FT8M_SB_RING);
      s_sb_pending++;
    }
  }
}

/* ── STFT: one 80 ms sub-block → one waterfall row ───────────────────────── */
static void ft8_stft_subblock(const float *sb)
{
  if (s_subblocks_done >= (uint16_t)(FT8M_MAX_BLOCKS * FT8M_TIME_OSR)) return;

  memmove(s_last_frame, s_last_frame + FT8M_SUBBLOCK,
          (FT8M_NFFT - FT8M_SUBBLOCK) * sizeof(float));
  memcpy(s_last_frame + (FT8M_NFFT - FT8M_SUBBLOCK), sb,
         FT8M_SUBBLOCK * sizeof(float));

  for (uint32_t i = 0U; i < FT8M_NFFT; i++)
    s_timedata[i] = s_window[i] * s_last_frame[i];

  kiss_fftr(s_fft_cfg, s_timedata, s_freqdata);

  /* Layout must match ftx_waterfall_t: [block][time_sub][freq_sub][bin].
   * Sub-blocks arrive strictly in time order, so this append is exact. */
  uint8_t *out = s_wf_mag + ((uint32_t)s_subblocks_done * FT8M_FREQ_OSR * FT8M_NUM_BINS);
  for (uint32_t freq_sub = 0U; freq_sub < FT8M_FREQ_OSR; freq_sub++) {
    for (uint32_t bin = FT8M_MIN_BIN; bin < FT8M_MAX_BIN; bin++) {
      uint32_t src = bin * FT8M_FREQ_OSR + freq_sub;
      float mag2 = (s_freqdata[src].r * s_freqdata[src].r)
                 + (s_freqdata[src].i * s_freqdata[src].i);
      float db   = 10.0f * log10f(1e-12f + mag2);
      int scaled = (int)(2.0f * db + 240.0f);   /* 0..240 ↔ −120..0 dB, 0.5 dB */
      if (scaled < 0)   scaled = 0;
      if (scaled > 255) scaled = 255;
      *out++ = (uint8_t)scaled;
    }
  }

  s_subblocks_done++;
  if ((s_subblocks_done % FT8M_TIME_OSR) == 0U)
    s_wf.num_blocks = (int)(s_subblocks_done / FT8M_TIME_OSR);
}

/* ── Capture control ─────────────────────────────────────────────────────── */
static void ft8_start_capture(void)
{
  s_wf.num_blocks   = 0;
  s_subblocks_done  = 0U;
  s_sb_fill         = 0U;
  s_sb_wr = s_sb_rd = 0U;
  s_sb_pending      = 0U;
  s_capturing       = true;
  s_state           = FT8_ST_CAPTURE;
  dbg_ft8_slots++;
}

/* ── Decode helpers ──────────────────────────────────────────────────────── */

/* SNR estimate straight from the waterfall: mean Costas-tone cell magnitude
 * vs mean of noise cells just outside the 8-tone block, referred to the
 * WSJT-X 2500 Hz noise bandwidth (−29 dB for a 3.125 Hz cell).  +4 dB
 * compensates the 2-symbol Hann analysis window smearing the tone energy.
 * Unlike the sync score (bounded by its neighbour comparisons), this scales
 * with strong signals instead of saturating ~15 dB low. */
static int8_t ft8_snr_estimate(const ftx_candidate_t *c)
{
  int32_t sig = 0, nsig = 0, noi = 0, nnoi = 0;
  for (int m = 0; m < FT8_NUM_SYNC; m++) {
    for (int k = 0; k < FT8_LENGTH_SYNC; k++) {
      int babs = c->time_offset + (m * FT8_SYNC_OFFSET) + k;
      if (babs < 0 || babs >= s_wf.num_blocks) continue;
      const uint8_t *p8 = s_wf_mag
        + ((uint32_t)babs * FT8M_BLOCK_STRIDE)
        + (((uint32_t)c->time_sub * FT8M_FREQ_OSR + c->freq_sub) * FT8M_NUM_BINS)
        + (uint32_t)c->freq_offset;
      sig += p8[kFT8_Costas_pattern[k]];
      nsig++;
      if (c->freq_offset >= 3)                    { noi += p8[-3] + p8[-2];  nnoi += 2; }
      if ((c->freq_offset + 11) < FT8M_NUM_BINS)  { noi += p8[9]  + p8[10];  nnoi += 2; }
    }
  }
  if (nsig == 0 || nnoi == 0) return -28;
  int snr = (int)((sig / nsig) - (noi / nnoi)) / 2 - 29 + 4;   /* WF units = 0.5 dB */
  if (snr < -28) snr = -28;
  if (snr >  35) snr =  35;
  return (int8_t)snr;
}

static void ft8_store_decode(const ftx_candidate_t *c, const ftx_message_t *msg)
{
  /* Dedup: the same transmission decodes from several nearby candidates */
  for (uint8_t i = 0U; i < s_num_dec; i++) {
    if (memcmp(s_dec[i].payload, msg->payload, FTX_PAYLOAD_LENGTH_BYTES) == 0)
      return;
  }
  if (s_num_dec >= FT8M_MAX_DECODES) return;

  ft8_decode_t *d = &s_dec[s_num_dec];
  memcpy(d->payload, msg->payload, FTX_PAYLOAD_LENGTH_BYTES);

  ftx_message_offsets_t offs;
  if (ftx_message_decode(msg, NULL, d->pub.text, &offs) != FTX_MESSAGE_RC_OK)
    snprintf(d->pub.text, sizeof(d->pub.text), "<i3.n3 %u.%u>",
             (unsigned)ftx_message_get_i3(msg), (unsigned)ftx_message_get_n3(msg));

  /* freq = (min_bin + offset) × 6.25 Hz + sub × 3.125 Hz */
  uint32_t f4 = ((uint32_t)(FT8M_MIN_BIN + c->freq_offset) * 25U)
              + ((uint32_t)c->freq_sub * 25U / 2U);
  d->pub.freq_hz = (int16_t)(f4 / 4U);

  /* dt = symbol-grid start − nominal 0.5 s TX start */
  d->pub.dt_ms = (int16_t)((int32_t)c->time_offset * 160
                           + (int32_t)c->time_sub * 80 - 500);

  d->pub.snr_db = ft8_snr_estimate(c);

  s_num_dec++;
  dbg_ft8_decodes++;
}

/* INFO strip: "i/N SNR DT FREQ TEXT" — integer-only formatting (printf float
 * is disabled project-wide / newlib-nano). */
static void ft8_ui_show(uint8_t idx)
{
  if (!s_strip_ui) return;   /* full-screen app owns the display */
  char line[64];
  if (s_num_dec == 0U) {
    snprintf(line, sizeof(line), "FT8: no decode");
  } else {
    const FT8_Decode_t *d = &s_dec[idx].pub;
    int dt_abs = (d->dt_ms < 0) ? -d->dt_ms : d->dt_ms;
    snprintf(line, sizeof(line), "%u/%u %+03d %c0.%u %4u %s",
             (unsigned)(idx + 1U), (unsigned)s_num_dec, (int)d->snr_db,
             (d->dt_ms < 0) ? '-' : '+', (unsigned)((dt_abs + 50) / 100),
             (unsigned)d->freq_hz, d->text);
  }
  SDR_UI_DrawCWText(line);
  s_ui_drawn = true;
}

/* ── Slot finalisation: auto-DT + first display ──────────────────────────── */
static void ft8_finalize_slot(void)
{
  if (s_num_dec > 0U) {
    /* Median DT of this slot's decodes: a consistent offset means our slot
     * clock is off — slew half of it into the cycle phase so the next
     * capture window centres the signals (works even with an unset RTC). */
    int16_t dts[FT8M_MAX_DECODES];
    for (uint8_t i = 0U; i < s_num_dec; i++) {
      dts[i] = s_dec[i].pub.dt_ms;
      for (int j = (int)i; j > 0 && dts[j] < dts[j - 1]; j--) {
        int16_t t = dts[j]; dts[j] = dts[j - 1]; dts[j - 1] = t;
      }
    }
    int32_t med = dts[s_num_dec / 2U];
    if (med > 240 || med < -240) {
      s_dt_offset_ms += (-med) / 2;
      s_dt_offset_ms %= FT8M_SLOT_MS;
      dbg_ft8_dt_adjust++;
    }
    dbg_ft8_dt_offset_ms = s_dt_offset_ms;
    s_dry_slots = 0U;

    /* One-shot per boot: bake the locked slot phase into the RTC (whole
     * seconds; VBAT keeps it) so the next power-up decodes immediately
     * instead of re-running the ~2 min acquisition scan.  Uses the shortest
     * mod-15 s equivalent, so the wall clock moves by at most ±7.5 s — and
     * since the FT8 grid is UTC-aligned, seconds actually get MORE accurate.
     * The rounding residual (≤0.5 s) stays in s_dt_offset_ms for auto-DT. */
    if (!s_rtc_baked && (med <= 240 && med >= -240)) {
      s_rtc_baked = true;              /* attempt once per boot */
      int32_t eq = ((s_dt_offset_ms % FT8M_SLOT_MS) + FT8M_SLOT_MS + 7500)
                   % FT8M_SLOT_MS - 7500;
      if (eq > 1500 || eq < -1500) {
        int32_t applied = RTC_Clock_ShiftMs(eq);   /* 0 when RTC is dead */
        s_dt_offset_ms -= applied;
        s_dt_offset_ms %= FT8M_SLOT_MS;
        dbg_ft8_rtc_bake_ms  = applied;
        dbg_ft8_dt_offset_ms = s_dt_offset_ms;
      }
    }
  } else if (++s_dry_slots >= 2U) {
    /* Acquisition phase scan.  The candidate search only covers signal
     * starts within about −1.6..+3.0 s of our capture start, i.e. ~4.6 s of
     * the 15 s slot.  With an unset/wrong RTC the slot phase can be off by
     * up to ±7.5 s and NOTHING ever decodes — and auto-DT cannot bootstrap
     * itself because it needs a decode first.  So after two consecutive dry
     * slots rotate the phase by a quarter slot; four steps sweep the whole
     * circle, locking onto a live band within ≤ 8 slots (2 min), after which
     * auto-DT takes over and the dry counter stays at zero. */
    s_dry_slots = 0U;
    s_dt_offset_ms = (s_dt_offset_ms + 3750) % FT8M_SLOT_MS;
    dbg_ft8_phase_scan++;
    dbg_ft8_dt_offset_ms = s_dt_offset_ms;
  }

  s_slot_seq++;
  s_ui_idx     = 0U;
  s_ui_tick_ms = HAL_GetTick();
  ft8_ui_show(0U);
  s_state = FT8_ST_WAIT;
}

/* ── Poll: cooperative scheduler tick ────────────────────────────────────── */
void FT8_Poll(void)
{
  if (!s_init_ok) return;

  bool active = s_enabled && !g_sdr.tx_mode
             && (g_sdr.mode == MODE_DIGU || g_sdr.mode == MODE_USB);
  g_ft8_tap_enable = active;

  if (!active) {
    if (s_state != FT8_ST_IDLE) {
      s_capturing = false;
      s_state     = FT8_ST_IDLE;
      s_num_dec   = 0U;
      s_dry_slots = 0U;
      if (s_ui_drawn) { SDR_UI_ClearCWText(); s_ui_drawn = false; }
    }
    return;
  }

  int32_t t_cyc = ft8_cycle_ms();
  bool boundary;
  if (s_state == FT8_ST_IDLE) {
    s_state    = FT8_ST_WAIT;
    s_num_dec  = 0U;
    boundary   = false;
    if (s_strip_ui) {
      SDR_UI_DrawCWText("FT8: sync...");
      s_ui_drawn = true;
    }
  } else {
    boundary = (t_cyc < s_prev_cyc_ms);   /* wrapped → new 15 s slot */
  }
  s_prev_cyc_ms = t_cyc;

  uint32_t budget_cyc = (SystemCoreClock / 1000000U) * FT8M_POLL_BUDGET_US;
  uint32_t t0 = DWT->CYCCNT;

  switch (s_state) {

  case FT8_ST_WAIT:
    if (boundary) ft8_start_capture();
    break;

  case FT8_ST_CAPTURE:
    if (boundary) {           /* under-filled slot (drops/late start) — resync */
      ft8_start_capture();
      break;
    }
    /* Drain pending sub-blocks (≤2 per poll fits the budget) */
    while (s_sb_pending > 0U && (DWT->CYCCNT - t0) < budget_cyc) {
      ft8_stft_subblock(s_sb_ring[s_sb_rd]);
      s_sb_rd = (uint8_t)((s_sb_rd + 1U) % FT8M_SB_RING);
      s_sb_pending--;
    }
    if (s_wf.num_blocks >= FT8M_DECODE_BLOCKS) {
      s_capturing     = false;
      s_heap_size     = 0;
      s_search_cursor = -10;
      s_state         = FT8_ST_SEARCH;
    }
    break;

  case FT8_ST_SEARCH:
    if (boundary) {           /* overran into the next slot — keep cadence */
      dbg_ft8_aborts++;
      ft8_start_capture();
      break;
    }
    /* One time_offset per slice ≈ 1 ms; full search = 30 slices */
    while (s_search_cursor < 20 && (DWT->CYCCNT - t0) < budget_cyc) {
      ftx_find_candidates_range(&s_wf, FT8M_MAX_CAND, s_cand, FT8M_MIN_SCORE,
                                s_search_cursor, s_search_cursor + 1, &s_heap_size);
      s_search_cursor++;
    }
    if (s_search_cursor >= 20) {
      ftx_candidates_sort(s_cand, s_heap_size);
      s_last_ncand = (uint16_t)s_heap_size;
      s_last_best  = (s_heap_size > 0) ? s_cand[0].score : 0;
      dbg_ft8_last_ncand = s_last_ncand;
      dbg_ft8_last_best  = s_last_best;
      s_cand_idx = 0;
      s_num_dec  = 0U;
      s_state    = FT8_ST_DECODE;
    }
    break;

  case FT8_ST_DECODE:
    if (boundary) {
      dbg_ft8_aborts++;
      ft8_finalize_slot();    /* keep what we already decoded */
      ft8_start_capture();
      break;
    }
    while (s_cand_idx < s_heap_size && (DWT->CYCCNT - t0) < budget_cyc) {
      const ftx_candidate_t *c = &s_cand[s_cand_idx++];
      ftx_message_t msg;
      ftx_decode_status_t st;
      if (ftx_decode_candidate(&s_wf, c, FT8M_LDPC_ITERS, &msg, &st)) {
        dbg_ft8_crc_pass++;          /* incl. duplicates from nearby candidates */
        ft8_store_decode(c, &msg);
      }
    }
    if (s_cand_idx >= s_heap_size)
      ft8_finalize_slot();
    break;

  default:
    s_state = FT8_ST_WAIT;
    break;
  }

  /* Rotate the INFO strip through this slot's decodes */
  if (s_strip_ui && s_num_dec > 1U && s_state != FT8_ST_DECODE) {
    uint32_t now = HAL_GetTick();
    if ((now - s_ui_tick_ms) >= FT8M_UI_ROTATE_MS) {
      s_ui_tick_ms = now;
      s_ui_idx = (uint8_t)((s_ui_idx + 1U) % s_num_dec);
      ft8_ui_show(s_ui_idx);
    }
  }
}

/* ── App-facing accessors (ft8_app.c) ────────────────────────────────────── */

void FT8_GetStatus(FT8_Status_t *st)
{
  st->state        = (FT8_State_t)s_state;
  st->cycle_ms     = (s_state == FT8_ST_IDLE) ? 0 : s_prev_cyc_ms;
  st->num_dec      = s_num_dec;
  st->slot_seq     = s_slot_seq;
  st->dt_offset_ms = s_dt_offset_ms;
  st->num_blocks   = (uint8_t)s_wf.num_blocks;
  st->num_cand     = s_last_ncand;
  st->best_score   = s_last_best;
  st->init_ok      = s_init_ok;
}

const FT8_Decode_t *FT8_GetDecode(uint8_t idx)
{
  if (idx >= s_num_dec) return NULL;
  return &s_dec[idx].pub;
}

void FT8_SetStripUI(bool on)
{
  s_strip_ui = on;
}

int32_t FT8_GetCycleMs(void)
{
  return ft8_cycle_ms();
}

/* ══════════════════════════════════════════════════════════════════════════
 *  TX — FT8 beacon synthesis (Phase 2)
 *
 *  pack77 (ftx_message_encode) → LDPC/Costas tone sequence (ft8_encode) →
 *  phase-continuous 8-GFSK at 48 kHz, injected into DSP_ProcessTX in place
 *  of the mic/USB source (pre-gain: digi drive, tx_power, PA foldback and
 *  ALC all apply exactly as for WSJT-X audio through the DIGU linear path).
 *
 *  GFSK frequency smoothing (BT=2, same pulse as ft8_lib's synth_gfsk) uses
 *  a 385-entry LUT + linear interpolation instead of per-sample erff — the
 *  pulse is a partition of unity, so the instantaneous frequency always
 *  needs only the 3 symbols overlapping the current sample.
 * ══════════════════════════════════════════════════════════════════════════ */

#define FT8M_TX_NN        79U
#define FT8M_TX_SPSYM     7680U           /* 48 kHz × 0.16 s                 */
#define FT8M_TX_F0_HZ     1500.0f         /* audio offset of tone 0          */
#define FT8M_TX_START_MS  500             /* nominal in-slot signal start    */
#define FT8M_TX_RAMP      960             /* 20 ms raised-cosine level ramp  */
#define FT8M_TX_PULSE_RES 128             /* LUT entries per symbol          */
#define FT8M_TX_PULSE_LEN (3 * FT8M_TX_PULSE_RES)

bool g_ft8_tx_tone = false;

static uint8_t  s_tx_tones[FT8M_TX_NN];
static bool     s_tx_msg_ok;
static int32_t  s_tx_n;                   /* samples since slot start        */
static uint32_t s_tx_phase;               /* Q32 phase accumulator           */
static float    s_tx_pulse[FT8M_TX_PULSE_LEN + 1];
static bool     s_tx_pulse_ready;

static void ft8_tx_pulse_init(void)
{
  /* pulse(t) = (erf(K·BT·(t+0.5)) − erf(K·BT·(t−0.5))) / 2,  t ∈ [−1.5,1.5]
   * K = π·√(2/ln 2), BT = 2.0 (FT8) — matches ft8_lib gfsk_pulse() */
  const float kbt = 5.336446f * 2.0f;
  for (int i = 0; i <= FT8M_TX_PULSE_LEN; i++) {
    float t = ((float)i / (float)FT8M_TX_PULSE_RES) - 1.5f;
    s_tx_pulse[i] = 0.5f * (erff(kbt * (t + 0.5f)) - erff(kbt * (t - 0.5f)));
  }
  s_tx_pulse_ready = true;
}

bool FT8_TxSetMessage(const char *text)
{
  ftx_message_t msg;
  ftx_message_init(&msg);
  if (ftx_message_encode(&msg, NULL, text) != FTX_MESSAGE_RC_OK) {
    s_tx_msg_ok = false;
    return false;
  }
  ft8_encode(msg.payload, s_tx_tones);
  if (!s_tx_pulse_ready) ft8_tx_pulse_init();
  s_tx_msg_ok = true;
  return true;
}

void FT8_TxStart(int32_t cycle_ms_now)
{
  if (!s_tx_msg_ok) return;
  if (cycle_ms_now < 0) cycle_ms_now = 0;
  s_tx_n     = cycle_ms_now * 48;   /* tone begins at the +0.5 s grid point  */
  s_tx_phase = 0U;
  g_ft8_tx_tone = true;
}

void FT8_TxStop(void)
{
  g_ft8_tx_tone = false;
}

bool FT8_TxDone(void)
{
  return s_tx_n >= (int32_t)(FT8M_TX_START_MS * 48 + FT8M_TX_NN * FT8M_TX_SPSYM);
}

float FT8_TxSample(void)
{
  int32_t n = s_tx_n++;
  int32_t n_sig = n - (FT8M_TX_START_MS * 48);
  const int32_t total = (int32_t)(FT8M_TX_NN * FT8M_TX_SPSYM);
  if (!s_tx_msg_ok || n_sig < 0 || n_sig >= total) return 0.0f;

  /* Instantaneous frequency: f0 + 6.25 × Σ tone[s]·pulse(u − s − 0.5) over
   * the 3 overlapping symbols; first/last tone held at the edges (matches
   * ft8_lib's dphi boundary extension). */
  float u  = (float)n_sig * (1.0f / (float)FT8M_TX_SPSYM);
  int   s0 = (int)u;
  float f  = FT8M_TX_F0_HZ;
  for (int s = s0 - 1; s <= s0 + 1; s++) {
    int sc = (s < 0) ? 0 : ((s >= (int)FT8M_TX_NN) ? (int)FT8M_TX_NN - 1 : s);
    float x = (u - (float)s + 1.0f) * (float)FT8M_TX_PULSE_RES; /* (u−s−0.5+1.5)·res */
    if (x < 0.0f || x >= (float)FT8M_TX_PULSE_LEN) continue;
    int   xi = (int)x;
    float xf = x - (float)xi;
    float p  = s_tx_pulse[xi] + xf * (s_tx_pulse[xi + 1] - s_tx_pulse[xi]);
    f += 6.25f * (float)s_tx_tones[sc] * p;
  }

  s_tx_phase += (uint32_t)(f * 89478.485f);   /* 2^32 / 48000 per Hz         */
  float ph = (float)s_tx_phase * (6.28318530718f / 4294967296.0f);

  float a = 0.9f;   /* pre-gain amplitude; digi drive scales downstream      */
  if (n_sig < FT8M_TX_RAMP)
    a *= 0.5f * (1.0f - cosf(3.14159265f * (float)n_sig / (float)FT8M_TX_RAMP));
  else if (n_sig > total - FT8M_TX_RAMP)
    a *= 0.5f * (1.0f - cosf(3.14159265f * (float)(total - n_sig) / (float)FT8M_TX_RAMP));

  return a * sinf(ph);
}
