/* USER CODE BEGIN Header */
/**
  * @file atu.c
  * @brief Automatic antenna tuner — L-network over a private 2-chip 74AHC595
  *        chain.  See atu.h for the pin map, bit layout and search strategy.
  */
/* USER CODE END Header */
#include "atu.h"
#include "csdr_app.h"
#include "sdr_dsp.h"
#include "fsdr_analog.h"
#include "pa_protect.h"
#include "w25q.h"
#include <stdio.h>
#include <string.h>

/* USER CODE BEGIN 0 */

Atu_State_t g_atu;

/* ── Search plan ─────────────────────────────────────────────────────────
 * Run once per C-side relay position (C at antenna end, then C at radio end):
 *   pass 0  coarse 2-D grid over (L, C)   8 x 8 = 64 pts
 *   pass 1  refine L   +/-16 step 4        9 pts
 *   pass 2  refine C   +/-16 step 4        9 pts
 *   pass 3  refine L   +/- 4 step 1        9 pts
 *   pass 4  refine C   +/- 4 step 1        9 pts
 * = 100 measurements per half, 200 for both, plus the initial bypass probe.
 *
 * The coarse stage MUST be two-dimensional.  A purely resistive mismatch
 * (the commonest real case) sits at a saddle point for coordinate descent:
 * from L=0,C=0 neither adding L alone nor adding C alone lowers the SWR —
 * only the combination does — so sweeping L and then C independently finds
 * nothing and the tuner falls back to bypass.  Simulation over 135
 * load/frequency combinations put that failure at 36% of cases.
 *
 * The grid is geometric, not uniform.  A match at 1.8 MHz needs more than
 * ten times the L and C of the same match at 29 MHz, so the useful part of
 * the ladder shifts by a decade across the HF range; a uniform grid spends
 * most of its points at the top of the ladder and samples nothing near the
 * bottom, where every high-band solution lives (12.5 ohm at 14 MHz wants
 * L=5, which a uniform step-18 grid never visits).  Same 64 points, same
 * run time, but 36% POOR drops to 5% and mean SWR 2.65 -> 1.30.  The
 * residual failures are all extreme loads on 160 m, where the ladder simply
 * runs out of L and C — a hardware limit, not a search limit. */
#define ATU_GRID_N        8U
static const uint8_t k_grid[ATU_GRID_N] = { 0U, 2U, 4U, 8U, 16U, 32U, 64U, 112U };

enum { ATU_PK_GRID = 0, ATU_PK_L, ATU_PK_C };

typedef struct {
  uint8_t kind;   /* ATU_PK_*                                              */
  uint8_t len;    /* measurement points in this pass                       */
  uint8_t span;   /* 1-D passes: +/- offset around the pass base           */
  uint8_t step;   /* 1-D passes: offset increment                          */
} AtuPass_t;

static const AtuPass_t k_passes[] = {
  { ATU_PK_GRID, ATU_GRID_N * ATU_GRID_N, 0U, 0U },
  { ATU_PK_L,    9U, 16U, 4U },
  { ATU_PK_C,    9U, 16U, 4U },
  { ATU_PK_L,    9U,  4U, 1U },
  { ATU_PK_C,    9U,  4U, 1U },
};
#define ATU_PASS_COUNT   (sizeof(k_passes) / sizeof(k_passes[0]))
#define ATU_PASS_TOTAL   100U                     /* 64 + 9 + 9 + 9 + 9     */
#define ATU_STEP_TOTAL   (2U * ATU_PASS_TOTAL)    /* both C-side positions  */

/* Consecutive carrier-less measurements tolerated before giving up. */
#define ATU_MAX_BAD_READS  5U

typedef enum {
  AST_IDLE = 0,
  AST_TX_SETTLE,   /* carrier keyed, waiting for it to stabilise */
  AST_SETTLE,      /* relays written, waiting ATU_SETTLE_MS      */
  AST_DONE,        /* finished; result latched for ATU_PollResult */
} atu_st_t;

static struct {
  uint8_t  state;
  uint8_t  half;         /* 0 = C at antenna end, 1 = C at radio end      */
  uint8_t  pass;         /* index into k_passes                           */
  uint8_t  idx;          /* point within the current pass                 */
  uint8_t  cur_l, cur_c; /* working point for this half                   */
  uint8_t  pass_base;    /* value the 1-D passes centre on                */
  uint16_t pass_best_swr;
  /* Winner of the current pass, kept as the (L,C) PAIR: the coarse pass
   * varies both at once, so a single "best value" cannot describe it.  For a
   * 1-D pass the other coordinate is constant, so storing both is still
   * exactly right and keeps the commit step uniform. */
  uint8_t  pass_best_l, pass_best_c;
  /* best across everything, including the bypass probe */
  uint16_t best_swr;
  uint8_t  best_l, best_c;
  bool     best_c_tx;
  bool     best_bypass;
  bool     probing_bypass;  /* the very first measurement of a cycle      */
  uint8_t  bad_reads;       /* consecutive measurements with no carrier   */
  uint32_t t_start;
  uint32_t t_step;
  uint8_t  result;       /* 0 none pending, 1 success, 2 failed           */
} s_tune;

/* Solution memory, mirrored in RAM (852 B) and flushed on the debounced
 * settings-save path — a tune must never stall audio on a flash erase. */
static AtuMemSlot_t s_mem[ATU_MEM_SLOTS];
static bool         s_mem_dirty;

/* Deferred relay write: a frequency change (or a menu bypass toggle) while
 * transmitting must not move relays under RF, so the request is parked here
 * and applied at the next RX.  s_defer_bypass distinguishes the two: a parked
 * "bypass ON" must not silently turn into a memory recall. */
static bool     s_recall_pending;
static bool     s_defer_bypass;
static uint32_t s_recall_freq;

/* Shadow of the last word actually latched, so redundant writes are skipped
 * (every skipped write is one less relay-coil transient). */
static uint16_t s_word_shadow;
static bool     s_word_valid;

/* ── 595 chain ───────────────────────────────────────────────────────────
 * Shift 16 bits MSB-first, then one RCLK pulse commits both chips at once.
 * Unlike the BPF mux (bpf_lpf.c) there is NO break-before-make word here:
 * the L/C ladder relays are independent series/shunt elements, not a mux
 * whose enables must never overlap, so an atomic single-latch update is
 * both correct and kinder to the relay coils. */
static void atu595_shift(uint16_t word)
{
  for (int8_t i = 15; i >= 0; i--) {
    HAL_GPIO_WritePin(ATU_SER_GPIO_Port, ATU_SER_Pin,
                      (word & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ATU_SRCLK_GPIO_Port, ATU_SRCLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ATU_SRCLK_GPIO_Port, ATU_SRCLK_Pin, GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(ATU_RCLK_GPIO_Port, ATU_RCLK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ATU_RCLK_GPIO_Port, ATU_RCLK_Pin, GPIO_PIN_RESET);
}

static uint16_t atu_word(uint8_t l, uint8_t c, bool c_tx_side, bool bypass)
{
  uint16_t w = (uint16_t)((l & ATU_LC_MAX) << ATU_BIT_L_SHIFT)
             | (uint16_t)((c & ATU_LC_MAX) << ATU_BIT_C_SHIFT);
  if (c_tx_side)   w |= (uint16_t)(1U << ATU_BIT_C_TX_SIDE);
  if (bypass) w |= (uint16_t)(1U << ATU_BIT_BYPASS);
  return w;
}

/* Latch a relay configuration and mirror it into g_atu for the UI. */
static void atu_apply(uint8_t l, uint8_t c, bool c_tx_side, bool bypass)
{
  uint16_t w = atu_word(l, c, c_tx_side, bypass);
  if (s_word_valid && w == s_word_shadow) return;   /* already latched */
  atu595_shift(w);
  s_word_shadow = w;
  s_word_valid  = true;
  g_atu.l_val  = (uint8_t)(l & ATU_LC_MAX);
  g_atu.c_val  = (uint8_t)(c & ATU_LC_MAX);
  g_atu.c_tx_side   = c_tx_side;
  g_atu.bypass = bypass;
}

/* ── Solution memory ─────────────────────────────────────────────────────
 * Bucket index for a frequency; ATU_MEM_SLOTS is out-of-range == "no slot",
 * which is what every frequency below 1.8 MHz or above 30.2 MHz gets. */
static uint16_t atu_mem_idx(uint32_t freq_hz)
{
  if (freq_hz < ATU_MEM_FREQ_MIN) return ATU_MEM_SLOTS;
  uint32_t i = (freq_hz - ATU_MEM_FREQ_MIN) / ATU_MEM_BUCKET_HZ;
  return (i < ATU_MEM_SLOTS) ? (uint16_t)i : ATU_MEM_SLOTS;
}

static void atu_mem_store(uint32_t freq_hz, uint8_t l, uint8_t c,
                          bool c_tx_side, bool bypass)
{
  uint16_t i = atu_mem_idx(freq_hz);
  if (i >= ATU_MEM_SLOTS) return;
  AtuMemSlot_t n;
  n.l_val = (uint8_t)(l & ATU_LC_MAX);
  n.c_val = (uint8_t)(c & ATU_LC_MAX);
  n.flags = (uint8_t)(ATU_MEM_F_VALID
                      | (c_tx_side   ? ATU_MEM_F_C_TX    : 0U)
                      | (bypass ? ATU_MEM_F_BYPASS : 0U));
  if (n.l_val == s_mem[i].l_val && n.c_val == s_mem[i].c_val
      && n.flags == s_mem[i].flags)
    return;                       /* unchanged — no flash write needed */
  s_mem[i]    = n;
  s_mem_dirty = true;
}

/* Recall the stored solution for freq_hz, or bypass when there is none.
 * Caller must guarantee RX (no relay switching under RF). */
static void atu_mem_recall(uint32_t freq_hz)
{
  if (!g_atu.enabled) { atu_apply(0U, 0U, false, true); return; }
  uint16_t i = atu_mem_idx(freq_hz);
  if (i < ATU_MEM_SLOTS && (s_mem[i].flags & ATU_MEM_F_VALID)) {
    atu_apply(s_mem[i].l_val, s_mem[i].c_val,
              (s_mem[i].flags & ATU_MEM_F_C_TX)    != 0U,
              (s_mem[i].flags & ATU_MEM_F_BYPASS) != 0U);
  } else {
    atu_apply(0U, 0U, false, true);   /* nothing known here — stay out of
                                         the RF path rather than guess */
  }
}

/* ── SWR measurement ─────────────────────────────────────────────────────
 * Two raw ADC pairs averaged, then the same per-band swr_scale correction
 * CSDR_Loop applies to g_analog.swr_x100, so the tuner minimises exactly the
 * figure the operator sees on the meter.
 *
 * The forward-level guard matters: Analog_Calc_SWR_x100() returns 100 (a
 * perfect 1.00) whenever the forward voltage is under its own noise gate, so
 * a dropped carrier would otherwise look like a flawless match and win the
 * search outright. */
static bool atu_measure(uint16_t *swr_out)
{
  uint32_t vf = (uint32_t)Analog_ReadSWR_For_Raw();
  uint32_t vr = (uint32_t)Analog_ReadSWR_Ref_Raw();
  vf += (uint32_t)Analog_ReadSWR_For_Raw();
  vr += (uint32_t)Analog_ReadSWR_Ref_Raw();
  vf >>= 1; vr >>= 1;
  if (vf < ATU_MIN_FWD_RAW) return false;     /* no usable carrier */

  uint16_t s = Analog_Calc_SWR_x100((uint16_t)vf, (uint16_t)vr);
  int16_t sc = g_band_cal[g_sdr.band_idx].swr_scale;
  if (sc <  50) sc =  50;
  if (sc > 200) sc = 200;
  uint32_t scaled = ((uint32_t)s * (uint32_t)(uint16_t)sc + 50U) / 100U;
  *swr_out = (uint16_t)(scaled > 9999U ? 9999U : scaled);
  return true;
}

/* Value swept at (pass, idx). */
/* Write the relays for the current (half, pass, idx) cursor.
 * The coarse pass walks the geometric grid in row-major order (row = L,
 * column = C); the 1-D passes step around the base captured at pass start. */
static void atu_apply_cursor(void)
{
  const AtuPass_t *p = &k_passes[s_tune.pass];
  if (p->kind == ATU_PK_GRID) {
    s_tune.cur_l = k_grid[s_tune.idx / ATU_GRID_N];
    s_tune.cur_c = k_grid[s_tune.idx % ATU_GRID_N];
  } else {
    int16_t v = (int16_t)s_tune.pass_base - (int16_t)p->span
              + (int16_t)s_tune.idx * (int16_t)p->step;
    if (v < 0)                    v = 0;
    if (v > (int16_t)ATU_LC_MAX)  v = (int16_t)ATU_LC_MAX;
    if (p->kind == ATU_PK_L) s_tune.cur_l = (uint8_t)v;
    else                     s_tune.cur_c = (uint8_t)v;
  }
  atu_apply(s_tune.cur_l, s_tune.cur_c, (s_tune.half != 0U), false);
}

static void atu_pass_begin(void)
{
  s_tune.idx           = 0U;
  s_tune.pass_base     = (k_passes[s_tune.pass].kind == ATU_PK_C)
                         ? s_tune.cur_c : s_tune.cur_l;
  s_tune.pass_best_swr = 0xFFFFU;
  s_tune.pass_best_l   = s_tune.cur_l;
  s_tune.pass_best_c   = s_tune.cur_c;
}

static void atu_half_begin(void)
{
  s_tune.cur_l = 0U;
  s_tune.cur_c = 0U;
  s_tune.pass  = 0U;
  atu_pass_begin();
}

static void atu_progress(void)
{
  uint16_t done = (uint16_t)s_tune.half * ATU_PASS_TOTAL + s_tune.idx;
  for (uint8_t p = 0U; p < s_tune.pass && p < ATU_PASS_COUNT; p++)
    done += k_passes[p].len;
  uint16_t pct = (uint16_t)((uint32_t)done * 100U / ATU_STEP_TOTAL);
  g_atu.progress = (uint8_t)(pct > 100U ? 100U : pct);
}

/* Drop the carrier raised by ATU_StartTune and release tune_mode. */
static void atu_unkey(void)
{
  CSDR_RequestTuneCarrier(false);
}

/* Land on the best configuration found and stop. */
static void atu_finish(bool ok)
{
  if (ok) {
    atu_apply(s_tune.best_l, s_tune.best_c,
              s_tune.best_c_tx, s_tune.best_bypass);
    g_atu.swr_x100 = s_tune.best_swr;
    g_atu.status   = (s_tune.best_swr <= ATU_SWR_OK_X100)
                     ? (uint8_t)ATU_STATUS_OK : (uint8_t)ATU_STATUS_POOR;
    /* Remember even a POOR result: it is still the best known match for this
     * bucket, and storing it stops the radio re-tuning on every band visit. */
    atu_mem_store(g_sdr.freq_hz, s_tune.best_l, s_tune.best_c,
                  s_tune.best_c_tx, s_tune.best_bypass);
    s_tune.result = 1U;
  } else {
    /* Aborted: fall back to bypass rather than leaving whatever half-searched
     * L/C happened to be latched in the antenna path. */
    atu_apply(0U, 0U, false, true);
    g_atu.status  = (uint8_t)ATU_STATUS_FAILED;
    s_tune.result = 2U;
  }
  g_atu.progress = 100U;
  s_tune.state   = AST_DONE;
  atu_unkey();
}

/* USER CODE END 0 */

void ATU_Init(void)
{
  /* USER CODE BEGIN ATU_Init_0 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Park OE HIGH (595 outputs Hi-Z) BEFORE the pin becomes an output, so the
   * switch to push-pull cannot momentarily pull OE low and expose whatever
   * random word the 595s powered up with.  The board's pull-up has been
   * holding OE high since power-on; this hands over without a gap. */
  HAL_GPIO_WritePin(ATU_OE_GPIO_Port, ATU_OE_Pin, GPIO_PIN_SET);
  GPIO_InitTypeDef gi = {
    .Pin   = ATU_OE_Pin,
    .Mode  = GPIO_MODE_OUTPUT_PP,
    .Pull  = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(ATU_OE_GPIO_Port, &gi);

  HAL_GPIO_WritePin(ATU_SER_GPIO_Port,   ATU_SER_Pin,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ATU_SRCLK_GPIO_Port, ATU_SRCLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ATU_RCLK_GPIO_Port,  ATU_RCLK_Pin,  GPIO_PIN_RESET);
  gi.Pin = ATU_SER_Pin | ATU_SRCLK_Pin | ATU_RCLK_Pin;
  HAL_GPIO_Init(ATU_SER_GPIO_Port, &gi);   /* SER/SRCLK/RCLK share GPIOD */

  /* Shift the bypass word in while the outputs are still Hi-Z, then enable
   * them — the relays go straight from "all released" to "bypass". */
  s_word_valid = false;
  atu_apply(0U, 0U, false, true);
  HAL_GPIO_WritePin(ATU_OE_GPIO_Port, ATU_OE_Pin, GPIO_PIN_RESET);

  s_tune.state     = AST_IDLE;
  s_tune.result    = 0U;
  g_atu.status     = (uint8_t)ATU_STATUS_IDLE;
  g_atu.swr_x100   = 100U;
  g_atu.progress   = 0U;
  s_recall_pending = false;
  /* USER CODE END ATU_Init_0 */
}

void ATU_SetEnabled(bool on)
{
  /* USER CODE BEGIN ATU_SetEnabled_0 */
  if (ATU_IsTuning()) return;
  g_atu.enabled = on;
  if (!on) {
    atu_apply(0U, 0U, false, true);        /* out of the RF path entirely */
    g_atu.status = (uint8_t)ATU_STATUS_IDLE;
  } else if (!g_sdr.tx_mode) {
    atu_mem_recall(g_sdr.freq_hz);
  }
  /* USER CODE END ATU_SetEnabled_0 */
}

void ATU_SetAuto(bool on)
{
  /* USER CODE BEGIN ATU_SetAuto_0 */
  g_atu.auto_tune = on;
  /* USER CODE END ATU_SetAuto_0 */
}

void ATU_SetBypass(bool on)
{
  /* USER CODE BEGIN ATU_SetBypass_0 */
  if (ATU_IsTuning()) return;
  if (g_sdr.tx_mode) {
    /* Never switch relays under RF outside a tune cycle — defer to RX,
     * preserving which of the two actions was actually asked for. */
    s_recall_pending = true;
    s_defer_bypass   = on;
    s_recall_freq    = g_sdr.freq_hz;
    return;
  }
  if (on) atu_apply(0U, 0U, false, true);
  else    atu_mem_recall(g_sdr.freq_hz);
  /* USER CODE END ATU_SetBypass_0 */
}

bool ATU_IsTuning(void)
{
  /* USER CODE BEGIN ATU_IsTuning_0 */
  return (s_tune.state == AST_TX_SETTLE) || (s_tune.state == AST_SETTLE);
  /* USER CODE END ATU_IsTuning_0 */
}

bool ATU_StartTune(void)
{
  /* USER CODE BEGIN ATU_StartTune_0 */
  if (!g_atu.enabled)             return false;
  if (ATU_IsTuning())             return false;
  if (g_sdr.pa_watts == 0U)       return false;   /* no PA — no carrier */
  if (!PA_Protect_IsTxAllowed())  return false;
  /* Already transmitting (PTT, keyer, CAT): refuse BEFORE touching a relay.
   * The bypass probe below would otherwise hot-switch the network out of a
   * live RF path, and CSDR_RequestTuneCarrier could not key tune_mode over an
   * existing transmission anyway. */
  if (g_sdr.tx_mode)              return false;

  memset(&s_tune, 0, sizeof(s_tune));
  s_tune.best_swr       = 0xFFFFU;
  s_tune.best_bypass    = true;    /* bypass is the fallback until beaten */
  s_tune.probing_bypass = true;
  s_tune.t_start        = HAL_GetTick();
  s_tune.t_step         = s_tune.t_start;
  s_tune.state          = AST_TX_SETTLE;
  g_atu.status          = (uint8_t)ATU_STATUS_TUNING;
  g_atu.progress        = 0U;

  /* Probe bypass first, so an antenna that is already flat ends the cycle
   * without inserting the network's loss. */
  atu_apply(0U, 0U, false, true);

  /* Key a low-power carrier through the same path the TUNE button uses:
   * tune_mode caps drive at TUNE_POWER_PCT and tells PA_Protect to ignore
   * SWR warn/trip (pa_protect.c) — a bad match is the point of tuning.
   * Overcurrent and thermal protection stay fully armed. */
  CSDR_RequestTuneCarrier(true);
  if (!g_sdr.tx_mode) {           /* refused (already transmitting) */
    s_tune.state = AST_IDLE;
    g_atu.status = (uint8_t)ATU_STATUS_IDLE;
    return false;
  }
  return true;
  /* USER CODE END ATU_StartTune_0 */
}

void ATU_AbortTune(void)
{
  /* USER CODE BEGIN ATU_AbortTune_0 */
  if (!ATU_IsTuning()) return;
  atu_finish(false);
  /* USER CODE END ATU_AbortTune_0 */
}

void ATU_Poll(void)
{
  /* USER CODE BEGIN ATU_Poll_0 */
  /* Deferred recall from a mid-TX frequency change — apply now that the
   * relays are no longer carrying RF. */
  if (s_recall_pending && !g_sdr.tx_mode && !ATU_IsTuning()) {
    s_recall_pending = false;
    if (s_defer_bypass) { s_defer_bypass = false; atu_apply(0U, 0U, false, true); }
    else                { atu_mem_recall(s_recall_freq); }
  }

  if (!ATU_IsTuning()) return;

  uint32_t now = HAL_GetTick();

  /* Survival conditions, checked in every state: something else dropped TX
   * (PTT released, PA overcurrent, power-down), protection latched, the ATU
   * was disabled from the menu mid-cycle, or the cycle overran. */
  if (!g_sdr.tx_mode || !g_atu.enabled || !PA_Protect_IsTxAllowed()
      || (now - s_tune.t_start) > ATU_TIMEOUT_MS) {
    atu_finish(false);
    return;
  }

  if (s_tune.state == AST_TX_SETTLE) {
    if ((now - s_tune.t_step) < ATU_TX_SETTLE_MS) return;
    s_tune.t_step = now;
    s_tune.state  = AST_SETTLE;
    return;
  }

  /* AST_SETTLE: relays have been holding their position for ATU_SETTLE_MS */
  if ((now - s_tune.t_step) < ATU_SETTLE_MS) return;
  s_tune.t_step = now;

  uint16_t swr;
  if (!atu_measure(&swr)) {
    /* No usable forward power this tick.  A single dropout is not fatal — the
     * Power ALC corrector and PA foldback both modulate drive, and a relay
     * transient can land between the two ADC pairs — so ride out a few in a
     * row and only give up once the carrier is clearly gone. */
    if (++s_tune.bad_reads < ATU_MAX_BAD_READS) return;
    atu_finish(false);
    return;
  }
  s_tune.bad_reads = 0U;

  if (s_tune.probing_bypass) {
    s_tune.probing_bypass = false;
    s_tune.best_swr    = swr;
    s_tune.best_l      = 0U;
    s_tune.best_c      = 0U;
    s_tune.best_c_tx    = false;
    s_tune.best_bypass = true;
    if (swr <= ATU_SWR_GOOD_X100) {   /* already flat — leave the network out */
      atu_finish(true);
      return;
    }
    atu_half_begin();
    atu_apply_cursor();
    atu_progress();
    return;
  }

  /* Score the point just measured.  The pass winner is recorded as the whole
   * (L,C) pair so the coarse 2-D pass, which moves both at once, commits a
   * point that was actually measured together. */
  if (swr < s_tune.pass_best_swr) {
    s_tune.pass_best_swr = swr;
    s_tune.pass_best_l   = s_tune.cur_l;
    s_tune.pass_best_c   = s_tune.cur_c;
  }
  if (swr < s_tune.best_swr) {
    s_tune.best_swr    = swr;
    s_tune.best_l      = s_tune.cur_l;
    s_tune.best_c      = s_tune.cur_c;
    s_tune.best_c_tx    = (s_tune.half != 0U);
    s_tune.best_bypass = false;
  }

  /* An exact match is as good as the search can get — stop early rather than
   * spend another second confirming it. */
  if (swr <= ATU_SWR_GOOD_X100) {
    atu_finish(true);
    return;
  }

  /* Advance the cursor: point → pass → half → done. */
  s_tune.idx++;
  if (s_tune.idx >= k_passes[s_tune.pass].len) {
    /* Commit this pass's winner into the working point before moving on. */
    s_tune.cur_l = s_tune.pass_best_l;
    s_tune.cur_c = s_tune.pass_best_c;
    s_tune.pass++;
    if (s_tune.pass >= ATU_PASS_COUNT) {
      s_tune.half++;
      if (s_tune.half >= 2U) { atu_finish(true); return; }
      atu_half_begin();
    } else {
      atu_pass_begin();
    }
  }

  atu_apply_cursor();
  atu_progress();
  /* USER CODE END ATU_Poll_0 */
}

uint8_t ATU_PollResult(uint16_t *swr_out)
{
  /* USER CODE BEGIN ATU_PollResult_0 */
  uint8_t r = s_tune.result;
  if (r != 0U) {
    s_tune.result = 0U;
    s_tune.state  = AST_IDLE;
    if (swr_out) *swr_out = g_atu.swr_x100;
  }
  return r;
  /* USER CODE END ATU_PollResult_0 */
}

void ATU_OnFreqChange(uint32_t freq_hz)
{
  /* USER CODE BEGIN ATU_OnFreqChange_0 */
  if (!g_atu.enabled || ATU_IsTuning()) return;
  if (g_sdr.tx_mode) {
    s_recall_pending = true;      /* relays stay put until RX */
    s_defer_bypass   = false;     /* a freq change means recall, not bypass */
    s_recall_freq    = freq_hz;
    return;
  }
  atu_mem_recall(freq_hz);
  /* USER CODE END ATU_OnFreqChange_0 */
}

void ATU_MemLoad(void)
{
  /* USER CODE BEGIN ATU_MemLoad_0 */
  Flash_LoadAtuMem(&g_flash, s_mem);   /* blank/corrupt → empty, not a fault */
  s_mem_dirty = false;
  /* USER CODE END ATU_MemLoad_0 */
}

bool ATU_MemDirty(void)
{
  /* USER CODE BEGIN ATU_MemDirty_0 */
  return s_mem_dirty;
  /* USER CODE END ATU_MemDirty_0 */
}

void ATU_MemFlush(void)
{
  /* USER CODE BEGIN ATU_MemFlush_0 */
  if (!s_mem_dirty) return;
  /* The sector erase blocks for tens to hundreds of ms — refuse while
   * transmitting or tuning and let the caller retry on the next pass. */
  if (g_sdr.tx_mode || ATU_IsTuning()) return;
  if (Flash_SaveAtuMem(&g_flash, s_mem) == HAL_OK) s_mem_dirty = false;
  /* USER CODE END ATU_MemFlush_0 */
}

void ATU_MemClear(void)
{
  /* USER CODE BEGIN ATU_MemClear_0 */
  memset(s_mem, 0, sizeof(s_mem));
  s_mem_dirty = true;
  if (!g_sdr.tx_mode && !ATU_IsTuning() && g_atu.enabled)
    atu_apply(0U, 0U, false, true);
  /* USER CODE END ATU_MemClear_0 */
}

const char *ATU_StatusText(void)
{
  /* USER CODE BEGIN ATU_StatusText_0 */
  static char buf[20];
  switch ((Atu_Status_t)g_atu.status) {
    case ATU_STATUS_TUNING:
      snprintf(buf, sizeof(buf), "ATU TUNING %u%%", (unsigned)g_atu.progress);
      break;
    case ATU_STATUS_OK:
    case ATU_STATUS_POOR:
      /* printf %f is disabled (newlib-nano) — split the fixed-point value. */
      snprintf(buf, sizeof(buf), "ATU %s %u.%02u",
               g_atu.bypass ? "BYP" : "OK",
               (unsigned)(g_atu.swr_x100 / 100U),
               (unsigned)(g_atu.swr_x100 % 100U));
      break;
    case ATU_STATUS_FAILED:
      snprintf(buf, sizeof(buf), "ATU FAILED");
      break;
    default:
      snprintf(buf, sizeof(buf), "ATU %s",
               g_atu.enabled ? (g_atu.bypass ? "BYPASS" : "ON") : "OFF");
      break;
  }
  return buf;
  /* USER CODE END ATU_StatusText_0 */
}
