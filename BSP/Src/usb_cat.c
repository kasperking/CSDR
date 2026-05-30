 /*
 * Features:
 *  - Kenwood TS-2000 CAT protocol (ID019)
 *  - IF frame 37 chars (no P15 shift byte)
 *  - SH/SL real filter bandwidth, FW = SH-SL for SSB/CW, IS real IF-shift
 *  - g_cat exported, CAT_FlushTX exported
 */

#include "usb_cat.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* USB CDC TX + busy query — forward-declared to avoid cross-directory include */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
extern uint8_t Composite_CDC_IsBusy(void);

/* Exported CAT handle */
CAT_Handle_t g_cat;

/* =========================================================
 * Helpers
 * ========================================================= */
static inline uint32_t cat_now_ms(void)
{
    return HAL_GetTick();
}

static char *cat_put_u32(char *p, uint32_t v, uint8_t width)
{
    char tmp[12];
    uint8_t n = 0U;

    if (v == 0U) {
        tmp[n++] = '0';
    } else {
        while (v > 0U && n < sizeof(tmp)) {
            tmp[n++] = (char)('0' + (v % 10U));
            v /= 10U;
        }
    }

    while (n < width) {
        tmp[n++] = '0';
    }

    /* Clamp to exactly width digits — if value has more digits than width,
     * truncate the most-significant excess (takes value mod 10^width).
     * Prevents field overrun in IF/FA/FB when a callback returns garbage. */
    if (n > width) n = width;

    while (n > 0U) {
        *p++ = tmp[--n];
    }
    return p;
}

static uint32_t cat_parse_u(const char *s, uint8_t n)
{
    uint32_t v = 0U;
    for (uint8_t i = 0U; i < n; i++) {
        if (s[i] >= '0' && s[i] <= '9') {
            v = v * 10U + (uint32_t)(s[i] - '0');
        }
    }
    return v;
}

static void cat_copy(char *dst, const char *src)
{
    /* Bounded copy: dst must be CAT_TX_BUF_SIZE bytes. Prevents runaway
     * strcpy if any future call site passes a longer-than-expected string. */
    uint8_t i = 0U;
    while (i < (uint8_t)(CAT_TX_BUF_SIZE - 1U) && src[i] != '\0') {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static inline char cat_vfo_digit(uint8_t vfo)
{
    return (vfo == 1U) ? '1' : '0';
}

static inline uint8_t cat_clamp_vfo(uint8_t vfo)
{
    return (vfo == 1U) ? 1U : 0U;
}

/* =========================================================
 * Mode mapping
 * SDR enum: 0=AM, 1=FM, 2=USB, 3=LSB, 4=CW, 5=DIGU, 6=DIGL
 * ========================================================= */
uint8_t CAT_SDRModeToCat(uint8_t m)
{
    switch (m) {
        case 0U: return CAT_MODE_AM;
        case 1U: return CAT_MODE_FM;
        case 2U: return CAT_MODE_USB;
        case 3U: return CAT_MODE_LSB;
        case 4U: return CAT_MODE_CW;
        /* DIGU/DIGL report as USB/LSB — Kenwood IF/MD is a single ASCII digit;
         * CAT_MODE_DIGU/DIGL (0x0C/0x0D) are non-standard and would corrupt the frame */
        case 5U: return CAT_MODE_USB;
        case 6U: return CAT_MODE_LSB;
        default: return CAT_MODE_USB;
    }
}

uint8_t CAT_CatModeToSDR(uint8_t m)
{
    switch (m) {
        case CAT_MODE_AM:   return 0U;
        case CAT_MODE_FM:   return 1U;
        case CAT_MODE_USB:  return 2U;
        case CAT_MODE_LSB:  return 3U;
        case CAT_MODE_CW:   return 4U;
        case CAT_MODE_CWR:  return 4U;  /* CW-R → same DSP path */
        case CAT_MODE_FSK:  return 5U;  /* FSK (DATA-USB) → DIGU */
        case CAT_MODE_FSKR: return 6U;  /* FSK-R (DATA-LSB) → DIGL */
        default:            return 2U;
    }
}

/* =========================================================
 * TX FIFO — bracketed with 0xCC/0xDD sentinels.
 * If dbg_fifo_guard_pre changes from 0xCCCCCCCC or
 * dbg_fifo_guard_post changes from 0xDDDDDDDD, something has
 * written OOB into the region adjacent to s_tx_fifo in .bss.
 * ========================================================= */
#define CAT_TX_FIFO_SIZE 512U
volatile uint32_t dbg_fifo_guard_pre  = 0U;  /* set to 0xCCCCCCCC in CAT_Init */
static char       s_tx_fifo[CAT_TX_FIFO_SIZE];
volatile uint32_t dbg_fifo_guard_post = 0U;  /* set to 0xDDDDDDDD in CAT_Init */
static uint16_t   s_tx_head = 0U;
static uint16_t   s_tx_tail = 0U;

/* Reentrancy guard — file-scope so CAT_Init can reset it and the debugger
 * can watch it in Live Expressions.  Must always be 0 between Process calls.
 * If it reads 1 while flrig is frozen, a hard fault or exception interrupted
 * CAT_Process before the cleanup line executed. */
volatile uint8_t  dbg_cat_in_process = 0U;

/* CAT diagnostic counters — watch in Live Expressions.
 *  cat_rx_bytes:      total bytes accepted into rx_buf from USB ISR
 *  cat_tx_bytes:      total bytes written to CDC_Transmit_FS
 *  cat_tx_fifo_drop:  characters silently lost when s_tx_fifo was full
 *  cat_parse_calls:   CAT_Process invocations (should tick every ~10ms) */
volatile uint32_t dbg_cat_rx_bytes         = 0U;
volatile uint32_t dbg_cat_tx_bytes         = 0U;
volatile uint32_t dbg_cat_fifo_drop        = 0U;
volatile uint32_t dbg_cat_parse_calls      = 0U;
/* Protocol-level diagnostics — watch in Live Expressions.
 *  dbg_cat_unknown_cmds:     ?; responses sent (unknown opcode or bad format)
 *  dbg_cat_partial_timeouts: partial commands discarded after 200ms idle
 *  dbg_cat_max_cmd_len:      peak parser_len ever seen (overflow risk if → 127)
 *  dbg_cat_parse_latency_us: last CAT_Process wall-time in µs (DWT-based)
 *  dbg_cat_last_cmd[]:       last complete command received (no ';', NUL-term)
 *  dbg_cat_last_resp[]:      last non-empty response enqueued */
volatile uint32_t dbg_cat_unknown_cmds     = 0U;
volatile uint32_t dbg_cat_partial_timeouts = 0U;
volatile uint32_t dbg_cat_max_cmd_len      = 0U;
volatile uint32_t dbg_cat_parse_latency_us = 0U;
char              dbg_cat_last_cmd [CAT_BUF_SIZE]    = {0};
char              dbg_cat_last_resp[CAT_TX_BUF_SIZE] = {0};
/* Last raw command that produced ?; — watch to identify unknown/malformed frames */
char              dbg_last_unknown_cmd[64]            = {0};

/* ── Malformed-frame and blocked-update counters ─────────────────────────────
 *
 *  dbg_cat_malformed_frames: ?; sent because a KNOWN opcode had invalid
 *    parameters (e.g. RA with wrong field count).
 *    Distinct from dbg_cat_unknown_cmds (unrecognised opcode).
 *    Non-zero = host is sending syntactically bad frames → check flrig profile.
 *
 *  dbg_cat_blocked_updates: a CAT SET arrived while the previous SET for the
 *    same dirty flag had not yet been serviced by CSDR_Loop (i.e. two FA/MD/TX/
 *    RA/RIT SETs within one 10 ms gate).
 *    Occasional counts are normal during rapid VFO tuning.
 *    Sustained high counts = polling loop faster than 10 ms CAT gate. */
volatile uint32_t dbg_cat_malformed_frames = 0U;
volatile uint32_t dbg_cat_blocked_updates  = 0U;

/* Extended crash-forensics variables */
char              dbg_last_malformed[64]  = {0}; /*!< known opcode, bad params → ?; */
char              dbg_last_timeout[64]    = {0}; /*!< partial frame discarded by timer */
char              dbg_last_opcode[3]      = {0};
volatile uint8_t  dbg_last_status        = 0U;
volatile uint8_t  dbg_last_cmd_len       = 0U;
volatile uint8_t  dbg_last_resp_len      = 0U;
volatile uint32_t dbg_resp_no_semi       = 0U;
volatile uint32_t dbg_resp_nonprint      = 0U;

/* Forensic ring buffer + per-opcode statistics */
CAT_TxnRing_t  dbg_txn              = {0};
CAT_OpStat_t   dbg_opc[COPI_COUNT]  = {0};

/* Transmit-payload snapshot + null-drop counter */
uint8_t           dbg_last_tx_raw[CAT_TX_RAW_SIZE] = {0};
volatile uint8_t  dbg_last_tx_n                    = 0U;
volatile uint32_t dbg_resp_null_drop               = 0U;

/* Memory guards — bracket the transport-race counters.
 * If dbg_guard_pre changes from 0xAAAAAAAA or dbg_guard_post changes from
 * 0xBBBBBBBB, a buffer adjacent in memory has overflowed into this region.
 * dbg_guard_status: 0=OK, 1=pre corrupted, 2=post corrupted, 3=both. */
volatile uint32_t dbg_guard_pre     = 0U;  /* sentinel set in CAT_Init; 0U keeps it in .bss adjacent to transport counters */

/* Transport-race diagnostics (flrig stress) --------------------------------
 *  dbg_cdc_busy_skips:   FlushTX called while CDC IN transfer in progress;
 *                        out[] NOT filled — safe retry next call.
 *                        Non-zero = proof the IsBusy guard is firing.
 *                        High value = flrig is polling faster than USB drains.
 *  dbg_fifo_high_water:  Peak bytes held in s_tx_fifo simultaneously.
 *                        Approaches 512 → fifo-drop risk under burst polling.
 *  dbg_process_reenters: CAT_Process entered while already running.
 *                        Non-zero = main-loop timing bug (should never fire). */
volatile uint32_t dbg_cdc_busy_skips    = 0U;
volatile uint32_t dbg_fifo_high_water   = 0U;
volatile uint32_t dbg_process_reenters  = 0U;
volatile uint32_t dbg_cat_pending_bytes = 0U;  /*!< live bytes waiting in TX FIFO      */
volatile uint32_t dbg_cdc_stuck_timeout = 0U;  /*!< IsBusy stuck > 10ms — stall event  */

volatile uint32_t dbg_guard_post        = 0U;  /* sentinel set in CAT_Init; 0U keeps it in .bss */
volatile uint32_t dbg_guard_status      = 0U;
volatile uint32_t dbg_guard_tripped_ms  = 0U;  /*!< HAL_GetTick() when guard_status first went non-zero */

/* Live CAT traffic snapshot — updated at transport boundary.
 *  dbg_last_cat_rx: last bytes received from host (before parser), NUL-term.
 *  dbg_last_cat_tx: last string enqueued into TX FIFO, NUL-term.
 *  dbg_rx_count / dbg_tx_count: monotonic counters for each side. */
volatile char     dbg_last_cat_rx[64] = {0};
volatile char     dbg_last_cat_tx[64] = {0};
volatile uint32_t dbg_rx_count        = 0U;
volatile uint32_t dbg_tx_count        = 0U;
volatile uint32_t dbg_cat_rx_null_bytes = 0U;  /*!< NUL bytes filtered by parser — confirms CDC OUT padding source */

/* Opcode enable mask — 0 = minimal safe subset, CAT_OPC_ALL = full handler set.
 * Modify in Live Watch to re-enable handlers one by one during crash isolation. */
volatile uint32_t dbg_cat_opc_mask = 0U;

/* ── Transport serialization & pacing diagnostics ───────────────────────────
 *
 *  Problem identified: CAT_FlushTX drains up to 64 bytes per call.  When
 *  CAT_Process handles multiple commands in one 10ms tick it enqueues multiple
 *  responses; FlushTX then coalesces them into one USB bulk IN packet.  flrig
 *  reads until ';' and leaves trailing frames in the OS serial buffer.  The
 *  next command's response read returns the PREVIOUS leftover frame instead —
 *  parser desynchronization that eventually freezes polling.
 *
 *  Fix: FlushTX now stops at the first ';' (one CAT frame per USB transfer).
 *  Pacing: a configurable minimum gap prevents zero-latency back-to-back bursts.
 *  AI push disable: prevents unsolicited IF frames from polluting the response stream.
 *
 *  dbg_cat_tx_min_gap_ms:  minimum ms between consecutive CDC IN packets (0=off).
 *    Default 2ms — enough to separate consecutive responses without being slow.
 *    Real TS-480 at 9600 baud sends a 14-char response in ~14ms.
 *    Set to 0 to disable pacing and measure raw throughput.
 *  dbg_pacing_skips:       FlushTX calls deferred by the pacing guard.
 *    Data is NOT dropped — it stays in the FIFO and retries on the next tick.
 *    Count growing steadily = pacing gap is too large for the polling rate.
 *  dbg_frames_per_tx:      Number of ';' terminator chars in the LAST USB packet.
 *    Target value after fix: exactly 1.  Any value >1 means the one-frame guard
 *    failed (should never happen after the loop-break-at-semicolon change).
 *  dbg_multi_frame_pkts:   Cumulative count of USB packets that contained >1 CAT frame.
 *    Should be 0 after the fix.  Non-zero proves coalescing is still occurring.
 *  dbg_cat_ai_push_disable: 1 = suppress AI unsolicited IF push regardless of ai_level.
 *    Default 1 (disabled) so unsolicited frames cannot interleave with polling responses.
 *    Set to 0 to re-enable AI push once strict serialization is confirmed stable. */
volatile uint32_t dbg_cat_tx_min_gap_ms   = 2U;
volatile uint32_t dbg_pacing_skips        = 0U;
volatile uint32_t dbg_frames_per_tx       = 0U;
volatile uint32_t dbg_multi_frame_pkts    = 0U;
volatile uint8_t  dbg_cat_ai_push_disable = 1U;

/* BW command tracking — last opcode that triggered a set_bw callback.
 *  Value: ('F'<<8)|'W' = 0x4657 when set by FW; ('S'<<8)|'H' = 0x5348 when set by SH.
 *  0 = no BW SET received since reset.
 *  Use alongside dbg_last_bw_* in csdr_app.c to see the full BW SET trace. */
volatile uint32_t dbg_last_bw_cmd = 0U;

/* Raw CAT parameter from the last BW command, captured BEFORE cat_exec boundary enforcement.
 *  For FW: the 4-digit value as parsed (may be 0..99999 — not yet clamped to 100..9999).
 *  For SH: the 2-digit index as parsed (may be > 11).
 *  Compare with dbg_last_bw_value (post-boundary, in csdr_app.c) to see how much was clipped. */
volatile uint32_t dbg_last_bw_cat_value = 0U;

/* =========================================================
 * FIFO lifecycle snapshot — updated after every enqueue and every flush.
 * These four variables form a consistent snapshot of the last FIFO operation.
 *
 *  dbg_fifo_op:
 *    'E' = cat_tx_enqueue just finished (head advanced)
 *    'F' = CAT_FlushTX sent bytes (tail advanced)
 *    'B' = CAT_FlushTX returned busy (tail unchanged, data still queued)
 *    'X' = consistency error detected (head/tail OOB or depth impossible)
 *
 *  dbg_fifo_head_snap / dbg_fifo_tail_snap:
 *    Value of s_tx_head / s_tx_tail at the moment of the snapshot.
 *    Both must always be in [0, 511]. If either is 512+, that is a corruption event.
 *
 *  dbg_fifo_depth_snap:
 *    Computed depth = (head - tail + 512) % 512 at snapshot time.
 *    Must equal dbg_cat_pending_bytes at the same moment.
 *    If depth == 0 but you expected data: head == tail means FIFO was already drained.
 *
 *  dbg_fifo_state_error:
 *    Incremented whenever a consistency invariant is violated at snapshot time:
 *      - head or tail >= CAT_TX_FIFO_SIZE
 *      - computed depth != dbg_cat_pending_bytes (stale pending counter)
 *      - depth > 511 (impossible for a 512-slot one-slot-sacrificed ring)
 *
 *  dbg_fifo_bytes_last_enq:
 *    Bytes actually written in the most recent cat_tx_enqueue call.
 *    Should equal strlen(enqueued string).  If less: FIFO was full, characters dropped.
 *
 *  dbg_fifo_bytes_last_flush:
 *    Bytes sent to CDC_Transmit_FS in the most recent successful CAT_FlushTX.
 *    Max 64 (one USB FS bulk packet).  If FIFO had more than 64 bytes,
 *    the remainder stays and is sent on the next non-busy tick.
 *
 *  dbg_last_enqueue_ms / dbg_last_flush_ms:
 *    HAL_GetTick() at last enqueue / last successful flush.
 *    If (dbg_last_flush_ms - dbg_last_enqueue_ms) > 50, flush was
 *    delayed many ticks after enqueue — investigate IsBusy or missed ticks.
 *    If dbg_last_enqueue_ms stops advancing while dbg_cat_parse_calls keeps
 *    going: parser is running but producing no responses — look at the last
 *    dbg_txn entry for the command that caused flrig to stop sending.
 * ========================================================= */
volatile uint8_t  dbg_fifo_op               = 0U;
volatile uint16_t dbg_fifo_head_snap        = 0U;
volatile uint16_t dbg_fifo_tail_snap        = 0U;
volatile uint16_t dbg_fifo_depth_snap       = 0U;
volatile uint32_t dbg_fifo_state_error      = 0U;
volatile uint16_t dbg_fifo_bytes_last_enq   = 0U;
volatile uint16_t dbg_fifo_bytes_last_flush = 0U;
volatile uint32_t dbg_last_enqueue_ms       = 0U;
volatile uint32_t dbg_last_flush_ms         = 0U;

/* Module-level parse status — set by cat_exec, read by CAT_Process */
static volatile CAT_ParseStatus_t s_exec_status = CAT_PARSE_OK;

/* FIFO consistency snapshot — called after every enqueue and every flush.
 * Computes depth from head/tail, checks all invariants, writes results to
 * the dbg_fifo_* variables for Live Expressions visibility.
 * op: 'E'=enqueue, 'F'=flush sent, 'B'=flush busy-deferred, 'X'=error. */
static void cat_fifo_snap(char op)
{
    uint16_t h = s_tx_head;
    uint16_t t = s_tx_tail;
    uint8_t  err = 0U;

    if (h >= CAT_TX_FIFO_SIZE || t >= CAT_TX_FIFO_SIZE) {
        err = 1U;
    }

    uint16_t depth = (h >= t) ? (h - t)
                               : (CAT_TX_FIFO_SIZE - t + h);

    /* depth must be 0..511 for a 512-slot ring sacrificing one slot */
    if (depth >= CAT_TX_FIFO_SIZE) {
        err = 1U;
    }

    /* pending_bytes must match what we just computed */
    if (!err && (uint32_t)depth != dbg_cat_pending_bytes) {
        dbg_fifo_state_error++;   /* stale pending counter — bookkeeping diverged */
        /* Correct it immediately so the next check doesn't cascade */
        dbg_cat_pending_bytes = depth;
    }

    if (err) {
        dbg_fifo_state_error++;
        dbg_fifo_op = (uint8_t)'X';
    } else {
        dbg_fifo_op = (uint8_t)op;
    }

    dbg_fifo_head_snap  = h;
    dbg_fifo_tail_snap  = t;
    dbg_fifo_depth_snap = depth;
}

static void cat_tx_enqueue(const char *s)
{
    if (!s || !*s) return;

    /* Sanity: head/tail must be in [0, CAT_TX_FIFO_SIZE-1].
     * If either is out of range (memory corruption), reset the FIFO rather
     * than writing OOB on the next s_tx_fifo[s_tx_head] store. */
    if (s_tx_head >= CAT_TX_FIFO_SIZE || s_tx_tail >= CAT_TX_FIFO_SIZE) {
        s_tx_head = s_tx_tail = 0U;
        dbg_cat_fifo_drop++;
        return;
    }

    /* Capture the response string before it enters the FIFO so it is visible
     * in the debugger even if the FIFO drains before the debugger pauses. */
    {
        const char *src = s;
        uint8_t cap = 0U;
        while (*src && cap < (uint8_t)(sizeof(dbg_last_cat_tx) - 1U)) {
            ((char *)dbg_last_cat_tx)[cap++] = *src++;
        }
        ((char *)dbg_last_cat_tx)[cap] = '\0';
        dbg_tx_count++;
    }

    uint16_t head_before  = s_tx_head;
    uint16_t bytes_added  = 0U;

    while (*s) {
        uint16_t next = (uint16_t)((s_tx_head + 1U) % CAT_TX_FIFO_SIZE);
        if (next == s_tx_tail) {
            dbg_cat_fifo_drop++;   /* FIFO full — character dropped */
            break;
        }
        s_tx_fifo[s_tx_head] = *s++;
        s_tx_head = next;
        bytes_added++;
    }

    /* Cross-check: head must have advanced by exactly bytes_added (mod 512).
     * Any deviation means the write loop left head in a corrupt state. */
    {
        uint16_t expected = (uint16_t)((head_before + bytes_added) % CAT_TX_FIFO_SIZE);
        if (s_tx_head != expected) {
            dbg_fifo_state_error++;
        }
    }

    dbg_fifo_bytes_last_enq = bytes_added;
    dbg_last_enqueue_ms     = cat_now_ms();

    /* Track live depth and peak fill — wrap-safe subtraction */
    uint16_t depth = (s_tx_head >= s_tx_tail)
                   ? (s_tx_head - s_tx_tail)
                   : (CAT_TX_FIFO_SIZE - s_tx_tail + s_tx_head);
    dbg_cat_pending_bytes = depth;
    if ((uint32_t)depth > dbg_fifo_high_water) dbg_fifo_high_water = depth;

    cat_fifo_snap('E');
}

void CAT_SendResponse(const char *resp)
{
    cat_tx_enqueue(resp);
}

/* =========================================================
 * Forensic helpers
 * ========================================================= */

static uint8_t cat_opc_idx(const char *cmd)
{
    if (cmd[0]=='I'&&cmd[1]=='F') return COPI_IF;
    if (cmd[0]=='F'&&cmd[1]=='A') return COPI_FA;
    if (cmd[0]=='F'&&cmd[1]=='B') return COPI_FB;
    if (cmd[0]=='M'&&cmd[1]=='D') return COPI_MD;
    if (cmd[0]=='A'&&cmd[1]=='I') return COPI_AI;
    if (cmd[0]=='I'&&cmd[1]=='D') return COPI_ID;
    if (cmd[0]=='T'&&cmd[1]=='X') return COPI_TX;
    if (cmd[0]=='R'&&cmd[1]=='X') return COPI_RX;
    if (cmd[0]=='P'&&cmd[1]=='S') return COPI_PS;
    if (cmd[0]=='U'&&cmd[1]=='P') return COPI_UP;
    if (cmd[0]=='D'&&cmd[1]=='N') return COPI_DN;
    if (cmd[0]=='A'&&cmd[1]=='G') return COPI_AG;
    if (cmd[0]=='S'&&cmd[1]=='Q') return COPI_SQ;
    return COPI_COUNT;
}

static void cat_validate_resp(const char *resp)
{
    uint8_t n = 0U;
    while (resp[n] && n < CAT_TXN_RESP_MAX) {
        uint8_t ch = (uint8_t)resp[n];
        if (ch < 0x20U || ch > 0x7EU) dbg_resp_nonprint++;
        n++;
    }
    if (n == 0U || resp[n - 1U] != ';') dbg_resp_no_semi++;
}

static void cat_mark_malformed(const char *cmd)
{
    size_t n = strlen(cmd);
    if (n >= sizeof(dbg_last_malformed)) n = sizeof(dbg_last_malformed) - 1U;
    memcpy(dbg_last_malformed, cmd, n);
    memset(dbg_last_malformed + n, 0, sizeof(dbg_last_malformed) - n);
    s_exec_status = CAT_PARSE_ERROR;
}

/* Returns true if the response string is safe to enqueue:
 *   - at least one printable ASCII character before ';'
 *   - no embedded NUL before the semicolon terminator
 *   - all payload bytes in printable ASCII range (0x20-0x7E or ';')
 * A false return increments dbg_resp_null_drop and suppresses the TX. */
static bool cat_resp_guard(const char *resp)
{
    uint8_t i = 0U;
    while (i < (uint8_t)CAT_TX_BUF_SIZE) {
        uint8_t ch = (uint8_t)resp[i];
        if (ch == (uint8_t)';') { return true; }       /* well-formed — accept */
        if (ch == 0U || ch < 0x20U || ch > 0x7EU) {   /* NUL or non-printable — reject */
            dbg_resp_null_drop++;
            return false;
        }
        i++;
    }
    dbg_resp_null_drop++;  /* no ';' found within buffer */
    return false;
}

static void cat_txn_record(const char *cmd, const char *resp,
                           CAT_ParseStatus_t st, uint32_t ms)
{
    CAT_Txn_t *e = &dbg_txn.entries[dbg_txn.next];

    e->ms      = ms;
    e->status  = st;
    e->is_set  = (cmd[2] != '\0') ? 1U : 0U;
    e->opcode[0] = cmd[0]; e->opcode[1] = cmd[1]; e->opcode[2] = '\0';
    e->cmd_len   = (uint8_t)strlen(cmd);
    e->resp_len  = (uint8_t)strlen(resp);

    {
        uint8_t cl = (e->cmd_len  < CAT_TXN_CMD_MAX  - 1U) ? e->cmd_len  : CAT_TXN_CMD_MAX  - 1U;
        uint8_t rl = (e->resp_len < CAT_TXN_RESP_MAX - 1U) ? e->resp_len : CAT_TXN_RESP_MAX - 1U;
        /* Zero entire fixed-size fields first: without this, bytes beyond rl/cl retain
         * data from the previous use of this ring slot, producing apparent null-prefixed
         * strings like "\0L00;" in the debugger when a NORESP entry reuses an old slot. */
        memset(e->cmd,      0, sizeof(e->cmd));
        memset(e->resp,     0, sizeof(e->resp));
        memset(e->resp_raw, 0, sizeof(e->resp_raw));
        memcpy(e->cmd,  cmd,  cl); e->cmd [cl] = '\0';
        memcpy(e->resp, resp, rl); e->resp[rl] = '\0';
        /* resp_raw: same bytes typed as uint8_t — visible as hex in debugger watch */
        memcpy(e->resp_raw, e->resp, sizeof(e->resp_raw));
    }

    dbg_txn.next = (uint8_t)((dbg_txn.next + 1U) % CAT_TXN_RING_SIZE);
    if (dbg_txn.count < 255U) dbg_txn.count++;

    /* Flat forensics */
    dbg_last_opcode[0] = cmd[0]; dbg_last_opcode[1] = cmd[1]; dbg_last_opcode[2] = '\0';
    dbg_last_status    = (uint8_t)st;
    dbg_last_cmd_len   = e->cmd_len;
    dbg_last_resp_len  = e->resp_len;

    /* Opcode statistics */
    uint8_t oi = cat_opc_idx(cmd);
    if (oi < (uint8_t)COPI_COUNT) {
        dbg_opc[oi].rx++;
        if (st == CAT_PARSE_OK || st == CAT_PARSE_NORESP) dbg_opc[oi].ok++;
        else                                               dbg_opc[oi].err++;
    }
}

void CAT_FlushTX(CAT_Handle_t *cat)
{
    (void)cat;

    /* Check ALL memory guards on every flush (runs every main-loop tick ≈1 ms).
     * Bits: transport-counter guards in [1:0], FIFO bracket guards in [3:2]. */
    {
        uint32_t st = 0U;
        if (dbg_guard_pre       != 0xAAAAAAAAU) st |= 1U;
        if (dbg_guard_post      != 0xBBBBBBBBU) st |= 2U;
        if (dbg_fifo_guard_pre  != 0xCCCCCCCCU) st |= 4U;
        if (dbg_fifo_guard_post != 0xDDDDDDDDU) st |= 8U;
        if (st != 0U && dbg_guard_status == 0U) {
            /* First detection — latch the timestamp so we know WHEN it happened */
            dbg_guard_tripped_ms = cat_now_ms();
        }
        dbg_guard_status = st;
    }

    /* If head/tail were corrupted, reset FIFO and bail — transmitting with
     * invalid indices would write OOB on the next enqueue. */
    if (s_tx_head >= CAT_TX_FIFO_SIZE || s_tx_tail >= CAT_TX_FIFO_SIZE) {
        s_tx_head = s_tx_tail = 0U;
        return;
    }

    if (s_tx_head == s_tx_tail) return;

    /* Guard: do NOT start a new transfer while the USB TXFE ISR still holds a
     * reference to out[] from the previous one.  In OTG FIFO mode, EPStartXfer
     * stores the out[] pointer and the TXFE ISR reads it after EPStartXfer
     * returns.  Overwriting out[] before that ISR fires would corrupt the
     * in-flight packet.  This fires legitimately when the FIFO holds more than
     * 64 bytes (multi-packet burst): first call starts the transfer, subsequent
     * calls defer until DataIn clears the flag (≤1 ms on FS USB).
     * Stuck-busy detection: a FS bulk IN transfer must complete in ≤2 ms.
     * If IsBusy persists for > 10 ms, the endpoint has stalled (DataIn will
     * never fire).  Record it so the debugger can see the deadlock. */
    static uint32_t s_busy_since_ms = 0U;
    if (Composite_CDC_IsBusy()) {
        uint32_t now_ms = cat_now_ms();
        if (s_busy_since_ms == 0U) s_busy_since_ms = now_ms;
        if ((now_ms - s_busy_since_ms) > 10U) dbg_cdc_stuck_timeout++;
        dbg_cdc_busy_skips++;
        cat_fifo_snap('B');   /* snapshot: data still in FIFO, deferred */
        return;
    }
    s_busy_since_ms = 0U;   /* clear on every successful not-busy check */

    /* Pacing: enforce minimum inter-frame gap to emulate TS-480 UART cadence.
     * Real TS-480 at 9600 baud: a 14-char "FA00007100000;" takes ~14ms.
     * dbg_cat_tx_min_gap_ms=2 is conservative — prevents zero-gap bursts without
     * slowing normal polling.  dbg_last_flush_ms is maintained below and is 0
     * on first call (guarantees the first packet goes through without delay). */
    if (dbg_cat_tx_min_gap_ms > 0U) {
        uint32_t now_p = cat_now_ms();
        if ((now_p - dbg_last_flush_ms) < dbg_cat_tx_min_gap_ms) {
            dbg_pacing_skips++;
            cat_fifo_snap('B');
            return;
        }
    }

    /* Static: keeps out[] alive until the TXFE ISR copies it to the USB FIFO.
     * A stack-local buffer would be freed before the ISR fires. */
    static uint8_t out[64];
    uint16_t n    = 0U;
    uint16_t tail = s_tx_tail;   /* work with a local copy — don't commit yet */

    /* One-frame-per-USB-transfer: drain exactly up to and including the first ';'.
     * Root cause of flrig freeze: when multiple CAT responses are coalesced into
     * one USB bulk IN packet, flrig reads until ';' for the first response and
     * leaves trailing frames in the OS serial buffer.  The next command-response
     * read picks up the stale leftover instead of the new response → desync.
     * Breaking at ';' guarantees each USB packet contains exactly one CAT frame,
     * matching the per-frame isolation that real UART cadence provides naturally. */
    while (tail != s_tx_head && n < sizeof(out)) {
        char ch = s_tx_fifo[tail];
        out[n++] = (uint8_t)ch;
        tail = (uint16_t)((tail + 1U) % CAT_TX_FIFO_SIZE);
        if (ch == ';') break;   /* one CAT frame per USB transfer */
    }

    /* Only advance the real tail when CDC actually accepts the transfer.
     * On USBD_BUSY the data stays in the FIFO and is retried next call. */
    if (CDC_Transmit_FS(out, n) == 0U) {  /* 0 = USBD_OK */
        s_tx_tail = tail;
        dbg_cat_tx_bytes += n;
        /* Update live pending depth after drain. */
        dbg_cat_pending_bytes = (s_tx_head >= s_tx_tail)
                              ? (s_tx_head - s_tx_tail)
                              : (CAT_TX_FIFO_SIZE - s_tx_tail + s_tx_head);
        /* Snapshot exact transmitted bytes for debugger Memory window. */
        dbg_last_tx_n = (uint8_t)((n < (uint16_t)CAT_TX_RAW_SIZE) ? n : (uint16_t)CAT_TX_RAW_SIZE);
        memcpy(dbg_last_tx_raw, out, dbg_last_tx_n);
        dbg_fifo_bytes_last_flush = n;
        dbg_last_flush_ms         = cat_now_ms();

        /* Per-packet frame validation: count ';' terminators in the transmitted bytes.
         * After the one-frame-per-transfer fix, dbg_frames_per_tx must always be 1.
         * dbg_multi_frame_pkts non-zero after this point = the break-at-semicolon
         * guard failed or a response without ';' was sent (see dbg_resp_null_drop). */
        {
            uint32_t fc = 0U;
            for (uint16_t fi = 0U; fi < n; fi++) {
                if (out[fi] == (uint8_t)';') fc++;
            }
            dbg_frames_per_tx = fc;
            if (fc > 1U) dbg_multi_frame_pkts++;
        }

        cat_fifo_snap('F');   /* snapshot: tail advanced, data sent */
    }
    /* If CDC_Transmit_FS returned non-zero (BUSY returned unexpectedly after
     * IsBusy() said false), data stays in FIFO and tail is NOT advanced.
     * No separate counter needed — dbg_fifo_op stays 'F' from the last
     * successful send and the FIFO depth won't change. */
}

/* =========================================================
 * RX accumulation
 * ========================================================= */
void CAT_Receive(CAT_Handle_t *cat, const uint8_t *data, uint16_t len)
{
    if (!cat) return;

    /* Capture raw RX bytes for debugger visibility before they enter the parser. */
    if (len > 0U) {
        uint16_t cap = (len < (uint16_t)(sizeof(dbg_last_cat_rx) - 1U))
                     ? len : (uint16_t)(sizeof(dbg_last_cat_rx) - 1U);
        memcpy((char *)dbg_last_cat_rx, data, cap);
        ((char *)dbg_last_cat_rx)[cap] = '\0';
        dbg_rx_count++;
    }

    for (uint16_t i = 0U; i < len; i++) {
        if (cat->rx_len < (CAT_BUF_SIZE - 1U)) {
            cat->rx_buf[cat->rx_len++] = (char)data[i];
            dbg_cat_rx_bytes++;
        }
    }
    /* Timestamp last byte for partial-command timeout in CAT_Process */
    if (len > 0U) cat->parser_last_rx_tick = cat_now_ms();
}

/* =========================================================
 * Timing
 * ========================================================= */
static uint32_t s_tx_ready_ms = 0U;

/* =========================================================
 * Init
 * ========================================================= */
void CAT_Init(CAT_Handle_t *cat, const CAT_Callbacks_t *cb)
{
    memset(cat, 0, sizeof(*cat));
    cat->cb = *cb;

    cat->ai_level   = 0U;
    cat->last_freq  = 0U;
    cat->last_mode  = 0xFFU;
    cat->last_tx    = false;
    cat->last_vfo   = 0U;
    cat->last_split = false;
    cat->rit_on     = false;
    cat->if_shift   = 0;
    cat->vfo_b_freq = 7100000UL;
    cat->vfo_b_mode = CAT_MODE_USB;
    cat->vfo_b_bw   = 3000U;
    cat->active_vfo = 0U;
    cat->split_on   = false;

    cat->initialized = true;
    cat->rx_len      = 0U;
    /* parser_cmd, parser_len, parser_last_rx_tick zeroed by memset above */

    memset(s_tx_fifo, 0, sizeof(s_tx_fifo));
    s_tx_head = s_tx_tail = 0U;
    s_tx_ready_ms = 0U;
    dbg_fifo_high_water    = 0U;
    dbg_cat_pending_bytes  = 0U;
    dbg_cdc_busy_skips     = 0U;
    dbg_cdc_stuck_timeout  = 0U;
    /* Transport-counter guards */
    dbg_guard_pre          = 0xAAAAAAAAU;
    dbg_guard_post         = 0xBBBBBBBBU;
    dbg_guard_status       = 0U;
    dbg_guard_tripped_ms   = 0U;
    /* FIFO bracket guards */
    dbg_fifo_guard_pre     = 0xCCCCCCCCU;
    dbg_fifo_guard_post    = 0xDDDDDDDDU;
    /* Reentrancy flag — reset so a fault mid-Process doesn't permanently
     * lock out future calls after a USB reconnect or watchdog reset */
    dbg_cat_in_process     = 0U;
    /* Pacing + frame-validation counters — reset per session */
    dbg_pacing_skips       = 0U;
    dbg_frames_per_tx      = 0U;
    dbg_multi_frame_pkts   = 0U;
    dbg_last_flush_ms      = 0U;

    /* Enable DWT cycle counter for parse-latency measurement.
     * Writing DEMCR/CTRL is safe even without a debugger attached —
     * the write is ignored on production silicon if the debug domain
     * is not powered, and CYCCNT reads 0 in that case. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

/* =========================================================
 * IF builder — TS-2000 format (ID019), 37 data chars + ';' = 38 total
 *
 * Field audit — REAL fields are driven from live SDR state.
 *               FROZEN fields are compile-time constants safe for any host.
 *               NEVER change a FROZEN field to a dynamic value unless the
 *               underlying hardware is actually present and tested.
 *
 *  Byte  Field  Status   Source / notes
 *  ────  ─────  ───────  ─────────────────────────────────────────────────
 *  0-1   "IF"   literal
 *  2-12  P1     REAL     get_freq() / get_vfo_b_freq() — active VFO Hz
 *  13-17 P2     FROZEN   "00000"  — tuning step not exposed in IF
 *  18    P3     REAL     get_rit_hz() sign (+/−)
 *  19-22 P4     REAL     get_rit_hz() magnitude (Hz); 0 when RIT off
 *  23    P5     REAL     cat->rit_on ('0'/'1')
 *  24    P6     FROZEN   '0' — XIT not supported
 *  25-27 P7     FROZEN   "000" — no channel memory
 *  28    P8     REAL     get_tx() — '1' in TX, '0' in RX
 *  29    P9     REAL     CAT mode code (1=LSB 2=USB 3=CW 4=FM 5=AM)
 *  30    P10    REAL     cat->active_vfo ('0'=A '1'=B)
 *  31    P11    FROZEN   '0' — scan not supported
 *  32    P12    REAL     cat->split_on ('0'/'1')
 *  33    P13    FROZEN   '0' — CTCSS/tone not supported
 *  34-35 P14    FROZEN   "00" — CTCSS code not supported
 *  36    P15    FROZEN   '0' — required by Hamlib if_len=37; unused field
 *  37    ';'
 *
 * Total: 37 data chars + ';'. Hamlib kenwood_safe_transaction checks
 * strlen(buf_without_semicolon)==if_len==37; previously this was 36 and
 * caused RIG_EPROTO on every IF query.
 * hamlib cross-check: TX@28, mode@29, VFO@30, split@32.
 *
 * CAT_DBG_HARDCODE_IF — set to 1 to inject a fixed known-good TS-2000
 * reference packet and confirm flrig parses IF correctly before testing
 * dynamic values. Expected string: IF0000710000000000+000000000020000000;
 * (7.100 MHz, USB, VFO A, RX, no split, 37 data chars)
 * ========================================================= */
#define CAT_DBG_HARDCODE_IF  0   /* set 1 to inject static TS-2000 IF test packet */

void CAT_BuildIF(CAT_Handle_t *cat, char *buf)
{
#if CAT_DBG_HARDCODE_IF
    /* Fixed TS-2000 reference: 7.100 MHz, USB, VFO A, RX, no split — 37 data chars + ';'
     * IF[00007100000][00000][+][0000][0][0][000][0][2][0][0][0][0][00][0]; */
    const char *ref = "IF0000710000000000+000000000020000000;";
    const char *s = ref;
    char *d = buf;
    while (*s) *d++ = *s++;
    *d = '\0';
    return;
#endif

    char *p = buf;

    /* P1 REAL — active VFO frequency */
    uint32_t f = (cat->active_vfo == 1U)
               ? (cat->cb.get_vfo_b_freq ? cat->cb.get_vfo_b_freq() : cat->vfo_b_freq)
               : (cat->cb.get_freq       ? cat->cb.get_freq()        : 7100000UL);

    /* P9 REAL — active VFO mode */
    uint8_t mode = (cat->active_vfo == 1U)
                 ? (cat->cb.get_vfo_b_mode
                    ? CAT_SDRModeToCat(cat->cb.get_vfo_b_mode())
                    : cat->vfo_b_mode)
                 : (cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB);

    /* P8 REAL — TX state */
    bool tx = cat->cb.get_tx ? cat->cb.get_tx() : false;

    *p++ = 'I'; *p++ = 'F';

    p = cat_put_u32(p, f, 11U);                                  /* [2-12]  P1  REAL  freq Hz, 11 digits */
    *p++ = '0'; *p++ = '0'; *p++ = '0'; *p++ = '0'; *p++ = '0'; /* [13-17] P2  FROZEN "00000" step */
    {
        int32_t  rv  = cat->cb.get_rit_hz ? cat->cb.get_rit_hz() : 0;
        bool     rp  = (rv >= 0);
        uint32_t rmg = (uint32_t)(rp ? rv : -rv);
        *p++ = rp ? '+' : '-';                                   /* [18]    P3  REAL  RIT sign */
        p = cat_put_u32(p, rmg, 4U);                            /* [19-22] P4  REAL  RIT Hz (0 when RIT off) */
        *p++ = cat->rit_on ? '1' : '0';                         /* [23]    P5  REAL  RIT on/off */
        *p++ = '0';                                              /* [24]    P6  FROZEN XIT off */
    }
    *p++ = '0'; *p++ = '0'; *p++ = '0';                         /* [25-27] P7  FROZEN "000" no memory */
    *p++ = tx ? '1' : '0';                                      /* [28]    P8  REAL  TX state */
    *p++ = (char)('0' + mode);                                   /* [29]    P9  REAL  mode code */
    *p++ = cat_vfo_digit(cat->active_vfo);                       /* [30]    P10 REAL  active VFO */
    *p++ = '0';                                                  /* [31]    P11 FROZEN no scan */
    *p++ = cat->split_on ? '1' : '0';                           /* [32]    P12 REAL  split */
    *p++ = '0';                                                  /* [33]    P13 FROZEN no tone */
    *p++ = '0'; *p++ = '0';                                     /* [34-35] P14 FROZEN no CTCSS */
    *p++ = '0';                                                  /* [36]    P15 FROZEN "0" — satisfies Hamlib if_len=37 */
    *p++ = ';';
    *p   = '\0';
}

/* =========================================================
 * Helpers for common responses
 * ========================================================= */
static void cat_build_FA(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint32_t f = (cat->active_vfo == 1U)
               ? (cat->cb.get_vfo_b_freq ? cat->cb.get_vfo_b_freq() : cat->vfo_b_freq)
               : (cat->cb.get_freq       ? cat->cb.get_freq()        : 7100000UL);

    *p++ = 'F'; *p++ = 'A';
    p = cat_put_u32(p, f, 11U);
    *p++ = ';';
    *p = '\0';
}

static void cat_build_FB(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    /* FA = active VFO (freq_hz), FB = inactive VFO (vfo_b.freq_hz) per storage model.
     * When active_vfo==1, active slot IS VFO B, so FB returns freq (get_freq),
     * not vfo_b (which would be VFO A after a swap). */
    uint32_t f = (cat->active_vfo == 1U)
               ? (cat->cb.get_freq       ? cat->cb.get_freq()       : 7100000UL)
               : (cat->cb.get_vfo_b_freq ? cat->cb.get_vfo_b_freq() : cat->vfo_b_freq);
    *p++ = 'F'; *p++ = 'B';
    p = cat_put_u32(p, f, 11U);
    *p++ = ';';
    *p = '\0';
}

static void cat_build_MD(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint8_t mode = (cat->active_vfo == 1U)
                 ? (cat->cb.get_vfo_b_mode
                    ? CAT_SDRModeToCat(cat->cb.get_vfo_b_mode())
                    : cat->vfo_b_mode)
                 : (cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB);

    *p++ = 'M'; *p++ = 'D';
    *p++ = (char)('0' + mode);
    *p++ = ';';
    *p = '\0';
}

static void cat_build_FW(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    /* TS-2000: FW = SH - SL for SSB/CW/DIGI (passband width in Hz, 4 digits).
     * FM: TS-2000 uses FW as a preset index (0001=2.5kHz, 0002=6kHz, 0003=15kHz),
     * not raw Hz.  Return FW0000 (auto/default) — Hamlib TS-2000 accepts 0000 as
     * "leave at current preset" and will not attempt to round-trip it back as Hz.
     * AM: SL is always 0, so FW = bw_hz directly. */
    uint8_t cur_mode = cat->cb.get_mode ? cat->cb.get_mode() : 2U;
    if (cur_mode == 1U) {          /* FM — return preset-index zero (auto) */
        cat_copy(buf, "FW0000;");
        return;
    }
    /* SH/SL are main-band (active VFO) parameters — always read from primary slot */
    uint32_t sh = cat->cb.get_bw ? cat->cb.get_bw() : 3000U;
    uint32_t sl = 0U;
    if (cur_mode != 0U) {          /* SSB/CW/DIGI: subtract lo-cut from passband */
        sl = cat->cb.get_lo_cut ? cat->cb.get_lo_cut() : 0U;
    }
    uint32_t fw = (sh > sl) ? (sh - sl) : 0U;
    if (fw > 9999U) fw = 9999U;
    *p++ = 'F'; *p++ = 'W';
    p = cat_put_u32(p, fw, 4U);
    *p++ = ';';
    *p = '\0';
}

/* TS-2000 SSB/CW high-cut filter table: index 00-09 → Hz
 * SH00-SH02 (1400-1800 Hz): narrow RX-only presets (below 2.0 kHz TX minimum)
 * SH03-SH08 (2000-3000 Hz): TX filter bandwidth presets per TS-2000 manual
 * SH09      (3400 Hz):      widest SSB preset */
static const uint32_t s_sh_tbl[10] = {
    1400U, 1600U, 1800U, 2000U, 2200U,
    2400U, 2600U, 2800U, 3000U, 3400U
};

static uint8_t cat_bw_to_sh(uint32_t bw)
{
    for (uint8_t i = 0U; i < 9U; i++) {
        if (bw <= s_sh_tbl[i]) return i;
    }
    return 9U;
}

static void cat_build_SH(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint32_t bw = cat->cb.get_bw ? cat->cb.get_bw() : 3000U;
    uint8_t idx = cat_bw_to_sh(bw);
    *p++ = 'S'; *p++ = 'H';
    *p++ = (char)('0' + (idx / 10U));
    *p++ = (char)('0' + (idx % 10U));
    *p++ = ';';
    *p = '\0';
}

/* TS-2000 SSB/CW low-cut filter table: index 00-11 → Hz
 * SL00 = 0 Hz (no low cut), SL01-SL11 = 50-1000 Hz in practical steps. */
static const uint32_t s_sl_tbl[12] = {
    0U, 50U, 100U, 200U, 300U, 400U,
    500U, 600U, 700U, 800U, 900U, 1000U
};

static uint8_t cat_hz_to_sl(uint32_t hz)
{
    for (uint8_t i = 0U; i < 11U; i++) {
        if (hz <= s_sl_tbl[i]) return i;
    }
    return 11U;
}

static void cat_build_SL(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    /* SL applies to SSB/CW only; AM/FM always return SL00 */
    uint8_t cur_mode = cat->cb.get_mode ? cat->cb.get_mode() : 2U;
    uint32_t sl = 0U;
    if (cur_mode != 0U && cur_mode != 1U) {
        sl = cat->cb.get_lo_cut ? cat->cb.get_lo_cut() : 0U;
    }
    uint8_t idx = cat_hz_to_sl(sl);
    *p++ = 'S'; *p++ = 'L';
    *p++ = (char)('0' + (idx / 10U));
    *p++ = (char)('0' + (idx % 10U));
    *p++ = ';';
    *p = '\0';
}

static void cat_build_SM(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    float db = cat->cb.get_signal_db ? cat->cb.get_signal_db() : -80.0f;
    if (db != db || db < -120.0f || db > 60.0f) db = -80.0f; /* NaN/inf/out-of-range guard */
    int32_t su = (int32_t)((db + 73.0f) / 2.0f);
    if (su < 0) su = 0;
    if (su > 30) su = 30;

    *p++ = 'S'; *p++ = 'M'; *p++ = '0';
    p = cat_put_u32(p, (uint32_t)su, 4U);
    *p++ = ';';
    *p = '\0';
}


static void cat_build_VS(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    *p++ = 'V'; *p++ = 'S';
    *p++ = cat_vfo_digit(cat->active_vfo);
    *p++ = ';';
    *p = '\0';
}

static void cat_build_DC(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    *p++ = 'D'; *p++ = 'C';
    *p++ = cat_vfo_digit(cat->active_vfo);
    *p++ = cat_vfo_digit(cat->split_on ? 1U : 0U);
    *p++ = ';';
    *p = '\0';
}

/* =========================================================
 * Command executor
 * ========================================================= */
static void cat_exec(CAT_Handle_t *cat, const char *cmd, char *resp)
{
    resp[0]       = '\0';
    s_exec_status = CAT_PARSE_ERROR; /* default for any ?; path in a known handler */

    /* FA — GET returns active VFO freq; SET is ACK-only, never touches active_vfo */
    if (cmd[0] == 'F' && cmd[1] == 'A') {
        if (cmd[2] == '\0') {
            cat_build_FA(cat, resp);
        } else {
            uint32_t f = cat_parse_u(&cmd[2], 11U);
            if (cat->active_vfo == 1U) {
                cat->vfo_b_freq = f;
                if (cat->cb.set_vfo_b_freq) cat->cb.set_vfo_b_freq(f);
            } else if (cat->cb.set_freq) {
                cat->cb.set_freq(f);
            }
            /* ACK-only: no response, active_vfo unchanged */
        }
    }

    /* FB — always VFO B; SET is ACK-only, never touches active_vfo */
    else if (cmd[0] == 'F' && cmd[1] == 'B') {
        if (cmd[2] == '\0') {
            cat_build_FB(cat, resp);
        } else {
            uint32_t f = cat_parse_u(&cmd[2], 11U);
            if (cat->active_vfo == 1U) {
                /* VFO B = active slot (freq_hz) — defer LO retune via set_freq */
                if (cat->cb.set_freq) cat->cb.set_freq(f);
            } else {
                /* VFO B = inactive slot (vfo_b.freq_hz) */
                cat->vfo_b_freq = f;
                if (cat->cb.set_vfo_b_freq) cat->cb.set_vfo_b_freq(f);
            }
            /* ACK-only: no response */
        }
    }

    /* MD — VFO-aware: routes to vfo_b_mode when active_vfo==1 */
    else if (cmd[0] == 'M' && cmd[1] == 'D') {
        if (cmd[2] == '\0') {
            cat_build_MD(cat, resp);
        } else {
            uint8_t m = (uint8_t)(cmd[2] - '0');
            if (cat->active_vfo == 1U) {
                cat->vfo_b_mode = m;
                if (cat->cb.set_vfo_b_mode) cat->cb.set_vfo_b_mode(CAT_CatModeToSDR(m));
            } else if (cat->cb.set_mode) {
                cat->cb.set_mode(CAT_CatModeToSDR(m));
            }
            /* ACK-only: TS-480 spec; Hamlib kenwood_transaction(NULL,0) does not read */
        }
    }

    /* TX variants — all silent, no response:
     *   TX;  / TX1; / TX2; → PTT on   (cmd[2] != '0')
     *   TX0;              → PTT off  (alias for RX;) */
    else if (cmd[0] == 'T' && cmd[1] == 'X') {
        if (cat->cb.set_tx) cat->cb.set_tx(cmd[2] != '0');
    }

    /* RX; = PTT off, silent */
    else if (cmd[0] == 'R' && cmd[1] == 'X') {
        if (cat->cb.set_tx) cat->cb.set_tx(false);
    }

    /* TQ; = PTT state query — Hamlib uses this to verify PTT after TX;/RX; */
    else if (cmd[0] == 'T' && cmd[1] == 'Q') {
        bool tx = cat->cb.get_tx ? cat->cb.get_tx() : false;
        resp[0] = 'T'; resp[1] = 'Q'; resp[2] = tx ? '1' : '0'; resp[3] = ';'; resp[4] = '\0';
    }

    /* IF */
    else if (cmd[0] == 'I' && cmd[1] == 'F') {
        CAT_BuildIF(cat, resp);
    }

    /* ID */
    else if (cmd[0] == 'I' && cmd[1] == 'D') {
        cat_copy(resp, "ID019;");
    }

    /* AI — SET is ACK-only: Hamlib calls kenwood_transaction("AI0", NULL, 0).
     * Echoing "AI0;" here would corrupt the next command's response buffer. */
    else if (cmd[0] == 'A' && cmd[1] == 'I') {
        if (cmd[2] != '\0') {
            uint8_t lv = (uint8_t)(cmd[2] - '0');
            cat->ai_level = (lv <= 2U) ? lv : 0U;
            /* ACK-only: no response */
        } else {
            resp[0] = 'A'; resp[1] = 'I'; resp[2] = (char)('0' + cat->ai_level); resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* AG — volume control: GET returns AG0nnn; (0-255), SET applies via callback.
     * TS-2000 format: AG0P2; where P2 = 000-255 (channel 0 = main receiver).
     * Internal volume is 0-100; scale on both sides.
     * SET is ACK-only — flrig TS-2000 uses sendCommand() with no readback. */
    else if (cmd[0] == 'A' && cmd[1] == 'G') {
        if (cmd[2] == '\0' || (cmd[2] == '0' && cmd[3] == '\0')) {
            /* GET: AG; or AG0; */
            uint8_t  vol = cat->cb.get_volume ? cat->cb.get_volume() : 100U;
            uint32_t ag  = (uint32_t)vol * 255U / 100U;
            if (ag > 255U) ag = 255U;
            char *p = resp;
            *p++ = 'A'; *p++ = 'G'; *p++ = '0';
            p = cat_put_u32(p, ag, 3U);
            *p++ = ';'; *p = '\0';
        } else if (cmd[2] == '0') {
            /* SET: AG0nnn; — scale 0-255 → 0-100 and apply */
            uint32_t ag  = cat_parse_u(&cmd[3], 3U);
            uint8_t  vol = (uint8_t)(ag * 100U / 255U);
            if (cat->cb.set_volume) cat->cb.set_volume(vol);
            /* ACK-only: AG is in suppress list below */
        }
    }

    /* NR — stub: no DSP NR wired through CAT path */
    else if (cmd[0] == 'N' && cmd[1] == 'R') {
        if (cmd[2] == '\0') { cat_copy(resp, "NR0;"); }
        /* SET NRn; — ACK-only stub */
    }

    /* NB — noise blanker: GET returns real state; SET wires to DSP via callback */
    else if (cmd[0] == 'N' && cmd[1] == 'B') {
        if (cmd[2] == '\0') {
            char tmp[5] = { 'N', 'B', '0', ';', '\0' };
            if (cat->cb.get_nb && cat->cb.get_nb()) { tmp[2] = '1'; }
            cat_copy(resp, tmp);
        } else {
            if (cat->cb.set_nb) { cat->cb.set_nb(cmd[2] != '0'); }
        }
    }

    /* FW — filter width: GET/SET routed to active VFO BW callbacks.
     * SET is ACK-only — flrig TS-2000 uses sendCommand() with no readback.
     * Returning FWnnnn; after SET leaves a stale frame in the host serial buffer
     * that desynchronises the next command's response read.
     * FM: BW stored in [5000, 15000] Hz; GET returns min(bw_hz, 9999) for 4-digit field. */
    else if (cmd[0] == 'F' && cmd[1] == 'W') {
        if (cmd[2] == '\0') {
            cat_build_FW(cat, resp);
        } else {
            uint32_t fw = cat_parse_u(&cmd[2], 4U);
            dbg_last_bw_cat_value = fw;
            dbg_last_bw_cmd = ((uint32_t)'F' << 8) | (uint32_t)'W';
            /* FW = passband width; SH = FW + SL. Retrieve current SL to compute new SH. */
            uint8_t cur_mode = cat->cb.get_mode ? cat->cb.get_mode() : 2U;
            uint32_t sl = 0U;
            if (cur_mode != 0U && cur_mode != 1U) {
                sl = cat->cb.get_lo_cut ? cat->cb.get_lo_cut() : 0U;
            }
            uint32_t sh = fw + sl;
            if (cat->cb.set_bw) cat->cb.set_bw(sh);
            /* ACK-only: FW is in suppress list below — no echo */
        }
    }

    /* SH — IF high-cut index: GET returns live BW→index; SET maps index→Hz, updates BW.
     * TS-2000: SH applies to SSB and CW only (indexes 00-09).
     * In AM, bandwidth is controlled by FW only; SH GET returns SH00 (N/A), SET ignored.
     * In FM, filter is fixed; SH is not applicable. */
    else if (cmd[0] == 'S' && cmd[1] == 'H') {
        uint8_t cur_mode = cat->cb.get_mode ? cat->cb.get_mode() : 2U;
        bool is_am_fm = (cur_mode == 0U || cur_mode == 1U);  /* SDR 0=AM, 1=FM */
        if (cmd[2] == '\0') {
            if (is_am_fm) {
                cat_copy(resp, "SH00;");  /* N/A in AM/FM — return index 0 */
            } else {
                cat_build_SH(cat, resp);
            }
        } else {
            if (!is_am_fm) {
                uint32_t idx = cat_parse_u(&cmd[2], 2U);
                dbg_last_bw_cat_value = idx;
                if (idx > 9U) idx = 9U;
                uint32_t bw = s_sh_tbl[idx];
                dbg_last_bw_cmd = ((uint32_t)'S' << 8) | (uint32_t)'H';
                if (cat->cb.set_bw) cat->cb.set_bw(bw);
            }
            /* ACK-only: SH SET suppress list prevents auto-echo (flrig no readback) */
        }
    }

    /* SL — IF low-cut: GET returns live index; SET applies to DSP via lo-cut callbacks.
     * SL applies to SSB/CW/DIGI only; AM/FM always return SL00 and ignore SETs. */
    else if (cmd[0] == 'S' && cmd[1] == 'L') {
        if (cmd[2] == '\0') {
            cat_build_SL(cat, resp);
        } else {
            uint8_t cur_mode = cat->cb.get_mode ? cat->cb.get_mode() : 2U;
            if (cur_mode != 0U && cur_mode != 1U) {
                uint32_t idx = cat_parse_u(&cmd[2], 2U);
                if (idx > 11U) idx = 11U;
                uint32_t sl = s_sl_tbl[idx];
                if (cat->cb.set_lo_cut) cat->cb.set_lo_cut(sl);
            }
            /* ACK-only: SL SET suppress list prevents auto-echo */
        }
    }

    /* SQ — stub: squelch not in minimal CAT set */
    else if (cmd[0] == 'S' && cmd[1] == 'Q') {
        if (cmd[2] == '\0' || (cmd[2] == '0' && cmd[3] == '\0')) {
            cat_copy(resp, "SQ0000;");
        }
        /* SET SQ0nnn; — ACK-only stub */
    }

    /* SM */
    else if (cmd[0] == 'S' && cmd[1] == 'M') {
        cat_build_SM(cat, resp);
    }

    /* RA */
    else if (cmd[0] == 'R' && cmd[1] == 'A') {
        if (cmd[2] == '\0') {
            uint8_t att = cat->cb.get_att ? cat->cb.get_att() : 0U;
            uint8_t lv = (att >= 18U) ? 3U : (att >= 12U) ? 2U : (att >= 6U) ? 1U : 0U;
            resp[0] = 'R'; resp[1] = 'A';
            resp[2] = (char)('0' + (lv / 10U));
            resp[3] = (char)('0' + (lv % 10U));
            resp[4] = ';';
            resp[5] = '\0';
        } else if (strlen(cmd) == 4U) {
            uint8_t lv = (uint8_t)(cmd[2] - '0') * 10U + (uint8_t)(cmd[3] - '0');
            if (cat->cb.set_att) cat->cb.set_att(lv > 3U ? 3U : lv);
            /* ACK-only */
        } else {
            cat_mark_malformed(cmd); cat_copy(resp, "?;");
        }
    }

    /* GT — stub: AGC speed not in minimal CAT set */
    else if (cmd[0] == 'G' && cmd[1] == 'T') {
        if (cmd[2] == '\0') { cat_copy(resp, "GT00;"); }  /* fast AGC */
        /* SET GT0n; — ACK-only stub */
    }

    /* PS */
    else if (cmd[0] == 'P' && cmd[1] == 'S') {
        cat_copy(resp, "PS1;");
    }

    /* PC — TX output power (0-100%).  Scales DSP audio amplitude via cat_set_tx_power.
     *   GET  PC;      → PC0nn; (3-digit, 000-100)
     *   SET  PCnnn;   → apply immediately; re-scales gain mid-TX if transmitting */
    else if (cmd[0] == 'P' && cmd[1] == 'C') {
        if (cmd[2] == '\0') {
            uint8_t pct = cat->cb.get_tx_power ? cat->cb.get_tx_power() : 100U;
            resp[0]='P'; resp[1]='C';
            resp[2]=(char)('0' + pct/100U);
            resp[3]=(char)('0' + (pct%100U)/10U);
            resp[4]=(char)('0' + pct%10U);
            resp[5]=';'; resp[6]='\0';
        } else if (cmd[2] >= '0' && cmd[2] <= '1' && cmd[3] >= '0' && cmd[3] <= '9' && cmd[4] >= '0' && cmd[4] <= '9') {
            uint8_t pct = (uint8_t)((cmd[2]-'0')*100 + (cmd[3]-'0')*10 + (cmd[4]-'0'));
            if (pct > 100U) pct = 100U;
            if (cat->cb.set_tx_power) cat->cb.set_tx_power(pct);
        } else {
            cat_mark_malformed(cmd); cat_copy(resp, "?;");
        }
    }

    /* BC — Beat Canceller stub (no hardware) */
    else if (cmd[0] == 'B' && cmd[1] == 'C') {
        if (cmd[2] == '\0') { cat_copy(resp, "BC0;"); }
        /* SET BCn; — ACK-only */
    }

    /* RG — RF AGC (PE4302 automatic front-end attenuator control)
     *   GET  RG;   → RG0; (off) or RG1; (on)
     *   SET  RG0;  → disable RF AGC (manual att control)
     *        RG1;  → enable RF AGC  (automatic overload prevention)
     * Non-standard extension; safe to ignore on TS-2000 compatible software. */
    else if (cmd[0] == 'R' && cmd[1] == 'G') {
        if (cmd[2] == '\0') {
            bool on = cat->cb.get_rf_agc ? cat->cb.get_rf_agc() : false;
            resp[0] = 'R'; resp[1] = 'G';
            resp[2] = on ? '1' : '0';
            resp[3] = ';'; resp[4] = '\0';
        } else if (cmd[2] == '0' || cmd[2] == '1') {
            if (cat->cb.set_rf_agc) cat->cb.set_rf_agc(cmd[2] == '1');
        } else {
            cat_mark_malformed(cmd); cat_copy(resp, "?;");
        }
    }

    /* VS — GET returns active VFO; SET selects active VFO (triggers hardware swap) */
    else if (cmd[0] == 'V' && cmd[1] == 'S') {
        if (cmd[2] != '\0') {
            cat->active_vfo = cat_clamp_vfo((uint8_t)(cmd[2] - '0'));
            if (cat->cb.set_active_vfo) cat->cb.set_active_vfo(cat->active_vfo);
        } else {
            cat_build_VS(cat, resp);
        }
    }

    /* DC — GET returns VFO+split routing; SET selects active VFO */
    else if (cmd[0] == 'D' && cmd[1] == 'C') {
        if (cmd[2] != '\0') {
            cat->active_vfo = cat_clamp_vfo((uint8_t)(cmd[2] - '0'));
            if (cmd[3] != '\0') cat->split_on = (cmd[3] == '1');
            if (cat->cb.set_active_vfo) cat->cb.set_active_vfo(cat->active_vfo);
        } else {
            cat_build_DC(cat, resp);
        }
    }

    /* FR — GET returns current RX VFO; SET is ACK-only.
     * flrig sends FR0;FT0; or FR1;FT1; via sendCommand() without reading back.
     * Echoing either pollutes the OS serial buffer and desynchronises all
     * subsequent reads (every subsequent get_* reads the wrong response). */
    else if (cmd[0] == 'F' && cmd[1] == 'R') {
        if (cmd[2] != '\0') {
            cat->active_vfo = cat_clamp_vfo((uint8_t)(cmd[2] - '0'));
            if (cat->cb.set_active_vfo) cat->cb.set_active_vfo(cat->active_vfo);
            /* ACK-only */
        } else {
            resp[0] = 'F'; resp[1] = 'R';
            resp[2] = cat_vfo_digit(cat->active_vfo); resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* FT — GET returns current TX VFO (split state); SET is ACK-only. */
    else if (cmd[0] == 'F' && cmd[1] == 'T') {
        if (cmd[2] != '\0') {
            cat->split_on = (cmd[2] == '1');
            /* ACK-only */
        } else {
            resp[0] = 'F'; resp[1] = 'T';
            resp[2] = cat->split_on ? '1' : '0'; resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* RT — RIT on/off.  SET re-applies DSP nco_if offset via set_rit_hz re-trigger. */
    else if (cmd[0] == 'R' && cmd[1] == 'T') {
        if (cmd[2] != '\0') {
            cat->rit_on = (cmd[2] == '1');
            /* Trigger cat_rit_dirty in main loop by calling set_rit_hz with current value */
            if (cat->cb.set_rit_hz && cat->cb.get_rit_hz)
                cat->cb.set_rit_hz(cat->cb.get_rit_hz());
        } else {
            resp[0] = 'R'; resp[1] = 'T';
            resp[2] = cat->rit_on ? '1' : '0'; resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* RC — RIT clear: zero offset and reapply DSP NCO */
    else if (cmd[0] == 'R' && cmd[1] == 'C') {
        if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(0);
        /* ACK-only */
    }

    /* RU — RIT up. TS-480: RUnnnnn; (5-digit Hz step). Bare RU; = 10 Hz. */
    else if (cmd[0] == 'R' && cmd[1] == 'U') {
        int32_t step = (cmd[2] != '\0') ? (int32_t)cat_parse_u(&cmd[2], 5U) : 10;
        int32_t cur  = cat->cb.get_rit_hz ? cat->cb.get_rit_hz() : 0;
        if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(cur + step);
        /* ACK-only */
    }

    /* RD — RIT down */
    else if (cmd[0] == 'R' && cmd[1] == 'D') {
        int32_t step = (cmd[2] != '\0') ? (int32_t)cat_parse_u(&cmd[2], 5U) : 10;
        int32_t cur  = cat->cb.get_rit_hz ? cat->cb.get_rit_hz() : 0;
        if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(cur - step);
        /* ACK-only */
    }

    /* XT — XIT on/off.  GET returns state; SET is ACK-only */
    else if (cmd[0] == 'X' && cmd[1] == 'T') {
        if (cmd[2] == '\0') {
            cat_copy(resp, "XT0;");
        }
        /* SET XT0;/XT1; — ACK-only */
    }

    /* IS — IF shift: GET formats current shift as IS0±nnnn; SET applies via callback.
     * TS-2000 IS format: IS0 + sign + 4-digit magnitude (e.g. IS0+0150; IS0-0300;)
     * GET may arrive as bare IS; or as IS0; (with P1='0' but no sign/magnitude). */
    else if (cmd[0] == 'I' && cmd[1] == 'S') {
        if (cmd[2] == '\0' || (cmd[2] == '0' && cmd[3] == '\0')) {
            int32_t hz = cat->cb.get_if_shift ? cat->cb.get_if_shift() : 0;
            char *p = resp;
            *p++ = 'I'; *p++ = 'S'; *p++ = '0';
            *p++ = (hz >= 0) ? '+' : '-';
            uint32_t mag = (uint32_t)(hz >= 0 ? hz : -hz);
            if (mag > 9999U) mag = 9999U;
            p = cat_put_u32(p, mag, 4U);
            *p++ = ';';
            *p = '\0';
        } else if (cmd[3] != '\0') {
            /* IS0±nnnn; — P1='0', P2=sign, P3-6=magnitude */
            char sign = cmd[3];
            uint32_t mag = cat_parse_u(&cmd[4], 4U);
            if (mag > 9999U) mag = 9999U;
            int32_t hz = (int32_t)mag;
            if (sign == '-') hz = -hz;
            if (cat->cb.set_if_shift) cat->cb.set_if_shift(hz);
            cat->if_shift = (int16_t)hz;
            /* ACK-only: IS is in suppress list — no echo */
        }
    }

    /* SP — GET returns split state; SET is ACK-only */
    else if (cmd[0] == 'S' && cmd[1] == 'P') {
        if (cmd[2] != '\0') {
            cat->split_on = (cmd[2] == '1');
        } else {
            resp[0] = 'S'; resp[1] = 'P';
            resp[2] = cat->split_on ? '1' : '0'; resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* TS — stub: tuning step not in minimal CAT set */
    else if (cmd[0] == 'T' && cmd[1] == 'S') {
        if ((uint8_t)strlen(cmd) <= 3U) { cat_copy(resp, "TS006;"); }  /* 100 Hz */
        /* SET TS0nn; — ACK-only stub */
    }

    /* VV — ACK-only stub: VFO copy not triggered from CAT path */
    else if (cmd[0] == 'V' && cmd[1] == 'V') { /* ACK-only stub */ }

    /* ACK-only — Hamlib uses kenwood_transaction(cmd, NULL, 0) for all of these.
     * Echoing them corrupts the next command's response buffer on the host. */
    else if (cmd[0] == 'U' && cmd[1] == 'P') { /* ACK-only */ }
    else if (cmd[0] == 'D' && cmd[1] == 'N') { /* ACK-only */ }
    else if (cmd[0] == 'B' && cmd[1] == 'U') { /* ACK-only */ }
    else if (cmd[0] == 'B' && cmd[1] == 'D') { /* ACK-only */ }
    else if (cmd[0] == 'M' && cmd[1] == 'W') { /* ACK-only */ }
    else if (cmd[0] == 'D' && cmd[1] == 'S') { /* ACK-only */ }
    else if (cmd[0] == 'T' && cmd[1] == 'C') { /* ACK-only */ }
    else if (cmd[0] == 'K' && cmd[1] == 'Y') { /* ACK-only */ }
    else if (cmd[0] == 'M' && cmd[1] == 'R') { /* ACK-only: memory recall not implemented */ }

    /* MN — menu item select.  flrig TS-480 init probes this; a ?; response
     * causes stoi("") → std::invalid_argument → crash in flrig. Stub = 000. */
    else if (cmd[0] == 'M' && cmd[1] == 'N') {
        if (cmd[2] == '\0') { cat_copy(resp, "MN000;"); }
        /* SET MNnnn; — ACK-only */
    }

    /* MP — menu parameter read/write.  Same crash risk as MN. Stub = 0. */
    else if (cmd[0] == 'M' && cmd[1] == 'P') {
        if (cmd[2] == '\0') { cat_copy(resp, "MP0000;"); }
        /* SET MPnnnn; — ACK-only */
    }

    /* KS — CW keyer speed (WPM).  flrig TS-480 queries this during init;
     * returning ?; without a guard check in flrig would stoi("") → crash. */
    else if (cmd[0] == 'K' && cmd[1] == 'S') {
        if (cmd[2] == '\0') {
            cat_copy(resp, "KS010;");   /* 10 WPM stub */
        }
        /* SET KSnnn; — ACK-only */
    }

    /* LK — panel lock query/set.  Stub: always unlocked */
    else if (cmd[0] == 'L' && cmd[1] == 'K') {
        if (cmd[2] == '\0') {
            cat_copy(resp, "LK0;");
        }
        /* SET LK0;/LK1; — ACK-only */
    }

    /* MG — microphone gain.  flrig TS-480 probes this; stub 50% */
    else if (cmd[0] == 'M' && cmd[1] == 'G') {
        if (cmd[2] == '\0') {
            cat_copy(resp, "MG050;");
        }
        /* SET MGnnn; — ACK-only */
    }

    /* EX — extended menu (TS-480 specific).
     * READ: flrig sends EXnnnXXXX; (9-char cmd, e.g. "EX0450000;") and expects an
     *       11-char response: EXnnnXXXXY; where Y is the current menu value digit.
     *       Stub: echo command back with '0' appended → all menu values = 0.
     *       menu_45 = false → standard SL/SH filter tables (correct for stub).
     * SET:  flrig sends EXnnnXXXXX; (10-char cmd, e.g. "EX01200000;") via
     *       sendCommand() with NO readback.  Must be ACK-only and NOT auto-echoed
     *       or the stale echo corrupts the next command's response buffer. */
    else if (cmd[0] == 'E' && cmd[1] == 'X') {
        size_t exlen = strlen(cmd);
        if (exlen == 9U) {
            /* READ: EXnnnXXXX → respond EXnnnXXXXY; (11 chars, Y='0' stub) */
            memcpy(resp, cmd, 9U);
            resp[9]  = '0';
            resp[10] = ';';
            resp[11] = '\0';
        }
        /* SET (exlen==10) or bare EX; — ACK-only, no response */
    }

    /* PA — preamplifier stub (no hardware preamp) */
    else if (cmd[0] == 'P' && cmd[1] == 'A') {
        if (cmd[2] == '\0') { cat_copy(resp, "PA0;"); }
        /* SET PAn; — ACK-only */
    }

    /* RG — RF gain stub (no hardware RF gain control) */
    else if (cmd[0] == 'R' && cmd[1] == 'G') {
        if (cmd[2] == '\0') { cat_copy(resp, "RG100;"); }
        /* SET RGnnn; — ACK-only */
    }

    /* RL — noise reduction level (0-09).  flrig TS-480 queries this during state read.
     * Stub: level 00 (minimum) — consistent with NR0 (off) stub. */
    else if (cmd[0] == 'R' && cmd[1] == 'L') {
        if (cmd[2] == '\0') { cat_copy(resp, "RL00;"); }
        /* SET RLnn; — ACK-only */
    }

    else {
        /* Capture raw frame — unknown opcode, not in handler list */
        s_exec_status = CAT_PARSE_UNKNOWN;
        {
            size_t ulen = strlen(cmd);
            if (ulen >= sizeof(dbg_last_unknown_cmd)) ulen = sizeof(dbg_last_unknown_cmd) - 1U;
            memcpy(dbg_last_unknown_cmd, cmd, ulen);
            memset(dbg_last_unknown_cmd + ulen, 0, sizeof(dbg_last_unknown_cmd) - ulen);
        }
        cat_copy(resp, "?;");
    }

}

/* =========================================================
 * CAT Process
 * ========================================================= */
void CAT_Process(CAT_Handle_t *cat)
{
    if (!cat || !cat->initialized) return;

    /* Reentrancy guard — uses file-scope dbg_cat_in_process (not a static local)
     * so CAT_Init can reset it after a fault, and the debugger can watch it. */
    if (dbg_cat_in_process) { dbg_process_reenters++; return; }
    dbg_cat_in_process = 1U;

    uint32_t t0_cyc = DWT->CYCCNT;
    uint32_t now    = cat_now_ms();
    bool     skip_ai = false;

    dbg_cat_parse_calls++;

    /* Partial-command timeout: if no ';' arrived within 200ms after the last
     * received byte, the in-flight assembly is stale (split-packet that never
     * completed, or line noise).  Discard it so it cannot prefix the next
     * command from the host. */
    if (cat->parser_len > 0U && (now - cat->parser_last_rx_tick) > 200U) {
        dbg_cat_partial_timeouts++;
        /* Capture partial frame before discarding */
        cat->parser_cmd[cat->parser_len] = '\0';
        {
            size_t tlen = cat->parser_len;
            if (tlen >= sizeof(dbg_last_timeout)) tlen = sizeof(dbg_last_timeout) - 1U;
            memcpy(dbg_last_timeout, cat->parser_cmd, tlen);
            memset(dbg_last_timeout + tlen, 0, sizeof(dbg_last_timeout) - tlen);
        }
        cat->parser_len = 0U;
    }

    /* Snapshot RX buffer in one atomic grab so CAT_Receive (USB ISR) cannot
     * race with the parse loop below.
     * BASEPRI = 0x20: masks USB OTG IRQ (priority 2 → encoded 0x20) while
     * leaving SAI/DMA audio IRQs (priority 0 → 0x00) fully unmasked.
     * The memcpy must stay inside the masked window: after rx_len is zeroed
     * the ISR would write new bytes from rx_buf[0], corrupting our snapshot.
     * At 480 MHz, copying ≤512 B takes ~1 µs — audio is unaffected. */
    char     work[CAT_BUF_SIZE];
    uint16_t work_len;
    __set_BASEPRI(0x20U);
    work_len    = cat->rx_len;
    memcpy(work, cat->rx_buf, work_len);
    cat->rx_len = 0U;
    __set_BASEPRI(0U);

    uint16_t wi = 0U;
    while (wi < work_len) {
        char c = work[wi++];

        if (c == '\r' || c == '\n') continue;
        if (c == '\0') { dbg_cat_rx_null_bytes++; continue; }
        if (cat->parser_len < (CAT_BUF_SIZE - 1U)) cat->parser_cmd[cat->parser_len++] = c;

        if (c == ';') {
            /* Strip terminator, NUL-terminate for cat_exec */
            cat->parser_cmd[cat->parser_len - 1U] = '\0';

            /* Skip bare ';' (empty command, parser_len==1 means only the ';' was
             * in the buffer).  cat_exec("") would fall through to '?;' which breaks
             * kenwood_transaction on the host.  Reset and continue silently. */
            if (cat->parser_len == 1U) {
                cat->parser_len = 0U;
                continue;
            }

            /* Track peak assembly length for overflow monitoring */
            if (cat->parser_len > dbg_cat_max_cmd_len)
                dbg_cat_max_cmd_len = cat->parser_len;

            /* Capture last complete command — bounded copy + zero tail so debugger
             * does not show stale bytes from a previous longer command. */
            {
                size_t clen = (cat->parser_len < sizeof(dbg_cat_last_cmd))
                              ? cat->parser_len : sizeof(dbg_cat_last_cmd) - 1U;
                memcpy(dbg_cat_last_cmd, cat->parser_cmd, clen);
                memset(dbg_cat_last_cmd + clen, 0, sizeof(dbg_cat_last_cmd) - clen);
            }

            /* Suppress AI IF notification for the cycle that contains a PTT command */
            if ((cat->parser_cmd[0] == 'T' && cat->parser_cmd[1] == 'X') ||
                (cat->parser_cmd[0] == 'R' && cat->parser_cmd[1] == 'X')) {
                skip_ai = true;
            }

            char resp[CAT_TX_BUF_SIZE];
            memset(resp, 0, sizeof(resp));  /* zero-init: eliminates stale stack bytes if a handler writes partial data */
            cat_exec(cat, cat->parser_cmd, resp);

            /* Auto-echo for SET commands that produced no response.
             * Condition: handler left resp empty (ACK-only) AND command has a payload
             * (cmd[2] != '\0', so it is a SET, not a bare GET opcode).
             * flrig reads back SET acknowledgements; Hamlib kenwood_transaction(cmd,NULL,0)
             * reads and discards the echo — safe for both hosts.
             *
             * EXCEPTIONS: commands where Hamlib uses kenwood_transaction(cmd,NULL,0) and
             * does NOT read any response.  Sending an unsolicited echo leaves it in the
             * host serial buffer and corrupts the next command's response read. */
            if (resp[0] == '\0' && cat->parser_cmd[2] != '\0') {
                const char *_c = cat->parser_cmd;
                bool _suppress =
                    (_c[0]=='A' && _c[1]=='I') ||  /* AI0; — explicit: corrupts next resp */
                    (_c[0]=='T' && _c[1]=='X') ||  /* TX;/TX0;/TX1; — all silent         */
                    (_c[0]=='U' && _c[1]=='P') ||  /* UP  — Hamlib no-read               */
                    (_c[0]=='D' && _c[1]=='N') ||  /* DN  — Hamlib no-read               */
                    (_c[0]=='B' && _c[1]=='U') ||  /* BU  — Hamlib no-read               */
                    (_c[0]=='B' && _c[1]=='D') ||  /* BD  — Hamlib no-read               */
                    (_c[0]=='M' && _c[1]=='W') ||  /* MW  — Hamlib no-read               */
                    (_c[0]=='D' && _c[1]=='S') ||  /* DS  — Hamlib no-read               */
                    (_c[0]=='T' && _c[1]=='C') ||  /* TC  — Hamlib no-read               */
                    (_c[0]=='K' && _c[1]=='Y') ||  /* KY  — Hamlib no-read               */
                    (_c[0]=='M' && _c[1]=='R') ||  /* MR  — Hamlib no-read               */
                    (_c[0]=='E' && _c[1]=='X') ||  /* EX SET: flrig sendCommand(), no readback — echo corrupts next resp */
                    (_c[0]=='M' && _c[1]=='D') ||  /* MD SET: flrig sendCommand() for set_modeA/B — no readback        */
                    (_c[0]=='S' && _c[1]=='L') ||  /* SL SET: flrig sendCommand() for lo-cut — no readback            */
                    (_c[0]=='S' && _c[1]=='H') ||  /* SH SET: flrig sendCommand() for hi-cut — no readback            */
                    (_c[0]=='F' && _c[1]=='R') ||  /* FR SET: flrig sendCommand(), no readback — echo desynchs buffer  */
                    (_c[0]=='F' && _c[1]=='T') ||  /* FT SET: flrig sendCommand(), no readback — echo desynchs buffer  */
                    (_c[0]=='F' && _c[1]=='W') ||  /* FW SET: flrig sendCommand(), no readback — stale FWnnnn; desynchs */
                    (_c[0]=='A' && _c[1]=='G') ||  /* AG SET: flrig sendCommand(), no readback — stale AG0nnn; desynchs */
                    (_c[0]=='V' && _c[1]=='S') ||  /* VS SET: flrig init/VFO switch, sendCommand(), no readback        */
                    (_c[0]=='S' && _c[1]=='P') ||  /* SP SET: flrig split, sendCommand(), no readback                  */
                    (_c[0]=='R' && _c[1]=='T') ||  /* RT SET: flrig RIT toggle, sendCommand(), no readback             */
                    (_c[0]=='D' && _c[1]=='C');    /* DC SET: flrig VFO routing, sendCommand(), no readback            */
                if (!_suppress) {
                    uint8_t clen = (uint8_t)strlen(cat->parser_cmd);
                    if (clen < (uint8_t)(CAT_TX_BUF_SIZE - 1U)) {
                        memcpy(resp, cat->parser_cmd, clen);
                        resp[clen]      = ';';
                        resp[clen + 1U] = '\0';
                    }
                }
            }

            /* Classify and record the transaction */
            {
                CAT_ParseStatus_t st;
                if (resp[0] == '\0') {
                    st = CAT_PARSE_NORESP;
                } else if (resp[0] == '?' && resp[1] == ';') {
                    st = s_exec_status; /* UNKNOWN or ERROR set inside cat_exec */
                    if (st == CAT_PARSE_UNKNOWN) dbg_cat_unknown_cmds++;
                    else                          dbg_cat_malformed_frames++;
                } else {
                    st = CAT_PARSE_OK;
                    cat_validate_resp(resp);
                }
                cat_txn_record(cat->parser_cmd, resp, st, now);
            }

            if (resp[0] != '\0') {
                /* Guard: reject responses with embedded NUL or non-ASCII before ';'.
                 * Such a response would either be silently truncated by cat_tx_enqueue
                 * (stopping at the embedded NUL, sending no terminator) or corrupt the
                 * host parser.  dbg_resp_null_drop is incremented on rejection. */
                if (cat_resp_guard(resp)) {
                    size_t rlen = strlen(resp) + 1U;
                    if (rlen > sizeof(dbg_cat_last_resp)) rlen = sizeof(dbg_cat_last_resp);
                    memcpy(dbg_cat_last_resp, resp, rlen);
                    memset(dbg_cat_last_resp + rlen, 0, sizeof(dbg_cat_last_resp) - rlen);
                    cat_tx_enqueue(resp);
                }
            }

            cat->parser_len = 0U;
        }
    }

    /* AI unsolicited IF — only when nothing is pending in the TX FIFO.
     * Injecting here while responses are queued would interleave with
     * command-response pairs and break kenwood_transaction verification.
     * dbg_cat_ai_push_disable=1 (default) suppresses all unsolicited push regardless
     * of ai_level.  Set to 0 only after strict serialization is confirmed stable:
     * an unsolicited IF injected between a command send and its response read
     * leaves a stale IF frame in the OS serial buffer that desynchronizes flrig. */
    if (!dbg_cat_ai_push_disable && cat->ai_level > 0U && !skip_ai && (s_tx_head == s_tx_tail)) {
        /* Always read live data through callbacks so AI fires after UI VFO swaps too */
        uint32_t cur_freq = (cat->active_vfo == 1U)
                          ? (cat->cb.get_vfo_b_freq ? cat->cb.get_vfo_b_freq() : cat->vfo_b_freq)
                          : (cat->cb.get_freq        ? cat->cb.get_freq()        : 0U);
        uint8_t  cur_mode = (cat->active_vfo == 1U)
                          ? (cat->cb.get_vfo_b_mode
                             ? CAT_SDRModeToCat(cat->cb.get_vfo_b_mode())
                             : cat->vfo_b_mode)
                          : (cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB);
        bool     cur_tx   = cat->cb.get_tx ? cat->cb.get_tx() : false;
        uint8_t  cur_vfo  = cat->cb.get_active_vfo ? cat->cb.get_active_vfo() : cat->active_vfo;

        if (cur_freq  != cat->last_freq  ||
            cur_mode  != cat->last_mode  ||
            cur_tx    != cat->last_tx    ||
            cur_vfo   != cat->last_vfo   ||
            cat->split_on != cat->last_split) {

            cat->last_freq  = cur_freq;
            cat->last_mode  = cur_mode;
            cat->last_tx    = cur_tx;
            cat->last_vfo   = cur_vfo;
            cat->last_split = cat->split_on;

            char if_buf[50];
            CAT_BuildIF(cat, if_buf);
            cat_tx_enqueue(if_buf);
            /* Do NOT flush here — CDC IN is busy from the FlushTX above.
             * The main-loop FlushTX fires within 1 ms and drains the FIFO. */
        }
    }

    /* Record wall-time for this pass using DWT (µs resolution at 480 MHz).
     * Unsigned subtraction handles the ~8.9 s CYCCNT wrap correctly. */
    dbg_cat_parse_latency_us = (DWT->CYCCNT - t0_cyc) / (SystemCoreClock / 1000000U);

    dbg_cat_in_process = 0U;
}
