/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_cat.h
  * @brief   USB CAT — Kenwood TS-480 (ID020) minimal stable profile
  *
  *  Transport : USB CDC VCP (EP1 IN/OUT, 64-byte FS bulk packets)
  *  Protocol  : Kenwood TS-480 CAT, ASCII semicolon-terminated frames
  *  Target    : flrig / WSJT-X / Hamlib — stable polling, no unsolicited push
  *
  *  ┌─────────────────────────────────────────────────────────────────────────┐
  *  │ COMMAND INVENTORY                                                       │
  *  │ REAL    = live hardware/DSP state, deferred via dirty flags             │
  *  │ STUB    = safe fixed response, ACK-only SET, no hardware side effect    │
  *  │ ACK     = SET accepted silently (no response), GET returns ?;           │
  *  ├──────┬──────────────────────────────┬────────┬───────────────────────┤
  *  │ Cmd  │ Description                  │ Status │ Response / note       │
  *  ├──────┼──────────────────────────────┼────────┼───────────────────────┤
  *  │ FA   │ VFO-A frequency              │ REAL   │ FA00007100000;        │
  *  │ FB   │ VFO-B frequency              │ REAL   │ FB00014200000;        │
  *  │ IF   │ Status frame (38 chars)      │ REAL*  │ *P1/P5/P8/P9/P10/P12 │
  *  │ MD   │ Mode (LSB/USB/CW/FM/AM)      │ REAL   │ MD2;                  │
  *  │ TX   │ PTT on (TX1/TX0 variants)    │ REAL   │ ACK-only              │
  *  │ RX   │ PTT off                      │ REAL   │ ACK-only              │
  *  │ TQ   │ TX state query               │ REAL   │ TQ0; / TQ1;           │
  *  │ RA   │ RX attenuator (PE4302)       │ REAL   │ RA00;..RA03;          │
  *  │ SM   │ Signal meter (DSP power)     │ REAL   │ SM00012;              │
  *  │ VS   │ Active VFO select            │ REAL   │ VS0; / VS1;           │
  *  │ FR   │ RX VFO route                 │ REAL   │ FR0; / FR1;           │
  *  │ FT   │ TX VFO / split               │ REAL   │ FT0; / FT1;           │
  *  │ SP   │ Split on/off                 │ REAL   │ SP0; / SP1;           │
  *  │ DC   │ Dual-VFO + split             │ REAL   │ DC00; / DC11;         │
  *  │ RT   │ RIT on/off                   │ REAL   │ RT0; / RT1;           │
  *  │ RC   │ RIT clear (zero offset)      │ REAL   │ ACK-only              │
  *  │ RU   │ RIT increment (Hz)           │ REAL   │ ACK-only              │
  *  │ RD   │ RIT decrement (Hz)           │ REAL   │ ACK-only              │
  *  │ AI   │ Auto-info level              │ REAL   │ push disabled (=0)    │
  *  │ ID   │ Device identity              │ STUB   │ ID020;                │
  *  │ PS   │ Power status                 │ STUB   │ PS1;                  │
  *  │ AG   │ Audio gain                   │ REAL   │ AG0nnn; live 0-255    │
  *  │ NR   │ Noise reduction              │ STUB   │ NR0; (fixed off)      │
  *  │ NB   │ Noise blanker                │ REAL   │ NB0;/NB1; + DSP live  │
  *  │ FW   │ Filter width                 │ REAL   │ FWnnnn; live BW       │
  *  │ SH   │ IF high-cut                  │ REAL   │ SHnn; live BW→index   │
  *  │ SL   │ IF low-cut                   │ STUB   │ SL00; (no low-cut HW) │
  *  │ SQ   │ Squelch                      │ STUB   │ SQ0000; (fixed off)   │
  *  │ GT   │ AGC speed                    │ STUB   │ GT00; (fixed fast)    │
  *  │ PC   │ TX power                     │ STUB   │ PC050; (no HW)        │
  *  │ PA   │ Preamp                       │ STUB   │ PA0; (no HW)          │
  *  │ RG   │ RF gain                      │ STUB   │ RG100; (no HW)        │
  *  │ RL   │ NR level                     │ STUB   │ RL00; (fixed)         │
  *  │ BC   │ Beat canceller               │ STUB   │ BC0; (no HW)          │
  *  │ TS   │ Tuning step                  │ STUB   │ TS006; (fixed)        │
  *  │ IS   │ IF shift                     │ STUB   │ IS0+0000; (fixed)     │
  *  │ XT   │ XIT on/off                   │ STUB   │ XT0; (fixed off)      │
  *  │ MN   │ Menu item select             │ STUB   │ MN000; (fixed)        │
  *  │ MP   │ Menu parameter               │ STUB   │ MP0000; (fixed)       │
  *  │ KS   │ CW keyer speed               │ STUB   │ KS010; (no keyer)     │
  *  │ LK   │ Panel lock                   │ STUB   │ LK0; (fixed unlock)   │
  *  │ MG   │ Mic gain                     │ STUB   │ MG050; (fixed)        │
  *  │ EX   │ Extended menu (TS-480)       │ STUB   │ echo + '0' suffix     │
  *  │ VV   │ VFO copy                     │ ACK    │ silent                │
  *  │ UP   │ Frequency up step            │ ACK    │ silent                │
  *  │ DN   │ Frequency down step          │ ACK    │ silent                │
  *  │ BU   │ Band up                      │ ACK    │ silent                │
  *  │ BD   │ Band down                    │ ACK    │ silent                │
  *  │ MW   │ Memory write                 │ ACK    │ silent (no memory)    │
  *  │ MR   │ Memory recall                │ ACK    │ silent (no memory)    │
  *  │ DS   │ Display set                  │ ACK    │ silent (no display)   │
  *  │ TC   │ TNC control                  │ ACK    │ silent                │
  *  │ KY   │ CW send                      │ ACK    │ silent (no keyer)     │
  *  │ *    │ All other opcodes            │ ?;     │ dbg_cat_unknown_cmds++│
  *  └──────┴──────────────────────────────┴────────┴───────────────────────┘
  *
  *  IF frame fields (* = frozen):
  *   P1 freq REAL  P2 step* P3 RIT-sign REAL  P4 RIT-Hz REAL  P5 RIT-on REAL
  *   P6 XIT*  P7 mem*  P8 TX REAL  P9 mode REAL  P10 VFO REAL
  *   P11 scan*  P12 split REAL  P13 tone*  P14 CTCSS*  P15 shift*
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USB_CAT_H
#define __USB_CAT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define CAT_BUF_SIZE      128U   /* Max command length */
#define CAT_TX_BUF_SIZE   128U   /* Response buffer    */
#define CAT_FIFO_SIZE      16U   /* Pending responses  */

/* Kenwood mode codes */
#define CAT_MODE_LSB    1U
#define CAT_MODE_USB    2U
#define CAT_MODE_CW     3U
#define CAT_MODE_FM     4U
#define CAT_MODE_AM     5U
#define CAT_MODE_FSK    6U
#define CAT_MODE_CWR    7U
#define CAT_MODE_FSKR   9U

/* Exported types ------------------------------------------------------------*/

/** Callback set: CAT driver calls these to apply commands to SDR state */
typedef struct {
  /* Setters – active VFO */
  void (*set_freq)(uint32_t freq_hz);
  void (*set_mode)(uint8_t sdr_mode);
  void (*set_tx)(bool tx_on);
  void (*set_att)(uint8_t level_0_3);
  void (*set_volume)(uint8_t vol);         /*!< AG: 0-100       */
  void (*set_nr)(bool on);                 /*!< NR on/off       */
  void (*set_nb)(bool on);                 /*!< NB on/off       */
  void (*set_bw)(uint32_t hz);             /*!< FW: Hz          */
  void (*set_agc_fast)(bool fast);         /*!< GT: fast/slow   */
  void (*set_squelch)(uint8_t sq);         /*!< SQ: 0-255       */
  void (*set_rit_hz)(int32_t hz);          /*!< RIT offset Hz   */
  void (*set_step)(uint32_t hz);           /*!< Tuning step Hz  */
  void (*set_if_shift)(int32_t hz);        /*!< IS: IF shift Hz */
  /* Getters – active VFO */
  uint32_t (*get_freq)(void);
  uint8_t  (*get_mode)(void);
  bool     (*get_tx)(void);
  float    (*get_signal_db)(void);
  uint8_t  (*get_att)(void);
  uint8_t  (*get_volume)(void);
  bool     (*get_nr)(void);
  bool     (*get_nb)(void);
  uint32_t (*get_bw)(void);
  bool     (*get_agc_fast)(void);
  uint8_t  (*get_squelch)(void);
  int32_t  (*get_rit_hz)(void);            /*!< RIT offset Hz   */
  uint32_t (*get_step)(void);              /*!< Tuning step Hz  */
  int32_t  (*get_if_shift)(void);          /*!< IS: IF shift Hz */
  /* VFO B state – keep inactive VFO in sync with main SDR state */
  void     (*set_vfo_b_freq)(uint32_t hz);
  void     (*set_vfo_b_mode)(uint8_t sdr_mode);
  void     (*set_vfo_b_bw)(uint32_t hz);
  uint32_t (*get_vfo_b_freq)(void);
  uint8_t  (*get_vfo_b_mode)(void);        /*!< returns SDR mode code   */
  uint32_t (*get_vfo_b_bw)(void);
  /* Active VFO selection – triggers hardware swap when changed */
  void     (*set_active_vfo)(uint8_t vfo); /*!< 0=A, 1=B                */
  uint8_t  (*get_active_vfo)(void);
} CAT_Callbacks_t;

/** CAT driver state */
typedef struct {
  char     rx_buf[CAT_BUF_SIZE];
  uint16_t rx_len;
  char     tx_buf[CAT_TX_BUF_SIZE];
  uint8_t  ai_level;               /*!< 0=off, 1=AI1, 2=AI2      */
  uint32_t last_freq;
  uint8_t  last_mode;
  bool     last_tx;                /*!< TX state tracked for AI unsolicited IF */
  uint8_t  last_vfo;               /*!< active_vfo at last AI notification     */
  bool     last_split;             /*!< split_on at last AI notification       */
  bool     rit_on;                 /*!< RIT on/off                */
  int16_t  if_shift;               /*!< IS: IF shift Hz           */
  uint32_t vfo_b_freq;             /*!< VFO B stored frequency    */
  uint8_t  vfo_b_mode;             /*!< VFO B stored mode (CAT code) */
  uint32_t vfo_b_bw;               /*!< VFO B stored bandwidth Hz */
  uint8_t  active_vfo;             /*!< 0=VFO_A, 1=VFO_B (VS cmd) */
  bool     split_on;               /*!< Split: TX on VFO B        */
  bool     pa_on;                  /*!< PA preamp on/off (compatibility state) */
  CAT_Callbacks_t cb;
  bool     initialized;
  /* Parser state — kept in handle so CAT_Init resets them on USB reconnect.
   * Previously these were static locals inside CAT_Process, which survived
   * reconnect and could prefix the first new command with stale bytes. */
  char     parser_cmd[CAT_BUF_SIZE]; /* command being assembled (no ';')    */
  uint8_t  parser_len;               /* bytes accumulated so far            */
  uint32_t parser_last_rx_tick;      /* HAL_GetTick() at last received byte */
} CAT_Handle_t;

/* =========================================================================
 * CAT forensic types
 * ========================================================================= */

/** Outcome of each completed CAT transaction */
typedef enum {
    CAT_PARSE_OK      = 0U, /*!< valid response sent                        */
    CAT_PARSE_NORESP  = 1U, /*!< ACK-only — no bytes enqueued               */
    CAT_PARSE_ERROR   = 2U, /*!< ?; sent — opcode known, bad parameters     */
    CAT_PARSE_UNKNOWN = 3U, /*!< ?; sent — opcode not in handler list       */
} CAT_ParseStatus_t;

/** One command+response pair recorded for forensics */
#define CAT_TXN_CMD_MAX  32U
#define CAT_TXN_RESP_MAX 48U
typedef struct {
    uint32_t          ms;                        /*!< HAL_GetTick() at exec              */
    char              cmd [CAT_TXN_CMD_MAX];     /*!< command string, no ';'             */
    char              resp[CAT_TXN_RESP_MAX];    /*!< response string, with ';'          */
    uint8_t           resp_raw[CAT_TXN_RESP_MAX];/*!< same bytes as resp, typed uint8_t  */
    uint8_t           cmd_len;
    uint8_t           resp_len;
    uint8_t           is_set;                    /*!< 1=SET (has payload), 0=GET         */
    CAT_ParseStatus_t status;
    char              opcode[3];                 /*!< 2-char opcode + NUL                */
} CAT_Txn_t;

/** Circular ring buffer — last CAT_TXN_RING_SIZE completed transactions */
#define CAT_TXN_RING_SIZE 32U
typedef struct {
    CAT_Txn_t entries[CAT_TXN_RING_SIZE];
    uint8_t   next;   /*!< next write slot (0..CAT_TXN_RING_SIZE-1) */
    uint8_t   count;  /*!< total written, saturates at 255           */
} CAT_TxnRing_t;

/** Per-opcode statistics for the 13 most common Kenwood commands */
typedef struct {
    uint32_t rx;  /*!< times received   */
    uint32_t ok;  /*!< successful reply */
    uint32_t err; /*!< ?; or unknown    */
} CAT_OpStat_t;

typedef enum {
    COPI_IF=0, COPI_FA, COPI_FB, COPI_MD, COPI_AI,
    COPI_ID,   COPI_TX, COPI_RX, COPI_PS, COPI_UP,
    COPI_DN,   COPI_AG, COPI_SQ,
    COPI_COUNT
} CAT_OpiIdx_t;

/* =========================================================================
 * Exported variables
 * ========================================================================= */
extern CAT_Handle_t g_cat;

/* Transport counters */
extern volatile uint32_t dbg_cat_rx_bytes;
extern volatile uint32_t dbg_cat_tx_bytes;
extern volatile uint32_t dbg_cat_fifo_drop;
extern volatile uint32_t dbg_cat_parse_calls;

/* Protocol counters */
extern volatile uint32_t dbg_cat_unknown_cmds;
extern volatile uint32_t dbg_cat_partial_timeouts;
extern volatile uint32_t dbg_cat_max_cmd_len;
extern volatile uint32_t dbg_cat_parse_latency_us;

/* Last-command snapshots */
extern char dbg_cat_last_cmd [CAT_BUF_SIZE];
extern char dbg_cat_last_resp[CAT_TX_BUF_SIZE];
extern char dbg_last_unknown_cmd[64];  /*!< opcode not in handler list        */
extern char dbg_last_malformed[64];    /*!< known opcode, bad parameters → ?; */
extern char dbg_last_timeout[64];      /*!< partial frame discarded by 200ms  */

/* Malformed-frame and blocked-update counters.
 *  dbg_cat_malformed_frames: ?; sent for a KNOWN opcode with bad parameters.
 *    Distinct from dbg_cat_unknown_cmds (unrecognised opcode entirely).
 *  dbg_cat_blocked_updates: SET arrived while the matching dirty flag was
 *    still pending in CSDR_Loop — two SETs within the 10 ms CAT gate.
 *    Incremented by the dirty-flag setter callbacks in csdr_app.c. */
extern volatile uint32_t dbg_cat_malformed_frames;
extern volatile uint32_t dbg_cat_blocked_updates;

/* Flat crash-forensics — single-variable watch in debugger */
extern char             dbg_last_opcode[3];
extern volatile uint8_t dbg_last_status;    /*!< CAT_ParseStatus_t of last txn */
extern volatile uint8_t dbg_last_cmd_len;
extern volatile uint8_t dbg_last_resp_len;

/* Response semantic validation error counters */
extern volatile uint32_t dbg_resp_no_semi;
extern volatile uint32_t dbg_resp_nonprint;

/* Forensic ring buffer (last 16 transactions) and opcode stats */
extern CAT_TxnRing_t  dbg_txn;
extern CAT_OpStat_t   dbg_opc[COPI_COUNT];

/* Opcode enable bits for dbg_cat_opc_mask — kept for debugger compatibility.
 * NR/NB/SH/SL/AG/SQ/GT/TS/IS handlers are now unconditional fixed stubs;
 * the mask bits below no longer gate their behaviour. */
#define CAT_OPC_RG   (1U << 0)  /*!< (reserved)                     */
#define CAT_OPC_PA   (1U << 1)  /*!< (reserved)                     */
#define CAT_OPC_PC   (1U << 2)  /*!< (reserved)                     */
#define CAT_OPC_NB   (1U << 3)  /*!< (reserved — NB is fixed stub)  */
#define CAT_OPC_BC   (1U << 4)  /*!< (reserved)                     */
#define CAT_OPC_NR   (1U << 5)  /*!< (reserved — NR is fixed stub)  */
#define CAT_OPC_SH   (1U << 6)  /*!< (reserved — SH is fixed stub)  */
#define CAT_OPC_SL   (1U << 7)  /*!< (reserved — SL is fixed stub)  */
#define CAT_OPC_EX   (1U << 8)  /*!< (reserved)                     */
#define CAT_OPC_ALL  0x1FFU
extern volatile uint32_t dbg_cat_opc_mask; /*!< not used by any active handler */

/* Transmit-payload snapshot — view in debugger Memory window as hex */
#define CAT_TX_RAW_SIZE 64U
extern uint8_t           dbg_last_tx_raw[CAT_TX_RAW_SIZE]; /*!< exact bytes of last CDC TX        */
extern volatile uint8_t  dbg_last_tx_n;                     /*!< valid byte count in dbg_last_tx_raw */
extern volatile uint32_t dbg_resp_null_drop;                 /*!< responses blocked: embedded NUL or non-ASCII */

/* Transport-race diagnostics — flrig stress testing
 *  dbg_cdc_busy_skips:    FlushTX deferred: CDC IN busy, out[] not overwritten.
 *                         Non-zero = IsBusy guard firing (data NOT dropped, retried).
 *                         Reset to 0 on each CAT_Init (USB reconnect).
 *  dbg_fifo_high_water:   Peak s_tx_fifo fill level in bytes (max 511).
 *                         Watch: if approaching 512 under flrig, increase FIFO.
 *                         Reset to 0 on each CAT_Init (USB reconnect).
 *  dbg_cat_pending_bytes: Live bytes currently queued in s_tx_fifo.
 *                         Non-zero = data waiting to be flushed. Healthy: ≤64 bytes.
 *  dbg_process_reenters:  CAT_Process reentrancy detected. Must remain 0. */
extern volatile uint32_t dbg_cdc_busy_skips;
extern volatile uint32_t dbg_fifo_high_water;
extern volatile uint32_t dbg_cat_pending_bytes;
extern volatile uint32_t dbg_process_reenters;
extern volatile uint32_t dbg_cdc_stuck_timeout; /*!< IsBusy held > 10ms — stall event count */

/* Memory corruption guards.
 *
 * Transport-counter bracket (0xAAAA/0xBBBB):
 *  dbg_guard_pre must always read 0xAAAAAAAA after CAT_Init.
 *  dbg_guard_post must always read 0xBBBBBBBB after CAT_Init.
 *
 * TX FIFO bracket (0xCCCC/0xDDDD):
 *  dbg_fifo_guard_pre  must always read 0xCCCCCCCC after CAT_Init.
 *  dbg_fifo_guard_post must always read 0xDDDDDDDD after CAT_Init.
 *  These bracket s_tx_fifo[512] in .bss — any OOB write into the FIFO's
 *  memory region that doesn't go through the FIFO API will show here.
 *
 * dbg_guard_status bit map (checked every CAT_FlushTX call, ≈1 ms):
 *   bit 0 = transport pre  (0xAAAAAAAA) corrupted
 *   bit 1 = transport post (0xBBBBBBBB) corrupted
 *   bit 2 = FIFO pre       (0xCCCCCCCC) corrupted
 *   bit 3 = FIFO post      (0xDDDDDDDD) corrupted
 *
 * dbg_guard_tripped_ms: HAL_GetTick() at first corruption detection.
 *   0 = never tripped.  Non-zero = timestamp of first guard failure.
 *
 * dbg_cat_in_process: 1 while CAT_Process body is executing, else 0.
 *   If stuck at 1 between calls, a fault prevented the cleanup line. */
extern volatile uint32_t dbg_guard_pre;
extern volatile uint32_t dbg_guard_post;
extern volatile uint32_t dbg_guard_status;
extern volatile uint32_t dbg_guard_tripped_ms;
extern volatile uint32_t dbg_fifo_guard_pre;
extern volatile uint32_t dbg_fifo_guard_post;
extern volatile uint8_t  dbg_cat_in_process;

/* Live CAT traffic snapshot — updated on every RX packet and every TX enqueue.
 *  dbg_last_cat_rx: last raw bytes received from host (NUL-terminated).
 *  dbg_last_cat_tx: last response string enqueued into TX FIFO (NUL-terminated).
 *  dbg_rx_count / dbg_tx_count: monotonic counters, one per packet / response. */
extern volatile char     dbg_last_cat_rx[64];
extern volatile char     dbg_last_cat_tx[64];
extern volatile uint32_t dbg_rx_count;
extern volatile uint32_t dbg_tx_count;
extern volatile uint32_t dbg_cat_rx_null_bytes;

/* FIFO lifecycle snapshot — consistent state after every enqueue/flush.
 * dbg_fifo_op: last operation code ('E'=enqueue, 'F'=flush-sent, 'B'=busy-defer, 'X'=error).
 * dbg_fifo_head_snap / dbg_fifo_tail_snap: head and tail at last operation.
 * dbg_fifo_depth_snap: computed depth at last operation (must equal dbg_cat_pending_bytes).
 * dbg_fifo_state_error: incremented on any invariant violation.
 * dbg_fifo_bytes_last_enq: bytes written to FIFO in last cat_tx_enqueue call.
 * dbg_fifo_bytes_last_flush: bytes sent to CDC in last successful CAT_FlushTX call.
 * dbg_last_enqueue_ms / dbg_last_flush_ms: HAL_GetTick() at last enqueue / flush. */
extern volatile uint8_t  dbg_fifo_op;
extern volatile uint16_t dbg_fifo_head_snap;
extern volatile uint16_t dbg_fifo_tail_snap;
extern volatile uint16_t dbg_fifo_depth_snap;
extern volatile uint32_t dbg_fifo_state_error;
extern volatile uint16_t dbg_fifo_bytes_last_enq;
extern volatile uint16_t dbg_fifo_bytes_last_flush;
extern volatile uint32_t dbg_last_enqueue_ms;
extern volatile uint32_t dbg_last_flush_ms;

/* Transport serialization & pacing — see usb_cat.c for full description.
 *
 * CRITICAL FIX: CAT_FlushTX now sends exactly one CAT frame (stops at ';') per
 * USB transfer.  Previously up to 64 bytes were sent together, coalescing multiple
 * responses into one packet and causing flrig parser desynchronization.
 *
 * dbg_cat_tx_min_gap_ms:  minimum ms gap between consecutive CDC IN packets.
 *   Default 2.  Set to 0 to disable.  Watch dbg_pacing_skips to confirm it fires.
 * dbg_pacing_skips:       FlushTX calls rate-limited by pacing guard (data not lost).
 * dbg_frames_per_tx:      ';' count in the last USB TX packet — must be 1 after fix.
 * dbg_multi_frame_pkts:   cumulative packets with >1 CAT frame — must be 0 after fix.
 * dbg_cat_ai_push_disable: 1=suppress unsolicited AI IF push (default).
 *   Set to 0 to re-enable AI push once strict serialization is confirmed stable. */
extern volatile uint32_t dbg_cat_tx_min_gap_ms;
extern volatile uint32_t dbg_pacing_skips;
extern volatile uint32_t dbg_frames_per_tx;
extern volatile uint32_t dbg_multi_frame_pkts;
extern volatile uint8_t  dbg_cat_ai_push_disable;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Khởi tạo CAT driver với callbacks.
  */
void CAT_Init(CAT_Handle_t *cat, const CAT_Callbacks_t *cb);

/**
  * @brief  Nhận bytes từ USB CDC, parse và execute lệnh.
  *         Gọi từ USBD_CDC_Receive callback.
  * @param  cat   CAT handle
  * @param  data  Dữ liệu nhận từ USB
  * @param  len   Số byte
  */
void CAT_Receive(CAT_Handle_t *cat, const uint8_t *data, uint16_t len);

/**
  * @brief  Gọi trong main loop: kiểm tra thay đổi và gửi AI notifications.
  */
void CAT_Process(CAT_Handle_t *cat);
void CAT_FlushTX (CAT_Handle_t *cat);  /*!< Call from main loop to flush TX queue */

/**
  * @brief  Gửi chuỗi response qua USB CDC.
  *         Gọi nội bộ sau khi parse lệnh.
  */
void CAT_SendResponse(const char *resp);

/**
  * @brief  Tạo chuỗi IF (Information) response từ trạng thái SDR hiện tại.
  * @param  buf  Output buffer (≥100 byte)
  */
void CAT_BuildIF(CAT_Handle_t *cat, char *buf);

/**
  * @brief  Chuyển đổi mode SDR ↔ CAT mode code.
  */
uint8_t CAT_SDRModeToCat(uint8_t sdr_mode);
uint8_t CAT_CatModeToSDR(uint8_t cat_mode);

/* ── CAT feature capability flags ───────────────────────────────────────────
 * Goal: MINIMAL + STABLE > FULL EMULATION.
 * Stable set (real hardware backing): FA, IF, MD, TX/RX, TQ, RA, SM.
 * Everything else returns a fixed safe stub — no callbacks, no side-effects. */
#define CAT_HAS_DUAL_VFO    1   /* VS/FR/DC trigger real VFO swap + deferred LO retune  */
#define CAT_HAS_SPLIT       1   /* SP/FT route TX to inactive VFO via csdr_apply_tx     */
#define CAT_HAS_RIT         1   /* RT/RC/RU/RD update rit_hz + apply via DSP nco_if     */
#define CAT_HAS_CW          0   /* No CW keyer: KS stub 10 WPM                          */
#define CAT_HAS_MEMORY      0   /* No channel memories: MR/MW are ACK-only              */
#define CAT_HAS_AI_PUSH     1   /* AI unsolicited IF: fires on freq/mode/TX change       */
#define CAT_HAS_PREAMP      0   /* No hardware preamp: PA always PA0                    */
#define CAT_HAS_RF_GAIN     0   /* No RF gain control: RG always RG100                  */
#define CAT_HAS_BEAT_CANCEL 0   /* No beat canceller: BC always BC0                     */
#define CAT_HAS_NR          0   /* NR stub: NR0; fixed — no DSP NR through CAT path     */
#define CAT_HAS_NB          0   /* NB stub: NB0; fixed — no DSP NB through CAT path     */
#define CAT_HAS_VOLUME      1   /* AG real: live get/set_volume callbacks, 0-100 ↔ 0-255 */
#define CAT_HAS_BW          1   /* FW/SH: live BW read/write via get_bw/set_bw callbacks */
#define CAT_HAS_SQUELCH     0   /* SQ stub: SQ0000; fixed — squelch not in minimal set  */
#define CAT_HAS_AGC_CTRL    0   /* GT stub: GT00; fixed — AGC speed not in minimal set  */

#ifdef __cplusplus
}
#endif
#endif /* __USB_CAT_H */
