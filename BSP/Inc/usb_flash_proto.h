/**
 * @file  usb_flash_proto.h
 * @brief NOR flash binary protocol multiplexed on the CAT CDC channel.
 *
 *  Any USB packet whose first byte is 0xFF is routed here instead of the
 *  CAT parser.  0xFF is never a valid ASCII CAT character so flrig/Hamlib
 *  will never send it — zero risk of collision.
 *
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │ FRAME FORMAT (host → device)                                         │
 *  │   Byte  0   : 0xFF  (magic)                                          │
 *  │   Byte  1   : CMD                                                    │
 *  │   Bytes 2-4 : ADDR  (big-endian, 24-bit)                             │
 *  │   Bytes 5-6 : DLEN  (big-endian) — payload bytes that follow (WRITE) │
 *  │              OR bytes to read (READ), OR 0 for other commands        │
 *  │   Bytes 7+  : DATA[DLEN]  (only for CMD_WRITE)                       │
 *  ├──────────────────────────────────────────────────────────────────────┤
 *  │ RESPONSE (device → host)                                             │
 *  │   Byte  0   : 0xFF  (magic)                                          │
 *  │   Byte  1   : STATUS                                                 │
 *  │   Bytes 2-3 : RLEN  (big-endian) — data bytes that follow            │
 *  │   Bytes 4+  : DATA[RLEN]                                             │
 *  ├──────────────────────────────────────────────────────────────────────┤
 *  │ COMMANDS                                                             │
 *  │  0x01 READ        addr dlen         → 0xFF 0x00 dlen data[dlen]      │
 *  │  0x02 WRITE       addr dlen data[dlen] → 0xFF 0x00 0x00 0x00         │
 *  │  0x03 SECTOR_ERASE addr 0x00 0x00   → 0xFF 0x00 0x00 0x00  (4 KB)   │
 *  │  0x04 CHIP_ID     0x00 0x00 0x00 0x00 0x00 → 0xFF 0x00 0x00 0x04 id[4]│
 *  │  0x05 BLOCK64_ERASE addr 0x00 0x00  → 0xFF 0x00 0x00 0x00  (64 KB)  │
 *  ├──────────────────────────────────────────────────────────────────────┤
 *  │ STATUS CODES                                                         │
 *  │  0x00 OK  0x01 ERR_FLASH  0x02 ERR_ADDR  0x03 ERR_LEN  0x04 ERR_CMD │
 *  ├──────────────────────────────────────────────────────────────────────┤
 *  │ CONSTRAINTS                                                          │
 *  │  • Max data per READ or WRITE = 256 bytes (one flash page)           │
 *  │  • WRITE does NOT auto-erase — caller must erase first               │
 *  │  • SECTOR_ERASE addr must be 4 KB aligned                            │
 *  │  • BLOCK64_ERASE addr must be 64 KB aligned                          │
 *  └──────────────────────────────────────────────────────────────────────┘
 */

#ifndef __USB_FLASH_PROTO_H
#define __USB_FLASH_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Magic & commands ───────────────────────────────────────────────────── */
#define FLASH_PROTO_MAGIC       0xFFU

#define FP_CMD_READ             0x01U
#define FP_CMD_WRITE            0x02U
#define FP_CMD_SECTOR_ERASE     0x03U
#define FP_CMD_CHIP_ID          0x04U
#define FP_CMD_BLOCK64_ERASE    0x05U

/* ── Status codes ───────────────────────────────────────────────────────── */
#define FP_STATUS_OK            0x00U
#define FP_STATUS_ERR_FLASH     0x01U
#define FP_STATUS_ERR_ADDR      0x02U
#define FP_STATUS_ERR_LEN       0x03U
#define FP_STATUS_ERR_CMD       0x04U

/* ── Payload limits ─────────────────────────────────────────────────────── */
#define FP_MAX_DATA_LEN         256U   /* one flash page — read & write     */

/* ── API ────────────────────────────────────────────────────────────────── */

/** Reset all state (call on USB reconnect alongside CAT_Init). */
void FlashProto_Init(void);

/**
 * Feed raw bytes from CDC_Receive_FS into the frame assembler.
 * Call this (instead of CAT_Receive) when the first byte is 0xFF
 * or when FlashProto_IsActive() is true.
 */
void FlashProto_Receive(const uint8_t *data, uint16_t len);

/**
 * Execute a pending command and send the response.
 * Call once per main-loop tick (same cadence as CAT_FlushTX).
 */
void FlashProto_Process(void);

/** Returns true while a frame is being assembled or awaiting execution. */
bool FlashProto_IsActive(void);

/* ── Diagnostics ────────────────────────────────────────────────────────── */
extern volatile uint32_t dbg_fp_rx_frames;   /*!< complete frames received  */
extern volatile uint32_t dbg_fp_tx_frames;   /*!< responses sent            */
extern volatile uint32_t dbg_fp_err_frames;  /*!< frames that returned error*/
extern volatile uint32_t dbg_fp_last_cmd;    /*!< last CMD byte executed    */
extern volatile uint32_t dbg_fp_last_status; /*!< last STATUS returned      */

#ifdef __cplusplus
}
#endif
#endif /* __USB_FLASH_PROTO_H */
