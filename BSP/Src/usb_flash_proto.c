/**
 * @file  usb_flash_proto.c
 * @brief NOR flash binary protocol — frame assembler + command executor.
 */

#include "usb_flash_proto.h"
#include "w25q128.h"
#include <string.h>

/* ── CDC transmit helpers (avoid cross-directory includes) ──────────────── */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
extern uint8_t Composite_CDC_IsBusy(void);
extern W25Q_Handle_t g_flash;

/* ── Protocol constants ─────────────────────────────────────────────────── */
#define HDR_LEN   7U   /* magic(1) + cmd(1) + addr(3) + dlen(2) */

/* ── State machine ──────────────────────────────────────────────────────── */
typedef enum {
    FPS_IDLE    = 0,
    FPS_HDR     = 1,
    FPS_DATA    = 2,
    FPS_PENDING = 3,
} FP_State_t;

static FP_State_t  s_state;
static uint8_t     s_hdr[HDR_LEN];
static uint8_t     s_hdr_cnt;
static uint8_t     s_data[FP_MAX_DATA_LEN];
static uint16_t    s_data_cnt;
static uint16_t    s_data_len;

/* Response buffer: magic(1) + status(1) + rlen(2) + data[256] = 260 bytes */
static uint8_t  s_resp[4U + FP_MAX_DATA_LEN];
static uint16_t s_resp_len;

/* ── Diagnostics ────────────────────────────────────────────────────────── */
volatile uint32_t dbg_fp_rx_frames;
volatile uint32_t dbg_fp_tx_frames;
volatile uint32_t dbg_fp_err_frames;
volatile uint32_t dbg_fp_last_cmd;
volatile uint32_t dbg_fp_last_status;

/* ── Private helpers ────────────────────────────────────────────────────── */

static void build_resp(uint8_t status, const uint8_t *data, uint16_t rlen)
{
    s_resp[0] = FLASH_PROTO_MAGIC;
    s_resp[1] = status;
    s_resp[2] = (uint8_t)(rlen >> 8);
    s_resp[3] = (uint8_t)(rlen & 0xFFU);
    if (data != NULL && rlen > 0U) {
        memcpy(&s_resp[4], data, rlen);
    }
    s_resp_len = 4U + rlen;
}

static void reset_assembler(void)
{
    s_state    = FPS_IDLE;
    s_hdr_cnt  = 0U;
    s_data_cnt = 0U;
    s_data_len = 0U;
    s_resp_len = 0U;
}

/* ── Command executor ───────────────────────────────────────────────────── */

static void execute(void)
{
    uint8_t  cmd  = s_hdr[1];
    uint32_t addr = ((uint32_t)s_hdr[2] << 16) |
                    ((uint32_t)s_hdr[3] <<  8) |
                     (uint32_t)s_hdr[4];
    /* For READ, dlen in header = bytes to read; for WRITE, = payload size. */
    uint16_t dlen = ((uint16_t)s_hdr[5] << 8) | s_hdr[6];

    dbg_fp_last_cmd = cmd;
    dbg_fp_rx_frames++;

    HAL_StatusTypeDef r;

    switch (cmd) {

    /* ── READ ─────────────────────────────────────────────────────────── */
    case FP_CMD_READ:
        if (dlen == 0U || dlen > FP_MAX_DATA_LEN) {
            build_resp(FP_STATUS_ERR_LEN, NULL, 0U);
            break;
        }
        if ((uint32_t)addr + dlen > W25Q_TOTAL_SIZE) {
            build_resp(FP_STATUS_ERR_ADDR, NULL, 0U);
            break;
        }
        /* Read directly into response payload slot */
        r = W25Q_Read(&g_flash, addr, &s_resp[4], dlen);
        if (r != HAL_OK) {
            build_resp(FP_STATUS_ERR_FLASH, NULL, 0U);
        } else {
            s_resp[0] = FLASH_PROTO_MAGIC;
            s_resp[1] = FP_STATUS_OK;
            s_resp[2] = (uint8_t)(dlen >> 8);
            s_resp[3] = (uint8_t)(dlen & 0xFFU);
            s_resp_len = 4U + dlen;
        }
        break;

    /* ── WRITE ────────────────────────────────────────────────────────── */
    case FP_CMD_WRITE:
        if (dlen == 0U || dlen > FP_MAX_DATA_LEN) {
            build_resp(FP_STATUS_ERR_LEN, NULL, 0U);
            break;
        }
        if ((uint32_t)addr + dlen > W25Q_TOTAL_SIZE) {
            build_resp(FP_STATUS_ERR_ADDR, NULL, 0U);
            break;
        }
        r = W25Q_Write(&g_flash, addr, s_data, dlen);
        build_resp((r == HAL_OK) ? FP_STATUS_OK : FP_STATUS_ERR_FLASH, NULL, 0U);
        break;

    /* ── SECTOR ERASE (4 KB) ──────────────────────────────────────────── */
    case FP_CMD_SECTOR_ERASE:
        if ((addr % W25Q_SECTOR_SIZE) != 0U) {
            build_resp(FP_STATUS_ERR_ADDR, NULL, 0U);
            break;
        }
        if (addr >= W25Q_TOTAL_SIZE) {
            build_resp(FP_STATUS_ERR_ADDR, NULL, 0U);
            break;
        }
        r = W25Q_SectorErase(&g_flash, addr);
        build_resp((r == HAL_OK) ? FP_STATUS_OK : FP_STATUS_ERR_FLASH, NULL, 0U);
        break;

    /* ── CHIP ID ──────────────────────────────────────────────────────── */
    case FP_CMD_CHIP_ID: {
        uint32_t id = g_flash.jedec_id;
        uint8_t  id_buf[4] = {
            (uint8_t)(id >> 24),
            (uint8_t)(id >> 16),
            (uint8_t)(id >>  8),
            (uint8_t)(id      ),
        };
        build_resp(FP_STATUS_OK, id_buf, 4U);
        break;
    }

    /* ── BLOCK ERASE 64 KB ────────────────────────────────────────────── */
    case FP_CMD_BLOCK64_ERASE:
        if ((addr % W25Q_BLOCK64_SIZE) != 0U) {
            build_resp(FP_STATUS_ERR_ADDR, NULL, 0U);
            break;
        }
        if (addr >= W25Q_TOTAL_SIZE) {
            build_resp(FP_STATUS_ERR_ADDR, NULL, 0U);
            break;
        }
        r = W25Q_BlockErase64K(&g_flash, addr);
        build_resp((r == HAL_OK) ? FP_STATUS_OK : FP_STATUS_ERR_FLASH, NULL, 0U);
        break;

    default:
        build_resp(FP_STATUS_ERR_CMD, NULL, 0U);
        break;
    }

    dbg_fp_last_status = s_resp[1];
    if (s_resp[1] != FP_STATUS_OK) {
        dbg_fp_err_frames++;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void FlashProto_Init(void)
{
    reset_assembler();
    dbg_fp_rx_frames   = 0U;
    dbg_fp_tx_frames   = 0U;
    dbg_fp_err_frames  = 0U;
    dbg_fp_last_cmd    = 0U;
    dbg_fp_last_status = 0U;
}

bool FlashProto_IsActive(void)
{
    return s_state != FPS_IDLE;
}

void FlashProto_Receive(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++) {
        uint8_t b = data[i];

        switch (s_state) {

        case FPS_IDLE:
            if (b == FLASH_PROTO_MAGIC) {
                s_hdr[0]  = b;
                s_hdr_cnt = 1U;
                s_state   = FPS_HDR;
            }
            break;

        case FPS_HDR:
            s_hdr[s_hdr_cnt++] = b;
            if (s_hdr_cnt == HDR_LEN) {
                uint8_t cmd = s_hdr[1];
                s_data_len  = ((uint16_t)s_hdr[5] << 8) | s_hdr[6];
                s_data_cnt  = 0U;

                /* Only WRITE has a payload; everything else is header-only. */
                if (cmd == FP_CMD_WRITE && s_data_len > 0U) {
                    if (s_data_len > FP_MAX_DATA_LEN) {
                        reset_assembler(); /* reject oversized write */
                    } else {
                        s_state = FPS_DATA;
                    }
                } else {
                    s_state = FPS_PENDING;
                }
            }
            break;

        case FPS_DATA:
            s_data[s_data_cnt++] = b;
            if (s_data_cnt == s_data_len) {
                s_state = FPS_PENDING;
            }
            break;

        case FPS_PENDING:
            /* Discard bytes that arrive before the response is sent. */
            break;
        }
    }
}

void FlashProto_Process(void)
{
    if (s_state != FPS_PENDING) {
        return;
    }

    execute();

    /* Wait for CDC TX to clear (max 50 ms — safe for any pending CAT frame). */
    uint32_t t0 = HAL_GetTick();
    while (Composite_CDC_IsBusy() && (HAL_GetTick() - t0) < 50U) {}

    CDC_Transmit_FS(s_resp, s_resp_len);
    dbg_fp_tx_frames++;

    reset_assembler();
}
