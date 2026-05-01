 /*
 * Features:
 *  - Kenwood TS-480 CAT protocol (ID020)
 *  - IF frame 38 chars (P15 shift byte added vs TS-2000)
 *  - SH/SL filter bandwidth (TS-480 standard), FW kept for legacy
 *  - g_cat exported, CAT_FlushTX exported
 */

#include "usb_cat.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* USB CDC TX */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

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
    while ((*dst++ = *src++) != '\0') {;}
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
 * Assumed SDR enum order: 0=AM, 1=FM, 2=USB, 3=LSB, 4=CW
 * ========================================================= */
uint8_t CAT_SDRModeToCat(uint8_t m)
{
    switch (m) {
        case 0U: return CAT_MODE_AM;
        case 1U: return CAT_MODE_FM;
        case 2U: return CAT_MODE_USB;
        case 3U: return CAT_MODE_LSB;
        case 4U: return CAT_MODE_CW;
        default: return CAT_MODE_USB;
    }
}

uint8_t CAT_CatModeToSDR(uint8_t m)
{
    switch (m) {
        case CAT_MODE_AM:  return 0U;
        case CAT_MODE_FM:  return 1U;
        case CAT_MODE_USB: return 2U;
        case CAT_MODE_LSB: return 3U;
        case CAT_MODE_CW:  return 4U;
        default:           return 2U;
    }
}

/* =========================================================
 * TX FIFO
 * ========================================================= */
#define CAT_TX_FIFO_SIZE 512U
static char     s_tx_fifo[CAT_TX_FIFO_SIZE];
static uint16_t s_tx_head = 0U;
static uint16_t s_tx_tail = 0U;

static void cat_tx_enqueue(const char *s)
{
    while (s && *s) {
        uint16_t next = (uint16_t)((s_tx_head + 1U) % CAT_TX_FIFO_SIZE);
        if (next == s_tx_tail) break;
        s_tx_fifo[s_tx_head] = *s++;
        s_tx_head = next;
    }
}

void CAT_SendResponse(const char *resp)
{
    cat_tx_enqueue(resp);
}

void CAT_FlushTX(CAT_Handle_t *cat)
{
    (void)cat;
    if (s_tx_head == s_tx_tail) return;

    uint8_t  out[64];
    uint16_t n    = 0U;
    uint16_t tail = s_tx_tail;   /* work with a local copy — don't commit yet */

    while (tail != s_tx_head && n < sizeof(out)) {
        out[n++] = (uint8_t)s_tx_fifo[tail];
        tail = (uint16_t)((tail + 1U) % CAT_TX_FIFO_SIZE);
    }

    /* Only advance the real tail when CDC actually accepts the transfer.
     * On USBD_BUSY the data stays in the FIFO and is retried next call. */
    if (CDC_Transmit_FS(out, n) == 0U) {  /* 0 = USBD_OK */
        s_tx_tail = tail;
    }
}

/* =========================================================
 * RX accumulation
 * ========================================================= */
void CAT_Receive(CAT_Handle_t *cat, const uint8_t *data, uint16_t len)
{
    if (!cat) return;

    for (uint16_t i = 0U; i < len; i++) {
        if (cat->rx_len < (CAT_BUF_SIZE - 1U)) {
            cat->rx_buf[cat->rx_len++] = (char)data[i];
        }
    }
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
    cat->rx_len = 0U;

    s_tx_head = s_tx_tail = 0U;
    s_tx_ready_ms = 0U;
}

/* =========================================================
 * IF builder — mcHF / TS-2000 standard
 * priv->info[] (0-based from content after "IF"):
 *  [0..10]  P1  freq (11)      [11..15] P2  step "     " (5 spaces, VFO mode)
 *  [16]     P3  '+' (sign)     [17..20] P4  "0000" (RIT, always zero)
 *  [21]     P5  '0' (RIT off)  [22]     P6  '0' (XIT off)
 *  [23..25] P7  mem "000" (3)  [26]     P8  TX
 *  [27]     P9  mode           [28]     P10 VFO ← hamlib kenwood_get_vfo_if
 *  [29]     P11 '0' scan       [30]     P12 split ← hamlib reads split here
 *  [31]     P13 '0' tone       [32..33] P14 "00" CTCSS
 *  [34]     ';'
 * Total: "IF"(2) + 34 payload + ";"(1) = 37 chars
 * ========================================================= */
void CAT_BuildIF(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;

    uint32_t f = (cat->active_vfo == 1U)
               ? cat->vfo_b_freq
               : (cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL);

    uint8_t mode = (cat->active_vfo == 1U)
                 ? cat->vfo_b_mode
                 : (cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB);

    bool tx = cat->cb.get_tx ? cat->cb.get_tx() : false;

    *p++ = 'I'; *p++ = 'F';

    p = cat_put_u32(p, f, 11U);                           /* [0..10]  P1  freq */
    *p++ = ' '; *p++ = ' '; *p++ = ' '; *p++ = ' '; *p++ = ' '; /* [11..15] P2  step (VFO mode) */
    *p++ = '+';                                           /* [16]     P3  sign */
    *p++ = '0'; *p++ = '0'; *p++ = '0'; *p++ = '0';     /* [17..20] P4  RIT = 0000 */
    *p++ = '0';                                           /* [21]     P5  RIT off */
    *p++ = '0';                                           /* [22]     P6  XIT off */
    *p++ = '0'; *p++ = '0'; *p++ = '0';                  /* [23..25] P7  mem */
    *p++ = tx ? '1' : '0';                               /* [26]     P8  TX */
    *p++ = (char)('0' + mode);                           /* [27]     P9  mode */
    *p++ = cat_vfo_digit(cat->active_vfo);               /* [28]     P10 VFO ← hamlib */
    *p++ = '0';                                          /* [29]     P11 scan */
    *p++ = cat->split_on ? '1' : '0';                   /* [30]     P12 split */
    *p++ = '0';                                          /* [31]     P13 tone */
    *p++ = '0'; *p++ = '0';                             /* [32..33] P14 CTCSS */
    *p++ = '0';                                          /* [34]     P15 shift (TS-480) */
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
               ? cat->vfo_b_freq
               : (cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL);

    *p++ = 'F'; *p++ = 'A';
    p = cat_put_u32(p, f, 11U);
    *p++ = ';';
    *p = '\0';
}

static void cat_build_FB(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    *p++ = 'F'; *p++ = 'B';
    p = cat_put_u32(p, cat->vfo_b_freq, 11U);
    *p++ = ';';
    *p = '\0';
}

static void cat_build_MD(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint8_t mode = (cat->active_vfo == 1U)
                 ? cat->vfo_b_mode
                 : (cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB);

    *p++ = 'M'; *p++ = 'D';
    *p++ = (char)('0' + mode);
    *p++ = ';';
    *p = '\0';
}

/* 0-255 raw ↔ 0-100 percent, bidirectional */
static uint8_t cat_to_pct(uint8_t raw)
{
    return (uint8_t)(((uint32_t)raw * 100U + 127U) / 255U);
}

static uint8_t cat_from_pct(uint32_t pct)
{
    if (pct >= 100U) return 255U;
    return (uint8_t)(pct * 255U / 100U);
}

static void cat_build_AG(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint8_t v = cat->cb.get_volume ? cat->cb.get_volume() : 50U; /* internal 0-100 */
    *p++ = 'A'; *p++ = 'G'; *p++ = '0';
    p = cat_put_u32(p, cat_from_pct(v), 3U); /* internal→CAT 0-255 */
    *p++ = ';';
    *p = '\0';
}

static void cat_build_NR(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    bool on = cat->cb.get_nr ? cat->cb.get_nr() : false;
    *p++ = 'N'; *p++ = 'R'; *p++ = on ? '1' : '0'; *p++ = ';'; *p = '\0';
}

static void cat_build_NB(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    bool on = cat->cb.get_nb ? cat->cb.get_nb() : false;
    *p++ = 'N'; *p++ = 'B'; *p++ = on ? '1' : '0'; *p++ = ';'; *p = '\0';
}

static void cat_build_FW(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint32_t bw = (cat->active_vfo == 1U)
                ? cat->vfo_b_bw
                : (cat->cb.get_bw ? cat->cb.get_bw() : 3000U);
    *p++ = 'F'; *p++ = 'W';
    p = cat_put_u32(p, bw, 4U);
    *p++ = ';';
    *p = '\0';
}

/* TS-480 SH high-cut table: index 00-11 → Hz */
static const uint32_t s_sh_tbl[12] = {
    1000U, 1200U, 1400U, 1600U, 1800U, 2000U,
    2200U, 2400U, 2600U, 2800U, 3000U, 3400U
};

static uint8_t cat_bw_to_sh(uint32_t bw)
{
    for (uint8_t i = 0U; i < 11U; i++) {
        if (bw <= s_sh_tbl[i]) return i;
    }
    return 11U;
}

static void cat_build_SH(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint32_t bw = (cat->active_vfo == 1U)
                ? cat->vfo_b_bw
                : (cat->cb.get_bw ? cat->cb.get_bw() : 3000U);
    uint8_t idx = cat_bw_to_sh(bw);
    *p++ = 'S'; *p++ = 'H';
    *p++ = (char)('0' + (idx / 10U));
    *p++ = (char)('0' + (idx % 10U));
    *p++ = ';';
    *p = '\0';
}

static void cat_build_SL(char *buf)
{
    buf[0] = 'S'; buf[1] = 'L'; buf[2] = '0'; buf[3] = '0'; buf[4] = ';'; buf[5] = '\0';
}

static void cat_build_SQ(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    uint8_t sq = cat->cb.get_squelch ? cat->cb.get_squelch() : 0U; /* internal 0-100 */
    *p++ = 'S'; *p++ = 'Q'; *p++ = '0';
    p = cat_put_u32(p, cat_from_pct(sq), 3U); /* internal→CAT 0-255 */
    *p++ = ';';
    *p = '\0';
}

static void cat_build_SM(CAT_Handle_t *cat, char *buf)
{
    char *p = buf;
    float db = cat->cb.get_signal_db ? cat->cb.get_signal_db() : -80.0f;
    int32_t su = (int32_t)((db + 73.0f) / 2.0f);
    if (su < 0) su = 0;
    if (su > 30) su = 30;

    *p++ = 'S'; *p++ = 'M'; *p++ = '0';
    p = cat_put_u32(p, (uint32_t)su, 4U);
    *p++ = ';';
    *p = '\0';
}

static void cat_build_BC(char *buf)
{
    buf[0] = 'B'; buf[1] = 'C'; buf[2] = '0'; buf[3] = ';'; buf[4] = '\0';
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
    resp[0] = '\0';

    /* FA — GET returns active VFO freq; SET is ACK-only, never touches active_vfo */
    if (cmd[0] == 'F' && cmd[1] == 'A') {
        if (cmd[2] == '\0') {
            cat_build_FA(cat, resp);
        } else {
            uint32_t f = cat_parse_u(&cmd[2], 11U);
            if (cat->active_vfo == 1U) {
                cat->vfo_b_freq = f;
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
            cat->vfo_b_freq = cat_parse_u(&cmd[2], 11U);
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
            } else if (cat->cb.set_mode) {
                cat->cb.set_mode(CAT_CatModeToSDR(m));
            }
            cat_build_MD(cat, resp);
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
        cat_copy(resp, "ID020;");
    }

    /* AI */
    else if (cmd[0] == 'A' && cmd[1] == 'I') {
        if (cmd[2] != '\0') {
            uint8_t lv = (uint8_t)(cmd[2] - '0');
            cat->ai_level = (lv <= 2U) ? lv : 0U;
        }
        resp[0] = 'A'; resp[1] = 'I'; resp[2] = (char)('0' + cat->ai_level); resp[3] = ';'; resp[4] = '\0';
    }

    /* AG — 0-100% ↔ 0-255 internal */
    else if (cmd[0] == 'A' && cmd[1] == 'G') {
        if (cmd[2] == '\0' || cmd[3] == '\0') {
            /* GET: AG; or AG0; */
            cat_build_AG(cat, resp);
        } else if (cmd[2] == '0') {
            /* SET: AG0xxx; — flrig sends 0-255, convert to internal 0-100 */
            uint32_t raw = cat_parse_u(&cmd[3], 3U);
            if (raw > 255U) raw = 255U;
            if (cat->cb.set_volume) cat->cb.set_volume(cat_to_pct((uint8_t)raw));
            cat_build_AG(cat, resp);
        } else {
            cat_copy(resp, "?;");
        }
    }

    /* NR — Kenwood standard: 0=off, 1=on */
    else if (cmd[0] == 'N' && cmd[1] == 'R') {
        if (cmd[2] == '\0') {
            cat_build_NR(cat, resp);
        } else {
            if (cat->cb.set_nr) cat->cb.set_nr(cmd[2] == '1');
            cat_build_NR(cat, resp);
        }
    }

    /* NB — Kenwood standard: 0=off, 1=on */
    else if (cmd[0] == 'N' && cmd[1] == 'B') {
        if (cmd[2] == '\0') {
            cat_build_NB(cat, resp);
        } else {
            if (cat->cb.set_nb) cat->cb.set_nb(cmd[2] == '1');
            cat_build_NB(cat, resp);
        }
    }

    /* FW */
    else if (cmd[0] == 'F' && cmd[1] == 'W') {
        if (cmd[2] == '\0') {
            cat_build_FW(cat, resp);
        } else {
            uint32_t bw = cat_parse_u(&cmd[2], 4U);
            if (bw < 100U) bw = 100U;
            if (bw > 9999U) bw = 9999U;
            if (cat->active_vfo == 1U) {
                cat->vfo_b_bw = bw;
            } else if (cat->cb.set_bw) {
                cat->cb.set_bw(bw);
            }
            cat_build_FW(cat, resp);
        }
    }

    /* SH — TS-480 IF high-cut */
    else if (cmd[0] == 'S' && cmd[1] == 'H') {
        if (cmd[2] == '\0') {
            cat_build_SH(cat, resp);
        } else {
            uint32_t idx = cat_parse_u(&cmd[2], 2U);
            if (idx > 11U) idx = 11U;
            uint32_t bw = s_sh_tbl[idx];
            if (cat->active_vfo == 1U) {
                cat->vfo_b_bw = bw;
            } else if (cat->cb.set_bw) {
                cat->cb.set_bw(bw);
            }
            cat_build_SH(cat, resp);
        }
    }

    /* SL — TS-480 IF low-cut (stub: always 0 Hz) */
    else if (cmd[0] == 'S' && cmd[1] == 'L') {
        cat_build_SL(resp);
    }

    /* SQ — 0-100% ↔ 0-255 internal */
    else if (cmd[0] == 'S' && cmd[1] == 'Q') {
        if (cmd[2] == '\0' || cmd[3] == '\0') {
            /* GET: SQ; or SQ0; */
            cat_build_SQ(cat, resp);
        } else if (cmd[2] == '0') {
            /* SET: SQ0xxx; — flrig sends 0-255, convert to internal 0-100 */
            uint32_t raw = cat_parse_u(&cmd[3], 3U);
            if (raw > 255U) raw = 255U;
            if (cat->cb.set_squelch) cat->cb.set_squelch(cat_to_pct((uint8_t)raw));
            cat_build_SQ(cat, resp);
        } else {
            cat_copy(resp, "?;");
        }
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
            resp[0] = 'R'; resp[1] = 'A';
            resp[2] = (char)('0' + ((lv > 3U ? 3U : lv) / 10U));
            resp[3] = (char)('0' + ((lv > 3U ? 3U : lv) % 10U));
            resp[4] = ';';
            resp[5] = '\0';
        } else {
            cat_copy(resp, "?;");
        }
    }

    /* GT */
    else if (cmd[0] == 'G' && cmd[1] == 'T') {
        if (cmd[2] == '\0') {
            bool fast = cat->cb.get_agc_fast ? cat->cb.get_agc_fast() : true;
            resp[0] = 'G'; resp[1] = 'T'; resp[2] = '0'; resp[3] = fast ? '0' : '1'; resp[4] = ';'; resp[5] = '\0';
        } else if (strlen(cmd) == 4U && cmd[2] == '0') {
            if (cat->cb.set_agc_fast) cat->cb.set_agc_fast(cmd[3] == '0');
            resp[0] = 'G'; resp[1] = 'T'; resp[2] = '0'; resp[3] = cmd[3]; resp[4] = ';'; resp[5] = '\0';
        } else {
            cat_copy(resp, "?;");
        }
    }

    /* PS */
    else if (cmd[0] == 'P' && cmd[1] == 'S') {
        cat_copy(resp, "PS1;");
    }

    /* PC - ACK only */
    else if (cmd[0] == 'P' && cmd[1] == 'C') {
        if (cmd[2] == '\0' || strlen(cmd) == 5U) {
            cat_copy(resp, "PC000;");
        } else {
            cat_copy(resp, "PC000;");
        }
    }

    /* BC — Beat Canceller (TS-2000); always stub off */
    else if (cmd[0] == 'B' && cmd[1] == 'C') {
        cat_build_BC(resp);
    }

    /* VS — GET returns active VFO; SET is ACK-only */
    else if (cmd[0] == 'V' && cmd[1] == 'S') {
        if (cmd[2] != '\0') {
            cat->active_vfo = cat_clamp_vfo((uint8_t)(cmd[2] - '0'));
        } else {
            cat_build_VS(cat, resp);
        }
    }

    /* DC — GET returns VFO+split routing; SET is ACK-only */
    else if (cmd[0] == 'D' && cmd[1] == 'C') {
        if (cmd[2] != '\0') {
            cat->active_vfo = cat_clamp_vfo((uint8_t)(cmd[2] - '0'));
            if (cmd[3] != '\0') cat->split_on = (cmd[3] == '1');
        } else {
            cat_build_DC(cat, resp);
        }
    }

    /* FR — GET returns current VFO; SET is ACK-only (Hamlib sends NULL expected) */
    else if (cmd[0] == 'F' && cmd[1] == 'R') {
        if (cmd[2] != '\0') {
            cat->active_vfo = cat_clamp_vfo((uint8_t)(cmd[2] - '0'));
        } else {
            resp[0] = 'F'; resp[1] = 'R';
            resp[2] = cat_vfo_digit(cat->active_vfo); resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* FT — GET returns split TX VFO; SET is ACK-only */
    else if (cmd[0] == 'F' && cmd[1] == 'T') {
        if (cmd[2] != '\0') {
            cat->split_on = (cmd[2] == '1');
        } else {
            resp[0] = 'F'; resp[1] = 'T';
            resp[2] = cat->split_on ? '1' : '0'; resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* RT — GET returns RIT state; SET is ACK-only */
    else if (cmd[0] == 'R' && cmd[1] == 'T') {
        if (cmd[2] != '\0') {
            cat->rit_on = (cmd[2] == '1');
            if (!cat->rit_on && cat->cb.set_rit_hz) cat->cb.set_rit_hz(0);
        } else {
            resp[0] = 'R'; resp[1] = 'T';
            resp[2] = cat->rit_on ? '1' : '0'; resp[3] = ';'; resp[4] = '\0';
        }
    }

    /* RC */
    else if (cmd[0] == 'R' && cmd[1] == 'C') {
        if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(0);
        cat_copy(resp, "RC;");
    }

    /* RU */
    else if (cmd[0] == 'R' && cmd[1] == 'U') {
        int32_t cur = cat->cb.get_rit_hz ? cat->cb.get_rit_hz() : 0;
        int32_t delta = (strlen(cmd) >= 7U) ? (int32_t)cat_parse_u(&cmd[2], 5U) : 10;
        if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(cur + delta);
        cat_copy(resp, "RU;");
    }

    /* RD */
    else if (cmd[0] == 'R' && cmd[1] == 'D') {
        int32_t cur = cat->cb.get_rit_hz ? cat->cb.get_rit_hz() : 0;
        int32_t delta = (strlen(cmd) >= 7U) ? (int32_t)cat_parse_u(&cmd[2], 5U) : 10;
        if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(cur - delta);
        cat_copy(resp, "RD;");
    }

    /* XT */
    else if (cmd[0] == 'X' && cmd[1] == 'T') {
        cat_copy(resp, "XT0;");
    }

    /* IS — IF shift: offset passband via NCO, BW unchanged */
    else if (cmd[0] == 'I' && cmd[1] == 'S') {
        if (cmd[2] == '\0' || cmd[3] == '\0') {
            /* GET */
            int16_t sh = cat->cb.get_if_shift
                       ? (int16_t)cat->cb.get_if_shift()
                       : cat->if_shift;
            cat->if_shift = sh;
            char *p = resp;
            *p++ = 'I'; *p++ = 'S'; *p++ = '0';
            *p++ = (sh >= 0) ? '+' : '-';
            p = cat_put_u32(p, (uint32_t)(sh >= 0 ? sh : -sh), 4U);
            *p++ = ';'; *p = '\0';
        } else if (cmd[2] == '0') {
            /* SET: IS0[+/-]nnnn; */
            int16_t sh = (int16_t)cat_parse_u(&cmd[4], 4U);
            cat->if_shift = (cmd[3] == '-') ? (int16_t)(-sh) : sh;
            if (cat->cb.set_if_shift) cat->cb.set_if_shift((int32_t)cat->if_shift);
            cat_copy(resp, "IS0;");
        } else {
            cat_copy(resp, "?;");
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

    /* TS */
    else if (cmd[0] == 'T' && cmd[1] == 'S') {
        if (cmd[2] == '\0' || cmd[2] == '0') {
            uint32_t st = cat->cb.get_step ? cat->cb.get_step() : 100U;
            uint8_t idx = 6U;
            if      (st >= 100000U) idx = 20U;
            else if (st >= 10000U)  idx = 13U;
            else if (st >= 5000U)   idx = 11U;
            else if (st >= 1000U)   idx = 9U;
            else if (st >= 500U)    idx = 8U;
            else if (st >= 100U)    idx = 6U;
            else if (st >= 10U)     idx = 3U;
            else                    idx = 0U;

            resp[0] = 'T'; resp[1] = 'S'; resp[2] = '0';
            resp[3] = (char)('0' + ((idx / 10U) % 10U));
            resp[4] = (char)('0' + (idx % 10U));
            resp[5] = ';'; resp[6] = '\0';
        } else if (strlen(cmd) == 5U && cmd[2] == '0') {
            static const uint32_t ts_tbl[] = {
                1U,2U,5U,10U,20U,50U,100U,200U,500U,1000U,2500U,5000U,6250U,
                10000U,12500U,15000U,20000U,25000U,30000U,50000U,100000U
            };
            uint32_t idx = cat_parse_u(&cmd[3], 2U);
            uint32_t st = (idx < 21U) ? ts_tbl[idx] : 100000U;
            if (cat->cb.set_step) cat->cb.set_step(st);
            resp[0] = 'T'; resp[1] = 'S'; resp[2] = '0';
            resp[3] = (char)('0' + ((idx / 10U) % 10U));
            resp[4] = (char)('0' + (idx % 10U));
            resp[5] = ';'; resp[6] = '\0';
        } else {
            cat_copy(resp, "?;");
        }
    }

    /* VV */
    else if (cmd[0] == 'V' && cmd[1] == 'V') {
        cat->vfo_b_freq = cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL;
        cat->vfo_b_mode = cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB;
        cat_copy(resp, "VV;");
    }

    /* ACK-only/common */
    else if (cmd[0] == 'U' && cmd[1] == 'P') { cat_copy(resp, "UP;"); }
    else if (cmd[0] == 'D' && cmd[1] == 'N') { cat_copy(resp, "DN;"); }
    else if (cmd[0] == 'B' && cmd[1] == 'U') { cat_copy(resp, "BU;"); }
    else if (cmd[0] == 'B' && cmd[1] == 'D') { cat_copy(resp, "BD;"); }
    else if (cmd[0] == 'M' && cmd[1] == 'W') { cat_copy(resp, "MW;"); }
    else if (cmd[0] == 'D' && cmd[1] == 'S') { cat_copy(resp, "DS;"); }
    else if (cmd[0] == 'T' && cmd[1] == 'C') { cat_copy(resp, "TC;"); }
    else if (cmd[0] == 'K' && cmd[1] == 'Y') { cat_copy(resp, "KY;"); }
    else if (cmd[0] == 'M' && cmd[1] == 'R') { cat_copy(resp, "?;"); }

    else {
        cat_copy(resp, "?;");
    }

}

/* =========================================================
 * CAT Process
 * ========================================================= */
void CAT_Process(CAT_Handle_t *cat)
{
    if (!cat || !cat->initialized) return;

    static char cmd[CAT_BUF_SIZE];
    static uint8_t len = 0U;
    bool skip_ai = false;

    while (cat->rx_len > 0U) {
        char c = cat->rx_buf[0];
        memmove(cat->rx_buf, cat->rx_buf + 1, --cat->rx_len);

        if (c == '\r' || c == '\n') continue;
        if (len < (CAT_BUF_SIZE - 1U)) cmd[len++] = c;

        if (c == ';') {
            cmd[len - 1U] = '\0';

            /* suppress AI IF notification after PTT commands */
            if ((cmd[0] == 'T' && cmd[1] == 'X') || (cmd[0] == 'R' && cmd[1] == 'X')) {
                skip_ai = true;
            }

            char resp[CAT_TX_BUF_SIZE];
            cat_exec(cat, cmd, resp);
            if (resp[0] != '\0') cat_tx_enqueue(resp);
            /* No per-command flush — collect all responses first */

            len = 0U;
        }
    }

    /* Flush all queued responses in one CDC transfer. */
    CAT_FlushTX(cat);

    /* AI unsolicited IF — only when nothing is pending in the TX FIFO.
     * Injecting here while responses are queued would interleave with
     * command-response pairs and break kenwood_transaction verification. */
    if (cat->ai_level > 0U && !skip_ai && (s_tx_head == s_tx_tail)) {
        uint32_t cur_freq = (cat->active_vfo == 1U)
                          ? cat->vfo_b_freq
                          : (cat->cb.get_freq ? cat->cb.get_freq() : 0U);
        uint8_t  cur_mode = (cat->active_vfo == 1U)
                          ? cat->vfo_b_mode
                          : (cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB);
        bool     cur_tx    = cat->cb.get_tx ? cat->cb.get_tx() : false;

        if (cur_freq  != cat->last_freq  ||
            cur_mode  != cat->last_mode  ||
            cur_tx    != cat->last_tx    ||
            cat->active_vfo != cat->last_vfo ||
            cat->split_on   != cat->last_split) {

            cat->last_freq  = cur_freq;
            cat->last_mode  = cur_mode;
            cat->last_tx    = cur_tx;
            cat->last_vfo   = cat->active_vfo;
            cat->last_split = cat->split_on;

            char if_buf[50];
            CAT_BuildIF(cat, if_buf);
            cat_tx_enqueue(if_buf);
            CAT_FlushTX(cat);
        }
    }
}
