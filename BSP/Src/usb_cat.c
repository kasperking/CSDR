/* USER CODE BEGIN Header */
/**
  * @file usb_cat.c
  * @brief USB CAT Control – Kenwood TS-2000 Protocol
  *
  *  Kiến trúc 2 tầng để tránh crash:
  *  
  *  CAT_Receive() ← gọi từ USB ISR (DataOut callback)
  *    → CHỈ copy raw bytes vào RX ring buffer
  *    → KHÔNG parse, KHÔNG gọi I2C/DSP/HAL
  *
  *  CAT_Process() ← gọi từ main loop mỗi vòng lặp
  *    → dequeue RX ring → parse → execute callbacks
  *    → callbacks (set_freq→I2C, set_mode→DSP) chạy ở main context
  *    → flush TX response qua CDC
  */
/* USER CODE END Header */

#include "usb_cat.h"
#include "stm32h7xx_hal.h"
#include <string.h>

CAT_Handle_t g_cat;
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);


/* ── Helper: format uint32 as 0-padded decimal, return end pointer ── */
static char *cat_u32(char *p, uint32_t v, uint8_t width)
{
  char tmp[12];
  uint8_t n = 0U;
  if (v == 0U) { tmp[n++] = '0'; }
  else {
    while (v > 0U && n < 12U) { tmp[n++] = (char)('0' + (v % 10U)); v /= 10U; }
  }
  while (n < width) { tmp[n++] = '0'; }
  while (n > 0U)    { *p++ = tmp[--n]; }
  return p;
}

static char *cat_str(char *p, const char *s) { while (*s) *p++ = *s++; return p; }

/* ── Parse fixed-length decimal number, no sign, no validation ── */
static uint32_t cat_parse_u(const char *s, uint8_t n)
{
  uint32_t v = 0U;
  for (uint8_t i = 0U; i < n; i++) {
    if (s[i] >= '0' && s[i] <= '9') { v = v * 10U + (uint32_t)(s[i] - '0'); }
  }
  return v;
}


/* ── RX ring: viết từ ISR, đọc từ main loop ──────────── */
#define RX_RING  256U
static uint8_t           s_rxring[RX_RING];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* ── TX buffer: main loop ghi + flush ────────────────── */
#define TX_BUF_SIZE  512U
static uint8_t  s_tx_buf[TX_BUF_SIZE];
static uint16_t s_tx_head = 0U;
static uint16_t s_tx_tail = 0U;

static void tx_enqueue(const char *str)
{
  while (str && *str) {
    uint16_t next = (uint16_t)((s_tx_head + 1U) % TX_BUF_SIZE);
    if (next == s_tx_tail) break;
    s_tx_buf[s_tx_head] = (uint8_t)*str++;
    s_tx_head = next;
  }
}

void CAT_FlushTX(CAT_Handle_t *cat)
{
  (void)cat;
  if (s_tx_tail == s_tx_head) return;
  static uint8_t chunk[TX_BUF_SIZE];
  uint16_t n = 0;
  while (s_tx_tail != s_tx_head && n < sizeof(chunk)-1U) {
    chunk[n++] = s_tx_buf[s_tx_tail];
    s_tx_tail = (uint16_t)((s_tx_tail+1U) % TX_BUF_SIZE);
  }
  for (uint8_t retry = 0; n && retry < 3U; retry++) {
    if (CDC_Transmit_FS(chunk, n) != 0x01U) break;
    HAL_Delay(1);
  }
}

/* ══════════════════════════════════════════════════════ */

void CAT_Init(CAT_Handle_t *cat, const CAT_Callbacks_t *cb)
{
  memset(cat, 0, sizeof(*cat));
  cat->cb          = *cb;
  cat->ai_level    = 0U;
  cat->last_freq   = 0U;
  cat->last_mode   = 0xFFU;
  cat->last_tx     = false;
  cat->initialized = true;
  s_rx_head = 0U; s_rx_tail = 0U;
  s_tx_head = 0U; s_tx_tail = 0U;
}

void CAT_SendResponse(const char *resp) { tx_enqueue(resp); }

void CAT_BuildIF(CAT_Handle_t *cat, char *buf)
{
  uint32_t f  = cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL;
  uint8_t  m  = cat->cb.get_mode ? cat->cb.get_mode() : CAT_MODE_USB;
  bool     tx = cat->cb.get_tx   ? cat->cb.get_tx()   : false;

  char *p = buf;
  p = cat_str(p, "IF");
  p = cat_u32(p, f, 11);
  p = cat_str(p, "     ");             /* 5 spaces                */
  p = cat_u32(p, 0U, 5);                /* RIT                     */
  *p++ = '0'; *p++ = '0'; *p++ = '0';   /* XIT/bank/memch flags    */
  p = cat_u32(p, 0U, 3);                /* step                    */
  *p++ = tx ? '1' : '0';                /* TX state                */
  *p++ = (char)('0' + CAT_SDRModeToCat(m)); /* mode                */
  *p++ = '0'; *p++ = '0'; *p++ = '0';   /* scan/split/tone flags   */
  *p++ = '0'; *p++ = '0';               /* tone / ctcss            */
  p = cat_u32(p, 0U, 2);                /* ctcss freq              */
  *p++ = '0';                           /* final flag              */
  *p++ = ';';
  *p   = '\0';
}

uint8_t CAT_SDRModeToCat(uint8_t sdr_mode)
{
  static const uint8_t map[] = {
    CAT_MODE_AM, CAT_MODE_FM, CAT_MODE_USB, CAT_MODE_LSB, CAT_MODE_CW
  };
  return (sdr_mode < 5U) ? map[sdr_mode] : CAT_MODE_USB;
}

uint8_t CAT_CatModeToSDR(uint8_t cat_mode)
{
  switch (cat_mode) {
    case CAT_MODE_LSB:           return 3U;
    case CAT_MODE_USB:           return 2U;
    case CAT_MODE_CW:
    case CAT_MODE_CWR:           return 4U;
    case CAT_MODE_FM:            return 1U;
    case CAT_MODE_AM:            return 0U;
    default:                     return 2U;
  }
}

/* ── cat_execute: MAIN LOOP ONLY ─────────────────────── */
static void cat_execute(CAT_Handle_t *cat, const char *cmd, uint16_t len)
{
  char resp[128];
  resp[0] = '\0';
  if (len < 2U) { tx_enqueue("?;"); return; }

  /* FA: VFO A frequency */
  if (cmd[0]=='F' && cmd[1]=='A') {
    if (len == 2U) {
      uint32_t f = cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL;
      char *p = resp; p = cat_str(p, "FA"); p = cat_u32(p, f, 11); *p++ = ';'; *p = 0;
    } else if (len == 13U) {
      uint32_t f = cat_parse_u(&cmd[2], 11U);
      if (cat->cb.set_freq) cat->cb.set_freq(f);
      cat->last_freq = f;
    } else { tx_enqueue("?;"); }
  }

  /* FB: VFO B (mirrors VFO A on single-VFO radio) */
  else if (cmd[0]=='F' && cmd[1]=='B') {
    if (len == 2U) {
      uint32_t f = cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL;
      char *p = resp; p = cat_str(p, "FB"); p = cat_u32(p, f, 11); *p++ = ';'; *p = 0;
    } else if (len == 13U) {
      uint32_t f = cat_parse_u(&cmd[2], 11U);
      if (cat->cb.set_freq) cat->cb.set_freq(f);
      cat->last_freq = f;
    }
  }

  /* MD: Mode */
  else if (cmd[0]=='M' && cmd[1]=='D') {
    if (len == 2U) {
      uint8_t cm = cat->cb.get_mode ?
                   CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB;
      char *p = resp; p = cat_str(p, "MD"); *p++ = (char)('0' + cm); *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      uint8_t cm = (uint8_t)(cmd[2] - '0');
      if (cat->cb.set_mode) cat->cb.set_mode(CAT_CatModeToSDR(cm));
      cat->last_mode = CAT_CatModeToSDR(cm);
    } else { tx_enqueue("?;"); }
  }

  /* TX: len==2 = QUERY (TS-2000 spec), returns current TX state.
   *     len==3 = SET (TX0; TX1;), activates TX mode. */
  else if (cmd[0]=='T' && cmd[1]=='X') {
    if (len == 2U) {
      bool tx = cat->cb.get_tx ? cat->cb.get_tx() : false;
      char *p = resp;
      p = cat_str(p, "TX"); *p++ = tx ? '1' : '0'; *p++ = ';'; *p = 0;
    } else {
      if (cat->cb.set_tx) cat->cb.set_tx(true);
      cat->last_tx = true;
      if (cat->ai_level > 0U) CAT_BuildIF(cat, resp);
    }
  }
  /* RX: always a SET command (no query form in TS-2000) */
  else if (cmd[0]=='R' && cmd[1]=='X') {
    if (cat->cb.set_tx) cat->cb.set_tx(false);
    cat->last_tx = false;
    if (cat->ai_level > 0U) CAT_BuildIF(cat, resp);
  }

  /* IF: information frame */
  else if (cmd[0]=='I' && cmd[1]=='F') { CAT_BuildIF(cat, resp); }

  /* AI: Auto Information  AI0=off  AI1=on  AI2=on (immediate IF + unsolicited) */
  else if (cmd[0]=='A' && cmd[1]=='I') {
    if (len == 2U) {
      char *p = resp;
      p = cat_str(p, "AI"); *p++ = (char)('0' + cat->ai_level); *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      uint8_t lv = (uint8_t)(cmd[2] - '0');
      cat->ai_level = (lv <= 2U) ? lv : 0U;
      /* Send AIx; echo immediately */
      char ai_echo[8];
      char *p = ai_echo;
      p = cat_str(p, "AI"); *p++ = (char)('0' + cat->ai_level); *p++ = ';'; *p = 0;
      tx_enqueue(ai_echo);
      /* If AI enabled, flrig expects an immediate IF frame for initial state */
      if (cat->ai_level > 0U) {
        CAT_BuildIF(cat, resp);
        cat->last_freq = cat->cb.get_freq ? cat->cb.get_freq() : 0U;
        cat->last_mode = cat->cb.get_mode ? cat->cb.get_mode() : 0xFFU;
        cat->last_tx   = cat->cb.get_tx   ? cat->cb.get_tx()   : false;
      }
      /* resp is empty if ai_level==0, IF frame if >0 — enqueued at bottom */
    }
  }

  /* AG: Audio Gain  AG0;=query  AG0nnn;=set (000-255) */
  else if (cmd[0]=='A' && cmd[1]=='G') {
    if (len == 3U && cmd[2] == '0') {
      uint8_t v = cat->cb.get_volume ? cat->cb.get_volume() : 127U;
      char *p = resp; p = cat_str(p, "AG0"); p = cat_u32(p, v, 3U); *p++ = ';'; *p = 0;
    } else if (len == 6U && cmd[2] == '0') {
      uint8_t v = (uint8_t)cat_parse_u(&cmd[3], 3U);
      if (cat->cb.set_volume) cat->cb.set_volume(v);
    }
  }

  /* NR: Noise Reduction  NR;=query  NR0/NR1=set */
  else if (cmd[0]=='N' && cmd[1]=='R') {
    if (len == 2U) {
      bool on = cat->cb.get_nr ? cat->cb.get_nr() : false;
      char *p = resp; p = cat_str(p, "NR"); *p++ = on ? '1' : '0'; *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      if (cat->cb.set_nr) cat->cb.set_nr(cmd[2] == '1');
    }
  }

  /* NB: Noise Blanker  NB;=query  NB0/NB1=set */
  else if (cmd[0]=='N' && cmd[1]=='B') {
    if (len == 2U) {
      bool on = cat->cb.get_nb ? cat->cb.get_nb() : false;
      char *p = resp; p = cat_str(p, "NB"); *p++ = on ? '1' : '0'; *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      if (cat->cb.set_nb) cat->cb.set_nb(cmd[2] == '1');
    }
  }

  /* FW: Filter Width (Hz)  FW;=query  FWnnnn;=set (4 digits, 0100-9999 Hz) */
  else if (cmd[0]=='F' && cmd[1]=='W') {
    if (len == 2U) {
      uint32_t bw = cat->cb.get_bw ? cat->cb.get_bw() : 3000U;
      char *p = resp; p = cat_str(p, "FW"); p = cat_u32(p, bw, 4U); *p++ = ';'; *p = 0;
    } else if (len == 6U) {
      uint32_t bw = cat_parse_u(&cmd[2], 4U);
      if (cat->cb.set_bw) cat->cb.set_bw(bw);
    }
  }

  /* GT: AGC Speed  GT;=query  GT00=fast  GT01=slow  GT02=off */
  else if (cmd[0]=='G' && cmd[1]=='T') {
    if (len == 2U) {
      bool fast = cat->cb.get_agc_fast ? cat->cb.get_agc_fast() : true;
      char *p = resp; p = cat_str(p, "GT0"); *p++ = fast ? '0' : '1'; *p++ = ';'; *p = 0;
    } else if (len == 4U && cmd[2] == '0') {
      if (cat->cb.set_agc_fast) cat->cb.set_agc_fast(cmd[3] == '0');
    }
  }

  /* SQ: Squelch  SQ0;=query  SQ0nnn;=set (000-255) */
  else if (cmd[0]=='S' && cmd[1]=='Q') {
    if (len == 3U && cmd[2] == '0') {
      uint8_t sq = cat->cb.get_squelch ? cat->cb.get_squelch() : 0U;
      char *p = resp; p = cat_str(p, "SQ0"); p = cat_u32(p, sq, 3U); *p++ = ';'; *p = 0;
    } else if (len == 6U && cmd[2] == '0') {
      uint8_t sq = (uint8_t)cat_parse_u(&cmd[3], 3U);
      if (cat->cb.set_squelch) cat->cb.set_squelch(sq);
    }
  }

  /* SM: S-meter (read-only) */
  else if (cmd[0]=='S' && cmd[1]=='M') {
    float db = cat->cb.get_signal_db ? cat->cb.get_signal_db() : -80.0f;
    int32_t su = (int32_t)((db + 73.0f) * 3.0f);
    if (su < 0) su = 0;
    if (su > 30) su = 30;
    char *p = resp; p = cat_str(p, "SM0"); p = cat_u32(p, (uint32_t)su * 100U, 5); *p++ = ';'; *p = 0;
  }

  /* RA: Attenuator */
  else if (cmd[0]=='R' && cmd[1]=='A') {
    if (len == 2U) {
      uint8_t att = cat->cb.get_att ? cat->cb.get_att() : 0U;
      uint8_t lv  = (att>=18U)?3U:(att>=12U)?2U:(att>=6U)?1U:0U;
      char *p = resp; p = cat_str(p, "RA"); p = cat_u32(p, lv, 2); *p++ = ';'; *p = 0;
    } else if (len == 4U) {
      uint8_t lv = (uint8_t)(cmd[2]-'0')*10U + (uint8_t)(cmd[3]-'0');
      if (cat->cb.set_att) cat->cb.set_att(lv > 3U ? 3U : lv);
    }
  }

  /* ID / PS */
  else if (cmd[0]=='I' && cmd[1]=='D') {
    char *p = resp; p = cat_str(p, "ID019;"); *p = 0;
  }
  else if (cmd[0]=='P' && cmd[1]=='S') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "PS1;"); *p = 0; }
  }

  /* ACK-only commands */
  else if (cmd[0]=='F' && (cmd[1]=='R' || cmd[1]=='T')) { /* ACK */ }
  else if (cmd[0]=='P' && cmd[1]=='C') { /* ACK */ }
  else if (cmd[0]=='V' && cmd[1]=='X') { /* ACK */ }

  /* Unknown */
  else { tx_enqueue("?;"); }

  if (resp[0]) tx_enqueue(resp);
}

/* ── CAT_Receive: ISR SAFE – copy only, no processing ── */
void CAT_Receive(CAT_Handle_t *cat, const uint8_t *data, uint16_t len)
{
  (void)cat;
  for (uint16_t i = 0; i < len; i++) {
    uint16_t next = (uint16_t)((s_rx_head + 1U) % RX_RING);
    if (next == s_rx_tail) break;  /* ring full: drop */
    s_rxring[s_rx_head] = data[i];
    s_rx_head = next;
  }
}

/* ── CAT_Process: MAIN LOOP – parse + execute + flush ── */
void CAT_Process(CAT_Handle_t *cat)
{
  if (!cat->initialized) return;

  /* 1. Dequeue RX, parse, execute (I2C/DSP calls happen here, safe) */
  while (s_rx_tail != s_rx_head) {
    char ch = (char)s_rxring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % RX_RING);

    if (ch == '\r' || ch == '\n') continue;
    if (cat->rx_len < CAT_BUF_SIZE - 1U)
      cat->rx_buf[cat->rx_len++] = ch;

    if (ch == ';') {
      cat->rx_buf[cat->rx_len] = '\0';
      cat_execute(cat, cat->rx_buf, (uint16_t)(cat->rx_len - 1U));
      cat->rx_len = 0U;
    }
  }

  /* 2. Auto-info (AI1 / AI2): send IF on freq, mode, or TX state change */
  if (cat->ai_level > 0U) {
    uint32_t f  = cat->cb.get_freq ? cat->cb.get_freq() : 0U;
    uint8_t  m  = cat->cb.get_mode ? cat->cb.get_mode() : 0xFFU;
    bool     tx = cat->cb.get_tx   ? cat->cb.get_tx()   : false;
    if (f != cat->last_freq || m != cat->last_mode || tx != cat->last_tx) {
      cat->last_freq = f;
      cat->last_mode = m;
      cat->last_tx   = tx;
      char buf[100];
      CAT_BuildIF(cat, buf);
      tx_enqueue(buf);
    }
  }

  /* 3. Flush TX → CDC */
  CAT_FlushTX(cat);
}
