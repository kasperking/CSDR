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
  cat->vfo_b_freq  = 7100000UL;
  cat->vfo_b_mode  = CAT_MODE_USB;
  cat->active_vfo  = 0U;
  cat->split_on    = false;
  cat->initialized = true;
  s_rx_head = 0U; s_rx_tail = 0U;
  s_tx_head = 0U; s_tx_tail = 0U;
}

void CAT_SendResponse(const char *resp) { tx_enqueue(resp); }

void CAT_BuildIF(CAT_Handle_t *cat, char *buf)
{
  uint32_t f;
  uint8_t  m_cat;
  if (cat->active_vfo == 1U) {
    f     = cat->vfo_b_freq;
    m_cat = cat->vfo_b_mode;
  } else {
    uint8_t sdr_m = cat->cb.get_mode ? cat->cb.get_mode() : CAT_MODE_USB;
    f     = cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL;
    m_cat = CAT_SDRModeToCat(sdr_m);
  }
  bool    tx  = cat->cb.get_tx ? cat->cb.get_tx() : false;
  int32_t rit = (cat->rit_on && cat->cb.get_rit_hz) ? cat->cb.get_rit_hz() : 0;

  char *p = buf;
  p = cat_str(p, "IF");
  p = cat_u32(p, f, 11);               /* P1 : freq 11 digits      */
  p = cat_str(p, "00000");             /* P2 : step (fixed 00000)  */
  *p++ = (rit >= 0) ? '+' : '-';       /* P3 : RIT sign            */
  p = cat_u32(p, (uint32_t)(rit >= 0 ? rit : -rit), 4); /* P3: 4-digit RIT → info[18..22] */
  *p++ = cat->rit_on ? '1' : '0';      /* P4 : RIT on/off  → info[23] */
  *p++ = '0';                          /* P5 : XIT off     → info[24] */
  *p++ = '0'; *p++ = '0'; *p++ = '0'; /* P6 : mem ch 000  → info[25..27] */
  *p++ = tx ? '1' : '0';              /* P7 : TX state    → info[28] */
  *p++ = (char)('0' + m_cat);          /* P8 : mode        → info[29] */
  *p++ = (char)('0' + cat->active_vfo);/* P9 : VFO select  → info[30]: 0=A,1=B */
  *p++ = '0'; *p++ = '0';             /* P10: scan 00     → info[31..32] */
  *p++ = cat->split_on ? '1' : '0';   /* P11: split       → info[33] */
  *p++ = '0';                          /* P12: tone 0      → info[34] */
  *p++ = '0'; *p++ = '0';             /* P13: CTCSS no 00 → info[35..36] */
  *p++ = '0'; *p++ = '0';             /* P14: CTCSS tone  → info[37..38] */
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
    case CAT_MODE_LSB:            return 3U;
    case CAT_MODE_USB:            return 2U;
    case CAT_MODE_CW:
    case CAT_MODE_CWR:            return 4U;
    case CAT_MODE_FM:             return 1U;
    case CAT_MODE_AM:             return 0U;
    case CAT_MODE_FSK:
    case CAT_MODE_FSKR:           return 2U;  /* RTTY/PKT → USB for SDR */
    default:                      return 2U;
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

  /* FB: VFO B frequency (independent storage, no hardware side-effect) */
  else if (cmd[0]=='F' && cmd[1]=='B') {
    if (len == 2U) {
      char *p = resp; p = cat_str(p, "FB");
      p = cat_u32(p, cat->vfo_b_freq, 11); *p++ = ';'; *p = 0;
    } else if (len == 13U) {
      cat->vfo_b_freq = cat_parse_u(&cmd[2], 11U);
    } else { tx_enqueue("?;"); }
  }

  /* MD: Mode (VFO A/B aware) */
  else if (cmd[0]=='M' && cmd[1]=='D') {
    if (len == 2U) {
      uint8_t cm = (cat->active_vfo == 1U)
                   ? cat->vfo_b_mode
                   : (cat->cb.get_mode ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB);
      char *p = resp; p = cat_str(p, "MD"); *p++ = (char)('0' + cm); *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      uint8_t cm = (uint8_t)(cmd[2] - '0');
      if (cat->active_vfo == 1U) {
        cat->vfo_b_mode = cm;
      } else {
        if (cat->cb.set_mode) cat->cb.set_mode(CAT_CatModeToSDR(cm));
        cat->last_mode = CAT_CatModeToSDR(cm);
      }
    } else { tx_enqueue("?;"); }
  }

  /* TX: flrig dùng "TX;" làm PTT-on fire-and-forget (log: "PTT S: TX; R: " rỗng).
   * KHÔNG trả response để tránh TX1; bị lẫn vào buffer khi flrig query IF; ngay sau.
   * TX0;/TX1; là SET tường minh, cũng không trả response.
   * last_tx KHÔNG cập nhật – CAT_Process phát hiện thay đổi và gửi IF. */
  else if (cmd[0]=='T' && cmd[1]=='X') {
    if (len == 2U) {
      if (cat->cb.set_tx) cat->cb.set_tx(true);
    } else if (len == 3U) {
      if (cat->cb.set_tx) cat->cb.set_tx(cmd[2] == '1');
    }
  }
  /* RX: SET tx=false, không trả response. */
  else if (cmd[0]=='R' && cmd[1]=='X') {
    if (cat->cb.set_tx) cat->cb.set_tx(false);
  }

  /* IF: information frame.
   * Sau khi trả response, sync last_* để CAT_Process không gửi thêm
   * một IF unsolicited trùng lặp trong cùng chu kỳ (tránh IF flood). */
  else if (cmd[0]=='I' && cmd[1]=='F') {
    CAT_BuildIF(cat, resp);
    cat->last_freq = cat->cb.get_freq ? cat->cb.get_freq() : 0U;
    cat->last_mode = cat->cb.get_mode ? cat->cb.get_mode() : 0xFFU;
    cat->last_tx   = cat->cb.get_tx   ? cat->cb.get_tx()   : false;
  }

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

  /* SM: S-meter (read-only) SM0; → SM0nnnn; (0000-0030) */
  else if (cmd[0]=='S' && cmd[1]=='M') {
    float db = cat->cb.get_signal_db ? cat->cb.get_signal_db() : -80.0f;
    int32_t su = (int32_t)((db + 73.0f) / 2.0f);
    if (su < 0)  su = 0;
    if (su > 30) su = 30;
    char *p = resp; p = cat_str(p, "SM0"); p = cat_u32(p, (uint32_t)su, 4U); *p++ = ';'; *p = 0;
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

  /* FT: TX VFO  FT;→FT0/FT1  FT0;/FT1; clears/sets split
   * FR: RX VFO  FR;→FR0      FR1; accepted (ACK, RX always VFO A in HW) */
  else if (cmd[0]=='F' && cmd[1]=='T') {
    if (len == 2U) {
      char *p = resp; *p++ = 'F'; *p++ = 'T';
      *p++ = cat->split_on ? '1' : '0'; *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      cat->split_on = (cmd[2] == '1');
    }
  }
  else if (cmd[0]=='F' && cmd[1]=='R') {
    if (len == 2U) {
      char *p = resp; p = cat_str(p, "FR0;"); *p = 0;
    }
    /* FR0;/FR1; set: ACK only – RX always on VFO A in hardware */
  }

  /* RT: RIT on/off  RT; → RT0;/RT1;  RT0;/RT1; → ACK */
  else if (cmd[0]=='R' && cmd[1]=='T') {
    if (len == 2U) {
      char *p = resp; p = cat_str(p, "RT"); *p++ = cat->rit_on?'1':'0'; *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      cat->rit_on = (cmd[2] == '1');
      if (!cat->rit_on && cat->cb.set_rit_hz) cat->cb.set_rit_hz(0);
    }
  }

  /* RC: Clear RIT offset (ACK) */
  else if (cmd[0]=='R' && cmd[1]=='C') {
    if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(0);
  }

  /* RU: RIT up  RUnnnnn; (5-digit Hz increment) */
  else if (cmd[0]=='R' && cmd[1]=='U') {
    int32_t cur = cat->cb.get_rit_hz ? cat->cb.get_rit_hz() : 0;
    int32_t delta = (len >= 7U) ? (int32_t)cat_parse_u(&cmd[2], 5U) : 10;
    if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(cur + delta);
  }

  /* RD: RIT down  RDnnnnn; */
  else if (cmd[0]=='R' && cmd[1]=='D') {
    int32_t cur = cat->cb.get_rit_hz ? cat->cb.get_rit_hz() : 0;
    int32_t delta = (len >= 7U) ? (int32_t)cat_parse_u(&cmd[2], 5U) : 10;
    if (cat->cb.set_rit_hz) cat->cb.set_rit_hz(cur - delta);
  }

  /* XT: XIT – always off */
  else if (cmd[0]=='X' && cmd[1]=='T') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "XT0;"); *p = 0; }
  }

  /* IS: IF shift  IS0; → IS0+nnnn;  IS0+nnnn;/IS0-nnnn; → ACK */
  else if (cmd[0]=='I' && cmd[1]=='S') {
    if (len == 2U || (len == 3U && cmd[2] == '0')) {
      char *p = resp; p = cat_str(p, "IS0");
      *p++ = (cat->if_shift >= 0) ? '+' : '-';
      p = cat_u32(p, (uint32_t)(cat->if_shift >= 0 ? cat->if_shift : -cat->if_shift), 4U);
      *p++ = ';'; *p = 0;
    } else if (len == 9U && cmd[2] == '0') {
      /* IS0±nnnn; */
      int16_t sh = (int16_t)cat_parse_u(&cmd[4], 4U);
      cat->if_shift = (cmd[3] == '-') ? (int16_t)(-sh) : sh;
    }
  }

  /* SP: Split  SP; → SP0;/SP1;  SP0;/SP1; → set split */
  else if (cmd[0]=='S' && cmd[1]=='P') {
    if (len == 2U) {
      char *p = resp; p = cat_str(p, "SP");
      *p++ = cat->split_on ? '1' : '0'; *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      cat->split_on = (cmd[2] == '1');
    }
  }

  /* TS: Tuning step  TS0; → TS0nn;  TS0nn; → set */
  else if (cmd[0]=='T' && cmd[1]=='S') {
    if (len == 3U && cmd[2] == '0') {
      uint32_t st = cat->cb.get_step ? cat->cb.get_step() : 100U;
      uint8_t idx = 6U; /* default 100Hz */
      if      (st >= 100000U) idx = 20U;
      else if (st >=  10000U) idx = 13U;
      else if (st >=   5000U) idx = 11U;
      else if (st >=   1000U) idx =  9U;
      else if (st >=    500U) idx =  8U;
      else if (st >=    100U) idx =  6U;
      else if (st >=     10U) idx =  3U;
      else                    idx =  0U;
      char *p = resp; p = cat_str(p, "TS0"); p = cat_u32(p, idx, 2U); *p++ = ';'; *p = 0;
    } else if (len == 5U && cmd[2] == '0') {
      static const uint32_t ts_tbl[] = {
        1,2,5,10,20,50,100,200,500,1000,2500,5000,6250,
        10000,12500,15000,20000,25000,30000,50000,100000
      };
      uint8_t idx = (uint8_t)cat_parse_u(&cmd[3], 2U);
      uint32_t st = (idx < 21U) ? ts_tbl[idx] : 100000U;
      if (cat->cb.set_step) cat->cb.set_step(st);
    }
  }

  /* VS: VFO select  VS;→VS0/VS1  VS0;/VS1;→switch active VFO */
  else if (cmd[0]=='V' && cmd[1]=='S') {
    if (len == 2U) {
      char *p = resp; p = cat_str(p, "VS");
      *p++ = (char)('0' + cat->active_vfo); *p++ = ';'; *p = 0;
    } else if (len == 3U) {
      uint8_t v = (uint8_t)(cmd[2] - '0');
      cat->active_vfo = (v <= 1U) ? v : 0U;
    }
  }

  /* SH: SSB hi-cut passband
   * TS-2000 index → Hz: 00=2000 01=2200 02=2400 03=2600 04=2800
   *                     05=3000 06=3400 07=4000 08=5000 09=thru
   * Query: SH0; → SH0nn;   SET: SH0nn; → apply BW */
  else if (cmd[0]=='S' && cmd[1]=='H') {
    static const uint32_t sh_tbl[10] = {
      2000,2200,2400,2600,2800,3000,3400,4000,5000,10000
    };
    if (len == 2U || (len == 3U && cmd[2] == '0')) {
      /* query: derive index from current bw_hz */
      uint32_t bw = cat->cb.get_bw ? cat->cb.get_bw() : 3000U;
      uint8_t idx = 9U;
      for (uint8_t i = 0U; i < 9U; i++) {
        if (bw <= (sh_tbl[i] + sh_tbl[i+1]) / 2U) { idx = i; break; }
      }
      char *p = resp; p = cat_str(p, "SH0"); p = cat_u32(p, idx, 2U); *p++ = ';'; *p = 0;
    } else if (len == 5U && cmd[2] == '0') {
      /* SET: SH0nn; → look up Hz and apply */
      uint8_t idx = (uint8_t)cat_parse_u(&cmd[3], 2U);
      if (idx > 9U) idx = 9U;
      if (cat->cb.set_bw) cat->cb.set_bw(sh_tbl[idx]);
    }
  }

  /* SL: SSB lo-cut passband — our FIR is a LPF from 0 Hz so lo-cut is
   * acknowledged but not applied (passband lower edge is always 0 Hz).
   * Query: SL0; → SL000;   SET: SL0nn; → ACK */
  else if (cmd[0]=='S' && cmd[1]=='L') {
    if (len == 2U || (len == 3U && cmd[2] == '0')) {
      char *p = resp; p = cat_str(p, "SL000;"); *p = 0;
    }
    /* len==5: SET SL0nn; → silently ACK, lo-cut not implemented */
  }

  /* MG: Mic gain  MG; → MG030; */
  else if (cmd[0]=='M' && cmd[1]=='G') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "MG030;"); *p = 0; }
  }

  /* KS: CW keyer speed  KS; → KS020; */
  else if (cmd[0]=='K' && cmd[1]=='S') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "KS020;"); *p = 0; }
  }

  /* TN: Tone/CTCSS number  TN; → TN00; */
  else if (cmd[0]=='T' && cmd[1]=='N') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "TN00;"); *p = 0; }
  }

  /* TO: Tone on/off  TO; → TO0; */
  else if (cmd[0]=='T' && cmd[1]=='O') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "TO0;"); *p = 0; }
  }

  /* RL: Noise limiter  RL; → RL00; */
  else if (cmd[0]=='R' && cmd[1]=='L') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "RL00;"); *p = 0; }
  }

  /* BC: Beat canceller  BC; → BC0; */
  else if (cmd[0]=='B' && cmd[1]=='C') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "BC0;"); *p = 0; }
  }

  /* SC: Scan  SC; → SC0; */
  else if (cmd[0]=='S' && cmd[1]=='C') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "SC0;"); *p = 0; }
  }

  /* LK: Lock  LK; → LK0; */
  else if (cmd[0]=='L' && cmd[1]=='K') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "LK0;"); *p = 0; }
  }

  /* VG: Voice gain  VG; → VG050; */
  else if (cmd[0]=='V' && cmd[1]=='G') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "VG050;"); *p = 0; }
  }

  /* MC: Memory channel  MC; → MC000; */
  else if (cmd[0]=='M' && cmd[1]=='C') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "MC000;"); *p = 0; }
  }

  /* SD: Semi-break-in delay  SD; → SD0290; */
  else if (cmd[0]=='S' && cmd[1]=='D') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "SD0290;"); *p = 0; }
  }

  /* QR: Quick RIT  QR; → QR0; */
  else if (cmd[0]=='Q' && cmd[1]=='R') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "QR0;"); *p = 0; }
  }

  /* BT: Beat tone  BT; → BT0; */
  else if (cmd[0]=='B' && cmd[1]=='T') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "BT0;"); *p = 0; }
  }

  /* AC: Antenna connector  AC; → AC000; */
  else if (cmd[0]=='A' && cmd[1]=='C') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "AC000;"); *p = 0; }
  }

  /* AN: Antenna number  AN; → AN0; */
  else if (cmd[0]=='A' && cmd[1]=='N') {
    if (len == 2U) { char *p = resp; p = cat_str(p, "AN0;"); *p = 0; }
  }

  /* ACK-only commands: no response expected */
  else if (cmd[0]=='P' && cmd[1]=='C') { /* TX power: ACK */ }
  else if (cmd[0]=='V' && cmd[1]=='X') { /* Voice TX: ACK */ }
  else if (cmd[0]=='V' && cmd[1]=='V') {
    /* VFO A=B: copy current VFO A state into VFO B shadow */
    cat->vfo_b_freq = cat->cb.get_freq ? cat->cb.get_freq() : 7100000UL;
    cat->vfo_b_mode = cat->cb.get_mode
                      ? CAT_SDRModeToCat(cat->cb.get_mode()) : CAT_MODE_USB;
  }
  else if (cmd[0]=='U' && cmd[1]=='P') { /* VFO up: ACK */ }
  else if (cmd[0]=='D' && cmd[1]=='N') { /* VFO down: ACK */ }
  else if (cmd[0]=='B' && cmd[1]=='U') { /* Band up: ACK */ }
  else if (cmd[0]=='B' && cmd[1]=='D') { /* Band down: ACK */ }
  else if (cmd[0]=='M' && cmd[1]=='W') { /* Memory write: ACK */ }
  else if (cmd[0]=='D' && cmd[1]=='S') { /* Display: ACK */ }
  else if (cmd[0]=='T' && cmd[1]=='C') { /* Tone code: ACK */ }
  else if (cmd[0]=='K' && cmd[1]=='Y') { /* CW keyer text: ACK */ }
  else if (cmd[0]=='M' && cmd[1]=='R') { /* Memory read: no memory */
    tx_enqueue("?;"); }

  /* Unknown / unimplemented */
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
