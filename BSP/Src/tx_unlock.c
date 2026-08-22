/* USER CODE BEGIN Header */
/**
  * @file  tx_unlock.c
  * @brief Out-of-band TX limiter + secret-code unlock with persistent trace.
  *        See tx_unlock.h for the policy / audit-trail description.
  */
/* USER CODE END Header */

#include "tx_unlock.h"
#include "bpf_lpf.h"       /* BPF_FreqToBand, BAND_FREQ_MIN/MAX */
#include "sdr_ui.h"        /* fonts, LCD helpers, ZONE_SPEC_Y, UI_BG, caches */
#include "lcd_render.h"
#include "lcd_bus_fmc.h"
#include "encoder.h"       /* Key_t API */
#include "input_scan.h"    /* g_pca9555_raw, PCA_BIT_* */
#include "rtc_clock.h"
#include "csdr_app.h"      /* CSDR_ProcessAudioPending */
#include "main.h"          /* ENC_SW_GPIO_Port / ENC_SW_Pin */
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ─────────────────────────────────────────────────────────────────────────
 * SECRET CODE — change this before deployment.
 * Compiled into firmware only; never displayed on screen nor sent over CAT.
 * Must be exactly UL_CODE_DIGITS decimal characters.
 * ───────────────────────────────────────────────────────────────────────── */
#define UL_CODE_DIGITS   6U
#ifndef TX_UNLOCK_SECRET
#define TX_UNLOCK_SECRET "731509"
#endif

extern TIM_HandleTypeDef htim4;   /* encoder timer (TIM4_CH1/CH2 = PD12/PD13) */

/* ── Live audit record + flash handle ─────────────────────────────────────── */
static TxUnlockLog_t   g_txul;            /* zeroed = locked, no history       */
static W25Q_Handle_t  *s_dev = NULL;

/* ── CRC32 (matches w25q.c crc32_simple) ──────────────────────────────────── */
static uint32_t ul_crc32(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0U; i < len; i++) {
    crc ^= (uint32_t)data[i];
    for (uint8_t b = 0U; b < 8U; b++)
      crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
  }
  return ~crc;
}

/* ── Flash persistence (blocking erase+program; rare, one-shot events) ─────── */
static void txul_flash_write(void)
{
  if (s_dev == NULL || !s_dev->present) return;
  g_txul.magic = TX_UNLOCK_MAGIC;
  g_txul.pad[0] = g_txul.pad[1] = g_txul.pad[2] = 0U;
  g_txul.crc32 = ul_crc32((const uint8_t *)&g_txul,
                          sizeof(g_txul) - sizeof(g_txul.crc32));
  if (W25Q_SectorErase(s_dev, FLASH_ADDR_TX_UNLOCK) != HAL_OK) return;
  (void)W25Q_Write(s_dev, FLASH_ADDR_TX_UNLOCK,
                   (const uint8_t *)&g_txul, sizeof(g_txul));
}

/* ════ Public non-UI API ══════════════════════════════════════════════════ */

void TxUnlock_Init(W25Q_Handle_t *dev)
{
  s_dev = dev;
  memset(&g_txul, 0, sizeof(g_txul));
  if (dev == NULL || !dev->present) return;

  TxUnlockLog_t tmp;
  if (W25Q_Read(dev, FLASH_ADDR_TX_UNLOCK, (uint8_t *)&tmp, sizeof(tmp)) == HAL_OK
      && tmp.magic == TX_UNLOCK_MAGIC
      && ul_crc32((const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc32)) == tmp.crc32) {
    memcpy(&g_txul, &tmp, sizeof(g_txul));
  }
}

bool TxUnlock_IsUnlocked(void) { return g_txul.unlocked != 0U; }

bool TxUnlock_FreqAllowed(uint32_t tx_hz)
{
  if (g_txul.unlocked) return true;
  return (BPF_FreqToBand(tx_hz) != 0xFFU);
}

void TxUnlock_NoteOobTx(void)
{
  if (g_txul.oob_tx_count != 0xFFFFFFFFUL) g_txul.oob_tx_count++;
}

const TxUnlockLog_t *TxUnlock_GetLog(void) { return &g_txul; }

void TxUnlock_PersistCounters(void) { txul_flash_write(); }

/* ════ Full-screen unlock/status UI ════════════════════════════════════════ */

#define LN  LCD_GetLineBuf()
#define UL_X          10U

#define UL_BG         0x0843U   /* dark blue-gray background      */
#define UL_BORDER     0x10A2U
#define UL_HDR_BG     0xF800U
#define UL_FG         0xFFFFU
#define UL_LBL        0xA514U
#define UL_HINT       0x5ACBU
#define UL_OK         0x07E0U   /* green — LOCKED (ham only)      */
#define UL_WARN       0xFD20U   /* amber — UNLOCKED              */
#define UL_BOX_BG     0x2104U
#define UL_CURSOR_BG  0xF800U   /* red — active digit           */
#define UL_DIGIT_FG   0xFFE0U   /* yellow                        */
#define UL_TOAST_BG   0x0400U

static inline uint16_t sw16(uint16_t c)
{ return (uint16_t)((c >> 8U) | (c << 8U)); }

static void push_ln(uint16_t y)
{ LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, LN, LCD_W); }

/* ── Encoder detent reader (same idiom as cal.c) ──────────────────────────── */
static uint32_t s_enc_last = 0U;
static int32_t enc_read_delta(void)
{
  uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);
  int32_t  d   = (int32_t)(cnt - s_enc_last);
  if (d >  2) { s_enc_last = cnt; return  1; }
  if (d < -2) { s_enc_last = cnt; return -1; }
  return 0;
}
static void enc_flush(void) { s_enc_last = __HAL_TIM_GET_COUNTER(&htim4); }

/* ── Render helpers ───────────────────────────────────────────────────────── */
static void row_header(const char *title, uint16_t *y)
{
  for (uint16_t fr = 0U; fr < 16U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, UL_X, (uint16_t)(LCD_W - 2U * UL_X),
                 (fr == 0U || fr == 15U) ? UL_BORDER : UL_HDR_BG);
    ln[UL_X]                 = sw16(UL_BORDER);
    ln[LCD_W - UL_X - 1U]    = sw16(UL_BORDER);
    if (fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      uint16_t tx = (uint16_t)(UL_X + ((LCD_W - 2U * UL_X) -
                    (uint16_t)strlen(title) * Font6x8.width) / 2U);
      LCD_LineStr(ln, tx, fr - 4U, title, &Font6x8, 0xFFFFU, UL_HDR_BG);
    }
    push_ln((*y)++);
  }
}

static void row_text(uint16_t y, uint8_t h, const char *s, uint16_t fg)
{
  for (uint16_t fr = 0U; fr < h; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UL_BG);
    if (s != NULL && s[0] != '\0' && fr < (uint16_t)Font6x8.height)
      LCD_LineStr(ln, UL_X, fr, s, &Font6x8, fg, UL_BG);
    push_ln((uint16_t)(y + fr));
  }
}

static void row_digits(uint16_t y, const uint8_t *d, uint8_t cursor)
{
  const uint16_t box_w = 22U, gap = 6U, boxh = 16U;
  uint16_t total = UL_CODE_DIGITS * box_w + (UL_CODE_DIGITS - 1U) * gap;
  uint16_t x0    = (uint16_t)((LCD_W - total) / 2U);

  for (uint16_t fr = 0U; fr < boxh; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UL_BG);
    bool edge = (fr == 0U || fr == boxh - 1U);
    for (uint8_t i = 0U; i < UL_CODE_DIGITS; i++) {
      uint16_t bx = (uint16_t)(x0 + i * (box_w + gap));
      uint16_t bg = (i == cursor) ? UL_CURSOR_BG : UL_BOX_BG;
      LCD_LineFill(ln, bx, box_w, edge ? UL_BORDER : bg);
      if (!edge) {
        ln[bx]              = sw16(UL_BORDER);
        ln[bx + box_w - 1U] = sw16(UL_BORDER);
        if (fr >= 3U && fr < 3U + (uint16_t)Font8x10.height) {
          char c[2] = { (char)('0' + (d[i] % 10U)), '\0' };
          uint16_t tx = (uint16_t)(bx + (box_w - Font8x10.width) / 2U);
          LCD_LineStrW(ln, tx, fr - 3U, c, &Font8x10,
                       (i == cursor) ? 0xFFFFU : UL_DIGIT_FG, bg);
        }
      }
    }
    push_ln((uint16_t)(y + fr));
  }
}

static void render_screen(const uint8_t *digits, uint8_t cursor)
{
  char buf[40];
  uint16_t y = (uint16_t)ZONE_SPEC_Y;
  bool unl = (g_txul.unlocked != 0U);

  row_header("TX BAND UNLOCK", &y);

  snprintf(buf, sizeof buf, "STATUS: %s",
           unl ? "UNLOCKED (OOB TX ON)" : "LOCKED (HAM BANDS ONLY)");
  row_text(y, 12U, buf, unl ? UL_WARN : UL_OK); y += 12U;

  snprintf(buf, sizeof buf, "Unlocks:%lu  Fails:%lu",
           (unsigned long)g_txul.unlock_count, (unsigned long)g_txul.fail_count);
  row_text(y, 12U, buf, UL_LBL); y += 12U;

  snprintf(buf, sizeof buf, "Out-of-band TX keys: %lu",
           (unsigned long)g_txul.oob_tx_count);
  row_text(y, 12U, buf, UL_LBL); y += 12U;

  if (g_txul.unlock_count == 0U)
    snprintf(buf, sizeof buf, "Last unlock: never");
  else if (g_txul.last_time_valid)
    snprintf(buf, sizeof buf, "Last unlock: %02u:%02u:%02u",
             g_txul.last_hh, g_txul.last_mm, g_txul.last_ss);
  else
    snprintf(buf, sizeof buf, "Last unlock: (clock not set)");
  row_text(y, 14U, buf, UL_LBL); y += 14U;

  if (!unl) {
    row_text(y, 13U, "ENTER SECRET CODE:", UL_FG); y += 13U;
    row_digits(y, digits, cursor);                 y += 18U;
    row_text(y, 10U, "Rotate=digit  F1=<  F2=>", UL_HINT); y += 11U;
    row_text(y, 10U, "ENC=SUBMIT   F4=EXIT",     UL_HINT); y += 11U;
  } else {
    row_text(y, 13U, "RE-LOCK OUT-OF-BAND TX?", UL_FG); y += 13U;
    row_text(y, 10U, "ENC=RE-LOCK   F4=KEEP UNLOCKED", UL_HINT); y += 11U;
  }
}

/* Toast: two centred lines, ~1.3 s, audio pumped throughout (no dead stall). */
static void toast(const char *l1, const char *l2)
{
  uint16_t y = (uint16_t)(ZONE_SPEC_Y + 44U);
  for (uint16_t fr = 0U; fr < 30U; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    bool edge = (fr == 0U || fr == 29U);
    LCD_LineFill(ln, UL_X, (uint16_t)(LCD_W - 2U * UL_X), edge ? UL_BORDER : UL_TOAST_BG);
    if (!edge) {
      ln[UL_X]              = sw16(UL_BORDER);
      ln[LCD_W - UL_X - 1U] = sw16(UL_BORDER);
      if (fr >= 4U && fr < 4U + (uint16_t)Font6x8.height && l1 && l1[0])
        LCD_LineStr(ln, (uint16_t)(UL_X + 8U), fr - 4U, l1, &Font6x8, 0xFFFFU, UL_TOAST_BG);
      if (fr >= 16U && fr < 16U + (uint16_t)Font6x8.height && l2 && l2[0])
        LCD_LineStr(ln, (uint16_t)(UL_X + 8U), fr - 16U, l2, &Font6x8, UL_DIGIT_FG, UL_TOAST_BG);
    }
    push_ln((uint16_t)(y + fr));
  }
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 1300U) CSDR_ProcessAudioPending();
}

static bool code_matches(const uint8_t *digits)
{
  static const char sec[] = TX_UNLOCK_SECRET;
  for (uint8_t i = 0U; i < UL_CODE_DIGITS; i++) {
    if (sec[i] == '\0') return false;                       /* secret too short */
    if ((char)('0' + (digits[i] % 10U)) != sec[i]) return false;
  }
  return sec[UL_CODE_DIGITS] == '\0';                       /* exactly N digits */
}

static void stamp_unlock_time(void)
{
  g_txul.last_uptime_s = HAL_GetTick() / 1000U;
  if (RTC_Clock_IsSet()) {
    RTC_Clock_GetTime(&g_txul.last_hh, &g_txul.last_mm, &g_txul.last_ss);
    g_txul.last_time_valid = 1U;
  } else {
    g_txul.last_time_valid = 0U;
  }
}

void TxUnlock_Run(void)
{
  uint8_t digits[UL_CODE_DIGITS] = {0};
  uint8_t cursor = 0U;

  Key_t k_enc = {0}, k_f1 = {0}, k_f2 = {0}, k_f4 = {0};
  Key_Init   (&k_enc, ENC_SW_GPIO_Port, ENC_SW_Pin);
  Key_InitPCA(&k_f1,  &g_pca9555_raw,  PCA_BIT_F1);
  Key_InitPCA(&k_f2,  &g_pca9555_raw,  PCA_BIT_F2);
  Key_InitPCA(&k_f4,  &g_pca9555_raw,  PCA_BIT_F4);
  /* The key press that opened this screen is likely still held — swallow it. */
  Key_Sync(&k_enc); Key_Sync(&k_f1); Key_Sync(&k_f2); Key_Sync(&k_f4);
  enc_flush();

  render_screen(digits, cursor);

  for (;;) {
    CSDR_ProcessAudioPending();               /* keep audio/USB pipeline alive */
    Key_Poll(&k_enc); Key_Poll(&k_f1); Key_Poll(&k_f2); Key_Poll(&k_f4);
    if (k_enc.state != KS_IDLE) enc_flush();

    if (g_txul.unlocked) {
      /* Already unlocked → offer a re-lock. */
      if (Key_Press(&k_enc)) {
        g_txul.unlocked = 0U;
        g_txul.relock_count++;
        txul_flash_write();
        toast("OUT-OF-BAND TX", "RE-LOCKED");
        break;
      }
      if (Key_Press(&k_f4)) break;            /* keep unlocked, exit */
      continue;
    }

    /* Locked → 6-digit code entry. */
    int32_t d = enc_read_delta();
    if (d != 0) {
      int32_t v = (int32_t)digits[cursor] + d;
      v %= 10; if (v < 0) v += 10;
      digits[cursor] = (uint8_t)v;
      render_screen(digits, cursor);
    }
    if (Key_PressOrRepeat(&k_f1)) {
      if (cursor > 0U) cursor--;
      render_screen(digits, cursor);
    }
    if (Key_PressOrRepeat(&k_f2)) {
      if (cursor < UL_CODE_DIGITS - 1U) cursor++;
      render_screen(digits, cursor);
    }
    if (Key_Press(&k_enc)) {
      if (code_matches(digits)) {
        g_txul.unlocked = 1U;
        g_txul.unlock_count++;
        stamp_unlock_time();
        txul_flash_write();
        toast("CODE ACCEPTED", "OUT-OF-BAND TX ENABLED");
        break;
      }
      g_txul.fail_count++;
      txul_flash_write();
      toast("WRONG CODE", "TX STAYS HAM-ONLY");
      memset(digits, 0, sizeof(digits));
      cursor = 0U;
      Key_Sync(&k_enc); Key_Sync(&k_f1); Key_Sync(&k_f2); Key_Sync(&k_f4);
      enc_flush();
      render_screen(digits, cursor);
    }
    if (Key_Press(&k_f4)) break;              /* cancel */
  }

  /* Full-screen overlay exit rule: drop partial-redraw caches + any CW text. */
  SDR_UI_InvalidateCaches();
  SDR_UI_ClearCWText();
}
