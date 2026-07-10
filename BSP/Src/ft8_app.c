/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ft8_app.c
  * @brief   Full-screen FT8 monitor + beacon app (see ft8_app.h)
  *
  *  Keys (list view):  F1 station setup · F2 arm/disarm CQ beacon ·
  *                     MENU/F4 exit.
  *  Keys (editor):     encoder = change char · ENC push = next position ·
  *                     F1 save · F4 cancel.
  *
  *  Beacon: transmits "CQ <call> <grid>" on every other 15 s slot (RX slots
  *  in between decode replies).  TX runs through CSDR_RequestTX → the full
  *  csdr_apply_tx chain, and the app loop keeps PA_Protect_Update /
  *  Analog_Update / Fan_Update alive (CSDR_Loop is not running while the
  *  app owns the radio); TX aborts the moment PA protection disallows it.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "ft8_app.h"
#include "ft8_mode.h"
#include "csdr_app.h"
#include "sdr_ui.h"
#include "encoder.h"
#include "input_scan.h"
#include "rtc_clock.h"
#include "runtime_diag.h"
#include "pa_protect.h"
#include "pa_overcurrent.h"
#include "fsdr_analog.h"

#include <string.h>
#include <stdio.h>

/* ── Layout ───────────────────────────────────────────────────────────────
 * Header (14 px) / decode list (10 px per row, Font6x8) / footer (12 px).
 * Sized from LCD_W/LCD_H so both ST7796 (480×320) and ST7789 (320×240) work. */
#define APP_HDR_H   14U
#define APP_FTR_H   12U
#define APP_ROW_H   10U
#define APP_LIST_Y  (APP_HDR_H + 2U)
#define APP_ROWS    ((uint16_t)((LCD_H - APP_LIST_Y - APP_FTR_H) / APP_ROW_H))
#define APP_FTR_Y   ((uint16_t)(LCD_H - APP_FTR_H))

#define APP_BG      UI_BG
#define APP_HDR_BG  0x0843U   /* dark blue-gray (same family as menu)       */
#define APP_TX_BG   0xF800U   /* red header while transmitting              */
#define APP_HDR_FG  0x07FFU   /* cyan                                       */
#define APP_TXT     0xFFFFU   /* white — normal decode line                 */
#define APP_CQ      0xFFE0U   /* yellow — lines starting with "CQ "         */
#define APP_DIM     UI_STATUS_LBL

/* ── Decode history — persists across app sessions (static) ─────────────── */
#define APP_HIST 40U
typedef struct {
  FT8_Decode_t d;
  uint8_t hh, mm, ss;   /* RTC time at slot finalise */
} ft8_hist_t;

static ft8_hist_t s_hist[APP_HIST];
static uint16_t   s_hist_len;   /* entries stored, saturates at APP_HIST    */
static uint16_t   s_hist_wr;    /* ring write index                         */
static uint32_t   s_total_dec;  /* lifetime decode counter (footer)         */

/* ── App state ───────────────────────────────────────────────────────────── */
typedef enum { APP_VIEW_LIST = 0, APP_VIEW_EDIT } app_view_t;

#define EDIT_CALL_LEN 11U
#define EDIT_GRID_LEN 4U
#define EDIT_LEN      (EDIT_CALL_LEN + EDIT_GRID_LEN)

static app_view_t s_view;
static char       s_edit[EDIT_LEN + 1U];  /* positions 0-10 call, 11-14 grid */
static uint8_t    s_cursor;
static bool       s_beacon_arm;
static bool       s_tx_active;
static bool       s_tx_parity;            /* true = TX on the next boundary  */
static char       s_cq_msg[32];           /* packed again before each CQ TX  */
static char       s_notice[28];
static uint32_t   s_notice_until;

/* List selection (encoder): offset from the newest entry, 0 = newest */
static uint16_t   s_sel_off;

/* ── QSO auto-sequencer (Phase 3) ─────────────────────────────────────────
 * Reactive machine keyed on the peer's last message to us:
 *   grid → send report · report → send R-report · R-report → send RR73 ·
 *   RRR/RR73 → send 73 · 73 → complete.  Each response transmits on the
 *   next slot boundary (opposite parity to the peer by construction, since
 *   we react to a decode from the slot that just finalised).  The current
 *   message is repeated while the peer stays silent, QSO_MAX_RETRY RX slots
 *   → timeout.  A decode addressed to us while the CQ beacon is armed
 *   starts a QSO automatically. */
#define QSO_MAX_RETRY 4U
static bool    s_qso_active;
static bool    s_qso_tx_due;         /* transmit s_qso_msg at next boundary */
static bool    s_qso_done_after_tx;  /* current message is the QSO closer   */
static uint8_t s_qso_retries;
static int8_t  s_qso_rpt;            /* their signal SNR as measured by us  */
static char    s_qso_peer[14];
static char    s_qso_msg[40];

static const char s_charset[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";
#define CHARSET_LEN (sizeof(s_charset) - 1U)

/* ── Row renderer: one APP_ROW_H-pixel text row, full width ──────────────── */
static void app_row_render(uint16_t y, const char *text, uint16_t fg, uint16_t bg)
{
  uint16_t *ln = LCD_GetLineBuf();
  for (uint16_t fr = 0U; fr < APP_ROW_H; fr++) {
    LCD_LineFill(ln, 0U, LCD_W, bg);
    if (fr >= 1U && (fr - 1U) < Font6x8.height && text[0] != '\0')
      LCD_LineStr(ln, 2U, (uint16_t)(fr - 1U), text, &Font6x8, fg, bg);
    LCD_PushWindow(0U, (uint16_t)(y + fr), (uint16_t)(LCD_W - 1U),
                   (uint16_t)(y + fr), ln, LCD_W);
  }
}

static void app_draw_header(const FT8_Status_t *st)
{
  static const char *state_str[5] = { "IDLE", "WAIT", "CAPT", "SRCH", "DEC " };
  char line[64];
  uint32_t f = g_sdr.freq_hz;
  uint16_t bg = s_tx_active ? APP_TX_BG : APP_HDR_BG;
  uint16_t fg = s_tx_active ? 0xFFFFU : APP_HDR_FG;
  if (!st->init_ok) {
    snprintf(line, sizeof(line), "FT8 INIT FAIL (fft mem)");
  } else {
    char tag[20] = "";
    if (s_qso_active)      snprintf(tag, sizeof(tag), "  >%s", s_qso_peer);
    else if (s_beacon_arm) snprintf(tag, sizeof(tag), "  CQ*");
    snprintf(line, sizeof(line), "FT8 %lu.%03lu.%03lu  %s %02lds%s",
             (unsigned long)(f / 1000000UL),
             (unsigned long)((f / 1000UL) % 1000UL),
             (unsigned long)(f % 1000UL),
             s_tx_active ? "TX  "
                         : state_str[(st->state <= FT8_STATE_DECODE) ? st->state : 0],
             (long)(FT8_GetCycleMs() / 1000),
             tag);
  }
  uint16_t *ln = LCD_GetLineBuf();
  for (uint16_t fr = 0U; fr < APP_HDR_H; fr++) {
    LCD_LineFill(ln, 0U, LCD_W, bg);
    if (fr >= 3U && (fr - 3U) < Font6x8.height)
      LCD_LineStr(ln, 2U, (uint16_t)(fr - 3U), line, &Font6x8, fg, bg);
    LCD_PushWindow(0U, fr, (uint16_t)(LCD_W - 1U), fr, ln, LCD_W);
  }
}

static void app_draw_footer(const FT8_Status_t *st)
{
  char line[80];
  if (s_notice[0] != '\0' && HAL_GetTick() < s_notice_until) {
    snprintf(line, sizeof(line), "%s", s_notice);
  } else {
    /* Diagnostic chain readout: blk → tap+STFT · cand/top → Costas sync ·
     * dec → LDPC+CRC · dt → slot-phase correction */
    int dt = (int)st->dt_offset_ms;
    int dta = (dt < 0) ? -dt : dt;
    snprintf(line, sizeof(line), "F1:CALL F2:CQ blk:%u cand:%u top:%d dec:%u/%lu dt%c%d.%02u",
             (unsigned)st->num_blocks, (unsigned)st->num_cand, (int)st->best_score,
             (unsigned)st->num_dec, (unsigned long)s_total_dec,
             (dt < 0) ? '-' : '+', dta / 1000, (unsigned)((dta % 1000) / 10));
  }
  app_row_render(APP_FTR_Y, line, APP_DIM, APP_BG);
  /* fill the leftover rows below the 10-px text row, if any */
  uint16_t *ln = LCD_GetLineBuf();
  for (uint16_t y = (uint16_t)(APP_FTR_Y + APP_ROW_H); y < LCD_H; y++) {
    LCD_LineFill(ln, 0U, LCD_W, APP_BG);
    LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, ln, LCD_W);
  }
}

static void app_notice(const char *text)
{
  snprintf(s_notice, sizeof(s_notice), "%s", text);
  s_notice_until = HAL_GetTick() + 2500U;
}

/* Redraw the whole list (newest at the bottom) under a fixed column-header
 * row.  Pumps the audio ring between rows so a full redraw never stalls the
 * DSP pipeline. */
#define APP_DATA_ROWS ((uint16_t)(APP_ROWS - 1U))   /* row 0 = column header */

static void app_draw_list(void)
{
  /* Column marker — aligned with the "%02u%02u%02u %+03d %c0.%u %4u  %s"
   * data format below (TIME cols 0-5, dB 7-9, DT 11-14, FREQ 16-19, MSG 22+) */
  app_row_render(APP_LIST_Y, "TIME    dB  DT  FREQ  MESSAGE", APP_HDR_FG, APP_BG);
  CSDR_ProcessAudioPending();

  uint16_t n = (s_hist_len < APP_DATA_ROWS) ? s_hist_len : APP_DATA_ROWS;
  if (s_sel_off >= n) s_sel_off = (n > 0U) ? (uint16_t)(n - 1U) : 0U;
  for (uint16_t r = 0U; r < APP_DATA_ROWS; r++) {
    char     line[64] = "";
    uint16_t fg = APP_TXT;
    uint16_t bg = APP_BG;
    /* bottom-align: row (APP_DATA_ROWS-n) .. APP_DATA_ROWS-1 hold the n entries */
    if (r >= (uint16_t)(APP_DATA_ROWS - n)) {
      uint16_t k   = (uint16_t)(r - (APP_DATA_ROWS - n));       /* 0..n-1   */
      uint16_t idx = (uint16_t)((s_hist_wr + APP_HIST - n + k) % APP_HIST);
      const ft8_hist_t *h = &s_hist[idx];
      int dta = (h->d.dt_ms < 0) ? -h->d.dt_ms : h->d.dt_ms;
      snprintf(line, sizeof(line), "%02u%02u%02u %+03d %c0.%u %4u  %s",
               (unsigned)h->hh, (unsigned)h->mm, (unsigned)h->ss,
               (int)h->d.snr_db,
               (h->d.dt_ms < 0) ? '-' : '+', (unsigned)((dta + 50) / 100),
               (unsigned)h->d.freq_hz, h->d.text);
      if (strncmp(h->d.text, "CQ ", 3U) == 0) fg = APP_CQ;
      /* selection bar: inverse video (bright cyan bg, black text) so it
       * stands out against the black list background */
      if (n > 0U && (uint16_t)(APP_DATA_ROWS - 1U - r) == s_sel_off) {
        bg = APP_HDR_FG;    /* bright cyan */
        fg = 0x0000U;
      }
    }
    app_row_render((uint16_t)(APP_LIST_Y + (1U + r) * APP_ROW_H), line, fg, bg);
    CSDR_ProcessAudioPending();
  }
}

/* History entry at "offset from newest" (0 = latest); NULL when empty */
static const ft8_hist_t *app_hist_at(uint16_t off)
{
  if (s_hist_len == 0U || off >= s_hist_len) return NULL;
  return &s_hist[(uint16_t)((s_hist_wr + APP_HIST - 1U - off) % APP_HIST)];
}

static void app_hist_append(const FT8_Status_t *st)
{
  uint8_t hh, mm, ss;
  RTC_Clock_GetTime(&hh, &mm, &ss);
  for (uint8_t i = 0U; i < st->num_dec; i++) {
    const FT8_Decode_t *d = FT8_GetDecode(i);
    if (d == NULL) break;
    ft8_hist_t *h = &s_hist[s_hist_wr];
    h->d  = *d;
    h->hh = hh; h->mm = mm; h->ss = ss;
    s_hist_wr = (uint16_t)((s_hist_wr + 1U) % APP_HIST);
    if (s_hist_len < APP_HIST) s_hist_len++;
    s_total_dec++;
  }
}

/* ── QSO auto-sequencer helpers ──────────────────────────────────────────── */

/* Split a decoded message into up to 3 tokens; strips <> around hashed calls */
static uint8_t app_split(const char *text, char tok[3][14])
{
  uint8_t n = 0U, len = 0U;
  for (uint8_t i = 0U; i < 3U; i++) tok[i][0] = '\0';
  for (const char *p = text; n < 3U; p++) {
    char c = *p;
    if (c == ' ' || c == '\0') {
      if (len > 0U) { tok[n][len] = '\0'; n++; len = 0U; }
      if (c == '\0') break;
    } else if (len < 13U) {
      if (c != '<' && c != '>') tok[n][len++] = c;
    }
  }
  return n;
}

static bool app_has_digit(const char *s)
{
  for (; *s != '\0'; s++)
    if (*s >= '0' && *s <= '9') return true;
  return false;
}

/* Build our response to the peer's field x ("" = none yet → we call with
 * grid).  Returns true when a transmission is due; false = QSO ended. */
static bool app_qso_respond(const char *x)
{
  int rpt = s_qso_rpt;
  if (rpt < -30) rpt = -30;
  if (rpt >  30) rpt =  30;

  if (x[0] == 'R' && (x[1] == '-' || x[1] == '+')) {
    snprintf(s_qso_msg, sizeof(s_qso_msg), "%s %s RR73", s_qso_peer, g_sdr.ft8_call);
    s_qso_done_after_tx = true;
  } else if (strcmp(x, "RRR") == 0 || strcmp(x, "RR73") == 0) {
    snprintf(s_qso_msg, sizeof(s_qso_msg), "%s %s 73", s_qso_peer, g_sdr.ft8_call);
    s_qso_done_after_tx = true;
  } else if (strcmp(x, "73") == 0) {
    s_qso_active = false;
    app_notice("QSO COMPLETE");
    return false;
  } else if (x[0] == '-' || x[0] == '+') {           /* their report to us */
    snprintf(s_qso_msg, sizeof(s_qso_msg), "%s %s R%+03d", s_qso_peer, g_sdr.ft8_call, rpt);
  } else {                                            /* grid / unknown     */
    snprintf(s_qso_msg, sizeof(s_qso_msg), "%s %s %+03d", s_qso_peer, g_sdr.ft8_call, rpt);
  }
  return true;
}

/* Start a QSO from a decoded line (list selection or auto-answer to our CQ) */
static void app_qso_start(const FT8_Decode_t *d)
{
  if (strlen(g_sdr.ft8_call) < 3U || strlen(g_sdr.ft8_grid) != 4U) {
    app_notice("SET CALL+GRID FIRST (F1)");
    return;
  }
  char tok[3][14];
  uint8_t n = app_split(d->text, tok);
  if (n < 2U) { app_notice("CANT PARSE MSG"); return; }

  s_qso_done_after_tx = false;
  s_qso_retries       = 0U;
  s_qso_rpt           = d->snr_db;
  s_beacon_arm        = false;   /* the sequencer owns TX from here */

  bool tx_due;
  if (strcmp(tok[0], "CQ") == 0) {
    /* "CQ CALL GRID" or "CQ DX CALL GRID" — the callsign has a digit */
    const char *peer = (n >= 3U && !app_has_digit(tok[1])) ? tok[2] : tok[1];
    snprintf(s_qso_peer, sizeof(s_qso_peer), "%s", peer);
    snprintf(s_qso_msg, sizeof(s_qso_msg), "%s %s %s",
             s_qso_peer, g_sdr.ft8_call, g_sdr.ft8_grid);
    tx_due = true;
  } else if (strcmp(tok[0], g_sdr.ft8_call) == 0) {
    /* they are calling us — answer according to their third field */
    snprintf(s_qso_peer, sizeof(s_qso_peer), "%s", tok[1]);
    s_qso_active = true;               /* respond may clear it on "73" */
    tx_due = app_qso_respond((n >= 3U) ? tok[2] : "");
    if (!tx_due) return;               /* was a lone 73 — nothing to send */
  } else {
    /* message between two other stations — cold-call the sender */
    snprintf(s_qso_peer, sizeof(s_qso_peer), "%s", tok[1]);
    snprintf(s_qso_msg, sizeof(s_qso_msg), "%s %s %s",
             s_qso_peer, g_sdr.ft8_call, g_sdr.ft8_grid);
    tx_due = true;
  }
  s_qso_active = true;
  s_qso_tx_due = tx_due;
  char nl[28];
  snprintf(nl, sizeof(nl), "CALLING %s", s_qso_peer);
  app_notice(nl);
}

/* Per-RX-slot QSO progress: advance on a reply from the peer, otherwise
 * repeat the current message; auto-answer stations calling us while the
 * CQ beacon is armed. */
static void app_qso_on_slot(const FT8_Status_t *st)
{
  char tok[3][14];
  if (s_qso_active) {
    for (uint8_t i = 0U; i < st->num_dec; i++) {
      const FT8_Decode_t *d = FT8_GetDecode(i);
      if (d == NULL) break;
      if (app_split(d->text, tok) < 2U) continue;
      if (strcmp(tok[0], g_sdr.ft8_call) != 0) continue;
      if (strcmp(tok[1], s_qso_peer) != 0) continue;
      s_qso_rpt     = d->snr_db;
      s_qso_retries = 0U;
      if (app_qso_respond(tok[2])) s_qso_tx_due = true;
      return;
    }
    /* peer silent this RX slot — repeat, then give up */
    if (++s_qso_retries > QSO_MAX_RETRY) {
      s_qso_active = false;
      s_qso_tx_due = false;
      app_notice("QSO TIMEOUT");
    } else {
      s_qso_tx_due = true;
    }
  } else if (s_beacon_arm && g_sdr.ft8_call[0] != '\0') {
    for (uint8_t i = 0U; i < st->num_dec; i++) {
      const FT8_Decode_t *d = FT8_GetDecode(i);
      if (d == NULL) break;
      if (app_split(d->text, tok) < 2U) continue;
      if (strcmp(tok[0], g_sdr.ft8_call) != 0) continue;
      if (strcmp(tok[1], g_sdr.ft8_call) == 0) continue;
      app_qso_start(d);          /* answers with the right message type */
      break;
    }
  }
}

/* ── Station editor ──────────────────────────────────────────────────────── */
static void app_edit_draw(void)
{
  char line[64];
  app_row_render((uint16_t)(APP_LIST_Y + 0U * APP_ROW_H),
                 "== STATION SETUP ==", APP_HDR_FG, APP_BG);

  snprintf(line, sizeof(line), "CALL: %.11s", s_edit);
  app_row_render((uint16_t)(APP_LIST_Y + 1U * APP_ROW_H), line, APP_TXT, APP_BG);
  memset(line, ' ', 22U); line[22] = '\0';
  if (s_cursor < EDIT_CALL_LEN) line[6U + s_cursor] = '^';
  app_row_render((uint16_t)(APP_LIST_Y + 2U * APP_ROW_H), line, APP_CQ, APP_BG);

  snprintf(line, sizeof(line), "GRID: %.4s", &s_edit[EDIT_CALL_LEN]);
  app_row_render((uint16_t)(APP_LIST_Y + 3U * APP_ROW_H), line, APP_TXT, APP_BG);
  memset(line, ' ', 22U); line[22] = '\0';
  if (s_cursor >= EDIT_CALL_LEN) line[6U + (s_cursor - EDIT_CALL_LEN)] = '^';
  app_row_render((uint16_t)(APP_LIST_Y + 4U * APP_ROW_H), line, APP_CQ, APP_BG);

  app_row_render((uint16_t)(APP_LIST_Y + 5U * APP_ROW_H),
                 "ENC:next pos  F1:SAVE  F4:CANCEL", APP_DIM, APP_BG);
}

static void app_edit_open(void)
{
  memset(s_edit, ' ', EDIT_LEN);
  s_edit[EDIT_LEN] = '\0';
  for (uint8_t i = 0U; i < EDIT_CALL_LEN && g_sdr.ft8_call[i] != '\0'; i++)
    s_edit[i] = g_sdr.ft8_call[i];
  for (uint8_t i = 0U; i < EDIT_GRID_LEN && g_sdr.ft8_grid[i] != '\0'; i++)
    s_edit[EDIT_CALL_LEN + i] = g_sdr.ft8_grid[i];
  s_cursor = 0U;
  s_view   = APP_VIEW_EDIT;
  /* clear the list area, then draw the editor rows */
  for (uint16_t r = 6U; r < APP_ROWS; r++)
    app_row_render((uint16_t)(APP_LIST_Y + r * APP_ROW_H), "", APP_TXT, APP_BG);
  app_edit_draw();
}

static void app_edit_save(void)
{
  /* strip spaces (callsigns/grids contain none) */
  uint8_t n = 0U;
  memset(g_sdr.ft8_call, 0, sizeof(g_sdr.ft8_call));
  for (uint8_t i = 0U; i < EDIT_CALL_LEN; i++)
    if (s_edit[i] != ' ') g_sdr.ft8_call[n++] = s_edit[i];
  n = 0U;
  memset(g_sdr.ft8_grid, 0, sizeof(g_sdr.ft8_grid));
  for (uint8_t i = 0U; i < EDIT_GRID_LEN; i++)
    if (s_edit[EDIT_CALL_LEN + i] != ' ') g_sdr.ft8_grid[n++] = s_edit[EDIT_CALL_LEN + i];
  CSDR_SaveSettings();
}

/* Toggle the CQ beacon (F2).  Validates station data + packs the message. */
static void app_beacon_toggle(void)
{
  if (s_beacon_arm) {
    s_beacon_arm = false;
    return;
  }
  if (strlen(g_sdr.ft8_call) < 3U || strlen(g_sdr.ft8_grid) != 4U) {
    app_notice("SET CALL+GRID FIRST (F1)");
    return;
  }
  snprintf(s_cq_msg, sizeof(s_cq_msg), "CQ %s %s", g_sdr.ft8_call, g_sdr.ft8_grid);
  if (!FT8_TxSetMessage(s_cq_msg)) {
    app_notice("BAD CALL/GRID - CHECK F1");
    return;
  }
  s_beacon_arm = true;
  s_tx_parity  = true;   /* transmit on the next slot boundary */
}

static void app_tx_end(void)
{
  FT8_TxStop();
  CSDR_RequestTX(false);
  s_tx_active = false;
  if (s_qso_active && s_qso_done_after_tx) {   /* RR73 / 73 fully sent */
    s_qso_active        = false;
    s_qso_done_after_tx = false;
    app_notice("QSO COMPLETE");
  }
}

/* ── Main app loop ───────────────────────────────────────────────────────── */
void FT8_App_Run(void)
{
  /* Own key instances (cal.c pattern) — synced so the press that launched
   * the app never fires inside it. */
  Key_t k_f4 = {0}, k_menu = {0}, k_f1 = {0}, k_f2 = {0}, k_enc = {0};
  Key_InitPCA(&k_f4,   &g_pca9555_raw, PCA_BIT_F4);
  Key_InitPCA(&k_menu, &g_pca9555_raw, PCA_BIT_MENU);
  Key_InitPCA(&k_f1,   &g_pca9555_raw, PCA_BIT_F1);
  Key_InitPCA(&k_f2,   &g_pca9555_raw, PCA_BIT_F2);
  Key_Init   (&k_enc,  ENC_SW_GPIO_Port, ENC_SW_Pin);
  Key_Sync(&k_f4); Key_Sync(&k_menu); Key_Sync(&k_f1); Key_Sync(&k_f2);
  Key_Sync(&k_enc);
  (void)Encoder_GetDelta(&g_encoder);   /* discard counts from before entry */

  FT8_SetStripUI(false);
  FT8_SetEnabled(true);

  s_view       = APP_VIEW_LIST;
  s_beacon_arm = false;
  s_tx_active  = false;
  s_notice[0]  = '\0';

  FT8_Status_t st;
  FT8_GetStatus(&st);
  uint32_t last_seq = st.slot_seq;
  int32_t  prev_cyc = FT8_GetCycleMs();

  app_draw_header(&st);
  app_draw_list();
  app_draw_footer(&st);

  uint32_t t_hdr = 0U, t_pa = 0U, t_analog = 0U, t_fan = 0U;
  for (;;) {
    /* Keep the radio alive: audio pump, decoder tick, watchdog, protection,
     * and the T/R sequencing that CSDR_Loop would otherwise be running */
    CSDR_ProcessAudioPending();
    FT8_Poll();
    CSDR_PollTxSequencing();
    PA_OC_HandleFaultInLoop();
    uint32_t now = HAL_GetTick();
    RuntimeDiag_WatchdogRefreshIfHealthy(now);
    if ((now - t_pa)     >=   20U) { t_pa     = now; PA_Protect_Update(); }
    if ((now - t_analog) >=  100U) { t_analog = now; Analog_Update(); }
    if ((now - t_fan)    >= 1000U) { t_fan    = now; Fan_Update(g_analog.temp_c); }

    Input_Scan();
    Key_Poll(&k_f4); Key_Poll(&k_menu); Key_Poll(&k_f1); Key_Poll(&k_f2);
    Key_Poll(&k_enc);
    int32_t enc_delta = Encoder_GetDelta(&g_encoder);

    /* ── TX slot scheduling: QSO sequencer first, else CQ beacon ── */
    int32_t cyc = FT8_GetCycleMs();
    bool slot_boundary = (cyc < prev_cyc);
    prev_cyc = cyc;
    if (s_tx_active) {
      if (FT8_TxDone() || !PA_Protect_IsTxAllowed()) {
        app_tx_end();
        app_draw_header(&st);
      }
    } else if (slot_boundary && s_view == APP_VIEW_LIST) {
      bool want_tx = false;
      if (s_qso_active && s_qso_tx_due) {
        if (FT8_TxSetMessage(s_qso_msg)) {
          want_tx      = true;
          s_qso_tx_due = false;
        } else {
          s_qso_active = false;      /* unpackable peer (nonstd call) */
          app_notice("CANT ENCODE MSG");
        }
      } else if (!s_qso_active && s_beacon_arm) {
        /* re-pack each time: a QSO may have overwritten the tone buffer */
        if (s_tx_parity && FT8_TxSetMessage(s_cq_msg)) want_tx = true;
        s_tx_parity = !s_tx_parity;
      }
      if (want_tx && PA_Protect_IsTxAllowed()) {
        FT8_TxStart(FT8_GetCycleMs());
        CSDR_RequestTX(true);
        s_tx_active = true;
        app_draw_header(&st);
      }
    }

    /* ── Keys ── */
    if (s_view == APP_VIEW_LIST) {
      if (Key_Press(&k_f4) || Key_Press(&k_menu)) break;
      if (Key_Press(&k_f1) && !s_tx_active && !s_qso_active) {
        s_beacon_arm = false;          /* editing invalidates the message */
        app_edit_open();
      }
      if (Key_Press(&k_f2)) {
        if (s_qso_active) {            /* F2 = abort a running QSO */
          if (s_tx_active) app_tx_end();
          s_qso_active = false;
          s_qso_tx_due = false;
          app_notice("QSO ABORT");
          app_draw_header(&st);
        } else {
          app_beacon_toggle();
        }
      }
      /* Encoder: move the selection bar; ENC push = reply to that line */
      if (enc_delta != 0) {
        uint16_t n = (s_hist_len < APP_DATA_ROWS) ? s_hist_len : APP_DATA_ROWS;
        if (n > 0U) {
          int32_t sel = (int32_t)s_sel_off + ((enc_delta > 0) ? 1 : -1);
          if (sel < 0) sel = 0;
          if (sel > (int32_t)(n - 1U)) sel = (int32_t)(n - 1U);
          if ((uint16_t)sel != s_sel_off) {
            s_sel_off = (uint16_t)sel;
            app_draw_list();
          }
        }
      }
      if (Key_Press(&k_enc) && !s_qso_active && !s_tx_active) {
        const ft8_hist_t *h = app_hist_at(s_sel_off);
        if (h != NULL) app_qso_start(&h->d);
        app_draw_header(&st);
      }
    } else { /* APP_VIEW_EDIT */
      if (enc_delta != 0) {
        /* rotate the char at the cursor through the charset */
        char cur = s_edit[s_cursor];
        int  ci  = 0;
        for (int i = 0; i < (int)CHARSET_LEN; i++)
          if (s_charset[i] == cur) { ci = i; break; }
        ci = (ci + (int)CHARSET_LEN + (enc_delta % (int)CHARSET_LEN)) % (int)CHARSET_LEN;
        s_edit[s_cursor] = s_charset[ci];
        app_edit_draw();
      }
      if (Key_Press(&k_enc)) {
        s_cursor = (uint8_t)((s_cursor + 1U) % EDIT_LEN);
        app_edit_draw();
      }
      if (Key_Press(&k_f1)) {
        app_edit_save();
        s_view = APP_VIEW_LIST;
        app_draw_list();
        app_notice("STATION SAVED");
      }
      if (Key_Press(&k_f4)) {
        s_view = APP_VIEW_LIST;
        app_draw_list();
      }
    }

    /* ── Display + QSO progress ── */
    FT8_GetStatus(&st);
    if (st.slot_seq != last_seq) {          /* a slot just finalised */
      last_seq = st.slot_seq;
      app_qso_on_slot(&st);                 /* advance/auto-start the QSO */
      app_hist_append(&st);
      if (s_view == APP_VIEW_LIST) {
        s_sel_off = 0U;                     /* follow the newest entry */
        app_draw_list();
      }
    }
    now = HAL_GetTick();
    if ((now - t_hdr) >= 250U) {
      t_hdr = now;
      app_draw_header(&st);
      app_draw_footer(&st);   /* live diagnostics (blk counts up in capture) */
    }
  }

  /* Never leave the PA keyed */
  if (s_tx_active) app_tx_end();
  s_beacon_arm = false;
  s_qso_active = false;
  s_qso_tx_due = false;

  /* Drain the exit key so csdr_app's own key machines (polled again after we
   * return) don't see the held button as a fresh press (ghost-press rule).
   * 1 s cap: on a PCA9555 I2C fault Input_Scan keeps the stale "pressed"
   * value forever — never wedge the radio on it. */
  uint32_t drain_t0 = HAL_GetTick();
  for (;;) {
    CSDR_ProcessAudioPending();
    Input_Scan();
    uint16_t raw = g_pca9555_raw;
    bool held = (((raw >> PCA_BIT_F4) & 1U) == 0U)
             || (((raw >> PCA_BIT_MENU) & 1U) == 0U);
    if (!held || (HAL_GetTick() - drain_t0) > 1000U) break;
  }

  /* Full-screen overlay exit rule: invalidate UI caches + clear INFO strip */
  FT8_SetStripUI(true);
  FT8_SetEnabled(g_sdr.ft8_decode_on);   /* back to the menu-toggle state */
  SDR_UI_InvalidateCaches();
  SDR_UI_ClearCWText();
}
