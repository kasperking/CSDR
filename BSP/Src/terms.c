/* USER CODE BEGIN Header */
/**
  * @file  terms.c
  * @brief Full-screen "Terms of Use / Disclaimer" text viewer.
  *        See terms.h for the scope / navigation summary.
  */
/* USER CODE END Header */

#include "terms.h"
#include "sdr_ui.h"        /* fonts, LCD helpers, ZONE_SPEC_Y, UI_BG, caches */
#include "lcd_render.h"
#include "lcd_bus_fmc.h"
#include "encoder.h"       /* Key_t API */
#include "input_scan.h"    /* g_pca9555_raw, PCA_BIT_* */
#include "csdr_app.h"      /* CSDR_ProcessAudioPending */
#include "main.h"          /* ENC_SW_GPIO_Port / ENC_SW_Pin */
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim4;   /* encoder timer (TIM4_CH1/CH2 = PD12/PD13) */

#define LN  LCD_GetLineBuf()
#define TM_X          10U
#define TM_HDR_H      16U
#define TM_LINE_H     10U   /* Font6x8 (8px) + 2px leading */
#define TM_FOOTER_H   11U

#define TM_BG         0x0843U   /* dark blue-gray background      */
#define TM_BORDER     0x10A2U
#define TM_HDR_BG     0xF800U
#define TM_FG         0xFFFFU
#define TM_HINT       0x5ACBU

/* Source text as whole paragraphs — wrapped at runtime to whatever width the
 * active panel actually has (see wrap_build()). A "" entry is a blank
 * separator line. Fixes the bug where hand-wrapped short lines only filled
 * the left half of a wide 480px ST7796 panel. */
static const char *const s_paragraphs[] = {
  "This device transmits RF energy. Operate it only if you hold a valid "
  "amateur radio license covering the band, mode, and power in use.",
  "",
  "This is open-source, experimental firmware provided AS IS, WITHOUT "
  "WARRANTY OF ANY KIND, express or implied, including any warranty of "
  "merchantability or fitness for a particular purpose.",
  "",
  "The developer(s) accept NO LIABILITY for equipment damage, interference, "
  "regulatory violations, RF exposure, or any other loss from using this "
  "device. You assume all risk.",
  "",
  "Modifying this device, or its firmware, in a way that results in "
  "unauthorized or illegal transmission is a violation of applicable "
  "law. Any such use is entirely your responsibility and is permanently "
  "logged with a timestamp.",
  "",
  "You are solely responsible for every transmission this device makes, "
  "and for complying with the radio regulations of your country at all "
  "times.",
  "",
  "Using this device means you accept these terms in full.",
};
#define TERMS_PARA_COUNT ((uint8_t)(sizeof(s_paragraphs) / sizeof(s_paragraphs[0])))

/* Wrapped-line buffer, rebuilt once per screen open for the active panel's
 * actual width. 80 cols covers the widest panel today (ST7796: (480-20)/6 =
 * 76) with margin; 64 lines covers this content even on the narrowest panel
 * (ST7789: (240-20)/6 = 36 cols wraps the same paragraphs into ~45 lines). */
#define TM_MAX_COLS   80U
#define TM_MAX_LINES  64U
static char    s_wrapped[TM_MAX_LINES][TM_MAX_COLS];
static uint8_t s_line_count;

static void wrap_build(uint8_t cols)
{
  if (cols == 0U) cols = 1U;
  if (cols >= TM_MAX_COLS) cols = (uint8_t)(TM_MAX_COLS - 1U);

  s_line_count = 0U;
  for (uint8_t p = 0U; p < TERMS_PARA_COUNT && s_line_count < TM_MAX_LINES; p++) {
    const char *s = s_paragraphs[p];
    if (s[0] == '\0') {
      s_wrapped[s_line_count][0] = '\0';
      s_line_count++;
      continue;
    }
    while (*s != '\0' && s_line_count < TM_MAX_LINES) {
      const char *p2 = s;
      const char *last_space = NULL;
      uint8_t n = 0U;
      while (*p2 != '\0' && n < cols) {
        if (*p2 == ' ') last_space = p2;
        p2++; n++;
      }
      uint8_t take;
      const char *next = p2;
      if (*p2 == '\0') {
        take = n;                                   /* remainder fits fully */
      } else if (last_space != NULL && last_space != s) {
        take = (uint8_t)(last_space - s);
        next = last_space + 1;                       /* skip the space      */
      } else {
        take = n;                                    /* hard break, no space */
      }
      /* Guard against a stalled cursor (e.g. a stray leading space in a
       * future edit) — this loop must always make forward progress, it
       * runs before the per-iteration CSDR_ProcessAudioPending() pump. */
      if (take == 0U) { take = 1U; next = s + 1; }
      memcpy(s_wrapped[s_line_count], s, take);
      s_wrapped[s_line_count][take] = '\0';
      s_line_count++;
      s = next;
    }
  }
}

static void push_ln(uint16_t y)
{ LCD_PushWindow(0U, y, (uint16_t)(LCD_W - 1U), y, LN, LCD_W); }

/* ── Encoder detent reader (same idiom as tx_unlock.c / cal.c) ─────────────── */
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

static void row_header(uint16_t *y)
{
  static const char title[] = "TERMS OF USE / DISCLAIMER";
  for (uint16_t fr = 0U; fr < TM_HDR_H; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, UI_BG);
    LCD_LineFill(ln, TM_X, (uint16_t)(LCD_W - 2U * TM_X),
                 (fr == 0U || fr == TM_HDR_H - 1U) ? TM_BORDER : TM_HDR_BG);
    if (fr >= 4U && fr < 4U + (uint16_t)Font6x8.height) {
      uint16_t tx = (uint16_t)(TM_X + ((LCD_W - 2U * TM_X) -
                    (uint16_t)strlen(title) * Font6x8.width) / 2U);
      LCD_LineStr(ln, tx, fr - 4U, title, &Font6x8, TM_FG, TM_HDR_BG);
    }
    push_ln((*y)++);
  }
}

static void row_text(uint16_t y, const char *s, uint16_t fg)
{
  for (uint16_t fr = 0U; fr < TM_LINE_H; fr++) {
    uint16_t *ln = LN;
    LCD_LineFill(ln, 0U, LCD_W, TM_BG);
    if (s != NULL && s[0] != '\0' && fr < (uint16_t)Font6x8.height)
      LCD_LineStr(ln, TM_X, fr, s, &Font6x8, fg, TM_BG);
    push_ln((uint16_t)(y + fr));
  }
}

static void render_screen(uint8_t scroll, uint8_t visible)
{
  uint16_t y = (uint16_t)ZONE_SPEC_Y;
  row_header(&y);

  for (uint8_t i = 0U; i < visible; i++) {
    uint8_t idx = (uint8_t)(scroll + i);
    row_text(y, (idx < s_line_count) ? s_wrapped[idx] : "", TM_FG);
    y = (uint16_t)(y + TM_LINE_H);
  }

  char foot[40];
  bool more_up   = (scroll > 0U);
  bool more_down = (uint8_t)(scroll + visible) < s_line_count;
  snprintf(foot, sizeof foot, "%s ENC/F1/F2=SCROLL  F4=EXIT %s",
           more_up ? "^" : " ", more_down ? "v" : " ");
  row_text(y, foot, TM_HINT); y += TM_FOOTER_H;
  (void)y;
}

void Terms_ShowDisclaimer(void)
{
  /* Wrap the source paragraphs to the panel's actual usable width — fixes
   * hand-wrapped text only filling the left half of a wide 480px panel. */
  uint8_t cols = (uint8_t)((LCD_W - 2U * TM_X) / Font6x8.width);
  wrap_build(cols);

  /* Visible content rows = whatever fits between the header and the footer
   * line, on whichever panel is active (LCD_H is 320 on every supported
   * panel today, but this stays correct if that ever changes). */
  uint16_t avail = (uint16_t)(LCD_H - ZONE_SPEC_Y - TM_HDR_H - TM_FOOTER_H);
  uint8_t  visible = (uint8_t)(avail / TM_LINE_H);
  if (visible == 0U) visible = 1U;
  if (visible > s_line_count) visible = s_line_count;

  uint8_t scroll_max = (s_line_count > visible)
                      ? (uint8_t)(s_line_count - visible) : 0U;
  uint8_t scroll = 0U;

  Key_t k_enc = {0}, k_f1 = {0}, k_f2 = {0}, k_f4 = {0};
  Key_Init   (&k_enc, ENC_SW_GPIO_Port, ENC_SW_Pin);
  Key_InitPCA(&k_f1,  &g_pca9555_raw,  PCA_BIT_F1);
  Key_InitPCA(&k_f2,  &g_pca9555_raw,  PCA_BIT_F2);
  Key_InitPCA(&k_f4,  &g_pca9555_raw,  PCA_BIT_F4);
  /* The key press that opened this screen is likely still held — swallow it. */
  Key_Sync(&k_enc); Key_Sync(&k_f1); Key_Sync(&k_f2); Key_Sync(&k_f4);
  enc_flush();

  render_screen(scroll, visible);

  for (;;) {
    CSDR_ProcessAudioPending();               /* keep audio/USB pipeline alive */
    Key_Poll(&k_enc); Key_Poll(&k_f1); Key_Poll(&k_f2); Key_Poll(&k_f4);
    if (k_enc.state != KS_IDLE) enc_flush();

    bool changed = false;
    int32_t d = enc_read_delta();
    if (d > 0 && scroll < scroll_max) { scroll++; changed = true; }
    if (d < 0 && scroll > 0U)         { scroll--; changed = true; }
    if (Key_PressOrRepeat(&k_f2) && scroll < scroll_max) { scroll++; changed = true; }
    if (Key_PressOrRepeat(&k_f1) && scroll > 0U)         { scroll--; changed = true; }
    if (changed) render_screen(scroll, visible);

    if (Key_Press(&k_enc) || Key_Press(&k_f4)) break;
  }

  /* Full-screen overlay exit rule: drop partial-redraw caches + any CW text. */
  SDR_UI_InvalidateCaches();
  SDR_UI_ClearCWText();
}
