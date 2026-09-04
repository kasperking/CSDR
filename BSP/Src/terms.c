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

/* Hand-wrapped to <=34 columns so it fits the narrowest supported panel
 * (ST7789 240px landscape: usable width 220px / 6px Font6x8 = 36 columns). */
static const char *const s_lines[] = {
  "This device transmits RF energy.",
  "Operate it only if you hold a",
  "valid amateur radio license",
  "covering the band, mode, and",
  "power in use.",
  "",
  "This is open-source, experimental",
  "firmware provided AS IS, WITHOUT",
  "WARRANTY OF ANY KIND, express or",
  "implied, including any warranty",
  "of merchantability or fitness",
  "for a particular purpose.",
  "",
  "The developer(s) accept NO",
  "LIABILITY for equipment damage,",
  "interference, regulatory",
  "violations, RF exposure, or any",
  "other loss from using this",
  "device. You assume all risk.",
  "",
  "TX BAND UNLOCK -- reachable via",
  "System > About, or the hidden",
  "F1+F2 gesture -- disables the",
  "amateur-band TX limiter.",
  "",
  "Enabling it may let this radio",
  "transmit outside bands you are",
  "licensed for. This can be",
  "ILLEGAL in your country.",
  "",
  "Every unlock, re-lock, and",
  "out-of-band TX event is logged",
  "with a timestamp to flash, and",
  "cannot be cleared from this menu.",
  "",
  "You are solely responsible for",
  "every transmission this device",
  "makes, and for complying with",
  "the radio regulations of your",
  "country at all times.",
  "",
  "Using this device means you",
  "accept these terms in full.",
};
#define TERMS_LINE_COUNT  ((uint8_t)(sizeof(s_lines) / sizeof(s_lines[0])))

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
    row_text(y, (idx < TERMS_LINE_COUNT) ? s_lines[idx] : "", TM_FG);
    y = (uint16_t)(y + TM_LINE_H);
  }

  char foot[40];
  bool more_up   = (scroll > 0U);
  bool more_down = (uint8_t)(scroll + visible) < TERMS_LINE_COUNT;
  snprintf(foot, sizeof foot, "%s ENC/F1/F2=SCROLL  F4=EXIT %s",
           more_up ? "^" : " ", more_down ? "v" : " ");
  row_text(y, foot, TM_HINT); y += TM_FOOTER_H;
  (void)y;
}

void Terms_ShowDisclaimer(void)
{
  /* Visible content rows = whatever fits between the header and the footer
   * line, on whichever panel is active (LCD_H is 320 on every supported
   * panel today, but this stays correct if that ever changes). */
  uint16_t avail = (uint16_t)(LCD_H - ZONE_SPEC_Y - TM_HDR_H - TM_FOOTER_H);
  uint8_t  visible = (uint8_t)(avail / TM_LINE_H);
  if (visible == 0U) visible = 1U;
  if (visible > TERMS_LINE_COUNT) visible = TERMS_LINE_COUNT;

  uint8_t scroll_max = (TERMS_LINE_COUNT > visible)
                      ? (uint8_t)(TERMS_LINE_COUNT - visible) : 0U;
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
