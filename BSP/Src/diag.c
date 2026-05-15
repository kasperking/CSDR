/* USER CODE BEGIN Header */
/**
  * @file  diag.c
  * @brief Lightweight runtime diagnostics page.
  *
  *  The page is intentionally non-blocking: Diag_Run() only toggles a RAM flag
  *  and CSDR_Loop() calls Diag_Process() at a slow rate.  This keeps audio DMA
  *  callbacks free of LCD/SPI work and avoids terminal-style logging.
  */
/* USER CODE END Header */

#include "diag.h"
#include "csdr_app.h"
#include "sdr_ui.h"
#include "runtime_diag.h"

#define DIAG_X          8U
#define DIAG_Y          62U
#define DIAG_W          112U
#define DIAG_ROW_H      9U
#define DIAG_ROWS       20U
#define DIAG_BG         0x0000U
#define DIAG_FG         0x07E0U
#define DIAG_WARN_FG    0xFFE0U
#define DIAG_BORDER     0x39E7U

static bool s_diag_active = false;
static bool s_diag_full_redraw = false;
static uint32_t s_diag_last_ms = 0U;
static char s_diag_last[DIAG_ROWS][18];
static bool s_diag_row_valid[DIAG_ROWS];
static RuntimeDiag_Snapshot_t s_diag_snapshot;

static void diag_u32_to_dec(uint32_t v, char *out, uint8_t out_len)
{
  char tmp[10];
  uint8_t n = 0U;
  if (out_len == 0U) return;
  if (v == 0U) {
    if (out_len > 1U) { out[0] = '0'; out[1] = '\0'; }
    else out[0] = '\0';
    return;
  }
  while (v != 0U && n < sizeof(tmp)) {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  }
  uint8_t i = 0U;
  while (n != 0U && i < (uint8_t)(out_len - 1U)) out[i++] = tmp[--n];
  out[i] = '\0';
}

static void diag_make_line(char *dst, uint8_t dst_len, const char *label, uint32_t value)
{
  uint8_t pos = 0U;
  while (*label != '\0' && pos < (uint8_t)(dst_len - 1U)) dst[pos++] = *label++;
  if (pos < (uint8_t)(dst_len - 1U)) dst[pos++] = ':';
  diag_u32_to_dec(value, &dst[pos], (uint8_t)(dst_len - pos));
}

static bool diag_str_eq(const char *a, const char *b)
{
  while (*a == *b) {
    if (*a == '\0') return true;
    a++;
    b++;
  }
  return false;
}

static void diag_str_copy(char *dst, const char *src, uint8_t dst_len)
{
  uint8_t i = 0U;
  if (dst_len == 0U) return;
  while (src[i] != '\0' && i < (uint8_t)(dst_len - 1U)) {
    dst[i] = src[i];
    i++;
  }
  dst[i] = '\0';
}

static void diag_draw_row(uint8_t row, const char *text, uint16_t fg)
{
  uint16_t *ln = LCD_GetLineBuf();
  uint16_t y = (uint16_t)(DIAG_Y + (uint16_t)row * DIAG_ROW_H);
  for (uint16_t fr = 0U; fr < DIAG_ROW_H; fr++) {
    LCD_LineFill(ln, DIAG_X, DIAG_W, DIAG_BG);
    ln[DIAG_X] = SWAP16(DIAG_BORDER);
    ln[DIAG_X + DIAG_W - 1U] = SWAP16(DIAG_BORDER);
    if (fr < (uint16_t)Font6x8.height) {
      LCD_LineStr(ln, (uint16_t)(DIAG_X + 3U), fr, text, &Font6x8, fg, DIAG_BG);
    }
    LCD_PushWindow(DIAG_X, (uint16_t)(y + fr),
                   (uint16_t)(DIAG_X + DIAG_W - 1U), (uint16_t)(y + fr),
                   ln + DIAG_X, DIAG_W);
  }
}

static void diag_draw_frame(void)
{
  uint16_t *ln = LCD_GetLineBuf();
  const uint16_t h = (uint16_t)(DIAG_ROWS * DIAG_ROW_H);
  for (uint16_t fr = 0U; fr < h; fr++) {
    LCD_LineFill(ln, DIAG_X, DIAG_W, DIAG_BG);
    if (fr == 0U || fr == (uint16_t)(h - 1U)) LCD_LineFill(ln, DIAG_X, DIAG_W, DIAG_BORDER);
    ln[DIAG_X] = SWAP16(DIAG_BORDER);
    ln[DIAG_X + DIAG_W - 1U] = SWAP16(DIAG_BORDER);
    LCD_PushWindow(DIAG_X, (uint16_t)(DIAG_Y + fr),
                   (uint16_t)(DIAG_X + DIAG_W - 1U), (uint16_t)(DIAG_Y + fr),
                   ln + DIAG_X, DIAG_W);
  }
}

void Diag_Run(void)
{
  s_diag_active = !s_diag_active;
  s_diag_full_redraw = s_diag_active;
  s_diag_last_ms = 0U;
  for (uint8_t i = 0U; i < DIAG_ROWS; i++) {
    s_diag_last[i][0] = '\0';
    s_diag_row_valid[i] = false;
  }
  if (s_diag_active) {
    /* Freeze metrics at the moment DIAG is entered.  All display updates use
     * this copy so the diagnostic screen has zero effect on live measurements. */
    RuntimeDiag_GetSnapshot(&s_diag_snapshot);
  } else {
    g_sdr.display_dirty = DIRTY_ALL;
    SDR_UI_RedrawFooter();
  }
}

bool Diag_IsActive(void)
{
  return s_diag_active;
}

static void diag_update_row(uint8_t row, const char *line, uint16_t fg)
{
  if (row >= DIAG_ROWS) return;
  if (s_diag_row_valid[row] && diag_str_eq(s_diag_last[row], line)) return;
  diag_draw_row(row, line, fg);
  diag_str_copy(s_diag_last[row], line, sizeof(s_diag_last[row]));
  s_diag_row_valid[row] = true;
}

void Diag_Process(void)
{
  if (!s_diag_active) return;

  uint32_t now = HAL_GetTick();
  if (!s_diag_full_redraw && (now - s_diag_last_ms) < 1000U) return;
  s_diag_last_ms = now;

  /* No UiRenderBegin/End here: DIAG rendering must not contaminate max_ui_us.
   * Normal UI rendering is already gated by Diag_IsActive() in csdr_app.c. */

  if (s_diag_full_redraw) {
    diag_draw_frame();
    for (uint8_t i = 0U; i < DIAG_ROWS; i++) s_diag_row_valid[i] = false;
    s_diag_full_redraw = false;
  }

  /* All values come from the frozen snapshot taken at DIAG entry (or last reset).
   * After the initial draw, frozen data matches the cached rows and produces no
   * SPI traffic until the user calls Diag_ResetPeaks(). */
  const RuntimeDiag_Snapshot_t snap = s_diag_snapshot;

  char line[18];
  diag_make_line(line, sizeof(line), "CPU", (uint32_t)snap.cpu_load_percent);
  diag_update_row(0U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "RXOVR", snap.rx_overrun_total);
  diag_update_row(1U, line, (snap.rx_overrun_total != 0U) ? DIAG_WARN_FG : DIAG_FG);
  diag_make_line(line, sizeof(line), "RXpk/s", snap.max_rx_overrun_per_sec);
  diag_update_row(2U, line, (snap.max_rx_overrun_per_sec != 0U) ? DIAG_WARN_FG : DIAG_FG);
  diag_make_line(line, sizeof(line), "TXUND", snap.tx_underrun_total);
  diag_update_row(3U, line, (snap.tx_underrun_total != 0U) ? DIAG_WARN_FG : DIAG_FG);
  diag_make_line(line, sizeof(line), "TXpk/s", snap.max_tx_underrun_per_sec);
  diag_update_row(4U, line, (snap.max_tx_underrun_per_sec != 0U) ? DIAG_WARN_FG : DIAG_FG);
  diag_make_line(line, sizeof(line), "FLT", snap.fault_flags);
  diag_update_row(5U, line, (snap.fault_flags != 0U) ? DIAG_WARN_FG : DIAG_FG);
  diag_make_line(line, sizeof(line), "DSPus", snap.max_dsp_us);
  diag_update_row(6U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "UIus", snap.max_ui_us);
  diag_update_row(7U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "LOOPus", snap.max_loop_stall_us);
  diag_update_row(8U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "UUIus", snap.underrun_ui_us);
  diag_update_row(9U, line, (snap.tx_underrun_total != 0U) ? DIAG_WARN_FG : DIAG_FG);
  diag_make_line(line, sizeof(line), "WFus", snap.ui_section_max_us[RUNTIME_DIAG_UI_WATERFALL]);
  diag_update_row(10U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "SPECus", snap.ui_section_max_us[RUNTIME_DIAG_UI_SPECTRUM]);
  diag_update_row(11U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "LCDus", snap.ui_section_max_us[RUNTIME_DIAG_UI_LCD_FLUSH]);
  diag_update_row(12U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "TXTus", snap.ui_section_max_us[RUNTIME_DIAG_UI_TEXT]);
  diag_update_row(13U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "STATus", snap.ui_section_max_us[RUNTIME_DIAG_UI_STATUS_BAR]);
  diag_update_row(14U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "VMODus", snap.ui_section_max_us[RUNTIME_DIAG_UI_VOLUME_MODE]);
  diag_update_row(15U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "SPIus", snap.ui_section_max_us[RUNTIME_DIAG_UI_SPI_TRANSFER]);
  diag_update_row(16U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "WFPRE", snap.ui_section_max_us[RUNTIME_DIAG_UI_WF_PRECOMPUTE]);
  diag_update_row(17U, line, DIAG_FG);
  diag_make_line(line, sizeof(line), "WFSC", snap.ui_section_max_us[RUNTIME_DIAG_UI_WF_SCROLL]);
  diag_update_row(18U, line, DIAG_FG);

  {
    uint32_t sk, dr;
    SDR_UI_GetSpecSkipStats(&sk, &dr);
    uint32_t total = sk + dr;
    uint32_t pct   = (total > 0U) ? (sk * 100U / total) : 0U;
    diag_make_line(line, sizeof(line), "SKIP%", pct);
    diag_update_row(19U, line, DIAG_FG);
  }
}

void Diag_ResetPeaks(void)
{
  RuntimeDiag_ResetPeaks();
  RuntimeDiag_GetSnapshot(&s_diag_snapshot);
  for (uint8_t i = 0U; i < DIAG_ROWS; i++) s_diag_row_valid[i] = false;
  s_diag_last_ms = 0U;
}
