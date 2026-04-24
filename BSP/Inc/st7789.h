/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st7789.h
  * @brief   ST7789VW – mcHF-style SDR UI, Scanline Renderer, Landscape 320×240
  *
  *  Giao diện phỏng theo mcHF (Michael, DF8OE) SDR Transceiver:
  *
  *  Layout 320×240 landscape:
  *  ┌──────────────────────────────────────────────────────────────────┐ Y=0
  *  │ [USB] [40m]  ┃   7 . 1 0 0 . 0 0 0   ┃  S▓▓▓▓▓░░░ S7  [TX]  │ 38px
  *  ├──────────────╂──────────────────────────────────────────────────┤ Y=38
  *  │ AGC : FAST   ┃                                                  │
  *  │ NB  : OFF    ┃       SPECTRUM   (240px × 96px)                  │ 96px
  *  │ NR  : OFF    ┃                                                  │
  *  │ RIT : +000   ┃                                                  │
  *  ├──────────────╂──────────────────────────────────────────────────┤ Y=134
  *  │ VOL : 70%    ┃                                                  │
  *  │ SQ  : 000    ┃       WATERFALL  (240px × 68px)                  │ 68px
  *  │ PWR : RX     ┃                                                  │
  *  │ STEP: 100Hz  ┃                                                  │
  *  ├──────────────┴──────────────────────────────────────────────────┤ Y=202
  *  │  [BAND-] [BAND+] [MODE] [STEP] [AGC] [NB]  [NR]               │ 38px
  *  └──────────────────────────────────────────────────────────────────┘ Y=240
  *
  *  Màu sắc phong cách mcHF:
  *   - Nền: đen tuyệt đối
  *   - Tần số: xanh lá cyan (#00FFAA style) – 14-segment emulation
  *   - Mode/Band: màu vàng trên nền xanh đậm
  *   - S-meter: xanh lá → vàng → đỏ theo thang S1-S9+
  *   - Status panel: trắng/xám trên nền rất tối
  *   - Spectrum: gradient xanh→cam theo biên độ
  *   - Function buttons: viền xám, text vàng, nền đen
  *   - TX indicator: đỏ nhấp nháy
  *
  *  Renderer: Scanline DMA (1 DMA call per scanline = 640 byte/call)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ST7789_H
#define __ST7789_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── ST7789 Commands ────────────────────────────────── */
#define ST7789_SWRESET    0x01U
#define ST7789_SLPOUT     0x11U
#define ST7789_NORON      0x13U
#define ST7789_INVON      0x21U
#define ST7789_DISPON     0x29U
#define ST7789_CASET      0x2AU
#define ST7789_RASET      0x2BU
#define ST7789_RAMWR      0x2CU
#define ST7789_MADCTL     0x36U
#define ST7789_COLMOD     0x3AU
#define ST7789_PORCTRL    0xB2U
#define ST7789_GCTRL      0xB7U
#define ST7789_VCOMS      0xBBU
#define ST7789_LCMCTRL    0xC0U
#define ST7789_VDVVRHEN   0xC2U
#define ST7789_VRHS       0xC3U
#define ST7789_VDVS       0xC4U
#define ST7789_FRCTRL2    0xC6U
#define ST7789_PWCTRL1    0xD0U
#define ST7789_PVGAMCTRL  0xE0U
#define ST7789_NVGAMCTRL  0xE1U

/* MADCTL: Landscape 320×240 (MX|MV|BGR) */
#define ST7789_MADCTL_LANDSCAPE   0x68U

/* ── Screen geometry ────────────────────────────────── */
#define LCD_W             320U
#define LCD_H             240U

/* Left status panel */
#define PANEL_W           78U    /* left panel width */
#define PANEL_X           0U
#define PANEL_DIV_X       (PANEL_W)   /* vertical divider */

/* Right display area */
#define DISP_X            (PANEL_W + 1U)
#define DISP_W            (LCD_W - DISP_X)    /* 241px */

/* Horizontal zones (Y coordinates)
 *  TopBar 62px (2 rows: row1=28px freq + row2=34px S-meter)
 *  Spec   88px | WF 52px | BTN 38px  → total 240
 */
#define ZONE_TOPBAR_Y     0U
#define ZONE_TOPBAR_H     62U
#define ZONE_TOPBAR_R1H   28U    /*!< Row1: freq / mode / band  */
#define ZONE_TOPBAR_R2H   34U    /*!< Row2: S-meter + RX/TX     */
#define ZONE_TOPBAR_Y2    (ZONE_TOPBAR_Y + ZONE_TOPBAR_H)    /* 62 */

#define ZONE_SPEC_Y       (ZONE_TOPBAR_Y2)         /* 62  */
#define ZONE_SPEC_H      108U
#define ZONE_SPEC_Y2      (ZONE_SPEC_Y + ZONE_SPEC_H)       /* 170 */

#define ZONE_WF_Y         (ZONE_SPEC_Y2)           /* 150 */
#define ZONE_WF_H         70U
#define ZONE_WF_Y2        (ZONE_WF_Y + ZONE_WF_H)           /* 240 */


/* S-meter geometry (Row 2 of TopBar) */
#define SM_BARS           12U    /*!< 12 bars: S1-S9 + S9+10..S9+30 */
#define SM_BAR_W          13U    /*!< Each bar: 13px wide           */
#define SM_BAR_GAP        1U     /*!< Gap between bars              */
#define SM_BAR_H          22U    /*!< Bar height (tall & bold)      */
#define SM_BAR_YOFF       9U     /*!< Y offset: after labels(0-7)+gap(8) */
#define SM_START_X        10U    /*!< X start of first bar          */
#define SM_TOTAL_W        (SM_BARS*(SM_BAR_W+SM_BAR_GAP))  /* 168px */
#define SM_SVAL_X         (SM_START_X+SM_TOTAL_W+4U)       /* S-value x */
#define RXTX_X            (LCD_W-56U)   /*!< RX/TX starts x=264   */
#define RXTX_W            56U           /*!< RX/TX width           */

/* Number of function buttons */
#define BTN_COUNT         7U
#define BTN_W             (LCD_W / BTN_COUNT)     /* ~45px */

/* ── mcHF color palette (RGB565) ────────────────────── */

/* Background */
#define MCH_BG            0x0000U   /* Đen tuyệt đối           */
#define MCH_PANEL_BG      0x0841U   /* Panel trái: xanh đen rất tối */
#define MCH_TOPBAR_BG     0x0208U   /* Top bar: xanh navy rất tối   */

/* Borders / dividers */
#define MCH_BORDER        0x2945U   /* Viền xám xanh nhạt      */
#define MCH_DIVIDER       0x31A6U   /* Đường phân cách sáng hơn */

/* Frequency display (phong cách 7-segment xanh lá) */
#define MCH_FREQ_MHZ      0x07FFU   /* Cyan sáng: MHz           */
#define MCH_FREQ_KHZ      0x3FE0U   /* Xanh lá: kHz             */
#define MCH_FREQ_HZ       0x2D65U   /* Xanh lá tối: Hz          */
#define MCH_FREQ_DOT      0x07FFU   /* Dấu chấm phân cách       */

/* Mode / Band tags */
#define MCH_MODE_BG       0x000FU   /* Nền mode: xanh dương đậm */
#define MCH_MODE_FG       0xFFE0U   /* Text mode: vàng          */
#define MCH_BAND_BG       0x3800U   /* Nền band: cam tối        */
#define MCH_BAND_FG       0xFD20U   /* Text band: cam sáng      */

/* S-meter */
#define MCH_SMETER_BG     0x1082U   /* Nền thanh S-meter        */
#define MCH_S1_6          0x07E0U   /* S1-S6: xanh lá           */
#define MCH_S7_9          0xFFE0U   /* S7-S9: vàng              */
#define MCH_S9P           0xF800U   /* S9+: đỏ                  */
#define MCH_SMETER_TICK   0x5AEBU   /* Vạch chia                */

/* Status panel text */
#define MCH_STATUS_LBL    0x8410U   /* Nhãn: xám                */
#define MCH_STATUS_VAL    0xFFFFU   /* Giá trị: trắng           */
#define MCH_STATUS_ON     0x07E0U   /* ON: xanh lá              */
#define MCH_STATUS_OFF    0xF800U   /* OFF: đỏ                  */

/* TX indicator */
#define MCH_TX_BG         0xF800U   /* TX: đỏ                   */
#define MCH_TX_FG         0xFFFFU   /* TX text: trắng           */
#define MCH_RX_BG         0x0400U   /* RX: xanh tối             */
#define MCH_RX_FG         0x07E0U   /* RX text: xanh lá         */

/* Spectrum colors */
#define MCH_SPEC_BG       0x0000U   /* Nền spectrum: đen        */
#define MCH_SPEC_GRID     0x18C6U   /* Lưới: xám rất tối        */
#define MCH_SPEC_LOW      0x001FU   /* Biên độ thấp: xanh       */
#define MCH_SPEC_MID      0x07E0U   /* Biên độ vừa: xanh lá     */
#define MCH_SPEC_HIGH     0xFD20U   /* Biên độ cao: cam         */
#define MCH_SPEC_PEAK     0xFFFFU   /* Đỉnh peak: trắng         */
#define MCH_SPEC_CENTER   0xF81FU   /* Marker tần số: magenta   */
#define MCH_SPEC_BW       0x07FFU   /* Marker bandwidth: cyan   */

/* Waterfall */
/* Gradient: 0=đen → 0.2=navy → 0.45=teal → 0.7=vàng → 1=đỏ */

/* Function buttons */
#define MCH_BTN_BG        0x0841U   /* Nền nút                  */
#define MCH_BTN_BORDER    0x2945U   /* Viền nút                 */
#define MCH_BTN_FG        0xFFE0U   /* Text nút: vàng           */
#define MCH_BTN_ACTIVE_BG 0x0007U   /* Nút đang active: xanh    */
#define MCH_BTN_ACTIVE_FG 0x07FFU   /* Text active: cyan        */

/* FreqStep_t defined in main.h – không khai báo lại ở đây */

/* ── SDR state for UI (gọn, chỉ phần UI cần) ───────── */
typedef struct {
  uint32_t   freq_hz;          /* Tần số thu                  */
  uint8_t    mode;             /* 0=AM 1=FM 2=USB 3=LSB 4=CW  */
  uint8_t    band_idx;         /* 0=160m 1=80m 2=40m ...      */
  float      signal_db;        /* S-meter dBFS                */
  uint8_t    volume;           /* 0-100                       */
  uint8_t    squelch;          /* 0-100                       */
  uint32_t   step;             /* Bước tần số (Hz)            */
  bool       agc_fast;         /* AGC fast/slow               */
  bool       nb_on;            /* Noise blanker               */
  bool       nr_on;            /* Noise reduction             */
  int16_t    rit_hz;           /* RIT offset Hz               */
  bool       tx_mode;          /* TX / RX                     */
  bool       si5351_ok;        /* SI5351 hoạt động            */
  bool       qse_on;           /* QSE TX đang bật             */
  uint8_t    active_btn;       /* Nút function đang chọn 0-6  */
} McHF_UI_State_t;

/* ── Font types ─────────────────────────────────────── */
typedef struct {
  const uint8_t *data;
  uint8_t        width;
  uint8_t        height;
} Font_t;

extern const Font_t Font6x8;

/* ── LCD handle ─────────────────────────────────────── */
typedef struct {
  SPI_HandleTypeDef *hspi;
  DMA_HandleTypeDef *hdma_tx;
  GPIO_TypeDef  *cs_port;  uint16_t cs_pin;
  GPIO_TypeDef  *dc_port;  uint16_t dc_pin;
  GPIO_TypeDef  *rst_port; uint16_t rst_pin;
  GPIO_TypeDef  *bl_port;  uint16_t bl_pin;
  uint16_t       width;
  uint16_t       height;
  volatile bool  dma_busy;

  /* Waterfall circular row pointer */
  uint16_t wf_row;

  /* Peak hold: max bar_h trên mỗi bin (decay theo thời gian) */
  /* peak hold đã bỏ - không cần spec_peak arrays */
} ST7789_Handle_t;

/* ── Core API ───────────────────────────────────────── */
void ST7789_Init(ST7789_Handle_t *lcd);
void ST7789_SetBacklight(ST7789_Handle_t *lcd, bool on);
void ST7789_DMA_TxCpltCallback(ST7789_Handle_t *lcd);
void ST7789_SetWindow(ST7789_Handle_t *lcd,
                      uint16_t x0, uint16_t y0,
                      uint16_t x1, uint16_t y1);
void ST7789_FillScreen(ST7789_Handle_t *lcd, uint16_t color);

/* ── Scanline primitives ────────────────────────────── */
void ST7789_PushScanline(ST7789_Handle_t *lcd,
                          uint16_t y, const uint16_t *line);
void ST7789_PushBlock(ST7789_Handle_t *lcd,
                      uint16_t y0, uint16_t y1, const uint16_t *buf);

/* ── Line buffer render helpers ─────────────────────── */
void LCD_LineFill(uint16_t *ln, uint16_t x0, uint16_t w, uint16_t color);
void LCD_LineChar(uint16_t *ln, uint16_t x, uint16_t frow,
                  char c, const Font_t *f, uint16_t fg, uint16_t bg);
void LCD_LineStr(uint16_t *ln, uint16_t x, uint16_t frow,
                 const char *s, const Font_t *f, uint16_t fg, uint16_t bg);
void LCD_LineRect(uint16_t *ln, uint16_t x0, uint16_t w,
                  uint16_t row, uint16_t total_h, uint16_t border,
                  uint16_t fill, uint16_t border_color);

/* ── mcHF UI zones ──────────────────────────────────── */

/**
  * @brief  Vẽ toàn bộ frame tĩnh một lần lúc boot.
  *         Bao gồm: borders, dividers, zone backgrounds.
  */
void McHF_DrawFrame(ST7789_Handle_t *lcd);

/**
  * @brief  Vẽ Top Bar (38 dòng):
  *         [MODE] [BAND] | Frequency (large) | S-meter | [RX/TX]
  *         Compute 38 dòng → PushBlock (1 DMA call cho 38×320=12160B)
  */
void McHF_DrawTopBar(ST7789_Handle_t *lcd, const McHF_UI_State_t *ui);

/**
  * @brief  Vẽ Left Status Panel (dòng 38..201, cột 0..77).
  *         Nội dung: AGC, NB, NR, RIT, VOL, SQ, STEP, SI.
  *         Compute từng scanline → PushScanline dòng nào thay đổi.
  *
  *  Vì panel trái chỉ 78px × 164px = 12792 byte, push block cho toàn panel.
  */
void McHF_DrawStatusPanel(ST7789_Handle_t *lcd, const McHF_UI_State_t *ui);

/**
  * @brief  Vẽ Spectrum (dòng 38..133, cột 79..319).
  *         Scanline renderer: 96 dòng × 1 DMA call/dòng.
  *         Gradient màu: xanh → xanh lá → cam theo biên độ.
  *         Peak hold: vạch trắng ở đỉnh peak, decay 2 giây.
  *         Center marker: vạch magenta tại tần số LO.
  */
/**
 * @brief  Vẽ Spectrum full-band (DISP_W × ZONE_SPEC_H).
 *         Hiển thị toàn dải ±Fs/2 quanh center (đã fftshift bởi DSP).
 *         Marker center (magenta) + marker bandwidth (cyan) tuỳ mode.
 * @param  fft_db        Mảng magnitude dB (bins phần tử, đã fftshift)
 * @param  bins          DSP_FFT_SIZE = 256
 * @param  bw_lo_ratio   Offset dương về phía TRÁI center (= -offset / Fs)
 *                         0 = không vẽ. LSB = BW/Fs.
 * @param  bw_hi_ratio   Offset dương về phía PHẢI center (= +offset / Fs)
 *                         0 = không vẽ. USB/CW = BW/Fs.
 *                         AM/FM: cả lo và hi = BW/2/Fs (2 đường ±BW/2).
 * @param  ui            State UI (hiện chưa dùng)
 */
void McHF_DrawSpectrum(ST7789_Handle_t *lcd,
                        const float *fft_db, uint16_t bins,
                        float bw_lo_ratio, float bw_hi_ratio,
                        McHF_UI_State_t *ui);

/**
  * @brief  Vẽ 1 dòng Waterfall (dòng 134..201, cột 79..319).
  *         1 DMA call = 241×2 = 482 byte.
  *         Gradient màu phong cách mcHF: đen→navy→teal→vàng→đỏ.
  */
void McHF_DrawWaterfall(ST7789_Handle_t *lcd,
                         const float *fft_db, uint16_t bins);

/**
  * @brief  Vẽ Function Button Bar (dòng 202..239).
  *         7 nút × 45px, viền xám, text vàng.
  *         active_btn được highlight xanh.
  *         Compute 38 dòng → PushBlock.
  */
void McHF_DrawFuncBar(ST7789_Handle_t *lcd, const McHF_UI_State_t *ui);

/**
  * @brief  Update nhẹ S-meter (chỉ vẽ lại phần thanh trong TopBar).
  *         Chỉ push các dòng liên quan đến thanh S-meter.
  */
void McHF_UpdateSMeter(ST7789_Handle_t *lcd, float signal_db);
void McHF_UpdateSMeter_SetTX(bool tx);  /*!< Sync TX/RX state cho partial update */

/* ── Convenience wrappers (compat với main.c cũ) ───── */
static inline void SDR_UI_DrawFrame(ST7789_Handle_t *l)
{ McHF_DrawFrame(l); }
static inline void SDR_UI_DrawError(ST7789_Handle_t *l, const char *m)
{ (void)l; (void)m; }
static inline void SDR_UI_DrawHeader(ST7789_Handle_t *l,
  uint32_t f, uint8_t mode, float db, bool si, bool qse)
{
  McHF_UI_State_t ui = {0};
  ui.freq_hz = f; ui.mode = mode; ui.signal_db = db;
  ui.si5351_ok = si; ui.qse_on = qse;
  McHF_DrawTopBar(l, &ui);
}
static inline void SDR_UI_DrawSpectrum(ST7789_Handle_t *l,
  const float *fft, uint16_t bins, float bw_lo, float bw_hi)
{
  McHF_UI_State_t ui = {0};
  McHF_DrawSpectrum(l, fft, bins, bw_lo, bw_hi, &ui);
}
static inline void SDR_UI_DrawWaterfall(ST7789_Handle_t *l,
  const float *fft, uint16_t bins)
{ McHF_DrawWaterfall(l, fft, bins); }
static inline void SDR_UI_DrawSMeter(ST7789_Handle_t *l, float db)
{ McHF_UpdateSMeter(l, db); }
static inline void SDR_UI_DrawFrequency(ST7789_Handle_t *l, uint32_t f)
{ (void)l; (void)f; }
static inline void SDR_UI_DrawMode(ST7789_Handle_t *l, uint8_t m)
{ (void)l; (void)m; }
static inline void SDR_UI_DrawVolume(ST7789_Handle_t *l, uint8_t v)
{ (void)l; (void)v; }


/* ── Internal helpers exposed for menu.c ────────────────── */
void dma_wait_pub(ST7789_Handle_t *lcd);
void cs_high_pub(ST7789_Handle_t *lcd);
uint16_t *ST7789_GetLineBuf(void);

#ifdef __cplusplus
}
#endif
#endif /* __ST7789_H */
