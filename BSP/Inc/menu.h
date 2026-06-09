/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    menu.h
  * @brief   SDR Menu System – 2-level parent/child hierarchy
  *
  *  Navigation:
  *   MENU_KEY     → Mở/đóng menu (toggle)
  *   F1           → Lên (UP) / tăng giá trị khi edit
  *   F2           → Xuống (DOWN) / giảm giá trị khi edit
  *   ENC rotate   → Thay đổi giá trị (edit mode)
  *   ENC press    → Vào nhóm / bắt đầu edit
  *   F3           → Xác nhận / về root
  *   F4           → Thoát edit → về root → đóng menu
  *
  *  Cấu trúc menu (2 cấp):
  *
  *  Root
  *   ├─ RX >
  *   │   ├─ AGC       [SLOW / FAST]
  *   │   ├─ NB        [OFF / ON]
  *   │   ├─ NR        [OFF / ON]
  *   │   ├─ ATT       [0..31 dB]
  *   │   ├─ Squelch   [0..100]
  *   │   ├─ RIT       [-999..+999 Hz]
  *   │   └─ Span      [+/-24k..+/-3k]
  *   ├─ Audio >
  *   │   ├─ Volume    [0..100]
  *   │   ├─ Mic Gain  [0..100]
  *   │   └─ Digi Drive[0..100]
  *   ├─ Tuning >
  *   │   ├─ Step      [1Hz..100KHz]
  *   │   ├─ Band      [160m..6m]
  *   │   └─ Mode      [AM/FM/USB/LSB/CW]
  *   ├─ TX >
  *   │   ├─ RF Power  [5..100% or 1..NW]
  *   │   └─ Ext ALC   [OFF / ON]
  *   ├─ CW >
  *   │   ├─ CW Pitch    [300..900 Hz]
  *   │   ├─ CW Speed    [5..40 WPM]
  *   │   ├─ Keyer Mode  [STRAIGHT/IAMBIC-A/B]
  *   │   ├─ CW Reverse  [OFF / ON]
  *   │   ├─ Paddle Rev  [OFF / ON]
  *   │   ├─ Sidetone Vol[0..100 %]
  *   │   ├─ BK-IN       [OFF/SEMI/FULL]
  *   │   ├─ BK Delay    [50..2000 ms]
  *   │   └─ CW Filter   [100..2000 Hz]
  *   └─ System >
  *       ├─ Backlight   [0..100 %]
  *       ├─ USB         [Off/CAT/CAT+Audio]
  *       ├─ Calibration [>> RUN]
  *       └─ SWR Scan    [>> RUN]
  *
  *  Renderer: overlay trên vùng Spectrum (Y=ZONE_SPEC_Y)
  *  Scanline-based: mỗi item 16px, header 16px, hint 8px
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MENU_H
#define __MENU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "sdr_ui.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/

#define MENU_ITEM_COUNT      36U   /* 6 groups + 30 leaf items */
#define MENU_VISIBLE_ROWS     6U
#define MENU_ITEM_H          16U
#define MENU_X               10U
#define MENU_W  (LCD_W - 2U * MENU_X)
#define MENU_Y               ZONE_SPEC_Y
#define MENU_HEADER_BG      0xF800U
#define MENU_BG_COLOR       0x0843U
#define MENU_FG_COLOR       0xFFFFU
#define MENU_SEL_COLOR      0xF800U
#define MENU_SEL_FG         0xFFFFU
#define MENU_LBL_COLOR      0xA514U
#define MENU_VAL_COLOR      0xFFFFU
#define MENU_BORDER_COLOR   0x10A2U
#define MENU_GROUP_COLOR    0x07FFU   /* Cyan for group label + arrow */

/* RF Power item index — used in LoadFromSDR for dynamic W/% config */
#define MENU_IDX_RFPOWER    19U

/* Exported types ------------------------------------------------------------*/

typedef enum {
  MENU_TYPE_BOOL,
  MENU_TYPE_INT,
  MENU_TYPE_ENUM,
  MENU_TYPE_ACTION,
  MENU_TYPE_GROUP,   /* Category header: enters sub-list on select, no value */
} MenuItemType_t;

typedef struct {
  const char  *label;
  MenuItemType_t type;
  int32_t      min;
  int32_t      max;
  int32_t      step;
  int32_t      *value_ptr;
  const char  **enum_strs;
  uint8_t      enum_count;
  void        (*on_change)(void);
  const char  *suffix;
  int8_t       parent;   /* -1 = root; >=0 = index of parent GROUP item */
} MenuItem_t;

typedef struct {
  bool         open;
  uint8_t      cursor;        /* Position in current view (0..view_count-1)  */
  uint8_t      scroll;
  bool         editing;
  int8_t       current_group; /* -1 = root; >=0 = index of active group item */
  MenuItem_t   items[MENU_ITEM_COUNT];
  uint8_t      item_count;
  uint8_t      view[MENU_ITEM_COUNT]; /* Indices of items in current view     */
  uint8_t      view_count;
} Menu_Handle_t;

/* Exported variables --------------------------------------------------------*/
extern Menu_Handle_t g_menu;

/* Exported functions prototypes ---------------------------------------------*/
void Menu_Init(Menu_Handle_t *m);
void Menu_Toggle(Menu_Handle_t *m);
void Menu_Up(Menu_Handle_t *m);
void Menu_Down(Menu_Handle_t *m);
void Menu_Select(Menu_Handle_t *m);
void Menu_Confirm(Menu_Handle_t *m);
void Menu_Back(Menu_Handle_t *m);
void Menu_EncoderEdit(Menu_Handle_t *m, int32_t delta);
void Menu_Render(Menu_Handle_t *m);

static inline bool Menu_IsOpen(const Menu_Handle_t *m) { return m->open; }

/* Returns pointer to the currently highlighted item (NULL if view empty) */
static inline MenuItem_t *Menu_CurrentItem(Menu_Handle_t *m) {
  if (m->view_count == 0U || m->cursor >= m->view_count) return NULL;
  return &m->items[m->view[m->cursor]];
}

typedef void (*MenuApplyFn)(void);

void Menu_LoadFromSDR(Menu_Handle_t *m,
                       bool agc_fast, bool nb, bool nr, int16_t rit,
                       uint8_t vol, uint8_t mic_gain, uint8_t digi_gain,
                       uint8_t sq, uint32_t step,
                       uint8_t att, uint8_t band, uint8_t mode,
                       uint8_t usb_mode, uint8_t zoom,
                       bool ext_alc, uint8_t rf_power_pct, uint8_t pa_watts,
                       uint8_t backlight,
                       uint16_t cw_pitch, uint8_t cw_wpm, uint8_t cw_keyer,
                       bool cw_rev, bool cw_paddle_rev,
                       uint8_t cw_st_vol, uint8_t cw_bkin, uint16_t cw_bk_delay,
                       uint16_t cw_filter,
                       uint16_t tx_audio_low_hz,
                       uint16_t tx_audio_high_hz,
                       MenuApplyFn apply_cb);

void Menu_SaveToSDR(Menu_Handle_t *m,
                     bool *agc_fast, bool *nb, bool *nr, int16_t *rit,
                     uint8_t *vol, uint8_t *mic_gain, uint8_t *digi_gain,
                     uint8_t *sq, uint32_t *step,
                     uint8_t *att, uint8_t *band, uint8_t *mode,
                     uint8_t *usb_mode, uint8_t *zoom,
                     bool *ext_alc, uint8_t *rf_power, uint8_t *backlight,
                     uint16_t *cw_pitch, uint8_t *cw_wpm, uint8_t *cw_keyer,
                     bool *cw_rev, bool *cw_paddle_rev,
                     uint8_t *cw_st_vol, uint8_t *cw_bkin, uint16_t *cw_bk_delay,
                     uint16_t *cw_filter,
                     uint16_t *tx_audio_low_hz,
                     uint16_t *tx_audio_high_hz);

#ifdef __cplusplus
}
#endif
#endif /* __MENU_H */
