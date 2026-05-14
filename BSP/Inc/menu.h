/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    menu.h
  * @brief   SDR Menu System – MENU_KEY toggle, F1=UP, F2=DOWN, ENC=edit, F4=back
  *
  *  Navigation:
  *   MENU_KEY     → Mở/đóng menu (toggle)
  *   F1           → Lên (UP) trong menu / tăng giá trị
  *   F2           → Xuống (DOWN) / giảm giá trị
  *   ENC rotate   → Thay đổi giá trị đang chọn (nếu đang edit)
  *   ENC press    → Chọn / vào submenu / bắt đầu edit
  *   F3           → Xác nhận (OK) / về parent menu
  *   F4           → Thoát về menu trên / đóng menu
  *
  *  Cấu trúc menu:
  *
  *  Main
  *   ├─ AGC      [FAST / SLOW]
  *   ├─ NB       [ON / OFF]
  *   ├─ NR       [ON / OFF]
  *   ├─ RIT      [-999 .. +999 Hz]
  *   ├─ Volume   [0 .. 100]
  *   ├─ Squelch  [0 .. 100]
  *   ├─ Step     [1/10/100/1K/10K/100K Hz]
  *   ├─ ATT      [0 .. 31 dB]
  *   ├─ Band     [160m .. 6m]
  *   ├─ Mode     [AM/FM/USB/LSB/CW]
  *   ├─ BL       [0 .. 100% backlight]
  *   └─ USB      [CAT / Audio / Off]
  *
  *  Renderer: overlay trên vùng Spectrum (Y=62..149)
  *  Scanline-based: mỗi menu item là 16px cao, width 280px, x=20
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

#define MENU_ITEM_COUNT      16U   /* 12 settings + Span + Diagnostics + Calibration + SWR Scan */
#define MENU_VISIBLE_ROWS     6U   /* Items shown at once; 6×16=96px → total 120px fits 100→232 */
#define MENU_ITEM_H          16U   /* Height per item (px)  */
#define MENU_X               10U   /* Left edge             */
#define MENU_W              300U   /* Width (320-2*10)      */
#define MENU_Y               ZONE_SPEC_Y   /* Overlay on spectrum  */
#define MENU_HEADER_BG      0xF800U
#define MENU_BG_COLOR       0x0843U   /* Dark blue-gray     */
#define MENU_FG_COLOR       0xFFFFU   /* White text         */
#define MENU_SEL_COLOR      0xF800U   /* Teal highlight     */
#define MENU_SEL_FG         0xFFFFU   /* White on selected  */
#define MENU_LBL_COLOR      0xA514U   /* Soft gray-cyan     */
#define MENU_VAL_COLOR      0xFFFFU   /* White value        */
#define MENU_BORDER_COLOR   0x10A2U   /* Dark subtle border */

/* Exported types ------------------------------------------------------------*/

typedef enum {
  MENU_TYPE_BOOL,       /* ON/OFF */
  MENU_TYPE_INT,        /* Integer range */
  MENU_TYPE_ENUM,       /* Fixed string options */
  MENU_TYPE_ACTION,     /* Immediate action (no value editing) */
} MenuItemType_t;

typedef struct {
  const char  *label;          /*!< Item label string            */
  MenuItemType_t type;
  int32_t      min;            /*!< Min value (INT type)         */
  int32_t      max;            /*!< Max value (INT type)         */
  int32_t      step;           /*!< Step size (INT type)         */
  int32_t      *value_ptr;     /*!< Pointer to current value     */
  const char  **enum_strs;     /*!< String options (ENUM type)   */
  uint8_t      enum_count;     /*!< Number of enum options       */
  void        (*on_change)(void);  /*!< Callback khi thay đổi   */
} MenuItem_t;

typedef struct {
  bool         open;           /*!< Menu đang hiển thị          */
  uint8_t      cursor;         /*!< Vị trí cursor (0..COUNT-1)  */
  uint8_t      scroll;         /*!< Scroll offset               */
  bool         editing;        /*!< Đang chỉnh sửa giá trị      */
  MenuItem_t   items[MENU_ITEM_COUNT];
  uint8_t      item_count;
} Menu_Handle_t;

/* Exported variables --------------------------------------------------------*/
extern Menu_Handle_t g_menu;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Khởi tạo menu với tất cả items và con trỏ đến SDR state.
  */
void Menu_Init(Menu_Handle_t *m);

/**
  * @brief  Toggle mở/đóng menu (MENU_KEY).
  */
void Menu_Toggle(Menu_Handle_t *m);

/**
  * @brief  F1 key: di chuyển lên hoặc tăng giá trị.
  */
void Menu_Up(Menu_Handle_t *m);

/**
  * @brief  F2 key: di chuyển xuống hoặc giảm giá trị.
  */
void Menu_Down(Menu_Handle_t *m);

/**
  * @brief  ENC press: chọn item / bắt đầu edit.
  */
void Menu_Select(Menu_Handle_t *m);

/**
  * @brief  F3: confirm (alias cho select trong edit mode).
  */
void Menu_Confirm(Menu_Handle_t *m);

/**
  * @brief  F4: back/cancel.
  */
void Menu_Back(Menu_Handle_t *m);

/**
  * @brief  ENC rotate trong edit mode: thay đổi giá trị.
  * @param  delta  +1/-1
  */
void Menu_EncoderEdit(Menu_Handle_t *m, int32_t delta);

/**
  * @brief  Render menu overlay lên LCD.
  *         Dùng scanline renderer (640B DMA per dòng).
  */
void Menu_Render(Menu_Handle_t *m);

/**
  * @brief  Kiểm tra menu đang mở.
  */
static inline bool Menu_IsOpen(const Menu_Handle_t *m) { return m->open; }


/* ── Apply callback type ─────────────────────────────── */
typedef void (*MenuApplyFn)(void);

/**
  * @brief  Load SDR state vào menu trước khi mở.
  */
void Menu_LoadFromSDR(Menu_Handle_t *m,
                       bool agc_fast, bool nb, bool nr, int16_t rit,
                       uint8_t vol, uint8_t sq, uint32_t step,
                       uint8_t att, uint8_t band, uint8_t mode,
                       uint8_t usb_mode, uint8_t zoom, MenuApplyFn apply_cb);

/**
  * @brief  Đọc giá trị từ menu ra SDR state sau khi đóng.
  */
void Menu_SaveToSDR(Menu_Handle_t *m,
                     bool *agc_fast, bool *nb, bool *nr, int16_t *rit,
                     uint8_t *vol, uint8_t *sq, uint32_t *step,
                     uint8_t *att, uint8_t *band, uint8_t *mode,
                     uint8_t *usb_mode, uint8_t *zoom);

#ifdef __cplusplus
}
#endif
#endif /* __MENU_H */
