/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    menu.h
  * @brief   SDR Menu System – hierarchical 2-level (group → settings)
  *
  *  Navigation:
  *   MENU_KEY     → Mở/đóng menu (toggle)
  *   F1           → Lên (UP)
  *   F2           → Xuống (DOWN)
  *   ENC rotate   → Thay đổi giá trị (edit mode)
  *   ENC press    → Vào group / bắt đầu edit / xác nhận
  *   F4           → Back (group → root) / đóng menu
  *
  *  Cấu trúc menu (2 cấp):
  *
  *  Root
  *   ├─ [RX]     → AGC / NB / NR / ATT / Squelch / RIT / Span
  *   ├─ [Audio]  → Volume / Mic Gain / Digi Drive
  *   ├─ [Tuning] → Step / Band / Mode
  *   ├─ [TX]     → RF Power / Ext ALC / TX Low / TX High / VOX / VOX Gain / VOX Delay
  *   ├─ [System] → Backlight / USB / Calibration
  *   └─ SWR Scan  (root action)
  *
  *  Renderer: overlay trên vùng Spectrum (Y=ZONE_SPEC_Y)
  *  Scanline-based: mỗi item = 16px cao, width MENU_W, x=MENU_X
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

#define MENU_ITEM_COUNT      32U   /* 5 groups + 27 leaf items */
#define MENU_VISIBLE_ROWS     6U   /* Items shown at once; 6×16=96px  */
#define MENU_ITEM_H          16U   /* Height per item (px)  */
#define MENU_X               10U   /* Left edge             */
#define MENU_W  (LCD_W - 2U * MENU_X)  /* adaptive: 460 on ST7796, 220 on ST7789 */
#define MENU_Y               ZONE_SPEC_Y   /* Overlay on spectrum  */
#define MENU_HEADER_BG      0xF800U
#define MENU_GROUP_COLOR    0x07FFU   /* Cyan for group rows  */
#define MENU_BG_COLOR       0x0843U   /* Dark blue-gray       */
#define MENU_FG_COLOR       0xFFFFU   /* White text           */
#define MENU_SEL_COLOR      0xF800U   /* Red highlight        */
#define MENU_SEL_FG         0xFFFFU   /* White on selected    */
#define MENU_LBL_COLOR      0xA514U   /* Soft gray-cyan       */
#define MENU_VAL_COLOR      0xFFFFU   /* White value          */
#define MENU_BORDER_COLOR   0x10A2U   /* Dark subtle border   */

/* Well-known item indices (leaf items in the TX group) */
#define MENU_IDX_RFPOWER    21U
#define MENU_IDX_TXLOW      23U
#define MENU_IDX_TXHIGH     24U

/* Exported types ------------------------------------------------------------*/

typedef enum {
  MENU_TYPE_BOOL,       /* ON/OFF */
  MENU_TYPE_INT,        /* Integer range */
  MENU_TYPE_ENUM,       /* Fixed string options */
  MENU_TYPE_ACTION,     /* Immediate action (no value editing) */
  MENU_TYPE_GROUP,      /* Sub-menu group header */
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
  const char  *suffix;             /*!< Unit appended to INT value */
  int8_t       parent;             /*!< -1 = root; ≥0 = group index */
} MenuItem_t;

typedef struct {
  bool         open;           /*!< Menu đang hiển thị          */
  uint8_t      cursor;         /*!< Index vào view[] hiện tại   */
  uint8_t      scroll;         /*!< Scroll offset               */
  bool         editing;        /*!< Đang chỉnh sửa giá trị      */
  int8_t       current_group;  /*!< -1 = root; ≥0 = group index */
  uint8_t      view[MENU_ITEM_COUNT]; /*!< Filtered item indices */
  uint8_t      view_count;     /*!< Number of items in view[]   */
  uint8_t      item_count;     /*!< Always MENU_ITEM_COUNT       */
  MenuItem_t   items[MENU_ITEM_COUNT];
} Menu_Handle_t;

/* Exported variables --------------------------------------------------------*/
extern Menu_Handle_t g_menu;

/* ── Inline helpers ──────────────────────────────────────── */
static inline MenuItem_t *Menu_CurrentItem(Menu_Handle_t *m)
{
  if (m->cursor < m->view_count)
    return &m->items[m->view[m->cursor]];
  return NULL;
}

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

/* ── Apply callback type ─────────────────────────────── */
typedef void (*MenuApplyFn)(void);

/**
  * @brief  Load SDR state vào menu trước khi mở.
  */
void Menu_LoadFromSDR(Menu_Handle_t *m,
                       bool agc_fast, bool nb, bool nr, int16_t rit,
                       uint8_t vol, uint8_t mic_gain, uint8_t digi_gain,
                       uint8_t sq, uint32_t step,
                       uint8_t att, uint8_t band, uint8_t mode,
                       uint8_t usb_mode, uint8_t zoom,
                       bool ext_alc, uint8_t rf_power_pct, uint8_t pa_watts,
                       uint16_t tx_audio_low_hz, uint16_t tx_audio_high_hz,
                       int16_t rx_shift_hz,
                       bool notch_on, int16_t notch_hz,
                       bool vox_on, uint8_t vox_gain, uint16_t vox_delay,
                       MenuApplyFn apply_cb);

/**
  * @brief  Đọc giá trị từ menu ra SDR state sau khi đóng.
  */
void Menu_SaveToSDR(Menu_Handle_t *m,
                     bool *agc_fast, bool *nb, bool *nr, int16_t *rit,
                     uint8_t *vol, uint8_t *mic_gain, uint8_t *digi_gain,
                     uint8_t *sq, uint32_t *step,
                     uint8_t *att, uint8_t *band, uint8_t *mode,
                     uint8_t *usb_mode, uint8_t *zoom,
                     bool *ext_alc, uint8_t *rf_power,
                     uint16_t *tx_audio_low_hz, uint16_t *tx_audio_high_hz,
                     int16_t *rx_shift_hz,
                     bool *notch_on, int16_t *notch_hz,
                     bool *vox_on, uint8_t *vox_gain, uint16_t *vox_delay);

#ifdef __cplusplus
}
#endif
#endif /* __MENU_H */
