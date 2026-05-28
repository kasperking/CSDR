# CSDR – Hướng dẫn cài đặt
## Tạo project từ CubeMX → copy BSP → patch → build

**MCU:** STM32H750VBT6 (480 MHz Cortex-M7) · **Display:** ST7796 480×320 FMC (default) hoặc ST7789 240×320 FMC · **Last updated:** 2026-05-28

---

## Bước 1: Tạo project từ file CSDR.ioc

1. Mở **STM32CubeIDE 2.1.1**
2. `File → New → STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)`
3. Chọn file **CSDR.ioc** → Next → đặt tên project `CSDR` → Finish
4. Đợi CubeIDE generate code (tạo đầy đủ Drivers/, USB_DEVICE/, startup, v.v.)
5. Sau khi generate xong: **không cần mở .ioc nữa**

---

## Bước 2: Copy thư mục BSP vào project

Copy thư mục **`BSP/`** (từ file zip này) vào thư mục gốc của project CSDR:

```
CSDR/            ← project root (do CubeIDE tạo)
├── BSP/         ← copy vào đây ✓
│   ├── Inc/
│   └── Src/
├── Core/
├── Drivers/
├── Middlewares/
├── USB_DEVICE/
└── CSDR.ioc
```

---

## Bước 3: Thêm BSP vào CubeIDE project

**3a. Thêm source folder:**

Right-click project → `Properties → C/C++ General → Paths and Symbols → Source Location`
→ `Add Folder` → chọn `BSP/Src` → OK

**3b. Thêm include path:**

Right-click project → `Properties → C/C++ Build → Settings → MCU GCC Compiler → Include paths`
→ `Add → Workspace` → chọn `BSP/Inc` → OK

---

## Bước 4: Patch các file CubeMX generate

Có **4 file** cần patch. Chỉ thêm code vào trong các block `/* USER CODE */` — CubeMX sẽ giữ nguyên khi regenerate.

---

### 4a. `Core/Src/main.c`

#### Includes (USER CODE BEGIN Includes)

```c
/* USER CODE BEGIN Includes */
#include "csdr_app.h"
#include "lcd_bus_fmc.h"
/* USER CODE END Includes */
```

#### MPU override — CRITICAL (USER CODE BEGIN SysInit)

> **KHÔNG XÓA BLOCK NÀY.** CubeMX tạo MPU_Config() đặt SRAM1 (0x24000000) thành NO_ACCESS — SAI DMA buffer nằm ở đó sẽ gây MemManage fault ngay khi boot. Block này re-program Region 0 thành FULL_ACCESS (non-cacheable) để DMA hoạt động.

```c
  /* USER CODE BEGIN SysInit */
  {
    MPU_Region_InitTypeDef rg = {0};
    HAL_MPU_Disable();
    rg.Enable           = MPU_REGION_ENABLE;
    rg.Number           = MPU_REGION_NUMBER0;
    rg.BaseAddress      = 0x24000000U;
    rg.Size             = MPU_REGION_SIZE_512KB;
    rg.SubRegionDisable = 0x00U;
    rg.TypeExtField     = MPU_TEX_LEVEL1;
    rg.AccessPermission = MPU_REGION_FULL_ACCESS;
    rg.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    rg.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    rg.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    rg.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&rg);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  }
  /* USER CODE END SysInit */
```

#### Init order — verify sau mỗi lần regen

Trong phần generated code (giữa `USER CODE END SysInit` và `USER CODE BEGIN 2`), đảm bảo thứ tự:

```c
  MX_FMC_Init();          // ← PHẢI trước USB
  MX_USB_DEVICE_Init();   // ← PHẢI sau FMC
```

> CubeMX đôi khi đổi thứ tự này khi regen. Nếu USB xuất hiện trước FMC, swap lại thủ công.

#### Init + Loop

```c
  /* USER CODE BEGIN 2 */
  CSDR_Init();
  /* USER CODE END 2 */
```

```c
    /* USER CODE BEGIN 3 */
    CSDR_Loop();
    /* USER CODE END 3 */
```

#### FMC timing + LCD init (USER CODE BEGIN FMC_Init 2)

Nằm bên trong hàm `MX_FMC_Init()`, sau `HAL_SRAM_Init`:

```c
  /* USER CODE BEGIN FMC_Init 2 */
  {
    FMC_NORSRAM_TimingTypeDef hw_tim = {0};
    hw_tim.AddressSetupTime      = HW_FMC_ADDR_SETUP;
    hw_tim.AddressHoldTime       = HW_FMC_ADDR_HOLD;
    hw_tim.DataSetupTime         = HW_FMC_DATA_SETUP;
    hw_tim.BusTurnAroundDuration = HW_FMC_BUS_TURN;
    hw_tim.CLKDivision           = HW_FMC_CLK_DIV;
    hw_tim.DataLatency           = HW_FMC_DATA_LATENCY;
    hw_tim.AccessMode            = FMC_ACCESS_MODE_A;
    HAL_SRAM_Init(&hsram1, &hw_tim, NULL);
  }
  LCD_Bus_Init();
  LCD_DMA_Init();
  /* USER CODE END FMC_Init 2 */
```

---

### 4b. `Core/Src/stm32h7xx_hal_msp.c`

#### GPIO speed override (USER CODE BEGIN FMC_MspInit 1)

Nằm bên trong `HAL_FMC_MspInit()`, sau khi GPIO được khởi tạo:

```c
  /* USER CODE BEGIN FMC_MspInit 1 */
  {
    GPIO_InitTypeDef gs = {0};
    gs.Mode  = GPIO_MODE_AF_PP;
    gs.Pull  = GPIO_NOPULL;
    gs.Speed = HW_FMC_GPIO_SPEED;
    gs.Alternate = GPIO_AF12_FMC;
    gs.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|
             GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gs);
    gs.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &gs);
    gs.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gs);
  }
  /* USER CODE END FMC_MspInit 1 */
```

---

### 4c. `Core/Src/stm32h7xx_it.c`

#### Includes

```c
/* USER CODE BEGIN Includes */
#include "csdr_app.h"
#include "lcd_dma.h"
/* USER CODE END Includes */
```

#### External variables (USER CODE BEGIN EV)

```c
/* USER CODE BEGIN EV */
extern DMA_HandleTypeDef  hdma_sai1_a;
extern DMA_HandleTypeDef  hdma_sai1_b;
extern SAI_HandleTypeDef  hsai_BlockB1;
extern PCD_HandleTypeDef  hpcd_USB_OTG_FS;
/* USER CODE END EV */
```

#### SysTick callback (USER CODE BEGIN SysTick_IRQn 1)

```c
  /* USER CODE BEGIN SysTick_IRQn 1 */
  CSDR_SysTickCallback();
  /* USER CODE END SysTick_IRQn 1 */
```

#### LCD async DMA IRQ (USER CODE BEGIN 1 — cuối file)

```c
/* USER CODE BEGIN 1 */
void DMA2_Stream0_IRQHandler(void)
{
  LCD_DMA_IRQHandler();
}
/* USER CODE END 1 */
```

---

### 4d. `USB_DEVICE/App/usbd_cdc_if.c`

#### Include

```c
/* USER CODE BEGIN INCLUDE */
#include "csdr_app.h"
/* USER CODE END INCLUDE */
```

#### Exported variables (USER CODE BEGIN EXPORTED_VARIABLES)

```c
/* USER CODE BEGIN EXPORTED_VARIABLES */
extern uint8_t Composite_CDC_Transmit(USBD_HandleTypeDef *pdev, uint8_t *buf, uint16_t len);
void CSDR_CDC_Receive(uint8_t *buf, uint32_t len);
/* USER CODE END EXPORTED_VARIABLES */
```

#### CDC_Init_FS — USER CODE BEGIN 3

> **KHÔNG** gọi `USBD_CDC_SetTxBuffer` / `USBD_CDC_SetRxBuffer` ở đây — `pClassData` còn NULL và gây ghost write vào ITCM.

```c
  /* USER CODE BEGIN 3 */
  return (USBD_OK);
  /* USER CODE END 3 */
```

#### CDC_Receive_FS — USER CODE BEGIN 6

> **KHÔNG** gọi `USBD_CDC_SetRxBuffer` / `USBD_CDC_ReceivePacket` — composite driver (`usb_composite.c`) tự re-arm endpoint. Gọi thêm sẽ double-arm và corrupt CDC RX.

```c
  /* USER CODE BEGIN 6 */
  CSDR_CDC_Receive(Buf, *Len);
  return (USBD_OK);
  /* USER CODE END 6 */
```

#### CDC_Transmit_FS — USER CODE BEGIN 7

```c
  /* USER CODE BEGIN 7 */
  result = (uint8_t)Composite_CDC_Transmit(&hUsbDeviceFS, Buf, Len);
  /* USER CODE END 7 */
```

---

## Bước 5: Cấu hình phần cứng (hw_config.py)

Chạy tool để generate `BSP/Inc/hw_config_active.h` và patch PLL/FMC trong `main.c`:

```
python tools/hw_config.py           # interactive wizard (lần đầu)
python tools/hw_config.py --apply   # reuse saved state (sau regen)
```

Hoặc apply một preset:

```
python tools/select_hw_profile.py hw_test_fmc     # bench/dev board
python tools/select_hw_profile.py hw_prod_v1      # production board
```

Xem **HARDWARE_PROFILES.md** để biết thêm chi tiết về các option và preset.

---

## Bước 6: Build

`Project → Build All` (Ctrl+B)

Nếu build thành công → Flash với `Run → Debug`.

---

## Checklist sau mỗi lần CubeMX regenerate

Sau mỗi lần regenerate từ .ioc, kiểm tra thủ công:

- [ ] `USER CODE BEGIN SysInit` — MPU SRAM1 override block còn nguyên
- [ ] Init order: `MX_FMC_Init()` trước `MX_USB_DEVICE_Init()`
- [ ] `USER CODE BEGIN FMC_Init 2` — FMC timing + `LCD_Bus_Init()` + `LCD_DMA_Init()` còn nguyên
- [ ] `USER CODE BEGIN FMC_MspInit 1` — GPIO speed override còn nguyên
- [ ] USB composite descriptor còn đủ 5 interface (AC + AS_OUT + AS_IN + CDC_CTRL + CDC_DATA)
- [ ] CubeMX không restore `USBD_CDC_SetRxBuffer` / `USBD_CDC_ReceivePacket` vào `CDC_Receive_FS`
- [ ] `HAL_SAI_RxHalfCpltCallback` / `HAL_SAI_RxCpltCallback` / `HAL_SAI_TxHalfCpltCallback` / `HAL_SAI_TxCpltCallback` hooks trong `csdr_app.c` còn hoạt động
- [ ] **KHÔNG** có `USB_Audio_WriteRX` trong DMA ISR (chỉ gọi từ main loop)
- [ ] Chạy lại `python tools/hw_config.py --apply`

---

## Cấu trúc sau khi setup

```
CSDR/
├── BSP/
│   ├── Inc/
│   │   ├── csdr_app.h          ← Application API, VFO/SDR state types
│   │   ├── sdr_dsp.h           ← DSP engine (NCO, FFT, FIR, AGC, demod)
│   │   ├── sdr_ui.h            ← Zone layout defines (cả 2 panel)
│   │   ├── sdr_scan.h          ← SWR scan
│   │   ├── lcd_bus_fmc.h       ← FMC 8080 LCD bus + MPU setup
│   │   ├── lcd_dma.h           ← DMA2 Stream0 async pixel push
│   │   ├── lcd_render.h        ← Primitive drawing helpers
│   │   ├── lcd_panel_config.h  ← Panel select (ST7796/ST7789)
│   │   ├── lcd_test_fmc.h      ← FMC bus test (dev only)
│   │   ├── hw_config_active.h  ← AUTO-GENERATED bởi hw_config.py
│   │   ├── bpf_lpf.h           ← BPF FST3253 + LPF 74HC238
│   │   ├── boot_dfu.h          ← DFU bootloader entry
│   │   ├── cal.h               ← Calibration parameters
│   │   ├── diag.h              ← Debug/diagnostic vars
│   │   ├── encoder.h           ← Encoder quadrature (TIM3)
│   │   ├── fsdr_analog.h       ← Power/SWR/ALC/Voltage/Fan/NTC
│   │   ├── input_scan.h        ← Keys via PCA9555 I2C expander
│   │   ├── menu.h              ← Menu overlay
│   │   ├── pca9555.h           ← PCA9555 16-bit I2C GPIO expander
│   │   ├── pe4302.h            ← RX Attenuator (bit-bang SPI)
│   │   ├── runtime_diag.h      ← Runtime diagnostics
│   │   ├── si5351.h            ← QSD/QSE clock (CLK0 × 4)
│   │   ├── usb_audio.h         ← USB Audio IQ streaming
│   │   ├── usb_cat.h           ← USB CAT TS-2000 emulation
│   │   ├── w25q128.h           ← SPI Flash (SPI3)
│   │   └── wm8731.h            ← Audio codec (SAI I²S)
│   └── Src/
│       ├── csdr_app.c          ← CSDR_Init, CSDR_Loop, mode dispatch
│       ├── sdr_dsp.c           ← All DSP: NCO, FFT, FIR, AGC, demod, TX
│       ├── sdr_ui.c            ← Display rendering, S-meter, menu
│       ├── sdr_scan.c          ← SWR sweep
│       ├── lcd_bus_fmc.c       ← FMC init, MPU Region 1, LCD_Bus_Init
│       ├── lcd_dma.c           ← DMA2 async push, ISR handler
│       ├── lcd_render.c        ← Pixel/rect/text primitives
│       ├── lcd_test_fmc.c      ← FMC test patterns
│       ├── bpf_lpf.c
│       ├── boot_dfu.c
│       ├── cal.c
│       ├── diag.c
│       ├── encoder.c
│       ├── fsdr_analog.c
│       ├── input_scan.c
│       ├── menu.c
│       ├── pca9555.c
│       ├── pe4302.c
│       ├── runtime_diag.c
│       ├── si5351.c
│       ├── usb_audio.c
│       ├── usb_cat.c
│       ├── w25q128.c
│       └── wm8731.c
├── Core/
│   └── Src/
│       ├── main.c              ← CubeMX generated + USER CODE patches (Bước 4a)
│       ├── stm32h7xx_it.c      ← CubeMX generated + USER CODE patches (Bước 4c)
│       └── stm32h7xx_hal_msp.c ← CubeMX generated + USER CODE patches (Bước 4b)
├── Drivers/                    ← CubeMX generated (KHÔNG sửa)
├── Middlewares/                ← CubeMX generated (KHÔNG sửa)
├── USB_DEVICE/
│   └── App/
│       └── usbd_cdc_if.c       ← USER CODE patches (Bước 4d)
├── tools/
│   ├── hw_config.py            ← Hardware config wizard
│   └── select_hw_profile.py    ← Apply named preset
├── config/
│   ├── hw_config_state.json    ← Saved hw_config selections
│   └── hw_profiles/            ← Named preset .h files
└── CSDR.ioc
```

---

## Lưu ý quan trọng

- **KHÔNG** copy/override các file trong `Core/Src/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/` từ zip cũ
- Chỉ thêm code vào bên trong `/* USER CODE BEGIN */` ... `/* USER CODE END */`
- CubeMX sẽ **giữ nguyên** USER CODE khi regenerate
- `BSP/Inc/hw_config_active.h` là file **auto-generated** — đừng sửa tay, chạy `hw_config.py`
- Sau mỗi CubeMX regen, chạy lại `python tools/hw_config.py --apply`

---

## Phần cứng

### LCD (FMC 8080-mode — KHÔNG phải SPI)

| Tín hiệu | Pin | Chức năng |
|---|---|---|
| FMC_D0–D1 | PD14, PD15 | Data bus bit 0–1 |
| FMC_D2–D3 | PD0, PD1 | Data bus bit 2–3 |
| FMC_D4–D7 | PE7, PE8, PE9, PE10 | Data bus bit 4–7 |
| FMC_NWE (WR) | PD5 | Write strobe |
| FMC_NOE (RD) | PD4 | Read strobe |
| FMC_NE1 (CS) | PC7 | Chip select |
| FMC_A16 (RS) | PD11 | Register/Data select |
| LCD_RESET | PD13 | Hardware reset |
| LCD_BL (TIM8_CH4) | PC9 | Backlight PWM |

### Các ngoại vi khác

| Chức năng | Pins |
|---|---|
| SAI1 Audio (WM8731) | PE2 (MCLK_A), PE3 (SD_B), PE4 (FS_A), PE5 (SCK_A), PE6 (SD_A) |
| I2C1 — WM8731 + Si5351 | PB6 (SCL), PB7 (SDA) |
| I2C2 — PCA9555 keys | PB10 (SCL), PB11 (SDA) |
| Encoder TIM3 quadrature | PB4 (CH1), PB5 (CH2), PA10 (SW) |
| BPF FST3253 | PA4 (S1), PA5 (S2), PA6 (OE1/TX), PA7 (OE2/RX) |
| LPF 74HC238 | PA0 (A0), PA1 (A1), PA2 (A2) |
| PE4302 ATT (bit-bang SPI) | PC4 (DATA), PC5 (CLK), PB0 (LATCH) |
| Flash W25Q128 (SPI3) | PC10 (SCK), PC11 (MISO), PC12 (MOSI), PA15 (CS) |
| USB OTG FS | PA11 (DM), PA12 (DP) |
| ADC NTC | PC0 |
| ADC ALC | PC1 |
| ADC SWR forward/reflected | PC2, PC3 |
| ADC Supply voltage | PA3 |
| Fan PWM (TIM17_CH1) | PB9 |
| T/R relay | PB2 |
| PTT | PB12 |
| DIT / DAH paddle | PB13, PB14 |
| Power latch | PD12 |
| PW_HOLD | PB1 |
