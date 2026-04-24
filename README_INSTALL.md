# CSDR – Hướng dẫn cài đặt
## Tạo project từ CubeMX → copy BSP → build

---

## Bước 1: Tạo project từ file CSDR.ioc

1. Mở **STM32CubeIDE 2.1.1**
2. `File → New → STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)`
3. Chọn file **CSDR.ioc** → Next → đặt tên project `CSDR` → Finish
4. Đợi CubeIDE generate code (sẽ tạo đầy đủ Drivers/, USB_DEVICE/, startup, v.v.)
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

## Bước 4: Patch 3 file do CubeMX generate

Mở file **`PATCH_main.c`** (trong zip) và copy-paste các đoạn code vào đúng vị trí:

### 4a. `Core/Src/main.c`

Tìm và thêm vào đúng các `USER CODE` block:

```c
/* USER CODE BEGIN Includes */
#include "csdr_app.h"         ← thêm dòng này
/* USER CODE END Includes */
```

```c
/* USER CODE BEGIN 2 */
  CSDR_Init();               ← thêm dòng này
/* USER CODE END 2 */
```

```c
  /* USER CODE BEGIN 3 */
    CSDR_Loop();             ← thêm dòng này
  /* USER CODE END 3 */
```

### 4b. `Core/Src/stm32h7xx_it.c`

Tìm hàm `SysTick_Handler`, thêm vào `USER CODE BEGIN SysTick_IRQn 1`:

```c
  /* USER CODE BEGIN SysTick_IRQn 1 */
  CSDR_SysTickCallback();    ← thêm dòng này
  /* USER CODE END SysTick_IRQn 1 */
```

### 4c. `USB_DEVICE/App/usbd_cdc_if.c`

Tìm hàm `CDC_Receive_FS`, thay thế nội dung trong `USER CODE BEGIN 6`:

```c
  /* USER CODE BEGIN 6 */
  CSDR_CDC_Receive(Buf, *Len);           ← thêm
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
```

---

## Bước 5: Build

`Project → Build All` (Ctrl+B)

Nếu build thành công → Flash vào board với `Run → Debug`.

---

## Cấu trúc sau khi setup

```
CSDR/
├── BSP/
│   ├── Inc/
│   │   ├── csdr_app.h      ← Application API
│   │   ├── bpf_lpf.h       ← BPF FST3253 + LPF 74HC238
│   │   ├── encoder.h       ← Encoder quadrature
│   │   ├── fsdr_analog.h   ← Power/SWR/ALC/Voltage/Fan/NTC
│   │   ├── menu.h          ← Menu overlay
│   │   ├── pe4302.h        ← RX Attenuator
│   │   ├── sdr_dsp.h       ← DSP engine
│   │   ├── si5351.h        ← QSD/QSE clock
│   │   ├── st7789.h        ← LCD mcHF UI
│   │   ├── usb_audio.h     ← USB Audio IQ streaming
│   │   ├── usb_cat.h       ← USB CAT TS-2000
│   │   ├── w25q128.h       ← SPI Flash
│   │   └── wm8731.h        ← Audio codec
│   └── Src/
│       ├── csdr_app.c      ← Application logic (CSDR_Init, CSDR_Loop)
│       ├── bpf_lpf.c
│       ├── encoder.c
│       ├── fsdr_analog.c
│       ├── menu.c
│       ├── pe4302.c
│       ├── sdr_dsp.c
│       ├── si5351.c
│       ├── st7789.c
│       ├── usb_audio.c
│       ├── usb_cat.c
│       ├── w25q128.c
│       └── wm8731.c
├── Core/
│   └── Src/
│       ├── main.c          ← CubeMX generated (chỉ thêm USER CODE)
│       ├── stm32h7xx_it.c  ← CubeMX generated (chỉ thêm USER CODE)
│       └── stm32h7xx_hal_msp.c ← CubeMX generated (KHÔNG sửa)
├── Drivers/                ← CubeMX generated (KHÔNG sửa)
├── Middlewares/            ← CubeMX generated (KHÔNG sửa)
├── USB_DEVICE/
│   └── App/
│       └── usbd_cdc_if.c  ← Chỉ thêm USER CODE BEGIN 6
└── CSDR.ioc
```

---

## Lưu ý quan trọng

- **KHÔNG** copy/override các file trong `Core/Src/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/` từ zip cũ
- Chỉ thêm code vào bên trong `/* USER CODE BEGIN */` ... `/* USER CODE END */`
- CubeMX sẽ **giữ nguyên** USER CODE khi regenerate

---

## Phần cứng (CSDR.ioc)

| Chức năng | Pins |
|---|---|
| SAI1 Audio | PE2/3/4/5/6 |
| LCD SPI1 | PA4/5/6/7, PB0/1/15 |
| Flash SPI3 | PC10/11/12, PD0 |
| I2C1 (WM8731+SI5351) | PB6/7 |
| Encoder TIM2 | PA0/1/2 |
| PE4302 ATT | PD8/9/10 |
| BPF FST3253 | PD11/12/13 |
| LPF 74HC238 | PC8/9, PA8 |
| USB OTG FS | PA11/12 |
| ADC NTC/V/ALC/SWR | PC0/1, PA3, PC2_C/PC3_C |
| Fan TIM3 | PC6 |
| LCD BL TIM1 | PB15 |
| Power | PB12/13 |
| Keys | PE7-15, PB10 |
