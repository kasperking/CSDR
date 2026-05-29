# Hardware Configuration

Firmware hardware options are selected before building via a configuration tool.
No source files need manual editing when switching boards or panels.

---

## Quick start — interactive (recommended)

```
python tools/hw_config.py
```

A numbered menu lets you choose every hardware option.  When you confirm,
the tool writes `BSP/Inc/hw_config_active.h` and saves your selections to
`config/hw_config_state.json`.  Then rebuild.

---

## Quick start — non-interactive rebuild

After running the interactive menu at least once:

```
python tools/hw_config.py --apply
```

Reads the saved state and regenerates the header without showing any menus.
Use this after a CubeMX regeneration or clean build.

---

## Quick start — profile by name (scripting / CI)

```
python tools/select_hw_profile.py hw_test_fmc
```

Applies a named preset directly.  Available presets:

| Name | Description |
|---|---|
| `hw_test_fmc` | ST7796 landscape BGR, MEDIUM GPIO, test-board timing |
| `hw_prod_v1` | ST7796 landscape BGR, VERY_HIGH GPIO, production timing |
| `hw_long_fpc_debug` | ST7796 landscape BGR, MEDIUM GPIO, relaxed FMC timing |

---

## Interactive menu overview

```
========================================================
  CSDR Firmware -- Hardware Configuration
========================================================
  1.  LCD Controller      : ST7796
  2.  LCD Orientation     : Landscape BGR
  3.  FMC Bus Width       : 8-bit
  4.  GPIO Speed          : MEDIUM
  5.  Board Type          : Test board
  6.  DMA Chunk Rows      : 8

  Resolved:
    LCD    : 480 x 320  (HW_LCD_PANEL = 1)
    MADCTL : 0xE8  [MY|MX|MV|BGR  landscape, BGR filter]
    FMC    : ADDR_SETUP=2  DATA_SETUP=10  BUS_TURN=15
    GPIO   : GPIO_SPEED_FREQ_MEDIUM
--------------------------------------------------------
  7.  Load preset
  8.  Generate config and exit
   0.  Exit without generating
```

Select a number to change a setting, choose **Load preset** to apply a
named hardware profile, or choose **Generate** to write the header.

---

## What each setting controls

| Setting | Options | Effect |
|---|---|---|
| LCD Controller | ST7796, ST7789 | Selects init sequence and MADCTL base |
| LCD Orientation | Landscape BGR/RGB, Portrait | Sets MADCTL byte and LCD_W/LCD_H |
| FMC Bus Width | 8-bit | Bus width (16-bit not yet supported) |
| GPIO Speed | LOW / MEDIUM / HIGH / VERY_HIGH | FMC GPIO drive strength |
| Board Type | Test / Production / Long FPC | FMC timing preset (AHB cycles) |
| DMA Chunk Rows | 4 / 8 / 16 | Spectrum strip height for async DMA |
| HSE Source | Crystal / TCXO / Bypass | Sets RCC_HSE_ON or RCC_HSE_BYPASS |
| HSE Frequency | 24 / 25 / 26 MHz or custom | PLL input; determines SYSCLK and SAI1 |
| Ext NVM Storage | None / I2C EEPROM / SPI EEPROM / W25Q NOR | Generates storage type flags and capability flags |
| W25Q Model *(sub)* | W25Q16/32/64/128 / Custom Mbit | Geometry defines; shown only when W25Q NOR selected |

### MADCTL resolution table

| Controller | Orientation | MADCTL | Description |
|---|---|---|---|
| ST7796 | Landscape BGR | `0xE8` | MY\|MX\|MV\|BGR |
| ST7796 | Landscape RGB | `0xE0` | MY\|MX\|MV |
| ST7796 | Portrait | `0x08` | BGR only |
| ST7789 | Landscape BGR | `0x68` | MX\|MV\|BGR |
| ST7789 | Landscape RGB | `0x60` | MX\|MV |
| ST7789 | Portrait | `0x08` | BGR only |

### FMC timing presets (AHB cycles at 200 MHz HCLK)

| Board Type | ADDR_SETUP | DATA_SETUP | Use case |
|---|---|---|---|
| Test board | 2 | 10 | Development, short FPC on bench |
| Production board | 1 | 8 | Controlled-impedance PCB traces |
| Long FPC debug | 4 | 14 | >100 mm FPC extension cable |

### HSE source and PLL resolution

The tool computes integer PLL1/PLL2 dividers from the chosen HSE frequency.

**Target frequencies (STM32H750, RM0433):**

| Clock | Target | Constraint |
|---|---|---|
| SYSCLK (PLL1P) | 480 MHz exact | Must be exact integer — error = build failure |
| SAI1 (PLL2P) | 12.288 MHz | Minimised error; typical result < 200 Hz at common HSE values |

**PLL results for common HSE frequencies:**

| HSE | PLL1 M/N/P | SYSCLK | PLL2 M/N/P | SAI1 | SAI error |
|---|---|---|---|---|---|
| 24 MHz (crystal) | 2/80/2 | 480 MHz | 5/64/25 | 12.2880 MHz | 0 Hz (exact) |
| 25 MHz (default) | 5/192/2 | 480 MHz | 2/58/59 | 12.2881 MHz | 136 Hz |
| 26 MHz (TCXO) | 13/480/2 | 480 MHz | 17/233/29 | 12.2880 MHz | 32 Hz |

The algorithm keeps VCO_out = 960 MHz for all cases (PLLP = 2), so PLL1Q and
PLL1R outputs are unchanged from their CubeMX defaults.

**HSE source modes:**

| Source | RCC mode | Notes |
|---|---|---|
| Crystal | `RCC_HSE_ON` | Passive oscillator, startup time ~2 ms |
| TCXO | `RCC_HSE_BYPASS` | Active clock input on OSC_IN pin only |
| Bypass | `RCC_HSE_BYPASS` | External square-wave signal |

**What the tool patches (in addition to `hw_config_active.h`):**

| File | What changes |
|---|---|
| `Core/Src/main.c` | `HSEState`, PLL1 M/N/P/RGE, PLL2 M/N/P/RGE |
| `Core/Inc/stm32h7xx_hal_conf.h` | `HSE_VALUE` literal |
| `Core/Src/system_stm32h7xx.c` | `HSE_VALUE` literal |

---

## Generated header

`tools/hw_config.py` writes `BSP/Inc/hw_config_active.h`:

```c
/* hw_config_active.h -- CSDR Hardware Configuration (auto-generated)
 * ...
 * HSE        : 25.000 MHz  CRYSTAL
 * SYSCLK     : 480 MHz  (PLL1 M=5 N=192 P=2)
 * SAI1       : 12.2881 MHz  (PLL2 M=2 N=58 P=59)
 */

#ifndef HW_CONFIG_ACTIVE_H
#define HW_CONFIG_ACTIVE_H

#define HW_LCD_PANEL        1   /* ST7796 */
#define LCD_W               480U
#define LCD_H               320U

#define HW_LCD_MADCTL       0xE8U

#define HW_FMC_ADDR_SETUP   2U
/* ... FMC timing ... */
#define HW_FMC_GPIO_SPEED   GPIO_SPEED_FREQ_MEDIUM
#define HW_DMA_CHUNK_ROWS   8U

#define HW_HSE_FREQ_HZ      25000000UL
#define HW_HSE_RCC_MODE     RCC_HSE_ON

#define HW_PLL1_M           5U
#define HW_PLL1_N           192U
#define HW_PLL1_P           2U
#define HW_PLL1_VCIRANGE    RCC_PLL1VCIRANGE_2

#define HW_PLL2_M           2U
#define HW_PLL2_N           58U
#define HW_PLL2_P           59U
#define HW_PLL2_VCIRANGE    RCC_PLL2VCIRANGE_3

#endif /* HW_CONFIG_ACTIVE_H */
```

### How the firmware picks it up

```
lcd_panel_config.h
  └── #include "hw_config_active.h"     ← generated active config
        defines: HW_LCD_PANEL, LCD_W, LCD_H, HW_LCD_MADCTL, HW_FMC_*, ...

lcd_bus_fmc.h   → lcd_panel_config.h   ← HW_LCD_MADCTL used for MADCTL write
sdr_ui.h        → lcd_bus_fmc.h        ← HW_DMA_CHUNK_ROWS → SPEC_CHUNK_ROWS
main.c          → lcd_bus_fmc.h        ← HW_FMC_* used in FMC timing re-init
stm32h7xx_hal_msp.c  (user-code block) ← HW_FMC_GPIO_SPEED for GPIO speed
```

`LCD_W` and `LCD_H` are defined directly in the generated header so that
portrait and all orientation combinations resolve correctly without any
conditional logic in `lcd_panel_config.h`.

### CubeMX regeneration

CubeMX regenerates literal values in `main.c` and `stm32h7xx_hal_msp.c`.
The tool handles two categories differently:

**FMC timing and GPIO speed** — applied via `USER CODE` blocks that survive
regeneration:
- `USER CODE BEGIN FMC_Init 2` — second `HAL_SRAM_Init` with `HW_FMC_*`.
- `USER CODE BEGIN FMC_MspInit 1` — re-initialises FMC GPIO with `HW_FMC_GPIO_SPEED`.

**HSE and PLL values** — CubeMX regenerates these as literals outside USER CODE
blocks.  The tool patches them with regex directly after generation:
- `SystemClock_Config` — `HSEState`, `PLLM/N/P/PLLRGE`.
- `PeriphCommonClock_Config` — `PLL2M/N/P/RGE`.
- `stm32h7xx_hal_conf.h` and `system_stm32h7xx.c` — `HSE_VALUE`.

**After every CubeMX regeneration:**
```
python tools/hw_config.py --apply
```
This re-patches all files and regenerates the header in one step.

---

## External NVM storage

### Storage type selection

| Menu option | `HW_STORAGE_*` flag | Type ID | Notes |
|---|---|---|---|
| None | `HW_STORAGE_NONE = 1` | 0 | No NVM fitted; all capability flags = 0 |
| I2C EEPROM | `HW_STORAGE_I2C_EE = 1` | 1 | AT24Cxx, M24xxx; `HAS_PERSISTENT` only |
| SPI EEPROM | `HW_STORAGE_SPI_EE = 1` | 2 | M95xxx, CAT25xxx; `HAS_PERSISTENT` only |
| W25Q NOR Flash | `HW_STORAGE_W25Q = 1` | 3 | W25Q16–W25Q128+; full capability flags |

Future variants (type IDs 4–7) use the same flag pattern and are reserved as
`HW_STORAGE_FRAM`, `HW_STORAGE_QSPI_NOR`, `HW_STORAGE_NAND`, `HW_STORAGE_SD`.

### W25Q model selection

Shown in the menu only when W25Q NOR Flash is selected.

| Model | Capacity | Notes |
|---|---|---|
| W25Q16 | 16 Mbit (2 MB) | Minimum for settings + small buffers |
| W25Q32 | 32 Mbit (4 MB) | |
| W25Q64 | 64 Mbit (8 MB) | Meets `HW_HAS_LARGE_NVM` threshold |
| W25Q128 | 128 Mbit (16 MB) | **CSDR default** — fitted on current board |
| Custom | 1–16384 Mbit | Enter exact capacity in Mbit |

All W25Q variants share the same SPI NOR geometry:
`PAGE_SIZE = 256 B`, `SECTOR_SIZE = 4096 B`, `BLOCK32 = 32768 B`, `BLOCK64 = 65536 B`.

### Capability flags

| Flag | Set when | SDR use cases |
|---|---|---|
| `HW_HAS_PERSISTENT_STORAGE` | Any NVM fitted | Settings storage (`Flash_Settings_t`), calibration data, band memory |
| `HW_HAS_LARGE_NVM` | W25Q >= 8 Mbit (>= 1 MB usable) | IQ capture buffering, audio snapshots, extended logging |
| `HW_SUPPORTS_WATERFALL_CACHE` | Same as `HW_HAS_LARGE_NVM` | Waterfall snapshot storage, playback, IQ demodulation replay |

Firmware usage pattern:

```c
#include "hw_config_active.h"

#if HW_STORAGE_W25Q
    /* W25Q driver active — Flash_Settings_t, sector layout, etc. */
    Flash_LoadSettings(&g_flash, &fs);
#elif HW_STORAGE_I2C_EE || HW_STORAGE_SPI_EE
    /* Small EEPROM — settings only, no large buffers */
#else
    /* HW_STORAGE_NONE — use RAM-only defaults */
#endif

#if HW_HAS_LARGE_NVM
    /* Allocate IQ capture ring in flash free area */
#endif

#if HW_SUPPORTS_WATERFALL_CACHE
    /* Enable waterfall snapshot save / restore */
#endif
```

### Flash sector layout (W25Q NOR, current firmware)

```
Sector 0   0x000000 .. 0x000FFF   4 KB   Settings (Flash_Settings_t + CRC32)
Sector 1   0x001000 .. 0x001FFF   4 KB   SI5351 band calibration (future)
Sectors 3-40  0x003000 .. 0x028FFF  152 KB  Boot logo (320×240 RGB565)
0x029000 onwards                   Free — IQ capture / waterfall cache
```

### Future-proofing

New storage variants require only:

1. Add a new entry to `STORAGE_TYPE_ID` in `tools/hw_config.py` (next free ID).
2. Add a choice to `SETTINGS["storage_type"]["choices"]`.
3. Add a `HW_STORAGE_<NAME>` flag line in `_storage_header_section()`.
4. If the variant has sub-options (model, capacity), add a child setting with
   `"parent": "storage_type"` and `"parent_val": "<NEW_TYPE>"`.
5. No changes to the firmware include chain — `hw_config_active.h` propagates
   through `lcd_panel_config.h` to all translation units.

---

## Validation

The interactive tool rejects or warns on:

| Condition | Severity |
|---|---|
| 16-bit FMC bus selected | Error — not implemented |
| HSE outside 1–50 MHz | Error — out of STM32H7 PLL input range |
| No integer PLL1 solution for SYSCLK = 480 MHz | Error — try 24/25/26 MHz |
| ST7789 + landscape orientation | Warning — non-native, verify MADCTL |
| ST7796 + portrait | Warning — UI zones designed for landscape |
| Production board + MEDIUM/LOW GPIO | Warning — VERY_HIGH recommended |
| Test board + VERY_HIGH GPIO | Warning — ringing risk on unmatched wiring |
| ST7789 + production timing | Warning — untested combination |
| SAI1 error > 1% (>122 kHz) | Warning — USB/audio drift will be significant |

---

## Standard workflow

```
  git checkout / CubeMX regen
          │
          ▼
  python tools/hw_config.py       ← interactive, or
  python tools/hw_config.py --apply   ← reuse saved state
          │  writes BSP/Inc/hw_config_active.h
          │  writes config/hw_config_state.json
          ▼
  Build All (CubeIDE  or  make)
          │
          ▼
  Flash and test
```

`BSP/Inc/hw_config_active.h` is tracked in git (default = hw_test_fmc values).
`config/hw_config_state.json` is also tracked.  Commit both when you
intentionally change the default configuration for a branch.

---

## Adding a new hardware option

### New option value (e.g. a new board type)

1. Add an entry to `FMC_TIMING` in `tools/hw_config.py`.
2. Add the corresponding choice tuple to `SETTINGS["board_type"]["choices"]`.
3. Run the tool and select the new option.

### New setting category (e.g. codec type)

1. Add a new key to `SETTINGS` in `tools/hw_config.py`.
2. If it affects header output, add a `#define HW_CODEC_*` line to
   `generate_header()`.
3. Add validation rules to `validate()` if needed.
4. The firmware includes `hw_config_active.h` through the existing chain —
   no include-path changes required.

### New named preset

Add an entry to `PRESETS` in `tools/hw_config.py`:

```python
("hw_my_board", {
    "label": "My custom board -- ST7789 portrait",
    "config": {
        "lcd_controller":  "ST7789",
        "lcd_orientation": "PORTRAIT",
        "fmc_bus_width":   "8BIT",
        "gpio_speed":      "MEDIUM",
        "board_type":      "TEST",
        "dma_chunk_rows":  "8",
        "hse_source":      "CRYSTAL",
        "hse_freq_hz":     "25000000",
        "storage_type":    "W25Q_NOR",
        "w25q_model":      "W25Q128",
    },
}),
```

The preset is immediately available via **Load preset** in the interactive menu
and via `select_hw_profile.py` if you also create a matching `.h` file in
`config/hw_profiles/`.
