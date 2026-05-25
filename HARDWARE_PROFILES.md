# Hardware Profiles

Firmware hardware configuration is selected via **hardware profiles** — small C
headers that define all board-specific values in one place.  Switch profiles
before building; no source files need manual editing.

---

## Quick start

```
python tools/select_hw_profile.py hw_test_fmc
# then rebuild in CubeIDE (Project → Build All) or via make
```

List available profiles:

```
python tools/select_hw_profile.py
```

---

## What a profile controls

| Macro | Description | Typical range |
|---|---|---|
| `HW_PROFILE_NAME` | String identifier (must match filename) | – |
| `HW_LCD_PANEL` | Panel chip: `1` = ST7796, `2` = ST7789 | 1 or 2 |
| `HW_LCD_MADCTL` | MADCTL byte sent to register 0x36 | `0x00`–`0xFF` |
| `HW_FMC_ADDR_SETUP` | FMC address setup time (AHB cycles) | 0–15 |
| `HW_FMC_ADDR_HOLD` | FMC address hold time | 1–15 |
| `HW_FMC_DATA_SETUP` | FMC data setup time | 1–255 |
| `HW_FMC_BUS_TURN` | FMC bus turn-around duration | 0–15 |
| `HW_FMC_CLK_DIV` | FMC clock division | 2–16 |
| `HW_FMC_DATA_LATENCY` | FMC data latency | 2–17 |
| `HW_FMC_GPIO_SPEED` | FMC GPIO drive strength | `GPIO_SPEED_FREQ_LOW/MEDIUM/HIGH/VERY_HIGH` |
| `HW_DMA_CHUNK_ROWS` | Spectrum DMA strip height (rows) | 1–64 |

**MADCTL bit layout** (ST7796/ST7789 compatible):
```
  bit 7: MY   bit 6: MX   bit 5: MV   bit 4: ML
  bit 3: BGR  bit 2: MH   bit 1: 0    bit 0: 0

  0xE8 = MY|MX|MV|BGR → ST7796 landscape, BGR filter  (test/prod board)
  0x08 =          BGR → ST7789 portrait,  BGR filter  (compact panel)
```

---

## Provided profiles

| Profile | Panel | FMC addr/data | GPIO speed | Use case |
|---|---|---|---|---|
| `hw_test_fmc` | ST7796 480×320 | 2 / 10 | MEDIUM | Development, test bench |
| `hw_prod_v1` | ST7796 480×320 | 1 / 8 | VERY_HIGH | Production PCB, short matched FPC |
| `hw_long_fpc_debug` | ST7796 480×320 | 4 / 14 | MEDIUM | Scope probing with >100 mm FPC extension |

---

## How switching works

1. `select_hw_profile.py` reads the chosen profile from `config/hw_profiles/`.
2. It validates all required macros and their value ranges.
3. It writes `BSP/Inc/hw_config_active.h` — a thin wrapper with the active
   `#define` values and the standard `HW_CONFIG_ACTIVE_H` include guard.
4. The include chain picks it up automatically:

```
lcd_panel_config.h
  └─ #include "hw_config_active.h"     ← generated active profile
       defines: HW_LCD_PANEL, HW_FMC_*, HW_DMA_CHUNK_ROWS

lcd_bus_fmc.h → lcd_panel_config.h     ← HW_LCD_MADCTL used for MADCTL write
sdr_ui.h      → lcd_bus_fmc.h          ← HW_DMA_CHUNK_ROWS → SPEC_CHUNK_ROWS
main.c        → lcd_bus_fmc.h          ← HW_FMC_* used in FMC timing re-init
stm32h7xx_hal_msp.c → lcd_bus_fmc.h   ← HW_FMC_GPIO_SPEED used in GPIO re-init
```

### CubeMX regeneration

`main.c` and `stm32h7xx_hal_msp.c` contain CubeMX-generated timing/GPIO literal
values.  The firmware applies the profile values on top via `USER CODE` blocks
that survive regeneration:

- **`USER CODE BEGIN FMC_Init 2`** (`main.c`) — calls `HAL_SRAM_Init` a second
  time with `HW_FMC_*` macro values, overriding the CubeMX literals.
- **`USER CODE BEGIN FMC_MspInit 1`** (`stm32h7xx_hal_msp.c`) — re-calls
  `HAL_GPIO_Init` for all FMC pins with `HW_FMC_GPIO_SPEED`.

After CubeMX regeneration, re-run `select_hw_profile.py` to confirm the active
profile is still correct (it rewrites `hw_config_active.h` from the source
profile), then rebuild.

---

## Creating a new profile

1. Copy the closest existing profile:
   ```
   cp config/hw_profiles/hw_test_fmc.h config/hw_profiles/hw_my_board.h
   ```

2. Edit all `HW_*` values to match the new hardware.

3. Set `HW_PROFILE_NAME` to match the filename stem exactly:
   ```c
   #define HW_PROFILE_NAME  "hw_my_board"
   ```

4. Activate and validate:
   ```
   python tools/select_hw_profile.py hw_my_board
   ```

5. Rebuild the firmware.

### New ST7789 compact-panel profile

Copy `hw_test_fmc.h`, set:
```c
#define HW_LCD_PANEL   2
#define HW_LCD_MADCTL  0x08U   /* BGR portrait */
```
Everything else (FMC timing, DMA rows) can remain the same.

---

## Intended workflow

```
 checkout / CubeMX regen
        │
        ▼
python tools/select_hw_profile.py <profile>
        │  writes BSP/Inc/hw_config_active.h
        ▼
Build All (CubeIDE or make)
        │
        ▼
Flash & test
```

`BSP/Inc/hw_config_active.h` is tracked in git (default = `hw_test_fmc`).
Commit it when you intentionally change the default profile for a branch.

---

## Validation rules

The script rejects a profile if:

- Any required macro is missing
- `HW_PROFILE_NAME` does not match the `.h` filename stem
- `HW_LCD_PANEL` is not `1` or `2`
- `HW_LCD_MADCTL` is outside `0x00`–`0xFF`
- Any FMC timing value is outside the STM32H7 RM0433 register limits
- `HW_FMC_GPIO_SPEED` is not one of the four HAL `GPIO_SPEED_FREQ_*` constants
- `HW_DMA_CHUNK_ROWS` is outside `1`–`64`
- Two profile filenames differ only in case (would be ambiguous on
  case-insensitive file systems)
