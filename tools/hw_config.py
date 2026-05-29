#!/usr/bin/env python3
"""
hw_config.py  --  CSDR firmware interactive hardware configuration tool.

    python tools/hw_config.py           # interactive menu (recommended)
    python tools/hw_config.py --apply   # apply saved config (non-interactive)
    python tools/hw_config.py --show    # print current saved config and exit

The tool writes BSP/Inc/hw_config_active.h (picked up by the firmware include
chain), patches PLL / HSE values in Core/Src/main.c and HSE_VALUE in
Core/Inc/stm32h7xx_hal_conf.h / Core/Src/system_stm32h7xx.c, and saves all
selections to config/hw_config_state.json so the next run can reload them.

See HARDWARE_PROFILES.md for full documentation.
"""

import argparse
import json
import os
import re
import sys
from collections import OrderedDict
from datetime import datetime


# ── Project paths ─────────────────────────────────────────────────────────────

ROOT        = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATE_FILE  = os.path.join(ROOT, "config", "hw_config_state.json")
OUTPUT_FILE = os.path.join(ROOT, "BSP", "Inc", "hw_config_active.h")
MAIN_C      = os.path.join(ROOT, "Core", "Src", "main.c")
CONF_H      = os.path.join(ROOT, "Core", "Inc", "stm32h7xx_hal_conf.h")
SYSTEM_C    = os.path.join(ROOT, "Core", "Src", "system_stm32h7xx.c")


# ── Menu catalogue ────────────────────────────────────────────────────────────
# Each entry:  (value_key, display_label)
# Double-space separates the short name (shown in main menu) from the detail.
# "custom_input": True  -- enables a CUSTOM sentinel that prompts for a value.
# "ask_custom":          -- "hz" (frequency Hz) or "mbit" (capacity Mbit).
# "parent"/"parent_val" -- setting only shown when parent matches parent_val.

SETTINGS = OrderedDict([
    ("lcd_controller", {
        "label":   "LCD Controller",
        "choices": OrderedDict([
            ("ST7796", "ST7796  (480x320 landscape chip)"),
            ("ST7789", "ST7789  (240x320 portrait chip)"),
        ]),
        "default": "ST7796",
    }),
    ("lcd_orientation", {
        "label":   "LCD Orientation",
        "choices": OrderedDict([
            ("LANDSCAPE_BGR", "Landscape BGR  (MY|MX|MV|BGR)"),
            ("LANDSCAPE_RGB", "Landscape RGB  (MY|MX|MV, no BGR swap)"),
            ("PORTRAIT",      "Portrait       (BGR only, no axis swap)"),
        ]),
        "default": "LANDSCAPE_BGR",
    }),
    ("fmc_bus_width", {
        "label":   "FMC Bus Width",
        "choices": OrderedDict([
            ("8BIT",  "8-bit   (current hardware)"),
            ("16BIT", "16-bit  [NOT SUPPORTED on current boards]"),
        ]),
        "default": "8BIT",
    }),
    ("gpio_speed", {
        "label":   "GPIO Speed",
        "choices": OrderedDict([
            ("LOW",       "LOW        GPIO_SPEED_FREQ_LOW"),
            ("MEDIUM",    "MEDIUM     GPIO_SPEED_FREQ_MEDIUM"),
            ("HIGH",      "HIGH       GPIO_SPEED_FREQ_HIGH"),
            ("VERY_HIGH", "VERY_HIGH  GPIO_SPEED_FREQ_VERY_HIGH"),
        ]),
        "default": "MEDIUM",
    }),
    ("board_type", {
        "label":   "Board Type",
        "choices": OrderedDict([
            ("TEST",       "Test board        (ADDR_SETUP=2 / DATA_SETUP=10)"),
            ("PRODUCTION", "Production board  (ADDR_SETUP=1 / DATA_SETUP=8)"),
            ("LONG_FPC",   "Long FPC debug    (ADDR_SETUP=4 / DATA_SETUP=14)"),
        ]),
        "default": "TEST",
    }),
    ("dma_chunk_rows", {
        "label":   "DMA Chunk Rows",
        "choices": OrderedDict([
            ("4",  "4 rows   (lighter per-strip load, more transactions)"),
            ("8",  "8 rows   (default, good balance)"),
            ("16", "16 rows  (fewer transactions, higher latency tolerance)"),
        ]),
        "default": "8",
    }),
    ("hse_source", {
        "label":   "HSE Source",
        "choices": OrderedDict([
            ("CRYSTAL", "Crystal       (RCC_HSE_ON,     passive oscillator)"),
            ("TCXO",    "TCXO          (RCC_HSE_BYPASS,  active clock input)"),
            ("BYPASS",  "Clock bypass  (RCC_HSE_BYPASS,  external signal)"),
        ]),
        "default": "CRYSTAL",
    }),
    ("hse_freq_hz", {
        "label":      "HSE Frequency",
        "choices": OrderedDict([
            ("24000000", "24.000 MHz  (common TCXO)"),
            ("25000000", "25.000 MHz  (default crystal)"),
            ("26000000", "26.000 MHz  (common TCXO)"),
            ("CUSTOM",   "Custom value  (enter exact Hz)"),
        ]),
        "default":     "25000000",
        "custom_input": True,
        "ask_custom":   "hz",
    }),
    ("storage_type", {
        "label":   "Ext NVM Storage",
        "choices": OrderedDict([
            ("NONE",     "None           (no external NVM fitted)"),
            ("I2C_EE",   "I2C EEPROM     (AT24Cxx, M24xxx, etc.)"),
            ("SPI_EE",   "SPI EEPROM     (M95xxx, CAT25xxx, etc.)"),
            ("W25Q_NOR", "W25Q NOR Flash  (W25Q16 .. W25Q128+)"),
        ]),
        "default": "W25Q_NOR",
    }),
    ("w25q_model", {
        "label":      "  W25Q Model",
        "choices": OrderedDict([
            ("W25Q16",  "W25Q16   16 Mbit   (2 MB)"),
            ("W25Q32",  "W25Q32   32 Mbit   (4 MB)"),
            ("W25Q64",  "W25Q64   64 Mbit   (8 MB)"),
            ("W25Q128", "W25Q128  128 Mbit  (16 MB)  [CSDR default]"),
            ("CUSTOM",  "Custom   (enter capacity in Mbit)"),
        ]),
        "default":      "W25Q128",
        "custom_input": True,
        "ask_custom":   "mbit",
        "parent":       "storage_type",   # only shown when parent matches
        "parent_val":   "W25Q_NOR",
    }),
])


# Built-in presets
PRESETS = OrderedDict([
    ("hw_test_fmc", {
        "label": "Test board -- ST7796 landscape BGR, MEDIUM GPIO, 25 MHz crystal",
        "config": {
            "lcd_controller":  "ST7796", "lcd_orientation": "LANDSCAPE_BGR",
            "fmc_bus_width":   "8BIT",   "gpio_speed":      "MEDIUM",
            "board_type":      "TEST",   "dma_chunk_rows":  "8",
            "hse_source":      "CRYSTAL","hse_freq_hz":     "25000000",
            "storage_type":    "W25Q_NOR", "w25q_model":    "W25Q128",
        },
    }),
    ("hw_prod_v1", {
        "label": "Production board -- ST7796 landscape BGR, VERY_HIGH GPIO",
        "config": {
            "lcd_controller":  "ST7796", "lcd_orientation": "LANDSCAPE_BGR",
            "fmc_bus_width":   "8BIT",   "gpio_speed":      "VERY_HIGH",
            "board_type":      "PRODUCTION", "dma_chunk_rows": "8",
            "hse_source":      "CRYSTAL","hse_freq_hz":     "25000000",
            "storage_type":    "W25Q_NOR", "w25q_model":    "W25Q128",
        },
    }),
    ("hw_long_fpc_debug", {
        "label": "Long FPC debug cable -- relaxed FMC timing",
        "config": {
            "lcd_controller":  "ST7796", "lcd_orientation": "LANDSCAPE_BGR",
            "fmc_bus_width":   "8BIT",   "gpio_speed":      "MEDIUM",
            "board_type":      "LONG_FPC", "dma_chunk_rows": "8",
            "hse_source":      "CRYSTAL","hse_freq_hz":     "25000000",
            "storage_type":    "W25Q_NOR", "w25q_model":    "W25Q128",
        },
    }),
    ("hw_compact_st7789", {
        "label": "Compact panel ST7789 -- portrait BGR, MEDIUM GPIO",
        "config": {
            "lcd_controller":  "ST7789", "lcd_orientation": "PORTRAIT",
            "fmc_bus_width":   "8BIT",   "gpio_speed":      "MEDIUM",
            "board_type":      "TEST",   "dma_chunk_rows":  "8",
            "hse_source":      "CRYSTAL","hse_freq_hz":     "25000000",
            "storage_type":    "W25Q_NOR", "w25q_model":    "W25Q128",
        },
    }),
])


# ── Resolution tables ─────────────────────────────────────────────────────────

MADCTL_TABLE = {
    ("ST7796", "LANDSCAPE_BGR"): (0xE8, "MY|MX|MV|BGR  landscape, BGR filter"),
    ("ST7796", "LANDSCAPE_RGB"): (0xE0, "MY|MX|MV       landscape, RGB filter"),
    ("ST7796", "PORTRAIT"):      (0x08, "BGR            portrait,  BGR filter"),
    ("ST7789", "LANDSCAPE_BGR"): (0x68, "MX|MV|BGR      landscape, BGR filter"),
    ("ST7789", "LANDSCAPE_RGB"): (0x60, "MX|MV          landscape, RGB filter"),
    ("ST7789", "PORTRAIT"):      (0x08, "BGR            portrait,  BGR filter"),
}

LCD_DIMENSIONS = {
    ("ST7796", "LANDSCAPE_BGR"): (480, 320),
    ("ST7796", "LANDSCAPE_RGB"): (480, 320),
    ("ST7796", "PORTRAIT"):      (320, 480),
    ("ST7789", "LANDSCAPE_BGR"): (320, 240),
    ("ST7789", "LANDSCAPE_RGB"): (320, 240),
    ("ST7789", "PORTRAIT"):      (240, 320),
}

FMC_TIMING = {
    "TEST": {
        "addr_setup": 2,  "addr_hold": 15, "data_setup": 10,
        "bus_turn":   15, "clk_div":   16, "data_latency": 17,
    },
    "PRODUCTION": {
        "addr_setup": 1,  "addr_hold": 15, "data_setup":  8,
        "bus_turn":   15, "clk_div":   16, "data_latency": 17,
    },
    "LONG_FPC": {
        "addr_setup": 4,  "addr_hold": 15, "data_setup": 14,
        "bus_turn":   15, "clk_div":   16, "data_latency": 17,
    },
}

GPIO_SPEED_CONST = {
    "LOW":       "GPIO_SPEED_FREQ_LOW",
    "MEDIUM":    "GPIO_SPEED_FREQ_MEDIUM",
    "HIGH":      "GPIO_SPEED_FREQ_HIGH",
    "VERY_HIGH": "GPIO_SPEED_FREQ_VERY_HIGH",
}

HSE_RCC_MODE = {
    "CRYSTAL": "RCC_HSE_ON",
    "TCXO":    "RCC_HSE_BYPASS",
    "BYPASS":  "RCC_HSE_BYPASS",
}

PANEL_ID = {"ST7796": 1, "ST7789": 2}

ORIENT_LABEL = {
    "LANDSCAPE_BGR": "Landscape BGR",
    "LANDSCAPE_RGB": "Landscape RGB",
    "PORTRAIT":      "Portrait",
}

BOARD_LABEL = {
    "TEST":       "Test board",
    "PRODUCTION": "Production board",
    "LONG_FPC":   "Long FPC debug",
}

# Known W25Q model -> capacity in Mbit
W25Q_MODELS = {
    "W25Q16":  16,
    "W25Q32":  32,
    "W25Q64":  64,
    "W25Q128": 128,
}

# Storage type numeric IDs emitted as HW_STORAGE_TYPE
# 0=none 1=i2c_ee 2=spi_ee 3=w25q 4=fram 5=qspi_nor 6=nand 7=sd
STORAGE_TYPE_ID = {
    "NONE":     0,
    "I2C_EE":   1,
    "SPI_EE":   2,
    "W25Q_NOR": 3,
}

STORAGE_LABEL = {
    "NONE":     "None",
    "I2C_EE":   "I2C EEPROM",
    "SPI_EE":   "SPI EEPROM",
    "W25Q_NOR": "W25Q NOR Flash",
}

# Minimum W25Q capacity for HW_HAS_LARGE_NVM and HW_SUPPORTS_WATERFALL_CACHE
_LARGE_NVM_MBIT = 8    # >= 1 MB usable


# ── PLL computation ────────────────────────────────────────────────────────────

def _vcirange(vco_in_hz, prefix="PLL1"):
    """Return RCC VCI-range constant for the given VCO input frequency."""
    if vco_in_hz < 2_000_000:
        idx = 0
    elif vco_in_hz < 4_000_000:
        idx = 1
    elif vco_in_hz < 8_000_000:
        idx = 2
    else:
        idx = 3
    return f"RCC_{prefix}VCIRANGE_{idx}"


def compute_pll1(hse_hz):
    """Find integer PLLM/N/P giving SYSCLK = 480 MHz exactly.

    Returns dict(m, n, p, vco_in, vco_out, rge) or None.
    Prefers smallest p (=2) to keep VCO_out = 960 MHz so PLL1Q/R outputs
    remain unchanged from the original CubeMX values.
    """
    TARGET = 480_000_000
    hse_hz = int(hse_hz)
    for m in range(1, 64):
        vco_in = hse_hz / m
        if not (1_000_000 <= vco_in <= 16_000_000):
            continue
        for p in range(2, 129):
            vco_out = TARGET * p          # exact: VCO_out = SYSCLK * p
            if vco_out > 960_000_000:
                break
            if vco_out < 192_000_000:
                continue
            # n = vco_out * m / hse_hz — must be exact integer in [4, 512]
            numer = vco_out * m
            if numer % hse_hz != 0:
                continue
            n = numer // hse_hz
            if n < 4 or n > 512:
                continue
            return {
                "m": m, "n": n, "p": p,
                "vco_in":  vco_in,
                "vco_out": float(vco_out),
                "rge":     _vcirange(vco_in, "PLL1"),
            }
    return None


def compute_pll2(hse_hz):
    """Find PLL2M/N/P minimising |SAI1 clock - 12.288 MHz|.

    Returns dict(m, n, p, vco_in, vco_out, sai_hz, err_hz, rge) or None.
    """
    TARGET = 12_288_000          # 48 kHz * 256
    hse_hz = int(hse_hz)
    best = None
    best_err = float("inf")
    for m in range(1, 64):
        vco_in = hse_hz / m
        if not (1_000_000 <= vco_in <= 16_000_000):
            continue
        for p in range(2, 129):
            n_exact = TARGET * p / vco_in
            n = int(round(n_exact))
            if n < 4 or n > 512:
                continue
            vco_out = vco_in * n
            if not (192_000_000 <= vco_out <= 960_000_000):
                continue
            sai_hz = vco_out / p
            err = abs(sai_hz - TARGET)
            if err < best_err:
                best_err = err
                best = {
                    "m": m, "n": n, "p": p,
                    "vco_in":  vco_in,
                    "vco_out": vco_out,
                    "sai_hz":  sai_hz,
                    "err_hz":  err,
                    "rge":     _vcirange(vco_in, "PLL2"),
                }
    return best


def _pll_fallback():
    return {"m": 0, "n": 0, "p": 0, "vco_in": 0, "vco_out": 0,
            "sai_hz": 0, "err_hz": 0, "rge": "RCC_PLL1VCIRANGE_0"}


# ── Storage resolution ────────────────────────────────────────────────────────

def resolve_w25q_mbit(model_val):
    """Return W25Q capacity in Mbit from model key or custom numeric string."""
    if model_val in W25Q_MODELS:
        return W25Q_MODELS[model_val]
    try:
        return int(model_val)
    except (ValueError, TypeError):
        return 128  # safe fallback


def resolve_storage(cfg):
    """Return dict of storage-derived values for the current config."""
    stype   = cfg.get("storage_type", "NONE")
    type_id = STORAGE_TYPE_ID.get(stype, 0)
    is_w25q = 1 if stype == "W25Q_NOR" else 0

    if is_w25q:
        mbit         = resolve_w25q_mbit(cfg.get("w25q_model", "W25Q128"))
        has_large    = 1 if mbit >= _LARGE_NVM_MBIT else 0
        supports_wf  = 1 if mbit >= _LARGE_NVM_MBIT else 0
        page_size    = 256
        sector_size  = 4096
    else:
        mbit        = 0
        has_large   = 0
        supports_wf = 0
        page_size   = 0
        sector_size = 0

    has_storage = 1 if stype != "NONE" else 0

    return {
        "stype":       stype,
        "type_id":     type_id,
        "is_w25q":     is_w25q,
        "has_storage": has_storage,
        "has_large":   has_large,
        "supports_wf": supports_wf,
        "mbit":        mbit,
        "page_size":   page_size,
        "sector_size": sector_size,
        "label":       STORAGE_LABEL.get(stype, stype),
    }


# ── Validation ────────────────────────────────────────────────────────────────

def validate(cfg):
    """Return (errors, warnings) for the current configuration."""
    errors, warnings = [], []
    ctrl   = cfg["lcd_controller"]
    orient = cfg["lcd_orientation"]
    board  = cfg["board_type"]
    speed  = cfg["gpio_speed"]
    width  = cfg["fmc_bus_width"]

    # ── FMC/LCD ──
    if width == "16BIT":
        errors.append(
            "16-bit FMC bus is not implemented in the current driver. "
            "Select 8-bit."
        )
    if ctrl == "ST7789" and "LANDSCAPE" in orient:
        warnings.append(
            "ST7789 landscape is non-native (MV axis swap required). "
            "Verify MADCTL on oscilloscope before production use."
        )
    if ctrl == "ST7796" and orient == "PORTRAIT":
        warnings.append(
            "ST7796 portrait (320x480): SDR UI zones were designed for 480x320 "
            "landscape. Expect layout issues until sdr_ui.h is updated."
        )
    if board == "PRODUCTION" and speed in ("LOW", "MEDIUM"):
        warnings.append(
            "Production board has controlled-impedance FMC traces. "
            "VERY_HIGH GPIO speed is recommended for signal integrity."
        )
    if board == "TEST" and speed == "VERY_HIGH":
        warnings.append(
            "VERY_HIGH GPIO speed on test/breadboard wiring may cause ringing. "
            "MEDIUM is safer for non-production boards."
        )
    if ctrl == "ST7789" and board == "PRODUCTION":
        warnings.append(
            "ST7789 on production board timing is untested. "
            "Validate with oscilloscope."
        )

    # ── HSE / PLL ──
    try:
        hse_hz = int(cfg["hse_freq_hz"])
    except (ValueError, TypeError):
        errors.append("HSE frequency is not a valid integer.")
        return errors, warnings

    if not (1_000_000 <= hse_hz <= 50_000_000):
        errors.append(
            f"HSE {hse_hz/1e6:.3f} MHz is outside the valid range (1 – 50 MHz)."
        )
        return errors, warnings

    pll1 = compute_pll1(hse_hz)
    if pll1 is None:
        errors.append(
            f"No valid PLL1 solution for HSE = {hse_hz/1e6:.3f} MHz targeting "
            f"SYSCLK = 480 MHz. Try 24, 25, or 26 MHz."
        )

    pll2 = compute_pll2(hse_hz)
    if pll2 is not None:
        err_ppm = pll2["err_hz"] / 12_288_000 * 1e6
        if err_ppm > 10_000:   # > 1 %
            warnings.append(
                f"HSE {hse_hz/1e6:.3f} MHz: SAI1 = {pll2['sai_hz']/1e6:.4f} MHz "
                f"(target 12.2880, error {err_ppm:.0f} ppm). "
                f"USB/audio drift will be significant."
            )

    return errors, warnings


# ── Resolution ────────────────────────────────────────────────────────────────

def resolve(cfg):
    """Return dict of all derived hardware values for the current config."""
    ctrl   = cfg["lcd_controller"]
    orient = cfg["lcd_orientation"]
    board  = cfg["board_type"]
    speed  = cfg["gpio_speed"]
    rows   = int(cfg["dma_chunk_rows"])

    madctl_val, madctl_desc = MADCTL_TABLE[(ctrl, orient)]
    w, h = LCD_DIMENSIONS[(ctrl, orient)]
    timing = FMC_TIMING[board]

    try:
        hse_hz = int(cfg["hse_freq_hz"])
    except (ValueError, TypeError):
        hse_hz = 25_000_000

    pll1    = compute_pll1(hse_hz) or _pll_fallback()
    pll2    = compute_pll2(hse_hz) or _pll_fallback()
    storage = resolve_storage(cfg)

    return {
        "panel":        PANEL_ID[ctrl],
        "madctl":       madctl_val,
        "madctl_desc":  madctl_desc,
        "lcd_w":        w,
        "lcd_h":        h,
        "timing":       timing,
        "gpio_speed":   GPIO_SPEED_CONST[speed],
        "dma_rows":     rows,
        "hse_hz":       hse_hz,
        "hse_source":   cfg["hse_source"],
        "hse_rcc_mode": HSE_RCC_MODE[cfg["hse_source"]],
        "pll1":         pll1,
        "pll2":         pll2,
        "storage":      storage,
    }


# ── Header generation ─────────────────────────────────────────────────────────

def _storage_header_section(cfg, st):
    """Return the storage section string for hw_config_active.h."""
    stype      = st["stype"]
    type_id    = st["type_id"]
    has_store  = st["has_storage"]
    has_large  = st["has_large"]
    sup_wf     = st["supports_wf"]
    is_w25q    = st["is_w25q"]
    mbit       = st["mbit"]
    page       = st["page_size"]
    sector     = st["sector_size"]

    # One-line summary for comment
    if is_w25q:
        model_str = cfg.get("w25q_model", "W25Q128")
        if model_str not in W25Q_MODELS:
            model_str = f"Custom {mbit} Mbit"
        summary = f"W25Q NOR Flash  {model_str}  {mbit} Mbit ({mbit // 8} MB)"
    elif stype == "NONE":
        summary = "None (no external NVM fitted)"
    else:
        summary = st["label"]

    # Build flag lines — one flag = 1 per type, rest = 0
    flags = {
        "HW_STORAGE_NONE":     1 if stype == "NONE"     else 0,
        "HW_STORAGE_I2C_EE":   1 if stype == "I2C_EE"   else 0,
        "HW_STORAGE_SPI_EE":   1 if stype == "SPI_EE"   else 0,
        "HW_STORAGE_W25Q":     1 if stype == "W25Q_NOR"  else 0,
        "HW_STORAGE_FRAM":     0,   # future: SPI FRAM (e.g. MB85RSxxx)
        "HW_STORAGE_QSPI_NOR": 0,   # future: QSPI NOR (OSPI / OCTOSPI)
        "HW_STORAGE_NAND":     0,   # future: SPI/parallel NAND
        "HW_STORAGE_SD":       0,   # future: SD / MMC card
    }

    lines = [
        "/* -- External NVM storage -----------------------------------------------\n",
        f" * Selected : {summary}\n",
        " *\n",
        " * Use #if HW_STORAGE_W25Q / HW_STORAGE_NONE etc. for conditional\n",
        " * compilation.  Future variants (FRAM, QSPI_NOR, NAND, SD) will use\n",
        " * the same flag pattern with type IDs 4-7.\n",
        " *\n",
        " * HW_HAS_PERSISTENT_STORAGE : any writable NVM is fitted\n",
        " * HW_HAS_LARGE_NVM          : >= 8 Mbit fitted (suitable for IQ/WF buffering)\n",
        " * HW_SUPPORTS_WATERFALL_CACHE: large NVM available for waterfall snapshots  */\n",
        "\n",
        "/* Storage type flags (exactly one equals 1) */\n",
    ]
    for name, val in flags.items():
        comment = ""
        if name == "HW_STORAGE_FRAM":
            comment = "   /* future */"
        elif name == "HW_STORAGE_QSPI_NOR":
            comment = "  /* future */"
        elif name == "HW_STORAGE_NAND":
            comment = "      /* future */"
        elif name == "HW_STORAGE_SD":
            comment = "        /* future */"
        lines.append(f"#define {name:<28s} {val}{comment}\n")

    lines += [
        "\n",
        f"/* Storage type ID  (0=none 1=i2c_ee 2=spi_ee 3=w25q 4-7=future) */\n",
        f"#define HW_STORAGE_TYPE              {type_id}\n",
        "\n",
        "/* Capability flags */\n",
        f"#define HW_HAS_PERSISTENT_STORAGE    {has_store}\n",
        f"#define HW_HAS_LARGE_NVM             {has_large}\n",
        f"#define HW_SUPPORTS_WATERFALL_CACHE  {sup_wf}\n",
    ]

    if is_w25q:
        cap_bytes_expr = f"({mbit}UL * 131072UL)"  # Mbit * 128KB
        lines += [
            "\n",
            "/* W25Q geometry  (valid only when HW_STORAGE_W25Q == 1) */\n",
            f"#define HW_W25Q_CAPACITY_MBIT        {mbit}U\n",
            f"#define HW_W25Q_CAPACITY_BYTES       {cap_bytes_expr}\n",
            f"#define HW_W25Q_PAGE_SIZE            {page}U\n",
            f"#define HW_W25Q_SECTOR_SIZE          {sector}U\n",
            f"#define HW_W25Q_BLOCK32_SIZE         32768U\n",
            f"#define HW_W25Q_BLOCK64_SIZE         65536U\n",
        ]

    return "".join(lines)


def generate_header(cfg, r):
    """Return the complete content of hw_config_active.h as a string."""
    t       = r["timing"]
    ts      = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    mdesc   = r["madctl_desc"]
    p1      = r["pll1"]
    p2      = r["pll2"]
    st      = r["storage"]
    sai_mhz = p2["sai_hz"] / 1e6 if p2["sai_hz"] else 0.0

    # Storage summary for file-header comment
    if st["is_w25q"]:
        model_str = cfg.get("w25q_model", "W25Q128")
        if model_str not in W25Q_MODELS:
            model_str = f"Custom {st['mbit']} Mbit"
        storage_comment = f"W25Q NOR  {model_str}  {st['mbit']} Mbit"
    elif st["stype"] == "NONE":
        storage_comment = "None"
    else:
        storage_comment = st["label"]

    return (
        "/* hw_config_active.h -- CSDR Hardware Configuration (auto-generated)\n"
        " * DO NOT EDIT -- regenerate with:  python tools/hw_config.py\n"
        " *\n"
        f" * Generated  : {ts}\n"
        f" * Controller : {cfg['lcd_controller']}\n"
        f" * Orientation: {ORIENT_LABEL[cfg['lcd_orientation']]}\n"
        f" * FMC width  : {'8-bit' if cfg['fmc_bus_width'] == '8BIT' else '16-bit'}\n"
        f" * GPIO speed : {cfg['gpio_speed']}\n"
        f" * Board      : {BOARD_LABEL[cfg['board_type']]}\n"
        f" * HSE        : {r['hse_hz']/1e6:.3f} MHz  {cfg['hse_source']}\n"
        f" * SYSCLK     : 480 MHz  (PLL1 M={p1['m']} N={p1['n']} P={p1['p']})\n"
        f" * SAI1       : {sai_mhz:.4f} MHz  (PLL2 M={p2['m']} N={p2['n']} P={p2['p']})\n"
        f" * Storage    : {storage_comment}\n"
        " */\n"
        "\n"
        "#ifndef HW_CONFIG_ACTIVE_H\n"
        "#define HW_CONFIG_ACTIVE_H\n"
        "\n"
        "/* -- Panel ---------------------------------------------------------------\n"
        " * HW_LCD_PANEL is read by lcd_panel_config.h to select the driver path.\n"
        " * LCD_W / LCD_H are provided here so portrait and landscape both resolve\n"
        " * correctly without editing lcd_panel_config.h.                          */\n"
        f"#define HW_LCD_PANEL        {r['panel']}   /* {cfg['lcd_controller']} */\n"
        f"#define LCD_W               {r['lcd_w']}U\n"
        f"#define LCD_H               {r['lcd_h']}U\n"
        "\n"
        "/* -- MADCTL (register 0x36) ----------------------------------------------\n"
        f" * {mdesc:<68s}*/\n"
        f"#define HW_LCD_MADCTL       0x{r['madctl']:02X}U\n"
        "\n"
        "/* -- FMC SRAM timing (AHB cycles, asynchronous mode A) ------------------\n"
        " * Limits per STM32H7 RM0433: ADDR_SETUP 0-15, ADDR_HOLD 1-15,\n"
        " * DATA_SETUP 1-255, BUS_TURN 0-15, CLK_DIV 2-16, DATA_LATENCY 2-17.   */\n"
        f"#define HW_FMC_ADDR_SETUP   {t['addr_setup']}U\n"
        f"#define HW_FMC_ADDR_HOLD    {t['addr_hold']}U\n"
        f"#define HW_FMC_DATA_SETUP   {t['data_setup']}U\n"
        f"#define HW_FMC_BUS_TURN     {t['bus_turn']}U\n"
        f"#define HW_FMC_CLK_DIV      {t['clk_div']}U\n"
        f"#define HW_FMC_DATA_LATENCY {t['data_latency']}U\n"
        "\n"
        "/* -- FMC GPIO drive strength ---------------------------------------------\n"
        " * One of GPIO_SPEED_FREQ_LOW / MEDIUM / HIGH / VERY_HIGH               */\n"
        f"#define HW_FMC_GPIO_SPEED   {r['gpio_speed']}\n"
        "\n"
        "/* -- LCD DMA push chunk size (spectrum strip height) --------------------\n"
        " * Must be <= SPEC_H (72 for ST7796, 76 for ST7789). Valid range 1-64.  */\n"
        f"#define HW_DMA_CHUNK_ROWS   {r['dma_rows']}U\n"
        "\n"
        "/* -- HSE clock source and PLL configuration -----------------------------\n"
        " * hw_config.py also patches Core/Src/main.c (PLL1/PLL2 dividers,\n"
        " * HSEState) and HSE_VALUE in stm32h7xx_hal_conf.h / system_stm32h7xx.c.\n"
        " * These macros mirror those patched values for reference / static assert.*/\n"
        f"#define HW_HSE_FREQ_HZ      {r['hse_hz']}UL\n"
        f"#define HW_HSE_RCC_MODE     {r['hse_rcc_mode']}\n"
        "\n"
        f"#define HW_PLL1_M           {p1['m']}U\n"
        f"#define HW_PLL1_N           {p1['n']}U\n"
        f"#define HW_PLL1_P           {p1['p']}U\n"
        f"#define HW_PLL1_VCIRANGE    {p1['rge']}\n"
        "\n"
        f"#define HW_PLL2_M           {p2['m']}U\n"
        f"#define HW_PLL2_N           {p2['n']}U\n"
        f"#define HW_PLL2_P           {p2['p']}U\n"
        f"#define HW_PLL2_VCIRANGE    {p2['rge']}\n"
        "\n"
        + _storage_header_section(cfg, st) +
        "\n"
        "#endif /* HW_CONFIG_ACTIVE_H */\n"
    )


# ── Source-file patching ──────────────────────────────────────────────────────

def _sub1(pattern, repl, text, label):
    """re.sub wrapper that prints a warning if the pattern matched 0 times."""
    result, count = re.subn(pattern, repl, text)
    if count == 0:
        print(f"  [!] patch: pattern not found in {label}: {pattern!r}",
              file=sys.stderr)
    return result


def patch_main_clock(r):
    """Patch PLL1/PLL2 dividers and HSEState in Core/Src/main.c."""
    if not os.path.isfile(MAIN_C):
        print(f"  [!] {MAIN_C} not found — skipping clock patch.", file=sys.stderr)
        return

    with open(MAIN_C, encoding="utf-8") as fh:
        text = fh.read()

    p1 = r["pll1"]
    p2 = r["pll2"]
    m  = r["hse_rcc_mode"]

    text = _sub1(
        r"(RCC_OscInitStruct\.HSEState\s*=\s*)RCC_HSE_\w+;",
        rf"\g<1>{m};", text, "main.c HSEState")
    text = _sub1(r"(RCC_OscInitStruct\.PLL\.PLLM\s*=\s*)\d+;",
                 rf"\g<1>{p1['m']};", text, "main.c PLLM")
    text = _sub1(r"(RCC_OscInitStruct\.PLL\.PLLN\s*=\s*)\d+;",
                 rf"\g<1>{p1['n']};", text, "main.c PLLN")
    text = _sub1(r"(RCC_OscInitStruct\.PLL\.PLLP\s*=\s*)\d+;",
                 rf"\g<1>{p1['p']};", text, "main.c PLLP")
    text = _sub1(r"(RCC_OscInitStruct\.PLL\.PLLRGE\s*=\s*)RCC_PLL1VCIRANGE_\d;",
                 rf"\g<1>{p1['rge']};", text, "main.c PLLRGE")
    text = _sub1(r"(PeriphClkInitStruct\.PLL2\.PLL2M\s*=\s*)\d+;",
                 rf"\g<1>{p2['m']};", text, "main.c PLL2M")
    text = _sub1(r"(PeriphClkInitStruct\.PLL2\.PLL2N\s*=\s*)\d+;",
                 rf"\g<1>{p2['n']};", text, "main.c PLL2N")
    text = _sub1(r"(PeriphClkInitStruct\.PLL2\.PLL2P\s*=\s*)\d+;",
                 rf"\g<1>{p2['p']};", text, "main.c PLL2P")
    text = _sub1(r"(PeriphClkInitStruct\.PLL2\.PLL2RGE\s*=\s*)RCC_PLL2VCIRANGE_\d;",
                 rf"\g<1>{p2['rge']};", text, "main.c PLL2RGE")

    with open(MAIN_C, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)


def patch_hse_value(hse_hz):
    """Patch HSE_VALUE literals in stm32h7xx_hal_conf.h and system_stm32h7xx.c."""
    hz = int(hse_hz)

    if os.path.isfile(CONF_H):
        with open(CONF_H, encoding="utf-8") as fh:
            text = fh.read()
        text = _sub1(
            r"(#define\s+HSE_VALUE\s+)\(\d+UL\)",
            rf"\g<1>({hz}UL)", text, "stm32h7xx_hal_conf.h")
        with open(CONF_H, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
    else:
        print(f"  [!] {CONF_H} not found.", file=sys.stderr)

    if os.path.isfile(SYSTEM_C):
        with open(SYSTEM_C, encoding="utf-8") as fh:
            text = fh.read()
        text = _sub1(
            r"(#define\s+HSE_VALUE\s+)\(\(uint32_t\)\d+\)",
            rf"\g<1>((uint32_t){hz})", text, "system_stm32h7xx.c")
        with open(SYSTEM_C, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
    else:
        print(f"  [!] {SYSTEM_C} not found.", file=sys.stderr)


# ── State persistence ─────────────────────────────────────────────────────────

def default_config():
    return {k: v["default"] for k, v in SETTINGS.items()}


def load_state():
    """Load saved config from JSON, filling missing keys with defaults."""
    if not os.path.isfile(STATE_FILE):
        return default_config()
    try:
        with open(STATE_FILE, encoding="utf-8") as fh:
            data = json.load(fh)
        cfg = default_config()
        for k in SETTINGS:
            if k not in data:
                continue
            sdef = SETTINGS[k]
            val  = data[k]
            if val in sdef["choices"]:
                cfg[k] = val
            elif sdef.get("custom_input"):
                ask_fn = sdef.get("ask_custom", "hz")
                if ask_fn == "mbit":
                    try:
                        v = int(val)
                        if 1 <= v <= 16384:
                            cfg[k] = str(v)
                    except (ValueError, TypeError):
                        pass
                else:  # hz
                    try:
                        hz = int(val)
                        if 1_000_000 <= hz <= 100_000_000:
                            cfg[k] = str(hz)
                    except (ValueError, TypeError):
                        pass
        return cfg
    except (json.JSONDecodeError, KeyError, TypeError):
        return default_config()


def save_state(cfg):
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    with open(STATE_FILE, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(cfg, fh, indent=2)
        fh.write("\n")


# ── Config apply ──────────────────────────────────────────────────────────────

def apply_config(cfg):
    """Validate, generate header, patch source files, persist state.
    Returns True on success."""
    errors, warnings = validate(cfg)

    if errors:
        print("\n  [ERROR] Cannot generate config:")
        for e in errors:
            print(f"    - {e}")
        return False

    if warnings:
        print("\n  [!] Warnings:")
        for w in warnings:
            print(f"    - {w}")
        print()

    r = resolve(cfg)
    content = generate_header(cfg, r)

    os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
    with open(OUTPUT_FILE, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(content)

    patch_main_clock(r)
    patch_hse_value(r["hse_hz"])
    save_state(cfg)

    t  = r["timing"]
    p1 = r["pll1"]
    p2 = r["pll2"]
    st = r["storage"]
    print("  Config written and source files patched:")
    print(f"    Header    : BSP/Inc/hw_config_active.h")
    print(f"    Patched   : Core/Src/main.c")
    print(f"    Patched   : Core/Inc/stm32h7xx_hal_conf.h")
    print(f"    Patched   : Core/Src/system_stm32h7xx.c")
    print(f"    State     : config/hw_config_state.json")
    print()
    print(f"  LCD     : {cfg['lcd_controller']}  {r['lcd_w']} x {r['lcd_h']}"
          f"  (HW_LCD_PANEL = {r['panel']})")
    print(f"  MADCTL  : 0x{r['madctl']:02X}  [{r['madctl_desc'].strip()}]")
    print(f"  FMC     : ADDR_SETUP={t['addr_setup']}  DATA_SETUP={t['data_setup']}"
          f"  BUS_TURN={t['bus_turn']}")
    print(f"  GPIO    : {r['gpio_speed']}")
    print(f"  DMA     : {r['dma_rows']} rows/strip")
    print(f"  Board   : {BOARD_LABEL[cfg['board_type']]}")
    print(f"  HSE     : {r['hse_hz']/1e6:.3f} MHz  {cfg['hse_source']}"
          f"  ({r['hse_rcc_mode']})")
    print(f"  PLL1    : M={p1['m']} N={p1['n']} P={p1['p']}"
          f"  -> SYSCLK 480 MHz  [{p1['rge']}]")
    sai_err = p2["err_hz"]
    print(f"  PLL2    : M={p2['m']} N={p2['n']} P={p2['p']}"
          f"  -> SAI1 {p2['sai_hz']/1e6:.4f} MHz"
          f"  (err {sai_err/1e3:.1f} kHz)  [{p2['rge']}]")
    _print_storage_summary(st, cfg)
    print()
    print("  Rebuild firmware to apply this configuration.")
    return True


def _print_storage_summary(st, cfg):
    """Print the storage section of the apply/show output."""
    if st["is_w25q"]:
        model_str = cfg.get("w25q_model", "W25Q128")
        if model_str not in W25Q_MODELS:
            model_str = f"Custom {st['mbit']} Mbit"
        cap = f"{st['mbit']} Mbit ({st['mbit'] // 8} MB)"
        print(f"  NVM     : W25Q NOR Flash  {model_str}  {cap}")
        flags = []
        if st["has_storage"]:  flags.append("HAS_PERSISTENT_STORAGE")
        if st["has_large"]:    flags.append("HAS_LARGE_NVM")
        if st["supports_wf"]:  flags.append("SUPPORTS_WATERFALL_CACHE")
        if flags:
            print(f"            flags: {' '.join(flags)}")
    elif st["stype"] == "NONE":
        print("  NVM     : None")
    else:
        print(f"  NVM     : {st['label']}  -> HAS_PERSISTENT_STORAGE")


# ── Terminal helpers ──────────────────────────────────────────────────────────

_W    = 56
_SEP  = "=" * _W
_THIN = "-" * _W


def clr():
    os.system("cls" if os.name == "nt" else "clear")


def active_keys(cfg):
    """Return the ordered list of setting keys to show/edit in the current cfg.

    Settings with a 'parent' key are only included when
    cfg[parent] == parent_val.
    """
    result = []
    for k, sdef in SETTINGS.items():
        parent = sdef.get("parent")
        if parent is None or cfg.get(parent) == sdef.get("parent_val"):
            result.append(k)
    return result


def short_label(key, val):
    """Short display name for a setting value."""
    sdef = SETTINGS[key]
    full = sdef["choices"].get(val)
    if full:
        return full.split("  ")[0].strip()
    if sdef.get("custom_input"):
        ask_fn = sdef.get("ask_custom", "hz")
        if ask_fn == "mbit":
            try:
                return f"Custom {int(val)} Mbit"
            except (ValueError, TypeError):
                pass
        else:
            try:
                return f"{int(val)/1e6:.3f} MHz"
            except (ValueError, TypeError):
                pass
    return str(val)


def ask_int(prompt, lo, hi, allow_enter=False):
    """Read an integer in [lo..hi]; returns -1 on bare Enter if allow_enter."""
    while True:
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if allow_enter and raw == "":
            return -1
        if not raw:
            continue
        try:
            v = int(raw)
            if lo <= v <= hi:
                return v
        except ValueError:
            pass
        print(f"    Please enter a number between {lo} and {hi}.")


def ask_hz(label):
    """Prompt for a frequency in Hz; returns as string."""
    while True:
        try:
            raw = input(f"\n  Enter {label} in Hz (e.g. 25000000): ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if not raw:
            continue
        try:
            v = int(raw)
            if 1_000_000 <= v <= 100_000_000:
                return str(v)
            print("    Value out of range (1 MHz - 100 MHz).")
        except ValueError:
            print("    Enter a plain integer in Hz, e.g. 25000000")


def ask_mbit(label):
    """Prompt for flash capacity in Mbit; returns as string."""
    while True:
        try:
            raw = input(f"\n  Enter {label} capacity in Mbit (e.g. 256): ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if not raw:
            continue
        try:
            v = int(raw)
            if 1 <= v <= 16384:
                return str(v)
            print("    Value out of range (1 - 16384 Mbit).")
        except ValueError:
            print("    Enter an integer in Mbit, e.g. 256")


# ── Sub-menus ─────────────────────────────────────────────────────────────────

def run_setting_menu(key, current):
    """Display choices for one setting; return the chosen value."""
    sdef    = SETTINGS[key]
    choices = list(sdef["choices"].items())
    is_custom_mode = current not in sdef["choices"] and sdef.get("custom_input")
    ask_fn  = sdef.get("ask_custom", "hz")

    clr()
    print(_SEP)
    print(f"  {sdef['label'].strip()}")
    print(_THIN)
    for i, (val, desc) in enumerate(choices, 1):
        if val == current:
            marker = "  <--"
        elif val == "CUSTOM" and is_custom_mode:
            if ask_fn == "mbit":
                try:
                    marker = f"  <-- ({int(current)} Mbit)"
                except (ValueError, TypeError):
                    marker = "  <--"
            else:
                try:
                    marker = f"  <-- ({int(current)/1e6:.3f} MHz)"
                except (ValueError, TypeError):
                    marker = "  <--"
        else:
            marker = ""
        print(f"  {i}.  {desc}{marker}")
    print(_THIN)

    idx = ask_int(
        f"\n  Select [1-{len(choices)}] or 0 to keep current: ",
        lo=0, hi=len(choices),
    )
    if idx == 0:
        return current
    chosen_val = choices[idx - 1][0]
    if chosen_val == "CUSTOM" and sdef.get("custom_input"):
        return ask_mbit(sdef["label"].strip()) if ask_fn == "mbit" else ask_hz(sdef["label"].strip())
    return chosen_val


def run_preset_menu(cfg):
    """Display presets list; return updated cfg dict (or original on cancel)."""
    preset_list = list(PRESETS.items())

    clr()
    print(_SEP)
    print("  Load Preset")
    print(_THIN)
    for i, (name, pdef) in enumerate(preset_list, 1):
        print(f"  {i}.  {name:<26s}  {pdef['label']}")
    print(_THIN)

    idx = ask_int(
        f"\n  Select [1-{len(preset_list)}] or 0 to cancel: ",
        lo=0, hi=len(preset_list),
    )
    if idx == 0:
        return cfg
    new_cfg = default_config()
    new_cfg.update(preset_list[idx - 1][1]["config"])
    return new_cfg


# ── Main menu ─────────────────────────────────────────────────────────────────

def _hse_resolved_line(r):
    p1 = r["pll1"]
    p2 = r["pll2"]
    if p1["m"] == 0:
        pll1_str = "no 480 MHz solution!"
    else:
        pll1_str = f"SYSCLK 480 MHz  M={p1['m']} N={p1['n']} P={p1['p']}"
    if p2["m"] == 0:
        pll2_str = "no SAI solution"
    else:
        pll2_str = f"SAI {p2['sai_hz']/1e6:.4f} MHz (err {p2['err_hz']/1e3:.1f} kHz)"
    hz_str = f"{r['hse_hz']/1e6:.3f} MHz  {r['hse_source']}"
    return hz_str, pll1_str, pll2_str


def _storage_resolved_line(r, cfg):
    st = r["storage"]
    if st["is_w25q"]:
        model_str = cfg.get("w25q_model", "W25Q128")
        if model_str not in W25Q_MODELS:
            model_str = f"Custom {st['mbit']} Mbit"
        cap = f"{st['mbit']} Mbit  page={st['page_size']}  sector={st['sector_size']}"
        flags = []
        if st["has_storage"]:  flags.append("HAS_PERSISTENT")
        if st["has_large"]:    flags.append("HAS_LARGE_NVM")
        if st["supports_wf"]:  flags.append("SUPPORTS_WF_CACHE")
        return f"W25Q  {model_str}  {cap}", " ".join(flags)
    elif st["stype"] == "NONE":
        return "None", ""
    else:
        return st["label"], "HAS_PERSISTENT"


def print_main_menu(cfg, errors, warnings):
    clr()
    print(_SEP)
    print("  CSDR Firmware -- Hardware Configuration")
    print(_SEP)

    def _wrap(prefix, msg):
        words = msg.split()
        line, out = prefix, []
        for w in words:
            if len(line) + len(w) + 1 > _W - 2:
                out.append(line)
                line = "         "
            line += w + " "
        out.append(line)
        print("\n".join(out))

    if errors:
        for e in errors:
            _wrap("  [ERR] ", e)
        print()
    if warnings:
        for w in warnings:
            _wrap("  [!]   ", w)
        print()

    keys = active_keys(cfg)
    for i, key in enumerate(keys, 1):
        lbl = SETTINGS[key]["label"]
        val = short_label(key, cfg[key])
        print(f"  {i}.  {lbl:<22s}: {val}")

    r = resolve(cfg)
    t = r["timing"]
    hz_str, pll1_str, pll2_str = _hse_resolved_line(r)
    st_str, st_flags = _storage_resolved_line(r, cfg)
    print()
    print("  Resolved:")
    print(f"    LCD    : {r['lcd_w']} x {r['lcd_h']}  (HW_LCD_PANEL = {r['panel']})")
    print(f"    MADCTL : 0x{r['madctl']:02X}  [{r['madctl_desc'].strip()}]")
    print(f"    FMC    : ADDR_SETUP={t['addr_setup']}  DATA_SETUP={t['data_setup']}"
          f"  BUS_TURN={t['bus_turn']}")
    print(f"    GPIO   : {r['gpio_speed']}")
    print(f"    HSE    : {hz_str}")
    print(f"           -> {pll1_str}")
    print(f"           -> {pll2_str}")
    print(f"    NVM    : {st_str}")
    if st_flags:
        print(f"           -> {st_flags}")

    n = len(keys)
    print()
    print(_THIN)
    print(f"  {n+1}.  Load preset")
    print(f"  {n+2}.  Generate config and exit")
    print(f"   0.  Exit without generating")
    print(_THIN)


def run_interactive(cfg):
    """Interactive menu loop. Returns final cfg on generate, None on exit."""
    while True:
        errors, warnings = validate(cfg)
        print_main_menu(cfg, errors, warnings)
        keys = active_keys(cfg)
        n    = len(keys)
        choice = ask_int(f"\n  Select option [0-{n+2}]: ", lo=0, hi=n + 2)

        if choice == 0:
            return None

        if 1 <= choice <= n:
            cfg[keys[choice - 1]] = run_setting_menu(keys[choice - 1],
                                                      cfg[keys[choice - 1]])
        elif choice == n + 1:
            cfg = run_preset_menu(cfg)

        elif choice == n + 2:
            clr()
            print(_SEP)
            print("  CSDR Firmware -- Hardware Configuration")
            print(_SEP)
            ok = apply_config(cfg)
            if ok:
                return cfg
            input("\n  Press Enter to continue...")


# ── CLI commands ──────────────────────────────────────────────────────────────

def cmd_show():
    if not os.path.isfile(STATE_FILE):
        print("No saved configuration. Run 'python tools/hw_config.py' first.")
        return

    cfg = load_state()
    errors, warnings = validate(cfg)
    r = resolve(cfg)
    t = r["timing"]
    hz_str, pll1_str, pll2_str = _hse_resolved_line(r)
    st_str, st_flags = _storage_resolved_line(r, cfg)

    print("Current hardware configuration  (config/hw_config_state.json):")
    print()
    for key, sdef in SETTINGS.items():
        # Skip hidden conditional settings
        parent = sdef.get("parent")
        if parent and cfg.get(parent) != sdef.get("parent_val"):
            continue
        val = short_label(key, cfg.get(key, sdef["default"]))
        print(f"  {sdef['label']:<22s}: {val}")
    print()
    print("  Resolved:")
    print(f"    LCD    : {r['lcd_w']} x {r['lcd_h']}  (HW_LCD_PANEL = {r['panel']})")
    print(f"    MADCTL : 0x{r['madctl']:02X}  [{r['madctl_desc'].strip()}]")
    print(f"    FMC    : ADDR_SETUP={t['addr_setup']}  DATA_SETUP={t['data_setup']}"
          f"  BUS_TURN={t['bus_turn']}")
    print(f"    GPIO   : {r['gpio_speed']}")
    print(f"    DMA    : {r['dma_rows']} rows/strip")
    print(f"    HSE    : {hz_str}")
    print(f"           -> {pll1_str}")
    print(f"           -> {pll2_str}")
    print(f"    NVM    : {st_str}")
    if st_flags:
        print(f"           -> {st_flags}")

    if errors:
        print()
        print("  [ERRORS]")
        for e in errors:
            print(f"    - {e}")
    if warnings:
        print()
        print("  [WARNINGS]")
        for w in warnings:
            print(f"    - {w}")


def cmd_apply():
    if not os.path.isfile(STATE_FILE):
        print("ERROR: No saved configuration found.", file=sys.stderr)
        print("Run 'python tools/hw_config.py' first.", file=sys.stderr)
        sys.exit(1)

    cfg = load_state()
    print("Applying saved hardware configuration...")
    ok = apply_config(cfg)
    sys.exit(0 if ok else 1)


def cmd_interactive():
    cfg   = load_state()
    final = run_interactive(cfg)
    if final is None:
        clr()
        print("Exited without generating config.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="CSDR firmware interactive hardware configuration tool.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--apply", action="store_true",
                        help="Apply saved config non-interactively and exit.")
    parser.add_argument("--show",  action="store_true",
                        help="Print current saved config and exit.")
    args = parser.parse_args()

    if args.show:
        cmd_show()
    elif args.apply:
        cmd_apply()
    else:
        cmd_interactive()


if __name__ == "__main__":
    main()
