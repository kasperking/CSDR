#!/usr/bin/env python3
"""
select_hw_profile.py  — CSDR firmware hardware profile selector.

Usage:
    python tools/select_hw_profile.py <profile_name>
    python tools/select_hw_profile.py          # list available profiles

Examples:
    python tools/select_hw_profile.py hw_test_fmc
    python tools/select_hw_profile.py hw_prod_v1
    python tools/select_hw_profile.py hw_long_fpc_debug

After switching profile, rebuild the entire firmware project.
See HARDWARE_PROFILES.md for full documentation.
"""

import sys
import os
import re
import argparse

# ── Required macros every profile must define ─────────────────────────────────
REQUIRED_MACROS = [
    "HW_PROFILE_NAME",
    "HW_LCD_PANEL",
    "HW_LCD_MADCTL",
    "HW_FMC_ADDR_SETUP",
    "HW_FMC_ADDR_HOLD",
    "HW_FMC_DATA_SETUP",
    "HW_FMC_BUS_TURN",
    "HW_FMC_CLK_DIV",
    "HW_FMC_DATA_LATENCY",
    "HW_FMC_GPIO_SPEED",
    "HW_DMA_CHUNK_ROWS",
]

VALID_GPIO_SPEEDS = {
    "GPIO_SPEED_FREQ_LOW",
    "GPIO_SPEED_FREQ_MEDIUM",
    "GPIO_SPEED_FREQ_HIGH",
    "GPIO_SPEED_FREQ_VERY_HIGH",
}

# Per STM32H7 RM0433 FMC NOR/SRAM timing register limits
FMC_TIMING_RANGES = {
    "HW_FMC_ADDR_SETUP":   (0,  15),
    "HW_FMC_ADDR_HOLD":    (1,  15),
    "HW_FMC_DATA_SETUP":   (1, 255),
    "HW_FMC_BUS_TURN":     (0,  15),
    "HW_FMC_CLK_DIV":      (2,  16),
    "HW_FMC_DATA_LATENCY": (2,  17),
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def find_root(script_path):
    """Return project root (parent of tools/), validated by BSP/Inc presence."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(script_path)))
    if not os.path.isdir(os.path.join(root, "BSP", "Inc")):
        raise FileNotFoundError(
            f"Project root not found from script location: {root}\n"
            "Expected to find BSP/Inc/ beneath the root directory."
        )
    return root


def parse_defines(text):
    """
    Extract bare #define NAME VALUE pairs from C source text.
    Returns dict mapping name -> raw value string (stripped, comments removed).
    Block comments and line comments on the value are stripped.
    """
    # Strip block comments first
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    defines = {}
    pattern = re.compile(
        r'^\s*#\s*define\s+(\w+)\s+(.*?)(?:\s*//.*)?$',
        re.MULTILINE
    )
    for m in pattern.finditer(text):
        name = m.group(1)
        value = m.group(2).strip()
        if value:
            defines[name] = value
    return defines


def parse_int(value_str):
    """Parse a C integer literal (decimal or hex, optional U/u/L suffix)."""
    v = value_str.rstrip('uUlL')
    try:
        if v.lower().startswith('0x'):
            return int(v, 16)
        return int(v, 10)
    except ValueError:
        return None


def list_profiles(profiles_dir):
    """Return sorted list of profile names (filename stems) in profiles_dir."""
    if not os.path.isdir(profiles_dir):
        return []
    return sorted(
        os.path.splitext(f)[0]
        for f in os.listdir(profiles_dir)
        if f.endswith('.h')
    )


# ── Validation ────────────────────────────────────────────────────────────────

def validate(defines, profile_name):
    """
    Validate all required macros and their values.
    Returns a list of error strings; empty list = valid.
    """
    errors = []

    # 1. All required macros present
    missing = [m for m in REQUIRED_MACROS if m not in defines]
    if missing:
        for m in missing:
            errors.append(f"Missing required macro: {m}")
        return errors  # can't validate values if macros are absent

    # 2. HW_PROFILE_NAME must match filename (without quotes)
    name_raw = defines["HW_PROFILE_NAME"]
    name_val = name_raw.strip('"').strip("'")
    if name_val != profile_name:
        errors.append(
            f'HW_PROFILE_NAME "{name_val}" does not match filename "{profile_name}"'
        )

    # 3. HW_LCD_PANEL: 1 or 2
    panel = parse_int(defines["HW_LCD_PANEL"])
    if panel not in (1, 2):
        errors.append(
            f"HW_LCD_PANEL must be 1 (ST7796) or 2 (ST7789), "
            f"got: {defines['HW_LCD_PANEL']}"
        )

    # 4. HW_LCD_MADCTL: 0x00..0xFF
    madctl = parse_int(defines["HW_LCD_MADCTL"])
    if madctl is None or not (0x00 <= madctl <= 0xFF):
        errors.append(
            f"HW_LCD_MADCTL must be 0x00..0xFF, got: {defines['HW_LCD_MADCTL']}"
        )

    # 5. FMC timing ranges
    for macro, (lo, hi) in FMC_TIMING_RANGES.items():
        v = parse_int(defines[macro])
        if v is None or not (lo <= v <= hi):
            errors.append(
                f"{macro} must be {lo}..{hi}, got: {defines[macro]}"
            )

    # 6. GPIO speed: must be a known HAL constant
    speed = defines["HW_FMC_GPIO_SPEED"].strip()
    if speed not in VALID_GPIO_SPEEDS:
        errors.append(
            f"HW_FMC_GPIO_SPEED must be one of "
            f"{', '.join(sorted(VALID_GPIO_SPEEDS))}, got: {speed}"
        )

    # 7. DMA chunk rows: 1..64
    rows = parse_int(defines["HW_DMA_CHUNK_ROWS"])
    if rows is None or not (1 <= rows <= 64):
        errors.append(
            f"HW_DMA_CHUNK_ROWS must be 1..64, got: {defines['HW_DMA_CHUNK_ROWS']}"
        )

    return errors


# ── Output generation ─────────────────────────────────────────────────────────

ACTIVE_HEADER_TEMPLATE = """\
/* AUTO-GENERATED — do not edit manually.
 * Run: python tools/select_hw_profile.py {profile_name}
 * Source: config/hw_profiles/{profile_name}.h
 */

#ifndef HW_CONFIG_ACTIVE_H
#define HW_CONFIG_ACTIVE_H

{profile_body}

#endif /* HW_CONFIG_ACTIVE_H */
"""


def generate_active_header(profile_name, profile_text):
    """
    Build the content of hw_config_active.h from the raw profile text.
    Strips the profile's own include guards (if any) and wraps in
    the standard HW_CONFIG_ACTIVE_H guard.
    """
    lines = []
    guard_define = None

    for line in profile_text.splitlines():
        stripped = line.strip()

        # Detect and skip the profile's own include guard
        if stripped.startswith('#ifndef ') and guard_define is None:
            guard_define = stripped.split()[1]
            continue
        if guard_define and stripped == f'#define {guard_define}':
            continue
        if guard_define and stripped == f'#endif /* {guard_define} */':
            continue
        if guard_define and stripped == f'#endif  /* {guard_define} */':
            continue
        if guard_define and re.fullmatch(r'#endif.*', stripped) and guard_define in stripped:
            continue

        lines.append(line)

    # Trim leading/trailing blank lines from body
    body = '\n'.join(lines).strip()

    return ACTIVE_HEADER_TEMPLATE.format(
        profile_name=profile_name,
        profile_body=body,
    )


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Select a hardware profile for the CSDR firmware build.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "profile",
        nargs="?",
        metavar="PROFILE_NAME",
        help="Profile to activate (omit to list available profiles).",
    )
    args = parser.parse_args()

    try:
        root = find_root(sys.argv[0])
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

    profiles_dir = os.path.join(root, "config", "hw_profiles")
    output_path  = os.path.join(root, "BSP", "Inc", "hw_config_active.h")

    available = list_profiles(profiles_dir)

    # ── List mode ──────────────────────────────────────────────────────────────
    if args.profile is None:
        if available:
            print("Available hardware profiles:")
            for p in available:
                print(f"  {p}")
        else:
            print(f"No profiles found in {profiles_dir}")
        print()
        print(f"Usage: python tools/select_hw_profile.py <profile_name>")
        sys.exit(0)

    profile_name = args.profile

    # ── Duplicate / case-conflict check ───────────────────────────────────────
    dupes = [p for p in available
             if p.lower() == profile_name.lower() and p != profile_name]
    if dupes:
        print(
            f"ERROR: Profile name '{profile_name}' conflicts with: {dupes}",
            file=sys.stderr,
        )
        sys.exit(1)

    # ── Find profile file ─────────────────────────────────────────────────────
    profile_path = os.path.join(profiles_dir, profile_name + ".h")
    if not os.path.isfile(profile_path):
        print(f"ERROR: Profile '{profile_name}' not found.", file=sys.stderr)
        if available:
            print(f"Available: {', '.join(available)}", file=sys.stderr)
        else:
            print(f"No profiles found in: {profiles_dir}", file=sys.stderr)
        sys.exit(1)

    # ── Read and parse ────────────────────────────────────────────────────────
    with open(profile_path, encoding="utf-8") as fh:
        profile_text = fh.read()

    defines = parse_defines(profile_text)

    # ── Validate ──────────────────────────────────────────────────────────────
    errors = validate(defines, profile_name)
    if errors:
        print(f"ERROR: Profile '{profile_name}' failed validation:", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        sys.exit(1)

    # ── Write hw_config_active.h ──────────────────────────────────────────────
    content = generate_active_header(profile_name, profile_text)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(content)

    # ── Report ────────────────────────────────────────────────────────────────
    panel_names = {"1": "ST7796 480x320 landscape", "2": "ST7789 240x320 portrait"}
    panel_key   = defines["HW_LCD_PANEL"].rstrip("uUlL")
    panel_desc  = panel_names.get(panel_key, f"panel {panel_key}")

    print(f"[hw_profile] Selected: {profile_name}")
    print(f"  Panel    : {panel_desc}")
    print(f"  MADCTL   : {defines['HW_LCD_MADCTL']}")
    print(f"  FMC      : ADDR_SETUP={defines['HW_FMC_ADDR_SETUP']} "
          f"DATA_SETUP={defines['HW_FMC_DATA_SETUP']} "
          f"BUS_TURN={defines['HW_FMC_BUS_TURN']}")
    print(f"  GPIO spd : {defines['HW_FMC_GPIO_SPEED']}")
    print(f"  DMA rows : {defines['HW_DMA_CHUNK_ROWS']}")
    print(f"  Output   : BSP/Inc/hw_config_active.h")
    print()
    print("Rebuild the firmware to apply the new profile.")


if __name__ == "__main__":
    main()
