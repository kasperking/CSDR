"""
CSDR SDR UI mockup renderer — 480x320 PNG
Zone geometry and colour palette are parsed live from the C headers,
so the render stays correct after code changes.

  python render_ui.py [output.png]
"""

import re, math, random, sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

# ══════════════════════════════════════════════════════════════════════════════
#  1.  Header parser — reads #define constants from C headers
# ══════════════════════════════════════════════════════════════════════════════
_STRIP_U    = re.compile(r'(\b(?:0x[0-9a-fA-F]+|\d+))[UuLl]+')
_STRIP_CAST = re.compile(r'\((?:u?int\d+_t|unsigned\s+long(?:\s+int)?|unsigned\s+int|long)\)')
_OBJ_MACRO  = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)\s+(.+?)(?:\s*/\*.*)?$')
_FUNC_MACRO = re.compile(r'#\s*define\s+\w+\s*\(')   # function-like — skip

def parse_defines(path, known=None):
    """
    Extract integer/hex #define values from a C header.
    Handles cascaded defines and simple arithmetic expressions.
    Returns a new dict merging `known` with any new names found.
    """
    defs = dict(known or {})
    try:
        lines = Path(path).read_text(encoding='utf-8', errors='ignore').splitlines()
    except FileNotFoundError:
        print(f"  WARNING: header not found: {path}")
        return defs

    for line in lines:
        if _FUNC_MACRO.search(line):
            continue
        m = _OBJ_MACRO.match(line)
        if not m:
            continue
        name, raw = m.group(1), m.group(2).strip()
        clean = _STRIP_CAST.sub('', raw)          # remove (uint16_t) etc.
        clean = _STRIP_U.sub(r'\1', clean)        # remove U/u/L/l suffixes
        clean = clean.replace('true', 'True').replace('false', 'False')
        try:
            val = eval(clean, {"__builtins__": {}, "abs": abs}, defs)
            if isinstance(val, (int, float)):
                defs[name] = int(val)
        except Exception:
            pass
    return defs

HERE = Path(__file__).parent
INC  = HERE / "BSP" / "Inc"

print("Parsing headers …")
defs = parse_defines(INC / "lcd_render.h")
defs = parse_defines(INC / "sdr_ui.h", defs)
print(f"  {len(defs)} defines loaded")

def G(name, fallback=None):
    """Look up a parsed define; warn and use fallback if missing."""
    if name not in defs:
        print(f"  WARNING: #{name} not found — using fallback {fallback}")
        if fallback is None:
            raise KeyError(f"Required define {name!r} missing from headers")
        return fallback
    return defs[name]

# ══════════════════════════════════════════════════════════════════════════════
#  2.  RGB565 → RGB888
# ══════════════════════════════════════════════════════════════════════════════
def c(rgb565):
    r = ((rgb565 >> 11) & 0x1F) * 255 // 31
    g = ((rgb565 >>  5) & 0x3F) * 255 // 63
    b = ( rgb565        & 0x1F) * 255 // 31
    return (r, g, b)

# ══════════════════════════════════════════════════════════════════════════════
#  3.  Zone geometry — all from parsed headers
# ══════════════════════════════════════════════════════════════════════════════
W  = G('LCD_W', 480)
H  = G('LCD_H', 320)

HDR_Y  = G('HDR_Y',   0);  HDR_H  = G('HDR_H',  24)
SBL_X  = G('SBL_X',   0);  SBL_W  = G('SBL_W',  80)
SBL_Y  = G('SBL_Y',  24);  SBL_H  = G('SBL_H',  96)
SBR_X  = G('SBR_X', 400);  SBR_W  = G('SBR_W',  80)
SBR_Y  = G('SBR_Y',  24);  SBR_H  = G('SBR_H',  96)
VFO_X  = G('VFO_X',  80);  VFO_W  = G('VFO_W', 320)
VFO_Y  = G('VFO_Y',  24);  VFO_H  = G('VFO_H',  64)
MTR_X  = G('MTR_X',  80);  MTR_W  = G('MTR_W', 320)
MTR_Y  = G('MTR_Y',  88);  MTR_H  = G('MTR_H',  32)
INFO_Y = G('INFO_Y', 120); INFO_H  = G('INFO_H',  24)
SPEC_Y = G('SPEC_Y', 144); SPEC_H  = G('SPEC_H',  72)
SPEC_W = G('SPEC_W',   W)
WF_Y   = G('WF_Y',  216);  WF_H   = G('WF_H',   72)
FTR_Y  = G('FTR_Y', 288);  FTR_H  = G('FTR_H',  32)

SM_BARS       = G('SM_BARS',        12)
SM_UNIT_W     = G('SM_UNIT_W',      18)   # segment pitch (18 × 12 = 216 px)
SM_START_X    = G('SM_START_X',      2)
SM_LBL_R0     = 1                         # label band start row (Font5x8 = 8 rows)
SM_TICK_R0    = 10                        # first tick row
SM_TICK_H_MAJ = 4                         # major tick height (ST7796)
SM_TICK_H_MIN = 2                         # minor tick height
SM_RAIL_TOP_R = 14                        # top rail row
SM_RAIL_BOT_R = 22                        # bottom rail row
SM_RULER_W    = SM_BARS * SM_UNIT_W       # 216 px
SM_LINE_H     = 4                         # signal line height in rows
SM_LINE_R0    = (SM_RAIL_TOP_R + SM_RAIL_BOT_R + 1 - SM_LINE_H) // 2  # centred between rails

# ══════════════════════════════════════════════════════════════════════════════
#  4.  Colour palette — all from parsed headers
# ══════════════════════════════════════════════════════════════════════════════
BG          = c(G('UI_BG',          0x0000))
DIVIDER     = c(G('UI_DIVIDER',     0x10A2))
FREQ_MHZ    = c(G('UI_FREQ_MHZ',    0x07FF))
FREQ_KHZ    = c(G('UI_FREQ_KHZ',    0x3FE0))
FREQ_FG     = c(G('UI_FREQ_FG',     0xFFFF))
FREQ_SUB    = c(G('UI_FREQ_SUB',    0x528A))
STATUS_LBL  = c(G('UI_STATUS_LBL',  0x4E7D))
STATUS_VAL  = c(G('UI_STATUS_VAL',  0xFFFF))
STATUS_ON   = c(G('UI_STATUS_ON',   0x07E0))
STATUS_OFF  = c(G('UI_STATUS_OFF',  0xF800))
S1_6        = c(G('UI_S1_6',        0x07E0))
S7_9        = c(G('UI_S7_9',        0xFFE0))
S9P         = c(G('UI_S9P',         0xF800))
SMETER_BG   = c(G('UI_SMETER_BG',   0x1082))
SMETER_TICK = c(G('UI_SMETER_TICK', 0xC618))   # bright gray: scale, ticks, rails
SMETER_ACT  = c(G('UI_SMETER_ACT',  0x06C0))   # RF green: active signal line
TX_COL      = c(G('UI_TX_BG',       0xF800))
RX_COL      = c(G('UI_RX_BG',       0x07E0))
SPEC_BG     = c(G('UI_SPEC_BG',     0x0843))
SPEC_GRID   = c(G('UI_SPEC_GRID',   0x18C6))
SPEC_CTR    = c(G('UI_SPEC_CENTER', 0xF81F))
SPEC_BW     = c(G('UI_SPEC_BW',     0x07FF))
SPEC_TRACE  = STATUS_ON   # green

MODE_COLORS = {
    "AM":   c(G('UI_MODE_AM',   0xFFFF)),
    "FM":   c(G('UI_MODE_FM',   0x07E0)),
    "USB":  c(G('UI_MODE_USB',  0xF800)),
    "LSB":  c(G('UI_MODE_LSB',  0xFFE0)),
    "CW":   c(G('UI_MODE_CW',   0x07FF)),
    "DIGU": c(G('UI_MODE_DIGU', 0xFD20)),
    "DIGL": c(G('UI_MODE_DIGL', 0xFCC0)),
}

# ══════════════════════════════════════════════════════════════════════════════
#  5.  Sample UI state (edit these to try different receiver settings)
# ══════════════════════════════════════════════════════════════════════════════
ui = dict(
    freq_hz   = 14_200_000,
    freq_b_hz = 28_500_000,
    mode      = "USB",
    att_db    = 6,
    voltage   = 13.9,
    volume    = 78,
    squelch   = 0,
    step      = 1000,
    nb_on     = False,
    nr_on     = True,
    rit_hz    = 150,
    tx_mode   = False,
    bw_hz     = 3000,
    mic_gain  = 21,
    dsp_level = 6,
    active_vfo= 0,       # 0 = VFO A active
    signal_db = -65.0,
)

# ══════════════════════════════════════════════════════════════════════════════
#  6.  Fonts  (Courier monospace approximates Font6x8 at 1× / 2× / 4×)
# ══════════════════════════════════════════════════════════════════════════════
try:
    FONT_SM  = ImageFont.truetype("cour.ttf",  9)
    FONT_MED = ImageFont.truetype("cour.ttf", 14)
    FONT_BIG = ImageFont.truetype("cour.ttf", 28)
except OSError:
    FONT_SM = FONT_MED = FONT_BIG = ImageFont.load_default()

# ══════════════════════════════════════════════════════════════════════════════
#  7.  Drawing helpers
# ══════════════════════════════════════════════════════════════════════════════
img = Image.new("RGB", (W, H), BG)
d   = ImageDraw.Draw(img)

def rect(x, y, w, h, fill=None, outline=None):
    d.rectangle([x, y, x+w-1, y+h-1], fill=fill or BG, outline=outline)

def hline(y, x0, x1, col):
    d.line([(x0, y), (x1, y)], fill=col)

def vline(x, y0, y1, col):
    d.line([(x, y0), (x, y1)], fill=col)

def tsm(x, y, s, col):  d.text((x, y), s, font=FONT_SM,  fill=col)
def tmd(x, y, s, col):  d.text((x, y), s, font=FONT_MED, fill=col)
def tbg(x, y, s, col):  d.text((x, y), s, font=FONT_BIG, fill=col)

def tw(f, s):  bb = f.getbbox(s); return bb[2] - bb[0]
def th(f, s):  bb = f.getbbox(s); return bb[3] - bb[1]

# ══════════════════════════════════════════════════════════════════════════════
#  8.  Header zone
# ══════════════════════════════════════════════════════════════════════════════
rect(0, HDR_Y, W, HDR_H)
hline(HDR_Y + HDR_H - 1, 0, W - 1, DIVIDER)

badge     = "RX" if not ui["tx_mode"] else "TX"
badge_col = RX_COL if not ui["tx_mode"] else TX_COL
ty = HDR_Y + (HDR_H - th(FONT_SM, "RX")) // 2
tsm(4, ty, badge, badge_col)
tsm(4 + tw(FONT_SM, "RX") + 8, ty, f"ATT:{ui['att_db']}", STATUS_VAL)

vstr = f"{ui['voltage']:.1f}V"
vcol = STATUS_OFF if 0.5 < ui["voltage"] < 11.5 else STATUS_VAL
tsm(W - tw(FONT_SM, vstr) - 6, ty, vstr, vcol)

# ══════════════════════════════════════════════════════════════════════════════
#  9.  Sidebar left  (5 items × ~19 rows)
# ══════════════════════════════════════════════════════════════════════════════
rect(SBL_X, SBL_Y, SBL_W, SBL_H)
item_h = SBL_H // 5

for i in range(5):
    y0 = SBL_Y + 1 + i * item_h
    if i > 0:
        hline(y0, SBL_X, SBL_X + SBL_W - 1, DIVIDER)
    ty = y0 + 4

    if i == 0:                                      # Mode
        ms = ui["mode"]
        mc = MODE_COLORS.get(ms, STATUS_VAL)
        cx = SBL_X + (SBL_W - tw(FONT_SM, ms)) // 2
        tsm(cx, ty, ms, mc)

    elif i == 1:                                    # VFO A / B
        col_a = STATUS_VAL if ui["active_vfo"] == 0 else STATUS_LBL
        col_b = STATUS_ON  if ui["active_vfo"] == 1 else STATUS_LBL
        tsm(SBL_X + 2, ty, "VFO ", STATUS_LBL)
        x = SBL_X + 2 + tw(FONT_SM, "VFO ")
        tsm(x, ty, "A", col_a);  x += tw(FONT_SM, "A")
        tsm(x, ty, "/", STATUS_LBL);  x += tw(FONT_SM, "/")
        tsm(x, ty, "B", col_b)

    elif i == 2:                                    # NR / NB buttons
        nr_c = STATUS_ON  if ui["nr_on"] else STATUS_OFF
        nb_c = STATUS_ON  if ui["nb_on"] else STATUS_OFF
        bw = (SBL_W - 4) // 2 - 2
        rect(SBL_X + 2,      ty - 1, bw, 11, fill=nr_c)
        rect(SBL_X + 4 + bw, ty - 1, bw, 11, fill=nb_c)
        tsm(SBL_X + 2      + (bw - tw(FONT_SM,"NR"))//2, ty, "NR", BG)
        tsm(SBL_X + 4 + bw + (bw - tw(FONT_SM,"NB"))//2, ty, "NB", BG)

    elif i == 3:                                    # VOL
        tsm(SBL_X + 2, ty, "VOL", STATUS_LBL)
        v = str(ui["volume"])
        tsm(SBL_X + SBL_W - tw(FONT_SM, v) - 4, ty, v, STATUS_VAL)

    else:                                           # SQL
        tsm(SBL_X + 2, ty, "SQL", STATUS_LBL)
        v = str(ui["squelch"])
        tsm(SBL_X + SBL_W - tw(FONT_SM, v) - 4, ty, v, STATUS_VAL)

# ══════════════════════════════════════════════════════════════════════════════
#  10. Sidebar right  (4 items × 24 rows)
# ══════════════════════════════════════════════════════════════════════════════
rect(SBR_X, SBR_Y, SBR_W, SBR_H)

rit_hz  = ui["rit_hz"]
rit_str = f"+{rit_hz}" if rit_hz > 0 else (f"{rit_hz}" if rit_hz < 0 else "OFF")
rit_col = FREQ_KHZ if rit_hz else STATUS_LBL

bw_hz  = ui["bw_hz"]
bw_str = (f"{bw_hz//1000}k"    if bw_hz >= 10000 else
          f"{bw_hz//1000}.{(bw_hz%1000)//100}k" if bw_hz >= 1000 else
          f"{bw_hz}Hz")

sbr_items = [
    ("RIT", rit_str,              rit_col),
    ("MIC", str(ui["mic_gain"]), STATUS_VAL),
    ("DSP", str(ui["dsp_level"]), STATUS_VAL),
    ("BW",  bw_str,               FREQ_KHZ),
]
item_h2 = SBR_H // 4
for i, (lbl, val, vc) in enumerate(sbr_items):
    y0 = SBR_Y + i * item_h2
    if i > 0:
        hline(y0, SBR_X, SBR_X + SBR_W - 1, DIVIDER)
    tsm(SBR_X + 2, y0 + 3, lbl, STATUS_LBL)
    tsm(SBR_X + SBR_W - tw(FONT_SM, val) - 4, y0 + 13, val, vc)

# ══════════════════════════════════════════════════════════════════════════════
#  11. VFO zone  (7-segment digits + sub-line)
# ══════════════════════════════════════════════════════════════════════════════
rect(VFO_X, VFO_Y, VFO_W, VFO_H)

freq     = ui["freq_hz"]
mhz_s    = str(freq // 1_000_000)
khz_s    = f"{(freq % 1_000_000) // 1000:03d}"
hz_s     = f"{freq % 1000:03d}"
full_s   = f"{mhz_s}.{khz_s}.{hz_s}"

total_w  = tw(FONT_BIG, full_s)
fx       = VFO_X + (VFO_W - total_w) // 2 - 10
fy       = VFO_Y + 2
tbg(fx, fy, full_s, FREQ_FG)

# Step + BW labels (top-right corner of VFO)
step     = ui["step"]
step_str = f"{step//1000}k" if step >= 1000 else str(step)
tsm(VFO_X + VFO_W - tw(FONT_SM, step_str)        - 3, VFO_Y + 1,  step_str,       STATUS_LBL)
tsm(VFO_X + VFO_W - tw(FONT_SM, f"BW:{bw_str}") - 3, VFO_Y + 10, f"BW:{bw_str}", STATUS_LBL)

# Sub-line + RX/TX badge (30×20px solid fill, white medium text, right-aligned in sub area)
rt_str = "TX" if ui["tx_mode"] else "RX"
rt_col = TX_COL if ui["tx_mode"] else RX_COL
BAD_W, BAD_H = 30, 20
bx = VFO_X + VFO_W - 2 - BAD_W
by = VFO_Y + 28 + (VFO_H - 28 - BAD_H) // 2
rect(bx, by, BAD_W, BAD_H, fill=rt_col)
tmd(bx + 3, by + 2, rt_str, STATUS_VAL)

freq_b = ui["freq_b_hz"]
if freq_b:
    pfx = "B:" if ui["active_vfo"] == 0 else "A:"
    bm, bk, bh = freq_b//1_000_000, (freq_b%1_000_000)//1000, freq_b%1000
    tmd(VFO_X + 4, VFO_Y + 28, f"{pfx}{bm}.{bk:03d}.{bh:03d}", FREQ_SUB)
elif ui["rit_hz"]:
    tsm(VFO_X + 4, VFO_Y + 28, f"RIT {ui['rit_hz']:+d} Hz", FREQ_SUB)
else:
    tmd(VFO_X + 4, by + 2, ui["mode"], STATUS_LBL)

# ══════════════════════════════════════════════════════════════════════════════
#  12. S-meter  — UHSDR-style calibrated ruler + signal line
# ══════════════════════════════════════════════════════════════════════════════
rect(MTR_X, MTR_Y, MTR_W, MTR_H, fill=BG)

sig_db  = ui["signal_db"]
bars    = max(0, min(SM_BARS, int((sig_db + 73.0) / 3.0)))
mk_col  = S9P if bars > 9 else S7_9 if bars > 5 else S1_6

# Major/minor tick segment indices
s_sbar  = [0, 1, 3, 5, 7, 9, 10, 11]   # major (labeled)
s_min   = [2, 4, 6, 8]                  # minor (unlabeled)
slbls   = ["S","1","3","5","7","9","20","40"]

ruler_end = MTR_X + SM_START_X + SM_RULER_W
val_x     = ruler_end + 4

# Calibrated right edge (mirrors sm_mark_x)
if bars <= 0:
    sig_end_x = MTR_X + SM_START_X
elif bars >= SM_BARS:
    sig_end_x = MTR_X + SM_START_X + SM_RULER_W - 1
else:
    sig_end_x = min(MTR_X + SM_START_X + bars * SM_RULER_W // SM_BARS,
                    MTR_X + SM_START_X + SM_RULER_W - 1)

# ── Scale labels in label band (rows SM_LBL_R0 .. +8) ──
for t, (idx, lbl) in enumerate(zip(s_sbar, slbls)):
    tx_px = MTR_X + SM_START_X + idx * SM_UNIT_W
    half_w = len(lbl) * 6 // 2
    lx = max(MTR_X, tx_px - half_w)
    tsm(lx, MTR_Y + SM_LBL_R0, lbl, SMETER_TICK)

# Inline S-value right of ruler
sv_str = f"S{bars}" if bars <= 9 else f"+{(bars-9)*3}"
tsm(val_x, MTR_Y + SM_LBL_R0, sv_str, mk_col)

# ── Major ticks (SM_TICK_H_MAJ rows) ──
for row_off in range(SM_TICK_R0, SM_TICK_R0 + SM_TICK_H_MAJ):
    for idx in s_sbar:
        tx = MTR_X + SM_START_X + idx * SM_UNIT_W
        d.point((tx, MTR_Y + row_off), fill=SMETER_TICK)

# ── Minor ticks (SM_TICK_H_MIN rows, bottom of tick band) ──
for row_off in range(SM_TICK_R0 + SM_TICK_H_MAJ - SM_TICK_H_MIN,
                     SM_TICK_R0 + SM_TICK_H_MAJ):
    for idx in s_min:
        tx = MTR_X + SM_START_X + idx * SM_UNIT_W
        d.point((tx, MTR_Y + row_off), fill=DIVIDER)

# ── Top rail (1-px horizontal) ──
for px in range(MTR_X + SM_START_X, min(ruler_end, MTR_X + MTR_W)):
    d.point((px, MTR_Y + SM_RAIL_TOP_R), fill=SMETER_TICK)

# ── Bottom rail (1-px horizontal) ──
for px in range(MTR_X + SM_START_X, min(ruler_end, MTR_X + MTR_W)):
    d.point((px, MTR_Y + SM_RAIL_BOT_R), fill=SMETER_TICK)

# ── Signal line: SM_LINE_H-px fill from ruler left to calibrated right edge ──
for row_off in range(SM_LINE_R0, SM_LINE_R0 + SM_LINE_H):
    for px in range(MTR_X + SM_START_X, min(sig_end_x + 1, MTR_X + MTR_W)):
        d.point((px, MTR_Y + row_off), fill=SMETER_ACT)

# ══════════════════════════════════════════════════════════════════════════════
#  13. Info strip
# ══════════════════════════════════════════════════════════════════════════════
rect(0, INFO_Y, W, INFO_H)
hline(INFO_Y,          0, W - 1, DIVIDER)
hline(INFO_Y + INFO_H, 0, W - 1, DIVIDER)
func_labels = ["VFO A/B", "MODE", "STEP↑", "ZOOM", "ATT", "SCAN"]
fw = W // len(func_labels)
for i, lbl in enumerate(func_labels):
    fx2 = i * fw + (fw - tw(FONT_SM, lbl)) // 2
    tsm(fx2, INFO_Y + (INFO_H - th(FONT_SM, lbl)) // 2, lbl, STATUS_LBL)

# ══════════════════════════════════════════════════════════════════════════════
#  14. Spectrum
# ══════════════════════════════════════════════════════════════════════════════
rect(0, SPEC_Y, SPEC_W, SPEC_H, fill=SPEC_BG)

for row in range(1, 4):
    hline(SPEC_Y + row * SPEC_H // 4, 0, SPEC_W - 1, SPEC_GRID)
for col in range(60, SPEC_W, 60):
    for y in range(SPEC_Y, SPEC_Y + SPEC_H):
        if (y - SPEC_Y) % 6 < 3:
            d.point((col, y), fill=SPEC_GRID)

random.seed(42)
bw_px = int(ui["bw_hz"] / 48000.0 * SPEC_W)
for x in range(SPEC_W):
    noise = random.gauss(-0.85, 0.04)
    peak  = math.exp(-(abs(x - SPEC_W//2) / 40.0)**2 * 0.5) * 0.6
    peak2 = math.exp(-(abs(x - SPEC_W*3//8) / 15.0)**2) * 0.25
    val   = max(0.0, min(1.0, noise + peak + peak2 + 0.85))
    col_h = int(val * (SPEC_H - 4))
    top_y = SPEC_Y + SPEC_H - col_h
    in_bw = abs(x - SPEC_W // 2) < bw_px // 2
    d.line([(x, top_y), (x, SPEC_Y + SPEC_H - 1)], fill=SPEC_BW if in_bw else SPEC_TRACE)

for y in range(SPEC_Y, SPEC_Y + SPEC_H):
    if (y - SPEC_Y) % 4 < 2:
        d.point((SPEC_W // 2, y), fill=SPEC_CTR)

# ══════════════════════════════════════════════════════════════════════════════
#  15. Waterfall  (Hermite-spline SDR palette matching wf_lut_init)
#      black → dark-navy → blue → cyan → yellow → white, gamma=1.3
# ══════════════════════════════════════════════════════════════════════════════
_WF_SR = [0,  0,  0,  0,  0,  0,  0, 14, 31, 31, 31]
_WF_SG = [0,  0,  1,  4, 10, 38, 63, 63, 63, 63, 63]
_WF_SB = [0,  3, 12, 22, 31, 31, 26,  8,  0, 16, 31]

def _wf_m(k, arr):
    if k == 0:  return arr[1] - arr[0]
    if k == 10: return arr[10] - arr[9]
    return 0.5 * (arr[k+1] - arr[k-1])

def wf_color(norm):
    pos = (norm ** 1.3) * 10.0
    lo  = int(pos)
    if lo >= 10:
        return (255, 255, 255)
    t = pos - lo;  t2 = t*t;  t3 = t2*t
    h00 = 2*t3-3*t2+1;  h10 = t3-2*t2+t;  h01 = -2*t3+3*t2;  h11 = t3-t2
    def ch(arr, mx):
        v = h00*arr[lo] + h10*_wf_m(lo,arr) + h01*arr[lo+1] + h11*_wf_m(lo+1,arr)
        return max(0, min(255, int(v * 255 / mx)))
    return (ch(_WF_SR, 31), ch(_WF_SG, 63), ch(_WF_SB, 31))

wf_img = Image.new("RGB", (W, WF_H), BG)
wfp    = wf_img.load()
for row in range(WF_H):
    age = row / WF_H
    for x in range(W):
        noise = random.gauss(0, 0.04)
        peak  = math.exp(-(abs(x - W//2)     / 40.0)**2) * 0.55 * (1 - age * 0.4)
        peak2 = math.exp(-(abs(x - W*3//8)   / 15.0)**2) * 0.20
        val   = max(0.0, min(1.0, noise + peak + peak2 + 0.08))
        wfp[x, row] = wf_color(val)
img.paste(wf_img, (0, WF_Y))

# ══════════════════════════════════════════════════════════════════════════════
#  16. Footer
# ══════════════════════════════════════════════════════════════════════════════
rect(0, FTR_Y, W, FTR_H)
hline(FTR_Y, 0, W - 1, DIVIDER)
half_k  = 24
ftr_lbl = [f"-{half_k}K", "0", f"+{half_k}K"]
ftr_xs  = [4,
           W//2 - tw(FONT_SM, "0")//2,
           W - tw(FONT_SM, f"+{half_k}K") - 4]
for lbl, fx3 in zip(ftr_lbl, ftr_xs):
    tsm(fx3, FTR_Y + (FTR_H - th(FONT_SM, lbl))//2, lbl, SPEC_GRID)

# ══════════════════════════════════════════════════════════════════════════════
#  17. Zone divider lines
# ══════════════════════════════════════════════════════════════════════════════
vline(SBL_X + SBL_W,  SBL_Y, SBL_Y + SBL_H - 1, DIVIDER)
vline(SBR_X - 1,      SBR_Y, SBR_Y + SBR_H - 1, DIVIDER)
hline(MTR_Y,          VFO_X, SBR_X - 1,          DIVIDER)

# ══════════════════════════════════════════════════════════════════════════════
#  18. Save
# ══════════════════════════════════════════════════════════════════════════════
out = sys.argv[1] if len(sys.argv) > 1 else str(HERE / "ui_render.png")
img.save(out)
print(f"Saved {out}  ({W}×{H} px)")
