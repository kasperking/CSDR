"""
CSDR SDR UI mockup renderer — 480x320 PNG
Mirrors the 9-zone layout in sdr_ui.h / sdr_ui.c exactly.
"""

import math, random
from PIL import Image, ImageDraw, ImageFont

# ── RGB565 → RGB888 ──────────────────────────────────────────────────────────
def c(rgb565):
    r = ((rgb565 >> 11) & 0x1F) * 255 // 31
    g = ((rgb565 >>  5) & 0x3F) * 255 // 63
    b = ( rgb565        & 0x1F) * 255 // 31
    return (r, g, b)

# ── Palette (matches sdr_ui.h) ────────────────────────────────────────────────
BG          = c(0x0000)
BORDER      = c(0x18C6)
DIVIDER     = c(0x10A2)
FREQ_MHZ    = c(0x07FF)   # cyan
FREQ_KHZ    = c(0x3FE0)   # yellow-green
FREQ_HZ     = c(0x2D65)   # medium green
FREQ_DOT    = c(0x07FF)
FREQ_SUB    = c(0x528A)
MODE_USB    = c(0xF800)   # red
STATUS_LBL  = c(0x4E7D)   # steel-blue label
STATUS_VAL  = c(0xFFFF)   # white
STATUS_ON   = c(0x07E0)   # green
STATUS_OFF  = c(0xF800)   # red
S1_6        = c(0x07E0)   # green bars
S7_9        = c(0xFFE0)   # yellow bars
S9P         = c(0xF800)   # red bars
SMETER_BG   = c(0x1082)
SMETER_TICK = c(0x5AEB)
TX_COL      = c(0x001F)   # blue  (TX badge)
RX_COL      = c(0xF800)   # red   (RX badge)
SPEC_BG     = c(0x0843)
SPEC_GRID   = c(0x18C6)
SPEC_TRACE  = c(0x07E0)   # green trace
SPEC_CTR    = c(0xF81F)   # magenta centre line
SPEC_BW     = c(0x07FF)   # cyan BW marker

# ── Zone geometry (mirrors sdr_ui.h) ─────────────────────────────────────────
W, H = 480, 320
HDR_Y,  HDR_H  =   0,  24
SBL_X,  SBL_W  =   0,  80
SBR_X,  SBR_W  = 400,  80
VFO_X,  VFO_W  =  80, 320
VFO_Y,  VFO_H  =  24,  64
MTR_X,  MTR_W  =  80, 320
MTR_Y,  MTR_H  =  88,  32
SB_Y,   SB_H   =  24,  96   # sidebars
INFO_Y, INFO_H = 120,  24
SPEC_Y, SPEC_H = 144,  72
WF_Y,   WF_H   = 216,  72
FTR_Y,  FTR_H  = 288,  32

# ── Sample state ──────────────────────────────────────────────────────────────
ui = dict(
    freq_hz   = 14_200_000,
    freq_b_hz = 28_500_000,
    mode      = "USB",
    att_db    = 6,
    voltage   = 13.9,
    volume    = 78,
    squelch   = 0,
    step      = 1000,
    agc_fast  = False,
    nb_on     = False,
    nr_on     = True,
    rit_hz    = 150,
    tx_mode   = False,
    bw_hz     = 3000,
    mic_gain  = 21,
    dsp_level = 6,
    active_vfo= 0,   # 0=A active
    signal_db = -65.0,
)

# ── Fonts ─────────────────────────────────────────────────────────────────────
# We scale a monospace font to approximate Font6x8 * 1×/2×/4×
try:
    FONT_SM  = ImageFont.truetype("cour.ttf",  9)   # ~6×8
    FONT_MED = ImageFont.truetype("cour.ttf", 14)   # ~12×16
    FONT_BIG = ImageFont.truetype("cour.ttf", 28)   # ~24×32  VFO main
except OSError:
    FONT_SM  = ImageFont.load_default()
    FONT_MED = ImageFont.load_default()
    FONT_BIG = ImageFont.load_default()

# ── Drawing helpers ───────────────────────────────────────────────────────────
img = Image.new("RGB", (W, H), BG)
d   = ImageDraw.Draw(img)

def rect(x, y, w, h, fill=BG, outline=None):
    d.rectangle([x, y, x+w-1, y+h-1], fill=fill, outline=outline)

def hline(y, x0, x1, col):
    d.line([(x0, y), (x1, y)], fill=col)

def vline(x, y0, y1, col):
    d.line([(x, y0), (x, y1)], fill=col)

def text_sm(x, y, s, col):
    d.text((x, y), s, font=FONT_SM, fill=col)

def text_med(x, y, s, col):
    d.text((x, y), s, font=FONT_MED, fill=col)

def text_big(x, y, s, col):
    d.text((x, y), s, font=FONT_BIG, fill=col)

def text_w(font, s):
    bb = font.getbbox(s)
    return bb[2] - bb[0]

def text_h(font, s):
    bb = font.getbbox(s)
    return bb[3] - bb[1]

# ── Header ────────────────────────────────────────────────────────────────────
rect(0, HDR_Y, W, HDR_H)
hline(HDR_Y + HDR_H - 1, 0, W - 1, DIVIDER)

badge = "RX" if not ui["tx_mode"] else "TX"
badge_col = RX_COL if not ui["tx_mode"] else TX_COL

ty = HDR_Y + (HDR_H - text_h(FONT_SM, "RX")) // 2
text_sm(4, ty, badge, badge_col)
att_str = f"ATT:{ui['att_db']}"
text_sm(4 + text_w(FONT_SM, "RX") + 8, ty, att_str, STATUS_VAL)

vstr = f"{ui['voltage']:.1f}V"
vcol = STATUS_OFF if ui["voltage"] < 11.5 and ui["voltage"] > 0.5 else STATUS_VAL
text_sm(W - text_w(FONT_SM, vstr) - 6, ty, vstr, vcol)

# ── Sidebar Left ──────────────────────────────────────────────────────────────
rect(SBL_X, SB_Y, SBL_W, SB_H)
item_h = 19
items_l = [
    ("", ui["mode"],                  c(0xF800) if ui["mode"]=="USB" else STATUS_VAL),
    ("VFO A/B", None,                 None),
    ("NR  NB",  None,                 None),
    ("VOL",     str(ui["volume"]),    STATUS_VAL),
    ("SQL",     str(ui["squelch"]),   STATUS_VAL),
]

for i in range(5):
    y0 = SB_Y + 1 + i * item_h
    if i > 0:
        hline(y0, SBL_X, SBL_X + SBL_W - 1, DIVIDER)
    ty = y0 + 4
    if i == 0:                       # Mode
        ms = ui["mode"]
        mode_colors = {"AM": STATUS_VAL, "FM": STATUS_ON, "USB": c(0xF800),
                       "LSB": c(0xFFE0), "CW": FREQ_MHZ}
        mc = mode_colors.get(ms, STATUS_VAL)
        cx = SBL_X + (SBL_W - text_w(FONT_SM, ms)) // 2
        text_sm(cx, ty, ms, mc)
    elif i == 1:                     # VFO A/B
        col_a = STATUS_VAL if ui["active_vfo"] == 0 else STATUS_LBL
        col_b = STATUS_ON  if ui["active_vfo"] == 1 else STATUS_LBL
        text_sm(SBL_X + 2, ty, "VFO ", STATUS_LBL)
        x = SBL_X + 2 + text_w(FONT_SM, "VFO ")
        text_sm(x, ty, "A", col_a)
        x += text_w(FONT_SM, "A")
        text_sm(x, ty, "/", STATUS_LBL)
        x += text_w(FONT_SM, "/")
        text_sm(x, ty, "B", col_b)
    elif i == 2:                     # NR/NB buttons
        nr_c = STATUS_ON if ui["nr_on"] else STATUS_OFF
        nb_c = STATUS_ON if ui["nb_on"] else STATUS_OFF
        bw = (SBL_W - 4) // 2 - 2
        rect(SBL_X + 2, ty - 1, bw, 11, fill=nr_c)
        text_sm(SBL_X + 2 + (bw - text_w(FONT_SM, "NR")) // 2, ty, "NR", BG)
        rect(SBL_X + 4 + bw, ty - 1, bw, 11, fill=nb_c)
        text_sm(SBL_X + 4 + bw + (bw - text_w(FONT_SM, "NB")) // 2, ty, "NB", BG)
    else:                            # VOL / SQL
        lbl, val = items_l[i][0], items_l[i][1]
        text_sm(SBL_X + 2, ty, lbl, STATUS_LBL)
        vx = SBL_X + SBL_W - text_w(FONT_SM, val) - 4
        text_sm(vx, ty, val, STATUS_VAL)

# ── Sidebar Right ─────────────────────────────────────────────────────────────
rect(SBR_X, SB_Y, SBR_W, SB_H)
rit_hz = ui["rit_hz"]
rit_str = f"+{rit_hz}" if rit_hz > 0 else (f"{rit_hz}" if rit_hz < 0 else "OFF")
rit_col = FREQ_KHZ if rit_hz != 0 else STATUS_LBL
bw_hz = ui["bw_hz"]
bw_str = f"{bw_hz//1000}k" if bw_hz >= 1000 else f"{bw_hz}Hz"

sbr_items = [
    ("RIT", rit_str,              rit_col),
    ("MIC", str(ui["mic_gain"]), STATUS_VAL),
    ("DSP", str(ui["dsp_level"]), STATUS_VAL),
    ("BW",  bw_str,               FREQ_KHZ),
]
item_h2 = 24
for i, (lbl, val, vc) in enumerate(sbr_items):
    y0 = SB_Y + i * item_h2
    if i > 0:
        hline(y0, SBR_X, SBR_X + SBR_W - 1, DIVIDER)
    text_sm(SBR_X + 2, y0 + 3, lbl, STATUS_LBL)
    vx = SBR_X + SBR_W - text_w(FONT_SM, val) - 4
    text_sm(vx, y0 + 13, val, vc)

# ── VFO ───────────────────────────────────────────────────────────────────────
rect(VFO_X, VFO_Y, VFO_W, VFO_H)

freq = ui["freq_hz"]
mhz  = freq // 1_000_000
khz  = (freq % 1_000_000) // 1000
hz_r = freq % 1000

mhz_s = str(mhz)
khz_s = f"{khz:03d}"
hz_s  = f"{hz_r:03d}"

# Big VFO digits (4× ~ font size 28)
# Lay out: MHZ . KHZ . HZ  at row VFO_Y+2
# Measure widths
bw_mhz = text_w(FONT_BIG, mhz_s)
bw_dot = text_w(FONT_BIG, ".")
bw_khz = text_w(FONT_BIG, khz_s)
bw_hz  = text_w(FONT_BIG, hz_s)
# Colour split: mhz=cyan, khz=yellow-green, hz=medium-green
total_freq_w = bw_mhz + bw_dot + bw_khz + bw_dot + bw_hz
fx = VFO_X + (VFO_W - total_freq_w) // 2 - 10
fy = VFO_Y + 2
text_big(fx, fy, mhz_s, FREQ_MHZ)
fx += bw_mhz
text_big(fx, fy, ".", FREQ_DOT)
fx += bw_dot
text_big(fx, fy, khz_s, FREQ_KHZ)
fx += bw_khz
text_big(fx, fy, ".", FREQ_DOT)
fx += bw_dot
text_big(fx, fy, hz_s, FREQ_HZ)

# Step label (top-right)
step = ui["step"]
step_str = f"{step//1000}k" if step >= 1000 else str(step)
text_sm(VFO_X + VFO_W - text_w(FONT_SM, step_str) - 3, VFO_Y + 1, step_str, STATUS_LBL)

# BW label below step
text_sm(VFO_X + VFO_W - text_w(FONT_SM, f"BW:{bw_str}") - 3, VFO_Y + 10, f"BW:{bw_str}", STATUS_LBL)

# Sub-line: inactive VFO (medium 2× font)
freq_b = ui["freq_b_hz"]
pfx = "B:" if ui["active_vfo"] == 0 else "A:"
bm = freq_b // 1_000_000
bk = (freq_b % 1_000_000) // 1000
bh = freq_b % 1000
sub_str = f"{pfx}{bm}.{bk:03d}.{bh:03d}"
rit_label = f"  RIT +{ui['rit_hz']} Hz" if ui["rit_hz"] else ""
sub_full = sub_str + rit_label

sub_y = VFO_Y + 2 + 32 + 2   # below big digits
text_med(VFO_X + 4, sub_y, sub_full, FREQ_SUB)

# ── S-Meter ───────────────────────────────────────────────────────────────────
rect(MTR_X, MTR_Y, MTR_W, MTR_H, fill=SMETER_BG)

BAR_W, BAR_GAP = 13, 1
N_BARS = 12
sm_x0 = MTR_X + 4
bar_top = MTR_Y + 12
bar_bot = MTR_Y + 14

# S-value from dBm: S1=−121, each S-unit=6dB, S9=−73, S9+10=−63…
sig_db = ui["signal_db"]  # e.g. -65
s_val = max(0.0, (sig_db + 121) / 6.0)  # S1=-121, S9=-73
s_bars = min(int(s_val + 0.5), N_BARS)

labels = ["1","3","5","7","9","+10","+20","+30"]
tick_xs = [sm_x0 + i * (BAR_W + BAR_GAP) for i in [0, 2, 4, 6, 8, 9, 10, 11]]

for i in range(N_BARS):
    bx = sm_x0 + i * (BAR_W + BAR_GAP)
    if i < s_bars:
        if   i < 6:  bc = S1_6
        elif i < 9:  bc = S7_9
        else:        bc = S9P
    else:
        bc = SMETER_BG
    rect(bx, bar_top, BAR_W, 3, fill=bc)

# Tick labels
for i, lbl in enumerate(labels):
    tx = tick_xs[i]
    text_sm(tx, MTR_Y + 3, lbl, SMETER_TICK)

# S-value text right side
s_int = int(s_val)
if s_val <= 9:
    sv_str = f"S{s_int}"
else:
    sv_str = f"S9+{int((s_val-9)*10)}"
text_sm(MTR_X + 190, MTR_Y + 12, sv_str, STATUS_VAL)

# ── Info strip ────────────────────────────────────────────────────────────────
rect(0, INFO_Y, W, INFO_H, fill=BG)
hline(INFO_Y,           0, W - 1, DIVIDER)
hline(INFO_Y + INFO_H - 1, 0, W - 1, DIVIDER)
func_labels = ["VFO A/B", "MODE", "STEP↑", "ZOOM", "ATT", "SCAN"]
fw = W // len(func_labels)
for i, lbl in enumerate(func_labels):
    fx2 = i * fw + (fw - text_w(FONT_SM, lbl)) // 2
    text_sm(fx2, INFO_Y + (INFO_H - text_h(FONT_SM, lbl)) // 2, lbl, STATUS_LBL)

# ── Spectrum ──────────────────────────────────────────────────────────────────
rect(0, SPEC_Y, W, SPEC_H, fill=SPEC_BG)

# Grid lines (horizontal, 4 rows)
for row in range(1, 4):
    gy = SPEC_Y + row * SPEC_H // 4
    hline(gy, 0, W - 1, SPEC_GRID)

# Grid lines (vertical, every 60px)
for col in range(60, W, 60):
    for y in range(SPEC_Y, SPEC_Y + SPEC_H):
        if (y - SPEC_Y) % 6 < 3:
            d.point((col, y), fill=SPEC_GRID)

# Fake FFT trace — noise floor + signal peaks (USB signal at 14.200)
random.seed(42)
spec = []
for x in range(W):
    noise = random.gauss(-0.85, 0.04)
    # Main signal peak near centre
    dist = abs(x - W // 2) / 40.0
    peak = math.exp(-dist * dist * 0.5) * 0.6
    # A few interferers
    d2 = abs(x - W * 3 // 8) / 15.0
    peak2 = math.exp(-d2 * d2) * 0.25
    val = max(0.0, min(1.0, noise + peak + peak2 + 0.85))
    spec.append(val)

for x in range(W):
    col_h = int(spec[x] * (SPEC_H - 4))
    top_y = SPEC_Y + SPEC_H - col_h
    # BW shading (cyan tint within ±BW/2 of centre)
    bw_px = int((ui["bw_hz"] / 48000.0) * W)
    in_bw = abs(x - W // 2) < bw_px // 2
    trace_col = SPEC_BW if in_bw else SPEC_TRACE
    d.line([(x, top_y), (x, SPEC_Y + SPEC_H - 1)], fill=trace_col)

# Centre marker (magenta vertical)
for y in range(SPEC_Y, SPEC_Y + SPEC_H):
    if (y - SPEC_Y) % 4 < 2:
        d.point((W // 2, y), fill=SPEC_CTR)

# ── Waterfall ─────────────────────────────────────────────────────────────────
# Hermite-spline thermal palette (matches wf_lut_init in sdr_ui.c)
def wf_color(norm):
    """norm in [0,1] → RGB using same knot table as wf_lut_init"""
    sr = [ 0, 0, 0, 0, 0, 0, 0, 0, 8,16,31]
    sg = [ 0, 0, 8,20,40,63,63,63,63,63,63]
    sb = [ 0,16,31,31,31,31,24,16, 8, 0,31]
    ng = norm ** 1.2
    pos = ng * 10.0
    lo = int(pos)
    if lo >= 10:
        return (255, 255, 255)
    t = pos - lo
    t2, t3 = t*t, t*t*t
    h00 = 2*t3-3*t2+1; h10 = t3-2*t2+t; h01 = -2*t3+3*t2; h11 = t3-t2
    def m(k, arr):
        if k == 0:   return arr[1]-arr[0]
        if k == 10:  return arr[10]-arr[9]
        return 0.5*(arr[k+1]-arr[k-1])
    def interp(arr):
        v = h00*arr[lo] + h10*m(lo,arr) + h01*arr[lo+1] + h11*m(lo+1,arr)
        return max(0, min(255, int(v * 255 / 63)))
    r5 = h00*sr[lo]+h10*(sr[1]-sr[0] if lo==0 else 0.5*(sr[lo+1]-sr[lo-1]))+h01*sr[lo+1]+h11*(sr[10]-sr[9] if lo+1==10 else 0.5*(sr[lo+2]-sr[lo]))
    # simpler: just scale each channel
    rv = h00*sr[lo]+h10*m(lo,sr)+h01*sr[lo+1]+h11*m(lo+1,sr)
    gv = h00*sg[lo]+h10*m(lo,sg)+h01*sg[lo+1]+h11*m(lo+1,sg)
    bv = h00*sb[lo]+h10*m(lo,sb)+h01*sb[lo+1]+h11*m(lo+1,sb)
    return (max(0,min(255,int(rv*255/31))), max(0,min(255,int(gv*255/63))), max(0,min(255,int(bv*255/31))))

wf_img = Image.new("RGB", (W, WF_H), BG)
wfp = wf_img.load()
for row in range(WF_H):
    age = row / WF_H   # older = closer to 0 = cooler
    for x in range(W):
        noise = random.gauss(0, 0.04)
        dist  = abs(x - W // 2) / 40.0
        peak  = math.exp(-dist*dist*0.5) * 0.55 * (1 - age * 0.4)
        d2    = abs(x - W * 3 // 8) / 15.0
        peak2 = math.exp(-d2*d2) * 0.20
        val   = max(0.0, min(1.0, noise + peak + peak2 + 0.08))
        wfp[x, row] = wf_color(val)

img.paste(wf_img, (0, WF_Y))

# ── Footer ────────────────────────────────────────────────────────────────────
rect(0, FTR_Y, W, FTR_H, fill=BG)
hline(FTR_Y, 0, W - 1, DIVIDER)
half_k = 24   # ±24 kHz at zoom=0
ftr_lbl = [f"-{half_k}K", "0", f"+{half_k}K"]
ftr_x   = [4, W//2 - text_w(FONT_SM,"0")//2, W - text_w(FONT_SM,f"+{half_k}K") - 4]
for lbl, fx3 in zip(ftr_lbl, ftr_x):
    text_sm(fx3, FTR_Y + (FTR_H - text_h(FONT_SM, lbl)) // 2, lbl, SPEC_GRID)

# ── Zone border lines ─────────────────────────────────────────────────────────
# Vertical dividers between SBL | VFO | SBR
vline(SBL_X + SBL_W,     SB_Y, SB_Y + SB_H - 1, DIVIDER)
vline(SBR_X - 1,         SB_Y, SB_Y + SB_H - 1, DIVIDER)
# Horizontal between VFO and MTR
hline(MTR_Y, VFO_X, SBR_X - 1, DIVIDER)
# Horizontal between top panel and INFO strip
hline(INFO_Y, 0, W - 1, DIVIDER)
hline(INFO_Y + INFO_H, 0, W - 1, DIVIDER)

# ── Save ──────────────────────────────────────────────────────────────────────
out = "h:/CSDR/ui_render.png"
img.save(out)
print(f"Saved {out}  ({W}x{H} px)")
