# CSDR Firmware – Developer & User Notes

Target: STM32H750VBT6 · Display: 320×240 ST7789 (SPI, DMA) · Audio: WM8731 (SAI I²S)

---

## 1. UI Layout Overview

```
Y=  0 ┌────────────────────────────────────────────────┐
      │ HEADER  320×18  [RX/TX] ATT:n  BAT 13.9V      │
Y= 18 ├────────┬──────────────────────────┬────────────┤
      │SBL 60  │ VFO 200×44               │ SBR 60     │
      │ Mode   │  7.100.000               │ RIT  OFF   │
      │ VFO A  │  B:7.100.000  1kHz       │ MIC  50    │
      │ NR NB  ├──────────────────────────┤ DSP  –     │
      │ VOL 78 │ METER 200×38             │ BW  2.7k   │
      │ SQL  0 │  S▐▐▐▐░░░  S7           │            │
      │        │  BATT 13.9V              │            │
Y=100 ├────────┴──────────────────────────┴────────────┤
      │ SPECTRUM  320×68                               │
Y=168 ├────────────────────────────────────────────────┤
      │ WATERFALL 320×62                               │
Y=230 ├────────────────────────────────────────────────┤
      │ FOOTER 320×10  –24k    0    +24k               │
Y=240 └────────────────────────────────────────────────┘
```

**Zone pixel extents (X×Y):**

| Zone      | X       | Y        | Size px  |
|-----------|---------|----------|----------|
| HEADER    | 0–319   | 0–17     | 320×18   |
| SBL       | 0–59    | 18–99    | 60×82    |
| VFO       | 60–259  | 18–61    | 200×44   |
| METER     | 60–259  | 62–99    | 200×38   |
| SBR       | 260–319 | 18–99    | 60×82    |
| SPECTRUM  | 0–319   | 100–167  | 320×68   |
| WATERFALL | 0–319   | 168–229  | 320×62   |
| FOOTER    | 0–319   | 230–239  | 320×10   |

**Rendering model:** Each zone has a dedicated DMA\_SRAM buffer. Draws render to the buffer, then push in a single SPI block transfer. The menu overwrites the spectrum+waterfall area (Y=100..239) directly via scanline DMA.

**SBL (sidebar left) items:** Mode badge · VFO A/B · NR/NB toggle badges · VOL value · SQL value  
**SBR (sidebar right) items:** RIT value · MIC gain · DSP level · BW (filter bandwidth, e.g. `2.7k`, `500Hz`)  
**VFO:** 3× scaled digits (15px wide, 24px tall), MHz·kHz·Hz groups, step label top-right. Sub-line shows inactive VFO frequency at 2× scale (12px wide, 16px tall) or RIT offset at base Font6x8 scale.

---

## 2. Key / Button Behavior

All keys are debounced via `Key_Poll` + `Key_Press` / `Key_PressOrRepeat`. Hold-repeat fires continuously while held.

| Key      | Menu closed                        | Menu open                        |
|----------|------------------------------------|----------------------------------|
| **MENU** | Open menu (loads current SDR state)| Close menu (restores main UI)    |
| **F1**   | Volume Down −2 (floor 0), repeat   | Cursor Up                        |
| **F2**   | Volume Up +2 (cap 100), repeat     | Cursor Down                      |
| **F3**   | *(no action)*                      | Confirm / execute ACTION item    |
| **F4**   | *(no action)*                      | Back / exit menu level           |
| **BAND** | Next band (BPF/LPF switched)       | *(no action)*                    |
| **MODE** | Cycle AM→FM→USB→LSB→CW→AM         | *(no action)*                    |
| **PTT**  | Toggle TX on/off                   | *(no action)*                    |
| **ENC↕** | Tune frequency by current step     | Menu encoder-edit (item value)   |
| **ENC⊙** | Cycle mode                         | Select item / run ACTION         |
| **ENC⊙** *(long)* | Cycle spectrum zoom level | *(no action)*               |

**Volume setter:** `cat_set_volume(vol)` writes WM8731 HP register (range 90–121 ≙ −31 dB to 0 dB; vol=0 forces hardware mute at register 0x2F) and sets `display_dirty = true`. The display update occurs on the next 200 ms refresh tick.

**F4 in SWR scan:** hold during sweep to abort; press after scan complete to exit.

---

## 3. Spectrum / Waterfall Behavior

### Spectrum (Y=100..167)

- **Source:** `dsp.fft_mag_db[256]` — power values (re²+im²) after fftshift, pre-log.  
- **Compression:** `pwr_compress()` uses bit-manipulation log₂ approximation at render time (not in DSP path), avoiding 256 `log10f` calls per FFT frame.  
- **Bar graph:** bottom-up per column, colour gradient: dark-blue → green → yellow → red.  
- **Grid:** horizontal lines at 25 / 50 / 75 % height; vertical dots every 40 px.  
- **BW markers:** cyan vertical lines at demod filter edges, derived from `bw_hz / sample_rate` ratio scaled to zoom window.  
- **Center marker:** magenta vertical line at DC (centre bin).  
- **Update:** fires when `dsp.fft_ready == true` and menu is closed. Inherent rate is ~187.5 FFT frames/sec (48 kHz ÷ 256 samples/block = one FFT per block). In practice limited to ~25 fps by the 40 ms waterfall tick.

### Waterfall (Y=168..229)

- **IIR smoothing:** α=0.72 per bin (`s_wf_smooth[b] = 0.72×prev + 0.28×new`).  
- **Level mapping:** `10×log10(pwr) − 42.14 dB`, clipped to [−120 dB, −20 dB] → 0–255 index.  
- **Colour palette:** 256-entry Hermite-spline thermal LUT built at init (black→dark-blue→cyan→green→yellow→white).  
- **Scroll:** ring buffer `s_wf_buf[62][320]`. Head decrements on each new line; a 2-split SPI push (Block A: head..end, Block B: 0..head−1) achieves true scroll with no `memcpy`.  
- **Rate cap:** at most one waterfall line per 40 ms (25 fps), enforced in `CSDR_Loop`.  
- **Both spectrum and waterfall are suppressed when the menu is open.**

---

## 4. FFT Span Modes (Zoom)

The DSP always runs at ±24 kHz (full 48 kHz sample rate). Zoom is display-only: `spec_window()` derives a centered bin window from the 256-bin output.

| Zoom level | Half-span | Visible bins | Footer label |
|-----------|-----------|-------------|--------------|
| 0 (default)| ±24 kHz  | 256          | −24k … +24k  |
| 1         | ±18 kHz   | 192          | −18k … +18k  |
| 2         | ±12 kHz   | 128          | −12k … +12k  |
| 3         | ±6 kHz    | 64           | −6k … +6k    |
| 4         | ±3 kHz    | 32           | −3k … +3k    |

**Toggle:** encoder long-press cycles 0→1→2→3→4→0.  
**Default per mode:** CW=zoom 4, USB/LSB=zoom 3, AM/FM=zoom 0.  
Spectrum and waterfall share the same `spec_window()` call so they stay aligned.  
Footer frequency labels are redrawn immediately on zoom change.

---

## 5. TX / RX Meter Behavior

### RX S-meter

- **Source:** `dsp.signal_power_db` — exponentially smoothed (α=0.9/0.1) pre-AGC band-limited IQ power in dBFS.  
  Measured as RMS of `√(filt_i²+filt_q²)` after the FIR LPF, before demodulation and AGC.  
  Strong signal ≈ 0 dBFS; noise floor ≈ −50…−70 dBFS depending on front-end noise figure.  
- **Bar mapping:** `bars = (signal_db + 73) / 3`, clamped 0–12.  
  - bars=0 ≈ −73 dBFS floor · bars=9 ≈ −46 dBFS · bars=10/11 ≡ S9+20/+40  
- **Segments:** 12 × 13 px + 1 px gap; S1–S6 green, S7–S9 yellow, S9+ red.  
- **Bar height:** 3 px thin bar (rows 14–16 of meter zone).  
- **Value text:** S-unit string below bar (e.g. `S7`, `S9+6`) at rows 18–25.  
- **BATT voltage:** supply voltage shown at rows 27–34 as `BATT X.XV`; red when < 11.5 V.  
- **Scale labels:** `S 1 3 5 7 9 +20 +40`. The `+20` label is shifted 6 px left of its tick position so it clears the adjacent `+40` label (both 18 px wide, 14 px pitch).  
- **Update rate:** via `SDR_UI_UpdateSMeter`, called every 200 ms cycle when menu is closed and not in TX.

### TX ALC + SWR meter

Active when `g_sdr.tx_mode == true`.

- **ALC bar:** same 12-segment geometry as S-meter; scale 0/25/50/75/100 %; source `g_analog.alc_percent`.  
- **SWR text:** label + value only, no bar. Colour: green SWR<2.0, yellow 2.0–3.0, red ≥3.0. Source `g_analog.swr_x100` (÷100 for float display).  
- Both written to the same MTR zone buffer and pushed as a single SPI block.

---

## 6. SWR Scan Usage

**Entry:** Menu → "SWR Scan" (ACTION item, triggered by encoder click or F3).  
**Blocking:** occupies Y=62..239, suspends normal display, DSP audio continues via DMA interrupts.

**Scan parameters:**

| Parameter         | Value                              |
|-------------------|------------------------------------|
| Span              | ±200 kHz around current VFO freq  |
| Step              | 10 kHz                             |
| Max points        | 201                                |
| TX settle time    | 30 ms per step                     |
| SWR floor         | 1.00 (hardware minimum clamped)    |

**Sweep sequence per step:**
1. Set BPF + LPF to matching band for that frequency.  
2. `scan_tx_on()` → CLK0 (shared LO) reprogrammed to step frequency × 4, T/R switch asserted.  
3. Wait 30 ms settle, call `Analog_Update()` twice.  
4. Record `g_analog.swr_x100`.  
5. `scan_tx_off()` → T/R switch released, CLK0 restored to `centre_hz + lo_offset_hz`.  
6. Incremental plot update.

**Plot layout (within scan zone):**

```
Y= 62  Title bar (dark blue)  "SWR SCAN  7.100 MHz  +/−200 kHz"
Y= 76  Plot area top  (SWR 5.0 = top, SWR 1.0 = bottom)
Y=210  Plot area bottom
Y=210  Frequency axis (start / centre / end MHz)
Y=220  Result/hint bar ("MIN 1.35 @ 7.090 MHz  [F4=EXIT]")
```

**Y-axis mapping:** `y = (SC_PLT_Y2 − 1) − norm × (SC_PLT_H − 1)` where `norm = (swr − 1.0) / 4.0`. Higher SWR plots higher on screen.  
**Colour coding:** green <1.5 · yellow 1.5–2.0 · orange 2.0–3.0 · red ≥3.0 · cyan = minimum marker.  
**Fill:** quarter-brightness fill drawn below each peak pixel toward the bottom baseline.  
**Progress bar:** right side of hint bar during sweep.  
**Abort:** hold F4 during sweep. After abort, partial results are displayed.  
**Exit:** press F4 after scan complete (debounced, requires release then fresh press).  
**Restore:** BPF/LPF reset to `g_sdr.band_idx`; QSD restored to `freq_hz + lo_offset_hz`; `g_sdr.display_dirty = true`.

---

## 7. DSP Signal Flow Summary

**Sample rate:** 48 kHz. **Block size:** 256 samples (ping/pong DMA, two 256-sample halves = 512-word DMA buffer each direction). **SAI format:** 16-bit data right-justified in 32-bit slots (RX read: `(int16_t)(uint16_t)word`; TX write: right-justified `(int32_t)(int16_t)sample`).

### RX Pipeline (per sample)

```
SAI DMA int32 I/Q
  ↓  read (int16_t)(uint16_t) → float × 1/32767
Pre-mix IIR DC block ×2            H(z) = (1−z⁻¹)/(1−0.995z⁻¹)  removes ADC offset
  ↓
NCO down-mix                       32-bit phase acc, 1024-entry sin LUT, supports negative freq
  ↓
Post-mix IIR DC block ×2           removes residual LO leakage → suppresses centre spike
  ↓
IQ mismatch correction             Q = (Q − iq_p × I) × iq_g_inv   (Gram-Schmidt)
  ↓
FFT accumulate                     256 samples, Hann window → FFT_Radix2 → magnitude (power)
  ↓
FIR LPF I and Q                    64-tap Hann-windowed sinc, cutoff = bw_hz / Fs
  ↓
Demodulate
  AM   → sqrt(I²+Q²)
  FM   → atan2 differentiator + 75µs de-emphasis IIR
  USB  → (I+Q)/2
  LSB  → (I−Q)/2
  CW   → AM envelope × 700 Hz BFO (32-bit NCO)
  ↓
Audio FIR LPF                      32-tap, 4 kHz cutoff
  ↓
Audio IIR DC block                 removes AM envelope DC bias (AC-coupled outputs)
  ↓
AGC                                peak-hold + hang; attack 1 ms always
                                   slow: 1.5 s decay / 500 ms hang
                                   fast: 300 ms decay / 100 ms hang
  ↓
Scale → int16, stereo L=R → SAI TX DMA
```

**Signal power for S-meter:** post-AGC RMS, exponential smoothing α=0.9/0.1.

### TX Pipeline (per sample)

```
USB Audio OUT ring (PC→radio, int16 LE stereo, L channel used)
  ↓
Audio IIR DC block                 removes mic/line DC
  ↓
TX gain (audio_gain, 0..1)
  ↓
Audio FIR LPF 32-tap 4 kHz        applied BEFORE modulation (real mono signal)
  ↓
Compressor 4:1 above 0.70          attack 1 ms / release 50 ms
Soft limiter C1-smooth at 0.95     y = 1 − 0.0025/(|x|−0.90) for |x|>0.95
  ↓
Modulate
  USB/LSB → Hilbert phasing method: 63-tap Hamming Hilbert (Q), matched delay (I)
  AM      → 0.5 + 0.5×audio  (carrier + DSB; no carrier suppression)
  FM/CW   → audio × 0.7 on I only  ← placeholder only; not true FM modulation
             (no phase integrator, no deviation control, no pre-emphasis)
  ↓
FFT accumulate (TX spectrum display)
  ↓
Scale → int16 right-justified → SAI TX DMA → WM8731 DAC → QSE mixer
```

**DC spike note:** The post-mix DC blocks are intentional. Without them, LO leakage from the QSD mixer appears as a large spike at the spectrum centre bin. The FFT feed is placed *after* these blockers, so the centre bin is clean in the display. This does not affect demodulation; it removes a visual artefact only.

---

## 8. LO Architecture Summary

### Si5351A – single 4× clock, external 74LVC74 shared quadrature

**Hardware topology:**

```
25 MHz XTAL
    │
Si5351A
    └─ PLL_A  VCO 600–900 MHz
          └─ CLK0  →  4 × RF frequency  (shared RX + TX LO)
                           │
                   74LVC74 dual D flip-flop
                   ÷4 Johnson counter
                      ├─ I (0°)  ──┬── QSD I port  (RX)
                      │            └── QSE I port  (TX)
                      └─ Q (90°) ──┬── QSD Q port  (RX)
                                   └── QSE Q port  (TX)

CLK1: powered down (unused)
CLK2: powered down (unused)
PLL_B: unused
```

RX and TX mixers are fed from the **same** 74LVC74 quadrature output. There is no separate TX LO; the Si5351 has a single active output (CLK0) at all times.

**Quadrature method:** The 74LVC74 dual D flip-flop is wired as a Johnson (divide-by-4) counter. This produces two outputs with a precise 90° phase relationship from a single 4× clock. No Si5351 phase-offset registers are used for quadrature; this avoids the even-MS\_div constraint and gives better I/Q phase accuracy than the register method.

**Phase coherence:** RX and TX share the same LO source. In the current single-VFO implementation, switching between RX and TX only requires toggling the T/R relay — the LO frequency does not change. (A future split-frequency or independent RX/TX VFO mode would require reprogramming CLK0 on mode transitions.)

**Programming API:**

```c
SI5351_SetQSDFrequency(&g_si5351, rf_hz);  // CLK0 = rf_hz × 4, PLL_A (shared LO)
// No separate TX setter — CLK0 serves both QSD and QSE.
```

Callers always pass the RF frequency; the ×4 factor is inside the driver.

**LO offset (`lo_offset_hz`, default 18 kHz):** The Si5351 is programmed to `freq_hz + lo_offset_hz`. The NCO then digitally shifts baseband by the same offset in the opposite direction, placing the desired signal exactly at DC after mixing. This keeps the QSD/QSE operating at a slight offset from the tuned frequency, which avoids self-reception of the LO at the antenna.

**NCO offset calculation:**
```
NCO freq = freq_hz − (if_hz + lo_offset_hz)
```
where `if_hz = freq_hz + if_shift_hz` (RIT/IF-shift adjusted). With no RIT and no IF shift: NCO freq = −lo\_offset\_hz, which digitally shifts the 18 kHz LO offset back to 0 Hz.

**Constraints:**
- VCO range: 600–900 MHz (PLL_A only; PLL_B unused).
- CLK0 output range: ~100 kHz to ~150 MHz (MS integer divider 6–2047, R divider 1..128).
- XTAL: 25 MHz on-board (`SI5351_XTAL_HZ = 25000000` in `csdr_app.h`; note `SI5351_XTAL_FREQ_HZ` in `si5351.h` defaults to 27 MHz but is overridden at init).
- PLL reset is issued after VCO reprogramming to guarantee phase lock.
