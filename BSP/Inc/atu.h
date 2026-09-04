/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    atu.h
  * @brief   Automatic Antenna Tuner — L-network, 16 relays via 2x 74AHC595
  *
  *  ── Why a private 595 chain (not the BPF one) ───────────────────────────
  *  The BPF 74AHC595 (bpf_lpf.c) has 2 spare bits and a free QH' cascade
  *  output, but its PCB neighbourhood is congested — daisy-chaining from it
  *  would force new traces through the busiest part of the board.  The ATU
  *  therefore carries its OWN 2-chip 595 chain on the ATU board itself, fed
  *  by 4 dedicated MCU pins.  BPF timing/word-building is untouched: the two
  *  chains share nothing but the bit-bang idiom.
  *
  *  ── Pins (reserved in CSDR.ioc as GPIO_Output + Locked, so CubeMX will
  *     not reassign them; ATU_Init() ALSO configures them itself, which is
  *     what makes the driver work today without a regen) ─────────────────
  *    PD2  ATU_SER    – 595 serial data in (first chip in the chain)
  *    PD3  ATU_SRCLK  – 595 shift clock, shared by both chips
  *    PD7  ATU_RCLK   – 595 storage-register latch, shared (pulse HIGH->LOW)
  *    PB3  ATU_OE     – 595 output-enable, active-LOW, shared
  *
  *  ATU_OE requires a 10k pull-up to 3V3 ON THE ATU BOARD.  It holds the
  *  595 outputs in Hi-Z from power-on (and whenever the control cable is
  *  unplugged) until ATU_Init() shifts a known-good word in — see ATU_Init.
  *  PB3 carries PinState=GPIO_PIN_SET in the .ioc for the same reason: if
  *  MX_GPIO_Init ever starts configuring this pin (after a regen) it must
  *  park OE HIGH first, never drive it low onto an unknown 595 state.  This
  *  mirrors what PA6/BPF_OE already does.
  *  The ULN2803 relay-driver inputs need 10k pull-DOWNs so Hi-Z reads as
  *  "all relays released" rather than floating.
  *
  *  PB3 IS THE JTDO/TRACESWO PIN.  Using it as GPIO costs SWO trace, which
  *  this project does not use: SWV is off in CSDR.launch, no ITM code exists,
  *  and the .ioc declares only PA13/PA14 (2-wire SWD).  PB4/NJTRST was
  *  already repurposed the same way for DAC_CS.  Two consequences to know:
  *    - Do NOT enable SWV in the debugger while the ATU is fitted.  Trace
  *      output would drive OE with serial data and make the relays chatter,
  *      because DBGMCU TRACE_IOEN takes the pin back from GPIO at any time.
  *    - PB3 has NO internal pull at reset (unlike PB4/NJTRST, which has a
  *      pull-up).  The external 10k pull-up below is therefore load-bearing,
  *      not just belt-and-braces: it is the only thing holding OE HIGH
  *      between power-on and ATU_Init().
  *
  *  PA8, PA9 and PA10 are left free.
  *
  *  ── 595 chain bit map ───────────────────────────────────────────────────
  *  atu595_shift() sends bit15 first, so after 16 clocks the HIGH byte has
  *  travelled through chip 1 into chip 2 (the far chip) and the LOW byte
  *  sits in chip 1 (the near chip, wired directly to ATU_SER):
  *
  *    chip 1 (near)  QA..QG = bit0..bit6  = L1..L7  inductor relays
  *                   QH     = bit7        = C-side select relay
  *    chip 2 (far)   QA..QG = bit8..bit14 = C1..C7  capacitor relays
  *                   QH     = bit15       = BYPASS relay
  *
  *  L and C are 7-bit binary-weighted ladders (0..127 steps).  Nominal
  *  hardware values, matching the usual L-network ATU build:
  *    L1..L7 = 0.05 0.1 0.2 0.4 0.8 1.6 3.2 uH   -> 0 .. 6.35 uH
  *    C1..C7 =   10  20  40  80 160 320 640 pF   -> 0 .. 1270 pF
  *  The firmware only ever writes relay bits; the values above are
  *  documentation for the hardware and do not appear in any calculation.
  *
  *  C-SIDE relay (bit 7) picks which end of the coil the capacitor bank
  *  shunts, which is what makes the network reversible:
  *    0 = C on the ANTENNA side   -> matches HIGH-impedance loads (R > 50)
  *    1 = C on the TRANSCEIVER side -> matches LOW-impedance loads (R < 50)
  *  The shunt element always belongs on the higher-impedance end, hence the
  *  pairing above.  A full tune tries both positions and keeps the better,
  *  so the firmware never has to know which one a given antenna needs — but
  *  get the wiring the other way round on the board and the labels in the
  *  UI/debugger will read backwards, so match this table when routing.
  *
  *  BYPASS relay: 1 = the whole L/C network is shorted out of the RF path.
  *  Asserted at init and whenever the ATU is disabled, so a radio with no
  *  ATU board fitted (or a disabled one) behaves exactly as before.
  *
  *  ── Tune cycle ──────────────────────────────────────────────────────────
  *  Non-blocking state machine, one relay move + one SWR reading per
  *  ATU_Poll() tick (ATU_SETTLE_MS apart).  Never blocks the main loop, so
  *  audio, USB and CAT keep running throughout — a tune takes ~2.4 s but the
  *  radio stays responsive.  Search order, run once per C-side position:
  *     coarse 2-D grid over (L,C), 8x8 geometric      64 pts
  *     -> refine L +/-16 step 4 -> refine C +/-16 step 4    9 + 9
  *     -> refine L +/- 4 step 1 -> refine C +/- 4 step 1    9 + 9
  *  giving 1 + 2x100 = 201 measurements.  The coarse stage has to be 2-D and
  *  geometrically spaced — see the long note in atu.c, both properties were
  *  needed to stop the tuner failing on ordinary resistive mismatches.
  *  The best (lowest-SWR) L/C/Z triple across both halves wins; bypass also
  *  competes, so an antenna that is already flat tunes to "bypass" rather
  *  than inserting loss.
  *
  *  TX is keyed through CSDR_RequestTX with g_sdr.tune_mode set, which caps
  *  drive at TUNE_POWER_PCT (25%) and makes PA_Protect ignore SWR warn/trip
  *  (pa_protect.c) — a bad match is the expected condition while tuning.
  *  Overcurrent and thermal protection stay fully armed, and the tuner
  *  aborts the moment PA_Protect_IsTxAllowed() goes false.
  *
  *  ── Solution memory ─────────────────────────────────────────────────────
  *  Successful tunes are stored per 100 kHz bucket across 1.8-30 MHz in a
  *  dedicated flash sector (FLASH_ADDR_ATU_MEM), so retuning after a band
  *  change is instant and silent.  Memory recall never moves relays while
  *  transmitting — a mid-TX frequency change defers the relay write to the
  *  next RX (see ATU_OnFreqChange / ATU_Poll).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ATU_H
#define __ATU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "main.h"   /* authoritative GPIO pin defines: ATU_SER_Pin, ATU_OE_Pin, ... */
#include <stdint.h>
#include <stdbool.h>

/* Control GPIO lives in CSDR.ioc (PB3/PD2/PD3/PD7, GPIO_Output, Locked) and
 * therefore in main.h — no private copies here, so CubeMX cannot hand these
 * pins to a peripheral behind the driver s back. */

/* ─── 595 word bit positions (see header comment for the chain layout) ──── */
#define ATU_BIT_L_SHIFT       0U    /* bits 0..6  : L ladder, chip 1 QA..QG */
#define ATU_BIT_C_TX_SIDE     7U    /* bit 7: C-side relay, chip 1 QH;      
                                     1 = C bank on the transceiver side */
#define ATU_BIT_C_SHIFT       8U    /* bits 8..14 : C ladder, chip 2 QA..QG */
#define ATU_BIT_BYPASS       15U    /* bit 15     : bypass relay, chip 2 QH */
#define ATU_LC_MAX          127U    /* 7-bit ladder: 0..127                 */

/* ─── Tune cycle parameters ─────────────────────────────────────────────── */
#define ATU_SETTLE_MS         12U   /* relay armature settle before reading  */
#define ATU_TX_SETTLE_MS     120U   /* carrier + T/R + keying gate settle    */
#define ATU_TIMEOUT_MS     15000U   /* hard safety stop for the whole cycle  */
#define ATU_SWR_GOOD_X100    120U   /* <=1.20 -> accept immediately (bypass) */
#define ATU_SWR_OK_X100      200U   /* <=2.00 -> tune reported as successful */
#define ATU_MIN_FWD_RAW     1200U   /* raw ADC floor for a valid SWR reading;
                                       Analog_Calc_SWR_x100 returns 1.00 (a
                                       false "perfect match") below its own
                                       noise gate, so guard before trusting it */

/* ─── Tuner status ──────────────────────────────────────────────────────── */
typedef enum {
  ATU_STATUS_IDLE = 0,   /*!< not tuning                                    */
  ATU_STATUS_TUNING,     /*!< cycle in progress                             */
  ATU_STATUS_OK,         /*!< last tune reached ATU_SWR_OK_X100 or better   */
  ATU_STATUS_POOR,       /*!< best match found but still above OK threshold */
  ATU_STATUS_FAILED,     /*!< aborted: no RF, PA protection, or timeout     */
} Atu_Status_t;

/* ─── Live state (read-only for UI/CAT) ─────────────────────────────────── */
typedef struct {
  bool     enabled;      /*!< ATU hardware fitted + enabled (menu/EEPROM)   */
  bool     auto_tune;    /*!< auto-tune on band change when no memory hit   */
  bool     bypass;       /*!< network currently shorted out of the RF path  */
  uint8_t  l_val;        /*!< live inductor ladder value 0..127             */
  uint8_t  c_val;        /*!< live capacitor ladder value 0..127            */
  bool     c_tx_side;    /*!< live C-side relay: 1 = C on transceiver side  */
  uint8_t  status;       /*!< Atu_Status_t of the last/current cycle        */
  uint16_t swr_x100;     /*!< SWR after the last completed tune             */
  uint8_t  progress;     /*!< 0..100 % through the current cycle (UI)       */
} Atu_State_t;

extern Atu_State_t g_atu;

/* ─── API ───────────────────────────────────────────────────────────────── */

/**
  * @brief  Init the ATU control pins and park the relays safely.
  *         Drives ATU_OE HIGH (595 Hi-Z) BEFORE switching the pin to an
  *         output, shifts the bypass word in while still Hi-Z, then enables
  *         the outputs — the relays therefore go straight from "all
  *         released" to "bypass" with no intermediate garbage state.
  *         Call from CSDR_Init() after the settings blob has been loaded.
  */
void ATU_Init(void);

/**
  * @brief  Enable/disable the tuner.  Disabling forces bypass immediately.
  *         Safe to call at any time; ignored while a tune is running.
  */
void ATU_SetEnabled(bool on);

/** @brief  Set the auto-tune-on-band-change policy. */
void ATU_SetAuto(bool on);

/**
  * @brief  Force the network in (false) or out (true) of the RF path.
  *         Refused while transmitting — relays must not switch under RF
  *         outside a tune cycle; the request is applied at the next RX.
  */
void ATU_SetBypass(bool on);

/**
  * @brief  Start an automatic tune cycle at the current frequency.
  *         Keys TX itself (low power, tune_mode) and returns immediately —
  *         progress is driven by ATU_Poll().
  * @retval false if the ATU is disabled, already tuning, PA protection
  *         forbids TX, or no PA is fitted (pa_watts == 0).
  */
bool ATU_StartTune(void);

/** @brief  Abort a running cycle and drop TX; keeps the best word so far. */
void ATU_AbortTune(void);

/** @brief  true while a tune cycle is in progress. */
bool ATU_IsTuning(void);

/**
  * @brief  Cooperative tick — call every main-loop iteration from CSDR_Loop.
  *         No-op when idle.  Performs at most one relay move plus one ADC
  *         pair per ATU_SETTLE_MS; never blocks.
  */
void ATU_Poll(void);

/**
  * @brief  Collect the result of a finished cycle exactly once (edge-
  *         triggered, like PA_BiasCal_Poll).
  * @param  swr_out  optional: SWR x100 achieved
  * @retval 0 = nothing to report, 1 = success (OK/POOR), 2 = failed
  */
uint8_t ATU_PollResult(uint16_t *swr_out);

/**
  * @brief  Frequency/band changed — recall the stored solution for the new
  *         frequency, or bypass when none exists.  Deferred to the next RX
  *         if called while transmitting.  Never starts a tune by itself;
  *         auto-tune is armed here and fired from ATU_Poll() in RX.
  */
void ATU_OnFreqChange(uint32_t freq_hz);

/**
  * @brief  Load the solution memory from flash.  Call once in CSDR_Init()
  *         after W25Q_Init(); a blank/corrupt sector simply yields an empty
  *         memory (every bucket "no solution"), never an error to the user.
  */
void ATU_MemLoad(void);

/**
  * @brief  Flush the solution memory to flash if it changed.  Blocks for the
  *         sector erase (~45-400 ms), so csdr_app calls it from the debounced
  *         save path in RX only — never while transmitting or tuning.
  */
void ATU_MemFlush(void);

/** @brief  true when a tune result is waiting to be written to flash. */
bool ATU_MemDirty(void);

/** @brief  Forget every stored solution (menu action). */
void ATU_MemClear(void);

/** @brief  Short status text for the UI info line, e.g. "ATU 1.3". */
const char *ATU_StatusText(void);

#ifdef __cplusplus
}
#endif
#endif /* __ATU_H */
