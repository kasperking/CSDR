/* USER CODE BEGIN Header */
/**
  * @file  pa_protect.c
  * @brief Centralized PA Protection Manager.
  *        See pa_protect.h for architecture and integration notes.
  *
  *  Sensor pipeline:
  *    INA226 current (PA_OC_ReadCurrent)
  *    NTC temperature (g_analog.temp_c, °C×10)
  *    SWR bridge      (g_analog.swr_x100, SWR×100)
  *    → IIR filter → threshold comparison → state machine → drive limit
  *
  *  Foldback steps (k_drive_pct[]):
  *    step 0 → 100%  (NORMAL)
  *    step 1 →  75%  (FOLDBACK entry)
  *    step 2 →  50%  (FOLDBACK sustained)
  *    step 3 →  25%  (LIMIT)
  *    TRIP/COOLDOWN → 0% drive, TX disabled
  *
  *  Timing:
  *    Called every 20 ms from CSDR_Loop.
  *    IIR τ: SWR/temp ≈200 ms (α=0.1), current ≈67 ms (α=0.3).
  *    100 ms TX-blanking window suppresses relay-click / ramp transients.
  *    Foldback ramps down every 1000 ms, recovers every 2000 ms.
  */
/* USER CODE END Header */

#include "pa_protect.h"
#include "pa_overcurrent.h"   /* PA_OC_ReadCurrent(), g_pa_oc.ina_ok           */
#include "fsdr_analog.h"      /* g_analog: swr_x100 (SWR×100), temp_c (°C×10) */
#include "csdr_app.h"         /* g_sdr: tx_mode, cat_tx_dirty, display_dirty   */
#include "hw_fault.h"         /* HW_Fault_PASensorMissing()                    */

/* ─── External ALC constants (fixed, professional-radio style) ──────────── */
/* PC1 / ADC2_INP11: external PA feedback voltage, 0-100% of ADC range.
 * Attack τ ≈ 28 ms  (α=0.5,  one 20 ms tick).
 * Release τ ≈ 490 ms (α=0.04, one 20 ms tick).
 * Reduction is proportional: threshold..100% input → 100..min_drive output. */
#define ALC_THRESHOLD_PCT  70U    /* above 70% input → begin reducing drive   */
#define ALC_MIN_DRIVE_PCT  30U    /* maximum reduction floor (30% drive left)  */
#define ALC_ALPHA_ATK      0.5f   /* fast attack                               */
#define ALC_ALPHA_REL      0.04f  /* slow release                              */

/* ─── Power ALC constants (tandem-match closed-loop leveling) ───────────── */
/* Forward power (g_analog.fwd_power_mw) vs target derived from pa_watts ×
 * tx_power% — the same watt figure shown in the RF Power menu.  Peak-hold
 * envelope (same attack/release profile as external ALC) feeds a slow
 * proportional corrector that trims drive to hit the target, compensating
 * band-to-band PA gain variation and supply-voltage sag.  Always active
 * when pa_watts>0 — unlike external ALC this uses hardware already
 * required for SWR protection, so there is no separate enable flag. */
#define PWR_ALC_DEADBAND      0.05f  /* ±5% of target before correcting    */
#define PWR_ALC_GAIN          8.0f   /* %-point step = err × GAIN           */
#define PWR_ALC_STEP_MAX_PCT  1.0f   /* max %-point change per 20 ms tick  */
#define PWR_ALC_MIN_PCT       50U    /* corrector floor (can't silence TX) */
#define PWR_ALC_MAX_PCT       150U   /* corrector ceiling (compensate sag) */
#define PWR_ALC_CARRIER_MW    50U    /* below this = SSB silence, hold     */

/* ─── Default threshold configuration ───────────────────────────────────── */

PA_Protect_Config_t g_pa_cfg = {
    .swr_warn         = 2.0f,
    .swr_trip         = 4.0f,
    .temp_warn_c10    = 750,     /* 75.0°C */
    .temp_trip_c10    = 900,     /* 90.0°C */
    .temp_recover_c10 = 700,     /* 70.0°C */
    .current_warn_a   = 7.5f,   /* 75% of default OC limit (10A, idx=3) */
    .current_trip_a   = 9.0f,   /* 90% of default OC limit (10A, idx=3) */
    .cooldown_ms      = 30000U,  /* 30 s absolute timeout */
};

/* ─── Drive limit lookup: foldback_step → % ─────────────────────────────── */

static const uint8_t k_drive_pct[4] = { 100U, 75U, 50U, 25U };

/* ─── Internal state ─────────────────────────────────────────────────────── */

static PA_State_t s_state         = PA_STATE_NORMAL;
static PA_Fault_t s_fault         = PA_FAULT_NONE;
static uint8_t    s_drive_pct     = 100U;
static uint8_t    s_foldback_step = 0U;   /* 0=100%  1=75%  2=50%  3=25%  */
static uint32_t   s_step_ms       = 0U;   /* timestamp of last foldback step */
static uint32_t   s_cooldown_ms   = 0U;   /* HAL_GetTick() at trip event    */
static uint32_t   s_tx_start_ms   = 0U;   /* HAL_GetTick() at TX start      */

/* IIR-filtered sensor values — updated on every PA_Protect_Update() tick */
static float s_filt_swr  = 100.0f;   /* swr_x100 units; 100 = SWR 1.00    */
static float s_filt_temp = 200.0f;   /* temp_c (°C×10); 200 = 20.0°C      */
static float s_filt_curr = 0.0f;     /* amperes                            */

/* External ALC state */
static float   s_alc_env       = 0.0f;   /* fast-attack / slow-release envelope (%) */
static uint8_t s_alc_drive_pct = 100U;   /* computed continuous drive multiplier     */

/* Power ALC state (tandem-match closed loop) */
static float   s_pwr_env       = 0.0f;   /* peak-held fwd power envelope (mW)       */
static float   s_pwr_corr_pct  = 100.0f; /* continuous drive corrector (%)          */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

static void update_filters(void)
{
    float new_swr  = (float)g_analog.swr_x100;
    float new_temp = (float)g_analog.temp_c;
    float new_curr = PA_OC_ReadCurrent();

    /* Cap open-antenna / no-load SWR spikes (swr_x100 = 9999 when Vf ≈ 0).
     * SWR 20:1 (2000) is well above any trip threshold; no real-world antenna
     * appears worse to the protection logic than this. */
    if (new_swr > 2000.0f) new_swr = 2000.0f;

    /* SWR and temperature: slow IIR (α=0.1) — τ ≈ 200 ms at 20 ms tick.
     * Suppresses SSB peaks, ADC noise, and relay-switching transients. */
    s_filt_swr  = s_filt_swr  * 0.9f + new_swr  * 0.1f;
    s_filt_temp = s_filt_temp * 0.9f + new_temp  * 0.1f;

    /* Current: faster IIR (α=0.3) — τ ≈ 67 ms — for quicker soft foldback.
     * Hard overcurrent is handled independently by the INA226 hardware alert
     * in pa_overcurrent.c (< 140 µs response), so this filter can afford
     * to be slightly slower while still catching sustained overload. */
    s_filt_curr = s_filt_curr * 0.7f + new_curr  * 0.3f;

    /* External ALC: fast-attack / slow-release envelope detector.
     * Runs unconditionally so the envelope is accurate when TX starts.
     * When ext_alc_on=false the envelope slowly decays to zero. */
    {
        float new_alc = g_sdr.ext_alc_on ? (float)g_analog.alc_percent : 0.0f;
        float alpha   = (new_alc > s_alc_env) ? ALC_ALPHA_ATK : ALC_ALPHA_REL;
        s_alc_env     = s_alc_env + alpha * (new_alc - s_alc_env);
    }
}

/* Re-apply audio gain without toggling the T/R relay.
 * Sets cat_tx_dirty → csdr_apply_tx() picks it up within the 10 ms CAT tick. */
static void request_gain_reapply(void)
{
    if (g_sdr.tx_mode) g_sdr.cat_tx_dirty = true;
}

/* Hard trip: zero drive immediately; TX UI stays up — relay stays in TX
 * position (no RF since gain=0), warning appears in INFO zone. */
static void do_trip(PA_Fault_t fault)
{
    s_fault         = fault;
    s_drive_pct     = 0U;
    s_foldback_step = 0U;
    s_cooldown_ms   = HAL_GetTick();
    s_state         = PA_STATE_TRIP;

    /* Trigger immediate gain recompute (drive_pct=0 → audio_gain=0 → silent). */
    request_gain_reapply();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void PA_Protect_Init(void)
{
    s_state         = PA_STATE_NORMAL;
    s_fault         = PA_FAULT_NONE;
    s_drive_pct     = 100U;
    s_foldback_step = 0U;
    s_step_ms       = 0U;
    s_cooldown_ms   = 0U;
    s_tx_start_ms   = 0U;
    s_pwr_env       = 0.0f;
    s_pwr_corr_pct  = 100.0f;
}

void PA_Protect_OnTxStart(void)
{
    s_tx_start_ms = HAL_GetTick();

    /* Seed IIR filters with pre-TX readings so the state machine starts from
     * a known-good baseline rather than the cold-boot defaults.  This prevents
     * the 100 ms blanking window from hiding a genuinely bad condition that was
     * already present before TX started. */
    s_filt_swr  = (float)g_analog.swr_x100;
    s_filt_temp = (float)g_analog.temp_c;
    s_filt_curr = PA_OC_ReadCurrent();

    /* Clamp seeded SWR (RX measurement may legitimately be 9999) */
    if (s_filt_swr > 2000.0f) s_filt_swr = 2000.0f;

    /* Seed ALC envelope from current PA feedback so there is no step-change
     * in drive at TX onset when ALC conditions are already non-zero. */
    s_alc_env = (float)g_analog.alc_percent;

    /* Power ALC: fresh start each transmission — avoids carrying a stale
     * correction from a different band/frequency into the new TX. */
    s_pwr_env      = 0.0f;
    s_pwr_corr_pct = 100.0f;
}

void PA_Protect_OnTxStop(void)
{
    /* Voluntary TX→RX: reset foldback so the next TX starts at full power.
     * The foldback steps are accumulated per-transmission; clearing here gives
     * the user a fresh start if they release PTT and try again after conditions
     * have improved. */
    if (s_state == PA_STATE_FOLDBACK || s_state == PA_STATE_LIMIT) {
        s_foldback_step = 0U;
        s_drive_pct     = 100U;
        s_state         = PA_STATE_NORMAL;
    }
    /* TRIP / COOLDOWN: no state change here — cooldown timer and sensor
     * values govern recovery.  (Called only on a real TX→RX transition;
     * mid-TX gain reapplies no longer reach OnTxStart/Stop.) */
}

void PA_Protect_ManualReset(void)
{
    if (s_state == PA_STATE_COOLDOWN) {
        s_state         = PA_STATE_NORMAL;
        s_fault         = PA_FAULT_NONE;
        s_drive_pct     = 100U;
        s_foldback_step = 0U;
        request_gain_reapply();   /* restore drive if PTT still held */
        g_sdr.display_dirty = 0xFFU;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PA_Protect_Update — call every 20 ms from CSDR_Loop
 * ═══════════════════════════════════════════════════════════════════════════ */

void PA_Protect_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* Filters run unconditionally — keeps thermal/SWR state warm during RX
     * so values are accurate the instant TX starts. */
    update_filters();

    /* ── External ALC: compute continuous drive multiplier ──────────────── */
    /* Runs regardless of TX/RX state so the gain is correct the instant
     * csdr_apply_tx() is called after a TX start. */
    {
        uint8_t new_alc_drive;
        if (g_sdr.ext_alc_on && s_alc_env >= (float)ALC_THRESHOLD_PCT) {
            float range    = (float)(100U - ALC_THRESHOLD_PCT);
            float excess   = (s_alc_env - (float)ALC_THRESHOLD_PCT) / range;
            if (excess > 1.0f) excess = 1.0f;
            float drive    = 100.0f - excess * (float)(100U - ALC_MIN_DRIVE_PCT);
            new_alc_drive  = (uint8_t)drive;
        } else {
            new_alc_drive = 100U;
        }
        if (new_alc_drive != s_alc_drive_pct) {
            s_alc_drive_pct = new_alc_drive;
            request_gain_reapply();   /* triggers csdr_apply_tx() gain recompute */
        }
    }

    /* ── COOLDOWN: checked regardless of TX state ───────────────────────── */
    if (s_state == PA_STATE_COOLDOWN) {
        uint32_t elapsed = now - s_cooldown_ms;
        bool     timeout = (elapsed >= g_pa_cfg.cooldown_ms);
        bool     recover = timeout;

        if (!recover) {
            if (s_fault == PA_FAULT_OVERTEMP) {
                /* Temperature fault: require temp to drop below hysteresis
                 * threshold AND a minimum dwell time to prevent oscillation. */
                bool temp_ok  = (s_filt_temp < (float)g_pa_cfg.temp_recover_c10);
                bool min_wait = (elapsed >= 5000U);
                recover = temp_ok && min_wait;
            } else {
                /* OC / SWR fault: recover when sensor values drop below 90% of
                 * the warn threshold AND a 3 s minimum has elapsed. */
                bool oc_ok  = (s_filt_curr < g_pa_cfg.current_warn_a * 0.9f);
                bool swr_ok = (s_filt_swr  < g_pa_cfg.swr_warn * 90.0f);
                recover = oc_ok && swr_ok && (elapsed >= 3000U);
            }
        }

        if (recover) {
            s_state         = PA_STATE_NORMAL;
            s_fault         = PA_FAULT_NONE;
            s_drive_pct     = 100U;
            s_foldback_step = 0U;
            request_gain_reapply();   /* resume TX drive if PTT still held */
            g_sdr.display_dirty = 0xFFU;
        }
        return;
    }

    /* ── TRIP: transient state (~20 ms) — immediately advance to COOLDOWN ─ */
    if (s_state == PA_STATE_TRIP) {
        s_state = PA_STATE_COOLDOWN;
        return;
    }

    /* ── Remaining states (NORMAL / FOLDBACK / LIMIT) apply only during TX ─ */
    if (!g_sdr.tx_mode) return;

    /* 100 ms blanking window after TX start: ignore protection while relay
     * contacts settle, PA bias ramps, and SWR bridge stabilizes.
     * Filters still run above, so state is current when blanking expires. */
    if ((now - s_tx_start_ms) < 100U) return;

    /* ── Power ALC: closed-loop drive correction from tandem-match fwd power ─
     * Target = pa_watts × tx_power% (the same watt figure shown in the RF
     * Power menu).  Peak-hold envelope (fast attack / slow release, same
     * profile as external ALC) tracks SSB syllabic peaks; correction only
     * adjusts while a carrier is actually present so it holds steady
     * through speech pauses instead of drifting toward max. */
    if (g_sdr.pa_watts > 0U) {
        uint32_t target_mw = (uint32_t)g_sdr.pa_watts * 1000U
                              * (uint32_t)g_sdr.tx_power / 100U;
        if (target_mw > 0U) {
            float meas  = (float)g_analog.fwd_power_mw;
            float alpha = (meas > s_pwr_env) ? ALC_ALPHA_ATK : ALC_ALPHA_REL;
            s_pwr_env   = s_pwr_env + alpha * (meas - s_pwr_env);

            if (g_analog.fwd_power_mw > PWR_ALC_CARRIER_MW) {
                float err = ((float)target_mw - s_pwr_env) / (float)target_mw;
                if (err >  1.0f) err =  1.0f;
                if (err < -1.0f) err = -1.0f;
                float abs_err = (err < 0.0f) ? -err : err;
                if (abs_err > PWR_ALC_DEADBAND) {
                    float step = err * PWR_ALC_GAIN;
                    /* External-ALC precedence: while the external amp commands
                     * a drive cut (s_alc_drive_pct < 100), low forward power is
                     * intentional — boosting to "correct" it would fight the
                     * amp's ALC (raising the effective drive floor from 30 % to
                     * 30 %×150 % = 45 %).  Upward steps are blocked, the
                     * corrector holds; downward steps stay allowed since both
                     * loops then agree on reducing drive. */
                    if (s_alc_drive_pct < 100U && step > 0.0f) step = 0.0f;
                    if (step >  PWR_ALC_STEP_MAX_PCT) step =  PWR_ALC_STEP_MAX_PCT;
                    if (step < -PWR_ALC_STEP_MAX_PCT) step = -PWR_ALC_STEP_MAX_PCT;
                    float new_corr = s_pwr_corr_pct + step;
                    if (new_corr < (float)PWR_ALC_MIN_PCT) new_corr = (float)PWR_ALC_MIN_PCT;
                    if (new_corr > (float)PWR_ALC_MAX_PCT) new_corr = (float)PWR_ALC_MAX_PCT;
                    if ((uint8_t)new_corr != (uint8_t)s_pwr_corr_pct) request_gain_reapply();
                    s_pwr_corr_pct = new_corr;
                }
            }
        }
    }

    /* ── Evaluate threshold conditions (priority: OC > temp > SWR) ──────── */

    bool oc_warn  = (s_filt_curr >= g_pa_cfg.current_warn_a);
    bool oc_trip  = (s_filt_curr >= g_pa_cfg.current_trip_a);
    bool tmp_warn = (s_filt_temp >= (float)g_pa_cfg.temp_warn_c10);
    bool tmp_trip = (s_filt_temp >= (float)g_pa_cfg.temp_trip_c10);
    /* SWR mismatch is the expected, intentional condition while TUNE is
     * active (the whole point is adjusting the ATU into a bad match) — so
     * SWR alone must not foldback/trip during TUNE.  OC and thermal limits
     * stay fully active since those are real hardware limits regardless of
     * match quality.  g_sdr.tune_mode is set by the dedicated TUNE button
     * (csdr_tune_start/csdr_tune_stop in csdr_app.c). */
    bool swr_warn = !g_sdr.tune_mode && (s_filt_swr >= g_pa_cfg.swr_warn * 100.0f);
    bool swr_trip = !g_sdr.tune_mode && (s_filt_swr >= g_pa_cfg.swr_trip * 100.0f);

    bool any_trip = oc_trip || tmp_trip || swr_trip;
    bool any_warn = oc_warn || tmp_warn || swr_warn;

    /* ── State machine ───────────────────────────────────────────────────── */

    switch (s_state) {

        /* ── NORMAL: full power, monitoring ────────────────────────────── */
        case PA_STATE_NORMAL:
            if (any_trip) {
                PA_Fault_t f = oc_trip  ? PA_FAULT_OVERCURRENT
                             : tmp_trip ? PA_FAULT_OVERTEMP
                             :            PA_FAULT_HIGH_SWR;
                do_trip(f);
            } else if (any_warn) {
                s_state         = PA_STATE_FOLDBACK;
                s_foldback_step = 1U;                  /* immediately 75%   */
                s_drive_pct     = k_drive_pct[1];
                s_step_ms       = now;
                request_gain_reapply();
            }
            break;

        /* ── FOLDBACK / LIMIT: graduated power reduction ─────────────── */
        case PA_STATE_FOLDBACK:  /* intentional fall-through */
        case PA_STATE_LIMIT:
            if (any_trip) {
                PA_Fault_t f = oc_trip  ? PA_FAULT_OVERCURRENT
                             : tmp_trip ? PA_FAULT_OVERTEMP
                             :            PA_FAULT_HIGH_SWR;
                do_trip(f);
            } else if (any_warn) {
                /* Ramp drive down: one step every 1000 ms while warn persists.
                 * Prevents rapid oscillation; gives PA time to respond. */
                if ((now - s_step_ms) >= 1000U) {
                    s_step_ms = now;
                    if (s_foldback_step < 3U) {
                        s_foldback_step++;
                        s_drive_pct = k_drive_pct[s_foldback_step];
                        request_gain_reapply();
                    }
                    s_state = (s_foldback_step >= 3U) ? PA_STATE_LIMIT
                                                       : PA_STATE_FOLDBACK;
                }
            } else {
                /* All warn conditions cleared — ramp back up.
                 * 2000 ms step-up rate is slower than step-down (1000 ms) to
                 * avoid oscillation when conditions hover near the threshold. */
                if ((now - s_step_ms) >= 2000U) {
                    s_step_ms = now;
                    if (s_foldback_step > 0U) {
                        s_foldback_step--;
                        s_drive_pct = k_drive_pct[s_foldback_step];
                        s_state     = PA_STATE_FOLDBACK;
                        request_gain_reapply();
                    } else {
                        s_state     = PA_STATE_NORMAL;
                        s_drive_pct = 100U;
                        request_gain_reapply();
                    }
                }
            }
            break;

        default:
            break;
    }
}

/* ─── Accessors ─────────────────────────────────────────────────────────── */

PA_State_t PA_Protect_GetState(void)      { return s_state;         }
PA_Fault_t PA_Protect_GetFault(void)      { return s_fault;         }
uint8_t    PA_Protect_GetDriveLimit(void) { return HW_Fault_PASensorMissing() ? 0U : s_drive_pct; }
uint8_t    PA_Protect_GetALCDrive(void)   { return s_alc_drive_pct; }
uint8_t    PA_Protect_GetPowerALCDrive(void) { return (uint8_t)s_pwr_corr_pct; }
uint32_t   PA_Protect_GetFwdPowerEnvelope_mW(void) { return (g_sdr.pa_watts > 0U && g_sdr.tx_mode) ? (uint32_t)s_pwr_env : 0U; }

uint16_t PA_Protect_GetSwrX100(void)
{
    /* Meaningful only while transmitting with a fitted PA + working sensor;
     * otherwise report a flat 1.00 so meters rest at zero deflection. */
    if (g_sdr.pa_watts == 0U || !g_sdr.tx_mode || HW_Fault_PASensorMissing()) {
        return 100U;
    }
    float s = s_filt_swr;
    if (s < 100.0f)  s = 100.0f;
    if (s > 2000.0f) s = 2000.0f;
    return (uint16_t)s;
}

uint8_t PA_Protect_GetAlcReductionPct(void)
{
    /* s_alc_drive_pct ∈ [ALC_MIN_DRIVE_PCT, 100]: 100 = ALC idle, 30 = full
     * reduction → reported range 0-70 %-points.  Zero when the external ALC
     * loop is disabled or not transmitting, so meters rest at no deflection. */
    if (!g_sdr.tx_mode || !g_sdr.ext_alc_on) return 0U;
    return (uint8_t)(100U - s_alc_drive_pct);
}

bool PA_Protect_IsTxAllowed(void)
{
    /* Block TX if any PA protection sensor is absent: operating without
     * overcurrent or thermal protection risks hardware damage. */
    if (HW_Fault_PASensorMissing()) return false;

    return s_state != PA_STATE_TRIP && s_state != PA_STATE_COOLDOWN;
}
