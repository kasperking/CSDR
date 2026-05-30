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
#include "pa_overcurrent.h"   /* PA_OC_ReadCurrent()                           */
#include "fsdr_analog.h"      /* g_analog: swr_x100 (SWR×100), temp_c (°C×10) */
#include "csdr_app.h"         /* g_sdr: tx_mode, cat_tx_dirty, display_dirty   */

/* ─── Default threshold configuration ───────────────────────────────────── */

PA_Protect_Config_t g_pa_cfg = {
    .swr_warn         = 2.0f,
    .swr_trip         = 4.0f,
    .temp_warn_c10    = 750,     /* 75.0°C */
    .temp_trip_c10    = 900,     /* 90.0°C */
    .temp_recover_c10 = 700,     /* 70.0°C */
    .current_warn_a   = 3.0f,
    .current_trip_a   = 4.0f,
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
}

/* Re-apply audio gain without toggling the T/R relay.
 * Sets cat_tx_dirty → csdr_apply_tx() picks it up within the 10 ms CAT tick. */
static void request_gain_reapply(void)
{
    if (g_sdr.tx_mode) g_sdr.cat_tx_dirty = true;
}

/* Hard trip: disable TX immediately via deferred relay path, store fault. */
static void do_trip(PA_Fault_t fault)
{
    s_fault         = fault;
    s_drive_pct     = 0U;
    s_foldback_step = 0U;
    s_cooldown_ms   = HAL_GetTick();
    s_state         = PA_STATE_TRIP;

    /* Force TX off via the existing deferred-apply mechanism.
     * csdr_apply_tx() will open the T/R relay and unmute the codec. */
    g_sdr.tx_mode       = false;
    g_sdr.cat_tx_dirty  = true;
    g_sdr.display_dirty = 0xFFU;
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
    /* TRIP / COOLDOWN: OnTxStop is called by the guard path in csdr_apply_tx()
     * when a TX attempt is rejected.  No state change here — cooldown timer
     * and sensor values govern recovery. */
}

void PA_Protect_ManualReset(void)
{
    if (s_state == PA_STATE_COOLDOWN && !g_sdr.tx_mode) {
        s_state         = PA_STATE_NORMAL;
        s_fault         = PA_FAULT_NONE;
        s_drive_pct     = 100U;
        s_foldback_step = 0U;
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

    /* ── Evaluate threshold conditions (priority: OC > temp > SWR) ──────── */

    bool oc_warn  = (s_filt_curr >= g_pa_cfg.current_warn_a);
    bool oc_trip  = (s_filt_curr >= g_pa_cfg.current_trip_a);
    bool tmp_warn = (s_filt_temp >= (float)g_pa_cfg.temp_warn_c10);
    bool tmp_trip = (s_filt_temp >= (float)g_pa_cfg.temp_trip_c10);
    bool swr_warn = (s_filt_swr  >= g_pa_cfg.swr_warn * 100.0f);
    bool swr_trip = (s_filt_swr  >= g_pa_cfg.swr_trip * 100.0f);

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

PA_State_t PA_Protect_GetState(void)      { return s_state;     }
PA_Fault_t PA_Protect_GetFault(void)      { return s_fault;     }
uint8_t    PA_Protect_GetDriveLimit(void) { return s_drive_pct; }

bool PA_Protect_IsTxAllowed(void)
{
    return s_state != PA_STATE_TRIP && s_state != PA_STATE_COOLDOWN;
}
