/**
  * @file rtty_decode.c
  * @brief RTTY RX decoder — audio FSK demod + UART framer + Baudot/ITA2.
  *
  * Deliberately hardware-free (stdint/math only) so the whole decoder can
  * be compiled and regression-tested on a host against synthetic AFSK, the
  * same way the CW decoder was validated.
  */

#include "rtty_decode.h"
#include <math.h>
#include <string.h>

/* ── Tunables ─────────────────────────────────────────────────────────────── */
#define RTTY_EDGE_RING   32U      /* power of two; ~45 edges/s worst case →
                                     32 rides out a >500 ms main-loop stall */
#define RTTY_HYST_FRAC   0.125f   /* slicer hysteresis, fraction of the
                                     tracked mark↔space swing               */
#define RTTY_ENV_TC_X100 32U      /* envelope TC = 0.32/baud (7 ms @45.45)   */
#define RTTY_PK_TC_S     0.5f     /* ATC peak/valley decay time constant:
                                     slow against the 22 ms bit, fast enough
                                     to re-slice within ~3 chars of a deep
                                     selective-fade step                     */
#define RTTY_DBN_MS      2U       /* decision debounce (bit = 22 ms)         */

/* AFC: flanking-filter discriminator on the logical-mark tone (RTTY idles on
 * mark, so it gets the most air time).  Two BPFs at ±RTTY_AFC_FLANK_HZ around
 * the expected tone; their energy imbalance, integrated over solid-mark
 * samples, steps the filter centres toward the real tone pair. */
#define RTTY_AFC_FLANK_HZ  60.0f
#define RTTY_AFC_STEP_HZ   5.0f
#define RTTY_AFC_DEADBAND  0.05f  /* |err| below this = centred (≈ ±8 Hz)    */
#define RTTY_AFC_WIN_DIV   10U    /* evaluate per fs/10 gated samples (100ms) */

/* Baud/shift option tables — menu indices and EEPROM values map 1:1 */
const uint16_t g_rtty_baud_x100[RTTY_BAUD_COUNT] = { 4545U, 5000U, 7500U };
const uint16_t g_rtty_shift_hz[RTTY_SHIFT_COUNT] = { 170U, 425U, 850U };

/* ── Baudot / ITA2 (US-TTY figures) ──────────────────────────────────────────
 * Index = 5-bit code, LSB-first as received.  0 entries are non-printing
 * (blank) or handled specially (LF/CR/shifts).  BEL (FIGS-S) prints nothing. */
#define BAUDOT_SP    0x04U
#define BAUDOT_LF    0x02U
#define BAUDOT_CR    0x08U
#define BAUDOT_FIGS  0x1BU
#define BAUDOT_LTRS  0x1FU

static const char c_baudot_ltrs[32] = {
    0,  'E',  0,  'A', ' ', 'S', 'I', 'U',
    0,  'D', 'R', 'J', 'N', 'F', 'C', 'K',
   'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q',
   'O', 'B', 'G',  0,  'M', 'X', 'V',  0,
};
static const char c_baudot_figs[32] = {
    0,  '3',  0,  '-', ' ',  0,  '8', '7',
    0,  '$', '4', '\'', ',', '!', ':', '(',
   '5', '"', ')', '2', '#', '6', '0', '1',
   '9', '?', '&',  0,  '.', '/', ';',  0,
};

/* ── State ────────────────────────────────────────────────────────────────── */
bool g_rtty_tap_enable = false;

typedef struct {            /* transposed direct-form-II biquad */
    float b0, b2, a1, a2;   /* bandpass: b1 = 0                  */
    float z1, z2;
} rtty_bpf_t;

/* Configuration (set by RTTY_Init / RTTY_Configure) */
static float    s_fs;                 /* sample rate                        */
static uint16_t s_baud_x100;          /* 4545 = 45.45 Bd                    */
static uint16_t s_shift_hz;           /* mark→space shift                   */

/* Demod front-end (written per-sample by RTTY_FeedAudio) */
static rtty_bpf_t s_bpf_m, s_bpf_s;
static float    s_env_m, s_env_s;     /* tone envelopes                     */
static float    s_dp, s_dn;           /* ATC peak/valley of env_m − env_s   */
static float    s_a_env, s_pk_decay;  /* IIR coefficients                   */
static bool     s_rev;                /* swapped tone sense (USB/DIGU)      */

/* AFC (flanking discriminator around the logical-mark tone) */
static rtty_bpf_t s_bpf_afc_lo, s_bpf_afc_hi;
static float    s_afc_hz;             /* current correction, ±RTTY_AFC_RANGE */
static float    s_afc_lo_acc, s_afc_hi_acc;
static uint32_t s_afc_n, s_afc_win;   /* gated samples counted / window len */
static uint8_t  s_key;                /* current debounced state, 1 = mark  */
static uint16_t s_dbn_len, s_dbn_ctr;
static uint32_t s_clock;              /* sample counter (wraps ~24 h @48k)  */
static uint16_t s_guard;              /* framer look-behind, samples        */

/* Edge ring: producer = FeedAudio, consumer = Poll (both main-loop) */
static uint32_t s_edge_t[RTTY_EDGE_RING];
static uint8_t  s_edge_s[RTTY_EDGE_RING];
static uint8_t  s_edge_wr, s_edge_rd;

/* Framer (Poll context) */
typedef enum {
    FR_WAIT_MARK = 0,   /* after a framing error: need idle mark first */
    FR_WAIT_START,      /* line marking; next mark→space edge = start  */
    FR_CHAR,            /* sampling start/data/stop at mid-bit points  */
} rtty_fr_state_t;

static rtty_fr_state_t s_fr;
static uint8_t  s_line;               /* line state after consumed edges    */
static uint32_t s_t0;                 /* start-bit edge time                */
static uint32_t s_sample_rate;
static uint32_t s_spb;                /* samples per bit (48000/45.45)      */
static uint8_t  s_bit_idx;            /* 0=start, 1..5=data, 6=stop         */
static uint8_t  s_code;               /* data bits, LSB first               */
static bool     s_figs;               /* Baudot shift state                 */
static char     s_last_emit;          /* consecutive-space suppression      */

/* Text ring */
static char     s_text[RTTY_TEXT_LEN];
static uint8_t  s_head, s_count;

/* Debug counters (inspect in debugger) */
volatile uint32_t dbg_rtty_chars;     /* printable chars emitted            */
volatile uint32_t dbg_rtty_ferr;      /* framing errors (bad start/stop)    */
volatile uint32_t dbg_rtty_edges;     /* debounced mark/space transitions   */
volatile uint32_t dbg_rtty_edge_drop; /* edges lost to a full ring          */
volatile uint32_t dbg_rtty_afc_steps; /* AFC filter re-centres applied      */
volatile int32_t  dbg_rtty_afc_hz;    /* mirror of the AFC offset           */

/* ── Helpers ──────────────────────────────────────────────────────────────── */

/* RBJ constant-peak-gain bandpass */
static void rtty_bpf_init(rtty_bpf_t *f, float fc, float bw, float fs)
{
    float w0 = 6.2831853f * fc / fs;
    float al = sinf(w0) * bw / (2.0f * fc);   /* sin(w0)/(2Q), Q = fc/bw */
    float a0 = 1.0f + al;
    f->b0 = al / a0;
    f->b2 = -al / a0;
    f->a1 = -2.0f * cosf(w0) / a0;
    f->a2 = (1.0f - al) / a0;
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

static inline float rtty_bpf_run(rtty_bpf_t *f, float x)
{
    float y = f->b0 * x + f->z1;
    f->z1 = -f->a1 * y + f->z2;               /* b1 = 0 */
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

/* Retune in place: new coefficients, state carried over (the scaling error
 * decays within a few ms — no envelope dropout on AFC steps). */
static void rtty_bpf_retune(rtty_bpf_t *f, float fc, float bw, float fs)
{
    float z1 = f->z1, z2 = f->z2;
    rtty_bpf_init(f, fc, bw, fs);
    f->z1 = z1;
    f->z2 = z2;
}

/* Rebuild the four tone filters at the configured pair + AFC offset.  The
 * AFC flanks straddle the LOGICAL-mark tone (what RTTY idles on): 2125 Hz
 * normally, the space slot (2125+shift) when the sense is reversed. */
static void rtty_apply_tuning(void)
{
    float mark  = RTTY_MARK_HZ + s_afc_hz;
    float space = RTTY_MARK_HZ + (float)s_shift_hz + s_afc_hz;
    float lmark = s_rev ? space : mark;
    rtty_bpf_retune(&s_bpf_m, mark,  RTTY_TONE_BW_HZ, s_fs);
    rtty_bpf_retune(&s_bpf_s, space, RTTY_TONE_BW_HZ, s_fs);
    rtty_bpf_retune(&s_bpf_afc_lo, lmark - RTTY_AFC_FLANK_HZ, RTTY_TONE_BW_HZ, s_fs);
    rtty_bpf_retune(&s_bpf_afc_hi, lmark + RTTY_AFC_FLANK_HZ, RTTY_TONE_BW_HZ, s_fs);
}

/* Wrap-safe "a occurs at or before b" on the sample clock */
static inline bool time_le(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) <= 0;
}

static void rtty_push_edge(uint8_t new_state, uint32_t t)
{
    uint8_t nxt = (uint8_t)((s_edge_wr + 1U) & (RTTY_EDGE_RING - 1U));
    if (nxt != s_edge_rd) {
        s_edge_t[s_edge_wr] = t;
        s_edge_s[s_edge_wr] = new_state;
        s_edge_wr = nxt;
        dbg_rtty_edges++;
    } else {
        dbg_rtty_edge_drop++;
    }
    s_key = new_state;
}

static void rtty_emit(char c)
{
    /* Collapse runs of whitespace (CR LF CR LF …) into one space */
    if (c == ' ' && s_last_emit == ' ') return;
    s_text[s_head] = c;
    s_head = (uint8_t)((s_head + 1U) % RTTY_TEXT_LEN);
    if (s_count < (uint8_t)RTTY_TEXT_LEN) s_count++;
    s_last_emit = c;
}

/* Decode one 5-bit Baudot code through the shift state.
 * Returns true when a character was appended to the text ring. */
static bool rtty_emit_code(uint8_t code)
{
    code &= 0x1FU;
    if (code == BAUDOT_LTRS) { s_figs = false; return false; }
    if (code == BAUDOT_FIGS) { s_figs = true;  return false; }
    if (code == BAUDOT_CR)   { return false; }
    if (code == BAUDOT_LF)   { rtty_emit(' '); return true; }
    if (code == BAUDOT_SP)   {
        s_figs = false;                       /* USOS: space unshifts */
        rtty_emit(' ');
        return true;
    }
    char c = s_figs ? c_baudot_figs[code] : c_baudot_ltrs[code];
    if (c == 0) return false;                 /* blank / BEL / unused */
    rtty_emit(c);
    dbg_rtty_chars++;
    return true;
}

/* ── API ──────────────────────────────────────────────────────────────────── */

void RTTY_Init(uint32_t sample_rate)
{
    s_sample_rate = sample_rate;
    s_fs          = (float)sample_rate;
    s_pk_decay = expf(-1.0f / (RTTY_PK_TC_S * s_fs));
    s_dbn_len = (uint16_t)(RTTY_DBN_MS * sample_rate / 1000U);
    s_guard   = (uint16_t)(sample_rate / 1000U);   /* 1 ms on top of dbn */
    s_clock   = 0U;
    s_rev     = false;
    s_head    = 0U;
    s_count   = 0U;
    s_last_emit = 0;
    RTTY_Configure(g_rtty_baud_x100[0], g_rtty_shift_hz[0]);  /* → Reset */
}

void RTTY_Configure(uint16_t baud_x100, uint16_t shift_hz)
{
    if (baud_x100 < 2000U || baud_x100 > 10000U) baud_x100 = g_rtty_baud_x100[0];
    if (shift_hz  < 100U  || shift_hz  > 1000U)  shift_hz  = g_rtty_shift_hz[0];
    s_baud_x100 = baud_x100;
    s_shift_hz  = shift_hz;
    s_spb = (s_sample_rate * 100U + baud_x100 / 2U) / baud_x100;
    /* Envelope TC scales with the bit period (0.32/baud: 7 ms at 45.45 Bd,
     * 4.3 ms at 75 Bd) so faster rates keep the same smoothing-to-bit ratio */
    float tc = (float)RTTY_ENV_TC_X100 / (float)baud_x100;
    s_a_env  = 1.0f - expf(-1.0f / (tc * s_fs));
    s_afc_hz  = 0.0f;
    s_afc_win = s_sample_rate / RTTY_AFC_WIN_DIV;
    rtty_apply_tuning();
    RTTY_Reset();
}

int16_t RTTY_GetAfcHz(void)
{
    return (int16_t)(s_afc_hz >= 0.0f ? s_afc_hz + 0.5f : s_afc_hz - 0.5f);
}

void RTTY_Reset(void)
{
    s_bpf_m.z1 = s_bpf_m.z2 = 0.0f;
    s_bpf_s.z1 = s_bpf_s.z2 = 0.0f;
    s_bpf_afc_lo.z1 = s_bpf_afc_lo.z2 = 0.0f;
    s_bpf_afc_hi.z1 = s_bpf_afc_hi.z2 = 0.0f;
    s_afc_lo_acc = s_afc_hi_acc = 0.0f;   /* offset itself is preserved */
    s_afc_n = 0U;
    s_env_m = s_env_s = 0.0f;
    s_dp    = s_dn    = 0.0f;      /* ATC re-learns within ~2 bit times */
    s_key     = 1U;                /* idle = mark */
    s_dbn_ctr = 0U;
    s_edge_rd = s_edge_wr;         /* drop queued edges */
    s_fr      = FR_WAIT_MARK;
    s_line    = 1U;
    s_bit_idx = 0U;
    s_code    = 0U;
    s_figs    = false;
    /* text ring + counters deliberately preserved (CWDec_Reset semantics) */
}

void RTTY_SetReverse(bool rev)
{
    if (rev == s_rev) return;
    s_rev = rev;
    rtty_apply_tuning();   /* AFC flanks follow the logical-mark tone */
}

void RTTY_FeedAudio(float sample)
{
    /* Tone envelopes */
    float em = fabsf(rtty_bpf_run(&s_bpf_m, sample));
    float es = fabsf(rtty_bpf_run(&s_bpf_s, sample));
    s_env_m += s_a_env * (em - s_env_m);
    s_env_s += s_a_env * (es - s_env_s);

    s_clock++;

    /* ATC slicer on the difference signal.  Per-channel peak normalization
     * is degenerate under cross-channel filter leakage (a strong tone leaks
     * a proportional copy into the other filter, and the normalized scores
     * cancel exactly), so the classic scheme is used instead: track the
     * peaks/valleys of d = env_m − env_s and slice at their midpoint.
     * Leakage and selective fading shift both extremes together, and a
     * vanished signal decays both toward 0 — the slicer recentres itself. */
    float d = s_rev ? (s_env_s - s_env_m) : (s_env_m - s_env_s);
    s_dp = (d > s_dp) ? d : s_dp * s_pk_decay;
    s_dn = (d < s_dn) ? d : s_dn * s_pk_decay;
    float thr  = 0.5f * (s_dp + s_dn);
    float hyst = RTTY_HYST_FRAC * (s_dp - s_dn);

    uint8_t raw = s_key;
    if (s_key != 0U) { if (d < thr - hyst) raw = 0U; }
    else             { if (d > thr + hyst) raw = 1U; }

    /* AFC: flank imbalance integrated over solid logical-mark samples.
     * The filters run every sample (state continuity); only confident mark
     * intervals contribute, so noise and space bits never steer it. */
    {
        float lo = fabsf(rtty_bpf_run(&s_bpf_afc_lo, sample));
        float hi = fabsf(rtty_bpf_run(&s_bpf_afc_hi, sample));
        if (s_key != 0U && d > 0.25f * s_dp) {
            s_afc_lo_acc += lo;
            s_afc_hi_acc += hi;
            if (++s_afc_n >= s_afc_win) {
                float sum = s_afc_lo_acc + s_afc_hi_acc;
                if (sum > 1e-6f) {
                    float err = (s_afc_hi_acc - s_afc_lo_acc) / sum;
                    float afc = s_afc_hz;
                    if (err >  RTTY_AFC_DEADBAND) afc += RTTY_AFC_STEP_HZ;
                    if (err < -RTTY_AFC_DEADBAND) afc -= RTTY_AFC_STEP_HZ;
                    if (afc >  (float)RTTY_AFC_RANGE_HZ) afc =  (float)RTTY_AFC_RANGE_HZ;
                    if (afc < -(float)RTTY_AFC_RANGE_HZ) afc = -(float)RTTY_AFC_RANGE_HZ;
                    if (afc != s_afc_hz) {
                        s_afc_hz = afc;
                        rtty_apply_tuning();
                        dbg_rtty_afc_steps++;
                        dbg_rtty_afc_hz = (int32_t)afc;
                    }
                }
                s_afc_lo_acc = s_afc_hi_acc = 0.0f;
                s_afc_n = 0U;
            }
        }
    }

    /* Debounce + back-dated edge commit (same scheme as the CW envelope) */
    if (raw != s_key) {
        if (++s_dbn_ctr >= s_dbn_len) {
            uint32_t t = (s_clock >= (uint32_t)s_dbn_len)
                         ? (s_clock - s_dbn_len) : 0U;
            rtty_push_edge(raw, t);
            s_dbn_ctr = 0U;
        }
    } else {
        s_dbn_ctr = 0U;
    }
}

bool RTTY_Poll(void)
{
    bool changed = false;
    /* Only times ≤ safe are final: a transition still inside the debounce
     * window will be committed back-dated by up to s_dbn_len samples. */
    uint32_t safe = s_clock - s_dbn_len - s_guard;

    for (;;) {
        bool     have_edge = (s_edge_rd != s_edge_wr);
        uint32_t et = 0U;
        uint8_t  es = 0U;
        if (have_edge) {
            et = s_edge_t[s_edge_rd];
            es = s_edge_s[s_edge_rd];
        }

        if (s_fr == FR_CHAR) {
            /* Next scheduled mid-bit sample point: 0.5, 1.5 … 6.5 bits */
            uint32_t tk = s_t0 + s_spb / 2U + (uint32_t)s_bit_idx * s_spb;

            /* Replay edges that precede the sample point */
            if (have_edge && time_le(et, tk)) {
                if (!time_le(et, safe)) break;
                s_line    = es;
                s_edge_rd = (uint8_t)((s_edge_rd + 1U) & (RTTY_EDGE_RING - 1U));
                continue;
            }
            if (!time_le(tk, safe)) break;

            uint8_t bit = s_line;              /* state at tk, 1 = mark */
            if (s_bit_idx == 0U) {
                if (bit != 0U) {               /* start bit vanished: blip */
                    dbg_rtty_ferr++;
                    s_fr = FR_WAIT_START;      /* line is marking again */
                    continue;
                }
            } else if (s_bit_idx <= 5U) {
                s_code |= (uint8_t)(bit << (s_bit_idx - 1U));
            } else {
                if (bit != 0U) {               /* stop bit OK → commit */
                    changed |= rtty_emit_code(s_code);
                    s_fr = FR_WAIT_START;
                } else {                       /* framing error → resync */
                    dbg_rtty_ferr++;
                    s_fr = FR_WAIT_MARK;
                }
                continue;
            }
            s_bit_idx++;
            continue;
        }

        /* FR_WAIT_MARK / FR_WAIT_START: purely edge-driven */
        if (!have_edge) {
            if (s_fr == FR_WAIT_MARK && s_line != 0U) s_fr = FR_WAIT_START;
            break;
        }
        if (!time_le(et, safe)) break;

        uint8_t prev = s_line;
        s_line    = es;
        s_edge_rd = (uint8_t)((s_edge_rd + 1U) & (RTTY_EDGE_RING - 1U));

        if (s_fr == FR_WAIT_MARK) {
            if (s_line != 0U) s_fr = FR_WAIT_START;
        } else if (prev != 0U && s_line == 0U) {
            s_t0      = et;                    /* mark→space = start edge */
            s_bit_idx = 0U;
            s_code    = 0U;
            s_fr      = FR_CHAR;
        }
    }

    return changed;
}

void RTTY_GetText(char *buf, uint8_t max_chars)
{
    if (!buf || max_chars == 0U) return;

    uint8_t avail = s_count;
    uint8_t n = (avail < max_chars) ? avail : (uint8_t)(max_chars - 1U);

    uint8_t start = (uint8_t)((s_head + RTTY_TEXT_LEN - n) % RTTY_TEXT_LEN);
    for (uint8_t i = 0U; i < n; i++)
        buf[i] = s_text[(start + i) % RTTY_TEXT_LEN];
    buf[n] = '\0';
}
