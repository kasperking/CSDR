/* Host-side regression sim for BSP/Src/rtty_decode.c
 *
 * Generates phase-continuous 45.45 Bd AFSK (mark 2125 / space 2295 Hz,
 * fs = 48 kHz) from an independently-written Baudot encoder table, feeds
 * RTTY_FeedAudio sample-by-sample with RTTY_Poll every 10 ms (main-loop
 * cadence), and compares decoded text against the expectation.
 *
 * Cases: clean, reverse polarity, AWGN (2 levels), selective mark fade,
 * +30 Hz tuning offset.
 *
 * Build (any host C compiler; no HAL dependencies):
 *   gcc -O2 -I BSP/Inc tools/sim_rtty.c BSP/Src/rtty_decode.c -o sim_rtty
 * or without a native gcc (pip install ziglang):
 *   python -m ziglang cc -O2 -I BSP/Inc tools/sim_rtty.c BSP/Src/rtty_decode.c -o sim_rtty.exe
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rtty_decode.h"

#define FS 48000.0
static double g_spb;              /* samples per bit (set per case) */

/* ── Independent Baudot encoder (US-TTY figures) ─────────────────────────── */
static int enc_lookup(char c, int *shift /*0=ltrs 1=figs*/)
{
    static const char *L = " E\nA SIU\rDRJNFCKTZLWHYPQOBG.MXV.";
    /* index:              0 1 2 3 4 5 6 7 8 9 ...                  */
    static const char ltrs[32] = {
        0,'E',0,'A',' ','S','I','U',0,'D','R','J','N','F','C','K',
        'T','Z','L','W','H','Y','P','Q','O','B','G',0,'M','X','V',0 };
    static const char figs[32] = {
        0,'3',0,'-',' ',0,'8','7',0,'$','4','\'',',','!',':','(',
        '5','"',')','2','#','6','0','1','9','?','&',0,'.','/',';',0 };
    (void)L;
    for (int i = 1; i < 32; i++) {
        if (i == 27 || i == 31) continue;
        if (ltrs[i] == c) { *shift = 0; return i; }
    }
    for (int i = 1; i < 32; i++) {
        if (i == 27 || i == 31) continue;
        if (figs[i] && figs[i] == c) { *shift = 1; return i; }
    }
    return -1;
}

/* ── FSK generator state ─────────────────────────────────────────────────── */
static double g_phase;
static double g_bit_frac;         /* fractional sample carry across bits */
static unsigned g_seed = 12345u;

static double urand(void)
{
    g_seed = g_seed * 1664525u + 1013904223u;
    return (double)g_seed / 4294967296.0;
}
static double gauss(void)
{
    double s = 0.0;
    for (int i = 0; i < 12; i++) s += urand();
    return s - 6.0;
}

typedef struct {
    double noise;                 /* AWGN sigma (broadband)               */
    double mark_amp, space_amp;   /* per-tone amplitude (fading tests)    */
    double freq_off;              /* tuning error Hz                      */
    double baud;                  /* modulation rate (0 → 45.45)          */
    double shift;                 /* mark→space shift Hz (0 → 170)        */
    int    leader_bits;           /* idle mark leader (0 → 30)            */
    int    reverse;               /* swap tones (USB sense) + SetReverse  */
    long   fade_from, fade_to;    /* sample window: mark_amp *= 0.2       */
    long   n;                     /* running sample counter               */
} gen_t;

static void send_bit(gen_t *g, int bit /*1=mark*/)
{
    double mark_f  = 2125.0;
    double space_f = 2125.0 + g->shift;
    double f = bit ? mark_f : space_f;
    if (g->reverse) f = bit ? space_f : mark_f;
    f += g->freq_off;
    double amp = bit ? g->mark_amp : g->space_amp;

    double len = g_spb + g_bit_frac;
    long   ns  = (long)len;
    g_bit_frac = len - (double)ns;

    for (long i = 0; i < ns; i++) {
        double a = amp;
        if (bit && g->fade_from >= 0 && g->n >= g->fade_from) {
            /* ramp 1.0 → 0.2 over 0.5 s, then hold */
            double r = (double)(g->n - g->fade_from) / 24000.0;
            if (r > 1.0) r = 1.0;
            a *= 1.0 - 0.8 * r;
        }
        g_phase += 2.0 * M_PI * f / FS;
        float s = (float)(a * sin(g_phase) + g->noise * gauss());
        RTTY_FeedAudio(s);
        g->n++;
        if ((g->n % 480) == 0) (void)RTTY_Poll();
    }
}

static void send_code(gen_t *g, int code)
{
    send_bit(g, 0);                              /* start */
    for (int i = 0; i < 5; i++) send_bit(g, (code >> i) & 1);
    send_bit(g, 1); send_bit(g, 1);              /* stop ≥1.5, send 2 */
}

static void send_text(gen_t *g, const char *txt)
{
    int state = 0;                               /* 0 = LTRS */
    int lead  = g->leader_bits > 0 ? g->leader_bits : 30;
    for (int i = 0; i < lead; i++) send_bit(g, 1); /* idle mark leader */
    send_code(g, 31);                            /* LTRS */
    for (const char *p = txt; *p; p++) {
        int shift, code = enc_lookup(*p, &shift);
        if (code < 0) { fprintf(stderr, "unencodable '%c'\n", *p); exit(2); }
        if (*p == ' ') {
            send_code(g, 4);
            state = 0;                           /* USOS: space unshifts */
            continue;
        }
        if (shift != state) { send_code(g, shift ? 27 : 31); state = shift; }
        send_code(g, code);
    }
    for (int i = 0; i < 30; i++) send_bit(g, 1); /* trailer */
    (void)RTTY_Poll();
}

static const char *trim(char *s)
{
    size_t len = strlen(s);
    while (len && s[len - 1] == ' ') s[--len] = 0;
    while (*s == ' ') s++;
    return s;
}

extern volatile uint32_t dbg_rtty_chars, dbg_rtty_ferr, dbg_rtty_edges,
                         dbg_rtty_edge_drop;

/* allow_errs: chars allowed to differ (same length) — RTTY has no FEC, a
 * stray bit at very low SNR is expected, not a decoder defect. */
static int run_case2(const char *name, gen_t *g, int reverse, const char *msg,
                     int allow_errs)
{
    if (g->baud  <= 0.0) g->baud  = 45.45;
    if (g->shift <= 0.0) g->shift = 170.0;
    g_spb = FS / g->baud;

    RTTY_Init(48000);
    RTTY_Configure((uint16_t)(g->baud * 100.0 + 0.5), (uint16_t)g->shift);
    RTTY_SetReverse(reverse);
    g_phase = 0.0; g_bit_frac = 0.0;
    dbg_rtty_chars = dbg_rtty_ferr = dbg_rtty_edges = 0;

    send_text(g, msg);

    char out[RTTY_TEXT_LEN + 1];
    RTTY_GetText(out, sizeof(out));
    const char *got = trim(out);
    int errs = -1;
    if (strlen(got) == strlen(msg)) {
        errs = 0;
        for (size_t i = 0; msg[i]; i++) if (got[i] != msg[i]) errs++;
    }
    int pass = (errs >= 0 && errs <= allow_errs);
    printf("%-22s %s  char_errs=%d ferr=%lu edges=%lu drop=%lu afc=%+d\n",
           name, pass ? "PASS" : "FAIL", errs,
           (unsigned long)dbg_rtty_ferr, (unsigned long)dbg_rtty_edges,
           (unsigned long)dbg_rtty_edge_drop, (int)RTTY_GetAfcHz());
    if (!pass || errs > 0) {
        printf("   exp: [%s]\n   got: [%s]\n", msg, got);
    }
    return pass;
}

static int run_case(const char *name, gen_t *g, int reverse, const char *msg)
{
    return run_case2(name, g, reverse, msg, 0);
}

int main(void)
{
    /* ≤ 62 chars so the full message fits the text ring */
    const char *msg = "CQ CQ DE CSDR CSDR RYRY 599 073 BTU 73";
    int ok = 1;
    gen_t g;

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1;
    ok &= run_case("clean", &g, 0, msg);

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1; g.reverse = 1;
    ok &= run_case("reverse (USB)", &g, 1, msg);

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1; g.noise = 0.05;
    ok &= run_case("noise sigma=0.05", &g, 0, msg);

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1; g.noise = 0.3;
    ok &= run_case2("noise sigma=0.30", &g, 0, msg, 2);

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.noise = 0.02;
    g.fade_from = 150000; g.fade_to = 450000;    /* mark −14 dB mid-msg */
    ok &= run_case("selective mark fade", &g, 0, msg);

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1; g.freq_off = 30.0;
    ok &= run_case("tuning +30 Hz", &g, 0, msg);

    /* AFC: +50 Hz detune, 200-bit idle-mark leader (~4.4 s) so the flanking
     * discriminator converges before the message starts */
    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1;
    g.freq_off = 50.0; g.leader_bits = 200;
    ok &= run_case("AFC detune +50 Hz", &g, 0, msg);

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1; g.baud = 75.0;
    ok &= run_case("75 Bd", &g, 0, msg);

    memset(&g, 0, sizeof(g));
    g.mark_amp = g.space_amp = 0.1; g.fade_from = -1; g.shift = 850.0;
    ok &= run_case("shift 850 Hz", &g, 0, msg);

    printf(ok ? "ALL PASS\n" : "FAILURES\n");
    return ok ? 0 : 1;
}
