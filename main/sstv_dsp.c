/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * sstv_dsp.c -- see sstv_dsp.h.
 *
 * AI PASSPORT (ESP32-C3) BUILD: copied from the iPhone / Watch build
 * (apricot-sstv-apple, Shared/Core). The C3 has no FPU and ~300 KB of heap:
 *  - Everything that runs per input or tone sample is integer: the
 *    high-pass (Q29 biquads), the mixer (an exact table of the LO's period),
 *    the FIR (Q15 taps on Q4 data), the discriminator and its atan2 (a
 *    257-entry table), the tone stream (Hz x 16), the VIS bins, the picture
 *    sampling (Q16 positions). Float / double is left only in what runs once
 *    a line or less (the line fit, the mode table, autosync's checks).
 *  - The level ring is SSTV_LV_LOG2 (default 2^16: 5.9 s at 11025) instead
 *    of a whole picture: lines older than that keep the timing they were
 *    drawn with (the nano's behaviour). Tone history: SSTV_FH_LOG2.
 *  - No picture buffer: finished rows go straight to the app (SstvSink).
 *    Robot 36 keeps only a ring of Y / chroma lines as deep as the level
 *    ring, since no older line can be re-timed anyway.
 *  - The next period's due time is kept as one integer, not recomputed in
 *    double for every tone sample.
 *
 * Freestanding: uses only malloc/free/memset (no libm; the few trig values
 * needed at init come from m_sin below). Every block is taken in sstv_new,
 * at a size fixed by the input rate's decimation (never by the mode).
 *
 * Every mode is one row of the table below: its sync, line period, picture
 * size and the place of each colour channel, in seconds from the START of the
 * mode's sync pulse (Scottie's G and B come before its sync, so they're
 * negative). The decoder locks on syncs, fits the line period, and samples
 * the channels from that table; nothing else is mode-specific except the
 * Robot 36 chroma (alternating R-Y / B-Y lines, picked by the separator tone).
 *
 * VIS: 1900 leader 300 ms, 1200 break 10 ms, 1900 leader 300 ms, 1200 start
 * bit 30 ms, 7 data bits LSB first + even parity (1100 = 1, 1300 = 0, 30 ms
 * each), 1200 stop bit 30 ms. Robot, Martin and PD send line 0's sync right
 * after the stop bit; Scottie sends an extra 9 ms sync, then a whole G/B half
 * line before line 0's sync (the `first` column).
 */
#include "sstv_dsp.h"
#include <stdlib.h>
#include <string.h>

#ifdef SSTV_POOL                        /* the nano: the app's pool (sstv_support.c) */
void *sstv_pool_alloc(size_t n);
void  sstv_pool_free(void *p);
#define D_ALLOC(n) sstv_pool_alloc(n)
#define D_FREE(p)  sstv_pool_free(p)
#else
#define D_ALLOC(n) malloc(n)
#define D_FREE(p)  free(p)
#endif
static void *d_calloc(size_t n, size_t sz)
{
    void *p = D_ALLOC(n * sz);
    if (p) memset(p, 0, n * sz);
    return p;
}

#define PI_D     3.14159265358979323846
#define FC_HZ    1700                   /* mixer centre: 1100..2300 -> -600..+600 */
#define FIR_FCUT 1500.0                 /* low-pass -6 dB point (image at -2800 Hz is rejected) */
#define FIR_TAPS_44K 127
/* High-pass before the mixer, HP_STAGES biquads (Butterworth, 2 = 4th order)
 * at HP_HZ. The low-pass above still passes 200 Hz at about -6 dB, and wind,
 * traffic and handling rumble picked up by a microphone are often louder
 * than the signal: enough to hide the VIS bits and the 1200 Hz syncs, and to
 * pull the tuning-offset estimate far off. */
#ifndef HP_STAGES
#define HP_STAGES    2
#endif
#ifndef HP_HZ
#define HP_HZ        450.0
#endif

#define VB_N     1024                   /* 1 ms tone bins kept for the VIS detector */
#define VB_MASK  (VB_N - 1)

#define MAXP         320                /* sync periods per image (Martin/Scottie: 256, PD290: 308) */
#define R36_W        320
#define R36_H        240

#define SYNC_Q_MIN   0.60f              /* matched-template score for a "good" sync */
#define AUTO_PULSE_Q 0.60f              /* pulse candidates for autosync (each is then checked
                                           against every mode's sync length, grid and landmarks) */
#ifndef SSTV_LV_LOG2
#define SSTV_LV_LOG2 16                 /* level ring: 64 KiB = 5.9 s at Fs = 11025. Lines older
                                           than that keep the timing they were drawn with */
#endif
#ifndef SSTV_FH_LOG2
#define SSTV_FH_LOG2 15                 /* tone history: 32 KiB = 3.0 s at 11025 (autosync's
                                           landmark checks for the slowest PD modes see fewer
                                           lines; their VIS start is unaffected) */
#endif
#define LV_LEN       (1u << SSTV_LV_LOG2)
#define FH_LEN       (1u << SSTV_FH_LOG2)
#define FH_HZ0       900.0f             /* tone history as uint8: 900 + 6 * q Hz */
#define FH_STEP      6.0f
/* tone samples are integers in 1/16 Hz */
#define TQ           16
#define HZ(x)        ((int32_t)((x) * TQ))
#define LOST_S_VIS   12.0               /* seconds of missed syncs -> give up (VIS start) */
#define LOST_S_SYNC  6.0                /* ... after an autosync start */
#define MAX_SKEW     0.03               /* clamp the fitted line period to +-3 % */
#ifndef RD_BUDGET
#define RD_BUDGET    10240              /* channel-pixels re-timed per sstv_feed call (bounds
                                           the cost of one call; the Passport sets it lower) */
#endif
#define NPULSE       32
#ifndef SYNC_LEN_PCT
#define SYNC_LEN_PCT 55                 /* % of a pulse that must read as sync tone for autosync
                                           (was 70: echo and fading smear a sync's edges; the
                                           grid and landmark checks still reject noise) */
#endif
#ifndef SP_STEP_HZ
#define SP_STEP_HZ   60                 /* a tone step this small (per tone sample) is "picture" */
#endif
#ifndef SP_MIN_PCT
#define SP_MIN_PCT   60u                /* % of small steps in a period for a picture to be there */
#endif
#define NGAPS        8                  /* recent input gaps remembered (tone-index ranges) */
#define RESUME_S     120.0              /* resume a lost picture if its syncs return within this */

/* ------------------------------------------------------------------ modes */
enum { CM_R36, CM_YUV, CM_RGB };
enum { CH_Y, CH_Y2, CH_V, CH_U, CH_R, CH_G, CH_B, CH_C };
typedef struct { float t0, len; uint8_t what; } Chan;
typedef struct { float a, b, f1, f2; } Shape;     /* tone in [a,b] s after sync start ~ f1 (or f2) */
typedef struct {
    const char *name;
    int    vis, w, h, rows, color;               /* rows: picture rows per sync (PD: 2) */
    double line, sync, porch, first;             /* s; first = VIS end -> line 0's sync */
    int    nch;
    Chan   ch[4];
    int    nshape;
    Shape  shape[3];
} Mode;

#define PDM(nm, v, W, H, L, T) { nm, v, W, H, 2, CM_YUV, L, 0.020, 0.00208, 0.0, 4, \
    { { 0.02208f, T, CH_Y }, { 0.02208f + (T), T, CH_V }, { 0.02208f + 2 * (T), T, CH_U }, \
      { 0.02208f + 3 * (T), T, CH_Y2 } }, 1, { { 0.0203f, 0.0218f, 1500, 0 } } }

static const Mode MODES[] = {
    { "Robot 36", 8, 320, 240, 1, CM_R36, 0.150, 0.009, 0.003, 0.0, 2,
      { { 0.012f, 0.088f, CH_Y }, { 0.106f, 0.044f, CH_C } },
      3, { { 0.0095f, 0.0115f, 1500, 0 }, { 0.1007f, 0.1038f, 1500, 2300 }, { 0.1048f, 0.1058f, 1900, 0 } } },
    { "Robot 72", 12, 320, 240, 1, CM_YUV, 0.300, 0.009, 0.003, 0.0, 3,
      { { 0.012f, 0.138f, CH_Y }, { 0.156f, 0.069f, CH_V }, { 0.231f, 0.069f, CH_U } },
      3, { { 0.0095f, 0.0115f, 1500, 0 }, { 0.1507f, 0.1538f, 1500, 0 }, { 0.2257f, 0.2288f, 2300, 0 } } },
    { "Martin M1", 44, 320, 256, 1, CM_RGB, 0.446446, 0.004862, 0.000572, 0.0, 3,
      { { 0.005434f, 0.146432f, CH_G }, { 0.152438f, 0.146432f, CH_B }, { 0.299442f, 0.146432f, CH_R } },
      0, { { 0, 0, 0, 0 } } },
    { "Martin M2", 40, 320, 256, 1, CM_RGB, 0.226798, 0.004862, 0.000572, 0.0, 3,
      { { 0.005434f, 0.073216f, CH_G }, { 0.079222f, 0.073216f, CH_B }, { 0.153010f, 0.073216f, CH_R } },
      0, { { 0, 0, 0, 0 } } },
    { "Scottie S1", 60, 320, 256, 1, CM_RGB, 0.42822, 0.009, 0.0015, 0.28848, 3,
      { { 0.0105f, 0.13824f, CH_R }, { -0.27798f, 0.13824f, CH_G }, { -0.13824f, 0.13824f, CH_B } },
      1, { { 0.0093f, 0.0102f, 1500, 0 } } },
    { "Scottie S2", 56, 320, 256, 1, CM_RGB, 0.277692, 0.009, 0.0015, 0.188128, 3,
      { { 0.0105f, 0.088064f, CH_R }, { -0.177628f, 0.088064f, CH_G }, { -0.088064f, 0.088064f, CH_B } },
      1, { { 0.0093f, 0.0102f, 1500, 0 } } },
    { "Scottie DX", 76, 320, 256, 1, CM_RGB, 1.0503, 0.009, 0.0015, 0.7032, 3,
      { { 0.0105f, 0.3456f, CH_R }, { -0.6927f, 0.3456f, CH_G }, { -0.3456f, 0.3456f, CH_B } },
      1, { { 0.0093f, 0.0102f, 1500, 0 } } },
    PDM("PD50",  93, 320, 256, 0.38816,  0.09152f),
    PDM("PD90",  99, 320, 256, 0.70304,  0.17024f),
    PDM("PD120", 95, 640, 496, 0.50848,  0.1216f),
    PDM("PD160", 98, 512, 400, 0.804416, 0.195584f),
    PDM("PD180", 96, 640, 496, 0.75424,  0.18304f),
    PDM("PD240", 97, 640, 496, 1.0,      0.24448f),
    PDM("PD290", 94, 800, 616, 0.93728,  0.2288f),
};
#define NMODES ((int)(sizeof MODES / sizeof MODES[0]))

static const Mode *mode_by_vis(int vis)
{
    int i;
    for (i = 0; i < NMODES; i++) if (MODES[i].vis == vis) return &MODES[i];
    return 0;
}
static int mode_periods(const Mode *m) { return m->h / m->rows; }
static int mode_sw(const Mode *m) { return m->w; }   /* rows go out at the size sent */
static double mode_t_min(const Mode *m)
{
    double t = 0; int c;
    for (c = 0; c < m->nch; c++) if (m->ch[c].t0 < t) t = m->ch[c].t0;
    return t;
}
static double mode_t_max(const Mode *m)
{
    double t = m->sync + m->porch; int c;
    for (c = 0; c < m->nch; c++) if (m->ch[c].t0 + m->ch[c].len > t) t = m->ch[c].t0 + m->ch[c].len;
    return t;
}

enum { CT_V = 0, CT_U = 1 };            /* Robot 36: chroma carried by a line */

typedef struct { uint32_t edge; } Pulse;      /* autosync: a sync's trailing edge */

struct SstvRx {
    int    in_rate, D;
    double Fs;                          /* tone (decimated) rate */

    /* high-pass, mixer and decimating complex FIR at the input rate, all integer */
    int32_t  hp_b[HP_STAGES][3], hp_a[HP_STAGES][2];   /* Q29 biquad coefficients (direct form I) */
    int32_t  hp_x[HP_STAGES][2], hp_y[HP_STAGES][2];   /* their history, Q8 */
    int16_t *lo; int lo_n, lo_i;        /* the LO e^(-j 2pi FC n/R): one exact period, (re, im) Q15 */
    int16_t *h; int L, M;               /* M = (L-1)/2, symmetric taps h[0..M], Q15 (sum 32768) */
    int32_t *bi, *bq; int bpos, dcount; /* mixed samples, Q4 */
    int32_t  zr1, zi1;                  /* previous decimated sample, Q4 */
    int64_t  tone_mul;                  /* Hz x 16 per (2^-24 turn), << 32: Fs * 4096 */

    /* tone stream (tones in 1/16 Hz) */
    uint32_t n;                         /* tone samples produced so far */
    uint8_t *fh; uint32_t fh_mask;      /* raw tone history, FH_HZ0 + FH_STEP * q */
    uint8_t *lv; uint8_t *sb; uint32_t lv_mask;   /* level + sync-bit rings */
    float    off;                       /* tuning offset (Hz) applied to lv/sb ... */
    int32_t  off_q;                     /* ... the same in 1/16 Hz */
    int32_t  tone_ema;                  /* smoothed tone, 1/16 Hz << 8 */
    int32_t  ema_a;                     /* its rate, Q16 */
    int32_t  med1, med2;                /* median-of-3 history */

    /* VIS detector (1 ms bins, 1/16 Hz) */
    int32_t  bins[VB_N];
    uint32_t bin_j;                     /* index of the bin being accumulated */
    uint32_t bin_end;                   /* tone sample index where it ends */
    int32_t  bin_acc; int bin_cnt;
    int      vis_active; uint32_t vis_first, vis_last, vis_deadline;
    int      vis_code; float vis_off;

    /* sync-pulse detector (autosync): generic 4 ms sync + 1 ms non-sync */
    int      autosync;
    int      lock_vis;                  /* sstv_set_mode: -1 = any mode */
    int      gA, gB, runA, runB;
    int      cand; int candC; uint32_t candT1, candT2;
    Pulse    pulses[NPULSE]; int npulses;

    /* the image */
    const Mode *m;                      /* current/last mode */
    int      Ls, La, Lp;                /* its sync, sync-template, porch lengths (tone samples) */
    int      state, start_kind, end_kind, last_vis, last_vis_ok;
    double   Pnom, Pprior; int ppm_prior;
    double   t_ref;                     /* absolute tone index all line times are relative to */
    double   fa, fb;                    /* period k's sync starts at t_ref + fa + fb*k */
    int      fit_n, fit_span, fit_full;
    int      k, nper;                   /* next sync period to decode, periods per image */
    int      miss_run, sync_good, sync_seen, redraws, relocks;
    int      absent_run;                /* consecutive periods with neither a sync nor a picture */
    uint32_t sp_n, sp_ok; int32_t sp_prev; /* this period: tone steps, and how many were small */
    uint32_t due;                       /* tone index from which period k can be processed */
    int      par_v_even, par_v_odd;     /* Robot 36: separator votes for V on even / odd lines */
    int      par_phase;                 /* ... and the parity they settled on (1: V on odd lines) */
    /* darkroom: decoding a recording again with settings chosen by hand */
    int      manual;                    /* started by sstv_start_at: no VIS starts, never "lost" */
    int      slant_fixed; double slant_ppm;   /* line period fixed at this clock error */
    double   shift_ms;                  /* every line sampled this much later (picture moves left) */
    int      can_resume;                /* last picture ended early (lost/aborted), not counted */
    uint32_t t_end;                     /* tone index where it ended */
    uint32_t gap0[NGAPS], gap1[NGAPS]; int gapi;   /* [gap0, gap1): tone samples that were gaps */
    int      rd_pending, rd_j;          /* re-timing sweep in progress, next period */
    double   mpos[MAXP];  uint8_t mgood[MAXP];
    double   tused[MAXP]; uint8_t done[MAXP], ct[R36_H];
    uint8_t  *Y, *C;                    /* Robot 36 lines, R36_W each (chroma pairs across lines):
                                           a ring of yc_n lines, line j in slot j % yc_n */
    int16_t  *yc_line;                  /* the line each slot holds, -1 = none */
    int      yc_n;
    uint8_t  chbuf[4][SSTV_MAX_W];      /* one period's channels, levels */
    uint16_t row[SSTV_MAX_W];           /* the row being handed to the sink */
    SstvSink sink; void *sink_user;
    unsigned revision;
    unsigned long images, gaps;
};

/* ------------------------------------------------------------------ math */
static double m_sin(double x)
{
    int q; double x2, t, s; int i;
    q = (int)(x / (2.0 * PI_D) + (x >= 0 ? 0.5 : -0.5));
    x -= q * 2.0 * PI_D;                            /* [-pi, pi] */
    if (x > PI_D / 2)  x = PI_D - x;                /* [-pi/2, pi/2] */
    if (x < -PI_D / 2) x = -PI_D - x;
    x2 = x * x; t = x; s = x;
    for (i = 1; i < 9; i++) { t *= -x2 / ((2 * i) * (2 * i + 1)); s += t; }
    return s;
}
static double m_cos(double x) { return m_sin(x + PI_D / 2); }

/* atan2 with a degree-11 odd polynomial on [0,1]: measured |error| < 2e-6 rad
 * in float (0.003 Hz of tone at 11025). No libm. */
static inline float f_atan2(float y, float x)
{
    float ax = x < 0 ? -x : x, ay = y < 0 ? -y : y, a, s, r;
    if (ax == 0.0f && ay == 0.0f) return 0.0f;
    a = ax > ay ? ay / ax : ax / ay;
    s = a * a;
    r = a * (0.99997726f + s * (-0.33262347f + s * (0.19354346f +
        s * (-0.11643287f + s * (0.05265332f + s * -0.01172120f)))));
    if (ay > ax) r = 1.57079637f - r;
    if (x < 0)   r = 3.14159274f - r;
    return y < 0 ? -r : r;
}

static int iround(double x) { return (int)(x + (x >= 0 ? 0.5 : -0.5)); }

/* Integer atan2 for the discriminator: angle in 2^-24 turns (a quarter turn
 * is 2^22). The ratio min/max is taken to 15 bits after normalising both to
 * 16 bits, and atan of it read from a 257-entry table with linear
 * interpolation: |error| < 2e-6 rad, as the float polynomial it replaces. */
#define AT_N 256
static int32_t at_tab[AT_N + 1];           /* atan(i / 256), 2^-24 turns; filled once */
static void at_init(void)
{
    int i;
    if (at_tab[AT_N]) return;
    for (i = 0; i <= AT_N; i++)
        at_tab[i] = (int32_t)(f_atan2((float)i, (float)AT_N) * (16777216.0 / (2.0 * PI_D)) + 0.5);
}
static inline int32_t i_atan2(int64_t y, int64_t x)
{
    uint64_t ax = (uint64_t)(x < 0 ? -x : x), ay = (uint64_t)(y < 0 ? -y : y), mx = ax > ay ? ax : ay;
    uint32_t a, b, r, fr;
    int32_t ang;
    int sh = 0;
    if (mx == 0) return 0;
    /* both to 16 bits (the larger in [2^15, 2^16)); binary steps, the C3 has no clz */
    if (mx >> 32) { sh = 16; if (mx >> 48) sh = 32; }
    while ((mx >> sh) >= (1u << 20)) sh += 4;
    while ((mx >> sh) >= 65536u) sh++;
    a = (uint32_t)((ax > ay ? ay : ax) >> sh);
    b = (uint32_t)(mx >> sh);
    if (b == 0) return 0;
    r = (a << 15) / b;                      /* min / max, Q15, 0..32768 */
    fr = r & 127u; r >>= 7;
    ang = at_tab[r] + (int32_t)(((int64_t)(at_tab[r < AT_N ? r + 1 : r] - at_tab[r]) * fr) >> 7);
    if (ay > ax) ang = (1 << 22) - ang;
    if (x < 0)   ang = (1 << 23) - ang;
    return y < 0 ? -ang : ang;
}

/* ------------------------------------------------------------ ring access */
static inline int sbit(const SstvRx *s, uint32_t i)
{
    i &= s->lv_mask;
    return (s->sb[i >> 3] >> (i & 7u)) & 1;
}
/* f in 1/16 Hz. Level = (f - off - 1500 Hz) * 255 / 800 Hz, rounded (1306 / 2^16
 * is 255 / 12800 to 1e-4) */
static inline void put_tone(SstvRx *s, uint32_t i, int32_t f)
{
    int32_t d = f - s->off_q - HZ(1500), v = (int32_t)(((int64_t)d * 1306 + 32768) >> 16);
    uint32_t j = i & s->lv_mask;
    uint8_t bit = (uint8_t)(1u << (j & 7u));
    s->lv[j] = (uint8_t)(v <= 0 ? 0 : v >= 255 ? 255 : v);
    if (f - s->off_q < HZ(1350)) s->sb[j >> 3] |= bit; else s->sb[j >> 3] &= (uint8_t)~bit;
}
static inline float fh_at(const SstvRx *s, uint32_t i) { return FH_HZ0 + FH_STEP * (float)s->fh[i & s->fh_mask]; }
static inline int32_t fh_q(const SstvRx *s, uint32_t i) { return HZ(900) + HZ(6) * (int32_t)s->fh[i & s->fh_mask]; }

/* --------------------------------------------------------------- picture */
static inline int clip8(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }
static inline uint16_t rgb565(int r, int g, int b)
{
    r = clip8(r); g = clip8(g); b = clip8(b);
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
/* full-range BT.601 (Q8): R = Y + 1.402V, G = Y - 0.344U - 0.714V, B = Y + 1.772U */
static inline uint16_t yuv565(int y, int v, int u)
{
    v -= 128; u -= 128;
    return rgb565(y + ((359 * v + 128) >> 8), y - ((88 * u + 183 * v + 128) >> 8), y + ((454 * u + 128) >> 8));
}
/* hand row y (stored size) of the current picture to the sink */
static void emit_row(SstvRx *s, int y, const uint16_t *px, int w)
{
    if (s->sink.row) s->sink.row(s->sink_user, y, px, w);
}

/* Robot 36 line j's slot in the Y / chroma ring, if it still holds line j */
static int yc_slot(const SstvRx *s, int j)
{
    int q;
    if (j < 0 || j >= R36_H) return -1;
    q = j % s->yc_n;
    return s->yc_line[q] == j ? q : -1;
}

/* Robot 36: a line takes its partner's chroma (the other kind, R-Y vs B-Y) */
static void compose36(SstvRx *s, int j)
{
    const uint8_t *V, *U, *O = 0, *Cj, *Yj;
    int t, i, cand[3], c, q;
    if (j < 0 || j >= R36_H || !s->done[j] || (q = yc_slot(s, j)) < 0) return;
    Yj = s->Y + q * R36_W; Cj = s->C + q * R36_W;
    t = s->ct[j];
    cand[0] = (j & 1) ? j - 1 : j + 1; cand[1] = j - 1; cand[2] = j + 1;
    for (c = 0; c < 3 && !O; c++) {
        int p = cand[c], qp = yc_slot(s, p);
        if (qp >= 0 && s->done[p] && s->ct[p] != t) O = s->C + qp * R36_W;
    }
    V = (t == CT_V) ? Cj : O;
    U = (t == CT_U) ? Cj : O;
    for (i = 0; i < R36_W; i++)                     /* missing partner line -> no colour */
        s->row[i] = yuv565(Yj[i], V ? V[i] : 128, U ? U[i] : 128);
    emit_row(s, j, s->row, R36_W);
}


/* positions within a period are Q16 tone samples from its base (a period is
 * at most ~17000 samples, well inside 16 integer bits) */
static inline uint32_t q16(float x) { return x <= 0.0f ? 0u : (uint32_t)(x * 65536.0f + 0.5f); }

/* level at base + off (Q16, off >= 0), Q8 */
static inline int32_t lvl_q(const SstvRx *s, uint32_t base, uint32_t off)
{
    uint32_t i = (off >> 16) + base, fr = off & 0xffffu;
    int32_t a, b;
    if (i + 1u >= s->n) { i = s->n - 2u; fr = 0x10000u; }  /* never read unwritten samples */
    a = s->lv[i & s->lv_mask]; b = s->lv[(i + 1u) & s->lv_mask];
    return a * 256 + (int32_t)(((b - a) * (int32_t)fr) >> 8);
}

/* npix pixels of a channel starting at `start` (samples from base), lasting
 * `len` samples. Sample points stay a little inside the scan so the filter's
 * smear of the neighbouring tones (porch, separator, the next sync) does not
 * tint the first/last pixels. Three points a pixel, averaged. */
static void sample_chan(const SstvRx *s, uint32_t base, float start, float len, int npix, uint8_t *out)
{
    float pt = len / (float)npix, ms = (float)(0.001 * s->fb / s->m->line);   /* samples per ms */
    float lo = start + (0.15f * ms < pt * 0.5f ? 0.15f * ms : pt * 0.5f);
    float hi = start + len - (0.25f * ms < pt * 0.5f ? 0.25f * ms : pt * 0.5f);
    uint32_t qlo = q16(lo), qhi = q16(hi), qpt = q16(pt), q3 = q16(pt / 3.0f);
    uint32_t c = q16(start) + qpt / 2u;
    int i;
    if (qhi < qlo) qhi = qlo;
    for (i = 0; i < npix; i++, c += qpt) {
        uint32_t a = c > q3 ? c - q3 : 0u, b = c + q3, m = c;
        int32_t v;
        a = a < qlo ? qlo : a > qhi ? qhi : a;
        m = m < qlo ? qlo : m > qhi ? qhi : m;
        b = b < qlo ? qlo : b > qhi ? qhi : b;
        v = lvl_q(s, base, a) + lvl_q(s, base, m) + lvl_q(s, base, b);   /* 3 x Q8 */
        v = (v + 384) / 768;
        out[i] = (uint8_t)(v > 255 ? 255 : v < 0 ? 0 : v);
    }
}

/* is most of [a, b) (tone indices) inside recorded input gaps? (Short gaps
 * are drawn: sstv_gap holds the last tone across them, a small smear.) */
static int in_gap(const SstvRx *s, double a, double b)
{
    int i;
    double lost = 0;
    for (i = 0; i < NGAPS; i++) {
        double g0 = (double)s->gap0[i], g1 = (double)s->gap1[i];
        if (g1 <= g0 || b <= g0 || a >= g1) continue;
        lost += (b < g1 ? b : g1) - (a > g0 ? a : g0);
    }
    return lost * 2.0 > b - a;
}

/* Sample sync period k, whose sync starts at tone index t (from the fit), and
 * write its picture row(s). */
static void render_period(SstvRx *s, int k, double t)
{
    const Mode *m = s->m;
    double t_model = t, Fs = s->fb / m->line;      /* the sender's clock as we see it (fitted) */
    double tmin = mode_t_min(m) * Fs;
    uint32_t base;
    int c, i, w = mode_sw(m);
    /* the measured sync edge is the first porch-bit index, i.e. ~half a tone
     * sample after the true crossing; take it back (edge test: decoded edges
     * sat 0.17 px early without this, within 0.05 px with it) */
    t -= 0.5;
    t += s->shift_ms * 0.001 * Fs;                 /* darkroom: move the picture sideways */
    /* t is (trailing edge - nominal sync); lay the channels out from the edge
     * itself with the fitted clock, so a big skew doesn't shift them */
    t += s->Ls - m->sync * Fs;
    if (t + tmin < 0) t = -tmin;
    if (in_gap(s, t + tmin, t + mode_t_max(m) * Fs)) {   /* audio lost: leave the row as it is */
        s->tused[k] = t_model;
        return;
    }
    base = (uint32_t)(t + tmin);
    t -= (double)base;                             /* everything below is relative to base */
    for (c = 0; c < m->nch; c++)
        sample_chan(s, base, (float)(t + m->ch[c].t0 * Fs), (float)(m->ch[c].len * Fs),
                    w, s->chbuf[c]);
    if (m->color == CM_R36) {
        /* R-Y and B-Y alternate line by line, and the separator after Y says
         * which one follows. A faded or echoing separator is easily misread,
         * and one misread line swaps its colours (a stripe), so the lines vote:
         * each clear separator votes for "V on even lines" or "V on odd
         * lines", and every line follows the majority. */
        uint32_t p = q16((float)(t + 0.1005 * Fs)), p1 = q16((float)(t + 0.1040 * Fs));
        int32_t sep = 0;
        int ns = 0, phase, j, q = k % s->yc_n;
        for (; p < p1; p += 0x10000u) { sep += lvl_q(s, base, p); ns++; }
        sep = ns ? (sep / ns + 128) >> 8 : 128;
        if (!s->done[k] && (sep < 80 || sep > 175)) {       /* first drawing only: one vote a line */
            if ((sep < 80) == !(k & 1)) s->par_v_even++; else s->par_v_odd++;
        }
        phase = s->par_v_odd > s->par_v_even;
        s->ct[k] = (uint8_t)(((k + phase) & 1) ? CT_U : CT_V);
        memcpy(s->Y + q * R36_W, s->chbuf[0], R36_W);
        memcpy(s->C + q * R36_W, s->chbuf[1], R36_W);
        s->yc_line[q] = (int16_t)k;
        s->done[k] = 1;
        if (phase != s->par_phase) {                         /* the vote turned: redo every line */
            s->par_phase = phase;
            for (j = 0; j < R36_H; j++) if (s->done[j]) s->ct[j] = (uint8_t)(((j + phase) & 1) ? CT_U : CT_V);
            for (j = 0; j < R36_H; j++) compose36(s, j);
        } else {
            compose36(s, k - 1); compose36(s, k); compose36(s, k + 1);
        }
    } else {
        const uint8_t *Y = 0, *Y2 = 0, *V = 0, *U = 0, *R = 0, *G = 0, *B = 0;
        for (c = 0; c < m->nch; c++) {
            const uint8_t *b = s->chbuf[c];
            switch (m->ch[c].what) {
            case CH_Y: Y = b; break;  case CH_Y2: Y2 = b; break;
            case CH_V: V = b; break;  case CH_U: U = b; break;
            case CH_R: R = b; break;  case CH_G: G = b; break;  case CH_B: B = b; break;
            default: break;
            }
        }
        if (m->color == CM_RGB) {
            for (i = 0; i < w; i++) s->row[i] = rgb565(R[i], G[i], B[i]);
            emit_row(s, k, s->row, w);
        } else {
            for (i = 0; i < w; i++) s->row[i] = yuv565(Y[i], V[i], U[i]);
            emit_row(s, k * m->rows, s->row, w);
            if (m->rows == 2) {
                for (i = 0; i < w; i++) s->row[i] = yuv565(Y2[i], V[i], U[i]);
                emit_row(s, k * m->rows + 1, s->row, w);
            }
        }
        s->done[k] = 1;
    }
    s->tused[k] = t_model;
}

static double line_t(const SstvRx *s, int k) { return s->t_ref + s->fa + s->fb * k; }

/* ---------------------------------------------------------------- timing */
/* Matched template over the sync bits, anchored on the sync's TRAILING edge
 * (always 1200 -> 1500 Hz, so the 1350 Hz threshold sits exactly mid-step):
 * +1 for sync during the last La samples of the pulse, -1 for sync during the
 * porch after it. The leading edge comes from whatever picture ended the
 * previous line, so the threshold crossing there is late by a content-
 * dependent amount; using it biased timing by ~half a pixel. Unique peak for
 * line 0 too (its sync runs on from the 1200 Hz VIS stop bit). Returns the
 * sync START (edge - sync length). */
static float find_sync(const SstvRx *s, double centre, double W, double *pos)
{
    long a = (long)(centre - W), b = (long)(centre + W), t;
    int Ls = s->Ls, La = s->La, Lp = s->Lp, A = 0, B = 0, i, best = -0x7fffffff;
    long t1 = 0, t2 = 0;
    if (a < 0) a = 0;
    if (b < a) b = a;
    for (i = Ls - La; i < Ls; i++) A += sbit(s, (uint32_t)(a + i));
    for (i = Ls; i < Ls + Lp; i++) B += sbit(s, (uint32_t)(a + i));
    for (t = a; t <= b; t++) {
        int C = (2 * A - La) - (2 * B - Lp);
        if (C > best) { best = C; t1 = t2 = t; }
        else if (C == best && t == t2 + 1) t2 = t;
        A += sbit(s, (uint32_t)(t + Ls)) - sbit(s, (uint32_t)(t + Ls - La));
        B += sbit(s, (uint32_t)(t + Ls + Lp)) - sbit(s, (uint32_t)(t + Ls));
    }
    *pos = 0.5 * (double)(t1 + t2);
    return (float)best / (float)(La + Lp);
}

static void sort_d(double *v, int n)
{
    int i, j;
    for (i = 1; i < n; i++) { double x = v[i]; for (j = i; j > 0 && v[j - 1] > x; j--) v[j] = v[j - 1]; v[j] = x; }
}

/* Fit sync starts (k, mpos) for the good periods so far: offset only (prior
 * period) until there are enough, then least squares with outlier rejection.
 * Result in fa, fb. */
static void fit_model(SstvRx *s)
{
    static double r[MAXP];
    static uint8_t use[MAXP];
    double Pmin = s->Pnom * (1 - MAX_SKEW), Pmax = s->Pnom * (1 + MAX_SKEW);
    double a0, thr, b;
    int k, n = 0, kmin = MAXP, kmax = -1, pass;
    for (k = 0; k <= s->k && k < MAXP; k++)
        if (s->mgood[k]) { r[n++] = s->mpos[k] - s->Pprior * k; if (k < kmin) kmin = k; if (k > kmax) kmax = k; }
    s->fit_n = n; s->fit_span = n ? kmax - kmin : 0; s->fit_full = 0;
    if (s->slant_fixed) {                             /* darkroom: the period is given, fit the phase */
        b = s->Pnom * (1.0 + s->slant_ppm * 1e-6);
        s->fb = b;
        if (n == 0) return;
        for (k = 0, n = 0; k <= s->k && k < MAXP; k++)
            if (s->mgood[k]) r[n++] = s->mpos[k] - b * k;
        sort_d(r, n);
        s->fa = (n & 1) ? r[n / 2] : 0.5 * (r[n / 2 - 1] + r[n / 2]);
        s->fit_full = (s->fit_span >= 8);
        return;
    }
    if (n == 0) return;                               /* keep the start guess */
    if (n < 4 || s->fit_span < 4) {                   /* too few: prior period, median offset */
        sort_d(r, n);
        a0 = (n & 1) ? r[n / 2] : 0.5 * (r[n / 2 - 1] + r[n / 2]);
        s->fa = a0; s->fb = s->Pprior;
        return;
    }
    thr = 1e30;                                       /* first pass: every good sync */
    for (pass = 0; pass < 3; pass++) {
        double sk = 0, sm = 0, skk = 0, skm = 0, rr = 0, a;
        int mcount = 0;
        for (k = 0; k <= s->k && k < MAXP; k++) {
            double e;
            use[k] = 0;
            if (!s->mgood[k]) continue;
            e = s->mpos[k] - (pass ? s->fa + s->fb * k : 0.0);
            if (pass && (e > thr || e < -thr)) continue;
            use[k] = 1;
            sk += k; sm += s->mpos[k]; skk += (double)k * k; skm += k * s->mpos[k]; mcount++;
        }
        if (mcount < 4) return;
        {
            double den = mcount * skk - sk * sk;
            if (den <= 0) return;
            b = (mcount * skm - sk * sm) / den;
        }
        if (b < Pmin || b > Pmax) return;             /* nonsense: keep what we had */
        a = (sm - b * sk) / mcount;
        for (k = 0; k <= s->k && k < MAXP; k++) {
            double e;
            if (!use[k]) continue;
            e = s->mpos[k] - (a + b * k);
            rr += e * e;
        }
        s->fa = a; s->fb = b; s->fit_full = (s->fit_span >= 8);
        rr = mcount > 2 ? rr / mcount : 0;            /* mean square residual */
        /* next pass keeps |e| <= max(3 sigma, 0.6 ms), capped at 3 ms; sqrt by bisection */
        thr = 0.0006 * s->Fs;
        if (rr * 9.0 > thr * thr) {
            double lo = thr, hi = 0.003 * s->Fs;
            int it;
            for (it = 0; it < 24; it++) { double mid = 0.5 * (lo + hi); if (mid * mid < 9.0 * rr) lo = mid; else hi = mid; }
            thr = hi;
        }
    }
}

static double search_window(const SstvRx *s)
{
    if (s->fit_n < 4 || s->miss_run >= 3) return 0.015 * s->Fs;
    if (!s->fit_full || s->fit_span < 20) return 0.006 * s->Fs;
    return 0.004 * s->Fs;
}

/* is period j's audio still in the level ring? */
static int period_in_ring(const SstvRx *s, int j)
{
    double t0 = line_t(s, j) + mode_t_min(s->m) * s->Fs - 2.0;
    return t0 >= 0 && (double)s->n - t0 < (double)s->lv_mask - 64.0;
}

static int period_cost(const Mode *m) { return m->nch * mode_sw(m); }

static void redraw_step(SstvRx *s, int budget)
{
    int j;
    if (!s->rd_pending) return;
    for (j = s->rd_j; j < s->k && j < s->nper && budget > 0; j++) {
        if (!period_in_ring(s, j)) continue;           /* too old: keeps its timing */
        render_period(s, j, line_t(s, j));
        budget -= period_cost(s->m);
    }
    s->rd_j = j;
    if (j >= s->k || j >= s->nper) s->rd_pending = 0;
    s->revision++;
}

static void finish(SstvRx *s, int why)
{
    redraw_step(s, 1 << 30);                        /* the saved picture is fully re-timed */
    s->state = SSTV_IDLE;
    s->end_kind = why;
    s->t_end = s->n;
    /* forget this picture's sync pulses: autosync would otherwise find its
     * last lines' grid and start a "new" picture on them at once */
    s->npulses = 0; s->cand = 0;
    s->can_resume = (why == SSTV_END_LOST || why == SSTV_END_ABORT) && s->k * 6 < s->nper * 5 && s->fit_n >= 4;
    if (s->k * 6 >= s->nper * 5) s->images++;
    if (s->k * 6 >= s->nper * 5 && s->fit_full && s->fit_n * 4 >= s->nper && s->fit_span * 2 >= s->nper) {
        double ppm = (s->fb / s->Pnom - 1.0) * 1e6;
        s->ppm_prior = iround(ppm);
    }
    s->revision++;
}

/* (Re)quantise [from, n) with the current offset, from the raw tone history. */
static void requant(SstvRx *s, double from)
{
    uint32_t i, i0;
    if (from < 0) from = 0;
    i0 = (uint32_t)from;
    if (s->n - i0 > s->fh_mask) i0 = s->n - s->fh_mask;
    for (i = i0; i != s->n; i++) put_tone(s, i, fh_q(s, i));
}

/* period k can be processed once its audio (plus the sync search's reach)
 * is in: kept as one integer, recomputed whenever k or the fit changes */
static void update_due(SstvRx *s)
{
    double thr;
    if (s->state != SSTV_RX) return;
    thr = line_t(s, s->k) + mode_t_max(s->m) * s->fb / s->m->line + 2.0 * search_window(s) + 8.0;
    s->due = thr < 0 ? 0u : (uint32_t)thr + 1u;     /* n > thr  <=>  n >= floor(thr) + 1 */
}

/* Recompute the autosync running sums (after the sync bits were rewritten). */
static void autosync_resync(SstvRx *s)
{
    uint32_t i, e = s->n, w = (uint32_t)(s->gA + s->gB);
    s->runA = s->runB = 0;
    if (e < w) return;
    for (i = e - w; i < e - (uint32_t)s->gB; i++) s->runA += sbit(s, i);
    for (; i < e; i++) s->runB += sbit(s, i);
}

/* t0: sync start of period 0 (tone index) */
static void start_rx(SstvRx *s, const Mode *m, double t0, int kind, float off)
{
    int k;
    if (s->state == SSTV_RX) finish(s, SSTV_END_NEWVIS);
    s->m = m;
    if (off > 150) off = 150;
    if (off < -150) off = -150;
    s->off = off;
    s->off_q = (int32_t)iround(off * TQ);
    requant(s, t0 + mode_t_min(m) * s->Fs - 0.03 * s->Fs);
    autosync_resync(s);
    s->Ls = iround(m->sync * s->Fs);
    s->La = iround((m->sync * 0.75 < 0.006 ? m->sync * 0.75 : 0.006) * s->Fs);
    s->Lp = iround((m->porch < 0.003 ? m->porch : 0.003) * s->Fs);
    if (s->Lp < 2) s->Lp = 2;
    s->Pnom = m->line * s->Fs;
    s->Pprior = s->Pnom * (1.0 + s->ppm_prior * 1e-6);
    s->nper = mode_periods(m);
    s->state = SSTV_RX; s->start_kind = kind; s->end_kind = SSTV_END_NONE;
    s->t_ref = t0; s->fa = 0; s->fb = s->slant_fixed ? s->Pnom * (1.0 + s->slant_ppm * 1e-6) : s->Pprior;
    s->fit_n = s->fit_span = s->fit_full = 0;
    s->k = 0; s->miss_run = 0; s->sync_good = s->sync_seen = 0; s->redraws = 0;
    s->absent_run = 0; s->sp_n = s->sp_ok = 0;
    s->par_v_even = s->par_v_odd = 0; s->par_phase = 0;
    s->rd_pending = 0; s->rd_j = 0;
    for (k = 0; k < MAXP; k++) { s->mgood[k] = 0; s->done[k] = 0; }
    for (k = 0; k < s->yc_n; k++) s->yc_line[k] = -1;
    if (s->sink.begin) s->sink.begin(s->sink_user, m->name, m->w, m->h);
    s->cand = 0; s->npulses = 0;
    s->can_resume = 0; s->relocks = 0;
    s->revision++;
    update_due(s);
}

static void process_period(SstvRx *s)
{
    int k = s->k, lost, present;
    double W = search_window(s), pos, t;
    float q = find_sync(s, line_t(s, k), W, &pos);
    s->sync_seen++;
    present = s->sp_n > 0 && s->sp_ok * 100u >= s->sp_n * SP_MIN_PCT
              && !in_gap(s, line_t(s, k), line_t(s, k) + s->fb);   /* a gap holds its last tone: not picture */
    s->sp_n = s->sp_ok = 0;
    if (q >= SYNC_Q_MIN) { s->mgood[k] = 1; s->mpos[k] = pos - s->t_ref; s->sync_good++; s->miss_run = 0; s->absent_run = 0; }
    else {
        s->mgood[k] = 0; s->miss_run++;
        /* no sync, but the picture is still coming in: carry on along the
         * fitted line grid (syncs are the first thing a weak or echoing
         * signal loses). Only a missing picture counts towards "lost". */
        if (present) s->absent_run = 0; else s->absent_run++;
    }
    fit_model(s);
    t = line_t(s, k);
    /* straighten: if the fit moved, re-time the periods already drawn. Done as
     * a sweep, RD_BUDGET channel-pixels per feed call (see redraw_step),
     * restarted from the top whenever the fit moves again. */
    if (k > 0) {
        int j0 = 0;
        double d0, d1;
        while (j0 < k - 1 && !period_in_ring(s, j0)) j0++;
        d0 = line_t(s, j0) - s->tused[j0]; d1 = line_t(s, k - 1) - s->tused[k - 1];
        if (d0 > 0.5 || d0 < -0.5 || d1 > 0.5 || d1 < -0.5) {
            if (!s->rd_pending) s->redraws++;
            s->rd_pending = 1; s->rd_j = j0;
        }
    }
    render_period(s, k, t);
    s->k = k + 1;
    s->revision++;
    lost = (int)((s->start_kind == SSTV_START_VIS ? LOST_S_VIS : LOST_S_SYNC) / s->m->line);
    if (lost < 8) lost = 8;
    if (s->k >= s->nper) finish(s, SSTV_END_COMPLETE);
    else if (!s->manual && s->absent_run >= lost) finish(s, SSTV_END_LOST);
    update_due(s);
}

/* ------------------------------------------------------------------ VIS */
static float bin_mean(const SstvRx *s, uint32_t j0, int len)
{
    int32_t a = 0; int i;
    for (i = 0; i < len; i++) a += s->bins[(j0 + (uint32_t)i) & VB_MASK];
    return (float)a / (float)(len * TQ);
}

/* Hypothesis: the VIS stop bit is bins [j-29, j]. Returns 1 and sets code/off/err.
 * Tolerances are set for ~8 dB SNR (3 kHz): noise pulls bin means toward the
 * band centre, so bits are decided against the 1200 Hz midpoint with a margin
 * rather than by distance to 1100/1300. A false start would need a 170 ms
 * steady leader (picture content can't: line syncs break the std test), plus
 * 10 matching regions and even parity. */
static int vis_test(const SstvRx *s, uint32_t j, int *code, float *off, float *err)
{
    float lead, var = 0, m, e = 0, d, mid;
    int b, bits = 0, ones = 0, i;
    if (j < 620) return 0;
    m = bin_mean(s, j - 29 + 5, 20);                   /* stop bit */
    if (m > 1440.0f || m < 960.0f) return 0;           /* cheap reject (runs every 1 ms) */
    lead = bin_mean(s, j - 480, 170);                  /* second leader, inner part */
    *off = lead - 1900.0f;
    if (*off > 150.0f || *off < -150.0f) return 0;
    mid = 1200.0f + *off;
    d = m - mid; if (d > 90.0f || d < -90.0f) return 0; e += d * d;
    m = bin_mean(s, j - 299 + 5, 20);                  /* start bit */
    d = m - mid; if (d > 90.0f || d < -90.0f) return 0; e += d * d;
    for (i = 0; i < 170; i++) { float x = (float)s->bins[(j - 480 + (uint32_t)i) & VB_MASK] * (1.0f / TQ) - lead; var += x * x; }
    if (var > 170.0f * 120.0f * 120.0f) return 0;      /* leader must be a steady tone */
    for (b = 0; b < 8; b++) {
        d = bin_mean(s, j - 29 - 30u * (uint32_t)(8 - b) + 5, 20) - mid;
        if (d < -25.0f && d > -200.0f) { bits |= 1 << b; ones++; d += 100.0f; }
        else if (d > 25.0f && d < 200.0f) d -= 100.0f;
        else return 0;
        e += d * d;
    }
    if (ones & 1) return 0;                            /* even parity over 8 bits */
    *code = bits & 0x7f;
    *err = e;
    return 1;
}

/* Were the picture in progress's syncs still coming in during [a, b]? Then a
 * "VIS" heard there is picture content that happened to look like one (a
 * grey picture holds the 1900 Hz leader tone well), not a new picture: a
 * real one comes after the old picture has stopped. */
static int syncs_during(const SstvRx *s, double a, double b)
{
    int k;
    if (s->state != SSTV_RX) return 0;
    for (k = s->k - 1; k >= 0 && k < MAXP; k--) {
        double t = s->mpos[k] + s->t_ref;
        if (!s->mgood[k]) continue;
        if (t < a) break;
        if (t <= b) return 1;
    }
    return 0;
}

static void vis_bin_done(SstvRx *s, uint32_t j)
{
    int code; float off, err;
    if (vis_test(s, j, &code, &off, &err)) {
        if (!s->vis_active) { s->vis_active = 1; s->vis_first = j; s->vis_deadline = j + 14; s->vis_code = code; s->vis_off = off; }
        if (code == s->vis_code) s->vis_last = j;
    }
    if (s->vis_active && j >= s->vis_deadline) {
        uint32_t jc = s->vis_first + (s->vis_last - s->vis_first) / 2;
        double t_end = (double)(uint32_t)((jc + 1) * (s->Fs / 1000.0) + 0.5);  /* end of the stop bit */
        const Mode *m = mode_by_vis(s->vis_code);
        if (m && s->lock_vis >= 0 && m->vis != s->lock_vis) m = 0;   /* not the chosen mode */
        s->vis_active = 0;
        s->last_vis = s->vis_code;
        s->last_vis_ok = m != 0;
        s->revision++;
        if (m && !s->manual && !(s->state == SSTV_RX && t_end < s->t_ref + 1.0 * s->Fs) && !syncs_during(s, t_end - 0.8 * s->Fs, t_end))
            start_rx(s, m, t_end + m->first * s->Fs * (1.0 + s->ppm_prior * 1e-6), SSTV_START_VIS, s->vis_off);
    }
}

/* ------------------------------------------------------------- autosync */
static float fh_mean(const SstvRx *s, double from, double to)
{
    double a = 0; uint32_t i, i0 = (uint32_t)from, i1 = (uint32_t)to; int c = 0;
    for (i = i0; i < i1; i++) { a += fh_at(s, i); c++; }
    return c ? (float)(a / c) : 0.0f;
}
/* Does the period whose sync starts at p have the mode's landmarks (porch /
 * separator tones)? -1 if its audio is no longer in the tone history. */
static int line_shape(const SstvRx *s, const Mode *m, double p, float off)
{
    int i;
    double Fs = s->Fs;
    if ((double)s->n - (p + m->shape[0].a * Fs) > (double)s->fh_mask - 16.0) return -1;
    for (i = 0; i < m->nshape; i++) {
        float f = fh_mean(s, p + m->shape[i].a * Fs, p + m->shape[i].b * Fs) - off;
        int ok = f > m->shape[i].f1 - 130.0f && f < m->shape[i].f1 + 130.0f;
        if (m->shape[i].f2 > 0) ok |= f > m->shape[i].f2 - 130.0f && f < m->shape[i].f2 + 130.0f;
        if (!ok) return 0;
    }
    return 1;
}

static int sbits(const SstvRx *s, uint32_t a, uint32_t b)       /* sync bits in [a, b) */
{
    int c = 0;
    for (; a < b; a++) c += sbit(s, a);
    return c;
}
/* Is the pulse ending at `edge` this mode's sync? Mostly sync for the mode's
 * sync length, mostly not just before it (so a 20 ms PD sync isn't taken for
 * a 9 ms one, nor a 9 ms one for Martin's 4.9 ms). Counts, not a contiguous
 * run, so noise doesn't break it. */
static int sync_len_ok(const SstvRx *s, const Mode *m, uint32_t edge)
{
    uint32_t Ls = (uint32_t)iround(m->sync * s->Fs), g = (uint32_t)iround(0.0005 * s->Fs);
    uint32_t pre = (uint32_t)iround((m->sync * 0.4 < 0.003 ? m->sync * 0.4 : 0.003) * s->Fs);
    uint32_t in0 = edge - Ls + Ls / 10u, in1 = edge - Ls / 10u;
    if (edge < Ls + g + pre + 1u) return 0;
    if (sbits(s, in0, in1) * 100 < (int)(in1 - in0) * SYNC_LEN_PCT) return 0;
    return sbits(s, edge - Ls - g - pre, edge - Ls - g) * 10 <= (int)pre * 3;
}

/* The last picture ended early (a fade longer than the lost timeout, or a
 * long gap) and syncs of the same mode are back: if the sync at t0 sits on
 * that picture's fitted line grid, carry on with the same picture from that
 * line instead of starting a new one at the top. 1 if resumed. */
static int try_resume(SstvRx *s, const Mode *m, double t0)
{
    double x, e;
    int j;
    if (!s->can_resume || s->m != m || s->state != SSTV_IDLE) return 0;
    if ((double)(s->n - s->t_end) > RESUME_S * s->Fs) { s->can_resume = 0; return 0; }
    x = (t0 - s->t_ref - s->fa) / s->fb;
    j = (int)(x + 0.5);
    e = (x - j) * s->fb; if (e < 0) e = -e;
    /* on the grid to 4 ms plus 2 ms per minute of extrapolation (fit error) */
    if (e > 0.004 * s->Fs + 0.002 * s->Fs * (t0 - s->t_ref) / (60.0 * s->Fs)) return 0;
    if (j < 0 || j >= s->nper) return 0;
    if (j < s->k) j = s->k;                         /* grid found from older pulses: go on from here */
    s->state = SSTV_RX; s->end_kind = SSTV_END_NONE;
    s->k = j; s->miss_run = 0; s->can_resume = 0;
    s->rd_pending = 0;
    s->relocks++;
    s->cand = 0; s->npulses = 0;
    s->revision++;
    update_due(s);
    return 1;
}

/* A new pulse arrived: does it and its predecessors form some mode's grid? */
static void autosync_try(SstvRx *s)
{
    int mi;
    const Pulse *np = &s->pulses[s->npulses - 1];
    for (mi = 0; mi < NMODES; mi++) {
        const Mode *m = &MODES[mi];
        double Ls = m->sync * s->Fs, P = m->line * s->Fs * (1.0 + s->ppm_prior * 1e-6);
        int q, on = 1, first = s->npulses - 1, need = m->nshape ? 4 : 5;
        if (s->lock_vis >= 0 && m->vis != s->lock_vis) continue;   /* not the chosen mode */
        if (!sync_len_ok(s, m, np->edge)) continue;
        for (q = s->npulses - 2; q >= 0; q--) {
            const Pulse *pq = &s->pulses[q];
            double d = (double)(np->edge - pq->edge), e;
            int mul = (int)(d / P + 0.5);
            if (mul < 1 || mul > 8) continue;
            e = d - mul * P; if (e < 0) e = -e;
            if (e >= 0.001 * s->Fs + 0.0015 * d) continue;
            if (s->n - pq->edge > s->lv_mask - 64u || !sync_len_ok(s, m, pq->edge)) continue;
            on++; first = q;
        }
        if (on < need) continue;
        {
            /* tuning offset from the sync tone itself, then the landmarks of the
             * grid's periods (all but one of those still in history must pass) */
            double om[NPULSE];
            int cnt = 0, shaped = 0, tried = 0;
            uint32_t a, e0 = s->pulses[first].edge;
            float off;
            /* median over the grid's syncs of each sync's mean tone */
            for (q = first; q < s->npulses; q++) {
                uint32_t e = s->pulses[q].edge;
                double d = (double)(np->edge - e), ee = d - (int)(d / P + 0.5) * P, acc = 0;
                int c2 = 0;
                if (ee < 0) ee = -ee;
                if (ee >= 0.001 * s->Fs + 0.0015 * d || s->n - e > s->fh_mask - 16u || !sync_len_ok(s, m, e)) continue;
                for (a = e - (uint32_t)(Ls * 0.8); a < e - (uint32_t)(Ls * 0.2); a++) { acc += fh_at(s, a); c2++; }
                if (c2) om[cnt++] = acc / c2;
            }
            if (cnt) { sort_d(om, cnt); off = (float)om[cnt / 2] - 1200.0f; } else off = 0.0f;
            if (off > 150.0f || off < -150.0f) continue;
            for (q = first; m->nshape && q < s->npulses; q++) {
                double d = (double)(np->edge - s->pulses[q].edge), e = d - (int)(d / P + 0.5) * P;
                int r;
                if (e < 0) e = -e;
                if (e >= 0.001 * s->Fs + 0.0015 * d) continue;      /* not on the grid */
                if (q == s->npulses - 1 && (double)s->n < s->pulses[q].edge - Ls + m->shape[m->nshape - 1].b * s->Fs)
                    continue;                                       /* landmarks not heard yet */
                r = line_shape(s, m, (double)s->pulses[q].edge - Ls, off);
                if (r < 0) continue;
                tried++; shaped += r;
            }
            if (m->nshape && (tried < 2 || shaped < tried - 1)) continue;
            if (!try_resume(s, m, (double)e0 - Ls)) start_rx(s, m, (double)e0 - Ls, SSTV_START_SYNC, off);
            return;
        }
    }
}

static void autosync_sample(SstvRx *s)
{
    uint32_t i = s->n - 1;                             /* newest tone sample */
    int gA = s->gA, gB = s->gB, C;
    uint32_t t;
    s->runB += sbit(s, i) - sbit(s, i - (uint32_t)gB);
    s->runA += sbit(s, i - (uint32_t)gB) - sbit(s, i - (uint32_t)(gB + gA));
    if (s->n < (uint32_t)(gA + gB + 1)) return;
    t = s->n - (uint32_t)gB;                           /* candidate trailing edge */
    C = (2 * s->runA - gA) - (2 * s->runB - gB);
    if (C >= (int)(AUTO_PULSE_Q * (float)(gA + gB))) {
        if (!s->cand || C > s->candC) { s->cand = 1; s->candC = C; s->candT1 = s->candT2 = t; }
        else if (C == s->candC && t == s->candT2 + 1) s->candT2 = t;
    }
    if (s->cand && t > s->candT2 + (uint32_t)(0.003 * s->Fs)) {
        uint32_t e = s->candT1 + (s->candT2 - s->candT1) / 2;
        int q;
        s->cand = 0;
        if (s->npulses == NPULSE) { for (q = 1; q < NPULSE; q++) s->pulses[q - 1] = s->pulses[q]; s->npulses = NPULSE - 1; }
        s->pulses[s->npulses].edge = e;
        s->npulses++;
        if (s->state == SSTV_IDLE && s->autosync && !s->vis_active && s->npulses >= 4) autosync_try(s);
    }
}

/* ------------------------------------------------------------ per tone */
static void on_tone(SstvRx *s, int32_t f)          /* f: tone, 1/16 Hz */
{
    uint32_t i = s->n;
    int32_t a = s->med1, b = s->med2;
    /* clamp FM clicks, then a 3-tap median: kills single-sample spikes at low
     * SNR without blurring edges (one tone sample of delay, same everywhere) */
    if (f < HZ(800)) f = HZ(800);
    if (f > HZ(2600)) f = HZ(2600);
    s->med1 = b; s->med2 = f;
    f = (a > b) ? (b > f ? b : (a > f ? f : a)) : (a > f ? a : (b > f ? f : b));
    {   /* f is clamped to 800..2600 above: q = (f - 900) / 6, rounded, 0..255 */
        int32_t q = (f - HZ(900) + HZ(3)) / HZ(6);
        s->fh[i & s->fh_mask] = (uint8_t)(q <= 0 ? 0 : q >= 255 ? 255 : q);
    }
    put_tone(s, i, f);
    {   /* picture present? A picture's tone moves in small steps (pixels are
         * many tone samples long); noise jumps all over the band. */
        int32_t d = f - s->sp_prev;
        if (d < 0) d = -d;
        s->sp_prev = f;
        s->sp_n++;
        if (d < HZ(SP_STEP_HZ)) s->sp_ok++;
    }
    s->n = i + 1;
    s->tone_ema += (int32_t)(((int64_t)(f * 256 - s->tone_ema) * s->ema_a) >> 16);

    s->bin_acc += f; s->bin_cnt++;
    if (s->n >= s->bin_end) {
        s->bins[s->bin_j & VB_MASK] = s->bin_cnt ? (s->bin_acc + s->bin_cnt / 2) / s->bin_cnt : HZ(FC_HZ);
        s->bin_acc = 0; s->bin_cnt = 0;
        vis_bin_done(s, s->bin_j);
        s->bin_j++;
        s->bin_end = (uint32_t)((s->bin_j + 1) * (s->Fs / 1000.0) + 0.5);
    }

    autosync_sample(s);

    while (s->state == SSTV_RX && s->n >= s->due)
        process_period(s);
}

/* -------------------------------------------------------------- public */
static int gcd_i(int a, int b) { while (b) { int t = a % b; a = b; b = t; } return a; }

SstvRx *sstv_new(int in_rate)
{
    SstvRx *s;
    int i;
    if (in_rate < 8000) return 0;
    at_init();
    s = (SstvRx *)d_calloc(1, sizeof *s);
    if (!s) return 0;
    s->in_rate = in_rate;
    s->D = (in_rate + 5512) / 11025; if (s->D < 1) s->D = 1;
    s->Fs = (double)in_rate / s->D;

    s->L = (int)(FIR_TAPS_44K * (double)in_rate / 44100.0 + 0.5) | 1;
    if (s->L < 15) s->L = 15;
    s->M = (s->L - 1) / 2;

    s->fh_mask = FH_LEN - 1;
    s->lv_mask = LV_LEN - 1;
    /* Robot 36 lines that can still be re-timed: as many as the level ring
     * holds, and a few more for the partner lines around them */
    s->yc_n = (int)((double)LV_LEN / (0.150 * s->Fs)) + 5;
    if (s->yc_n > R36_H) s->yc_n = R36_H;
    s->lo_n = in_rate / gcd_i(in_rate, FC_HZ);          /* the LO's period, samples */

    s->h  = (int16_t *)d_calloc((size_t)(s->M + 1), sizeof(int16_t));
    s->lo = (int16_t *)d_calloc((size_t)(2 * s->lo_n), sizeof(int16_t));
    s->bi = (int32_t *)d_calloc((size_t)(2 * s->L), sizeof(int32_t));
    s->bq = (int32_t *)d_calloc((size_t)(2 * s->L), sizeof(int32_t));
    s->fh = (uint8_t *)d_calloc((size_t)s->fh_mask + 1, 1);
    s->lv = (uint8_t *)d_calloc((size_t)s->lv_mask + 1, 1);
    s->sb = (uint8_t *)d_calloc(((size_t)s->lv_mask + 1) / 8, 1);
    s->Y  = (uint8_t *)d_calloc((size_t)s->yc_n * 2u * R36_W, 1);
    s->yc_line = (int16_t *)d_calloc((size_t)s->yc_n, sizeof(int16_t));
    if (!s->h || !s->lo || !s->bi || !s->bq || !s->fh || !s->lv || !s->sb || !s->Y || !s->yc_line) {
        sstv_free(s); return 0;
    }
    s->C = s->Y + (size_t)s->yc_n * R36_W;
    for (i = 0; i < s->yc_n; i++) s->yc_line[i] = -1;

    /* high-pass: 4th-order Butterworth as two biquads, bilinear transform,
     * coefficients Q29 (|a1| < 2) */
    {
        static const double q4[2] = { 0.54119610, 1.30656296 }, q2[1] = { 0.70710678 };
        const double *q = HP_STAGES == 1 ? q2 : q4;
        double w = PI_D * HP_HZ / in_rate, K = m_sin(w) / m_cos(w), K2 = K * K, Q29 = 536870912.0;
        int j;
        for (j = 0; j < HP_STAGES; j++) {
            double norm = 1.0 / (1.0 + K / q[j] + K2);
            s->hp_b[j][0] = (int32_t)iround(norm * Q29);
            s->hp_b[j][1] = (int32_t)iround(-2.0 * norm * Q29);
            s->hp_b[j][2] = s->hp_b[j][0];
            s->hp_a[j][0] = (int32_t)iround(2.0 * (K2 - 1.0) * norm * Q29);
            s->hp_a[j][1] = (int32_t)iround((1.0 - K / q[j] + K2) * norm * Q29);
        }
    }

    /* mixer: e^(-j 2 pi FC n / R) repeats every lo_n samples; one exact period */
    for (i = 0; i < s->lo_n; i++) {
        double ph = -2.0 * PI_D * (double)((long)FC_HZ * i % in_rate) / in_rate;
        s->lo[2 * i]     = (int16_t)iround(m_cos(ph) * 32767.0);
        s->lo[2 * i + 1] = (int16_t)iround(m_sin(ph) * 32767.0);
    }
    /* windowed-sinc (Blackman) low-pass, unity DC gain (taps sum to exactly
     * 32768), symmetric half */
    {
        double sum = 0, wc = 2.0 * FIR_FCUT / in_rate;
        double *tmp = (double *)D_ALLOC(sizeof(double) * (size_t)s->L);
        int32_t tot = 0;
        if (!tmp) { sstv_free(s); return 0; }
        for (i = 0; i < s->L; i++) {
            double x = i - s->M, w, sn;
            w = 0.42 - 0.5 * m_cos(2 * PI_D * i / (s->L - 1)) + 0.08 * m_cos(4 * PI_D * i / (s->L - 1));
            sn = (x == 0) ? wc : m_sin(PI_D * wc * x) / (PI_D * x);
            tmp[i] = w * sn; sum += tmp[i];
        }
        for (i = 0; i <= s->M; i++) {
            s->h[i] = (int16_t)iround(tmp[i] / sum * 32768.0);
            tot += (i < s->M ? 2 : 1) * s->h[i];
        }
        s->h[s->M] = (int16_t)(s->h[s->M] + (32768 - tot));
        D_FREE(tmp);
    }
    s->tone_mul = (int64_t)(s->Fs * 4096.0 + 0.5);

    s->med1 = s->med2 = HZ(FC_HZ);
    s->tone_ema = HZ(FC_HZ) * 256; s->ema_a = (int32_t)iround(65536.0 / (0.05 * s->Fs));
    s->zr1 = 16; s->zi1 = 0;
    s->bin_j = 0; s->bin_end = (uint32_t)(s->Fs / 1000.0 + 0.5);
    s->gA = iround(0.004 * s->Fs);                         /* fits inside Martin's 4.9 ms sync */
    s->gB = iround(0.001 * s->Fs); if (s->gB < 2) s->gB = 2;
    s->autosync = 1;
    s->lock_vis = -1;
    s->m = &MODES[0];
    s->last_vis = -1;
    return s;
}

void sstv_free(SstvRx *s)
{
    if (!s) return;
    D_FREE(s->yc_line); D_FREE(s->Y); D_FREE(s->sb); D_FREE(s->lv); D_FREE(s->fh);
    D_FREE(s->bq); D_FREE(s->bi); D_FREE(s->lo); D_FREE(s->h);
    D_FREE(s);
}

void sstv_set_sink(SstvRx *s, const SstvSink *sink, void *user)
{
    if (!s) return;
    if (sink) s->sink = *sink; else memset(&s->sink, 0, sizeof s->sink);
    s->sink_user = user;
}

void sstv_feed(SstvRx *s, const int16_t *x, int n, int stride)
{
    int L, M, D, i, k, j;
    const int16_t *h;
    if (!s || !x || n <= 0) return;
    L = s->L; M = s->M; D = s->D; h = s->h;
    if (stride < 1) stride = 1;
    for (i = 0; i < n; i++) {
        int32_t v = (int32_t)x[(long)i * stride] * 256, re, im;   /* Q8 */
        int p = s->bpos;
        for (j = 0; j < HP_STAGES; j++) {             /* high-pass, direct form I */
            int64_t acc = (int64_t)s->hp_b[j][0] * v + (int64_t)s->hp_b[j][1] * s->hp_x[j][0]
                        + (int64_t)s->hp_b[j][2] * s->hp_x[j][1]
                        - (int64_t)s->hp_a[j][0] * s->hp_y[j][0] - (int64_t)s->hp_a[j][1] * s->hp_y[j][1];
            int32_t y = (int32_t)((acc + (1 << 28)) >> 29);
            s->hp_x[j][1] = s->hp_x[j][0]; s->hp_x[j][0] = v;
            s->hp_y[j][1] = s->hp_y[j][0]; s->hp_y[j][0] = y;
            v = y;
        }
        v >>= 4;                                      /* Q4 */
        {
            const int16_t *lo = s->lo + 2 * s->lo_i;
            re = (int32_t)(((int64_t)v * lo[0]) >> 15);
            im = (int32_t)(((int64_t)v * lo[1]) >> 15);
            if (++s->lo_i == s->lo_n) s->lo_i = 0;
        }
        s->bi[p] = s->bi[p + L] = re;
        s->bq[p] = s->bq[p + L] = im;
        if (++p == L) p = 0;
        s->bpos = p;
        if (++s->dcount == D) {
            const int32_t *wi = s->bi + p, *wq = s->bq + p;   /* oldest .. newest */
            int64_t ar = (int64_t)h[M] * wi[M], ai = (int64_t)h[M] * wq[M], dr, di;
            int32_t zr, zi, ang;
            s->dcount = 0;
            for (k = 0; k < M; k++) {                 /* symmetric taps */
                ar += (int64_t)h[k] * (wi[k] + wi[L - 1 - k]);
                ai += (int64_t)h[k] * (wq[k] + wq[L - 1 - k]);
            }
            zr = (int32_t)(ar >> 15); zi = (int32_t)(ai >> 15);
            dr = (int64_t)zr * s->zr1 + (int64_t)zi * s->zi1;
            di = (int64_t)zi * s->zr1 - (int64_t)zr * s->zi1;
            s->zr1 = zr; s->zi1 = zi;
            ang = i_atan2(di, dr);                    /* 2^-24 turns per tone sample */
            on_tone(s, HZ(FC_HZ) + (int32_t)(((int64_t)ang * s->tone_mul) >> 32));
        }
    }
    if (s->state == SSTV_RX) redraw_step(s, RD_BUDGET);
}

void sstv_gap(SstvRx *s, int n)
{
    uint32_t t, g0;
    int k;
    if (!s || n <= 0) return;
    s->gaps += (unsigned long)n;
    t = (uint32_t)((s->dcount + n) / s->D);        /* tone samples the gap spans */
    s->dcount = (s->dcount + n) % s->D;
    s->lo_i = (int)(((long)s->lo_i + n) % s->lo_n); /* the LO runs on through the gap */
    if (t == 0) return;
    g0 = s->n;
    s->gap0[s->gapi] = g0; s->gap1[s->gapi] = g0 + t + 8u;   /* + the filter's settling */
    s->gapi = (s->gapi + 1) % NGAPS;
    for (k = 0; k < 2 * s->L; k++) s->bi[k] = s->bq[k] = 0;   /* restart the filters clean */
    memset(s->hp_x, 0, sizeof s->hp_x); memset(s->hp_y, 0, sizeof s->hp_y);
    s->zr1 = 16; s->zi1 = 0;
    {   /* hold the last tone (never a sync tone, so no false syncs) */
        int32_t f = s->med2 < HZ(1450) ? HZ(1500) : s->med2;
        while (t--) on_tone(s, f);
    }
    if (s->state == SSTV_RX) redraw_step(s, RD_BUDGET);
}

void sstv_status(const SstvRx *s, SstvStatus *st)
{
    double ppm;
    memset(st, 0, sizeof *st);
    st->last_vis = -1;
    if (!s) return;
    st->state = s->state;
    if (s->start_kind != SSTV_START_NONE) {
        st->mode = s->m->name; st->width = s->m->w; st->height = s->m->h;   /* as sent */
        st->line = s->k * s->m->rows;
    }
    st->start_kind = s->start_kind;
    st->end_kind = s->end_kind;
    st->last_vis = s->last_vis;
    st->last_vis_ok = s->last_vis_ok;
    st->tone_hz = (int)((s->tone_ema + 2048) >> 12);
    st->offset_hz = iround(s->off);
    if (s->start_kind != SSTV_START_NONE && s->fit_full && s->fit_span >= 40) {
        ppm = (s->fb / s->Pnom - 1.0) * 1e6;
        st->ppm_valid = 1;
    } else ppm = s->ppm_prior;
    st->ppm = iround(ppm);
    st->sync_good = s->sync_good;
    st->sync_seen = s->sync_seen;
    st->slant_redraws = s->redraws;
    st->relocks = s->relocks;
    st->images = s->images;
    st->gaps = s->gaps;
    st->revision = s->revision;
    if (s->start_kind != SSTV_START_NONE) {
        st->start_s = (s->t_ref + s->fa) / s->Fs;
        st->ppm_fit = (s->fb / s->Pnom - 1.0) * 1e6;
    }
}

void sstv_abort(SstvRx *s) { if (s && s->state == SSTV_RX) finish(s, SSTV_END_ABORT); }

void sstv_clear(SstvRx *s)
{
    int k;
    if (!s) return;
    if (s->state == SSTV_RX) finish(s, SSTV_END_ABORT);
    for (k = 0; k < MAXP; k++) s->done[k] = 0;
    for (k = 0; k < s->yc_n; k++) s->yc_line[k] = -1;
    s->k = 0;
    s->can_resume = 0;
    s->revision++;
}

void sstv_set_autosync(SstvRx *s, int on) { if (s) s->autosync = on ? 1 : 0; }
void sstv_set_mode(SstvRx *s, int vis) { if (s) s->lock_vis = mode_by_vis(vis) ? vis : -1; }

void sstv_set_slant(SstvRx *s, int fixed, double ppm)
{
    if (!s) return;
    if (ppm > 20000) ppm = 20000;
    if (ppm < -20000) ppm = -20000;
    s->slant_fixed = fixed ? 1 : 0;
    s->slant_ppm = ppm;
}

void sstv_set_shift(SstvRx *s, double ms) { if (s) s->shift_ms = ms; }

int sstv_start_at(SstvRx *s, int vis, double seconds, double offset_hz)
{
    const Mode *m = mode_by_vis(vis);
    if (!s || !m || seconds < 0) return 0;
    s->manual = 1;
    s->autosync = 0;
    start_rx(s, m, seconds * s->Fs, SSTV_START_SYNC, (float)offset_hz);
    return 1;
}

void sstv_set_ppm_prior(SstvRx *s, int ppm)
{
    if (!s) return;
    if (ppm > 20000) ppm = 20000;
    if (ppm < -20000) ppm = -20000;
    s->ppm_prior = ppm;
}

const char *sstv_vis_name(int code)
{
    const Mode *m = mode_by_vis(code);
    if (m) return m->name;
    switch (code) {
    case 0:  return "Robot 12";
    case 4:  return "Robot 24";
    default: return 0;
    }
}

int sstv_vis_supported(int code) { return mode_by_vis(code) != 0; }
