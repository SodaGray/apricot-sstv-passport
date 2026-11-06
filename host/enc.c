/* SPDX-License-Identifier: GPL-3.0-only */
/* enc.c -- see enc.h. Host only (uses libm). */
#include "enc.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct { double dur, f; } Seg;
typedef struct { Seg *v; long n, cap; } SegList;

static void push(SegList *l, double dur, double f)
{
    if (l->n == l->cap) { l->cap = l->cap ? l->cap * 2 : 4096; l->v = realloc(l->v, (size_t)l->cap * sizeof *l->v); }
    l->v[l->n].dur = dur; l->v[l->n].f = f; l->n++;
}
static double clampd(double x) { return x < 0 ? 0 : x > 255 ? 255 : x; }
static double tone(double v) { return 1500.0 + clampd(v) * (800.0 / 255.0); }

enum { F_R36, F_R72, F_MARTIN, F_SCOTTIE, F_PD };
typedef struct { int vis; const char *name; int fam, w, h; double pt; } EncMode;   /* pt: pixel time, s */
static const EncMode EM[] = {
    { 8,  "Robot 36",   F_R36,     320, 240, 0 },
    { 12, "Robot 72",   F_R72,     320, 240, 0 },
    { 44, "Martin M1",  F_MARTIN,  320, 256, 0.0004576 },
    { 40, "Martin M2",  F_MARTIN,  320, 256, 0.0002288 },
    { 60, "Scottie S1", F_SCOTTIE, 320, 256, 0.000432 },
    { 56, "Scottie S2", F_SCOTTIE, 320, 256, 0.0002752 },
    { 76, "Scottie DX", F_SCOTTIE, 320, 256, 0.00108 },
    { 93, "PD50",       F_PD,      320, 256, 0.000286 },
    { 99, "PD90",       F_PD,      320, 256, 0.000532 },
    { 95, "PD120",      F_PD,      640, 496, 0.00019 },
    { 98, "PD160",      F_PD,      512, 400, 0.000382 },
    { 96, "PD180",      F_PD,      640, 496, 0.000286 },
    { 97, "PD240",      F_PD,      640, 496, 0.000382 },
    { 94, "PD290",      F_PD,      800, 616, 0.000286 },
};
static const EncMode *em(int vis)
{
    unsigned i;
    for (i = 0; i < sizeof EM / sizeof EM[0]; i++) if (EM[i].vis == vis) return &EM[i];
    return 0;
}
int enc_mode_size(int mode, int *w, int *h)
{
    const EncMode *m = em(mode);
    if (!m) return 0;
    *w = m->w; *h = m->h;
    return 1;
}
const char *enc_mode_name(int mode) { const EncMode *m = em(mode); return m ? m->name : "?"; }

void enc_default_opts(EncOpts *o)
{
    memset(o, 0, sizeof *o);
    o->mode = 8; o->rate = 44100; o->snr_db = 100; o->amplitude = 12000; o->vis_code = 8;
    o->lead_s = 0.5; o->tail_s = 0.5; o->seed = 1;
}

static void yuv(const uint8_t *p, double *y, double *u, double *v)
{
    double r = p[0], g = p[1], b = p[2];
    *y = 0.299 * r + 0.587 * g + 0.114 * b;
    *v = 128.0 + 0.713 * (r - *y);
    *u = 128.0 + 0.564 * (b - *y);
}

static uint8_t q8(double x) { return (uint8_t)clampd(floor(x + 0.5)); }
static void yuv_to(uint8_t *o, double y, double u, double v)
{
    y = floor(clampd(y) + 0.5); u = floor(clampd(u) + 0.5) - 128; v = floor(clampd(v) + 0.5) - 128;
    o[0] = q8(y + 1.402 * v); o[1] = q8(y - 0.344 * u - 0.714 * v); o[2] = q8(y + 1.772 * u);
}

/* Robot 36: chroma of the 2x2 block */
static void block_uv(const uint8_t *rgb, int W, int x2, int j2, double *u, double *v)
{
    double su = 0, sv = 0, y, uu, vv; int dx, dy;
    for (dy = 0; dy < 2; dy++) for (dx = 0; dx < 2; dx++) {
        yuv(rgb + ((j2 * 2 + dy) * W + x2 * 2 + dx) * 3, &y, &uu, &vv);
        su += uu; sv += vv;
    }
    *u = su / 4; *v = sv / 4;
}
/* PD: chroma of the vertical pair at column x */
static void pair_uv(const uint8_t *rgb, int W, int x, int k, double *u, double *v)
{
    double y, u0, v0, u1, v1;
    yuv(rgb + ((2 * k) * W + x) * 3, &y, &u0, &v0);
    yuv(rgb + ((2 * k + 1) * W + x) * 3, &y, &u1, &v1);
    *u = (u0 + u1) / 2; *v = (v0 + v1) / 2;
}

void enc_reference(int mode, const uint8_t *rgb, uint8_t *out)
{
    const EncMode *m = em(mode);
    int x, j, W, H;
    if (!m) return;
    W = m->w; H = m->h;
    for (j = 0; j < H; j++) for (x = 0; x < W; x++) {
        const uint8_t *p = rgb + (j * W + x) * 3;
        uint8_t *o = out + (j * W + x) * 3;
        double y, u, v;
        switch (m->fam) {
        case F_MARTIN: case F_SCOTTIE: o[0] = p[0]; o[1] = p[1]; o[2] = p[2]; break;
        case F_R72: yuv(p, &y, &u, &v); yuv_to(o, y, u, v); break;
        case F_PD: yuv(p, &y, &u, &v); pair_uv(rgb, W, x, j / 2, &u, &v); yuv_to(o, y, u, v); break;
        default: yuv(p, &y, &u, &v); block_uv(rgb, W, x / 2, j / 2, &u, &v); yuv_to(o, y, u, v); break;
        }
    }
}

static double gauss(unsigned *st)
{
    double a, b;
    *st = *st * 1103515245u + 12345u; a = ((*st >> 8) + 1.0) / 16777217.0;
    *st = *st * 1103515245u + 12345u; b = ((*st >> 8) + 1.0) / 16777217.0;
    return sqrt(-2.0 * log(a)) * cos(2 * M_PI * b);
}

static void scan(SegList *L, const uint8_t *rgb, int W, int row, int ch, double len)
{
    int x;
    for (x = 0; x < W; x++) push(L, len / W, tone(rgb[(row * W + x) * 3 + ch]));
}

int16_t *enc_encode(const uint8_t *rgb, const EncOpts *o, long *nout)
{
    SegList L = { 0, 0, 0 };
    const EncMode *m = em(o->mode);
    long i, n, si;
    int j, x, b, W, H, periods;
    double t, seg_end, ph = 0, sig, k = 1.0 + o->skew_ppm * 1e-6, A = o->amplitude;
    int16_t *out;
    unsigned st = o->seed ? o->seed : 1;
    if (!m) { *nout = 0; return 0; }
    W = m->w; H = m->h;
    periods = m->fam == F_PD ? H / 2 : H;
    if (o->lines > 0 && o->lines < periods) periods = o->lines;

    push(&L, o->lead_s, -1);                          /* -1 = silence */
    if (o->vis_code >= 0) {
        int ones = 0;
        push(&L, 0.300, 1900); push(&L, 0.010, 1200); push(&L, 0.300, 1900);
        push(&L, 0.030, 1200);
        for (b = 0; b < 7; b++) { int bit = (o->vis_code >> b) & 1; ones += bit; push(&L, 0.030, bit ? 1100 : 1300); }
        push(&L, 0.030, (ones & 1) ? 1100 : 1300);    /* even parity */
        push(&L, 0.030, 1200);
    }
    if (m->fam == F_SCOTTIE) push(&L, 0.009, 1200);  /* Scottie's starting sync */
    for (j = 0; j < periods; j++) {
        double y, u, v, cu, cv;
        switch (m->fam) {
        case F_R36:
            push(&L, 0.009, 1200); push(&L, 0.003, 1500);
            for (x = 0; x < W; x++) { yuv(rgb + (j * W + x) * 3, &y, &u, &v); push(&L, 0.088 / W, tone(y)); }
            push(&L, 0.0045, (j & 1) ? 2300 : 1500); push(&L, 0.0015, 1900);
            for (x = 0; x < W / 2; x++) {
                block_uv(rgb, W, x, j / 2, &cu, &cv);
                push(&L, 0.044 / (W / 2), tone((j & 1) ? cu : cv));
            }
            break;
        case F_R72:
            push(&L, 0.009, 1200); push(&L, 0.003, 1500);
            for (x = 0; x < W; x++) { yuv(rgb + (j * W + x) * 3, &y, &u, &v); push(&L, 0.138 / W, tone(y)); }
            push(&L, 0.0045, 1500); push(&L, 0.0015, 1900);
            for (x = 0; x < W; x++) { yuv(rgb + (j * W + x) * 3, &y, &u, &v); push(&L, 0.069 / W, tone(v)); }
            push(&L, 0.0045, 2300); push(&L, 0.0015, 1900);
            for (x = 0; x < W; x++) { yuv(rgb + (j * W + x) * 3, &y, &u, &v); push(&L, 0.069 / W, tone(u)); }
            break;
        case F_MARTIN:
            push(&L, 0.004862, 1200); push(&L, 0.000572, 1500);
            scan(&L, rgb, W, j, 1, m->pt * W); push(&L, 0.000572, 1500);
            scan(&L, rgb, W, j, 2, m->pt * W); push(&L, 0.000572, 1500);
            scan(&L, rgb, W, j, 0, m->pt * W); push(&L, 0.000572, 1500);
            break;
        case F_SCOTTIE:
            push(&L, 0.0015, 1500); scan(&L, rgb, W, j, 1, m->pt * W);
            push(&L, 0.0015, 1500); scan(&L, rgb, W, j, 2, m->pt * W);
            push(&L, 0.009, 1200); push(&L, 0.0015, 1500);
            scan(&L, rgb, W, j, 0, m->pt * W);
            break;
        case F_PD:
            push(&L, 0.020, 1200); push(&L, 0.00208, 1500);
            for (x = 0; x < W; x++) { yuv(rgb + ((2 * j) * W + x) * 3, &y, &u, &v); push(&L, m->pt, tone(y)); }
            for (x = 0; x < W; x++) { pair_uv(rgb, W, x, j, &cu, &cv); push(&L, m->pt, tone(cv)); }
            for (x = 0; x < W; x++) { pair_uv(rgb, W, x, j, &cu, &cv); push(&L, m->pt, tone(cu)); }
            for (x = 0; x < W; x++) { yuv(rgb + ((2 * j + 1) * W + x) * 3, &y, &u, &v); push(&L, m->pt, tone(y)); }
            break;
        }
    }
    push(&L, o->tail_s, -1);

    t = 0; for (i = 0; i < L.n; i++) t += L.v[i].dur;
    n = (long)(t * k * o->rate) + 1;
    out = malloc((size_t)n * sizeof *out);
    sig = A * A / 2 / pow(10.0, o->snr_db / 10.0) * (o->rate / 2.0) / 3000.0;
    sig = o->snr_db >= 99 ? 0 : sqrt(sig);
    si = 0; seg_end = L.v[0].dur * k;
    for (i = 0; i < n; i++) {
        double tt = (double)i / o->rate, f, a, s;
        while (si < L.n - 1 && tt >= seg_end) { si++; seg_end += L.v[si].dur * k; }
        f = L.v[si].f;
        if (f > 0) { ph += 2 * M_PI * (f + o->offset_hz) / o->rate; if (ph > 2 * M_PI) ph -= 2 * M_PI; }
        a = (f > 0) ? A : 0;
        if (o->fade_hz > 0) a *= 1.0 - o->fade_depth * 0.5 * (1 + sin(2 * M_PI * o->fade_hz * tt));
        s = a * sin(ph) + sig * gauss(&st);
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        out[i] = (int16_t)lrint(s);
    }
    free(L.v);
    *nout = n;
    return out;
}

int16_t *enc_concat(int16_t *a, long na, const int16_t *b, long nb, long *n)
{
    a = realloc(a, (size_t)(na + nb) * sizeof *a);
    memcpy(a + na, b, (size_t)nb * sizeof *b);
    *n = na + nb;
    return a;
}
