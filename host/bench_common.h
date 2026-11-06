/* SPDX-License-Identifier: GPL-3.0-only */
/* bench_common.h -- shared bits of the host benches: a synthetic test
 * picture, the cases, and luma PSNR against what an ideal decoder gives. */
#ifndef BENCH_COMMON_H
#define BENCH_COMMON_H
#include "enc.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* colour bars on top, a grey ramp, a colour ramp, and a soft pattern */
static inline uint8_t *bench_picture(int w, int h)
{
    uint8_t *p = (uint8_t *)malloc((size_t)w * h * 3);
    static const uint8_t bars[8][3] = { {255,255,255},{255,255,0},{0,255,255},{0,255,0},
                                        {255,0,255},{255,0,0},{0,0,255},{0,0,0} };
    int x, y;
    for (y = 0; y < h; y++) for (x = 0; x < w; x++) {
        uint8_t *o = p + ((size_t)y * w + x) * 3;
        if (y < h / 4) { const uint8_t *b = bars[x * 8 / w]; o[0] = b[0]; o[1] = b[1]; o[2] = b[2]; }
        else if (y < h / 2) { o[0] = o[1] = o[2] = (uint8_t)(x * 255 / (w - 1)); }
        else if (y < 3 * h / 4) {
            o[0] = (uint8_t)(x * 255 / (w - 1)); o[1] = (uint8_t)(255 - x * 255 / (w - 1));
            o[2] = (uint8_t)(y * 255 / (h - 1));
        } else {
            double v = 0.5 + 0.5 * sin(x * 0.05) * cos(y * 0.07);
            o[0] = (uint8_t)(v * 255); o[1] = (uint8_t)(128 + 100 * sin(x * 0.02));
            o[2] = (uint8_t)(255 - v * 200);
        }
    }
    return p;
}

typedef struct {
    const char *name;
    int mode, rate;
    double skew, off, snr, fade_hz, fade_depth;
    int vis;                       /* -100 = the mode's own */
    double min_psnr;
    double amp;                    /* peak, s16 units (0 = 12000) */
} BenchCase;

#define SAMEVIS (-100)
static const BenchCase BENCH_CASES[] = {
    { "r36 clean 11025",       8, 11025,    0,   0, 99, 0, 0, SAMEVIS, 30, 0 },
    { "r36 clean 16000",       8, 16000,    0,   0, 99, 0, 0, SAMEVIS, 30, 0 },
    { "r36 skew+off+20dB",     8, 11025,  300,  60, 20, 0, 0, SAMEVIS, 24, 0 },
    { "r36 10dB",              8, 11025, -150, -40, 10, 0, 0, SAMEVIS, 16, 0 },
    { "r36 fade",              8, 11025,  100,  20, 25, 0.5, 0.8, SAMEVIS, 22, 0 },
    { "r36 no VIS (autosync)", 8, 11025,  200,   0, 25, 0, 0, -1, 15, 0 },
    { "m1 clean",             44, 11025,    0,   0, 99, 0, 0, SAMEVIS, 30, 0 },
    { "m1 skew 15dB",         44, 11025,  500,  80, 15, 0, 0, SAMEVIS, 20, 0 },
    { "s1 clean",             60, 11025,    0,   0, 99, 0, 0, SAMEVIS, 30, 0 },
    { "s1 no VIS",            60, 11025, -300,   0, 25, 0, 0, -1, 15, 0 },
    { "r72 clean",            12, 11025,    0,   0, 99, 0, 0, SAMEVIS, 30, 0 },
    { "pd50 clean",           93, 11025,    0,   0, 99, 0, 0, SAMEVIS, 28, 0 },
    { "pd120 clean",          95, 11025,    0,   0, 99, 0, 0, SAMEVIS, 26, 0 },
    { "pd120 skew 20dB",      95, 16000,  200,  50, 20, 0, 0, SAMEVIS, 22, 0 },
    { "r36 quiet (peak 150)",  8, 11025,  100,  30, 25, 0, 0, SAMEVIS, 22, 150 },
    { "r36 very quiet (40)",   8, 11025,    0,   0, 30, 0, 0, SAMEVIS, 20, 40 },
    { "r36 loud (peak 32000)", 8, 11025,  100,  30, 25, 0, 0, SAMEVIS, 22, 32000 },
};
#define BENCH_NCASES ((int)(sizeof BENCH_CASES / sizeof BENCH_CASES[0]))

static inline int16_t *bench_audio(const BenchCase *c, int w, int h, uint8_t **ref_out, long *n)
{
    EncOpts o;
    uint8_t *pic = bench_picture(w, h), *ref = (uint8_t *)malloc((size_t)w * h * 3);
    int16_t *a;
    enc_default_opts(&o);
    o.mode = c->mode; o.rate = c->rate; o.skew_ppm = c->skew; o.offset_hz = c->off;
    o.snr_db = c->snr; o.fade_hz = c->fade_hz; o.fade_depth = c->fade_depth;
    o.vis_code = c->vis == SAMEVIS ? c->mode : c->vis;
    o.lead_s = 1.0; o.tail_s = 2.0; o.amplitude = c->amp > 0 ? c->amp : 12000; o.seed = 1234;
    a = enc_encode(pic, &o, n);
    enc_reference(c->mode, pic, ref);
    free(pic);
    *ref_out = ref;
    return a;
}

/* luma PSNR of an RGB565 picture (w x h) against an 8-bit reference;
 * skips the first `skip` rows (autosync starts a few lines in) */
static inline double bench_psnr(const uint16_t *dec, const uint8_t *ref, int w, int h, int skip)
{
    double se = 0; long n = 0; int x, y;
    for (y = skip; y < h; y++) for (x = 0; x < w; x++) {
        uint16_t v = dec[(size_t)y * w + x];
        double r = ((v >> 11) & 31) * 255.0 / 31, g = ((v >> 5) & 63) * 255.0 / 63, b = (v & 31) * 255.0 / 31;
        const uint8_t *q = ref + ((size_t)y * w + x) * 3;
        double yd = 0.299 * r + 0.587 * g + 0.114 * b, yr = 0.299 * q[0] + 0.587 * q[1] + 0.114 * q[2];
        se += (yd - yr) * (yd - yr); n++;
    }
    return se > 0 ? 10 * log10(255.0 * 255.0 / (se / n)) : 99;
}
#endif
