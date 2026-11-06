/* SPDX-License-Identifier: GPL-3.0-only */
/* bench_port.c -- the bench cases through the Passport decoder (integer,
 * row sink), collecting rows into a picture to score against the same
 * reference as bench_orig. Also counts the heap the decoder takes. */
#include "sstv_dsp.h"
#include "bench_common.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static size_t g_heap;
void *sstv_pool_alloc(size_t n) { g_heap += n; return malloc(n); }
void sstv_pool_free(void *p) { free(p); }

typedef struct { uint16_t *img; int w, h, begins; long rows; } Pic;

static void on_begin(void *u, const char *mode, int w, int h)
{
    Pic *p = (Pic *)u;
    (void)mode;
    free(p->img);
    p->img = (uint16_t *)calloc((size_t)w * h, 2); p->w = w; p->h = h; p->begins++;
}
static void on_row(void *u, int y, const uint16_t *px, int w)
{
    Pic *p = (Pic *)u;
    if (!p->img || y < 0 || y >= p->h || w != p->w) { fprintf(stderr, "bad row %d w %d\n", y, w); exit(3); }
    memcpy(p->img + (size_t)y * w, px, (size_t)w * 2); p->rows++;
}

int main(int argc, char **argv)
{
    int i, bad = 0;
    const char *outdir = argc > 1 ? argv[1] : 0;
    for (i = 0; i < BENCH_NCASES; i++) {
        const BenchCase *c = &BENCH_CASES[i];
        int w, h;
        long n, j;
        uint8_t *ref;
        int16_t *a;
        SstvRx *s;
        SstvStatus st;
        Pic pic = { 0, 0, 0, 0, 0 };
        SstvSink sink = { on_begin, on_row };
        clock_t t0;
        double cpu, p;
        enc_mode_size(c->mode, &w, &h);
        a = bench_audio(c, w, h, &ref, &n);
        g_heap = 0;
        s = sstv_new(c->rate);
        sstv_set_sink(s, &sink, &pic);
        t0 = clock();
        for (j = 0; j < n; j += 256) sstv_feed(s, a + j, (int)(n - j < 256 ? n - j : 256), 1);
        cpu = (double)(clock() - t0) / CLOCKS_PER_SEC;
        sstv_status(s, &st);
        p = (pic.img && pic.w == w && pic.h == h) ? bench_psnr(pic.img, ref, w, h, c->vis == -1 ? 8 : 0) : 0;
        if (p < c->min_psnr || st.images != 1) bad++;
        printf("%-24s %-10s images=%lu end=%d psnr=%5.1f %s ppm=%5d sync=%d/%d rows=%ld heap=%zuK %.0f us/s\n",
               c->name, st.mode ? st.mode : "-", st.images, st.end_kind, p, p >= c->min_psnr ? "ok  " : "LOW ",
               st.ppm, st.sync_good, st.sync_seen, pic.rows, g_heap / 1024, cpu * 1e6 / ((double)n / c->rate));
        if (outdir && pic.img) {
            char path[512]; extern int ppm_write_565(const char *, const uint16_t *, int, int);
            snprintf(path, sizeof path, "%s/case%02d.ppm", outdir, i);
            ppm_write_565(path, pic.img, pic.w, pic.h);
        }
        sstv_free(s); free(a); free(ref); free(pic.img);
    }
    if (bad) printf("%d case(s) below their mark\n", bad);
    return bad ? 1 : 0;
}
