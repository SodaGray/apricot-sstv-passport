/* SPDX-License-Identifier: GPL-3.0-only */
/* bench_orig.c -- the same cases through the unmodified iPhone/Watch decoder
 * (float, full picture buffer): the baseline the Passport port is held to. */
#include "sstv_dsp.h"
#include "bench_common.h"
#include <stdio.h>
#include <time.h>

int main(void)
{
    int i;
    for (i = 0; i < BENCH_NCASES; i++) {
        const BenchCase *c = &BENCH_CASES[i];
        int w, h, iw, ih;
        long n, j;
        uint8_t *ref;
        int16_t *a;
        SstvRx *s;
        SstvStatus st;
        clock_t t0;
        double cpu;
        enc_mode_size(c->mode, &w, &h);
        a = bench_audio(c, w, h, &ref, &n);
        s = sstv_new(c->rate);
        t0 = clock();
        for (j = 0; j < n; j += 256) sstv_feed(s, a + j, (int)(n - j < 256 ? n - j : 256), 1);
        cpu = (double)(clock() - t0) / CLOCKS_PER_SEC;
        sstv_status(s, &st);
        {
            const uint16_t *img = sstv_image(s, &iw, &ih);
            double p = (img && iw == w && ih == h) ? bench_psnr(img, ref, w, h, c->vis == -1 ? 8 : 0) : 0;
            printf("%-24s %-10s images=%lu end=%d psnr=%5.1f ppm=%5d sync=%d/%d  %.0f us/s-audio\n",
                   c->name, st.mode ? st.mode : "-", st.images, st.end_kind, p, st.ppm,
                   st.sync_good, st.sync_seen, cpu * 1e6 / ((double)n / c->rate));
        }
        sstv_free(s); free(a); free(ref);
    }
    return 0;
}
