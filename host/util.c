/* SPDX-License-Identifier: GPL-3.0-only */
/* util.c -- see util.h */
#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

int16_t *wav_read(const char *path, long *frames, int *channels, int *rate)
{
    FILE *f = fopen(path, "rb");
    uint8_t hdr[12], ch[8], fmt[40];
    int fmt_ok = 0, bits = 0, ftag = 0, nch = 0;
    long sr = 0;
    int16_t *out = 0;
    if (!f) return 0;
    if (fread(hdr, 1, 12, f) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) { fclose(f); return 0; }
    while (fread(ch, 1, 8, f) == 8) {
        uint32_t len = le32(ch + 4);
        if (!memcmp(ch, "fmt ", 4)) {
            uint32_t r = len < sizeof fmt ? len : sizeof fmt;
            if (fread(fmt, 1, r, f) != r) break;
            if (len > r) fseek(f, (long)(len - r), SEEK_CUR);
            ftag = le16(fmt); nch = le16(fmt + 2); sr = (long)le32(fmt + 4); bits = le16(fmt + 14);
            if (ftag == 0xFFFE && r >= 26) ftag = le16(fmt + 24);   /* WAVE_FORMAT_EXTENSIBLE */
            fmt_ok = 1;
        } else if (!memcmp(ch, "data", 4) && fmt_ok) {
            int bps = bits / 8;
            long nfr, i;
            uint8_t *raw;
            if (nch < 1 || bps < 2 || bps > 4) break;
            nfr = (long)(len / (uint32_t)(bps * nch));
            raw = malloc((size_t)nfr * bps * nch);
            nfr = (long)(fread(raw, (size_t)(bps * nch), (size_t)nfr, f));
            out = malloc((size_t)nfr * nch * sizeof *out + 1);
            for (i = 0; i < nfr * nch; i++) {
                const uint8_t *p = raw + i * bps;
                int32_t v;
                if (ftag == 3 && bps == 4) {
                    float x; memcpy(&x, p, 4); x *= 32768.0f;
                    v = x > 32767 ? 32767 : x < -32768 ? -32768 : (int32_t)x;
                } else if (bps == 2) v = (int16_t)le16(p);
                else if (bps == 3) v = ((int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24)) >> 16;
                else v = ((int32_t)le32(p)) >> 16;
                out[i] = (int16_t)v;
            }
            free(raw);
            *frames = nfr; *channels = nch; *rate = (int)sr;
            break;
        } else {
            fseek(f, (long)(len + (len & 1)), SEEK_CUR);
        }
    }
    fclose(f);
    return out;
}

static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

int wav_write_mono(const char *path, const int16_t *x, long n, int rate)
{
    uint8_t h[44];
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    memcpy(h, "RIFF", 4); put32(h + 4, (uint32_t)(36 + n * 2)); memcpy(h + 8, "WAVEfmt ", 8);
    put32(h + 16, 16); put16(h + 20, 1); put16(h + 22, 1); put32(h + 24, (uint32_t)rate);
    put32(h + 28, (uint32_t)rate * 2); put16(h + 32, 2); put16(h + 34, 16);
    memcpy(h + 36, "data", 4); put32(h + 40, (uint32_t)(n * 2));
    fwrite(h, 1, 44, f); fwrite(x, 2, (size_t)n, f);
    fclose(f);
    return 1;
}

static int ppm_tok(FILE *f)
{
    int c, v = 0;
    do { c = fgetc(f); if (c == '#') while (c != '\n' && c != EOF) c = fgetc(f); } while (c == ' ' || c == '\n' || c == '\r' || c == '\t');
    while (c >= '0' && c <= '9') { v = v * 10 + (c - '0'); c = fgetc(f); }
    return v;
}

uint8_t *ppm_read(const char *path, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    char m[3] = { 0 };
    uint8_t *px;
    int mx;
    if (!f) return 0;
    if (fread(m, 1, 2, f) != 2 || strcmp(m, "P6")) { fclose(f); return 0; }
    *w = ppm_tok(f); *h = ppm_tok(f); mx = ppm_tok(f);
    if (mx != 255 || *w <= 0 || *h <= 0) { fclose(f); return 0; }
    px = malloc((size_t)*w * *h * 3);
    if (fread(px, 3, (size_t)*w * *h, f) != (size_t)*w * *h) { free(px); fclose(f); return 0; }
    fclose(f);
    return px;
}

int ppm_write(const char *path, const uint8_t *rgb, int w, int h)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(rgb, 3, (size_t)w * h, f);
    fclose(f);
    return 1;
}

int ppm_write_xrgb(const char *path, const uint32_t *px, int w, int h)
{
    uint8_t *b = malloc((size_t)w * h * 3);
    int i, ok;
    for (i = 0; i < w * h; i++) { b[i * 3] = px[i] >> 16; b[i * 3 + 1] = px[i] >> 8; b[i * 3 + 2] = px[i]; }
    ok = ppm_write(path, b, w, h);
    free(b);
    return ok;
}

void rgb565_to888(uint16_t v, uint8_t *o)
{
    o[0] = (uint8_t)(((v >> 8) & 0xF8) | (v >> 13));
    o[1] = (uint8_t)(((v >> 3) & 0xFC) | ((v >> 9) & 3));
    o[2] = (uint8_t)(((v << 3) & 0xF8) | ((v >> 2) & 7));
}

int ppm_write_565(const char *path, const uint16_t *px, int w, int h)
{
    uint8_t *b = malloc((size_t)w * h * 3);
    int i, ok;
    for (i = 0; i < w * h; i++) rgb565_to888(px[i], b + i * 3);
    ok = ppm_write(path, b, w, h);
    free(b);
    return ok;
}

uint8_t *rgb_resize(const uint8_t *in, int w, int h, int W, int H)
{
    uint8_t *o = malloc((size_t)W * H * 3);
    int x, y;
    for (y = 0; y < H; y++) for (x = 0; x < W; x++)
        memcpy(o + (y * W + x) * 3, in + (((long)y * h / H) * w + (long)x * w / W) * 3, 3);
    return o;
}
