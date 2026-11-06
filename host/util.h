/* SPDX-License-Identifier: GPL-3.0-only */
/* util.h -- host helpers: WAV in/out, PPM in/out. */
#ifndef SSTV_HOST_UTIL_H
#define SSTV_HOST_UTIL_H
#include <stdint.h>

/* Reads PCM 16/24/32-bit or float32 WAV, any channel count; returns the
 * interleaved s16 samples (malloc'd), sets frames/channels/rate. NULL on error. */
int16_t *wav_read(const char *path, long *frames, int *channels, int *rate);
int      wav_write_mono(const char *path, const int16_t *x, long n, int rate);

/* P6 PPM, 8-bit. read returns malloc'd w*h*3. */
uint8_t *ppm_read(const char *path, int *w, int *h);
int      ppm_write(const char *path, const uint8_t *rgb, int w, int h);
/* write 0x00RRGGBB pixels as PPM */
int      ppm_write_xrgb(const char *path, const uint32_t *px, int w, int h);
/* write RGB565 pixels as PPM (bits replicated to 8) */
int      ppm_write_565(const char *path, const uint16_t *px, int w, int h);
/* RGB565 -> 3 bytes */
void     rgb565_to888(uint16_t v, uint8_t *o);
/* nearest-neighbour resize of w x h x 3 to W x H x 3 (malloc'd) */
uint8_t *rgb_resize(const uint8_t *in, int w, int h, int W, int H);

#endif
