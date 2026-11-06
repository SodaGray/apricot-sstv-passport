/* SPDX-License-Identifier: GPL-3.0-only */
/* enc.h -- host-only SSTV encoder for testing the receiver: Robot 36/72,
 * Martin M1/M2, Scottie S1/S2/DX, PD50/90/120/160/180/240/290, each family
 * written out from the published timings (independently of the decoder's
 * mode table). Phase-continuous FM synthesis, with channel impairments:
 * clock skew, tuning offset, AWGN, fading. */
#ifndef SSTV_ENC_H
#define SSTV_ENC_H
#include <stdint.h>

typedef struct {
    int    mode;          /* VIS code of the mode to send (8 = Robot 36, 44 = Martin M1 ...) */
    int    rate;          /* output sample rate */
    double skew_ppm;      /* stretch the signal's timeline: + = each line lasts longer at the receiver */
    double offset_hz;     /* add to every tone (mistuned SSB) */
    double snr_db;        /* SNR in a 3 kHz bandwidth; >= 99 = no noise */
    double fade_hz;       /* sinusoidal amplitude fade rate (0 = none) */
    double fade_depth;    /* 0..1 */
    double amplitude;     /* peak, s16 units */
    int    vis_code;      /* VIS to send (normally = mode); -1 = no VIS */
    double lead_s;        /* silence before, seconds */
    double tail_s;        /* silence after */
    int    lines;         /* sync periods to send (0 = the whole picture) */
    unsigned seed;
} EncOpts;

void enc_default_opts(EncOpts *o);                 /* Robot 36, 44100, clean */
/* picture size of a mode; 0 if the encoder doesn't know it */
int  enc_mode_size(int mode, int *w, int *h);
const char *enc_mode_name(int mode);
/* rgb: w x h x 3 bytes of the mode's size. Returns a malloc'd mono s16 buffer, *n samples. */
int16_t *enc_encode(const uint8_t *rgb, const EncOpts *o, long *n);
/* Append b to a (realloc); returns new buffer. */
int16_t *enc_concat(int16_t *a, long na, const int16_t *b, long nb, long *n);
/* What an ideal decoder would reproduce (the mode's chroma subsampling
 * applied), w x h x 3. */
void enc_reference(int mode, const uint8_t *rgb, uint8_t *out);

#endif
