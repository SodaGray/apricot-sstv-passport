/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * sstv_dsp.h -- portable SSTV receiver core, integer per sample, no libm.
 *
 * AI PASSPORT (ESP32-C3) BUILD: copied from the iPhone / Watch build
 * (apricot-sstv-apple, Shared/Core/sstv_dsp.h), which was copied from the
 * iPod repository. Changes for a chip with no FPU and ~300 KB of heap
 * (details at the top of sstv_dsp.c):
 *  - The per-sample path is all integer (same filters, same results to a
 *    fraction of a pixel level; checked against the float build on the host
 *    bench in host/).
 *  - No picture buffer: the decoder hands each finished row to an SstvSink,
 *    at the size sent (PD290: 800 wide), and the app draws it. sstv_image /
 *    sstv_dirty are gone.
 *  - The level ring holds SSTV_LV_LOG2 tone samples (default 5.9 s at
 *    11025), so lines older than that keep the timing they were drawn with.
 *  - About 150 KB in all at 11025 Hz (sstv_new takes it all at once).
 *
 * Modes: Robot 36 / 72, Martin M1 / M2, Scottie S1 / S2 / DX,
 * PD50 / 90 / 120 / 160 / 180 / 240 / 290 -- every mode is a row of one table
 * (sstv_dsp.c), described relative to its sync pulse.
 *
 * Feed it raw s16 audio at the capture rate (11025 on the Passport; any rate
 * from 8000 up works, for WAV files on a computer). It demodulates, watches for a
 * VIS header (or, optionally, for a run of line syncs with a mode's period and
 * sync length when the VIS was missed), decodes the picture line by line, and
 * straightens it as it goes: every line's sync is measured and a line-period
 * fit (offset + rate) re-times the picture, so clock skew between sender and
 * receiver does not shear it. The fitted rate is reported as ppm.
 *
 * DSP chain (at the input rate R, decimation D = round(R/11025), Fs = R/D):
 *   x[n] -> 450 Hz high-pass -> * e^{-j 2pi 1700 n/R}  ->  Blackman low-pass
 *   (127 taps at 44100, scaled with R; complex),
 *   decimate by D  ->  FM discriminator arg(z[m] z*[m-1])  ->  tone in Hz
 *   tone -> level ring (uint8, 1500..2300 Hz = 0..255) + sync-bit ring
 *   tone -> 1 ms bins -> VIS detector (leader offset = tuning correction)
 *
 * Re-timing reaches back as far as the level ring (~6 s); lines older than
 * that keep the timing they were drawn with (by then the fit has settled).
 *
 * Threading: none. One owner calls everything (the Passport: the capture
 * task); the sink is called from inside sstv_feed / sstv_gap.
 *
 * Memory: one fixed set of blocks, all taken by sstv_new and nothing after.
 */
#ifndef SSTV_DSP_H
#define SSTV_DSP_H
#include <stdint.h>

#define SSTV_MAX_W 800               /* widest row handed to the sink (PD290) */
#define SSTV_MAX_H 616

enum { SSTV_IDLE = 0, SSTV_RX = 1 };
enum { SSTV_START_NONE = 0, SSTV_START_VIS = 1, SSTV_START_SYNC = 2 };
enum { SSTV_END_NONE = 0, SSTV_END_COMPLETE = 1, SSTV_END_LOST = 2,
       SSTV_END_ABORT = 3, SSTV_END_NEWVIS = 4, SSTV_END_NOMEM = 5 };

typedef struct {
    int state;              /* SSTV_IDLE / SSTV_RX */
    const char *mode;       /* current/last image's mode ("PD120"), 0 if none yet */
    int width, height;      /* its picture size as sent (PD120: 640x496) */
    int line;               /* picture rows decoded (0..height, as sent) */
    int start_kind;         /* how the current/last image started */
    int end_kind;           /* how the last image ended (SSTV_END_*) */
    int last_vis;           /* last VIS code decoded, -1 if none yet */
    int last_vis_ok;        /* 1 if that VIS is a mode we decode */
    int tone_hz;            /* smoothed tone (tuning indicator) */
    int offset_hz;          /* tuning offset applied to the current/last image */
    int ppm;                /* line-period fit: rx clock vs tx clock, ppm (+ = rx fast) */
    int ppm_valid;          /* 1 once the fit has enough lines to mean something */
    int sync_good;          /* lines whose sync pulse was found ... */
    int sync_seen;          /* ... out of lines searched */
    int slant_redraws;      /* times the picture was re-timed (straightened) */
    int relocks;            /* times the picture was resumed after a fade or gap */
    unsigned long images;   /* images completed (>= 83 % of their lines) */
    unsigned long gaps;     /* input samples reported lost (sstv_gap) */
    unsigned revision;      /* bumps whenever the picture changes */
    double start_s;         /* where the current/last picture's line 0 sync starts,
                               seconds of input from the first sample fed */
    double ppm_fit;         /* its clock error as fitted right now (ppm), valid or not */
} SstvStatus;

typedef struct SstvRx SstvRx;

/* in_rate: input sample rate (Hz). NULL on OOM or a rate below 8000. */
SstvRx *sstv_new(int in_rate);
void    sstv_free(SstvRx *s);

/* Feed n input samples, taking every `stride`-th s16 (stride 2 = left channel
 * of interleaved stereo, as the nano's capture DMA delivers). */
void sstv_feed(SstvRx *s, const int16_t *x, int n, int stride);

/* n input samples were lost (capture overrun / the app stalled). Keeps the
 * time base exact without filtering silence: costs ~nothing per sample, so a
 * multi-second gap doesn't stall the caller. Picture rows whose audio falls
 * in a gap are left as they were (not drawn from silence). */
void sstv_gap(SstvRx *s, int n);

void sstv_status(const SstvRx *s, SstvStatus *st);

/* Where finished rows go. begin: a new picture starts (its mode and size as
 * sent; clear the picture area). row: row y (0..h-1) is w RGB565 pixels,
 * valid during the call only. A row can come again later, re-timed, when
 * the slant fit improves; Robot 36 rows also come again once their partner
 * line's chroma arrives. Both are called from inside sstv_feed / sstv_gap. */
typedef struct {
    void (*begin)(void *user, const char *mode, int w, int h);
    void (*row)(void *user, int y, const uint16_t *px, int w);
} SstvSink;
void sstv_set_sink(SstvRx *s, const SstvSink *sink, void *user);

/* Stop the current image (picture kept, back to listening). */
void sstv_abort(SstvRx *s);
/* Forget the picture (rows are no longer re-timed or recoloured). */
void sstv_clear(SstvRx *s);
/* Start on a run of line syncs when no VIS was heard (default on). */
void sstv_set_autosync(SstvRx *s, int on);
/* Receive only the mode with this VIS code (-1 = any mode, the default).
 * A VIS for another mode is reported (last_vis, with last_vis_ok = 0) but
 * starts nothing, and autosync looks only for this mode's line syncs.
 * A code this build doesn't decode means any mode. Takes effect for the
 * next picture; one in progress carries on. */
void sstv_set_mode(SstvRx *s, int vis);
/* Prior for the clock error (ppm), used until the fit takes over. The decoder
 * updates its own prior after every good image; this seeds it. */
void sstv_set_ppm_prior(SstvRx *s, int ppm);

/* Darkroom: decoding a recording again with settings chosen by hand.
 * sstv_start_at, called before any audio is fed, starts a picture of the
 * mode with this VIS code whose line 0 sync starts `seconds` into the audio
 * (the tuning offset given), and from then on: no VIS starts anything,
 * autosync is off, and the picture never ends as "lost" (it ends when all
 * its lines are drawn, or with the audio). 0 if the mode isn't decoded. */
int  sstv_start_at(SstvRx *s, int vis, double seconds, double offset_hz);
/* Fix the line period at this clock error (ppm, as SstvStatus.ppm) instead
 * of fitting it; the phase is still fitted to the syncs found. fixed = 0
 * goes back to fitting. Set before the picture starts. */
void sstv_set_slant(SstvRx *s, int fixed, double ppm);
/* Sample every line `ms` later than its sync says (the picture moves left;
 * negative: right). */
void sstv_set_shift(SstvRx *s, double ms);

/* Name for a VIS code ("Robot 36", "Martin M1", ...), or 0 if unknown. */
const char *sstv_vis_name(int code);
/* 1 if this build decodes that VIS code */
int sstv_vis_supported(int code);

#endif /* SSTV_DSP_H */
