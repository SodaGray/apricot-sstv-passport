# Changelog

## 26.11(3) — 2026-11-06

First release.

- Apricot SSTV's receiver on the FoloToy AI Passport: 14 modes, VIS and
  autosync starts, slant correction, fade recovery, through the Passport's
  own microphone.
- Decoder ported from the iPhone/Watch build to integer arithmetic, with no
  picture buffer (about 141 KB at 16 kHz).
- Landscape 320×240 screen: Robot 36 at 1:1, other modes scaled to fit; a
  status bar kept clear of the rounded corners.
- Keys for the status bar, mode lock, screen rotation, backlight and
  start-without-VIS; the mode starts at AUTO on every boot, the rest is
  kept across restarts.
- Mandelbrot boot splash, shown until a key is pressed.
- Key help after the splash, in English and Chinese (UP / DOWN switches,
  OK closes; double-click OK to see it again).
- Host checks (decoder bench, app simulation) and a GitHub Actions build.
