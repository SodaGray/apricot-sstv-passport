#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""make_splash.py -- renders the boot splash (an apricot Mandelbrot with the
"Apricot" wordmark) at the panel's 320 x 240 and writes it as raw RGB565 in
the panel's byte order (high byte first), for main/CMakeLists.txt to embed.

Needs numpy, Pillow and two OFL fonts from github.com/google/fonts:
  ofl/newsreader/Newsreader-Italic[opsz,wght].ttf
  ofl/inter/Inter[opsz,wght].ttf

  python3 tools/make_splash.py --serif <ttf> --inter <ttf> \
      --version '26.11(3)' --out main/splash.rgb565 --preview splash.png
"""
import argparse
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, S = 320, 240, 4                 # panel size, text supersampling
ACCENT = (242, 154, 91)


def font(path, size, wght, opsz=None):
    f = ImageFont.truetype(path, size)
    axes = []
    for a in f.get_variation_axes():
        name = a['name'] if isinstance(a['name'], str) else a['name'].decode()
        if name.lower().startswith('weight'):
            axes.append(wght)
        elif name.lower().startswith('optical'):
            # the size's own optical cut unless told otherwise
            o = opsz if opsz is not None else size / S
            axes.append(max(a['minimum'], min(a['maximum'], o)))
        else:
            axes.append(a['default'])
    f.set_variation_by_axes(axes)
    return f


def mandelbrot():
    # 0.01 per pixel; the set's tail reaches in from the left under the title
    xs = -2.73 + (3.0 / 299) * np.arange(W)
    ys = np.linspace(-1.2, 1.2, H)
    c = xs[None, :] + 1j * ys[:, None]
    z = np.zeros_like(c)
    n = np.zeros(c.shape)
    alive = np.ones(c.shape, bool)
    for i in range(120):
        z[alive] = z[alive] ** 2 + c[alive]
        esc = alive & (np.abs(z) > 4)
        n[esc] = i + 1 - np.log2(np.log(np.abs(z[esc])))
        alive &= ~esc
    v = np.where(alive, 0, np.sqrt(np.clip(n, 0, None) / 120))
    stops = [(0, (0, 0, 0)), (0.25, (40, 22, 30)), (0.5, (160, 70, 50)),
             (0.75, ACCENT), (1, (255, 240, 225))]

    def cm(t):
        for (a, ca), (b, cb) in zip(stops, stops[1:]):
            if t <= b:
                u = (t - a) / (b - a)
                return [ca[k] + (cb[k] - ca[k]) * u for k in range(3)]
        return stops[-1][1]
    lut = np.array([cm(t) for t in np.linspace(0, 1, 256)])
    m = lut[(np.clip(v * 1.6, 0, 1) * 255).astype(int)]
    m[alive] = 0
    return Image.fromarray(m.astype(np.uint8)).convert('RGBA')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--serif', required=True)
    ap.add_argument('--inter', required=True)
    ap.add_argument('--version', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--preview')
    a = ap.parse_args()

    text = Image.new('RGBA', (W * S, H * S), (0, 0, 0, 0))
    d = ImageDraw.Draw(text)
    d.text((28 * S, 60 * S), 'Apricot', font=font(a.serif, 38 * S, 600),
           fill=(255, 241, 228, 255), anchor='ls')
    sm = font(a.inter, 9 * S, 600, 14)
    x = 29 * S
    for ch in 'SSTV':                 # letter-spaced 3 px
        d.text((x, 80 * S), ch, font=sm, fill=ACCENT + (255,), anchor='ls')
        x += d.textlength(ch, font=sm) + 3 * S
    d.text((28 * S, 198 * S), 'Press any key', font=font(a.inter, 9 * S, 600, 14),
           fill=ACCENT + (255,), anchor='ls')
    d.text((28 * S, 214 * S), 'AI Passport · ' + a.version, font=font(a.inter, 9 * S, 500, 14),
           fill=(150, 155, 164, 255), anchor='ls')

    img = Image.alpha_composite(mandelbrot(), text.resize((W, H), Image.BOX)).convert('RGB')
    mask = Image.new('L', (W, H), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, W - 1, H - 1], radius=30, fill=255)
    px = np.array(img).astype(np.uint16)
    px[np.array(mask) == 0] = 0       # outside the glass's rounded corners
    v = ((px[..., 0] & 0xF8) << 8) | ((px[..., 1] & 0xFC) << 3) | (px[..., 2] >> 3)
    v.astype('>u2').tofile(a.out)     # panel order: high byte first

    if a.preview:                     # what the panel shows, 4x
        r = ((v >> 11) & 31) * 255 // 31
        g = ((v >> 5) & 63) * 255 // 63
        b = (v & 31) * 255 // 31
        Image.fromarray(np.dstack([r, g, b]).astype(np.uint8)).resize((W * 4, H * 4), Image.NEAREST).save(a.preview)


if __name__ == '__main__':
    main()
