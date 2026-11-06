#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""make_help.py -- renders the key help shown after the boot splash, in
English and in Chinese, at the panel's 320 x 240, as raw RGB565 in the
panel's byte order (high byte first), for main/CMakeLists.txt to embed.

Fonts (all SIL OFL 1.1):
  --serif    Newsreader Italic, github.com/google/fonts
             ofl/newsreader/Newsreader-Italic[opsz,wght].ttf
  --inter    Inter, github.com/google/fonts  ofl/inter/Inter[opsz,wght].ttf
  --cjk      Zhuque Fangsong (朱雀仿宋), github.com/TrionesType/zhuque,
             ZhuqueFangsong-Regular.ttf: the Chinese title
  --pixel    the glyph folder of Ark Pixel Font (方舟像素字体),
             github.com/TakWolf/ark-pixel-font  assets/glyphs/12/cmap:
             the Chinese text, drawn pixel for pixel at 12 px

  python3 tools/make_help.py --serif ... --inter ... --cjk ... --pixel ... \\
      --out-en main/help_en.rgb565 --out-zh main/help_zh.rgb565 --preview help.png
"""
import argparse
import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, S = 320, 240, 4
BG, INK, DIM, ACCENT, CAP = (16, 17, 20), (244, 241, 236), (150, 155, 164), (242, 154, 91), (42, 45, 51)

EN = {
    'title': 'How to use',
    'sub': 'Hold it near the radio’s speaker.',
    'rows': [
        ('OK',    [('Click', 'Status bar on / off'), ('Hold', 'Stop the picture and clear')]),
        ('UP',    [('Click', 'Mode   AUTO › R36 › PD120 › M1 › S1'), ('Hold', 'Turn the screen 180°')]),
        ('DOWN',  [('Click', 'Backlight 100 / 50 / 20 %'), ('Hold', 'Start without VIS on / off')]),
        ('OK ×2', [('', 'Show this help again')]),
    ],
    'foot': 'OK closes this.',
    'switch': ('UP / DOWN', '中文'),          # 中文
}
ZH = {
    'title': '使用方法',                    # 使用方法
    'sub': '把它放在电台喇叭旁边。',
    'rows': [
        ('OK',    [('单击', '显示/隐藏状态栏'), ('长按', '停止收图并清屏')]),
        ('UP',    [('单击', '模式 AUTO→R36→PD120→M1→S1'), ('长按', '屏幕转 180°')]),
        ('DOWN',  [('单击', '背光 100/50/20%'), ('长按', '没有 VIS 也开始收图：开/关')]),
        ('OK ×2', [('', '再次显示帮助')]),
    ],
    'foot': '按 OK 关闭。',
    'switch': ('UP / DOWN', 'English'),
}


def font(path, px, wght=None, opsz=None):
    f = ImageFont.truetype(path, px * S)
    try:
        axes_info = f.get_variation_axes()
    except Exception:              # not a variable font
        return f
    axes = []
    for a in axes_info:
        name = (a['name'] if isinstance(a['name'], str) else a['name'].decode()).lower()
        if name.startswith('weight') and wght is not None:
            axes.append(max(a['minimum'], min(a['maximum'], wght)))
        elif name.startswith('optical'):
            axes.append(max(a['minimum'], min(a['maximum'], opsz if opsz is not None else px)))
        else:
            axes.append(a['default'])
    f.set_variation_by_axes(axes)
    return f


class Pixel:
    """Ark Pixel 12 px glyphs, one PNG per code point: Latin ones 18 px tall
    with the baseline 14 px down, CJK ones 12 px tall centred in the same
    18 px line. The Simplified Chinese form where a character has several."""
    BASE, LINE = 14, 18

    def __init__(self, root):
        self.files = {}
        for d, _, fs in os.walk(root):
            for f in fs:
                if not f.endswith('.png') or f == 'notdef.png':
                    continue
                stem = f[:-4]
                cp, _, langs = stem.partition(' ')
                ls = langs.split(',')
                rank = 0 if ('zh_cn' in ls or 'zh_hans' in ls) else (1 if not langs else 9)
                if '/proportional/' in d + '/' or '/common/' in d + '/':
                    rank -= 0
                elif '/monospaced/' in d + '/':
                    rank += 5
                key = int(cp, 16)
                if key not in self.files or rank < self.files[key][0]:
                    self.files[key] = (rank, os.path.join(d, f))

    def glyph(self, ch):
        if ord(ch) not in self.files or self.files[ord(ch)][0] >= 9:
            raise SystemExit('Ark Pixel has no (Simplified Chinese) glyph for %r U+%04X' % (ch, ord(ch)))
        return Image.open(self.files[ord(ch)][1]).convert('RGBA')

    def width(self, s):
        return sum(self.glyph(c).width for c in s)

    def draw(self, img, x, y, s, rgb):
        for c in s:
            g = self.glyph(c)
            a = np.array(g)[..., 3] > 127
            px = np.array(img)
            top = y - self.BASE + (self.LINE - g.height) // 2
            sub = px[top:top + g.height, x:x + g.width]
            sub[a[:sub.shape[0], :sub.shape[1]]] = rgb
            img.paste(Image.fromarray(px))
            x += g.width
        return x


def render(lang, a, pixel):
    t = EN if lang == 'en' else ZH
    hi = Image.new('RGB', (W * S, H * S), BG)          # anti-aliased parts, 4x
    d = ImageDraw.Draw(hi)
    later = []                                          # pixel text, drawn at 1x afterwards
    body = font(a.inter, 10, 500, 14)
    small = font(a.inter, 9, 500, 14)
    cap = font(a.inter, 8, 700, 14)

    def text(x, y, s, f, c):
        d.text((x * S, y * S), s, font=f, fill=c, anchor='ls')

    if lang == 'en':
        text(24, 36, t['title'], font(a.serif, 24, 600), INK)
        text(24, 52, t['sub'], small, DIM)
        y, step, gap, x_k, x_a = 74, 15, 9, 76, 110
    else:
        text(24, 37, t['title'], font(a.cjk, 23), INK)
        later.append((24, 56, t['sub'], DIM))
        y, step, gap, x_k, x_a = 78, 16, 8, 72, 104

    for key, acts in t['rows']:
        w = d.textlength(key, font=cap) / S + 10
        top, bot = (y - 9, y + 3) if lang == 'en' else (y - 10, y + 3)
        d.rounded_rectangle([24 * S, top * S, (24 + w) * S, bot * S], radius=3 * S, fill=CAP)
        text(29, y - (0 if lang == 'en' else 1), key, cap, INK)
        for i, (k, act) in enumerate(acts):
            yy = y + i * step
            if lang == 'en':
                if k:
                    text(x_k, yy, k, small, DIM)
                text(x_a, yy, act, body, INK)
            else:
                if k:
                    later.append((x_k, yy, k, DIM))
                later.append((x_a, yy, act, INK))
        y += step * len(acts) + gap

    # footer: closing on the left, the other language on the right
    lab, other = t['switch']
    if lang == 'en':
        text(24, 222, t['foot'], small, ACCENT)
        ow = pixel.width(other)
        lw = d.textlength(lab + '  ', font=small) / S
        text(W - 24 - ow - lw, 222, lab + '  ', small, DIM)
        later.append((W - 24 - ow, 224, other, ACCENT))
    else:
        later.append((24, 224, t['foot'], ACCENT))
        ow = d.textlength(other, font=small) / S
        lw = d.textlength(lab + '  ', font=small) / S
        text(W - 24 - ow - lw, 222, lab + '  ', small, DIM)
        text(W - 24 - ow, 222, other, small, ACCENT)

    img = hi.resize((W, H), Image.BOX)
    for x, yy, s, c in later:
        pixel.draw(img, int(x), int(yy), s, c)
    return img


def write(img, out):
    mask = Image.new('L', (W, H), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, W - 1, H - 1], radius=30, fill=255)
    px = np.array(img).astype(np.uint16)
    px[np.array(mask) == 0] = 0
    v = ((px[..., 0] & 0xF8) << 8) | ((px[..., 1] & 0xFC) << 3) | (px[..., 2] >> 3)
    v.astype('>u2').tofile(out)
    r = ((v >> 11) & 31) * 255 // 31
    g = ((v >> 5) & 63) * 255 // 63
    b = (v & 31) * 255 // 31
    return Image.fromarray(np.dstack([r, g, b]).astype(np.uint8))


def main():
    ap = argparse.ArgumentParser()
    for k in ('serif', 'inter', 'cjk', 'pixel', 'out_en', 'out_zh'):
        ap.add_argument('--' + k.replace('_', '-'), required=True)
    ap.add_argument('--preview')
    a = ap.parse_args()
    pixel = Pixel(a.pixel)
    en = write(render('en', a, pixel), a.out_en)
    zh = write(render('zh', a, pixel), a.out_zh)
    if a.preview:
        sheet = Image.new('RGB', (W * 2 + 10, H), (48, 48, 52))
        sheet.paste(en, (0, 0))
        sheet.paste(zh, (W + 10, 0))
        sheet.resize((sheet.width * 4, H * 4), Image.NEAREST).save(a.preview)


if __name__ == '__main__':
    main()
