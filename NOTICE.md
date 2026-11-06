# Third-party parts

Apricot SSTV for AI Passport is GPL-3.0-only (see `LICENSE`). It builds on,
without copying into this repository:

| Part | Where | License |
|------|-------|---------|
| FoloToy AI Passport board support package | `third_party/ai-passport` (git submodule) | MIT, © FoloToy |
| ESP-IDF and the components the BSP pulls in (LVGL, esp_lvgl_port, button, esp_codec_dev) | downloaded at build time | Apache-2.0 / MIT, see each |
| Newsreader, Inter (only to regenerate the splash and help images) | not included; from github.com/google/fonts | SIL Open Font License 1.1 |
| Zhuque Fangsong 朱雀仿宋 (the Chinese help's title) | not included; github.com/TrionesType/zhuque | SIL Open Font License 1.1 |
| Ark Pixel Font 方舟像素字体, 12 px (the Chinese help's text) | not included; github.com/TakWolf/ark-pixel-font | SIL Open Font License 1.1 |

The images `main/splash.rgb565`, `main/help_en.rgb565` and `main/help_zh.rgb565`
are rendered from those fonts.
The decoder in `main/sstv_dsp.c` and the test encoder in `host/enc.c` come
from Apricot SSTV (iPod nano 7 and iPhone/Watch), by the same author.
