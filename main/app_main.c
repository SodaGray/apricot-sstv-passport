/* SPDX-License-Identifier: GPL-3.0-only */
/*
 * app_main.c -- Apricot SSTV on the FoloToy AI Passport (ESP32-C3).
 *
 * One task does everything: reads the microphone (ES8311, 16 kHz mono),
 * feeds the decoder, draws the rows it hands back straight to the panel
 * (no LVGL, no frame buffer), handles the three keys and redraws the status
 * bar a few times a second.
 *
 * Screen: the 240 x 320 panel is turned to landscape, 320 x 240. Robot 36 is
 * 320 x 240 and fills it pixel for pixel; every other mode is scaled to fit
 * (nearest neighbour), centred.
 *
 * Screens: the boot splash stays until a key is pressed; then the key help
 * (English or Chinese, UP / DOWN switch, OK closes; the decoder already
 * listens, and a picture starting closes it too); then the picture.
 *
 * Keys, on the picture screen:
 *   OK    click  status bar on / off       long  stop the picture, clear
 *         double the key help (while no picture is coming in)
 *   UP    click  receive mode: AUTO > R36 > PD120 > M1 > S1 > AUTO
 *         long   turn the screen 180 degrees (remembered)
 *   DOWN  click  backlight 100 > 50 > 20 %   long  autosync on / off
 * The receive mode starts at AUTO on every boot; the rest is remembered.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "bsp_audio.h"
#include "bsp_battery.h"
#include "bsp_button.h"
#include "bsp_display.h"
#include "bsp_i2c.h"

#include "sstv_dsp.h"
#include "sstv_font5x7.h"
#include "version.h"

static const char *TAG = "apricot";

#define SCR_W      320
#define SCR_H      240
#define RATE       16000            /* what the BSP demo records at; the decoder takes any rate */
#define CHUNK      256              /* samples per read: 16 ms */
#define BAR_H      11               /* status bar: 7 px font, 2 px above and below */
#define BAR_Y      (SCR_H - BAR_H)
#define NROWBUF    4                /* see draw_rows */
#define STATUS_US  250000           /* status bar refresh */

/* ------------------------------------------------------------- settings */
static const struct { int vis; const char *name; } MODE_LOCKS[] = {
    { -1, "AUTO" }, { 8, "R36" }, { 95, "PD120" }, { 44, "M1" }, { 60, "S1" },
};
#define NLOCKS ((int)(sizeof MODE_LOCKS / sizeof MODE_LOCKS[0]))
static const uint8_t BACKLIGHT[] = { 100, 50, 20 };

typedef struct {
    uint8_t flip;                   /* screen turned 180 degrees */
    uint8_t lock;                   /* index into MODE_LOCKS (not restored: AUTO at boot) */
    uint8_t backlight;              /* index into BACKLIGHT */
    uint8_t autosync;
    uint8_t lang;                   /* key help: 0 English, 1 Chinese */
} Settings;
static Settings g_set = { 0, 0, 0, 1, 0 };

static void settings_load(void)
{
    nvs_handle_t h;
    size_t n = sizeof g_set;
    Settings t;
    if (nvs_open("apricot", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_blob(h, "set", &t, &n) == ESP_OK && n == sizeof t) {
        if (t.backlight < sizeof BACKLIGHT && t.lang < 2) { g_set = t; g_set.lock = 0; }
    }
    nvs_close(h);
}
static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open("apricot", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "set", &g_set, sizeof g_set);
    nvs_commit(h);
    nvs_close(h);
}

/* --------------------------------------------------------------- display */
static esp_lcd_panel_handle_t g_panel;
static uint16_t *g_rowbuf[NROWBUF]; /* DMA-capable, one screen row each */
static int g_rowbuf_i;
static uint16_t *g_bar;             /* the status bar, SCR_W x BAR_H, panel byte order */
static uint16_t *g_under;           /* what the picture has under the bar, panel byte order */
static int g_bar_on = 1;
enum { SCREEN_SPLASH, SCREEN_HELP, SCREEN_PICTURE };
static int g_screen = SCREEN_SPLASH;
static int64_t g_help_closed_us;    /* an OK double click that closed the help must not reopen it */

static inline uint16_t be16(uint16_t v) { return (uint16_t)((v >> 8) | (v << 8)); }   /* the panel takes MSB first */

static void screen_orient(void)
{
    /* landscape: swap x/y, then mirror one axis; the other mirror is the
     * same picture turned 180 degrees */
    esp_lcd_panel_swap_xy(g_panel, true);
    esp_lcd_panel_mirror(g_panel, !g_set.flip, g_set.flip);
}

/* Draw one screen row. esp_lcd's SPI path sends a row's CASET/RASET only
 * after the previous colour transfer has finished, so a buffer is free again
 * once a later row has been queued; cycling NROWBUF of them keeps the one
 * being filled out of flight. */
static uint16_t *next_rowbuf(void)
{
    uint16_t *b = g_rowbuf[g_rowbuf_i];
    g_rowbuf_i = (g_rowbuf_i + 1) % NROWBUF;
    return b;
}

static void fill_rect(int x0, int y0, int x1, int y1, uint16_t rgb)
{
    int y, x;
    for (y = y0; y < y1; y++) {
        uint16_t *b = next_rowbuf();
        for (x = 0; x < x1 - x0; x++) b[x] = be16(rgb);
        esp_lcd_panel_draw_bitmap(g_panel, x0, y, x1, y + 1, b);
    }
}

/* ----------------------------------------------------------- the picture */
typedef struct {
    int on;                         /* a picture has started */
    int w, h;                       /* as sent */
    int dw, dh, ox, oy;             /* on screen */
    uint16_t xmap[SCR_W];           /* screen column -> picture column */
} Place;
static Place g_pl;

static void pic_begin(void *user, const char *mode, int w, int h)
{
    Place *p = &g_pl;
    int x;
    (void)user;
    /* fit: the smaller of the two scales, rounded to whole pixels */
    if ((long)SCR_W * h <= (long)SCR_H * w) { p->dw = SCR_W; p->dh = (int)((long)h * SCR_W / w); }
    else { p->dh = SCR_H; p->dw = (int)((long)w * SCR_H / h); }
    if (p->dw > SCR_W) p->dw = SCR_W;
    if (p->dh > SCR_H) p->dh = SCR_H;
    p->ox = (SCR_W - p->dw) / 2; p->oy = (SCR_H - p->dh) / 2;
    p->w = w; p->h = h; p->on = 1;
    for (x = 0; x < p->dw; x++) p->xmap[x] = (uint16_t)((long)x * w / p->dw);
    memset(g_under, 0, SCR_W * BAR_H * 2);
    g_bar_on = 0;                   /* the whole picture in view; OK brings the bar back */
    g_screen = SCREEN_PICTURE;      /* a picture closes the key help */
    fill_rect(0, 0, SCR_W, SCR_H, 0);
    ESP_LOGI(TAG, "picture: %s %dx%d -> %dx%d at %d,%d", mode, w, h, p->dw, p->dh, p->ox, p->oy);
}

static void pic_row(void *user, int y, const uint16_t *px, int w)
{
    Place *p = &g_pl;
    int d0, d1, d, x;
    uint16_t *b;
    (void)user;
    if (!p->on || g_screen != SCREEN_PICTURE || w != p->w || y < 0 || y >= p->h) return;
    d0 = (int)((long)y * p->dh / p->h);
    d1 = (int)((long)(y + 1) * p->dh / p->h);
    if (d1 <= d0) return;            /* scaled down: this row falls between screen rows */
    b = next_rowbuf();
    for (x = 0; x < p->dw; x++) b[x] = be16(px[p->xmap[x]]);
    for (d = d0; d < d1; d++) {
        int sy = p->oy + d;
        if (sy >= BAR_Y) {           /* keep what is under the bar, for when it goes */
            memcpy(g_under + (sy - BAR_Y) * SCR_W + p->ox, b, (size_t)p->dw * 2);
            if (g_bar_on) continue;
        }
        esp_lcd_panel_draw_bitmap(g_panel, p->ox, sy, p->ox + p->dw, sy + 1, b);
    }
}

/* ------------------------------------------------------------ status bar */
#define C_BAR   0x2104              /* dark grey */
#define C_TEXT  0xFFFF
#define C_DIM   0x8410
#define C_ACC   0xFD20              /* apricot */
#define C_GOOD  0x07E0
#define C_WARN  0xF800

/* The glass has 30 px rounded corners (the BSP masks LVGL frames to the same
 * radius): at the bar's text rows, 2..9 px above the bottom edge, the corner
 * takes 9..19 px off each end, so the bar's content stays BAR_IN inside. */
#define BAR_IN    20
#define METER_W   32
#define X_BATT    (SCR_W - BAR_IN - 4 * 6)
#define X_METER   (X_BATT - 4 - METER_W)
#define BAR_RIGHT (X_METER - 4 - 6 * 6)   /* the tone readout; the left-hand text stops short of it */
static int bar_text_to(int x, const char *s, uint16_t rgb, int xmax)
{
    for (; *s && x + SF_W <= xmax; s++, x += SF_W + 1) {
        int c = (unsigned char)*s, r, i;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c < 0x20 || c > 0x5f) c = '?';
        for (r = 0; r < SF_H; r++) {
            uint8_t bits = sf_font[c - 0x20][r];
            for (i = 0; i < SF_W; i++)
                if (bits & (0x10 >> i)) g_bar[(r + 2) * SCR_W + x + i] = be16(rgb);
        }
    }
    return x;
}
static int bar_text(int x, const char *s, uint16_t rgb) { return bar_text_to(x, s, rgb, BAR_RIGHT - 4); }

static void bar_box(int x0, int y0, int x1, int y1, uint16_t rgb)
{
    int x, y;
    for (y = y0; y < y1; y++) for (x = x0; x < x1; x++) g_bar[y * SCR_W + x] = be16(rgb);
}

typedef struct { int peak_db; int battery; } Meters;

static void bar_draw(const SstvStatus *st, const Meters *mt)
{
    char t[48];
    int x = BAR_IN, lvl;
    bar_box(0, 0, SCR_W, BAR_H, C_BAR);
    if (st->state == SSTV_RX) {
        x = bar_text(x, "RX", C_GOOD);
        snprintf(t, sizeof t, " %s %d/%d", st->mode ? st->mode : "?", st->line, st->height);
        x = bar_text(x, t, C_TEXT);
        if (st->ppm_valid) { snprintf(t, sizeof t, " %+dPPM", st->ppm); x = bar_text(x, t, C_DIM); }
    } else {
        x = bar_text(x, "LISTEN", C_ACC);
        snprintf(t, sizeof t, " %s%s", MODE_LOCKS[g_set.lock].name, g_set.autosync ? "" : " NOAS");
        x = bar_text(x, t, C_TEXT);
        if (st->mode) {
            static const char *END[] = { "", "DONE", "LOST", "STOP", "NEW", "MEM" };
            snprintf(t, sizeof t, " %s %s", st->mode, st->end_kind >= 0 && st->end_kind <= 5 ? END[st->end_kind] : "");
            x = bar_text(x, t, C_DIM);
        }
    }
    /* right: tone, level meter, battery */
    snprintf(t, sizeof t, "%4dHZ", st->tone_hz);
    bar_text_to(BAR_RIGHT, t, C_DIM, SCR_W);
    lvl = (mt->peak_db + 60) * METER_W / 60;         /* -60..0 dBFS across the meter */
    if (lvl < 0) lvl = 0;
    if (lvl > METER_W) lvl = METER_W;
    bar_box(X_METER, 3, X_METER + METER_W, 8, 0x0000);
    bar_box(X_METER, 3, X_METER + lvl, 8, mt->peak_db > -3 ? C_WARN : mt->peak_db > -40 ? C_GOOD : C_DIM);
    if (mt->battery >= 0) { snprintf(t, sizeof t, "%3d%%", mt->battery); bar_text_to(X_BATT, t, C_DIM, SCR_W - BAR_IN + 2); }
    esp_lcd_panel_draw_bitmap(g_panel, 0, BAR_Y, SCR_W, SCR_H, g_bar);
}

static void bar_hide(void)
{
    int r;
    for (r = 0; r < BAR_H; r++) {
        uint16_t *b = next_rowbuf();
        memcpy(b, g_under + r * SCR_W, SCR_W * 2);
        esp_lcd_panel_draw_bitmap(g_panel, 0, BAR_Y + r, SCR_W, BAR_Y + r + 1, b);
    }
}

/* Full-screen images in the panel's byte order, 320 x 240, embedded from
 * main/<name>.rgb565: the boot splash (tools/make_splash.py) and the key help in
 * English and Chinese (tools/make_help.py). Flash can't be read by the SPI
 * DMA, so each row is copied into a row buffer first. */
extern const uint8_t splash_start[]  asm("_binary_splash_rgb565_start");
extern const uint8_t splash_end[]    asm("_binary_splash_rgb565_end");
extern const uint8_t help_en_start[] asm("_binary_help_en_rgb565_start");
extern const uint8_t help_en_end[]   asm("_binary_help_en_rgb565_end");
extern const uint8_t help_zh_start[] asm("_binary_help_zh_rgb565_start");
extern const uint8_t help_zh_end[]   asm("_binary_help_zh_rgb565_end");

static void show_image(const uint8_t *start, const uint8_t *end)
{
    int y;
    if (end - start != SCR_W * SCR_H * 2) {
        ESP_LOGW(TAG, "embedded image has the wrong size");
        fill_rect(0, 0, SCR_W, SCR_H, 0);
        return;
    }
    for (y = 0; y < SCR_H; y++) {
        uint16_t *b = next_rowbuf();
        memcpy(b, start + (size_t)y * SCR_W * 2, SCR_W * 2);
        esp_lcd_panel_draw_bitmap(g_panel, 0, y, SCR_W, y + 1, b);
    }
}

static void show_help(void)
{
    g_screen = SCREEN_HELP;
    if (g_set.lang) show_image(help_zh_start, help_zh_end);
    else show_image(help_en_start, help_en_end);
}

static void close_help(void)
{
    g_screen = SCREEN_PICTURE;
    g_help_closed_us = esp_timer_get_time();
    g_pl.on = 0;                    /* the help covered whatever was on the screen */
    memset(g_under, 0, SCR_W * BAR_H * 2);
    fill_rect(0, 0, SCR_W, SCR_H, 0);
    g_bar_on = 1;
}

/* ------------------------------------------------------------------ keys */
typedef struct { bsp_btn_t btn; bsp_btn_ev_t ev; } Key;
static QueueHandle_t g_keys;

static void on_key(bsp_btn_t btn, bsp_btn_ev_t ev, void *user)
{
    Key k = { btn, ev };
    (void)user;
    if (ev == BSP_BTN_CLICK || ev == BSP_BTN_LONG || ev == BSP_BTN_DOUBLE) xQueueSend(g_keys, &k, 0);
}

static void handle_key(SstvRx *rx, const Key *k, const SstvStatus *st, const Meters *mt)
{
    if (g_screen == SCREEN_HELP) {
        if (k->btn == BSP_BTN_OK) {             /* any OK press closes it, and does only that */
            close_help();
            bar_draw(st, mt);
        } else if (k->ev != BSP_BTN_DOUBLE) {   /* UP / DOWN: the other language */
            g_set.lang = !g_set.lang;
            settings_save();
            show_help();
        }
        return;
    }
    if (k->btn == BSP_BTN_OK && k->ev == BSP_BTN_DOUBLE) {
        if (st->state != SSTV_RX                /* not over a picture coming in */
            && esp_timer_get_time() - g_help_closed_us > 600000) show_help();
        return;
    }
    if (k->btn == BSP_BTN_OK && k->ev == BSP_BTN_CLICK) {
        g_bar_on = !g_bar_on;
        if (g_bar_on) bar_draw(st, mt); else bar_hide();
        return;
    }
    if (k->btn == BSP_BTN_OK && k->ev == BSP_BTN_LONG) {
        sstv_abort(rx); sstv_clear(rx);
        g_pl.on = 0;
        memset(g_under, 0, SCR_W * BAR_H * 2);
        fill_rect(0, 0, SCR_W, SCR_H, 0);
        g_bar_on = 1;
    } else if (k->btn == BSP_BTN_UP && k->ev == BSP_BTN_CLICK) {
        g_set.lock = (uint8_t)((g_set.lock + 1) % NLOCKS);
        sstv_set_mode(rx, MODE_LOCKS[g_set.lock].vis);
        g_bar_on = 1;
    } else if (k->btn == BSP_BTN_UP && k->ev == BSP_BTN_LONG) {
        g_set.flip = !g_set.flip;
        screen_orient();
        fill_rect(0, 0, SCR_W, SCR_H, 0);    /* the picture is gone from the panel; rows redrawn come back */
        g_bar_on = 1;
    } else if (k->btn == BSP_BTN_DOWN && k->ev == BSP_BTN_CLICK) {
        g_set.backlight = (uint8_t)((g_set.backlight + 1) % sizeof BACKLIGHT);
        bsp_display_backlight(BACKLIGHT[g_set.backlight]);
    } else if (k->btn == BSP_BTN_DOWN && k->ev == BSP_BTN_LONG) {
        g_set.autosync = !g_set.autosync;
        sstv_set_autosync(rx, g_set.autosync);
        g_bar_on = 1;
    } else return;
    settings_save();
    if (g_bar_on) bar_draw(st, mt);
}

/* ------------------------------------------------------------- the task */
static void rx_task(void *arg)
{
    SstvRx *rx = (SstvRx *)arg;
    static int16_t buf[CHUNK];
    SstvStatus st;
    Meters mt = { -96, -1 };
    int peak = 0, last_state = SSTV_IDLE;
    int64_t next_status = 0, next_battery = 0;
    Key k;

    sstv_status(rx, &st);
    for (;;) {
        int i;
        int64_t now;
        if (bsp_audio_read(buf, sizeof buf) != ESP_OK) {
            ESP_LOGW(TAG, "mic read failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        for (i = 0; i < CHUNK; i++) { int v = buf[i] < 0 ? -buf[i] : buf[i]; if (v > peak) peak = v; }
        sstv_feed(rx, buf, CHUNK, 1);

        while (xQueueReceive(g_keys, &k, 0) == pdTRUE) handle_key(rx, &k, &st, &mt);

        now = esp_timer_get_time();
        if (now >= next_status) {
            next_status = now + STATUS_US;
            mt.peak_db = peak > 0 ? (int)lrintf(20.0f * log10f((float)peak / 32768.0f)) : -96;
            peak = 0;
            if (now >= next_battery) { next_battery = now + 10 * 1000000LL; mt.battery = bsp_battery_soc(); }
            sstv_status(rx, &st);
            if (last_state == SSTV_RX && st.state == SSTV_IDLE) g_bar_on = 1;   /* picture over: say how */
            last_state = st.state;
            if (g_bar_on && g_screen == SCREEN_PICTURE) bar_draw(&st, &mt);
        }
    }
}

void app_main(void)
{
    SstvRx *rx;
    SstvSink sink = { pic_begin, pic_row };
    int i;
    esp_err_t e;

    ESP_LOGI(TAG, "Apricot SSTV for AI Passport %s", APRICOT_PASSPORT_VERSION);
    e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    settings_load();

    bsp_i2c_init();
    if (bsp_display_init() != ESP_OK || !(g_panel = bsp_display_panel())) {
        ESP_LOGE(TAG, "display init failed");
        return;
    }
    for (i = 0; i < NROWBUF; i++) g_rowbuf[i] = heap_caps_malloc(SCR_W * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    g_bar = heap_caps_malloc(SCR_W * BAR_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    g_under = heap_caps_calloc(SCR_W * BAR_H, 2, MALLOC_CAP_INTERNAL);
    for (i = 0; i < NROWBUF; i++) if (!g_rowbuf[i]) { ESP_LOGE(TAG, "no memory for row buffers"); return; }
    if (!g_bar || !g_under) { ESP_LOGE(TAG, "no memory for the status bar"); return; }
    screen_orient();
    g_screen = SCREEN_SPLASH;
    show_image(splash_start, splash_end);
    bsp_display_backlight(BACKLIGHT[g_set.backlight]);

    g_keys = xQueueCreate(8, sizeof(Key));
    if (!g_keys || bsp_button_init(on_key, NULL) != ESP_OK) ESP_LOGE(TAG, "keys unavailable");
    if (bsp_battery_init() != ESP_OK) ESP_LOGW(TAG, "battery gauge unavailable");

    if (bsp_audio_init() != ESP_OK || bsp_audio_set_format(RATE, 16, 1) != ESP_OK) {
        ESP_LOGE(TAG, "microphone init failed");
        return;
    }

    ESP_LOGI(TAG, "heap before decoder: %u free, %u largest block",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    rx = sstv_new(RATE);
    if (!rx) { ESP_LOGE(TAG, "no memory for the decoder"); return; }
    ESP_LOGI(TAG, "heap after decoder: %u free", (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    sstv_set_sink(rx, &sink, NULL);
    sstv_set_mode(rx, MODE_LOCKS[g_set.lock].vis);
    sstv_set_autosync(rx, g_set.autosync);

    /* the splash stays until a key is pressed; that press does nothing else */
    if (g_keys) {
        Key k;
        while (xQueueReceive(g_keys, &k, portMAX_DELAY) != pdTRUE) { }
        vTaskDelay(pdMS_TO_TICKS(400));             /* the rest of that press (a double click's second half) */
        xQueueReset(g_keys);
    }
    show_help();
    if (xTaskCreate(rx_task, "sstv_rx", 6144, rx, 5, NULL) != pdPASS) ESP_LOGE(TAG, "task create failed");
}
