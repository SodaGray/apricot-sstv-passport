/* SPDX-License-Identifier: GPL-3.0-only */
/* sim_app.c -- runs the firmware's app_main.c on a computer: the panel is a
 * 320 x 240 frame buffer, the microphone plays encoded SSTV audio, keys come
 * from a script. Writes what the screen shows to <out>/screen_*.ppm.
 * Usage: sim_app <outdir> <mode> [<mode> ...]   (VIS codes, sent back to back) */
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "bsp_audio.h"
#include "bsp_battery.h"
#include "bsp_button.h"
#include "bsp_display.h"
#include "bsp_i2c.h"
#include "enc.h"
#include "bench_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void app_main(void);
int ppm_write_565(const char *path, const uint16_t *px, int w, int h);

static uint16_t g_fb[240][320];
static int g_swap;
static long g_draws, g_draw_px;
static const char *g_out = "out";
static int16_t *g_audio; static long g_n, g_pos;
static int g_rate;
static bsp_btn_cb_t g_keycb;

/* ---- panel */
esp_lcd_panel_handle_t bsp_display_panel(void) { return (esp_lcd_panel_handle_t)1; }
esp_lcd_panel_io_handle_t bsp_display_io(void) { return 0; }
esp_err_t bsp_display_init(void) { return ESP_OK; }
void bsp_display_backlight(uint8_t p) { (void)p; }
esp_err_t esp_lcd_panel_swap_xy(esp_lcd_panel_handle_t p, bool s) { (void)p; g_swap = s; return ESP_OK; }
esp_err_t esp_lcd_panel_mirror(esp_lcd_panel_handle_t p, bool x, bool y) { (void)p; (void)x; (void)y; return ESP_OK; }
esp_err_t esp_lcd_panel_draw_bitmap(esp_lcd_panel_handle_t p, int x0, int y0, int x1, int y1, const void *data)
{
    const uint16_t *d = (const uint16_t *)data;
    int x, y;
    (void)p;
    if (!g_swap || x0 < 0 || y0 < 0 || x1 > 320 || y1 > 240 || x1 <= x0 || y1 <= y0) {
        fprintf(stderr, "bad draw %d,%d-%d,%d swap=%d\n", x0, y0, x1, y1, g_swap); exit(4);
    }
    for (y = y0; y < y1; y++) for (x = x0; x < x1; x++) {
        uint16_t v = *d++;
        g_fb[y][x] = (uint16_t)((v >> 8) | (v << 8));   /* back from panel byte order */
    }
    g_draws++; g_draw_px += (long)(x1 - x0) * (y1 - y0);
    return ESP_OK;
}

/* ---- memory, time, rtos, nvs */
void *heap_caps_malloc(size_t n, uint32_t c) { (void)c; return malloc(n); }
void *heap_caps_calloc(size_t n, size_t s, uint32_t c) { (void)c; return calloc(n, s); }
size_t heap_caps_get_free_size(uint32_t c) { (void)c; return 300000; }
size_t heap_caps_get_largest_free_block(uint32_t c) { (void)c; return 200000; }
int64_t esp_timer_get_time(void) { return g_rate ? (int64_t)g_pos * 1000000 / g_rate : 0; }
static void snap(const char *name);
void vTaskDelay(const TickType_t t) { (void)t; }
BaseType_t xTaskCreate(TaskFunction_t fn, const char *const name, const uint32_t st, void *const arg, UBaseType_t pr, TaskHandle_t *const h)
{ (void)name; (void)st; (void)pr; (void)h; fn(arg); return pdPASS; }
typedef struct { int n, head, len, size; char buf[8][16]; } Q;
QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t sz) { Q *q = calloc(1, sizeof *q); q->size = (int)sz; (void)len; return q; }
BaseType_t xQueueSend(QueueHandle_t qh, const void *it, TickType_t w)
{ Q *q = qh; (void)w; if (q->len == 8) return 0; memcpy(q->buf[(q->head + q->len++) % 8], it, (size_t)q->size); return pdTRUE; }
BaseType_t xQueueReset(QueueHandle_t qh) { Q *q = qh; q->len = 0; return pdTRUE; }
/* app_main waits on the splash for a key: take a picture of it, then press OK */
BaseType_t xQueueReceive(QueueHandle_t qh, void *it, TickType_t w)
{
    Q *q = qh;
    if (!q->len && w == portMAX_DELAY && g_keycb) { snap("splash"); g_keycb(BSP_BTN_OK, BSP_BTN_CLICK, 0); }
    if (!q->len) return 0;
    memcpy(it, q->buf[q->head], (size_t)q->size); q->head = (q->head + 1) % 8; q->len--; return pdTRUE;
}
esp_err_t nvs_flash_init(void) { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { return ESP_OK; }
esp_err_t nvs_open(const char *n, nvs_open_mode_t m, nvs_handle_t *h) { (void)n; (void)m; *h = 1; return ESP_OK; }
esp_err_t nvs_get_blob(nvs_handle_t h, const char *k, void *v, size_t *l) { (void)h; (void)k; (void)v; (void)l; return ESP_FAIL; }
esp_err_t nvs_set_blob(nvs_handle_t h, const char *k, const void *v, size_t l) { (void)h; (void)k; (void)v; (void)l; return ESP_OK; }
esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }
void nvs_close(nvs_handle_t h) { (void)h; }

/* ---- BSP */
esp_err_t bsp_i2c_init(void) { return ESP_OK; }
esp_err_t bsp_battery_init(void) { return ESP_OK; }
int bsp_battery_soc(void) { return 87; }
esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *u) { (void)u; g_keycb = cb; return ESP_OK; }
esp_err_t bsp_audio_init(void) { return ESP_OK; }
esp_err_t bsp_audio_set_format(uint32_t hz, uint8_t bits, uint8_t ch) { (void)bits; (void)ch; g_rate = (int)hz; return ESP_OK; }

static void snap(const char *name)
{
    char p[512];
    snprintf(p, sizeof p, "%s/screen_%s.ppm", g_out, name);
    ppm_write_565(p, &g_fb[0][0], 320, 240);
}

esp_err_t bsp_audio_read(void *pcm, size_t bytes)
{
    long n = (long)(bytes / 2), i;
    int16_t *o = (int16_t *)pcm;
    static long next_snap = 0; static int snaps = 0;
    if (g_pos >= g_n) {
        static int once;
        if (once++) { snap("end_nobar"); printf("draw calls %ld, %ld px\n", g_draws, g_draw_px); exit(0); }
        snap("end");
        if (g_keycb) g_keycb(BSP_BTN_OK, BSP_BTN_CLICK, 0);   /* OK click: bar off, then a last frame */
        memset(o, 0, bytes);
        return ESP_OK;
    }
    {
        static int reads;
        reads++;
        if (reads == 1) { snap("help_en"); g_keycb(BSP_BTN_UP, BSP_BTN_CLICK, 0); }
        if (reads == 2) { snap("help_zh"); if (!getenv("SIM_NOCLOSE")) g_keycb(BSP_BTN_OK, BSP_BTN_CLICK, 0); }
    }
    for (i = 0; i < n; i++) o[i] = g_pos + i < g_n ? g_audio[g_pos + i] : 0;
    g_pos += n;
    if (g_pos >= next_snap) {                   /* a frame every 20 s of audio */
        char nm[16]; snprintf(nm, sizeof nm, "t%02d", snaps++); snap(nm);
        next_snap = g_pos + 20L * g_rate;
    }
    return ESP_OK;
}

int main(int argc, char **argv)
{
    int i;
    if (argc < 3) { fprintf(stderr, "usage: sim_app outdir mode...\n"); return 2; }
    g_out = argv[1];
    g_rate = 16000;
    for (i = 2; i < argc; i++) {
        EncOpts o; int w, h; long n; uint8_t *pic; int16_t *a;
        enc_default_opts(&o);
        o.mode = atoi(argv[i]); o.rate = 16000; o.vis_code = o.mode; o.snr_db = 25; o.skew_ppm = 150;
        o.offset_hz = 30; o.lead_s = 1.5; o.tail_s = 1.5; o.amplitude = 6000; o.seed = (unsigned)i;
        if (!enc_mode_size(o.mode, &w, &h)) { fprintf(stderr, "mode %d?\n", o.mode); return 2; }
        pic = bench_picture(w, h);
        a = enc_encode(pic, &o, &n);
        g_audio = enc_concat(g_audio, g_n, a, n, &g_n);
        free(a); free(pic);
    }
    app_main();
    return 0;
}
