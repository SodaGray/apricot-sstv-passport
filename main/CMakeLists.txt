# SPDX-License-Identifier: GPL-3.0-only
idf_component_register(
    SRCS "app_main.c" "sstv_dsp.c"
    INCLUDE_DIRS "."
    REQUIRES bsp esp_lcd esp_timer nvs_flash
    EMBED_FILES "splash.rgb565" "help_en.rgb565" "help_zh.rgb565"
)
# re-time at most this many channel-pixels per 16 ms read, so a straightening
# sweep never holds the mic task long enough to overrun the I2S DMA (90 ms)
target_compile_definitions(${COMPONENT_LIB} PRIVATE RD_BUDGET=4096)
