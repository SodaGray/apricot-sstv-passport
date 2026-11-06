#pragma once
#include <stdio.h>
#define ESP_LOGI(tag, fmt, ...) ((void)(tag), printf(fmt "\n", ##__VA_ARGS__))
#define ESP_LOGW ESP_LOGI
#define ESP_LOGE ESP_LOGI
