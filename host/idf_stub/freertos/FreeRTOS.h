#pragma once
#include <stdint.h>
typedef uint32_t TickType_t; typedef long BaseType_t; typedef unsigned long UBaseType_t;
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
