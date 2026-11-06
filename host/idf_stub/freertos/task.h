#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t; typedef void (*TaskFunction_t)(void *);
BaseType_t xTaskCreate(TaskFunction_t fn, const char *const name, const uint32_t stack, void *const arg, UBaseType_t prio, TaskHandle_t *const handle);
void vTaskDelay(const TickType_t ticks);
