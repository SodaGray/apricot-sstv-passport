#pragma once
#include <stddef.h>
#include <stdint.h>
#define MALLOC_CAP_DMA (1<<3)
#define MALLOC_CAP_8BIT (1<<2)
#define MALLOC_CAP_INTERNAL (1<<11)
void *heap_caps_malloc(size_t size, uint32_t caps);
void *heap_caps_calloc(size_t n, size_t size, uint32_t caps);
size_t heap_caps_get_free_size(uint32_t caps);
size_t heap_caps_get_largest_free_block(uint32_t caps);
