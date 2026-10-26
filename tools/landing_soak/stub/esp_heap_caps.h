// The PSRAM allocator's calls, on the PC's ordinary heap.
#pragma once
#include <cstdlib>
#define MALLOC_CAP_SPIRAM 0
#define MALLOC_CAP_INTERNAL 0
#define MALLOC_CAP_8BIT 0
inline void *heap_caps_malloc(size_t n, unsigned) { return malloc(n); }
inline void *heap_caps_realloc(void *p, size_t n, unsigned) { return realloc(p, n); }
inline void heap_caps_free(void *p) { free(p); }
