#pragma once

#include <esp_heap_caps.h>
#include <stdlib.h>

// A std:: allocator that puts its storage in PSRAM, for the containers that
// grow with a recording or a video - a replay's index, an MP4's frame table -
// and would otherwise take internal RAM the WiFi and TLS stacks need. Falls
// back to the ordinary heap if PSRAM is out.
template <typename T>
struct PsramAllocator {
    using value_type = T;
    PsramAllocator() = default;
    template <typename U>
    PsramAllocator(const PsramAllocator<U> &) {}
    T *allocate(size_t n) {
        void *p = heap_caps_malloc(n * sizeof(T), MALLOC_CAP_SPIRAM);
        return static_cast<T *>(p ? p : malloc(n * sizeof(T)));
    }
    void deallocate(T *p, size_t) { heap_caps_free(p); }
    template <typename U>
    bool operator==(const PsramAllocator<U> &) const { return true; }
    template <typename U>
    bool operator!=(const PsramAllocator<U> &) const { return false; }
};
