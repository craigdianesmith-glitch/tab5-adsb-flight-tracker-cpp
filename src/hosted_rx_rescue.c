// The Tab5's WiFi stall mended where it starts: ESP-Hosted's receive buffers
// never let fail (esp-hosted-mcu #243).
//
// The Tab5 reaches its WiFi through an ESP32-C6 over SDIO, by ESP-Hosted. In
// its streaming mode ESP-Hosted 2.12.13 - the version in Arduino-ESP32 3.3 -
// grows the buffer it reads the C6's bursts into as the bursts grow, from
// internal DMA-capable memory only: a small part of the P4's memory, which a
// busy app can use up. When that allocation fails, the retry it is given
// never runs - it waits on an interrupt it has itself already cleared - and
// the P4 never reads from the C6 again until the C6 is reset. WiFi.status()
// goes on saying connected, every request fails, nothing is logged. On the
// bench, internal DMA memory held down to 2.8KB stalled it within a second,
// every time, and it was still stalled three minutes after the memory was
// given back. ESP-Hosted 2.13.1 allocates the buffers up front instead, but
// only for a C6 that reports its transmit queue, which the factory C6
// firmware on a Tab5 (1.4.1) does not.
//
// So the allocation is never let fail. The linker sends ESP-Hosted's calls to
// heap_caps_aligned_alloc() here (-Wl,--wrap=heap_caps_aligned_alloc, in
// platformio.ini), and one from its SDIO tasks for internal DMA memory that
// cannot be had is given DMA-capable PSRAM instead: just what ESP-Hosted
// does itself, first, when built with CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM
// - see hosted_malloc_align() in its port_esp_hosted_host_os.c. The P4's DMA
// reaches PSRAM through the cache, and the Arduino libraries' 64-byte L2
// cache line keeps such buffers legal (esp-hosted-mcu #219). The same covers
// the packets the stream is split into (task sdio_rx_buf), whose allocation
// failing drops a packet - perhaps the C6 saying it has left the network.
//
// Everything else is as it was: every allocation that succeeds, and every
// one not from ESP-Hosted's SDIO tasks or not for exactly the memory it asks
// for. A drop-in for any ESP32-P4 project on ESP-Hosted 2.12.x over SDIO:
// this file and the one linker flag.
#include <stdint.h>
#include <string.h>

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// What hosted_malloc_align() asks for, and what it asks for instead with
// PREFER_SPIRAM.
#define HOSTED_INTERNAL_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT)
#define HOSTED_PSRAM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT)

void *__real_heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps);

// How many of ESP-Hosted's allocations were given PSRAM, and the size of the
// last: for the log, by way of hostedRxRescues().
static volatile uint32_t s_rescues = 0, s_lastSize = 0;

void *__wrap_heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps)
{
    void *p = __real_heap_caps_aligned_alloc(alignment, size, caps);
    if (p != NULL || caps != HOSTED_INTERNAL_CAPS || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return p;
    }
    // ESP-Hosted's SDIO tasks: sdio_read, which grows the stream buffer, and
    // sdio_rx_buf, which splits it into packets.
    const char *task = pcTaskGetName(NULL);
    if (task == NULL || strncmp(task, "sdio_", 5) != 0) {
        return p;
    }
    p = __real_heap_caps_aligned_alloc(alignment, size, HOSTED_PSRAM_CAPS);
    if (p != NULL) {
        s_lastSize = size;
        s_rescues = s_rescues + 1;
    }
    return p;
}

uint32_t hostedRxRescues(uint32_t *lastSize)
{
    if (lastSize != NULL) {
        *lastSize = s_lastSize;
    }
    return s_rescues;
}
