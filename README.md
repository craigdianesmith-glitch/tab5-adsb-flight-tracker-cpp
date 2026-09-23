# Overhead (C++)

A C++ rewrite of [Overhead](https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-micropython), a live ADS-B flight tracker for the [M5Stack Tab5](https://docs.m5stack.com/en/core/Tab5). Built on PlatformIO + Arduino + M5Unified/M5GFX (no LVGL), with genuine multithreading: the network fetch runs on its own FreeRTOS task pinned to core 0, so the UI never blocks - the thing that wasn't possible on the MicroPython version, where sockets couldn't be created from a secondary thread at all.

## Hardware

- M5Stack Tab5 (5" 1280x720 touchscreen, ESP32-P4 + ESP32-C6)

## Toolchain

Tab5's ESP32-P4 isn't supported by mainline PlatformIO yet - this uses the community [`pioarduino`](https://github.com/pioarduino/platform-espressif32) fork of the espressif32 platform (see `platformio.ini`).

```
pip install platformio
```

## Setup

1. Copy `include/secrets.h.example` to `include/secrets.h` and fill in your WiFi credentials (`secrets.h` is gitignored).
2. Build and flash:
   ```
   pio run -t upload --upload-port /dev/ttyACM0
   ```
3. Watch serial output at 115200 baud for boot/WiFi diagnostics if anything looks wrong.

## Notable hardware quirks this project works around

- **WiFi**: Tab5's WiFi lives on a separate ESP32-C6 co-processor reached over SDIO. The generic P4 eval-board pin defaults don't reach it, so `hostedSetPins(12, 13, 11, 10, 9, 8, 15)` must be called before WiFi initializes (see `main.cpp`). M5Unified is supposed to do this automatically in `M5.begin()`, but it's called explicitly here too.
- **Flash partitioning**: the board's default Arduino partition scheme reserves a tiny (~1.3MB) app partition despite 16MB of flash. `partitions_custom.csv` gives the app a single ~14MB partition instead (no OTA needed for this project).
- **HTTPClient's User-Agent**: `HTTPClient::addHeader("User-Agent", ...)` is silently overridden by an internal default (`ESP32HTTPClient`) - use `setUserAgent()` instead. adsb.lol rejects generic User-Agents outright.
- **mbedTLS stack size**: the background poll task needs a considerably larger stack (16KB) than a typical FreeRTOS task, or HTTPS requests fail silently.
- **Rotated framebuffer**: the panel is physically 720x1280 portrait, so in the landscape orientation this app runs at, every horizontal line of the UI is a *column* in memory. LovyanGFX's rotated `pushSprite` can't memcpy in that case and walks the image pixel by pixel - a full-screen push measures **650ms** on this device. `src/screen.cpp` hands the rotation to the ESP32-P4's PPA (its 2D graphics accelerator) instead; see below.

## Rendering

Everything draws into one shared landscape 1280x720 canvas in PSRAM, never straight to the panel. Drawing there is unrotated, so it's ordinary memcpy work. Each draw marks the region it touched, and `screen::flush()` hands just those regions to the PPA, which rotates them into the panel's framebuffer by DMA.

Measured on hardware (`-DRENDER_PROFILE` in `platformio.ini` logs the per-flush times):

| | before | after |
| --- | --- | --- |
| live data refresh | 650ms | 5-16ms |
| keypress on the location screen | 650ms | <1ms |
| full-screen repaint (boot, screen change) | 650ms | 42ms |
| clearing the canvas | 42ms (CPU) | 5ms (PPA fill) |

Two details worth knowing if you touch `screen.cpp`: the PPA's RGB565 byte order is the opposite of the one LovyanGFX uses for this panel (hence `byte_swap` on every blit, and the pre-swapped fill colour, both checked against what LovyanGFX itself reads back), and the canvas has to be flushed out of the CPU cache before the PPA's DMA can see it.

Rendering also still does per-cell diffing, so a refresh where nothing changed costs nothing at all, and the three screens share the one canvas rather than holding 1.8MB each.

## Layout

- `src/main.cpp` - setup/loop, WiFi, the background poll task, screen state, touch dispatch
- `src/screen.cpp` - the shared canvas, dirty-region tracking and the PPA-accelerated push to the panel
- `src/display.cpp` - main table rendering, with per-cell diffing so only changed cells are repainted
- `src/location_screen.cpp` - location search screen: text entry, on-screen keyboard, results list
- `src/adsb_client.cpp` - adsb.lol polling and aircraft parsing
- `src/geocode.cpp` - Open-Meteo location search
- `src/settings.cpp` - persists the chosen location via ESP32 `Preferences` (NVS)
- `include/config.h` - tunable constants
