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
- **Speaker startup**: `M5.begin()` configures the Tab5's ES8388 codec and enables its amp, but stops short of starting the I2S output - nothing is audible until `M5.Speaker.begin()` is called as well (see `src/sound.cpp`).
- **Rotated framebuffer**: the panel is physically 720x1280 portrait, so in the landscape orientation this app runs at, every horizontal line of the UI is a *column* in memory. LovyanGFX's rotated `pushSprite` can't memcpy in that case and walks the image pixel by pixel - a full-screen push measures **650ms** on this device. `src/screen.cpp` hands the rotation to the ESP32-P4's PPA (its 2D graphics accelerator) instead; see below.

## Settings

The cog in the header opens a settings screen holding:

- **Traffic filter** - civilian or military, as an either/or choice rather than two independent switches. Military aircraft are the ones adsb.lol sets bit 0 of `dbFlags` on.
- **Range** - a slider whose ceiling follows the filter above it: 60nm for civil traffic, 150nm for military, since military traffic is worth watching further out. Switching to civil with the slider up high clamps it back down.
- **Show flight refresh** - whether cells that changed on the last poll are shaded for a moment (see Rendering below). On by default.
- **WiFi** - scans for networks and connects to one, so the device can move between networks without a reflash. Credentials are saved to NVS and win over the ones compiled in from `secrets.h`, which stay as the fallback for a device that's never had WiFi set on-screen.

Filters, range, the refresh toggle and WiFi credentials all persist across reboots alongside the chosen location. Changing any of them refetches immediately rather than waiting out the rest of the poll interval.

## Aircraft types

adsb.lol's `desc` field is always null on the endpoints this uses, so the detail screen's full aircraft name is filled in locally from `src/aircraft_db.cpp`. There are two tables: civil types, and around 85 military ones covering transports, tankers, surveillance, combat, trainers, helicopters and drones.

Which is consulted first depends on the aircraft's own military flag, because a good number of ICAO designators cover both - `EC45` is an air ambulance or a US Army UH-72 Lakota, `BE20` a King Air or a C-12 Huron, `B762` a 767-200 or a KC-46 Pegasus. The other table is still searched as a fallback, so a type listed in only one resolves either way.

Every designator in both tables is checked against the ICAO doc 8643 list rather than written from memory, which is how four bad entries came to light - each one a row that could never have matched an aircraft:

| was | problem | now |
| --- | --- | --- |
| `RC135` | designators are four characters at most | `R135` |
| `TYPH` | not a designator | `EUFI` |
| `E175` | not a designator; the E175 is split by wing | `E75L` + `E75S` |
| `PA28` | not a designator | `P28A` (already present) |

## Sound

A two-note rise once the firmware is up, and a short blip whenever an aircraft that wasn't there before appears in the table - one blip per poll however many arrived, and never on the first poll after a start or a location change, where every aircraft is new by definition. `SOUND_ENABLED` and `SOUND_VOLUME` in `include/config.h` turn it off or change the level.

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

The poll task also trims each result set to the number of rows that actually fit (eleven at this size), nearest first - a 60nm radius over a busy area returns well over a hundred aircraft, and carrying the other ninety through two vector copies per refresh bought nothing. Trimming there rather than at draw time also keeps "new arrival" meaning a new *row*, rather than beeping at every aircraft that enters the radius unseen.

The per-cell diffing earns its keep twice over: a cell whose value hasn't moved is never redrawn, and the cells that *have* moved are shaded for two seconds so a change is visible without having to watch for it. That costs one extra flush per poll - about 24ms in every 10 seconds, or a quarter of one percent of the time. `CELL_HIGHLIGHT_MS` in `include/config.h` sets how long the shading lasts, and the *Show flight refresh* toggle on the settings screen turns it off.

Rendering also means a refresh where nothing changed costs nothing at all, and the three screens share the one canvas rather than holding 1.8MB each.

## Layout

- `src/main.cpp` - setup/loop, WiFi, the background poll task, screen state, touch dispatch
- `src/screen.cpp` - the shared canvas, dirty-region tracking and the PPA-accelerated push to the panel
- `src/display.cpp` - main table rendering, with per-cell diffing so only changed cells are repainted
- `src/location_screen.cpp` - location search screen: text entry and results list
- `src/settings_screen.cpp` - the cog screen: traffic filters and the range slider
- `src/wifi_screen.cpp` - network scan, passphrase entry and connection
- `src/keyboard.cpp` - the on-screen keyboard shared by the location and WiFi screens
- `src/adsb_client.cpp` - adsb.lol polling and aircraft parsing
- `src/aircraft_db.cpp` - ICAO type code and operator lookups for the detail screen
- `src/geocode.cpp` - Open-Meteo location search
- `src/settings.cpp` - persists location, filters and WiFi credentials via ESP32 `Preferences` (NVS)
- `src/sound.cpp` - boot and new-arrival beeps through the built-in speaker
- `include/config.h` - tunable constants
