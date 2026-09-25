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

## Header

The left of the bar says what is being shown and where from - `ADSB Flights - Civilian (Nearest airport GLA)` - and the right carries the three controls: radar, mute, and the cog. The airport is the nearest one to the configured location from the same table the radar centres on, and is left off entirely when nothing is within range of it.

The controls sit together at the right edge rather than scattered across the bar, which leaves the whole left side to that one line. Muting is a header control rather than a settings one because it is the thing most likely to be wanted in a hurry.

## Settings

The cog opens a settings screen in two columns - six controls will not stack down 720px of height, and 1280px of width was going spare:

- **Traffic filter** - civilian or military, as an either/or choice rather than two independent switches. Military aircraft are the ones adsb.lol sets bit 0 of `dbFlags` on.
- **Range** - a slider whose ceiling follows the filter above it: 60nm for civil traffic, 150nm for military, since military traffic is worth watching further out. Switching to civil with the slider up high clamps it back down.
- **Show flight refresh** - whether cells that changed on the last poll are shaded for a moment (see Rendering below). On by default.
- **Refresh interval** - how often the sky is refetched, from 5 to 60 seconds in fives. The slider is stepped rather than continuous, with a detent mark per position, so it can't be left on a value nobody asked for. Below five seconds the endpoint starts refusing; past a minute the table is stale enough that a slower dial wouldn't be asked for.
- **Location** - the place search, which used to be a button filling half the main header. It is a setting rather than a permanent fixture of the table: it gets changed once when the device moves and then not again. Both finishing and cancelling return here rather than to the table, so a new location lands you back on the button that shows it took.
- **WiFi** - scans for networks and connects to one, so the device can move between networks without a reflash. Credentials are saved to NVS and win over the ones compiled in from `secrets.h`, which stay as the fallback for a device that's never had WiFi set on-screen.

All of it persists across reboots, along with the mute state and the chosen location. Changing the filter or the range refetches immediately rather than waiting out the rest of the interval; changing the interval itself doesn't - the new value simply applies to the wait already running, including shortening one in progress.

## Polling

adsb.lol returns around forty fields per aircraft and this app reads fifteen, so the fetch hands ArduinoJson a `DeserializationOption::Filter` and the rest are skipped in the tokeniser rather than allocated into the document and then ignored. Where the response declares a `Content-Length` the document is parsed straight off the socket; where it doesn't, it goes through `getString()` first, because `HTTPClient` de-chunks on the `getString()`/`writeToStream()` paths but *not* on the raw stream - a chunked reply read directly still has the chunk headers in it.

The HTTPS client is held for the life of the firmware rather than built per fetch, with `setReuse(true)`, so the socket stays open between polls and each fetch skips DNS, TCP and the TLS handshake. `HTTPClient` stores the client by reference and its `clear()` touches neither the socket nor the reuse flag, so this needs nothing more than keeping both objects alive. Measured with `-DNET_PROFILE`:

| | before | after |
| --- | --- | --- |
| fetch, start to parsed | ~340ms | ~90ms |

The document itself is allocated from PSRAM through a custom `ArduinoJson::Allocator`, keeping the largest thing this task holds off the 512KB of internal RAM that the WiFi and TLS stacks compete for. PSRAM is the slower memory, so this could have cost parse time; measured, it didn't - parse sits at 25-29ms either way, because with a streaming parse that figure is mostly the body still arriving rather than the document being written. The filter document stays in internal RAM: it is a handful of keys and not worth the slower memory.

The first fetch after a boot still pays the full handshake, and a connection the far end has closed in the meantime shows up as a transport error on the next request rather than at the time it was dropped - so a *connection-level* failure retries once on a fresh socket. Only a connection-level one: a read timeout means the server has the request and is simply slow, and sending it again would put two of the same request on an endpoint that is already throttling. The read timeout is 12s rather than the 5s default for the same reason - a served request takes about 45ms, but a throttled one can take several seconds and is still worth waiting for. The standing cost is one mbedTLS context resident (about 380 bytes of static RAM here) instead of one built and torn down every ten seconds.

adsb.lol answers `429 Too Many Requests` once it has had enough, and a poll that collects one is followed by a minute in which no request goes out at all, rather than by more of the same at the usual cadence. If 429s are a standing feature rather than an occasional one, `POLL_INTERVAL_MS` in `include/config.h` is the dial to turn - ten seconds is not guaranteed to be within what the endpoint will serve, and a long session of reflashing (each boot polls immediately) is enough to trip it.

The status line under the table says how old the data is, and turns amber when the last poll didn't land, so a quiet sky can be told apart from a dead network. A link that drops is reconnected from the poll task with a 5s-to-2min backoff, standing down while the WiFi screen is up, since that screen drives the radio itself.

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

## Radar

The radar button in the header opens a plan-position plot centred on the configured location, in phosphor green: range rings on round numbers, bearing marks every 30 degrees, and every contact that reported a position as a blip with a vector showing a minute of flight at its current groundspeed. A tap on a blip opens that aircraft's detail screen.

A single hue is deliberate - on a plot like this brightness is what carries meaning, which leaves it free to mark a new arrival as the brightest thing on screen.

Callsigns are placed nearest-first and one may not overlap another already placed, so in a cluster the closest aircraft keeps its label and the rest stay as bare blips. The footer says how many were plotted and how many of those are labelled, so a thinned display doesn't read as a missing one.

The code at the centre is the nearest airport, from a generated table of the 3244 large and medium airports with scheduled service in the public-domain [OurAirports](https://ourairports.com/data/) dataset (~39KB of flash). A full scan takes 6.7ms, so the result is cached until the location changes rather than recomputed per frame. Nothing within 120nm leaves the centre as a bare cross.

## Sound

A two-note rise once the firmware is up, and a short blip whenever an aircraft that wasn't there before appears in the table - one blip per poll however many arrived, and never on the first poll after a start or a location change, where every aircraft is new by definition.

The speaker icon in the header mutes it, and the setting persists. Muting silences the beeps rather than shutting the speaker down, so unmuting needs no re-initialisation - and unmuting plays the arrival blip, which is the one confirmation that can only be given in the medium being switched back on. `SOUND_ENABLED` and `SOUND_VOLUME` in `include/config.h` remain the build-time "never make a sound" and the level.

## Rendering

Everything draws into one shared landscape 1280x720 canvas in PSRAM, never straight to the panel. Drawing there is unrotated, so it's ordinary memcpy work. Each draw marks the region it touched, and `screen::flush()` hands just those regions to the PPA, which rotates them into the panel's framebuffer by DMA.

Measured on hardware (`-DRENDER_PROFILE` in `platformio.ini` logs the per-flush times):

| | before | after |
| --- | --- | --- |
| live data refresh | 650ms | 5-16ms |
| keypress on the location screen | 650ms | <1ms |
| full-screen repaint (boot, screen change) | 650ms | 42ms |
| clearing the canvas | 42ms (CPU) | 5ms (PPA fill) |

One thing that was tried and rejected: queueing the rects as `PPA_TRANS_MODE_NON_BLOCKING` and waiting only on the last, so the CPU fills the queue instead of making a round trip per transfer. Normalised against the work each flush actually did, it measured *slower* - 16.8us per KB against 12.5 - so the transfers are not waiting on the CPU, and the queue plus the completion interrupt cost more than the round trip they replaced. The blit stays blocking. What did come out of that attempt is worth keeping: `ppa_do_scale_rotate_mirror`'s return value is now checked, where a failure used to pass silently and leave that region of the panel showing the previous frame.

Two details worth knowing if you touch `screen.cpp`: the PPA's RGB565 byte order is the opposite of the one LovyanGFX uses for this panel (hence `byte_swap` on every blit, and the pre-swapped fill colour, both checked against what LovyanGFX itself reads back), and the canvas has to be flushed out of the CPU cache before the PPA's DMA can see it.

That cache flush used to be the floor under every push. `esp_cache_msync` is a range operation - it becomes `Cache_WriteBack_Addr(vaddr, size)`, run over both the L1 D-cache and L2 - so its cost follows the range it is given, and flushing all 1.8MB of the canvas cost the same whether the transfer that followed was the whole screen or one cell. With a 128KB L2 (`CONFIG_CACHE_L2_CACHE_SIZE`) at most ~2,000 lines of that canvas can be dirty, yet the sync was walking 28,800 lines' worth of addresses.

`flush()` now writes back only the rows its dirty rects cover, merged so cells sharing a table row sync once between them. A canvas row is 2560 bytes - a whole number of 64-byte cache lines - so every span is naturally aligned. Measured over ~100s of live traffic with `-DRENDER_PROFILE`:

| | before | after |
| --- | --- | --- |
| smallest flush | 4.07ms | 0.69ms |
| median flush | 9.19ms | 7.17ms (table) / 0.96ms (status line) |

The floor is what moved: a small update no longer pays for the whole canvas. A refresh that touches cells across eight table rows still syncs most of it, and there the PPA transfer dominates anyway.

This is only safe because of an invariant the screens already keep: every canvas write is covered by the dirty list at the flush that follows it - a partial redraw marks the region it touched, a full repaint goes through `clear()`, which marks the lot. Rows outside the list were therefore written back by an earlier flush and are already clean in PSRAM, which is also what lets the merged bounding-box blit stay correct. A screen that drew without marking would now show stale pixels rather than merely wasting a sync, so keep marking.

The poll task also trims each result set, nearest first, to the sixty contacts the radar will plot - a 60nm radius over a busy area returns well over a hundred aircraft, and carrying the long tail through a vector copy per refresh bought nothing. The cap is the radar's rather than the table's, since the table draws only the eleven rows that fit but the radar plots the lot; the result is published by `std::move`, so the only copy left per refresh is the one `loop()` takes to render from. Trimming in the poll task rather than at draw time also keeps "new arrival" meaning a new *row*, rather than beeping at every aircraft that enters the radius unseen.

The per-cell diffing earns its keep twice over: a cell whose value hasn't moved is never redrawn, and the cells that *have* moved are shaded for two seconds so a change is visible without having to watch for it. That costs one extra flush per poll - about 24ms in every 10 seconds, or a quarter of one percent of the time. `CELL_HIGHLIGHT_MS` in `include/config.h` sets how long the shading lasts, and the *Show flight refresh* toggle on the settings screen turns it off.

Rendering also means a refresh where nothing changed costs nothing at all, and the three screens share the one canvas rather than holding 1.8MB each.

## Layout

- `src/main.cpp` - setup/loop, WiFi, the background poll task, screen state, touch dispatch
- `src/screen.cpp` - the shared canvas, dirty-region tracking and the PPA-accelerated push to the panel
- `src/display.cpp` - main table rendering, with per-cell diffing so only changed cells are repainted
- `src/location_screen.cpp` - location search screen: text entry and results list, reached from the settings screen
- `src/settings_screen.cpp` - the cog screen: filters, the range and interval sliders, and the way in to location and WiFi
- `src/wifi_screen.cpp` - network scan, passphrase entry and connection
- `src/keyboard.cpp` - the on-screen keyboard shared by the location and WiFi screens
- `src/adsb_client.cpp` - adsb.lol polling and aircraft parsing
- `src/aircraft_db.cpp` - ICAO type code and operator lookups for the detail screen
- `src/geocode.cpp` - Open-Meteo location search
- `src/settings.cpp` - persists location, filters and WiFi credentials via ESP32 `Preferences` (NVS)
- `src/radar_screen.cpp` - the radar plot: range rings, bearings, contacts and vectors
- `src/airports.cpp` - generated nearest-airport lookup, for the code at the centre of the plot
- `src/sound.cpp` - boot and new-arrival beeps through the built-in speaker
- `include/config.h` - tunable constants
