# Overhead (C++)

A C++ rewrite of [Overhead](https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-micropython), a live ADS-B flight tracker for the [M5Stack Tab5](https://docs.m5stack.com/en/core/Tab5). Built on PlatformIO + Arduino + M5Unified/M5GFX (no LVGL), with genuine multithreading: the network fetch runs on its own FreeRTOS task pinned to core 0, so the UI never blocks - the thing that wasn't possible on the MicroPython version, where sockets couldn't be created from a secondary thread at all.

| | |
| --- | --- |
| ![The table of aircraft overhead](docs/screenshots/table.png) | ![The radar plot, centred on the nearest airport](docs/screenshots/radar.png) |
| ![The detail screen for one flight](docs/screenshots/detail.png) | ![The settings screen](docs/screenshots/settings.png) |

Captured from the device with `tools/screenshot.py` - see [Screenshots](#screenshots).

## Hardware

- M5Stack Tab5 (5" 1280x720 touchscreen, ESP32-P4 + ESP32-C6)

## Toolchain

Tab5's ESP32-P4 isn't supported by mainline PlatformIO yet - this uses the community [`pioarduino`](https://github.com/pioarduino/platform-espressif32) fork of the espressif32 platform (see `platformio.ini`).

```
pip install platformio
```

## Setup

1. Copy `include/secrets.h.example` to `include/secrets.h` (it is gitignored). Filling in WiFi credentials there is optional: a build left with the placeholder opens the WiFi screen on boot and takes a network on-screen instead, which is how a device flashed from a released image is expected to be set up. Credentials compiled in here are only ever a fallback for a device that has never been given one that way.
2. Build and flash:
   ```
   pio run -t upload --upload-port /dev/ttyACM0
   ```
3. Watch serial output at 115200 baud for boot/WiFi diagnostics if anything looks wrong.

## Screenshots

The Tab5 has no screenshot function of its own, but everything on screen is drawn into a landscape canvas in PSRAM before the PPA rotates it onto the panel - so the firmware answers an `S` on USB serial with that canvas, and `tools/screenshot.py` turns it into a PNG:

```
pip install pyserial pillow
python tools/screenshot.py radar.png
```

It is pixel-exact and the right way up whatever the device's orientation, and takes about three seconds. Close any serial monitor first. The script leaves DTR and RTS as the kernel sets them on open: clearing DTR while RTS is still raised is the ESP32's USB-serial reset signal, which is what pyserial's usual `dtr = False` sends. And it checks that the end marker follows the last pixel exactly, retrying if a log line from the poll task landed mid-transfer and shifted the image.

## Notable hardware quirks this project works around

- **WiFi**: Tab5's WiFi lives on a separate ESP32-C6 co-processor reached over SDIO. The generic P4 eval-board pin defaults don't reach it, so `hostedSetPins(12, 13, 11, 10, 9, 8, 15)` must be called before WiFi initializes (see `main.cpp`). M5Unified is supposed to do this automatically in `M5.begin()`, but it's called explicitly here too.
- **Flash partitioning**: the board's default Arduino partition scheme reserves a tiny (~1.3MB) app partition despite 16MB of flash. `partitions_custom.csv` gives the app a single ~14MB partition instead (no OTA needed for this project).
- **HTTPClient's User-Agent**: `HTTPClient::addHeader("User-Agent", ...)` is silently overridden by an internal default (`ESP32HTTPClient`) - use `setUserAgent()` instead. adsb.lol rejects generic User-Agents outright.
- **mbedTLS stack size**: the background poll task needs a considerably larger stack (16KB) than a typical FreeRTOS task, or HTTPS requests fail silently.
- **Speaker startup**: `M5.begin()` configures the Tab5's ES8388 codec and enables its amp, but stops short of starting the I2S output - nothing is audible until `M5.Speaker.begin()` is called as well (see `src/sound.cpp`).
- **Rotated framebuffer**: the panel is physically 720x1280 portrait, so in the landscape orientation this app runs at, every horizontal line of the UI is a *column* in memory. LovyanGFX's rotated `pushSprite` can't memcpy in that case and walks the image pixel by pixel - a full-screen push measures **650ms** on this device. `src/screen.cpp` hands the rotation to the ESP32-P4's PPA (its 2D graphics accelerator) instead; see below.

## Header

The left of the bar says what is being shown, where from, and who it came from - `ADSB Flights - Civilian (Nearest airport GLA)   via adsb.lol` - and the right carries the three controls: radar, mute, and the cog. The airport is the nearest one to the configured location from the same table the radar centres on, and is left off entirely when nothing is within range of it.

The source is named because it can change on its own: under **Auto** the tracker fails over between providers, and a table quietly being served by the other one should say so rather than leave it to be guessed. It appears once a poll has succeeded, and changing it forces the repaint that draws it.

The controls sit together at the right edge rather than scattered across the bar, which leaves the whole left side to that one line. Muting is a header control rather than a settings one because it is the thing most likely to be wanted in a hurry.

## The table

Seven columns: flight, type, altitude, speed, distance, heading and status.

Altitude is given in feet up to 9999 and as a flight level above that - `FL200` for 20000ft, hundreds of feet as the convention has it. Past ten thousand the exact figure is neither how the altitude gets referred to nor worth the width of five digits, and the column is narrower for it.

Heading is the reported ground track in three digits, `035` rather than `35`, the way a heading is written and spoken. Status carries a drawn icon ahead of its word, so the column reads at a glance without the text having to be parsed.

Widths are fixed rather than measured: the content of each column is known and bounded - eight characters of callsign, four of designator, five of flight level - so there is nothing to be gained from measuring at runtime, and a table whose columns don't move between refreshes is easier to read. They are sized against the widest realistic value in each column, in the table's font: FLIGHT fits eight `W`s, the widest callsign there can be (272px), where it used to be too narrow for `LOG27VQ`; STATUS fits `DESCEND` behind its icon; HDG never holds more than three digits, and had been given room for seven. A value that is still too wide is drawn at a smaller size until it fits, rather than running over the border into the next cell.

A callsign the transponder sent as blank arrives from readsb as a row of `@`s. Those are stripped, and one that was nothing else shows as `UNKNOWN`; an aircraft with no callsign field at all is shown by its ICAO hex, as before.

## Settings

The cog opens a settings screen in two columns - seven controls will not stack down 720px of height, and 1280px of width was going spare:

- **Traffic filter** - civilian or military, as an either/or choice rather than two independent switches. Military aircraft are the ones readsb sets bit 0 of `dbFlags` on - or, on a feed that doesn't carry that field, the ones a military-only endpoint returned.
- **Range** - a slider whose ceiling follows the filter above it: 60nm for civil traffic, 150nm for military, since military traffic is worth watching further out. Switching to civil with the slider up high clamps it back down.
- **Show flight refresh** - whether cells that changed on the last poll are shaded for a moment (see Rendering below). On by default.
- **Data source** - which provider to poll: **Auto**, or one of them pinned. Auto is the default and the reason this control exists - see [Data sources](#data-sources). Pinning is for when you would rather know which one you are looking at than have it chosen for you; a pinned provider's empty answer is reported as it stands, with no second opinion sought.
- **Refresh interval** - how often the sky is refetched, from 5 to 60 seconds in fives, defaulting to 30. The slider is stepped rather than continuous, with a detent mark per position, so it can't be left on a value nobody asked for. Below five seconds the endpoint starts refusing; past a minute the table is stale enough that a slower dial wouldn't be asked for. The default is deliberately not the fastest the dial allows - see [Data sources](#data-sources).
- **Location** - the place search, which used to be a button filling half the main header. It is a setting rather than a permanent fixture of the table: it gets changed once when the device moves and then not again. Both finishing and cancelling return here rather than to the table, so a new location lands you back on the button that shows it took.
- **WiFi** - scans for networks and connects to one, so the device can move between networks without a reflash. Credentials are saved to NVS and win over the ones compiled in from `secrets.h`, which stay as the fallback for a device that's never had WiFi set on-screen. A device with neither - a fresh flash of a released image - opens this screen on boot rather than spending the connect timeout proving it has nothing to connect with, and Back from there lands on the table. Credentials that are merely wrong, or an access point that is down, are left to the poll task to retry, since being thrown into setup over a router reboot would be worse than the status line saying what is happening.

All of it persists across reboots, along with the mute state and the chosen location. Changing the filter, the range or the source refetches immediately rather than waiting out the rest of the interval; changing the interval itself doesn't - the new value simply applies to the wait already running, including shortening one in progress.

## Polling

These endpoints return around forty fields per aircraft and this app reads fifteen, so the fetch hands ArduinoJson a `DeserializationOption::Filter` and the rest are skipped in the tokeniser rather than allocated into the document and then ignored. Where the response declares a `Content-Length` the document is parsed straight off the socket; where it doesn't, it goes through `getString()` first, because `HTTPClient` de-chunks on the `getString()`/`writeToStream()` paths but *not* on the raw stream - a chunked reply read directly still has the chunk headers in it.

The HTTPS client is held for the life of the firmware rather than built per fetch, with `setReuse(true)`, so the socket stays open between polls and each fetch skips DNS, TCP and the TLS handshake. `HTTPClient` stores the client by reference and its `clear()` touches neither the socket nor the reuse flag, so this needs nothing more than keeping both objects alive. Measured with `-DNET_PROFILE`:

| | before | after |
| --- | --- | --- |
| fetch, start to parsed | ~340ms | ~90ms |

The document itself is allocated from PSRAM through a custom `ArduinoJson::Allocator`, keeping the largest thing this task holds off the 512KB of internal RAM that the WiFi and TLS stacks compete for. PSRAM is the slower memory, so this could have cost parse time; measured, it didn't - parse sits at 25-29ms either way, because with a streaming parse that figure is mostly the body still arriving rather than the document being written. The filter document stays in internal RAM: it is a handful of keys and not worth the slower memory.

The build uses `-O2` rather than the Arduino default of `-Os`, which on a synthetic 66KB, 150-aircraft response cut the filtered parse from 79ms to 45ms. The same comparison left drawing unchanged - a full radar repaint draws in ~22ms and the table in ~44ms at either level, because writing into the PSRAM canvas is bound by the memory rather than the instructions - so the gain is the poll task's, for about 90KB of flash. The precompiled framework (WiFi, TLS, drivers) is untouched by either.

The first fetch after a boot still pays the full handshake, and a connection the far end has closed in the meantime shows up as a transport error on the next request rather than at the time it was dropped - so a *connection-level* failure retries once on a fresh socket. Only a connection-level one: a read timeout means the server has the request and is simply slow, and sending it again would put two of the same request on an endpoint that is already throttling. The read timeout is 12s rather than the 5s default for the same reason - a served request takes about 45ms, but a throttled one can take several seconds and is still worth waiting for. The standing cost is one mbedTLS context resident (about 380 bytes of static RAM here) instead of one built and torn down every ten seconds.

Providers answer `429 Too Many Requests` once they have had enough, and a poll that collects one is followed by a minute in which no request goes out to *that* provider at all, rather than by more of the same at the usual cadence. The backoff is per provider, so one throttling the device doesn't sideline the other. If 429s are a standing feature rather than an occasional one, `POLL_INTERVAL_MS` in `include/config.h` is the dial to turn - ten seconds is not guaranteed to be within what the endpoint will serve, and a long session of reflashing (each boot polls immediately) is enough to trip it.

The kept-alive socket is closed when a poll is aimed at a different host, since reusing it would send the request down a connection to the wrong server.

The status line under the table says how old the data is, and turns amber when the last poll didn't land, so a quiet sky can be told apart from a dead network. A link that drops is reconnected from the poll task with a 5s-to-2min backoff, standing down while the WiFi screen is up, since that screen drives the radio itself.

## Aircraft types

The `desc` field is always null on the endpoints this uses, so the detail screen's full aircraft name is filled in locally from `src/aircraft_db.cpp`. There are two tables: civil types, and around 85 military ones covering transports, tankers, surveillance, combat, trainers, helicopters and drones.

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

Which is also why vertical movement is a shape rather than a colour or a shade: both of those were already spoken for, and neither was free to take on a second meaning without muddling the first. A contact that is climbing carries a chevron above the blip and one that is descending carries it below, pointing the way the aircraft is going; level and ground traffic carry nothing at all, so the plot stays quiet when nothing is doing anything vertically. The footer names the two marks rather than leaving them to be worked out, stacked above the contact count in the bottom right: the plot is still 165px wide at the footer's height, so a legend appended to the range line on the left ran straight into it. Each of the three readouts sits in its own boxed cell the way a scope's data blocks do, drawn in the same dim green as the range rings so the boxes read as furniture and the text inside them as the reading.

That is roughly what a real secondary-radar display does, for the same reason - the scopes those plots were drawn on had one phosphor and no colour to spend, so trend information went into the symbol.

A data refresh redraws only what can change: the plot square, and the footer readouts beside it. The header and the margins either side of the plot stay as they were drawn on arrival, which halves the area pushed to the panel, and the square is cleared by the PPA rather than the CPU - 15ms of `fillRect` into PSRAM, where the hardware fills the whole screen in 5. Measured with 60 contacts, a refresh went from 64ms to 41ms; arriving at the screen, or toggling its centre, is still the full 64ms repaint. Contacts are clipped to the square so that nothing is drawn outside what a refresh erases - at short range a fast aircraft's vector can otherwise run well past the outer ring.

Callsigns are placed nearest-first and one may not overlap another already placed, so in a cluster the closest aircraft keeps its label and the rest stay as bare blips. The footer says how many were plotted and how many of those are labelled, so a thinned display doesn't read as a missing one.

The code at the centre is the nearest airport, from a generated table of the 3244 large and medium airports with scheduled service in the public-domain [OurAirports](https://ourairports.com/data/) dataset (~39KB of flash). A full scan takes 6.7ms, so the result is cached until the location changes rather than recomputed per frame. Nothing within 120nm leaves the centre as a bare cross.

The button beside Back chooses what the plot is centred on, and is labelled with the current choice: **AIRPORT**, the default, puts that airport at the centre with its code under the cross; **HOME** centres on the configured location itself and marks it with a bare `+`. The choice persists. Airport mode with nothing in range falls back to home. The aircraft are still fetched around home, so with the plot centred on an airport some way off, the edge of the plot furthest from home can sit beyond the fetch radius and show empty.

## Sound

A two-note rise once the firmware is up, and a short blip whenever an aircraft that wasn't there before appears in the table - one blip per poll however many arrived, and never on the first poll after a start or a location change, where every aircraft is new by definition.

The on-screen keyboard ticks on each key - 25ms at 3kHz, on a channel of its own that cuts off the tick before it, so fast typing doesn't queue up behind itself or behind an arrival blip.

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

A full repaint of the table, returning to it from another screen, draws in 33ms and pushes in 42ms. The drawing used to be 44ms, half of it filling cell backgrounds that `clear()` had just filled with the same colour - narrow rects are slow to write into a canvas in PSRAM. A cell with no cached value is now known to be background already and gets only its border and text. A cell being redrawn is filled inside its border rather than over it, so the border is drawn once rather than on every refresh. A per-poll refresh is bound by the push rather than the drawing: an update of four columns across eleven rows takes ~17ms, and taking the border redraws out of it saved only half a millisecond - the bulk is the PPA rotating those rows into the panel, which already runs at its maximum burst length.

Rendering also means a refresh where nothing changed costs nothing at all, and the three screens share the one canvas rather than holding 1.8MB each.

## Data sources

Live aircraft come from two providers, either of which can serve the whole app:

- [adsb.lol](https://adsb.lol), whose data is made available under the [Open Database License 1.0](https://opendatacommons.org/licenses/odbl/1-0/).
- [adsb.fi](https://adsb.fi), whose open data API is free to use without a key, for personal and non-commercial use, and asks that adsb.fi be cited with a link to its home page - which this is.

The firmware carries none of that data - it is fetched at runtime and displayed - so what appears on screen is an ODbL Produced Work, and the attribution is the obligation that comes with it.

### Why there are two

adsb.lol spent an evening answering `200 OK` with an empty aircraft array. Its API was up and its responses were well-formed; the feed behind it had drained, and its own `/0/me` reported `global.aircraft: 0` while `/v2/all` returned 503. Sampled at 8-second intervals, the point endpoint returned nothing for about eighty seconds and then refilled, cycling:

```
22:00:57  lol_point=0   lol_global=0       lol_all=503  | fi_point=6
   ...     (nine consecutive polls, total: 0)
22:02:22  lol_point=4   lol_global=3690    lol_all=503  | fi_point=6
22:02:39  lol_point=5   lol_global=10726   lol_all=503  | fi_point=6
```

At the point of parsing, that is indistinguishable from an empty sky - which is exactly what the table said, for several polls in a row. A failed request would have left the last known data up (see below); only a *successful* empty response produces "No aircraft in range".

So a single source is a single point of failure that fails silently, and **Auto** does something about it:

- Sticky, not round-robin: whichever provider last answered is asked first, so one that has gone down doesn't cost a 12-second timeout on every poll for the duration of the outage.
- A provider that fails outright is skipped to the next one.
- An empty result that follows a *non-empty* one is checked against the other provider before it reaches the screen. If that one finds traffic, it takes over and says so in the header. If both agree the sky is empty, it is empty.
- Only on that transition, so a genuinely quiet sky costs one request per poll rather than two.

### What a source has to describe

A source is a pair of endpoints rather than a URL, because providers disagree about more than their hostname:

| | adsb.lol | adsb.fi |
| --- | --- | --- |
| civil endpoint | `/v2/point/{lat}/{lon}/{radius}` | `/v2/lat/{lat}/lon/{lon}/dist/{radius}` |
| military endpoint | the same point query | `/v2/mil`, global |
| array key | `ac` | `aircraft` (point), `ac` (mil) |
| carries `dbFlags` | yes | no (point), yes (mil) |

Which is why `include/config.h` holds a table of `AdsbProvider` rather than a format string, and why each endpoint carries its own array key: adsb.fi's two really are differently shaped, not one path with a variant.

Where an endpoint omits `dbFlags` there is nothing to test, so the endpoint itself is the answer - everything from a military feed is military, nothing from a civil one is. And a *global* endpoint hasn't applied the radius, so the client does: it drops anything outside it, and anything it cannot place at all. That last part matters more than it sounds. adsb.fi's military feed carried 232 aircraft when this was written, 50 of them with no `lat`/`lon` at all; without that rule every one would have landed in a table captioned as showing traffic within a few dozen miles.

The URL templates use `{lat}`/`{lon}`/`{radius}` substitution rather than printf formatting. A format string that reaches `snprintf` from anywhere but a literal is how a `double` gets read as a pointer, and these are data - a mis-spelled placeholder can only ever produce a wrong URL.

### Paid alternatives, and why there are none here

Worth recording, since it looks like the obvious fix and isn't. A paid tier buys quota, not uptime, and the quota on offer doesn't fit a device that polls around the clock: 30-second polling is ~86,400 requests a month, where ADS-B Exchange's $10 Community tier allows 10,000 - about one poll every four minutes. [OpenSky](https://opensky-network.org)'s free tier is generous enough (4,000 credits a day, 1 per small bounding box, so a poll every 21s) but its state vectors carry no type designator and no military flag, which would empty the TYPE column and the military filter outright. [airplanes.live](https://airplanes.live) is the same readsb schema *with* `dbFlags` and would be a genuine drop-in, but its public endpoint currently answers 403 and asks you to get in touch first.

The reliable answer, if this ever needs one, is not a subscription but a receiver: an RTL-SDR running readsb on the same network serves `aircraft.json` in this exact schema with no rate limit, no TLS and no outages.

That endpoint is free, volunteer-run infrastructure, and the default refresh interval is set with that in mind rather than at the fastest the hardware or the dial would allow. `DEFAULT_POLL_INTERVAL_S` is 30 seconds: an aircraft covers perhaps three miles in that time, which at these ranges moves a blip by a few pixels, so the cost to the display is slight and the cost to the server is a third of what ten seconds asks of it. The dial is there for anyone who wants it faster on their own account - what matters is that every device doesn't take that by default.

`POLL_INTERVAL_MIN_S` stops the slider going below five seconds, and either provider answers `429` well before that if it has had enough - adsb.fi documents a limit of one request per second; a poll that collects one is followed by a minute's silence from that provider rather than more of the same. If you are flashing this onto several devices, leave them slower still.

Place search is [Open-Meteo's geocoding API](https://open-meteo.com/), free for non-commercial use, its data licensed [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

The airport table is generated from [OurAirports](https://ourairports.com/data/), which is released into the public domain.

Aircraft type designators are checked against ICAO Doc 8643. Airline names and colours identify the operator and imply no endorsement by it.

## Licence

MIT - see [LICENSE](LICENSE). The libraries it builds on (M5Unified, M5GFX, ArduinoJson) are MIT too; note that the ESP32 Arduino core is LGPL, which is worth reading up on before distributing compiled binaries rather than source.

## Layout

- `src/main.cpp` - setup/loop, WiFi, the background poll task, screen state, touch dispatch
- `src/screen.cpp` - the shared canvas, dirty-region tracking and the PPA-accelerated push to the panel
- `src/display.cpp` - main table rendering, with per-cell diffing so only changed cells are repainted
- `src/location_screen.cpp` - location search screen: text entry and results list, reached from the settings screen
- `src/settings_screen.cpp` - the cog screen: filters, the data source, the range and interval sliders, and the way in to location and WiFi
- `src/wifi_screen.cpp` - network scan, passphrase entry and connection
- `src/keyboard.cpp` - the on-screen keyboard shared by the location and WiFi screens
- `src/adsb_client.cpp` - provider polling, failover and aircraft parsing
- `src/aircraft_db.cpp` - ICAO type code and operator lookups for the detail screen
- `src/geocode.cpp` - Open-Meteo location search
- `src/settings.cpp` - persists location, filters and WiFi credentials via ESP32 `Preferences` (NVS)
- `src/radar_screen.cpp` - the radar plot: range rings, bearings, contacts and vectors
- `src/airports.cpp` - generated nearest-airport lookup, for the code at the centre of the plot
- `src/sound.cpp` - boot and new-arrival beeps through the built-in speaker
- `include/config.h` - tunable constants, and the ADS-B provider table
