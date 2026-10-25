#ifdef BENCH3D
// The rest of the app timed on the device - see bench2d.h - each part on its
// own, with made-up traffic where it takes traffic, so one build's figures
// can be set against another's.

#include "bench2d.h"

#include <M5Unified.h>
#include <esp_chip_info.h>
#include <esp_clk_tree.h>
#include <esp_private/esp_clk.h>
#include <esp_cpu.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <soc/cache_reg.h>
#include <soc/soc.h>

#include <algorithm>
#include <vector>

#include "adsb_client.h"
#include "coast.h"
#include "display.h"
#include "names.h"
#include "radar_screen.h"
#include "screen.h"
#include "view3d.h"

// Kept beside what they time - see each.
void adsbBenchParse();  // adsb_client.cpp
void soundBench();      // sound.cpp
void airportsBench();   // airports.cpp

namespace {

struct Place {
    const char *name;
    double lat, lon;
};
const Place PLACES[] = {{"GLA", 55.8719, -4.4331}, {"LHR", 51.4700, -0.4543}, {"JFK", 40.6413, -73.7781}};

// The least, the mean and the most of a run of times, in ms.
struct Times {
    uint32_t n = 0, sum = 0, least = UINT32_MAX, most = 0;
    void add(uint32_t us) {
        n++;
        sum += us;
        least = std::min(least, us);
        most = std::max(most, us);
    }
    float mean() const { return n ? sum / 1000.0f / n : 0; }
    float min() const { return n ? least / 1000.0f : 0; }
    float max() const { return most / 1000.0f; }
};

}  // namespace

std::vector<Aircraft> benchMadeUp(int n, double lat, double lon, float spreadNm, float movedS) {
    std::vector<Aircraft> out;
    out.reserve(n);
    uint32_t seed = 12345;
    auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return (seed >> 8) / 16777216.0f;
    };
    static const char *const AIRLINES[] = {"EZY", "BAW", "RYR", "LOG", "TOM", "KLM"};
    static const char *const TYPES[] = {"A320", "B738", "E190", "A21N", "DH8D", "B789"};
    double perLon = 60.0 * cos(lat * M_PI / 180.0);
    for (int i = 0; i < n; i++) {
        Aircraft a;
        char text[16];
        snprintf(text, sizeof text, "~b%04x", i);
        a.hex = text;
        snprintf(text, sizeof text, "%s%d", AIRLINES[i % 6], 100 + i);
        a.callsign = text;
        a.type = TYPES[i % 6];
        float bearing = rnd() * 2 * (float)M_PI, dist = sqrtf(rnd()) * spreadNm, track = rnd() * 360;
        int kt = 140 + (int)(rnd() * 330);
        bool low = i % 3 == 0;
        int alt = low ? 1000 + (int)(rnd() * 3000) : 8000 + (int)(rnd() * 30000);
        float moved = kt * movedS / 3600.0f, t = track * (float)M_PI / 180.0f;
        a.hasPos = true;
        a.lat = (float)(lat + (dist * cosf(bearing) + moved * cosf(t)) / 60.0);
        a.lon = (float)(lon + (dist * sinf(bearing) + moved * sinf(t)) / perLon);
        a.hasDist = true;
        a.distNm = haversineNm(lat, lon, a.lat, a.lon);
        a.hasTrack = true;
        a.track = track;
        a.altStr = String(alt);
        a.speedStr = String(kt);
        a.hasVertRate = true;
        a.vertRate = low ? -700 : 0;
        a.status = low ? "DESCEND" : "LEVEL";
        a.category = "A3";
        a.squawk = "4521";
        out.push_back(std::move(a));
    }
    std::sort(out.begin(), out.end(), [](const Aircraft &a, const Aircraft &b) { return a.distNm < b.distNm; });
    return out;
}

namespace {

void benchMap() {
    auto &canvas = screen::canvas();
    uint16_t *pixels = (uint16_t *)canvas.getBuffer();
    const float RANGES[] = {5, 10, 25, 60, 150, 250, 500, 1000, 2000};
    Serial.printf("[bench] the map's first draw, %d thinned levels built, each onto the plot filled first\n",
                  coastLevels());
    for (const Place &p : PLACES) {
        for (float r : RANGES) {
            CoastPlot plot = {pixels, canvas.width(), 640, 362, 342, p.lat, p.lon, r, 0xFFFF};
            Times t;
            int strokes = 0;
            for (int i = 0; i < 3; i++) {
                // The plot's square filled by the PPA, which leaves the CPU's
                // cache holding none of it - as the live screen's fill does
                // before the map goes on - rather than drawn over the last
                // run's, still in the cache.
                screen::fillRect(640 - 342, 362 - 342, 2 * 342 + 1, 2 * 342 + 1, 0);
                uint32_t t0 = micros();
                strokes = coastDraw(plot);
                t.add(micros() - t0);
            }
            Serial.printf("[bench] map %s %4.0fnm  %6.2fms (least %.2f, most %.2f)  %d strokes\n", p.name, r, t.mean(),
                          t.min(), t.max(), strokes);
        }
    }
}

void benchNames() {
    const float RANGES[] = {10, 25, 60, 150, 250, 500, 1000, 2000};
    std::vector<PlacedName> out;
    for (const Place &p : PLACES) {
        for (float r : RANGES) {
            NamePlot plot = {640, 362, 342, p.lat, p.lon, r};
            auto clear = [](int, int, int, int) { return false; };
            out.clear();
            uint32_t t0 = micros();
            namesPlace(plot, clear, out);
            uint32_t first = micros() - t0;
            Times again;
            for (int i = 0; i < 5; i++) {
                out.clear();
                t0 = micros();
                namesPlace(plot, clear, out);
                again.add(micros() - t0);
            }
            Serial.printf("[bench] names %s %4.0fnm  first %6.2fms, again %6.3fms  %u placed\n", p.name, r,
                          first / 1000.0f, again.mean(), (unsigned)out.size());
        }
    }
}

// The radar's plot or the airport zoom's (`code` set), as the live screens
// draw it: whole, without the push and with it, and then refreshed, the
// traffic moved on two seconds a time.
void plotRuns(const char *what, const String &code, double lat, double lon, int rangeNm, int n, float spreadNm) {
    std::vector<Aircraft> aircraft = benchMadeUp(n, lat, lon, spreadNm, 0);
    std::vector<uint8_t> isNew(aircraft.size(), 0);
    RadarScene scene{aircraft, isNew, nullptr, lat, lon, rangeNm, RadarCentre::AIRPORT, true};
    scene.nowMs = millis();
    auto draw = [&]() {
        if (code.length()) {
            radarZoomPlotDraw(scene, code, false);
        } else {
            radarPlotDraw(scene, false);
        }
    };
    radarTakeTimings();
    Times whole, pushed, refresh;
    scene.partial = false;
    scene.push = false;
    for (int i = 0; i < 3; i++) {
        uint32_t t0 = micros();
        draw();
        whole.add(micros() - t0);
    }
    scene.push = true;
    for (int i = 0; i < 3; i++) {
        uint32_t t0 = micros();
        draw();
        pushed.add(micros() - t0);
    }
    String parts = radarTakeTimings();
    // The second whole draw of a view keeps its background; after that a
    // refresh with the traffic moved is of the traffic alone, where it is
    // cheap enough to be.
    scene.partial = true;
    draw();
    draw();
    for (int i = 1; i <= 6; i++) {
        aircraft = benchMadeUp(n, lat, lon, spreadNm, 2.0f * i);
        scene.nowMs = millis();
        uint32_t t0 = micros();
        draw();
        refresh.add(micros() - t0);
    }
    Serial.printf("[bench] %s %3dnm %3d aircraft  whole %6.2fms, pushed %6.2fms, refresh %6.2fms (most %.2f)  [%s]\n",
                  what, rangeNm, n, whole.mean(), pushed.mean(), refresh.mean(), refresh.max(), parts.c_str());
}

void benchRadar() {
    const Place &home = PLACES[0];
    for (int range : {10, 25, 60}) {
        for (int n : {0, 20, 60, 120}) {
            plotRuns("radar", String(), home.lat, home.lon, range, n, range * 0.9f);
        }
    }
}

// The airport zoom as it is live: opened, then drawn as the screen draws it
// - its first draw whole, its second whole keeping the background - and then
// refreshed, the traffic moved on two seconds a time.
void benchZoom() {
    for (const char *code : {"GLA", "LHR"}) {
        double lat, lon;
        float rangeNm;
        if (!radarZoomOpen(code, lat, lon, rangeNm)) {
            Serial.printf("[bench] no zoom of %s\n", code);
            continue;
        }
        for (int n : {0, 12, 40}) {
            radarTakeTimings();
            std::vector<Aircraft> aircraft = benchMadeUp(n, lat, lon, rangeNm * 1.2f, 0);
            uint32_t t0 = micros();
            radarZoomDraw(aircraft, false);
            uint32_t first = micros() - t0;
            t0 = micros();
            radarZoomDraw(aircraft, false);
            uint32_t second = micros() - t0;
            Times refresh;
            for (int i = 1; i <= 6; i++) {
                aircraft = benchMadeUp(n, lat, lon, rangeNm * 1.2f, 2.0f * i);
                t0 = micros();
                radarZoomDraw(aircraft, false);
                refresh.add(micros() - t0);
            }
            Serial.printf("[bench] zoom %s %.1fnm %2d aircraft  whole %6.2fms, whole keeping it %6.2fms, refresh "
                          "%6.2fms (most %.2f)  [%s]\n",
                          code, rangeNm, n, first / 1000.0f, second / 1000.0f, refresh.mean(), refresh.max(),
                          radarTakeTimings().c_str());
        }
    }
}

void benchTable() {
    std::vector<Aircraft> aircraft = benchMadeUp(40, PLACES[0].lat, PLACES[0].lon, 30, 0);
    std::vector<uint8_t> isNew(aircraft.size(), 0);
    Times whole, cells, same;
    for (int i = 0; i < 3; i++) {
        displayInvalidate();
        uint32_t t0 = micros();
        displayRenderAircraft(aircraft, isNew);
        whole.add(micros() - t0);
    }
    for (int i = 1; i <= 5; i++) {
        for (size_t k = 0; k < aircraft.size(); k += 3) {
            aircraft[k].altStr = String(aircraft[k].altStr.toInt() + 100 * i);
        }
        uint32_t t0 = micros();
        displayRenderAircraft(aircraft, isNew);
        cells.add(micros() - t0);
    }
    for (int i = 0; i < 5; i++) {
        uint32_t t0 = micros();
        displayRenderAircraft(aircraft, isNew);
        same.add(micros() - t0);
    }
    Serial.printf("[bench] table %d rows  whole %.2fms, a third of the heights changed %.2fms, unchanged %.2fms\n",
                  displayMaxRows(), whole.mean(), cells.mean(), same.mean());
}

// --- the code's misses -----------------------------------------------------------
// The instruction caches' counters around one frame of the radar's plot: each
// core's L1 misses, and the L2's misses for each L1 - code fetched from flash,
// some 666 cycles a line - with the frame's cycles on this core, the loop's.
// A frame whole and pushed, a refresh of the traffic alone and a following
// frame, each after one of its own kind; then again after the canvas has been
// read through the caches, as a frame's pixels go through the L2 live and
// leave it holding none of the code.

struct CodeMisses {
    uint32_t l1[2], l2[2], cycles;
};

uint32_t g_sink;

template <typename F>
CodeMisses codeMisses(F frame) {
    REG_WRITE(CACHE_L1_CACHE_ACS_CNT_CTRL_REG, REG_READ(CACHE_L1_CACHE_ACS_CNT_CTRL_REG) | CACHE_L1_IBUS0_CNT_ENA |
                                                   CACHE_L1_IBUS1_CNT_ENA | CACHE_L1_IBUS0_CNT_CLR |
                                                   CACHE_L1_IBUS1_CNT_CLR);
    REG_WRITE(CACHE_L2_CACHE_ACS_CNT_CTRL_REG, REG_READ(CACHE_L2_CACHE_ACS_CNT_CTRL_REG) | CACHE_L2_IBUS0_CNT_ENA |
                                                   CACHE_L2_IBUS1_CNT_ENA | CACHE_L2_IBUS0_CNT_CLR |
                                                   CACHE_L2_IBUS1_CNT_CLR);
    uint32_t t0 = esp_cpu_get_cycle_count();
    frame();
    CodeMisses m;
    m.cycles = esp_cpu_get_cycle_count() - t0;
    m.l1[0] = REG_READ(CACHE_L1_IBUS0_ACS_MISS_CNT_REG);
    m.l1[1] = REG_READ(CACHE_L1_IBUS1_ACS_MISS_CNT_REG);
    m.l2[0] = REG_READ(CACHE_L2_IBUS0_ACS_MISS_CNT_REG);
    m.l2[1] = REG_READ(CACHE_L2_IBUS1_ACS_MISS_CNT_REG);
    return m;
}

void readCanvasThrough() {
    const volatile uint32_t *p = (const volatile uint32_t *)screen::canvas().getBuffer();
    uint32_t sum = 0;
    for (size_t i = 0; i < (size_t)1280 * 720 / 2; i += 16) {  // a word a 64-byte line
        sum += p[i];
    }
    g_sink = sum;
}

void benchCodeMisses() {
    const Place &home = PLACES[0];
    constexpr int RANGE = 40, N = 60;
    float moved = 0;
    std::vector<Aircraft> aircraft = benchMadeUp(N, home.lat, home.lon, RANGE * 0.9f, moved);
    std::vector<uint8_t> isNew(aircraft.size(), 0);
    RadarScene scene{aircraft, isNew, nullptr, home.lat, home.lon, RANGE, RadarCentre::AIRPORT, true};
    scene.nowMs = millis();
    int core = xPortGetCoreID();
    auto report = [&](const char *what, const CodeMisses &m) {
        Serial.printf("[bench] code %-28s core %d: L1 %6u, L2 %5u (~%.2fms at 666 cycles); core %d: L1 %6u, L2 %5u;"
                      "  the frame %.2fms\n",
                      what, core, (unsigned)m.l1[core], (unsigned)m.l2[core], m.l2[core] * 666.0f / 360000.0f,
                      1 - core, (unsigned)m.l1[1 - core], (unsigned)m.l2[1 - core], m.cycles / 360000.0f);
    };
    auto next = [&] {  // the traffic moved on, outside the count
        moved += 2;
        aircraft = benchMadeUp(N, home.lat, home.lon, RANGE * 0.9f, moved);
        scene.nowMs = millis();
    };
    auto draw = [&] { radarPlotDraw(scene, false); };
    Serial.printf("[bench] the instruction caches' misses round a frame, radar %dnm, %d aircraft, on core %d\n",
                  RANGE, N, core);
    scene.partial = false;
    draw();
    next();
    report("whole, pushed", codeMisses(draw));
    next();
    readCanvasThrough();
    report("whole, after the canvas read", codeMisses(draw));
    // A refresh: the background kept by the second whole draw of the view.
    scene.partial = true;
    draw();
    draw();
    next();
    draw();
    next();
    report("refresh", codeMisses(draw));
    next();
    readCanvasThrough();
    report("refresh, after the canvas read", codeMisses(draw));
    // Following one: centred on it, ringed, never a refresh.
    scene.partial = false;
    scene.centre = RadarCentre::HOME;
    scene.followHex = aircraft[N / 2].hex;
    auto onFollowed = [&] {
        for (const Aircraft &a : aircraft) {
            if (a.hex == scene.followHex) {
                scene.lat = a.lat;
                scene.lon = a.lon;
            }
        }
    };
    onFollowed();
    draw();
    next();
    onFollowed();
    report("following", codeMisses(draw));
    next();
    onFollowed();
    readCanvasThrough();
    report("following, after the canvas read", codeMisses(draw));
}

// --- big pushes through internal RAM -----------------------------------------------
// screen::pushStrips() against the push flush() makes today - the PPA reading
// the canvas from the PSRAM - for the plot's column and the whole screen, the
// latter in half strips too: five of each, timed, and the panel read back
// after each way and set against the other's, pixel for pixel, and the PPA's
// own against the canvas. Each run's pattern differs from its neighbours'
// pixels and from the last run's, and the panel is given another before the
// strips go, so that a strip left unpushed or misplaced shows.

uint16_t patternAt(int x, int y, uint32_t seed) {
    uint32_t v = (uint32_t)x * 2654435761u ^ (uint32_t)y * 40503u ^ seed * 97u;
    v ^= v >> 15;
    v *= 2246822519u;
    v ^= v >> 13;
    return (uint16_t)v;
}

void fillPattern(int x, int y, int w, int h, uint32_t seed) {
    uint16_t *canvas = (uint16_t *)screen::canvas().getBuffer();
    for (int cy = y; cy < y + h; cy++) {
        for (int cx = x; cx < x + w; cx++) {
            canvas[(size_t)cy * 1280 + cx] = patternAt(cx, cy, seed);
        }
    }
    screen::markDirty(x, y, w, h);
}

void benchStrips() {
    struct Case {
        const char *name;
        int x, y, w, h, widest;
    };
    const Case CASES[] = {{"the plot's column", 292, 0, 696, 720, 0},
                          {"the whole screen", 0, 0, 1280, 720, 0},
                          {"the whole screen, half strips", 0, 0, 1280, 720, 640}};
    if (view3d::isOpen()) {
        Serial.println("[bench] strips: not with the 3D view open, which the strips never are");
        return;
    }
    constexpr size_t MOST = (size_t)1280 * 720;
    uint16_t *bySrm = (uint16_t *)heap_caps_malloc(MOST * 2, MALLOC_CAP_SPIRAM);
    uint16_t *byStrips = (uint16_t *)heap_caps_malloc(MOST * 2, MALLOC_CAP_SPIRAM);
    if (bySrm == nullptr || byStrips == nullptr) {
        Serial.println("[bench] strips: no room in the PSRAM to read the panel back into");
        heap_caps_free(bySrm);
        heap_caps_free(byStrips);
        return;
    }
    const uint16_t *canvas = (const uint16_t *)screen::canvas().getBuffer();
    for (const Case &k : CASES) {
        Times srm, blit, sync, strips;
        screen::StripPerf perf, each;
        uint32_t apart = 0, srmWrong = 0, pushed = 0;
        size_t n = (size_t)k.w * k.h;
        for (int run = 0; run < 5; run++) {
            // As flush() pushes it today: the cache written back, then the PPA.
            fillPattern(k.x, k.y, k.w, k.h, 1 + run);
            screen::takePpaPerf();
            uint32_t t0 = micros();
            screen::flush();
            srm.add(micros() - t0);
            screen::PpaPerf ppa = screen::takePpaPerf();
            blit.add((uint32_t)ppa.blitUs);
            sync.add((uint32_t)ppa.syncUs);
            screen::benchReadPanel(k.x, k.y, k.w, k.h, bySrm);
            for (size_t i = 0; i < n; i++) {
                srmWrong += bySrm[i] != canvas[(size_t)(k.y + i / k.w) * 1280 + k.x + i % k.w];
            }
            // Something else on the panel, then the same by strips.
            fillPattern(k.x, k.y, k.w, k.h, 101 + run);
            screen::flush();
            fillPattern(k.x, k.y, k.w, k.h, 1 + run);
            t0 = micros();
            bool ok = screen::pushStrips(k.x, k.y, k.w, k.h, &each, k.widest);
            uint32_t took = micros() - t0;
            if (!ok) {
                Serial.printf("[bench] strips: %s not pushed by strips - internal RAM %uKB free, the largest block "
                              "%uKB\n",
                              k.name, (unsigned)(each.freeBefore / 1024), (unsigned)(each.largestBefore / 1024));
                screen::flush();
                continue;
            }
            strips.add(took);
            pushed++;
            perf.strips += each.strips;
            perf.stripW = each.stripW;
            perf.allocUs += each.allocUs;
            perf.copyUs += each.copyUs;
            perf.waitUs += each.waitUs;
            perf.freeBefore = each.freeBefore;
            perf.largestBefore = each.largestBefore;
            perf.freeHeld = pushed == 1 ? each.freeHeld : std::min(perf.freeHeld, each.freeHeld);
            screen::benchReadPanel(k.x, k.y, k.w, k.h, byStrips);
            for (size_t i = 0; i < n; i++) {
                apart += byStrips[i] != bySrm[i];
            }
            screen::flush();  // the rect still marked, as a flush would leave the canvas
        }
        Serial.printf("[bench] strips: %s %dx%d  flush() %.2fms (least %.2f, most %.2f) - the PPA from the PSRAM "
                      "%.2f of it, the cache written back %.2f", k.name, k.w, k.h, srm.mean(), srm.min(), srm.max(),
                      blit.mean(), sync.mean());
        if (pushed == 0) {
            Serial.println("; strips none");
            continue;
        }
        int a = std::max(1, perf.strips);
        Serial.printf("; strips %.2fms (least %.2f, most %.2f), the cache not written back: %d a push, %d wide, "
                      "copying %uus and waiting %uus a strip, %uus taking them; internal RAM %uKB free, the largest "
                      "block %uKB, at the least %uKB left with the strips taken, %uKB the lowest since boot; the "
                      "panel %u of %u pixels apart from the PPA's own push, which was %u apart from the canvas\n",
                      strips.mean(), strips.min(), strips.max(), perf.strips / (int)pushed, perf.stripW,
                      (unsigned)(perf.copyUs / a), (unsigned)(perf.waitUs / a), (unsigned)(perf.allocUs / pushed),
                      (unsigned)(perf.freeBefore / 1024), (unsigned)(perf.largestBefore / 1024),
                      (unsigned)(perf.freeHeld / 1024),
                      (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024), (unsigned)apart,
                      (unsigned)(n * pushed), (unsigned)srmWrong);
    }
    heap_caps_free(bySrm);
    heap_caps_free(byStrips);
}

void benchPush() {
    Times t;
    for (int i = 0; i < 5; i++) {
        screen::markAllDirty();
        uint32_t t0 = micros();
        screen::flush();
        t.add(micros() - t0);
    }
    Serial.printf("[bench] the whole screen pushed  %.2fms (least %.2f, most %.2f)\n", t.mean(), t.min(), t.max());
}


// --- what an instruction costs ------------------------------------------------
// Cycles on the CPU's own counter for 256 of one instruction: each waiting on
// the one before (latency), and eight of them taking turns, none waiting
// (throughput) - the least of five runs, interrupts and all, with an empty
// run's cycles taken off.

alignas(16) uint8_t g_cycleBuf[4096];  // in the chip's own memory
uint32_t g_chase[64];                  // a pointer chased round itself

template <typename F>
uint32_t leastCycles(F f) {
    uint32_t least = UINT32_MAX;
    for (int i = 0; i < 5; i++) {
        uint32_t t0 = esp_cpu_get_cycle_count();
        f();
        least = std::min(least, esp_cpu_get_cycle_count() - t0);
    }
    return least;
}

#define REPEAT256(ins) ".rept 256\n" ins "\n.endr\n"
#define REPEAT32x8(a, b, c, d, e, f, g, h) ".rept 32\n" a "\n" b "\n" c "\n" d "\n" e "\n" f "\n" g "\n" h "\n.endr\n"

// What PIE does with known lanes: a byte swap by unzip and zip, as the
// shadows' darkening turns its pixels round, and a multiply's product shifted
// down by 8 - whether the whole product is kept before the shift.
void checkPie() {
    alignas(16) static uint16_t in[8] = {0x1234, 0x5678, 0x9ABC, 0xDEF0, 0xF800, 0x07E0, 0x001F, 0xFFFF};
    alignas(16) static uint16_t k[8] = {200, 200, 200, 200, 200, 200, 200, 200};
    alignas(16) static uint16_t swapped[8], product[8];
    {
        register uint16_t *pin asm("a0") = in;
        register uint16_t *pout asm("a1") = swapped;
        asm volatile("esp.vld.128.ip q0, a0, 0\nesp.zero.q q1\nesp.vunzip.8 q0, q1\nesp.vzip.8 q1, q0\n"
                     "esp.vst.128.ip q1, a1, 0\n"
                     : "+r"(pin), "+r"(pout)
                     :
                     : "memory");
    }
    {
        register uint16_t *pin asm("a0") = in;
        register uint16_t *pk asm("a1") = k;
        register uint16_t *pout asm("a2") = product;
        asm volatile("li t3, 8\nesp.movx.w.sar t3\nesp.vld.128.ip q0, a0, 0\nesp.vld.128.ip q1, a1, 0\n"
                     "esp.vmul.u16 q2, q0, q1\nesp.vst.128.ip q2, a2, 0\n"
                     : "+r"(pin), "+r"(pk), "+r"(pout)
                     :
                     : "t3", "memory");
    }
    String a, b, c;
    bool swapOk = true, mulOk = true;
    for (int i = 0; i < 8; i++) {
        char t[12];
        snprintf(t, sizeof t, "%04x ", in[i]);
        a += t;
        snprintf(t, sizeof t, "%04x ", swapped[i]);
        b += t;
        snprintf(t, sizeof t, "%04x ", product[i]);
        c += t;
        swapOk = swapOk && swapped[i] == (uint16_t)(in[i] >> 8 | in[i] << 8);
        mulOk = mulOk && product[i] == (uint16_t)(((uint32_t)in[i] * 200u) >> 8);
    }
    Serial.printf("[bench] PIE check: lanes %s\n", a.c_str());
    Serial.printf("[bench] PIE check: unzip then zip %s- %s\n", b.c_str(), swapOk ? "each pixel's bytes swapped" : "NOT A BYTE SWAP");
    Serial.printf("[bench] PIE check: vmul.u16 by 200, SAR 8 %s- %s\n", c.c_str(),
                  mulOk ? "the whole product kept, then shifted" : "NOT (a*b)>>8");
}

// The shadows' PIE darkening (scene3d_pie.S) against the same sums in C,
// pixel for pixel: random pixels, high byte first, random masks, every k a
// shadow can have, and runs starting and ending anywhere in a chunk.
extern "C" void scene3d_darken8(uint16_t *c, const uint16_t *m, int chunks, const uint16_t *constants);

void checkDarken() {
    constexpr int N = 256;
    alignas(16) static uint16_t pixels[N], expect[N], mask[N], lanes[4][8];
    uint32_t seed = 99, wrong = 0, tried = 0;
    auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    for (int k = 0; k <= 256; k += 3) {
        for (int i = 0; i < 8; i++) {
            lanes[0][i] = (uint16_t)k;
            lanes[1][i] = 0xF800;
            lanes[2][i] = 0x07E0;
            lanes[3][i] = 0x001F;
        }
        for (int i = 0; i < N; i++) {
            pixels[i] = (uint16_t)rnd();
            mask[i] = (rnd() & 3) ? 1 : 0;
            uint16_t c = (uint16_t)(pixels[i] >> 8 | pixels[i] << 8);
            c = (uint16_t)(((c >> 11) * k >> 8) << 11 | ((c >> 5 & 63) * k >> 8) << 5 | ((c & 31) * k >> 8));
            expect[i] = mask[i] ? (uint16_t)(c >> 8 | c << 8) : pixels[i];
        }
        scene3d_darken8(pixels, mask, N / 8, &lanes[0][0]);
        for (int i = 0; i < N; i++) {
            tried++;
            if (pixels[i] != expect[i]) {
                if (wrong < 4) {
                    Serial.printf("[bench] darken8: k %d pixel %d gave %04x, C %04x\n", k, i, pixels[i], expect[i]);
                }
                wrong++;
            }
        }
    }
    Serial.printf("[bench] darken8 against C: %u of %u pixels different\n", (unsigned)wrong, (unsigned)tried);
}

void benchCycles() {
    checkDarken();
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    Serial.printf("[bench] chip revision v%d.%d, %d cores, at %luMHz\n", chip.revision / 100, chip.revision % 100,
                  chip.cores, (unsigned long)(esp_clk_cpu_freq() / 1000000));
    checkPie();
    uint32_t empty = leastCycles([] { asm volatile("" ::: "memory"); });
    auto report = [&](const char *what, uint32_t cycles) {
        float each = (cycles > empty ? cycles - empty : 0) / 256.0f;
        Serial.printf("[bench] cycles %-44s %6.2f\n", what, each);
    };
    // Integer.
    report("add, each waiting", leastCycles([] { asm volatile(REPEAT256("add a5, a5, a4") ::: "a5"); }));
    report("mul, each waiting", leastCycles([] { asm volatile("li a4, 3\n" REPEAT256("mul a5, a5, a4") ::: "a4", "a5"); }));
    report("mul, none waiting",
           leastCycles([] {
               asm volatile("li a4, 3\n" REPEAT32x8("mul t0, a4, a4", "mul t1, a4, a4", "mul t2, a4, a4", "mul t3, a4, a4",
                                                    "mul t4, a4, a4", "mul t5, a4, a4", "mul t6, a4, a4", "mul a5, a4, a4")
                            ::: "a4", "a5", "t0", "t1", "t2", "t3", "t4", "t5", "t6");
           }));
    report("div, each waiting",
           leastCycles([] { asm volatile("li a5, 1000000\nli a4, 3\n" REPEAT256("div a5, a5, a4") ::: "a4", "a5"); }));
    report("divu, small numbers, each waiting",
           leastCycles([] { asm volatile("li a5, 7\nli a4, 3\n" REPEAT256("divu a5, a5, a4") ::: "a4", "a5"); }));
    // Memory.
    for (int i = 0; i < 64; i++) {
        g_chase[i] = (uint32_t)&g_chase[(i + 1) % 64];
    }
    report("lw, chased (internal RAM), each waiting",
           leastCycles([] {
               asm volatile("mv a5, %0\n" REPEAT256("lw a5, 0(a5)") : : "r"(g_chase) : "a5", "memory");
           }));
    report("lw, none waiting",
           leastCycles([] {
               asm volatile("mv a4, %0\n" REPEAT32x8("lw t0, 0(a4)", "lw t1, 4(a4)", "lw t2, 8(a4)", "lw t3, 12(a4)",
                                                     "lw t4, 16(a4)", "lw t5, 20(a4)", "lw t6, 24(a4)", "lw a5, 28(a4)")
                            : : "r"(g_cycleBuf) : "a4", "a5", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "memory");
           }));
    report("sw, one after another",
           leastCycles([] {
               asm volatile("mv a4, %0\n" REPEAT32x8("sw zero, 0(a4)", "sw zero, 4(a4)", "sw zero, 8(a4)", "sw zero, 12(a4)",
                                                     "sw zero, 16(a4)", "sw zero, 20(a4)", "sw zero, 24(a4)", "sw zero, 28(a4)")
                            : : "r"(g_cycleBuf) : "a4", "memory");
           }));
    // A chase through PSRAM, 4KB apart over 8MB: each a miss all the way out.
    uint32_t *far = (uint32_t *)heap_caps_malloc(8u << 20, MALLOC_CAP_SPIRAM);
    if (far != nullptr) {
        constexpr uint32_t STRIDE = 4096 / 4, COUNT = (8u << 20) / 4096;
        for (uint32_t i = 0; i < COUNT; i++) {
            uint32_t next = (i * 1031u + 17u) % COUNT;  // a step that visits every one, out of order
            far[i * STRIDE] = (uint32_t)&far[next * STRIDE];
        }
        report("lw, chased through PSRAM, missing the caches",
               leastCycles([far] { asm volatile("mv a5, %0\n" REPEAT256("lw a5, 0(a5)") : : "r"(far) : "a5", "memory"); }));
        // And round a small part of it, which the caches keep.
        for (int i = 0; i < 64; i++) {
            far[i * 16] = (uint32_t)&far[((i + 1) % 64) * 16];
        }
        report("lw, chased through PSRAM, in the cache",
               leastCycles([far] { asm volatile("mv a5, %0\n" REPEAT256("lw a5, 0(a5)") : : "r"(far) : "a5", "memory"); }));
        heap_caps_free(far);
    }
    // Floating point.
    report("fadd.s, each waiting", leastCycles([] { asm volatile(REPEAT256("fadd.s ft0, ft0, ft1") ::: "ft0"); }));
    report("fadd.s, none waiting",
           leastCycles([] {
               asm volatile(REPEAT32x8("fadd.s ft0, ft8, ft9", "fadd.s ft1, ft8, ft9", "fadd.s ft2, ft8, ft9",
                                       "fadd.s ft3, ft8, ft9", "fadd.s ft4, ft8, ft9", "fadd.s ft5, ft8, ft9",
                                       "fadd.s ft6, ft8, ft9", "fadd.s ft7, ft8, ft9")
                            ::: "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7");
           }));
    report("fmul.s, each waiting", leastCycles([] { asm volatile(REPEAT256("fmul.s ft0, ft0, ft1") ::: "ft0"); }));
    report("fmadd.s, each waiting", leastCycles([] { asm volatile(REPEAT256("fmadd.s ft0, ft0, ft1, ft2") ::: "ft0"); }));
    report("fmadd.s, none waiting",
           leastCycles([] {
               asm volatile(REPEAT32x8("fmadd.s ft0, ft8, ft9, ft10", "fmadd.s ft1, ft8, ft9, ft10",
                                       "fmadd.s ft2, ft8, ft9, ft10", "fmadd.s ft3, ft8, ft9, ft10",
                                       "fmadd.s ft4, ft8, ft9, ft10", "fmadd.s ft5, ft8, ft9, ft10",
                                       "fmadd.s ft6, ft8, ft9, ft10", "fmadd.s ft7, ft8, ft9, ft10")
                            ::: "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7");
           }));
    report("fdiv.s, each waiting",
           leastCycles([] {
               asm volatile("li a5, 0x3f800000\nfmv.w.x ft0, a5\nfmv.w.x ft1, a5\n" REPEAT256("fdiv.s ft0, ft0, ft1")
                            ::: "a5", "ft0", "ft1");
           }));
    report("fdiv.s, none waiting",
           leastCycles([] {
               asm volatile("li a5, 0x40400000\nfmv.w.x ft8, a5\nfmv.w.x ft9, a5\n"
                            REPEAT32x8("fdiv.s ft0, ft8, ft9", "fdiv.s ft1, ft8, ft9", "fdiv.s ft2, ft8, ft9",
                                       "fdiv.s ft3, ft8, ft9", "fdiv.s ft4, ft8, ft9", "fdiv.s ft5, ft8, ft9",
                                       "fdiv.s ft6, ft8, ft9", "fdiv.s ft7, ft8, ft9")
                            ::: "a5", "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7", "ft8", "ft9");
           }));
    report("fsqrt.s, each waiting",
           leastCycles([] {
               asm volatile("li a5, 0x40800000\nfmv.w.x ft0, a5\n" REPEAT256("fsqrt.s ft0, ft0") ::: "a5", "ft0");
           }));
    report("fcvt.w.s (float to int), none waiting",
           leastCycles([] {
               asm volatile(REPEAT32x8("fcvt.w.s t0, ft8, rup", "fcvt.w.s t1, ft8, rup", "fcvt.w.s t2, ft8, rup",
                                       "fcvt.w.s t3, ft8, rup", "fcvt.w.s t4, ft8, rup", "fcvt.w.s t5, ft8, rup",
                                       "fcvt.w.s t6, ft8, rup", "fcvt.w.s a5, ft8, rup")
                            ::: "a5", "t0", "t1", "t2", "t3", "t4", "t5", "t6");
           }));
    report("fcvt.s.w then fadd.s on it (int to float, used)",
           leastCycles([] {
               asm volatile(".rept 128\nfcvt.s.w ft0, a4\nfadd.s ft1, ft0, ft1\n.endr\n" ::: "ft0", "ft1");
           }));
    report("flt.s then a branch on it",
           leastCycles([] { asm volatile(".rept 128\nflt.s a5, ft8, ft9\nbnez a5, 1f\n1:\n.endr\n" ::: "a5"); }));
    // Branches: a loop's own, taken; and one that can't be guessed.
    report("a loop's turn, addi and bnez taken",
           leastCycles([] { asm volatile("li a5, 256\n1:\naddi a5, a5, -1\nbnez a5, 1b\n" ::: "a5"); }));
    report("a branch on a coin toss",
           leastCycles([] {
               asm volatile("li a5, 256\nli a4, 0x9e3779b9\n1:\nslli t0, a4, 13\nxor a4, a4, t0\nsrli t0, a4, 17\n"
                            "xor a4, a4, t0\nslli t0, a4, 5\nxor a4, a4, t0\nandi t0, a4, 1\nbeqz t0, 2f\nnop\n2:\n"
                            "addi a5, a5, -1\nbnez a5, 1b\n" ::: "a4", "a5", "t0");
           }));
    report("the same, the toss always the same",
           leastCycles([] {
               asm volatile("li a5, 256\nli a4, 0x9e3779b9\n1:\nslli t0, a4, 13\nxor a4, a4, t0\nsrli t0, a4, 17\n"
                            "xor a4, a4, t0\nslli t0, a4, 5\nxor a4, a4, t0\nli t0, 1\nbeqz t0, 2f\nnop\n2:\n"
                            "addi a5, a5, -1\nbnez a5, 1b\n" ::: "a4", "a5", "t0");
           }));
    // PIE: 128-bit registers, eight 16-bit lanes.
    report("esp.vadd.u16, each waiting",
           leastCycles([] { asm volatile("esp.zero.q q0\nesp.zero.q q1\n" REPEAT256("esp.vadd.u16 q0, q0, q1")); }));
    report("esp.vadd.u16, none waiting",
           leastCycles([] {
               asm volatile(REPEAT32x8("esp.vadd.u16 q0, q6, q7", "esp.vadd.u16 q1, q6, q7", "esp.vadd.u16 q2, q6, q7",
                                       "esp.vadd.u16 q3, q6, q7", "esp.vadd.u16 q4, q6, q7", "esp.vadd.u16 q5, q6, q7",
                                       "esp.vadd.u16 q0, q6, q7", "esp.vadd.u16 q1, q6, q7"));
           }));
    report("esp.vmul.u16, each waiting",
           leastCycles([] {
               asm volatile("li a5, 8\nesp.movx.w.sar a5\n" REPEAT256("esp.vmul.u16 q0, q0, q1") ::: "a5");
           }));
    report("esp.vmul.u16, none waiting",
           leastCycles([] {
               asm volatile(REPEAT32x8("esp.vmul.u16 q0, q6, q7", "esp.vmul.u16 q1, q6, q7", "esp.vmul.u16 q2, q6, q7",
                                       "esp.vmul.u16 q3, q6, q7", "esp.vmul.u16 q4, q6, q7", "esp.vmul.u16 q5, q6, q7",
                                       "esp.vmul.u16 q0, q6, q7", "esp.vmul.u16 q1, q6, q7"));
           }));
    report("esp.andq, each waiting", leastCycles([] { asm volatile(REPEAT256("esp.andq q0, q0, q1")); }));
    report("esp.vcmp.eq.u16, none waiting",
           leastCycles([] {
               asm volatile(REPEAT32x8("esp.vcmp.eq.u16 q0, q6, q7", "esp.vcmp.eq.u16 q1, q6, q7",
                                       "esp.vcmp.eq.u16 q2, q6, q7", "esp.vcmp.eq.u16 q3, q6, q7",
                                       "esp.vcmp.eq.u16 q4, q6, q7", "esp.vcmp.eq.u16 q5, q6, q7",
                                       "esp.vcmp.eq.u16 q0, q6, q7", "esp.vcmp.eq.u16 q1, q6, q7"));
           }));
    report("esp.vunzip.8 then esp.vzip.8 (a byte swap), each pair",
           leastCycles([] { asm volatile(".rept 128\nesp.vunzip.8 q0, q1\nesp.vzip.8 q1, q0\n.endr\n"); }) );
    report("esp.vld.128.ip, through internal RAM",
           leastCycles([] {
               asm volatile("mv a4, %0\n" ".rept 256\nesp.vld.128.ip q0, a4, 16\n.endr\n" : : "r"(g_cycleBuf) : "a4", "memory");
           }));
    report("esp.vst.128.ip, through internal RAM",
           leastCycles([] {
               asm volatile("mv a4, %0\n" ".rept 256\nesp.vst.128.ip q0, a4, 16\n.endr\n" : : "r"(g_cycleBuf) : "a4", "memory");
           }));
    report("esp.vld.128.ip then using it at once",
           leastCycles([] {
               asm volatile("mv a4, %0\n" ".rept 128\nesp.vld.128.ip q0, a4, 0\nesp.vadd.u16 q1, q0, q1\n.endr\n"
                            : : "r"(g_cycleBuf) : "a4", "memory");
           }));
    report("esp.vldbc.16.ip (a broadcast)",
           leastCycles([] {
               asm volatile("mv a4, %0\n" ".rept 256\nesp.vldbc.16.ip q0, a4, 0\n.endr\n" : : "r"(g_cycleBuf) : "a4", "memory");
           }));
    // A hardware loop against a counted one: an add each turn, 256 turns.
    report("a counted loop's turn (addi, bnez) with an add",
           leastCycles([] { asm volatile("li a5, 256\n1:\nadd a3, a3, a4\naddi a5, a5, -1\nbnez a5, 1b\n" ::: "a3", "a5"); }));
    report("a hardware loop's turn (esp.lp.setup) with an add",
           leastCycles([] {
               asm volatile("li a5, 256\nesp.lp.setup 0, a5, 1f\nadd a3, a3, a4\n1:\nnop\n" ::: "a3", "a5");
           }));
}
}  // namespace

void bench2dCommand(char c, void (*repaint)()) {
    Serial.printf("[bench] internal RAM %uKB free, PSRAM %uKB free\n",
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    switch (c) {
    case 'm': benchMap(); break;
    case 'n': benchNames(); break;
    case 'r': benchRadar(); break;
    case 'z': benchZoom(); break;
    case 't': benchTable(); break;
    case 'x': benchPush(); break;
    case 'p': adsbBenchParse(); break;
    case 's': soundBench(); break;
    case 'i': benchCycles(); break;
    case 'a': airportsBench(); break;
    case 'c': benchCodeMisses(); break;
    case 'X': benchStrips(); break;
    default: Serial.printf("[bench] no bench '%c'\n", c); break;
    }
    if (repaint != nullptr) {
        repaint();
    }
    Serial.println("[bench] done");
}
#endif
