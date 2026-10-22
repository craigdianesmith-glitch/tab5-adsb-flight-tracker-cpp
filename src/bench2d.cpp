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

#include <algorithm>
#include <vector>

#include "adsb_client.h"
#include "coast.h"
#include "display.h"
#include "names.h"
#include "radar_screen.h"
#include "screen.h"

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
    Serial.printf("[bench] the map's first draw, %d thinned levels built\n", coastLevels());
    for (const Place &p : PLACES) {
        for (float r : RANGES) {
            CoastPlot plot = {pixels, canvas.width(), 640, 362, 342, p.lat, p.lon, r, 0xFFFF};
            Times t;
            int strokes = 0;
            for (int i = 0; i < 3; i++) {
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
    default: Serial.printf("[bench] no bench '%c'\n", c); break;
    }
    if (repaint != nullptr) {
        repaint();
    }
    Serial.println("[bench] done");
}
#endif
