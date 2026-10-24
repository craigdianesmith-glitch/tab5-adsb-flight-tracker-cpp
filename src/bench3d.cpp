#ifdef BENCH3D

#include "bench3d.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <esp_cpu.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <algorithm>
#include <atomic>

#include "coast.h"
#include "labels3d.h"
#include "navaids.h"
#include "psram_alloc.h"
#include "runways.h"
#include "scene3d.h"
#include "screen.h"

using namespace scene3d;

namespace {

constexpr int W = 1280, H = 720, BAND_ROWS = 8;

// Where on the screen the 3D view is drawn, and how big: the whole of it, or
// a smaller picture with buttons and the aircraft's details round it.
int g_vx = 0, g_vy = 0, g_vw = W, g_vh = H;

// The view at `vw` by `vh`: the whole screen, or - smaller - in a border as
// the 3D screen has it (view3d.cpp): in from the top left by 16, where it
// leaves room before the radar's column of buttons, and the details under it.
void setView(int vw, int vh) {
    g_vw = vw;
    g_vh = vh;
    bool inset = vw + 32 <= 1280 - 16 - 150;
    g_vx = vw < W && inset ? 16 : 0;
    g_vy = vh < H && inset ? 16 : 0;
}

// Each core's band: colour, depth and shadow mask, in the chip's own memory.
void *g_scratch[2] = {nullptr, nullptr};
std::atomic<int> g_next{0};
void (*g_repaint)() = nullptr;
enum class Sink { NONE, CANVAS, PANEL, COMPARE };
std::atomic<int> g_mismatched{0};
// Where they differ: the box round them, and the first's two pixels.
int g_misBox[4] = {1 << 30, 1 << 30, -1, -1};
uint16_t g_misWas = 0, g_misNow = 0;
// And the first of them, each its place and its two pixels, for the log.
struct Mis {
    int16_t x, y;
    uint16_t was, now;
};
Mis g_mis[1024];
std::atomic<int> g_misCount{0};
Sink g_sink = Sink::NONE;
TaskHandle_t g_helper = nullptr;
SemaphoreHandle_t g_done = nullptr;
uint32_t g_busyUs[2];
// What each core was last doing, for a stall to be reported by: a step and
// the band it was on.
enum Step : uint8_t { IDLE, PREPARING, DRAWING, OVERLAY, PUSHING, COPYING, INFO, BORDER };
volatile uint8_t g_step[2] = {IDLE, IDLE};
volatile int16_t g_stepBand[2] = {-1, -1};
static const char *const STEP_NAMES[] = {"idle", "preparing", "drawing", "overlay", "pushing", "copying", "info", "border"};
bool g_stalled = false;
uint32_t g_stageUs[2][6];
bool g_profile = false;
bool g_pushFailed = false;

// What is written over the picture, a band at a time: the traffic's labels,
// stamped into the band from the bitmaps labels3d made of their words (see
// labels3d.h); and the captions, for which each core points a canvas of its
// own at the band it has just drawn and draws its share of the text there,
// the canvas clipping the rest.
struct Text {
    char line[48];
    int x, y, size;
    uint16_t color, back;  // the same: no box behind it
};
constexpr int TEXTS = 20;
Text g_text[TEXTS];
int g_textCount = 0;
M5Canvas g_bandCanvas[2];
labels3d::Labeller g_labeller;
labels3d::Placed g_placed[labels3d::MOST + 1];  // the traffic's, and the one followed's
int g_placedCount = 0;
uint16_t g_labelInk = 0xFFFF, g_labelDim = 0;  // byte-swapped, as the bands' pixels are
uint32_t g_overlayUs = 0;                      // the drawing core's time on it, for a run

void clearOverlay() { g_textCount = g_placedCount = 0; }

void addText(const char *line, int x, int y, int size, uint16_t color, uint16_t back) {
    if (g_textCount < TEXTS) {
        Text &tx = g_text[g_textCount++];
        snprintf(tx.line, sizeof tx.line, "%s", line);
        tx.x = x, tx.y = y, tx.size = size, tx.color = color, tx.back = back;
    }
}

void overlay(int who, uint16_t *band, int y, int rows) {
    labels3d::stamp(band, y, rows, g_vw, g_placed, g_placedCount, g_labelInk, g_labelDim);
    if (g_textCount == 0) {
        return;
    }
    M5Canvas &c = g_bandCanvas[who];
    c.setBuffer(band, g_vw, rows, 16);
    c.setFont(&fonts::Font0);
    for (int i = 0; i < g_textCount; i++) {
        const Text &t = g_text[i];
        int h = 8 * t.size;
        if (t.y + h + 4 <= y || t.y - 4 >= y + rows) {
            continue;
        }
        c.setTextSize(t.size);
        if (t.back != t.color) {
            c.fillRect(t.x - 6, t.y - 4 - y, c.textWidth(t.line) + 12, h + 8, t.back);
        }
        c.setTextColor(t.color);
        c.drawString(t.line, t.x, t.y - y);
    }
}

// Takes bands until there are none left, drawing each and sending it on.
void work(int who) {
    uint32_t t0 = micros();
    uint16_t *canvas = (uint16_t *)screen::canvas().getBuffer();
    for (int b; (b = g_next.fetch_add(1)) < bandCount();) {
        g_stepBand[who] = (int16_t)b;
        g_step[who] = DRAWING;
        uint16_t *px = drawBand(b, g_scratch[who], g_profile ? g_stageUs[who] : nullptr);
        int y = b * BAND_ROWS, rows = std::min(BAND_ROWS, g_vh - y);
        g_step[who] = OVERLAY;
        overlay(who, px, y, rows);
        g_step[who] = g_sink == Sink::PANEL ? PUSHING : COPYING;
        if (g_sink == Sink::CANVAS) {
            for (int r = 0; r < rows; r++) {
                memcpy(canvas + (size_t)(g_vy + y + r) * W + g_vx, px + (size_t)r * g_vw, (size_t)g_vw * 2);
            }
        } else if (g_sink == Sink::COMPARE) {
            // Against the frame left in the canvas: how many pixels differ.
            int n = 0;
            for (int r = 0; r < rows; r++) {
                const uint16_t *was = canvas + (size_t)(g_vy + y + r) * W + g_vx, *now = px + (size_t)r * g_vw;
                for (int i = 0; i < g_vw; i++) {
                    if (now[i] != was[i]) {
                        if (g_misBox[2] < 0) {
                            g_misWas = was[i];
                            g_misNow = now[i];
                        }
                        int k = g_misCount.fetch_add(1);
                        if (k < 1024) {
                            g_mis[k] = Mis{(int16_t)i, (int16_t)(y + r), was[i], now[i]};
                        }
                        g_misBox[0] = std::min(g_misBox[0], i);
                        g_misBox[1] = std::min(g_misBox[1], y + r);
                        g_misBox[2] = std::max(g_misBox[2], i);
                        g_misBox[3] = std::max(g_misBox[3], y + r);
                        n++;
                    }
                }
            }
            g_mismatched += n;
        } else if (g_sink == Sink::PANEL && !screen::pushBand(px, g_vx, g_vy + y, g_vw, rows, who)) {
            g_pushFailed = true;
        }
    }
    g_busyUs[who] = micros() - t0;
    g_step[who] = IDLE;
}

// Waits for the other core's share, two seconds at most: a stall is reported,
// with what each core was doing, and the run given up rather than the board
// hung.
bool waitForHelper() {
    if (xSemaphoreTake(g_done, pdMS_TO_TICKS(2000)) == pdTRUE) {
        return true;
    }
    Serial.printf("[3d] STALL: this core %s band %d, the other %s band %d\n", STEP_NAMES[g_step[0]], g_stepBand[0],
                  STEP_NAMES[g_step[1]], g_stepBand[1]);
    g_stalled = true;
    return false;
}

uint32_t g_pushWaitUs = 0;

// The pipeline's bands: two colour buffers and a depth buffer, each a band
// of g_pipeRows rows the view's width - taken for a run and given back
// after, each on its own, as the heap is by then too broken up for one block
// of all three.
void *g_pipe[3] = {nullptr, nullptr, nullptr};
int g_pipeRows = 8;

size_t g_pipeBytes = 0;  // each of the three's, while they are held

void releasePipe() {
    for (void *&p : g_pipe) {
        heap_caps_free(p);
        p = nullptr;
    }
    g_pipeBytes = 0;
}

size_t pipeColorBytes() { return ((size_t)g_vw * g_pipeRows * 2 + 63) & ~(size_t)63; }

// A whole frame drawn on this core, each band pushed while the next is
// drawn: the colour of one band in one buffer going to the screen while the
// next is drawn into the other. (A third, for the drawing to get a band
// ahead of the pushes, made no difference: the bands drawn faster than one
// is pushed - 0.3ms - are the sky's, all together at the top.)
void drawPipelined() {
    uint16_t *color[2] = {(uint16_t *)g_pipe[0], (uint16_t *)g_pipe[1]};
    uint16_t *depth = (uint16_t *)g_pipe[2];
    bool pushing = false;
    for (int b = 0; b < bandCount(); b++) {
        uint16_t *c = color[b & 1];
        int y = b * g_pipeRows, rows = std::min(g_pipeRows, g_vh - y);
        g_stepBand[1] = (int16_t)b;
        g_step[1] = DRAWING;
        drawBandInto(b, c, depth, g_profile ? g_stageUs[1] : nullptr);
        g_step[1] = OVERLAY;
        uint32_t o0 = micros();
        overlay(1, c, y, rows);
        g_overlayUs += micros() - o0;
        g_step[1] = PUSHING;
        uint32_t w0 = micros();
        if (pushing && !screen::pushBandWait()) {
            g_pushFailed = true;
            break;
        }
        g_pushWaitUs += micros() - w0;
        pushing = screen::pushBandStart(c, g_vx, g_vy + y, g_vw, rows);
        if (!pushing) {
            g_pushFailed = true;
            break;
        }
    }
    if (pushing && !screen::pushBandWait()) {
        g_pushFailed = true;
    }
    g_step[1] = IDLE;
}

// The other core's share of a frame: its bands, its half of the aircraft, or
// - pipelined - the whole of the drawing.
enum class Job { BANDS, AIRCRAFT, PIPELINE };
volatile Job g_job = Job::BANDS;

void helperTask(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (g_job == Job::AIRCRAFT) {
            g_step[1] = PREPARING;
            prepareAircraft(1, 2);
            g_step[1] = IDLE;
        } else if (g_job == Job::PIPELINE) {
            drawPipelined();
        } else {
            work(1);
        }
        xSemaphoreGive(g_done);
    }
}

uint32_t clockUs() { return (uint32_t)micros(); }

// The bands' scratch, taken for a command and given back after it: the chip's
// own memory is what WiFi and TLS live on, and with 140KB of it held the poll
// starved and the watchdog reset the board.
void release() {
    for (void *&s : g_scratch) {
        heap_caps_free(s);
        s = nullptr;
    }
    releasePipe();
}

// The bands' scratch in the chip's own memory, for both cores to draw.
bool ready(bool bothDraw = true) {
    setClock(clockUs);
    if (!bothDraw) {
        // The pipeline takes its own bands; it needs only the helper.
        if (g_helper == nullptr) {
            g_done = xSemaphoreCreateBinary();
            if (xTaskCreatePinnedToCore(helperTask, "bench3d", 6144, nullptr, 3, &g_helper, 0) != pdPASS) {
                g_helper = nullptr;
                return false;
            }
        }
        return true;
    }
    if (g_scratch[1] != nullptr && g_scratch[0] != nullptr) {
        return true;
    }
    size_t colorBytes = ((size_t)W * BAND_ROWS * 2 + 63) & ~(size_t)63, band = scratchBytes(W, BAND_ROWS);
    Serial.printf("[3d] internal RAM free %uKB, largest block %uKB; taking %uKB for the bands\n",
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(((bothDraw ? band : 0) + band + colorBytes) / 1024));
    if (g_scratch[1] == nullptr) {
        g_scratch[1] = heap_caps_aligned_alloc(64, band + colorBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (bothDraw && g_scratch[0] == nullptr) {
        g_scratch[0] = heap_caps_aligned_alloc(64, band, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (g_scratch[1] == nullptr || (bothDraw && g_scratch[0] == nullptr)) {
        Serial.println("[3d] no room for the bands in internal RAM");
        release();
        return false;
    }
    if (g_helper == nullptr) {
        g_done = xSemaphoreCreateBinary();
        if (xTaskCreatePinnedToCore(helperTask, "bench3d", 6144, nullptr, 3, &g_helper, 0) != pdPASS) {
            g_helper = nullptr;
            release();
            return false;
        }
    }
    return true;
}

// A frame prepared with the aircraft shared between the cores.
void prepareBoth(int w, int h, const Scene &scene, const Camera &cam) {
    prepareBegin(w, h, BAND_ROWS, true, scene, cam);
    g_job = Job::AIRCRAFT;
    xTaskNotifyGive(g_helper);
    g_step[0] = PREPARING;
    prepareAircraft(0, 2);
    g_step[0] = IDLE;
    if (!waitForHelper()) {
        return;
    }
    g_job = Job::BANDS;
    prepareEnd();
    flip();
}

// One frame drawn, on one core or both: how long, start to finish.
uint32_t draw(bool both, Sink sink) {
    g_sink = sink;
    g_next = 0;
    uint32_t t0 = micros();
    if (both) {
        xTaskNotifyGive(g_helper);
        work(0);
        waitForHelper();
    } else {
        g_busyUs[1] = 0;
        work(0);
    }
    return micros() - t0;
}

Aircraft g_parked[22];
int g_parkedCount = 0;

// A busy airport's traffic, every frame: in PSRAM, like everything else
// that grows with the scene.
constexpr int MAX_TRAFFIC = 400;
Aircraft *g_traffic = nullptr;
bool g_busy = false;
int g_standCount = 0;  // the busy traffic's first, parked
char g_runwayName[8] = "09";

// A real airport from the firmware's tables: its open runways in metres round
// the middle of their ends, its VOR/DMEs within two miles, and the runway
// ends landed on and departed from.
// The runway ends the landing and departures use, as setLanding() and
// setDeparture() were given them; and whether the one followed is departing.
int g_landRunway = 0, g_landEnd = 0, g_departRunway = -1, g_departEnd = 0;
bool g_departing = false;
// Where the airport's places are about - see useAirport() - and whether its
// land, and the cruise's, are drawn in what covers them: see scene3d's
// GroundCover.
double g_airportLat0 = 0, g_airportLon0 = 0;
float g_airportFieldM = 0;
bool g_cover = false;

// Where the one followed is at t: landing, or - lined up for twenty seconds
// first - departing.
Pose followedAt(float t) { return g_departing ? departurePose(t - 20) : landingPose(t); }

bool useAirport(const char *code, const char *land, const char *depart) {
    int count = 0;
    const Runway *runways = runwaysAt(code, count);
    if (runways == nullptr || count == 0) {
        return false;
    }
    double lat0 = 0, lon0 = 0;
    int open = 0;
    for (int i = 0; i < count; i++) {
        if (!runways[i].closed) {
            lat0 += (runways[i].end[0].lat + runways[i].end[1].lat) / 2.0;
            lon0 += (runways[i].end[0].lon + runways[i].end[1].lon) / 2.0;
            open++;
        }
    }
    lat0 /= open;
    lon0 /= open;
    g_airportLat0 = lat0;
    g_airportLon0 = lon0;
    g_airportFieldM = runways[0].elevationFt * 0.3048f;
    auto local = [&](double lat, double lon) {
        return Vec3{(float)((lon - lon0) * 111320.0 * cos(lat0 * M_PI / 180)), (float)((lat - lat0) * 110574.0), 0};
    };
    RunwaySpec specs[12];
    int n = 0, landRunway = 0, landEnd = 0, departRunway = -1, departEnd = 0;
    for (int i = 0; i < count && n < 12; i++) {
        if (runways[i].closed) {
            continue;
        }
        RunwaySpec &spec = specs[n];
        for (int e = 0; e < 2; e++) {
            spec.end[e] = local(runways[i].end[e].lat, runways[i].end[e].lon);
            snprintf(spec.name[e], sizeof spec.name[e], "%s", runways[i].end[e].ident);
            if (strcmp(land, runways[i].end[e].ident) == 0) {
                landRunway = n, landEnd = e;
            }
            if (strcmp(depart, runways[i].end[e].ident) == 0) {
                departRunway = n, departEnd = e;
            }
        }
        spec.width = runways[i].widthFt * 0.3048f;
        n++;
    }
    Vec3 beacons[4];
    int beaconCount = 0;
    for (const Navaid *nav : navaidsWithin(lat0, lon0, 2)) {
        if (beaconCount < 4 && nav->type != NavaidType::TACAN && nav->type != NavaidType::DME &&
            nav->type != NavaidType::NDB_DME) {
            beacons[beaconCount++] = local(nav->lat, nav->lon);
        }
    }
    setAirfield(specs, n, beacons, beaconCount, nullptr, true);
    setLanding(landRunway, landEnd);
    setDeparture(departRunway, departEnd);
    g_landRunway = landRunway, g_landEnd = landEnd, g_departRunway = departRunway, g_departEnd = departEnd;
    snprintf(g_runwayName, sizeof g_runwayName, "%s", land);
    if (g_traffic == nullptr) {
        g_traffic = (Aircraft *)heap_caps_malloc(sizeof(Aircraft) * MAX_TRAFFIC, MALLOC_CAP_SPIRAM);
    }
    g_busy = g_traffic != nullptr;
    g_standCount = g_busy ? parkedAtStands(g_traffic, MAX_TRAFFIC) : 0;
    Serial.printf("[3d] %s: %d runways, %d beacons, landing %s, departing %s\n", code, n, beaconCount, land, depart);
    return g_busy;
}

Scene sceneAt(float t, TimeOfDay time) {
    if (g_parkedCount == 0) {
        g_parkedCount = parkedAtStands(g_parked, 22);
    }
    Scene s;
    s.followed = {followedAt(t), lookFor(g_departing ? "B789" : "A320", 0x1B2A55), {}};
    {
        // Set up as what it is doing calls for: gear, flaps, spoilers.
        Pose a = followedAt(t - 0.1f), b = followedAt(t + 0.1f);
        float knots = sqrtf((b.at.x - a.at.x) * (b.at.x - a.at.x) + (b.at.y - a.at.y) * (b.at.y - a.at.y)) / 0.2f * 1.944f;
        bool ground = s.followed.pose.at.z <= 0.01f && (g_departing || t >= landingTouchdown() - 0.05f);
        s.followed.config = configFor(s.followed.pose.at.z, knots, ground, g_departing);
        if (!g_departing) {
            s.followed.config.spoilerDeploy *= spoilersRising(t - landingTouchdown());
            s.followed.config.spoilers = s.followed.config.spoilerDeploy > 0.01f;
        }
    }
    s.traffic = g_parked;
    s.trafficCount = g_parkedCount;
    if (g_busy) {
        // The made-up departures fly the runway the one followed leaves by:
        // none of them while it does.
        if (g_departing) {
            setDeparture(-1, 0);
        }
        s.traffic = g_traffic;
        s.trafficCount = busyTraffic(t, g_traffic, MAX_TRAFFIC);
        setDeparture(g_departRunway, g_departEnd);
    }
    s.seconds = t;
    s.sun = sunFor(time);
    if (g_cover) {
        Vec3 at = s.followed.pose.at;
        double lat = g_airportLat0 + at.y / 110574.0,
               lon = g_airportLon0 + at.x / (111320.0 * cos(g_airportLat0 * M_PI / 180));
        if (groundpack::want(lat, lon, 45)) {
            s.cover = flatCover(groundpack::window(), g_airportLat0, g_airportLon0);
        }
    }
    if (!g_departing && t >= landingTouchdown()) {
        s.smokeAge = t - landingTouchdown();
        s.touchdown = landingPose(landingTouchdown());
    }
    return s;
}

enum class View { CINEMA, CHASE, APRON };

Camera cameraAt(View view, float t, const Pose &pose, const char **shot) {
    switch (view) {
    case View::CHASE: return orbitCamera(pose, 20, 12, 45, 50);
    case View::APRON: return orbitCamera(pose, 15, 10, 90, 55);
    default: return cinemaCamera(t, shot);
    }
}

// The traffic's labels - as many, and saying as much, as the view has room
// for - for overlay() to stamp into the frame about to be drawn.
float g_labelledAt = -1;

void addLabels(const labels3d::Candidate *candidates, int count, const labels3d::Rect *keepOut, int keepOutCount,
               float t) {
    float dt = t - g_labelledAt;
    if (g_labelledAt < 0 || dt < 0 || dt > 1) {
        g_labeller.reset();  // a new run, or a jump in it
        dt = 0;
    }
    g_labelledAt = t;
    g_labelDim = __builtin_bswap16(M5.Display.color565(0x3D, 0xE8, 0x7C));  // the radar's blip green
    g_placedCount = g_labeller.place(candidates, count, keepOut, keepOutCount, g_vw, g_vh, dt, g_placed,
                                     labels3d::MOST);
}

// What labels must keep off: the aircraft followed, as far as it reaches on
// the screen. Returns how many places it added.
int keepOffFollowed(labels3d::Rect *out) {
    float x, y, reach;
    if (!followedOnScreen(x, y, reach)) {
        return 0;
    }
    *out = {(int)(x - reach), (int)(y - reach * 0.45f), (int)(x + reach), (int)(y + reach * 0.45f)};
    return 1;
}

// The broadcast's captions: the shot, and the lower third.
void captions(float t, const char *shot) {
    clearOverlay();
    Pose a = followedAt(t - 0.1f), b = followedAt(t + 0.1f), pose = followedAt(t);
    float knots = sqrtf((b.at.x - a.at.x) * (b.at.x - a.at.x) + (b.at.y - a.at.y) * (b.at.y - a.at.y)) / 0.2f * 1.944f;
    float fpm = (b.at.z - a.at.z) / 0.2f * 196.85f;
    char line[48];
    float touch = landingTouchdown();
    if (t >= touch && t < touch + 6) {
        Pose c = landingPose(touch - 0.25f), d = landingPose(touch - 0.05f);
        snprintf(line, sizeof line, "TOUCHDOWN  %.0f fpm", -(d.at.z - c.at.z) / 0.2f * 196.85f);
    } else if (pose.at.z > 0) {
        snprintf(line, sizeof line, "%3.0f kt  %4.0f ft  %+5.0f fpm", knots, pose.at.z * 3.28f, fpm);
    } else {
        snprintf(line, sizeof line, "%3.0f kt  on the ground", knots);
    }
    uint16_t ink = M5.Display.color565(255, 255, 255), grey = M5.Display.color565(190, 196, 204),
             back = M5.Display.color565(16, 20, 26);
    char who[40];
    snprintf(who, sizeof who, "BAW27K  A320  RWY %s", g_runwayName);
    labels3d::Rect keep[4];
    int keepCount = 0;
    if (g_vw == W) {  // with no border to put them in
        addText(who, 56, H - 70, 2, ink, back);
        addText(line, 56, H - 44, 2, grey, back);
        keep[keepCount++] = {50, H - 74, 56 + 12 * (int)std::max(strlen(who), strlen(line)) + 6, H - 20};
        if (shot != nullptr) {
            addText(shot, 40, 30, 2, ink, ink);
            keep[keepCount++] = {34, 26, 40 + 12 * (int)strlen(shot) + 6, 50};
        }
    }
    // The one followed: its callsign by it, as the traffic's are, and where
    // theirs must keep off.
    static labels3d::Placed followed;
    static uint8_t *followedBits = (uint8_t *)heap_caps_malloc(labels3d::BITS_BYTES, MALLOC_CAP_SPIRAM);
    float x, y;
    bool named = followedBits != nullptr && projectPoint({pose.at.x, pose.at.y, pose.at.z + 14}, x, y) && x > 0 &&
                 x < g_vw - 100 && y > 40 && y < g_vh - 40;
    if (named) {
        followed = {};
        snprintf(followed.line1, sizeof followed.line1, "BAW27K");
        int w = (int)strlen(followed.line1) * labels3d::CHAR_W + 2 * labels3d::PAD;
        followed.box = {(int)x + 10, (int)y - 10 - labels3d::LINE - 2 * labels3d::PAD, (int)x + 10 + w, (int)y - 10};
        followed.fromX = followed.box.x0, followed.fromY = followed.box.y1 - 1;
        followed.toX = (int)x, followed.toY = (int)y;
        labels3d::write(followed, followedBits);
        keep[keepCount++] = followed.box;
    }
    keepCount += keepOffFollowed(&keep[keepCount]);

    // The traffic's: made-up callsigns, and the parked told apart from the
    // rest by being on the stands.
    static const char *const AIRLINES[] = {"BAW", "EZY", "RYR", "DLH", "AFR", "KLM",
                                           "UAE", "QTR", "AAL", "UAL", "EIN", "VIR"};
    const Aircraft *traffic = g_busy ? g_traffic : g_parked;
    int parked = g_busy ? g_standCount : g_parkedCount;
    static Label anchors[48];
    static labels3d::Candidate candidates[48];
    static char calls[MAX_TRAFFIC][10], heights[48][16];  // the callsigns written once; heights as they change
    static bool written = false;
    for (int k = 0; k < MAX_TRAFFIC && !written; k++) {
        snprintf(calls[k], sizeof calls[k], "%s%d", AIRLINES[k % 12], 100 + k * 37 % 900);
    }
    written = true;
    int n = labelAnchors(anchors, 48);
    for (int i = 0; i < n; i++) {
        int k = anchors[i].index;
        const Aircraft &craft = traffic[k];
        bool flying = craft.pose.at.z > 1;
        if (flying) {
            snprintf(heights[i], sizeof heights[i], "%dft", (int)lroundf(craft.pose.at.z * 3.28f / 10) * 10);
        }
        labels3d::Kind kind = k < parked ? labels3d::Kind::PARKED : flying ? labels3d::Kind::AIRBORNE
                                                                           : labels3d::Kind::MOVING;
        candidates[i] = {anchors[i].x, anchors[i].y, anchors[i].sizePx, (uint32_t)k + 1, kind, calls[k],
                         flying ? heights[i] : "GND", typeOf(craft.look.family)};
    }
    addLabels(candidates, n, keep, keepCount, t);
    if (named) {
        g_placed[g_placedCount++] = followed;
    }
}

// The labels' stamping timed every way, over a frame's worth of 16-row bands
// of a 960x544 view: ten labels, as a busy view has them, and four, as a
// quiet one does - in C and with PIE, the same pixels either way or it says
// so - and, to compare, as the bench wrote them before: each band's share of
// the words drawn by M5GFX over a box darkened a pixel at a time.
void labelsBench() {
    constexpr int VW = 960, VH = 544, ROWS = 16, BANDS = VH / ROWS, REPEATS = 20;
    // Never the low-power core's memory, which the heap has too: see view3d.
    uint16_t *band = (uint16_t *)heap_caps_aligned_alloc(16, VW * ROWS * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    uint16_t *copy = (uint16_t *)heap_caps_malloc(VW * ROWS * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (band == nullptr || copy == nullptr) {
        Serial.println("[3d] no room for a band in internal RAM");
        heap_caps_free(band);
        heap_caps_free(copy);
        return;
    }
    static labels3d::Labeller labeller;
    static labels3d::Candidate candidates[10];
    static char calls[10][10];
    static labels3d::Placed placed[labels3d::MOST];
    for (int i = 0; i < 10; i++) {
        snprintf(calls[i], sizeof calls[i], "BAW%d", 100 + i * 37);
        candidates[i] = {(float)(70 + (i % 5) * 185), (float)(120 + (i / 5) * 230), 40, (uint32_t)i + 1,
                         labels3d::Kind::AIRBORNE, calls[i], "FL350^", "A320"};
    }
    uint16_t ink = 0xFFFF, dim = __builtin_bswap16(M5.Display.color565(0x3D, 0xE8, 0x7C));
    uint32_t seed = 12345;
    auto scribble = [&]() {  // something to darken: a different picture each band
        for (int i = 0; i < VW * ROWS; i++) {
            seed = seed * 1664525u + 1013904223u;
            band[i] = (uint16_t)(seed >> 16);
        }
    };
    // As the bench did it before.
    auto before = [&](int n, int y) {
        for (int i = 0; i < n; i++) {
            const labels3d::Rect &b = placed[i].box;
            int r0 = std::max(b.y0, y), r1 = std::min(b.y1, y + ROWS);
            for (int r = r0; r < r1; r++) {
                uint16_t *px = band + (size_t)(r - y) * VW;
                for (int x = std::max(0, b.x0); x < std::min(VW, b.x1); x++) {
                    px[x] = __builtin_bswap16((uint16_t)(__builtin_bswap16(px[x]) >> 2 & 0x39E7));
                }
            }
        }
        M5Canvas &c = g_bandCanvas[0];
        c.setBuffer(band, VW, ROWS, 16);
        for (int i = 0; i < n; i++) {
            const labels3d::Placed &p = placed[i];
            const labels3d::Candidate &k = candidates[p.candidate];
            if (std::max(p.fromY, (int)k.y) >= y && std::min(p.fromY, (int)k.y) < y + ROWS) {
                c.drawLine(p.fromX, p.fromY - y, (int)k.x, (int)k.y - y, M5.Display.color565(200, 214, 206));
            }
        }
        c.setFont(&fonts::Font0);
        for (int i = 0; i < n; i++) {
            const labels3d::Placed &p = placed[i];
            for (int l = 0; l < 2; l++) {
                const char *line = l == 0 ? p.line1 : p.line2;
                int ty = p.box.y0 + 5 + 18 * l;
                if (line[0] == 0 || ty + 20 <= y || ty - 4 >= y + ROWS) {
                    continue;
                }
                c.setTextSize(2);
                c.setTextColor(l == 0 ? M5.Display.color565(255, 255, 255) : M5.Display.color565(200, 214, 206));
                c.drawString(line, p.box.x0 + 5, ty - y);
            }
        }
    };
    for (int count : {10, 4}) {
        labeller.reset();
        int n = labeller.place(candidates, count, nullptr, 0, VW, VH, 0, placed, labels3d::MOST);
        // The same pixels, C and PIE?
        long differ = 0;
        for (int b = 0; b < BANDS; b++) {
            scribble();
            memcpy(copy, band, VW * ROWS * 2);
            labels3d::setSimd(false);
            labels3d::stamp(band, b * ROWS, ROWS, VW, placed, n, ink, dim);
            std::swap(band, copy);
            labels3d::setSimd(true);
            labels3d::stamp(band, b * ROWS, ROWS, VW, placed, n, ink, dim);
            for (int i = 0; i < VW * ROWS; i++) {
                differ += band[i] != copy[i];
            }
        }
        // How long a frame's worth takes each way.
        uint32_t us[3];
        for (int way = 0; way < 3; way++) {
            labels3d::setSimd(way == 1);
            uint32_t t0 = micros();
            for (int r = 0; r < REPEATS; r++) {
                for (int b = 0; b < BANDS; b++) {
                    if (way == 2) {
                        before(n, b * ROWS);
                    } else {
                        labels3d::stamp(band, b * ROWS, ROWS, VW, placed, n, ink, dim);
                    }
                }
            }
            us[way] = (micros() - t0) / REPEATS;
        }
        labels3d::setSimd(true);
        Serial.printf("[3d] %d labels (%s): stamped in C %.3f ms a frame, with PIE %.3f ms (%s); as the bench "
                      "wrote them before - M5GFX's text on a box darkened a pixel at a time - %.3f ms\n",
                      n, n > 4 ? "one line" : "two lines", us[0] / 1000.0f, us[1] / 1000.0f,
                      differ == 0 ? "the same pixels" : "DIFFERENT PIXELS", us[2] / 1000.0f);
        if (differ != 0) {
            Serial.printf("[3d]   %ld pixels differ between C and PIE\n", differ);
        }
    }
    heap_caps_free(band);
    heap_caps_free(copy);
}

void bench() {
    if (!ready()) {
        return;
    }
    struct Case {
        const char *name;
        float t;
        View view;
        TimeOfDay time;
    };
    const Case CASES[] = {
        {"long lens day", 8, View::CINEMA, TimeOfDay::DAY},
        {"chase day", 22, View::CINEMA, TimeOfDay::DAY},
        {"touchdown day", 26.6f, View::CINEMA, TimeOfDay::DAY},
        {"touchdown dusk", 26.6f, View::CINEMA, TimeOfDay::DUSK},
        {"touchdown night", 26.6f, View::CINEMA, TimeOfDay::NIGHT},
        {"rollout day", 35, View::CINEMA, TimeOfDay::DAY},
        {"tower night", 50, View::CINEMA, TimeOfDay::NIGHT},
        {"close chase day", 20, View::CHASE, TimeOfDay::DAY},
        {"apron 22 parked day", 66, View::APRON, TimeOfDay::DAY},
        {"apron 22 parked dusk", 66, View::APRON, TimeOfDay::DUSK},
    };
    Serial.println("[3d] case: prepare 1 core / prepare 2 cores (begin, aircraft A, B, end) / draw 1 core C, PIE / "
                   "draw 2 cores C, PIE (pixels PIE drew differently) / +copy to canvas / canvas flush / bands "
                   "straight to the panel - ms; what was drawn");
    for (const Case &c : CASES) {
        Scene scene = sceneAt(c.t, c.time);
        Camera cam = cameraAt(c.view, c.t, scene.followed.pose, nullptr);
        clearOverlay();
        uint32_t prep = UINT32_MAX, prep2 = UINT32_MAX, one = UINT32_MAX, two = UINT32_MAX, canvas = UINT32_MAX,
                 flush = UINT32_MAX, panel = UINT32_MAX, onePie = UINT32_MAX, twoPie = UINT32_MAX;
        Counts steps{};
        for (int run = 0; run < 3; run++) {
            uint32_t t0 = micros();
            prepare(W, H, BAND_ROWS, true, scene, cam);
            prep = std::min(prep, micros() - t0);
            t0 = micros();
            prepareBoth(W, H, scene, cam);
            uint32_t d2 = micros() - t0;
            if (d2 < prep2) {
                prep2 = d2;
                steps = counts();
            }
            setSimd(false);
            one = std::min(one, draw(false, Sink::NONE));
            two = std::min(two, draw(true, Sink::NONE));
            setSimd(true);
            onePie = std::min(onePie, draw(false, Sink::NONE));
            twoPie = std::min(twoPie, draw(true, Sink::NONE));
            canvas = std::min(canvas, draw(true, Sink::CANVAS));
            screen::markAllDirty();
            t0 = micros();
            screen::flush();
            flush = std::min(flush, micros() - t0);
            panel = std::min(panel, draw(true, Sink::PANEL));
            vTaskDelay(1);
        }
        // Where a frame's drawing goes, on one core, in C and with PIE.
        for (bool simd : {false, true}) {
            setSimd(simd);
            memset(g_stageUs, 0, sizeof g_stageUs);
            g_profile = true;
            draw(false, Sink::NONE);
            g_profile = false;
            Serial.printf("[3d]   drawing on one core, %s: sky %.1f, ground %.1f, shadows %.1f, depth clear %.1f, "
                          "solids %.1f, lines and lights %.1f ms\n",
                          simd ? "PIE" : "C  ", g_stageUs[0][0] / 1000.0f, g_stageUs[0][1] / 1000.0f,
                          g_stageUs[0][2] / 1000.0f, g_stageUs[0][3] / 1000.0f, g_stageUs[0][4] / 1000.0f,
                          g_stageUs[0][5] / 1000.0f);
        }
        // The same frame in C, then with PIE against it.
        setSimd(false);
        draw(true, Sink::CANVAS);
        setSimd(true);
        g_mismatched = 0;
        draw(true, Sink::COMPARE);
        Counts n = counts();
        Serial.printf("[3d] %-22s %5.1f / %5.1f (%4.1f %4.1f %4.1f %4.1f) / %5.1f %5.1f / %5.1f %5.1f (%d) / %5.1f / "
                      "%5.1f / %5.1f ms; %d tris %d lines %d lights, aircraft %d full %d simple %d dots%s\n",
                      c.name, prep / 1000.0f, prep2 / 1000.0f, steps.beginUs / 1000.0f, steps.aircraftUs[0] / 1000.0f,
                      steps.aircraftUs[1] / 1000.0f, steps.endUs / 1000.0f, one / 1000.0f, onePie / 1000.0f,
                      two / 1000.0f, twoPie / 1000.0f, (int)g_mismatched, canvas / 1000.0f, flush / 1000.0f,
                      panel / 1000.0f, n.triangles, n.lines, n.lights, n.aircraftFull, n.aircraftSimple,
                      n.aircraftDots, g_pushFailed ? " PUSH FAILED" : "");
    }
    release();
    g_repaint();
}

// The landing in real time through the cinema's shots, bands straight to the
// panel, with a governor: frames that run over budget turn the detail down -
// more of the other aircraft simple - and quick ones turn it back up. The
// budget allows for the push, some 35ms of any frame, which no amount of
// detail shed makes shorter.
// The border round a smaller view, as the 3D screen has it: the radar's
// column of buttons, its colours and sizes, down the right - CINEMA lit, the
// bench playing the cinema - and a strip of the aircraft's details under the
// view.
uint16_t g_borderBg, g_buttonBg, g_buttonLit, g_buttonInk, g_infoInk, g_infoFaint;

void drawBorder() {
    g_borderBg = M5.Display.color565(0x04, 0x12, 0x0A);
    g_buttonBg = M5.Display.color565(0x14, 0x3A, 0x28);
    g_buttonLit = M5.Display.color565(0x7C, 0xD9, 0xA0);
    g_buttonInk = M5.Display.color565(0xCF, 0xFF, 0xE2);
    g_infoInk = g_buttonLit;
    g_infoFaint = M5.Display.color565(0x5A, 0x9E, 0x78);
    auto &c = screen::canvas();
    screen::clear(g_borderBg);
    c.setFont(&fonts::Font0);
    c.setTextSize(2);
    c.setTextDatum(MC_DATUM);
    static const char *const BUTTONS[] = {"Back", "CINEMA", "CHASE", "ORBIT", "TOWER", "ZOOM IN", "ZOOM OUT"};
    static const int TOPS[] = {8, 84, 144, 204, 264, 340, 400};
    constexpr int BW = 150, BH = 50, BX = W - 16 - BW;
    for (int i = 0; i < 7; i++) {
        c.fillRoundRect(BX, TOPS[i], BW, BH, 6, i == 1 ? g_buttonLit : g_buttonBg);
        c.setTextColor(i == 1 ? g_borderBg : g_buttonInk);
        c.drawString(BUTTONS[i], BX + BW / 2, TOPS[i] + BH / 2);
    }
    c.setTextDatum(TL_DATUM);
    screen::markAllDirty();
    screen::flush();
}

// The details strip, every half second or so - pushed through the canvas,
// a few milliseconds, between frames.
void drawInfo(float t, const char *shot, float fps) {
    if (g_vh >= H) {
        return;
    }
    auto &c = screen::canvas();
    int top = g_vy + g_vh + (g_vy > 0 ? 16 : 8), x = g_vx > 0 ? g_vx : 16;
    c.fillRect(g_vx, g_vy + g_vh, g_vw, H - g_vy - g_vh, g_borderBg);
    c.setFont(&fonts::Font0);
    Pose a = followedAt(t - 0.1f), b = followedAt(t + 0.1f), pose = followedAt(t);
    float knots = sqrtf((b.at.x - a.at.x) * (b.at.x - a.at.x) + (b.at.y - a.at.y) * (b.at.y - a.at.y)) / 0.2f * 1.944f;
    float fpm = (b.at.z - a.at.z) / 0.2f * 196.85f;
    char line[96];
    c.setTextSize(3);
    c.setTextColor(g_buttonInk);
    char who[40];
    snprintf(who, sizeof who, "BAW27K   A320   RWY %s", g_runwayName);
    c.drawString(who, x, top + 4);
    c.setTextSize(2);
    c.setTextColor(g_infoInk);
    snprintf(line, sizeof line, "%.0f ft   %.0f kt   %+.0f fpm", pose.at.z * 3.28f, knots, fpm);
    c.drawString(line, x, top + 44);
    c.setTextColor(g_infoFaint);
    snprintf(line, sizeof line, "CINEMA - %s     %.1f fps  %dx%d", shot != nullptr ? shot : "", fps, g_vw, g_vh);
    c.drawString(line, x, top + 72);
    screen::markDirty(g_vx, g_vy + g_vh, g_vw, H - g_vy - g_vh);
    screen::flush();
}


// --- high up ------------------------------------------------------------------
// The view from cruise, as tools/landing3d_preview flies it: an easyJet A320
// at FL350 over the Firth of Clyde, turning from 205 onto 235, with traffic
// about at other heights - over the firmware's own coasts, borders and
// runways, the earth curving away to its horizon.

// Metres east, north and up of a point on the ground, through WGS84's
// earth-centred frame - the earth's curve and all.
struct Enu {
    double x0, y0, z0, sp, cp, sl, cl;
    static void ecef(double lat, double lon, double h, double &x, double &y, double &z) {
        constexpr double A = 6378137.0, E2 = 6.69437999014e-3;
        double p = lat * M_PI / 180, l = lon * M_PI / 180, n = A / sqrt(1 - E2 * sin(p) * sin(p));
        x = (n + h) * cos(p) * cos(l);
        y = (n + h) * cos(p) * sin(l);
        z = (n * (1 - E2) + h) * sin(p);
    }
    void at(double lat, double lon) {
        ecef(lat, lon, 0, x0, y0, z0);
        sp = sin(lat * M_PI / 180), cp = cos(lat * M_PI / 180), sl = sin(lon * M_PI / 180), cl = cos(lon * M_PI / 180);
    }
    Vec3 of(double lat, double lon, double h) const {
        double x, y, z;
        ecef(lat, lon, h, x, y, z);
        double dx = x - x0, dy = y - y0, dz = z - z0;
        return {(float)(-sl * dx + cl * dy), (float)(-sp * cl * dx - sp * sl * dy + cp * dz),
                (float)(cp * cl * dx + cp * sl * dy + sp * dz)};
    }
};

constexpr double CRUISE_LAT = 55.58, CRUISE_LON = -5.25;
Enu g_enu;
bool g_enroute = false;

struct Other {
    const char *type, *callsign;
    float east, north, feet, course, knots, fpm;
    bool military;
};
const Other OTHERS[] = {
    {"A388", "UAE18", -9000, 14000, 38000, 30, 485, 0, false},
    {"B738", "RYR4LH", 7000, -2000, 21000, 160, 330, 2200, false},
    {"AT76", "LOG6KG", -14000, -9000, 13000, 225, 260, 0, false},
    {"A320", "BAW1497", -5000, -22000, 33000, 205, 440, 0, false},
    {"C17", "RCH457", 26000, -14000, 30000, 95, 420, 0, true},
    {"EUFI", "TARTN1", -11000, -4000, 20000, 175, 480, -1500, true},
    {"A343", "DLH4KN", 16000, -26000, 36000, 300, 470, 0, false},
    {"B789", "VIR35", 30000, 12000, 39000, 285, 490, 0, false},
};
constexpr int OTHER_COUNT = sizeof(OTHERS) / sizeof(OTHERS[0]);

// High up: the others' labels - callsign, flight level with an arrow for
// climbing or descending, and type.
void enrouteCaptions(float t) {
    clearOverlay();
    Label anchors[OTHER_COUNT];
    int n = labelAnchors(anchors, OTHER_COUNT);
    static labels3d::Candidate candidates[OTHER_COUNT];
    static char heights[OTHER_COUNT][16];
    for (int i = 0; i < n; i++) {
        const Other &o = OTHERS[anchors[i].index];
        snprintf(heights[i], sizeof heights[i], "FL%03d%s", (int)lroundf((o.feet + o.fpm * t / 60) / 100),
                 o.fpm > 300 ? "^" : o.fpm < -300 ? "v" : "");
        candidates[i] = {anchors[i].x, anchors[i].y, anchors[i].sizePx, (uint32_t)anchors[i].index + 1,
                         labels3d::Kind::AIRBORNE, o.callsign, heights[i], o.type};
    }
    labels3d::Rect keep[1];
    addLabels(candidates, n, keep, keepOffFollowed(keep), t);
}
Pilot g_pilot;
float g_pilotAt = 0;

void sinkLine(void *context, OutlineKind kind, const double *latLon, int n) {
    auto *out = (std::vector<GroundLine, PsramAllocator<GroundLine>> *)context;
    Vec3 a = g_enu.of(latLon[0], latLon[1], 0);
    for (int i = 0; i + 1 < n; i++) {
        Vec3 b = g_enu.of(latLon[2 * i + 2], latLon[2 * i + 3], 0);
        out->push_back({a, b, (GroundKind)kind});
        a = b;
    }
}

// The country under the cruise, from the coast table and the runway table -
// timed, being what a 3D view high up would do each time the aircraft has
// gone far enough to want the next stretch of it.
bool enrouteSetup() {
    uint32_t t0 = micros();
    g_enu.at(CRUISE_LAT, CRUISE_LON);
    std::vector<GroundLine, PsramAllocator<GroundLine>> lines;
    coastLines(CRUISE_LAT, CRUISE_LON, 520, 520, 0, sinkLine, &lines);  // in full, as the live view
    setGroundLines(lines.data(), (int)lines.size());
    int count = 0;
    const Runway *all = allRunways(count);
    std::vector<RunwaySpec, PsramAllocator<RunwaySpec>> runways;
    for (int i = 0; i < count; i++) {
        const Runway &r = all[i];
        double dn = (r.end[0].lat - CRUISE_LAT) * 111.0, de = (r.end[0].lon - CRUISE_LON) * 111.0 * cos(CRUISE_LAT * M_PI / 180);
        if (!r.closed && dn * dn + de * de < 400.0 * 400.0) {
            RunwaySpec spec{};
            for (int e = 0; e < 2; e++) {
                spec.end[e] = g_enu.of(r.end[e].lat, r.end[e].lon, r.elevationFt * 0.3048);
            }
            spec.width = r.widthFt * 0.3048f;
            runways.push_back(spec);
        }
    }
    clearAirfield();
    setFarRunways(runways.data(), (int)runways.size());
    if (g_traffic == nullptr) {
        g_traffic = (Aircraft *)heap_caps_malloc(sizeof(Aircraft) * MAX_TRAFFIC, MALLOC_CAP_SPIRAM);
    }
    g_pilot = Pilot();
    g_pilotAt = 0;
    g_enroute = g_traffic != nullptr;
    Serial.printf("[3d] high up: %u lines on the ground and %u runways in %.1fms\n", (unsigned)lines.size(),
                  (unsigned)runways.size(), (micros() - t0) / 1000.0f);
    return g_enroute;
}

void enrouteEnd() {
    g_enroute = false;
    setGroundLines(nullptr, 0);
    setFarRunways(nullptr, 0);
    setAirfield(nullptr, 0, nullptr, 0);
}

// The cruise at t: the one followed flown by the pilot, the others on their
// way, and the shot - close behind, wide, or from ahead and below.
void enrouteAt(float t, TimeOfDay time, Scene &scene, Camera &cam, const char **shot) {
    const float SPEED = 450 * 0.5144f, FEET = 35000;
    auto courseAt = [](float u) { return 205 + std::max(0.0f, std::min(30.0f, u - 20)); };
    // Where it is: along 205 to 20s, round the turn, then along 235.
    float e = 0, n = 0;
    {
        float straight = std::min(t, 20.0f), turning = std::max(0.0f, std::min(t, 50.0f) - 20), after = std::max(0.0f, t - 50);
        float c0 = 205 * (float)M_PI / 180;
        e += sinf(c0) * SPEED * straight, n += cosf(c0) * SPEED * straight;
        for (float u = 0; u < turning; u += 0.25f) {
            float c = (205 + u) * (float)M_PI / 180;
            e += sinf(c) * SPEED * 0.25f, n += cosf(c) * SPEED * 0.25f;
        }
        float c1 = 235 * (float)M_PI / 180;
        e += sinf(c1) * SPEED * after, n += cosf(c1) * SPEED * after;
    }
    float turnRate = (courseAt(t + 0.5f) - courseAt(t - 0.5f)) * -(float)M_PI / 180;
    g_pilot.fly(t - g_pilotAt, Family::NARROWBODY, turnRate, 450, 0, false);
    g_pilotAt = t;
    double lat = CRUISE_LAT + n / 111320.0, lon = CRUISE_LON + e / (111320.0 * cos(CRUISE_LAT * M_PI / 180));
    Pose pose = {g_enu.of(lat, lon, FEET * 0.3048f), (90 - courseAt(t)) * (float)M_PI / 180, g_pilot.pitch, g_pilot.roll};
    Look look = lookFor("A320", 0xFF6600);
    paintAirline(look, "EZY");
    Config config;
    config.gear = false;
    config.aileron = g_pilot.aileron;
    config.elevator = g_pilot.elevator;
    config.rudder = g_pilot.rudder;
    scene.followed = {pose, look, config};
    for (int i = 0; i < OTHER_COUNT; i++) {
        const Other &o = OTHERS[i];
        float c = o.course * (float)M_PI / 180, v = o.knots * 0.5144f;
        float oe = o.east + sinf(c) * v * t, on = o.north + cosf(c) * v * t;
        double olat = CRUISE_LAT + on / 111320.0, olon = CRUISE_LON + oe / (111320.0 * cos(CRUISE_LAT * M_PI / 180));
        Look ol = lookFor(o.type, 0x1B2A55, o.military);
        paintAirline(ol, o.callsign);
        Config oc;
        oc.gear = false;
        g_traffic[i] = {{g_enu.of(olat, olon, (o.feet + o.fpm * t / 60) * 0.3048f), (90 - o.course) * (float)M_PI / 180,
                         atanf(o.fpm * 0.00508f / v) + 0.04f},
                        ol, oc};
    }
    scene.traffic = g_traffic;
    scene.trafficCount = OTHER_COUNT;
    scene.seconds = t;
    scene.sun = sunFor(time);
    scene.earthRadius = 6371000;
    scene.visibility = 140000;
    scene.stemsAbove = 0;
    scene.trailsAbove = 8000;
    scene.landAndSea = true;
    scene.landingLights = false;
    if (g_cover && groundpack::want(lat, lon, 45, FEET * 0.3048f)) {
        scene.cover = roundCover(groundpack::window(), CRUISE_LAT, CRUISE_LON);
    }
    float size = sizeOf(look), phase = fmodf(t, 30);
    if (phase < 10) {
        *shot = "chase";
        cam = orbitCamera(pose, 18 + phase * 2, 9, 70 * size, 48);
    } else if (phase < 20) {
        *shot = "wide";
        cam = orbitCamera(pose, 160 - (phase - 10) * 4, 22, 230 * size, 50);
    } else {
        *shot = "from below";
        cam = orbitCamera(pose, 200 + (phase - 20) * 3, -18, 120 * size, 52);
    }
}

// The skyline for a frame with the land's cover - as view3d's setHorizon()
// has it: the eye's place from the camera, about the airport or the cruise's
// origin; with `wait`, till the ground pack has worked a horizon out.
void benchHorizon(Scene &scene, const Camera &cam, bool wait) {
    if (!g_cover) {
        return;
    }
    Vec3 eye = cam.eye;
    double lat0 = g_enroute ? CRUISE_LAT : g_airportLat0, lon0 = g_enroute ? CRUISE_LON : g_airportLon0;
    double lat = lat0 + eye.y / 110574.0, lon = lon0 + eye.x / (111320.0 * cos(lat0 * M_PI / 180));
    float heightM = g_enroute ? eye.z + (eye.x * eye.x + eye.y * eye.y) / (2 * 6371000.0f) : eye.z + g_airportFieldM;
    scene.eyeHeightM = heightM;
    if (heightM > 4000) {
        return;  // as the view: no skyline up there, nor one worked out for it
    }
    groundpack::wantHorizon(lat, lon, heightM);
    for (uint32_t start = millis(); wait && millis() - start < 3000; delay(20)) {
        const groundpack::Horizon *hz = groundpack::horizon();
        if (hz != nullptr && fabs(hz->lat - lat) < 0.003 && fabs(hz->lon - lon) < 0.005 &&
            fabsf(hz->heightM - heightM) < 30) {
            break;
        }
    }
    const groundpack::Horizon *hz = groundpack::horizon();
    scene.horizon = hz;
    scene.horizonTurn = (float)((lon - lon0) * M_PI / 180 * sin(lat * M_PI / 180));
    scene.eyeHeightM = heightM;
    if (hz != nullptr) {
        // Where it was worked out from, in the scene's frame.
        scene.horizonAt = g_enroute ? g_enu.of(hz->lat, hz->lon, hz->heightM)
                                    : Vec3{(float)((hz->lon - lon0) * 111320.0 * cos(lat0 * M_PI / 180)),
                                           (float)((hz->lat - lat0) * 110574.0), hz->heightM - g_airportFieldM};
    }
}

// The landing from `from` to `to` seconds in, in real time, pipelined: the
// helper draws each frame - and the PPA pushes it, a band behind - while this
// core prepares the next. A frame then takes as long as the slowest of the
// three, rather than all of them one after another.
void playPipelined(TimeOfDay time, int vw, int vh, float from = 0, float to = -1, int rows = 16, bool simd = true,
                   bool directed = false) {
    vh -= vh % rows;
    setView(vw, vh);
    g_pipeRows = rows;
    setSimd(simd);
    uint32_t discard[6];
    spanStats(discard, true);
    uint64_t discard64[11];
    coverStats(discard64, true);
    seeStats(discard64, true);
    aircraftStats(discard64, true);
    size_t bytes = 3 * pipeColorBytes();
    // Held from one run to the next of a command, given back with the rest
    // by release(): taken afresh for each, the chip's own memory had broken
    // up by the third - the network's and the card's own small takings
    // landing in the gaps - and there was no room left for them.
    bool room = ready(false);
    if (g_pipeBytes != pipeColorBytes()) {
        releasePipe();
        for (void *&p : g_pipe) {
            room = room && (p = heap_caps_aligned_alloc(64, pipeColorBytes(), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        }
        g_pipeBytes = room ? pipeColorBytes() : 0;
    }
    if (!room) {
        Serial.printf("[3d] no room for the pipeline's bands: %uKB free, largest block %uKB\n",
                      (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                      (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
        releasePipe();
        return;
    }
    if (vw < W || vh < H) {
        drawBorder();
    }
    // Aiming at 30 a second: over 33ms and aircraft go to simpler models
    // sooner; under 25, back towards the finest the view can show.
    constexpr uint32_t BUDGET_US = 33000, SPARE_US = 25000;
    float detailPx = 60, tinyPx = 12;
    uint32_t start = millis(), frames = 0, worst = 0, sum = 0, overBudget = 0, infoMs = 0, prepSum = 0, drawWait = 0;
    int trafficMost = 0;
    size_t lowestInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    float duration = (to < 0 ? landingSeconds() : to) - from;
    const char *shot = nullptr, *nextShot = nullptr;
    // Where the time goes: the helper's drawing by stage, and its waits for
    // the pushes; this core's preparing by step.
    memset(g_stageUs, 0, sizeof g_stageUs);
    g_pushWaitUs = 0;
    g_overlayUs = 0;
    uint32_t pushBusy, pushes;
    screen::pushStats(pushBusy, pushes);  // from here
    g_profile = true;
    uint32_t beginSum = 0, aircraftSum = 0, endSum = 0, captionsSum = 0, stepSum[STEPS] = {};
    // Directed: shots picked by a Director from the flight as though it were
    // live - its pose, and its speed and climb from either side of it.
    Director director;
    auto prepareAt = [&](float t) {
        if (g_enroute) {
            Scene scene;
            Camera cam;
            enrouteAt(t, time, scene, cam, &nextShot);
            scene.detailPx = detailPx;
            scene.tinyPx = tinyPx;
            trafficMost = std::max(trafficMost, scene.trafficCount);
            benchHorizon(scene, cam, false);
            prepareBegin(vw, vh, rows, true, scene, cam);
            prepareAircraft(0, 1);
            prepareEnd();
            return;
        }
        Scene scene = sceneAt(t, time);
        scene.detailPx = detailPx;
        scene.tinyPx = tinyPx;
        trafficMost = std::max(trafficMost, scene.trafficCount);
        Camera cam;
        if (directed) {
            Pose a = followedAt(t - 0.1f), b = followedAt(t + 0.1f);
            Flight flight;
            flight.aircraft = scene.followed;
            flight.speedKt = sqrtf((b.at.x - a.at.x) * (b.at.x - a.at.x) + (b.at.y - a.at.y) * (b.at.y - a.at.y)) /
                             0.2f * 1.944f;
            flight.climbFpm = (b.at.z - a.at.z) / 0.2f * 196.85f;
            flight.onGround = scene.followed.pose.at.z <= 0.01f && (g_departing || t >= landingTouchdown() - 0.05f);
            flight.departing = g_departing;
            flight.runway = g_departing ? g_departRunway : g_landRunway;
            flight.end = g_departing ? g_departEnd : g_landEnd;
            Pose shown;
            bool replaying = false;
            cam = director.next(flight, t, &nextShot, &shown, &replaying);
            if (replaying) {
                scene.followed.pose = shown;
            }
        } else {
            cam = cameraAt(View::CINEMA, t, scene.followed.pose, &nextShot);
        }
        benchHorizon(scene, cam, false);
        prepareBegin(vw, vh, rows, true, scene, cam);
        prepareAircraft(0, 1);
        prepareEnd();
    };
    // The first frame on its own; then each loop draws one while preparing
    // the one after.
    float t = from;
    prepareAt(t);
    uint32_t frameStart = micros();
    for (;;) {
        flip();
        Counts steps = counts();
        beginSum += steps.beginUs;
        aircraftSum += steps.aircraftUs[0];
        endSum += steps.endUs;
        for (int step = 0; step < STEPS; step++) {
            stepSum[step] += steps.stepUs[step];
        }
        shot = nextShot;
        float shown = t;
        uint32_t c0 = micros();
        if (g_enroute) {
            enrouteCaptions(shown);
        } else {
            captions(shown, shot);
        }
        captionsSum += micros() - c0;
        g_job = Job::PIPELINE;
        xTaskNotifyGive(g_helper);
        t = from + (millis() - start) / 1000.0f;
        bool last = t > from + duration;
        uint32_t p0 = micros();
        if (!last) {
            prepareAt(t);
        }
        prepSum += micros() - p0;
        uint32_t w0 = micros();
        if (!waitForHelper()) {
            break;
        }
        drawWait += micros() - w0;
        g_job = Job::BANDS;
        if (millis() - infoMs > 500 && !g_enroute) {
            infoMs = millis();
            drawInfo(shown, shot, frames / std::max(0.001f, shown - from));
        }
        uint32_t now = micros(), took = now - frameStart;
        frameStart = now;
        frames++;
        sum += took;
        worst = std::max(worst, took);
        overBudget += took > BUDGET_US;
        if (took > BUDGET_US) {
            detailPx = std::min(400.0f, detailPx * 1.25f);
            tinyPx = std::min(200.0f, tinyPx * 1.25f);
        } else if (took < SPARE_US) {
            detailPx = std::max(16.0f, detailPx / 1.1f);
            tinyPx = std::max(5.0f, tinyPx / 1.1f);
        }
        lowestInternal = std::min(lowestInternal, heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        if (last || g_stalled || g_pushFailed) {
            break;
        }
        vTaskDelay(1);  // the idle tasks, and the watchdog, their turn
    }
    g_profile = false;
    Serial.printf("[3d]   a frame's drawing: sky %.1f, ground %.1f, shadows %.1f, depth clear %.1f, solids %.1f, lines "
                  "and lights %.1f, labels and captions over it %.2f, waiting on pushes %.1f ms; preparing: begin "
                  "%.1f, aircraft %.1f, end %.1f, labels placed %.2f ms\n",
                  g_stageUs[1][0] / 1000.0f / frames, g_stageUs[1][1] / 1000.0f / frames,
                  g_stageUs[1][2] / 1000.0f / frames, g_stageUs[1][3] / 1000.0f / frames,
                  g_stageUs[1][4] / 1000.0f / frames, g_stageUs[1][5] / 1000.0f / frames,
                  g_overlayUs / 1000.0f / frames, g_pushWaitUs / 1000.0f / frames, beginSum / 1000.0f / frames,
                  aircraftSum / 1000.0f / frames, endSum / 1000.0f / frames, captionsSum / 1000.0f / frames);
    char detail[400];
    int at = 0;
    for (int step = 0; step < STEPS && at < (int)sizeof detail; step++) {
        at += snprintf(detail + at, sizeof detail - at, "%s%s %.2f", step ? ", " : "", stepName(step),
                       stepSum[step] / 1000.0f / frames);
    }
    Serial.printf("[3d]   preparing step by step, ms a frame: %s\n", detail);
    screen::pushStats(pushBusy, pushes);
    Serial.printf("[3d]   pushes: %.1f a frame, %.2f ms each start to finish, %.1f ms a frame\n",
                  pushes / (float)frames, pushBusy / 1000.0f / std::max<uint32_t>(1, pushes), pushBusy / 1000.0f / frames);
    clearOverlay();
    Counts last = counts();
    Serial.printf("[3d]   the last frame: %d triangles, %d lines, %d lights; aircraft %d full, %d simple, %d tiny, %d as "
                  "lights\n",
                  last.triangles, last.lines, last.lights, last.aircraftFull, last.aircraftSimple, last.aircraftTiny,
                  last.aircraftDots);
    uint32_t spans[6];
    spanStats(spans, true);
    uint32_t allSpans = std::max<uint32_t>(1, spans[0] + spans[1] + spans[2]),
             allPixels = std::max<uint32_t>(1, spans[3] + spans[4] + spans[5]);
    Serial.printf("[3d]   solids' spans a frame: %.0f, of which %.0f%% under 8px (%.0f%% of the pixels), %.0f%% 8-23px "
                  "(%.0f%%), %.0f%% 24px and more, PIE's (%.0f%%); %.1f pixels a span\n",
                  allSpans / (float)frames, 100.0f * spans[0] / allSpans, 100.0f * spans[3] / allPixels,
                  100.0f * spans[1] / allSpans, 100.0f * spans[4] / allPixels, 100.0f * spans[2] / allSpans,
                  100.0f * spans[5] / allPixels, allPixels / (float)allSpans);
    uint64_t see[7];
    seeStats(see, true);
    Serial.printf("[3d]   the solids and see-through together: %.2f ms by the clock, %.2f ms by the cycle counter\n",
                  see[5] / 1000.0f / frames, see[6] / (getCpuFrequencyMhz() * 1000.0f * frames));
    float frameMs = getCpuFrequencyMhz() * 1000.0f * frames;
    Serial.printf("[3d]   see-through a frame: %.2f ms, %.0f pixels looked at, %.1f cycles each; the solids %.2f ms, "
                  "%.0f triangles a band at a time and %.0f rows of them, %.0f cycles a row\n",
                  see[0] / frameMs, see[1] / (float)frames, see[0] / (float)std::max<uint64_t>(1, see[1]),
                  see[2] / frameMs, see[4] / (float)frames, see[3] / (float)frames,
                  see[2] / (float)std::max<uint64_t>(1, see[3]));
    uint64_t plane[6];
    aircraftStats(plane, true);
    Serial.printf("[3d]   the one followed a frame: its corners placed %.2f ms (%.0f of them, %.0f cycles each), its "
                  "faces %.2f ms (%.0f of them, %.0f cycles each, making %.0f triangles)\n",
                  plane[0] / frameMs, plane[3] / (float)frames, plane[0] / (float)std::max<uint64_t>(1, plane[3]),
                  plane[1] / frameMs, plane[4] / (float)frames, plane[1] / (float)std::max<uint64_t>(1, plane[4]),
                  plane[5] / (float)frames);
    uint64_t cover[11];
    coverStats(cover, true);
    if (cover[3] > 0) {
        float perMs = getCpuFrequencyMhz() * 1000.0f * frames;
        Serial.printf("[3d]   the land's cover a frame, %s: cells read %.2f ms (%.0f rows of them, %.1f cycles a cell, "
                      "%.1f of it the innermost loop's), their colours %.2f ms (%.0f worked out), pixels written %.2f "
                      "ms\n",
                      simd ? "assembly and PIE" : "C", cover[0] / perMs, cover[3] / (float)frames,
                      cover[0] / (float)std::max<uint64_t>(1, cover[6]), cover[5] / (float)std::max<uint64_t>(1, cover[6]),
                      cover[1] / perMs, cover[4] / (float)frames, cover[2] / perMs);
        Serial.printf("[3d]   and preparing: the rows' places on its grid %.2f ms (%.0f rows worked out, %.0f cycles "
                      "a row), stored %.2f ms; the skyline %.2f ms\n",
                      cover[7] / perMs, cover[8] / (float)frames, cover[7] / (float)std::max<uint64_t>(1, cover[8]),
                      cover[9] / perMs, cover[10] / perMs);
    }
    Serial.printf("[3d] pipelined %dx%d, %d-row bands (%uKB), %s, for %.1fs: %u frames, %.1f a second, average %.1fms (preparing %.1f; waiting "
                  "on the drawing %.1f), worst frame %.1fms, %u over %.0fms; detail ended at %.0f/%.0fpx; up to %d "
                  "other aircraft; internal RAM at least %uKB free%s%s\n",
                  vw, vh, rows, (unsigned)(bytes / 1024), simd ? "PIE" : "C", duration, (unsigned)frames,
                  frames / duration, sum / 1000.0f / frames, prepSum / 1000.0f / frames, drawWait / 1000.0f / frames,
                  worst / 1000.0f, (unsigned)overBudget,
                  BUDGET_US / 1000.0f, detailPx, tinyPx, trafficMost, (unsigned)(lowestInternal / 1024),
                  g_pushFailed ? " PUSH FAILED" : "", g_stalled ? " STALLED" : "");
    setView(W, H);
}

// The landing from `from` to `to` seconds in, in real time, in a view
// vw x vh - with a border round it when that is smaller than the screen.
void play(TimeOfDay time, int vw = W, int vh = H, float from = 0, float to = -1) {
    if (!ready()) {
        return;
    }
    // Whole bands only: a push of fewer than 8 rows - the last band of a view
    // 540 or 630 tall - now and then never finished, and the PPA hung behind
    // it, and the board with it.
    vh -= vh % BAND_ROWS;
    setView(vw, vh);
    if (vw < W || vh < H) {
        g_step[0] = BORDER;
        drawBorder();
        g_step[0] = IDLE;
    }
    // Aiming at 30 a second, for which a frame has 33ms: over 36, and
    // aircraft go to simpler models sooner; under 28, back to finer ones.
    constexpr uint32_t BUDGET_US = 36000, SPARE_US = 28000;
    float detailPx = 60, tinyPx = 12;
    uint32_t start = millis(), frames = 0, worst = 0, sum = 0, overBudget = 0, infoMs = 0;
    uint32_t prepSum = 0, prepWorst = 0, drawSum = 0;
    int trafficMost = 0;
    size_t lowestInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    float duration = (to < 0 ? landingSeconds() : to) - from;
    for (;;) {
        float t = from + (millis() - start) / 1000.0f;
        if (t > from + duration) {
            break;
        }
        uint32_t t0 = micros();
        Scene scene = sceneAt(t, time);
        scene.detailPx = detailPx;
        scene.tinyPx = tinyPx;
        trafficMost = std::max(trafficMost, scene.trafficCount);
        const char *shot = nullptr;
        Camera cam = cameraAt(View::CINEMA, t, scene.followed.pose, &shot);
        prepareBoth(vw, vh, scene, cam);
        captions(t, shot);
        uint32_t t1 = micros();
        prepSum += t1 - t0;
        prepWorst = std::max(prepWorst, t1 - t0);
        draw(true, Sink::PANEL);
        drawSum += micros() - t1;
        lowestInternal = std::min(lowestInternal, heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        if (millis() - infoMs > 500) {
            infoMs = millis();
            g_step[0] = INFO;
            drawInfo(t, shot, frames / std::max(0.001f, t - from));
            g_step[0] = IDLE;
        }
        if (g_stalled || g_pushFailed) {
            Serial.printf("[3d] stopped at %.1fs: %s\n", t, g_stalled ? "a stall" : "a push failed");
            break;
        }
        uint32_t took = micros() - t0;
        frames++;
        sum += took;
        worst = std::max(worst, took);
        overBudget += took > BUDGET_US;
        if (took > BUDGET_US) {
            detailPx = std::min(400.0f, detailPx * 1.25f);
            tinyPx = std::min(200.0f, tinyPx * 1.25f);
        } else if (took < SPARE_US) {
            detailPx = std::max(40.0f, detailPx / 1.1f);
            tinyPx = std::max(8.0f, tinyPx / 1.1f);
        }
        vTaskDelay(1);  // the idle tasks, and the watchdog, their turn
    }
    clearOverlay();
    Serial.printf("[3d] played %dx%d for %.1fs: %u frames, %.1f a second, average %.1fms (preparing %.1f, worst %.1f; "
                  "drawing and pushing %.1f), worst frame %.1fms, %u over %.0fms; detail ended at %.0f/%.0fpx; up to "
                  "%d other aircraft; internal RAM at least %uKB free%s\n",
                  vw, vh, duration, (unsigned)frames, frames / duration, sum / 1000.0f / frames,
                  prepSum / 1000.0f / frames, prepWorst / 1000.0f, drawSum / 1000.0f / frames, worst / 1000.0f,
                  (unsigned)overBudget, BUDGET_US / 1000.0f, detailPx, tinyPx, trafficMost,
                  (unsigned)(lowestInternal / 1024), g_pushFailed ? " PUSH FAILED" : "");
    setView(W, H);
}

// One frame, prepared as `scene` from `cam` - the whole screen, as the
// comparing has it - drawn in C into the canvas and then with assembly and
// PIE against it: how many pixels differ - nought,
// or the two ways aren't the same.
int coverMismatches(const Scene &given, const Camera &cam) {
    Scene scene = given;
    benchHorizon(scene, cam, true);
    prepare(W, H, BAND_ROWS, true, scene, cam);
    setSimd(false);
    draw(true, Sink::CANVAS);
    setSimd(true);
    g_mismatched = 0;
    draw(true, Sink::COMPARE);
    return (int)g_mismatched;
}

// The ground pack open, and its tiles round (lat, lon) read in - till the
// reads stop for two seconds, or 20 seconds: a run timed from there, as
// the live view is once it has been at a place a while. False with no pack.
bool coverAt(double lat, double lon) {
    groundpack::open();
    uint32_t start = millis();
    while (!groundpack::isOpen() && millis() - start < 5000) {
        delay(50);
    }
    if (!groundpack::want(lat, lon, 45)) {
        Serial.println("[3d] no ground pack - /overhead/ground.bin - to cover the land with");
        return false;
    }
    int held = 0, detailHeld = 0, finerHeld = 0, reads = -1, last = -2;
    uint32_t readUs = 0, finerUs = 0, quietSince = millis();
    while (millis() - start < 20000) {
        groundpack::want(lat, lon, 45);
        groundpack::stats(held, detailHeld, finerHeld, reads, readUs, finerUs);
        reads += finerHeld;  // a level worked out counts as a read: quiet once they are done too
        if (reads != last) {
            last = reads;
            quietSince = millis();
        } else if (millis() - quietSince > 2000) {
            break;
        }
        delay(50);
    }
    Serial.printf("[3d] the land's cover: %d tiles of it in close, %d of them in detail and %d finer still, %d read "
                  "in %.1fs, the finer worked out in %.1fs\n",
                  held, detailHeld, finerHeld, reads - finerHeld, readUs / 1e6f, finerUs / 1e6f);
    return true;
}

}  // namespace

namespace {
void runCommand(char c);
}  // namespace

extern "C" void scene3d_latencies(uint32_t out[12]);

void bench3dCommand(char c, void (*repaint)()) {
    g_repaint = repaint;
    runCommand(c);
    // For tools/bench3d.py to know it's over.
    Serial.println("[3d] done");
}

namespace {
void runCommand(char c) {
    switch (c) {
    case '3':
        bench();
        break;
    case 'l':
        labelsBench();
        break;

    case '4':
    case '5':
    case '6':
        play(c == '4' ? TimeOfDay::DAY : c == '5' ? TimeOfDay::DUSK : TimeOfDay::NIGHT);
        release();
        g_repaint();
        break;
    case '1': {
        // The director at Heathrow, as though live: a landing on 27L, its
        // touchdown replayed, then a 787 off 27R - busy, at 960x544.
        if (useAirport("LHR", "27L", "27R")) {
            playPipelined(TimeOfDay::DAY, 960, 544, 0, -1, 16, true, true);
            g_departing = true;
            playPipelined(TimeOfDay::DAY, 960, 544, 0, 95, 16, true, true);
            g_departing = false;
        }
        g_busy = false;
        setAirfield(nullptr, 0, nullptr, 0);
        snprintf(g_runwayName, sizeof g_runwayName, "09");
        release();
        g_repaint();
        break;
    }
    case 'c': {
        // Heathrow as '0' has it, by day, but in plain C throughout - no PIE
        // for the spans, the fills or the labels: what the vector
        // instructions are worth, set against '0'.
        labels3d::setSimd(false);
        if (useAirport("LHR", "27L", "27R")) {
            playPipelined(TimeOfDay::DAY, 960, 544, 0, -1, 16, false);
        }
        labels3d::setSimd(true);
        g_busy = false;
        setAirfield(nullptr, 0, nullptr, 0);
        snprintf(g_runwayName, sizeof g_runwayName, "09");
        release();
        setSimd(true);
        g_repaint();
        break;
    }
    case '0': {
        // Busy airports, at the view's size to be: Heathrow by day landing
        // 27L with departures off 27R, and Kennedy at dusk landing 22L with
        // departures off 22R.
        if (useAirport("LHR", "27L", "27R")) {
            playPipelined(TimeOfDay::DAY, 960, 544, 0, -1, 16, true);
        }
        if (useAirport("JFK", "22L", "22R")) {
            playPipelined(TimeOfDay::DUSK, 960, 544, 0, -1, 16, true);
        }
        g_busy = false;
        setAirfield(nullptr, 0, nullptr, 0);
        snprintf(g_runwayName, sizeof g_runwayName, "09");
        release();
        g_repaint();
        break;
    }
    case '9': {
        // The busiest half minute - short final, touchdown, rollout - at each
        // size of view, the smaller ones with their border.
        const struct {
            int w, h;
        } SIZES[] = {{1280, 720}, {1120, 632}, {960, 544}, {800, 448}};
        for (auto size : SIZES) {
            play(TimeOfDay::DAY, size.w, size.h, 12, 42);
        }
        release();
        g_repaint();
        break;
    }
    case '7': {
        // A showcase frame left in the canvas, for an "##S" straight after it to
        // take - the next of five each time, at the view's size to be, with its
        // border.
        static int next = 0;
        struct Shot {
            const char *airport, *land, *depart;
            TimeOfDay time;
            float t;
            View view;
            const char *type;
        };
        static const Shot SHOTS[] = {
            {"LHR", "27L", "27R", TimeOfDay::DAY, 21.5f, View::CINEMA, "A320"},
            {"LHR", "27L", "27R", TimeOfDay::DUSK, 26.6f, View::CINEMA, "A320"},
            {"JFK", "22L", "22R", TimeOfDay::NIGHT, 26.6f, View::CINEMA, "B77W"},
            {"LHR", "27L", "27R", TimeOfDay::DAY, 66, View::APRON, "A320"},
            {"JFK", "22L", "22R", TimeOfDay::DAY, 9, View::CINEMA, "A388"},
        };
        const Shot &shot = SHOTS[next++ % 5];
        if (!ready() || !useAirport(shot.airport, shot.land, shot.depart)) {
            return;
        }
        setView(960, 544);
        drawBorder();
        Scene scene = sceneAt(shot.t, shot.time);
        scene.followed.look = lookFor(shot.type, 0x1B2A55);
        scene.detailPx = 16;
        scene.tinyPx = 5;
        const char *name = nullptr;
        Camera cam = shot.view == View::APRON ? orbitCamera(scene.followed.pose, 15, 10, 90, 55)
                                              : cinemaCamera(shot.t, &name, sizeOf(scene.followed.look));
        prepare(g_vw, g_vh, BAND_ROWS, true, scene, cam);
        captions(shot.t, name);
        draw(true, Sink::CANVAS);
        clearOverlay();
        drawInfo(shot.t, name, 0);
        screen::markAllDirty();
        screen::flush();
        setView(W, H);
        g_busy = false;
        setAirfield(nullptr, 0, nullptr, 0);
        release();
        break;
    }
    case '8':
        g_repaint();
        break;
    case 'h':
    case 'n': {
        // High up: the cruise over the Firth of Clyde, a minute of it by
        // day ('h') or the half of it at night ('n'), at the view's size to be.
        size_t before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        Serial.printf("[3d] models: %uKB of PSRAM\n", (unsigned)(buildModels() / 1024));
        if (enrouteSetup()) {
            Serial.printf("[3d] the country below: %uKB of PSRAM\n",
                          (unsigned)((before - heap_caps_get_free_size(MALLOC_CAP_SPIRAM)) / 1024));
            playPipelined(c == 'h' ? TimeOfDay::DAY : TimeOfDay::NIGHT, 960, 544, 0, c == 'h' ? 60 : 30, 16, true);
        }
        enrouteEnd();
        release();
        g_repaint();
        break;
    }
    case 'q': {
        // The cycle counter against the microsecond clock: busy, then reading
        // PSRAM a cache line at a time - whether a stall on memory counts -
        // on this core, then on the other, where the drawing is.
        uint8_t *buf = (uint8_t *)heap_caps_malloc(4 << 20, MALLOC_CAP_SPIRAM);
        if (buf == nullptr) {
            break;
        }
        static uint8_t *s_buf;
        s_buf = buf;
        static SemaphoreHandle_t s_done;
        s_done = xSemaphoreCreateBinary();
        auto calibrate = [](void *arg) {
            uint8_t *b = s_buf;
            for (int run = 0; run < 3; run++) {
                uint32_t t0 = micros(), c0 = esp_cpu_get_cycle_count();
                volatile uint32_t sink = 0;
                if (run == 0) {
                    for (int i = 0; i < 1000000; i++) {
                        sink = sink + i;
                    }
                } else if (run == 1) {
                    for (int i = 0; i < (4 << 20); i += 64) {
                        sink = sink + b[i];
                    }
                } else {
                    // Half a second, the other tasks on the core their turns.
                    while (micros() - t0 < 500000) {
                        sink = sink + 1;
                    }
                }
                uint32_t us = micros() - t0, cycles = esp_cpu_get_cycle_count() - c0;
                Serial.printf("[3d] core %d, %s: %u us, %u cycles - %.1f cycles a microsecond\n", xPortGetCoreID(),
                              run == 0 ? "busy" : run == 1 ? "PSRAM a line at a time" : "half a second, sharing",
                              (unsigned)us, (unsigned)cycles, cycles / (float)us);
            }
            xSemaphoreGive(s_done);
            if (arg != nullptr) {
                vTaskDelete(nullptr);
            }
        };
        calibrate(nullptr);
        xTaskCreatePinnedToCore(calibrate, "calibrate", 4096, (void *)1, 3, nullptr, 0);
        xSemaphoreTake(s_done, pdMS_TO_TICKS(5000));
        xSemaphoreTake(s_done, pdMS_TO_TICKS(5000));
        vSemaphoreDelete(s_done);
        heap_caps_free(buf);
        break;
    }
    case 'T': {
        // What the land at its height costs (##iT): Glasgow's departure off
        // 23 by day - lined up, rolling, the climb out - from the chase and
        // the director's camera, at the 3D screen's size. Each frame's march
        // four times over, the first from cold; its bands drawn on this core
        // alone; and its march again while the other core draws the frame
        // before, as the view has them.
        g_departing = true;
        g_cover = true;
        setView(1008, 560);
        bool ok = useAirport("GLA", "23", "05") && coverAt(g_airportLat0, g_airportLon0) && ready();
        uint32_t since = millis();
        while (ok && groundpack::terrainGrid() == nullptr && millis() - since < 15000) {
            groundpack::want(g_airportLat0, g_airportLon0, 45);
            delay(100);
        }
        const groundpack::TerrainGrid *grid = groundpack::terrainGrid();
        if (!ok || grid == nullptr) {
            Serial.println("[3d] terrain bench: no ground pack, or no heights from it");
        }
        {
            // The core's latencies the terrain's loops wait on: each of 64
            // dependent, the loop's own two each turn included.
            uint32_t lat[12];
            scene3d_latencies(lat);
            // Cycles an instruction: 256 turns of each test's body - two
            // chains' 512 instructions, four chains' 1024 - the pairs' figure
            // the pair's.
            auto per = [&](int i, float n) { return lat[i] / n; };
            Serial.printf("[3d] latencies, cycles each: fadd.s %.2f, fmul.s %.2f, fmadd.s %.2f; fmadd.s 2 chains "
                          "%.2f, 4 chains %.2f, independent %.2f; fcvt.w.s + fcvt.s.w %.2f, fcvt.w.s independent "
                          "%.2f; fdiv.s %.2f, independent %.2f; lw chased %.2f; fmv.x.w + fmv.w.x %.2f\n",
                          per(0, 256), per(1, 256), per(2, 256), per(3, 512), per(4, 1024), per(5, 256),
                          per(6, 256), per(7, 256), per(8, 256), per(9, 256), per(10, 256), per(11, 256));
        }
        // One frame's: everything above, said.
        auto measure = [&](float t, const Scene &scene, const Camera &cam, const char *shot) {
            uint64_t st[8];
            uint32_t march[4] = {}, samples = 0, rows = 0, skips = 0;
            terrainStats(st, true);
            for (int run = 0; run < 4; run++) {
                prepareBegin(g_vw, g_vh, BAND_ROWS, true, scene, cam);
                prepareAircraft(0, 1);
                prepareEnd();
                flip();
                terrainStats(st, true);
                march[run] = (uint32_t)st[0];
                samples = (uint32_t)st[1];
                rows = (uint32_t)st[2];
                skips = (uint32_t)st[3];
            }
            terrainBandInstructions();
            terrainBandHanded();
            terrainBandSwitched();
            draw(false, Sink::CANVAS);
            uint64_t bandInstructions = terrainBandInstructions(), handed = terrainBandHanded(),
                     switched = terrainBandSwitched();
            terrainStats(st, true);
            uint32_t bandCycles = (uint32_t)st[4], bandSamples = (uint32_t)st[5];
            // The same bands in C alone, against what the assembly drew.
            g_mismatched = 0;
            g_misCount = 0;
            g_misBox[0] = g_misBox[1] = 1 << 30;
            g_misBox[2] = g_misBox[3] = -1;
            terrainProbe(4);
            draw(false, Sink::COMPARE);
            terrainProbe(0);
            int mismatched = g_mismatched;
            uint64_t cOnly[8];
            terrainStats(cOnly, true);
            if (mismatched > 0) {
                Serial.printf("[3d]   differing in x %d-%d, y %d-%d; the first %04x in assembly, %04x in C\n",
                              g_misBox[0], g_misBox[2], g_misBox[1], g_misBox[3], g_misWas, g_misNow);
                // Each row's runs of them, and the pixels at each run's start.
                int n = std::min(1024, g_misCount.load());
                for (int k = 0; k < n;) {
                    char line[600];
                    int len = snprintf(line, sizeof line, "[3d]   y %d:", g_mis[k].y);
                    int y = g_mis[k].y;
                    while (k < n && g_mis[k].y == y && len < (int)sizeof line - 40) {
                        int j = k;
                        while (j + 1 < n && g_mis[j + 1].y == y && g_mis[j + 1].x == g_mis[j].x + 1) {
                            j++;
                        }
                        len += snprintf(line + len, sizeof line - len, " %d-%d %04x/%04x", g_mis[k].x, g_mis[j].x,
                                        g_mis[k].was, g_mis[k].now);
                        k = j + 1;
                    }
                    while (k < n && g_mis[k].y == y) {
                        k++;
                    }
                    Serial.println(line);
                }
            }
            g_misCount = 0;
            if (mismatched > 0) {
                // And C in the assembly's order, against the assembly.
                g_mismatched = 0;
                terrainProbe(4 | 16);
                draw(false, Sink::COMPARE);
                terrainProbe(0);
                Serial.printf("[3d]   in C, row-pair by row-pair as the assembly goes: %d pixels differ\n",
                              (int)g_mismatched);
                g_misCount = 0;
                uint64_t again[8];
                terrainStats(again, true);
            }
            terrainBandInstructions();
            terrainBandHanded();
            terrainBandSwitched();
            Serial.printf("[3d]   the bands in C alone: %d pixels differ from the assembly's; %.1fms in C, %.1fms in "
                          "assembly\n",
                          mismatched, cOnly[4] / 360000.0f, bandCycles / 360000.0f);
            Serial.printf("[3d]   the bands: %u cells the long way, %u maps taken up again in assembly, %u colours "
                          "worked out, %u samples left for C; %.1f instructions a sample\n",
                          (unsigned)st[6], (unsigned)switched, (unsigned)st[7], (unsigned)handed,
                          bandSamples ? (float)bandInstructions / bandSamples : 0.0f);
            g_sink = Sink::CANVAS;
            g_next = 0;
            xTaskNotifyGive(g_helper);
            prepareBegin(g_vw, g_vh, BAND_ROWS, true, scene, cam);
            prepareAircraft(0, 1);
            prepareEnd();
            waitForHelper();
            flip();
            terrainStats(st, true);
            uint32_t busy = (uint32_t)st[0];
            // Each of the core's events over the march, a sample.
            prepareBegin(g_vw, g_vh, BAND_ROWS, true, scene, cam);
            uint32_t ev[6], paths[7], check[4];
            terrainMarchCheck(check);
            // The paths as C's march went them, the check's second.
            terrainPaths(paths);
            Serial.printf("[3d]   the march in C alone: %u columns' tops and %u rows differ from the "
                          "assembly's; %.1fms in assembly, %.1fms in C\n",
                          (unsigned)check[0], (unsigned)check[1], check[2] / 360000.0f, check[3] / 360000.0f);
            terrainEvents(ev);
            prepareBegin(g_vw, g_vh, BAND_ROWS, true, scene, cam);
            Serial.printf("[3d]   the samples: %u hidden by their cell's highest, %u hidden worked out, %u seen "
                          "with no rows, %u filling rows; %u before worked out again, %u blocks tested, %u off "
                          "the grid\n",
                          (unsigned)paths[0], (unsigned)paths[1], (unsigned)(paths[2] - paths[3]),
                          (unsigned)paths[3], (unsigned)paths[4], (unsigned)paths[5], (unsigned)paths[6]);
            prepareAircraft(0, 1);
            prepareEnd();
            flip();
            terrainStats(st, true);
            auto per = [](uint32_t n, uint32_t of) { return of ? (float)n / of : 0.0f; };
            Serial.printf("[3d]   a sample: %.1f cycles, %.1f instructions; its heights all cached: %u samples, "
                          "%.1f cycles, %.1f instructions\n",
                          per(ev[0], ev[2]), per(ev[1], ev[2]), (unsigned)ev[5], per(ev[3], ev[5]),
                          per(ev[4], ev[5]));
            auto each = [&](uint32_t c) { return samples ? c / samples : 0; };
            Serial.printf("[3d] terrain at %.0fs, %s: the march %u samples, %u rows, %u blocks skipped; %u, %u, %u, "
                          "%u cycles each (%.1fms warm); the bands alone %u samples, %u cycles each (%.1fms); the "
                          "march with the other core drawing %u cycles each (%.1fms)\n",
                          t, shot ? shot : "?", (unsigned)samples, (unsigned)rows, (unsigned)skips,
                          (unsigned)each(march[0]),
                          (unsigned)each(march[1]), (unsigned)each(march[2]), (unsigned)each(march[3]),
                          march[3] / 360000.0f, (unsigned)bandSamples,
                          (unsigned)(bandSamples ? bandCycles / bandSamples : 0), bandCycles / 360000.0f,
                          (unsigned)each(busy), busy / 360000.0f);
        };
        for (float t : {15.0f, 40.0f, 60.0f, 80.0f, 100.0f}) {
            if (!ok || grid == nullptr) {
                break;
            }
            for (View view : {View::CHASE, View::CINEMA}) {
                Scene scene = sceneAt(t, TimeOfDay::DAY);
                scene.terrain = true;
                scene.terrainGrid = grid;
                scene.groundUpM = g_airportFieldM;
                const char *shot = "chase";
                Camera cam = cameraAt(view, t, scene.followed.pose, &shot);
                measure(t, scene, cam, shot);
                if (t == 80.0f && view == View::CHASE) {
                    // And skimping all it can, as the governor has it when
                    // frames run long: four rows a sample.
                    Scene skimped = scene;
                    skimped.terrainEffort = 1;
                    measure(t, skimped, cam, "chase, skimping");
                }
            }
        }
        // And the cruise at FL350 off Arran - the round earth's cover - its
        // chase, wide and from below.
        g_departing = false;
        g_busy = false;
        setAirfield(nullptr, 0, nullptr, 0);
        if (ok && enrouteSetup() && coverAt(CRUISE_LAT, CRUISE_LON)) {
            const groundpack::TerrainGrid *was = groundpack::terrainGrid();
            uint32_t from = millis();
            while (groundpack::terrainGrid() == was && millis() - from < 8000) {
                groundpack::want(CRUISE_LAT, CRUISE_LON, 45, 35000 * 0.3048f);
                delay(100);
            }
            const groundpack::TerrainGrid *high = groundpack::terrainGrid();
            for (float t : {3.0f, 14.0f, 25.0f}) {
                if (high == nullptr) {
                    break;
                }
                Scene scene;
                Camera cam;
                const char *shot = nullptr;
                enrouteAt(t, TimeOfDay::DAY, scene, cam, &shot);
                scene.terrain = true;
                scene.terrainGrid = high;
                measure(t, scene, cam, shot);
                if (t == 14.0f) {
                    Scene skimped = scene;
                    skimped.terrainEffort = 1;
                    measure(t, skimped, cam, "wide, skimping");
                }
            }
            enrouteEnd();
        }
        g_cover = false;
        g_departing = false;
        g_busy = false;
        setAirfield(nullptr, 0, nullptr, 0);
        snprintf(g_runwayName, sizeof g_runwayName, "09");
        setView(W, H);
        release();
        g_repaint();
        break;
    }
    case 'f': {
        // What the land's cover costs - see scene3d's GroundCover: the first
        // minute of Heathrow's landing by day as '0' has it, then the first
        // half minute of the cruise as 'h' has it - each with the cover in
        // assembly and PIE, with it in C, and without it. The frames compared,
        // C against assembly, first of all, while the chip's own memory is
        // still in large enough pieces for both cores' bands - the radar's
        // polls, going on all the while, break it up - and the pipeline's
        // bands then held from the first run to the last.
        auto heathrow = [] {
            if (!useAirport("LHR", "27L", "27R")) {
                return false;
            }
            Vec3 at = landingPose(0).at;
            return coverAt(g_airportLat0 + at.y / 110574.0,
                           g_airportLon0 + at.x / (111320.0 * cos(g_airportLat0 * M_PI / 180)));
        };
        auto leaveHeathrow = [] {
            g_busy = false;
            setAirfield(nullptr, 0, nullptr, 0);
            snprintf(g_runwayName, sizeof g_runwayName, "09");
        };
        auto compare = [](const char *where, float t, const Scene &scene, const Camera &cam) {
            Serial.printf("[3d] %s at %.0fs with the cover: %d pixels differ between C and assembly\n", where, t,
                          coverMismatches(scene, cam));
            uint32_t micro[4];
            coverMicro(960, 544, scene, cam, micro);
            Serial.printf("[3d]   a row's places on the grid %u cycles, a point's %u, storing a row's %u; every "
                          "other row's %u\n",
                          (unsigned)micro[0], (unsigned)micro[1], (unsigned)micro[2], (unsigned)micro[3]);
        };
        g_cover = true;
        if (heathrow() && ready()) {
            for (float t : {5.0f, 20.0f}) {
                Scene scene = sceneAt(t, TimeOfDay::DAY);
                const char *shot = nullptr;
                compare("Heathrow", t, scene, cameraAt(View::CINEMA, t, scene.followed.pose, &shot));
            }
        }
        leaveHeathrow();
        buildModels();
        if (enrouteSetup() && coverAt(CRUISE_LAT, CRUISE_LON) && ready()) {
            for (float t : {3.0f, 14.0f, 25.0f}) {
                Scene scene;
                Camera cam;
                const char *shot = nullptr;
                enrouteAt(t, TimeOfDay::DAY, scene, cam, &shot);
                compare("the cruise", t, scene, cam);
            }
        }
        enrouteEnd();
        release();
        if (heathrow()) {
            for (int run = 0; run < 3; run++) {
                g_cover = run < 2;
                playPipelined(TimeOfDay::DAY, 960, 544, 0, 60, 16, run != 1);
            }
        }
        leaveHeathrow();
        buildModels();
        if (enrouteSetup() && coverAt(CRUISE_LAT, CRUISE_LON)) {
            for (int run = 0; run < 3; run++) {
                g_cover = run < 2;
                playPipelined(TimeOfDay::DAY, 960, 544, 0, 30, 16, run != 1);
            }
        }
        g_cover = false;
        enrouteEnd();
        release();
        setSimd(true);
        g_repaint();
        break;
    }
    }
}
}  // namespace

#endif
