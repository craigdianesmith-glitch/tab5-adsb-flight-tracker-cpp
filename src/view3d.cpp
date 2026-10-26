#include "view3d.h"

#include <M5Unified.h>
#include <esp_freertos_hooks.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>
#include <time.h>

#include <algorithm>
#include <atomic>

#include "ground_pack.h"
#include "aircraft_db.h"
#include "airfield_pack.h"
#include "airports.h"
#include "clock_zone.h"
#include "engines3d.h"
#include "ground_follow.h"
#include "ground_route.h"
#include "coast.h"
#include "filter.h"
#include "labels3d.h"
#include "navaids.h"
#include "psram_alloc.h"
#include "runways.h"
#include "scene3d.h"
#include "scenery.h"
#include "screen.h"
#include "settings.h"
#include "sound.h"
#include "telemetry.h"

namespace view3d {
namespace {

namespace s3 = scene3d;
using s3::Vec3;

constexpr float DEG = (float)M_PI / 180;

Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(Vec3 a, float k) { return {a.x * k, a.y * k, a.z * k}; }
float length(Vec3 a) { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }

// --- where things go on the screen ------------------------------------------

// The picture, 1008x560, 40 in from the left and 16 down - its width a whole
// number of 16 pixels, for the PIE's rows; the radar's buttons, the same size
// and in the same column down the right as on the radar, so that Back is
// where it always is; and the flight's details under the picture.
constexpr int VIEW_X = 40, VIEW_Y = 16, VIEW_W = 1008, VIEW_H = 560;
constexpr int BTN_W = 150, BTN_H = 50, BTN_X = 1280 - 16 - BTN_W;
enum Button : uint8_t {
    B_BACK, B_CINEMA, B_CHASE, B_ORBIT, B_TOWER, B_PAST, B_ZOOM_IN, B_ZOOM_OUT, B_GROUND, B_SOUND, BUTTONS
};
// Back; the cameras; how close; the view's own, its land and its sound -
// grouped as the radar groups its own.
constexpr int BTN_Y[BUTTONS] = {8, 84, 144, 204, 264, 324, 400, 460, 536, 596};
const char *const BTN_NAME[BUTTONS] = {"Back",    "CINEMA",   "CHASE",  "ORBIT", "TOWER", "FLY PAST",
                                       "ZOOM IN", "ZOOM OUT", "GROUND", "SOUND"};
constexpr int STRIP_Y = VIEW_Y + VIEW_H + 16, STRIP_H = 720 - 8 - STRIP_Y;

// The radar's colours.
uint16_t colorBg, colorText, colorMuted, colorFaint, colorBtnBg, colorDisabled, colorNote;

void colors() {
    colorBg = M5.Display.color565(0x04, 0x12, 0x0A);
    colorText = M5.Display.color565(0xCF, 0xFF, 0xE2);
    colorMuted = M5.Display.color565(0x7C, 0xD9, 0xA0);
    colorFaint = M5.Display.color565(0x5A, 0x9E, 0x78);
    colorBtnBg = M5.Display.color565(0x14, 0x3A, 0x28);
    colorDisabled = M5.Display.color565(0x4A, 0x55, 0x50);
    colorNote = M5.Display.color565(0xFF, 0xB3, 0x2E);  // the radar's alert amber: something to put right
}

// --- drawing it --------------------------------------------------------------

// Two colour buffers and a depth buffer, each a band of the picture, in the
// chip's own memory: the helper draws a band into one while the PPA sends
// the last from the other to the panel. Taken while the view is up, and given
// back when it goes - the WiFi and TLS live on the same memory. Bands of 16
// rows, 31.5KB each; or, with the memory too broken up for three blocks that
// big - as it can be, the view opened a second time - fewer rows: a frame
// pushed in more pieces, a little slower, rather than no view at all.
void *g_pipe[3] = {};
int g_rows = 16;
size_t pipeBytes(int rows) { return ((size_t)VIEW_W * rows * 2 + 63) & ~(size_t)63; }

TaskHandle_t g_helper = nullptr;
SemaphoreHandle_t g_done = nullptr;
bool g_open = false, g_drawing = false, g_prepared = false;

// The labels for the frame being drawn, stamped into its bands as each is.
labels3d::Labeller g_labeller;
labels3d::Placed g_placed[labels3d::MOST];
int g_placedCount = 0;
uint16_t g_ink = 0xFFFF, g_dim = 0;

// Where the drawing core has got to, for busy().
enum class Step : uint8_t { IDLE, DRAWING, LABELLING, WAITING, PUSHING };
const char *const STEP_NAMES[] = {"idle", "drawing", "labelling", "waiting for the push of the one before", "pushing"};
volatile Step g_step = Step::IDLE;
volatile int g_stepBand = 0;
// How long the last frame took to draw, for the log - and, summed over
// frames until the log takes them, each stage of it (see s3::drawBandInto())
// and the waits for the PPA to finish the band before.
volatile uint32_t g_drawUs = 0;
uint32_t g_drawStageUs[6] = {}, g_pushWaitUs = 0;

#ifdef BENCH3D
int g_benchSun = 0;  // see benchSun()
#endif

// The loop's share of the drawing. Once it has the next frame prepared, as it
// often has well before the helper has drawn this one - an airfield seen from
// the ground, its buildings and the aircraft across the screen, takes longer
// to draw than to prepare - it draws bands of this one too: into colour
// buffers and a depth buffer of its own, each band then handed to the helper
// to push, the PPA having the one owner. The helper takes its bands from the
// top - the sky's, quick to draw, so it is soon back to push the loop's - and
// the loop from the bottom, where the airfield and the aircraft are, until
// they meet.
enum : uint8_t { BAND_FREE, BAND_DRAWING, BAND_READY, BAND_PUSHING };
struct Shared {
    void *color = nullptr;
    int band = 0;
    std::atomic<uint8_t> state{BAND_FREE};
};
Shared g_share[2];
void *g_shareDepth = nullptr;
// All six of them in the one piece of memory, where there was one to be
// had - see open() - or nullptr, each its own.
void *g_arena = nullptr;

// The bands and the loop's buffers given back.
void releaseBands() {
    if (g_arena != nullptr) {
        heap_caps_free(g_arena);
        g_arena = nullptr;
    } else {
        for (void *&p : g_pipe) {
            heap_caps_free(p);
        }
        heap_caps_free(g_shareDepth);
        for (Shared &sh : g_share) {
            heap_caps_free(sh.color);
        }
    }
    for (void *&p : g_pipe) {
        p = nullptr;
    }
    g_shareDepth = nullptr;
    for (Shared &sh : g_share) {
        sh.color = nullptr;
    }
}
// The bands still to take, packed so the two cores never take the same one:
// the next from the top in the low 16 bits, one past the next from the bottom
// in the high.
std::atomic<uint32_t> g_bands{0};
uint32_t bandsFrom(int top, int bottom) { return (uint32_t)top | (uint32_t)bottom << 16; }
// The band at the top still to take, for the helper - -1 if none is.
int bandFromTop() {
    uint32_t v = g_bands.load();
    for (;;) {
        int top = (int)(v & 0xFFFF), bottom = (int)(v >> 16);
        if (top >= bottom) {
            return -1;
        }
        if (g_bands.compare_exchange_weak(v, bandsFrom(top + 1, bottom))) {
            return top;
        }
    }
}
// And at the bottom, for the loop.
int bandFromBottom() {
    uint32_t v = g_bands.load();
    for (;;) {
        int top = (int)(v & 0xFFFF), bottom = (int)(v >> 16);
        if (top >= bottom) {
            return -1;
        }
        if (g_bands.compare_exchange_weak(v, bandsFrom(top, bottom - 1))) {
            return bottom - 1;
        }
    }
}
bool bandsLeft() {
    uint32_t v = g_bands.load();
    return (v & 0xFFFF) < (v >> 16);
}
// Where the bands to share end: where the last band is short, not it or the
// one before - it is pushed as a whole band's worth with the rows of the one
// before in front of it, from the helper's own buffer - which the helper
// draws after the rest.
int g_bandLimit = 0;
// How many bands the loop drew, and the stages of its drawing, for the log.
volatile uint32_t g_loopBands = 0;
uint32_t g_loopStageUs[6] = {};

// On the helper: every band of the frame flipped to, each pushed while the
// next is drawn - its own, and those the loop draws.
void drawFrame() {
    uint16_t *color[2] = {(uint16_t *)g_pipe[0], (uint16_t *)g_pipe[1]};
    uint16_t *depth = (uint16_t *)g_pipe[2];
    int bands = s3::bandCount();
    bool pushing = false;
    int sharing = -1;   // which of the loop's buffers the push in flight is from, if one is
    int mine = 0;       // which of its own buffers the next band of its own goes into
    uint32_t start = micros(), busyFrom = start;
    // The push in flight seen through - the loop's buffer it was from given
    // back - false if the PPA never finished it.
    auto pushed = [&]() {
        g_step = Step::WAITING;
        uint32_t w0 = micros();
        if (pushing && !screen::pushBandWait()) {
            return false;
        }
        g_pushWaitUs += micros() - w0;
        pushing = false;
        if (sharing >= 0) {
            g_share[sharing].state.store(BAND_FREE);
            sharing = -1;
        }
        return true;
    };
    // The frame given up - the PPA stuck - with nothing left for the loop to
    // take or to wait on.
    int tail = g_bandLimit;  // the next of those only it draws, after the rest
    auto giveUp = [&]() {
        g_bands.store(0);
        tail = bands;
        for (Shared &s : g_share) {
            s.state.store(BAND_FREE);
        }
        g_step = Step::IDLE;
    };
    for (;;) {
        // A band the loop has drawn: pushed first, so its buffer is back for
        // the next.
        int ready = -1;
        for (int k = 0; k < 2 && ready < 0; k++) {
            if (g_share[k].state.load() == BAND_READY) {
                ready = k;
            }
        }
        if (ready >= 0) {
            if (!pushed()) {
                giveUp();
                return;
            }
            Shared &s = g_share[ready];
            s.state.store(BAND_PUSHING);
            int y = s.band * g_rows;
            g_step = Step::PUSHING;
            pushing = screen::pushBandStart((uint16_t *)s.color, VIEW_X, VIEW_Y + y, VIEW_W, std::min(g_rows, VIEW_H - y));
            if (!pushing) {
                giveUp();
                return;
            }
            sharing = ready;
            continue;
        }
        int b = bandFromTop();
        if (b < 0 && !bandsLeft() && tail < bands) {
            b = tail++;
        }
        if (b < 0) {
            // None left to take: the loop's still being drawn waited for.
            bool waiting = false;
            for (Shared &s : g_share) {
                uint8_t state = s.state.load();
                waiting = waiting || state == BAND_DRAWING || state == BAND_READY;
            }
            if (!waiting) {
                break;
            }
            continue;
        }
        uint16_t *c = color[mine];
        int y = b * g_rows, rows = std::min(g_rows, VIEW_H - y);
        g_stepBand = b;
        g_step = Step::DRAWING;
        s3::drawBandInto(b, c, depth, g_drawStageUs);
        g_step = Step::LABELLING;
        labels3d::stamp(c, y, rows, VIEW_W, g_placed, g_placedCount, g_ink, g_dim);
        if (!pushed()) {
            giveUp();
            return;
        }
        // The last band, short of the others where they don't divide the
        // view's height, pushed as a whole band's worth - the rows of the one
        // before it below its own - because the PPA has been seen never to
        // finish a band only a few rows high: 960 by 4, on 17 Oct, the frame
        // stopped for good. The band before is this core's - the loop never
        // takes it - in its other buffer, and its push is done with by now.
        uint16_t *push = c;
        int pushY = y, pushRows = rows;
        if (rows < g_rows && b > 0) {
            uint16_t *before = color[mine ^ 1];
            int again = g_rows - rows;
            memmove(before, before + (size_t)rows * VIEW_W, (size_t)again * VIEW_W * 2);
            memcpy(before + (size_t)again * VIEW_W, c, (size_t)rows * VIEW_W * 2);
            push = before;
            pushY = y - again;
            pushRows = g_rows;
        }
        g_step = Step::PUSHING;
        pushing = screen::pushBandStart(push, VIEW_X, VIEW_Y + pushY, VIEW_W, pushRows);
        if (!pushing) {
            giveUp();
            return;
        }
        mine ^= 1;
        // Never long without a pause, an eighth of the time back: this core's
        // idle task has to run within the task watchdog's five seconds or
        // the board is reset, and the poll on this core has to get on - and
        // a frame can take seconds to draw: the camera right up against an
        // aircraft, its every triangle across the screen.
        uint32_t busy = micros() - busyFrom;
        if (busy > 25000) {
            vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(1, busy / 8000)));
            busyFrom = micros();
        }
    }
    if (!pushed()) {
        giveUp();
        return;
    }
    g_drawUs = micros() - start;
    g_step = Step::IDLE;
}

// On the loop, the next frame prepared: bands of the one being drawn, while
// there are any to take and a buffer of its own free to take them - a buffer
// waits for the helper to push it, which it does as soon as it is done with
// the band in hand.
void helpDraw() {
    if (g_shareDepth == nullptr) {
        return;
    }
    uint32_t since = millis();
    for (;;) {
        int k = -1;
        for (int i = 0; i < 2 && k < 0; i++) {
            if (g_share[i].color != nullptr && g_share[i].state.load() == BAND_FREE) {
                k = i;
            }
        }
        if (k < 0) {
            // Both handed over, the helper not yet at them: waited for - but
            // not for ever, should the PPA have stopped with them.
            if (!bandsLeft() || millis() - since > 200) {
                return;
            }
            continue;
        }
        // Marked as being drawn before the band is taken, so the helper,
        // finding none left, waits for it.
        Shared &s = g_share[k];
        s.state.store(BAND_DRAWING);
        int b = bandFromBottom();
        if (b < 0) {
            s.state.store(BAND_FREE);
            return;
        }
        int y = b * g_rows, rows = std::min(g_rows, VIEW_H - y);
        s3::drawBandInto(b, (uint16_t *)s.color, (uint16_t *)g_shareDepth, g_loopStageUs);
        labels3d::stamp((uint16_t *)s.color, y, rows, VIEW_W, g_placed, g_placedCount, g_ink, g_dim);
        s.band = b;
        s.state.store(BAND_READY);
        g_loopBands = g_loopBands + 1;  // only this task writes it
        since = millis();
    }
}

// When this core's idle task last ran - see helperTask() - and the longest
// it has gone without, for the log.
volatile uint32_t g_idle0Ms = 0, g_idle0GapMs = 0;
bool idle0Ran() {
    g_idle0Ms = millis();
    return true;
}

// What the helper is woken for: a frame to draw, and the next one's
// buildings to work out - see s3::prepareSolids().
constexpr uint32_t WAKE_DRAW = 1, WAKE_SOLIDS = 2;
// How many frames' buildings it worked out, for the log.
volatile uint32_t g_solidsAside = 0;

void helperTask(void *) {
    for (;;) {
        uint32_t woken = 0;
        xTaskNotifyWait(0, UINT32_MAX, &woken, portMAX_DELAY);
        if (woken & WAKE_DRAW) {
            drawFrame();
            xSemaphoreGive(g_done);
        }
        // Then the next frame's buildings, being prepared on the other core,
        // if it hasn't come to them yet: at a big airport as long as the
        // drawing, and this core idle otherwise. At the poll's priority, so
        // they don't keep it from the network; the other core does them
        // itself if they're still waiting when it needs them.
        vTaskPrioritySet(nullptr, 1);
        if (s3::prepareSolids()) {
            g_solidsAside = g_solidsAside + 1;  // only this task writes it
        }
        vTaskPrioritySet(nullptr, 3);
        // The core's idle task kept in: frame after frame, with the poll on
        // this core busy too, it can go without for the task watchdog's five
        // seconds, which resets the board - the more so with the PPA given
        // up and every band pushed by this core. So with a fifth of a second
        // gone by since it last ran, a moment off - longer if that hasn't
        // let it in, the poll having taken it.
        uint32_t starved = millis() - g_idle0Ms;
        if (starved > g_idle0GapMs) {
            g_idle0GapMs = starved;
        }
        if (starved > 200) {
            vTaskDelay(pdMS_TO_TICKS(starved > 1000 ? 10 : 2));
        }
    }
}

// The frame being drawn finished - this core drawing bands of it too, while
// there are any - false if it hasn't been in a second.
bool finishDrawing() {
    if (g_drawing) {
        helpDraw();
    }
    if (g_drawing && xSemaphoreTake(g_done, pdMS_TO_TICKS(1000)) == pdTRUE) {
        g_drawing = false;
    }
    return !g_drawing;
}

// --- where it is ---------------------------------------------------------------

// Metres east, north and up of a point on the ground, through WGS84's
// earth-centred frame - the earth's curve and all.
struct Enu {
    double x0, y0, z0, sp, cp, sl, cl, lon0;
    float north0, up0, fsp, fcp;  // the origin's, for ground(): see there
    static void ecef(double lat, double lon, double h, double &x, double &y, double &z) {
        constexpr double A = 6378137.0, E2 = 6.69437999014e-3;
        double p = lat * M_PI / 180, l = lon * M_PI / 180, n = A / sqrt(1 - E2 * sin(p) * sin(p));
        x = (n + h) * cos(p) * cos(l);
        y = (n + h) * cos(p) * sin(l);
        z = (n * (1 - E2) + h) * sin(p);
    }
    void at(double lat, double lon) {
        ecef(lat, lon, 0, x0, y0, z0);
        sp = sin(lat * M_PI / 180), cp = cos(lat * M_PI / 180);
        sl = sin(lon * M_PI / 180), cl = cos(lon * M_PI / 180);
        lon0 = lon;
        constexpr double A = 6378137.0, E2 = 6.69437999014e-3;
        double n0 = A / sqrt(1 - E2 * sp * sp);
        north0 = (float)(n0 * cp);
        up0 = (float)(n0 * (1 - E2) * sp);
        fsp = (float)sp;
        fcp = (float)cp;
    }
    Vec3 of(double lat, double lon, double h) const {
        double x, y, z;
        ecef(lat, lon, h, x, y, z);
        double dx = x - x0, dy = y - y0, dz = z - z0;
        return {(float)(-sl * dx + cl * dy), (float)(-sp * cl * dx - sp * sl * dy + cp * dz),
                (float)(cp * cl * dx + cp * sl * dy + sp * dz)};
    }
    // A point on the ground, as of() has it to within a metre or two out to
    // 650km - in float, which the P4 does in its FPU: the country's tens of
    // thousands of points through of()'s double, done in software, took a
    // second. In the frame turned about the pole to the origin's longitude,
    // where east is y, and north and up are x and z turned by its latitude.
    Vec3 ground(double lat, double lon) const {
        constexpr float A = 6378137.0f, E2 = 6.69437999014e-3f;
        float p = (float)(lat * (M_PI / 180)), l = (float)((lon - lon0) * (M_PI / 180));
        float s = sinf(p), c = cosf(p), n = A / sqrtf(1 - E2 * s * s);
        float x = n * c * cosf(l) - north0, y = n * c * sinf(l), z = n * (1 - E2) * s - up0;
        return {y, -fsp * x + fcp * z, fcp * x + fsp * z};
    }
};

// What is round it: an airfield, low near one - laid out flat, metres east
// and north of the middle of its runways and up from them - or, high up, the
// country below, round, through the earth's frame from a point under it.
enum class Where : uint8_t { NOWHERE, AIRFIELD, COUNTRY };
Where g_where = Where::NOWHERE;
// Counted up each time the view's frame moves - a new airfield, the country's
// middle moved on - for what was placed in the old one to be placed again.
uint32_t g_frameGen = 0;
char g_airport[5] = "";
double g_lat0 = 0, g_lon0 = 0, g_cosLat0 = 1;
float g_fieldFt = 0;
char g_endName[24][4];  // the airfield's runway ends, as setAirfield() has them, two to a runway
Vec3 g_endAt[24];       // and where each is
float g_runwayWidth[12];  // each runway's width, metres
int g_runways = 0;
Enu g_enu;
double g_enuLat = 0, g_enuLon = 0;
uint32_t g_whereMs = 0;

Vec3 local(double lat, double lon) {
    return {(float)((lon - g_lon0) * 111320.0 * g_cosLat0), (float)((lat - g_lat0) * 110574.0), 0};
}

// The airfield's taxiways as a network, from its real layout - empty without
// one - and the followed aircraft taxiing on it, as its reports have it -
// see ground_follow.h - while on the ground and slower than a runway's roll.
ground::Network g_network;
ground::Follower g_follower;
// Its speed as the follower has it, knots, while it does; -1 otherwise.
int g_taxiKt = -1;
// What the follower last said it was doing, in the log.
char g_taxiSaid[40] = "";
// The others standing on the ground, for the stands they take.
airfield::List<ground::Vec2> g_standing;
// The airfield's real layout, as loaded; empty without one.
airfield::Layout g_layout;
#ifdef BENCH3D
// The airfield's stands, and points on its taxiways with their way, as
// latitude and longitude, for the bench's made-up traffic - see
// benchStands(). Taken as the airfield is built, read by the poll.
constexpr int BENCH_STANDS = 400, BENCH_WAYS = 16;
double g_benchStandLat[BENCH_STANDS], g_benchStandLon[BENCH_STANDS];
double g_benchWayLat[BENCH_WAYS], g_benchWayLon[BENCH_WAYS];
float g_benchWayTrack[BENCH_WAYS];
std::atomic<int> g_benchStands{0}, g_benchWays{0};
#endif

// The coasts and lakes' shores about an airfield, laid out flat as it is -
// and not the borders, which aren't there to be seen from a few thousand
// feet.
void sinkShore(void *context, OutlineKind kind, const double *latLon, int points) {
    if (kind != OutlineKind::COAST && kind != OutlineKind::LAKE) {
        return;
    }
    auto *out = (std::vector<s3::GroundLine, PsramAllocator<s3::GroundLine>> *)context;
    Vec3 a = local(latLon[0], latLon[1]);
    for (int i = 0; i + 1 < points; i++) {
        Vec3 b = local(latLon[2 * i + 2], latLon[2 * i + 3]);
        out->push_back({a, b, (s3::GroundKind)kind});
        a = b;
    }
}

// The towns' outlines and the rivers, each point put by `place` - in the
// earth's frame high up, the airfield's flat one low down.
struct SceneryOut {
    Vec3 (*place)(double lat, double lon);
    std::vector<s3::GroundLine, PsramAllocator<s3::GroundLine>> *lines;
    std::vector<s3::RiverLine, PsramAllocator<s3::RiverLine>> *rivers;
};

void sinkScenery(void *context, SceneryKind kind, const double *latLon, int points) {
    SceneryOut &out = *(SceneryOut *)context;
    Vec3 a = out.place(latLon[0], latLon[1]);
    for (int i = 0; i + 1 < points; i++) {
        Vec3 b = out.place(latLon[2 * i + 2], latLon[2 * i + 3]);
        if (kind == SceneryKind::TOWN) {
            out.lines->push_back({a, b, s3::GroundKind::TOWN});
        } else {
            out.rivers->push_back({a, b, riverWidth(kind)});
        }
        a = b;
    }
}

// Why the airfield the view is at was drawn from its runways alone - the
// card's airfield pack missing, or this airport not in it - or NOTHING, its
// real layout drawn.
airfield::Missing g_missing = airfield::Missing::NOTHING;

// That said on the strip, under the picture.
const char *basicWhy() {
    static char text[80];
    using M = airfield::Missing;
    switch (g_missing) {
    case M::NO_CARD: return "Basic view - no SD card in for the airfield pack, airfields.bin";
    case M::NO_PACK: return "Basic view - no airfields.bin in the SD card's overhead folder";
    case M::OTHER_VERSION: return "Basic view - the card's airfields.bin is for another version";
    case M::NOT_IN_PACK: snprintf(text, sizeof text, "Basic view - %s isn't in the card's airfields.bin", g_airport); break;
    case M::DAMAGED: snprintf(text, sizeof text, "Basic view - %s's layout in airfields.bin is damaged", g_airport); break;
    default: return "";
    }
    return text;
}

// Why the land is flat and one colour - the card's ground pack missing, or
// refused, or with no land of it anywhere round here - or nullptr, the land
// drawn from it: said on the strip as the airfield pack's is, and what to do.
const char *flatWhy() {
    using S = groundpack::Status;
    switch (groundpack::status()) {
    case S::NO_CARD: return "Flat land - no SD card in for the ground pack, ground.bin";
    case S::NO_PACK: return "Flat land - no ground.bin in the SD card's overhead folder";
    case S::OTHER_VERSION: return "Flat land - the card's ground.bin is for another version";
    case S::TOO_BIG: return "Flat land - the card's ground.bin is too big for the Tab5: build a smaller one";
    case S::DAMAGED: return "Flat land - the card's ground.bin is damaged: copy it to the card again";
    case S::NO_ROOM: return "Flat land - no room in memory for the card's ground.bin: restart the Tab5";
    case S::READY: return groundpack::offPack() ? "Flat land - here isn't in the card's ground.bin" : nullptr;
    default: return nullptr;  // not looked for yet, or being read
    }
}

bool buildAirfield(const char *code) {
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
    if (open == 0) {
        return false;
    }
    uint32_t t0 = millis();
    g_lat0 = lat0 / open;
    g_lon0 = lon0 / open;
    g_cosLat0 = cos(g_lat0 * M_PI / 180);
    g_fieldFt = runways[0].elevationFt;
    s3::RunwaySpec specs[12];
    int n = 0;
    for (int i = 0; i < count && n < 12; i++) {
        if (runways[i].closed) {
            continue;
        }
        s3::RunwaySpec &spec = specs[n];
        for (int e = 0; e < 2; e++) {
            spec.end[e] = local(runways[i].end[e].lat, runways[i].end[e].lon);
            snprintf(spec.name[e], sizeof spec.name[e], "%s", runways[i].end[e].ident);
            snprintf(g_endName[n * 2 + e], sizeof g_endName[0], "%s", runways[i].end[e].ident);
            g_endAt[n * 2 + e] = spec.end[e];
        }
        spec.width = runways[i].widthFt * 0.3048f;
        g_runwayWidth[n] = spec.width;
        n++;
    }
    Vec3 beacons[4];
    int beaconCount = 0;
    for (const Navaid *nav : navaidsWithin(g_lat0, g_lon0, 2)) {
        if (beaconCount < 4 && nav->type != NavaidType::TACAN && nav->type != NavaidType::DME &&
            nav->type != NavaidType::NDB_DME) {
            beacons[beaconCount++] = local(nav->lat, nav->lon);
        }
    }
    // Its real taxiways, stands and buildings, if the card has them - in the
    // frame here, should the layout's (0, 0) be anywhere else.
    airfield::Layout &layout = g_layout;
    bool real = airfield::load(code, layout, &g_missing);
    if (real) {
        Vec3 offset = local(layout.lat0, layout.lon0);
        if (fabsf(offset.x) > 0.01f || fabsf(offset.y) > 0.01f) {
            layout.moveBy(offset.x, offset.y);
        }
        g_network.build(layout);
    } else {
        g_network = ground::Network();
    }
#ifdef BENCH3D
    g_benchStands = 0;
    g_benchWays = 0;
    if (real) {
        auto latLon = [](float x, float y, double &lat, double &lon) {
            lat = g_lat0 + y / 110574.0;
            lon = g_lon0 + x / (111320.0 * g_cosLat0);
        };
        int n = 0;
        for (const airfield::Stand &st : layout.stands) {
            if (n < BENCH_STANDS) {
                latLon(st.at.x, st.at.y, g_benchStandLat[n], g_benchStandLon[n]);
                n++;
            }
        }
        // A taxiway's middle every so many of them, the way along it.
        int w = 0, taxiways = 0;
        for (const airfield::Way &way : layout.ways) {
            taxiways += way.kind == airfield::WayKind::TAXIWAY && way.between.size() >= 2;
        }
        int every = std::max(1, taxiways / BENCH_WAYS), k = 0;
        for (const airfield::Way &way : layout.ways) {
            if (way.kind != airfield::WayKind::TAXIWAY || way.between.size() < 2 || k++ % every != 0 || w >= BENCH_WAYS) {
                continue;
            }
            size_t i = way.between.size() / 2 - (way.between.size() % 2 == 0 ? 1 : 0);
            i = std::min(i, way.between.size() - 2);
            const airfield::Point &p = way.between[i], &q = way.between[i + 1];
            latLon(p.x, p.y, g_benchWayLat[w], g_benchWayLon[w]);
            g_benchWayTrack[w] = fmodf(atan2f(q.x - p.x, q.y - p.y) * 180 / (float)M_PI + 360, 360);
            w++;
        }
        g_benchStands = n;
        g_benchWays = w;
    }
#endif
    g_follower.end();
    g_taxiKt = -1;
    // The sea and the lochs about it, as far off as is seen from a few
    // thousand feet - not grass to the horizon: Prestwick, on the Firth of
    // Clyde, was an airfield inland. Natural Earth's coasts, good to a few
    // hundred metres - near enough at that distance.
    std::vector<s3::GroundLine, PsramAllocator<s3::GroundLine>> shores;
    coastLines(g_lat0, g_lon0, 60, 60, 0, sinkShore, &shores);
    // And the towns about it, and the rivers.
    std::vector<s3::RiverLine, PsramAllocator<s3::RiverLine>> rivers;
    SceneryOut scenery = {local, &shores, &rivers};
    sceneryLines(g_lat0, g_lon0, 60, sinkScenery, &scenery);
    s3::setGroundLines(shores.data(), (int)shores.size());
    s3::setRivers(rivers.data(), (int)rivers.size());
    s3::setFarRunways(nullptr, 0);
    s3::setAirfield(specs, n, beacons, beaconCount, real ? &layout : nullptr);
    s3::lightTowns(4000, 120, 6000, false);
    g_runways = n;
    snprintf(g_airport, sizeof g_airport, "%s", code);
    g_where = Where::AIRFIELD;
    g_frameGen++;
    if (real) {
        Serial.printf("[3d] at %s: %d runways, %d beacons; its layout: %u ways, %u stands, %u buildings; the tower's "
                      "camera %.0fm over its cab; in %lums\n",
                      code, n, beaconCount, (unsigned)layout.ways.size(), (unsigned)layout.stands.size(),
                      (unsigned)layout.buildings.size(), s3::towerRaised(), (unsigned long)(millis() - t0));
    } else {
        Serial.printf("[3d] at %s: %d runways, %d beacons; %s; in %lums\n", code, n, beaconCount, basicWhy(),
                      (unsigned long)(millis() - t0));
    }
    return true;
}

// Each point through the earth's frame once, and not again as the start of
// the next line.
void sinkLine(void *context, OutlineKind kind, const double *latLon, int points) {
    auto *out = (std::vector<s3::GroundLine, PsramAllocator<s3::GroundLine>> *)context;
    Vec3 a = g_enu.ground(latLon[0], latLon[1]);
    for (int i = 0; i + 1 < points; i++) {
        Vec3 b = g_enu.ground(latLon[2 * i + 2], latLon[2 * i + 3]);
        out->push_back({a, b, (s3::GroundKind)kind});
        a = b;
    }
}

void buildCountry(double lat, double lon) {
    uint32_t t0 = millis();
    g_enu.at(lat, lon);
    g_enuLat = lat;
    g_enuLon = lon;
    std::vector<s3::GroundLine, PsramAllocator<s3::GroundLine>> lines;
    // In full, all of it: the land is told from the sea by which way each
    // coast crosses a row of the screen, and the thinned copies - to 3.5km -
    // cross a narrow loch's or fjord's shores over, which then crosses the
    // wrong way and paints a band of the other across the view (Loch Maree,
    // seen from over the Clyde). Twice the lines to keep; a frame reads only
    // those on the screen.
    coastLines(lat, lon, 520, 520, 0, sinkLine, &lines);
    std::vector<s3::RiverLine, PsramAllocator<s3::RiverLine>> rivers;
    SceneryOut scenery = {[](double la, double lo) { return g_enu.ground(la, lo); }, &lines, &rivers};
    sceneryLines(lat, lon, 520, sinkScenery, &scenery);
    s3::setGroundLines(lines.data(), (int)lines.size());
    s3::setRivers(rivers.data(), (int)rivers.size());
    int count = 0;
    const Runway *all = allRunways(count);
    std::vector<s3::RunwaySpec, PsramAllocator<s3::RunwaySpec>> far;
    double cosLat = cos(lat * M_PI / 180);
    for (int i = 0; i < count; i++) {
        const Runway &r = all[i];
        double dn = (r.end[0].lat - lat) * 111.0, de = (r.end[0].lon - lon) * 111.0 * cosLat;
        if (!r.closed && dn * dn + de * de < 400.0 * 400.0) {
            s3::RunwaySpec spec{};
            for (int e = 0; e < 2; e++) {
                spec.end[e] = g_enu.of(r.end[e].lat, r.end[e].lon, r.elevationFt * 0.3048);
            }
            spec.width = r.widthFt * 0.3048f;
            far.push_back(spec);
        }
    }
    s3::clearAirfield();
    s3::setFarRunways(far.data(), (int)far.size());
    s3::lightTowns(6000, 500, 60000, true);
    g_where = Where::COUNTRY;
    g_frameGen++;
    g_airport[0] = 0;
    g_runways = 0;
    Serial.printf("[3d] high up: %u lines, %u of rivers, %u runways, in %lums\n", (unsigned)lines.size(),
                  (unsigned)rivers.size(), (unsigned)far.size(), (unsigned long)(millis() - t0));
}

float distanceKm(double lat0, double lon0, double lat1, double lon1) {
    double dn = (lat1 - lat0) * 111.2, de = (lon1 - lon0) * 111.2 * cos(lat0 * M_PI / 180);
    return (float)sqrt(dn * dn + de * de);
}

// Once a second: the airfield it is taking off from or landing on, or one it
// is low near - within 10nm and under 5,000ft above it, and staying with it
// until 15nm off or 7,000ft up, so as not to flick between the two - and
// otherwise the country, built again once it is 150km from where it was.
void choose(const Followed &f) {
    uint32_t now = millis();
    if (g_where != Where::NOWHERE && now - g_whereMs < 1000) {
        return;
    }
    g_whereMs = now;
    const FollowEstimate &e = f.now;
    String want;
    if (f.movement != RunwaySim::Movement::NONE && f.runway.airport != nullptr) {
        want = f.runway.airport;
    } else {
        Airport a;
        bool near = g_where == Where::AIRFIELD;
        if (nearestAirport(e.lat, e.lon, a, near ? 15 : 10, true)) {
            int count = 0;
            const Runway *r = runwaysAt(a.code.c_str(), count);
            float above = e.ground ? 0 : e.altFt - (count > 0 ? r[0].elevationFt : 0);
            if (count > 0 && above < (near ? 7000 : 5000)) {
                want = a.code;
            }
        }
    }
    if (want.length() > 0) {
        if ((g_where == Where::AIRFIELD && want == g_airport) || buildAirfield(want.c_str())) {
            return;
        }
    }
    if (g_where != Where::COUNTRY || distanceKm(g_enuLat, g_enuLon, e.lat, e.lon) > 150) {
        buildCountry(e.lat, e.lon);
    }
}

// A report's place in the view's world.
Vec3 place(double lat, double lon, float altFt, bool ground) {
    if (g_where == Where::AIRFIELD) {
        Vec3 p = local(lat, lon);
        p.z = ground ? 0 : std::max(0.0f, (altFt - g_fieldFt) * 0.3048f);
        return p;
    }
    return g_enu.of(lat, lon, altFt * 0.3048);
}

// --- what is there --------------------------------------------------------------

// Each aircraft's look - its type's model, its airline's colours - worked
// out once, the first time it is seen; and where it was last shown, and
// which way it pointed, for one standing still to stay put.
struct Known {
    char hex[8];
    s3::Look look;
    uint32_t usedMs;
    Vec3 at;
    float heading;
    Where where;
    // On the ground at an airfield with real stands: the stand it is parked
    // at (-1 none), worked out from where it was reported then - again only
    // once it is reported elsewhere; and when it was last shown, for its
    // taxiing to be eased from there.
    int16_t stand = -1;
    float standFromX = 0, standFromY = 0;
    uint32_t shownMs = 0;
};
std::vector<Known, PsramAllocator<Known>> g_known;

// Good until the next new aircraft is seen.
// g_known by hex, for one to be found without comparing hundreds of strings:
// open addressing on the 24-bit ICAO address (with a bit for the feed's "~"
// addresses that aren't ICAO ones); each slot g_known's index, -1 empty.
constexpr uint32_t KNOWN_SLOTS = 2048;  // a power of two, over three times the most kept
struct KnownSlot {
    uint32_t key;
    int16_t at;
};
KnownSlot *g_knownSlots = nullptr;  // in PSRAM

uint32_t keyOf(const char *hex) {
    bool other = hex[0] == '~';
    return (uint32_t)strtoul(hex + (other ? 1 : 0), nullptr, 16) | (other ? 0x80000000u : 0);
}

uint32_t slotOf(uint32_t key) { return (key * 2654435761u) >> 21 & (KNOWN_SLOTS - 1); }

void indexKnown() {
    if (g_knownSlots == nullptr) {
        g_knownSlots = (KnownSlot *)heap_caps_malloc(sizeof(KnownSlot) * KNOWN_SLOTS, MALLOC_CAP_SPIRAM);
    }
    for (uint32_t i = 0; i < KNOWN_SLOTS; i++) {
        g_knownSlots[i].at = -1;
    }
    for (size_t k = 0; k < g_known.size(); k++) {
        uint32_t key = keyOf(g_known[k].hex), i = slotOf(key);
        while (g_knownSlots[i].at >= 0) {
            i = (i + 1) & (KNOWN_SLOTS - 1);
        }
        g_knownSlots[i] = {key, (int16_t)k};
    }
}

// The least recently seen let go, down to 300, when there are over 380 - only
// as a poll's traffic is worked out, before anything holds an index into
// g_known; which leaves room for a whole poll of new ones, and the followed,
// under KNOWN_SLOTS / 3 without another.
void pruneKnown() {
    if (g_known.size() > 380) {
        std::sort(g_known.begin(), g_known.end(), [](const Known &a, const Known &b) { return a.usedMs > b.usedMs; });
        g_known.resize(300);
        indexKnown();
    }
}

int knownIndexOf(const String &hex, const String &type, const String &callsign, bool military,
                 const String &category) {
    if (g_knownSlots == nullptr) {
        indexKnown();
    }
    uint32_t key = keyOf(hex.c_str()), i = slotOf(key);
    for (; g_knownSlots[i].at >= 0; i = (i + 1) & (KNOWN_SLOTS - 1)) {
        if (g_knownSlots[i].key == key && strcmp(g_known[g_knownSlots[i].at].hex, hex.c_str()) == 0) {
            g_known[g_knownSlots[i].at].usedMs = millis();
            return g_knownSlots[i].at;
        }
    }
    if (g_known.size() >= KNOWN_SLOTS / 3) {
        pruneKnown();  // can't happen between polls: there are at most 300 traffic and one followed
        return knownIndexOf(hex, type, callsign, military, category);
    }
    Known k{};
    snprintf(k.hex, sizeof k.hex, "%s", hex.c_str());
    k.look = s3::lookFor(type.c_str(), 0x1B2A55, military, category.c_str());
    s3::paintAirline(k.look, callsign.c_str());
    k.usedMs = millis();
    k.where = Where::NOWHERE;
    g_known.push_back(k);
    g_knownSlots[i] = {key, (int16_t)(g_known.size() - 1)};
    return (int)g_known.size() - 1;
}

Known &knownOf(const String &hex, const String &type, const String &callsign, bool military, const String &category) {
    return g_known[knownIndexOf(hex, type, callsign, military, category)];
}

s3::Look lookOf(const String &hex, const String &type, const String &callsign, bool military, const String &category) {
    return knownOf(hex, type, callsign, military, category).look;
}

// Standing still on the ground - under 6 knots, a stopped transponder's
// speed wandering up to 4 or 5 - a report's track means nothing, and its
// position wanders by tens of metres from one to the next:
// so it keeps the heading it had, and the place it had too unless a report
// puts it more than 30m away, which is a move.
constexpr float STILL_KT = 6, STILL_M = 30;
// On a runway: on it, or this many metres past an end - a quarter of it
// either side.
constexpr float RUNWAY_SLACK = 60;
// On a runway this fast, knots, an aircraft is landing or taking off; slower,
// taxiing across or along it.
constexpr int RUNWAY_ROLL_KT = 30;
// Reported this near a stand's stop, it is at that stand.
constexpr float AT_STAND_M = 35;
bool standingStill(bool ground, float gsKt) { return ground && gsKt < STILL_KT; }
// Hovering - a helicopter in the air under 12 knots - its track is the drift's,
// all round the compass: it keeps the heading it had, though not its place,
// the drift being real.
constexpr float HOVER_KT = 12;
bool hovering(bool ground, float gsKt) { return !ground && gsKt < HOVER_KT; }
// A rotorcraft set down: its transponder saying so, or within ROTOR_DOWN_FT
// of the ground under the view - the airfield's, or the sea's high up - and
// all but still. Many never say they are on the ground (see
// followSampleOf(), which puts one by its GNSS height).
constexpr int ROTOR_DOWN_FT = 60, ROTOR_DOWN_KT = 3;
// And one not set down is off the ground, by a hover taxi's few feet at
// least: an estimate, or a pressure altitude, putting it lower - at the
// ground's height, or under it - had its skids sliding along the grass.
constexpr float ROTOR_LOWEST_M = 1.5f;
int groundFt() { return g_where == Where::AIRFIELD ? (int)lroundf(g_fieldFt) : 0; }
bool rotorDown(bool rotorcraft, bool ground, int altFt, int gsKt) {
    return ground || (rotorcraft && altFt - groundFt() < ROTOR_DOWN_FT && gsKt < ROTOR_DOWN_KT);
}

// The one followed's estimate - a helicopter set down, its transponder
// saying so or not, on the ground.
FollowEstimate estimateOf(const Followed &f) {
    FollowEstimate e = f.now;
    if (!e.ground && rotorDown(isRotorcraft(f.type, f.category), false, e.altFt, e.gsKt)) {
        e.ground = true;
        e.altFt = groundFt();
    }
    return e;
}

// The time now on the clock the flight's reports and the traffic's poll are
// timed on - millis() live, the recording's in a replay - see frame(); and,
// run on by it, the view's own seconds: standing still while a replay is.
uint32_t g_nowMs = 0;
bool g_haveNow = false;
float g_seconds = 0;
// Moved on further than this in a frame - or back - it has jumped: a
// replay's minute, or its bar.
constexpr uint32_t JUMP_MS = 5000;
// Showing a replay, and its place and pace: see setReplay().
bool g_inReplay = false;
ReplayBar g_replay;
// Where the traffic's pressure altitudes are put right from.
DayPressureFn g_dayPressure = dayPressureFt;

// What a label says of one, and who it is, for each of the traffic in a
// frame - in two, the frame being prepared's and the one being drawn's.
struct Info {
    char hex[8], callsign[10], height[10], type[6];
    labels3d::Kind kind;
};
constexpr int MOST_TRAFFIC = 300;
std::vector<s3::Aircraft, PsramAllocator<s3::Aircraft>> g_traffic;
std::vector<Info, PsramAllocator<Info>> g_info[2];
int g_preparing = 0;  // which of g_info is the frame being prepared's; the other, the drawn one's

void heightText(char *out, size_t size, const FollowSample &s) {
    if (s.ground) {
        snprintf(out, size, "GND");
    } else if (s.altFt >= 18000) {
        snprintf(out, size, "FL%03d", (int)((s.altFt + 50) / 100));
    } else {
        snprintf(out, size, "%dft", (int)((s.altFt + 50) / 100) * 100);
    }
}

// The traffic as last polled, worked out once a poll - or once the view's
// frame moves - for each frame only to move it on: who each is, its look's
// entry, its report as a sample, where it was reported in the view's frame
// and the way its track runs there, what its label says. Done every frame -
// the feed's words parsed, its look found among hundreds, the round earth's
// maths in double precision, which the P4 does in software - it took 20ms
// a frame high up.
struct Polled {
    FollowSample s;
    Vec3 at;    // where it was reported, in the view's frame
    Vec3 way;   // a metre along its track from there, in the view's frame
    float heading;  // its track, radians anticlockwise from east
    float speed;    // metres a second
    float heightM;
    int16_t known;  // its entry in g_known
    bool moves;     // carried on along its track between polls
    Info info;
};
std::vector<Polled, PsramAllocator<Polled>> g_polled;
uint32_t g_polledMs = 0, g_polledGen = UINT32_MAX;
String g_polledFollowed;

void pollTraffic(const std::vector<Aircraft> &list, const String &followed) {
    pruneKnown();
    g_polled.clear();
    const TrafficFilterSet &filter = trafficFilter();
    for (const Aircraft &ac : list) {
        if (!ac.hasPos || ac.hex == followed || (int)g_polled.size() >= MOST_TRAFFIC) {
            continue;
        }
        // Not an aircraft - a vehicle on the airfield, which has no model:
        // drawn as an airliner's, one was parked at a stand - or one the
        // traffic filter leaves out, as the table and the radar do.
        if (isGroundVehicle(ac.type, ac.category) || (filter.active() && !filter.passes(ac))) {
            continue;
        }
        Polled p{};
        p.s = followSampleOf(ac, 0, g_dayPressure);
        if (!p.s.ground && rotorDown(p.s.rotorcraft, false, p.s.altFt, p.s.gsKt)) {
            p.s.ground = true;
            p.s.altFt = groundFt();
        }
        const FollowSample &s = p.s;
        if (!s.hasAlt) {
            continue;
        }
        // At its height above the sea, as the one followed is shown - not
        // its pressure altitude, a few hundred feet out on most days.
        p.at = place(s.lat, s.lon, aboveSeaFt(s), s.ground);
        if (s.rotorcraft && !s.ground && g_where == Where::AIRFIELD) {
            p.at.z = std::max(p.at.z, ROTOR_LOWEST_M);
        }
        p.heading = (90 - (s.hasTrack ? s.track : 0)) * DEG;
        p.speed = s.gsKt * 0.5144f;
        p.heightM = s.ground ? 0 : g_where == Where::AIRFIELD ? p.at.z : aboveSeaFt(s) * 0.3048f;
        p.moves = (!s.ground || s.gsKt >= STILL_KT) && s.hasGs && s.hasTrack && !s.posStale;
        if (p.moves) {
            // The way it goes, in this frame: to a point a kilometre along its
            // track, placed the same way.
            double lat = s.lat + 1000 * cos(s.track * M_PI / 180) / 111320.0;
            double lon = s.lon + 1000 * sin(s.track * M_PI / 180) / (111320.0 * cos(s.lat * M_PI / 180));
            p.way = (place(lat, lon, aboveSeaFt(s), s.ground) - p.at) * (1.0f / 1000);
        }
        p.known = (int16_t)knownIndexOf(ac.hex, ac.type, ac.callsign, ac.military, ac.category);
        snprintf(p.info.hex, sizeof p.info.hex, "%s", ac.hex.c_str());
        String call = ac.callsign;
        call.trim();
        snprintf(p.info.callsign, sizeof p.info.callsign, "%s", call.length() ? call.c_str() : ac.hex.c_str());
        heightText(p.info.height, sizeof p.info.height, s);
        snprintf(p.info.type, sizeof p.info.type, "%s", ac.type.c_str());
        p.info.kind = !s.ground ? labels3d::Kind::AIRBORNE : s.gsKt > 3 ? labels3d::Kind::MOVING : labels3d::Kind::PARKED;
        g_polled.push_back(p);
    }
}

// Which of the airfield's runways `at` is on - or just off an end of - or -1.
int runwayAt(Vec3 at) {
    for (int r = 0; r < g_runways; r++) {
        Vec3 a = g_endAt[r * 2], b = g_endAt[r * 2 + 1];
        float dx = b.x - a.x, dy = b.y - a.y, len = hypotf(dx, dy);
        if (len < 1) {
            continue;
        }
        float along = ((at.x - a.x) * dx + (at.y - a.y) * dy) / len;
        float across = fabsf((at.x - a.x) * dy - (at.y - a.y) * dx) / len;
        if (along > -RUNWAY_SLACK && along < len + RUNWAY_SLACK && across < g_runwayWidth[r] / 2 + RUNWAY_SLACK / 4) {
            return r;
        }
    }
    return -1;
}

// The traffic as last polled, `polledMs` ago, moved on since along its track
// at its speed - those near enough to matter, and not the one followed. On
// runway `busy` - the one the view's aircraft is landing on or taking off
// from, -1 none - nothing else on the ground: the feed's lag can leave the
// landing before still on it as the next take-off rolls.
void fillTraffic(const std::vector<Aircraft> &list, uint32_t polledMs, const String &followed, Vec3 from, int busy) {
    if (polledMs != g_polledMs || g_frameGen != g_polledGen || followed != g_polledFollowed) {
        pollTraffic(list, followed);
        g_polledMs = polledMs;
        g_polledGen = g_frameGen;
        g_polledFollowed = followed;
    }
    float age = std::min(30.0f, std::max<int32_t>(0, (int32_t)(g_nowMs - polledMs)) / 1000.0f);
    float reach = g_where == Where::AIRFIELD ? 25000 : 250000;
    std::vector<Info, PsramAllocator<Info>> &info = g_info[g_preparing];
    g_traffic.clear();
    info.clear();
    g_standing.clear();
    uint32_t now = g_nowMs;
    for (const Polled &p : g_polled) {
        const FollowSample &s = p.s;
        bool taxiways = s.ground && !s.rotorcraft && g_where == Where::AIRFIELD && !g_network.empty();
        // Taxiing among real taxiways: carried on along them below, from
        // where it was reported, rather than straight across the grass.
        bool along = taxiways && p.moves;
        Vec3 at = p.at;
        if (p.moves && !along) {
            at = at + p.way * (p.speed * age);
        }
        float dx = at.x - from.x, dy = at.y - from.y;
        if (dx * dx + dy * dy > reach * reach) {
            continue;
        }
        float heading = p.heading;
        Known &k = g_known[p.known];
        float since = k.where == g_where && k.shownMs != 0 ? std::min(1.0f, (now - k.shownMs) / 1000.0f) : -1;
        // Taxiing among the airfield's real taxiways: from the line it was
        // reported on, on along them the way it was going - straight on at
        // each junction - as far as its speed takes it since; facing along
        // them; eased there from where it was shown, so a new report's
        // correction isn't a jump.
        static airfield::List<ground::Vec2> ahead;
        static airfield::List<uint8_t> aheadKinds;
        ground::Place onLine;
        if (along && g_network.snap({at.x, at.y}, heading, 40, onLine)) {
            float line = g_network.heading(onLine);
            float way = fabsf(remainderf(line - heading, 2 * (float)M_PI)) < (float)M_PI / 2 ? line : line + (float)M_PI;
            g_network.ahead(onLine, way, p.speed * std::min(age, 20.0f), ahead, aheadKinds);
            ground::Vec2 q = ahead.back();
            heading = way;
            size_t n = ahead.size();
            if (n >= 2 && hypotf(ahead[n - 1].x - ahead[n - 2].x, ahead[n - 1].y - ahead[n - 2].y) > 0.1f) {
                heading = atan2f(ahead[n - 1].y - ahead[n - 2].y, ahead[n - 1].x - ahead[n - 2].x);
            }
            Vec3 there = {q.x, q.y, at.z};
            if (since >= 0 && length(there - k.at) < 80) {
                at = k.at + (there - k.at) * std::min(1.0f, since / 0.6f);
                float turn = remainderf(heading - k.heading, 2 * (float)M_PI);
                if (fabsf(turn) < 2.1f) {
                    heading = k.heading + turn * std::min(1.0f, since / 0.4f);
                }
            } else {
                at = there;
            }
        } else if (along) {
            at = at + p.way * (p.speed * age);  // off the lines: straight on, as anywhere
        }
        // Parked at a stand: its stop, which the follower tells the stands
        // taken by.
        bool atStand = false;
        ground::Vec2 stop = {0, 0};
        if (!along && taxiways && standingStill(s.ground, s.gsKt)) {
            // Parked: at its stand, nose in, if it is at one - the stand
            // worked out again only when it is reported somewhere else.
            const airfield::List<ground::StandWay> &stands = g_network.stands();
            if (k.where != g_where || hypotf(at.x - k.standFromX, at.y - k.standFromY) > 10) {
                k.stand = -1;
                k.standFromX = at.x;
                k.standFromY = at.y;
                float best = 30;
                for (size_t i = 0; i < stands.size(); i++) {
                    if (!stands[i].ok) {
                        continue;
                    }
                    float d = hypotf(at.x - stands[i].line.back().x, at.y - stands[i].line.back().y);
                    if (d < best) {
                        best = d;
                        k.stand = (int16_t)i;
                    }
                }
            }
            if (k.stand >= 0 && (size_t)k.stand < stands.size()) {
                // Its nose wheel on the stop, as OpenStreetMap has stands -
                // its main wheels, where it is placed, back along the line -
                // and its nose clear of the terminal.
                const ground::StandWay &st = stands[k.stand];
                float facing = st.heading;
                ground::Vec2 q =
                    ground::parkedAt(st, ground::parkBack(st, s3::noseWheelAhead(k.look), s3::noseAhead(k.look)), facing);
                at = {q.x, q.y, at.z};
                heading = facing;
                atStand = true;
                stop = st.line.back();
                k.heading = heading;
                k.at = at;
            }
        }
        if (k.where == g_where && standingStill(s.ground, s.gsKt)) {
            heading = k.heading;
            if (length(at - k.at) < STILL_M) {
                at = {k.at.x, k.at.y, at.z};
            }
            if (g_where == Where::AIRFIELD && g_standing.size() < 200) {
                g_standing.push_back(atStand ? stop : ground::Vec2{at.x, at.y});
            }
        } else if (k.where == g_where && hovering(s.ground, s.gsKt)) {
            heading = k.heading;
        }
        k.at = at;
        k.heading = heading;
        k.where = g_where;
        k.shownMs = now;
        // No taxiing shown - its reports too few and far between to show
        // it truly: on the ground, only stopped on a stand, or on a runway
        // at a runway's speed, landing or taking off - not the runway the
        // view's own is on. A helicopter, set down wherever it is, a helipad
        // or an apron.
        if (s.ground && g_where == Where::AIRFIELD && !(atStand && standingStill(s.ground, s.gsKt))) {
            int r = runwayAt(at);
            if (s.rotorcraft ? !standingStill(s.ground, s.gsKt) : r < 0 || r == busy || s.gsKt < RUNWAY_ROLL_KT) {
                continue;
            }
        }
        float speed = std::max(1.0f, p.speed);
        // Its nose along its climb - raised coming in to land, as it flares -
        // or a helicopter's down as it goes faster.
        float pitch = s.ground         ? 0.0f
                      : s.rotorcraft ? -std::min(9.0f, s.gsKt / 15.0f) * DEG
                                     : s3::landingPitch(k.look.family, s.gsKt, s.vsFpm,
                                                        g_where == Where::AIRFIELD ? p.heightM : NAN,
                                                        atanf(s.vsFpm * 0.00508f / speed) + 0.04f);
        s3::Pose pose = {at, heading, pitch, 0};
        g_traffic.push_back({pose, k.look, s3::configFor(p.heightM, s.gsKt, s.ground, false)});
        info.push_back(p.info);
    }
}

// --- the camera -----------------------------------------------------------------

enum class Cam : uint8_t { CINEMA, CHASE, ORBIT, TOWER, PAST };
Cam g_cam = Cam::CINEMA;
float g_azimuth = 25, g_elevation = 12, g_zoom = 1;  // the orbit's, dragged; and how close, ZOOM IN and OUT
s3::Director *g_director = nullptr;
const char *g_shot = "";

// Which runway, of those setAirfield() was given, `ident` is the end of.
bool runwayOf(const char *ident, int &runway, int &end) {
    for (int i = 0; ident != nullptr && i < g_runways * 2; i++) {
        if (strcmp(g_endName[i], ident) == 0) {
            runway = i / 2;
            end = i % 2;
            return true;
        }
    }
    return false;
}

#ifdef BENCH3D
uint32_t g_sweepUntilMs = 0, g_sweepSaidMs = 0;
#endif

// A helicopter's spot camera: see cameraFor(). Taken below the first height
// over the field and kept up to the second - one hovering about the first
// not flicking between it and a chase - and placed again once its helicopter
// is going away from it this far off, or come to a hover further off than
// the nearer.
constexpr float SPOT_BELOW_M = 300 * 0.3048f, SPOT_UNTIL_M = 450 * 0.3048f, SPOT_REACH_M = 400, SPOT_NEAR_M = 250;
Vec3 g_spotEye{};
bool g_spotSet = false;
bool g_spotAway = false;  // climbing away from it: chased until it comes down again
String g_spotHex;

// Where a helicopter low at an airfield is making for: where its way down
// meets the ground, short of it - it slows to a hover as it comes - or,
// level or climbing, where it will be in ten seconds.
Vec3 spotAhead(const s3::Pose &pose, const FollowEstimate &e) {
    Vec3 along = {cosf(pose.heading), sinf(pose.heading), 0};
    float speed = e.gsKt * 0.5144f, sink = -e.vsFpm * 0.00508f;
    float ahead = sink > 0.5f && speed > 2 ? std::min(700.0f, 0.8f * pose.at.z * speed / sink)
                                           : std::min(250.0f, speed * 10);
    return Vec3{pose.at.x, pose.at.y, 0} + along * ahead;
}

// The spot camera, on the ground off to one side of the way to `spot`: the
// first place, near or a little further, either side, square to it or
// angled, that sees both the spot and the helicopter past the buildings.
Vec3 spotEye(const s3::Pose &pose, Vec3 spot) {
    static const float BEARINGS[] = {90, -90, 50, -50, 130, -130};
    Vec3 first{};
    for (float away : {110.0f, 180.0f}) {
        for (float bearing : BEARINGS) {
            float a = pose.heading + bearing * DEG;
            Vec3 eye = spot + Vec3{cosf(a), sinf(a), 0} * away;
            eye.z = 4;  // a fence, a van's roof
            if (away == 110 && bearing == BEARINGS[0]) {
                first = eye;
            }
            if (s3::seenPast(eye, spot + Vec3{0, 0, 2}) && s3::seenPast(eye, pose.at + Vec3{0, 0, 2})) {
                return eye;
            }
        }
    }
    return first;
}

// The one followed's turning, radians a second, to the left positive: the
// shown heading's, smoothed - see prepareNext(); and the frame's step.
float g_turn = 0, g_frameDt = 0;
// The TOWER camera's raising, over the tower's own, for a building between.
float g_towerRaise = 0;

// FLY PAST: a camera set down ahead of it, beside the way it is going -
// where it will be in PAST_AHEAD_S - held still as it comes, goes by and
// away, its lens following it but not zooming: small far off, growing as it
// comes, sweeping across side-on, shrinking as it goes; set down ahead again
// once it is PAST_AFTER_S past - the other side of its way. Low, it stands
// on the ground; high up, it hangs in the air beside its way, a little
// below. (Held the same size, with the lens's every turn, it seemed to hang
// still.)
constexpr float PAST_AHEAD_S = 7, PAST_AFTER_S = 5;
constexpr float PAST_SPANS = 4;     // how far off its way: four wingspans - 140m for an A320 - ZOOM IN nearer
constexpr float PAST_SIDE_S = 0.8f;  // and further for one fast, to be side-on a while: 185m at 450kt - at 280m, a fast jet's pass was faint
constexpr float PAST_FILL = 0.45f;  // how much of the view it fills going by
constexpr float PAST_LOW_M = 150;   // lower than this where it passes, the camera is on the ground
Vec3 g_passEye{};
float g_passFar = 0;  // how far off it was, the camera set down
float g_passFov = 30;  // the lens, for the pass
bool g_passSet = false, g_passKeepSide = false;
int g_passSide = 1;  // its left; -1 its right
String g_passHex;
Where g_passWhere = Where::NOWHERE;

// Where it will be in `t` seconds, going on as it is: round its turn - a
// quarter of one at most - at its speed, climbing or coming down as it is.
Vec3 aheadOf(const s3::Pose &pose, float speedMs, float turn, float climbMs, float t, float &heading) {
    if (fabsf(turn) * t > (float)M_PI / 2) {
        t = (float)M_PI / 2 / fabsf(turn);
    }
    float h = pose.heading;
    Vec3 p = pose.at;
    if (fabsf(turn) < 1e-3f) {
        p = p + Vec3{cosf(h), sinf(h), 0} * (speedMs * t);
    } else {
        float r = speedMs / turn;
        p = p + Vec3{r * (sinf(h + turn * t) - sinf(h)), -r * (cosf(h + turn * t) - cosf(h)), 0};
    }
    p.z += climbMs * t;
    heading = h + turn * t;
    return p;
}

// The fly past's camera set down: beside where it will be, on the side it
// is the turn of - the other, or further off, if a building is in the way.
void placePass(const s3::Pose &pose, const Followed &f, float size) {
    float speed = f.now.gsKt * 0.5144f, climb = f.now.ground ? 0 : f.now.vsFpm * 0.00508f, heading;
    Vec3 p = aheadOf(pose, speed, g_turn, climb, PAST_AHEAD_S, heading);
    if (g_where == Where::AIRFIELD) {
        p.z = std::max(0.0f, p.z);
    }
    if (!g_passKeepSide && g_passSet) {
        g_passSide = -g_passSide;
    }
    g_passKeepSide = false;
    Vec3 left = {-sinf(heading), cosf(heading), 0};
    float off = std::max(20.0f, std::max(PAST_SPANS * 36 * size, speed * PAST_SIDE_S) / g_zoom);
    // The ground under a point: the airfield's flat; high up, the earth's
    // round, falling away from the frame's middle - 31m lower 20km off.
    auto ground = [](Vec3 q) { return g_where == Where::AIRFIELD ? 0.0f : -(q.x * q.x + q.y * q.y) / (2 * 6371000.0f); };
    bool low = p.z - ground(p) < PAST_LOW_M;
    Vec3 first{};
    bool found = false;
    for (int side : {g_passSide, -g_passSide}) {
        for (float further : {1.0f, 1.6f}) {
            Vec3 eye = p + left * (side * off * further);
            eye.z = low ? ground(eye) + 2 : p.z - 0.3f * off;
            if (side == g_passSide && further == 1) {
                first = eye;
            }
            if (!found && (g_where != Where::AIRFIELD || s3::seenPast(eye, p + Vec3{0, 0, 3}))) {
                g_passEye = eye;
                found = true;
            }
        }
    }
    if (!found) {
        g_passEye = first;
    }
    g_passFar = hypotf(g_passEye.x - pose.at.x, g_passEye.y - pose.at.y);
    // Zoomed for when it is nearest, going by.
    float nearest = std::max(1.0f, length(p + Vec3{0, 0, 3.5f * size} - g_passEye));
    g_passFov = std::max(1.5f, std::min(65.0f, 2 * atanf(40 * size / PAST_FILL / 2 / nearest) / DEG));
    g_passSet = true;
    g_passHex = f.hex;
    g_passWhere = g_where;
}

s3::Camera cameraFor(const s3::Aircraft &followed, const Followed &f, float seconds, s3::Pose &shown,
                     bool &replaying) {
    const s3::Pose &pose = followed.pose;
#ifdef BENCH3D
    if (g_sweepUntilMs != 0) {
        uint32_t now = millis();
        if ((int32_t)(now - g_sweepUntilMs) >= 0) {
            g_sweepUntilMs = 0;
            Serial.println("[3d] sweep done");
        } else {
            // Rates that never line up, so the angles fill in as it goes.
            float t = now / 1000.0f;
            g_azimuth = fmodf(t * 97.0f, 360.0f);
            g_elevation = -10 + 90 * (0.5f + 0.5f * sinf(t * 0.91f));
            g_zoom = 0.3f * powf(4.0f / 0.3f, 0.5f + 0.5f * sinf(t * 0.37f));
            if (now - g_sweepSaidMs >= 500) {
                g_sweepSaidMs = now;
                Serial.printf("[3d] sweep: azimuth %.0f elevation %.0f zoom %.2f, %s\n", g_azimuth, g_elevation,
                              g_zoom, g_where == Where::AIRFIELD ? g_airport : "high up");
            }
        }
    }
#endif
    float size = s3::sizeOf(followed.look);
    replaying = false;
    Vec3 eye;
    switch (g_cam) {
    // Zoomed in no nearer than 28 of its size: nearer, the eye was in a
    // Chinook's rotor discs.
    case Cam::CHASE:
        g_shot = "chase";
        return s3::orbitCamera(pose, 0, 10, std::max(28 * size, 55 * size / g_zoom), 50);
    case Cam::ORBIT:
        g_shot = "orbit";
        return s3::orbitCamera(pose, g_azimuth, g_elevation, std::max(28 * size, 80 * size / g_zoom), 50);
    case Cam::TOWER:
        if (s3::towerEye(eye)) {
            // Raised further while a building stands between the cab and it
            // - quickly up, slowly down - for it not to be watched through
            // a terminal's wall.
            float need = 0;
            Vec3 middle = pose.at + Vec3{0, 0, 3 * size};
            while (need < 60 && !s3::seenPast(eye + Vec3{0, 0, need}, middle, true)) {
                need += 5;
            }
            g_towerRaise += (need - g_towerRaise) * std::min(1.0f, g_frameDt / (need > g_towerRaise ? 0.6f : 3.0f));
            eye.z += g_towerRaise;
            g_shot = "tower";
            return s3::fixedCamera(pose, eye, std::min(0.9f, 0.3f * g_zoom), size);
        }
        break;
    case Cam::PAST: {
        // Set down again once it is well past - or once it has turned away
        // and will never come by - or it is another, or another frame.
        float speed = f.now.gsKt * 0.5144f;
        Vec3 to = g_passEye - pose.at;
        float along = to.x * cosf(pose.heading) + to.y * sinf(pose.heading);
        bool past = along < -std::max(60.0f, speed * PAST_AFTER_S);
        bool away = hypotf(to.x, to.y) > g_passFar + 500;
        if (!g_passSet || g_passHex != f.hex || g_passWhere != g_where || past || away) {
            placePass(pose, f, size);
        }
        g_shot = "fly past";
        s3::Camera camera = s3::fixedCamera(pose, g_passEye, PAST_FILL, size);
        camera.fovDeg = g_passFov;
        return camera;
    }
    case Cam::CINEMA:
        break;
    }
    // A helicopter low at an airfield, coming in or going: a camera on the
    // ground near where it sets down or lifts off - ahead of it, where it is
    // making for, and off to one side - its lens following it as it hovers
    // in and touches down, or lifts and goes; placed again once it is far
    // off; a chase once it is well up.
    if (g_where == Where::AIRFIELD && isRotorcraft(f.type, f.category)) {
        if (g_spotHex != f.hex) {
            g_spotSet = g_spotAway = false;
            g_spotHex = f.hex;
        }
        if (f.now.vsFpm < -200) {
            g_spotAway = false;  // coming down again
        }
        Vec3 off = {pose.at.x - g_spotEye.x, pose.at.y - g_spotEye.y, 0};
        float far = length(off);
        bool leaving = f.now.gsKt > 5 && off.x * cosf(pose.heading) + off.y * sinf(pose.heading) > 0;
        if (g_spotSet && leaving && far > SPOT_REACH_M && f.now.vsFpm > 300) {
            g_spotAway = true;
        }
        if (!g_spotAway && pose.at.z < (g_spotSet ? SPOT_UNTIL_M : SPOT_BELOW_M)) {
            // Placed again for one going off low - across the field, say -
            // or one a long way from it, the view opened again on it; and
            // nearer for one come to a hover well off from where it was
            // looked for, to set down there, or to lift off and go.
            bool settling = f.now.gsKt < 10 && far > SPOT_NEAR_M;
            if (!g_spotSet || (leaving && far > SPOT_REACH_M) || far > 1500 || settling) {
                g_spotEye = spotEye(pose, spotAhead(pose, f.now));
                g_spotSet = true;
            }
            g_shot = "spot";
            return s3::fixedCamera(pose, g_spotEye, std::min(0.9f, 0.3f * g_zoom), size);
        }
        g_spotSet = false;
        g_shot = "chase";
        return s3::orbitCamera(pose, 25, 9, 70 * size, 48);
    }
    if (g_where == Where::AIRFIELD) {
        // The director's cuts, from what it is doing: on its runway, the
        // approach and touchdown, the roll and climb-out; elsewhere a chase.
        s3::Flight flight;
        flight.aircraft = followed;
        flight.speedKt = g_taxiKt >= 0 ? g_taxiKt : f.now.gsKt;
        flight.climbFpm = f.now.vsFpm;
        flight.onGround = f.now.ground;
        flight.departing = f.movement == RunwaySim::Movement::TAKEOFF;
        int runway = -1, end = 0;
        if (f.movement != RunwaySim::Movement::NONE && runwayOf(f.runway.ident, runway, end)) {
            flight.runway = runway;
            flight.end = end;
            if (flight.departing) {
                s3::setDeparture(runway, end);
            } else {
                s3::setLanding(runway, end);
            }
        }
        return g_director->next(flight, seconds, &g_shot, &shown, &replaying);
    }
    // High up: close behind, then wide, then from below, in turn - each
    // held steady on it, none swinging round it.
    float phase = fmodf(seconds, 30);
    if (phase < 10) {
        g_shot = "chase";
        return s3::orbitCamera(pose, 25, 9, 70 * size, 48);
    }
    if (phase < 20) {
        // Wide, but by its size: 900m off, a light aircraft was a dot.
        g_shot = "wide";
        return s3::orbitCamera(pose, 140, 22, 230 * size, 50);
    }
    g_shot = "from below";
    return s3::orbitCamera(pose, 215, -18, 120 * size, 52);
}

// --- the one followed's moments -------------------------------------------------------

// Whether the one followed was on the ground last frame; when it last
// touched down, on the view's clock, and where.
bool g_wasGround = false;
float g_touchdownAt = -100;
s3::Pose g_touchdownPose{};
// Since when, on the view's clock, it has been held on the runway rotating -
// a report having it off the ground first - or -1.
float g_rotateHeldAt = -1;
// How fast it was coming down as it touched, m/s - its tyres' screech the
// louder the harder - and, till it does, as it last was in the air.
float g_touchdownSinkMs = 1, g_sinkMs = 1;
// Parked at its stand since (g_seconds), -1 not; and whether it has been seen
// moving on the ground, or was found parked.
float g_parkedAt = -1;
bool g_movedOnGround = false;
// When its engines were last started, on the view's clock; and whether they
// were running a frame ago - -1 not known yet, a new follow's first frame
// no start.
float g_startedAt = -1000;
int g_runningWas = -1;

// --- the engines' noise ---------------------------------------------------------------

// The SOUND button's: whether the engines are heard - see engine_noise.h.
bool g_engineSound = true;
float g_engineDistance = -1;  // the camera's from the one followed, a frame ago

// The one followed's engines as the camera hears them: their kind, from its
// type - the A320 family's and the Typhoon's, any other silent; how hard
// they are working, from what it is doing - see engines3d.h, which a PC
// check runs through each phase; where it is from the camera - how far off,
// how fast it is closing, which way it faces, and how high it and the camera
// are over the ground, for the sound off it; and its tyres, rolling and
// just down. See engine_noise.h for what the air does with it all.
engine_noise::Sound engineSound(const Followed &f, const FollowEstimate &e, int shownKt, const s3::Look &look,
                                const s3::Pose &pose, const s3::Config &setUp, const s3::Camera &camera, float dt) {
    using engine_noise::Kind;
    // Which aircraft it is: another heard afresh, as it is - not this one's
    // engines and APU spooled up from where the last one's left off - and
    // which engines it has, the same each time it is followed.
    uint32_t who = 2166136261u;
    for (size_t i = 0; i < f.hex.length(); i++) {
        who = (who ^ (uint8_t)f.hex[i]) * 16777619u;
    }
    engine_noise::Sound s = engines3d::enginesOf(f.type.c_str(), who);
    s.who = who;
    float aboveFt = g_where == Where::AIRFIELD ? pose.at.z / 0.3048f : 1e9f;
    // What it is doing, for how hard its engines work and its reheat - a
    // fighter's alone, not a trainer's, an AWACS's or a U-2's.
    engines3d::Doing doing;
    doing.ground = e.ground;
    doing.takingOff = f.movement == RunwaySim::Movement::TAKEOFF;
    doing.landing = f.movement == RunwaySim::Movement::LANDING;
    doing.simulating = f.simulating;
    doing.kt = shownKt;
    doing.rollKt = e.gsKt;
    doing.vsFpm = e.vsFpm;
    doing.altFt = e.altFt;
    doing.low = aboveFt < 3000;
    doing.fighter = look.family == s3::Family::FIGHTER_DELTA || look.family == s3::Family::FIGHTER ||
                    look.family == s3::Family::FIGHTER_SINGLE;
    engines3d::ask(s, doing);
    // Its engines about its middle, a little up - a Typhoon's lower.
    float up = engines3d::enginesUpPerScale(s, look.scale);
    Vec3 to = camera.eye - (pose.at + Vec3{0, 0, up * look.scale});
    float d = length(to);
    s.distanceM = d;
    // How fast it is coming closer, from the distance a frame ago - a cut
    // from one camera to another, a jump no aircraft makes, as none.
    float closing = g_engineDistance >= 0 && dt > 0 ? (g_engineDistance - d) / dt : 0;
    s.closingMs = fabsf(closing) < 300 ? closing : 0;
    g_engineDistance = d;
    Vec3 nose = {cosf(pose.heading) * cosf(pose.pitch), sinf(pose.heading) * cosf(pose.pitch), sinf(pose.pitch)};
    float c = (nose.x * to.x + nose.y * to.y + nose.z * to.z) / std::max(1.0f, d);
    s.aspect = acosf(std::max(-1.0f, std::min(1.0f, c))) / (float)M_PI;
    // And where it and the camera are, for the synthesiser to work out the
    // sound's way itself - when what is heard left it, from where - which
    // the three above, as one frame has them, can't: a fast jet's sound
    // arriving all at once as it passes, and a cut to another camera heard
    // as that camera hears it.
    Vec3 engines = pose.at + Vec3{0, 0, up * look.scale};
    s.placed = true;
    s.at[0] = engines.x, s.at[1] = engines.y, s.at[2] = engines.z;
    s.nose[0] = nose.x, s.nose[1] = nose.y, s.nose[2] = nose.z;
    s.ear[0] = camera.eye.x, s.ear[1] = camera.eye.y, s.ear[2] = camera.eye.z;
    // Timed by the clock it moves by - millis() live, the recording's in a
    // replay - for its speed to be its own: by the wall's, a replay at 4x
    // had it coming four times as fast as it flew, its whine as it came
    // three times its pitch.
    s.ms = g_nowMs;
    // Off the ground: its engines' height over it, and the camera's - high
    // up, over the sea the round earth is, too high for an echo to matter;
    // the ground hard where it is on the airfield or just over it, a
    // runway's or an apron's, grass and earth elsewhere.
    bool airfield = g_where == Where::AIRFIELD;
    Vec3 eye = camera.eye;
    s.sourceUpM = airfield ? pose.at.z + up * look.scale : e.altFt * 0.3048f;
    s.earUpM = airfield ? eye.z : eye.z + (eye.x * eye.x + eye.y * eye.y) / (2 * 6371000.0f);
    s.hardGround = airfield && (e.ground || aboveFt < 300);
    // Its tyres: rolling at its speed on the ground; touched down lately,
    // how long ago and how hard.
    s.rollingMs = e.ground ? shownKt * 0.5144f : 0;
    float since = g_seconds - g_touchdownAt;
    s.touchdownS = since >= 0 && since < 3 ? since : -1;
    s.sinkMs = g_touchdownSinkMs;
    // Its airframe as the view has it set up: the air past it, its gear,
    // flaps and spoilers - see engines3d::airframe() for a Typhoon's.
    s.airspeedMs = shownKt * 0.5144f;
    s.size = s3::sizeOf(look);
    engines3d::airframe(s, setUp.gear, setUp.flaps, setUp.spoilerDeploy);
    // Its engines shut down at its stand - a minute after it parks there,
    // cooled at idle, or already when it was found parked, as at a gate -
    // and started again as it is pushed back or moves off, heard spooling up
    // from nothing. An airliner's APU running while it stands or taxies,
    // not as it takes off. Without the airfield pack's stands there is
    // nothing to tell a stand from a hold short of the runway: running.
    bool parked = g_follower.parked();
    if (!parked) {
        g_parkedAt = -1;
    } else if (g_parkedAt < 0) {
        g_parkedAt = g_seconds;
    }
    g_movedOnGround = g_movedOnGround || (e.ground && shownKt > 3);
    s.running = !(parked && (!g_movedOnGround || g_seconds - g_parkedAt > 60));
    if (s.running && g_runningWas == 0) {
        g_startedAt = g_seconds;
    }
    g_runningWas = s.running ? 1 : 0;
    // The APU where an airliner runs it: at its stand; for a minute and a
    // half after its engines start there, its air and power handed over to
    // them; and taxiing in from a landing, started for the stand. Not
    // taxiing out or lining up - shut down minutes before.
    bool startedLately = g_seconds - g_startedAt < 90, taxiingIn = g_touchdownAt > -100 && shownKt < 25;
    s.apu = s.kind == Kind::TURBOFAN && e.ground && f.movement != RunwaySim::Movement::TAKEOFF &&
            (parked || startedLately || taxiingIn);
    static bool wasRunning = true, wasApu = false;
    if (s.running != wasRunning || s.apu != wasApu) {
        Serial.printf("[3d] %s: engines %s, APU %s\n", f.callsign.c_str(), s.running ? "running" : "shut down",
                      s.apu ? "on" : "off");
        wasRunning = s.running;
        wasApu = s.apu;
    }
    return s;
}

// --- a frame -----------------------------------------------------------------------

s3::Scene g_scene;
s3::Pilot g_pilot;
// Where the one followed is shown, and which way it points: the estimate
// smoothed - see prepareNext().
Vec3 g_shownAt{};
float g_shownHeading = 0;
bool g_haveShown = false;
Where g_shownWhere = Where::NOWHERE;
float g_lastSeconds = -1, g_lastTrack = 0;
String g_followedHex;
// The frame-rate governor's: aircraft drawn simpler sooner while frames run
// long, finer again while they don't - as the bench found best; and the land
// at its height skimping, its samples fewer and less far, likewise - see
// scene3d's Scene::terrainEffort.
float g_detailPx = 60, g_tinyPx = 12, g_terrainEffort = 0;
#ifdef BENCH3D
bool g_benchFullLand = false;  // the bench's "##K0": the land never skimped, for a still
#endif
uint32_t g_frameMs = 0;
// For the log, every ten seconds: frames, and the time preparing them and
// waiting for the drawing.
uint32_t g_statFromMs = 0, g_statFrames = 0, g_statPrepUs = 0, g_statWaitUs = 0;
// And the drawing's own time, on the other core.
uint32_t g_statDrawUs = 0;
// And how many frames' buildings the drawing core had worked out, then.
uint32_t g_statSolidsAsideFrom = 0;
// And the drawing's stages and waits for the PPA, as they were then.
uint32_t g_statStageFrom[6] = {}, g_statPushWaitFrom = 0, g_statLoopBandsFrom = 0;
// And the preparing's steps, summed over those frames, for the costliest of
// them said with it - and how much each frame had.
uint32_t g_statStepUs[s3::STEPS] = {};
uint32_t g_statTriangles = 0, g_statLines = 0;
// And this view's own, round the scene's: the traffic placed, and the labels.
uint32_t g_statTrafficUs = 0, g_statLabelsUs = 0;

// What covers the land - the ground pack's tiles round (lat, lon), and where
// the view's ground is on their grid: see scene3d's GroundCover. At an
// airfield, straight from local()'s x and y, as it puts places there; high
// up, from latitude and longitude about the view's origin on the sphere it
// draws, each scaled to the ellipsoid's radii there, as g_enu is.
#ifdef BENCH3D
bool g_benchCoverOff = false;
#endif

void setCover(double lat, double lon, bool high, float upM) {
    g_scene.cover = s3::GroundCover();
#ifdef BENCH3D
    if (g_benchCoverOff) {
        return;
    }
#endif
    if (groundpack::want(lat, lon, 45, upM)) {
        g_scene.cover = high ? s3::roundCover(groundpack::window(), g_enuLat, g_enuLon)
                             : s3::flatCover(groundpack::window(), g_lat0, g_lon0);
    }
}

// Where the eye was last, for ##D.
double g_eyeLat = 0, g_eyeLon = 0;
// The land at its height - the hills from any height - rather than flat
// with a skyline: the GROUND button's, saved, and the settings' at boot -
// off to start with; on the bench ##m switches it too, unsaved, to set the
// two side by side.
bool g_terrain = false;

// The land can be drawn at its height here: the card's pack read in, with
// land of it round the eye. Else it is flat whatever GROUND says - the strip
// saying why - and the button greyed.
bool terrainHere() {
    return groundpack::status() == groundpack::Status::READY && !groundpack::offPack();
}

// The skyline: the horizon round the camera's eye, from the ground pack -
// see groundpack::Horizon - wanted afresh as the eye moves, the last worked
// out given the scene. The eye's latitude, longitude and height above the
// sea from where the view puts it: at an airfield, in metres east and north
// of the origin and up from the field; high up, about the ENU origin, the
// earth curving away below.
void setHorizon(const s3::Camera &camera, bool high) {
    g_scene.horizon = nullptr;
    g_scene.terrainGrid = groundpack::terrainGrid();
    g_scene.terrain = g_terrain && g_scene.cover.window != nullptr && g_scene.terrainGrid != nullptr;
    g_scene.groundUpM = high ? 0 : g_fieldFt * 0.3048f;
    if (g_scene.cover.window == nullptr || g_scene.terrain) {
        return;  // no pack, or the bench's cover off - or no skyline, the land its own hills
    }
    Vec3 eye = camera.eye;
    double lat0 = high ? g_enuLat : g_lat0, lon0 = high ? g_enuLon : g_lon0;
    double lat = lat0 + eye.y / 110574.0, lon = lon0 + eye.x / (111320.0 * cos(lat0 * M_PI / 180));
    float heightM = high ? eye.z + (eye.x * eye.x + eye.y * eye.y) / (2 * 6371000.0f) : eye.z + g_fieldFt * 0.3048f;
    g_scene.eyeHeightM = heightM;
    g_eyeLat = lat;
    g_eyeLon = lon;
    // None wanted over 4,000m, where the skyline isn't drawn - the hills seen
    // from above - nor worked out afresh each 300m the eye goes up there, a
    // second of the ground task's at a time.
    if (heightM > 4000) {
        return;
    }
    groundpack::wantHorizon(lat, lon, heightM);
    const groundpack::Horizon *hz = groundpack::horizon();
    g_scene.horizon = hz;
    if (hz != nullptr) {
        // Where it was worked out from, in the view's frame.
        g_scene.horizonAt = high ? g_enu.of(hz->lat, hz->lon, hz->heightM)
                                 : local(hz->lat, hz->lon) + Vec3{0, 0, hz->heightM - g_fieldFt * 0.3048f};
    }
    // North at the eye, turned from the view's north by the meridians
    // closing in: east of the origin, towards it.
    g_scene.horizonTurn = (float)((lon - lon0) * M_PI / 180 * sin(lat * M_PI / 180));
    g_scene.eyeHeightM = heightM;
}

void prepareNext(const Followed &f, const std::vector<Aircraft> &traffic, uint32_t polledMs) {
    choose(f);
    float seconds = g_seconds;
    float dt = g_lastSeconds < 0 ? 0 : std::min(0.5f, seconds - g_lastSeconds);
    g_frameDt = dt;
    g_lastSeconds = seconds;
    FollowEstimate e = estimateOf(f);
    bool rotor = isRotorcraft(f.type, f.category);
    // Taking off, a report off the ground before it has been seen rotating -
    // the runway simulation reckoning it slower than it was, the feed's
    // reports far apart - held on its main wheels while its nose comes up,
    // a couple of seconds at most, then away: no airliner leaves a runway
    // level, as it was shown doing.
    if (g_where == Where::AIRFIELD && f.movement == RunwaySim::Movement::TAKEOFF && !rotor) {
        float lift = s3::liftOffPitch(lookOf(f.hex, f.type, f.callsign, f.military, f.category).family);
        if (!e.ground && g_wasGround && g_rotateHeldAt < 0 && g_pilot.pitch < lift - 1 * DEG) {
            g_rotateHeldAt = seconds;
        }
        if (g_rotateHeldAt >= 0 && (e.ground || g_pilot.pitch >= lift - 0.5f * DEG || seconds - g_rotateHeldAt > 4)) {
            g_rotateHeldAt = -1;
        }
    } else {
        g_rotateHeldAt = -1;
    }
    if (g_rotateHeldAt >= 0) {
        e.ground = true;
        e.vsFpm = 0;
    }

    // Where it is shown. Taxiing between reports, which the estimate holds
    // it still for, moved on along its track at its speed. And each jump
    // the estimate makes - a report putting it right, on the ground or in a
    // turn - not shown as a jump: the shown position carried on by its speed
    // and eased towards the estimate over a second or so, and its heading
    // likewise; on a straight steady path that is the estimate exactly.
    double lat = e.lat, lon = e.lon;
    if (e.ground && !f.simulating && e.gsKt >= STILL_KT && f.reportMs != 0) {
        double d = e.gsKt * 0.5144 * std::min(8.0f, std::max<int32_t>(0, (int32_t)(g_nowMs - f.reportMs)) / 1000.0f);
        lat += d * cos(e.track * M_PI / 180) / 111320.0;
        lon += d * sin(e.track * M_PI / 180) / (111320.0 * cos(lat * M_PI / 180));
    }
    Vec3 target = place(lat, lon, e.altFt, e.ground);
    if (rotor && !e.ground && g_where == Where::AIRFIELD) {
        target.z = std::max(target.z, ROTOR_LOWEST_M);
    }
    float targetHeading = (90 - e.track) * DEG;
    if (g_haveShown && g_shownWhere == g_where && standingStill(e.ground, e.gsKt)) {
        targetHeading = g_shownHeading;
        if (length(target - g_shownAt) < STILL_M) {
            target = {g_shownAt.x, g_shownAt.y, target.z};  // a helicopter's height still coming down
        }
    } else if (g_haveShown && g_shownWhere == g_where && hovering(e.ground, e.gsKt)) {
        targetHeading = g_shownHeading;
    }
    if (!g_haveShown || g_shownWhere != g_where || length(target - g_shownAt) > 2000 || dt <= 0) {
        g_shownAt = target;
        g_shownHeading = targetHeading;
        g_haveShown = true;
        g_shownWhere = g_where;
    } else {
        float speed = e.gsKt * 0.5144f;
        Vec3 velocity = {sinf(e.track * DEG) * speed, cosf(e.track * DEG) * speed,
                         e.ground ? 0.0f : e.vsFpm * 0.00508f};
        // A helicopter a report puts back - its estimate, slowing to a
        // hover, having taken it on too far - eased back the more gently,
        // and not carried on meanwhile: a drift back, not a lurch.
        Vec3 off = target - g_shownAt;
        bool back = rotor && off.x * velocity.x + off.y * velocity.y < 0;
        if (!back) {
            g_shownAt = g_shownAt + velocity * dt;
        }
        Vec3 step = (target - g_shownAt) * std::min(1.0f, dt / (back ? 2.5f : 0.8f));
        if (rotor && !back) {
            // And one a report puts well on - up and away since the last,
            // the feed slow - caught up with at no more than its speed again
            // and 6m/s up or down: seen to lift and go, not to jump there.
            float most = std::max(15.0f, speed) * dt, across = hypotf(step.x, step.y);
            if (across > most) {
                step.x *= most / across;
                step.y *= most / across;
            }
            step.z = std::max(-6 * dt, std::min(6 * dt, step.z));
        }
        g_shownAt = g_shownAt + step;
        g_shownHeading += remainderf(targetHeading - g_shownHeading, 2 * (float)M_PI) * std::min(1.0f, dt / 1.0f);
    }
    if (e.ground && !rotor) {
        g_shownAt.z = target.z;  // on the ground, on it
    } else if (g_where == Where::AIRFIELD) {
        // Never under it. A helicopter set down - shown higher by its
        // pressure until its transponder said so, or a few feet up - settles
        // onto it over a second or two rather than dropping; one not set
        // down is off it.
        g_shownAt.z = std::max(rotor && !e.ground ? ROTOR_LOWEST_M : 0.0f, g_shownAt.z);
    }

    // On the ground at an airfield whose real taxiways are known - and not
    // racing down a runway, landing or taking off, which the runway
    // simulation has - the follower's: along the taxiways from report to
    // report, never across the grass, nor backwards; to a stand, landed,
    // or a holding point, departing, when the reports stop - see
    // ground_follow.h. Handed over from a landing's roll at 40 knots, it
    // keeps it until it is airborne or rolling for take-off.
    // Not a helicopter: it sets down where it means to, and lifts off from
    // there, along no taxiway.
    bool taxiways = g_where == Where::AIRFIELD && e.ground && !g_network.empty() && !rotor;
    s3::Look look = lookOf(f.hex, f.type, f.callsign, f.military, f.category);
    // Rolling for take-off it is handed back to the runway simulation once
    // it is shown where that has it - or at 80 knots, whatever - so there is
    // no lurch.
    bool rolling = e.gsKt > 40 &&
                   (!g_follower.active() ||
                    (f.movement == RunwaySim::Movement::TAKEOFF &&
                     (e.gsKt > 80 || hypotf(g_follower.at().x - target.x, g_follower.at().y - target.y) < 15)));
    g_taxiKt = -1;
    if (taxiways && !rolling) {
        ground::Follower::Input in{};
        in.ms = g_nowMs;
        in.dt = dt;
        if (f.reportMs != 0 && f.report.hasPos) {
            Vec3 r = place(f.report.lat, f.report.lon, 0, true);
            in.reportMs = f.reportMs;
            in.reportAt = {r.x, r.y};
            in.reportHeading = f.report.hasTrack ? (90 - f.report.track) * DEG : NAN;
            in.reportSpeed = f.report.hasGs ? f.report.gsKt * 0.5144f : 0;
        }
        in.takeoff = f.simulating && f.movement == RunwaySim::Movement::TAKEOFF;
        in.simAt = {target.x, target.y};
        in.simHeading = targetHeading;
        in.simSpeed = e.gsKt * 0.5144f;
        in.standing = g_standing.data();
        in.standingCount = (int)g_standing.size();
        in.arrived = f.landed;
        // The runway ends departures are using, as the zoom saw: where each
        // starts.
        static ground::Vec2 departFrom[8];
        int departs = 0;
        int runway, end;
        for (int i = 0; strcmp(f.inUseAirport, g_airport) == 0 && i < f.inUseCount && departs < 8; i++) {
            if (runwayOf(f.inUse[i], runway, end)) {
                const Vec3 &at = g_endAt[runway * 2 + end];
                departFrom[departs++] = {at.x, at.y};
            }
        }
        in.departFrom = departFrom;
        in.departFromCount = departs;
        in.lost = f.lost;
        in.noseWheel = s3::noseWheelAhead(look);
        in.nose = s3::noseAhead(look);
        if (!g_follower.active()) {
            bool landed = f.movement == RunwaySim::Movement::LANDING;
            g_follower.begin(g_network, {g_shownAt.x, g_shownAt.y}, g_shownHeading, e.gsKt * 0.5144f, landed, in);
        }
        g_follower.update(g_network, in);
        g_shownAt = {g_follower.at().x, g_follower.at().y, target.z};
        g_shownHeading = g_follower.heading();
        g_taxiKt = (int)lroundf(g_follower.speed() / 0.5144f);
        if (strcmp(g_taxiSaid, g_follower.doing()) != 0) {
            snprintf(g_taxiSaid, sizeof g_taxiSaid, "%s", g_follower.doing());
            Serial.printf("[3d] taxi: %s\n", g_taxiSaid);
        }
    } else if (g_follower.active()) {
        g_follower.end();
        g_taxiSaid[0] = 0;
    }
    int shownKt = g_taxiKt >= 0 ? g_taxiKt : e.gsKt;

    // The one followed, flown: banked into its turns - the shown heading's
    // turning - and pitched by its climb, its controls moving with it.
    float turn = remainderf(g_shownHeading - g_lastTrack, 2 * (float)M_PI);
    g_lastTrack = g_shownHeading;
    if (dt > 0) {
        g_turn += (turn / dt - g_turn) * std::min(1.0f, dt / 2);
    }
    Vec3 at = g_shownAt;
    float heightM = g_where == Where::AIRFIELD ? at.z : e.altFt * 0.3048f;
    // Over the airfield its height over the ground is known, for its flare.
    g_pilot.fly(dt, look.family, g_turn, shownKt, e.vsFpm, e.ground, g_where == Where::AIRFIELD ? at.z : NAN,
                f.movement == RunwaySim::Movement::TAKEOFF);
    s3::Pose pose = {at, g_shownHeading, g_pilot.pitch, g_pilot.roll};
    s3::Config config = s3::configFor(heightM, shownKt, e.ground, f.movement == RunwaySim::Movement::TAKEOFF);
    config.aileron = g_pilot.aileron;
    config.elevator = g_pilot.elevator;
    config.rudder = g_pilot.rudder;
    g_scene.followed = {pose, look, config};
    // Its tyres' smoke, where they touch.
    if (!e.ground) {
        g_sinkMs = std::max(0.3f, -e.vsFpm * 0.00508f);
    }
    if (g_where == Where::AIRFIELD && e.ground && !g_wasGround && f.movement == RunwaySim::Movement::LANDING) {
        g_touchdownAt = seconds;
        g_touchdownPose = pose;
        g_touchdownSinkMs = g_sinkMs;
        String call = f.callsign;
        call.trim();
        Serial.printf("[3d] %s down on %s at %s, its nose %.1f degrees up\n", call.length() ? call.c_str() : f.hex.c_str(),
                      f.runway.ident != nullptr ? f.runway.ident : "?", g_airport, g_pilot.pitch / DEG);
    }
    // And lifting off, rotated for - its nose up as it leaves the runway.
    if (g_where == Where::AIRFIELD && !e.ground && g_wasGround && f.movement == RunwaySim::Movement::TAKEOFF) {
        String call = f.callsign;
        call.trim();
        Serial.printf("[3d] %s off %s at %s at %d kt, its nose %.1f degrees up\n",
                      call.length() ? call.c_str() : f.hex.c_str(), f.runway.ident != nullptr ? f.runway.ident : "?",
                      g_airport, shownKt, g_pilot.pitch / DEG);
    }
    g_wasGround = e.ground;
    // Its spoilers springing up as it touches down, not there at once.
    s3::Config &setUp = g_scene.followed.config;
    setUp.spoilerDeploy *= s3::spoilersRising(seconds - g_touchdownAt);
    setUp.spoilers = setUp.spoilerDeploy > 0.01f;
    g_scene.smokeAge = seconds - g_touchdownAt < 3 ? seconds - g_touchdownAt : -1;
    g_scene.touchdown = g_touchdownPose;

    // The runway the view's aircraft is landing on or taking off from - its
    // own, kept clear of the rest.
    int busy = -1, end;
    if (g_where == Where::AIRFIELD) {
        if (f.movement != RunwaySim::Movement::NONE && f.runway.ident != nullptr) {
            runwayOf(f.runway.ident, busy, end);
        } else if (e.ground) {
            busy = runwayAt(at);
        }
    }
    uint32_t t0 = micros();
    fillTraffic(traffic, polledMs, f.hex, at, busy);
    g_statTrafficUs += micros() - t0;
    g_scene.traffic = g_traffic.data();
    g_scene.trafficCount = (int)g_traffic.size();
    g_scene.seconds = seconds;
    // The sun as it was then, in a replay - morning's flight not drawn by
    // the night it is replayed in.
    time_t now = g_inReplay && g_replay.epoch != 0 ? (time_t)g_replay.epoch : time(nullptr);
    g_scene.sun = now > 1600000000 ? s3::sunAt((int64_t)now, e.lat, e.lon) : s3::sunFor(s3::TimeOfDay::DAY);
#ifdef BENCH3D
    if (g_benchSun != 0) {
        g_scene.sun = s3::sunFor(g_benchSun == 1   ? s3::TimeOfDay::DAY
                                 : g_benchSun == 2 ? s3::TimeOfDay::DUSK
                                                   : s3::TimeOfDay::NIGHT);
    }
#endif
    bool high = g_where == Where::COUNTRY;
    g_scene.earthRadius = high ? 6371000 : 0;
    g_scene.visibility = high ? 140000 : 0;
    g_scene.stemsAbove = high ? 0 : -1;
    g_scene.trailsAbove = 8000;
    g_scene.landAndSea = true;  // where there are coasts: high up, or an airfield's shores
    setCover(e.lat, e.lon, high, std::max(0.0f, e.altFt * 0.3048f));
    g_scene.landingLights = !high;
    g_scene.detailPx = g_detailPx;
    g_scene.terrainEffort = g_terrainEffort;
    g_scene.tinyPx = g_tinyPx;

    s3::Pose shown;
    bool replaying = false;
    s3::Camera camera = cameraFor(g_scene.followed, f, seconds, shown, replaying);
    if (replaying) {
        g_scene.followed.pose = shown;
    }
    // Heard where it really is: through CINEMA's replay of its flare the
    // sound stays with the live roll-out, as a TV replay runs over the live
    // sound - its wheels, reversers and spoilers are the roll-out's, and the
    // replay's pose moves at half speed against the live clock.
    if (g_engineSound) {
        soundEngine(g_inReplay && !g_replay.playing ? engine_noise::Sound()
                                                    : engineSound(f, e, shownKt, look, pose, g_scene.followed.config,
                                                                  camera, dt));
    }
    setHorizon(camera, high);
    s3::prepareBegin(VIEW_W, VIEW_H, g_rows, true, g_scene, camera);
    s3::prepareAircraft(0, 1);
    s3::prepareEnd();
}

// Where on the picture each aircraft in the frame drawn is, and who, for a
// tap to find.
struct TapPoint {
    int16_t x, y;
    char hex[8];
};
TapPoint g_taps[49];
int g_tapCount = 0;
float g_labelledAt = -1;

// The labels for the frame flipped to, about to be drawn.
void placeLabels() {
    static s3::Label anchors[48];
    static labels3d::Candidate candidates[48];
    const std::vector<Info, PsramAllocator<Info>> &info = g_info[g_preparing ^ 1];
    int n = s3::labelAnchors(anchors, 48), count = 0;
    g_tapCount = 0;
    for (int i = 0; i < n; i++) {
        int k = anchors[i].index;
        if (k < 0 || k >= (int)info.size()) {
            continue;
        }
        const Info &in = info[k];
        candidates[count++] = {anchors[i].x, anchors[i].y, anchors[i].sizePx, (uint32_t)strtoul(in.hex, nullptr, 16),
                               in.kind, in.callsign, in.height, in.type};
        TapPoint &t = g_taps[g_tapCount++];
        t = {(int16_t)anchors[i].x, (int16_t)(anchors[i].y + std::min(anchors[i].sizePx / 4, 30.0f)), {}};
        snprintf(t.hex, sizeof t.hex, "%s", in.hex);
    }
    labels3d::Rect keep[1];
    int keepCount = 0;
    float x, y, reach;
    if (s3::followedOnScreen(x, y, reach)) {
        keep[keepCount++] = {(int)(x - reach), (int)(y - reach * 0.45f), (int)(x + reach), (int)(y + reach * 0.45f)};
        TapPoint &t = g_taps[g_tapCount++];
        t = {(int16_t)x, (int16_t)y, {}};
        snprintf(t.hex, sizeof t.hex, "%s", g_followedHex.c_str());
    }
    float now = millis() / 1000.0f, dt = now - g_labelledAt;
    if (g_labelledAt < 0 || dt > 1) {
        g_labeller.reset();
        dt = 0;
    }
    g_labelledAt = now;
    g_placedCount = g_labeller.place(candidates, count, keep, keepCount, VIEW_W, VIEW_H, dt, g_placed, labels3d::MOST);
}

// --- around the picture ---------------------------------------------------------------

bool buttonGreyed(int i) {
    return (i == B_TOWER && g_where != Where::AIRFIELD) || ((i == B_ZOOM_IN || i == B_ZOOM_OUT) && g_cam == Cam::CINEMA) ||
           (i == B_GROUND && !terrainHere()) || (i == B_SOUND && soundMuted());
}

bool buttonLit(int i) {
    return (i == B_CINEMA && g_cam == Cam::CINEMA) || (i == B_CHASE && g_cam == Cam::CHASE) ||
           (i == B_ORBIT && g_cam == Cam::ORBIT) || (i == B_TOWER && g_cam == Cam::TOWER) ||
           (i == B_PAST && g_cam == Cam::PAST) || (i == B_GROUND && g_terrain) || (i == B_SOUND && g_engineSound);
}

// As the radar draws its own.
void drawButtons() {
    auto &c = screen::canvas();
    c.setFont(&fonts::Font0);
    c.setTextSize(2);
    c.setTextDatum(MC_DATUM);
    for (int i = 0; i < BUTTONS; i++) {
        int y = BTN_Y[i];
        bool greyed = buttonGreyed(i), lit = buttonLit(i);
        c.fillRect(BTN_X, y, BTN_W, BTN_H, colorBg);
        if (greyed) {
            c.drawRoundRect(BTN_X, y, BTN_W, BTN_H, 6, colorDisabled);
        } else {
            c.fillRoundRect(BTN_X, y, BTN_W, BTN_H, 6, lit ? colorMuted : colorBtnBg);
        }
        c.setTextColor(greyed ? colorDisabled : lit ? colorBg : colorText);
        c.drawString(BTN_NAME[i], BTN_X + BTN_W / 2, y + BTN_H / 2);
    }
    screen::markDirty(BTN_X, BTN_Y[0], BTN_W, BTN_Y[BUTTONS - 1] + BTN_H - BTN_Y[0]);
}

// The flight: who, how high and fast, the time where it is; and the shot.
// Pushed through the canvas between frames, every half second.
uint32_t g_stripMs = 0;
Where g_stripWhere = Where::NOWHERE;
bool g_stripTerrain = false;

// Its height as the radar's label has it: a pressure altitude, a flight level
// from 18,000ft.
void heightOf(const FollowEstimate &e, char *out, size_t size) {
    int32_t feedFt = feedAltFt(e);
    if (e.ground) {
        snprintf(out, size, "on the ground");
    } else if (feedFt >= 18000) {
        snprintf(out, size, "FL%03d", (int)((feedFt + 50) / 100));
    } else {
        snprintf(out, size, "%d ft", (int)((feedFt + 50) / 100) * 100);
    }
}

// Its speed: taxiing, as the view moves it - see g_taxiKt - or as `e` has
// it; "?" where that is nought for want of one.
void speedOf(const Followed &f, const FollowEstimate &e, char *out, size_t size) {
    if (g_taxiKt >= 0 || speedKnown(f)) {
        snprintf(out, size, "%d", g_taxiKt >= 0 ? g_taxiKt : e.gsKt);
    } else {
        snprintf(out, size, "?");
    }
}

void drawFlightStrip(const Followed &f) {
    auto &c = screen::canvas();
    c.fillRect(VIEW_X, STRIP_Y, VIEW_W, STRIP_H, colorBg);
    c.setFont(&fonts::Font0);
    c.setTextDatum(TL_DATUM);
    String call = f.callsign;
    call.trim();
    String who = (call.length() ? call : f.hex) + "   " + f.type;
    c.setTextSize(3);
    c.setTextColor(colorText);
    c.drawString(who, VIEW_X, STRIP_Y + 4);
    FollowEstimate e = estimateOf(f);
    char height[16], speed[12], clock[16] = "", line[96];
    heightOf(e, height, sizeof height);
    speedOf(f, e, speed, sizeof speed);
    time_t now = time(nullptr);
    if (now > 1600000000) {
        timeZoneFormat(now, timeZoneAt(e.lat, e.lon), clock, sizeof clock);
    }
    snprintf(line, sizeof line, "%s   %s kt   %+d fpm   %s", height, speed, e.ground ? 0 : e.vsFpm, clock);
    c.setTextSize(2);
    c.setTextColor(colorMuted);
    c.drawString(line, VIEW_X, STRIP_Y + 44);
    const char *camera = g_cam == Cam::CINEMA  ? "CINEMA"
                         : g_cam == Cam::CHASE ? "CHASE"
                         : g_cam == Cam::ORBIT ? "ORBIT"
                         : g_cam == Cam::TOWER ? "TOWER"
                                               : "FLY PAST";
    // And its shot, where it has one of its own - CINEMA's: the others' are
    // their names again, "TOWER - tower".
    char shot[40];
    if (strcasecmp(camera, g_shot) == 0) {
        snprintf(shot, sizeof shot, "%s", camera);
    } else {
        snprintf(shot, sizeof shot, "%s - %s", camera, g_shot);
    }
    if (g_where == Where::AIRFIELD) {
        snprintf(line, sizeof line, "%s     at %s", shot, g_airport);
    } else {
        snprintf(line, sizeof line, "%s     high up", shot);
    }
    if (f.note.length()) {
        size_t used = strlen(line);
        snprintf(line + used, sizeof line - used, "     %s", f.note.c_str());
    }
    c.setTextColor(colorFaint);
    c.drawString(line, VIEW_X, STRIP_Y + 72);
    // An airport drawn from its runways alone, or the land flat, and why.
    // The two on the one line there is room for: one sentence where neither
    // pack is there, or no card; else in turn, four seconds each.
    using M = airfield::Missing;
    using S = groundpack::Status;
    const char *basic = g_where == Where::AIRFIELD && g_missing != M::NOTHING ? basicWhy() : nullptr;
    const char *flat = flatWhy(), *note = basic != nullptr ? basic : flat;
    if (basic != nullptr && flat != nullptr) {
        S ground = groundpack::status();
        if (g_missing == M::NO_CARD && ground == S::NO_CARD) {
            note = "Basic view and flat land - no SD card in for airfields.bin and ground.bin";
        } else if (g_missing == M::NO_PACK && ground == S::NO_PACK) {
            note = "Basic view and flat land - no airfields.bin or ground.bin on the SD card";
        } else if (millis() / 4000 % 2 == 1) {
            note = flat;
        }
    }
    if (note != nullptr) {
        c.setTextColor(colorNote);
        c.drawString(note, VIEW_X, STRIP_Y + 98);
    }
    screen::markDirty(VIEW_X, STRIP_Y, VIEW_W, STRIP_H);
}

// A replay's: the flight on a line - who, how high and fast, and when, as
// the recording's clock has it - and under it the controls as the replay
// screen has them: Play or Pause, back a minute, the speed, on a minute, and
// the bar of how far through it is, with the time into it under that.
constexpr int RB_Y = STRIP_Y + 30, RB_H = 54;
constexpr int RB_PLAY_X = VIEW_X, RB_PLAY_W = 150, RB_STEP_W = 96, RB_GAP = 10;
constexpr int RB_BACK_X = RB_PLAY_X + RB_PLAY_W + RB_GAP;
constexpr int RB_SPEED_X = RB_BACK_X + RB_STEP_W + RB_GAP;
constexpr int RB_ON_X = RB_SPEED_X + RB_STEP_W + RB_GAP;
constexpr int RB_BAR_X = RB_ON_X + RB_STEP_W + 24, RB_BAR_W = VIEW_X + VIEW_W - RB_BAR_X;
constexpr int RB_BAR_Y = RB_Y + 4, RB_BAR_H = 30;
float g_seekTo = 0;  // see seekTo()

void controlButton(int x, int w, const char *label, bool lit) {
    auto &c = screen::canvas();
    c.fillRoundRect(x, RB_Y, w, RB_H, 6, lit ? colorMuted : colorBtnBg);
    c.setTextColor(lit ? colorBg : colorText);
    c.drawString(label, x + w / 2, RB_Y + RB_H / 2);
}

String timeText(uint32_t ms) {
    uint32_t s = ms / 1000;
    char out[16];
    if (s >= 3600) {
        snprintf(out, sizeof out, "%lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60),
                 (unsigned long)(s % 60));
    } else {
        snprintf(out, sizeof out, "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
    }
    return out;
}

void drawReplayStrip(const Followed &f) {
    auto &c = screen::canvas();
    c.fillRect(VIEW_X, STRIP_Y, VIEW_W, STRIP_H, colorBg);
    c.setFont(&fonts::Font0);
    FollowEstimate e = estimateOf(f);
    String call = f.callsign;
    call.trim();
    char height[16], knots[12], when[16] = "", line[128];
    heightOf(e, height, sizeof height);
    speedOf(f, e, knots, sizeof knots);
    if (g_replay.epoch != 0) {
        time_t t = g_replay.epoch;
        struct tm tm;
        gmtime_r(&t, &tm);
        strftime(when, sizeof when, "%H:%M:%S UTC", &tm);
    }
    snprintf(line, sizeof line, "%s  %s   %s   %s kt   %s   %s%s", call.length() ? call.c_str() : f.hex.c_str(),
             f.type.c_str(), height, knots, when, g_where == Where::AIRFIELD ? "at " : "high up",
             g_where == Where::AIRFIELD ? g_airport : "");
    c.setTextDatum(TL_DATUM);
    c.setTextSize(2);
    c.setTextColor(colorText);
    c.drawString(line, VIEW_X, STRIP_Y + 6);

    c.setTextDatum(MC_DATUM);
    bool atEnd = g_replay.t >= g_replay.durationMs;
    controlButton(RB_PLAY_X, RB_PLAY_W, g_replay.playing ? "Pause" : atEnd ? "Replay" : "Play", !g_replay.playing);
    controlButton(RB_BACK_X, RB_STEP_W, "-1m", false);
    char speed[8];
    snprintf(speed, sizeof speed, "%dx", g_replay.speed);
    controlButton(RB_SPEED_X, RB_STEP_W, speed, false);
    controlButton(RB_ON_X, RB_STEP_W, "+1m", false);

    uint32_t dur = g_replay.durationMs;
    c.fillRoundRect(RB_BAR_X, RB_BAR_Y, RB_BAR_W, RB_BAR_H, 6, colorBtnBg);
    int filled = dur ? (int)((uint64_t)RB_BAR_W * std::min(g_replay.t, dur) / dur) : 0;
    if (filled > 0) {
        c.fillRoundRect(RB_BAR_X, RB_BAR_Y, std::max(filled, 12), RB_BAR_H, 6, colorMuted);
    }
    c.fillRect(RB_BAR_X + std::min(filled, RB_BAR_W - 4), RB_BAR_Y - 4, 4, RB_BAR_H + 8, colorText);
    c.setTextDatum(TL_DATUM);
    c.setTextColor(colorFaint);
    c.drawString(timeText(g_replay.t) + " / " + timeText(dur), RB_BAR_X, RB_BAR_Y + RB_BAR_H + 10);
    screen::markDirty(VIEW_X, STRIP_Y, VIEW_W, STRIP_H);
}

void drawStrip(const Followed &f) {
    if (g_inReplay) {
        drawReplayStrip(f);
    } else {
        drawFlightStrip(f);
    }
    // TOWER greys and lights with where it is, GROUND with whether the pack
    // has land here.
    bool terrain = terrainHere();
    if (g_stripWhere != g_where || g_stripTerrain != terrain) {
        g_stripWhere = g_where;
        g_stripTerrain = terrain;
        if (g_cam == Cam::TOWER && g_where != Where::AIRFIELD) {
            g_cam = Cam::CINEMA;
        }
        drawButtons();
    }
    screen::flush();
}

// --- touch ----------------------------------------------------------------------------

bool g_onView = false, g_dragged = false;
int g_downX = 0, g_downY = 0, g_lastX = 0, g_lastY = 0;

Action press(int b) {
    if (b == B_BACK) {
        return Action::BACK;
    }
    if (buttonGreyed(b)) {
        return Action::NONE;
    }
    switch (b) {
    case B_CINEMA: g_cam = Cam::CINEMA; break;
    case B_CHASE: g_cam = Cam::CHASE; break;
    case B_ORBIT: g_cam = Cam::ORBIT; break;
    case B_TOWER: g_cam = Cam::TOWER; break;
    case B_PAST:
        // Set down ahead afresh, on its left.
        g_cam = Cam::PAST;
        g_passSet = false;
        g_passSide = 1;
        break;
    case B_ZOOM_IN:
    case B_ZOOM_OUT:
        g_zoom = b == B_ZOOM_IN ? std::min(4.0f, g_zoom * 1.4f) : std::max(0.3f, g_zoom / 1.4f);
        // Flying past, nearer or further off at once, the same side.
        if (g_cam == Cam::PAST && g_passSet) {
            g_passSet = false;
            g_passKeepSide = true;
        }
        break;
    case B_GROUND:
        g_terrain = !g_terrain;
        saveTerrain(g_terrain);
        Serial.printf("[3d] the ground %s\n", g_terrain ? "at its height" : "flat, its hills a skyline");
        break;
    case B_SOUND:
        g_engineSound = !g_engineSound;
        saveEngineSound(g_engineSound);
        if (!g_engineSound) {
            soundEngine(engine_noise::Sound());
        }
        Serial.printf("[3d] engine noise %s\n", g_engineSound ? "on" : "off");
        break;
    default: break;
    }
    drawButtons();
    screen::flush();
    g_stripMs = 0;  // the shot named again at once
    return Action::NONE;
}

}  // namespace

#ifdef BENCH3D
int g_forceRows = 0;  // see benchBandRows()
#endif

#ifdef BENCH3D
bool benchToggleCover() {
    g_benchCoverOff = !g_benchCoverOff;
    return !g_benchCoverOff;
}

void benchCoverReport() {
    static uint32_t lastMs = 0;
    uint64_t c[11];
    s3::coverStats(c, true);
    uint32_t nowMs = millis();
    float seconds = lastMs != 0 ? (nowMs - lastMs) / 1000.0f : 0;
    lastMs = nowMs;
    if (seconds <= 0) {
        Serial.println("[ground] the cover's time, from now");
        return;
    }
    float perMs = getCpuFrequencyMhz() * 1000.0f * seconds;  // cycles a millisecond of each second
    Serial.printf("[ground] the cover, ms a second of both cores': cells read %.1f (%.0f rows a second, %.1f cycles a "
                  "cell, %.1f of them the innermost loop's), colours %.1f (%.0f worked out a second), pixels written "
                  "%.1f; preparing: rows' places %.1f, the skyline %.1f\n",
                  c[0] / perMs, c[3] / seconds, c[0] / (float)std::max<uint64_t>(1, c[6]),
                  c[5] / (float)std::max<uint64_t>(1, c[6]), c[1] / perMs, c[4] / seconds, c[2] / perMs, c[7] / perMs,
                  c[10] / perMs);
}
#endif

bool open() {
    groundpack::open();  // what covers the land, if the card has it: read in the background
    if (g_open) {
        return true;
    }
    colors();
    // Never bands of 4: the PPA has been seen never to finish a block that
    // shallow, and every band would be one. 15 and 14 between 16 and 12: at
    // 1008 wide a band of 16 is 31.5KB, and with the chip's memory broken up
    // the view went to 12 at once.
    //
    // The bands, and the loop's own buffers for its share of the drawing - a
    // depth buffer and two colour buffers, or one: the tallest bands they all
    // fit in at once, or failing that the tallest the bands alone do and the
    // helper drawing it all. The loop's help is worth more than the bands'
    // height - 60fps rather than 50 - and with the memory in pieces the
    // tallest bands had left it none.
    //
    // All where DMA reaches, the depth buffers too, though no DMA touches
    // them: of the chip's own memory that leaves out only the 32KB of the
    // low-power core's, which the heap has too - and a band of 15 fits
    // there. A depth buffer was put there with the rest of the memory in
    // pieces, and the first store to it, eight pixels at once, faulted. Not
    // in PSRAM: drawing there, through the same small cache as the helper's
    // reading, made the frames slower than no help at all.
    auto block = [](int rows) { return heap_caps_aligned_alloc(64, pipeBytes(rows), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA); };
    // As one piece, the six, where there is one - before anything else, a
    // band shallower if need be: small blocks put in the gaps between six
    // pieces - their alignment's - stayed when they went, and the chip's
    // memory came back to the next open in pieces just short of a band.
    // Otherwise each its own, the bands and the loop's; or the bands alone.
    auto take = [&](int rows, int how) {
        size_t each = (pipeBytes(rows) + 63) & ~(size_t)63;
        if (how == 0) {
            g_arena = heap_caps_aligned_alloc(64, each * 6, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            if (g_arena == nullptr) {
                return false;
            }
            uint8_t *at = (uint8_t *)g_arena;
            for (void *&p : g_pipe) {
                p = at;
                at += each;
            }
            g_shareDepth = at;
            g_share[0].color = at + each;
            g_share[1].color = at + 2 * each;
            return true;
        }
        bool help = how == 1, all = true;
        for (void *&p : g_pipe) {
            p = block(rows);
            all = all && p != nullptr;
        }
        if (all && help) {
            g_shareDepth = block(rows);
            g_share[0].color = g_shareDepth != nullptr ? block(rows) : nullptr;
            g_share[1].color = g_share[0].color != nullptr ? block(rows) : nullptr;
            all = g_share[0].color != nullptr;
        }
        if (!all) {
            releaseBands();
        }
        return all;
    };
    // In the order they are worth having: deep bands in one piece, then in
    // pieces, then shallower - a band of 8 is twice the pushes to the panel
    // of one of 16 - and the bands without the loop's help last of all.
    static const struct {
        uint8_t how, rows;
    } TRIES[] = {{0, 16}, {0, 15}, {0, 14}, {1, 16}, {1, 15}, {1, 14}, {0, 12}, {1, 12}, {0, 10},
                 {1, 10}, {0, 8},  {1, 8},  {2, 16}, {2, 15}, {2, 14}, {2, 12}, {2, 10}, {2, 8}};
    g_rows = 0;
    for (const auto &t : TRIES) {
#ifdef BENCH3D
        if (g_forceRows != 0 && t.rows != g_forceRows) {
            continue;
        }
#endif
        if (take(t.rows, t.how)) {
            g_rows = t.rows;
            break;
        }
    }
    if (g_pipe[0] == nullptr) {
        Serial.printf("[3d] no room for the bands: %uKB free, the largest %uKB\n",
                      (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                      (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
        return false;
    }
    for (Shared &sh : g_share) {
        sh.state.store(BAND_FREE);
    }
    if (g_helper == nullptr) {
        g_done = xSemaphoreCreateBinary();
        g_idle0Ms = millis();
        esp_register_freertos_idle_hook_for_cpu(idle0Ran, 0);
        xTaskCreatePinnedToCore(helperTask, "view3d", 6144, nullptr, 3, &g_helper, 0);
    }
    s3::setSolidsReady([] { xTaskNotify(g_helper, WAKE_SOLIDS, eSetBits); });
    if (g_director == nullptr) {
        g_director = new s3::Director();
    }
    s3::setClock([] { return (uint32_t)micros(); });
    s3::buildModels();
    g_dim = __builtin_bswap16(M5.Display.color565(0x3D, 0xE8, 0x7C));  // the radar's blip green
    g_open = true;
    g_drawing = false;
    g_prepared = false;
    g_haveNow = false;
    g_seconds = 0;
    g_lastSeconds = -1;
    g_spotSet = g_spotAway = false;
    g_engineDistance = -1;
    // The view's clock starts again: what was timed by it, forgotten.
    g_touchdownAt = -100;
    g_parkedAt = -1;
    g_movedOnGround = false;
    g_startedAt = -1000;
    g_runningWas = -1;
    g_rotateHeldAt = -1;
    g_passSet = false;
    g_towerRaise = 0;
    g_labelledAt = -1;
    g_stripMs = 0;
    g_frameMs = 0;
    g_whereMs = 0;
    g_stripWhere = Where::NOWHERE;
    screen::setLiveRect(VIEW_X, VIEW_Y, VIEW_W, VIEW_H);
    redraw();
    Serial.printf("[3d] open, %d-row bands: internal RAM %uKB free, the largest block %uKB; the loop drawing "
                  "into %d buffers of its own\n",
                  g_rows, (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
                  (g_share[0].color != nullptr) + (g_share[1].color != nullptr));
    return true;
}

void close() {
    if (!g_open) {
        return;
    }
    finishDrawing();
    s3::setSolidsReady(nullptr);
    soundEngine(engine_noise::Sound());
    g_open = false;
    g_prepared = false;
    screen::setLiveRect(0, 0, 0, 0);
    releaseBands();
    Serial.println("[3d] closed");
}

bool isOpen() { return g_open; }

void setEngineSound(bool on) { g_engineSound = on; }

void setTerrain(bool on) { g_terrain = on; }

bool parkingLost() { return g_open && g_follower.active() && g_follower.parking(); }

const char *airport() { return g_open && g_where == Where::AIRFIELD ? g_airport : ""; }

bool atStand(double lat, double lon) {
    if (!g_open || g_where != Where::AIRFIELD || g_network.empty()) {
        return false;
    }
    Vec3 p = local(lat, lon);
    for (const ground::StandWay &st : g_network.stands()) {
        if (st.ok && hypotf(p.x - st.line.back().x, p.y - st.line.back().y) < AT_STAND_M) {
            return true;
        }
    }
    return false;
}

void redraw() {
    colors();
    screen::clear(colorBg);
    drawButtons();
    g_stripMs = 0;
    screen::flush();
}

void frame(const Followed &flight, const std::vector<Aircraft> &traffic, uint32_t polledMs, uint32_t nowMs) {
    uint32_t w0 = micros();
    if (!g_open || !finishDrawing()) {
        return;
    }
    g_statWaitUs += micros() - w0;
    g_statDrawUs += g_drawUs;
    // The view's seconds run on with the world's clock, and a jump starts
    // everything afresh, as a new flight.
    int32_t step = g_haveNow ? (int32_t)(nowMs - g_nowMs) : 0;
    bool jumped = g_haveNow && (step < 0 || step > (int32_t)JUMP_MS);
    g_nowMs = nowMs;
    g_haveNow = true;
    if (!jumped) {
        g_seconds += step / 1000.0f;
    }
    // A new flight: its director, its pilot and its labels from the start.
    if (flight.hex != g_followedHex || jumped) {
        g_followedHex = flight.hex;
        delete g_director;
        g_director = new s3::Director();
        g_pilot = s3::Pilot();
        g_lastTrack = (90 - flight.now.track) * DEG;
        g_turn = 0;
        g_haveShown = false;
        g_follower.end();
        g_taxiSaid[0] = 0;
        g_touchdownAt = -100;
        g_parkedAt = -1;
        g_movedOnGround = false;
        g_startedAt = -1000;
        g_runningWas = -1;
        g_rotateHeldAt = -1;
        g_labeller.reset();
    }
    // The governor: by how long the last frame took, all told.
    uint32_t now = millis();
    if (g_frameMs != 0) {
        uint32_t took = now - g_frameMs;
        if (took > 33) {
            g_detailPx = std::min(400.0f, g_detailPx * 1.25f);
            g_tinyPx = std::min(200.0f, g_tinyPx * 1.25f);
        } else if (took < 25) {
            g_detailPx = std::max(16.0f, g_detailPx / 1.1f);
            g_tinyPx = std::max(5.0f, g_tinyPx / 1.1f);
        }
        // The land skimps from 27ms a frame - under 37 a second - and has its
        // detail back, slowly, under 23: its far hills' outlines changing
        // little from one frame to the next.
#ifdef BENCH3D
        if (g_benchFullLand) {
            g_terrainEffort = 0;  // the bench's stills: the land in full, however long the frames
        } else
#endif
        if (took > 27) {
            g_terrainEffort = std::min(1.0f, g_terrainEffort + 0.04f);
        } else if (took < 23) {
            g_terrainEffort = std::max(0.0f, g_terrainEffort - 0.01f);
        }
    }
    g_frameMs = now;
    // A frame that took long to draw, said with what was in it and where the
    // camera was - the frame just drawn, before the next is handed over.
    if (g_prepared && g_drawUs > 250000) {
        s3::Counts c = s3::counts();
        Serial.printf("[3d] slow frame: %ums drawing; %d triangles, %d lines, %d lights, %d coast crossings; "
                      "aircraft %d full %d simple %d tiny; %s, azimuth %.0f elevation %.0f zoom %.2f, %s\n",
                      (unsigned)(g_drawUs / 1000), c.triangles, c.lines, c.lights, c.crossings, c.aircraftFull,
                      c.aircraftSimple, c.aircraftTiny, g_shot, g_azimuth, g_elevation, g_zoom,
                      g_where == Where::AIRFIELD ? g_airport : "high up");
        g_drawUs = 0;
    }
    if (g_prepared) {
        s3::flip();
        g_preparing ^= 1;
        uint32_t l0 = micros();
        placeLabels();
        g_statLabelsUs += micros() - l0;
        if (now - g_stripMs >= 500) {
            g_stripMs = now;
            drawStrip(flight);
        }
        g_drawing = true;
        int bands = s3::bandCount();
        g_bandLimit = VIEW_H - (bands - 1) * g_rows < g_rows ? bands - 2 : bands;
        g_bands.store(bandsFrom(0, g_bandLimit));
        xTaskNotify(g_helper, WAKE_DRAW, eSetBits);
    }
    uint32_t p0 = micros();
    prepareNext(flight, traffic, polledMs);
    g_statPrepUs += micros() - p0;
    g_prepared = true;
    g_statFrames++;
    {
        s3::Counts c = s3::counts();
        for (int i = 0; i < s3::STEPS; i++) {
            g_statStepUs[i] += c.stepUs[i];
        }
        g_statTriangles += c.triangles;
        g_statLines += c.lines;
    }
    if (now - g_statFromMs >= 10000) {
        if (g_statFromMs != 0) {
            float n = std::max<uint32_t>(1, g_statFrames);
            Serial.printf("[3d] %.1f fps; preparing %.1fms, drawing %.1fms, waiting for the drawing %.1fms; %d other "
                          "aircraft, %s; internal RAM %uKB free; core 0 idle at most %ums apart\n",
                          g_statFrames * 1000.0f / (now - g_statFromMs), g_statPrepUs / 1000.0f / n,
                          g_statDrawUs / 1000.0f / n, g_statWaitUs / 1000.0f / n, (int)g_traffic.size(), g_shot,
                          (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), (unsigned)g_idle0GapMs);
            // The preparing's costliest steps, a frame on average.
            int top[4] = {-1, -1, -1, -1};
            for (int i = 0; i < s3::STEPS; i++) {
                for (int k = 0; k < 4; k++) {
                    if (top[k] < 0 || g_statStepUs[i] > g_statStepUs[top[k]]) {
                        for (int j = 3; j > k; j--) {
                            top[j] = top[j - 1];
                        }
                        top[k] = i;
                        break;
                    }
                }
            }
            Serial.printf("[3d]   %u triangles, %u lines a frame; most of the preparing: %s %.1fms, %s %.1fms, %s %.1fms, "
                          "%s %.1fms; the traffic placed %.1fms, labelled %.1fms; the solids on the drawing core in "
                          "%u%% of frames, its stack %u spare\n",
                          (unsigned)(g_statTriangles / n), (unsigned)(g_statLines / n), s3::stepName(top[0]),
                          g_statStepUs[top[0]] / 1000.0f / n, s3::stepName(top[1]), g_statStepUs[top[1]] / 1000.0f / n,
                          s3::stepName(top[2]), g_statStepUs[top[2]] / 1000.0f / n, s3::stepName(top[3]),
                          g_statStepUs[top[3]] / 1000.0f / n, g_statTrafficUs / 1000.0f / n, g_statLabelsUs / 1000.0f / n,
                          (unsigned)((g_solidsAside - g_statSolidsAsideFrom) * 100 / n),
                          (unsigned)uxTaskGetStackHighWaterMark(g_helper));
            // And the drawing's, on the other core.
            // (both cores' together)
            float stage[6];
            for (int i = 0; i < 6; i++) {
                stage[i] = (g_drawStageUs[i] + g_loopStageUs[i] - g_statStageFrom[i]) / 1000.0f / n;
            }
            Serial.printf("[3d]   drawing: sky %.1fms, ground %.1fms, shadows %.1fms, depth %.1fms, solids %.1fms, "
                          "lines and lights %.1fms; waiting for the PPA %.1fms; %u%% of the bands drawn by the loop\n",
                          stage[0], stage[1], stage[2], stage[3], stage[4], stage[5],
                          (g_pushWaitUs - g_statPushWaitFrom) / 1000.0f / n,
                          (unsigned)((g_loopBands - g_statLoopBandsFrom) * 100 / (n * s3::bandCount())));
            // And the land at its height's, a frame's: the march's and the
            // rows' cycles, and what they did.
            uint64_t t[8];
            s3::terrainStats(t, true);
            if (t[1] > 0 || t[5] > 0) {
                Serial.printf("[3d]   terrain, a frame: the march %.1fms - %u samples, %.0f cycles each, %u rows "
                              "filled, %u blocks skipped; the drawing %.1fms - %u samples, %.0f cycles each, %u cells "
                              "the long way, %u colours; skimping %.2f\n",
                              t[0] / (getCpuFrequencyMhz() * 1000.0f) / n, (unsigned)(t[1] / n),
                              t[0] / (float)std::max<uint64_t>(1, t[1]), (unsigned)(t[2] / n), (unsigned)(t[3] / n),
                              t[4] / (getCpuFrequencyMhz() * 1000.0f) / n, (unsigned)(t[5] / n),
                              t[4] / (float)std::max<uint64_t>(1, t[5]), (unsigned)(t[6] / n), (unsigned)(t[7] / n),
                              g_terrainEffort);
            }
            uint64_t q[8];
            s3::terrainCache(q, true);
            if (t[1] > 0 && (q[0] + q[1]) > 0) {
                Serial.printf("[3d]   terrain, the caches a sample: the march's L1 %.2f misses of %.1f reads, L2 %.2f "
                              "misses; the drawing's L1 %.2f of %.1f, L2 %.2f\n",
                              q[1] / (float)t[1], (q[0] + q[1]) / (float)t[1], q[3] / (float)t[1],
                              q[5] / (float)std::max<uint64_t>(1, t[5]), (q[4] + q[5]) / (float)std::max<uint64_t>(1, t[5]),
                              q[7] / (float)std::max<uint64_t>(1, t[5]));
            }
        }
        g_idle0GapMs = 0;
        memset(g_statStepUs, 0, sizeof g_statStepUs);
        g_statTriangles = g_statLines = 0;
        g_statTrafficUs = g_statLabelsUs = 0;
        g_statFromMs = now;
        g_statFrames = g_statPrepUs = g_statWaitUs = g_statDrawUs = 0;
        g_statSolidsAsideFrom = g_solidsAside;
        for (int i = 0; i < 6; i++) {
            g_statStageFrom[i] = g_drawStageUs[i] + g_loopStageUs[i];
        }
        g_statPushWaitFrom = g_pushWaitUs;
        g_statLoopBandsFrom = g_loopBands;
    }
}

Action touch(int x, int y, bool landed) {
    if (landed && g_inReplay && y >= RB_Y - 8 && y < RB_Y + RB_H + 24 && x >= VIEW_X - 16 &&
        x < VIEW_X + VIEW_W + 16) {
        g_onView = false;
        g_stripMs = 0;  // shown as it is now at the next frame
        if (x < RB_PLAY_X + RB_PLAY_W + RB_GAP / 2) {
            return Action::PLAY;
        }
        if (x < RB_BACK_X + RB_STEP_W + RB_GAP / 2) {
            return Action::BACK_MINUTE;
        }
        if (x < RB_SPEED_X + RB_STEP_W + RB_GAP / 2) {
            return Action::SPEED;
        }
        if (x < RB_ON_X + RB_STEP_W + RB_GAP / 2) {
            return Action::ON_MINUTE;
        }
        if (x < RB_BAR_X - RB_GAP / 2) {
            return Action::NONE;  // between them: not the start of the bar
        }
        g_seekTo = std::max(0.0f, std::min(1.0f, (x - RB_BAR_X) / (float)RB_BAR_W));
        return Action::SEEK;
    }
    if (landed) {
        g_onView = false;
        for (int i = 0; i < BUTTONS; i++) {
            if (x >= BTN_X - 16 && x < BTN_X + BTN_W + 16 && y >= BTN_Y[i] - 5 && y < BTN_Y[i] + BTN_H + 5) {
                return press(i);
            }
        }
        if (x >= VIEW_X && x < VIEW_X + VIEW_W && y >= VIEW_Y && y < VIEW_Y + VIEW_H) {
            g_onView = true;
            g_dragged = false;
            g_downX = g_lastX = x;
            g_downY = g_lastY = y;
        }
        return Action::NONE;
    }
    if (!g_onView) {
        return Action::NONE;
    }
    // Moved across the picture: the camera round the aircraft with it.
    if (!g_dragged && abs(x - g_downX) + abs(y - g_downY) > 12) {
        g_dragged = true;
        if (g_cam != Cam::ORBIT) {
            g_cam = Cam::ORBIT;
            drawButtons();
            screen::flush();
            g_stripMs = 0;
        }
    }
    if (g_dragged) {
        g_azimuth -= (x - g_lastX) * 0.4f;
        g_elevation = std::max(-10.0f, std::min(80.0f, g_elevation + (y - g_lastY) * 0.25f));
    }
    g_lastX = x;
    g_lastY = y;
    return Action::NONE;
}

Action lift(String &hex) {
    bool tap = g_onView && !g_dragged;
    g_onView = false;
    if (!tap) {
        return Action::NONE;
    }
    int x = g_downX - VIEW_X, y = g_downY - VIEW_Y, best = 48 * 48;
    const TapPoint *found = nullptr;
    for (int i = 0; i < g_tapCount; i++) {
        int dx = g_taps[i].x - x, dy = g_taps[i].y - y;
        if (dx * dx + dy * dy < best) {
            best = dx * dx + dy * dy;
            found = &g_taps[i];
        }
    }
    if (found == nullptr) {
        return Action::NONE;
    }
    hex = found->hex;
    return Action::DETAIL;
}

void setReplay(const ReplayBar *bar) {
    if ((bar != nullptr) != g_inReplay ||
        (bar != nullptr && (bar->playing != g_replay.playing || bar->speed != g_replay.speed))) {
        g_stripMs = 0;  // shown as it is now at the next frame
    }
    g_inReplay = bar != nullptr;
    if (bar != nullptr) {
        g_replay = *bar;
    }
}

void setDayPressure(DayPressureFn dayPressure) {
    g_dayPressure = dayPressure != nullptr ? dayPressure : dayPressureFt;
    g_polledMs = 0;  // the traffic's samples taken again
}

float seekTo() { return g_seekTo; }

String busy() {
    if (g_step == Step::IDLE) {
        return g_drawing ? "idle, the frame's end not yet taken" : "idle";
    }
    char out[96];
    snprintf(out, sizeof out, "%s band %d of %d", STEP_NAMES[(int)g_step], g_stepBand, s3::bandCount());
    return out;
}

#ifdef BENCH3D
void sweep(uint32_t seconds) {
    if (!g_open) {
        return;
    }
    g_cam = Cam::ORBIT;
    g_sweepUntilMs = (millis() + seconds * 1000) | 1;
    g_sweepSaidMs = 0;
    drawButtons();
    screen::flush();
    Serial.printf("[3d] sweeping the orbit camera for %us\n", (unsigned)seconds);
}

void toggleTerrain() {
    g_terrain = !g_terrain;
    Serial.printf("[3d] the land %s\n", g_terrain ? "at its height, its hills its own" : "flat, the hills a skyline");
    if (g_open) {
        drawButtons();
        screen::flush();
    }
}

void terrainProbe(int which) {
    s3::terrainProbe(which);
    Serial.printf("[3d] terrain probe %d\n", which);
}

void dumpSkyline() {
    Vec3 eye, right, down, forward;
    float f, cx, cy;
    const int16_t *top, *foot;
    int width;
    bool on = s3::skylineDrawn(eye, right, down, forward, f, cx, cy, top, foot, width);
    Serial.printf("[3d] skyline: eye %.6f %.6f %.1fm; right %.5f %.5f %.5f down %.5f %.5f %.5f ahead %.5f %.5f %.5f; "
                  "focal %.2f middle %.1f %.1f; north turned %.5f; %d wide; hills %s\n",
                  g_eyeLat, g_eyeLon, g_scene.eyeHeightM, right.x, right.y, right.z, down.x, down.y, down.z, forward.x,
                  forward.y, forward.z, f, cx, cy, g_scene.horizonTurn, width, on ? "on" : "off");
    if (!on) {
        return;
    }
    // Each fourth column's top and foot, the foot where none: top >= foot.
    String line = "[3d] skyline rows:";
    for (int x = 0; x < width; x += 4) {
        line += ' ';
        line += top[x];
        line += '/';
        line += foot[x];
    }
    Serial.println(line);
}

void benchPress(char what) {
    if (what == '0' || what == '1') {
        // The land in full for a still, however long the frames take - or
        // governed again. Kept with the view shut too.
        g_benchFullLand = what == '0';
        Serial.printf("[3d] the land %s\n", g_benchFullLand ? "in full, not skimped" : "governed again");
        return;
    }
    if (!g_open) {
        return;
    }
    int b = what == 'c' ? B_CINEMA : what == 'h' ? B_CHASE : what == 'o' ? B_ORBIT : what == 't' ? B_TOWER
          : what == '+' ? B_ZOOM_IN : what == '-' ? B_ZOOM_OUT : what == 's' ? B_SOUND : what == 'f' ? B_PAST
                                                                                                         : -1;
    if (b >= 0) {
        g_sweepUntilMs = 0;
        press(b);
        Serial.printf("[3d] pressed %s%s\n", BTN_NAME[b], buttonGreyed(b) ? ", greyed" : "");
    }
}

void benchBandRows(int rows) {
    g_forceRows = rows;
    Serial.printf("[3d] bands of %d rows from the next open\n", rows);
}

void benchSolids() {
    if (!g_open || !finishDrawing()) {
        return;
    }
    s3::SolidsBench b = s3::benchSolids();
    Serial.printf("[3d] the solids, a time: %u looked at, %u facing; their facing %uus, coloured %uus, placed and "
                  "projected %uus, set up %uus, kept %uus\n",
                  (unsigned)b.tested, (unsigned)b.facing, (unsigned)b.us[0], (unsigned)b.us[1], (unsigned)b.us[2],
                  (unsigned)b.us[3], (unsigned)b.us[4]);
    Serial.printf("[3d]   the ground's %u triangles read %uus, its step %uus\n", (unsigned)b.groundTris,
                  (unsigned)b.groundReadUs, (unsigned)b.groundUs);
    Serial.printf("[3d]   partly seen cells' triangles: %u, %u wholly outside a plane, %u wholly inside, %u cut\n",
                  (unsigned)b.seen[0], (unsigned)b.seen[1], (unsigned)b.seen[2], (unsigned)b.seen[3]);
}

int benchStands(double *lat, double *lon, int max) {
    int n = std::min(max, g_benchStands.load());
    for (int i = 0; i < n; i++) {
        lat[i] = g_benchStandLat[i];
        lon[i] = g_benchStandLon[i];
    }
    return n;
}

int benchWays(double *lat, double *lon, float *track, int max) {
    int n = std::min(max, g_benchWays.load());
    for (int i = 0; i < n; i++) {
        lat[i] = g_benchWayLat[i];
        lon[i] = g_benchWayLon[i];
        track[i] = g_benchWayTrack[i];
    }
    return n;
}

void benchSun(int which) {
    g_benchSun = which;
    Serial.printf("[3d] the sun %s\n", which == 1 ? "as by day" : which == 2 ? "as at dusk" : which == 3 ? "as by night" : "as it is");
}

bool benchTap(String &hex) {
    if (!g_open) {
        return false;
    }
    int pick = -1;
    for (int i = 0; i < g_tapCount; i++) {
        if (pick < 0 || g_followedHex == g_taps[pick].hex) {
            pick = i;
        }
    }
    if (pick < 0) {
        return false;
    }
    hex = g_taps[pick].hex;
    return true;
}
#endif

}  // namespace view3d
