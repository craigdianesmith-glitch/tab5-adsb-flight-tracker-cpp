// Soak-tests the 3D view's landing on the PC, end to end: a made-up arrival's
// reports, as the feeds give them, into the real runway simulation
// (src/follow.cpp), main.cpp's Followed made from it each frame as
// followed3d() makes it - f.now, f.report, f.reportMs, f.landed,
// f.simulating, f.movement, f.runway, f.lost - and the view's own placement:
// prepareNext() from its first line to "int shownKt", and the block from
// "constexpr float STILL_KT" to the end of estimateOf(), cut out of
// src/view3d.cpp by build.sh each time it builds, not copied, so a change
// there is what is tested. That is the carry-and-ease, "rolling", and the
// hand-over to and from the real ground::Follower (src/ground_follow.cpp,
// ground_taxi.cpp, ground_route.cpp) on the airfield's real network, read
// from airfields.bin as tools/ground_soak reads it and built as
// buildAirfield() builds it.
//
// Each frame (1/30 s, or uneven with --jitter) what is shown is checked for
// what would look wrong:
//
//   back3s     going backwards - more than 1m of steps against the shown
//              heading within 3 s, the ground soak's measure
//   disp3s     moved back along its shown heading by more than 1m over 3 s
//   tailfirst  pointing more than 120 degrees from the way it moved over the
//              last half second, faster than 1m/s - pushed back excepted
//   jump       moved further in a frame than its shown speed takes it
//
// Build and run, from the repo's root:
//
//   sh tools/landing_soak/build.sh                  (into /tmp/landing_soak)
//   AIRFIELDS=airfields.bin /tmp/landing_soak --set all
//
// build.sh takes SRC= another checkout, to build this against its code - the
// same runs before and after a change - and OUT= the binary.
//
// Scenarios, by name - none given, ##w at Glasgow at three pressures:
//
//   w:CODE[:pressure]          "##w" as the bench gives it: TEST01, an A320,
//                              one report 4nm out on the extended centreline
//                              of CODE's first runway's first end, 1,300ft
//                              over the field, 140kt, -700fpm - and never
//                              again; pressure the day's, in feet
//   lost:CODE:nm[:pressure]    the same nm out on a three-degree glideslope
//   again:CODE:nm:t:offM:kt    as lost, then found again once at t s on the
//                              runway, offM metres on from where the
//                              simulation has it, at kt (--xoff M: that far
//                              right of the centreline, on the exit it took)
//   taxi:CODE:nm:t:offM:kt     as again, then reported every 10 s, slowing to
//                              15kt where the simulation has it
//   feed:CODE:decel:period[:lostFrom:found[:farKt]]
//                              an arrival reported every period s from 4nm
//                              out, at farKt there (140) slowing evenly to
//                              140kt over the threshold, down 0.17nm in,
//                              braking at decel kt/s to 15kt and stopping
//                              short of the far end; nothing heard from
//                              lostFrom s until found s
//
// Options: --set w|homes|lostfinal|again|all, --seconds S (240), --fps N (30),
// --seed N, --poll S (10: the poll f.lost is set at), --jitter, --taken F
// (that share of the stands taken), --take REF,REF, --xoff M, --no-taxiways
// (no layout, as with no airfields.bin on the card), --trace (every frame, or
// --every N; --from S --to S a window), --log (the firmware's own lines).
//
// One row a scenario: the touchdown (td_), the hand-over to the follower
// (ho_: time, metres along, the estimate's knots the frame before), when
// main would move on to another (pk45_t) and end the follow (end_t), the
// simulation's stop, then eps (episodes), back3s, disp3s, tailf (frames),
// jump - the worst of each - and what it was doing when; then each episode.
#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "adsb_client.h"
#include "aircraft_db.h"
#include "airfield_pack.h"
#include "airports.h"
#include "follow.h"
#include "ground_follow.h"
#include "ground_route.h"
#include "runways.h"
#include "scene3d.h"
#include "subject3d.h"
#include "telemetry.h"
#include "view3d.h"

uint32_t g_harnessMs = 0;
bool g_harnessSerial = false;
HarnessSerial Serial;

// The aircraft database isn't built in: an A320 is no rotorcraft, nor is
// anything here but a category A7.
bool isRotorcraft(const String &type, const String &category) { return category == "A7"; }

// adsb_client.cpp's, as it has it - the rest of that file is the feed's.
float haversineNm(double lat1, double lon1, double lat2, double lon2) {
    constexpr float R_KM = 6371.0f, DEG = (float)M_PI / 180.0f;
    float p1 = (float)lat1 * DEG, p2 = (float)lat2 * DEG;
    float dphi = (float)(lat2 - lat1) * DEG, dlambda = (float)(lon2 - lon1) * DEG;
    float s1 = sinf(dphi / 2), s2 = sinf(dlambda / 2);
    float a = s1 * s1 + cosf(p1) * cosf(p2) * s2 * s2;
    float km = 2 * R_KM * asinf(sqrtf(std::min(a, 1.0f)));
    return km * 0.539957f;
}

// The harness's day pressure, as dayPressureFt() would have it.
static int32_t g_dayFt = 0;
static bool harnessPressure(int32_t &ft) {
    ft = g_dayFt;
    return true;
}

namespace view3d {
namespace {

namespace s3 = scene3d;
using s3::Vec3;
constexpr float DEG = (float)M_PI / 180;
Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(Vec3 a, float k) { return {a.x * k, a.y * k, a.z * k}; }
float length(Vec3 a) { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }

// --- the view's globals the placement uses, as view3d.cpp has them ---------
enum class Where : uint8_t { NOWHERE, AIRFIELD, COUNTRY };
Where g_where = Where::NOWHERE;
char g_airport[5] = "";
double g_lat0 = 0, g_lon0 = 0, g_cosLat0 = 1;
float g_fieldFt = 0;
char g_endName[24][4];
Vec3 g_endAt[24];
int g_runways = 0;
Vec3 local(double lat, double lon) {
    return {(float)((lon - g_lon0) * 111320.0 * g_cosLat0), (float)((lat - g_lat0) * 110574.0), 0};
}
ground::Network g_network;
ground::Follower g_follower;
int g_taxiKt = -1;
char g_taxiSaid[40] = "";
airfield::List<ground::Vec2> g_standing;
airfield::Layout g_layout;
Vec3 place(double lat, double lon, float altFt, bool ground) {
    Vec3 p = local(lat, lon);  // the harness is only ever at an airfield
    p.z = ground ? 0 : std::max(0.0f, (altFt - g_fieldFt) * 0.3048f);
    return p;
}
s3::Look lookOf(const String &hex, const String &type, const String &callsign, bool military, const String &category) {
    return s3::lookFor(type.c_str(), 0x1B2A55, military, category.c_str());
}
bool g_wasGround = false;
s3::Pilot g_pilot;
Vec3 g_shownAt{};
float g_shownHeading = 0;
bool g_haveShown = false;
Where g_shownWhere = Where::NOWHERE;
float g_lastSeconds = -1, g_lastTrack = 0;
float g_frameDt = 0;
uint32_t g_nowMs = 0;
float g_seconds = 0;
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
// The view's frame is set up once, by buildAirfield(): choose() keeps it.
void choose(const Followed &) {}

#include "still.inc"

// What the harness reads back of each frame.
int g_shownKt = 0;
FollowEstimate g_lastE;
bool g_lastTaxiways = false;

void prepareNext(const Followed &f, const std::vector<Aircraft> &traffic, uint32_t polledMs) {
#include "placement.inc"
    g_shownKt = shownKt;
    g_lastE = e;
    g_lastTaxiways = taxiways && !rolling;
    // From after "int shownKt": the touchdown's log line, and g_wasGround -
    // which it and the lift-off's read - as the rest of prepareNext() has them.
    if (g_where == Where::AIRFIELD && e.ground && !g_wasGround && f.movement == RunwaySim::Movement::LANDING) {
        Serial.printf("[3d] %s down on %s at %s\n", f.callsign.c_str(), f.runway.ident != nullptr ? f.runway.ident : "?",
                      g_airport);
    }
    g_wasGround = e.ground;
}

}  // namespace
}  // namespace view3d

using namespace view3d;

// --- the pack, as tools/ground_soak reads it ------------------------------------
struct Pack {
    std::vector<uint8_t> file;
    struct Row {
        char code[5];
        uint32_t at, size;
    };
    std::vector<Row> rows;
    uint32_t u32(size_t i) const {
        return file[i] | file[i + 1] << 8 | file[i + 2] << 16 | (uint32_t)file[i + 3] << 24;
    }
    bool open(const char *path) {
        FILE *fp = fopen(path, "rb");
        if (fp == nullptr) return false;
        fseek(fp, 0, SEEK_END);
        long n = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        file.resize(n > 0 ? n : 0);
        size_t got = fread(file.data(), 1, file.size(), fp);
        fclose(fp);
        if (got != file.size() || file.size() < 16 || memcmp(file.data(), "OVAF", 4) != 0) return false;
        unsigned count = file[6] | file[7] << 8;
        uint32_t index = u32(8);
        for (unsigned i = 0; i < count; i++) {
            Row r{};
            memcpy(r.code, &file[index + i * 12], 4);
            r.at = u32(index + i * 12 + 4);
            r.size = u32(index + i * 12 + 8);
            rows.push_back(r);
        }
        return true;
    }
    bool layout(const char *code, airfield::Layout &out) const {
        for (const Row &r : rows) {
            if (strcmp(r.code, code) == 0) return airfield::parse(&file[r.at], r.size, out);
        }
        return false;
    }
};
static Pack g_pack;
static bool g_noTaxiways = false;  // --no-taxiways: as with no airfields.bin on the card

// buildAirfield() as view3d.cpp has it, the layout from the pack rather than
// the card.
static bool buildAirfield(const char *code) {
    int count = 0;
    const Runway *runways = runwaysAt(code, count);
    if (runways == nullptr || count == 0) return false;
    double lat0 = 0, lon0 = 0;
    int open = 0;
    for (int i = 0; i < count; i++) {
        if (!runways[i].closed) {
            lat0 += (runways[i].end[0].lat + runways[i].end[1].lat) / 2.0;
            lon0 += (runways[i].end[0].lon + runways[i].end[1].lon) / 2.0;
            open++;
        }
    }
    if (open == 0) return false;
    g_lat0 = lat0 / open;
    g_lon0 = lon0 / open;
    g_cosLat0 = cos(g_lat0 * M_PI / 180);
    g_fieldFt = runways[0].elevationFt;
    int n = 0;
    for (int i = 0; i < count && n < 12; i++) {
        if (runways[i].closed) continue;
        for (int e = 0; e < 2; e++) {
            snprintf(g_endName[n * 2 + e], sizeof g_endName[0], "%s", runways[i].end[e].ident);
            g_endAt[n * 2 + e] = local(runways[i].end[e].lat, runways[i].end[e].lon);
        }
        n++;
    }
    g_runways = n;
    g_layout = airfield::Layout();
    bool real = !g_noTaxiways && g_pack.layout(code, g_layout);
    if (real) {
        Vec3 offset = local(g_layout.lat0, g_layout.lon0);
        if (fabsf(offset.x) > 0.01f || fabsf(offset.y) > 0.01f) g_layout.moveBy(offset.x, offset.y);
        g_network.build(g_layout);
    } else {
        g_network = ground::Network();
    }
    g_follower.end();
    g_taxiKt = -1;
    snprintf(g_airport, sizeof g_airport, "%s", code);
    g_where = Where::AIRFIELD;
    return true;
}

// --- a scenario ---------------------------------------------------------------------
struct Scenario {
    std::string name;
    std::string code;
    int kind = 0;        // 0 ##w, 1 lost on final, 2 found again rolling, 3 found again and taxiing on, 4 a feed
    // kind 4: a real arrival's reports every periodS, braking at decelKt a second on the
    // roll-out - harder than the simulation's 2.2 - from 4nm out; none between lostFromS
    // and foundS (0: none lost).
    float decelKt = 3.5f, periodS = 10, lostFromS = 0, foundS = 0;
    float farKt = 140;  // its speed 4nm out, slowing evenly to 140kt over the threshold
    float nm = 4;        // the last report on final, nm out
    int pressure = 0;    // the day's pressure, feet
    float againS = 0, againOffM = 0, againKt = 0;
};

struct Options {
    float seconds = 240, fps = 30;
    unsigned seed = 1;
    float poll = 10;
    bool trace = false;
    int every = 1;
    float from = -1, to = 1e9f;
    float taken = 0;     // this share of the stands taken by others standing there, by --seed
    bool jitter = false;  // frames of uneven length, by --seed: 1/30 to 1/8 s, a 0.6 s stall now and then
    std::string takenRefs;  // these stands taken, by ref, comma separated
    float xoff = 0;  // again/taxi: found that many metres right of the centreline, turned off
};

struct Episode {
    const char *what;
    float t0, t1, worst;
    char doing[40];
};

static float wrapPi(float a) { return remainderf(a, 2 * (float)M_PI); }

struct Result {
    std::string name;
    float tdT = -1, tdAlong = 0, hoT = -1, hoAlong = 0, hoKt = 0, picker45T = -1, mainEndT = -1, stopT = -1;
    float lastAlong = 0;
    std::vector<Episode> eps;
    std::vector<std::string> doings;
    float worstBack = 0, worstDisp = 0, worstJump = 0;
    int tailFrames = 0, backEps = 0;
    float runwayLen = 0;
    std::string endName;
    std::string finalDoing;
};

static unsigned rng(unsigned &s) {
    s = s * 1103515245u + 12345u;
    return (s >> 16) & 0x7fff;
}

static Result run(const Scenario &sc, const Options &o) {
    Result res;
    res.name = sc.name;
    // The view's state from nothing, as a new flight: see view3d::frame().
    g_haveShown = false;
    g_follower.end();
    g_taxiSaid[0] = 0;
    g_wasGround = false;
    g_lastSeconds = -1;
    g_pilot = s3::Pilot();
    g_standing.clear();
    if (!buildAirfield(sc.code.c_str())) {
        printf("%s: no runways for %s\n", sc.name.c_str(), sc.code.c_str());
        return res;
    }
    // Others parked on stands - overnight most are - their stands taken.
    {
        unsigned ts = o.seed * 2246822519u + 3;
        const airfield::List<ground::StandWay> &stands = g_network.stands();
        for (size_t k = 0; k < stands.size(); k++) {
            if (!stands[k].ok) continue;
            bool take = o.taken > 0 && (rng(ts) % 1000) < o.taken * 1000;
            std::string refs = "," + o.takenRefs + ",";
            take = take || refs.find(std::string(",") + stands[k].ref + ",") != std::string::npos;
            if (take) g_standing.push_back(stands[k].line.back());
        }
    }
    int count = 0;
    const Runway *r = runwaysAt(sc.code.c_str(), count);
    // The first end of the first runway, as ##w has it.
    double dn = (r->end[1].lat - r->end[0].lat) * 60, de = (r->end[1].lon - r->end[0].lon) * 60 * cos(r->end[0].lat * M_PI / 180);
    double len = sqrt(dn * dn + de * de);
    Vec3 thr = local(r->end[0].lat, r->end[0].lon), far = local(r->end[1].lat, r->end[1].lon);
    float ux = far.x - thr.x, uy = far.y - thr.y, ul = hypotf(ux, uy);
    ux /= ul;
    uy /= ul;
    res.runwayLen = ul;
    res.endName = r->end[0].ident;
    auto alongOf = [&](float x, float y) { return (x - thr.x) * ux + (y - thr.y) * uy; };
    auto crossOf = [&](float x, float y) { return (x - thr.x) * uy - (y - thr.y) * ux; };

    const uint32_t BASE = 1000000;
    g_harnessMs = BASE;
    g_dayFt = sc.pressure;
    Aircraft pick;
    pick.hex = "aa0001";
    pick.callsign = "TEST01";
    pick.type = "A320";
    pick.category = "A3";
    pick.status = "DESCEND";
    pick.hasPos = pick.hasTrack = pick.hasVertRate = true;
    pick.lat = (float)(r->end[0].lat - sc.nm * dn / len / 60);
    pick.lon = (float)(r->end[0].lon - sc.nm * de / len / (60 * cos(r->end[0].lat * M_PI / 180)));
    pick.track = (float)(fmod(atan2(de, dn) * 180 / M_PI + 360, 360));
    // ##w: 1,300ft over the field whatever the distance; otherwise on a
    // three-degree glideslope, 318ft a mile, from 50ft over the threshold.
    int above = sc.kind == 0 ? 1300 : (int)lroundf(50 + sc.nm * 318);
    pick.altStr = String(r->elevationFt + above);
    pick.speedStr = String((int)lroundf(sc.kind == 4 ? sc.farKt : 140));
    pick.vertRate = -700;
    RunwaySim sim;
    sim.report(followSampleOf(pick, millis(), harnessPressure));

    Followed f;
    f.hex = pick.hex;
    f.callsign = pick.callsign;
    f.type = pick.type;
    f.category = pick.category;
    // g_follow.lost: set at the first poll after it, as it is in none.
    unsigned seed = o.seed * 2654435761u + 7;
    float firstPoll = (rng(seed) % 1000) / 1000.0f * o.poll;
    bool lost = false;
    uint32_t seenMs = BASE;  // g_follow.seenMs: startFollow()
    bool again = false;
    float nextAgain = sc.againS;
    int againN = 0;

    const float dt = 1 / o.fps;
    struct Frame {
        float t, x, y, h, v;
    };
    std::vector<Frame> hist;
    struct Slip {
        float t, back;
    };
    std::vector<Slip> slips;
    Episode cur[4] = {};
    bool on[4] = {};
    const char *names[4] = {"back3s", "disp3s", "tailfirst", "jump"};
    float pathS = 0;
    char lastDoing[40] = "";
    bool haveTd = false, haveHo = false;
    float prevKt = 0;
    unsigned js = o.seed * 3266489917u + 11;
    float tj = 0;
    for (int i = 0;; i++) {
        float t = o.jitter ? tj : i * dt;
        if (o.jitter) {
            unsigned r1 = rng(js) % 1000;
            tj += r1 < 15 ? 0.6f : 1 / 30.0f + (r1 / 1000.0f) * (1 / 8.0f - 1 / 30.0f);
        }
        if (t > o.seconds) break;
        g_harnessMs = BASE + (uint32_t)lroundf(t * 1000);
        uint32_t now = millis();
        // Found again: a report on the runway where the simulation has it,
        // offM on, at kt - then, taxiing on, more every 10s slowing to 15kt.
        if (sc.kind == 4 && t >= nextAgain) {
            // The real aircraft: on the glideslope at 140kt to its touchdown 0.17nm
            // in, then braking at decelKt to 15kt and on down the runway at that.
            // Slowing evenly with distance from farKt 4nm out to 140kt over the
            // threshold: v = 140 + k x, x the nm to go - so x(t) = (x0 + 140/k) e^(-kt) - 140/k.
            float kk = (sc.farKt - 140.0f) / sc.nm / 3600.0f;  // per second
            auto togo = [&](float tt) {
                return kk > 1e-9f ? (sc.nm + 140.0f / 3600.0f / kk) * expf(-kk * tt) - 140.0f / 3600.0f / kk
                                  : sc.nm - 140.0f * tt / 3600.0f;
            };
            float thrS = kk > 1e-9f ? logf((sc.nm + 140.0f / 3600.0f / kk) / (140.0f / 3600.0f / kk)) / kk
                                    : sc.nm / 140.0f * 3600.0f;
            float tdS = thrS + 0.17f / 140.0f * 3600.0f, a, kt;
            if (t < thrS) {
                float x = togo(t);
                a = -x * 1852.0f;
                kt = 140.0f + kk * 3600.0f * x;
            } else if (t < tdS) {
                a = (t - thrS) * 140.0f / 3600.0f * 1852.0f;
                kt = 140;
            } else {
                // Down to 15kt, on at that, and slowing at 1kt/s to stop 185m
                // short of the far end - not on past it, as no aircraft goes.
                float rs = t - tdS, bs = std::min(rs, (140.0f - 15.0f) / sc.decelKt);
                float aBraked = 0.17f * 1852.0f + (140.0f * bs - sc.decelKt * bs * bs / 2) / 3600.0f * 1852.0f;
                float stopAt = (float)len * 1852.0f - 185.0f, slowM = 15.0f * 15.0f / 2 / 3600.0f * 1852.0f;
                float onS = std::max(0.0f, (stopAt - slowM - aBraked) / (15.0f / 3600.0f * 1852.0f));
                float cs = std::min(rs - bs, onS), ss = std::min(std::max(0.0f, rs - bs - onS), 15.0f);
                a = aBraked + (15.0f * cs + 15.0f * ss - ss * ss / 2) / 3600.0f * 1852.0f;
                kt = rs <= bs ? 140.0f - sc.decelKt * bs : 15.0f - ss;
            }
            bool gap = sc.lostFromS > 0 && t >= sc.lostFromS && (sc.foundS <= 0 || t < sc.foundS);
            if (!gap && t > 0) {
                Aircraft g = pick;
                double la = r->end[0].lat + a / 1852.0 * dn / len / 60;
                double lo = r->end[0].lon + a / 1852.0 * de / len / (60 * cos(r->end[0].lat * M_PI / 180));
                g.lat = (float)la;
                g.lon = (float)lo;
                g.speedStr = String((int)lroundf(kt));
                if (t >= tdS) {
                    g.status = "GROUND";
                    g.altStr = "GND";
                    g.hasVertRate = false;
                } else {
                    g.altStr = String(r->elevationFt + (int)lroundf(50 + (0.17f * 1852 - a) / 1852.0f * 318));
                }
                sim.report(followSampleOf(g, now, harnessPressure));
                seenMs = now;
                again = true;
            }
            nextAgain = t + sc.periodS;
        }
        if (sc.kind >= 2 && sc.kind <= 3 && t >= nextAgain && (sc.kind == 3 || againN == 0)) {
            FollowEstimate e = sim.at(now);
            float a = alongOf(local(e.lat, e.lon).x, local(e.lat, e.lon).y) + (againN == 0 ? sc.againOffM : 0);
            float kt = againN == 0 ? sc.againKt : std::max(15.0f, sc.againKt - 10 * againN);
            Aircraft g = pick;
            g.status = "GROUND";
            g.altStr = "GND";
            g.hasVertRate = false;
            double la = r->end[0].lat + a / 1852.0 * dn / len / 60;
            double lo = r->end[0].lon + a / 1852.0 * de / len / (60 * cos(r->end[0].lat * M_PI / 180));
            // --xoff: found off the runway, that many metres to its right - on
            // the exit it took - and turned that way.
            if (o.xoff != 0) {
                la += -o.xoff / 1852.0 * de / len / 60;
                lo += o.xoff / 1852.0 * dn / len / (60 * cos(r->end[0].lat * M_PI / 180));
                g.track = fmodf((float)(atan2(de, dn) * 180 / M_PI) + 90 + 360, 360);
            }
            g.lat = (float)la;
            g.lon = (float)lo;
            g.speedStr = String((int)lroundf(kt));
            sim.report(followSampleOf(g, now, harnessPressure));
            seenMs = now;
            again = true;
            againN++;
            nextAgain = t + 10;
        }
        // The poll: lost while it isn't in it.
        if (t >= firstPoll && fmodf(t - firstPoll, o.poll) < dt) {
            lost = sc.kind == 4 ? now - seenMs > (uint32_t)(o.poll * 1000) : !(again && now - seenMs < 1000);
        }
        f.now = sim.at(now);
        f.reportMs = sim.haveFresh() ? sim.fresh().ms : 0;
        if (sim.haveFresh()) {
            f.report = sim.fresh();
            f.landed = sim.airborneWithin(f.report.ms, subject3d::LANDED_WITHIN_MS);
        }
        f.simulating = sim.simulating(now);
        f.movement = sim.movement();
        f.runway = sim.runway();
        f.lost = lost;
        // main's update3dSubject(): seen landing, slowed under 45kt on the
        // ground - the moment it moves on, if there is anything else about.
        {
            FollowEstimate pe = sim.at(now);
            if (res.picker45T < 0 && pe.ground && f.movement == RunwaySim::Movement::LANDING &&
                pe.gsKt < subject3d::ROLLED_OUT_KT) {
                res.picker45T = t;
            }
        }
        // main's tickFollow(): the follow ends - "[follow] lost" - unless the
        // view is parking it (view3d::parkingLost()).
        bool parkingLost = g_follower.active() && g_follower.parking();
        if (res.mainEndT < 0 && now - seenMs > std::max(FOLLOW_LOST_MS, sim.keepMs()) && !parkingLost) {
            res.mainEndT = t;
        }
        g_nowMs = now;
        g_seconds = t;
        prepareNext(f, {}, 0);

        float x = g_shownAt.x, y = g_shownAt.y, h = g_shownHeading;
        float v = g_shownKt * 0.5144f;
        float along = alongOf(x, y), cross = crossOf(x, y);
        const FollowEstimate &e = g_lastE;
        if (!haveTd && e.ground) {
            haveTd = true;
            res.tdT = t;
            res.tdAlong = along;
        }
        if (!haveHo && g_follower.active()) {
            haveHo = true;
            res.hoT = t;
            res.hoAlong = along;
            res.hoKt = prevKt;
        }
        if (res.stopT < 0 && e.ground && e.gsKt == 0 && f.movement != RunwaySim::Movement::TAKEOFF) res.stopT = t;
        prevKt = e.gsKt;
        const char *doing = g_follower.active() ? g_follower.doing() : (e.ground ? "runway sim" : "in the air");
        if (strcmp(doing, lastDoing) != 0) {
            snprintf(lastDoing, sizeof lastDoing, "%s", doing);
            char b[80];
            snprintf(b, sizeof b, "%.1f %s", t, doing);
            res.doings.push_back(b);
        }
        bool pushing = strncmp(doing, "pushing", 7) == 0;
        // The checks.
        float flags[4] = {0, 0, 0, 0};
        if (!hist.empty()) {
            const Frame &p = hist.back();
            float sx = x - p.x, sy = y - p.y, step = hypotf(sx, sy);
            float hd = pushing ? h + (float)M_PI : h;
            if (step > 0.03f) {
                float dir = atan2f(sy, sx);
                float off = fabsf(wrapPi(dir - hd));
                slips.push_back({t, off > (float)M_PI / 2 ? step : 0});
                pathS += off > (float)M_PI / 2 ? -step : step;
            }
            while (!slips.empty() && slips.front().t < t - 3) slips.erase(slips.begin());
            float backSum = 0;
            for (const Slip &sl : slips) backSum += sl.back;
            if (backSum > 1) flags[0] = backSum;
            // disp3s
            // The frames 3 s and 0.5 s back, by the clock: frames needn't be even.
            int k = (int)hist.size() - 1, m = k;
            while (k >= 0 && hist[k].t > t - 3) k--;
            while (m >= 0 && hist[m].t > t - 0.5f) m--;
            if (k >= 0) {
                const Frame &q = hist[k];
                float d = (x - q.x) * cosf(hd) + (y - q.y) * sinf(hd);
                if (d < -1) flags[1] = -d;
            }
            // tail-first over 0.5s
            if (m >= 0) {
                const Frame &q = hist[m];
                float mx = x - q.x, my = y - q.y, mv = hypotf(mx, my) / (t - q.t);
                if (mv > 1) {
                    float off = fabsf(wrapPi(atan2f(my, mx) - hd));
                    if (off > 120 * DEG) flags[2] = off / DEG;
                }
            }
            float allowed = std::max(v, p.v) * (t - p.t) * 1.05f + 0.02f;
            if (step > allowed + 0.25f) flags[3] = step - allowed;
        }
        hist.push_back({t, x, y, h, v});
        if (hist.size() > 400) hist.erase(hist.begin(), hist.begin() + 100);
        for (int c = 0; c < 4; c++) {
            if (flags[c] > 0) {
                if (!on[c]) {
                    on[c] = true;
                    cur[c] = {names[c], t, t, flags[c], ""};
                    snprintf(cur[c].doing, sizeof cur[c].doing, "%s", doing);
                } else {
                    cur[c].t1 = t;
                    cur[c].worst = std::max(cur[c].worst, flags[c]);
                }
            } else if (on[c]) {
                on[c] = false;
                res.eps.push_back(cur[c]);
            }
        }
        res.worstBack = std::max(res.worstBack, flags[0]);
        res.worstDisp = std::max(res.worstDisp, flags[1]);
        res.worstJump = std::max(res.worstJump, flags[3]);
        res.tailFrames += flags[2] > 0;
        res.lastAlong = along;
        bool anyFlag = flags[0] > 0 || flags[1] > 0 || flags[2] > 0 || flags[3] > 0;
        if (o.trace && t >= o.from && t <= o.to && (i % o.every == 0 || anyFlag)) {
            printf("t=%7.3f %-22s sim=%d mv=%d lost=%d est a=%7.1f gs=%3d gnd=%d | shown a=%7.1f c=%+6.1f s=%7.1f hdg=%5.1f kt=%3d %-34s%s%s%s%s\n",
                   t, e.how, (int)f.simulating, (int)f.movement, (int)f.lost,
                   alongOf(local(e.lat, e.lon).x, local(e.lat, e.lon).y), e.gsKt, (int)e.ground, along, cross, pathS,
                   fmodf(90 - h / DEG + 720, 360), g_shownKt, doing, flags[0] > 0 ? " BACK3S" : "",
                   flags[1] > 0 ? " DISP3S" : "", flags[2] > 0 ? " TAILFIRST" : "", flags[3] > 0 ? " JUMP" : "");
        }
    }
    for (int c = 0; c < 4; c++) {
        if (on[c]) res.eps.push_back(cur[c]);
    }
    for (const Episode &ep : res.eps) res.backEps += strcmp(ep.what, "jump") != 0;
    res.finalDoing = lastDoing;
    return res;
}

static void addHomes(std::vector<Scenario> &v, const char *codes[], int n, int pressure) {
    for (int i = 0; i < n; i++) {
        Scenario s;
        s.code = codes[i];
        s.kind = 0;
        s.nm = 4;
        s.pressure = pressure;
        s.name = std::string("w:") + codes[i] + ":" + std::to_string(pressure);
        v.push_back(s);
    }
}

static bool parseScenario(const char *arg, Scenario &s) {
    std::vector<std::string> p;
    std::string a = arg;
    size_t from = 0;
    while (true) {
        size_t c = a.find(':', from);
        p.push_back(a.substr(from, c == std::string::npos ? std::string::npos : c - from));
        if (c == std::string::npos) break;
        from = c + 1;
    }
    if (p.size() < 2) return false;
    s.name = arg;
    s.code = p[1];
    if (p[0] == "w") {
        s.kind = 0;
        s.pressure = p.size() > 2 ? atoi(p[2].c_str()) : 0;
    } else if (p[0] == "lost" && p.size() >= 3) {
        s.kind = 1;
        s.nm = atof(p[2].c_str());
        s.pressure = p.size() > 3 ? atoi(p[3].c_str()) : 0;
    } else if (p[0] == "feed" && p.size() >= 4) {
        // feed:CODE:decelKt:periodS[:lostFromS:foundS]
        s.kind = 4;
        s.nm = 4;
        s.decelKt = atof(p[2].c_str());
        s.periodS = atof(p[3].c_str());
        s.lostFromS = p.size() > 4 ? atof(p[4].c_str()) : 0;
        s.foundS = p.size() > 5 ? atof(p[5].c_str()) : 0;
        s.farKt = p.size() > 6 ? atof(p[6].c_str()) : 140;
        s.againS = 0;
    } else if ((p[0] == "again" || p[0] == "taxi") && p.size() >= 6) {
        s.kind = p[0] == "again" ? 2 : 3;
        s.nm = atof(p[2].c_str());
        s.againS = atof(p[3].c_str());
        s.againOffM = atof(p[4].c_str());
        s.againKt = atof(p[5].c_str());
        s.pressure = p.size() > 6 ? atoi(p[6].c_str()) : 0;
    } else {
        return false;
    }
    return true;
}

int main(int argc, char **argv) {
    Options o;
    std::vector<Scenario> list;
    std::string set = "";
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--seconds" && i + 1 < argc) o.seconds = atof(argv[++i]);
        else if (a == "--fps" && i + 1 < argc) o.fps = atof(argv[++i]);
        else if (a == "--seed" && i + 1 < argc) o.seed = (unsigned)atoi(argv[++i]);
        else if (a == "--poll" && i + 1 < argc) o.poll = atof(argv[++i]);
        else if (a == "--trace") o.trace = true;
        else if (a == "--every" && i + 1 < argc) o.every = std::max(1, atoi(argv[++i]));
        else if (a == "--from" && i + 1 < argc) o.from = atof(argv[++i]);
        else if (a == "--to" && i + 1 < argc) o.to = atof(argv[++i]);
        else if (a == "--log") g_harnessSerial = true;
        else if (a == "--taken" && i + 1 < argc) o.taken = atof(argv[++i]);
        else if (a == "--take" && i + 1 < argc) o.takenRefs = argv[++i];
        else if (a == "--jitter") o.jitter = true;
        else if (a == "--xoff" && i + 1 < argc) o.xoff = atof(argv[++i]);
        else if (a == "--no-taxiways") g_noTaxiways = true;
        else if (a == "--set" && i + 1 < argc) set = argv[++i];
        else {
            Scenario s;
            if (!parseScenario(argv[i], s)) {
                fprintf(stderr, "what is %s?\n", argv[i]);
                return 2;
            }
            list.push_back(s);
        }
    }
    const char *pack = getenv("AIRFIELDS") ? getenv("AIRFIELDS") : "airfields.bin";
    if (!g_pack.open(pack)) {
        fprintf(stderr, "can't read %s\n", pack);
        return 1;
    }
    if (list.empty() && set.empty()) set = "w";
    if (set == "w" || set == "all") {
        const char *gla[] = {"GLA"};
        addHomes(list, gla, 1, 0);
        addHomes(list, gla, 1, 300);
        addHomes(list, gla, 1, -300);
    }
    if (set == "homes" || set == "all") {
        const char *homes[] = {"EDI", "LHR", "MAN", "DUB", "AMS", "BHX", "LGW", "STN", "BRS", "ABZ", "BFS", "CDG", "FRA"};
        addHomes(list, homes, sizeof homes / sizeof homes[0], 0);
    }
    if (set == "lostfinal" || set == "all") {
        const char *codes[] = {"GLA", "EDI", "LHR", "MAN", "DUB", "AMS"};
        float nms[] = {0.5f, 1, 2, 3, 4, 6};
        for (const char *c : codes) {
            for (float nm : nms) {
                Scenario s;
                s.code = c;
                s.kind = 1;
                s.nm = nm;
                char b[40];
                snprintf(b, sizeof b, "lost:%s:%g", c, nm);
                s.name = b;
                list.push_back(s);
            }
        }
    }
    if (set == "again" || set == "all") {
        const char *codes[] = {"GLA", "EDI", "LHR", "MAN", "DUB", "AMS"};
        for (const char *c : codes) {
            // Found again rolling - 30s after touching down, behind or ahead of
            // the simulation - and found again taxiing on.
            float offs[] = {-200, 0, 200};
            for (float off : offs) {
                Scenario s;
                s.code = c;
                s.kind = 2;
                s.nm = 4;
                s.againS = 125;
                s.againOffM = off;
                s.againKt = 60;
                char b[60];
                snprintf(b, sizeof b, "again:%s:4:125:%g:60", c, off);
                s.name = b;
                list.push_back(s);
            }
            Scenario s;
            s.code = c;
            s.kind = 3;
            s.nm = 4;
            s.againS = 160;
            s.againOffM = 100;
            s.againKt = 25;
            char b[60];
            snprintf(b, sizeof b, "taxi:%s:4:160:100:25", c);
            s.name = b;
            list.push_back(s);
        }
    }
    printf("%-26s %-4s %6s %6s %6s %6s %6s %5s %6s %6s %6s %5s %7s %7s %5s %7s  %s\n", "scenario", "end", "rwy_m", "td_t",
           "td_a", "ho_t", "ho_a", "ho_kt", "pk45_t", "stop_t", "end_t", "eps", "back3s", "disp3s", "tailf", "jump",
           "doing...");
    std::vector<Result> all;
    for (const Scenario &s : list) {
        Result r = run(s, o);
        all.push_back(r);
        std::string doing;
        for (size_t i = 0; i < r.doings.size(); i++) doing += (i ? " | " : "") + r.doings[i];
        printf("%-26s %-4s %6.0f %6.1f %6.0f %6.1f %6.0f %5.0f %6.1f %6.1f %6.1f %5d %7.1f %7.1f %5d %7.2f  %s\n",
               r.name.c_str(), r.endName.c_str(), r.runwayLen, r.tdT, r.tdAlong, r.hoT, r.hoAlong, r.hoKt, r.picker45T,
               r.stopT, r.mainEndT, r.backEps, r.worstBack, r.worstDisp, r.tailFrames, r.worstJump, doing.c_str());
        for (const Episode &ep : r.eps) {
            printf("    %-9s t=%6.1f-%6.1f worst %6.1f  while %s\n", ep.what, ep.t0, ep.t1, ep.worst, ep.doing);
        }
        fflush(stdout);
    }
    return 0;
}
