// Drives RunwaySim (src/follow.cpp) through take-offs and landings at
// Glasgow's runway 23 on the PC - the feed losing them at the touchdown or on
// the roll, finding them again taxiing or climbing away, a go-around, one
// crossing the runway - and prints what the plot would show each half second
// against where the aircraft really was, with the worst jump. Build and run,
// from the repo's root:
//
//   g++ -std=c++17 -O1 -Itools/runway_sim_test/stub -Iinclude tools/runway_sim_test/*.cpp src/follow.cpp src/runways.cpp -o /tmp/runway_sim_test
//   /tmp/runway_sim_test
#include <cstdio>
#include <functional>
#include <vector>
#include "follow.h"
#include "runways.h"

// Glasgow 23: from its threshold towards the 05 end.
const float T_LAT = 55.879902f, T_LON = -4.418310f, F_LAT = 55.863499f, F_LON = -4.449060f;
const float K = 60.0f * cosf(T_LAT * M_PI / 180);
float UX, UY, LEN;
const int FIELD = 26;

struct Truth { float along, cross, alt; float gs; float track; bool ground; };
void place(float along, float cross, float &lat, float &lon) {
    // cross: to the right of the direction of travel
    float x = UX * along + UY * cross, y = UY * along - UX * cross;
    lat = T_LAT + y / 60; lon = T_LON + x / K;
}
void toRunway(float lat, float lon, float &along, float &cross) {
    float x = (lon - T_LON) * K, y = (lat - T_LAT) * 60;
    along = x * UX + y * UY; cross = x * UY - y * UX;
}
float rwHeading() { float h = atan2f(UX, UY) * 180 / M_PI; return h < 0 ? h + 360 : h; }

bool g_rotor = false;  // the reports a rotorcraft's
bool g_noGs = false;   // the reports without a ground speed, as some feeds' military contacts are
// The reports' take-off figures, as followSampleOf() gives them by its kind
// (scene3d::takeoffOf()): nought, an airliner's.
uint8_t g_liftoffKt = 0, g_accelTenths = 0, g_climbHundreds = 0;
// A report's position taken from this many seconds before it was given, as
// the feed's can be near the ground; and whether it is timed by the
// position, as followSampleOf() does, or by when it was given.
std::function<float(float)> g_ageS = [](float) { return 0.0f; };
bool g_timedByPosition = true;

FollowSample sampleOf(const Truth &t, uint32_t ms, int baroBias) {
    FollowSample s = {};
    s.rotorcraft = g_rotor;
    s.ms = ms; s.ground = t.ground; s.hasAlt = true; s.altFt = t.ground ? FIELD : (int)t.alt + baroBias;
    s.hasGs = !g_noGs; s.gsKt = (int16_t)lroundf(t.gs); s.hasPos = true; place(t.along, t.cross, s.lat, s.lon);
    s.hasTrack = true; s.track = t.track; s.hasVs = !t.ground; s.vsFpm = 0;
    s.liftoffKt = g_liftoffKt; s.accelTenths = g_accelTenths; s.climbHundreds = g_climbHundreds;
    return s;
}

void run(const char *name, std::function<Truth(float)> truth, std::function<bool(float)> reported, float t0, float t1,
         int baroBias, float vsSign, bool dupes = false) {
    printf("\n=== %s ===\n", name);
    RunwaySim sim;
    const uint32_t BASE = 1000000;
    float nextReport = t0;
    float pa = NAN, pc = NAN;
    float worst = 0, worstT = 0;
    // And how far it went backwards - against the way the estimate before
    // had it going - in a half second at worst, and how long it was shown
    // at the ground's height but not on it, moving: skimming along it.
    float plat = NAN, plon = NAN, ptrack = 0;
    float back = 0, backT = 0, skimS = 0;
    // The steepest it was shown climbing over a second, and where it was
    // shown leaving the ground.
    float palt = NAN, steepest = 0, steepestT = 0, offAlong = NAN;
    bool pground = true;
    const char *lastHow = "";
    bool wasSim = false;
    for (float t = t0; t <= t1 + 1e-3f; t += 0.5f) {
        uint32_t ms = BASE + (uint32_t)lroundf((t - t0) * 1000);
        if (t >= nextReport - 1e-3f) {
            nextReport += 10;
            if (reported(t)) {
                float age = g_ageS(t);
                Truth tr = truth(t - age);
                uint32_t at = g_timedByPosition ? ms - (uint32_t)lroundf(age * 1000) : ms;
                FollowSample s = sampleOf(tr, at, baroBias);
                s.ageCs = (uint16_t)(ms - at) / 10;
                Truth tp = truth(t - age - 1); s.vsFpm = (int16_t)((tr.alt - tp.alt) * 60) ; s.hasVs = !tr.ground;
                sim.report(s);
                if (dupes) { FollowSample d = s; d.ms += 2000; sim.report(d); }
                printf("  t=%6.1f REPORT along=%.3f cross=%.3f alt=%d gs=%d %s  -> phase %s\n", t, tr.along, tr.cross,
                       s.altFt, s.gsKt, tr.ground ? "GND" : "AIR", sim.phase());
            }
        }
        if (!sim.haveFresh()) continue;
        FollowEstimate e = sim.at(ms);
        float a, c; toRunway(e.lat, e.lon, a, c);
        Truth tr = truth(t);
        if (!std::isnan(pa)) {
            float step = hypotf(a - pa, c - pc) / 0.5f * 3600;  // kt shown
            float excess = step - std::max(tr.gs, 30.0f) * 1.6f;
            if (excess > worst) { worst = excess; worstT = t; }
            float dx = (e.lon - plon) * K, dy = (e.lat - plat) * 60;
            float forward = (dx * sinf(ptrack * M_PI / 180) + dy * cosf(ptrack * M_PI / 180)) * 1852;
            if (-forward > back) { back = -forward; backT = t; }
        }
        if (!e.ground && e.gsKt > 5 && e.altFt <= FIELD + 5) skimS += 0.5f;
        float alt = e.ground ? FIELD : e.altFt;
        if (!std::isnan(palt) && (alt - palt) * 120 > steepest) { steepest = (alt - palt) * 120; steepestT = t; }
        if (!e.ground && pground && std::isnan(offAlong) && vsSign > 0) offAlong = a;
        palt = alt; pground = e.ground;
        plat = e.lat; plon = e.lon; ptrack = e.track;
        bool simNow = sim.simulating(ms);
        if (strcmp(lastHow, e.how) != 0 || simNow != wasSim || fmodf(t - t0, 10) < 0.25f) {
            printf("  t=%6.1f %-22s sim=%d shown along=%.3f cross=%+.3f alt=%5d gs=%3d trk=%3.0f | truth along=%.3f cross=%+.3f alt=%5.0f  err=%.3fnm\n",
                   t, e.how, simNow, a, c, e.ground ? FIELD : e.altFt, e.gsKt, e.track, tr.along, tr.cross, tr.alt,
                   hypotf(a - tr.along, c - tr.cross));
        }
        lastHow = e.how; wasSim = simNow;
        pa = a; pc = c;
    }
    printf("  worst speed excess over 1.6x truth: %.0f kt at t=%.1f\n", worst, worstT);
    printf("  worst backwards: %.0f m in half a second at t=%.1f; skimming the ground %.1f s\n", back, backT, skimS);
    if (vsSign > 0) {
        printf("  steepest shown climb: %.0f fpm at t=%.1f; shown off the ground at along=%.3f nm (runway %.2f nm)\n",
               steepest, steepestT, offAlong, LEN);
    }
}

// On a base leg five miles out, across the extended centreline at 160kt,
// coming down at 1200ft a minute and levelling at 2000ft - lost at t=0 and
// not found again. Shown, it should carry on across and level off, not hold
// still and sink into the ground where it was lost.
Truth baseLeg(float t) {
    Truth r = {};
    r.along = -5;
    r.cross = 3 - 160 * (t + 60) / 3600;
    r.alt = std::max(2000.0f, 2500 - 1200 * t / 60);
    r.gs = 160;
    r.track = fmodf(rwHeading() + 270, 360);
    return r;
}

// Landing on 23: touchdown at t=0 at 0.17nm, 140kt; braking 2.5kt/s to 20kt;
// turns right onto a taxiway at t=48 over 8s, then taxis at 15kt.
Truth landing(float t) {
    Truth r = {};
    if (t < 0) {
        r.along = 0.17f + 140 * t / 3600; r.alt = FIELD + (0.17f - r.along) * 318; r.gs = 140; r.track = rwHeading();
        return r;
    }
    r.ground = true; r.alt = FIELD;
    float tb = std::min(t, 48.0f);
    r.along = 0.17f + (140 * tb - 2.5f * tb * tb / 2) / 3600; r.gs = 140 - 2.5f * tb; r.track = rwHeading();
    if (t > 48) {
        float tt = std::min(t - 48, 8.0f), ang = tt / 8 * (M_PI / 2), rad = 20 * 8 / 3600.0f / (M_PI / 2);
        r.along += rad * sinf(ang); r.cross = rad * (1 - cosf(ang)); r.gs = 15; r.track = rwHeading() + 90 * tt / 8;
        if (t > 56) r.cross += 15 * (t - 56) / 3600;
    }
    return r;
}

// Take-off on 23: lined up and stopped until t=0, rolls at 3kt/s, rotates at
// 150kt at t=50, then climbs at 2200fpm accelerating 1kt/s.
Truth takeoff(float t) {
    Truth r = {};
    r.track = rwHeading();
    if (t < 0) { r.ground = true; r.alt = FIELD; r.along = 0.02f; return r; }
    if (t < 50) { r.ground = true; r.alt = FIELD; r.gs = 3 * t; r.along = 0.02f + 1.5f * t * t / 3600; return r; }
    float a = t - 50;
    r.along = 0.02f + 1.5f * 2500 / 3600 + (150 * a + a * a / 2) / 3600; r.gs = 150 + a; r.alt = FIELD + 2200 * a / 60;
    return r;
}

// A widebody's take-off on 23, as a Qatar 777's at Heathrow: lined up and
// stopped until t=0, rolling at 3kt/s, off at 160kt at t=53, its climb
// built to 2500fpm over the next five seconds, gathering speed at 1kt/s.
Truth heavyTakeoff(float t) {
    Truth r = {};
    r.track = rwHeading();
    const float offT = 160.0f / 3;
    if (t < 0) { r.ground = true; r.alt = FIELD; r.along = 0.02f; return r; }
    if (t < offT) { r.ground = true; r.alt = FIELD; r.gs = 3 * t; r.along = 0.02f + 1.5f * t * t / 3600; return r; }
    float a = t - offT, u = std::min(a / 5, 1.0f);
    r.along = 0.02f + 1.5f * offT * offT / 3600 + (160 * a + a * a / 2) / 3600; r.gs = 160 + a;
    r.alt = FIELD + 2500 * (a < 5 ? 5 * u * u * u * (1 - u / 2) : a - 2.5f) / 60;
    return r;
}

// A Cessna 172's on 23: from a stop at t=0, 2.5kt/s, off at 60kt, 700fpm.
Truth lightTakeoff(float t) {
    Truth r = {};
    r.track = rwHeading();
    if (t < 0) { r.ground = true; r.alt = FIELD; r.along = 0.02f; return r; }
    if (t < 24) { r.ground = true; r.alt = FIELD; r.gs = 2.5f * t; r.along = 0.02f + 1.25f * t * t / 3600; return r; }
    float a = t - 24;
    r.along = 0.02f + 1.25f * 576 / 3600 + (60 * a + 0.5f * a * a / 2) / 3600; r.gs = 60 + 0.5f * std::min(a, 30.0f);
    r.alt = FIELD + 700 * a / 60;
    return r;
}

// A rolling take-off on 23: taxiing down it at 10kt, then from t=0 off at
// 3kt/s, rotating at 150kt. Its transponder says it is in the air from 40kt,
// as an E190's at Glasgow did - and the zoom cleared it to land.
Truth rollingTakeoff(float t) {
    Truth r = {};
    r.track = rwHeading();
    const float rotateT = 140.0f / 3;
    float tr = std::min(std::max(t, 0.0f), rotateT);
    r.along = 0.1f + 10 * std::min(t, 0.0f) / 3600 + (10 * tr + 1.5f * tr * tr) / 3600;
    r.gs = 10 + 3 * tr; r.alt = FIELD; r.ground = r.gs < 40;
    if (t > rotateT) { float a = t - rotateT; r.along += (150 * a + a * a / 2) / 3600; r.gs = 150 + a; r.alt = FIELD + 2200 * a / 60; }
    return r;
}

// A helicopter: in along 23's line from 0.1nm short at 500ft and 60kt, slowing
// evenly to a hover over a spot 0.4nm in and 0.1nm right of it at t=0, down
// there at t=0 and on the ground (or saying so: `ground`) for two minutes; then
// off across the runway, climbing at 500fpm, speeding up 1.5kt/s.
bool g_heliSaysGround = true;
Truth helicopter(float t) {
    Truth r = {};
    if (t < 0) {
        float u = (t + 60) / 60, done = 2 * u - u * u;  // the way covered, slowing evenly to a stop
        r.along = -0.1f + 0.5f * done; r.cross = 0.1f * done; r.alt = FIELD + 500 * (1 - u);
        r.gs = 60 * (1 - u); r.track = rwHeading() + 6;
        return r;
    }
    r.along = 0.4f; r.cross = 0.1f; r.alt = FIELD; r.track = rwHeading() + 90;
    if (t < 120) { r.ground = g_heliSaysGround; return r; }
    float a = t - 120;
    r.cross += 0.75f * a * a / 3600; r.gs = 1.5f * a; r.alt = FIELD + std::min(50.0f, a * 10) + 500 * a / 60;
    return r;
}

// Go-around: on final until t=-10 at 150ft, then climbs at 1500fpm, straight ahead.
Truth goAround(float t) {
    Truth r = landing(std::min(t, -10.0f));
    if (t > -10) { r.along += 140 * (t + 10) / 3600; r.alt += 1500 * (t + 10) / 60; }
    return r;
}
// Taxiing across the runway at 15kt, 0.6nm in, then accelerating a little after.
Truth crossing(float t) {
    Truth r = {}; r.ground = true; r.alt = FIELD; r.along = 0.6f;
    r.gs = t < 0 ? 10 : 22; r.cross = -0.15f + (std::min(t, 0.0f) + 60) * 10 / 3600 + std::max(t, 0.0f) * 22 / 3600; r.track = rwHeading() + 90;
    return r;
}
int main() {
    int n; runwaysAt("GLA", n);
    LEN = hypotf((F_LON - T_LON) * K, (F_LAT - T_LAT) * 60);
    UX = (F_LON - T_LON) * K / LEN; UY = (F_LAT - T_LAT) * 60 / LEN;
    printf("GLA 23 heading %.0f len %.2fnm, %d runways\n", rwHeading(), LEN, n);

    run("landing, lost below 400ft, found taxiing at t=60", landing,
        [](float t) { Truth r = landing(t); return (!r.ground && r.alt > 400) || t > 55; }, -150, 110, 150, -1);
    run("landing, lost below 400ft, found taxiing at t=130 (170s gap)", landing,
        [](float t) { Truth r = landing(t); return (!r.ground && r.alt > 400) || t > 125; }, -150, 160, 150, -1);
    run("landing, lost below 400ft, found on the rollout at t=20", landing,
        [](float t) { Truth r = landing(t); return (!r.ground && r.alt > 400) || t > 20; }, -150, 110, 0, -1);
    run("landing, every report, both lists", landing, [](float t) { return true; }, -150, 110, -120, -1, true);
    run("take-off, lost when it rolls, found at 800ft", takeoff,
        [](float t) { Truth r = takeoff(t); return t < 0 || (!r.ground && r.alt > 800); }, -40, 110, 100, 1);
    run("take-off, every other report", takeoff,
        [](float t) { return ((int)lroundf(t + 40) / 10) % 2 == 0; }, -40, 110, 100, 1);
    run("go-around", goAround, [](float t) { return true; }, -150, 60, 0, 1);
    run("crossing the runway", crossing, [](float t) { return true; }, -60, 40, 0, 1);
    run("take-off, every report", takeoff, [](float t) { return true; }, -40, 110, 100, 1);
    run("rolling take-off, in the air by the feed from 40kt", rollingTakeoff, [](float t) { return true; }, -30, 90, 0, 1);
    // A widebody - its kind's take-off figures, as scene3d::takeoffOf() has
    // a 777's - picked up as QTR30C was at Heathrow: lined up, lost on its
    // roll, found again at 1800ft; then with its last report on the runway
    // ten seconds into its roll still saying it was stopped there - a
    // position fifteen seconds old - taken as new, as reports were before
    // they were timed by their positions; and timed by its position.
    g_liftoffKt = 160; g_accelTenths = 36; g_climbHundreds = 25;
    auto lostRolling = [](float t) { Truth r = heavyTakeoff(t); return t <= 0 || (!r.ground && r.alt > 1800); };
    run("widebody take-off, lost on its roll, found at 1800ft", heavyTakeoff, lostRolling, -40, 180, 0, 1);
    g_ageS = [](float t) { return t > 5 && t < 15 ? 15.0f : 0.0f; };
    auto staleToo = [](float t) { Truth r = heavyTakeoff(t); return t <= 10 || (!r.ground && r.alt > 1800); };
    g_timedByPosition = false;
    run("the same, its last report on the runway 15s old, taken as new", heavyTakeoff, staleToo, -40, 180, 0, 1);
    g_timedByPosition = true;
    run("the same, that report timed by its position", heavyTakeoff, staleToo, -40, 180, 0, 1);
    g_ageS = [](float) { return 0.0f; };
    run("widebody take-off, every report", heavyTakeoff, [](float t) { return true; }, -40, 130, 0, 1);
    g_liftoffKt = 62; g_accelTenths = 25; g_climbHundreds = 7;
    run("Cessna 172 take-off, lost when it rolls, found at 400ft", lightTakeoff,
        [](float t) { Truth r = lightTakeoff(t); return t <= 0 || (!r.ground && r.alt > 400); }, -40, 110, 0, 1);
    g_liftoffKt = g_accelTenths = g_climbHundreds = 0;
    // A helicopter lined up with the runway coming in: none of it simulated.
    g_rotor = true;
    run("helicopter, in along the runway to a spot beside it, down, off again", helicopter, [](float t) { return true; },
        -60, 200, 0, -1);
    g_heliSaysGround = false;
    run("helicopter, never saying it is on the ground", helicopter, [](float t) { return true; }, -60, 200, 0, -1);
    // The feed busy - rate-limited - and its reports 50 seconds apart: the
    // first a minute out, then 10 seconds short of the hover; and again
    // from 40 seconds out.
    run("helicopter, a report every 50 seconds", helicopter, [](float t) { return fmodf(t + 60, 50) < 1; }, -60, 200,
        0, -1);
    run("helicopter, a report every 50 seconds from 40s out", helicopter,
        [](float t) { return t >= -40 && fmodf(t + 40, 50) < 1; }, -60, 200, 0, -1);
    g_rotor = false;
    g_heliSaysGround = true;
    run("the same, not known for a helicopter (as before)", helicopter, [](float t) { return true; }, -60, 200, 0, -1);
    run("base leg five miles out, lost coming down", baseLeg, [](float t) { return t <= 0; }, -60, 180, 0, -1);
    g_noGs = true;
    run("the same, no speed in its reports", baseLeg, [](float t) { return t <= 0; }, -60, 180, 0, -1);
    g_noGs = false;
}
