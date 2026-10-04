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

FollowSample sampleOf(const Truth &t, uint32_t ms, int baroBias) {
    FollowSample s = {};
    s.ms = ms; s.ground = t.ground; s.hasAlt = true; s.altFt = t.ground ? FIELD : (int)t.alt + baroBias;
    s.hasGs = true; s.gsKt = (int16_t)lroundf(t.gs); s.hasPos = true; place(t.along, t.cross, s.lat, s.lon);
    s.hasTrack = true; s.track = t.track; s.hasVs = !t.ground; s.vsFpm = 0;
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
    const char *lastHow = "";
    bool wasSim = false;
    for (float t = t0; t <= t1 + 1e-3f; t += 0.5f) {
        uint32_t ms = BASE + (uint32_t)lroundf((t - t0) * 1000);
        if (t >= nextReport - 1e-3f) {
            nextReport += 10;
            if (reported(t)) {
                Truth tr = truth(t);
                FollowSample s = sampleOf(tr, ms, baroBias);
                Truth tp = truth(t - 1); s.vsFpm = (int16_t)((tr.alt - tp.alt) * 60) ; s.hasVs = !tr.ground;
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
        }
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
}
