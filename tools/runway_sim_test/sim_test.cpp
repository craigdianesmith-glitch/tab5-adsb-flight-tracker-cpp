// Drives RunwaySim (src/follow.cpp) through take-offs and landings at
// Glasgow's runway 23 on the PC - the feed losing them at the touchdown or on
// the roll, finding them again taxiing or climbing away, a go-around, one
// crossing the runway - and prints what the plot would show each half second
// against where the aircraft really was, with the worst jump. Then take-offs
// reported as Heathrow's departures were (see Feed) - the hang, the
// trampoline, the late pick-up's rocket and the take-off design's stress
// cases - each sampled every 50 ms and judged as the design's test plan
// judges them: one lift-off, never back on the runway, no hang or sinking,
// no steeper than the real climb allows, its roll's speed never dropping,
// off before the runway's end - PASS or FAIL. Build and run, from the repo's
// root:
//
//   g++ -std=c++17 -O1 -Itools/runway_sim_test/stub -Iinclude tools/runway_sim_test/*.cpp src/follow.cpp src/runways.cpp -o /tmp/runway_sim_test
//   /tmp/runway_sim_test
#include <algorithm>
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
// The reports' kind of take-off, as followSampleOf() gives it (takeoff.h):
// an A320's, unless a scenario says.
TakeoffKind g_kind = TakeoffKind::NARROW;
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
    s.takeoffKind = g_kind;
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
// --- Take-offs as the feed reports them -------------------------------------------
//
// A take-off down 23 - or 05 - as the departures off Heathrow's 27R were
// reported on 27 Oct 2026: lined up and stopped until `rollFrom`, then
// rolling at A - B v knots a second (takeoff.h's kind's, by default), off
// at `liftoffKt`, its climb built over `buildS` to `fpm`, gathering `gainKt`
// as it climbs away. Its transponder has it in the air from `airborneKt`,
// still rolling - its height then the field's and 17ft, its pressure put
// right, dipping 25ft over the 3 s before it leaves - and its reported rate
// is its climb's, building: 64ft a minute on the roll. Rejected, it brakes
// at 5 kt a second from `rejectKt`. Each report can be made stranger by
// `g_oddly`: an old position, a speed repeated, a rate of nonsense.
struct Feed {
    TakeoffKind kind = TakeoffKind::NARROW;
    float accel = 5.3f, drag = 0.012f;
    float liftoffKt = 140, airborneKt = 80, fpm = 2700, buildS = 7, gainKt = 6;
    float rollFrom = 0, rejectKt = 0;
    bool reverse = false;  // down 05: from 23's far end
};
Feed g_feed;
struct FeedAt { float along, alt, gs, vs; bool reportedGround, off; };
float feedRollS(float v) {  // seconds from a standstill to v
    const Feed &f = g_feed;
    if (f.drag <= 0) return v / f.accel;
    float top = f.accel / f.drag;
    return logf(top / (top - v)) / f.drag;
}
FeedAt feedAt(float t) {
    const Feed &f = g_feed;
    FeedAt a = {};
    float top = f.drag > 0 ? f.accel / f.drag : 0, rt = std::max(0.0f, t - f.rollFrom);
    float offT = feedRollS(f.liftoffKt);
    auto rollV = [&](float x) { return f.drag > 0 ? top - top * expf(-f.drag * x) : f.accel * x; };
    auto rollNm = [&](float x) {
        return (f.drag > 0 ? top * x - top * (1 - expf(-f.drag * x)) / f.drag : f.accel * x * x / 2) / 3600;
    };
    a.alt = FIELD + 17;
    if (f.rejectKt > 0) {
        float rejT = feedRollS(f.rejectKt);
        if (rt <= rejT) { a.along = 0.02f + rollNm(rt); a.gs = rollV(rt); }
        else {
            float b = std::min(rt - rejT, f.rejectKt / 5);
            a.along = 0.02f + rollNm(rejT) + (f.rejectKt * b - 2.5f * b * b) / 3600; a.gs = f.rejectKt - 5 * b;
        }
        a.reportedGround = a.gs < f.airborneKt;
        if (a.reportedGround) a.alt = FIELD;
        a.vs = 64;
        return a;
    }
    if (rt < offT) {
        a.along = 0.02f + rollNm(rt);
        a.gs = rollV(rt);
        a.reportedGround = a.gs < f.airborneKt;
        if (a.reportedGround) a.alt = FIELD;
        else if (rt > offT - 3) a.alt -= 25 * (rt - (offT - 3)) / 3;
        a.vs = 64;
        return a;
    }
    float tau = rt - offT, built = std::min(tau, f.buildS);
    a.off = true;
    a.along = 0.02f + rollNm(offT) + (f.liftoffKt * tau + f.gainKt * (tau - 3 * (1 - expf(-tau / 3)))) / 3600;
    a.gs = f.liftoffKt + f.gainKt * (1 - expf(-tau / 3));
    float h = tau < f.buildS ? f.fpm / 60 * tau * tau / (2 * f.buildS) : f.fpm / 60 * (tau - f.buildS / 2);
    a.alt = FIELD + std::max(h, 17 - 25.0f + 25 * std::min(tau, 1.0f));
    a.vs = f.fpm * built / f.buildS;
    return a;
}
// Changes a report heard at `t` before it goes in - or drops it.
std::function<bool(float, FollowSample &)> g_oddly = [](float, FollowSample &) { return true; };

// The worst of what a take-off was shown doing, sampled every 50 ms.
void takeoffRun(const char *name, std::function<bool(float)> reported, float t0, float t1, float every = 10) {
    printf("\n=== %s ===\n", name);
    const Feed &f = g_feed;
    const TakeoffProfile &p = takeoffProfile(f.kind);
    RunwaySim sim;
    const uint32_t BASE = 1000000;
    const int PRESSURE = 30;
    float nextReport = t0;
    struct Sample { float t, along, h, kt, fpm; bool ground; };
    std::vector<Sample> sm;
    float realOffT = f.rollFrom + feedRollS(f.liftoffKt), realOffAlong = feedAt(realOffT).along;
    int lastGs = -1;
    for (float t = t0; t <= t1 + 1e-3f; t += 0.05f) {
        uint32_t ms = BASE + (uint32_t)lroundf((t - t0) * 1000);
        if (t >= nextReport - 1e-3f) {
            nextReport += every;
            if (reported(t)) {
                FeedAt a = feedAt(t);
                FollowSample s = {};
                s.ms = ms; s.ground = a.reportedGround; s.hasAlt = true;
                s.altFt = a.reportedGround ? FIELD : (int)lroundf(a.alt) - PRESSURE;
                s.pressureFt = a.reportedGround ? 0 : PRESSURE;
                s.hasGs = true; s.gsKt = (int16_t)lroundf(a.gs); s.hasPos = true;
                float along = f.reverse ? LEN - a.along : a.along;
                place(along, 0, s.lat, s.lon);
                s.hasTrack = true; s.track = f.reverse ? fmodf(rwHeading() + 180, 360) : rwHeading();
                s.hasVs = !a.reportedGround; s.vsFpm = (int16_t)lroundf(a.vs);
                s.takeoffKind = f.kind;
                if (g_oddly(t, s)) {
                    sim.report(s);
                    printf("  t=%6.2f REPORT along=%.3f %s alt=%5d gs=%3d vs=%5d age=%.1f -> %s\n", t, a.along,
                           s.ground ? "GND" : "AIR", s.altFt + s.pressureFt, s.gsKt, s.vsFpm, s.ageCs / 100.0f, sim.phase());
                }
            }
        }
        if (!sim.haveFresh()) continue;
        FollowEstimate e = sim.at(ms);
        float al, cr; toRunway(e.lat, e.lon, al, cr);
        if (f.reverse) al = LEN - al;
        sm.push_back({t, al, e.ground ? 0.0f : (float)(e.altFt - FIELD), (float)e.gsKt, (float)e.vsFpm, e.ground});
        if (fmodf(t - t0 + 0.025f, 5) < 0.05f)
            printf("  t=%6.2f %-14s gnd=%d along=%.3f h=%5.0f kt=%3.0f fpm=%5.0f off in %5.1f s | truth along=%.3f h=%5.0f kt=%3.0f\n",
                   t, e.how, e.ground, al, sm.back().h, sm.back().kt, sm.back().fpm, e.liftoffInS, feedAt(t).along,
                   feedAt(t).alt - FIELD, feedAt(t).gs);
        (void)lastGs;
    }
    // Shown off the ground once, never put back on it; its speed on the
    // roll never dropping (rejected: by no more than 3 kt in 50 ms); no
    // climb over a second slower than 500ft a minute - half its kind's, a
    // Cessna's - from a second after it leaves to 1200ft for more than 2 s
    // (an A380's or a 747's 3.5), nor faster than half as fast again as the
    // real one's and 1000ft a minute, nor in its first 6 s than a quarter
    // over its kind's; never sinking; off 0.1nm short of the far end. Found
    // in the air, from there.
    int lifts = 0, downs = 0, lift = sm.empty() || sm[0].ground ? -1 : 0;
    float drop = 0, step = 0;
    for (size_t i = 1; i < sm.size(); i++) {
        if (sm[i - 1].ground && !sm[i].ground) { lifts++; if (lift < 0) lift = (int)i; }
        if (!sm[i - 1].ground && sm[i].ground) downs++;
        if (sm[i - 1].ground && sm[i].ground && sm[i - 1].kt >= 5) {
            drop = std::min(drop, sm[i].kt - sm[i - 1].kt);
            step = std::max(step, sm[i].kt - sm[i - 1].kt);
        }
    }
    float steep = 0, launch = 0, hang = 0, run = 0, sank = 0, worst = 0, slow = std::min(500.0f, p.climbFpm / 2);
    if (lift >= 0) {
        for (size_t i = lift + 20; i < sm.size() && sm[i].h <= 1200; i++) {
            float fpm = (sm[i].h - sm[i - 20].h) * 60;
            steep = std::max(steep, fpm);
            if (sm[i].t - sm[lift].t <= 6) launch = std::max(launch, fpm);
            if (sm[i].t - sm[lift].t >= 1) { run = fpm < slow ? run + 0.05f : 0; hang = std::max(hang, run); }
            if (sm[i].h < sm[i - 1].h) sank += sm[i - 1].h - sm[i].h;
            if (sm[i].t > realOffT + 5) worst = std::max(worst, fabsf(sm[i].h - (feedAt(sm[i].t).alt - FIELD)));
        }
    }
    bool quad = f.kind == TakeoffKind::SUPERJUMBO || f.kind == TakeoffKind::JUMBO || f.kind == TakeoffKind::WIDE_4;
    bool rejected = f.rejectKt > 0, found = lift == 0;
    bool ok = rejected ? lifts == 0 && downs == 0 && drop >= -3
                       : (found ? lifts == 0 : lifts == 1) && downs == 0 && hang < (quad ? 3.5f : 2.0f) && sank <= 5 &&
                             steep <= 1.5f * f.fpm + 1000 && launch <= 1.25f * p.climbFpm && drop > -0.01f &&
                             (found || sm[lift].along <= LEN - 0.1f);
    if (lift >= 0)
        printf("  lift-offs %d, back on the runway %d; %s at %.2f s (really off at %.2f) at %.0f m (really %.0f, runway "
               "%.0f) at %.0f kt (really %.0f)\n", lifts, downs, found ? "found in the air" : "off", sm[lift].t, realOffT,
               sm[lift].along * 1852, realOffAlong * 1852, LEN * 1852, sm[lift].kt, f.liftoffKt);
    else
        printf("  lift-offs %d, back on the runway %d\n", lifts, downs);
    printf("  steepest %.0f fpm (limit %.0f), first 6 s %.0f (limit %.0f), hung %.2f s (under %.0f fpm), sank %.0f ft, worst "
           "height off the real %.0f ft; on the roll its speed dropped %.1f kt in 50 ms at most, rose %.0f - %s\n",
           steep, 1.5f * f.fpm + 1000, launch, 1.25f * p.climbFpm, hang, slow, sank, worst, -drop, step,
           ok ? "PASS" : "FAIL");
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
    // A widebody - its kind's take-off, as takeoff.h has a 777's - picked
    // up as QTR30C was at Heathrow: lined up, lost on its
    // roll, found again at 1800ft; then with its last report on the runway
    // ten seconds into its roll still saying it was stopped there - a
    // position fifteen seconds old - taken as new, as reports were before
    // they were timed by their positions; and timed by its position.
    g_kind = TakeoffKind::WIDE_LONG;
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
    g_kind = TakeoffKind::LIGHT;
    run("Cessna 172 take-off, lost when it rolls, found at 400ft", lightTakeoff,
        [](float t) { Truth r = lightTakeoff(t); return t <= 0 || (!r.ground && r.alt > 400); }, -40, 110, 0, 1);
    g_kind = TakeoffKind::NARROW;
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

    // The take-offs of the design for take-offs that look like take-offs,
    // their reports as the feed's were at Heathrow: see Feed.
    auto all = [](float) { return true; };
    auto offAt = []() { return g_feed.rollFrom + feedRollS(g_feed.liftoffKt); };
    // The hang (BAW131): a 777 reported just after it leaves - 60ft up,
    // climbing at 512ft a minute, its climb building - then not for 10 s.
    g_feed = Feed(); g_feed.kind = TakeoffKind::WIDE_LONG; g_feed.liftoffKt = 163; g_feed.airborneKt = 92;
    g_feed.accel = 5.1f; g_feed.drag = 0.011f; g_feed.fpm = 2550; g_feed.buildS = 9.5f; g_feed.gainKt = 9;
    takeoffRun("the hang: a 777 reported 2 s after it leaves, rising at 512ft a minute, then 10 s on", all, offAt() + 2 - 50,
               offAt() + 70);
    // The trampoline (BAW594): an A320 last reported rolling at 105 kt, shown
    // still below its rotation speed when the next has it 300ft up.
    g_feed = Feed(); g_feed.airborneKt = 75;
    takeoffRun("the trampoline: an A320 reported rolling at 105 kt, then 300ft up climbing", all,
               feedRollS(105) - 40, feedRollS(105) + 60, 13);
    // The late pick-up's rocket (QTR30C): a 777 rolling, lost, found at
    // 1800ft climbing - and found at 600ft.
    g_feed = Feed(); g_feed.kind = TakeoffKind::WIDE_LONG; g_feed.liftoffKt = 165; g_feed.accel = 5.1f; g_feed.drag = 0.011f;
    g_feed.fpm = 2600; g_feed.buildS = 9.5f; g_feed.gainKt = 9; g_feed.rollFrom = 20;
    takeoffRun("the rocket: a 777 lined up, lost as it rolls, found at 1800ft", [](float t) {
        FeedAt a = feedAt(t); return t <= 20 || a.alt - FIELD > 1800; }, -20, 160);
    takeoffRun("D5: the same, nothing heard from 80 kt till 600ft", [](float t) {
        FeedAt a = feedAt(t); return a.gs < 80 || a.alt - FIELD > 600; }, -20, 160);
    // D1 (BAW2PD): off 17 kt faster than its kind's, a report level at 147 kt
    // after it was shown off.
    g_feed = Feed(); g_feed.liftoffKt = 155; g_feed.airborneKt = 90;
    takeoffRun("D1: an A320 off at 155 kt, a report level and 'airborne' at 147 kt after it was shown off", all,
               feedRollS(147) - 40, feedRollS(147) + 60, 5);
    // D2 (BAW143): a speed repeated, 139 kt twice, gathering speed.
    g_feed = Feed(); g_feed.kind = TakeoffKind::WIDE; g_feed.liftoffKt = 162; g_feed.accel = 5.1f; g_feed.drag = 0.011f;
    g_feed.fpm = 2600; g_feed.buildS = 9.5f; g_feed.gainKt = 9; g_feed.airborneKt = 85;
    {
        static int lastGs = 0;
        g_oddly = [](float t, FollowSample &s) {
            if (s.gsKt >= 145 && s.gsKt < 160 && lastGs >= 130) s.gsKt = (int16_t)lastGs;  // the feed's old one again
            lastGs = s.gsKt;
            return true;
        };
        takeoffRun("D2: an A350, its speed repeated at 139 kt as it gathers speed", all, feedRollS(139) - 40,
                   feedRollS(139) + 60, 5);
        g_oddly = [](float, FollowSample &) { return true; };
    }
    // D3: its first report in the air in the dip, 25ft below its rolling
    // height at 64ft a minute, at the speed it leaves at.
    g_feed = Feed();
    takeoffRun("D3: an A320's first report 'airborne' in the dip, 1 s before it leaves", all, offAt() - 1 - 40,
               offAt() + 60);
    // D4: Comm-B's rates of nonsense, -960 on the roll and -1856 climbing.
    g_feed = Feed();
    g_oddly = [](float t, FollowSample &s) {
        if (s.hasVs) s.vsFpm = feedAt(t).off ? -1856 : -960;
        return true;
    };
    takeoffRun("D4: an A320 reported at -960ft a minute rolling and -1856 climbing", all, -20, 90, 5);
    g_oddly = [](float, FollowSample &) { return true; };
    // D6: found first 400ft up past the runway's end, climbing at 2000.
    g_feed = Feed(); g_feed.fpm = 2000;
    takeoffRun("D6: an A320 first heard 400ft up past the runway's end, at 2000ft a minute", [](float t) {
        FeedAt a = feedAt(t); return a.alt - FIELD >= 400 && a.along > LEN; }, -20, 120, 5);
    // D7: first heard at 130 kt, 'airborne', still rolling.
    g_feed = Feed(); g_feed.airborneKt = 70;
    takeoffRun("D7: an A320 first heard 'airborne' at 130 kt, rolling", [](float t) { return feedAt(t).gs >= 130; },
               feedRollS(130) + 0.05f, feedRollS(130) + 80, 10);
    // D8: the radar's 30 s: on the ground at 40 kt, then 900ft up.
    g_feed = Feed();
    takeoffRun("D8: an A320 every 30 s: on the ground at 40 kt, then 900ft up", all, feedRollS(40), feedRollS(40) + 120, 30);
    // D9: a position 15 s old at 110 kt.
    g_feed = Feed();
    g_oddly = [](float t, FollowSample &s) {
        if (s.gsKt >= 105 && s.gsKt <= 125 && !feedAt(t).off) {
            FeedAt then = feedAt(t - 15);
            place(then.along, 0, s.lat, s.lon);
            s.ms -= 15000;
            s.ageCs = 1500;
        }
        return true;
    };
    takeoffRun("D9: an A320, a report at 110 kt with its position 15 s old", all, -20, 90, 10);
    g_oddly = [](float, FollowSample &) { return true; };
    // D10: off 15 kt faster and 10 kt slower than its kind's.
    g_feed = Feed(); g_feed.liftoffKt = 153;
    takeoffRun("D10: an A320 off at 153 kt, its kind's 138", all, -20, 100, 10);
    g_feed = Feed(); g_feed.liftoffKt = 128;
    takeoffRun("D10: an A320 off at 128 kt", all, -20, 100, 10);
    // D11: a Cessna, and a Typhoon.
    g_feed = Feed(); g_feed.kind = TakeoffKind::LIGHT; g_feed.accel = 2.5f; g_feed.drag = 0; g_feed.liftoffKt = 58;
    g_feed.airborneKt = 200; g_feed.fpm = 700; g_feed.buildS = 4; g_feed.gainKt = 4;
    takeoffRun("D11: a Cessna 172", all, -20, 160, 10);
    g_feed = Feed(); g_feed.kind = TakeoffKind::FIGHTER; g_feed.accel = 12; g_feed.drag = 0.012f; g_feed.liftoffKt = 165;
    g_feed.airborneKt = 300; g_feed.fpm = 6000; g_feed.buildS = 5; g_feed.gainKt = 20;
    takeoffRun("D11: a Typhoon, its reheat lit", all, -20, 60, 5);
    // D12: lined up, lost; then heard at 60 kt 300m further on.
    g_feed = Feed(); g_feed.rollFrom = 25;
    takeoffRun("D12: an A320 lined up, lost, then heard at 60 kt", [](float t) {
        FeedAt a = feedAt(t); return t <= 20 || (a.gs >= 55 && a.gs < 75) || a.off; }, -30, 120, 5);
    // D13: rejected at 100 kt.
    g_feed = Feed(); g_feed.rejectKt = 100; g_feed.airborneKt = 200;
    takeoffRun("D13: an A320 rejecting its take-off at 100 kt", all, -20, 80, 5);
    // D14: off 05, the other way.
    g_feed = Feed(); g_feed.reverse = true;
    takeoffRun("D14: an A320 off 05", all, -20, 100, 10);
}
