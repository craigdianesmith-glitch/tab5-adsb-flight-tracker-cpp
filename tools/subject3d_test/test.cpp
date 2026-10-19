// Drives subject3d::Picker - what the 3D view shows, include/subject3d.h - on
// the PC through what it meets at Glasgow, every aircraft reported each ten
// seconds into its own RunwaySim as the app's tracks are, and checks what it
// picks and when: following moving on once the one followed has landed and
// slowed - to a take-off lined up, a landing coming in, or the nearest in
// the air - and, while the one followed is taxiing out, the others shown
// meanwhile, until it lines up; a helicopter shown throughout, set down and
// lifting off again. Build and run, from the repo's root:
//
//   g++ -std=c++17 -O1 -Itools/runway_sim_test/stub -Iinclude tools/subject3d_test/test.cpp src/follow.cpp
//       src/runways.cpp -o /tmp/subject3d_test
//   /tmp/subject3d_test
//
// (all one line). Each run prints what was shown as it changed; the last
// line says how many checks failed.
#include <cstdio>
#include <functional>
#include <map>
#include <vector>

#include "airports.h"
#include "runways.h"
#include "subject3d.h"

// --- what the device's other files give it ---------------------------------------

float haversineNm(double lat1, double lon1, double lat2, double lon2) {
    double r = M_PI / 180, dlat = (lat2 - lat1) * r, dlon = (lon2 - lon1) * r;
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1 * r) * cos(lat2 * r) * sin(dlon / 2) * sin(dlon / 2);
    return 2 * 3440.065 * asin(sqrt(a));
}
const std::vector<Airport> &airportsWithin(double lat, double lon, float rangeNm) {
    static std::vector<Airport> v{{"GLA", 55.8719, -4.43306}};
    static std::vector<Airport> none;
    return haversineNm(lat, lon, 55.8719, -4.43306) <= rangeNm ? v : none;
}
bool nearestAirport(double lat, double lon, Airport &out, float maxNm, bool) {
    if (haversineNm(lat, lon, 55.8719, -4.43306) > maxNm) {
        return false;
    }
    out = {"GLA", 55.8719, -4.43306};
    return true;
}

// --- Glasgow's runway 23, and where things are on it -----------------------------------

const float T_LAT = 55.879902f, T_LON = -4.418310f, F_LAT = 55.863499f, F_LON = -4.449060f;
const float K = 60.0f * cosf(T_LAT * M_PI / 180);
const int FIELD = 26;
float UX, UY, HEADING23;

// Along 23 from its threshold, and across it to the right, in nm.
void place(float along, float cross, float &lat, float &lon) {
    float x = UX * along + UY * cross, y = UY * along - UX * cross;
    lat = T_LAT + y / 60;
    lon = T_LON + x / K;
}

struct State {
    float along, cross, alt, gs, track;
    bool ground;
    bool rotor;
};
using Path = std::function<State(float)>;

FollowSample sampleOf(const State &s, uint32_t ms) {
    FollowSample f = {};
    f.ms = ms;
    f.ground = s.ground;
    f.hasAlt = true;
    f.altFt = s.ground ? FIELD : (int)s.alt;
    f.hasGs = true;
    f.gsKt = (int16_t)lroundf(s.gs);
    f.hasPos = true;
    place(s.along, s.cross, f.lat, f.lon);
    f.hasTrack = true;
    f.track = s.track;
    f.rotorcraft = s.rotor;
    return f;
}

// Landing on 23: touching down at t=`at` 0.17nm in at 140kt, braking 2.5kt/s
// to 20kt, then turning off and taxiing at 15kt.
Path landing(float at) {
    return [at](float t) {
        t -= at;
        State r = {};
        r.track = HEADING23;
        if (t < 0) {
            r.along = 0.17f + 140 * t / 3600;
            r.alt = FIELD + (0.17f - r.along) * 318;
            r.gs = 140;
            return r;
        }
        r.ground = true;
        float tb = std::min(t, 48.0f);
        r.along = 0.17f + (140 * tb - 2.5f * tb * tb / 2) / 3600;
        r.gs = 140 - 2.5f * tb;
        if (t > 48) {
            r.cross = 15 * (t - 48) / 3600;
            r.gs = 15;
            r.track = HEADING23 + 90;
        }
        return r;
    };
}

// Lined up on 23 from t=`from`; rolling at 3kt/s from t=`roll`, off at 150kt,
// climbing at 2,200fpm.
Path takeoff(float roll) {
    return [roll](float t) {
        State r = {};
        r.track = HEADING23;
        r.ground = true;
        r.along = 0.02f;
        if (t < roll) {
            return r;
        }
        t -= roll;
        if (t < 50) {
            r.gs = 3 * t;
            r.along = 0.02f + 1.5f * t * t / 3600;
            return r;
        }
        float a = t - 50;
        r.ground = false;
        r.along = 0.02f + 1.5f * 2500 / 3600 + (150 * a + a * a / 2) / 3600;
        r.gs = 150 + a;
        r.alt = FIELD + 2200 * a / 60;
        return r;
    };
}

// Taxiing on the airfield off the runway, 0.3nm to its side, at 15kt;
// stopping at t=`stopAt` (never, if negative).
Path taxiing(float stopAt = -1) {
    return [stopAt](float t) {
        State r = {};
        r.ground = true;
        r.cross = -0.3f;
        r.track = HEADING23 + 180;
        float moving = stopAt >= 0 ? std::min(t, stopAt) : t;
        r.along = 1.2f - 15 * moving / 3600;
        r.gs = stopAt >= 0 && t >= stopAt ? 0 : 15;
        return r;
    };
}

// A helicopter in to the apron, 0.3nm off 23's side a mile in: from a mile
// further off at 60kt and 600ft, slowing to a hover over it and set down at
// t=`at` - its transponder saying so if `saysGround`, as many never do - and
// off again `down` seconds later, climbing away at 90kt.
Path helicopter(float at, float down, bool saysGround) {
    return [=](float t) {
        State r = {};
        r.rotor = true;
        r.along = 1;
        r.track = HEADING23 + 90;
        t -= at;
        if (t < 0) {
            float out = std::min(1.0f, -t / 60);
            r.cross = -0.3f - out;
            r.gs = 60 * std::min(1.0f, -t / 30);
            r.alt = FIELD + 30 + 570 * out;
            return r;
        }
        if (t < down) {
            r.cross = -0.3f;
            r.ground = saysGround;
            r.alt = FIELD + 5;
            return r;
        }
        t -= down;
        r.cross = -0.3f + 90 * t / 3600;
        r.gs = 90;
        r.alt = FIELD + 5 + 800 * t / 60;
        return r;
    };
}

// In the air `nm` off, at `alt`, flying on.
Path flying(float nm, float alt) {
    return [nm, alt](float t) {
        State r = {};
        r.along = -nm;
        r.cross = 3 + 250 * t / 3600;
        r.alt = alt;
        r.gs = 250;
        r.track = HEADING23 + 90;
        return r;
    };
}

// --- a run ----------------------------------------------------------------------------

struct Track {
    RunwaySim sim;
    Aircraft last;
    uint32_t freshMs = 0;
};
using Tracks = std::map<String, Track>;

struct Contact {
    String hex;
    Path path;
    float from, until;  // reported from, until: seconds
};

int failures = 0;

void check(bool ok, const char *what) {
    printf("    %s %s\n", ok ? "ok  " : "FAIL", what);
    failures += !ok;
}

const uint32_t BASE = 1000000;

struct Shown {
    float t;
    subject3d::Show show;
    String hex;  // the guest, or who following moved on to; the one followed for FOLLOWED
};

// Runs `contacts` from t0 to t1, the one followed being `followed`, its view
// at `airport` - `atStand` answering for it - opened at `opened`, its tracks
// kept from t0; and on NEXT following moving on as main does. What was shown
// each second, as it changed.
std::vector<Shown> run(const char *name, String followed, std::vector<Contact> contacts, float t0, float t1,
                       std::function<const char *(float)> airport, std::function<bool(double, double)> atStand,
                       float opened = -1e9f, float closed = -1e9f, float reopened = -1e9f) {
    printf("\n=== %s ===\n", name);
    Tracks tracks;
    subject3d::Picker picker;
    picker.reset();
    std::vector<Shown> shown;
    float nextPoll = t0;
    for (float t = t0; t <= t1 + 1e-3f; t += 1) {
        uint32_t ms = BASE + (uint32_t)lroundf((t - t0) * 1000);
        if (t >= nextPoll - 1e-3f) {
            nextPoll += 10;
            for (const Contact &c : contacts) {
                if (t < c.from || t > c.until) {
                    continue;
                }
                Track &tr = tracks[c.hex];
                tr.sim.report(sampleOf(c.path(t), ms));
                tr.last.hex = c.hex;
                tr.last.callsign = c.hex;
                tr.freshMs = ms;
            }
        }
        auto it = tracks.find(followed);
        if (it == tracks.end() || t < opened || (t >= closed && t < reopened)) {
            continue;  // the view not open: nothing picked
        }
        if (fabsf(t - reopened) < 0.5f) {
            picker.reset();  // the view closed a while - an aircraft's details - and opened again
        }
        String next;
        subject3d::Show s = picker.pick(followed, it->second.sim, tracks, airport(t), atStand, ms, next);
        String who = s == subject3d::Show::GUEST ? picker.guest() : s == subject3d::Show::NEXT ? next : followed;
        if (shown.empty() || shown.back().show != s || shown.back().hex != who) {
            const char *what = s == subject3d::Show::FOLLOWED ? "following, shown" : s == subject3d::Show::GUEST ? "guest"
                                                                                                           : "moved on to";
            printf("  t=%5.0f %-16s %s\n", t, what, who.c_str());
            shown.push_back({t, s, who});
        }
        if (s == subject3d::Show::NEXT) {
            followed = next;  // as main's followOnTo(): following it now, the view kept open
            picker.clearGuest();
        }
    }
    return shown;
}

// What was shown at t.
const Shown *at(const std::vector<Shown> &shown, float t) {
    const Shown *last = nullptr;
    for (const Shown &s : shown) {
        if (s.t <= t) {
            last = &s;
        }
    }
    return last;
}

bool showing(const std::vector<Shown> &shown, float t, subject3d::Show show, const char *hex) {
    const Shown *s = at(shown, t);
    return s != nullptr && s->show == show && s->hex == hex;
}

// When it first showed `show` `hex`, -1 never.
float firstAt(const std::vector<Shown> &shown, subject3d::Show show, const char *hex) {
    for (const Shown &s : shown) {
        if (s.show == show && s.hex == hex) {
            return s.t;
        }
    }
    return -1;
}

int main() {
    int n;
    runwaysAt("GLA", n);
    float len = hypotf((F_LON - T_LON) * K, (F_LAT - T_LAT) * 60);
    UX = (F_LON - T_LON) * K / len;
    UY = (F_LAT - T_LAT) * 60 / len;
    HEADING23 = atan2f(UX, UY) * 180 / M_PI;
    if (HEADING23 < 0) {
        HEADING23 += 360;
    }
    auto atGLA = [](float) { return "GLA"; };
    auto noStands = [](double, double) { return false; };
    auto stands = [](double, double) { return true; };
    using S = subject3d::Show;

    {
        // Landed, slowed: on to the take-off lined up for after it.
        auto shown = run("landed: on to the one lined up", "LAND1",
                         {{"LAND1", landing(0), -150, 200}, {"DEP1", takeoff(120), 30, 300}}, -150, 200, atGLA,
                         noStands);
        float moved = firstAt(shown, S::NEXT, "DEP1");
        check(showing(shown, -60, S::FOLLOWED, "LAND1"), "on final: the one followed");
        check(showing(shown, 20, S::FOLLOWED, "LAND1"), "rolling out fast: still the one followed");
        check(moved >= 30 && moved <= 50, "following moved on to the take-off once slowed on the runway (t 30-50)");
        check(showing(shown, 150, S::FOLLOWED, "DEP1"), "the take-off then followed, rolling");
    }
    {
        // Landed, the view closed over the end of its rollout and opened
        // again once it had slowed: still seen landing - on to the next.
        auto shown = run("landed: the view closed over its rollout's end", "LAND1",
                         {{"LAND1", landing(0), -150, 200}, {"DEP1", takeoff(120), 30, 300}}, -150, 200, atGLA,
                         noStands, -1e9f, 30, 45);
        float moved = firstAt(shown, S::NEXT, "DEP1");
        check(moved >= 45 && moved <= 47, "following moved on to the take-off as the view opened again");
    }
    {
        // Landed: on to the landing coming in next - not the one far off in the air.
        auto shown = run("landed: on to the next landing", "LAND1",
                         {{"LAND1", landing(0), -150, 200},
                          {"LAND2", landing(150), -150, 300},
                          {"HIGH1", flying(12, 9000), -150, 300}},
                         -150, 200, atGLA, noStands);
        float land2 = firstAt(shown, S::NEXT, "LAND2"), high1 = firstAt(shown, S::NEXT, "HIGH1");
        check(land2 >= 30 && land2 <= 50, "following moved on to the landing due next");
        check(high1 < 0 || high1 > land2 + 100, "not to the one high up while a landing is due - only after it");
    }
    {
        // Landed, nothing due here: the nearest in the air.
        auto shown = run("landed: nothing due - the nearest in the air", "LAND1",
                         {{"LAND1", landing(0), -150, 200},
                          {"NEAR1", flying(10, 5000), -150, 300},
                          {"FAR1", flying(40, 30000), -150, 300}},
                         -150, 200, atGLA, noStands);
        check(firstAt(shown, S::NEXT, "NEAR1") >= 30, "following moved on to the nearest in the air");
        check(firstAt(shown, S::NEXT, "FAR1") < 0, "not the further one");
    }
    {
        // Landed, nothing else anywhere: itself, still.
        auto shown =
            run("landed: nothing else at all", "LAND1", {{"LAND1", landing(0), -150, 200}}, -150, 200, atGLA, noStands);
        check(shown.size() == 1 && shown[0].show == S::FOLLOWED, "the one followed throughout");
    }
    {
        // Departing, taxiing out: a landing meanwhile, then a take-off, then
        // back to it as it lines up.
        auto shown = run("departing: others meanwhile, then its own take-off", "OUT1",
                         {{"OUT1", [](float t) { return t < 300 ? taxiing()(t) : takeoff(340)(t); }, 0, 500},
                          {"LAND2", landing(100), -100, 300},
                          {"DEP2", takeoff(200), 150, 400},
                          {"HIGH1", flying(20, 9000), 0, 500}},
                         0, 450, atGLA, noStands);
        check(showing(shown, 50, S::GUEST, "LAND2"), "taxiing: the landing coming in shown meanwhile");
        float dep2 = firstAt(shown, S::GUEST, "DEP2");
        check(dep2 >= 130 && dep2 <= 170, "once it has landed and slowed, the take-off lined up next");
        float own = -1;
        for (const Shown &s : shown) {
            if (s.t >= 300 && s.show == S::FOLLOWED && s.hex == "OUT1" && own < 0) {
                own = s.t;
            }
        }
        check(own >= 300 && own <= 315, "back to the one followed as it lines up (t 300-315)");
        check(showing(shown, 420, S::FOLLOWED, "OUT1"), "and on its take-off");
        bool neverNext = true;
        for (const Shown &s : shown) {
            neverNext = neverNext && s.show != S::NEXT;
        }
        check(neverNext, "following never moved off the departure");
    }
    {
        // Departing, nothing due: one in the air - until a take-off lines up.
        auto shown = run("departing: in the air meanwhile, a take-off looked for", "OUT1",
                         {{"OUT1", taxiing(), 0, 300}, {"HIGH1", flying(20, 9000), 0, 300}, {"DEP2", takeoff(200), 60, 300}},
                         0, 250, atGLA, noStands);
        check(showing(shown, 30, S::GUEST, "HIGH1"), "the one in the air, nothing due");
        float dep2 = firstAt(shown, S::GUEST, "DEP2");
        check(dep2 >= 60 && dep2 <= 76, "the take-off lined up taken within the look's 15s");
    }
    {
        // Departing, its guest lost: another.
        auto shown = run("departing: a guest lost", "OUT1",
                         {{"OUT1", taxiing(), 0, 400}, {"HIGH1", flying(10, 9000), 0, 50}, {"HIGH2", flying(30, 9000), 0, 400}},
                         0, 300, atGLA, noStands);
        check(showing(shown, 30, S::GUEST, "HIGH1"), "the nearer one in the air");
        float other = firstAt(shown, S::GUEST, "HIGH2");
        check(other >= 130 && other <= 160, "lost, the other once its reports are long stale");
    }
    {
        // Landed a while ago, found taxiing in, then stopped on its stand.
        auto shown = run("landed earlier: shown once parked on its stand", "IN1",
                         {{"IN1", [](float t) { return t < 0 ? landing(-200)(t) : taxiing(60)(t); }, -300, 300},
                          {"HIGH1", flying(20, 9000), -300, 300}},
                         -300, 200, atGLA, stands, 0);
        check(showing(shown, 30, S::GUEST, "HIGH1"), "taxiing in: another meanwhile, following kept");
        float parked = firstAt(shown, S::FOLLOWED, "IN1");
        check(parked >= 89 && parked <= 100, "stopped 30s on its stand: shown parked (t 90-100)");
    }
    {
        // ...stopped somewhere not a stand: another, still.
        auto shown = run("landed earlier: stopped off any stand", "IN1",
                         {{"IN1", [](float t) { return t < 0 ? landing(-200)(t) : taxiing(60)(t); }, -300, 300},
                          {"HIGH1", flying(20, 9000), -300, 300}},
                         -300, 200, atGLA, noStands, 0);
        check(firstAt(shown, S::FOLLOWED, "IN1") < 0, "never shown, stopped off the stands");
    }
    {
        // ...the view high up with its guest: shown to find out, then not.
        auto shown = run("landed earlier: view elsewhere, then at it", "IN1",
                         {{"IN1", [](float t) { return t < 0 ? landing(-200)(t) : taxiing(60)(t); }, -300, 300},
                          {"HIGH1", flying(20, 9000), -300, 300}},
                         -300, 200, [](float t) { return t < 120 ? "" : "GLA"; }, noStands, 0);
        float shownAt = firstAt(shown, S::FOLLOWED, "IN1");
        check(shownAt >= 89 && shownAt <= 100, "high up: shown, its stand unknown");
        check(showing(shown, 150, S::GUEST, "HIGH1"), "at its airport, not on a stand: another again");
    }
    for (bool saysGround : {false, true}) {
        // A helicopter set down by the terminal for five minutes, a landing
        // due meanwhile: shown all the while, never moved on from.
        auto shown = run(saysGround ? "a helicopter set down, saying so" : "a helicopter set down, saying nothing",
                         "HELI1",
                         {{"HELI1", helicopter(0, 300, saysGround), -120, 500},
                          {"LAND2", landing(100), -100, 300},
                          {"HIGH1", flying(20, 9000), -120, 500}},
                         -120, 500, atGLA, noStands, -60);
        check(shown.size() == 1 && shown[0].show == S::FOLLOWED && shown[0].hex == "HELI1",
              "the helicopter throughout: coming in, set down, and away again");
    }
    printf("\n%d checks failed\n", failures);
    return failures != 0;
}
