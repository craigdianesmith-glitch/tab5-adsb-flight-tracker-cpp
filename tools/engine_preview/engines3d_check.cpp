// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

// Checks on the PC what the 3D view asks of the one followed's engines -
// include/engines3d.h, the code the view's engineSound() runs - through each
// phase of a Eurofighter Typhoon's flight, with the look, the set-up and the
// airbrake the view's scene3d gives it (lookFor, configFor, spoilersRising);
// beside it an A320's; and that an A320's, and every other type's, is still
// what the view gave it before it was taught the Typhoon - bit for bit, at
// every speed, climb and height, on the ground and in the air, taking off,
// landing or neither. Build and run, from the repo's root:
//
//   g++ -std=c++17 -O1 -ffunction-sections -fdata-sections -Iinclude tools/engine_preview/engines3d_check.cpp
//       src/engine_noise.cpp src/scene3d.cpp src/airfield_pack.cpp -Wl,--gc-sections -o /tmp/engines3d_check
//   /tmp/engines3d_check
//
// (all one line). It prints the phases' inputs; the last line says how many
// checks failed.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <string>

#include "engine_noise.h"
#include "engines3d.h"
#include "scene3d.h"

using engine_noise::Engine;
using engine_noise::Kind;
using engine_noise::Sound;
namespace s3 = scene3d;

int g_failed = 0, g_checked = 0;

void check(bool ok, const char *what) {
    g_checked++;
    if (!ok) {
        g_failed++;
        printf("  FAILED: %s\n", what);
    }
}

// --- the view as it was, before the Typhoon (40cf5f6's view3d.cpp, verbatim but
// for its inputs' names) ---------------------------------------------------------------

Sound oldEnginesOf(const std::string &type, uint32_t who) {
    Sound s;
    for (const char *t : {"A318", "A319", "A320", "A321", "A19N", "A20N", "A21N"}) {
        if (type == t) {
            s.kind = Kind::TURBOFAN;
            s.engine = engine_noise::a320Engines(t, who);
            s.fopp = engine_noise::a320Fopp(s.engine, who);
        }
    }
    return s;
}

void oldAsk(Sound &s, const engines3d::Doing &d) {
    bool low = d.low;
    if (s.kind == Kind::ROTOR) {
        s.power = d.ground ? 0.25f : d.kt < 25 ? 0.95f : d.vsFpm > 300 ? 0.85f : d.vsFpm < -300 ? 0.45f : 0.65f;
        if (!d.ground && d.vsFpm < -250) {
            float speed = std::min(1.0f, std::max(0.0f, (d.kt - 30) / 20.0f)) *
                          std::min(1.0f, std::max(0.0f, (90 - d.kt) / 20.0f));
            s.slap = std::min(1.0f, (-d.vsFpm - 250) / 600.0f) * speed;
        }
    } else if (d.ground) {
        if (d.takingOff && (d.simulating || d.kt > 30)) {
            s.power = 1;
        } else if (d.landing && d.kt > 60) {
            s.power = 0.7f;
            s.reverse = true;
        } else {
            s.power = 0.05f + std::min(0.12f, d.kt / 150.0f);
        }
    } else {
        s.power = d.vsFpm > 1200 ? 1.0f : d.vsFpm > 300 ? 0.88f : d.vsFpm < -400 ? (low ? 0.45f : 0.05f) : 0.65f;
    }
    s.afterburner = s.kind == Kind::LOW_BYPASS && d.fighter && s.power >= 0.95f && (d.ground || low);
}

bool sameBits(float a, float b) { return memcmp(&a, &b, sizeof a) == 0; }

// Every other type as it was: which engines, and what they are asked, over
// every phase, speed, climb and height - the Typhoon alone changed.
void unchanged() {
    printf("Every type but the Typhoon as before:\n");
    const char *types[] = {"A318", "A319", "A320", "A321", "A19N", "A20N", "A21N", "B738", "F35",
                           "HAWK", "TOR",  "C17",  "A400", "EC45", "H47",  "U2",   "B77W", "C172"};
    long differ = 0, tried = 0;
    for (const char *type : types) {
        for (uint32_t who = 0; who < 200000; who += 7) {
            Sound a = engines3d::enginesOf(type, who), b = oldEnginesOf(type, who);
            tried++;
            differ += a.kind != b.kind || a.engine != b.engine || a.fopp != b.fopp;
        }
    }
    printf("  engines: %ld airframes of %zu types, %ld differ\n", tried, sizeof types / sizeof types[0], differ);
    check(differ == 0, "an airframe's engines changed");
    // Each kind the view can be given but the fighter's, against the view's
    // old rule: the A320's, a silent type's, a helicopter's (not made yet).
    long asked = 0, wrong = 0;
    const int ALT[] = {0, 1800, 13000, 25000, 41000};
    for (Kind kind : {Kind::NONE, Kind::TURBOFAN, Kind::ROTOR}) {
        for (int phase = 0; phase < 3; phase++) {
            for (int bits = 0; bits < 32; bits++) {
                for (int kt = 0; kt <= 650; kt++) {
                    for (int vs = -4000; vs <= 6000; vs += 25) {
                        for (int alt : ALT) {
                            engines3d::Doing d;
                            d.ground = bits & 1;
                            d.simulating = bits & 2;
                            d.low = bits & 4;
                            d.fighter = bits & 8;
                            d.takingOff = phase == 1;
                            d.landing = phase == 2;
                            d.kt = kt;
                            d.rollKt = bits & 16 ? 0 : kt;
                            d.vsFpm = vs;
                            d.altFt = alt;
                            Sound a, b;
                            a.kind = b.kind = kind;
                            engines3d::ask(a, d);
                            oldAsk(b, d);
                            asked++;
                            wrong += !sameBits(a.power, b.power) || !sameBits(a.slap, b.slap) ||
                                     a.reverse != b.reverse || a.afterburner != b.afterburner ||
                                     !sameBits(a.reheat, b.reheat);
                        }
                    }
                }
            }
        }
    }
    printf("  power, reversers, reheat, blade slap: %ld cases, %ld differ\n", asked, wrong);
    check(wrong == 0, "an airliner's, a silent type's or a helicopter's inputs changed");
    // Its gear, flaps and spoilers, and its engines' height, as before.
    long setUps = 0, moved = 0;
    for (Kind kind : {Kind::NONE, Kind::TURBOFAN, Kind::ROTOR}) {
        for (int bits = 0; bits < 4; bits++) {
            for (float sp = 0; sp <= 1.0f; sp += 0.125f) {
                Sound s;
                s.kind = kind;
                engines3d::airframe(s, bits & 1, bits & 2, sp);
                setUps++;
                moved += s.gearDown != (bool)(bits & 1) || s.flapsOut != (bool)(bits & 2) || !sameBits(s.spoilers, sp);
            }
        }
    }
    for (const char *type : types) {
        s3::Look look = s3::lookFor(type, 0);
        Sound s = engines3d::enginesOf(type, 1);
        float was = 2.5f * look.scale;
        setUps++;
        moved += !sameBits(engines3d::enginesUpPerScale(s, look.scale) * look.scale, was);
    }
    printf("  gear, flaps, spoilers and engines' height: %ld cases, %ld differ\n", setUps, moved);
    check(moved == 0, "an airliner's set-up or engines' height changed");
}

// --- a Typhoon's flight, phase by phase ----------------------------------------------------

struct Phase {
    const char *what;
    bool ground;
    int movement;  // 0 none, 1 taking off (lined up counts), 2 landing
    bool simulating;
    int kt, rollKt, vsFpm, altFt;  // rollKt: the runway simulation's; altFt: over the sea, the field 41ft
    bool low;              // under 3,000ft over the field, the view at it
    float sinceDown;       // seconds since it touched down; -1 none lately
    // What the Typhoon should be given.
    float power;
    bool reheat;
    float airbrake;
};

// The view's height over the field, for its set-up: from the sea's, the field
// at 41ft - on the ground, none.
float heightM(const Phase &p) { return p.ground ? 0 : std::max(0, p.altFt - 41) * 0.3048f; }

Sound asked(const char *type, const Phase &p, const s3::Look &look) {
    engines3d::Doing d;
    d.ground = p.ground;
    d.takingOff = p.movement == 1;
    d.landing = p.movement == 2;
    d.simulating = p.simulating;
    d.kt = p.kt;
    d.rollKt = p.rollKt;
    d.vsFpm = p.vsFpm;
    d.altFt = p.altFt;
    d.low = p.low;
    d.fighter = look.family == s3::Family::FIGHTER_DELTA || look.family == s3::Family::FIGHTER ||
                look.family == s3::Family::FIGHTER_SINGLE;
    Sound s = engines3d::enginesOf(type, 0x43c6f1u);
    engines3d::ask(s, d);
    // Its set-up as the view's scene3d gives it, the spoilers springing up as
    // it touches down - see view3d.cpp's frame.
    s3::Config c = s3::configFor(heightM(p), (float)p.kt, p.ground, p.movement == 1);
    c.spoilerDeploy *= s3::spoilersRising(p.sinceDown >= 0 ? p.sinceDown : 100);
    engines3d::airframe(s, c.gear, c.flaps, c.spoilerDeploy);
    return s;
}

void typhoon() {
    // clang-format off
    const Phase phases[] = {
        {"at its stand, running",                       true,  0, false,   0,   0,      0,     41, true,  -1,   0.05f,  false, 0},
        {"taxiing out at 15kt",                         true,  0, false,  15,  15,      0,     41, true,  -1,   0.15f,  false, 0},
        {"lined up on 23, waiting",                     true,  1, true,    0,   0,      0,     41, true,  -1,   0.88f,  false, 0},
        {"lined up by the sim, the view taxiing it on", true,  1, true,   10,   0,      0,     41, true,  -1,   0.88f,  false, 0},
        {"its roll begun, 3kt",                         true,  1, true,    3,   3,      0,     41, true,  -1,   1.00f,  true,  0},
        {"take-off roll, 80kt",                         true,  1, true,   80,  80,      0,     41, true,  -1,   1.00f,  true,  0},
        {"take-off roll, 145kt",                        true,  1, true,  145, 145,      0,     41, true,  -1,   1.00f,  true,  0},
        {"lifted off, 2,000ft/min, 300ft",              false, 1, true,  150, 150,   2000,    341, true,  -1,   1.00f,  true,  0},
        {"climbing out, 10,000ft/min, 2,000ft",         false, 0, false, 300, 300,  10000,   2041, true,  -1,   1.00f,  true,  0},
        {"climbing through 3,500ft over the field",     false, 0, false, 300, 300,  10000,   3541, false, -1,   1.00f,  false, 0},
        {"climbing gently, 600ft/min",                  false, 0, false, 350, 350,    600,   8000, false, -1,   0.88f,  false, 0},
        {"circling the field, 280kt at 1,800ft (##k0)", false, 0, false, 280, 280,      0,   1800, true,  -1,   0.65f,  false, 0},
        {"display pass, 300kt at 100ft",                false, 0, false, 300, 300,      0,    141, true,  -1,   0.65f,  false, 0},
        {"running in, 400kt at 1,000ft (##ke)",         false, 0, false, 400, 400,      0,   1041, true,  -1,   0.785f, false, 0},
        {"low pass, 420kt at 250ft",                    false, 0, false, 420, 420,      0,    291, true,  -1,   0.846f, false, 0},
        {"480kt at 10,000ft (##kk)",                    false, 0, false, 480, 480,      0,  10000, false, -1,   0.831f, false, 0},
        {"cruise, M0.85 at FL250 (511kt)",              false, 0, false, 511, 511,      0,  25000, false, -1,   0.65f,  false, 0},
        {"cruise at FL300, 150kt jet stream behind",    false, 0, false, 640, 640,      0,  30000, false, -1,   0.65f,  false, 0},
        {"480kt at 17,500ft",                           false, 0, false, 480, 480,      0,  17500, false, -1,   0.682f, false, 0},
        {"579kt at 13,000ft (eufi-flypast)",            false, 0, false, 579, 579,      0,  13000, false, -1,   0.877f, false, 0},
        {"flat out low, 600kt at 500ft",                false, 0, false, 600, 600,      0,    541, true,  -1,   0.88f,  false, 0},
        {"descending at 2,000ft/min, FL200",            false, 0, false, 400, 400,  -2000,  20000, false, -1,   0.05f,  false, 0},
        {"coming down from FL300, jet stream behind",   false, 0, false, 600, 600,  -2000,  30000, false, -1,   0.05f,  false, 0},
        {"into a valley at 420kt, 1,500ft/min down",    false, 0, false, 420, 420,  -1500,    250, false, -1,   0.831f, false, 0},
        {"flat out low, 600kt, 3,000ft/min down",       false, 0, false, 600, 600,  -3000,    541, false, -1,   0.88f,  false, 0},
        {"letting down to run in, 400kt, 600ft/min",    false, 0, false, 400, 400,   -600,   1541, true,  -1,   0.659f, false, 0},
        {"letting down to join, 300kt, 1,000ft/min",    false, 0, false, 300, 300,  -1000,   2041, true,  -1,   0.45f,  false, 0},
        {"on the approach, 150kt, 800ft/min down",      false, 2, true,  150, 150,   -800,    541, true,  -1,   0.45f,  false, 0},
        {"going around, 3,000ft/min",                   false, 0, false, 160, 160,   3000,    441, true,  -1,   1.00f,  true,  0},
        {"touched down, 135kt, 0.4s on",                true,  2, true,  135, 135,      0,     41, true,  0.4f, 0.05f,  false, 0.5f},
        {"landing roll, 100kt",                         true,  2, true,  100, 100,      0,     41, true,  6,    0.05f,  false, 1},
        {"landing roll, 50kt",                          true,  2, true,   50,  50,      0,     41, true,  20,   0.05f,  false, 0.5f},
        {"landing roll, 30kt",                          true,  2, true,   30,  30,      0,     41, true,  28,   0.05f,  false, 0},
        {"taxiing in at 15kt",                          true,  0, false,  15,  15,      0,     41, true,  -1,   0.15f,  false, 0},
    };
    // clang-format on
    s3::Look eufi = s3::lookFor("EUFI", 0x6E6E6E, true, "A6"), a320 = s3::lookFor("A320", 0x1C428C);
    Sound kind = engines3d::enginesOf("EUFI", 1);
    printf("\nA Typhoon (EUFI), as the 3D view gives it:\n");
    printf("  kind %s, engine %s; drawn as a fighter %s; size %.3f (the preview's 0.425); engines %.2fm up (an "
           "A320's %.2fm)\n",
           kind.kind == Kind::LOW_BYPASS ? "LOW_BYPASS" : "?", kind.engine == Engine::EJ200 ? "EJ200" : "?",
           eufi.family == s3::Family::FIGHTER_DELTA ? "yes" : "no", s3::sizeOf(eufi),
           engines3d::enginesUpPerScale(kind, eufi.scale) * eufi.scale,
           engines3d::enginesUpPerScale(engines3d::enginesOf("A320", 1), a320.scale) * a320.scale);
    check(kind.kind == Kind::LOW_BYPASS && kind.engine == Engine::EJ200, "EUFI's engines");
    check(fabsf(engines3d::enginesUpPerScale(kind, eufi.scale) * eufi.scale - 1.5f) < 1e-6f,
          "the Typhoon's engines 1.5m up");
    check(fabsf(s3::sizeOf(eufi) - 0.425f) < 0.002f, "the Typhoon's size as the preview's");
    printf("  %-44s  %-36s  %s\n", "", "Typhoon", "A320, for comparison");
    printf("  %-44s  %5s %-7s %-6s %5s %-5s  %5s %-7s %-5s\n", "phase", "power", "reverse", "reheat", "airbk", "flaps",
           "power", "reverse", "flaps");
    for (const Phase &p : phases) {
        Sound t = asked("EUFI", p, eufi), a = asked("A320", p, a320);
        printf("  %-44s  %5.3f %-7s %-6s %5.2f %-5s  %5.3f %-7s %-5s\n", p.what, t.power, t.reverse ? "yes" : "-",
               t.reheat > 0 ? (t.afterburner ? "lit" : "?") : "-", t.spoilers, t.flapsOut ? "out" : "-", a.power,
               a.reverse ? "yes" : "-", a.flapsOut ? "out" : "-");
        char what[160];
        snprintf(what, sizeof what, "%s: power %.3f, wanted %.3f", p.what, t.power, p.power);
        check(fabsf(t.power - p.power) < 0.002f, what);
        snprintf(what, sizeof what, "%s: reheat %s", p.what, t.reheat > 0 ? "lit" : "not lit");
        check((t.reheat > 0) == p.reheat && t.afterburner == p.reheat && (t.reheat == 0 || t.reheat == 1), what);
        snprintf(what, sizeof what, "%s: reversers", p.what);
        check(!t.reverse, what);
        snprintf(what, sizeof what, "%s: airbrake %.2f, wanted %.2f", p.what, t.spoilers, p.airbrake);
        check(fabsf(t.spoilers - p.airbrake) < 0.01f, what);
        snprintf(what, sizeof what, "%s: flaps", p.what);
        check(!t.flapsOut, what);
        check(!t.apu && !a.apu, "no APU from engines3d");  // the view's own rule: an airliner's alone
    }
}

int main() {
    unchanged();
    typhoon();
    printf("\n%d checks, %d failed\n", g_checked, g_failed);
    return g_failed == 0 ? 0 : 1;
}
