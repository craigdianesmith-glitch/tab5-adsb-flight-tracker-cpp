// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

// Plays the 3D view's engine noise (src/engine_noise.cpp) on the PC: an
// aircraft flown past a camera, the synthesiser set from it fifty times a
// second as the 3D view sets it - where it is, which way it points, what its
// engines are doing - and each scene saved as a WAV to listen to, with its
// loudness every two seconds; and beside it the same through a stand-in for
// the Tab5's little speaker - nothing below 300Hz, a little lift at 3kHz -
// to hear on a PC what the Tab5 would play. Build and run, from the repo's
// root:
//
//   g++ -std=c++17 -O2 -Iinclude tools/engine_preview/preview.cpp src/engine_noise.cpp -o /tmp/engine_preview
//   /tmp/engine_preview outdir [scene]
//
// The scenes: a320-takeoff, a320-landing, a320-touchdown, a320-flypast,
// a320-approach-chase, a320-arrival-park, a320-departure-start, a320-chase;
// and heard from where real A320s were recorded, for measuring against
// them: x-a320-climbout-overhead, x-a320-approach-overhead,
// x-a320-approach-side, x-a320-taxi-past; x-a320-touchdown-far, the
// touchdown from 250m, where the sound's lateness is heard; and
// x-a320-intermediate, an approach at 210 knots, where an old A320's fuel
// vents whistle.
//
// And the Eurofighter Typhoon's (Kind::LOW_BYPASS, Engine::EJ200), as
// eufi-sources.md's section 12 has them: eufi-stand-start, eufi-taxi-past,
// eufi-takeoff, eufi-climbout-overhead, eufi-display-pass, eufi-low-pass,
// eufi-flypast, eufi-chase-cruise, eufi-approach-chase, eufi-landing; and
// for measuring against real ones: x-eufi-zeltweg, x-eufi-close-pass,
// x-eufi-display-100m. Each fed as the 3D view feeds a fighter - its power
// and reheat as the table has them, the reversers left on above 60 knots
// on its landing rolls, as the view sets them, for the fighter to ignore.
// tools/engine_preview/render_eufi.sh renders them all; NOTES.md beside it.
//
// And a320-splash: the boot splash's take-off, as include/splash_takeoff.h
// gives it whole and the device makes it, a 40ms block at a time.
//
// And the Boeing 777's: b777-takeoff; for measuring against real ones,
// x-b777-approach-side, x-b777-approach-overhead, x-b777-touchdown-side.
//
// And the ATR's (Kind::TURBOPROP), as atr72-sources.md's phase tables (14)
// have them: atr72-start-idle, atr72-taxi-past, atr72-takeoff,
// atr72-climbout-flyover, atr72-cruise-flyover, atr72-approach-side,
// atr72-approach-behind, atr72-touchdown-reverse, atr42-takeoff and
// atr72-cruise-cinema - each fed as the 3D view feeds a turboprop, its power
// and its propellers' speed by engines3d::ask() itself. Its sound not made
// yet, they are silent; tools/engine_preview/render_atr72.sh renders them
// all, NOTES.md beside it.
//
// ENGINE=cfm56 (the default), v2500, leap or pw: the airframe's engines; for
// a 777's scenes ge90 (the default there), ge90-94 or trent; for the ATR's,
// atr72-600 (the default there, atr42-500 in atr42-takeoff), atr72-200,
// atr72-210, atr42-500 or atr42-300 - each kind's name leaving the other
// kind's scenes as they are.
// FOPP=1: its fuel vents unmodified, whistling. SYNC=1: the touchdown's
// squeal and the reversers' coming on heard as the picture shows them -
// engine_noise::g_transientsOnPicture. SUFFIX=-x: added to the files' names.
// WHO=n: the airframe, for its own little differences. DEVICE=1: frames
// 20ms apart give or take a third and the sound made in 40ms blocks, as the
// device has it. POWER=x: eufi-low-pass's power, 0.65 unless given (the 3D
// view's level flight); 0.85 is more like a real one's. TRACE=1: what the
// synthesiser is given, each second, on stderr - for a turboprop its
// propellers' speed asked too.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "engine_noise.h"
#include "engines3d.h"
#include "splash_takeoff.h"

using engine_noise::Engine;
using engine_noise::Kind;
using engine_noise::Sound;

struct V3 {
    float x, y, z;
};
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
float length(V3 a) { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }

// At `t` seconds: where the aircraft is and which way it points (radians
// from +x, and nose up), its engines' doing, and where the camera is.
struct Moment {
    V3 at;
    float heading = 0, pitch = 0;
    float power = 0;
    bool reverse = false;
    V3 eye;
    bool hardGround = false;  // the ground between it and the camera a runway's, not grass
    bool gear = false, flaps = false;  // down, out
    float spoilers = 0;               // up, 0 to 1 - a fighter's airbrake open
    bool running = true, apu = false;  // its engines, its APU - a turboprop's hotel mode
    float reheat = 0;  // a fighter's reheat asked for, 0 dry to 1 full - afterburner set with it
    // The runway simulation's take-off (lined up counts) or landing, for a
    // turboprop's scenes, whose power and propellers' speed the 3D view's
    // own engines3d::ask() works out from it and the path - `power` unread.
    int movement = 0;  // 0 neither, 1 taking off, 2 landing
};

struct Scene {
    const char *name;
    float seconds;
    std::function<Moment(float)> at;
};

float clamp01(float x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

// Along x from a standstill: rolling at `accel` from `rollAt`, off at
// `liftMs`, climbing at `climbDeg`, `speedUp` m/s2 on.
V3 takeoffPath(float t, float rollAt, float accel, float liftMs, float climbDeg, float &speed, float &pitch) {
    float tr = t - rollAt;
    if (tr <= 0) {
        speed = 0;
        pitch = 0;
        return {0, 0, 0};
    }
    float tLift = liftMs / accel;
    if (tr < tLift) {
        speed = accel * tr;
        pitch = 0;
        return {accel * tr * tr / 2, 0, 0};
    }
    float a = tr - tLift, v = liftMs + 0.8f * a;
    speed = v;
    pitch = climbDeg * (float)M_PI / 180;
    float s = liftMs * liftMs / (2 * accel) + (liftMs + v) / 2 * a;
    float along = s - liftMs * liftMs / (2 * accel);
    return {s, 0, along * tanf(pitch)};
}

// A take-off seen from beside the runway, `side` metres off it and `along`
// down it: idle at the threshold, the power up, the roll, past, and away.
Moment takeoffSeen(float t, float along, float side, float liftMs) {
    Moment m;
    float speed, pitch;
    m.at = takeoffPath(t, 8, 2.0f, liftMs, 8, speed, pitch);
    m.heading = 0;
    m.pitch = pitch;
    m.power = t < 6 ? 0.08f : 1.0f;
    m.eye = {along, side, 4};
    return m;
}

// In on a three-degree glide at 70m/s, down 300m past the threshold - its
// sink some 3.7m/s - its reversers out to 30m/s, then taxiing; heard from
// `eye`.
Moment landing(float t, V3 eye) {
    Moment m;
    float x = -4500 + 70 * t, tTouch = 4800 / 70.0f;
    m.heading = 0;
    m.pitch = 0;
    m.power = 0.45f;
    // Its flaps out; its gear coming down a few seconds in; the spoilers
    // springing up as it touches down and coming down as it slows from 60kt
    // to 40, as the 3D view has them.
    m.flaps = true;
    m.gear = t > 3;
    if (t < tTouch) {
        m.at = {x, 0, std::max(0.0f, (300 - x) * 0.0524f)};
    } else {
        float d = t - tTouch, v = std::max(8.0f, 70 - 2.2f * d);
        float s = d < 28.2f ? 70 * d - 1.1f * d * d : 70 * 28.2f - 1.1f * 28.2f * 28.2f + 8 * (d - 28.2f);
        m.at = {300 + s, 0, 0};
        m.reverse = v > 30;
        m.power = v > 30 ? 0.75f : 0.08f;
        float rise = clamp01(d / 0.8f), slow = clamp01((v / 0.5144f - 40) / 20);
        m.spoilers = rise * rise * (3 - 2 * rise) * slow * slow * (3 - 2 * slow);
    }
    m.eye = eye;
    return m;
}

// The Typhoon's scenes' figures.
// Its length against an A320's: 15.96m against 37.57m.
constexpr float EUFI_SIZE = 0.425f;
// Its jets' nozzles about 1.5m over the ground on its gear - not an
// airliner's engines' 3m.
constexpr float EUFI_ENGINES_UP = 1.5f;
// The 3D view gives a fighter reheat only below 3,000ft over the airfield.
constexpr float FT3000 = 914.4f;
float g_lowPassPower = 0.65f;  // POWER=

// An angle-of-attack approach's power: 0.45, and a correction of up to 3
// points every 3 to 5 seconds - the pilot holding the speed through the
// gusts - the same each time. Not something the 3D view gives (it holds
// 0.45 on a descent), but what the table asks for; hunting by its own
// figures (eufi-sources 11.8) is the synthesiser's to add or not.
float approachPower(float t) {
    static const float AT[] = {0,     3.6f,  7.9f,  11.2f, 15.8f, 19.1f, 23.5f, 27.4f, 31.0f, 35.3f,
                               39.0f, 43.6f, 47.2f, 51.5f, 55.0f, 59.4f, 63.0f, 67.5f, 71.1f, 75.6f};
    static const float BY[] = {0,     0.03f,  -0.02f, 0.01f, -0.03f, 0.02f, 0,     -0.01f, 0.03f, -0.02f,
                               0.01f, -0.03f, 0.02f,  -0.01f, 0.03f, 0,     -0.02f, 0.01f, -0.03f, 0.02f};
    int k = 0;
    while (k + 1 < (int)(sizeof(AT) / sizeof(AT[0])) && t >= AT[k + 1]) {
        k++;
    }
    return 0.45f + BY[k];
}

// A Typhoon's landing: in on a three-degree glide at 70m/s (135 knots) from
// x0, gear down, approach power, touching down at x = 300. Then no
// reversers: its engines to idle at once, its nose held high - aerobraking
// - from 70m/s to 51 (100 knots) at 1.6m/s2, then its wheel brakes at
// 2.5m/s2 to 10m/s, taxiing [eufi-sources 4.2; the rates are mine]. The 3D
// view's reverse = true left on above 60 knots, as it sets it on any
// landing roll, for the fighter to ignore - but its power idle, as the
// table has it, not the 0.7 the view gives with it today (NOTES.md). Its
// airbrake open as it touches down, over 0.8s, and shut as it slows from
// 60 knots to 40, as the 3D view's spoilers are; whether the RAF opens it on
// the roll-out isn't published - the recipe's choice (eufi-methods 9.1).
Moment fighterLanding(float t, float x0, V3 eye) {
    Moment m;
    float tTouch = (300 - x0) / 70;
    m.heading = 0;
    m.pitch = 0;
    m.gear = true;
    m.power = approachPower(t);
    if (t < tTouch) {
        float x = x0 + 70 * t;
        m.at = {x, 0, std::max(0.0f, (300 - x) * 0.0524f)};
    } else {
        const float dA = (70 - 51.4f) / 1.6f, dB = (51.4f - 10) / 2.5f;
        float d = t - tTouch, v, s;
        if (d < dA) {
            v = 70 - 1.6f * d;
            s = 70 * d - 0.8f * d * d;
        } else if (d < dA + dB) {
            float e = d - dA;
            v = 51.4f - 2.5f * e;
            s = 70 * dA - 0.8f * dA * dA + 51.4f * e - 1.25f * e * e;
        } else {
            v = 10;
            s = 70 * dA - 0.8f * dA * dA + 51.4f * dB - 1.25f * dB * dB + 10 * (d - dA - dB);
        }
        m.at = {300 + s, 0, 0};
        m.power = 0.05f;
        m.reverse = v > 60 * 0.5144f;
        float rise = clamp01(d / 0.8f), slow = clamp01((v / 0.5144f - 40) / 20);
        m.spoilers = rise * rise * (3 - 2 * rise) * slow * slow * (3 - 2 * slow);
        m.hardGround = true;
    }
    m.eye = eye;
    return m;
}

// A level pass along x at `speed`, abeam x = 0 at `atS`, `height` up and
// `side` metres off the camera, which is `earUp` up.
Moment levelPass(float t, float speed, float atS, float height, float side, float earUp) {
    Moment m;
    m.at = {speed * (t - atS), side, height};
    m.heading = 0;
    m.pitch = 0;
    m.eye = {0, 0, earUp};
    return m;
}

void eufiScenes(std::vector<Scene> &out) {
    // 1. At its stand, heard from the front quarter 35m off: the APU on from
    // the start, the engines started at 5s, idling, then taxiing away at 65s
    // (power 0.14), straight ahead, past the camera 25m off. The table's
    // second engine started at 35s isn't the caller's to say - Sound has one
    // `running`, as the 3D view has, for both - so it is the synthesiser's
    // own start sequence: the second engine 30s after the first. The APU:
    // the 3D view gives one only to an airliner today, and the recipe has
    // none; on here as the table has it, off as it taxies (NOTES.md).
    out.push_back({"eufi-stand-start", 80, [](float t) {
                       Moment m;
                       float d = t < 65 ? 0 : 0.25f * (t - 65) * (t - 65);  // 0.5m/s2, 7.5m/s by 80s
                       m.at = {d, 0, 0};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = t < 65 ? 0.05f : 0.14f;
                       m.gear = true;
                       m.apu = t < 65;
                       m.running = t >= 5;
                       m.eye = {24.75f, 24.75f, 2};
                       m.hardGround = true;
                       return m;
                   }});
    // 2. Taxiing past at 8m/s, 35m off, on the apron: taxi power, and one
    // breakaway burst - power 0.22, 50% N1 by the EJ200's row - from 9s to
    // 12s, coming, the intake ahead of it towards the camera.
    out.push_back({"eufi-taxi-past", 30, [](float t) {
                       Moment m;
                       m.at = {-120 + 8 * t, 0, 0};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = t >= 9 && t < 12 ? 0.22f : 0.14f;
                       m.gear = true;
                       m.eye = {0, 35, 1.5f};
                       m.hardGround = true;
                       return m;
                   }});
    // 3. The 3D view's take-off, heard from 150m off the centreline and 500m
    // from brake release, 4m up as the A320's: the brakes held at full dry
    // power (0.88) for the first 3s, released with reheat lit; lift-off 8s
    // after release - "brakes-off to lift-off < 8s", 1g, 80m/s after 320m -
    // then a 13.5 degree climb, reheat held to 3,000ft (at 51s) and cancelled
    // there, power 0.88, as the table has it. The gear up 3s after lift-off.
    // So the camera hears it pass 43m up, 82m/s, its jets already climbing.
    out.push_back({"eufi-takeoff", 60, [](float t) {
                       Moment m;
                       float speed, pitch;
                       m.at = takeoffPath(t, 3, 10, 80, 13.5f, speed, pitch);
                       m.heading = 0;
                       m.pitch = pitch;
                       bool lit = t >= 3 && m.at.z < FT3000;
                       m.power = lit ? 1.0f : 0.88f;
                       m.reheat = lit ? 1.0f : 0.0f;
                       m.gear = t < 3 + 8 + 3;
                       m.eye = {500, 150, 4};
                       return m;
                   }});
    // 4. Under the climb 3km from brake release: past overhead at 22s, 600m
    // up, 120m/s, climbing at 12.6 degrees (lift-off at 320m: 600m up 3km
    // along), in reheat - and, past 3,000ft 12s later, cancelled as the 3D
    // view would, power 0.88 as in eufi-takeoff. The microphone 1.5m up.
    out.push_back({"eufi-climbout-overhead", 45, [](float t) {
                       Moment m;
                       float pitch = 12.6f * (float)M_PI / 180, s = 120 * (t - 22);
                       m.at = {s * cosf(pitch), 0, 600 + s * sinf(pitch)};
                       m.heading = 0;
                       m.pitch = pitch;
                       bool lit = m.at.z < FT3000;
                       m.power = lit ? 1.0f : 0.88f;
                       m.reheat = lit ? 1.0f : 0.0f;
                       m.eye = {0, 0, 1.5f};
                       return m;
                   }});
    // 5. A display pass along the runway, 30m up at 154m/s (300 knots), past
    // the tower 230m off and 20m up, abeam at 20s: level at the 3D view's
    // 0.65 before; reheat lit at the pass (power 1), and from 21s a 5g pull
    // to 60 degrees (radius 483m, 3.3s), climbing on at 154m/s - and, as the
    // 3D view would, the reheat cancelled at 3,000ft (29s), power 1 still,
    // dry, as it gives a climb over 1,200ft a minute.
    out.push_back({"eufi-display-pass", 40, [](float t) {
                       Moment m;
                       const float v = 154, pull = 21, r = v * v / (5 * 9.81f), most = (float)M_PI / 3;
                       float x1 = v * (pull - 20);
                       m.heading = 0;
                       if (t < pull) {
                           m.at = {v * (t - 20), 0, 30};
                           m.pitch = 0;
                       } else {
                           float th = std::min(most, (t - pull) * v / r);
                           float x = x1 + r * sinf(th), z = 30 + r * (1 - cosf(th));
                           float after = std::max(0.0f, t - pull - most * r / v) * v;
                           m.at = {x + after * cosf(most), 0, z + after * sinf(most)};
                           m.pitch = th;
                       }
                       bool lit = t >= 20 && m.at.z < FT3000;
                       m.power = t < 20 ? 0.65f : 1.0f;
                       m.reheat = lit ? 1.0f : 0.0f;
                       m.eye = {0, 230, 20};
                       m.hardGround = true;
                       return m;
                   }});
    // 6. A fast low pass: 76m (250ft) up at 216m/s (420 knots), over the
    // ground 50m off the track, abeam at 15s; the 3D view's level 0.65, or
    // POWER= (0.85 is more like it, eufi-sources 11.6). No reheat.
    out.push_back({"eufi-low-pass", 30, [](float t) {
                       Moment m = levelPass(t, 216, 15, 76, 50, 1.5f);
                       m.power = g_lowPassPower;
                       return m;
                   }});
    // 7. The 3D view's FLY PAST camera on a fast one high up: 3,962m (13,000ft)
    // at 298m/s (579 knots, the old VANDAL22 case), the camera hung off to
    // the side by 0.8s of its speed (238m) and a third of that below,
    // set down afresh each 12s, as x-a320-intermediate's. The 3D view's level
    // 0.65 - which the integration may raise with speed (NOTES.md).
    out.push_back({"eufi-flypast", 36, [](float t) {
                       const float speed = 298, height = 3962, off = speed * 0.8f, cycle = 12;
                       int n = (int)(t / cycle);
                       Moment m;
                       m.at = {speed * t, 0, height};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.65f;
                       m.eye = {speed * (n * cycle + 7), (n % 2 ? -1 : 1) * off, height - 0.3f * off};
                       return m;
                   }});
    // 8. The chase at FL250 (7,620m), 263m/s (M0.85), the camera 54m behind
    // and 10m above, moving with it: cruise power 0.65 for 15s, 0.05 for the
    // descent's 15s, then 0.88 for 15s.
    out.push_back({"eufi-chase-cruise", 45, [](float t) {
                       Moment m;
                       m.at = {263 * t, 0, 7620};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = t < 15 ? 0.65f : t < 30 ? 0.05f : 0.88f;
                       m.eye = {m.at.x - 54, 0, m.at.z + 10};
                       return m;
                   }});
    // 9. The chase camera 54m behind and 10m above on the final approach,
    // from 120m up as the A320's: 70m/s on three degrees, gear down, power
    // 0.45 with its corrections; touched down at 32.7s, aerobraking to 50m/s
    // by 45s, idle, the reversers' flag on throughout (all above 60 knots).
    out.push_back({"eufi-approach-chase", 45, [](float t) {
                       Moment m = fighterLanding(t, 300 - 120 / 0.0524f, {0, 0, 0});
                       m.eye = {m.at.x - 54, 0, m.at.z + 10};
                       return m;
                   }});
    // 10. Beside the touchdown zone, 60m off the centreline and 2m up, as
    // a320-touchdown's: the approach heard coming from 3.6km, ahead of it -
    // its whine - past, the tyres' chirp at 51.4s, the spool-down, the
    // roll-out to taxiing by 79.6s. The approach's power hunting as in
    // eufi-approach-chase: an angle-of-attack approach hunts whoever hears it.
    out.push_back({"eufi-landing", 80, [](float t) { return fighterLanding(t, -3300, {320, 60, 2}); }});
    // x1. Zeltweg's: a display pass at 154m/s (300 knots) in reheat, 50m up
    // and 320m off a microphone 2.5m up over grass - 324m at the closest -
    // abeam at 20s; for LAF,max (115dB(A) measured) and the third-octave
    // shape.
    out.push_back({"x-eufi-zeltweg", 40, [](float t) {
                       Moment m = levelPass(t, 154, 20, 50, 320, 2.5f);
                       m.power = 1;
                       m.reheat = 1;
                       return m;
                   }});
    // x2. A close pass, as ILA 2022's (640505): 150m/s, 40m up and 70m off -
    // 80m at the closest - abeam at 7.5s. "High power": full, in reheat -
    // its crackle measured at 3.8-5.0, which the research puts in reheat,
    // not at full dry's 1-2 (eufi-sources 11.3, 11.4).
    out.push_back({"x-eufi-close-pass", 15, [](float t) {
                       Moment m = levelPass(t, 150, 7.5f, 40, 70, 1.5f);
                       m.power = 1;
                       m.reheat = 1;
                       return m;
                   }});
    // x3. Display passes about 100m off, as r69878's: 60m up and 80m off a
    // microphone 1.5m up, three passes 20s apart, turn about: at 300 knots
    // (154m/s) left to right at full dry (0.88); at 350 knots (180m/s) back
    // in reheat; at 250 knots (129m/s) left to right, dry, its reheat lit
    // as it passes. Between passes it is cut from one end of a pass to the
    // start of the next on the same side, 1.3-1.8km off, as a camera cut is -
    // heard at once from there, as the synthesiser does with a jump.
    out.push_back({"x-eufi-display-100m", 60, [](float t) {
                       int leg = std::min(2, (int)(t / 20));
                       float lt = t - 20 * leg;
                       const float V[3] = {154, 180, 129};
                       Moment m = levelPass(lt, V[leg], 10, 60, 80, 1.5f);
                       if (leg == 1) {
                           m.at.x = -m.at.x;
                           m.heading = (float)M_PI;
                       }
                       bool lit = leg == 1 || (leg == 2 && lt >= 10);
                       m.power = lit ? 1.0f : 0.88f;
                       m.reheat = lit ? 1.0f : 0.0f;
                       return m;
                   }});
}

// The ATR's scenes' figures - see the phase tables of atr72-sources.md 14
// and ATR's fact sheets. Its length against an A320's as the 3D view's
// sizeOf() has it: an ATR 72's 27.17m, an ATR 42's 22.67m, over 37.6m.
float propSize(Engine e) {
    return e == Engine::PW127_568F_42 || e == Engine::PW120_14SF ? 22.67f / 37.6f : 27.17f / 37.6f;
}
// The 3D view's height under which the ground about it is hard - the
// airfield's runways and aprons: on it, or under 300ft over it.
bool viewHard(float z) { return z < 300 * 0.3048f; }

// A turboprop's take-off along x from x = 0, its roll from `rollAt` at
// `accel`, off at `liftMs`, then climbing at `climbDeg` and gathering speed
// at `speedUp` to `topMs`; its speed and pitch out.
V3 propTakeoffPath(float t, float rollAt, float accel, float liftMs, float climbDeg, float speedUp, float topMs,
                   float &speed, float &pitch) {
    float tr = t - rollAt;
    speed = 0;
    pitch = 0;
    if (tr <= 0) {
        return {0, 0, 0};
    }
    float tLift = liftMs / accel;
    if (tr < tLift) {
        speed = accel * tr;
        return {accel * tr * tr / 2, 0, 0};
    }
    float a = tr - tLift, aTop = (topMs - liftMs) / speedUp, s;
    if (a < aTop) {
        speed = liftMs + speedUp * a;
        s = liftMs * a + speedUp * a * a / 2;
    } else {
        speed = topMs;
        s = liftMs * aTop + speedUp * aTop * aTop / 2 + topMs * (a - aTop);
    }
    pitch = climbDeg * (float)M_PI / 180;
    return {liftMs * liftMs / (2 * accel) + s * cosf(pitch), 0, s * sinf(pitch)};
}

// A turboprop taking off from x = 0 as the 3D view has it: on the runway's
// end at ground idle in beta until `lineUp`, then lined up - the runway
// simulation's take-off, its power and its propellers' speed coming up -
// rolling two seconds on; the simulation's take-off until 1,500ft over the
// field, where it is handed back to the feed, and a climb's from there.
Moment propTakeoff(float t, float lineUp, float accel, float liftMs, float climbDeg, V3 eye) {
    Moment m;
    float speed, pitch;
    m.at = propTakeoffPath(t, lineUp + 2, accel, liftMs, climbDeg, 0.5f, 80, speed, pitch);
    m.heading = 0;
    m.pitch = pitch;
    m.movement = t >= lineUp && m.at.z < 1500 * 0.3048f ? 1 : 0;
    m.eye = eye;
    m.hardGround = viewHard(m.at.z);
    return m;
}

// A turboprop's landing as the 3D view has it: in on a three-degree glide at
// `approachMs` from x0 - the runway simulation's landing from there, within
// its 8nm - touching down at x = 300, the threshold at nought; then braking
// at `brake` to taxiing at 8m/s on along the runway. Its reverse pitch
// above 60 knots, and beta below, as engines3d::ask() gives them.
Moment propLanding(float t, float x0, float approachMs, float brake, V3 eye) {
    Moment m;
    float tTouch = (300 - x0) / approachMs;
    m.heading = 0;
    m.pitch = 0;
    m.movement = 2;
    if (t < tTouch) {
        float x = x0 + approachMs * t;
        m.at = {x, 0, std::max(0.0f, (300 - x) * 0.0524f)};
    } else {
        float d = t - tTouch, dStop = (approachMs - 8) / brake;
        float s = d < dStop ? approachMs * d - brake * d * d / 2
                            : approachMs * dStop - brake * dStop * dStop / 2 + 8 * (d - dStop);
        m.at = {300 + s, 0, 0};
        // the spoilers the view draws - for a turboprop without ground
        // spoilers, engines3d::airframe() hears none
        float v = std::max(8.0f, approachMs - brake * d);
        float rise = clamp01(d / 0.8f), slow = clamp01((v / 0.5144f - 40) / 20);
        m.spoilers = rise * rise * (3 - 2 * rise) * slow * slow * (3 - 2 * slow);
    }
    m.eye = eye;
    m.hardGround = viewHard(m.at.z);
    return m;
}

// The ATR's, as the phase tables of atr72-sources.md 14 have them - an ATR
// 72-500/-600 unless ENGINE= says, an ATR 42-500/-600 in atr42-takeoff -
// each fed as the 3D view feeds a turboprop: its power and propellers'
// speed from engines3d::ask(), its gear and flaps as the view's set-up
// gives them, its spoilers through engines3d::airframe(), its propellers'
// axes 2.80m up on its wheels.
void atrScenes(std::vector<Scene> &out) {
    // 1. At its stand, heard from the apron 35m off at the front quarter (14.1):
    // hotel mode from the start - its right engine's gas generator running,
    // that propeller braked - its engines started at 8s, the left propeller
    // turning, then the brake released at 60s, the right one let go - the
    // view keeps hotel mode for 90s after a start (view3d.cpp), here 52s, to
    // hear the brake come off - and both idling in beta to the end.
    out.push_back({"atr72-start-idle", 90, [](float t) {
                       Moment m;
                       m.at = {0, 0, 0};
                       m.heading = 0;
                       m.pitch = 0;
                       m.running = t >= 8;
                       m.apu = t < 60;
                       m.eye = {24.75f, 24.75f, 2};
                       m.hardGround = true;
                       return m;
                   }});
    // 2. Taxiing out past the camera at 8m/s (16 knots), 35m off, abeam at
    // 15s, on the apron (14.2): beta, both propellers turning, no hotel mode.
    out.push_back({"atr72-taxi-past", 30, [](float t) {
                       Moment m;
                       m.at = {-120 + 8 * t, 0, 0};
                       m.heading = 0;
                       m.pitch = 0;
                       m.eye = {0, 35, 1.5f};
                       m.hardGround = true;
                       return m;
                   }});
    // 3. The take-off from beside the runway, 700m from where it lines up and
    // 120m off it, 4m up as the A320's (14.3): lined up at 4s, rolling at
    // 6s at 1.8m/s2, off at 57m/s (111 knots) after 900m - V2 116 knots,
    // 1,279m to 35ft at its heaviest [P: the fact sheet] - so past the
    // camera at 50m/s 28s into its roll, then climbing at 6 degrees, 1,320ft
    // a minute at 64m/s (1,355 at its heaviest [P]), gathering speed to
    // 80m/s; its take-off power and 100% to 1,500ft, some 100s in - beyond
    // this scene, 5km off.
    out.push_back({"atr72-takeoff", 75, [](float t) { return propTakeoff(t, 4, 1.8f, 57, 6, {700, 120, 4}); }});
    // 4. Under its climb (14.4), past overhead at 22s, 400m (1,310ft) up at
    // 75m/s, climbing at 6 degrees - its take-off power held till 1,500ft,
    // at 29s, then its climb's, its propellers gliding from 100% to 82% as
    // it goes away. The microphone 1.5m up, on grass.
    out.push_back({"atr72-climbout-flyover", 50, [](float t) {
                       Moment m;
                       float pitch = 6 * (float)M_PI / 180, s = 75 * (t - 22);
                       m.at = {s * cosf(pitch), 0, 400 + s * sinf(pitch)};
                       m.heading = 0;
                       m.pitch = pitch;
                       m.movement = m.at.z < 1500 * 0.3048f ? 1 : 0;
                       m.eye = {0, 0, 1.5f};
                       return m;
                   }});
    // 5. Cruising overhead high (14.5): FL170 (5,182m) at 275 knots (141m/s)
    // [P: the fact sheet's cruise], overhead at 35s, heard from the ground
    // 1.5m up - its cruise's power at 82%. Its sound 15s late overhead, more
    // than the synth's five seconds of what was asked (Synth::HEARD): heard
    // from where it was five seconds ago, its pass some ten seconds early
    // (NOTES.md).
    out.push_back({"atr72-cruise-flyover", 70, [](float t) {
                       Moment m;
                       m.at = {141 * (t - 35), 0, 5182};
                       m.heading = 0;
                       m.pitch = 0;
                       m.eye = {0, 0, 1.5f};
                       return m;
                   }});
    // 6. Its approach beside the camera (14.6): 300m up and 300m off at the
    // closest - as the Schiphol recording's uploader has it (fs315660, the
    // targets' main ATR measurement) - on a three-degree glide at 117 knots
    // (60m/s) [P: ANP's ATR 72 approach], abeam at 40s, its gear and flaps
    // out: 100% and its approach's power.
    out.push_back({"atr72-approach-side", 75, [](float t) {
                       Moment m;
                       float s = 60 * (t - 40);
                       m.at = {s, 0, 300 - s * 0.0524f};
                       m.heading = 0;
                       m.pitch = 0;
                       m.movement = 2;
                       m.eye = {0, 300, 1.5f};
                       m.hardGround = viewHard(m.at.z);
                       return m;
                   }});
    // 7. Its approach from behind (14.6): the 3D view's CHASE camera, 55 of
    // its sizes back - 39.7m for an ATR 72 - and 10 degrees up from its
    // middle, 3.5m over its wheels; down a three-degree glide at 60m/s from
    // 120m up, touching down at 38s, its reverse pitch to 60 knots and on
    // to taxiing.
    out.push_back({"atr72-approach-behind", 55, [](float t) {
                       Moment m = propLanding(t, 300 - 120 / 0.0524f, 60, 2.5f, {0, 0, 0});
                       float back = 55 * propSize(Engine::PW127_568F), el = 10 * (float)M_PI / 180;
                       m.eye = {m.at.x - back * cosf(el), 0, m.at.z + 3.5f + back * sinf(el)};
                       return m;
                   }});
    // 8. Its touchdown and reverse beside the touchdown zone (14.7), 50m off
    // the centreline and 2m up: in from 2.4km out at 60m/s, touching down at
    // 40s; its reverse pitch above 60 knots - 12s - braking at 2.5m/s2 to
    // taxiing by 61s, in beta.
    out.push_back({"atr72-touchdown-reverse", 75, [](float t) { return propLanding(t, -2100, 60, 2.5f, {320, 50, 2}); }});
    // 9. An ATR 42-500/-600's take-off from beside the runway, 600m from
    // where it lines up and 120m off: V2 112 knots, 1,107m to 35ft at its
    // heaviest, 1,851ft a minute climbing [P: its fact sheet] - so off at
    // 54m/s (105 knots) after 810m and climbing at 8.5 degrees.
    out.push_back({"atr42-takeoff", 75, [](float t) { return propTakeoff(t, 4, 1.8f, 54, 8.5f, {600, 120, 4}); }});
    // 10. Cruising as the 3D view shows one high up (view3d.cpp's CINEMA,
    // away from an airfield): close behind, then wide, then from below, ten
    // seconds each - each its orbit about its middle, 70, 230 and 120 of its
    // sizes off - at FL170 and 275 knots, level for 30s, then coming down at
    // 2,000ft a minute at flight idle, its propellers at 82% throughout.
    out.push_back({"atr72-cruise-cinema", 45, [](float t) {
                       Moment m;
                       float sink = t < 30 ? 0 : (t - 30) * 2000 * 0.00508f;
                       m.at = {141 * t, 0, 5182 - sink};
                       m.heading = 0;
                       m.pitch = t < 30 ? 0 : -asinf(2000 * 0.00508f / 141);
                       float phase = fmodf(t, 30), size = propSize(Engine::PW127_568F);
                       float az = phase < 10 ? 25 : phase < 20 ? 140 : 215, el = phase < 10 ? 9 : phase < 20 ? 22 : -18;
                       float far = (phase < 10 ? 70 : phase < 20 ? 230 : 120) * size;
                       float bearing = (float)M_PI + az * (float)M_PI / 180, e = el * (float)M_PI / 180;
                       m.eye = {m.at.x + cosf(e) * cosf(bearing) * far, cosf(e) * sinf(bearing) * far,
                                m.at.z + 3.5f + sinf(e) * far};
                       return m;
                   }});
}

std::vector<Scene> scenes() {
    std::vector<Scene> out;
    out.push_back({"a320-takeoff", 70, [](float t) { return takeoffSeen(t, 1200, 120, 75); }});
    out.push_back({"a320-landing", 90, [](float t) { return landing(t, {0, 70, 2}); }});
    out.push_back({"a320-touchdown", 80, [](float t) {
                       // Beside the touchdown zone, 45m off the centreline.
                       Moment m = landing(t, {320, 45, 2});
                       m.hardGround = true;
                       return m;
                   }});
    out.push_back({"a320-flypast", 30, [](float t) {
                       // Low and level along a runway at 75m/s, 30m up, past a
                       // camera on the tower's balcony 150m off it, 20m up:
                       // the sound off the runway sweeping as it goes by.
                       Moment m;
                       m.at = {-1100 + 75 * t, 0, 30};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.7f;
                       m.eye = {0, 150, 20};
                       m.hardGround = true;
                       return m;
                   }});
    out.push_back({"a320-arrival-park", 60, [](float t) {
                       // Taxiing in on to its stand, its APU on, heard from the
                       // apron 35m off: stopped at 14s, its engines shut down
                       // at 18s - a minute after parking, on the device; here
                       // sooner - winding down to the APU's whine alone.
                       Moment m;
                       float x = t < 10 ? -110 + 8 * t : t < 14 ? -30 + 8 * (t - 10) - (t - 10) * (t - 10) : -14;
                       m.at = {x, 0, 0};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.06f;
                       m.gear = true;
                       m.apu = true;
                       m.running = t < 18;
                       m.eye = {-14, 35, 2};
                       m.hardGround = true;
                       return m;
                   }});
    out.push_back({"a320-departure-start", 60, [](float t) {
                       // At its stand on its APU, heard from the apron 35m off:
                       // its engines started at 4s - the core turning on the
                       // starter, lit at a third of idle, on up to idle - then
                       // taxiing away at 42s.
                       Moment m;
                       float d = t < 42 ? 0 : (t - 42) * (t - 42) * 0.3f;
                       m.at = {std::min(d, 0.0f + 4 * (t - 42) + 30), 0, 0};
                       if (t < 42) {
                           m.at = {0, 0, 0};
                       }
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = t < 42 ? 0.05f : 0.14f;
                       m.gear = true;
                       m.apu = true;
                       m.running = t > 4;
                       m.eye = {0, 35, 2};
                       m.hardGround = true;
                       return m;
                   }});
    out.push_back({"a320-approach-chase", 42, [](float t) {
                       // The chase camera behind and above it on its final
                       // approach: flaps out, the gear coming down four
                       // seconds in, the air rushing past it, then the
                       // touchdown: the tyres, the spoilers and the reversers.
                       Moment m;
                       float tTouch = 120 / 3.7f;
                       m.heading = 0;
                       m.pitch = 0;
                       m.flaps = true;
                       m.gear = t > 4;
                       m.power = 0.45f;
                       if (t < tTouch) {
                           m.at = {70 * t, 0, 120 - 3.7f * t};
                       } else {
                           float d = t - tTouch, v = std::max(8.0f, 70 - 2.2f * d);
                           m.at = {70 * tTouch + 70 * d - 1.1f * d * d, 0, 0};
                           m.reverse = v > 30;
                           m.power = v > 30 ? 0.75f : 0.08f;
                           float rise = clamp01(d / 0.8f), slow = clamp01((v / 0.5144f - 40) / 20);
                           m.spoilers = rise * rise * (3 - 2 * rise) * slow * slow * (3 - 2 * slow);
                           m.hardGround = true;
                       }
                       m.eye = {m.at.x - 54, 0, m.at.z + 10};
                       return m;
                   }});
    out.push_back({"a320-chase", 45, [](float t) {
                       // Behind it and a little above, as the chase is: cruise, then
                       // the power back for the descent, then out for the approach.
                       Moment m;
                       m.at = {230 * t, 0, 11000};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = t < 15 ? 0.65f : t < 30 ? 0.0f : 0.45f;
                       m.eye = {m.at.x - 54, 0, m.at.z + 10};
                       return m;
                   }});
    // Heard from where real ones were recorded, for measuring against them
    // (the targets' matched scenes).
    // A climb-out overhead: full power, 8 degrees, 80 m/s, over a microphone
    // 1.2 m up at 300 m height - the DLR and Empa flyovers' sort.
    out.push_back({"x-a320-climbout-overhead", 45, [](float t) {
                       Moment m;
                       float pitch = 8 * (float)M_PI / 180, v = 80;
                       float s = v * (t - 22);  // overhead at 22 s
                       m.at = {s * cosf(pitch), 0, 300 + s * sinf(pitch)};
                       m.heading = 0;
                       m.pitch = pitch;
                       m.power = 1.0f;
                       m.eye = {0, 0, 1.2f};
                       return m;
                   }});
    // An approach overhead, as DLR's REC139: 88 m/s, descending 6 degrees,
    // 186 m over a ground microphone, gear and flaps out, approach power.
    out.push_back({"x-a320-approach-overhead", 30, [](float t) {
                       Moment m;
                       float s = 88 * (t - 15);
                       m.at = {s, 0, 186 - s * 0.1045f};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.45f;
                       m.gear = true;
                       m.flaps = true;
                       m.eye = {0, 0, 0.05f};
                       return m;
                   }});
    // An approach to the side, as Hoscalegeek's A319: 300 m up, 300 m off,
    // 70 m/s on a 3 degree glide, gear and flaps out, approach power.
    out.push_back({"x-a320-approach-side", 70, [](float t) {
                       Moment m;
                       float s = 70 * (t - 40);
                       m.at = {s, 0, 300 - s * 0.0524f};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.45f;
                       m.gear = true;
                       m.flaps = true;
                       m.eye = {0, 300, 1.5f};
                       return m;
                   }});
    // A taxi past at 8 m/s, 35 m off, idle (taxi) power, no APU.
    out.push_back({"x-a320-taxi-past", 30, [](float t) {
                       Moment m;
                       m.at = {-120 + 8 * t, 0, 0};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.08f;
                       m.gear = true;
                       m.eye = {0, 35, 1.5f};
                       m.hardGround = true;
                       return m;
                   }});
    // The touchdown from further off: the camera 250m from the touchdown
    // zone, its sound 0.73s after the picture - honest - or a tenth of a
    // second, with transients on the picture (SYNC=1): to choose between.
    out.push_back({"x-a320-touchdown-far", 50, [](float t) {
                       Moment m = landing(t + 30, {360, 245, 2});
                       m.hardGround = false;
                       return m;
                   }});
    // An intermediate approach: level at 3,000ft and 210 knots, flight idle,
    // the first stage of flap, the gear up - where an older A320's fuel
    // vents whistle - seen as the 3D view's FLY PAST camera hangs one, off
    // to the side by 1.2s of its speed and a third of that below it, set
    // down afresh each 12s.
    out.push_back({"x-a320-intermediate", 36, [](float t) {
                       const float speed = 108, height = 914, off = speed * 0.8f, cycle = 12;
                       int n = (int)(t / cycle);
                       Moment m;
                       m.at = {speed * t, 0, height};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.05f;
                       m.flaps = true;
                       m.eye = {speed * (n * cycle + 7), (n % 2 ? -1 : 1) * off, height - 0.3f * off};
                       return m;
                   }});
    // The Boeing 777's, its GE90-115Bs unless ENGINE= says: a take-off seen
    // from beside the runway further down, as a 777 lifts later and faster;
    // and heard from where the reference recordings were made
    // [b777-sources 7]: an approach 300m up and 300m off, as Hoscalegeek's
    // 777-200; one 40m over a microphone near the threshold, as Sydney's and
    // the BA 777-236ER's; and a landing 700m down the runway and 350m off,
    // as the FedEx 777F's at Sydney.
    out.push_back({"b777-takeoff", 75, [](float t) { return takeoffSeen(t, 1500, 140, 85); }});
    out.push_back({"x-b777-approach-side", 75, [](float t) {
                       Moment m;
                       float s = 72 * (t - 40);
                       m.at = {s, 0, 300 - s * 0.0524f};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.45f;
                       m.gear = true;
                       m.flaps = true;
                       m.eye = {0, 300, 1.5f};
                       return m;
                   }});
    out.push_back({"x-b777-approach-overhead", 40, [](float t) {
                       Moment m;
                       float s = 72 * (t - 20);
                       m.at = {s, 0, std::max(0.0f, 40 - s * 0.0524f)};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.45f;
                       m.gear = true;
                       m.flaps = true;
                       m.eye = {0, 0, 1.5f};
                       m.hardGround = true;
                       return m;
                   }});
    out.push_back({"x-b777-touchdown-side", 90, [](float t) { return landing(t, {700, 350, 2}); }});
    eufiScenes(out);
    // Last, so every scene before them - and DEVICE=1's frames' jitter,
    // drawn scene after scene - is as it was.
    atrScenes(out);
    return out;
}

void writeWav(const std::string &path, const std::vector<int16_t> &samples) {
    FILE *f = fopen(path.c_str(), "wb");
    if (f == nullptr) {
        perror(path.c_str());
        return;
    }
    auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
    auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
    uint32_t bytes = (uint32_t)samples.size() * 2;
    fwrite("RIFF", 1, 4, f);
    u32(36 + bytes);
    fwrite("WAVEfmt ", 1, 8, f);
    u32(16);
    u16(1);
    u16(1);
    u32(engine_noise::RATE);
    u32(engine_noise::RATE * 2);
    u16(2);
    u16(16);
    fwrite("data", 1, 4, f);
    u32(bytes);
    fwrite(samples.data(), 2, samples.size(), f);
    fclose(f);
}

// A biquad, as the RBJ cookbook has it.
struct Biquad {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    double run(double x) {
        double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }
    static Biquad highPass(double hz, double q) {
        double w = 2 * M_PI * hz / engine_noise::RATE, c = cos(w), alpha = sin(w) / (2 * q), a0 = 1 + alpha;
        Biquad f;
        f.b0 = (1 + c) / 2 / a0;
        f.b1 = -(1 + c) / a0;
        f.b2 = (1 + c) / 2 / a0;
        f.a1 = -2 * c / a0;
        f.a2 = (1 - alpha) / a0;
        return f;
    }
    static Biquad peak(double hz, double q, double db) {
        double w = 2 * M_PI * hz / engine_noise::RATE, c = cos(w), alpha = sin(w) / (2 * q), a = pow(10, db / 40),
               a0 = 1 + alpha / a;
        Biquad f;
        f.b0 = (1 + alpha * a) / a0;
        f.b1 = -2 * c / a0;
        f.b2 = (1 - alpha * a) / a0;
        f.a1 = -2 * c / a0;
        f.a2 = (1 - alpha / a) / a0;
        return f;
    }
};

// The 3D view's set-up for it, as scene3d's configFor() gives it: taking off,
// its flaps out on the ground and under 900m, its gear under 15m; on the
// ground otherwise, its flaps out over 40 knots, its gear down; coming in,
// its gear down under 600m and 210 knots, its flaps out under 1,200m and 230
// knots - which, the take-off handed back to the feed at 1,500ft, puts a
// climb's gear down again from there to 600m, as the view has it.
void viewConfig(float heightM, float kt, bool ground, bool departing, bool &gear, bool &flaps) {
    gear = true;
    flaps = false;
    if (departing) {
        flaps = ground || heightM < 900;
        gear = ground || heightM < 15;
    } else if (ground) {
        flaps = kt > 40;
    } else {
        gear = heightM < 600 && kt < 210;
        flaps = heightM < 1200 && kt < 230;
    }
}

// The Tab5's speaker, near enough: a fourth-order Butterworth high-pass at
// 300Hz - two second-order sections - and a gentle presence lift, 3dB at
// 3kHz, an octave and a bit wide.
std::vector<int16_t> tab5(const std::vector<int16_t> &in) {
    Biquad h1 = Biquad::highPass(300, 0.54119610), h2 = Biquad::highPass(300, 1.30656296),
           lift = Biquad::peak(3000, 0.9, 3.0);
    std::vector<int16_t> out(in.size());
    for (size_t i = 0; i < in.size(); i++) {
        double y = lift.run(h2.run(h1.run(in[i])));
        out[i] = (int16_t)std::max(-32768.0, std::min(32767.0, round(y)));
    }
    return out;
}

Engine engineNamed(const char *name) {
    if (name == nullptr || strcmp(name, "cfm56") == 0) {
        return Engine::CFM56_5B;
    }
    if (strcmp(name, "v2500") == 0) {
        return Engine::V2500_A5;
    }
    if (strcmp(name, "leap") == 0) {
        return Engine::LEAP_1A;
    }
    if (strcmp(name, "pw") == 0) {
        return Engine::PW1100G;
    }
    if (strcmp(name, "ge90") == 0) {
        return Engine::GE90_115B;
    }
    if (strcmp(name, "ge90-94") == 0) {
        return Engine::GE90_94B;
    }
    if (strcmp(name, "trent") == 0) {
        return Engine::TRENT_800;
    }
    // A turboprop's installation, for the ATR's scenes.
    if (strcmp(name, "atr72-600") == 0) {
        return Engine::PW127_568F;
    }
    if (strcmp(name, "atr72-200") == 0) {
        return Engine::PW124_14SF;
    }
    if (strcmp(name, "atr72-210") == 0) {
        return Engine::PW127_247F;
    }
    if (strcmp(name, "atr42-500") == 0) {
        return Engine::PW127_568F_42;
    }
    if (strcmp(name, "atr42-300") == 0) {
        return Engine::PW120_14SF;
    }
    fprintf(stderr, "ENGINE=%s: cfm56, v2500, leap, pw, ge90, ge90-94, trent, atr72-600, atr72-200, atr72-210, "
                    "atr42-500 or atr42-300\n", name);
    exit(1);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s outdir [scene]\n", argv[0]);
        return 1;
    }
    std::string dir = argv[1];
    bool device = getenv("DEVICE") != nullptr;
    engine_noise::g_transientsOnPicture = getenv("SYNC") != nullptr;
    // ENGINE= a turbofan's for the airliners' scenes, a propeller
    // installation's for the turboprops' - each kind's own default for the
    // other's.
    Engine named = engineNamed(getenv("ENGINE"));
    bool namedProp = engine_noise::isProp(named);
    Engine engine = namedProp ? Engine::CFM56_5B : named;
    bool engineSet = getenv("ENGINE") != nullptr && !namedProp;
    bool fopp = getenv("FOPP") != nullptr;
    uint32_t who = getenv("WHO") ? (uint32_t)strtoul(getenv("WHO"), nullptr, 0) : 0x406a3cu + (uint32_t)engine;
    std::string suffix = getenv("SUFFIX") ? getenv("SUFFIX") : "";
    if (getenv("POWER")) {
        g_lowPassPower = (float)atof(getenv("POWER"));
    }
    // The Typhoon's own airframe unless WHO= says.
    uint32_t whoFighter = getenv("WHO") ? who : 0x43c6f1u;
    bool trace = getenv("TRACE") != nullptr;
    for (const Scene &scene : scenes()) {
        if (argc > 2 && strcmp(argv[2], scene.name) != 0) {
            continue;
        }
        bool fighter = strstr(scene.name, "eufi-") != nullptr;
        // A turboprop's scenes: an ATR 72-500/-600's, an ATR 42-500/-600's
        // in atr42-*, unless ENGINE= names another propeller installation.
        bool prop = strncmp(scene.name, "atr", 3) == 0;
        Engine propEngine = namedProp                             ? named
                            : strncmp(scene.name, "atr42", 5) == 0 ? Engine::PW127_568F_42
                                                                   : Engine::PW127_568F;
        uint32_t whoProp = getenv("WHO") ? who : 0x406a3cu + (uint32_t)propEngine;
        engine_noise::Synth synth;
        std::vector<int16_t> samples;
        const int BLOCK = engine_noise::RATE / 50;  // a frame's worth
        float touchdownAt = -1, touchdownSink = 1, lastZ = -1, frameMs = 0, renderedMs = 0;
        V3 lastAt = {0, 0, 0};
        double sum = 0;
        int peak = 0, counted = 0;
        printf("%s%s:", scene.name, suffix.c_str());
        for (float t = 0; t < scene.seconds; t += 1.0f / 50) {
            Moment m = scene.at(t);
            Sound s;
            s.kind = Kind::TURBOFAN;
            // A 777's scenes its GE90-115Bs unless ENGINE= says.
            s.engine = strstr(scene.name, "b777-") && !engineSet ? Engine::GE90_115B : engine;
            s.fanHz = 3000;
            s.engines = 2;
            s.who = who;
            s.fopp = fopp;
            s.power = m.power;
            s.reverse = m.reverse;
            V3 mid = {m.at.x, m.at.y, m.at.z + 3};
            if (fighter) {
                // A Typhoon, as the 3D view will give one: its kind and
                // engines from its type, EUFI; its size; its reheat, the view
                // knowing only lit or not; its jets low on its gear.
                s.kind = Kind::LOW_BYPASS;
                s.engine = Engine::EJ200;
                s.who = whoFighter;
                s.fopp = false;
                s.size = EUFI_SIZE;
                s.reheat = m.reheat;
                s.afterburner = m.reheat > 0;
                mid.z = m.at.z + EUFI_ENGINES_UP;
            }
            if (prop) {
                // A turboprop, as the 3D view gives one: its kind, engines
                // and propellers from its type; its size; its propellers'
                // axes as high as its row has them.
                const engine_noise::PropData &p = engine_noise::propData(propEngine);
                s.kind = Kind::TURBOPROP;
                s.engine = propEngine;
                s.engines = p.engines;
                s.who = whoProp;
                s.fopp = false;
                s.size = propSize(propEngine);
                mid.z = m.at.z + p.axisUp;
            }
            V3 to = m.eye - mid;
            float d = length(to);
            s.distanceM = d;
            V3 nose = {cosf(m.heading) * cosf(m.pitch), sinf(m.heading) * cosf(m.pitch), sinf(m.pitch)};
            float c = (nose.x * to.x + nose.y * to.y + nose.z * to.z) / std::max(1.0f, d);
            s.aspect = acosf(std::max(-1.0f, std::min(1.0f, c))) / (float)M_PI;
            // Off the ground: its engines' height and the camera's. Its
            // tyres: rolling at its speed on the ground; touched down, how
            // long ago and how fast it was coming down - held from then on,
            // as the 3D view holds it.
            s.sourceUpM = mid.z;
            s.earUpM = m.eye.z;
            s.hardGround = m.hardGround;
            bool onGround = m.at.z <= 0.01f;
            V3 moved = m.at - lastAt;
            float speed = t > 0 ? length(moved) * 50 : 0;
            if (onGround && lastZ > 0.01f) {
                touchdownAt = t;
                touchdownSink = (lastZ - m.at.z) * 50;
            }
            s.sinkMs = touchdownSink;
            s.rollingMs = onGround ? speed : 0;
            s.touchdownS = touchdownAt >= 0 && t - touchdownAt < 3 ? t - touchdownAt : -1;
            s.airspeedMs = speed;
            s.gearDown = m.gear;
            s.flapsOut = m.flaps;
            s.spoilers = m.spoilers;
            s.running = m.running;
            s.apu = m.apu;
            if (prop) {
                // ...and what it is doing, as the view works it out: its power,
                // propellers' speed and reverse pitch by engines3d::ask(), from
                // the speed and climb its path gives and the runway
                // simulation's take-off or landing; its gear and flaps as the
                // view's set-up gives them, and its spoilers - none on an ATR -
                // through engines3d::airframe().
                engines3d::Doing doing;
                doing.ground = onGround;
                doing.takingOff = m.movement == 1;
                doing.landing = m.movement == 2;
                doing.simulating = m.movement != 0;
                doing.kt = (int)lroundf(speed / 0.5144f);
                doing.rollKt = doing.kt;
                doing.vsFpm = t > 0 ? (int)lroundf((m.at.z - lastZ) * 50 / 0.00508f) : 0;
                doing.altFt = (int)lroundf(m.at.z / 0.3048f);
                doing.low = m.at.z < 3000 * 0.3048f;
                s.reverse = false;
                engines3d::ask(s, doing);
                bool gear, flaps;
                viewConfig(m.at.z, (float)doing.kt, onGround, doing.takingOff, gear, flaps);
                engines3d::airframe(s, gear, flaps, m.spoilers);
            }
            s.placed = true;
            s.at[0] = mid.x, s.at[1] = mid.y, s.at[2] = mid.z;
            s.nose[0] = nose.x, s.nose[1] = nose.y, s.nose[2] = nose.z;
            s.ear[0] = m.eye.x, s.ear[1] = m.eye.y, s.ear[2] = m.eye.z;
            lastAt = m.at;
            lastZ = m.at.z;
            s.ms = (uint32_t)lroundf(t * 1000);
            if (trace && lroundf(t * 50) % 50 == 0) {
                fprintf(stderr,
                        "%5.1fs at %7.0f %5.0f %6.0f  %4.0fm aspect %.2f  %3.0fm/s power %.2f reheat %.0f%s%s%s%s "
                        "air-brake %.2f rolling %.0f touchdown %.1f %s%s\n",
                        t, s.at[0], s.at[1], s.at[2], d, s.aspect, s.airspeedMs, s.power, s.reheat,
                        s.afterburner ? " lit" : "", s.reverse ? " reverse" : "", s.gearDown ? " gear" : "",
                        s.hardGround ? " hard" : "", s.spoilers, s.rollingMs, s.touchdownS, s.running ? "" : "off",
                        s.apu ? " apu" : "");
                if (prop) {
                    fprintf(stderr, "        %s: propellers asked %.0f%%%s\n", engine_noise::propData(s.engine).name,
                            100 * s.np, s.flapsOut ? ", flaps" : "");
                }
            }
            synth.set(s);
            // As the device has it, DEVICE=1: frames 20ms apart give or take a
            // third, the sound made in 40ms blocks whenever one is due.
            int block = BLOCK;
            if (device) {
                static uint32_t jitter = 12345;
                jitter = jitter * 1103515245u + 12345u;
                frameMs += 20 * (0.67f + 0.66f * ((jitter >> 8) & 0xFFFF) / 65535.0f);
                block = 0;
                while (renderedMs + 40 <= frameMs) {
                    block += engine_noise::RATE / 25;
                    renderedMs += 40;
                }
                t = frameMs / 1000.0f - 1.0f / 50;  // the loop adds the 20ms back
            }
            size_t from = samples.size();
            samples.resize(from + block);
            synth.render(samples.data() + from, block);
            for (int i = 0; i < block; i++) {
                int v = samples[from + i];
                sum += (double)v * v;
                peak = std::max(peak, abs(v));
            }
            counted += block;
            if (counted >= 2 * engine_noise::RATE) {
                double rms = sqrt(sum / counted) / 32768;
                printf(" %.0f", rms > 0 ? 20 * log10(rms) : -99);
                sum = 0;
                counted = 0;
            }
        }
        printf("  (dBFS every 2s; peak %.0f dBFS)\n", peak > 0 ? 20 * log10(peak / 32768.0) : -99);
#ifdef ENGINE_NOISE_DEBUG
        fprintf(stderr, "%s: the limiter rounded off %llu of %llu samples (%.4f%%)\n", scene.name,
                (unsigned long long)engine_noise::g_limited, (unsigned long long)engine_noise::g_samples,
                100.0 * engine_noise::g_limited / std::max<uint64_t>(1, engine_noise::g_samples));
        engine_noise::g_limited = engine_noise::g_samples = 0;
#endif
        writeWav(dir + "/" + scene.name + suffix + ".wav", samples);
        writeWav(dir + "/" + scene.name + suffix + "-tab5.wav", tab5(samples));
    }
    // The splash screen's take-off, as the device makes it at boot: its sound
    // given whole by include/splash_takeoff.h, not worked out from a Moment
    // as the scenes' are, set a 40ms block at a time - and two seconds more,
    // to its fading out.
    if (argc <= 2 || strcmp(argv[2], "a320-splash") == 0) {
        engine_noise::Synth synth;
        std::vector<int16_t> samples;
        const int BLOCK = engine_noise::RATE / 25;
        double sum = 0;
        int peak = 0, counted = 0;
        printf("a320-splash%s:", suffix.c_str());
        for (int i = 0; i * 0.04f < splash_takeoff::END_S + 2; i++) {
            synth.set(splash_takeoff::at(i * 0.04f));
            size_t from = samples.size();
            samples.resize(from + BLOCK);
            synth.render(samples.data() + from, BLOCK);
            for (int k = 0; k < BLOCK; k++) {
                int v = samples[from + k];
                sum += (double)v * v;
                peak = std::max(peak, abs(v));
            }
            counted += BLOCK;
            if (counted >= engine_noise::RATE / 2) {
                double rms = sqrt(sum / counted) / 32768;
                printf(" %.0f", rms > 0 ? 20 * log10(rms) : -99);
#ifdef ENGINE_NOISE_DEBUG
                // and how many samples of that half second the limiter rounded off
                static uint64_t limitedWas = 0;
                if (engine_noise::g_limited > limitedWas) {
                    printf("(%llu)", (unsigned long long)(engine_noise::g_limited - limitedWas));
                }
                limitedWas = engine_noise::g_limited;
#endif
                sum = 0;
                counted = 0;
            }
        }
        printf("  (dBFS every 0.5s; peak %.0f dBFS)\n", peak > 0 ? 20 * log10(peak / 32768.0) : -99);
#ifdef ENGINE_NOISE_DEBUG
        fprintf(stderr, "a320-splash: the limiter rounded off %llu of %llu samples (%.4f%%)\n",
                (unsigned long long)engine_noise::g_limited, (unsigned long long)engine_noise::g_samples,
                100.0 * engine_noise::g_limited / std::max<uint64_t>(1, engine_noise::g_samples));
        engine_noise::g_limited = engine_noise::g_samples = 0;
#endif
        writeWav(dir + "/a320-splash" + suffix + ".wav", samples);
        writeWav(dir + "/a320-splash" + suffix + "-tab5.wav", tab5(samples));
    }
    return 0;
}
