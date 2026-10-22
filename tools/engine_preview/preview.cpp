// Plays the 3D view's engine noise (src/engine_noise.cpp) on the PC: an
// aircraft flown past a camera, the synthesiser set from it fifty times a
// second as the 3D view sets it - how far, how fast closing, which way it
// faces, what its engines are doing - and each scene saved as a WAV to
// listen to, with its loudness every two seconds. Build and run, from the
// repo's root:
//
//   g++ -std=c++17 -O2 -Iinclude tools/engine_preview/preview.cpp src/engine_noise.cpp -o /tmp/engine_preview
//   /tmp/engine_preview /tmp/engines [scene]
//
// The scenes: a320-takeoff, a320-landing, a320-touchdown, a320-flypast,
// a320-approach-chase, a320-arrival-park, a320-departure-start, a320-chase, b777-takeoff, a380-takeoff, atr72-takeoff, c172-flyby,
// h145-visit, aw139-flyby, chinook-flyby, typhoon-takeoff, hawk-flyby.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <functional>
#include <string>
#include <vector>

#include "engine_noise.h"

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
    float heading, pitch;
    float power;
    bool reverse = false, afterburner = false;
    float slap = 0;
    V3 eye;
    bool hardGround = false;  // the ground between it and the camera a runway's, not grass
    bool gear = false, flaps = false;  // down, out
    float spoilers = 0;               // up, 0 to 1
    bool running = true, apu = false;  // its engines, its APU
};

struct Scene {
    const char *name;
    float seconds;
    Sound type;  // the engines' kind and figures
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

Sound turbofan(float fanHz, int engines) {
    Sound s;
    s.kind = Kind::TURBOFAN;
    s.fanHz = fanHz;
    s.engines = (uint8_t)engines;
    return s;
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
    // Its flaps out; its gear coming down a few seconds in, late, for its
    // clunk to be heard; the spoilers springing up as it touches down and
    // coming down as it slows from 60kt to 40, as the 3D view has them.
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

std::vector<Scene> scenes() {
    std::vector<Scene> out;
    out.push_back({"a320-takeoff", 70, turbofan(3100, 2), [](float t) { return takeoffSeen(t, 1200, 120, 75); }});
    out.push_back({"b777-takeoff", 75, turbofan(1100, 2), [](float t) { return takeoffSeen(t, 1500, 140, 85); }});
    out.push_back({"a380-takeoff", 80, turbofan(1200, 4), [](float t) { return takeoffSeen(t, 1700, 160, 85); }});
    out.push_back({"a320-landing", 90, turbofan(3100, 2), [](float t) { return landing(t, {0, 70, 2}); }});
    out.push_back({"a320-touchdown", 80, turbofan(3100, 2), [](float t) {
                       // Beside the touchdown zone, 45m off the centreline.
                       Moment m = landing(t, {320, 45, 2});
                       m.hardGround = true;
                       return m;
                   }});
    out.push_back({"a320-flypast", 30, turbofan(3100, 2), [](float t) {
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
    out.push_back({"a320-arrival-park", 60, turbofan(3100, 2), [](float t) {
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
    out.push_back({"a320-departure-start", 60, turbofan(3100, 2), [](float t) {
                       // At its stand on its APU, heard from the apron 35m off:
                       // its engines started at 4s - the core's whine rising on
                       // the starter, lit at a third of idle, on up to idle -
                       // then taxiing away at 42s.
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
    out.push_back({"a320-approach-chase", 42, turbofan(3100, 2), [](float t) {
                       // The chase camera behind and above it on its final
                       // approach: flaps out, the gear coming down four
                       // seconds in - its clunk as it locks - the air rushing
                       // past it, then the touchdown: the tyres, the spoilers'
                       // buffet and the reversers.
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
    out.push_back({"a320-chase", 45, turbofan(3100, 2), [](float t) {
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
    Sound atr;
    atr.kind = Kind::TURBOPROP;
    atr.bladeHz = 121;
    out.push_back({"atr72-takeoff", 60, atr, [](float t) { return takeoffSeen(t, 900, 100, 58); }});
    Sound c172;
    c172.kind = Kind::PISTON;
    c172.engines = 1;
    out.push_back({"c172-flyby", 30, c172, [](float t) {
                       Moment m;
                       m.at = {-750 + 50 * t, 0, 150};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = t < 20 ? 0.75f : 0.4f;
                       m.eye = {0, 120, 2};
                       return m;
                   }});
    Sound h145;
    h145.kind = Kind::ROTOR;
    h145.bladeHz = 26;
    h145.tailHz = 600;
    out.push_back({"h145-visit", 150, h145, [](float t) {
                       // In from 2.6km, slowing to a hover 12m up over a spot 110m from
                       // the camera, down for a while, then up and away.
                       Moment m;
                       const float V = 46, D = 2600, A = V * V / (2 * D), tIn = V / A;
                       m.heading = 0;
                       if (t < tIn) {
                           float left = D - (V * t - A * t * t / 2), v = V - A * t;
                           m.at = {-left, 0, 12 + 293 * left / D};
                           m.power = 0.55f;
                           m.slap = clamp01((v - 8) / 12) * clamp01((40 - v) / 10) * 0.9f;
                           m.pitch = 0;
                       } else if (t < tIn + 12) {
                           m.at = {0, 0, 12 * (1 - (t - tIn) / 12)};
                           m.power = 0.95f;
                           m.pitch = 0;
                       } else if (t < tIn + 40) {
                           m.at = {0, 0, 0};
                           m.power = 0.25f;
                           m.pitch = 0;
                       } else {
                           float d = t - tIn - 40, s = 0.6f * d * d;
                           m.at = {s, 0, std::min(150.0f, 3 + 4.6f * d)};
                           m.power = 0.95f;
                           m.pitch = 0;
                       }
                       m.eye = {30, 105, 4};
                       return m;
                   }});
    Sound aw139;
    aw139.kind = Kind::ROTOR;
    aw139.bladeHz = 27;
    aw139.tailHz = 120;
    out.push_back({"aw139-flyby", 30, aw139, [](float t) {
                       Moment m;
                       m.at = {-900 + 60 * t, 0, 60};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.7f;
                       m.eye = {0, 80, 2};
                       return m;
                   }});
    Sound chinook;
    chinook.kind = Kind::ROTOR;
    chinook.bladeHz = 11.25f;
    chinook.tailHz = 0;
    chinook.tandem = true;
    out.push_back({"chinook-flyby", 30, chinook, [](float t) {
                       Moment m;
                       m.at = {-900 + 60 * t, 0, 80};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.8f;
                       m.eye = {0, 90, 2};
                       return m;
                   }});
    Sound typhoon;
    typhoon.kind = Kind::LOW_BYPASS;
    typhoon.fanHz = 4500;
    out.push_back({"typhoon-takeoff", 50, typhoon, [](float t) {
                       Moment m = takeoffSeen(t, 700, 100, 80);
                       m.afterburner = t > 7 && t < 35;
                       return m;
                   }});
    Sound hawk;
    hawk.kind = Kind::LOW_BYPASS;
    hawk.fanHz = 4000;
    out.push_back({"hawk-flyby", 20, hawk, [](float t) {
                       Moment m;
                       m.at = {-1500 + 150 * t, 0, 90};
                       m.heading = 0;
                       m.pitch = 0;
                       m.power = 0.85f;
                       m.eye = {0, 100, 2};
                       return m;
                   }});
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

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s outdir [scene]\n", argv[0]);
        return 1;
    }
    std::string dir = argv[1];
    for (const Scene &scene : scenes()) {
        if (argc > 2 && strcmp(argv[2], scene.name) != 0) {
            continue;
        }
        engine_noise::Synth synth;
        std::vector<int16_t> samples;
        const int BLOCK = engine_noise::RATE / 50;  // a frame's worth
        float lastDistance = -1, touchdownAt = -1, lastZ = -1;
        V3 lastAt = {0, 0, 0};
        double sum = 0;
        int peak = 0, counted = 0;
        printf("%s:", scene.name);
        for (float t = 0; t < scene.seconds; t += 1.0f / 50) {
            Moment m = scene.at(t);
            Sound s = scene.type;
            s.power = m.power;
            s.reverse = m.reverse;
            s.afterburner = m.afterburner;
            s.slap = m.slap;
            V3 mid = {m.at.x, m.at.y, m.at.z + 3};
            V3 to = m.eye - mid;
            float d = length(to);
            s.distanceM = d;
            s.closingMs = lastDistance < 0 ? 0 : (lastDistance - d) * 50;
            lastDistance = d;
            V3 nose = {cosf(m.heading) * cosf(m.pitch), sinf(m.heading) * cosf(m.pitch), sinf(m.pitch)};
            float c = (nose.x * to.x + nose.y * to.y + nose.z * to.z) / std::max(1.0f, d);
            s.aspect = acosf(std::max(-1.0f, std::min(1.0f, c))) / (float)M_PI;
            // Off the ground: its engines' height and the camera's. Its
            // tyres: rolling at its speed on the ground; touched down, how
            // long ago and how fast it was coming down.
            s.sourceUpM = mid.z;
            s.earUpM = m.eye.z;
            s.hardGround = m.hardGround;
            bool onGround = m.at.z <= 0.01f;
            V3 moved = m.at - lastAt;
            float speed = t > 0 ? length(moved) * 50 : 0;
            if (onGround && lastZ > 0.01f) {
                touchdownAt = t;
                s.sinkMs = (lastZ - m.at.z) * 50;
            }
            s.rollingMs = onGround ? speed : 0;
            s.touchdownS = touchdownAt >= 0 && t - touchdownAt < 3 ? t - touchdownAt : -1;
            s.airspeedMs = speed;
            s.gearDown = m.gear;
            s.flapsOut = m.flaps;
            s.spoilers = m.spoilers;
            s.running = m.running;
            s.apu = m.apu;
            lastAt = m.at;
            lastZ = m.at.z;
            synth.set(s);
            size_t from = samples.size();
            samples.resize(from + BLOCK);
            synth.render(samples.data() + from, BLOCK);
            for (int i = 0; i < BLOCK; i++) {
                int v = samples[from + i];
                sum += (double)v * v;
                peak = std::max(peak, abs(v));
            }
            counted += BLOCK;
            if (counted >= 2 * engine_noise::RATE) {
                double rms = sqrt(sum / counted) / 32768;
                printf(" %.0f", rms > 0 ? 20 * log10(rms) : -99);
                sum = 0;
                counted = 0;
            }
        }
        printf("  (dBFS every 2s; peak %.0f dBFS)\n", peak > 0 ? 20 * log10(peak / 32768.0) : -99);
        writeWav(dir + "/" + scene.name + ".wav", samples);
    }
    return 0;
}
