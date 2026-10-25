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
// ENGINE=cfm56 (the default), v2500, leap or pw: the airframe's engines.
// FOPP=1: its fuel vents unmodified, whistling. SYNC=1: the touchdown's
// squeal and the reversers' coming on heard as the picture shows them -
// engine_noise::g_transientsOnPicture. SUFFIX=-x: added to the files' names.
// WHO=n: the airframe, for its own little differences. DEVICE=1: frames
// 20ms apart give or take a third and the sound made in 40ms blocks, as the
// device has it.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "engine_noise.h"

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
    float heading, pitch;
    float power;
    bool reverse = false;
    V3 eye;
    bool hardGround = false;  // the ground between it and the camera a runway's, not grass
    bool gear = false, flaps = false;  // down, out
    float spoilers = 0;               // up, 0 to 1
    bool running = true, apu = false;  // its engines, its APU
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
    fprintf(stderr, "ENGINE=%s: cfm56, v2500, leap or pw\n", name);
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
    Engine engine = engineNamed(getenv("ENGINE"));
    bool fopp = getenv("FOPP") != nullptr;
    uint32_t who = getenv("WHO") ? (uint32_t)strtoul(getenv("WHO"), nullptr, 0) : 0x406a3cu + (uint32_t)engine;
    std::string suffix = getenv("SUFFIX") ? getenv("SUFFIX") : "";
    for (const Scene &scene : scenes()) {
        if (argc > 2 && strcmp(argv[2], scene.name) != 0) {
            continue;
        }
        engine_noise::Synth synth;
        std::vector<int16_t> samples;
        const int BLOCK = engine_noise::RATE / 50;  // a frame's worth
        float touchdownAt = -1, lastZ = -1, frameMs = 0, renderedMs = 0;
        V3 lastAt = {0, 0, 0};
        double sum = 0;
        int peak = 0, counted = 0;
        printf("%s%s:", scene.name, suffix.c_str());
        for (float t = 0; t < scene.seconds; t += 1.0f / 50) {
            Moment m = scene.at(t);
            Sound s;
            s.kind = Kind::TURBOFAN;
            s.engine = engine;
            s.fanHz = 3000;
            s.engines = 2;
            s.who = who;
            s.fopp = fopp;
            s.power = m.power;
            s.reverse = m.reverse;
            V3 mid = {m.at.x, m.at.y, m.at.z + 3};
            V3 to = m.eye - mid;
            float d = length(to);
            s.distanceM = d;
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
            s.placed = true;
            s.at[0] = mid.x, s.at[1] = mid.y, s.at[2] = mid.z;
            s.nose[0] = nose.x, s.nose[1] = nose.y, s.nose[2] = nose.z;
            s.ear[0] = m.eye.x, s.ear[1] = m.eye.y, s.ear[2] = m.eye.z;
            lastAt = m.at;
            lastZ = m.at.z;
            s.ms = (uint32_t)lroundf(t * 1000);
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
        writeWav(dir + "/" + scene.name + suffix + ".wav", samples);
        writeWav(dir + "/" + scene.name + suffix + "-tab5.wav", tab5(samples));
    }
    return 0;
}
