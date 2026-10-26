// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

// What the engine noise costs to make on this PC: the time for a 40ms block
// (640 samples at 16kHz) in a few typical states, the Sound set once a block
// as the device's sound task sets it - for scale only, the Tab5's RISC-V core
// at 360MHz being a very different machine: the old synthesiser measured
// 3.5% of one of its cores and 0.12-0.17% of one of this i7-1260P's, so the
// Tab5's share is some 25 times the PC's. Build and run, from the repo's
// root:
//
//   g++ -std=c++17 -O2 -Iinclude tools/engine_preview/bench.cpp src/engine_noise.cpp -o /tmp/engine_bench
//   /tmp/engine_bench
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "engine_noise.h"

using engine_noise::Engine;
using engine_noise::Kind;
using engine_noise::Sound;

int main() {
    struct Case {
        const char *name;
        Engine engine;
        float power, rolling, airspeed, x0, vx, z;
        bool gear, flaps, reverse, apu, fopp;
        float spoilers;
    };
    std::vector<Case> cases = {
        {"take-off roll, CFM56, full power", Engine::CFM56_5B, 1.0f, 50, 50, -600, 50, 0, false, false, false, false, false, 0},
        {"take-off roll, V2500, full power", Engine::V2500_A5, 1.0f, 50, 50, -600, 50, 0, false, false, false, false, false, 0},
        {"climb-out, CFM56, far", Engine::CFM56_5B, 1.0f, 0, 80, -2000, 80, 400, false, false, false, false, false, 0},
        {"approach, CFM56, gear+flaps, vents", Engine::CFM56_5B, 0.45f, 0, 70, -1500, 70, 150, true, true, false, false, true, 0},
        {"touchdown + reverse, spoilers", Engine::CFM56_5B, 0.75f, 60, 60, -50, 60, 0, true, true, true, false, false, 1},
        {"taxi with the APU", Engine::CFM56_5B, 0.08f, 8, 8, -100, 8, 0, true, false, false, true, false, 0},
        {"take-off roll, LEAP-1A", Engine::LEAP_1A, 1.0f, 50, 50, -600, 50, 0, false, false, false, false, false, 0},
    };
    const int BLOCK = engine_noise::RATE / 25;  // 40 ms
    const int BLOCKS = 4000;                    // 160 s of sound
    for (const Case &c : cases) {
        engine_noise::Synth synth;
        std::vector<int16_t> out(BLOCK);
        std::vector<double> times;
        double total = 0;
        int busy = 0;
        for (int b = 0; b < BLOCKS; b++) {
            float t = b * 0.04f;
            Sound s;
            s.kind = Kind::TURBOFAN;
            s.engine = c.engine;
            s.fopp = c.fopp;
            s.who = 0x406a3c;
            s.power = c.power;
            s.reverse = c.reverse;
            s.rollingMs = c.rolling;
            s.airspeedMs = c.airspeed;
            s.gearDown = c.gear;
            s.flapsOut = c.flaps;
            s.spoilers = c.spoilers;
            s.apu = c.apu;
            s.placed = true;
            float x = c.x0 + c.vx * fmodf(t, 40.0f);
            s.at[0] = x, s.at[1] = 0, s.at[2] = c.z + 3;
            s.nose[0] = 1, s.nose[1] = 0, s.nose[2] = 0;
            s.ear[0] = 0, s.ear[1] = 60, s.ear[2] = 1.5f;
            s.sourceUpM = c.z + 3;
            s.earUpM = 1.5f;
            s.hardGround = true;
            s.ms = (uint32_t)(t * 1000);
            auto b0 = std::chrono::steady_clock::now();
            synth.set(s);
            synth.render(out.data(), BLOCK);
            double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - b0).count();
            if (b > 10) {
                times.push_back(us);
                total += us;
                busy = std::max(busy, synth.busy());
            }
        }
        std::sort(times.begin(), times.end());
        double mean = total / times.size(), med = times[times.size() / 2], p99 = times[times.size() * 99 / 100];
        printf("%-36s mean %6.1f us, median %6.1f, p99 %6.1f a 40ms block = %.3f%% of a PC core (%.0f ns a sample); "
               "Tab5 ~%.1f%%; up to %d resonances and bands\n",
               c.name, mean, med, p99, 100.0 * mean / 40000.0, 1000.0 * mean / BLOCK, 25 * 100.0 * mean / 40000.0,
               busy);
    }
    return 0;
}
