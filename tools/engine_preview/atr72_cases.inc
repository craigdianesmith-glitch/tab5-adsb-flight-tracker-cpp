// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

// The ATR's cost cases for eufi-tools/cost.sh, after the Typhoon's: included
// at the end of eufi_cases.inc, so into qcost/harness.cpp's main() after the
// A320's. Each a fresh Synth in `room`, 250 blocks of 40ms (10s) by run(),
// set() each block as the 3D view sets a turboprop - its power and its
// propellers' speed as engines3d::ask() gives them for the phase, its
// propellers' axes 2.80m up, no spoilers - then case_end() and its checksum,
// as the A320's and the Typhoon's are. The busiest of each phase: placed, the
// ear where its scenes have it (preview.cpp's atr72-*), each case past its
// loudest moment.
{
    struct Turboprop {
        const char *name;
        engine_noise::Engine engine;
        float power, np;            // shaft power as a share of take-off's; propellers' speed asked, of 100%
        float rolling;              // m/s on the ground, 0 in the air
        float airspeed, x0, vx, z;  // m/s; where it starts along x and how fast it moves; its height
        bool gear, flaps, reverse, apu;
        int ear;  // 0 beside it: earY m off, earZ up; 1 the chase: 40m behind, 10m above
        float earY, earZ;
    };
    static const Turboprop turboprops[] = {
        // its take-off roll at 100%, 120m off the runway, abeam at 5s at 45m/s
        {"atr72-takeoff-roll", engine_noise::Engine::PW127_568F, 1.0f, 1.00f, 45, 45, -225, 45, 0, true, true, false,
         false, 0, 120, 4},
        // its approach at 100%, 300m up and 300m off, abeam at 5s
        {"atr72-approach-side", engine_noise::Engine::PW127_568F, 0.22f, 1.00f, 0, 60, -300, 60, 300, true, true,
         false, false, 0, 300, 1.5f},
        // its touchdown in reverse, 50m off the runway, abeam at 5s
        {"atr72-touchdown-reverse", engine_noise::Engine::PW127_568F, 0.70f, 1.00f, 50, 50, -250, 50, 0, true, true,
         true, false, 0, 50, 2},
        // taxiing past in beta, 35m off, abeam at 5s - six blades and four
        {"atr72-taxi-past", engine_noise::Engine::PW127_568F, 0.15f, 0.71f, 8, 8, -40, 8, 0, true, false, false, false,
         0, 35, 1.5f},
        {"atr72-200-taxi-past", engine_noise::Engine::PW124_14SF, 0.15f, 0.71f, 8, 8, -40, 8, 0, true, false, false,
         false, 0, 35, 1.5f},
        // at its stand in hotel mode, its left engine started, 35m off
        {"atr72-stand-hotel", engine_noise::Engine::PW127_568F, 0.05f, 0.71f, 0, 0, 0, 0, 0, true, false, false, true,
         0, 35, 2},
        // climbing at 82%, from the chase
        {"atr72-climb-chase", engine_noise::Engine::PW127_568F, 0.886f, 0.82f, 0, 75, 0, 75, 1000, false, false, false,
         false, 1, 0, 0},
        // the other turboprops' dearest (render_turboprops.sh): the four-engined
        // taxiing past in beta, each wing's two as one, and the A400M's roll
        // and approach, its four lobes and its handed pairs; a single taxiing
        // past, one propeller; the Twin Otter, not synchrophased, climbing -
        // its two propellers apart in the air too - and taxiing; the Q400
        {"c130j-taxi-past", engine_noise::Engine::AE2100D3_R391, 0.15f, 0.72f, 8, 8, -40, 8, 0, true, false, false,
         false, 0, 35, 1.5f},
        {"a400m-taxi-past", engine_noise::Engine::TP400_FH386, 0.15f, 0.762f, 8, 8, -40, 8, 0, true, false, false,
         false, 0, 35, 1.5f},
        {"a400m-takeoff-roll", engine_noise::Engine::TP400_FH386, 1.0f, 1.00f, 45, 45, -225, 45, 0, true, true, false,
         false, 0, 120, 4},
        {"a400m-approach-side", engine_noise::Engine::TP400_FH386, 0.25f, 0.979f, 0, 60, -300, 60, 300, true, true,
         false, false, 0, 300, 1.5f},
        {"pc12-taxi-past", engine_noise::Engine::PT6A_67P_PC12, 0.15f, 0.60f, 8, 8, -40, 8, 0, true, false, false,
         false, 0, 35, 1.5f},
        {"twinotter-climb-chase", engine_noise::Engine::PT6A27_HC3, 0.91f, 0.91f, 0, 50, 0, 50, 300, false, false,
         false, false, 1, 0, 0},
        {"twinotter-taxi-past", engine_noise::Engine::PT6A27_HC3, 0.15f, 0.55f, 8, 8, -40, 8, 0, true, false, false,
         false, 0, 35, 1.5f},
        {"q400-taxi-past", engine_noise::Engine::PW150_R408, 0.15f, 0.647f, 8, 8, -40, 8, 0, true, false, false,
         false, 0, 35, 1.5f},
    };
    auto make = +[](int i, const void *a) -> Sound {
        const Turboprop &c = *(const Turboprop *)a;
        const engine_noise::PropData &p = engine_noise::propData(c.engine);
        Sound s;
        s.kind = Kind::TURBOPROP;
        s.engine = c.engine;
        s.engines = p.engines;
        s.size = 27.17f / 37.6f;
        s.who = 0x406a3c;
        s.power = c.power;
        s.np = c.np;
        s.reverse = c.reverse;
        s.apu = c.apu;
        s.rollingMs = c.rolling;
        s.airspeedMs = c.airspeed;
        s.gearDown = c.gear;
        s.flapsOut = c.flaps;
        s.placed = true;
        s.at[0] = c.x0 + c.vx * i * 0.04f, s.at[2] = c.z + p.axisUp;
        if (c.ear == 1) {
            s.ear[0] = s.at[0] - 40, s.ear[1] = 0, s.ear[2] = s.at[2] + 10;
        } else {
            s.ear[1] = c.earY, s.ear[2] = c.earZ;
        }
        s.earUpM = s.ear[2];
        s.sourceUpM = s.at[2];
        s.hardGround = c.z < 91;
        s.ms = (uint32_t)i * 40;
        return s;
    };
    for (const Turboprop &c : turboprops) {
        engine_noise::Synth *synth = new (room) engine_noise::Synth();
        uint32_t sum = run(synth, blocks, make, &c);
        synth->~Synth();
        case_end();
        printf("case %s sound %08lx\n", c.name, (unsigned long)sum);
    }
}
