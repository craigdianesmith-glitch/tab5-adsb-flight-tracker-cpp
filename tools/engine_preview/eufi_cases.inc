// The Typhoon's cost cases for eufi-tools/cost.sh: included into
// qcost/harness.cpp's main() after the A320's, on the include path from
// tools/engine_preview/. Each a fresh Synth in `room`, 250 blocks of 40ms
// (10s) by run(), set() each block as the 3D view sets it, then case_end()
// and its checksum - as the A320's cases are. Placed, the ear as the 3D
// view's cameras have it; each case past its loudest moment, so whatever
// the fighter does only near and behind it - crackle - is in the count.
{
    struct Fighter {
        const char *name;
        float power, reheat;  // power 0..1; reheat asked for, 0 dry to 1 full
        float rolling;        // m/s on the ground, 0 in the air
        float airspeed, x0, vx, z;  // m/s; where it starts along x and how fast it moves; its height
        bool gear, reverse;
        float airbrake;
        int ear;  // 0 beside it: earY m off, earZ up; 1 the chase: 54m behind, 10m above
        float earY, earZ;
    };
    static const Fighter fighters[] = {
        // the 3D view's take-off: reheat on the roll, 150m off the runway,
        // passing abeam at 5s, 50m/s
        {"eufi-takeoff-roll", 1.0f, 1, 50, 50, -250, 50, 0, true, false, 0, 0, 150, 4},
        // a display pass in reheat at 154m/s, 30m up, past the tower 230m
        // off and 20m up, abeam at 5s
        {"eufi-display-reheat", 1.0f, 1, 0, 154, -770, 154, 30, false, false, 0, 0, 230, 20},
        // the fast low pass at power 0.85: 216m/s, 76m up, 50m off the track,
        // abeam at 5s
        {"eufi-low-pass-085", 0.85f, 0, 0, 216, -1080, 216, 76, false, false, 0, 0, 50, 1.5f},
        // the chase in cruise at FL250, M0.85
        {"eufi-chase-cruise", 0.65f, 0, 0, 263, 0, 263, 7620, false, false, 0, 1, 0, 0},
        // the chase on the final approach: gear down, approach power
        {"eufi-approach-chase", 0.45f, 0, 0, 70, -1500, 70, 100, true, false, 0, 1, 0, 0},
        // taxiing past at 8m/s, 35m off, abeam at 5s
        {"eufi-taxi-past", 0.14f, 0, 8, 8, -40, 8, 0, true, false, 0, 0, 35, 1.5f},
    };
    auto make = +[](int i, const void *a) -> Sound {
        const Fighter &c = *(const Fighter *)a;
        Sound s;
        s.kind = Kind::LOW_BYPASS;
        s.engine = engine_noise::Engine::EJ200;
        s.engines = 2;
        s.size = 0.425f;
        s.who = 0x43c6f1;
        s.power = c.power;
        s.reheat = c.reheat;
        s.afterburner = c.reheat > 0;
        s.reverse = c.reverse;
        s.rollingMs = c.rolling;
        s.airspeedMs = c.airspeed;
        s.gearDown = c.gear;
        s.spoilers = c.airbrake;
        s.placed = true;
        s.at[0] = c.x0 + c.vx * i * 0.04f, s.at[2] = c.z + 1.5f;
        if (c.ear == 1) {
            s.ear[0] = s.at[0] - 54, s.ear[1] = 0, s.ear[2] = s.at[2] + 10;
        } else {
            s.ear[1] = c.earY, s.ear[2] = c.earZ;
        }
        s.earUpM = s.ear[2];
        s.sourceUpM = s.at[2];
        s.hardGround = c.rolling > 0;
        s.ms = (uint32_t)i * 40;
        return s;
    };
    for (const Fighter &c : fighters) {
        engine_noise::Synth *synth = new (room) engine_noise::Synth();
        uint32_t sum = run(synth, blocks, make, &c);
        synth->~Synth();
        case_end();
        printf("case %s sound %08lx\n", c.name, (unsigned long)sum);
    }
}
