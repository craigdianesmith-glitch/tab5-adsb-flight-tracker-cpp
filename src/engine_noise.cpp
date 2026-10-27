// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

#include "engine_noise.h"

#include <math.h>
#include <string.h>
#ifdef ENGINE_NOISE_DEBUG
#include <stdio.h>
#endif

#include <algorithm>

namespace engine_noise {

bool g_transientsOnPicture = false;
#ifdef ENGINE_NOISE_DEBUG
uint64_t g_limited = 0, g_samples = 0;  // samples the limiter rounded off, of all
#endif

namespace {

constexpr float PI = 3.14159265f;
// The controls - loudness, pitch, the resonances' turns - worked out again
// every this many samples, 2ms: often enough for none of them to be heard
// stepping.
constexpr int STEP = 32;
constexpr float STEP_S = STEP / (float)RATE;
// The spectrum's sums - the bands', the tones', the buzz-saws' - each
// sixteenth step: 32ms, under half the quickest surge's time and still
// smooth, each band's loudness carried across it a little each sample, each
// buzz-saw's delay each step.
constexpr int SLOW = 16;
constexpr float SLOW_S = SLOW * STEP_S;
constexpr float SOUND_MS = 343;  // the speed of sound, at 20C
// What is made now is heard this much later on the Tab5 - a 40ms block
// queued behind the one playing, and the speaker's own 43ms of buffers - so
// what is heard is looked for that much sooner.
#ifdef ESP_PLATFORM
constexpr float HEARD_AHEAD_S = 0.08f;
#else
constexpr float HEARD_AHEAD_S = 0;
#endif
// The highest a tone is played at, faded out over the last 1,200Hz to it -
// under 8kHz, half the samples' rate, past which a tone comes back down as
// another.
constexpr float TOP_HZ = 7600, TOP_FADE_HZ = 1200;
// The mix's level, as samples go: an airliner taking off past a camera
// beside the runway some -20dBFS at its loudest, as loud in the speaker's
// band as the old synthesiser's, which was set by ear on a Tab5 - and its
// approach overhead 4dB under a climb-out over the same microphone (the
// noise data has 4.5-6.5), its chase at cruise -20dBFS; under the words,
// which are near full scale.
constexpr float LEVEL = 0.59f;

float clamp01(float x) { return x < 0 ? 0 : x > 1 ? 1 : x; }
float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
float smoothstep(float a, float b, float x) {
    x = clamp01((x - a) / (b - a));
    return x * x * (3 - 2 * x);
}
float ease(float from, float to, float part) { return from + (to - from) * part; }

// Quick forms of the few functions the controls need, once a step - no
// library call in them, and none at all for each sample.
//  2 to the x, to a few parts in a hundred thousand.
float exp2Fast(float x) {
    if (x < -125) {
        return 0;
    }
    x = std::min(x, 125.0f);
    int i = (int)x;
    if ((float)i > x) {
        i--;
    }
    float f = x - (float)i;
    float p = 1.0f + f * (0.693147180f + f * (0.241367f + f * (0.0520025f + f * 0.0135557f)));
    union {
        int32_t i;
        float f;
    } u;
    u.i = (i + 127) << 23;
    return p * u.f;
}
//  log 2 of x, more than nought, to a few parts in a hundred thousand.
float log2Fast(float x) {
    union {
        float f;
        int32_t i;
    } u;
    u.f = x;
    int e = ((u.i >> 23) & 255) - 127;
    u.i = (u.i & 0x007FFFFF) | 0x3F800000;
    float m = u.f, t = (m - 1) / (m + 1), t2 = t * t;
    return e + t * (2.88539008f + t2 * (0.961796694f + t2 * (0.577078016f + t2 * 0.412198583f)));
}
float dbAmp(float db) { return exp2Fast(db * 0.166096405f); }    // 10^(db/20)
float dbPower(float db) { return exp2Fast(db * 0.332192809f); }  // 10^(db/10)
float expFast(float x) { return exp2Fast(x * 1.44269504f); }
float lg10(float x) { return 0.301029996f * log2Fast(x); }  // log 10, x more than nought
float powFast(float x, float y) { return x > 0 ? exp2Fast(y * log2Fast(x)) : 0.0f; }
//  The sine of x turns, x between -0.5 and 0.5, to a few parts in a million.
float sinTurn(float x) {
    if (x > 0.25f) {
        x = 0.5f - x;
    } else if (x < -0.25f) {
        x = -0.5f - x;
    }
    float a = x * 6.28318531f, a2 = a * a;
    return a * (1 - a2 * (0.166666667f - a2 * (0.00833333333f - a2 * (0.000198412698f - a2 * 2.75573192e-6f))));
}
void sinCosTurns(float x, float &s, float &c) {
    x -= (float)(int)x;
    if (x > 0.5f) {
        x -= 1;
    } else if (x < -0.5f) {
        x += 1;
    }
    s = sinTurn(x);
    float y = x + 0.25f;
    c = sinTurn(y > 0.5f ? y - 1 : y);
}

// What the air takes of a sound going a kilometre through it, dB, at 63Hz
// to 8kHz an octave apart - ISO 9613-1's, at 20C and 70% humidity - kept as
// their logs, for between the octaves as the logs go: next to nothing of a
// rumble, three quarters of the eight kilohertz' level in a hundred metres.
constexpr float ABSORPTION_LOG2[8] = {-3.32193f, -1.73697f, 0.137504f, 1.48543f,
                                      2.32193f,  3.16993f,  4.51677f,  6.25929f};
float absorptionDbKm(float hz) {
    float o = clampf(log2Fast(std::max(hz, 1.0f) * (1.0f / 63)), 0, 6.999f);
    int i = (int)o;
    float f = o - i;
    return exp2Fast(ABSORPTION_LOG2[i] + (ABSORPTION_LOG2[i + 1] - ABSORPTION_LOG2[i]) * f);
}
// The frequency the air has taken 6dB of over `km`: the broadband's muffling
// tuned there - the tones have theirs worked out exactly - between the
// octaves as the logs of both go; not under 400Hz.
float mufflingHz(float km) {
    float most = log2Fast(6 / std::max(km, 1e-4f));  // the log of the dB a kilometre the 6dB is
    for (int i = 1; i < 8; i++) {
        if (ABSORPTION_LOG2[i] >= most) {
            float octave = (i - 1) + (most - ABSORPTION_LOG2[i - 1]) / (ABSORPTION_LOG2[i] - ABSORPTION_LOG2[i - 1]);
            return clampf(63 * exp2Fast(octave), 400, 7000);
        }
    }
    return 7000;
}

// A table of dB by angle off the nose, every 20 degrees from 0 to 180, read
// between its points: never a hard edge, which a pass would hear as a step.
using Lobe = float[10];
float lobe(const Lobe &t, float aspect) {
    float x = clamp01(aspect) * 9;
    int i = std::min(8, (int)x);
    return t[i] + (t[i + 1] - t[i]) * (x - i);
}
//                           0     20    40    60    80   100   120   140   160   180
// The fan's tones: from its intake ahead, most at 30 to 50 degrees off the
// nose, and from its bypass duct behind, most at 120 to 130 and weaker -
// Heidmann's shapes. Its broadband hiss in two parts, as they surge apart:
// out of its intake, broad ahead; and out of its bypass duct, most at 120
// degrees and stronger, through the jet's shear layer and wake.
constexpr Lobe FAN_TONE = {-4.0f, -1.0f, 0.0f, -2.5f, -6.0f, -8.5f, -8.0f, -9.0f, -12.0f, -18.0f};
constexpr Lobe FAN_IN = {-4.0f, -2.0f, -2.0f, -4.5f, -9.0f, -15.0f, -21.0f, -27.0f, -33.0f, -39.0f};
constexpr Lobe FAN_AFT = {-38.0f, -32.0f, -24.0f, -16.0f, -8.0f, -2.7f, 0.0f, -1.0f, -4.0f, -10.0f};
// The buzz-saw: out of the intake only, most at 40 to 60 degrees and 8dB
// down by 90 - NASA's combination tones' shape - and gone behind, as every
// recording of one has it.
constexpr Lobe BUZZ = {-3.0f, -0.5f, 0.0f, -1.5f, -8.0f, -11.0f, -17.0f, -23.0f, -27.0f, -29.0f};
// The jet's roar: most 50 to 70 degrees off its tail; falling away behind
// it, into the cone of silence along the jet; broad to the side, where its
// fine turbulence is heard; and ahead, some 15dB under, carrying the low
// end of a take-off coming at the camera [M: the recordings' 300-600Hz the
// loudest and the most surging; Zellmann's A320 directivity, its 250Hz
// most at 130 degrees, its 500Hz at 100-110 and 6-8dB down by 40 and by
// 150-170]. (Most at 140, it stood 4-5dB too loud behind against 90, and a
// climb-out going away stood 7dB nearer its pass than the recordings'.)
constexpr Lobe JET = {-17.0f, -15.0f, -13.0f, -9.5f, -6.0f, -4.5f, -2.5f, -4.0f, -7.0f, -10.0f};
// The core's rumble, behind a little; the airframe's rush, all round; the
// flaps' edges, more behind; the reversers' efflux, thrown forward; the
// APU's exhaust, out of the tail cone, most 60 degrees off its axis; a
// V2500's turbine, behind.
constexpr Lobe CORE = {-6.0f, -5.5f, -5.0f, -4.0f, -2.5f, -1.0f, 0.0f, 0.0f, -1.5f, -4.0f};
constexpr Lobe AIRFRAME = {-3.0f, -2.0f, -1.0f, 0.0f, 0.0f, 0.0f, -0.5f, -1.0f, -1.5f, -2.0f};
constexpr Lobe FLAPS = {-4.0f, -3.5f, -2.5f, -1.5f, -1.0f, -0.5f, 0.0f, 0.0f, -1.0f, -2.0f};
constexpr Lobe REVERSE = {-2.0f, 0.0f, 0.0f, 0.0f, -0.5f, -2.0f, -4.0f, -6.0f, -8.0f, -9.0f};
constexpr Lobe APU = {-12.0f, -11.0f, -9.0f, -7.0f, -4.0f, -2.0f, 0.0f, -1.0f, -2.5f, -5.0f};
constexpr Lobe TURBINE = {-30.0f, -28.0f, -25.0f, -20.0f, -12.0f, -5.0f, 0.0f, -2.0f, -6.0f, -12.0f};

// An engine's figures [P: type certificates, measured N1 by phase - see
// the A320 research]: its fan's blades and shaft's turning at 100% N1; its
// fan speed idling on the ground and in the air, on the approach, level at
// height, climbing, for take-off and in full reverse; the speed its buzz-saw
// starts at, and how strong it is against a CFM56-5B's; its fan stream's
// speed at take-off; its noise and its whine against a CFM56-5B's; its
// turbine's last two stages' blades, if heard; how long its start motors
// before it lights. And its size against a CFM56-5B's: its jet's width -
// the roar as many times lower and its surges as many times slower, its
// eddies as much bigger [b777-sources 9.1: D/V 6.7-8.9ms against 3.5ms];
// its fan's second harmonic over the A320's; the shaft orders its buzz-saw
// is heard at, from and to; how quickly its fan spools, as a share of a
// CFM56-5B's; and the airframe it hangs on - its gear's and flaps' rush
// against an A320's.
struct EngineData {
    float blades, shaft100;
    float groundIdle, flightIdle, approach, cruise, climb, takeoff, reverse;
    float buzzOn, buzzDb, jetMs, levelDb, toneDb;
    float turbine[2];
    float motorS;
    float jetScale, h2Db, buzzLo, buzzHi, spool, frameDb;
};
// One row for each Engine, in its order - restart(), engines() and update()
// read a turboprop's too.
const EngineData ENGINES[] = {
    // GENERIC: its shaft from Sound::fanHz, 24 blades assumed.
    {24, 125.0f, 0.22f, 0.27f, 0.51f, 0.82f, 0.85f, 0.90f, 0.70f, 0.67f, 0, 300, 0, 0, {0, 0}, 15, 1, 0, 4, 18, 1, 0},
    // CFM56-5B: idle 20-22%; final 49-55% (Zurich); take-off 85-94%; reverse 68%.
    {36, 83.3333f, 0.22f, 0.27f, 0.51f, 0.82f, 0.84f, 0.90f, 0.70f, 0.67f, 0, 300, 0, 0, {0, 0}, 15, 1, 0, 4, 18, 1, 0},
    // V2500-A5: idling at 30% on the ground, its fan's line at 622Hz [M: DLR,
    // D-ATRA], 29% in the air; the BPF of a 1,768-2,067Hz take-off measured;
    // its buzz-saw the strongest of the four in the recorded climb-outs [M:
    // 2.6-6.7dB clear]; LPT tones at engine orders 85 and 89, behind it.
    {22, 94.1667f, 0.30f, 0.29f, 0.50f, 0.82f, 0.82f, 0.90f, 0.68f, 0.64f, 2.0f, 310, -1, 2, {85, 89}, 15, 1, 0, 4, 18, 1, 0},
    // LEAP-1A: TOGA ~83%, FLEX ~75%; level at height at its take-off's 91%,
    // as a CFM56-5B's 82 is of its 90 [I: no neo's is published] - at 80%,
    // all but its take-off's, the chase at cruise stood 1.6dB over the
    // CFM56's; its tips barely past the speed of sound at take-off - its
    // buzz-saw from 73%, weak; its jet slower; 3.5dB quieter departing and
    // as loud arriving [M: Luton's monitors]; motoring longer before
    // light-off.
    {18, 64.2667f, 0.21f, 0.24f, 0.47f, 0.74f, 0.77f, 0.81f, 0.62f, 0.73f, 0, 270, -3, -2, {0, 0}, 24, 1, 0, 4, 18, 1, 0},
    // PW1100G: the fan's speed as a share of its 3,281rpm most; its turbine's
    // tones over 8kHz, the air's.
    {20, 54.6833f, 0.19f, 0.25f, 0.55f, 0.82f, 0.83f, 0.88f, 0.65f, 0.84f, -12, 250, -3, -1, {0, 0}, 30, 1, 0, 4, 18, 1, 0},
    // EJ200 - the fighter's, for what of update() it shares: the start's
    // motoring and the fan's speed from power (engines()); the airliner's
    // sums don't run for it (see THE FIGHTER). Estimates, none published
    // [eufi-sources 1.2, 2.2]: 24 blades, 12,700rpm at 100%; idling at 30%
    // on the ground, 35% in the air, 75% on the approach, 85% level, 100%
    // dry for the climb and take-off; no reversers - their figure its
    // idle, so the reversers asked for leave it idling whatever the power;
    // no buzz-saw heard (its S-shaped chin intake); its jet 770m/s dry; its
    // air starters motoring it 25s to a third of idle - some 23% of its HP
    // spool's speed - and lit there.
    {24, 211.667f, 0.30f, 0.35f, 0.75f, 0.85f, 1.00f, 1.00f, 0.30f, 2.0f, -60, 770, 0, 0, {0, 0}, 25, 1, 0, 4, 18, 1, 0},
    // The 777's [b777-sources 1, 2.2, 3.1-3.3, 8, 9.1]. GE90-94B: idling at
    // 20-27%, its final 55-70%, its climb 84-95%, its take-off 90-104%; its
    // tips supersonic, its buzz-saw on, from 78-80%; its jet slower, its
    // bypass 8.7; 2-10dB louder than an A320 by phase, its jet an octave
    // lower and churning half as fast, its eddies 2-2.5 times a CFM56's; its
    // second harmonic strong and clean at every power, the fan's voice on a
    // little speaker where an idle's 170-230Hz fundamental isn't; its
    // buzz-saw a few clear orders, 7 to 12 (10 the strongest on the one
    // measured), 1-5 cut off in its intake; its big fan spooling slowly,
    // gliding low through a start; its six-stage turbine's tones behind it at
    // shaft orders of about a hundred - 2-4kHz on the approach, where the
    // recorded ones' treble is - its blades' count not published, so 100
    // and 104 [I]; its airframe's rush 8dB up - twelve main
    // wheels to four, three and a half times the wing - its approach the
    // airframe's more than its engines', as the recorded ones' treble has it.
    {22, 37.6917f, 0.23f, 0.28f, 0.62f, 0.86f, 0.89f, 0.97f, 0.72f, 0.79f, 0, 280, 4, 0, {100, 104}, 35, 2.0f, 5, 7, 13, 0.75f, 8},
    // GE90-115B: the same at 2,355rpm, its buzz-saw from 72-74%, its bypass
    // 7.2 - a faster, louder jet.
    {22, 39.25f, 0.23f, 0.28f, 0.62f, 0.86f, 0.89f, 0.97f, 0.72f, 0.73f, 0, 300, 6, 0, {100, 104}, 35, 2.1f, 5, 7, 13, 0.75f, 8},
    // Trent 800: three shafts, 26 blades at 3,300rpm, 1,430Hz at 100%; its
    // tips supersonic already on the approach at 66-81% - its buzz-saw from
    // 61%, the strongest of the three, more orders - its approach's fan tone
    // weak; its take-off 88-100%, its reverse 70-80%; a smaller fan than a
    // GE90's - its jet's eddies a shade smaller, its harmonic less marked.
    {26, 55.0f, 0.23f, 0.28f, 0.73f, 0.86f, 0.90f, 0.95f, 0.75f, 0.61f, 2, 310, 6, -2, {0, 0}, 35, 1.9f, 2, 5, 16, 0.75f, 8},
    // The ATR's PW100s, Kind::TURBOPROP, for what of update() a turboprop
    // shares - its start's motoring and spool, and N1 from power by
    // engines(), which for it is its gas generator's high-pressure spool's
    // speed (NH) as a share of its 100%: some 66% idling on the ground, 72%
    // in the air, 85% on the approach, 95% cruising, 97% climbing, 100% for
    // take-off, 92% in reverse [I: none published]. Its spool's turning at
    // 100% as `shaft100` - 34,360rpm on a PW127, 34,200 on a PW124B,
    // 34,350 on a PW120 [P: EASA TCDS IM.E.041] - `blades` a placeholder; its
    // start motoring 20s to a third of idle [I]; its spool 0.8 of a CFM56's
    // [I]. No fan, no buzz-saw, no jet - the airliner's sums never run for
    // it: its sound is its own, from PROPS.
    {24, 572.667f, 0.66f, 0.72f, 0.85f, 0.95f, 0.97f, 1.00f, 0.92f, 2.0f, -60, 0, 0, 0, {0, 0}, 20, 1, 0, 4, 18, 0.8f, 0},
    {24, 570.0f, 0.66f, 0.72f, 0.85f, 0.95f, 0.97f, 1.00f, 0.92f, 2.0f, -60, 0, 0, 0, {0, 0}, 20, 1, 0, 4, 18, 0.8f, 0},
    {24, 572.667f, 0.66f, 0.72f, 0.85f, 0.95f, 0.97f, 1.00f, 0.92f, 2.0f, -60, 0, 0, 0, {0, 0}, 20, 1, 0, 4, 18, 0.8f, 0},
    {24, 572.667f, 0.66f, 0.72f, 0.85f, 0.95f, 0.97f, 1.00f, 0.92f, 2.0f, -60, 0, 0, 0, {0, 0}, 20, 1, 0, 4, 18, 0.8f, 0},
    {24, 572.5f, 0.66f, 0.72f, 0.85f, 0.95f, 0.97f, 1.00f, 0.92f, 2.0f, -60, 0, 0, 0, {0, 0}, 20, 1, 0, 4, 18, 0.8f, 0},
};
static_assert(sizeof ENGINES / sizeof ENGINES[0] == (size_t)Engine::PW120_14SF + 1, "a row in ENGINES for each Engine");

// The propeller installations' figures - see PropData in engine_noise.h -
// each with where it comes from: [P] published, [P*] in a secondary source,
// [M] measured on the CC0 and CC BY recordings of the ATR research (in
// ~/.cache/terrain-lab/sound/: research/atr72-sources.md,
// research/turboprop-methods.md, targets/atr72.md), [I] inferred, [O] this
// project's own. Shared by the five ATR rows:
//  - Their propeller's speed at 100% 1,200rpm, the PW100's output shaft
//    allowed 1,212 [P: EASA TCDS IM.E.041,
//    https://www.easa.europa.eu/en/downloads/7725/en]; the governor's 100%
//    for take-off and the approach and 82% climbing and cruising, ground
//    idle in beta 71%, nothing governing it in beta or reverse - the
//    overspeed governor at 118% on the ground [P: BEA, 9Y-TTC,
//    https://bea.aero/fileadmin/user_upload/9y-c140505.en_22_03_22.pdf].
//    Measured aboard an ATR 72-500/-600 at Kajaani [M: fs680325, CC0,
//    targets 4.1]: 1,199.7rpm, 984.0 and 824-851; ground idle to take-off
//    in 2.5-3.5s; take-off to climb a straight glide over 8.5s, 2.1-2.3% a
//    second. On an approach to Schiphol 1,211-1,214rpm [M: fs315660, CC0].
//  - Their two propellers synchrophased governed, the left the master -
//    "± 1 RPM speed and ± 1° phase holding" [P: BEA; ATR at EWADE 2011,
//    https://www.fzt.haw-hamburg.de/pers/Scholz/ewade/2011/Presentations/Workshop_Opening_Day_Conference/07_ATR_2_ORSI_EWADE_2011.pdf]:
//    one line per harmonic in every governed stretch recorded [M]. In beta
//    a taxiing Dash 8's two ran 4-5% apart [M: fs637148, CC0], an ATR's at
//    ground idle some 0.25% [M, aboard]: half a percent between [I].
//  - Their axes 8.10m apart and their tips 0.835m over the ground - so
//    2.80m up - on the ATR 72-600's and 42-600's drawings, 0.82m under the
//    -200's bigger 14SF-11 [P: ATR's fact sheets,
//    https://www.atr-aircraft.com/wp-content/uploads/2020/07/Factsheets_-_ATR_72-600.pdf
//    and Factsheets_-_ATR_42-600.pdf beside it].
//  - Their power by phase from each type's ratings, take-off's against
//    its maximum climb's and cruise's [P: the fact sheets]; level low down
//    some half of an ATR 72-600's take-off power - level at 180 knots, its
//    drag as the square of its equivalent airspeed and its power as its
//    drag times its true airspeed, from its cruise's [I]; coming down at
//    flight idle [P: the BEA; I: its share]; on the approach some 0.22 -
//    the noise data's 1,050-1,290lb an engine at 117 knots against
//    5,150-5,350lb near lift-off [P: EASA ANP v2.3,
//    https://www.easa.europa.eu/en/downloads/138164/en; I: thrust as
//    power at the same speed]; in reverse 0.7 - the 3D view's airliner's
//    [O], the noise data's ATR 72 landing roll starting at 75.9, against
//    an A320's 40 [P: ANP; I: shares of their static thrust].
//  - Their gas generator's whine at 5,682Hz at the source on an ATR 72's
//    approach, its interquartile 5,652-5,765 - 13dB clear, and far out two
//    lines 2.4% apart, each engine's own [M: fs315660, fs166417 (CC BY 4.0),
//    targets 5.1]; its impellers' blades not published, so no order of its
//    spool's turning.
//  - Their reduction gearbox's lines: the first as heard outside, 47.7
//    times the propeller's turning - 7.94 times the blades' passing on
//    an ATR 72's approach, ± 0.6, 6-16dB over its neighbours 450m off and
//    12-25 close [M: fs315660, fs166417, targets 5.2] - aboard at Kajaani
//    48.8, "probably the same line" [M: turboprop-methods 1.6, locked to
//    the propeller within 0.2% at three speeds]; the second at 72.9,
//    heard aboard only - not outside, 450m off [M: targets 5.2].
//  - Hotel mode on their right engine - "the propeller is held stationary
//    by a hydraulically operated brake, allowing the right engine to
//    operate as an auxiliary power unit" - and no APU in the type design
//    [P: AAIB, G-CMFI, Bulletin 1/2024,
//    https://assets.publishing.service.gov.uk/media/659d27a748baa2000ebbe381/ATR_72-212_A_G-CMFI_01-24.pdf;
//    EASA TCDS A.084: APU "Not Applicable"]. No ground spoilers: its two
//    spoilers worked by the control wheels, with the ailerons [P*:
//    SmartCockpit, https://www.smartcockpit.com/wp_quiz/atr-72-flight-controls/;
//    I: for roll alone].
const PropData PROPS[] = {
    // The ATR 72-500/-600 (AT75, AT76): PW127F/M/Ns, six-bladed 568Fs
    // 3.93m across [P: the fact sheet; EASA TCDS A.084]; 2,475shp each for
    // take-off, 2,192 its maximum climb, 2,132 its maximum cruise [P: the
    // fact sheet]. Certified 82.5 / 80.5 / 92.2EPNdB lateral, flyover and
    // approach, the A320's 93.6 / 86.4 / 95.3 [P: EASA's noise data sheets,
    // atr72-sources 8.1]; 5-8dB under an A320 taking off and 4-6 on the
    // approach, A-weighted [P: ANP].
    {Engine::PW127_568F, "ATR 72-500/-600", 2, 6, 1200, 3.93f, 4.05f, 2.80f,
     0.71f, 0.82f, 1.00f, 1.00f, 1.00f, 0.12f, 0.022f, true, 0.005f,
     0.886f, 0.861f, 0.50f, 0.05f, 0.22f, 0.70f,
     34360, 5682, 47.7f, 72.9f, true, false},
    // The ATR 72-200 (AT72): PW124Bs, four-bladed 14SF-11s 3.96m across, the
    // tips 0.82m up; 2,160shp each for take-off, 2,088 its maximum climb and
    // cruise [P: the ATR 72-200's fact sheet, refs/atr72/tcds]. Certified
    // 86.2 / 87.5 / 94.2EPNdB - 3.7, 7.0 and 2.0 over the -600 [P]. Its
    // speeds by phase, its whine and its gearbox's lines as the -500's [I:
    // none measured; the PW100s' reduction ratios run 15.4-17.16 (P*:
    // Wikipedia's PW100), so its lines' orders may be others]; level and
    // approach power as the -600's shaft power in its smaller take-off's
    // shares [I].
    {Engine::PW124_14SF, "ATR 72-200", 2, 4, 1200, 3.96f, 4.05f, 2.80f,
     0.71f, 0.82f, 1.00f, 1.00f, 1.00f, 0.12f, 0.022f, true, 0.005f,
     0.967f, 0.967f, 0.58f, 0.05f, 0.25f, 0.70f,
     34200, 5682, 47.7f, 72.9f, true, false},
    // The ATR 72-210 (AT73): PW127s - 2,475shp for take-off [P: EASA TCDS
    // IM.E.041, 1,846kW] - turning four-bladed 247Fs [P: EASA TCDS A.084],
    // their diameter not found, 3.93m [I]. Certified 84.7 / 80.9 /
    // 92.5EPNdB [P]. The rest as the -500's [I].
    {Engine::PW127_247F, "ATR 72-210", 2, 4, 1200, 3.93f, 4.05f, 2.80f,
     0.71f, 0.82f, 1.00f, 1.00f, 1.00f, 0.12f, 0.022f, true, 0.005f,
     0.886f, 0.861f, 0.50f, 0.05f, 0.22f, 0.70f,
     34360, 5682, 47.7f, 72.9f, true, false},
    // The ATR 42-400/-500/-600 (AT44, AT45, AT46): the 72-500's six-bladed
    // 568Fs on PW121As or PW127E/Ms [P: TCDS A.084], its axes as far apart
    // and as high [P: the ATR 42-600's fact sheet]; 2,160shp each for
    // take-off and climbing alike, 2,132 cruising [P: the fact sheet] - so
    // its climb at full power, its propellers at 82% none the less (np, not
    // power, says which). Certified 80.7 / 76.6 / 92.4EPNdB, the -500 [P].
    // Its speeds by phase as the 72's [I]; lighter - 18.6t at most against
    // 22.8t - and its wing smaller, 54.5m2 against 61 [P: the fact sheets],
    // so level and on the approach a shade under the 72's share [I].
    {Engine::PW127_568F_42, "ATR 42-500/-600", 2, 6, 1200, 3.93f, 4.05f, 2.80f,
     0.71f, 0.82f, 1.00f, 1.00f, 1.00f, 0.12f, 0.022f, true, 0.005f,
     1.00f, 0.987f, 0.48f, 0.05f, 0.20f, 0.70f,
     34360, 5682, 47.7f, 72.9f, true, false},
    // The ATR 42-300/-320 (AT43): PW120s or PW121s - 1,342 and 1,454kW
    // (1,800 and 1,950shp) for take-off, the PW120's maximum continuous
    // 1,268kW [P: TCDS IM.E.041] - turning four-bladed 14SF-5s [P: TCDS
    // A.084; M: four blades in the FedEx videos' frames], 3.96m across as
    // the 14SF-11 [I]. Certified 84.5 / 77.1 / 96.8EPNdB [P]. On its approach
    // seven FedEx ATR 42s' blades passed at 67-79Hz at the source, median
    // 73Hz - 91% of 1,200rpm, not 100% [M: targets 4.2, camera sound, the
    // fits unreliable]. Its climb and cruise ratings not found: 0.95 and
    // 0.90 [I]; the rest as the 72's [I].
    {Engine::PW120_14SF, "ATR 42-300/-320", 2, 4, 1200, 3.96f, 4.05f, 2.80f,
     0.71f, 0.82f, 1.00f, 0.91f, 1.00f, 0.12f, 0.022f, true, 0.005f,
     0.95f, 0.90f, 0.55f, 0.05f, 0.24f, 0.70f,
     34350, 5682, 47.7f, 72.9f, true, false},
};
// A turboprop with no row of its own (Engine::GENERIC): a twin like an ATR
// 72-600, its blades passing at Sound::bladeHz at 100% - 120Hz unless set,
// six blades at 1,200rpm - no whine nor gearbox lines known, no hotel mode.
const PropData PROP_GENERIC = {Engine::GENERIC, "a turboprop", 2, 6, 1200, 3.93f, 4.05f, 2.80f,
                               0.71f, 0.82f, 1.00f, 1.00f, 1.00f, 0.12f, 0.022f, true, 0.005f,
                               0.886f, 0.861f, 0.50f, 0.05f, 0.22f, 0.70f,
                               34360, 0, 0, 0, false, false};

// A hash's bits mixed, for an airframe's little differences from `who`.
uint32_t mix(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}
float unit(uint32_t h) { return (h >> 8) * (1.0f / 16777216.0f); }  // 0 to 1
// A float's bits, for a hash of where something is.
uint32_t bitsOf(float f) {
    union {
        float f;
        uint32_t u;
    } v;
    v.f = f;
    return v.u;
}

// The bank's bands: their centres, 250Hz to 6.3kHz two thirds of an octave
// apart, and how sharp each stage is - with the resonance after it, enough
// apart to be told apart, enough together to leave no gaps between them.
constexpr float BAND_HZ[Synth::BANDS] = {250, 400, 630, 1000, 1600, 2500, 4000, 6300};
// Each a band-pass and then a resonance on the same poles, its skirts
// falling 18dB an octave, not 6: the spectrum falls 25dB from the loud low
// bands to the quiet treble, and a band-pass alone's skirts filled the
// treble with the low bands' roar and their surges.
constexpr float BAND_Q = 0.8f;

// The broadband sources, each with its own surges - see bandsStep(). The
// bands' loudnesses move by them - a few slow processes in all, each band
// moving with the sources that fill it, as much as they fill it - and by
// the air's flicker, which moves every band above its corner together, and
// a little of each band's own: so the bands move partly together and partly
// apart, as much as the recordings' do [M: the 500Hz, 1, 2 and 4kHz octaves'
// levels correlated 0.07 on a take-off roll still coming, 0.24-0.41 as it
// passes and goes, the treble's three the most].
enum Source { OF_JET, OF_FAN, OF_FAN_AFT, OF_CORE, OF_AIRFRAME, OF_REVERSER, OF_BUFFET, OF_APU, OF_BUZZ };
// How far each surges, in nepers of its loudness, and how quickly - its
// quick part's and its slow part's time constants, s, and how much of it is
// the quick part. Slowly, most of it: over a second, a little over a third
// of one - the recordings' octaves wander most at 0.25-2Hz, their 1-4Hz a
// quarter or less of it as an aircraft passes [M] - each source alone, so
// a band's surges are its sources' mix. The jet, the fan's hiss out of its
// intake and out of its bypass duct, the airframe's rush, by a fifth to a
// quarter of their loudness; the core's barely; the reversers' efflux and
// the spoilers' buffet the most and the quickest; the APU's steady; and the
// buzz-saws', all their lines together, their own (with the intake's, a
// take-off still coming had its octaves moving together, which the
// recordings' don't: r 0.31 against 0.05-0.16).
constexpr float SURGE_NP[Synth::SOURCES] = {0.18f, 0.26f, 0.21f, 0.08f, 0.24f, 0.22f, 0.25f, 0.05f, 0.20f};
constexpr float SURGE_QUICK_S[Synth::SOURCES] = {0.40f, 0.40f, 0.40f, 0.23f, 0.40f, 0.12f, 0.07f, 0.31f, 0.30f};
constexpr float SURGE_SLOW_S[Synth::SOURCES] = {1.15f, 1.15f, 1.15f, 1.30f, 1.15f, 0.50f, 0.40f, 1.70f, 1.15f};
constexpr float SURGE_QUICK_W[Synth::SOURCES] = {0.4f, 0.4f, 0.4f, 0.8f, 0.4f, 0.8f, 0.8f, 0.8f, 0.4f};
// Each band's own shimmer: a decibel and a half; more where the turbulent
// sources fill it, the further the sound has come through the air's eddies,
// a little more as the aircraft crosses the way the sound comes fast, and
// close behind it - each band wandering by itself at about a hertz, quicker
// and slower, as the recordings' bands each churn by a decibel or more of
// their own [M]. (More of it as it passes, 0.1Np, made the passes flutter:
// 1-4Hz half the level's wander, against the recordings' quarter.)
constexpr float SHIMMER_NP = 0.18f, SHIMMER_TURBULENT_NP = 0.04f, SHIMMER_FAR_NP = 0.07f, SHIMMER_PASS_NP = 0.06f,
                SHIMMER_S = 0.15f, SHIMMER_SLOW_S = 0.23f, SHIMMER_QW = 0.8f, SHIMMER_WAKE_NP = 0.13f;

// The air's flicker as quick as the eddies cross the way the sound comes:
// its rate in hertz (this much of the aircraft's speed across it, plus a
// breeze's few m/s) over 5m - a pass's flicker quicker, but not so quick as
// to flutter - and its spread, dB at a kilometre: a decibel from 650m [M:
// Empa's calm air, 0.8-1.5dB].
constexpr float FLICKER_ACROSS = 0.15f, FLICKER_BASE = 3.7f, FLICKER_DB = 1.25f;

// The autothrust's hunting, as a noise-driven resonance stepped each slow
// step: its poles' radius and turn - 0.42Hz, damped by 0.35 - and the push
// that gives it a unit spread (an AR(2)'s variance worked out).
constexpr float HUNT_W = 2 * PI * 0.42f * SLOW_S, HUNT_Z = 0.35f;
const float HUNT_R = expf(-HUNT_Z * HUNT_W), HUNT_A1 = 2 * HUNT_R * cosf(HUNT_W * sqrtf(1 - HUNT_Z * HUNT_Z)),
            HUNT_A2 = -HUNT_R * HUNT_R,
            HUNT_B = sqrtf((1 + HUNT_A2) * ((1 - HUNT_A2) * (1 - HUNT_A2) - HUNT_A1 * HUNT_A1) / (1 - HUNT_A2));

// The tones' places.
enum ToneSlot {
    FAN_1 = 0,      // each engine's blade passing, 0-1
    FAN_H2 = 2,     // the two engines' second harmonics as one, and their third, 2-3
    TURBINE_1 = 4,  // a V2500's turbine tones, 4-5
    NOSE_GEAR = 6,
    VENT_1 = 7,     // the fuel vents' two, 7-8
    APU_1 = 9,      // the APU's two, 9-10
    SQUEAL_1 = 11,  // the main gears' squeal, 11-12
    CORE_1 = 13,    // each engine's core compressor's whine, 13-14
};

}  // namespace

// Tuned again only as its cutoff moves a fifth of a percent or more.
void Synth::Svf::tune(float toHz, float toQ) {
    toHz = std::min(std::max(toHz, 20.0f), RATE * 0.45f);
    if (toQ == q && fabsf(toHz - hz) < hz * 0.002f) {
        return;
    }
    hz = toHz;
    q = toQ;
    float s, c;
    sinCosTurns(0.5f * hz / RATE, s, c);
    float g = s / c;
    k = 1 / q;
    a1 = 1 / (1 + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
}

// The low-pass and the band-pass - the band-pass at the cutoff as loud as
// what goes in.
static inline __attribute__((always_inline)) void svfRun(float &ic1, float &ic2, float a1, float a2, float a3,
                                                         float k, float x, float &low, float &band) {
    float v3 = x - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    float v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = 2 * v1 - ic1;
    ic2 = 2 * v2 - ic2;
    low = v2;
    band = v1 * k;
}

Synth::Synth() {
    // The bank's band-passes, as loud at their centres as what goes in, then
    // scaled for a noise of unit power to come out of each at unit power:
    // the bands' loudnesses are then their sources' powers' sums.
    for (int b = 0; b < BANDS; b++) {
        float s, c;
        sinCosTurns(BAND_HZ[b] / RATE, s, c);
        float alpha = s / (2 * BAND_Q), a0 = 1 + alpha;
        float b0 = alpha / a0, a1 = -2 * c / a0, a2 = (1 - alpha) / a0;
        // Its power for white noise in, by its first few thousand samples
        // out.
        float y1 = 0, y2 = 0, z1 = 0, z2 = 0, sum = 0;
        for (int i = 0; i < 8000; i++) {
            float x = i == 0 ? 1.0f : i == 2 ? -1.0f : 0.0f;
            float y = b0 * x - a1 * y1 - a2 * y2;
            float z = y - a1 * z1 - a2 * z2;
            y2 = y1;
            y1 = y;
            z2 = z1;
            z1 = z;
            sum += z * z;
        }
        bandB0_[b] = (b & 2 ? -1 : 1) * b0 / sqrtf(sum / 3);
        bandA1_[b] = a1;
        bandA2_[b] = a2;
    }
    thumpBand_.tune(300, 8);
    for (int i = 0; i < TONES; i++) {
        tones_[i] = Tone();
    }
    for (int b = 0; b < BANDS; b++) {
        bandLog_[b] = log2f(BAND_HZ[b] * 0.001f);
        float sn, cs;
        sinCosTurns(0.5f * BAND_HZ[b] / RATE, sn, cs);
        bandTan_[b] = sn / cs;
        bandAirDbKm_[b] = absorptionDbKm(BAND_HZ[b]);
    }
    air_.tune(7000, 0.6f);
    // The speaker's high-pass, Butterworth at 180Hz: its two zeros at
    // nought, so b1 = -2 b0 and b2 = b0.
    {
        float s, c;
        sinCosTurns(180.0f / RATE, s, c);
        float alpha = s * 0.70710678f, a0 = 1 + alpha;  // s / (2Q), Q the root of a half
        hpB0_ = (1 + c) * 0.5f / a0;
        hpA1_ = -2 * c / a0;
        hpA2_ = (1 - alpha) / a0;
    }
    setSurges(1);
    shimmerKeep_[0] = expf(-SLOW_S / SHIMMER_S);
    shimmerKeep_[1] = sqrtf(1 - shimmerKeep_[0] * shimmerKeep_[0]);
    shimmerKeep_[2] = expf(-SLOW_S / SHIMMER_SLOW_S);
    shimmerKeep_[3] = sqrtf(1 - shimmerKeep_[2] * shimmerKeep_[2]);
}

// What is asked for, kept with the sample it was asked at - a snapshot each
// 20ms at most - for what is heard to be what was asked as long ago as the
// sound takes to come. Another kind, another aircraft: heard at once, the
// one before's history forgotten.
void Synth::set(const Sound &sound) {
    bool forget = sound.kind != target_.kind || sound.who != target_.who || sound.engine != target_.engine;
    // Placed, a jump no aircraft makes in the time between - its frame moved
    // under it, a replay sought back - leaves what was asked before in
    // another place: forgotten too, rather than heard from there. The time
    // between by the frames' own clock where they give it: by the sound made
    // since, a frame held up a third of a second - the last set again each
    // block meanwhile - had the next one a jump, and a far aircraft heard at
    // once from where it then was. A replay sought on is heard as flown
    // between.
    if (!forget && sound.placed && askedCount_ > 0 && asked_[askedNewest_].sound.placed) {
        const Asked &newest = asked_[askedNewest_];
        const float *a = sound.at, *b = newest.sound.at;
        float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
        float since = sound.ms != 0 && newest.sound.ms != 0 ? (int32_t)(sound.ms - newest.sound.ms) * 0.001f
                                                            : (clock_ - newest.at) / (float)RATE;
        float most = 400 * std::max(0.04f, since) + 50;
        forget = dx * dx + dy * dy + dz * dz > most * most;
    }
    if (forget) {
        askedCount_ = 0;
        askedNewest_ = -1;
        heardPlaced_ = false;
    }
    target_ = sound;
    if (askedCount_ > 0 && clock_ - asked_[askedNewest_].at < RATE / 50) {
        asked_[askedNewest_].sound = sound;
        return;
    }
    askedNewest_ = (askedNewest_ + 1) % HEARD;
    asked_[askedNewest_].at = clock_;
    asked_[askedNewest_].sound = sound;
    askedCount_ = std::min(HEARD, askedCount_ + 1);
}

void Synth::duck(bool under) { under_ = under; }

// The surges' pace and depth for an aircraft `scale` times an A320's size
// - see EngineData::jetScale. A bigger fan's intake and a bigger airframe
// take in and shed bigger, slower eddies: their hiss and their rush surge
// as many times slower [b777-sources 9.1], so a 777's approach churns less
// quickly. Its jet's surges keep their pace but go deeper: the recorded
// 777s' take-off passes swell more than an A320's model made them, 0.43 of
// their level at 0.25-1Hz against its 0.20 [M: the research's references].
void Synth::setSurges(float scale) {
    for (int c = 0; c < SOURCES; c++) {
        float pace = (c == OF_FAN || c == OF_AIRFRAME) ? scale : 1.0f;
        float kq = expf(-SLOW_S / (SURGE_QUICK_S[c] * pace)), ks = expf(-SLOW_S / (SURGE_SLOW_S[c] * pace));
        surgeKeep_[c][0] = kq;
        surgeKeep_[c][1] = sqrtf(1 - kq * kq);
        surgeKeep_[c][2] = ks;
        surgeKeep_[c][3] = sqrtf(1 - ks * ks);
        surgeKeep_[c][4] = sqrtf(1 - SURGE_QUICK_W[c] * SURGE_QUICK_W[c]);  // the slow part's weight
        surgeNp_[c] = SURGE_NP[c] * (c == OF_JET ? 1 + 0.25f * (scale - 1) : 1.0f);
    }
}

// Another aircraft starting: heard as it is now, its engines already turning,
// rather than spooling up from nothing; its own little differences from who
// it is.
void Synth::restart() {
    const Sound &s = heard_;
    traits_ = mix(s.who * 2654435761u + (uint32_t)s.engine * 97u + 12345u);
    setSurges(ENGINES[(int)s.engine].jetScale);
    // Its surges, the air's flicker and its noise its own each time it is
    // heard - from who it is, and where it and the camera are and when it
    // starts - so no two hearings share a realisation: not this aircraft's
    // last, not another scene's. (From a fixed seed every render had the
    // same surges, a 7dB dip at the same second in two scenes.)
    uint32_t where = mix(bitsOf(s.at[0]) ^ mix(bitsOf(s.at[1]) ^ mix(bitsOf(s.at[2]) ^ mix(bitsOf(s.ear[0]) ^
                                                                                        mix(bitsOf(s.ear[1]) ^ s.ms)))));
    slow_ = mix(traits_ ^ where) | 1;
    seed_ = mix(slow_ + 0x632be5abu);
    uint32_t h = traits_;
    auto next = [&]() {
        h = mix(h + 0x9e3779b9u);
        return unit(h);
    };
    // Its two engines a twentieth to a quarter of a percent apart in speed, one way
    // or the other - as near as autothrust holds two engines' N1, under what
    // an ear tells apart in a tone's pitch, enough for their lines never to
    // beat in step.
    split_ = (0.0005f + 0.002f * next()) * (next() < 0.5f ? -1 : 1);
    drift_ = 0;
    // Its nose gear's cavity and its fuel vents' pitches, a couple of percent
    // either way.
    nlgHz_ = 1720 * (0.98f + 0.04f * next());
    foppHz_[0] = 540 * (0.98f + 0.04f * next());
    foppHz_[1] = 585 * (0.98f + 0.04f * next());
    // Each fan's buzz-saw pattern: its noise now and three times as much from
    // as many fractions of a turn back, at strengths and signs of its own -
    // the harmonics of its turning then come out each as strong as those add
    // up to at it, some 5dB either way at random, as its blades' little
    // differences in stagger make them [P: Han et al.; DLR: each engine its
    // own], and the same at any speed. Their powers summed to one, so the
    // pattern moves the lines about and not the whole.
    for (int g = 0; g < 2; g++) {
        Comb &c = comb_[g];
        float sum = 1;
        c.w[0] = 1;
        c.turn[0] = 0;
        for (int k = 1; k < TAPS; k++) {
            c.turn[k] = 0.08f + 0.84f * next();
            c.w[k] = (0.45f + 0.55f * next()) * (next() < 0.5f ? -1 : 1);
            sum += c.w[k] * c.w[k];
        }
        for (int k = 0; k < TAPS; k++) {
            c.w[k] /= sqrtf(sum);
        }
    }
    float n1 = s.running ? wantN1(s, s) : 0.0f;
    n1_[0] = n1;
    n1_[1] = n1 * (1 + split_);
    n1Rate_[0] = n1Rate_[1] = 0;
    steadyS_ = n1 < 0.4f ? -1 : 2;
    reverse_ = s.reverse ? 1.0f : 0.0f;
    hunt_ = huntX1_ = huntX2_ = 0;
    huntWeight_ = s.flapsOut && !s.reverse && s.rollingMs < 0.5f ? 1.0f : 0.0f;
    gear_ = s.gearDown ? 1.0f : 0.0f;
    flaps_ = s.flapsOut ? 1.0f : 0.0f;
    spoilers_ = clamp01(s.spoilers);
    airspeed_ = std::max(0.0f, s.airspeedMs);
    logDistance_ = logf(std::max(5.0f, s.distanceM));
    closing_ = std::max(-SOUND_MS, std::min(0.7f * SOUND_MS, s.closingMs));
    convection_ = convectionTo_;
    aspect_ = clamp01(s.aspect);
    below_ = belowTo_;
    across_ = acrossTo_;
    spool_ = lit_ = s.running ? 1.0f : 0.0f;
    apu_ = s.apu ? 1.0f : 0.0f;
    for (int i = 0; i < TONES; i++) {
        tones_[i] = Tone();
    }
    for (Comb &c : comb_) {
        memset(c.line, 0, sizeof(c.line));
        memset(c.noise, 0, sizeof(c.noise));
        c.y1 = c.y2 = c.lo = c.hi = 0;
    }
    combOn_[0] = combOn_[1] = false;
    for (int b = 0; b < BANDS; b++) {
        bandY1_[b] = bandY2_[b] = bandZ1_[b] = bandZ2_[b] = 0;
        bandGain_[b] = bandBy_[b] = 0;
    }
    slowIn_ = 0;
    x1_ = x2_ = u1_ = u2_ = 0;
    std::fill(echo_, echo_ + ECHO, 0.0f);
    echoLow_ = echoLow2_ = echoWander_ = 0;
    air_.clear();
    touchdownWas_ = s.touchdownS;
    touchdownNowWas_ = target_.touchdownS;
    squealS_ = 10;
    squealDueS_ = -1;
    thump_ = 0;
    thumpBand_.clear();
    if (s.kind == Kind::LOW_BYPASS) {
        fighterRestart();
    }
    if (s.kind == Kind::TURBOPROP) {
        propRestart();
    }
}

// The sound's way, placed: back through what was asked, the moment that
// c x (now - then) = |the ear now - it then| - what left it then is what
// arrives now. From where the ear is now, so a cut to another camera hears
// it at once as that camera would. Coming at nearly the speed of sound,
// seconds of what it said arrive in a moment, high; going away, drawn out,
// low: its pitch from how fast it and the ear were closing along the way
// the sound came, each as it was moving - the camera moving with it, as the
// chase does, none.
void Synth::placedWay() {
    const float *ear = target_.ear;
    auto entry = [&](int back) -> const Asked & { return asked_[(askedNewest_ - back + HEARD) % HEARD]; };
    // How long ago each was so, seconds: by the frames' own times, and since
    // the newest by the sound made since it - or, with no times given, by
    // the sound alone.
    const Asked &newest = entry(0);
    float sinceNewest = (float)(clock_ - newest.at) / RATE;
    auto age = [&](const Asked &a) {
        return (newest.sound.ms != 0 ? (newest.sound.ms - a.sound.ms) * 0.001f + sinceNewest
                                     : (float)(clock_ - a.at) / RATE) +
               HEARD_AHEAD_S;
    };
    auto between = [&](const Asked &a, const Asked &b) { return age(a) - age(b); };  // a the older
    // Under nought, what left it then not here yet.
    auto arrived = [&](int back) {
        const Asked &a = entry(back);
        float dx = ear[0] - a.sound.at[0], dy = ear[1] - a.sound.at[1], dz = ear[2] - a.sound.at[2];
        return SOUND_MS * age(a) - sqrtf(dx * dx + dy * dy + dz * dz);
    };
    // The newest whose sound has arrived, found by halves - the further back,
    // the more of it has, as long as it goes slower than sound - or failing
    // that, the oldest there is: further off than ten seconds of sound.
    int lo = 0, hi = askedCount_ - 1;
    bool any = arrived(hi) >= 0;
    if (!any) {
        lo = hi;
    }
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (arrived(mid) >= 0) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    // Between it and the next newer: how far along, where it was and which
    // way it pointed.
    int k = lo;
    float u = 0;
    if (any && k > 0) {
        float a = arrived(k), b = arrived(k - 1);
        u = a - b > 0 ? a / (a - b) : 0;
    }
    const Asked &old = entry(k), &next = entry(std::max(0, k - 1));
    float at[3], nose[3];
    for (int i = 0; i < 3; i++) {
        at[i] = old.sound.at[i] + (next.sound.at[i] - old.sound.at[i]) * u;
        nose[i] = old.sound.nose[i] + (next.sound.nose[i] - old.sound.nose[i]) * u;
    }
    heard_ = (u < 0.5f ? old : next).sound;
    heard_.kind = target_.kind;
    heard_.who = target_.who;
    heard_.engines = target_.engines;
    heard_.fanHz = target_.fanHz;
    heard_.engine = target_.engine;
    heard_.fopp = target_.fopp;
    heard_.sourceUpM = old.sound.sourceUpM + (next.sound.sourceUpM - old.sound.sourceUpM) * u;
    heard_.earUpM = target_.earUpM;
    float to[3] = {ear[0] - at[0], ear[1] - at[1], ear[2] - at[2]};
    float d = sqrtf(to[0] * to[0] + to[1] * to[1] + to[2] * to[2]), per = 1 / std::max(1.0f, d);
    heard_.distanceM = d;
    // Each engine's distance and height: 5.75m either side, along the wing
    // - square to its nose and the vertical - from where it was; a
    // turboprop's propellers as far either side as its row has them (4.05m
    // on an ATR, nought for a single).
    {
        float wx = nose[1], wy = -nose[0], wn = sqrtf(wx * wx + wy * wy);
        float half = target_.kind == Kind::TURBOPROP ? propData(target_.engine).halfSpan : 5.75f;
        float k = wn > 1e-3f ? half / wn : 0;
        for (int g = 0; g < 2; g++) {
            float side = g == 0 ? k : -k;
            float ex = to[0] - wx * side, ey = to[1] - wy * side;
            engineM_[g] = sqrtf(ex * ex + ey * ey + to[2] * to[2]);
            engineUpM_[g] = heard_.sourceUpM;
        }
    }
    // Its speed along the way the sound came, from the two asked either side
    // of then; the ear's, from the last two - none across a cut from one
    // camera to another, a jump no camera moving makes. And its speed across
    // that way, for how fast the air's eddies flicker past it.
    float vs = 0, vo = 0, across = 0;
    if (askedCount_ > 1) {
        int i0 = std::max(k, 1);
        const Asked &a = entry(i0), &b = entry(i0 - 1);
        float s = between(a, b);
        if (s > 0) {
            float v[3] = {(b.sound.at[0] - a.sound.at[0]) / s, (b.sound.at[1] - a.sound.at[1]) / s,
                          (b.sound.at[2] - a.sound.at[2]) / s};
            vs = (v[0] * to[0] + v[1] * to[1] + v[2] * to[2]) * per;
            across = sqrtf(std::max(0.0f, v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - vs * vs));
        }
        const Asked &e1 = entry(1), &e0 = entry(0);
        float s1 = between(e1, e0);
        float mx = e0.sound.ear[0] - e1.sound.ear[0], my = e0.sound.ear[1] - e1.sound.ear[1],
              mz = e0.sound.ear[2] - e1.sound.ear[2], most = 400 * s1 + 20;
        if (s1 > 0 && mx * mx + my * my + mz * mz < most * most) {
            vo = (mx * to[0] + my * to[1] + mz * to[2]) * per / s1;
            // A camera keeping pace, as the chase does, sees the air go by
            // no faster than the aircraft goes past it.
            float w[3] = {mx / s1, my / s1, mz / s1}, vsx[3] = {0, 0, 0};
            if (s > 0) {
                vsx[0] = (b.sound.at[0] - a.sound.at[0]) / s;
                vsx[1] = (b.sound.at[1] - a.sound.at[1]) / s;
                vsx[2] = (b.sound.at[2] - a.sound.at[2]) / s;
            }
            float r[3] = {vsx[0] - w[0], vsx[1] - w[1], vsx[2] - w[2]};
            float along = (r[0] * to[0] + r[1] * to[1] + r[2] * to[2]) * per;
            across = sqrtf(std::max(0.0f, r[0] * r[0] + r[1] * r[1] + r[2] * r[2] - along * along));
        }
    }
    float doppler = (SOUND_MS - vo) / std::max(1.0f, SOUND_MS - vs);
    heard_.closingMs = SOUND_MS * (1 - 1 / std::max(0.1f, doppler));
    // How much louder its own coming makes it - what it said over a while
    // arriving in less - or quieter going: by its speed alone, the ear's
    // making none.
    convectionTo_ = SOUND_MS / std::max(1.0f, SOUND_MS - vs);
    acrossTo_ = across;
    // How far below it the ear is: the sine of the way down to it - for what
    // hangs under it, its gear, sounding down more than up.
    belowTo_ = clampf(-to[2] * per, -1, 1);
    float n = sqrtf(nose[0] * nose[0] + nose[1] * nose[1] + nose[2] * nose[2]);
    float c = (nose[0] * to[0] + nose[1] * to[1] + nose[2] * to[2]) * per / std::max(1e-6f, n);
    heard_.aspect = acosf(std::max(-1.0f, std::min(1.0f, c))) / PI;
}

// The fan speed the engines are asked for: given, or from the power the
// aircraft is asked for and what it is doing, by the engine's own figures.
float Synth::wantN1(const Sound &s, const Sound &rev) const {
    const EngineData &e = ENGINES[(int)s.engine];
    bool onGround = s.rollingMs > 0.5f || (s.sourceUpM < 12 && s.airspeedMs < 35);
    float power = clamp01(rev.reverse ? rev.power : s.power);
    float idle = onGround ? e.groundIdle : e.flightIdle;
    if (s.n1 >= 0) {
        return s.n1;
    }
    if (rev.reverse) {
        return idle + (e.reverse - idle) * clamp01((power - 0.05f) / 0.65f);
    }
    if (onGround && power < 0.2f) {
        // Taxiing: idle, and a little more to get it rolling and keep it so.
        return idle + 0.06f * clamp01((power - 0.05f) / 0.12f);
    }
    // Level at height, the air thin, takes far more of the fan than level
    // low down and slow - a flypast's.
    float level = e.cruise - (e.cruise - e.approach - 0.1f) * (1 - smoothstep(1500, 9000, s.sourceUpM));
    const float P[5] = {0, 0.45f, 0.65f, 0.88f, 1.0f}, N[5] = {idle, e.approach, level, e.climb, e.takeoff};
    for (int i = 1; i < 5; i++) {
        if (power <= P[i]) {
            return N[i - 1] + (N[i] - N[i - 1]) * (power - P[i - 1]) / (P[i] - P[i - 1]);
        }
    }
    return N[4];
}

// The engines' fan speeds, from what they are asked for and how fast an
// engine can change: on its fuel's schedule, slowly off idle - its core's
// compressor short of air - then quickly, from take-off's 50% step to its
// power 10-90% in 1.6s [M: 1.45-2.4s from the step, 1.8s a go-around]; and
// no overshoot. The
// reversers, and the power with them: as the picture shows them, with
// transients on the picture - see g_transientsOnPicture - else as late as
// the rest.
void Synth::engines() {
    float want = wantN1(sound_, g_transientsOnPicture ? target_ : sound_);
    // Take-off power asked for on the ground from idle: the thrust levers
    // to 50% N1 first, the engines held there till both are steady - two
    // seconds - then on to take-off power, as the procedure has it [P:
    // FCOM, quoted in UAE AAIS A6-ANV]: a step a listener hears, the fan's
    // pitch pausing half way up.
    bool ground = sound_.rollingMs < 30 && sound_.sourceUpM < 12;
    if (want < 0.4f || !ground) {
        steadyS_ = want < 0.4f ? -1 : 2;
    } else if (want > 0.6f && steadyS_ < 2) {
        if (steadyS_ < 0 && n1_[0] < 0.46f) {
            want = 0.5f;
        } else {
            steadyS_ = std::max(steadyS_, 0.0f) + STEP_S;
            want = 0.5f;
        }
    }
    // The second engine's speed a shade off the first's, drifting a tenth of
    // a percent over ten seconds or so.
    Rng rng{slow_};
    drift_ += -drift_ * (STEP_S / 10) + 0.001f * 0.02f * rng.gauss();
    slow_ = rng.s;
    for (int g = 0; g < 2; g++) {
        float to = want * (g == 0 ? 1 : 1 + split_ + drift_);
        float n = n1_[g], w = (to > n ? 1.0f + 0.8f * smoothstep(0.38f, 0.55f, n) : 1.4f) * ENGINES[(int)sound_.engine].spool;
        n1Rate_[g] += (w * w * (to - n) - 2 * w * n1Rate_[g]) * STEP_S;
        n1_[g] = clampf(n + n1Rate_[g] * STEP_S, 0, 1.1f);
    }
}

void Synth::update() {
    // What is heard: what was asked as long ago as the sound takes to come
    // from where it was - its distance then, near enough its distance now.
    float late = std::min(target_.distanceM, 2000.0f) / SOUND_MS;
    delayS_ = fabsf(late - delayS_) > 0.3f ? late : ease(delayS_, late, 1 - expFast(-STEP_S / 0.05f));
    // Placed, its way worked out each other step - 4ms - which is often
    // enough for a way that changes as slowly as an aircraft moves.
    bool placed = askedCount_ > 0 && target_.placed;
    if (placed && ((placedIn_ ^= 1) == 0 || !heardPlaced_)) {
        heard_ = target_;
        placedWay();
        heardPlaced_ = true;
    } else if (placed) {
        // as worked out a step ago
    } else if (askedCount_ > 0) {
        heardPlaced_ = false;
        heard_ = target_;
        uint32_t want = clock_ - (uint32_t)(std::max(0.0f, delayS_ - HEARD_AHEAD_S) * RATE);
        int at = askedNewest_;
        for (int k = 1; k < askedCount_ && (int32_t)(asked_[at].at - want) > 0; k++) {
            at = (at + HEARD - 1) % HEARD;
        }
        heard_ = asked_[at].sound;
        heard_.kind = target_.kind;
        heard_.who = target_.who;
        heard_.engines = target_.engines;
        heard_.fanHz = target_.fanHz;
        heard_.engine = target_.engine;
        heard_.fopp = target_.fopp;
        float d = std::max(1.0f, heard_.distanceM);
        engineM_[0] = engineM_[1] = d;
        engineUpM_[0] = engineUpM_[1] = heard_.sourceUpM;
        belowTo_ = clampf((heard_.sourceUpM - heard_.earUpM) / d, -1, 1);
        acrossTo_ = std::max(0.0f, heard_.airspeedMs) * sinf(PI * clamp01(heard_.aspect));
        convectionTo_ = 1;
    } else {
        heardPlaced_ = false;
        heard_ = target_;
    }
    // Another aircraft, or none, once the one before has faded out; the
    // same one, its figures as they come. Only an airliner's for now.
    bool same = heard_.kind == sound_.kind && heard_.who == sound_.who && heard_.engine == sound_.engine;
    if (!same && fade_ < 0.003f) {
        sound_.kind = heard_.kind;
        sound_.who = heard_.who;
        sound_.engine = heard_.engine;
        same = true;
        restart();
    }
    if (same) {
        sound_ = heard_;
    }
    bool on = same && sound_.kind == Kind::TURBOFAN;
    bool fighter = same && sound_.kind == Kind::LOW_BYPASS;  // THE FIGHTER: faded in as an airliner is
    bool prop = same && sound_.kind == Kind::TURBOPROP;      // THE TURBOPROP: likewise
    static const float FADE_IN = 1 - expf(-STEP_S / 0.5f), FADE_OUT = 1 - expf(-STEP_S / 0.2f),
                       DUCK = 1 - expFast(-STEP_S / 0.12f), REVERSE = 1 - expFast(-STEP_S / 0.9f),
                       WHERE = 1 - expFast(-STEP_S / 0.12f), CLOSING = 1 - expFast(-STEP_S / 0.25f),
                       ASPECT = 1 - expFast(-STEP_S / 0.2f), GEAR = 1 - expFast(-STEP_S / 2.5f),
                       FLAPS = 1 - expFast(-STEP_S / 3.0f), SPOILERS = 1 - expFast(-STEP_S / 0.25f),
                       AIRSPEED = 1 - expFast(-STEP_S / 0.5f), LIGHT = 1 - expFast(-STEP_S / 0.8f),
                       OUT = 1 - expf(-STEP_S / 0.3f), WIND_DOWN = expf(-STEP_S / 12), APU_DOWN = expf(-STEP_S / 5),
                       ECHO_WANDER = 1 - expf(-STEP_S / 0.6f);
    fade_ = ease(fade_, on || fighter || prop ? 1.0f : 0.0f, on || fighter || prop ? FADE_IN : FADE_OUT);
    duck_ = ease(duck_, under_ ? 0.2f : 1.0f, DUCK);
    const Sound &s = sound_;
    const Sound &rev = g_transientsOnPicture ? target_ : s;
    // The reversers' doors and sleeves out over a second or so, the fan's
    // stream thrown forward as they go: a burst, and the engines spooling up
    // behind it.
    reverse_ = ease(reverse_, rev.reverse ? 1.0f : 0.0f, REVERSE);
    gear_ = ease(gear_, s.gearDown ? 1.0f : 0.0f, GEAR);
    flaps_ = ease(flaps_, s.flapsOut ? 1.0f : 0.0f, FLAPS);
    spoilers_ = ease(spoilers_, clamp01(s.spoilers), SPOILERS);
    airspeed_ = ease(airspeed_, std::max(0.0f, s.airspeedMs), AIRSPEED);
    // Starting: the core on its starter to a third of idle - longer on a
    // neo, motored till its rotors are straight - the fuel lit there, and on
    // up to idle in 14 seconds more. Shut down: the fire out at once, the
    // spool winding down over half a minute. The APU spooling up over 25
    // seconds and down over a few.
    const EngineData &e = ENGINES[(int)s.engine];
    if (s.running) {
        spool_ = std::min(1.0f, spool_ + (spool_ < 0.35f ? 0.35f / e.motorS : 0.65f / 14) * STEP_S);
    } else {
        spool_ = spool_ < 0.002f ? 0 : spool_ * WIND_DOWN;
    }
    bool burning = s.running && spool_ >= 0.35f;
    lit_ = ease(lit_, burning ? 1.0f : 0.0f, burning ? LIGHT : OUT);
    apu_ = s.apu ? std::min(1.0f, apu_ + STEP_S / 25) : (apu_ < 0.002f ? 0 : apu_ * APU_DOWN);
    engines();
    // Where it is heard from: eased a little, so a cut from one camera to
    // another isn't a click.
    logDistance_ = ease(logDistance_, 0.693147181f * log2Fast(std::max(5.0f, s.distanceM)), WHERE);
    float closing = s.placed ? std::max(-SOUND_MS, std::min(0.7f * SOUND_MS, s.closingMs))
                             : std::max(-150.0f, std::min(150.0f, s.closingMs));
    closing_ = ease(closing_, closing, CLOSING);
    aspect_ = ease(aspect_, clamp01(s.aspect), ASPECT);
    below_ = ease(below_, belowTo_, ASPECT);
    across_ = ease(across_, acrossTo_, CLOSING);
    float d = expFast(logDistance_);
    doppler_ = SOUND_MS / (SOUND_MS - closing_);
    if (s.placed) {
        convection_ = ease(convection_, convectionTo_, CLOSING);
    }
    float up = std::max(0.0f, s.sourceUpM), ear = std::max(0.0f, s.earUpM);
    float rise = up - ear, across2 = std::max(1.0f, d * d - rise * rise);
    // Off the ground: the source's image under the ground, as far below it
    // as the source is above - the echo's way the straight way's length
    // with the two heights added rather than taken from each other. How much
    // later it comes; and how strong, the longer way spreading it further,
    // a runway giving back more than grass. Alike only low down the
    // spectrum: higher, the ground's roughness, the air's eddies and the
    // engines being metres across all scatter it - the further, the lower
    // that starts - so it comes back through a low-pass, and its lateness
    // wanders a little, as the eddies do. Coherent all the way up, as a
    // mirror would be, it swept its notches through a jet's whine as it
    // came, and the whine throbbed. None more than 32ms late.
    float straight = sqrtf(across2 + rise * rise), round = sqrtf(across2 + (up + ear) * (up + ear));
    Rng rng{slow_};
    echoWander_ = ease(echoWander_, rng.noise() * 3, ECHO_WANDER);
    slow_ = rng.s;
    // Each engine's echo as late as its own way makes it - the tones' sums
    // reckon with both - and the broadband's one echo between the two, its
    // lateness wandering a few samples as the eddies make it.
    echoGamma_ = 0;
    bool fits = true;
    for (int g = 0; g < 2; g++) {
        float m = engineM_[g], u = std::max(0.0f, engineUpM_[g]), ri = u - ear;
        float a2 = std::max(1.0f, m * m - ri * ri), back = sqrtf(a2 + (u + ear) * (u + ear));
        echoLate_[g] = (back - sqrtf(a2 + ri * ri)) / SOUND_MS * RATE;
        fits = fits && echoLate_[g] < ECHO - 12;
    }
    echoTapWas_ = echoTap_;
    if (fits) {
        echoTap_ = std::max(0.0f, 0.5f * (echoLate_[0] + echoLate_[1]) + echoWander_);
        echoGamma_ = (s.hardGround ? 0.6f : 0.45f) * straight / std::max(1.0f, round);
    }
    echoGain_ = echoGamma_;
    // Coherent to 2kHz x the root of 150m over the distance - 1.4kHz from
    // 300m, as the research has it - and close in, lower the nearer the ear
    // is to it: the engines, their nacelles and their jets sound from metres
    // of the air, each part's echo a different distance behind - over four
    // metres, by 2 x the ear's height x 4m / the distance.
    float coherent = clampf(std::min(2000 * sqrtf(150 / std::max(10.0f, d)), SOUND_MS * d / (16 * std::max(1.0f, ear))),
                            400, 3000);
    echoSoft_ = 1 - expFast(-2 * PI * coherent / RATE);

    // The spectrum's sums, a few times as long apart: 32ms; and each
    // buzz-saw's delay moved on toward its next.
    if (--slowIn_ < 0) {
        slowIn_ = SLOW - 1;
        slowStep();
    }
    comb_[0].late += comb_[0].lateBy;
    comb_[1].late += comb_[1].lateBy;
    // And each tone's pitch moved on toward its next.
    for (int i = 0; i < TONES; i++) {
        Tone &t = tones_[i];
        if (t.on && t.di != 0) {
            float cr = t.cr * t.dr - t.ci * t.di;
            t.ci = t.cr * t.di + t.ci * t.dr;
            t.cr = cr;
        }
    }
    tyresStep();
    if (fighter) {
        fighterStep();
    }
    if (prop) {
        propStep();
    }
}

// Every sixteenth step, 32ms: the loudness as it spreads, the bands' and the
// tones' sums, the buzz-saws' tuning - none of which moves faster than that,
// each eased across the next 32ms.
void Synth::slowStep() {
    const Sound &s = sound_;
    float d = expFast(logDistance_);
    // Spreading: the pressure a sound carries falling as one over the
    // distance - a sixth for each doubling - from 35m, then squeezed for a
    // small speaker on a desk, to the 0.65th power: a kilometre off a fifth
    // as loud as the physics, not a thirtieth, which it would play as
    // nothing. Low over the ground, held back too - Doc 29's lateral
    // attenuation, the ground's and the air's near it, as much as 11dB a
    // kilometre off at grazing, gone by 20 degrees up - for a camera on the
    // ground; and, placed, louder coming and quieter going, by no more than
    // 6dB up or 4 down - all squeezed alike.
    float up = std::max(0.0f, s.sourceUpM), ear = std::max(0.0f, s.earUpM);
    float rise = up - ear, lateral = sqrtf(std::max(1.0f, d * d - rise * rise));
    float beta = rise > 0 ? atanf(rise / lateral) * (180 / PI) : 0;
    heldDb_ = 0;
    if (beta < 50) {
        float gamma = lateral < 914 ? 1.089f * (1 - expFast(-0.00274f * lateral)) : 1.0f;
        heldDb_ = gamma * (1.137f - 0.0229f * beta + 9.72f * expFast(-0.142f * beta)) * clamp01((60 - ear) / 40);
    }
    gain_ = powFast(35 / std::max(35.0f, d) * dbAmp(-heldDb_), 0.65f);
    if (s.placed) {
        gain_ *= std::max(0.6f, std::min(2.0f, powFast(convection_, 0.65f)));
    }
    air_.tune(mufflingHz(d * 0.001f), 0.6f);
    if (s.kind == Kind::LOW_BYPASS) {
        fighterSlowStep();  // THE FIGHTER: none of the airliner's sums
        return;
    }
    if (s.kind == Kind::TURBOPROP) {
        propSlowStep();  // THE TURBOPROP: nor its
        return;
    }
    huntStep();
    bandsStep();
    tonesStep();
    combsStep();
}

// The autothrust hunting on the approach: holding the approach speed
// through the gusts, its N1 wanders by a percent and a half, to and fro
// every two to three seconds and never regularly [M: the fan tone of three
// A319 approaches wandering 1.0-2.4% about its trend, most at 2.1-2.9s,
// where a take-off's holds within 0.2-0.5%] - a gust-driven loop, so a
// resonance of its own driven by noise, as the tones are: two poles at
// 0.42Hz, damped by a third. Only in the air with the flaps out and the
// thrust off its idle stop - not on the ground, not at take-off or climb
// thrust, which hold their N1, not idling down an intermediate approach.
void Synth::huntStep() {
    const Sound &s = sound_;
    const EngineData &e = ENGINES[(int)s.engine];
    bool onGround = s.rollingMs > 0.5f || (s.sourceUpM < 12 && s.airspeedMs < 35);
    float n1 = 0.5f * (n1_[0] + n1_[1]);
    float hunting = onGround || reverse_ > 0.01f ? 0.0f
                                                 : flaps_ * smoothstep(e.flightIdle + 0.03f, e.flightIdle + 0.15f, n1) *
                                                       (1 - smoothstep(e.climb - 0.06f, e.climb, n1));
    Rng rng{slow_};
    float x = HUNT_A1 * huntX1_ + HUNT_A2 * huntX2_ + HUNT_B * rng.gauss();  // unit variance
    huntX2_ = huntX1_;
    huntX1_ = x;
    slow_ = rng.s;
    huntWeight_ = ease(huntWeight_, hunting, 1 - expFast(-SLOW_S / 2.0f));
    hunt_ = 0.015f * huntWeight_ * x;
}

// The broadband: each band as loud as the sources in it - each source's own
// spectrum, at the frequency the band is heard at (Doppler moving the whole
// spectrum, not the bands), its loudness by how hard the engines work and
// how fast the air goes past, and which way it faces - times that source's
// surges, shared by all its bands; the band's own shimmer; the air's
// absorption over the way, exactly, at the band's frequency; and the air's
// flicker. All in octaves and decibels, so each source in each band costs
// one power of two.
void Synth::bandsStep() {
    const Sound &s = sound_;
    const EngineData &e = ENGINES[(int)s.engine];
    Rng rng{slow_};
    // The slow processes, each a quick part and a slow part - noise smoothed
    // over a tenth of a second or so and over most of one - in units of
    // their own spread, so each source's surges are its own and have no beat.
    float *surge = surge_;  // kept: the buzz-saws' is theirs
    for (int c = 0; c < SOURCES; c++) {
        const float *k = surgeKeep_[c];
        surgeQuick_[c] = surgeQuick_[c] * k[0] + k[1] * rng.gauss();
        surgeSlow_[c] = surgeSlow_[c] * k[2] + k[3] * rng.gauss();
        float n = surgeKeep_[c][4] * surgeSlow_[c] + SURGE_QUICK_W[c] * surgeQuick_[c];
        surge[c] = exp2Fast(2.88539008f * surgeNp_[c] * (n - surgeNp_[c]));  // e^(2sn - 2s^2): its mean one
    }
    for (int b = 0; b < BANDS; b++) {
        shimmerQuick_[b] = shimmerQuick_[b] * shimmerKeep_[0] + shimmerKeep_[1] * rng.gauss();
        shimmerSlow_[b] = shimmerSlow_[b] * shimmerKeep_[2] + shimmerKeep_[3] * rng.gauss();
        shimmer_[b] = SHIMMER_QW * shimmerQuick_[b] + sqrtf(1 - SHIMMER_QW * SHIMMER_QW) * shimmerSlow_[b];
    }
    // The air's flicker: the turbulence along the way focusing and
    // scattering the sound, its level wandering by 1.25dB a kilometre off,
    // more further - the root of the distance - as much as 6dB, saturating;
    // above a frequency that falls with the distance, 9kHz over its root,
    // 350Hz from 650m - every band above it together; as fast as the
    // aircraft crosses the eddies, the slower the more it is seen head on [P:
    // Empa's turbulence shelf, Rietdijk's scintillation].
    float d = expFast(logDistance_), km = d * 0.001f;
    // Smooth, as eddies drifting through the way are - two stages, its
    // spectrum falling steeply above its rate - so seen head on, crossing
    // slowly, it wanders over seconds and moves the bands together only
    // slowly [M: bands correlated 0.07 on a take-off roll still coming].
    float kf = expFast(-SLOW_S * (across_ * FLICKER_ACROSS + FLICKER_BASE) * (1.0f / 5));
    flickerA_ = flickerA_ * kf + sqrtf(1 - kf * kf) * rng.gauss();
    flicker_ = flicker_ * kf + (1 - kf) * flickerA_;
    float flickerN = flickerN_ = flicker_ * (1 + kf) * (1 / sqrtf(1 + kf * kf));
    float flickerNp = std::min(6.0f, FLICKER_DB * sqrtf(km)) * (1 / 8.686f);
    float cornerLog = log2Fast(9 / sqrtf(d));  // octaves from 1kHz
    float airSin, airCos;  // the muffling's tuning, prewarped, as its cotangent
    sinCosTurns(0.5f * air_.hz / RATE, airSin, airCos);
    float airCot = airCos / std::max(1e-6f, airSin);
    // ...and close behind it, through its jet's and its wake's turbulence - as
    // the chase hears it - each band scattered its own way [P: the jet's
    // shear layer scattering what crosses it, haystacking its tones].
    float shimmerFar = SHIMMER_FAR_NP * std::min(1.0f, sqrtf(km)) + SHIMMER_PASS_NP * smoothstep(10, 80, across_) +
                       SHIMMER_WAKE_NP * smoothstep(0.85f, 0.97f, aspect_) * (1 - smoothstep(80, 250, d));
    slow_ = rng.s;
    // Each source's loudness, dB.
    float n1 = std::max(0.0f, 0.5f * (n1_[0] + n1_[1])) * (spool_ < 1 ? powFast(spool_, 1.6f) : 1.0f) * (1 + hunt_);
    // ...and as a share of its own take-off's: the jet's and the fan's hiss
    // reckoned from that - a LEAP's take-off at 81%, its fan bigger and
    // slower, is as hard as a CFM56's at 90%, its own level by levelDb.
    float n1r = std::max(0.02f, n1 / e.takeoff);
    float v = airspeed_, vr = std::max(0.05f, v / 70);
    float th = aspect_;
    // The jet: its roar as the eighth power of its speed relative to the air
    // [P: Lighthill], its speed as the fan's; its peak frequency with it; and
    // quieter as it flies, the jet's speed against the air the less - by no
    // more than 10dB [P: the flight effect, Stone]. In reverse only its
    // core's is still thrown aft. At take-off the strongest thing behind the
    // engines, a climb-out over the microphone 4-6dB over an approach's [P:
    // the noise data's departures and arrivals, with the distances squeezed
    // as here].
    float jetMs = e.jetMs * n1r;
    float flight = std::max(-10.0f, 50 * lg10(std::max(0.01f, 1 - v / std::max(60.0f, jetMs))));
    float jetDb = e.levelDb + 3 + 80 * lg10(n1r) + flight + lobe(JET, th) +
                  10 * lg10(std::max(1e-6f, lit_ * lit_ + 0.02f * spool_)) + 20 * lg10(1 - 0.7f * reverse_);
    // Its peak low behind it, where its big eddies' roar beams, and falling
    // the quicker above it; higher ahead and to the side, where its fine
    // turbulence's hiss is heard - two octaves up abeam, one and a half ahead.
    float jetPeak = log2Fast(0.17f * jetMs / 300) - log2Fast(e.jetScale) + 1.5f + 0.5f * smoothstep(0.25f, 0.5f, th) -
                    2 * smoothstep(0.5f, 0.83f, th);
    float coneCut = 3 * smoothstep(0.86f, 1.0f, th);  // the cone of silence, right behind: dB an octave over 500Hz
    float jetFall = 4 + smoothstep(0.6f, 0.85f, th);  // dB an octave over its peak: steeper behind, its big eddies' roar
    // The fan's broadband hiss: rising gently in third octaves from 300Hz to
    // a corner that follows its tips' speed - 1.6kHz at take-off power, 700Hz
    // idling - its ducts' liners taking the rest; and Heidmann's broad hump at
    // two and a half times its blade passing - 4kHz on an approach, as
    // Zellmann found it on CFM56s. The louder as its tips go faster - as
    // their speed to the fifth and the air through it, 14dB from take-off to
    // the approach [P: Heidmann], so the approach's engines are as loud as
    // its airframe [M: Zurich] and 9dB under a take-off's in the speaker's
    // band (they were 7: the approach overhead stood 2dB under a climb-out
    // over the microphone, where the noise data has 4.5-6.5); more slowly
    // idling, its core's own compressor and turbine hiss in it [M: Gatwick's
    // taxiing A320s]. Most of it out of its bypass duct, behind - an
    // intake's liners take more of it - each part by its own lobe: as loud
    // ahead as the intake's made it, a take-off coming stood 7-9dB too loud
    // against its pass, 10dB at 500Hz ahead [M: Zellmann's directivity].
    float shaft = (s.engine == Engine::GENERIC ? s.fanHz / e.blades : e.shaft100);
    float fanDb = e.levelDb - 6 + (n1r > 0.5556f ? 55 * lg10(n1r) : -14.0f + 12 * lg10(std::max(0.05f, n1r / 0.5556f)));
    float fanInDb = fanDb - 7 + lobe(FAN_IN, th), fanAftDb = fanDb - 3 + lobe(FAN_AFT, th);
    float fanHump = log2Fast(std::max(0.15f, 0.0025f * e.blades * shaft * n1));
    // The hump 6dB over the rest on the approach - the Zurich CFM56s' local
    // peak at 4.2kHz at 55% N1, the approach's spectrum rising again from 2
    // to 5kHz [M: Zellmann] - and under it at take-off power, where the jet's
    // and the fan's other hiss are the more, and idling, its tips too slow for
    // much of it, the core's rumble the more.
    float humpDb = 6 - 20 * smoothstep(0.55f, 0.9f, n1) - 11 * (1 - smoothstep(0.25f, 0.45f, n1));
    // Octaves from 1kHz: 1.6kHz at a CFM56-5B's take-off, three quarters of
    // an octave lower for each doubling of the fan's size - against the
    // recorded 777s, a whole octave left its take-off going away too dull,
    // half of one its pass too bright [M].
    float fanCorner = 0.678072f + 0.6f * log2Fast(std::max(0.1f, n1r)) - 0.75f * log2Fast(e.jetScale);
    // The core's combustion rumble: most at 450Hz, near enough as loud
    // idling as on the approach, behind it a little; lit only.
    float coreDb = e.levelDb - 24 + 25 * lg10(std::max(0.05f, std::min(n1, 0.55f) / 0.5f)) + lobe(CORE, th) +
                   20 * lg10(std::max(1e-3f, lit_));
    // The airframe: its rush going as the sixth power of the air's speed
    // [P: dipole, Curle] - a little off a clean wing; much more off the gear,
    // flat in third octaves to 5kHz (the approach's source flat to 6kHz once
    // the air's absorption is taken out), down more than up; and the flaps'
    // side edges at 1 to 1.6kHz; the spoilers on the roll-out, a low buffet
    // as the square of the speed. At 70m/s on the approach a shade under its
    // engines, as Zurich's A320s have them [M: "a very similar contribution"].
    // Its air the thinner the higher - the rush as the square of its density
    // [P: Curle] - and no faster than an approach's 1.6 times counted: the
    // clean wing at cruise, from the chase, a hiss under the engines, not a
    // gale.
    float thin = -0.00102f * std::max(0.0f, s.sourceUpM);  // e^(-h/8.5km), squared, in dB
    float rush = 60 * lg10(std::min(vr, 1.6f)) + thin + e.frameDb;
    float airframe = lobe(AIRFRAME, th);
    float gearDb = gear_ > 0.001f ? -29.5f + rush + 10 * lg10(gear_) + airframe + 1.5f * below_ -
                                        5 * std::max(0.0f, -below_)
                                  : -200;
    float flapDb = flaps_ > 0.001f ? -32 + rush + 10 * lg10(flaps_) + lobe(FLAPS, th) : -200;
    float cleanDb = -36 + rush + airframe;
    float buffetDb = spoilers_ > 0.001f ? -16.5f + 40 * lg10(vr) + 20 * lg10(spoilers_) : -200;
    // The reversers: the fan's stream thrown forward, mixing against the air
    // - more noise than as much thrust forward, Doc 29's 5dB [P] - under a
    // take-off's roll from as far, 1.5dB abeam in the speaker's band and 4dB
    // at 120 degrees [P: ANP and Doc 29 put it 4-6dB under, at the A320's
    // mean 68% N1]. They and the spoilers' buffet stayed where the old
    // synth's level, set by ear, had the chase's reverse (-15.6dBFS over its
    // nine seconds) when the rest came up 2.5dB: with them up too, one sample
    // in a hundred there went into the limiter, seven times as many as before.
    float revDb = reverse_ > 0.001f ? e.levelDb - 6.5f + 70 * lg10(std::max(0.05f, n1 / 0.7f)) + lobe(REVERSE, th) +
                                          20 * lg10(reverse_)
                                    : -200;
    // The tyres rolling: a low roar of rubber on concrete, under the engines.
    float roll = s.rollingMs;
    float rollDb = roll > 0.5f ? -32 + 30 * lg10(std::max(0.05f, roll / 70)) : -200;
    // The APU's exhaust from the tail cone: its combustion's hump at 250 to
    // 350Hz, some 9dB under the engines taxiing.
    float apuDb = apu_ > 0.001f ? -24 + 20 * lg10(apu_) + lobe(APU, th) : -200;
    float logDoppler = log2Fast(doppler_);
    constexpr float L5000 = 2.32193f, L180 = -2.47393f, L300 = -1.73697f, L320 = -1.64386f, L330 = -1.59946f,
                    L450 = -1.15200f, L600 = -0.736966f, L700 = -0.514573f, L900 = -0.152003f, L1250 = 0.321928f;
#ifdef ENGINE_NOISE_DEBUG
    static int debugIn = 0;
    bool debug = --debugIn < 0;
    if (debug) {
        debugIn = 50;
        fprintf(stderr, "t %.2f d %.0f asp %.2f n1 %.3f v %.0f | dB: jet %.1f fanIn %.1f fanAft %.1f core %.1f gear %.1f "
                        "flap %.1f clean %.1f rev %.1f roll %.1f apu %.1f\n",
                clock_ / (float)RATE, d, th, n1, v, jetDb, fanInDb, fanAftDb, coreDb, gearDb, flapDb, cleanDb, revDb,
                rollDb, apuDb);
    }
#endif
    for (int b = 0; b < BANDS; b++) {
        float lf = bandLog_[b] - logDoppler;  // the band's frequency where it left the source, octaves from 1kHz
        // The jet: rising to its peak, then falling 4dB an octave in band
        // level - 5 behind - and 3 more above 2kHz; less of the treble right
        // behind.
        float xj = lf - jetPeak;
        float jet = jetDb - (xj < 0 ? -3 * xj : jetFall * xj) - 3 * std::max(0.0f, lf - 1.0f) -
                    coneCut * std::max(0.0f, lf + 1.0f);
        // Out of the bypass duct the corner lower - its liners and the
        // nozzle taking more of the treble - and gentler above it, 4.5dB an
        // octave: so a climb-out overhead falls 6dB an octave over 1kHz, as
        // recorded, and one going away, 1.5km off, keeps the 2 and 4kHz the
        // recordings have, 3-4dB more than at 7dB an octave [M].
        float xh = lf - fanHump, hump = dbPower(humpDb - std::min(20.0f, 1.7f * xh * xh)), low = L300 - lf;
        float fanShape =
            dbPower(-7 * std::max(0.0f, lf - fanCorner) - 2 * std::max(0.0f, fanCorner - lf) - 3 * std::max(0.0f, low)) + hump;
        float aftShape = dbPower(-4.5f * std::max(0.0f, lf - fanCorner + 0.678f) - 3 * std::max(0.0f, fanCorner - 0.678f - lf) -
                                 3 * std::max(0.0f, low)) +
                         hump;
        float xc = lf - L450;
        float pJet = dbPower(jet), pFanAft = aftShape * dbPower(fanAftDb);
        float p = pJet * surge[OF_JET] + fanShape * dbPower(fanInDb) * surge[OF_FAN] + pFanAft * surge[OF_FAN_AFT] +
                  dbPower(coreDb - std::min(25.0f, 4 * xc * xc)) * surge[OF_CORE];
        float turbulent = pJet;
        float xw = lf - L600, rushing = dbPower(cleanDb - std::min(25.0f, 4 * xw * xw));
        if (gearDb > -150) {
            rushing += dbPower(gearDb - 3 * std::max(0.0f, lf - L5000) - 6 * std::max(0.0f, L180 - lf));
        }
        if (flapDb > -150) {
            float xp = lf - L1250;
            rushing += dbPower(flapDb - std::min(25.0f, 5 * xp * xp));
        }
        if (rollDb > -150) {
            float xt = lf - L700;
            rushing += dbPower(rollDb - std::min(25.0f, 5 * xt * xt));
        }
        p += rushing * surge[OF_AIRFRAME];
        turbulent += 0.6f * rushing;
        if (revDb > -150) {
            float xr = lf - L900, pr = dbPower(revDb - std::min(25.0f, 3 * xr * xr));
            p += pr * surge[OF_REVERSER];
            turbulent += pr;
        }
        if (buffetDb > -150) {
            float xb = lf - L330, pb = dbPower(buffetDb - std::min(25.0f, 4 * xb * xb));
            p += pb * surge[OF_BUFFET];
            turbulent += pb;
        }
        if (apuDb > -150) {
            float xa = lf - L320;
            p += dbPower(apuDb - (xa < 0 ? -6 * xa : 3 * xa)) * surge[OF_APU];
        }
        // The band's own shimmer and its flicker, a shelf over the corner
        // frequency; and the air's absorption on the way made exact at the
        // band's centre - one power of two for the three. (The absorption
        // is mostly the muffling's, after the bank: a band-pass's skirts
        // would fill the treble the air takes 20 or 40dB of. But a low-pass
        // isn't the air: from a kilometre and a half it took 4-6dB too much
        // at 1.6-2.5kHz against 400Hz, and 1-2dB too little under 630Hz - a
        // climb-out going away darker and quieter than it is. So each band
        // is given the difference, the air's loss at its centre less the
        // low-pass's there.)
        float fl = flickerNp * smoothstep(cornerLog - 0.5f, cornerLog + 0.5f, bandLog_[b]);
        float sh = SHIMMER_NP + SHIMMER_TURBULENT_NP * clamp01(turbulent / std::max(1e-12f, p)) + shimmerFar;
        float w = bandTan_[b] * airCot, w2 = w * w;
        float lowPassDb = 4.34294482f * 0.693147181f * log2Fast((1 - w2) * (1 - w2) + w2 * (1 / (0.6f * 0.6f)));
        float airDb = clampf(lowPassDb - bandAirDbKm_[b] * km, -30, 15);
        float ln = 2 * sh * (shimmer_[b] - sh) + 2 * fl * (flickerN - fl) + 0.230258509f * airDb;
        p *= exp2Fast(1.44269504f * ln);
#ifdef ENGINE_NOISE_DEBUG
        if (debug) {
            fprintf(stderr, "   band %4.0f: %6.1f dB (jet %6.1f fan %6.1f aft %6.1f)\n", BAND_HZ[b], 10 * lg10(p + 1e-20f),
                    10 * lg10(pJet + 1e-20f), 10 * lg10(fanShape * dbPower(fanInDb) + 1e-20f), 10 * lg10(pFanAft + 1e-20f));
        }
#endif
        // Reached by the next slow step, a little each sample.
        bandBy_[b] = (sqrtf(p) - bandGain_[b]) * (1.0f / (SLOW * STEP));
    }
}

// The resonances: each tone's pitch as heard - Doppler on it - its width,
// its loudness by its source's physics, which way it faces, the air's
// absorption at its pitch, the ground's echo on it worked out as a sum, and
// the air's flicker; and from those, how far its pointer turns and shrinks
// each sample, and how hard the noise pushes it. A tone's loudness set by
// the push, it follows over its own ringing - a tenth of a second or so -
// so nothing about it ever steps.
void Synth::tonesStep() {
    const Sound &s = sound_;
    const EngineData &e = ENGINES[(int)s.engine];
    float d = expFast(logDistance_), km = d * 0.001f;
    float th = aspect_;
    float flickerNp = std::min(6.0f, FLICKER_DB * sqrtf(km)) * (1 / 8.686f), cornerLog = log2Fast(9000 / sqrtf(d));
    float flickerN = flickerN_;
    // The ground's echo, straight from its strength and lateness: its sum
    // with the straight sound at each pitch, the coherent part and the
    // scattered part - the scattered more, the higher and further - held to
    // 2dB either way of their powers' sum, so a tone swells and fades a
    // little as the echo's notches sweep by, but never throbs - its swells
    // as deep as narrow-band noise's, no deeper [M: 4.7-5.3dB]. (Held to 3,
    // a fan's whine past a tower over a runway, its echo's notches sweeping
    // it four times a second, left its octave's envelope as peaky as a
    // klaxon's - kurtosis 6.8 where the recordings stay under 5.5.)
    float gamma = echoGamma_, mean = 1 + gamma * gamma;
    float lateOf[3] = {echoLate_[0] / RATE, echoLate_[1] / RATE, 0.5f * (echoLate_[0] + echoLate_[1]) / RATE};
    float coherent = 1 / clampf(std::min(2000 * sqrtf(150 / std::max(10.0f, d)),
                                         SOUND_MS * d / (16 * std::max(1.0f, sound_.earUpM))),
                                400, 3000);
    float n1Spool = (spool_ < 1 ? powFast(spool_, 1.6f) : 1.0f) * (1 + hunt_);
    float shaft = (s.engine == Engine::GENERIC ? s.fanHz / e.blades : e.shaft100);
    // Its width: 1.2Hz near - its swells and fades as slow as a real fan
    // tone's, half their power under 1 to 5Hz [M] - wider far off as the
    // air's eddies broaden it [P: phase fluctuations], and its harmonics'
    // wider; the tone from the bypass duct, behind, broadened more through
    // the jet's shear layer.
    float widen = 1.2f + 3.5f * km, aft = smoothstep(0.55f, 0.8f, th);
    // `db` its level, `width` its width in hertz, `stream` which noise, and
    // `from` which engine it comes from - 2 the airframe, between them.
    auto set = [&](int slot, float hz, float db, float width, int stream, int from) {
        Tone &t = tones_[slot];
        float heard = hz * doppler_;
        if (heard < 30 || heard > TOP_HZ || db < -110) {
            t.in = 0;
            t.dr = 1, t.di = 0;
            t.on = t.on && (t.zr * t.zr + t.zi * t.zi > 1e-14f);
            t.turn = t.on ? t.turn : 0;
            return;
        }
        // The air's absorption at its pitch and its flicker above the corner,
        // as dB, with its level - one power of two for its loudness.
        float lh = log2Fast(heard);
        float o = clampf(lh - 5.97728f, 0, 6.999f);  // octaves from 63Hz
        int i = (int)o;
        float absorbs = exp2Fast(ABSORPTION_LOG2[i] + (ABSORPTION_LOG2[i + 1] - ABSORPTION_LOG2[i]) * (o - i));
        float w = smoothstep(cornerLog - 0.5f, cornerLog + 0.5f, lh) * flickerNp;
        float amp = dbAmp(db - absorbs * km + 8.68589f * (w * flickerN - w * w));
        if (heard > TOP_HZ - TOP_FADE_HZ) {
            amp *= (TOP_HZ - heard) * (1 / TOP_FADE_HZ);
        }
        if (gamma > 0) {
            float x = heard * coherent, x2 = x * x, coh = 1 / (1 + x2 + 0.5f * x2 * x2), gc = gamma * coh;
            float turns = heard * lateOf[from] + 0.25f;  // its cosine, as a sine a quarter turn on
            turns -= (float)(int)turns;
            float cs = sinTurn(turns > 0.5f ? turns - 1 : turns);
            float p = 1 + gc * gc + 2 * gc * cs + gamma * gamma * (1 - coh * coh);
            amp *= sqrtf(clampf(p / mean, 0.63f, 1.58f) * mean);
        }
        // Shrinking by pi x width / rate each sample - as exp would have it,
        // the width small - and turning by its pitch; pushed hard enough for
        // its real part to carry `amp` as its RMS, the noise a third of a
        // square's power. Its pitch got to over the next slow step, a little
        // each step - its turn turned on by a sixteenth of the change each
        // time - not jumped to: a fan's whine falling 600Hz a second as it
        // passes, by 19Hz steps 31 times a second, had sidebands 26dB down.
        float x = PI * width / RATE, r = 1 - x * (1 - x * (0.5f - x * (1.0f / 6)));
        float turn = heard * (1.0f / RATE), was = t.on && t.turn > 0 ? t.turn : turn;
        float sn, cs;
        sinCosTurns(was, sn, cs);
        float norm = r / sqrtf(sn * sn + cs * cs);
        t.cr = cs * norm;
        t.ci = sn * norm;
        sinCosTurns((turn - was) * (1.0f / SLOW), sn, cs);
        float unit = 1 / sqrtf(sn * sn + cs * cs);  // exactly a turn, its radius kept
        t.dr = cs * unit;
        t.di = sn * unit;
        t.turn = turn;
        t.in = amp * sqrtf(6 * (1 - r * r));
        t.stream = (uint8_t)stream;
        t.wide = width > 20;
        t.on = true;
    };
    // The fan's blade-passing tone and its harmonics, each engine's: rising
    // steeply with its tips' speed to the approach's, 16dB from idle, and
    // little more above, where the buzz-saw takes over; its second harmonic
    // 6dB under, its third 12. Standing on the ground, louder: a vortex off
    // the ground swallowed by its intake, stirring the fan [M: DLR's V2500
    // ground runs, "much higher than in flight"] - which forms only while
    // the intake draws its air some seven times faster than the engine
    // moves through it: blown away by 4-7m/s idling, so an idling fan's
    // line stands out at the stand [M: 15dB at an apron] and not taxiing
    // [M: none over 6dB], and by 15-30m/s at take-off power. Each engine's
    // own line at its blade passing, the two a shade apart; their harmonics,
    // the two engines' lines as near as their beat, one resonance each as
    // wide as both - as they fade, the quicker, as real harmonics do [M: 2x a
    // CFM56's tone, half its swells' power under 5Hz, the tone's under 1.2].
    bool onGround = s.rollingMs > 0.5f || (s.sourceUpM < 12 && s.airspeedMs < 35);
    float blown = 25 * std::max(0.1f, 0.5f * (n1_[0] + n1_[1]) * n1Spool / 0.9f);  // m/s
    float vortex = onGround ? 1 - smoothstep(0.5f * blown, blown, airspeed_) : 0;
    float fanTone = lobe(FAN_TONE, th) + e.toneDb + 12 * vortex;
    float bpfs[2], dbs[2], width = 0;
    for (int g = 0; g < 2; g++) {
        float n1 = n1_[g] * n1Spool;
        bpfs[g] = e.blades * shaft * n1;
        float law = n1 < 0.55f ? 40 * lg10(std::max(0.02f, n1 / 0.55f)) : 18 * lg10(n1 / 0.55f);
        dbs[g] = n1 > 0.01f ? -23.5f + e.levelDb + law + fanTone : -200;
        width = widen + 3.0f * aft * n1;
        set(FAN_1 + g, bpfs[g], dbs[g], width, g, g);
    }
    // A fan whose blades pass under 300-450Hz - a neo's idling, 190-290Hz -
    // heard by its harmonics: its stator's interaction with the fundamental
    // cut off in the duct at so low a speed, the second's not, and the
    // speaker plays nothing so low anyway [P: the research's k = 2-4].
    float bpf = 0.5f * (bpfs[0] + bpfs[1]), apart = fabsf(bpfs[1] - bpfs[0]), db = 0.5f * (dbs[0] + dbs[1]) + 3;
    float low = 1 - smoothstep(300, 450, bpf);
    set(FAN_H2, 2 * bpf, db - 6 + 4 * low + e.h2Db, width * 1.4f + 2 * apart, 0, 2);
    set(FAN_H2 + 1, 3 * bpf, db - 12 + 6 * low, width * 1.8f + 3 * apart, 1, 2);
    // A V2500's turbine tones, its last two stages', behind it, smeared broad
    // by the jet - a haystack - the most on the approach.
    float n1 = 0.5f * (n1_[0] + n1_[1]) * n1Spool;
    if (e.turbine[0] > 0) {
        float db = lit_ > 0.001f ? -25 + 30 * lg10(std::max(0.05f, n1 / 0.5f)) + lobe(TURBINE, th) + 20 * lg10(lit_)
                                 : -200;
        set(TURBINE_1, e.turbine[0] * shaft * n1, db, 25 + 30 * n1, 1, 2);
        set(TURBINE_1 + 1, e.turbine[1] * shaft * n1, db, 25 + 30 * n1, 1, 2);
    }
    // The nose gear's cavity tone at 1.7kHz, on every A320 [M: TU Delft]: its
    // pitch the cavity's, its loudness strongly with the speed - as the
    // eighth power [I] - more than 10dB over the rush about it [M], from
    // under the aircraft; with the gear down. A cavity's resonance, as
    // narrow as the fuel vents' [P: Empa's 2Hz]: 11-14dB over its
    // neighbourhood in a hertz-fine spectrum with the Doppler taken out, as
    // a real approach's steady line stands [M: 13.6dB at 2.05-2.09kHz
    // heard, an A319 coming at 70m/s].
    float v = airspeed_, vr = std::max(0.05f, v / 70);
    float nlg = gear_ > 0.01f && v > 20 ? -30.5f + 80 * lg10(vr) + 10 * lg10(gear_) + lobe(AIRFRAME, th) + 2 * below_ -
                                              8 * std::max(0.0f, -below_)
                                        : -200;
    set(NOSE_GEAR, nlgHz_, nlg, 2.0f + 0.5f * widen, 0, 2);
    // The fuel vents' whistle, an unmodified ceo's: two cavities on the
    // wings' undersides ringing near 540 and 585Hz, a couple of hertz wide
    // [P: Empa], as the 10.3th power of the speed - 10dB from 80 to 100m/s -
    // so heard out at 165 to 230 knots, and all but gone by the threshold -
    // and under engines at any power, as a flypast's; at Mach 0.35 some 5dB
    // over the 500Hz third octave's rush, as Empa measured; down and
    // sideways from the wing.
    float vent = s.fopp && v > 40 && v < 170 ? -16.5f + 103 * lg10(v / 119) + lobe(AIRFRAME, th) + 2 * below_ -
                                                   6 * std::max(0.0f, -below_) +
                                                   20 * lg10(std::max(1e-3f, 1 - smoothstep(140, 170, v)))
                                             : -200;
    set(VENT_1, foppHz_[0], vent, 2.0f + 0.3f * widen, 0, 0);
    set(VENT_1 + 1, foppHz_[1], vent - 2, 2.0f + 0.3f * widen, 0, 1);
    // The core's compressor whine, each engine's: its high-pressure spool's
    // blades passing, with N2 - turning first as the engine starts, on the
    // starter to a third of idle before it lights, a whine rising from
    // nothing, then on up to idle's 3.56 and 3.68kHz, the two engines a few
    // percent apart, as measured over an A319's apron with its engines
    // running [M: 16-25dB over the rest; I: the HP compressor's rather than
    // the APU's]; fading as the power comes on, the fan's drowning it, and
    // none at all past 72% N2 - an approach's - where no recording has it.
    // Out of the intake, ahead.
    float n2 = spool_ < 1 ? 0.585f * powFast(std::max(spool_, 1e-3f), 0.7f)
                          : 0.585f + 0.415f * clamp01((n1 - e.groundIdle) / (e.takeoff - e.groundIdle));
    float coreTone = lobe(FAN_TONE, th) + 4 * vortex;
    float whine = n2 > 0.05f && n2 < 0.72f && spool_ > 0.02f
                      ? -36 + 10 * lg10(n2 / 0.585f) - 20 * smoothstep(0.62f, 0.72f, n2) + coreTone
                      : -200;
    set(CORE_1, 3620 * (n2 / 0.585f) * 0.984f, whine, widen, 0, 0);
    set(CORE_1 + 1, 3620 * (n2 / 0.585f) * 1.016f, whine - 2, widen, 1, 1);
    // The APU's whine [I: no A320 APU's pitch is published]: its generator's,
    // turning for the aircraft's 400Hz power - its sixth harmonic and its
    // third - rising to pitch as it spools up, from the tail cone.
    float apuHz = 0.3f + 0.7f * apu_;
    float apuDb = apu_ > 0.001f ? -37 + 40 * lg10(apu_) + lobe(APU, th) : -200;
    set(APU_1, 2400 * apuHz, apuDb, 1.5f + 0.5f * widen, 0, 2);
    set(APU_1 + 1, 1200 * apuHz, apuDb - 6, 1.5f + 0.5f * widen, 1, 2);
    tonesOn_ = 0;
    for (int i = 0; i < TONES; i++) {
        tonesOn_ += tones_[i].on;
    }
}

// The buzz-saw, each engine's: its shaft's turning as heard, a delay of one
// turn fed back - each turn as wide a line at each harmonic as `g` makes it
// - and its noise shaped to where the lines are strongest, a band from the
// sixth harmonic to the sixteenth, falling off toward its blade passing, and
// muffled by the air as the way is long. Over its N1 - its tips past the
// speed of sound - its level rising 15dB as they go faster, out of the
// intake only.
void Synth::combsStep() {
    const Sound &s = sound_;
    const EngineData &e = ENGINES[(int)s.engine];
    float d = expFast(logDistance_), km = d * 0.001f;
    float th = aspect_;
    float n1Spool = (spool_ < 1 ? powFast(spool_, 1.6f) : 1.0f) * (1 + hunt_);
    float shaft = (s.engine == Engine::GENERIC ? s.fanHz / e.blades : e.shaft100);
    float flickerNp = std::min(6.0f, FLICKER_DB * sqrtf(km)) * (1 / 8.686f);
    float flickerN = flickerN_;
    // The shocks as strong as the air the fan swallows lets them be: they
    // surge with the turbulence it takes in, all the lines together, on
    // their own - each line's own fading is its own too (a dozen lines
    // fading apart add up to a steady rasp, which no recorded take-off has;
    // surging with the intake's hiss, the bands of a roll still coming
    // moved together, where the recordings' move apart).
    float surged = sqrtf(surge_[OF_BUZZ]);
    for (int g = 0; g < 2; g++) {
        Comb &c = comb_[g];
        float n1 = n1_[g] * n1Spool, hz = shaft * n1 * doppler_;
        float on = smoothstep(e.buzzOn - 0.02f, e.buzzOn + 0.02f, n1);
        float db = -11.0f + e.levelDb + e.buzzDb - 15 * (1 - smoothstep(e.buzzOn, e.buzzOn + 0.12f, n1)) + lobe(BUZZ, th);
        float amp = on * dbAmp(db) * surged * expFast(flickerNp * flickerN - flickerNp * flickerNp);
        if (amp < 1e-6f || hz < RATE / (float)(COMB - 8)) {
            c.in = 0;
            c.lateBy = 0;
            if (combOn_[g] && c.y1 * c.y1 + c.y2 * c.y2 < 1e-14f) {
                combOn_[g] = false;
                memset(c.line, 0, sizeof(c.line));
                c.y1 = c.y2 = c.lo = c.hi = 0;
            }
            continue;
        }
        bool was = combOn_[g];
        combOn_[g] = true;
        // One turn late, less the sample the loop's smoothing adds - got to
        // a little each step, not jumped to, its read moving by a fraction of
        // a sample at a time as its speed and its Doppler change, not a click;
        // and its pattern's taps, each its fraction of a turn back.
        float late = RATE / hz - 1;
        if (!was || fabsf(late - c.late) > 40) {
            c.late = late;
        }
        c.lateBy = (late - c.late) * (1.0f / SLOW);
        for (int k = 1; k < TAPS; k++) {
            c.tap[k] = (int)(c.turn[k] * (late + 1) + 0.5f);
        }
        // Each line 2.5Hz wide near, wider far off - as the tones are.
        float width = 2.0f + 2.0f * km;
        c.g = expFast(-PI * width / hz);
        // The band: the difference of two one-pole low-passes, at the fourth
        // harmonic and at the eighteenth (or its blades', if fewer) - the
        // lines strongest from 300Hz to a kilohertz and a half, and on less
        // strongly to the blade passing, as recorded take-offs have them [M:
        // orders 6-7 and 12-14 static (DLR), 18-22 among the strongest on a
        // CFM56 in flight; P: a broad peak at orders 8-16, falling toward the
        // blade passing] - and what power of white noise each lets through
        // and the two share. (Flat to a CFM56's 36th, its upper lines made the
        // take-off's 2.5-5kHz 5-12dB too strong ahead of it.)
        float loHz = e.buzzLo * hz, hiHz = std::min(std::min(e.buzzHi, e.blades) * hz, 6000.0f);
        float kl = 1 - expFast(-2 * PI * loHz / RATE), kh = 1 - expFast(-2 * PI * hiHz / RATE);
        c.kLo = kl;
        c.kHi = kh;
        float power = kh / (2 - kh) + kl / (2 - kl) - 2 * kl * kh / (kl + kh - kl * kh);
        // Pushed hard enough for the lines' sum to carry `amp`, the noise a
        // third of a square's power.
        c.in = amp * sqrtf((1 - c.g * c.g) / (std::max(1e-6f, power) * (1.0f / 3)));
    }
}

// The tyres. Touching down, each main gear's in turn a squeal - its tyres
// dragged along the runway till they spin up, a band a tenth of a kilohertz
// wide falling from 2.3kHz to 1.3 as the slip does, a tenth of a second -
// the louder the harder it came down: the loudest short thing heard from
// within a hundred metres [I: the research], 15dB over the approach's roar
// in its band from 45m, still 10 from 250m, where the choice of when to hear
// it is made. Heard as late as its distance makes it or, with transients on
// the picture, as the picture shows it, a tenth of a second late at most.
// Rolling, the nose wheel over each centreline light, one every 15 metres, a
// thump each.
void Synth::tyresStep() {
    const Sound &s = sound_;
    float d = expFast(logDistance_);
    auto start = [&](float sinkMs, float rollingMs) {
        squealS_ = 0;
        squealLoud_ = std::min(1.6f, std::max(0.5f, sinkMs / 1.5f)) * std::min(1.0f, rollingMs / 50);
        squealAt_[0] = 0;
        Rng rng{slow_};
        squealAt_[1] = 0.03f + 0.06f * rng.uniform();
        slow_ = rng.s;
    };
    if (g_transientsOnPicture) {
        // As the picture shows it: how hard it came down and how fast it
        // rolled taken from the frame that showed it touch - a frame or
        // two on, a caller may have the sink back at its default, and the
        // squeal came out 6dB quiet - and heard as late as the sound would
        // be, at most 75ms on: within a tenth of a second of the picture,
        // give or take the frame it fell between.
        const Sound &now = target_;
        if (now.touchdownS >= 0 && (touchdownNowWas_ < 0 || now.touchdownS + 0.05f < touchdownNowWas_)) {
            squealDueS_ = std::max(0.0f, std::min(d / SOUND_MS, 0.075f) - HEARD_AHEAD_S);
            squealSinkMs_ = now.sinkMs;
            squealRollingMs_ = now.rollingMs;
        }
        touchdownNowWas_ = now.touchdownS;
        if (squealDueS_ >= 0) {
            squealDueS_ -= STEP_S;
            if (squealDueS_ < 0) {
                start(squealSinkMs_, squealRollingMs_);
            }
        }
    } else if (s.touchdownS >= 0 && (touchdownWas_ < 0 || s.touchdownS + 0.05f < touchdownWas_)) {
        start(s.sinkMs, s.rollingMs);
    }
    touchdownWas_ = s.touchdownS;
    squealS_ += STEP_S;
    float km = d * 0.001f;
    for (int g = 0; g < 2; g++) {
        Tone &t = tones_[SQUEAL_1 + g];
        float at = squealS_ - squealAt_[g];
        if (at < 0 || at > 0.6f) {
            t.in = 0;
            t.on = t.on && (t.zr * t.zr + t.zi * t.zi > 1e-14f);
            continue;
        }
        float hz = (1300 + 1000 * expFast(-at / 0.07f)) * doppler_;
        float envelope = (1 - expFast(-at / 0.003f)) * expFast(-at / 0.09f);
        float amp = 0.65f * squealLoud_ * envelope * dbAmp(-absorptionDbKm(hz) * km);
        float width = 110, x = PI * width / RATE, r = 1 - x * (1 - x * (0.5f - x * (1.0f / 6)));
        float sn, cs;
        sinCosTurns(hz / RATE, sn, cs);
        t.cr = cs * r;
        t.ci = sn * r;
        t.in = amp * sqrtf(6 * (1 - r * r));
        t.hz = hz;
        t.stream = (uint8_t)g;
        t.wide = true;
        t.on = true;
    }
    thumpBand_.tune(300 * doppler_, 8);
}

// THE FIGHTER - the Eurofighter Typhoon's sound (Kind::LOW_BYPASS,
// Engine::EJ200), as heard from where the camera is: beside the runway, at
// the display line, under the climb, behind it in the chase.
//
// A fighter's roar shaped as noise alone was a gale, every time [O: four
// blind builds, 23 Oct]: the speaker plays nothing much under 300Hz, and
// steady noise in the band it does play is wind. What says "fighter" is in
// how the roar moves, measured on 18 real recordings [M: targets/eufi.json]:
//  - it churns, 2.5dB in 50ms as it passes, a third of that at 1-4Hz, all
//    its bands surging together - 0.67 slow, 0.36 fast - where an airliner's
//    move apart (0.23-0.37);
//  - its envelopes are the spikier the higher - kurtosis 5.7, 7.0, 8.4 and
//    15 at 0.5, 1, 2 and 4kHz - for it crackles: shocks, sharp compressions
//    and slow expansions, its pressure's rate of change skewed 1-3 at 16kHz
//    close by, coming thicker in its surges and thinning with distance;
//  - and in the speaker's band it is dark, its centre at 700-850Hz coming
//    and going alike - the big swing from bright to deep is under 300Hz.
// So it is made of three things, as the airliner's resonator design is,
// noise through filters, moved by a few shared random processes:
//  - its roar: the airliner's bank of eight bands, each as loud as the
//    jet's two noises - Tam's big eddies' and fine turbulence's spectra,
//    each with its own directivity and its own surges - make it;
//  - the big eddies' own sound under the speaker's band: a band of noise at
//    their frequency, 100-250Hz, never played, its envelope moving the
//    bank as it swells and drops - the roughness, the fast surging all the
//    bands share - and its compressions timing the shocks;
//  - the shocks: at the start of each of its compressions, now and then, a
//    sawtooth's edge - a fall into it, a jump, a slow decay - as often as
//    the jet's eddies run faster than sound in the air about them, the more
//    so in its surges and just after another, and fewer the further off.
// No recording in it; a table or two of figures.

namespace {

// The user's four open questions, each one figure, so an answer is a change
// of one line [O: the brief's defaults, 26 Oct; the user heard them and asked
// for a tad less crackle, Q2, the other three kept].
//  Q1, the intake's whine: how loud it idles, dB against the jet's own
//  level idling - heard ahead and to the side only, one line, under 3kHz as
//  heard and gone by full dry power; from behind 20dB and more under the
//  roar. In v1, 2dB under it with the airliner's lobe and a second line, it
//  stood 24-31dB over its neighbours, behind too [M: v1, the judges].
constexpr float TY_Q1_WHINE_DB = -35.0f;
//  Q2, the crackle: its shocks' strength, 1 as fitted - "sporadic", the
//  derivative's skewness 1-2 through the speaker's 180Hz high-pass over a
//  pass's loudest 2s, as the recordings' median. Heard, "a tad less" [O:
//  the user, 26 Oct]: 0.85, the shocks 1.4dB weaker - that skewness a
//  quarter less at the passes' loudest 2s and over them (the 80m pass's
//  peak 1.01 to 0.81, the take-off's 2.31 to 1.80), still crackling at
//  every reheat pass's peak; 0.8 took a third or more off [M: 7 airframes].
constexpr float TY_Q2_CRACKLE = 0.85f;
//  Q3, the low powers' level: idling's, dB under full dry power, the whole
//  law under it squeezed with it - as the airliner's distance is squeezed,
//  for a small speaker. The physics' -38 left the stand 31LU and the taxi
//  27LU under the airliner's take-off, all but silent [M: v1].
constexpr float TY_Q3_IDLE_DB = -14.5f;
//  Q4, the light-up's thump: how far over the rising roar it peaks, dB, for
//  its tenth of a second.
constexpr float TY_Q4_THUMP_DB = 3.0f;

// The figures fitted on the three measurement scenes (x-eufi-zeltweg,
// x-eufi-close-pass, x-eufi-display-100m) against the 18 recordings, six
// realisations of each scene, through the same chains as the recordings
// [M: tools/engine_preview/NOTES.md, renders/eufi-fit/metrics.json].
//  Its level, dB, against the airliner's LEVEL - its loudest held under the
//  soft limiter by the ride (TY_RIDE, TY_CEIL), so this sets the rest: v1's
//  -7 took its take-off to 2.2LU over the airliner's but rounded off 0.1-
//  0.28% of the 80m pass's samples in 3 airframes of 7, and 1.1% standing
//  lit 150m behind it [M: v1, the judges]; ridden, the loudest are as loud
//  as 0.1% lets them be in every airframe, and the low powers 2dB louder.
constexpr float TY_LEVEL_DB = -5.0f;
//  The fine turbulence's level against the big eddies', dB, and its peak,
//  octaves from theirs; the share of the Doppler the bank's spectrum moves
//  by - v1's 0.15 left a fly-past brighter going than coming, 0.22 octave;
//  half moves the 80m pass 0.28 octave, as the ILA passes' 0.29, and costs
//  none of the fitted figures, where the whole Doppler cost 11 [M]; the
//  250Hz band's power kept, the speaker playing next to none of it.
constexpr float TY_FST_DB = -1.0f, TY_FST_OCT = -0.6f, TY_KAPPA = 0.5f, TY_BAND0 = 0.25f;
//  The 4 and 6.3kHz bands' power kept, the shocks filling the rest of the
//  top; the share of the air's treble loss the jet's steepening puts back.
constexpr float TY_TOP = 0.3f, TY_NL = 0.2f;
//  The big eddies' Strouhal number; the low band's sharpness.
constexpr float TY_ST_L = 0.13f, TY_LOW_Q = 0.8f;
//  The surges: their quick and slow parts' time constants, s, the quick
//  part's weight, their spread, nepers; the angle, degrees, between the
//  low end's process and the treble's; each band's own shimmer, nepers.
constexpr float TY_SURGE_Q_S = 0.12f, TY_SURGE_S_S = 0.6f, TY_SURGE_QW = 0.6f, TY_SURGE_NP = 0.36f, TY_PHI = 70.0f;
constexpr float TY_SHIM_LO = 0.15f, TY_SHIM_HI = 0.15f;
//  The flutter's time constant, s, and spread, nepers; the low band's
//  envelope's smoothing, Hz, and how deep it moves the bank's two halves.
constexpr float TY_FLUT_S = 0.02f, TY_FLUT_NP = 0.10f, TY_ENV_HZ = 60.0f, TY_MU_LO = 0.12f, TY_MU_HI = 0.3f;
//  The shocks: their chance at a compression's start and the envelope it
//  must be over, as a share of its mean; their burstiness's spread,
//  nepers; the self-excitation each adds; their strengths' spread,
//  nepers; their strength against the roar's RMS; the distance their
//  wearing down halves them over, m; the eddies' Mach numbers their gate
//  opens over.
constexpr float TY_P0 = 0.35f, TY_THETA = 0.4f, TY_BURST_NP = 1.1f, TY_ALPHA = 0.35f, TY_LN_NP = 0.25f, TY_LOCK = 0.5f;
constexpr float TY_G_CR = 9.25f * TY_Q2_CRACKLE, TY_CR_FAR = 1500.0f, TY_CR_DB = 0.0f, TY_MC_ON = 0.8f, TY_MC_ALL = 1.5f;
//  The level the roar is ridden down to at its loudest, as samples go, and
//  the most it is let be.
constexpr float TY_RIDE = 0.2f, TY_CEIL = 0.27f;
//  The light-up's whoomp, dB against the jet's level - its noise through a
//  resonance an octave wide some 12dB under the roar in the speaker's band
//  at nought, so TY_Q4_THUMP_DB over it for its tenth of a second - and the
//  shocks' self-excitation it starts.
constexpr float TY_WHOOMP_DB = 12.0f + TY_Q4_THUMP_DB, TY_CATCH = 2.0f;
//  The spools' sound lagging their speed as it rises and falls, s: the
//  roar's swells and falls with the power over the seconds a fan and its
//  jet take, not as a switch [I: eufi-sources 11.7, idle -15 to -20dB over
//  3-5s; 11.9, 0 to -15dB over 3-8s - where the airliner's spool alone fell
//  42dB in 0.6s, the law steep at the top: M, v1].
constexpr float TY_SPOOL_UP_S = 0.3f, TY_SPOOL_DOWN_S = 2.0f;
//  The tyres' squeal touching down against the airliner's: two small main
//  wheels, and at the roar's level then [I: eufi-sources 11.9, 0dB; the
//  airliner's half stood 23-30dB over it: M, v1].
constexpr float TY_SQUEAL = 0.04f;

// Its jet's roar against its fan's speed, dB against its full dry power's
// (MIL) - as an installed F404's sound power, the EJ200's size, goes with
// thrust: -35dB at 17% thrust, -22 at 36%, -8 at 58%, the jet going
// supersonic between, then little more [M: Christian et al. 2023, JASA-EL
// 3, 073601]; the EJ200's thrust from its fan speed as the cube and a half
// [I: eufi-sources 2.2]. Idling, some -38 [I: the F404's power extrapolated
// to idle, 120dB re 1pW an engine against 168-170, and its core's rumble and
// the nozzle's hiss with it]; the approach at -20, between
// the F404's law (-32 at 20% thrust) and the German noise model's landing
// class, 18dB under its take-off [P: AzB 2007, S-MIL]. Reheat adds 4dB more,
// most behind it - see the lobes. That law, -38, -30, -20, -13, -6 and 0dB,
// left every low power all but silent on the speaker beside the airliner, so
// it is squeezed - idling at TY_Q3_IDLE_DB, the rest in proportion, as a
// share of that: the chase's cruise to idle 15dB, its cruise to full dry
// some 6, the approach's touchdown to idle 13 [I: eufi-sources 11.7, 11.9].
constexpr float JET_N1[6] = {0.30f, 0.52f, 0.75f, 0.85f, 0.93f, 1.0f};
constexpr float JET_SQ[6] = {1.0f, 0.78f, 0.42f, 0.344f, 0.156f, 0.0f};

// Tam's two similarity spectra, which every supersonic jet's mixing noise is
// a sum of [P: Tam, Golebiowski & Seiner, AIAA 96-1716]: F, the big eddies'
// - beamed aft, a sharp peak, falling 5.4dB an octave above it - and G, the
// fine turbulence's - to the side and ahead, broad. As levels in
// proportional bands, dB under each one's highest, every third of an octave
// from four octaves under its peak frequency to seven over.
constexpr float TAM_F[34] = {-32.3f, -29.5f, -26.6f, -23.8f, -20.9f, -18.1f, -15.2f, -12.4f, -9.5f,  -6.7f,  -4.1f,
                             -2.0f,  -0.7f,  0.0f,   -0.1f,  -0.9f,  -2.2f,  -3.9f,  -5.7f,  -7.5f,  -9.3f,  -11.1f,
                             -12.8f, -14.6f, -16.4f, -18.2f, -20.0f, -21.7f, -23.5f, -25.3f, -27.1f, -28.9f, -30.7f,
                             -32.4f};
constexpr float TAM_G[34] = {-22.8f, -20.6f, -18.3f, -16.0f, -13.8f, -11.8f, -10.0f, -8.4f,  -7.0f,  -5.7f,  -4.6f,
                             -3.5f,  -2.5f,  -1.6f,  -0.9f,  -0.3f,  -0.0f,  0.0f,   -0.2f,  -0.8f,  -1.6f,  -2.8f,
                             -4.4f,  -6.1f,  -8.0f,  -10.1f, -12.4f, -15.1f, -17.9f, -20.8f, -23.6f, -26.4f, -29.3f,
                             -32.1f};
float tam(const float *t, float octaves) {
    float x = clampf((octaves + 4) * 3, 0, 32.999f);
    int i = (int)x;
    return t[i] + (t[i + 1] - t[i]) * (x - i);
}

//                             0     20    40    60    80   100   120   140   160   180
// The big eddies' roar: most at the Mach waves' angle off the nose - 130
// to 140 degrees dry, 115 to 125 lit, its jet the faster [I: eufi-sources
// 1.3; P: the F-22A's 120-140, the F404's 150 at low power to 120 in
// afterburner] - 20dB down ahead, and a little down right behind, in the
// jet's cone of silence: flatter than the research's 30dB, which left the
// passes 23dB under their peak two seconds before it where the recordings
// are 9-12dB under [M].
constexpr Lobe LTS_DRY = {-20.0f, -19.0f, -17.0f, -14.0f, -10.0f, -6.0f, -2.0f, 0.0f, -0.5f, -2.5f};
constexpr Lobe LTS_WET = {-19.0f, -18.0f, -16.0f, -12.0f, -7.0f, -2.5f, 0.0f, -0.5f, -1.0f, -3.0f};
// The fine turbulence's: broad, most to the side, 4dB down ahead - the front
// of a pass, its in-band spectrum the same as behind [M: no forward hump in
// 28 passes, so no shock-associated hump here].
constexpr Lobe FST = {-4.0f, -3.5f, -3.0f, -2.0f, -1.0f, 0.0f, 0.0f, -2.0f, -4.0f, -6.0f};
// The shocks', against the roar they ride on: most about the Mach angle,
// and still there, weaker, coming [M: the passes' derivative skewness, through
// the speaker's 180Hz high-pass, 0.3-1.0 coming, 0.8-1.0 at the peak, 0.5
// going - not the 20-30dB swing of their source alone]; 3dB weaker ahead
// than v1's, which crackled 1.1 coming at the 80m pass where the ILA passes
// give 0.43 [M: the judges], and down in the jet's cone of silence.
// The intake's whine: out of its chin intake, ahead and to the side, and
// gone behind - 20dB down at 140 degrees, 40 at the tail, the jet and the
// airframe between [I: eufi-sources 11.1, 11.7, 11.8b: -10 to -20 behind].
constexpr Lobe INTAKE = {0.0f, 0.0f, -1.0f, -3.0f, -6.0f, -12.0f, -20.0f, -28.0f, -34.0f, -40.0f};
constexpr Lobe CRACKLE_DRY = {-9.0f, -8.5f, -8.0f, -7.0f, -5.0f, -3.0f, -1.0f, 0.0f, -4.0f, -12.0f};
constexpr Lobe CRACKLE_WET = {-8.0f, -7.5f, -7.0f, -6.0f, -4.0f, -1.5f, 0.0f, -1.5f, -5.0f, -12.0f};

// A shock's shape, as the real ones average at 16kHz [M: r69878's loud
// passes, eufi-methods 1.2]: a slow fall into it, the rarefaction before
// it, over fourteen samples; a jump, as steep as a sample or two lets it
// be; and a decay to half in five - a steepened wave's sawtooth; the fall
// before it half as deep as would leave it no area, the rest of its
// expansion - slower, longer - the speaker's 180Hz high-pass's to give it;
// through two one-pole low-passes at 5kHz, critically damped, so nothing in
// it aliases and nothing rings after the jump; its power one.
constexpr int SHOCK_N = 32;
constexpr float SHOCK[SHOCK_N] = {-0.01081f, -0.02465f, -0.03913f, -0.05372f, -0.06834f, -0.08297f, -0.09759f, -0.11221f,
                                  -0.12684f, -0.14146f, -0.15609f, -0.17071f, -0.18533f, -0.19996f, 0.33965f,  0.42624f,
                                  0.38556f,  0.32712f,  0.27373f,  0.22839f,  0.19045f,  0.15879f,  0.13239f,  0.11038f,
                                  0.09203f,  0.07673f,  0.06398f,  0.05334f,  0.04447f,  0.03708f,  0.03091f,  0.02577f};

}  // namespace

#ifdef ENGINE_NOISE_DEBUG
static int g_fired = 0;  // shocks fired, for the debug line
#endif

// Another fighter starting: as it is now, its reheat as asked, both engines
// already turning if they run.
void Synth::fighterRestart() {
    const Sound &s = heard_;
    Jet &j = jet_;
    j = Jet();
    j.aspect = clamp01(s.aspect);
    j.closing = std::max(-SOUND_MS, std::min(0.8f * SOUND_MS, s.closingMs));
    j.doppler = SOUND_MS / (SOUND_MS - j.closing);
    j.convection = convectionTo_;
    float n1 = 0.5f * (n1_[0] + n1_[1]);
    j.reheat = j.reheatWas = std::max(clamp01(s.reheat), s.afterburner ? 1.0f : 0.0f) * smoothstep(0.93f, 0.97f, n1);
    j.spool2 = j.lit2 = s.running ? 1.0f : 0.0f;
    j.runS = s.running ? 60.0f : 0.0f;
    j.lowHz = -1;
}

// Each 2ms: where it is heard from, quickly; its engines and reheat; its
// jet; the low band's tuning and the shocks' chance.
__attribute__((noinline)) void Synth::fighterStep() {
    const Sound &s = sound_;
    Jet &j = jet_;
    const EngineData &e = ENGINES[(int)s.engine];
    // Which way it faces and how fast it closes, eased in 35ms: at 200m/s
    // past 50m the way the sound comes turns 230 degrees a second, and the
    // airliner's 0.2s would lag it 45 degrees and smear the roar's arrival
    // [I: eufi-methods 7.2]. Placed, the way is already smooth. Coming at up
    // to 0.8 of the speed of sound - a factor of 5 - not the airliner's 0.7.
    static const float QUICK = 1 - expf(-STEP_S / 0.035f), REHEAT_OUT = 1 - expf(-STEP_S / 0.25f),
                       LIGHT = 1 - expFast(-STEP_S / 0.8f), OUT = 1 - expf(-STEP_S / 0.3f),
                       WIND_DOWN = expf(-STEP_S / 12), MEAN = 1 - expf(-STEP_S / 1.0f),
                       BURST = expf(-STEP_S / 0.05f), HAWKES = expf(-STEP_S / 0.03f);
    j.aspect = ease(j.aspect, clamp01(s.aspect), QUICK);
    float closing = s.placed ? std::max(-SOUND_MS, std::min(0.8f * SOUND_MS, s.closingMs))
                             : std::max(-150.0f, std::min(150.0f, s.closingMs));
    j.closing = ease(j.closing, closing, QUICK);
    j.doppler = SOUND_MS / (SOUND_MS - j.closing);
    if (s.placed) {
        j.convection = ease(j.convection, convectionTo_, QUICK);
    }
    // The second engine started half a minute after the first, as the
    // first's: motored to a third of idle, lit, on up to idle.
    if (s.running) {
        j.runS += STEP_S;
    } else {
        j.runS = 0;
    }
    if (s.running && j.runS >= 30) {
        j.spool2 = std::min(1.0f, j.spool2 + (j.spool2 < 0.35f ? 0.35f / e.motorS : 0.65f / 14) * STEP_S);
    } else if (!s.running) {
        j.spool2 = j.spool2 < 0.002f ? 0 : j.spool2 * WIND_DOWN;
    }
    bool burning = s.running && j.spool2 >= 0.35f;
    j.lit2 = ease(j.lit2, burning ? 1.0f : 0.0f, burning ? LIGHT : OUT);
    // Reheat: lit only at full dry power, over some 0.6s - the nozzle
    // opening, the fire catching - as a second-order rise with no overshoot
    // [M: a light-up's level 10-90% in 0.72s, Farnborough]; cancelled
    // quicker, in a quarter of a second. Whatever `reverse` says: an EJ200
    // has none.
    float n1 = 0.5f * (n1_[0] + n1_[1]);
    float want = std::max(clamp01(s.reheat), s.afterburner ? 1.0f : 0.0f) * smoothstep(0.93f, 0.97f, n1) * lit_;
    if (want > j.reheat) {
        const float w = 5.5f;
        j.reheatRate += (w * w * (want - j.reheat) - 2 * w * j.reheatRate) * STEP_S;
        j.reheat = std::min(want, j.reheat + std::max(0.0f, j.reheatRate) * STEP_S);
    } else {
        j.reheat = ease(j.reheat, want, REHEAT_OUT);
        j.reheatRate = 0;
    }
    // The light-up, as it catches: a whoomp - the fire's pressure pulse,
    // under the speaker's band itself, heard as a burst of noise at 320Hz
    // as the Doppler has it, dying in a tenth of a second - and the shocks
    // catching a sixth of a second on, a cluster of them [I: eufi-methods
    // 6.2; M: a light-up's level steepest at 98dB/s over 100ms].
    if (j.reheat > 0.02f && j.reheatWas <= 0.02f) {
        j.lightS = 0;
        float sn, cs, hz = std::min(320 * j.doppler, 1500.0f);
        sinCosTurns(hz / RATE, sn, cs);
        float alpha = sn / (2 * 1.4f), a0 = 1 + alpha;
        j.whoompB0 = alpha / a0;
        j.whoompA1 = -2 * cs / a0;
        j.whoompA2 = (1 - alpha) / a0;
    }
    if (j.lightS < 0.15f && j.lightS + STEP_S >= 0.15f) {
        j.hawkesAdded += TY_CATCH;
    }
    j.reheatWas = j.reheat;
    j.lightS += STEP_S;
    // Its jet's speed [I: eufi-sources 1.3, 2.2]: 200m/s idling, 770 at full
    // dry power, half as fast again lit; and its big eddies' speed against
    // the air about it, 0.7 of the jet's less the aircraft's own, as a Mach
    // number - over one, Mach waves and shocks: 1.6 at full dry power on the
    // ground, 2.3 lit; a fast low pass at part power, the approach and the
    // cruise under one.
    float x = clamp01((n1 - 0.30f) / 0.70f);
    j.jetMs = (200 + 570 * x * x) * (1 + 0.44f * j.reheat);
    j.mc = 0.7f * std::max(0.0f, j.jetMs - std::min(airspeed_, 400.0f)) / SOUND_MS;
    // The low band at the big eddies' frequency, a Strouhal number of 0.13 on
    // the jet's speed and width - 200Hz at full dry, 230 lit [I] - as heard,
    // the whole Doppler on it: its rhythm is the shocks'. Tuned again as it
    // moves a percent, as loud at its centre as what goes in.
    float dj = 0.49f + 0.14f * j.reheat;
    float lowHz = clampf(TY_ST_L * j.jetMs / dj * j.doppler, 40, 1500);
    if (fabsf(lowHz - j.lowHz) > 0.01f * j.lowHz) {
        static const float Q = TY_LOW_Q;
        float sn, cs, s2, c2;
        sinCosTurns(lowHz / RATE, sn, cs);
        float alpha = sn / (2 * Q), a0 = 1 + alpha, a1 = -2 * cs / a0, a2 = (1 - alpha) / a0;
        sinCosTurns(2 * lowHz / RATE, s2, c2);
        float re = 1 + a1 * cs + a2 * c2, im = a1 * sn + a2 * s2;
        // the resonance's gain at the centre taken out of the band-pass's, and
        // its width's: the noise through it as strong wherever it is tuned, so
        // its envelope against its mean doesn't jump as the Doppler moves it -
        // at a cut from going to coming, or past the peak of a pass
        j.lowB0 = alpha / a0 * sqrtf(re * re + im * im) * sqrtf(100 / lowHz);
        j.lowA1 = a1;
        j.lowA2 = a2;
        if (j.lowHz < 0) {
            // its mean envelope as noise through it gives - a third's power, over
            // some 0.85 of its width - till its own mean takes over
            j.envMean = 0.8f * sqrtf((1.0f / 3) * 0.85f * 100 / (Q * 0.5f * RATE));
        }
        j.lowHz = lowHz;
    }
    j.envMean += (j.envSum * (1.0f / STEP) - j.envMean) * MEAN;
    j.envSum = 0;
    // The shocks' chance at each compression's start: as the eddies run
    // faster than sound - none under Mach 1, all over 1.9 - the more in the
    // big eddies' surges (as the root of them) and in a 50ms burstiness of
    // their own, and after each shock for 30ms as much again for each one -
    // a self-exciting process, clustering them [M: their counts in 50ms
    // windows 1.5-5 times as varied as a steady rate's; P: Ffowcs Williams
    // 1975, groups of ten in a tenth of a second].
    Rng rng{slow_};
    j.burst = j.burst * BURST + sqrtf(1 - BURST * BURST) * rng.gauss();
    // The flutter: the whole roar's level, quickly, a twentieth of a second
    // at a time - the passes' bands moving together at 4-50Hz [M: 0.36] -
    // got to across the step.
    static const float FLUTTER = expf(-STEP_S / TY_FLUT_S), FLUT_NP = TY_FLUT_NP;
    j.flutter = j.flutter * FLUTTER + sqrtf(1 - FLUTTER * FLUTTER) * rng.gauss();
    j.flutterWas = j.flutterTo;
    j.flutterTo = exp2Fast(1.44269504f * FLUT_NP * (j.flutter - 0.5f * FLUT_NP));
    slow_ = rng.s;
    j.hawkes = j.hawkes * HAWKES + j.hawkesAdded;
    j.hawkesAdded = 0;
    static const float P0 = TY_P0, BURST_NP = TY_BURST_NP, MC_ON = TY_MC_ON,
                       MC_ALL = TY_MC_ALL;
    float gate = smoothstep(MC_ON, MC_ALL, j.mc) * lit_;
    j.shockUnit = P0 * gate * powFast(j.ltsSurge, TY_LOCK) * exp2Fast(1.44269504f * BURST_NP * (j.burst - 0.5f * BURST_NP));
    j.shockP = j.shockUnit * (1 + j.hawkes);
    // Its intake's whine, the fan's blades passing - 24 blades assumed, none
    // published [I: eufi-sources 1.2] - as heard, the Doppler on it, turned on
    // each step: noise through a resonance 3Hz wide near, as the airliner's
    // tones are; as loud as its slow step makes it. One line, not two - the
    // pair a steady two-tone drone [O: the user]; and none heard over 3kHz,
    // the dentist's drill the user heard in 3-4.8kHz [O: 23 Oct] - so the
    // approach's whine, at 3.8kHz and more, is gone.
    float spun = n1 * (spool_ < 1 ? powFast(std::max(spool_, 1e-3f), 0.7f) : 1.0f);
    {
        Tone &t = tones_[FAN_1];
        float heard = e.blades * e.shaft100 * spun * j.doppler;
        if (j.whine < 1e-7f || heard < 30 || heard > 3200) {
            t.in = 0;
            t.on = t.on && (t.zr * t.zr + t.zi * t.zi > 1e-14f);
        } else {
            float x = PI * (3.0f + 3.5f * expFast(logDistance_) * 0.001f) / RATE,
                  r = 1 - x * (1 - x * (0.5f - x * (1.0f / 6)));
            float sn, cs;
            sinCosTurns(heard / RATE, sn, cs);
            t.cr = cs * r;
            t.ci = sn * r;
            float amp = j.whine * (1 - smoothstep(2400, 3200, heard));
            t.in = amp * sqrtf(6 * (1 - r * r));
            t.stream = 1;
            t.on = true;
        }
    }
}

// Each 32ms: the loudness as it spreads, its roar's surges and its bands'
// sums, the shocks' strength.
__attribute__((noinline)) void Synth::fighterSlowStep() {
    const Sound &s = sound_;
    Jet &j = jet_;
    // Its distance as it is, not eased over the airliner's 0.12s: its way's
    // other figures move in 35ms, and at a cut from a near camera hearing it
    // go to a far one hearing it come, the near one's loudness carried over
    // into the coming roar's was a thump of 10dB for a tenth of a second [M].
    float d = std::max(5.0f, s.distanceM);
    // The air takes less of its treble than of a quiet sound's: at full dry
    // power and lit its waves steepen as they go, putting back high notes
    // the air takes [P: Gee et al. 2008, near an F-22A far more high-
    // frequency energy arriving than linear propagation gives; M: the passes
    // 300m off keep their 4kHz octave 19dB under the band, where the air's
    // loss alone left it 22] - as if the way were shorter, by up to TY_NL.
    float steep = smoothstep(TY_MC_ON, TY_MC_ALL, j.mc), km = d * 0.001f * (1 - TY_NL * steep);
    air_.tune(mufflingHz(km), 0.6f);
    // Its own convective gain, with its quicker way: louder coming, by no
    // more than 2.5 times, quieter going, by no more than half - in its
    // sources' own levels, not the step's, so that at a camera's cut from
    // going to coming it moves with the roar's lobes over the next 32ms, not
    // at once while they still are going's: v1's cuts thumped 10-15dB for
    // 30ms where the airliner's stay within 1.1 [M: the judges].
    gain_ = powFast(35 / std::max(35.0f, d) * dbAmp(-heldDb_), 0.65f);
    float conv = s.placed ? std::max(0.5f, std::min(2.5f, powFast(j.convection, 0.65f))) : 1.0f;
    // The roar's surges. Its sound comes from all along the jet, its low
    // notes from further down it than its high ones, and each stretch
    // bursts in its own time [P: Olaveson & Gee 2024, the F404's lobes
    // "occur intermittently and at different times from each other"]: so
    // two processes, the low end's and the treble's, each a quick part and
    // a slow part - noise smoothed over a tenth of a second and over half of
    // one - and each band moved by the two as much as it is low or high, so
    // the treble's octaves surge as one and the 500Hz octave only partly with
    // them [M: the passes' octaves 0.55-0.7 together slowly with 500Hz, 0.7-
    // 0.88 among 1-4kHz; their churn's correlation time 120-140ms, a third of
    // it at 1-4Hz]; a fifth more lit. Each band's own shimmer too.
    static const float SURGE_Q_S = TY_SURGE_Q_S, SURGE_S_S = TY_SURGE_S_S,
                       SURGE_QW = TY_SURGE_QW, SURGE_NP = TY_SURGE_NP,
                       SHIM_LO = TY_SHIM_LO, SHIM_HI = TY_SHIM_HI;
    static const float SQ = expf(-SLOW_S / SURGE_Q_S), SS = expf(-SLOW_S / SURGE_S_S);
    // each band's share of the low end's and the treble's: as the angle
    // between them, nought up to 400Hz, PHI degrees from 3kHz up
    static float lowW[BANDS], highW[BANDS];
    static bool weighed = false;
    if (!weighed) {
        float most = TY_PHI / 360;
        for (int b = 0; b < BANDS; b++) {
            float sn, cs;
            sinCosTurns(most * smoothstep(0, 2.907f, log2f(BAND_HZ[b] / 400)), sn, cs);
            lowW[b] = cs;
            highW[b] = sn;
        }
        weighed = true;
    }
    Rng rng{slow_};
    j.ltsQuick = j.ltsQuick * SQ + sqrtf(1 - SQ * SQ) * rng.gauss();
    j.ltsSlow = j.ltsSlow * SS + sqrtf(1 - SS * SS) * rng.gauss();
    j.fstQuick = j.fstQuick * SQ + sqrtf(1 - SQ * SQ) * rng.gauss();
    j.fstSlow = j.fstSlow * SS + sqrtf(1 - SS * SS) * rng.gauss();
    float sg = SURGE_NP * (1 + 0.2f * j.reheat), qw = SURGE_QW, sw = sqrtf(1 - SURGE_QW * SURGE_QW);
    float nLow = qw * j.ltsQuick + sw * j.ltsSlow, nHigh = qw * j.fstQuick + sw * j.fstSlow;
    j.ltsSurge = exp2Fast(1.44269504f * sg * (0.7071f * (nLow + nHigh) - 0.7071f * sg));  // the shocks' share
    for (int b = 0; b < BANDS; b++) {
        shimmerQuick_[b] = shimmerQuick_[b] * shimmerKeep_[0] + shimmerKeep_[1] * rng.gauss();
        shimmerSlow_[b] = shimmerSlow_[b] * shimmerKeep_[2] + shimmerKeep_[3] * rng.gauss();
        shimmer_[b] = SHIMMER_QW * shimmerQuick_[b] + sqrtf(1 - SHIMMER_QW * SHIMMER_QW) * shimmerSlow_[b];
    }
    // The air's flicker, as the airliner's.
    float kf = expFast(-SLOW_S * (across_ * FLICKER_ACROSS + FLICKER_BASE) * (1.0f / 5));
    flickerA_ = flickerA_ * kf + sqrtf(1 - kf * kf) * rng.gauss();
    flicker_ = flicker_ * kf + (1 - kf) * flickerA_;
    float flickerN = flickerN_ = flicker_ * (1 + kf) * (1 / sqrtf(1 + kf * kf));
    float flickerNp = std::min(6.0f, FLICKER_DB * sqrtf(km)) * (1 / 8.686f);
    float cornerLog = log2Fast(9 / sqrtf(d));
    slow_ = rng.s;
    // Its jet's level: each engine's by its fan's speed - its spool, starting
    // - and its fire, the two's powers summed; quieter as it flies, its
    // jet's speed against the air the less [P: the flight effect, by no more
    // than 10dB]; reheat's 4dB. Starting, before the fire: the air its
    // starter drives through it, 18dB under idling as it is lit - the light-
    // off a step of 8dB over that, then on up to idle with the spool [I:
    // eufi-sources 11.1, +5 to +10dB over 2-3s], none of it silence. The two
    // engines' sound lagging their spools' speed, rising and falling.
    float n1 = std::max(0.0f, 0.5f * (n1_[0] + n1_[1]));
    float power = 0;
    static const float IDLE = TY_Q3_IDLE_DB, MOTOR = dbPower(TY_Q3_IDLE_DB - 18);
    for (int g = 0; g < 2; g++) {
        float sp = g == 0 ? spool_ : j.spool2, fire = g == 0 ? lit_ : j.lit2;
        float n = n1 * (sp < 1 ? powFast(sp, 1.6f) : 1.0f), db = 0;
        if (n <= JET_N1[0]) {
            db = IDLE - 12 * (1 - n / JET_N1[0]);  // spooling up to idle
        } else {
            for (int i = 1; i < 6; i++) {
                if (n <= JET_N1[i]) {
                    db = IDLE * (JET_SQ[i - 1] + (JET_SQ[i] - JET_SQ[i - 1]) * (n - JET_N1[i - 1]) /
                                                     (JET_N1[i] - JET_N1[i - 1]));
                    break;
                }
            }
        }
        power += 0.5f * (dbPower(db) * fire * fire + MOTOR * sqrtf(std::min(1.0f, sp * (1 / 0.35f))));
    }
    static const float LEVEL_DB = TY_LEVEL_DB, FST_DB = TY_FST_DB, KAPPA = TY_KAPPA;
    static const float SPOOL_UP = 1 - expf(-SLOW_S / TY_SPOOL_UP_S), SPOOL_DOWN = 1 - expf(-SLOW_S / TY_SPOOL_DOWN_S);
    float dryDb = 10 * lg10(std::max(1e-9f, power));
    j.dryDb = j.dryDb < -900 ? dryDb : j.dryDb + (dryDb - j.dryDb) * (dryDb > j.dryDb ? SPOOL_UP : SPOOL_DOWN);
    float v = std::min(airspeed_, 400.0f);
    float flight = std::max(-10.0f, 50 * lg10(std::max(0.01f, 1 - v / std::max(60.0f, j.jetMs))));
    float jetDb = LEVEL_DB + j.dryDb + flight + 4 * j.reheat;
    float th = j.aspect, r = j.reheat;
    // The big eddies' beaming as deep as its jet is fast against the air: an
    // idling jet's roar much the same all round - 5dB down ahead, not 20 [I:
    // eufi-sources 11.1, 11.2].
    float deep = 0.35f + 0.65f * smoothstep(0.4f, 1.2f, j.mc);
    float ltsLobe = (lobe(LTS_DRY, th) + (lobe(LTS_WET, th) - lobe(LTS_DRY, th)) * r) * deep;
    float ltsDb = jetDb + ltsLobe, fstDb = jetDb + FST_DB + lobe(FST, th);
    // Its spectra's peaks, as it gives them out: the big eddies' at the low
    // band's Strouhal number, the fine turbulence's 1.8 times higher; heard
    // half the Doppler up or down - for in the speaker's band, the passes'
    // spectra move less than it [M: centroid 844Hz coming, 710 going, where
    // the whole Doppler of a display pass would move it more than an octave]:
    // the rest of the swing from bright to deep is the roar's below 300Hz.
    float dj = 0.49f + 0.14f * r;
    float peak = log2Fast(std::max(20.0f, TY_ST_L * j.jetMs / dj) * 0.001f);
    float shift = KAPPA * log2Fast(j.doppler);
    float airSin, airCos;
    sinCosTurns(0.5f * air_.hz / RATE, airSin, airCos);
    float airCot = airCos / std::max(1e-6f, airSin);
    static float bandTanAt[BANDS][3];  // tan(pi f / rate) half an octave under each band's centre, at it, over it
    static bool tangents = false;
    if (!tangents) {
        for (int b = 0; b < BANDS; b++) {
            for (int k = 0; k < 3; k++) {
                float sn, cs;
                sinCosTurns(0.5f * std::min(BAND_HZ[b] * exp2Fast(0.5f * (k - 1)), 0.47f * RATE) / RATE, sn, cs);
                bandTanAt[b][k] = sn / cs;
            }
        }
        tangents = true;
    }
    // The airbrake open on the landing roll, a low buffet as the square of the
    // speed - the airliner's spoilers' as a share of their ten panels - under
    // the approach's roar as it touches down, over the idling engines' till it
    // slows [I: eufi-sources 11.9, aerobraking 15dB under the loudest].
    float vr = std::max(0.05f, v / 70);
    float buffetDb = spoilers_ > 0.001f ? LEVEL_DB - 28 + 40 * lg10(vr) + 20 * lg10(spoilers_) : -200;
    // Its intake's whine, ahead of it, at low power only - from the starter's
    // first turn, idling, taxiing - TY_Q1_WHINE_DB against the jet's level
    // idling [I: eufi-sources 3.5, 11.1: no recording of one]; none at
    // display power, where no recording has one [M: the strongest line in any
    // 0.5s of 29 passes no more than noise's].
    float spun = n1 * (spool_ < 1 ? powFast(std::max(spool_, 1e-3f), 0.7f) : 1.0f);
    j.whine = spun > 0.01f ? dbAmp(LEVEL_DB + TY_Q3_IDLE_DB + TY_Q1_WHINE_DB + 30 * lg10(std::max(0.1f, spun / 0.3f)) +
                                   lobe(INTAKE, th)) *
                                 (1 - smoothstep(0.75f, 0.9f, spun)) * conv
                           : 0.0f;
    float roar = 0;
    for (int b = 0; b < BANDS; b++) {
        float lf = bandLog_[b] - shift;
        float pL = dbPower(ltsDb + tam(TAM_F, lf - peak)), pF = dbPower(fstDb + tam(TAM_G, lf - peak - TY_FST_OCT));
        float surge = exp2Fast(2.88539008f * sg * (lowW[b] * nLow + highW[b] * nHigh - sg));  // e^(2sn - 2s^2): mean one
        float p = (pL + pF) * surge * (b == 0 ? TY_BAND0 : b >= 6 ? TY_TOP : 1.0f);
        if (buffetDb > -150) {
            float xb = lf + 1.59946f;  // octaves from 330Hz
            p += dbPower(buffetDb - std::min(25.0f, 4 * xb * xb));
        }
        float fl = flickerNp * smoothstep(cornerLog - 0.5f, cornerLog + 0.5f, bandLog_[b]);
        float sh = b < 4 ? SHIM_LO : SHIM_HI;
        // The air's loss made exact over the band, as the airliner's is at
        // its centre - but over its half-octaves either side too: the band is
        // broad and the muffling steep, and from a kilometre the band's centre
        // alone left its upper half, and the sound, 3-5dB too weak.
        float through = 0;
        for (int k = 0; k < 3; k++) {
            float w = bandTanAt[b][k] * airCot, w2 = w * w;
            through += 1 / ((1 - w2) * (1 - w2) + w2 * (1 / (0.6f * 0.6f)));
        }
        float lowPassDb = -4.34294482f * 0.693147181f * log2Fast(through * (1.0f / 3));
        float airDb = clampf(lowPassDb - bandAirDbKm_[b] * km, -30, 15);
        float ln = 2 * sh * (shimmer_[b] - sh) + 2 * fl * (flickerN - fl) + 0.230258509f * airDb;
        p *= exp2Fast(1.44269504f * ln) * (conv * conv);
        roar += (pL + pF) * surge;
        bandBy_[b] = (sqrtf(p) - bandGain_[b]) * (1.0f / (SLOW * STEP));
    }
    // The whoomp as loud as the jet lit, a little more ahead and to the side.
    j.whoomp = dbAmp(jetDb + TY_WHOOMP_DB + 3 * (1 - th)) * conv;
    // The shocks as strong, against the roar they ride on, as the crackle's
    // own directivity makes them; formed as the wave steepens over the
    // first hundred metres or two - two thirds as strong 80m off as 300m -
    // and worn down further off, the air's muffling taking their edges after
    // them [P: Gee et al. 2013, the derivative skewness near an F-35A growing
    // with distance; M: the passes' crackle much the same 80-300m off, 1.1-1.2
    // through the 180Hz high-pass, half that 8-16dB quieter, none far off].
    static const float G_CR = TY_G_CR, CR_FAR = TY_CR_FAR, CR_DB = TY_CR_DB;
    float crLobe = lobe(CRACKLE_DRY, th) + (lobe(CRACKLE_WET, th) - lobe(CRACKLE_DRY, th)) * r;
    float far = (0.6f + 0.4f * smoothstep(20, 200, d)) / (1 + d / CR_FAR);
    float gate = sqrtf(smoothstep(TY_MC_ON, TY_MC_ALL, j.mc));
    j.shockAmp = G_CR * sqrtf(roar) * dbAmp(crLobe + CR_DB) * far * gate * conv;
#ifdef ENGINE_NOISE_DEBUG
    static int debugIn = 0;
    if (--debugIn < 0) {
        debugIn = 31;
        fprintf(stderr, "t %.2f d %.0f asp %.2f n1 %.3f rh %.2f mc %.2f dop %.2f | jet %.1f lts %.1f fst %.1f mu %.2f "
                        "shockP %.3f amp %.4f low %.0fHz env %.4f fired %d gain %.3f held %.1f conv %.2f over %.5f ceil %.2f ride %.2f\n",
                clock_ / (float)RATE, d, th, n1, r, j.mc, j.doppler, jetDb, ltsDb, fstDb, TY_MU_LO, j.shockP, j.shockAmp,
                j.lowHz, j.envMean, g_fired, gain_, heldDb_, j.convection, j.overShare, j.ceilBy, j.ride);
        g_fired = 0;
    }
#endif
}

// The step's samples: the bank, its two halves apart; the low band, its
// envelope moving them; the shocks at its compressions.
__attribute__((noinline)) void Synth::fighterRender(float *mix, const float *lateA, const float *lateB,
                                                    const float *noiseA, const float *noiseB, Rng &rng, int n) {
    Jet &j = jet_;
    float half[2][STEP];
    // The bank as the airliner's, four bands at a time: 250Hz-1kHz, where
    // the big eddies' roar is, and 1.6-6.3kHz, summed apart.
    for (int b = 0; b < BANDS; b += 4) {
        float *out = half[b >> 2];
        const float *B0 = bandB0_ + b, *A1 = bandA1_ + b, *A2 = bandA2_ + b;
        float b0 = B0[0], b1 = B0[1], b2 = B0[2], b3 = B0[3];
        float p0 = A1[0], p1 = A1[1], p2 = A1[2], p3 = A1[3];
        float q0 = A2[0], q1 = A2[1], q2 = A2[2], q3 = A2[3];
        float y0 = bandY1_[b], y1 = bandY1_[b + 1], y2 = bandY1_[b + 2], y3 = bandY1_[b + 3];
        float z0 = bandY2_[b], z1 = bandY2_[b + 1], z2 = bandY2_[b + 2], z3 = bandY2_[b + 3];
        float v0 = bandZ1_[b], v1 = bandZ1_[b + 1], v2 = bandZ1_[b + 2], v3 = bandZ1_[b + 3];
        float w0 = bandZ2_[b], w1 = bandZ2_[b + 1], w2 = bandZ2_[b + 2], w3 = bandZ2_[b + 3];
        float g0 = bandGain_[b], g1 = bandGain_[b + 1], g2 = bandGain_[b + 2], g3 = bandGain_[b + 3];
        float d0 = bandBy_[b], d1 = bandBy_[b + 1], d2 = bandBy_[b + 2], d3 = bandBy_[b + 3];
        for (int i = 0; i < n; i++) {
            float x = lateA[i], u = lateB[i];
            float u0 = (b0 * x - q0 * z0) - p0 * y0, u1 = (b1 * u - q1 * z1) - p1 * y1;
            float u2 = (b2 * x - q2 * z2) - p2 * y2, u3 = (b3 * u - q3 * z3) - p3 * y3;
            z0 = y0, z1 = y1, z2 = y2, z3 = y3;
            y0 = u0, y1 = u1, y2 = u2, y3 = u3;
            float r0 = (u0 - q0 * w0) - p0 * v0, r1 = (u1 - q1 * w1) - p1 * v1;
            float r2 = (u2 - q2 * w2) - p2 * v2, r3 = (u3 - q3 * w3) - p3 * v3;
            w0 = v0, w1 = v1, w2 = v2, w3 = v3;
            v0 = r0, v1 = r1, v2 = r2, v3 = r3;
            g0 += d0, g1 += d1, g2 += d2, g3 += d3;
            out[i] = (g0 * r0 + g1 * r1) + (g2 * r2 + g3 * r3);
        }
        bandY1_[b] = y0, bandY1_[b + 1] = y1, bandY1_[b + 2] = y2, bandY1_[b + 3] = y3;
        bandY2_[b] = z0, bandY2_[b + 1] = z1, bandY2_[b + 2] = z2, bandY2_[b + 3] = z3;
        bandZ1_[b] = v0, bandZ1_[b + 1] = v1, bandZ1_[b + 2] = v2, bandZ1_[b + 3] = v3;
        bandZ2_[b] = w0, bandZ2_[b + 1] = w1, bandZ2_[b + 2] = w2, bandZ2_[b + 3] = w3;
        bandGain_[b] = std::max(0.0f, g0), bandGain_[b + 1] = std::max(0.0f, g1);
        bandGain_[b + 2] = std::max(0.0f, g2), bandGain_[b + 3] = std::max(0.0f, g3);
    }
    // The low band, its envelope - smoothed to 60Hz, as its mean over a
    // second - and at each of its compressions' starts, a shock or not; its
    // shape added into the ring ahead, the ring's next sample out.
    static const float ENV_K = 1 - expf(-2 * PI * TY_ENV_HZ / RATE), THETA = TY_THETA,
                       LN_NP = TY_LN_NP, ALPHA = TY_ALPHA;
    static const float LN_A = LN_NP * 1.44269504f, LN_B = 0.5f * LN_NP * LN_NP * 1.44269504f;
    float b0 = j.lowB0, a1 = j.lowA1, a2 = j.lowA2;
    float y1 = j.lowY1, y2 = j.lowY2, z1 = j.lowZ1, z2 = j.lowZ2, x1 = j.lowX1, x2 = j.lowX2;
    float env = j.env, sum = 0, was = j.lowWas, inv = 1 / std::max(1e-9f, j.envMean);
    static const float muLo = TY_MU_LO, muHi = TY_MU_HI;
    float p = j.shockP, more = j.shockUnit * ALPHA, amp = j.shockAmp;
    float *ring = j.ring;
    int at = j.ringAt, fired = 0;
    const float *lo = half[0], *hi = half[1];
    float flutter = j.flutterWas, flutterBy = (j.flutterTo - j.flutterWas) * (1.0f / STEP);
    // The roar and its shocks ridden down at their very loudest - a reheat
    // pass 80m off, the take-off beside the runway, standing behind it lit -
    // so they peak under the soft limiter: their level as it will leave, over
    // a fifth of a second rising and a second falling, held to RIDE by a
    // third of what it goes over, and never over CEIL [I: eufi-methods 9.8].
    // Under that, as it is. The shocks ridden with the roar, not over it: in
    // v1 they weren't, and standing behind it lit 1% of its samples reached
    // the limiter [M: the judges' probe].
    float ride = j.ride, rideBy = 0, power = 0;
    {
        static const float RIDE = TY_RIDE, CEIL = TY_CEIL;
        float out = j.roarRms * gain_ * fade_ * duck_ * LEVEL;
        float want = out > RIDE ? std::min(powFast(RIDE / out, 0.6667f), CEIL * j.ceilBy / out) : 1.0f;
        rideBy = (want - ride) * (1.0f / STEP);
    }
    // No one shock past the soft limiter by itself, as heard: its peak no
    // more than the limiter's threshold - the giants, 15-18 standard
    // deviations where the recordings' reach 11, gone [M: v1, the judges].
    float scale = std::max(1e-6f, gain_ * fade_ * duck_ * LEVEL), most = 0.6f / (SHOCK[15] * ride * scale);
    float over = 0, threshold = 0.6f / (scale * (1 + 0.5f * echoGain_));  // the echo off the ground adding to it
    for (int i = 0; i < n; i++) {
        float x = noiseA[i];
        float y = b0 * (x - x2) - a1 * y1 - a2 * y2;
        x2 = x1, x1 = x, y2 = y1, y1 = y;
        float z = y - a1 * z1 - a2 * z2;
        z2 = z1, z1 = z;
        env += ENV_K * (fabsf(z) - env);
        sum += env;
        float m = env * inv;
        if (z > 0 && was <= 0 && p > 0 && rng.uniform() < p * (m - THETA)) {
            float a = std::min(most, amp * m * exp2Fast(LN_A * rng.gauss() - LN_B));
            for (int k = 0; k < SHOCK_N; k++) {
                ring[(at + k) & 63] += a * SHOCK[k];
            }
            p += more;
            fired++;
        }
        was = z;
        flutter += flutterBy;
        float jet = (lo[i] + muLo * (m - 1) * lo[i]) + (hi[i] + muHi * (m - 1) * hi[i]) * flutter;
        jet += ring[at];
        power += jet * jet;
        ride += rideBy;
        float heard = jet * ride;
        over += fabsf(heard) > threshold ? 1.0f : 0.0f;
        mix[i] += heard;
        ring[at] = 0;
        at = (at + 1) & 63;
    }
    j.lowY1 = fabsf(y1) < 1e-20f ? 0 : y1, j.lowY2 = fabsf(y2) < 1e-20f ? 0 : y2;
    j.lowZ1 = fabsf(z1) < 1e-20f ? 0 : z1, j.lowZ2 = fabsf(z2) < 1e-20f ? 0 : z2;
    j.lowX1 = x1, j.lowX2 = x2, j.lowWas = was;
    j.env = env;
    j.envSum = sum;
    j.ringAt = at;
    j.hawkesAdded += fired * ALPHA;
    j.ride = ride;
    if (j.lightS < 0.6f) {
        // the whoomp: noise through its resonance, its envelope a rise in
        // 10ms and a fall in 100ms, carried across the step
        float t0 = j.lightS - STEP_S, t1 = j.lightS;
        auto envelope = [](float t) { return t <= 0 ? 0.0f : (1 - expFast(-t / 0.01f)) * expFast(-t / 0.1f); };
        float e = j.whoomp * envelope(t0), eBy = (j.whoomp * envelope(t1) - e) * (1.0f / STEP);
        float b0 = j.whoompB0, a1 = j.whoompA1, a2 = j.whoompA2;
        float y1 = j.whoompY1, y2 = j.whoompY2, x1 = j.whoompX1, x2 = j.whoompX2;
        for (int i = 0; i < n; i++) {
            e += eBy;
            float x = e * noiseB[i];
            float y = b0 * (x - x2) - a1 * y1 - a2 * y2;
            x2 = x1, x1 = x, y2 = y1, y1 = y;
            mix[i] += y;
        }
        j.whoompY1 = fabsf(y1) < 1e-20f ? 0 : y1, j.whoompY2 = fabsf(y2) < 1e-20f ? 0 : y2;
        j.whoompX1 = x1, j.whoompX2 = x2;
    }
    {
        static const float UP = 1 - expf(-STEP_S / 0.05f), DOWN = 1 - expf(-STEP_S / 1.0f);
        float rms = sqrtf(power * (1.0f / STEP));
        j.roarRms += (rms - j.roarRms) * (rms > j.roarRms ? (j.roarRms > 0 ? UP : 1.0f) : DOWN);
        // The share of its samples over the limiter's threshold over the last
        // few seconds, and the ceiling lowered 3dB a second while that is over
        // 0.05%, raised 1dB a second under: as loud as the limiter's 0.1% lets
        // it be wherever it is heard from, the longest loud scene - lit,
        // standing behind it - as the shortest [O: the limiter's rule].
        static const float SHARE = 1 - expf(-STEP_S / 5.0f);
        j.overShare += (over * (1.0f / STEP) - j.overShare) * SHARE;
        j.ceilBy = clampf(j.ceilBy * (j.overShare > 0.00035f ? 0.99862f : 1.00023f), 0.3f, 1.0f);
    }
    // The whine's two resonances, and the tyres' squeal touching down - the
    // airliner's, as tyresStep() makes it, half as loud: two small main wheels.
    static const int RINGING[4] = {FAN_1, FAN_1 + 1, SQUEAL_1, SQUEAL_1 + 1};
    for (int k = 0; k < 4; k++) {
        Tone &t = tones_[RINGING[k]];
        if (!t.on) {
            continue;
        }
        const float *push = t.stream ? noiseB : noiseA;
        float pr = t.zr, pi = t.zi, pc = t.cr, ps = t.ci, pn = t.in * (k < 2 ? 1.0f : TY_SQUEAL);
        for (int i = 0; i < n; i++) {
            float a = (pn * push[i] - ps * pi) + pc * pr;
            pi = ps * pr + pc * pi;
            pr = a;
            mix[i] += a;
        }
        t.zr = fabsf(pr) < 1e-20f ? 0 : pr, t.zi = fabsf(pi) < 1e-20f ? 0 : pi;
    }
#ifdef ENGINE_NOISE_DEBUG
    g_fired += fired;
#endif
}

// THE TURBOPROP - the ATR 72's and ATR 42's sound (Kind::TURBOPROP, PROPS),
// as heard from where the camera is: at its stand, beside the taxiway and
// the runway, under the climb and the approach, behind it in the chase.
//
// Made as turboprop-methods.md's recipe (section 13) has it, on the A320's
// resonator design - every tone noise through a resonance, the broadband a
// few bands moved by a few shared slow processes, nothing replayed - its
// figures fitted to what CC0 and CC BY recordings of ATRs and their kin
// measure (targets/atr72.md). Every figure below says where it comes from:
// [P] published, [M] measured on those recordings, [I] inferred, [O] this
// project's own. What makes an ATR an ATR, on a speaker that plays nothing
// much below 300Hz [M: targets 0.2-0.3]:
//  - its propellers' blades passing, 85 to 120Hz and their harmonics: each
//    harmonic up to the sixth a resonance of its own. At the distances the
//    cameras sit a propeller's harmonics above the second fade and swell
//    apart, each as narrow-band noise does - their level wandering 3.6 to
//    5.2dB, their phases all but free of each other's, no band swelling
//    once a blade's passing [M: turboprop-methods 1.3, 1.4] - which a pulse
//    a blade, or any one waveform, can't do: that is a buzzer, its
//    harmonics locked (coherence 0.96-1.00), every band chopping [M: its
//    prototype B]. The fundamental steadier while governed, a steady line
//    under it - heard outside, its level wandering 2.4-5.1dB against
//    narrow-band noise's 5.4 [M: targets 4.4] - and the first two swelling
//    and fading together as the bearing to the two propellers turns. From
//    the seventh up a comb: a delay of one blade's passing fed back, a line
//    at every harmonic, each fading on its own [M: its option 4, coherence
//    0.17-0.46], played where those harmonics stand: on the ground and at
//    take-off power.
//  - how strong each harmonic is by what the blades are doing: falling
//    steeply in flight - 6-18dB from the first to the third, nothing from the
//    fourth up standing over 4dB 270-450m off - richer at take-off power, its
//    third to fifth still 8-9dB over the rush heard from ahead - and flat and
//    long in beta on the ground, the second to the eighth all standing, the
//    first the weakest [M: targets 4.3].
//    RP-1258 has it: standing, high levels "that tend to dominate the
//    higher sound harmonics ... disappears quickly when a small amount of
//    forward speed is attained" [P: NASA RP-1258 vol. 1, ch. 1,
//    https://ntrs.nasa.gov/citations/19920001380];
//  - its propellers' speed as its crew set it, not following the power: 71%
//    on the ground in beta, 100% for take-off and the approach, 82% climbing
//    and cruising [P: BEA, 9Y-TTC] - up from beta in some 2.5s and down from
//    take-off to climb in a straight glide of 8.5s, every propeller line on
//    the aircraft moving together [M: targets 4.1];
//  - its two propellers locked together while governed - one line each
//    harmonic, no beat [P: ATR at EWADE 2011; M] - their steady parts
//    swelling and fading as the bearing to them changes, two sources 8.10m
//    apart; and in beta, where nothing governs them, each its own speed, a
//    few tenths of a percent to a few percent apart and wandering [M: a
//    taxiing Dash 8's 4-5%, targets 3.6's pitch wander 1.1-2.4%];
//  - the missing fundamental: the harmonics the speaker plays, the third to
//    the sixth, carry the low pitch the ear supplies - resolved harmonics
//    give it whatever their phases [P: Houtsma & Smurzynski 1990,
//    https://doi.org/10.1121/1.399297] - so in flight, where the measured
//    fall leaves them 15-26dB under the first climbing and 24-38 on the
//    approach, the first one the speaker can't play, they are lifted to no
//    less than its level less 10dB an octave of harmonic number and no more
//    than less 3, as Moliner et al. bound theirs [P: DAFx 2020,
//    https://acris.aalto.fi/ws/portalfiles/portal/51973173/Eloi2020_Virtual_bass_DAFx2020.pdf]
//    - PR_LIFT, the user's to judge;
//  - its gearbox's line near a kilohertz, locked to the propeller, loudest
//    ahead [M: targets 5.2], where the speaker is good and the blades'
//    harmonics have died away;
//  - its gas generators' whine near 5.7kHz on the approach, far out two
//    lines a couple of percent apart - each engine's own speed, where the
//    governed propellers' lines are one [M: targets 5.1] - and a haystack
//    about it around overhead [M: turboprop-methods 1.7];
//  - its propellers' rush, darker than an airliner's at power and churning
//    livelier, its octaves moving together more [M: targets 3.1, 3.5]; and
//    at its stand in hotel mode its right gas generator alone, no propeller
//    turning [P: AAIB, G-CMFI].
// And its way to the ear as the airliner's, with three things of its own: its
// Doppler eased in 35ms, as the fighter's - a pass sweeps a whole harmonic
// series and the gearbox's line together, and the airliner's 0.25s left a
// close pass's pitch 1.2 semitones late [I: turboprop-methods 10]; each of
// its tones' echo off the ground let swell and fade by up to 6dB at 300Hz
// and under, 2 from 1.5kHz, where the airliner's is held to 2 - a
// propeller's harmonics phasing in and out as it passes overhead is the
// ground's, and the reflection is coherent that low [I: turboprop-methods
// 11]; and its two propellers 4.05m either side of it, not the airliner's
// engines' 5.75m (placedWay()).

namespace {

// Its level: its two propellers' fundamental at take-off power and 100%,
// abeam, dB as the airliner's sources are reckoned - set so that its
// take-off beside the runway is some 6dB under the A320's in the speaker's
// band, and its approach some 5dB under the A320's from the same place, as
// the noise data puts them 5-8 and 4-6dB apart, A-weighted [P: EASA ANP
// v2.3, https://www.easa.europa.eu/en/downloads/138164/en] - over seven
// airframes the renders' take-off 6.1dB under, their approach 3.1 under with
// the lift (PR_LIFT) on and 4.3 with it off (tools/engine_preview/NOTES.md).
constexpr float PR_LEVEL_DB = -2.0f;
// Its harmonics' level with its shaft power and its propellers' speed: as
// the power, 10dB a decade of it - the noise data's approach to full power
// 9-12dB at 100% [P: ANP], loading noise as the thrust [I: turboprop-methods
// 4.3]; and as the speed's 2.5th power, some 4.3dB from 100% down to 82% [I:
// Gutin's Bessel terms an upper bound, turboprop-methods 4.3].
constexpr float PR_TORQUE_LAW = 10.0f, PR_SPEED_LAW = 50.0f;
// In beta, the blades flat - their power next to nothing, as the 3D view
// asks it, 0.05 to 0.17 - the propellers' harmonics and rush as their power
// makes them, the harmonics PR_BETA_TONE_DB and the rush PR_BETA_RUSH_DB
// more: what the ground's vortices and the distorted inflow add [P:
// RP-1258, "high levels of unsteady-loading noise" standing], set so that a
// taxi past 35m off is as loud in the speaker's band as the A320's - quiet
// for its size in the air, not on the ground [I: atr72-sources 10] - and
// its harmonics stand 6-12dB over the rush, as the taxiing Dash 8s' do [M:
// targets 3.6].
constexpr float PR_BETA_TONE_DB = -13.5f, PR_BETA_RUSH_DB = -6.0f;
// The harmonics' envelope, dB under the strongest, for harmonics 1 to 8 -
// and the slope on from there, dB an octave of harmonic number - fitted to
// the outside recordings, not the Kajaani clip, which was recorded aboard
// [M: targets 10.3]:
//  - in clean flight at low power, the approach: the second 10dB under the
//    first, the third 24 - fitted to the ATR 72 at 450m, its first standing
//    13-16dB over the rush, its second 7-10, its third 0-4.5 and the rest
//    lost in it [M: targets 3.1, 9], with the lift (PR_LIFT) off - then 14dB
//    an octave;
//  - in flight at high power, the climb: the second 10dB under, the third
//    15 - the Q400 climb-outs' second 5-9dB under the first and their third
//    8-13 as received [M: targets 3.5], the second fitted to their 5-12dB
//    over the rush, their camera's and recorder's low end the weaker [I] -
//    then 11dB an octave;
//  - standing at take-off power: the second 7dB under the first, the third
//    to the fifth 9.5, 12 and 14 - so that through the speaker's 180Hz
//    high-pass, which takes 7.8dB off a 120Hz first and 1.2 off its second,
//    they come out as the Q400's roll heard from ahead was received, its
//    first two equal and its third to fifth 1.5, 4 and 6dB under [M: targets
//    3.4], a camera's soundtrack, its low end the weaker [I] - then falling
//    10dB an octave; on the ground at take-off power, rolling at any
//    speed, half way to it from clean flight's, the propeller still taking
//    in the ground's disturbed air [I: that roll's harmonics strong at
//    70m/s];
//  - in beta standing: the first weak, some 12dB under, the second to the
//    sixth within 3dB of each other, falling 8dB an octave on [M: the two
//    taxiing Dash 8s, targets 3.6 - the 7-11 harmonics standing 6-18dB to
//    535-620Hz, the fundamental 0-5dB];
//  - in reverse, flatter still, 6dB an octave [I: turboprop-methods 8].
constexpr float PR_E_FLIGHT[8] = {0.0f, -10.0f, -24.0f, -31.0f, -35.0f, -38.0f, -41.0f, -44.0f};
constexpr float PR_E_CLIMB[8] = {0.0f, -10.0f, -15.0f, -20.0f, -23.0f, -26.0f, -28.5f, -31.0f};
constexpr float PR_E_TAKEOFF[8] = {0.0f, -7.0f, -9.5f, -12.0f, -14.0f, -20.0f, -24.0f, -27.0f};
constexpr float PR_E_BETA[8] = {-12.0f, 0.0f, 0.0f, -1.0f, -2.5f, -3.5f, -6.0f, -8.5f};
constexpr float PR_E_REVERSE[8] = {-6.0f, -2.0f, 0.0f, -1.0f, -2.0f, -3.0f, -4.0f, -5.0f};
constexpr float PR_SLOPE_FLIGHT = -14.0f, PR_SLOPE_CLIMB = -11.0f, PR_SLOPE_TAKEOFF = -10.0f, PR_SLOPE_BETA = -11.0f,
                PR_SLOPE_REVERSE = -6.0f;
// The lift for the speaker - 0 the physics alone, 1 Moliner's bounds (see
// above) - over the first four harmonics heard at 300Hz or more, up to the
// sixth [I: turboprop-methods 7.3; the user's ear to judge].
constexpr float PR_LIFT = 1.0f;
// Each harmonic's width, Hz, as its fading's pace has it: 0.85Hz for the
// third and 0.35Hz more for each harmonic on, in clean flight; 2.2 times
// that standing, a fifth more in beta, 6Hz more in reverse [I]; and the
// airliner's widening far off on top [M: the harmonics' amplitude moving at
// 0.5-1.5Hz in flight, 1.6-2.3Hz standing, turboprop-methods 1.3; targets
// 4.4: a governed fundamental's fading's correlation time 150ms, the
// harmonics above it 85ms, on the ground 60ms]. The first two narrower
// still, 0.2 and 0.4Hz: a governed fundamental is a line within 0.01Hz
// standing [M: turboprop-methods 1.3], and so narrow a resonance's level
// wanders slowly, a 2s mean's worth - steadier, as a governed one measures,
// 2.4-5.1dB and 0.26-0.46 of its mean against narrow-band noise's 5.4dB and
// 0.52 [M: targets 4.4] - its steady part's lobes, below, on its level.
constexpr float PR_WIDTH_1 = 0.2f, PR_WIDTH_2 = 0.4f, PR_WIDTH_3 = 0.85f, PR_WIDTH_BY = 0.35f, PR_WIDTH_STATIC = 1.2f,
                PR_WIDTH_REVERSE = 6.0f;
// The steady part of the first two harmonics while governed: K, its power
// over the random part's - the fundamental's 4, the second's 0.8 - as heard
// outside: a governed fundamental's amplitude wandering 0.26-0.46 of its
// mean, its level 2.4-5.1dB, the harmonics above it 0.38-0.68 [M: targets
// 4.4, the ATR 72's approach and the Q400's roll and climbs]; K 1 to 5 makes
// 0.45 to 0.27 alone, the ground's echo and the two propellers' lobes
// adding to it. The fundamental's steady part a line of its own; the
// second's only its share of the level the two propellers' lobes move.
// (The 5.6-10.6 of turboprop-methods 1.3 was measured aboard [M: targets
// 10.3]: the cabin's, steadier.) None in beta, where the first is as random
// as the rest.
constexpr float PR_K1 = 4.0f, PR_K2 = 0.8f;
// Ungoverned in beta, each propeller's speed wandering 1%, over most of a
// second [M: the strongest harmonic's pitch wandering 1.1-2.4% taxiing,
// targets 4.4, its resonance's own wander adding to it]; the right running
// half a percent off the left (its row's splitBeta). Governed, the two
// hunting together a tenth of a percent, the propeller control holding
// them to a rpm or so [P: ATR at EWADE 2011; M: governed 0.13-0.83%, the
// resonances' own wander most of it].
constexpr float PR_WANDER = 0.010f, PR_WANDER_S = 0.8f, PR_HUNT = 0.0012f, PR_HUNT_S = 0.7f;
// Its power's spool (propDoing()): rising by at most PR_SPOOL_GROW of itself
// a second and PR_SPOOL_FLOOR more, ground idle to take-off in some 5s [P:
// 14 CFR 33.73's 5s; M: the Q400 at Itami 16dB over 4s]. Its propellers' speed
// up from beta critically damped at PR_NP_RISE_W a second, its 10-90% in
// some 2.5-3s [M: aboard an ATR 72, 850 to 1,200rpm in 2.5-3.5s, targets
// 4.1]. Each band's air loss let fall to PR_AIR_FLOOR_DB, where the airliner's
// stops at 30dB: at FL170 5km of air takes over 100dB off 6kHz [P: ISO
// 9613-1]. Not what made the cruise's hiss - lowering it changed nothing
// measurable there [M] - that was the tones' skirts (PR_CUT_*); kept as the
// physics.
constexpr float PR_SPOOL_GROW = 0.30f, PR_SPOOL_FLOOR = 0.05f, PR_NP_RISE_W = 1.2f, PR_AIR_FLOOR_DB = -95.0f;
// Its whole sound in the air by its power, PR_FLIGHT_POWER_DB a decade of its
// share of take-off power, nought at take-off and fading out as it slows on
// the ground (the static share): so the approach at 22% is 2.3dB down, the
// climb at 89% 0.2dB, take-off power the same. The harmonics' and the
// rush's own laws left the approach too near the climb at the same slant -
// 7.1dB A-weighted over it, where ANP's ATR 72 gives 9-12dB [P: EASA ANP
// v2.3, https://www.easa.europa.eu/en/downloads/138164/en] - and 3.1dB under
// the A320's approach, where ANP gives 4-6dB; the approach's spectrum, matched
// to the Schiphol recording, is moved as a whole, not reshaped.
constexpr float PR_FLIGHT_POWER_DB = 3.5f;
// The tones' low-pass beyond 3km (propTones()): each resonance's noise is fed
// straight to its output, so a tone carries a white skirt some 30dB under it,
// wider as its line widens with distance - at FL170 that skirt, which no air
// takes, was the hiss 9-11dB under the 0.5-1kHz band [M: the cruise without
// propTones() 38dB under]. Two one-poles, their corner from 8kHz at 3km falling
// 1.8 octaves a kilometre to 400Hz (the broadband's muffling's least), each
// tone's input raised by what they take at its pitch, so the lines are as
// before and only their skirts go.
constexpr float PR_CUT_FROM_KM = 3.0f, PR_CUT_OCTAVES_KM = 1.8f;
// The harmonics' shared surge: all of them, and the steady line, a little
// together - their 50ms wanders correlated -0.2 to +0.3 between harmonics,
// so not much [M: turboprop-methods 6] - nepers, and its two parts' time
// constants, s.
constexpr float PR_TONE_SURGE_NP = 0.10f, PR_TONE_SURGE_Q_S = 0.4f, PR_TONE_SURGE_S_S = 1.15f;
// The comb's lines, from the seventh harmonic up: each as wide as 2.5Hz in
// clean flight, twice that standing [I: turboprop-methods 6, 2-3 and 4-6Hz],
// and the airliner's widening far off.
constexpr float PR_COMB_WIDTH = 2.5f;
// The gearbox's line [M: targets 5.2; aboard, turboprop-methods 1.6]: at
// its row's gearA times the propeller's turning, 22dB under the fundamental
// on the approach - 26 by the order spectrum at 450m, -19 to -21 by its
// heterodyne - 6-16dB over its neighbours there, 12-25 close; as the root of
// the shaft power and the speed's 1.5th power [I: a gear mesh's whine];
// 0.4Hz wide, wandering slowly, as measured aboard [M: 2.5-3.7dB,
// 0.65-0.75Hz]. Its second line, gearB, heard aboard only - under the rush
// 450m off [M: targets 5.2] - is not played.
constexpr float PR_GEAR_A_DB = -22.0f, PR_GEAR_WIDTH = 0.4f;
// The gas generators [M: targets 5.1; turboprop-methods 1.7]: each one's
// whine at its row's whineHz at the approach's speed (NH 85% [I]), as the
// speed goes; its level, dB under the fundamental at take-off power, and
// as the speed's fourth power [I: a compressor's tone]; the two engines a
// couple of percent apart, each wandering a quarter of a percent over three
// seconds [M: far out two lines 2.4% apart; I: the torque trimmed on the
// approach]; and a haystack about them, 250Hz wide at 5.9kHz on the
// approach, heard around overhead [M: turboprop-methods 1.7].
constexpr float PR_WHINE_DB = -36.0f, PR_WHINE_APART = 0.012f, PR_WHINE_WANDER = 0.0025f, PR_WHINE_S = 3.0f;
constexpr float PR_HAY_HZ = 5900, PR_HAY_WIDTH = 250, PR_HAY_DB = -36.0f;
// The broadband, each source's level, dB as the harmonics' are, and its
// peak, Hz, before the air's loss [M: targets 3.1-3.6, the octaves' shapes;
// I: what makes each]:
//  - the propellers' loading rush: the turbulence through the discs and the
//    blades' unsteady loading - as the power's cube and the speed's
//    third, most at 450Hz at 100%, rising 3dB an octave under it, falling
//    6dB an octave over: a climb-out's dark, low-weighted band [M: the Q400
//    climb-outs' centroid 600-895Hz, their 250 and 500Hz octaves the
//    loudest]; 6dB more standing at take-off power, and behind it at its
//    start of roll as Doc 29 has it, up to 10dB over the side [P: ECAC Doc
//    29 4th ed. vol. 2, eq. 4-24b];
//  - the blades' own edges and tips: as the speed's fifth power, little
//    with the power, most at 1.2kHz at 100%, rising 12dB an octave under,
//    falling 2dB over - the approach's hump at a kilohertz over a weaker
//    500Hz [M: the ATR 72 at 450m, its octaves 500Hz to 4kHz -7.0, -3.3,
//    -7.0 and -14.9dB against the band, the 1kHz the loudest];
//  - its gas generators' exhaust, hot and slow, from the nacelles' sides:
//    weak, aft, most at 400Hz [I: atr72-sources 4];
//  - its gas generators' compressors, out of the intakes: most at 2.2kHz,
//    as a Dash 8's gas generator alone, and loud on the ground - beta
//    taxiing only 4-7dB over the gas generator alone [M: its 2-2.5kHz bands
//    the loudest, atr72-sources 4, 14.2] - the taxi's brightness [M: the
//    taxiing Dash 8s' centroid 1.7kHz, their 4kHz octave 7dB under the
//    band];
//  - its blades in reverse pitch: a roar most at 630Hz falling 11dB an
//    octave over a kilohertz, 8dB over the roll-out before it [M: the ATR
//    42-500 at London City, targets 3.3];
//  - its airframe as the A320's, PR_FRAME_DB under: its gear and flaps at
//    113 knots against an A320's 136, and smaller [I: turboprop-methods 13.2];
//  - the propellers' low rush, most at 170Hz at 100% - the turbulence they
//    take in broadening their first harmonics, the blades' leading edges -
//    falling 9dB an octave over it, as the speed's fifth power and little
//    with the power: under the bank's bands, a ninth of its own at 140Hz -
//    the speaker plays none of it, a full-range one all - as loud in the
//    250Hz octave as the whole 300Hz-6kHz band on the approach 450m off,
//    under the first two harmonics 10-12dB down, a hertz-fine floor [M:
//    fs315660, targets/atr72.json; the noise data's approach spectrum, its
//    160Hz third-octave 8.5dB over its 1kHz one, P: ANP class 240].
constexpr float PR_LOAD_DB = -12.0f, PR_LOAD_HZ = 450, PR_LOAD_STATIC_DB = 6.0f;
constexpr float PR_LOW_DB = -15.0f, PR_LOW_HZ = 170, PR_LOW_BAND_HZ = 140;
constexpr float PR_EDGE_DB = -19.0f, PR_EDGE_HZ = 1200;
constexpr float PR_EXHAUST_DB = -27.0f, PR_EXHAUST_HZ = 400;
constexpr float PR_GG_DB = -24.0f, PR_GG_HZ = 2200;
constexpr float PR_REVERSE_DB = -17.0f, PR_REVERSE_HZ = 630;
constexpr float PR_FRAME_DB = -7.0f;
// The tyres' squeal touching down against the A320's - four main wheels on
// a third of its weight, "a short, light chirp" [I: atr72-sources 6] - and
// the nose wheel's thump over each centreline light.
constexpr float PR_SQUEAL = 0.3f, PR_THUMP = 0.5f;
// The broadband sources' surges, each as the airliner's are - how far, in
// nepers of its loudness, its quick and slow parts' time constants, s, and
// the quick part's weight - by the airliner's Source places: its loading
// rush in OF_FAN's, its blades' edges in OF_FAN_AFT's, its exhaust in
// OF_JET's, its compressors in OF_CORE's; its airframe, reverse and combs
// in theirs. Livelier than an airliner's - a turboprop's level churns
// 1.5-2.7dB on the approach where an A320's churns 1.0-1.6, its octaves
// moving together 0.29-0.73 against 0.23-0.37 [M: targets 0.7], surging some
// once a second by 2.5dB, its fluctuation's correlation time 130-150ms [M:
// the approach passes, targets/atr72.json] - its two propeller sources on one
// surge, so the whole band moves with them, quickly.
constexpr float PR_SURGE_NP[Synth::SOURCES] = {0.15f, 0.36f, 0.26f, 0.10f, 0.24f, 0.25f, 0.20f, 0.10f, 0.20f};
constexpr float PR_SURGE_QUICK_S[Synth::SOURCES] = {0.35f, 0.15f, 0.25f, 0.30f, 0.40f, 0.12f, 0.25f, 0.30f, 0.30f};
constexpr float PR_SURGE_SLOW_S[Synth::SOURCES] = {1.15f, 0.5f, 1.0f, 1.3f, 1.15f, 0.50f, 1.0f, 1.7f, 1.15f};
constexpr float PR_SURGE_QUICK_W[Synth::SOURCES] = {0.5f, 0.75f, 0.5f, 0.6f, 0.4f, 0.8f, 0.5f, 0.6f, 0.4f};
// ...and standing, its rush as steady as a taxiing Dash 8's - 0.65-0.86dB, at
// 1-4Hz a tenth of it [M: targets 3.6] - nepers. Each band's own shimmer
// two thirds of the airliner's, so the octaves move together as much as a
// turboprop's do.
constexpr float PR_RUSH_STILL_NP = 0.12f, PR_RUSH_LOW_NP = 0.10f, PR_RUSH_FAR_NP = 0.2f, PR_SHIMMER = 0.67f;
// The air's flicker, as the airliner's but 0.6 of its depth: far off an
// approaching turboprop churns 0.9-1.2dB [M: targets 3.1, 1.4-2km], where the
// airliner's 1.25dB at a kilometre, more further, left it 1.6.
constexpr float PR_FLICKER = 0.6f;

//                             0     20    40    60    80   100   120   140   160   180
// The blade tones, most near the propellers' plane, a little ahead of it in
// flight - at low power the first two -12dB at 30-40 degrees, nought at
// 60-70, -18 to -23dB at 110-140 [M: the ATR 72's approach,
// turboprop-methods 1.9, targets 4.6]; at high power far less deep ahead,
// -7 to -10dB at 20 degrees, -5 at 40-60 [M: the two Q400 climb-outs,
// targets 4.6; and its take-off roll's harmonics strong heard from ahead,
// targets 3.4] - its loading's thrust term radiating forward [P: Gutin, NACA
// TM 1195, https://ntrs.nasa.gov/citations/20030068996]; behind, 7-32dB down
// beyond 110 degrees, typically 19, at either. The higher harmonics at low
// power narrower toward the axis - its steady loading's harmonics nothing
// there, the Bessel terms falling steeply with the harmonic's number, the
// axis filled only by unsteady loading and scattering [P: Gutin; I] - -26 to
// -30dB ahead and behind, as an approach 1.5-2km out has its third and up
// lost in the rush where its first still stands 9dB [M: fs315660, targets
// 3.1]; at high power as broad as the first two. Between the two by the
// power (PROP_TONE and PROP_HIGH, their _POWER twins). Standing, broader
// still - the distorted inflow's unsteady loading radiating all round - a
// third as deep (PR_STATIC_LOBE).
constexpr Lobe PROP_TONE = {-18.0f, -17.0f, -10.0f, -2.0f, 0.0f, -2.0f, -13.0f, -19.0f, -22.0f, -25.0f};
constexpr Lobe PROP_HIGH = {-28.0f, -26.0f, -15.0f, -4.0f, 0.0f, -3.0f, -15.0f, -22.0f, -27.0f, -30.0f};
constexpr Lobe PROP_TONE_POWER = {-10.0f, -8.5f, -5.5f, -4.0f, -1.0f, -3.5f, -12.0f, -19.0f, -23.0f, -26.0f};
constexpr Lobe PROP_HIGH_POWER = {-12.0f, -10.0f, -7.0f, -4.0f, -1.0f, -4.0f, -13.0f, -20.0f, -24.0f, -27.0f};
constexpr float PR_STATIC_LOBE = 0.65f;
// The gearbox's line, loudest ahead, 8dB down abeam, 4 behind [M: targets
// 5.2; turboprop-methods 1.6].
constexpr Lobe GEARBOX = {0.0f, 0.0f, -4.0f, -6.5f, -7.6f, -7.7f, -6.5f, -4.0f, -3.0f, -3.0f};
// The propellers' rush: within some 5dB from 20 to 130 degrees, 7-15dB down
// by 150-160 [M: targets 4.6, three passes - the ATR 72's approach -7 to -11
// at 120-140 degrees, the two Q400 climb-outs nought to -6]; standing at
// take-off power, behind it as Doc 29's start of roll has it [P: Doc 29 eq.
// 4-24b: +6.9dB at 150 degrees, +10.1 at 180, against the side].
constexpr Lobe PROP_BROAD = {-6.0f, -5.0f, -2.0f, 0.0f, -1.0f, -1.0f, -2.0f, -5.0f, -11.0f, -16.0f};
constexpr Lobe PROPWASH = {-6.0f, -5.0f, -2.0f, 0.0f, -1.0f, -1.0f, -2.0f, 2.0f, 7.0f, 10.0f};
// Its exhaust, to the side and aft; its compressors and their whine, out of
// the intakes ahead and to the side; the haystack, ahead and below, gone
// behind [I: atr72-sources 14.1-14.6; M: turboprop-methods 1.7].
constexpr Lobe EXHAUST = {-12.0f, -11.0f, -9.0f, -7.0f, -4.0f, -2.0f, 0.0f, 0.0f, -1.0f, -3.0f};
constexpr Lobe INTAKES = {0.0f, 0.0f, -1.0f, -2.0f, -3.0f, -4.0f, -6.0f, -9.0f, -12.0f, -15.0f};
constexpr Lobe WHINES = {-4.0f, -2.0f, -1.0f, 0.0f, 0.0f, -1.0f, -3.0f, -7.0f, -11.0f, -15.0f};
constexpr Lobe HAYSTACK = {-8.0f, -5.0f, -2.0f, 0.0f, 0.0f, -1.0f, -4.0f, -10.0f, -16.0f, -20.0f};

// The turboprop's tones' places in propTones_.
enum PropSlot {
    PR_HARM = 0,     // the harmonics 1-6 - both propellers' governed, the left's in beta - 0-5
    PR_HARM_R = 6,   // the right propeller's 2-6 in beta, those heard at 300Hz or more, 6-10
    PR_STEADY = 11,  // the fundamental's steady line, governed
    PR_GEAR = 12,    // the gearbox's line
    PR_WHINE = 13,   // each gas generator's whine, 13-14
    PR_HAY = 15,     // the haystack about them
};

}  // namespace

// The propellers' low band, under the bank's: a band-pass and a resonance on
// the same poles at PR_LOW_BAND_HZ, as each of the bank's bands is (see
// Synth()), scaled for a noise of unit power to come out of it at unit power.
struct LowBand {
    float b0, a1, a2;
    LowBand() {
        float s, c;
        sinCosTurns(PR_LOW_BAND_HZ / RATE, s, c);
        float alpha = s / (2 * BAND_Q), a0 = 1 + alpha;
        b0 = alpha / a0, a1 = -2 * c / a0, a2 = (1 - alpha) / a0;
        float y1 = 0, y2 = 0, z1 = 0, z2 = 0, sum = 0;
        for (int i = 0; i < 16000; i++) {
            float x = i == 0 ? 1.0f : i == 2 ? -1.0f : 0.0f;
            float y = b0 * x - a1 * y1 - a2 * y2;
            float z = y - a1 * z1 - a2 * z2;
            y2 = y1, y1 = y, z2 = z1, z1 = z;
            sum += z * z;
        }
        b0 = -b0 / sqrtf(sum / 3);  // turned over: its neighbour on the same noise is the bank's 400Hz band
    }
};
const LowBand &lowBand() {
    static const LowBand band;
    return band;
}

// Another turboprop starting: as it is now - its propellers turning at the
// speed asked if its engines run, its hotel mode's right engine running if
// asked - its own little differences from who it is.
__attribute__((noinline)) void Synth::propRestart() {
    const Sound &s = heard_;
    const PropData &p = propData(s.engine);
    const EngineData &e = ENGINES[(int)s.engine];
    Prop &q = prop_;
    q = Prop();
    q.aspect = clamp01(s.aspect);
    q.closing = std::max(-SOUND_MS, std::min(0.8f * SOUND_MS, s.closingMs));
    q.doppler = SOUND_MS / (SOUND_MS - q.closing);
    uint32_t h = mix(traits_ ^ 0x70726f70u);
    auto next = [&]() {
        h = mix(h + 0x9e3779b9u);
        return unit(h);
    };
    // The synchrophaser's phase between its two propellers, the airframe's
    // own [I: the ATR's setting not published]; its gas generators' whines
    // a percent or so either side of their mean, the right's the higher or
    // the lower by who it is [M: 2.4% apart far out].
    q.psi = next();
    lowBand();  // worked out once, before its first sums
    float apart = PR_WHINE_APART * (0.6f + 0.8f * next()) * (next() < 0.5f ? -1 : 1);
    q.whineBy[0] = -apart;
    q.whineBy[1] = apart;
    // Its propellers as they are now: their speed asked, or the synth's own
    // guess from its power; governed or in beta; its power; its engines.
    bool onGround = s.rollingMs > 0.5f || (s.sourceUpM < 12 && s.airspeedMs < 35);
    bool beta = onGround && (s.reverse || s.power < 0.2f);
    float np = s.np >= 0 ? s.np : beta ? (s.reverse ? p.npReverse : p.npGround) : p.npClimb;
    q.governed = beta ? 0.0f : 1.0f;
    q.reverse = s.reverse ? 1.0f : 0.0f;
    q.torque = clamp01(s.power);
    q.stat = onGround ? 1 - smoothstep(15, 40, std::max(0.0f, s.airspeedMs)) : 0.0f;
    for (int g = 0; g < 2; g++) {
        bool runs = s.running || (g == 1 && s.apu && p.hotel);
        bool turns = s.running && (g == 0 || !s.apu || !p.hotel);
        q.spool[g] = q.lit[g] = runs ? 1.0f : 0.0f;
        float idle = onGround ? e.groundIdle : e.flightIdle;
        q.nh[g] = !runs                      ? 0.0f
                  : g == 1 && s.apu && p.hotel ? 0.70f
                  : s.reverse                  ? e.reverse
                  : beta                       ? idle + 0.03f * clamp01((clamp01(s.power) - 0.05f) * (1 / 0.12f))
                                               : idle + (1 - idle) * sqrtf(std::max(0.0f, clamp01(s.power) - 0.05f) * (1 / 0.95f));
        q.pitch[g] = turns ? 1.0f : 0.0f;
        q.np[g] = turns ? np : 0.0f;
    }
    for (Tone &t : propTones_) {
        t = Tone();
    }
    // Its broadband's surges at its own paces and depths (PR_SURGE_*), in
    // place of the airliner's setSurges() has just set.
    for (int c = 0; c < SOURCES; c++) {
        float kq = expf(-SLOW_S / PR_SURGE_QUICK_S[c]), ks = expf(-SLOW_S / PR_SURGE_SLOW_S[c]);
        surgeKeep_[c][0] = kq;
        surgeKeep_[c][1] = sqrtf(1 - kq * kq);
        surgeKeep_[c][2] = ks;
        surgeKeep_[c][3] = sqrtf(1 - ks * ks);
        surgeKeep_[c][4] = sqrtf(1 - PR_SURGE_QUICK_W[c] * PR_SURGE_QUICK_W[c]);
        surgeNp_[c] = PR_SURGE_NP[c];
    }
}

// Each 2ms: its tones' pitches moved on toward their next, as the airliner's
// are in update(); the rest of what it does moves no faster than its 32ms
// step needs it to (propSlowStep()).
__attribute__((noinline)) void Synth::propStep() {
    for (Tone &t : propTones_) {
        if (t.on && t.di != 0) {
            float cr = t.cr * t.dr - t.ci * t.di;
            t.ci = t.cr * t.di + t.ci * t.dr;
            t.cr = cr;
        }
    }
}

// Each 32ms, before the sums: where it is heard from, quickly; its power, its
// gas generators, its propellers' pitch and speed - none of which moves
// faster than its tones' sums each 32ms can follow.
__attribute__((noinline)) void Synth::propDoing() {
    const Sound &s = sound_;
    const PropData &p = propData(s.engine);
    const EngineData &e = ENGINES[(int)s.engine];
    Prop &q = prop_;
    static const float QUICK = 1 - expf(-SLOW_S / 0.035f), LIGHT = 1 - expf(-SLOW_S / 0.8f),
                       OUT = 1 - expf(-SLOW_S / 0.3f), WIND_DOWN = expf(-SLOW_S / 12), UP = 1 - expf(-SLOW_S / 0.5f),
                       DOWN = 1 - expf(-SLOW_S / 1.0f), GOVERN = 1 - expf(-SLOW_S / 2.0f),
                       UNGOVERN = 1 - expf(-SLOW_S / 0.3f), REVERSE = 1 - expf(-SLOW_S / 0.8f),
                       NH_UP = 1 - expf(-SLOW_S / 1.2f), NH_DOWN = 1 - expf(-SLOW_S / 2.0f),
                       SPIN_DOWN = expf(-SLOW_S / 8.0f), BRAKE = expf(-SLOW_S / 1.5f);
    // Which way it faces and how fast it closes, eased in 35ms: a pass
    // sweeps its whole harmonic series and its gearbox's line together, and
    // the airliner's 0.25s left a close pass's pitch drop late by up to 1.2
    // semitones [I: turboprop-methods 10]. Placed, the way is already smooth;
    // the tones' pitches glide from one 32ms step's to the next's.
    q.aspect = ease(q.aspect, clamp01(s.aspect), QUICK);
    float closing = s.placed ? std::max(-SOUND_MS, std::min(0.8f * SOUND_MS, s.closingMs))
                             : std::max(-150.0f, std::min(150.0f, s.closingMs));
    q.closing = ease(q.closing, closing, QUICK);
    q.doppler = SOUND_MS / (SOUND_MS - q.closing);
    // What it is doing: on the ground; in beta - on the ground, its power
    // levers under flight idle or in reverse - where nothing governs its
    // propellers; its power, eased; its static share, 1 standing, nought by
    // 40m/s, where the harmonics standing's richness has gone [P: RP-1258].
    bool onGround = s.rollingMs > 0.5f || (s.sourceUpM < 12 && s.airspeedMs < 35);
    bool beta = onGround && (s.reverse || s.power < 0.2f);
    float power = clamp01(s.power);
    // Its power rising no faster than its engines accelerate: by at most a
    // share of itself each second (PR_SPOOL_GROW) and a little more
    // (PR_SPOOL_FLOOR), so that it comes up from ground idle to take-off
    // in some 5s, its loudness rising evenly in dB rather than in the half
    // second the easing alone gave (+20dB in 1.3s, where a Dash 8 Q400
    // rolling at Itami rises 16dB over some 4s and the A320 from the same
    // place 17dB over 7s [M: review atr72-v1]); certification asks for
    // flight idle to 95% of take-off power in no more than 5s [P: 14 CFR
    // 33.73, https://www.ecfr.gov/current/title-14/chapter-I/subchapter-C/part-33/subpart-E/section-33.73].
    static const float GROW = expf(SLOW_S * PR_SPOOL_GROW), FLOOR = PR_SPOOL_FLOOR * SLOW_S;
    float eased = ease(q.torque, power, power > q.torque ? UP : DOWN);
    q.torque = power > q.torque ? std::min(eased, q.torque * GROW + FLOOR) : eased;
    q.governed = ease(q.governed, beta ? 0.0f : 1.0f, beta ? UNGOVERN : GOVERN);
    q.reverse = ease(q.reverse, s.reverse ? 1.0f : 0.0f, REVERSE);
    q.stat = onGround ? 1 - smoothstep(15, 40, airspeed_) : 0.0f;  // the ground's disturbed air, on the ground only
    q.ground = onGround ? 1.0f : 0.0f;
    // Its gas generators: the left's running with its engines, the right's
    // too - or alone, in hotel mode, its propeller braked [P: AAIB, G-CMFI].
    // Each started as the airliner's is - motored to a third of idle, lit,
    // on up to idle in 14s more - and shut down, its fire out at once, its
    // spool winding down over some 12s [M: a Dash 8's whine an octave down
    // in 7s, targets 3.7]. Its speed (NH) from its power: 66% idling on the
    // ground, 72% in the air, 85% on the approach, 100% at take-off power [I:
    // none published; its ENGINES row], as the root of the power over idle;
    // 70% in hotel mode, its bleed and generator loaded [I].
    for (int g = 0; g < 2; g++) {
        bool runs = s.running || (g == 1 && s.apu && p.hotel);
        bool hotel = g == 1 && s.apu && p.hotel;
        float &sp = q.spool[g];
        if (runs) {
            sp = std::min(1.0f, sp + (sp < 0.35f ? 0.35f / e.motorS : 0.65f / 14) * SLOW_S);
        } else {
            sp = sp < 0.002f ? 0 : sp * WIND_DOWN;
        }
        bool burning = runs && sp >= 0.35f;
        q.lit[g] = ease(q.lit[g], burning ? 1.0f : 0.0f, burning ? LIGHT : OUT);
        float idle = onGround ? e.groundIdle : e.flightIdle;
        float want = hotel              ? 0.70f
                     : q.reverse > 0.5f ? e.reverse
                     : beta             ? idle + 0.03f * clamp01((power - 0.05f) * (1 / 0.12f))
                                        : idle + (1 - idle) * sqrtf(std::max(0.0f, power - 0.05f) * (1 / 0.95f));
        want *= sp < 1 ? powFast(std::max(sp, 1e-3f), 0.7f) : 1.0f;
        q.nh[g] = ease(q.nh[g], want, want > q.nh[g] ? NH_UP : NH_DOWN);
    }
    // Its propellers' speed asked: as the 3D view gives it, or the synth's
    // own guess from its power where it doesn't [I: turboprop-methods 13.4].
    float ask = s.np;
    if (ask < 0) {
        ask = beta ? (s.reverse ? p.npReverse : p.npGround)
              : power >= 0.95f || (onGround && power >= 0.6f) ? p.npTakeoff
              : s.flapsOut && s.gearDown                       ? p.npApproach
                                                               : p.npClimb;
    }
    // Each propeller: out of feather once its engine idles - the left as its
    // engines run, the right once hotel mode has let its brake go - over 4s,
    // its speed rising from a feathered windmill's to what is asked, swelling
    // as it comes [I: atr72-sources 14.1]; feathered as its engine is shut
    // down, in a second, its speed falling a fifth as its blades unload, and
    // then gone, spinning down unheard [M: a Dash 8's line 17-23% down in
    // 1.0-1.2s, targets 3.7]; braked, stopped in a couple of seconds. Moved
    // toward the speed asked as measured aboard an ATR 72: up from beta in
    // some 2.5s, critically damped, at no more than npRise; down from
    // take-off to the climb's in a straight glide at npGlide [M: targets 4.1]
    // - its control limiting the change [P: BEA] - and into beta, nothing
    // governing it, in a second and a half [I: atr72-sources 14.7].
    for (int g = 0; g < 2; g++) {
        bool braked = g == 1 && s.apu && p.hotel;
        bool free = s.running && !braked && q.spool[g] > 0.97f;
        q.pitch[g] = clampf(q.pitch[g] + (free ? 0.25f : -0.9f) * SLOW_S, 0, 1);
        float &n = q.np[g], &rate = q.npRate[g];
        if (braked) {
            n *= BRAKE;
            rate = 0;
        } else if (q.lit[g] < 0.5f && q.pitch[g] < 0.01f) {
            n *= SPIN_DOWN;  // shut down, feathered: unheard
            rate = 0;
        } else {
            float want = ask * q.pitch[g] + 0.25f * (1 - q.pitch[g]);
            if (want > n) {
                const float w = PR_NP_RISE_W;
                rate += (w * w * (want - n) - 2 * w * rate) * SLOW_S;
                rate = clampf(rate, 0, 1.25f * p.npRise);
                n = std::min(want, n + rate * SLOW_S);
            } else {
                rate = 0;
                float glide = q.pitch[g] < 0.99f ? 0.15f : q.governed > 0.5f && !beta ? p.npGlide : 0.2f;
                n = std::max(want, n - glide * SLOW_S);
            }
        }
    }
    q.split = p.splitBeta * (1 - q.governed);
}

// Each 32ms: its surges and the air's flicker; its harmonics', the steady
// line's, the comb's, the gearbox's, the whines' and the haystack's levels,
// widths and pitches as heard; its broadband's bands.
__attribute__((noinline)) void Synth::propSlowStep() {
    propDoing();
    const Sound &s = sound_;
    const PropData &p = propData(s.engine);
    Prop &q = prop_;
    float d = expFast(logDistance_), km = d * 0.001f;
    // Its loudness in the air by its power (PR_FLIGHT_POWER_DB), and its
    // tones' low-pass far off (PR_CUT_*): off nearer than 3km.
    gain_ *= dbAmp(PR_FLIGHT_POWER_DB * (1 - q.stat) * lg10(std::max(0.05f, q.torque)));
    q.cutA = km > PR_CUT_FROM_KM
                 ? 1 - expf(-2 * PI / RATE * std::max(400.0f, 8000 * exp2Fast(-PR_CUT_OCTAVES_KM * (km - PR_CUT_FROM_KM))))
                 : 0.0f;
    float th = q.aspect;
    // The surges, each a quick part and a slow part, and each band's shimmer
    // and the air's flicker - the airliner's, run here as its bandsStep()
    // would - and the harmonics' own shared surge.
    Rng rng{slow_};
    float *surge = surge_;
    for (int c = 0; c < SOURCES; c++) {
        const float *k = surgeKeep_[c];
        surgeQuick_[c] = surgeQuick_[c] * k[0] + k[1] * rng.gauss();
        surgeSlow_[c] = surgeSlow_[c] * k[2] + k[3] * rng.gauss();
        float n = k[4] * surgeSlow_[c] + PR_SURGE_QUICK_W[c] * surgeQuick_[c];
        float np = surgeNp_[c];
        if (c == OF_FAN) {
            // the propellers' rush: the more the quicker the bearing to it
            // turns - its unsteady loading's lumpy directivity [P: RP-1258,
            // "lobes ... unrelated to the number of blades"] sweeping by as it
            // passes, still far off, an approach 1.5-2km out churning
            // 0.9-1.2dB where it churns 1.5-2.7 passing [M: targets 3.1]; more
            // at low power, a climb's 1.2-2.1 [M: targets 0.7]; standing, in
            // the still air of the ground, steadier
            float turning = across_ / std::max(50.0f, d);  // radians a second
            np = PR_RUSH_FAR_NP + (np - PR_RUSH_FAR_NP) * smoothstep(0.008f, 0.05f, turning);
            np += PR_RUSH_LOW_NP * (1 - smoothstep(0.25f, 0.85f, q.torque));
            np += (PR_RUSH_STILL_NP - np) * q.stat;
        }
        surge[c] = exp2Fast(2.88539008f * np * (n - np));
    }
    for (int b = 0; b < BANDS; b++) {
        shimmerQuick_[b] = shimmerQuick_[b] * shimmerKeep_[0] + shimmerKeep_[1] * rng.gauss();
        shimmerSlow_[b] = shimmerSlow_[b] * shimmerKeep_[2] + shimmerKeep_[3] * rng.gauss();
        shimmer_[b] = SHIMMER_QW * shimmerQuick_[b] + sqrtf(1 - SHIMMER_QW * SHIMMER_QW) * shimmerSlow_[b];
    }
    float kf = expFast(-SLOW_S * (across_ * FLICKER_ACROSS + FLICKER_BASE) * (1.0f / 5));
    flickerA_ = flickerA_ * kf + sqrtf(1 - kf * kf) * rng.gauss();
    flicker_ = flicker_ * kf + (1 - kf) * flickerA_;
    float flickerN = flickerN_ = flicker_ * (1 + kf) * (1 / sqrtf(1 + kf * kf));
    float flickerNp = PR_FLICKER * std::min(6.0f, FLICKER_DB * sqrtf(km)) * (1 / 8.686f);
    static const float TQ = expf(-SLOW_S / PR_TONE_SURGE_Q_S), TS = expf(-SLOW_S / PR_TONE_SURGE_S_S);
    q.surgeQ = q.surgeQ * TQ + sqrtf(1 - TQ * TQ) * rng.gauss();
    q.surgeS = q.surgeS * TS + sqrtf(1 - TS * TS) * rng.gauss();
    // Ungoverned in beta, each propeller's speed wandering on its own, the
    // right off the left by its row's split; governed, the two hunting
    // together; each gas generator's whine wandering - slow processes, each
    // a unit spread.
    static const float WANDER = expf(-SLOW_S / PR_WANDER_S), HUNT = expf(-SLOW_S / PR_HUNT_S),
                       WHINE = expf(-SLOW_S / PR_WHINE_S);
    static const float WANDER_IN = sqrtf(1 - WANDER * WANDER), HUNT_IN = sqrtf(1 - HUNT * HUNT),
                       WHINE_IN = sqrtf(1 - WHINE * WHINE);
    for (int g = 0; g < 2; g++) {
        q.wander[g] = q.wander[g] * WANDER + WANDER_IN * 0.57735f * rng.gauss();
        q.whineX[g] = q.whineX[g] * WHINE + WHINE_IN * 0.57735f * rng.gauss();
    }
    q.hunt = q.hunt * HUNT + HUNT_IN * 0.57735f * rng.gauss();
    slow_ = rng.s;
    float toneSurgeDb = 8.68589f * PR_TONE_SURGE_NP * (0.6f * q.surgeQ + 0.8f * q.surgeS - PR_TONE_SURGE_NP);

    // What its propellers are doing: each one's speed as heard - its
    // governor's hunting governed, its own wander in beta - and how far its
    // blades bite, 0 feathered or standing still to 1; how static it is, how
    // governed, how far in reverse.
    float gv = q.governed, st = q.stat, rv = q.reverse;
    float np[2], bite[2];
    for (int g = 0; g < 2; g++) {
        float drift = gv * PR_HUNT * q.hunt + (1 - gv) * (PR_WANDER * q.wander[g] + (g == 1 ? p.splitBeta : 0.0f));
        np[g] = q.np[g] * (1 + drift);
        bite[g] = smoothstep(0, 0.25f, q.pitch[g]) * smoothstep(0.08f, 0.45f, q.np[g]);
    }
    // In reverse the harmonics no stronger for the reverse's power - its
    // blades stalled, their noise mostly a roar - but as an approach's [I:
    // the CAA's turboprops 1.4dB louder by SEL landing with reverse above
    // idle, against an A320's 10.9, atr72-sources 6].
    float torque = q.torque + (std::min(q.torque, p.approachPower) - q.torque) * rv;
    float high = smoothstep(0.25f, 0.85f, q.torque);
    float bpf = p.blades * (s.engine == Engine::GENERIC ? s.bladeHz / p.blades : p.rpm100 / 60);  // at 100%
    // The harmonics' envelope now: clean flight's, by the power from the
    // approach's to the climb's; standing, beta's or take-off's as governed;
    // between, as the static share; and reverse's as its blades turn.
    float env[8], slope;
    {
        float sF = PR_SLOPE_FLIGHT + (PR_SLOPE_CLIMB - PR_SLOPE_FLIGHT) * high;
        float sS = PR_SLOPE_BETA + (PR_SLOPE_TAKEOFF - PR_SLOPE_BETA) * gv;
        slope = sF + (sS - sF) * std::max(st, q.ground * 0.5f * high * gv);
        slope += (PR_SLOPE_REVERSE - slope) * rv;
        float rich = std::max(st, q.ground * 0.5f * high * gv);
        for (int k = 0; k < 8; k++) {
            float f = PR_E_FLIGHT[k] + (PR_E_CLIMB[k] - PR_E_FLIGHT[k]) * high;
            float t = PR_E_BETA[k] + (PR_E_TAKEOFF[k] - PR_E_BETA[k]) * gv;
            float v = f + (t - f) * rich;
            env[k] = v + (PR_E_REVERSE[k] - v) * rv;
        }
    }
    // The way's figures for the tones: the ground's echo, its lateness from
    // each propeller and from between them, how coherent; the flicker's
    // corner; the widening far off.
    float gamma = echoGamma_, mean = 1 + gamma * gamma;
    float lateOf[3] = {echoLate_[0] / RATE, echoLate_[1] / RATE, 0.5f * (echoLate_[0] + echoLate_[1]) / RATE};
    float coherent = 1 / clampf(std::min(2000 * sqrtf(150 / std::max(10.0f, d)),
                                         SOUND_MS * d / (16 * std::max(1.0f, s.earUpM))),
                                400, 3000);
    float cornerLog = log2Fast(9000 / sqrtf(d)), widen = 3.5f * km;
    // A tone's loudness as heard: its level, the air's absorption at its
    // pitch, its flicker over the corner, and the echo off the ground as a
    // sum - held within 6dB of the powers' sum at 300Hz and under, 2dB from
    // 1.5kHz, between as the log of the pitch - as an RMS.
    auto heardAmp = [&](float heard, float db, int from) {
        float lh = log2Fast(heard);
        float o = clampf(lh - 5.97728f, 0, 6.999f);  // octaves from 63Hz
        int i = (int)o;
        float absorbs = exp2Fast(ABSORPTION_LOG2[i] + (ABSORPTION_LOG2[i + 1] - ABSORPTION_LOG2[i]) * (o - i));
        float u = clamp01(lh - cornerLog + 0.5f), w = u * u * (3 - 2 * u) * flickerNp;  // its shelf an octave wide
        float amp = dbAmp(db - absorbs * km + 8.68589f * (w * flickerN - w * w));
        if (heard > TOP_HZ - TOP_FADE_HZ) {
            amp *= (TOP_HZ - heard) * (1 / TOP_FADE_HZ);
        }
        if (gamma > 0) {
            float x = heard * coherent, x2 = x * x, coh = 1 / (1 + x2 + 0.5f * x2 * x2), gc = gamma * coh;
            float turns = heard * lateOf[from] + 0.25f;  // its cosine, as a sine a quarter turn on
            turns -= (float)(int)turns;
            float cs = sinTurn(turns > 0.5f ? turns - 1 : turns);
            float pw = 1 + gc * gc + 2 * gc * cs + gamma * gamma * (1 - coh * coh);
            float held = 6 - 4 * clamp01((lh - 8.22882f) * (1 / 2.32193f));  // dB: 6 to 300Hz, 2 from 1.5kHz
            float least = dbPower(-held);
            amp *= sqrtf(clampf(pw / mean, least, 1 / least) * mean);
        }
        return amp;
    };
    // A resonance: off; or its pitch as heard, its loudness as an RMS, its
    // width, its noise - set as the airliner's tonesStep() sets one.
    auto off = [](Tone &t) {
        t.in = 0;
        t.dr = 1, t.di = 0;
        t.on = t.on && (t.zr * t.zr + t.zi * t.zi > 1e-14f);
        t.turn = t.on ? t.turn : 0;
    };
    auto tune = [&](Tone &t, float heard, float r) {
        // sinCosTurns' pairs a unit to a few parts in a million: not
        // renormalised, as the airliner's tones are
        float turn = heard * (1.0f / RATE), was = t.on && t.turn > 0 ? t.turn : turn;
        float sn, cs;
        sinCosTurns(was, sn, cs);
        t.cr = cs * r;
        t.ci = sn * r;
        sinCosTurns((turn - was) * (1.0f / SLOW), sn, cs);
        t.dr = cs;
        t.di = sn;
        t.turn = turn;
        t.hz = heard;
        t.on = true;
    };
    auto set = [&](Tone &t, float hz, float db, float width, int stream, int from) {
        float heard = hz * q.doppler;
        if (heard < 30 || heard > TOP_HZ || db < -110) {
            off(t);
            return;
        }
        float amp = heardAmp(heard, db, from);
        if (q.cutA > 0) {  // what the far low-pass's two stages take at its pitch, given back
            float sn, cs, k = 1 - q.cutA;
            sinCosTurns(heard * (1.0f / RATE), sn, cs);
            amp *= (1 + k * k - 2 * k * cs) / (q.cutA * q.cutA);
        }
        float x = PI * width / RATE, r = 1 - x * (1 - x * (0.5f - x * (1.0f / 6)));
        tune(t, heard, r);
        t.in = amp * sqrtf(6 * (1 - r * r));
        t.stream = (uint8_t)stream;
        t.wide = width > 20;
    };
    // The propellers' harmonics. Each propeller's level: its power and speed
    // (PR_TORQUE_LAW, PR_SPEED_LAW), how far its blades bite, the two
    // propellers' together PR_LEVEL_DB, each alone 3dB under; its harmonics'
    // shared surge.
    float levelDb[2], lobeFlat = 1 - PR_STATIC_LOBE * st;
    for (int g = 0; g < 2; g++) {
        levelDb[g] = PR_LEVEL_DB - 3.0103f + (1 - gv) * PR_BETA_TONE_DB + PR_TORQUE_LAW * lg10(std::max(0.05f, torque)) +
                     PR_SPEED_LAW * lg10(std::max(0.05f, np[g])) + 20 * lg10(std::max(1e-7f, bite[g])) + toneSurgeDb;
    }
    float tone12 = (lobe(PROP_TONE, th) + (lobe(PROP_TONE_POWER, th) - lobe(PROP_TONE, th)) * high) * lobeFlat;
    float tone3 = (lobe(PROP_HIGH, th) + (lobe(PROP_HIGH_POWER, th) - lobe(PROP_HIGH, th)) * high) * lobeFlat;
    // Each harmonic's level by its envelope and its lobe, and the lift for the
    // speaker (PR_LIFT): over the first four heard at 300Hz or more, up to the
    // sixth, each raised to the first's level less 10dB an octave of harmonic
    // number if under it, never past less 3 - the first's level as this
    // harmonic's lobe would have it: the series' shape put back, not a
    // directivity the physics doesn't have.
    auto harmonicDb = [&](int g, int k, float heardBpf, int &lifted) {
        float lk = levelDb[g] + env[k - 1] + (k <= 2 ? tone12 : tone3);
        if (PR_LIFT > 0 && k >= 2 && lifted < 4 && k * heardBpf >= 300) {
            lifted++;
            static const float OCTAVES[8] = {0, 0, 1, 1.5849625f, 2, 2.3219281f, 2.5849625f, 2.8073549f};
            float l1 = levelDb[g] + env[0] + (k <= 2 ? tone12 : tone3), oct = OCTAVES[k];
            float floor = l1 - 10 * oct, ceiling = l1 - 3 * oct;
            if (lk < floor) {
                lk += PR_LIFT * (std::min(floor, ceiling) - lk);
            }
        }
        return lk;
    };
    // Governed - in the air and on its take-off roll - the two propellers on
    // one line each harmonic: one resonance each, both propellers' power;
    // the first two's steady part - K of their power in K + 1 - the two
    // propellers' sum as the ear hears it, their path's difference, 8.10m
    // apart, and the synchrophaser's phase, its lobes sweeping by as the
    // bearing changes [I: turboprop-methods 5], on their level. In beta each
    // propeller its own speed: two resonances each harmonic heard at 300Hz
    // or more, each on its own noise; under it, where the speaker plays
    // nothing, one for both, as wide as their speeds' spread.
    float k1 = PR_K1 * gv, k2 = PR_K2 * gv;
    float pathTurns = (engineM_[0] - engineM_[1]) * (1 / SOUND_MS);
    float widthStatic = 1 + PR_WIDTH_STATIC * st;
    bool apart = gv < 0.98f && rv < 0.5f;  // in reverse both near their top speed: as one, widened
    float spread = (1 - gv) * (p.splitBeta + PR_WANDER);  // the two's speeds apart in beta, a share
    int lifted[2] = {0, 0};
    for (int k = 1; k <= 6; k++) {
        float width = ((k == 1 ? PR_WIDTH_1 : k == 2 ? PR_WIDTH_2 : PR_WIDTH_3 + PR_WIDTH_BY * (k - 3)) * widthStatic +
                       widen + PR_WIDTH_REVERSE * rv) *
                      (1 + 0.2f * (1 - gv));
        float heardL = k * bpf * np[0] * q.doppler;
        Tone &t = propTones_[PR_HARM + k - 1];
        if (apart && heardL >= 300) {
            for (int g = 0; g < 2; g++) {
                float bpfG = bpf * np[g];
                set(g == 0 ? t : propTones_[PR_HARM_R + k - 2], k * bpfG, harmonicDb(g, k, bpfG * q.doppler, lifted[g]),
                    width, g, g);
            }
            continue;
        }
        if (k >= 2) {
            off(propTones_[PR_HARM_R + k - 2]);
        }
        float bpfM = 0.5f * bpf * (np[0] + np[1]);
        float db = harmonicDb(0, k, bpfM * q.doppler, lifted[0]) + 3.0103f;  // both propellers' power, as one
        float kk = k == 1 ? k1 : k == 2 ? k2 : 0.0f;
        float pair = 1;  // the two propellers' steady parts' sum as heard, against their powers'
        if (kk > 0.01f) {
            float phase = k * (bpfM * q.doppler * pathTurns + q.psi);
            phase -= (float)(int)phase;
            float cs = sinTurn((phase > 0.5f ? phase - 1 : phase) * 0.5f + 0.25f);  // cos(pi x)
            // never quite equal at the ear nor exactly opposed - the fuselage
            // between them, their blades' small differences [I] - so never
            // cancelling to nothing
            pair = 0.3f + 1.4f * cs * cs;
        }
        if (k == 1) {
            // the fundamental's steady part its own line - a resonance with
            // no noise, its pointer's radius moved to its loudness over the
            // next 32ms by its own shrinking or growing - its noise-driven part
            // the resonance; so its phase holds through the resonance's fades,
            // as a governed fundamental's does [M: its pitch wandering 0.13-0.83%,
            // targets 4.4]
            Tone &u = propTones_[PR_STEADY];
            float heard = bpfM * q.doppler, want = kk > 0.01f && heard >= 30
                                                       ? 1.41421356f * heardAmp(heard, db + 10 * lg10(kk / (1 + kk) * pair + 1e-6f), 2)
                                                       : 0.0f;
            float now = sqrtf(u.zr * u.zr + u.zi * u.zi);
            if (want > 1e-9f || now > 1e-7f) {
                if (!u.on || now < 1e-9f) {
                    u.zr = want, u.zi = 0;
                    now = std::max(want, 1e-9f);
                }
                float r = want > 1e-9f ? clampf(exp2Fast(log2Fast(want / now) * (1.0f / (SLOW * STEP))), 0.996f, 1.004f)
                                       : 0.996f;
                tune(u, heard, r);
                u.in = 0;
                u.stream = 0;
            } else {
                u = Tone();
            }
            db -= 10 * lg10(1 + kk);  // the resonance the rest
        } else if (kk > 0.01f) {
            db += 10 * lg10(std::max(1e-3f, (kk * pair + 1) / (kk + 1)));
        }
        set(t, k * bpfM, db, width + k * bpfM * spread, 0, apart ? 0 : 2);
    }
    // The gearbox's line, locked to the left propeller's turning (both, as
    // one, governed) - PR_GEAR_A_DB under its fundamental on the approach -
    // as the root of the power and the speed's 1.5th power, wider in beta by
    // the two propellers' split, loudest ahead. (Its row's second line,
    // gearB, heard aboard only, is not played.)
    {
        float shaft = (s.engine == Engine::GENERIC ? s.bladeHz / p.blades : p.rpm100 / 60) * np[0];
        float biting = 0.5f * (bite[0] * bite[0] + bite[1] * bite[1]);
        float db = PR_LEVEL_DB + PR_TORQUE_LAW * lg10(0.22f) + 5 * lg10(std::max(0.05f, torque / 0.22f)) +
                   30 * lg10(std::max(0.05f, np[0])) + lobe(GEARBOX, th) + 10 * lg10(std::max(1e-8f, biting));
        float width = PR_GEAR_WIDTH + widen + shaft * 48 * q.split;
        if (p.gearA > 0) {
            set(propTones_[PR_GEAR], p.gearA * shaft, db + PR_GEAR_A_DB, width, 1, 2);
        } else {
            off(propTones_[PR_GEAR]);
        }
    }
    // The gas generators' whines - each engine's line at its own speed, a
    // couple of percent from the other's and wandering - as the speed's
    // fourth power, from the starter's first turn; and the haystack about
    // them, wide, through the air's muffling with the broadband.
    float whineHz = p.whineHz > 0 ? p.whineHz : 5682;
    float nhMean = 0.5f * (q.nh[0] + q.nh[1]), hay = 0;
    for (int g = 0; g < 2; g++) {
        float nh = q.nh[g];
        float db = nh > 0.02f ? PR_LEVEL_DB + PR_WHINE_DB + 40 * lg10(nh / 0.85f) + lobe(WHINES, th) +
                                    10 * lg10(std::max(1e-3f, 0.2f + 0.8f * q.lit[g]))
                              : -200;
        hay += nh > 0.02f ? dbPower(40 * lg10(nh / 0.85f)) * (0.2f + 0.8f * q.lit[g]) : 0.0f;
        float hz = whineHz * (nh / 0.85f) * (1 + q.whineBy[g] + PR_WHINE_WANDER * q.whineX[g]);
        set(propTones_[PR_WHINE + g], hz, db, 2.0f + widen, g, g);
    }
    // The haystack, as wide as 250Hz, with the narrow tones after the
    // muffling and the echo: its air's loss exact at its pitch; in the air
    // only, heard about an approach's pass [M: turboprop-methods 1.7].
    {
        float db = hay > 1e-6f && q.ground < 0.5f ? PR_LEVEL_DB + PR_HAY_DB + 10 * lg10(0.5f * hay) + lobe(HAYSTACK, th)
                                                  : -200.0f;
        Tone &t = propTones_[PR_HAY];
        float heard = PR_HAY_HZ * (nhMean / 0.85f) * q.doppler;
        if (db < -110 || heard > TOP_HZ || heardAmp(heard, db, 2) < 1e-6f) {
            off(t);
        } else {
            set(t, PR_HAY_HZ * (nhMean / 0.85f), db, PR_HAY_WIDTH, 0, 2);
        }
    }

    // The broadband: each band as loud as its sources in it - each source's
    // spectrum at the frequency the band is heard at, the Doppler moving it,
    // its loudness by what the propellers and engines are doing, its lobe,
    // its surges - then the band's shimmer, flicker and the air's loss, as the
    // airliner's bandsStep() has them.
    float biteSum = 0.5f * (bite[0] * bite[0] + bite[1] * bite[1]);
    float npMean = std::max(0.05f, 0.5f * (np[0] + np[1]));
    float biteDb = 10 * lg10(std::max(1e-8f, biteSum));
    float washLobe = lobe(PROP_BROAD, th) + st * high * gv * (lobe(PROPWASH, th) - lobe(PROP_BROAD, th));
    float loadDb = PR_LEVEL_DB + PR_LOAD_DB + 30 * lg10(std::max(0.03f, torque)) + 30 * lg10(npMean) +
                   PR_LOAD_STATIC_DB * st * high + washLobe + biteDb + (1 - gv) * PR_BETA_RUSH_DB;
    float edgeDb = PR_LEVEL_DB + PR_EDGE_DB + 50 * lg10(npMean) + 3 * lg10(std::max(0.05f, torque)) +
                   lobe(PROP_BROAD, th) + biteDb + (1 - gv) * PR_BETA_RUSH_DB;
    float lowDb = PR_LEVEL_DB + PR_LOW_DB + 50 * lg10(npMean) + 3 * lg10(std::max(0.05f, torque)) +
                  lobe(PROP_BROAD, th) + biteDb + (1 - gv) * PR_BETA_RUSH_DB;
    float exhaust = 0, intakes = 0;
    for (int g = 0; g < 2; g++) {
        float nh = std::max(1e-3f, q.nh[g] / 0.85f);
        exhaust += dbPower(30 * lg10(nh)) * (q.lit[g] * q.lit[g] + 0.02f * q.spool[g]);
        intakes += dbPower(40 * lg10(nh)) * (0.2f + 0.8f * q.lit[g]) * (q.nh[g] > 0.02f);
    }
    float exhaustDb = exhaust > 1e-9f ? PR_LEVEL_DB + PR_EXHAUST_DB + 10 * lg10(0.5f * exhaust) + lobe(EXHAUST, th) : -200;
    float intakeDb = intakes > 1e-9f ? PR_LEVEL_DB + PR_GG_DB + 10 * lg10(0.5f * intakes) + lobe(INTAKES, th) : -200;
    float revDb = rv > 0.001f ? PR_LEVEL_DB + PR_REVERSE_DB + 20 * lg10(rv) + 10 * lg10(std::max(0.05f, q.torque / 0.7f)) +
                                    lobe(REVERSE, th) + biteDb
                              : -200;
    // Its airframe as the A320's - see bandsStep() - PR_FRAME_DB under; its
    // tyres rolling.
    float v = airspeed_, vr = std::max(0.05f, v / 70);
    float thin = -0.00102f * std::max(0.0f, s.sourceUpM);
    float rush = 60 * lg10(std::min(vr, 1.6f)) + thin + PR_FRAME_DB;
    float airframe = lobe(AIRFRAME, th);
    float gearDb = gear_ > 0.001f ? -29.5f + rush + 10 * lg10(gear_) + airframe + 1.5f * below_ -
                                        5 * std::max(0.0f, -below_)
                                  : -200;
    float flapDb = flaps_ > 0.001f ? -32 + rush + 10 * lg10(flaps_) + lobe(FLAPS, th) : -200;
    float cleanDb = -36 + rush + airframe;
    float roll = s.rollingMs;
    float rollDb = roll > 0.5f ? -32 + 30 * lg10(std::max(0.05f, roll / 70)) + PR_FRAME_DB : -200;
    bool moving = v > 20 || roll > 20;
    float logDoppler = log2Fast(q.doppler);
    float loadPeak = log2Fast(PR_LOAD_HZ * 0.001f * npMean), edgePeak = log2Fast(PR_EDGE_HZ * 0.001f * npMean),
          lowPeak = log2Fast(PR_LOW_HZ * 0.001f * npMean);
    constexpr float L5000 = 2.32193f, L180 = -2.47393f, L600 = -0.736966f, L700 = -0.514573f, L1250 = 0.321928f,
                    LEXHAUST = -1.32193f, LGG = 1.13750f, LREVERSE = -0.666576f;
    float surgeEdge = surge[OF_FAN];  // the loading rush's own: the whole band moving with it
    float airSin, airCos;
    sinCosTurns(0.5f * air_.hz / RATE, airSin, airCos);
    float airCot = airCos / std::max(1e-6f, airSin);
    float shimmerFar = SHIMMER_FAR_NP * std::min(1.0f, sqrtf(km)) + SHIMMER_PASS_NP * smoothstep(10, 80, across_);
    float cornerBand = log2Fast(9 / sqrtf(d));
    float rushOf[BANDS];  // each band's propellers' rush, unsurged: the comb's measure
#ifdef ENGINE_NOISE_DEBUG
    static int debugIn = 0;
    if (--debugIn < 0) {
        debugIn = 31;
        fprintf(stderr, "t %.2f d %.0f asp %.2f np %.3f/%.3f gv %.2f st %.2f rv %.2f tq %.2f nh %.2f/%.2f | lvl %.1f "
                        "load %.1f edge %.1f exh %.1f gg %.1f rev %.1f gear %.1f | dop %.3f\n",
                clock_ / (float)RATE, d, th, np[0], np[1], gv, st, rv, torque, q.nh[0], q.nh[1], levelDb[0], loadDb,
                edgeDb, exhaustDb, intakeDb, revDb, gearDb, q.doppler);
    }
#endif
    // Each band - the bank's eight, then the low band, `b` BANDS - as loud as
    // its sources: the band's frequency, octaves from 1kHz, its tangent of pi
    // x it over the rate and the air's loss there, dB a kilometre.
    static const float LOW_LOG = log2f(PR_LOW_BAND_HZ * 0.001f), LOW_TAN = tanf(PI * PR_LOW_BAND_HZ / RATE),
                       LOW_AIR = absorptionDbKm(PR_LOW_BAND_HZ);
    for (int b = 0; b <= BANDS; b++) {
        float bandLog = b < BANDS ? bandLog_[b] : LOW_LOG;
        float lf = bandLog - logDoppler;  // octaves from 1kHz where it left the source
        float xl = lf - loadPeak, xe = lf - edgePeak, xx = lf - LEXHAUST, xg = lf - LGG, xo = lf - lowPeak;
        float pl = dbPower(loadDb + (xl < 0 ? 3 * xl : -6 * xl));
        float pe = dbPower(edgeDb + (xe < 0 ? 12 * xe : -2 * xe)) + dbPower(lowDb + (xo < 0 ? 3 * xo : -9 * xo));
        float p = pl * surge[OF_FAN] + pe * surgeEdge;
        float turbulent = pl + pe;
        if (b < BANDS) {
            rushOf[b] = pl + pe;
        }
        if (exhaustDb > -150) {
            p += dbPower(exhaustDb + (xx < 0 ? 4 * xx : -6 * xx)) * surge[OF_JET];
        }
        if (intakeDb > -150) {
            p += dbPower(intakeDb + (xg < 0 ? 5 * xg : -6 * xg)) * surge[OF_CORE];
        }
        if (moving) {  // its airframe's rush and its tyres' roll, 50dB down and more under 20m/s: not reckoned
            float xw = lf - L600, rushing = dbPower(cleanDb - std::min(25.0f, 4 * xw * xw));
            if (gearDb > -150) {
                rushing += dbPower(gearDb - 3 * std::max(0.0f, lf - L5000) - 6 * std::max(0.0f, L180 - lf));
            }
            if (flapDb > -150) {
                float xp = lf - L1250;
                rushing += dbPower(flapDb - std::min(25.0f, 5 * xp * xp));
            }
            if (rollDb > -150) {
                float xt = lf - L700;
                rushing += dbPower(rollDb - std::min(25.0f, 5 * xt * xt));
            }
            p += rushing * surge[OF_AIRFRAME];
            turbulent += 0.6f * rushing;
        }
        if (revDb > -150) {
            float xr = lf - LREVERSE, pr = dbPower(revDb + (xr < 0 ? 6 * xr : -11 * xr));
            p += pr * surge[OF_REVERSER];
            turbulent += pr;
        }
        float fl = flickerNp * smoothstep(cornerBand - 0.5f, cornerBand + 0.5f, bandLog);
        float sh = PR_SHIMMER * (SHIMMER_NP + SHIMMER_TURBULENT_NP * clamp01(turbulent / std::max(1e-12f, p)) + shimmerFar);
        float w = (b < BANDS ? bandTan_[b] : LOW_TAN) * airCot, w2 = w * w;
        float lowPassDb = 4.34294482f * 0.693147181f * log2Fast((1 - w2) * (1 - w2) + w2 * (1 / (0.6f * 0.6f)));
        float airDb = clampf(lowPassDb - (b < BANDS ? bandAirDbKm_[b] : LOW_AIR) * km, PR_AIR_FLOOR_DB, 15);
        float ln = 2 * sh * (shimmer_[b < BANDS ? b : 0] - sh) + 2 * fl * (flickerN - fl) + 0.230258509f * airDb;
        p *= exp2Fast(1.44269504f * ln);
        if (b < BANDS) {
            bandBy_[b] = (sqrtf(p) - bandGain_[b]) * (1.0f / (SLOW * STEP));
        } else {
            q.lowBy = (sqrtf(p) - q.lowGain) * (1.0f / (SLOW * STEP));
        }
    }
    // The comb, from the seventh harmonic up - both propellers' as one, at
    // their mean speed, its lines as much wider in beta as their speeds are
    // apart there - a line at every harmonic, each fading on its own: its
    // delay one blade's passing as heard, its lines as wide as PR_COMB_WIDTH
    // makes them - twice that standing - its noise shaped to rise to the
    // seventh harmonic and fall 6dB an octave over it, pushed hard enough
    // for the seventh's power to be the envelope's. It goes through the
    // air's muffling with the broadband, as the airliner's buzz-saw does,
    // which takes the air's share of it. Worked through only while its
    // seventh line is within 10dB of the propellers' rush in the bank's band
    // about it, a band hundreds of hertz wide - so a line still clear of the
    // rush in the few hertz beside it: on the ground and at take-off power;
    // in clean flight, where the harmonics from the seventh up are lost in
    // the rush [M: turboprop-methods 1.3], not at all.
    {
        Comb &c = comb_[0];
        float bpfM = 0.5f * bpf * (np[0] + np[1]), hz = bpfM * q.doppler;
        int lifted = 4;
        float db = harmonicDb(0, 7, hz, lifted) + 3.0103f;
        float amp = dbAmp(db) * sqrtf(surge[OF_BUZZ]) * expFast(flickerNp * flickerN - flickerNp * flickerNp);
        // the propellers' rush in the band about its seventh line
        float at = clampf(1.5f * log2Fast(std::max(250.0f, 7 * hz) * 0.004f), 0, 6.999f);
        int band = (int)at;
        float rush = rushOf[band] + (rushOf[band + 1] - rushOf[band]) * (at - band);
        if (amp * amp < 0.1f * rush || hz < RATE / (float)(COMB - 8) || 7 * hz > 0.45f * RATE) {
            c.in = 0;
            c.lateBy = 0;
            if (combOn_[0] && c.y1 * c.y1 + c.y2 * c.y2 < 1e-14f) {
                combOn_[0] = false;
                memset(c.line, 0, sizeof(c.line));
                c.y1 = c.y2 = c.lo = c.hi = 0;
            }
        } else {
            bool was = combOn_[0];
            combOn_[0] = true;
            float late = RATE / hz - 1;
            if (!was || fabsf(late - c.late) > 40) {
                c.late = late;
            }
            c.lateBy = (late - c.late) * (1.0f / SLOW);
            float width = PR_COMB_WIDTH * (1 + st) + widen + PR_WIDTH_REVERSE * rv + 7 * bpfM * spread;
            c.g = expFast(-PI * width / hz);
            // its noise: the band between two one-pole low-passes' corners, at
            // 5.5 and 7 times the blades' passing, as the airliner's buzz-saw's
            // is shaped; and what power of it reaches the seventh harmonic's
            // band, a blade's passing wide - the band's gain there, the two
            // one-poles' responses taken one from the other
            float kl = 1 - expFast(-2 * PI * 5.5f * hz / RATE), kh = 1 - expFast(-2 * PI * 7 * hz / RATE);
            c.kLo = kl;
            c.kHi = kh;
            float sn, cs;
            sinCosTurns(7 * hz / RATE, sn, cs);
            auto pole = [&](float k, float &re, float &im) {  // k / (1 - (1 - k) e^-iw)
                float a = 1 - k, dr = 1 - a * cs, di = a * sn, m = k / (dr * dr + di * di);
                re = m * dr, im = -m * di;
            };
            float hr, hi, lr, li;
            pole(kh, hr, hi);
            pole(kl, lr, li);
            float gain = (hr - lr) * (hr - lr) + (hi - li) * (hi - li);
            float power = gain * (1.0f / 3) * 2 * hz / RATE;
            c.in = amp * sqrtf((1 - c.g * c.g) / std::max(1e-9f, power));
        }
    }
    tonesOn_ = 0;
    for (const Tone &t : propTones_) {
        tonesOn_ += t.on;
    }
}

// The step's samples before the air's muffling and the echo: the bank and the
// low band under it; the comb; the nose wheel's thumps; the tyres' squeal.
__attribute__((noinline)) void Synth::propRender(float *mix, const float *lateA, const float *lateB,
                                                 const float *noiseA, const float *noiseB, Rng &rng, int n) {
    Prop &q = prop_;
    // The bank as the airliner's, four bands at a time - its four lower with
    // the low band beside them, a fifth chain of sums the core works through
    // side by side with theirs, on the bank's second noise.
    const LowBand &lb = lowBand();
    for (int b = 0; b < BANDS; b += 4) {
        const float *B0 = bandB0_ + b, *A1 = bandA1_ + b, *A2 = bandA2_ + b;
        float b0 = B0[0], b1 = B0[1], b2 = B0[2], b3 = B0[3];
        float p0 = A1[0], p1 = A1[1], p2 = A1[2], p3 = A1[3];
        float q0 = A2[0], q1 = A2[1], q2 = A2[2], q3 = A2[3];
        float y0 = bandY1_[b], y1 = bandY1_[b + 1], y2 = bandY1_[b + 2], y3 = bandY1_[b + 3];
        float z0 = bandY2_[b], z1 = bandY2_[b + 1], z2 = bandY2_[b + 2], z3 = bandY2_[b + 3];
        float v0 = bandZ1_[b], v1 = bandZ1_[b + 1], v2 = bandZ1_[b + 2], v3 = bandZ1_[b + 3];
        float w0 = bandZ2_[b], w1 = bandZ2_[b + 1], w2 = bandZ2_[b + 2], w3 = bandZ2_[b + 3];
        float g0 = bandGain_[b], g1 = bandGain_[b + 1], g2 = bandGain_[b + 2], g3 = bandGain_[b + 3];
        float d0 = bandBy_[b], d1 = bandBy_[b + 1], d2 = bandBy_[b + 2], d3 = bandBy_[b + 3];
        if (b == 0) {
            float lb0 = lb.b0, la1 = lb.a1, la2 = lb.a2;
            float ly1 = q.lowY1, ly2 = q.lowY2, lz1 = q.lowZ1, lz2 = q.lowZ2, lg = q.lowGain, ld = q.lowBy;
            for (int i = 0; i < n; i++) {
                float x = lateA[i], u = lateB[i];
                float u0 = (b0 * x - q0 * z0) - p0 * y0, u1 = (b1 * u - q1 * z1) - p1 * y1;
                float u2 = (b2 * x - q2 * z2) - p2 * y2, u3 = (b3 * u - q3 * z3) - p3 * y3;
                float ul = (lb0 * u - la2 * ly2) - la1 * ly1;
                z0 = y0, z1 = y1, z2 = y2, z3 = y3, ly2 = ly1;
                y0 = u0, y1 = u1, y2 = u2, y3 = u3, ly1 = ul;
                float r0 = (u0 - q0 * w0) - p0 * v0, r1 = (u1 - q1 * w1) - p1 * v1;
                float r2 = (u2 - q2 * w2) - p2 * v2, r3 = (u3 - q3 * w3) - p3 * v3;
                float rl = (ul - la2 * lz2) - la1 * lz1;
                w0 = v0, w1 = v1, w2 = v2, w3 = v3, lz2 = lz1;
                v0 = r0, v1 = r1, v2 = r2, v3 = r3, lz1 = rl;
                g0 += d0, g1 += d1, g2 += d2, g3 += d3, lg += ld;
                mix[i] += ((g0 * r0 + g1 * r1) + (g2 * r2 + g3 * r3)) + lg * rl;
            }
            q.lowY1 = fabsf(ly1) < 1e-20f ? 0 : ly1, q.lowY2 = fabsf(ly2) < 1e-20f ? 0 : ly2;
            q.lowZ1 = fabsf(lz1) < 1e-20f ? 0 : lz1, q.lowZ2 = fabsf(lz2) < 1e-20f ? 0 : lz2;
            q.lowGain = std::max(0.0f, lg);
        } else {
            for (int i = 0; i < n; i++) {
                float x = lateA[i], u = lateB[i];
                float u0 = (b0 * x - q0 * z0) - p0 * y0, u1 = (b1 * u - q1 * z1) - p1 * y1;
                float u2 = (b2 * x - q2 * z2) - p2 * y2, u3 = (b3 * u - q3 * z3) - p3 * y3;
                z0 = y0, z1 = y1, z2 = y2, z3 = y3;
                y0 = u0, y1 = u1, y2 = u2, y3 = u3;
                float r0 = (u0 - q0 * w0) - p0 * v0, r1 = (u1 - q1 * w1) - p1 * v1;
                float r2 = (u2 - q2 * w2) - p2 * v2, r3 = (u3 - q3 * w3) - p3 * v3;
                w0 = v0, w1 = v1, w2 = v2, w3 = v3;
                v0 = r0, v1 = r1, v2 = r2, v3 = r3;
                g0 += d0, g1 += d1, g2 += d2, g3 += d3;
                mix[i] += (g0 * r0 + g1 * r1) + (g2 * r2 + g3 * r3);
            }
        }
        bandY1_[b] = y0, bandY1_[b + 1] = y1, bandY1_[b + 2] = y2, bandY1_[b + 3] = y3;
        bandY2_[b] = z0, bandY2_[b + 1] = z1, bandY2_[b + 2] = z2, bandY2_[b + 3] = z3;
        bandZ1_[b] = v0, bandZ1_[b + 1] = v1, bandZ1_[b + 2] = v2, bandZ1_[b + 3] = v3;
        bandZ2_[b] = w0, bandZ2_[b + 1] = w1, bandZ2_[b + 2] = w2, bandZ2_[b + 3] = w3;
        bandGain_[b] = std::max(0.0f, g0), bandGain_[b + 1] = std::max(0.0f, g1);
        bandGain_[b + 2] = std::max(0.0f, g2), bandGain_[b + 3] = std::max(0.0f, g3);
    }
    // The comb: a delay of one blade's passing read between two samples, fed
    // back through the airliner's gentle smoothing, so its higher lines are a
    // little wider; its noise its own, drawn here, so its lines owe nothing
    // to the resonances' or the bank's; shaped as a band.
    if (combOn_[0]) {
        Comb &c = comb_[0];
        int cw = (int)c.late;
        float f = c.late - cw;
        float cg = c.g, cin = c.in, cy1 = c.y1, cy2 = c.y2, clo = c.lo, chi = c.hi, ckl = c.kLo, ckh = c.kHi;
        float *cl = c.line;
        int at = combAt_;
        constexpr int M = COMB - 1;
        for (int i = 0; i < n; i++) {
            float a = rng.noise();
            clo += ckl * (a - clo);
            chi += ckh * (a - chi);
            int pp = at - cw - 1;
            float near = cl[pp & M], far = cl[(pp - 1) & M];
            float cy = cin * (chi - clo) + cg * (near + (far - near) * f);
            cl[at] = 0.025f * (cy + cy2) + 0.95f * cy1;
            cy2 = cy1;
            cy1 = cy;
            at = (at + 1) & M;
            mix[i] += cy;
        }
        c.y1 = fabsf(cy1) < 1e-20f ? 0 : cy1, c.y2 = fabsf(cy2) < 1e-20f ? 0 : cy2;
        c.lo = clo, c.hi = chi;
        combAt_ = at;
    }
    // The nose wheel over the centreline lights, as the airliner's, lighter.
    float v = sound_.rollingMs, lights = v / 15;
    bool kick = v > 0.5f && lightPhase_ + lights * n * (1.0f / RATE) >= 1;
    if (!kick && fabsf(thumpBand_.ic1) + fabsf(thumpBand_.ic2) < 1e-6f) {
        thumpBand_.ic1 = thumpBand_.ic2 = 0;
        if (v > 0.5f) {
            lightPhase_ += lights * n * (1.0f / RATE);
        }
    } else {
        float phase = lightPhase_, ic1 = thumpBand_.ic1, ic2 = thumpBand_.ic2;
        float ta1 = thumpBand_.a1, ta2 = thumpBand_.a2, ta3 = thumpBand_.a3, tk = thumpBand_.k,
              thud = PR_THUMP * 0.5f * std::min(1.0f, v / 40);
        for (int i = 0; i < n; i++) {
            phase += lights * (1.0f / RATE);
            float x = 0;
            if (phase >= 1) {
                phase -= 1;
                x = thud * (0.8f + 0.4f * rng.uniform()) * 30;
            }
            float low, band;
            svfRun(ic1, ic2, ta1, ta2, ta3, tk, x, low, band);
            mix[i] += band * 0.012f;
        }
        lightPhase_ = phase;
        thumpBand_.ic1 = fabsf(ic1) < 1e-20f ? 0 : ic1;
        thumpBand_.ic2 = fabsf(ic2) < 1e-20f ? 0 : ic2;
    }
    // The tyres' squeal, through the air's muffling with the rest - PR_SQUEAL
    // of the airliner's.
    for (int k = 0; k < 2; k++) {
        Tone &t = tones_[SQUEAL_1 + k];
        if (!t.on) {
            continue;
        }
        const float *push = t.stream ? noiseB : noiseA;
        float pr = t.zr, pi = t.zi, pc = t.cr, ps = t.ci, pn = t.in * PR_SQUEAL;
        for (int i = 0; i < n; i++) {
            float a = (pn * push[i] - ps * pi) + pc * pr;
            pi = ps * pr + pc * pi;
            pr = a;
            mix[i] += a;
        }
        t.zr = fabsf(pr) < 1e-20f ? 0 : pr, t.zi = fabsf(pi) < 1e-20f ? 0 : pi;
    }
}

// The step's tones after the air's muffling and the echo - each has both
// worked out for its own pitch: the propellers' harmonics, the fundamental's
// steady line, the gearbox's, the whines and their haystack - four at a time,
// as the airliner's are.
__attribute__((noinline)) void Synth::propTones(float *mix, const float *noiseA, const float *noiseB, int n) {
    // Far off, the mix as it was kept, so the tones alone can go through
    // their low-pass (PR_CUT_*) after their sums.
    float cut = prop_.cutA, kept[STEP];
    if (cut > 0) {
        for (int i = 0; i < n; i++) {
            kept[i] = mix[i];
        }
    }
    propToneSums(mix, noiseA, noiseB, n);
    Prop &q = prop_;
    if (cut > 0) {
        float y1 = q.cutY1, y2 = q.cutY2;
        for (int i = 0; i < n; i++) {
            y1 += cut * ((mix[i] - kept[i]) - y1);
            y2 += cut * (y1 - y2);
            mix[i] = kept[i] + y2;
        }
        q.cutY1 = fabsf(y1) < 1e-20f ? 0 : y1, q.cutY2 = fabsf(y2) < 1e-20f ? 0 : y2;
    } else {
        q.cutY1 = q.cutY2 = 0;
    }
}

// The tones' sums, four at a time, each a resonance pushed by its noise.
__attribute__((noinline)) void Synth::propToneSums(float *mix, const float *noiseA, const float *noiseB, int n) {
    Tone *list[PROP_TONES + 1];
    int count = 0;
    for (Tone &t : propTones_) {
        if (t.on) {
            list[count++] = &t;
        }
    }
    Tone idle[1];
    if (count % 2 != 0) {
        list[count++] = &idle[0];
    }
    int fours = count & ~3;
    for (int j = 0; j < fours; j += 4) {
        Tone &p = *list[j], &q = *list[j + 1], &r = *list[j + 2], &u = *list[j + 3];
        const float *np = p.stream ? noiseB : noiseA, *nq = q.stream ? noiseB : noiseA,
                    *nr = r.stream ? noiseB : noiseA, *nu = u.stream ? noiseB : noiseA;
        float pr = p.zr, pi = p.zi, pc = p.cr, ps = p.ci, pn = p.in;
        float qr = q.zr, qi = q.zi, qc = q.cr, qs = q.ci, qn = q.in;
        float rr = r.zr, ri = r.zi, rc = r.cr, rs = r.ci, rn = r.in;
        float ur = u.zr, ui = u.zi, uc = u.cr, us = u.ci, un = u.in;
        for (int i = 0; i < n; i++) {
            float a = (pn * np[i] - ps * pi) + pc * pr, b = (qn * nq[i] - qs * qi) + qc * qr;
            float c = (rn * nr[i] - rs * ri) + rc * rr, e = (un * nu[i] - us * ui) + uc * ur;
            pi = ps * pr + pc * pi, qi = qs * qr + qc * qi, ri = rs * rr + rc * ri, ui = us * ur + uc * ui;
            pr = a, qr = b, rr = c, ur = e;
            mix[i] += (a + b) + (c + e);
        }
        p.zr = fabsf(pr) < 1e-20f ? 0 : pr, p.zi = fabsf(pi) < 1e-20f ? 0 : pi;
        q.zr = fabsf(qr) < 1e-20f ? 0 : qr, q.zi = fabsf(qi) < 1e-20f ? 0 : qi;
        r.zr = fabsf(rr) < 1e-20f ? 0 : rr, r.zi = fabsf(ri) < 1e-20f ? 0 : ri;
        u.zr = fabsf(ur) < 1e-20f ? 0 : ur, u.zi = fabsf(ui) < 1e-20f ? 0 : ui;
    }
    // And the last two, if there are, side by side.
    if (fours < count) {
        Tone &p = *list[fours], &q = *list[fours + 1];
        const float *np = p.stream ? noiseB : noiseA, *nq = q.stream ? noiseB : noiseA;
        float pr = p.zr, pi = p.zi, pc = p.cr, ps = p.ci, pn = p.in;
        float qr = q.zr, qi = q.zi, qc = q.cr, qs = q.ci, qn = q.in;
        for (int i = 0; i < n; i++) {
            float a = (pn * np[i] - ps * pi) + pc * pr, b = (qn * nq[i] - qs * qi) + qc * qr;
            pi = ps * pr + pc * pi, qi = qs * qr + qc * qi;
            pr = a, qr = b;
            mix[i] += a + b;
        }
        p.zr = fabsf(pr) < 1e-20f ? 0 : pr, p.zi = fabsf(pi) < 1e-20f ? 0 : pi;
        q.zr = fabsf(qr) < 1e-20f ? 0 : qr, q.zi = fabsf(qi) < 1e-20f ? 0 : qi;
    }
}

bool Synth::render(int16_t *out, int count) {
    float mix[STEP], lateA[STEP], lateB[STEP], noiseA[STEP], noiseB[STEP];
    float level = gain_ * fade_ * duck_ * LEVEL;
    for (int done = 0; done < count; done += STEP) {
        int n = std::min(STEP, count - done);
        update();
        bool on = sound_.kind == Kind::TURBOFAN;
        bool fighter = sound_.kind == Kind::LOW_BYPASS;
        bool prop = sound_.kind == Kind::TURBOPROP;
        // The step's noises, all from the one generator in turn: the two the
        // tones and the buzz-saws are pushed by - each engine's - and the
        // bank's two, its zeros at nought and half the rate taken out once
        // for all eight band-passes.
        Rng rng{seed_};
        float x1 = x1_, x2 = x2_, u1 = u1_, u2 = u2_;
        for (int i = 0; i < n; i++) {
            noiseA[i] = rng.noise();
            noiseB[i] = rng.noise();
            float x = rng.noise(), u = rng.noise();
            lateA[i] = x - x2;
            lateB[i] = u - u2;
            x2 = x1, x1 = x, u2 = u1, u1 = u;
            mix[i] = 0;
        }
        x1_ = x1, x2_ = x2, u1_ = u1, u2_ = u2;
        bool combs = on && (combOn_[0] || combOn_[1]);
        // The bank, four bands at a time - four chains of sums the core can
        // work through side by side - each band's band-pass and then its
        // resonance, each band's loudness carried smoothly on toward the next
        // slow step's. Neighbouring bands on the two noises, and every other
        // band on each turned over: between two bands, on the one noise and
        // the same way up, the two sounded 180 degrees apart and cancelled -
        // a notch at 5.6kHz, 20dB deep, and the low end 2dB too loud - where
        // so they add as their powers do, flat within a decibel or two.
        if (on) {
            for (int b = 0; b < BANDS; b += 4) {
                const float *B0 = bandB0_ + b, *A1 = bandA1_ + b, *A2 = bandA2_ + b;
                float b0 = B0[0], b1 = B0[1], b2 = B0[2], b3 = B0[3];
                float p0 = A1[0], p1 = A1[1], p2 = A1[2], p3 = A1[3];
                float q0 = A2[0], q1 = A2[1], q2 = A2[2], q3 = A2[3];
                float y0 = bandY1_[b], y1 = bandY1_[b + 1], y2 = bandY1_[b + 2], y3 = bandY1_[b + 3];
                float z0 = bandY2_[b], z1 = bandY2_[b + 1], z2 = bandY2_[b + 2], z3 = bandY2_[b + 3];
                float v0 = bandZ1_[b], v1 = bandZ1_[b + 1], v2 = bandZ1_[b + 2], v3 = bandZ1_[b + 3];
                float w0 = bandZ2_[b], w1 = bandZ2_[b + 1], w2 = bandZ2_[b + 2], w3 = bandZ2_[b + 3];
                float g0 = bandGain_[b], g1 = bandGain_[b + 1], g2 = bandGain_[b + 2], g3 = bandGain_[b + 3];
                float d0 = bandBy_[b], d1 = bandBy_[b + 1], d2 = bandBy_[b + 2], d3 = bandBy_[b + 3];
                for (int i = 0; i < n; i++) {
                    // Each stage's last output on the critical path only once.
                    float x = lateA[i], u = lateB[i];
                    float u0 = (b0 * x - q0 * z0) - p0 * y0, u1 = (b1 * u - q1 * z1) - p1 * y1;
                    float u2 = (b2 * x - q2 * z2) - p2 * y2, u3 = (b3 * u - q3 * z3) - p3 * y3;
                    z0 = y0, z1 = y1, z2 = y2, z3 = y3;
                    y0 = u0, y1 = u1, y2 = u2, y3 = u3;
                    float r0 = (u0 - q0 * w0) - p0 * v0, r1 = (u1 - q1 * w1) - p1 * v1;
                    float r2 = (u2 - q2 * w2) - p2 * v2, r3 = (u3 - q3 * w3) - p3 * v3;
                    w0 = v0, w1 = v1, w2 = v2, w3 = v3;
                    v0 = r0, v1 = r1, v2 = r2, v3 = r3;
                    g0 += d0, g1 += d1, g2 += d2, g3 += d3;
                    mix[i] += (g0 * r0 + g1 * r1) + (g2 * r2 + g3 * r3);
                }
                bandY1_[b] = y0, bandY1_[b + 1] = y1, bandY1_[b + 2] = y2, bandY1_[b + 3] = y3;
                bandY2_[b] = z0, bandY2_[b + 1] = z1, bandY2_[b + 2] = z2, bandY2_[b + 3] = z3;
                bandZ1_[b] = v0, bandZ1_[b + 1] = v1, bandZ1_[b + 2] = v2, bandZ1_[b + 3] = v3;
                bandZ2_[b] = w0, bandZ2_[b + 1] = w1, bandZ2_[b + 2] = w2, bandZ2_[b + 3] = w3;
                bandGain_[b] = std::max(0.0f, g0), bandGain_[b + 1] = std::max(0.0f, g1);
                bandGain_[b + 2] = std::max(0.0f, g2), bandGain_[b + 3] = std::max(0.0f, g3);
            }
        }
        // The buzz-saws, the two side by side: each a delay of one turn read
        // between samples - four of them, as Lagrange has it - fed back
        // through a gentle smoothing, so its higher lines are a little wider,
        // as the eddies make them.
        if (combs) {
            Comb &c = comb_[0], &k = comb_[1];
            int cw = (int)c.late, kw = (int)k.late;
            // The read points' fractions past the samples at - whole - 1, and
            // Lagrange's four weights for the points -1, 0, 1, 2 about them.
            float f = 1 - (c.late - cw), u = 1 - (k.late - kw);
            float cm = -f * (f - 1) * (f - 2) * (1.0f / 6), c0 = (f + 1) * (f - 1) * (f - 2) * 0.5f,
                  c1 = -(f + 1) * f * (f - 2) * 0.5f, c2 = (f + 1) * f * (f - 1) * (1.0f / 6);
            float km = -u * (u - 1) * (u - 2) * (1.0f / 6), k0 = (u + 1) * (u - 1) * (u - 2) * 0.5f,
                  k1 = -(u + 1) * u * (u - 2) * 0.5f, k2 = (u + 1) * u * (u - 1) * (1.0f / 6);
            float cg = c.g, cin = c.in, cy1 = c.y1, cy2 = c.y2, clo = c.lo, chi = c.hi, ckl = c.kLo, ckh = c.kHi;
            float kg = k.g, kin = k.in, ky1 = k.y1, ky2 = k.y2, klo = k.lo, khi = k.hi, kkl = k.kLo, kkh = k.kHi;
            float *cl = c.line, *kl = k.line, *cn = c.noise, *kn = k.noise;
            float cw0 = c.w[0] * cin, cw1 = c.w[1] * cin, cw2 = c.w[2] * cin, cw3 = c.w[3] * cin;
            float kw0 = k.w[0] * kin, kw1 = k.w[1] * kin, kw2 = k.w[2] * kin, kw3 = k.w[3] * kin;
            int ct1 = c.tap[1], ct2 = c.tap[2], ct3 = c.tap[3], kt1 = k.tap[1], kt2 = k.tap[2], kt3 = k.tap[3];
            int at = combAt_;
            constexpr int M = COMB - 1;
            for (int i = 0; i < n; i++) {
                clo += ckl * (noiseA[i] - clo);
                chi += ckh * (noiseA[i] - chi);
                klo += kkl * (noiseB[i] - klo);
                khi += kkh * (noiseB[i] - khi);
                // The shaped noise, and its pattern: as much again from
                // fractions of a turn back - noise, any sample as good as
                // another, so a tap moving on a sample as the speed changes
                // is no click.
                float cx = chi - clo, kx = khi - klo;
                cn[at] = cx;
                kn[at] = kx;
                float ce = cw0 * cx + cw1 * cn[(at - ct1) & M] + cw2 * cn[(at - ct2) & M] + cw3 * cn[(at - ct3) & M];
                float ke = kw0 * kx + kw1 * kn[(at - kt1) & M] + kw2 * kn[(at - kt2) & M] + kw3 * kn[(at - kt3) & M];
                int p = at - cw - 1, q = at - kw - 1;
                float cb = cm * cl[(p - 1) & M] + c0 * cl[p & M] + c1 * cl[(p + 1) & M] + c2 * cl[(p + 2) & M];
                float kb = km * kl[(q - 1) & M] + k0 * kl[q & M] + k1 * kl[(q + 1) & M] + k2 * kl[(q + 2) & M];
                float cy = ce + cg * cb, ky = ke + kg * kb;
                cl[at] = 0.025f * (cy + cy2) + 0.95f * cy1;
                kl[at] = 0.025f * (ky + ky2) + 0.95f * ky1;
                cy2 = cy1;
                cy1 = cy;
                ky2 = ky1;
                ky1 = ky;
                at = (at + 1) & M;
                mix[i] += cy + ky;
            }
            c.y1 = cy1, c.y2 = cy2, c.lo = clo, c.hi = chi;
            k.y1 = ky1, k.y2 = ky2, k.lo = klo, k.hi = khi;
            combAt_ = at;
        }
        // The nose wheel over the centreline lights: a thump at each - worked
        // through only while one rings, its resonance a few hundredths of a
        // second, which most of the time none is.
        float v = sound_.rollingMs, lights = v / 15;
        bool kick = on && v > 0.5f && lightPhase_ + lights * n * (1.0f / RATE) >= 1;
        if (!kick && fabsf(thumpBand_.ic1) + fabsf(thumpBand_.ic2) < 1e-6f) {
            thumpBand_.ic1 = thumpBand_.ic2 = 0;
            if (on && v > 0.5f) {
                lightPhase_ += lights * n * (1.0f / RATE);
            }
        } else if (on) {
            float phase = lightPhase_, ic1 = thumpBand_.ic1, ic2 = thumpBand_.ic2;
            float ta1 = thumpBand_.a1, ta2 = thumpBand_.a2, ta3 = thumpBand_.a3, tk = thumpBand_.k,
                  thud = 0.5f * std::min(1.0f, v / 40);
            for (int i = 0; i < n; i++) {
                phase += lights * (1.0f / RATE);
                float x = 0;
                if (phase >= 1) {
                    phase -= 1;
                    x = thud * (0.8f + 0.4f * rng.uniform()) * 30;
                }
                float low, band;
                svfRun(ic1, ic2, ta1, ta2, ta3, tk, x, low, band);
                mix[i] += band * 0.012f;
            }
            lightPhase_ = phase;
            thumpBand_.ic1 = fabsf(ic1) < 1e-20f ? 0 : ic1;
            thumpBand_.ic2 = fabsf(ic2) < 1e-20f ? 0 : ic2;
        }
        // The wide resonances - the tyres' squeal, a V2500's turbine tones
        // smeared by the jet - through the air's muffling with the rest: a
        // resonance's skirts fall only 6dB an octave either side of it, and
        // after the muffling, a squeal 250m off rang on at 7kHz, 15dB over
        // everything else there, as nothing heard from so far off does.
        if (on) {
            Tone *wide[5];
            int count = 0;
            for (int t = 0; t < TONES && count < 4; t++) {
                if (tones_[t].on && tones_[t].wide) {
                    wide[count++] = &tones_[t];
                }
            }
            Tone idle;
            if (count % 2 != 0) {
                wide[count++] = &idle;
            }
            for (int j = 0; j < count; j += 2) {
                Tone &p = *wide[j], &q = *wide[j + 1];
                const float *np = p.stream ? noiseB : noiseA, *nq = q.stream ? noiseB : noiseA;
                float pr = p.zr, pi = p.zi, pc = p.cr, ps = p.ci, pn = p.in;
                float qr = q.zr, qi = q.zi, qc = q.cr, qs = q.ci, qn = q.in;
                for (int i = 0; i < n; i++) {
                    float a = (pn * np[i] - ps * pi) + pc * pr, b = (qn * nq[i] - qs * qi) + qc * qr;
                    pi = ps * pr + pc * pi, qi = qs * qr + qc * qi;
                    pr = a, qr = b;
                    mix[i] += a + b;
                }
                p.zr = fabsf(pr) < 1e-20f ? 0 : pr, p.zi = fabsf(pi) < 1e-20f ? 0 : pi;
                q.zr = fabsf(qr) < 1e-20f ? 0 : qr, q.zi = fabsf(qi) < 1e-20f ? 0 : qi;
            }
        }
        // The fighter's sources, into the mix before the air's muffling and
        // the echo, as the airliner's broadband goes.
        if (fighter) {
            fighterRender(mix, lateA, lateB, noiseA, noiseB, rng, n);
        }
        // And the turboprop's, likewise.
        if (prop) {
            propRender(mix, lateA, lateB, noiseA, noiseB, rng, n);
        }
        // The air's muffling of all that; then the echo off the ground, its
        // lateness stepped across the step too.
        if (on || fighter || prop) {
            float ic1 = air_.ic1, ic2 = air_.ic2, a1 = air_.a1, a2 = air_.a2, a3 = air_.a3;
            for (int i = 0; i < n; i++) {
                float v3 = mix[i] - ic2;
                float v1 = a1 * ic1 + a2 * v3;
                float v2 = ic2 + a2 * ic1 + a3 * v3;
                ic1 = 2 * v1 - ic1;
                ic2 = 2 * v2 - ic2;
                mix[i] = v2;
            }
            air_.ic1 = fabsf(ic1) < 1e-20f ? 0 : ic1;
            air_.ic2 = fabsf(ic2) < 1e-20f ? 0 : ic2;
        }
        {
            float lateBy = (echoTap_ - echoTapWas_) / n, lateNow = echoTapWas_;
            float *echo = echo_;
            int echoAt = echoAt_;
            float echoLow = echoLow_, echoLow2 = echoLow2_, echoGain = echoGain_, echoSoft = echoSoft_;
            if (echoGain > 0) {
                for (int i = 0; i < n; i++) {
                    float x = mix[i];
                    echo[echoAt] = x;
                    lateNow += lateBy;
                    float back = echoAt - lateNow;
                    back += back < 0 ? ECHO : 0;
                    int k = (int)back;
                    float e = echo[k] + (echo[(k + 1) & (ECHO - 1)] - echo[k]) * (back - k);
                    echoLow += echoSoft * (e - echoLow);
                    echoLow2 += echoSoft * (echoLow - echoLow2);
                    mix[i] = x + echoGain * echoLow2;
                    echoAt = (echoAt + 1) & (ECHO - 1);
                }
            } else {
                for (int i = 0; i < n; i++) {
                    echo[echoAt] = mix[i];
                    echoAt = (echoAt + 1) & (ECHO - 1);
                }
            }
            echoAt_ = echoAt;
            echoLow_ = fabsf(echoLow) < 1e-20f ? 0 : echoLow;
            echoLow2_ = fabsf(echoLow2) < 1e-20f ? 0 : echoLow2;
        }
        // The turboprop's narrow tones, after the echo as the airliner's are.
        if (prop) {
            propTones(mix, noiseA, noiseB, n);
        }
        // The resonances, four at a time - four chains of sums the core can
        // work through side by side - each its pointer turned and shrunk and
        // pushed by its noise, its real part the tone.
        if (on) {
            Tone *list[TONES + 1];
            int count = 0;
            for (int t = 0; t < TONES; t++) {
                if (tones_[t].on && !tones_[t].wide) {
                    list[count++] = &tones_[t];
                }
            }
            Tone idle[1];
            if (count % 2 != 0) {
                list[count++] = &idle[0];
            }
            int fours = count & ~3;
            for (int j = 0; j < fours; j += 4) {
                Tone &p = *list[j], &q = *list[j + 1], &r = *list[j + 2], &u = *list[j + 3];
                const float *np = p.stream ? noiseB : noiseA, *nq = q.stream ? noiseB : noiseA,
                            *nr = r.stream ? noiseB : noiseA, *nu = u.stream ? noiseB : noiseA;
                float pr = p.zr, pi = p.zi, pc = p.cr, ps = p.ci, pn = p.in;
                float qr = q.zr, qi = q.zi, qc = q.cr, qs = q.ci, qn = q.in;
                float rr = r.zr, ri = r.zi, rc = r.cr, rs = r.ci, rn = r.in;
                float ur = u.zr, ui = u.zi, uc = u.cr, us = u.ci, un = u.in;
                for (int i = 0; i < n; i++) {
                    float a = (pn * np[i] - ps * pi) + pc * pr, b = (qn * nq[i] - qs * qi) + qc * qr;
                    float c = (rn * nr[i] - rs * ri) + rc * rr, e = (un * nu[i] - us * ui) + uc * ur;
                    pi = ps * pr + pc * pi, qi = qs * qr + qc * qi, ri = rs * rr + rc * ri, ui = us * ur + uc * ui;
                    pr = a, qr = b, rr = c, ur = e;
                    mix[i] += (a + b) + (c + e);
                }
                p.zr = fabsf(pr) < 1e-20f ? 0 : pr, p.zi = fabsf(pi) < 1e-20f ? 0 : pi;
                q.zr = fabsf(qr) < 1e-20f ? 0 : qr, q.zi = fabsf(qi) < 1e-20f ? 0 : qi;
                r.zr = fabsf(rr) < 1e-20f ? 0 : rr, r.zi = fabsf(ri) < 1e-20f ? 0 : ri;
                u.zr = fabsf(ur) < 1e-20f ? 0 : ur, u.zi = fabsf(ui) < 1e-20f ? 0 : ui;
            }
            // And the last two, if there are, side by side.
            if (fours < count) {
                Tone &p = *list[fours], &q = *list[fours + 1];
                const float *np = p.stream ? noiseB : noiseA, *nq = q.stream ? noiseB : noiseA;
                float pr = p.zr, pi = p.zi, pc = p.cr, ps = p.ci, pn = p.in;
                float qr = q.zr, qi = q.zi, qc = q.cr, qs = q.ci, qn = q.in;
                for (int i = 0; i < n; i++) {
                    float a = (pn * np[i] - ps * pi) + pc * pr, b = (qn * nq[i] - qs * qi) + qc * qr;
                    pi = ps * pr + pc * pi, qi = qs * qr + qc * qi;
                    pr = a, qr = b;
                    mix[i] += a + b;
                }
                p.zr = fabsf(pr) < 1e-20f ? 0 : pr, p.zi = fabsf(pi) < 1e-20f ? 0 : pi;
                q.zr = fabsf(qr) < 1e-20f ? 0 : qr, q.zi = fabsf(qi) < 1e-20f ? 0 : qi;
            }
        }
        seed_ = rng.s;
        // The level carried smoothly across the step from the last; nothing
        // under 180Hz, which the speaker can't play - two poles, 12dB an
        // octave: with one, the reversers' roar under 300Hz, unheard, still
        // carried peaks into the limiter - and a peak rounded off rather
        // than cut.
        float to = gain_ * fade_ * duck_ * LEVEL, by = (to - level) / n;
        float h1 = hp1_, h2 = hp2_, hb0 = hpB0_, ha1 = hpA1_, ha2 = hpA2_;
        int16_t *o = out + done;
        for (int i = 0; i < n; i++) {
            level += by;
            float x = mix[i] * level;
            float y = hb0 * x + h1;
            h1 = h2 - 2 * hb0 * x - ha1 * y;
            h2 = hb0 * x - ha2 * y;
            float a = fabsf(y);
#ifdef ENGINE_NOISE_DEBUG
            g_limited += a > 0.6f;
            g_samples++;
#endif
            if (a > 0.6f) {
                // 0.6 + 0.4 tanh((a - 0.6) / 0.4), tanh as a ratio near enough
                float t = (a - 0.6f) * 2.5f, t2 = t * t;
                float th = t < 3 ? t * (27 + t2) / (27 + 9 * t2) : 1.0f;
                y = copysignf(0.6f + 0.4f * th, y);
            }
            o[i] = (int16_t)(y * 32000 + (y < 0 ? -0.5f : 0.5f));
        }
        hp1_ = fabsf(h1) < 1e-20f ? 0 : h1;
        hp2_ = fabsf(h2) < 1e-20f ? 0 : h2;
        level = to;
        clock_ += n;
    }
    return sound_.kind == Kind::TURBOFAN || fade_ > 0.003f;
}

Engine a320Engines(const char *type, uint32_t who) {
    if (type == nullptr) {
        return Engine::GENERIC;
    }
    auto is = [&](const char *t) { return strcmp(type, t) == 0; };
    float pick = unit(mix(who ^ 0x5eedf00du));
    if (is("A19N") || is("A20N") || is("A21N")) {
        return pick < 0.60f ? Engine::LEAP_1A : Engine::PW1100G;
    }
    if (is("A318")) {
        return Engine::CFM56_5B;
    }
    if (is("A319") || is("A320") || is("A321")) {
        return pick < 0.55f ? Engine::CFM56_5B : Engine::V2500_A5;
    }
    return Engine::GENERIC;
}

bool a320Fopp(Engine engine, uint32_t who) {
    if (engine != Engine::CFM56_5B && engine != Engine::V2500_A5) {
        return false;
    }
    return unit(mix(who ^ 0xf0bb1e5u)) < 0.40f;
}

Engine b777Engines(const char *type, uint32_t who) {
    if (type == nullptr) {
        return Engine::GENERIC;
    }
    auto is = [&](const char *t) { return strcmp(type, t) == 0; };
    if (is("B77W") || is("B77L")) {
        return Engine::GE90_115B;
    }
    if (is("B772")) {
        return unit(mix(who ^ 0x777e7777u)) < 0.60f ? Engine::GE90_94B : Engine::TRENT_800;
    }
    if (is("B773")) {
        return Engine::TRENT_800;
    }
    return Engine::GENERIC;
}

const PropData &propData(Engine engine) {
    for (const PropData &p : PROPS) {
        if (p.engine == engine) {
            return p;
        }
    }
    return PROP_GENERIC;
}

bool isProp(Engine engine) { return &propData(engine) != &PROP_GENERIC; }

}  // namespace engine_noise
