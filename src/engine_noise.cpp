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
// before it lights.
struct EngineData {
    float blades, shaft100;
    float groundIdle, flightIdle, approach, cruise, climb, takeoff, reverse;
    float buzzOn, buzzDb, jetMs, levelDb, toneDb;
    float turbine[2];
    float motorS;
};
const EngineData ENGINES[6] = {
    // GENERIC: its shaft from Sound::fanHz, 24 blades assumed.
    {24, 125.0f, 0.22f, 0.27f, 0.51f, 0.82f, 0.85f, 0.90f, 0.70f, 0.67f, 0, 300, 0, 0, {0, 0}, 15},
    // CFM56-5B: idle 20-22%; final 49-55% (Zurich); take-off 85-94%; reverse 68%.
    {36, 83.3333f, 0.22f, 0.27f, 0.51f, 0.82f, 0.84f, 0.90f, 0.70f, 0.67f, 0, 300, 0, 0, {0, 0}, 15},
    // V2500-A5: idling at 30% on the ground, its fan's line at 622Hz [M: DLR,
    // D-ATRA], 29% in the air; the BPF of a 1,768-2,067Hz take-off measured;
    // its buzz-saw the strongest of the four in the recorded climb-outs [M:
    // 2.6-6.7dB clear]; LPT tones at engine orders 85 and 89, behind it.
    {22, 94.1667f, 0.30f, 0.29f, 0.50f, 0.82f, 0.82f, 0.90f, 0.68f, 0.64f, 2.0f, 310, -1, 2, {85, 89}, 15},
    // LEAP-1A: TOGA ~83%, FLEX ~75%; level at height at its take-off's 91%,
    // as a CFM56-5B's 82 is of its 90 [I: no neo's is published] - at 80%,
    // all but its take-off's, the chase at cruise stood 1.6dB over the
    // CFM56's; its tips barely past the speed of sound at take-off - its
    // buzz-saw from 73%, weak; its jet slower; 3.5dB quieter departing and
    // as loud arriving [M: Luton's monitors]; motoring longer before
    // light-off.
    {18, 64.2667f, 0.21f, 0.24f, 0.47f, 0.74f, 0.77f, 0.81f, 0.62f, 0.73f, 0, 270, -3, -2, {0, 0}, 24},
    // PW1100G: the fan's speed as a share of its 3,281rpm most; its turbine's
    // tones over 8kHz, the air's.
    {20, 54.6833f, 0.19f, 0.25f, 0.55f, 0.82f, 0.83f, 0.88f, 0.65f, 0.84f, -12, 250, -3, -1, {0, 0}, 30},
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
    {24, 211.667f, 0.30f, 0.35f, 0.75f, 0.85f, 1.00f, 1.00f, 0.30f, 2.0f, -60, 770, 0, 0, {0, 0}, 25},
};

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
    for (int c = 0; c < SOURCES; c++) {
        float kq = expf(-SLOW_S / SURGE_QUICK_S[c]), ks = expf(-SLOW_S / SURGE_SLOW_S[c]);
        surgeKeep_[c][0] = kq;
        surgeKeep_[c][1] = sqrtf(1 - kq * kq);
        surgeKeep_[c][2] = ks;
        surgeKeep_[c][3] = sqrtf(1 - ks * ks);
        surgeKeep_[c][4] = sqrtf(1 - SURGE_QUICK_W[c] * SURGE_QUICK_W[c]);  // the slow part's weight
    }
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

// Another aircraft starting: heard as it is now, its engines already turning,
// rather than spooling up from nothing; its own little differences from who
// it is.
void Synth::restart() {
    const Sound &s = heard_;
    traits_ = mix(s.who * 2654435761u + (uint32_t)s.engine * 97u + 12345u);
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
    // - square to its nose and the vertical - from where it was.
    {
        float wx = nose[1], wy = -nose[0], wn = sqrtf(wx * wx + wy * wy);
        float k = wn > 1e-3f ? 5.75f / wn : 0;
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
        float n = n1_[g], w = to > n ? 1.0f + 0.8f * smoothstep(0.38f, 0.55f, n) : 1.4f;
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
    static const float FADE_IN = 1 - expf(-STEP_S / 0.5f), FADE_OUT = 1 - expf(-STEP_S / 0.2f),
                       DUCK = 1 - expFast(-STEP_S / 0.12f), REVERSE = 1 - expFast(-STEP_S / 0.9f),
                       WHERE = 1 - expFast(-STEP_S / 0.12f), CLOSING = 1 - expFast(-STEP_S / 0.25f),
                       ASPECT = 1 - expFast(-STEP_S / 0.2f), GEAR = 1 - expFast(-STEP_S / 2.5f),
                       FLAPS = 1 - expFast(-STEP_S / 3.0f), SPOILERS = 1 - expFast(-STEP_S / 0.25f),
                       AIRSPEED = 1 - expFast(-STEP_S / 0.5f), LIGHT = 1 - expFast(-STEP_S / 0.8f),
                       OUT = 1 - expf(-STEP_S / 0.3f), WIND_DOWN = expf(-STEP_S / 12), APU_DOWN = expf(-STEP_S / 5),
                       ECHO_WANDER = 1 - expf(-STEP_S / 0.6f);
    fade_ = ease(fade_, on || fighter ? 1.0f : 0.0f, on || fighter ? FADE_IN : FADE_OUT);
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
        surge[c] = exp2Fast(2.88539008f * SURGE_NP[c] * (n - SURGE_NP[c]));  // e^(2sn - 2s^2): its mean one
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
    float jetPeak = log2Fast(0.17f * jetMs / 300) + 1.5f + 0.5f * smoothstep(0.25f, 0.5f, th) - 2 * smoothstep(0.5f, 0.83f, th);
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
    float fanCorner = 0.678072f + 0.6f * log2Fast(std::max(0.1f, n1r));  // octaves from 1kHz: 1.6kHz at take-off
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
    float rush = 60 * lg10(std::min(vr, 1.6f)) + thin;
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
    set(FAN_H2, 2 * bpf, db - 6 + 4 * low, width * 1.4f + 2 * apart, 0, 2);
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
        float loHz = 4 * hz, hiHz = std::min(std::min(18.0f, e.blades) * hz, 6000.0f);
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

// The figures fitted on the three measurement scenes (x-eufi-zeltweg,
// x-eufi-close-pass, x-eufi-display-100m) against the 18 recordings, six
// realisations of each scene, through the same chains as the recordings
// [M: tools/engine_preview/NOTES.md, renders/eufi-fit/metrics.json].
//  Its level, dB, against the airliner's LEVEL: its reheat take-off 2.3LU
//  over the airliner's in short-term loudness, its 80m reheat pass 1.8LU,
//  the soft limiter rounding off 0.06% and 0.09% of their samples - a
//  decibel more put the pass's over 0.1% (the roar's own peaks, not its
//  shocks', at that loudness).
constexpr float TY_LEVEL_DB = -7.5f;
//  The fine turbulence's level against the big eddies', dB, and its peak,
//  octaves from theirs; the share of the Doppler the bank's spectrum moves
//  by; the 250Hz band's power kept, the speaker playing next to none of it.
constexpr float TY_FST_DB = -1.0f, TY_FST_OCT = -0.6f, TY_KAPPA = 0.15f, TY_BAND0 = 0.25f;
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
constexpr float TY_P0 = 1.0f, TY_THETA = 0.4f, TY_BURST_NP = 1.1f, TY_ALPHA = 0.35f, TY_LN_NP = 0.25f;
constexpr float TY_G_CR = 7.0f, TY_CR_FAR = 1500.0f, TY_CR_DB = 0.0f, TY_MC_ON = 0.8f, TY_MC_ALL = 1.5f;
//  The intake's whine idling, dB against the jet's level; the level the
//  roar is ridden down to at its loudest, as samples go.
constexpr float TY_WHINE_DB = -40.0f, TY_RIDE = 0.14f;

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
// most behind it - see the lobes.
constexpr float JET_N1[6] = {0.30f, 0.52f, 0.75f, 0.85f, 0.93f, 1.0f};
constexpr float JET_DB[6] = {-38, -30, -20, -13, -6, 0};

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
// afterburner] - 30dB down ahead, and down again right behind, in the jet's
// cone of silence.
constexpr Lobe LTS_DRY = {-20.0f, -19.0f, -17.0f, -14.0f, -10.0f, -6.0f, -2.0f, 0.0f, -0.5f, -2.5f};
constexpr Lobe LTS_WET = {-19.0f, -18.0f, -16.0f, -12.0f, -7.0f, -2.5f, 0.0f, -0.5f, -1.0f, -3.0f};
// The fine turbulence's: broad, most to the side, 12dB down ahead.
constexpr Lobe FST = {-4.0f, -3.5f, -3.0f, -2.0f, -1.0f, 0.0f, 0.0f, -2.0f, -4.0f, -6.0f};
// The shocks', against the roar they ride on: most about the Mach angle,
// and still there, weaker, coming [M: the passes' derivative skewness, through
// the speaker's 180Hz high-pass, 0.3-1.0 coming, 0.8-1.0 at the peak, 0.5
// going - not the 20-30dB swing of their source alone].
constexpr Lobe CRACKLE_DRY = {-6.0f, -5.5f, -5.0f, -4.5f, -3.5f, -2.5f, -1.0f, 0.0f, -1.5f, -5.0f};
constexpr Lobe CRACKLE_WET = {-5.0f, -4.5f, -4.0f, -3.5f, -2.5f, -1.0f, 0.0f, -0.5f, -2.0f, -5.0f};

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
    j.reheat = std::max(clamp01(s.reheat), s.afterburner ? 1.0f : 0.0f) * smoothstep(0.93f, 0.97f, n1);
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
        // the resonance's gain at the centre taken out of the band-pass's
        j.lowB0 = alpha / a0 * sqrtf(re * re + im * im);
        j.lowA1 = a1;
        j.lowA2 = a2;
        if (j.lowHz < 0) {
            // its mean envelope as noise through it gives - a third's power, over
            // some 0.85 of its width - till its own mean takes over
            j.envMean = 0.8f * sqrtf((1.0f / 3) * 0.85f * lowHz / (Q * 0.5f * RATE));
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
    j.shockUnit = P0 * gate * sqrtf(j.ltsSurge) * exp2Fast(1.44269504f * BURST_NP * (j.burst - 0.5f * BURST_NP));
    j.shockP = j.shockUnit * (1 + j.hawkes);
    // Its intake's whine, the fan's blades passing and twice that - 24 blades
    // assumed, none published [I: eufi-sources 1.2] - as heard, the Doppler on
    // it, turned on each step: noise through a resonance 3Hz wide near, as
    // the airliner's tones are; as loud as its slow step makes it.
    float spun = n1 * (spool_ < 1 ? powFast(std::max(spool_, 1e-3f), 0.7f) : 1.0f);
    for (int k = 0; k < 2; k++) {
        Tone &t = tones_[FAN_1 + k];
        float heard = (k + 1) * e.blades * e.shaft100 * spun * j.doppler;
        if (j.whine < 1e-7f || heard < 30 || heard > TOP_HZ) {
            t.in = 0;
            t.on = t.on && (t.zr * t.zr + t.zi * t.zi > 1e-14f);
            continue;
        }
        float x = PI * (3.0f + 3.5f * expFast(logDistance_) * 0.001f) / RATE, r = 1 - x * (1 - x * (0.5f - x * (1.0f / 6)));
        float sn, cs;
        sinCosTurns(heard / RATE, sn, cs);
        t.cr = cs * r;
        t.ci = sn * r;
        float amp = j.whine * (k == 0 ? 1.0f : 0.5f);
        if (heard > TOP_HZ - TOP_FADE_HZ) {
            amp *= (TOP_HZ - heard) * (1 / TOP_FADE_HZ);
        }
        t.in = amp * sqrtf(6 * (1 - r * r));
        t.stream = 1;
        t.on = true;
    }
}

// Each 32ms: the loudness as it spreads, its roar's surges and its bands'
// sums, the shocks' strength.
__attribute__((noinline)) void Synth::fighterSlowStep() {
    const Sound &s = sound_;
    Jet &j = jet_;
    float d = expFast(logDistance_), km = d * 0.001f;
    // Its own convective gain, with its quicker way: louder coming, by no
    // more than 2.5 times, quieter going, by no more than half.
    gain_ = powFast(35 / std::max(35.0f, d) * dbAmp(-heldDb_), 0.65f);
    if (s.placed) {
        gain_ *= std::max(0.5f, std::min(2.5f, powFast(j.convection, 0.65f)));
    }
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
    // than 10dB]; reheat's 4dB.
    float n1 = std::max(0.0f, 0.5f * (n1_[0] + n1_[1]));
    float power = 0;
    for (int g = 0; g < 2; g++) {
        float sp = g == 0 ? spool_ : j.spool2, fire = g == 0 ? lit_ : j.lit2;
        float n = n1 * (sp < 1 ? powFast(sp, 1.6f) : 1.0f), db = JET_DB[5];
        if (n <= JET_N1[0]) {
            db = JET_DB[0] - 20 * (1 - n / JET_N1[0]);  // spooling up to idle
        } else {
            for (int i = 1; i < 6; i++) {
                if (n <= JET_N1[i]) {
                    db = JET_DB[i - 1] + (JET_DB[i] - JET_DB[i - 1]) * (n - JET_N1[i - 1]) / (JET_N1[i] - JET_N1[i - 1]);
                    break;
                }
            }
        }
        power += 0.5f * dbPower(db) * (fire * fire + 0.02f * sp);
    }
    static const float LEVEL_DB = TY_LEVEL_DB, FST_DB = TY_FST_DB, KAPPA = TY_KAPPA;
    float v = std::min(airspeed_, 400.0f);
    float flight = std::max(-10.0f, 50 * lg10(std::max(0.01f, 1 - v / std::max(60.0f, j.jetMs))));
    float jetDb = LEVEL_DB + 10 * lg10(std::max(1e-9f, power)) + flight + 4 * j.reheat;
    float th = j.aspect, r = j.reheat;
    float ltsLobe = lobe(LTS_DRY, th) + (lobe(LTS_WET, th) - lobe(LTS_DRY, th)) * r;
    float ltsDb = jetDb + ltsLobe, fstDb = jetDb + FST_DB + lobe(FST, th);
    // Its spectra's peaks, as it gives them out: the big eddies' at the low
    // band's Strouhal number, the fine turbulence's 1.8 times higher; heard
    // a third of the Doppler up or down - for in the speaker's band, the
    // passes' spectra hardly move [M: centroid 844Hz coming, 710 going, where
    // the whole Doppler of a display pass would move it more than an octave]:
    // the swing from bright to deep is the roar's below 300Hz.
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
    // Its intake's whine, ahead of it, at low power only - idling, taxiing,
    // on the approach - as loud as the jet ahead idling and rising to some
    // 5dB under it on the approach [I: eufi-sources 3.5, 11.1, 11.8: no
    // recording of one]; none at display power, where no recording has one
    // [M: the strongest line in any 0.5s of 29 passes no more than noise's].
    float spun = n1 * (spool_ < 1 ? powFast(std::max(spool_, 1e-3f), 0.7f) : 1.0f);
    j.whine = spun > 0.05f ? dbAmp(LEVEL_DB + TY_WHINE_DB + 30 * lg10(std::max(0.1f, spun / 0.3f)) +
                                   lobe(FAN_TONE, th)) *
                                 (1 - smoothstep(0.75f, 0.9f, spun))
                           : 0.0f;
    float roar = 0;
    for (int b = 0; b < BANDS; b++) {
        float lf = bandLog_[b] - shift;
        float pL = dbPower(ltsDb + tam(TAM_F, lf - peak)), pF = dbPower(fstDb + tam(TAM_G, lf - peak - TY_FST_OCT));
        float surge = exp2Fast(2.88539008f * sg * (lowW[b] * nLow + highW[b] * nHigh - sg));  // e^(2sn - 2s^2): mean one
        float p = (pL + pF) * surge * (b == 0 ? TY_BAND0 : 1.0f);
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
        p *= exp2Fast(1.44269504f * ln);
        roar += (pL + pF) * surge;
        bandBy_[b] = (sqrtf(p) - bandGain_[b]) * (1.0f / (SLOW * STEP));
    }
    // The low band's envelope moving the bank.
    j.muLo = TY_MU_LO;
    j.muHi = TY_MU_HI;
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
    j.shockAmp = G_CR * sqrtf(roar) * dbAmp(crLobe + CR_DB) * far * gate;
#ifdef ENGINE_NOISE_DEBUG
    static int debugIn = 0;
    if (--debugIn < 0) {
        debugIn = 31;
        fprintf(stderr, "t %.2f d %.0f asp %.2f n1 %.3f rh %.2f mc %.2f dop %.2f | jet %.1f lts %.1f fst %.1f mu %.2f "
                        "shockP %.3f amp %.4f low %.0fHz env %.4f fired %d gain %.3f held %.1f conv %.2f\n",
                clock_ / (float)RATE, d, th, n1, r, j.mc, j.doppler, jetDb, ltsDb, fstDb, j.muLo, j.shockP, j.shockAmp,
                j.lowHz, j.envMean, g_fired, gain_, heldDb_, j.convection);
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
    float muLo = j.muLo, muHi = j.muHi, p = j.shockP, more = j.shockUnit * ALPHA, amp = j.shockAmp;
    float *ring = j.ring;
    int at = j.ringAt, fired = 0;
    const float *lo = half[0], *hi = half[1];
    float flutter = j.flutterWas, flutterBy = (j.flutterTo - j.flutterWas) * (1.0f / STEP);
    // The roar ridden down at its very loudest - a reheat pass 80m off, the
    // take-off beside the runway - so it peaks under the soft limiter and the
    // shocks keep their edges: its level as it will leave, over a fifth of a
    // second rising and a second falling, held to RIDE by a third of what it
    // goes over [I: eufi-methods 9.8]. Under that, as it is.
    float ride = j.ride, rideBy = 0, power = 0;
    {
        static const float RIDE = TY_RIDE;
        float out = j.roarRms * gain_ * fade_ * duck_ * LEVEL;
        float want = out > RIDE ? powFast(RIDE / out, 0.6667f) : 1.0f;
        rideBy = (want - ride) * (1.0f / STEP);
    }
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
            float a = amp * m * exp2Fast(LN_A * rng.gauss() - LN_B);
            for (int k = 0; k < SHOCK_N; k++) {
                ring[(at + k) & 63] += a * SHOCK[k];
            }
            p += more;
            fired++;
        }
        was = z;
        flutter += flutterBy;
        float jet = (lo[i] + muLo * (m - 1) * lo[i]) + (hi[i] + muHi * (m - 1) * hi[i]) * flutter;
        power += jet * jet;
        ride += rideBy;
        mix[i] += jet * ride + ring[at];
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
    {
        static const float UP = 1 - expf(-STEP_S / 0.2f), DOWN = 1 - expf(-STEP_S / 1.0f);
        float rms = sqrtf(power * (1.0f / STEP));
        j.roarRms += (rms - j.roarRms) * (rms > j.roarRms ? UP : DOWN);
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
        float pr = t.zr, pi = t.zi, pc = t.cr, ps = t.ci, pn = t.in * (k < 2 ? 1.0f : 0.5f);
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


bool Synth::render(int16_t *out, int count) {
    float mix[STEP], lateA[STEP], lateB[STEP], noiseA[STEP], noiseB[STEP];
    float level = gain_ * fade_ * duck_ * LEVEL;
    for (int done = 0; done < count; done += STEP) {
        int n = std::min(STEP, count - done);
        update();
        bool on = sound_.kind == Kind::TURBOFAN;
        bool fighter = sound_.kind == Kind::LOW_BYPASS;
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
        // The air's muffling of all that; then the echo off the ground, its
        // lateness stepped across the step too.
        if (on || fighter) {
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

}  // namespace engine_noise
