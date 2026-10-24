#include "engine_noise.h"

#include <math.h>
#include <stdlib.h>

#include <algorithm>

#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

namespace engine_noise {

bool g_recordedVoices = false;

namespace {

// A fighter's roar: nine seconds of a Eurofighter Typhoon's, recorded, and
// five of one's front, coming at the microphone; an airliner's, a
// turboprop's and a light aircraft's - see tools/gen_roar.py.
#include "roar_table.inc"
constexpr uint32_t ROAR_LENGTH = sizeof(ROAR_LOOP) / sizeof(ROAR_LOOP[0]),
                   FRONT_LENGTH = sizeof(ROAR_FRONT) / sizeof(ROAR_FRONT[0]),
                   TAKEOFF_LENGTH = sizeof(AIRLINER_TAKEOFF) / sizeof(AIRLINER_TAKEOFF[0]),
                   APPROACH_LENGTH = sizeof(AIRLINER_APPROACH) / sizeof(AIRLINER_APPROACH[0]),
                   LOW_LENGTH = sizeof(TURBOPROP_LOW) / sizeof(TURBOPROP_LOW[0]),
                   HIGH_LENGTH = sizeof(TURBOPROP_HIGH) / sizeof(TURBOPROP_HIGH[0]),
                   PISTON_LENGTH = sizeof(PISTON_LOOP) / sizeof(PISTON_LOOP[0]);
// Their propellers' blades passing in the recordings - an ATR's at 1,200rpm,
// a King Air's at 1,700 - and the light aircraft's strongest tone, the third
// of its firing at 1,710rpm.
constexpr float LOW_BLADE_HZ = 119.3f, HIGH_BLADE_HZ = 113.1f, PISTON_RPM = 1710;

// A stretch of a recording runs this long, 0.6 to 1.6 seconds at random,
// and fades into the next over this: played round as a loop, nine seconds'
// surges and crackles came back on a beat, from the chase. The next is from
// at least this far from where this one and the one before began - or a
// sixth of the recording, a short one's - so nothing heard is heard again
// at once.
constexpr int GRAIN_MIN = RATE * 6 / 10, GRAIN_SPREAD = RATE, GRAIN_FADE = RATE * 16 / 100,
              GRAIN_APART = RATE * 3 / 2;

// A recorded loop's sample at `at` - 20.12 fixed point - between its two
// nearest, and `at` moved on `step`, round to its start at its end - or,
// `back`, moved back, round to its end at its start.
__attribute__((always_inline)) inline float loopSample(const int16_t *loop, uint32_t length, uint32_t &at,
                                                       uint32_t step, bool back = false) {
    uint32_t k = at >> 12, next = k + 1 < length ? k + 1 : 0;
    float f = (float)(at & 4095) * (1.0f / 4096), a = loop[k];
    if (back) {
        at = at >= step ? at - step : at + (length << 12) - step;
    } else {
        at += step;
        if ((at >> 12) >= length) {
            at -= length << 12;
        }
    }
    return a + (loop[next] - a) * f;
}

constexpr float PI = 3.14159265f;
// The controls - loudness, pitch, the filters - worked out again every this
// many samples, 2ms: often enough for none of them to be heard stepping.
constexpr int STEP = 32;
constexpr float STEP_S = STEP / (float)RATE;
constexpr float SOUND_MS = 343;  // the speed of sound, at 20C
// What is made now is heard this much later on the Tab5 - a 40ms block
// queued behind the one playing, and the speaker's own 43ms of buffers - so
// what is heard is looked for that much sooner: a tyre's screech as late as
// its distance makes it, not 80ms later still.
#ifdef ESP_PLATFORM
constexpr float HEARD_AHEAD_S = 0.08f;
#else
constexpr float HEARD_AHEAD_S = 0;
#endif
// The highest a tone is played at, faded out over the last 1,200Hz to it -
// under 8kHz, half the samples' rate, past which a tone comes back down as
// another.
constexpr float TOP_HZ = 7600, TOP_FADE_HZ = 1200;
// The recorded roar's samples to the mix's scale; an airliner's, a
// turboprop's and a light aircraft's, likewise.
constexpr float ROAR_GAIN = 1.0f / 32768 * 3.0f, AIRLINER_GAIN = 1.0f / 32768 * 1.8f,
                TURBOPROP_GAIN = 1.0f / 32768 * 1.3f, PISTON_GAIN = 1.0f / 32768 * 1.65f;

// What the air takes of a sound going a kilometre through it, dB, at 125Hz
// to 8kHz an octave apart - ISO 9613-1's, at 20C and 70% humidity: next to
// nothing of a rumble, three quarters of the eight kilohertz' level in a
// hundred metres.
constexpr float ABSORPTION_DB_KM[7] = {0.3f, 1.1f, 2.8f, 5.0f, 9.0f, 22.9f, 76.6f};

// The frequency the air has taken 6dB of over `km`: the air's muffling
// tuned there - between the octaves, as the logs of both go - and not
// under 400Hz, the speaker having nothing much below.
float mufflingHz(float km) {
    float most = 6 / std::max(km, 1e-4f);  // dB a kilometre the 6dB is
    if (most <= ABSORPTION_DB_KM[0]) {
        return 7000;
    }
    for (int i = 1; i < 7; i++) {
        if (ABSORPTION_DB_KM[i] >= most) {
            float a = logf(ABSORPTION_DB_KM[i - 1]), b = logf(ABSORPTION_DB_KM[i]);
            float octave = (i - 1) + (logf(most) - a) / (b - a);
            return std::min(7000.0f, std::max(400.0f, 125 * exp2f(octave)));
        }
    }
    return 7000;
}
// The mix's level, as samples go: an airliner cruising, from the chase, a
// steady -22dBFS; taking off past a camera beside the runway about the same,
// brighter; a fighter's afterburner going by, -10. Under the words, which
// are near full scale.
constexpr float LEVEL = 1.6f;

float clamp01(float x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

// The closing speed a sound's pitch is worked from, as far as it is let go:
// placed, from half its pitch to over three times - a fast jet coming at
// nearly the speed of its sound - and otherwise within 150m/s either way.
float closingOf(const Sound &s) {
    return s.placed ? std::max(-SOUND_MS, std::min(0.7f * SOUND_MS, s.closingMs))
                    : std::max(-150.0f, std::min(150.0f, s.closingMs));
}

float smoothstep(float a, float b, float x) {
    x = clamp01((x - a) / (b - a));
    return x * x * (3 - 2 * x);
}

// The part of the way something easing over `tau` seconds comes in a STEP;
// and `from` eased that part of the way toward `to`.
float stepOf(float tau) { return 1 - expf(-STEP_S / tau); }
float ease(float from, float to, float part) { return from + (to - from) * part; }

// One turn of a sine; of a blade going by - a pulse, its first 24
// harmonics each a little weaker than the one before; and of a fan with its
// tips going faster than sound - the buzz-saw - every harmonic of its
// turning there, each as loud as its blades' small differences make it,
// which is to say any old how, but the same every turn. Read with the
// fraction of a turn between two points taken.
// In PSRAM on the device: the chip's own memory is the 3D view's, and three
// tables of floats there for good were 6KB less of it.
constexpr int TABLE = 512;
float *g_sine = nullptr, *g_blade = nullptr, *g_buzz = nullptr;
bool g_tables = false;

float *newTable() {
#ifdef ESP_PLATFORM
    return (float *)heap_caps_malloc((TABLE + 1) * sizeof(float), MALLOC_CAP_SPIRAM);
#else
    return (float *)malloc((TABLE + 1) * sizeof(float));
#endif
}

void normalise(float *table) {
    float most = 0;
    for (int i = 0; i <= TABLE; i++) {
        most = std::max(most, fabsf(table[i]));
    }
    for (int i = 0; i <= TABLE; i++) {
        table[i] /= most;
    }
}

void makeTables() {
    if (g_tables) {
        return;
    }
    g_sine = newTable();
    g_blade = newTable();
    g_buzz = newTable();
    float amp[36], phase[36];
    uint32_t seed = 0x5eed;
    auto random = [&]() {
        seed = seed * 1664525 + 1013904223;
        return (seed >> 8) * (1.0f / 16777216.0f);
    };
    for (int k = 0; k < 36; k++) {
        amp[k] = (0.25f + 0.75f * random()) / sqrtf(k + 1.0f);
        phase[k] = 2 * PI * random();
    }
    for (int i = 0; i <= TABLE; i++) {
        float a = 2 * PI * i / TABLE, b = 0, z = 0;
        g_sine[i] = sinf(a);
        for (int k = 1; k <= 24; k++) {
            b += cosf(k * a) / powf((float)k, 0.75f);
        }
        for (int k = 1; k <= 36; k++) {
            z += amp[k - 1] * sinf(k * a + phase[k - 1]);
        }
        g_blade[i] = b;
        g_buzz[i] = z;
    }
    normalise(g_blade);
    normalise(g_buzz);
    g_tables = true;
}

inline __attribute__((always_inline)) float lookup(const float *table, float phase) {
    float x = phase * TABLE;
    int i = (int)x;
    return table[i] + (table[i + 1] - table[i]) * (x - i);
}

// A phase moved on a sample's worth at `hz`, kept between 0 and 1. True as
// it comes round.
inline __attribute__((always_inline)) bool advance(float &phase, float hz) {
    phase += hz * (1.0f / RATE);
    if (phase < 1) {
        return false;
    }
    phase -= (float)(int)phase;  // floorf's, a phase being nought or more: one convert, not a call
    return true;
}

}  // namespace

// Tuned again only as its cutoff moves a fifth of a percent or more: the
// tangent is most of what a step's controls cost.
void Synth::Svf::tune(float toHz, float toQ) {
    toHz = std::min(std::max(toHz, 20.0f), RATE * 0.45f);
    if (toQ == q && fabsf(toHz - hz) < hz * 0.002f) {
        return;
    }
    hz = toHz;
    q = toQ;
    float g = tanf(PI * hz / RATE);
    k = 1 / q;
    a1 = 1 / (1 + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
}

// The low-pass and the band-pass - the band-pass at the cutoff as loud as
// what goes in.
__attribute__((always_inline)) inline void Synth::Svf::run(float x, float &low, float &band) {
    float v3 = x - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    float v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = 2 * v1 - ic1;
    ic2 = 2 * v2 - ic2;
    low = v2;
    band = v1 * k;
}

Synth::Synth() {
    makeTables();
    air_.tune(7000, 0.6f);
}

// What is asked for, kept with the sample it was asked at - a snapshot each
// 20ms at most - for what is heard to be what was asked as long ago as the
// sound takes to come. Another kind, another aircraft: heard at once, the
// one before's history forgotten.
void Synth::set(const Sound &sound) {
    bool forget = sound.kind != target_.kind || sound.who != target_.who;
    // Placed, a jump no aircraft makes - its frame moved under it, a replay
    // sought on - leaves what was asked before in another place: forgotten
    // too, rather than heard from there.
    if (!forget && sound.placed && askedCount_ > 0 && asked_[askedNewest_].sound.placed) {
        const float *a = sound.at, *b = asked_[askedNewest_].sound.at;
        float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
        float most = 400 * std::max(0.04f, (clock_ - asked_[askedNewest_].at) / (float)RATE) + 50;
        forget = dx * dx + dy * dy + dz * dz > most * most;
    }
    if (forget) {
        askedCount_ = 0;
        askedNewest_ = -1;
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

float Synth::uniform() {
    seed_ ^= seed_ << 13;
    seed_ ^= seed_ >> 17;
    seed_ ^= seed_ << 5;
    return (seed_ >> 8) * (1.0f / 16777216.0f);
}

float Synth::noise() { return uniform() * 2 - 1; }

// The tone, its loudness steady, its pitch drifting - see wander(); and
// with `narrow` the narrow band of noise about it, its filter tuned to it
// each STEP by the sound that has it.
__attribute__((always_inline)) inline float Synth::whine(Whine &w, float hz, bool narrow, Rng &rng) {
    // Over what 16,000 samples a second can carry - a fan's whine three
    // times up, coming at nearly the speed of sound - faded out, rather than
    // folded back down as another, wrong one.
    float f = hz * w.drift;
    advance(w.phase, std::min(f, TOP_HZ));
    float y = lookup(g_sine, w.phase) * 0.5f;
    if (narrow) {
        float low, band;
        w.band.run(rng.noise(), low, band);
        y += band * 1.2f;
    }
    return f < TOP_HZ - TOP_FADE_HZ ? y : y * clamp01((TOP_HZ - f) * (1 / TOP_FADE_HZ));
}

// The next sample of a recording played a stretch at a time - see Grains -
// the stretches faded into each other at equal power, the two unrelated.
// `vary`, a roar's: each stretch played backwards as often as not - a jet's
// roar has no way round, its crackle's rises as steep as its falls - and up
// to a fortieth faster or slower, so a stretch heard again isn't the same.
__attribute__((always_inline)) inline float Synth::grain(Grains &g, const int16_t *loop, uint32_t length,
                                                         uint32_t step, Rng &rng, bool vary) {
    float y = loopSample(loop, length, g.at, (uint32_t)(step * g.rate), g.back);
    if (g.fading >= 0) {
        float t = g.fading * (1.0f / GRAIN_FADE),
              b = loopSample(loop, length, g.next, (uint32_t)(step * g.nextRate), g.nextBack);
        y = y * lookup(g_sine, (1 - t) * 0.25f) + b * lookup(g_sine, t * 0.25f);
        if (++g.fading >= GRAIN_FADE) {
            g.at = g.next;
            g.back = g.nextBack;
            g.rate = g.nextRate;
            g.fading = -1;
            g.left = GRAIN_MIN + (int)(rng.uniform() * GRAIN_SPREAD);
        }
    } else if (--g.left <= 0) {
        // From elsewhere than this stretch and the last began - three tries.
        uint32_t apart = std::min((uint32_t)GRAIN_APART, length / 6), next = 0;
        auto near = [&](uint32_t a, uint32_t b) {
            uint32_t d = a > b ? a - b : b - a;
            return std::min(d, length - d) < apart;
        };
        for (int tries = 0; tries < 3; tries++) {
            next = (uint32_t)(rng.uniform() * (length - 1));
            if (!near(next, g.from) && !near(next, g.was)) {
                break;
            }
        }
        g.was = g.from;
        g.from = next;
        g.next = next << 12;
        g.nextBack = vary && rng.uniform() < 0.5f;
        g.nextRate = vary ? 0.975f + 0.05f * rng.uniform() : 1.0f;
        g.fading = 0;
    }
    return y;
}

// A whine's pitch drifting by a fifth of a percent at most - noise smoothed
// to a hertz or so - moved on once a STEP.
void Synth::wander(Whine &w) {
    w.wander += 0.0128f * (noise() - w.wander);
    w.drift = 1 + 0.002f * std::max(-1.0f, std::min(1.0f, w.wander * 11));
}

// Another kind starting - another aircraft - heard as it is now, its engines
// already turning, rather than spooling up from nothing.
void Synth::restart() {
    power_ = clamp01(heard_.power);
    reverse_ = heard_.reverse ? 1.0f : 0.0f;
    burner_ = heard_.afterburner ? 1.0f : 0.0f;
    slap_ = clamp01(heard_.slap);
    logDistance_ = logf(std::max(5.0f, heard_.distanceM));
    closing_ = closingOf(heard_);
    convection_ = convectionTo_;
    aspect_ = clamp01(heard_.aspect);
    for (Svf *f : {&roar_, &body_, &buzz_, &formant1_, &formant2_, &thumpBand_, &slapBand_, &crackleBand_, &fan_.band,
                   &fan2_.band, &turbine_.band, &screechBand_[0], &screechBand_[1], &rollBand_, &lightBand_,
                   &rushBand_, &flapBand_, &buffetBand_}) {
        f->clear();
    }
    // Its engines and APU as they are: none heard starting that was running.
    spool_ = lit_ = heard_.running ? 1.0f : 0.0f;
    apu_ = heard_.apu ? 1.0f : 0.0f;
    apuTone_.band.clear();
    apuBand_.clear();
    // Its airframe as it is: no gear heard coming down that was down already.
    gear_ = heard_.gearDown ? 1.0f : 0.0f;
    flaps_ = heard_.flapsOut ? 1.0f : 0.0f;
    spoilers_ = clamp01(heard_.spoilers);
    airspeed_ = std::max(0.0f, heard_.airspeedMs);
    buffet_ = 0;
    thump_ = smack_ = crackle_ = kick_ = 0;
    std::fill(echo_, echo_ + ECHO, 0.0f);
    echoLow_ = echoLow2_ = echoWander_ = 0;
    touchdownWas_ = heard_.touchdownS;
    screech_[0] = screech_[1] = 0;
    screechS_ = 10;
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
    heard_.bladeHz = target_.bladeHz;
    heard_.tailHz = target_.tailHz;
    heard_.tandem = target_.tandem;
    heard_.sourceUpM = old.sound.sourceUpM + (next.sound.sourceUpM - old.sound.sourceUpM) * u;
    heard_.earUpM = target_.earUpM;
    float to[3] = {ear[0] - at[0], ear[1] - at[1], ear[2] - at[2]};
    float d = sqrtf(to[0] * to[0] + to[1] * to[1] + to[2] * to[2]), per = 1 / std::max(1.0f, d);
    heard_.distanceM = d;
    // Its speed along the way the sound came, from the two asked either side
    // of then; the ear's, from the last two - none across a cut from one
    // camera to another, a jump no camera moving makes.
    auto along = [&](const Asked &a, const Asked &b) {
        float s = between(a, b);
        return s > 0 ? ((b.sound.at[0] - a.sound.at[0]) * to[0] + (b.sound.at[1] - a.sound.at[1]) * to[1] +
                        (b.sound.at[2] - a.sound.at[2]) * to[2]) *
                           per / s
                     : 0.0f;
    };
    float vs = 0, vo = 0;
    if (askedCount_ > 1) {
        int i0 = std::max(k, 1);
        vs = along(entry(i0), entry(i0 - 1));
        const Asked &e1 = entry(1), &e0 = entry(0);
        float s = between(e1, e0);
        float mx = e0.sound.ear[0] - e1.sound.ear[0], my = e0.sound.ear[1] - e1.sound.ear[1],
              mz = e0.sound.ear[2] - e1.sound.ear[2], most = 400 * s + 20;
        if (s > 0 && mx * mx + my * my + mz * mz < most * most) {
            vo = (mx * to[0] + my * to[1] + mz * to[2]) * per / s;
        }
    }
    float doppler = (SOUND_MS - vo) / std::max(1.0f, SOUND_MS - vs);
    heard_.closingMs = SOUND_MS * (1 - 1 / std::max(0.1f, doppler));
    // How much louder its own coming makes it - what it said over a while
    // arriving in less - or quieter going: by its speed alone, the ear's
    // making none. The camera following, as the chase does, hears it from
    // nearer along the sound's way but no louder: the two come out even.
    convectionTo_ = SOUND_MS / std::max(1.0f, SOUND_MS - vs);
    float n = sqrtf(nose[0] * nose[0] + nose[1] * nose[1] + nose[2] * nose[2]);
    float c = (nose[0] * to[0] + nose[1] * to[1] + nose[2] * to[2]) * per / std::max(1e-6f, n);
    heard_.aspect = acosf(std::max(-1.0f, std::min(1.0f, c))) / PI;
}

void Synth::update() {
    // What is heard: what was asked as long ago as the sound takes to come
    // from where it was - its distance then, near enough its distance now.
    // A cut from one camera to another, a jump in the distance, makes the
    // lateness jump with it; otherwise it moves only as the aircraft does.
    float late = std::min(target_.distanceM, 2000.0f) / SOUND_MS;
    delayS_ = fabsf(late - delayS_) > 0.3f ? late : ease(delayS_, late, stepOf(0.05f));
    heard_ = target_;
    if (askedCount_ > 0 && target_.placed) {
        placedWay();
    } else if (askedCount_ > 0) {
        uint32_t want = clock_ - (uint32_t)(std::max(0.0f, delayS_ - HEARD_AHEAD_S) * RATE);
        int at = askedNewest_;
        for (int k = 1; k < askedCount_ && (int32_t)(asked_[at].at - want) > 0; k++) {
            at = (at + HEARD - 1) % HEARD;
        }
        heard_ = asked_[at].sound;
        // The aircraft's own - its kind and figures - as asked now.
        heard_.kind = target_.kind;
        heard_.who = target_.who;
        heard_.engines = target_.engines;
        heard_.fanHz = target_.fanHz;
        heard_.bladeHz = target_.bladeHz;
        heard_.tailHz = target_.tailHz;
        heard_.tandem = target_.tandem;
    }
    // Another aircraft, or none, once the one before has faded out; the
    // same one, its figures as they come.
    bool same = heard_.kind == sound_.kind && heard_.who == sound_.who;
    if (!same && fade_ < 0.003f) {
        sound_.kind = heard_.kind;
        sound_.who = heard_.who;
        same = true;
        restart();
    }
    if (same) {
        sound_ = heard_;
    }
    bool on = same && sound_.kind != Kind::NONE;
    static const float FADE_IN = stepOf(0.5f), FADE_OUT = stepOf(0.2f), DUCK = stepOf(0.12f), JET_UP = stepOf(1.8f),
                       JET_DOWN = stepOf(2.5f), PROP_UP = stepOf(0.6f), PROP_DOWN = stepOf(0.8f),
                       REVERSE = stepOf(0.8f), BURNER = stepOf(0.4f), SLAP = stepOf(0.6f), WHERE = stepOf(0.12f),
                       CLOSING = stepOf(0.25f), ASPECT = stepOf(0.2f), SHIMMER = stepOf(0.4f), SURGE = stepOf(0.3f),
                       ECHO_WANDER = stepOf(0.6f);
    fade_ = ease(fade_, on ? 1.0f : 0.0f, on ? FADE_IN : FADE_OUT);
    duck_ = ease(duck_, under_ ? 0.2f : 1.0f, DUCK);
    // Engines spool up and down over seconds - a jet's slower than a
    // propeller's - and a jet's slower down than up.
    bool jet = sound_.kind == Kind::TURBOFAN || sound_.kind == Kind::LOW_BYPASS;
    float spool = heard_.power > power_ ? (jet ? JET_UP : PROP_UP) : (jet ? JET_DOWN : PROP_DOWN);
    power_ = ease(power_, clamp01(heard_.power), spool);
    reverse_ = ease(reverse_, heard_.reverse ? 1.0f : 0.0f, REVERSE);
    burner_ = ease(burner_, heard_.afterburner ? 1.0f : 0.0f, BURNER);
    slap_ = ease(slap_, clamp01(heard_.slap), SLAP);
    // The gear's rush building over its few seconds of coming down - doors
    // open, legs swinging out into the air - the flaps' as they run out, the
    // spoilers' at once, as they spring up.
    static const float GEAR = stepOf(2.5f), FLAPS = stepOf(3.0f), SPOILERS = stepOf(0.25f), AIRSPEED = stepOf(0.5f);
    gear_ = ease(gear_, heard_.gearDown ? 1.0f : 0.0f, GEAR);
    // Starting: the core on its starter to a third of idle in 15 seconds, the
    // fuel lit there - the exhaust's roar swelling in - and on up to idle in
    // 14 more. Shut down: the fuel cut, the fire out at once, the spool
    // winding down over half a minute. The APU spooling up over 25 seconds
    // and down over a few.
    static const float WIND_DOWN = expf(-STEP_S / 12), LIGHT = stepOf(0.8f), OUT = stepOf(0.3f),
                       APU_DOWN = expf(-STEP_S / 5);
    if (heard_.running) {
        spool_ = std::min(1.0f, spool_ + (spool_ < 0.35f ? 0.35f / 15 : 0.65f / 14) * STEP_S);
    } else {
        spool_ = spool_ < 0.002f ? 0 : spool_ * WIND_DOWN;
    }
    bool burning = heard_.running && spool_ >= 0.35f;
    lit_ = ease(lit_, burning ? 1.0f : 0.0f, burning ? LIGHT : OUT);
    apu_ = heard_.apu ? std::min(1.0f, apu_ + STEP_S / 25) : (apu_ < 0.002f ? 0 : apu_ * APU_DOWN);
    flaps_ = ease(flaps_, heard_.flapsOut ? 1.0f : 0.0f, FLAPS);
    spoilers_ = ease(spoilers_, clamp01(heard_.spoilers), SPOILERS);
    airspeed_ = ease(airspeed_, std::max(0.0f, heard_.airspeedMs), AIRSPEED);
    // Where it is heard from: eased a little, so a cut from one camera to
    // another isn't a click.
    logDistance_ = ease(logDistance_, logf(std::max(5.0f, heard_.distanceM)), WHERE);
    closing_ = ease(closing_, closingOf(heard_), CLOSING);
    aspect_ = ease(aspect_, clamp01(heard_.aspect), ASPECT);
    float d = expf(logDistance_);
    // Its pitch as it was coming then: up coming, down going.
    doppler_ = SOUND_MS / (SOUND_MS - closing_);
    wander(fan_);
    wander(fan2_);
    wander(turbine_);
    if (apu_ > 0) {
        wander(apuTone_);
    }
    // Spreading: the pressure a sound carries falling as one over the
    // distance - a sixth for each doubling - from 35m, then squeezed for a
    // small speaker on a desk, to the 0.65th power: a kilometre off a fifth
    // as loud as the physics, not a thirtieth, which it would play as
    // nothing. Far off, the air's eddies bending it about: the level
    // wandering by a seventh at three kilometres, a shimmer.
    shimmer_ = ease(shimmer_, noise(), SHIMMER);
    // A fighter's recorded roar held level, its surges slower than half a
    // second taken out - one came round with the stretches it was in, a
    // landmark the loop was known by - and made here instead: a new one each
    // 0.4 to 1.2 seconds, up to 2.5dB either way, never the same twice.
    if ((surgeLeft_ -= STEP) <= 0) {
        surgeTo_ = noise() * 2.5f;
        surgeLeft_ = (int)(RATE * (0.4f + 0.8f * uniform()));
    }
    surgeDb_ = ease(surgeDb_, surgeTo_, SURGE);
    float wobble = 1 + 0.15f * std::min(1.0f, d / 3000) * std::max(-1.0f, std::min(1.0f, shimmer_ * 4));
    gain_ = powf(35 / std::max(35.0f, d), 0.65f) * wobble;
    // Placed, louder coming and quieter going - squeezed as the spreading
    // is, the two together a camera following it hearing it as loud as from
    // where it is - by no more than 6dB up or 4 down.
    if (heard_.placed) {
        convection_ = ease(convection_, convectionTo_, CLOSING);
        gain_ *= std::max(0.6f, std::min(2.0f, powf(convection_, 0.65f)));
    }
    // The air's muffling, as ISO 9613-1 has it at that distance.
    air_.tune(mufflingHz(d * 0.001f), 0.6f);
    // Off the ground: the source's image under the ground, as far below it
    // as the source is above - the echo's way the straight way's length
    // with the two heights added rather than taken from each other. How much
    // later it comes; and how strong, the longer way spreading it further.
    // Steady and alike only low down the spectrum: higher, the ground's
    // roughness, the air's eddies and the engines being metres apart all
    // scatter it, so it comes back through a low-pass - and its lateness
    // wanders a little, as the eddies do. A runway gives back more than
    // grass. Coherent all the way up, as a mirror would be, it swept its
    // notches through a jet's whine as it came, and the whine throbbed
    // three times a second rather than climbing. None more than 32ms late -
    // a whole echo, not the ground's colouring, and too weak by then.
    float up = std::max(0.0f, heard_.sourceUpM), ear = std::max(0.0f, heard_.earUpM);
    float across2 = std::max(0.0f, d * d - (up - ear) * (up - ear));
    float straight = sqrtf(across2 + (up - ear) * (up - ear)), round = sqrtf(across2 + (up + ear) * (up + ear));
    echoLateWas_ = echoLate_;
    echoWander_ = ease(echoWander_, noise() * 3, ECHO_WANDER);
    float lateSamples = (round - straight) / SOUND_MS * RATE + echoWander_;
    if (lateSamples >= 0 && lateSamples < ECHO - 8) {
        echoLate_ = lateSamples;
        echoGain_ = (heard_.hardGround ? 0.6f : 0.4f) * straight / std::max(1.0f, round);
    } else {
        echoGain_ = 0;
    }
    echoSoft_ = 1 - expf(-2 * PI * (heard_.hardGround ? 700 : 500) / RATE);
}

bool Synth::render(int16_t *out, int count) {
    float mix[STEP];
    const float dc = 1 - expf(-2 * PI * 180 / RATE);
    float level = gain_ * fade_ * duck_ * LEVEL;
    for (int done = 0; done < count; done += STEP) {
        int n = std::min(STEP, count - done);
        update();
        switch (sound_.kind) {
        case Kind::TURBOFAN:
        case Kind::LOW_BYPASS: turbofan(mix, n); break;
        case Kind::TURBOPROP: turboprop(mix, n); break;
        case Kind::PISTON: piston(mix, n); break;
        case Kind::ROTOR: rotor(mix, n); break;
        default: std::fill(mix, mix + n, 0.0f); break;
        }
        if (sound_.kind != Kind::NONE) {
            tyres(mix, n);
        }
        if (sound_.kind != Kind::NONE && sound_.kind != Kind::ROTOR) {
            airframe(mix, n);
        }
        if (sound_.kind != Kind::NONE && apu_ > 0) {
            apu(mix, n);
        }
        // The level carried smoothly across the step from the last; the
        // echo off the ground, its lateness stepped across the step too; the
        // air's muffling; nothing under 180Hz, which the speaker can't play;
        // and a peak rounded off rather than cut.
        float to = gain_ * fade_ * duck_ * LEVEL, by = (to - level) / n;
        float lateBy = (echoLate_ - echoLateWas_) / n, lateNow = echoLateWas_;
        // The step's state in locals - in registers - written back after it.
        float *echo = echo_;
        int echoAt = echoAt_;
        float echoLow = echoLow_, echoLow2 = echoLow2_, echoGain = echoGain_, echoSoft = echoSoft_, dcLow = dcLow_;
        Svf air = air_;
        int16_t *o = out + done;
        for (int i = 0; i < n; i++) {
            float x = mix[i];
            echo[echoAt] = x;
            if (echoGain > 0) {
                lateNow += lateBy;
                float back = echoAt - lateNow;
                if (back < 0) {
                    back += ECHO;
                }
                int k = (int)back;
                float frac = back - k, e = echo[k] + (echo[(k + 1) % ECHO] - echo[k]) * frac;
                echoLow += echoSoft * (e - echoLow);
                echoLow2 += echoSoft * (echoLow - echoLow2);
                x += echoGain * echoLow2;
            }
            echoAt = (echoAt + 1) % ECHO;
            float low, band;
            air.run(x, low, band);
            level += by;
            float y = low * level;
            dcLow += dc * (y - dcLow);
            y -= dcLow;
            float a = fabsf(y);
            if (a > 0.6f) {
                y = copysignf(0.6f + 0.4f * tanhf((a - 0.6f) / 0.4f), y);
            }
            o[i] = (int16_t)(y * 32000 + (y < 0 ? -0.5f : 0.5f));
        }
        echoAt_ = echoAt;
        echoLow_ = echoLow;
        echoLow2_ = echoLow2;
        dcLow_ = dcLow;
        air_ = air;
        level = to;
        clock_ += n;
    }
    return sound_.kind != Kind::NONE;
}

// Its tyres. Touching down, each main gear's in turn a chirp - its tyres
// dragged along the runway till they spin up, a squeal in a narrow band
// falling from two kilohertz or so as the slip does, a tenth of a second -
// the louder the harder it came down. Rolling, a low rumble of rubber on
// concrete, under the engines rather than over them - as loud as it was, a
// broad roar a third of the engines', it was a rocket going up - and the
// nose wheel over the centreline lights, one every 15 metres, a thump each.
void Synth::tyres(float *mix, int n) {
    const Sound &s = sound_;
    if (s.touchdownS >= 0 && (touchdownWas_ < 0 || s.touchdownS + 0.05f < touchdownWas_)) {
        screechS_ = 0;
        screechLoud_ = std::min(1.6f, std::max(0.5f, s.sinkMs / 1.5f)) * std::min(1.0f, s.rollingMs / 50);
        screechAt_[0] = 0;
        screechAt_[1] = 0.03f + 0.06f * uniform();
    }
    touchdownWas_ = s.touchdownS;
    float v = s.rollingMs;
    bool screeching = screechS_ < 0.8f, rolling = v > 0.5f;
    if (!screeching && !rolling) {
        return;
    }
    for (int g = 0; g < 2 && screeching; g++) {
        float t = screechS_ - screechAt_[g];
        screechBand_[g].tune((1300 + 1000 * expf(-std::max(0.0f, t) / 0.07f)) * doppler_, 10);
    }
    rollBand_.tune((180 + 3 * v) * doppler_, 0.7f);
    lightBand_.tune(300 * doppler_, 8);
    float roll = 0.03f * powf(std::min(1.5f, v / 70), 1.5f), lights = v / 15;
    Rng rng{seed_};
    Svf screech[2] = {screechBand_[0], screechBand_[1]}, rollBand = rollBand_, lightBand = lightBand_;
    float screechS = screechS_, lightPhase = lightPhase_;
    for (int i = 0; i < n; i++) {
        float y = 0;
        if (screeching) {
            for (int g = 0; g < 2; g++) {
                float t = screechS - screechAt_[g];
                if (t < 0) {
                    continue;
                }
                float envelope = (1 - expf(-t / 0.003f)) * expf(-t / 0.09f), low, band;
                screech[g].run(rng.noise(), low, band);
                y += band * envelope * screechLoud_ * 0.7f;
            }
            screechS += 1.0f / RATE;
        }
        if (rolling) {
            float low, band;
            rollBand.run(rng.noise(), low, band);
            y += low * roll;
            float kick = 0;
            if (advance(lightPhase, lights)) {
                kick = 0.5f * std::min(1.0f, v / 40) * (0.8f + 0.4f * rng.uniform());
            }
            lightBand.run(kick * 30, low, band);
            y += band * 0.04f;
        }
        mix[i] += y;
    }
    screechBand_[0] = screech[0];
    screechBand_[1] = screech[1];
    rollBand_ = rollBand;
    lightBand_ = lightBand;
    screechS_ = screechS;
    lightPhase_ = lightPhase;
    seed_ = rng.s;
}

// Its airframe: the air tearing past it, dipole noise off its edges - its
// power going as the sixth power of the speed, the sound's pressure as the
// cube - a light rush off a clean wing, and much more off the gear's legs
// and wheels, broad and low, and the flaps' and slats' edges, higher: on
// an approach, the engines throttled back, as loud as they are. The
// spoilers up on the roll-out, the air over the wing torn off it - a low
// buffet, wandering, going as the square of the speed. The gear's coming
// down is heard as its rush building over the few seconds it takes - not
// as its down-locks' knock, which those in the cabin hear and those outside
// don't: put in, a knock seven seconds after, it was heard as a blip.
void Synth::airframe(float *mix, int n) {
    float v = airspeed_ * (1.0f / 72), v2 = std::min(4.0f, v * v), v3 = std::min(8.0f, v2 * v);
    // By its size, against an A320's, which these were set by: the edges the
    // air tears past as long as it is, each as deep as its chord - a
    // fighter's a third of an airliner's, its rush a wind over its roar.
    // Not over an A320's, as yet unheard.
    // A fighter clean, none: its engines drown the little it makes, and
    // alone in front of it, where its roar is beamed away, the rush was all
    // there was to hear - a gale. Its gear and flaps, landing, as anyone's.
    float k = std::max(0.3f, std::min(1.0f, sound_.size));
    // An airliner's roar recorded has its gear and flaps in it already - the
    // approach overhead was recorded with them out: a third as much again.
    if (g_recordedVoices && sound_.kind == Kind::TURBOFAN) {
        k *= 0.35f;
    }
    float clean = sound_.kind == Kind::LOW_BYPASS ? 0 : 0.011f;
    float rush = (clean * v3 + 0.064f * gear_ * v3) * k, flap = 0.035f * flaps_ * v3 * k,
          buffet = 0.12f * spoilers_ * v2 * k;
    if (rush + flap + buffet < 1e-4f) {
        return;
    }
    rushBand_.tune(520 * doppler_, 0.5f);
    flapBand_.tune(1700 * doppler_, 0.6f);
    buffetBand_.tune(330 * doppler_, 0.9f);
    Rng rng{seed_};
    Svf rushBand = rushBand_, flapBand = flapBand_, buffetBand = buffetBand_;
    float wander = buffet_;
    for (int i = 0; i < n; i++) {
        float x = rng.noise(), low, band, flow, fband;
        rushBand.run(x, low, band);
        flapBand.run(x, flow, fband);
        float y = band * rush + fband * flap;
        if (buffet > 0) {
            // Its loudness wandering a few times a second, as torn-off air
            // does: noise smoothed to ten hertz or so.
            wander += 0.004f * (rng.noise() - wander);
            float blow, bband;
            buffetBand.run(x, blow, bband);
            y += bband * buffet * (0.55f + 0.45f * std::max(-1.0f, std::min(1.0f, wander * 12)));
        }
        mix[i] += y;
    }
    rushBand_ = rushBand;
    flapBand_ = flapBand;
    buffetBand_ = buffetBand;
    buffet_ = wander;
    seed_ = rng.s;
}

// An airliner's APU - a small gas turbine in its tail - as heard on an
// apron: its compressor's steady whine, a little of its second harmonic, and
// its exhaust's hiss; a few decibels under an engine at idle, its pitch
// rising as it spools up.
void Synth::apu(float *mix, int n) {
    float hz = 2350 * (0.3f + 0.7f * apu_) * doppler_, tone = 0.05f * apu_ * apu_, hiss = 0.03f * apu_;
    float secondTone = tone * 0.15f * clamp01((TOP_HZ - 2 * hz) * (1 / TOP_FADE_HZ));
    apuTone_.band.tune(hz, 14);
    apuBand_.tune(1100 * doppler_, 0.7f);
    Rng rng{seed_};
    Whine whine1 = apuTone_;
    Svf band = apuBand_;
    for (int i = 0; i < n; i++) {
        float y = whine(whine1, hz, true, rng) * tone;
        float second = whine1.phase * 2;
        second -= (float)(int)second;
        y += lookup(g_sine, second) * secondTone;
        float low, b;
        band.run(rng.noise(), low, b);
        mix[i] += y + b * hiss;
    }
    apuTone_ = whine1;
    apuBand_ = band;
    seed_ = rng.s;
}

// A turbofan: its fan's blades whining at their passing, the louder ahead of
// it and plainest on the approach; the jet's roar, the louder behind and the
// more as the thrust comes on, steeply; at take-off power, over the whine,
// the buzz-saw - the fan's tips going faster than sound, a rasp at each of
// the shaft's harmonics; and the compressor's thin whistle, heard over an
// idling engine. Reverse thrust throws the roar forward, a steady surge -
// shaken, it was a steam train; an afterburner adds a brighter roar and its
// crackle. A low-bypass one - a fighter's, an old jet's - is mostly roar.
void Synth::turbofan(float *mix, int n) {
    bool lowBypass = sound_.kind == Kind::LOW_BYPASS;
    // Starting or winding down, the fan lagging the core.
    float n1 = (0.22f + 0.76f * power_) * (spool_ < 1 ? powf(spool_, 1.6f) : 1.0f);
    // The fan heard ahead of it, the exhaust most at 40 degrees off its tail -
    // a fighter's narrow, hot jet beaming it aft the more: there 6dB over
    // its side and 10 over its front, and less again right behind it, the
    // cone of silence along the jet. Evenly, its pass had no peak: the
    // recordings' passes swing 30 to 40dB from coming to its roar going by.
    float ahead = 1 - 0.75f * smoothstep(0.35f, 0.85f, aspect_);
    float behind = lowBypass ? (aspect_ < 0.78f ? 0.3f + 0.7f * smoothstep(0.35f, 0.78f, aspect_)
                                                : 1 - 0.45f * smoothstep(0.78f, 1.0f, aspect_))
                             : 0.25f + 0.75f * smoothstep(0.15f, 0.8f, aspect_);
    behind = std::max(behind, reverse_);
    float fan = sound_.fanHz * n1 * doppler_;
    // A fighter's fan barely heard over its jet: Eurofighters recorded close
    // by - an overflight, Dunsfold's displays - show no whine at all above
    // their roar, which their fan's 3kHz here stood 30dB clear of.
    // And an airliner's whine 10dB down with its roar recorded: over a real
    // approach's rumble, recorded overhead and landing, it stood 17dB under.
    float fanLevel = (lowBypass ? 0.012f : g_recordedVoices ? 0.038f : 0.12f) * (0.5f + 0.5f * n1) *
                     (1 - 0.55f * smoothstep(0.7f, 0.95f, n1)) * ahead * (spool_ < 1 ? clamp01(n1 / 0.15f) : 1.0f);
    float fan2Level = 2 * fan < 7400 ? fanLevel * 0.25f : 0;
    fan_.band.tune(fan, 18);
    float saw = lowBypass ? 0 : 0.35f * smoothstep(0.8f, 0.95f, n1) * ahead;
    float shaft = fan / 32;  // the fan's turning: blades go by some 32 times a turn
    float roar = (0.06f + 0.94f * powf(n1, 3.5f)) * behind * (lowBypass ? 1.25f : 0.85f) *
                 (sound_.engines >= 4 ? 1.2f : 1) * (1 + 0.5f * reverse_) * (1 + 0.9f * burner_) * lit_;
    // An airliner's roar shaped noise; a fighter's the recording's, below.
    roar_.tune((250 + 1800 * powf(n1, 1.5f) + 1200 * burner_) * doppler_, 0.6f);
    body_.tune(650 * doppler_ * (0.7f + 0.5f * n1), 1.2f);
    buzz_.tune(1500 * doppler_, 0.8f);
    crackleBand_.tune(2500 * doppler_, 0.7f);
    // The core's whine, turning first as it starts: its pitch with the core's
    // spool, heard as soon as it turns at all.
    float whistle = (lowBypass ? 0.006f : 0.025f) * (1 - 0.5f * n1) * ahead * (spool_ < 1 ? clamp01(spool_ / 0.2f) : 1.0f),
          whistleHz = (5200 + 1600 * n1) * doppler_ * (spool_ < 1 ? 0.25f + 0.75f * spool_ : 1.0f);
    // A fighter's roar the recording's, played at the pitch its flight puts
    // on it - and a little higher the harder its engines work, its jet the
    // faster - in 20.12 fixed point: shaped noise, however its spectrum was
    // set, was a gale on the Tab5's speaker. Its churning, its tearing, come
    // with it.
    uint32_t roarStep = (uint32_t)(std::max(0.1f, doppler_ * (0.8f + 0.3f * n1)) * 4096.0f);
    Grains roarG = roar_g_, frontG = front_g_;
    const float roarLevel = roar * ROAR_GAIN, surge = powf(10, surgeDb_ * (1 / 20.0f));
    // Ahead of it, its front's - its intake screaming, its shocks tearing -
    // past 90 degrees off its nose, its roar's; between, the two together.
    // Its roar alone ahead, quiet, was a dull rush: a gale.
    const float frontShare = lowBypass ? 1 - smoothstep(0.2f, 0.5f, aspect_) : 0;
    // An airliner's recorded, if asked for: its approach's at low power, its
    // take-off's at high, between the two by its fan's speed; as loud as its
    // shaped noise was at each, by its power and which way it is heard.
    const bool recorded = !lowBypass && g_recordedVoices;
    const float takeoffShare = smoothstep(0.55f, 0.9f, n1),
                airlinerLevel = (0.06f + 0.94f * powf(n1, 3.5f)) * behind * (sound_.engines >= 4 ? 1.2f : 1) *
                                (1 + 0.5f * reverse_) * lit_ * AIRLINER_GAIN;
    Grains takeoffG = takeoff_g_, approachG = approach_g_;
    Rng rng{seed_};
    Svf roarBand = roar_, bodyBand = body_, buzzBand = buzz_, crackleBand = crackleBand_;
    Whine fan1 = fan_, fan2 = fan2_, turbine = turbine_;
    float shaftPhase = shaftPhase_, kick = kick_, burner = burner_;
    for (int i = 0; i < n; i++) {
        float y;
        if (lowBypass) {
            float rear = grain(roarG, ROAR_LOOP, ROAR_LENGTH, roarStep, rng, true) * surge,
                  front = grain(frontG, ROAR_FRONT, FRONT_LENGTH, roarStep, rng);
            y = (rear + (front - rear) * frontShare) * roarLevel;
        } else if (recorded) {
            float high = grain(takeoffG, AIRLINER_TAKEOFF, TAKEOFF_LENGTH, roarStep, rng),
                  low = grain(approachG, AIRLINER_APPROACH, APPROACH_LENGTH, roarStep, rng);
            y = (low + (high - low) * takeoffShare) * airlinerLevel;
        } else {
            float x = rng.noise(), low, band, low2, band2;
            roarBand.run(x, low, band);
            bodyBand.run(x, low2, band2);
            y = (low * 0.8f + band2 * 0.7f) * roar;
        }
        y += whine(fan1, fan, true, rng) * fanLevel;
        if (fan2Level > 0) {
            y += whine(fan2, 2 * fan, false, rng) * fan2Level;
        }
        if (saw > 0) {
            advance(shaftPhase, shaft);
            float blow, bband;
            buzzBand.run(lookup(g_buzz, shaftPhase), blow, bband);
            y += bband * saw;
        }
        y += whine(turbine, whistleHz, false, rng) * whistle;
        if (burner > 0.02f) {
            if (rng.uniform() < 0.003f * burner) {
                kick = (0.5f + 0.5f * rng.uniform()) * burner;
            }
            kick *= 0.8f;
            float clow, cband;
            crackleBand.run(kick * rng.noise(), clow, cband);
            y += cband * 6 * behind;
        }
        mix[i] = y;
    }
    roar_ = roarBand;
    body_ = bodyBand;
    buzz_ = buzzBand;
    crackleBand_ = crackleBand;
    fan_ = fan1;
    fan2_ = fan2;
    turbine_ = turbine;
    shaftPhase_ = shaftPhase;
    kick_ = kick;
    roar_g_ = roarG;
    front_g_ = frontG;
    takeoff_g_ = takeoffG;
    approach_g_ = approachG;
    seed_ = rng.s;
}

// A turboprop: its propellers' blades going by - a pulse rich in harmonics,
// the drone - two of them a shade apart in speed, phasing slowly against
// each other (further apart, the beat pulsed); the roar of the
// air through them, the more with the power; and the turbines' whine. Their
// speed held nearly steady, as a constant-speed propeller's is, the power
// in how hard the blades bite.
void Synth::turboprop(float *mix, int n) {
    // Starting or winding down: the propellers spinning up or down with the
    // spool, the turbines' whine with it, the exhaust's roar only lit.
    float np = (0.72f + 0.28f * power_) * spool_;
    float blades = sound_.bladeHz * np * doppler_;
    float prop = 1.2f * (0.35f + 0.65f * power_) * (sound_.engines >= 4 ? 1.2f : 1) * (1 + 0.5f * reverse_) *
                 (spool_ < 1 ? spool_ * spool_ : 1.0f);
    float roar = 0.3f * (0.2f + 0.8f * power_ * power_) * (1 + 1.2f * reverse_) * lit_;
    roar_.tune(900 * (0.6f + 0.4f * power_) * doppler_, 0.7f);
    float whistle = 0.03f * (spool_ < 1 ? clamp01(spool_ / 0.2f) : 1.0f),
          whistleHz = (5600 + 900 * power_) * doppler_ * (spool_ < 1 ? 0.25f + 0.75f * spool_ : 1.0f);
    Rng rng{seed_};
    Whine turbine = turbine_;
    if (g_recordedVoices) {
        // Recorded, if asked for: an ATR's approach at low power, a King
        // Air's take-off at full, between the two by its power, as loud as
        // its propellers synthesised. Each at its type's blades' pitch - a
        // constant-speed propeller's, held at its speed in the air, slower
        // only idling on the ground - but moved less than in step with it:
        // the further from its own a recording is played, the more it is a
        // tape run fast or slow, its exhausts' and nacelles' ringing moved
        // with it. Its turbines' whine and its exhaust are in it; the whine
        // alone as it starts, before its propellers turn.
        float governed = std::max(smoothstep(0.05f, 0.35f, power_), smoothstep(25.0f, 40.0f, airspeed_)),
              blading = sound_.bladeHz * (0.72f + 0.28f * governed) * spool_;
        uint32_t lowStep = (uint32_t)(std::max(0.02f, doppler_ * powf(blading / LOW_BLADE_HZ, 0.7f)) * 4096.0f),
                 highStep = (uint32_t)(std::max(0.02f, doppler_ * powf(blading / HIGH_BLADE_HZ, 0.7f)) * 4096.0f);
        float level = prop * (1 / 1.2f) * TURBOPROP_GAIN, highShare = smoothstep(0.55f, 0.95f, power_),
              starting = spool_ < 1 ? whistle * (1 - smoothstep(0.5f, 1.0f, spool_)) : 0;
        Grains low = low_g_, high = high_g_;
        for (int i = 0; i < n; i++) {
            float a = grain(low, TURBOPROP_LOW, LOW_LENGTH, lowStep, rng),
                  b = grain(high, TURBOPROP_HIGH, HIGH_LENGTH, highStep, rng);
            mix[i] = (a + (b - a) * highShare) * level;
            if (starting > 0) {
                mix[i] += whine(turbine, whistleHz, false, rng) * starting;
            }
        }
        low_g_ = low;
        high_g_ = high;
        turbine_ = turbine;
        seed_ = rng.s;
        return;
    }
    Svf roarBand = roar_;
    float blade0 = bladePhase_[0], blade1 = bladePhase_[1];
    for (int i = 0; i < n; i++) {
        advance(blade0, blades);
        advance(blade1, blades * 1.0012f);
        float low, band;
        roarBand.run(rng.noise(), low, band);
        mix[i] = (lookup(g_blade, blade0) + lookup(g_blade, blade1)) * 0.5f * prop + low * roar +
                 whine(turbine, whistleHz, false, rng) * whistle;
    }
    roar_ = roarBand;
    turbine_ = turbine;
    bladePhase_[0] = blade0;
    bladePhase_[1] = blade1;
    seed_ = rng.s;
}

// A light aircraft's piston engine: four cylinders firing twice a turn, each
// a little differently and not quite evenly - the exhaust's bark through
// the pipe's two resonances - and its two-bladed propeller going by as
// often.
void Synth::piston(float *mix, int n) {
    static const float CYLINDER[4] = {1.0f, 0.86f, 0.95f, 0.8f};
    float rpm = (900 + 1800 * power_) * spool_;
    float fire = rpm / 30 * doppler_;
    formant1_.tune(420 * doppler_, 2.5f);
    formant2_.tune(1150 * doppler_, 3);
    // Shut down, the firing stops and the propeller coasts on to a stop.
    float exhaust = 2.5f * (0.3f + 0.7f * power_) * lit_,
          prop = 0.3f * (0.3f + 0.7f * power_) * (spool_ < 1 ? spool_ : 1.0f);
    Rng rng{seed_};
    if (g_recordedVoices) {
        // Recorded, if asked for: a Cessna's, on its approach, at its
        // engine's speed - moved less than in step with it, as a turboprop's
        // is - as loud as its exhaust synthesised by its power, and stopping
        // with its firing as it is shut down.
        uint32_t step = (uint32_t)(std::max(0.02f, doppler_ * powf(rpm / PISTON_RPM, 0.7f)) * 4096.0f);
        float level = (0.3f + 0.7f * power_) * lit_ * (spool_ < 1 ? spool_ : 1.0f) * PISTON_GAIN;
        Grains g = piston_g_;
        for (int i = 0; i < n; i++) {
            mix[i] = grain(g, PISTON_LOOP, PISTON_LENGTH, step, rng) * level;
        }
        piston_g_ = g;
        seed_ = rng.s;
        return;
    }
    Svf formant1 = formant1_, formant2 = formant2_;
    float firePhase = firePhase_, kick = kick_, blade = bladePhase_[0];
    int cylinder = cylinder_;
    for (int i = 0; i < n; i++) {
        float x = 0;
        if (advance(firePhase, fire)) {
            firePhase -= 0.04f * rng.uniform();  // the next a shade late, or not
            cylinder = (cylinder + 1) & 3;
            kick = CYLINDER[cylinder] * (0.85f + 0.3f * rng.uniform());
            x = kick * 4;
        }
        kick *= 0.995f;
        x += kick * rng.noise() * 0.6f;
        float low1, band1, low2, band2;
        formant1.run(x, low1, band1);
        formant2.run(x, low2, band2);
        advance(blade, fire);
        mix[i] = (band1 + 0.6f * band2) * exhaust + lookup(g_blade, blade) * prop;
    }
    formant1_ = formant1;
    formant2_ = formant2;
    firePhase_ = firePhase;
    kick_ = kick;
    bladePhase_[0] = blade;
    cylinder_ = cylinder;
    seed_ = rng.s;
}

// A helicopter: each of its main rotor's blades going by a thump - a burst
// of low noise dying away before the next, the "wop" - the harder the more
// it is working; coming down, slowing, the sharp slap of a blade meeting
// the one before's wake; two rotors' blades in turn. Its tail rotor's buzz,
// its turbine's whine, and the broad rush of air under it all.
void Synth::rotor(float *mix, int n) {
    // Starting or winding down: the rotor's speed with the spool - its chop
    // slowing to a stop - working only lit.
    float passes = sound_.bladeHz * doppler_ * (sound_.tandem ? 2 : 1) * spool_;
    float load = (0.35f + 0.65f * power_) * (spool_ < 1 ? spool_ * lit_ : 1.0f);
    thumpBand_.tune(420 * doppler_, 0.9f);
    slapBand_.tune(1600 * doppler_, 0.9f);
    roar_.tune(1100 * doppler_, 0.7f);
    float tail = sound_.tailHz > 0 ? 0.08f * (0.5f + 0.5f * power_) * (spool_ < 1 ? spool_ : 1.0f) : 0,
          tailHz = sound_.tailHz * doppler_ * spool_;
    float whistle = 0.02f * (spool_ < 1 ? clamp01(spool_ / 0.2f) : 1.0f),
          whistleHz = 6400 * doppler_ * (spool_ < 1 ? 0.25f + 0.75f * spool_ : 1.0f);
    static const float thumpFade = expf(-1 / (0.022f * RATE)), smackFade = expf(-1 / (0.0035f * RATE));
    Rng rng{seed_};
    Svf thumpBand = thumpBand_, slapBand = slapBand_, roarBand = roar_;
    Whine turbine = turbine_;
    float blade = bladePhase_[0], tailPhase = tailPhase_, thump = thump_, smack = smack_, slap = slap_;
    bool second = secondRotor_, tandem = sound_.tandem;
    for (int i = 0; i < n; i++) {
        if (advance(blade, passes)) {
            second = !second;
            float which = tandem && second ? 0.8f : 1;
            thump = load * which * (0.9f + 0.2f * rng.uniform());
            smack = slap * which * (0.85f + 0.3f * rng.uniform());
        }
        float x = rng.noise(), low, band, slow, sband, rlow, rband;
        thumpBand.run(x, low, band);
        slapBand.run(x, slow, sband);
        roarBand.run(x, rlow, rband);
        thump *= thumpFade;
        smack *= smackFade;
        advance(tailPhase, tailHz);
        mix[i] = band * thump * 2.8f + sband * smack * 2.4f + lookup(g_blade, tailPhase) * tail +
                 whine(turbine, whistleHz, false, rng) * whistle + rlow * 0.1f * load;
    }
    thumpBand_ = thumpBand;
    slapBand_ = slapBand;
    roar_ = roarBand;
    turbine_ = turbine;
    bladePhase_[0] = blade;
    tailPhase_ = tailPhase;
    thump_ = thump;
    smack_ = smack;
    secondRotor_ = second;
    seed_ = rng.s;
}

}  // namespace engine_noise
