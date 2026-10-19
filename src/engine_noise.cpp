#include "engine_noise.h"

#include <math.h>
#include <stdlib.h>

#include <algorithm>

#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

namespace engine_noise {

namespace {

constexpr float PI = 3.14159265f;
// The controls - loudness, pitch, the filters - worked out again every this
// many samples, 2ms: often enough for none of them to be heard stepping.
constexpr int STEP = 32;
constexpr float STEP_S = STEP / (float)RATE;
constexpr float SOUND_MS = 343;  // the speed of sound
// The mix's level, as samples go: an airliner cruising, from the chase, a
// steady -22dBFS; taking off past a camera beside the runway about the same,
// brighter; a fighter's afterburner going by, -10. Under the words, which
// are near full scale.
constexpr float LEVEL = 1.6f;

float clamp01(float x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

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

inline float lookup(const float *table, float phase) {
    float x = phase * TABLE;
    int i = (int)x;
    return table[i] + (table[i + 1] - table[i]) * (x - i);
}

// A phase moved on a sample's worth at `hz`, kept between 0 and 1. True as
// it comes round.
inline bool advance(float &phase, float hz) {
    phase += hz * (1.0f / RATE);
    if (phase < 1) {
        return false;
    }
    phase -= floorf(phase);
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
void Synth::Svf::run(float x, float &low, float &band) {
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

void Synth::set(const Sound &sound) { target_ = sound; }

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
float Synth::whine(Whine &w, float hz, bool narrow) {
    advance(w.phase, hz * w.drift);
    float y = lookup(g_sine, w.phase) * 0.5f;
    if (narrow) {
        float low, band;
        w.band.run(noise(), low, band);
        y += band * 1.2f;
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
    power_ = clamp01(target_.power);
    reverse_ = target_.reverse ? 1.0f : 0.0f;
    burner_ = target_.afterburner ? 1.0f : 0.0f;
    slap_ = clamp01(target_.slap);
    logDistance_ = logf(std::max(5.0f, target_.distanceM));
    closing_ = std::max(-150.0f, std::min(150.0f, target_.closingMs));
    aspect_ = clamp01(target_.aspect);
    for (Svf *f : {&roar_, &body_, &buzz_, &formant1_, &formant2_, &thumpBand_, &slapBand_, &crackleBand_, &fan_.band,
                   &fan2_.band, &turbine_.band}) {
        f->clear();
    }
    thump_ = smack_ = crackle_ = kick_ = 0;
}

void Synth::update() {
    // Another kind - another aircraft, or none - once the one before has
    // faded out; the same kind, its type's figures as they come.
    bool same = target_.kind == sound_.kind;
    if (!same && fade_ < 0.003f) {
        sound_.kind = target_.kind;
        same = true;
        restart();
    }
    if (same) {
        sound_ = target_;
    }
    bool on = same && sound_.kind != Kind::NONE;
    static const float FADE_IN = stepOf(0.5f), FADE_OUT = stepOf(0.2f), DUCK = stepOf(0.12f), JET_UP = stepOf(1.8f),
                       JET_DOWN = stepOf(2.5f), PROP_UP = stepOf(0.6f), PROP_DOWN = stepOf(0.8f),
                       REVERSE = stepOf(0.8f), BURNER = stepOf(0.4f), SLAP = stepOf(0.6f), WHERE = stepOf(0.12f),
                       CLOSING = stepOf(0.25f), ASPECT = stepOf(0.2f);
    fade_ = ease(fade_, on ? 1.0f : 0.0f, on ? FADE_IN : FADE_OUT);
    duck_ = ease(duck_, under_ ? 0.2f : 1.0f, DUCK);
    // Engines spool up and down over seconds - a jet's slower than a
    // propeller's - and a jet's slower down than up.
    bool jet = sound_.kind == Kind::TURBOFAN || sound_.kind == Kind::LOW_BYPASS;
    float spool = target_.power > power_ ? (jet ? JET_UP : PROP_UP) : (jet ? JET_DOWN : PROP_DOWN);
    power_ = ease(power_, clamp01(target_.power), spool);
    reverse_ = ease(reverse_, target_.reverse ? 1.0f : 0.0f, REVERSE);
    burner_ = ease(burner_, target_.afterburner ? 1.0f : 0.0f, BURNER);
    slap_ = ease(slap_, clamp01(target_.slap), SLAP);
    // Where it is heard from: eased a little, so a cut from one camera to
    // another isn't a click.
    logDistance_ = ease(logDistance_, logf(std::max(5.0f, target_.distanceM)), WHERE);
    closing_ = ease(closing_, std::max(-150.0f, std::min(150.0f, target_.closingMs)), CLOSING);
    aspect_ = ease(aspect_, clamp01(target_.aspect), ASPECT);
    float d = expf(logDistance_);
    doppler_ = SOUND_MS / (SOUND_MS - closing_);
    wander(fan_);
    wander(fan2_);
    wander(turbine_);
    // Quieter the further it is - less so than the square of the distance
    // would have it, for a small speaker on a desk - and the air taking the
    // treble first: muffled a kilometre off.
    gain_ = powf(35 / std::max(35.0f, d), 0.65f);
    air_.tune(std::min(7000.0f, std::max(250.0f, 7000 / (1 + d / 300))), 0.6f);
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
        // The level carried smoothly across the step from the last; the air's
        // muffling; nothing under 180Hz, which the speaker can't play; and a
        // peak rounded off rather than cut.
        float to = gain_ * fade_ * duck_ * LEVEL, by = (to - level) / n;
        for (int i = 0; i < n; i++) {
            float low, band;
            air_.run(mix[i], low, band);
            level += by;
            float y = low * level;
            dcLow_ += dc * (y - dcLow_);
            y -= dcLow_;
            float a = fabsf(y);
            if (a > 0.6f) {
                y = copysignf(0.6f + 0.4f * tanhf((a - 0.6f) / 0.4f), y);
            }
            out[done + i] = (int16_t)(y * 32000 + (y < 0 ? -0.5f : 0.5f));
        }
        level = to;
    }
    return sound_.kind != Kind::NONE;
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
    float n1 = 0.22f + 0.76f * power_;
    // The fan heard ahead of it, the exhaust most at 40 degrees off its tail.
    float ahead = 1 - 0.75f * smoothstep(0.35f, 0.85f, aspect_);
    float behind = std::max(0.25f + 0.75f * smoothstep(0.15f, 0.8f, aspect_), reverse_);
    float fan = sound_.fanHz * n1 * doppler_;
    float fanLevel =
        (lowBypass ? 0.04f : 0.12f) * (0.5f + 0.5f * n1) * (1 - 0.55f * smoothstep(0.7f, 0.95f, n1)) * ahead;
    float fan2Level = 2 * fan < 7400 ? fanLevel * 0.25f : 0;
    fan_.band.tune(fan, 18);
    float saw = lowBypass ? 0 : 0.35f * smoothstep(0.8f, 0.95f, n1) * ahead;
    float shaft = fan / 32;  // the fan's turning: blades go by some 32 times a turn
    float roar = (0.06f + 0.94f * powf(n1, 3.5f)) * behind * (lowBypass ? 1.25f : 0.85f) *
                 (sound_.engines >= 4 ? 1.2f : 1) * (1 + 0.5f * reverse_) * (1 + 0.9f * burner_);
    roar_.tune((250 + (lowBypass ? 2600 : 1800) * powf(n1, 1.5f) + 1200 * burner_) * doppler_, 0.6f);
    body_.tune(650 * doppler_ * (0.7f + 0.5f * n1), 1.2f);
    buzz_.tune(1500 * doppler_, 0.8f);
    crackleBand_.tune(2500 * doppler_, 0.7f);
    float whistle = 0.025f * (1 - 0.5f * n1) * ahead, whistleHz = (5200 + 1600 * n1) * doppler_;
    for (int i = 0; i < n; i++) {
        float x = noise(), low, band, low2, band2;
        roar_.run(x, low, band);
        body_.run(x, low2, band2);
        float y = (low * 0.8f + band2 * 0.7f) * roar;
        y += whine(fan_, fan, true) * fanLevel;
        if (fan2Level > 0) {
            y += whine(fan2_, 2 * fan, false) * fan2Level;
        }
        if (saw > 0) {
            advance(shaftPhase_, shaft);
            float blow, bband;
            buzz_.run(lookup(g_buzz, shaftPhase_), blow, bband);
            y += bband * saw;
        }
        y += whine(turbine_, whistleHz, false) * whistle;
        if (burner_ > 0.02f) {
            if (uniform() < 0.003f * burner_) {
                kick_ = (0.5f + 0.5f * uniform()) * burner_;
            }
            kick_ *= 0.8f;
            float clow, cband;
            crackleBand_.run(kick_ * noise(), clow, cband);
            y += cband * 6 * behind;
        }
        mix[i] = y;
    }
}

// A turboprop: its propellers' blades going by - a pulse rich in harmonics,
// the drone - two of them a shade apart in speed, phasing slowly against
// each other (further apart, the beat pulsed); the roar of the
// air through them, the more with the power; and the turbines' whine. Their
// speed held nearly steady, as a constant-speed propeller's is, the power
// in how hard the blades bite.
void Synth::turboprop(float *mix, int n) {
    float np = 0.72f + 0.28f * power_;
    float blades = sound_.bladeHz * np * doppler_;
    float prop = 1.2f * (0.35f + 0.65f * power_) * (sound_.engines >= 4 ? 1.2f : 1) * (1 + 0.5f * reverse_);
    float roar = 0.3f * (0.2f + 0.8f * power_ * power_) * (1 + 1.2f * reverse_);
    roar_.tune(900 * (0.6f + 0.4f * power_) * doppler_, 0.7f);
    float whistle = 0.03f, whistleHz = (5600 + 900 * power_) * doppler_;
    for (int i = 0; i < n; i++) {
        advance(bladePhase_[0], blades);
        advance(bladePhase_[1], blades * 1.0012f);
        float low, band;
        roar_.run(noise(), low, band);
        mix[i] = (lookup(g_blade, bladePhase_[0]) + lookup(g_blade, bladePhase_[1])) * 0.5f * prop + low * roar +
                 whine(turbine_, whistleHz, false) * whistle;
    }
}

// A light aircraft's piston engine: four cylinders firing twice a turn, each
// a little differently and not quite evenly - the exhaust's bark through
// the pipe's two resonances - and its two-bladed propeller going by as
// often.
void Synth::piston(float *mix, int n) {
    static const float CYLINDER[4] = {1.0f, 0.86f, 0.95f, 0.8f};
    float rpm = 900 + 1800 * power_;
    float fire = rpm / 30 * doppler_;
    formant1_.tune(420 * doppler_, 2.5f);
    formant2_.tune(1150 * doppler_, 3);
    float exhaust = 2.5f * (0.3f + 0.7f * power_), prop = 0.3f * (0.3f + 0.7f * power_);
    for (int i = 0; i < n; i++) {
        float x = 0;
        if (advance(firePhase_, fire)) {
            firePhase_ -= 0.04f * uniform();  // the next a shade late, or not
            cylinder_ = (cylinder_ + 1) & 3;
            kick_ = CYLINDER[cylinder_] * (0.85f + 0.3f * uniform());
            x = kick_ * 4;
        }
        kick_ *= 0.995f;
        x += kick_ * noise() * 0.6f;
        float low1, band1, low2, band2;
        formant1_.run(x, low1, band1);
        formant2_.run(x, low2, band2);
        advance(bladePhase_[0], fire);
        mix[i] = (band1 + 0.6f * band2) * exhaust + lookup(g_blade, bladePhase_[0]) * prop;
    }
}

// A helicopter: each of its main rotor's blades going by a thump - a burst
// of low noise dying away before the next, the "wop" - the harder the more
// it is working; coming down, slowing, the sharp slap of a blade meeting
// the one before's wake; two rotors' blades in turn. Its tail rotor's buzz,
// its turbine's whine, and the broad rush of air under it all.
void Synth::rotor(float *mix, int n) {
    float passes = sound_.bladeHz * doppler_ * (sound_.tandem ? 2 : 1);
    float load = 0.35f + 0.65f * power_;
    thumpBand_.tune(420 * doppler_, 0.9f);
    slapBand_.tune(1600 * doppler_, 0.9f);
    roar_.tune(1100 * doppler_, 0.7f);
    float tail = sound_.tailHz > 0 ? 0.08f * (0.5f + 0.5f * power_) : 0, tailHz = sound_.tailHz * doppler_;
    float whistle = 0.02f, whistleHz = 6400 * doppler_;
    const float thumpFade = expf(-1 / (0.022f * RATE)), smackFade = expf(-1 / (0.0035f * RATE));
    for (int i = 0; i < n; i++) {
        if (advance(bladePhase_[0], passes)) {
            secondRotor_ = !secondRotor_;
            float which = sound_.tandem && secondRotor_ ? 0.8f : 1;
            thump_ = load * which * (0.9f + 0.2f * uniform());
            smack_ = slap_ * which * (0.85f + 0.3f * uniform());
        }
        float x = noise(), low, band, slow, sband, rlow, rband;
        thumpBand_.run(x, low, band);
        slapBand_.run(x, slow, sband);
        roar_.run(x, rlow, rband);
        thump_ *= thumpFade;
        smack_ *= smackFade;
        advance(tailPhase_, tailHz);
        mix[i] = band * thump_ * 2.8f + sband * smack_ * 2.4f + lookup(g_blade, tailPhase_) * tail +
                 whine(turbine_, whistleHz, false) * whistle + rlow * 0.1f * load;
    }
}

}  // namespace engine_noise
