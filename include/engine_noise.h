#pragma once

#include <stdint.h>

// The 3D view's engine noise: what the aircraft followed sounds like from
// where the camera is - made as it plays, out of noise, tones and a few
// filters, as a sound designer would rough one out (as tools/gen_jet.py
// makes the splash screen's take-off ahead of time), and a fighter's roar
// out of recordings of real ones, cut by tools/gen_roar.py. Its
// engines' kind and how hard they are working, and its tyres on the ground;
// and the sound's way to the ear, as the air carries it:
//  - late: at 343m a second, what is heard is what the aircraft was doing
//    a moment ago, from where it was - a second late from 343m - and its
//    pitch up as it comes and down as it goes, by how fast it was coming;
//  - spreading, a sixth weaker for each doubling of the distance, and the
//    air taking the treble first, as ISO 9613-1 has it: a kilometre off,
//    nothing much above a kilohertz;
//  - off the ground as well as straight, the echo a little later and
//    weaker, the two adding and cancelling by turns up the low end of the
//    spectrum - the sweep in the roar of a jet's flypast, as the difference
//    between the two ways changes - and scattered higher up, as the
//    ground's roughness and the air's eddies scatter it;
//  - shimmering, far off, as the air's eddies bend it this way and that.
// Which way it faces too - a jet's fan whining ahead of it and its exhaust
// roaring behind. The Tab5's speaker has nothing much below 300Hz, so the
// weight is in the mid-range rather than a rumble it couldn't play, and
// the loudness squeezed into what it can - see LEVEL.
//
// Plain C++, for the PC's preview (tools/engine_preview) as well as the
// device, where sound.cpp plays it on a channel of its own under the words.
namespace engine_noise {

constexpr int RATE = 16000;  // samples a second

enum class Kind : uint8_t {
    NONE,        // nothing: fades out
    TURBOFAN,    // an airliner's, a business jet's: the fans' whine, the roar, the buzz-saw at full power
    LOW_BYPASS,  // a fighter's, a trainer's, an old jet's: mostly roar, and the afterburner's crackle
    TURBOPROP,   // the propellers' drone, beating against each other, and the turbines' whine
    PISTON,      // a light aircraft's: its exhaust's putter and its propeller
    ROTOR,       // a helicopter's: its blades' chop, its tail rotor and its turbine
};

struct Sound {
    Kind kind = Kind::NONE;
    // Which aircraft: another - even of the same kind - faded in afresh, as
    // it is, rather than this one's engines and APU carried over to it.
    uint32_t who = 0;
    // The engines, from its type.
    uint8_t engines = 2;
    float fanHz = 3000;   // a turbofan's fan blades passing at full power
    float bladeHz = 120;  // a propeller's blades passing at full power; a helicopter's main rotor's
    float tailHz = 110;   // a helicopter's tail rotor's blades passing
    bool tandem = false;  // two main rotors, fore and aft, passing in turn
    // What they are doing.
    float power = 0;  // 0 idle to 1 take-off - for a helicopter, how hard its rotor is working
    bool reverse = false;
    bool afterburner = false;
    float slap = 0;  // a helicopter's blade slap, 0 to 1: coming down, slowing
    // Where they are heard from.
    float distanceM = 50;
    float closingMs = 0;  // how fast it is coming closer, m/s: going away, less than nought
    float aspect = 0.5f;  // 0 heard from dead ahead of it, 1 from dead astern
    // Its engines' height above the ground, and the ear's - the camera's -
    // for the sound off the ground; and whether that is hard, a runway's
    // or an apron's, or grass and earth, which give a little and take more
    // of the treble.
    float sourceUpM = 0, earUpM = 1.5f;
    bool hardGround = false;
    // Its tyres: rolling along the ground at `rollingMs`, nought in the air;
    // and touched down `touchdownS` seconds ago - less than nought, not
    // lately - coming down at `sinkMs` as they did.
    float rollingMs = 0, touchdownS = -1, sinkMs = 1;
    // Its airframe: how fast the air goes past it, m/s, and how it is set
    // up - the gear down, the flaps out, the spoilers up, 0 to 1 - for the
    // air tearing past them: on an approach, gear and flaps out and the
    // engines throttled back, as loud as the engines.
    float airspeedMs = 0;
    bool gearDown = false, flapsOut = false;
    float spoilers = 0;
    // And how big it is, its length against an A320's: the air tears past
    // less of a small one - a fighter's a third as much.
    float size = 1;
    // Its engines running, or shut down - at its stand - and started again,
    // heard spooling up from nothing; and its APU, an airliner's, whining
    // while it stands or taxies.
    bool running = true, apu = false;
    // Where its engines and the ear - the camera - are, in metres in a frame
    // of the caller's own, the same from one call to the next, and which way
    // it points. `placed`, the synthesiser works out the sound's way from
    // them itself - when what is heard now left it, how far it had come and
    // how fast it was closing then, and which way it faced - and distanceM,
    // closingMs and aspect are left aside.
    bool placed = false;
    float at[3] = {}, nose[3] = {1, 0, 0}, ear[3] = {};
    // When those were so, milliseconds on the caller's clock: the frame's
    // time, which the sound's blocks, made in their own time, aren't - by
    // theirs, each place was one, two or three frames on from the last,
    // its speed jumping by half, and its sound gusting with it.
    uint32_t ms = 0;
};

// Airliners', turboprops' and light aircraft's engines from recordings -
// each at low power and at high, by their power - rather than synthesised:
// to compare the two, by ear, on the device. Off, as approved.
extern bool g_recordedVoices;

class Synth {
  public:
    Synth();
    // What it is to sound like from now, eased into: the engines spooling
    // up and down as engines do. Another kind - another aircraft - fades
    // the one before out first; Kind::NONE fades it out and leaves it.
    void set(const Sound &sound);
    // Down to a murmur under the words - an alert's chime, a clearance - or
    // back up.
    void duck(bool under);
    // The next `count` samples, at RATE. False once it has faded to
    // silence: there is nothing more to play until set() again.
    bool render(int16_t *out, int count);

  private:
    void update();
    void placedWay();
    void restart();
    void tyres(float *mix, int n);
    void airframe(float *mix, int n);
    void apu(float *mix, int n);
    void turbofan(float *mix, int n);
    void turboprop(float *mix, int n);
    void piston(float *mix, int n);
    void rotor(float *mix, int n);
    float noise();
    float uniform();

    // A state-variable filter, done the trapezoidal way: steady at any
    // cutoff below half the rate.
    struct Svf {
        float ic1 = 0, ic2 = 0, k = 1, a1 = 0, a2 = 0, a3 = 0;
        float hz = -1, q = 0;  // as last tuned
        void tune(float toHz, float toQ);
        void run(float x, float &low, float &band);
        void clear() { ic1 = ic2 = 0; }
    };
    // The noise as a step's samples have it: the seed in a register, copied
    // in and back once a step, not read from the synth - in the PSRAM - and
    // written back at every draw. The same draws, in the same order.
    struct Rng {
        uint32_t s;
        float uniform() {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            return (s >> 8) * (1.0f / 16777216.0f);
        }
        float noise() { return uniform() * 2 - 1; }
    };
    // A whine as an engine makes one - a fan's, a turbine's: a steady tone,
    // its pitch drifting a shade, and for the fan's noise in a narrow band
    // about it. Two pure tones a little apart, beating, were an alarm; a
    // tone swelling and falling a little at random, pulsing.
    struct Whine {
        float phase = 0, wander = 0, drift = 1;
        Svf band;
    };
    static float whine(Whine &w, float hz, bool narrow, Rng &rng);
    void wander(Whine &w);

    Sound target_, sound_;  // what was asked for, and the kind playing - target_'s once the fade allows
    // What was asked for over the last few seconds, a snapshot each 20ms
    // or so - each block rendered, on the device each 40ms: ten seconds, a
    // sound's way from three and a half kilometres - each with the sample
    // it was asked at: what is heard is the one from as long ago as the
    // sound takes to come.
    static constexpr int HEARD = 256;
    struct Asked {
        uint32_t at = 0;
        Sound sound;
    };
    Asked asked_[HEARD];
    int askedNewest_ = -1, askedCount_ = 0;
    uint32_t clock_ = 0;  // samples rendered
    float delayS_ = 0;    // how long the sound takes to come, eased
    Sound heard_;         // what it says now, as asked that long ago
    bool under_ = false;
    uint32_t seed_ = 0x1234567;
    // Eased toward target_.
    float power_ = 0, reverse_ = 0, burner_ = 0, slap_ = 0;
    float logDistance_ = 3.9f, closing_ = 0, aspect_ = 0.5f;
    float convection_ = 1, convectionTo_ = 1;  // placed: louder coming, quieter going
    // A recording played a stretch at a time from places picked at random,
    // each faded into the next, so it runs on with no beat of its own: where
    // in it - 20.12 fixed point - and where the next stretch is from, how far
    // into fading to it, and how long till it; where this stretch and the one
    // before began, for the next to be from elsewhere; and, a roar's, which
    // way each plays and how much faster or slower.
    struct Grains {
        uint32_t at = 0, next = 0, from = 0, was = 0;
        int fading = -1, left = 0;
        bool back = false, nextBack = false;
        float rate = 1, nextRate = 1;
    };
    Grains roar_g_, front_g_;  // a fighter's recorded roar and front
    Grains takeoff_g_, approach_g_;  // an airliner's recorded take-off and approach
    Grains low_g_, high_g_;          // a turboprop's recorded approach and take-off
    Grains piston_g_;                // a light aircraft's recorded
    static float grain(Grains &g, const int16_t *loop, uint32_t length, uint32_t step, Rng &rng, bool vary = false);
    float fade_ = 0, duck_ = 1;
    // Worked out each few samples from those.
    float doppler_ = 1, gain_ = 0;
    Svf air_;
    float dcLow_ = 0;
    // The sound off the ground: the last 32ms, the echo's lateness in
    // samples - stepped across each STEP from the last's - its strength, its
    // low-pass's coefficient and two stages, and its lateness's wander.
    static constexpr int ECHO = 512;
    float echo_[ECHO] = {};
    int echoAt_ = 0;
    float echoLate_ = 0, echoLateWas_ = 0, echoGain_ = 0, echoSoft_ = 0, echoLow_ = 0, echoLow2_ = 0, echoWander_ = 0;
    // The air's eddies, far off: the level wandering, smoothed noise.
    float shimmer_ = 0, shimmerWas_ = 0;
    // A recorded roar's own surges, dB, and the one it is easing to - held
    // that long yet, in samples.
    float surgeDb_ = 0, surgeTo_ = 0;
    int surgeLeft_ = 0;
    // The tyres': their screech - each main gear's in turn, its pitch
    // gliding down as the wheels spin up - their roll, and the nose wheel
    // over each centreline light.
    float touchdownWas_ = -1, screech_[2] = {}, screechAt_[2] = {}, screechS_ = 0, screechLoud_ = 0;
    float lightPhase_ = 0;
    Svf screechBand_[2], rollBand_, lightBand_;
    // The airframe's: the gear's, the flaps' and the spoilers' share of its
    // rush, eased in and out as they move; the air past it; and the
    // spoilers' buffet wandering.
    float gear_ = 0, flaps_ = 0, spoilers_ = 0, airspeed_ = 0, buffet_ = 0;
    Svf rushBand_, flapBand_, buffetBand_;
    // The engines' spool, 0 stopped to 1 at idle or above - rising as they
    // start, the core on its starter to a third of idle, lit there, then up
    // to idle; winding down when shut down - and how much their fire is
    // heard, swelling as they light and gone as the fuel is cut. The APU's
    // spool, its compressor's whine and its exhaust's hiss.
    float spool_ = 1, lit_ = 1, apu_ = 0;
    Whine apuTone_;
    Svf apuBand_;
    // The sounds' own: their oscillators, envelopes and filters.
    float shaftPhase_ = 0;
    Whine fan_, fan2_, turbine_;
    float bladePhase_[2] = {}, tailPhase_ = 0, firePhase_ = 0;
    float thump_ = 0, smack_ = 0, crackle_ = 0, kick_ = 0;
    int cylinder_ = 0;
    bool secondRotor_ = false;
    Svf roar_, body_, buzz_, formant1_, formant2_, thumpBand_, slapBand_, crackleBand_;
};

}  // namespace engine_noise
