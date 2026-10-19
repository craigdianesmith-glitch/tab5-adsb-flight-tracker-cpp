#pragma once

#include <stdint.h>

// The 3D view's engine noise: what the aircraft followed sounds like from
// where the camera is - made as it plays, no recordings, out of noise, tones
// and a few filters, as a sound designer would rough one out (as
// tools/gen_jet.py makes the splash screen's take-off ahead of time). Its
// engines' kind and how hard they are working; how far off it is - the
// further, the quieter and the more muffled, the air taking the treble
// first; how fast it is coming or going - its pitch up as it comes and down
// as it goes, past a camera standing still; and which way it faces - a jet's
// fan whining ahead of it and its exhaust roaring behind. The Tab5's speaker
// has nothing much below 300Hz, so the weight is in the mid-range rather
// than a rumble it couldn't play.
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
};

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
    void restart();
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
    // A whine as an engine makes one - a fan's, a turbine's: a steady tone,
    // its pitch drifting a shade, and for the fan's noise in a narrow band
    // about it. Two pure tones a little apart, beating, were an alarm; a
    // tone swelling and falling a little at random, pulsing.
    struct Whine {
        float phase = 0, wander = 0, drift = 1;
        Svf band;
    };
    float whine(Whine &w, float hz, bool narrow);
    void wander(Whine &w);

    Sound target_, sound_;  // what was asked for, and the kind playing - target_'s once the fade allows
    bool under_ = false;
    uint32_t seed_ = 0x1234567;
    // Eased toward target_.
    float power_ = 0, reverse_ = 0, burner_ = 0, slap_ = 0;
    float logDistance_ = 3.9f, closing_ = 0, aspect_ = 0.5f;
    float fade_ = 0, duck_ = 1;
    // Worked out each few samples from those.
    float doppler_ = 1, gain_ = 0;
    Svf air_;
    float dcLow_ = 0;
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
