#pragma once

#include <stdint.h>

// The 3D view's engine noise: what the aircraft followed sounds like from
// where the camera is - made as it plays, with no recording anywhere in it.
// For now an airliner of the A320 family's: its CFM56s or V2500s, its LEAPs
// or geared fans, its airframe, tyres and APU.
//
// What it is made of is noise, and what noise does through a resonance. A
// tone heard from the ground is never a sine: a fan's blade-passing whine
// comes out of the intake as a tangle of spinning duct modes, stirred by the
// air it swallows, then bent and scattered by the eddies on the way - and
// arrives as a narrow band of noise about its pitch, swelling and fading at
// random a few times a second, as narrow-band noise does. Measured on real
// A320s the whine's level wanders 4.7 to 5.3dB, half as much again as its
// mean - which is what noise through a resonance a few hertz wide gives of
// itself (a Rayleigh envelope), and what a steady oscillator, however it is
// shaken, doesn't. So every tone here is white noise through a resonance:
//  - the fan's blade-passing tone, each engine's, at its blades times its
//    shaft's turning, and its harmonics;
//  - each engine's core compressor's whine, rising as it starts, idling at
//    3.6kHz;
//  - the buzz-saw at take-off power, the fan's tips past the speed of sound:
//    a line at every harmonic of the shaft, as a noise driven comb - a delay
//    of one turn, fed back, each turn the last one's shocks a little changed
//    - its noise given the fan's own pattern, so each harmonic is as strong
//    as this airframe's blades make it, the same each time it is heard;
//  - the nose gear's cavity tone at 1.7kHz on the approach, and the fuel
//    vents' two-tone whistle on older airframes, flow over a hole ringing as
//    a bottle does - as Empa found they must be made to sound real;
//  - a V2500's turbine tones behind it, smeared broad by the jet;
//  - the APU's whine, and the tyres' squeal at touchdown.
// The rest - the jet's roar, the fan's broadband hiss, the core's rumble,
// the air tearing past the gear and flaps, the reversers - is noise through
// a fixed bank of eight bands, 250Hz to 6.3kHz, each band as loud as the
// sources in it make it. Each source surges as it does - the jet's roar
// swelling in all its bands together, the fan's hiss out of its intake
// barely - and the air's flicker moves the treble together, the more the
// faster the aircraft crosses the way the sound comes; so a band's churn,
// and how far it moves with the others, comes from which sources fill it
// and where it is heard from: a take-off coming, bands churning apart; as
// it passes and goes, together - as the recordings' octaves do.
//
// And the sound's way to the ear, as the air carries it:
//  - late: at 343m a second, what is heard is what the aircraft was doing
//    a moment ago, from where it was - a second late from 343m - and its
//    pitch up as it comes and down as it goes, by how fast it was coming;
//  - spreading, a sixth weaker for each doubling of the distance, and the
//    air taking the treble first, as ISO 9613-1 has it, band by band and
//    tone by tone: a kilometre off, nothing much above a kilohertz;
//  - off the ground as well as straight, the echo a little later and
//    weaker, the two adding and cancelling by turns up the spectrum - the
//    sweep in the roar of a jet's flypast - and scattered higher up;
//  - held back low over the ground, as a take-off roll seen from the side
//    is, and opening up as it climbs;
//  - flickering, the further the more, as the air's eddies focus and
//    scatter it - the treble most, the whole band a kilometre off.
// Which way it faces too - a fan's whine and its buzz-saw ahead of it, its
// exhaust roaring behind. The Tab5's speaker has nothing much below 300Hz,
// so the weight is in the mid-range rather than a rumble it couldn't play,
// and the loudness squeezed into what it can - see LEVEL.
//
// Plain C++, for the PC's preview (tools/engine_preview) as well as the
// device, where sound.cpp plays it on a channel of its own under the words.
namespace engine_noise {

constexpr int RATE = 16000;  // samples a second

enum class Kind : uint8_t {
    NONE,        // nothing: fades out
    TURBOFAN,    // an airliner's: the fans' whine, the roar, the buzz-saw at full power
    LOW_BYPASS,  // a fighter's, a trainer's: not made yet - silent
    TURBOPROP,   // not made yet - silent
    PISTON,      // not made yet - silent
    ROTOR,       // not made yet - silent
};

// Which engines a turbofan has, where its type alone doesn't say: an A320
// may have either of two, its neo either of two others - see a320Engines().
enum class Engine : uint8_t {
    GENERIC,   // known only by its fan's blades passing at full power, Sound::fanHz
    CFM56_5B,  // A319/A320/A321ceo: 36 fan blades, 5,000rpm at 100% N1
    V2500_A5,  // A319/A320/A321ceo: 22 blades, 5,650rpm; its turbine heard behind it
    LEAP_1A,   // A319/A320/A321neo: 18 blades, 3,856rpm
    PW1100G,   // A319/A320/A321neo: 20 blades, the fan geared down to 3,281rpm at most
};

struct Sound {
    Kind kind = Kind::NONE;
    // Which aircraft: another - even of the same kind - faded in afresh, as
    // it is, rather than this one's engines and APU carried over to it. Its
    // own little differences come from it too - its two engines' speeds a
    // shade apart, which of its buzz-saw's lines are strongest, its nose
    // gear's and fuel vents' pitch - the same each time it is heard.
    uint32_t who = 0;
    // The engines, from its type.
    uint8_t engines = 2;
    float fanHz = 3000;   // a turbofan's fan blades passing at full power - an Engine::GENERIC one's
    float bladeHz = 120;  // a propeller's blades passing at full power; a helicopter's main rotor's
    float tailHz = 110;   // a helicopter's tail rotor's blades passing
    bool tandem = false;  // two main rotors, fore and aft, passing in turn
    Engine engine = Engine::GENERIC;
    // Its fuel tanks' overpressure vents unmodified, as on A320s built before
    // the vortex generators of 2014 and not retrofitted: their two-tone
    // whistle on the approach at 165 to 230 knots - see a320Fopp().
    bool fopp = false;
    // What they are doing.
    float power = 0;  // 0 idle to 1 take-off - for a helicopter, how hard its rotor is working
    // The fan's speed as a share of its 100% - if known; otherwise, less than
    // nought, worked out from power and what the aircraft is doing, by the
    // engine's own: an A320's CFM56s at 90% for take-off, 51% on the approach.
    float n1 = -1;
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
    // And how big it is, its length against an A320's.
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

// An A320-family type's engines, for this airframe: ADS-B doesn't say which,
// so they are picked from `who` - the same airframe always the same - in the
// fleets' shares: a ceo's (A318, A319, A320, A321) CFM56-5Bs 55 times in a
// hundred, V2500s the rest; a neo's (A19N, A20N, A21N) LEAP-1As 60 times in
// a hundred, PW1100Gs the rest. Engine::GENERIC for any other type.
Engine a320Engines(const char *type, uint32_t who);
// Whether this airframe has its fuel vents unmodified: a ceo, 40 times in a
// hundred, by `who`; never a neo, all built with the fix.
bool a320Fopp(Engine engine, uint32_t who);

// The touchdown's squeal and the reversers' coming on heard as the picture
// shows them - a tenth of a second late at most - rather than as late as the
// sound takes to come: from 300m that is nine tenths of a second, and on a
// screen, with nothing to say how far off it is, it reads as late. The rest,
// the engines' and the airframe's sound, is as late as it is, always - its
// pitch and its swell as the aircraft passes depend on it. Off: all as late
// as the sound takes.
extern bool g_transientsOnPicture;

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

    // The number of resonances and bands it is running now - for the bench.
    int busy() const { return tonesOn_ + (combOn_[0] ? 4 : 0) + (combOn_[1] ? 4 : 0) + BANDS; }

    static constexpr int BANDS = 8;    // the broadband's bands
    static constexpr int TONES = 15;   // the resonances' places - see tonesStep()
    static constexpr int SOURCES = 8;  // the broadband's sources, each surging its own way

  private:
    void update();
    void placedWay();
    void restart();
    void engines();
    float wantN1(const Sound &s, const Sound &rev) const;
    void slowStep();
    void bandsStep();
    void tonesStep();
    void combsStep();
    void tyresStep();

    // The noise as a step's samples have it: the seed in a register, copied
    // in and back once a step, not read from the synth - in the PSRAM - and
    // written back at every draw.
    // A linear congruential generator's: its top 23 bits made a float's
    // mantissa straight, no conversion - its low bits, poor, never used.
    struct Rng {
        uint32_t s;
        float noise() {  // -1 to 1, a third of a square's power
            s = s * 1664525u + 1013904223u;
            union {
                uint32_t u;
                float f;
            } v;
            v.u = (s >> 9) | 0x40000000u;  // 2 to 4
            return v.f - 3.0f;
        }
        float uniform() { return 0.5f * (noise() + 1); }  // 0 to 1
        // Near enough Gaussian, unit variance: three of those added.
        float gauss() { return noise() + noise() + noise(); }
    };
    // A state-variable filter, done the trapezoidal way: steady at any
    // cutoff below half the rate, and tuned again only as its cutoff moves.
    struct Svf {
        float ic1 = 0, ic2 = 0, k = 1, a1 = 0, a2 = 0, a3 = 0;
        float hz = -1, q = 0;
        void tune(float toHz, float toQ);
        void clear() { ic1 = ic2 = 0; }
    };
    // A resonance: a turning pointer, shrinking a little each sample - by as
    // much as its width in hertz makes it - and pushed each sample by the
    // noise: its real part rings at its pitch, its loudness wandering as
    // narrow-band noise's does. `cr`, `ci` the turn and shrink, `in` how
    // hard the noise pushes it; `stream` which of the two noises.
    struct Tone {
        float zr = 0, zi = 0, cr = 0, ci = 0, in = 0;
        float dr = 1, di = 0, turn = 0;  // its turn's own turn each step, toward its pitch's next; that pitch, in turns
        float hz = 0;  // its pitch as heard, for the step's sums
        uint8_t stream = 0;
        bool on = false;
        bool wide = false;  // tens of hertz wide: through the air's muffling with the broadband
    };
    // The buzz-saw of each engine: a turn's worth of what it radiated, fed
    // back through a delay of one turn - a line at every harmonic of the
    // shaft - each turn the last one's a little weaker, `g`, and new noise
    // added; the noise first shaped to where its lines are strongest, a
    // band between two one-pole low-passes' corners - `lo`, `hi` their
    // states, `kLo`, `kHi` their coefficients - and then given its fan's own
    // pattern: the noise of a moment ago, from fractions of a turn back,
    // added to it at strengths of the airframe's own - `tap` how many
    // samples back, `w` how much - so each harmonic of the shaft comes out
    // as strong as its blades' little differences make it, the same each
    // time this aircraft is heard. `noise` the shaped noise's last turn.
    static constexpr int COMB = 512;  // samples: the slowest shaft, 31Hz
    static constexpr int TAPS = 4;
    struct Comb {
        float line[COMB] = {}, noise[COMB] = {};
        float y1 = 0, y2 = 0, g = 0, in = 0, late = 100, lateBy = 0;
        float lo = 0, hi = 0, kLo = 0.1f, kHi = 0.3f;
        float w[TAPS] = {1, 0, 0, 0}, turn[TAPS] = {};  // its pattern: each tap's strength and fraction of a turn
        int tap[TAPS] = {};
    };

    Sound target_, sound_;  // what was asked for, and the kind playing - target_'s once the fade allows
    // What was asked for over the last few seconds, a snapshot each 20ms
    // or so - ten seconds, a sound's way from three and a half kilometres -
    // each with the sample it was asked at: what is heard is the one from
    // as long ago as the sound takes to come.
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
    int placedIn_ = 0;    // its way worked out each other step: this one or not
    bool heardPlaced_ = false;  // heard_ worked out from where it was, lately
    bool under_ = false;
    uint32_t seed_ = 0x1234567;  // the samples' noise
    uint32_t slow_ = 0x9e3779b9;  // the controls' own, drawn once a step
    // Eased toward what is heard.
    float logDistance_ = 3.9f, closing_ = 0, aspect_ = 0.5f, below_ = 0.3f, across_ = 0;
    float convection_ = 1, convectionTo_ = 1;  // placed: louder coming, quieter going
    float acrossTo_ = 0;  // how fast it crosses the line the sound comes along, m/s
    float belowTo_ = 0.3f;
    float fade_ = 0, duck_ = 1;
    float doppler_ = 1, gain_ = 0;
    // The engines: each one's fan speed and how fast it is changing - two
    // spools, a shade apart as no two engines run quite alike; their reverse,
    // the airframe's gear, flaps and spoilers, the air past it.
    float n1_[2] = {0.25f, 0.25f}, n1Rate_[2] = {}, split_ = 0, drift_ = 0;
    float steadyS_ = 2;  // held at 50% N1 before take-off power this long, s; less than nought, not there yet
    float reverse_ = 0, gear_ = 0, flaps_ = 0, spoilers_ = 0, airspeed_ = 0;
    // The engines' spool, 0 stopped to 1 at idle or above, and how much their
    // fire is heard; the APU's spool.
    float spool_ = 1, lit_ = 1, apu_ = 0;
    // The airframe's own: from `who`.
    uint32_t traits_ = 0;
    float nlgHz_ = 1720, foppHz_[2] = {540, 585};
    // The slow processes the sources surge by, in units of their own spread:
    // each a quick one and a slow one together; and each band's own shimmer;
    // and the air's flicker.
    float surgeQuick_[SOURCES] = {}, surgeSlow_[SOURCES] = {}, shimmer_[BANDS] = {}, flicker_ = 0, flickerA_ = 0;
    float shimmerQuick_[BANDS] = {}, shimmerSlow_[BANDS] = {};
    // How much of each is kept from one slow step to the next, and how much
    // new noise comes in: worked out once.
    float surgeKeep_[SOURCES][5] = {}, shimmerKeep_[4] = {};
    // Each band's centre, as octaves from 1kHz.
    float bandLog_[BANDS] = {};
    // The bank: eight bands on two noises, fixed - each noise's last two
    // samples shared - each a band-pass and then a resonance on the same poles, so
    // its skirt above falls 18dB an octave, not 6, and a band's surges stay
    // its own rather than filling the bands above it: each with its two
    // stages' last two outputs, its coefficients, and its loudness now.
    float x1_ = 0, x2_ = 0, u1_ = 0, u2_ = 0;  // the bank's two noises' last two samples
    float bandY1_[BANDS] = {}, bandY2_[BANDS] = {}, bandZ1_[BANDS] = {}, bandZ2_[BANDS] = {};
    float bandB0_[BANDS] = {}, bandA1_[BANDS] = {}, bandA2_[BANDS] = {};
    float bandGain_[BANDS] = {}, bandBy_[BANDS] = {};  // its loudness, and its change each sample
    int slowIn_ = 0;  // steps till the next slow one
    Tone tones_[TONES];
    int tonesOn_ = 0;
    Comb comb_[2];
    int combAt_ = 0;  // where the two buzz-saws' next samples go
    bool combOn_[2] = {};
    float heldDb_ = 0;  // held back low over the ground - Doc 29's lateral attenuation
    // The air's muffling of the broadband and the buzz-saws, as a low-pass -
    // the tones have theirs worked out exactly - and what goes in under 180Hz.
    Svf air_;
    float dcLow_ = 0;
    // The sound off the ground: the last 32ms, the echo's lateness in
    // samples - stepped across each STEP from the last's - its strength, its
    // low-pass's coefficient and two stages, and its lateness's wander.
    static constexpr int ECHO = 512;
    float echo_[ECHO] = {};
    int echoAt_ = 0;
    // Each engine's echo's lateness - 11.5m apart, each at its own distance,
    // their notches at different frequencies, for the tones' sums - and the
    // broadband's one echo between the two, stepped across each STEP from
    // the last's.
    float echoLate_[2] = {}, echoTap_ = 0, echoTapWas_ = 0, echoGain_ = 0, echoSoft_ = 0, echoLow_ = 0,
          echoLow2_ = 0, echoWander_ = 0;
    float echoGamma_ = 0;  // the ground's reflection, for the tones' sums
    float engineM_[2] = {50, 50}, engineUpM_[2] = {3, 3};  // each engine's distance from the ear, and height
    // The tyres: their squeal, each main gear's in turn; the nose wheel over
    // each centreline light.
    float touchdownWas_ = -1, touchdownNowWas_ = -1, squealS_ = 10, squealAt_[2] = {}, squealLoud_ = 0;
    float squealDueS_ = -1;
    float lightPhase_ = 0, thump_ = 0;
    Svf thumpBand_;
};

}  // namespace engine_noise
