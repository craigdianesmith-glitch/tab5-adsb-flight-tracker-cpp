// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

#pragma once

#include <stdint.h>

// The 3D view's engine noise: what the aircraft followed sounds like from
// where the camera is - made as it plays, with no recording anywhere in it.
// For now an airliner of the A320 family's: its CFM56s or V2500s, its LEAPs
// or geared fans, its airframe, tyres and APU. And a fighter's, the
// Eurofighter Typhoon's two reheated EJ200s - its roar churning and tearing,
// crackling behind it at high power - on a path of its own in
// engine_noise.cpp (THE FIGHTER). And a turboprop's, the ATR 72's and ATR
// 42's: its propellers' blades passing - their harmonics each a resonance,
// their speed stepping as its crew set it and gliding between, the two
// propellers locked together in the air and apart on the ground - its
// gearbox's line near a kilohertz, its gas generators' whine, its
// propellers' rush, on a path of its own too (THE TURBOPROP), asked as
// Kind::TURBOPROP, its Engine values, PropData and Sound::np.
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
// swelling in all its bands together, the fan's hiss and the buzz-saw each
// their own way - and the air's flicker moves the treble together, the
// quicker the faster the aircraft crosses the way the sound comes; so a
// band's churn, and how far it moves with the others, comes from which
// sources fill it and where it is heard from: a take-off coming, bands
// churning apart; as it passes and goes, together - as the recordings'
// octaves do. On the approach its autothrust hunts, the fans' whine
// wandering a percent or two in pitch every few seconds, as a recorded
// approach's does.
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
    LOW_BYPASS,  // a fighter's, a trainer's: the Typhoon's EJ200s (Engine::EJ200)
    TURBOPROP,   // a propeller airliner's: the ATR 72's and 42's (PropData)
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
    // The Eurofighter Typhoon's (EUFI), Kind::LOW_BYPASS: two EJ200s side by
    // side in its tail, low-bypass (0.4) and reheated - 60kN dry, 90kN with
    // reheat - its jets supersonic at full power, dry or lit. No reversers.
    // Its fan's blades and speeds aren't published.
    EJ200,
    // The Boeing 777's, Kind::TURBOFAN: the A320's voice on far bigger fans
    // [b777-sources.md 1, 2.2, 9.1] - lower, bigger, its churn slower.
    GE90_94B,   // 777-200ER: 22 blades on a 3.12m fan, 2,261.5rpm at 100% N1
    GE90_115B,  // 777-300ER, -200LR, 777F: 22 blades on a 3.26m fan, 2,355rpm
    TRENT_800,  // 777-200/-200ER/-300: 26 blades on a 2.8m fan, 3,300rpm
    // The ATR's, Kind::TURBOPROP: two Pratt & Whitney Canada PW100s, each
    // turning a Hamilton Sundstrand propeller - a type's engines, propellers
    // and airframe together, one row of PROPS each (see PropData).
    PW127_568F,     // ATR 72-500/-600 (AT75, AT76): six-bladed 568Fs, 3.93m
    PW124_14SF,     // ATR 72-200 (AT72): four-bladed 14SF-11s, 3.96m
    PW127_247F,     // ATR 72-210 (AT73): four-bladed 247Fs
    PW127_568F_42,  // ATR 42-400/-500/-600 (AT44, AT45, AT46): the 72-500's propellers, a smaller airframe
    PW120_14SF,     // ATR 42-300/-320 (AT43): four-bladed 14SF-5s
};

// A propeller aircraft's figures: one row for each installation - a type's
// engines, their propellers and the airframe they are on - in PROPS
// (engine_noise.cpp), so another propeller aircraft is a value of Engine, a
// row there and a type in engines3d::enginesOf(). Its figures are what is
// known of the type, each with where it comes from in its row's comment -
// published [P], measured [M] or inferred [I]: what the 3D view needs to ask
// for its power and its propellers' speed (engines3d::ask()) and where its
// propellers are, and what a synthesiser needs to put its lines where they
// belong. How loud and how rich each line is, is the synthesiser's own to
// make of them, by ear.
//
// A row for a Dash 8 Q400 (DH8D), say: two PW150As turning six-bladed Dowty
// R408s 4.115m across at 1,020rpm for take-off [P: EASA TCDS P.002], 900
// climbing and 850 cruising [P*], 1,020 on the approach or 850 under its
// flight manual's supplement 39 [P: its noise certificate], about 660 in
// beta on the ground [M: aboard, targets/atr72.md 4.1] - npClimb 0.88 and
// npGround 0.65 of a rpm100 of 1,020 - its axes' spacing and height from its
// drawings; its gas generator's whine at 7.44-7.46kHz on the climb [M:
// targets/atr72.md 5.1]. A Saab 340 (SF34): two CT7s turning four-bladed Dowty
// R354s [P], their speed and diameter still to be found, its low wing's
// axes lower. A King Air 350 (B350): two PT6As turning four-bladed
// Hartzells at 1,700rpm, 113Hz [M: a pass fitted on 23 Oct,
// tools-old/kingair.txt]. Each then a value of Engine, appended, a neutral
// row in ENGINES beside the ATR's, its row here and its type in
// engines3d::enginesOf() - each researched in its own turn.
struct PropData {
    Engine engine;     // the row's own
    const char *name;  // "ATR 72-500/-600": the preview's and the checks' lines
    // Its engines, and each one's propeller: its blades, its speed at 100%,
    // rpm - its governor's take-off setting - and its disc's diameter, m; how
    // far each propeller's axis is from the centre line, m - nought for a
    // single - and how high it stands on the aircraft's wheels, m.
    uint8_t engines, blades;
    float rpm100, diameter, halfSpan, axisUp;
    // Its propellers' speeds as its crew set them, as shares of rpm100 -
    // what engines3d::ask() gives as Sound::np: in beta on the ground (ground
    // idle, taxiing, the landing roll out of reverse), where nothing governs
    // them; climbing, cruising and coming down from height; for take-off; on
    // the approach; and in reverse, ungoverned again.
    float npGround, npClimb, npTakeoff, npApproach, npReverse;
    // How fast they get from one to another, shares of rpm100 a second: up,
    // as the levers go forward, and down, a steady glide.
    float npRise, npGlide;
    // Its two propellers held to one speed and phase while governed -
    // synchrophased - or not; and in beta, where nothing holds them, how far
    // apart in speed the two run, as a share.
    bool synchro;
    float splitBeta;
    // Its shaft power as a share of its take-off's, by phase - what
    // engines3d::ask() gives as Sound::power: climbing; cruising; level low
    // down, about an airfield; coming down from height; on the approach; in
    // reverse.
    float climbPower, cruisePower, levelPower, descentPower, approachPower, reversePower;
    // Its gas generator's high-pressure spool at 100%, rpm; and its whine as
    // heard on the approach, Hz at the source (nought: not known). Its
    // reduction gearbox's lines, as orders of the propeller's turning - the
    // one heard outside first (nought: none known).
    float nh100, whineHz, gearA, gearB;
    // At its stand a propeller brake on its right engine instead of an APU:
    // hotel mode, its gas generator running with its propeller held still,
    // which the 3D view asks for as Sound::apu. And ground spoilers rising
    // on its landing roll, or none.
    bool hotel, groundSpoilers;
};

// A propeller installation's row - for any other engine a generic twin
// turboprop's, its blades passing at Sound::bladeHz at 100%.
const PropData &propData(Engine engine);
// Whether an engine is a propeller installation with a row of its own.
bool isProp(Engine engine);

struct Sound {
    Kind kind = Kind::NONE;
    // Which aircraft: another - even of the same kind - faded in afresh, as
    // it is, rather than this one's engines and APU carried over to it. Its
    // own little differences come from it too - its two engines' speeds a
    // shade apart, which of its buzz-saw's lines are strongest, its nose
    // gear's and fuel vents' pitch - the same each time it is heard.
    uint32_t who = 0;
    // The engines, from its type - a turboprop's from its row (PropData).
    uint8_t engines = 2;
    float fanHz = 3000;   // a turbofan's fan blades passing at full power - an Engine::GENERIC one's
    float bladeHz = 120;  // a propeller's blades passing at 100% - an Engine::GENERIC one's; a helicopter's main rotor's
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
    // A turboprop's propellers' speed as its crew set it, a share of their
    // 100% (PropData::rpm100): its governor's setting - 100% for take-off and
    // on the approach, 82% climbing and cruising, an ATR's - or in beta on
    // the ground and in reverse, where nothing governs them, the speed they
    // settle at. Its power is its shaft power as a share of its take-off's:
    // how hard the blades bite, at whatever speed they turn. The 3D view
    // gives both from what the aircraft is doing (engines3d::ask()); less
    // than nought, not said, and the synthesiser makes its own guess from the
    // power. Asked for, not reached: the propellers get there at their own
    // pace (PropData::npRise, npGlide). Nothing else's sound reads it.
    float np = -1;
    // Its reversers out. A fighter's - an EJ200's - has none and doesn't
    // hear it; nor does the 3D view give a Typhoon them, as it gives an
    // airliner on its landing roll above 60 knots, power 0.7: it holds its
    // nose high to 100 knots, its engines idling, then brakes - quieter
    // after touchdown, not louder (engines3d.h). A turboprop's: its
    // propellers' blades turned past flat to reverse pitch.
    bool reverse = false;
    // A fighter's reheat (Kind::LOW_BYPASS): `afterburner` lit or not, as
    // the 3D view has it today - at power 0.95 or more on the ground or under
    // 3,000ft at an airfield, so every take-off it shows is a reheat one -
    // and `reheat` how much: 0 dry to 1 full, as asked for, the synthesiser
    // lighting and cancelling at the engines' own pace. Heard, the larger of
    // the two (afterburner as 1); for a caller that knows only lit or not,
    // as the 3D view does, reheat = afterburner ? 1 : 0. An airliner's sound
    // reads neither.
    bool afterburner = false;
    float reheat = 0;
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
    // engines throttled back, as loud as the engines. A fighter's
    // `spoilers` is its airbrake, the one big panel behind a Typhoon's
    // cockpit, 0 shut to 1 open - which the 3D view gives as open on its
    // landing roll down to taxiing speed, else shut; it has no flaps to
    // speak of (flaperons), so flapsOut is never set for it. A turboprop
    // without ground spoilers - an ATR - has `spoilers` nought.
    float airspeedMs = 0;
    bool gearDown = false, flapsOut = false;
    float spoilers = 0;
    // And how big it is, its length against an A320's.
    float size = 1;
    // Its engines running, or shut down - at its stand - and started again,
    // heard spooling up from nothing; and its APU, an airliner's, whining
    // while it stands or taxies. A turboprop with a propeller brake has no
    // APU (PropData::hotel): for it `apu` is its hotel mode - its right
    // engine's gas generator running, that propeller held still - alone
    // while its engines are shut down, and with `running` the left one's
    // propeller turning, the right one's held until hotel mode ends.
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
// A Boeing 777's engines by its type [b777-sources.md 1]: a 777-300ER's
// (B77W), a -200LR's or a 777F's (B77L) GE90-115Bs - the -110B1 as near as
// heard; a 777-200's or -200ER's (B772) GE90-94Bs 60 times in a hundred by
// `who`, Trent 800s the rest - the PW4000s grounded and retired; a 777-300's
// (B773) Trent 800s. Engine::GENERIC for any other type.
Engine b777Engines(const char *type, uint32_t who);

// The touchdown's squeal and the reversers' coming on heard as the picture
// shows them - a tenth of a second late at most - rather than as late as the
// sound takes to come: from 300m that is nine tenths of a second, and on a
// screen, with nothing to say how far off it is, it reads as late. The rest,
// the engines' and the airframe's sound, is as late as it is, always - its
// pitch and its swell as the aircraft passes depend on it. Off: all as late
// as the sound takes.
extern bool g_transientsOnPicture;
#ifdef ENGINE_NOISE_DEBUG
extern uint64_t g_limited, g_samples;
#endif

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
    static constexpr int SOURCES = 9;  // the broadband's sources and the buzz-saws, each surging its own way

  private:
    void update();
    void placedWay();
    void restart();
    void setSurges(float scale);
    void engines();
    float wantN1(const Sound &s, const Sound &rev) const;
    void slowStep();
    void bandsStep();
    void tonesStep();
    void combsStep();
    void huntStep();
    void tyresStep();
    // The fighter's - Kind::LOW_BYPASS, the Typhoon's: its controls each 2ms
    // step and each 32ms one, after the way and the propagation's figures
    // are worked out; its sources into the step's mix, before the air's
    // muffling and the echo off the ground - from the bank's two noises,
    // `lateA` and `lateB`, and the tones' two, `noiseA` for its unplayed low
    // band, `noiseB` for its whine. See engine_noise.cpp's THE FIGHTER.
    void fighterStep();
    void fighterSlowStep();
    void fighterRestart();
    struct Rng;
    void fighterRender(float *mix, const float *lateA, const float *lateB, const float *noiseA, const float *noiseB,
                       Rng &rng, int n);
    // The turboprop's - Kind::TURBOPROP: its places, as the fighter's are -
    // its controls each 2ms step and each 32ms one; its broadband, its
    // propellers' comb and its wheels' thumps and squeal into the step's mix
    // before the air's muffling and the echo off the ground; and its tones
    // after them, each with the air's loss and the echo worked out for its
    // own pitch, as the airliner's are. See engine_noise.cpp's THE TURBOPROP.
    void propStep();
    void propSlowStep();
    void propDoing();
    void propRestart();
    void propRender(float *mix, const float *lateA, const float *lateB, const float *noiseA, const float *noiseB,
                    Rng &rng, int n);
    void propTones(float *mix, const float *noiseA, const float *noiseB, int n);
    void propToneSums(float *mix, const float *noiseA, const float *noiseB, int n);

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
    // The autothrust's hunting on the approach: N1's wander as a share of
    // it, its process's last two values, and how much it is hunting.
    float hunt_ = 0, huntX1_ = 0, huntX2_ = 0, huntWeight_ = 0;
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
    float surge_[SOURCES] = {}, flickerN_ = 0;  // as they are now: each source's surge as a power, the flicker in its spreads
    float shimmerQuick_[BANDS] = {}, shimmerSlow_[BANDS] = {};
    // How much of each is kept from one slow step to the next, how much
    // new noise comes in, and how far each surges: worked out for each
    // aircraft as it starts - see setSurges().
    float surgeKeep_[SOURCES][5] = {}, surgeNp_[SOURCES] = {}, shimmerKeep_[4] = {};
    // Each band's centre, as octaves from 1kHz.
    float bandLog_[BANDS] = {};
    // And its tangent of pi x its centre over the rate - for the muffling's
    // loss there - and the air's loss there, dB a kilometre.
    float bandTan_[BANDS] = {}, bandAirDbKm_[BANDS] = {};
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
    // the tones have theirs worked out exactly - and the speaker's
    // high-pass, taking out what goes in under 180Hz: its two states and its
    // coefficients.
    Svf air_;
    float hp1_ = 0, hp2_ = 0, hpB0_ = 0, hpA1_ = 0, hpA2_ = 0;
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
    float squealDueS_ = -1, squealSinkMs_ = 1, squealRollingMs_ = 0;  // due in, and as the frame that showed it had it
    float lightPhase_ = 0, thump_ = 0;
    Svf thumpBand_;

    // The fighter's own (Kind::LOW_BYPASS) - what the airliner's sums don't
    // have: see engine_noise.cpp's THE FIGHTER. It shares the airliner's
    // bank (its coefficients, states and gains), its surges' and shimmer's
    // states and its flicker, none of which an airliner's sound runs while
    // a fighter's plays.
    struct Jet {
        // Heard: which way it faces, how fast it is closing - eased in 35ms,
        // not the airliner's 0.2s - and the Doppler and convection from that.
        float aspect = 0.5f, closing = 0, doppler = 1, convection = 1;
        // Its engines: the reheat lit, 0 to 1, and how fast that is moving;
        // the second engine's spool and fire - its start half a minute after
        // the first's; how long the engines have been asked to run.
        float reheat = 0, reheatRate = 0, spool2 = 1, lit2 = 1, runS = 0;
        // Its jet: its speed, m/s, and its eddies' Mach number against the
        // air about it - over 1, crackling.
        float jetMs = 770, mc = 0;
        // The slow processes the roar surges by, in their own spreads: the big
        // eddies' roar's quick and slow parts, and the fine turbulence's.
        float ltsQuick = 0, ltsSlow = 0, fstQuick = 0, fstSlow = 0, ltsSurge = 1;
        // The unplayed low band - the big eddies' own sound, under the
        // speaker's - a band-pass and a resonance on the same poles: its
        // coefficients, its states, its noise's last two; its envelope, that
        // envelope's mean over a second and the step's sum of it; and its
        // last sample, for its compressions' starts.
        float lowB0 = 0, lowA1 = 0, lowA2 = 0, lowHz = 0;
        float lowY1 = 0, lowY2 = 0, lowZ1 = 0, lowZ2 = 0, lowX1 = 0, lowX2 = 0;
        float env = 0, envMean = 0.5f, envSum = 0, lowWas = 0;
        // Its engines' dry level as heard, dB: the spools' sound lagging their
        // speed - first set, not eased, at a restart.
        float dryDb = -999;
        // The shocks: their chance at each compression's start this step,
        // their strength, the self-excitation after each one and its unit,
        // the 50ms burstiness's process; and the ring their shapes are added
        // into ahead of the sample being made.
        float shockP = 0, shockUnit = 0, shockAmp = 0, hawkes = 0, hawkesAdded = 0, burst = 0;
        // The flutter: the whole roar's quick level, its process and its
        // gain at the step's start and end.
        float flutter = 0, flutterWas = 1, flutterTo = 1;
        // Its intake's whine, as an amplitude: worked out each slow step.
        float whine = 0;
        // The roar's level, smoothed, and the gain it is ridden down by at
        // its loudest; the share of its samples lately over the limiter's
        // threshold, and how far that has lowered the ride's ceiling.
        float roarRms = 0, ride = 1, overShare = 0, ceilBy = 1;
        // The light-up: the reheat a step ago, the time since it caught, s;
        // its whoomp's loudness, its resonance's coefficients and states.
        float reheatWas = 0, lightS = 9, whoomp = 0;
        float whoompB0 = 0, whoompA1 = 0, whoompA2 = 0, whoompY1 = 0, whoompY2 = 0, whoompX1 = 0, whoompX2 = 0;
        float ring[64] = {};
        int ringAt = 0;
    };
    Jet jet_;

    // The turboprop's own (Kind::TURBOPROP) - what the airliner's sums don't
    // have: see engine_noise.cpp's THE TURBOPROP. It shares the airliner's
    // bank, its surges', shimmer's and flicker's states, the first of its
    // buzz-saws' combs and its place - both propellers' comb of blade
    // harmonics here - and the tyres' squeal and the nose wheel's thump, none
    // of which an airliner's sound runs while a turboprop's plays.
    struct Prop {
        // Heard: which way it faces, how fast it is closing - eased in 35ms,
        // the fighter's, not the airliner's quarter second - and the Doppler
        // from that.
        float aspect = 0.5f, closing = 0, doppler = 1;
        // Its two propellers, the left - the master - and the right: each
        // one's speed as a share of its 100% (PropData::rpm100) and how fast
        // that is changing; how far its blades have come out of feather, 0
        // feathered to 1 biting; and, ungoverned in beta, how far its speed
        // wanders - a slow process each - and the right one's off the left's.
        float np[2] = {0.71f, 0.71f}, npRate[2] = {}, pitch[2] = {1, 1}, wander[2] = {}, split = 0;
        // Governed, 1, or in beta on the ground, 0, eased; the static share,
        // 1 standing to 0 by 40m/s, and its reverse pitch, 0 to 1; its shaft
        // power as a share of its take-off's, lagged; the governor's own
        // hunting, both propellers together; and on the ground, 1, or in the
        // air, 0.
        float governed = 1, stat = 0, reverse = 0, torque = 0, hunt = 0, ground = 0;
        // Its two gas generators, the left and the right: each one's spool,
        // 0 stopped to 1 idling or above, its fire, its speed (NH) as a share
        // of its 100%, and its whine's own offset and wander.
        float spool[2] = {1, 1}, lit[2] = {1, 1}, nh[2] = {0.66f, 0.66f}, whineBy[2] = {}, whineX[2] = {};
        // The synchrophaser's phase between the two propellers, as a share of
        // a blade's passing - the airframe's own; and the harmonics' shared
        // surge, its quick and slow parts.
        float psi = 0, surgeQ = 0, surgeS = 0;
        // The low band under the bank's: its two stages' last two outputs,
        // its loudness now and its change each sample.
        float lowY1 = 0, lowY2 = 0, lowZ1 = 0, lowZ2 = 0, lowGain = 0, lowBy = 0;
        // Its tones' low-pass far off (propTones()): its coefficient, nought
        // nearer than 3km where it is not run, and its two stages' outputs.
        float cutA = 0, cutY1 = 0, cutY2 = 0;
    };
    Prop prop_;
    // Its tones after the air's muffling and the echo: its blade harmonics
    // k = 1-6 - the right propeller's apart only in beta, where the two run at
    // their own speeds, and only where the speaker plays them - the
    // fundamental's steady line while governed, the gearbox's line, each gas
    // generator's whine and the haystack about them - see engine_noise.cpp's
    // PropSlot.
    static constexpr int PROP_TONES = 16;
    Tone propTones_[PROP_TONES];
};

}  // namespace engine_noise
