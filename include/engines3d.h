#pragma once

#include <math.h>
#include <string.h>

#include <algorithm>

#include "engine_noise.h"

// What the 3D view asks of the one followed's engines - see engine_noise.h
// for what is made of it. Which engines, from its type; how hard they are
// working, from what it is doing, which ADS-B doesn't say but its speed, its
// climb and the runway simulation's take-off or landing give away: idling as
// it taxies, take-off power on its roll and in its climb, back on the
// approach, reverse thrust just down - an airliner's. A fighter's - the
// Typhoon's - where it is flown otherwise [eufi-sources.md 2.1, 4.2, 7, 11]:
// full dry power on its brakes lined up, its reheat lit as it rolls; no
// reversers, its engines idling all down its landing roll; and level, as
// hard as its speed through the air needs, not an airliner's cruise power at
// any speed.
//
// Header only, for the view's engineSound() and a PC check's alike
// (tools/engine_preview/engines3d_check.cpp).
namespace engines3d {

// An aircraft's engines, by its type: the A320 family's and the Eurofighter
// Typhoon's - each kind of aircraft made and heard in its turn - and any
// other silent. ADS-B doesn't say which of two engines an A320 has, so they
// are picked by the airframe, `who`, in the fleets' shares - a ceo's CFM56s
// or V2500s, a neo's LEAPs or geared fans - and with them whether an older
// one's fuel vents whistle on its approach: the same airframe always the
// same.
inline engine_noise::Sound enginesOf(const char *type, uint32_t who) {
    engine_noise::Sound s;
    for (const char *t : {"A318", "A319", "A320", "A321", "A19N", "A20N", "A21N"}) {
        if (strcmp(type, t) == 0) {
            s.kind = engine_noise::Kind::TURBOFAN;
            s.engine = engine_noise::a320Engines(t, who);
            s.fopp = engine_noise::a320Fopp(s.engine, who);
        }
    }
    if (strcmp(type, "EUFI") == 0) {
        s.kind = engine_noise::Kind::LOW_BYPASS;
        s.engine = engine_noise::Engine::EJ200;
    }
    return s;
}

// What it is doing, as the 3D view has it.
struct Doing {
    bool ground = false;
    // The runway simulation's take-off - lined up counts - or landing, and
    // whether that is simulated, not the feed's reports as they come.
    bool takingOff = false, landing = false, simulating = false;
    int kt = 0;     // its speed as shown
    // Its speed as the runway simulation or the feed has it: nought while
    // it waits lined up, though the view may be taxiing it there still.
    int rollKt = 0;
    int vsFpm = 0;  // climbing, feet a minute
    int altFt = 0;  // above the sea - for how thin the air is
    bool low = false;      // under 3,000ft over the airfield the view is at
    bool fighter = false;  // drawn as a fighter: not a trainer, an AWACS or a U-2
};

// How hard a fighter's engines work level, by its speed through the air. An
// airliner's 0.65 at any speed - its cruise's, some 85% of its fan's speed -
// left a Typhoon's low pass at 420 knots crackle-free and 16LU quieter than
// at 0.85 [M: eufi-fit's NOTES]. Its drag, and so the thrust it needs, goes
// as the square of its speed through the air - equivalent airspeed: its
// speed, the ground's near enough the air's, by the root of the air's
// density there against the sea's, as the standard atmosphere has it. So
// 0.65 to 340 knots - a display's 300, its cruise at M0.85 and FL250, which
// is 342 - then 0.85 by 420, a low pass at 250ft [I: eufi-sources 2.2, 11.6:
// 0.75 to 0.9], and full dry power, 0.88, from 480, as level passes at 350
// to 500 knots nearly are [I: eufi-methods 9.1]. Never reheat: the view
// gives that to a climb low down alone.
inline float fighterLevelPower(int kt, int altFt) {
    float h = (float)std::max(0, altFt);
    float density = h < 36089 ? powf(1 - 6.8756e-6f * h, 4.2559f) : 0.2971f * expf((36089 - h) / 20806.0f);
    float eas = kt * sqrtf(density);
    if (eas <= 340) {
        return 0.65f;
    }
    if (eas <= 420) {
        return 0.65f + 0.2f * (eas - 340) / 80;
    }
    return std::min(0.88f, 0.85f + 0.03f * (eas - 420) / 60);
}

// Its engines' power, its reversers and reheat - and a helicopter's blade
// slap - into `s`, its kind already set from its type.
inline void ask(engine_noise::Sound &s, const Doing &d) {
    using engine_noise::Kind;
    bool jet = s.kind == Kind::LOW_BYPASS;
    if (s.kind == Kind::ROTOR) {
        // Set down its rotor turning, idling; hovering the hardest it
        // works; coming down at 30 to 90 knots - an approach - its blades
        // slapping.
        s.power = d.ground ? 0.25f : d.kt < 25 ? 0.95f : d.vsFpm > 300 ? 0.85f : d.vsFpm < -300 ? 0.45f : 0.65f;
        if (!d.ground && d.vsFpm < -250) {
            float speed = std::min(1.0f, std::max(0.0f, (d.kt - 30) / 20.0f)) *
                          std::min(1.0f, std::max(0.0f, (90 - d.kt) / 20.0f));
            s.slap = std::min(1.0f, (-d.vsFpm - 250) / 600.0f) * speed;
        }
    } else if (d.ground) {
        if (d.takingOff && (d.simulating || d.kt > 30)) {
            // Lined up, its thrust coming up as it is cleared: a fighter's
            // to full dry power on its brakes, its reheat lit as they come
            // off and it rolls [I: eufi-sources 11.3] - not standing on the
            // runway for as long as the view holds it there.
            s.power = jet && d.simulating && d.rollKt == 0 ? 0.88f : 1;
        } else if (jet && d.landing) {
            // No reversers: its nose held high to 100 knots, then its wheel
            // brakes, its engines idling all the way - quieter after its
            // touchdown, not louder [P, I: eufi-sources 4.2].
            s.power = 0.05f;
        } else if (d.landing && d.kt > 60) {
            s.power = 0.7f;
            s.reverse = true;
        } else {
            s.power = 0.05f + std::min(0.12f, d.kt / 150.0f);
        }
    } else {
        s.power = d.vsFpm > 1200 ? 1.0f : d.vsFpm > 300 ? 0.88f : d.vsFpm < -400 ? (d.low ? 0.45f : 0.05f) : 0.65f;
        if (jet && d.vsFpm <= 300 && d.vsFpm >= -400) {
            s.power = fighterLevelPower(d.kt, d.altFt);
        }
    }
    // Reheat only a fighter's: at take-off power on the ground or under
    // 3,000ft at an airfield - its take-off roll and its climb - and only
    // lit or not, as the view knows it.
    s.afterburner = jet && d.fighter && s.power >= 0.95f && (d.ground || d.low);
    s.reheat = s.afterburner ? 1.0f : 0.0f;
}

// Its airframe as the view has it set up: its gear down, its flaps out, its
// spoilers up, 0 to 1 - a Typhoon's spoilers its airbrake, up on its landing
// roll as an airliner's are, and no flaps for it, only flaperons.
inline void airframe(engine_noise::Sound &s, bool gear, bool flaps, float spoilers) {
    s.gearDown = gear;
    s.flapsOut = flaps && s.kind != engine_noise::Kind::LOW_BYPASS;
    s.spoilers = spoilers;
}

// Its engines' height over where it is placed - its main wheels - for the
// sound off the ground, in metres for each of its look's scale, which the
// view multiplies by as it always has, so an airliner's comes out exactly as
// it did: 2.5, about its middle, a little up. A Typhoon's jets' nozzles are
// low in its tail, 1.5m up on its gear whatever its scale, where its sound's
// scenes have them (tools/engine_preview/NOTES.md).
inline float enginesUpPerScale(const engine_noise::Sound &s, float scale) {
    return s.kind == engine_noise::Kind::LOW_BYPASS ? 1.5f / scale : 2.5f;
}

}  // namespace engines3d
