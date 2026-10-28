// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's take-off, landing and taxi simulation: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

#pragma once

#include <stdint.h>

// How each kind of aircraft takes off - one table for the runway
// simulation's roll and climb (src/follow.cpp), the reports' kind
// (followSampleOf()) and the 3D view's pilot rotating it (scene3d's
// Pilot::fly()), so the nose comes up as the simulation has it about to
// leave the runway.
//
// Fitted to 43 departures off Heathrow's 27R recorded from adsb.fi every
// 5 s on 27 Oct 2026 (10:02-12:05), and checked against 29 more recorded
// after them (12:06-12:46) that the fit never saw. Speeds are over the
// ground, as the feed gives them - a little under the airspeed, into that
// day's 10-16 kt of headwind.
//
// The roll: an acceleration of a(v) = A - B v knots a second, its thrust
// against the drag and the wheels' friction growing with speed - so its
// speed v(t) = A/B - (A/B - v0) e^(-Bt), all of it in closed form. From a
// standstill the thrust comes up first: the acceleration rising from
// nought to A over 2 standS seconds (30 kt after 8.7 s and 55 m for an
// A320, as they were seen to). Its nose comes up over rotateS seconds
// before it leaves, gathering speed at 0.65 of the roll's rate as the wing
// takes the weight - the plain roll run on to the lift-off overshot the
// speeds seen by 4.6 kt - to pitchDeg; so it rotates at about 8 kt below
// the speed it lifts off at.
//
// The climb: built up linearly over buildS seconds from nought to
// climbFpm - its height V t^2 / (2 T_b) until then, and V (t - T_b/2) after
// - which the recorded heights follow to about 15 ft from 10 s on; and from
// easeFt over the field, where the thrust is cut back to accelerate, the
// rate eased to under half of it over 6 s. Its speed gathering gainKt over
// the first few seconds - v_lo + gainKt (1 - e^(-t/3)) - as its nose comes
// up and the wing cleans up.
//
// Measured: lift-off speeds narrowbodies 132/138/150 kt (tenth, middle,
// ninetieth percentile), A321neos 146-153, widebodies 154/157/173 - 777s
// 168 (165 in the later recording), 787s 164 (168), A330s 155, A350s 157 -
// A380s 145-149, an ATR 72 114. Climbs 2741 ft a minute for the
// narrowbodies (2677), 2612 for the widebodies (2641), 1596 for the A380s
// (1498); the build 6.5 s and 10.7 s. The alt_baro dip as the nose comes
// up began some 3.5 s before the wheels left [M]; Airbus rotates at 3
// degrees a second, the 777 at 2-2.5 [manuals]; a 747 leaves at 10 degrees.
//
// The kinds no departure was recorded for keep the old table's figures -
// the speed they lifted off at, their acceleration as the roll's middle
// value (A = the old rate + 0.6 B v_lo) and their climb - with a build,
// rotation and gain in proportion: light aircraft from the POH, a fighter
// with its reheat lit gone in a few seconds.
enum class TakeoffKind : uint8_t {
    NARROW,           // A319, A320, A20N, the 737s and 757-200s, the E-Jets - and any not known
    NARROW_LONG,      // A321, A21N, B739, B39M, B753
    WIDE,             // the A330s, A350s and 767s
    WIDE_LONG,        // the 777s and 787s
    SUPERJUMBO,       // the A380
    TURBOPROP,        // ATR 72s and Dash 8s
    LIGHT,            // Cessnas, Pipers, Cirruses: no recorded departures for the rest
    TURBOPROP_LOW,    // Saab 340s, Jetstreams, King Airs
    TURBOPROP_HEAVY,  // C295s, Hercules, A400Ms
    BUSINESS,         // Gulfstreams, Globals, Citations
    TRAINER,          // Hawks
    QUAD_HIGH,        // BAe 146s and Avro RJs
    REGIONAL,         // CRJs, ERJs, Fokkers
    AIRLIFTER,        // C-17s
    FIGHTER,          // Typhoons, F-35s, F-16s
    WIDE_4,           // A340s
    JUMBO,            // 747s
    COUNT
};

struct TakeoffProfile {
    float liftoffKt;  // v_lo: over the ground, as the feed has it
    float accelKtS;   // A: knots a second at a standstill
    float dragPerS;   // B: a(v) = A - B v; nought, a light aircraft's steady rate
    float standS;     // a standing start's thrust coming up over twice this
    float rotateS;    // the rotation's seconds, its nose coming up before it leaves
    float pitchDeg;   // the nose as it leaves, degrees
    float climbFpm;   // V: the climb once it has built up
    float buildS;     // T_b: the climb built up over this, from nought
    float gainKt;     // its speed gathered in the climb's first seconds
    float easeFt;     // over the field, where the climb eases to accelerate
};

inline const TakeoffProfile &takeoffProfile(TakeoffKind kind) {
    static constexpr TakeoffProfile TABLE[] = {
        {138, 5.30f, 0.012f, 3, 3.2f, 8, 2750, 7.0f, 6, 1300},    // NARROW
        {148, 5.30f, 0.012f, 3, 3.2f, 8, 2750, 7.0f, 6, 1300},    // NARROW_LONG
        {157, 5.10f, 0.011f, 5, 4.0f, 9, 2600, 9.5f, 9, 1450},    // WIDE
        {165, 5.10f, 0.011f, 5, 4.0f, 9, 2600, 9.5f, 9, 1450},    // WIDE_LONG
        {147, 3.70f, 0.0066f, 2, 5.0f, 10, 1600, 11.0f, 10, 1450},  // SUPERJUMBO
        {114, 5.00f, 0.012f, 1, 2.7f, 8, 2000, 9.0f, 3, 1000},    // TURBOPROP
        {62, 2.50f, 0.0f, 1, 2.7f, 8, 700, 4.0f, 3, 5000},        // LIGHT
        {104, 4.05f, 0.012f, 1, 2.7f, 8, 1800, 7.0f, 4, 1000},    // TURBOPROP_LOW
        {114, 4.12f, 0.012f, 2, 2.7f, 8, 1700, 8.0f, 4, 1000},    // TURBOPROP_HEAVY
        {127, 5.41f, 0.012f, 2, 3.0f, 8, 3500, 6.0f, 6, 1500},    // BUSINESS
        {128, 5.92f, 0.012f, 1, 2.7f, 8, 4000, 5.0f, 8, 5000},    // TRAINER
        {124, 4.19f, 0.012f, 3, 3.2f, 8, 1800, 8.0f, 5, 1300},    // QUAD_HIGH
        {136, 4.98f, 0.012f, 3, 3.2f, 8, 2800, 7.0f, 6, 1300},    // REGIONAL
        {134, 4.26f, 0.012f, 3, 4.0f, 8, 2000, 9.0f, 8, 1450},    // AIRLIFTER
        {160, 13.15f, 0.012f, 1, 2.5f, 11, 6000, 5.0f, 15, 5000},  // FIGHTER
        {158, 4.14f, 0.012f, 4, 4.0f, 8, 1800, 10.0f, 9, 1450},   // WIDE_4
        {169, 4.52f, 0.012f, 4, 4.0f, 8, 2000, 10.0f, 9, 1450},   // JUMBO
    };
    static_assert(sizeof TABLE / sizeof TABLE[0] == (unsigned)TakeoffKind::COUNT, "a profile for each kind");
    return TABLE[(unsigned)kind < (unsigned)TakeoffKind::COUNT ? (unsigned)kind : 0];
}
