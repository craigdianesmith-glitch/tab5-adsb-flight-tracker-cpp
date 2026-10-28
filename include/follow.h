// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's take-off, landing and taxi simulation: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "airports.h"
#include "telemetry.h"

// A run of reports: on the Tab5 in its PSRAM - there is one for every
// contact tracked, a hundred or more, and the chip's own memory is for the
// WiFi, TLS and the 3D view's drawing.
#ifdef ESP_PLATFORM
#include "psram_alloc.h"
using SampleList = std::vector<FollowSample, PsramAllocator<FollowSample>>;
#else
using SampleList = std::vector<FollowSample>;
#endif

// What following makes of a followed aircraft's report - kept apart from the
// live screen so a replay of one works it out the same way.

// The airport it looks to be coming down to, if any: descending, or low and
// not climbing, below FOLLOW_APPROACH_MAX_FT; and of those within FOLLOW_RANGE_NM the
// nearest it is pointing towards - or, very close or on the ground, the
// nearest at all. `distNm` is how far off it is.
bool followDestination(const Aircraft &ac, Airport &out, float &distNm);

// The range to show it at: FOLLOW_RANGE_NM in the cruise, closing in on
// the way down - in steps, so the scale only changes now and then - to
// keep its destination on the plot without the miles beyond it. `fine` for
// auto zoom's finer steps, FOLLOW_FINE_STEPS_NM.
int followRangeFor(const Aircraft &ac, bool fine = false);

// Where it probably is `ageS` seconds after `last`, its last report with a
// current position, while the feed has lost it - as it often does at a
// touchdown, the aircraft dropping below the receivers' horizon until it is
// taxiing near one, or on its take-off roll before it climbs into their
// view. Arriving and lined up on a runway, it flies on to the touchdown point
// and rolls out along the centreline, braking to a stop no further than the
// far end, its height brought down from its last to the field's by the
// touchdown. Departing and rolling down one, it accelerates along the
// centreline, lifts off at rotation speed - by the far end at the latest -
// and climbs out on the runway's heading. Otherwise - or climbing, lifted
// off or going around, however it is lined up - in the air it carries
// straight on; slow on the ground, it stays put. Distance along the runway
// rather than height decides the touchdown: the feed's barometric altitude
// reads a few hundred feet out on a day of high or low pressure.
//
// `history` is its reports before `last`, a few minutes of them, on the same
// clock: they say whether a roll down a runway is a take-off or a landing -
// see followDeparting() - and which way it was going where `last` doesn't,
// as a report on the ground may not.
struct FollowEstimate {
    float lat, lon;
    float track;
    // Above the sea, where the feed's pressure altitude can be put right -
    // see FollowSample::pressureFt - and otherwise as the feed gives it.
    int32_t altFt;
    bool ground;
    int gsKt;
    int vsFpm;
    const char *how;  // which of the above it was taken to be doing, for the log
    int16_t pressureFt = 0;  // what putting it right added to altFt: see feedAltFt()
    // Taking off, the seconds till it leaves the runway - nought or less
    // once it has, its pilot rotating it over the last of them (see
    // Pilot::fly()); NAN, not taking off.
    float liftoffInS = NAN;
};
// An estimate's height as the feed would have given it - its pressure
// altitude, as every label shows a height.
inline int32_t feedAltFt(const FollowEstimate &e) { return e.altFt - e.pressureFt; }
FollowEstimate estimateFollowed(const FollowSample &last, float ageS, const SampleList &history);

// Whether `last`, a report on the ground, is of an aircraft departing rather
// than one that has just landed: none of the reports in `history` in the
// few minutes before it has it in the air.
bool followDeparting(const SampleList &history, const FollowSample &last);

// A runway as a line: one end, its threshold, and the way down it from there.
struct RunwayLine {
    float thrLat, thrLon;
    float ux, uy;  // a nautical mile down it, east and north, in nm
    float len;     // nm
    float halfWidthNm;
    int32_t fieldFt;
    // Which runway end it is, as painted on it, and the airport's IATA code:
    // pointers into the runway table, so they last.
    const char *ident = nullptr;
    const char *airport = nullptr;
};

// A take-off or a landing, simulated from when it begins to when the feed
// can be trusted with it again - rather than the feed's reports, which come
// and go near the ground, with the gaps between them filled in. Each report
// while it is under way corrects the simulation rather than replacing it:
// put on the runway's centreline, or its extension, and eased into over a
// few seconds, so the plot never jumps. Every low contact has one, live and
// in a replay alike, fed the same reports, so both show the same thing.
//
// A take-off begins on a runway, pointing down it, departing - see
// followDeparting() - with its thrust picking up: rolling faster than a taxi,
// or faster than at its report before; or stopped there, lined up, and lost
// by the feed at the next poll, as one is as it goes. It ends with the feed
// in charge again once it is well up, or turning off the runway's line.
//
// A landing begins lined up on final, low, not climbing and no faster than a
// landing is flown, and runs on down the runway to a stop. It ends at the
// first report off the runway: it is shown turning off it and taxiing to where
// that report puts it, since without the taxiways it can't be said where it
// would turn off before then. Or with a go-around, the feed in charge again.
//
// Otherwise - and for anything higher, and a rotorcraft always - it is
// estimateFollowed() from the last report, as before.
class RunwaySim {
public:
    // Its next report, in time order: stale ones only say what it was doing.
    void report(const FollowSample &s);
    // Where it is shown at `ms`, no earlier than its last report - on the
    // same clock - taken no more than `maxAgeS` on from that report.
    FollowEstimate at(uint32_t ms, float maxAgeS = 1e9f) const;
    // Where the radar, the zoom and their replays put it: at()'s, but a
    // report behind where it was shown taken at once rather than eased -
    // see plotAt() in follow.cpp.
    FollowEstimate plotAt(uint32_t ms, float maxAgeS = 1e9f) const;
    // Whether a take-off or landing, or the ease out of one, is what at()
    // shows at `ms`, rather than estimateFollowed() from its last report.
    bool simulating(uint32_t ms) const;
    // How long since its last current position it is kept for, lost.
    uint32_t keepMs() const;
    bool haveFresh() const { return haveFresh_; }
    // Whether it was reported in the air in the `withinMs` before `ms`.
    bool airborneWithin(uint32_t ms, uint32_t withinMs) const {
        return seenAir_ && (int32_t)(ms - airMs_) <= (int32_t)withinMs;
    }
    const FollowSample &fresh() const { return fresh_; }  // its last report with a current position
    const char *phase() const;                              // for the log
    size_t historySize() const { return history_.size(); }  // its low reports kept, for the bench
    // The take-off or landing under way, for the zoom to announce: lined up
    // counts as a take-off, cleared for it. `runway()` is which, while one is.
    enum class Movement : uint8_t { NONE, TAKEOFF, LANDING };
    Movement movement() const;
    const RunwayLine &runway() const { return rw_; }

    // A take-off's roll down the runway, its rotation and lift-off, from a
    // report rolling at `kt` from `along` at `ms` - or with `stand` from a
    // standstill there, its thrust coming up first: see takeoff.h, and
    // report() for how one is made to fit the reports.
    struct Rolled {
        uint32_t ms;
        float along, kt;  // nm from the threshold, knots
        float k;          // its acceleration as a share of its kind's
        uint32_t rotMs;   // its nose coming up from here
        float rotAlong, rotKt, rotAccel;
        uint32_t offMs;  // and off the runway here
        float offAlong, offKt;
        bool air;    // off already by its report: rolled to offMs, not from ms
        bool stand;  // from a standstill at ms
    };
    // A take-off at a moment: on the runway, rolling or rotating, or
    // climbing away - its height over the field, its climb's seconds since
    // its lift-off and how fast they are being played (see report()).
    struct TakeoffAt {
        bool ground, rotating;
        float along, kt;     // nm from the threshold, knots
        float ft, fpm;       // over the field
        float climbS, rate;  // its climb's clock and its pace: 1 as it is flown
        float offInS;        // seconds till it leaves the runway: nought or less, gone
    };

private:
    enum Phase : uint8_t { NONE, LINED_UP, TAKEOFF, LANDING };
    FollowEstimate model(uint32_t ms, float maxAgeS) const;
    bool advance(const FollowSample &prev, bool hadPrev);
    void start(const FollowSample &prev, bool hadPrev);
    uint32_t waitFor(const FollowSample &prev, bool hadPrev) const;
    // A take-off under way and report `s`: see advance().
    bool takeoffFt(const FollowSample &s, float &ft) const;
    bool rollingStill(const FollowSample &s) const;
    bool taxiingOn(const FollowSample &s, float along) const;
    bool levelledOff(const FollowSample &s) const;
    bool turnedDown(const FollowSample &s, float ft) const;
    const TakeoffProfile &kindProfile() const;
    // A take-off: see report().
    void takeoffReport(uint32_t heardMs, Phase before, const TakeoffAt *was, const FollowEstimate *shown);
    void takeoffEstimate(float along);
    void takeoffPlan(uint32_t t0, const TakeoffAt *from, bool replanning);
    void takeoffClimb(uint32_t ta, float tau0, float rate0, float fpm, float along, float kt, uint32_t tR);
    TakeoffAt realAt(uint32_t ms) const;    // as the reports have it
    void flightAt(uint32_t ms, float &along, float &kt) const;
    TakeoffAt shownAt(uint32_t ms) const;   // as it is shown
    TakeoffAt linedUpAt(uint32_t ms) const;  // LINED_UP, its roll begun from a standstill unseen
    FollowEstimate takeoffEstimateOf(const TakeoffAt &a) const;
    void climbOutOf(uint32_t ms, float maxAgeS, FollowEstimate &e) const;
    FollowEstimate brakingAt(uint32_t ms, float maxAgeS) const;
    float handbackUp() const;
    bool brakeOn(uint32_t heardMs, const FollowEstimate &shown, const FollowEstimate &now, bool anyway = false);

    SampleList history_;  // its low reports of the last few minutes, oldest first
    uint32_t airMs_ = 0;                 // its last report in the air, if seenAir_
    bool seenAir_ = false;
    // Its heights' steady run: how long, in tenths of a second, its reports
    // have been level by their rates and their heights within HEIGHT_STEP_FT
    // of steadyFt_, the feed's height as the run began - levelled off, or
    // not: see report(). INT16_MIN with no run begun; held to 32,767ft, never
    // steady above it, where nothing is handed back. Each in a gap the
    // members' alignment leaves, so they take no room.
    uint16_t steadyDs_ = 0;
    FollowSample fresh_ = {};
    bool haveFresh_ = false;
    Phase phase_ = NONE;
    int16_t steadyFt_ = INT16_MIN;
    RunwayLine rw_ = {};
    uint32_t rollMs_ = 0;  // LINED_UP: when it is taken to have begun its roll, not seen since
    // The ease from where it was shown as fresh_ came in to where fresh_ puts
    // it: an offset that shrinks to nothing, or a turn off the runway.
    uint32_t easeStartMs_ = 0, easeEndMs_ = 0;
    uint32_t altEndMs_ = 0;  // the offset's height, over its own span
    // A take-off's offset, taken in over climbLenS_ seconds and then
    // settling, its place and height together - see report().
    bool climbEase_ = false;
    bool turning_ = false;
    bool heldDown_ = false;  // LANDING: fresh_ in the air, but taken as down - see report()
    // The ease a report began puts it back along its way by the clock, which
    // the radar takes at once - see plotAt().
    bool plotCut_ = false;
    float climbLenS_ = 0;
    float dLat_ = 0, dLon_ = 0, dAlt_ = 0;     // the offset
    // Landing, put back down the runway: the offset taken in over the
    // rollOverNm_ it rolls on from rollFromNm_ along it, rather than by time.
    float rollFromNm_ = 0, rollOverNm_ = 0;
    float fromLat_ = 0, fromLon_ = 0, fromTrack_ = 0;  // the turn's start
    // A take-off's height, handed back to the feed: an offset eased out
    // with its rate kept, its climb never turned into a descent - see
    // report() - from dAlt_ at altRate_ feet a second, over altLenS_.
    bool climbSafe_ = false;
    bool climbingOut_ = false;  // and climbing on out after it, still low: see report()
    // A take-off ended on the runway - rejected, or a taxi along it taken for
    // one - shown slowing from where it was shown, rejAlong_ along it at
    // rejKt_, brought back down to it first should it have been shown off
    // it (dAlt_ over the field, climbing at altRate_ feet a second): see
    // brakingAt(). brakingNow_ for the report that ends one so.
    bool braking_ = false, brakingNow_ = false;
    float rejAlong_ = 0, rejKt_ = 0;
    float altRate_ = 0, altLenS_ = 0;
    float dKt_ = 0;  // a take-off ended on the runway: its speed's offset, eased out with its place
    // A take-off ended in the air - handed back to the feed - its place eased
    // out along the feed's track as it was then (fromTrack_) and across it,
    // from as fast as it was shown going: the offset's rates placeRate_ and
    // placeAcross_ nm a second to begin with, no faster than placeMost_; and
    // its height's taken down no faster than altDown_ feet a second - its
    // rate changed by no more than altAccel_ feet a second each second: see
    // easeOut().
    float placeRate_ = 0, placeAcross_ = 0, placeMost_ = 0, altDown_ = 0, altAccel_ = 0;
    // A take-off under way: what its reports have said of it - the speed
    // it lifts off at and its climb, as its kind's to begin with; its height
    // read rolling, as an average of so many; and its last report rolling,
    // where it was and how fast - its own speed, and the feed's - and the
    // fastest the feed had it rolling. Its kind, as it began: a type the
    // main list doesn't give and the zoom's does flipped it with each list,
    // its climb jumping 59ft up and down.
    float tkLiftoffKt_ = 0, tkClimbFpm_ = 0, tkLevelFt_ = 0;
    uint8_t tkLevels_ = 0;
    bool tkRolled_ = false;
    TakeoffKind tkKind_ = TakeoffKind::NARROW;
    // Begun from a report "airborne" at speed down the runway, nothing heard
    // of it before: as likely a landing's rollout, so shown rolling on at its
    // speed until the next report says which - see takeoffPlan().
    bool tkTentative_ = false;
    int16_t tkRollGs_ = -1, tkTopGs_ = -1;
    uint32_t tkRollMs_ = 0;
    float tkRollAlong_ = 0, tkRollKt_ = 0;
    // The real one as its reports have it: rebuilt at each report.
    Rolled est_ = {};
    // The one shown: planned at each report from where it was shown as that
    // report came in - its own roll, sped up or slowed to leave the runway
    // as the real one is reckoned to, then its climb's clock played a
    // little faster or slower to meet the real one's, from climbMs_ on;
    // and its place, the real one's with a residual eased out from restMs_.
    Rolled plan_ = {};
    bool planAir_ = false;  // planned in the air: off the runway for good
    uint32_t climbMs_ = 0, restMs_ = 0;
    float climbS0_ = 0, rate0_ = 1, climbGapS_ = 0, holdS_ = 0, warpS_ = 0, planFpm_ = 0;
    float rest0Nm_ = 0, rest0NmS_ = 0, restS_ = 0;
};
// One for each contact tracked - a hundred and more, in PSRAM - and the one
// followed's in the chip's own memory: 168 bytes before take-offs were
// simulated as above, 356 since, 380 with a rejected take-off's braking and
// the hand-back's eases - and its heights' steady run - on the Tab5's 32
// bits. Kept from growing unseen.
static_assert(sizeof(void *) != 4 || sizeof(RunwaySim) <= 384, "RunwaySim kept small");
