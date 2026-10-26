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

private:
    enum Phase : uint8_t { NONE, LINED_UP, TAKEOFF, LANDING };
    FollowEstimate model(uint32_t ms, float maxAgeS) const;
    bool advance(const FollowSample &prev, bool hadPrev);
    void start(const FollowSample &prev, bool hadPrev);
    uint32_t waitFor(const FollowSample &prev, bool hadPrev) const;

    SampleList history_;  // its low reports of the last few minutes, oldest first
    uint32_t airMs_ = 0;                 // its last report in the air, if seenAir_
    bool seenAir_ = false;
    FollowSample fresh_ = {};
    bool haveFresh_ = false;
    Phase phase_ = NONE;
    RunwayLine rw_ = {};
    uint32_t rollMs_ = 0;  // LINED_UP: when it is taken to have begun its roll, not seen since
    // The ease from where it was shown as fresh_ came in to where fresh_ puts
    // it: an offset that shrinks to nothing, or a turn off the runway.
    uint32_t easeStartMs_ = 0, easeEndMs_ = 0;
    uint32_t altEndMs_ = 0;  // the offset's height, over its own span
    bool turning_ = false;
    bool heldDown_ = false;  // LANDING: fresh_ in the air, but taken as down - see report()
    float dLat_ = 0, dLon_ = 0, dAlt_ = 0;     // the offset
    // Landing, put back down the runway: the offset taken in over the
    // rollOverNm_ it rolls on from rollFromNm_ along it, rather than by time.
    float rollFromNm_ = 0, rollOverNm_ = 0;
    // The ease a report began puts it back along its way by the clock, which
    // the radar takes at once - see plotAt().
    bool plotCut_ = false;
    float fromLat_ = 0, fromLon_ = 0, fromTrack_ = 0;  // the turn's start
};
