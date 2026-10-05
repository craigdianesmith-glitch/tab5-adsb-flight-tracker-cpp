#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "airports.h"
#include "telemetry.h"

// What following makes of a followed aircraft's report - kept apart from the
// live screen so a replay of one works it out the same way.

// The airport it looks to be coming down to, if any: descending, or low and
// not climbing, below FOLLOW_APPROACH_MAX_FT; and of those within FOLLOW_RANGE_NM the
// nearest it is pointing towards - or, very close or on the ground, the
// nearest at all. `distNm` is how far off it is.
bool followDestination(const Aircraft &ac, Airport &out, float &distNm);

// The range to show it at: FOLLOW_RANGE_NM in the cruise, closing in on
// the way down - in steps, so the scale only changes now and then - to
// keep its destination on the plot without the miles beyond it.
int followRangeFor(const Aircraft &ac);

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
    int32_t altFt;
    bool ground;
    int gsKt;
    int vsFpm;
    const char *how;  // which of the above it was taken to be doing, for the log
};
FollowEstimate estimateFollowed(const FollowSample &last, float ageS, const std::vector<FollowSample> &history);

// Whether `last`, a report on the ground, is of an aircraft departing rather
// than one that has just landed: none of the reports in `history` in the
// few minutes before it has it in the air.
bool followDeparting(const std::vector<FollowSample> &history, const FollowSample &last);

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
// A landing begins lined up on final, low and not climbing, and runs on down
// the runway to a stop. It ends at the first report off the runway: it is
// shown turning off it and taxiing to where that report puts it, since
// without the taxiways it can't be said where it would turn off before then.
// Or with a go-around, the feed in charge again.
//
// Otherwise - and for anything higher - it is estimateFollowed() from the
// last report, as before.
class RunwaySim {
public:
    // Its next report, in time order: stale ones only say what it was doing.
    void report(const FollowSample &s);
    // Where it is shown at `ms`, no earlier than its last report - on the
    // same clock - taken no more than `maxAgeS` on from that report.
    FollowEstimate at(uint32_t ms, float maxAgeS = 1e9f) const;
    // Whether a take-off or landing, or the ease out of one, is what at()
    // shows at `ms`, rather than estimateFollowed() from its last report.
    bool simulating(uint32_t ms) const;
    // How long since its last current position it is kept for, lost.
    uint32_t keepMs() const;
    bool haveFresh() const { return haveFresh_; }
    const FollowSample &fresh() const { return fresh_; }  // its last report with a current position
    const char *phase() const;                              // for the log
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

    std::vector<FollowSample> history_;  // its low reports of the last few minutes, oldest first
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
    bool turning_ = false;
    float dLat_ = 0, dLon_ = 0, dAlt_ = 0;     // the offset
    float fromLat_ = 0, fromLon_ = 0, fromTrack_ = 0;  // the turn's start
};
