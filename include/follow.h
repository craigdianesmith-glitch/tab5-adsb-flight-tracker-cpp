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
// and rolls out along the centreline, braking to taxi speed and no further
// than the far end. Departing and rolling down one, it accelerates along the
// centreline, lifts off at rotation speed - by the far end at the latest -
// and climbs out on the runway's heading. Otherwise, in the air, it carries
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
