#pragma once

#include <Arduino.h>

#include "adsb_client.h"

// One report of a followed aircraft, kept over the minutes before it - live,
// and read back from a recording that follows one: its height and speed for
// the telemetry panel's chart, and where it was going, for estimating where
// it is once the feed loses it.
struct FollowSample {
    uint32_t ms;  // on whatever clock the caller keeps: only differences matter
    // As reported - barometric, so above sea level - or on the ground, the
    // elevation of the field it is on, so a touchdown meets the ground where
    // the ground is rather than at sea level.
    int32_t altFt;
    bool hasAlt;
    bool ground;
    bool hasVs;
    int16_t vsFpm;
    bool hasGs;
    int16_t gsKt;
    bool hasPos;
    bool posStale;  // the feed's last position rather than a current one
    float lat, lon;
    bool hasTrack;
    float track;
};

// The sample an aircraft's report makes at `ms`.
FollowSample followSampleOf(const Aircraft &ac, uint32_t ms);
