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
    // the ground is rather than at sea level. A rotorcraft's, its GNSS
    // height less the geoid where the feed has it: see followSampleOf().
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
    bool rotorcraft;  // a helicopter: see isRotorcraft()
};

// The sample an aircraft's report makes at `ms`.
FollowSample followSampleOf(const Aircraft &ac, uint32_t ms);

// The day's pressure, from a list of aircraft just polled: the feet to add to
// a pressure altitude for a height above the sea - which a GNSS height less
// the geoid is, near enough - as the middle of the differences between the
// two, of those in it that give both, below 6,000ft (higher, the air's
// temperature makes more of it), five of them at least. For a rotorcraft
// that gives only its pressure altitude: see followSampleOf().
void notePressure(const std::vector<Aircraft> &list);
// It, if a list has given it in the last half hour.
bool dayPressureFt(int32_t &ft);
