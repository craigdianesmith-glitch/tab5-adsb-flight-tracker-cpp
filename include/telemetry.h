#pragma once

#include <Arduino.h>

#include "adsb_client.h"
#include "takeoff.h"

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
    // Where altFt is a pressure altitude, the feet to add to it for a height
    // above the sea - from its GNSS height, where the feed has that, or the
    // day's pressure, see dayPressureFt() - if either is known; 0 otherwise.
    // Where it is estimated to be is worked out in those heights - a landing
    // brought down to the field's - and the 3D view shows it at them.
    int16_t pressureFt;
    bool ground;
    bool hasVs;
    int16_t vsFpm;
    bool hasGs;
    int16_t gsKt;
    bool hasPos;
    bool posStale;  // the feed's last position rather than a current one
    // How old its position was when it was heard, in hundredths of a second:
    // `ms` is the position's time, and it was heard this long after.
    uint16_t ageCs;
    float lat, lon;
    bool hasTrack;
    float track;
    bool rotorcraft;  // a helicopter: see isRotorcraft()
    // How its kind takes off, for the runway simulation's roll and climb -
    // see takeoff.h. Nought, an A320's.
    TakeoffKind takeoffKind;
};
// One for each low contact's dozen reports kept, a hundred contacts and
// more, and the chart's: in the chip's own memory, which the WiFi and TLS
// need.
static_assert(sizeof(FollowSample) <= 48, "FollowSample kept small");

// Its height above the sea, as near as the feed says: its pressure altitude
// put right where that can be - see FollowSample::pressureFt.
inline int32_t aboveSeaFt(const FollowSample &s) { return s.altFt + s.pressureFt; }

// The day's pressure, from a list of aircraft just polled: the feet to add to
// a pressure altitude for a height above the sea - which a GNSS height less
// the geoid is, near enough - as the middle of the differences between the
// two, of those in it that give both, below 6,000ft (higher, the air's
// temperature makes more of it), five of them at least. For one that gives
// only its pressure altitude: see followSampleOf().
void notePressure(const std::vector<Aircraft> &list);
// It, if a list has given it in the last half hour.
bool dayPressureFt(int32_t &ft);
constexpr int DAY_PRESSURE_FROM = 5;
constexpr uint32_t DAY_PRESSURE_KEEP_MS = 30UL * 60 * 1000;
// One aircraft's difference, as notePressure() takes them: false for one
// that doesn't give both, or is too high to count.
bool pressureDifferenceOf(const Aircraft &a, int32_t &ft);
// Where the day's pressure comes from, for a sample: the live polls', as
// above, or a recording's own - a replay's day isn't today.
using DayPressureFn = bool (*)(int32_t &ft);

// The sample an aircraft's report makes at `ms`, a pressure altitude with
// no GNSS height beside it put right by `dayPressure`.
FollowSample followSampleOf(const Aircraft &ac, uint32_t ms, DayPressureFn dayPressure = dayPressureFt);
