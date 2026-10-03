#include "telemetry.h"

#include "airports.h"
#include "runways.h"

namespace {

// An aircraft on the ground is at an airport; this far from the table's point
// for it covers the largest of them.
constexpr float FIELD_SEARCH_NM = 3.0f;

// The fields last looked up, and where from. Ground reports come in runs -
// a landing roll, then the taxi in - at an airport or two, so most are near
// one asked about already, and a scan of the airport table each time would
// add up over a recording.
struct FieldCache {
    float lat = NAN, lon = NAN;
    int32_t ft = 0;
    uint32_t used = 0;
};
FieldCache g_fields[4];
uint32_t g_fieldUse = 0;
constexpr float FIELD_REUSE_DEG = 0.02f;  // about a mile

int32_t fieldElevationNear(float lat, float lon) {
    for (FieldCache &c : g_fields) {
        if (fabsf(lat - c.lat) < FIELD_REUSE_DEG && fabsf(lon - c.lon) < FIELD_REUSE_DEG) {
            c.used = ++g_fieldUse;
            return c.ft;
        }
    }
    FieldCache *oldest = &g_fields[0];
    for (FieldCache &c : g_fields) {
        if (c.used < oldest->used) {
            oldest = &c;
        }
    }
    *oldest = {lat, lon, 0, ++g_fieldUse};
    Airport airport;
    if (nearestAirport(lat, lon, airport, FIELD_SEARCH_NM)) {
        int count = 0;
        const Runway *runways = runwaysAt(airport.code, count);
        if (count) {
            oldest->ft = runways[0].elevationFt;
        }
    }
    return oldest->ft;
}

}  // namespace

FollowSample followSampleOf(const Aircraft &ac, uint32_t ms) {
    FollowSample s = {};
    s.ms = ms;
    s.ground = (ac.status == "GROUND" || ac.status == "TAXI" || ac.altStr == "GND");
    if (s.ground) {
        s.hasAlt = true;
        s.altFt = ac.hasPos ? fieldElevationNear(ac.lat, ac.lon) : 0;
    } else if (ac.altStr != "?") {
        s.hasAlt = true;
        s.altFt = ac.altStr.toInt();
    }
    s.hasVs = ac.hasVertRate && !s.ground;
    s.vsFpm = s.hasVs ? (int16_t)constrain((int)lroundf(ac.vertRate), -32000, 32000) : 0;
    s.hasPos = ac.hasPos;
    s.posStale = ac.posStale;
    s.lat = ac.lat;
    s.lon = ac.lon;
    s.hasTrack = ac.hasTrack;
    s.track = ac.track;
    s.hasGs = ac.speedStr != "?";
    s.gsKt = s.hasGs ? (int16_t)ac.speedStr.toInt() : 0;
    return s;
}
