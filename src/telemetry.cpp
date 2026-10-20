#include "telemetry.h"

#include <algorithm>

#include "aircraft_db.h"
#include "airports.h"
#include "geoid.h"
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

// The day's pressure: see notePressure(). Taken from aircraft below this.
constexpr int PRESSURE_BELOW_FT = 6000;
int32_t g_pressureFt = 0;
uint32_t g_pressureMs = 0;
bool g_havePressure = false;

}  // namespace

bool dayPressureFt(int32_t &ft) {
    ft = g_pressureFt;
    return g_havePressure && millis() - g_pressureMs < DAY_PRESSURE_KEEP_MS;
}

bool pressureDifferenceOf(const Aircraft &a, int32_t &ft) {
    if (!a.hasAltGeom || !a.hasPos || a.posStale || a.altStr == "GND" || a.altStr == "?") {
        return false;
    }
    int32_t pressure = a.altStr.toInt();
    if (pressure >= PRESSURE_BELOW_FT) {
        return false;
    }
    ft = a.altGeom - (int32_t)lroundf(geoidM(a.lat, a.lon) * 3.2808f) - pressure;
    return true;
}

void notePressure(const std::vector<Aircraft> &list) {
    int32_t differences[64];
    int n = 0;
    for (const Aircraft &a : list) {
        if (n == 64) {
            break;
        }
        if (pressureDifferenceOf(a, differences[n])) {
            n++;
        }
    }
    if (n < DAY_PRESSURE_FROM) {
        return;
    }
    std::nth_element(differences, differences + n / 2, differences + n);
    int32_t was = g_pressureFt;
    bool had = g_havePressure;
    g_pressureFt = differences[n / 2];
    g_pressureMs = millis();
    g_havePressure = true;
    if (!had || abs(g_pressureFt - was) >= 30) {
        Serial.printf("[telemetry] the day's pressure: %+d ft on a pressure altitude, from %d aircraft\n",
                      (int)g_pressureFt, n);
    }
}

FollowSample followSampleOf(const Aircraft &ac, uint32_t ms, DayPressureFn dayPressure) {
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
    // A rotorcraft by its GNSS height, less the geoid, where the feed has it:
    // its pressure altitude is out by the day's pressure - by hundreds of
    // feet - and near the ground that is the difference between on it and
    // hovering, or under it; and its transponder often never says it is on
    // the ground (an R44 at Leicester, its field 469ft: 200ft by its
    // pressure, 491ft by its GNSS). One that gives only its pressure
    // altitude, put right by the day's pressure, as the aircraft about have
    // it.
    s.rotorcraft = isRotorcraft(ac.type, ac.category);
    int32_t pressure;
    if (s.rotorcraft && !s.ground && ac.hasPos) {
        if (ac.hasAltGeom) {
            s.hasAlt = true;
            s.altFt = ac.altGeom - (int32_t)lroundf(geoidM(ac.lat, ac.lon) * 3.2808f);
        } else if (s.hasAlt && dayPressure(pressure)) {
            s.altFt += pressure;
        }
    }
    // Anything else keeps its pressure altitude, as the feed gives it and the
    // labels show it - with the feet that put it right beside it: the
    // difference from its GNSS height, less the geoid, where the feed has
    // that, or else the day's pressure. A landing is brought down from its
    // height to the field's, and from the pressure altitude alone - a few
    // hundred feet high on a day of low pressure - the last of that would
    // come down as a dive onto the runway; and the 3D view had the traffic
    // round an airfield that much above it.
    if (!s.rotorcraft && !s.ground && s.hasAlt && ac.hasPos) {
        if (ac.hasAltGeom) {
            pressure = ac.altGeom - (int32_t)lroundf(geoidM(ac.lat, ac.lon) * 3.2808f) - s.altFt;
            s.pressureFt = (int16_t)constrain((int)pressure, -3000, 3000);
        } else if (dayPressure(pressure)) {
            s.pressureFt = (int16_t)constrain((int)pressure, -3000, 3000);
        }
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
