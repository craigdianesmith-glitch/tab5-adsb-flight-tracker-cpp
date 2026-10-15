#include "clock_zone.h"

#include <stdio.h>

#include "regions.h"

namespace {

enum class Rule : uint8_t { NONE, EU, US };

struct Zone {
    const char *name;
    int offsetMin;  // standard time's, east of UTC
    const char *standard, *summer;
    Rule rule;
};

const Zone ZONES[CLOCK_ZONE_COUNT] = {
    {"Off", 0, "", "", Rule::NONE},
    {"UTC", 0, "Z", "Z", Rule::NONE},
    {"UK", 0, "GMT", "BST", Rule::EU},
    {"C. Europe", 60, "CET", "CEST", Rule::EU},
    {"US East", -300, "EST", "EDT", Rule::US},
    {"US Central", -360, "CST", "CDT", Rule::US},
    {"US Mountain", -420, "MST", "MDT", Rule::US},
    {"US Pacific", -480, "PST", "PDT", Rule::US},
};

// Days since 1970-01-01 of a date in the Gregorian calendar.
int64_t daysFromCivil(int y, int m, int d) {
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int yoe = (int)(y - era * 400);
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

// 1970-01-01 was a Thursday; Sunday is 0.
int weekday(int64_t days) { return (int)(((days + 4) % 7 + 7) % 7); }

// The day of the month's `nth` Sunday, counting from 1 - or, for 0, its last.
int64_t sunday(int y, int m, int nth) {
    if (nth == 0) {
        int64_t last = daysFromCivil(m == 12 ? y + 1 : y, m == 12 ? 1 : m + 1, 1) - 1;
        return last - weekday(last);
    }
    int64_t first = daysFromCivil(y, m, 1);
    return first + (7 - weekday(first)) % 7 + 7 * (nth - 1);
}

bool summerTime(time_t utc, const Zone &z, int year) {
    int64_t t = (int64_t)utc;
    switch (z.rule) {
    case Rule::EU:
        // From 01:00 UTC on March's last Sunday to 01:00 UTC on October's,
        // everywhere at once.
        return t >= sunday(year, 3, 0) * 86400 + 3600 && t < sunday(year, 10, 0) * 86400 + 3600;
    case Rule::US:
        // From 02:00 standard time on March's second Sunday to 02:00 summer
        // time - 01:00 standard - on November's first.
        return t >= sunday(year, 3, 2) * 86400 + 7200 - z.offsetMin * 60 &&
               t < sunday(year, 11, 1) * 86400 + 3600 - z.offsetMin * 60;
    case Rule::NONE:
        break;
    }
    return false;
}

}  // namespace

const char *clockZoneName(ClockZone z) { return ZONES[(int)z % CLOCK_ZONE_COUNT].name; }

void clockZoneFormat(time_t utc, ClockZone z, char *out, size_t size) {
    const Zone &zone = ZONES[(int)z % CLOCK_ZONE_COUNT];
    struct tm tm;
    gmtime_r(&utc, &tm);
    bool summer = summerTime(utc, zone, tm.tm_year + 1900);
    time_t local = utc + (zone.offsetMin + (summer ? 60 : 0)) * 60;
    gmtime_r(&local, &tm);
    if (z == ClockZone::UTC) {
        snprintf(out, size, "%02d:%02dZ", tm.tm_hour, tm.tm_min);
    } else {
        snprintf(out, size, "%02d:%02d %s", tm.tm_hour, tm.tm_min, summer ? zone.summer : zone.standard);
    }
}

ClockZone clockZoneFor(double lat, double lon) {
    Region r = regionNear((float)lat, (float)lon);
    if (r == Region::SCOTLAND || r == Region::ENGLAND) {
        return ClockZone::UK;
    }
    if (r == Region::USA && lat > 24 && lat < 50) {  // the mainland: not Alaska, Hawaii or Puerto Rico
        return lon > -86.5   ? ClockZone::US_EASTERN
               : lon > -101  ? ClockZone::US_CENTRAL
               : lon > -114.5 ? ClockZone::US_MOUNTAIN
                              : ClockZone::US_PACIFIC;
    }
    // The rest of the British Isles keep the UK's time - Ireland calling its
    // summer's IST rather than BST.
    if (lat > 49.8 && lat < 61 && lon > -10.7 && lon < 1.8) {
        return ClockZone::UK;
    }
    // Spain to Poland, Italy to Norway, roughly: not Portugal, which keeps
    // the UK's hours, nor north Africa's coast across the water from Spain.
    // The display page picks otherwise.
    bool portugal = lat < 42.2 && lon < -6.9;
    bool africa = lat < 37.4 && lon > -2 && lon < 11.5;
    if (lat > 36 && lat < 71 && lon > -9.5 && !portugal && !africa && (lon < 15 || (lon < 24 && lat < 55))) {
        return ClockZone::CENTRAL_EUROPE;
    }
    return ClockZone::UTC;
}
