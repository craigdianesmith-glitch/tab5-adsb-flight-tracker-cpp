#include "clock_zone.h"

#include <stdio.h>

#include "regions.h"

namespace {

// The world's zones' clocks - see tools/gen_timezones.py: each a zone's
// offset and name in 2025 and its changes after, to 2037; then its rules.
struct ZoneChange {
    uint32_t at;   // seconds since 1970, UTC
    int16_t east;  // the offset after it, minutes east of UTC
    uint8_t name;  // and its name, in ZONE_NAMES
};
struct ZoneClock {
    int16_t east;  // in 2025
    uint8_t name;
    uint16_t first, count;  // its changes in ZONE_CHANGES
    const char *rule;       // a POSIX TZ string: its rules from 2037
};

#include "timezone_table.inc"

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

// The day, as days since 1970, of the month's `week`th weekday `day` -
// Sunday 0 - or, for week 5, its last.
int64_t weekdayIn(int y, int m, int week, int day) {
    if (week >= 5) {
        int64_t last = daysFromCivil(m == 12 ? y + 1 : y, m == 12 ? 1 : m + 1, 1) - 1;
        return last - ((weekday(last) - day + 7) % 7);
    }
    int64_t first = daysFromCivil(y, m, 1);
    return first + (day - weekday(first) + 7) % 7 + 7 * (week - 1);
}

// A zone's rules from its POSIX TZ string, "CET-1CEST,M3.5.0,M10.5.0/3":
// the names and offsets of its standard time and - if it has one - its
// summer time, and the day and time each starts.
struct Posix {
    char name[2][8];  // standard, summer
    int east[2];      // seconds east of UTC
    bool summer;
    struct When {
        char kind;  // 'M': a month's nth weekday; 'J': a day 1-365, February 29th never; 'D': a day 0-365
        int month, week, day, seconds;
    } begins, ends;
};

const char *number(const char *s, int &n) {
    n = 0;
    while (*s >= '0' && *s <= '9') {
        n = n * 10 + (*s++ - '0');
    }
    return s;
}

// A name: letters, or anything between < and >, as "<+03>".
const char *name(const char *s, char *out) {
    int n = 0;
    bool quoted = *s == '<';
    s += quoted;
    for (; *s != 0 && (quoted ? *s != '>' : ((*s | 32) >= 'a' && (*s | 32) <= 'z')); s++) {
        if (n < 7) {
            out[n++] = *s;
        }
    }
    out[n] = 0;
    return s + (quoted && *s == '>');
}

// [+-]hh[:mm[:ss]], in seconds.
const char *clock(const char *s, int &seconds) {
    int sign = *s == '-' ? -1 : 1, h, m = 0, sec = 0;
    s += *s == '-' || *s == '+';
    s = number(s, h);
    if (*s == ':') {
        s = number(s + 1, m);
    }
    if (*s == ':') {
        s = number(s + 1, sec);
    }
    seconds = sign * (h * 3600 + m * 60 + sec);
    return s;
}

const char *when(const char *s, Posix::When &w) {
    w = {'D', 0, 0, 0, 7200};
    if (*s == 'M') {
        w.kind = 'M';
        s = number(s + 1, w.month);
        s = number(s + (*s == '.'), w.week);
        s = number(s + (*s == '.'), w.day);
    } else {
        w.kind = *s == 'J' ? 'J' : 'D';
        s = number(s + (*s == 'J'), w.day);
    }
    if (*s == '/') {
        s = clock(s + 1, w.seconds);
    }
    return s;
}

void parse(const char *s, Posix &p) {
    p = {};
    int west;
    s = clock(name(s, p.name[0]), west);
    p.east[0] = -west;  // POSIX counts west of UTC
    p.summer = *s != 0 && *s != ',';
    if (!p.summer) {
        return;
    }
    s = name(s, p.name[1]);
    p.east[1] = p.east[0] + 3600;
    if (*s != 0 && *s != ',') {
        s = clock(s, west);
        p.east[1] = -west;
    }
    if (*s == ',') {
        s = when(s + 1, p.begins);
    }
    if (*s == ',') {
        when(s + 1, p.ends);
    }
}

int64_t dayOf(const Posix::When &w, int year) {
    if (w.kind == 'M') {
        return weekdayIn(year, w.month, w.week, w.day);
    }
    int64_t jan1 = daysFromCivil(year, 1, 1);
    bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
    return w.kind == 'J' ? jan1 + w.day - 1 + (leap && w.day >= 60) : jan1 + w.day;
}

// Whether it is summer time there at `utc`: from its start, at standard
// time's hour, to its end, at summer time's - the other way round across
// the new year south of the equator.
bool inSummer(const Posix &p, time_t utc) {
    if (!p.summer) {
        return false;
    }
    time_t standard = utc + p.east[0];
    struct tm tm;
    gmtime_r(&standard, &tm);
    int year = tm.tm_year + 1900;
    int64_t t = (int64_t)utc, start = dayOf(p.begins, year) * 86400 + p.begins.seconds - p.east[0],
            end = dayOf(p.ends, year) * 86400 + p.ends.seconds - p.east[1];
    return start < end ? t >= start && t < end : !(t >= end && t < start);
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

int timeZoneAt(double lat, double lon) {
    int row = (int)((90 - lat) / ZONE_STEP), col = (int)((lon + 180) / ZONE_STEP);
    row = row < 0 ? 0 : row >= ZONE_ROWS ? ZONE_ROWS - 1 : row;
    col = ((col % ZONE_COLS) + ZONE_COLS) % ZONE_COLS;
    const uint8_t *p = ZONE_RUNS + ZONE_ROW_AT[row], *end = ZONE_RUNS + ZONE_ROW_AT[row + 1];
    for (int at = 0; p < end;) {
        int length = *p++;
        if (length & 0x80) {
            length = (length & 0x7F) << 8 | *p++;
        }
        int rule = *p++;
        if (col < (at += length)) {
            return rule;
        }
    }
    return 0;
}

void timeZoneFormat(time_t utc, int zone, char *out, size_t size) {
    constexpr int CLOCKS = (int)(sizeof ZONE_CLOCKS / sizeof ZONE_CLOCKS[0]);
    const ZoneClock &z = ZONE_CLOCKS[zone >= 0 && zone < CLOCKS ? zone : 0];
    int east;  // seconds
    const char *name;
    Posix p;
    if ((int64_t)utc >= ZONE_UNTIL) {
        // Past the changes the table has: by its rules.
        parse(z.rule, p);
        bool summer = inSummer(p, utc);
        east = p.east[summer];
        name = p.name[summer];
    } else {
        east = z.east * 60;
        name = ZONE_NAMES[z.name];
        for (int i = z.first; i < z.first + z.count && (int64_t)ZONE_CHANGES[i].at <= (int64_t)utc; i++) {
            east = ZONE_CHANGES[i].east * 60;
            name = ZONE_NAMES[ZONE_CHANGES[i].name];
        }
    }
    time_t local = utc + east;
    struct tm tm;
    gmtime_r(&local, &tm);
    snprintf(out, size, "%02d:%02d %s", tm.tm_hour, tm.tm_min, name);
}
