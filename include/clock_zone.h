#pragma once

#include <stdint.h>
#include <time.h>

// The radar's clock, in UTC or in one of a few zones' local time, with their
// summer time: the UK's, central Europe's - both changing as the EU's rules
// have it - and the four of the mainland United States. Worked out here
// rather than through the C library's TZ, which stays UTC: recordings are
// named and stamped in it, and the share page's mktime() counts on it.
enum class ClockZone : uint8_t { OFF, UTC, UK, CENTRAL_EUROPE, US_EASTERN, US_CENTRAL, US_MOUNTAIN, US_PACIFIC };
constexpr int CLOCK_ZONE_COUNT = 8;

// As the display page names it: "UK", "US East".
const char *clockZoneName(ClockZone z);

// "14:32Z" in UTC, "14:32 BST" or "09:32 EDT" in the others - into `out`,
// which wants 12 bytes.
void clockZoneFormat(time_t utc, ClockZone z, char *out, size_t size);

// The zone for home at (lat, lon), for a device whose zone has never been
// picked: the UK's in Scotland and England, central Europe's across the
// continent's middle, the US's by longitude across the mainland - and UTC
// anywhere else.
ClockZone clockZoneFor(double lat, double lon);

// The whole world's zones, from the table tools/gen_timezones.py makes: the
// one (lat, lon) is in - an index, the same for everywhere whose clocks agree
// all year round - and the time in one, "22:47 CEST" or "06:12 +03", into
// `out`, which wants 16 bytes. For the clock to keep the time where an
// aircraft followed is.
int timeZoneAt(double lat, double lon);
void timeZoneFormat(time_t utc, int zone, char *out, size_t size);
