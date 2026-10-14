#include "regions.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <algorithm>

#include "airports.h"

namespace {

// Nearer an airport than this, something is taken to be where that airport is.
constexpr float REGION_AIRPORT_NM = 40.0f;

struct RegionRing {
    uint32_t first, count;  // in points, into REGION_POINTS
    float south, west, north, east;
};
struct RegionShape {
    Region region;
    uint16_t firstRing, ringCount;
};
struct AirportRegion {
    char code[4];
    Region region;
};

#include "regions_table.inc"

// Whether (lat, lon) is inside `ring`: a line from it due east crosses the
// ring's edge an odd number of times. A hole - a lake, in Natural Earth's
// countries - is a ring of its own, so a point in it crosses one more.
bool inRing(const RegionRing &r, float lat, float lon) {
    if (lat < r.south || lat > r.north || lon < r.west || lon > r.east) {
        return false;
    }
    bool in = false;
    const float *p = REGION_POINTS + 2 * r.first;
    for (uint32_t i = 0, j = r.count - 1; i < r.count; j = i++) {
        float latI = p[2 * i], lonI = p[2 * i + 1], latJ = p[2 * j], lonJ = p[2 * j + 1];
        if ((latI > lat) != (latJ > lat) && lon < lonI + (lonJ - lonI) * (lat - latI) / (latJ - latI)) {
            in = !in;
        }
    }
    return in;
}

// Which outline (lat, lon) is inside, if any.
Region regionAt(float lat, float lon) {
    for (const RegionShape &s : REGION_SHAPES) {
        bool in = false;
        for (uint16_t i = 0; i < s.ringCount; i++) {
            in ^= inRing(REGION_RINGS[s.firstRing + i], lat, lon);
        }
        if (in) {
            return s.region;
        }
    }
    return Region::ELSEWHERE;
}

}  // namespace

Region airportRegion(const char *code) {
    const AirportRegion *end = AIRPORT_REGIONS + sizeof(AIRPORT_REGIONS) / sizeof(AIRPORT_REGIONS[0]);
    const AirportRegion *at = std::lower_bound(AIRPORT_REGIONS, end, code, [](const AirportRegion &a, const char *c) {
        return strcmp(a.code, c) < 0;
    });
    return at != end && strcmp(at->code, code) == 0 ? at->region : Region::ELSEWHERE;
}

Region regionNear(float lat, float lon) {
    if (isnan(lat) || isnan(lon)) {
        return Region::ELSEWHERE;
    }
    Airport a;
    if (nearestAirport(lat, lon, a, REGION_AIRPORT_NM)) {
        return airportRegion(a.code.c_str());
    }
    return regionAt(lat, lon);
}
