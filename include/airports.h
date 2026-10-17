#pragma once

#include <Arduino.h>
#include <vector>

struct Airport {
    String code;  // IATA - or, for a military airfield without one, ICAO: "EGXE"
    double lat;
    double lon;
    bool military = false;  // one of the military's airfields, from src/military_table.inc
};

// The airport nearest (lat, lon), or false if the nearest is further off than
// maxNm - past that it says more about the dataset than about where you are.
// With `military`, the military's airfields as well: where an aircraft has
// come from or is landing, rather than which airport home is by.
bool nearestAirport(double lat, double lon, Airport &out, float maxNm = 120.0f, bool military = true);

// The airport with `code`, civil or military, or false if neither table has it.
bool airportByCode(const String &code, Airport &out);

// Just the code, or "" where nearestAirport() would return false.
String nearestAirportCode(double lat, double lon, float maxNm = 120.0f, bool military = true);

// Every airport within rangeNm of (lat, lon), the military's airfields among
// them, nearest first, for the radar's overlay. Cached on its own: the radar asks for it and for the nearest
// airport on every refresh, about different points, and sharing one cache
// would have each evict the other.
const std::vector<Airport> &airportsWithin(double lat, double lon, float rangeNm);
