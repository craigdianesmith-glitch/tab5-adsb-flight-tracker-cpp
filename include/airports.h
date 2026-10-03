#pragma once

#include <Arduino.h>
#include <vector>

struct Airport {
    String code;  // IATA
    double lat;
    double lon;
};

// The airport nearest (lat, lon), or false if the nearest is further off than
// maxNm - past that it says more about the dataset than about where you are.
bool nearestAirport(double lat, double lon, Airport &out, float maxNm = 120.0f);

// Just the code, or "" where nearestAirport() would return false.
String nearestAirportCode(double lat, double lon, float maxNm = 120.0f);

// Every airport within rangeNm of (lat, lon), nearest first, for the radar's
// overlay. Cached on its own: the radar asks for it and for the nearest
// airport on every refresh, about different points, and sharing one cache
// would have each evict the other.
const std::vector<Airport> &airportsWithin(double lat, double lon, float rangeNm);
