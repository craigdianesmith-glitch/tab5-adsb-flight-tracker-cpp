#pragma once

#include <Arduino.h>

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
