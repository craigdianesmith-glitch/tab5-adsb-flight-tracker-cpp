#pragma once

#include <Arduino.h>

// IATA code of the airport nearest (lat, lon), or "" if the nearest is further
// off than maxNm - past that it says more about the dataset than about where
// you are.
String nearestAirportCode(double lat, double lon, float maxNm = 120.0f);
