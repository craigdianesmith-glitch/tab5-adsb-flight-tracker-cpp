#pragma once

#include <Arduino.h>
#include <vector>

struct GeoResult {
    String label;
    double lat;
    double lon;
};

bool geocodeSearch(const String &query, std::vector<GeoResult> &out);
