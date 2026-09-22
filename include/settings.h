#pragma once

#include <Arduino.h>

struct LocationSettings {
    double lat;
    double lon;
    String label;
};

LocationSettings loadSettings();
void saveSettings(double lat, double lon, const String &label);
