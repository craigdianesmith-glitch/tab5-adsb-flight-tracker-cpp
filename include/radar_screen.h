#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"

enum class RadarAction {
    NONE,
    BACK,
    SELECT,  // outHex identifies the tapped contact
};

// A plan-position plot centred on the configured location: range rings,
// bearing marks, and every aircraft that reported a position as a blip with a
// vector showing where it'll be a minute from now.
void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military);

// Contacts are identified by ICAO hex rather than by index: a poll can land
// between the plot being drawn and the screen being tapped, and an index into
// the old list would then point at the wrong aircraft.
RadarAction radarScreenHandleTouch(int x, int y, String &outHex);
