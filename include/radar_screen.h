#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "config.h"

enum class RadarAction {
    NONE,
    BACK,
    SELECT,         // outHex identifies the tapped contact
    TOGGLE_CENTRE,  // switch between centring on home and on the airport
};

// A plan-position plot centred on the configured location (lat, lon) or the
// airport nearest it: range rings, bearing marks, and every aircraft that
// reported a position as a blip with a vector showing where it'll be a minute
// from now.
//
// `full` repaints the whole screen, for arriving at it or changing what it is
// centred on. Without it only the plot and the footer readouts are redrawn,
// for a data refresh; that relies on the rest still being on the canvas from
// the last full draw.
void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military, RadarCentre centre, bool full);

// Contacts are identified by ICAO hex rather than by index: a poll can land
// between the plot being drawn and the screen being tapped, and an index into
// the old list would then point at the wrong aircraft.
RadarAction radarScreenHandleTouch(int x, int y, String &outHex);
