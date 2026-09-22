#pragma once

#include <Arduino.h>

// Returns the full aircraft name for an ICAO type code (e.g. "A320" ->
// "Airbus A320"), or the code itself if not in the table. adsb.lol's `desc`
// field is always null on this endpoint, so this is filled in locally.
String lookupAircraftType(const String &icaoCode);

struct AirlineInfo {
    String name;
    uint32_t color;  // 0xRRGGBB
};

// Extracts the callsign's 3-letter ICAO airline prefix and looks it up.
// Returns false (out untouched) if the callsign doesn't match a known airline.
bool lookupAirline(const String &callsign, AirlineInfo &out);
