#pragma once

#include <Arduino.h>

// Returns the full aircraft name for an ICAO type code (e.g. "A320" ->
// "Airbus A320"), or the code itself if not in the table. adsb.lol's `desc`
// field is always null on this endpoint, so this is filled in locally.
//
// Plenty of designators cover both a civil and a military aircraft - EC45 is
// an air ambulance or a UH-72 Lakota, BE20 a King Air or a C-12 Huron - so
// `military` (from the aircraft's dbFlags) picks which table is consulted
// first. Either way the other is still searched, so a type only listed in one
// of them resolves whatever the flag says.
String lookupAircraftType(const String &icaoCode, bool military = false);

struct AirlineInfo {
    String name;
    uint32_t color;  // 0xRRGGBB
};

// Extracts the callsign's 3-letter ICAO airline prefix and looks it up.
// Returns false (out untouched) if the callsign doesn't match a known airline.
bool lookupAirline(const String &callsign, AirlineInfo &out);
