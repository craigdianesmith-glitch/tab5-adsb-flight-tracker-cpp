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
    bool cargo = false;  // a cargo operator: FedEx, UPS, DHL...
};

// Extracts the callsign's 3-letter ICAO airline prefix and looks it up.
// Returns false (out untouched) if the callsign doesn't match a known airline.
bool lookupAirline(const String &callsign, AirlineInfo &out);

// Whether the callsign is a cargo operator's, as the airline table has it -
// for the table and the radar to mark, and the filter's Cargo chip. A cargo
// flight under a passenger airline's callsign - Qatar's, Emirates' - isn't one.
bool isCargo(const String &callsign);

// Whether an aircraft is a rotorcraft - a helicopter, a gyroplane, a
// tilt-rotor - by the feed's category ("A7"), or failing that its type. One
// takes off and lands where it likes, not on a runway, and hovers.
bool isRotorcraft(const String &type, const String &category);

// Whether an aircraft can stand still in the air over one spot: a
// rotorcraft, or by the feed's category a balloon or airship (B2), a
// parachutist (B3), a hang-glider or paraglider - the feed's ultralights,
// B4 - or a drone (B6). Anything else with a height is going somewhere.
bool canHover(const String &type, const String &category);

// A vehicle on an airfield - a fire engine, a follow-me car, an ops van: the
// feed's types GND, SERV and EMER, or its categories C1 and C2.
bool isGroundVehicle(const String &type, const String &category);
