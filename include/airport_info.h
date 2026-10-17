#pragma once

#include <stdint.h>

// What an airport's card says of it: its name, its ICAO code, how high it
// is, and the radio frequencies to hear it on. From the public-domain
// OurAirports dataset, by tools/gen_airport_info.py, for every airport in
// the airport table.
//
// A frequency there belongs to an airport, with no place of its own on the
// map - nothing says where a tower's aerial or a radar's head stands - so
// these are listed on the airport's card rather than marked on the radar as
// the navaids are.

// Who is on a frequency, in the order a card lists them: the airport's own
// controllers, then those around and above it, then what it broadcasts. INFO
// is what a field without a tower has instead - its information service, or
// the frequency its traffic calls on.
enum class AtcUse : uint8_t { TOWER, GROUND, DELIVERY, APPROACH, DEPARTURE, RADAR, CONTROL, ATIS, INFO };
constexpr int ATC_USE_COUNT = 9;

struct AirportFrequency {
    uint32_t khz : 24;  // 118805 for 118.805 MHz
    uint32_t use : 8;   // an AtcUse
};

struct AirportInfo {
    char code[5];  // IATA - or ICAO, for a military airfield without an IATA code
    char icao[5];  // "EGPF", or "" for the few the dataset has none for
    int16_t elevationFt;
    uint16_t firstFrequency;  // its own, in the table of them all
    uint8_t frequencyCount;   // none, for one airport in six
    bool large;               // a large airport, as OurAirports has it - the rest are medium
    const char *name;         // "Glasgow Airport", in ASCII, 66 characters at most
};

// The airport with `code` - civil, or one of the military's airfields - or
// nullptr if neither table has it.
const AirportInfo *airportInfo(const char *code);

// Its frequencies, frequencyCount of them: by use, in AtcUse's order, and by
// frequency within one - four at most for any one use.
const AirportFrequency *airportFrequencies(const AirportInfo &airport);

// "TOWER", "APPROACH" and so on.
const char *atcUseName(AtcUse use);
