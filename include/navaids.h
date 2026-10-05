#pragma once

#include <Arduino.h>
#include <vector>

// The radio beacons a chart marks for aircraft to navigate by: a VOR gives
// a bearing from it, a DME the distance to it, and a TACAN - the military's -
// both. Often co-located, as a VOR-DME, a VORTAC (VOR and TACAN), or a DME
// beside an NDB.
enum class NavaidType : uint8_t { VOR, VOR_DME, VORTAC, TACAN, DME, NDB_DME };

struct Navaid {
    char ident[5];      // "GOW"
    NavaidType type;
    char channel[5];    // the DME or TACAN channel, "101X"; "" for a VOR alone
    uint32_t freqKhz;   // the VOR's, or a DME's paired VHF one; an NDB-DME's is the NDB's
    float lat, lon;
    const char *name;   // "Glasgow"
    char airport[5];    // the ICAO code of the airport it serves, or ""
};

// Every navaid within rangeNm of (lat, lon), nearest first, for the radar's
// overlay. Cached like airportsWithin(), for the same reason.
const std::vector<const Navaid *> &navaidsWithin(double lat, double lon, float rangeNm);

// What a chart calls it: "VOR-DME", "VORTAC" and so on.
const char *navaidTypeName(NavaidType type);

// Its frequency as tuned: "115.40" in MHz for a VOR or DME, "374" in kHz for
// an NDB-DME's NDB.
String navaidFrequency(const Navaid &n);

// The radar's label for it: ident and what a pilot would dial - the
// frequency, or for a TACAN its channel. "GOW 115.40", "AAL 114X".
String navaidLabel(const Navaid &n);
