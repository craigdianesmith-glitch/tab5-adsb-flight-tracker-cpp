#pragma once

#include <Arduino.h>

#include <vector>

#include "adsb_client.h"

// What the table and the radar show of the traffic polled, as the filter
// screen sets it: a band of height and of speed, civil or military or both,
// and airlines and types. The airport zoom, the alerts and the aircraft
// followed are never filtered, and an emergency is never hidden. A recording
// keeps every aircraft, with the filter written into it, and a replay shows
// the radar through the filter it was recorded with.
constexpr int FILTER_TOP_FT = 45000;  // the height slider's top: no ceiling
constexpr int FILTER_TOP_KT = 600;    // and the speed's

struct TrafficFilterSet {
    int minFt = 0, maxFt = FILTER_TOP_FT;
    int minKt = 0, maxKt = FILTER_TOP_KT;
    // Which kinds: civil - passenger flights and the rest - cargo, by the
    // airline table's cargo operators, and military. At least one.
    bool civil = true, cargo = true, military = true;
    String airlines;  // airline codes, as a callsign begins: "EZY RYR"
    String types;     // type designators, or how they begin: "A32 B738"
    String hideTypes;  // and those left out, whatever else: "GLID", the gliders

    bool active() const;
    bool passes(const Aircraft &a) const;
};

// The filter in force, and taking it to the list: `list` cut to what passes,
// keeping `keepHex` - the followed aircraft - whatever it is. `isNew`, if
// given, is cut alongside.
const TrafficFilterSet &trafficFilter();

// What the table and the radar show, by kind, for their titles: what the
// filter's kinds leave - civil and cargo together being all the civil
// traffic, and with military as well everything, which keeps the civil name -
// so "Civilian", "Cargo", "Military", "Civil & military" or "Cargo &
// military". `code` for the radar's letters, "CRG" or "CRG+MIL", rather than
// the table's words.
String shownKinds(bool code);
// The same for `f` rather than the filter in force: a replay's. With
// `militaryMode`, a recording made in the military mode there once was: the
// military alone.
String kindsName(const TrafficFilterSet &f, bool militaryMode, bool code);

// The widest the poll radius may be: MAX_RADIUS_NM, or with a filter on as
// far as the feeds' area queries go, there being fewer aircraft to show and
// so room to show them further out. `filtered` for a filter on.
int radiusCeilingNm(bool filtered);
void setTrafficFilter(const TrafficFilterSet &f);
void applyTrafficFilter(std::vector<Aircraft> &list, std::vector<uint8_t> *isNew = nullptr,
                        const String &keepHex = String());
// The same through `f` - a recording's, in a replay.
void filterList(const TrafficFilterSet &f, std::vector<Aircraft> &list, std::vector<uint8_t> *isNew = nullptr,
                const String &keepHex = String());

// The filter as a recording keeps it - see recorder.h's `Q` lines - on one
// line: "0 45000 0 600 -G- FDX,UPS - TWR", the height and speed bands, which
// kinds (Civil, carGo, Military), and the airlines, the types and the types
// left out, comma-separated or "-" for none. And read back - a line without
// the last, as the first were written, hiding none; false for a line that
// isn't one.
String filterText(const TrafficFilterSet &f);
bool parseFilterText(const char *text, TrafficFilterSet &out);
