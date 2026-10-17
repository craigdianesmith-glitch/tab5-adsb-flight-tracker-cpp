#pragma once

#include <Arduino.h>

// One end of a runway: the number painted on it, and where it is.
struct RunwayEnd {
    char ident[4];  // "05", "27L"
    float lat;
    float lon;
};

struct Runway {
    char code[5];       // the airport's IATA code - or a military airfield's ICAO code, where it has no IATA one
    uint16_t widthFt;
    int16_t elevationFt;  // the airport's, the same on each of its runways
    // Withdrawn from use, but still there as pavement: drawn the way a chart
    // draws one, and never taken to be in use.
    bool closed;
    RunwayEnd end[2];   // the low-numbered end first
};

// The runways at the airport with `code`, open and closed, as `count`
// consecutive entries - or nullptr, with count 0, where the dataset places none.
// The military's airfields' too, from src/military_runways.inc. That is
// about one airport in nine of the airport table, mostly smaller fields
// whose runway ends nobody has surveyed into it.
const Runway *runwaysAt(const String &code, int &count);

// Every runway in the table, `count` of them - for a view from high up, which
// wants all of them within a few hundred miles; with `military`, the
// military's airfields' instead.
const Runway *allRunways(int &count, bool military = false);
