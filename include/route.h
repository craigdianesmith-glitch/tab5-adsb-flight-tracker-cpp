#pragma once

#include <Arduino.h>

// Where a flight is from and going to, by its callsign, from adsbdb.com's
// free lookup - crowd-sourced, so now and then wrong, and for a scheduled
// flight number only. Asked for as an aircraft's details open, in a task of
// its own so the screen doesn't wait on it, and kept for the next time.
struct Route {
    String fromCode, fromCity;  // "GVA", "Geneva"
    String toCode, toCity;      // "LTN", "London"
};

enum class RouteState : uint8_t {
    NONE,     // not asked for, or not a callsign there can be a route for
    LOOKING,  // asked, no answer yet
    FOUND,
    UNKNOWN,  // adsbdb hasn't one, or couldn't be reached
};

// Starts a lookup for `callsign`, unless one is known or under way.
void routeRequest(const String &callsign);
RouteState routeFor(const String &callsign, Route &out);
// Goes up as each lookup finishes, for a screen showing one to redraw.
uint32_t routeGeneration();
