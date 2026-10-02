#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"

// Why a flight is worth interrupting for. A bitmask rather than one value: an
// aircraft can be more than one of these at once - a watchlisted type
// squawking 7700, say - and each is switched on and off separately.
// Stored in NVS by value, so the bits are not free to change.
enum AlertReason : uint8_t {
    ALERT_EMERGENCY = 1 << 0,  // squawking 7700, 7600 or 7500
    ALERT_MILITARY = 1 << 1,   // military, seen while civil traffic is showing
    ALERT_WATCHLIST = 1 << 2,  // matches an entry on the user's list
    ALERT_RARE = 1 << 3,       // a notable type, or flagged by the feed's database
};
constexpr uint8_t ALERT_ALL = ALERT_EMERGENCY | ALERT_MILITARY | ALERT_WATCHLIST | ALERT_RARE;

struct AlertRules {
    uint8_t enabled = ALERT_ALL;
    std::vector<String> watch;  // upper-cased, from parseWatchlist()
};

// Splits the watchlist as typed - entries separated by spaces or commas - into
// upper-cased tokens.
std::vector<String> parseWatchlist(const String &text);

// What's wrong with a watchlist as typed, worded for the screen - or empty if
// every entry could match something. Every kind of entry is letters, digits
// and at most a dash, between 2 and 10 long.
String watchlistProblem(const String &text);

// Which of the enabled rules this aircraft trips. `civilMode` says whether
// civil traffic is what's being shown: in military mode every contact is
// military, so that rule would flag the lot.
uint8_t evaluateAlert(const Aircraft &ac, const AlertRules &rules, bool civilMode);

// The most important reason in the mask, worded for the banner.
String alertReasonText(const Aircraft &ac, uint8_t reasons);

// Whether one alert outranks another, for choosing which a banner shows when
// several arrive at once: an emergency first, then whatever is nearest.
bool alertOutranks(const Aircraft &a, const Aircraft &b);

// --- the callsign flash ------------------------------------------------------
// When an alert goes off, the callsigns of the aircraft that raised it flash
// red a few times on the table and the live radar - alternating with the
// colour an ordinary flight is drawn in - and then settle to the alert colour.
// UI-side state: only loop() and what it draws may touch it.

enum class AlertFlash { NONE, RED, OFF };

// Starts a flash for these aircraft (ICAO hex), replacing any still running.
void alertFlashStart(const std::vector<String> &hexes);

// Where this aircraft's callsign is in a flash right now.
AlertFlash alertFlash(const String &hex);

// Changes at each step of a running flash, and is 0 once none is: a screen
// showing callsigns redraws when it moves.
uint32_t alertFlashStep();
