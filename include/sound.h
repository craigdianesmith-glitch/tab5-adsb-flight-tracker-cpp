#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "adsb_client.h"
#include "airport_info.h"
#include "config.h"
#include "navaids.h"

// Feedback beeps through the Tab5's built-in speaker.
//
// M5.begin() configures the board's ES8388 codec and amp but stops short of
// starting the I2S output, so soundInit() has to do that before anything will
// be audible.
void soundInit();

// Two-note rise, played once the firmware is up.
// The jet take-off the splash screen plays to - see tools/gen_jet.py.
void soundTakeoff();

// Short blip when an aircraft that wasn't there before appears in the table.
void soundNewFlight();

// A key's click for each one pressed on the on-screen keyboard, as a phone's
// keyboard makes - see tools/gen_click.py.
void soundKeyClick();

// The words are said in three voices, by where what is said is about -
// Scotland's in a Scottish voice, England's in an English one, the United
// States' in an American one, as regions.h has them - and everywhere else's
// in the one chosen on the alerts screen, `accent`.
void soundSetElsewhereAccent(Accent accent);

// A rising three-note chime for an interesting flight, or for an emergency
// squawk a two-tone warble that can't be mistaken for it - and then, spoken,
// what it is: "Emergency", "Watchlist", "Rare aircraft" or "Military", for
// the reason the banner names, out of the AlertReason bits in `reasons`;
// `callsign` spelt out in the phonetic alphabet, "Echo Zulu Yankee one two";
// and `typeName`, as lookupAircraftType() has it, as its maker and model:
// "Airbus... Alpha three two zero neo". In the voice of where the aircraft
// is, (lat, lon) - NAN for not known. Returns at once: it is played out by
// soundTick().
//
// Not `spoken`, the chime or warble alone - for an alert while following
// something else - and in its turn: after what is being said, an
// announcement or a reading, rather than cutting it short; ahead of any
// waiting. Kept waiting more than a few seconds, it would sound well after
// its banner, and is dropped.
void soundAlert(uint8_t reasons, const String &callsign, const String &typeName, float lat, float lon,
                bool spoken = true);

// On the airport zoom, a take-off or landing as a controller would clear it:
// "easyJet five three Tango Hotel, runway two three, cleared for take-off",
// or "cleared to land". The airline is said by its name where the airline
// table and the voice clips have it, the rest of `callsign` spelt; otherwise
// the whole callsign is. `runway` is the end's ident, "23" or "27L", at the
// airport with IATA code `airport`, in whose voice it is said. Queued behind
// an alert or another announcement - an alert that comes while one is being
// said cuts it short - and played out by soundTick().
void soundAnnounce(const String &callsign, const String &runway, bool takeoff, const char *airport);

// Read aloud, a setting on the alerts screen: an aircraft's details as they
// open, "Ryanair six six eight Tango, Boeing seven three seven max,
// climbing, four thousand four hundred feet, one seven five knots, heading
// two four zero"; a navaid's card, "Golf Oscar Whiskey, V O R D M E, one one
// five decimal four"; and an airport's, a single set of its frequencies -
// one for each use, the first of several - "Tower, one one eight decimal
// eight; Ground..." - each in the voice of where it is. The
// latest asked for is said once what is being said has been - an alert, an
// announcement, and any announcements waiting, which go stale in seconds -
// or at once, cutting it short, if that was a reading too.
// `always` says it whatever the setting, for the serial tests.
void soundSetReadAloud(bool on);
void soundReadAircraft(const Aircraft &ac, bool always = false);
void soundReadNavaid(const Navaid &navaid, bool always = false);
void soundReadAirport(const AirportInfo &airport, bool always = false);
// What it was about has closed - the card, the details: a reading of it
// stops, or doesn't start.
void soundStopReading();

// A few words in voice `accent`, as it is chosen: "Runway two three, cleared
// for take-off."
void soundSampleAccent(Accent accent);

// Plays the next note or word of what is queued once the one before has
// finished. Run every 10ms by the sound's own task, which soundInit() starts.
void soundTick();

// Silences the beeps without shutting the speaker down, so unmuting needs no
// re-initialisation. Toggled from the header and persisted.
void soundSetMuted(bool muted);
bool soundMuted();

// The least room its task has had left on its stack, bytes - for the log.
unsigned soundStackSpare();
