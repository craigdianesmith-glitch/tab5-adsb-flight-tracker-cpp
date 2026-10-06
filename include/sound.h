#pragma once

#include <Arduino.h>
#include <stdint.h>

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

// A short tick for each key pressed on the on-screen keyboard.
void soundKeyClick();

// A rising three-note chime for an interesting flight, or for an emergency
// squawk a two-tone warble that can't be mistaken for it - and then, spoken,
// what it is: "Emergency", "Watchlist", "Rare aircraft" or "Military", for
// the reason the banner names, out of the AlertReason bits in `reasons`;
// `callsign` spelt out in the phonetic alphabet, "Echo Zulu Yankee one two";
// and `typeName`, as lookupAircraftType() has it, as its maker and model:
// "Airbus... Alpha three two zero neo". Returns at once: it is played out by
// soundTick().
void soundAlert(uint8_t reasons, const String &callsign = String(), const String &typeName = String());

// On the airport zoom, a take-off or landing as a controller would clear it:
// "easyJet five three Tango Hotel, runway two three, cleared for take-off",
// or "cleared to land". The airline is said by its name where the airline
// table and the voice clips have it, the rest of `callsign` spelt; otherwise
// the whole callsign is. `runway` is the end's ident, "23" or "27L". Queued
// behind an alert or another announcement - an alert that comes while one is
// being said cuts it short - and played out by soundTick().
void soundAnnounce(const String &callsign, const String &runway, bool takeoff);

// Plays the next note of a queued alert once the one before has finished.
// Call every loop.
void soundTick();

// Silences the beeps without shutting the speaker down, so unmuting needs no
// re-initialisation. Toggled from the header and persisted.
void soundSetMuted(bool muted);
bool soundMuted();
