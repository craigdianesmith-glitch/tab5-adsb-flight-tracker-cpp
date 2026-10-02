#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "config.h"
#include "replay.h"

enum class RadarAction {
    NONE,
    BACK,
    SELECT,           // outHex identifies the tapped contact
    TOGGLE_CENTRE,    // switch between centring on home and on the airport
    TOGGLE_RECORD,    // start or stop a recording
    OPEN_RECORDINGS,  // the list of recordings, to play one back
};

// What the record button shows. NO_CARD greys out Replays along with it:
// both need a card, and a tap on either has another go at finding one.
enum class RecButton { IDLE, RECORDING, NO_CARD };

// A plan-position plot centred on the configured location (lat, lon) or the
// airport nearest it: range rings, bearing marks, and every aircraft that
// reported a position as a blip with a vector showing where it'll be a minute
// from now.
//
// `full` repaints the whole screen, for arriving at it or changing what it is
// centred on. Without it only the plot and the footer readouts are redrawn,
// for a data refresh; that relies on the rest still being on the canvas from
// the last full draw.
void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military, RadarCentre centre, bool full);

// Contacts are identified by ICAO hex rather than by index: a poll can land
// between the plot being drawn and the screen being tapped, and an index into
// the old list would then point at the wrong aircraft.
RadarAction radarScreenHandleTouch(int x, int y, String &outHex);

// Updates the record button. It is part of the header, which a refresh
// doesn't repaint, so this repaints the button itself when `onScreen` says the
// radar is up and what it would show has changed.
void radarScreenSetRecording(RecButton state, uint32_t elapsedS, bool onScreen);

// --- the plot on its own, for playback -------------------------------------

struct RadarScene {
    const std::vector<Aircraft> &aircraft;
    const std::vector<uint8_t> &isNew;
    const std::vector<replay::TrailPoint> *trails;  // nullptr for none
    double lat, lon;  // home
    int rangeNm;
    RadarCentre centre;
    bool flash = false;  // live alerts' callsign flash - never for a replay
    // A refresh normally pushes what it drew to the panel. A video export
    // draws frames to encode, not to show, so it says when to push instead.
    bool push = true;
};

// Draws the plot, and the footer readouts at the bottom right. With `full`
// the caller has cleared the screen and flushes once it has drawn the rest;
// without it, only the plot and footer are erased and redrawn, and pushed.
void radarPlotDraw(const RadarScene &scene, bool full);

// The contact nearest a tap on the last plot drawn, if one is within reach.
bool radarPlotHit(int x, int y, String &outHex);

// The radar's palette, so another screen drawing around the plot matches it.
struct RadarPalette {
    uint16_t bg, text, muted, faint, btnBg, ring, rec, disabled;
};
const RadarPalette &radarPalette();
