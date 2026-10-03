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
    TOGGLE_AIRPORTS,  // show or hide the other airports in range
    TOGGLE_RECORD,    // start or stop a recording
    OPEN_RECORDINGS,  // the list of recordings, to play one back
    ZOOM_AIRPORT,     // outAirport is the IATA code of the tapped airport
    DISMISS,          // the list of targets closed without a pick; redraw to take it off
};

// What the record button shows. NO_CARD greys out Replays along with it:
// both need a card, and a tap on either has another go at finding one.
enum class RecButton { IDLE, RECORDING, NO_CARD };

// A plan-position plot centred on the configured location (lat, lon) or the
// airport nearest it: range rings, bearing marks, and every aircraft that
// reported a position as a blip with a vector showing where it'll be a minute
// from now. With `airports`, and the plot centred on an airport, every other
// airport within the range is marked on it as well.
//
// `full` repaints the whole screen, for arriving at it or changing what it is
// centred on. Without it only the plot and the footer readouts are redrawn,
// for a data refresh; that relies on the rest still being on the canvas from
// the last full draw.
void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military, RadarCentre centre, bool airports, bool full);

// Contacts are identified by ICAO hex rather than by index: a poll can land
// between the plot being drawn and the screen being tapped, and an index into
// the old list would then point at the wrong aircraft.
//
// An airport on the plot - the centre's, or one of the others in range - is
// tapped by its code or its symbol, to zoom in on it.
//
// A tap within reach of several targets - an airport under its own ground
// traffic, a cluster of contacts - puts up a list of them beside it, and the
// next tap picks one (SELECT or ZOOM_AIRPORT) or dismisses it (DISMISS). The
// list stays up over refreshes; a full draw starts without one.
RadarAction radarScreenHandleTouch(int x, int y, String &outHex, String &outAirport);

// A long press. On the centre toggle it shows or hides the airports - a tap
// there already switches the centre - and anywhere else it does nothing.
// Whether that applies is the caller's to say: it only does while centred on
// the airport, since the airports are drawn only then.
RadarAction radarScreenHandleHold(int x, int y);

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
    bool airports;       // mark the other airports in range, when centred on one
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

// --- the airport zoom ----------------------------------------------------------

// A close-up of one airport, opened by tapping it on the radar: each of its
// runways drawn to scale from where its ends are, numbered at each end, with
// the traffic around it - which ends are in use read off what is lined up on
// them. Same scope, same contacts, at a few miles across rather than tens.

// SELECT and DISMISS as the radar's: overlapping contacts get the same list.
enum class ZoomAction { NONE, UNZOOM, SELECT, DISMISS };

// Frames the zoom on the airport with IATA `code`: on the middle of its
// runways, at a range that fits them with room around for traffic on final.
// Says where and how far, for the poll to fetch the traffic around it. False
// for a code the airport table hasn't got.
bool radarZoomOpen(const String &code, double &lat, double &lon, float &rangeNm);

// As radarScreenDraw(): `full` for arriving, without it only the plot and the
// footer readouts are redrawn.
void radarZoomDraw(const std::vector<Aircraft> &aircraft, bool full);

ZoomAction radarZoomHandleTouch(int x, int y, String &outHex);

// The radar's palette, so another screen drawing around the plot matches it.
struct RadarPalette {
    uint16_t bg, text, muted, faint, btnBg, ring, rec, disabled;
};
const RadarPalette &radarPalette();
