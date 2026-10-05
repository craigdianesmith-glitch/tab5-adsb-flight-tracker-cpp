#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "config.h"
#include "replay.h"
#include "telemetry.h"

enum class RadarAction {
    NONE,
    BACK,
    SELECT,           // outHex identifies the tapped contact
    // HOME, AIRPORT and ALL: one of the three views, only ever one lit.
    CENTRE_HOME,      // centred on home
    CENTRE_AIRPORT,   // centred on the airport nearest home
    CENTRE_ALL,       // the same, with every other airport in range marked
    TOGGLE_FOLLOW,    // pick an aircraft to follow, or stop following one
    TOGGLE_RECORD,    // start or stop a recording
    OPEN_RECORDINGS,  // the list of recordings, to play one back
    ZOOM_IN,          // show a shorter range
    ZOOM_OUT,         // show a longer one
    ZOOM_AIRPORT,     // outAirport is the IATA code of the tapped airport
    DISMISS,          // the list of targets closed without a pick; redraw to take it off
};

// What the record button shows. NO_CARD greys out Replays along with it:
// both need a card, and a tap on either has another go at finding one.
enum class RecButton { IDLE, RECORDING, NO_CARD };

// Follow me: OFF, PICKING once FOLLOW has been tapped and the next contact
// tapped is the one to follow, ON while following it.
enum class FollowButton { OFF, PICKING, ON };

// What the buttons down the right-hand edge show, besides recording: which
// view is chosen - HOME, AIRPORT, or ALL for the airport centre with
// `airports` on - and following. While following, none of the three is lit -
// the plot is centred on the aircraft, with the airports marked whichever was -
// and `followHex` and `followCallsign` say which it is, for the plot to mark
// it and the title to name it. Repaints what changed when `onScreen`.
void radarScreenSetControls(RadarCentre centre, bool airports, FollowButton follow, const String &followHex,
                            const String &followCallsign, const String &followOrigin, bool onScreen);

// Whether ZOOM IN and ZOOM OUT have a range to go to - neither has while
// following, which sets its own. Repaints them if they changed and `onScreen`.
void radarScreenSetZoom(bool canIn, bool canOut, bool onScreen);

// A plan-position plot centred on (lat, lon) - the configured location, the
// airport nearest it, or the aircraft being followed, as the controls say:
// range rings, bearing marks, and every aircraft that reported a position as
// a blip with a vector showing where it'll be a minute from now. With the
// airports on and centred on the airport, or following, every other airport
// within the range is marked on it as well.
//
// `full` repaints the whole screen, for arriving at it. Without it only the
// plot and the footer readouts are redrawn, for a data refresh or a change of
// centre; that relies on the rest still being on the canvas from the last
// full draw.
void radarScreenDraw(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew, double lat, double lon,
                     int rangeNm, bool military, bool full);

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

// Updates the record button. It is part of the header, which a refresh
// doesn't repaint, so this repaints the button itself when `onScreen` says the
// radar - or with `zoom`, the airport zoom, which has REC without Replays - is
// up and what it would show has changed.
void radarScreenSetRecording(RecButton state, uint32_t elapsedS, bool onScreen, bool zoom);

// --- the plot on its own, for playback -------------------------------------

struct RadarScene {
    const std::vector<Aircraft> &aircraft;
    const std::vector<uint8_t> &isNew;
    const std::vector<replay::TrailPoint> *trails;  // nullptr for none
    double lat, lon;  // home
    int rangeNm;
    RadarCentre centre;
    bool airports;       // mark the other airports in range, centred on one - following, they always are
    bool flash = false;  // live alerts' callsign flash - never for a replay
    // A refresh normally pushes what it drew to the panel. A video export
    // draws frames to encode, not to show, so it says when to push instead.
    bool push = true;
    // The aircraft being followed, by ICAO hex: ringed, and the plot is
    // centred on it - (lat, lon) is where it is - rather than marked with a
    // cross. Empty for none.
    String followHex;
    // Following a departure, the airport it took off from: the airports are
    // drawn around it, that one brighter. Empty for none.
    String originCode;
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
// TOGGLE_FOLLOW and TOGGLE_RECORD are the FOLLOW and REC buttons, as the
// radar's and where the radar has them.
// REDRAW: ZOOM IN or OUT changed the range shown, for a full redraw. ZOOM OUT
// from the widest is UNZOOM.
enum class ZoomAction { NONE, UNZOOM, TOGGLE_FOLLOW, TOGGLE_RECORD, SELECT, DISMISS, REDRAW };

// Where the zoom on the airport with IATA `code` would be framed: on the
// middle of its runways, at a range that fits them with room around for
// traffic on final. False for a code the airport table hasn't got.
bool radarZoomFrame(const String &code, double &lat, double &lon, float &rangeNm);

// Frames the zoom on that airport, as radarZoomFrame() says. Says where and
// how far, for the poll to fetch the traffic around it.
bool radarZoomOpen(const String &code, double &lat, double &lon, float &rangeNm);

// The aircraft being followed in the zoom - ringed, named in the title, and
// FOLLOW lit to stop - or empty strings for none; and with `picking`, FOLLOW
// tapped and waiting for a contact to be. Takes effect on the next full draw.
void radarZoomSetFollow(const String &hex, const String &callsign, bool picking);

// As radarScreenDraw(): `full` for arriving, without it only the plot and the
// footer readouts are redrawn.
void radarZoomDraw(const std::vector<Aircraft> &aircraft, bool full);

// Whether the zoom's last draw was centred on the aircraft followed rather
// than on its runways - zoomed in past its framing while following - so that
// it moves with it, to be redrawn as often as a lost one is.
bool radarZoomOnFollowed();

// The zoom's plot on its own, for a replay of a followed aircraft: the
// airport with IATA `code` as the live zoom frames it, and the scene's
// contacts, trails and followed aircraft on it - its centre, range and
// airports go unused. Drawn and pushed as radarPlotDraw() is. False for a
// code the airport table hasn't got, with nothing drawn.
bool radarZoomPlotDraw(const RadarScene &scene, const String &code, bool full);

ZoomAction radarZoomHandleTouch(int x, int y, String &outHex);

// --- the followed aircraft's height ---------------------------------------------

// What a take-off or landing looks like from the side: the followed
// aircraft's altitude, vertical speed and groundspeed as it last reported
// them, over a chart of its height across the last TELEMETRY_WINDOW_MS with
// each touchdown and take-off marked. `fieldFt` is the elevation of the
// airport being zoomed on, drawn as a line across the chart with the height
// above it among the readouts, or INT32_MIN for none. Samples after `nowMs`
// are left out, for a replay partway through. Drawn into the TELEMETRY_W by
// TELEMETRY_H box at (x, y) and marked for the caller to push. `lostS`, when
// not 0, is how long the feed has had no position for it - the plot showing
// where it is estimated to be - and is said beside the readouts, which stay
// as it last reported them.
constexpr int TELEMETRY_W = 274, TELEMETRY_H = 330;
constexpr uint32_t TELEMETRY_WINDOW_MS = 10UL * 60 * 1000;
void radarTelemetryDraw(int x, int y, const std::vector<FollowSample> &samples, uint32_t nowMs, int32_t fieldFt,
                        uint32_t lostS);

// The followed aircraft's samples, for the live radar and zoom to draw beside
// the plot while following, and how long it has been lost, if it has.
void radarSetHeights(const std::vector<FollowSample> &samples, uint32_t lostS);

// The radar's palette, so another screen drawing around the plot matches it.
struct RadarPalette {
    uint16_t bg, text, muted, faint, btnBg, ring, rec, disabled;
};
const RadarPalette &radarPalette();

// Where the plots' drawing time has gone since last asked, for the [perf]
// line - the parts that don't move, the contacts, the push - or "-" for no
// plots drawn.
String radarTakeTimings();
