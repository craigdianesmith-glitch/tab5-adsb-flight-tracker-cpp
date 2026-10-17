#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "clock_zone.h"
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
    DRAG,             // landed on the plot with nothing under it: see radarDragTo()
    OPEN_3D,          // following: the 3D view of it
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
                     int rangeNm, bool full);

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

// Updates the record button, and Replays under it, which is greyed out with
// it. They are part of the header, which a refresh doesn't repaint, so this
// repaints them itself when `onScreen` says the radar or the airport zoom -
// which has the two where the radar does - is up and what they would show has
// changed.
void radarScreenSetRecording(RecButton state, uint32_t elapsedS, bool onScreen);

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
    // For radarZoomPlotDraw(), the range the zoom was showing, as recorded;
    // 0 for its framing. And the moment shown, in ms on any clock that runs
    // forward through a replay, for the runways lately in use.
    float zoomRangeNm = 0;
    uint32_t nowMs = 0;
    // Dragged off its own centre - see radarDragTo(): (lat, lon) is where it
    // was let go, and with `markHome` home is crossed where it is, at
    // (homeLat, homeLon), having been what the plot was dragged off.
    bool freeLook = false;
    bool markHome = false;
    double homeLat = 0, homeLon = 0;
    // Every airport in range marked, whichever view it is: dragged off its
    // centre, or being dragged, they are what a drag is aimed by.
    bool everyAirport = false;
    // Where nothing is drawn over the plot afterwards and its centre stays
    // put - the live radar left to itself - a refresh may put back only what
    // was under the traffic and push only where it moved: see radarPlotDraw().
    bool partial = false;
    // Following with auto zoom choosing the range, which the range readout
    // says - never in a replay, which frames it as it always has.
    bool autoRange = false;
    // A drag's line has run off the edge of the plot, which is drawn at a
    // range that takes it: the map and the airports alone, the traffic left
    // off until the plot is back at its own range, wherever it is let go. Or
    // the radar is out on the map's own ranges - see radarScreenSetMapOnly(),
    // which leaves off the airports too.
    bool mapOnly = false;
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
// from the widest is UNZOOM, back to the radar; BACK is the Back button, to
// the table. OPEN_RECORDINGS is Replays, where the radar has it. DRAG is the
// radar's: a touch on the plot that may go on to drag it.
// RANGE: ZOOM IN or OUT changed the range shown - drawn again, as REDRAW.
enum class ZoomAction {
    NONE, UNZOOM, BACK, TOGGLE_FOLLOW, TOGGLE_RECORD, OPEN_RECORDINGS, SELECT, DISMISS, REDRAW, RANGE, DRAG, OPEN_3D
};

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

// The range the zoom shows, as ZOOM IN and OUT have it, for a recording to
// say - or 0 at its framing.
float radarZoomViewNm();
// ZOOM IN and OUT drawn again as the range now allows - after it has been
// changed other than by them, auto zoom's doing - and marked to go with the
// next push. Only with the zoom on screen.
void radarZoomRefreshButtons();

// Puts the zoom just opened back to a range radarZoomViewNm() gave, to come
// back to it as it was left - 0 for its framing.
void radarZoomSetViewNm(float nm);

// The zoom's plot on its own, for a replay of a followed aircraft: the
// airport with IATA `code` as the live zoom frames it, and the scene's
// contacts, trails and followed aircraft on it - its centre, range and
// airports go unused. Drawn and pushed as radarPlotDraw() is. False for a
// code the airport table hasn't got, with nothing drawn.
bool radarZoomPlotDraw(const RadarScene &scene, const String &code, bool full);

// Whether the radar marks the navaids in range - VORs, DMEs and TACANs, as
// the settings screen has it - live and in replays. A navaid's ident or
// symbol is tapped for its card: its name, type, frequency and DME channel.
void radarScreenSetNavaids(bool on);

// --- dragging the plot -----------------------------------------------------------
//
// Free look. A touch that lands on the radar's plot, or the zoom's, with
// nothing under it to tap is one that may drag: the screen's touch handler
// says DRAG, and for as long as the finger stays down each place it moves to
// is given to radarDragTo(). Once it has gone a little way a line stretches
// out from the middle of the plot, the way it has been dragged and as far,
// with how far that is in nautical miles and on what bearing - over the plot
// as it stands, the map under it and every airport in range, marked for as
// long as the line is out, being what the line is read against. A line
// dragged past the edge of the plot - a swipe across the screen is twice its
// range - has the plot drawn again at a range that takes it: its map and its
// airports only, the traffic left off until it is let go. Where it is let
// go, radarDragEnd(), is the plot's new centre, at the range it had, until
// the radar's HOME, AIRPORT or ALL, or the zoom's CENTRE, puts it back.
//
// The radar goes anywhere, a swipe at a time, and the traffic around where it
// has got to is the caller's to fetch - radarScreenFreeLook() says where
// that is. The zoom is of its airport: its middle goes no further from the
// middle of its runways than its framing reaches. Neither drags while
// following, which has a centre of its own.
//
// Both say whether there is something to redraw: the line having moved, or
// the plot.
bool radarDragTo(int x, int y);
bool radarDragEnd();

// Whether the radar has been dragged off its own centre, and where to.
bool radarScreenFreeLook(double &lat, double &lon);

// The radar put on a free look at (lat, lon), as if dragged there from home
// at (homeLat, homeLon) - the centre a drag would have been measured from.
void radarScreenLookAt(double lat, double lon, double homeLat, double homeLon);

// Puts the radar back on its own centre, as HOME, AIRPORT and ALL do. True if
// it had been dragged off it.
bool radarScreenEndFreeLook();

// Zoomed out past the traffic's ranges: the plot is the map alone - its
// outlines and names, no traffic, navaids or airports - and the title says
// so. For the plot drawn next, which ought to be a full one when this has
// changed, for the title.
void radarScreenSetMapOnly(bool on);
// The title and what it says under it - map only, free look, filtered - drawn
// again and pushed, for a refresh of the plot that changes what it says
// without the whole screen being drawn again.
void radarScreenRedrawTitle();

// Following, whether auto zoom is choosing the range: see RadarScene.
void radarScreenSetAutoRange(bool on);

// The filter's funnel, `r` across from its middle at (cx, cy), lit for a
// filter on - as the radar's title has it, for a replay's to say the same.
void radarDrawFunnel(int cx, int cy, int r, bool lit);

// The fastest of the contacts the last plot drew - the radar's or the zoom's,
// whichever was drawn last - in knots: how soon a redraw would move one of
// them a pixel. Those left off by the filter or beyond the outer ring don't
// count, nor do those standing where the feed last had them.
float radarFastestKt();
// Whether a filter is on - see filter.h - for the title to say so and its
// funnel to be lit.
void radarScreenSetFiltered(bool on);
// True if (x, y) is on that funnel, under the radar's speaker.
bool radarScreenHitFilter(int x, int y);

// Whether the radar has the map under it - coasts and lakes, countries'
// borders, and the state lines of the big federations, as the settings screen
// has it - live and in replays. The airport zoom never does: see coast.h.
void radarScreenSetCoast(bool on);

// Whether the radar has its range rings - the ones inside the outer ring,
// with their distances. Without them there is the outer ring, with its range
// and the compass on it, and the map under it drawn a little brighter, having
// the plot to itself. The airport zoom keeps its rings either way: it has no
// map, and they are all that says how far out the traffic on final is.
void radarScreenSetRings(bool on);

// The sound's switch at the end of the radar's and the zoom's titles, as the
// table's header has it: what it shows, redrawn with `onScreen`; and whether
// a tap is on it.
void radarScreenSetMuted(bool muted, bool onScreen);
bool radarScreenHitMute(int x, int y);

// The UTC clock in the radar's and the zoom's margin - a setting - and its
// minute turning over, with one of them up: called every pass.
void radarScreenSetClock(ClockZone zone);
void radarScreenTickClock();
// Following, the time where the aircraft is - at (lat, lon) - instead, by a
// blip: its zone's, anywhere in the world, changing as it crosses from one
// zone into the next. Unless the clock is set to UTC, the same everywhere,
// or is off. Called every pass; `on` false for home's time again.
void radarScreenSetClockAt(bool on, double lat, double lon);

// The runway ends the live zoom has lately seen in use, and how long ago,
// for a recording's file to begin with - see recorder::RunwaysInUse.
void radarLiveRunwaysInUse(recorder::RunwaysInUse &out);
// What a replay's zoom starts out remembering of the runways in use: its
// recording's, as the live zoom had them when it began.
void radarReplayRunwaysInUse(const recorder::RunwaysInUse &seed);

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
