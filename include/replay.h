#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "follow.h"
#include "recorder.h"
#include "telemetry.h"

// Reads a recording back for playback. Loading indexes the file - where each
// poll starts, and every position each aircraft reported - so that any moment
// in it can be shown without reading it from the top, and so that trails can
// be drawn without holding the whole recording's aircraft in memory.
namespace replay {

// A trail is a run of points; a point with a NaN latitude starts a new one.
struct TrailPoint {
    float lat, lon;
};

// Told now and then how far load() has got through the file - every 1% or
// so - so a screen can show it. It runs while the card is held, so it must not
// touch the card itself.
using LoadProgress = void (*)(uint32_t done, uint32_t total);

bool load(const String &path, LoadProgress progress = nullptr);
void unload();
bool loaded();

const recorder::Header &header();
size_t frameCount();
uint32_t durationMs();  // from the first poll to the last

// Which poll was the latest at `t` ms into the recording.
size_t frameIndexAt(uint32_t t);
uint32_t frameMs(size_t i);
// Wall-clock UTC seconds at `t`, or 0 if the clock wasn't set when recording.
uint32_t epochAt(uint32_t t);

// What a poll was showing - see recorder.h: the radar around home, the radar
// centred on a followed aircraft, or an airport's zoom with or without one.
struct View {
    enum Kind : uint8_t { HOME, FOLLOW, ZOOM };
    Kind kind = HOME;
    char code[4] = "";  // ZOOM's airport
    char hex[8] = "";   // the aircraft followed, in either FOLLOW or ZOOM; "" for none
    // HOME's or ZOOM's range as ZOOM IN had it; 0 for the view's own - the
    // recording's radius, or the zoom's framing.
    float rangeNm = 0;
    bool sameAs(const View &o) const {
        return kind == o.kind && strcmp(code, o.code) == 0 && strcmp(hex, o.hex) == 0 && rangeNm == o.rangeNm;
    }
};

// The view of the poll latest at `t`.
View viewAt(uint32_t t);

// Where the aircraft with ICAO `hex` was last reported by `t` ms in, for
// centring on one that is missing from the poll at that point. False if it
// hadn't been reported yet.
bool lastFixOf(const String &hex, uint32_t t, float &lat, float &lon);

// Where the aircraft followed at `t` - on the radar, having left a zoom it
// was on the ground in - took off from: that zoom's airport, or "" for none.
String departedFrom(uint32_t t);

// The callsign the recording has for the aircraft with ICAO `hex`, or `hex`
// itself where it has none.
String callsignOf(const String &hex);

// An aircraft followed in the recording: its height and speed at each poll
// that reported it while it was being followed, timed in ms into the
// recording as `t` is. Empty for one never followed.
const std::vector<FollowSample> &followHeights(const String &hex);

// Where the aircraft with ICAO `hex` is shown `t` ms in, from its reports
// up to then - see RunwaySim - and `lostMs`, how long since the last of them
// with a current position. False where it has none, or none for longer than
// following would have waited for it.
bool estimateAt(const String &hex, uint32_t t, FollowEstimate &out, uint32_t &lostMs);
// The aircraft followed at `t`, as the poll latest at `t` reported it - not
// glided towards the next, so what is worked out from it changes at a poll,
// as it did live. False where nothing is followed or that poll lacks it.
bool followedAt(uint32_t t, Aircraft &out);

// Following, the range the live radar had around the aircraft at `t` - as it
// was at the poll latest at `t`, or as it was changed since - or 0 where the
// recording doesn't say, being older than that.
float followRangeAt(uint32_t t);
// And a zoom's: what ZOOM IN, OUT or auto zoom had it at, at `t`; 0 for its
// framing.
float zoomRangeAt(uint32_t t);

// The filter the table and the radar showed the traffic through at `t`, as
// the recording has it; false for none written - an older recording, whose
// replay shows everything. sceneAt() puts the radar's views through it.
bool filterAt(uint32_t t, TrafficFilterSet &out);

// The scene `t` ms into the recording: the aircraft of the latest poll at
// that point, each moved part of the way toward where the next poll found it
// so the plot glides rather than jumps between polls, and the trail each has
// left behind it over the last REPLAY_TRAIL_MS.
void sceneAt(uint32_t t, std::vector<Aircraft> &aircraft, std::vector<TrailPoint> &trails);

}  // namespace replay
