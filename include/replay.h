#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "recorder.h"

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

// The scene `t` ms into the recording: the aircraft of the latest poll at
// that point, each moved part of the way toward where the next poll found it
// so the plot glides rather than jumps between polls, and the trail each has
// left behind it over the last REPLAY_TRAIL_MS.
void sceneAt(uint32_t t, std::vector<Aircraft> &aircraft, std::vector<TrailPoint> &trails);

}  // namespace replay
