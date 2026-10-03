#pragma once

#include <Arduino.h>

#include "adsb_client.h"
#include "config.h"
#include "replay.h"

enum class PlaybackAction {
    NONE,
    BACK,
    SELECT,         // outAircraft is the tapped contact, as recorded
    TOGGLE_CENTRE,  // same as the live radar's, and shared with it
    EXPORT,         // make a video of it: playbackScreenExport()
};

// Loads a recording and sets it up paused at the start. False if the file
// couldn't be read. `progress` hears how the load is going, for showing it.
bool playbackScreenOpen(const String &path, replay::LoadProgress progress = nullptr);
void playbackScreenClose();

void playbackScreenSetCentre(RadarCentre centre);
// The airports, as the live radar's ALL has them.
void playbackScreenSetAirports(bool airports);
void playbackScreenDraw();

// Advances playback and redraws when there is something new to show. Call
// every loop while this screen is up.
void playbackScreenTick();

PlaybackAction playbackScreenHandleTouch(int x, int y, Aircraft &outAircraft);

// Renders the whole recording, at the speed chosen for playback, to an MP4 in
// /videos on the card, showing progress with a Cancel button as it goes. Takes
// over the UI until it is done - as long as the video will run for, roughly -
// and leaves the replay screen redrawn, saying how it went.
void playbackScreenExport();
