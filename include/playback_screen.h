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
    VIEW_3D,        // the aircraft followed at the moment shown, in the 3D view: playbackScreenFollowed()
};

// Loads a recording and sets it up paused at the start. False if the file
// couldn't be read. `progress` hears how the load is going, for showing it.
bool playbackScreenOpen(const String &path, replay::LoadProgress progress = nullptr);
void playbackScreenClose();

void playbackScreenSetCentre(RadarCentre centre);
// The airports, as the live radar's ALL has them.
void playbackScreenSetAirports(bool airports);
// Each aircraft's last ten minutes drawn behind it - in a video exported too -
// or not, as the display page has it.
void playbackScreenSetTracks(bool on);
void playbackScreenDraw();

// Advances playback and redraws when there is something new to show. Call
// every loop while this screen is up. True when it drew a frame.
bool playbackScreenTick();

PlaybackAction playbackScreenHandleTouch(int x, int y, Aircraft &outAircraft);

// For the 3D view of the replay, which takes this screen's place and works
// its playing from its own controls: the moment shown, ms into the
// recording; whether it is playing, and how fast; the aircraft followed at
// that moment, "" for none.
uint32_t playbackScreenTime();
bool playbackScreenPlaying();
int playbackScreenSpeed();
String playbackScreenFollowed();
// The 3D view opening: no faster than 4x, which is as fast as an aircraft can
// be followed in it, and playing on from now.
void playbackScreenEnter3d();
// Its time moved on as playing moves it, nothing drawn: the 3D view is up.
void playbackScreenAdvance();
// As the 3D view's controls work it: Play or Pause; a minute back or on; the
// next speed, no faster than 4x; a place on its bar, 0 to 1.
void playbackScreenPlayPause();
void playbackScreenStep(bool on);
void playbackScreenNextSpeed();
void playbackScreenSeekTo(float fraction);

// Renders the whole recording, at the speed chosen for playback, to an MP4 in
// /videos on the card, showing progress with a Cancel button as it goes. Takes
// over the UI until it is done - as long as the video will run for, roughly -
// and leaves the replay screen redrawn, saying how it went.
void playbackScreenExport();
