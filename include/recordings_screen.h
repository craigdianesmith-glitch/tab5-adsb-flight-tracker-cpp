#pragma once

#include <Arduino.h>

enum class RecordingsAction {
    NONE,
    BACK,
    PLAY,   // outPath names the recording tapped
    SHARE,  // put the videos and recordings on the network for a phone
};

// Mounts the card if it isn't already, reads the list of recordings, and
// paints the first page of it.
void recordingsScreenEnter();
void recordingsScreenDraw();

RecordingsAction recordingsScreenHandleTouch(int x, int y, String &outPath);

// Shows how far the recording last returned by PLAY has got loading, as a bar
// across its row. Matches replay::LoadProgress, to be handed to the load.
void recordingsScreenLoadProgress(uint32_t done, uint32_t total);
