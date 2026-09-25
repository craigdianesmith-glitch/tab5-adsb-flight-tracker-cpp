#pragma once

// Feedback beeps through the Tab5's built-in speaker.
//
// M5.begin() configures the board's ES8388 codec and amp but stops short of
// starting the I2S output, so soundInit() has to do that before anything will
// be audible.
void soundInit();

// Two-note rise, played once the firmware is up.
void soundBoot();

// Short blip when an aircraft that wasn't there before appears in the table.
void soundNewFlight();

// Silences the beeps without shutting the speaker down, so unmuting needs no
// re-initialisation. Toggled from the header and persisted.
void soundSetMuted(bool muted);
bool soundMuted();
