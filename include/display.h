#pragma once

#include <M5Unified.h>
#include <vector>

#include "adsb_client.h"

void displayInit();

// How many aircraft rows actually fit on screen. The poll task trims its
// results to this, so the rows that could never be drawn aren't carried
// around and copied every refresh.
int displayMaxRows();

// True if (x, y) is on the settings cog in the header.
bool displayHitCog(int x, int y);

// True if (x, y) is on the radar button in the header.
bool displayHitRadar(int x, int y);

// True if (x, y) is on the mute speaker in the header.
bool displayHitMute(int x, int y);

// True if (x, y) lands on a row the table actually drew, with outRow set to
// its index. Bounded by the rows on screen rather than by the size of the
// result set: the poll keeps up to MAX_CONTACTS aircraft for the radar to
// plot, so without the bound the gap below the last drawn row resolves to an
// aircraft that was never on screen.
bool displayHitRow(int x, int y, int &outRow);

// Drop the render cache - call when another screen has drawn over the shared
// canvas, so the next render repaints from scratch.
void displayInvalidate();

// The header's subtitle: which traffic is being shown, and the nearest
// airport to the configured location (empty if nothing is near enough).
// Takes effect on the next full repaint, which any of these changing causes.
void displaySetHeader(bool military, const String &airportCode);

// Whether the speaker icon is drawn muted. Repaints the icon on its own.
void displaySetMuted(bool muted);

// Turns the shading of just-changed cells on or off.
void displaySetShowRefresh(bool enabled);

// Fades out cell shading once it has had its time on screen. Call every loop
// while the main screen is up; it does nothing until something expires.
void displayTickHighlights();

// The state of the polling, for the status line under the table. A
// lastSuccessMs of 0 (everSucceeded false) means nothing has arrived yet.
void displaySetPollState(bool linkUp, bool lastPollOk, bool everSucceeded, uint32_t lastSuccessMs);

// Redraws the status line when what it would say has changed - which includes
// the data ageing by a second. Call every loop while the main screen is up.
void displayTickStatus();

// isNew[i] true => aircraft[i]'s row is drawn in dark green for this frame.
void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew);
