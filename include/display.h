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

// Drop the render cache - call when another screen has drawn over the shared
// canvas, so the next render repaints from scratch.
void displayInvalidate();

// Which traffic the table is showing, for the header's MIL/CIV marker. Takes
// effect on the next full repaint, which a filter change causes anyway.
void displaySetMilitary(bool military);

// Turns the shading of just-changed cells on or off.
void displaySetShowRefresh(bool enabled);

// Fades out cell shading once it has had its time on screen. Call every loop
// while the main screen is up; it does nothing until something expires.
void displayTickHighlights();

// isNew[i] true => aircraft[i]'s row is drawn in dark green for this frame.
void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const String &locationLabel,
                            const std::vector<uint8_t> &isNew);
