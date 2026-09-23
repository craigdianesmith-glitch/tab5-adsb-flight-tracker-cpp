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

// isNew[i] true => aircraft[i]'s row is drawn in dark green for this frame.
void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const String &locationLabel,
                            const std::vector<uint8_t> &isNew);
