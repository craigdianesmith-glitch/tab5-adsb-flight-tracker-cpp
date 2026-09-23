#pragma once

#include <M5Unified.h>
#include <vector>

#include "adsb_client.h"

void displayInit();

// Drop the render cache - call when another screen has drawn over the shared
// canvas, so the next render repaints from scratch.
void displayInvalidate();

// isNew[i] true => aircraft[i]'s row is drawn in dark green for this frame.
void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const String &locationLabel,
                            const std::vector<uint8_t> &isNew);
