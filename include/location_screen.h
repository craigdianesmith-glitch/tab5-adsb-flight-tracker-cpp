#pragma once

#include <Arduino.h>

enum class LocationAction { NONE, BACK, LOCATION_SET };

void locationScreenReset();
void locationScreenDraw();

// Call with a touch point while this screen is active. On LOCATION_SET,
// outLat/outLon/outLabel are filled with the newly chosen location.
LocationAction locationScreenHandleTouch(int x, int y, double &outLat, double &outLon, String &outLabel);
