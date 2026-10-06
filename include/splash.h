#pragma once

#include <Arduino.h>

// The title screen at boot: a runway running off to a dusk horizon, OVERHEAD
// across the sky, and an airliner taking off down the runway at the viewer,
// growing until it roars out over the top of the screen - with soundTakeoff()
// started alongside.

// Plays it through: about three seconds, blocking. WiFi can be connecting
// meanwhile - it is begun before and waited on after.
void splashPlay();

// A line along the bottom of the finished splash, for what boot is waiting
// on: "Connecting to ..." - drawn over the last frame.
void splashStatus(const String &text);
