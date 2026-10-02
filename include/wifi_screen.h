#pragma once

#include <Arduino.h>

enum class WifiAction {
    NONE,
    BACK,
    CONNECTED,  // a connection started from this screen has just come up
};

// Kicks off a scan and paints the network list. The notice, if given, sits on
// the status line until something connects - it says why the screen is up.
void wifiScreenEnter(const String &notice = "");
void wifiScreenDraw();

// Called every loop while this screen is up: picks up scan results and
// connection state changes, both of which arrive asynchronously.
WifiAction wifiScreenTick();

WifiAction wifiScreenHandleTouch(int x, int y);
