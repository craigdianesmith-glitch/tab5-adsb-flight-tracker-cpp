#pragma once

#include <Arduino.h>

enum class WifiAction {
    NONE,
    BACK,
};

// Kicks off a scan and paints the network list.
void wifiScreenEnter();
void wifiScreenDraw();

// Called every loop while this screen is up: picks up scan results and
// connection state changes, both of which arrive asynchronously.
void wifiScreenTick();

WifiAction wifiScreenHandleTouch(int x, int y);
