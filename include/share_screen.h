#pragma once

enum class ShareAction {
    NONE,
    BACK,
};

// Starts sharing over WiFi and shows how to reach it: a QR code a phone's
// camera can open, the address to type, and what is being sent. Sharing runs
// for as long as this screen is up - leaving it is what stops it.
void shareScreenEnter();
void shareScreenDraw();
// Keeps the status line current. Call every loop while the screen is up.
void shareScreenTick();
ShareAction shareScreenHandleTouch(int x, int y);
// Stops sharing; for whoever takes the screen away.
void shareScreenLeave();
