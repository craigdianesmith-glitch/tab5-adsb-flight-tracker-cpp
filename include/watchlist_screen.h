#pragma once

#include <Arduino.h>

enum class WatchlistAction {
    NONE,
    CANCEL,  // Back: leave the list as it was
    SAVE,    // Save or the keyboard's OK: watchlistScreenText() has the new list
};

void watchlistScreenSet(const String &text);
void watchlistScreenDraw();
WatchlistAction watchlistScreenHandleTouch(int x, int y);
String watchlistScreenText();
