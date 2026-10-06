#pragma once

#include <Arduino.h>

// Two screens of reading, both from the settings screen's title bar: how to
// use the tracker, and what it is - its version, the disclaimer, and the
// credits for the data and the voice it is built on. Laid out to the screen
// a page at a time, with Prev and Next where there is more than one.
enum class InfoPage { HELP, ABOUT };

// Opens `page` at its first page, and draws it.
void infoScreenOpen(InfoPage page);
void infoScreenDraw();

enum class InfoAction { NONE, BACK };
InfoAction infoScreenHandleTouch(int x, int y);
