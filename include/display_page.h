#pragma once

#include <stdint.h>

#include "clock_zone.h"

// The display page, from the settings screen's title bar: how bright the
// screen is, whether it dims itself when nobody has touched it for a while,
// the clock on the radar and the zoom, and the zone it keeps - and whether a
// replay draws each aircraft's track behind it.
//
// The brightness is put to the screen as the slider moves; the rest is the
// caller's to apply as the page is left - see displayPageBrightness() and
// the rest below.
void displayPageSet(uint8_t brightnessPct, uint8_t dimMinutes, ClockZone clock, bool replayTracks);
void displayPageDraw();

enum class DisplayPageAction { NONE, BACK };
// Every pass, touching or not, as the settings screen's: the slider has to
// hear the finger lift. (x, y) are where it is, or -1 for no touch.
DisplayPageAction displayPageHandleTouch(int x, int y, bool pressed, bool clicked);

uint8_t displayPageBrightness();  // percent, 10 to 100
uint8_t displayPageDimMinutes();  // 0 for never
ClockZone displayPageClock();
bool displayPageReplayTracks();

// The screen at `pct` per cent of full.
void displayApplyBrightness(uint8_t pct);
