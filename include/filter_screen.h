#pragma once

#include "filter.h"

// The filter screen, from the funnel in the table's header: what the table
// and the radar show - a band of height and of speed, civil or military,
// airlines and types. See filter.h.
void filterScreenOpen(const TrafficFilterSet &f);
void filterScreenDraw();

enum class FilterAction { NONE, BACK };
// Every pass, touching or not, as the display page's: the sliders have to
// hear the finger lift. (x, y) are where it is, or -1 for no touch.
FilterAction filterScreenHandleTouch(int x, int y, bool pressed, bool clicked);

// The filter as the screen has it, to put in force as it is left.
TrafficFilterSet filterScreenResult();
