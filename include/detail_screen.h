#pragma once

#include "adsb_client.h"

void detailScreenSet(const Aircraft &ac);
void detailScreenDraw();

// Returns true if the back button was tapped.
bool detailScreenHandleTouch(int x, int y);
