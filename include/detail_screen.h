#pragma once

#include "adsb_client.h"

// What FOLLOW beside Back offers for the aircraft shown: to follow it, to
// stop following it - it is the one followed already - or nothing, for one
// out of a replay, or with no position to follow it from.
enum class DetailFollow { NONE, FOLLOW, UNFOLLOW };

void detailScreenSet(const Aircraft &ac, DetailFollow follow);
void detailScreenDraw();

enum class DetailAction { NONE, BACK, FOLLOW, UNFOLLOW };
DetailAction detailScreenHandleTouch(int x, int y);
