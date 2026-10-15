#pragma once

#include "adsb_client.h"

// What FOLLOW beside Back offers for the aircraft shown: to follow it, to
// stop following it - it is the one followed already - or nothing, for one
// out of a replay, or with no position to follow it from.
enum class DetailFollow { NONE, FOLLOW, UNFOLLOW };
// And WATCH beside it: to put its callsign on the watchlist, or take it off -
// as a long press on its table row does - or nothing, for one out of a
// replay, or with no callsign to watch for.
enum class DetailWatch { NONE, WATCH, UNWATCH };

void detailScreenSet(const Aircraft &ac, DetailFollow follow, DetailWatch watch);
void detailScreenDraw();

// WATCH: WATCH or UNWATCH was tapped - the watchlist decides which.
// OPEN_FROM and OPEN_TO: one of the route's airports was tapped, for its
// zoom - detailScreenRouteCode() says which airport.
enum class DetailAction { NONE, BACK, FOLLOW, UNFOLLOW, WATCH, SAY, OPEN_FROM, OPEN_TO };
DetailAction detailScreenHandleTouch(int x, int y);
String detailScreenRouteCode(bool from);
