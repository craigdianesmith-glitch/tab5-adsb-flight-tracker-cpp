#pragma once

// The rest of the app on the device, for measuring - the 3D view has its own,
// bench3d.h - compiled in only with -DBENCH3D. Over serial, behind "##=",
// from the table, each printing "[bench]" lines and then "[bench] done" (see
// tools/bench2d.py):
//
//   m  the map's first draw, at Glasgow, Heathrow and Kennedy, from 5 to
//      2000nm, each onto the plot filled first - cold, as the live screen's
//   n  the names placed on it, the first time and again
//   r  the radar's plot at Glasgow, 10 to 60nm, with 0 to 120 made-up
//      aircraft: whole, and refreshed with the traffic moved
//   z  the airport zoom at Glasgow and Heathrow, the same
//   t  the table, drawn whole and with its cells changed
//   x  the whole screen pushed to the panel
//   p  a reply from the feed, from the card's overhead folder, parsed as a
//      poll parses one (main.cpp's "##u" puts one there)
//   k  the tracks taking in a poll's list, a step at a time (main.cpp)
//   s  the voices' words turned back into sound, and the engines' noise made
//   c  the instruction caches' misses, L1 and L2, round one frame of the
//      radar's plot - whole, a refresh and following - and the frame's time
//   X  the plot's column and the whole screen pushed through internal RAM by
//      screen::pushStrips() against the PPA reading the canvas, timed, and
//      the panel read back after each, set against the other's; and the
//      internal RAM the strips left
#ifdef BENCH3D
#include <vector>

#include "adsb_client.h"

void bench2dCommand(char c, void (*repaint)());

// `n` made-up aircraft within `spreadNm` of (lat, lon) - the same ones each
// time for the same n, a third of them low and coming down - moved on along
// their tracks by `movedS` seconds. Nearest first, as a poll has them; their
// hexes begin with '~', which no real one does.
std::vector<Aircraft> benchMadeUp(int n, double lat, double lon, float spreadNm, float movedS);
#endif
