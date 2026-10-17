#pragma once

#include <stdint.h>

// The map under a plot: the world's coastlines and lakes, its countries'
// borders, and the state and province lines of the big federations - the
// United States, Canada, Mexico, Brazil and Australia - where a plot can be a
// long way from any of the rest. From Natural Earth's 1:10m set, which is
// public domain, cut up ahead of time by tools/gen_coast.py.
//
// It is a map drawn for a wall, not a chart: a coast is where it should be
// to a few hundred metres, with a point every mile or so. That is finer than
// a pixel at the radar's widest ranges and good down to the closest of them,
// and no use against a runway drawn to scale - which is why the airport zoom
// is left without it.
//
// All in the one colour, and told apart by the line: a coast or a lake's
// shore solid, a border dashed, a state line dotted.
enum class OutlineKind : uint8_t { COAST, LAKE, BORDER, STATE };

// A plot to draw it on: a circle on a 16-bit canvas, and what is under it.
struct CoastPlot {
    uint16_t *pixels;  // the canvas's own, `stride` of them to a row
    int stride;
    int centerX, centerY, radius;
    double lat, lon;  // at the middle of the circle
    float rangeNm;    // at its edge
    uint16_t color;   // as the canvas stores a pixel
};

// Draws every outline that falls inside the circle, stopping just short of
// its edge, where a scope has its outer ring. Straight onto the canvas's
// pixels, so the caller marks the circle as changed. Says how many strokes
// it took.
int coastDraw(const CoastPlot &plot);

// Starts building the thinned copies of the map the widest ranges draw from,
// in the background on core 0 - see coast.cpp. Once, at start-up.
void coastBegin();

// How many of those there are yet, the full map counted: a plot drawn before
// one is built and after it can differ by a pixel here and there.
int coastLevels();

// The outlines within `rangeKm` of (lat, lon), for a view in 3D of the
// country under an aircraft: each line's kind and its points - latitude and
// longitude, in degrees, in pairs - passed to `line` in turn. A coast or a
// lake's shore has the land on its left. In full in the cells within
// `fullKm`; further off from thinned copies (see coastLevels()) - `level` 1
// or 2, whose points are a couple of kilometres apart, or four. A cell's
// lines come from one table or the other, and every table cuts its lines at
// the same cells' edges, so a coast from near to far is unbroken.
void coastLines(double lat, double lon, float rangeKm, float fullKm, int level,
                void (*line)(void *context, OutlineKind kind, const double *latLon, int points), void *context);
