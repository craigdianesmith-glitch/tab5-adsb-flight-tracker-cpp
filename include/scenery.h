#pragma once

#include <stdint.h>

// The country under a 3D view besides its coasts: the towns - the land built
// over, as seen from orbit - and the rivers. From Natural Earth's 1:10m
// urban areas and rivers, which are public domain, laid out ahead of time by
// tools/gen_scenery.py.
enum class SceneryKind : uint8_t { TOWN, RIVER_SMALL, RIVER, RIVER_BIG, RIVER_HUGE };

// How wide a river of a kind is drawn, in metres; a town's outline, none.
float riverWidth(SceneryKind kind);

// The towns' outlines and the rivers within `rangeKm` of (lat, lon): each
// line's kind and its points - latitude and longitude, in degrees, in pairs
// - passed to `line` in turn. A town's outline has the town on its left; a
// line is cut where it crosses from one cell of a degree square to the next,
// unbroken across them.
void sceneryLines(double lat, double lon, float rangeKm,
                  void (*line)(void *context, SceneryKind kind, const double *latLon, int points), void *context);
