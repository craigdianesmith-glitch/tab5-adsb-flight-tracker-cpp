#include "scenery.h"

#include <math.h>
#include <stddef.h>

#include <algorithm>
#include <vector>

namespace {

// Laid out as the coast table is - see tools/gen_scenery.py: in cells a
// degree square, each line a start and steps of a byte each way on a grid of
// SCENERY_UNIT to the degree. About 1.7MB of flash.
#include "scenery_table.inc"
constexpr size_t SCENERY_CELL_COUNT = sizeof(SCENERY_CELLS) / sizeof(SCENERY_CELLS[0]);

// Above a line's start x: what it is, and whether it goes on from the line
// before it - one with more steps than a byte counts.
constexpr int START_BITS = 12, KIND_MASK = 7, CONTINUES = 8;

}  // namespace

float riverWidth(SceneryKind kind) {
    switch (kind) {
    case SceneryKind::RIVER_SMALL: return 25;
    case SceneryKind::RIVER: return 60;
    case SceneryKind::RIVER_BIG: return 150;
    case SceneryKind::RIVER_HUGE: return 400;
    default: return 0;
    }
}

void sceneryLines(double lat, double lon, float rangeKm,
                  void (*line)(void *context, SceneryKind kind, const double *latLon, int points), void *context) {
    double dLat = rangeKm / 111.0, dLon = rangeKm / (111.0 * std::max(0.05, cos(lat * M_PI / 180)));
    int south = std::max(0, (int)floor(lat - dLat + 90)), north = std::min(179, (int)floor(lat + dLat + 90));
    int west = (int)floor(lon - dLon + 180), east = (int)floor(lon + dLon + 180);
    std::vector<double> points;
    const uint16_t *cellsEnd = SCENERY_CELLS + SCENERY_CELL_COUNT;
    for (int row = south; row <= north; row++) {
        // A row's cells west to east, cut at the date line.
        for (int k = (int)floor(west / 360.0); k <= (int)floor(east / 360.0); k++) {
            int first = std::max(west, k * 360), last = std::min(east, k * 360 + 359);
            uint16_t lo = (uint16_t)(row * 360 + (first - k * 360)), hi = (uint16_t)(row * 360 + (last - k * 360));
            for (const uint16_t *cell = std::lower_bound(SCENERY_CELLS, cellsEnd, lo); cell < cellsEnd && *cell <= hi;
                 cell++) {
                double cellLat = row - 90.0, cellLon = *cell - row * 360 - 180.0 + k * 360.0;
                size_t index = (size_t)(cell - SCENERY_CELLS);
                const uint8_t *p = SCENERY_LINES + SCENERY_STARTS[index], *end = SCENERY_LINES + SCENERY_STARTS[index + 1];
                int kind = 0, x = 0, y = 0;
                points.clear();
                auto put = [&]() {
                    points.push_back(cellLat + y / (double)SCENERY_UNIT);
                    points.push_back(cellLon + x / (double)SCENERY_UNIT);
                };
                auto done = [&]() {
                    if (points.size() >= 4) {
                        line(context, (SceneryKind)kind, points.data(), (int)points.size() / 2);
                    }
                    points.clear();
                };
                while (p + 5 <= end) {
                    int head = p[0] | p[1] << 8, flags = head >> START_BITS, steps = p[4];
                    if (!(flags & CONTINUES)) {
                        done();
                        kind = flags & KIND_MASK;
                        x = head & ((1 << START_BITS) - 1);
                        y = p[2] | p[3] << 8;
                        put();
                    }
                    p += 5;
                    steps = std::min(steps, (int)(end - p) / 2);
                    for (int i = 0; i < steps; i++, p += 2) {
                        x += (int8_t)p[0];
                        y += (int8_t)p[1];
                        put();
                    }
                }
                done();
            }
        }
    }
}
