#include "coast.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include <algorithm>
#include <vector>

#ifdef ESP_PLATFORM
#include "psram_alloc.h"
#endif

namespace {

// Natural Earth's lines, made by tools/gen_coast.py, which says how they are
// laid out: in cells a degree square, each line a start and then steps of a
// byte each way on a grid of COAST_UNIT to the degree. About 2MB of flash.
#include "coast_table.inc"
constexpr size_t COAST_CELL_COUNT = sizeof(COAST_CELLS) / sizeof(COAST_CELLS[0]);

// Above a line's start x: what it is, and whether it goes on from the line
// before it - one with more steps than a byte counts.
constexpr int START_BITS = 12, KIND_MASK = 3, CONTINUES = 4;

// A coast stops this far inside the circle, clear of the ring drawn on it.
constexpr int RING_CLEAR = 2;

// Natural Earth has a point every mile or so, which close in is a coast of
// straight edges a hundred pixels long. So a coast or a lake is drawn as a
// curve through its points instead: a centripetal Catmull-Rom spline, the kind
// that never loops or overshoots however unevenly the points are spaced. It
// makes the line smooth, not more accurate - the points are still all that is
// known. An edge shorter than this is left straight, a curve that small being
// no different, which at the wide ranges is nearly all of them.
constexpr float CURVE_MIN_PX = 8;
// And the most straight strokes one edge's curve is drawn in.
constexpr int CURVE_MAX_PARTS = 16;

// A point of a line, on the canvas - and, once a curve has asked for it, the
// square root of how far it is to the next point, which is what spaces a
// centripetal spline.
struct Point {
    float x, y;
    float root;
};

// How a line is drawn: a coast or a shore solid, a border dashed and a state
// line dotted, as an atlas has them, so neither is taken for a coast. A
// dash's length, and a dash and its gap together, in pixels.
enum Style : uint8_t { SOLID, DASHED, DOTTED };
constexpr float DASH_ON[] = {0, 7, 2}, DASH_PERIOD[] = {0, 11, 6};

// The plot's map is kept from one draw of it to the next, as the pixels it
// came to: where on the canvas each one is. A radar that isn't following
// anything draws the same map every time - each poll, and twice between
// polls - and finding it is most of the work: reading the cells, placing
// every point, bending the coasts, cutting it all off at the ring and
// stepping along each stroke takes 12ms at 150nm round Glasgow, where
// putting those pixels back is a loop that does nothing else, and takes 4.
// In PSRAM on the device, four bytes a pixel: a couple of thousand of them at
// 20nm, fourteen thousand in that plot of Glasgow's, thirty over the lakes
// of Finland.
//
// What is left is the canvas, not the code: it is in PSRAM, behind a cache
// far smaller than it, and each pixel put on it is a line of that cache
// fetched first - a quarter of a microsecond a pixel, with reading the list
// under a millisecond of the four. Tried, on the device, and no better:
// keeping the strokes rather than their pixels, to step along again (6ms);
// the pixels sorted into the order the canvas is in memory (4ms still, and
// 3ms to sort); and having the PPA copy a whole plot kept ready-made, which
// takes it 23ms - longer than drawing everything on the plot afresh.
#ifdef ESP_PLATFORM
template <typename T>
using Buffer = std::vector<T, PsramAllocator<T>>;
#else
template <typename T>
using Buffer = std::vector<T>;  // on the PC, for tools/coast_preview
#endif
struct Kept {
    bool valid = false;
    int stride = 0, centerX = 0, centerY = 0, radius = 0;
    double lat = 0, lon = 0;
    float rangeNm = 0;
    int strokes = 0;
    Buffer<uint32_t> pixels;
};
Kept g_kept;

struct Pen {
    uint16_t *pixels;
    int stride;
    int centerX, centerY, radius2;   // the circle to stay inside
    float left, top, right, bottom;  // and the square round it
    uint16_t color;
    Style style;
    float travelled;  // how far through its dashes the line has got
};

// A straight stroke from one point to another, as much of it as is inside
// the circle: onto the canvas, and its pixels kept.
void stroke(Pen &pen, float ax, float ay, float bx, float by) {
    float dx = bx - ax, dy = by - ay;
    float len = 0, from = pen.travelled, period = DASH_PERIOD[pen.style];
    bool dashed = pen.style != SOLID;
    if (dashed) {
        // Off the plot or on it, so the dashes stay put on the ground as the
        // map moves under a followed aircraft.
        len = sqrtf(dx * dx + dy * dy);
        float to = from + len;
        pen.travelled = to - period * floorf(to / period);
    }
    // Off one side of the square altogether, as most of a cell is.
    if ((ax < pen.left && bx < pen.left) || (ax > pen.right && bx > pen.right) || (ay < pen.top && by < pen.top) ||
        (ay > pen.bottom && by > pen.bottom)) {
        return;
    }
    // Liang-Barsky, for one that crosses the square's edge: the part inside.
    float t0 = 0, t1 = 1;
    bool inside = ax >= pen.left && ax <= pen.right && ay >= pen.top && ay <= pen.bottom && bx >= pen.left &&
                  bx <= pen.right && by >= pen.top && by <= pen.bottom;
    if (!inside) {
        const float p[4] = {-dx, dx, -dy, dy};
        const float q[4] = {ax - pen.left, pen.right - ax, ay - pen.top, pen.bottom - ay};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                continue;  // along this side, and not beyond it or it had gone already
            }
            float t = q[i] / p[i];
            if (p[i] < 0) {
                t0 = std::max(t0, t);
            } else {
                t1 = std::min(t1, t);
            }
        }
        if (t0 > t1) {
            return;
        }
    }
    // Rounded by adding a half and dropping the fraction, which is right for
    // anything inside the square - none of it is negative - and is one
    // instruction where lroundf() is a call, four times a stroke.
    int x = (int)(ax + dx * t0 + 0.5f), y = (int)(ay + dy * t0 + 0.5f);
    int x1 = (int)(ax + dx * t1 + 0.5f), y1 = (int)(ay + dy * t1 + 0.5f);
    int stepX = x1 >= x ? 1 : -1, stepY = y1 >= y ? 1 : -1;
    int acrossX = abs(x1 - x), acrossY = abs(y1 - y);
    bool steep = acrossY > acrossX;
    int major = steep ? acrossY : acrossX, minor = steep ? acrossX : acrossY;

    // Only one with an end outside the circle has to ask of every pixel.
    auto inCircle = [&pen](int px, int py) {
        int ox = px - pen.centerX, oy = py - pen.centerY;
        return ox * ox + oy * oy <= pen.radius2;
    };
    bool check = !inCircle(x, y) || !inCircle(x1, y1);
    float dash = 0, dashStep = 0;
    if (dashed) {
        dash = from + len * t0;
        dash -= period * floorf(dash / period);
        dashStep = major ? len * (t1 - t0) / major : 0;
    }

    uint32_t at = (uint32_t)y * pen.stride + x;
    int alongStep = steep ? stepY * pen.stride : stepX, sideStep = steep ? stepX : stepY * pen.stride;
    int error = major / 2;
    for (int i = 0; i <= major; i++) {
        if ((!check || inCircle(x, y)) && (!dashed || dash < DASH_ON[pen.style])) {
            pen.pixels[at] = pen.color;
            g_kept.pixels.push_back(at);
        }
        if (dashed) {
            dash += dashStep;
            if (dash >= period) {
                dash -= period;
            }
        }
        error -= minor;
        if (error < 0) {
            error += major;
            at += sideStep;
            (steep ? x : y) += steep ? stepX : stepY;
        }
        at += alongStep;
        (steep ? y : x) += steep ? stepY : stepX;
    }
    g_kept.strokes++;
}

// The square root of the distance from one point to the next, worked out the
// once for the two edges that want it.
float rootTo(Point &a, const Point &b) {
    if (a.root < 0) {
        float dx = b.x - a.x, dy = b.y - a.y;
        a.root = sqrtf(sqrtf(dx * dx + dy * dy));
    }
    return a.root;
}

// A coast or a lake's shore: the curve through its points. One that ends
// where it began - an island, or a lake, all in the one cell - is a ring, and
// curves through that point as through the rest.
void curve(Pen &pen, Point *p, int count, bool ring) {
    for (int i = 0; i + 1 < count; i++) {
        Point &p1 = p[i], &p2 = p[i + 1];
        float dx = p2.x - p1.x, dy = p2.y - p1.y;
        // A curve bows out from its edge by less than half the edge's length.
        float reach = (fabsf(dx) + fabsf(dy)) * 0.5f;
        if (std::max(p1.x, p2.x) + reach < pen.left || std::min(p1.x, p2.x) - reach > pen.right ||
            std::max(p1.y, p2.y) + reach < pen.top || std::min(p1.y, p2.y) - reach > pen.bottom) {
            continue;
        }
        Point *p0 = i > 0 ? &p[i - 1] : ring ? &p[count - 2] : nullptr;
        Point *p3 = i + 2 < count ? &p[i + 2] : ring ? &p[1] : nullptr;
        if (dx * dx + dy * dy < CURVE_MIN_PX * CURVE_MIN_PX || (!p0 && !p3)) {
            stroke(pen, p1.x, p1.y, p2.x, p2.y);
            continue;
        }
        // The way the curve is heading as it leaves p1 and as it arrives at
        // p2 - or straight along the edge, at an end with nothing beyond it.
        float m1x = dx, m1y = dy, m2x = dx, m2y = dy;
        float t12 = rootTo(p1, p2);
        float t01 = p0 ? rootTo(*p0, p1) : 0;
        if (t01 > 0) {
            float k = t12 / t01, j = t12 / (t01 + t12);
            m1x += k * (p1.x - p0->x) - j * (p2.x - p0->x);
            m1y += k * (p1.y - p0->y) - j * (p2.y - p0->y);
        }
        float t23 = p3 ? rootTo(p2, *p3) : 0;
        if (t23 > 0) {
            float k = t12 / t23, j = t12 / (t12 + t23);
            m2x += k * (p3->x - p2.x) - j * (p3->x - p1.x);
            m2y += k * (p3->y - p2.y) - j * (p3->y - p1.y);
        }
        // In as many strokes as keep them within a third of a pixel of it,
        // which is how far it bends over how much.
        float bend = fabsf(m1x - dx) + fabsf(m1y - dy) + fabsf(m2x - dx) + fabsf(m2y - dy);
        int parts = bend <= 2 ? 1 : std::min(CURVE_MAX_PARTS, (int)sqrtf(bend * 0.5f) + 1);
        float ax = m1x + m2x - 2 * dx, ay = m1y + m2y - 2 * dy;
        float bx = 3 * dx - 2 * m1x - m2x, by = 3 * dy - 2 * m1y - m2y;
        float fromX = p1.x, fromY = p1.y;
        for (int k = 1; k <= parts; k++) {
            float t = (float)k / parts;
            float x = k == parts ? p2.x : p1.x + t * (m1x + t * (bx + t * ax));
            float y = k == parts ? p2.y : p1.y + t * (m1y + t * (by + t * ay));
            stroke(pen, fromX, fromY, x, y);
            fromX = x;
            fromY = y;
        }
    }
}

// One cell's lines, from `lines` to `end`, with its south-west corner at
// (originX, originY) on the canvas and a step of its grid stepX by stepY.
void drawCell(Pen &pen, const uint8_t *lines, const uint8_t *end, float originX, float originY, float stepX,
              float stepY) {
    // The line being read, a point at a time: kept whole before any of it is
    // drawn, a curve through a point needing the points either side of it.
    static std::vector<Point> points;
    int kind = 0;
    int firstX = 0, firstY = 0, x = 0, y = 0;
    float minX = 0, maxX = 0, minY = 0, maxY = 0;

    auto add = [&]() {
        Point p = {originX + x * stepX, originY - y * stepY, -1};
        if (points.empty()) {
            minX = maxX = p.x;
            minY = maxY = p.y;
        } else {
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }
        points.push_back(p);
    };
    auto draw = [&]() {
        int count = (int)points.size();
        // Most of a cell's lines are nowhere near the plot.
        if (count > 1 && maxX >= pen.left && minX <= pen.right && maxY >= pen.top && minY <= pen.bottom) {
            pen.travelled = 0;
            if (kind == (int)OutlineKind::COAST || kind == (int)OutlineKind::LAKE) {
                pen.style = SOLID;
                curve(pen, points.data(), count, count > 3 && x == firstX && y == firstY);
            } else {
                pen.style = kind == (int)OutlineKind::BORDER ? DASHED : DOTTED;
                for (int i = 0; i + 1 < count; i++) {
                    stroke(pen, points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
                }
            }
        }
        points.clear();
    };

    while (lines + 5 <= end) {
        int head = lines[0] | lines[1] << 8;
        int flags = head >> START_BITS, steps = lines[4];
        if (!(flags & CONTINUES)) {
            draw();
            kind = flags & KIND_MASK;
            firstX = x = head & ((1 << START_BITS) - 1);
            firstY = y = lines[2] | lines[3] << 8;
            add();
        }
        lines += 5;
        for (int i = 0; i < steps && lines + 2 <= end; i++, lines += 2) {
            x += (int8_t)lines[0];
            y += (int8_t)lines[1];
            add();
        }
    }
    draw();
}

}  // namespace

int coastDraw(const CoastPlot &plot) {
    if (plot.rangeNm <= 0 || plot.radius <= RING_CLEAR) {
        return 0;
    }
    // The same plot as the last one drawn: the pixels that one came to.
    Kept &kept = g_kept;
    if (kept.valid && kept.stride == plot.stride && kept.centerX == plot.centerX && kept.centerY == plot.centerY &&
        kept.radius == plot.radius && kept.lat == plot.lat && kept.lon == plot.lon && kept.rangeNm == plot.rangeNm) {
        for (uint32_t at : kept.pixels) {
            plot.pixels[at] = plot.color;
        }
        return kept.strokes;
    }
    kept.valid = true;
    kept.stride = plot.stride;
    kept.centerX = plot.centerX;
    kept.centerY = plot.centerY;
    kept.radius = plot.radius;
    kept.lat = plot.lat;
    kept.lon = plot.lon;
    kept.rangeNm = plot.rangeNm;
    kept.strokes = 0;
    kept.pixels.clear();

    int clip = plot.radius - RING_CLEAR;
    Pen pen = {};
    pen.pixels = plot.pixels;
    pen.stride = plot.stride;
    pen.color = plot.color;
    pen.centerX = plot.centerX;
    pen.centerY = plot.centerY;
    pen.radius2 = clip * clip;
    pen.left = (float)(plot.centerX - clip);
    pen.right = (float)(plot.centerX + clip);
    pen.top = (float)(plot.centerY - clip);
    pen.bottom = (float)(plot.centerY + clip);

    // The radar's own flat-earth projection, so a coast is where the traffic
    // over it is plotted: a degree of longitude as long everywhere on the
    // plot as it is at its middle. In doubles once here and once a cell - the
    // P4 has no hardware for them - and in floats for every point after.
    double pxPerLat = 60.0 * plot.radius / plot.rangeNm;
    double cosLat = std::max(cos(plot.lat * M_PI / 180.0), 0.01);
    double pxPerLon = pxPerLat * cosLat;
    double reachLat = plot.rangeNm / 60.0, reachLon = reachLat / cosLat;
    int south = std::max(0, (int)floor(plot.lat - reachLat + 90.0));
    int north = std::min(179, (int)floor(plot.lat + reachLat + 90.0));
    // Counted on past the date line rather than wrapped, so the cells either
    // side of it come out side by side - and once round at most, near a pole.
    int west = (int)floor(plot.lon - reachLon + 180.0);
    int east = std::min((int)floor(plot.lon + reachLon + 180.0), west + 359);

    const uint16_t *cellsEnd = COAST_CELLS + COAST_CELL_COUNT;
    for (int row = south; row <= north; row++) {
        for (int col = west; col <= east; col++) {
            // Where the cell is on the canvas, by its south-west corner - and
            // one in a corner of the square, wholly outside the circle, is
            // passed over without a look at what is in it.
            float originX = (float)(plot.centerX + (col - 180 - plot.lon) * pxPerLon);
            float originY = (float)(plot.centerY - (row - 90 - plot.lat) * pxPerLat);
            float offX = std::max({originX - plot.centerX, plot.centerX - (originX + (float)pxPerLon), 0.0f});
            float offY = std::max({originY - (float)pxPerLat - plot.centerY, plot.centerY - originY, 0.0f});
            if (offX * offX + offY * offY > (float)pen.radius2) {
                continue;
            }
            uint16_t id = (uint16_t)(row * 360 + ((col % 360) + 360) % 360);
            const uint16_t *cell = std::lower_bound(COAST_CELLS, cellsEnd, id);
            if (cell == cellsEnd || *cell != id) {
                continue;  // open sea, or land with no border across it
            }
            size_t i = cell - COAST_CELLS;
            drawCell(pen, COAST_LINES + COAST_STARTS[i], COAST_LINES + COAST_STARTS[i + 1], originX, originY,
                     (float)(pxPerLon / COAST_UNIT), (float)(pxPerLat / COAST_UNIT));
        }
    }
    return kept.strokes;
}
