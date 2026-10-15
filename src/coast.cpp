#include "coast.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include <string.h>

#include <algorithm>
#include <atomic>
#include <vector>

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

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
// A point less than this from the last one kept, along or across, is passed
// over - the line's last excepted. Close in that is none of them; out on the
// map's widest ranges, where Natural Earth has several to a pixel, it is
// most: at 2000nm round Glasgow, 187,000 strokes came to 25,000 pixels.
constexpr float POINT_MIN_PX = 1.0f;

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
// putting those pixels back is a loop that does nothing else, and took 4.
// In PSRAM on the device, four bytes a pixel, each pixel once: a couple of
// thousand of them at 20nm, nine thousand in that plot of Glasgow's -
// fourteen, when a pixel two strokes crossed was kept twice - fifteen over
// the lakes of Finland, and no more than thirty-five out on the map's widest
// ranges, where it was once a quarter of a million.
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
    // A list for each core that drew it - see work().
    Buffer<uint32_t> pixels[2];
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
    Buffer<uint32_t> *kept;  // where the pixels it sets are listed
    int strokes;             // and how many strokes it has drawn
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
        // Kept the once, however many strokes cross it: the plot is cleared
        // before the map is drawn on it, and nothing else is in its colour.
        // That is a read of the line of cache being written anyway, where a
        // pixel kept twice is put back twice every time the plot is redrawn -
        // nine in ten of them, at 2000nm.
        if ((!check || inCircle(x, y)) && (!dashed || dash < DASH_ON[pen.style]) && pen.pixels[at] != pen.color) {
            pen.pixels[at] = pen.color;
            pen.kept->push_back(at);
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
    pen.strokes++;
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
// (originX, originY) on the canvas and a step of its grid stepX by stepY -
// the line being read, a point at a time, gathered in `points`: kept whole
// before any of it is drawn, a curve through a point needing the points
// either side of it.
//
// A point is passed over or kept in the grid's own whole numbers, and only
// those kept are turned into canvas floats - at 2000nm, one in five. In
// locals for the loop over a line's steps: draw() sees the line's state by
// reference, which kept every one of them on the stack - a load and a store
// each, every point - where copies stay in registers. All of which took a
// first draw at 2000nm round Glasgow from 149ms to 70ms, the same pixels.
void drawCell(Pen &pen, std::vector<Point> &points, const uint8_t *lines, const uint8_t *end, float originX,
              float originY, float stepX, float stepY) {
    // Within a pixel, in grid steps: |d| * step < POINT_MIN_PX, for whole d.
    const int nearX = (int)ceilf(POINT_MIN_PX / stepX) - 1, nearY = (int)ceilf(POINT_MIN_PX / stepY) - 1;
    const unsigned spanX = 2u * nearX, spanY = 2u * nearY;
    int kind = 0, firstX = 0, firstY = 0, x = 0, y = 0;
    int minX = 0, maxX = 0, minY = 0, maxY = 0;
    // The point kept before the last, and the last - still to be placed, as
    // the next may yet stand in for it.
    int beforeX = 0, beforeY = 0, lastX = 0, lastY = 0, n = 0;
    points.clear();
    auto draw = [&]() {
        if (n > 0) {
            points.push_back({originX + lastX * stepX, originY - lastY * stepY, -1});
        }
        int count = (int)points.size();
        // Most of a cell's lines are nowhere near the plot.
        float l = originX + minX * stepX, r = originX + maxX * stepX;
        float t = originY - maxY * stepY, b = originY - minY * stepY;
        if (count > 1 && r >= pen.left && l <= pen.right && b >= pen.top && t <= pen.bottom) {
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
        n = 0;
    };
    while (lines + 5 <= end) {
        int head = lines[0] | lines[1] << 8;
        int flags = head >> START_BITS, steps = lines[4];
        if (!(flags & CONTINUES)) {
            draw();
            kind = flags & KIND_MASK;
            firstX = x = head & ((1 << START_BITS) - 1);
            firstY = y = lines[2] | lines[3] << 8;
            minX = maxX = x;
            minY = maxY = y;
            lastX = x;
            lastY = y;
            n = 1;
        }
        lines += 5;
        if (lines + 2 * steps > end) {
            steps = (int)(end - lines) / 2;
        }
        int cx = x, cy = y, lox = minX, hix = maxX, loy = minY, hiy = maxY;
        int bx = beforeX, by = beforeY, lx = lastX, ly = lastY, count = n;
        const uint8_t *p = lines, *stop = lines + 2 * steps;
        // A line's first two points can't be thinned - there is nothing kept
        // before them to be near - so they have a loop of their own, and the
        // loop for the rest has no count to test; and each distance test is
        // one unsigned compare, |d| <= k as (unsigned)(d + k) <= 2k. Some 15%
        // off the loop, measured - as fast as the same written in assembly.
        for (; count < 2 && p < stop; p += 2) {
            cx += (int8_t)p[0];
            cy += (int8_t)p[1];
            lox = cx < lox ? cx : lox;
            hix = cx > hix ? cx : hix;
            loy = cy < loy ? cy : loy;
            hiy = cy > hiy ? cy : hiy;
            if (count >= 1) {
                points.push_back({originX + lx * stepX, originY - ly * stepY, -1});
            }
            bx = lx;
            by = ly;
            lx = cx;
            ly = cy;
            count++;
        }
        for (; p < stop; p += 2) {
            cx += (int8_t)p[0];
            cy += (int8_t)p[1];
            lox = cx < lox ? cx : lox;
            hix = cx > hix ? cx : hix;
            loy = cy < loy ? cy : loy;
            hiy = cy > hiy ? cy : hiy;
            if ((unsigned)(cx - bx + nearX) <= spanX && (unsigned)(cy - by + nearY) <= spanY) {
                lx = cx;  // stands in for the last, which is passed over
                ly = cy;
            } else {
                points.push_back({originX + lx * stepX, originY - ly * stepY, -1});
                bx = lx;
                by = ly;
                lx = cx;
                ly = cy;
                count++;
            }
        }
        lines += 2 * steps;
        x = cx;
        y = cy;
        minX = lox;
        maxX = hix;
        minY = loy;
        maxY = hiy;
        beforeX = bx;
        beforeY = by;
        lastX = lx;
        lastY = ly;
        n = count;
    }
    draw();
}

// --- levels of detail --------------------------------------------------------
//
// Out on the map's widest ranges a pixel is kilometres across - ten of them at
// 2000nm - where Natural Earth has a point every one or two: a first draw there
// read 200,000 points for the 36,000 it kept. So the lines are kept twice more,
// thinned: built at start-up, on core 0, from the table in flash into PSRAM,
// the same layout read by the same code. A level is a line through no fewer of
// its points than keep it within `tolerance` grid steps of where it ran - the
// Douglas-Peucker way - and is drawn only where that is half a pixel or less,
// so a coast is where it was to within half a pixel. Each is thinned from the
// level before it, by the difference: the errors add, and stay inside.
struct Level {
    const uint32_t *starts;  // where each cell's lines begin in `lines`, and its end
    const uint8_t *lines;
    int tolerance;  // grid steps, from where the full table's lines run
};
constexpr int LEVEL_MAX = 3;
constexpr int LEVEL_TOLERANCE[LEVEL_MAX] = {0, 32, 64};
Level g_levels[LEVEL_MAX] = {{COAST_STARTS, COAST_LINES, 0}, {nullptr, nullptr, 0}, {nullptr, nullptr, 0}};
std::atomic<int> g_levelCount{1};  // how many are there to draw from yet
Buffer<uint32_t> g_levelStarts[LEVEL_MAX];
Buffer<uint8_t> g_levelLines[LEVEL_MAX];

struct GridPoint {
    int16_t x, y;
};

// Marks which of the points from a to b - both kept - a line within
// `tolerance` of them all needs: the farthest from a to b, if beyond it, and
// so on either side of that.
void keepFar(const GridPoint *p, int a, int b, int tolerance, std::vector<uint8_t> &keep) {
    static std::vector<std::pair<int, int>> todo;
    todo.clear();
    todo.push_back({a, b});
    int64_t tol2 = (int64_t)tolerance * tolerance;
    while (!todo.empty()) {
        std::pair<int, int> span = todo.back();
        todo.pop_back();
        int i0 = span.first, i1 = span.second;
        if (i1 - i0 < 2) {
            continue;
        }
        int dx = p[i1].x - p[i0].x, dy = p[i1].y - p[i0].y;
        int64_t len2 = (int64_t)dx * dx + (int64_t)dy * dy;
        int64_t far = -1;
        int at = -1;
        for (int i = i0 + 1; i < i1; i++) {
            int ex = p[i].x - p[i0].x, ey = p[i].y - p[i0].y;
            // The squared distance from the chord, times its squared length;
            // from the point a, for a chord of no length.
            int64_t d = len2 ? (int64_t)(dx * ey - dy * ex) * (dx * ey - dy * ex)
                             : ((int64_t)ex * ex + (int64_t)ey * ey);
            if (d > far) {
                far = d;
                at = i;
            }
        }
        if (far > (len2 ? tol2 * len2 : tol2)) {
            keep[at] = 1;
            todo.push_back({i0, at});
            todo.push_back({at, i1});
        }
    }
}

// One line, thinned, onto the end of `out` as the table lays one out: a start
// and its steps of a byte each way, a record for every 255 of them - and a
// step too long for a byte is taken in as many as it needs.
void writeLine(int kind, const std::vector<GridPoint> &pts, const std::vector<uint8_t> &keep, Buffer<uint8_t> &out) {
    size_t countAt = 0;
    int steps = 0;
    auto record = [&](bool continues, int x, int y) {
        int head = (continues ? CONTINUES : kind) << START_BITS | (continues ? 0 : x);
        out.push_back((uint8_t)(head & 0xFF));
        out.push_back((uint8_t)(head >> 8));
        out.push_back((uint8_t)(y & 0xFF));
        out.push_back((uint8_t)(y >> 8));
        countAt = out.size();
        out.push_back(0);
        steps = 0;
    };
    auto step = [&](int dx, int dy) {
        if (steps == 255) {
            record(true, 0, 0);
        }
        out.push_back((uint8_t)(int8_t)dx);
        out.push_back((uint8_t)(int8_t)dy);
        out[countAt] = (uint8_t)++steps;
    };
    record(false, pts[0].x, pts[0].y);
    int px = pts[0].x, py = pts[0].y;
    for (size_t i = 1; i < pts.size(); i++) {
        if (!keep[i]) {
            continue;
        }
        int dx = pts[i].x - px, dy = pts[i].y - py;
        int parts = std::max(1, (std::max(abs(dx), abs(dy)) + 126) / 127);
        int doneX = 0, doneY = 0;
        for (int k = 1; k <= parts; k++) {
            int toX = dx * k / parts, toY = dy * k / parts;
            step(toX - doneX, toY - doneY);
            doneX = toX;
            doneY = toY;
        }
        px = pts[i].x;
        py = pts[i].y;
    }
}

// One level from the one before it: every cell's lines read, thinned and
// written out again.
void buildLevel(int level) {
    const Level &from = g_levels[level - 1];
    int tolerance = LEVEL_TOLERANCE[level] - LEVEL_TOLERANCE[level - 1];
    Buffer<uint32_t> &starts = g_levelStarts[level];
    Buffer<uint8_t> &out = g_levelLines[level];
    starts.assign(COAST_CELL_COUNT + 1, 0);
    out.clear();
    out.reserve(from.starts[COAST_CELL_COUNT] / 2);
    std::vector<GridPoint> pts;
    std::vector<uint8_t> keep;
    int kind = 0;
    auto finish = [&]() {
        if (pts.empty()) {
            return;
        }
        keep.assign(pts.size(), 0);
        keep.front() = keep.back() = 1;
        int n = (int)pts.size();
        if (n > 3 && pts[0].x == pts[n - 1].x && pts[0].y == pts[n - 1].y) {
            // A ring: split at the point farthest from where it starts and
            // ends, so it isn't thinned against a chord of no length.
            int far = 1;
            int64_t best = -1;
            for (int i = 1; i < n - 1; i++) {
                int64_t d = (int64_t)(pts[i].x - pts[0].x) * (pts[i].x - pts[0].x) +
                            (int64_t)(pts[i].y - pts[0].y) * (pts[i].y - pts[0].y);
                if (d > best) {
                    best = d;
                    far = i;
                }
            }
            keep[far] = 1;
            keepFar(pts.data(), 0, far, tolerance, keep);
            keepFar(pts.data(), far, n - 1, tolerance, keep);
        } else {
            keepFar(pts.data(), 0, n - 1, tolerance, keep);
        }
        writeLine(kind, pts, keep, out);
        pts.clear();
    };
    for (size_t c = 0; c < COAST_CELL_COUNT; c++) {
        starts[c] = (uint32_t)out.size();
        const uint8_t *p = from.lines + from.starts[c], *end = from.lines + from.starts[c + 1];
        int x = 0, y = 0;
        while (p + 5 <= end) {
            int head = p[0] | p[1] << 8;
            int flags = head >> START_BITS, steps = p[4];
            if (!(flags & CONTINUES)) {
                finish();
                kind = flags & KIND_MASK;
                x = head & ((1 << START_BITS) - 1);
                y = p[2] | p[3] << 8;
                pts.push_back({(int16_t)x, (int16_t)y});
            }
            p += 5;
            for (int i = 0; i < steps && p + 2 <= end; i++, p += 2) {
                x += (int8_t)p[0];
                y += (int8_t)p[1];
                pts.push_back({(int16_t)x, (int16_t)y});
            }
        }
        finish();
#ifdef ESP_PLATFORM
        if ((c & 255) == 255) {
            vTaskDelay(1);  // core 0's idle task, and its watchdog, a look in
        }
#endif
    }
    starts[COAST_CELL_COUNT] = (uint32_t)out.size();
    g_levels[level] = {starts.data(), out.data(), LEVEL_TOLERANCE[level]};
    g_levelCount.store(level + 1, std::memory_order_release);
}

void buildLevels() {
    for (int level = 1; level < LEVEL_MAX; level++) {
        buildLevel(level);
    }
}

// The level a plot draws from: the most thinned whose tolerance is half a
// pixel or less at its scale.
const Level &levelFor(float pxPerStep) {
    int count = g_levelCount.load(std::memory_order_acquire);
    int best = 0;
    for (int level = 1; level < count; level++) {
        if (g_levels[level].tolerance * pxPerStep <= 0.5f) {
            best = level;
        }
    }
    return g_levels[best];
}

// --- the two cores ----------------------------------------------------------
//
// A first draw's cells shared out between the cores: each takes the next one
// not yet taken until none are left, so neither waits on the other's share
// of the dense ones. The canvas is the same memory to both - the P4's two
// cores share their data cache - and a pixel set by both is only kept twice.
struct CellJob {
    uint32_t cell;
    float originX, originY;
};
struct Job {
    const Level *level;
    const CellJob *cells;
    int count;
    float stepX, stepY;
    Pen pen;  // a template: each core draws with its own
    std::atomic<int> next{0};
};

void work(Job &job, Pen &pen, std::vector<Point> &points) {
    for (;;) {
        int k = job.next.fetch_add(1, std::memory_order_relaxed);
        if (k >= job.count) {
            return;
        }
        const CellJob &c = job.cells[k];
        drawCell(pen, points, job.level->lines + job.level->starts[c.cell],
                 job.level->lines + job.level->starts[c.cell + 1], c.originX, c.originY, job.stepX, job.stepY);
    }
}

#ifdef ESP_PLATFORM
// Core 0's share, drawn by a task of its own there - above the poll and the
// video encoder, so the loop is never left waiting on it for long.
TaskHandle_t g_helper = nullptr;
SemaphoreHandle_t g_helperDone = nullptr;
Job *g_helperJob = nullptr;
Pen g_helperPen;
std::vector<Point> g_helperPoints;
constexpr UBaseType_t HELPER_PRIORITY = 3;

void helperTask(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Job &job = *g_helperJob;
        g_helperPen = job.pen;
        g_helperPen.kept = &g_kept.pixels[1];
        g_helperPen.strokes = 0;
        work(job, g_helperPen, g_helperPoints);
        xSemaphoreGive(g_helperDone);
    }
}

bool helperReady() {
    if (g_helper == nullptr) {
        g_helperDone = xSemaphoreCreateBinary();
        if (g_helperDone == nullptr ||
            xTaskCreatePinnedToCore(helperTask, "coast", 6144, nullptr, HELPER_PRIORITY, &g_helper, 0) != pdPASS) {
            g_helper = nullptr;
            return false;
        }
    }
    return true;
}

void levelsTask(void *) {
    buildLevels();
    vTaskDelete(nullptr);
}
#endif

// The cells a plot reads, in the order the table has them: see coastDraw().
void gatherCells(const CoastPlot &plot, double pxPerLat, double pxPerLon, int west, int east, int south, int north,
                 int radius2, std::vector<CellJob> &cells) {
    // The cells there are, west to east along each row: the table has them
    // in order of row * 360 + column, so one search a row finds its first,
    // and only cells with something in them are looked at - not every square
    // degree of open sea, as when each was searched for. Most are left out
    // as wholly outside the circle without a look at what is in them.
    cells.clear();
    float pxLon = (float)pxPerLon, pxLat = (float)pxPerLat;
    double lonFrom180 = plot.lon + 180.0;
    float r2 = (float)radius2;
    const uint16_t *cellsEnd = COAST_CELLS + COAST_CELL_COUNT;
    for (int row = south; row <= north; row++) {
        float originY = (float)(plot.centerY - (row - 90 - plot.lat) * pxPerLat);
        float offY = std::max({originY - pxLat - plot.centerY, plot.centerY - originY, 0.0f});
        if (offY * offY > r2) {
            continue;
        }
        // West to east as columns counted on past the date line either way,
        // as the plot needs them: cut at each multiple of 360, each piece is
        // a run of the row's cells - two at most.
        int k0 = (int)floor(west / 360.0), k1 = (int)floor(east / 360.0);
        for (int k = k0; k <= k1; k++) {
            int from = std::max(west, k * 360), to = std::min(east, k * 360 + 359);
            uint16_t lo = (uint16_t)(row * 360 + (from - k * 360)), hi = (uint16_t)(row * 360 + (to - k * 360));
            for (const uint16_t *cell = std::lower_bound(COAST_CELLS, cellsEnd, lo); cell < cellsEnd && *cell <= hi;
                 cell++) {
                int col = *cell - row * 360 + k * 360;
                float originX = plot.centerX + (float)(col - lonFrom180) * pxLon;
                float offX = std::max({originX - plot.centerX, plot.centerX - (originX + pxLon), 0.0f});
                if (offX * offX + offY * offY <= r2) {
                    cells.push_back({(uint32_t)(cell - COAST_CELLS), originX, originY});
                }
            }
        }
    }
}

}  // namespace

void coastBegin() {
#ifdef ESP_PLATFORM
    // Below the poll: it is in no hurry, and the wide ranges draw from the
    // full table until it is done.
    xTaskCreatePinnedToCore(levelsTask, "coastlod", 6144, nullptr, 1, nullptr, 0);
#endif
}

int coastLevels() { return g_levelCount.load(std::memory_order_acquire); }

int coastDraw(const CoastPlot &plot) {
    if (plot.rangeNm <= 0 || plot.radius <= RING_CLEAR) {
        return 0;
    }
#ifndef ESP_PLATFORM
    // On the PC, built the first time, before the first draw.
    static bool built = false;
    if (!built) {
        buildLevels();
        built = true;
    }
#endif
    // The same plot as the last one drawn: the pixels that one came to.
    Kept &kept = g_kept;
    if (kept.valid && kept.stride == plot.stride && kept.centerX == plot.centerX && kept.centerY == plot.centerY &&
        kept.radius == plot.radius && kept.lat == plot.lat && kept.lon == plot.lon && kept.rangeNm == plot.rangeNm) {
        for (const Buffer<uint32_t> &list : kept.pixels) {
            for (uint32_t at : list) {
                plot.pixels[at] = plot.color;
            }
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
    kept.pixels[0].clear();
    kept.pixels[1].clear();

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
    // plot as it is at its middle. In doubles once here and once a row - the
    // P4 has no hardware for them - and in floats for every cell after.
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

    static std::vector<CellJob> cells;
    gatherCells(plot, pxPerLat, pxPerLon, west, east, south, north, pen.radius2, cells);

    Job job;
    job.level = &levelFor((float)(pxPerLat / COAST_UNIT));
    job.cells = cells.data();
    job.count = (int)cells.size();
    job.stepX = (float)(pxPerLon / COAST_UNIT);
    job.stepY = (float)(pxPerLat / COAST_UNIT);
    job.pen = pen;
    static std::vector<Point> points;
    Pen mine = pen;
    mine.kept = &kept.pixels[0];
    mine.strokes = 0;
#ifdef ESP_PLATFORM
    bool both = job.count >= 2 && helperReady();
    if (both) {
        g_helperJob = &job;
        xTaskNotifyGive(g_helper);
    }
    work(job, mine, points);
    if (both) {
        xSemaphoreTake(g_helperDone, portMAX_DELAY);
        mine.strokes += g_helperPen.strokes;
    }
#else
    work(job, mine, points);
#endif
    kept.strokes = mine.strokes;
    return kept.strokes;
}

