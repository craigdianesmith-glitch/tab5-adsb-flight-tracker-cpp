#pragma once

#include <stdint.h>

#include <vector>

// The names written on the map under a plot, faintly: its countries; their
// regions - states, provinces, counties, council areas; its seas, bays and
// straits; and its lakes. From Natural Earth, placed ahead of time by
// tools/gen_names.py.
//
// Each goes where its outline has the most room for it, sized to that room
// at the range shown: zooming out, a region's name shrinks until it is too
// small to read and goes, and its country's comes into view; zooming in, the
// other way. Between the font's sizes its letters spread apart, so that it
// grows with the range rather than in steps. Only where that spot is on the
// plot and the whole name fits inside the ring, clear of the names before it
// and of whatever the caller says is in the way.
enum class NameKind : uint8_t { COUNTRY, SEA, REGION, LAKE };

// A plot to name: a circle on the canvas, and what is under it.
struct NamePlot {
    int centerX, centerY, radius;
    double lat, lon;  // at the middle of the circle
    float rangeNm;    // at its edge
};

// A name to write, in Font0.
struct PlacedName {
    const char *text;  // its lines apart at a newline
    NameKind kind;
    uint8_t size;       // the font's
    uint8_t spacing;    // pixels between letters, besides the font's own
    int16_t x, y, w, h;  // what it covers on the canvas
};

// The names that go on `plot` into `out`, the more important first: the
// countries, then the seas, the regions and the lakes, and of each the
// bigger first. Each is kept off wherever `blocked` says.
void namesPlace(const NamePlot &plot, bool (*blocked)(int x, int y, int w, int h), std::vector<PlacedName> &out);

// Where each letter of `n` goes: `draw(c, x, y)` with its top left corner,
// in Font0 at n.size - line by line, each centred in the name's width.
constexpr int NAME_ADVANCE = 6, NAME_HEIGHT = 8, NAME_LINE_GAP = 3;  // at size 1, as tools/gen_names.py has them

// How wide a line of `letters` letters of `n` is.
int nameLineWidth(int letters, const PlacedName &n);

template <typename F>
void nameLetters(const PlacedName &n, F draw) {
    const char *line = n.text;
    int y = n.y;
    for (;;) {
        int letters = 0;
        while (line[letters] && line[letters] != '\n') {
            letters++;
        }
        int x = n.x + (n.w - nameLineWidth(letters, n)) / 2;
        for (int i = 0; i < letters; i++) {
            draw(line[i], x, y);
            x += NAME_ADVANCE * n.size + n.spacing;
        }
        if (!line[letters]) {
            return;
        }
        line += letters + 1;
        y += (NAME_HEIGHT + NAME_LINE_GAP) * n.size;
    }
}
