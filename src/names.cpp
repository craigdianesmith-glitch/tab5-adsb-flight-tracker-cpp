#include "names.h"

#include <math.h>
#include <string.h>

#include <algorithm>

namespace {

struct PlaceName {
    float lat, lon;
    // The most a pixel of the name at size 1 can stand for, in nm, for it to
    // fit the room its outline has around (lat, lon).
    float nmPerPx;
    uint8_t kind;
    const char *text;
};

// Natural Earth's names, made by tools/gen_names.py: about seven thousand,
// in order of latitude, for a search to go straight to those near a plot.
#include "names_table.inc"
constexpr size_t NAME_COUNT = sizeof(NAMES) / sizeof(NAMES[0]);

// A name takes this much of the room it has, so that it sits inside its
// outline with some of the outline around it.
constexpr float FILL = 0.8f;
// Smaller than this it can't be read, and is left off.
constexpr int MIN_SIZE = 2;
// Its kind's: the most the font is scaled up, so a country's name is never
// more than a quiet word across the plot, and a region's or a lake's stays at
// the size of the rest of the plot's labels - Stirling's, a size up and spread
// wide, was a heading over the plot at 30nm; and the space between letters,
// over the font's own pixel, at size 1 - and the most it spreads to, as a
// name grows towards the next size.
constexpr uint8_t MAX_SIZE[] = {3, 3, 2, 2};
constexpr uint8_t SPACING[] = {2, 2, 1, 1};  // as tools/gen_names.py has them
constexpr uint8_t MAX_SPREAD[] = {2, 2, 1, 1};
// Between one name and the next, and from the plot's edge.
constexpr int NAME_PAD = 8, EDGE_PAD = 4;
// Where its spot is taken - by the cross at the middle of the plot, which is
// in the middle of the country it is centred on - a name is tried this far
// above it and below, if its room reaches that far: Germany's name was lost
// under Frankfurt's cross at 1000nm.
constexpr int NUDGE_PAD = 6;

struct Candidate {
    const PlaceName *name;
    float x, y;
    float size;  // the size it has room for, before it is made a whole one
};

int lettersOf(const char *line) {
    int n = 0;
    while (line[n] && line[n] != '\n') {
        n++;
    }
    return n;
}

// Whether `p` can go where it is: inside the ring, corner to corner, and
// clear of what `blocked` says and of the names already placed.
bool fits(const NamePlot &plot, float inner, const PlacedName &p, bool (*blocked)(int x, int y, int w, int h),
          const std::vector<PlacedName> &out) {
    for (int k = 0; k < 4; k++) {
        float dx = (k & 1 ? p.x + p.w : p.x) - plot.centerX, dy = (k & 2 ? p.y + p.h : p.y) - plot.centerY;
        if (dx * dx + dy * dy > inner * inner) {
            return false;
        }
    }
    if (blocked(p.x, p.y, p.w, p.h)) {
        return false;
    }
    for (const PlacedName &o : out) {
        if (p.x - NAME_PAD < o.x + o.w && p.x + p.w + NAME_PAD > o.x && p.y - NAME_PAD < o.y + o.h &&
            p.y + p.h + NAME_PAD > o.y) {
            return false;
        }
    }
    return true;
}

}  // namespace

int nameLineWidth(int letters, const PlacedName &n) {
    // The last letter's own blank column is no part of it.
    return letters ? letters * (NAME_ADVANCE * n.size + n.spacing) - n.spacing - n.size : 0;
}

void namesPlace(const NamePlot &plot, bool (*blocked)(int x, int y, int w, int h), std::vector<PlacedName> &out) {
    out.clear();
    float pxPerNm = plot.radius / plot.rangeNm;
    double nmPerDegLon = 60.0 * cos(plot.lat * M_PI / 180.0);
    float latReach = plot.rangeNm / 60.0f;

    static std::vector<Candidate> candidates;
    candidates.clear();
    const PlaceName *first =
        std::lower_bound(NAMES, NAMES + NAME_COUNT, (float)plot.lat - latReach,
                         [](const PlaceName &n, float lat) { return n.lat < lat; });
    for (const PlaceName *n = first; n < NAMES + NAME_COUNT && n->lat <= plot.lat + latReach; n++) {
        float size = FILL * n->nmPerPx * pxPerNm;
        if (size < MIN_SIZE) {
            continue;
        }
        double dLon = n->lon - plot.lon;
        if (dLon > 180) dLon -= 360;
        if (dLon < -180) dLon += 360;
        float ox = (float)(dLon * nmPerDegLon * pxPerNm), oy = (float)((n->lat - plot.lat) * 60.0 * pxPerNm);
        if (ox * ox + oy * oy > (float)plot.radius * plot.radius) {
            continue;  // its middle is off the plot
        }
        candidates.push_back({n, plot.centerX + ox, plot.centerY - oy, size});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
        return a.name->kind != b.name->kind ? a.name->kind < b.name->kind : a.size > b.size;
    });

    float inner = plot.radius - EDGE_PAD;
    for (const Candidate &c : candidates) {
        int kind = c.name->kind;
        PlacedName p = {c.name->text, (NameKind)kind, 0, 0, 0, 0, 0, 0};
        p.size = (uint8_t)std::min((int)c.size, (int)MAX_SIZE[kind]);
        // Its lines, and the longest of them in letters.
        int lines = 0, longest = 0;
        for (const char *line = p.text;; line += lettersOf(line) + 1) {
            lines++;
            longest = std::max(longest, lettersOf(line));
            if (!line[lettersOf(line)]) {
                break;
            }
        }
        // As wide as the room it has at its size, spread apart up to the most.
        p.spacing = SPACING[kind] * p.size;
        PlacedName unit = p;
        unit.size = 1;
        unit.spacing = SPACING[kind];
        int unitW = nameLineWidth(longest, unit);
        if (longest > 1) {
            float spare = (c.size - p.size) * unitW / (longest - 1);
            p.spacing += (uint8_t)std::min(spare, (float)MAX_SPREAD[kind] * p.size);
        }
        p.w = (int16_t)nameLineWidth(longest, p);
        p.h = (int16_t)(lines * NAME_HEIGHT * p.size + (lines - 1) * NAME_LINE_GAP * p.size);
        p.x = (int16_t)lroundf(c.x - p.w / 2.0f);
        int unitH = lines * NAME_HEIGHT + (lines - 1) * NAME_LINE_GAP;
        float slack = (c.size / FILL * unitH - p.h) / 2.0f;  // its room above and below it, all of it
        float nudge = p.h + NUDGE_PAD;
        int tries = slack >= nudge ? 3 : 1;
        for (int t = 0; t < tries; t++) {
            p.y = (int16_t)lroundf(c.y - p.h / 2.0f + (t == 0 ? 0 : t == 1 ? -nudge : nudge));
            if (fits(plot, inner, p, blocked, out)) {
                out.push_back(p);
                break;
            }
        }
    }
}
