#include "display.h"

#include "aircraft_db.h"

#include <math.h>

#include <algorithm>

#include "alerts.h"
#include "config.h"
#include "filter.h"
#include "screen.h"

namespace {

struct Column {
    const char *title;
    int width;
};

const Column COLUMNS[] = {
    {"FLIGHT", 300}, {"TYPE", 140}, {"ALT", 150}, {"SPD", 150}, {"DIST", 150}, {"HDG", 110}, {"STATUS", 250},
};
// Sized against the widest realistic value in each, drawn in FreeSans 9pt at
// size 2: FLIGHT fits eight W's, the widest callsign there can be (272px);
// STATUS fits DESCEND behind its icon; HDG never holds more than three digits.
// Each leaves at least CELL_PAD_RIGHT clear of its right border.
constexpr int NUM_COLS = 7;
constexpr int CELL_PAD_RIGHT = 8;
constexpr float MIN_CELL_TEXT_SIZE = 1.2f;
// The only column drawn differently - it carries an icon ahead of its text.
constexpr int STATUS_COL = NUM_COLS - 1;
constexpr int HDG_COL = 5;  // not one to sort by: which way each is pointing says nothing in order
constexpr int TABLE_X = 8;
constexpr int TABLE_W = 1264;
constexpr int HEADER_Y = 68;
constexpr int HEADER_H = 54;
constexpr int TABLE_Y = HEADER_Y + HEADER_H;
constexpr int ROW_HEIGHT = DISPLAY_ROW_HEIGHT;  // in display.h, for the drag that scrolls it
constexpr int MAX_CACHE_ROWS = 16;  // generous upper bound on any screen size we'd realistically run at

uint16_t colorBg, colorWhite, colorHeaderBg, colorHeaderSorted, colorHeaderText, colorBorder, colorGrey, colorNew,
    colorChanged, colorStale, colorMutedIcon, colorAlert, colorEmergency, colorCargo;

constexpr int STATUS_ICON_SIZE = 18;

// The header's controls sit together at the right edge, leaving the whole
// left of the bar to the title line. Laid out from the right so the cog - the
// one that leads everywhere else now - lands in the corner.
constexpr int ICON_Y = 6, ICON_SIZE = 52, ICON_GAP = 12;
constexpr int COG_X = TABLE_X + TABLE_W - ICON_SIZE;
constexpr int MUTE_X = COG_X - ICON_GAP - ICON_SIZE;
constexpr int RADAR_X = MUTE_X - ICON_GAP - ICON_SIZE;
// The recordings, which were two screens away - or three, with the radar
// coming back to an airport's zoom - behind the radar's Replays button.
constexpr int REPLAYS_X = RADAR_X - ICON_GAP - ICON_SIZE;
// The filter screen's way in, lit while a filter is on - see filter.h.
constexpr int FILTER_X = REPLAYS_X - ICON_GAP - ICON_SIZE;

// The strip left over between the last row that fits and the bottom of the
// screen - too short for a row, tall enough for a line of size-2 text.
//
// Only as wide as the longest string it can hold ("Last update 10m 30s ago -
// retrying", 34 chars of 12px), rather than the full table width: this strip
// repaints every second as the age ticks over, and the rotated blit costs its
// own area, so the 800px of empty background to the right of the text was the
// bulk of what that repaint was paying for.
constexpr int STATUS_H = 22;
constexpr int STATUS_W = 460;

// The header bar's vertical centre. The icons sit in the 6..58 band, so the
// title lines up with them rather than with the
// nominal centre of the strip. The +3 is an optical nudge: ML_DATUM centres
// the whole glyph cell including the descender space, which a line of capitals
// doesn't use, so a mathematically centred title reads as sitting high.
constexpr int HEADER_MID_Y = ICON_Y + ICON_SIZE / 2 + 3;
int colX[NUM_COLS];

// Render cache: what's currently on screen, so a redraw only touches cells
// whose value or color actually changed instead of repainting everything.
bool g_headerDrawn = false;
bool g_lastWasEmpty = false;
String g_lastCell[MAX_CACHE_ROWS][NUM_COLS];
uint16_t g_lastCellColor[MAX_CACHE_ROWS][NUM_COLS];
bool g_lastCellValid[MAX_CACHE_ROWS][NUM_COLS] = {};
// millis() at which a cell's shading lapses; 0 means it isn't shaded.
uint32_t g_highlightUntil[MAX_CACHE_ROWS][NUM_COLS] = {};
bool g_showRefresh = true;
// A filter on, and how many aircraft there were before it: the list given
// to displayRenderAircraft() is what passed it.
bool g_filterOn = false;
int g_unfiltered = 0, g_lastUnfilteredShown = -1;
String g_airportCode;
String g_provider;
bool g_muted = false;

// Poll state, for the status line under the table.
bool g_linkUp = true;
bool g_pollOk = true;
bool g_everSucceeded = false;
uint32_t g_lastSuccessMs = 0;
// Which message the status line shows and the age it quotes, packed so that
// loop()'s hundred passes a second can tell nothing has changed before
// building any Strings to say so.
constexpr uint32_t STATUS_NONE = UINT32_MAX;  // sentinel that no real status packs to
uint32_t g_lastStatusKey = STATUS_NONE;
// More aircraft than rows: the first shown, dragged through by
// displayScrollTo(), and how many there were at the last render. And the two
// as the status line last said them.
int g_scroll = 0, g_count = 0;
// The column the rows are in order of - a tap on its header, again the other
// way - and the aircraft each row shows, by hex, for a tap on it to find.
int g_sortCol = 4;  // DIST, nearest first
bool g_sortDesc = false;
String g_rowHex[MAX_CACHE_ROWS];
int g_lastScrollShown = -1, g_lastCountShown = -1;

constexpr uint32_t NOTICE_MS = 4000;
String g_notice;
uint32_t g_noticeUntil = 0;
uint32_t g_noticeGen = 0;  // packed into the status key, so each notice is a change

// Drawn geometric icons rather than a hand-authored bitmap: precise and
// reliable without needing to eyeball pixel arrays on real hardware.
void drawStatusIcon(int cx, int cy, const String &status, uint16_t color) {
    auto &canvas = screen::canvas();
    int half = STATUS_ICON_SIZE / 2;
    if (status == "CLIMB") {
        canvas.fillTriangle(cx, cy - half, cx - half, cy + half, cx + half, cy + half, color);
    } else if (status == "DESCEND") {
        canvas.fillTriangle(cx, cy + half, cx - half, cy - half, cx + half, cy - half, color);
    } else if (status == "LEVEL") {
        canvas.fillRect(cx - half, cy - 3, STATUS_ICON_SIZE, 6, color);
    } else if (status == "TAXI") {
        canvas.fillCircle(cx, cy, half, color);
    } else if (status == "GROUND") {
        canvas.drawRect(cx - half, cy - half, STATUS_ICON_SIZE, STATUS_ICON_SIZE, color);
    }
}

// Drawn rather than a bitmap, for the same reason as the status icons: the
// geometry is exact at any size and there are no pixel arrays to eyeball.
void drawGear(int cx, int cy, int r, uint16_t color) {
    auto &canvas = screen::canvas();
    constexpr int TEETH = 8;
    const float ri = r * 0.90f;   // tooth root, just inside the rim
    const float ro = r * 1.32f;   // tooth tip
    for (int i = 0; i < TEETH; i++) {
        float a = (float)i * 2.0f * (float)PI / TEETH;
        float root = 0.26f;       // half-angle at the root
        float tip = 0.15f;        // narrower at the tip, so the teeth taper
        int x1 = cx + (int)lroundf(ri * cosf(a - root)), y1 = cy + (int)lroundf(ri * sinf(a - root));
        int x2 = cx + (int)lroundf(ro * cosf(a - tip)), y2 = cy + (int)lroundf(ro * sinf(a - tip));
        int x3 = cx + (int)lroundf(ro * cosf(a + tip)), y3 = cy + (int)lroundf(ro * sinf(a + tip));
        int x4 = cx + (int)lroundf(ri * cosf(a + root)), y4 = cy + (int)lroundf(ri * sinf(a + root));
        canvas.fillTriangle(x1, y1, x2, y2, x3, y3, color);
        canvas.fillTriangle(x1, y1, x3, y3, x4, y4, color);
    }
    canvas.fillCircle(cx, cy, r, color);
    canvas.fillCircle(cx, cy, (int)lroundf(r * 0.40f), colorBg);
}

void drawRadarIcon(int cx, int cy, int r, uint16_t color) {
    auto &canvas = screen::canvas();
    canvas.drawCircle(cx, cy, r, color);
    canvas.drawCircle(cx, cy, (r * 2) / 3, color);
    canvas.drawCircle(cx, cy, r / 3, color);
    // sweep line, up and to the right
    canvas.drawLine(cx, cy, cx + (int)lroundf(r * 0.707f), cy - (int)lroundf(r * 0.707f), color);
}

// Replays: an arrow round on itself, as a player's "again" is, with a play
// triangle in the middle of it. The arrow runs anticlockwise from eleven
// o'clock round to one, where its head is, leaving the top open.
void drawReplayIcon(int cx, int cy, int r, uint16_t color) {
    auto &canvas = screen::canvas();
    // Split at 0, as the speaker's arcs are: clockwise from due east.
    canvas.drawArc(cx, cy, r - 2, r, 300, 360, color);
    canvas.drawArc(cx, cy, r - 2, r, 0, 240, color);
    // The head, at 300 degrees, pointing the way the arrow has come round:
    // up and to the left, across the gap.
    constexpr float AT = 300.0f * (float)M_PI / 180.0f;
    float px = cx + (r - 1) * cosf(AT), py = cy + (r - 1) * sinf(AT);
    float outX = cosf(AT), outY = sinf(AT), onX = sinf(AT), onY = -cosf(AT);
    float half = r * 0.34f, len = r * 0.52f;
    canvas.fillTriangle((int)lroundf(px + outX * half), (int)lroundf(py + outY * half),
                        (int)lroundf(px - outX * half), (int)lroundf(py - outY * half),
                        (int)lroundf(px + onX * len), (int)lroundf(py + onY * len), color);
    int left = cx - (int)lroundf(r * 0.24f), up = (int)lroundf(r * 0.40f);
    canvas.fillTriangle(left, cy - up, left, cy + up, cx + (int)lroundf(r * 0.44f), cy, color);
}

// A funnel: a wide mouth narrowing to a spout.
void drawFunnel(int cx, int cy, int r, uint16_t color) {
    auto &canvas = screen::canvas();
    int top = cy - (int)lroundf(r * 0.80f), neck = cy + (int)lroundf(r * 0.15f), bottom = cy + (int)lroundf(r * 0.85f);
    int spout = (int)lroundf(r * 0.20f);
    canvas.fillTriangle(cx - r, top, cx + r, top, cx - spout, neck, color);
    canvas.fillTriangle(cx + r, top, cx + spout, neck, cx - spout, neck, color);
    canvas.fillRect(cx - spout, neck, 2 * spout + 1, bottom - neck, color);
}

// Drawn like the others: a driver box, a cone flaring right as two triangles,
// and either sound coming out of it or a bar through it.
void drawSpeaker(int cx, int cy, int r, bool muted, uint16_t color) {
    auto &canvas = screen::canvas();
    int boxW = (int)lroundf(r * 0.45f);
    int boxH = (int)lroundf(r * 0.44f);
    int left = cx - r;
    int coneX = left + boxW;
    int coneR = (int)lroundf(r * 0.30f);   // right edge of the cone
    int coneH = (int)lroundf(r * 0.92f);   // half-height at that edge

    canvas.fillRect(left, cy - boxH / 2, boxW + 1, boxH, color);
    canvas.fillTriangle(coneX, cy - boxH / 2, coneX, cy + boxH / 2, cx + coneR, cy - coneH, color);
    canvas.fillTriangle(coneX, cy + boxH / 2, cx + coneR, cy - coneH, cx + coneR, cy + coneH, color);

    if (muted) {
        // A bar through it rather than waves - unmistakable at this size, and
        // it needs no arc drawing to come out symmetrical.
        for (int i = -1; i <= 1; i++) {
            canvas.drawLine(left + i, cy - coneH, cx + coneR + i, cy + coneH, color);
        }
        return;
    }
    // Split at 0 rather than asking for 300..420 and trusting the wrap to be
    // handled: two explicit quadrants either side of due east.
    for (int i = 1; i <= 2; i++) {
        int rr = coneR + i * (int)lroundf(r * 0.26f);
        canvas.drawArc(cx, cy, rr, rr + 2, 300, 360, color);
        canvas.drawArc(cx, cy, rr, rr + 2, 0, 60, color);
    }
}

// Above 9999ft the five-digit number stops fitting a narrower column and
// stops being how the altitude is actually referred to, so it reads as a
// flight level: hundreds of feet, as the convention has it, so 20000ft is
// FL200. Below that the figure is given in feet as before.
String formatAltitude(const String &altStr) {
    if (altStr == "GND" || altStr == "?") {
        return altStr;
    }
    long ft = altStr.toInt();
    if (ft > 9999) {
        return "FL" + String((ft + 50) / 100);
    }
    return altStr + "ft";
}

// Three digits, the way a heading is written and spoken - "035", not "35".
String formatHeading(const Aircraft &ac) {
    if (!ac.hasTrack) {
        return "?";
    }
    int deg = ((int)lroundf(ac.track) % 360 + 360) % 360;
    char buf[8];
    snprintf(buf, sizeof(buf), "%03d", deg);
    return String(buf);
}

// `erase` says whether the cell's area may hold anything but background. A
// cell with no cached value never does - every path that invalidates one also
// paints it over, whether by clear() on a full repaint or clearRow() - so it
// gets its border and nothing else. Narrow rects are slow to fill into a
// canvas in PSRAM, and skipping fills that repainted what the PPA had just
// filled with the same colour took a full repaint from 87ms to 75ms.
//
// A cell that is erased is filled inside its border rather than over it, so
// the border survives and is only ever drawn once: every refresh and every
// shading lapsing used to redraw it too.
void drawCell(int r, int c, const String &value, uint16_t color, bool highlight, bool erase) {
    auto &canvas = screen::canvas();
    int w = COLUMNS[c].width - 4;
    int y = TABLE_Y + r * ROW_HEIGHT;
    int h = ROW_HEIGHT - 4;
    if (erase || highlight) {
        canvas.fillRect(colX[c] + 1, y + 1, w - 2, h - 2, highlight ? colorChanged : colorBg);
    }
    if (!erase) {
        canvas.drawRect(colX[c], y, w, h, colorBorder);
    }
    canvas.setFont(&fonts::FreeSans9pt7b);
    canvas.setTextColor(color);
    canvas.setTextDatum(ML_DATUM);
    int midY = y + h / 2;
    int textX = colX[c] + 10;
    if (c == STATUS_COL) {
        drawStatusIcon(textX + STATUS_ICON_SIZE / 2, midY, value, color);
        textX += STATUS_ICON_SIZE + 8;
    }
    // The widths are sized for the widest realistic value at size 2, so this
    // rarely does anything - but a value wider than that is drawn smaller to
    // fit, rather than running over the border into the next cell.
    int room = colX[c] + w - CELL_PAD_RIGHT - textX;
    float size = 2.0f;
    canvas.setTextSize(size);
    while (size > MIN_CELL_TEXT_SIZE && canvas.textWidth(value) > room) {
        size -= 0.1f;
        canvas.setTextSize(size);
    }
    canvas.drawString(value, textX, midY);
    screen::markDirty(colX[c], y, w, h);
}

void clearRow(int r) {
    int y = TABLE_Y + r * ROW_HEIGHT;
    screen::canvas().fillRect(TABLE_X, y, TABLE_W, ROW_HEIGHT - 4, colorBg);
    screen::markDirty(TABLE_X, y, TABLE_W, ROW_HEIGHT - 4);
    for (int c = 0; c < NUM_COLS; c++) {
        g_lastCellValid[r][c] = false;
        g_highlightUntil[r][c] = 0;
    }
}

}  // namespace

void displayInit() {
    M5.Display.setRotation(3);

    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorHeaderBg = M5.Display.color565(0xB6, 0xF2, 0xB6);
    colorHeaderSorted = M5.Display.color565(0x5E, 0xC9, 0x6E);  // the column the rows are in order of
    colorHeaderText = M5.Display.color565(0x10, 0x20, 0x10);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorGrey = M5.Display.color565(0x88, 0x88, 0x88);
    colorNew = M5.Display.color565(0x1B, 0x7A, 0x1B);
    colorChanged = M5.Display.color565(0x1E, 0x2E, 0x40);
    colorStale = M5.Display.color565(0xF3, 0x9C, 0x12);  // amber: on screen but not current
    colorMutedIcon = M5.Display.color565(0x5A, 0x60, 0x68);  // dimmer than the live icons
    // The same two as the radar's, so an alerted aircraft reads the same on both.
    colorAlert = M5.Display.color565(0xFF, 0xB3, 0x2E);
    colorEmergency = M5.Display.color565(0xFF, 0x4D, 0x4D);
    colorCargo = M5.Display.color565(0x7F, 0xC8, 0xFF);  // a cargo operator's callsign - the radar's too

    int x = TABLE_X;
    for (int i = 0; i < NUM_COLS; i++) {
        colX[i] = x;
        x += COLUMNS[i].width;
    }
}

// The column headings: the one the rows are sorted by a deeper green, with a
// small triangle after its title pointing the way they run - up for smallest
// first - the two centred together in the cell.
static void drawColumnHeads() {
    auto &canvas = screen::canvas();
    // Its own font, set here: redrawn by a tap, the canvas still has the
    // cells' FreeSans from the last refresh, and the headings came out in it.
    canvas.setFont(&fonts::Font0);
    constexpr int TRI_GAP = 10, TRI_W = 12;
    int x = TABLE_X;
    for (int i = 0; i < NUM_COLS; i++) {
        int w = COLUMNS[i].width - 4;
        bool sorted = (i == g_sortCol);
        canvas.fillRect(x, HEADER_Y, w, HEADER_H, sorted ? colorHeaderSorted : colorHeaderBg);
        canvas.setTextColor(colorHeaderText);
        canvas.setTextSize(2);
        canvas.setTextDatum(MC_DATUM);
        int tw = canvas.textWidth(COLUMNS[i].title);
        int cx = x + w / 2 - (sorted ? (TRI_GAP + TRI_W) / 2 : 0);
        canvas.drawString(COLUMNS[i].title, cx, HEADER_Y + HEADER_H / 2);
        if (sorted) {
            int tx = cx + tw / 2 + TRI_GAP + TRI_W / 2, ty = HEADER_Y + HEADER_H / 2;
            if (g_sortDesc) {
                canvas.fillTriangle(tx - 6, ty - 4, tx + 6, ty - 4, tx, ty + 5, colorHeaderText);
            } else {
                canvas.fillTriangle(tx - 6, ty + 4, tx + 6, ty + 4, tx, ty - 5, colorHeaderText);
            }
        }
        x += COLUMNS[i].width;
    }
    screen::markDirty(TABLE_X, HEADER_Y, TABLE_W, HEADER_H);
}

// What a row is sorted by, in its column: a number where the column holds one,
// with what isn't known - a "?" - at the end whichever way the column runs.
struct SortKey {
    bool known;
    float number;
    const String *text;
};
static SortKey sortKey(const Aircraft &a, int col) {
    switch (col) {
    case 0: return {true, 0, &a.callsign};
    case 1: return {a.type.length() > 0 && a.type != "----", 0, &a.type};
    case 2: return {a.altStr != "?", a.altStr == "GND" ? 0.0f : (float)a.altStr.toInt(), nullptr};
    case 3: return {a.speedStr != "?", (float)a.speedStr.toInt(), nullptr};
    case 4: return {a.hasDist, a.distNm, nullptr};
    case 5: return {a.hasTrack, a.track, nullptr};
    default: return {a.status.length() > 0, 0, &a.status};
    }
}

int displayMaxRows() {
    int rows = (screen::canvas().height() - TABLE_Y - 8) / ROW_HEIGHT;
    if (rows > MAX_CACHE_ROWS) {
        rows = MAX_CACHE_ROWS;
    }
    return rows < 0 ? 0 : rows;
}

namespace {
bool hitIcon(int iconX, int x, int y) {
    return x >= iconX && x < iconX + ICON_SIZE && y >= ICON_Y && y < ICON_Y + ICON_SIZE;
}
}  // namespace

bool displayHitCog(int x, int y) { return hitIcon(COG_X, x, y); }

bool displayHitRadar(int x, int y) { return hitIcon(RADAR_X, x, y); }

bool displayHitReplays(int x, int y) { return hitIcon(REPLAYS_X, x, y); }

bool displayHitMute(int x, int y) { return hitIcon(MUTE_X, x, y); }

bool displayHitFilter(int x, int y) { return hitIcon(FILTER_X, x, y); }

bool displayHitHeader(int x, int y) {
    if (y < HEADER_Y || y >= HEADER_Y + HEADER_H || x < TABLE_X || x >= TABLE_X + TABLE_W) {
        return false;
    }
    int col = 0;
    for (int cx = TABLE_X + COLUMNS[0].width; col < NUM_COLS - 1 && x >= cx; cx += COLUMNS[++col].width) {
    }
    if (col == HDG_COL) {
        return true;  // a heading isn't an order: the tap is taken, and does nothing
    }
    g_sortDesc = (col == g_sortCol) ? !g_sortDesc : false;
    g_sortCol = col;
    g_scroll = 0;  // from the top of the new order
    drawColumnHeads();
    screen::flush();
    return true;
}

void displaySetSort(int col, bool descending) {
    g_sortCol = (col >= 0 && col < NUM_COLS && col != HDG_COL) ? col : 4;
    g_sortDesc = descending;
}

int displaySortColumn() { return g_sortCol; }
bool displaySortDescending() { return g_sortDesc; }

bool displayHitRow(int x, int y, String &outHex) {
    if (x < TABLE_X || x >= TABLE_X + TABLE_W || y < TABLE_Y) {
        return false;
    }
    int row = (y - TABLE_Y) / ROW_HEIGHT;
    // Bounded by the rows actually drawn, not by how many aircraft are held:
    // the strip below the last row is the status line, and a tap there used to
    // land on an aircraft the table had never shown.
    if (row < 0 || row >= displayMaxRows()) {
        return false;
    }
    outHex = g_rowHex[row];
    return outHex.length() > 0;
}

int displayScroll() { return g_scroll; }

bool displayScrollTo(int first) {
    first = std::max(0, std::min(first, g_count - displayMaxRows()));
    if (first == g_scroll) {
        return false;
    }
    g_scroll = first;
    return true;
}

void displayInvalidate() {
    // Another screen has been drawing on the shared canvas, so nothing cached
    // here is on it any more. Clearing g_headerDrawn makes the next render
    // clear the canvas and repaint from scratch.
    g_headerDrawn = false;
    g_lastWasEmpty = false;
    g_lastStatusKey = STATUS_NONE;
    g_lastScrollShown = g_lastCountShown = -1;
    for (int r = 0; r < MAX_CACHE_ROWS; r++) {
        for (int c = 0; c < NUM_COLS; c++) {
            g_lastCellValid[r][c] = false;
            g_highlightUntil[r][c] = 0;
        }
    }
}

void displaySetShowRefresh(bool enabled) { g_showRefresh = enabled; }

void displaySetPollState(bool linkUp, bool lastPollOk, bool everSucceeded, uint32_t lastSuccessMs) {
    g_linkUp = linkUp;
    g_pollOk = lastPollOk;
    g_everSucceeded = everSucceeded;
    g_lastSuccessMs = lastSuccessMs;
}

// Which of the aircraft the rows are, when they aren't all of them - "1-11 of
// 23 - drag for more" - in a strip of its own at the right of the status
// line's, repainted only when it changes rather than with the age each second.
constexpr int RANGE_W = 480;
void drawRangeShown() {
    if (g_scroll == g_lastScrollShown && g_count == g_lastCountShown && g_unfiltered == g_lastUnfilteredShown) {
        return;
    }
    g_lastScrollShown = g_scroll;
    g_lastCountShown = g_count;
    g_lastUnfilteredShown = g_unfiltered;
    auto &canvas = screen::canvas();
    int rows = displayMaxRows();
    int x = TABLE_X + TABLE_W - RANGE_W, y = TABLE_Y + rows * ROW_HEIGHT;
    canvas.fillRect(x, y, RANGE_W, STATUS_H, colorBg);
    String text;
    if (g_count > rows) {
        text = String(g_scroll + 1) + "-" + String(std::min(g_scroll + rows, g_count)) + " of " + String(g_count) +
               (g_filterOn ? " filtered" : "") + " - drag for more";
    } else if (g_filterOn) {
        text = String(g_count) + " of " + String(g_unfiltered) + " pass the filter";
    }
    if (text.length()) {
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(2);
        canvas.setTextColor(g_filterOn ? colorHeaderSorted : colorGrey);
        canvas.setTextDatum(MR_DATUM);
        canvas.drawString(text, TABLE_X + TABLE_W - 10, y + STATUS_H / 2);
    }
    screen::markDirty(x, y, RANGE_W, STATUS_H);
}

void displayTickStatus() {
    // The status line belongs to the table, so don't paint it over whatever is
    // there until the table itself has been drawn: returning from the radar
    // invalidates the cache, and a render that loses the race for the data
    // mutex would otherwise leave this strip sitting on the old screen.
    if (!g_headerDrawn) {
        return;
    }
    // Named separately from a failed fetch: a dropped link is the device's
    // problem to fix and it is already trying, where a failed fetch is the
    // API's and there is nothing to do but wait for the next one. And a link
    // never yet established is the boot case, where "reconnecting" would be a
    // claim about a connection that never existed.
    enum : uint32_t { RECONNECTING, CONNECTING, WAITING, RETRYING, UPDATED, NOTICE };
    bool noticeUp = g_noticeUntil != 0 && (int32_t)(millis() - g_noticeUntil) < 0;
    if (!noticeUp) {
        g_noticeUntil = 0;
    }
    uint32_t state = noticeUp           ? NOTICE
                     : !g_linkUp        ? (g_everSucceeded ? RECONNECTING : CONNECTING)
                     : !g_everSucceeded ? WAITING
                     : !g_pollOk        ? RETRYING
                                        : UPDATED;
    // Only RETRYING and UPDATED quote an age, so only they change with the
    // clock; a notice's slot carries which notice it is instead.
    uint32_t age = (state == RETRYING || state == UPDATED) ? (millis() - g_lastSuccessMs) / 1000
                   : state == NOTICE                       ? g_noticeGen
                                                           : 0;
    uint32_t key = (age << 3) | state;
    drawRangeShown();
    if (key == g_lastStatusKey) {
        return;
    }
    g_lastStatusKey = key;

    String ageStr = age < 60 ? (String(age) + "s") : (String(age / 60) + "m " + String(age % 60) + "s");
    String text;
    uint16_t color = colorGrey;
    switch (state) {
    case RECONNECTING:
        text = "WiFi disconnected - reconnecting";
        color = colorStale;
        break;
    case CONNECTING:
        text = "Connecting to WiFi";
        color = colorStale;
        break;
    case WAITING:
        text = "Waiting for data";
        break;
    case RETRYING:
        text = "Last update " + ageStr + " ago - retrying";
        color = colorStale;
        break;
    case NOTICE:
        text = g_notice;
        color = colorAlert;
        break;
    default:
        text = "Updated " + ageStr + " ago";
        break;
    }

    auto &canvas = screen::canvas();
    int y = TABLE_Y + displayMaxRows() * ROW_HEIGHT;
    canvas.fillRect(TABLE_X, y, STATUS_W, STATUS_H, colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(color);
    canvas.setTextDatum(ML_DATUM);
    canvas.drawString(text, TABLE_X + 10, y + STATUS_H / 2);
    screen::markDirty(TABLE_X, y, STATUS_W, STATUS_H);
    screen::flush();
}

void displaySetHeader(const String &airportCode, const String &provider) {
    if (airportCode == g_airportCode && provider == g_provider) {
        return;
    }
    g_airportCode = airportCode;
    g_provider = provider;
    // The subtitle is only drawn by a full repaint. A filter or location change
    // already causes one by another route, but a failover behind the scenes
    // does not, so ask for it here rather than leaving a stale source on show.
    displayInvalidate();
}

void displaySetFiltered(bool on, int unfiltered) {
    g_unfiltered = unfiltered;
    if (on == g_filterOn) {
        return;
    }
    g_filterOn = on;
    displayInvalidate();  // the funnel lit or not, and the empty table's words
}

void displaySetMuted(bool muted) {
    if (muted == g_muted) {
        return;
    }
    g_muted = muted;
    if (!g_headerDrawn) {
        return;  // nothing on screen to update; the next repaint draws it right
    }
    auto &canvas = screen::canvas();
    canvas.fillRect(MUTE_X, ICON_Y, ICON_SIZE, ICON_SIZE, colorBg);
    drawSpeaker(MUTE_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 8, g_muted,
                g_muted ? colorMutedIcon : colorGrey);
    screen::markDirty(MUTE_X, ICON_Y, ICON_SIZE, ICON_SIZE);
    screen::flush();
}

bool displayTickHighlights() {
    uint32_t now = millis();
    bool any = false;
    for (int r = 0; r < MAX_CACHE_ROWS; r++) {
        for (int c = 0; c < NUM_COLS; c++) {
            if (g_highlightUntil[r][c] == 0 || (int32_t)(now - g_highlightUntil[r][c]) < 0) {
                continue;
            }
            g_highlightUntil[r][c] = 0;
            if (g_lastCellValid[r][c]) {
                drawCell(r, c, g_lastCell[r][c], g_lastCellColor[r][c], false, true);
                any = true;
            }
        }
    }
    if (any) {
        screen::flush();
    }
    return any;
}

void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const std::vector<uint8_t> &isNew) {
    auto &canvas = screen::canvas();

    if (!g_headerDrawn) {
        screen::clear(colorBg);
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(3);
        canvas.setTextDatum(ML_DATUM);
        canvas.setTextColor(colorWhite);
        const char *title = "ADSB Flights";
        canvas.drawString(title, 16, HEADER_MID_Y);
        // Which traffic is being shown - so a table filtered down to the
        // military reads as "nothing about" rather than "something's broken"
        // when it is empty, and one down to the cargo flights says so - and
        // where it is being shown from, which is the only place the location
        // now appears once the header's location button has moved into the
        // settings screen.
        String sub = " - " + shownKinds(false);
        if (g_airportCode.length()) {
            sub += " near " + g_airportCode;
        }
        canvas.setTextColor(colorGrey);
        int subX = 16 + canvas.textWidth(title);
        canvas.drawString(sub, subX, HEADER_MID_Y);

        // Which provider served this, named because it can now change on its
        // own: a table quietly being fed by the fallback should say so rather
        // than leave you guessing. Set a size down from the line it follows,
        // both because it is the least of the three things said here and
        // because at size 3 it reached past the icons at the right edge.
        if (g_provider.length()) {
            int x = subX + canvas.textWidth(sub);
            String via = "  via " + g_provider;
            int size = 2;
            canvas.setTextSize(size);
            // The icons own everything from FILTER_X rightwards. A name long
            // enough to reach them is dropped rather than drawn underneath.
            while (size > 1 && x + canvas.textWidth(via) > FILTER_X - 12) {
                canvas.setTextSize(--size);
            }
            if (x + canvas.textWidth(via) <= FILTER_X - 12) {
                canvas.drawString(via, x, HEADER_MID_Y);
            }
            canvas.setTextSize(3);
        }

        drawFunnel(FILTER_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 10,
                   g_filterOn ? colorHeaderSorted : colorGrey);
        drawReplayIcon(REPLAYS_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 8, colorGrey);
        drawRadarIcon(RADAR_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 6, colorGrey);
        drawSpeaker(MUTE_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 8, g_muted,
                    g_muted ? colorMutedIcon : colorGrey);
        drawGear(COG_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 6, colorGrey);

        drawColumnHeads();
        g_headerDrawn = true;
        g_lastStatusKey = STATUS_NONE;  // cleared along with the rest of the canvas
        g_lastScrollShown = g_lastCountShown = g_lastUnfilteredShown = -1;
    }

    int maxRows = displayMaxRows();
    // The list can have shrunk under a scroll.
    g_count = (int)aircraft.size();
    g_scroll = std::max(0, std::min(g_scroll, g_count - maxRows));

    if (aircraft.empty()) {
        for (String &h : g_rowHex) {
            h = "";
        }
        if (!g_lastWasEmpty) {
            for (int r = 0; r < maxRows; r++) {
                clearRow(r);
            }
            canvas.setFont(&fonts::Font0);
            canvas.setTextColor(colorGrey);
            canvas.setTextSize(2);
            canvas.setTextDatum(ML_DATUM);
            canvas.drawString(g_filterOn && g_unfiltered ? "No aircraft pass the filter" : "No aircraft in range", TABLE_X + 10, TABLE_Y + ROW_HEIGHT / 2);
        }
    } else {
        if (g_lastWasEmpty) {
            // wipe the "No aircraft" message before drawing real rows
            canvas.fillRect(TABLE_X, TABLE_Y, TABLE_W, maxRows * ROW_HEIGHT, colorBg);
            screen::markDirty(TABLE_X, TABLE_Y, TABLE_W, maxRows * ROW_HEIGHT);
        }
        int count = (int)aircraft.size();
        // In order of the column picked: the poll's is nearest first, which
        // is what the default asks for anyway - stable, so ties stay so.
        static std::vector<int> order;
        order.resize(count);
        for (int i = 0; i < count; i++) {
            order[i] = i;
        }
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            SortKey ka = sortKey(aircraft[a], g_sortCol), kb = sortKey(aircraft[b], g_sortCol);
            if (ka.known != kb.known) {
                return ka.known;  // the unknown last, either way
            }
            if (!ka.known) {
                return false;
            }
            int c = ka.text ? ka.text->compareTo(*kb.text) : (ka.number < kb.number ? -1 : ka.number > kb.number ? 1 : 0);
            return g_sortDesc ? c > 0 : c < 0;
        });
        for (int r = 0; r < maxRows; r++) {
            int shown = g_scroll + r;
            if (shown >= count) {
                g_rowHex[r] = "";
                if (g_lastCellValid[r][0]) {
                    clearRow(r);
                }
                continue;
            }
            int i = order[shown];
            const Aircraft &ac = aircraft[i];
            g_rowHex[r] = ac.hex;
            bool rowIsNew = (i < (int)isNew.size()) && isNew[i];
            // An alert outlasts the one refresh a new row is green for, so it
            // wins: the row stays marked for as long as the aircraft is about.
            uint16_t color = (ac.alert & ALERT_EMERGENCY) ? colorEmergency
                             : ac.alert                   ? colorAlert
                             : rowIsNew                   ? colorNew
                                                          : colorWhite;

            String dist = ac.hasDist ? (String((int)lroundf(ac.distNm)) + "nm") : "?";
            String alt = formatAltitude(ac.altStr);
            String speedKt = (ac.speedStr == "?") ? String("?") : (ac.speedStr + "kt");
            String values[NUM_COLS] = {ac.callsign, ac.type, alt, speedKt, dist, formatHeading(ac), ac.status};
            // Only the callsign flashes, between red and an ordinary row's white.
            // A cargo operator's is blue, where nothing else colours it.
            AlertFlash flash = alertFlash(ac.hex);
            uint16_t callsignColor = flash == AlertFlash::RED ? colorEmergency
                                     : flash == AlertFlash::OFF ? colorWhite
                                     : color == colorWhite && isCargo(ac.callsign) ? colorCargo
                                                                                   : color;

            for (int c = 0; c < NUM_COLS; c++) {
                uint16_t cellColor = (c == 0) ? callsignColor : color;
                if (g_lastCellValid[r][c] && g_lastCell[r][c] == values[c] && g_lastCellColor[r][c] == cellColor) {
                    continue;  // unchanged - skip the redraw entirely
                }
                // Shade genuine changes only. A cell with no cached value is
                // being painted for the first time or repainted after another
                // screen covered the canvas, and one whose value is the same
                // has only changed colour - none of which is news.
                bool changed = g_lastCellValid[r][c] && g_lastCell[r][c] != values[c] && g_showRefresh;
                drawCell(r, c, values[c], cellColor, changed, g_lastCellValid[r][c]);
                uint32_t until = millis() + CELL_HIGHLIGHT_MS;
                if (until == 0) {
                    until = 1;  // 0 is the "not shaded" sentinel
                }
                g_highlightUntil[r][c] = changed ? until : 0;
                g_lastCell[r][c] = values[c];
                g_lastCellColor[r][c] = cellColor;
                g_lastCellValid[r][c] = true;
            }
        }
    }
    g_lastWasEmpty = aircraft.empty();

    screen::flush();
}

void displayShowNotice(const String &text) {
    g_notice = text;
    g_noticeUntil = (millis() + NOTICE_MS) | 1;  // 0 means no notice
    g_noticeGen++;
    displayTickStatus();
}
