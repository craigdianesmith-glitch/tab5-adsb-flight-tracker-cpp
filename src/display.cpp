#include "display.h"

#include <math.h>

#include "config.h"
#include "screen.h"

namespace {

struct Column {
    const char *title;
    int width;
};

const Column COLUMNS[] = {
    {"FLIGHT", 180}, {"TYPE", 150}, {"ALT", 170}, {"SPD", 160}, {"DIST", 160}, {"HDG", 150}, {"STATUS", 280},
};
constexpr int NUM_COLS = 7;
// The only column drawn differently - it carries an icon ahead of its text.
constexpr int STATUS_COL = NUM_COLS - 1;
constexpr int TABLE_X = 8;
constexpr int TABLE_W = 1264;
constexpr int HEADER_Y = 68;
constexpr int HEADER_H = 54;
constexpr int TABLE_Y = HEADER_Y + HEADER_H;
constexpr int ROW_HEIGHT = 52;
constexpr int MAX_CACHE_ROWS = 16;  // generous upper bound on any screen size we'd realistically run at

uint16_t colorBg, colorWhite, colorHeaderBg, colorHeaderText, colorBorder, colorGrey, colorNew,
    colorChanged, colorStale, colorMutedIcon;

constexpr int STATUS_ICON_SIZE = 18;

// The header's controls sit together at the right edge, leaving the whole
// left of the bar to the title line. Laid out from the right so the cog - the
// one that leads everywhere else now - lands in the corner.
constexpr int ICON_Y = 6, ICON_SIZE = 52, ICON_GAP = 12;
constexpr int COG_X = TABLE_X + TABLE_W - ICON_SIZE;
constexpr int MUTE_X = COG_X - ICON_GAP - ICON_SIZE;
constexpr int RADAR_X = MUTE_X - ICON_GAP - ICON_SIZE;

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
bool g_military = false;
String g_airportCode;
bool g_muted = false;

// Poll state, for the status line under the table.
bool g_linkUp = true;
bool g_pollOk = true;
bool g_everSucceeded = false;
uint32_t g_lastSuccessMs = 0;
String g_lastStatus = "\x01";  // sentinel that can never equal a real status
uint16_t g_lastStatusColor = 0;

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

void drawCell(int r, int c, const String &value, uint16_t color, bool highlight) {
    auto &canvas = screen::canvas();
    int w = COLUMNS[c].width - 4;
    int y = TABLE_Y + r * ROW_HEIGHT;
    int h = ROW_HEIGHT - 4;
    canvas.fillRect(colX[c], y, w, h, highlight ? colorChanged : colorBg);
    canvas.drawRect(colX[c], y, w, h, colorBorder);
    canvas.setFont(&fonts::FreeSans9pt7b);
    canvas.setTextColor(color);
    canvas.setTextDatum(ML_DATUM);
    int midY = y + h / 2;
    if (c == STATUS_COL) {
        int iconCx = colX[c] + 10 + STATUS_ICON_SIZE / 2;
        drawStatusIcon(iconCx, midY, value, color);
        canvas.drawString(value, colX[c] + 10 + STATUS_ICON_SIZE + 8, midY);
    } else {
        canvas.drawString(value, colX[c] + 10, midY);
    }
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
    colorHeaderText = M5.Display.color565(0x10, 0x20, 0x10);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorGrey = M5.Display.color565(0x88, 0x88, 0x88);
    colorNew = M5.Display.color565(0x1B, 0x7A, 0x1B);
    colorChanged = M5.Display.color565(0x1E, 0x2E, 0x40);
    colorStale = M5.Display.color565(0xF3, 0x9C, 0x12);  // amber: on screen but not current
    colorMutedIcon = M5.Display.color565(0x5A, 0x60, 0x68);  // dimmer than the live icons

    int x = TABLE_X;
    for (int i = 0; i < NUM_COLS; i++) {
        colX[i] = x;
        x += COLUMNS[i].width;
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

bool displayHitMute(int x, int y) { return hitIcon(MUTE_X, x, y); }

bool displayHitRow(int x, int y, int &outRow) {
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
    outRow = row;
    return true;
}

void displayInvalidate() {
    // Another screen has been drawing on the shared canvas, so nothing cached
    // here is on it any more. Clearing g_headerDrawn makes the next render
    // clear the canvas and repaint from scratch.
    g_headerDrawn = false;
    g_lastWasEmpty = false;
    g_lastStatus = "\x01";
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

void displayTickStatus() {
    // The status line belongs to the table, so don't paint it over whatever is
    // there until the table itself has been drawn: returning from the radar
    // invalidates the cache, and a render that loses the race for the data
    // mutex would otherwise leave this strip sitting on the old screen.
    if (!g_headerDrawn) {
        return;
    }
    uint32_t age = (millis() - g_lastSuccessMs) / 1000;
    String ageStr = age < 60 ? (String(age) + "s") : (String(age / 60) + "m " + String(age % 60) + "s");

    String text;
    uint16_t color = colorGrey;
    if (!g_linkUp) {
        // Named separately from a failed fetch: a dropped link is the device's
        // problem to fix and it is already trying, where a failed fetch is the
        // API's and there is nothing to do but wait for the next one. And a
        // link never yet established is the boot case, where "reconnecting"
        // would be a claim about a connection that never existed.
        text = g_everSucceeded ? "WiFi disconnected - reconnecting" : "Connecting to WiFi";
        color = colorStale;
    } else if (!g_everSucceeded) {
        text = "Waiting for data";
    } else if (!g_pollOk) {
        text = "Last update " + ageStr + " ago - retrying";
        color = colorStale;
    } else {
        text = "Updated " + ageStr + " ago";
    }

    if (text == g_lastStatus && color == g_lastStatusColor) {
        return;
    }
    g_lastStatus = text;
    g_lastStatusColor = color;

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

void displaySetHeader(bool military, const String &airportCode) {
    g_military = military;
    g_airportCode = airportCode;
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

void displayTickHighlights() {
    uint32_t now = millis();
    bool any = false;
    for (int r = 0; r < MAX_CACHE_ROWS; r++) {
        for (int c = 0; c < NUM_COLS; c++) {
            if (g_highlightUntil[r][c] == 0 || (int32_t)(now - g_highlightUntil[r][c]) < 0) {
                continue;
            }
            g_highlightUntil[r][c] = 0;
            if (g_lastCellValid[r][c]) {
                drawCell(r, c, g_lastCell[r][c], g_lastCellColor[r][c], false);
                any = true;
            }
        }
    }
    if (any) {
        screen::flush();
    }
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
        // Which traffic is being shown - so an empty table in military mode
        // reads as "nothing about" rather than "something's broken" - and
        // where it is being shown from, which is the only place the location
        // now appears once the header's location button has moved into the
        // settings screen.
        String sub = g_military ? " - Military" : " - Civilian";
        if (g_airportCode.length()) {
            sub += " (Nearest airport " + g_airportCode + ")";
        }
        canvas.setTextColor(colorGrey);
        canvas.drawString(sub, 16 + canvas.textWidth(title), HEADER_MID_Y);

        drawRadarIcon(RADAR_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 6, colorGrey);
        drawSpeaker(MUTE_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 8, g_muted,
                    g_muted ? colorMutedIcon : colorGrey);
        drawGear(COG_X + ICON_SIZE / 2, ICON_Y + ICON_SIZE / 2, ICON_SIZE / 2 - 6, colorGrey);

        int x = TABLE_X;
        for (int i = 0; i < NUM_COLS; i++) {
            int w = COLUMNS[i].width - 4;
            canvas.fillRect(x, HEADER_Y, w, HEADER_H, colorHeaderBg);
            canvas.setTextColor(colorHeaderText);
            canvas.setTextSize(2);
            canvas.setTextDatum(MC_DATUM);
            canvas.drawString(COLUMNS[i].title, x + w / 2, HEADER_Y + HEADER_H / 2);
            x += COLUMNS[i].width;
        }
        g_headerDrawn = true;
        g_lastStatus = "\x01";  // cleared along with the rest of the canvas
    }

    int maxRows = displayMaxRows();

    if (aircraft.empty()) {
        if (!g_lastWasEmpty) {
            for (int r = 0; r < maxRows; r++) {
                clearRow(r);
            }
            canvas.setFont(&fonts::Font0);
            canvas.setTextColor(colorGrey);
            canvas.setTextSize(2);
            canvas.setTextDatum(ML_DATUM);
            canvas.drawString("No aircraft in range", TABLE_X + 10, TABLE_Y + ROW_HEIGHT / 2);
        }
    } else {
        if (g_lastWasEmpty) {
            // wipe the "No aircraft" message before drawing real rows
            canvas.fillRect(TABLE_X, TABLE_Y, TABLE_W, maxRows * ROW_HEIGHT, colorBg);
            screen::markDirty(TABLE_X, TABLE_Y, TABLE_W, maxRows * ROW_HEIGHT);
        }
        int count = (int)aircraft.size();
        for (int r = 0; r < maxRows; r++) {
            if (r >= count) {
                if (g_lastCellValid[r][0]) {
                    clearRow(r);
                }
                continue;
            }
            const Aircraft &ac = aircraft[r];
            bool rowIsNew = (r < (int)isNew.size()) && isNew[r];
            uint16_t color = rowIsNew ? colorNew : colorWhite;

            String dist = ac.hasDist ? (String((int)lroundf(ac.distNm)) + "nm") : "?";
            String alt = formatAltitude(ac.altStr);
            String speedKt = (ac.speedStr == "?") ? String("?") : (ac.speedStr + "kt");
            String values[NUM_COLS] = {ac.callsign, ac.type, alt, speedKt, dist, formatHeading(ac), ac.status};

            for (int c = 0; c < NUM_COLS; c++) {
                if (g_lastCellValid[r][c] && g_lastCell[r][c] == values[c] && g_lastCellColor[r][c] == color) {
                    continue;  // unchanged - skip the redraw entirely
                }
                // Shade genuine changes only. A cell with no cached value is
                // being painted for the first time or repainted after another
                // screen covered the canvas, neither of which is news.
                bool changed = g_lastCellValid[r][c] && g_showRefresh;
                drawCell(r, c, values[c], color, changed);
                uint32_t until = millis() + CELL_HIGHLIGHT_MS;
                if (until == 0) {
                    until = 1;  // 0 is the "not shaded" sentinel
                }
                g_highlightUntil[r][c] = changed ? until : 0;
                g_lastCell[r][c] = values[c];
                g_lastCellColor[r][c] = color;
                g_lastCellValid[r][c] = true;
            }
        }
    }
    g_lastWasEmpty = aircraft.empty();

    screen::flush();
}
