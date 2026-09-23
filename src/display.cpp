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
    {"FLIGHT", 240}, {"TYPE", 150}, {"ALT", 220}, {"SPD", 190}, {"DIST", 190}, {"STATUS", 260},
};
constexpr int NUM_COLS = 6;
constexpr int TABLE_X = 8;
constexpr int TABLE_W = 1264;
constexpr int HEADER_Y = 68;
constexpr int HEADER_H = 54;
constexpr int TABLE_Y = HEADER_Y + HEADER_H;
constexpr int ROW_HEIGHT = 52;
constexpr int MAX_CACHE_ROWS = 16;  // generous upper bound on any screen size we'd realistically run at

uint16_t colorBg, colorWhite, colorHeaderBg, colorHeaderText, colorButtonBg, colorBorder, colorGrey, colorNew,
    colorChanged;

constexpr int ICON_SIZE = 18;

// Settings cog, centred in the gap between the title and the location button.
constexpr int COG_X = 452, COG_Y = 6, COG_SIZE = 52;
int colX[NUM_COLS];

// Render cache: what's currently on screen, so a redraw only touches cells
// whose value or color actually changed instead of repainting everything.
bool g_headerDrawn = false;
String g_lastLabel = "\x01";  // sentinel that can never equal a real label
bool g_lastWasEmpty = false;
String g_lastCell[MAX_CACHE_ROWS][NUM_COLS];
uint16_t g_lastCellColor[MAX_CACHE_ROWS][NUM_COLS];
bool g_lastCellValid[MAX_CACHE_ROWS][NUM_COLS] = {};
// millis() at which a cell's shading lapses; 0 means it isn't shaded.
uint32_t g_highlightUntil[MAX_CACHE_ROWS][NUM_COLS] = {};
bool g_showRefresh = true;

// Drawn geometric icons rather than a hand-authored bitmap: precise and
// reliable without needing to eyeball pixel arrays on real hardware.
void drawStatusIcon(int cx, int cy, const String &status, uint16_t color) {
    auto &canvas = screen::canvas();
    int half = ICON_SIZE / 2;
    if (status == "CLIMB") {
        canvas.fillTriangle(cx, cy - half, cx - half, cy + half, cx + half, cy + half, color);
    } else if (status == "DESCEND") {
        canvas.fillTriangle(cx, cy + half, cx - half, cy - half, cx + half, cy - half, color);
    } else if (status == "LEVEL") {
        canvas.fillRect(cx - half, cy - 3, ICON_SIZE, 6, color);
    } else if (status == "TAXI") {
        canvas.fillCircle(cx, cy, half, color);
    } else if (status == "GROUND") {
        canvas.drawRect(cx - half, cy - half, ICON_SIZE, ICON_SIZE, color);
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
    if (c == 5) {
        int iconCx = colX[c] + 10 + ICON_SIZE / 2;
        drawStatusIcon(iconCx, midY, value, color);
        canvas.drawString(value, colX[c] + 10 + ICON_SIZE + 8, midY);
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
    colorButtonBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorGrey = M5.Display.color565(0x88, 0x88, 0x88);
    colorNew = M5.Display.color565(0x1B, 0x7A, 0x1B);
    colorChanged = M5.Display.color565(0x1E, 0x2E, 0x40);

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

bool displayHitCog(int x, int y) {
    return x >= COG_X && x < COG_X + COG_SIZE && y >= COG_Y && y < COG_Y + COG_SIZE;
}

void displayInvalidate() {
    // Another screen has been drawing on the shared canvas, so nothing cached
    // here is on it any more. Clearing g_headerDrawn makes the next render
    // clear the canvas and repaint from scratch.
    g_headerDrawn = false;
    g_lastLabel = "\x01";
    g_lastWasEmpty = false;
    for (int r = 0; r < MAX_CACHE_ROWS; r++) {
        for (int c = 0; c < NUM_COLS; c++) {
            g_lastCellValid[r][c] = false;
            g_highlightUntil[r][c] = 0;
        }
    }
}

void displaySetShowRefresh(bool enabled) { g_showRefresh = enabled; }

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

void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const String &locationLabel,
                            const std::vector<uint8_t> &isNew) {
    auto &canvas = screen::canvas();

    if (!g_headerDrawn) {
        screen::clear(colorBg);
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(colorWhite);
        canvas.setTextSize(3);
        canvas.setTextDatum(TL_DATUM);
        canvas.drawString("ADSB Flight Display", 16, 10);

        drawGear(COG_X + COG_SIZE / 2, COG_Y + COG_SIZE / 2, COG_SIZE / 2 - 6, colorGrey);

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
    }

    if (locationLabel != g_lastLabel) {
        canvas.setFont(&fonts::Font0);
        int btnX = 600, btnY = 6, btnW = 664, btnH = 52;
        canvas.fillRoundRect(btnX, btnY, btnW, btnH, 6, colorButtonBg);
        canvas.setTextColor(colorWhite);
        canvas.setTextDatum(MC_DATUM);
        int maxTextW = btnW - 20;
        int size = 2;
        canvas.setTextSize(size);
        while (size > 1 && canvas.textWidth(locationLabel) > maxTextW) {
            size--;
            canvas.setTextSize(size);
        }
        canvas.drawString(locationLabel, btnX + btnW / 2, btnY + btnH / 2);
        screen::markDirty(btnX, btnY, btnW, btnH);
        g_lastLabel = locationLabel;
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
            String alt = (ac.altStr == "GND") ? ac.altStr : (ac.altStr + "ft");
            String speedKt = ac.speedStr + "kt";
            String values[NUM_COLS] = {ac.callsign, ac.type, alt, speedKt, dist, ac.status};

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
