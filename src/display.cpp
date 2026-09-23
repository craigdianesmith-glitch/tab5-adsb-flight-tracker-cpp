#include "display.h"

#include <math.h>

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

uint16_t colorBg, colorWhite, colorHeaderBg, colorHeaderText, colorButtonBg, colorBorder, colorGrey, colorNew;

constexpr int ICON_SIZE = 18;
int colX[NUM_COLS];

// Render cache: what's currently on screen, so a redraw only touches cells
// whose value or color actually changed instead of repainting everything.
bool g_headerDrawn = false;
String g_lastLabel = "\x01";  // sentinel that can never equal a real label
bool g_lastWasEmpty = false;
String g_lastCell[MAX_CACHE_ROWS][NUM_COLS];
uint16_t g_lastCellColor[MAX_CACHE_ROWS][NUM_COLS];
bool g_lastCellValid[MAX_CACHE_ROWS][NUM_COLS] = {};

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

void drawCell(int r, int c, const String &value, uint16_t color) {
    auto &canvas = screen::canvas();
    int w = COLUMNS[c].width - 4;
    int y = TABLE_Y + r * ROW_HEIGHT;
    int h = ROW_HEIGHT - 4;
    canvas.fillRect(colX[c], y, w, h, colorBg);
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

    int x = TABLE_X;
    for (int i = 0; i < NUM_COLS; i++) {
        colX[i] = x;
        x += COLUMNS[i].width;
    }
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
        }
    }
}

void displayRenderAircraft(const std::vector<Aircraft> &aircraft, const String &locationLabel,
                            const std::vector<uint8_t> &isNew) {
    auto &canvas = screen::canvas();
    int screenH = canvas.height();

    if (!g_headerDrawn) {
        screen::clear(colorBg);
        canvas.setFont(&fonts::Font0);
        canvas.setTextColor(colorWhite);
        canvas.setTextSize(3);
        canvas.setTextDatum(TL_DATUM);
        canvas.drawString("ADSB Flight Display", 16, 10);

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

    int maxRows = (screenH - TABLE_Y - 8) / ROW_HEIGHT;
    if (maxRows > MAX_CACHE_ROWS) {
        maxRows = MAX_CACHE_ROWS;
    }

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
                drawCell(r, c, values[c], color);
                g_lastCell[r][c] = values[c];
                g_lastCellColor[r][c] = color;
                g_lastCellValid[r][c] = true;
            }
        }
    }
    g_lastWasEmpty = aircraft.empty();

    screen::flush();
}
