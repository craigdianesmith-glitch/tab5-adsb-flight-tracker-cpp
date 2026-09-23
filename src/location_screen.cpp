#include "location_screen.h"

#include <M5Unified.h>
#include <vector>

#include "geocode.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int BOX_X = 16, BOX_Y = 64, BOX_W = 700, BOX_H = 50;
constexpr int SEARCH_BTN_X = 726, SEARCH_BTN_Y = 64, SEARCH_BTN_W = 140, SEARCH_BTN_H = 50;
constexpr int STATUS_X = 16, STATUS_Y = 126, STATUS_W = 700, STATUS_H = 24;
constexpr int LIST_X = 16, LIST_Y = 156, LIST_W = 1248, ROW_H = 42, MAX_RESULT_ROWS = 5;
constexpr int SET_BTN_X = 16, SET_BTN_Y = LIST_Y + MAX_RESULT_ROWS * ROW_H + 16, SET_BTN_W = 280, SET_BTN_H = 50;

constexpr int GRID_X = 16, GRID_Y = 440, CELL_W = 124, CELL_H = 62, GRID_COLS = 10, GRID_ROWS = 4;

const char *KEY_GRID[GRID_ROWS][GRID_COLS] = {
    {"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"},
    {"", "A", "S", "D", "F", "G", "H", "J", "K", "L"},
    {"", "Z", "X", "C", "V", "B", "N", "M", "BKSP", "BKSP"},
    {"SPACE", "SPACE", "SPACE", "SPACE", "SPACE", "SPACE", "SPACE", "SEARCH", "SEARCH", "SEARCH"},
};

String g_query;
std::vector<GeoResult> g_results;
int g_selected = -1;
String g_status;

uint16_t colorBg, colorWhite, colorGrey, colorBtnBg, colorSearchBtn, colorSetBtn, colorKeyBg, colorBorder,
    colorSelected;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorGrey = M5.Display.color565(0xBB, 0xBB, 0xBB);
    colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorSearchBtn = M5.Display.color565(0x29, 0x80, 0xB9);
    colorSetBtn = M5.Display.color565(0x27, 0xAE, 0x60);
    colorKeyBg = M5.Display.color565(0x30, 0x36, 0x3D);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorSelected = M5.Display.color565(0x1B, 0x7A, 0x1B);
    colorsReady = true;
}

// Title, Back, the Search button and the keyboard: none of it changes while
// this screen is up, so it's painted once on entry and then left alone. Typing
// only repaints the text box, which is what makes a keypress cheap.
void drawChrome() {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);

    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Set location", 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    canvas.fillRoundRect(SEARCH_BTN_X, SEARCH_BTN_Y, SEARCH_BTN_W, SEARCH_BTN_H, 6, colorSearchBtn);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Search", SEARCH_BTN_X + SEARCH_BTN_W / 2, SEARCH_BTN_Y + SEARCH_BTN_H / 2);

    // Keyboard - draw merged rects for the multi-cell keys (BKSP/SPACE/SEARCH)
    // rather than repeating per cell, using the known fixed layout.
    auto drawKey = [&](int col0, int row, int colspan, const char *label) {
        int x = GRID_X + col0 * CELL_W;
        int y = GRID_Y + row * CELL_H;
        int w = colspan * CELL_W - 6;
        int h = CELL_H - 6;
        canvas.fillRoundRect(x, y, w, h, 4, colorKeyBg);
        canvas.setTextColor(colorWhite);
        canvas.setTextSize(2);
        canvas.setTextDatum(MC_DATUM);
        canvas.drawString(label, x + w / 2, y + h / 2);
    };
    for (int c = 0; c < 10; c++) drawKey(c, 0, 1, KEY_GRID[0][c]);
    for (int c = 1; c < 10; c++) drawKey(c, 1, 1, KEY_GRID[1][c]);
    for (int c = 1; c < 8; c++) drawKey(c, 2, 1, KEY_GRID[2][c]);
    drawKey(8, 2, 2, "BKSP");
    drawKey(0, 3, 7, "SPACE");
    drawKey(7, 3, 3, "SEARCH");
}

void drawQuery() {
    auto &canvas = screen::canvas();
    canvas.fillRect(BOX_X, BOX_Y, BOX_W, BOX_H, colorBg);
    canvas.drawRect(BOX_X, BOX_Y, BOX_W, BOX_H, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(ML_DATUM);
    canvas.drawString(g_query, BOX_X + 10, BOX_Y + BOX_H / 2);
    screen::markDirty(BOX_X, BOX_Y, BOX_W, BOX_H);
}

void drawResults() {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);

    canvas.fillRect(STATUS_X, STATUS_Y, STATUS_W, STATUS_H, colorBg);
    if (g_status.length()) {
        canvas.setTextColor(colorGrey);
        canvas.setTextDatum(TL_DATUM);
        canvas.drawString(g_status, STATUS_X, STATUS_Y);
    }
    screen::markDirty(STATUS_X, STATUS_Y, STATUS_W, STATUS_H);

    for (int i = 0; i < MAX_RESULT_ROWS; i++) {
        int y = LIST_Y + i * ROW_H;
        uint16_t bg = (i == g_selected) ? colorSelected : colorKeyBg;
        canvas.fillRect(LIST_X, y, LIST_W, ROW_H - 4, bg);
        canvas.drawRect(LIST_X, y, LIST_W, ROW_H - 4, colorBorder);
        if (i < (int)g_results.size()) {
            canvas.setTextColor(colorWhite);
            canvas.setTextDatum(ML_DATUM);
            canvas.drawString(g_results[i].label, LIST_X + 10, y + (ROW_H - 4) / 2);
        }
    }
    screen::markDirty(LIST_X, LIST_Y, LIST_W, MAX_RESULT_ROWS * ROW_H);

    canvas.fillRect(SET_BTN_X, SET_BTN_Y, SET_BTN_W, SET_BTN_H, colorBg);
    if (g_selected >= 0) {
        canvas.fillRoundRect(SET_BTN_X, SET_BTN_Y, SET_BTN_W, SET_BTN_H, 6, colorSetBtn);
        canvas.setTextColor(colorWhite);
        canvas.setTextDatum(MC_DATUM);
        canvas.drawString("Set location", SET_BTN_X + SET_BTN_W / 2, SET_BTN_Y + SET_BTN_H / 2);
    }
    screen::markDirty(SET_BTN_X, SET_BTN_Y, SET_BTN_W, SET_BTN_H);
}

void doSearch() {
    if (g_query.length() == 0) {
        return;
    }
    g_status = "Searching...";
    drawResults();
    screen::flush();

    std::vector<GeoResult> results;
    if (geocodeSearch(g_query, results) && !results.empty()) {
        g_results = results;
        g_status = "";
    } else {
        g_results.clear();
        g_status = "No matches";
    }
    g_selected = -1;
}

}  // namespace

void locationScreenReset() {
    g_query = "";
    g_results.clear();
    g_selected = -1;
    g_status = "";
}

void locationScreenDraw() {
    ensureColors();
    screen::clear(colorBg);
    drawChrome();
    drawQuery();
    drawResults();
    screen::flush();
}

LocationAction locationScreenHandleTouch(int x, int y, double &outLat, double &outLon, String &outLabel) {
    if (x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return LocationAction::BACK;
    }
    if (x >= SEARCH_BTN_X && x < SEARCH_BTN_X + SEARCH_BTN_W && y >= SEARCH_BTN_Y && y < SEARCH_BTN_Y + SEARCH_BTN_H) {
        doSearch();
        drawResults();
        screen::flush();
        return LocationAction::NONE;
    }
    if (g_selected >= 0 && x >= SET_BTN_X && x < SET_BTN_X + SET_BTN_W && y >= SET_BTN_Y && y < SET_BTN_Y + SET_BTN_H) {
        outLat = g_results[g_selected].lat;
        outLon = g_results[g_selected].lon;
        outLabel = g_results[g_selected].label;
        return LocationAction::LOCATION_SET;
    }
    if (y >= LIST_Y && y < LIST_Y + MAX_RESULT_ROWS * ROW_H && x >= LIST_X && x < LIST_X + LIST_W) {
        int row = (y - LIST_Y) / ROW_H;
        if (row < (int)g_results.size()) {
            g_selected = row;
            drawResults();
            screen::flush();
        }
        return LocationAction::NONE;
    }
    if (y >= GRID_Y && y < GRID_Y + GRID_ROWS * CELL_H && x >= GRID_X && x < GRID_X + GRID_COLS * CELL_W) {
        int col = (x - GRID_X) / CELL_W;
        int row = (y - GRID_Y) / CELL_H;
        const char *key = KEY_GRID[row][col];
        if (key[0] == '\0') {
            return LocationAction::NONE;  // unused cell
        }
        if (strcmp(key, "SPACE") == 0) {
            g_query += " ";
        } else if (strcmp(key, "BKSP") == 0) {
            if (g_query.length()) g_query.remove(g_query.length() - 1);
        } else if (strcmp(key, "SEARCH") == 0) {
            doSearch();
            drawResults();
            screen::flush();
            return LocationAction::NONE;
        } else {
            g_query += key;
        }
        // Only the text box changed - repaint and push just that.
        drawQuery();
        screen::flush();
    }
    return LocationAction::NONE;
}
