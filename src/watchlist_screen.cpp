#include "watchlist_screen.h"

#include <M5Unified.h>

#include "alerts.h"
#include "config.h"
#include "keyboard.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int SAVE_W = 124;
constexpr int SAVE_X = BACK_X - 12 - SAVE_W;
constexpr int BOX_X = 16, BOX_Y = 64, BOX_W = 1248, BOX_H = 92;
constexpr int BOX_PAD = 12, LINE_H = 26;
constexpr int HINT_Y = 180;
constexpr int ERROR_Y = BOX_Y + BOX_H + 6, ERROR_H = 18;

String g_text;
bool g_errorShown = false;

uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorSave, colorBorder, colorError;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorGrey = M5.Display.color565(0xBB, 0xBB, 0xBB);
    colorDim = M5.Display.color565(0x88, 0x91, 0x9B);
    colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorSave = M5.Display.color565(0x27, 0xAE, 0x60);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorError = M5.Display.color565(0xFF, 0x4D, 0x4D);
    colorsReady = true;
}

// Shown in capitals whatever case it was typed in, since that is how every
// one of the things it matches is written - and matched that way too.
void drawBox() {
    auto &canvas = screen::canvas();
    canvas.fillRect(BOX_X, BOX_Y, BOX_W, BOX_H, colorBg);
    canvas.drawRect(BOX_X, BOX_Y, BOX_W, BOX_H, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(TL_DATUM);
    String shown = g_text + "_";
    shown.toUpperCase();
    // Wrapped by character: the font is fixed-width, so a line is a known count.
    int perLine = (BOX_W - 2 * BOX_PAD) / canvas.textWidth("M");
    for (int line = 0; line * perLine < (int)shown.length() && line < 3; line++) {
        canvas.drawString(shown.substring(line * perLine, (line + 1) * perLine), BOX_X + BOX_PAD,
                          BOX_Y + BOX_PAD + line * LINE_H);
    }
    screen::markDirty(BOX_X, BOX_Y, BOX_W, BOX_H);
}

// Under the box, between it and the hints: what stopped a save, or nothing.
void drawError(const String &text) {
    auto &canvas = screen::canvas();
    canvas.fillRect(BOX_X, ERROR_Y, BOX_W, ERROR_H, colorBg);
    if (text.length()) {
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(2);
        canvas.setTextColor(colorError);
        canvas.setTextDatum(TL_DATUM);
        canvas.drawString(text, BOX_X, ERROR_Y);
    }
    g_errorShown = text.length() > 0;
    screen::markDirty(BOX_X, ERROR_Y, BOX_W, ERROR_H);
}

// A save that would keep entries nothing can match is refused, saying which.
WatchlistAction trySave() {
    String problem = watchlistProblem(g_text);
    if (problem.length()) {
        drawError(problem);
        screen::flush();
        return WatchlistAction::NONE;
    }
    return WatchlistAction::SAVE;
}

}  // namespace

void watchlistScreenSet(const String &text) {
    g_errorShown = false;
    g_text = text;
    keyboard::reset();
}

String watchlistScreenText() {
    String t = g_text;
    t.toUpperCase();
    t.trim();
    return t;
}

void watchlistScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Watchlist", 16, 12);

    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    canvas.fillRoundRect(SAVE_X, BACK_Y, SAVE_W, BACK_H, 6, colorSave);
    canvas.drawString("Save", SAVE_X + SAVE_W / 2, BACK_Y + BACK_H / 2);

    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(colorGrey);
    canvas.drawString("Separate entries with spaces or commas. Each one is matched against:", 16, HINT_Y);
    canvas.setTextColor(colorDim);
    canvas.drawString("  callsigns  BAW123        airline codes  RYR        registrations  G-ABCD", 16, HINT_Y + 34);
    canvas.drawString("  type codes  A388 B748    ICAO hex codes  4CA2D1", 16, HINT_Y + 62);
    canvas.drawString("A three-letter airline code matches every flight number that starts with it.", 16,
                      HINT_Y + 104);

    drawBox();
    keyboard::draw();
    screen::flush();
}

WatchlistAction watchlistScreenHandleTouch(int x, int y) {
    if (y >= BACK_Y && y < BACK_Y + BACK_H) {
        if (x >= BACK_X && x < BACK_X + BACK_W) {
            return WatchlistAction::CANCEL;
        }
        if (x >= SAVE_X && x < SAVE_X + SAVE_W) {
            return trySave();
        }
    }
    switch (keyboard::handleTouch(x, y, g_text, WATCHLIST_MAX_LEN)) {
    case keyboard::Result::EDITED:
        if (g_errorShown) {
            drawError("");
        }
        drawBox();
        screen::flush();
        break;
    case keyboard::Result::SUBMIT:
        return trySave();
    case keyboard::Result::NONE:
        break;
    }
    return WatchlistAction::NONE;
}
