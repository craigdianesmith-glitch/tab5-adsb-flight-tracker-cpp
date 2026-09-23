#include "keyboard.h"

#include <M5Unified.h>

#include "screen.h"

namespace keyboard {
namespace {

constexpr int GRID_X = 16;
constexpr int CELL_W = 124;
constexpr int CELL_H = 56;
constexpr int COLS = 10;
constexpr int ROWS = 5;

// Multi-cell keys are described by the span, so one cell's label repeated
// across its columns is what makes a wide key: the draw pass merges a run of
// identical labels into a single rounded rect.
const char *LETTERS[ROWS][COLS] = {
    {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"},
    {"q", "w", "e", "r", "t", "y", "u", "i", "o", "p"},
    {"a", "s", "d", "f", "g", "h", "j", "k", "l", "BKSP"},
    {"SHIFT", "z", "x", "c", "v", "b", "n", "m", ".", "-"},
    {"?#", "SPACE", "SPACE", "SPACE", "SPACE", "SPACE", "SPACE", "OK", "OK", "OK"},
};

const char *SYMBOLS[ROWS][COLS] = {
    {"!", "@", "#", "$", "%", "^", "&", "*", "(", ")"},
    {"-", "_", "=", "+", "[", "]", "{", "}", ";", ":"},
    {"'", "\"", "\\", "|", "/", "?", "<", ">", "BKSP", "BKSP"},
    {",", ".", "~", "`", "", "", "", "", "", ""},
    {"abc", "SPACE", "SPACE", "SPACE", "SPACE", "SPACE", "SPACE", "OK", "OK", "OK"},
};

enum class Mode { LOWER, UPPER, SYMBOL };
Mode g_mode = Mode::LOWER;

bool colorsReady = false;
uint16_t colorBg, colorKeyBg, colorKeyActive, colorWide, colorOk, colorWhite;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorKeyBg = M5.Display.color565(0x30, 0x36, 0x3D);
    colorKeyActive = M5.Display.color565(0x1B, 0x7A, 0x1B);
    colorWide = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorOk = M5.Display.color565(0x27, 0xAE, 0x60);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorsReady = true;
}

const char *keyAt(int row, int col) {
    return (g_mode == Mode::SYMBOL) ? SYMBOLS[row][col] : LETTERS[row][col];
}

// The label a key shows, which is the only place case is applied.
String labelAt(int row, int col) {
    const char *k = keyAt(row, col);
    String s(k);
    if (g_mode == Mode::UPPER && s.length() == 1 && isAlpha(s[0])) {
        s.toUpperCase();
    }
    return s;
}

uint16_t keyColor(const char *k) {
    if (strcmp(k, "OK") == 0) {
        return colorOk;
    }
    if (strcmp(k, "SHIFT") == 0 && g_mode == Mode::UPPER) {
        return colorKeyActive;
    }
    if (strcmp(k, "SPACE") == 0 || strcmp(k, "BKSP") == 0 || strcmp(k, "SHIFT") == 0 || strcmp(k, "?#") == 0 ||
        strcmp(k, "abc") == 0) {
        return colorWide;
    }
    return colorKeyBg;
}

}  // namespace

void reset() { g_mode = Mode::LOWER; }

bool contains(int x, int y) {
    return y >= TOP_Y && y < TOP_Y + HEIGHT && x >= GRID_X && x < GRID_X + COLS * CELL_W;
}

void draw() {
    ensureColors();
    auto &canvas = screen::canvas();
    canvas.fillRect(0, TOP_Y, canvas.width(), HEIGHT, colorBg);
    canvas.setFont(&fonts::Font0);

    for (int row = 0; row < ROWS; row++) {
        int col = 0;
        while (col < COLS) {
            const char *k = keyAt(row, col);
            if (k[0] == '\0') {
                col++;
                continue;
            }
            int span = 1;
            while (col + span < COLS && strcmp(keyAt(row, col + span), k) == 0) {
                span++;
            }
            int x = GRID_X + col * CELL_W;
            int y = TOP_Y + row * CELL_H;
            int w = span * CELL_W - 6;
            int h = CELL_H - 6;
            canvas.fillRoundRect(x, y, w, h, 4, keyColor(k));
            canvas.setTextColor(colorWhite);
            canvas.setTextSize(2);
            canvas.setTextDatum(MC_DATUM);
            canvas.drawString(labelAt(row, col), x + w / 2, y + h / 2);
            col += span;
        }
    }
    screen::markDirty(0, TOP_Y, canvas.width(), HEIGHT);
}

Result handleTouch(int x, int y, String &text, size_t maxLen) {
    if (!contains(x, y)) {
        return Result::NONE;
    }
    int col = (x - GRID_X) / CELL_W;
    int row = (y - TOP_Y) / CELL_H;
    if (col < 0 || col >= COLS || row < 0 || row >= ROWS) {
        return Result::NONE;
    }
    const char *k = keyAt(row, col);
    if (k[0] == '\0') {
        return Result::NONE;
    }

    if (strcmp(k, "OK") == 0) {
        return Result::SUBMIT;
    }
    if (strcmp(k, "SHIFT") == 0) {
        g_mode = (g_mode == Mode::UPPER) ? Mode::LOWER : Mode::UPPER;
        draw();
        return Result::NONE;
    }
    if (strcmp(k, "?#") == 0) {
        g_mode = Mode::SYMBOL;
        draw();
        return Result::NONE;
    }
    if (strcmp(k, "abc") == 0) {
        g_mode = Mode::LOWER;
        draw();
        return Result::NONE;
    }
    if (strcmp(k, "BKSP") == 0) {
        if (text.length()) {
            text.remove(text.length() - 1);
            return Result::EDITED;
        }
        return Result::NONE;
    }
    if (text.length() >= maxLen) {
        return Result::NONE;
    }
    if (strcmp(k, "SPACE") == 0) {
        text += ' ';
    } else {
        text += labelAt(row, col);
    }
    // A shifted letter is a one-shot, the way a phone keyboard behaves.
    if (g_mode == Mode::UPPER) {
        g_mode = Mode::LOWER;
        draw();
    }
    return Result::EDITED;
}

}  // namespace keyboard
