#include "filter_screen.h"

#include <M5Unified.h>

#include <algorithm>

#include "keyboard.h"
#include "screen.h"

namespace {

bool colorsReady = false;
uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorOn, colorOff, colorTrack, colorKnob, colorBorder,
    colorClear;

// The display page's, as the screen is laid out as it is.
void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorGrey = M5.Display.color565(0xBB, 0xBB, 0xBB);
    colorDim = M5.Display.color565(0x88, 0x91, 0x9B);
    colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorOn = M5.Display.color565(0x27, 0xAE, 0x60);
    colorOff = M5.Display.color565(0x30, 0x36, 0x3D);
    colorTrack = M5.Display.color565(0x30, 0x36, 0x3D);
    colorKnob = M5.Display.color565(0x29, 0x80, 0xB9);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorClear = M5.Display.color565(0xC0, 0x39, 0x2B);
    colorsReady = true;
}

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int CLEAR_X = 990, CLEAR_W = 138;
constexpr int COL_W = 560, COL1_X = 40, COL2_X = 680;
constexpr int SLIDER_INSET = 24, SLIDER_W = COL_W - 2 * SLIDER_INSET, SLIDER_TRACK_H = 10, KNOB_R = 22;
constexpr int SLIDER_BAND_ABOVE = 44, SLIDER_BAND_H = 88;  // where a slider takes a finger

// A band of values: its slider's place, its range and step, and how a value
// is written.
struct Band {
    const char *label;
    int labelY, valueY, trackY, capsY;
    int top, step;
    bool feet;
};
// --- left column: the two bands, clear of the keyboard ---
constexpr Band HEIGHT = {"HEIGHT", 84, 112, 176, 212, FILTER_TOP_FT, 1000, true};
constexpr Band SPEED = {"SPEED", 262, 290, 354, 390, FILTER_TOP_KT, 10, false};

// --- right column: civil or military, airlines and types ---
constexpr int SHOW_LABEL_Y = 84, SHOW_Y = 112, SHOW_H = 64;
constexpr int AIRLINE_LABEL_Y = 196, AIRLINE_Y = 224;
// The types shown, and beside them those left out: half the column each, the
// keyboard leaving no room for a row of their own.
constexpr int TYPE_LABEL_Y = 312, TYPE_Y = 340;
constexpr int HALF_W = (COL_W - 16) / 2, HIDE_X = COL2_X + HALF_W + 16;
constexpr int BOX_H = 56, HINT_GAP = 6;
constexpr size_t BOX_MAX = 40;

// --- under both, while no box is being typed in: what the filter is for ---
constexpr int NOTE_Y = 470;

TrafficFilterSet g_f;
enum class Editing { NONE, AIRLINE, TYPE, HIDE };
Editing g_editing = Editing::NONE;
// Which knob of which band a finger has, from landing until it lifts.
const Band *g_sliding = nullptr;
bool g_slidingMax = false;

int &lowOf(const Band &b) { return &b == &HEIGHT ? g_f.minFt : g_f.minKt; }
int &highOf(const Band &b) { return &b == &HEIGHT ? g_f.maxFt : g_f.maxKt; }

String valueText(const Band &b, int v, bool high) {
    if (high && v >= b.top) {
        return "any";
    }
    if (b.feet) {
        return v == 0 ? String("GND") : String(v) + " ft";
    }
    return String(v) + " kt";
}

int knobX(const Band &b, int v) { return COL1_X + SLIDER_INSET + v * SLIDER_W / b.top; }

void sectionLabel(const char *text, int x, int y) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorGrey);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(text, x, y);
}

void drawKnob(int x, int y) {
    auto &canvas = screen::canvas();
    canvas.fillCircle(x, y, KNOB_R, colorKnob);
    canvas.fillCircle(x, y, KNOB_R - 8, colorWhite);
}

void drawBand(const Band &b) {
    auto &canvas = screen::canvas();
    int top = b.valueY - 4, h = b.capsY + 20 - top;
    canvas.fillRect(COL1_X, top, COL_W, h, colorBg);
    int lo = lowOf(b), hi = highOf(b);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(colorWhite);
    String text = lo == 0 && hi >= b.top ? String("Any")
                                         : valueText(b, lo, false) + " to " + valueText(b, hi, true);
    canvas.drawString(text, COL1_X + SLIDER_INSET, b.valueY);
    int x0 = COL1_X + SLIDER_INSET, cy = b.trackY + SLIDER_TRACK_H / 2;
    int xl = knobX(b, lo), xh = knobX(b, hi);
    canvas.fillRoundRect(x0, b.trackY, SLIDER_W, SLIDER_TRACK_H, SLIDER_TRACK_H / 2, colorTrack);
    canvas.fillRect(xl, b.trackY, xh - xl, SLIDER_TRACK_H, colorKnob);
    drawKnob(xl, cy);
    drawKnob(xh, cy);
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.drawString(b.feet ? "GND" : "0 kt", x0, b.capsY);
    canvas.setTextDatum(TR_DATUM);
    canvas.drawString(b.feet ? "45000 ft +" : "600 kt +", x0 + SLIDER_W, b.capsY);
    screen::markDirty(COL1_X, top, COL_W, h);
}

// The kinds, three chips in a row - each its own switch.
constexpr int SHOW_COUNT = 3;
constexpr const char *SHOW_NAMES[SHOW_COUNT] = {"Civil", "Cargo", "Military"};

bool &showOf(int i) { return i == 0 ? g_f.civil : i == 1 ? g_f.cargo : g_f.military; }

void drawShow() {
    auto &canvas = screen::canvas();
    int seg = COL_W / SHOW_COUNT;
    canvas.fillRoundRect(COL2_X, SHOW_Y, COL_W, SHOW_H, 8, colorOff);
    for (int i = 0; i < SHOW_COUNT; i++) {
        if (showOf(i)) {
            canvas.fillRoundRect(COL2_X + i * seg, SHOW_Y, i == SHOW_COUNT - 1 ? COL_W - i * seg : seg, SHOW_H, 8,
                                 colorOn);
        }
    }
    canvas.drawRoundRect(COL2_X, SHOW_Y, COL_W, SHOW_H, 8, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(MC_DATUM);
    for (int i = 0; i < SHOW_COUNT; i++) {
        if (i > 0) {
            canvas.drawFastVLine(COL2_X + i * seg, SHOW_Y, SHOW_H, colorBorder);
        }
        canvas.setTextColor(showOf(i) ? colorWhite : colorDim);
        canvas.drawString(SHOW_NAMES[i], COL2_X + i * seg + seg / 2, SHOW_Y + SHOW_H / 2);
    }
    screen::markDirty(COL2_X, SHOW_Y, COL_W, SHOW_H);
}

// A box `w` wide at (x, y), what has been typed in it - a size down when
// that won't fit, and then its end, which is where the typing is - and a hint
// under it.
void drawBox(int x, int w, int y, const String &text, const char *placeholder, const char *hint, bool editing) {
    auto &canvas = screen::canvas();
    canvas.fillRect(x, y, w, BOX_H + HINT_GAP + 18, colorBg);
    canvas.fillRoundRect(x, y, w, BOX_H, 6, colorOff);
    canvas.drawRoundRect(x, y, w, BOX_H, 6, editing ? colorKnob : colorBorder);
    if (editing) {
        canvas.drawRoundRect(x + 1, y + 1, w - 2, BOX_H - 2, 5, colorKnob);
    }
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    String shown = text;
    shown.toUpperCase();
    if (shown.length()) {
        if (editing) {
            shown += "_";
        }
        int room = w - 32;
        if (canvas.textWidth(shown) > room) {
            canvas.setTextSize(2);
            if (canvas.textWidth(shown) > room) {
                while (shown.length() > 1 && canvas.textWidth(".." + shown) > room) {
                    shown.remove(0, 1);
                }
                shown = ".." + shown;
            }
        }
        canvas.setTextColor(colorWhite);
        canvas.drawString(shown, x + 16, y + BOX_H / 2);
    } else {
        canvas.setTextColor(colorDim);
        canvas.drawString(editing ? String("_") : String(placeholder), x + 16, y + BOX_H / 2);
    }
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(hint, x, y + BOX_H + HINT_GAP);
    screen::markDirty(x, y, w, BOX_H + HINT_GAP + 18);
}

void drawAirline() {
    drawBox(COL2_X, COL_W, AIRLINE_Y, g_f.airlines, "All airlines", "As callsigns begin:  EZY RYR BAW",
            g_editing == Editing::AIRLINE);
}

void drawType() {
    drawBox(COL2_X, HALF_W, TYPE_Y, g_f.types, "All types", "As they begin: A32 C17", g_editing == Editing::TYPE);
}

void drawHide() {
    drawBox(HIDE_X, HALF_W, TYPE_Y, g_f.hideTypes, "None", "Left out:  TWR", g_editing == Editing::HIDE);
}

void drawNote() {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("For the table, the radar, and replays as they were recorded. The aircraft followed, the airport",
                      COL1_X, NOTE_Y);
    canvas.drawString("zoom and alerts are never filtered, recordings keep every aircraft, and an emergency is never",
                      COL1_X, NOTE_Y + 28);
    canvas.drawString("hidden. An aircraft whose height or speed isn't known is left out by a band that has been set.",
                      COL1_X, NOTE_Y + 56);
    canvas.drawString("While a filter is on, the funnel in the table's header is lit.", COL1_X, NOTE_Y + 98);
}

void drawClearButton() {
    auto &canvas = screen::canvas();
    canvas.fillRect(CLEAR_X, BACK_Y, CLEAR_W, BACK_H, colorBg);
    if (g_f.active()) {
        canvas.fillRoundRect(CLEAR_X, BACK_Y, CLEAR_W, BACK_H, 6, colorClear);
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(2);
        canvas.setTextColor(colorWhite);
        canvas.setTextDatum(MC_DATUM);
        canvas.drawString("Clear all", CLEAR_X + CLEAR_W / 2, BACK_Y + BACK_H / 2);
    }
    screen::markDirty(CLEAR_X, BACK_Y, CLEAR_W, BACK_H);
}

bool inside(int x, int y, int rx, int ry, int rw, int rh) { return x >= rx && x < rx + rw && y >= ry && y < ry + rh; }

bool inBand(const Band &b, int x, int y) {
    return inside(x, y, COL1_X + SLIDER_INSET - KNOB_R, b.trackY - SLIDER_BAND_ABOVE, SLIDER_W + 2 * KNOB_R,
                  SLIDER_BAND_H);
}

// The value under x, to the band's step.
int valueAt(const Band &b, int x) {
    int v = (x - COL1_X - SLIDER_INSET) * b.top / SLIDER_W;
    v = (v + b.step / 2) / b.step * b.step;
    return std::min(std::max(v, 0), b.top);
}

void stopEditing() {
    g_editing = Editing::NONE;
    filterScreenDraw();
}

}  // namespace

void filterScreenOpen(const TrafficFilterSet &f) {
    g_f = f;
    g_editing = Editing::NONE;
    g_sliding = nullptr;
}

void filterScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Filter", 16, 12);
    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    drawClearButton();

    sectionLabel(HEIGHT.label, COL1_X, HEIGHT.labelY);
    drawBand(HEIGHT);
    sectionLabel(SPEED.label, COL1_X, SPEED.labelY);
    drawBand(SPEED);
    sectionLabel("SHOW", COL2_X, SHOW_LABEL_Y);
    drawShow();
    sectionLabel("AIRLINE", COL2_X, AIRLINE_LABEL_Y);
    drawAirline();
    sectionLabel("TYPE", COL2_X, TYPE_LABEL_Y);
    drawType();
    sectionLabel("HIDE TYPES", HIDE_X, TYPE_LABEL_Y);
    drawHide();
    if (g_editing != Editing::NONE) {
        keyboard::draw();
    } else {
        drawNote();
    }
    screen::flush();
}

FilterAction filterScreenHandleTouch(int x, int y, bool pressed, bool clicked) {
    // A slider's nearer knob follows the finger from where it lands in its
    // band until it lifts, never past the other.
    if (pressed && !g_sliding && g_editing == Editing::NONE) {
        for (const Band *b : {&HEIGHT, &SPEED}) {
            if (inBand(*b, x, y)) {
                int v = valueAt(*b, x), lo = lowOf(*b), hi = highOf(*b);
                g_sliding = b;
                // Where both knobs sit together, the side the finger is on
                // says which: the high one at the bottom can't go lower.
                g_slidingMax = std::abs(v - hi) < std::abs(v - lo) || (lo == hi && v > hi) ||
                               (lo == hi && lo == 0);
                if (lo == hi && hi == b->top) {
                    g_slidingMax = false;
                }
                break;
            }
        }
    }
    if (pressed && g_sliding) {
        const Band &b = *g_sliding;
        int v = valueAt(b, x);
        int &lo = lowOf(b), &hi = highOf(b);
        int was = g_slidingMax ? hi : lo;
        if (g_slidingMax) {
            hi = std::max(v, lo);
        } else {
            lo = std::min(v, hi);
        }
        if ((g_slidingMax ? hi : lo) != was) {
            drawBand(b);
            drawClearButton();
            screen::flush();
        }
        return FilterAction::NONE;
    }
    if (!pressed) {
        g_sliding = nullptr;
    }
    if (!clicked) {
        return FilterAction::NONE;
    }
    if (inside(x, y, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        return FilterAction::BACK;
    }
    if (inside(x, y, CLEAR_X, BACK_Y, CLEAR_W, BACK_H) && g_f.active()) {
        g_f = TrafficFilterSet();
        g_editing = Editing::NONE;
        filterScreenDraw();
        return FilterAction::NONE;
    }
    if (g_editing != Editing::NONE && keyboard::contains(x, y)) {
        String &text = g_editing == Editing::AIRLINE ? g_f.airlines
                       : g_editing == Editing::TYPE  ? g_f.types
                                                     : g_f.hideTypes;
        switch (keyboard::handleTouch(x, y, text, BOX_MAX)) {
        case keyboard::Result::EDITED:
            text.toUpperCase();
            g_editing == Editing::AIRLINE ? drawAirline() : g_editing == Editing::TYPE ? drawType() : drawHide();
            drawClearButton();
            screen::flush();
            break;
        case keyboard::Result::SUBMIT:
            text.trim();
            stopEditing();
            break;
        case keyboard::Result::NONE:
            break;
        }
        return FilterAction::NONE;
    }
    if (inside(x, y, COL2_X, AIRLINE_Y, COL_W, BOX_H) || inside(x, y, COL2_X, TYPE_Y, COL_W, BOX_H)) {
        Editing which = y < TYPE_Y ? Editing::AIRLINE : x < HIDE_X - 8 ? Editing::TYPE : Editing::HIDE;
        if (which != g_editing) {
            if (g_editing == Editing::NONE) {
                keyboard::reset();
            }
            g_editing = which;
            filterScreenDraw();
        }
        return FilterAction::NONE;
    }
    if (g_editing != Editing::NONE) {
        // A tap anywhere else puts the keyboard away, and does nothing more.
        g_f.airlines.trim();
        g_f.types.trim();
        g_f.hideTypes.trim();
        stopEditing();
        return FilterAction::NONE;
    }
    if (inside(x, y, COL2_X, SHOW_Y, COL_W, SHOW_H)) {
        // One is always lit: with none, nothing would show.
        int i = std::min((x - COL2_X) * SHOW_COUNT / COL_W, SHOW_COUNT - 1);
        int lit = (int)g_f.civil + (int)g_f.cargo + (int)g_f.military;
        if (!showOf(i) || lit > 1) {
            showOf(i) = !showOf(i);
        }
        drawShow();
        drawClearButton();
        screen::flush();
    }
    return FilterAction::NONE;
}

TrafficFilterSet filterScreenResult() {
    TrafficFilterSet f = g_f;
    f.airlines.trim();
    f.types.trim();
    f.hideTypes.trim();
    f.airlines.toUpperCase();
    f.types.toUpperCase();
    f.hideTypes.toUpperCase();
    return f;
}
