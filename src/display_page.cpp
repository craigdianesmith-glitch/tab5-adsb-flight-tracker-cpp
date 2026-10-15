#include "display_page.h"

#include <M5Unified.h>

#include <algorithm>

#include "screen.h"
#include "sound.h"

namespace {

bool colorsReady = false;
uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorOn, colorOff, colorTrack, colorKnob, colorBorder;

// The settings screen's own, as the page is one of its.
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
    colorsReady = true;
}

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int COL_W = 560, COL1_X = 40, COL2_X = 680;
constexpr int SLIDER_INSET = 24, SLIDER_W = COL_W - 2 * SLIDER_INSET, SLIDER_TRACK_H = 10, KNOB_R = 24;

// --- left column: brightness, and dimming ---
constexpr int BRIGHT_LABEL_Y = 84, BRIGHT_VALUE_Y = 112;
constexpr int SLIDER_X = COL1_X + SLIDER_INSET, SLIDER_Y = 186;
constexpr int SLIDER_BAND_ABOVE = 44, SLIDER_BAND_H = 88;  // where the slider takes a finger
constexpr int CAPS_Y = 228;
constexpr int DIM_LABEL_Y = 290, DIM_Y = 318, DIM_H = 64;
constexpr int DIM_NOTE_Y = 400;
// And under the clock, the replays' tracks: a switch as the settings screen
// draws one, and what it does.
constexpr int SWITCH_LABEL_Y = 496;
constexpr int SWITCH_Y = 522, SWITCH_H = 56;
constexpr int SWITCH_NOTE_Y = 594;
constexpr uint8_t DIM_CHOICES[] = {0, 5, 15, 30};
constexpr const char *DIM_NAMES[] = {"Off", "5 min", "15 min", "30 min"};
constexpr int DIM_COUNT = 4;

// --- right column: the clock, off or in a zone, two to a row ---
constexpr int CLOCK_LABEL_Y = 84, CLOCK_Y = 112, ZONE_H = 64, ZONE_GAP = 10, ZONE_W = (COL_W - ZONE_GAP) / 2;
constexpr int ZONE_ROWS = (CLOCK_ZONE_COUNT + 1) / 2;
constexpr int CLOCK_NOTE_Y = CLOCK_Y + ZONE_ROWS * (ZONE_H + ZONE_GAP) + 8;
constexpr int MIN_PCT = 10, MAX_PCT = 100;

uint8_t g_brightness = 100;
uint8_t g_dimMinutes = 0;
ClockZone g_clock = ClockZone::UTC;
bool g_replayTracks = true;
bool g_sliding = false;

void sectionLabel(const char *text, int x, int y) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorGrey);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(text, x, y);
}

void drawBrightness() {
    auto &canvas = screen::canvas();
    canvas.fillRect(COL1_X, BRIGHT_VALUE_Y - 4, COL_W, CAPS_Y + 24 - BRIGHT_VALUE_Y, colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(colorWhite);
    canvas.drawString(String(g_brightness) + " %", COL1_X + SLIDER_INSET, BRIGHT_VALUE_Y);
    int knobX = SLIDER_X + (g_brightness - MIN_PCT) * SLIDER_W / (MAX_PCT - MIN_PCT);
    canvas.fillRoundRect(SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_TRACK_H, SLIDER_TRACK_H / 2, colorTrack);
    canvas.fillRoundRect(SLIDER_X, SLIDER_Y, knobX - SLIDER_X, SLIDER_TRACK_H, SLIDER_TRACK_H / 2, colorKnob);
    canvas.fillCircle(knobX, SLIDER_Y + SLIDER_TRACK_H / 2, KNOB_R, colorKnob);
    canvas.fillCircle(knobX, SLIDER_Y + SLIDER_TRACK_H / 2, KNOB_R - 8, colorWhite);
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.drawString(String(MIN_PCT) + " %", SLIDER_X, CAPS_Y);
    canvas.setTextDatum(TR_DATUM);
    canvas.drawString(String(MAX_PCT) + " %", SLIDER_X + SLIDER_W, CAPS_Y);
    screen::markDirty(COL1_X, BRIGHT_VALUE_Y - 4, COL_W, CAPS_Y + 24 - BRIGHT_VALUE_Y);
}

int dimIndex() {
    for (int i = 0; i < DIM_COUNT; i++) {
        if (DIM_CHOICES[i] == g_dimMinutes) {
            return i;
        }
    }
    return 0;
}

void drawDim() {
    auto &canvas = screen::canvas();
    int seg = COL_W / DIM_COUNT, active = dimIndex();
    canvas.fillRoundRect(COL1_X, DIM_Y, COL_W, DIM_H, 8, colorOff);
    canvas.fillRoundRect(COL1_X + active * seg, DIM_Y, seg, DIM_H, 8, colorOn);
    canvas.drawRoundRect(COL1_X, DIM_Y, COL_W, DIM_H, 8, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(MC_DATUM);
    for (int i = 0; i < DIM_COUNT; i++) {
        canvas.setTextColor(i == active ? colorWhite : colorDim);
        canvas.drawString(DIM_NAMES[i], COL1_X + i * seg + seg / 2, DIM_Y + DIM_H / 2);
    }
    screen::markDirty(COL1_X, DIM_Y, COL_W, DIM_H);
}

// As the settings screen's switches are drawn.
void drawSwitch(int x, const char *title, const char *detail, bool on) {
    auto &canvas = screen::canvas();
    int y = SWITCH_Y, w = COL_W, h = SWITCH_H;
    canvas.fillRoundRect(x, y, w, h, 8, colorBtnBg);
    canvas.drawRoundRect(x, y, w, h, 8, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextSize(3);
    canvas.setTextColor(colorWhite);
    canvas.drawString(title, x + 24, y + h / 2 - 10);
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.drawString(detail, x + 24, y + h / 2 + 15);
    int sw = 108, sh = 40;
    int sx = x + w - sw - 20, sy = y + (h - sh) / 2;
    canvas.fillRoundRect(sx, sy, sw, sh, sh / 2, on ? colorOn : colorOff);
    int knob = sh - 8;
    canvas.fillCircle(on ? (sx + sw - 4 - knob / 2) : (sx + 4 + knob / 2), sy + sh / 2, knob / 2, colorWhite);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString(on ? "ON" : "OFF", on ? (sx + 30) : (sx + sw - 30), sy + sh / 2);
    screen::markDirty(x, y, w, h);
}

void drawReplayTracks() { drawSwitch(COL2_X, "Replay tracks", "Trails behind each aircraft", g_replayTracks); }

// The zones as chips, the one picked lit, and under them the time it gives.
void drawClockZones() {
    auto &canvas = screen::canvas();
    int h = CLOCK_NOTE_Y + 24 - CLOCK_Y;
    canvas.fillRect(COL2_X, CLOCK_Y, COL_W, h, colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(MC_DATUM);
    for (int i = 0; i < CLOCK_ZONE_COUNT; i++) {
        int x = COL2_X + (i % 2) * (ZONE_W + ZONE_GAP), y = CLOCK_Y + (i / 2) * (ZONE_H + ZONE_GAP);
        bool on = (int)g_clock == i;
        canvas.fillRoundRect(x, y, ZONE_W, ZONE_H, 8, on ? colorOn : colorOff);
        canvas.drawRoundRect(x, y, ZONE_W, ZONE_H, 8, colorBorder);
        canvas.setTextColor(on ? colorWhite : colorDim);
        canvas.drawString(clockZoneName((ClockZone)i), x + ZONE_W / 2, y + ZONE_H / 2);
    }
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    time_t now = time(nullptr);
    if (g_clock == ClockZone::OFF) {
        canvas.drawString("No clock on the radar or the zoom", COL2_X, CLOCK_NOTE_Y);
    } else if (now > 1600000000) {
        char text[16];
        clockZoneFormat(now, g_clock, text, sizeof(text));
        canvas.drawString(String("On the radar and the zoom: ") + text, COL2_X, CLOCK_NOTE_Y);
    }
    screen::markDirty(COL2_X, CLOCK_Y, COL_W, h);
}

bool inside(int x, int y, int rx, int ry, int rw, int rh) { return x >= rx && x < rx + rw && y >= ry && y < ry + rh; }

}  // namespace

void displayApplyBrightness(uint8_t pct) {
    pct = std::min<uint8_t>(std::max<uint8_t>(pct, 1), 100);
    M5.Display.setBrightness((uint8_t)(pct * 255 / 100));
}

void displayPageSet(uint8_t brightnessPct, uint8_t dimMinutes, ClockZone clock, bool replayTracks) {
    g_brightness = std::min<uint8_t>(std::max<uint8_t>(brightnessPct, MIN_PCT), MAX_PCT);
    g_dimMinutes = dimMinutes;
    g_clock = clock;
    g_replayTracks = replayTracks;
}

void displayPageDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Display", 16, 12);
    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    sectionLabel("BRIGHTNESS", COL1_X, BRIGHT_LABEL_Y);
    drawBrightness();
    sectionLabel("DIM WHEN UNTOUCHED", COL1_X, DIM_LABEL_Y);
    drawDim();
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Down to a glow, for a desk or a wall at", COL1_X, DIM_NOTE_Y);
    canvas.drawString("night. A touch brings it back - that touch", COL1_X, DIM_NOTE_Y + 26);
    canvas.drawString("does nothing else - and so does an alert.", COL1_X, DIM_NOTE_Y + 52);
    sectionLabel("REPLAYS", COL2_X, SWITCH_LABEL_Y);
    drawReplayTracks();
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Each aircraft's track over the last ten", COL2_X, SWITCH_NOTE_Y);
    canvas.drawString("minutes, behind it as a replay plays - and", COL2_X, SWITCH_NOTE_Y + 26);
    canvas.drawString("in the videos exported from one.", COL2_X, SWITCH_NOTE_Y + 52);
    sectionLabel("CLOCK", COL2_X, CLOCK_LABEL_Y);
    drawClockZones();
    screen::flush();
}

DisplayPageAction displayPageHandleTouch(int x, int y, bool pressed, bool clicked) {
    // The slider follows the finger from where it lands in its band until it
    // lifts, the brightness going with it.
    if (pressed && (g_sliding || inside(x, y, SLIDER_X - KNOB_R, SLIDER_Y - SLIDER_BAND_ABOVE, SLIDER_W + 2 * KNOB_R,
                                        SLIDER_BAND_H))) {
        g_sliding = true;
        int pct = MIN_PCT + (x - SLIDER_X) * (MAX_PCT - MIN_PCT) / SLIDER_W;
        pct = std::min(std::max(pct, MIN_PCT), MAX_PCT);
        if (pct != g_brightness) {
            g_brightness = (uint8_t)pct;
            displayApplyBrightness(g_brightness);
            drawBrightness();
            screen::flush();
        }
        return DisplayPageAction::NONE;
    }
    if (!pressed) {
        g_sliding = false;
    }
    if (!clicked) {
        return DisplayPageAction::NONE;
    }
    if (inside(x, y, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        return DisplayPageAction::BACK;
    }
    if (inside(x, y, COL1_X, DIM_Y, COL_W, DIM_H)) {
        g_dimMinutes = DIM_CHOICES[std::min((x - COL1_X) * DIM_COUNT / COL_W, DIM_COUNT - 1)];
        drawDim();
        screen::flush();
    } else if (inside(x, y, COL2_X, SWITCH_Y, COL_W, SWITCH_H)) {
        g_replayTracks = !g_replayTracks;
        drawReplayTracks();
        screen::flush();
    } else if (inside(x, y, COL2_X, CLOCK_Y, COL_W, ZONE_ROWS * (ZONE_H + ZONE_GAP))) {
        int col = x - COL2_X < ZONE_W + ZONE_GAP / 2 ? 0 : 1;
        int i = (y - CLOCK_Y) / (ZONE_H + ZONE_GAP) * 2 + col;
        if (i < CLOCK_ZONE_COUNT) {
            g_clock = (ClockZone)i;
            drawClockZones();
            screen::flush();
        }
    }
    return DisplayPageAction::NONE;
}

uint8_t displayPageBrightness() { return g_brightness; }
uint8_t displayPageDimMinutes() { return g_dimMinutes; }
ClockZone displayPageClock() { return g_clock; }
bool displayPageReplayTracks() { return g_replayTracks; }
