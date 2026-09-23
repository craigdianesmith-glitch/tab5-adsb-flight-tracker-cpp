#include "settings_screen.h"

#include <M5Unified.h>
#include <WiFi.h>

#include "config.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;

constexpr int SEG_X = 40, SEG_Y = 128, SEG_W = 1200, SEG_H = 76;

constexpr int RADIUS_LABEL_Y = 232;
constexpr int RADIUS_VALUE_Y = 262;
constexpr int SLIDER_X = 64, SLIDER_W = 1152, SLIDER_Y = 344, SLIDER_TRACK_H = 10;
constexpr int KNOB_R = 24;
constexpr int SLIDER_BAND_TOP = SLIDER_Y - 44, SLIDER_BAND_H = 88;
constexpr int TICKS_Y = SLIDER_Y + 40;
constexpr int CAPS_Y = 424;

constexpr int WIFI_LABEL_Y = 470;
constexpr int WIFI_X = 40, WIFI_Y = 500, WIFI_W = 1200, WIFI_H = 96;

TrafficFilter g_traffic = TrafficFilter::CIVIL;
int g_radius = DEFAULT_RADIUS_NM;
bool g_dragging = false;

uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorOn, colorOff, colorTrack, colorKnob, colorBorder;

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

bool isMilitary() { return g_traffic == TrafficFilter::MILITARY; }

// Military traffic is worth watching a good deal further out than civil, so
// the range ceiling follows the choice above it.
int maxRadius() { return isMilitary() ? MILITARY_MAX_RADIUS_NM : CIVIL_MAX_RADIUS_NM; }

void clampRadius() {
    if (g_radius > maxRadius()) {
        g_radius = maxRadius();
    }
    if (g_radius < RADIUS_MIN_NM) {
        g_radius = RADIUS_MIN_NM;
    }
}

// One control with two halves rather than two switches: the two are mutually
// exclusive, and a segmented control says so without needing to be explained.
void drawSegments() {
    auto &canvas = screen::canvas();
    int half = SEG_W / 2;
    canvas.fillRoundRect(SEG_X, SEG_Y, SEG_W, SEG_H, 8, colorOff);
    canvas.fillRoundRect(isMilitary() ? SEG_X + half : SEG_X, SEG_Y, half, SEG_H, 8, colorOn);
    canvas.drawRoundRect(SEG_X, SEG_Y, SEG_W, SEG_H, 8, colorBorder);

    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextColor(isMilitary() ? colorDim : colorWhite);
    canvas.drawString("Civilian flights", SEG_X + half / 2, SEG_Y + SEG_H / 2);
    canvas.setTextColor(isMilitary() ? colorWhite : colorDim);
    canvas.drawString("Military flights", SEG_X + half + half / 2, SEG_Y + SEG_H / 2);

    screen::markDirty(SEG_X, SEG_Y, SEG_W, SEG_H);
}

void drawSlider() {
    auto &canvas = screen::canvas();
    int top = RADIUS_VALUE_Y - 8;
    int height = (CAPS_Y + 28) - top;
    canvas.fillRect(0, top, canvas.width(), height, colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(String(g_radius) + " nm", SLIDER_X, RADIUS_VALUE_Y);

    canvas.fillRoundRect(SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_TRACK_H, SLIDER_TRACK_H / 2, colorTrack);
    int span = maxRadius() - RADIUS_MIN_NM;
    int knobX = SLIDER_X + (int)((int64_t)(g_radius - RADIUS_MIN_NM) * SLIDER_W / (span > 0 ? span : 1));
    canvas.fillRoundRect(SLIDER_X, SLIDER_Y, knobX - SLIDER_X, SLIDER_TRACK_H, SLIDER_TRACK_H / 2, colorKnob);
    canvas.fillCircle(knobX, SLIDER_Y + SLIDER_TRACK_H / 2, KNOB_R, colorKnob);
    canvas.fillCircle(knobX, SLIDER_Y + SLIDER_TRACK_H / 2, KNOB_R - 8, colorWhite);

    canvas.setTextSize(2);
    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(String(RADIUS_MIN_NM) + " nm", SLIDER_X, TICKS_Y);
    canvas.setTextDatum(TR_DATUM);
    canvas.drawString(String(maxRadius()) + " nm", SLIDER_X + SLIDER_W, TICKS_Y);

    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Range reaches 60 nm for civil traffic, 150 nm for military", SLIDER_X, CAPS_Y);

    screen::markDirty(0, top, canvas.width(), height);
}

}  // namespace

void settingsScreenSet(TrafficFilter traffic, int radiusNm) {
    g_traffic = traffic;
    g_radius = radiusNm;
    g_dragging = false;
    clampRadius();
}

void settingsScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Settings", 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    canvas.setTextColor(colorGrey);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("SHOW", 40, 92);
    drawSegments();

    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("RANGE", 40, RADIUS_LABEL_Y);
    drawSlider();

    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("NETWORK", 40, WIFI_LABEL_Y);
    canvas.fillRoundRect(WIFI_X, WIFI_Y, WIFI_W, WIFI_H, 8, colorBtnBg);
    canvas.drawRoundRect(WIFI_X, WIFI_Y, WIFI_W, WIFI_H, 8, colorBorder);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.drawString("WiFi settings", WIFI_X + 24, WIFI_Y + 30);
    canvas.setTextColor(colorDim);
    canvas.setTextSize(2);
    canvas.drawString(WiFi.status() == WL_CONNECTED ? ("Connected to " + WiFi.SSID() + "   " + WiFi.localIP().toString())
                                                    : String("Not connected"),
                      WIFI_X + 24, WIFI_Y + 68);
    canvas.setTextColor(colorGrey);
    canvas.setTextSize(3);
    canvas.setTextDatum(MR_DATUM);
    canvas.drawString(">", WIFI_X + WIFI_W - 24, WIFI_Y + WIFI_H / 2);

    screen::flush();
}

SettingsAction settingsScreenHandleTouch(int x, int y, bool pressed, bool clicked) {
    // Slider first: a held finger keeps control of it even once it wanders
    // outside the band, which is what makes dragging feel right.
    if (g_dragging || (pressed && y >= SLIDER_BAND_TOP && y < SLIDER_BAND_TOP + SLIDER_BAND_H)) {
        if (pressed) {
            g_dragging = true;
            int span = maxRadius() - RADIUS_MIN_NM;
            int pos = x - SLIDER_X;
            if (pos < 0) pos = 0;
            if (pos > SLIDER_W) pos = SLIDER_W;
            int value = RADIUS_MIN_NM + (int)(((int64_t)pos * span + SLIDER_W / 2) / SLIDER_W);
            if (value != g_radius) {
                g_radius = value;
                drawSlider();
                screen::flush();
            }
            return SettingsAction::NONE;
        }
        g_dragging = false;
        return SettingsAction::NONE;
    }

    if (!clicked) {
        return SettingsAction::NONE;
    }

    if (x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return SettingsAction::BACK;
    }

    if (y >= SEG_Y && y < SEG_Y + SEG_H && x >= SEG_X && x < SEG_X + SEG_W) {
        TrafficFilter picked = (x < SEG_X + SEG_W / 2) ? TrafficFilter::CIVIL : TrafficFilter::MILITARY;
        if (picked != g_traffic) {
            g_traffic = picked;
            clampRadius();  // the ceiling moves with the choice
            drawSegments();
            drawSlider();
            screen::flush();
        }
        return SettingsAction::NONE;
    }

    if (x >= WIFI_X && x < WIFI_X + WIFI_W && y >= WIFI_Y && y < WIFI_Y + WIFI_H) {
        return SettingsAction::OPEN_WIFI;
    }
    return SettingsAction::NONE;
}

TrafficFilter settingsScreenTraffic() { return g_traffic; }
int settingsScreenRadius() { return g_radius; }
