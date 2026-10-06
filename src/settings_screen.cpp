#include "settings_screen.h"

#include <M5Unified.h>
#include <WiFi.h>

#include "config.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
// Beside Back in the title bar, as Share is on the recordings screen: the
// two screens of reading, about the tracker rather than its settings.
constexpr int ABOUT_X = BACK_X - 12 - BACK_W, HELP_X = ABOUT_X - 12 - BACK_W;

// Two columns rather than one full-width stack: there are six things on this
// screen now and 720px of height won't take them end to end, while 1280px of
// width was going spare.
constexpr int COL_W = 560;
constexpr int COL1_X = 40;
constexpr int COL2_X = 680;
constexpr int SLIDER_INSET = 24;
constexpr int SLIDER_W = COL_W - 2 * SLIDER_INSET;
constexpr int SLIDER_TRACK_H = 10;
constexpr int KNOB_R = 24;
// A slider grabs a finger anywhere in this band, not just on the knob itself.
constexpr int BAND_ABOVE = 44, BAND_H = 88;

// --- left column ---
constexpr int SHOW_LABEL_Y = 84;
constexpr int SEG_X = COL1_X, SEG_Y = 112, SEG_W = COL_W, SEG_H = 76;

constexpr int RADIUS_LABEL_Y = 216;
constexpr int RADIUS_VALUE_Y = 244;
constexpr int RADIUS_SLIDER_X = COL1_X + SLIDER_INSET, RADIUS_SLIDER_Y = 318;
constexpr int RADIUS_TICKS_Y = RADIUS_SLIDER_Y + 38;
constexpr int RADIUS_CAPS_Y = 392;

// Two switches, a little shorter than the other rows to fit them both.
constexpr int DISPLAY_LABEL_Y = 440;
constexpr int TOGGLE_H = 62;
constexpr int REFRESH_X = COL1_X, REFRESH_Y = 468, REFRESH_W = COL_W, REFRESH_H = TOGGLE_H;
constexpr int NAVAIDS_X = COL1_X, NAVAIDS_Y = REFRESH_Y + TOGGLE_H + 8, NAVAIDS_W = COL_W, NAVAIDS_H = TOGGLE_H;

// The space below the display toggles was the only room left for this, which
// is no hardship: the source is the one thing here you set once and forget.
constexpr int SOURCE_LABEL_Y = 616;
constexpr int SRC_X = COL1_X, SRC_Y = 640, SRC_W = COL_W, SRC_H = 66;
// Auto plus one per provider.
constexpr int SRC_SEGMENTS = ADSB_PROVIDER_COUNT + 1;

// --- right column ---
constexpr int INTERVAL_LABEL_Y = 84;
constexpr int INTERVAL_VALUE_Y = 112;
constexpr int INTERVAL_SLIDER_X = COL2_X + SLIDER_INSET, INTERVAL_SLIDER_Y = 186;
constexpr int INTERVAL_TICKS_Y = INTERVAL_SLIDER_Y + 38;
constexpr int INTERVAL_CAPS_Y = 260;

// Three "leads somewhere else" rows now, so they are a little shorter than the
// two used to be, to fit them all under the interval slider.
constexpr int NAV_H = 84;
constexpr int LOC_LABEL_Y = 308;
constexpr int LOC_X = COL2_X, LOC_Y = 336, LOC_W = COL_W, LOC_H = NAV_H;

constexpr int WIFI_LABEL_Y = 440;
constexpr int WIFI_X = COL2_X, WIFI_Y = 468, WIFI_W = COL_W, WIFI_H = NAV_H;

constexpr int ALERTS_LABEL_Y = 572;
constexpr int ALERTS_X = COL2_X, ALERTS_Y = 600, ALERTS_W = COL_W, ALERTS_H = NAV_H;

TrafficFilter g_traffic = TrafficFilter::CIVIL;
int g_radius = DEFAULT_RADIUS_NM;
int g_interval = DEFAULT_POLL_INTERVAL_S;
bool g_showRefresh = true;
bool g_navaids = true;
String g_locationLabel;
String g_alertSummary;
AdsbSource g_source = DEFAULT_ADSB_SOURCE;

// Which slider, if either, currently owns the finger.
enum class Drag { NONE, RADIUS, INTERVAL };
Drag g_drag = Drag::NONE;

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

// The interval slider is stepped, not continuous: twelve detents from 5s to
// 60s, so it can't be left on a value nobody asked for.
constexpr int intervalSteps() { return (POLL_INTERVAL_MAX_S - POLL_INTERVAL_MIN_S) / POLL_INTERVAL_STEP_S; }

// Shared by both sliders: the track, the filled portion and the knob.
void drawTrack(int x, int y, int knobX) {
    auto &canvas = screen::canvas();
    canvas.fillRoundRect(x, y, SLIDER_W, SLIDER_TRACK_H, SLIDER_TRACK_H / 2, colorTrack);
    canvas.fillRoundRect(x, y, knobX - x, SLIDER_TRACK_H, SLIDER_TRACK_H / 2, colorKnob);
    canvas.fillCircle(knobX, y + SLIDER_TRACK_H / 2, KNOB_R, colorKnob);
    canvas.fillCircle(knobX, y + SLIDER_TRACK_H / 2, KNOB_R - 8, colorWhite);
}

void drawRadiusSlider() {
    auto &canvas = screen::canvas();
    int top = RADIUS_VALUE_Y - 8;
    int height = (RADIUS_CAPS_Y + 28) - top;
    canvas.fillRect(COL1_X, top, COL_W, height, colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(String(g_radius) + " nm", RADIUS_SLIDER_X, RADIUS_VALUE_Y);

    int span = maxRadius() - RADIUS_MIN_NM;
    int knobX = RADIUS_SLIDER_X + (int)((int64_t)(g_radius - RADIUS_MIN_NM) * SLIDER_W / (span > 0 ? span : 1));
    drawTrack(RADIUS_SLIDER_X, RADIUS_SLIDER_Y, knobX);

    canvas.setTextSize(2);
    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(String(RADIUS_MIN_NM) + " nm", RADIUS_SLIDER_X, RADIUS_TICKS_Y);
    canvas.setTextDatum(TR_DATUM);
    canvas.drawString(String(maxRadius()) + " nm", RADIUS_SLIDER_X + SLIDER_W, RADIUS_TICKS_Y);

    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("60 nm civil, 150 nm military", RADIUS_SLIDER_X, RADIUS_CAPS_Y);

    screen::markDirty(COL1_X, top, COL_W, height);
}

void drawIntervalSlider() {
    auto &canvas = screen::canvas();
    int top = INTERVAL_VALUE_Y - 8;
    int height = (INTERVAL_CAPS_Y + 28) - top;
    canvas.fillRect(COL2_X, top, COL_W, height, colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Every " + String(g_interval) + " s", INTERVAL_SLIDER_X, INTERVAL_VALUE_Y);

    int idx = (g_interval - POLL_INTERVAL_MIN_S) / POLL_INTERVAL_STEP_S;
    int knobX = INTERVAL_SLIDER_X + (int)((int64_t)idx * SLIDER_W / intervalSteps());

    // Detent marks under the track, so it reads as stepped before it's touched.
    for (int i = 0; i <= intervalSteps(); i++) {
        int tx = INTERVAL_SLIDER_X + (int)((int64_t)i * SLIDER_W / intervalSteps());
        canvas.drawFastVLine(tx, INTERVAL_SLIDER_Y + SLIDER_TRACK_H + 14, 6, colorBorder);
    }
    drawTrack(INTERVAL_SLIDER_X, INTERVAL_SLIDER_Y, knobX);

    canvas.setTextSize(2);
    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(String(POLL_INTERVAL_MIN_S) + " s", INTERVAL_SLIDER_X, INTERVAL_TICKS_Y);
    canvas.setTextDatum(TR_DATUM);
    canvas.drawString(String(POLL_INTERVAL_MAX_S) + " s", INTERVAL_SLIDER_X + SLIDER_W, INTERVAL_TICKS_Y);

    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("How often the sky is refetched", INTERVAL_SLIDER_X, INTERVAL_CAPS_Y);

    screen::markDirty(COL2_X, top, COL_W, height);
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
    canvas.drawString("Civilian", SEG_X + half / 2, SEG_Y + SEG_H / 2);
    canvas.setTextColor(isMilitary() ? colorWhite : colorDim);
    canvas.drawString("Military", SEG_X + half + half / 2, SEG_Y + SEG_H / 2);

    screen::markDirty(SEG_X, SEG_Y, SEG_W, SEG_H);
}

// Reads as one control with three choices, for the same reason the traffic
// filter does: they are mutually exclusive, and a segmented control says so
// without a caption. "Auto" first because it is what almost everyone wants -
// the named providers are there for pinning one deliberately.
const char *sourceName(int seg) { return seg == 0 ? "Auto" : ADSB_PROVIDERS[seg - 1].name; }

int sourceSegment() { return (int)g_source; }

void drawSourceSegments() {
    auto &canvas = screen::canvas();
    int seg = SRC_W / SRC_SEGMENTS;
    int active = sourceSegment();
    canvas.fillRoundRect(SRC_X, SRC_Y, SRC_W, SRC_H, 8, colorOff);
    canvas.fillRoundRect(SRC_X + active * seg, SRC_Y, seg, SRC_H, 8, colorOn);
    canvas.drawRoundRect(SRC_X, SRC_Y, SRC_W, SRC_H, 8, colorBorder);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(MC_DATUM);
    for (int i = 0; i < SRC_SEGMENTS; i++) {
        canvas.setTextColor(i == active ? colorWhite : colorDim);
        // Provider names are longer than "Civilian"/"Military" and the segments
        // are narrower, so step the text down until it fits rather than let it
        // run into its neighbour.
        String name(sourceName(i));
        int size = 3;
        canvas.setTextSize(size);
        while (size > 1 && canvas.textWidth(name) > seg - 16) {
            canvas.setTextSize(--size);
        }
        canvas.drawString(name, SRC_X + i * seg + seg / 2, SRC_Y + SRC_H / 2);
    }

    screen::markDirty(SRC_X, SRC_Y, SRC_W, SRC_H);
}

// A switch row: its title, a line saying what it does, and the switch.
void drawToggle(int x, int y, int w, int h, const char *title, const char *detail, bool on) {
    auto &canvas = screen::canvas();
    canvas.fillRoundRect(x, y, w, h, 8, colorBtnBg);
    canvas.drawRoundRect(x, y, w, h, 8, colorBorder);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextSize(3);
    canvas.setTextColor(colorWhite);
    canvas.drawString(title, x + 24, y + 20);
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.drawString(detail, x + 24, y + 46);

    int sw = 108, sh = 40;
    int sx = x + w - sw - 20, sy = y + (h - sh) / 2;
    canvas.fillRoundRect(sx, sy, sw, sh, sh / 2, on ? colorOn : colorOff);
    int knob = sh - 8;
    canvas.fillCircle(on ? (sx + sw - 4 - knob / 2) : (sx + 4 + knob / 2), sy + sh / 2, knob / 2, colorWhite);
    canvas.setTextSize(2);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString(on ? "ON" : "OFF", on ? (sx + 30) : (sx + sw - 30), sy + sh / 2);

    screen::markDirty(x, y, w, h);
}

void drawRefreshToggle() {
    drawToggle(REFRESH_X, REFRESH_Y, REFRESH_W, REFRESH_H, "Show flight refresh", "Shades cells that changed",
               g_showRefresh);
}

void drawNavaidsToggle() {
    drawToggle(NAVAIDS_X, NAVAIDS_Y, NAVAIDS_W, NAVAIDS_H, "Navaids", "VOR, DME and TACAN on the radar", g_navaids);
}

// The "leads somewhere else" rows share a shape: a title, a line of
// current state underneath, and a chevron.
void drawNavButton(int x, int y, int w, int h, const char *title, const String &detail) {
    auto &canvas = screen::canvas();
    canvas.fillRoundRect(x, y, w, h, 8, colorBtnBg);
    canvas.drawRoundRect(x, y, w, h, 8, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(ML_DATUM);
    canvas.drawString(title, x + 24, y + 28);

    canvas.setTextColor(colorDim);
    canvas.setTextSize(2);
    // Shrink rather than run under the chevron - place names and SSIDs are
    // whatever length they happen to be.
    int maxW = w - 24 - 48;
    int size = 2;
    while (size > 1 && canvas.textWidth(detail) > maxW) {
        size--;
        canvas.setTextSize(size);
    }
    canvas.drawString(detail, x + 24, y + 60);

    canvas.setTextColor(colorGrey);
    canvas.setTextSize(3);
    canvas.setTextDatum(MR_DATUM);
    canvas.drawString(">", x + w - 24, y + h / 2);
    screen::markDirty(x, y, w, h);
}

void drawLocationButton() {
    drawNavButton(LOC_X, LOC_Y, LOC_W, LOC_H, "Location",
                  g_locationLabel.length() ? g_locationLabel : String("Not set"));
}

void drawWifiButton() {
    drawNavButton(WIFI_X, WIFI_Y, WIFI_W, WIFI_H, "WiFi settings",
                  WiFi.status() == WL_CONNECTED
                      ? ("Connected to " + WiFi.SSID() + "   " + WiFi.localIP().toString())
                      : String("Not connected"));
}

void drawAlertsButton() {
    drawNavButton(ALERTS_X, ALERTS_Y, ALERTS_W, ALERTS_H, "Alerts & recording", g_alertSummary);
}

void sectionLabel(const char *text, int x, int y) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorGrey);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(text, x, y);
}

// True if (x, y) is inside a slider's grab band. Both the column and the band
// matter now that the two sliders sit side by side: the interval slider's band
// runs across the rows the range slider's label occupies.
bool inBand(int x, int y, int colX, int sliderY) {
    return x >= colX && x < colX + COL_W && y >= sliderY - BAND_ABOVE && y < sliderY - BAND_ABOVE + BAND_H;
}

}  // namespace

void settingsScreenSet(TrafficFilter traffic, int radiusNm, bool showRefresh, int pollIntervalS,
                       const String &locationLabel, AdsbSource source) {
    g_traffic = traffic;
    g_radius = radiusNm;
    g_showRefresh = showRefresh;
    g_interval = pollIntervalS;
    g_locationLabel = locationLabel;
    g_source = source;
    g_drag = Drag::NONE;
    clampRadius();
}

void settingsScreenSetLocation(const String &locationLabel) { g_locationLabel = locationLabel; }

void settingsScreenSetNavaids(bool on) { g_navaids = on; }
bool settingsScreenNavaids() { return g_navaids; }

void settingsScreenSetAlertSummary(const String &summary) { g_alertSummary = summary; }

int settingsScreenPollInterval() { return g_interval; }

void settingsScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Settings", 16, 12);
    // Which release is running, beside the title and set lower to share its baseline.
    int versionX = 16 + canvas.textWidth("Settings") + 16;
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.drawString("v" FIRMWARE_VERSION, versionX, 20);
    canvas.setTextColor(colorWhite);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    canvas.fillRoundRect(ABOUT_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.drawString("About", ABOUT_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    canvas.fillRoundRect(HELP_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.drawString("Help", HELP_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    sectionLabel("SHOW", COL1_X, SHOW_LABEL_Y);
    drawSegments();
    sectionLabel("RANGE", COL1_X, RADIUS_LABEL_Y);
    drawRadiusSlider();
    sectionLabel("DISPLAY", COL1_X, DISPLAY_LABEL_Y);
    drawRefreshToggle();
    drawNavaidsToggle();
    sectionLabel("DATA SOURCE", COL1_X, SOURCE_LABEL_Y);
    drawSourceSegments();

    sectionLabel("REFRESH", COL2_X, INTERVAL_LABEL_Y);
    drawIntervalSlider();
    sectionLabel("LOCATION", COL2_X, LOC_LABEL_Y);
    drawLocationButton();
    sectionLabel("NETWORK", COL2_X, WIFI_LABEL_Y);
    drawWifiButton();
    sectionLabel("ALERTS", COL2_X, ALERTS_LABEL_Y);
    drawAlertsButton();

    screen::flush();
}

SettingsAction settingsScreenHandleTouch(int x, int y, bool pressed, bool clicked) {
    // Sliders first: a held finger keeps control of the one it started on even
    // once it wanders outside the band, which is what makes dragging feel right.
    if (g_drag == Drag::RADIUS || (g_drag == Drag::NONE && pressed && inBand(x, y, COL1_X, RADIUS_SLIDER_Y))) {
        if (pressed) {
            g_drag = Drag::RADIUS;
            int span = maxRadius() - RADIUS_MIN_NM;
            int pos = x - RADIUS_SLIDER_X;
            if (pos < 0) pos = 0;
            if (pos > SLIDER_W) pos = SLIDER_W;
            int value = RADIUS_MIN_NM + (int)(((int64_t)pos * span + SLIDER_W / 2) / SLIDER_W);
            if (value != g_radius) {
                g_radius = value;
                drawRadiusSlider();
                screen::flush();
            }
            return SettingsAction::NONE;
        }
        g_drag = Drag::NONE;
        return SettingsAction::NONE;
    }

    if (g_drag == Drag::INTERVAL || (g_drag == Drag::NONE && pressed && inBand(x, y, COL2_X, INTERVAL_SLIDER_Y))) {
        if (pressed) {
            g_drag = Drag::INTERVAL;
            int pos = x - INTERVAL_SLIDER_X;
            if (pos < 0) pos = 0;
            if (pos > SLIDER_W) pos = SLIDER_W;
            int idx = (int)(((int64_t)pos * intervalSteps() + SLIDER_W / 2) / SLIDER_W);
            int value = POLL_INTERVAL_MIN_S + idx * POLL_INTERVAL_STEP_S;
            if (value != g_interval) {
                g_interval = value;
                drawIntervalSlider();
                screen::flush();
            }
            return SettingsAction::NONE;
        }
        g_drag = Drag::NONE;
        return SettingsAction::NONE;
    }

    if (!clicked) {
        return SettingsAction::NONE;
    }

    if (x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return SettingsAction::BACK;
    }
    if (x >= ABOUT_X && x < ABOUT_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return SettingsAction::OPEN_ABOUT;
    }
    if (x >= HELP_X && x < HELP_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return SettingsAction::OPEN_HELP;
    }

    if (y >= SEG_Y && y < SEG_Y + SEG_H && x >= SEG_X && x < SEG_X + SEG_W) {
        TrafficFilter picked = (x < SEG_X + SEG_W / 2) ? TrafficFilter::CIVIL : TrafficFilter::MILITARY;
        if (picked != g_traffic) {
            g_traffic = picked;
            clampRadius();  // the ceiling moves with the choice
            drawSegments();
            drawRadiusSlider();
            screen::flush();
        }
        return SettingsAction::NONE;
    }

    if (x >= REFRESH_X && x < REFRESH_X + REFRESH_W && y >= REFRESH_Y && y < REFRESH_Y + REFRESH_H) {
        g_showRefresh = !g_showRefresh;
        drawRefreshToggle();
        screen::flush();
        return SettingsAction::NONE;
    }

    if (x >= NAVAIDS_X && x < NAVAIDS_X + NAVAIDS_W && y >= NAVAIDS_Y && y < NAVAIDS_Y + NAVAIDS_H) {
        g_navaids = !g_navaids;
        drawNavaidsToggle();
        screen::flush();
        return SettingsAction::NONE;
    }

    if (y >= SRC_Y && y < SRC_Y + SRC_H && x >= SRC_X && x < SRC_X + SRC_W) {
        int seg = (x - SRC_X) / (SRC_W / SRC_SEGMENTS);
        if (seg >= SRC_SEGMENTS) {
            seg = SRC_SEGMENTS - 1;  // the rounding slack at the right edge
        }
        AdsbSource picked = (AdsbSource)seg;
        if (picked != g_source) {
            g_source = picked;
            drawSourceSegments();
            screen::flush();
        }
        return SettingsAction::NONE;
    }

    if (x >= LOC_X && x < LOC_X + LOC_W && y >= LOC_Y && y < LOC_Y + LOC_H) {
        return SettingsAction::OPEN_LOCATION;
    }

    if (x >= WIFI_X && x < WIFI_X + WIFI_W && y >= WIFI_Y && y < WIFI_Y + WIFI_H) {
        return SettingsAction::OPEN_WIFI;
    }

    if (x >= ALERTS_X && x < ALERTS_X + ALERTS_W && y >= ALERTS_Y && y < ALERTS_Y + ALERTS_H) {
        return SettingsAction::OPEN_ALERTS;
    }
    return SettingsAction::NONE;
}

TrafficFilter settingsScreenTraffic() { return g_traffic; }
int settingsScreenRadius() { return g_radius; }
bool settingsScreenShowRefresh() { return g_showRefresh; }
AdsbSource settingsScreenSource() { return g_source; }
