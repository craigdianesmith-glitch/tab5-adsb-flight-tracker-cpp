#include "alerts_screen.h"

#include <M5Unified.h>

#include "alerts.h"
#include "recorder.h"
#include "screen.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;

// Two columns, as on the settings screen this is reached from.
constexpr int COL_W = 560;
constexpr int COL1_X = 40;
constexpr int COL2_X = 680;
constexpr int TOGGLE_H = 76, TOGGLE_STEP = 88;

// --- left column: what raises an alert ---
constexpr int ALERT_LABEL_Y = 84;
constexpr int ALERT_TOGGLE_Y = 112;
constexpr int ALERT_COUNT = 4;

// --- right column ---
constexpr int WATCH_LABEL_Y = 84;
constexpr int WATCH_X = COL2_X, WATCH_Y = 112, WATCH_W = COL_W, WATCH_H = 140;

constexpr int REC_LABEL_Y = 284;
constexpr int AUTOREC_Y = 312;
constexpr int AUTOFOLLOW_Y = AUTOREC_Y + TOGGLE_STEP;
constexpr int REC_NOTE_Y = AUTOFOLLOW_Y + TOGGLE_H + 14;

struct AlertToggle {
    uint8_t bit;
    const char *title;
    const char *detail;
};
const AlertToggle TOGGLES[ALERT_COUNT] = {
    {ALERT_EMERGENCY, "Emergency squawks", "7700 emergency, 7600 radio, 7500 hijack"},
    {ALERT_MILITARY, "Military in civil mode", "Shows and flags military passing through"},
    {ALERT_RARE, "Rare and notable types", "A380, 747, Beluga, C-17, warbirds..."},
    {ALERT_WATCHLIST, "Watchlist", "The entries listed on the right"},
};

uint8_t g_mask = ALERT_ALL;
bool g_autoRecord = false;
bool g_autoFollow = false;
bool g_cardPresent = true;
String g_watchlist;

uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorOn, colorOff, colorBorder;

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
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorsReady = true;
}

void sectionLabel(const char *text, int x, int y) {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorGrey);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(text, x, y);
}

// The same switch the settings screen's refresh toggle is.
void drawToggle(int x, int y, const char *title, const char *detail, bool on, bool enabled = true) {
    auto &canvas = screen::canvas();
    canvas.fillRoundRect(x, y, COL_W, TOGGLE_H, 8, colorBtnBg);
    canvas.drawRoundRect(x, y, COL_W, TOGGLE_H, 8, colorBorder);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextSize(3);
    canvas.setTextColor(enabled ? colorWhite : colorDim);
    canvas.drawString(title, x + 24, y + 26);
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.drawString(detail, x + 24, y + 54);

    int sw = 108, sh = 40;
    int sx = x + COL_W - sw - 20, sy = y + (TOGGLE_H - sh) / 2;
    canvas.fillRoundRect(sx, sy, sw, sh, sh / 2, on && enabled ? colorOn : colorOff);
    int knob = sh - 8;
    canvas.fillCircle(on ? (sx + sw - 4 - knob / 2) : (sx + 4 + knob / 2), sy + sh / 2, knob / 2,
                      enabled ? colorWhite : colorDim);
    canvas.setTextSize(2);
    canvas.setTextColor(enabled ? colorWhite : colorDim);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString(on ? "ON" : "OFF", on ? (sx + 30) : (sx + sw - 30), sy + sh / 2);

    screen::markDirty(x, y, COL_W, TOGGLE_H);
}

void drawAlertToggle(int i) {
    drawToggle(COL1_X, ALERT_TOGGLE_Y + i * TOGGLE_STEP, TOGGLES[i].title, TOGGLES[i].detail,
               (g_mask & TOGGLES[i].bit) != 0);
}

void drawAutoRecord() {
    drawToggle(COL2_X, AUTOREC_Y, "Auto-record alerts",
               g_cardPresent ? "Records the radar to the SD card" : "No SD card found - insert one to record",
               g_autoRecord, g_cardPresent);
}

void drawAutoFollow() {
    drawToggle(COL2_X, AUTOFOLLOW_Y, "Auto-follow alerts", "Follows the alerted flight on the radar", g_autoFollow);
}

// The entries themselves, wrapped onto as many lines as the button holds, so
// what is being watched for can be seen without opening the editor.
void drawWatchButton() {
    auto &canvas = screen::canvas();
    canvas.fillRoundRect(WATCH_X, WATCH_Y, WATCH_W, WATCH_H, 8, colorBtnBg);
    canvas.drawRoundRect(WATCH_X, WATCH_Y, WATCH_W, WATCH_H, 8, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextSize(2);

    std::vector<String> entries = parseWatchlist(g_watchlist);
    int textW = WATCH_W - 24 - 56;
    if (entries.empty()) {
        canvas.setTextColor(colorDim);
        canvas.drawString("Empty - tap to add callsigns,", WATCH_X + 24, WATCH_Y + 24);
        canvas.drawString("airlines, registrations or types", WATCH_X + 24, WATCH_Y + 50);
    } else {
        canvas.setTextColor(colorWhite);
        constexpr int LINE_H = 26, MAX_LINES = 4;
        int line = 0;
        String text;
        for (size_t i = 0; i < entries.size() && line < MAX_LINES; i++) {
            String next = text.length() ? text + "  " + entries[i] : entries[i];
            if (canvas.textWidth(next) > textW && text.length()) {
                canvas.drawString(text, WATCH_X + 24, WATCH_Y + 20 + line * LINE_H);
                line++;
                text = entries[i];
            } else {
                text = next;
            }
        }
        if (line < MAX_LINES && text.length()) {
            canvas.drawString(text, WATCH_X + 24, WATCH_Y + 20 + line * LINE_H);
        }
    }

    canvas.setTextColor(colorGrey);
    canvas.setTextSize(3);
    canvas.setTextDatum(MR_DATUM);
    canvas.drawString(">", WATCH_X + WATCH_W - 24, WATCH_Y + WATCH_H / 2);
    screen::markDirty(WATCH_X, WATCH_Y, WATCH_W, WATCH_H);
}

void drawRecordingNote() {
    auto &canvas = screen::canvas();
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextColor(colorDim);
    // Auto-record alone records the radar until a minute after the last
    // alerted aircraft has gone; with auto-follow, it follows the one alerted
    // and stops once it has landed or left the radius.
    const char *lines[] = {
        "Records until the alerted flight has gone,",
        "or with auto-follow, landed or left. Play",
        "them from Replays on the radar screen.",
    };
    for (int i = 0; i < 3; i++) {
        canvas.drawString(lines[i], COL2_X, REC_NOTE_Y + i * 26);
    }

    // From what the screen was given rather than a fresh look at the card,
    // which could find one the greyed-out switch above says isn't there.
    String card;
    if (g_cardPresent) {
        card = "SD card: " + String((float)(recorder::cardBytes() / (1024 * 1024)) / 1024.0f, 0) + " GB, ready";
        canvas.setTextColor(colorOn);
    } else {
        card = "SD card: none found";
        canvas.setTextColor(M5.Display.color565(0xF3, 0x9C, 0x12));
    }
    canvas.drawString(card, COL2_X, REC_NOTE_Y + 3 * 26 + 10);
}

bool inside(int x, int y, int rx, int ry, int rw, int rh) { return x >= rx && x < rx + rw && y >= ry && y < ry + rh; }

}  // namespace

void alertsScreenSetCard(bool present) { g_cardPresent = present; }

void alertsScreenSet(uint8_t alertMask, bool autoRecord, bool autoFollow, const String &watchlist) {
    g_mask = alertMask;
    g_autoRecord = autoRecord;
    g_autoFollow = autoFollow;
    g_watchlist = watchlist;
}

void alertsScreenSetWatchlist(const String &watchlist) { g_watchlist = watchlist; }

void alertsScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Alerts & recording", 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    sectionLabel("ALERT ON", COL1_X, ALERT_LABEL_Y);
    for (int i = 0; i < ALERT_COUNT; i++) {
        drawAlertToggle(i);
    }
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.setTextDatum(TL_DATUM);
    int noteY = ALERT_TOGGLE_Y + ALERT_COUNT * TOGGLE_STEP + 8;
    canvas.drawString("An alert shows a banner over the table or", COL1_X, noteY);
    canvas.drawString("radar, chimes, and marks the aircraft in", COL1_X, noteY + 26);
    canvas.drawString("amber - red for an emergency. Tap the", COL1_X, noteY + 52);
    canvas.drawString("banner to see the aircraft's details.", COL1_X, noteY + 78);

    sectionLabel("WATCHLIST", COL2_X, WATCH_LABEL_Y);
    drawWatchButton();
    sectionLabel("RECORDING AND FOLLOWING", COL2_X, REC_LABEL_Y);
    drawAutoRecord();
    drawAutoFollow();
    drawRecordingNote();

    screen::flush();
}

AlertsAction alertsScreenHandleTouch(int x, int y) {
    if (inside(x, y, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        return AlertsAction::BACK;
    }
    for (int i = 0; i < ALERT_COUNT; i++) {
        if (inside(x, y, COL1_X, ALERT_TOGGLE_Y + i * TOGGLE_STEP, COL_W, TOGGLE_H)) {
            g_mask ^= TOGGLES[i].bit;
            drawAlertToggle(i);
            screen::flush();
            return AlertsAction::NONE;
        }
    }
    if (inside(x, y, COL2_X, AUTOREC_Y, COL_W, TOGGLE_H) && g_cardPresent) {
        g_autoRecord = !g_autoRecord;
        drawAutoRecord();
        screen::flush();
        return AlertsAction::NONE;
    }
    if (inside(x, y, COL2_X, AUTOFOLLOW_Y, COL_W, TOGGLE_H)) {
        g_autoFollow = !g_autoFollow;
        drawAutoFollow();
        screen::flush();
        return AlertsAction::NONE;
    }
    if (inside(x, y, WATCH_X, WATCH_Y, WATCH_W, WATCH_H)) {
        return AlertsAction::OPEN_WATCHLIST;
    }
    return AlertsAction::NONE;
}

uint8_t alertsScreenMask() { return g_mask; }
bool alertsScreenAutoRecord() { return g_autoRecord; }
bool alertsScreenAutoFollow() { return g_autoFollow; }
String alertsScreenWatchlist() { return g_watchlist; }
