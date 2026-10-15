#include "alerts_screen.h"

#include <M5Unified.h>

#include <algorithm>

#include "alerts.h"
#include "recorder.h"
#include "screen.h"
#include "sound.h"

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
// Under them, the voice: read aloud, and the voice for everywhere without
// its own.
constexpr int VOICE_LABEL_Y = ALERT_TOGGLE_Y + ALERT_COUNT * TOGGLE_STEP + 8;
constexpr int READ_Y = VOICE_LABEL_Y + 28;
constexpr int ACCENT_LABEL_Y = READ_Y + TOGGLE_H + 12;
constexpr int ACCENT_Y = ACCENT_LABEL_Y + 28, ACCENT_H = 64;
const char *const ACCENTS[] = {"Scottish", "English", "American"};  // in Accent's order
constexpr int ACCENT_COUNT = 3;

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
    {ALERT_EMERGENCY, "Emergency squawks", "Squawk 7700, 7600 or 7500"},
    {ALERT_MILITARY, "Military", "Flags military passing through"},
    {ALERT_RARE, "Rare and notable types", "A380, 747, Beluga, C-17..."},
    {ALERT_WATCHLIST, "Watchlist", "The entries listed on the right"},
};

uint8_t g_mask = ALERT_ALL;
bool g_autoRecord = false;
bool g_autoFollow = false;
bool g_readAloud = false;
Accent g_accent = DEFAULT_ELSEWHERE_ACCENT;
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

// `text` as it fits in `maxW` at the size set - cut short with "..." if it
// doesn't, rather than run under what is beside it.
String fitted(const String &text, int maxW) {
    auto &canvas = screen::canvas();
    if (canvas.textWidth(text) <= maxW) {
        return text;
    }
    String s = text;
    while (s.length() > 1 && canvas.textWidth(s + "...") > maxW) {
        s.remove(s.length() - 1);
    }
    return s + "...";
}

// The same switch the settings screen's refresh toggle is - its words kept
// to the room left of the switch, which some ran under: 33 characters of
// the line under the title.
void drawToggle(int x, int y, const char *title, const char *detail, bool on, bool enabled = true) {
    auto &canvas = screen::canvas();
    canvas.fillRoundRect(x, y, COL_W, TOGGLE_H, 8, colorBtnBg);
    canvas.drawRoundRect(x, y, COL_W, TOGGLE_H, 8, colorBorder);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(ML_DATUM);
    int sw = 108, sh = 40;
    int sx = x + COL_W - sw - 20, sy = y + (TOGGLE_H - sh) / 2;
    int room = sx - 12 - (x + 24);
    canvas.setTextSize(3);
    canvas.setTextColor(enabled ? colorWhite : colorDim);
    canvas.drawString(fitted(title, room), x + 24, y + 26);
    canvas.setTextSize(2);
    canvas.setTextColor(colorDim);
    canvas.drawString(fitted(detail, room), x + 24, y + 54);

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
               g_cardPresent ? "Records the radar to the SD card" : "Insert an SD card to record",
               g_autoRecord, g_cardPresent);
}

void drawAutoFollow() {
    drawToggle(COL2_X, AUTOFOLLOW_Y, "Auto-follow alerts", "Follows the alerted flight", g_autoFollow);
}

void drawReadAloud() {
    drawToggle(COL1_X, READ_Y, "Read aloud", "Details and frequency cards", g_readAloud);
}

// As the settings screen's choices are: one control, one of its three lit.
void drawAccents() {
    auto &canvas = screen::canvas();
    int seg = COL_W / ACCENT_COUNT;
    canvas.fillRoundRect(COL1_X, ACCENT_Y, COL_W, ACCENT_H, 8, colorOff);
    canvas.fillRoundRect(COL1_X + (int)g_accent * seg, ACCENT_Y, seg, ACCENT_H, 8, colorOn);
    canvas.drawRoundRect(COL1_X, ACCENT_Y, COL_W, ACCENT_H, 8, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(3);
    canvas.setTextDatum(MC_DATUM);
    for (int i = 0; i < ACCENT_COUNT; i++) {
        canvas.setTextColor(i == (int)g_accent ? colorWhite : colorDim);
        canvas.drawString(ACCENTS[i], COL1_X + i * seg + seg / 2, ACCENT_Y + ACCENT_H / 2);
    }
    screen::markDirty(COL1_X, ACCENT_Y, COL_W, ACCENT_H);
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

void alertsScreenSetVoice(bool readAloud, Accent elsewhere) {
    g_readAloud = readAloud;
    g_accent = elsewhere;
}

void alertsScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Alerts, voice & recording", 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    sectionLabel("ALERT ON", COL1_X, ALERT_LABEL_Y);
    for (int i = 0; i < ALERT_COUNT; i++) {
        drawAlertToggle(i);
    }
    sectionLabel("VOICE", COL1_X, VOICE_LABEL_Y);
    drawReadAloud();
    sectionLabel("FOR COUNTRIES WITHOUT THEIR OWN", COL1_X, ACCENT_LABEL_Y);
    drawAccents();

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
    if (inside(x, y, COL1_X, READ_Y, COL_W, TOGGLE_H)) {
        g_readAloud = !g_readAloud;
        drawReadAloud();
        screen::flush();
        return AlertsAction::NONE;
    }
    if (inside(x, y, COL1_X, ACCENT_Y, COL_W, ACCENT_H)) {
        // Said in it as it is picked, to hear what it is.
        g_accent = (Accent)std::min((x - COL1_X) * ACCENT_COUNT / COL_W, ACCENT_COUNT - 1);
        drawAccents();
        screen::flush();
        soundSampleAccent(g_accent);
        return AlertsAction::NONE;
    }
    return AlertsAction::NONE;
}

uint8_t alertsScreenMask() { return g_mask; }
bool alertsScreenAutoRecord() { return g_autoRecord; }
bool alertsScreenAutoFollow() { return g_autoFollow; }
String alertsScreenWatchlist() { return g_watchlist; }
bool alertsScreenReadAloud() { return g_readAloud; }
Accent alertsScreenElsewhereAccent() { return g_accent; }
