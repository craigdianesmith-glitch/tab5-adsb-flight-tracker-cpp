#include "share_screen.h"

#include <M5Unified.h>
#include <WiFi.h>

#include "screen.h"
#include "share_server.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
// The code as large as the space allows: a phone across the room should read
// it. Version 3 holds well over the longest address it is given.
constexpr int QR_X = 60, QR_Y = 100, QR_W = 440;
constexpr int TEXT_X = 560;
constexpr int STATUS_Y = 600, STATUS_H = 60;

bool g_started = false;
share::Status g_shown = {};  // what the status line says now

uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorAccent, colorWarn;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorGrey = M5.Display.color565(0xBB, 0xBB, 0xBB);
    colorDim = M5.Display.color565(0x88, 0x91, 0x9B);
    colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorAccent = M5.Display.color565(0x2E, 0xCC, 0x71);
    colorWarn = M5.Display.color565(0xF3, 0x9C, 0x12);
    colorsReady = true;
}

String mb(uint32_t bytes) { return String(bytes / (1024.0f * 1024.0f), 1) + " MB"; }

void drawStatus() {
    auto &canvas = screen::canvas();
    canvas.fillRect(TEXT_X, STATUS_Y, 1280 - TEXT_X, STATUS_H, colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(TL_DATUM);
    canvas.setTextSize(2);
    if (g_shown.current[0]) {
        canvas.setTextColor(colorAccent);
        canvas.drawString(String("Sending ") + g_shown.current, TEXT_X, STATUS_Y);
        canvas.setTextColor(colorGrey);
        canvas.drawString(mb(g_shown.sent) + " of " + mb(g_shown.total), TEXT_X, STATUS_Y + 26);
    } else {
        canvas.setTextColor(colorDim);
        canvas.drawString(g_shown.requests == 0 ? String("Waiting for a phone or computer to connect")
                                                : "Connected - " + String(g_shown.requests) + " requests served",
                          TEXT_X, STATUS_Y);
    }
    screen::markDirty(TEXT_X, STATUS_Y, 1280 - TEXT_X, STATUS_H);
}

}  // namespace

void shareScreenEnter() {
    g_started = share::start();
    // Drawn from the server's own count, not a sentinel meant to force the
    // first update: the screen is drawn before that update, and for a moment
    // said four billion requests had been served.
    g_shown = share::status();
    shareScreenDraw();
}

void shareScreenLeave() {
    share::stop();
    g_started = false;
}

void shareScreenDraw() {
    ensureColors();
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("Share over WiFi", 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);
    canvas.setTextDatum(TL_DATUM);

    if (!g_started) {
        canvas.setTextColor(colorWarn);
        canvas.setTextSize(3);
        canvas.drawString(WiFi.status() == WL_CONNECTED ? "Couldn't start sharing" : "Not connected to WiFi", 60, 140);
        canvas.setTextColor(colorGrey);
        canvas.setTextSize(2);
        canvas.drawString("Sharing needs the tracker on the same WiFi network as your phone.", 60, 200);
        canvas.drawString("Connect it under Settings > WiFi, then come back here.", 60, 230);
        screen::flush();
        return;
    }

    String url = share::url();
    // The code is drawn by the library, dark on light with its own quiet zone,
    // which is what a phone camera expects whatever the screen around it is.
    canvas.qrcode(url.c_str(), QR_X, QR_Y, QR_W, 3, true);

    canvas.setTextColor(colorGrey);
    canvas.setTextSize(2);
    canvas.drawString("Scan the code with your phone's camera,", TEXT_X, 110);
    canvas.drawString("or open this address in a browser:", TEXT_X, 136);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(4);
    canvas.drawString(url, TEXT_X, 180);
    String local = share::localUrl();
    if (local.length()) {
        canvas.setTextColor(colorGrey);
        canvas.setTextSize(2);
        canvas.drawString("or " + local, TEXT_X, 232);
    }

    canvas.setTextColor(colorDim);
    canvas.setTextSize(2);
    canvas.drawString("Your phone has to be on the same WiFi:", TEXT_X, 300);
    canvas.setTextColor(colorGrey);
    canvas.drawString(WiFi.SSID(), TEXT_X, 326);
    canvas.setTextColor(colorDim);
    canvas.drawString("Videos can be played, downloaded or", TEXT_X, 390);
    canvas.drawString("deleted there; recordings downloaded.", TEXT_X, 416);
    canvas.drawString("Sharing stops when you leave this screen.", TEXT_X, 468);

    drawStatus();
    screen::flush();
}

void shareScreenTick() {
    if (!g_started) {
        return;
    }
    static uint32_t lastCheck = 0;
    uint32_t now = millis();
    if (now - lastCheck < 300) {
        return;
    }
    lastCheck = now;
    share::Status s = share::status();
    // Redrawn when what it says has changed, with the byte count steadied to
    // a tenth of a megabyte so a transfer doesn't repaint it every check.
    bool changed = s.requests != g_shown.requests || strcmp(s.current, g_shown.current) != 0 ||
                   s.sent / 104858 != g_shown.sent / 104858;
    if (changed) {
        g_shown = s;
        drawStatus();
        screen::flush();
    }
}

ShareAction shareScreenHandleTouch(int x, int y) {
    if (x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        return ShareAction::BACK;
    }
    return ShareAction::NONE;
}
