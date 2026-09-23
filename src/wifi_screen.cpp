#include "wifi_screen.h"

#include <M5Unified.h>
#include <WiFi.h>
#include <vector>

#include "keyboard.h"
#include "screen.h"
#include "settings.h"

namespace {

bool colorsReady = false;

constexpr int BACK_X = 1140, BACK_Y = 8, BACK_W = 124, BACK_H = 44;
constexpr int STATUS_X = 40, STATUS_Y = 76, STATUS_W = 1200, STATUS_H = 30;

// network list
constexpr int SCAN_X = 1000, SCAN_Y = 124, SCAN_W = 240, SCAN_H = 56;
constexpr int LIST_X = 40, LIST_Y = 200, LIST_W = 1200, LIST_ROW_H = 60, MAX_ROWS = 7;

// password entry
constexpr int PW_X = 40, PW_Y = 200, PW_W = 900, PW_H = 60;
constexpr int CONNECT_X = 980, CONNECT_Y = 200, CONNECT_W = 260, CONNECT_H = 60;
constexpr int PW_HINT_Y = 300;

enum class Mode { LIST, PASSWORD };
Mode g_mode = Mode::LIST;

struct Network {
    String ssid;
    int32_t rssi;
    bool open;
};
std::vector<Network> g_networks;

bool g_scanning = false;
String g_status;
String g_ssid;
String g_password;
uint32_t g_connectStartedAt = 0;
bool g_connecting = false;

uint16_t colorBg, colorWhite, colorGrey, colorDim, colorBtnBg, colorScanBtn, colorConnect, colorRow, colorBorder;

void ensureColors() {
    if (colorsReady) {
        return;
    }
    colorBg = M5.Display.color565(0x10, 0x14, 0x18);
    colorWhite = M5.Display.color565(0xFF, 0xFF, 0xFF);
    colorGrey = M5.Display.color565(0xBB, 0xBB, 0xBB);
    colorDim = M5.Display.color565(0x77, 0x7F, 0x88);
    colorBtnBg = M5.Display.color565(0x2C, 0x3E, 0x50);
    colorScanBtn = M5.Display.color565(0x29, 0x80, 0xB9);
    colorConnect = M5.Display.color565(0x27, 0xAE, 0x60);
    colorRow = M5.Display.color565(0x30, 0x36, 0x3D);
    colorBorder = M5.Display.color565(0x44, 0x44, 0x44);
    colorsReady = true;
}

String connectionSummary() {
    if (WiFi.status() == WL_CONNECTED) {
        return "Connected to " + WiFi.SSID() + "   " + WiFi.localIP().toString();
    }
    return "Not connected";
}

void startScan() {
    g_networks.clear();
    g_scanning = true;
    g_status = "Scanning...";
    WiFi.scanDelete();
    WiFi.scanNetworks(true);  // async: results collected in wifiScreenTick()
}

void drawStatus() {
    auto &canvas = screen::canvas();
    canvas.fillRect(STATUS_X, STATUS_Y, STATUS_W, STATUS_H, colorBg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextColor(colorGrey);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(g_status.length() ? g_status : connectionSummary(), STATUS_X, STATUS_Y);
    screen::markDirty(STATUS_X, STATUS_Y, STATUS_W, STATUS_H);
}

// Four bars, filled according to RSSI - easier to read at a glance than a
// number, and the number is there anyway.
void drawSignal(int x, int cy, int32_t rssi) {
    auto &canvas = screen::canvas();
    int bars = rssi >= -55 ? 4 : rssi >= -67 ? 3 : rssi >= -78 ? 2 : 1;
    for (int i = 0; i < 4; i++) {
        int h = 8 + i * 7;
        canvas.fillRect(x + i * 12, cy + 14 - h, 8, h, i < bars ? colorWhite : colorBorder);
    }
}

void drawList() {
    auto &canvas = screen::canvas();
    canvas.fillRect(LIST_X, LIST_Y, LIST_W, MAX_ROWS * LIST_ROW_H, colorBg);
    canvas.setFont(&fonts::Font0);

    int rows = (int)g_networks.size();
    if (rows > MAX_ROWS) {
        rows = MAX_ROWS;
    }
    for (int i = 0; i < rows; i++) {
        int y = LIST_Y + i * LIST_ROW_H;
        canvas.fillRoundRect(LIST_X, y, LIST_W, LIST_ROW_H - 6, 6, colorRow);
        canvas.setTextSize(2);
        canvas.setTextColor(colorWhite);
        canvas.setTextDatum(ML_DATUM);
        canvas.drawString(g_networks[i].ssid, LIST_X + 20, y + (LIST_ROW_H - 6) / 2);

        canvas.setTextSize(1);
        canvas.setTextColor(colorDim);
        canvas.setTextDatum(MR_DATUM);
        canvas.drawString(g_networks[i].open ? "open" : "secured", LIST_X + LIST_W - 130,
                          y + (LIST_ROW_H - 6) / 2);
        drawSignal(LIST_X + LIST_W - 90, y + (LIST_ROW_H - 6) / 2, g_networks[i].rssi);
    }
    if (rows == 0 && !g_scanning) {
        canvas.setTextSize(2);
        canvas.setTextColor(colorDim);
        canvas.setTextDatum(TL_DATUM);
        canvas.drawString("No networks found - tap Scan to try again", LIST_X, LIST_Y + 8);
    }
    screen::markDirty(LIST_X, LIST_Y, LIST_W, MAX_ROWS * LIST_ROW_H);
}

void drawPasswordField() {
    auto &canvas = screen::canvas();
    canvas.fillRect(PW_X, PW_Y, PW_W, PW_H, colorBg);
    canvas.drawRect(PW_X, PW_Y, PW_W, PW_H, colorBorder);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(2);
    canvas.setTextDatum(ML_DATUM);
    if (g_password.length()) {
        canvas.setTextColor(colorWhite);
        canvas.drawString(g_password, PW_X + 12, PW_Y + PW_H / 2);
    } else {
        canvas.setTextColor(colorDim);
        canvas.drawString("Password", PW_X + 12, PW_Y + PW_H / 2);
    }
    screen::markDirty(PW_X, PW_Y, PW_W, PW_H);
}

void drawListScreen() {
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("WiFi", 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    canvas.fillRoundRect(SCAN_X, SCAN_Y, SCAN_W, SCAN_H, 6, colorScanBtn);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Scan", SCAN_X + SCAN_W / 2, SCAN_Y + SCAN_H / 2);

    canvas.setTextColor(colorGrey);
    canvas.setTextSize(2);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("NETWORKS", LIST_X, SCAN_Y + 20);

    drawStatus();
    drawList();
    screen::flush();
}

void drawPasswordScreen() {
    auto &canvas = screen::canvas();
    screen::clear(colorBg);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(colorWhite);
    canvas.setTextSize(3);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString(g_ssid, 16, 12);

    canvas.fillRoundRect(BACK_X, BACK_Y, BACK_W, BACK_H, 6, colorBtnBg);
    canvas.setTextSize(2);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Back", BACK_X + BACK_W / 2, BACK_Y + BACK_H / 2);

    canvas.fillRoundRect(CONNECT_X, CONNECT_Y, CONNECT_W, CONNECT_H, 6, colorConnect);
    canvas.setTextColor(colorWhite);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("Connect", CONNECT_X + CONNECT_W / 2, CONNECT_Y + CONNECT_H / 2);

    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setTextDatum(TL_DATUM);
    canvas.drawString("The passphrase is shown as typed - there's no second chance to check it later.", PW_X,
                      PW_HINT_Y);

    drawStatus();
    drawPasswordField();
    keyboard::draw();
    screen::flush();
}

void beginConnect() {
    saveWifi(g_ssid, g_password);
    g_connecting = true;
    g_connectStartedAt = millis();
    g_status = "Connecting to " + g_ssid + "...";
    drawStatus();
    screen::flush();
    WiFi.disconnect();
    WiFi.begin(g_ssid.c_str(), g_password.c_str());
}

}  // namespace

void wifiScreenEnter() {
    ensureColors();
    keyboard::reset();
    g_mode = Mode::LIST;
    g_password = "";
    g_connecting = false;
    g_status = "";
    startScan();
    drawListScreen();
}

void wifiScreenDraw() {
    ensureColors();
    if (g_mode == Mode::LIST) {
        drawListScreen();
    } else {
        drawPasswordScreen();
    }
}

void wifiScreenTick() {
    if (g_scanning) {
        int found = WiFi.scanComplete();
        if (found >= 0) {
            g_scanning = false;
            g_networks.clear();
            for (int i = 0; i < found; i++) {
                // The scan is already sorted strongest-first; skip hidden SSIDs
                // and any duplicate a multi-AP network reports.
                String ssid = WiFi.SSID(i);
                if (ssid.length() == 0) {
                    continue;
                }
                bool seen = false;
                for (auto &n : g_networks) {
                    if (n.ssid == ssid) {
                        seen = true;
                        break;
                    }
                }
                if (seen) {
                    continue;
                }
                g_networks.push_back({ssid, WiFi.RSSI(i), WiFi.encryptionType(i) == WIFI_AUTH_OPEN});
            }
            WiFi.scanDelete();
            g_status = "";
            if (g_mode == Mode::LIST) {
                drawStatus();
                drawList();
                screen::flush();
            }
        } else if (found == WIFI_SCAN_FAILED) {
            g_scanning = false;
            g_status = "Scan failed";
            if (g_mode == Mode::LIST) {
                drawStatus();
                screen::flush();
            }
        }
    }

    if (g_connecting) {
        if (WiFi.status() == WL_CONNECTED) {
            g_connecting = false;
            g_status = "";
            drawStatus();
            screen::flush();
        } else if (millis() - g_connectStartedAt > 20000) {
            g_connecting = false;
            g_status = "Could not connect to " + g_ssid;
            drawStatus();
            screen::flush();
        }
    }
}

WifiAction wifiScreenHandleTouch(int x, int y) {
    if (x >= BACK_X && x < BACK_X + BACK_W && y >= BACK_Y && y < BACK_Y + BACK_H) {
        if (g_mode == Mode::PASSWORD) {
            g_mode = Mode::LIST;  // back out to the list, not off the screen
            drawListScreen();
            return WifiAction::NONE;
        }
        return WifiAction::BACK;
    }

    if (g_mode == Mode::LIST) {
        if (x >= SCAN_X && x < SCAN_X + SCAN_W && y >= SCAN_Y && y < SCAN_Y + SCAN_H) {
            startScan();
            drawStatus();
            drawList();
            screen::flush();
            return WifiAction::NONE;
        }
        if (x >= LIST_X && x < LIST_X + LIST_W && y >= LIST_Y) {
            int row = (y - LIST_Y) / LIST_ROW_H;
            if (row >= 0 && row < (int)g_networks.size() && row < MAX_ROWS) {
                g_ssid = g_networks[row].ssid;
                g_password = "";
                g_status = "";
                keyboard::reset();
                if (g_networks[row].open) {
                    beginConnect();  // nothing to type
                    return WifiAction::NONE;
                }
                g_mode = Mode::PASSWORD;
                drawPasswordScreen();
            }
        }
        return WifiAction::NONE;
    }

    if (x >= CONNECT_X && x < CONNECT_X + CONNECT_W && y >= CONNECT_Y && y < CONNECT_Y + CONNECT_H) {
        beginConnect();
        return WifiAction::NONE;
    }

    switch (keyboard::handleTouch(x, y, g_password, 63)) {
    case keyboard::Result::EDITED:
        drawPasswordField();
        screen::flush();
        break;
    case keyboard::Result::SUBMIT:
        beginConnect();
        break;
    case keyboard::Result::NONE:
        break;
    }
    return WifiAction::NONE;
}
