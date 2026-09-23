#include <M5Unified.h>
#include <WiFi.h>
#include <esp32-hal-hosted.h>
#include <map>
#include <vector>

#include "adsb_client.h"
#include "config.h"
#include "detail_screen.h"
#include "display.h"
#include "location_screen.h"
#include "screen.h"
#include "secrets.h"
#include "settings.h"

namespace {

enum class Screen { MAIN, LOCATION, DETAIL };
Screen g_screen = Screen::MAIN;

SemaphoreHandle_t g_dataMutex;
std::vector<Aircraft> g_latestAircraft;
std::vector<uint8_t> g_latestIsNew;
bool g_dataReady = false;

double g_lat, g_lon;
String g_label;
bool g_locationChanged = false;  // set by UI touch handler, consumed by pollTask

// Only touched by pollTask - no locking needed since it's the sole writer/reader.
std::map<String, uint32_t> g_seenMap;
bool g_baseline = true;

int g_rotation = 3;
constexpr float ROTATION_THRESHOLD = 0.5f;

// Must match the geometry drawn in display.cpp.
constexpr int LOC_BTN_X = 600, LOC_BTN_Y = 6, LOC_BTN_W = 664, LOC_BTN_H = 52;
constexpr int TABLE_X = 8, TABLE_Y = 122, TABLE_W = 1264, ROW_HEIGHT = 52;

void pollTask(void *) {
    for (;;) {
        double lat, lon;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            if (g_locationChanged) {
                g_seenMap.clear();
                g_baseline = true;
                g_locationChanged = false;
            }
            lat = g_lat;
            lon = g_lon;
            xSemaphoreGive(g_dataMutex);
        }

        std::vector<Aircraft> aircraft;
        bool ok = fetchAircraft(lat, lon, DEFAULT_RADIUS_NM, aircraft);
        if (ok) {
            uint32_t now = millis();
            std::vector<uint8_t> isNew(aircraft.size(), 0);
            for (size_t i = 0; i < aircraft.size(); i++) {
                if (g_seenMap.find(aircraft[i].hex) == g_seenMap.end() && !g_baseline) {
                    isNew[i] = 1;
                }
                g_seenMap[aircraft[i].hex] = now;
            }
            g_baseline = false;

            for (auto it = g_seenMap.begin(); it != g_seenMap.end();) {
                if (now - it->second > FORGET_AFTER_MS) {
                    it = g_seenMap.erase(it);
                } else {
                    ++it;
                }
            }

            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                g_latestAircraft = aircraft;
                g_latestIsNew = isNew;
                g_dataReady = true;
                xSemaphoreGive(g_dataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

void checkRotation() {
    M5.Imu.update();
    float ax, ay, az;
    if (!M5.Imu.getAccel(&ax, &ay, &az)) {
        return;
    }
    if (ax > ROTATION_THRESHOLD && g_rotation != 3) {
        M5.Display.setRotation(3);  // keeps touch coordinates in step
        g_rotation = 3;
    } else if (ax < -ROTATION_THRESHOLD && g_rotation != 1) {
        M5.Display.setRotation(1);
        g_rotation = 1;
    } else {
        return;
    }
    // The canvas is drawn landscape either way; only the angle it's rotated
    // through on the way to the panel changes, so re-push what's already there.
    screen::setRotation(g_rotation);
    screen::flush();
}

void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
    }
    Serial.printf("[wifi] %s\n",
                  WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "FAILED to connect");
}

void handleMainTouch(int x, int y) {
    if (x >= LOC_BTN_X && x < LOC_BTN_X + LOC_BTN_W && y >= LOC_BTN_Y && y < LOC_BTN_Y + LOC_BTN_H) {
        locationScreenReset();
        g_screen = Screen::LOCATION;
        locationScreenDraw();
        return;
    }

    if (x >= TABLE_X && x < TABLE_X + TABLE_W && y >= TABLE_Y) {
        int row = (y - TABLE_Y) / ROW_HEIGHT;
        Aircraft tapped;
        bool found = false;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            if (row >= 0 && row < (int)g_latestAircraft.size()) {
                tapped = g_latestAircraft[row];
                found = true;
            }
            xSemaphoreGive(g_dataMutex);
        }
        if (found) {
            detailScreenSet(tapped);
            g_screen = Screen::DETAIL;
            detailScreenDraw();
        }
    }
}

void handleDetailTouch(int x, int y) {
    if (detailScreenHandleTouch(x, y)) {
        g_screen = Screen::MAIN;
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;  // force a redraw of the main screen with current data
            xSemaphoreGive(g_dataMutex);
        }
    }
}

void handleLocationTouch(int x, int y) {
    double lat, lon;
    String label;
    LocationAction action = locationScreenHandleTouch(x, y, lat, lon, label);
    if (action == LocationAction::BACK) {
        g_screen = Screen::MAIN;
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;  // force a redraw of the main screen with current data
            xSemaphoreGive(g_dataMutex);
        }
    } else if (action == LocationAction::LOCATION_SET) {
        saveSettings(lat, lon, label);
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_lat = lat;
            g_lon = lon;
            g_label = label;
            g_locationChanged = true;
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        g_screen = Screen::MAIN;
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);
    // Tab5's WiFi lives on a separate ESP32-C6 over SDIO, and the generic P4
    // eval-board pin defaults don't reach it. M5Unified is supposed to fix
    // this via a weak-linked hostedSetPins() call in M5.begin(), but set it
    // explicitly here too to be certain.
    hostedSetPins(12, 13, 11, 10, 9, 8, 15);

    auto cfg = M5.config();
    M5.begin(cfg);

    if (!screen::init()) {
        Serial.println("[setup] display canvas unavailable");
    }
    displayInit();

    LocationSettings s = loadSettings();
    g_lat = s.lat;
    g_lon = s.lon;
    g_label = s.label;

    std::vector<Aircraft> none;
    std::vector<uint8_t> noneNew;
    displayRenderAircraft(none, "Connecting...", noneNew);

    connectWifi();

    g_dataMutex = xSemaphoreCreateMutex();
    // Pinned to core 0 so it doesn't contend with the render/UI loop on core 1.
    // mbedTLS/HTTPS needs considerably more stack than a typical task.
    xTaskCreatePinnedToCore(pollTask, "poll", 16384, nullptr, 1, nullptr, 0);
}

void loop() {
    M5.update();

    if (M5.Touch.getCount()) {
        auto t = M5.Touch.getDetail(0);
        if (t.wasClicked()) {
            if (g_screen == Screen::MAIN) {
                handleMainTouch(t.x, t.y);
            } else if (g_screen == Screen::LOCATION) {
                handleLocationTouch(t.x, t.y);
            } else {
                handleDetailTouch(t.x, t.y);
            }
        }
    }

    static uint32_t nextRotationCheck = 0;
    uint32_t now = millis();
    if (now >= nextRotationCheck) {
        nextRotationCheck = now + 1000;
        checkRotation();
    }

    if (g_screen == Screen::MAIN) {
        std::vector<Aircraft> aircraft;
        std::vector<uint8_t> isNew;
        String label;
        bool shouldRender = false;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            label = g_label;
            if (g_dataReady) {
                aircraft = g_latestAircraft;
                isNew = g_latestIsNew;
                g_dataReady = false;
                shouldRender = true;
            }
            xSemaphoreGive(g_dataMutex);
        }
        if (shouldRender) {
            displayRenderAircraft(aircraft, label, isNew);
        }
    }

    delay(10);
}
