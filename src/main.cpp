#include <M5Unified.h>
#include <WiFi.h>
#include <esp32-hal-hosted.h>
#include <algorithm>
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
#include "settings_screen.h"
#include "sound.h"
#include "wifi_screen.h"

namespace {

enum class Screen { MAIN, LOCATION, DETAIL, SETTINGS, WIFI };
Screen g_screen = Screen::MAIN;

SemaphoreHandle_t g_dataMutex;
std::vector<Aircraft> g_latestAircraft;
std::vector<uint8_t> g_latestIsNew;
bool g_dataReady = false;
bool g_newFlightPending = false;  // set by pollTask, consumed by loop() to beep

double g_lat, g_lon;
String g_label;
TrafficFilter g_traffic = TrafficFilter::CIVIL;
int g_radiusNm = DEFAULT_RADIUS_NM;
// Set by a UI touch handler, consumed by pollTask: forget which aircraft have
// been seen, so a changed location or filter doesn't flag everything as new.
bool g_resetBaseline = false;
bool g_pollNow = false;  // skip the rest of the poll interval and refetch

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
        TrafficFilter traffic = TrafficFilter::CIVIL;
        int radius = DEFAULT_RADIUS_NM;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            if (g_resetBaseline) {
                g_seenMap.clear();
                g_baseline = true;
                g_resetBaseline = false;
            }
            g_pollNow = false;
            lat = g_lat;
            lon = g_lon;
            traffic = g_traffic;
            radius = g_radiusNm;
            xSemaphoreGive(g_dataMutex);
        }

        // The radius is already clamped to whichever ceiling the chosen filter
        // carries, so it doubles as the poll radius.
        bool wantMilitary = (traffic == TrafficFilter::MILITARY);

        std::vector<Aircraft> fetched;
        bool ok = fetchAircraft(lat, lon, radius, fetched);
        if (ok) {
            std::vector<Aircraft> aircraft;
            for (const Aircraft &a : fetched) {
                if (a.military != wantMilitary) {
                    continue;
                }
                if (a.hasDist && a.distNm > radius) {
                    continue;  // the API is occasionally a little generous
                }
                aircraft.push_back(a);
            }

            // adsb_client sorts by distance, so this keeps the nearest few and
            // drops the rest: a 60nm radius can easily return a hundred
            // aircraft where only eleven rows fit. Trimming here rather than at
            // draw time also means "new arrival" means a new *row*, instead of
            // beeping at every aircraft entering the radius unseen.
            size_t maxRows = (size_t)displayMaxRows();
            if (aircraft.size() > maxRows) {
                aircraft.resize(maxRows);
            }
            uint32_t now = millis();
            std::vector<uint8_t> isNew(aircraft.size(), 0);
            bool anyNew = false;
            for (size_t i = 0; i < aircraft.size(); i++) {
                if (g_seenMap.find(aircraft[i].hex) == g_seenMap.end() && !g_baseline) {
                    isNew[i] = 1;
                    anyNew = true;
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
                // One blip per poll however many arrived, and never on the
                // first poll after a start or a location change, where
                // everything is new by definition.
                g_newFlightPending = g_newFlightPending || anyNew;
                xSemaphoreGive(g_dataMutex);
            }
        }
        // Sliced rather than one long delay, so a location or filter change
        // takes effect straight away instead of up to a poll interval later.
        for (uint32_t waited = 0; waited < POLL_INTERVAL_MS; waited += 200) {
            vTaskDelay(pdMS_TO_TICKS(200));
            bool now = false;
            if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
                now = g_pollNow;
                xSemaphoreGive(g_dataMutex);
            }
            if (now) {
                break;
            }
        }
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

void connectWifi(const String &ssid, const String &password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
    }
    Serial.printf("[wifi] %s\n",
                  WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "FAILED to connect");
}

void handleMainTouch(int x, int y) {
    if (displayHitCog(x, y)) {
        settingsScreenSet(g_traffic, g_radiusNm);
        g_screen = Screen::SETTINGS;
        settingsScreenDraw();
        return;
    }

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
        saveLocation(lat, lon, label);
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_lat = lat;
            g_lon = lon;
            g_label = label;
            g_resetBaseline = true;
            g_pollNow = true;
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        g_screen = Screen::MAIN;
    }
}

void handleSettingsTouch(int x, int y, bool pressed, bool clicked) {
    switch (settingsScreenHandleTouch(x, y, pressed, clicked)) {
    case SettingsAction::BACK: {
        TrafficFilter traffic = settingsScreenTraffic();
        int radius = settingsScreenRadius();
        saveFilters(traffic, radius);
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            bool changed = (traffic != g_traffic) || (radius != g_radiusNm);
            g_traffic = traffic;
            g_radiusNm = radius;
            if (changed) {
                g_resetBaseline = true;
                g_pollNow = true;
            }
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        g_screen = Screen::MAIN;
        displayInvalidate();
        break;
    }
    case SettingsAction::OPEN_WIFI:
        g_screen = Screen::WIFI;
        wifiScreenEnter();
        break;
    case SettingsAction::NONE:
        break;
    }
}

void handleWifiTouch(int x, int y) {
    if (wifiScreenHandleTouch(x, y) == WifiAction::BACK) {
        g_screen = Screen::SETTINGS;
        settingsScreenDraw();
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

    soundInit();

    if (!screen::init()) {
        Serial.println("[setup] display canvas unavailable");
    }
    displayInit();

    AppSettings s = loadSettings();
    g_lat = s.lat;
    g_lon = s.lon;
    g_label = s.label;
    g_traffic = s.traffic;
    g_radiusNm = s.radiusNm;

    std::vector<Aircraft> none;
    std::vector<uint8_t> noneNew;
    displayRenderAircraft(none, "Connecting...", noneNew);

    // Credentials set on-screen win; secrets.h is the fallback for a device
    // that's never had WiFi configured through the settings screen.
    connectWifi(s.wifiSsid.length() ? s.wifiSsid : String(WIFI_SSID),
                s.wifiSsid.length() ? s.wifiPass : String(WIFI_PASSWORD));

    g_dataMutex = xSemaphoreCreateMutex();
    // Pinned to core 0 so it doesn't contend with the render/UI loop on core 1.
    // mbedTLS/HTTPS needs considerably more stack than a typical task.
    xTaskCreatePinnedToCore(pollTask, "poll", 16384, nullptr, 1, nullptr, 0);

    soundBoot();
}

void loop() {
    M5.update();

    if (g_screen == Screen::SETTINGS) {
        // Dispatched every pass, touch or not: the slider has to hear about
        // the finger lifting, and a release arrives as an absence of touch
        // rather than as an event of its own.
        bool touching = M5.Touch.getCount() > 0;
        int tx = -1, ty = -1;
        bool pressed = false, clicked = false;
        if (touching) {
            auto t = M5.Touch.getDetail(0);
            tx = t.x;
            ty = t.y;
            pressed = t.isPressed();
            clicked = t.wasClicked();
        }
        handleSettingsTouch(tx, ty, pressed, clicked);
    } else if (M5.Touch.getCount()) {
        auto t = M5.Touch.getDetail(0);
        if (t.wasClicked()) {
            switch (g_screen) {
            case Screen::MAIN:
                handleMainTouch(t.x, t.y);
                break;
            case Screen::LOCATION:
                handleLocationTouch(t.x, t.y);
                break;
            case Screen::DETAIL:
                handleDetailTouch(t.x, t.y);
                break;
            case Screen::WIFI:
                handleWifiTouch(t.x, t.y);
                break;
            default:
                break;
            }
        }
    }

    if (g_screen == Screen::WIFI) {
        wifiScreenTick();  // scans and connection attempts both complete asynchronously
    }

    static uint32_t nextRotationCheck = 0;
    uint32_t now = millis();
    if (now >= nextRotationCheck) {
        nextRotationCheck = now + 1000;
        checkRotation();
    }

    // Deliberately outside the MAIN branch: an arrival is worth hearing even
    // while the location or detail screen is up.
    bool newFlight = false;
    if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
        newFlight = g_newFlightPending;
        g_newFlightPending = false;
        xSemaphoreGive(g_dataMutex);
    }
    if (newFlight) {
        soundNewFlight();
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
