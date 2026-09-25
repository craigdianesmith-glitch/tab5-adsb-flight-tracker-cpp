#include <M5Unified.h>
#include <WiFi.h>
#include <esp32-hal-hosted.h>
#include <algorithm>
#include <map>
#include <vector>

#include "adsb_client.h"
#include "airports.h"
#include "config.h"
#include "detail_screen.h"
#include "display.h"
#include "location_screen.h"
#include "radar_screen.h"
#include "screen.h"
#include "secrets.h"
#include "settings.h"
#include "settings_screen.h"
#include "sound.h"
#include "wifi_screen.h"

namespace {

enum class Screen { MAIN, LOCATION, DETAIL, SETTINGS, WIFI, RADAR };
Screen g_screen = Screen::MAIN;
// The detail screen is reachable from the table and from the radar, and Back
// should land wherever you came from.
Screen g_detailReturnTo = Screen::MAIN;
// The WiFi screen is normally reached from settings, but a device with no
// credentials opens it straight from boot - where Back belongs on the table.
Screen g_wifiReturnTo = Screen::SETTINGS;

SemaphoreHandle_t g_dataMutex;
std::vector<Aircraft> g_latestAircraft;
std::vector<uint8_t> g_latestIsNew;
bool g_dataReady = false;
bool g_newFlightPending = false;  // set by pollTask, consumed by loop() to beep

double g_lat, g_lon;
String g_label;
TrafficFilter g_traffic = TrafficFilter::CIVIL;
int g_radiusNm = DEFAULT_RADIUS_NM;
bool g_showRefresh = true;
int g_pollIntervalS = DEFAULT_POLL_INTERVAL_S;
bool g_muted = false;
// The header shows the nearest airport rather than the place name now, and it
// only changes when the location does, so it's resolved there and kept.
String g_airportCode;
// Set by a UI touch handler, consumed by pollTask: forget which aircraft have
// been seen, so a changed location or filter doesn't flag everything as new.
bool g_resetBaseline = false;
bool g_pollNow = false;  // skip the rest of the poll interval and refetch

// Only touched by pollTask - no locking needed since it's the sole writer/reader.
std::map<String, uint32_t> g_seenMap;
bool g_baseline = true;

// Enough for the radar plot to look like the sky rather than a handful of
// dots, while still dropping the long tail of a busy radius.
constexpr size_t MAX_CONTACTS = 60;

int g_rotation = 3;
constexpr float ROTATION_THRESHOLD = 0.5f;

// What the status line under the table reports. Written by pollTask, read by
// loop(); the link flag is separate from the fetch flag because a dropped
// link is something the device is actively fixing and a failed fetch isn't.
bool g_linkUp = false;
bool g_pollOk = false;
bool g_everSucceeded = false;
uint32_t g_lastSuccessMs = 0;

// Credentials the poll task reconnects with, refreshed whenever the WiFi
// screen has been in and possibly changed them.
String g_wifiSsid, g_wifiPass;
// The WiFi screen drives the radio itself (scan, disconnect, begin) while it
// is up, so the poll task's reconnect stands down for the duration.
bool g_uiOwnsWifi = false;

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
constexpr uint32_t WIFI_RETRY_MIN_MS = 5000;
constexpr uint32_t WIFI_RETRY_MAX_MS = 120000;
// Backed off rather than retried flat out: a network that is genuinely gone
// shouldn't have the radio hammering at it for however long the device sits
// there, and a network that's merely rebooting is back inside the first step.
uint32_t g_wifiRetryMs = WIFI_RETRY_MIN_MS;
uint32_t g_nextWifiTryMs = 0;

// A build whose secrets.h was never filled in carries the example's
// placeholder, which is no more usable than an empty string. Either way there
// is nothing to connect with, and fifteen seconds spent failing to prove it
// is fifteen seconds of a blank table.
bool credentialsUnset(const String &ssid) { return ssid.length() == 0 || ssid == "your-ssid"; }

// Reconnects a link that has dropped since boot. Without this the tracker
// would sit there failing a fetch every interval until someone power-cycled
// it - an AP reboot or a few minutes out of range was enough to lose it for
// good. Runs on the poll task, so the blocking wait costs the UI nothing.
bool ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        g_wifiRetryMs = WIFI_RETRY_MIN_MS;
        return true;
    }

    String ssid, pass;
    bool uiBusy = false;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        uiBusy = g_uiOwnsWifi;
        ssid = g_wifiSsid;
        pass = g_wifiPass;
        xSemaphoreGive(g_dataMutex);
    }
    if (uiBusy || credentialsUnset(ssid)) {
        return false;
    }

    uint32_t now = millis();
    if (g_nextWifiTryMs != 0 && (int32_t)(now - g_nextWifiTryMs) < 0) {
        return false;
    }

    Serial.printf("[wifi] link down, reconnecting to %s\n", ssid.c_str());
    WiFi.disconnect();
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[wifi] reconnected: %s\n", WiFi.localIP().toString().c_str());
        g_wifiRetryMs = WIFI_RETRY_MIN_MS;
        g_nextWifiTryMs = 0;
        return true;
    }
    g_nextWifiTryMs = millis() + g_wifiRetryMs;
    g_wifiRetryMs = std::min(g_wifiRetryMs * 2, WIFI_RETRY_MAX_MS);
    Serial.printf("[wifi] reconnect failed, next try in %us\n", (unsigned)(g_wifiRetryMs / 1000));
    return false;
}

void pollTask(void *) {
    for (;;) {
        double lat, lon;
        TrafficFilter traffic = TrafficFilter::CIVIL;
        int radius = DEFAULT_RADIUS_NM;
        uint32_t intervalMs = (uint32_t)DEFAULT_POLL_INTERVAL_S * 1000;
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
            intervalMs = (uint32_t)g_pollIntervalS * 1000;
            xSemaphoreGive(g_dataMutex);
        }

        // The radius is already clamped to whichever ceiling the chosen filter
        // carries, so it doubles as the poll radius.
        bool wantMilitary = (traffic == TrafficFilter::MILITARY);

        bool linkUp = ensureWifi();
        std::vector<Aircraft> fetched;
        bool ok = linkUp && fetchAircraft(lat, lon, radius, fetched);
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

            // adsb_client sorts by distance, so this keeps the nearest and
            // drops the long tail: a 60nm radius can easily return well over a
            // hundred aircraft. The table draws only the rows that fit, but the
            // radar plots the lot, so the cap is the radar's rather than the
            // table's.
            size_t maxRows = (size_t)displayMaxRows();
            if (aircraft.size() > MAX_CONTACTS) {
                aircraft.resize(MAX_CONTACTS);
            }
            uint32_t now = millis();
            std::vector<uint8_t> isNew(aircraft.size(), 0);
            bool anyNew = false;
            for (size_t i = 0; i < aircraft.size(); i++) {
                if (g_seenMap.find(aircraft[i].hex) == g_seenMap.end() && !g_baseline) {
                    isNew[i] = 1;
                    // The beep still means a new *row*, so only contacts near
                    // enough to reach the table count towards it.
                    if (i < maxRows) {
                        anyNew = true;
                    }
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
                g_latestAircraft = std::move(aircraft);
                g_latestIsNew = std::move(isNew);
                g_dataReady = true;
                // One blip per poll however many arrived, and never on the
                // first poll after a start or a location change, where
                // everything is new by definition.
                g_newFlightPending = g_newFlightPending || anyNew;
                xSemaphoreGive(g_dataMutex);
            }
        }

        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_linkUp = linkUp;
            g_pollOk = ok;
            if (ok) {
                g_everSucceeded = true;
                g_lastSuccessMs = millis();
            }
            xSemaphoreGive(g_dataMutex);
        }

        // Sliced rather than one long delay, so a location or filter change
        // takes effect straight away instead of up to a poll interval later.
        // The interval is re-read each slice too, so dragging it shorter on the
        // settings screen shortens the wait already in progress.
        for (uint32_t waited = 0; waited < intervalMs; waited += 200) {
            vTaskDelay(pdMS_TO_TICKS(200));
            bool now = false;
            if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
                now = g_pollNow;
                intervalMs = (uint32_t)g_pollIntervalS * 1000;
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

void drawRadar() {
    std::vector<Aircraft> aircraft;
    std::vector<uint8_t> isNew;
    double lat = 0, lon = 0;
    int radius = DEFAULT_RADIUS_NM;
    bool military = false;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        aircraft = g_latestAircraft;
        isNew = g_latestIsNew;
        lat = g_lat;
        lon = g_lon;
        radius = g_radiusNm;
        military = (g_traffic == TrafficFilter::MILITARY);
        xSemaphoreGive(g_dataMutex);
    }
    radarScreenDraw(aircraft, isNew, lat, lon, radius, military);
}

void handleRadarTouch(int x, int y) {
    String hex;
    switch (radarScreenHandleTouch(x, y, hex)) {
    case RadarAction::BACK:
        g_screen = Screen::MAIN;
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        break;
    case RadarAction::SELECT: {
        Aircraft tapped;
        bool found = false;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            for (const Aircraft &a : g_latestAircraft) {
                if (a.hex == hex) {
                    tapped = a;
                    found = true;
                    break;
                }
            }
            xSemaphoreGive(g_dataMutex);
        }
        if (found) {
            detailScreenSet(tapped);
            g_detailReturnTo = Screen::RADAR;
            g_screen = Screen::DETAIL;
            detailScreenDraw();
        }
        break;
    }
    case RadarAction::NONE:
        break;
    }
}

void handleMainTouch(int x, int y) {
    if (displayHitRadar(x, y)) {
        g_screen = Screen::RADAR;
        drawRadar();
        return;
    }

    if (displayHitCog(x, y)) {
        String label;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            label = g_label;
            xSemaphoreGive(g_dataMutex);
        }
        settingsScreenSet(g_traffic, g_radiusNm, g_showRefresh, g_pollIntervalS, label);
        g_screen = Screen::SETTINGS;
        settingsScreenDraw();
        return;
    }

    if (displayHitMute(x, y)) {
        g_muted = !g_muted;
        soundSetMuted(g_muted);
        saveMuted(g_muted);
        displaySetMuted(g_muted);  // repaints just the icon
        if (!g_muted) {
            soundNewFlight();  // unmuting says so in the medium being unmuted
        }
        return;
    }

    int row;
    if (displayHitRow(x, y, row)) {
        Aircraft tapped;
        bool found = false;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            if (row < (int)g_latestAircraft.size()) {
                tapped = g_latestAircraft[row];
                found = true;
            }
            xSemaphoreGive(g_dataMutex);
        }
        if (found) {
            detailScreenSet(tapped);
            g_detailReturnTo = Screen::MAIN;
            g_screen = Screen::DETAIL;
            detailScreenDraw();
        }
    }
}

void handleDetailTouch(int x, int y) {
    if (!detailScreenHandleTouch(x, y)) {
        return;
    }
    if (g_detailReturnTo == Screen::RADAR) {
        g_screen = Screen::RADAR;
        drawRadar();
        return;
    }
    g_screen = Screen::MAIN;
    displayInvalidate();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_dataReady = true;  // force a redraw of the main screen with current data
        xSemaphoreGive(g_dataMutex);
    }
}

void handleLocationTouch(int x, int y) {
    double lat, lon;
    String label;
    LocationAction action = locationScreenHandleTouch(x, y, lat, lon, label);
    if (action == LocationAction::NONE) {
        return;
    }
    // Reached from the settings screen now, so both outcomes go back there
    // rather than to the table: setting a location then lands you where you
    // can see it took, on the button you pressed to get here.
    if (action == LocationAction::LOCATION_SET) {
        saveLocation(lat, lon, label);
        g_airportCode = nearestAirportCode(lat, lon);
        displaySetHeader(g_traffic == TrafficFilter::MILITARY, g_airportCode);
        settingsScreenSetLocation(label);
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_lat = lat;
            g_lon = lon;
            g_label = label;
            g_resetBaseline = true;
            g_pollNow = true;
            xSemaphoreGive(g_dataMutex);
        }
    }
    g_screen = Screen::SETTINGS;
    settingsScreenDraw();
}

void handleSettingsTouch(int x, int y, bool pressed, bool clicked) {
    switch (settingsScreenHandleTouch(x, y, pressed, clicked)) {
    case SettingsAction::BACK: {
        TrafficFilter traffic = settingsScreenTraffic();
        int radius = settingsScreenRadius();
        int interval = settingsScreenPollInterval();
        g_showRefresh = settingsScreenShowRefresh();
        saveFilters(traffic, radius, g_showRefresh, interval);
        displaySetShowRefresh(g_showRefresh);
        displaySetHeader(traffic == TrafficFilter::MILITARY, g_airportCode);
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            // A changed interval alone doesn't warrant refetching - the new
            // one simply applies to the wait already running.
            bool changed = (traffic != g_traffic) || (radius != g_radiusNm);
            g_traffic = traffic;
            g_radiusNm = radius;
            g_pollIntervalS = interval;
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
    case SettingsAction::OPEN_LOCATION:
        locationScreenReset();
        g_screen = Screen::LOCATION;
        locationScreenDraw();
        break;
    case SettingsAction::OPEN_WIFI:
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_uiOwnsWifi = true;  // hands the radio to the screen; see ensureWifi()
            xSemaphoreGive(g_dataMutex);
        }
        g_wifiReturnTo = Screen::SETTINGS;
        g_screen = Screen::WIFI;
        wifiScreenEnter();
        break;
    case SettingsAction::NONE:
        break;
    }
}

void handleWifiTouch(int x, int y) {
    if (wifiScreenHandleTouch(x, y) == WifiAction::BACK) {
        // Whatever the screen did, NVS now holds the credentials the poll task
        // should reconnect with - so take them back from there rather than
        // tracking every path through the screen that might have changed them.
        AppSettings s = loadSettings();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_uiOwnsWifi = false;
            g_wifiSsid = s.wifiSsid.length() ? s.wifiSsid : String(WIFI_SSID);
            g_wifiPass = s.wifiSsid.length() ? s.wifiPass : String(WIFI_PASSWORD);
            xSemaphoreGive(g_dataMutex);
        }
        g_nextWifiTryMs = 0;  // a fresh attempt is wanted now, not after the backoff
        g_wifiRetryMs = WIFI_RETRY_MIN_MS;
        if (g_wifiReturnTo == Screen::MAIN) {
            g_wifiReturnTo = Screen::SETTINGS;  // only the boot case lands on the table
            g_screen = Screen::MAIN;
            displayInvalidate();
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                g_dataReady = true;
                g_pollNow = true;  // credentials may have just arrived - don't wait out the interval
                xSemaphoreGive(g_dataMutex);
            }
            return;
        }
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
    g_showRefresh = s.showRefresh;
    g_pollIntervalS = s.pollIntervalS;
    g_muted = s.muted;
    soundSetMuted(g_muted);
    displaySetShowRefresh(g_showRefresh);
    displaySetMuted(g_muted);
    g_airportCode = nearestAirportCode(g_lat, g_lon);
    displaySetHeader(g_traffic == TrafficFilter::MILITARY, g_airportCode);

    // Something on screen before the WiFi connect blocks for up to 15s. The
    // status line under it says what's happening.
    std::vector<Aircraft> none;
    std::vector<uint8_t> noneNew;
    displayRenderAircraft(none, noneNew);
    displaySetPollState(false, false, false, 0);
    displayTickStatus();

    // Credentials set on-screen win; secrets.h is the fallback for a device
    // that's never had WiFi configured through the settings screen. Kept so
    // the poll task can reconnect with them without re-reading NVS each time.
    g_wifiSsid = s.wifiSsid.length() ? s.wifiSsid : String(WIFI_SSID);
    g_wifiPass = s.wifiSsid.length() ? s.wifiPass : String(WIFI_PASSWORD);

    bool haveCredentials = !credentialsUnset(g_wifiSsid);
    if (haveCredentials) {
        connectWifi(g_wifiSsid, g_wifiPass);
    } else {
        // Nothing to connect with, so don't spend the timeout finding out.
        // The screen scans, which needs the radio in station mode either way.
        Serial.println("[wifi] no credentials set - opening the WiFi screen");
        WiFi.mode(WIFI_STA);
    }
    g_linkUp = (WiFi.status() == WL_CONNECTED);
    // Set before the poll task exists, or its first pass could race the scan
    // the WiFi screen is about to start.
    g_uiOwnsWifi = !haveCredentials;

    g_dataMutex = xSemaphoreCreateMutex();
    // Pinned to core 0 so it doesn't contend with the render/UI loop on core 1.
    // mbedTLS/HTTPS needs considerably more stack than a typical task.
    xTaskCreatePinnedToCore(pollTask, "poll", 16384, nullptr, 1, nullptr, 0);

    soundBoot();

    // A device that has never been told about a network lands on the screen
    // that fixes that, rather than on a table that can never fill.
    if (!haveCredentials) {
        g_wifiReturnTo = Screen::MAIN;
        g_screen = Screen::WIFI;
        wifiScreenEnter();
    }
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
            case Screen::RADAR:
                handleRadarTouch(t.x, t.y);
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

    if (g_screen == Screen::RADAR) {
        bool fresh = false;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            fresh = g_dataReady;
            g_dataReady = false;
            xSemaphoreGive(g_dataMutex);
        }
        if (fresh) {
            drawRadar();
        }
    }

    if (g_screen == Screen::MAIN) {
        std::vector<Aircraft> aircraft;
        std::vector<uint8_t> isNew;
        bool shouldRender = false;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            if (g_dataReady) {
                aircraft = g_latestAircraft;
                isNew = g_latestIsNew;
                g_dataReady = false;
                shouldRender = true;
            }
            xSemaphoreGive(g_dataMutex);
        }
        if (shouldRender) {
            displayRenderAircraft(aircraft, isNew);
        }
        displayTickHighlights();

        bool linkUp = false, pollOk = false, everSucceeded = false;
        uint32_t lastSuccess = 0;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            linkUp = g_linkUp;
            pollOk = g_pollOk;
            everSucceeded = g_everSucceeded;
            lastSuccess = g_lastSuccessMs;
            xSemaphoreGive(g_dataMutex);
            displaySetPollState(linkUp, pollOk, everSucceeded, lastSuccess);
        }
        displayTickStatus();
    }

    delay(10);
}
