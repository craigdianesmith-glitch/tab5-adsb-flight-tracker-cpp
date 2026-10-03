#include <M5Unified.h>
#include <WiFi.h>
#include <esp32-hal-hosted.h>
#include <algorithm>
#include <map>
#include <time.h>
#include <vector>

#include "adsb_client.h"
#include "aircraft_db.h"
#include "airports.h"
#include "alerts.h"
#include "alerts_screen.h"
#include "config.h"
#include "detail_screen.h"
#include "display.h"
#include "location_screen.h"
#include "playback_screen.h"
#include "radar_screen.h"
#include "recorder.h"
#include "recordings_screen.h"
#include "screen.h"
#include "secrets.h"
#include "settings.h"
#include "settings_screen.h"
#include "share_screen.h"
#include "sound.h"
#include "watchlist_screen.h"
#include "wifi_screen.h"

namespace {

enum class Screen { MAIN, LOCATION, DETAIL, SETTINGS, WIFI, RADAR, ZOOM, ALERTS, WATCHLIST, RECORDINGS, PLAYBACK, SHARE };
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
AdsbSource g_source = DEFAULT_ADSB_SOURCE;
RadarCentre g_radarCentre = DEFAULT_RADAR_CENTRE;  // only touched by loop()
bool g_radarAirports = false;                       // likewise
// Which provider actually answered the last successful poll, for the header.
// Under AUTO that is not necessarily the one at the top of the table.
String g_activeProvider;
// The header shows the nearest airport rather than the place name now, and it
// only changes when the location does, so it's resolved there and kept.
String g_airportCode;
// Set by a UI touch handler, consumed by pollTask: forget which aircraft have
// been seen, so a changed location or filter doesn't flag everything as new.
bool g_resetBaseline = false;
bool g_pollNow = false;  // skip the rest of the poll interval and refetch
// Set with the location, range or traffic changing. A recording's header holds
// the home and range its plot is drawn around, so one that ran on across the
// change would replay the new sky against the old centre - off the plot, or
// clipped at the old range. The poll task stops it instead.
bool g_sceneChanged = false;

// The airport zoom. While it is up - or a detail opened from it - the poll
// task also fetches the traffic around its airport, in between the main
// polls, into a list of its own: the table, alerts and recordings carry on
// with the main poll's as before.
struct ZoomPoll {
    bool active = false;
    uint32_t gen = 0;  // which zoom, so a fetch for one since closed is dropped
    double lat = 0, lon = 0;
    int radiusNm = 0;
};
ZoomPoll g_zoomPoll;                   // under the mutex
std::vector<Aircraft> g_zoomAircraft;  // likewise
bool g_zoomReady = false;              // likewise: a new list, for loop() to draw
bool g_zoomUp = false;                 // only touched by loop(): g_zoomPoll.active, without the lock
uint32_t g_zoomTouchedMs = 0;          // likewise: the last touch, for closing the zoom left alone

// Only touched by pollTask - no locking needed since it's the sole writer/reader.
std::map<String, uint32_t> g_seenMap;
bool g_baseline = true;

// --- alerts and recording ---------------------------------------------------

// The rules pollTask checks each aircraft against, and whether an alert
// starts a recording. Set from the alerts screen, so read under the mutex.
AlertRules g_alertRules;
bool g_autoRecord = false;
// The same, as the alerts screen edits them. Only touched by loop().
uint8_t g_alertMask = ALERT_ALL;
String g_watchlist;

// Set by pollTask, consumed by loop(): the most important of the alerts the
// last poll raised, and how many more came with it.
bool g_alertPending = false;
Aircraft g_pendingAlert;
int g_pendingAlertMore = 0;
// Every aircraft the alert is for, by ICAO hex, for the callsign flash.
std::vector<String> g_pendingFlash;

// Set when a recording that auto-record started is stopped by hand. Without
// it, the next poll would find the same aircraft still about and start
// another; pollTask clears it once no alerted aircraft are left.
bool g_autoRecordHeld = false;

// Only touched by pollTask. Which reasons each aircraft has already been
// alerted for, so a contact alerts once per sighting rather than every poll -
// but again if it trips a new rule, say by squawking 7700.
struct AlertSeen {
    uint8_t reasons;
    uint32_t lastMs;
};
std::map<String, AlertSeen> g_alertSeen;
uint32_t g_lastAlertedMs = 0;

// The banner, owned by loop(). Shown over the screens where the sky is on
// show; put off on the ones being typed into or worked through, and shown
// on the next of the others that comes up.
constexpr int BANNER_H = 60;
constexpr int BANNER_CLOSE_W = 84;
bool g_bannerUp = false;
Aircraft g_bannerAc;
int g_bannerMore = 0;
uint32_t g_bannerShownMs = 0;  // 0 until it has actually been on screen

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

// Consecutive failed attempts, the boot connect included. Past the limit the
// network is taken to be gone rather than blipping - a hotspot that's been
// switched off, or a device carried somewhere else - and the WiFi screen is
// put up so another can be picked. Retries carry on meanwhile, so a network
// that comes back before anyone looks still wins.
constexpr int WIFI_MAX_FAILS = 3;
int g_wifiFails = 0;
// Set by the poll task once the limit is reached; loop() acts on it.
bool g_wifiGaveUp = false;

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
        // Cleared here too, not only after a reconnect of our own: a link the
        // driver brought back by itself left a stale time behind, and once
        // millis() had moved more than 24.8 days past it, the signed test
        // below read it as the future and held off for weeks.
        g_nextWifiTryMs = 0;
        if (g_wifiFails != 0) {
            g_wifiFails = 0;  // came back by itself between attempts
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                g_wifiGaveUp = false;
                xSemaphoreGive(g_dataMutex);
            }
        }
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
        g_wifiFails = 0;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_wifiGaveUp = false;
            xSemaphoreGive(g_dataMutex);
        }
        return true;
    }
    uint32_t wait = g_wifiRetryMs;
    g_nextWifiTryMs = millis() + wait;
    g_wifiRetryMs = std::min(g_wifiRetryMs * 2, WIFI_RETRY_MAX_MS);
    g_wifiFails++;
    Serial.printf("[wifi] reconnect failed (%d of %d), next try in %us\n", g_wifiFails, WIFI_MAX_FAILS,
                  (unsigned)(wait / 1000));
    if (g_wifiFails >= WIFI_MAX_FAILS) {
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_wifiGaveUp = true;
            xSemaphoreGive(g_dataMutex);
        }
    }
    return false;
}

// One fetch of the traffic around the zoom's airport, from the poll task.
// Everything flying is kept, whichever traffic the table is filtered to - at
// an airport the airliners are the point, even in military mode - and alerts
// are evaluated only for the colours they draw in: the main poll is what
// raises them.
void pollZoom(const ZoomPoll &zoom, AdsbSource source) {
    std::vector<Aircraft> fetched;
    const char *provider = nullptr;
    if (WiFi.status() != WL_CONNECTED ||
        !fetchAircraft(zoom.lat, zoom.lon, zoom.radiusNm, false, source, fetched, provider)) {
        return;  // the last list stays up, as the main poll's does
    }
    AlertRules rules;
    double homeLat = 0, homeLon = 0;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        rules = g_alertRules;
        homeLat = g_lat;
        homeLon = g_lon;
        xSemaphoreGive(g_dataMutex);
    }
    if (fetched.size() > MAX_CONTACTS) {
        fetched.erase(fetched.begin() + MAX_CONTACTS, fetched.end());  // nearest the airport first
    }
    for (Aircraft &a : fetched) {
        a.alert = evaluateAlert(a, rules, true);
        // From home, as a distance is everywhere else it shows - the detail
        // screen, opened from the zoom - where the fetch measured it from
        // the airport.
        if (a.hasPos) {
            a.distNm = haversineNm(homeLat, homeLon, a.lat, a.lon);
        }
    }
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        if (g_zoomPoll.active && g_zoomPoll.gen == zoom.gen) {
            g_zoomAircraft = std::move(fetched);
            g_zoomReady = true;
        }
        xSemaphoreGive(g_dataMutex);
    }
}

void pollTask(void *) {
    // When the last request of either kind went, and the last of the zoom's,
    // for spacing them - see the wait at the bottom.
    uint32_t lastRequestMs = 0, lastZoomMs = 0;
    uint32_t zoomGenServed = 0;
    for (;;) {
        double lat, lon;
        TrafficFilter traffic = TrafficFilter::CIVIL;
        int radius = DEFAULT_RADIUS_NM;
        AdsbSource source = DEFAULT_ADSB_SOURCE;
        uint32_t intervalMs = (uint32_t)DEFAULT_POLL_INTERVAL_S * 1000;
        AlertRules rules;
        bool autoRecord = false;
        bool sceneChanged = false;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            sceneChanged = g_sceneChanged;
            g_sceneChanged = false;
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
            source = g_source;
            intervalMs = (uint32_t)g_pollIntervalS * 1000;
            rules = g_alertRules;
            autoRecord = g_autoRecord;
            xSemaphoreGive(g_dataMutex);
        }

        if (sceneChanged && recorder::active()) {
            Serial.println("[rec] location, range or traffic changed - stopping the recording");
            recorder::stop();  // auto-record starts afresh, with the new header, if an alert is still about
        }

        // The radius is already clamped to whichever ceiling the chosen filter
        // carries, so it doubles as the poll radius.
        bool wantMilitary = (traffic == TrafficFilter::MILITARY);
        // A civil poll from a feed that marks military traffic carries it
        // anyway, and it is only filtered out here - so with that alert on, it
        // is kept, to be flagged. (A feed that doesn't mark it can't say which
        // of its contacts are military, so there is nothing to keep.)
        bool keepMilitary = !wantMilitary && (rules.enabled & ALERT_MILITARY);

        bool linkUp = ensureWifi();
        std::vector<Aircraft> fetched;
        const char *provider = nullptr;
        bool ok = linkUp && fetchAircraft(lat, lon, radius, wantMilitary, source, fetched, provider);
        if (ok) {
            std::vector<Aircraft> aircraft;
            aircraft.reserve(fetched.size());
            for (Aircraft &a : fetched) {
                if (a.military != wantMilitary && !(a.military && keepMilitary)) {
                    continue;
                }
                if (a.hasDist && a.distNm > radius) {
                    continue;  // the API is occasionally a little generous
                }
                a.alert = evaluateAlert(a, rules, !wantMilitary);
                aircraft.push_back(std::move(a));
            }

            // adsb_client sorts by distance, so this keeps the nearest and
            // drops the long tail: a 60nm radius can easily return well over a
            // hundred aircraft. The table draws only the rows that fit, but the
            // radar plots the lot, so the cap is the radar's rather than the
            // table's.
            //
            // Except that an alerted aircraft is never the tail: one that has
            // tripped an alert at the edge of a busy radius is kept, after the
            // rest, in distance order among themselves.
            size_t maxRows = (size_t)displayMaxRows();
            if (aircraft.size() > MAX_CONTACTS) {
                auto keepEnd = std::stable_partition(aircraft.begin() + MAX_CONTACTS, aircraft.end(),
                                                     [](const Aircraft &a) { return a.alert != 0; });
                aircraft.erase(keepEnd, aircraft.end());
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

            // Which alerts are news. Unlike the new-row blip there is no
            // baseline to sit out: an emergency in the sky at boot is as worth
            // hearing about as one that turns up later.
            const Aircraft *best = nullptr;
            const Aircraft *firstAlerted = nullptr;
            int fresh = 0;
            std::vector<String> freshHexes;
            for (const Aircraft &a : aircraft) {
                if (!a.alert) {
                    continue;
                }
                if (firstAlerted == nullptr) {
                    firstAlerted = &a;
                }
                AlertSeen &seen = g_alertSeen[a.hex];
                uint8_t newReasons = a.alert & ~seen.reasons;
                seen.reasons |= a.alert;
                seen.lastMs = now;
                if (newReasons) {
                    fresh++;
                    freshHexes.push_back(a.hex);
                    if (best == nullptr || alertOutranks(a, *best)) {
                        best = &a;
                    }
                }
            }
            for (auto it = g_alertSeen.begin(); it != g_alertSeen.end();) {
                if (now - it->second.lastMs > FORGET_AFTER_MS) {
                    it = g_alertSeen.erase(it);
                } else {
                    ++it;
                }
            }

            // Auto-record: started by any alerted aircraft being about, not
            // only a fresh alert, so switching it on with one already in view
            // starts it too; stopped once none has been seen for the tail.
            bool held = false;
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                if (firstAlerted == nullptr) {
                    g_autoRecordHeld = false;
                }
                held = g_autoRecordHeld;
                xSemaphoreGive(g_dataMutex);
            }
            if (firstAlerted != nullptr) {
                g_lastAlertedMs = now;
            }
            if (autoRecord && firstAlerted != nullptr && !held && !recorder::active()) {
                const Aircraft &why = best ? *best : *firstAlerted;
                recorder::Header h;
                time_t wall = time(nullptr);
                h.startEpoch = wall > 1700000000 ? (uint32_t)wall : 0;
                h.trigger = recorder::Trigger::AUTO;
                h.note = why.callsign + " " + alertReasonText(why, why.alert);
                h.lat = lat;
                h.lon = lon;
                h.radiusNm = radius;
                h.military = wantMilitary;
                recorder::start(h);
            } else if (recorder::active() && recorder::activeTrigger() == recorder::Trigger::AUTO &&
                       now - g_lastAlertedMs > AUTO_RECORD_TAIL_MS) {
                recorder::stop();
            }
            recorder::addFrame(aircraft);

            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                if (best != nullptr) {
                    g_pendingAlert = *best;
                    g_pendingAlertMore = fresh - 1;
                    g_pendingFlash = std::move(freshHexes);
                    g_alertPending = true;
                }
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
                if (provider != nullptr) {
                    g_activeProvider = provider;
                }
            }
            xSemaphoreGive(g_dataMutex);
        }

        // Sliced rather than one long delay, so a location or filter change
        // takes effect straight away instead of up to a poll interval later.
        // The interval is re-read each slice too, so dragging it shorter on the
        // settings screen shortens the wait already in progress.
        //
        // While the airport zoom is up, its fetches go in the gaps. No two
        // requests go closer together than the shortest poll interval, which
        // is what the providers will take, and when both are due the one
        // that has waited longer goes first - so neither starves the other,
        // whatever the main interval is set to. Without a zoom, this is the
        // plain interval it always was.
        constexpr uint32_t SPACING_MS = (uint32_t)POLL_INTERVAL_MIN_S * 1000;
        constexpr uint32_t ZOOM_POLL_MS = (uint32_t)ZOOM_POLL_INTERVAL_S * 1000;
        lastRequestMs = millis();
        uint32_t waitStart = lastRequestMs;
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(200));
            bool now = false;
            ZoomPoll zoom;
            if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
                now = g_pollNow;
                intervalMs = (uint32_t)g_pollIntervalS * 1000;
                zoom = g_zoomPoll;
                xSemaphoreGive(g_dataMutex);
            }
            if (now) {
                break;
            }
            uint32_t t = millis();
            if (t - lastRequestMs < SPACING_MS) {
                continue;
            }
            int32_t mainLate = (int32_t)(t - waitStart - intervalMs);
            int32_t zoomLate = INT32_MIN;
            if (zoom.active) {
                // A zoom just opened has waited longest of all.
                zoomLate = zoom.gen != zoomGenServed ? INT32_MAX : (int32_t)(t - lastZoomMs - ZOOM_POLL_MS);
            }
            if (mainLate >= 0 && mainLate >= zoomLate) {
                break;
            }
            if (zoomLate >= 0) {
                pollZoom(zoom, source);
                zoomGenServed = zoom.gen;
                lastZoomMs = lastRequestMs = millis();
            }
        }
    }
}

// g_activeProvider is written by pollTask on the other core, so the UI copies
// it under the mutex rather than reading a String mid-reassignment. Not usable
// before the mutex exists - setup() reads the variable directly, where it is
// still empty and nothing else is running yet.
String activeProvider() {
    String p;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        p = g_activeProvider;
        xSemaphoreGive(g_dataMutex);
    }
    return p;
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

void updateRecButton(bool onScreen);

void drawRadar(bool full) {
    if (full) {
        updateRecButton(false);  // so the header is drawn showing the state as it is now
    }
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
    radarScreenDraw(aircraft, isNew, lat, lon, radius, military, g_radarCentre, g_radarAirports, full);
}

void drawZoom(bool full) {
    std::vector<Aircraft> aircraft;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        aircraft = g_zoomAircraft;
        xSemaphoreGive(g_dataMutex);
    }
    radarZoomDraw(aircraft, full);
}

// Zooms in on an airport tapped on the radar. Until the zoom's own first
// fetch lands, the radar's contacts stand in: around an airport inside the
// radius they are the same aircraft, a poll older.
bool openZoom(const String &code) {
    double lat, lon;
    float rangeNm;
    if (!radarZoomOpen(code, lat, lon, rangeNm)) {
        return false;
    }
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.active = true;
        g_zoomPoll.gen++;
        g_zoomPoll.lat = lat;
        g_zoomPoll.lon = lon;
        g_zoomPoll.radiusNm = (int)ceilf(rangeNm) + ZOOM_FETCH_EXTRA_NM;
        g_zoomAircraft = g_latestAircraft;
        g_zoomReady = false;
        xSemaphoreGive(g_dataMutex);
    }
    g_zoomUp = true;
    g_zoomTouchedMs = millis();
    g_screen = Screen::ZOOM;
    drawZoom(true);
    return true;
}

// Stops the zoom's fetches. Called whichever way it was left - see loop().
void endZoom() {
    if (!g_zoomUp) {
        return;
    }
    g_zoomUp = false;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.active = false;
        g_zoomAircraft.clear();
        g_zoomReady = false;
        xSemaphoreGive(g_dataMutex);
    }
}

void closeZoom() {
    endZoom();
    g_screen = Screen::RADAR;
    drawRadar(true);
}

// Where the banner may be shown: the screens the sky is on. The others are
// either being typed into or are a replay, where a live alert drawn over the
// top would read as part of what was recorded.
bool bannerScreen(Screen s) {
    return s == Screen::MAIN || s == Screen::RADAR || s == Screen::ZOOM || s == Screen::DETAIL;
}

// The screen overlay: drawn by screen::flush() over whatever is up.
bool drawBanner(M5Canvas &canvas) {
    if (!g_bannerUp || !bannerScreen(g_screen)) {
        return false;
    }
    if (g_bannerShownMs == 0) {
        g_bannerShownMs = millis() | 1;  // never 0, which means "not yet shown"
    } else if (millis() - g_bannerShownMs > ALERT_BANNER_MS) {
        // Ran out while another screen was up. Coming back, it is about to be
        // taken down - so it isn't put up first, only to flash off again.
        return false;
    }
    const Aircraft &ac = g_bannerAc;
    bool emergency = (ac.alert & ALERT_EMERGENCY) != 0;
    uint16_t bg = emergency ? M5.Display.color565(0xC0, 0x39, 0x2B) : M5.Display.color565(0xB9, 0x6A, 0x0A);
    uint16_t fg = M5.Display.color565(0xFF, 0xFF, 0xFF);
    int midY = BANNER_H / 2;
    canvas.fillRect(0, 0, 1280, BANNER_H, bg);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(ML_DATUM);
    canvas.setTextColor(fg);

    canvas.setTextSize(3);
    String reason = alertReasonText(ac, ac.alert);
    canvas.drawString(reason, 16, midY);
    int x = 16 + canvas.textWidth(reason) + 28;
    canvas.drawString(ac.callsign, x, midY);
    x += canvas.textWidth(ac.callsign) + 24;

    String rest = (ac.type.length() && ac.type != "----") ? lookupAircraftType(ac.type, ac.military) : String("");
    if (ac.hasDist) {
        rest += (rest.length() ? "   " : "") + String((int)lroundf(ac.distNm)) + " nm";
    }
    if (g_bannerMore > 0) {
        rest += "   +" + String(g_bannerMore) + " more";
    }
    rest += "   - tap for details";
    int room = 1280 - BANNER_CLOSE_W - 16 - x;
    int size = 2;
    canvas.setTextSize(size);
    while (size > 1 && canvas.textWidth(rest) > room) {
        canvas.setTextSize(--size);
    }
    canvas.drawString(rest, x, midY);

    canvas.drawFastVLine(1280 - BANNER_CLOSE_W, 10, BANNER_H - 20, fg);
    canvas.setTextSize(3);
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("X", 1280 - BANNER_CLOSE_W / 2, midY);
    return true;
}

// Paints whichever banner screen is up from scratch, to take the banner off it.
void repaintCurrentScreen() {
    switch (g_screen) {
    case Screen::MAIN:
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        break;
    case Screen::RADAR:
        drawRadar(true);
        break;
    case Screen::ZOOM:
        drawZoom(true);
        break;
    case Screen::DETAIL:
        detailScreenDraw();
        break;
    default:
        break;
    }
}

void dismissBanner() {
    if (!g_bannerUp) {
        return;
    }
    bool wasShowing = g_bannerShownMs != 0 && bannerScreen(g_screen);
    g_bannerUp = false;
    g_bannerShownMs = 0;
    if (wasShowing) {
        repaintCurrentScreen();
    }
}

// Picks up an alert the poll task has raised, and times the banner out.
void tickAlerts() {
    bool fresh = false;
    std::vector<String> flash;
    if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
        if (g_alertPending) {
            g_bannerAc = g_pendingAlert;
            g_bannerMore = g_pendingAlertMore;
            flash = std::move(g_pendingFlash);
            g_pendingFlash.clear();
            g_alertPending = false;
            fresh = true;
        }
        xSemaphoreGive(g_dataMutex);
    }
    if (fresh) {
        alertFlashStart(flash);  // in step with the chime
    }
    if (fresh) {
        Serial.printf("[alert] %s %s (+%d more)\n", g_bannerAc.callsign.c_str(),
                      alertReasonText(g_bannerAc, g_bannerAc.alert).c_str(), g_bannerMore);
        g_bannerUp = true;
        g_bannerShownMs = 0;
        soundAlert((g_bannerAc.alert & ALERT_EMERGENCY) != 0);
    }
    if (!g_bannerUp || !bannerScreen(g_screen)) {
        return;
    }
    if (g_bannerShownMs == 0 || fresh) {
        // Not on screen yet, or replaced by a newer alert: marking its strip
        // dirty is what has the flush draw it.
        g_bannerShownMs = 0;
        screen::markDirty(0, 0, 1280, BANNER_H);
        screen::flush();
    } else if (millis() - g_bannerShownMs > ALERT_BANNER_MS) {
        dismissBanner();
    }
}

void openDetail(const Aircraft &ac, Screen returnTo) {
    detailScreenSet(ac);
    g_detailReturnTo = returnTo;
    g_screen = Screen::DETAIL;
    detailScreenDraw();
}

// Whether a touch at height y lands on the banner, if it is showing.
bool bannerOver(int y) { return g_bannerUp && g_bannerShownMs != 0 && bannerScreen(g_screen) && y < BANNER_H; }

// A tap on the banner: the X closes it, anywhere else opens the aircraft -
// as it is now if it's still in range, as it was when it alerted if not.
bool handleBannerTouch(int x, int y) {
    if (!bannerOver(y)) {
        return false;
    }
    if (x >= 1280 - BANNER_CLOSE_W) {
        dismissBanner();
        return true;
    }
    Aircraft ac = g_bannerAc;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        for (const Aircraft &a : g_latestAircraft) {
            if (a.hex == ac.hex) {
                ac = a;
                break;
            }
        }
        xSemaphoreGive(g_dataMutex);
    }
    g_bannerUp = false;
    g_bannerShownMs = 0;
    // From the detail screen, Back still goes wherever it was going to.
    openDetail(ac, g_screen == Screen::DETAIL ? g_detailReturnTo : g_screen);
    return true;
}

void updateRecButton(bool onScreen) {
    bool active = recorder::active();
    RecButton state = active                ? RecButton::RECORDING
                      : !recorder::mounted() ? RecButton::NO_CARD
                                             : RecButton::IDLE;
    uint32_t elapsed = active ? (millis() - recorder::activeSinceMs()) / 1000 : 0;
    radarScreenSetRecording(state, elapsed, onScreen);
}

void toggleRecording() {
    if (recorder::active()) {
        if (recorder::activeTrigger() == recorder::Trigger::AUTO) {
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                g_autoRecordHeld = true;
                xSemaphoreGive(g_dataMutex);
            }
        }
        recorder::stop();
    } else {
        recorder::Header h;
        time_t wall = time(nullptr);
        h.startEpoch = wall > 1700000000 ? (uint32_t)wall : 0;
        h.trigger = recorder::Trigger::MANUAL;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            h.lat = g_lat;
            h.lon = g_lon;
            h.radiusNm = g_radiusNm;
            h.military = (g_traffic == TrafficFilter::MILITARY);
            xSemaphoreGive(g_dataMutex);
        }
        recorder::start(h);  // which mounts a card put in since; NO SD stays up if there still isn't one
    }
    updateRecButton(true);
}

String alertSummary() {
    int on = 0;
    for (uint8_t bit : {ALERT_EMERGENCY, ALERT_MILITARY, ALERT_WATCHLIST, ALERT_RARE}) {
        on += (g_alertMask & bit) ? 1 : 0;
    }
    return String(on) + " of 4 alerts on, auto-record " +
           (!g_autoRecord ? "off" : recorder::mounted() ? "on" : "on (no SD card)");
}

void handleRadarTouch(int x, int y) {
    String hex, airport;
    switch (radarScreenHandleTouch(x, y, hex, airport)) {
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
            openDetail(tapped, Screen::RADAR);
        } else {
            drawRadar(true);  // gone since: at least take the list off, if it was picked from one
        }
        break;
    }
    case RadarAction::DISMISS:
        drawRadar(true);
        break;
    case RadarAction::TOGGLE_CENTRE:
        g_radarCentre = (g_radarCentre == RadarCentre::HOME) ? RadarCentre::AIRPORT : RadarCentre::HOME;
        saveRadarCentre(g_radarCentre);
        drawRadar(true);
        break;
    case RadarAction::TOGGLE_RECORD:
        toggleRecording();
        break;
    case RadarAction::TOGGLE_AIRPORTS:  // only ever from a hold
        break;
    case RadarAction::OPEN_RECORDINGS:
        // Greyed out without a card, but a tap still has another look for one.
        if (!recorder::mount()) {
            soundKeyClick();
            updateRecButton(true);
            break;
        }
        g_screen = Screen::RECORDINGS;
        recordingsScreenEnter();
        break;
    case RadarAction::ZOOM_AIRPORT:
        if (!openZoom(airport)) {
            drawRadar(true);
        }
        break;
    case RadarAction::NONE:
        break;
    }
}

void handleZoomTouch(int x, int y) {
    String hex;
    switch (radarZoomHandleTouch(x, y, hex)) {
    case ZoomAction::UNZOOM:
        closeZoom();
        break;
    case ZoomAction::SELECT: {
        Aircraft tapped;
        bool found = false;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            for (const Aircraft &a : g_zoomAircraft) {
                if (a.hex == hex) {
                    tapped = a;
                    found = true;
                    break;
                }
            }
            xSemaphoreGive(g_dataMutex);
        }
        if (found) {
            openDetail(tapped, Screen::ZOOM);
        } else {
            drawZoom(true);
        }
        break;
    }
    case ZoomAction::DISMISS:
        drawZoom(true);
        break;
    case ZoomAction::NONE:
        break;
    }
}

void handleRecordingsTouch(int x, int y) {
    String path;
    switch (recordingsScreenHandleTouch(x, y, path)) {
    case RecordingsAction::BACK:
        g_screen = Screen::RADAR;
        drawRadar(true);
        break;
    case RecordingsAction::PLAY:
        if (playbackScreenOpen(path, recordingsScreenLoadProgress)) {
            playbackScreenSetCentre(g_radarCentre);
            playbackScreenSetAirports(g_radarAirports);
            g_screen = Screen::PLAYBACK;
            playbackScreenDraw();
        } else {
            recordingsScreenEnter();  // the card has likely changed under it
        }
        break;
    case RecordingsAction::SHARE:
        g_screen = Screen::SHARE;
        shareScreenEnter();
        break;
    case RecordingsAction::NONE:
        break;
    }
}

void handleShareTouch(int x, int y) {
    if (shareScreenHandleTouch(x, y) == ShareAction::BACK) {
        shareScreenLeave();
        g_screen = Screen::RECORDINGS;
        recordingsScreenEnter();  // the page may have deleted videos meanwhile
    }
}

void handlePlaybackTouch(int x, int y) {
    Aircraft tapped;
    switch (playbackScreenHandleTouch(x, y, tapped)) {
    case PlaybackAction::BACK:
        playbackScreenClose();
        g_screen = Screen::RECORDINGS;
        recordingsScreenEnter();
        break;
    case PlaybackAction::TOGGLE_CENTRE:
        g_radarCentre = (g_radarCentre == RadarCentre::HOME) ? RadarCentre::AIRPORT : RadarCentre::HOME;
        saveRadarCentre(g_radarCentre);
        playbackScreenSetCentre(g_radarCentre);
        playbackScreenDraw();
        break;
    case PlaybackAction::TOGGLE_AIRPORTS:  // only ever from a hold
        break;
    case PlaybackAction::SELECT:
        openDetail(tapped, Screen::PLAYBACK);
        break;
    case PlaybackAction::EXPORT:
        playbackScreenExport();  // blocks until the video is made or cancelled
        break;
    case PlaybackAction::NONE:
        break;
    }
}

// A long press on the radar's centre toggle, while it says AIRPORT, shows or
// hides the other airports around it, here and in replays alike. On HOME it
// does nothing: the airports are part of the airport view, so they are hidden
// there, and a press that changed what shows on switching back would only
// surprise. It clicks, since with no airport in range the plot itself
// wouldn't change to say the hold registered.
bool toggleRadarAirports() {
    if (g_radarCentre != RadarCentre::AIRPORT) {
        return false;
    }
    soundKeyClick();
    g_radarAirports = !g_radarAirports;
    saveRadarAirports(g_radarAirports);
    return true;
}

void handleRadarHold(int x, int y) {
    if (radarScreenHandleHold(x, y) == RadarAction::TOGGLE_AIRPORTS && toggleRadarAirports()) {
        drawRadar(false);  // the header doesn't change
    }
}

void handlePlaybackHold(int x, int y) {
    if (playbackScreenHandleHold(x, y) == PlaybackAction::TOGGLE_AIRPORTS && toggleRadarAirports()) {
        playbackScreenSetAirports(g_radarAirports);
        playbackScreenDraw();
    }
}

// A long press on a table row puts that flight's callsign on the watchlist,
// or takes it off if it is already there.
void handleMainHold(int x, int y) {
    int row;
    if (!displayHitRow(x, y, row)) {
        return;
    }
    String callsign;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        if (row < (int)g_latestAircraft.size()) {
            callsign = g_latestAircraft[row].callsign;
        }
        xSemaphoreGive(g_dataMutex);
    }
    callsign.trim();
    callsign.toUpperCase();
    soundKeyClick();
    // Held to the same rules as a typed entry, so a feed's oddity can't put
    // something on the list that the editor would then refuse to save.
    if (callsign.length() == 0 || callsign == "UNKNOWN" || watchlistProblem(callsign).length()) {
        displayShowNotice("This flight has no callsign to watch for");
        return;
    }

    // Rebuilt from the parsed list rather than edited as typed, so a callsign
    // is found however it was separated - at the cost of commas becoming
    // spaces, which the parser reads the same.
    std::vector<String> entries = parseWatchlist(g_watchlist);
    bool removed = false;
    for (auto it = entries.begin(); it != entries.end();) {
        if (*it == callsign) {
            it = entries.erase(it);
            removed = true;
        } else {
            ++it;
        }
    }
    if (!removed) {
        entries.push_back(callsign);
    }
    String list;
    for (const String &e : entries) {
        list += (list.length() ? " " : "") + e;
    }
    if (list.length() > WATCHLIST_MAX_LEN) {
        displayShowNotice("The watchlist is full - remove something in Settings first");
        return;
    }

    g_watchlist = list;
    saveAlerts(g_alertMask, g_watchlist, g_autoRecord);
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_alertRules.watch = parseWatchlist(g_watchlist);
        g_pollNow = true;  // so the row changes colour now rather than an interval on
        xSemaphoreGive(g_dataMutex);
    }
    String notice = removed ? "Removed " + callsign + " from the watchlist" : "Added " + callsign + " to the watchlist";
    if (!removed && !(g_alertMask & ALERT_WATCHLIST)) {
        notice += " - but watchlist alerts are switched off";
    }
    displayShowNotice(notice);
}

void handleAlertsTouch(int x, int y) {
    switch (alertsScreenHandleTouch(x, y)) {
    case AlertsAction::BACK: {
        g_alertMask = alertsScreenMask();
        g_autoRecord = alertsScreenAutoRecord();
        g_watchlist = alertsScreenWatchlist();
        saveAlerts(g_alertMask, g_watchlist, g_autoRecord);
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_alertRules.enabled = g_alertMask;
            g_alertRules.watch = parseWatchlist(g_watchlist);
            g_pollNow = true;  // so a new rule shows on the next poll, not one interval on
            xSemaphoreGive(g_dataMutex);
        }
        settingsScreenSetAlertSummary(alertSummary());
        g_screen = Screen::SETTINGS;
        settingsScreenDraw();
        break;
    }
    case AlertsAction::OPEN_WATCHLIST:
        watchlistScreenSet(alertsScreenWatchlist());
        g_screen = Screen::WATCHLIST;
        watchlistScreenDraw();
        break;
    case AlertsAction::NONE:
        break;
    }
}

void handleWatchlistTouch(int x, int y) {
    switch (watchlistScreenHandleTouch(x, y)) {
    case WatchlistAction::SAVE:
        alertsScreenSetWatchlist(watchlistScreenText());
        [[fallthrough]];  // both go back to the alerts screen
    case WatchlistAction::CANCEL:
        g_screen = Screen::ALERTS;
        alertsScreenDraw();
        break;
    case WatchlistAction::NONE:
        break;
    }
}

void handleMainTouch(int x, int y) {
    if (displayHitRadar(x, y)) {
        g_screen = Screen::RADAR;
        drawRadar(true);
        return;
    }

    if (displayHitCog(x, y)) {
        String label;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            label = g_label;
            xSemaphoreGive(g_dataMutex);
        }
        settingsScreenSet(g_traffic, g_radiusNm, g_showRefresh, g_pollIntervalS, label, g_source);
        settingsScreenSetAlertSummary(alertSummary());
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
            openDetail(tapped, Screen::MAIN);
        }
    }
}

void handleDetailTouch(int x, int y) {
    if (!detailScreenHandleTouch(x, y)) {
        return;
    }
    if (g_detailReturnTo == Screen::RADAR) {
        g_screen = Screen::RADAR;
        drawRadar(true);
        return;
    }
    if (g_detailReturnTo == Screen::ZOOM) {
        g_screen = Screen::ZOOM;
        drawZoom(true);
        return;
    }
    if (g_detailReturnTo == Screen::PLAYBACK) {
        g_screen = Screen::PLAYBACK;
        playbackScreenDraw();
        return;
    }
    g_screen = Screen::MAIN;
    displayInvalidate();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_dataReady = true;  // force a redraw of the main screen with current data
        xSemaphoreGive(g_dataMutex);
    }
}

// With the mutex held, when the location or the traffic filter changes. The
// last poll's contacts answer the old question - somewhere else, or the other
// filter - so they go now, rather than staying up under the new header until
// a poll for it succeeds: a failed poll leaves the last good one showing, and
// with the network down that is every poll. A changed range alone doesn't
// call for it - those are still real contacts around the same place.
void forgetContactsLocked() {
    g_latestAircraft.clear();
    g_latestIsNew.clear();
    g_dataReady = true;
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
        displaySetHeader(g_traffic == TrafficFilter::MILITARY, g_airportCode, activeProvider());
        settingsScreenSetLocation(label);
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_lat = lat;
            g_lon = lon;
            g_label = label;
            g_resetBaseline = true;
            g_sceneChanged = true;
            g_pollNow = true;
            forgetContactsLocked();
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
        AdsbSource source = settingsScreenSource();
        g_showRefresh = settingsScreenShowRefresh();
        saveFilters(traffic, radius, g_showRefresh, interval, source);
        displaySetShowRefresh(g_showRefresh);
        displaySetHeader(traffic == TrafficFilter::MILITARY, g_airportCode, activeProvider());
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            // A changed interval alone doesn't warrant refetching - the new
            // one simply applies to the wait already running.
            bool changed = (traffic != g_traffic) || (radius != g_radiusNm) || (source != g_source);
            if (traffic != g_traffic || radius != g_radiusNm) {
                g_sceneChanged = true;  // the source doesn't alter what a recording's plot is drawn around
            }
            if (traffic != g_traffic) {
                forgetContactsLocked();
            }
            g_traffic = traffic;
            g_radiusNm = radius;
            g_source = source;
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
    case SettingsAction::OPEN_ALERTS:
        alertsScreenSet(g_alertMask, g_autoRecord, g_watchlist);
        alertsScreenSetCard(recorder::mount());  // one look for a card put in since
        g_screen = Screen::ALERTS;
        alertsScreenDraw();
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

void leaveWifiScreen() {
    // Whatever the screen did, NVS now holds the credentials the poll task
    // should reconnect with - so take them back from there rather than
    // tracking every path through the screen that might have changed them.
    AppSettings s = loadSettings();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_uiOwnsWifi = false;
        g_wifiGaveUp = false;
        g_wifiSsid = s.wifiSsid.length() ? s.wifiSsid : String(WIFI_SSID);
        g_wifiPass = s.wifiSsid.length() ? s.wifiPass : String(WIFI_PASSWORD);
        xSemaphoreGive(g_dataMutex);
    }
    g_nextWifiTryMs = 0;  // a fresh attempt is wanted now, not after the backoff
    g_wifiRetryMs = WIFI_RETRY_MIN_MS;
    g_wifiFails = 0;  // and a full set of them before the screen comes back
    if (g_wifiReturnTo == Screen::MAIN) {
        g_wifiReturnTo = Screen::SETTINGS;  // only the boot and gave-up cases land on the table
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

void handleWifiTouch(int x, int y) {
    if (wifiScreenHandleTouch(x, y) == WifiAction::BACK) {
        leaveWifiScreen();
    }
}

// Puts the WiFi screen up once the poll task has given up on the saved
// network. Only from the live screens: a replay or a half-typed watchlist
// doesn't need the network, and being snatched away from either would be
// worse than the wait. The flag keeps until one of them is back up.
void checkWifiGaveUp() {
    if (g_screen != Screen::MAIN && g_screen != Screen::RADAR && g_screen != Screen::ZOOM &&
        g_screen != Screen::DETAIL) {
        return;
    }
    if (g_screen == Screen::DETAIL && g_detailReturnTo == Screen::PLAYBACK) {
        return;  // an aircraft out of a replay is still the replay
    }
    bool gaveUp = false;
    String ssid;
    if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
        gaveUp = g_wifiGaveUp;
        if (gaveUp) {
            g_wifiGaveUp = false;
            g_uiOwnsWifi = true;  // hands the radio to the screen; see ensureWifi()
            ssid = g_wifiSsid;
        }
        xSemaphoreGive(g_dataMutex);
    }
    if (!gaveUp) {
        return;
    }
    Serial.printf("[wifi] giving up on %s - opening the WiFi screen\n", ssid.c_str());
    g_wifiReturnTo = Screen::MAIN;
    g_screen = Screen::WIFI;
    wifiScreenEnter("Could not reach " + ssid + " - choose a network");
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
    g_source = s.source;
    g_radarCentre = s.radarCentre;
    g_radarAirports = s.radarAirports;
    g_alertMask = s.alertMask;
    g_watchlist = s.watchlist;
    g_autoRecord = s.autoRecord;
    g_alertRules.enabled = g_alertMask;
    g_alertRules.watch = parseWatchlist(g_watchlist);
    soundSetMuted(g_muted);
    displaySetShowRefresh(g_showRefresh);
    displaySetMuted(g_muted);
    g_airportCode = nearestAirportCode(g_lat, g_lon);
    displaySetHeader(g_traffic == TrafficFilter::MILITARY, g_airportCode, g_activeProvider);
    screen::setOverlay(drawBanner, 0, 0, 1280, BANNER_H);

    // Mounted now so the log says whether there is a card; recording mounts
    // it again later if one is put in after boot.
    if (!recorder::mount()) {
        Serial.println("[sd] no card - recording unavailable until one is inserted");
    }

    // What the device actually loaded, said once at boot. An empty table has
    // several innocent explanations - military mode over quiet airspace being
    // the likeliest - and none of them were visible from the log, which made
    // "nothing is showing" impossible to tell from "nothing is working".
    Serial.printf("[settings] %s  %.4f,%.4f  %dnm  every %ds  source=%s\n",
                  g_traffic == TrafficFilter::MILITARY ? "MILITARY" : "CIVIL", g_lat, g_lon, g_radiusNm,
                  g_pollIntervalS,
                  adsbSourceIndex(g_source) < 0 ? "auto" : ADSB_PROVIDERS[adsbSourceIndex(g_source)].name);

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
        if (WiFi.status() != WL_CONNECTED) {
            g_wifiFails = 1;  // the first of the attempts before the WiFi screen comes up
        }
    } else {
        // Nothing to connect with, so don't spend the timeout finding out.
        // The screen scans, which needs the radio in station mode either way.
        Serial.println("[wifi] no credentials set - opening the WiFi screen");
        WiFi.mode(WIFI_STA);
    }
    g_linkUp = (WiFi.status() == WL_CONNECTED);
    // UTC, for naming recordings and stamping their polls. SNTP keeps trying in
    // the background, so a link that comes up later still gets the time.
    configTime(0, 0, "pool.ntp.org", "time.google.com");
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

    // 'S' over serial sends the screen back, for tools/screenshot.py.
    while (Serial.available()) {
        if (Serial.read() == 'S') {
            screen::dumpToSerial();
        }
    }

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
        if (g_zoomUp) {
            g_zoomTouchedMs = millis();  // being looked at, so not to be closed
        }
        if (t.wasHold() && !bannerOver(t.y)) {
            switch (g_screen) {
            case Screen::MAIN:
                handleMainHold(t.x, t.y);
                break;
            case Screen::RADAR:
                handleRadarHold(t.x, t.y);
                break;
            case Screen::PLAYBACK:
                handlePlaybackHold(t.x, t.y);
                break;
            default:
                break;
            }
        } else if (t.wasClicked() && !handleBannerTouch(t.x, t.y)) {
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
            case Screen::ZOOM:
                handleZoomTouch(t.x, t.y);
                break;
            case Screen::ALERTS:
                handleAlertsTouch(t.x, t.y);
                break;
            case Screen::WATCHLIST:
                handleWatchlistTouch(t.x, t.y);
                break;
            case Screen::RECORDINGS:
                handleRecordingsTouch(t.x, t.y);
                break;
            case Screen::PLAYBACK:
                handlePlaybackTouch(t.x, t.y);
                break;
            case Screen::SHARE:
                handleShareTouch(t.x, t.y);
                break;
            default:
                break;
            }
        }
    }

    if (g_screen == Screen::WIFI) {
        // Scans and connection attempts both complete asynchronously. Put up
        // for want of a network, the screen is done once it has one.
        if (wifiScreenTick() == WifiAction::CONNECTED && g_wifiReturnTo == Screen::MAIN) {
            leaveWifiScreen();
        }
    }
    checkWifiGaveUp();

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
    soundTick();
    tickAlerts();

    // Each step of a callsign flash is a redraw of whichever of the screens
    // showing callsigns is up, from the data it already has.
    static uint32_t shownFlashStep = 0;
    uint32_t flashStep = alertFlashStep();
    if (flashStep != shownFlashStep) {
        shownFlashStep = flashStep;
        if ((g_screen == Screen::MAIN || g_screen == Screen::RADAR) &&
            xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        if (g_screen == Screen::ZOOM && xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_zoomReady = true;
            xSemaphoreGive(g_dataMutex);
        }
    }

    if (g_screen == Screen::RADAR) {
        updateRecButton(true);
    }
    if (g_screen == Screen::PLAYBACK) {
        playbackScreenTick();
    }
    if (g_screen == Screen::SHARE) {
        shareScreenTick();
    }

    if (g_screen == Screen::RADAR) {
        bool fresh = false;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            fresh = g_dataReady;
            g_dataReady = false;
            xSemaphoreGive(g_dataMutex);
        }
        if (fresh) {
            drawRadar(false);
        }
    }

    // The zoom is a temporary view, and its extra fetches go with it: left
    // untouched, it closes itself - and from a detail opened from it, Back
    // then goes to the radar instead.
    if (g_zoomUp && millis() - g_zoomTouchedMs > ZOOM_TIMEOUT_MS) {
        if (g_screen == Screen::ZOOM) {
            closeZoom();
        } else if (g_screen == Screen::DETAIL && g_detailReturnTo == Screen::ZOOM) {
            g_detailReturnTo = Screen::RADAR;
        }
    }
    if (g_screen == Screen::ZOOM) {
        bool fresh = false;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            fresh = g_zoomReady;
            g_zoomReady = false;
            xSemaphoreGive(g_dataMutex);
        }
        if (fresh) {
            drawZoom(false);
        }
    }
    // The zoom's fetches stop as soon as neither it nor a detail opened from
    // it is up, however it was left - Unzoom, the timeout, or the WiFi screen
    // taking over.
    if (g_zoomUp && g_screen != Screen::ZOOM && !(g_screen == Screen::DETAIL && g_detailReturnTo == Screen::ZOOM)) {
        endZoom();
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
        String provider;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            linkUp = g_linkUp;
            pollOk = g_pollOk;
            everSucceeded = g_everSucceeded;
            lastSuccess = g_lastSuccessMs;
            provider = g_activeProvider;
            xSemaphoreGive(g_dataMutex);
            displaySetPollState(linkUp, pollOk, everSucceeded, lastSuccess);
        }

        // A failover happens on the poll task with nothing else to announce it,
        // so the header is checked against what answered last. Naming it drops
        // the render cache, and only a render repaints the header - so ask for
        // one too, or the new source wouldn't show until the next poll.
        static String shownProvider;
        if (provider.length() && provider != shownProvider) {
            shownProvider = provider;
            displaySetHeader(g_traffic == TrafficFilter::MILITARY, g_airportCode, provider);
            if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
                g_dataReady = true;
                xSemaphoreGive(g_dataMutex);
            }
        }
        displayTickStatus();
    }

    delay(10);
}
