#include <M5Unified.h>
#include <WiFi.h>
#include <esp32-hal-hosted.h>
#include <esp_heap_caps.h>
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
#include "follow.h"
#include "location_screen.h"
#include "playback_screen.h"
#include "radar_screen.h"
#include "recorder.h"
#include "recordings_screen.h"
#include "runways.h"
#include "screen.h"
#include "secrets.h"
#include "settings.h"
#include "settings_screen.h"
#include "share_screen.h"
#include "sound.h"
#include "telemetry.h"
#include "watchlist_screen.h"
#include "wifi_screen.h"

namespace {

enum class Screen { MAIN, LOCATION, DETAIL, SETTINGS, WIFI, RADAR, ZOOM, ALERTS, WATCHLIST, RECORDINGS, PLAYBACK, SHARE };
Screen g_screen = Screen::MAIN;
// The detail screen is reachable from the table and from the radar, and Back
// should land wherever you came from.
Screen g_detailReturnTo = Screen::MAIN;
Aircraft g_detailAc;  // the aircraft the detail screen shows, for its FOLLOW
// The WiFi screen is normally reached from settings, but a device with no
// credentials opens it straight from boot - where Back belongs on the table.
Screen g_wifiReturnTo = Screen::SETTINGS;

SemaphoreHandle_t g_dataMutex;
std::vector<Aircraft> g_latestAircraft;
std::vector<uint8_t> g_latestIsNew;
bool g_dataReady = false;
bool g_newFlightPending = false;  // set by pollTask, consumed by loop() to beep
uint32_t g_latestSeq = 0;         // counts the main poll's lists, for loop() to time their arrival
uint32_t g_latestDataMs = 0;      // only touched by loop(): when the latest of them arrived

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
// The range ZOOM IN and OUT have chosen for the radar: one of
// RADAR_RANGE_STEPS_NM, or 0 for the whole poll radius. Written by loop()
// under the mutex, for the poll task to record it.
int g_radarRangeNm = 0;
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
// polls, into a list of its own: the table and alerts carry on with the main
// poll's, and a recording takes this one's polls instead, it being what is
// on show. Following an aircraft uses the same poll, aimed at the aircraft
// rather than an airport.
struct ZoomPoll {
    bool active = false;
    uint32_t gen = 0;  // which zoom, so a fetch for one since closed is dropped
    double lat = 0, lon = 0;
    int radiusNm = 0;
    // Aimed at a followed aircraft on the radar rather than at the zoom: then
    // it goes at the main poll's interval, as the radar refreshes, rather than
    // the zoom's quicker one.
    bool follow = false;
    String code;       // the zoom's airport, when it isn't following
    String followHex;  // the aircraft being followed, in either view; empty for none
    float viewNm = 0;  // the zoom's range as ZOOM IN has it, for the recording; 0 for its framing
};
ZoomPoll g_zoomPoll;                   // under the mutex
std::vector<Aircraft> g_zoomAircraft;  // likewise
bool g_zoomReady = false;              // likewise: a new list, for loop() to draw
String g_zoomProvider;                 // likewise: who answered for it
uint32_t g_zoomSeq = 0;                // likewise: counts the lists, for following to look through each once
bool g_zoomUp = false;                 // only touched by loop(): g_zoomPoll.active, without the lock
uint32_t g_zoomDataMs = 0;             // likewise: when its list last arrived, to move it on from
uint32_t g_zoomDrawnMs = 0;            // and when it was last drawn
uint32_t g_radarDrawnMs = 0;           // likewise, the radar
String g_zoomCode;                     // likewise: the airport it is on
float g_zoomRangeNm = 0;               // and how far its plot reaches from g_zoomPoll's middle

// Follow me. Only touched by loop(). FOLLOW on the radar, then a tap on a
// contact, and the radar is centred on that aircraft wherever it goes; low
// near an airport it is handed to that airport's zoom, to watch it land or
// take off, and back to the radar as it leaves.
struct Follow {
    bool picking = false;  // FOLLOW tapped; the next contact tapped is the one
    bool active = false;
    String hex, callsign;
    Aircraft last;         // as last seen, with a position
    uint32_t seenMs = 0;   // when
    uint32_t seq = 0;      // the last of the extra poll's lists looked through
    // Seen in the air, so a stop on the ground after it is an arrival rather
    // than where it was sitting when picked, or holding short to depart.
    bool airborne = false;
    // Seen on the ground and then in the air: a departure, followed until it
    // leaves the radius - see tickFollow().
    bool seenGround = false;
    bool departed = false;
    String originCode;  // the airport it departed from, marked on the radar as it climbs away
    bool recording = false;  // it started the recording running now
    bool inZoom = false;     // has been on the zoom's plot since it opened
    // The zoom up is one following handed it to, or it was picked in - not
    // one tapped open on some other airport, whose poll doesn't reach it.
    bool zoomIsItsOwn = false;
    float zoomDistNm = -1;   // how far from the zoom's middle it was at the last poll, -1 for not yet
    String skipZoom;         // Unzoomed from while following: not handed back to
    std::vector<FollowSample> heights;  // each report, for the telemetry's chart
    int rangeNm = FOLLOW_RANGE_NM;      // the radar's, closing in as it comes down
    // Its reports, for where it is shown - simulated down the runway when
    // it is taking off or landing, and estimated from its last current
    // position while the feed has lost it: `lost` from the first poll
    // without a current position for it to the next with one.
    RunwaySim sim;
    const char *simPhase = "none";  // as last logged
    bool lost = false;
};
Follow g_follow;

// --- where the time goes ------------------------------------------------------
// A line a minute over serial: how long the radar and zoom take to draw, the
// longest pass of loop() - the gap in which a tap can go unseen - and how
// much internal RAM is left, which WiFi and TLS need. Only touched by loop().
struct PerfStat {
    uint32_t count = 0;
    uint64_t totalUs = 0;
    uint32_t maxUs = 0;
    void add(uint32_t us) {
        count++;
        totalUs += us;
        maxUs = std::max(maxUs, us);
    }
    String text() const {
        return count ? String(count) + "x avg " + String((uint32_t)(totalUs / count / 1000)) + " max " +
                           String(maxUs / 1000) + "ms"
                     : String("-");
    }
};
PerfStat g_perfRadar, g_perfZoom, g_perfTracks, g_perfLoop;

// A touch the radar or the zoom has already acted on, as it landed. The rest
// of it - its release, or its turning into a hold - is ignored, so a tap that
// opens another screen doesn't go on to press whatever is under it there.
bool g_touchSpent = false;

// Only touched by pollTask - no locking needed since it's the sole writer/reader.
std::map<String, uint32_t> g_seenMap;
bool g_baseline = true;

// --- alerts and recording ---------------------------------------------------

// The rules pollTask checks each aircraft against, and whether an alert
// starts a recording. Set from the alerts screen, so read under the mutex.
AlertRules g_alertRules;
bool g_autoRecord = false;
// Whether an alert starts following the aircraft it is for. Only touched by
// loop().
bool g_autoFollow = false;
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

// One fetch of the traffic around the zoom's airport - or the aircraft being
// followed - from the poll task.
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
    bool current = false;
    ZoomPoll now;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        current = g_zoomPoll.active && g_zoomPoll.gen == zoom.gen;
        if (current) {
            g_zoomAircraft = fetched;
            g_zoomReady = true;
            g_zoomSeq++;
            g_zoomProvider = provider != nullptr ? provider : "";
            now = g_zoomPoll;  // who is being followed now, rather than when the fetch went
        }
        xSemaphoreGive(g_dataMutex);
    }
    // While this poll is up, a recording is made of it rather than the main
    // poll's - it is what the screens are showing - each marked with the view
    // it was for, so a replay shows it the way it was seen.
    if (current && recorder::active()) {
        String view = now.follow ? "-" + now.followHex : now.code + "/" + now.followHex;
        recorder::addFrame(fetched, view, provider, now.follow ? 0.0f : now.viewNm);
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
            // The radar around home - unless the zoom's poll is up, following
            // or zoomed in, which is then what is on show and recorded instead.
            bool extraUp = false;
            int shownNm = 0;  // the radar's range as ZOOM IN has it, inside the radius
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                extraUp = g_zoomPoll.active;
                shownNm = g_radarRangeNm > 0 && g_radarRangeNm < g_radiusNm ? g_radarRangeNm : 0;
                xSemaphoreGive(g_dataMutex);
            }
            if (!extraUp) {
                recorder::addFrame(aircraft, "H", provider, (float)shownNm);
            }

            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                if (best != nullptr) {
                    g_pendingAlert = *best;
                    g_pendingAlertMore = fresh - 1;
                    g_pendingFlash = std::move(freshHexes);
                    g_alertPending = true;
                }
                g_latestAircraft = std::move(aircraft);
                g_latestIsNew = std::move(isNew);
                g_latestSeq++;
                if (provider != nullptr) {
                    g_activeProvider = provider;  // with the list, for loop() to take the two together
                }
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
        // whatever the main interval is set to. Following on the radar, the
        // fetch around the aircraft goes at the main interval instead: it is
        // only the zoom that wants the sky any fresher. Without either, this
        // is the plain interval it always was.
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
                uint32_t every = zoom.follow ? intervalMs : ZOOM_POLL_MS;
                zoomLate = zoom.gen != zoomGenServed ? INT32_MAX : (int32_t)(t - lastZoomMs - every);
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

void setRadarControls(bool onScreen);

// Where the followed aircraft is shown now: see RunwaySim.
FollowEstimate followEstimateNow() { return g_follow.sim.at(millis()); }

// The status an estimate draws with: its trend chevron, and ring or dot.
String estimateStatus(const FollowEstimate &e) {
    if (e.ground) {
        return e.gsKt > 2 ? "TAXI" : "GROUND";
    }
    return e.vsFpm >= CLIMB_THRESHOLD_FPM ? "CLIMB" : e.vsFpm <= DESCEND_THRESHOLD_FPM ? "DESCEND" : "LEVEL";
}

// Seconds since the followed aircraft's last current position, while it is
// lost; 0 while it isn't.
uint32_t followLostS() {
    return g_follow.lost && g_follow.sim.haveFresh() ? (millis() - g_follow.sim.fresh().ms) / 1000 + 1 : 0;
}

// `base` - an aircraft as last reported - where estimate `e` puts it, dimmed
// as a lost position is, in place of whatever `aircraft` has for it.
void putEstimate(std::vector<Aircraft> &aircraft, const Aircraft &base, const FollowEstimate &e) {
    Aircraft ghost = base;
    ghost.hasPos = true;
    ghost.posStale = true;
    ghost.lat = e.lat;
    ghost.lon = e.lon;
    ghost.hasTrack = true;
    ghost.track = e.track;
    ghost.altStr = e.ground ? String("GND") : String(e.altFt);
    ghost.speedStr = String(e.gsKt);
    ghost.status = estimateStatus(e);
    // In its place where the list has it, so the radar's new-contact marks,
    // which go by position in the list, stay with the contacts they are for.
    for (Aircraft &a : aircraft) {
        if (a.hex == base.hex) {
            a = ghost;
            return;
        }
    }
    aircraft.push_back(ghost);
}

// The followed aircraft, while the feed has lost it - missing from the poll,
// or there with only its last position - put where it is estimated to be
// from its last current one, until a poll has it again or following gives
// up on it. At a touchdown the feed can lose it for a minute or more, and the
// ring freezing in the air just as it lands was the one moment it was there
// to show.
void keepFollowedOnPlot(std::vector<Aircraft> &aircraft) {
    radarSetHeights(g_follow.heights, followLostS());
    if (!g_follow.lost || !g_follow.sim.haveFresh()) {
        return;
    }
    putEstimate(aircraft, g_follow.last, followEstimateNow());
}

// --- every contact -----------------------------------------------------------
// What a followed aircraft gets, every contact gets too: its recent reports
// kept, and while the feed has lost it, an estimate of where it is in its
// place - for ESTIMATE_LOST_MS rather than following's longer wait. Lost,
// that is, if it was low, where the receivers lose them near the ground; or
// at any height, if the list it is missing from came from a different
// provider than the last that had it - one rate-limited, the other answering
// with a fraction of the sky. Otherwise one that goes missing has most likely
// flown out of range, and is let go.

struct Track {
    // Its reports, for where it is shown: see RunwaySim. Their history is
    // kept only while it is low, where it matters - there can be a hundred
    // tracks, and these live in the internal RAM that WiFi and TLS need.
    RunwaySim sim;
    Aircraft last;      // as at its last current position, for its callsign and the rest
    uint32_t freshMs = 0;  // when that was
    // When a list of each kind last had it current: a lost contact is only
    // put back into the kind of list it went missing from.
    uint32_t inMainMs = 0, inZoomMs = 0;
    String provider;  // who answered the list that last had it current
};
std::map<String, Track> g_tracks;  // only touched by loop()

bool lowSample(const FollowSample &s) { return s.ground || (s.hasAlt && s.altFt < ESTIMATE_MAX_FT); }

// Takes in a list that has just arrived - the main poll's, or the zoom's -
// and who answered it.
void noteTracks(const std::vector<Aircraft> &list, bool zoomList, const String &provider) {
    uint32_t now = millis();
    for (const Aircraft &a : list) {
        if (!a.hasPos) {
            continue;
        }
        Track &t = g_tracks[a.hex];
        t.sim.report(followSampleOf(a, now));
        if (!a.posStale) {
            t.last = a;
            t.freshMs = now;
            t.provider = provider;
            (zoomList ? t.inZoomMs : t.inMainMs) = now;
        }
    }
    // Past how long it is kept for without a current position, it can't be
    // put back into any list, so there is nothing left to keep it for.
    for (auto it = g_tracks.begin(); it != g_tracks.end();) {
        if (now - it->second.freshMs > it->second.sim.keepMs()) {
            it = g_tracks.erase(it);
        } else {
            ++it;
        }
    }
}

// `maxAgeS` caps how far on it is taken - see moveOnToNow().
FollowEstimate trackEstimate(const Track &t, float maxAgeS = 1e9f) { return t.sim.at(millis(), maxAgeS); }

// Puts each contact `aircraft` - a list of the kind `zoomList` says, which
// `provider` answered - has lost since it last had it where it is estimated
// to be. The followed one is keepFollowedOnPlot()'s.
void fillLost(std::vector<Aircraft> &aircraft, bool zoomList, const String &provider) {
    uint32_t now = millis();
    for (const auto &kv : g_tracks) {
        const Track &t = kv.second;
        uint32_t inList = zoomList ? t.inZoomMs : t.inMainMs;
        if (inList == 0 || now - inList > t.sim.keepMs() || !t.sim.haveFresh() ||
            (g_follow.active && kv.first == g_follow.hex)) {
            continue;
        }
        bool otherProvider = t.provider.length() && provider.length() && t.provider != provider;
        if (!lowSample(t.sim.fresh()) && !otherProvider) {
            continue;  // most likely flown out of range
        }
        bool current = false;
        for (const Aircraft &a : aircraft) {
            if (a.hex == kv.first) {
                current = a.hasPos && !a.posStale;
                break;
            }
        }
        if (!current) {
            putEstimate(aircraft, t.last, trackEstimate(t));
        }
    }
}

// Where the radar centres while following: where the aircraft is estimated
// to be by now - moved on between polls as its blip is, and on through a gap
// in the feed.
void followCentre(double &lat, double &lon) {
    lat = g_follow.last.lat;
    lon = g_follow.last.lon;
    if (g_follow.sim.haveFresh()) {
        FollowEstimate e = followEstimateNow();
        lat = e.lat;
        lon = e.lon;
    }
}

// `a`, as the poll has it, where `e` puts it - and, while `sim` has it taking
// off or landing, pointing and climbing or rolling as it is shown doing, so
// its blip lifts off or touches down where it is seen to.
void showAt(Aircraft &a, const RunwaySim &sim, const FollowEstimate &e) {
    a.lat = e.lat;
    a.lon = e.lon;
    if (sim.simulating(millis())) {
        a.hasTrack = true;
        a.track = e.track;
        a.altStr = e.ground ? String("GND") : String(e.altFt);
        a.speedStr = String(e.gsKt);
        a.status = estimateStatus(e);
    }
}

// Moves contacts on from where the poll that arrived at `dataMs` put them, to
// now: each along its track at its groundspeed, and a followed one by the
// estimate, which knows to bring it down a runway rather than straight on.
// Not past two of the poll's intervals, `intervalS`: a feed that has stopped
// answering doesn't send the sky sailing on without it. Between polls the
// radar and the zoom are redrawn with it every tweenMs(), so the traffic
// moves in steps of a few seconds rather than jumping at each poll.
void moveOnToNow(std::vector<Aircraft> &aircraft, uint32_t dataMs, int intervalS) {
    float ageS = std::min((millis() - dataMs) / 1000.0f, 2.0f * intervalS);
    for (Aircraft &a : aircraft) {
        if (!a.hasPos || a.posStale) {
            continue;  // a lost one is the estimate's, already moved on
        }
        if (g_follow.active && a.hex == g_follow.hex && g_follow.sim.haveFresh()) {
            showAt(a, g_follow.sim, followEstimateNow());
            continue;
        }
        // Low, by the estimate too: one rolling out slows down the runway
        // rather than carrying on at its touchdown speed.
        auto track = g_tracks.find(a.hex);
        if (track != g_tracks.end() && track->second.freshMs >= dataMs) {
            showAt(a, track->second.sim, trackEstimate(track->second, 2.0f * intervalS));
            continue;
        }
        if (!a.hasTrack) {
            continue;
        }
        float d = std::max(a.speedStr.toFloat(), 0.0f) * ageS / 3600.0f;
        float t = a.track * (float)M_PI / 180.0f;
        a.lat += cosf(t) * d / 60.0f;
        a.lon += sinf(t) * d / (60.0f * cosf(a.lat * (float)M_PI / 180.0f));
    }
}

// How often a view polled every `intervalS` is redrawn between its polls:
// TWEEN_FRAMES times between each, but at least every TWEEN_MAX_MS, so the
// radar's half-minute polls get more of them than the zoom's ten seconds.
uint32_t tweenMs(int intervalS) {
    return std::min((uint32_t)intervalS * 1000 / (TWEEN_FRAMES + 1), TWEEN_MAX_MS);
}

// How far the radar shows when not following: the range ZOOM IN and OUT
// chose, inside the poll radius - which a range chosen before the radius was
// brought in under it falls back to.
int radarRangeShown() {
    return g_radarRangeNm > 0 && g_radarRangeNm < g_radiusNm ? g_radarRangeNm : g_radiusNm;
}

// The widest the poll radius may be, as the settings screen's slider allows
// for the traffic chosen.
int maxRadiusNow() {
    return g_traffic == TrafficFilter::MILITARY ? MILITARY_MAX_RADIUS_NM : CIVIL_MAX_RADIUS_NM;
}

// Widens the poll radius from the radar's ZOOM OUT, as the settings screen
// would: saved, the sky refetched, and a recording of the narrower one ended.
void setPollRadius(int radiusNm) {
    saveFilters(g_traffic, radiusNm, g_showRefresh, g_pollIntervalS, g_source);
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_radiusNm = radiusNm;
        g_sceneChanged = true;
        g_resetBaseline = true;
        g_pollNow = true;
        xSemaphoreGive(g_dataMutex);
    }
}

// The airport ZOOM IN goes on to from the closest range: the one at the
// radar's centre - or, centred on home, the nearest on the plot at that range.
bool radarZoomTarget(String &code) {
    Airport a;
    float withinNm = g_radarCentre == RadarCentre::AIRPORT ? 120.0f : (float)RADAR_RANGE_STEPS_NM[0];
    double lat, lon;
    float rangeNm;
    if (!nearestAirport(g_lat, g_lon, a, withinNm) || !radarZoomFrame(a.code, lat, lon, rangeNm)) {
        return false;
    }
    code = a.code;
    return true;
}

void drawRadar(bool full) {
    uint32_t t0 = micros();
    if (full) {
        // So the header is drawn showing the state as it is now.
        updateRecButton(false);
        setRadarControls(false);
    }
    std::vector<Aircraft> aircraft;
    std::vector<uint8_t> isNew;
    double lat = 0, lon = 0;
    int radius = DEFAULT_RADIUS_NM;
    bool military = false;
    String provider;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        provider = g_follow.active ? g_zoomProvider : g_activeProvider;
        // Following, the plot is the list fetched around the aircraft,
        // centred on where it is reckoned to be. Nothing in that list is
        // marked new: the marks belong to the main poll's.
        aircraft = g_follow.active ? g_zoomAircraft : g_latestAircraft;
        if (!g_follow.active) {
            isNew = g_latestIsNew;
        }
        lat = g_lat;
        lon = g_lon;
        radius = g_radiusNm;
        military = (g_traffic == TrafficFilter::MILITARY);
        xSemaphoreGive(g_dataMutex);
    }
    if (g_follow.active) {
        followCentre(lat, lon);
        radius = g_follow.rangeNm;
        keepFollowedOnPlot(aircraft);
    } else {
        radius = radarRangeShown();
    }
    fillLost(aircraft, g_follow.active, provider);
    // Following, the list is the extra poll's, which on the radar goes at the
    // main poll's interval too.
    moveOnToNow(aircraft, g_follow.active ? g_zoomDataMs : g_latestDataMs, g_pollIntervalS);
    g_radarDrawnMs = millis();
    radarScreenDraw(aircraft, isNew, lat, lon, radius, military, full);
    g_perfRadar.add(micros() - t0);
}

// The radar's buttons, from the state here.
void setRadarControls(bool onScreen) {
    FollowButton follow = g_follow.active ? FollowButton::ON : g_follow.picking ? FollowButton::PICKING : FollowButton::OFF;
    radarScreenSetControls(g_radarCentre, g_radarAirports, follow, g_follow.hex, g_follow.callsign,
                           g_follow.originCode, onScreen);
    // Following sets its own range.
    int shown = radarRangeShown();
    String code;
    bool canIn = shown > RADAR_RANGE_STEPS_NM[0] || radarZoomTarget(code);
    bool canOut = shown < g_radiusNm || g_radiusNm < maxRadiusNow();
    radarScreenSetZoom(!g_follow.active && canIn, !g_follow.active && canOut, onScreen);
}

bool openZoom(const String &code, bool forFollowed = false);

// ZOOM IN or OUT: the next of the steps in that direction from the range
// shown now. In from the closest, the airport's own zoom; out past the poll
// radius, the radius itself, as far as the settings allow.
void zoomRadar(bool in) {
    int shown = radarRangeShown(), chosen = 0;
    if (in) {
        int next = 0;
        for (int r : RADAR_RANGE_STEPS_NM) {
            if (r < shown) {
                next = r;
            }
        }
        if (next == 0) {
            String code;
            if (radarZoomTarget(code)) {
                openZoom(code);  // Unzoom comes back to the radar at this range
            }
            return;
        }
        chosen = next;
    } else if (shown < g_radiusNm) {
        int next = g_radiusNm;
        for (int r : RADAR_RANGE_STEPS_NM) {
            if (r > shown && r < g_radiusNm) {
                next = r;
                break;
            }
        }
        chosen = next >= g_radiusNm ? 0 : next;
    } else {
        int next = maxRadiusNow();
        for (int r : RADAR_RANGE_STEPS_NM) {
            if (r > g_radiusNm && r < next) {
                next = r;
                break;
            }
        }
        setPollRadius(next);
    }
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_radarRangeNm = chosen;
        xSemaphoreGive(g_dataMutex);
    }
    saveRadarRange(g_radarRangeNm);
    setRadarControls(true);
    drawRadar(true);  // full: the range readout is outside the plot's part
}

// The zoom's range as ZOOM IN and OUT have it, for its polls' frames to be
// recorded with.
void shareZoomView() {
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.viewNm = radarZoomViewNm();
        xSemaphoreGive(g_dataMutex);
    }
}

void drawZoom(bool full) {
    uint32_t t0 = micros();
    if (full) {
        updateRecButton(false);  // so REC is drawn showing the state as it is now
    }
    std::vector<Aircraft> aircraft;
    String provider;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        aircraft = g_zoomAircraft;
        provider = g_zoomProvider;
        xSemaphoreGive(g_dataMutex);
    }
    if (g_follow.active) {
        keepFollowedOnPlot(aircraft);
    }
    fillLost(aircraft, true, provider);
    moveOnToNow(aircraft, g_zoomDataMs, ZOOM_POLL_INTERVAL_S);
    g_zoomDrawnMs = millis();
    radarZoomDraw(aircraft, full);
    g_perfZoom.add(micros() - t0);
}

// Zooms in on an airport tapped on the radar, or one the followed aircraft
// is coming down to. Until the zoom's own first fetch lands, the radar's
// contacts stand in: around an airport inside the radius they are the same
// aircraft, a poll older - and following, those fetched around the aircraft
// are, the airport being near it.
bool openZoom(const String &code, bool forFollowed) {
    double lat, lon;
    float rangeNm;
    if (!radarZoomOpen(code, lat, lon, rangeNm)) {
        return false;
    }
    g_zoomCode = code;
    g_zoomRangeNm = rangeNm;
    g_follow.picking = false;
    g_follow.inZoom = false;
    g_follow.zoomDistNm = -1;
    g_follow.zoomIsItsOwn = forFollowed;
    radarZoomSetFollow(g_follow.active ? g_follow.hex : String(), g_follow.callsign, false);
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.active = true;
        g_zoomPoll.follow = false;
        g_zoomPoll.code = code;
        g_zoomPoll.followHex = g_follow.active ? g_follow.hex : String();
        g_zoomPoll.gen++;
        g_zoomPoll.lat = lat;
        g_zoomPoll.lon = lon;
        g_zoomPoll.radiusNm = (int)ceilf(rangeNm) + ZOOM_FETCH_EXTRA_NM;
        g_zoomPoll.viewNm = 0;  // opened at its framing
        if (!g_follow.active) {
            g_zoomAircraft = g_latestAircraft;
            g_zoomDataMs = g_latestDataMs;  // moved on between polls from when it really arrived
        }
        g_zoomReady = false;
        xSemaphoreGive(g_dataMutex);
    }
    g_zoomUp = true;
    g_screen = Screen::ZOOM;
    drawZoom(true);
    return true;
}

void followCentre(double &lat, double &lon);

// Aims the extra poll at the aircraft being followed. `fresh` for a new aim -
// following just started, or back from a zoom - which is fetched straight
// away; without it, the poll is only moved along with the aircraft.
void aimFollowPoll(bool fresh) {
    // Where it is reckoned to be by now, which after a while in a zoom of
    // some other airport can be a long way from where it was last seen.
    double lat, lon;
    followCentre(lat, lon);
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.active = true;
        g_zoomPoll.follow = true;
        g_zoomPoll.followHex = g_follow.hex;
        if (fresh) {
            g_zoomPoll.gen++;
        }
        g_zoomPoll.lat = lat;
        g_zoomPoll.lon = lon;
        g_zoomPoll.radiusNm = g_follow.rangeNm + ZOOM_FETCH_EXTRA_NM;
        xSemaphoreGive(g_dataMutex);
    }
}

// Stops the zoom's fetches - or, following, aims them back at the aircraft.
// Called whichever way it was left - see loop().
void endZoom() {
    if (!g_zoomUp) {
        return;
    }
    g_zoomUp = false;
    if (g_follow.active) {
        aimFollowPoll(true);  // the zoom's list stands in until the first fetch lands
        return;
    }
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.active = false;
        g_zoomAircraft.clear();
        g_zoomReady = false;
        xSemaphoreGive(g_dataMutex);
    }
}

void closeZoom() {
    if (!g_follow.active) {
        g_follow.picking = false;  // tapped in the zoom, it was for the zoom's contacts
    }
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
void autoFollow(const Aircraft &ac);

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
        if (g_autoFollow) {
            autoFollow(g_bannerAc);
        }
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
    DetailFollow follow = returnTo == Screen::PLAYBACK || !ac.hasPos ? DetailFollow::NONE
                          : g_follow.active && g_follow.hex == ac.hex ? DetailFollow::UNFOLLOW
                                                                      : DetailFollow::FOLLOW;
    detailScreenSet(ac, follow);
    g_detailAc = ac;
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
    bool zoom = (g_screen == Screen::ZOOM);
    radarScreenSetRecording(state, elapsed, onScreen && (zoom || g_screen == Screen::RADAR), zoom);
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
        // By hand, it runs until REC is tapped again, of whatever is on show
        // as it goes - see recorder.h - and while following, it says who.
        recorder::Header h;
        time_t wall = time(nullptr);
        h.startEpoch = wall > 1700000000 ? (uint32_t)wall : 0;
        h.trigger = recorder::Trigger::MANUAL;
        if (g_follow.active) {
            h.note = g_follow.callsign;
            h.followHex = g_follow.hex;
            h.followRangeNm = FOLLOW_RANGE_NM;
        }
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

bool onGround(const Aircraft &ac) { return ac.status == "GROUND" || ac.status == "TAXI"; }

// An aircraft drawn where it is estimated to be, which no list has any more:
// as it was last reported, for its details to be shown.
bool lastReported(const String &hex, Aircraft &out) {
    if (g_follow.active && hex == g_follow.hex) {
        out = g_follow.last;
        return true;
    }
    auto it = g_tracks.find(hex);
    if (it == g_tracks.end()) {
        return false;
    }
    out = it->second.last;
    return true;
}

// The aircraft with ICAO `hex` in the list the radar is showing - the main
// poll's, or following, the one fetched around the followed aircraft.
bool findShown(const String &hex, Aircraft &out) {
    bool found = false;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        for (const Aircraft &a : g_follow.active ? g_zoomAircraft : g_latestAircraft) {
            if (a.hex == hex) {
                out = a;
                found = true;
                break;
            }
        }
        xSemaphoreGive(g_dataMutex);
    }
    return found || lastReported(hex, out);
}

// Following starts a recording when auto-record is on; it stops once the
// aircraft has come to rest on the ground, or following does. One started by
// hand is left to run, since it was asked for. One an alert started gives way
// - more often than not it is of this very aircraft, a watchlist entry being
// what makes one worth following, and it would only ever show it around home
// - and auto-record is held off, as it is when one is stopped by hand, so it
// doesn't start another once following's stops with the alert still about.
//
// One started by hand, with REC, is the user's: it runs until REC is tapped
// again, through following or not.
void startFollowRecording() {
    if (!g_autoRecord) {
        return;
    }
    if (recorder::active()) {
        if (recorder::activeTrigger() != recorder::Trigger::AUTO) {
            return;
        }
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_autoRecordHeld = true;
            xSemaphoreGive(g_dataMutex);
        }
        Serial.println("[follow] taking over from the alert's recording");
        // Auto-follow follows an alert's aircraft from the poll that raised
        // it, which has started the alert's recording already: just begun,
        // it is a frame or two that the following one carries on from, and
        // left on the card it was a recording of next to nothing.
        bool stub = millis() - recorder::activeSinceMs() < ALERT_STUB_MS;
        String path = recorder::activePath();
        recorder::stop();
        if (stub && recorder::remove(path)) {
            Serial.printf("[rec] removed %s - the follow's recording takes over from it\n", path.c_str());
        }
    }
    recorder::Header h;
    time_t wall = time(nullptr);
    h.startEpoch = wall > 1700000000 ? (uint32_t)wall : 0;
    h.trigger = recorder::Trigger::FOLLOW;
    h.note = g_follow.callsign;
    h.followHex = g_follow.hex;
    h.followRangeNm = FOLLOW_RANGE_NM;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        h.lat = g_lat;
        h.lon = g_lon;
        h.radiusNm = g_radiusNm;
        h.military = (g_traffic == TrafficFilter::MILITARY);
        xSemaphoreGive(g_dataMutex);
    }
    g_follow.recording = recorder::start(h);
}

// Whether the recording following started is still the one running: REC
// may have stopped it, and another been started since.
bool followRecording() {
    return g_follow.recording && recorder::active() && recorder::activeTrigger() == recorder::Trigger::FOLLOW;
}

void stopFollowRecording() {
    if (followRecording()) {
        recorder::stop();
    }
    g_follow.recording = false;
}

// Adds a report to the telemetry's history, dropping what has scrolled off
// the chart.
void noteHeight(const Aircraft &ac) {
    uint32_t now = millis();
    std::vector<FollowSample> &h = g_follow.heights;
    h.push_back(followSampleOf(ac, now));
    while (!h.empty() && now - h.front().ms > TELEMETRY_WINDOW_MS) {
        h.erase(h.begin());
    }
    g_follow.sim.report(h.back());
}

void startFollow(const Aircraft &ac) {
    g_follow = Follow();
    g_follow.active = true;
    g_follow.hex = ac.hex;
    g_follow.callsign = ac.callsign;
    g_follow.last = ac;
    g_follow.seenMs = millis();
    g_follow.airborne = !onGround(ac);
    g_follow.seenGround = onGround(ac);
    g_follow.rangeNm = followRangeFor(ac);
    noteHeight(ac);
    Serial.printf("[follow] %s (%s)\n", ac.callsign.c_str(), ac.hex.c_str());
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        // What the radar shows now stands in until the first fetch around it
        // lands, a few seconds from now.
        if (g_zoomAircraft.empty()) {
            g_zoomAircraft = g_latestAircraft;
            g_zoomDataMs = g_latestDataMs;
        }
        g_follow.seq = g_zoomSeq;
        g_zoomPoll.followHex = ac.hex;  // for the zoom's polls, if it is up, to be recorded as following it
        xSemaphoreGive(g_dataMutex);
    }
    startFollowRecording();  // first, so the fetch about to go is its first poll
    // Picked in a zoom, the zoom's poll around its airport already has it.
    if (!g_zoomUp) {
        aimFollowPoll(true);
    }
}

// Back to the radar's own centre. The zoom, if following had it up, stays up
// as an ordinary one.
void stopFollow() {
    if (!g_follow.active) {
        g_follow.picking = false;
        return;
    }
    Serial.printf("[follow] stopped following %s\n", g_follow.callsign.c_str());
    stopFollowRecording();
    g_follow = Follow();
    radarZoomSetFollow(String(), String(), false);
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.followHex = "";
        if (!g_zoomUp) {
            g_zoomPoll.active = false;
            g_zoomAircraft.clear();
            g_zoomReady = false;
        }
        xSemaphoreGive(g_dataMutex);
    }
}

// The airport the followed aircraft should be handed to the zoom of, if any:
// one it is low near and not climbing away from - coming in to land, or
// still on the ground to take off - and close enough to be on its zoom's plot
// by the time it is next fetched. The radar fetches it only as often as the
// main poll, which at thirty seconds would otherwise see it well down final
// - or on the runway - before the zoom, at ten, took over.
//
// Every airport near it is a candidate, nearest first, not only the nearest:
// on final to one, it can pass closer to another.
bool followZoomTarget(const Aircraft &ac, String &code) {
    if (!ac.hasPos || ac.status == "CLIMB") {
        return false;
    }
    float leadNm = std::max(ac.speedStr.toFloat(), 0.0f) * g_pollIntervalS / 3600.0f;
    for (const Airport &airport : airportsWithin(ac.lat, ac.lon, FOLLOW_ZOOM_SEARCH_NM)) {
        if (airport.code == g_follow.skipZoom) {
            continue;
        }
        if (!onGround(ac)) {
            int count = 0;
            const Runway *runways = runwaysAt(airport.code, count);
            int fieldFt = count ? runways[0].elevationFt : 0;
            if (ac.altStr == "?" || ac.altStr.toInt() - fieldFt > FOLLOW_ZOOM_AGL_FT) {
                continue;
            }
        }
        double lat, lon;
        float rangeNm;
        if (radarZoomFrame(airport.code, lat, lon, rangeNm) &&
            haversineNm(lat, lon, ac.lat, ac.lon) <= rangeNm + leadNm) {
            code = airport.code;
            return true;
        }
    }
    return false;
}

// Keeps following up to date with each list fetched around the aircraft:
// where it is, whether it has landed, and which view it belongs in.
void tickFollow() {
    if (!g_follow.active) {
        return;
    }
    Aircraft ac;
    bool fresh = false, found = false;
    if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
        if (g_zoomSeq != g_follow.seq) {
            g_follow.seq = g_zoomSeq;
            fresh = true;
            for (const Aircraft &a : g_zoomAircraft) {
                if (a.hex == g_follow.hex) {
                    ac = a;
                    found = a.hasPos;
                    break;
                }
            }
        }
        xSemaphoreGive(g_dataMutex);
    }
    uint32_t now = millis();
    // In a zoom tapped open on some other airport, its poll doesn't reach the
    // aircraft, and its absence from it says nothing: following carries on,
    // the radar centred on it again from the moment the zoom closes.
    if (g_zoomUp && !g_follow.zoomIsItsOwn && !found) {
        g_follow.seenMs = now;
        return;
    }
    if (found) {
        g_follow.last = ac;
        g_follow.seenMs = now;
        noteHeight(ac);
        if (strcmp(g_follow.sim.phase(), g_follow.simPhase) != 0) {
            Serial.printf("[follow] %s: %s -> %s\n", g_follow.callsign.c_str(), g_follow.simPhase,
                          g_follow.sim.phase());
            g_follow.simPhase = g_follow.sim.phase();
        }
        if (ac.callsign.length() && ac.callsign != g_follow.callsign) {
            g_follow.callsign = ac.callsign;  // one picked before its callsign came through
            setRadarControls(g_screen == Screen::RADAR);
        }
        if (onGround(ac)) {
            g_follow.seenGround = true;
        } else {
            g_follow.airborne = true;
            if (g_follow.seenGround && !g_follow.departed) {
                g_follow.departed = true;
                // The zoom it is in, or the airport nearest where it lifted off.
                Airport from;
                if (g_zoomUp && g_follow.zoomIsItsOwn) {
                    g_follow.originCode = g_zoomCode;
                } else if (nearestAirport(ac.lat, ac.lon, from, 5.0f)) {
                    g_follow.originCode = from.code;
                }
                setRadarControls(false);  // drawn with it from the next redraw
                Serial.printf("[follow] %s is off from %s\n", g_follow.callsign.c_str(),
                              g_follow.originCode.length() ? g_follow.originCode.c_str() : "?");
            }
        }
    } else if (now - g_follow.seenMs > std::max(FOLLOW_LOST_MS, g_follow.sim.keepMs())) {
        Serial.printf("[follow] lost %s\n", g_follow.callsign.c_str());
        stopFollow();
        if (g_screen == Screen::RADAR) {
            drawRadar(true);
        } else if (g_screen == Screen::ZOOM) {
            drawZoom(true);
        }
        return;
    }
    if (fresh) {
        bool wasLost = g_follow.lost;
        g_follow.lost = !found || ac.posStale;
        if (g_follow.lost != wasLost) {
            if (g_follow.lost && g_follow.sim.haveFresh()) {
                const FollowSample &f = g_follow.sim.fresh();
                FollowEstimate e = followEstimateNow();
                Serial.printf("[follow] %s lost - estimating from %s %d kt%s track %s: %s (%s)\n",
                              g_follow.callsign.c_str(), f.ground ? "GND" : String(f.altFt).c_str(), f.gsKt,
                              f.ground ? "" : " ft", f.hasTrack ? String((int)f.track).c_str() : "none", e.how,
                              g_follow.sim.phase());
            } else {
                Serial.printf(g_follow.lost ? "[follow] %s lost - nothing to estimate from\n"
                                            : "[follow] %s found again\n",
                              g_follow.callsign.c_str());
            }
        }
    }
    if (!found) {
        if (fresh) {
            Serial.printf("[follow] %s not in this poll - last seen %lus ago\n", g_follow.callsign.c_str(),
                          (unsigned long)((now - g_follow.seenMs) / 1000));
        }
        return;
    }

    // Down and stopped - or stopped saying how fast it is, as many do once
    // they are parked - after having been seen moving: it has arrived.
    bool stopped = ac.speedStr == "?" || ac.speedStr.toInt() == 0;
    if (followRecording() && g_follow.airborne && !g_follow.departed && onGround(ac) && stopped) {
        Serial.printf("[follow] %s has stopped on the ground - ending its recording\n", g_follow.callsign.c_str());
        stopFollowRecording();
        updateRecButton(true);
    }

    // A departure is followed until it is beyond the radius, where the rest
    // of the radar's sky stops: by then it is on its way to somewhere else.
    if (g_follow.departed && !g_follow.lost && haversineNm(g_lat, g_lon, ac.lat, ac.lon) > g_radiusNm) {
        Serial.printf("[follow] %s has left the %d nm radius - done following it\n", g_follow.callsign.c_str(),
                      g_radiusNm);
        stopFollow();
        if (g_screen == Screen::RADAR) {
            drawRadar(true);
        } else if (g_screen == Screen::ZOOM) {
            drawZoom(true);
        }
        return;
    }

    if (g_zoomUp) {
        // Back to the radar once it has been on the zoom's plot and is
        // flying off it - climbing out, or gone around: in the air, further
        // out than at the last poll, and off the plot by the next at the
        // speed it is going. Or, whichever way it is going, well clear.
        double d = haversineNm(g_zoomPoll.lat, g_zoomPoll.lon, ac.lat, ac.lon);
        if (d <= g_zoomRangeNm) {
            g_follow.inZoom = true;
        }
        float leadNm = std::max(ac.speedStr.toFloat(), 0.0f) * ZOOM_POLL_INTERVAL_S / 3600.0f;
        bool leaving = g_follow.zoomDistNm >= 0 && d > g_follow.zoomDistNm && d + leadNm > g_zoomRangeNm;
        g_follow.zoomDistNm = d;
        Serial.printf("[follow] %s %s ft %s kt %s%s, %.1f nm from %s's middle\n", g_follow.callsign.c_str(),
                      ac.altStr.c_str(), ac.speedStr.c_str(), ac.status.c_str(), ac.posStale ? " (last position)" : "",
                      d, g_zoomCode.c_str());
        if (g_screen == Screen::ZOOM && g_follow.inZoom && !onGround(ac) &&
            (leaving || d > g_zoomRangeNm * FOLLOW_UNZOOM_RANGES)) {
            Serial.printf("[follow] %s has left %s\n", g_follow.callsign.c_str(), g_zoomCode.c_str());
            g_follow.rangeNm = followRangeFor(ac);  // what it had coming down is no use climbing away
            // Departed, it isn't handed back to the airport it left - one
            // levelling off low in the circuit would go back and forth. Gone
            // around, it may be, for its next approach.
            if (g_follow.departed) {
                g_follow.skipZoom = g_zoomCode;
            }
            closeZoom();
        }
        return;
    }
    // Closing in on its destination, the range and the fetch around it with
    // it: the plot keeps the airport on it, and the answers get smaller.
    int range = followRangeFor(ac);
    if (range != g_follow.rangeNm) {
        Serial.printf("[follow] %s range %d -> %d nm\n", g_follow.callsign.c_str(), g_follow.rangeNm, range);
        g_follow.rangeNm = range;
    }
    aimFollowPoll(false);
    String code;
    bool handOff = followZoomTarget(ac, code);
    // A line a poll, so a landing that wasn't handed to its zoom can be
    // worked out afterwards from the serial log.
    Airport dest;
    float destNm = 0;
    bool haveDest = followDestination(ac, dest, destNm);
    Serial.printf("[follow] %s %s ft %s kt %s, %s %.1f nm, range %d, screen %d%s%s\n", g_follow.callsign.c_str(),
                  ac.altStr.c_str(), ac.speedStr.c_str(), ac.status.c_str(), haveDest ? dest.code.c_str() : "-",
                  destNm, g_follow.rangeNm, (int)g_screen, handOff ? ", zoom " : "", handOff ? code.c_str() : "");
    if (handOff && g_screen == Screen::RADAR) {
        openZoom(code, true);
    }
}

// Auto-follow: an alert just fired for `ac`, so it is followed - unless
// something is already, which is left alone. From the table, the radar or a
// zoom, the radar comes up centred on it, a zoom closing first - following
// hands it to its airport's zoom again if that is where it is. Anywhere else
// it is followed out of sight, with the radar ready on return, and recorded
// with auto-record on, as any following is.
void autoFollow(const Aircraft &ac) {
    if (g_follow.active || !ac.hasPos) {
        return;
    }
    Serial.printf("[follow] auto-following %s\n", ac.callsign.c_str());
    bool showRadar = (g_screen == Screen::MAIN || g_screen == Screen::RADAR || g_screen == Screen::ZOOM);
    if (g_zoomUp) {
        // A zoom on some other airport's poll would keep it from the
        // aircraft's: it goes, and a detail opened from it goes back to the
        // radar instead.
        if (g_screen == Screen::DETAIL && g_detailReturnTo == Screen::ZOOM) {
            g_detailReturnTo = Screen::RADAR;
        }
        g_follow.picking = false;
        endZoom();
    }
    g_follow.picking = false;
    startFollow(ac);
    if (showRadar) {
        g_screen = Screen::RADAR;
        drawRadar(true);
    }
}

void handleRadarTouch(int x, int y) {
    String hex, airport;
    RadarAction action = radarScreenHandleTouch(x, y, hex, airport);
    switch (action) {
    case RadarAction::BACK:
        // Following carries on - and records, if it is - with the radar out
        // of sight. FOLLOW is how it stops.
        g_follow.picking = false;
        g_screen = Screen::MAIN;
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        break;
    case RadarAction::SELECT: {
        Aircraft tapped;
        bool found = findShown(hex, tapped);
        if (found && g_follow.picking) {
            stopFollow();  // whoever was followed before, if anyone
            startFollow(tapped);
            setRadarControls(true);
            drawRadar(true);  // full: the range readout changes, and only part of it is in the plot
        } else if (found) {
            openDetail(tapped, Screen::RADAR);
        } else {
            drawRadar(true);  // gone since: at least take the list off, if it was picked from one
        }
        break;
    }
    case RadarAction::DISMISS:
        drawRadar(true);
        break;
    // Each button is lit as soon as it is tapped, so the tap shows it
    // registered before the plot - the slower part - is redrawn under it.
    case RadarAction::CENTRE_HOME:
    case RadarAction::CENTRE_AIRPORT:
    case RadarAction::CENTRE_ALL: {
        RadarCentre centre = (action == RadarAction::CENTRE_HOME) ? RadarCentre::HOME : RadarCentre::AIRPORT;
        bool airports = (action == RadarAction::CENTRE_ALL);
        bool wasFollowing = g_follow.active;
        stopFollow();  // a view of its own is the end of following, or of picking
        bool changed = wasFollowing || centre != g_radarCentre || airports != g_radarAirports;
        if (centre != g_radarCentre) {
            g_radarCentre = centre;
            saveRadarCentre(g_radarCentre);
        }
        if (airports != g_radarAirports) {
            g_radarAirports = airports;
            saveRadarAirports(g_radarAirports);
        }
        setRadarControls(true);
        if (wasFollowing) {
            drawRadar(true);  // the range changes back too
        } else if (changed) {
            drawRadar(false);
        }
        break;
    }
    case RadarAction::TOGGLE_FOLLOW:
        if (g_follow.active) {
            stopFollow();
            setRadarControls(true);
            drawRadar(true);
        } else {
            g_follow.picking = !g_follow.picking;
            setRadarControls(true);
        }
        break;
    case RadarAction::TOGGLE_RECORD:
        toggleRecording();
        break;
    case RadarAction::OPEN_RECORDINGS:
        // Greyed out without a card, but a tap still has another look for one.
        if (!recorder::mount()) {
            soundKeyClick();
            updateRecButton(true);
            break;
        }
        g_follow.picking = false;
        g_screen = Screen::RECORDINGS;
        recordingsScreenEnter();
        break;
    case RadarAction::ZOOM_AIRPORT:
        if (!openZoom(airport)) {
            drawRadar(true);
        }
        break;
    case RadarAction::ZOOM_IN:
    case RadarAction::ZOOM_OUT:
        zoomRadar(action == RadarAction::ZOOM_IN);
        break;
    case RadarAction::NONE:
        break;
    }
}

void handleZoomTouch(int x, int y) {
    String hex;
    switch (radarZoomHandleTouch(x, y, hex)) {
    case ZoomAction::UNZOOM:
        // Following, back to the radar centred on it - and not handed to
        // this airport's zoom again, which would undo the tap.
        if (g_follow.active) {
            g_follow.skipZoom = g_zoomCode;
        }
        closeZoom();
        break;
    case ZoomAction::TOGGLE_RECORD:
        toggleRecording();
        break;
    case ZoomAction::TOGGLE_FOLLOW:
        if (g_follow.active) {
            stopFollow();
        } else {
            g_follow.picking = !g_follow.picking;
            radarZoomSetFollow(String(), String(), g_follow.picking);
        }
        drawZoom(true);
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
        found = found || lastReported(hex, tapped);
        if (found && g_follow.picking) {
            // Followed from here, the zoom stays on its airport: the aircraft
            // is handed back to the radar once it has flown clear of it.
            startFollow(tapped);
            g_follow.zoomIsItsOwn = true;
            radarZoomSetFollow(g_follow.hex, g_follow.callsign, false);
            drawZoom(true);
        } else if (found) {
            openDetail(tapped, Screen::ZOOM);
        } else {
            drawZoom(true);
        }
        break;
    }
    case ZoomAction::REDRAW:
        shareZoomView();
        drawZoom(true);
        break;
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
    saveAlerts(g_alertMask, g_watchlist, g_autoRecord, g_autoFollow);
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
        g_autoFollow = alertsScreenAutoFollow();
        g_watchlist = alertsScreenWatchlist();
        saveAlerts(g_alertMask, g_watchlist, g_autoRecord, g_autoFollow);
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

// FOLLOW on the detail screen: followed as a tap on it after FOLLOW would
// have - in the zoom it was opened from, if it was, and otherwise on the
// radar centred on it, as auto-follow brings it up - from its latest report
// rather than the one the details were opened on.
void followFromDetail() {
    Aircraft ac = g_detailAc;
    lastReported(ac.hex, ac);
    stopFollow();  // whoever was followed before, if anyone
    g_follow.picking = false;
    if (g_detailReturnTo == Screen::ZOOM && g_zoomUp) {
        startFollow(ac);
        g_follow.zoomIsItsOwn = true;
        radarZoomSetFollow(g_follow.hex, g_follow.callsign, false);
        g_screen = Screen::ZOOM;
        drawZoom(true);
        return;
    }
    if (g_zoomUp) {
        endZoom();  // a zoom's poll is around its airport, not the aircraft
    }
    startFollow(ac);
    g_screen = Screen::RADAR;
    drawRadar(true);
}

void handleDetailTouch(int x, int y) {
    DetailAction action = detailScreenHandleTouch(x, y);
    if (action == DetailAction::NONE) {
        return;
    }
    if (action == DetailAction::FOLLOW) {
        followFromDetail();
        return;
    }
    if (action == DetailAction::UNFOLLOW) {
        stopFollow();  // and then back, as Back would go
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
        alertsScreenSet(g_alertMask, g_autoRecord, g_autoFollow, g_watchlist);
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
    g_radarRangeNm = s.radarRangeNm;
    g_alertMask = s.alertMask;
    g_watchlist = s.watchlist;
    g_autoRecord = s.autoRecord;
    g_autoFollow = s.autoFollow;
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

namespace {

void reportPerf() {
    static uint32_t lastMs = 0;
    if (millis() - lastMs < 60000) {
        return;
    }
    lastMs = millis();
    Serial.printf("[perf] radar %s, zoom %s (plots: %s), tracks %s (%u), loop max %ums; internal RAM %uKB free, "
                  "%uKB lowest, %uKB largest block\n",
                  g_perfRadar.text().c_str(), g_perfZoom.text().c_str(), radarTakeTimings().c_str(),
                  g_perfTracks.text().c_str(),
                  (unsigned)g_tracks.size(), (unsigned)(g_perfLoop.maxUs / 1000),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
    g_perfRadar = g_perfZoom = g_perfTracks = g_perfLoop = PerfStat();
}

}  // namespace

void loop() {
    uint32_t loopStart = micros();
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
    } else if (M5.Touch.getCount() == 0) {
        g_touchSpent = false;
    } else {
        auto t = M5.Touch.getDetail(0);
        // The radar and the zoom act on a touch as it lands. Elsewhere a tap
        // is a click, which comes as it lifts - and only if it lifted within
        // half a second of landing and less than 8px from where it landed;
        // otherwise it was a hold or a flick, and nothing. With a finger on
        // a glass panel, enough taps strayed past one limit or the other
        // that the radar's buttons felt laggy, or missed taps altogether.
        // Nothing on either screen drags or holds, so nothing is lost by it.
        bool onLanding = (g_screen == Screen::RADAR || g_screen == Screen::ZOOM);
        if (g_touchSpent) {
            if (!t.isPressed()) {
                g_touchSpent = false;
            }
        } else if (onLanding) {
            if (t.wasPressed()) {
                g_touchSpent = true;
                if (!handleBannerTouch(t.x, t.y)) {
                    if (g_screen == Screen::RADAR) {
                        handleRadarTouch(t.x, t.y);
                    } else {
                        handleZoomTouch(t.x, t.y);
                    }
                }
            }
        } else if (t.wasHold() && !bannerOver(t.y)) {
            if (g_screen == Screen::MAIN) {
                handleMainHold(t.x, t.y);
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

    if (g_screen == Screen::RADAR || g_screen == Screen::ZOOM) {
        updateRecButton(true);
    }
    if (g_screen == Screen::PLAYBACK) {
        playbackScreenTick();
    }
    if (g_screen == Screen::SHARE) {
        shareScreenTick();
    }

    // When each poll's list arrived, to move its contacts on from between
    // polls. Timed here rather than as each is drawn, so a view opened since
    // still knows how old what it is drawing is.
    static uint32_t seenLatestSeq = 0, seenZoomSeq = 0;
    std::vector<Aircraft> arrivedMain, arrivedZoom;
    String mainProvider, zoomProvider;
    bool mainArrived = false, zoomArrived = false;
    if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
        if (g_latestSeq != seenLatestSeq) {
            seenLatestSeq = g_latestSeq;
            g_latestDataMs = millis();
            arrivedMain = g_latestAircraft;
            mainProvider = g_activeProvider;
            mainArrived = true;
        }
        if (g_zoomSeq != seenZoomSeq) {
            seenZoomSeq = g_zoomSeq;
            g_zoomDataMs = millis();
            arrivedZoom = g_zoomAircraft;
            zoomProvider = g_zoomProvider;
            zoomArrived = true;
        }
        xSemaphoreGive(g_dataMutex);
    }
    if (mainArrived || zoomArrived) {
        uint32_t t0 = micros();
        if (mainArrived) {
            noteTracks(arrivedMain, false, mainProvider);
        }
        if (zoomArrived) {
            noteTracks(arrivedZoom, true, zoomProvider);
        }
        g_perfTracks.add(micros() - t0);
    }

    tickFollow();

    if (g_screen == Screen::RADAR) {
        // Following, the plot is drawn from the extra poll's list rather
        // than the main poll's - but a flash step still asks for a redraw
        // through g_dataReady.
        bool fresh = false;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            fresh = g_dataReady || (g_follow.active && g_zoomReady);
            g_dataReady = false;
            if (g_follow.active) {
                g_zoomReady = false;
            }
            xSemaphoreGive(g_dataMutex);
        }
        uint32_t dataMs = g_follow.active ? g_zoomDataMs : g_latestDataMs;
        if (fresh) {
            drawRadar(false);
        } else if (millis() - g_radarDrawnMs >= tweenMs(g_pollIntervalS) &&
                   millis() - dataMs < 2UL * g_pollIntervalS * 1000) {
            drawRadar(false);  // a frame between polls
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
        } else if (millis() - g_zoomDrawnMs >= tweenMs(ZOOM_POLL_INTERVAL_S) &&
                   millis() - g_zoomDataMs < 2UL * ZOOM_POLL_INTERVAL_S * 1000) {
            drawZoom(false);  // a frame between polls
        }
    }

    // While the followed aircraft is lost, its estimate moves on between
    // polls: redrawn every second, so it is seen to roll out rather than
    // jump between polls. And in a zoom centred on it, which moves with it,
    // whether lost or not: the runways would otherwise jump past under it.
    // Timed from the last draw of either kind, so a poll drawn this pass
    // isn't drawn again straight after.
    if (g_follow.active && (g_follow.lost || (g_screen == Screen::ZOOM && radarZoomOnFollowed()))) {
        if (g_screen == Screen::ZOOM && millis() - g_zoomDrawnMs >= FOLLOW_ESTIMATE_DRAW_MS) {
            drawZoom(false);
        } else if (g_screen == Screen::RADAR && millis() - g_radarDrawnMs >= FOLLOW_ESTIMATE_DRAW_MS) {
            drawRadar(false);
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

    g_perfLoop.add(micros() - loopStart);
    reportPerf();
    delay(10);
}
