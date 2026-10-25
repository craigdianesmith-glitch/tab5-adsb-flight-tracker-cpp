#include <M5Unified.h>
#include <SD.h>
#include <WiFi.h>
#include <esp32-hal-hosted.h>
#include <esp_heap_caps.h>
#include <mbedtls/platform.h>
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
#include "display_page.h"
#include "route.h"
#include "follow.h"
#include "geoid.h"
#include "info_screen.h"
#include "location_screen.h"
#include "playback_screen.h"
#include "psram_alloc.h"
#include "radar_screen.h"
#include "recorder.h"
#include "recordings_screen.h"
#include "runways.h"
#include "screen.h"
#ifdef RELEASE_BUILD
// A released image carries no network of its own: see platformio.ini.
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#else
#include "secrets.h"
#endif
#include "coast.h"
#include "filter.h"
#include "filter_screen.h"
#include "settings.h"
#include "settings_screen.h"
#include "share_screen.h"
#include "sound.h"
#include "splash.h"
#include "subject3d.h"
#include "telemetry.h"
#include "view3d.h"
#include "watchlist_screen.h"
#include "airfield_pack.h"
#include "ground_pack.h"
#include "bench3d.h"
#include "bench2d.h"
#include "wifi_screen.h"

// The loop's stack, 10KB rather than the framework's 8: at its deepest it had
// 1.6KB of the 8 to spare, little margin for a path a soak doesn't take.
SET_LOOP_TASK_STACK_SIZE(10 * 1024);

namespace {

enum class Screen { MAIN, LOCATION, DETAIL, SETTINGS, WIFI, RADAR, ZOOM, ALERTS, WATCHLIST, RECORDINGS, PLAYBACK, SHARE,
                    INFO, DISPLAY_PAGE, FILTER, VIEW3D };
Screen g_screen = Screen::MAIN;
// The detail screen is reachable from the table and from the radar, and Back
// should land wherever you came from.
Screen g_detailReturnTo = Screen::MAIN;
Aircraft g_detailAc;  // the aircraft the detail screen shows, for its FOLLOW
// The WiFi screen is normally reached from settings, but a device with no
// credentials opens it straight from boot - where Back belongs on the table.
Screen g_wifiReturnTo = Screen::SETTINGS;
// The 3D view of the flight followed - see view3d.h - opened from the radar
// or the zoom, and Back to it; and the traffic it shows, the latest list
// fetched around the aircraft and when it came. Only touched by loop().
Screen g_view3dReturnTo = Screen::RADAR;
std::vector<Aircraft> g_view3dTraffic;
uint32_t g_view3dTrafficMs = 0;
// Or the 3D view of a replay - see view3d::setReplay() - opened from the
// replay screen, or an aircraft's details there, and Back to the replay: of
// the aircraft g_replayHex, or of whichever the recording follows as it
// plays, `tracking`, where opened by the replay's own 3D. And which of its
// polls the traffic is from, for the next to be taken when it is reached.
bool g_view3dReplay = false;
bool g_replayTracking = false;
String g_replayHex;
size_t g_replayPollIdx = SIZE_MAX;

SemaphoreHandle_t g_dataMutex;
std::vector<Aircraft> g_latestAircraft;
std::vector<uint8_t> g_latestIsNew;
bool g_dataReady = false;
bool g_newFlightPending = false;  // set by pollTask, consumed by loop() to beep
uint32_t g_latestSeq = 0;         // counts the main poll's lists, for loop() to time their arrival
uint32_t g_latestDataMs = 0;      // only touched by loop(): when the latest of them arrived

double g_lat, g_lon;
String g_label;
int g_radiusNm = DEFAULT_RADIUS_NM;
TrafficFilterSet g_pollFilter;  // the filter, for the poll task's own use - see setFilterInForce()
bool g_showRefresh = true;
bool g_navaids = true;  // marked on the radar - see radarScreenSetNavaids()
bool g_coast = true;    // the map under the radar - see radarScreenSetCoast()
bool g_rings = false;   // and its range rings - see radarScreenSetRings()
int g_pollIntervalS = DEFAULT_POLL_INTERVAL_S;
TaskHandle_t g_pollTask = nullptr;  // for its stack's high-water mark, in the [perf] line
bool g_muted = false;
AdsbSource g_source = DEFAULT_ADSB_SOURCE;
RadarCentre g_radarCentre = DEFAULT_RADAR_CENTRE;  // only touched by loop()
bool g_radarAirports = false;                       // likewise
// The range ZOOM IN and OUT have chosen for the radar: one of
// RADAR_RANGE_STEPS_NM, or 0 for the whole poll radius. Written by loop()
// under the mutex, for the poll task to record it.
int g_radarRangeNm = 0;
// Zoomed out past the poll radius's ceiling, the radar is the map alone, at
// one of MAP_RANGE_STEPS_NM; 0 for the traffic's ranges. Not kept across a
// reboot, which comes back to those. Only touched by loop().
int g_mapRangeNm = 0;
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
    // Aimed where the radar has been dragged to - free look, see tickLook() -
    // rather than at either: at the main poll's interval too, of the traffic
    // the table is filtered to, as the radar it is drawn on would have, and
    // left out of a recording, which goes on being of home.
    bool look = false;
    String code;       // the zoom's airport, when it isn't following
    String followHex;  // the aircraft being followed, in either view; empty for none
    float viewNm = 0;  // the zoom's range as ZOOM IN has it, for the recording; 0 for its framing
    // Following, the radar's range around the aircraft, and each change of it
    // since the last poll recorded - for a replay to change range when the
    // live radar did: see recorder::RangeMark.
    int followNm = 0;
    std::vector<recorder::RangeMark> rangeMarks;
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

// A change of range, and when, for the recording - see recorder::RangeMark:
// a handful at most between two polls, unless the fetches have stopped
// coming. With the mutex held.
void noteRangeMark(float nm, bool zoom) {
    if (!recorder::active()) {
        return;
    }
    if (g_zoomPoll.rangeMarks.size() >= 16) {
        g_zoomPoll.rangeMarks.erase(g_zoomPoll.rangeMarks.begin());
    }
    g_zoomPoll.rangeMarks.push_back({millis(), nm, zoom});
}

// The radar's view when it is an airport zoom: the airport, and the range
// ZOOM IN and OUT have it at - for the table's radar icon to come back to as
// it was left, and saved, so after a reboot too. Empty while the radar is the
// radar itself. A zoom following hands an aircraft to isn't kept: following
// isn't either. Only touched by loop().
String g_returnZoomCode;
float g_returnZoomViewNm = 0;
// An aircraft's route airport, tapped on its details, opens that airport's
// zoom; the zoom's Back comes back to the details, and leaves the zoom the
// radar comes back to as it was. Only touched by loop().
struct RouteJump {
    bool active = false;
    Aircraft ac;               // the details it was tapped on
    Screen returnTo;           // and where their Back goes
    String priorZoom;          // the zoom they were opened from, if they were
    float priorViewNm = 0;
    String savedCode;          // g_returnZoomCode, and its range, before
    float savedViewNm = 0;
};
RouteJump g_jump;

void rememberZoom(const String &code, float viewNm) {
    if (code == g_returnZoomCode && fabsf(viewNm - g_returnZoomViewNm) < 0.01f) {
        return;  // as saved already: no write to flash
    }
    g_returnZoomCode = code;
    g_returnZoomViewNm = viewNm;
    saveRadarZoom(code, viewNm);
}

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
    // Seen on the ground and then in the air: a departure - which auto-follow
    // follows until it leaves the radius, see tickFollow().
    bool seenGround = false;
    bool departed = false;
    String originCode;  // the airport it departed from, marked on the radar as it climbs away
    bool recording = false;  // it started the recording running now
    // Started by an alert rather than by hand: with nobody there to end it,
    // it ends itself - see tickFollow(). One started by hand runs, and its
    // recording with it, until the user ends it.
    bool automatic = false;
    bool inZoom = false;     // has been on the zoom's plot since it opened
    // The zoom up is one following handed it to, or it was picked in - not
    // one tapped open on some other airport, whose poll doesn't reach it.
    bool zoomIsItsOwn = false;
    float zoomDistNm = -1;   // how far from the zoom's middle it was at the last poll, -1 for not yet
    String skipZoom;         // Left by Back while following: not handed back to
    std::vector<FollowSample> heights;  // each report, for the telemetry's chart
    int rangeNm = FOLLOW_RANGE_NM;      // the radar's, closing in as it comes down
    // ZOOM IN or OUT while following: the range picked, held - auto zoom
    // standing aside - until the next landing or take-off. 0 for none.
    int manualNm = 0;
    // Coming down to an airfield, which the range frames, and how many polls
    // running it hasn't looked to be: a level-off on the way down, or a poll
    // that misses it, doesn't end it.
    bool approach = false;
    int approachMisses = 0;
    int autoVote = 0;  // auto zoom's wish at the last poll - out 1, in -1 - a step in waiting for a second
    // In its airport's zoom, a range picked there with ZOOM IN or OUT, held
    // until the next touchdown or lift-off - and whether it was on the ground
    // when last drawn, to tell when that is.
    bool zoomManual = false;
    bool zoomGround = false;
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
// And where else the 2D screens' time goes: a replay's frames, the table's
// renders and its shading lapsing, the full draws - and whether the longest
// pass of loop() held one.
PerfStat g_perfReplay, g_perfTable, g_perfLapse, g_perfFull;
bool g_passFull = false, g_longestPassFull = false;
// drawRadar's and drawZoom's preparation before the plot, a step at a time:
// the list copied under the lock, the lost filled in, everything moved on to
// now, and the filter - the radar's alone - in us summed over the frames.
struct PrepStat {
    uint32_t count = 0;
    uint64_t copyUs = 0, lostUs = 0, nowUs = 0, filterUs = 0;
    void add(uint32_t copy, uint32_t lost, uint32_t now, uint32_t filter) {
        count++;
        copyUs += copy;
        lostUs += lost;
        nowUs += now;
        filterUs += filter;
    }
    String text(bool filter) const {
        if (count == 0) {
            return String("-");
        }
        String s = "copy " + String((uint32_t)(copyUs / count)) + ", lost " + String((uint32_t)(lostUs / count)) +
                   ", now " + String((uint32_t)(nowUs / count));
        return filter ? s + ", filter " + String((uint32_t)(filterUs / count)) : s;
    }
};
PrepStat g_perfPrepRadar, g_perfPrepZoom;

// A touch the radar or the zoom has already acted on, as it landed. The rest
// of it - its release, or its turning into a hold - is ignored, so a tap that
// opens another screen doesn't go on to press whatever is under it there.
bool g_touchSpent = false;

// A freeze made a crash, for the core dump to say where it was stuck. The
// loop counts its passes; a task of its own looks every second, and once the
// count has stood still for a minute and a half - which nothing the loop
// does takes, but a bench run, which says so - stops the board where it
// stands. The core dump keeps where every task was, and the board starts
// again rather than staying frozen.
// Fifteen seconds in, it says where the loop had got to, what the 3D view's
// drawing core is doing, and who has the screen's turn at the PPA.
volatile uint32_t g_loopPasses = 0;
volatile bool g_loopBusyLong = false;
const char *volatile g_loopAt = "";
constexpr uint32_t HANG_SAY_MS = 15000, HANG_MS = 90000;

void hangWatch(void *) {
    uint32_t seen = g_loopPasses, sinceMs = millis();
    bool said = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_loopPasses != seen || g_loopBusyLong) {
            seen = g_loopPasses;
            sinceMs = millis();
            said = false;
            continue;
        }
        uint32_t stuck = millis() - sinceMs;
        if (stuck > HANG_SAY_MS && !said) {
            said = true;
            const char *holder = screen::pushTurnHolder();
            Serial.printf("[hang] the loop stuck %us at %s; 3D: %s; the PPA's turn held by %s\n",
                          (unsigned)(stuck / 1000), (const char *)g_loopAt, view3d::busy().c_str(),
                          *holder ? holder : "nobody");
        }
        if (stuck > HANG_MS) {
            Serial.println("[hang] stopping here for the core dump");
            abort();
        }
    }
}
// Unless it landed on the plot with nothing under it: then the rest of it is
// a drag of the plot - see radarDragTo() - for as long as the finger is down.
bool g_dragging = false;

// Free look: the radar dragged off its own centre, to anywhere. The traffic
// there is fetched by the extra poll - the zoom's, and following's - aimed
// at where the radar has been left, for as long as neither of those wants
// it: `aimed`, and at what. Only touched by loop().
struct Look {
    bool aimed = false;
    double lat = 0, lon = 0;
    int radiusNm = 0;
};
Look g_look;

// Whether the radar is drawn from the extra poll's list rather than the main
// poll's: following an aircraft, or looking somewhere of its own.
bool radarOnExtraList() { return g_follow.active || g_look.aimed; }

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
bool g_followIn3d = false;  // an auto-follow opens the 3D view, rather than the radar
// The same, as the alerts screen edits them. Only touched by loop().
uint8_t g_alertMask = ALERT_ALL;
String g_watchlist;
// The display page's - see display_page.h - and whether the screen is dimmed
// now for want of a touch. Only touched by loop().
uint8_t g_brightnessPct = 100, g_dimMinutes = 0;
// The clock's zone as picked on the display page, or CLOCK_ZONE_AUTO for
// home's - see clockZoneFor().
uint8_t g_clockSetting = CLOCK_ZONE_AUTO;
// Following, the radar's range sized to the sky round the aircraft - see
// followRangeNow().
bool g_autoZoom = false;
bool g_replayTracks = true;  // see playbackScreenSetTracks()

// Whether auto zoom is choosing the range around the aircraft followed now:
// switched on, and neither held at a range picked by hand nor closing in on a
// landing.
bool followAutoNow() { return g_follow.active && g_autoZoom && !g_follow.manualNm && !g_follow.approach; }
ClockZone clockZoneShown() {
    return g_clockSetting == CLOCK_ZONE_AUTO ? clockZoneFor(g_lat, g_lon) : (ClockZone)g_clockSetting;
}
bool g_dimmed = false;
// A touch that woke the dimmed screen, used up on that: nothing under the
// finger acts until it lifts.
bool g_wakeTouch = false;
uint32_t g_lastTouchMs = 0;
void wakeScreen();
// Read aloud, and the voice for everywhere without one of its own - see
// sound.h. Only touched by loop().
bool g_readAloud = false;
Accent g_elsewhereAccent = DEFAULT_ELSEWHERE_ACCENT;

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

#ifdef BENCH3D
// --- made-up flights, for the soak test ---------------------------------------------

// A flight that isn't there, put into every poll's list as though the feed had
// it - where its pattern has it by the time of the poll, said the way the feed
// says it - for the 3D view to be tried for an hour on what the real sky won't
// lay on to order: a fighter circling a military field, a Chinook, a police
// helicopter hovering, an airliner landing and parking, one taking off, a
// helicopter setting down at an airfield and lifting off again, a fast jet
// running in to break into the circuit, a light aircraft joining overhead.
// See tools/soak3d.py and the "##k" command.
enum class Pattern : uint8_t { CIRCLE, HOVER, LAND, DEPART, VISIT, CIRCUIT };
struct MadeUp {
    bool active = false;
    Pattern pattern = Pattern::CIRCLE;
    const char *hex = "", *callsign = "", *type = "", *category = "";
    bool military = false;
    double lat = 0, lon = 0;  // the middle of a circle or a hover; a runway's threshold; a visit's spot
    float track = 0;          // down the runway from its threshold, or in to the spot, degrees
    float radiusM = 0, speedKt = 0;
    int altFt = 0, fieldFt = 0;
    uint32_t startMs = 0;
    float quietAfterS = 0;  // no more reports after this long - its transponder off on its stand; 0 never
    // A visit's: its pressure altitude this far over its height, by the
    // day's pressure - or by the day's pressure as the aircraft about have
    // it, `dayPressure`, and this far more; its GNSS height given too, as
    // the feed's alt_geom - above the ellipsoid - or not; and whether its
    // transponder says when it is on the ground, as many helicopters' never
    // do.
    int baroOverFt = 0;
    bool dayPressure = false, geom = false, saysGround = false;
    // A circuit's - see madeUpAt(): in along the runway at speedKt and altFt
    // over the field from `runInM` before it; round onto the downwind leg at
    // `circuitKt` and `circuitFt`; round again onto final `finalM` out, at
    // `finalKt`; the turns `radiusM` across. The runway `runwayM` long.
    float runInM = 0, runwayM = 0, circuitKt = 0, finalKt = 0, finalM = 0;
    int circuitFt = 0;
};
MadeUp g_madeUp;  // under g_dataMutex

// A visit's minute on the ground, its rotors turning.
constexpr float VISIT_DOWN_S = 60;

// Where `m` is at `ms`, as the feed would have it.
Aircraft madeUpAt(const MadeUp &m, uint32_t ms) {
    float t = (int32_t)(ms - m.startMs) / 1000.0f;
    Aircraft a;
    a.hex = m.hex;
    a.callsign = m.callsign;
    a.type = m.type;
    a.category = m.category;
    a.military = m.military;
    a.hasPos = a.hasTrack = a.hasVertRate = true;
    double perLat = 111320.0, perLon = 111320.0 * cos(m.lat * M_PI / 180);
    double east = 0, north = 0;
    float gs = m.speedKt, track = 0, alt = (float)m.altFt, vs = 0;
    bool ground = false;
    // Down the runway, `s` metres from its threshold and `side` to its right.
    float ux = sinf(m.track * (float)M_PI / 180), uy = cosf(m.track * (float)M_PI / 180);
    auto onRunway = [&](float s, float side) {
        east = ux * s + uy * side;
        north = uy * s - ux * side;
    };
    switch (m.pattern) {
    case Pattern::CIRCLE: {
        // Anticlockwise, from east of its middle.
        float th = m.speedKt * 0.5144f / m.radiusM * t;
        east = m.radiusM * cosf(th);
        north = m.radiusM * sinf(th);
        track = fmodf(720 - th * 180 / (float)M_PI, 360);
        break;
    }
    case Pattern::HOVER: {
        // Drifting a few metres about, and turning slowly on the spot now and
        // then - the track the feed gives for it, from the drift, all over
        // the place, as a real one's is.
        east = 15 * sinf(t / 40) + 4 * sinf(t / 7);
        north = 10 * sinf(t / 55) + 4 * cosf(t / 9);
        float ve = 15.0f / 40 * cosf(t / 40) + 4.0f / 7 * cosf(t / 7);
        float vn = 10.0f / 55 * cosf(t / 55) - 4.0f / 9 * sinf(t / 9);
        gs = sqrtf(ve * ve + vn * vn) / 0.5144f;
        track = fmodf(atan2f(ve, vn) * 180 / (float)M_PI + 360, 360);
        alt += 25 * sinf(t / 23);
        vs = 25.0f / 23 * cosf(t / 23) * 60;
        break;
    }
    case Pattern::LAND: {
        // From four miles out on a three-degree glide at 140kt; down 300m in,
        // slowing at 2.5m/s2 to 15kt; then stopped where it is.
        const float V = 72, S0 = -7400, TOUCH = 300, DECEL = 2.5f, TAXI = 8;
        float tTouch = (TOUCH - S0) / V, tSlow = tTouch + (V - TAXI) / DECEL;
        float sSlow = TOUCH + (V * V - TAXI * TAXI) / (2 * DECEL);
        track = m.track;
        if (t < tTouch) {
            float s = S0 + V * t;
            onRunway(s, 0);
            alt = m.fieldFt + (s < 0 ? 15.2f - s * 0.0524f : 15.2f * (1 - s / TOUCH)) * 3.281f;
            vs = s < 0 ? -V * 0.0524f * 196.85f : -15.2f / (TOUCH / V) * 196.85f;
            gs = V / 0.5144f;
        } else if (t < tSlow) {
            float dt = t - tTouch;
            onRunway(TOUCH + V * dt - DECEL * dt * dt / 2, 0);
            gs = (V - DECEL * dt) / 0.5144f;
            ground = true;
        } else {
            // Stopped on the runway at its slowest - its reports going quiet
            // soon after, for the 3D view's parking to take it in.
            onRunway(sSlow, 0);
            gs = 0;
            ground = true;
        }
        break;
    }
    case Pattern::DEPART: {
        // Lined up 60m in for 20 seconds; rolling at 2m/s2 to 155kt and off
        // at 40 seconds; climbing at 2,000ft a minute, speeding up to 230kt.
        const float HOLD = 20, ACC = 2, VR = 80, S1 = 60;
        float tLift = HOLD + VR / ACC;
        track = m.track;
        if (t < HOLD) {
            onRunway(S1, 0);
            gs = 0;
            ground = true;
        } else if (t < tLift) {
            float dt = t - HOLD;
            onRunway(S1 + ACC * dt * dt / 2, 0);
            gs = ACC * dt / 0.5144f;
            ground = true;
        } else {
            float dt = t - tLift, v = std::min(118.0f, VR + dt * 0.6f);
            onRunway(S1 + VR * VR / (2 * ACC) + (VR + v) / 2 * dt, 0);
            gs = v / 0.5144f;
            alt = m.fieldFt + 2000 * dt / 60;
            vs = 2000;
        }
        break;
    }
    case Pattern::CIRCUIT: {
        // In along the runway and over it; round to the left onto the
        // downwind leg, coming down to circuit height; back past the
        // threshold and round again onto final, coming down to a
        // three-degree glide; down it to land 300m in, and slowing at 2.5m/s2
        // to a stop - as a fast jet runs in and breaks into the circuit, or a
        // light aircraft joins overhead.
        const float TOUCH = 300, DECEL = 2.5f, PI_F = (float)M_PI;
        float vIn = m.speedKt * 0.5144f, vCircuit = m.circuitKt * 0.5144f, vFinal = m.finalKt * 0.5144f;
        float r = m.radiusM, len = m.runwayM;
        float hIn = m.altFt / 3.281f, hCircuit = m.circuitFt / 3.281f, hFinal = (m.finalM + TOUCH) * 0.0524f;
        float t1 = (m.runInM + len) / vIn;
        float t2 = t1 + PI_F * r / ((vIn + vCircuit) / 2);
        float t3 = t2 + (len + m.finalM) / vCircuit;
        float t4 = t3 + PI_F * r / ((vCircuit + vFinal) / 2);
        float t5 = t4 + (m.finalM + TOUCH) / vFinal;
        float h = 0;
        if (t < t1) {
            onRunway(-m.runInM + vIn * t, 0);
            track = m.track;
            gs = vIn / 0.5144f;
            h = hIn;
        } else if (t < t2) {
            float u = (t - t1) / (t2 - t1), phi = PI_F * u;
            onRunway(len + r * sinf(phi), -r + r * cosf(phi));
            track = fmodf(m.track - phi * 180 / PI_F + 360, 360);
            gs = (vIn + (vCircuit - vIn) * u) / 0.5144f;
            h = hIn + (hCircuit - hIn) * u;
            vs = (hCircuit - hIn) / (t2 - t1) * 196.85f;
        } else if (t < t3) {
            onRunway(len - vCircuit * (t - t2), -2 * r);
            track = fmodf(m.track + 180, 360);
            gs = vCircuit / 0.5144f;
            h = hCircuit;
        } else if (t < t4) {
            float u = (t - t3) / (t4 - t3), phi = PI_F * u;
            onRunway(-m.finalM - r * sinf(phi), -r - r * cosf(phi));
            track = fmodf(m.track + 180 - phi * 180 / PI_F + 360, 360);
            gs = (vCircuit + (vFinal - vCircuit) * u) / 0.5144f;
            h = hCircuit + (hFinal - hCircuit) * u;
            vs = (hFinal - hCircuit) / (t4 - t3) * 196.85f;
        } else if (t < t5) {
            float s = -m.finalM + vFinal * (t - t4);
            onRunway(s, 0);
            track = m.track;
            gs = vFinal / 0.5144f;
            h = (TOUCH - s) * 0.0524f;
            vs = -vFinal * 0.0524f * 196.85f;
        } else {
            float dt = std::min(t - t5, (vFinal - 8) / DECEL);
            onRunway(TOUCH + vFinal * dt - DECEL * dt * dt / 2, 0);
            track = m.track;
            gs = t - t5 < (vFinal - 8) / DECEL ? (vFinal - DECEL * dt) / 0.5144f : 0;
            ground = true;
        }
        alt = m.fieldFt + h * 3.281f;
        break;
    }
    case Pattern::VISIT: {
        // A helicopter in from 2.6km out at 90kt and 1,000ft, slowing evenly
        // all the way to a hover 40ft over its spot, and down onto it over
        // twelve seconds; there a minute; then up into a hover at 10ft and
        // away, off to the right of the way it came, climbing at 900ft a
        // minute and speeding up to 110kt.
        const float V = 46, D = 2600, A = V * V / (2 * D), HOVER = 12, TOP = 305, VA = 57, ACC = 1.2f;
        const float tIn = V / A, tDown = tIn + 12, tUp = tDown + VISIT_DOWN_S, tOff = tUp + 15;
        float h = 0;
        track = m.track;
        gs = 0;
        if (t < tIn) {
            float left = D - (V * t - A * t * t / 2), v = V - A * t;
            onRunway(-left, 0);
            h = HOVER + (TOP - HOVER) * left / D;
            gs = v / 0.5144f;
            vs = -(TOP - HOVER) / D * v * 196.85f;
        } else if (t < tDown) {
            h = HOVER * (1 - (t - tIn) / 12);
            vs = -HOVER / 12 * 196.85f;
        } else if (t < tUp) {
            ground = m.saysGround;
        } else if (t < tOff) {
            h = std::min(3.0f, (t - tUp) * 0.6f);
            vs = t - tUp < 5 ? 0.6f * 196.85f : 0;
        } else {
            float dt = t - tOff, v = std::min(VA, ACC * dt);
            float s = v < VA ? ACC * dt * dt / 2 : VA * VA / (2 * ACC) + VA * (dt - VA / ACC);
            track = fmodf(m.track + 60, 360);
            east = sinf(track * (float)M_PI / 180) * s;
            north = cosf(track * (float)M_PI / 180) * s;
            h = std::min(460.0f, 3 + 4.57f * dt);
            vs = h < 460 ? 900 : 0;
            gs = v / 0.5144f;
        }
        alt = m.fieldFt + h * 3.281f;
        break;
    }
    }
    a.lat = (float)(m.lat + north / perLat);
    a.lon = (float)(m.lon + east / perLon);
    a.track = track;
    a.vertRate = vs;
    int32_t over = m.baroOverFt, pressure;
    if (m.dayPressure && dayPressureFt(pressure)) {
        over -= pressure;
    }
    a.altStr = ground ? String("GND") : String((int)lroundf(alt + over));
    if (m.geom && !ground) {
        a.hasAltGeom = true;
        a.altGeom = (int)lroundf(alt + geoidM(a.lat, a.lon) * 3.2808f);
    }
    a.speedStr = String((int)lroundf(gs));
    a.status = ground ? (gs > 2 ? "TAXI" : "GROUND") : vs > 300 ? "CLIMB" : vs < -300 ? "DESCEND" : "LEVEL";
    return a;
}

// A made-up landing's reports lost on its rollout, at 35 knots - as a ground
// station's cover often ends - for the 3D view to take it off the runway and
// in to a stand by itself.
constexpr float LOST_ON_ROLLOUT_S = (300 + 7400) / 72.0f + (72 - 18) / 2.5f;

// The made-up flight into a poll's list, if one is up and within the poll's
// reach of (lat, lon) - in its place by distance, as the feed sorts them.
// Made-up traffic about the airfield the 3D view was last at, while a
// made-up flight is up and "##r1" has asked for it: an aircraft parked on
// every third stand, and others taxiing along its taxiways - all as the
// polls would bring them, for the 3D view's and the radar's traffic to be
// tried with the network down.
bool g_benchTraffic = false;
void benchTrafficInto(std::vector<Aircraft> &list, double lat, double lon, float radiusNm, uint32_t ms) {
    static const char *const TYPES[] = {"A320", "B738", "A21N", "B77W", "A388", "E190", "B789", "A333", "CRJ9", "AT76"};
    static double standLat[400], standLon[400], wayLat[16], wayLon[16];
    static float wayTrack[16];
    int stands = view3d::benchStands(standLat, standLon, 400);
    int ways = view3d::benchWays(wayLat, wayLon, wayTrack, 16);
    auto add = [&](int n, double aLat, double aLon, float track, int kt, bool moving) {
        float d = haversineNm(lat, lon, aLat, aLon);
        if (d > radiusNm) {
            return;
        }
        Aircraft a;
        char hex[8], callsign[10];
        snprintf(hex, sizeof hex, "bb%04x", n);
        snprintf(callsign, sizeof callsign, "TST%03d", n);
        a.hex = hex;
        a.callsign = callsign;
        a.type = TYPES[n % 10];
        a.category = "A3";
        a.hasPos = a.hasTrack = true;
        a.lat = (float)aLat;
        a.lon = (float)aLon;
        a.track = track;
        a.altStr = "GND";
        a.speedStr = String(kt);
        a.status = moving ? "TAXI" : "GROUND";
        a.hasDist = true;
        a.distNm = d;
        list.push_back(a);
    };
    int n = 0;
    for (int i = 0; i < stands && n < 120; i += 3) {
        add(n++, standLat[i], standLon[i], (float)((i * 37) % 360), 0, false);
    }
    // Along each taxiway from its point, at 12kt for a minute, then back to
    // the start: far enough to be seen going, not so far as to leave it.
    float t = (ms / 1000) % 60;
    for (int i = 0; i < ways; i++) {
        double metres = 12 * 0.5144 * t, rad = wayTrack[i] * M_PI / 180;
        double aLat = wayLat[i] + metres * cos(rad) / 111320.0,
               aLon = wayLon[i] + metres * sin(rad) / (111320.0 * cos(wayLat[i] * M_PI / 180));
        add(200 + i, aLat, aLon, wayTrack[i], 12, true);
    }
    // A fire engine going along the first taxiway, as the feed has one: for
    // the 3D view to be seen to leave it out.
    auto notAircraft = [&](int n, double aLat, double aLon, const char *type, const char *category,
                           const char *callsign, int kt, float track) {
        float d = haversineNm(lat, lon, aLat, aLon);
        if (d > radiusNm) {
            return;
        }
        Aircraft a;
        char hex[8];
        snprintf(hex, sizeof hex, "bc%04x", n);
        a.hex = hex;
        a.callsign = callsign;
        a.type = type;
        a.reg = type;
        a.category = category;
        a.hasPos = a.hasTrack = true;
        a.lat = (float)aLat;
        a.lon = (float)aLon;
        a.track = track;
        a.altStr = "GND";
        a.speedStr = String(kt);
        a.status = kt > 0 ? "TAXI" : "GROUND";
        a.hasDist = true;
        a.distNm = d;
        list.push_back(a);
    };
    if (ways > 0) {
        double metres = 15 * 0.5144 * t, rad = wayTrack[0] * M_PI / 180;
        notAircraft(3, wayLat[0] + metres * cos(rad) / 111320.0,
                    wayLon[0] + metres * sin(rad) / (111320.0 * cos(wayLat[0] * M_PI / 180)), "EMER", "C2", "FIRE1",
                    15, wayTrack[0]);
    }
}

void madeUpInto(std::vector<Aircraft> &list, double lat, double lon, float radiusNm) {
    MadeUp m;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        m = g_madeUp;
        xSemaphoreGive(g_dataMutex);
    }
    if (m.active && g_benchTraffic) {
        benchTrafficInto(list, lat, lon, radiusNm, millis());
        std::sort(list.begin(), list.end(), [](const Aircraft &a, const Aircraft &b) {
            return a.hasDist && (!b.hasDist || a.distNm < b.distNm);
        });
    }
    if (!m.active || (m.quietAfterS > 0 && (int32_t)(millis() - m.startMs) > m.quietAfterS * 1000)) {
        return;
    }
    Aircraft a = madeUpAt(m, millis());
    float d = haversineNm(lat, lon, a.lat, a.lon);
    if (d > radiusNm) {
        return;
    }
    a.hasDist = true;
    a.distNm = d;
    list.insert(std::find_if(list.begin(), list.end(), [&](const Aircraft &b) { return b.hasDist && b.distNm > d; }),
                a);
}

// The soak test's flights, by number: "##k" and the digit.
const char *const SOAK_NAMES[] = {
    "a Typhoon circling RAF Lossiemouth",       "an F-35 high over the North Sea",
    "a Hawk low over the Lake District",        "a Chinook circling RAF Odiham",
    "a police EC135 hovering over Glasgow",      "an air ambulance H145 circling Edinburgh",
    "an A400M orbiting high over Perthshire",    "an A320 landing on 27L at Heathrow, then taxiing in",
    "a 737-800 taking off from 24 at Edinburgh", "a C-17 landing at RAF Brize Norton, then taxiing in",
    "an air ambulance H145 setting down at Glasgow and off again, by its GNSS height",
    "a coastguard AW139 setting down at Edinburgh by its pressure altitude alone, saying so, then quiet",
    "an Emirates A380 landing on 27L at Heathrow, then taxiing in",
    "a British Airways 777 taking off from 27R at Heathrow",
    "a Typhoon running in at 400kt to break into the circuit at Lossiemouth, then landing on 23",
    "a Cessna joining overhead at Glasgow at 2,000ft, then landing on 23",
    "an A320 landing on 23 at Glasgow on a day of low pressure, by its pressure altitude and the day's pressure",
    "an A320 landing on 23 at Lossiemouth on a day of high pressure, its pressure altitude under nought on short final",
    "an alert for an A320 landing at Glasgow, auto-followed into 3D: no banner left round the view",
    "a U-2 at 65,000ft circling over Fairford: the 3D view twice the cruise's height up, the horizon 500km off",
    "a Typhoon at 10,000ft and 480kt circling the Cheviots: the hills seen from above, its fly past fast",
    "an E190 taking off from 23 at Glasgow: its nose rotated for, its APU off as it lines up, the hills beyond",
    "a 767 at FL300 circling the Southern Uplands: its trails, and a widebody's fly past",
    "a Loganair ATR 72 landing on 23 at Glasgow, then taxiing in: a turboprop's approach and reverse",
    "a Loganair ATR 72 taking off from 23 at Glasgow: a turboprop at full power",
    "a Cessna cruising at 12,500ft round Zermatt, the Matterhorn above it: the title shot, with the Alps in the pack",
};

MadeUp circling(const char *hex, const char *callsign, const char *type, const char *category, bool military,
                double lat, double lon, float radiusM, float speedKt, int altFt) {
    MadeUp m;
    m.pattern = Pattern::CIRCLE;
    m.hex = hex;
    m.callsign = callsign;
    m.type = type;
    m.category = category;
    m.military = military;
    m.lat = lat;
    m.lon = lon;
    m.radiusM = radiusM;
    m.speedKt = speedKt;
    m.altFt = altFt;
    return m;
}

// A landing or a take-off from the end `end` of a runway at `code`: its
// threshold and its way from the runway table. False if the table hasn't it.
bool onRunwayAt(MadeUp &m, const char *code, const char *end) {
    int count = 0;
    const Runway *r = runwaysAt(code, count);
    for (int i = 0; r != nullptr && i < count; i++) {
        for (int e = 0; e < 2; e++) {
            if (strcmp(r[i].end[e].ident, end) != 0) {
                continue;
            }
            const RunwayEnd &a = r[i].end[e], &b = r[i].end[1 - e];
            double dn = (b.lat - a.lat) * 60, de = (b.lon - a.lon) * 60 * cos(a.lat * M_PI / 180);
            m.lat = a.lat;
            m.lon = a.lon;
            m.track = (float)fmod(atan2(de, dn) * 180 / M_PI + 360, 360);
            m.fieldFt = r[i].elevationFt;
            m.runwayM = (float)(sqrt(dn * dn + de * de) * 1852);
            return true;
        }
    }
    return false;
}

// A helicopter's visit to a spot `along` metres down the runway `end` at
// `code` from its threshold and `right` of it, coming in on `track`.
bool spotAt(MadeUp &m, const char *code, const char *end, float along, float right, float track) {
    if (!onRunwayAt(m, code, end)) {
        return false;
    }
    float ux = sinf(m.track * (float)M_PI / 180), uy = cosf(m.track * (float)M_PI / 180);
    m.lat += (uy * along - ux * right) / 111320.0;
    m.lon += (ux * along + uy * right) / (111320.0 * cos(m.lat * M_PI / 180));
    m.track = track;
    m.pattern = Pattern::VISIT;
    return true;
}

#endif

// One fetch of the traffic around the zoom's airport - or the aircraft being
// followed - from the poll task.
// Everything flying is kept, whichever traffic the table is filtered to - at
// an airport the airliners are the point, whatever the filter - and alerts
// are evaluated only for the colours they draw in: the main poll is what
// raises them.
void pollZoom(const ZoomPoll &zoom, AdsbSource source) {
    std::vector<Aircraft> fetched;
    const char *provider = nullptr;
    if (WiFi.status() != WL_CONNECTED ||
        !fetchAircraft(zoom.lat, zoom.lon, zoom.radiusNm, false, source, fetched, provider)) {
#ifdef BENCH3D
        // A made-up flight goes on being seen whatever the feed does - the
        // soak test is of the 3D view, not the network: the list as it
        // stands, with it where it is by now.
        MadeUp m;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            m = g_madeUp;
            fetched = g_zoomAircraft;
            xSemaphoreGive(g_dataMutex);
        }
        if (!m.active) {
            return;
        }
        fetched.erase(std::remove_if(fetched.begin(), fetched.end(), [&](const Aircraft &a) { return a.hex == m.hex; }),
                      fetched.end());
        provider = "made up";
#else
        return;  // the last list stays up, as the main poll's does
#endif
    }
#ifdef BENCH3D
    madeUpInto(fetched, zoom.lat, zoom.lon, zoom.radiusNm);
#endif
    AlertRules rules;
    TrafficFilterSet filter;
    double homeLat = 0, homeLon = 0;
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        rules = g_alertRules;
        filter = g_pollFilter;
        homeLat = g_lat;
        homeLon = g_lon;
        xSemaphoreGive(g_dataMutex);
    }
    if (fetched.size() > MAX_CONTACTS) {
        // Nearest the airport first - and on a free look, with a filter on,
        // as many again beyond them that pass it, as the main poll keeps.
        auto keepEnd = fetched.begin() + MAX_CONTACTS;
        if (zoom.look && filter.active()) {
            auto tail = keepEnd;
            keepEnd = std::stable_partition(tail, fetched.end(),
                                            [&filter](const Aircraft &a) { return filter.passes(a); });
            if (keepEnd - tail > (std::ptrdiff_t)MAX_CONTACTS) {
                keepEnd = tail + MAX_CONTACTS;
            }
        }
        fetched.erase(keepEnd, fetched.end());
    }
    for (Aircraft &a : fetched) {
        a.alert = evaluateAlert(a, rules);
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
            g_zoomPoll.rangeMarks.clear();  // recorded with this poll, if they are recorded at all
        }
        xSemaphoreGive(g_dataMutex);
    }
    // While this poll is up, a recording is made of it rather than the main
    // poll's - it is what the screens are showing - each marked with the view
    // it was for, so a replay shows it the way it was seen.
    if (current && recorder::active() && !now.look) {
        String view = now.follow ? "-" + now.followHex : now.code + "/" + now.followHex;
        recorder::addFrame(fetched, view, provider, now.follow ? (float)now.followNm : now.viewNm, &now.rangeMarks,
                           &filter);
    }
}

void pollTask(void *) {
    // When the last request of either kind went, and the last of the zoom's,
    // for spacing them - see the wait at the bottom.
    uint32_t lastRequestMs = 0, lastZoomMs = 0;
    uint32_t zoomGenServed = 0;
    for (;;) {
        double lat = 0, lon = 0;
        int radius = DEFAULT_RADIUS_NM;
        AdsbSource source = DEFAULT_ADSB_SOURCE;
        uint32_t intervalMs = (uint32_t)DEFAULT_POLL_INTERVAL_S * 1000;
        AlertRules rules;
        TrafficFilterSet filter;
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
            radius = g_radiusNm;
            source = g_source;
            intervalMs = (uint32_t)g_pollIntervalS * 1000;
            rules = g_alertRules;
            filter = g_pollFilter;
            autoRecord = g_autoRecord;
            xSemaphoreGive(g_dataMutex);
        }

        if (sceneChanged && recorder::active()) {
            Serial.println("[rec] location or range changed - stopping the recording");
            recorder::stop();  // auto-record starts afresh, with the new header, if an alert is still about
        }

        // The radius is already clamped to the ceiling the filter carries, so
        // it doubles as the poll radius. Everything is kept, military and
        // all: the filter's kinds are what the table and the radar show.
        bool linkUp = ensureWifi();
        std::vector<Aircraft> fetched;
        const char *provider = nullptr;
        bool ok = linkUp && fetchAircraft(lat, lon, radius, false, source, fetched, provider);
#ifdef BENCH3D
        if (ok) {
            madeUpInto(fetched, lat, lon, radius);
        }
#endif
        if (ok) {
            std::vector<Aircraft> aircraft;
            aircraft.reserve(fetched.size());
            for (Aircraft &a : fetched) {
                if (a.hasDist && a.distNm > radius) {
                    continue;  // the API is occasionally a little generous
                }
                a.alert = evaluateAlert(a, rules);
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
            //
            // And with a filter on, as many again of the tail as pass it: the
            // table and the radar show only those, which among the nearest
            // alone could be a handful. The nearest are kept whatever they
            // are, for the recording and the alerts, which are of everything.
            size_t maxRows = (size_t)displayMaxRows();
            if (aircraft.size() > MAX_CONTACTS) {
                auto tail = aircraft.begin() + MAX_CONTACTS;
                auto alertEnd =
                    std::stable_partition(tail, aircraft.end(), [](const Aircraft &a) { return a.alert != 0; });
                auto keepEnd = alertEnd;
                if (filter.active()) {
                    keepEnd = std::stable_partition(alertEnd, aircraft.end(),
                                                    [&filter](const Aircraft &a) { return filter.passes(a); });
                    if (keepEnd - alertEnd > (std::ptrdiff_t)MAX_CONTACTS) {
                        keepEnd = alertEnd + MAX_CONTACTS;
                    }
                    // Both in distance order already: the two put back into it.
                    std::inplace_merge(tail, alertEnd, keepEnd, [](const Aircraft &a, const Aircraft &b) {
                        return a.distNm < b.distNm;
                    });
                }
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
                extraUp = g_zoomPoll.active && !g_zoomPoll.look;  // a free look isn't recorded
                shownNm = g_radarRangeNm > 0 && g_radarRangeNm < g_radiusNm ? g_radarRangeNm : 0;
                xSemaphoreGive(g_dataMutex);
            }
            if (!extraUp) {
                recorder::addFrame(aircraft, "H", provider, (float)shownNm, nullptr, &filter);
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
                uint32_t every = zoom.follow || zoom.look ? intervalMs : ZOOM_POLL_MS;
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

// How long boot waits on the network, from when it is begun - the splash
// screen playing out in the first few seconds of it.
constexpr uint32_t WIFI_BOOT_TIMEOUT_MS = 15000;

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
    ghost.altStr = e.ground ? String("GND") : String(feedAltFt(e));
    // Not known if the feed never gave it: the estimate's nought is only for
    // want of one.
    ghost.speedStr = base.speedStr == "?" ? base.speedStr : String(e.gsKt);
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
    // What the zoom last announced it for, and when - see announceMovement().
    RunwaySim::Movement announced = RunwaySim::Movement::NONE;
    uint32_t announcedMs = 0;
};
// In the PSRAM, a node for each contact: a hundred or more of them, each with
// its last report and its runway simulation, a kilobyte between them, took
// that much of the chip's own memory.
std::map<String, Track, std::less<String>, PsramAllocator<std::pair<const String, Track>>> g_tracks;  // only touched by loop()

// What the 3D view shows - see subject3d.h and update3dSubject().
subject3d::Picker g_picker;

// Not announced for the same again within this: a landing the sim loses
// for a poll and picks up again is one landing.
constexpr uint32_t ANNOUNCE_AGAIN_MS = 5UL * 60 * 1000;

// The zoom on screen - or the 3D view, at an airfield - says each take-off
// and landing at its airport as it begins, as a controller would clear it -
// lining up, for a take-off, or established on final for a landing, as the
// runway sim has them.
//
// Only while it would still be said: a take-off on the ground, a landing in
// the air. The sim can be into a take-off by the time the zoom's list next
// has it - begun from the main poll's, ten seconds or more before - and keeps
// it up to 1500ft; cleared for take-off was being said of an aircraft already
// climbing away. One too late is taken as said, so it isn't said later.
void announceMovement(const Aircraft &a, Track &t, uint32_t now) {
    RunwaySim::Movement m = t.sim.movement();
    if (m == RunwaySim::Movement::NONE || (m == t.announced && now - t.announcedMs < ANNOUNCE_AGAIN_MS)) {
        return;
    }
    const RunwayLine &rw = t.sim.runway();
    if (!rw.ident || !rw.airport) {
        return;
    }
    // Its airport on the screen: the zoom of it, or the 3D view at it - every
    // take-off and landing there, as the zoom has them, not only the one the
    // view shows.
    bool inZoom = g_screen == Screen::ZOOM && g_zoomUp && g_zoomCode == rw.airport;
    bool in3d = g_screen == Screen::VIEW3D && !g_view3dReplay && strcmp(view3d::airport(), rw.airport) == 0;
    if (!inZoom && !in3d) {
        return;
    }
    t.announced = m;
    t.announcedMs = now;
    bool takeoff = m == RunwaySim::Movement::TAKEOFF;
    if (t.sim.at(now).ground != takeoff) {
        Serial.printf("[sound] not announcing %s, runway %s: already %s\n", a.callsign.c_str(), rw.ident,
                      takeoff ? "airborne" : "down");
        return;
    }
    soundAnnounce(a.callsign, rw.ident, takeoff, rw.airport);
}

// And each take-off and landing under way said as soon as its airport is on
// the screen - the 3D view come to it, the zoom up - not only as a list
// comes in, ten seconds or more on: one lined up as the 3D view opened was
// cleared for take-off once it was rolling at 40 knots.
void announcePending(uint32_t now) {
    static uint32_t lastMs = 0;
    if (now - lastMs < 1000 || (g_screen != Screen::ZOOM && g_screen != Screen::VIEW3D)) {
        return;
    }
    lastMs = now;
    for (auto &kv : g_tracks) {
        if (kv.second.freshMs != 0) {
            announceMovement(kv.second.last, kv.second, now);
        }
    }
}

bool lowSample(const FollowSample &s) { return s.ground || (s.hasAlt && s.altFt < ESTIMATE_MAX_FT); }

// Takes in a list that has just arrived - the main poll's, or the zoom's -
// and who answered it.
void noteTracks(const std::vector<Aircraft> &list, bool zoomList, const String &provider) {
    uint32_t now = millis();
    notePressure(list);
#ifdef BENCH3D
    // A list that takes long, said in the log with the one that took longest.
    uint32_t startUs = micros(), worstUs = 0;
    const Aircraft *worst = nullptr;
    size_t worstKept = 0;
    const char *worstPhase = "";
#endif
    for (const Aircraft &a : list) {
        if (!a.hasPos) {
            continue;
        }
#ifdef BENCH3D
        uint32_t t0 = micros();
#endif
        Track &t = g_tracks[a.hex];
        t.sim.report(followSampleOf(a, now));
        if (zoomList) {
            announceMovement(a, t, now);
        }
        if (!a.posStale) {
            t.last = a;
            t.freshMs = now;
            t.provider = provider;
            (zoomList ? t.inZoomMs : t.inMainMs) = now;
        }
#ifdef BENCH3D
        uint32_t us = micros() - t0;
        if (us > worstUs) {
            worstUs = us;
            worst = &a;
            worstKept = t.sim.historySize();
            worstPhase = t.sim.phase();
        }
#endif
    }
#ifdef BENCH3D
    uint32_t tookUs = micros() - startUs;
    if (tookUs > 30000 && worst != nullptr) {
        Serial.printf("[bench] tracks: %u in the %s list took %.1fms; the longest, %s %s %s ft %s kt, %.1fms with "
                      "%u reports kept, %s\n",
                      (unsigned)list.size(), zoomList ? "zoom's" : "main", tookUs / 1000.0f, worst->hex.c_str(),
                      worst->callsign.c_str(), worst->altStr.c_str(), worst->speedStr.c_str(), worstUs / 1000.0f,
                      (unsigned)worstKept, worstPhase);
    }
#endif
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
        // In the air with no speed to carry it on by, the estimate would hold
        // it where it was: standing still in the air, as only what hovers
        // does - see stillInTheAir(). It is let go instead.
        if (!t.sim.fresh().ground && !t.sim.fresh().hasGs && !canHover(t.last.type, t.last.category)) {
            continue;
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

#ifdef BENCH3D
// "##=k" (bench2d.h): a poll's list taken into the tracks, as every poll's
// is - 120 made-up aircraft round home, a third of them low - six polls of
// it, ten seconds apart as the aircraft go; then each step of taking one in,
// timed on its own; and the made-up tracks let go again.
void benchTracks() {
    constexpr int N = 120;
    uint32_t whole[6];
    for (int round = 0; round < 6; round++) {
        std::vector<Aircraft> list = benchMadeUp(N, g_lat, g_lon, 50, 10.0f * round);
        uint32_t t0 = micros();
        noteTracks(list, false, "bench");
        whole[round] = micros() - t0;
    }
    Serial.printf("[bench] tracks: %d aircraft taken in %.2fms the first time, then %.2f %.2f %.2f %.2f %.2fms\n", N,
                  whole[0] / 1000.0f, whole[1] / 1000.0f, whole[2] / 1000.0f, whole[3] / 1000.0f, whole[4] / 1000.0f,
                  whole[5] / 1000.0f);
    std::vector<Aircraft> list = benchMadeUp(N, g_lat, g_lon, 50, 60);
    uint32_t pressureUs = micros();
    notePressure(list);
    pressureUs = micros() - pressureUs;
    uint32_t findUs = 0, sampleUs = 0, reportUs = 0, copyUs = 0, now = millis();
    for (const Aircraft &a : list) {
        uint32_t t0 = micros();
        auto it = g_tracks.find(a.hex);
        uint32_t t1 = micros();
        FollowSample s = followSampleOf(a, now);
        uint32_t t2 = micros();
        if (it != g_tracks.end()) {
            it->second.sim.report(s);
        }
        uint32_t t3 = micros();
        if (it != g_tracks.end()) {
            it->second.last = a;
        }
        uint32_t t4 = micros();
        findUs += t1 - t0;
        sampleUs += t2 - t1;
        reportUs += t3 - t2;
        copyUs += t4 - t3;
    }
    Serial.printf("[bench] tracks, a step at a time for %d: the day's pressure %.2fms; finding each %.2fms, its "
                  "sample %.2fms, its runway simulation %.2fms, keeping its report %.2fms\n",
                  N, pressureUs / 1000.0f, findUs / 1000.0f, sampleUs / 1000.0f, reportUs / 1000.0f, copyUs / 1000.0f);
    for (auto it = g_tracks.begin(); it != g_tracks.end();) {
        it = it->first.startsWith("~") ? g_tracks.erase(it) : std::next(it);
    }
}
#endif

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
        a.altStr = e.ground ? String("GND") : String(feedAltFt(e));
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

// The fastest contact each view last drew, in knots, and how long its last
// frame took - see glideMs().
float g_radarFastestKt = 0, g_zoomFastestKt = 0;
uint32_t g_radarFrameUs = 0, g_zoomFrameUs = 0;

// How often a view is drawn between its polls: as tweenMs() says, or more
// often where its fastest contact crosses a pixel sooner - the radar close in,
// the zoom - so it glides across the plot a pixel at a time rather than
// jumping, down to `minMs` between frames, and to GLIDE_DRAWING_SHARE times
// as long as its last frame took.
uint32_t glideMs(float fastestKt, float rangeNm, int intervalS, uint32_t minMs, uint32_t frameUs) {
    float pxPerSecond = fastestKt / 3600.0f * (342.0f / std::max(rangeNm, 0.1f));
    uint32_t ms = pxPerSecond > 0.01f ? (uint32_t)(1000.0f / pxPerSecond) : UINT32_MAX;
    minMs = std::max(minMs, frameUs * GLIDE_DRAWING_SHARE / 1000);
    return std::max(minMs, std::min(ms, tweenMs(intervalS)));
}

// How far the radar shows when not following: the range ZOOM IN and OUT
// chose, inside the poll radius - which a range chosen before the radius was
// brought in under it falls back to.
int radarRangeShown() {
    if (g_mapRangeNm > 0) {
        return g_mapRangeNm;
    }
    return g_radarRangeNm > 0 && g_radarRangeNm < g_radiusNm ? g_radarRangeNm : g_radiusNm;
}

// Whether the radar is the map alone: zoomed out past where there is any
// traffic, and not following - which sets a range of its own.
bool radarOnMap() { return g_mapRangeNm > 0 && !g_follow.active; }

// The widest the poll radius may be, as the settings screen's slider allows
// for the traffic chosen.
int maxRadiusNow() { return radiusCeilingNm(trafficFilter().active()); }

// Following, the next of the radar's ranges out from `fromNm` - or in - no
// further than the poll may reach now and no closer than `minNm`; 0 for
// none. In from past that reach, the widest within it.
int followStep(int fromNm, bool out, int minNm) {
    int ceiling = maxRadiusNow(), next = 0;
    for (int r : RADAR_RANGE_STEPS_NM) {
        if (r < minNm || r > ceiling) {
            continue;
        }
        if (out && r > fromNm) {
            return r;
        }
        if (!out && r < fromNm) {
            next = r;
        }
    }
    return next;
}

// Widens the poll radius from the radar's ZOOM OUT, as the settings screen
// would: saved, the sky refetched, and a recording of the narrower one ended.
void setPollRadius(int radiusNm) {
    saveFilters(radiusNm, g_showRefresh, g_pollIntervalS, g_source);
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
    // Dragged somewhere of its own, the one nearest where it has been left.
    double fromLat = g_lat, fromLon = g_lon;
    if (radarScreenFreeLook(lat, lon)) {
        fromLat = lat;
        fromLon = lon;
        withinNm = (float)RADAR_RANGE_STEPS_NM[0];
    }
    if (!nearestAirport(fromLat, fromLon, a, withinNm) || !radarZoomFrame(a.code, lat, lon, rangeNm)) {
        return false;
    }
    code = a.code;
    return true;
}

// With `title`, a refresh of the plot - not `full` - draws the title again
// too, once it is known what it says: ZOOM IN and OUT, which can take the
// radar out onto the map and back, change only that, the plot and the
// buttons, and the whole screen drawn and pushed for them was 75 to 85ms.
void drawRadar(bool full, bool title = false) {
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
    String provider;
    uint32_t copyAt = micros();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        provider = radarOnExtraList() ? g_zoomProvider : g_activeProvider;
        // Following, the plot is the list fetched around the aircraft,
        // centred on where it is reckoned to be - and on a free look, the
        // one fetched around where the radar has been left. Nothing in that
        // list is marked new: the marks belong to the main poll's.
        aircraft = radarOnExtraList() ? g_zoomAircraft : g_latestAircraft;
        if (!radarOnExtraList()) {
            isNew = g_latestIsNew;
        }
        lat = g_lat;
        lon = g_lon;
        radius = g_radiusNm;
        xSemaphoreGive(g_dataMutex);
    }
    uint32_t copyUs = micros() - copyAt, lostUs = 0, nowUs = 0;
    if (g_follow.active) {
        followCentre(lat, lon);
        radius = g_follow.rangeNm;
        keepFollowedOnPlot(aircraft);
    } else {
        radius = radarRangeShown();
    }
    radarScreenSetMapOnly(radarOnMap());
    if (radarOnMap()) {
        aircraft.clear();
        isNew.clear();
    } else {
        uint32_t at = micros();
        fillLost(aircraft, radarOnExtraList(), provider);
        lostUs = micros() - at;
        at = micros();
        // Following, the list is the extra poll's, which on the radar goes at
        // the main poll's interval too.
        moveOnToNow(aircraft, radarOnExtraList() ? g_zoomDataMs : g_latestDataMs, g_pollIntervalS);
        nowUs = micros() - at;
    }
    // What the filter screen has left out - lost ones too, or they would come
    // back as their estimates - but never the aircraft followed.
    uint32_t filterAt = micros();
    applyTrafficFilter(aircraft, &isNew, g_follow.active ? g_follow.hex : String());
    g_perfPrepRadar.add(copyUs, lostUs, nowUs, micros() - filterAt);
    radarScreenSetFiltered(trafficFilter().active());
    if (title && !full) {
        radarScreenRedrawTitle();
    }
    g_radarDrawnMs = millis();
    radarScreenSetAutoRange(followAutoNow());
    radarScreenDraw(aircraft, isNew, lat, lon, radius, full);
    g_radarFastestKt = radarFastestKt();
    g_radarFrameUs = micros() - t0;
    g_perfRadar.add(g_radarFrameUs);
    if (full) {
        g_perfFull.add(g_radarFrameUs);
        g_passFull = true;
    }
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
    // Past the radius's ceiling there is always the map, to its own widest.
    constexpr int widestMap = MAP_RANGE_STEPS_NM[sizeof(MAP_RANGE_STEPS_NM) / sizeof(MAP_RANGE_STEPS_NM[0]) - 1];
    bool canOut = g_mapRangeNm < widestMap;
    if (g_follow.active) {
        // Its own ranges - see zoomFollow().
        canIn = followStep(g_follow.rangeNm, false, RADAR_RANGE_STEPS_NM[0]) != 0;
        canOut = followStep(g_follow.rangeNm, true, RADAR_RANGE_STEPS_NM[0]) != 0;
    }
    radarScreenSetZoom(canIn, canOut, onScreen);
}

bool openZoom(const String &code, bool forFollowed = false, float viewNm = 0);

// ZOOM IN or OUT: the next of the steps in that direction from the range
// shown now. In from the closest, the airport's own zoom; out past the poll
// radius, the radius itself, as far as the settings allow - and past that,
// the map alone, through MAP_RANGE_STEPS_NM, and in again back down them to
// the radius.
void aimFollowPoll(bool fresh);

// ZOOM IN or OUT while following: the next of the radar's ranges, held -
// auto zoom standing aside - until the next landing or take-off. Out, the
// sky to the new edge is fetched at once rather than at the next poll.
void zoomFollow(bool in) {
    int next = followStep(g_follow.rangeNm, !in, RADAR_RANGE_STEPS_NM[0]);
    if (next == 0) {
        return;
    }
    bool wider = next > g_follow.rangeNm;
    Serial.printf("[follow] %s range %d -> %d nm, picked\n", g_follow.callsign.c_str(), g_follow.rangeNm, next);
    g_follow.manualNm = next;
    g_follow.rangeNm = next;
    g_follow.autoVote = 0;
    aimFollowPoll(wider);
    setRadarControls(true);
    drawRadar(false, true);
}

void zoomRadar(bool in) {
    if (g_follow.active) {
        zoomFollow(in);
        return;
    }
    if (g_mapRangeNm > 0 || (!in && radarRangeShown() >= g_radiusNm && g_radiusNm >= maxRadiusNow())) {
        int next = 0;
        if (in) {
            for (int r : MAP_RANGE_STEPS_NM) {
                if (r < g_mapRangeNm && r > g_radiusNm) {
                    next = r;
                }
            }
        } else {
            for (int r : MAP_RANGE_STEPS_NM) {
                if (r > std::max(g_mapRangeNm, g_radiusNm)) {
                    next = r;
                    break;
                }
            }
            if (next == 0) {
                return;  // as wide as it goes
            }
        }
        g_mapRangeNm = next;  // 0, in past the last of them: the radius, and its traffic, again
        setRadarControls(true);
        drawRadar(false, true);
        return;
    }
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
                openZoom(code);  // Back comes back to the radar at this range
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
    drawRadar(false, true);  // the range readout too: its part in the margin goes with the footer
}

// The zoom's range as ZOOM IN and OUT have it, for its polls' frames to be
// recorded with.
void shareZoomView() {
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        float was = g_zoomPoll.viewNm;
        g_zoomPoll.viewNm = radarZoomViewNm();
        if (g_zoomPoll.viewNm != was) {
            noteRangeMark(g_zoomPoll.viewNm > 0 ? g_zoomPoll.viewNm : g_zoomRangeNm, true);
        }
        xSemaphoreGive(g_dataMutex);
    }
}

bool onGround(const Aircraft &ac);

// Following in its own airport's zoom with auto zoom on: the range there as
// it comes in - stepping in through the zoom's ranges as it nears, so the
// runways' middle stays on the plot round it (closer in than the framing,
// the zoom is centred on it) - and from touchdown, through the rollout and
// the taxi, the closest; a take-off the other way about, out a step at a time
// as it climbs away. In a step only once it is well inside the next one down,
// and out once it no longer fits, so a report jittering at an edge doesn't
// pump it. A range picked with ZOOM IN or OUT holds until the next touchdown
// or lift-off.
void autoZoomInZoom(const std::vector<Aircraft> &aircraft) {
    if (!g_follow.active || !g_follow.zoomIsItsOwn || !g_autoZoom) {
        return;
    }
    for (const Aircraft &ac : aircraft) {
        if (ac.hex != g_follow.hex || !ac.hasPos) {
            continue;
        }
        bool ground = onGround(ac);
        if (ground != g_follow.zoomGround) {
            g_follow.zoomGround = ground;
            g_follow.zoomManual = false;  // down, or off: what was picked was for before
        }
        if (g_follow.zoomManual) {
            return;
        }
        static const float STEPS[] = {0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};  // the zoom's own
        float framed = g_zoomRangeNm;
        float now = radarZoomViewNm() > 0 ? radarZoomViewNm() : framed;
        float want = now;
        if (ground) {
            want = STEPS[0];
        } else {
            float d = (float)haversineNm(g_zoomPoll.lat, g_zoomPoll.lon, ac.lat, ac.lon);
            if (d * 1.05f > now) {
                want = framed;  // out to the first that fits, or the framing
                for (float r : STEPS) {
                    if (r >= d * 1.25f) {
                        want = std::min(r, framed);
                        break;
                    }
                }
            } else {
                for (int i = (int)(sizeof(STEPS) / sizeof(STEPS[0])) - 1; i >= 0; i--) {
                    if (STEPS[i] < now - 0.01f) {
                        if (d * 1.25f <= STEPS[i]) {
                            want = STEPS[i];  // a step in
                        }
                        break;
                    }
                }
            }
        }
        if (fabsf(want - now) > 0.01f) {
            Serial.printf("[follow] %s zoom %.1f -> %.1f nm, auto zoom\n", g_follow.callsign.c_str(), now, want);
            radarZoomSetViewNm(want);
            radarZoomRefreshButtons();
            shareZoomView();
        }
        return;
    }
}

void drawZoom(bool full) {
    uint32_t t0 = micros();
    if (full) {
        updateRecButton(false);  // so REC is drawn showing the state as it is now
    }
    std::vector<Aircraft> aircraft;
    String provider;
    uint32_t copyAt = micros();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        aircraft = g_zoomAircraft;
        provider = g_zoomProvider;
        xSemaphoreGive(g_dataMutex);
    }
    uint32_t copyUs = micros() - copyAt;
    if (g_follow.active) {
        keepFollowedOnPlot(aircraft);
    }
    uint32_t at = micros();
    fillLost(aircraft, true, provider);
    uint32_t lostUs = micros() - at;
    at = micros();
    moveOnToNow(aircraft, g_zoomDataMs, ZOOM_POLL_INTERVAL_S);
    g_perfPrepZoom.add(copyUs, lostUs, micros() - at, 0);
    autoZoomInZoom(aircraft);
    g_zoomDrawnMs = millis();
    radarZoomDraw(aircraft, full);
    g_zoomFastestKt = radarFastestKt();
    g_zoomFrameUs = micros() - t0;
    g_perfZoom.add(g_zoomFrameUs);
    if (full) {
        g_perfFull.add(g_zoomFrameUs);
        g_passFull = true;
    }
}

// Zooms in on an airport tapped on the radar, or one the followed aircraft
// is coming down to - at its framing, or at `viewNm` as radarZoomViewNm() had
// it, for one being come back to. Until the zoom's own first fetch lands, the radar's
// contacts stand in: around an airport inside the radius they are the same
// aircraft, a poll older - and following, those fetched around the aircraft
// are, the airport being near it.
bool openZoom(const String &code, bool forFollowed, float viewNm) {
    double lat, lon;
    float rangeNm;
    if (!radarZoomOpen(code, lat, lon, rangeNm)) {
        return false;
    }
    radarZoomSetViewNm(viewNm);
    g_zoomCode = code;
    g_zoomRangeNm = rangeNm;
    g_follow.picking = false;
    g_follow.inZoom = false;
    g_follow.zoomDistNm = -1;
    g_follow.zoomIsItsOwn = forFollowed;
    g_follow.zoomManual = false;
    radarZoomSetFollow(g_follow.active ? g_follow.hex : String(), g_follow.callsign, false);
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_zoomPoll.active = true;
        g_zoomPoll.follow = false;
        g_zoomPoll.look = false;
        g_zoomPoll.code = code;
        g_zoomPoll.followHex = g_follow.active ? g_follow.hex : String();
        g_zoomPoll.gen++;
        g_zoomPoll.lat = lat;
        g_zoomPoll.lon = lon;
        g_zoomPoll.radiusNm = (int)ceilf(rangeNm) + ZOOM_FETCH_EXTRA_NM;
        g_zoomPoll.viewNm = radarZoomViewNm();
        // Until its own first list lands: the main poll's - or, opened from a
        // radar dragged somewhere else, the one that was fetched around there.
        if (!g_follow.active && !g_look.aimed) {
            g_zoomAircraft = g_latestAircraft;
            g_zoomDataMs = g_latestDataMs;  // moved on between polls from when it really arrived
        }
        g_zoomReady = false;
        xSemaphoreGive(g_dataMutex);
    }
    g_zoomUp = true;
    g_screen = Screen::ZOOM;
    if (!g_follow.active) {
        rememberZoom(code, radarZoomViewNm());
    }
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
        g_zoomPoll.look = false;
        g_zoomPoll.followHex = g_follow.hex;
        if (fresh) {
            g_zoomPoll.gen++;
        }
        g_zoomPoll.lat = lat;
        g_zoomPoll.lon = lon;
        g_zoomPoll.radiusNm = g_follow.rangeNm + ZOOM_FETCH_EXTRA_NM;
        if (g_zoomPoll.followNm != g_follow.rangeNm) {
            g_zoomPoll.followNm = g_follow.rangeNm;
            noteRangeMark((float)g_follow.rangeNm, false);
        }
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
        // Out a step at a time: the radar comes up around the airport, at
        // the first of its ranges wider than the zoom's, for its own ZOOM OUT
        // to go on from. Centred on it as if dragged there - unless it is
        // the airport the radar centres on anyway - with HOME or AIRPORT to
        // go back.
        double lat, lon;
        float rangeNm;
        if (radarZoomFrame(g_zoomCode, lat, lon, rangeNm)) {
            Airport own;
            if (g_radarCentre == RadarCentre::AIRPORT && nearestAirport(g_lat, g_lon, own, 120.0f, false) &&
                own.code == g_zoomCode) {
                radarScreenEndFreeLook();
            } else {
                radarScreenLookAt(lat, lon, g_lat, g_lon);
            }
            int step = 0;
            for (int r : RADAR_RANGE_STEPS_NM) {
                if (r > g_zoomRangeNm + 0.01f) {
                    step = r;
                    break;
                }
            }
            g_mapRangeNm = 0;
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                g_radarRangeNm = step >= g_radiusNm ? 0 : step;
                xSemaphoreGive(g_dataMutex);
            }
            saveRadarRange(g_radarRangeNm);
            setRadarControls(false);  // drawn with the radar, below
        }
    }
    rememberZoom(String(), 0);  // the radar is the radar again
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
//
// Never over the 3D view - not even as it opens, when g_screen still says the
// radar, the zoom or the details it is opened from, until view3d::open() has
// drawn it: auto-follow opening it on an alert put the banner into its first
// frame, and there it stayed, round the picture, which draws only within
// itself.
bool drawBanner(M5Canvas &canvas) {
    if (!g_bannerUp || !bannerScreen(g_screen) || view3d::isOpen()) {
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
    case Screen::VIEW3D:
        view3d::redraw();
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
void open3d();

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
        wakeScreen();            // and the screen up to see it
    }
    if (fresh) {
        Serial.printf("[alert] %s %s (+%d more)\n", g_bannerAc.callsign.c_str(),
                      alertReasonText(g_bannerAc, g_bannerAc.alert).c_str(), g_bannerMore);
        g_bannerUp = true;
        g_bannerShownMs = 0;
        // Following, the chime and the banner alone: the words cut across
        // the clearances and the engines of the one being watched, with news
        // of something else. An emergency is said whatever.
        bool spoken = !g_follow.active || (g_bannerAc.alert & ALERT_EMERGENCY);
        // In the voice of where it is - or, with no position, of home, which it is near.
        soundAlert(g_bannerAc.alert, g_bannerAc.callsign,
                   g_bannerAc.type.length() ? lookupAircraftType(g_bannerAc.type, g_bannerAc.military) : String(),
                   g_bannerAc.hasPos ? g_bannerAc.lat : (float)g_lat, g_bannerAc.hasPos ? g_bannerAc.lon : (float)g_lon,
                   spoken);
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

String watchableCallsign(String callsign);
bool onWatchlist(const String &callsign);

// Its details, read aloud unless `read` says not - see soundReadAircraft().
// The route lookups finished when the details were last drawn - see route.h.
uint32_t g_detailRouteGen = 0;

void openDetail(const Aircraft &ac, Screen returnTo, bool read = true) {
    // Out of a replay - its screen, or its 3D view - 3D in FOLLOW's place,
    // but for the one that view shows.
    bool replayed = returnTo == Screen::PLAYBACK || (returnTo == Screen::VIEW3D && g_view3dReplay);
    DetailFollow follow = !ac.hasPos ? DetailFollow::NONE
                          : replayed ? (returnTo == Screen::VIEW3D && ac.hex == g_replayHex ? DetailFollow::NONE
                                                                                           : DetailFollow::VIEW_3D)
                          : g_follow.active && g_follow.hex == ac.hex ? DetailFollow::UNFOLLOW
                                                                      : DetailFollow::FOLLOW;
    String callsign = watchableCallsign(ac.callsign);
    DetailWatch watch = replayed || !callsign.length() ? DetailWatch::NONE
                        : onWatchlist(callsign)        ? DetailWatch::UNWATCH
                                                       : DetailWatch::WATCH;
    detailScreenSet(ac, follow, watch);
    g_detailAc = ac;
    g_detailReturnTo = returnTo;
    g_screen = Screen::DETAIL;
    routeRequest(ac.callsign);  // its row says "looking up..." until the answer redraws it
    g_detailRouteGen = routeGeneration();
    detailScreenDraw();
    if (read) {
        soundReadAircraft(ac);  // if read aloud is on
    }
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
    radarScreenSetRecording(state, elapsed, onScreen && (g_screen == Screen::ZOOM || g_screen == Screen::RADAR));
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
            h.followRangeNm = g_follow.rangeNm;
            h.followAuto = followAutoNow();
        }
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            h.lat = g_lat;
            h.lon = g_lon;
            h.radiusNm = g_radiusNm;
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
    // Short enough for its row with read aloud on the end: "4/4 alerts, auto-record off, read aloud".
    return String(on) + "/4 alerts, auto-record " +
           (!g_autoRecord ? "off" : recorder::mounted() ? "on" : "on (no card)") + (g_readAloud ? ", read aloud" : "");
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
        for (const Aircraft &a : radarOnExtraList() ? g_zoomAircraft : g_latestAircraft) {
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

// Following starts a recording when auto-record is on. Following by hand,
// it runs until the user ends following or taps REC - and on past an
// aircraft lost, for REC to stop. Auto-follow's stops too once the aircraft
// has come to rest on the ground, or following ends itself. A recording
// already running that was started by hand is left to run, since it was
// asked for. One an alert started gives way
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
    h.followRangeNm = g_follow.rangeNm;
    h.followAuto = followAutoNow();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        h.lat = g_lat;
        h.lon = g_lon;
        h.radiusNm = g_radiusNm;
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
    g_mapRangeNm = 0;  // an alert's auto-follow can find the radar out on the map
    g_follow.hex = ac.hex;
    g_follow.callsign = ac.callsign;
    g_follow.last = ac;
    g_follow.seenMs = millis();
    g_follow.airborne = !onGround(ac);
    g_follow.seenGround = onGround(ac);
    g_follow.rangeNm = followRangeFor(ac);
    // Its runway simulation as its track has it - whether it was in the air
    // lately, a take-off or landing under way - not begun again from nothing.
    auto track = g_tracks.find(ac.hex);
    if (track != g_tracks.end() && track->second.sim.haveFresh()) {
        g_follow.sim = track->second.sim;
    }
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

// Like stopFollow(), for following that has run out on its own: auto-follow's
// recording stops with it, but one of following by hand runs on until the user
// stops it with REC.
void followEnded() {
    if (!g_follow.automatic && followRecording()) {
        Serial.printf("[follow] %s's recording runs on until REC is tapped\n", g_follow.callsign.c_str());
        g_follow.recording = false;
    }
    stopFollow();
    updateRecButton(true);
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

// How many others are on the plot `rangeNm` round the aircraft with ICAO
// `hex`, at (lat, lon): airborne, and through the filter - what auto zoom
// goes by.
int othersAround(const std::vector<Aircraft> &list, const String &hex, double lat, double lon, float rangeNm) {
    const TrafficFilterSet &f = trafficFilter();
    bool filtered = f.active();
    double nmPerDegLon = 60.0 * cos(lat * M_PI / 180.0);
    int n = 0;
    for (const Aircraft &a : list) {
        if (!a.hasPos || a.hex == hex || onGround(a) || (filtered && !f.passes(a))) {
            continue;
        }
        double dLon = a.lon - lon;
        dLon += dLon > 180 ? -360 : dLon < -180 ? 360 : 0;
        double dn = (a.lat - lat) * 60.0, de = dLon * nmPerDegLon;
        n += dn * dn + de * de <= (double)rangeNm * rangeNm;
    }
    return n;
}

// Auto zoom's step, from `others` on the plot at the range it has: out while
// the sky round it is quiet and in while it is busy - see AUTO_ZOOM_FEW and
// AUTO_ZOOM_MANY - so long as the step, going by how many there are to the
// square mile now, wouldn't leave it as far the other way. Out at once, so a
// quiet sky opens up a poll at a time, each widening fetched straight away;
// in only when a second poll running agrees, so a crowd passing through
// doesn't pump it in and out.
int autoZoomRange(int others) {
    int range = g_follow.rangeNm;
    int out = followStep(range, true, AUTO_ZOOM_MIN_NM), in = followStep(range, false, AUTO_ZOOM_MIN_NM);
    if (range < AUTO_ZOOM_MIN_NM) {
        return out ? out : range;  // as close as a landing had it, and away again
    }
    auto scaled = [&](int to) { return others * (float)to * to / ((float)range * range); };
    int want = 0;
    if (out && others < AUTO_ZOOM_FEW && scaled(out) <= AUTO_ZOOM_MANY) {
        want = 1;
    } else if (in && others > AUTO_ZOOM_MANY && scaled(in) >= AUTO_ZOOM_FEW) {
        want = -1;
    }
    if (want > 0 || (want < 0 && g_follow.autoVote < 0)) {
        g_follow.autoVote = 0;
        return want > 0 ? out : in;
    }
    g_follow.autoVote = want;
    return range;
}

// The range following shows it at, worked out again at each poll around it
// with `others` on the plot at the range it has now: coming down to an
// airfield, closing in on it as ever - see followRangeFor() - riding over a
// level-off on the way down; in the climb and the cruise 20nm, or with auto
// zoom sized to the sky round it; and over either, a range picked with ZOOM
// IN or OUT, held until the next landing or take-off.
int followRangeNow(const Aircraft &ac, int others) {
    Airport dest;
    float destNm;
    bool found = followDestination(ac, dest, destNm);
    bool coming = found || (g_follow.approach && ac.status != "CLIMB" && g_follow.approachMisses < 2);
    g_follow.approachMisses = found ? 0 : std::min(g_follow.approachMisses + 1, 99);
    if (coming != g_follow.approach) {
        // On the way down, or away again: a range picked was for what went
        // before, and auto zoom starts again from where it is.
        g_follow.approach = coming;
        g_follow.manualNm = 0;
        g_follow.autoVote = 0;
    }
    int range;
    if (g_follow.manualNm) {
        range = g_follow.manualNm;
    } else if (coming) {
        // Its airport missed this poll - level a moment on the way down - as it was.
        range = found ? followRangeFor(ac, g_autoZoom) : std::min(g_follow.rangeNm, FOLLOW_RANGE_NM);
    } else if (g_autoZoom) {
        range = autoZoomRange(others);
    } else {
        range = FOLLOW_RANGE_NM;
    }
    // A filter taken off since can leave it past what the poll may reach now.
    if (range > maxRadiusNow()) {
        range = followStep(range, false, RADAR_RANGE_STEPS_NM[0]);
    }
    return range;
}

// Keeps following up to date with each list fetched around the aircraft:
// where it is, whether it has landed, and which view it belongs in.
void tickFollow() {
    if (!g_follow.active) {
        return;
    }
    Aircraft ac;
    bool fresh = false, found = false;
    int others = 0;
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
            if (found) {
                others = othersAround(g_zoomAircraft, g_follow.hex, ac.lat, ac.lon, (float)g_follow.rangeNm);
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
    } else if (now - g_follow.seenMs > std::max(FOLLOW_LOST_MS, g_follow.sim.keepMs()) &&
               !(g_screen == Screen::VIEW3D && !g_view3dReplay &&
                 (g_picker.guest().length() > 0 || view3d::parkingLost()))) {
        Serial.printf("[follow] lost %s\n", g_follow.callsign.c_str());
        followEnded();
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
    if (g_follow.automatic && followRecording() && g_follow.airborne && !g_follow.departed && onGround(ac) &&
        stopped) {
        Serial.printf("[follow] %s has stopped on the ground - ending its recording\n", g_follow.callsign.c_str());
        stopFollowRecording();
        updateRecButton(true);
    }

    // Auto-follow follows a departure until it is beyond the radius, where the
    // rest of the radar's sky stops: by then it is on its way to somewhere
    // else. Followed by hand, it is followed on, as far as the user likes.
    if (g_follow.automatic && g_follow.departed && !g_follow.lost &&
        haversineNm(g_lat, g_lon, ac.lat, ac.lon) > g_radiusNm) {
        Serial.printf("[follow] %s has left the %d nm radius - done following it\n", g_follow.callsign.c_str(),
                      g_radiusNm);
        followEnded();
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
            // Up from the field: a range picked on the way down let go, and
            // auto zoom from here.
            g_follow.manualNm = 0;
            g_follow.approach = false;
            g_follow.approachMisses = 0;
            g_follow.autoVote = 0;
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
    // The range, and the fetch around it with it: closing in on its
    // destination, so the plot keeps the airport on it and the answers get
    // smaller - or out and in with the sky round it, under auto zoom.
    int was = g_follow.rangeNm;
    int range = followRangeNow(ac, others);
    if (range != was) {
        Serial.printf("[follow] %s range %d -> %d nm%s, %d others\n", g_follow.callsign.c_str(), was, range,
                      g_follow.manualNm ? ", picked" : g_follow.approach ? "" : g_autoZoom ? ", auto zoom" : "", others);
        g_follow.rangeNm = range;
        setRadarControls(g_screen == Screen::RADAR);
    }
    aimFollowPoll(range > was);  // wider: the sky out to it fetched now, rather than at the next poll
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
    g_follow.automatic = true;
    if (showRadar) {
        g_screen = Screen::RADAR;
        drawRadar(true);
        // Set to follow in 3D: into the 3D view, as the 3D button would -
        // from the radar or the zoom, never out of anything else being done.
        if (g_followIn3d) {
            open3d();
        }
    }
}

// Where the list of recordings was opened from - the table's header, the
// radar or the airport zoom, each of which has a button for it - for its Back
// to go back to.
Screen g_recordingsReturnTo = Screen::RADAR;

void openRecordings(Screen from) {
    g_follow.picking = false;
    g_recordingsReturnTo = from;
    g_screen = Screen::RECORDINGS;
    recordingsScreenEnter();
}

void toggleMute();

void openFilter(Screen returnTo);

// 3D on the radar or the zoom while following: the 3D view of it - unless
// the chip's own memory hasn't room for its bands just now, when nothing
// happens but a line in the log.
void open3d() {
    if (!g_follow.active) {
        return;
    }
    // A replay's left behind - its details' route jumped to a zoom, and the
    // zoom's 3D pressed there: the live view now.
    if (g_view3dReplay) {
        g_view3dReplay = false;
        view3d::setReplay(nullptr);
        view3d::setDayPressure(dayPressureFt);
    }
    if (!view3d::open()) {
        return;
    }
    g_view3dReturnTo = g_screen;
    g_screen = Screen::VIEW3D;
    g_picker.reset();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_view3dTraffic = g_zoomAircraft;
        xSemaphoreGive(g_dataMutex);
    }
    g_view3dTrafficMs = g_zoomDataMs;
}

void leave3dReplay();

// Back - or the following ended - to where it was opened from.
void leave3d() {
    if (g_view3dReplay) {
        leave3dReplay();
        return;
    }
    view3d::close();
    g_screen = g_view3dReturnTo;
    g_picker.reset();
    repaintCurrentScreen();
}

// --- the 3D view of a replay ----------------------------------------------------------

// The 3D view's flight out of the replay at `t` - as followedOf() makes it
// of a live track: the aircraft with `hex`, as its simulation has it then.
// False where the recording has nothing of it by then, or nothing for as
// long as following would wait.
bool replayFollowed(const String &hex, uint32_t t, view3d::Followed &f) {
    const RunwaySim *sim = replay::simAt(hex, t);
    uint32_t now = replay::clockAt(t);
    if (sim == nullptr || now - sim->fresh().ms > std::max(FOLLOW_LOST_MS, sim->keepMs())) {
        return false;
    }
    f = view3d::Followed();
    f.hex = hex;
    f.callsign = replay::callsignOf(hex);
    replay::kindOf(hex, f.type, f.category, f.military);
    f.now = sim->at(now);
    f.report = sim->fresh();
    f.reportMs = f.report.ms;
    f.landed = sim->airborneWithin(f.report.ms, subject3d::LANDED_WITHIN_MS);
    f.simulating = sim->simulating(now);
    f.movement = sim->movement();
    f.runway = sim->runway();
    f.lost = now - f.report.ms > subject3d::FRESH_MS;
    return true;
}

// The aircraft with ICAO `hex` as the replay's poll at the moment shown has
// it - or, missing from it, as the recording knows it, where it is reckoned
// to be - for its details.
bool replayAircraft(const String &hex, Aircraft &out) {
    for (const Aircraft &a : g_view3dTraffic) {
        if (a.hex == hex) {
            out = a;
            return true;
        }
    }
    view3d::Followed f;
    if (!replayFollowed(hex, playbackScreenTime(), f)) {
        return false;
    }
    out = Aircraft();
    out.hex = hex;
    out.callsign = f.callsign;
    out.type = f.type;
    out.category = f.category;
    out.military = f.military;
    out.hasPos = out.posStale = out.hasTrack = true;
    out.lat = f.now.lat;
    out.lon = f.now.lon;
    out.track = f.now.track;
    out.altStr = f.now.ground ? String("GND") : String(feedAltFt(f.now));
    out.speedStr = view3d::speedKnown(f) ? String(f.now.gsKt) : String("?");
    out.status = estimateStatus(f.now);
    return true;
}

// The replay's place and pace, for the 3D view's strip.
view3d::ReplayBar replayBar() {
    view3d::ReplayBar bar;
    bar.playing = playbackScreenPlaying();
    bar.speed = playbackScreenSpeed();
    bar.t = playbackScreenTime();
    bar.durationMs = replay::durationMs();
    bar.epoch = replay::epochAt(bar.t);
    return bar;
}

// The day's pressure as the recording had it at the moment shown, for the
// 3D view's traffic.
bool replayDayPressure(int32_t &ft) { return replay::dayPressureAt(playbackScreenTime(), ft); }

// Into the 3D view of the replay, at the moment it shows: of `hex`, or -
// `tracking` - of whichever aircraft the recording follows as it plays.
// False, and nothing changed, where the recording has nothing of it then, or
// the chip's own memory hasn't room for the view's bands.
bool open3dReplay(const String &hex, bool tracking) {
    view3d::Followed f;
    if (!replayFollowed(hex, playbackScreenTime(), f)) {
        Serial.printf("[3d] %s isn't in the recording at %lus\n", hex.c_str(),
                      (unsigned long)(playbackScreenTime() / 1000));
        return false;
    }
    playbackScreenEnter3d();
    view3d::ReplayBar bar = replayBar();
    view3d::setReplay(&bar);
    view3d::setDayPressure(replayDayPressure);
    Screen was = g_screen;
    g_screen = Screen::VIEW3D;
    if (!view3d::open()) {
        view3d::setReplay(nullptr);
        view3d::setDayPressure(dayPressureFt);
        g_screen = was;
        return false;
    }
    g_view3dReplay = true;
    g_replayTracking = tracking;
    g_replayHex = hex;
    g_replayPollIdx = SIZE_MAX;
    g_picker.reset();
    Serial.printf("[3d] replaying %s in 3D%s\n", hex.c_str(), tracking ? ", as the recording follows" : "");
    return true;
}

// Back from it, to the replay screen at the moment it shows.
void leave3dReplay() {
    view3d::close();
    view3d::setReplay(nullptr);
    view3d::setDayPressure(dayPressureFt);
    g_view3dReplay = false;
    g_screen = Screen::PLAYBACK;
    playbackScreenDraw();
}

// Each pass with it up: the replay played on, and its frame - or, its
// aircraft gone from the recording by then, back to the replay screen.
void frame3dReplay() {
    playbackScreenAdvance();
    uint32_t t = playbackScreenTime();
    if (g_replayTracking) {
        String followed = playbackScreenFollowed();
        if (followed.length() && followed != g_replayHex) {
            Serial.printf("[3d] the recording follows %s now\n", followed.c_str());
            g_replayHex = followed;
        }
    }
    view3d::Followed f;
    if (!replayFollowed(g_replayHex, t, f)) {
        Serial.printf("[3d] %s isn't in the recording at %lus - back to the replay\n", g_replayHex.c_str(),
                      (unsigned long)(t / 1000));
        leave3dReplay();
        return;
    }
    size_t poll = replay::pollIndexAt(t);
    if (poll != g_replayPollIdx) {
        g_replayPollIdx = poll;
        replay::pollAt(t, g_view3dTraffic, g_view3dTrafficMs);
    }
    view3d::ReplayBar bar = replayBar();
    view3d::setReplay(&bar);
    view3d::frame(f, g_view3dTraffic, g_view3dTrafficMs, replay::clockAt(t));
}

// --- what the 3D view shows ---------------------------------------------------------

// Following moved on to `hex`, the 3D view kept open on it: its take-off or
// landing, under way, carried on - startFollow() takes its track's runway
// simulation.
void followOnTo(const String &hex) {
    auto it = g_tracks.find(hex);
    if (it == g_tracks.end()) {
        return;
    }
    Serial.printf("[3d] %s has landed - following %s, next\n", g_follow.callsign.c_str(),
                  it->second.last.callsign.c_str());
    Aircraft next = it->second.last;
    stopFollow();
    startFollow(next);
    g_picker.clearGuest();
}

// Each pass, the 3D view open: what it shows - the one followed, or its
// guest - see subject3d.h.
void update3dSubject() {
    if (!g_follow.active) {
        g_picker.clearGuest();
        return;
    }
    String was = g_picker.guest(), next;
    subject3d::Show show = g_picker.pick(g_follow.hex, g_follow.sim, g_tracks, view3d::airport(),
                                         [](double lat, double lon) { return view3d::atStand(lat, lon); }, millis(),
                                         next);
    if (show == subject3d::Show::NEXT) {
        followOnTo(next);
        return;
    }
    const String &guest = g_picker.guest();
    if (guest != was) {
        auto it = guest.length() ? g_tracks.find(guest) : g_tracks.end();
        if (it != g_tracks.end()) {
            Serial.printf("[3d] showing %s while %s is on the ground\n", it->second.last.callsign.c_str(),
                          g_follow.callsign.c_str());
        } else {
            Serial.printf("[3d] back to %s\n", g_follow.callsign.c_str());
        }
    }
}

// The 3D view's flight, from a track: the guest's.
view3d::Followed followedOf(const String &hex, const Track &t) {
    view3d::Followed f;
    f.hex = hex;
    f.callsign = t.last.callsign;
    f.type = t.last.type;
    f.category = t.last.category;
    f.military = t.last.military;
    f.now = t.sim.at(millis());
    f.reportMs = t.sim.haveFresh() ? t.sim.fresh().ms : 0;
    if (t.sim.haveFresh()) {
        f.report = t.sim.fresh();
        f.landed = t.sim.airborneWithin(f.report.ms, subject3d::LANDED_WITHIN_MS);
    }
    f.simulating = t.sim.simulating(millis());
    f.movement = t.sim.movement();
    f.runway = t.sim.runway();
    f.lost = millis() - t.freshMs > subject3d::FRESH_MS;
    String call = g_follow.callsign;
    call.trim();
    f.note = "while " + (call.length() ? call : g_follow.hex) +
             (g_follow.sim.airborneWithin(millis(), subject3d::LANDED_WITHIN_MS) ? " taxies in" : " is on the ground");
    return f;
}

// The flight as the 3D view wants it, from what following knows - or its
// guest, while the one followed is taxiing.
view3d::Followed followed3d() {
    view3d::Followed f;
    const String &guestHex = g_picker.guest();
    auto guest = guestHex.length() ? g_tracks.find(guestHex) : g_tracks.end();
    if (guest != g_tracks.end()) {
        f = followedOf(guestHex, guest->second);
    } else {
        f.hex = g_follow.hex;
        f.callsign = g_follow.callsign;
        f.type = g_follow.last.type;
        f.category = g_follow.last.category;
        f.military = g_follow.last.military;
        f.now = followEstimateNow();
        f.reportMs = g_follow.sim.haveFresh() ? g_follow.sim.fresh().ms : 0;
        if (g_follow.sim.haveFresh()) {
            f.report = g_follow.sim.fresh();
            f.landed = g_follow.sim.airborneWithin(f.report.ms, subject3d::LANDED_WITHIN_MS);
        }
        f.simulating = g_follow.sim.simulating(millis());
        f.movement = g_follow.sim.movement();
        f.runway = g_follow.sim.runway();
        f.lost = g_follow.lost;
    }
    // The zoom's runways in use - read once a second, its list not built
    // every frame.
    static uint32_t inUseMs = 0;
    static char inUseAirport[5] = "";
    static char inUse[8][4] = {};
    static int inUseCount = 0;
    if (inUseMs == 0 || millis() - inUseMs >= 1000) {
        inUseMs = millis();
        recorder::RunwaysInUse now;
        radarLiveRunwaysInUse(now);
        snprintf(inUseAirport, sizeof inUseAirport, "%s", now.code.c_str());
        inUseCount = 0;
        for (const recorder::RunwaysInUse::End &e : now.ends) {
            if (inUseCount < 8) {
                snprintf(inUse[inUseCount++], sizeof inUse[0], "%s", e.ident);
            }
        }
    }
    memcpy(f.inUseAirport, inUseAirport, sizeof f.inUseAirport);
    memcpy(f.inUse, inUse, sizeof f.inUse);
    f.inUseCount = inUseCount;
    return f;
}

// On the 3D view: a button acts as the finger lands; a finger moved across
// the picture turns the camera; a tap on an aircraft opens its details.
// What an action on the 3D view does - a button, a tap on an aircraft, a
// replay's controls - `hex` the aircraft tapped.
void act3d(view3d::Action action, const String &hex);

void handleView3dTouch(bool touching) {
    static bool down = false;
    // The finger that opened it - acting as it landed on the radar or the
    // zoom - is spent until it lifts.
    if (g_touchSpent) {
        g_touchSpent = touching;
        return;
    }
    view3d::Action action = view3d::Action::NONE;
    String hex;
    if (touching) {
        auto t = M5.Touch.getDetail(0);
        bool landed = !down;
        down = true;
        action = view3d::touch(t.x, t.y, landed);
        if (landed) {
            soundKeyClick();
        }
    } else if (down) {
        down = false;
        action = view3d::lift(hex);
    }
    if (action == view3d::Action::BACK) {
        down = false;
        g_touchSpent = true;  // the radar or the zoom, back under the finger, not to act on it
    }
    act3d(action, hex);
}

void act3d(view3d::Action action, const String &hex) {
    if (action == view3d::Action::BACK) {
        leave3d();
    } else if (action == view3d::Action::DETAIL) {
        Aircraft ac;
        if (g_view3dReplay ? replayAircraft(hex, ac) : lastReported(hex, ac)) {
            view3d::close();
            // A replay paused, the moment still there to come back to - as a
            // tap on its plot pauses it.
            if (g_view3dReplay && playbackScreenPlaying()) {
                playbackScreenPlayPause();
            }
            openDetail(ac, Screen::VIEW3D);
        }
    } else if (g_view3dReplay) {
        switch (action) {
        case view3d::Action::PLAY: playbackScreenPlayPause(); break;
        case view3d::Action::BACK_MINUTE: playbackScreenStep(false); break;
        case view3d::Action::SPEED: playbackScreenNextSpeed(); break;
        case view3d::Action::ON_MINUTE: playbackScreenStep(true); break;
        case view3d::Action::SEEK: playbackScreenSeekTo(view3d::seekTo()); break;
        default: break;
        }
    }
}

void handleRadarTouch(int x, int y) {
    if (radarScreenHitMute(x, y)) {
        toggleMute();
        return;
    }
    if (radarScreenHitFilter(x, y)) {
        openFilter(Screen::RADAR);
        return;
    }
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
    case RadarAction::DRAG:
        g_dragging = true;
        break;
    case RadarAction::OPEN_3D:
        open3d();
        break;
    // Each button is lit as soon as it is tapped, so the tap shows it
    // registered before the plot - the slower part - is redrawn under it.
    case RadarAction::CENTRE_HOME:
    case RadarAction::CENTRE_AIRPORT:
    case RadarAction::CENTRE_ALL: {
        RadarCentre centre = (action == RadarAction::CENTRE_HOME) ? RadarCentre::HOME : RadarCentre::AIRPORT;
        bool airports = (action == RadarAction::CENTRE_ALL);
        // Dragged off its centre, the plot goes back to the one tapped - with
        // the title, which said so, as when following ends - but for ALL,
        // which marks every airport around where it has been left, and
        // leaves it there.
        double lookLat, lookLon;
        bool stay = airports && !g_follow.active && radarScreenFreeLook(lookLat, lookLon);
        bool wasFollowing = (!stay && radarScreenEndFreeLook()) || g_follow.active;
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
            // Out on the map, back in to the traffic, for one to be picked.
            bool offMap = g_follow.picking && g_mapRangeNm > 0;
            if (offMap) {
                g_mapRangeNm = 0;
            }
            setRadarControls(true);
            if (offMap) {
                drawRadar(true);
            }
        }
        break;
    case RadarAction::TOGGLE_RECORD:
        toggleRecording();
        break;
    case RadarAction::OPEN_RECORDINGS:
        // Greyed out without a card, but a tap still has another look for one.
        if (!recorder::mount()) {
            updateRecButton(true);
            break;
        }
        openRecordings(Screen::RADAR);
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
    if (radarScreenHitMute(x, y)) {
        toggleMute();
        return;
    }
    String hex;
    switch (radarZoomHandleTouch(x, y, hex)) {
    case ZoomAction::BACK:
        if (g_jump.active) {
            // Opened from an aircraft's route: back to its details, as they
            // were come to - from another zoom, that one opened again under
            // them for their own Back.
            g_jump.active = false;
            rememberZoom(g_jump.savedCode, g_jump.savedViewNm);
            if (g_jump.priorZoom.length() && !openZoom(g_jump.priorZoom, false, g_jump.priorViewNm)) {
                g_jump.returnTo = Screen::MAIN;
            }
            if (g_jump.priorZoom.length()) {
                rememberZoom(g_jump.savedCode, g_jump.savedViewNm);
            }
            openDetail(g_jump.ac, g_jump.returnTo, false);
            break;
        }
        // To the table, as the radar's Back is, the zoom kept for the radar to
        // come back to - see g_returnZoomCode. Following carries on out of
        // sight, and it is the radar centred on the aircraft that comes back,
        // to hand it to a zoom again if that is where it still is.
        g_follow.picking = false;
        g_screen = Screen::MAIN;  // and loop() ends the zoom's fetches
        displayInvalidate();
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        break;
    case ZoomAction::UNZOOM:
        g_jump.active = false;  // out to the radar, the details left behind
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
    case ZoomAction::OPEN_RECORDINGS:
        // As the radar's: greyed out without a card, but a tap still has
        // another look for one. The zoom is left as Back leaves it, for
        // the recordings' own Back to come back to.
        if (!recorder::mount()) {
            updateRecButton(true);
            break;
        }
        openRecordings(Screen::ZOOM);
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
    case ZoomAction::RANGE:
        // Picked by hand, following: auto zoom stands aside till the next
        // touchdown or lift-off.
        if (g_follow.active) {
            g_follow.zoomManual = true;
        }
        [[fallthrough]];
    case ZoomAction::REDRAW:
        shareZoomView();
        if (!g_follow.active) {
            rememberZoom(g_zoomCode, radarZoomViewNm());
        }
        drawZoom(true);
        break;
    case ZoomAction::DISMISS:
        drawZoom(true);
        break;
    case ZoomAction::DRAG:
        g_dragging = true;
        break;
    case ZoomAction::OPEN_3D:
        open3d();
        break;
    case ZoomAction::NONE:
        break;
    }
}

void tickLook();

// The finger that landed on a plot's empty sky has moved, to (x, y): the line
// it is dragging out goes with it.
void dragTo(int x, int y) {
    if (!radarDragTo(x, y)) {
        return;
    }
    if (g_screen == Screen::RADAR) {
        drawRadar(false);
    } else if (g_screen == Screen::ZOOM) {
        drawZoom(false);
    }
}

// And it has lifted: the plot is centred where the line had got to, with the
// title and the buttons saying so.
void endDrag() {
    if (!g_dragging) {
        return;
    }
    g_dragging = false;
    if (!radarDragEnd()) {
        return;
    }
    if (g_screen == Screen::RADAR) {
        tickLook();  // so that it is drawn with what there is for where it now is
        setRadarControls(false);
        drawRadar(true);
    } else if (g_screen == Screen::ZOOM) {
        drawZoom(true);
    }
}

// Keeps the extra poll aimed where the radar has been dragged to, at the
// range it shows, while that is where it is and nothing else has the poll -
// and lets go of it when the radar is back on its own centre. Looked at
// every pass rather than at each thing that could change it: a drag, HOME, a
// change of range or of traffic, following starting, a zoom opening over the
// radar and closing again.
void tickLook() {
    double lat = 0, lon = 0;
    // Only while the radar is what is up, or a detail opened from it: left
    // for the table, it stops asking, and asks again on the way back.
    bool shown = g_screen == Screen::RADAR || (g_screen == Screen::DETAIL && g_detailReturnTo == Screen::RADAR);
    // Nor out on the map, which has no traffic to fetch.
    bool look = radarScreenFreeLook(lat, lon) && !g_follow.active && shown && !radarOnMap();
    if (g_zoomUp || g_follow.active) {
        g_look.aimed = false;  // theirs for now: a zoom's closing gives it up, for this to take again
        return;
    }
    int radius = radarRangeShown() + ZOOM_FETCH_EXTRA_NM;
    if (look && g_look.aimed && g_look.lat == lat && g_look.lon == lon && g_look.radiusNm == radius) {
        return;
    }
    if (!look && !g_look.aimed) {
        return;
    }
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        if (look) {
            g_zoomPoll.active = true;
            g_zoomPoll.follow = false;
            g_zoomPoll.look = true;
            g_zoomPoll.followHex = "";
            g_zoomPoll.code = "";
            g_zoomPoll.gen++;  // straight away, and whatever is on its way for somewhere else dropped
            g_zoomPoll.lat = lat;
            g_zoomPoll.lon = lon;
            g_zoomPoll.radiusNm = radius;
            if (!g_look.aimed) {
                // The main poll's stands in until the first fetch lands, a
                // few seconds from now: what of it is near enough is real.
                g_zoomAircraft = g_latestAircraft;
                g_zoomProvider = g_activeProvider;
                g_zoomDataMs = g_latestDataMs;
            }
        } else {
            g_zoomPoll.active = false;
            g_zoomPoll.look = false;
            g_zoomAircraft.clear();
            g_zoomReady = false;
        }
        xSemaphoreGive(g_dataMutex);
    }
    g_look = {look, lat, lon, radius};
}

void handleRecordingsTouch(int x, int y) {
    String path;
    switch (recordingsScreenHandleTouch(x, y, path)) {
    case RecordingsAction::BACK:
        // To wherever Replays was tapped. The table's own button brought it
        // from there; the zoom's left the zoom as its Back does, to be opened
        // again as the table's radar button would - unless something is being
        // followed, when it is the radar centred on that which comes back.
        if (g_recordingsReturnTo == Screen::MAIN) {
            g_screen = Screen::MAIN;
            displayInvalidate();
            if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                g_dataReady = true;
                xSemaphoreGive(g_dataMutex);
            }
            break;
        }
        if (g_recordingsReturnTo == Screen::ZOOM && g_returnZoomCode.length() && !g_follow.active &&
            openZoom(g_returnZoomCode, false, g_returnZoomViewNm)) {
            break;
        }
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
    case PlaybackAction::VIEW_3D:
        open3dReplay(playbackScreenFollowed(), true);
        break;
    case PlaybackAction::EXPORT:
        // Blocks until the video is made or cancelled - minutes, for a long
        // recording at a slow speed, which hangWatch() would otherwise take
        // for a hang and abort 90 seconds in.
        g_loopBusyLong = true;
        playbackScreenExport();
        g_loopBusyLong = false;
        break;
    case PlaybackAction::NONE:
        break;
    }
}

// A callsign as the watchlist has it: trimmed and upper-cased - or "" where
// it can't go on it, held to the same rules as a typed entry so a feed's
// oddity can't put something on the list that the editor would then refuse
// to save.
String watchableCallsign(String callsign) {
    callsign.trim();
    callsign.toUpperCase();
    if (callsign.length() == 0 || callsign == "UNKNOWN" || watchlistProblem(callsign).length()) {
        return String();
    }
    return callsign;
}

bool onWatchlist(const String &callsign) {
    for (const String &e : parseWatchlist(g_watchlist)) {
        if (e == callsign) {
            return true;
        }
    }
    return false;
}

// Puts `callsign`, a watchableCallsign(), on the watchlist, or takes it off
// if it is already there - from a long press on its table row, or WATCH on
// its details. What happened, for a notice to say.
String toggleWatchlist(const String &callsign) {
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
        return "The watchlist is full - remove something in Settings first";
    }

    g_watchlist = list;
    saveAlerts(g_alertMask, g_watchlist, g_autoRecord, g_autoFollow, g_followIn3d);
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_alertRules.watch = parseWatchlist(g_watchlist);
        g_pollNow = true;  // so the row changes colour now rather than an interval on
        xSemaphoreGive(g_dataMutex);
    }
    String notice = removed ? "Removed " + callsign + " from the watchlist" : "Added " + callsign + " to the watchlist";
    if (!removed && !(g_alertMask & ALERT_WATCHLIST)) {
        notice += " - but watchlist alerts are switched off";
    }
    Serial.printf("[watch] %s\n", notice.c_str());
    return notice;
}

// Back to the display page's brightness from dim: a touch, or an alert.
void wakeScreen() {
    g_lastTouchMs = millis();
    if (g_dimmed) {
        g_dimmed = false;
        displayApplyBrightness(g_brightnessPct);
    }
}

// The sound off or on again: from the table's header, or the radar's or the
// zoom's title - the one switch, shown in all three.
void toggleMute() {
    g_muted = !g_muted;
    soundSetMuted(g_muted);
    saveMuted(g_muted);
    bool plot = g_screen == Screen::RADAR || g_screen == Screen::ZOOM;
    // The table's icon repaints itself where the table thinks it is up - which,
    // under the radar, it would still - so it is told it isn't, first; it is
    // drawn afresh on the way back anyway.
    if (g_screen != Screen::MAIN) {
        displayInvalidate();
    }
    displaySetMuted(g_muted);
    radarScreenSetMuted(g_muted, plot);
    if (!g_muted) {
        soundNewFlight();  // unmuting says so in the medium being unmuted
    }
}

// A long press on a table row puts that flight's callsign on the watchlist,
// or takes it off if it is already there.
void handleMainHold(int x, int y) {
    String hex, callsign;
    if (!displayHitRow(x, y, hex)) {
        return;
    }
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        for (const Aircraft &a : g_latestAircraft) {
            if (a.hex == hex) {
                callsign = a.callsign;
            }
        }
        xSemaphoreGive(g_dataMutex);
    }
    soundKeyClick();
    callsign = watchableCallsign(callsign);
    if (!callsign.length()) {
        displayShowNotice("This flight has no callsign to watch for");
        return;
    }
    displayShowNotice(toggleWatchlist(callsign));
}

void handleAlertsTouch(int x, int y) {
    switch (alertsScreenHandleTouch(x, y)) {
    case AlertsAction::BACK: {
        g_alertMask = alertsScreenMask();
        g_autoRecord = alertsScreenAutoRecord();
        g_autoFollow = alertsScreenAutoFollow();
        g_followIn3d = alertsScreenFollowIn3d();
        g_watchlist = alertsScreenWatchlist();
        saveAlerts(g_alertMask, g_watchlist, g_autoRecord, g_autoFollow, g_followIn3d);
        if (alertsScreenReadAloud() != g_readAloud || alertsScreenElsewhereAccent() != g_elsewhereAccent) {
            g_readAloud = alertsScreenReadAloud();
            g_elsewhereAccent = alertsScreenElsewhereAccent();
            saveVoice(g_readAloud, g_elsewhereAccent);
            soundSetReadAloud(g_readAloud);
            soundSetElsewhereAccent(g_elsewhereAccent);
        }
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
        // Back where it was left: the zoom, if it was on one - this boot or
        // the last.
        String code = g_returnZoomCode;
        if (code.length() && !g_follow.active) {
            if (openZoom(code, false, g_returnZoomViewNm)) {
                return;
            }
            rememberZoom(String(), 0);  // an airport no longer in the table
        }
        g_screen = Screen::RADAR;
        drawRadar(true);
        return;
    }

    if (displayHitFilter(x, y)) {
        openFilter(Screen::MAIN);
        return;
    }

    if (displayHitReplays(x, y)) {
        // Card or no card: the list says which, where the radar's button
        // would only be greyed out.
        openRecordings(Screen::MAIN);
        return;
    }

    if (displayHitCog(x, y)) {
        String label;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            label = g_label;
            xSemaphoreGive(g_dataMutex);
        }
        settingsScreenSet(g_radiusNm, g_showRefresh, g_pollIntervalS, label, g_source);
        settingsScreenSetAutoZoom(g_autoZoom);
        settingsScreenSetAlertSummary(alertSummary());
        g_screen = Screen::SETTINGS;
        settingsScreenDraw();
        return;
    }

    if (displayHitMute(x, y)) {
        toggleMute();
        return;
    }

    // A heading: the rows in its order, kept for next time.
    if (displayHitHeader(x, y)) {
        saveTableSort(displaySortColumn(), displaySortDescending());
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_dataReady = true;
            xSemaphoreGive(g_dataMutex);
        }
        return;
    }

    // A row: the aircraft on it, by hex - a poll can have landed since it
    // was drawn, and moved it.
    String hex;
    if (displayHitRow(x, y, hex)) {
        Aircraft tapped;
        bool found = false;
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            for (const Aircraft &a : g_latestAircraft) {
                if (a.hex == hex) {
                    tapped = a;
                    found = true;
                }
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
    // Picked on the 3D view: back into it, following this one - the radar or
    // the zoom it was opened over following it too, for Back.
    bool into3d = g_detailReturnTo == Screen::VIEW3D;
    Screen under = into3d ? g_view3dReturnTo : g_detailReturnTo;
    if (under == Screen::ZOOM && g_zoomUp) {
        startFollow(ac);
        g_follow.zoomIsItsOwn = true;
        radarZoomSetFollow(g_follow.hex, g_follow.callsign, false);
        if (into3d && view3d::open()) {
            g_screen = Screen::VIEW3D;
            return;
        }
        g_screen = Screen::ZOOM;
        drawZoom(true);
        return;
    }
    if (g_zoomUp) {
        endZoom();  // a zoom's poll is around its airport, not the aircraft
    }
    startFollow(ac);
    if (into3d && view3d::open()) {
        g_view3dReturnTo = Screen::RADAR;
        g_screen = Screen::VIEW3D;
        return;
    }
    g_screen = Screen::RADAR;
    drawRadar(true);
}

void handleDetailTouch(int x, int y) {
    DetailAction action = detailScreenHandleTouch(x, y);
    if (action == DetailAction::NONE) {
        return;
    }
    if (action == DetailAction::SAY) {
        soundReadAircraft(g_detailAc, true);
        return;
    }
    if (action == DetailAction::OPEN_FROM || action == DetailAction::OPEN_TO) {
        RouteJump j;
        j.ac = g_detailAc;
        j.returnTo = g_detailReturnTo;
        if (g_detailReturnTo == Screen::ZOOM) {
            j.priorZoom = g_zoomCode;
            j.priorViewNm = radarZoomViewNm();
        }
        j.savedCode = g_returnZoomCode;
        j.savedViewNm = g_returnZoomViewNm;
        soundStopReading();
        if (openZoom(detailScreenRouteCode(action == DetailAction::OPEN_FROM))) {
            j.active = true;
            g_jump = j;
        }
        return;
    }
    if (action != DetailAction::WATCH) {
        soundStopReading();  // leaving them: their details, if they are being read, are done with
    }
    if (action == DetailAction::FOLLOW) {
        followFromDetail();
        return;
    }
    // Into the replay's 3D view, of this one - from the replay screen, or
    // from that view showing another. Not to be had - no room for its bands,
    // or nothing of it at this moment - back as Back would go.
    if (action == DetailAction::VIEW_3D && open3dReplay(g_detailAc.hex, false)) {
        return;
    }
    if (action == DetailAction::WATCH) {
        // Said on the table's status line, for when it is back there; here
        // the button turning to UNWATCH says it.
        displayShowNotice(toggleWatchlist(watchableCallsign(g_detailAc.callsign)));
        openDetail(g_detailAc, g_detailReturnTo, false);  // drawn again for the button, not read again
        return;
    }
    if (action == DetailAction::UNFOLLOW) {
        stopFollow();  // and then back, as Back would go
    }
    if (g_detailReturnTo == Screen::VIEW3D && g_view3dReplay) {
        // Back into the replay's 3D view - or, not to be had, its screen.
        if (open3dReplay(g_replayHex, g_replayTracking)) {
            return;
        }
        g_view3dReplay = false;
        view3d::setReplay(nullptr);
        view3d::setDayPressure(dayPressureFt);
        g_detailReturnTo = Screen::PLAYBACK;
    }
    if (g_detailReturnTo == Screen::VIEW3D) {
        // Back into it, still following - or, not, to where it was opened.
        if (g_follow.active && view3d::open()) {
            g_screen = Screen::VIEW3D;
            return;
        }
        g_detailReturnTo = g_view3dReturnTo;
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

// Out of settings, with what was changed there saved and put to use - by
// its Back, or by setting a new location, which is what was gone in for.
void leaveSettings() {
    int radius = settingsScreenRadius();
    int interval = settingsScreenPollInterval();
    AdsbSource source = settingsScreenSource();
    g_showRefresh = settingsScreenShowRefresh();
    saveFilters(radius, g_showRefresh, interval, source);
    displaySetShowRefresh(g_showRefresh);
    if (settingsScreenNavaids() != g_navaids) {
        g_navaids = settingsScreenNavaids();
        saveNavaids(g_navaids);
        radarScreenSetNavaids(g_navaids);
    }
    if (settingsScreenCoast() != g_coast) {
        g_coast = settingsScreenCoast();
        saveCoast(g_coast);
        radarScreenSetCoast(g_coast);
    }
    if (settingsScreenRings() != g_rings) {
        g_rings = settingsScreenRings();
        saveRings(g_rings);
        radarScreenSetRings(g_rings);
    }
    if (settingsScreenAutoZoom() != g_autoZoom) {
        g_autoZoom = settingsScreenAutoZoom();
        saveAutoZoom(g_autoZoom);
    }
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        // A changed interval alone doesn't warrant refetching - the new
        // one simply applies to the wait already running.
        bool changed = (radius != g_radiusNm) || (source != g_source);
        if (radius != g_radiusNm) {
            g_sceneChanged = true;  // the source doesn't alter what a recording's plot is drawn around
        }
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
}

void handleLocationTouch(int x, int y) {
    double lat, lon;
    String label;
    LocationAction action = locationScreenHandleTouch(x, y, lat, lon, label);
    if (action == LocationAction::NONE) {
        return;
    }
    // Reached from the settings screen. Back goes back there; setting a
    // location goes on out of it as well, to the traffic around the new one,
    // rather than leave a second Back to press.
    if (action == LocationAction::LOCATION_SET) {
        saveLocation(lat, lon, label);
        g_airportCode = nearestAirportCode(lat, lon, 120.0f, false);
        displaySetHeader(g_airportCode, activeProvider());
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
        radarScreenSetClock(clockZoneShown());  // home's zone, unless one was picked
        leaveSettings();
        return;
    }
    g_screen = Screen::SETTINGS;
    settingsScreenDraw();
}

void handleSettingsTouch(int x, int y, bool pressed, bool clicked) {
    SettingsAction action = settingsScreenHandleTouch(x, y, pressed, clicked);
    switch (action) {
    case SettingsAction::BACK:
        leaveSettings();
        break;
    case SettingsAction::OPEN_LOCATION:
        locationScreenReset();
        g_screen = Screen::LOCATION;
        locationScreenDraw();
        break;
    case SettingsAction::OPEN_ALERTS:
        alertsScreenSet(g_alertMask, g_autoRecord, g_autoFollow, g_followIn3d, g_watchlist);
        alertsScreenSetVoice(g_readAloud, g_elsewhereAccent);
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
    case SettingsAction::OPEN_DISPLAY: {
        AppSettings s = loadSettings();
        displayPageSet(g_brightnessPct, g_dimMinutes, clockZoneShown(), g_replayTracks);
        g_screen = Screen::DISPLAY_PAGE;
        displayPageDraw();
        break;
    }
    case SettingsAction::OPEN_HELP:
    case SettingsAction::OPEN_ABOUT:
        // Settings keeps what has been changed on it meanwhile, for its Back.
        g_screen = Screen::INFO;
        infoScreenOpen(action == SettingsAction::OPEN_HELP ? InfoPage::HELP : InfoPage::ABOUT);
        break;
    case SettingsAction::NONE:
        break;
    }
}

// The filter screen, from the table's funnel or the radar's - its Back going
// back to whichever.
Screen g_filterReturnTo = Screen::MAIN;

void openFilter(Screen returnTo) {
    g_filterReturnTo = returnTo;
    filterScreenOpen(trafficFilter());
    g_screen = Screen::FILTER;
    filterScreenDraw();
}

// The filter put in force: for the table and the radar, and for the poll,
// which keeps what passes it beyond the nearest - see pollTask().
void setFilterInForce(const TrafficFilterSet &f) {
    setTrafficFilter(f);
    // At boot this comes before the mutex, and the poll task, exist.
    if (g_dataMutex == nullptr) {
        g_pollFilter = trafficFilter();
    } else if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_pollFilter = trafficFilter();
        xSemaphoreGive(g_dataMutex);
    }
}

// Back to the table, the filter put in force and kept.
void handleFilterTouch(int x, int y, bool pressed, bool clicked) {
    if (filterScreenHandleTouch(x, y, pressed, clicked) != FilterAction::BACK) {
        return;
    }
    TrafficFilterSet f = filterScreenResult();
    bool wasOn = trafficFilter().active();
    setFilterInForce(f);
    saveTrafficFilter(f);
    // Civil traffic's range back down to its own ceiling, with the filter
    // that took it past it gone - as switching from military does.
    if (g_radiusNm > maxRadiusNow()) {
        setPollRadius(maxRadiusNow());
    } else if (f.active() != wasOn) {
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_pollNow = true;  // the tail kept, or not, from the next poll on: now
            xSemaphoreGive(g_dataMutex);
        }
    }
    if (g_filterReturnTo == Screen::RADAR) {
        g_screen = Screen::RADAR;
        drawRadar(true);
        return;
    }
    g_screen = Screen::MAIN;
    displayInvalidate();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_dataReady = true;  // drawn again, through the filter as it now is
        xSemaphoreGive(g_dataMutex);
    }
}

void handleDisplayPageTouch(int x, int y, bool pressed, bool clicked) {
    if (displayPageHandleTouch(x, y, pressed, clicked) != DisplayPageAction::BACK) {
        return;
    }
    g_brightnessPct = displayPageBrightness();
    g_dimMinutes = displayPageDimMinutes();
    g_replayTracks = displayPageReplayTracks();
    playbackScreenSetTracks(g_replayTracks);
    // Home's zone, left as it was, is still home's - to follow it to a new
    // location - until another is picked.
    if (displayPageClock() != clockZoneShown()) {
        g_clockSetting = (uint8_t)displayPageClock();
    }
    saveDisplay(g_brightnessPct, g_dimMinutes, g_clockSetting, g_replayTracks);
    radarScreenSetClock(clockZoneShown());
    g_screen = Screen::SETTINGS;
    settingsScreenDraw();
}

void handleInfoTouch(int x, int y) {
    if (infoScreenHandleTouch(x, y) == InfoAction::BACK) {
        g_screen = Screen::SETTINGS;
        settingsScreenDraw();
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

// The chip's own memory - fast, and all the 3D view's drawing and the
// WiFi's buffers want it - kept for what needs it: TLS's buffers, 40KB a
// connection, which the framework puts there, and anything over 1KB,
// allocated in the PSRAM instead, as there is 32MB of it.
void *tlsCalloc(size_t n, size_t size) {
    void *p = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p != nullptr ? p : heap_caps_calloc(n, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
void tlsFree(void *p) { heap_caps_free(p); }

void setup() {
    mbedtls_platform_set_calloc_free(tlsCalloc, tlsFree);
    heap_caps_malloc_extmem_enable(1024);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(SERIAL_TX_TIMEOUT_MS);
    delay(300);
    Serial.printf("[boot] Overhead v%s\n", FIRMWARE_VERSION);
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
    coastBegin();  // the map's thinned copies, built in the background on core 0

    AppSettings s = loadSettings();
    g_lat = s.lat;
    g_lon = s.lon;
    g_label = s.label;
    g_radiusNm = s.radiusNm;
    g_showRefresh = s.showRefresh;
    g_navaids = s.navaids;
    g_coast = s.coast;
    g_rings = s.rings;
    g_pollIntervalS = s.pollIntervalS;
    g_muted = s.muted;
    g_source = s.source;
    g_radarCentre = s.radarCentre;
    g_radarAirports = s.radarAirports;
    g_radarRangeNm = s.radarRangeNm;
    g_returnZoomCode = s.zoomCode;
    g_returnZoomViewNm = s.zoomViewNm;
    g_alertMask = s.alertMask;
    g_watchlist = s.watchlist;
    g_autoRecord = s.autoRecord;
    g_autoFollow = s.autoFollow;
    g_followIn3d = s.followIn3d;
    g_readAloud = s.readAloud;
    g_elsewhereAccent = s.elsewhereAccent;
    g_alertRules.enabled = g_alertMask;
    g_alertRules.watch = parseWatchlist(g_watchlist);
    soundSetMuted(g_muted);
    radarScreenSetMuted(g_muted, false);
    view3d::setTerrain(s.terrain);
    soundSetReadAloud(g_readAloud);
    soundSetElsewhereAccent(g_elsewhereAccent);
    displaySetSort(s.tableSortCol, s.tableSortDesc);
    if (s.brightnessPct) {
        g_brightnessPct = s.brightnessPct;
        displayApplyBrightness(g_brightnessPct);
    } else {
        g_brightnessPct = (uint8_t)std::max(10, M5.Display.getBrightness() * 100 / 255);
    }
    g_dimMinutes = s.dimMinutes;
    g_autoZoom = s.autoZoom;
    g_replayTracks = s.replayTracks;
    playbackScreenSetTracks(g_replayTracks);
    g_clockSetting = s.clockZone;
    radarScreenSetClock(clockZoneShown());
    setFilterInForce(loadTrafficFilter());
    g_lastTouchMs = millis();
    displaySetShowRefresh(g_showRefresh);
    displaySetMuted(g_muted);
    radarScreenSetNavaids(g_navaids);
    settingsScreenSetNavaids(g_navaids);
    radarScreenSetCoast(g_coast);
    settingsScreenSetCoast(g_coast);
    radarScreenSetRings(g_rings);
    settingsScreenSetRings(g_rings);
    g_airportCode = nearestAirportCode(g_lat, g_lon, 120.0f, false);
    displaySetHeader(g_airportCode, g_activeProvider);
    screen::setOverlay(drawBanner, 0, 0, 1280, BANNER_H);

    recorder::setRunwaysInUseSource(radarLiveRunwaysInUse);
    // Mounted now so the log says whether there is a card; recording mounts
    // it again later if one is put in after boot.
    if (!recorder::mount()) {
        Serial.println("[sd] no card - recording unavailable until one is inserted");
    }
    // And the airfield pack on it, for the 3D view's airports: what there is,
    // or that they will be the basic view, their runways alone.
    {
        uint16_t airfields = 0;
        uint32_t bytes = 0;
        airfield::Missing why;
        if (airfield::packInfo(airfields, bytes, why)) {
            Serial.printf("[airfield] the card's airfields.bin: %u airfields, %.1fMB\n", (unsigned)airfields,
                          bytes / 1048576.0f);
        } else {
            Serial.printf("[airfield] %s: the 3D view's airports will be the basic view, their runways alone\n",
                          why == airfield::Missing::NO_CARD   ? "no SD card"
                          : why == airfield::Missing::NO_PACK ? "no airfields.bin on the card"
                                                               : "the card's airfields.bin is for another version");
        }
    }
    // And the ground pack, for the 3D view's land and hills: what there is -
    // its header and index checked, as the view will - or why its land will
    // be flat. Read in only once the view first opens.
    groundpack::logCard();

    // What the device actually loaded, said once at boot. An empty table has
    // several innocent explanations - a filter down to the military over quiet
    // airspace being the likeliest - and none of them were visible from the
    // log, which made "nothing is showing" impossible to tell from "nothing
    // is working".
    Serial.printf("[settings] %.4f,%.4f  %dnm  every %ds  source=%s  filter %s\n", g_lat, g_lon, g_radiusNm,
                  g_pollIntervalS,
                  adsbSourceIndex(g_source) < 0 ? "auto" : ADSB_PROVIDERS[adsbSourceIndex(g_source)].name,
                  trafficFilter().active() ? filterText(trafficFilter()).c_str() : "off");

    // Credentials set on-screen win; secrets.h is the fallback for a device
    // that's never had WiFi configured through the settings screen. Kept so
    // the poll task can reconnect with them without re-reading NVS each time.
    g_wifiSsid = s.wifiSsid.length() ? s.wifiSsid : String(WIFI_SSID);
    g_wifiPass = s.wifiSsid.length() ? s.wifiPass : String(WIFI_PASSWORD);
    bool haveCredentials = !credentialsUnset(g_wifiSsid);

    // The network is joined while the splash screen plays, and waited on
    // after it for what is left of the timeout, the splash saying so. With
    // nothing to connect with, the timeout isn't spent finding out; the WiFi
    // screen scans, which needs the radio in station mode either way.
    WiFi.mode(WIFI_STA);
    uint32_t wifiStart = millis();
    if (haveCredentials) {
        WiFi.begin(g_wifiSsid.c_str(), g_wifiPass.c_str());
    }
    soundTakeoff();
    splashPlay();
    if (haveCredentials) {
        if (WiFi.status() != WL_CONNECTED) {
            splashStatus("Connecting to " + g_wifiSsid + "...");
        }
        while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_BOOT_TIMEOUT_MS) {
            delay(100);
        }
        Serial.printf("[wifi] %s\n",
                      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "FAILED to connect");
        if (WiFi.status() != WL_CONNECTED) {
            g_wifiFails = 1;  // the first of the attempts before the WiFi screen comes up
        }
    } else {
        Serial.println("[wifi] no credentials set - opening the WiFi screen");
    }

    std::vector<Aircraft> none;
    std::vector<uint8_t> noneNew;
    displayRenderAircraft(none, noneNew);
    displaySetPollState(false, false, false, 0);
    displayTickStatus();
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
    xTaskCreatePinnedToCore(pollTask, "poll", 16384, nullptr, 1, &g_pollTask, 0);
    // Above everything else, to be sure of running whatever is spinning;
    // asleep all but a moment a second.
    xTaskCreate(hangWatch, "hangwatch", 3072, nullptr, configMAX_PRIORITIES - 2, nullptr);

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
    // Then, after what the line has always said - the same, for anything
    // reading it - where the rest of the 2D screens' time goes: their
    // preparation step by step, in us a frame; the PPA's blits and fills, how
    // many and the ms waited on them in the minute, and the cache written back
    // for them; a whole plot's layers; a replay's frames; the table's renders
    // and its shading lapsing; the full draws, and whether the longest pass
    // held one.
    screen::PpaPerf ppa = screen::takePpaPerf();
    String more = "; prep us radar " + g_perfPrepRadar.text(true) + ", zoom " + g_perfPrepZoom.text(false) +
                  "; PPA blits " + String(ppa.blits) + "x " + String((uint32_t)(ppa.blitUs / 1000)) + "ms " +
                  String((uint32_t)(ppa.blitPixels / 1000)) + "kpx, fills " + String(ppa.fills) + "x " +
                  String((uint32_t)(ppa.fillUs / 1000)) + "ms, syncs " + String((uint32_t)(ppa.syncUs / 1000)) +
                  "ms; layers " + radarTakeLayers() + "; replay " + g_perfReplay.text() + "; table " +
                  g_perfTable.text() + ", lapses " + g_perfLapse.text() + "; full draws " + g_perfFull.text() +
                  ", the longest pass " + (g_perfLoop.count && g_longestPassFull ? "held one" : "held none");
    // And the stacks' high-water marks - the least each has had spare - for
    // them to be sized by what they use.
    Serial.printf("[perf] radar %s, zoom %s (plots: %s), tracks %s (%u), loop max %ums; internal RAM %uKB free, "
                  "%uKB lowest, %uKB largest block; PSRAM %uKB free; stacks spare: loop %u, poll %u, sound %u%s\n",
                  g_perfRadar.text().c_str(), g_perfZoom.text().c_str(), radarTakeTimings().c_str(),
                  g_perfTracks.text().c_str(),
                  (unsigned)g_tracks.size(), (unsigned)(g_perfLoop.maxUs / 1000),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                  (unsigned)uxTaskGetStackHighWaterMark(nullptr),
                  g_pollTask ? (unsigned)uxTaskGetStackHighWaterMark(g_pollTask) : 0u, soundStackSpare(),
                  more.c_str());
    g_perfRadar = g_perfZoom = g_perfTracks = g_perfLoop = PerfStat();
    g_perfReplay = g_perfTable = g_perfLapse = g_perfFull = PerfStat();
    g_perfPrepRadar = g_perfPrepZoom = PrepStat();
    g_longestPassFull = false;
}

// For the serial tests: the airport last said everything at.
const char *g_testCode = "GLA";

// At airport `code`, an alert, a landing on `runway` by `callsign`, and its
// details - in that place's voice, one after another.
void sayEverything(const char *code, const char *callsign, const char *runway) {
    double lat, lon;
    float rangeNm;
    if (!radarZoomFrame(code, lat, lon, rangeNm)) {
        return;
    }
    g_testCode = code;
    soundAlert(ALERT_RARE, callsign, "Boeing 747-400", lat, lon);
    soundAnnounce(callsign, runway, false, code);
    Aircraft ac;
    ac.callsign = callsign;
    ac.type = "B744";
    ac.altStr = "4375";
    ac.speedStr = "175";
    ac.status = "DESCEND";
    ac.hasTrack = true;
    ac.track = 240;
    ac.hasPos = true;
    ac.lat = lat;
    ac.lon = lon;
    soundReadAircraft(ac, true);
}

#ifdef BENCH3D
// Scenario 18's: the settings auto-follow had, while it forces them on.
bool g_benchForcedFollow = false, g_benchAutoFollowWas = false, g_benchIn3dWas = false;

// Scenario `n` made up and followed, its 3D view opened.
void startSoak(int n) {
    MadeUp m;
    bool ok = true;
    switch (n) {
    case 0: m = circling("aa0010", "TYPHON1", "EUFI", "A6", true, 57.7052, -3.3392, 2500, 280, 1800); break;
    case 1: m = circling("aa0011", "LIGHTN1", "F35", "A6", true, 57.9, -1.5, 9000, 420, 24000); break;
    case 2: m = circling("aa0012", "HAWK01", "HAWK", "A6", true, 54.45, -3.05, 3000, 330, 1200); break;
    case 3: m = circling("aa0013", "CHINOK1", "H47", "A7", true, 51.2341, -0.9428, 1500, 120, 1000); break;
    case 4:
        m = circling("aa0014", "POLICE1", "EC35", "A7", false, 55.8609, -4.2514, 0, 0, 900);
        m.pattern = Pattern::HOVER;
        break;
    case 5: m = circling("aa0015", "HELIMED5", "EC45", "A7", false, 55.9214, -3.1347, 700, 90, 800); break;
    case 6: m = circling("aa0016", "ATLAS01", "A400", "A5", true, 56.5, -4.0, 12000, 250, 16000); break;
    case 7:
        m = circling("aa0017", "TEST320", "A320", "A3", false, 0, 0, 0, 0, 0);
        m.pattern = Pattern::LAND;
        m.quietAfterS = LOST_ON_ROLLOUT_S;
        ok = onRunwayAt(m, "LHR", "27L");
        break;
    case 8:
        m = circling("aa0018", "TEST738", "B738", "A3", false, 0, 0, 0, 0, 0);
        m.pattern = Pattern::DEPART;
        ok = onRunwayAt(m, "EDI", "24");
        break;
    case 9:
        m = circling("aa0019", "RCH001", "C17", "A5", true, 0, 0, 0, 0, 0);
        m.pattern = Pattern::LAND;
        m.quietAfterS = LOST_ON_ROLLOUT_S;
        ok = onRunwayAt(m, "BZZ", "25");
        break;
    case 10:
        // A high day's pressure: its pressure altitude 270ft under its
        // height, and under the ground once down.
        m = circling("aa001a", "HELIMED1", "EC45", "A7", false, 0, 0, 0, 0, 0);
        m.baroOverFt = -270;
        m.geom = true;
        ok = spotAt(m, "GLA", "23", 1300, -250, 110);
        break;
    case 11:
        // Its pressure altitude as the day's pressure has it - the aircraft
        // about having it a little wrong, by 40ft; its transponder off 40
        // seconds after it is down.
        m = circling("aa001b", "RESCUE9", "A139", "A7", false, 0, 0, 0, 0, 0);
        m.dayPressure = true;
        m.baroOverFt = 40;
        m.saysGround = true;
        m.quietAfterS = 2 * 2600 / 46.0f + 12 + 40;
        ok = spotAt(m, "EDI", "24", 1200, -300, 200);
        break;
    // Not soaked as the rest are, but for pictures: the biggest, in their
    // airlines' colours.
    case 12:
        m = circling("aa001c", "UAE5", "A388", "A5", false, 0, 0, 0, 0, 0);
        m.pattern = Pattern::LAND;
        m.quietAfterS = LOST_ON_ROLLOUT_S;
        ok = onRunwayAt(m, "LHR", "27L");
        break;
    case 13:
        m = circling("aa001d", "BAW117", "B77W", "A5", false, 0, 0, 0, 0, 0);
        m.pattern = Pattern::DEPART;
        ok = onRunwayAt(m, "LHR", "27R");
        break;
    // Tonight's fixes, 20 October: no landing begun by a run in, nor by an
    // overhead join; a landing's nose up whatever the day's pressure; a
    // pressure altitude under nought kept; and no banner left round the 3D
    // view an alert's auto-follow opened.
    case 14:
        m = circling("aa001e", "TARTN11", "EUFI", "A6", true, 0, 0, 0, 0, 0);
        m.pattern = Pattern::CIRCUIT;
        ok = onRunwayAt(m, "LMO", "23");
        m.speedKt = 400, m.altFt = 1000, m.runInM = 9000, m.radiusM = 1100;
        m.circuitKt = 230, m.circuitFt = 1000, m.finalM = 1800, m.finalKt = 150;
        m.dayPressure = true;
        break;
    case 15:
        m = circling("aa001f", "GOVHD", "C172", "A1", false, 0, 0, 0, 0, 0);
        m.pattern = Pattern::CIRCUIT;
        ok = onRunwayAt(m, "GLA", "23");
        m.speedKt = 90, m.altFt = 2000, m.runInM = 1500, m.radiusM = 500;
        m.circuitKt = 85, m.circuitFt = 1000, m.finalM = 1200, m.finalKt = 65;
        m.dayPressure = true;
        break;
    case 16:
    case 18:
        m = circling(n == 16 ? "aa0020" : "aa0022", n == 16 ? "EZY77LP" : "EZY99AL", "A320", "A3", false, 0, 0, 0, 0,
                     0);
        m.pattern = Pattern::LAND;
        m.quietAfterS = LOST_ON_ROLLOUT_S;
        m.dayPressure = true;
        m.baroOverFt = 40;  // and the day's pressure a little out for it
        ok = onRunwayAt(m, "GLA", "23");
        break;
    case 19:
        // A spy plane at 65,000ft - one was seen over the UK in October - a
        // wide circle over Fairford, where they fly from, at its 410kt.
        m = circling("aa0023", "DRAGN71", "U2", "A2", true, 51.682, -1.790, 25000, 410, 65000);
        break;
    // The fixes of 23 October: hills no longer laid across the land seen
    // from above them; a fast jet's sound arriving all at once as it passes,
    // its engines working as its speed needs; a take-off rotated; an APU
    // run only where an airliner runs it; trails continuous and grey.
    case 20: m = circling("aa0024", "VANDL22", "EUFI", "A6", true, 55.48, -2.15, 9000, 480, 10000); break;
    case 21:
        m = circling("aa0025", "CFE12ZF", "E190", "A3", false, 0, 0, 0, 0, 0);
        m.pattern = Pattern::DEPART;
        ok = onRunwayAt(m, "GLA", "23");
        break;
    case 22: m = circling("aa0026", "UPS208", "B763", "A5", false, 55.4, -3.6, 20000, 410, 30000); break;
    // A turboprop, for its engines' sound - synthesised and recorded, side
    // by side with "##e": its approach and reverse, and its full power.
    case 23:
    case 24:
        m = circling(n == 23 ? "aa0027" : "aa0028", n == 23 ? "LOG6KM" : "LOG8PW", "AT76", "A2", false, 0, 0, 0, 0, 0);
        m.pattern = n == 23 ? Pattern::LAND : Pattern::DEPART;
        if (n == 23) {
            m.quietAfterS = LOST_ON_ROLLOUT_S;
        }
        ok = onRunwayAt(m, "GLA", "23");
        break;
    // The title shot of 24 October: the land at its height where it is
    // highest - a ground pack with the Alps in it, made by
    // tools/make_ground_pack.py - a light aircraft 700m above the valley's
    // ridges and 650m under the Matterhorn's top, 5km off.
    case 25: m = circling("aa0029", "OVRHD", "C172", "A1", false, 46.020, 7.749, 2500, 100, 12500); break;
    case 17:
        m = circling("aa0021", "EZY88HP", "A320", "A3", false, 0, 0, 0, 0, 0);
        m.pattern = Pattern::LAND;
        m.quietAfterS = LOST_ON_ROLLOUT_S;
        m.baroOverFt = -500;
        m.geom = true;
        ok = onRunwayAt(m, "LMO", "23");
        break;
    default: ok = false; break;
    }
    if (!ok) {
        Serial.printf("[soak] no scenario %d\n", n);
        return;
    }
    m.active = true;
    m.startMs = millis();
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_madeUp = m;
        xSemaphoreGive(g_dataMutex);
    }
    Aircraft a = madeUpAt(m, millis());
    a.hasDist = true;
    a.distNm = haversineNm(g_lat, g_lon, a.lat, a.lon);
    Serial.printf("[soak] scenario %d: %s (%s)\n", n, SOAK_NAMES[n], m.callsign);
    if (g_screen == Screen::VIEW3D) {
        leave3d();
    }
    stopFollow();
    if (n == 18) {
        // Raised as an alert, from the radar - for auto-follow to take into
        // the 3D view, as set to whatever the settings say, until "##x".
        if (!g_benchForcedFollow) {
            g_benchAutoFollowWas = g_autoFollow;
            g_benchIn3dWas = g_followIn3d;
            g_benchForcedFollow = true;
        }
        g_autoFollow = g_followIn3d = true;
        g_screen = Screen::RADAR;
        drawRadar(true);
        a.alert = ALERT_EMERGENCY;
        a.squawk = "7700";
        if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
            g_pendingAlert = a;
            g_pendingAlertMore = 0;
            g_pendingFlash.clear();
            g_alertPending = true;
            xSemaphoreGive(g_dataMutex);
        }
        return;
    }
    startFollow(a);
    g_follow.sim.report(followSampleOf(a, millis()));
    g_screen = Screen::RADAR;
    drawRadar(true);
    open3d();
}

void stopSoak() {
    if (xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
        g_madeUp.active = false;
        xSemaphoreGive(g_dataMutex);
    }
    if (g_benchForcedFollow) {
        g_autoFollow = g_benchAutoFollowWas;
        g_followIn3d = g_benchIn3dWas;
        g_benchForcedFollow = false;
    }
    dismissBanner();
}

// A tap at (x, y), as a finger would make it on whatever screen is up: on
// the radar and the zoom as it lands, on the pages with sliders as it lands
// and lifts, elsewhere as a click.
void benchTap(int x, int y) {
    switch (g_screen) {
    case Screen::RADAR: handleRadarTouch(x, y); break;
    case Screen::ZOOM: handleZoomTouch(x, y); break;
    case Screen::VIEW3D: {
        String hex;
        act3d(view3d::touch(x, y, true), hex);
        if (g_screen == Screen::VIEW3D) {
            view3d::Action lifted = view3d::lift(hex);
            act3d(lifted, hex);
        }
        break;
    }
    case Screen::FILTER:
        handleFilterTouch(x, y, true, false);
        handleFilterTouch(x, y, false, true);
        break;
    case Screen::DISPLAY_PAGE:
        handleDisplayPageTouch(x, y, true, false);
        handleDisplayPageTouch(x, y, false, true);
        break;
    case Screen::SETTINGS:
        handleSettingsTouch(x, y, true, false);
        handleSettingsTouch(x, y, false, true);
        break;
    case Screen::MAIN: handleMainTouch(x, y); break;
    case Screen::LOCATION: handleLocationTouch(x, y); break;
    case Screen::DETAIL: handleDetailTouch(x, y); break;
    case Screen::WIFI: handleWifiTouch(x, y); break;
    case Screen::ALERTS: handleAlertsTouch(x, y); break;
    case Screen::WATCHLIST: handleWatchlistTouch(x, y); break;
    case Screen::RECORDINGS: handleRecordingsTouch(x, y); break;
    case Screen::PLAYBACK: handlePlaybackTouch(x, y); break;
    case Screen::SHARE: handleShareTouch(x, y); break;
    case Screen::INFO: handleInfoTouch(x, y); break;
    default: break;
    }
}

// The newest recording on the card that follows an aircraft - or "" - for
// the soak's replay in 3D.
String newestFollowingRecording() {
    for (const String &path : recorder::list()) {
        recorder::Header h;
        uint32_t bytes;
        if (recorder::readHeader(path, h, bytes) && h.followHex.length()) {
            return path;
        }
    }
    return String();
}

// A file onto the card, from tools/sd_upload.py: its name, size and CRC-32 on
// a line, then the bytes in blocks of up to 4KB, each answered "UPLOAD n" with
// how many have come, so the PC never sends more than the USB's buffer holds;
// at the end "UPLOAD DONE" if the CRC agrees - written first to a .part and
// renamed, so a broken upload leaves the old file as it was.
void uploadToCard() {
    char line[96];
    size_t n = 0;
    uint32_t until = millis() + 3000;
    while (n + 1 < sizeof line && (int32_t)(millis() - until) < 0) {
        int c = Serial.read();
        if (c < 0) {
            delay(1);
            continue;
        }
        if (c == '\n') {
            break;
        }
        line[n++] = (char)c;
    }
    line[n] = 0;
    char name[32] = {};
    unsigned long size = 0, crc = 0;
    // The airfield pack, the ground pack, or a reply from the feed for
    // bench2d.h's "##=p".
    if (sscanf(line, "%31s %lu %lx", name, &size, &crc) != 3 ||
        (strcmp(name, "airfields.bin") != 0 && strcmp(name, "ground.bin") != 0 && strncmp(name, "bench-", 6) != 0) ||
        size == 0 || size > 64UL * 1024 * 1024) {
        Serial.printf("UPLOAD REFUSED %s\n", line);
        return;
    }
    if (!recorder::mount()) {
        Serial.println("UPLOAD REFUSED no card");
        return;
    }
    // Not the ground pack the 3D view is reading tiles from: hidden first
    // ("##s1"), before the view has opened since the restart.
    groundpack::Status ground = groundpack::status();
    if (strcmp(name, "ground.bin") == 0 &&
        (ground == groundpack::Status::LOADING || ground == groundpack::Status::READY)) {
        Serial.println("UPLOAD REFUSED the 3D view has ground.bin open - restart, then ##s1, then upload");
        return;
    }
    recorder::CardLock lock;
    String part = String("/overhead/") + name + ".part", whole = String("/overhead/") + name;
    File f = SD.open(part, FILE_WRITE);
    if (!f) {
        Serial.println("UPLOAD REFUSED can't write the card");
        return;
    }
    static uint8_t block[4096];
    uint32_t got = 0, sum = 0xFFFFFFFF;
    Serial.println("UPLOAD READY");
    g_loopBusyLong = true;
    while (got < size) {
        size_t want = std::min<size_t>(sizeof block, size - got), have = 0;
        until = millis() + 5000;
        while (have < want && (int32_t)(millis() - until) < 0) {
            int c = Serial.available();
            if (c <= 0) {
                delay(1);
                continue;
            }
            have += Serial.readBytes(block + have, std::min<size_t>(want - have, (size_t)c));
        }
        if (have < want || f.write(block, want) != want) {
            break;
        }
        for (size_t i = 0; i < want; i++) {
            sum ^= block[i];
            for (int k = 0; k < 8; k++) {
                sum = (sum >> 1) ^ (0xEDB88320u & -(sum & 1));
            }
        }
        got += want;
        Serial.printf("UPLOAD %u\n", (unsigned)got);
    }
    f.close();
    g_loopBusyLong = false;
    sum = ~sum;
    if (got != size || sum != crc) {
        SD.remove(part);
        Serial.printf("UPLOAD FAILED %u of %lu bytes, crc %08x\n", (unsigned)got, size, (unsigned)sum);
        return;
    }
    SD.remove(whole);
    SD.rename(part, whole);
    Serial.printf("UPLOAD DONE %s %lu bytes\n", name, size);
}
#endif

}  // namespace


void loop() {
    uint32_t loopStart = micros();
    g_loopPasses = g_loopPasses + 1;
    g_loopAt = "the touch read";
    // The touch chip is read over I2C, a quarter of a millisecond a time:
    // every pass while a finger is down, so a drag keeps up with it, and
    // every other pass while nothing was, so a touch landing is seen within
    // 20ms - where reading it a hundred times a second with nothing on the
    // glass was 2% of core 1, all the time. Only a read that saw no touch is
    // skipped after, so nothing a touch does is ever seen twice.
    static bool touchSeen = false, readLastPass = false;
    if (touchSeen || !readLastPass) {
        M5.update();
        touchSeen = M5.Touch.getCount() > 0;
        readLastPass = true;
    } else {
        readLastPass = false;
    }

    // "##S" over serial sends the screen back, for tools/screenshot.py;
    // "##E", "##W", "##R" and "##M" sound an alert of each kind, for a
    // made-up callsign at home, to hear what it says; "##T" and "##L" the
    // zoom's take-off and landing announcements at home's airport. "##C",
    // "##G" and "##U" say an alert, a landing and an aircraft's details at
    // Glasgow, Heathrow and JFK - in the Scottish, English and American
    // voices - and "##N" and "##A" read the card of the navaid nearest the
    // last of those, and of its airport. Behind "##"
    // rather than a bare letter: a computer opening the port echoes back the
    // log lines it catches for a moment, and their letters ran the tests - a
    // warble and a screenshot each time the port was opened.
    static int hashes = 0;
    g_loopAt = "the serial commands";
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '#') {
            hashes++;
            continue;
        }
        bool armed = hashes >= 2;
        hashes = 0;
        if (!armed) {
            continue;
        }
        switch (c) {
        case 'S':
            screen::dumpToSerial();
            break;
#ifdef BENCH3D
        case 'D':
            // "##D" the 3D view's camera and its skyline as drawn.
            view3d::dumpSkyline();
            break;
        case 'm':
            // "##m" the land at its height, or flat with a skyline.
            view3d::toggleTerrain();
            break;
        case 'i': {
            // "##i" and a digit: the terrain's memory probe - see
            // scene3d::terrainProbe().
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            if (arg == 'T' || arg == 'M') {
                // "##iT" what the terrain costs each core, on its own: see
                // bench3d's 'T'; "##iM" what the P4's loads do on a miss.
                g_loopBusyLong = true;
                bench3dCommand((char)arg, repaintCurrentScreen);
                g_loopBusyLong = false;
                break;
            }
            view3d::terrainProbe(arg >= '0' && arg <= '3' ? arg - '0' : 0);
            break;
        }
#endif
        case 'E':
            soundAlert(ALERT_EMERGENCY, "EZY12AB", "Airbus A320neo", g_lat, g_lon);
            break;
        case 'W':
            soundAlert(ALERT_WATCHLIST, "GSGTS", "Boeing 787-9 Dreamliner", g_lat, g_lon);
            break;
        case 'R':
            soundAlert(ALERT_RARE, "N747NA", "Airbus A380-800", g_lat, g_lon);
            break;
        case 'M':
            soundAlert(ALERT_MILITARY, "RCH21", "Lockheed Martin F-35 Lightning II", g_lat, g_lon);
            break;
        case 'T':
            soundAnnounce("EZY53TH", "23", true, g_airportCode.c_str());
            break;
        case 'L':
            soundAnnounce("BAW27K", "27L", false, g_airportCode.c_str());
            break;
        case 'C':
            sayEverything("GLA", "EZY53TH", "23");
            break;
        case 'G':
            sayEverything("LHR", "BAW27K", "27L");
            break;
        case 'U':
            sayEverything("JFK", "N787RX", "22R");
            break;
        case 'N': {
            double lat, lon;
            float rangeNm;
            if (radarZoomFrame(g_testCode, lat, lon, rangeNm)) {
                const std::vector<const Navaid *> &near = navaidsWithin(lat, lon, 40);
                if (!near.empty()) {
                    soundReadNavaid(*near.front(), true);
                }
            }
            break;
        }
        case 'A':
            if (const AirportInfo *info = airportInfo(g_testCode)) {
                soundReadAirport(*info, true);
            }
            break;
#ifdef BENCH3D
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
        case '1':
        case 'h':
        case 'n':
        case 'l':
        case 'c':
        case 'f':
        case 'q':
            g_loopBusyLong = true;  // minutes, some of them
            bench3dCommand((char)c, repaintCurrentScreen);
            g_loopBusyLong = false;
            break;
        case '_':
            // "##_" the 3D view's land in its cover, from the ground pack,
            // or one colour, as it was - each time the other.
            Serial.printf("[ground] the land's cover %s\n", view3d::benchToggleCover() ? "on" : "off");
            break;
        case 'O':
            // "##O" where the cover's time has gone since the last "##O".
            view3d::benchCoverReport();
            break;
        case '=': {
            // "##=" and a letter: the rest of the app timed, a part at a
            // time - see bench2d.h.
            int which = -1;
            uint32_t until = millis() + 1000;
            while (which < 0 && (int32_t)(millis() - until) < 0) {
                which = Serial.read();
                if (which < 0) {
                    delay(2);
                }
            }
            g_loopBusyLong = true;
            if (which == 'k') {
                benchTracks();
                Serial.println("[bench] done");
            } else {
                bench2dCommand((char)which, repaintCurrentScreen);
            }
            g_loopBusyLong = false;
            break;
        }
        case 'o':
            // "##o" the 3D view's orbit camera swept every way for 45
            // seconds, as a finger might turn it.
            if (g_screen == Screen::VIEW3D) {
                view3d::sweep(45);
            }
            break;
        case 'v':
        case 'w':
        case 'p':
        case 'g': {
            // Following the nearest aircraft in the air, its 3D view: a test
            // of it with no finger on the glass - and, with no traffic to
            // follow, a made-up A320 at 10,000ft over home. "##w" one on a
            // four-mile final to home's airport instead, to land. "##V" back
            // out of it.
            Aircraft pick;
            bool found = false;
            // "##g" the nearest on the ground.
            if ((c == 'v' || c == 'g') && xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                float best = 1e9f;
                for (const Aircraft &a : g_latestAircraft) {
                    bool want = c == 'v' ? a.altStr != "GND" : a.altStr == "GND";
                    if (a.hasPos && want && a.hasDist && a.distNm < best) {
                        best = a.distNm;
                        pick = a;
                        found = true;
                    }
                }
                xSemaphoreGive(g_dataMutex);
            }
            if (!found) {
                pick.hex = "aa0001";
                pick.callsign = "TEST01";
                pick.type = "A320";
                pick.category = "A3";
                pick.status = "LEVEL";
                pick.hasPos = pick.hasTrack = pick.hasVertRate = true;
                pick.lat = (float)g_lat + 0.1f;
                pick.lon = (float)g_lon;
                pick.track = 270;
                pick.altStr = "10000";
                pick.speedStr = "300";
                pick.vertRate = 0;
                if (c == 'p') {
                    // "##p" a made-up Cessna low over the Firth of Clyde, off
                    // Bute and Arran - coasts all round, and no airport near
                    // enough for the airfield view: the country one, low.
                    pick.hex = "aa0002";
                    pick.callsign = "TEST02";
                    pick.type = "C172";
                    pick.category = "A1";
                    pick.lat = 55.75f;
                    pick.lon = -5.05f;
                    pick.track = 200;
                    pick.altStr = "1500";
                    pick.speedStr = "100";
                }
                int count = 0;
                const Runway *r = c == 'w' ? runwaysAt(g_airportCode.c_str(), count) : nullptr;
                if (r != nullptr && count > 0) {
                    // Four miles out from the first end of its first runway, on
                    // its centreline, at 1,300ft above it, coming down.
                    double dn = (r->end[1].lat - r->end[0].lat) * 60, de = (r->end[1].lon - r->end[0].lon) * 60 *
                                                                           cos(r->end[0].lat * M_PI / 180);
                    double len = sqrt(dn * dn + de * de);
                    pick.lat = (float)(r->end[0].lat - 4 * dn / len / 60);
                    pick.lon = (float)(r->end[0].lon - 4 * de / len / (60 * cos(r->end[0].lat * M_PI / 180)));
                    pick.track = (float)(fmod(atan2(de, dn) * 180 / M_PI + 360, 360));
                    pick.altStr = String(r->elevationFt + 1300);
                    pick.speedStr = "140";
                    pick.vertRate = -700;
                    pick.status = "DESCEND";
                }
            }
            Serial.printf("[3d] following %s for the 3D view\n", pick.callsign.c_str());
            if (g_screen == Screen::VIEW3D) {
                leave3d();
            }
            stopFollow();
            startFollow(pick);
            g_follow.sim.report(followSampleOf(pick, millis()));
            g_screen = Screen::RADAR;
            drawRadar(true);
            open3d();
            break;
        }
        case 'V':
            if (g_screen == Screen::VIEW3D) {
                leave3d();
            }
            break;
        // The soak test's, for tools/soak3d.py - what a finger and the sky
        // would do, by serial: "##k" and a digit - or a letter, from 10 -
        // follows made-up flight n in 3D; "##x" stops it; "##K" and
        // c/h/o/t/+/- presses that camera button; "##d" taps an aircraft in
        // the 3D view, for its details; "##B" and "##F" press Back and
        // FOLLOW there; "##2" is the radar's 3D button.
        case 'k':
        case 'K': {
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            if (c == 'k') {
                startSoak(arg >= 'a' ? 10 + arg - 'a' : arg - '0');
            } else {
                view3d::benchPress((char)arg);
            }
            break;
        }
        case 'x':
            stopSoak();
            if (g_screen == Screen::VIEW3D) {
                leave3d();
            }
            stopFollow();
            Serial.println("[soak] stopped");
            break;
        case 'H': {
            // The chip's own memory, region by region: how much each has free,
            // its largest piece and in how many - for what breaks it up.
            multi_heap_info_t all;
            heap_caps_get_info(&all, MALLOC_CAP_INTERNAL);
            Serial.printf("[heap] internal: %uKB free in %u pieces, the largest %uKB; %u blocks in use\n",
                          (unsigned)(all.total_free_bytes / 1024), (unsigned)all.free_blocks,
                          (unsigned)(all.largest_free_block / 1024), (unsigned)all.allocated_blocks);
            heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
            break;
        }
        case 'd': {
            String hex;
            Aircraft ac;
            if (g_screen == Screen::VIEW3D && view3d::benchTap(hex) &&
                (g_view3dReplay ? replayAircraft(hex, ac) : lastReported(hex, ac))) {
                Serial.printf("[soak] tapped %s in 3D\n", ac.callsign.c_str());
                view3d::close();
                openDetail(ac, Screen::VIEW3D);
            } else {
                Serial.println("[soak] nothing to tap in 3D");
            }
            break;
        }
        case 'B':
        case 'F':
            if (g_screen == Screen::DETAIL) {
                Serial.printf("[soak] details: %s\n", c == 'B' ? "Back" : "FOLLOW");
                handleDetailTouch(c == 'B' ? 1202 : 1066, 30);
            }
            break;
        case 'I': {
            // "##I" the screen round the 3D view looked over for the alert
            // banner's colours: none, the banner taken off as the view opened.
            auto &canvas = screen::canvas();
            int found = 0;
            for (int y = 0; y < 60; y++) {
                for (int x = 0; x < 1280; x++) {
                    if (y >= 16 && x >= 40 && x < 1048) {
                        continue;  // the picture: the panel's, not the canvas's
                    }
                    auto p = canvas.readPixelRGB(x, y);
                    bool amber = abs(p.r - 0xB9) <= 8 && abs(p.g - 0x6A) <= 4 && abs(p.b - 0x0A) <= 8;
                    bool red = abs(p.r - 0xC0) <= 8 && abs(p.g - 0x39) <= 4 && abs(p.b - 0x2B) <= 8;
                    found += amber || red;
                }
            }
            Serial.printf("[soak] banner check on %s: %d of its pixels round the view%s\n",
                          g_screen == Screen::VIEW3D ? "the 3D view" : "another screen", found,
                          found > 0 ? " - the banner left behind" : "");
            break;
        }
        case 'y': {
            // "##y" the newest recording that follows an aircraft, in the
            // replay screen at its start - for the soak's replay in 3D.
            String path = newestFollowingRecording();
            if (g_screen == Screen::VIEW3D) {
                leave3d();
            }
            if (path.length() && playbackScreenOpen(path)) {
                playbackScreenSetCentre(g_radarCentre);
                playbackScreenSetAirports(g_radarAirports);
                g_screen = Screen::PLAYBACK;
                playbackScreenDraw();
                Serial.printf("[soak] replaying %s\n", path.c_str());
            } else {
                Serial.println("[soak] no recording that follows an aircraft");
            }
            break;
        }
        case 'j': {
            // "##j" and a letter: the replay's - '3' its 3D, by its button;
            // in its 3D view 'p' Play or Pause, 'b' and 'f' a minute back and
            // on, 's' the speed, 'm' the middle of its bar, 'B' Back to it;
            // 'x' out of the replay to the recordings.
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            struct Spot {
                char key;
                int x, y;
            };
            static const Spot SPOTS[] = {{'p', 115, 649}, {'b', 248, 649}, {'s', 354, 649},
                                         {'f', 460, 649}, {'m', 790, 641}, {'B', 1189, 33}};
            if (arg == '3' && g_screen == Screen::PLAYBACK) {
                Serial.println("[soak] replay: 3D");
                handlePlaybackTouch(1202, 134);
            } else if (arg == 'x' && g_screen == Screen::PLAYBACK) {
                Serial.println("[soak] replay: Back");
                handlePlaybackTouch(1202, 30);
            } else if (g_screen == Screen::VIEW3D && g_view3dReplay) {
                for (const Spot &spot : SPOTS) {
                    if (spot.key == arg) {
                        Serial.printf("[soak] 3D replay: %c\n", arg);
                        String none;
                        act3d(view3d::touch(spot.x, spot.y, true), none);
                        view3d::lift(none);
                    }
                }
            }
            break;
        }
        case 'P':
            // "##P" the PPA given up, as though a transfer never finished:
            // the CPU pushes the screen from then on.
            screen::benchGiveUpPpa();
            break;
        case 'u': {
            // "##u" and "name size crc\n": a file onto the card's overhead
            // folder, sent a block at a time, each answered - see
            // tools/sd_upload.py. Only airfields.bin, for now.
            uploadToCard();
            break;
        }
        case 'Q':
            // "##Q" the WiFi turned off and on again. (It does not clear the
            // home router's blocking of the Tab5 after a day of reflashing:
            // the router's own WiFi turned off and on does.)
            Serial.println("[soak] WiFi off and on");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            delay(2000);
            WiFi.mode(WIFI_STA);
            WiFi.begin(g_wifiSsid.c_str(), g_wifiPass.c_str());
            break;
        case 'X': {
            // "##X" and 1 or 0: the card's airfield pack taken as not there,
            // or there again - the 3D view's basic airports, from its next
            // airfield, without touching the card.
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            if (arg == '0' || arg == '1') {
                airfield::benchHidePack(arg == '1');
                Serial.printf("[soak] airfield pack %s\n", arg == '1' ? "hidden" : "back");
            }
            break;
        }
        case 's': {
            // "##s" and 1 or 0: the card's ground pack taken as not there, or
            // there again - the 3D view's flat land with the card as it is.
            // Only before the pack is read in: after a restart, before the
            // 3D view first opens.
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            if (arg == '0' || arg == '1') {
                bool done = groundpack::benchHidePack(arg == '1');
                Serial.printf("[soak] ground pack %s\n", !done ? "already read in - restart first"
                                                       : arg == '1' ? "hidden" : "back");
            }
            break;
        }
        case 't': {
            // "##t" and "x y" on a line: a tap there, as a finger would make
            // it on whatever screen is up - for the bench's screenshots.
            char line[16] = {};
            size_t n = 0;
            uint32_t until = millis() + 1000;
            while (n + 1 < sizeof line && (int32_t)(millis() - until) < 0) {
                int ch = Serial.read();
                if (ch < 0) {
                    delay(2);
                    continue;
                }
                if (ch == '\n') {
                    break;
                }
                line[n++] = (char)ch;
            }
            int x = 0, y = 0;
            if (sscanf(line, "%d %d", &x, &y) == 2) {
                Serial.printf("[soak] tap at %d, %d\n", x, y);
                benchTap(x, y);
            }
            break;
        }
        case 'z': {
            // "##z" and an airport's code on a line - none for home's - its
            // zoom, from the table, the radar or another zoom, nothing being
            // followed: for a screenshot of it.
            char code[8] = {};
            size_t n = 0;
            uint32_t until = millis() + 1000;
            while (n + 1 < sizeof code && (int32_t)(millis() - until) < 0) {
                int ch = Serial.read();
                if (ch < 0) {
                    delay(2);
                    continue;
                }
                if (ch == '\n') {
                    break;
                }
                code[n++] = (char)ch;
            }
            String which = n ? String(code) : g_airportCode;
            if ((g_screen == Screen::MAIN || g_screen == Screen::RADAR || g_screen == Screen::ZOOM) &&
                !g_follow.active && !openZoom(which)) {
                Serial.printf("[soak] no zoom of %s\n", which.c_str());
            }
            break;
        }
        case 'a':
            // "##a" the alerts page, as Settings opens it - from the radar or
            // the zoom - for a screenshot of it; and again, back to the radar,
            // nothing on it saved.
            if (g_screen == Screen::RADAR || g_screen == Screen::ZOOM) {
                alertsScreenSet(g_alertMask, g_autoRecord, g_autoFollow, g_followIn3d, g_watchlist);
                alertsScreenSetVoice(g_readAloud, g_elsewhereAccent);
                alertsScreenSetCard(recorder::mount());
                g_screen = Screen::ALERTS;
                alertsScreenDraw();
            } else if (g_screen == Screen::ALERTS) {
                g_screen = Screen::RADAR;
                drawRadar(true);
            }
            break;
        case 'J': {
            // "##J" and "hex lat lon" on a line: that aircraft followed in 3D -
            // a real one the PC picked from the feed, taxiing at an airport
            // far from home - to see its taxiing as its real reports have it.
            char line[64];
            size_t n = 0;
            uint32_t until = millis() + 2000;
            while (n + 1 < sizeof line && (int32_t)(millis() - until) < 0) {
                int ch = Serial.read();
                if (ch < 0) {
                    delay(2);
                    continue;
                }
                if (ch == '\n') {
                    break;
                }
                line[n++] = (char)ch;
            }
            line[n] = 0;
            char hex[8] = {};
            double lat = 0, lon = 0;
            if (sscanf(line, "%7s %lf %lf", hex, &lat, &lon) == 3) {
                Aircraft a;
                a.hex = hex;
                a.hasPos = true;
                a.lat = (float)lat;
                a.lon = (float)lon;
                a.altStr = "GND";
                a.hasDist = true;
                a.distNm = haversineNm(g_lat, g_lon, a.lat, a.lon);
                Serial.printf("[soak] following %s at %.4f, %.4f in 3D\n", hex, lat, lon);
                if (g_screen == Screen::VIEW3D) {
                    leave3d();
                }
                stopFollow();
                startFollow(a);
                g_screen = Screen::RADAR;
                drawRadar(true);
                open3d();
            }
            break;
        }
        case 'Y': {
            // "##Y" and a digit: the 3D view's bands that many rows from its
            // next open - 1 for 16, 2 for 12, 3 for 8, 4 for 4, 0 as room allows.
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            const int ROWS[] = {0, 16, 12, 8, 4};
            if (arg >= '0' && arg <= '4') {
                view3d::benchBandRows(ROWS[arg - '0']);
            }
            break;
        }
        case 'b':
            // "##b" the 3D view's buildings timed a step at a time - see
            // view3d::benchSolids().
            if (g_screen == Screen::VIEW3D) {
                view3d::benchSolids();
            }
            break;
        case 'r': {
            // "##r" and 1 or 0: made-up traffic about the airfield with the
            // soak's flights, or none - see benchTrafficInto().
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            if (arg == '0' || arg == '1') {
                g_benchTraffic = arg == '1';
                Serial.printf("[soak] made-up traffic %s\n", g_benchTraffic ? "on" : "off");
            }
            break;
        }
        case 'Z': {
            // "##Z" and a digit: the 3D view's sun as by day (1), at dusk (2)
            // or by night (3) - for runs to compare like with like - or as it
            // is (0).
            int arg = Serial.read();
            if (arg < 0) {
                delay(5);
                arg = Serial.read();
            }
            if (arg >= '0' && arg <= '3') {
                view3d::benchSun(arg - '0');
            }
            break;
        }
        case '2':
            if (g_screen == Screen::RADAR || g_screen == Screen::ZOOM) {
                Serial.println("[soak] the 3D button");
                open3d();
            }
            break;
#endif
        }
    }

    // Dimmed after the display page's minutes untouched - and back as a finger
    // lands, that touch doing nothing else, so a tap meant to wake it doesn't
    // also press whatever was under it.
    g_loopAt = "the touch";
    bool touchingNow = M5.Touch.getCount() > 0;
    if (touchingNow) {
        g_lastTouchMs = millis();
        if (g_dimmed) {
            wakeScreen();
            g_wakeTouch = true;
        }
    } else if (g_dimMinutes && !g_dimmed && millis() - g_lastTouchMs > (uint32_t)g_dimMinutes * 60000) {
        g_dimmed = true;
        displayApplyBrightness(std::max(3, g_brightnessPct / 8));
    }
    // Every tap ticks, as a phone's keys do: as it completes, which is when
    // everything acts but the radar and the zoom - so a scroll or a slider's
    // drag doesn't - and on those two as it lands, below, unless it starts a
    // drag across the plot.
    // Not for a touch the radar or the zoom has acted on already, as it
    // landed: its Back, say, puts the table up under the finger, which then
    // lifts as a click there - a second tick for the one tap.
    if (touchingNow && !g_wakeTouch && !g_touchSpent && g_screen != Screen::RADAR && g_screen != Screen::ZOOM &&
        g_screen != Screen::VIEW3D && M5.Touch.getDetail(0).wasClicked()) {
        soundKeyClick();
    }
    if (g_wakeTouch) {
        if (!touchingNow) {
            g_wakeTouch = false;
        }
    } else if (g_screen == Screen::VIEW3D) {
        // Pressed, not merely counted: a finger that has lifted is still
        // counted for a pass after the pass it lifted in - where it was. Back
        // on the details, which acts as it lifts, put this screen up under
        // it, which took it for a finger landing on its own Back, there too.
        handleView3dTouch(touchingNow && M5.Touch.getDetail(0).isPressed());
    } else if (g_screen == Screen::FILTER) {
        // As the display page, every pass: its sliders have to hear the
        // finger lift.
        // Opened from the radar, which acts on a touch as it lands, the
        // touch that opened it is spent: none of the screen's until it lifts.
        int tx = -1, ty = -1;
        bool pressed = false, clicked = false;
        if (g_touchSpent) {
            if (!touchingNow) {
                g_touchSpent = false;
            }
        } else if (touchingNow) {
            auto t = M5.Touch.getDetail(0);
            tx = t.x;
            ty = t.y;
            pressed = t.isPressed();
            clicked = t.wasClicked();
        }
        handleFilterTouch(tx, ty, pressed, clicked);
    } else if (g_screen == Screen::DISPLAY_PAGE) {
        // As settings, every pass: its slider has to hear the finger lift.
        int tx = -1, ty = -1;
        bool pressed = false, clicked = false;
        if (touchingNow) {
            auto t = M5.Touch.getDetail(0);
            tx = t.x;
            ty = t.y;
            pressed = t.isPressed();
            clicked = t.wasClicked();
        }
        handleDisplayPageTouch(tx, ty, pressed, clicked);
    } else if (g_screen == Screen::SETTINGS) {
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
        endDrag();
    } else {
        auto t = M5.Touch.getDetail(0);
        // The radar and the zoom act on a touch as it lands. Elsewhere a tap
        // is a click, which comes as it lifts - and only if it lifted within
        // half a second of landing and less than 8px from where it landed;
        // otherwise it was a hold or a flick, and nothing. With a finger on
        // a glass panel, enough taps strayed past one limit or the other
        // that the radar's buttons felt laggy, or missed taps altogether.
        // Nothing on either screen holds, and the one thing that drags - the
        // plot, by its empty sky - is a touch that had nothing to act on.
        bool onLanding = (g_screen == Screen::RADAR || g_screen == Screen::ZOOM);
        if (g_touchSpent) {
            if (!t.isPressed()) {
                g_touchSpent = false;
                endDrag();
            } else if (g_dragging) {
                dragTo(t.x, t.y);
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
                if (!g_dragging) {
                    soundKeyClick();
                }
            }
        } else if (g_screen == Screen::MAIN && (t.isDragging() || t.isFlicking()) && !bannerOver(t.y)) {
            // The table dragged up or down a row at a time, with more
            // aircraft than rows - from where it was as the finger moved off.
            static int from = 0;
            if (t.wasDragStart() || t.wasFlickStart()) {
                from = displayScroll();
            }
            if (displayScrollTo(from - t.distanceY() / DISPLAY_ROW_HEIGHT) &&
                xSemaphoreTake(g_dataMutex, portMAX_DELAY) == pdTRUE) {
                g_dataReady = true;  // drawn again from the list as it stands
                xSemaphoreGive(g_dataMutex);
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
            case Screen::INFO:
                handleInfoTouch(t.x, t.y);
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
    if ((int32_t)(now - nextRotationCheck) >= 0) {  // across millis() wrapping, after 49 days
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
    tickAlerts();  // what it says is played out by the sound's own task - see soundInit()

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
        uint32_t t0 = micros();
        if (playbackScreenTick()) {
            g_perfReplay.add(micros() - t0);
        }
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
            if (g_screen == Screen::VIEW3D && !g_view3dReplay) {
                g_view3dTraffic = arrivedZoom;
                g_view3dTrafficMs = g_zoomDataMs;
            }
        }
        g_perfTracks.add(micros() - t0);
    }

    g_loopAt = "following";
    tickFollow();
    tickLook();
    announcePending(millis());

    if (g_screen == Screen::VIEW3D) {
        if (g_view3dReplay) {
            g_loopAt = "the 3D replay's frame";
            frame3dReplay();
            g_loopAt = "after the 3D replay's frame";
        } else if (g_follow.active) {
            g_loopAt = "the 3D view's frame";
            update3dSubject();
            view3d::frame(followed3d(), g_view3dTraffic, g_view3dTrafficMs, millis());
            g_loopAt = "after the 3D view's frame";
        } else {
            leave3d();
        }
    }

    if (g_screen == Screen::RADAR) {
        // Following, the plot is drawn from the extra poll's list rather
        // than the main poll's - but a flash step still asks for a redraw
        // through g_dataReady.
        bool fresh = false;
        if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
            fresh = g_dataReady || (radarOnExtraList() && g_zoomReady);
            g_dataReady = false;
            if (radarOnExtraList()) {
                g_zoomReady = false;
            }
            xSemaphoreGive(g_dataMutex);
        }
        uint32_t dataMs = radarOnExtraList() ? g_zoomDataMs : g_latestDataMs;
        uint32_t frameMs = radarOnMap() ? tweenMs(g_pollIntervalS)
                                        : glideMs(g_radarFastestKt,
                                                  (float)(g_follow.active ? g_follow.rangeNm : radarRangeShown()),
                                                  g_pollIntervalS, g_follow.active ? GLIDE_FOLLOW_MIN_MS : GLIDE_MIN_MS,
                                                  g_radarFrameUs);
        if (fresh) {
            drawRadar(false);
        } else if (millis() - g_radarDrawnMs >= frameMs && millis() - dataMs < 2UL * g_pollIntervalS * 1000) {
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
        } else if (millis() - g_zoomDrawnMs >= glideMs(g_zoomFastestKt,
                                                       radarZoomViewNm() > 0 ? radarZoomViewNm() : g_zoomRangeNm,
                                                       ZOOM_POLL_INTERVAL_S, GLIDE_MIN_MS, g_zoomFrameUs) &&
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
    // it is up, however it was left - Back, the timeout, or the WiFi screen
    // taking over.
    bool view3dFromZoom = g_view3dReturnTo == Screen::ZOOM &&
                          (g_screen == Screen::VIEW3D || (g_screen == Screen::DETAIL && g_detailReturnTo == Screen::VIEW3D));
    if (g_zoomUp && g_screen != Screen::ZOOM && !(g_screen == Screen::DETAIL && g_detailReturnTo == Screen::ZOOM) &&
        !view3dFromZoom) {
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
            int unfiltered = (int)aircraft.size();
            applyTrafficFilter(aircraft, &isNew, g_follow.active ? g_follow.hex : String());
            displaySetFiltered(trafficFilter().active(), unfiltered);
            uint32_t t0 = micros();
            displayRenderAircraft(aircraft, isNew);
            g_perfTable.add(micros() - t0);
        }
        uint32_t lapseAt = micros();
        if (displayTickHighlights()) {
            g_perfLapse.add(micros() - lapseAt);
        }

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
            displaySetHeader(g_airportCode, provider);
            if (xSemaphoreTake(g_dataMutex, 0) == pdTRUE) {
                g_dataReady = true;
                xSemaphoreGive(g_dataMutex);
            }
        }
        displayTickStatus();
    }

    if ((g_screen == Screen::RADAR || g_screen == Screen::ZOOM) && !g_dragging) {
        // Following: the clock the time where the aircraft is.
        double lat = 0, lon = 0;
        if (g_follow.active) {
            followCentre(lat, lon);
        }
        radarScreenSetClockAt(g_follow.active, lat, lon);
        radarScreenTickClock();
    }
    // A route lookup finished with the details up: drawn again, with it.
    if (g_screen == Screen::DETAIL && routeGeneration() != g_detailRouteGen) {
        g_detailRouteGen = routeGeneration();
        detailScreenDraw();
    }

    uint32_t passUs = micros() - loopStart;
    if (passUs > g_perfLoop.maxUs) {
        g_longestPassFull = g_passFull;
    }
    g_passFull = false;
    g_perfLoop.add(passUs);
    reportPerf();
    if (g_screen == Screen::VIEW3D) {
        vTaskDelay(1);  // its frames pace the loop; the idle task and the watchdog their turn
    } else {
        delay(10);
    }
}
