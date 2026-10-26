#include "wifi_watch.h"

#include <M5Unified.h>
#include <WiFi.h>
#include <esp32-hal-hosted.h>
#include <esp_attr.h>
#include <esp_event.h>
#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <lwip/dns.h>
#include <lwip/sockets.h>
#include <lwip/tcpip.h>

#include <algorithm>

#include "adsb_client.h"
#include "esp_hosted_event.h"
#include "recorder.h"

// ESP-Hosted 2.12.13's own, exported from its library though not in a header
// this build can include (theirs pull in its protobuf definitions): whether
// its link to the C6 is up, so that a request can be sent at all - see
// askC6().
extern "C" uint8_t is_transport_tx_ready(void);
// How many of ESP-Hosted's receive buffers were given PSRAM for want of
// internal DMA memory: see hosted_rx_rescue.c.
extern "C" uint32_t hostedRxRescues(uint32_t *lastSize);

#ifdef BENCH3D
// ESP-Hosted 2.12.13's own, exported from its library though not in a header
// this build can include (theirs pulls in its protobuf definitions): the
// table of which of the C6's events the P4 acts on. Only the bench's
// simulation of the stuck state uses them - see benchStuck().
extern "C" int reset_event_callback(int event);
extern "C" int rpc_register_event_callbacks(void);
#endif

namespace wifiwatch {
namespace {

// --- the timings ---------------------------------------------------------------

// The link's truth asked of the C6 this often, and not within this long of
// any of the link's events or a step of the ladder: the framework answers an
// event with requests of its own to the C6, and two requests from two tasks
// at once can get each other's answers in ESP-Hosted 2.12.13 (see askC6()).
constexpr uint32_t PROBE_EVERY_MS = 10000;
constexpr uint32_t PROBE_QUIET_MS = 8000;
// A request to the C6 that fails after this long went unanswered: ESP-Hosted
// gives each 5s, and says ESP_FAIL whether it timed out or was refused.
constexpr uint32_t NO_ANSWER_MS = 4000;
// Answers of "not on the network" in a row that mean it, 10s apart: one
// could be the moment of a roam.
constexpr int OFF_AP_RUN = 2;
// Requests unanswered in a row that mean the C6 hears nothing at all: the
// link check's, and then the rejoin's own. Had the first only been slow, its
// late answer would have come in the second's five seconds and been taken
// by it - so two in a row is ten seconds of silence, not one slow reply. And
// every step after is another request that cannot reach it.
constexpr uint32_t DEAF_RUN = 2;

// The feed: network failures in a row, over at least this long, while the
// link looks up. A minute is two polls at the default interval, each asking
// both providers; and the router asked at most this often while it lasts.
constexpr uint32_t FEED_FAILS = 3;
constexpr uint32_t FEED_MIN_MS = 60000;
constexpr uint32_t ROUTER_RECHECK_MS = 120000;
constexpr uint32_t ROUTER_WAIT_MS = 2500;  // per port asked

// How long a step is given before it is judged: the rejoin 45s, the WiFi's
// restart a minute and then twice as long each time it is done again, to a
// quarter of an hour.
constexpr uint32_t VERIFY_REJOIN_MS = 45000;
constexpr uint32_t VERIFY_RESTART_WIFI_MS = 60000;
constexpr uint32_t VERIFY_MOST_MS = 15 * 60000;
// Joining waited on as ensureWifi() waits.
constexpr uint32_t JOIN_WAIT_MS = 12000;
// Trouble back within this long of the last ending starts the ladder a step
// up: the first step was not enough to keep it away.
constexpr uint32_t RECUR_MS = 10 * 60000;
// Never two steps closer than this, nor more than this many in an hour,
// whatever brings them on.
constexpr uint32_t MIN_ACTION_GAP_MS = 30000;
constexpr size_t MAX_ACTIONS_PER_HOUR = 10;

// The restart, after this long of failed recovery - longer for each restart
// in a row that has not brought the feed back, as a restart that did not help
// once is unlikely to the next time either.
constexpr uint32_t RESTART_AFTER_MS[] = {4 * 60000, 20 * 60000, 60 * 60000, 120 * 60000};

// The restart at a quiet moment, for a request that went unanswered: not
// made again in the boot after one until that boot has been up this long -
// a C6 slow to answer now and then would otherwise have the Tab5 restarting
// every time it was left alone.
constexpr int64_t QUIET_AGAIN_US = 12LL * 3600 * 1000000;

// --- the restart's note, kept across it -------------------------------------------

// In the RTC memory, which a restart leaves alone and power-on fills with
// noise - hence the magic and the check. `armed` is set only as the watchdog
// restarts, so a restart of anyone else's - ESP-Hosted makes one itself when
// its link to the C6 fails - is not taken for one of its.
struct RestartNote {
    uint32_t magic;
    uint32_t armed;
    uint32_t inARow;  // restarts by the watchdog without the feed answering in between
    uint32_t flags;   // NOTE_QUIET, NOTE_THEN_WIFI
    char why[96];
    uint32_t check;
};
constexpr uint32_t NOTE_MAGIC = 0x57444732;  // "WDG2": the note since it had flags
// Made at a quiet moment, the WiFi working, for a request gone unanswered -
// see settleRestart() - rather than for the WiFi down.
constexpr uint32_t NOTE_QUIET = 1;
// Made as the WiFi screen was being opened: the boot puts it up.
constexpr uint32_t NOTE_THEN_WIFI = 2;
RTC_NOINIT_ATTR RestartNote g_note;

uint32_t noteCheck(const RestartNote &n) {
    uint32_t h = 2166136261u;
    auto mix = [&h](const void *p, size_t len) {
        const uint8_t *b = (const uint8_t *)p;
        for (size_t i = 0; i < len; i++) {
            h = (h ^ b[i]) * 16777619u;
        }
    };
    mix(&n.magic, sizeof n.magic);
    mix(&n.armed, sizeof n.armed);
    mix(&n.inARow, sizeof n.inARow);
    mix(&n.flags, sizeof n.flags);
    mix(n.why, sizeof n.why);
    return h;
}

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
uint32_t g_restartsInARow = 0;  // from the note, at boot
bool g_restartWorkedSaid = false;
bool g_bootAfterQuiet = false;  // this boot follows a restart made at a quiet moment
bool g_bootOpensWifi = false;   // one made as the WiFi screen was being opened

void saveNote(bool armed, uint32_t inARow, uint32_t flags, const char *why) {
    RestartNote n = {};
    n.magic = NOTE_MAGIC;
    n.armed = armed ? 1 : 0;
    n.inARow = inARow;
    n.flags = flags;
    strlcpy(n.why, why ? why : "", sizeof n.why);
    n.check = noteCheck(n);
    portENTER_CRITICAL(&g_mux);  // the loop writes it as it restarts, the poll task as the feed comes back
    g_note = n;
    portEXIT_CRITICAL(&g_mux);
}

// --- what the link has been doing, from its events -------------------------------

Hooks g_hooks = {};
bool g_begun = false;
uint32_t g_hostMajor = 0, g_hostMinor = 0, g_hostPatch = 0, g_c6Major = 0, g_c6Minor = 0, g_c6Patch = 0;

// Written by the event loop's task - two kilobytes of stack, so counted and
// timed there and nothing printed - and read by the poll task. Counted
// rather than timed where the watchdog asks whether one has come since it
// last acted: two times compared can read the wrong way round once they are
// weeks apart, as millis() comes round every 49.7 days, and an address can
// last for months - a renewal that keeps it is not an event at all.
volatile uint32_t g_lastEventMs = 0;  // any of the station's events, or an address's
volatile uint32_t g_assocs = 0;       // joinings, ever
volatile uint32_t g_gotIps = 0;       // addresses given, ever
// The network the C6 last said it had joined, from its own "connected"
// event; emptied as it says it has left, or its WiFi stops. For the WiFi
// screen: see joinedNetwork().
char g_joinedSsid[33] = "";

void onLinkEvent(void *, esp_event_base_t base, int32_t id, void *data) {
    g_lastEventMs = millis();
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        const wifi_event_sta_connected_t *c = (const wifi_event_sta_connected_t *)data;
        size_t n = 0;
        portENTER_CRITICAL(&g_mux);
        if (c != nullptr) {
            n = c->ssid_len > 0 && c->ssid_len <= sizeof c->ssid ? c->ssid_len
                                                                  : strnlen((const char *)c->ssid, sizeof c->ssid);
            memcpy(g_joinedSsid, c->ssid, n);
        }
        g_joinedSsid[n] = 0;
        g_assocs = g_assocs + 1;
        portEXIT_CRITICAL(&g_mux);
    } else if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_DISCONNECTED || id == WIFI_EVENT_STA_STOP)) {
        portENTER_CRITICAL(&g_mux);
        g_joinedSsid[0] = 0;
        portEXIT_CRITICAL(&g_mux);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        g_gotIps = g_gotIps + 1;
    }
}

// The poll task's own note of when it saw the last address got, for how
// long the feed has been failing on the link as it now is: the count it
// last saw, and when it saw it change - forgotten after an hour, so that an
// age is never taken from a time millis() has since come round past.
uint32_t g_gotIpsSeen = 0, g_gotIpSeenMs = 0;

// ESP-Hosted's own: the C6 starting again, and its link coming and going.
volatile uint32_t g_cpInits = 0, g_transportDowns = 0, g_transportFailures = 0;
volatile int g_cpInitReason = 0;
uint32_t g_cpInitsSaid = 0, g_transportDownsSaid = 0, g_transportFailuresSaid = 0;

void onHostedEvent(void *, esp_event_base_t, int32_t id, void *data) {
    if (id == ESP_HOSTED_EVENT_CP_INIT) {
        g_cpInitReason = data ? (int)((esp_hosted_event_init_t *)data)->reason : -1;
        g_cpInits = g_cpInits + 1;
    } else if (id == ESP_HOSTED_EVENT_TRANSPORT_DOWN) {
        g_transportDowns = g_transportDowns + 1;
    } else if (id == ESP_HOSTED_EVENT_TRANSPORT_FAILURE) {
        g_transportFailures = g_transportFailures + 1;
    }
}

// The same events, said in the log, from the framework's event task, which
// has the stack for it. Arduino's own "Reason:" line is a warning, and the
// log is built to show errors only, so a link that dropped left no trace of
// why.
void logEvent(arduino_event_t *ev) {
    switch (ev->event_id) {
    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("[wifi] the C6's WiFi started");
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
        Serial.println("[wifi] the C6's WiFi stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.printf("[wifi] joined the network, channel %u\n", (unsigned)ev->event_info.wifi_sta_connected.channel);
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
        const wifi_event_sta_disconnected_t &d = ev->event_info.wifi_sta_disconnected;
        Serial.printf("[wifi] left the network: reason %u (%s), signal %d dBm\n", (unsigned)d.reason,
                      WiFi.disconnectReasonName((wifi_err_reason_t)d.reason), (int)d.rssi);
        break;
    }
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.printf("[wifi] given the address %s\n",
                      IPAddress(ev->event_info.got_ip.ip_info.ip.addr).toString().c_str());
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        Serial.println("[wifi] the address lost");
        break;
    default:
        break;
    }
}

// An allocation ESP-Hosted's own tasks could not have - the failure that, in
// its streaming reads, stops the P4 reading from the C6, and whose warning is
// compiled out of this build. Noted here, in whatever task failed, and said
// by the poll task; anyone else's failures - the 3D view's trial sizes,
// which fall back by design - are left alone.
volatile uint32_t g_allocFails = 0, g_allocFailSize = 0, g_allocFailCaps = 0;
char g_allocFailTask[configMAX_TASK_NAME_LEN] = "";
uint32_t g_allocFailsSaid = 0, g_rescuesSaid = 0, g_allocSaidMs = 0;

void onAllocFailed(size_t size, uint32_t caps, const char *) {
    const char *name = pcTaskGetName(nullptr);
    if (name == nullptr) {
        return;
    }
    bool hosted = (name[0] == 's' && name[1] == 'd' && name[2] == 'i' && name[3] == 'o') ||
                  (name[0] == 'r' && name[1] == 'p' && name[2] == 'c' && name[3] == '_');
    if (!hosted) {
        return;
    }
    size_t i = 0;
    for (; i + 1 < sizeof g_allocFailTask && name[i]; i++) {
        g_allocFailTask[i] = name[i];
    }
    g_allocFailTask[i] = 0;
    g_allocFailSize = size;
    g_allocFailCaps = caps;
    g_allocFails = g_allocFails + 1;
}

// --- the poll task's turn at the C6 ------------------------------------------------

// Held by the poll task around its requests to the C6 - see RadioTurn - and
// tried, never waited on, by the WiFi screen before its first: see
// radioFree(). Recursive, so that a function holding it can call another
// that takes it too; and a mutex, so only the task that took it gives it.
SemaphoreHandle_t g_radio = xSemaphoreCreateRecursiveMutex();

const char *const SCREEN_TOOK = "the WiFi screen took the radio";

// --- asking the C6 ---------------------------------------------------------------

enum class Answer : uint8_t {
    ON_AP,      // associated
    OFF_AP,     // alive, and not on the network
    WIFI_OFF,   // alive, its WiFi not initialised or not started
    NO_ANSWER,  // nothing back in the request's 5s: the link to it is stuck
    LINK_DOWN,  // ESP-Hosted would not even send it: its link to the C6 is down
};
struct Probe {
    Answer answer = Answer::ON_AP;
    esp_err_t err = ESP_OK;
    int rssi = 0;  // the signal, in dBm, while it is on the network
    uint32_t tookMs = 0, atMs = 0;
};

// Set once a request to the C6 has gone unanswered, and kept until the
// restart: its answer may still come, and would then be taken by the next
// request made, and that one's by the one after - every answer one out of
// step from then on, for every task that asks the C6 anything. Nothing here
// asks the C6 anything after that but the ladder's own steps, whose answers
// are only ever a yes or no; and once the trouble is over a restart is still
// wanted, at a quiet moment - see settleRestart(). Read by the loop too.
volatile bool g_rpcSuspect = false;
// Requests unanswered in a row, the link check's and the steps' alike.
uint32_t g_unansweredRun = 0;

// Whether a request that came back with `err`, made at `sinceMs`, went
// unanswered: ESP-Hosted says ESP_FAIL whether it timed out or refused, so
// it is told by how long it took. Anything else is the C6 heard from, or
// ESP-Hosted refusing at once, and ends a run of them.
bool unanswered(esp_err_t err, uint32_t sinceMs) {
    if (err == ESP_FAIL && millis() - sinceMs >= NO_ANSWER_MS) {
        g_rpcSuspect = true;
        g_unansweredRun++;
        return true;
    }
    g_unansweredRun = 0;
    return false;
}

bool deaf() { return g_unansweredRun >= DEAF_RUN; }

// Whether the C6 is associated, asked of the C6 itself: the signal of the
// access point it is on, which it has only while it is on one, and says
// "not connected" otherwise. Not its association id, the question asked at
// first: the factory C6 firmware keeps that after leaving - on the bench on
// 26 Oct, the C6 off the network, it answered "association 4" and ESP_OK,
// and the stuck state it was there to find went unseen until the feed's
// watchdog found it a minute and a half later. Nor esp_wifi_sta_get_ap_info(),
// the obvious question, for one reason. ESP-Hosted 2.12.13 hands each
// answer to whichever waiting request takes it first, not the request it
// answers (esp-hosted-mcu #242), and an answer left behind by a request that
// gave up waiting goes to the next request made. An answer read as the wrong
// kind is harmless where only a number is copied out of it, as here; the
// access point's details are copied through a pointer, which in another
// request's answer is nothing, and the Tab5 crashes. Asked holding the poll
// task's turn.
Probe askC6() {
    Probe p;
    int rssi = 0;
    uint32_t t0 = millis();
    esp_err_t err = esp_wifi_sta_get_rssi(&rssi);
    p.atMs = millis();
    p.tookMs = p.atMs - t0;
    p.err = err;
    p.rssi = rssi;
    if (unanswered(err, t0)) {
        p.answer = Answer::NO_ANSWER;
    } else if (err == ESP_OK) {
        p.answer = Answer::ON_AP;
    } else if (err == ESP_ERR_WIFI_NOT_CONNECT) {
        p.answer = Answer::OFF_AP;
    } else if (err == ESP_ERR_WIFI_NOT_INIT || err == ESP_ERR_WIFI_NOT_STARTED) {
        p.answer = Answer::WIFI_OFF;
    } else if (err == ESP_FAIL) {
        // At once: the C6's own answer, if ESP-Hosted's link to it is up -
        // the factory firmware says ESP_FAIL, not "not connected", for a
        // signal it does not have, as the bench found - and otherwise
        // ESP-Hosted refusing to send the request at all.
        p.answer = is_transport_tx_ready() ? Answer::OFF_AP : Answer::LINK_DOWN;
    } else {
        p.answer = Answer::OFF_AP;  // an error of some other kind: not on the network, as far as can be told
    }
    return p;
}

// Whether the router answers: a TCP connection to it, on the DNS port and
// then the web page's, either taken or refused - a refusal is an answer just
// the same - within 2.5s. It is the one test of the whole path, both ways
// through the C6 and the air, that does not depend on anything beyond the
// router: when the feed fails and the router answers, the trouble is the
// internet or the providers, which no rejoining here can mend. No socket to
// ask with is trouble of this Tab5's own - every one taken - which the
// ladder's restart does mend, so it is acted on as silence is.
enum class Reach : uint8_t { ANSWERS, SILENT, NO_SOCKET, UNKNOWN };

Reach routerReach(uint32_t *tookMs = nullptr) {
    uint32_t t0 = millis();
    IPAddress gw = WiFi.gatewayIP();
    if ((uint32_t)gw == 0) {
        return Reach::UNKNOWN;
    }
    static const uint16_t PORTS[] = {53, 80};
    for (uint16_t port : PORTS) {
        int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s < 0) {
            return Reach::NO_SOCKET;
        }
        fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
        struct sockaddr_in to = {};
        to.sin_family = AF_INET;
        to.sin_port = htons(port);
        to.sin_addr.s_addr = (uint32_t)gw;
        bool answered = false;
        int r = connect(s, (struct sockaddr *)&to, sizeof to);
        if (r == 0) {
            answered = true;
        } else if (errno == EINPROGRESS) {
            fd_set writable;
            FD_ZERO(&writable);
            FD_SET(s, &writable);
            struct timeval tv = {(time_t)(ROUTER_WAIT_MS / 1000), (suseconds_t)((ROUTER_WAIT_MS % 1000) * 1000)};
            if (select(s + 1, nullptr, &writable, nullptr, &tv) > 0) {
                int err = 0;
                socklen_t len = sizeof err;
                if (getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &len) == 0) {
                    answered = err == 0 || err == ECONNREFUSED || err == ECONNRESET;
                }
            }
        } else {
            answered = errno == ECONNREFUSED || errno == ECONNRESET;
        }
        close(s);
        if (answered) {
            if (tookMs) {
                *tookMs = millis() - t0;
            }
            return Reach::ANSWERS;
        }
    }
    if (tookMs) {
        *tookMs = millis() - t0;
    }
    return Reach::SILENT;
}

// --- the bench's changes, BENCH3D --------------------------------------------------

struct Bench {
    bool probePaused = false;  // the link check left to the feed's watchdog
    bool skipRouter = false;   // the router taken as silent
    bool dnsBroken = false;
};
Bench g_bench;
volatile char g_benchCmd = 0;

// --- the ladder -----------------------------------------------------------------

enum class Cause : uint8_t { OFF_AP, WIFI_OFF, NO_ANSWER, FEED };

struct Episode {
    bool active = false;
    Cause cause = Cause::OFF_AP;
    uint32_t startMs = 0;
    int firstStep = 1;
    int lastStep = 0;  // 1 rejoined, 2 the WiFi restarted; 0 none yet
    // When the last step was taken - or the trouble began, before one has
    // been - and the counts then: what has changed since is what judges it.
    // The counts, not times, decide whether anything has: see g_gotIps.
    uint32_t lastStepMs = 0;
    uint32_t assocsAtStep = 0, gotIpsAtStep = 0, answersAtStep = 0, netFailsAtStep = 0;
    uint32_t nextMs = 0;  // when it is next judged
    uint32_t wifiRestarts = 0;
    bool restartAsked = false;
};
Episode g_ep;
uint32_t g_lastEpisodeEndMs = 0;
uint32_t g_lastActionMs = 0;
uint32_t g_actionTimes[MAX_ACTIONS_PER_HOUR] = {};
size_t g_actionNext = 0;
uint32_t g_heldOffSaidMs = 0;
// The link check waits a while after anything done to the WiFi, by a step or
// from the bench, as it does after the link's events.
uint32_t g_quietFromMs = 0;

uint32_t g_lastProbeMs = 0;
int g_offApRun = 0;
Probe g_probe;
bool g_probed = false;

uint32_t g_lastRouterCheckMs = 0;
// The failure run already said to be beyond the router, by its first
// failure's place in the count of them: see checkFeed().
uint32_t g_feedHeldRun = UINT32_MAX;

volatile Restart g_restart = Restart::NONE;
char g_restartWhy[96] = "";
bool g_quietHeldSaid = false;

String took(uint32_t ms) {
    if (ms < 60000) {
        return String(ms / 1000) + "s";
    }
    char buf[16];
    snprintf(buf, sizeof buf, "%um%02us", (unsigned)(ms / 60000), (unsigned)(ms / 1000 % 60));
    return String(buf);
}

// How far into an episode its restart is asked for: 4 minutes, and longer
// for each restart in a row that has not brought the feed back. At once for
// a C6 that hears nothing - see DEAF_RUN - unless restarts in a row have not
// been helping, when it waits its turn like any other.
uint32_t restartAfterMs() {
    if (deaf() && g_restartsInARow == 0) {
        return 0;
    }
    constexpr uint32_t LAST = sizeof RESTART_AFTER_MS / sizeof RESTART_AFTER_MS[0] - 1;
    return RESTART_AFTER_MS[std::min(g_restartsInARow, LAST)];
}

// How long the n-th restart of the WiFi in a row is given: a minute, then
// twice as long each time, to a quarter of an hour.
uint32_t verifyRestartWifiMs(uint32_t n) {
    return std::min(VERIFY_RESTART_WIFI_MS << std::min<uint32_t>(n > 0 ? n - 1 : 0, 4), VERIFY_MOST_MS);
}

// 0 if a step may be taken now; otherwise when one may.
uint32_t actionWaitUntil(uint32_t now) {
    if (g_lastActionMs != 0 && now - g_lastActionMs < MIN_ACTION_GAP_MS) {
        return g_lastActionMs + MIN_ACTION_GAP_MS;
    }
    uint32_t oldest = g_actionTimes[g_actionNext];  // the slot written longest ago
    if (oldest != 0 && now - oldest < 3600000) {
        return oldest + 3600000;
    }
    return 0;
}

void noteAction() {
    uint32_t now = millis();
    g_actionTimes[g_actionNext] = now != 0 ? now : 1;
    g_actionNext = (g_actionNext + 1) % MAX_ACTIONS_PER_HOUR;
    g_lastActionMs = now;
    g_quietFromMs = now;
}

// The times above that only matter for so long, let go once they are past
// it - before millis() can come round to where an old one reads as new, and
// holds a step off for an hour, or a log line, for nothing.
void forgetOldTimes(uint32_t now) {
    for (uint32_t &t : g_actionTimes) {
        if (t != 0 && now - t >= 3600000) {
            t = 0;
        }
    }
    if (g_lastActionMs != 0 && now - g_lastActionMs >= 3600000) {
        g_lastActionMs = 0;
    }
    if (g_heldOffSaidMs != 0 && now - g_heldOffSaidMs >= 3600000) {
        g_heldOffSaidMs = 0;
    }
    if (g_gotIpSeenMs != 0 && now - g_gotIpSeenMs >= 3600000) {
        g_gotIpSeenMs = 0;
    }
    if (g_allocSaidMs != 0 && now - g_allocSaidMs >= 3600000) {
        g_allocSaidMs = 0;
    }
    if (g_lastEpisodeEndMs != 0 && now - g_lastEpisodeEndMs >= RECUR_MS) {
        g_lastEpisodeEndMs = 0;
    }
}

// Noted as the poll task sees a new address: see g_gotIpSeenMs.
void noteGotIps(uint32_t now) {
    uint32_t n = g_gotIps;
    if (n != g_gotIpsSeen) {
        g_gotIpsSeen = n;
        g_gotIpSeenMs = now != 0 ? now : 1;
    }
}

// The counts taken as a step starts - or as the trouble begins: what has
// changed since is what judges it.
void markStep(uint32_t now) {
    const AdsbFeedHealth &h = adsbFeedHealth();
    g_ep.lastStepMs = now;
    g_ep.assocsAtStep = g_assocs;
    g_ep.gotIpsAtStep = g_gotIps;
    g_ep.answersAtStep = h.answers;
    g_ep.netFailsAtStep = h.netFailsEver;
}

void waitJoined() {
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < JOIN_WAIT_MS && !g_hooks.uiOwnsWifi()) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

String joinedText() {
    return WiFi.status() == WL_CONNECTED ? "back, address " + WiFi.localIP().toString() : String("not back yet");
}

// Waits up to `ms` for `done`, giving up early if the WiFi screen claims the
// radio: true if it did.
template <typename Done>
bool waitOrScreen(uint32_t ms, Done done) {
    uint32_t start = millis();
    while (!done() && millis() - start < ms) {
        if (g_hooks.uiOwnsWifi()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return g_hooks.uiOwnsWifi();
}

// Step 1: the network rejoined - ensureWifi()'s disconnect and begin, made to
// work when the P4 is out of date. A begin waits for the P4's "connected" to
// clear, and when the C6 left without the P4 hearing, nothing ever clears
// it: the C6, already off the network, has nothing to say when told to
// leave, the begin gives up after a second, and the C6 is never asked to join
// at all. So the C6 is told to leave, and if the P4 hears it - the C6 was
// still on, and the trouble was the data, not the association - the ordinary
// begin follows. If it does not hear, the C6 is asked to join again with the
// configuration it already holds. It does, and says so; the P4, still
// believing it was connected all along, lets that pass, and carries on with
// its address as it was, which the router still has for it.
//
// Made holding the poll task's turn at the C6, and given up wherever the
// WiFi screen claims the radio meanwhile - looked at before each request,
// the begin's above all, as the second's settling before it is time enough
// for the screen to have come up: it then has the radio to itself.
bool stepRejoin(String &said) {
    adsbDropConnection();
    String ssid, pass;
    if (!g_hooks.credentials(ssid, pass)) {
        said = "no network to join";
        return false;
    }
    if (g_hooks.uiOwnsWifi()) {
        said = SCREEN_TOOK;
        return true;
    }
    uint32_t t0 = millis();
    esp_err_t err = esp_wifi_disconnect();
    if (unanswered(err, t0)) {
        said = "the C6 did not answer being told to leave";
        return false;
    }
    if (waitOrScreen(2000, [] { return !WiFi.STA.connected(); })) {
        said = SCREEN_TOOK;
        return true;
    }
    if (!WiFi.STA.connected()) {
        // The framework's own handling of the leaving - a request or two of
        // its own to the C6 - finished before this one's start.
        if (waitOrScreen(1000, [] { return false; })) {
            said = String("left the network, then ") + SCREEN_TOOK + " before the join";
            return true;
        }
        WiFi.begin(ssid.c_str(), pass.c_str());
        waitJoined();
        said = "left and joined again: " + joinedText();
        return true;
    }
    t0 = millis();
    err = esp_wifi_connect();
    if (unanswered(err, t0)) {
        said = "the Tab5 had not heard the C6 leave, and the C6 did not answer being asked to join";
        return false;
    }
    said = String("the Tab5 had not heard the C6 leave - the C6 asked to join again (") + esp_err_to_name(err) + ")";
    return true;
}

// Step 2: the C6's WiFi stopped and started, then joined afresh. Its stop
// comes to the P4 as an event that clears what the P4 believes all the way
// down - the framework's "connected" and ESP-Hosted's own flags with it - so
// the start and the join that follow go as they did at boot, address and
// all. Done with the C6's WiFi calls directly, not WiFi.mode(): turning the
// WiFi off that way takes ESP-Hosted down with it, which on 2.12.13
// corrupts the heap. The start waits for the P4 to have seen the stop, or
// ESP-Hosted would not pass the start on, and the join would never get an
// address.
//
// Given up for the WiFi screen as step 1 is, except between the stop and
// the start: the screen given the radio with the C6's WiFi stopped would
// find its scan and its joins all failing, and no way to start it again.
// Those few seconds are seen through, then the screen has its turn.
bool stepRestartWifi(String &said) {
    adsbDropConnection();
    String ssid, pass;
    if (!g_hooks.credentials(ssid, pass)) {
        said = "no network to join";
        return false;
    }
    if (g_hooks.uiOwnsWifi()) {
        said = SCREEN_TOOK;
        return true;
    }
    uint32_t t0 = millis();
    esp_err_t err = esp_wifi_disconnect();
    if (unanswered(err, t0)) {
        said = "the C6 did not answer being told to leave";
        return false;
    }
    if (waitOrScreen(500, [] { return false; })) {
        said = String("left the network, then ") + SCREEN_TOOK + " before the stop";
        return true;
    }
    t0 = millis();
    err = esp_wifi_stop();
    if (unanswered(err, t0)) {
        said = "the C6 did not answer being told to stop";
        return false;
    }
    esp_netif_t *netif = WiFi.STA.netif();
    uint32_t wait = millis();
    auto stopped = [netif]() {
        return WiFi.status() == WL_STOPPED && (netif == nullptr || !esp_netif_is_netif_up(netif));
    };
    while (!stopped() && millis() - wait < 3000) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    bool heard = stopped();
    vTaskDelay(pdMS_TO_TICKS(300));
    t0 = millis();
    err = esp_wifi_start();
    if (unanswered(err, t0)) {
        said = "the C6 stopped its WiFi, then did not answer being told to start it";
        return false;
    }
    wait = millis();
    while (heard && !WiFi.STA.started() && millis() - wait < 3000) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // The framework's own answer to the start - the power saving it sets, a
    // request of its own - finished before this one's join.
    if (waitOrScreen(1000, [] { return false; })) {
        said = String("stopped and started, then ") + SCREEN_TOOK + " before the join";
        return true;
    }
    if (heard) {
        WiFi.begin(ssid.c_str(), pass.c_str());
        waitJoined();
        said = "stopped, started and joined again: " + joinedText();
    } else {
        // The P4 never heard the stop, so believes itself connected still,
        // and a begin would give up as step 1 explains: the C6 asked to join
        // as it is asked there.
        t0 = millis();
        err = esp_wifi_connect();
        if (unanswered(err, t0)) {
            said = "the Tab5 never heard the C6 stop, and the C6 did not answer being asked to join";
            return false;
        }
        said = WiFi.status() == WL_STOPPED
                   ? "stopped and started, the Tab5's network interface never going down - the C6 asked to join again"
                   : "stopped and started, the Tab5 never hearing the stop - the C6 asked to join again";
    }
    return true;
}

void setRestart(Restart kind, const char *why) {
    portENTER_CRITICAL(&g_mux);
    if (kind != Restart::NONE) {
        strlcpy(g_restartWhy, why, sizeof g_restartWhy);
    }
    g_restart = kind;
    portEXIT_CRITICAL(&g_mux);
}

// The restart wanted once the trouble is over, or the WiFi screen has taken
// the radio from it: none - unless a request to the C6 has gone unanswered
// this boot. Then every answer since may be out of step, and will be until a
// restart, so one is still wanted; not at once, the WiFi working, but at a
// quiet moment - see checkWatchdogRestart() in main.cpp - or just before the
// WiFi screen opens. Not in the boot after one made for this, though, until
// it has been up twelve hours (QUIET_AGAIN_US): meanwhile the link check
// stays off, and the feed's watchdog looks after the link alone.
void settleRestart() {
    if (!g_rpcSuspect) {
        setRestart(Restart::NONE, "");
        return;
    }
    if (g_bootAfterQuiet && esp_timer_get_time() < QUIET_AGAIN_US) {
        setRestart(Restart::NONE, "");
        if (!g_quietHeldSaid) {
            g_quietHeldSaid = true;
            Serial.println("[wifi] watchdog: a request to the C6 went unanswered, but this boot follows a restart made "
                           "for that - no other for it until 12 hours up; the link check off meanwhile");
        }
        return;
    }
    if (g_restart != Restart::WHEN_IDLE) {
        setRestart(Restart::WHEN_IDLE, "a request to the C6 unanswered, its answers since perhaps out of step");
        Serial.println("[wifi] watchdog: a restart asked for at the next quiet moment - a request to the C6 went "
                       "unanswered, and every answer since may be out of step until one");
    }
}

void takeStep(int step);

void endEpisode(const char *how) {
    uint32_t now = millis();
    if (g_ep.active) {
        Serial.printf("[wifi] watchdog: %s, %s after it began%s\n", how, took(now - g_ep.startMs).c_str(),
                      g_ep.lastStep == 1   ? ", after step 1 (the network rejoined)"
                      : g_ep.lastStep == 2 ? ", after step 2 (the WiFi restarted)"
                                           : "");
    }
    g_ep = Episode();
    g_lastEpisodeEndMs = now != 0 ? now : 1;
    g_offApRun = 0;
    g_bench.probePaused = false;
    settleRestart();
}

void startEpisode(Cause cause, const String &seen) {
    if (g_ep.active) {
        return;
    }
    String ssid, pass;
    if (!g_hooks.credentials(ssid, pass)) {
        return;  // nothing to join: the WiFi screen's business
    }
    uint32_t now = millis();
    g_ep = Episode();
    g_ep.active = true;
    g_ep.cause = cause;
    g_ep.startMs = now;
    markStep(now);
    g_ep.firstStep = cause == Cause::WIFI_OFF ? 2 : 1;
    bool recur = g_lastEpisodeEndMs != 0 && now - g_lastEpisodeEndMs < RECUR_MS;
    if (recur) {
        g_ep.firstStep = 2;
    }
    Serial.printf("[wifi] watchdog: %s%s\n", seen.c_str(), recur ? " - back within 10 minutes of the last time" : "");
    takeStep(g_ep.firstStep);
}

void takeStep(int step) {
    uint32_t now = millis();
    uint32_t until = actionWaitUntil(now);
    if (until != 0) {
        if (now - g_lastActionMs >= MIN_ACTION_GAP_MS && g_heldOffSaidMs == 0) {
            g_heldOffSaidMs = now != 0 ? now : 1;
            Serial.printf("[wifi] watchdog: holding off - %u steps in the last hour already; the next in %s\n",
                          (unsigned)MAX_ACTIONS_PER_HOUR, took(until - now).c_str());
        }
        g_ep.nextMs = until;
        return;
    }
    noteAction();
    Serial.printf("[wifi] watchdog: step %d - %s\n", step,
                  step == 1 ? "rejoining the network" : "restarting the C6's WiFi");
    // Judged from the step's start: a joining it makes itself that gets no
    // address is the network taking the Tab5 back like any other.
    g_ep.lastStep = step;
    markStep(now);
    String said;
    {
        RadioTurn turn;
        if (step == 1) {
            stepRejoin(said);
        } else {
            stepRestartWifi(said);
        }
    }
    if (step == 2) {
        g_ep.wifiRestarts++;
    }
    uint32_t done = millis();
    g_lastActionMs = done;
    g_quietFromMs = done;
    Serial.printf("[wifi] watchdog: step %d done in %.1fs: %s\n", step, (done - now) / 1000.0f, said.c_str());
    if (deaf()) {
        // Judged straight away: a C6 that has answered neither of the last
        // two requests is the restart's to mend, and nothing else's.
        g_ep.nextMs = done;
        return;
    }
    g_ep.nextMs = done + (step == 2 ? verifyRestartWifiMs(g_ep.wifiRestarts) : VERIFY_REJOIN_MS);
    // The restart's time isn't put off by the steps' spacing.
    uint32_t restartAt = g_ep.startMs + restartAfterMs();
    if (!g_ep.restartAsked && (int32_t)(restartAt - done) > 0 && (int32_t)(restartAt - g_ep.nextMs) < 0) {
        g_ep.nextMs = restartAt;
    }
}

void askRestart(uint32_t now, const char *still) {
    g_ep.restartAsked = true;
    char why[96];
    if (deaf()) {
        snprintf(why, sizeof why, "the C6 answering nothing - %u requests in a row unanswered",
                 (unsigned)g_unansweredRun);
        setRestart(Restart::NOW, why);
        Serial.printf("[wifi] watchdog: the C6 has answered none of the last %u requests (%s) - nothing but a restart "
                      "reaches it: asking for one, the C6's power cut first; %u in a row before this\n",
                      (unsigned)g_unansweredRun, still, (unsigned)g_restartsInARow);
        return;
    }
    snprintf(why, sizeof why, "the WiFi down %s, %u WiFi restart%s not mending it", took(now - g_ep.startMs).c_str(),
             (unsigned)g_ep.wifiRestarts, g_ep.wifiRestarts == 1 ? "" : "s");
    setRestart(Restart::NOW, why);
    Serial.printf("[wifi] watchdog: still down %s in (%s) - asking for a restart of the Tab5, the C6's power cut "
                  "first; %u in a row before this\n",
                  took(now - g_ep.startMs).c_str(), still, (unsigned)g_restartsInARow);
}

enum class Verdict : uint8_t { GOOD, BAD, UNKNOWN, WAIT };

// How the link is, at a step's time: back, still down, or out of this
// watchdog's hands - the P4 knows the link is down, for a reason the
// ordinary reconnect is for, such as a network that has gone.
Verdict judge(wl_status_t st, String &why) {
    const AdsbFeedHealth &h = adsbFeedHealth();
    if (h.answers != g_ep.answersAtStep) {
        why = "the feed answers";
        return Verdict::GOOD;
    }
    if (st == WL_STOPPED || !WiFi.STA.started()) {
        // Left stopped by step 2 - its start never reaching the P4 - which
        // the ordinary reconnect cannot mend: its begin waits for a start.
        why = "the WiFi left stopped";
        return Verdict::BAD;
    }
    if (st != WL_CONNECTED) {
        bool takenBack = g_assocs != g_ep.assocsAtStep && g_gotIps == g_ep.gotIpsAtStep;
        if (takenBack) {
            why = "taken back by the network without an address";
            return Verdict::BAD;
        }
        why = "the Tab5 knows the link is down";
        return Verdict::UNKNOWN;
    }
    Reach r = g_bench.skipRouter ? Reach::SILENT : routerReach();
    if (r == Reach::ANSWERS) {
        why = "the router answers";
        return Verdict::GOOD;
    }
    if (r == Reach::SILENT || r == Reach::NO_SOCKET) {
        why = g_bench.skipRouter      ? "the router taken as silent, from the bench"
              : r == Reach::NO_SOCKET ? "no socket to be had to ask the router with"
                                      : "the router does not answer";
        return Verdict::BAD;
    }
    if (h.netFailsEver != g_ep.netFailsAtStep) {
        why = String("the feed still failing: ") + h.lastNetFail;
        return Verdict::BAD;
    }
    return Verdict::WAIT;
}

void decide(uint32_t now, wl_status_t st) {
    String why;
    Verdict v = judge(st, why);
    if (v == Verdict::GOOD) {
        endEpisode((String("recovered - ") + why).c_str());
        return;
    }
    if (v == Verdict::UNKNOWN) {
        endEpisode("left to the ordinary reconnect - the Tab5 knows the link is down");
        return;
    }
    if (v == Verdict::WAIT) {
        // Nothing to judge it by - the router could not be asked, and no
        // request has gone since the step, the providers' backoff holding
        // them all. Asked again shortly; not for ever.
        if (now - g_ep.lastStepMs > RECUR_MS) {
            endEpisode("given up judging - nothing to judge it by for 10 minutes");
        } else {
            g_ep.nextMs = now + MIN_ACTION_GAP_MS;
        }
        return;
    }
    if (!g_ep.restartAsked && now - g_ep.startMs >= restartAfterMs()) {
        askRestart(now, why.c_str());
        // The loop takes it within a pass unless it has to wait; nothing is
        // done to the WiFi in the meantime.
        g_ep.nextMs = now + MIN_ACTION_GAP_MS;
        return;
    }
    uint32_t restartAt = g_ep.startMs + restartAfterMs();
    if (deaf()) {
        // No step - each is another request to a C6 that has answered none
        // of the last two - only the restart, when its time comes; once it
        // is asked for, another look each minute while the loop holds it,
        // for the router answering again. The feed answering ends this at
        // once, in tick().
        g_ep.nextMs = g_ep.restartAsked ? now + VERIFY_RESTART_WIFI_MS : restartAt;
        return;
    }
    if (g_ep.lastStep == 0) {
        takeStep(g_ep.firstStep);  // held off by the spacing when the trouble began
        return;
    }
    if (g_ep.lastStep == 1) {
        Serial.printf("[wifi] watchdog: still down %s after step 1 (%s)\n", took(now - g_ep.lastStepMs).c_str(),
                      why.c_str());
        takeStep(2);
        return;
    }
    if (!g_ep.restartAsked && (int32_t)(restartAt - now) > 0 && restartAt - now <= VERIFY_RESTART_WIFI_MS) {
        g_ep.nextMs = restartAt;  // the restart's time is near: no restart of the WiFi just before it
        return;
    }
    uint32_t again = g_ep.lastStepMs + verifyRestartWifiMs(g_ep.wifiRestarts);
    if ((int32_t)(now - again) >= 0) {
        Serial.printf("[wifi] watchdog: still down %s after step 2 (%s)%s\n", took(now - g_ep.lastStepMs).c_str(),
                      why.c_str(), g_ep.restartAsked ? ", the restart waiting" : "");
        takeStep(2);
    } else {
        g_ep.nextMs = again;  // woken early for the restart's time; the next restart of the WiFi when it is due
    }
}

// --- the two triggers ------------------------------------------------------------

void maybeProbe(uint32_t now) {
    if (g_rpcSuspect || g_bench.probePaused) {
        return;
    }
    if (now - g_lastProbeMs < PROBE_EVERY_MS || now - g_lastEventMs < PROBE_QUIET_MS ||
        (g_quietFromMs != 0 && now - g_quietFromMs < PROBE_QUIET_MS)) {
        return;
    }
    g_lastProbeMs = now;
    {
        RadioTurn turn;
        if (g_hooks.uiOwnsWifi()) {
            return;  // the WiFi screen's now; the next tick stands down
        }
        g_probe = askC6();
    }
    g_probed = true;
    if (g_probe.answer == Answer::ON_AP) {
        g_offApRun = 0;
        return;
    }
    if (g_probe.answer == Answer::NO_ANSWER) {
        startEpisode(Cause::NO_ANSWER, "the C6 did not answer in 5s whether it is on the network - its link to the "
                                       "Tab5 looks stuck, and it will not be asked again before a restart");
        return;
    }
    // Not on the network, its WiFi not running, or ESP-Hosted refusing to
    // ask: twice running, 10s apart, before anything is done about it.
    if (++g_offApRun < OFF_AP_RUN || g_ep.active) {
        return;
    }
    g_offApRun = 0;
    if (g_probe.answer == Answer::WIFI_OFF) {
        startEpisode(Cause::WIFI_OFF, String("the Tab5 believes it is connected, but the C6 says its WiFi is not "
                                             "running (asked twice, 10s apart; ") +
                                          esp_err_to_name(g_probe.err) + ")");
    } else if (g_probe.answer == Answer::LINK_DOWN) {
        startEpisode(Cause::NO_ANSWER, "the Tab5 believes it is connected, but ESP-Hosted says its link to the C6 is "
                                       "down (asked twice, 10s apart)");
    } else {
        startEpisode(Cause::OFF_AP, String("the Tab5 believes it is connected, but the C6 says it is not on the "
                                           "network (asked twice, 10s apart; ") +
                                        esp_err_to_name(g_probe.err) + ")");
    }
}

// The feed failing while the link looks up. If the router still answers, the
// WiFi works and the trouble is beyond it, and that is all there is to it:
// no step and no restart, for however long it lasts. A restart once came of
// half an hour of it, in case this Tab5's own networking was what had stuck;
// but the router taking a fresh connection is that networking working - a
// socket to be had, lwIP and the C6 carrying it both ways - so what was left
// was an outage of the internet or of the providers, which a restart cannot
// mend. And with a provider picked by hand, which nothing falls back from,
// that provider's outage alone would have had the Tab5 restarting for as
// long as it lasted.
void checkFeed(uint32_t now) {
    const AdsbFeedHealth &h = adsbFeedHealth();
    if (h.netFails < FEED_FAILS) {
        return;
    }
    // Over at least a minute: from the run's first failure, or from an
    // address got since, the link a new one then. Ages compared, not times.
    uint32_t age = now - h.netFailsSinceMs;
    if (g_gotIpSeenMs != 0 && now - g_gotIpSeenMs < age) {
        age = now - g_gotIpSeenMs;
    }
    if (age < FEED_MIN_MS) {
        return;
    }
    if (g_lastRouterCheckMs != 0 && now - g_lastRouterCheckMs < ROUTER_RECHECK_MS) {
        return;
    }
    g_lastRouterCheckMs = now != 0 ? now : 1;
    uint32_t routerMs = 0;
    Reach r = g_bench.skipRouter ? Reach::SILENT : routerReach(&routerMs);
    if (r == Reach::UNKNOWN) {
        return;
    }
    char seen[200];
    snprintf(seen, sizeof seen, "the feed has failed %u times in a row over %s (the last: %s)", (unsigned)h.netFails,
             took(now - h.netFailsSinceMs).c_str(), h.lastNetFail);
    if (r == Reach::ANSWERS) {
        // Said once for each run of failures, known by where its first
        // falls in the count of them all.
        uint32_t run = h.netFailsEver - h.netFails;
        if (g_feedHeldRun != run) {
            g_feedHeldRun = run;
            Serial.printf("[wifi] watchdog: %s, but the router answers (%ums) - the trouble is beyond it, so the "
                          "WiFi is left alone\n",
                          seen, (unsigned)routerMs);
        }
        return;
    }
    const char *router = g_bench.skipRouter      ? ", and the router is taken as silent (bench)"
                         : r == Reach::NO_SOCKET ? ", and there is no socket to ask the router with"
                                                 : ", and the router does not answer";
    startEpisode(Cause::FEED, String(seen) + router);
}

// Said once, as they are seen: the C6 starting again on its own, ESP-Hosted's
// link failing, its tasks short of memory; and the end of a run of
// restarts, once the feed answers again.
void sayWhatHappened() {
    uint32_t n = g_cpInits;
    if (n != g_cpInitsSaid) {
        g_cpInitsSaid = n;
        Serial.printf("[wifi] the C6 started again on its own (its reset reason %d)\n", (int)g_cpInitReason);
    }
    n = g_transportDowns;
    if (n != g_transportDownsSaid) {
        g_transportDownsSaid = n;
        Serial.println("[wifi] ESP-Hosted says its link to the C6 went down");
    }
    n = g_transportFailures;
    if (n != g_transportFailuresSaid) {
        g_transportFailuresSaid = n;
        Serial.println("[wifi] ESP-Hosted says its link to the C6 failed");
    }
    // At most once a minute, with a count: short of internal DMA memory for
    // long, ESP-Hosted can fail a packet's allocation many times a second.
    n = g_allocFails;
    uint32_t now = millis();
    if (n != g_allocFailsSaid && (g_allocSaidMs == 0 || now - g_allocSaidMs >= 60000)) {
        uint32_t fresh = n - g_allocFailsSaid;
        g_allocFailsSaid = n;
        g_allocSaidMs = now != 0 ? now : 1;
        // Given PSRAM instead, by hosted_rx_rescue.c, or not.
        uint32_t rescues = hostedRxRescues(nullptr);
        bool rescued = rescues != g_rescuesSaid;
        g_rescuesSaid = rescues;
        Serial.printf("[mem] ESP-Hosted's %s could not have %u bytes (caps 0x%x)%s; the largest block of internal DMA "
                      "memory %uKB now - %s\n",
                      g_allocFailTask, (unsigned)g_allocFailSize, (unsigned)g_allocFailCaps,
                      fresh > 1 ? (", " + String(fresh) + " times since the last said").c_str() : "",
                      (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) / 1024),
                      rescued ? "given PSRAM instead, so the Tab5 still hears the C6"
                              : "in its streaming reads this stops the Tab5 hearing the C6");
    }
    if (g_restartsInARow > 0 && adsbFeedHealth().answers != 0) {
        portENTER_CRITICAL(&g_mux);
        uint32_t was = g_restartsInARow;
        g_restartsInARow = 0;
        RestartNote note = g_note;
        portEXIT_CRITICAL(&g_mux);
        saveNote(false, 0, 0, note.why);
        if (!g_restartWorkedSaid) {
            g_restartWorkedSaid = true;
            Serial.printf("[wifi] watchdog: the feed answers again after its restart%s\n",
                          was > 1 ? (" - " + String(was) + " in a row").c_str() : "");
        }
    }
}

#ifdef BENCH3D
const char *answerText(Answer a) {
    switch (a) {
    case Answer::ON_AP:
        return "on the network";
    case Answer::OFF_AP:
        return "not on the network";
    case Answer::WIFI_OFF:
        return "its WiFi not running";
    case Answer::NO_ANSWER:
        return "no answer in 5s";
    case Answer::LINK_DOWN:
        return "ESP-Hosted's link to it down";
    }
    return "?";
}

const char *statusText(wl_status_t st) {
    switch (st) {
    case WL_CONNECTED:
        return "connected";
    case WL_IDLE_STATUS:
        return "joined, no address yet";
    case WL_NO_SSID_AVAIL:
        return "the network not found";
    case WL_CONNECT_FAILED:
        return "joining failed";
    case WL_CONNECTION_LOST:
        return "the connection lost";
    case WL_DISCONNECTED:
        return "disconnected";
    case WL_STOPPED:
        return "the WiFi stopped";
    default:
        return "other";
    }
}

const char *restartText(Restart r) {
    return r == Restart::NOW ? "; a restart wanted now" : r == Restart::WHEN_IDLE ? "; a restart wanted when quiet" : "";
}

// The #243 stall, the crude way: ESP-Hosted's sdio_read task suspended, so
// nothing from the C6 is read from then on - no answers, no events, no
// data - which is what the stall would look like. Suspended only at a moment
// it is blocked, waiting for the C6, not part way through a read with the
// bus held; though blocked inside a read's transfer looks the same, which is
// why it is crude. Done from the loop, at once, not by the poll task: 'Z'
// must be free to let it go while the poll task waits on the C6, for a late
// answer to be tried. Written by whichever task acts, under the lock.
TaskHandle_t g_deafened = nullptr;

void benchDeafen(bool deafen) {
    if (!deafen) {
        portENTER_CRITICAL(&g_mux);
        TaskHandle_t t = g_deafened;
        g_deafened = nullptr;
        portEXIT_CRITICAL(&g_mux);
        if (t == nullptr) {
            Serial.println("[wifi] bench: ESP-Hosted's reads were not stopped");
            return;
        }
        vTaskResume(t);
        Serial.println("[wifi] bench: ESP-Hosted's sdio_read let go again - whatever the C6 sent meanwhile is read now");
        return;
    }
    if (g_deafened != nullptr) {
        Serial.println("[wifi] bench: ESP-Hosted's reads are already stopped");
        return;
    }
    TaskHandle_t t = xTaskGetHandle("sdio_read");
    if (t == nullptr) {
        Serial.println("[wifi] bench: no task called sdio_read - ESP-Hosted's names differ in this build");
        return;
    }
    for (int i = 0; i < 100; i++) {
        if (eTaskGetState(t) == eBlocked) {
            vTaskSuspend(t);
            portENTER_CRITICAL(&g_mux);
            g_deafened = t;
            portEXIT_CRITICAL(&g_mux);
            Serial.println("[wifi] bench: ESP-Hosted's sdio_read suspended as it waited for the C6 - nothing from the "
                           "C6 read from now on: no answers, no events, no data, as in the #243 stall; '##!Z' lets it "
                           "go, or the restart does");
            return;
        }
        vTaskDelay(1);
    }
    Serial.println("[wifi] bench: ESP-Hosted's sdio_read never seen waiting in 100ms - left alone");
}

// The #243 stall tried for real, as far as a bench can. In streaming mode
// ESP-Hosted reads from the C6 into a buffer, one each side of a double
// buffer, that grows to the largest burst it has had to read and never
// shrinks; and on 26 Oct the 3D view had internal DMA memory short. So: the
// internal DMA memory taken, a block at a time, until the largest piece
// left is under 4KB - small pieces left everywhere, for anything wanting a
// packet's worth - held for 20s for a burst from the C6, a flood ping from
// the PC or the next fetch, to need the buffer to grow and find no room;
// then given back. Soon after boot, before the buffers have grown, or
// nothing can fail; and off the 3D view, whose own allocations would fail
// too. In 2.12.13's source a read that cannot have its buffer is dropped
// and tried again a millisecond later, so the host should come back once
// the memory does. If it stays deaf - the link check unanswered, the router
// logging the C6 leaving with reason 4 - this is how 26 Oct began, and the
// ladder should go on to the restart.
constexpr uint32_t SQUEEZE_CAPS = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
constexpr size_t SQUEEZE_LEFT = 4096;
constexpr uint32_t SQUEEZE_MS = 20000;
constexpr int SQUEEZE_BLOCKS = 96;
void *g_squeeze[SQUEEZE_BLOCKS] = {};
int g_squeezed = 0;
uint32_t g_squeezeUntilMs = 0;  // 0 while nothing is held

void benchSqueeze() {
    if (g_squeezed > 0) {
        Serial.println("[wifi] bench: internal DMA memory already held");
        return;
    }
    size_t largestBefore = heap_caps_get_largest_free_block(SQUEEZE_CAPS);
    size_t held = 0;
    while (g_squeezed < SQUEEZE_BLOCKS) {
        size_t largest = heap_caps_get_largest_free_block(SQUEEZE_CAPS);
        if (largest < SQUEEZE_LEFT) {
            break;
        }
        // Half the limit left behind where this block was: a piece too
        // small for a burst, big enough for a packet.
        size_t take = largest - SQUEEZE_LEFT / 2;
        void *p = heap_caps_malloc(take, SQUEEZE_CAPS);
        if (p == nullptr) {
            break;
        }
        g_squeeze[g_squeezed++] = p;
        held += take;
    }
    uint32_t now = millis();
    g_squeezeUntilMs = (now + SQUEEZE_MS) != 0 ? now + SQUEEZE_MS : 1;
    Serial.printf("[wifi] bench: internal DMA memory held - %d blocks, %uKB; the largest piece left %u bytes (it "
                  "was %uKB); given back in %us. Send a burst now: a flood ping from the PC\n",
                  g_squeezed, (unsigned)(held / 1024),
                  (unsigned)heap_caps_get_largest_free_block(SQUEEZE_CAPS), (unsigned)(largestBefore / 1024),
                  (unsigned)(SQUEEZE_MS / 1000));
}

void benchSqueezeEnd(const char *why) {
    if (g_squeezed == 0) {
        return;
    }
    for (int i = 0; i < g_squeezed; i++) {
        heap_caps_free(g_squeeze[i]);
        g_squeeze[i] = nullptr;
    }
    g_squeezed = 0;
    g_squeezeUntilMs = 0;
    Serial.printf("[wifi] bench: internal DMA memory given back (%s); the largest piece now %uKB - if the C6 answers "
                  "again the host came back by itself, if not the ladder is on its way\n",
                  why, (unsigned)(heap_caps_get_largest_free_block(SQUEEZE_CAPS) / 1024));
}

void printState() {
    wl_status_t st = WiFi.status();
    esp_netif_dns_info_t dns = {};
    String resolver = "-";
    if (WiFi.STA.netif() && esp_netif_get_dns_info(WiFi.STA.netif(), ESP_NETIF_DNS_MAIN, &dns) == ESP_OK &&
        dns.ip.type == ESP_IPADDR_TYPE_V4) {
        resolver = IPAddress(dns.ip.u_addr.ip4.addr).toString();
    }
    Serial.printf("[wifi] state: the Tab5 believes %s (connected %s, address %s, router %s, resolver %s%s)\n",
                  statusText(st), WiFi.STA.connected() ? "yes" : "no", WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(), resolver.c_str(),
                  g_bench.dnsBroken ? ", broken by the bench" : "");
    if (g_rpcSuspect) {
        Serial.println("[wifi] state: the C6 not asked - an earlier request went unanswered, and every answer since "
                       "could be one out of step");
    } else {
        RadioTurn turn;
        if (g_hooks.uiOwnsWifi()) {
            Serial.println("[wifi] state: the C6 not asked - the WiFi screen has the radio");
        } else {
            Probe p = askC6();
            uint8_t primary = 0;
            wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
            bool haveChannel = p.answer == Answer::ON_AP && esp_wifi_get_channel(&primary, &second) == ESP_OK;
            Serial.printf("[wifi] state: the C6 says %s (%s) in %ums; signal %s, channel %s\n", answerText(p.answer),
                          esp_err_to_name(p.err), (unsigned)p.tookMs,
                          p.answer == Answer::ON_AP ? (String(p.rssi) + " dBm").c_str() : "-",
                          haveChannel ? String(primary).c_str() : "-");
        }
    }
    uint32_t routerMs = 0;
    Reach r = st == WL_CONNECTED ? routerReach(&routerMs) : Reach::UNKNOWN;
    Serial.printf("[wifi] state: ESP-Hosted %u.%u.%u on the Tab5, %u.%u.%u on the C6; the router %s (%ums)%s%s%s\n",
                  (unsigned)g_hostMajor, (unsigned)g_hostMinor, (unsigned)g_hostPatch, (unsigned)g_c6Major,
                  (unsigned)g_c6Minor, (unsigned)g_c6Patch,
                  r == Reach::ANSWERS     ? "answers"
                  : r == Reach::SILENT    ? "does not answer"
                  : r == Reach::NO_SOCKET ? "not asked - no socket to be had"
                                          : "not asked",
                  (unsigned)routerMs, g_bench.skipRouter ? ", though the watchdog takes it as silent" : "",
                  g_deafened != nullptr ? "; ESP-Hosted's reads stopped by the bench" : "",
                  g_squeezed > 0 ? "; internal DMA memory held by the bench" : "");
    const AdsbFeedHealth &h = adsbFeedHealth();
    uint32_t now = millis();
    Serial.printf("[wifi] state: the feed: %u requests, %u answered; the last answer %s; %u network failures in a "
                  "row%s%s\n",
                  (unsigned)h.requests, (unsigned)h.answers,
                  h.lastAnswerMs ? (took(now - h.lastAnswerMs) + " ago").c_str() : "never", (unsigned)h.netFails,
                  h.netFails ? ", the last: " : "", h.netFails ? h.lastNetFail : "");
    Serial.printf("[wifi] state: the watchdog: %s; checks every 10s%s (the last %s); %u joinings and %u addresses "
                  "since boot; %u requests unanswered in a row; restarts in a row %u%s\n",
                  g_ep.active ? ("in trouble for " + took(now - g_ep.startMs) + ", step " + String(g_ep.lastStep) +
                                 " done " + took(now - g_ep.lastStepMs) + " ago" +
                                 (g_ep.restartAsked ? ", a restart asked for" : ""))
                                    .c_str()
                              : "no trouble",
                  g_rpcSuspect          ? " - stopped, a request went unanswered"
                  : g_bench.probePaused ? " - paused by the bench"
                                        : "",
                  g_probed ? (String(answerText(g_probe.answer)) + ", " + took(now - g_probe.atMs) + " ago").c_str()
                           : "none yet",
                  (unsigned)g_assocs, (unsigned)g_gotIps, (unsigned)g_unansweredRun, (unsigned)g_restartsInARow,
                  restartText(g_restart));
}

// The stuck state of 26 Oct, made: the C6 told to leave the network while the
// P4's handler for the C6's "disconnected" is taken out of ESP-Hosted's
// table, so the event that says so comes across and is dropped, and put back
// three seconds later. What is left is what the router and the serial log
// showed: the C6 off the network - the router logs it leaving, though with
// "leaving" (8) for its reason, not inactivity (4) - and the P4 believing it
// is connected, its WiFi.status(), the framework's "connected" and
// ESP-Hosted's own flags all as they were, every request failing at the name
// lookup. What it does not make is the cause: the P4 still reads from the
// C6, and its requests are answered, where on 26 Oct, if ESP-Hosted's reads
// had stopped, a request would get no answer at all. So it tries the steps
// that need the C6 to answer - the check, the rejoin - and not the restart,
// which needs a stuck link: "##!z" makes one, and "##!m" tries for the real
// thing.
void benchStuck(bool pauseProbe) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[wifi] bench: not connected now - nothing to cut off");
        return;
    }
    if (g_rpcSuspect) {
        Serial.println("[wifi] bench: a request to the C6 has already gone unanswered - restart first");
        return;
    }
    constexpr int EVENT_STA_DISCONNECTED = 776;  // RPC_ID__Event_StaDisconnected, in 1.4.1's protocol and 2.12.13's
    esp_err_t err;
    {
        RadioTurn turn;
        if (g_hooks.uiOwnsWifi()) {
            Serial.println("[wifi] bench: the WiFi screen has the radio - not done");
            return;
        }
        reset_event_callback(EVENT_STA_DISCONNECTED);
        err = esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(3000));
        rpc_register_event_callbacks();
    }
    g_quietFromMs = millis();  // the link check's usual wait, as after a step - not a step itself
    bool stuck = WiFi.status() == WL_CONNECTED && WiFi.STA.connected();
    g_bench.probePaused = pauseProbe && stuck;
    Serial.printf("[wifi] bench: the C6 told to leave the network (%s) with the Tab5 not told; the Tab5 %s%s\n",
                  esp_err_to_name(err),
                  stuck ? "still believes it is connected - the stuck state" : "heard anyway - it did not take",
                  g_bench.probePaused ? "; the link check paused, for the feed's watchdog to find it" : "");
}

// Name lookups broken: every DNS server pointed at 192.0.2.1, an address kept
// for documentation that nothing answers, and the names already looked up
// forgotten, so every lookup waits and fails as it did on 26 Oct - "DNS
// Failed ... error -54" - and the feed's watchdog has something to count.
// The router still answers, so the watchdog should say the trouble is
// beyond it and leave the WiFi alone; with "##!r" it takes the router as
// silent and goes on to the ladder instead, whose rejoin gets an address
// and the router's own DNS server afresh.
struct DnsJob {
    bool breakIt = false;
    SemaphoreHandle_t done = nullptr;
};
DnsJob g_dnsJob;
ip_addr_t g_savedDns[DNS_MAX_SERVERS];

void dnsJob(void *arg) {
    DnsJob *job = (DnsJob *)arg;
    if (job->breakIt) {
        if (!g_bench.dnsBroken) {
            for (int i = 0; i < DNS_MAX_SERVERS; i++) {
                g_savedDns[i] = *dns_getserver(i);
            }
        }
        ip_addr_t nowhere;
        IP_ADDR4(&nowhere, 192, 0, 2, 1);
        for (int i = 0; i < DNS_MAX_SERVERS; i++) {
            dns_setserver(i, &nowhere);
        }
    } else {
        for (int i = 0; i < DNS_MAX_SERVERS; i++) {
            dns_setserver(i, &g_savedDns[i]);
        }
    }
    dns_clear_cache();
    xSemaphoreGive(job->done);
}

void benchDns(bool breakIt) {
    if (!breakIt && !g_bench.dnsBroken) {
        Serial.println("[wifi] bench: name lookups were not broken");
        return;
    }
    if (g_dnsJob.done == nullptr) {
        g_dnsJob.done = xSemaphoreCreateBinary();
    }
    g_dnsJob.breakIt = breakIt;
    if (tcpip_callback(dnsJob, &g_dnsJob) != ERR_OK || xSemaphoreTake(g_dnsJob.done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.println("[wifi] bench: lwIP did not take the change");
        return;
    }
    g_bench.dnsBroken = breakIt;
    adsbDropConnection();  // or the kept connection carries on without a lookup
    Serial.println(breakIt ? "[wifi] bench: name lookups broken - every DNS server 192.0.2.1, the cache emptied"
                           : "[wifi] bench: name lookups put back as they were");
}

void benchCommandNow(char c) {
    switch (c) {
    case 's':
        printState();
        break;
    case 'x':
    case 'X':
        benchStuck(c == 'X');
        break;
    case 'd':
        benchDns(true);
        break;
    case 'D':
        benchDns(false);
        break;
    case 'r':
        g_bench.skipRouter = !g_bench.skipRouter;
        Serial.printf("[wifi] bench: the router %s\n",
                      g_bench.skipRouter ? "taken as silent by the watchdog" : "asked again as it should be");
        break;
    case 'm':
        benchSqueeze();
        break;
    case 'M':
        benchSqueezeEnd("from the bench");
        break;
    case '1':
    case '2': {
        Serial.printf("[wifi] bench: step %c on demand\n", c);
        String said;
        uint32_t t0 = millis();
        {
            RadioTurn turn;
            if (c == '1') {
                stepRejoin(said);
            } else {
                stepRestartWifi(said);
            }
        }
        g_quietFromMs = millis();
        Serial.printf("[wifi] bench: step %c done in %.1fs: %s\n", c, (millis() - t0) / 1000.0f, said.c_str());
        break;
    }
    case '3':
        setRestart(Restart::NOW, "asked for from the bench");
        Serial.println("[wifi] bench: step 3 on demand - the restart asked for now");
        break;
    case '4':
        setRestart(Restart::WHEN_IDLE, "asked for from the bench");
        Serial.println("[wifi] bench: the restart asked for at the next quiet moment");
        break;
    case '0':
        if (g_bench.dnsBroken) {
            benchDns(false);
        }
        g_bench.skipRouter = false;
        g_bench.probePaused = false;
        benchSqueezeEnd("from the bench");
        if (g_deafened != nullptr) {
            benchDeafen(false);
        }
        if (g_ep.active) {
            endEpisode("ended from the bench");
        }
        setRestart(Restart::NONE, "");  // though one for a request unanswered is asked again straight away
        Serial.println("[wifi] bench: the bench's changes undone");
        break;
    default:
        Serial.printf("[wifi] bench: no command '%c'\n", c);
        break;
    }
}
#endif

void standDown() {
    if (g_ep.active) {
        Serial.println("[wifi] watchdog: standing down - the WiFi screen has the radio, or is about to");
        g_ep = Episode();
        uint32_t now = millis();
        g_lastEpisodeEndMs = now != 0 ? now : 1;
    }
    g_offApRun = 0;
    // The WiFi is the screen's to mend now, so a restart for the WiFi down
    // is not wanted - only one for answers perhaps out of step.
    if (g_restart == Restart::NOW) {
        settleRestart();
    }
}

const char *resetText(esp_reset_reason_t r) {
    switch (r) {
    case ESP_RST_POWERON:
        return "power-on";
    case ESP_RST_EXT:
        return "the reset pin";
    case ESP_RST_SW:
        return "a software restart";
    case ESP_RST_PANIC:
        return "a crash";
    case ESP_RST_INT_WDT:
        return "the interrupt watchdog";
    case ESP_RST_TASK_WDT:
        return "the task watchdog";
    case ESP_RST_WDT:
        return "a hardware watchdog";
    case ESP_RST_DEEPSLEEP:
        return "waking from deep sleep";
    case ESP_RST_BROWNOUT:
        return "a brownout";
    case ESP_RST_USB:
        return "USB";
    case ESP_RST_JTAG:
        return "JTAG";
    case ESP_RST_PWR_GLITCH:
        return "a power glitch";
    case ESP_RST_CPU_LOCKUP:
        return "a CPU lock-up";
    default:
        return "something unknown";
    }
}

}  // namespace

bool noteBoot() {
    esp_reset_reason_t why = esp_reset_reason();
    bool valid = g_note.magic == NOTE_MAGIC && g_note.check == noteCheck(g_note);
    bool ours = why == ESP_RST_SW && valid && g_note.armed;
    if (ours) {
        g_restartsInARow = g_note.inARow;
        g_bootAfterQuiet = (g_note.flags & NOTE_QUIET) != 0;
        g_bootOpensWifi = (g_note.flags & NOTE_THEN_WIFI) != 0;
        if (g_bootAfterQuiet) {
            Serial.printf("[boot] started by the WiFi watchdog's restart at a quiet moment%s: %s\n",
                          g_bootOpensWifi ? ", made as the WiFi screen was opened" : "", g_note.why);
        } else {
            Serial.printf("[boot] started by the WiFi watchdog's restart, %u in a row%s: %s\n",
                          (unsigned)g_restartsInARow, g_bootOpensWifi ? ", made as the WiFi screen was opened" : "",
                          g_note.why);
        }
        saveNote(false, g_restartsInARow, 0, g_note.why);
    } else {
        g_restartsInARow = 0;
        Serial.printf("[boot] started by %s%s\n", resetText(why),
                      why == ESP_RST_SW ? " - not the WiFi watchdog's; ESP-Hosted restarts the Tab5 itself when its "
                                          "link to the C6 fails"
                                        : "");
        saveNote(false, 0, 0, "");
    }
    return ours;
}

bool bootOpensWifiScreen() { return g_bootOpensWifi; }

void begin(const Hooks &hooks) {
    g_hooks = hooks;
    hostedGetHostVersion(&g_hostMajor, &g_hostMinor, &g_hostPatch);
    hostedGetSlaveVersion(&g_c6Major, &g_c6Minor, &g_c6Patch);
    // The C6's is what it answered when asked as ESP-Hosted came up, a request
    // given a second: 0.0.0 if it did not answer in that.
    if (!hostedIsInitialized()) {
        Serial.printf("[wifi] ESP-Hosted %u.%u.%u did not come up - no link to the C6\n", (unsigned)g_hostMajor,
                      (unsigned)g_hostMinor, (unsigned)g_hostPatch);
    } else if (g_c6Major == 0 && g_c6Minor == 0 && g_c6Patch == 0) {
        Serial.printf("[wifi] ESP-Hosted %u.%u.%u on the Tab5; the C6's not known - it did not answer when asked\n",
                      (unsigned)g_hostMajor, (unsigned)g_hostMinor, (unsigned)g_hostPatch);
    } else {
        Serial.printf("[wifi] ESP-Hosted %u.%u.%u on the Tab5, %u.%u.%u on the C6\n", (unsigned)g_hostMajor,
                      (unsigned)g_hostMinor, (unsigned)g_hostPatch, (unsigned)g_c6Major, (unsigned)g_c6Minor,
                      (unsigned)g_c6Patch);
    }
    esp_event_handler_instance_t instance;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onLinkEvent, nullptr, &instance);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onLinkEvent, nullptr, &instance);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_LOST_IP, onLinkEvent, nullptr, &instance);
    esp_event_handler_instance_register(ESP_HOSTED_EVENT, ESP_EVENT_ANY_ID, onHostedEvent, nullptr, &instance);
    Network.onEvent(logEvent);
    heap_caps_register_failed_alloc_callback(onAllocFailed);
    g_begun = true;
}

void tick() {
    if (!g_begun) {
        return;
    }
    uint32_t now = millis();
    noteGotIps(now);
    forgetOldTimes(now);
    sayWhatHappened();
#ifdef BENCH3D
    char c = g_benchCmd;
    if (c) {
        g_benchCmd = 0;
        benchCommandNow(c);
    }
    if (g_squeezeUntilMs != 0 && (int32_t)(millis() - g_squeezeUntilMs) >= 0) {
        benchSqueezeEnd("its 20s up");
    }
#endif
    if (g_hooks.uiOwnsWifi()) {
        standDown();
        return;
    }
    // A restart for answers perhaps out of step, once there is no trouble
    // for one to be asked as it ends - and again once twelve hours of the
    // boot after the last have passed: see settleRestart().
    if (g_rpcSuspect && !g_ep.active && g_restart == Restart::NONE) {
        settleRestart();
    }
    now = millis();
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
        maybeProbe(now);
        now = millis();
    }
    if (!g_ep.active) {
        if (st == WL_CONNECTED) {
            checkFeed(now);
        }
        return;
    }
    // A feed that answers ends it at once, whatever the step's time.
    if (adsbFeedHealth().answers != g_ep.answersAtStep) {
        endEpisode("recovered - the feed answers");
        return;
    }
    if ((int32_t)(now - g_ep.nextMs) >= 0) {
        decide(now, st);
    }
}

RadioTurn::RadioTurn() {
    if (g_radio != nullptr) {
        xSemaphoreTakeRecursive(g_radio, portMAX_DELAY);
    }
}

RadioTurn::~RadioTurn() {
    if (g_radio != nullptr) {
        xSemaphoreGiveRecursive(g_radio);
    }
}

bool radioFree() {
    if (g_radio == nullptr) {
        return true;
    }
    if (xSemaphoreTakeRecursive(g_radio, 0) != pdTRUE) {
        return false;
    }
    xSemaphoreGiveRecursive(g_radio);
    return true;
}

String joinedNetwork(uint32_t *joinings) {
    char name[sizeof g_joinedSsid];
    portENTER_CRITICAL(&g_mux);
    memcpy(name, g_joinedSsid, sizeof name);
    uint32_t n = g_assocs;
    portEXIT_CRITICAL(&g_mux);
    if (joinings != nullptr) {
        *joinings = n;
    }
    return String(name);
}

Restart restartWanted() { return g_restart; }

const char *restartReason() {
    static char why[96];
    portENTER_CRITICAL(&g_mux);
    strlcpy(why, g_restartWhy, sizeof why);
    portEXIT_CRITICAL(&g_mux);
    return why;
}

bool restartBeforeWifiScreen() { return g_rpcSuspect && g_restart != Restart::NONE; }

void restartNow(bool thenWifiScreen) {
    const char *why = restartReason();
    portENTER_CRITICAL(&g_mux);
    bool quiet = g_restart == Restart::WHEN_IDLE;
    uint32_t inARow = g_restartsInARow;
    portEXIT_CRITICAL(&g_mux);
    // One at a quiet moment, the WiFi working, is no restart that failed to
    // bring the feed back: the count of those is left as it was.
    saveNote(true, quiet ? inARow : inARow + 1, (quiet ? NOTE_QUIET : 0) | (thenWifiScreen ? NOTE_THEN_WIFI : 0),
             why);
    Serial.printf("[wifi] watchdog: restarting the Tab5 %s - %s\n",
                  quiet ? "at a quiet moment" : "to bring the WiFi back", why);
    Serial.flush();
    // The card's lock taken first, and kept: whatever was writing to the card
    // - the poll task a recording's frame, the share page's server a
    // deletion - has finished, and nothing can start another, so the restart
    // cuts no file off part way.
    recorder::CardLock card;
    // The C6's power cut for half a second, through the IO expander's
    // WLAN_PWR_EN, so that it starts from nothing whether or not the reset
    // line ESP-Hosted pulses at boot reaches it on this board - one report
    // has a stuck Tab5 C6 deaf to those pulses and revived only by its power.
    // ESP-Hosted may notice the C6 gone in that half second and restart the
    // Tab5 itself, which is as good; the note above is already written.
    M5.getIOExpander(1).digitalWrite(0, false);
    delay(500);
    M5.getIOExpander(1).digitalWrite(0, true);
    esp_restart();
}

void benchCommand(char c) {
#ifdef BENCH3D
    if (c == 'z' || c == 'Z') {
        benchDeafen(c == 'z');  // at once, from the loop: see benchDeafen()
        return;
    }
    g_benchCmd = c;
#else
    (void)c;
#endif
}

}  // namespace wifiwatch
