#include "route.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <map>

#include "config.h"
#include "psram_alloc.h"

namespace {

struct Entry {
    RouteState state = RouteState::NONE;
    Route route;
};
// The answers so far, by callsign - a few dozen at most, the oldest let go.
constexpr size_t MAX_KEPT = 48;
std::map<String, Entry, std::less<String>, PsramAllocator<std::pair<const String, Entry>>> g_routes;  // in the PSRAM
SemaphoreHandle_t g_lock = nullptr;
QueueHandle_t g_queue = nullptr;
volatile uint32_t g_generation = 0;
constexpr size_t CALLSIGN_MAX = 9;

// A callsign there can be a route for: an airline's three letters and its
// number - not a registration, which an aircraft without a callsign goes by,
// nor its hex.
bool routable(const String &cs) {
    if (cs.length() < 4 || cs.length() > 8) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (!isalpha((unsigned char)cs[i])) {
            return false;
        }
    }
    for (size_t i = 3; i < cs.length(); i++) {
        if (!isalnum((unsigned char)cs[i])) {
            return false;
        }
    }
    return isdigit((unsigned char)cs[3]);
}

void store(const String &cs, RouteState state, const Route &route) {
    xSemaphoreTake(g_lock, portMAX_DELAY);
    if (state == RouteState::NONE) {
        g_routes.erase(cs);  // couldn't be asked: asked again next time
    } else {
        g_routes[cs] = {state, route};
    }
    g_generation = g_generation + 1;
    xSemaphoreGive(g_lock);
}

void lookup(const String &cs) {
    if (WiFi.status() != WL_CONNECTED) {
        store(cs, RouteState::NONE, Route());
        return;
    }
    WiFiClientSecure tls;
    tls.setInsecure();  // as the feed's: no CA bundle kept for a hobby project
    HTTPClient http;
    http.setTimeout(8000);
    http.setUserAgent("OverheadFlightTracker/" FIRMWARE_VERSION
                      " (+https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-cpp)");
    if (!http.begin(tls, "https://api.adsbdb.com/v0/callsign/" + cs)) {
        store(cs, RouteState::NONE, Route());
        return;
    }
    int code = http.GET();
    if (code != 200 && code != 404) {
        http.end();
        store(cs, RouteState::NONE, Route());
        return;
    }
    // Only what is shown: the two ends' codes and towns.
    JsonDocument filter;
    for (const char *end : {"origin", "destination"}) {
        filter["response"]["flightroute"][end]["iata_code"] = true;
        filter["response"]["flightroute"][end]["municipality"] = true;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
    http.end();
    JsonVariantConst r = doc["response"]["flightroute"];
    if (err || r.isNull()) {
        store(cs, RouteState::UNKNOWN, Route());  // "unknown callsign"
        return;
    }
    Route route;
    route.fromCode = (const char *)(r["origin"]["iata_code"] | "");
    route.fromCity = (const char *)(r["origin"]["municipality"] | "");
    route.toCode = (const char *)(r["destination"]["iata_code"] | "");
    route.toCity = (const char *)(r["destination"]["municipality"] | "");
    store(cs, route.fromCode.length() && route.toCode.length() ? RouteState::FOUND : RouteState::UNKNOWN, route);
}

void routeTask(void *) {
    char cs[CALLSIGN_MAX];
    for (;;) {
        if (xQueueReceive(g_queue, cs, portMAX_DELAY) == pdTRUE) {
            lookup(String(cs));
        }
    }
}

}  // namespace

void routeRequest(const String &callsign) {
    String cs = callsign;
    cs.trim();
    cs.toUpperCase();
    if (!routable(cs)) {
        return;
    }
    if (!g_lock) {
        g_lock = xSemaphoreCreateMutex();
        g_queue = xQueueCreate(4, CALLSIGN_MAX);
        // On the core the feed's polls are on, out of the screen's way; TLS
        // wants the stack.
        xTaskCreatePinnedToCore(routeTask, "route", 8192, nullptr, 1, nullptr, 0);
    }
    xSemaphoreTake(g_lock, portMAX_DELAY);
    bool asked = g_routes.count(cs) > 0;
    if (!asked) {
        if (g_routes.size() >= MAX_KEPT) {
            g_routes.erase(g_routes.begin());
        }
        g_routes[cs].state = RouteState::LOOKING;
    }
    xSemaphoreGive(g_lock);
    if (!asked) {
        char buf[CALLSIGN_MAX] = {};
        strlcpy(buf, cs.c_str(), sizeof(buf));
        if (xQueueSend(g_queue, buf, 0) != pdTRUE) {
            store(cs, RouteState::NONE, Route());  // a backlog: let it be asked again
        }
    }
}

RouteState routeFor(const String &callsign, Route &out) {
    if (!g_lock) {
        return RouteState::NONE;
    }
    String cs = callsign;
    cs.trim();
    cs.toUpperCase();
    xSemaphoreTake(g_lock, portMAX_DELAY);
    auto it = g_routes.find(cs);
    RouteState state = it == g_routes.end() ? RouteState::NONE : it->second.state;
    if (state == RouteState::FOUND) {
        out = it->second.route;
    }
    xSemaphoreGive(g_lock);
    return state;
}

uint32_t routeGeneration() { return g_generation; }
