#include "adsb_client.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <math.h>

#include "config.h"

static float haversineNm(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R_KM = 6371.0;
    double p1 = radians(lat1), p2 = radians(lat2);
    double dphi = radians(lat2 - lat1);
    double dlambda = radians(lon2 - lon1);
    double a = sin(dphi / 2) * sin(dphi / 2) + cos(p1) * cos(p2) * sin(dlambda / 2) * sin(dlambda / 2);
    double km = 2 * R_KM * asin(sqrt(a));
    return (float)(km * 0.539957);
}

// The parsed document is the largest thing this task holds, and a busy radius
// at the military range ceiling is its worst case. Putting it in PSRAM keeps
// that peak off the 512KB of internal RAM, which is what the WiFi and TLS
// stacks are competing for. PSRAM is the slower memory, so this trades a
// little parse time for headroom where headroom is scarce.
struct PsramAllocator : ArduinoJson::Allocator {
    void *allocate(size_t size) override { return heap_caps_malloc(size, MALLOC_CAP_SPIRAM); }
    void deallocate(void *ptr) override { heap_caps_free(ptr); }
    void *reallocate(void *ptr, size_t size) override { return heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM); }
};
static PsramAllocator g_psram;

// adsb.lol returns around forty fields per aircraft; this app reads fifteen.
// Handing ArduinoJson a filter means the rest are skipped in the tokeniser
// rather than allocated into the document and then ignored - on a busy 60nm
// radius that is the difference between a document of a few hundred KB and one
// of a few tens.
static void buildFilter(JsonDocument &filter) {
    // A filter for an array is a one-element array: that element is applied to
    // every entry in turn.
    JsonObject ac = filter["ac"].add<JsonObject>();
    for (const char *key : {"hex", "flight", "t", "dbFlags", "alt_baro", "gs", "lat", "lon", "r", "squawk",
                            "category", "track", "alt_geom", "baro_rate", "geom_rate"}) {
        ac[key] = true;
    }
}

// Held for the life of the firmware rather than built per poll, so the TLS
// session survives the gap between polls: HTTPClient keeps the socket open
// when _reuse is set and the response allows it, and stores the client by
// reference, so the next fetch skips DNS, TCP and the handshake entirely.
// The cost is one mbedTLS context resident instead of one per fetch.
static WiFiClientSecure g_tls;
static HTTPClient g_http;
static bool g_tlsConfigured = false;

// adsb.lol answers 429 when it has had enough, and going straight back at the
// same cadence only collects more of them. After one, the next minute of polls
// are skipped without a request going out at all - the status line already
// says the data is ageing, which is the honest thing to show meanwhile.
constexpr uint32_t RATE_LIMIT_BACKOFF_MS = 60000;
static uint32_t g_rateLimitedUntil = 0;

// HTTPClient always sends its own internal _userAgent field regardless of
// addHeader("User-Agent", ...) (see HTTPClient.cpp ~line 1226), so that call
// was silently overridden by the default "ESP32HTTPClient" - which is exactly
// the kind of generic string adsb.lol rejects. setUserAgent() is the actual
// override point.
static const char *USER_AGENT =
    "OverheadFlightTracker/1.0 (+https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-cpp)";

#ifdef NET_PROFILE
// Set inside requestInto so the request and the parse stay separable now that
// both happen in there.
static uint32_t g_tAfterGet = 0, g_tAfterParse = 0;
#endif

// One request on the shared client. Returns the HTTP status, or a negative
// HTTPClient error for a transport-level failure.
static int requestInto(const char *url, JsonDocument &doc, DeserializationError &jerr) {
    if (!g_http.begin(g_tls, url)) {
        Serial.println("[adsb] http.begin() failed");
        return HTTPC_ERROR_CONNECTION_REFUSED;
    }
    g_http.setUserAgent(USER_AGENT);
    g_http.setReuse(true);
    // Longer than the 5s default: a served request takes ~45ms, but adsb.lol
    // delays rather than refuses when it is throttling, and a reply that takes
    // several seconds is still worth having. Only the poll task waits on this.
    g_http.setTimeout(12000);

    int code = g_http.GET();
#ifdef NET_PROFILE
    g_tAfterGet = millis();
#endif
    if (code != 200) {
        if (code > 0) {
            // Only worth a copy of the body when the server actually answered.
            String payload = g_http.getString();
            Serial.printf("[adsb] status=%d body=%s\n", code, payload.c_str());
        }
        g_http.end();
        return code;
    }

    JsonDocument filter;  // a handful of keys - not worth the slower memory
    buildFilter(filter);
    // Parsed straight off the socket where the response declared a length: a
    // busy radius returns a couple of hundred KB, and holding that as a String
    // on top of the document it is about to become was the peak of this task's
    // heap use for no benefit.
    //
    // Only where it declared a length, though - HTTPClient de-chunks on the
    // getString()/writeToStream() paths but not on the raw stream, so a
    // chunked reply read straight would have the chunk headers still in it.
    if (g_http.getSize() >= 0) {
        jerr = deserializeJson(doc, g_http.getStream(), DeserializationOption::Filter(filter));
    } else {
        String payload = g_http.getString();
        jerr = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
    }
#ifdef NET_PROFILE
    g_tAfterParse = millis();
#endif
    // Leaves the socket open for the next poll; HTTPClient drains whatever the
    // filter stopped short of reading.
    g_http.end();
    return code;
}

bool fetchAircraft(double lat, double lon, int radiusNm, std::vector<Aircraft> &out) {
    char url[160];
    snprintf(url, sizeof(url), ADSB_API_URL_FMT, lat, lon, radiusNm);
#ifdef NET_PROFILE
    uint32_t tStart = millis();
#endif

    if (g_rateLimitedUntil != 0) {
        if ((int32_t)(millis() - g_rateLimitedUntil) < 0) {
            return false;  // still inside the backoff - don't even ask
        }
        g_rateLimitedUntil = 0;
    }

    if (!g_tlsConfigured) {
        g_tls.setInsecure();  // no CA bundle managed for this hobby project
        g_tlsConfigured = true;
    }

    JsonDocument doc(&g_psram);
    DeserializationError jerr;
    int code = requestInto(url, doc, jerr);
    if (code < 0) {
        // However it failed, a kept connection is no longer trustworthy.
        g_http.end();
        g_tls.stop();
        // Only a socket that died *before* the request landed is worth sending
        // again. A read timeout means the server has the request and is merely
        // slow, so retrying it would put two of the same request on an endpoint
        // that - on this evidence - is already asking us to ease off.
        bool socketDied = (code == HTTPC_ERROR_CONNECTION_LOST || code == HTTPC_ERROR_SEND_HEADER_FAILED ||
                           code == HTTPC_ERROR_SEND_PAYLOAD_FAILED || code == HTTPC_ERROR_NOT_CONNECTED);
        if (!socketDied) {
            Serial.printf("[adsb] transport error %d\n", code);
            return false;
        }
        Serial.printf("[adsb] connection %d on a reused socket, retrying once\n", code);
        code = requestInto(url, doc, jerr);
    }
    if (code == 429) {
        g_rateLimitedUntil = millis() + RATE_LIMIT_BACKOFF_MS;
        Serial.printf("[adsb] rate limited, backing off %us\n", (unsigned)(RATE_LIMIT_BACKOFF_MS / 1000));
        return false;
    }
    if (code != 200) {
        return false;
    }
    if (jerr) {
        Serial.printf("[adsb] json parse error: %s\n", jerr.c_str());
        return false;
    }

    std::vector<Aircraft> result;
    for (JsonObjectConst ac : doc["ac"].as<JsonArrayConst>()) {
        const char *hex = ac["hex"] | "";
        if (!hex[0]) {
            continue;
        }

        JsonVariantConst altV = ac["alt_baro"];
        bool onGround = altV.is<const char *>();
        float altNum = 0;
        if (!onGround && !altV.isNull()) {
            altNum = altV.as<float>();
            if (altNum < 0) {
                continue;  // bogus negative-altitude reports occasionally show up
            }
        }

        Aircraft a;
        a.hex = hex;
        String cs = String((const char *)(ac["flight"] | ""));
        cs.trim();
        a.callsign = cs.length() ? cs : a.hex;
        a.type = (const char *)(ac["t"] | "----");
        // adsb.lol only emits dbFlags for aircraft it has flagged; bit 0 is
        // the military one (bits 1/2/3 are interesting/PIA/LADD).
        a.military = ((ac["dbFlags"] | 0) & 1) != 0;

        if (onGround) {
            a.altStr = "GND";
        } else if (altV.isNull()) {
            a.altStr = "?";
        } else {
            a.altStr = String((long)altNum);
        }

        a.speedStr = ac["gs"].isNull() ? "?" : String((long)lround(ac["gs"].as<float>()));

        if (!ac["lat"].isNull() && !ac["lon"].isNull()) {
            a.hasDist = true;
            a.distNm = haversineNm(lat, lon, ac["lat"].as<double>(), ac["lon"].as<double>());
            a.hasPos = true;
            a.lat = ac["lat"].as<float>();
            a.lon = ac["lon"].as<float>();
        } else {
            a.hasDist = false;
            a.distNm = 0;
            a.hasPos = false;
        }

        a.reg = ac["r"].isNull() ? "" : String((const char *)ac["r"]);
        a.squawk = ac["squawk"].isNull() ? "" : String((const char *)ac["squawk"]);
        a.category = ac["category"].isNull() ? "" : String((const char *)ac["category"]);

        if (!ac["track"].isNull()) {
            a.hasTrack = true;
            a.track = ac["track"].as<float>();
        } else {
            a.hasTrack = false;
        }

        if (!ac["alt_geom"].isNull()) {
            a.hasAltGeom = true;
            a.altGeom = ac["alt_geom"].as<int>();
        } else {
            a.hasAltGeom = false;
        }

        a.hasVertRate = !ac["baro_rate"].isNull() || !ac["geom_rate"].isNull();
        a.vertRate = ac["baro_rate"].isNull() ? ac["geom_rate"] | 0.0f : ac["baro_rate"].as<float>();

        if (onGround) {
            float gs = ac["gs"] | -1.0f;
            a.status = (gs > 2) ? "TAXI" : "GROUND";
        } else {
            float rate = a.vertRate;
            if (rate >= CLIMB_THRESHOLD_FPM) {
                a.status = "CLIMB";
            } else if (rate <= DESCEND_THRESHOLD_FPM) {
                a.status = "DESCEND";
            } else {
                a.status = "LEVEL";
            }
        }

        result.push_back(a);
    }

    std::sort(result.begin(), result.end(), [](const Aircraft &a, const Aircraft &b) {
        float da = a.hasDist ? a.distNm : 1e9f;
        float db = b.hasDist ? b.distNm : 1e9f;
        return da < db;
    });

#ifdef NET_PROFILE
    Serial.printf("[net] request %lums  parse %lums  build %lums  total %lums  %u aircraft\n",
                  (unsigned long)(g_tAfterGet - tStart), (unsigned long)(g_tAfterParse - g_tAfterGet),
                  (unsigned long)(millis() - g_tAfterParse), (unsigned long)(millis() - tStart),
                  (unsigned)result.size());
#endif
    out = std::move(result);
    return true;
}
