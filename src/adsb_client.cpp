#include "adsb_client.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <math.h>

#include "config.h"

float haversineNm(double lat1, double lon1, double lat2, double lon2) {
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

// These endpoints return around forty fields per aircraft; this app reads
// fifteen. Handing ArduinoJson a filter means the rest are skipped in the
// tokeniser rather than allocated into the document and then ignored - on a
// busy 60nm radius, or on a global military feed, that is the difference
// between a document of a few hundred KB and one of a few tens.
static void buildFilter(JsonDocument &filter, const char *arrayKey) {
    // A filter for an array is a one-element array: that element is applied to
    // every entry in turn. The key it hangs off differs by provider - `ac` on
    // adsb.lol, `aircraft` on an adsb.fi point query - so it comes from the
    // endpoint rather than being spelled here.
    JsonObject ac = filter[arrayKey].add<JsonObject>();
    for (const char *key : {"hex", "flight", "t", "dbFlags", "alt_baro", "gs", "lat", "lon", "r", "squawk",
                            "category", "track", "alt_geom", "baro_rate", "geom_rate"}) {
        ac[key] = true;
    }
    JsonObject last = ac["lastPosition"].to<JsonObject>();
    last["lat"] = true;
    last["lon"] = true;
    last["seen_pos"] = true;
}

// {lat}/{lon}/{radius} substitution rather than a printf format held in the
// provider table. The templates are data, and a format string reaching
// snprintf from anywhere but a literal is how a double gets read as a pointer
// - a mis-spelled placeholder here can only ever produce a wrong URL.
static String buildUrl(const AdsbEndpoint &ep, double lat, double lon, int radiusNm) {
    String url(ep.urlTemplate);
    char buf[24];
    snprintf(buf, sizeof(buf), "%.4f", lat);
    url.replace("{lat}", buf);
    snprintf(buf, sizeof(buf), "%.4f", lon);
    url.replace("{lon}", buf);
    url.replace("{radius}", String(radiusNm));
    return url;
}

// Just the host, to notice when a poll is going somewhere other than where the
// kept-alive socket is pointed.
static String hostOf(const String &url) {
    int start = url.indexOf("//");
    start = (start < 0) ? 0 : start + 2;
    int end = url.indexOf('/', start);
    return (end < 0) ? url.substring(start) : url.substring(start, end);
}

// Held for the life of the firmware rather than built per poll, so the TLS
// session survives the gap between polls: HTTPClient keeps the socket open
// when _reuse is set and the response allows it, and stores the client by
// reference, so the next fetch skips DNS, TCP and the handshake entirely.
// The cost is one mbedTLS context resident instead of one per fetch.
// Its reads, though, made to wait for the network by sleeping. The
// framework's readBytes() reads in blocks but takes "nothing there yet" for
// an error and returns at once, and HTTPClient asks again straight away with
// only a delay(0) - which lets nothing lower in - so a poll spun this core
// from the first byte of a big reply to the last. With the 3D view drawing on
// the same core, a 130KB reply kept the core's idle task out past the task
// watchdog's five seconds, and the board was reset.
//
// And the whole of a reply given a time: HTTPClient asks for more for as long
// as the connection stays open, so a server that stopped sending part way
// and kept it open held the poll task for good - every poll after it never
// made, the screens showing the sky as it was. Past the time, the connection
// is closed, the reply fails, and the next poll goes ahead.
class PoliteTls : public WiFiClientSecure {
  public:
    // Until when, on millis(), the reply under way may go on being read; 0
    // for no limit.
    void allowUntil(uint32_t ms) { untilMs_ = ms; }

    size_t readBytes(char *buffer, size_t length) override {
        size_t got = 0;
        uint32_t since = millis();
        while (got < length) {
            if (untilMs_ != 0 && (int32_t)(millis() - untilMs_) > 0) {
                Serial.println("[adsb] a reply still not in after 30s - dropped");
                untilMs_ = 0;
                stop();
                break;
            }
            int r = read((uint8_t *)buffer + got, length - got);
            if (r > 0) {
                got += r;
                since = millis();
                continue;
            }
            // What has come, at once - its caller asks again for the rest -
            // or, with nothing yet, a millisecond's sleep before looking
            // again, until the connection goes or the time allowed is up.
            if (got > 0 || !connected() || millis() - since >= getTimeout()) {
                break;
            }
            vTaskDelay(1);
        }
        return got;
    }
    size_t readBytes(uint8_t *buffer, size_t length) override { return readBytes((char *)buffer, length); }

  private:
    uint32_t untilMs_ = 0;
};
static PoliteTls g_tls;
static HTTPClient g_http;

// The reply read a block at a time for the parser, which asks for it a
// character at a time: straight off the TLS client, each character was a
// decrypting read of its own, and a 300KB reply - a follow poll 120nm round
// a jet at FL240 - took core 0 a second and more.
class BlockReader {
  public:
    explicit BlockReader(Stream &stream) : stream_(stream) {}
    int read() {
        if (at_ == got_) {
            got_ = stream_.readBytes(block_, sizeof block_);
            at_ = 0;
            if (got_ == 0) {
                return -1;
            }
        }
        return (uint8_t)block_[at_++];
    }
    size_t readBytes(char *out, size_t length) {
        size_t done = 0;
        for (int c; done < length && (c = read()) >= 0;) {
            out[done++] = (char)c;
        }
        return done;
    }

  private:
    Stream &stream_;
    char block_[1024];
    size_t got_ = 0, at_ = 0;
};
static bool g_tlsConfigured = false;
// Which host that kept-alive socket is connected to. Reusing it for a request
// to a different provider would send the request down a connection to the
// wrong server, so a change of host closes it first.
static String g_connectedHost;

// Providers answer 429 when they have had enough, and going straight back at
// the same cadence only collects more of them. After one, the polls skip that
// provider for a minute without a request going out at all - the status line
// already says the data is ageing, which is the honest thing to show
// meanwhile - and after each more in a row twice as long, a quarter of an hour
// at most, or for as long as its Retry-After asks, until it answers again:
// many devices polling a volunteers' server at a minute's spacing had it
// refusing them all day. Per provider, so one throttling us doesn't sideline
// the other.
constexpr uint32_t RATE_LIMIT_BACKOFF_MS = 60000, RATE_LIMIT_MOST_MS = 15 * 60000;
static uint32_t g_rateLimitedUntil[ADSB_PROVIDER_COUNT] = {};
static uint8_t g_rateLimitedRun[ADSB_PROVIDER_COUNT] = {};
static uint32_t g_retryAfterS = 0;  // the last 429's Retry-After, seconds; 0 none

// HTTPClient always sends its own internal _userAgent field regardless of
// addHeader("User-Agent", ...) (see HTTPClient.cpp ~line 1226), so that call
// was silently overridden by the default "ESP32HTTPClient" - which is exactly
// the kind of generic string adsb.lol rejects. setUserAgent() is the actual
// override point.
static const char *USER_AGENT =
    "OverheadFlightTracker/" FIRMWARE_VERSION " (+https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-cpp)";

#ifdef NET_PROFILE
// Set inside requestInto so the request and the parse stay separable now that
// both happen in there.
static uint32_t g_tAfterGet = 0, g_tAfterParse = 0;
static int g_bytes = 0;  // the response's size, as it declared it or as read
#endif

// One request on the shared client. Returns the HTTP status, or a negative
// HTTPClient error for a transport-level failure.
static int requestInto(const String &url, const AdsbEndpoint &ep, JsonDocument &doc, DeserializationError &jerr) {
    String host = hostOf(url);
    if (g_connectedHost.length() && host != g_connectedHost) {
        // Pointed somewhere else now; the kept connection is no use for it.
        g_http.end();
        g_tls.stop();
    }
    g_connectedHost = host;

    if (!g_http.begin(g_tls, url)) {
        Serial.println("[adsb] http.begin() failed");
        return HTTPC_ERROR_CONNECTION_REFUSED;
    }
    g_http.setUserAgent(USER_AGENT);
    g_http.setReuse(true);
    // Longer than the 5s default: a served request takes ~45ms, but these
    // endpoints delay rather than refuse when throttling, and a reply that
    // takes several seconds is still worth having. Only the poll task waits.
    g_http.setTimeout(12000);
    g_tls.allowUntil(millis() + 30000);
    static const char *KEEP[] = {"Retry-After"};
    g_http.collectHeaders(KEEP, 1);

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
        g_retryAfterS = code == 429 ? (uint32_t)std::max(0L, g_http.header("Retry-After").toInt()) : 0;
        g_http.end();
        return code;
    }

    JsonDocument filter;  // a handful of keys - not worth the slower memory
    buildFilter(filter, ep.arrayKey);
    // Parsed straight off the socket where the response declared a length: a
    // busy radius returns a couple of hundred KB, and holding that as a String
    // on top of the document it is about to become was the peak of this task's
    // heap use for no benefit.
    //
    // Only where it declared a length, though - HTTPClient de-chunks on the
    // getString()/writeToStream() paths but not on the raw stream, so a
    // chunked reply read straight would have the chunk headers still in it.
#ifdef NET_PROFILE
    g_bytes = g_http.getSize();
#endif
    if (g_http.getSize() >= 0) {
        BlockReader reader(g_http.getStream());
        jerr = deserializeJson(doc, reader, DeserializationOption::Filter(filter));
    } else {
        String payload = g_http.getString();
#ifdef NET_PROFILE
        g_bytes = payload.length();
#endif
        jerr = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
    }
#ifdef NET_PROFILE
    g_tAfterParse = millis();
#endif
    // Leaves the socket open for the next poll; HTTPClient drains whatever the
    // filter stopped short of reading.
    g_http.end();
    g_tls.allowUntil(0);
    return code;
}

// Turns a parsed document into aircraft. `military` is which endpoint was
// asked, which is the answer for a feed that doesn't carry dbFlags.
//
// A global endpoint hasn't applied the radius, so this does it - rather than
// leaving it to the caller's own distance filter, which would mean building a
// couple of hundred string-heavy Aircraft to throw all but a few of them away.
// It also means an aircraft with no position has to go: on a radius-limited
// endpoint the server has already vouched for it being near, but on a global
// feed there is nothing to place it by, and adsb.fi's military feed reports
// around fifty of those at any moment - every one of which would otherwise
// land in a table captioned as showing traffic within a few dozen miles.
static void parseInto(JsonDocument &doc, const AdsbEndpoint &ep, bool military, double lat, double lon,
                      int radiusNm, std::vector<Aircraft> &result) {
    for (JsonObjectConst ac : doc[ep.arrayKey].as<JsonArrayConst>()) {
        const char *hex = ac["hex"] | "";
        if (!hex[0]) {
            continue;
        }
        // A ground station - the database's TWR: a fixed transmitter at an
        // airport with a transponder address of its own - is no aircraft,
        // and is left out here, for good: not in the table, on the radar or
        // in the 3D view, nor in a recording.
        const char *type = ac["t"] | "";
        if (strcmp(type, "TWR") == 0) {
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
        // readsb passes on '@' for a callsign character the transponder sent
        // as blank, so a callsign that is nothing but those was never set.
        // Without one it goes by its registration - below - or failing that
        // its hex: "UNKNOWN", as it used to be for the first, named nothing.
        cs.replace("@", "");
        cs.trim();
        a.callsign = cs;
        a.type = (const char *)(ac["t"] | "----");
        // Where the feed carries readsb's dbFlags, bit 0 is the military one
        // (bits 1/2/3 are interesting/PIA/LADD). Where it doesn't, which feed
        // this came from is the only thing there is to go on - and it is a
        // sound answer, since a military endpoint returns nothing else.
        int dbFlags = ac["dbFlags"] | 0;
        a.military = ep.hasDbFlags ? ((dbFlags & 1) != 0) : military;
        a.dbInteresting = ep.hasDbFlags && (dbFlags & 2) != 0;

        if (onGround) {
            a.altStr = "GND";
        } else if (altV.isNull()) {
            a.altStr = "?";
        } else {
            a.altStr = String((long)altNum);
        }

        // Each lookup is a walk of the object's keys, so the ones read more
        // than once are looked up once.
        JsonVariantConst gsV = ac["gs"];
        JsonVariantConst latV = ac["lat"], lonV = ac["lon"];
        a.speedStr = gsV.isNull() ? "?" : String((long)lround(gsV.as<float>()));

        // A position gone stale - most often at a touchdown, as the aircraft
        // drops below the receivers' horizon and switches to the ground's
        // messages - leaves the feed with its last one instead. Recent, that
        // still says where it is better than nothing does: the plot keeps it,
        // dimmed, rather than having it vanish until a receiver picks it up
        // taxiing.
        a.posStale = false;
        if (latV.isNull() || lonV.isNull()) {
            JsonObjectConst last = ac["lastPosition"];
            if (!last.isNull() && !last["lat"].isNull() && !last["lon"].isNull() &&
                (last["seen_pos"] | 1e9f) <= STALE_POSITION_MAX_S) {
                latV = last["lat"];
                lonV = last["lon"];
                a.posStale = true;
            }
        }
        if (!latV.isNull() && !lonV.isNull()) {
            double acLat = latV.as<double>(), acLon = lonV.as<double>();
            a.hasDist = true;
            a.distNm = haversineNm(lat, lon, acLat, acLon);
            a.hasPos = true;
            a.lat = (float)acLat;
            a.lon = (float)acLon;
            if (ep.global && a.distNm > radiusNm) {
                continue;
            }
        } else {
            if (ep.global) {
                continue;  // nothing to place it by, and the feed is the world
            }
            a.hasDist = false;
            a.distNm = 0;
            a.hasPos = false;
        }

        a.reg = (const char *)(ac["r"] | "");
        if (a.callsign.length() == 0) {
            a.callsign = a.reg.length() ? a.reg : a.hex;
        }
        a.squawk = (const char *)(ac["squawk"] | "");
        a.category = (const char *)(ac["category"] | "");

        JsonVariantConst trackV = ac["track"];
        if (!trackV.isNull()) {
            a.hasTrack = true;
            a.track = trackV.as<float>();
        } else {
            a.hasTrack = false;
        }

        JsonVariantConst altGeomV = ac["alt_geom"];
        if (!altGeomV.isNull()) {
            a.hasAltGeom = true;
            a.altGeom = altGeomV.as<int>();
        } else {
            a.hasAltGeom = false;
        }

        JsonVariantConst baroV = ac["baro_rate"], geomV = ac["geom_rate"];
        a.hasVertRate = !baroV.isNull() || !geomV.isNull();
        a.vertRate = baroV.isNull() ? geomV | 0.0f : baroV.as<float>();

        if (onGround) {
            float gs = gsV | -1.0f;
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

        result.push_back(std::move(a));
    }

    std::sort(result.begin(), result.end(), [](const Aircraft &a, const Aircraft &b) {
        float da = a.hasDist ? a.distNm : 1e9f;
        float db = b.hasDist ? b.distNm : 1e9f;
        return da < db;
    });
}

// One provider, asked once (plus the single retry a died socket earns).
static bool fetchFrom(int idx, double lat, double lon, int radiusNm, bool military, std::vector<Aircraft> &out) {
    const AdsbProvider &provider = ADSB_PROVIDERS[idx];
    const AdsbEndpoint &ep = military ? provider.military : provider.civil;
    String url = buildUrl(ep, lat, lon, radiusNm);
#ifdef NET_PROFILE
    uint32_t tStart = millis();
#endif

    if (g_rateLimitedUntil[idx] != 0) {
        if ((int32_t)(millis() - g_rateLimitedUntil[idx]) < 0) {
            return false;  // still inside the backoff - don't even ask
        }
        g_rateLimitedUntil[idx] = 0;
    }

    if (!g_tlsConfigured) {
        g_tls.setInsecure();  // no CA bundle managed for this hobby project
        g_tlsConfigured = true;
    }

    JsonDocument doc(&g_psram);
    DeserializationError jerr;
    int code = requestInto(url, ep, doc, jerr);
    if (code < 0) {
        // However it failed, a kept connection is no longer trustworthy.
        g_http.end();
        g_tls.stop();
        g_connectedHost = "";
        // Only a socket that died *before* the request landed is worth sending
        // again. A read timeout means the server has the request and is merely
        // slow, so retrying it would put two of the same request on an endpoint
        // that - on this evidence - is already asking us to ease off.
        bool socketDied = (code == HTTPC_ERROR_CONNECTION_LOST || code == HTTPC_ERROR_SEND_HEADER_FAILED ||
                           code == HTTPC_ERROR_SEND_PAYLOAD_FAILED || code == HTTPC_ERROR_NOT_CONNECTED);
        if (!socketDied) {
            Serial.printf("[adsb] %s: transport error %d\n", provider.name, code);
            return false;
        }
        Serial.printf("[adsb] %s: connection %d on a reused socket, retrying once\n", provider.name, code);
        code = requestInto(url, ep, doc, jerr);
    }
    if (code == 429) {
        uint32_t wait = RATE_LIMIT_BACKOFF_MS << std::min<int>(g_rateLimitedRun[idx], 4);
        wait = std::max(std::min(wait, RATE_LIMIT_MOST_MS), std::min(g_retryAfterS, (uint32_t)3600) * 1000);
        g_rateLimitedRun[idx] = (uint8_t)std::min(g_rateLimitedRun[idx] + 1, 8);
        g_rateLimitedUntil[idx] = millis() + wait;
        Serial.printf("[adsb] %s: rate limited, backing off %us\n", provider.name, (unsigned)(wait / 1000));
        return false;
    }
    if (code != 200) {
        return false;
    }
    g_rateLimitedRun[idx] = 0;
    if (jerr) {
        Serial.printf("[adsb] %s: json parse error: %s\n", provider.name, jerr.c_str());
        return false;
    }

    std::vector<Aircraft> result;
    parseInto(doc, ep, military, lat, lon, radiusNm, result);

#ifdef NET_PROFILE
    Serial.printf("[net] %s %dnm %dKB  request %lums  read+parse %lums  build %lums  total %lums  %u aircraft\n",
                  provider.name, radiusNm, g_bytes / 1024, (unsigned long)(g_tAfterGet - tStart),
                  (unsigned long)(g_tAfterParse - g_tAfterGet), (unsigned long)(millis() - g_tAfterParse),
                  (unsigned long)(millis() - tStart), (unsigned)result.size());
#endif
    out = std::move(result);
    return true;
}

// Which provider AUTO reaches for first. Sticky rather than always starting at
// the top of the table: one that has gone down would otherwise cost a 12s
// timeout on every poll for as long as the outage lasted.
static int g_preferred = 0;
// ...but not for good. The first provider is the one whose feed marks military
// and "interesting" airframes, so a fallback that stuck after a single 429 or
// blip quietly switched those alerts off for as long as the device stayed up.
// Off it, it is asked first again every few minutes: a provider that is still
// down costs one failed request each time, rather than the alerts.
constexpr uint32_t PRIMARY_RETRY_MS = 5 * 60 * 1000;
static uint32_t g_preferredSinceMs = 0;

// Records which provider answered. `retriedPrimary` restarts the wait even
// when the answer came from the same fallback as before, so a primary that
// is still failing is tried once per interval rather than on every poll.
static void prefer(int idx, bool retriedPrimary) {
    if (idx != g_preferred || retriedPrimary) {
        g_preferredSinceMs = millis();
    }
    g_preferred = idx;
}
// Whether an empty result has already been checked against another provider and
// stood up. An empty sky and a drained feed are the same 200 response, so an
// empty result wants a second opinion - but only until one has been given: once
// two providers agree the sky is empty, a genuinely quiet sky is back to one
// request per poll. Any non-empty result clears it, so the next empty stretch
// is checked afresh.
//
// It starts false rather than tracking "the last poll had aircraft", which is
// the same thing everywhere except the one place it matters: a device booting
// into an outage has never seen aircraft, so a first empty result would have
// been believed, and - never being followed by a non-empty one - so would every
// empty result after it. That is the failure this whole arrangement exists to
// catch, and it would have sailed straight through it.
static bool g_emptyCorroborated = false;

// The last provider whose name was logged. A poll that succeeds is otherwise
// silent, which left the log unable to answer the one question worth asking
// during an outage - who is actually serving this? - since a quiet log could
// equally mean the primary recovered or the fallback taking over without
// comment. Logged on change only, so steady running stays quiet.
static const char *g_loggedProvider = nullptr;

static void noteProvider(const char *name, size_t count) {
    if (g_loggedProvider == name) {
        return;
    }
    g_loggedProvider = name;
    Serial.printf("[adsb] serving from %s (%u aircraft)\n", name, (unsigned)count);
}

bool fetchAircraft(double lat, double lon, int radiusNm, bool military, AdsbSource source,
                   std::vector<Aircraft> &out, const char *&usedProvider) {
    int pinned = adsbSourceIndex(source);
    if (pinned >= 0) {
        if (pinned >= ADSB_PROVIDER_COUNT) {
            pinned = 0;  // a value saved by a build that knew more providers
        }
        std::vector<Aircraft> got;
        if (!fetchFrom(pinned, lat, lon, radiusNm, military, got)) {
            return false;
        }
        // Pinned means pinned: an empty answer from the chosen provider is
        // reported as it stands, with no second opinion sought. Nothing is
        // marked corroborated here, so switching back to Auto checks again.
        if (!got.empty()) {
            g_emptyCorroborated = false;
        }
        usedProvider = ADSB_PROVIDERS[pinned].name;
        noteProvider(usedProvider, got.size());
        out = std::move(got);
        return true;
    }

    int first = g_preferred;
    bool retryPrimary = (first != 0 && millis() - g_preferredSinceMs >= PRIMARY_RETRY_MS);
    if (retryPrimary) {
        first = 0;
    }
    for (int attempt = 0; attempt < ADSB_PROVIDER_COUNT; attempt++) {
        int idx = (first + attempt) % ADSB_PROVIDER_COUNT;
        std::vector<Aircraft> got;
        if (!fetchFrom(idx, lat, lon, radiusNm, military, got)) {
            continue;  // failed outright - next provider
        }

        // The case this whole arrangement exists for: a 200 with nothing in it,
        // where the poll before found traffic. Ask someone else before putting
        // "No aircraft in range" on screen. Skipped once we are already on a
        // fallback, since the provider ahead of it in the ring just failed.
        if (got.empty() && !g_emptyCorroborated && attempt + 1 < ADSB_PROVIDER_COUNT) {
            int alt = (idx + 1) % ADSB_PROVIDER_COUNT;
            std::vector<Aircraft> second;
            if (fetchFrom(alt, lat, lon, radiusNm, military, second) && !second.empty()) {
                Serial.printf("[adsb] %s reported an empty sky, %s found %u - switching to %s\n",
                              ADSB_PROVIDERS[idx].name, ADSB_PROVIDERS[alt].name, (unsigned)second.size(),
                              ADSB_PROVIDERS[alt].name);
                prefer(alt, retryPrimary);
                g_emptyCorroborated = false;
                usedProvider = ADSB_PROVIDERS[alt].name;
                noteProvider(usedProvider, second.size());
                out = std::move(second);
                return true;
            }
            // Both agree there is nothing there, so there is nothing there -
            // and it need not be asked again until something shows up.
            g_emptyCorroborated = true;
        }

        prefer(idx, retryPrimary);
        if (!got.empty()) {
            g_emptyCorroborated = false;
        }
        usedProvider = ADSB_PROVIDERS[idx].name;
        noteProvider(usedProvider, got.size());
        out = std::move(got);
        return true;
    }
    return false;
}
