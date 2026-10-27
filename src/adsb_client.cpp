#include "adsb_client.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <math.h>

#include "aircraft_db.h"
#include "config.h"

bool stillInTheAir(float gsKt, const String &type, const String &category) {
    return gsKt < AIRBORNE_STILL_KT && !canHover(type, category);
}

float haversineNm(double lat1, double lon1, double lat2, double lon2) {
    // In single precision, as the P4's FPU does it: in double, done in
    // software, the sines, cosines and arcsine were most of what making each
    // aircraft of a poll took. Each difference is taken in double first, so
    // two points close together keep their digits; a float then carries the
    // distance to a few centimetres in a hundred miles.
    constexpr float R_KM = 6371.0f, DEG = (float)M_PI / 180.0f;
    float p1 = (float)lat1 * DEG, p2 = (float)lat2 * DEG;
    float dphi = (float)(lat2 - lat1) * DEG, dlambda = (float)(lon2 - lon1) * DEG;
    float s1 = sinf(dphi / 2), s2 = sinf(dlambda / 2);
    float a = s1 * s1 + cosf(p1) * cosf(p2) * s2 * s2;
    float km = 2 * R_KM * asinf(sqrtf(std::min(a, 1.0f)));
    return km * 0.539957f;
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

#ifdef BENCH3D
// The fields of one aircraft, as ArduinoJson's filter has them - for the
// bench's parse the way it was (see adsbBenchParse()), skipping the rest of
// some forty in the tokeniser: the reply as one document, and one aircraft a
// document.
static void fillFilter(JsonObject ac) {
    for (const char *key : {"hex", "flight", "t", "dbFlags", "alt_baro", "gs", "lat", "lon", "r", "squawk",
                            "category", "track", "alt_geom", "baro_rate", "geom_rate", "seen_pos"}) {
        ac[key] = true;
    }
    JsonObject last = ac["lastPosition"].to<JsonObject>();
    last["lat"] = true;
    last["lon"] = true;
    last["seen_pos"] = true;
}

// A filter for an array is a one-element array, that element applied to every
// entry in turn, hung off the key the provider uses - `ac` on adsb.lol,
// `aircraft` on an adsb.fi point query.
static void buildFilter(JsonDocument &filter, const char *arrayKey) { fillFilter(filter[arrayKey].add<JsonObject>()); }
#endif

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
    void allowUntil(uint32_t ms) {
        untilMs_ = ms;
        if (ms != 0) {
            dropped_ = false;
        }
    }
    // Whether the reply was cut off by that time. It shows as a reply that
    // will not parse, but it is the network that stopped delivering - the
    // first sign, twice now, of the WiFi link to the co-processor sticking -
    // so the watchdog's count needs it told apart from a server's garbage.
    bool dropped() const { return dropped_; }

    size_t readBytes(char *buffer, size_t length) override {
        size_t got = 0;
        uint32_t since = millis();
        while (got < length) {
            if (untilMs_ != 0 && (int32_t)(millis() - untilMs_) > 0) {
                Serial.println("[adsb] a reply still not in after 30s - dropped");
                untilMs_ = 0;
                dropped_ = true;
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
    bool dropped_ = false;
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
// Set by requestInto() when the request was never made - a URL the client
// would not take - which fails with the same code as a refused connection
// but says nothing about the network.
static bool g_requestNotMade = false;

static AdsbFeedHealth g_health;

const AdsbFeedHealth &adsbFeedHealth() { return g_health; }

void adsbDropConnection() {
    g_http.end();
    g_tls.stop();
    g_connectedHost = "";
}

// What came of one request, for the watchdog's count: see AdsbFeedHealth.
static void noteAnswered() {
    g_health.answers++;
    g_health.lastAnswerMs = millis();
    g_health.netFails = 0;
}
static void noteNetFail(const char *what) {
    uint32_t now = millis();
    if (g_health.netFails == 0) {
        g_health.netFailsSinceMs = now;
    }
    g_health.netFails++;
    g_health.netFailsEver++;
    g_health.lastNetFailMs = now;
    g_health.lastNetFail = what;
}

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

// One aircraft as a reply has it: the fields this app reads, each as the
// reply gave it - a number, a string, something else, or not at all - so
// buildAircraft() reads them as it read ArduinoJson's variants: a string's
// default where it isn't one, a number's where it isn't one.
struct FeedNumber {
    enum Kind : uint8_t { NONE, NUMBER, STRING, OTHER } kind = NONE;
    bool integer = false;  // written without a point or an exponent
    float value = 0;
    float asFloat() const { return kind == NUMBER ? value : 0.0f; }
    float asFloatOr(float d) const { return kind == NUMBER ? value : d; }
    // A float's whole part, as ArduinoJson's as<int>() takes it - 0 outside an int's range.
    int asInt() const { return kind == NUMBER && fabsf(value) < 2.1e9f ? (int)value : 0; }
    // As ArduinoJson's `| 0`: only a whole number written as one.
    int asIntOr(int d) const { return kind == NUMBER && integer && fabsf(value) < 2.1e9f ? (int)value : d; }
};
template <size_t N>
struct FeedString {
    bool isString = false;
    char chars[N] = "";
    const char *text() const { return isString ? chars : ""; }
};
struct FeedAircraft {
    FeedString<16> hex, flight, type, squawk, category;
    FeedString<24> reg;
    FeedNumber altBaro, gs, lat, lon, dbFlags, track, altGeom, baroRate, geomRate;
    FeedNumber seenPos;  // how old lat and lon are, in seconds
    bool hasLast = false;  // lastPosition, an object
    FeedNumber lastLat, lastLon, lastSeen;
};

// A report left out as standing still in the air, logged the first time it
// is - a parked one is left out at every poll - so the log says what the rule
// is doing without a line a poll. The poll task's - and the bench's parse
// (bench2d.h) from another task, so nothing here is allocated: the worst a
// clash could do is log one twice.
static void noteStill(const char *hex, const char *flight, const char *type, float altFt, float gsKt) {
    static char logged[16][12];
    static int next = 0;
    for (const char *h : logged) {
        if (strcmp(h, hex) == 0) {
            return;
        }
    }
    strlcpy(logged[next], hex, sizeof logged[next]);
    next = (next + 1) % 16;
    String callsign(flight);
    callsign.trim();
    Serial.printf("[adsb] %s %s %s at %.0fft and %.1fkt: standing still in the air, left out\n", hex,
                  callsign.length() ? callsign.c_str() : "-", type[0] ? type : "-", altFt, gsKt);
}

// One aircraft of a reply - false for one that is left out. `military` is which endpoint was
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
static bool buildAircraft(const FeedAircraft &f, const AdsbEndpoint &ep, bool military, double lat, double lon,
                          int radiusNm, Aircraft &out) {
    const char *hex = f.hex.text();
    if (!hex[0]) {
        return false;
    }
    // A ground station - the database's TWR: a fixed transmitter at an
    // airport with a transponder address of its own - is no aircraft,
    // and is left out here, for good: not in the table, on the radar or
    // in the 3D view, nor in a recording.
    const char *type = f.type.text();
    if (strcmp(type, "TWR") == 0) {
        return false;
    }

    bool onGround = f.altBaro.kind == FeedNumber::STRING;
    bool altKnown = f.altBaro.kind != FeedNumber::NONE;
    float altNum = 0;
    if (!onGround && altKnown) {
        altNum = f.altBaro.asFloat();
        // Below sea level is real on a day of high pressure: a pressure
        // altitude reads a few hundred feet low, and one on short final
        // to a field by the sea gives less than nought - left out, it
        // went missing from the plot for its last few hundred feet. Far
        // below - lower than any field on the deepest high - is bogus.
        if (altNum < LOWEST_PRESSURE_ALT_FT) {
            return false;
        }
    }
    // A height in the air and no groundspeed to speak of: see
    // stillInTheAir(). One with no speed at all is kept - most often a
    // position worked out from when its signals reached the receivers,
    // its speed yet to be.
    bool gsKnown = f.gs.kind != FeedNumber::NONE;
    if (!onGround && altKnown && gsKnown && stillInTheAir(f.gs.asFloat(), type, f.category.text())) {
        noteStill(hex, f.flight.text(), type, altNum, f.gs.asFloat());
        return false;
    }

    Aircraft &a = out;
    a.hex = hex;
    String cs = String(f.flight.text());
    // readsb passes on '@' for a callsign character the transponder sent
    // as blank, so a callsign that is nothing but those was never set.
    // Without one it goes by its registration - below - or failing that
    // its hex: "UNKNOWN", as it used to be for the first, named nothing.
    cs.replace("@", "");
    cs.trim();
    a.callsign = cs;
    a.type = f.type.isString ? f.type.text() : "----";
    // Where the feed carries readsb's dbFlags, bit 0 is the military one
    // (bits 1/2/3 are interesting/PIA/LADD). Where it doesn't, which feed
    // this came from is the only thing there is to go on - and it is a
    // sound answer, since a military endpoint returns nothing else.
    int dbFlags = f.dbFlags.asIntOr(0);
    a.military = ep.hasDbFlags ? ((dbFlags & 1) != 0) : military;
    a.dbInteresting = ep.hasDbFlags && (dbFlags & 2) != 0;

    if (onGround) {
        a.altStr = "GND";
    } else if (!altKnown) {
        a.altStr = "?";
    } else {
        a.altStr = String((long)altNum);
    }
    a.speedStr = gsKnown ? String((long)lround(f.gs.asFloat())) : "?";

    // A position gone stale - most often at a touchdown, as the aircraft
    // drops below the receivers' horizon and switches to the ground's
    // messages - leaves the feed with its last one instead. Recent, that
    // still says where it is better than nothing does: the plot keeps it,
    // dimmed, rather than having it vanish until a receiver picks it up
    // taxiing.
    a.posStale = false;
    a.posAgeS = 0;
    const FeedNumber *latV = &f.lat, *lonV = &f.lon;
    if (latV->kind == FeedNumber::NONE || lonV->kind == FeedNumber::NONE) {
        if (f.hasLast && f.lastLat.kind != FeedNumber::NONE && f.lastLon.kind != FeedNumber::NONE &&
            f.lastSeen.asFloatOr(1e9f) <= STALE_POSITION_MAX_S) {
            latV = &f.lastLat;
            lonV = &f.lastLon;
            a.posStale = true;
        }
    }
    if (latV->kind != FeedNumber::NONE && lonV->kind != FeedNumber::NONE) {
        double acLat = latV->asFloat(), acLon = lonV->asFloat();
        a.hasDist = true;
        a.distNm = haversineNm(lat, lon, acLat, acLon);
        a.hasPos = true;
        a.lat = (float)acLat;
        a.lon = (float)acLon;
        a.posAgeS = std::min(std::max((a.posStale ? f.lastSeen : f.seenPos).asFloatOr(0.0f), 0.0f), 120.0f);
        if (ep.global && a.distNm > radiusNm) {
            return false;
        }
    } else {
        if (ep.global) {
            return false;  // nothing to place it by, and the feed is the world
        }
        a.hasDist = false;
        a.distNm = 0;
        a.hasPos = false;
    }

    a.reg = f.reg.text();
    if (a.callsign.length() == 0) {
        a.callsign = a.reg.length() ? a.reg : a.hex;
    }
    a.squawk = f.squawk.text();
    a.category = f.category.text();

    a.hasTrack = f.track.kind != FeedNumber::NONE;
    a.track = a.hasTrack ? f.track.asFloat() : 0;
    a.hasAltGeom = f.altGeom.kind != FeedNumber::NONE;
    a.altGeom = a.hasAltGeom ? f.altGeom.asInt() : 0;

    bool baroKnown = f.baroRate.kind != FeedNumber::NONE;
    a.hasVertRate = baroKnown || f.geomRate.kind != FeedNumber::NONE;
    a.vertRate = baroKnown ? f.baroRate.asFloat() : f.geomRate.asFloatOr(0.0f);

    if (onGround) {
        float gs = f.gs.asFloatOr(-1.0f);
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

    return true;
}

// Nearest first, as everything after the poll takes them.
static void sortByDistance(std::vector<Aircraft> &result) {
    std::sort(result.begin(), result.end(), [](const Aircraft &a, const Aircraft &b) {
        float da = a.hasDist ? a.distNm : 1e9f;
        float db = b.hasDist ? b.distNm : 1e9f;
        return da < db;
    });
}

// A reader with one character it can be given back - for finding what comes
// next between the reply's aircraft without taking it from ArduinoJson.
template <typename Reader>
class PushbackReader {
  public:
    explicit PushbackReader(Reader &reader) : reader_(reader) {}
    int read() {
        if (back_ >= 0) {
            int c = back_;
            back_ = -1;
            return c;
        }
        return reader_.read();
    }
    size_t readBytes(char *out, size_t length) {
        size_t done = 0;
        if (back_ >= 0 && length > 0) {
            out[done++] = (char)back_;
            back_ = -1;
        }
        return done + reader_.readBytes(out + done, length - done);
    }
    void giveBack(int c) { back_ = c; }

  private:
    Reader &reader_;
    int back_ = -1;
};

// A reply held whole in memory, read as the socket is.
class TextReader {
  public:
    TextReader(const char *text, size_t length) : at_(text), end_(text + length) {}
    int read() { return at_ < end_ ? (uint8_t)*at_++ : -1; }
    size_t readBytes(char *out, size_t length) {
        size_t n = std::min(length, (size_t)(end_ - at_));
        memcpy(out, at_, n);
        at_ += n;
        return n;
    }

  private:
    const char *at_, *end_;
};

// The next character that isn't white space, or -1 at the end.
template <typename Reader>
static int nextSolid(Reader &reader) {
    for (;;) {
        int c = reader.read();
        if (c != ' ' && c != '\n' && c != '\r' && c != '\t') {
            return c;
        }
    }
}

// On to just inside the array of aircraft: past `"key"`, its colon and the
// opening bracket, at the reply's top level. False if the reply ends first -
// a reply without one, which is an empty sky, as a document without it was.
// What turns out not to be the array's opening is given back, so a key of
// that name with something else for a value is scanned on past as anything is.
template <typename Reader>
static bool toArray(PushbackReader<Reader> &reader, const char *key) {
    const size_t keyLength = strlen(key);
    int depth = 0;
    bool inString = false, escaped = false, same = false;
    size_t at = 0;  // characters of the string so far
    for (int c; (c = reader.read()) >= 0;) {
        if (inString) {
            if (escaped) {
                escaped = false;
                same = false;  // the key has no escapes in it
                at++;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                inString = false;
                if (depth == 1 && same && at == keyLength) {
                    int colon = nextSolid(reader);
                    if (colon != ':') {
                        reader.giveBack(colon);
                        continue;
                    }
                    int open = nextSolid(reader);
                    if (open == '[') {
                        return true;
                    }
                    reader.giveBack(open);
                }
            } else {
                same = same && at < keyLength && c == (uint8_t)key[at];
                at++;
            }
            continue;
        }
        if (c == '"') {
            inString = true;
            same = true;
            at = 0;
        } else if (c == '{' || c == '[') {
            depth++;
        } else if (c == '}' || c == ']') {
            depth--;
        }
    }
    return false;
}

// Reads a reply's aircraft, field by field, straight into FeedAircraft -
// see parseAircraftList(). JSON as RFC 8259 has it; a field this app doesn't
// read skipped, whatever it holds; a string longer than its field cut short
// - the fields' sizes are many times what the feeds send.
template <typename Reader>
class FeedScanner {
  public:
    explicit FeedScanner(PushbackReader<Reader> &reader) : r_(reader) {}

    // An aircraft's object, its '{' read, into `out`, to its closing '}'.
    bool aircraft(FeedAircraft &out) {
        int c = nextSolid(r_);
        if (c == '}') {
            return true;
        }
        for (;;) {
            if (c != '"') {
                return false;
            }
            char key[16];
            if (!string(key, sizeof key)) {
                return false;
            }
            if (nextSolid(r_) != ':') {
                return false;
            }
            if (!field(key, nextSolid(r_), out)) {
                return false;
            }
            c = nextSolid(r_);
            if (c == '}') {
                return true;
            }
            if (c != ',') {
                return false;
            }
            c = nextSolid(r_);
        }
    }

  private:
    // The value of field `key`, its first character `c` read.
    bool field(const char *key, int c, FeedAircraft &out) {
        switch (key[0]) {
        case 'h': if (strcmp(key, "hex") == 0) return text(c, out.hex); break;
        case 'f': if (strcmp(key, "flight") == 0) return text(c, out.flight); break;
        case 't':
            if (key[1] == 0) return text(c, out.type);
            if (strcmp(key, "track") == 0) return number(c, out.track);
            break;
        case 'd': if (strcmp(key, "dbFlags") == 0) return number(c, out.dbFlags); break;
        case 'a':
            if (strcmp(key, "alt_baro") == 0) return number(c, out.altBaro);
            if (strcmp(key, "alt_geom") == 0) return number(c, out.altGeom);
            break;
        case 'g':
            if (key[1] == 's' && key[2] == 0) return number(c, out.gs);
            if (strcmp(key, "geom_rate") == 0) return number(c, out.geomRate);
            break;
        case 'l':
            if (strcmp(key, "lat") == 0) return number(c, out.lat);
            if (strcmp(key, "lon") == 0) return number(c, out.lon);
            if (strcmp(key, "lastPosition") == 0) return lastPosition(c, out);
            break;
        case 'r': if (key[1] == 0) return text(c, out.reg); break;
        case 's':
            if (strcmp(key, "squawk") == 0) return text(c, out.squawk);
            if (strcmp(key, "seen_pos") == 0) return number(c, out.seenPos);
            break;
        case 'c': if (strcmp(key, "category") == 0) return text(c, out.category); break;
        case 'b': if (strcmp(key, "baro_rate") == 0) return number(c, out.baroRate); break;
        }
        return skip(c, 0);
    }

    // lastPosition: an object with its own lat, lon and seen_pos.
    bool lastPosition(int c, FeedAircraft &out) {
        if (c != '{') {
            out.hasLast = false;
            return skip(c, 0);
        }
        out.hasLast = true;
        c = nextSolid(r_);
        if (c == '}') {
            return true;
        }
        for (;;) {
            char key[16];
            if (c != '"' || !string(key, sizeof key) || nextSolid(r_) != ':') {
                return false;
            }
            c = nextSolid(r_);
            bool ok = strcmp(key, "lat") == 0        ? number(c, out.lastLat)
                      : strcmp(key, "lon") == 0      ? number(c, out.lastLon)
                      : strcmp(key, "seen_pos") == 0 ? number(c, out.lastSeen)
                                                     : skip(c, 1);
            if (!ok) {
                return false;
            }
            c = nextSolid(r_);
            if (c == '}') {
                return true;
            }
            if (c != ',') {
                return false;
            }
            c = nextSolid(r_);
        }
    }

    template <size_t N>
    bool text(int c, FeedString<N> &out) {
        out.isString = c == '"';
        out.chars[0] = 0;
        return out.isString ? string(out.chars, N) : skip(c, 0);
    }

    // A number field: a number, or whatever else is there, as such.
    bool number(int c, FeedNumber &out) {
        if (c == '-' || (c >= '0' && c <= '9')) {
            out.kind = FeedNumber::NUMBER;
            return parseNumber(c, out);
        }
        out.kind = c == 'n' ? FeedNumber::NONE : c == '"' ? FeedNumber::STRING : FeedNumber::OTHER;
        return skip(c, 0);
    }

    // A number, its first character read, rounded once to the float nearest:
    // in the FPU's single precision where its digits are a whole number a
    // float holds exactly, scaled by a power of ten one holds exactly - one
    // sum, one rounding - and otherwise, as a position's eight or nine digits
    // are, in double, done in software, its power of ten exact there too.
    bool parseNumber(int c, FeedNumber &out) {
        bool negative = c == '-';
        if (negative) {
            c = r_.read();
        }
        if (c < '0' || c > '9') {
            return false;
        }
        // The digits in 32 bits while they fit - nine of them, a position's
        // eight or so - and in 64 past that, up to nineteen.
        uint32_t small = 0;
        uint64_t digits = 0;
        int count = 0, exponent = 0;
        bool integer = true, wide = false;
        auto digit = [&](int d) {
            if (!wide && count < 9) {
                small = small * 10 + (uint32_t)d;
                count += small != 0;
                return true;
            }
            if (!wide) {
                wide = true;
                digits = small;
            }
            if (count < 19) {
                digits = digits * 10 + (uint32_t)d;
                count++;
                return true;
            }
            return false;
        };
        for (; c >= '0' && c <= '9'; c = r_.read()) {
            if (!digit(c - '0')) {
                exponent++;
            }
        }
        if (c == '.') {
            integer = false;
            for (c = r_.read(); c >= '0' && c <= '9'; c = r_.read()) {
                if (digit(c - '0')) {
                    exponent--;
                }
            }
        }
        if (!wide) {
            digits = small;
        }
        if (c == 'e' || c == 'E') {
            integer = false;
            c = r_.read();
            bool down = c == '-';
            if (c == '-' || c == '+') {
                c = r_.read();
            }
            int e = 0;
            for (; c >= '0' && c <= '9'; c = r_.read()) {
                e = std::min(e * 10 + (c - '0'), 10000);
            }
            exponent += down ? -e : e;
        }
        r_.giveBack(c);
        static const float P10F[] = {1e0f, 1e1f, 1e2f, 1e3f, 1e4f, 1e5f, 1e6f, 1e7f, 1e8f, 1e9f, 1e10f};
        static const double P10D[] = {1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,  1e10, 1e11,
                                      1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
        float value;
        if (digits <= (1u << 24) && exponent >= -10 && exponent <= 10) {
            float whole = (float)(uint32_t)digits;
            value = exponent >= 0 ? whole * P10F[exponent] : whole / P10F[-exponent];
        } else if (exponent >= -22 && exponent <= 22) {
            double whole = (double)digits;
            value = (float)(exponent >= 0 ? whole * P10D[exponent] : whole / P10D[-exponent]);
        } else {
            value = (float)((double)digits * pow(10.0, exponent));
        }
        out.integer = integer;
        out.value = negative ? -value : value;
        return true;
    }

    // A string, its opening quote read, into `out` - at most `size` - 1 of it
    // and the nought after, or nowhere with `size` 0 - to its closing quote.
    bool string(char *out, size_t size) {
        size_t n = 0;
        auto put = [&](int byte) {
            if (n + 1 < size) {
                out[n++] = (char)byte;
            }
        };
        for (;;) {
            int c = r_.read();
            if (c < 0) {
                if (size) out[n] = 0;
                return false;
            }
            if (c == '"') {
                break;
            }
            if (c != '\\') {
                put(c);
                continue;
            }
            c = r_.read();
            switch (c) {
            case '"': case '\\': case '/': put(c); break;
            case 'b': put('\b'); break;
            case 'f': put('\f'); break;
            case 'n': put('\n'); break;
            case 'r': put('\r'); break;
            case 't': put('\t'); break;
            case 'u': {
                uint32_t u = 0;
                if (!hex4(u)) {
                    if (size) out[n] = 0;
                    return false;
                }
                // A pair of surrogates is one character past the first plane.
                if (u >= 0xD800 && u < 0xDC00) {
                    uint32_t low = 0;
                    if (r_.read() != '\\' || r_.read() != 'u' || !hex4(low) || low < 0xDC00 || low > 0xDFFF) {
                        if (size) out[n] = 0;
                        return false;
                    }
                    u = 0x10000 + ((u - 0xD800) << 10) + (low - 0xDC00);
                }
                if (u < 0x80) {
                    put((int)u);
                } else if (u < 0x800) {
                    put(0xC0 | (u >> 6));
                    put(0x80 | (u & 63));
                } else if (u < 0x10000) {
                    put(0xE0 | (u >> 12));
                    put(0x80 | (u >> 6 & 63));
                    put(0x80 | (u & 63));
                } else {
                    put(0xF0 | (u >> 18));
                    put(0x80 | (u >> 12 & 63));
                    put(0x80 | (u >> 6 & 63));
                    put(0x80 | (u & 63));
                }
                break;
            }
            default:
                if (size) out[n] = 0;
                return false;
            }
        }
        if (size) {
            out[n] = 0;
        }
        return true;
    }

    bool hex4(uint32_t &u) {
        for (int i = 0; i < 4; i++) {
            int c = r_.read();
            int d = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
            if (d < 0) {
                return false;
            }
            u = u << 4 | (uint32_t)d;
        }
        return true;
    }

    // A string, its opening quote read, to its closing quote - nothing kept.
    bool skipString() {
        for (;;) {
            int c = r_.read();
            if (c == '"') {
                return true;
            }
            if (c == '\\') {
                if (r_.read() < 0) {
                    return false;
                }
            } else if (c < 0) {
                return false;
            }
        }
    }

    // A number, its first character read, to the character after it, which is
    // given back - nothing worked out.
    bool skipNumber(int c) {
        bool any = c >= '0' && c <= '9';
        for (c = r_.read(); (c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-';
             c = r_.read()) {
            any = true;
        }
        r_.giveBack(c);
        return any;
    }

    // Any value, its first character read - nested no deeper than ten.
    bool skip(int c, int depth) {
        if (c == '"') {
            return skipString();
        }
        if (c == '-' || (c >= '0' && c <= '9')) {
            return skipNumber(c);
        }
        if (c == 't') return word("rue");
        if (c == 'f') return word("alse");
        if (c == 'n') return word("ull");
        if ((c != '{' && c != '[') || depth >= 10) {
            return false;
        }
        int close = c == '{' ? '}' : ']';
        c = nextSolid(r_);
        if (c == close) {
            return true;
        }
        for (;;) {
            if (close == '}') {
                if (c != '"' || !skipString() || nextSolid(r_) != ':') {
                    return false;
                }
                c = nextSolid(r_);
            }
            if (!skip(c, depth + 1)) {
                return false;
            }
            c = nextSolid(r_);
            if (c == close) {
                return true;
            }
            if (c != ',') {
                return false;
            }
            c = nextSolid(r_);
        }
    }

    bool word(const char *rest) {
        for (; *rest; rest++) {
            if (r_.read() != *rest) {
                return false;
            }
        }
        return true;
    }

    PushbackReader<Reader> &r_;
};

// The reply's aircraft, one at a time, into `out`, nearest first.
//
// Not with ArduinoJson. As one document, the whole reply took time in the
// square of its length: ArduinoJson 7 keeps one copy of each string, and
// whether it has one already is found by a walk of every string it holds -
// 1.5s for 822 aircraft, on the core the 3D view draws on with. A document
// an aircraft fixed that, but making and filling one - its keys matched
// against a filter, its strings and slots allocated and given back - was
// still half a millisecond each, against a twentieth for reading the reply.
// This reads each aircraft's fields straight into a FeedAircraft, in place,
// and skips the rest as it goes - a few tens of microseconds an aircraft.
template <typename Reader>
static DeserializationError parseAircraftList(Reader &source, const AdsbEndpoint &ep, bool military, double lat,
                                              double lon, int radiusNm, std::vector<Aircraft> &out) {
    PushbackReader<Reader> reader(source);
    if (!toArray(reader, ep.arrayKey)) {
        return DeserializationError::Ok;  // none: an empty sky, as a reply without it always was
    }
    FeedScanner<Reader> scan(reader);
    out.reserve(64);
    for (;;) {
        int c = nextSolid(reader);
        if (c == ',') {
            c = nextSolid(reader);
        }
        if (c == ']') {
            break;
        }
        if (c != '{') {
            return c < 0 ? DeserializationError::IncompleteInput : DeserializationError::InvalidInput;
        }
        FeedAircraft f;
        if (!scan.aircraft(f)) {
            return DeserializationError::InvalidInput;
        }
        // Made where it is to stay: one left out goes again at once.
        out.emplace_back();
        if (!buildAircraft(f, ep, military, lat, lon, radiusNm, out.back())) {
            out.pop_back();
        }
    }
    sortByDistance(out);
    return DeserializationError::Ok;
}

#ifdef BENCH3D
// The reply as one document, the way it was parsed until the aircraft were
// taken one at a time - for the bench to check they come out the same.
static void feedNumber(JsonVariantConst v, FeedNumber &out) {
    out.kind = v.isNull()                ? FeedNumber::NONE
               : v.is<const char *>()    ? FeedNumber::STRING
               : v.is<float>()           ? FeedNumber::NUMBER
                                         : FeedNumber::OTHER;
    out.integer = v.is<int>();
    out.value = out.kind == FeedNumber::NUMBER ? v.as<float>() : 0;
}
template <size_t N>
static void feedString(JsonVariantConst v, FeedString<N> &out) {
    out.isString = v.is<const char *>();
    strlcpy(out.chars, out.isString ? v.as<const char *>() : "", N);
}
static void feedFromJson(JsonObjectConst ac, FeedAircraft &f) {
    feedString(ac["hex"], f.hex);
    feedString(ac["flight"], f.flight);
    feedString(ac["t"], f.type);
    feedString(ac["r"], f.reg);
    feedString(ac["squawk"], f.squawk);
    feedString(ac["category"], f.category);
    feedNumber(ac["alt_baro"], f.altBaro);
    feedNumber(ac["gs"], f.gs);
    feedNumber(ac["lat"], f.lat);
    feedNumber(ac["lon"], f.lon);
    feedNumber(ac["dbFlags"], f.dbFlags);
    feedNumber(ac["track"], f.track);
    feedNumber(ac["alt_geom"], f.altGeom);
    feedNumber(ac["baro_rate"], f.baroRate);
    feedNumber(ac["geom_rate"], f.geomRate);
    feedNumber(ac["seen_pos"], f.seenPos);
    JsonObjectConst last = ac["lastPosition"];
    f.hasLast = !last.isNull();
    if (f.hasLast) {
        feedNumber(last["lat"], f.lastLat);
        feedNumber(last["lon"], f.lastLon);
        feedNumber(last["seen_pos"], f.lastSeen);
    }
}
static void parseInto(JsonDocument &doc, const AdsbEndpoint &ep, bool military, double lat, double lon,
                      int radiusNm, std::vector<Aircraft> &result) {
    for (JsonObjectConst ac : doc[ep.arrayKey].as<JsonArrayConst>()) {
        FeedAircraft f;
        feedFromJson(ac, f);
        Aircraft a;
        if (buildAircraft(f, ep, military, lat, lon, radiusNm, a)) {
            result.push_back(std::move(a));
        }
    }
    sortByDistance(result);
}
#endif

// One request on the shared client. Returns the HTTP status, or a negative
// HTTPClient error for a transport-level failure.
static int requestInto(const String &url, const AdsbEndpoint &ep, bool military, double lat, double lon, int radiusNm,
                       std::vector<Aircraft> &out, DeserializationError &jerr) {
    String host = hostOf(url);
    if (g_connectedHost.length() && host != g_connectedHost) {
        // Pointed somewhere else now; the kept connection is no use for it.
        g_http.end();
        g_tls.stop();
    }
    g_connectedHost = host;

    g_requestNotMade = false;
    if (!g_http.begin(g_tls, url)) {
        Serial.println("[adsb] http.begin() failed");
        g_requestNotMade = true;
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

    // Parsed straight off the socket where the response declared a length: a
    // busy radius returns a couple of hundred KB, and holding that as a String
    // was the peak of this task's heap use for no benefit.
    //
    // Only where it declared a length, though - HTTPClient de-chunks on the
    // getString()/writeToStream() paths but not on the raw stream, so a
    // chunked reply read straight would have the chunk headers still in it.
#ifdef NET_PROFILE
    g_bytes = g_http.getSize();
#endif
    out.clear();
    if (g_http.getSize() >= 0) {
        BlockReader reader(g_http.getStream());
        jerr = parseAircraftList(reader, ep, military, lat, lon, radiusNm, out);
    } else {
        String payload = g_http.getString();
#ifdef NET_PROFILE
        g_bytes = payload.length();
#endif
        TextReader reader(payload.c_str(), payload.length());
        jerr = parseAircraftList(reader, ep, military, lat, lon, radiusNm, out);
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

// How one request ended, for AdsbFeedHealth: answered, failed at the network,
// or neither - never made, or out of memory and the like, which are this
// device's own trouble and no sign of the network's.
static void noteOutcome(int code) {
    g_health.requests++;
    if (code > 0) {
        if (g_tls.dropped()) {
            noteNetFail("a reply cut off after 30s");
        } else {
            noteAnswered();
        }
        return;
    }
    if (g_requestNotMade) {
        return;
    }
    switch (code) {
    case HTTPC_ERROR_CONNECTION_REFUSED:
        // The name not looked up, the server not reached, or the TLS
        // handshake not made: HTTPClient says the same for all three.
        noteNetFail("no connection made");
        break;
    case HTTPC_ERROR_READ_TIMEOUT:
        noteNetFail("no reply in 12s");
        break;
    case HTTPC_ERROR_SEND_HEADER_FAILED:
    case HTTPC_ERROR_SEND_PAYLOAD_FAILED:
    case HTTPC_ERROR_NOT_CONNECTED:
    case HTTPC_ERROR_CONNECTION_LOST:
        noteNetFail("the connection lost");
        break;
    default:
        break;
    }
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

    std::vector<Aircraft> result;
    DeserializationError jerr;
    int code = requestInto(url, ep, military, lat, lon, radiusNm, result, jerr);
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
            noteOutcome(code);
            return false;
        }
        Serial.printf("[adsb] %s: connection %d on a reused socket, retrying once\n", provider.name, code);
        code = requestInto(url, ep, military, lat, lon, radiusNm, result, jerr);
    }
    noteOutcome(code);
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

#ifdef BENCH3D
#include <SD.h>

#include "recorder.h"

namespace {

// A reply held in memory, read as the socket is - see BlockReader.
class MemoryStream : public Stream {
  public:
    MemoryStream(const uint8_t *data, size_t size) : data_(data), size_(size) {}
    int available() override { return (int)(size_ - at_); }
    int read() override { return at_ < size_ ? data_[at_++] : -1; }
    int peek() override { return at_ < size_ ? data_[at_] : -1; }
    size_t readBytes(char *out, size_t length) override {
        size_t n = std::min(length, size_ - at_);
        memcpy(out, data_ + at_, n);
        at_ += n;
        return n;
    }
    size_t write(uint8_t) override { return 0; }

  private:
    const uint8_t *data_;
    size_t size_, at_ = 0;
};

}  // namespace

// "##=p" (bench2d.h): replies from the feed, saved and put on the card as
// bench-*.json in its overhead folder (main.cpp's "##u"), each parsed as a
// poll parses one - from memory, and through the block reader as off the
// socket, with the filter - and the aircraft made from it. Without the
// network, so what is timed is the parse and the build alone.
void adsbBenchParse() {
    struct Reply {
        const char *file;
        double lat, lon;
        int radiusNm;
    };
    static const Reply REPLIES[] = {{"bench-gla60.json", 55.9297, -4.4664, 60},
                                    {"bench-uk_n.json", 55.93, -4.47, 250},
                                    {"bench-uk_s.json", 52.5, -1.0, 250},
                                    {"bench-de.json", 50.5, 8.5, 250}};
    if (!recorder::mount()) {
        Serial.println("[bench] parse: no card");
        return;
    }
    for (const Reply &r : REPLIES) {
        uint8_t *json = nullptr;
        size_t len = 0;
        {
            recorder::CardLock lock;
            File f = SD.open(String("/overhead/") + r.file, FILE_READ);
            if (!f) {
                Serial.printf("[bench] parse: no %s on the card\n", r.file);
                continue;
            }
            len = f.size();
            json = (uint8_t *)heap_caps_malloc(len + 1, MALLOC_CAP_SPIRAM);
            if (json != nullptr) {
                len = f.read(json, len);
            }
            f.close();
        }
        if (json == nullptr) {
            continue;
        }
        // Where a parse's time goes, a layer at a time, per aircraft: reading
        // the bytes; ArduinoJson keeping everything, keeping nothing, keeping
        // the filter's fields; one aircraft at a time without making them.
        {
            uint32_t t0 = micros();
            volatile uint32_t quotes = 0;
            for (size_t k = 0; k < len; k++) quotes += json[k] == '"';
            uint32_t scanUs = micros() - t0;
            JsonDocument all(&g_psram), none(&g_psram), some(&g_psram), keepNothing, fields;
            buildFilter(fields, "ac");
            keepNothing["nothing"] = true;
            t0 = micros();
            deserializeJson(all, (const char *)json, len);
            uint32_t allUs = micros() - t0;
            t0 = micros();
            deserializeJson(none, (const char *)json, len, DeserializationOption::Filter(keepNothing));
            uint32_t noneUs = micros() - t0;
            t0 = micros();
            deserializeJson(some, (const char *)json, len, DeserializationOption::Filter(fields));
            uint32_t someUs = micros() - t0;
            size_t count = std::max<size_t>(1, some["ac"].size());
            all.clear();
            none.clear();
            some.clear();
            // The scanner's reading alone, nothing made of it.
            TextReader text((const char *)json, len);
            PushbackReader<TextReader> reader(text);
            FeedScanner<TextReader> scan(reader);
            t0 = micros();
            size_t parsed = 0;
            if (toArray(reader, "ac")) {
                for (;;) {
                    int c = nextSolid(reader);
                    if (c == ',') c = nextSolid(reader);
                    if (c != '{') break;
                    FeedAircraft f;
                    if (!scan.aircraft(f)) break;
                    parsed++;
                }
            }
            uint32_t eachUs = micros() - t0;
            Serial.printf("[bench] parse %s, us an aircraft: the bytes read %.1f; whole, keeping everything %.1f, "
                          "keeping nothing %.1f, the fields %.1f; the scanner, nothing made %.1f (%u)\n",
                          r.file, scanUs / (float)count, allUs / (float)count, noneUs / (float)count,
                          someUs / (float)count, eachUs / (float)count, (unsigned)parsed);
        }
        constexpr int RUNS = 3;
        uint32_t whole = 0, oneAtATime = 0, offSocket = 0;
        std::vector<Aircraft> before, after;
        size_t inReply = 0;
        bool failed = false;
        for (int i = 0; i < RUNS && !failed; i++) {
            // The way it was: the reply as one document, then the aircraft made.
            {
                JsonDocument filter;
                buildFilter(filter, "ac");
                JsonDocument doc(&g_psram);
                before.clear();
                uint32_t t0 = micros();
                DeserializationError e =
                    deserializeJson(doc, (const char *)json, len, DeserializationOption::Filter(filter));
                if (!e) {
                    parseInto(doc, ADSB_PROVIDERS[0].civil, false, r.lat, r.lon, r.radiusNm, before);
                }
                whole += micros() - t0;
                inReply = doc["ac"].size();
                if (e) {
                    Serial.printf("[bench] parse %s: %s\n", r.file, e.c_str());
                    failed = true;
                    break;
                }
            }
            // One aircraft at a time, from memory and as off the socket.
            after.clear();
            TextReader text((const char *)json, len);
            uint32_t t0 = micros();
            DeserializationError e =
                parseAircraftList(text, ADSB_PROVIDERS[0].civil, false, r.lat, r.lon, r.radiusNm, after);
            oneAtATime += micros() - t0;
            std::vector<Aircraft> socket;
            MemoryStream stream(json, len);
            BlockReader block(stream);
            t0 = micros();
            parseAircraftList(block, ADSB_PROVIDERS[0].civil, false, r.lat, r.lon, r.radiusNm, socket);
            offSocket += micros() - t0;
            if (e) {
                Serial.printf("[bench] parse %s one at a time: %s\n", r.file, e.c_str());
                failed = true;
            }
        }
        if (!failed) {
            // The same aircraft, field for field, either way.
            size_t differ = before.size() == after.size() ? 0 : SIZE_MAX;
            for (size_t i = 0; differ != SIZE_MAX && i < before.size(); i++) {
                const Aircraft &x = before[i], &y = after[i];
                // Floats to a few of their last bits - ArduinoJson's sums in
                // single precision round some a bit differently from the
                // scanner's single rounding - and a distance, which a 0.4m
                // move in a position changes by more than that, to 2m.
                auto near = [](float p, float q) { return fabsf(p - q) <= 5e-7f * std::max(1.0f, fabsf(p)); };
                bool same = x.hex == y.hex && x.callsign == y.callsign && x.type == y.type && x.altStr == y.altStr &&
                            x.speedStr == y.speedStr && x.status == y.status && x.reg == y.reg &&
                            x.squawk == y.squawk && x.category == y.category && x.military == y.military &&
                            x.dbInteresting == y.dbInteresting && x.hasPos == y.hasPos && near(x.lat, y.lat) &&
                            near(x.lon, y.lon) && x.posStale == y.posStale && x.hasTrack == y.hasTrack &&
                            near(x.track, y.track) && x.hasAltGeom == y.hasAltGeom && x.altGeom == y.altGeom &&
                            x.hasVertRate == y.hasVertRate && near(x.vertRate, y.vertRate) &&
                            x.hasDist == y.hasDist && fabsf(x.distNm - y.distNm) <= 0.001f;
                if (!same && differ < 2) {
                    Serial.printf("[bench] parse: %s %s/%s %s/%s %s/%s %.6f/%.6f %.6f/%.6f %.3f/%.3f\n", x.hex.c_str(),
                                  x.callsign.c_str(), y.callsign.c_str(), x.altStr.c_str(), y.altStr.c_str(),
                                  x.speedStr.c_str(), y.speedStr.c_str(), x.lat, y.lat, x.lon, y.lon, x.distNm,
                                  y.distNm);
                }
                differ += same ? 0 : 1;
            }
            Serial.printf("[bench] parse %s %uKB, %u aircraft, %u made: as one document %.2fms, one at a time "
                          "%.2fms from memory and %.2fms through the block reader - %s\n",
                          r.file, (unsigned)(len / 1024), (unsigned)inReply, (unsigned)after.size(),
                          whole / 1000.0f / RUNS, oneAtATime / 1000.0f / RUNS, offSocket / 1000.0f / RUNS,
                          differ == 0          ? "the same aircraft, field for field"
                          : differ == SIZE_MAX ? "A DIFFERENT NUMBER OF AIRCRAFT"
                                               : "SOME AIRCRAFT DIFFERENT");
        }
        heap_caps_free(json);
    }
}
#endif
