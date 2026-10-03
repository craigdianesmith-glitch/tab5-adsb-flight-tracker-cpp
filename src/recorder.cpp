#include "recorder.h"

#include <M5Unified.h>
#include <SD.h>
#include <SPI.h>
#include <algorithm>
#include <time.h>

namespace recorder {
namespace {

constexpr const char *DIR = "/overhead";
constexpr const char *MAGIC = "OVERHEAD-RECORDING 1";

// As the header's trigger line has it. An older build reads "follow" as
// manual, which is near enough: nobody pressed anything, but nothing alerted.
const char *triggerName(Trigger t) {
    return t == Trigger::AUTO ? "auto" : t == Trigger::FOLLOW ? "follow" : "manual";
}
// The card is wired for 4-bit SDIO, but SPI mode on the same pins is what
// M5Stack's own Tab5 examples use, and a few KB per poll needs nothing faster.
constexpr uint32_t SD_SPI_HZ = 25000000;

// Recursive, so a CardLock held by a reader doesn't deadlock against a
// function here that the same task calls while holding it.
SemaphoreHandle_t g_lock = xSemaphoreCreateRecursiveMutex();

bool g_spiStarted = false;
volatile bool g_mounted = false;  // read without the lock; see mounted()
// A card that isn't there takes SD.begin() a while to give up on, and the
// poll task asks on every poll while auto-record is waiting to start - so
// after a failure, it isn't asked again for a few seconds.
constexpr uint32_t MOUNT_RETRY_MS = 5000;
uint32_t g_mountFailedMs = 0;

File g_file;
// Read without the lock by the UI every pass of its loop, which must not wait
// behind a write in progress on the poll task. Each is a single word, written
// only under the lock.
volatile bool g_active = false;
volatile Trigger g_trigger = Trigger::MANUAL;
volatile uint32_t g_startMs = 0;
String g_path;
// The header the recording was started with, kept for the parts after the
// first; and the part being written, which is what decides when to split.
Header g_header;
uint32_t g_fileStartMs = 0;
uint32_t g_fileBytes = 0;

struct Lock {
    Lock() { xSemaphoreTakeRecursive(g_lock, portMAX_DELAY); }
    ~Lock() { xSemaphoreGiveRecursive(g_lock); }
};

void unmountLocked() {
    SD.end();
    g_mounted = false;
}

bool mountLocked() {
    if (g_mounted) {
        return true;
    }
    if (g_mountFailedMs != 0 && millis() - g_mountFailedMs < MOUNT_RETRY_MS) {
        return false;
    }
    int sck = M5.getPin(m5::pin_name_t::sd_spi_sclk);
    int miso = M5.getPin(m5::pin_name_t::sd_spi_miso);
    int mosi = M5.getPin(m5::pin_name_t::sd_spi_mosi);
    int cs = M5.getPin(m5::pin_name_t::sd_spi_cs);
    if (sck < 0 || miso < 0 || mosi < 0 || cs < 0) {
        return false;  // a board without a card slot
    }
    if (!g_spiStarted) {
        SPI.begin(sck, miso, mosi, cs);
        g_spiStarted = true;
    }
    if (!SD.begin(cs, SPI, SD_SPI_HZ, "/sd", 5, false) || SD.cardType() == CARD_NONE) {
        SD.end();
        g_mountFailedMs = millis() | 1;  // never 0, which means "no failure"
        return false;
    }
    if (!SD.exists(DIR)) {
        SD.mkdir(DIR);
    }
    // Written, read back and removed: a card that mounts but can't take a
    // write would otherwise only say so once a recording had been lost to it.
    static const char PROBE[] = "/overhead/.probe";
    bool writable = false;
    File probe = SD.open(PROBE, FILE_WRITE);
    if (probe) {
        writable = probe.print("ok") == 2;
        probe.close();
        probe = SD.open(PROBE, FILE_READ);
        writable = writable && probe && probe.size() == 2;
        if (probe) {
            probe.close();
        }
        SD.remove(PROBE);
    }
    if (!writable) {
        Serial.println("[sd] card found but can't be written to - recording unavailable");
        SD.end();
        g_mountFailedMs = millis() | 1;
        return false;
    }
    g_mounted = true;
    g_mountFailedMs = 0;
    Serial.printf("[sd] card mounted, %llu MB\n", (unsigned long long)(SD.cardSize() / (1024 * 1024)));
    return true;
}

// Named by the UTC time it started where the clock has been set from the
// network, so the files sort by date on the card and on a laptop alike;
// numbered otherwise.
String newPathLocked() {
    char buf[48];
    time_t now = time(nullptr);
    if (now > 1700000000) {
        struct tm tm;
        gmtime_r(&now, &tm);
        strftime(buf, sizeof(buf), "/overhead/%Y%m%d-%H%M%S.rec", &tm);
        if (!SD.exists(buf)) {
            return String(buf);
        }
    }
    for (int n = 1; n < 10000; n++) {
        snprintf(buf, sizeof(buf), "%s/rec-%04d.rec", DIR, n);
        if (!SD.exists(buf)) {
            return String(buf);
        }
    }
    return String();
}

void closeLocked() {
    if (g_file) {
        g_file.close();
    }
    g_active = false;
    g_path = "";
}

// A feed's text, made safe for a recording's '|'-separated lines: a stray
// separator or line break would shift every field after it, and an absurdly
// long value would overflow the line. Nothing the feed sends legitimately
// comes near either.
String field(const String &s, size_t maxLen) {
    String out = s.length() > maxLen ? s.substring(0, maxLen) : s;
    out.replace('|', '/');
    out.replace('\n', ' ');
    out.replace('\r', ' ');
    return out;
}

// Takes the text after a header key, or nullptr if the line isn't that key.
const char *after(const char *line, const char *key) {
    size_t n = strlen(key);
    if (strncmp(line, key, n) != 0 || line[n] != ' ') {
        return nullptr;
    }
    return line + n + 1;
}

}  // namespace

bool mount() {
    Lock lock;
    return mountLocked();
}

bool mounted() { return g_mounted; }

uint64_t cardBytes() {
    Lock lock;
    return g_mounted ? SD.cardSize() : 0;
}

namespace {

// Opens a new file and writes the header into it, as the part being written.
bool openPartLocked(const Header &h) {
    String path = newPathLocked();
    if (!path.length()) {
        return false;
    }
    g_file = SD.open(path, FILE_WRITE);
    if (!g_file) {
        // Most likely the card has gone since it was mounted; forget it so the
        // next attempt mounts afresh rather than failing the same way.
        unmountLocked();
        return false;
    }
    String note = field(h.note, 80);
    // Only a following recording has the follow line, so an older build
    // reads any other exactly as before.
    char follow[40] = "";
    if (h.followHex.length()) {
        snprintf(follow, sizeof(follow), "follow %s %d\n", field(h.followHex, 8).c_str(), h.followRangeNm);
    }
    char buf[320];
    int n = snprintf(buf, sizeof(buf),
                     "%s\nstart %lu\ntrigger %s\nnote %s\nhome %.5f %.5f\nradius %d\ntraffic %s\npart %d\n%send-header\n",
                     MAGIC, (unsigned long)h.startEpoch, triggerName(h.trigger), note.c_str(),
                     h.lat, h.lon, h.radiusNm, h.military ? "military" : "civil", h.part, follow);
    size_t want = std::min<size_t>(n, sizeof(buf) - 1);
    size_t wrote = g_file.write((const uint8_t *)buf, want);
    g_file.flush();
    if (wrote != want) {
        // Full, or gone: no use starting a recording that can't be written.
        Serial.printf("[rec] can't write %s - card full or removed\n", path.c_str());
        g_file.close();
        SD.remove(path);
        unmountLocked();
        return false;
    }
    g_path = path;
    g_fileStartMs = millis();
    g_fileBytes = wrote;
    return true;
}

// Closes the part being written and carries on in the next. Called with a
// frame about to be written, so the new part starts with it.
bool nextPartLocked() {
    String done = g_path;
    g_file.close();
    g_header.part++;
    time_t now = time(nullptr);
    g_header.startEpoch = now > 1700000000 ? (uint32_t)now : 0;
    if (!openPartLocked(g_header)) {
        Serial.printf("[rec] couldn't start part %d after %s - stopping\n", g_header.part, done.c_str());
        closeLocked();
        return false;
    }
    Serial.printf("[rec] %s full - continuing in %s (part %d)\n", done.c_str(), g_path.c_str(), g_header.part);
    return true;
}

}  // namespace

bool start(const Header &h) {
    Lock lock;
    if (g_active) {
        return true;
    }
    if (!mountLocked()) {
        return false;
    }
    g_header = h;
    g_header.part = 1;
    if (!openPartLocked(g_header)) {
        return false;
    }
    g_active = true;
    g_trigger = h.trigger;
    g_startMs = millis();
    Serial.printf("[rec] started %s (%s)\n", g_path.c_str(), triggerName(h.trigger));
    return true;
}

void stop() {
    Lock lock;
    if (!g_active) {
        return;
    }
    Serial.printf("[rec] stopped %s after %lus\n", g_path.c_str(), (unsigned long)((millis() - g_startMs) / 1000));
    closeLocked();
}

bool active() { return g_active; }

Trigger activeTrigger() { return g_trigger; }

uint32_t activeSinceMs() { return g_startMs; }

String activePath() {
    Lock lock;
    return g_path;
}

void addFrame(const std::vector<Aircraft> &aircraft, const String &view, const char *provider) {
    Lock lock;
    if (!g_active) {
        return;
    }
    if ((millis() - g_fileStartMs >= RECORDING_SPLIT_MS || g_fileBytes >= RECORDING_SPLIT_BYTES) &&
        !nextPartLocked()) {
        return;
    }
    // Built whole and written in one go: one write and one flush per poll,
    // rather than one per aircraft, is what keeps the card's time to itself.
    String out;
    out.reserve(32 + aircraft.size() * 96);
    char line[256];
    time_t now = time(nullptr);
    // The view and provider go last, where an older build's reading of the
    // line stops short of them. Neither may hold a space.
    String v = field(view, 16), p = field(provider ? String(provider) : String("?"), 16);
    v.replace(' ', '_');
    p.replace(' ', '_');
    snprintf(line, sizeof(line), "F %lu %lu %s %s\n", (unsigned long)(millis() - g_fileStartMs),
             (unsigned long)(now > 1700000000 ? now : 0), v.c_str(), p.c_str());
    out += line;
    for (const Aircraft &a : aircraft) {
        char track[12] = "", lat[16] = "", lon[16] = "", vrate[12] = "", altGeom[12] = "", dist[12] = "";
        if (a.hasTrack) snprintf(track, sizeof(track), "%.0f", a.track);
        if (a.hasPos) {
            snprintf(lat, sizeof(lat), "%.5f", a.lat);
            snprintf(lon, sizeof(lon), "%.5f", a.lon);
        }
        if (a.hasVertRate) snprintf(vrate, sizeof(vrate), "%.0f", a.vertRate);
        if (a.hasAltGeom) snprintf(altGeom, sizeof(altGeom), "%d", a.altGeom);
        if (a.hasDist) snprintf(dist, sizeof(dist), "%.1f", a.distNm);
        int flags = (a.military ? 1 : 0) | (a.dbInteresting ? 2 : 0) | (a.posStale ? 4 : 0);
        snprintf(line, sizeof(line), "A %s|%s|%s|%s|%s|%s|%d|%d|%s|%s|%s|%s|%s|%s|%s|%s|%s\n",
                 field(a.hex, 8).c_str(), field(a.callsign, 12).c_str(), field(a.type, 8).c_str(),
                 field(a.altStr, 12).c_str(), field(a.speedStr, 8).c_str(), field(a.status, 12).c_str(), flags,
                 a.alert, track, lat, lon, field(a.squawk, 6).c_str(), field(a.reg, 12).c_str(),
                 field(a.category, 4).c_str(), vrate, altGeom, dist);
        out += line;
    }
    size_t written = g_file.write((const uint8_t *)out.c_str(), out.length());
    g_fileBytes += written;
    // Flushed every poll, so a card pulled or a battery run flat mid-recording
    // loses at most the poll in progress rather than everything since the start.
    g_file.flush();
    if (written != out.length()) {
        Serial.printf("[rec] write failed on %s - stopping\n", g_path.c_str());
        closeLocked();
        unmountLocked();
    }
}

namespace {

// The files in `dir` ending in `ext`, newest first - which is reverse name
// order, since names are the UTC time they were started.
std::vector<String> listLocked(const char *dirPath, const char *ext) {
    std::vector<String> out;
    if (!mountLocked()) {
        return out;
    }
    File dir = SD.open(dirPath);
    if (!dir) {
        if (strcmp(dirPath, DIR) != 0) {
            return out;  // no videos yet: the folder is made by the first export
        }
        // A card swapped since it was mounted reads as an unopenable root.
        unmountLocked();
        if (!mountLocked() || !(dir = SD.open(dirPath))) {
            return out;
        }
    }
    // Names only: opening each file to ask its name made the list slower with
    // every recording kept, and auto-record can leave a good many.
    bool isDir = false;
    for (String path = dir.getNextFileName(&isDir); path.length(); path = dir.getNextFileName(&isDir)) {
        if (!isDir && path.endsWith(ext)) {
            out.push_back(path);
        }
    }
    dir.close();
    std::sort(out.begin(), out.end(), [](const String &a, const String &b) { return a > b; });
    return out;
}

String baseName(const String &path) {
    String name = path.substring(path.lastIndexOf('/') + 1);
    int dot = name.lastIndexOf('.');
    return dot > 0 ? name.substring(0, dot) : name;
}

}  // namespace

std::vector<String> list() {
    Lock lock;
    return listLocked(DIR, ".rec");
}

std::vector<String> videos() {
    Lock lock;
    return listLocked(VIDEO_DIR, ".mp4");
}

std::vector<String> videosOf(const String &recordingPath, const std::vector<String> &videos) {
    std::vector<String> out;
    String prefix = String(VIDEO_DIR) + "/" + baseName(recordingPath) + "-";
    for (const String &v : videos) {
        if (v.startsWith(prefix)) {
            out.push_back(v);
        }
    }
    return out;
}

bool removeVideo(const String &path) {
    Lock lock;
    String prefix = String(VIDEO_DIR) + "/";
    // Nothing but a file directly in the video folder: this is reachable from
    // the web page, where the path arrives from outside.
    if (!g_mounted || !path.startsWith(prefix) || !path.endsWith(".mp4") || path.indexOf("..") >= 0 ||
        path.indexOf('/', prefix.length()) >= 0) {
        return false;
    }
    return SD.remove(path);
}

bool readHeader(const String &path, Header &out, uint32_t &bytes) {
    Lock lock;
    if (!g_mounted) {
        return false;
    }
    File f = SD.open(path, FILE_READ);
    if (!f) {
        return false;
    }
    bytes = f.size();
    char buf[512];
    size_t n = f.read((uint8_t *)buf, sizeof(buf) - 1);
    f.close();
    buf[n] = 0;
    char *save = nullptr;
    char *line = strtok_r(buf, "\n", &save);
    if (line == nullptr || strcmp(line, MAGIC) != 0) {
        return false;
    }
    while ((line = strtok_r(nullptr, "\n", &save)) != nullptr && strcmp(line, "end-header") != 0) {
        parseHeaderLine(line, out);
    }
    return true;
}

bool remove(const String &path) {
    Lock lock;
    if (!g_mounted || path == g_path) {
        return false;
    }
    return SD.remove(path);
}

CardLock::CardLock() { xSemaphoreTakeRecursive(g_lock, portMAX_DELAY); }
CardLock::~CardLock() { xSemaphoreGiveRecursive(g_lock); }

bool parseHeaderLine(const char *line, Header &h) {
    const char *v;
    if ((v = after(line, "start"))) {
        h.startEpoch = strtoul(v, nullptr, 10);
    } else if ((v = after(line, "trigger"))) {
        h.trigger = strcmp(v, "auto") == 0     ? Trigger::AUTO
                    : strcmp(v, "follow") == 0 ? Trigger::FOLLOW
                                               : Trigger::MANUAL;
    } else if ((v = after(line, "note"))) {
        h.note = v;
    } else if ((v = after(line, "home"))) {
        char *end = nullptr;
        h.lat = strtod(v, &end);
        h.lon = strtod(end, nullptr);
    } else if ((v = after(line, "radius"))) {
        h.radiusNm = atoi(v);
    } else if ((v = after(line, "traffic"))) {
        h.military = (strcmp(v, "military") == 0);
    } else if ((v = after(line, "part"))) {
        h.part = std::max(1, atoi(v));
    } else if ((v = after(line, "follow"))) {
        const char *space = strchr(v, ' ');
        h.followHex = space ? String(v).substring(0, space - v) : String(v);
        h.followRangeNm = space ? atoi(space + 1) : 0;
    } else {
        return false;
    }
    return true;
}

bool parseAircraftLine(char *line, Aircraft &a) {
    if (line[0] != 'A' || line[1] != ' ') {
        return false;
    }
    constexpr int FIELDS = 17;
    char *f[FIELDS];
    int n = 0;
    f[n++] = line + 2;
    for (char *p = line + 2; *p; p++) {
        if (*p == '|') {
            *p = 0;
            if (n < FIELDS) {
                f[n++] = p + 1;
            }
        } else if (*p == '\r' || *p == '\n') {
            *p = 0;
            break;
        }
    }
    if (n < FIELDS) {
        return false;
    }
    a.hex = f[0];
    a.callsign = f[1];
    a.type = f[2];
    a.altStr = f[3];
    a.speedStr = f[4];
    a.status = f[5];
    int flags = atoi(f[6]);
    a.military = (flags & 1) != 0;
    a.dbInteresting = (flags & 2) != 0;
    a.posStale = (flags & 4) != 0;
    a.alert = (uint8_t)atoi(f[7]);
    a.hasTrack = f[8][0] != 0;
    a.track = a.hasTrack ? strtof(f[8], nullptr) : 0;
    a.hasPos = f[9][0] != 0 && f[10][0] != 0;
    a.lat = a.hasPos ? strtof(f[9], nullptr) : 0;
    a.lon = a.hasPos ? strtof(f[10], nullptr) : 0;
    a.squawk = f[11];
    a.reg = f[12];
    a.category = f[13];
    a.hasVertRate = f[14][0] != 0;
    a.vertRate = a.hasVertRate ? strtof(f[14], nullptr) : 0;
    a.hasAltGeom = f[15][0] != 0;
    a.altGeom = a.hasAltGeom ? atoi(f[15]) : 0;
    a.hasDist = f[16][0] != 0;
    a.distNm = a.hasDist ? strtof(f[16], nullptr) : 0;
    return true;
}

}  // namespace recorder
