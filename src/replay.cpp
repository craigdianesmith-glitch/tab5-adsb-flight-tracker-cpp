#include "replay.h"

#include <SD.h>
#include <algorithm>
#include <map>
#include <esp_heap_caps.h>
#include <math.h>

#include "config.h"
#include "follow.h"
#include "psram_alloc.h"

namespace replay {
namespace {

// Between two polls further apart than this - a run of failed fetches, say -
// a contact is left where it was seen rather than slid across the gap, since
// a straight line over minutes says nothing true about where it went.
constexpr uint32_t MAX_GLIDE_GAP_MS = 180000;

struct Frame {
    uint32_t offset;  // of its F line in the file
    uint32_t ms;
    uint32_t epoch;
    View view;
    uint8_t provider;  // which answered it, as an index into g_providers; 0 for not known
};

struct Fix {
    uint32_t key;  // the aircraft, from keyOf()
    uint32_t ms;
    float lat, lon;
    uint32_t frame;  // which poll
    // The rest of the report, for estimating where it went once the feed
    // lost it - see Gap.
    FollowSample s;
    char callsign[9];
};

// A low contact the feed lost - a landing below the receivers' horizon, a
// take-off roll before it climbs into their view - from its last current
// position until the next, or ESTIMATE_LOST_MS: where the replay puts it
// where it is estimated to be, as the live screens do.
struct Gap {
    size_t fix;  // its last current position, in g_fixes
    uint32_t startMs, endMs;
};

// Reads lines through a buffer of its own: File::read a byte at a time goes
// through the VFS layer for every byte and is far too slow to scan a file with.
// Static rather than on the stack, which on the loop task is only 8KB.
class LineReader {
public:
    void attach(File *f) {
        f_ = f;
        seek(0);
    }
    void seek(uint32_t offset) {
        f_->seek(offset);
        base_ = offset;
        len_ = pos_ = 0;
    }
    // The next line, without its newline, and the file offset it started at.
    bool next(char *out, size_t cap, uint32_t &offset) {
        offset = base_ + pos_;
        size_t n = 0;
        for (;;) {
            if (pos_ >= len_) {
                base_ += len_;
                len_ = f_->read(buf_, sizeof(buf_));
                pos_ = 0;
                if (len_ == 0) {
                    out[n] = 0;
                    return n > 0;
                }
            }
            char c = (char)buf_[pos_++];
            if (c == '\n') {
                out[n] = 0;
                return true;
            }
            if (n < cap - 1) {
                out[n++] = c;
            }
        }
    }

private:
    File *f_ = nullptr;
    uint8_t buf_[4096];
    uint32_t base_ = 0;
    size_t len_ = 0, pos_ = 0;
};

LineReader g_reader;
char g_line[320];

File g_file;
bool g_loaded = false;
recorder::Header g_header;
std::vector<Frame> g_frames;
// The trail index is the one thing here that grows with the recording: a
// point per aircraft per poll, so tens of thousands over a long one. Kept in
// PSRAM, where there is room for it, rather than the internal RAM that the
// WiFi and TLS stacks need.
std::vector<Fix, PsramAllocator<Fix>> g_fixes;  // sorted by key, then time
std::vector<Gap, PsramAllocator<Gap>> g_gaps;
// For each aircraft followed, one per poll that reported it while it was: a
// few hundred over a landing at the zoom's ten seconds, so small enough to
// keep anywhere.
std::map<String, std::vector<FollowSample>> g_heights;
std::vector<String> g_providers;  // the names the polls give, in the order first met
bool g_hasHome = false;

// Each low contact's RunwaySim, fed its fixes up to the moment last shown
// as the live screens feed theirs at each poll, so that a take-off or
// landing plays back as it was shown live. Fed on from where it was as the
// playback moves on, and started again - from SIM_WARMUP_MS before - when
// it goes back, or has gathered too many.
struct SimFeed {
    RunwaySim sim;
    size_t next;  // the next of its fixes to feed it, in g_fixes
};
std::map<uint32_t, SimFeed> g_sims;
uint32_t g_simsAt = 0;
constexpr uint32_t SIM_WARMUP_MS = 300000;
constexpr size_t SIMS_MAX = 64;

// The two polls either side of the moment last asked for, kept so that a
// playback ticking through one gap doesn't reread them from the card each time.
size_t g_curIdx = SIZE_MAX, g_nextIdx = SIZE_MAX;
std::vector<Aircraft> g_cur, g_next;

// The 24-bit ICAO address as a number. readsb prefixes a '~' to addresses
// that aren't ICAO ones (TIS-B and the like); those get a bit of their own so
// they can't collide with a real one.
uint32_t keyOf(const String &hex) {
    if (hex.length() && hex[0] == '~') {
        return 0x1000000u | (uint32_t)strtoul(hex.c_str() + 1, nullptr, 16);
    }
    return (uint32_t)strtoul(hex.c_str(), nullptr, 16);
}

String hexOf(uint32_t key) {
    char buf[12];
    snprintf(buf, sizeof(buf), (key & 0x1000000u) ? "~%06lx" : "%06lx", (unsigned long)(key & 0xFFFFFFu));
    return String(buf);
}

// An F line's view token - see recorder.h - including an older following
// recording's bare `-` or airport code, whose aircraft the header names.
View parseView(const char *tok) {
    View v;
    if (!tok[0] || strcmp(tok, "H") == 0) {
        return v;
    }
    if (tok[0] == '-') {
        v.kind = View::FOLLOW;
        strlcpy(v.hex, tok[1] ? tok + 1 : g_header.followHex.c_str(), sizeof(v.hex));
        return v;
    }
    v.kind = View::ZOOM;
    const char *slash = strchr(tok, '/');
    if (slash) {
        size_t n = std::min<size_t>(slash - tok, sizeof(v.code) - 1);
        memcpy(v.code, tok, n);
        v.code[n] = 0;
        strlcpy(v.hex, slash + 1, sizeof(v.hex));
    } else {
        strlcpy(v.code, tok, sizeof(v.code));
        strlcpy(v.hex, g_header.followHex.c_str(), sizeof(v.hex));
    }
    return v;
}

// Low enough that a gap in the feed is the ground's doing, not its leaving.
bool low(const FollowSample &s) { return s.ground || (s.hasAlt && s.altFt < ESTIMATE_MAX_FT); }

bool low(const Aircraft &a) {
    return a.status == "GROUND" || a.status == "TAXI" || a.altStr == "GND" ||
           (a.altStr != "?" && a.altStr.toInt() < ESTIMATE_MAX_FT);
}

// The aircraft with `key`'s simulation, fed its fixes up to `abs`. Only good
// until the next call: that can start them all again.
const RunwaySim &simFor(uint32_t key, uint32_t abs) {
    if (abs < g_simsAt || g_sims.size() > SIMS_MAX) {
        g_sims.clear();
    }
    g_simsAt = abs;
    auto it = g_sims.find(key);
    if (it == g_sims.end()) {
        auto f = std::lower_bound(g_fixes.begin(), g_fixes.end(), key,
                                  [](const Fix &f, uint32_t k) { return f.key < k; });
        while (f != g_fixes.end() && f->key == key && f->ms + SIM_WARMUP_MS < abs) {
            ++f;
        }
        it = g_sims.emplace(key, SimFeed{RunwaySim(), (size_t)(f - g_fixes.begin())}).first;
    }
    SimFeed &feed = it->second;
    while (feed.next < g_fixes.size() && g_fixes[feed.next].key == key && g_fixes[feed.next].ms <= abs) {
        feed.sim.report(g_fixes[feed.next].s);
        feed.next++;
    }
    return feed.sim;
}

// `a` where `e` puts it, as the live screens draw it.
void showEstimate(Aircraft &a, const FollowEstimate &e) {
    a.hasPos = true;
    a.lat = e.lat;
    a.lon = e.lon;
    a.hasTrack = true;
    a.track = e.track;
    a.altStr = e.ground ? String("GND") : String(e.altFt);
    a.speedStr = String(e.gsKt);
    a.status = e.ground                           ? (e.gsKt > 2 ? "TAXI" : "GROUND")
               : e.vsFpm >= CLIMB_THRESHOLD_FPM   ? "CLIMB"
               : e.vsFpm <= DESCEND_THRESHOLD_FPM ? "DESCEND"
                                                  : "LEVEL";
}

// Where each contact went missing that the feed lost - rather than one that
// flew out of the poll's reach - from the sorted fixes: a current position
// whose next current one, if there is one, is in a later poll than the very
// next. Lost, that is, if it was low, where the receivers lose them; or at
// any height, if the poll it went missing from was answered by a different
// provider, which may simply not have had it. Not where the view changed
// between the two polls, as from a zoom to the radar: the poll's reach
// changed with it.
void findGaps() {
    g_gaps.clear();
    size_t lastFrame = g_frames.empty() ? 0 : g_frames.size() - 1;
    for (size_t i = 0; i < g_fixes.size(); i++) {
        const Fix &f = g_fixes[i];
        if (f.s.posStale || f.frame >= lastFrame) {
            continue;
        }
        const Frame &was = g_frames[f.frame], &then = g_frames[f.frame + 1];
        bool otherProvider = was.provider && then.provider && was.provider != then.provider;
        if (!(low(f.s) || otherProvider) || was.view.kind != then.view.kind ||
            strcmp(was.view.code, then.view.code) != 0) {
            continue;
        }
        size_t next = i + 1;
        while (next < g_fixes.size() && g_fixes[next].key == f.key && g_fixes[next].s.posStale) {
            next++;
        }
        bool haveNext = next < g_fixes.size() && g_fixes[next].key == f.key;
        if (haveNext ? g_fixes[next].frame <= f.frame + 1 : f.frame >= lastFrame) {
            continue;  // seen again at the next poll, or the recording ended
        }
        // As long as the longest it could be kept: whether it is, is up to
        // its simulation, at the time.
        uint32_t end = f.ms + std::max(ESTIMATE_LOST_MS, RUNWAY_SIM_LOST_MS);
        if (haveNext) {
            end = std::min(end, g_fixes[next].ms);
        }
        g_gaps.push_back({i, f.ms, end});
    }
}

// The contacts lost at `abs`, put into `aircraft` where they are estimated
// to be, in place of any stale position the poll had for them. The aircraft
// followed then is left to the playback screen, which keeps it longer.
void fillGaps(uint32_t abs, const char *followed, std::vector<Aircraft> &aircraft) {
    for (const Gap &g : g_gaps) {
        if (abs < g.startMs || abs >= g.endMs) {
            continue;
        }
        const Fix &f = g_fixes[g.fix];
        String hex = hexOf(f.key);
        if (hex == followed) {
            continue;
        }
        const RunwaySim &sim = simFor(f.key, abs);
        if (!sim.haveFresh() || abs - sim.fresh().ms > sim.keepMs()) {
            continue;
        }
        FollowEstimate e = sim.at(abs);
        Aircraft ghost;
        for (auto it = aircraft.begin(); it != aircraft.end(); ++it) {
            if (it->hex == hex) {
                ghost = *it;
                aircraft.erase(it);
                break;
            }
        }
        ghost.hex = hex;
        if (!ghost.callsign.length()) {
            ghost.callsign = f.callsign;
            ghost.type = "----";
        }
        showEstimate(ghost, e);
        ghost.posStale = true;
        aircraft.push_back(ghost);
    }
}

bool readFrame(size_t i, std::vector<Aircraft> &out) {
    out.clear();
    recorder::CardLock lock;
    if (!g_file) {
        return false;
    }
    g_reader.seek(g_frames[i].offset);
    uint32_t offset;
    if (!g_reader.next(g_line, sizeof(g_line), offset)) {  // the F line itself
        return false;
    }
    while (g_reader.next(g_line, sizeof(g_line), offset)) {
        if (g_line[0] == 'F') {
            break;
        }
        Aircraft a;
        if (recorder::parseAircraftLine(g_line, a)) {
            out.push_back(std::move(a));
        }
    }
    return true;
}

void ensureCached(size_t i, size_t &idx, std::vector<Aircraft> &cache) {
    if (idx == i) {
        return;
    }
    // The next poll of the last pair is often the current one of this pair.
    if (&cache == &g_cur && g_nextIdx == i) {
        g_cur.swap(g_next);
        g_curIdx = i;
        g_nextIdx = SIZE_MAX;
        return;
    }
    readFrame(i, cache);
    idx = i;
}

}  // namespace

bool load(const String &path, LoadProgress progress) {
    unload();
    if (!recorder::mount()) {
        return false;
    }
    recorder::CardLock lock;
    g_file = SD.open(path, FILE_READ);
    if (!g_file) {
        return false;
    }
    g_reader.attach(&g_file);
    g_header = recorder::Header();

    uint32_t offset;
    bool inHeader = true;
    uint32_t frameMs = 0;
    uint32_t total = g_file.size();
    uint32_t step = total / 100 + 1;
    uint32_t nextReport = step;
    while (g_reader.next(g_line, sizeof(g_line), offset)) {
        if (progress != nullptr && offset >= nextReport) {
            progress(offset, total);
            nextReport = offset + step;
        }
        if (inHeader) {
            if (strcmp(g_line, "end-header") == 0) {
                inHeader = false;
            } else {
                recorder::parseHeaderLine(g_line, g_header);
            }
            continue;
        }
        if (g_line[0] == 'F') {
            char *end = nullptr;
            frameMs = strtoul(g_line + 2, &end, 10);
            uint32_t epoch = strtoul(end, &end, 10);
            Frame f = {offset, frameMs, epoch, View(), 0};
            char viewTok[24] = "", providerTok[24] = "";
            sscanf(end, "%23s %23s", viewTok, providerTok);
            f.view = parseView(viewTok);
            g_hasHome = g_hasHome || f.view.kind == View::HOME;
            if (providerTok[0] && strcmp(providerTok, "?") != 0) {
                auto it = std::find(g_providers.begin(), g_providers.end(), String(providerTok));
                if (it == g_providers.end()) {
                    g_providers.push_back(providerTok);
                    it = g_providers.end() - 1;
                }
                f.provider = (uint8_t)(it - g_providers.begin() + 1);
            }
            g_frames.push_back(f);
        } else if (g_line[0] == 'A' && !g_frames.empty()) {
            Aircraft a;
            if (recorder::parseAircraftLine(g_line, a)) {
                if (a.hasPos) {
                    Fix fix = {keyOf(a.hex), frameMs, a.lat, a.lon, (uint32_t)(g_frames.size() - 1),
                               followSampleOf(a, frameMs), ""};
                    strlcpy(fix.callsign, a.callsign.c_str(), sizeof(fix.callsign));
                    g_fixes.push_back(fix);
                }
                const char *followed = g_frames.back().view.hex;
                if (followed[0] && a.hex == followed) {
                    g_heights[a.hex].push_back(followSampleOf(a, frameMs - g_frames.front().ms));
                }
            }
        }
    }
    if (progress != nullptr) {
        progress(total, total);
    }
    std::sort(g_fixes.begin(), g_fixes.end(), [](const Fix &a, const Fix &b) {
        return a.key != b.key ? a.key < b.key : a.ms < b.ms;
    });
    findGaps();
    g_loaded = true;
    Serial.printf("[replay] %s: %u polls, %u fixes, %u gaps filled, %lus\n", path.c_str(), (unsigned)g_frames.size(),
                  (unsigned)g_fixes.size(), (unsigned)g_gaps.size(), (unsigned long)(durationMs() / 1000));
    return true;
}

void unload() {
    {
        recorder::CardLock lock;
        if (g_file) {
            g_file.close();
        }
    }
    g_loaded = false;
    g_frames.clear();
    g_frames.shrink_to_fit();
    g_fixes.clear();
    g_fixes.shrink_to_fit();
    g_gaps.clear();
    g_gaps.shrink_to_fit();
    g_heights.clear();
    g_sims.clear();
    g_simsAt = 0;
    g_providers.clear();
    g_hasHome = false;
    g_cur.clear();
    g_next.clear();
    g_curIdx = g_nextIdx = SIZE_MAX;
}

bool loaded() { return g_loaded; }

const recorder::Header &header() { return g_header; }

size_t frameCount() { return g_frames.size(); }

uint32_t durationMs() { return g_frames.empty() ? 0 : g_frames.back().ms - g_frames.front().ms; }

size_t frameIndexAt(uint32_t t) {
    if (g_frames.empty()) {
        return 0;
    }
    uint32_t abs = g_frames.front().ms + t;
    auto it = std::upper_bound(g_frames.begin(), g_frames.end(), abs,
                               [](uint32_t v, const Frame &f) { return v < f.ms; });
    return it == g_frames.begin() ? 0 : (size_t)(it - g_frames.begin()) - 1;
}

uint32_t frameMs(size_t i) { return i < g_frames.size() ? g_frames[i].ms - g_frames.front().ms : 0; }

uint32_t epochAt(uint32_t t) {
    if (g_frames.empty()) {
        return 0;
    }
    const Frame &f = g_frames[frameIndexAt(t)];
    if (f.epoch == 0) {
        return 0;
    }
    return f.epoch + (g_frames.front().ms + t - f.ms) / 1000;
}

const std::vector<FollowSample> &followHeights(const String &hex) {
    static const std::vector<FollowSample> none;
    auto it = g_heights.find(hex);
    return it == g_heights.end() ? none : it->second;
}

bool estimateAt(const String &hex, uint32_t t, FollowEstimate &out, uint32_t &lostMs) {
    if (g_frames.empty()) {
        return false;
    }
    uint32_t abs = g_frames.front().ms + t;
    const RunwaySim &sim = simFor(keyOf(hex), abs);
    if (!sim.haveFresh() || abs - sim.fresh().ms > std::max(FOLLOW_LOST_MS, sim.keepMs())) {
        return false;
    }
    out = sim.at(abs);
    lostMs = abs - sim.fresh().ms;
    return true;
}

bool followedAt(uint32_t t, Aircraft &out) {
    if (g_frames.empty()) {
        return false;
    }
    size_t i = frameIndexAt(t);
    const char *hex = g_frames[i].view.hex;
    if (!hex[0]) {
        return false;
    }
    ensureCached(i, g_curIdx, g_cur);
    for (const Aircraft &a : g_cur) {
        if (a.hex == hex) {
            out = a;
            return true;
        }
    }
    return false;
}

String departedFrom(uint32_t t) {
    if (g_frames.empty()) {
        return String();
    }
    size_t i = frameIndexAt(t);
    const View &v = g_frames[i].view;
    if (v.kind != View::FOLLOW || !v.hex[0]) {
        return String();
    }
    // Back through the polls following it, to the zoom it was in before.
    for (size_t j = i; j-- > 0;) {
        const View &w = g_frames[j].view;
        if (strcmp(w.hex, v.hex) != 0) {
            break;
        }
        if (w.kind != View::ZOOM) {
            continue;
        }
        uint32_t then = frameMs(j);
        for (const FollowSample &s : followHeights(v.hex)) {
            if (s.ms <= then && s.ground) {
                return String(w.code);  // on the ground there: it departed from it
            }
        }
        break;  // in the air there: a go-around, not a departure
    }
    return String();
}

String callsignOf(const String &hex) {
    uint32_t key = keyOf(hex);
    auto lo = std::lower_bound(g_fixes.begin(), g_fixes.end(), key,
                               [](const Fix &f, uint32_t k) { return f.key < k; });
    for (auto it = lo; it != g_fixes.end() && it->key == key; ++it) {
        if (it->callsign[0]) {
            return String(it->callsign);
        }
    }
    return hex;
}

View viewAt(uint32_t t) { return g_frames.empty() ? View() : g_frames[frameIndexAt(t)].view; }

bool hasHomeView() { return g_hasHome; }

bool lastFixOf(const String &hex, uint32_t t, float &lat, float &lon) {
    if (g_frames.empty()) {
        return false;
    }
    uint32_t key = keyOf(hex);
    uint32_t abs = g_frames.front().ms + t;
    auto lo = std::lower_bound(g_fixes.begin(), g_fixes.end(), key,
                               [](const Fix &f, uint32_t k) { return f.key < k; });
    bool found = false;
    for (auto it = lo; it != g_fixes.end() && it->key == key && it->ms <= abs; ++it) {
        lat = it->lat;
        lon = it->lon;
        found = true;
    }
    return found;
}

void sceneAt(uint32_t t, std::vector<Aircraft> &aircraft, std::vector<TrailPoint> &trails) {
    aircraft.clear();
    trails.clear();
    if (g_frames.empty()) {
        return;
    }
    size_t i = frameIndexAt(t);
    ensureCached(i, g_curIdx, g_cur);
    bool haveNext = (i + 1 < g_frames.size());
    if (haveNext) {
        ensureCached(i + 1, g_nextIdx, g_next);
    }

    uint32_t abs = g_frames.front().ms + t;
    float frac = 0;
    if (haveNext) {
        uint32_t span = g_frames[i + 1].ms - g_frames[i].ms;
        if (span > 0 && span <= MAX_GLIDE_GAP_MS) {
            frac = std::min(1.0f, (float)(abs - g_frames[i].ms) / span);
        }
    }

    aircraft = g_cur;
    for (Aircraft &a : aircraft) {
        if (!a.hasPos || frac <= 0) {
            continue;
        }
        for (const Aircraft &n : g_next) {
            if (n.hasPos && n.hex == a.hex) {
                a.lat += (n.lat - a.lat) * frac;
                a.lon += (n.lon - a.lon) * frac;
                break;
            }
        }
    }

    // Taking off or landing, where it was shown live rather than glided
    // between its reports.
    for (Aircraft &a : aircraft) {
        if (!a.hasPos || a.posStale || !low(a)) {
            continue;
        }
        const RunwaySim &sim = simFor(keyOf(a.hex), abs);
        if (sim.simulating(abs)) {
            showEstimate(a, sim.at(abs));
        }
    }

    fillGaps(abs, g_frames[i].view.hex, aircraft);

    // Trails only for the aircraft on the plot now: those that have left took
    // their trails with them, which keeps a long recording's plot readable.
    uint32_t from = abs > REPLAY_TRAIL_MS ? abs - REPLAY_TRAIL_MS : 0;
    uint32_t upTo = g_frames[i].ms;
    for (const Aircraft &a : aircraft) {
        if (!a.hasPos) {
            continue;
        }
        uint32_t key = keyOf(a.hex);
        auto lo = std::lower_bound(g_fixes.begin(), g_fixes.end(), key,
                                   [](const Fix &f, uint32_t k) { return f.key < k; });
        trails.push_back({NAN, NAN});
        for (auto it = lo; it != g_fixes.end() && it->key == key && it->ms <= upTo; ++it) {
            if (it->ms >= from) {
                trails.push_back({it->lat, it->lon});
            }
        }
        trails.push_back({a.lat, a.lon});
    }
}

}  // namespace replay
