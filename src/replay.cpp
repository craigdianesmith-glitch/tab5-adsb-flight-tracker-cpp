#include "replay.h"

#include <SD.h>
#include <algorithm>
#include <esp_heap_caps.h>
#include <math.h>

#include "config.h"
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
};

struct Fix {
    uint32_t key;  // the aircraft, from keyOf()
    uint32_t ms;
    float lat, lon;
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
            uint32_t epoch = strtoul(end, nullptr, 10);
            g_frames.push_back({offset, frameMs, epoch});
        } else if (g_line[0] == 'A' && !g_frames.empty()) {
            Aircraft a;
            if (recorder::parseAircraftLine(g_line, a) && a.hasPos) {
                g_fixes.push_back({keyOf(a.hex), frameMs, a.lat, a.lon});
            }
        }
    }
    if (progress != nullptr) {
        progress(total, total);
    }
    std::sort(g_fixes.begin(), g_fixes.end(), [](const Fix &a, const Fix &b) {
        return a.key != b.key ? a.key < b.key : a.ms < b.ms;
    });
    g_loaded = true;
    Serial.printf("[replay] %s: %u polls, %u fixes, %lus\n", path.c_str(), (unsigned)g_frames.size(),
                  (unsigned)g_fixes.size(), (unsigned long)(durationMs() / 1000));
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
