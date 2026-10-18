// Soak-tests the 3D view's ground movement on the PC - the one followed,
// landed and taxiing in to its stand, or pushed back from it and taxiing out
// to take off: src/ground_follow.cpp, ground_taxi.cpp and ground_route.cpp -
// at real airports, from airfields.bin. Each run is a made-up aircraft
// really doing so there, reported as the feeds report one: a poll every 5
// to 60 seconds, its place metres out and its aerial ahead of its wheels,
// some reports late, some missing, some wild. What the 3D view would show of
// it is checked, frame by frame, for what would look wrong:
//
//   jump        moved further in a frame than its speed takes it
//   snap-turn   turned faster than it can - or at all, standing still
//   backwards   going backwards, not being pushed
//   sideways    going one way, pointing another
//   tight-turn  turned about in less room than an aircraft can
//   loop        gone round in a circle
//   grass       its wheels off the paved ground
//   building    its nose, tail or wheels in a building; wing-building, a wingtip
//   stall       stopped, its reports going on without it
//   far         a long way from where it really is, for long
//   lurch       handed back to the take-off's simulation from somewhere else
//   wrong-stand / not-parked / parked-off   where it ends up, landed
//
// Build and run, from the repo's root:
//
//   g++ -std=c++17 -O2 -Iinclude tools/ground_soak/soak.cpp src/ground_route.cpp src/ground_taxi.cpp
//       src/ground_follow.cpp src/airfield_pack.cpp src/scene3d.cpp -o /tmp/ground_soak
//   /tmp/ground_soak airfields.bin DUB LHR CDG --runs 100
//
// (all one line). --list prints every run, clean ones too; -DGROUND_TRACE
// adds the follower's own reasoning to --trace.
//
// A run again on its own, said second by second, and drawn: --run N with
// --trace, and --svg DIR for its picture - the taxiways, buildings and
// stands, where it really went (blue), where it was shown (orange), its
// reports (red) and what looked wrong (magenta). --kinds arrive,depart picks
// the runs' kinds: arrive (landed, handed over at 40 knots, in to a stand),
// depart (parked, pushed back, out to a runway, lined up and rolling),
// arrive-pickup and depart-pickup (the view opened on it taxiing), parked.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "airfield_pack.h"
#include "ground_follow.h"
#include "ground_route.h"
#include "ground_taxi.h"
#include "scene3d.h"

using namespace ground;
using airfield::List;

namespace {

constexpr float PI = (float)M_PI, DEG = PI / 180, KT = 0.5144f;
constexpr uint32_t BASE_MS = 10000000;  // the reports' clock at the run's start

float dist(Vec2 a, Vec2 b) { return hypotf(b.x - a.x, b.y - a.y); }
Vec2 aheadOf(Vec2 p, float h, float d) { return {p.x + cosf(h) * d, p.y + sinf(h) * d}; }
float wrap(float a) { return remainderf(a, 2 * PI); }
uint32_t msAt(float t) { return BASE_MS + (uint32_t)lrintf(t * 1000); }

// --- random numbers: a stream for each run, from its number ---------------------

struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s((seed + 1) * 0x9E3779B97F4A7C15ull ^ 0xD1B54A32D192ED03ull) {
        next();
        next();
    }
    uint64_t next() {
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return s * 0x2545F4914F6CDD1Dull;
    }
    float uniform() { return (float)((next() >> 40) * (1.0 / (1ull << 24))); }
    float range(float a, float b) { return a + (b - a) * uniform(); }
    int below(int n) { return n <= 0 ? 0 : (int)(next() % (uint64_t)n); }
    bool chance(float p) { return uniform() < p; }
    float normal() {
        float u = std::max(1e-7f, uniform()), v = uniform();
        return sqrtf(-2 * logf(u)) * cosf(2 * PI * v);
    }
};

// --- the pack ----------------------------------------------------------------------

struct Pack {
    std::vector<uint8_t> file;
    struct Row {
        char code[5];
        uint32_t at, size;
    };
    std::vector<Row> rows;

    uint32_t u32(size_t i) const {
        return file[i] | file[i + 1] << 8 | file[i + 2] << 16 | (uint32_t)file[i + 3] << 24;
    }
    bool open(const char *path) {
        FILE *fp = fopen(path, "rb");
        if (fp == nullptr) {
            return false;
        }
        fseek(fp, 0, SEEK_END);
        long n = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        file.resize(n > 0 ? n : 0);
        size_t got = fread(file.data(), 1, file.size(), fp);
        fclose(fp);
        if (got != file.size() || file.size() < 16 || memcmp(file.data(), "OVAF", 4) != 0) {
            return false;
        }
        unsigned count = file[6] | file[7] << 8;
        uint32_t index = u32(8);
        for (unsigned i = 0; i < count; i++) {
            Row r{};
            memcpy(r.code, &file[index + i * 12], 4);
            r.at = u32(index + i * 12 + 4);
            r.size = u32(index + i * 12 + 8);
            rows.push_back(r);
        }
        return true;
    }
    bool layout(const char *code, airfield::Layout &out) const {
        for (const Row &r : rows) {
            if (strcmp(r.code, code) == 0) {
                return airfield::parse(&file[r.at], r.size, out);
            }
        }
        return false;
    }
};

// --- the airfield's buildings and paved ground, by 40m squares ---------------------

struct Ground {
    struct Tri {
        Vec2 a, b, c;
    };
    struct Seg {
        Vec2 a, b;
        float half;
    };
    static constexpr float CELL = 40;
    float x0 = 0, y0 = 0;
    int w = 0, h = 0;
    std::vector<Tri> buildings, surfaces;
    std::vector<Seg> paved;
    std::vector<std::vector<uint32_t>> buildingAt, surfaceAt, pavedAt;

    int cellOf(Vec2 p) const {
        int i = (int)floorf((p.x - x0) / CELL), j = (int)floorf((p.y - y0) / CELL);
        return i < 0 || j < 0 || i >= w || j >= h ? -1 : j * w + i;
    }
    template <typename F>
    void cover(float ax, float ay, float bx, float by, F f) {
        int i0 = std::max(0, (int)floorf((std::min(ax, bx) - x0) / CELL));
        int i1 = std::min(w - 1, (int)floorf((std::max(ax, bx) - x0) / CELL));
        int j0 = std::max(0, (int)floorf((std::min(ay, by) - y0) / CELL));
        int j1 = std::min(h - 1, (int)floorf((std::max(ay, by) - y0) / CELL));
        for (int j = j0; j <= j1; j++) {
            for (int i = i0; i <= i1; i++) {
                f(j * w + i);
            }
        }
    }
    static bool inTri(const Tri &t, Vec2 p) {
        auto side = [](Vec2 a, Vec2 b, Vec2 q) { return (b.x - a.x) * (q.y - a.y) - (b.y - a.y) * (q.x - a.x); };
        float s0 = side(t.a, t.b, p), s1 = side(t.b, t.c, p), s2 = side(t.c, t.a, p);
        return (s0 >= 0 && s1 >= 0 && s2 >= 0) || (s0 <= 0 && s1 <= 0 && s2 <= 0);
    }
    static float segDist(Vec2 p, Vec2 a, Vec2 b) {
        float dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        float t = len2 < 1e-6f ? 0 : std::max(0.0f, std::min(1.0f, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
        return hypotf(a.x + dx * t - p.x, a.y + dy * t - p.y);
    }

    void build(const airfield::Layout &l, const Network &net) {
        float bx0 = 1e30f, by0 = 1e30f, bx1 = -1e30f, by1 = -1e30f;
        auto grow = [&](float x, float y) {
            bx0 = std::min(bx0, x), by0 = std::min(by0, y), bx1 = std::max(bx1, x), by1 = std::max(by1, y);
        };
        for (const airfield::Point &p : l.junctions) {
            grow(p.x, p.y);
        }
        for (const airfield::Area &a : l.surfaces) {
            for (const airfield::Point &p : a.corners) {
                grow(p.x, p.y);
            }
        }
        for (const airfield::Area &a : l.buildings) {
            for (const airfield::Point &p : a.corners) {
                grow(p.x, p.y);
            }
        }
        if (bx0 > bx1) {
            bx0 = by0 = -100, bx1 = by1 = 100;
        }
        x0 = bx0 - 400, y0 = by0 - 400;
        w = (int)((bx1 + 400 - x0) / CELL) + 1, h = (int)((by1 + 400 - y0) / CELL) + 1;
        buildingAt.assign((size_t)w * h, {});
        surfaceAt.assign((size_t)w * h, {});
        pavedAt.assign((size_t)w * h, {});
        auto triangles = [&](const List<airfield::Area> &areas, std::vector<Tri> &tris,
                             std::vector<std::vector<uint32_t>> &at) {
            for (const airfield::Area &a : areas) {
                for (size_t t = 0; t + 2 < a.triangles.size(); t += 3) {
                    const airfield::Point &p = a.corners[a.triangles[t]], &q = a.corners[a.triangles[t + 1]],
                                          &r = a.corners[a.triangles[t + 2]];
                    uint32_t k = (uint32_t)tris.size();
                    tris.push_back({{p.x, p.y}, {q.x, q.y}, {r.x, r.y}});
                    cover(std::min({p.x, q.x, r.x}), std::min({p.y, q.y, r.y}), std::max({p.x, q.x, r.x}),
                          std::max({p.y, q.y, r.y}), [&](int c) { at[c].push_back(k); });
                }
            }
        };
        triangles(l.buildings, buildings, buildingAt);
        triangles(l.surfaces, surfaces, surfaceAt);
        auto addSeg = [&](Vec2 a, Vec2 b, float half) {
            uint32_t k = (uint32_t)paved.size();
            paved.push_back({a, b, half});
            cover(std::min(a.x, b.x) - half, std::min(a.y, b.y) - half, std::max(a.x, b.x) + half,
                  std::max(a.y, b.y) + half, [&](int c) { pavedAt[c].push_back(k); });
        };
        for (const airfield::Way &way : l.ways) {
            float half = way.width > 1 ? way.width / 2
                         : way.kind == airfield::WayKind::RUNWAY ? 22.5f
                         : way.kind == airfield::WayKind::TAXIWAY ? 11.5f
                                                                  : 7.5f;
            Vec2 prev = {l.junctions[way.from].x, l.junctions[way.from].y};
            for (const airfield::Point &p : way.between) {
                addSeg(prev, {p.x, p.y}, half);
                prev = {p.x, p.y};
            }
            addSeg(prev, {l.junctions[way.to].x, l.junctions[way.to].y}, half);
        }
        for (const StandWay &st : net.stands()) {
            if (!st.ok) {
                continue;
            }
            for (size_t i = 1; i < st.line.size(); i++) {
                addSeg(st.line[i - 1], st.line[i], 8);
            }
            addSeg(st.line.back(), st.line.back(), 15);
        }
    }
    bool inBuilding(Vec2 p) const {
        int c = cellOf(p);
        if (c < 0) {
            return false;
        }
        for (uint32_t k : buildingAt[c]) {
            if (inTri(buildings[k], p)) {
                return true;
            }
        }
        return false;
    }
    bool onSurface(Vec2 p) const {
        int c = cellOf(p);
        if (c < 0) {
            return false;
        }
        for (uint32_t k : surfaceAt[c]) {
            if (inTri(surfaces[k], p)) {
                return true;
            }
        }
        return false;
    }
    // How far `p` is beyond the paved ground - the edges of the taxiways,
    // taxilanes and runways as wide as they are, the aprons, the stands'
    // lines - metres: 0 on it, up to 80 looked for.
    float offPaved(Vec2 p) const {
        if (onSurface(p)) {
            return 0;
        }
        int ci = (int)floorf((p.x - x0) / CELL), cj = (int)floorf((p.y - y0) / CELL);
        float best = 80;
        for (int j = cj - 2; j <= cj + 2; j++) {
            for (int i = ci - 2; i <= ci + 2; i++) {
                if (i < 0 || j < 0 || i >= w || j >= h) {
                    continue;
                }
                for (uint32_t k : pavedAt[j * w + i]) {
                    best = std::min(best, segDist(p, paved[k].a, paved[k].b) - paved[k].half);
                }
            }
        }
        return std::max(0.0f, best);
    }
};

// --- the aircraft --------------------------------------------------------------------

struct Type {
    const char *code;
    float noseWheel, nose, length, span;
};

std::vector<Type> allTypes() {
    std::vector<Type> out;
    // Mostly narrowbodies, as at a busy airport.
    for (const char *code : {"A320", "A320", "B738", "B738", "A321", "A319", "B77W", "A333", "B789", "A388", "AT76",
                             "E190"}) {
        scene3d::Look look = scene3d::lookFor(code, 0);
        float length = scene3d::sizeOf(look) * 37.6f;
        out.push_back({code, scene3d::noseWheelAhead(look), scene3d::noseAhead(look), length, length * 0.95f});
    }
    return out;
}

// --- where it really goes ------------------------------------------------------------

struct Leg {
    enum Kind : uint8_t { WAIT, TAXI, PUSH, ROLL } kind = WAIT;
    Plan plan;
    float want = 0;     // its speed on the straights, metres a second
    float seconds = 0;  // WAIT: for how long
    bool landing = false;
    bool lineUp = false;     // onto the runway, lining up: the take-off's from here on
    bool afterPush = false;  // nose first again: its wheels' trail as if it came this way
    std::vector<float> stopsAt, stopFor;  // where along it, it stops a while - giving way - and for how long
    int stand = -1;
    std::string what;
};

struct Sample {
    float t;
    Vec2 at;
    float heading, speed, track;
    bool onRunway;  // lined up, or rolling
};

struct Truth {
    std::vector<Leg> legs;
    std::vector<Sample> history;
    Taxi taxi;
    size_t leg = 0;
    bool started = false, done = false, pausing = false, onRunway = false, airborne = false;
    float waited = 0, paused = 0, track = 0;
    size_t nextStop = 0;

    void start(Vec2 at, float heading, float speed) {
        taxi.place(at, heading, speed);
        track = heading;
    }
    const Leg *current() const { return leg < legs.size() ? &legs[leg] : nullptr; }
    void step(float t, float dt) {
        Vec2 before = taxi.at();
        if (leg < legs.size()) {
            Leg &L = legs[leg];
            if (!started) {
                started = true;
                waited = paused = 0;
                pausing = false;
                nextStop = 0;
                if (L.kind != Leg::WAIT) {
                    if (L.afterPush) {
                        taxi.place(taxi.at(), taxi.heading(), 0);
                    }
                    taxi.take(L.plan);
                    taxi.pushing = L.kind == Leg::PUSH;
                    taxi.landing = L.landing;
                    taxi.takingOff = L.kind == Leg::ROLL;
                }
            }
            if (L.kind == Leg::WAIT) {
                taxi.step(dt, 0);
                waited += dt;
                if (waited >= L.seconds) {
                    leg++;
                    started = false;
                }
            } else {
                float want = L.want;
                if (!pausing && nextStop < L.stopsAt.size() && taxi.along() >= L.stopsAt[nextStop]) {
                    pausing = true;
                    paused = 0;
                }
                if (pausing) {
                    want = std::max(0.0f, taxi.speed() - 1.2f * dt);
                    if (taxi.speed() < 0.02f) {
                        paused += dt;
                        if (paused >= L.stopFor[nextStop]) {
                            pausing = false;
                            nextStop++;
                        }
                    }
                }
                taxi.step(dt, want);
                if (L.kind == Leg::ROLL) {
                    airborne = taxi.speed() > 75;  // off the ground, well past the view's hand back
                } else if (taxi.arrived() && !pausing) {
                    taxi.pushing = false;
                    taxi.landing = false;
                    leg++;
                    started = false;
                }
            }
            onRunway = onRunway || (L.lineUp && L.plan.kindAt(taxi.along()) == (uint8_t)airfield::WayKind::RUNWAY &&
                                    taxi.along() > L.plan.length() - 80) ||
                       L.kind == Leg::ROLL;
        }
        done = leg >= legs.size() || airborne;
        float moved = dist(before, taxi.at());
        if (moved > 0.02f) {
            track = atan2f(taxi.at().y - before.y, taxi.at().x - before.x);
        }
        history.push_back({t, taxi.at(), taxi.heading(), dt > 0 ? moved / dt : 0, track, onRunway});
    }
    Sample at(float t) const {
        if (history.empty()) {
            return {t, taxi.at(), taxi.heading(), 0, track, false};
        }
        if (t <= history.front().t) {
            Sample s = history.front();
            s.t = t;
            return s;
        }
        size_t i = std::upper_bound(history.begin(), history.end(), t,
                                    [](float v, const Sample &s) { return v < s.t; }) -
                   history.begin();
        if (i >= history.size()) {
            return history.back();
        }
        const Sample &a = history[i - 1], &b = history[i];
        float k = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
        Sample s = b;
        s.t = t;
        s.at = {a.at.x + (b.at.x - a.at.x) * k, a.at.y + (b.at.y - a.at.y) * k};
        return s;
    }
};

// --- its reports, as the feeds have them ------------------------------------------

struct Report {
    float t;
    Vec2 at;
    float heading;  // its track, radians anticlockwise from east; NAN none
    float speed;    // metres a second, whole knots
    bool wild;
};

struct Feed {
    float interval = 10;      // seconds between polls
    float noise = 3;          // metres, each way, moving
    float stillNoise = 5;     // standing still - wandering more, in a terminal's shadow
    float antenna = 8;        // metres ahead of its main wheels
    float wild = 0;           // the chance a report is wildly out
    float drop = 0;           // the chance a poll misses it
    float late = 2;           // its place up to this many seconds old
    std::vector<std::pair<float, float>> gaps;  // none at all from, to
    float nextPoll = 0;
    bool lost = false;  // the last poll came back without it, as the app's g_follow.lost
    Vec2 wander = {0, 0};
    float lastTrack = 0;

    bool make(float t, const Truth &truth, Rng &rng, Report &out) {
        if (t < nextPoll) {
            return false;
        }
        nextPoll = t + interval * rng.range(0.9f, 1.1f);
        lost = true;
        for (const std::pair<float, float> &g : gaps) {
            if (t >= g.first && t < g.second) {
                return false;
            }
        }
        if (rng.chance(drop)) {
            return false;
        }
        lost = false;
        Sample s = truth.at(t - rng.range(0, late));
        bool still = s.speed < 0.3f;
        Vec2 p = aheadOf(s.at, s.heading, antenna);
        float n = still ? stillNoise : noise;
        if (still) {
            wander = {std::max(-20.0f, std::min(20.0f, wander.x + rng.normal() * stillNoise * 0.3f)),
                      std::max(-20.0f, std::min(20.0f, wander.y + rng.normal() * stillNoise * 0.3f))};
        } else {
            wander = {wander.x * 0.5f, wander.y * 0.5f};
        }
        p = {p.x + rng.normal() * n + (still ? wander.x : 0), p.y + rng.normal() * n + (still ? wander.y : 0)};
        out.wild = rng.chance(wild);
        if (out.wild) {
            float a = rng.range(0, 2 * PI), d = rng.range(40, 250);
            p = {p.x + cosf(a) * d, p.y + sinf(a) * d};
        }
        out.t = t;
        out.at = p;
        float kt = s.speed / KT;
        if (still) {
            kt = rng.chance(0.6f) ? 0 : (float)rng.below(4);
        } else {
            kt = std::max(0.0f, roundf(kt + rng.normal() * 0.6f));
        }
        out.speed = kt * KT;
        if (s.speed > 1) {
            lastTrack = s.track + rng.normal() * 2 * DEG;
            out.heading = lastTrack;
        } else {
            // Standing still, its track anything - or the last it had.
            out.heading = rng.chance(0.15f) ? NAN : rng.chance(0.5f) ? rng.range(-PI, PI) : lastTrack;
        }
        return true;
    }
};

// --- what looked wrong ---------------------------------------------------------------

struct Worst {
    int count = 0;
    float size = 0, t = 0;
    Vec2 at = {0, 0};
    std::string doing;
};

struct Glitches {
    std::map<std::string, Worst> kinds;
    void add(const char *kind, float size, float t, Vec2 at, const char *doing) {
        Worst &w = kinds[kind];
        w.count++;
        if (size > w.size || w.count == 1) {
            w.size = size;
            w.t = t;
            w.at = at;
            w.doing = doing;
        }
    }
};

// --- a run ---------------------------------------------------------------------------

enum Kind : uint8_t { ARRIVE, DEPART, ARRIVE_LOST, ARRIVE_PICKUP, DEPART_PICKUP, PARKED, DEPART_LINED, KINDS };
const char *KIND_NAMES[KINDS] = {"arrive", "depart", "arrive-lost", "arrive-pickup", "depart-pickup", "parked",
                                 "depart-lined"};

struct Airport {
    std::string code;
    airfield::Layout layout;
    Network net;
    Ground ground;
    std::vector<int> runwayWays, endHolds, stands, terminalStands;
};

struct Run {
    const Airport *ap = nullptr;
    int number = 0;
    Kind kind = ARRIVE;
    Type type{};
    Feed feed;
    Truth truth;
    std::vector<Vec2> standing;
    std::vector<Vec2> departFrom;  // the runway ends in use, as the follower is told them
    float frameDt = 1 / 20.0f;
    float beginAt = 0;
    bool arrived = false, landedBegin = false;
    int stand = -1;   // arriving: where it really parks; departing: where from
    int hold = -1;    // departing: where it really holds
    float lostAt = 1e9f;  // arrive-lost: when its signal goes, for good
    std::string skip;  // why the run couldn't be made
    std::string story;  // what it really does, for the log
};

void append(List<Vec2> &pts, List<uint8_t> &kinds, const List<Vec2> &more, const List<uint8_t> &moreKinds,
            uint8_t kind = 0) {
    for (size_t i = 0; i < more.size(); i++) {
        if (!pts.empty() && dist(pts.back(), more[i]) < 0.3f) {
            continue;
        }
        pts.push_back(more[i]);
        kinds.push_back(kind != 0 ? kind : i < moreKinds.size() ? moreKinds[i] : (uint8_t)airfield::WayKind::TAXIWAY);
    }
}

// The sharpest turn along `pts`, radians: a turn about where it is near PI.
float sharpest(const List<Vec2> &pts) {
    float worst = 0;
    for (size_t i = 1; i + 1 < pts.size(); i++) {
        float a = atan2f(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
        float b = atan2f(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
        if (dist(pts[i - 1], pts[i]) > 0.5f && dist(pts[i], pts[i + 1]) > 0.5f) {
            worst = std::max(worst, fabsf(wrap(b - a)));
        }
    }
    return worst;
}

// Whether `plan` turns about anywhere - by more than 120 degrees in 20m - as
// a route round by somewhere else can, through a junction's corners: not a
// way anyone is sent.
bool hairpin(const List<Vec2> &pts, const List<uint8_t> &kinds) {
    Plan plan;
    plan.build(pts, kinds, true);
    float length = plan.length(), h0 = 0, h1 = 0;
    for (float s = 0; s + 20 < length; s += 2) {
        plan.pointAt(s, h0);
        plan.pointAt(s + 20, h1);
        if (fabsf(wrap(h1 - h0)) > 120 * DEG) {
            return true;
        }
    }
    return false;
}

// The way it really taxis from `from` - facing `heading` - to `to`, arriving
// `arrive`: as the network's search finds it, or now and then round by
// somewhere else on the way, as a controller might send it.
bool taxiRoute(const Network &net, const airfield::Layout &l, Rng &rng, const Place &from, float heading,
               const Place &to, float arrive, const Rules &rules, bool via, List<Vec2> &pts, List<uint8_t> &kinds) {
    net.search(from, heading, 30000, rules);
    if (!net.pathTo(to, arrive, pts, &kinds) || pts.size() < 2) {
        return false;
    }
    float direct = lengthOf(pts);
    if (!via || direct < 400) {
        return true;
    }
    for (int tries = 0; tries < 8; tries++) {
        float h;
        Vec2 mid = along(pts, direct * rng.range(0.2f, 0.8f), h);
        float a = rng.range(0, 2 * PI), d = rng.range(60, 350);
        Place v;
        if (!net.snap({mid.x + cosf(a) * d, mid.y + sinf(a) * d}, NAN, 80, v) || net.onRunway(v)) {
            continue;
        }
        List<Vec2> first, second;
        List<uint8_t> firstKinds, secondKinds;
        net.search(from, heading, 30000, rules);
        if (!net.pathTo(v, NAN, first, &firstKinds) || first.size() < 2) {
            continue;
        }
        size_t n = first.size();
        float vh = atan2f(first[n - 1].y - first[n - 2].y, first[n - 1].x - first[n - 2].x);
        Rules on = rules;
        on.back = false;
        net.search(v, vh, 30000, on);
        if (!net.pathTo(to, arrive, second, &secondKinds) || second.size() < 2) {
            continue;
        }
        List<Vec2> all = first;
        List<uint8_t> allKinds = firstKinds;
        append(all, allKinds, second, secondKinds);
        if (lengthOf(all) > direct * 1.8f + 600 || sharpest(all) > 2.4f || hairpin(all, allKinds)) {
            continue;
        }
        pts = all;
        kinds = allKinds;
        return true;
    }
    (void)l;
    return true;
}

// Where along `plan` - on a taxiway or taxilane, well away from its ends - it
// stops a while, giving way.
void giveWay(Leg &leg, Rng &rng, float chance, int most) {
    float length = leg.plan.length();
    if (length < 400 || !rng.chance(chance)) {
        return;
    }
    int n = 1 + rng.below(most);
    for (int i = 0; i < n; i++) {
        float s = rng.range(150, length - 200);
        uint8_t k = leg.plan.kindAt(s);
        if (k == (uint8_t)airfield::WayKind::TAXIWAY || k == (uint8_t)airfield::WayKind::TAXILANE) {
            leg.stopsAt.push_back(s);
        }
    }
    std::sort(leg.stopsAt.begin(), leg.stopsAt.end());
    for (size_t i = 0; i < leg.stopsAt.size(); i++) {
        leg.stopFor.push_back(rng.range(8, 75));
    }
}

int pickStand(const Airport &ap, Rng &rng, const std::vector<Vec2> &standing, int notThis = -1) {
    const std::vector<int> &from = !ap.terminalStands.empty() && rng.chance(0.85f) ? ap.terminalStands : ap.stands;
    for (int tries = 0; tries < 40 && !from.empty(); tries++) {
        int k = from[rng.below((int)from.size())];
        if (k == notThis) {
            continue;
        }
        const StandWay &st = ap.net.stands()[k];
        bool taken = false;
        for (const Vec2 &s : standing) {
            taken = taken || dist(s, st.line.back()) < 30;
        }
        if (!taken) {
            return k;
        }
    }
    return -1;
}

// Others parked about: a share of the stands taken, as fillTraffic() tells
// the follower - by their stops.
void parkOthers(Run &run, Rng &rng) {
    float share = rng.range(0, 0.75f);
    for (int k : run.ap->stands) {
        if (rng.chance(share)) {
            run.standing.push_back(run.ap->net.stands()[k].line.back());
        }
    }
}

// The way onto the runway from end hold `hold`, lined up down its longer way,
// as the follower's lineUp() has it: false if there is none.
bool lineUpPath(const Network &net, int hold, List<Vec2> &path, List<uint8_t> &kinds, Place &runway,
                float &takeoff) {
    const HoldWay &h = net.holds()[hold];
    List<Vec2> on;
    List<uint8_t> onKinds;
    net.ahead(h.at, h.arrive, 300, on, onKinds);
    bool met = false;
    for (size_t i = 0; i < on.size() && !met; i++) {
        if (onKinds[i] == (uint8_t)airfield::WayKind::RUNWAY) {
            for (float reach = 2; reach <= 30 && !met; reach += 4) {
                met = net.snap(on[i], NAN, reach, runway, true) && net.onRunway(runway);
            }
        }
    }
    if (!met) {
        return false;
    }
    float along = net.heading(runway);
    List<Vec2> one, other;
    List<uint8_t> oneKinds, otherKinds;
    net.ahead(runway, along, 4000, one, oneKinds);
    net.ahead(runway, along + PI, 4000, other, otherKinds);
    takeoff = lengthOf(one) >= lengthOf(other) ? along : along + PI;
    Rules rules;
    rules.runwayFactor = 3;
    net.search(h.at, h.arrive, 3000, rules);
    if (net.costTo(runway, takeoff) > 500 || !net.pathTo(runway, takeoff, path, &kinds)) {
        return false;
    }
    net.ahead(runway, takeoff, 60, on, onKinds);
    append(path, kinds, on, onKinds);
    return true;
}

// The run's made-up aircraft, its feed and the view's start: false, and why
// in run.skip, if this airport hasn't what it takes.
bool setUp(Run &run, Rng &rng, const std::vector<Type> &types) {
    const Airport &ap = *run.ap;
    const Network &net = ap.net;
    run.type = types[rng.below((int)types.size())];
    static const float POLLS[] = {5, 10, 10, 15, 30, 30, 60};
    Feed &f = run.feed;
    f.interval = POLLS[rng.below(7)];
    f.noise = rng.chance(0.5f) ? rng.range(1, 4) : rng.range(4, 12);
    f.stillNoise = rng.range(2, 14);
    f.antenna = rng.range(0, std::max(1.0f, run.type.nose - 2));
    f.wild = rng.chance(0.6f) ? 0 : rng.range(0.01f, 0.06f);
    f.drop = rng.chance(0.5f) ? 0 : rng.range(0.05f, 0.3f);
    f.late = rng.chance(0.8f) ? rng.range(0, 3) : rng.range(3, 10);
    static const float FRAMES[] = {1 / 30.0f, 1 / 20.0f, 1 / 15.0f, 1 / 12.0f};
    run.frameDt = FRAMES[rng.below(4)];
    parkOthers(run, rng);
    float back = 0;

    if (run.kind == ARRIVE || run.kind == ARRIVE_LOST || run.kind == ARRIVE_PICKUP) {
        if (ap.runwayWays.empty() || ap.stands.empty()) {
            run.skip = "no runway or stands";
            return false;
        }
        for (int tries = 0; tries < 40; tries++) {
            // Landing: a place down a runway with the rest of its roll ahead.
            const airfield::Way &w = ap.layout.ways[ap.runwayWays[rng.below((int)ap.runwayWays.size())]];
            List<Vec2> pts;
            pts.push_back({ap.layout.junctions[w.from].x, ap.layout.junctions[w.from].y});
            for (const airfield::Point &p : w.between) {
                pts.push_back({p.x, p.y});
            }
            pts.push_back({ap.layout.junctions[w.to].x, ap.layout.junctions[w.to].y});
            float h0;
            Vec2 p = along(pts, lengthOf(pts) * rng.range(0.05f, 0.95f), h0);
            Place on;
            if (!net.snap(p, NAN, 5, on, true)) {
                continue;
            }
            float heading = net.heading(on) + (rng.chance(0.5f) ? PI : 0);
            List<Vec2> ahead;
            List<uint8_t> aheadKinds;
            net.ahead(on, heading, 3000, ahead, aheadKinds);
            float runway = 0;
            for (size_t i = 1; i < ahead.size() && aheadKinds[i] == (uint8_t)airfield::WayKind::RUNWAY; i++) {
                runway += dist(ahead[i - 1], ahead[i]);
            }
            if (runway < 700) {
                continue;
            }
            int k = pickStand(ap, rng, run.standing);
            if (k < 0) {
                continue;
            }
            const StandWay &st = net.stands()[k];
            Rules rules;
            rules.speed = 20.6f * rng.range(0.85f, 1.3f);
            rules.brake = rng.range(1.0f, 1.8f);
            rules.runwayFactor = 4;  // off at an exit, not taxiing on down it
            rules.back = false;
            List<Vec2> way;
            List<uint8_t> kinds;
            if (!taxiRoute(net, ap.layout, rng, on, heading, st.entry, st.arrive, rules, false, way, kinds)) {
                continue;
            }
            back = parkBack(st, run.type.noseWheel, run.type.nose);
            List<Vec2> line;
            lineIn(st, back, line);
            append(way, kinds, line, {}, ON_STAND_LINE);
            Leg in;
            in.kind = Leg::TAXI;
            in.plan.build(way, kinds, true);
            if (in.plan.empty()) {
                continue;
            }
            in.want = rng.range(7, 12);
            in.landing = true;
            in.stand = k;
            in.what = std::string("taxi in to ") + st.ref;
            giveWay(in, rng, 0.35f, 2);
            Leg parked;
            parked.kind = Leg::WAIT;
            parked.seconds = rng.range(90, 240);
            parked.what = "parked";
            run.truth.legs.push_back(std::move(in));
            run.truth.legs.push_back(std::move(parked));
            run.truth.start(net.at(on), heading, 20.6f);
            run.stand = k;
            run.arrived = true;
            run.landedBegin = run.kind == ARRIVE || run.kind == ARRIVE_LOST;
            run.beginAt = run.landedBegin ? 0 : rng.range(30, std::max(40.0f, run.truth.legs[0].plan.length() / 9));
            if (run.kind == ARRIVE_LOST) {
                // Its signal gone for good: on the rollout, or taxiing in.
                run.lostAt = rng.range(5, std::max(10.0f, std::min(240.0f, run.truth.legs[0].plan.length() / 8 * 0.8f)));
                run.feed.gaps.push_back({run.lostAt, 1e9f});
            }
            char story[96];
            snprintf(story, sizeof story, "landed, %.0fm of taxiing in to stand %s", run.truth.legs[0].plan.length(), st.ref);
            run.story = story;
            if (run.kind == ARRIVE_LOST) {
                snprintf(story, sizeof story, ", its signal lost at %.0fs", run.lostAt);
                run.story += story;
            }
            return true;
        }
        run.skip = "no landing to a stand found";
        return false;
    }

    // Departing - or just parked: at a stand.
    if (ap.stands.empty()) {
        run.skip = "no stands";
        return false;
    }
    for (int tries = 0; tries < 40; tries++) {
        int k = pickStand(ap, rng, run.standing);
        if (k < 0) {
            continue;
        }
        const StandWay &st = net.stands()[k];
        back = parkBack(st, run.type.noseWheel, run.type.nose);
        float parkedHeading = st.heading;
        Vec2 park = parkedAt(st, back, parkedHeading);
        run.stand = k;
        run.truth.start(park, parkedHeading, 0);
        if (run.kind == PARKED) {
            Leg parked;
            parked.kind = Leg::WAIT;
            parked.seconds = 600;
            parked.what = "parked";
            run.truth.legs.push_back(std::move(parked));
            run.story = std::string("parked at ") + st.ref;
            return true;
        }
        if (ap.endHolds.empty()) {
            run.skip = "no runway end holds";
            return false;
        }
        // The runway in use: departures from the holds at its end - and the
        // follower told which that is, as the airport zoom tells it, most of
        // the time.
        int first = ap.endHolds[rng.below((int)ap.endHolds.size())];
        List<Vec2> firstUp;
        List<uint8_t> firstUpKinds;
        Place firstRunway;
        float firstTakeoff;
        if (!lineUpPath(net, first, firstUp, firstUpKinds, firstRunway, firstTakeoff)) {
            continue;
        }
        Vec2 inUse = net.at(firstRunway);
        std::vector<int> atEnd;
        for (int k2 : ap.endHolds) {
            if (dist(net.at(net.holds()[k2].at), inUse) < 700) {
                atEnd.push_back(k2);
            }
        }
        int hold = atEnd.empty() ? first : atEnd[rng.below((int)atEnd.size())];
        run.departFrom.clear();
        if (rng.chance(0.8f)) {
            run.departFrom.push_back(inUse);
        }
        const HoldWay &hw = net.holds()[hold];
        // Pushed back to face the way it will go.
        float lane = net.heading(st.entry), best = 1e30f, go = lane;
        Rules rules;
        rules.runwayFactor = 3;
        rules.back = false;
        for (float d : {lane, lane + PI}) {
            net.search(st.entry, d, 30000, rules);
            float c = net.costTo(hw.at, hw.arrive);
            if (c < best) {
                best = c;
                go = d;
            }
        }
        if (best > 1e29f) {
            continue;
        }
        List<Vec2> line, push, lanePts;
        List<uint8_t> pushKinds, laneKinds;
        lineIn(st, back, line);
        for (size_t i = line.size(); i-- > 0;) {
            push.push_back(line[i]);
            pushKinds.push_back(ON_STAND_LINE);
        }
        net.ahead(st.entry, go + PI, rng.range(25, 45), lanePts, laneKinds);
        append(push, pushKinds, lanePts, laneKinds);
        Leg pushLeg;
        pushLeg.kind = Leg::PUSH;
        pushLeg.plan.build(push, pushKinds, true);
        pushLeg.want = rng.range(1.0f, 1.5f);
        pushLeg.stand = k;
        pushLeg.what = std::string("push back from ") + st.ref;
        if (pushLeg.plan.empty()) {
            continue;
        }
        Vec2 pushed = push.back();
        Place from;
        if (!net.snap(pushed, go, 15, from)) {
            continue;
        }
        List<Vec2> way;
        List<uint8_t> kinds;
        if (!taxiRoute(net, ap.layout, rng, from, go, hw.at, hw.arrive, rules, rng.chance(0.4f), way, kinds)) {
            continue;
        }
        way[0] = pushed;
        // Pushed back to face the way it then taxies, as a pushback leaves
        // one: its nose - the push's last way, reversed - along the route's
        // start, not a turn about.
        {
            float hp = 0, hw0 = 0;
            pushLeg.plan.pointAt(pushLeg.plan.length() - 0.5f, hp);
            Plan first;
            first.build(way, kinds, true);
            first.pointAt(std::min(20.0f, first.length()), hw0);
            if (cosf(hp + PI - hw0) < 0.2f) {
                continue;
            }
        }
        Leg out;
        out.kind = Leg::TAXI;
        out.plan.build(way, kinds, true);
        out.want = rng.range(7, 12);
        out.afterPush = true;
        out.what = std::string("taxi out to ") + hw.ref;
        giveWay(out, rng, 0.5f, 3);
        if (out.plan.empty()) {
            continue;
        }
        List<Vec2> up;
        List<uint8_t> upKinds;
        Place runway;
        float takeoff = 0;
        if (!lineUpPath(net, hold, up, upKinds, runway, takeoff)) {
            continue;
        }
        Leg lineUp;
        lineUp.kind = Leg::TAXI;
        lineUp.plan.build(up, upKinds, true);
        lineUp.want = 6;
        lineUp.lineUp = true;
        lineUp.what = "line up";
        Place lined;
        if (lineUp.plan.empty() || !net.snap(up.back(), takeoff, 10, lined, true)) {
            continue;
        }
        List<Vec2> roll;
        List<uint8_t> rollKinds;
        net.ahead(lined, takeoff, 3500, roll, rollKinds);
        // Straight down the runway: not on round a turning loop at its end.
        for (size_t i = 0; i + 1 < roll.size(); i++) {
            float b = atan2f(roll[i + 1].y - roll[i].y, roll[i + 1].x - roll[i].x);
            if (dist(roll[i], roll[i + 1]) > 1 && fabsf(wrap(b - takeoff)) > 30 * DEG) {
                roll.resize(i + 1);
                rollKinds.resize(i + 1);
                break;
            }
        }
        for (uint8_t &rk : rollKinds) {
            rk = (uint8_t)airfield::WayKind::RUNWAY;
        }
        Leg rollLeg;
        rollLeg.kind = Leg::ROLL;
        rollLeg.plan.build(roll, rollKinds, false);
        rollLeg.want = 1e9f;
        rollLeg.what = "take-off roll";
        if (rollLeg.plan.length() < 1300) {
            continue;
        }
        Leg wait;
        wait.kind = Leg::WAIT;
        wait.what = "parked";
        wait.seconds = run.kind == DEPART ? rng.range(20, 150) : 0;
        run.truth.legs.push_back(wait);
        run.truth.legs.push_back(std::move(pushLeg));
        wait.what = "starting engines";
        wait.seconds = rng.range(20, 120);
        run.truth.legs.push_back(wait);
        float outLength = out.plan.length();
        run.truth.legs.push_back(std::move(out));
        wait.what = "holding";
        wait.seconds = rng.range(0, 120);
        run.truth.legs.push_back(wait);
        run.truth.legs.push_back(std::move(lineUp));
        wait.what = "lined up";
        wait.seconds = rng.range(0, 50);
        run.truth.legs.push_back(wait);
        run.truth.legs.push_back(std::move(rollLeg));
        run.hold = hold;
        run.arrived = false;
        run.landedBegin = false;
        if (run.kind == DEPART_PICKUP) {
            // The view opened as it taxies out.
            float before = 0;
            for (size_t i = 0; i < 3; i++) {
                before += run.truth.legs[i].kind == Leg::WAIT ? run.truth.legs[i].seconds : 60;
            }
            run.beginAt = before + rng.range(10, std::max(20.0f, outLength / 9));
        }
        char story[120];
        snprintf(story, sizeof story, "parked at %s, pushed back, %.0fm out to hold %s, take-off", st.ref, outLength,
                 hw.ref);
        run.story = story;
        return true;
    }
    run.skip = "no departure from a stand found";
    return false;
}

// --- the run, and its checks ----------------------------------------------------------

struct Result {
    Glitches glitches;
    std::string end;  // what the follower ended up doing
    float seconds = 0;
    int frames = 0, reports = 0;
    float worstGap = 0;
    // For a picture.
    std::vector<Vec2> shownPath, truthPath;
    std::vector<std::pair<Vec2, float>> shownPoses;  // every few seconds: where, and its heading
    std::vector<Report> reports_;
};

void runOne(Run &run, Rng &rng, Result &res, bool trace) {
    const Airport &ap = *run.ap;
    const Network &net = ap.net;
    Follower f;
    Truth &truth = run.truth;
    Feed &feed = run.feed;
    Report last{};
    bool haveReport = false;
    // A report from before the start: landing, the roll just before; else where it is.
    feed.nextPoll = rng.range(0, feed.interval);
    {
        float age = rng.range(0.5f, feed.interval);
        last.t = -age;
        if (run.landedBegin) {
            float h = truth.taxi.heading(), v = 20.6f + 1.2f * age;
            last.at = aheadOf(truth.taxi.at(), h, -(20.6f + 0.6f * age) * age);
            last.heading = h;
            last.speed = roundf(v / KT) * KT;
        } else {
            last.at = aheadOf(truth.taxi.at(), truth.taxi.heading(), feed.antenna);
            last.at = {last.at.x + rng.normal() * feed.stillNoise, last.at.y + rng.normal() * feed.stillNoise};
            last.heading = rng.range(-PI, PI);
            last.speed = 0;
        }
        last.wild = false;
        haveReport = true;
    }
    bool takeoffSim = false, handedBack = false;
    Vec2 prev = {0, 0};
    float prevH = 0, prevV = 0;
    bool havePrev = false;
    float stalled = 0, farFor = 0, grassFor = 0, hesitating = 0, parkedFor = 0, truthDoneAt = -1;
    struct Slip {
        float t, back, side;
    };
    std::vector<Slip> slips;
    // For the loops: each metre gone forwards, the way it pointed.
    std::vector<std::pair<float, float>> turns;  // distance gone, heading
    float gone = 0;
    float t = 0, lastPose = -100;
    float tEnd = 3600;
    std::string lastDoing;
    int lastTraceSecond = -1;
    Vec2 simAt = {0, 0};
    float simHeading = 0, simSpeed = 0;
    while (t < tEnd) {
        float dt = run.frameDt * rng.range(0.85f, 1.2f);
        if (rng.chance(0.002f)) {
            dt = rng.range(0.2f, 0.5f);  // a long frame: a layout loaded, a poll parsed
        }
        t += dt;
        truth.step(t, dt);
        Report r;
        if (feed.make(t, truth, rng, r)) {
            last = r;
            haveReport = true;
            res.reports++;
            res.reports_.push_back(r);
            if (truth.onRunway && run.kind != ARRIVE && run.kind != ARRIVE_PICKUP) {
                takeoffSim = true;  // the runway simulation's, seeing it on the runway
            }
            if (trace) {
                printf("  %6.1f report (%.0f,%.0f) %3.0fkt track %4.0f%s\n", t, r.at.x, r.at.y, r.speed / KT,
                       isnan(r.heading) ? -1 : wrap(PI / 2 - r.heading) / DEG + (wrap(PI / 2 - r.heading) < 0 ? 360 : 0),
                       r.wild ? "  WILD" : "");
            }
        }
        if (truth.done && !takeoffSim && run.kind != ARRIVE_LOST) {  // never seen on the runway
            break;
        }
        if (run.kind == DEPART_LINED ? !takeoffSim : t < run.beginAt) {
            continue;  // depart-lined: the view showing others until it is on the runway
        }
        Sample now = truth.at(t);
        if (takeoffSim) {
            simAt = now.at;
            simHeading = now.heading;
            simSpeed = now.speed;
        }
        // The view: rolling for take-off, handed back to the runway
        // simulation - shown where that has it, or at 80 knots.
        float simKt = takeoffSim ? simSpeed / KT : 0;
        if (simKt > 40 && f.active() && (simKt > 80 || dist(f.at(), simAt) < 15)) {
            float lurch = dist(f.at(), simAt);
            if (lurch > 15) {
                res.glitches.add("lurch", lurch, t, f.at(), f.doing());
            }
            handedBack = true;
            res.end = std::string("handed back: ") + f.doing();
            if (trace) {
                printf("  %6.1f handed back to the take-off's simulation, %.0fm from where it has it\n", t, lurch);
            }
            break;
        }
        if (truth.airborne) {
            res.glitches.add("no-handback", dist(f.at(), now.at), t, f.at(), f.doing());
            res.end = std::string("airborne, never handed back: ") + f.doing();
            break;
        }
        Follower::Input in{};
        in.ms = msAt(t);
        in.dt = dt;
        if (haveReport) {
            in.reportMs = msAt(last.t);
            in.reportAt = last.at;
            in.reportHeading = last.heading;
            in.reportSpeed = last.speed;
        }
        in.takeoff = takeoffSim;
        in.simAt = simAt;
        in.simHeading = simHeading;
        in.simSpeed = simSpeed;
        in.standing = run.standing.data();
        in.standingCount = (int)run.standing.size();
        in.arrived = run.arrived;
#ifndef SOAK_BASELINE
        in.lost = feed.lost;  // the follower before these inputs were given it
        in.departFrom = run.departFrom.data();
        in.departFromCount = (int)run.departFrom.size();
        in.noseWheel = run.type.noseWheel;
        in.nose = run.type.nose;
#endif
        if (!f.active()) {
            // As the view begins it: where it shows it - landed, where the
            // runway simulation had it; else its last report, carried on
            // along its track.
            Vec2 at = last.at;
            float heading = isnan(last.heading) ? rng.range(-PI, PI) : last.heading;
            float speed = last.speed;
            if (run.landedBegin) {
                at = aheadOf(now.at, now.heading, rng.normal() * 12);
                heading = now.heading + rng.normal() * DEG;
                speed = std::min(now.speed, 20.6f);
            } else if (run.kind == DEPART_LINED) {
                // Where the take-off's simulation has it, lined up or rolling.
                at = now.at;
                heading = now.heading;
                speed = now.speed;
            } else if (last.speed >= 3.1f && !isnan(last.heading)) {
                at = aheadOf(last.at, last.heading, last.speed * std::min(8.0f, t - last.t));
            }
            f.begin(net, at, heading, speed, run.landedBegin, in);
            // Handed over from the landing's roll, it carries on from where
            // that showed it; opened on it, what the view showed before isn't
            // drawn - its first frame is the follower's.
            prev = f.at();
            prevH = f.heading();
            prevV = f.speed();
            havePrev = run.landedBegin;
        }
        f.update(net, in);
        res.frames++;
        Vec2 p = f.at();
        float h = f.heading(), v = f.speed();
        const char *doing = f.doing();
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(h) || !std::isfinite(v)) {
            res.glitches.add("nan", 1, t, prev, doing);
            res.end = "NaN";
            break;
        }
        bool pushing = strncmp(doing, "pushing", 7) == 0;
        if (havePrev) {
            float step = dist(prev, p);
            float allowed = std::max(v, prevV) * dt * 1.05f + 0.02f;
            if (step > allowed + 0.25f) {
                res.glitches.add("jump", step - allowed, t, p, doing);
            }
            float dh = fabsf(wrap(h - prevH));
            float vmax = std::max(v, prevV);
            // As fast as the taxi turns at its speed - or, parked, settling
            // straight onto its stand, slowly.
            float most = std::max(std::min(vmax / 5, 2.5f / std::max(vmax, 0.5f)), 5 * DEG) * dt;
            if (dh > most + 0.5f * DEG) {
                res.glitches.add("snap-turn", dh / DEG, t, p, doing);
            }
            if (step > 0.03f) {
                // Visible as it adds up: more than a metre backwards, or two
                // sideways, in three seconds - not a frame's wobble.
                float dir = atan2f(p.y - prev.y, p.x - prev.x);
                float off = fabsf(wrap(dir - (pushing ? h + PI : h)));
                slips.push_back({t, off > PI / 2 ? step : 0, off > 20 * DEG && off <= PI / 2 ? step * sinf(off) : 0});
                while (!slips.empty() && slips.front().t < t - 3) {
                    slips.erase(slips.begin());
                }
                float backSum = 0, sideSum = 0;
                for (const Slip &sl : slips) {
                    backSum += sl.back;
                    sideSum += sl.side;
                }
                if (backSum > 1) {
                    res.glitches.add("backwards", backSum, t, p, doing);
                }
                if (sideSum > 2) {
                    res.glitches.add("sideways", sideSum, t, p, doing);
                }
                if (!pushing && off < PI / 2) {
                    gone += step;
                    if (turns.empty() || gone - turns.back().first >= 1) {
                        turns.push_back({gone, h});
                        // Turned about in under 45m; round more than a circle in 250m.
                        float net45 = 0, total = 0;
                        for (size_t i = turns.size() - 1; i > 0; i--) {
                            total += wrap(turns[i].second - turns[i - 1].second);
                            if (gone - turns[i - 1].first > 250) {
                                break;
                            }
                            if (gone - turns[i - 1].first <= 45) {
                                net45 = std::max(net45, fabsf(wrap(h - turns[i - 1].second)));
                            }
                        }
                        if (net45 > 150 * DEG) {
                            res.glitches.add("tight-turn", net45 / DEG, t, p, doing);
                        }
                        if (fabsf(total) > 330 * DEG) {
                            res.glitches.add("loop", fabsf(total) / DEG, t, p, doing);
                            turns.clear();
                        }
                        if (turns.size() > 400) {
                            turns.erase(turns.begin(), turns.begin() + 100);
                        }
                    }
                }
            }
        }
        // Its wheels on the paved ground; its nose, tail and wingtips out of
        // the buildings.
        // Off the paved ground - its edges as the map has them - for more than
        // a second, as a wheel over the edge for a frame or two isn't seen.
        float off = ap.ground.offPaved(p);
        grassFor = off > 8 ? grassFor + dt : 0;
        if (grassFor > 1) {
            res.glitches.add("grass", off, t, p, doing);
        }
        Vec2 nose = aheadOf(p, h, run.type.nose), tail = aheadOf(p, h, run.type.nose - run.type.length);
        Vec2 side = {-sinf(h), cosf(h)};
        Vec2 wingRoot = aheadOf(p, h, -run.type.span * 0.15f);
        Vec2 left = {wingRoot.x + side.x * run.type.span / 2, wingRoot.y + side.y * run.type.span / 2};
        Vec2 right = {wingRoot.x - side.x * run.type.span / 2, wingRoot.y - side.y * run.type.span / 2};
        if (ap.ground.inBuilding(nose) || ap.ground.inBuilding(p) || ap.ground.inBuilding(tail)) {
            res.glitches.add("building", 1, t, p, doing);
        } else if (ap.ground.inBuilding(left) || ap.ground.inBuilding(right)) {
            res.glitches.add("wing-building", 1, t, p, doing);
        }
        // The same of where it really is: the map's own gaps, not the follower's.
        if (ap.ground.offPaved(now.at) > 8) {
            res.glitches.add("truth-grass", ap.ground.offPaved(now.at), t, now.at, "");
        }
        {
            Vec2 tn = aheadOf(now.at, now.heading, run.type.nose);
            Vec2 tt = aheadOf(now.at, now.heading, run.type.nose - run.type.length);
            if (ap.ground.inBuilding(tn) || ap.ground.inBuilding(now.at) || ap.ground.inBuilding(tt)) {
                res.glitches.add("truth-building", 1, t, now.at, "");
            }
        }
        // Stopped short, taxiing on by itself with its signal lost - waiting
        // for reports that won't come.
        if (feed.lost && t > run.lostAt && v < 0.05f && strncmp(doing, "taxiing", 7) == 0) {
            hesitating += dt;
            if (hesitating > 8) {
                res.glitches.add("hesitate", hesitating, t, p, doing);
            }
        } else {
            hesitating = 0;
        }
        // Stopped, its reports going on - fresh, moving - without it.
        bool fresh = haveReport && t - last.t < 15;
        if (fresh && last.speed >= 3.1f && v < 0.3f && dist(p, last.at) > 80) {
            stalled += dt;
            if (stalled > 20) {
                res.glitches.add("stall", stalled, t, p, doing);
            }
        } else {
            stalled = 0;
        }
        float gap = dist(p, now.at);
        res.worstGap = std::max(res.worstGap, gap);
        if (gap > 400 && t < run.lostAt) {
            farFor += dt;
            if (farFor > 30) {
                res.glitches.add("far", gap, t, p, doing);
            }
        } else {
            farFor = 0;
        }
        if (t - lastPose >= 4) {
            res.shownPoses.push_back({p, h});
            lastPose = t;
        }
        if (res.shownPath.empty() || dist(res.shownPath.back(), p) > 1) {
            res.shownPath.push_back(p);
        }
        if (res.truthPath.empty() || dist(res.truthPath.back(), now.at) > 1) {
            res.truthPath.push_back(now.at);
        }
        if (trace && ((int)t != lastTraceSecond || lastDoing != doing)) {
            lastTraceSecond = (int)t;
            const Leg *L = truth.current();
            printf("  %6.1f shown (%7.1f,%7.1f) h %4.0f %4.1fm/s %-30s | truth (%7.1f,%7.1f) h %4.0f %4.1fm/s %-18s gap %4.0f\n",
                   t, p.x, p.y, h / DEG, v, doing, now.at.x, now.at.y, now.heading / DEG, now.speed,
                   L ? L->what.c_str() : "done", gap);
        }
        lastDoing = doing;
        prev = p;
        prevH = h;
        prevV = v;
        havePrev = true;
        if (truth.done && truthDoneAt < 0) {
            truthDoneAt = t;
        }
        if (run.kind == ARRIVE_LOST) {
            parkedFor = strncmp(doing, "parked at stand", 15) == 0 ? parkedFor + dt : 0;
            if (parkedFor > 20 || (truthDoneAt >= 0 && t > truthDoneAt + 600)) {
                break;
            }
        } else if (truth.done && run.kind != DEPART && run.kind != DEPART_PICKUP && run.kind != DEPART_LINED) {
            break;
        }
    }
    res.seconds = t;
    if (res.end.empty()) {
        res.end = f.doing();
    }
    // Landed, its signal lost: parked - at a free stand, where and as an
    // aircraft parks there.
    if (run.kind == ARRIVE_LOST) {
        int at = -1;
        float best = 1e30f, heading = 0;
        for (int k : ap.stands) {
            const StandWay &st = net.stands()[k];
            float h = st.heading;
            float d = dist(parkedAt(st, parkBack(st, run.type.noseWheel, run.type.nose), h), f.at());
            if (d < best) {
                best = d;
                at = k;
                heading = h;
            }
        }
        if (strncmp(f.doing(), "parked at stand", 15) != 0 || at < 0) {
            res.glitches.add("not-parked", 0, t, f.at(), f.doing());
        } else {
            bool taken = false;
            for (const Vec2 &o : run.standing) {
                taken = taken || dist(o, net.stands()[at].line.back()) < 25;
            }
            if (taken) {
                res.glitches.add("taken-stand", 1, t, f.at(), f.doing());
            }
            float turned = fabsf(wrap(f.heading() - heading)) / DEG;
            if (best > 1 || turned > 2) {
                res.glitches.add("parked-off", std::max(best, turned / 2), t, f.at(), f.doing());
            }
        }
    }
    // Landed: parked at its stand, where and as an aircraft parks there.
    if (run.kind == ARRIVE || run.kind == ARRIVE_PICKUP || run.kind == PARKED) {
        const StandWay &st = net.stands()[run.stand];
        std::string want = std::string("parked at stand ") + st.ref;
        float heading = st.heading;
        Vec2 park = parkedAt(st, parkBack(st, run.type.noseWheel, run.type.nose), heading);
        if (strncmp(f.doing(), "parked at stand", 15) != 0) {
            res.glitches.add("not-parked", dist(f.at(), park), t, f.at(), f.doing());
        } else if (want != f.doing()) {
            res.glitches.add("wrong-stand", dist(f.at(), park), t, f.at(), f.doing());
        } else {
            float off = dist(f.at(), park), turned = fabsf(wrap(f.heading() - heading)) / DEG;
            if (off > 1 || turned > 2) {
                res.glitches.add("parked-off", std::max(off, turned / 2), t, f.at(), f.doing());
                char why[64];
                snprintf(why, sizeof why, "parked %.1fm, %.1f deg off", off, turned);
                res.glitches.kinds["parked-off"].doing = why;
            }
        }
    } else if (!handedBack && (run.kind == DEPART || (run.kind == DEPART_LINED && takeoffSim))) {
        res.glitches.add("no-handback", 0, t, f.at(), f.doing());
    }
}

// --- a picture of a run ---------------------------------------------------------------

void svg(const char *path, const Run &run, const Result &res, Vec2 middle, float half) {
    FILE *fp = fopen(path, "w");
    if (fp == nullptr) {
        return;
    }
    const Airport &ap = *run.ap;
    const float SIZE = 1400, scale = SIZE / (2 * half);
    float left = middle.x - half, top = middle.y + half;
    auto X = [&](float x) { return (x - left) * scale; };
    auto Y = [&](float y) { return (top - y) * scale; };
    auto inView = [&](Vec2 p, float margin) {
        return p.x > left - margin && p.x < left + 2 * half + margin && p.y < top + margin && p.y > top - 2 * half - margin;
    };
    fprintf(fp, "<svg xmlns='http://www.w3.org/2000/svg' width='%.0f' height='%.0f' viewBox='0 0 %.0f %.0f' "
                "font-family='sans-serif'>\n<rect width='100%%' height='100%%' fill='#d4e4c4'/>\n",
            SIZE, SIZE, SIZE, SIZE);
    auto tris = [&](const std::vector<Ground::Tri> &all, const char *fill) {
        for (const Ground::Tri &t : all) {
            if (!inView(t.a, 300) && !inView(t.b, 300) && !inView(t.c, 300)) {
                continue;
            }
            fprintf(fp, "<polygon points='%.1f,%.1f %.1f,%.1f %.1f,%.1f' fill='%s' stroke='%s' stroke-width='0.5'/>\n",
                    X(t.a.x), Y(t.a.y), X(t.b.x), Y(t.b.y), X(t.c.x), Y(t.c.y), fill, fill);
        }
    };
    tris(ap.ground.surfaces, "#bdbdbd");
    for (const airfield::Way &w : ap.layout.ways) {
        float width = w.width > 1 ? w.width : w.kind == airfield::WayKind::RUNWAY ? 45 : 18;
        fprintf(fp, "<polyline fill='none' stroke='%s' stroke-width='%.1f' stroke-linejoin='round' points='",
                w.kind == airfield::WayKind::RUNWAY ? "#8a8a8a" : "#a9a9a9", width * scale);
        fprintf(fp, "%.1f,%.1f ", X(ap.layout.junctions[w.from].x), Y(ap.layout.junctions[w.from].y));
        for (const airfield::Point &q : w.between) {
            fprintf(fp, "%.1f,%.1f ", X(q.x), Y(q.y));
        }
        fprintf(fp, "%.1f,%.1f'/>\n", X(ap.layout.junctions[w.to].x), Y(ap.layout.junctions[w.to].y));
    }
    for (const airfield::Way &w : ap.layout.ways) {
        fprintf(fp, "<polyline fill='none' stroke='#f0e68c' stroke-width='1' points='");
        fprintf(fp, "%.1f,%.1f ", X(ap.layout.junctions[w.from].x), Y(ap.layout.junctions[w.from].y));
        for (const airfield::Point &q : w.between) {
            fprintf(fp, "%.1f,%.1f ", X(q.x), Y(q.y));
        }
        fprintf(fp, "%.1f,%.1f'/>\n", X(ap.layout.junctions[w.to].x), Y(ap.layout.junctions[w.to].y));
    }
    tris(ap.ground.buildings, "#7d6250");
    for (size_t k = 0; k < ap.net.stands().size(); k++) {
        const StandWay &st = ap.net.stands()[k];
        if (!st.ok || !inView(st.line.back(), 100)) {
            continue;
        }
        fprintf(fp, "<polyline fill='none' stroke='#d4a017' stroke-width='1.2' points='");
        for (const Vec2 &q : st.line) {
            fprintf(fp, "%.1f,%.1f ", X(q.x), Y(q.y));
        }
        fprintf(fp, "'/>\n<text x='%.1f' y='%.1f' font-size='9' fill='#6b4f00'>%s</text>\n", X(st.line.back().x) + 2,
                Y(st.line.back().y) - 2, st.ref);
    }
    for (const Vec2 &s : run.standing) {
        if (inView(s, 50)) {
            fprintf(fp, "<circle cx='%.1f' cy='%.1f' r='4' fill='#555' opacity='0.6'/>\n", X(s.x), Y(s.y));
        }
    }
    auto line = [&](const std::vector<Vec2> &pts, const char *colour, float width) {
        fprintf(fp, "<polyline fill='none' stroke='%s' stroke-width='%.1f' opacity='0.85' points='", colour, width);
        for (const Vec2 &q : pts) {
            fprintf(fp, "%.1f,%.1f ", X(q.x), Y(q.y));
        }
        fprintf(fp, "'/>\n");
    };
    line(res.truthPath, "#1f5fd6", 2.5f);
    line(res.shownPath, "#ff7f0e", 2);
    for (const std::pair<Vec2, float> &pose : res.shownPoses) {
        if (!inView(pose.first, 50)) {
            continue;
        }
        Vec2 p = pose.first;
        float h = pose.second;
        Vec2 nose = aheadOf(p, h, run.type.nose), tail = aheadOf(p, h, run.type.nose - run.type.length);
        Vec2 side = {-sinf(h) * run.type.span / 2, cosf(h) * run.type.span / 2};
        Vec2 root = aheadOf(p, h, -run.type.span * 0.15f);
        fprintf(fp, "<polyline fill='none' stroke='#c45c00' stroke-width='1' points='%.1f,%.1f %.1f,%.1f'/>\n", X(nose.x),
                Y(nose.y), X(tail.x), Y(tail.y));
        fprintf(fp, "<polyline fill='none' stroke='#c45c00' stroke-width='1' points='%.1f,%.1f %.1f,%.1f'/>\n",
                X(root.x + side.x), Y(root.y + side.y), X(root.x - side.x), Y(root.y - side.y));
    }
    int n = 0;
    for (const Report &r : res.reports_) {
        n++;
        if (!inView(r.at, 20)) {
            continue;
        }
        fprintf(fp, "<circle cx='%.1f' cy='%.1f' r='3' fill='%s'/>\n<text x='%.1f' y='%.1f' font-size='8' fill='#900'>%d</text>\n",
                X(r.at.x), Y(r.at.y), r.wild ? "#ff00ff" : "#d62728", X(r.at.x) + 3, Y(r.at.y) + 8, n);
    }
    int row = 0;
    for (const auto &k : res.glitches.kinds) {
        const Worst &w = k.second;
        fprintf(fp, "<circle cx='%.1f' cy='%.1f' r='9' fill='none' stroke='#e600e6' stroke-width='2.5'/>\n"
                    "<text x='%.1f' y='%.1f' font-size='12' fill='#a000a0'>%s</text>\n",
                X(w.at.x), Y(w.at.y), X(w.at.x) + 11, Y(w.at.y) - 4, k.first.c_str());
        fprintf(fp, "<text x='10' y='%d' font-size='13' fill='#000'>%s %.1f x%d at %.0fs (%s)</text>\n", 40 + 16 * row++,
                k.first.c_str(), w.size, w.count, w.t, w.doing.c_str());
    }
    fprintf(fp, "<text x='10' y='20' font-size='14' fill='#000'>%s %s #%d %s: %s</text>\n", ap.code.c_str(),
            KIND_NAMES[run.kind], run.number, run.type.code, run.story.c_str());
    fprintf(fp, "</svg>\n");
    fclose(fp);
}

bool loadAirport(const Pack &pack, const char *code, Airport &ap) {
    ap.code = code;
    if (!pack.layout(code, ap.layout)) {
        return false;
    }
    ap.net.build(ap.layout);
    if (ap.net.empty()) {
        return false;
    }
    ap.ground.build(ap.layout, ap.net);
    for (size_t w = 0; w < ap.layout.ways.size(); w++) {
        if (ap.layout.ways[w].kind == airfield::WayKind::RUNWAY) {
            ap.runwayWays.push_back((int)w);
        }
    }
    for (size_t k = 0; k < ap.net.holds().size(); k++) {
        const HoldWay &h = ap.net.holds()[k];
        if (h.ok && h.runway && h.end) {
            ap.endHolds.push_back((int)k);
        }
    }
    for (size_t k = 0; k < ap.net.stands().size(); k++) {
        const StandWay &st = ap.net.stands()[k];
        if (st.ok) {
            ap.stands.push_back((int)k);
            if (st.terminal) {
                ap.terminalStands.push_back((int)k);
            }
        }
    }
    return true;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s airfields.bin CODE... [--runs N] [--run N] [--seed N] [--kinds a,b] [--trace] "
                        "[--svg DIR] [--all] [--quiet]\n",
                argv[0]);
        return 2;
    }
    Pack pack;
    if (!pack.open(argv[1])) {
        fprintf(stderr, "can't read %s\n", argv[1]);
        return 2;
    }
    std::vector<std::string> codes;
    int runs = 50, only = -1;
    uint64_t seed = 1;
    bool trace = false, quiet = false, listAll = false;
    const char *svgDir = nullptr;
    bool kinds[KINDS] = {true, true, true, false, false, false, false};
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--runs" && i + 1 < argc) {
            runs = atoi(argv[++i]);
        } else if (a == "--run" && i + 1 < argc) {
            only = atoi(argv[++i]);
        } else if (a == "--seed" && i + 1 < argc) {
            seed = strtoull(argv[++i], nullptr, 10);
        } else if (a == "--trace") {
            trace = true;
        } else if (a == "--quiet") {
            quiet = true;
        } else if (a == "--list") {
            listAll = true;  // every run's line, clean ones too
        } else if (a == "--svg" && i + 1 < argc) {
            svgDir = argv[++i];
        } else if (a == "--kinds" && i + 1 < argc) {
            std::string list = argv[++i];
            for (int k = 0; k < KINDS; k++) {
                kinds[k] = list == "all" || ("," + list + ",").find(std::string(",") + KIND_NAMES[k] + ",") !=
                                                 std::string::npos;
            }
        } else if (a == "--all") {
            for (const Pack::Row &r : pack.rows) {
                codes.push_back(r.code);
            }
        } else {
            codes.push_back(a);
        }
    }
    std::vector<Kind> kindList;
    for (int k = 0; k < KINDS; k++) {
        if (kinds[k]) {
            kindList.push_back((Kind)k);
        }
    }
    std::vector<Type> types = allTypes();
    // Over every airport: each kind of glitch, how many runs had it.
    std::map<std::string, int> runsWith;
    std::map<std::string, std::pair<float, std::string>> worstOf;
    int made = 0, clean = 0, skipped = 0;
    for (const std::string &code : codes) {
        Airport ap;
        if (!loadAirport(pack, code.c_str(), ap)) {
            if (!quiet) {
                printf("%s: no layout\n", code.c_str());
            }
            continue;
        }
        int apMade = 0, apClean = 0;
        std::map<std::string, int> apWith;
        for (int n = 0; n < runs; n++) {
            if (only >= 0 && n != only) {
                continue;
            }
            uint64_t s = seed * 1000003ull + std::hash<std::string>()(code) * 31ull + (uint64_t)n;
            Rng rng(s);
            Run run;
            run.ap = &ap;
            run.number = n;
            run.kind = kindList[n % kindList.size()];
            if (!setUp(run, rng, types)) {
                skipped++;
                if (trace) {
                    printf("%s #%d %s: skipped, %s\n", code.c_str(), n, KIND_NAMES[run.kind], run.skip.c_str());
                }
                continue;
            }
            if (trace) {
                printf("%s %s #%d %s: %s; polls %.0fs, %.0fm out (%.0fm still), aerial %.0fm ahead, %.0f%% wild, "
                       "%.0f%% missed, up to %.0fs late; %zu others parked; frames %.0fms\n",
                       code.c_str(), KIND_NAMES[run.kind], n, run.type.code, run.story.c_str(), run.feed.interval,
                       run.feed.noise, run.feed.stillNoise, run.feed.antenna, run.feed.wild * 100,
                       run.feed.drop * 100, run.feed.late, run.standing.size(), run.frameDt * 1000);
            }
            Result res;
            runOne(run, rng, res, trace);
            made++;
            apMade++;
            if (res.glitches.kinds.empty()) {
                clean++;
                apClean++;
            }
            std::string line;
            for (const auto &k : res.glitches.kinds) {
                runsWith[k.first]++;
                apWith[k.first]++;
                char part[200];
                snprintf(part, sizeof part, "  %s %.1f x%d at %.0fs (%s)", k.first.c_str(), k.second.size, k.second.count,
                         k.second.t, k.second.doing.c_str());
                line += part;
                std::pair<float, std::string> &w = worstOf[k.first];
                if (k.second.size > w.first) {
                    char where[64];
                    snprintf(where, sizeof where, "%s --run %d (%s)", code.c_str(), n, KIND_NAMES[run.kind]);
                    w = {k.second.size, where};
                }
            }
            if (!res.glitches.kinds.empty() || trace || listAll) {
                printf("%s %-13s #%-4d %-4s poll %2.0fs: %s -> %s%s\n", code.c_str(), KIND_NAMES[run.kind], n,
                       run.type.code, run.feed.interval, run.story.c_str(), res.end.c_str(), line.c_str());
            }
            if (svgDir != nullptr && (!res.glitches.kinds.empty() || only >= 0)) {
                char path[512];
                float x0 = 1e30f, y0 = 1e30f, x1 = -1e30f, y1 = -1e30f;
                for (const std::vector<Vec2> *pts : {&res.truthPath, &res.shownPath}) {
                    for (const Vec2 &q : *pts) {
                        x0 = std::min(x0, q.x), y0 = std::min(y0, q.y), x1 = std::max(x1, q.x), y1 = std::max(y1, q.y);
                    }
                }
                if (x0 < x1) {
                    snprintf(path, sizeof path, "%s/%s-%d.svg", svgDir, code.c_str(), n);
                    svg(path, run, res, {(x0 + x1) / 2, (y0 + y1) / 2}, std::max(x1 - x0, y1 - y0) / 2 + 80);
                }
                // And close up on the worst of what looked wrong.
                float worst = -1;
                Vec2 at = {0, 0};
                for (const auto &k : res.glitches.kinds) {
                    if (k.second.size > worst) {
                        worst = k.second.size;
                        at = k.second.at;
                    }
                }
                if (worst >= 0) {
                    snprintf(path, sizeof path, "%s/%s-%d-close.svg", svgDir, code.c_str(), n);
                    svg(path, run, res, at, 120);
                }
            }
        }
        if (!quiet && only < 0) {
            printf("%s: %d runs, %d clean", code.c_str(), apMade, apClean);
            for (const auto &k : apWith) {
                printf(", %s %d", k.first.c_str(), k.second);
            }
            printf("\n");
        }
    }
    printf("\n%d runs, %d clean (%.0f%%), %d couldn't be made\n", made, clean, made ? 100.0 * clean / made : 0, skipped);
    for (const auto &k : runsWith) {
        printf("  %-14s %4d runs  worst %.1f: %s\n", k.first.c_str(), k.second, worstOf[k.first].first,
               worstOf[k.first].second.c_str());
    }
    return 0;
}
