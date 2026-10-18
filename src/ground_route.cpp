#include "ground_route.h"

#include <math.h>
#include <string.h>

#include <algorithm>
#include <functional>

namespace ground {
namespace {

constexpr float GRID = 100;  // metres, snap()'s squares
constexpr float BACK = 1000;  // metres, what leaving backwards counts as - see search()
constexpr float INF = 1e30f;
// Turning at a junction: a right angle counting as this many metres further,
// a sharper turn more; one sharper than 120 degrees - cos -0.5 - as much as
// SHARP_COST more, taken slowly where the way round is far longer, as some
// exits are angled for the runway's other end; none sharper than 150.
constexpr float COS_MOST_TURN = -0.866f, COS_SHARP_TURN = -0.5f;
constexpr float TURN_COST = 25, SHARP_COST = 400;
// Arriving at a place going within this of the way wanted - cos 110 degrees:
// a stand's line in leaves its taxilane at a right angle or so, either way,
// but not back the way the aircraft came.
constexpr float COS_MOST_ARRIVE = -0.342f;
// The way an edge leaves a junction, or arrives: over this much of it, so a
// node a metre from the end doesn't decide it.
constexpr float END_SPAN = 12;
// Off a runway: a fast exit's shallow turn - under 40 degrees - at up to
// 20m/s, 39 knots; one under 65 at 13; a right angle at 8, 15 knots; one
// sharper still at 3.
// A stand whose stop is within this of a terminal building is the terminal's.
constexpr float AT_TERMINAL = 150;
// An aircraft parked on a stand keeps its nose this far from the building
// ahead; and the building is looked for this far ahead of the stop.
constexpr float PARK_CLEAR = 3, ROOM_REACH = 80;
constexpr float EXIT_FAST_COS = 0.766f, EXIT_MID_COS = 0.423f;
constexpr float EXIT_FAST = 20, EXIT_MID = 13, EXIT_SLOW = 8, EXIT_SHARP = 3;

float dist(Vec2 a, Vec2 b) { return hypotf(b.x - a.x, b.y - a.y); }

Vec2 unit(Vec2 from, Vec2 to) {
    float d = dist(from, to);
    return d < 1e-4f ? Vec2{1, 0} : Vec2{(to.x - from.x) / d, (to.y - from.y) / d};
}

// How much two headings differ, either way along a line: 0 to pi/2 - a taxiway
// is used in both directions.
float across(float a, float b) { return fabsf(remainderf(a - b, (float)M_PI)); }

float headingOf(Vec2 v) { return atan2f(v.y, v.x); }

// The point `d` along a line, from its start.
Vec2 pointAlong(const airfield::List<Vec2> &line, float d) {
    float h;
    return along(line, d, h);
}

}  // namespace

void Network::build(const airfield::Layout &layout) {
    edges_.clear();
    junctions_.clear();
    meets_.clear();
    stands_.clear();
    holds_.clear();
    enter_.clear();
    for (const airfield::Point &p : layout.junctions) {
        junctions_.push_back({p.x, p.y});
    }
    meets_.resize(junctions_.size());
    airfield::List<int32_t> edgeOfWay(layout.ways.size(), -1);
    for (size_t w = 0; w < layout.ways.size(); w++) {
        const airfield::Way &way = layout.ways[w];
        Edge e;
        e.from = way.from;
        e.to = way.to;
        e.kind = way.kind;
        e.points.push_back(junctions_[way.from]);
        for (const airfield::Point &p : way.between) {
            e.points.push_back({p.x, p.y});
        }
        e.points.push_back(junctions_[way.to]);
        float total = 0;
        e.lengths.push_back(0);
        for (size_t i = 1; i < e.points.size(); i++) {
            total += dist(e.points[i - 1], e.points[i]);
            e.lengths.push_back(total);
        }
        if (total < 0.5f || edges_.size() >= 0x7FFF) {
            continue;
        }
        e.afterFrom = pointAlong(e.points, std::min(END_SPAN, total));
        e.beforeTo = pointAlong(e.points, std::max(0.0f, total - END_SPAN));
        e.out = unit(e.points.front(), e.afterFrom);
        e.in = unit(e.beforeTo, e.points.back());
        uint16_t index = (uint16_t)edges_.size();
        edgeOfWay[w] = index;
        meets_[e.from].push_back(index);
        if (e.to != e.from) {
            meets_[e.to].push_back(index);
        }
        edges_.push_back(std::move(e));
    }
    // The squares: over all the edges' points, each segment in every square
    // its box covers - counted, then placed.
    float x0 = 1e30f, y0 = 1e30f, x1 = -1e30f, y1 = -1e30f;
    for (const Edge &e : edges_) {
        for (const Vec2 &q : e.points) {
            x0 = std::min(x0, q.x), y0 = std::min(y0, q.y), x1 = std::max(x1, q.x), y1 = std::max(y1, q.y);
        }
    }
    first_.clear();
    segments_.clear();
    if (edges_.empty()) {
        gridW_ = gridH_ = 0;
        return;
    }
    gridX_ = x0, gridY_ = y0;
    gridW_ = (int)((x1 - x0) / GRID) + 1, gridH_ = (int)((y1 - y0) / GRID) + 1;
    first_.assign((size_t)gridW_ * gridH_ + 1, 0);
    for (int pass = 0; pass < 2; pass++) {
        for (size_t k = 0; k < edges_.size(); k++) {
            const Edge &e = edges_[k];
            for (size_t i = 0; i + 1 < e.points.size(); i++) {
                Vec2 a = e.points[i], b = e.points[i + 1];
                int cx0 = (int)((std::min(a.x, b.x) - gridX_) / GRID),
                    cx1 = (int)((std::max(a.x, b.x) - gridX_) / GRID);
                int cy0 = (int)((std::min(a.y, b.y) - gridY_) / GRID),
                    cy1 = (int)((std::max(a.y, b.y) - gridY_) / GRID);
                for (int cy = cy0; cy <= cy1; cy++) {
                    for (int cx = cx0; cx <= cx1; cx++) {
                        size_t cell = (size_t)cy * gridW_ + cx;
                        if (pass == 0) {
                            first_[cell + 1]++;
                        } else {
                            segments_[first_[cell]++] = (uint32_t)k << 16 | (uint32_t)i;
                        }
                    }
                }
            }
        }
        if (pass == 0) {
            for (size_t c = 1; c < first_.size(); c++) {
                first_[c] += first_[c - 1];
            }
            segments_.resize(first_.back());
        } else {
            // Placing moved each square's start on to the next's: back.
            for (size_t c = first_.size() - 1; c > 0; c--) {
                first_[c] = first_[c - 1];
            }
            first_[0] = 0;
        }
    }

    // The stands: each line in from the end of it nearer the taxilanes -
    // the lines aren't all drawn the same way round - to the other, where it
    // stops; one without a line, straight in from the nearest taxilane. Some
    // are drawn as their outline - a closed shape - besides, or instead of, a
    // line: an outline, as its middle, but none where the stand has a line.
    auto outline = [](const airfield::Stand &st) {
        return st.lead.size() >= 4 && hypotf(st.lead.front().x - st.lead.back().x,
                                             st.lead.front().y - st.lead.back().y) < 5;
    };
    // The terminals' boxes, AT_TERMINAL wider all round, for a stand to be
    // tested against only those it is inside of.
    struct Box {
        float x0, y0, x1, y1;
        const airfield::Area *area;
    };
    airfield::List<Box> terminals;
    for (const airfield::Area &b : layout.buildings) {
        if (b.kind != 1 || b.corners.empty()) {
            continue;
        }
        Box box = {1e30f, 1e30f, -1e30f, -1e30f, &b};
        for (const airfield::Point &c : b.corners) {
            box.x0 = std::min(box.x0, c.x), box.y0 = std::min(box.y0, c.y);
            box.x1 = std::max(box.x1, c.x), box.y1 = std::max(box.y1, c.y);
        }
        box.x0 -= AT_TERMINAL, box.y0 -= AT_TERMINAL, box.x1 += AT_TERMINAL, box.y1 += AT_TERMINAL;
        terminals.push_back(box);
    }
    for (const airfield::Stand &st : layout.stands) {
        StandWay w;
        airfield::List<Vec2> line;
        if (outline(st)) {
            bool lined = false;
            for (const airfield::Stand &other : layout.stands) {
                lined = lined || (other.lead.size() >= 2 && !outline(other) && strncmp(other.ref, st.ref, 8) == 0);
            }
            if (lined) {
                stands_.push_back(std::move(w));  // not ok: its line is the stand
                continue;
            }
            Vec2 middle = {0, 0};
            for (size_t i = 0; i + 1 < st.lead.size(); i++) {
                middle.x += st.lead[i].x / (st.lead.size() - 1);
                middle.y += st.lead[i].y / (st.lead.size() - 1);
            }
            line.push_back(middle);
        } else {
            for (const airfield::Point &p : st.lead) {
                line.push_back({p.x, p.y});
            }
        }
        if (line.size() < 2 && !outline(st)) {
            line.clear();
            line.push_back({st.at.x, st.at.y});
        }
        Place front, back;
        bool atFront = snap(line.front(), NAN, 60, front);
        bool atBack = line.size() >= 2 && snap(line.back(), NAN, 60, back);
        if (atBack && (!atFront || dist(line.back(), at(back)) + 3 < dist(line.front(), at(front)))) {
            std::reverse(line.begin(), line.end());
            front = back;
            atFront = true;
        }
        memcpy(w.ref, st.ref, sizeof w.ref);
        if (atFront) {
            w.entry = front;
            w.line.push_back(at(front));
            for (const Vec2 &p : line) {
                if (dist(p, w.line.back()) > 0.5f) {
                    w.line.push_back(p);
                }
            }
            float total = lengthOf(w.line);
            if (total >= 5) {
                w.ok = true;
                w.arrive = headingOf(unit(w.line.front(), pointAlong(w.line, std::min(8.0f, total))));
                w.heading = headingOf(unit(pointAlong(w.line, std::max(0.0f, total - 5)), w.line.back()));
                w.middle = outline(st);
                w.room = layout.roomAhead({w.line.back().x, w.line.back().y}, w.heading, ROOM_REACH);
            }
        }
        if (w.ok) {
            Vec2 stop = w.line.back();
            for (const Box &box : terminals) {
                if (w.terminal || stop.x < box.x0 || stop.x > box.x1 || stop.y < box.y0 || stop.y > box.y1) {
                    continue;
                }
                const airfield::Area &b = *box.area;
                for (size_t i = 0; i < b.corners.size(); i++) {
                    const airfield::Point &a = b.corners[i], &c = b.corners[(i + 1) % b.corners.size()];
                    float dx = c.x - a.x, dy = c.y - a.y, len2 = dx * dx + dy * dy;
                    float t = len2 < 1e-4f ? 0
                                           : std::max(0.0f, std::min(1.0f, ((stop.x - a.x) * dx + (stop.y - a.y) * dy) / len2));
                    if (hypotf(a.x + dx * t - stop.x, a.y + dy * t - stop.y) < AT_TERMINAL) {
                        w.terminal = true;
                        break;
                    }
                }
            }
        }
        stands_.push_back(std::move(w));
    }
    // The runways' ends: where a runway's edge meets no other runway's.
    airfield::List<Vec2> ends;
    for (size_t j = 0; j < meets_.size(); j++) {
        int runways = 0;
        for (uint16_t k : meets_[j]) {
            runways += edges_[k].kind == airfield::WayKind::RUNWAY;
        }
        if (runways == 1) {
            ends.push_back(junctions_[j]);
        }
    }
    // The holding points, on their ways - a runway's facing the runway.
    for (const airfield::Hold &h : layout.holds) {
        HoldWay w;
        memcpy(w.ref, h.ref, sizeof w.ref);
        Vec2 p = {h.at.x, h.at.y};
        int32_t k = h.way < edgeOfWay.size() ? edgeOfWay[h.way] : -1;
        Place place;
        bool found = false;
        if (k >= 0) {
            // The point of its own edge nearest.
            const Edge &e = edges_[k];
            float best = INF;
            for (size_t i = 0; i + 1 < e.points.size(); i++) {
                Vec2 a = e.points[i], b = e.points[i + 1];
                float dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
                float t = len2 < 1e-4f ? 0
                                       : std::max(0.0f, std::min(1.0f, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
                float d = hypotf(a.x + dx * t - p.x, a.y + dy * t - p.y);
                if (d < best) {
                    best = d;
                    place = {(uint16_t)k, e.lengths[i] + sqrtf(len2) * t};
                }
            }
            found = best < 20;
        }
        if (!found) {
            found = snap(p, NAN, 20, place);
        }
        if (found) {
            w.ok = true;
            w.at = place;
            w.runway = h.kind == airfield::HoldKind::RUNWAY;
            Vec2 q;
            if (w.runway && nearestRunway(at(place), 400, q) && dist(at(place), q) > 5) {
                w.arrive = headingOf(unit(at(place), q));
                for (const Vec2 &e : ends) {
                    w.end = w.end || dist(e, q) < 500;
                }
            }
        }
        holds_.push_back(w);
    }
}

bool Network::snap(Vec2 p, float heading, float reach, Place &out, bool runwaysOnly) const {
    float best = 1e30f;
    bool found = false;
    if (gridW_ == 0) {
        return false;
    }
    int cx0 = std::max(0, (int)floorf((p.x - reach - gridX_) / GRID));
    int cx1 = std::min(gridW_ - 1, (int)floorf((p.x + reach - gridX_) / GRID));
    int cy0 = std::max(0, (int)floorf((p.y - reach - gridY_) / GRID));
    int cy1 = std::min(gridH_ - 1, (int)floorf((p.y + reach - gridY_) / GRID));
    for (int cy = cy0; cy <= cy1; cy++) {
        for (int cx = cx0; cx <= cx1; cx++) {
            size_t cell = (size_t)cy * gridW_ + cx;
            for (uint32_t n = first_[cell]; n < first_[cell + 1]; n++) {
                size_t k = segments_[n] >> 16, i = segments_[n] & 0xFFFF;
                const Edge &e = edges_[k];
                if (runwaysOnly && e.kind != airfield::WayKind::RUNWAY) {
                    continue;
                }
                Vec2 a = e.points[i], b = e.points[i + 1];
                float dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
                if (len2 < 1e-4f) {
                    continue;
                }
                float t = std::max(0.0f, std::min(1.0f, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
                Vec2 q = {a.x + dx * t, a.y + dy * t};
                float d = dist(p, q);
                if (d > reach) {
                    continue;
                }
                // A line running across the way it is heading counts as further
                // off than one running with it: 40m at right angles.
                float score = d + (isnan(heading) ? 0 : 40 * across(heading, atan2f(dy, dx)) / (float)(M_PI / 2));
                if (score < best) {
                    best = score;
                    out.edge = (uint16_t)k;
                    out.s = e.lengths[i] + sqrtf(len2) * t;
                    found = true;
                }
            }
        }
    }
    return found;
}

bool Network::nearestRunway(Vec2 p, float reach, Vec2 &q) const {
    float best = reach;
    bool found = false;
    for (const Edge &e : edges_) {
        if (e.kind != airfield::WayKind::RUNWAY) {
            continue;
        }
        for (size_t i = 0; i + 1 < e.points.size(); i++) {
            Vec2 a = e.points[i], b = e.points[i + 1];
            float dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
            float t = len2 < 1e-4f ? 0 : std::max(0.0f, std::min(1.0f, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
            Vec2 c = {a.x + dx * t, a.y + dy * t};
            float d = dist(p, c);
            if (d < best) {
                best = d;
                q = c;
                found = true;
            }
        }
    }
    return found;
}

Vec2 Network::at(const Place &p) const {
    float h;
    return along(edges_[p.edge].points, p.s, h);
}

float Network::heading(const Place &p) const {
    float h = 0;
    along(edges_[p.edge].points, p.s, h);
    return h;
}

bool Network::onRunway(const Place &p) const { return edges_[p.edge].kind == airfield::WayKind::RUNWAY; }

void Network::search(const Place &from, float heading, float most, const Rules &rules) const {
    size_t n = edges_.size() * 2;
    enter_.assign(n, INF);
    arrive_.assign(n, INF);
    before_.assign(n, -1);
    started_.assign(n, 0);
    tail_.assign(n, Vec2{0, 0});
    heap_.clear();
    from_ = from;
    rules_ = rules;
    if (edges_.empty()) {
        return;
    }
    // Which way it faces along its edge: the other, a kilometre more.
    startCost_[0] = startCost_[1] = 0;
    if (!isnan(heading)) {
        bool forwards = cosf(this->heading(from) - heading) >= 0;
        startCost_[forwards ? 1 : 0] = BACK;
    }
    auto push = [&](float cost, int32_t state) {
        heap_.push_back({cost, state});
        std::push_heap(heap_.begin(), heap_.end(), std::greater<std::pair<float, int32_t>>());
    };
    if (!rules.back && !isnan(heading)) {
        startCost_[startCost_[0] > 0 ? 0 : 1] = INF;
    }
    for (int back = 0; back < 2; back++) {
        int32_t state = 2 * from.edge + back;
        if (startCost_[back] >= INF) {
            continue;
        }
        arrive_[state] = startCost_[back] + (back ? from.s : length(from.edge) - from.s);
        started_[state] = 1;
        tail_[state] = back ? edges_[from.edge].afterFrom : edges_[from.edge].beforeTo;
        push(arrive_[state], state);
    }
    while (!heap_.empty()) {
        std::pop_heap(heap_.begin(), heap_.end(), std::greater<std::pair<float, int32_t>>());
        std::pair<float, int32_t> top = heap_.back();
        heap_.pop_back();
        float c = top.first;
        int32_t u = top.second;
        if (c > arrive_[u]) {
            continue;  // since bettered
        }
        if (c > most) {
            break;
        }
        const Edge &e = edges_[u / 2];
        bool back = u & 1;
        uint16_t j = back ? e.from : e.to;
        // The way it arrives: over the last 12m of the way there, so that two
        // turns either end of a stub of taxiway are seen for the one they are.
        Vec2 in = unit(tail_[u], junctions_[j]);
        for (uint16_t k : meets_[j]) {
            const Edge &next = edges_[k];
            for (int b = 0; b < 2; b++) {
                if ((b ? next.to : next.from) != j) {
                    continue;
                }
                int32_t v = 2 * k + b;
                if (k == u / 2 && b != (int)back) {
                    continue;  // about, back along the same edge
                }
                Vec2 out = b ? Vec2{-next.in.x, -next.in.y} : next.out;
                float turn = in.x * out.x + in.y * out.y;
                if (turn < COS_MOST_TURN) {
                    continue;
                }
                // Off a runway rolling out: only by a turn it can slow for.
                if (rules_.speed > 0 && e.kind == airfield::WayKind::RUNWAY &&
                    next.kind != airfield::WayKind::RUNWAY) {
                    float exit = turn >= EXIT_FAST_COS  ? EXIT_FAST
                                 : turn >= EXIT_MID_COS   ? EXIT_MID
                                 : turn >= COS_SHARP_TURN ? EXIT_SLOW
                                                          : EXIT_SHARP;
                    if (rules_.speed > exit && rules_.speed * rules_.speed - exit * exit > 2 * rules_.brake * c) {
                        continue;
                    }
                }
                float angle = acosf(std::max(-1.0f, std::min(1.0f, turn))) / (float)(M_PI / 2);
                float ce = c + TURN_COST * angle * angle + (turn < COS_SHARP_TURN ? SHARP_COST : 0);
                if (ce >= enter_[v]) {
                    continue;
                }
                enter_[v] = ce;
                before_[v] = u;
                float factor = next.kind == airfield::WayKind::RUNWAY ? rules_.runwayFactor : 1;
                float ca = ce + length(k) * factor;
                if (ca < arrive_[v]) {
                    arrive_[v] = ca;
                    started_[v] = 0;
                    float l = length(k);
                    tail_[v] = l >= END_SPAN ? (b ? next.afterFrom : next.beforeTo)
                                             : Vec2{junctions_[j].x - in.x * (END_SPAN - l),
                                                    junctions_[j].y - in.y * (END_SPAN - l)};
                    push(ca, v);
                }
            }
        }
    }
}

namespace {

// Which of the ways to a place a search found is the cheapest: in along its
// edge forwards or backwards, or on along the search's own edge.
enum class Way : uint8_t { NONE, IN_FORWARDS, IN_BACKWARDS, ON_FORWARDS, ON_BACKWARDS };

}  // namespace

float Network::costTo(const Place &to, float arrive) const {
    float cost = INF;
    if (edges_.empty() || enter_.size() != edges_.size() * 2) {
        return cost;
    }
    const Edge &b = edges_[to.edge];
    float factor = b.kind == airfield::WayKind::RUNWAY ? rules_.runwayFactor : 1;
    float h = heading(to);
    bool forwards = isnan(arrive) || cosf(h - arrive) >= COS_MOST_ARRIVE;
    bool backwards = isnan(arrive) || cosf(h + (float)M_PI - arrive) >= COS_MOST_ARRIVE;
    if (forwards) {
        cost = std::min(cost, enter_[2 * to.edge] + to.s * factor);
    }
    if (backwards) {
        cost = std::min(cost, enter_[2 * to.edge + 1] + (length(to.edge) - to.s) * factor);
    }
    if (to.edge == from_.edge) {
        if (forwards && to.s >= from_.s) {
            cost = std::min(cost, startCost_[0] + to.s - from_.s);
        }
        if (backwards && to.s <= from_.s) {
            cost = std::min(cost, startCost_[1] + from_.s - to.s);
        }
    }
    return cost;
}

bool Network::pathTo(const Place &to, float arrive, airfield::List<Vec2> &path, airfield::List<uint8_t> *kinds,
                     airfield::List<float> *forks) const {
    path.clear();
    if (kinds) {
        kinds->clear();
    }
    if (forks) {
        forks->clear();
    }
    if (edges_.empty() || enter_.size() != edges_.size() * 2) {
        return false;
    }
    const Edge &b = edges_[to.edge];
    float factor = b.kind == airfield::WayKind::RUNWAY ? rules_.runwayFactor : 1;
    float h = heading(to);
    bool forwards = isnan(arrive) || cosf(h - arrive) >= COS_MOST_ARRIVE;
    bool backwards = isnan(arrive) || cosf(h + (float)M_PI - arrive) >= COS_MOST_ARRIVE;
    Way way = Way::NONE;
    float best = INF;
    auto consider = [&](bool ok, float cost, Way w) {
        if (ok && cost < best) {
            best = cost;
            way = w;
        }
    };
    consider(forwards, enter_[2 * to.edge] + to.s * factor, Way::IN_FORWARDS);
    consider(backwards, enter_[2 * to.edge + 1] + (length(to.edge) - to.s) * factor, Way::IN_BACKWARDS);
    if (to.edge == from_.edge) {
        consider(forwards && to.s >= from_.s, startCost_[0] + to.s - from_.s, Way::ON_FORWARDS);
        consider(backwards && to.s <= from_.s, startCost_[1] + from_.s - to.s, Way::ON_BACKWARDS);
    }
    if (way == Way::NONE) {
        return false;
    }
    auto add = [&](Vec2 p, airfield::WayKind kind) {
        if (path.empty() || dist(path.back(), p) > 0.01f) {
            path.push_back(p);
            if (kinds) {
                kinds->push_back((uint8_t)kind);
            }
        }
    };
    // An edge's points from `s0` to `s1`, either way along it, neither end's.
    auto part = [&](const Edge &e, float s0, float s1) {
        if (s0 <= s1) {
            for (size_t i = 0; i < e.points.size(); i++) {
                if (e.lengths[i] > s0 && e.lengths[i] < s1) {
                    add(e.points[i], e.kind);
                }
            }
        } else {
            for (size_t i = e.points.size(); i-- > 0;) {
                if (e.lengths[i] < s0 && e.lengths[i] > s1) {
                    add(e.points[i], e.kind);
                }
            }
        }
    };
    const Edge &a = edges_[from_.edge];
    add(at(from_), a.kind);
    if (way == Way::ON_FORWARDS || way == Way::ON_BACKWARDS) {
        part(a, from_.s, to.s);
        add(at(to), b.kind);
        return true;
    }
    // The edges turned onto, back from the one `to` is on to the one the
    // search left its place along.
    static airfield::List<int32_t> chain;
    chain.clear();
    int32_t u = before_[2 * to.edge + (way == Way::IN_BACKWARDS ? 1 : 0)];
    for (size_t guard = 0; u >= 0 && guard <= enter_.size(); guard++) {
        chain.push_back(u);
        if (started_[u]) {
            break;
        }
        u = before_[u];
    }
    if (chain.empty() || !started_[chain.back()]) {
        path.clear();
        if (kinds) {
            kinds->clear();
        }
        return false;
    }
    std::reverse(chain.begin(), chain.end());
    // Whether the turn from state `u` onto state `v` at their junction is one
    // of two or more much the same.
    auto fork = [&](int32_t u, int32_t v) {
        const Edge &a = edges_[u / 2], &b = edges_[v / 2];
        if (a.kind == airfield::WayKind::RUNWAY) {
            return false;
        }
        uint16_t j = (u & 1) ? a.from : a.to;
        Vec2 in = (u & 1) ? Vec2{-a.out.x, -a.out.y} : a.in;
        Vec2 out = (v & 1) ? Vec2{-b.in.x, -b.in.y} : b.out;
        float taken = in.x * out.x + in.y * out.y;
        for (uint16_t k : meets_[j]) {
            for (int c = 0; c < 2; c++) {
                const Edge &o = edges_[k];
                if ((c ? o.to : o.from) != j || 2 * k + c == v || (k == u / 2 && c != (u & 1))) {
                    continue;
                }
                Vec2 other = c ? Vec2{-o.in.x, -o.in.y} : o.out;
                float turn = in.x * other.x + in.y * other.y;
                if (turn < COS_MOST_TURN) {
                    continue;  // not a way it could go
                }
                // Within 45 degrees of the way taken; or straighter, the way
                // taken turning by more than 30.
                if (other.x * out.x + other.y * out.y > 0.707f || (taken < 0.866f && turn > taken)) {
                    return true;
                }
            }
        }
        return false;
    };
    for (size_t i = 0; i < chain.size(); i++) {
        const Edge &e = edges_[chain[i] / 2];
        bool back = chain[i] & 1;
        float s0 = i == 0 ? from_.s : back ? e.lengths.back() : 0;
        part(e, s0, back ? 0 : e.lengths.back());
        add(back ? e.points.front() : e.points.back(), e.kind);
        int32_t next = i + 1 < chain.size() ? chain[i + 1] : 2 * to.edge + (way == Way::IN_BACKWARDS ? 1 : 0);
        if (forks && fork(chain[i], next)) {
            forks->push_back(lengthOf(path));
        }
    }
    bool back = way == Way::IN_BACKWARDS;
    part(b, back ? b.lengths.back() : 0, to.s);
    add(at(to), b.kind);
    return true;
}

bool Network::route(const Place &from, const Place &to, float most, airfield::List<Vec2> &path,
                    float heading) const {
    search(from, heading, most);
    return costTo(to) <= most && pathTo(to, NAN, path);
}

void Network::ahead(const Place &from, float heading, float distance, airfield::List<Vec2> &path,
                    airfield::List<uint8_t> &kinds) const {
    path.clear();
    kinds.clear();
    if (edges_.empty()) {
        return;
    }
    auto add = [&](Vec2 p, airfield::WayKind kind) {
        if (path.empty() || dist(path.back(), p) > 0.01f) {
            path.push_back(p);
            kinds.push_back((uint8_t)kind);
        }
    };
    const Edge *e = &edges_[from.edge];
    bool back = !isnan(heading) && cosf(this->heading(from) - heading) < 0;
    add(at(from), e->kind);
    float gone = 0, s = from.s;
    uint16_t k = from.edge;
    for (int guard = 0; guard < 64; guard++) {
        // On along this edge to its end, or as far as is wanted.
        if (back) {
            for (size_t i = e->points.size(); i-- > 0;) {
                if (e->lengths[i] < s) {
                    float step = s - e->lengths[i];
                    if (gone + step >= distance) {
                        float h;
                        add(along(e->points, s - (distance - gone), h), e->kind);
                        return;
                    }
                    gone += step;
                    s = e->lengths[i];
                    add(e->points[i], e->kind);
                }
            }
        } else {
            for (size_t i = 0; i < e->points.size(); i++) {
                if (e->lengths[i] > s) {
                    float step = e->lengths[i] - s;
                    if (gone + step >= distance) {
                        float h;
                        add(along(e->points, s + (distance - gone), h), e->kind);
                        return;
                    }
                    gone += step;
                    s = e->lengths[i];
                    add(e->points[i], e->kind);
                }
            }
        }
        // The straightest way on from the junction.
        uint16_t j = back ? e->from : e->to;
        Vec2 in = back ? Vec2{-e->out.x, -e->out.y} : e->in;
        int32_t best = -1;
        bool bestBack = false;
        float straightest = 0.5f;  // cos 60 degrees
        for (uint16_t n : meets_[j]) {
            const Edge &next = edges_[n];
            for (int b = 0; b < 2; b++) {
                if ((b ? next.to : next.from) != j || (n == k && b != (int)back)) {
                    continue;
                }
                Vec2 out = b ? Vec2{-next.in.x, -next.in.y} : next.out;
                float c = in.x * out.x + in.y * out.y;
                if (c > straightest) {
                    straightest = c;
                    best = n;
                    bestBack = b;
                }
            }
        }
        if (best < 0) {
            return;
        }
        k = (uint16_t)best;
        e = &edges_[k];
        back = bestBack;
        s = back ? e->lengths.back() : 0;
    }
}

float lengthOf(const airfield::List<Vec2> &path) {
    float total = 0;
    for (size_t i = 1; i < path.size(); i++) {
        total += dist(path[i - 1], path[i]);
    }
    return total;
}

float parkBack(const StandWay &st, float noseWheel, float nose) {
    float back = std::max(st.middle ? 0.0f : noseWheel, nose - (st.room - PARK_CLEAR));
    return std::max(0.0f, std::min(back, lengthOf(st.line)));
}

Vec2 parkedAt(const StandWay &st, float back, float &heading) {
    if (st.line.empty()) {
        heading = st.heading;
        return {0, 0};
    }
    float h = st.heading;
    Vec2 p = along(st.line, std::max(0.0f, lengthOf(st.line) - back), h);
    heading = dist(p, st.line.back()) > 3 ? headingOf(unit(p, st.line.back())) : st.heading;
    return p;
}

void lineIn(const StandWay &st, float back, airfield::List<Vec2> &out) {
    out.clear();
    if (st.line.empty()) {
        return;
    }
    float want = std::max(0.0f, lengthOf(st.line) - back), at = 0;
    out.push_back(st.line[0]);
    for (size_t i = 1; i < st.line.size(); i++) {
        Vec2 a = st.line[i - 1], b = st.line[i];
        float d = dist(a, b);
        if (at + d >= want) {
            float t = d < 1e-4f ? 1 : (want - at) / d;
            Vec2 p = {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
            if (dist(p, out.back()) > 0.05f) {
                out.push_back(p);
            }
            return;
        }
        at += d;
        out.push_back(b);
    }
}

Vec2 along(const airfield::List<Vec2> &path, float distance, float &heading) {
    if (path.empty()) {
        return {0, 0};
    }
    for (size_t i = 1; i < path.size(); i++) {
        float d = dist(path[i - 1], path[i]);
        if (d < 1e-4f) {
            continue;
        }
        heading = atan2f(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
        if (distance <= d || i + 1 == path.size()) {
            float t = std::max(0.0f, std::min(1.0f, distance / d));
            return {path[i - 1].x + (path[i].x - path[i - 1].x) * t, path[i - 1].y + (path[i].y - path[i - 1].y) * t};
        }
        distance -= d;
    }
    return path.back();
}

}  // namespace ground
