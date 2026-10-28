// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's take-off, landing and taxi simulation: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

#include "ground_follow.h"

#include <math.h>
#include <stdio.h>

#include <algorithm>

#ifdef GROUND_TRACE
#define TRACE(...) printf(__VA_ARGS__)
#else
#define TRACE(...)
#endif

namespace ground {
namespace {

// Slower than this, metres a second - six knots - it is standing still: its
// track means nothing, and where it is wanders by metres.
constexpr float STILL = 3.1f;
// No report for this long, it is taxied on by itself.
constexpr uint32_t LOST_MS = 15000;
// A report this near a line is on it; one standing this near a stand's stop,
// at the stand.
constexpr float REACH = 45, AT_STAND = 35;
// Slower than STILL, but this far from the report before: moving - further
// than a standing one's reports wander from one to the next, by 10m and
// more.
constexpr float CREPT = 25;
// Off the lines - across to one, into a stand - it turns no tighter than this,
// metres radius.
constexpr float TURN_RADIUS = 20;
// A report this near a stand's line in is on it - unless this near the
// network: on a taxilane, going past.
constexpr float ON_LINE = 12, ON_TAXILANE = 8;
// A report whose place on the taxiways is this near the way it is taking is
// on it: the plan cuts the corners by up to 20m, its middle by less.
constexpr float ON_PLAN = 8;
// At a fork no report has said which way it took at, it waits - no further
// than this past it, where the ways have parted - for one that does.
constexpr float FORK_ROOM = 8, FORK_NEAR = 22, FORK_BRAKE = 0.8f;
// Parked within this many metres of where it parks, it settles there - at
// up to these many radians and metres a second.
constexpr float SETTLE = 3, SETTLE_TURN = 0.07f, SETTLE_SPEED = 0.3f;
// It waits before the plan's turn there begins - its rounded corner, up to
// this far before the fork - unless it has begun it already; until a report
// has it this far past the fork, on one way or the other.
constexpr float FORK_LEAD = 45, FORK_PAST = 15;
// Making for a stand by itself, no report having put it there, it waits at
// the stand's way in while its last report is no older than this: for one
// to say it is that stand - or, its reports stopped, in to it.
constexpr uint32_t STAND_WAIT_MS = 75000;
// Between reports, carried on from the last at its speed - slowing for the
// turns ahead as it would - for no more than this.
constexpr float CARRY_S = 20;
// In step: going faster or slower than the reports' speed by this much a
// second for each metre behind or ahead.
constexpr float KEEP_UP = 0.15f;
// Pushed back along the taxilane, tail first, this far past its stand's line
// - its tail swung round onto it from this far back along the line, or less
// of a short one.
constexpr float PUSH_ALONG = 35, PUSH_TURN = 18;
// Seen to have left its stand only once taxiing, or this far out: pushed
// back faster, metres a second, to catch up.
constexpr float LATE_PUSH = 60, PUSH_LATE = 2.5f;
// Parked, reported this far from where it was seen there - and nearer the
// taxilane - it has left; this far, if its speed says it is going, or two
// reports one after the other do. Further, the more its reports wander
// about where it stands: this far on average, until some have.
constexpr float LEFT_STAND = 45, LEFT_STAND_GOING = 15, WANDER = 8, MOST_WANDER = 20;
// Or in two reports one after the other this far from it, whichever way -
// gone, seen late; and ahead of its stop by this much isn't out of it.
constexpr float LEFT_FAR = 150, AHEAD_OF_STOP = 5;
// A report this far from the one before, at more than twice either's speed
// and this many metres a second besides, is wild; and one this much nearer
// another stand's stop than the one it is making for is at that one.
constexpr float WILD_JUMP = 60, WILD_SPEED = 8, STAND_SWITCH = 25;
// A report off every taxiway - in an apron's open space - is gone to straight,
// unless going faster than this, metres a second, or further than
// OFF_NETWORK_MOST from where it is shown: wild.
constexpr float OFF_NETWORK_FAST = 6, OFF_NETWORK_MOST = 150;
// Departing, standing no nearer a stand's stop than this, it isn't at that
// stand; and one shown further than SNAP_PARK from a stand's stop it is
// reported standing at isn't put there, but taxied there.
constexpr float AT_STAND_DEPARTING = 12, SNAP_PARK = 60;
// Unless reported standing at it in this many reports one after the other.
constexpr int STANDING_PARK = 3;
// Just begun, shown no further than this from a stand's stop and from its
// report, standing at it - the view opened on it there: parked there at once.
constexpr float BEGUN_PARKED = 60;
// A stand with another aircraft standing this near its stop is taken.
constexpr float TAKEN = 25;
// Holding points other than at a runway's end - crossings - count as this
// much further: a departure is making for the end.
constexpr float NOT_AN_END = 1500;
// Where it is making for is kept to unless another is this much nearer.
constexpr float STICK = 150;
// And looked for along the taxiways this far: Schiphol's Polderbaan is 13km
// by taxiway from its piers. Finding none, stopped, it looks again - free to
// turn about - no more often than this.
constexpr float WAY_ON_MOST = 25000;
constexpr uint32_t OWN_WAY_AGAIN_MS = 10000;
// A departure this near a runway's end hold, going on, is lining up: this
// far down the runway. As is one going on from no further past one.
constexpr float AT_HOLD = 15, LINED_UP_IN = 60, PASSED_HOLD = 150;
// Past one - its runway this near ahead - is lining up too.
constexpr float RUNWAY_AHEAD = 120;
// Holding points not at a runway end departures are using - none this near
// where one starts - count as this much further, where it is known which
// those are.
constexpr float NOT_IN_USE = 3000, IN_USE_NEAR = 700;
// A stand away from the terminals - remote, cargo's, the flying club's -
// counts as this much further, landed, where the airport has stands at them.
constexpr float REMOTE = 800;
// Gone a different way from the way its report has it going, but no further
// from it than this: across to it, smoothly, rather than round by the
// taxiways. No further than ACROSS_MOST from the way, nor round in a circle -
// turning more than MOST_ACROSS radians all told - to a point on it up to
// ACROSS_REACH metres on: otherwise on by itself, along the taxiways.
constexpr float ACROSS = 25, ACROSS_MOST = 80, MOST_ACROSS = 3.5f, ACROSS_REACH = 150;

float dist(Vec2 a, Vec2 b) { return hypotf(b.x - a.x, b.y - a.y); }

// Whether `h` is at a runway end departures are using - or there's no
// saying which those are.
bool departingFrom(const Network &net, const HoldWay &h, const Follower::Input &in) {
    if (in.departFrom == nullptr || in.departFromCount <= 0) {
        return true;
    }
    Vec2 at = net.at(h.at);
    for (int i = 0; i < in.departFromCount; i++) {
        if (dist(at, in.departFrom[i]) < IN_USE_NEAR) {
            return true;
        }
    }
    return false;
}

// The point on `line` nearest `p` - in its first `within` metres - how far
// along, how far off, and the way the line runs there.
float nearestOn(const airfield::List<Vec2> &line, Vec2 p, float &off, float &heading, float within = 1e30f) {
    off = 1e30f;
    float best = 0, at = 0;
    for (size_t i = 1; i < line.size() && at <= within; i++) {
        Vec2 a = line[i - 1], b = line[i];
        float dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy, len = sqrtf(len2);
        if (len < 1e-3f) {
            continue;
        }
        float t = std::max(0.0f, std::min(1.0f, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
        float d = hypotf(a.x + dx * t - p.x, a.y + dy * t - p.y);
        if (d < off) {
            off = d;
            best = at + len * t;
            heading = atan2f(dy, dx);
        }
        at += len;
    }
    return best;
}

// A smooth curve from `p0` facing `h0` to `p1` facing `h1`: points every 5m
// or so, neither end's.
void curve(Vec2 p0, float h0, Vec2 p1, float h1, airfield::List<Vec2> &out, airfield::List<uint8_t> &kinds) {
    float d = dist(p0, p1), m = d * 0.6f;
    float tx0 = cosf(h0) * m, ty0 = sinf(h0) * m, tx1 = cosf(h1) * m, ty1 = sinf(h1) * m;
    int n = std::max(2, (int)(d / 5));
    for (int i = 1; i < n; i++) {
        float t = (float)i / n, t2 = t * t, t3 = t2 * t;
        float a = 2 * t3 - 3 * t2 + 1, b = t3 - 2 * t2 + t, c = -2 * t3 + 3 * t2, e = t3 - t2;
        out.push_back({a * p0.x + b * tx0 + c * p1.x + e * tx1, a * p0.y + b * ty0 + c * p1.y + e * ty1});
        kinds.push_back((uint8_t)airfield::WayKind::TAXILANE);
    }
}

// The way an aircraft gets from `p0` facing `h0` to `p1` facing `h1`, turning
// no tighter than `radius`: a turn, a straight and a turn - the shortest of
// the four such ways - as points a few metres apart, neither end's. False,
// and nothing, if they are on top of each other.
// Each side's turning circle's middle: left, s = 1; right, s = -1.
Vec2 turnMiddle(Vec2 p, float h, float s, float radius) {
    return Vec2{p.x - s * radius * sinf(h), p.y + s * radius * cosf(h)};
}

float turnWrap(float a) {
    a = fmodf(a, 2 * (float)M_PI);
    return a < 0 ? a + 2 * (float)M_PI : a;
}

// The shortest of turnAcross()'s four ways: which way each turn goes, the
// straight's heading, and how far it turns all told, radians. False if there
// is none.
bool shortestAcross(Vec2 p0, float h0, Vec2 p1, float h1, float radius, float &bestS0, float &bestS1, float &bestPhi,
                    float &turned) {
    auto middle = [&](Vec2 p, float h, float s) { return turnMiddle(p, h, s, radius); };
    auto wrap = turnWrap;
    float best = 1e30f;
    bestS0 = bestS1 = 1;
    bestPhi = 0;
    for (float s0 : {1.0f, -1.0f}) {
        for (float s1 : {1.0f, -1.0f}) {
            Vec2 c0 = middle(p0, h0, s0), c1 = middle(p1, h1, s1);
            float dx = c1.x - c0.x, dy = c1.y - c0.y, d = hypotf(dx, dy), a = atan2f(dy, dx);
            float phi;
            if (s0 == s1) {
                phi = a;  // the outer tangent
            } else {
                if (d < 2 * radius) {
                    continue;
                }
                phi = a + s0 * asinf(2 * radius / d);  // the inner
            }
            // Where the straight leaves the first circle and meets the second.
            Vec2 t0 = {c0.x + s0 * radius * sinf(phi), c0.y - s0 * radius * cosf(phi)};
            Vec2 t1 = {c1.x + s1 * radius * sinf(phi), c1.y - s1 * radius * cosf(phi)};
            float turn0 = wrap(s0 * (phi - h0)), turn1 = wrap(s1 * (h1 - phi));
            float length = radius * (turn0 + turn1) + dist(t0, t1);
            if (length < best) {
                best = length;
                bestS0 = s0, bestS1 = s1, bestPhi = phi;
                turned = turn0 + turn1;
            }
        }
    }
    return best < 1e29f;
}

bool turnAcross(Vec2 p0, float h0, Vec2 p1, float h1, float radius, airfield::List<Vec2> &out,
                airfield::List<uint8_t> &kinds, uint8_t kind) {
    float bestS0, bestS1, bestPhi, turned;
    if (dist(p0, p1) < 1 || !shortestAcross(p0, h0, p1, h1, radius, bestS0, bestS1, bestPhi, turned)) {
        return false;
    }
    auto middle = [&](Vec2 p, float h, float s) { return turnMiddle(p, h, s, radius); };
    auto wrap = turnWrap;
    auto arc = [&](Vec2 p, float h, float s, float turn) {
        Vec2 c = middle(p, h, s);
        int steps = std::max(1, (int)ceilf(turn / 0.17f));
        for (int i = 1; i <= steps; i++) {
            float hh = h + s * turn * i / steps;
            out.push_back({c.x + s * radius * sinf(hh), c.y - s * radius * cosf(hh)});
            kinds.push_back(kind);
        }
    };
    float turn0 = wrap(bestS0 * (bestPhi - h0)), turn1 = wrap(bestS1 * (h1 - bestPhi));
    arc(p0, h0, bestS0, turn0);
    // The second turn, from where the straight meets it.
    Vec2 c1 = middle(p1, h1, bestS1);
    Vec2 t1 = {c1.x + bestS1 * radius * sinf(bestPhi), c1.y - bestS1 * radius * cosf(bestPhi)};
    out.push_back(t1);
    kinds.push_back(kind);
    int steps = std::max(1, (int)ceilf(turn1 / 0.17f));
    for (int i = 1; i < steps; i++) {
        float hh = bestPhi + bestS1 * turn1 * i / steps;
        out.push_back({c1.x + bestS1 * radius * sinf(hh), c1.y - bestS1 * radius * cosf(hh)});
        kinds.push_back(kind);
    }
    return true;
}

// Across onto `line` from `p0` facing `h0`, as turnAcross() has it: to the
// point `from` metres along it; or - that too near to turn onto without
// going round in a circle, more than `most` radians all told - the first one
// further on, up to `reach` further, that isn't; failing any, the one
// turning least. Its points onto `out`, that one on the line last; how far
// along the line it is, and in `turned`, how far the way turns all told. -1,
// and nothing, if there is no way at all.
float turnOnto(Vec2 p0, float h0, const airfield::List<Vec2> &line, float from, float reach, float most, float &turned,
               airfield::List<Vec2> &out, airfield::List<uint8_t> &kinds, uint8_t kind) {
    float length = lengthOf(line), best = -1;
    from = std::min(std::max(from, 0.0f), length);
    turned = 1e30f;
    for (float s = from;; s = std::min(s + 10, length)) {
        float h = 0, s0, s1, phi, t;
        Vec2 q = along(line, s, h);
        if (dist(p0, q) >= 1 && shortestAcross(p0, h0, q, h, TURN_RADIUS, s0, s1, phi, t) && t < turned) {
            best = s;
            turned = t;
            if (t <= most) {
                break;
            }
        }
        if (s >= length || s >= from + reach) {
            break;
        }
    }
    if (best < 0) {
        return -1;
    }
    float h = 0;
    Vec2 q = along(line, best, h);
    turnAcross(p0, h0, q, h, TURN_RADIUS, out, kinds, kind);
    out.push_back(q);
    kinds.push_back(kind);
    return best;
}

// `from`'s points after `s` along it, and their kinds, on the end of `out`.
void appendAfter(const airfield::List<Vec2> &from, const airfield::List<uint8_t> &fromKinds, float s,
                 airfield::List<Vec2> &out, airfield::List<uint8_t> &kinds) {
    float at = 0;
    for (size_t i = 0; i < from.size(); i++) {
        if (i > 0) {
            at += dist(from[i - 1], from[i]);
        }
        if (at > s + 0.01f) {
            if (out.empty() || dist(out.back(), from[i]) > 0.01f) {
                out.push_back(from[i]);
                kinds.push_back(i < fromKinds.size() ? fromKinds[i] : (uint8_t)airfield::WayKind::TAXIWAY);
            }
        }
    }
}

// A way from where it is shown, `path`, started smoothly from the way it
// faces, `heading`, going `speed`: eased onto if it turns by a little; if by
// more - the line it was snapped to running across the way it goes, as it
// cuts a corner - a turn across to it further on, as tight as it can turn.
void smoothStart(airfield::List<Vec2> &path, airfield::List<uint8_t> &kinds, float heading, float speed);

// Whether a way found from where it is shown starts back the way it came -
// from the network place nearest, which can be behind it, beside a line - as
// one going at a taxi can't.
bool turnsAbout(const airfield::List<Vec2> &path, float heading, float speed) {
    if (speed < 2 || path.size() < 2) {
        return false;
    }
    float h = 0;
    Vec2 ahead = along(path, std::min(15.0f, lengthOf(path)), h);
    Vec2 d = {ahead.x - path[0].x, ahead.y - path[0].y};
    return d.x * cosf(heading) + d.y * sinf(heading) < -0.34f * hypotf(d.x, d.y);
}

// A way found from the network place nearest where it is shown, `p`, started
// at `p` itself: shown cutting a corner, a few metres inside it, it carries on
// from there rather than jumping back to the line.
void startAt(airfield::List<Vec2> &path, Vec2 p) {
    if (!path.empty() && dist(path[0], p) < 30) {
        path[0] = p;
    }
}

// And its first few metres - `reach`, as far as it goes in a couple of
// seconds - a curve from the way it is facing, `heading`, onto the way: no
// twitch as a new way starts, mid-turn, a little off the old one's line.
// Unless the way leaves the way it faces by more than 60 degrees: then it
// turns as the way does.
void easeOnto(airfield::List<Vec2> &path, airfield::List<uint8_t> &kinds, float heading, float reach) {
    if (path.size() < 2 || lengthOf(path) < reach * 1.5f) {
        return;
    }
    float h = 0;
    Vec2 q = along(path, reach, h);
    if (cosf(h - heading) < 0.5f) {
        return;
    }
    // The points before it, out; the curve to it, in.
    float at = 0;
    size_t i = 1;
    for (; i < path.size(); i++) {
        at += dist(path[i - 1], path[i]);
        if (at > reach) {
            break;
        }
    }
    static airfield::List<Vec2> curved;
    static airfield::List<uint8_t> curvedKinds;
    curved.clear();
    curvedKinds.clear();
    curved.push_back(path[0]);
    curvedKinds.push_back(kinds[0]);
    curve(path[0], heading, q, h, curved, curvedKinds);
    for (size_t k = 1; k < curvedKinds.size(); k++) {
        curvedKinds[k] |= KEEP;
    }
    curved.push_back(q);
    curvedKinds.push_back(kinds[std::min(i, kinds.size() - 1)] | KEEP);
    for (; i < path.size(); i++) {
        curved.push_back(path[i]);
        curvedKinds.push_back(kinds[i]);
    }
    path = curved;
    kinds = curvedKinds;
}

void smoothStart(airfield::List<Vec2> &path, airfield::List<uint8_t> &kinds, float heading, float speed) {
    float reach = std::max(10.0f, std::min(30.0f, speed * 2.5f));
    if (path.size() < 2 || lengthOf(path) < reach * 1.5f) {
        return;
    }
    float h = 0;
    along(path, reach, h);
    if (cosf(h - heading) >= 0.5f) {
        easeOnto(path, kinds, heading, reach);
        return;
    }
    float to = std::min(lengthOf(path), std::max(40.0f, reach * 2)), turnedBy;
    static airfield::List<Vec2> turned;
    static airfield::List<uint8_t> turnedKinds;
    turned.clear();
    turnedKinds.clear();
    turned.push_back(path[0]);
    turnedKinds.push_back(kinds[0] | KEEP);
    float at = turnOnto(path[0], heading, path, to, ACROSS_REACH, MOST_ACROSS, turnedBy, turned, turnedKinds,
                        kinds[0] | KEEP);
    if (at < 0) {
        return;  // no way onto it: as it is
    }
    appendAfter(path, kinds, at, turned, turnedKinds);
    path = turned;
    kinds = turnedKinds;
}

}  // namespace

void Follower::keepUpFrom(float s, float speed, uint32_t ms) {
    syncS_ = s;
    syncSpeed_ = speed;
    syncMs_ = ms;
    ghostS_ = s;
    ghostSpeed_ = speed;
}

void Follower::stopSoon() {
    const Plan &was = taxi_.plan();
    float v = taxi_.speed();
    static airfield::List<Vec2> pts;
    static airfield::List<uint8_t> ks;
    pts.clear();
    ks.clear();
    if (!was.empty() && v > 0.3f) {
        float from = taxi_.along(), to = std::min(was.length(), from + v * v / (2 * TAXI_BRAKE) + 1), h;
        pts.push_back(taxi_.at());
        ks.push_back(was.kindAt(from));
        for (float s = from + 4; s < to; s += 4) {
            pts.push_back(was.pointAt(s, h));
            ks.push_back(was.kindAt(s));
        }
        pts.push_back(was.pointAt(to, h));
        ks.push_back(was.kindAt(to));
    }
    plan_.build(pts, ks, true);
    taxi_.take(plan_);
}

float Follower::acrossOnto(const airfield::List<Vec2> &line, float from, airfield::List<Vec2> &pts,
                           airfield::List<uint8_t> &ks) {
    float turned;
    float at = turnOnto(taxi_.at(), taxi_.heading(), line, from, ACROSS_REACH, MOST_ACROSS, turned, pts, ks,
                        ON_STAND_LINE | KEEP);
    if (at >= 0) {
        return at;
    }
    float h;
    pts.push_back(along(line, from, h));
    ks.push_back(ON_STAND_LINE | KEEP);
    return from;
}

void Follower::say(const char *what, const char *ref) {
    if (ref != nullptr && ref[0] != 0) {
        snprintf(doing_, sizeof doing_, "%s %s", what, ref);
    } else {
        snprintf(doing_, sizeof doing_, "%s", what);
    }
}

int Follower::standNear(const Network &net, Vec2 p, float within) const {
    int best = -1;
    for (size_t i = 0; i < net.stands().size(); i++) {
        const StandWay &st = net.stands()[i];
        float d = st.ok ? dist(p, st.line.back()) : 1e30f;
        if (d < within) {
            within = d;
            best = (int)i;
        }
    }
    return best;
}

Vec2 Follower::parkPoint(const StandWay &st, float &heading) const {
    return parkedAt(st, parkBack(st, noseWheel_, nose_), heading);
}

const airfield::List<Vec2> &Follower::lineInto(const StandWay &st) {
    lineIn(st, parkBack(st, noseWheel_, nose_), standLine_);
    return standLine_;
}

void Follower::notPast(const StandWay &st) {
    float off, h;
    float s = nearestOn(st.line, taxi_.at(), off, h);
    if (off < 20 && s > lengthOf(st.line) - parkBack(st, noseWheel_, nose_) + 0.5f) {
        float heading = st.heading;
        Vec2 at = parkPoint(st, heading);
        taxi_.place(at, heading, 0);
    }
}

void Follower::begin(const Network &net, Vec2 at, float heading, float speed, bool landed, const Input &in) {
    active_ = true;
    noseWheel_ = in.noseWheel;
    nose_ = in.nose;
    taxi_.place(at, heading, speed);
    taxi_.landing = landed;
    plan_.clear();
    way_.clear();
    kinds_.clear();
    stand_ = hold_ = parkedAt_ = leftStand_ = -1;
    standSeen_ = false;
    haveParkedSeen_ = false;
    prevMs_ = 0;
    wildInRow_ = 0;
    rolling_ = false;
    arriving_ = landed || in.arrived;
    mode_ = Mode::FOLLOW;
    reportMs_ = 0;  // the report there is, taken afresh
    say("following its reports");
    justBegun_ = !landed;
    if (!landed) {
        return;
    }
    // Rolling out: to the first exit it can make, and on to a stand.
    reportMs_ = in.reportMs;  // the report before - in the air, or rolling faster - had
    Place from;
    if (net.snap(at, heading, 60, from)) {
        Rules rules;
        rules.speed = speed;
        rules.brake = 1.2f;  // planned a little gentler than it can brake
        rules.runwayFactor = 1.5f;
        rules.back = false;
        wayOn(net, from, heading, rules, in);
        startAt(way_, at);
        // Shown a little past the network place it was snapped to - the
        // junction of the exit it will take, say, behind it - not back to it.
        smoothStart(way_, kinds_, heading, speed);
        plan_.build(way_, kinds_, true);
        taxi_.take(plan_);
        placeForks();
    }
    mode_ = Mode::OWN_WAY;
    say(stand_ >= 0 ? "rolling out, for stand" : "rolling out", stand_ >= 0 ? net.stands()[stand_].ref : nullptr);
}

void Follower::wayOn(const Network &net, const Place &from, float heading, const Rules &rules, const Input &in) {
    int wasStand = stand_, wasHold = hold_;
    bool wasSeen = standSeen_;
    way_.clear();
    kinds_.clear();
    stand_ = hold_ = -1;
    lineUp_ = false;
    net.search(from, heading, WAY_ON_MOST, rules);
    float best = 1e30f, was = 1e30f;
    if (arriving_) {
        bool terminals = false;
        for (const StandWay &st : net.stands()) {
            terminals = terminals || (st.ok && st.terminal);
        }
        for (size_t i = 0; i < net.stands().size(); i++) {
            const StandWay &st = net.stands()[i];
            if (!st.ok) {
                continue;
            }
            bool taken = false;
            for (int k = 0; k < in.standingCount && !taken; k++) {
                taken = dist(in.standing[k], st.line.back()) < TAKEN;
            }
            float cost = taken ? 1e30f : net.costTo(st.entry, st.arrive) + (st.terminal || !terminals ? 0 : REMOTE);
            if ((int)i == wasStand) {
                was = cost;
            }
            if (cost < best) {
                best = cost;
                stand_ = (int)i;
            }
        }
    } else if (net.onRunway(from) && !isnan(heading)) {
        // Departing, on the runway already, going: lining up, or rolling - on
        // down it.
        net.ahead(from, heading, 4000, way_, kinds_);
        forks_.clear();
        lineUp_ = true;
        return;
    } else {
        for (size_t i = 0; i < net.holds().size(); i++) {
            const HoldWay &h = net.holds()[i];
            if (!h.ok || !h.runway) {
                continue;
            }
            float cost = net.costTo(h.at, h.arrive) + (h.end ? 0 : NOT_AN_END) + (departingFrom(net, h, in) ? 0 : NOT_IN_USE);
            if ((int)i == wasHold) {
                was = cost;
            }
            if (cost < best) {
                best = cost;
                hold_ = (int)i;
            }
        }
        // At a runway's end hold already, going on - or past one, between it
        // and the runway: lining up - onto the runway, turning down its
        // longer way.
        if (hold_ >= 0 && net.holds()[hold_].end && best < AT_HOLD && !isnan(heading) &&
            lineUp(net, hold_, net.holds()[hold_].at, net.holds()[hold_].arrive)) {
            return;
        }
        // Past one: the runway just ahead, a runway end's hold just behind.
        static airfield::List<Vec2> on;
        static airfield::List<uint8_t> onKinds;
        bool runwayAhead = false;
        if (!isnan(heading)) {
            net.ahead(from, heading, RUNWAY_AHEAD, on, onKinds);
            for (uint8_t k : onKinds) {
                runwayAhead = runwayAhead || k == (uint8_t)airfield::WayKind::RUNWAY;
            }
        }
        if (runwayAhead && !net.onRunway(from)) {
            int passed = -1;
            float nearest = PASSED_HOLD;
            Vec2 here = net.at(from);
            for (size_t i = 0; i < net.holds().size(); i++) {
                const HoldWay &h = net.holds()[i];
                Vec2 at = h.ok && h.runway && h.end ? net.at(h.at) : Vec2{1e30f, 1e30f};
                float d = dist(here, at);
                if (d < nearest && (here.x - at.x) * cosf(heading) + (here.y - at.y) * sinf(heading) > 0) {
                    nearest = d;
                    passed = (int)i;
                }
            }
            if (passed >= 0 && lineUp(net, passed, from, heading)) {
                return;
            }
            net.search(from, heading, WAY_ON_MOST, rules);  // lining up's search done: the holds' again
        }
    }
    // Still making for where it was - unless that is now well out of its way:
    // the way ahead not changing its mind at every report.
    if (was < 1e29f && was <= best + std::max(STICK, best * 0.25f)) {
        stand_ = arriving_ ? wasStand : -1;
        hold_ = arriving_ ? -1 : wasHold;
    }
    static airfield::List<float> at;
    forks_.clear();
    standSeen_ = wasSeen && stand_ == wasStand;
    if (stand_ >= 0) {
        const StandWay &st = net.stands()[stand_];
        if (net.pathTo(st.entry, st.arrive, way_, &kinds_, &at)) {
            for (float f : at) {
                float h;
                forks_.push_back(along(way_, f, h));
            }
            forks_.push_back(way_.back());  // in to the stand, or on past it
            const airfield::List<Vec2> &line = lineInto(st);
            for (size_t i = 1; i < line.size(); i++) {
                way_.push_back(line[i]);
                kinds_.push_back(ON_STAND_LINE);
            }
            return;
        }
        stand_ = -1;
    } else if (hold_ >= 0) {
        const HoldWay &h = net.holds()[hold_];
        if (net.pathTo(h.at, h.arrive, way_, &kinds_, &at)) {
            for (float f : at) {
                float hh;
                forks_.push_back(along(way_, f, hh));
            }
            return;
        }
        hold_ = -1;
    }
    net.ahead(from, heading, 400, way_, kinds_);
}

void Follower::placeForks() {
    forksOnPlan_.clear();
    forksWait_.clear();
    for (const Vec2 &f : forks_) {
        float off;
        float s = plan_.nearest(f, plan_.length(), off);
        if (off < FORK_NEAR) {
            forksOnPlan_.push_back(s);
        }
    }
    std::sort(forksOnPlan_.begin(), forksOnPlan_.end());
    for (float s : forksOnPlan_) {
        forksWait_.push_back(std::max(0.0f, plan_.turnStart(s, FORK_LEAD) - 2));
    }
}

float Follower::searchHeading(const Network &net, const Place &from) const {
    float h = taxi_.heading();
    return taxi_.speed() < 3 && fabsf(cosf(h - net.heading(from))) < 0.5f ? NAN : h;
}

bool Follower::lineUp(const Network &net, int hold, const Place &from, float heading) {
    // Where the way on from there - the way it faces - meets the runway.
    static airfield::List<Vec2> on;
    static airfield::List<uint8_t> onKinds;
    net.ahead(from, heading, 300, on, onKinds);
    Place runway;
    bool met = false;
    for (size_t i = 0; i < on.size() && !met; i++) {
        if (onKinds[i] == (uint8_t)airfield::WayKind::RUNWAY) {
            for (float reach = 2; reach <= 30 && !met; reach += 4) {
                met = net.snap(on[i], NAN, reach, runway) && net.onRunway(runway);
            }
        }
    }
    if (!met) {
        return false;
    }
    // Down the runway the longer way from there: the way it will take off.
    float along = net.heading(runway);
    static airfield::List<Vec2> one, other;
    static airfield::List<uint8_t> oneKinds, otherKinds;
    net.ahead(runway, along, 4000, one, oneKinds);
    net.ahead(runway, along + (float)M_PI, 4000, other, otherKinds);
    float takeoff = lengthOf(one) >= lengthOf(other) ? along : along + (float)M_PI;
    static airfield::List<Vec2> path;
    static airfield::List<uint8_t> kinds;
    Rules rules;
    rules.back = false;
    net.search(from, heading, 1000, rules);
    if (net.costTo(runway, takeoff) > AT_HOLD + 400 || !net.pathTo(runway, takeoff, path, &kinds)) {
        return false;
    }
    net.ahead(runway, takeoff, LINED_UP_IN, on, onKinds);
    for (size_t i = 1; i < on.size(); i++) {
        path.push_back(on[i]);
        kinds.push_back(onKinds[i]);
    }
    way_ = path;
    kinds_ = kinds;
    forks_.clear();
    hold_ = hold;
    lineUp_ = true;
    return true;
}

void Follower::planInto(const Network &net, int stand) {
    const StandWay &st = net.stands()[stand];
    notPast(st);
    const airfield::List<Vec2> &line = lineInto(st);
    static airfield::List<uint8_t> lineKinds;
    lineKinds.assign(line.size(), ON_STAND_LINE);
    way_.clear();
    kinds_.clear();
    float off, h;
    float sm = nearestOn(line, taxi_.at(), off, h);
    Place from;
    bool routed = false;
    if (off >= 6 && net.snap(taxi_.at(), taxi_.heading(), 30, from)) {
        Rules rules;
        rules.runwayFactor = 3;
        rules.back = taxi_.speed() < 1;
        net.search(from, searchHeading(net, from), 6000, rules);
        routed = net.pathTo(st.entry, st.arrive, way_, &kinds_) && !turnsAbout(way_, taxi_.heading(), taxi_.speed());
        if (routed) {
            startAt(way_, taxi_.at());
            smoothStart(way_, kinds_, taxi_.heading(), taxi_.speed());
            appendAfter(line, lineKinds, 0, way_, kinds_);
        }
    }
    if (!routed && off > ACROSS_MOST) {
        // No way to it along the taxiways, and too far from its line to go
        // across to it: stopped, for a report or its own way to bring it.
        stopSoon();
        stand_ = stand;
        hold_ = -1;
        forks_.clear();
        forksOnPlan_.clear();
        forksWait_.clear();
        return;
    }
    if (!routed) {
        // On its line already, or beside it: onto it a little further in.
        way_.clear();
        kinds_.clear();
        way_.push_back(taxi_.at());
        kinds_.push_back(ON_STAND_LINE | KEEP);
        float sq = std::min(sm + (off < 6 ? 0 : 30), lengthOf(line)), turned;
        if (off >= 6) {
            float at = turnOnto(taxi_.at(), taxi_.heading(), line, sq, ACROSS_REACH, MOST_ACROSS, turned, way_, kinds_,
                                ON_STAND_LINE | KEEP);
            sq = at >= 0 ? at : sq;
        }
        appendAfter(line, lineKinds, sq, way_, kinds_);
    }
    stand_ = stand;
    standSeen_ = true;
    hold_ = -1;
    plan_.build(way_, kinds_, true);
    taxi_.take(plan_);
    forks_.clear();
    forksOnPlan_.clear();
    forksWait_.clear();
}

void Follower::parkAt(const Network &net, int stand, Vec2 seen) {
    const StandWay &st = net.stands()[stand];
    float heading = st.heading;
    Vec2 at = parkPoint(st, heading);
    taxi_.place(at, heading, 0);
    plan_.clear();
    mode_ = Mode::PARKED;
    parkedAt_ = stand;
    parkedSeen_ = seen;
    haveParkedSeen_ = true;
    outBefore_ = farBefore_ = false;
    wander_ = 0;
    wanderN_ = 0;
    arriving_ = false;
    say("parked at stand", st.ref);
}

void Follower::onReport(const Network &net, const Input &in) {
    reportMs_ = in.reportMs;
    if (in.takeoff && rolling_) {
        return;  // rolling for take-off: the runway simulation has it, not its reports
    }
    Vec2 p = in.reportAt;
    float v = in.reportSpeed;
    // Further from the report before than either's speed could have taken it
    // since: a wild one, its place out by hundreds of metres - passed over,
    // unless the next two are as far, when it has gone there after all.
    if (prevMs_ != 0 && in.reportMs - prevMs_ >= 3000 && in.reportMs - prevMs_ < 120000) {
        float seconds = (in.reportMs - prevMs_) / 1000.0f, jumped = dist(p, prevAt_);
        if (jumped > WILD_JUMP && jumped / seconds > 2 * std::max(v, prevSpeed_) + WILD_SPEED && wildInRow_ < 2) {
            wildInRow_++;
            TRACE("    %.0fm from the report before in %.0fs: wild, passed over\n", jumped, seconds);
            return;
        }
    }
    wildInRow_ = 0;
    bool moving = v >= STILL && !isnan(in.reportHeading);
    float h = moving ? in.reportHeading : NAN;
    // Slower, but well away from where the report before had it: creeping
    // along - in along a stand's line, or pushed back - the way it went.
    float moved = prevMs_ != 0 && in.reportMs - prevMs_ < 30000 ? dist(p, prevAt_) : 0;
    if (!moving && v >= 0.8f && moved > CREPT) {
        // The way it went - its track, slow as it is, only if that agrees.
        moving = true;
        h = atan2f(p.y - prevAt_.y, p.x - prevAt_.x);
        if (!isnan(in.reportHeading) && cosf(in.reportHeading - h) > 0.7f) {
            h = in.reportHeading;
        }
    }
    speedingUp_ = prevMs_ != 0 && in.reportMs - prevMs_ < 30000 && v > prevSpeed_ + 0.5f && v < TAXI_SPEED;
    if (moving) {
        standingAt_ = -1;
    }
    Vec2 wasAt = prevAt_;
    uint32_t wasMs = prevMs_;
    float wasSpeed = prevSpeed_;
    prevAt_ = p;
    prevMs_ = in.reportMs;
    prevSpeed_ = v;
    int near = standNear(net, p, AT_STAND);
    // Standing near the stand it is making for, and not much nearer another:
    // that one - a noisy report not taking it to a neighbour it happens to be
    // a few metres nearer. A neighbour well nearer is where it is.
    if (stand_ >= 0 && near >= 0 && near != stand_) {
        float mine = dist(p, net.stands()[stand_].line.back()), theirs = dist(p, net.stands()[near].line.back());
        if (mine < AT_STAND && mine < theirs + STAND_SWITCH) {
            near = stand_;
        }
    }
    TRACE("  report (%.0f,%.0f) %.1fm/s, mode %d, shown (%.0f,%.0f) %.1fm/s, stand near %d\n", p.x, p.y, v, (int)mode_,
          taxi_.at().x, taxi_.at().y, taxi_.speed(), near);
    if (mode_ == Mode::PUSH) {
        return;  // the push finished first
    }
    if (mode_ == Mode::PARKED) {
        // Still there - its first report there where it is seen to be - or
        // gone from it: pushed back.
        if (parkedAt_ < 0) {
            return;
        }
        bool taxiing = v >= STILL && !isnan(in.reportHeading);
        if (!haveParkedSeen_ && !taxiing && dist(p, net.stands()[parkedAt_].line.back()) < AT_STAND) {
            parkedSeen_ = p;
            haveParkedSeen_ = true;
        } else {
            // Gone from where it was seen parked - out towards the taxilane,
            // along its line or onto the taxilane, as a pushback goes: taxiing
            // off, its speed says; or out there in two reports one after the
            // other, the second further out; or in one, a long way out - not
            // a report wandering about where it stands, as a noisy one can by
            // 30m with a knot or two on it, or one wild one.
            // How far out is out, by how far its reports have wandered while
            // it stood there.
            // Out: further from the stop than where it was seen, and not
            // ahead of the stop, towards the terminal - along its line to
            // the taxilane, or past the line's start and away.
            const StandWay &st = net.stands()[parkedAt_];
            Vec2 seen = haveParkedSeen_ ? parkedSeen_ : st.line.back();
            Vec2 stop = st.line.back();
            float gone = dist(p, seen);
            float beyond = (p.x - stop.x) * cosf(st.heading) + (p.y - stop.y) * sinf(st.heading);
            bool outwards = beyond < AHEAD_OF_STOP && dist(p, stop) > dist(seen, stop);
            float lineOff, lineHeading;
            nearestOn(st.line, p, lineOff, lineHeading);
            Place lane;
            bool wayOut = outwards && (lineOff < ON_LINE || net.snap(p, NAN, ON_LINE, lane));
            float wander = wanderN_ > 0 ? wander_ : WANDER;
            bool out = wayOut && gone > std::max(LEFT_STAND_GOING, 2.5f * wander);
            bool far = gone > LEFT_FAR;
            bool left = (taxiing && outwards && gone > LEFT_STAND_GOING) ||
                        (out && outBefore_ && gone > goneBefore_ + 3) ||
                        (wayOut && gone > std::max(LEFT_STAND, 3.5f * wander)) || (far && farBefore_);
            farBefore_ = far;
            if (!wayOut) {
                wanderN_ = std::min(wanderN_ + 1, 8);
                wander_ += (std::min(gone, MOST_WANDER) - wander_) / wanderN_;
            }
            outBefore_ = out;
            goneBefore_ = gone;
            if (left) {
                float parkedHeading;
                if (dist(taxi_.at(), parkPoint(st, parkedHeading)) < 15) {
                    pushBack(net, in);
                    if (taxiing || gone > LATE_PUSH) {
                        taxi_.pushSpeed = PUSH_LATE;  // seen late: catching up
                    }
                } else {
                    mode_ = Mode::FOLLOW;  // not shown on the stand after all: as its reports have it
                    parkedAt_ = -1;
                    reportMs_ = 0;
                }
            }
        }
        return;
    }
    if (leftStand_ >= 0 && dist(p, net.stands()[leftStand_].line.back()) > 100) {
        leftStand_ = -1;  // well away from it
    }
    Place onLine;
    bool onTaxilane = net.snap(p, NAN, ON_TAXILANE, onLine);
    // Departing, not just begun: standing near a stand's stop - holding
    // short on an apron's taxilane among them, as often as not - is at that
    // stand only right by it.
    if (near >= 0 && !arriving_ && stand_ < 0 && !justBegun_ &&
        dist(p, net.stands()[near].line.back()) > AT_STAND_DEPARTING) {
        near = -1;
    }
    if (!moving && near >= 0 && near != leftStand_ && !onTaxilane) {
        // Standing at a stand: there - taxied in, if on its way to it.
        const StandWay &st = net.stands()[near];
        // Shown there already, and not on its way in: where its first report
        // put it - by the stop, or between there and where it parks - parked
        // where it does, rather than turned about to it.
        float fromStop = dist(taxi_.at(), st.line.back());
        if ((justBegun_ && fromStop < BEGUN_PARKED && dist(taxi_.at(), p) < BEGUN_PARKED) ||
            (stand_ != near && fromStop < parkBack(st, noseWheel_, nose_) + 5)) {
            parkAt(net, near, p);
            return;
        }
        if (dist(taxi_.at(), st.line.back()) < 6000) {
            Place from;
            if (stand_ != near && net.snap(taxi_.at(), taxi_.heading(), 30, from)) {
                Rules rules;
                rules.runwayFactor = 3;
                net.search(from, searchHeading(net, from), 6000, rules);
                if (net.pathTo(st.entry, st.arrive, way_, &kinds_) && !turnsAbout(way_, taxi_.heading(), taxi_.speed())) {
                    startAt(way_, taxi_.at());
                    smoothStart(way_, kinds_, taxi_.heading(), taxi_.speed());
                    const airfield::List<Vec2> &line = lineInto(st);
                    for (size_t i = 1; i < line.size(); i++) {
                        way_.push_back(line[i]);
                        kinds_.push_back(ON_STAND_LINE);
                    }
                    stand_ = near;
                    standSeen_ = true;
                    hold_ = -1;
                    plan_.build(way_, kinds_, true);
                    taxi_.take(plan_);
                    forks_.clear();
                    forksOnPlan_.clear();
                    forksWait_.clear();
                }
            }
            if (stand_ != near && dist(taxi_.at(), st.line.back()) < 400) {
                // No way along the taxiways that doesn't start back the way it
                // came: a turn across onto its line, and in.
                const airfield::List<Vec2> &line = lineInto(st);
                static airfield::List<uint8_t> lineKinds;
                lineKinds.assign(line.size(), ON_STAND_LINE);
                float off, h;
                float sm = nearestOn(line, taxi_.at(), off, h);
                float sq = std::min(sm + 30, lengthOf(line)), turned;
                way_.clear();
                kinds_.clear();
                way_.push_back(taxi_.at());
                kinds_.push_back(ON_STAND_LINE | KEEP);
                float at = turnOnto(taxi_.at(), taxi_.heading(), line, sq, ACROSS_REACH, MOST_ACROSS, turned, way_,
                                    kinds_, ON_STAND_LINE | KEEP);
                sq = at >= 0 ? at : sq;
                appendAfter(line, lineKinds, sq, way_, kinds_);
                stand_ = near;
                standSeen_ = true;
                hold_ = -1;
                plan_.build(way_, kinds_, true);
                taxi_.take(plan_);
                forks_.clear();
                forksOnPlan_.clear();
                forksWait_.clear();
            }
            if (stand_ == near) {
                standSeen_ = true;
                // Its plan in to where it parks at that stand - not one that
                // ends short of it, where its last report put it on the
                // taxilane.
                const airfield::List<Vec2> &planned = plan_.points();
                float parkedHeading;
                if (planned.empty() || dist(planned.back(), parkPoint(st, parkedHeading)) > 5) {
                    planInto(net, near);
                }
                mode_ = Mode::OWN_WAY;
                say("taxiing in to stand", st.ref);
                return;
            }
        }
        standingCount_ = standingAt_ == near ? standingCount_ + 1 : 1;
        standingAt_ = near;
        if (dist(taxi_.at(), st.line.back()) < SNAP_PARK || (standingCount_ >= STANDING_PARK && taxi_.speed() < 0.5f)) {
            parkAt(net, near, p);  // there - or reported there again and again: there, however it got there
        } else {
            // No way there along the taxiways it can be seen to take - nor
            // put there, so far away: stopped, for later reports to bring it.
            stopSoon();
            keepUpFrom(0, 0, in.reportMs);
            mode_ = Mode::FOLLOW;
            say("standing");
        }
        return;
    }
    // On the way it is taking already - its plan reaching where the report
    // puts it, going the way the report has it going: kept to, in step with
    // the report from there.
    Place snapped;
    if (!plan_.empty() && (mode_ == Mode::FOLLOW || mode_ == Mode::OWN_WAY) && net.snap(p, h, REACH, snapped)) {
        // Its place on the taxiways - which of two ways parting it is on -
        // on the plan, near enough the way the plan goes there.
        float off, ph = 0;
        float sp = plan_.nearest(net.at(snapped), plan_.length(), off);
        plan_.pointAt(sp, ph);
        if (off < ON_PLAN && sp > taxi_.along() - 60 && (!moving || cosf(ph - h) > 0.7f) &&
            sp < plan_.length() - (moving ? 1.0f : -1.0f)) {
            keepUpFrom(sp, moving ? v : 0, in.reportMs);
            mode_ = Mode::FOLLOW;
            TRACE("    on its plan %.0fm along it, %.0fm off\n", sp, off);
            return;
        }
    }
    Place r;
    if ((arriving_ || stand_ >= 0) && !onTaxilane && onStandLine(net, p, moving, h, in)) {
        return;
    }
    if (!net.snap(p, h, REACH, r)) {
        if ((moving && v > OFF_NETWORK_FAST) || dist(taxi_.at(), p) > OFF_NETWORK_MOST) {
            // Going a taxiway's speed off every taxiway, or far off one from
            // where it is shown: a wild report - its place out by hundreds of
            // metres, as one now and then is - not one to follow onto the
            // grass.
            TRACE("    off the taxiways, going %.1fm/s, %.0fm away: wild, passed over\n", v, dist(taxi_.at(), p));
            prevAt_ = wasAt;
            prevMs_ = wasMs;
            prevSpeed_ = wasSpeed;
            return;
        }
        // Off the taxiways, in an apron's open space: straight to it.
        static airfield::List<Vec2> pts;
        static airfield::List<uint8_t> ks;
        pts.clear();
        ks.clear();
        pts.push_back(taxi_.at());
        ks.push_back((uint8_t)airfield::WayKind::TAXILANE);
        float to = moving ? h : atan2f(p.y - taxi_.at().y, p.x - taxi_.at().x);
        if (dist(taxi_.at(), p) > 3) {
            curve(taxi_.at(), taxi_.heading(), p, to, pts, ks);
            pts.push_back(p);
            ks.push_back((uint8_t)airfield::WayKind::TAXILANE);
        }
        float s0 = lengthOf(pts);
        if (moving) {
            pts.push_back({p.x + cosf(h) * v * CARRY_S, p.y + sinf(h) * v * CARRY_S});
            ks.push_back((uint8_t)airfield::WayKind::TAXILANE);
        }
        plan_.build(pts, ks, true);
        taxi_.take(plan_);
        forks_.clear();
        forksOnPlan_.clear();
        forksWait_.clear();
        keepUpFrom(s0, moving ? v : 0, in.reportMs);
        mode_ = Mode::FOLLOW;
        say("following its reports");
        return;
    }
    if (!moving) {
        way_.clear();
        kinds_.clear();
        way_.push_back(net.at(r));
        kinds_.push_back(net.onRunway(r) ? (uint8_t)airfield::WayKind::RUNWAY : (uint8_t)airfield::WayKind::TAXIWAY);
        stand_ = hold_ = -1;
    } else {
        Rules rules;
        rules.runwayFactor = 3;
        rules.back = false;  // going: on the way it is going, not back
        wayOn(net, r, h, rules, in);
    }
    join(net, r, h, in);
}

bool Follower::onStandLine(const Network &net, Vec2 p, bool moving, float heading, const Input &in) {
    // The stand whose line it is on - beyond its first few metres, which are
    // on the taxilane - going in along it, if going; the one it is making for,
    // if it is that one's.
    int best = -1;
    float bestOff = ON_LINE, reportAlong = 0;
    for (size_t i = 0; i < net.stands().size(); i++) {
        const StandWay &st = net.stands()[i];
        if (!st.ok || dist(p, st.line.back()) > dist(st.line.front(), st.line.back()) + ON_LINE) {
            continue;
        }
        float off, h;
        float s = nearestOn(st.line, p, off, h);
        if (moving && cosf(heading - h) < 0.7f) {
            continue;
        }
        if (s > 15 && (off < bestOff || ((int)i == stand_ && off < ON_LINE))) {
            bestOff = (int)i == stand_ ? -1 : off;
            best = (int)i;
            reportAlong = s;
        }
    }
    if (best < 0) {
        return false;
    }
    const StandWay &st = net.stands()[best];
    // In along its line as far as where it parks: a report beyond that - as
    // its aerial, ahead of its main wheels, can be - there.
    notPast(st);
    const airfield::List<Vec2> &line = lineInto(st);
    reportAlong = std::min(reportAlong, lengthOf(line));
    static airfield::List<Vec2> pts;
    static airfield::List<uint8_t> ks, lineKinds;
    lineKinds.assign(line.size(), ON_STAND_LINE);
    pts.clear();
    ks.clear();
    float off, h, s0;
    float sm = nearestOn(line, taxi_.at(), off, h);
    if (off < 6) {
        // On the line already.
        pts.push_back(taxi_.at());
        ks.push_back(ON_STAND_LINE);
        appendAfter(line, lineKinds, sm, pts, ks);
        easeOnto(pts, ks, taxi_.heading(), std::max(10.0f, std::min(30.0f, taxi_.speed() * 2.5f)));
        s0 = reportAlong - sm;
    } else if (off < 20 && sm < reportAlong) {
        // Beside it - by its start, turning in: onto it a little further on.
        float sq = std::min(sm + std::max(10.0f, off * 2), lengthOf(line));
        pts.push_back(taxi_.at());
        ks.push_back(ON_STAND_LINE | KEEP);
        sq = acrossOnto(line, sq, pts, ks);
        float before = lengthOf(pts);
        appendAfter(line, lineKinds, sq, pts, ks);
        s0 = before + reportAlong - sq;
    } else {
        // Along the taxiways to the line's start - unless that is the long way
        // round, its start just passed: then a tight turn across onto it.
        Place from;
        bool routed = false;
        if (net.snap(taxi_.at(), taxi_.heading(), 30, from)) {
            Rules rules;
            rules.runwayFactor = 3;
            rules.back = taxi_.speed() < 1;
            net.search(from, searchHeading(net, from), 3000, rules);
            routed = net.pathTo(st.entry, st.arrive, pts, &ks) &&
                     lengthOf(pts) < dist(taxi_.at(), st.line.front()) * 1.5f + 60 &&
                     !turnsAbout(pts, taxi_.heading(), taxi_.speed());
        }
        if (routed) {
            startAt(pts, taxi_.at());
            smoothStart(pts, ks, taxi_.heading(), taxi_.speed());
            s0 = lengthOf(pts) + reportAlong;
            appendAfter(line, lineKinds, 0, pts, ks);
        } else {
            pts.clear();
            ks.clear();
            float sq = std::min(std::max(sm, 0.0f) + 30, lengthOf(line));
            pts.push_back(taxi_.at());
            ks.push_back(ON_STAND_LINE | KEEP);
            sq = acrossOnto(line, sq, pts, ks);
            float before = lengthOf(pts);
            appendAfter(line, lineKinds, sq, pts, ks);
            s0 = before + reportAlong - sq;
        }
    }
    plan_.build(pts, ks, true);
    taxi_.take(plan_);
    forks_.clear();
    forksOnPlan_.clear();
    forksWait_.clear();
    keepUpFrom(s0, moving ? in.reportSpeed : 0, in.reportMs);
    stand_ = best;
    standSeen_ = true;
    hold_ = -1;
    way_.clear();
    kinds_.clear();
    appendAfter(line, lineKinds, reportAlong, way_, kinds_);
    mode_ = Mode::FOLLOW;
    say("taxiing in to stand", st.ref);
    TRACE("    on stand %s's line, %.0fm along it; plan %.0fm, report's place at %.0f\n", st.ref, reportAlong,
          plan_.length(), s0);
    return true;
}

void Follower::join(const Network &net, const Place &reported, float heading, const Input &in) {
    static airfield::List<Vec2> pts;
    static airfield::List<uint8_t> ks;
    pts.clear();
    ks.clear();
    bool moving = !isnan(heading);
    Vec2 m = taxi_.at();
    float mh = taxi_.heading();
    float wayLength = lengthOf(way_);
    float s0 = 0;
    bool joined = false;
    // On the way already - ahead of the report's place, carried on past it:
    // in its first stretch, as far as it could have gone since - not where
    // the way comes back by later.
    float off, wh = 0;
    float sm = way_.size() >= 2 ? nearestOn(way_, m, off, wh, std::max(150.0f, in.reportSpeed * CARRY_S)) : 0;
    if (way_.size() >= 2 && off < 6 && cosf(wh - mh) > 0.5f) {
        pts.push_back(m);
        ks.push_back(kinds_[0]);
        appendAfter(way_, kinds_, sm, pts, ks);
        easeOnto(pts, ks, mh, std::max(10.0f, std::min(30.0f, taxi_.speed() * 2.5f)));
        s0 = -sm;
        joined = true;
        TRACE("    join: on the way, %.0fm past the report\n", sm);
    }
    Place pm;
    if (!joined && net.snap(m, mh, 30, pm)) {
        Rules rules;
        rules.runwayFactor = 3;
        rules.back = taxi_.speed() < 1;
        net.search(pm, searchHeading(net, pm), 3000, rules);
        // Behind it: along the taxiways to the report's place, and on.
        float cost = net.costTo(reported, heading);
        TRACE("    join: from (%.0f,%.0f) on edge %d at %.0f, to the report's place on edge %d at %.0f: %.0fm, %.0fm straight\n",
              m.x, m.y, pm.edge, pm.s, reported.edge, reported.s, cost, dist(m, net.at(reported)));
        // Stopped, it can go the long way round to where it is reported -
        // there is no hurry, and no turn to be seen making; going, only a way
        // not much longer than straight there.
        float straight = dist(m, net.at(reported));
        float sane = taxi_.speed() < 1 ? straight * 3 + 400 : straight * 1.5f + 60;
        if (cost <= sane && net.pathTo(reported, heading, pts, &ks) &&
            !turnsAbout(pts, mh, taxi_.speed())) {
            startAt(pts, m);
            smoothStart(pts, ks, mh, taxi_.speed());
            s0 = lengthOf(pts);
            appendAfter(way_, kinds_, 0, pts, ks);
            joined = true;
            TRACE("    join: %.0fm along to the report's place\n", s0);
        }
        // Gone another way - by a little, just past where the ways part:
        // across to it, below; by more, round to the way further on.
        float aside = 1e30f, sh = 0;
        if (way_.size() >= 2) {
            nearestOn(way_, m, aside, sh);
        }
        for (float ahead : {60.0f, 120.0f, 200.0f, 320.0f, 500.0f}) {
            if (joined || !moving || ahead > wayLength || aside < ACROSS) {
                break;
            }
            float qh = 0;
            Vec2 q = along(way_, ahead, qh);
            Place pq;
            if (!net.snap(q, qh, 5, pq) || net.costTo(pq, qh) > dist(m, q) * 1.4f + 80 ||
                !net.pathTo(pq, qh, pts, &ks) || turnsAbout(pts, mh, taxi_.speed())) {
                continue;
            }
            startAt(pts, m);
            smoothStart(pts, ks, mh, taxi_.speed());
            s0 = lengthOf(pts) - ahead;
            appendAfter(way_, kinds_, ahead, pts, ks);
            joined = true;
            TRACE("    join: round to the way %.0fm on\n", ahead);
        }
    }
    if (!joined && !moving) {
        // Standing somewhere it can't be reached from where it is shown - behind
        // it: it stops, as soon as it can.
        stopSoon();
        keepUpFrom(0, 0, in.reportMs);
        mode_ = Mode::FOLLOW;
        say("standing");
        TRACE("    join: standing behind - stopped\n");
        return;
    }
    if (!joined) {
        TRACE("    join: across\n");
        // Across to the way, ahead of where the report puts it now - far
        // enough along it to be ahead of where it is shown, not behind: no
        // loop back to it.
        float t = std::min(CARRY_S, (in.ms - in.reportMs) / 1000.0f);
        float ahead = std::min(wayLength, in.reportSpeed * t + std::max(40.0f, taxi_.speed() * 5));
        float qh = 0;
        Vec2 q = along(way_, ahead, qh);
        for (float further = ahead; further <= wayLength; further += 20) {
            float fh = 0;
            Vec2 f = along(way_, further, fh);
            float d = dist(m, f);
            if (d > 1 && ((f.x - m.x) * cosf(mh) + (f.y - m.y) * sinf(mh)) / d > 0.26f) {
                ahead = further;
                q = f;
                qh = fh;
                break;
            }
        }
        pts.push_back(m);
        ks.push_back((uint8_t)airfield::WayKind::TAXILANE | KEEP);
        float turned = 1e30f;
        float at = dist(m, q) <= ACROSS_MOST ? turnOnto(m, mh, way_, ahead, ACROSS_REACH, MOST_ACROSS, turned, pts, ks,
                                                        (uint8_t)airfield::WayKind::TAXILANE | KEEP)
                                             : -1;
        if (at < 0 || turned > MOST_ACROSS) {
            // Too far across to it, or no way onto it but round a circle: on
            // by itself along the taxiways to where it is making for, its
            // reports to bring it back to them.
            TRACE("    join: too far across, or round a circle: its own way\n");
            ownWay(net, in);
            return;
        }
        ks.back() = (kinds_.empty() ? (uint8_t)airfield::WayKind::TAXIWAY : kinds_[0]) | KEEP;
        s0 = lengthOf(pts) - at;
        appendAfter(way_, kinds_, at, pts, ks);
    }
    plan_.build(pts, ks, true);
    taxi_.take(plan_);
    placeForks();
    keepUpFrom(s0, moving ? in.reportSpeed : 0, in.reportMs);
    mode_ = Mode::FOLLOW;
    TRACE("    plan %.0fm, at %.0f, report's place at %.0f, way %.0fm to %s %d\n", plan_.length(), taxi_.along(), s0,
          wayLength, stand_ >= 0 ? "stand" : hold_ >= 0 ? "hold" : "-", stand_ >= 0 ? stand_ : hold_);
    if (stand_ >= 0) {
        say("taxiing to stand", net.stands()[stand_].ref);
    } else if (hold_ >= 0) {
        say(lineUp_ ? "lining up from" : "taxiing to hold", net.holds()[hold_].ref);
    } else {
        say(moving ? "taxiing" : "standing");
    }
}

void Follower::ownWay(const Network &net, const Input &in) {
    Place from;
    if (net.snap(taxi_.at(), taxi_.heading(), 30, from) || net.snap(taxi_.at(), NAN, OFF_NETWORK_MOST, from)) {
        Rules rules;
        rules.runwayFactor = 3;
        rules.back = taxi_.speed() < 1;
        wayOn(net, from, searchHeading(net, from), rules, in);
        if (!way_.empty() && dist(way_[0], taxi_.at()) >= 30) {
            // Shown off the taxiways - back onto them from where it is.
            way_.insert(way_.begin(), taxi_.at());
            kinds_.insert(kinds_.begin(), (uint8_t)airfield::WayKind::TAXILANE);
        }
        startAt(way_, taxi_.at());
        smoothStart(way_, kinds_, taxi_.heading(), taxi_.speed());
        plan_.build(way_, kinds_, true);
        taxi_.take(plan_);
        placeForks();
    }
    mode_ = Mode::OWN_WAY;
    if (stand_ >= 0) {
        say("taxiing to stand", net.stands()[stand_].ref);
    } else if (hold_ >= 0) {
        say("taxiing to hold", net.holds()[hold_].ref);
    }
}

void Follower::pushBack(const Network &net, const Input &in) {
    const StandWay &st = net.stands()[parkedAt_];
    static airfield::List<Vec2> pts;
    static airfield::List<uint8_t> ks;
    pts.clear();
    ks.clear();
    // Back along its stand's line towards the taxilane first - which way
    // along that, as its reports by then say: see pushOn(). Not quite to it:
    // the room left for the turn onto it, its tail swung round.
    const airfield::List<Vec2> &line = lineInto(st);
    float total = lengthOf(line), stop = std::max(0.0f, total - std::min(PUSH_TURN, total * 0.4f)), gone = 0;
    pts.push_back(line.back());
    ks.push_back(ON_STAND_LINE);
    for (size_t i = line.size() - 1; i-- > 0 && gone < stop;) {
        float d = dist(line[i + 1], line[i]);
        if (gone + d > stop && d > 1e-3f) {
            float k = (stop - gone) / d;
            pts.push_back({line[i + 1].x + (line[i].x - line[i + 1].x) * k, line[i + 1].y + (line[i].y - line[i + 1].y) * k});
        } else {
            pts.push_back(line[i]);
        }
        ks.push_back(ON_STAND_LINE);
        gone += d;
    }
    // From where it is shown - at the stop, or near it - not put back there.
    startAt(pts, taxi_.at());
    plan_.build(pts, ks, false);
    taxi_.take(plan_);
    taxi_.stop();
    taxi_.pushing = true;
    taxi_.pushSpeed = PUSH_SPEED;
    taxi_.landing = false;
    pushFrom_ = parkedAt_;
    pushOnward_ = false;
    leftStand_ = parkedAt_;
    parkedAt_ = -1;
    arriving_ = false;
    way_.clear();
    kinds_.clear();
    forks_.clear();
    forksOnPlan_.clear();
    forksWait_.clear();
    stand_ = hold_ = -1;
    mode_ = Mode::PUSH;
    say("pushing back from stand", st.ref);
    if (plan_.empty()) {
        pushOn(net, in);
    }
}

void Follower::pushOn(const Network &net, const Input &in) {
    const StandWay &st = net.stands()[pushFrom_];
    // On along the taxilane, tail first: away from the way it will go - the
    // way its report has it going, if it is taxiing; the way it is being
    // pushed, if its reports have it out on the taxilane already; or away
    // from the way to the holding points.
    float lane = net.heading(st.entry);
    Vec2 entry = net.at(st.entry);
    bool fresh = in.reportMs != 0 && (int32_t)(in.ms - in.reportMs) < 20000;
    float away = NAN;
    if (fresh && in.reportSpeed >= STILL && !isnan(in.reportHeading)) {
        away = in.reportHeading + (float)M_PI;  // taxiing already, nose first
    } else if (fresh && dist(in.reportAt, entry) > 10) {
        away = atan2f(in.reportAt.y - entry.y, in.reportAt.x - entry.x);
    } else {
        Rules rules;
        rules.runwayFactor = 3;
        wayOn(net, st.entry, NAN, rules, in);
        if (way_.size() >= 2) {
            away = atan2f(way_[1].y - way_[0].y, way_[1].x - way_[0].x) + (float)M_PI;
        }
        way_.clear();
        kinds_.clear();
        forks_.clear();
        stand_ = hold_ = -1;
    }
    float push = isnan(away) || cosf(away - lane) >= 0 ? lane : lane + (float)M_PI;
    static airfield::List<Vec2> on;
    static airfield::List<uint8_t> onKinds;
    net.ahead(st.entry, push, PUSH_ALONG, on, onKinds);
    if (lengthOf(on) < PUSH_ALONG / 2) {
        // The taxilane ends there that way: the other.
        static airfield::List<Vec2> other;
        static airfield::List<uint8_t> otherKinds;
        net.ahead(st.entry, push + (float)M_PI, PUSH_ALONG, other, otherKinds);
        if (lengthOf(other) > lengthOf(on)) {
            on = other;
            onKinds = otherKinds;
        }
    }
    // From where it is, short of the taxilane: the turn onto it rounded.
    if (on.empty() || dist(on[0], taxi_.at()) > 1) {
        on.insert(on.begin(), taxi_.at());
        onKinds.insert(onKinds.begin(), ON_STAND_LINE);
    }
    plan_.build(on, onKinds, true);
    taxi_.take(plan_);
    pushOnward_ = true;
}

void Follower::update(const Network &net, const Input &in) {
    if (!active_ || net.empty()) {
        return;
    }
    noseWheel_ = in.noseWheel;
    nose_ = in.nose;
    if (in.reportMs != 0 && in.reportMs != reportMs_) {
        onReport(net, in);
    }
    justBegun_ = false;
    if (!in.takeoff) {
        rolling_ = false;
        taxi_.takingOff = false;
    }
    if (in.takeoff && mode_ != Mode::PUSH) {
        // The take-off simulated - lined up, its roll begun: in step with it,
        // along the runway the way it is going - its roll's plan the length
        // of the runway, not stopping where it lined up.
        float off = 1e30f;
        float s = plan_.empty() ? 0 : plan_.nearest(in.simAt, plan_.length(), off);
        bool onIt = !plan_.empty() && off < 8;
        if (onIt) {
            float h = 0;
            plan_.pointAt(s, h);
            onIt = cosf(h - in.simHeading) > 0.7f || in.simSpeed < 1;
        }
        Place mine;
        if (!rolling_ && in.simSpeed > 1 && net.snap(taxi_.at(), in.simHeading, 30, mine, true)) {
            static airfield::List<Vec2> roll;
            static airfield::List<uint8_t> rollKinds;
            net.ahead(mine, in.simHeading, 4000, roll, rollKinds);
            for (uint8_t &k : rollKinds) {
                k = (uint8_t)airfield::WayKind::RUNWAY;  // a runway's speeds, all of it
            }
            startAt(roll, taxi_.at());
            taxi_.takingOff = true;
            plan_.build(roll, rollKinds, false);
            taxi_.take(plan_);
            forks_.clear();
            forksOnPlan_.clear();
            forksWait_.clear();
            rolling_ = true;
            onIt = true;
            s = plan_.nearest(in.simAt, plan_.length(), off);
        }
        Place r;
        if (!onIt && (int32_t)(in.ms - takeoffJoinMs_) >= 1000 && net.snap(in.simAt, in.simHeading, REACH, r)) {
            takeoffJoinMs_ = in.ms;
            net.ahead(r, in.simHeading, 3000, way_, kinds_);
            forks_.clear();
            stand_ = hold_ = -1;
            Input given = in;
            given.reportMs = in.ms;
            given.reportAt = in.simAt;
            given.reportHeading = in.simHeading;
            given.reportSpeed = std::max(in.simSpeed, STILL);
            join(net, r, in.simHeading, given);
            s = plan_.nearest(in.simAt, plan_.length(), off);
        }
        keepUpFrom(s, in.simSpeed, in.ms);
        mode_ = Mode::FOLLOW;
        say("lining up");
    }
    bool lost = in.reportMs == 0 || (int32_t)(in.ms - in.reportMs) > (int32_t)LOST_MS || in.lost;
    if (mode_ == Mode::FOLLOW && lost && !in.takeoff) {
        ownWay(net, in);  // its reports gone
    }
    if (mode_ == Mode::PARKED && parkedAt_ >= 0) {
        // Settled onto where it parks - as it creeps the last of the way in,
        // its nose wheel straightening - a few degrees off, having come in
        // along a line that curves to the stop.
        float heading = net.stands()[parkedAt_].heading;
        Vec2 park = parkPoint(net.stands()[parkedAt_], heading);
        float turn = remainderf(heading - taxi_.heading(), 2 * (float)M_PI), off = dist(taxi_.at(), park);
        if (off < SETTLE && (fabsf(turn) > 1e-3f || off > 0.01f)) {
            float most = SETTLE_TURN * in.dt, step = std::min(off, SETTLE_SPEED * in.dt);
            Vec2 at = off > 0.01f ? Vec2{taxi_.at().x + (park.x - taxi_.at().x) * step / off,
                                         taxi_.at().y + (park.y - taxi_.at().y) * step / off}
                                  : park;
            taxi_.place(at, taxi_.heading() + std::max(-most, std::min(most, turn)), 0);
        }
    }
    float want = 0;
    switch (mode_) {
    case Mode::FOLLOW: {
        // Where the report puts it now - carried on from its place along the
        // plan at its speed, slowing for the turns as it would, for no more
        // than CARRY_S - and it kept up with, or let catch up.
        if ((int32_t)(in.ms - syncMs_) < (int32_t)(CARRY_S * 1000)) {
            float cap = ghostS_ < 0 ? TAXI_SPEED : plan_.fastestAt(ghostS_, TAXI_BRAKE);
            float to = speedingUp_ ? cap : std::min(syncSpeed_, cap);
            ghostSpeed_ = to > ghostSpeed_ ? std::min(to, ghostSpeed_ + TAXI_ACCEL * in.dt)
                                           : std::max(to, ghostSpeed_ - 2 * TAXI_BRAKE * in.dt);
            ghostS_ = std::min(ghostS_ + ghostSpeed_ * in.dt, std::max(ghostS_, plan_.length()));
        }
        float behind = ghostS_ - taxi_.along();
        want = ghostSpeed_ + KEEP_UP * behind;
        // Well behind - a late start, a way round - catching up harder.
        float most = behind > 60 ? std::max(ghostSpeed_ * 1.5f, ghostSpeed_ + 5)
                                 : std::max(ghostSpeed_ * 1.3f, ghostSpeed_ + 3);
        want = std::max(0.0f, std::min(want, most));
        if (behind > 2 && want < 1.5f) {
            want = 1.5f;
        }
        // Behind, the way's speeds - guesses - not keeping it from where its
        // reports put it: up to 30 knots on a straight, 35 well behind.
        taxi_.hurry = behind > 15 ? std::min(behind > 60 ? 1.45f : 1.25f, 1 + (behind - 15) / 100) : 1;
        // Not past the first fork beyond the report's place, until a report
        // says which way it went there.
        for (size_t i = 0; i < forksOnPlan_.size(); i++) {
            float f = forksOnPlan_[i];
            if (f + FORK_PAST > syncS_) {
                float wait = i < forksWait_.size() ? forksWait_[i] : f;
                float limit = taxi_.along() > wait + 1 ? f + FORK_ROOM : wait;
                float room = std::max(0.0f, limit - taxi_.along());
                want = std::min(want, sqrtf(2 * FORK_BRAKE * room));
                break;
            }
        }
        break;
    }
    case Mode::OWN_WAY:
        want = 1e9f;
        if (stand_ >= 0 && !standSeen_ && arriving_ && !in.lost && in.reportMs != 0 &&
            (int32_t)(in.ms - in.reportMs) < (int32_t)STAND_WAIT_MS) {
            float off;
            float s = plan_.nearest(net.at(net.stands()[stand_].entry), plan_.length(), off);
            float wait = std::max(0.0f, plan_.turnStart(s, FORK_LEAD) - 2);
            if (off < FORK_NEAR && taxi_.along() < s + FORK_ROOM) {
                // Past where it waits - its plan made too near to stop by
                // then - as soon as it can.
                want = taxi_.along() <= wait + 1 ? sqrtf(2 * FORK_BRAKE * std::max(0.0f, wait - taxi_.along())) : 0;
            }
        }
        break;
    case Mode::PUSH:
        want = taxi_.pushSpeed;
        break;
    case Mode::PARKED:
    case Mode::HELD:
        want = 0;
        break;
    }
    if (mode_ != Mode::FOLLOW) {
        taxi_.hurry = 1;
    }
#ifdef GROUND_TRACE
    {
        static uint32_t lastMs = 0;
        if (in.ms - lastMs >= 1000) {
            lastMs = in.ms;
            printf("    [mode %d along %.0f/%.0f ghost %.0f@%.1f sync %.0f want %.1f forks", (int)mode_, taxi_.along(),
                   plan_.length(), ghostS_, ghostSpeed_, syncS_, want);
            for (size_t i = 0; i < forksOnPlan_.size(); i++) {
                printf(" %.0f/%.0f", forksOnPlan_[i], i < forksWait_.size() ? forksWait_[i] : -1.0f);
            }
            printf("]\n");
        }
    }
#endif
    taxi_.step(in.dt, want);
    if (mode_ == Mode::PUSH && !pushOnward_ && taxi_.atEnd()) {
        pushOn(net, in);  // out of its stand: on along the taxilane
    } else if (taxi_.arrived()) {
        float parkedHeading;
        if (mode_ == Mode::OWN_WAY && stand_ >= 0 && dist(taxi_.at(), parkPoint(net.stands()[stand_], parkedHeading)) > 10) {
            planInto(net, stand_);  // stopped short of it: on in
        } else if (mode_ == Mode::OWN_WAY && stand_ >= 0) {
            mode_ = Mode::PARKED;
            parkedAt_ = stand_;
            haveParkedSeen_ = false;
            outBefore_ = farBefore_ = false;
            wander_ = 0;
            wanderN_ = 0;
            arriving_ = false;
            say("parked at stand", net.stands()[stand_].ref);
        } else if (mode_ == Mode::OWN_WAY && hold_ >= 0) {
            mode_ = Mode::HELD;
            say(lineUp_ ? "lined up, from" : "holding at", net.holds()[hold_].ref);
        } else if (mode_ == Mode::OWN_WAY && (int32_t)(in.ms - ownWayMs_) >= (int32_t)OWN_WAY_AGAIN_MS) {
            // By itself, and nowhere to go from where it was: stopped now,
            // free to turn about - again, for a stand or a hold that way.
            ownWayMs_ = in.ms;
            ownWay(net, in);
        } else if (mode_ == Mode::PUSH) {
            taxi_.pushing = false;
            mode_ = Mode::FOLLOW;
            plan_.clear();
            taxi_.take(plan_);
            keepUpFrom(0, 0, in.ms);
            reportMs_ = 0;  // the latest report, taken afresh
            say("pushed back");
        }
    }
}

}  // namespace ground
