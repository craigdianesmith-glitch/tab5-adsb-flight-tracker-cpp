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
// Slower than STILL, but this far from the report before: moving.
constexpr float CREPT = 10;
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
// Between reports, carried on from the last at its speed - slowing for the
// turns ahead as it would - for no more than this.
constexpr float CARRY_S = 20;
// In step: going faster or slower than the reports' speed by this much a
// second for each metre behind or ahead.
constexpr float KEEP_UP = 0.15f;
// Pushed back along the taxilane, tail first, this far past its stand's line.
constexpr float PUSH_ALONG = 35;
// Parked, reported this far from where it was seen there - and nearer the
// taxilane - it has left; this far, if its speed says it is going.
constexpr float LEFT_STAND = 45, LEFT_STAND_GOING = 15;
// A stand with another aircraft standing this near its stop is taken.
constexpr float TAKEN = 25;
// Holding points other than at a runway's end - crossings - count as this
// much further: a departure is making for the end.
constexpr float NOT_AN_END = 1500;
// Where it is making for is kept to unless another is this much nearer.
constexpr float STICK = 150;
// A departure this near a runway's end hold, going on, is lining up: this
// far down the runway.
constexpr float AT_HOLD = 15, LINED_UP_IN = 60;
// A stand away from the terminals - remote, cargo's, the flying club's -
// counts as this much further, landed, where the airport has stands at them.
constexpr float REMOTE = 800;
// Gone a different way from the way its report has it going, but no further
// from it than this: across to it, smoothly, rather than round by the
// taxiways.
constexpr float ACROSS = 25;

float dist(Vec2 a, Vec2 b) { return hypotf(b.x - a.x, b.y - a.y); }

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
bool turnAcross(Vec2 p0, float h0, Vec2 p1, float h1, float radius, airfield::List<Vec2> &out,
                airfield::List<uint8_t> &kinds, uint8_t kind) {
    if (dist(p0, p1) < 1) {
        return false;
    }
    // Each side's turning circle's middle: left, s = 1; right, s = -1.
    auto middle = [&](Vec2 p, float h, float s) { return Vec2{p.x - s * radius * sinf(h), p.y + s * radius * cosf(h)}; };
    auto wrap = [](float a) {
        a = fmodf(a, 2 * (float)M_PI);
        return a < 0 ? a + 2 * (float)M_PI : a;
    };
    float best = 1e30f, bestS0 = 1, bestS1 = 1, bestPhi = 0;
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
            }
        }
    }
    if (best >= 1e29f) {
        return false;
    }
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
    float to = std::min(lengthOf(path), std::max(40.0f, reach * 2)), qh = 0;
    Vec2 q = along(path, to, qh);
    static airfield::List<Vec2> turned;
    static airfield::List<uint8_t> turnedKinds;
    turned.clear();
    turnedKinds.clear();
    turned.push_back(path[0]);
    turnedKinds.push_back(kinds[0] | KEEP);
    turnAcross(path[0], heading, q, qh, TURN_RADIUS, turned, turnedKinds, kinds[0] | KEEP);
    turned.push_back(q);
    turnedKinds.push_back(kinds[0] | KEEP);
    appendAfter(path, kinds, to, turned, turnedKinds);
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

void Follower::begin(const Network &net, Vec2 at, float heading, float speed, bool landed, const Input &in) {
    active_ = true;
    taxi_.place(at, heading, speed);
    taxi_.landing = landed;
    plan_.clear();
    way_.clear();
    kinds_.clear();
    stand_ = hold_ = parkedAt_ = leftStand_ = -1;
    haveParkedSeen_ = false;
    prevMs_ = 0;
    rolling_ = false;
    arriving_ = landed || in.arrived;
    mode_ = Mode::FOLLOW;
    reportMs_ = 0;  // the report there is, taken afresh
    say("following its reports");
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
        plan_.build(way_, kinds_, true);
        taxi_.take(plan_);
        placeForks();
    }
    mode_ = Mode::OWN_WAY;
    say(stand_ >= 0 ? "rolling out, for stand" : "rolling out", stand_ >= 0 ? net.stands()[stand_].ref : nullptr);
}

void Follower::wayOn(const Network &net, const Place &from, float heading, const Rules &rules, const Input &in) {
    int wasStand = stand_, wasHold = hold_;
    way_.clear();
    kinds_.clear();
    stand_ = hold_ = -1;
    lineUp_ = false;
    net.search(from, heading, 8000, rules);
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
            float cost = net.costTo(h.at, h.arrive) + (h.end ? 0 : NOT_AN_END);
            if ((int)i == wasHold) {
                was = cost;
            }
            if (cost < best) {
                best = cost;
                hold_ = (int)i;
            }
        }
        // At a runway's end hold already, going on: lining up - onto the
        // runway, turning down its longer way.
        if (hold_ >= 0 && net.holds()[hold_].end && best < AT_HOLD && !isnan(heading) && lineUp(net, hold_)) {
            return;
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
    if (stand_ >= 0) {
        const StandWay &st = net.stands()[stand_];
        if (net.pathTo(st.entry, st.arrive, way_, &kinds_, &at)) {
            for (float f : at) {
                float h;
                forks_.push_back(along(way_, f, h));
            }
            forks_.push_back(way_.back());  // in to the stand, or on past it
            for (size_t i = 1; i < st.line.size(); i++) {
                way_.push_back(st.line[i]);
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
    for (const Vec2 &f : forks_) {
        float off;
        float s = plan_.nearest(f, plan_.length(), off);
        if (off < FORK_NEAR) {
            forksOnPlan_.push_back(s);
        }
    }
    std::sort(forksOnPlan_.begin(), forksOnPlan_.end());
}

bool Follower::lineUp(const Network &net, int hold) {
    // Where the way on from the hold - the way it faces - meets the runway.
    const HoldWay &h = net.holds()[hold];
    static airfield::List<Vec2> on;
    static airfield::List<uint8_t> onKinds;
    net.ahead(h.at, h.arrive, 300, on, onKinds);
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
    static airfield::List<uint8_t> lineKinds;
    lineKinds.assign(st.line.size(), ON_STAND_LINE);
    way_.clear();
    kinds_.clear();
    float off, h;
    float sm = nearestOn(st.line, taxi_.at(), off, h);
    Place from;
    bool routed = false;
    if (off >= 6 && net.snap(taxi_.at(), taxi_.heading(), 30, from)) {
        Rules rules;
        rules.runwayFactor = 3;
        rules.back = taxi_.speed() < 1;
        net.search(from, taxi_.heading(), 6000, rules);
        routed = net.pathTo(st.entry, st.arrive, way_, &kinds_) && !turnsAbout(way_, taxi_.heading(), taxi_.speed());
        if (routed) {
            startAt(way_, taxi_.at());
            smoothStart(way_, kinds_, taxi_.heading(), taxi_.speed());
            appendAfter(st.line, lineKinds, 0, way_, kinds_);
        }
    }
    if (!routed) {
        // On its line already, or beside it: onto it a little further in.
        way_.clear();
        kinds_.clear();
        way_.push_back(taxi_.at());
        kinds_.push_back(ON_STAND_LINE | KEEP);
        float sq = std::min(sm + (off < 6 ? 0 : 30), lengthOf(st.line)), qh = 0;
        Vec2 q = along(st.line, sq, qh);
        if (off >= 6) {
            turnAcross(taxi_.at(), taxi_.heading(), q, qh, TURN_RADIUS, way_, kinds_, ON_STAND_LINE | KEEP);
            way_.push_back(q);
            kinds_.push_back(ON_STAND_LINE | KEEP);
        }
        appendAfter(st.line, lineKinds, sq, way_, kinds_);
    }
    stand_ = stand;
    hold_ = -1;
    plan_.build(way_, kinds_, true);
    taxi_.take(plan_);
    forks_.clear();
    forksOnPlan_.clear();
}

void Follower::parkAt(const Network &net, int stand, Vec2 seen) {
    const StandWay &st = net.stands()[stand];
    taxi_.place(st.line.back(), st.heading, 0);
    plan_.clear();
    mode_ = Mode::PARKED;
    parkedAt_ = stand;
    parkedSeen_ = seen;
    haveParkedSeen_ = true;
    arriving_ = false;
    say("parked at stand", st.ref);
}

void Follower::onReport(const Network &net, const Input &in) {
    reportMs_ = in.reportMs;
    Vec2 p = in.reportAt;
    float v = in.reportSpeed;
    bool moving = v >= STILL && !isnan(in.reportHeading);
    float h = moving ? in.reportHeading : NAN;
    // Slower, but well away from where the report before had it: creeping
    // along - in along a stand's line, or pushed back - the way it went.
    float moved = prevMs_ != 0 && in.reportMs - prevMs_ < 30000 ? dist(p, prevAt_) : 0;
    if (!moving && v >= 0.8f && moved > CREPT) {
        moving = true;
        h = !isnan(in.reportHeading) ? in.reportHeading : atan2f(p.y - prevAt_.y, p.x - prevAt_.x);
    }
    speedingUp_ = prevMs_ != 0 && in.reportMs - prevMs_ < 30000 && v > prevSpeed_ + 0.5f && v < TAXI_SPEED;
    prevAt_ = p;
    prevMs_ = in.reportMs;
    prevSpeed_ = v;
    int near = standNear(net, p, AT_STAND);
    // Standing near the stand it is making for, and not much nearer another:
    // that one - a noisy report not taking it to a neighbour it happens to be
    // a few metres nearer. A neighbour well nearer is where it is.
    if (stand_ >= 0 && near >= 0 && near != stand_) {
        float mine = dist(p, net.stands()[stand_].line.back()), theirs = dist(p, net.stands()[near].line.back());
        if (mine < AT_STAND && mine < theirs + 12) {
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
        if (!haveParkedSeen_ && !moving && dist(p, net.stands()[parkedAt_].line.back()) < AT_STAND) {
            parkedSeen_ = p;
            haveParkedSeen_ = true;
        } else {
            // Gone from where it was seen parked - well beyond the few metres a
            // report wanders by - out towards the taxilane.
            // Its speed says it is going, and it has gone a little; or it
            // has gone a long way, whatever its speed says - not a report
            // wandering about where it stands, as a noisy one can by 30m.
            const StandWay &st = net.stands()[parkedAt_];
            Vec2 seen = haveParkedSeen_ ? parkedSeen_ : st.line.back();
            float gone = dist(p, seen);
            bool outwards = dist(p, st.line.front()) < dist(seen, st.line.front());
            if (moving || (outwards && ((v >= 0.8f && gone > LEFT_STAND_GOING) || gone > LEFT_STAND))) {
                if (dist(taxi_.at(), st.line.back()) < 15) {
                    pushBack(net, in);
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
    if (!moving && near >= 0 && near != leftStand_ && !onTaxilane) {
        // Standing at a stand: there - taxied in, if on its way to it.
        const StandWay &st = net.stands()[near];
        if (dist(taxi_.at(), st.line.back()) < 3000) {
            Place from;
            if (stand_ != near && net.snap(taxi_.at(), taxi_.heading(), 30, from)) {
                Rules rules;
                rules.runwayFactor = 3;
                net.search(from, taxi_.heading(), 6000, rules);
                if (net.pathTo(st.entry, st.arrive, way_, &kinds_) && !turnsAbout(way_, taxi_.heading(), taxi_.speed())) {
                    startAt(way_, taxi_.at());
                    easeOnto(way_, kinds_, taxi_.heading(), std::max(10.0f, std::min(30.0f, taxi_.speed() * 2.5f)));
                    for (size_t i = 1; i < st.line.size(); i++) {
                        way_.push_back(st.line[i]);
                        kinds_.push_back(ON_STAND_LINE);
                    }
                    stand_ = near;
                    hold_ = -1;
                    plan_.build(way_, kinds_, true);
                    taxi_.take(plan_);
                    forks_.clear();
                    forksOnPlan_.clear();
                }
            }
            if (stand_ != near && dist(taxi_.at(), st.line.back()) < 400) {
                // No way along the taxiways that doesn't start back the way it
                // came: a turn across onto its line, and in.
                static airfield::List<uint8_t> lineKinds;
                lineKinds.assign(st.line.size(), ON_STAND_LINE);
                float off, h;
                float sm = nearestOn(st.line, taxi_.at(), off, h);
                float sq = std::min(sm + 30, lengthOf(st.line)), qh = 0;
                Vec2 q = along(st.line, sq, qh);
                way_.clear();
                kinds_.clear();
                way_.push_back(taxi_.at());
                kinds_.push_back(ON_STAND_LINE | KEEP);
                turnAcross(taxi_.at(), taxi_.heading(), q, qh, TURN_RADIUS, way_, kinds_, ON_STAND_LINE | KEEP);
                way_.push_back(q);
                kinds_.push_back(ON_STAND_LINE | KEEP);
                appendAfter(st.line, lineKinds, sq, way_, kinds_);
                stand_ = near;
                hold_ = -1;
                plan_.build(way_, kinds_, true);
                taxi_.take(plan_);
                forks_.clear();
                forksOnPlan_.clear();
            }
            if (stand_ == near) {
                // Its plan in to that stand's stop - not one that ends short
                // of it, where its last report put it on the taxilane.
                const airfield::List<Vec2> &planned = plan_.points();
                if (planned.empty() || dist(planned.back(), st.line.back()) > 5) {
                    planInto(net, near);
                }
                mode_ = Mode::OWN_WAY;
                say("taxiing in to stand", st.ref);
                return;
            }
        }
        parkAt(net, near, p);
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
        wayOn(net, r, h, rules, in);
    }
    join(net, r, h, in);
    mode_ = Mode::FOLLOW;
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
    static airfield::List<Vec2> pts;
    static airfield::List<uint8_t> ks, lineKinds;
    lineKinds.assign(st.line.size(), ON_STAND_LINE);
    pts.clear();
    ks.clear();
    float off, h, s0;
    float sm = nearestOn(st.line, taxi_.at(), off, h);
    if (off < 6) {
        // On the line already.
        pts.push_back(taxi_.at());
        ks.push_back(ON_STAND_LINE);
        appendAfter(st.line, lineKinds, sm, pts, ks);
        easeOnto(pts, ks, taxi_.heading(), std::max(10.0f, std::min(30.0f, taxi_.speed() * 2.5f)));
        s0 = reportAlong - sm;
    } else if (off < 20 && sm < reportAlong) {
        // Beside it - by its start, turning in: onto it a little further on.
        float sq = std::min(sm + std::max(10.0f, off * 2), lengthOf(st.line)), qh = 0;
        Vec2 q = along(st.line, sq, qh);
        pts.push_back(taxi_.at());
        ks.push_back(ON_STAND_LINE | KEEP);
        turnAcross(taxi_.at(), taxi_.heading(), q, qh, TURN_RADIUS, pts, ks, ON_STAND_LINE | KEEP);
        float before = lengthOf(pts) + dist(pts.back(), q);
        pts.push_back(q);
        ks.push_back(ON_STAND_LINE | KEEP);
        appendAfter(st.line, lineKinds, sq, pts, ks);
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
            net.search(from, taxi_.heading(), 3000, rules);
            routed = net.pathTo(st.entry, st.arrive, pts, &ks) &&
                     lengthOf(pts) < dist(taxi_.at(), st.line.front()) * 1.5f + 60 &&
                     !turnsAbout(pts, taxi_.heading(), taxi_.speed());
        }
        if (routed) {
            startAt(pts, taxi_.at());
            smoothStart(pts, ks, taxi_.heading(), taxi_.speed());
            s0 = lengthOf(pts) + reportAlong;
            appendAfter(st.line, lineKinds, 0, pts, ks);
        } else {
            pts.clear();
            ks.clear();
            float sq = std::min(std::max(sm, 0.0f) + 30, lengthOf(st.line)), qh = 0;
            Vec2 q = along(st.line, sq, qh);
            pts.push_back(taxi_.at());
            ks.push_back(ON_STAND_LINE | KEEP);
            turnAcross(taxi_.at(), taxi_.heading(), q, qh, TURN_RADIUS, pts, ks, ON_STAND_LINE | KEEP);
            float before = lengthOf(pts) + dist(pts.back(), q);
            pts.push_back(q);
            ks.push_back(ON_STAND_LINE | KEEP);
            appendAfter(st.line, lineKinds, sq, pts, ks);
            s0 = before + reportAlong - sq;
        }
    }
    plan_.build(pts, ks, true);
    taxi_.take(plan_);
    forks_.clear();
    forksOnPlan_.clear();
    keepUpFrom(s0, moving ? in.reportSpeed : 0, in.reportMs);
    stand_ = best;
    hold_ = -1;
    way_.clear();
    kinds_.clear();
    appendAfter(st.line, lineKinds, reportAlong, way_, kinds_);
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
        net.search(pm, mh, 3000, rules);
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
        // it: it stops where it is.
        plan_.clear();
        taxi_.take(plan_);
        keepUpFrom(0, 0, in.reportMs);
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
        turnAcross(m, mh, q, qh, TURN_RADIUS, pts, ks, (uint8_t)airfield::WayKind::TAXILANE | KEEP);
        float before = lengthOf(pts) + dist(pts.back(), q);
        pts.push_back(q);
        ks.push_back((kinds_.empty() ? (uint8_t)airfield::WayKind::TAXIWAY : kinds_[0]) | KEEP);
        s0 = before - ahead;
        appendAfter(way_, kinds_, ahead, pts, ks);
    }
    plan_.build(pts, ks, true);
    taxi_.take(plan_);
    placeForks();
    keepUpFrom(s0, moving ? in.reportSpeed : 0, in.reportMs);
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

void Follower::pushBack(const Network &net, const Input &in) {
    const StandWay &st = net.stands()[parkedAt_];
    static airfield::List<Vec2> pts;
    static airfield::List<uint8_t> ks;
    pts.clear();
    ks.clear();
    // Back along its stand's line to the taxilane first - which way along
    // that, once there, as its reports by then say: see pushOn().
    for (size_t i = st.line.size(); i-- > 0;) {
        pts.push_back(st.line[i]);
        ks.push_back(ON_STAND_LINE);
    }
    // From where it is shown - at the stop, or near it - not put back there.
    startAt(pts, taxi_.at());
    plan_.build(pts, ks, false);
    taxi_.take(plan_);
    taxi_.stop();
    taxi_.pushing = true;
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
    stand_ = hold_ = -1;
    mode_ = Mode::PUSH;
    say("pushing back from stand", st.ref);
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
    startAt(on, taxi_.at());
    plan_.build(on, onKinds, true);
    taxi_.take(plan_);
    pushOnward_ = true;
}

void Follower::update(const Network &net, const Input &in) {
    if (!active_ || net.empty()) {
        return;
    }
    if (in.reportMs != 0 && in.reportMs != reportMs_) {
        onReport(net, in);
    }
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
            rolling_ = true;
            onIt = true;
            s = plan_.nearest(in.simAt, plan_.length(), off);
        }
        Place r;
        if (!onIt && net.snap(in.simAt, in.simHeading, REACH, r)) {
            net.ahead(r, in.simHeading, 3000, way_, kinds_);
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
    bool lost = in.reportMs == 0 || (int32_t)(in.ms - in.reportMs) > (int32_t)LOST_MS;
    if (mode_ == Mode::FOLLOW && lost && !in.takeoff) {
        // Its reports gone: on by itself to where it was making for.
        Place from;
        if (net.snap(taxi_.at(), taxi_.heading(), 30, from)) {
            Rules rules;
            rules.runwayFactor = 3;
            rules.back = taxi_.speed() < 1;
            wayOn(net, from, taxi_.heading(), rules, in);
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
        for (float f : forksOnPlan_) {
            if (f > syncS_ + 1) {
                float room = std::max(0.0f, f + FORK_ROOM - taxi_.along());
                want = std::min(want, sqrtf(2 * FORK_BRAKE * room));
                break;
            }
        }
        break;
    }
    case Mode::OWN_WAY:
        want = 1e9f;
        break;
    case Mode::PUSH:
        want = PUSH_SPEED;
        break;
    case Mode::PARKED:
    case Mode::HELD:
        want = 0;
        break;
    }
    if (mode_ != Mode::FOLLOW) {
        taxi_.hurry = 1;
    }
    taxi_.step(in.dt, want);
    if (mode_ == Mode::PUSH && !pushOnward_ && taxi_.atEnd()) {
        pushOn(net, in);  // out of its stand: on along the taxilane
    } else if (taxi_.arrived()) {
        if (mode_ == Mode::OWN_WAY && stand_ >= 0 && dist(taxi_.at(), net.stands()[stand_].line.back()) > 10) {
            planInto(net, stand_);  // stopped short of it: on in
        } else if (mode_ == Mode::OWN_WAY && stand_ >= 0) {
            mode_ = Mode::PARKED;
            parkedAt_ = stand_;
            haveParkedSeen_ = false;
            arriving_ = false;
            say("parked at stand", net.stands()[stand_].ref);
        } else if (mode_ == Mode::OWN_WAY && hold_ >= 0) {
            mode_ = Mode::HELD;
            say(lineUp_ ? "lined up, from" : "holding at", net.holds()[hold_].ref);
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
