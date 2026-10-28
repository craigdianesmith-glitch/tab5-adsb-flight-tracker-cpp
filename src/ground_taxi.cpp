// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2026 Craig Smith. Overhead's take-off, landing and taxi simulation: PolyForm Noncommercial 1.0.0
// (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.

#include "ground_taxi.h"

#include <math.h>

#include <algorithm>

namespace ground {
namespace {

// Turning, it keeps to this sideways pull, metres a second a second - an
// eighth of a g: 15 knots round a taxiway corner, 30 off a fast exit.
constexpr float TURN_PULL = 1.2f;
// A corner's curve cuts inside the corner by no more than this, metres: a
// right angle's as its painted line does, a slight bend's barely.
constexpr float CUT_SHARP = 20, CUT_SLIGHT = 6;
constexpr float WIDEST_TURN = 600;  // metres, radius
// Its nose this far ahead of where it is, its tail as far behind: the way
// it points is the way between, as the way an aircraft's wheels point.
constexpr float HALF_BASE = 8;
// And it turns no tighter than this, metres radius, nor with more than this
// sideways pull, metres a second a second - the way it points following a
// new way's start, a little off the old, smoothly.
constexpr float TIGHTEST = 5, TIGHTEST_GOING = 15, MOST_PULL = 2.5f;
// Slowing harder than it plans to, to a speed a new way allows at once.
constexpr float HARD_BRAKE = 2.5f;
// Coming to its stop - a stand's, a holding point's - gently, creeping the
// last of it; and straight for this much before it, or half its last part,
// to stop lined up with the way.
constexpr float STOPPING = 0.3f, STRAIGHT_IN = 15;
// A way's points this close to the line between those either side are kinks
// in how it is drawn, not turns.
constexpr float KINK = 2;
constexpr float FASTEST = 90;  // m/s: a runway's, no limit of its own

float dist(Vec2 a, Vec2 b) { return hypotf(b.x - a.x, b.y - a.y); }

// How far back from a corner of `turn` radians its curve starts - as round
// as cutting it by no more than it may allows - and its radius.
float curveStart(float turn, float &radius) {
    float cut = turn >= 1.047f   ? CUT_SHARP
                : turn <= 0.436f ? CUT_SLIGHT
                                 : CUT_SLIGHT + (CUT_SHARP - CUT_SLIGHT) * (turn - 0.436f) / (1.047f - 0.436f);
    radius = std::min(WIDEST_TURN, cut / (1 / cosf(turn / 2) - 1));
    return radius * tanf(std::min(turn, 3.0f) / 2);
}

// The turn at p[i], radians, and which way: 1 left, -1 right.
float turnAt(const airfield::List<Vec2> &p, size_t i, float &side) {
    float inLength = dist(p[i - 1], p[i]), outLength = dist(p[i], p[i + 1]);
    if (inLength < 1e-3f || outLength < 1e-3f) {
        side = 1;
        return 0;
    }
    float ix = (p[i].x - p[i - 1].x) / inLength, iy = (p[i].y - p[i - 1].y) / inLength;
    float ox = (p[i + 1].x - p[i].x) / outLength, oy = (p[i + 1].y - p[i].y) / outLength;
    side = ix * oy - iy * ox >= 0 ? 1.0f : -1.0f;
    return acosf(std::max(-1.0f, std::min(1.0f, ix * ox + iy * oy)));
}

// The fastest along a straight part of what `kind` is.
float straightFastest(uint8_t kind) {
    switch (kind) {
    case (uint8_t)airfield::WayKind::RUNWAY:
        return FASTEST;
    case ON_STAND_LINE:
        return STAND_SPEED;
    default:
        return TAXI_SPEED;
    }
}

}  // namespace

void Plan::clear() {
    points_.clear();
    at_.clear();
    fastest_.clear();
    kinds_.clear();
    arc_.clear();
}

void Plan::build(const airfield::List<Vec2> &given, const airfield::List<uint8_t> &givenKinds, bool stop) {
    clear();
    stop_ = stop;
    // The points, none on top of the one before; each part of the way as
    // what its far end is on.
    static airfield::List<Vec2> p;
    static airfield::List<uint8_t> k;
    p.clear();
    k.clear();
    for (size_t i = 0; i < given.size(); i++) {
        if (p.empty() || dist(p.back(), given[i]) > 0.5f) {
            p.push_back(given[i]);
            k.push_back(i < givenKinds.size() ? givenKinds[i] : (uint8_t)airfield::WayKind::TAXIWAY);
        } else if (i + 1 == given.size()) {
            p.back() = given[i];
        }
    }
    // A first part under 3m - from where it is to a junction it has all but
    // reached - turning as sharply as the next: not, but straight on to the
    // point after.
    if (p.size() > 2 && dist(p[0], p[1]) < 3) {
        p.erase(p.begin() + 1);
        k.erase(k.begin() + 1);
    }
    if (p.size() < 2) {
        return;  // nowhere to go
    }
    // Kinks of a metre or two - where two ways' ends don't quite meet, as
    // drawn - not turned at: only the points more than 2m off the line
    // through those either side kept.
    static airfield::List<uint8_t> keep;
    static airfield::List<std::pair<uint16_t, uint16_t>> spans;
    keep.assign(p.size(), 0);
    keep.front() = keep.back() = 1;
    for (size_t i = 0; i < p.size(); i++) {
        if (k[i] & KEEP) {
            keep[i] = 1;
            k[i] &= ~KEEP;
        }
    }
    spans.clear();
    // Between kept points, each span on its own.
    for (size_t i = 1, from = 0; i < p.size(); i++) {
        if (keep[i]) {
            spans.push_back({(uint16_t)from, (uint16_t)i});
            from = i;
        }
    }
    while (!spans.empty()) {
        std::pair<uint16_t, uint16_t> span = spans.back();
        spans.pop_back();
        Vec2 a = p[span.first], b = p[span.second];
        float dx = b.x - a.x, dy = b.y - a.y, len = hypotf(dx, dy);
        float worst = KINK;
        uint16_t at = 0;
        for (uint16_t i = span.first + 1; i < span.second; i++) {
            float off = len < 1e-3f ? dist(a, p[i]) : fabsf((p[i].x - a.x) * dy - (p[i].y - a.y) * dx) / len;
            if (off > worst) {
                worst = off;
                at = i;
            }
        }
        if (at != 0) {
            keep[at] = 1;
            spans.push_back({span.first, at});
            spans.push_back({at, span.second});
        }
    }
    size_t kept = 0;
    for (size_t i = 0; i < p.size(); i++) {
        if (keep[i]) {
            p[kept] = p[i];
            k[kept] = k[i];
            kept++;
        }
    }
    p.resize(kept);
    k.resize(kept);
    // Two corners the same way round, too close together for each to have
    // its curve - a sharp corner and a short part before another, as some
    // junctions are drawn - one corner, where the parts either side meet, and
    // its curve.
    for (size_t i = 1; i + 2 < p.size();) {
        float side0, side1, r;
        float turn0 = turnAt(p, i, side0), turn1 = turnAt(p, i + 1, side1);
        float between = dist(p[i], p[i + 1]);
        bool merged = false;
        if (turn0 > 0.02f && turn1 > 0.02f && side0 == side1 && turn0 + turn1 < 2.6f &&
            between < curveStart(turn0, r) + curveStart(turn1, r)) {
            // Where the part in, carried on, meets the part out, carried back.
            float ix = p[i].x - p[i - 1].x, iy = p[i].y - p[i - 1].y;
            float ox = p[i + 2].x - p[i + 1].x, oy = p[i + 2].y - p[i + 1].y;
            float il = hypotf(ix, iy), ol = hypotf(ox, oy);
            if (il < 0.1f || ol < 0.1f) {
                i++;
                continue;
            }
            ix /= il, iy /= il, ox /= ol, oy /= ol;
            float det = ix * oy - iy * ox;
            if (fabsf(det) > 1e-3f) {
                float dx = p[i + 1].x - p[i].x, dy = p[i + 1].y - p[i].y;
                float a = (dx * oy - dy * ox) / det;  // along the part in, from p[i]
                float b = (dx * iy - dy * ix) / det;  // back along the part out, from p[i + 1]
                if (a >= -0.1f && b >= -0.1f && a < 3 * between + 10 && b < 3 * between + 10) {
                    p[i] = {p[i].x + ix * a, p[i].y + iy * a};
                    p.erase(p.begin() + i + 1);
                    k.erase(k.begin() + i + 1);
                    merged = true;
                }
            }
        }
        i = merged ? std::max((size_t)1, i - 1) : i + 1;
    }
    // None left on top of the one before, the merging done.
    size_t left = 1;
    for (size_t i = 1; i < p.size(); i++) {
        if (dist(p[i], p[left - 1]) > 0.5f || i + 1 == p.size()) {
            p[left] = p[i];
            k[left] = k[i];
            left++;
        }
    }
    p.resize(left);
    k.resize(left);
    if (p.size() >= 2 && dist(p[p.size() - 1], p[p.size() - 2]) < 0.05f) {
        p.erase(p.end() - 2);
        k.erase(k.end() - 2);
    }
    if (p.size() < 2) {
        return;
    }
    auto add = [&](Vec2 q, uint8_t kind, float fastest, bool arc) {
        if (!points_.empty() && dist(points_.back(), q) < 0.05f) {
            fastest_.back() = std::min(fastest_.back(), fastest);
            return;
        }
        points_.push_back(q);
        kinds_.push_back(kind);
        fastest_.push_back(fastest);
        arc_.push_back(arc);
    };
    add(p[0], k[0], FASTEST, false);
    // Each corner as a curve - as round as its parts' lengths allow, and no
    // more than it cuts the corner by - the curve's points as fast as its
    // radius lets an aircraft turn.
    for (size_t i = 1; i + 1 < p.size(); i++) {
        float inLength = dist(p[i - 1], p[i]), outLength = dist(p[i], p[i + 1]);
        if (inLength < 0.01f || outLength < 0.01f) {
            add(p[i], k[i], FASTEST, false);
            continue;
        }
        float ix = (p[i].x - p[i - 1].x) / inLength, iy = (p[i].y - p[i - 1].y) / inLength;
        float ox = (p[i + 1].x - p[i].x) / outLength, oy = (p[i + 1].y - p[i].y) / outLength;
        float turn = acosf(std::max(-1.0f, std::min(1.0f, ix * ox + iy * oy)));
        uint8_t kind = k[i + 1];
        if (turn < 0.026f) {  // a degree and a half
            add(p[i], k[i], FASTEST, false);
            continue;
        }
        float half = tanf(std::min(turn, 3.0f) / 2);
        // The parts either side shared with the corners at their other ends -
        // half each - but the first's and the last's its own; the last's less
        // its straight in to a stop.
        float lastRoom = stop ? outLength - std::min(STRAIGHT_IN, outLength / 2) : outLength;
        float room = std::min(i == 1 ? inLength : inLength / 2, i + 2 == p.size() ? lastRoom : outLength / 2);
        float radius;
        curveStart(turn, radius);
        radius = std::min(radius, room / half);
        float t = radius * half;
        float fastest = sqrtf(TURN_PULL * radius);
        Vec2 a = {p[i].x - ix * t, p[i].y - iy * t};
        float left = ix * oy - iy * ox > 0 ? 1.0f : -1.0f;
        Vec2 centre = {a.x - iy * radius * left, a.y + ix * radius * left};
        float start = atan2f(a.y - centre.y, a.x - centre.x);
        int steps = std::max(1, (int)ceilf(turn / 0.17f));  // ten degrees each
        add(a, k[i], fastest, false);
        for (int s = 1; s <= steps; s++) {
            float angle = start + left * turn * s / steps;
            add({centre.x + radius * cosf(angle), centre.y + radius * sinf(angle)}, kind, fastest, true);
        }
    }
    add(p.back(), k.back(), stop ? 0.0f : FASTEST, false);
    if (points_.size() < 2) {
        clear();
        return;
    }
    at_.push_back(0);
    for (size_t i = 1; i < points_.size(); i++) {
        at_.push_back(at_.back() + dist(points_[i - 1], points_[i]));
    }
}

Vec2 Plan::pointAt(float s, float &heading) const {
    if (empty()) {
        return {0, 0};
    }
    // The part it is on: the first or last carried on past the ends.
    size_t i = std::upper_bound(at_.begin(), at_.end(), s) - at_.begin();
    i = std::max((size_t)1, std::min(points_.size() - 1, i));
    // Past a point too close to its neighbour to give a way: the part before.
    while (i > 1 && at_[i] - at_[i - 1] < 1e-3f) {
        i--;
    }
    Vec2 a = points_[i - 1], b = points_[i];
    float d = at_[i] - at_[i - 1];
    if (d < 1e-3f) {
        return b;
    }
    heading = atan2f(b.y - a.y, b.x - a.x);
    float t = (s - at_[i - 1]) / d;
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

float Plan::fastestAt(float s, float brake, float hurry) const {
    if (empty()) {
        return 0;
    }
    size_t i = std::upper_bound(at_.begin(), at_.end(), s) - at_.begin();
    i = std::max((size_t)1, std::min(points_.size() - 1, i));
    float best = straightFastest(kinds_[i]) * hurry;
    for (size_t j = i; j < points_.size(); j++) {
        float ahead = std::max(0.0f, at_[j] - s);
        bool last = j + 1 == points_.size() && stop_;
        float slowing = last ? std::min(brake, STOPPING) : brake;
        if (!last && 2 * brake * ahead >= best * best &&
            (!stop_ || 2 * STOPPING * std::max(0.0f, at_.back() - s) >= best * best)) {
            break;  // none further on can slow it more
        }
        float there = std::min(fastest_[j], straightFastest(kinds_[j]) * hurry);
        best = std::min(best, sqrtf(there * there + 2 * slowing * ahead));
    }
    return best;
}

float Plan::nearest(Vec2 p, float within, float &off) const {
    off = 1e30f;
    float best = 0;
    for (size_t i = 1; i < points_.size() && at_[i - 1] <= within; i++) {
        Vec2 a = points_[i - 1], b = points_[i];
        float dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        float t = len2 < 1e-6f ? 0 : std::max(0.0f, std::min(1.0f, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
        float d = hypotf(a.x + dx * t - p.x, a.y + dy * t - p.y);
        if (d < off) {
            off = d;
            best = at_[i - 1] + sqrtf(len2) * t;
        }
    }
    return best;
}

uint8_t Plan::kindAt(float s) const {
    if (empty()) {
        return 0;
    }
    size_t i = std::upper_bound(at_.begin(), at_.end(), s) - at_.begin();
    return kinds_[std::max((size_t)1, std::min(points_.size() - 1, i))];
}

float Plan::turnStart(float s, float within) const {
    if (empty()) {
        return s;
    }
    // The part it is on - from point i - 1 to point i - and back over the
    // curve's parts before it.
    size_t i = std::upper_bound(at_.begin(), at_.end(), s) - at_.begin();
    i = std::max((size_t)1, std::min(points_.size() - 1, i));
    if (!arc_[i]) {
        return s;  // on a straight
    }
    while (i > 1 && arc_[i - 1] && s - at_[i - 2] <= within) {
        i--;
    }
    return std::min(s, at_[i - 1]);
}

void Taxi::place(Vec2 p, float heading, float speed) {
    plan_.clear();
    s_ = 0;
    at_ = p;
    heading_ = heading;
    speed_ = speed;
    pushing = false;
    takingOff = false;
    // As if come straight along the way it faces.
    trailCount_ = 2;
    trail_[0] = {p.x - cosf(heading) * 2 * HALF_BASE, p.y - sinf(heading) * 2 * HALF_BASE};
    trail_[1] = p;
}

Vec2 Taxi::behind(float back) const {
    if (s_ >= back || plan_.empty()) {
        float h;
        return plan_.pointAt(s_ - back, h);
    }
    // Before the plan's start: back along the trail from there.
    float h = 0;
    Vec2 at = plan_.pointAt(0, h);
    back -= s_;
    for (int i = trailCount_ - 1; i >= 0; i--) {
        float d = dist(at, trail_[i]);
        if (d >= back && d > 1e-3f) {
            return {at.x + (trail_[i].x - at.x) * back / d, at.y + (trail_[i].y - at.y) * back / d};
        }
        back -= d;
        at = trail_[i];
    }
    return {at.x - cosf(h) * back, at.y - sinf(h) * back};
}

void Taxi::take(const Plan &plan) {
    plan_ = plan;
    float off;
    s_ = plan_.nearest(at_, 30, off);
    if (off > 30) {
        s_ = 0;
    }
}

void Taxi::step(float dt, float want) {
    if (dt <= 0) {
        return;
    }
    if (plan_.empty()) {
        speed_ = std::max(0.0f, speed_ - HARD_BRAKE * dt);
        return;
    }
    bool runway = plan_.kindAt(s_) == (uint8_t)airfield::WayKind::RUNWAY;
    float fastest = std::min(want, plan_.fastestAt(s_, runway ? RUNWAY_BRAKE : TAXI_BRAKE, hurry));
    if (runway && !takingOff && !landing) {
        fastest = std::min(fastest, TAXI_SPEED * hurry);  // taxiing along a runway: at a taxi's speed
    }
    if (pushing) {
        fastest = std::min(fastest, pushSpeed);
    }
    if (landing && runway && speed_ > TAXI_SPEED) {
        fastest = std::min(fastest, speed_ - ROLLOUT_SLOWING * dt);
    }
    if (fastest > speed_) {
        // On a runway, rolling for take-off, it gathers speed as one does -
        // a little faster, behind where it should be, to catch up before
        // it is handed back to the take-off's simulation.
        float accel = takingOff ? ROLL_ACCEL : TAXI_ACCEL;
        if (takingOff && hurry > 1) {
            accel *= 1.6f;
        }
        speed_ = std::min(fastest, speed_ + accel * dt);
    } else {
        speed_ = std::max(fastest, speed_ - HARD_BRAKE * dt);
    }
    s_ += speed_ * dt;
    if (s_ >= plan_.length()) {
        s_ = plan_.length();
        if (plan_.stops()) {
            speed_ = 0;
        }
    }
    float h = heading_;
    at_ = plan_.pointAt(s_, h);
    if (trailCount_ == 0 || dist(trail_[trailCount_ - 1], at_) >= 1) {
        if (trailCount_ == TRAIL) {
            for (int i = 1; i < TRAIL; i++) {
                trail_[i - 1] = trail_[i];
            }
            trailCount_--;
        }
        trail_[trailCount_++] = at_;
    }
    // The way its wheels point: from its tail - where it came from - to its
    // nose, on the way ahead; turning to it as fast as it can at its speed.
    Vec2 nose = plan_.pointAt(s_ + HALF_BASE, h), tail = behind(HALF_BASE);
    float want_heading = atan2f(nose.y - tail.y, nose.x - tail.x) + (pushing ? (float)M_PI : 0.0f);
    float turn = remainderf(want_heading - heading_, 2 * (float)M_PI) * std::min(1.0f, dt / 0.25f);
    // Standing still it can't turn at all; creeping, it can pivot tightly -
    // about its main gear, as a tug or a stand's lead-in has it - and going,
    // only as tight as a taxiway's curve.
    float tightest = TIGHTEST + std::max(0.0f, std::min(1.0f, (speed_ - 1.5f) / 3.5f)) * (TIGHTEST_GOING - TIGHTEST);
    float most = std::min(speed_ / tightest, MOST_PULL / std::max(speed_, 0.5f)) * dt;
    heading_ = remainderf(heading_ + std::max(-most, std::min(most, turn)), 2 * (float)M_PI);
}

}  // namespace ground
