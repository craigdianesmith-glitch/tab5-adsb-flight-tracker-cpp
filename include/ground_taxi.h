#pragma once

#include <math.h>
#include <stdint.h>

#include "ground_route.h"

// An aircraft taxiing, as the 3D view shows it: a way to go - along the
// network, see ground_route.h - its corners rounded as an aircraft turns
// them, and the speed it can take each part at; and the aircraft going
// along it, never backwards unless pushed, speeding up and slowing down as
// an aircraft does, its nose along the way.
namespace ground {

// A way to go, ready to be taxied.
class Plan {
  public:
    // From `points` - what each is on in `kinds`, see pathTo() - its corners
    // rounded; to stop at its end, or to go on at speed.
    void build(const airfield::List<Vec2> &points, const airfield::List<uint8_t> &kinds, bool stop);
    void clear();
    bool empty() const { return points_.size() < 2; }
    float length() const { return empty() ? 0 : at_.back(); }
    bool stops() const { return stop_; }
    // Where it is `s` metres along, and the way it runs there; before its
    // start or past its end, on along the line its first or last part is on.
    Vec2 pointAt(float s, float &heading) const;
    // The fastest it can be going `s` along, metres a second, to make every
    // turn ahead - and the stop at its end - slowing down at `brake`; the
    // straights' speeds `hurry` times over.
    float fastestAt(float s, float brake, float hurry = 1) const;
    // How far along it the point nearest `p` is, and how far off that point
    // is; looking no further along than `within`.
    float nearest(Vec2 p, float within, float &off) const;
    // What the point `s` along is on.
    uint8_t kindAt(float s) const;
    const airfield::List<Vec2> &points() const { return points_; }

  private:
    airfield::List<Vec2> points_;
    airfield::List<float> at_;       // along it to each point
    airfield::List<float> fastest_;  // the speed each point's turn can be taken at
    airfield::List<uint8_t> kinds_;
    bool stop_ = true;
};

// The aircraft on its way.
class Taxi {
  public:
    // Put at `p` facing `heading` - radians anticlockwise from east - going
    // `speed` metres a second, with nowhere to go.
    void place(Vec2 p, float heading, float speed);
    // Given a way to go, from where it is - the plan's start, near enough -
    // carrying on at the speed it is going.
    void take(const Plan &plan);
    // `dt` seconds on: along its way at up to `want` metres a second, as
    // fast as the turns ahead let it, speeding up and slowing down gently.
    // Pushed back, it goes backwards along it, tail first.
    void step(float dt, float want);
    void stop() { speed_ = 0; }

    Vec2 at() const { return at_; }
    float heading() const { return heading_; }
    float speed() const { return speed_; }  // metres a second
    float along() const { return s_; }
    const Plan &plan() const { return plan_; }
    bool arrived() const { return plan_.empty() || (s_ >= plan_.length() - 0.05f && speed_ < 0.05f); }
    // At its plan's end, stopped or not - one that goes on at speed.
    bool atEnd() const { return !plan_.empty() && s_ >= plan_.length() - 0.05f; }
    // Pushed back: going backwards, tail first.
    bool pushing = false;
    // Landed, rolling out: slowing down the runway all the while, never
    // speeding up on it.
    bool landing = false;
    // Catching up with where its reports put it: the straights' speeds this
    // many times over - though no faster round the turns.
    float hurry = 1;
    // Rolling for take-off: gathering speed as one does.
    bool takingOff = false;

  private:
    // Where its tail is, `back` metres behind it along the way it came: on
    // the plan, or before the plan's start along where it went before it.
    Vec2 behind(float back) const;

    Plan plan_;
    float s_ = 0;
    float speed_ = 0;
    Vec2 at_ = {0, 0};
    float heading_ = 0;
    // The last of where it went, a point a metre or so apart, newest last.
    static constexpr int TRAIL = 16;
    Vec2 trail_[TRAIL];
    int trailCount_ = 0;
};

// A point of a way handed to Plan::build() with this added to its kind is
// one to keep, however near the line through those either side: part of a
// curve onto the way from the way it faces.
constexpr uint8_t KEEP = 0x80;

// Its usual speeds and how it gets to them, metres a second and per second.
constexpr float TAXI_SPEED = 12.5f;  // 24 knots, on a straight taxiway
constexpr float STAND_SPEED = 5;     // in along a stand's line, 10 knots, creeping the last of it
constexpr float PUSH_SPEED = 1.5f;   // pushed back
constexpr float TAXI_ACCEL = 0.5f, TAXI_BRAKE = 1.0f;
// On a runway: braking for a turn off it, and slowing, landed, regardless;
// and gathering speed, rolling for take-off.
constexpr float RUNWAY_BRAKE = 1.5f, ROLLOUT_SLOWING = 0.6f, ROLL_ACCEL = 2.5f;

}  // namespace ground
