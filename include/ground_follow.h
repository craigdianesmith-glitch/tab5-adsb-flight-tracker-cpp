#pragma once

#include <math.h>
#include <stdint.h>

#include "ground_route.h"
#include "ground_taxi.h"

// The one followed, on the ground at an airfield whose taxiways are known, as
// the 3D view shows it. Its reports - ten seconds apart at best, and a few
// metres out - are taken as where it was then and the way it was going: the
// way it is making for worked out from there - to a stand, landed; to a
// holding point, departing - and it is taxied along the taxiways to them, in
// step with the reports, never backwards. When they stop it taxies on along
// that way by itself, and parks, or waits at the holding point. Landed and
// handed over from the runway simulation, rolling out, it brakes for the
// first exit it can make and taxies in. Leaving a stand, it is pushed back.
namespace ground {

class Follower {
  public:
    struct Input {
        uint32_t ms;  // now, on the reports' clock
        float dt;     // seconds since the last update
        // Its last report with a position - `reportMs` 0 for none: where, in
        // the view's metres, its track - radians anticlockwise from east, NAN
        // for none - and its speed, metres a second.
        uint32_t reportMs;
        Vec2 reportAt;
        float reportHeading, reportSpeed;
        // The runway simulation's take-off - lined up, or the start of its
        // roll - while there is one, and where it puts it.
        bool takeoff;
        Vec2 simAt;
        float simHeading, simSpeed;
        // Others on the ground standing still, whose stands are taken.
        const Vec2 *standing;
        int standingCount;
        // Seen in the air in the last quarter of an hour: landed, so making
        // for a stand. Otherwise it is taken to be departing.
        bool arrived;
        // The runway ends departures are taking off from - where each
        // starts, as the airport zoom has lately seen them in use - for one
        // departing to make for a holding point at one of those rather than
        // whichever is nearest. None: any.
        const Vec2 *departFrom;
        int departFromCount;
        // Its signal lost: the last poll came back without it. Landed, it
        // taxies on by itself to a free stand and parks there - straight
        // in, no waiting at the way in for a report to say which stand.
        bool lost;
        // How far ahead of where it is placed - its main wheels - its nose
        // wheel is, and its nose, in metres: for it to park with its nose
        // wheel on a stand's stop, as OpenStreetMap maps them, and its nose
        // clear of the terminal. Both 0: its main wheels on the stop.
        float noseWheel, nose;
    };

    // Taken over - from where it was shown, facing `heading`, going `speed`
    // metres a second - having just landed and still rolling out, if
    // `landed`, or otherwise from its reports.
    void begin(const Network &net, Vec2 at, float heading, float speed, bool landed, const Input &in);
    void update(const Network &net, const Input &in);
    void end() { active_ = false; }
    bool active() const { return active_; }
    // Parked at a stand, or landed and making for one by itself.
    bool parking() const { return active_ && (mode_ == Mode::PARKED || (mode_ == Mode::OWN_WAY && arriving_ && stand_ >= 0)); }
    // Standing parked at a stand - not on the way to one.
    bool parked() const { return active_ && mode_ == Mode::PARKED; }

    Vec2 at() const { return taxi_.at(); }
    float heading() const { return taxi_.heading(); }
    float speed() const { return taxi_.speed(); }
    // What it is doing, for the log: "taxiing to stand 457R".
    const char *doing() const { return doing_; }

  private:
    enum class Mode : uint8_t { FOLLOW, OWN_WAY, PUSH, PARKED, HELD };
    // The way from `from` to where it is making for - or straight on, if
    // there is nowhere - as points and what each is on.
    void wayOn(const Network &net, const Place &from, float heading, const Rules &rules, const Input &in);
    void onReport(const Network &net, const Input &in);
    // Its plan: from where it is to the report's place, `reported`, and on
    // along way_ - or round to way_ further on, or across to it.
    void join(const Network &net, const Place &reported, float heading, const Input &in);
    void pushBack(const Network &net, const Input &in);
    // way_ from `from` - runway end hold `hold`, or past it - going
    // `heading`, onto its runway, lined up: false if there is no way.
    bool lineUp(const Network &net, int hold, const Place &from, float heading);
    // Pushed back to the taxilane: on along it, tail first.
    void pushOn(const Network &net, const Input &in);
    // A report on a stand's line in, off the network - `moving` along it,
    // facing `heading`, or not: in along it, in step with the reports. False
    // if it isn't on one.
    bool onStandLine(const Network &net, Vec2 p, bool moving, float heading, const Input &in);
    // Its plan in to `stand`'s stop, from where it is shown.
    void planInto(const Network &net, int stand);
    // Put at `stand`, parked - seen there by a report at `seen`.
    void parkAt(const Network &net, int stand, Vec2 seen);
    int standNear(const Network &net, Vec2 p, float within) const;
    // Where at `st` it parks - its main wheels, short of the stop - and the
    // way it faces there; and its line in to there, into standLine_.
    Vec2 parkPoint(const StandWay &st, float &heading) const;
    const airfield::List<Vec2> &lineInto(const StandWay &st);
    // Shown past where it parks at `st` - on its line on in towards the
    // stop, where a first report put it - put back there, the plan to it
    // otherwise starting behind it.
    void notPast(const StandWay &st);
    void say(const char *what, const char *ref = nullptr);
    // On by itself along the taxiways to where it is making for - its
    // reports gone, or too far off them to go across to them.
    void ownWay(const Network &net, const Input &in);
    // Across from where it is shown onto a stand's `line`, at `from` along it
    // or further on - see turnOnto() - its points onto `pts`; how far along
    // the line it joins it.
    float acrossOnto(const airfield::List<Vec2> &line, float from, airfield::List<Vec2> &pts,
                     airfield::List<uint8_t> &ks);
    // Its way on cut to where it can stop, braking as it does - rather than
    // stopped dead where it is.
    void stopSoon();
    // In step from here: the report's place `s` along the plan, at `speed`.
    void keepUpFrom(float s, float speed, uint32_t ms);
    // forksOnPlan_ for the plan just made.
    void placeForks();
    // The way to search the network from `from` - where it is shown, snapped
    // onto it: the way it faces; or either, slow and side on to the line
    // there - as just pushed back off its stand's line - a turn either way
    // onto it.
    float searchHeading(const Network &net, const Place &from) const;

    bool active_ = false;
    Mode mode_ = Mode::FOLLOW;
    bool arriving_ = false;
    Taxi taxi_;
    Plan plan_;
    // The way it is making for, from its last report's place - the points,
    // what each is on - and what it ends at: a stand, a holding point, or
    // neither (-1).
    airfield::List<Vec2> way_;
    airfield::List<uint8_t> kinds_;
    // Its forks - where it goes one of two or more ways much the same, or in
    // to its stand - that a report has yet to say which of: where on the way,
    // and how far along the plan, once there is one.
    airfield::List<Vec2> forks_;
    airfield::List<float> forksOnPlan_;
    // And where along the plan it waits for that report: before the turn
    // there begins.
    airfield::List<float> forksWait_;
    int stand_ = -1, hold_ = -1;
    bool standSeen_ = false;  // a report has put it at stand_, or on its line: not a guess
    // The stand its reports have had it standing at, one after the other,
    // where it couldn't be taxied to: and how many.
    int standingAt_ = -1, standingCount_ = 0;
    bool lineUp_ = false;  // way_ goes on from hold_ onto its runway
    bool rolling_ = false;  // the take-off simulated rolling: its plan down the runway
    uint32_t takeoffJoinMs_ = 0;  // when it last joined the take-off's way, off its plan
    uint32_t ownWayMs_ = 0;  // when it last looked again, by itself, for somewhere to go
    int wildInRow_ = 0;  // reports passed over as wild, one after the other
    int parkedAt_ = -1;  // PARKED: which stand
    // Parked, where its reports put it - a few metres from the stand's stop,
    // as its aerial is - for it to be seen to move from there.
    Vec2 parkedSeen_ = {0, 0};
    bool haveParkedSeen_ = false;
    // Parked: whether its last report was out towards the taxilane, and how
    // far from where it was seen; and how far its reports wander about
    // where it stands, on average, over how many.
    bool outBefore_ = false, farBefore_ = false;
    float goneBefore_ = 0, wander_ = 0;
    int wanderN_ = 0;
    // Just begun, its first report not yet taken: shown nowhere before.
    bool justBegun_ = false;
    int leftStand_ = -1;  // pushed back from, its reports there not taken as back on it
    int pushFrom_ = -1;   // PUSH: from which stand
    bool pushOnward_ = false;  // PUSH: out on the taxilane, the second part
    uint32_t reportMs_ = 0;
    // The report before, for one creeping along slower than STILL to be
    // seen to move.
    Vec2 prevAt_ = {0, 0};
    uint32_t prevMs_ = 0;
    // In step with the reports: the report's place as far along the plan as
    // syncS_ - before its start, negative - from where the report puts it
    // on, at its speed, the time since.
    float syncS_ = 0, syncSpeed_ = 0;
    uint32_t syncMs_ = 0;
    // Where the reports put it now: carried on from the last along the plan
    // at its speed, slowing for the turns as it would - how far along the
    // plan, and how fast.
    float ghostS_ = 0, ghostSpeed_ = 0;
    // Faster in its last report than in the one before: speeding up, and
    // taken to go on doing so, to the speed the way allows.
    bool speedingUp_ = false;
    float prevSpeed_ = 0;
    char doing_[40] = "";
    float noseWheel_ = 0, nose_ = 0;  // the input's
    airfield::List<Vec2> standLine_;
};

}  // namespace ground
