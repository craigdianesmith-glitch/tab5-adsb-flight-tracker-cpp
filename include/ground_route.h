#pragma once

#include <math.h>
#include <stdint.h>

#include <utility>

#include "airfield_pack.h"

// An airfield's taxiways, taxilanes and runways as a network - from its real
// layout, see airfield_pack.h - for an aircraft on the ground to be shown
// moving along them rather than straight across the grass between the places
// the feed reports it: snapped to the line it is on, and taken from there to
// where it is reported next the way the taxiways go - turning at the
// junctions as an aircraft can, never about.
namespace ground {

struct Vec2 {
    float x, y;
};

// A place on the network: which of its edges, how far along it from the
// edge's start, in metres.
struct Place {
    uint16_t edge = 0;
    float s = 0;
};

// What a point of a way found is on: a WayKind's value, or a stand's line in.
constexpr uint8_t ON_STAND_LINE = 4;

// What a search may do besides follow the lines.
struct Rules {
    // Leaving a runway for a taxiway, landed and rolling out at this many
    // metres a second at the start: only by a turn it can make by then,
    // braking at `brake` - a fast exit's shallow one sooner than a right
    // angle. 0: any.
    float speed = 0;
    float brake = 1.5f;
    // Along a runway - but not across one - counts as this many times as
    // far, for taxiing to keep off them where there is another way.
    float runwayFactor = 1;
    // Leaving the place backwards - as a kilometre more - at all: not for
    // one rolling out.
    bool back = true;
};

// A stand as the network reaches it: the place on a taxilane its line in
// starts from - or, with no line, the nearest - the way it leaves the
// taxilane, and the line on in to where it stops, facing `heading`.
struct StandWay {
    bool ok = false;
    char ref[8] = {};
    Place entry;
    float arrive = 0;  // radians anticlockwise from east
    airfield::List<Vec2> line;  // from the entry's point to the stop, both included
    float heading = 0;
    // At a terminal - its stop within 150m of one - rather than a remote one,
    // cargo's, or the flying club's.
    bool terminal = false;
    // The stop where the nose wheel stops, as OpenStreetMap maps a stand -
    // or, for one drawn only as its outline, the outline's middle, where the
    // middle of the aircraft goes.
    bool middle = false;
    // How far on from the stop, the way it faces, before a building.
    float room = 1e30f;
};

// Where an aircraft parks at `st`: how far back along its line from the stop
// its main wheels are - where the 3D view places an aircraft - with its nose
// wheel `noseWheel` metres ahead of them and its nose `nose`. Its nose wheel
// on the stop, or its middle on an outline's; further back if that would put
// its nose within a few metres of the building ahead.
float parkBack(const StandWay &st, float noseWheel, float nose);
// The point that far back along its line, and the way an aircraft there
// faces: towards the stop.
Vec2 parkedAt(const StandWay &st, float back, float &heading);
// Its line in, cut that far short of the stop: the way in to where it parks.
void lineIn(const StandWay &st, float back, airfield::List<Vec2> &out);

// A holding point as the network has it: where, and which way an aircraft
// waiting there faces - towards the runway, for a runway's.
struct HoldWay {
    bool ok = false;
    bool runway = false;
    // A runway's at one of its ends - where a departure waits - rather than
    // at a crossing.
    bool end = false;
    char ref[8] = {};
    Place at;
    float arrive = NAN;
};

class Network {
  public:
    // Built from `layout`'s ways, each an edge between two junctions; anything
    // there was before, gone.
    void build(const airfield::Layout &layout);
    bool empty() const { return edges_.empty(); }

    // The place on the network nearest `p` - by distance, and by how well the
    // line there runs with `heading` (radians anticlockwise from east; NAN
    // for none) - within `reach` metres: false if there is none so near. With
    // `runwaysOnly`, on a runway: some airfields have a taxiway drawn down a
    // runway, a metre or two off its centreline.
    bool snap(Vec2 p, float heading, float reach, Place &out, bool runwaysOnly = false) const;

    // One search out along the network from `from`, going the way `heading`
    // says - radians anticlockwise from east; NAN for either - up to `most`
    // metres: turning at the junctions by no more than an aircraft can,
    // never about, a turn counting as a little further the sharper it is,
    // and leaving `from`'s edge backwards as a kilometre more. Then costTo()
    // and pathTo() for anywhere, from there.
    void search(const Place &from, float heading, float most, const Rules &rules = Rules()) const;
    // How far, along the network, the last search found `to` - arriving
    // going within a right angle or so of `arrive` (NAN: either way) - or
    // 1e30 if not within its `most`.
    float costTo(const Place &to, float arrive = NAN) const;
    // The way there, as the points to pass through, both ends included -
    // and, with `kinds`, what each is on: false if it wasn't found. With
    // `forks`, the junctions on it where the way taken is one of two or more
    // much the same - a Y, or a turn where it could go straighter - that a
    // report has to say which of: how far along the way each is. Those off a
    // runway - its exits - not.
    bool pathTo(const Place &to, float arrive, airfield::List<Vec2> &path,
                airfield::List<uint8_t> *kinds = nullptr, airfield::List<float> *forks = nullptr) const;
    // search() and pathTo() in one.
    bool route(const Place &from, const Place &to, float most, airfield::List<Vec2> &path,
               float heading = NAN) const;
    // The way on from `from`, going the way `heading` says, straight on at
    // each junction - or as near it as there is, within 60 degrees - for
    // `distance` metres, or to where none goes on.
    void ahead(const Place &from, float heading, float distance, airfield::List<Vec2> &path,
               airfield::List<uint8_t> &kinds) const;

    Vec2 at(const Place &p) const;
    // The line's own way at `p`, radians anticlockwise from east, as the edge
    // runs.
    float heading(const Place &p) const;
    // Whether `p` is on a runway.
    bool onRunway(const Place &p) const;

    // The layout's stands and holding points, each as the network reaches
    // it, in the layout's order.
    const airfield::List<StandWay> &stands() const { return stands_; }
    const airfield::List<HoldWay> &holds() const { return holds_; }

  private:
    struct Edge {
        uint16_t from, to;  // junctions
        airfield::WayKind kind;
        airfield::List<Vec2> points;  // from's, the way's own, to's
        airfield::List<float> lengths;  // along it to each point
        Vec2 out, in;  // the way it leaves `from` and arrives at `to`, a unit long
        Vec2 beforeTo, afterFrom;  // its points 12m from each end, for those
    };
    float length(uint16_t k) const { return edges_[k].lengths.back(); }
    // A runway nearest `p`, within `reach`: the point on its centreline.
    bool nearestRunway(Vec2 p, float reach, Vec2 &q) const;

    airfield::List<Edge> edges_;
    airfield::List<Vec2> junctions_;
    airfield::List<airfield::List<uint16_t>> meets_;  // each junction's edges
    // Every segment of every edge by the 100m squares it crosses, for snap()
    // to look only near: square i's run in segments_ is from first_[i] to
    // first_[i + 1]; each an edge and its segment, in 16 bits each.
    float gridX_ = 0, gridY_ = 0;
    int gridW_ = 0, gridH_ = 0;
    airfield::List<uint32_t> first_, segments_;
    airfield::List<StandWay> stands_;
    airfield::List<HoldWay> holds_;

    // The last search's, for each edge each way - [2k] from its `from` to its
    // `to`, [2k + 1] back: the cost on turning onto it at its start, which
    // of them it was turned onto from (-1 none), and the cost at its far end,
    // whether by that or - `started` - leaving the search's place along it.
    mutable airfield::List<float> enter_, arrive_;
    mutable airfield::List<int32_t> before_;
    mutable airfield::List<uint8_t> started_;
    // And where it was 12m before it got to the far end - on the edges
    // before, for one shorter - for the way it arrives there.
    mutable airfield::List<Vec2> tail_;
    mutable airfield::List<std::pair<float, int32_t>> heap_;
    mutable Place from_;
    mutable float startCost_[2] = {0, 0};  // leaving from_ forwards, backwards
    mutable Rules rules_;
};

// Where along `path` - its points in order - `distance` metres from its start
// is, and the way it runs there.
Vec2 along(const airfield::List<Vec2> &path, float distance, float &heading);
float lengthOf(const airfield::List<Vec2> &path);

}  // namespace ground
