#pragma once

#include <Arduino.h>
#include <vector>

#include "adsb_client.h"
#include "follow.h"

// The 3D view of the flight followed: the aircraft as it flies - its type's
// model in its airline's colours, banking into its turns - with the traffic
// about it, labelled, over the airport it is at or, high up, the country
// below; through a director's shots or a camera of one's own. In the middle
// of the screen, with the radar's column of buttons down the right and the
// flight's details under it. See scene3d.h for the drawing and labels3d.h
// for the labels.
namespace view3d {

// The flight followed, as the loop knows it, each frame.
struct Followed {
    String hex, callsign, type, category;
    bool military = false;
    FollowEstimate now;  // where it is shown now - see RunwaySim::at()
    uint32_t reportMs = 0;    // when its last report with a position came, on millis()'s clock; 0 none
    FollowSample report = {};  // that report, as it came: where, its track and speed
    bool landed = false;       // seen in the air in the last quarter of an hour
    bool simulating = false;  // a take-off or landing simulated - see RunwaySim::simulating()
    // A take-off or a landing under way, as the runway simulation has it,
    // and its runway.
    RunwaySim::Movement movement = RunwaySim::Movement::NONE;
    RunwayLine runway;
};

// Takes the screen and what drawing it takes - 90KB of the chip's own memory
// for the bands the picture is drawn in - and draws its buttons: false, and
// nothing taken, if there isn't the memory.
bool open();
// Gives them back, the frame being drawn finished first.
void close();
bool isOpen();

// Each pass of the loop: the frame prepared the pass before drawn - and sent
// to the screen a band at a time - on the other core, while this one is
// prepared, for `flight` where it is now and the traffic about it as last
// polled, at `polledMs`.
void frame(const Followed &flight, const std::vector<Aircraft> &traffic, uint32_t polledMs);

// The buttons, details and all drawn again, for the screen coming back.
void redraw();

// What the drawing core is doing - "drawing band 12 of 34" - for a hang's
// report.
String busy();

enum class Action : uint8_t { NONE, BACK, DETAIL };
// A finger at (x, y) - `landed` as it lands, after that as it moves: a button
// acts as it lands, and a finger moved across the picture turns the camera
// round the aircraft.
Action touch(int x, int y, bool landed);
// It lifted: having stayed put on the picture, it was a tap - DETAIL, and the
// hex of the aircraft under it, if there was one.
Action lift(String &hex);

#ifdef BENCH3D
// The orbit camera swept round, up and down, and in and out, a little every
// frame for `seconds`: every way a finger could turn it, for the bench to
// try - each half second of it said in the log.
void sweep(uint32_t seconds);
// As though a camera button were pressed: 'c' CINEMA, 'h' CHASE, 'o' ORBIT,
// 't' TOWER, '+' ZOOM IN, '-' ZOOM OUT.
void benchPress(char what);
// The aircraft a tap on the picture would open the details of - another than
// the one followed, if any is labelled - false if there is none.
bool benchTap(String &hex);
// The bands drawn `rows` high from the next open, whatever room there is -
// 16, 12, 8 or 4; 0 for as there is room.
void benchBandRows(int rows);
// The sun as at a time of day - 1 day, 2 dusk, 3 night - for the bench to
// compare like with like; 0 the real one.
void benchSun(int which);
// The airfield the view was last at, for made-up traffic: its stands, as
// latitude and longitude, up to `max` - and points on its taxiways with the
// way along them, degrees from north. How many there are.
int benchStands(double *lat, double *lon, int max);
int benchWays(double *lat, double *lon, float *track, int max);
// The buildings' work timed, step by step, on the frame in view - see
// s3::benchSolids() - and said.
void benchSolids();
#endif

}  // namespace view3d
