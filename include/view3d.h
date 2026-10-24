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
    // The runway ends the airport zoom has lately seen in use - "27L" - at
    // the airport `inUseAirport`: for one departing to make for a holding
    // point at one of them.
    char inUseAirport[5] = "";
    char inUse[8][4] = {};
    int inUseCount = 0;
    // Its signal lost: the last poll came back without it, or with its place
    // stale.
    bool lost = false;
    // Said on the strip after where it is: "while EZY45 taxis out", for one
    // shown while the one followed is taxiing - see main's 3D subject.
    String note;
};

// Whether the speed `now` has is one: its report gave it, or its take-off or
// landing is simulated. Otherwise its nought is only for want of one, and is
// shown as "?".
inline bool speedKnown(const Followed &f) { return f.report.hasGs || f.simulating; }

// Takes the screen and what drawing it takes - 90KB of the chip's own memory
// for the bands the picture is drawn in - and draws its buttons: false, and
// nothing taken, if there isn't the memory.
bool open();
#ifdef BENCH3D
// The land's cover - see scene3d's GroundCover - off, or on again: true for on.
bool benchToggleCover();
// Where the cover's time has gone since the last asked, both cores', to the
// log: a second's worth of each part of it.
void benchCoverReport();
#endif
// Gives them back, the frame being drawn finished first.
void close();
// Whether the one followed's engines are heard - the SOUND button's, which
// saves it; this from the settings at boot.
void setEngineSound(bool on);
bool isOpen();
// Showing the one followed parked at a stand, or making for one - landed,
// its signal lost on the ground, taxied in by itself: following is kept on
// while it is, lost or not, for it to be seen to park and stay parked.
bool parkingLost();
// Whether (lat, lon) is at one of the stands of the airfield the 3D view is
// at - its stop within a few tens of metres. False if it is at none, or the
// view knows no stands there.
bool atStand(double lat, double lon);
// The airport whose airfield the 3D view is at - "EDI" - or "" high up, or
// closed.
const char *airport();

// Each pass of the loop: the frame prepared the pass before drawn - and sent
// to the screen a band at a time - on the other core, while this one is
// prepared, for `flight` where it is now and the traffic about it as last
// polled, at `polledMs`. `nowMs` is the time now on the clock its reports and
// the poll are timed on - millis() live, the recording's in a replay: the
// view runs on with it, stands still while it does, and starts afresh where
// it jumps, a replay taken a minute on or back.
void frame(const Followed &flight, const std::vector<Aircraft> &traffic, uint32_t polledMs, uint32_t nowMs);

// A replay's place and pace, for the strip under the picture to show and
// work - see setReplay().
struct ReplayBar {
    bool playing = false;
    int speed = 1;                   // 1x or 4x
    uint32_t t = 0, durationMs = 0;  // ms into the recording, and how long it is
    uint32_t epoch = 0;              // UTC seconds at t; 0 where the recording's clock wasn't set
};
// Showing a replay, `bar` as it is - each frame - or live, null. In a replay
// the strip under the picture has the replay's controls, the flight on a line
// above them, and the engines fall quiet while it is paused.
void setReplay(const ReplayBar *bar);
// Where the traffic's pressure altitudes are put right from - see
// FollowSample::pressureFt: the live day's pressure, or a replay's own.
void setDayPressure(DayPressureFn dayPressure);

// The buttons, details and all drawn again, for the screen coming back.
void redraw();

// What the drawing core is doing - "drawing band 12 of 34" - for a hang's
// report.
String busy();

// And in a replay, its controls: PLAY - or pause - BACK_MINUTE, SPEED,
// ON_MINUTE, and SEEK to seekTo() of the way through, 0 to 1.
enum class Action : uint8_t { NONE, BACK, DETAIL, PLAY, BACK_MINUTE, SPEED, ON_MINUTE, SEEK };
float seekTo();
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
// The camera and the skyline's hills as drawn, column by column, on the
// serial port - for checking them against the land's heights.
void dumpSkyline();
// The land at its height, or flat with the hills a skyline - each time the
// other (##m).
void toggleTerrain();
// On the bench (##i and a digit): what the terrain's memory costs it - see
// scene3d::terrainProbe().
void terrainProbe(int which);
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
