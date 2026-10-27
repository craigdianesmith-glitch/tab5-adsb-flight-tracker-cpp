#pragma once

#include "ground_pack.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

// A landing seen in 3D, drawn by the CPU: a low-poly airliner over a runway
// and its markings, all to scale, flat-shaded, in a sky and on grass that
// fade into haze with distance - by day, at dusk or at night, with the
// airfield's lights and the aircraft's own. There is no 3D hardware on the
// P4, so it is drawn a band of rows at a time into small buffers - colour and
// depth - that fit in the chip's own fast memory, each band then sent on to
// the screen.
//
// Plain C++ with no Arduino in it, so tools/landing3d_preview draws the same
// thing on the PC.
//
// The world is in metres: the runway runs along +x from its threshold at the
// origin, its centreline on y = 0, z up.
namespace airfield {
struct Layout;
}

namespace scene3d {

struct Vec3 {
    float x, y, z;
};

// Where an aircraft is: its main wheels' contact point, and which way it
// points (radians: heading round z from +x, pitch nose-up, roll right wing
// down).
struct Pose {
    Vec3 at;
    float heading, pitch;
    float roll = 0;
};

// The shapes an aircraft can be drawn as: each a model of one type, sized to
// any other of its kind - an A321 is the A320 made longer and wider alike.
enum class Family : uint8_t {
    NARROWBODY,      // A320 - and the 737s, 757s, E-Jets
    NARROWBODY_4,    // the 707 four-engined: KC-135, RC-135, E-6
    AWACS,           // the E-3 Sentry: a 707 under its radar's dish
    WIDEBODY,        // 787 - and 777s, 767s, A330s, A350s
    WIDEBODY_4,      // A340
    JUMBO,           // 747, its upper deck a hump
    SUPERJUMBO,      // A380, two decks the length of it
    REGIONAL_JET,    // CRJ900 - and ERJs, Fokkers, MD-80s: engines at the tail, a T-tail
    BUSINESS_JET,    // Gulfstream G650 - and Globals, Challengers, Citations, Learjets
    TURBOPROP,       // ATR 72 - and Dash 8s: a high wing, two propellers, a T-tail
    TURBOPROP_LOW,   // Saab 340 - and Jetstreams, King Airs: a low wing, two propellers
    LIGHT_HIGH,      // Cessna 172 - and the Caravan, the Islander: a high wing on struts
    LIGHT_LOW,       // Piper Cherokee - and Cirruses, Diamonds, Tutors: a low wing, a propeller in the nose
    AIRLIFTER,       // C-17 - and the C-5, the Il-76: a high wing, four jets, a T-tail, a ramp
    HERCULES,        // C-130 - four propellers, a ramp under a tall tail
    ATLAS,           // A400M - eight-bladed propellers, a T-tail
    TACTICAL_TWIN,   // C295 - and the CN235, C-27J: two propellers, a ramp
    QUAD_HIGH,       // BAe 146 and Avro RJ: a high wing, four jets, a T-tail
    FIGHTER_DELTA,   // Typhoon - and Rafale, Gripen: a delta wing and canards
    FIGHTER,         // F-35 - and F-22, F-15, F/A-18: twin fins
    FIGHTER_SINGLE,  // F-16 - one fin, a cropped delta
    JET_TRAINER,     // Hawk - and T-38, Alpha Jet, M-346: a low swept wing, two seats
    HELICOPTER,      // AW139 - and S-92, H225, Merlin, Black Hawk: wheels, five blades
    HELICOPTER_LIGHT,  // H145 - and H135, AS350, Bell 407, R44: skids
    CHINOOK,         // two rotors, fore and aft
    COUNT
};

// What an aircraft looks like: its family, its size against that family's
// model - in length and span alike - and how it is painted: its body above
// and below, its tail, its engines and its wings, all 0xRRGGBB. White with the
// airline's colour on its tail for most; grey all over for the military. A
// freighter has no windows down its cabin.
struct Look {
    float scale = 1;
    Family family = Family::NARROWBODY;
    uint32_t tail = 0x1C428C;
    uint32_t body = 0xECEEF2, belly = 0xCED2D8, engines = 0xECEEF2, wings = 0xC4C8CE;
    bool windows = true;
};

// The look of an ICAO type - "A320", "B77W", "AT76", "EUFI" - with its tail
// in an airline's colour: its family and size, from the light aircraft and
// helicopters to the A380 and the C-5. A military one is painted grey all
// over. A type not known goes by its ADS-B emitter category - "A1" light,
// "A3" large, "A5" heavy, "A6" fast and agile, "A7" a rotorcraft - and failing
// that is an A320.
Look lookFor(const char *type, uint32_t tail, bool military = false, const char *category = nullptr);
// Its family alone, as lookFor() has it - without building the models, for
// code that runs whether the 3D view has been opened or not.
Family familyOf(const char *type, bool military = false, const char *category = nullptr);

// An airline's basic colours, where it is one known by its callsign - BAW,
// EZY, KLM, FDX... - on its body above and below, its tail and its engines;
// a freighter's without windows. A look whose airline isn't known is left as
// it was.
void paintAirline(Look &look, const char *callsign);

// How big it is against an A320, for a camera to stand off by: its length.
float sizeOf(const Look &look);

// How far ahead of its main wheels - where a Pose has it - its nose wheel is,
// and the tip of its nose, in metres: for one parked with its nose wheel on a
// stand's stop, as OpenStreetMap maps them, to be placed by its main wheels.
float noseWheelAhead(const Look &look);
float noseAhead(const Look &look);

// A type of each family, as its ICAO code - "A320" for the narrowbodies:
// what the made-up traffic of the preview and the bench is labelled as.
const char *typeOf(Family family);

// How it is set up - which ADS-B doesn't say, but what it is doing does: the
// gear down from the approach to the stand, up soon after a take-off; flaps
// out on the approach and for take-off; spoilers up from the touchdown until
// the roll-out slows. And the controls, -1 to 1 of their travel: the
// ailerons rolling it right, the elevator pitching it up, the rudder yawing
// it right; its propellers and rotors turning, or stopped.
struct Config {
    bool gear = true, flaps = false, spoilers = false;
    float spoilerDeploy = 0;  // how far up the spoilers are, 0 to 1 of their travel
    float aileron = 0, elevator = 0, rudder = 0;
    bool turning = true;
};

// How far up the ground spoilers of one that touched down `sinceS` seconds
// ago are, as far as that goes: up in under a second, as they spring up
// once the main wheels spin up.
inline float spoilersRising(float sinceS) {
    float t = sinceS < 0 ? 0 : sinceS > 0.8f ? 1 : sinceS / 0.8f;
    return t * t * (3 - 2 * t);
}

// Flies an aircraft the way its track says it is being flown - which ADS-B
// doesn't report, but its turning, climbing and slowing give away. Fed, a
// frame at a time, how fast it is turning, how fast it is going and how
// fast it is climbing, it banks into a turn as steeply as a coordinated turn
// at that speed calls for, rolling in and out no faster than its kind can;
// pitches its nose with its climb; and moves its controls as a pilot would to
// do that - ailerons to roll, then back to the middle, the elevator to pitch
// and to hold the nose up in a turn, a touch of rudder with the ailerons.
// Landing, it flares: see landingPitch(); its main wheels touch first, the
// nose held up, then lowered onto its wheel over a few seconds.
class Pilot {
  public:
    // `turnRate` in radians a second, positive to the left - as a heading
    // goes - over `dt` seconds since the last; `heightM` above the ground
    // under it, NAN when that isn't known; `departing` on its take-off roll
    // and climb-out, rotated for.
    void fly(float dt, Family family, float turnRate, float speedKt, float climbFpm, bool onGround,
             float heightM = NAN, bool departing = false);
    float roll = 0, pitch = 0;  // radians, as a Pose has them
    float aileron = 0, elevator = 0, rudder = 0;

  private:
    float rollRate_ = 0, pitchTarget_ = 0;
    float lastKt_ = 0, accel_ = 0;  // how fast it is speeding up, knots a second, smoothed
    bool wasGround_ = true;
    float touchPitch_ = 0, sinceTouch_ = -1;  // the nose as the wheels touched, and how long since
};

// The nose as it lifts off, taking off, rotated to: an airliner's 8
// degrees, a fast jet's 11.
float liftOffPitch(Family family);

// Its take-off by its kind - one table for the pilot's rotation and the
// runway simulation's roll and climb-out alike (see followSampleOf()), so its
// nose comes up as it nears the speed it leaves the runway at: rotated at
// `rotateKt`, off at `liftoffKt`, gathering speed down the runway at
// `accelKtS` knots a second, climbing away at `climbFpm`.
struct Takeoff {
    float rotateKt, liftoffKt, accelKtS;
    int climbFpm;
};
Takeoff takeoffOf(Family family);

// The nose of one coming in to land, given `pitch` along its way down and a
// few degrees above it: higher on the approach - slow, its flaps out, its
// wing needs more - and raised in the last fifty feet to the attitude it
// touches down at, its flare. `heightM` above the ground, NAN unknown.
float landingPitch(Family family, float speedKt, float climbFpm, float heightM, float pitch);

// The set-up of one doing this: at this height above the field, this fast,
// on the ground or not, landing or leaving.
Config configFor(float heightM, float speedKt, bool onGround, bool departing);

struct Aircraft {
    Pose pose;
    Look look;
    Config config;
};

struct Camera {
    Vec3 eye, target;
    float fovDeg;  // across the width of the view
};

// Where the sun is, seen from the airfield: degrees round from north, and up
// from the horizon - below it, negative.
struct Sun {
    float azimuthDeg, elevationDeg;
};

// The sun at `utc` - seconds since 1970 - from (lat, lon). The sky, the light,
// the shadows, the haze and whether the airfield's lights and the stars are
// out all follow from it.
Sun sunAt(int64_t utc, double lat, double lon);

// Stand-ins for times of day: a high afternoon sun, one just setting in the
// west, and one well down.
enum class TimeOfDay { DAY, DUSK, NIGHT };
Sun sunFor(TimeOfDay time);

// What covers the land, from the ground pack - see ground_pack.h - stamped
// onto the ground's rows: trees, fields, moor, towns and water, shaded as
// the hills lie. `window` the pack's tiles, as they are this frame, or
// nullptr for none. Where a point on the ground is in the window, in tiles -
// u east from its west edge, v south from its north edge: flat, at an
// airfield, straight from the world's x and y, u = ux x + u0 and v = vy y +
// v0; round, high up, from the point's latitude and longitude on the
// sphere about the origin, (lat0, lon0) in degrees, each offset scaled by
// `kLat` or `kLon` - the sphere's radius over the ellipsoid's there, which
// the view's own places are on.
struct GroundCover {
    const groundpack::Window *window = nullptr;
    bool round = false;
    float ux = 0, u0 = 0, vy = 0, v0 = 0;
    double lat0 = 0, lon0 = 0;
    float kLat = 1, kLon = 1;
};
// The cover over `window` for a view whose places are about (lat0, lon0):
// flat, at an airfield, each x metres east and y north of there - 111,320m
// to a degree of longitude at the equator, by the cosine of lat0, and
// 110,574m to one of latitude; or round, high up, on WGS84 about there.
GroundCover flatCover(const groundpack::Window &window, double lat0, double lon0);
GroundCover roundCover(const groundpack::Window &window, double lat0, double lon0);

struct Scene {
    Aircraft followed;                   // the aircraft the view is about
    const Aircraft *traffic = nullptr;   // everything else about - parked, taxiing, flying
    int trafficCount = 0;
    float seconds = 0;               // a clock, for lights flashing
    float smokeAge = -1;             // seconds since the followed aircraft's tyres touched; < 0 none
    Pose touchdown{};                // and where they did
    bool landingLights = true;
    Sun sun = {200, 40};
    // An aircraft smaller on the screen than detailPx - pixels from its
    // middle to its nose - is drawn as the simple model, a fifth of the
    // faces, and one smaller than tinyPx as the tiny one, some thirty. What a
    // frame-rate governor turns up when frames run long. The one followed is
    // always drawn in full.
    float detailPx = 60, tinyPx = 12;
    // How much the land at its height may skimp, 0 to 1, as the frame-rate
    // governor sets it: from steps a fortieth further each, half a grid cell
    // at least, out to 100km, to a twentieth, a whole cell, and 70km - and
    // its colours a sample every four rows, not two, once past 0.65, till
    // back under 0.45.
    float terrainEffort = 0;
    // High up: the ground the sphere it is, `earthRadius` metres round, its
    // middle that far under the origin - 0 for an airfield's flat ground; how
    // far one can see, 0 for what the light makes it; from each aircraft
    // higher than `stemsAbove` metres a line down to the ground under it -
    // none for < 0 - and behind each jet higher than `trailsAbove`, its trail.
    float earthRadius = 0;
    float visibility = 0;
    float stemsAbove = -1;
    float trailsAbove = 8000;
    // And the land told from the sea by the coasts - which needs every
    // coast's land on the side it says: high up, or low by an airfield on
    // the sea, the land then its grass.
    bool landAndSea = false;
    // What covers the land - see GroundCover - or none, the land one colour.
    GroundCover cover;
    // The land at its height, from the cover's window: its hills, near and
    // far, from any height - the view's ground at `groundUpM` above the sea,
    // an airfield's held level round it. Off, the land flat and the hills a
    // skyline.
    bool terrain = false;
    float groundUpM = 0;
    const groundpack::TerrainGrid *terrainGrid = nullptr;  // its heights; none, no terrain
    // The skyline - the hills along the horizon - from the ground pack's
    // horizon round the eye (see groundpack::Horizon), or nullptr for none.
    // `horizonTurn` how far north at the eye is turned from the view's
    // north, radians, the meridians closing in; and the eye's height above
    // the sea now.
    const groundpack::Horizon *horizon = nullptr;
    float horizonTurn = 0, eyeHeightM = 0;
    // Where the horizon was worked out from, in the view's own frame - its
    // highest points are put there and seen from where the eye is now.
    Vec3 horizonAt{};
};

// What is drawn of the country under an aircraft high up: lines on the
// ground - coasts and lakes' shores, borders and state lines - each from a to
// b in the world's metres, the earth's curve and all; or about an airfield,
// its coasts and shores laid out flat as it is - and the towns' outlines,
// the land built over. A coast or a shore has the land on its left, going
// from a to b, as the firmware's coast table has them all, and a town's
// outline the town - unless `landRight`: which tells which side is which.
enum class GroundKind : uint8_t { COAST, LAKE, BORDER, STATE, TOWN };
struct GroundLine {
    Vec3 a, b;
    GroundKind kind;
    bool landRight = false;
};
void setGroundLines(const GroundLine *lines, int count);

// The rivers about, each a line from a to b `width` metres across: drawn as
// a ribbon of water where that is a pixel or more on the screen, and where it
// is less as a line faded for how much of one it covers, so it doesn't break
// up into dashes as the view moves.
struct RiverLine {
    Vec3 a, b;
    float width;
};
void setRivers(const RiverLine *rivers, int count);

// Lights where the towns are, after dark: in the towns' outlines set by
// setGroundLines(), `most` of them, no nearer together than `closest`
// metres, thickest about the middle - half as thick `falloff` metres out -
// and `round` for outlines on the round earth, high up. Where none of the
// lines are a town's, the lights there are are left as they were. After
// setAirfield() or clearAirfield(), which set their own.
void lightTowns(int most, float closest, float falloff, bool round);


// Every family's model - each at each level of detail it is drawn at - built
// now rather than when the first frame needs it: some tens of milliseconds
// on the device, and what they hold in PSRAM. Returns the bytes they take.
size_t buildModels();

// Gets a frame ready: the aircraft and the camera placed, and every triangle,
// line and light lit, clipped to the view, projected and sorted into the
// bands of `bandRows` rows it will be drawn in. `swapBytes` for the device's
// canvas, which holds RGB565 high byte first. `w` a multiple of four - the
// land at its height is drawn a column of four pixels at a time - and in
// assembly to 2048 of them, C past that.
void prepare(int w, int h, int bandRows, bool swapBytes, const Scene &scene, const Camera &camera);

// The same in three steps, for two cores to share the middle one: the
// begin, then prepareAircraft(0, 2) on one core and (1, 2) on the other at
// once, then the end once both are done. The scene's traffic has to stay
// where it is until then. Unlike prepare(), these leave the frame to be
// drawn as it was - prepared into one of two, while the other is drawn -
// until flip() swaps them: so one core can prepare the next frame while the
// other draws this one.
void prepareBegin(int w, int h, int bandRows, bool swapBytes, const Scene &scene, const Camera &camera);
void prepareAircraft(int part, int parts);
void prepareEnd();
void flip();

// The airfield's buildings - at a big airport as costly as its ground - for
// another core to do while this one gets on with the rest: ready from
// prepareBegin() placing the camera until prepareEnd() needs them. Whichever
// comes to them first does them, and prepareEnd() waits for the other if it
// has. True if this call did them. `ready`, set, is called as they become
// ready to do - from prepareBegin(), to wake a core waiting on work.
bool prepareSolids();
void setSolidsReady(void (*ready)());

// For the bench: the buildings of the frame prepared last worked out 20 times
// over, a step more of the work each time - the cells and each triangle's
// facing; its colour; its corners placed and projected; its screen triangle
// set up; all of it, kept - and how long each took, a time, with how many
// triangles were looked at and how many face the eye.
struct SolidsBench {
    uint32_t us[5];
    uint32_t tested, facing;
    // And: the ground's triangles only read, and the ground step as a frame
    // has it.
    uint32_t groundReadUs, groundUs, groundTris;
    // The partly seen cells' triangles: all, wholly outside a plane, wholly
    // inside, cut (the device's bench only).
    uint32_t seen[4];
};
SolidsBench benchSolids();

// Draws band `band` of the frame prepared - rows [band * bandRows, and on) -
// into `scratch` and returns where in it they are: lines of w pixels, one
// after the other. `scratch` holds scratchBytes() and is 64-byte aligned; the
// colour comes first, for a DMA to take from. Two calls on different bands,
// each with its own scratch, can run at once.
size_t scratchBytes(int w, int bandRows);
// With `stageUs` - six of them - and a clock, adds how long each stage took:
// the sky and grass, the ground, shadows, clearing depth, solids, and lines
// and lights.
uint16_t *drawBand(int band, void *scratch, uint32_t *stageUs = nullptr);
// The same into a colour buffer and a depth buffer of the caller's: for a
// core to draw the next band into one colour buffer while the last is pushed
// to the screen from the other.
void drawBandInto(int band, uint16_t *color, uint16_t *depth, uint32_t *stageUs = nullptr);
int bandCount();
#if !defined(ESP_PLATFORM)
// Off the Tab5, the core a band is drawn as on, 0 or 1 - the terrain keeps
// caches of each core's own - for the preview to draw them as the Tab5's
// two cores share them.
extern int g_previewCore;
#endif

// What the frame prepared holds, after clipping and culling - and, given a
// clock, how long each step of preparing it took.
// Where the time preparing a frame goes, step by step - prepareBegin()'s,
// the aircraft's, prepareEnd()'s - for the bench to say.
enum Step : uint8_t {
    STEP_ROWS, STEP_FAR_GROUND, STEP_GROUND, STEP_PAINT, STEP_PLACE, STEP_SORT,
    STEP_FOLLOWED, STEP_TRAFFIC, STEP_SHADOWS,
    STEP_SOLIDS, STEP_WIRES, STEP_COUNTRY, STEP_STEMS, STEP_LIGHTS, STEP_SOLIDS_WAIT, STEP_BINS, STEP_DEPTH,
    STEP_RIVERS, STEPS
};
const char *stepName(int step);

struct Counts {
    int triangles, lines, lights, aircraftFull, aircraftSimple, aircraftTiny, aircraftDots;
    int crossings;  // where coasts cross the rows, for land and sea
    uint32_t beginUs, aircraftUs[2], endUs;
    uint32_t stepUs[STEPS];
};
Counts counts();
#ifdef BENCH3D
// Solids' spans since the last reset: how many under 8 pixels long, 8 to 23,
// and 24 and more - PIE's - then the pixels in each.
void spanStats(uint32_t out[6], bool reset);
// The land's cover since the last reset, in CPU cycles: reading the cells
// along the rows, working out their colours, and writing the pixels; then
// the rows read, the colours worked out, the cycles of the reading spent in
// its innermost loop - C's or the assembly's - and the cells it read; and,
// preparing, the cycles working out where rows are on the pack's grid, the
// rows, and the cycles storing what was worked out; and the skyline's.
void coverStats(uint64_t out[11], bool reset);
// The see-through - blades' blur, and the like - since the last reset: its
// cycles drawing, and the pixels it looked at; then the solids': their
// cycles, the rows stepped down, and the triangles taken up, a band at a time.
void seeStats(uint64_t out[7], bool reset);
// The aircraft followed since the last reset: cycles placing its corners,
// and lighting and setting up its faces; its corners, its faces, and the
// triangles they made.
void aircraftStats(uint64_t out[6], bool reset);
// A frame prepared, then one row's points on the pack's grid worked out 256
// times over - the cycles each time - one point's 256 times, the row's
// stored 256 times, and every other row's points once, together.
void coverMicro(int w, int h, const Scene &scene, const Camera &camera, uint32_t out[4]);
#endif
void setClock(uint32_t (*micros)());

// On the device: whether solids' long spans are drawn eight pixels at a time
// with the P4's PIE vector instructions (the default) or one at a time in C.
void setSimd(bool on);

// Where a point in the world is in the frame prepared, for a label to go by
// it; false if it is behind the eye.
bool projectPoint(Vec3 p, float &x, float &y);

// The frame prepared's camera - its eye, which ways are right, down and
// ahead, and its focal length and middle in pixels - and the rows its
// skyline's hills are drawn from and to, each column's; false with no hills
// in it. For checking them against the land's heights (##D).
bool skylineDrawn(Vec3 &eye, Vec3 &right, Vec3 &down, Vec3 &forward, float &f, float &cx, float &cy,
                  const int16_t *&top, const int16_t *&foot, int &width);

// The terrain's costs since the last reset, both cores': the march's
// cycles, samples, rows filled and blocks skipped; the drawing's cycles,
// samples, cells found the long way and colours worked out.
void terrainStats(uint64_t out[8], bool reset);
// A digest of the last march's output as the drawing reads it - each
// column's top and flat top and its rows' 1 / depths - to set one build's
// march against another's.
uint32_t terrainDigest();
// On the bench, the caches' data hits and misses over the march (L1 hits,
// misses, L2 hits, misses) and over the drawing (likewise), both cores'.
void terrainCache(uint64_t out[8], bool reset);
// On the bench: 1 the march's heights read from the grid's first four rows, 2
// the drawing's cells made up, 3 both, 0 neither - what memory costs them.
void terrainProbe(int which);
// On the bench: the march of the frame being prepared run again - as it is,
// then with its heights all from the cache - its cycles, instructions and
// samples each time.
void terrainEvents(uint32_t out[6]);
// On the bench: the march of the frame being prepared in assembly and in C
// alone - how many columns' tops and rows' depths differ, and the cycles of
// each.
void terrainMarchCheck(uint32_t out[4]);
// On the bench: the last march's samples by the way they went - see
// scene3d.cpp's MarchCounts.
void terrainPaths(uint32_t out[7]);
// On the bench: the instructions the land's bands have taken since last
// asked.
uint64_t terrainBandInstructions();
// On the bench: the samples the bands' assembly has left for C since last
// asked.
uint64_t terrainBandHanded();
// On the bench: the maps the bands' assembly has taken up again from those
// it keeps, where the part of the cover changed, since last asked.
uint64_t terrainBandSwitched();


// Where labels for the traffic go in the frame drawn: a point over each
// aircraft in view and not hidden behind a nearer one, nearest first - its
// place on the screen, how far it is, its index in the scene's traffic, and
// how long it looks on the screen. Up to `max`; returns how many.
struct Label {
    float x, y, distance;
    int index;
    float sizePx;
};
int labelAnchors(Label *out, int max);

// Where the aircraft followed is in the frame drawn - the middle of it, and
// how far it reaches from there on the screen - for labels to keep off it;
// false if it is behind the eye.
bool followedOnScreen(float &x, float &y, float &reach);

// An airfield's runway: where its two ends are, in metres east and north of
// somewhere near the airfield (z is ignored), how wide it is, and the names
// on its ends - "09L" - in the same order.
struct RunwaySpec {
    Vec3 end[2];
    float width;
    char name[2][4];
};

// The airfield to draw: its runways, painted and lit, and its beacons,
// VOR/DMEs, where they are. With `layout`, its real taxiways, aprons, stands,
// buildings and tower - in the runways' frame: see airfield_pack.h. Without
// one, the runways alone - or, `madeUp`, each with a taxiway alongside and a
// terminal of piers and stands between them, or beside the one: the bench's
// scenes, as they were measured. With no runways, a made-up airfield of one,
// 09/27, 3000m east from the origin - the one there is to begin with.
void setAirfield(const RunwaySpec *runways, int count, const Vec3 *beacons, int beaconCount,
                 const airfield::Layout *layout = nullptr, bool madeUp = false);

// No airfield of that kind at all: for a view high up, between them.
void clearAirfield();

// Where a camera in the airfield's tower looks out from, the cab's glass:
// false, with no airfield.
bool towerEye(Vec3 &eye);

// Whether a camera at `eye` sees `at` past the airfield's buildings - each
// taken as the box about it square to north, so one at an angle hides a
// little more than it does. With `outside`, any building round the eye
// itself left out: the tower's own, for a camera at its cab.
bool seenPast(Vec3 eye, Vec3 at, bool outside = false);

// How far the tower's camera is raised over its cab, metres, for the
// runways to be seen past the buildings between.
float towerRaised();

// Airfields far off, seen from high up: their runways as strips, their edges
// lit after dark.
void setFarRunways(const RunwaySpec *runways, int count);

// Which end of which runway the landing comes in to, and which the
// departures leave from (runway -1: none). Runways and ends counted as they
// were given; at first, the first end of the first and no departures.
void setLanding(int runway, int end);
void setDeparture(int runway, int end);

// The landing the preview and the bench fly: `t` seconds from 1200m short of
// the threshold on a three degree glide path, through the flare and
// touchdown, down the runway slowing to taxi speed, and off onto its taxiway.
// Lasts landingSeconds(); the wheels touch at landingTouchdown().
Pose landingPose(float t);
float landingSeconds();
float landingTouchdown();

// Aircraft parked nose-in on the terminal's stands: as many as fit in `max`,
// returning how many.
int parkedAtStands(Aircraft *out, int max);

// A busy airport at `t`: everything on the stands, three more arrivals
// following the one landing, departures every minute and a half, and
// aircraft taxiing both ways along every taxiway.
int busyTraffic(float t, Aircraft *out, int max);

// Round the aircraft: `azimuthDeg` from its tail (0 is straight behind, 90 off
// its right side), `elevationDeg` above it, `distanceM` away - what a drag and
// a pinch would change.
Camera orbitCamera(const Pose &pose, float azimuthDeg, float elevationDeg, float distanceM, float fovDeg);

// From a fixed point - beside the runway, the tower - turning to follow it and
// zooming to keep it `fill` of the view's width, `size` times an A320's.
Camera fixedCamera(const Pose &pose, Vec3 eye, float fill, float size = 1);

// The cinema: a director cutting between shots as the landing goes on - the
// long lens up the approach, a low chase on short final, side-on at the
// touchdown zone as the wheels touch, a crane up over the rollout, the tower
// for the taxi in - and moving the camera slowly within each. `shot` is set
// to its name.
Camera cinemaCamera(float t, const char **shot = nullptr, float size = 1);

// A departure off the end set by setDeparture(): `t` seconds from brake
// release - lined up before 0 - through the take-off roll, rotation and the
// climb away.
Pose departurePose(float t);

// What the director goes by, of the aircraft followed at a moment - from the
// live feed or a scripted landing alike.
struct Flight {
    Aircraft aircraft;
    float speedKt = 0;  // over the ground
    float climbFpm = 0;
    bool onGround = false;
    bool departing = false;  // rolling for, or climbing away from, a take-off
    // The runway it is landing on or leaving, counted as setLanding() counts
    // them; -1 for none, when the director simply chases it.
    int runway = -1, end = 0;
};

// The cinema for a live flight: shots picked by what the aircraft is doing -
// out on the approach, short final, the flare and touchdown, the roll-out,
// taxiing in; or lined up, rolling, rotating and climbing away - each held a
// few seconds at least, so the cuts come when something happens rather than
// whenever the feed twitches. A few seconds after a touchdown, it is played
// again in slow motion from another side, as television would.
class Director {
  public:
    Director();
    ~Director();
    Director(const Director &) = delete;
    Director &operator=(const Director &) = delete;

    // The camera at `seconds`, on any steady clock, and the shot's name.
    // While a replay is showing, `shown` is set to where to draw the aircraft
    // followed - where it was - and true returned.
    Camera next(const Flight &flight, float seconds, const char **shot, Pose *shown, bool *replaying);

  private:
    struct State;
    State *s;
};

}  // namespace scene3d
