#include "scene3d.h"

#include <cstdio>
#include <cstdlib>

#include "airfield_pack.h"

#include <math.h>
#include <string.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <vector>

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#ifdef BENCH3D
#include <esp_cpu.h>
#endif

#include "psram_alloc.h"
#endif

namespace scene3d {
namespace {

// Everything here kept in PSRAM on the device, however small: the chip's own
// memory is what WiFi and TLS live on, and the heap gives anything under a
// few kilobytes from there.
#ifdef ESP_PLATFORM
template <typename T>
using Vector = std::vector<T, PsramAllocator<T>>;
#else
template <typename T>
using Vector = std::vector<T>;
#endif

// --- vectors and colours -----------------------------------------------------

Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float length(Vec3 a) { return sqrtf(dot(a, a)); }
float lengthSquared(Vec3 a) { return dot(a, a); }
Vec3 normalize(Vec3 a) {
    float l = length(a);
    return l > 0 ? a * (1 / l) : a;
}
Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }

struct Rgb {
    float r, g, b;
};
Rgb mix(Rgb a, Rgb b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }
Rgb scale(Rgb a, float s) { return {a.r * s, a.g * s, a.b * s}; }

constexpr float PI = 3.14159265f;
constexpr float DEG = PI / 180;

float clamp01(float v) { return std::max(0.0f, std::min(1.0f, v)); }

// --- the light and the air -----------------------------------------------------

// How the world looks with the sun at some height: the sky overhead and the
// haze at the horizon, the grass, how the light falls on a surface facing it
// and on one facing away, the least light on what stands on the airfield -
// after dark, the floodlights' - how far one can see, what a shadow leaves of
// the light, whether the lights are on, and how bright the stars are.
struct Palette {
    Rgb skyTop, haze, grass;
    float ambient, diffuse, floodlit;
    float visibility;  // how far a colour goes before haze has taken 63% of it
    float shadow;      // 1: none
    float lights;      // above a half, on
    float stars;       // of their full brightness
};

Palette mixPalette(const Palette &a, const Palette &b, float t) {
    auto f = [t](float x, float y) { return x + (y - x) * t; };
    return {mix(a.skyTop, b.skyTop, t), mix(a.haze, b.haze, t), mix(a.grass, b.grass, t), f(a.ambient, b.ambient),
            f(a.diffuse, b.diffuse), f(a.floodlit, b.floodlit), f(a.visibility, b.visibility), f(a.shadow, b.shadow),
            f(a.lights, b.lights), f(a.stars, b.stars)};
}

// The sky with the sun well down, in twilight, at golden hour, low, and high
// - and at any height between, a blend of the two either side.
struct Key {
    float elevationDeg;
    Palette palette;
};

const Key KEYS[] = {
    {-12, {{3, 6, 16}, {18, 24, 42}, {8, 11, 10}, 0.14f, 0.10f, 0.36f, 9000, 1, 1, 1}},
    {-4, {{16, 22, 58}, {128, 86, 96}, {20, 26, 24}, 0.20f, 0.16f, 0.34f, 7500, 1, 1, 0.3f}},
    {3, {{40, 60, 124}, {236, 150, 98}, {52, 62, 42}, 0.30f, 0.62f, 0.30f, 6000, 0.85f, 1, 0}},
    {12, {{62, 108, 182}, {216, 204, 194}, {82, 112, 60}, 0.42f, 0.62f, 0, 6500, 0.68f, 0, 0}},
    {30, {{70, 122, 192}, {198, 212, 226}, {94, 124, 66}, 0.50f, 0.60f, 0, 7000, 0.62f, 0, 0}},
};

Palette paletteFor(float elevationDeg) {
    constexpr int N = sizeof(KEYS) / sizeof(KEYS[0]);
    if (elevationDeg <= KEYS[0].elevationDeg) {
        return KEYS[0].palette;
    }
    for (int i = 0; i + 1 < N; i++) {
        if (elevationDeg <= KEYS[i + 1].elevationDeg) {
            float t = (elevationDeg - KEYS[i].elevationDeg) / (KEYS[i + 1].elevationDeg - KEYS[i].elevationDeg);
            return mixPalette(KEYS[i].palette, KEYS[i + 1].palette, t);
        }
    }
    return KEYS[N - 1].palette;
}

// Nothing nearer the eye than this is drawn; and a triangle is clipped this
// many pixels beyond the edges of the view rather than at them, so that no
// clipped edge is ever seen and none is far enough off it to lose precision.
constexpr float NEAR = 0.5f;
constexpr float GUARD = 32;
// How far past the view's sides a triangle may reach before it is cut there,
// in pixels. The drawing stops at the screen's edges anyway: only the near
// plane must cut, and these sides just keep the numbers to what a float
// draws cleanly - cutting at the guard band, a third of a big airfield's
// buildings crossed it, at a few thousand cycles each.
constexpr float WIDE = 4096;

// An aircraft smaller than a pixel and a half from its middle to its nose is
// not drawn at all - only its lights, which are what is seen of it at that
// distance anyway.
constexpr float DOT_BELOW_PX = 1.5f;
// The levels an aircraft is drawn at: the one followed finest of all; others
// near, in a little less; then simply; tiny; and as only its lights.
constexpr int DETAIL_HERO = 0, DETAIL_MEDIUM = 1, DETAIL_SIMPLE = 2, DETAIL_TINY = 3, DETAIL_LIGHTS = 4;

// --- the models ----------------------------------------------------------------

// What a face is besides its shape and colour, set on the pen before it is
// made: how it is painted - as built, or in the body's, the belly's, the
// airline's, the engines' or the wings' colour, or as a cabin window, which a
// freighter hasn't - or, a piece of the ground, as the grass, in the light of
// the time; when it is
// shown - always, or only with the gear down, the flaps or the spoilers out,
// or the propellers turning; which control it moves with, or which propeller
// or rotor it turns with; and, for the blur of turning blades, how much it
// tints what is behind it, in eighths.
enum Paint : uint8_t {
    PAINT_AS_BUILT,
    PAINT_BODY,
    PAINT_BELLY,
    PAINT_LIVERY,
    PAINT_ENGINES,
    PAINT_CABIN,
    PAINT_WINGS,
    PAINT_GRASS
};
enum Show : uint8_t { SHOW_ALWAYS, SHOW_GEAR, SHOW_FLAPS, SHOW_SPOILERS, SHOW_TURNING };
enum Move : uint8_t {
    MOVE_NONE,
    MOVE_AILERON_L,
    MOVE_AILERON_R,
    MOVE_ELEVATOR_L,
    MOVE_ELEVATOR_R,
    MOVE_RUDDER,
    MOVE_RUDDER_2,
    MOVE_CANARD,
    MOVE_SPOILER_L,
    MOVE_SPOILER_R,
    MOVE_SPIN  // and on: spinner i turns with MOVE_SPIN + i
};
constexpr int HINGES = MOVE_SPIN, MAX_SPINNERS = 8;

struct Pen {
    uint8_t paint = PAINT_AS_BUILT, show = SHOW_ALWAYS, move = MOVE_NONE, blur = 0;
};
Pen g_pen;

// The pen as it is with one thing changed - for as long as a Using lasts.
Pen painted(uint8_t paint) {
    Pen p = g_pen;
    p.paint = paint;
    return p;
}
Pen moving(uint8_t move) {
    Pen p = g_pen;
    p.move = move;
    return p;
}
Pen shown(uint8_t show) {
    Pen p = g_pen;
    p.show = show;
    return p;
}

struct Using {
    Pen was;
    explicit Using(Pen pen) : was(g_pen) { g_pen = pen; }
    ~Using() { g_pen = was; }
    Using(const Using &) = delete;
    Using &operator=(const Using &) = delete;
};

struct ModelTri {
    Vec3 v[3];
    Rgb color;
    // A marking on the ground: how narrow it is, in metres - left out of a
    // frame where it would be less than half a pixel wide.
    float fine = 1e9f;
    uint8_t paint = PAINT_AS_BUILT, show = SHOW_ALWAYS, move = MOVE_NONE, blur = 0;
};

// The airfield's ground and buildings as a frame reads them: a third the size
// of a ModelTri, for a frame reads thousands of them out of PSRAM, which -
// measured on the device - was most of what the ground and the buildings
// cost. Their corners in whole units (Models.unit, a few centimetres) east,
// north and up of the airfield's middle, the same grid for all of them, so a
// corner two triangles share is still shared; their colours in whole levels.
// The ground is all flat, at nought.
struct GroundTri {
    int16_t x[3], y[3];
    uint8_t rgb[3];
    uint8_t grass;  // 1: the grass, coloured as the light has it
    float fine;     // as ModelTri's
};
// A building's, its unit normal in 32767ths.
struct SolidTri {
    int16_t v[3][3];
    int16_t n[3];
    uint8_t rgb[3];
    uint8_t spare;
};

struct ModelLine {
    Vec3 a, b;
    Rgb color;
};

// A face, wound so that its normal points away from `inside` - the middle of
// the part it is a face of - for its back to be culled by.
void face(Vector<ModelTri> &out, Vec3 a, Vec3 b, Vec3 c, Vec3 inside, Rgb color) {
    if (dot(cross(b - a, c - a), (a + b + c) * (1.0f / 3) - inside) < 0) {
        std::swap(b, c);
    }
    out.push_back({{a, b, c}, color, 1e9f, g_pen.paint, g_pen.show, g_pen.move, g_pen.blur});
}

void faceQuad(Vector<ModelTri> &out, Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 inside, Rgb color) {
    face(out, a, b, c, inside, color);
    face(out, a, c, d, inside, color);
}

// A tube along `axis` from `origin`, its cross-section the circle of `u` and
// `v`: a ring of `sides` at each of rings[i] = {how far along, radius}.
struct Ring {
    float along, r;
};

Vec3 ringPoint(Vec3 origin, Vec3 axis, Vec3 u, Vec3 v, Ring ring, int k, int sides) {
    float a = (k + 0.5f) * 2 * PI / sides;
    return origin + axis * ring.along + (u * cosf(a) + v * sinf(a)) * ring.r;
}

void tube(Vector<ModelTri> &out, Vec3 origin, Vec3 axis, Vec3 u, Vec3 v, const Ring *rings, int n, int sides,
          Rgb color) {
    for (int i = 0; i + 1 < n; i++) {
        Vec3 inside = origin + axis * ((rings[i].along + rings[i + 1].along) / 2);
        for (int k = 0; k < sides; k++) {
            faceQuad(out, ringPoint(origin, axis, u, v, rings[i], k, sides),
                     ringPoint(origin, axis, u, v, rings[i], k + 1, sides),
                     ringPoint(origin, axis, u, v, rings[i + 1], k + 1, sides),
                     ringPoint(origin, axis, u, v, rings[i + 1], k, sides), inside, color);
        }
    }
}

// A tube's end closed with a disc, facing away from `inside`.
void disc(Vector<ModelTri> &out, Vec3 origin, Vec3 axis, Vec3 u, Vec3 v, Ring ring, int sides, Vec3 inside,
          Rgb color) {
    Vec3 middle = origin + axis * ring.along;
    for (int k = 0; k < sides; k++) {
        face(out, middle, ringPoint(origin, axis, u, v, ring, k, sides),
             ringPoint(origin, axis, u, v, ring, k + 1, sides), inside, color);
    }
}

void box(Vector<ModelTri> &out, Vec3 lo, Vec3 hi, Rgb sides, Rgb top) {
    Vec3 c[8];
    for (int i = 0; i < 8; i++) {
        c[i] = {i & 1 ? hi.x : lo.x, i & 2 ? hi.y : lo.y, i & 4 ? hi.z : lo.z};
    }
    Vec3 inside = (lo + hi) * 0.5f;
    faceQuad(out, c[0], c[1], c[3], c[2], inside, sides);  // bottom
    faceQuad(out, c[4], c[5], c[7], c[6], inside, top);
    faceQuad(out, c[0], c[1], c[5], c[4], inside, sides);
    faceQuad(out, c[2], c[3], c[7], c[6], inside, sides);
    faceQuad(out, c[0], c[2], c[6], c[4], inside, sides);
    faceQuad(out, c[1], c[3], c[7], c[5], inside, sides);
}

// A wing, a tailplane or a fin, between two sections: a lozenge of leading
// edge, the thickest point a third of the way back, and trailing edge, made
// `thick` along `across`.
struct Section {
    Vec3 le, te;
    float thick;
};

void slab(Vector<ModelTri> &out, Section root, Section tip, Vec3 across, Rgb color) {
    auto points = [&](const Section &s, Vec3 p[4]) {
        Vec3 mid = lerp(s.le, s.te, 0.35f);
        p[0] = s.le;
        p[1] = mid + across * (s.thick / 2);
        p[2] = s.te;
        p[3] = mid - across * (s.thick / 2);
    };
    Vec3 a[4], b[4];
    points(root, a);
    points(tip, b);
    Vec3 inside = (lerp(root.le, root.te, 0.35f) + lerp(tip.le, tip.te, 0.35f)) * 0.5f;
    for (int i = 0; i < 4; i++) {
        faceQuad(out, a[i], a[(i + 1) % 4], b[(i + 1) % 4], b[i], inside, color);
    }
    faceQuad(out, b[0], b[1], b[2], b[3], inside, color);  // the tip
}

// --- the aircraft ----------------------------------------------------------------
// A model for each family, of one type of it, in metres: x forward, y to its
// left, z up, from the point on the ground under its main wheels. Each is
// built in full, for the one followed; a little less, for another near it;
// simply - a fifth of the faces - for one further off; and tiny, some thirty
// faces, for one a few pixels long.

const Rgb WHITE = {236, 238, 242}, BELLY = {206, 210, 216}, WING = {196, 200, 206}, TAIL = {28, 66, 140},
          DARK = {36, 42, 56}, FAN = {44, 46, 52}, GEAR = {80, 80, 84}, TYRE = {30, 30, 32}, EXHAUST = {78, 78, 82},
          FLAP = {197, 201, 207}, SPOILER = {194, 198, 204}, BLADE = {40, 40, 44}, NOZZLE = {66, 60, 56},
          GLASS = {40, 52, 70};

const Vec3 AXIS_X = {1, 0, 0}, AXIS_Y = {0, 1, 0}, AXIS_Z = {0, 0, 1};

// Where a hinged control turns about - a point on its hinge and the way the
// hinge runs - and how far, in radians, its full travel turns it.
struct Hinge {
    Vec3 origin, axis;
    float travel = 0;
};

// A propeller or a rotor: its hub, the way its shaft points, and how fast it
// is seen to turn - radians a second, far slower than the real thing, as a
// camera's shutter catches it.
struct Spinner {
    Vec3 hub, axis;
    float rate;
};

// Its lights: red on the left wingtip, green on the right and white at the
// tail; the red beacons over and under it flashing in turn; the white strobes
// at the tips and tail flashing twice; and the landing lights, a glare when
// they point at you.
enum class Lamp { NAV, BEACON_TOP, BEACON_BELLY, STROBE, LANDING };

struct PlaneLight {
    Vec3 at;
    Rgb color;
    float size;
    Lamp lamp;
};

// A model as its corners and the faces between them, so that each corner is
// placed and projected once a frame rather than once for every face it is a
// corner of - four or five times over, on a model like these. Kept small,
// eleven bytes a face and seven a corner: corners to the centimetre in
// 16 bits, normals in 8, colours an index into a palette, and how a face is
// painted, when it is shown and how see-through it is in a byte between them.
struct Mesh {
    Vector<int16_t> corners;  // x, y and z, three to a corner
    Vector<uint8_t> moves;    // what each corner moves with
    Vector<uint16_t> faces;   // three corners each
    Vector<int8_t> normals;   // outwards: x, y and z of 127, three to a face
    Vector<uint8_t> colors;   // into g_palette
    Vector<uint8_t> looks;    // paint in bits 0-2, show in 3-5, blur in 6-7
    bool moving = false;      // any corner moves

    size_t cornerCount() const { return moves.size(); }
    size_t faceCount() const { return colors.size(); }
    Vec3 corner(size_t i) const {
        const int16_t *c = &corners[i * 3];
        return {c[0] * 0.01f, c[1] * 0.01f, c[2] * 0.01f};
    }
    Vec3 normal(size_t f) const {
        const int8_t *n = &normals[f * 3];
        return {n[0] * (1 / 127.0f), n[1] * (1 / 127.0f), n[2] * (1 / 127.0f)};
    }
    uint8_t paint(size_t f) const { return looks[f] & 7; }
    uint8_t show(size_t f) const { return looks[f] >> 3 & 7; }
    uint8_t blur(size_t f) const {  // eighths: 0, or 2 to 4
        int b = looks[f] >> 6;
        return b ? (uint8_t)(b + 1) : 0;
    }
};

Vector<Rgb> g_palette;

uint8_t paletteIndex(Rgb c) {
    for (size_t i = 0; i < g_palette.size(); i++) {
        if (g_palette[i].r == c.r && g_palette[i].g == c.g && g_palette[i].b == c.b) {
            return (uint8_t)i;
        }
    }
    if (g_palette.size() >= 255) {
        return 0;
    }
    g_palette.push_back(c);
    return (uint8_t)(g_palette.size() - 1);
}

// A family's model at each level of detail, and what goes with it: the
// hinges its controls turn on, its propellers and rotors, its lights, the box
// it fits in, a shadow of a few faces, and where its main wheels are.
struct Design {
    Mesh lod[4];
    Hinge hinge[HINGES];
    Spinner spinners[MAX_SPINNERS];
    int spinnerCount = 0;
    Vector<PlaneLight> lights;
    Vec3 lo{}, hi{};
    Vec3 middle{};      // of the box
    float radius = 24;  // from its middle, the box's corners
    float halfLength = 18.8f;
    float length = 37.6f;  // nose to tail
    Vec3 blob[3][4]{};     // its shadow, simply: body and wings, flat
    float gearY = 3.8f;
    float noseGear = 12.4f;  // how far ahead of the main wheels its nose wheel is
    // Where the trails of its engines start, at height - none for
    // propellers, whose exhaust is too warm and wet for nothing.
    Vec3 exhaust[4]{};
    int exhausts = 0;
};

// 25KB: on the heap - in the PSRAM, a block this size - not in the chip's own
// memory as a static would be.
std::vector<Design> &designs() {
    static std::vector<Design> all((int)Family::COUNT);
    return all;
}

// --- bodies -----------------------------------------------------------------------

// A body's cross-section: where along it, its half-width, the height of its
// middle and its half-height.
struct Station {
    float x, w, z, h;
};

// Facet k runs from corner k to corner k + 1 and faces (k + 1) / sides of the
// way round from the top, towards its left.
Vec3 stationPoint(const Station &s, int k, int sides) {
    float a = (k + 0.5f) * 2 * PI / sides;
    return {s.x, s.w * sinf(a), s.z + s.h * cosf(a)};
}

// The A320's body, which an airliner's is made from, in its own measures: a
// rounded nose, the windscreen across stations 2 and 3, the cabin a
// cylinder, and the tail cone sweeping up. Along the nose and the cone, of
// their lengths; across, of the cabin's half-width; the nose's middle below
// the cabin's, of its half-width; the cone's above, of its rise.
const float NOSE_AT[] = {0, 0.078f, 0.208f, 0.403f, 0.662f, 1.0f},
            NOSE_R[] = {0.041f, 0.279f, 0.482f, 0.685f, 0.863f, 0.980f},
            NOSE_DROP[] = {0.279f, 0.264f, 0.213f, 0.152f, 0.076f, 0.025f};
const float CONE_AT[] = {0, 0.219f, 0.439f, 0.658f, 0.851f, 1.0f},
            CONE_R[] = {1.0f, 0.939f, 0.787f, 0.558f, 0.315f, 0.112f},
            CONE_RISE[] = {0, 0.104f, 0.278f, 0.522f, 0.765f, 1.0f};

// An airliner's body: its nose at x = nose, the end of its tail cone at x =
// tail; the cabin's half-width and half-height and the height of its middle;
// how long the nose and the cone are, and how far the cone rises.
struct BodyShape {
    float nose, tail, w, h, z, noseLength, coneLength, coneRise;
};

int stationsOf(const BodyShape &b, Station out[12]) {
    for (int i = 0; i < 6; i++) {
        out[i] = {b.nose - NOSE_AT[i] * b.noseLength, b.w * NOSE_R[i], b.z - NOSE_DROP[i] * b.w, b.h * NOSE_R[i]};
    }
    float coneFrom = b.tail + b.coneLength;
    for (int i = 0; i < 6; i++) {
        out[6 + i] = {coneFrom - CONE_AT[i] * b.coneLength, b.w * CONE_R[i], b.z + CONE_RISE[i] * b.coneRise,
                      b.h * CONE_R[i]};
    }
    return 12;
}

// A body through stations - all of them, at `roundness` sides, for the one
// followed; for one near, the most of them at five eighths of the sides; and
// simply, a few at six - its faces towards the ground the belly's colour and
// the rest the body's, closed at each end. With `screen` one of the
// stations, the windscreen: dark panes wrapping round the nose's upper sides
// across it and the next.
void body(Vector<ModelTri> &out, int level, const Station *st, int n, int screen, int roundness = 16) {
    static const int ALL12[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, MEDIUM12[] = {0, 2, 3, 5, 6, 8, 9, 11},
                     FEW12[] = {0, 3, 5, 6, 8, 11};
    int use[32], count = 0;
    if (n == 12) {
        const int *from = level == DETAIL_HERO ? ALL12 : level == DETAIL_MEDIUM ? MEDIUM12 : FEW12;
        count = level == DETAIL_HERO ? 12 : level == DETAIL_MEDIUM ? 8 : 6;
        memcpy(use, from, sizeof(int) * count);
    } else {
        for (int i = 0; i < n && count < 32; i++) {
            if (level <= DETAIL_MEDIUM || i == 0 || i == n - 1 || i % 2 == 0) {
                use[count++] = i;
            }
        }
    }
    int sides = level == DETAIL_HERO ? roundness : level == DETAIL_MEDIUM ? std::max(6, (roundness * 5 / 8) & ~1) : 6;
    bool full = level <= DETAIL_MEDIUM;
    for (int i = 0; i + 1 < count; i++) {
        const Station &s0 = st[use[i]], &s1 = st[use[i + 1]];
        Vec3 inside = {(s0.x + s1.x) / 2, 0, (s0.z + s1.z) / 2};
        for (int k = 0; k < sides; k++) {
            float facing = (k + 1) * 360.0f / sides;
            bool belly = facing >= 150 && facing <= 210;
            Rgb color = belly ? BELLY : WHITE;
            uint8_t paint = belly ? PAINT_BELLY : PAINT_BODY;
            float off = std::min(facing, 360 - facing);
            if (full && screen >= 0 &&
                ((use[i] == screen && off > 15 && off < 70) || (use[i] == screen + 1 && off > 60 && off < 75))) {
                color = DARK;
                paint = PAINT_AS_BUILT;
            }
            Using pen(painted(paint));
            faceQuad(out, stationPoint(s0, k, sides), stationPoint(s0, k + 1, sides), stationPoint(s1, k + 1, sides),
                     stationPoint(s1, k, sides), inside, color);
        }
    }
    const Station &nose = st[use[0]], &tail = st[use[count - 1]];
    Using pen(painted(PAINT_BODY));
    float point = std::min(0.25f, 3 * nose.w);
    for (int k = 0; k < sides; k++) {
        face(out, {nose.x + point, 0, nose.z}, stationPoint(nose, k, sides), stationPoint(nose, k + 1, sides),
             {nose.x - 1, 0, nose.z}, WHITE);
        face(out, {tail.x - std::min(0.2f, tail.w), 0, tail.z}, stationPoint(tail, k, sides),
             stationPoint(tail, k + 1, sides), {tail.x + 1, 0, tail.z}, WHITE);
    }
}

// A pod of the body's colour - a sponson over the main gear, an engine's
// fairing, a canopy - through its own stations, at fewer sides.
void pod(Vector<ModelTri> &out, int level, const Station *st, int n, Rgb color, uint8_t paint) {
    if (level == DETAIL_TINY) {
        return;
    }
    int sides = level == DETAIL_HERO ? 10 : level == DETAIL_MEDIUM ? 8 : 4;
    Using pen(painted(paint));
    for (int i = 0; i + 1 < n; i++) {
        const Station &s0 = st[i], &s1 = st[i + 1];
        Vec3 inside = {(s0.x + s1.x) / 2, (s0.w + s1.w) * 0, (s0.z + s1.z) / 2};
        for (int k = 0; k < sides; k++) {
            auto at = [&](const Station &s, int j) {
                float a = (j + 0.5f) * 2 * PI / sides;
                return Vec3{s.x, s.w * sinf(a), s.z + s.h * cosf(a)};
            };
            faceQuad(out, at(s0, k), at(s0, k + 1), at(s1, k + 1), at(s1, k), inside, color);
        }
    }
}

// The same, moved off the middle to y.
void podAt(Vector<ModelTri> &out, int level, const Station *st, int n, float y, Rgb color, uint8_t paint) {
    if (level == DETAIL_TINY) {
        return;
    }
    Vector<ModelTri> one;
    pod(one, level, st, n, color, paint);
    for (ModelTri &t : one) {
        for (Vec3 &v : t.v) {
            v.y += y;
        }
        out.push_back(t);
    }
}

// The body's half-width, middle and half-height at x, between the stations
// either side.
void bodyAt(const Station *st, int n, float x, float &w, float &z, float &h) {
    for (int i = 0; i + 1 < n; i++) {
        const Station &a = st[i], &b = st[i + 1];
        if (x <= a.x && x >= b.x && a.x > b.x) {
            float t = (a.x - x) / (a.x - b.x);
            w = a.w + (b.w - a.w) * t;
            z = a.z + (b.z - a.z) * t;
            h = a.h + (b.h - a.h) * t;
            return;
        }
    }
    const Station &m = st[n / 2];
    w = m.w, z = m.z, h = m.h;
}

// How far out a body's side is at height `z` above its middle: on the facets
// it is drawn with, not the oval they are cut from - a window strip set on
// the oval sank into the flat facets here and there, and broke up into dots
// at a distance.
float sideAt(float w, float h, int sides, float z) {
    float yWas = 0, zWas = h;  // the top
    for (int k = 0; k < sides; k++) {
        float a = (k + 0.5f) * 2 * PI / sides;
        if (a > PI) {
            break;
        }
        float y = w * sinf(a), zc = h * cosf(a);
        if (z <= zWas && z >= zc) {
            float t = zWas > zc ? (zWas - z) / (zWas - zc) : 0;
            return yWas + (y - yWas) * t;
        }
        yWas = y, zWas = zc;
    }
    return yWas;
}

// The cabin's windows: a dark strip along each side from x0 to x1, `up`
// above the body's middle there and `tall` high, standing 5cm proud of the
// side - far enough to stay in front of it at any distance the depth can
// tell apart.
void windows(Vector<ModelTri> &out, const Station *st, int n, int sides, float x0, float x1, float up, float tall) {
    Using pen(painted(PAINT_CABIN));
    // In pieces of no more than 6m, so that it follows the body.
    int pieces = std::max(1, (int)ceilf(fabsf(x0 - x1) / 6));
    for (float side : {1.0f, -1.0f}) {
        auto corner = [&](float x, float dz) {
            float w, z, h;
            bodyAt(st, n, x, w, z, h);
            return Vec3{x, side * (sideAt(w, h, sides, up + dz) + 0.05f), z + up + dz};
        };
        for (int i = 0; i < pieces; i++) {
            float a = x0 + (x1 - x0) * i / pieces, b = x0 + (x1 - x0) * (i + 1) / pieces;
            float w, z, h;
            bodyAt(st, n, (a + b) / 2, w, z, h);
            faceQuad(out, corner(a, -tall / 2), corner(b, -tall / 2), corner(b, tall / 2), corner(a, tall / 2),
                     {(a + b) / 2, 0, z + up}, DARK);
        }
    }
}

// --- wings and tails ------------------------------------------------------------

// A wing's edges at a distance out along it: where its leading and trailing
// edges are, how high it is there and how thick.
struct WingStation {
    float y, le, te, z, thick;
};

// A wing from where it meets the body, through its kink - which can be at
// the root, for none - to its tip; how far out its flaps go, of root to tip,
// none for 0; where its ailerons are, of root to tip, and how much of the
// chord they take; whether it has spoilers.
struct Wing {
    WingStation root{}, kink{}, tip{};
    float flapsTo = 0;
    float aileronFrom = 0.72f, aileronTo = 0.94f, aileronChord = 0.26f;
    bool spoilers = false;
    // On a delta, the inner part of the trailing edge is the elevator - the
    // elevons inboard pitch it, those outboard roll it.
    bool elevons = false;
};

WingStation wingAt(const Wing &w, float y) {
    const WingStation &a = y <= w.kink.y ? w.root : w.kink, &b = y <= w.kink.y ? w.kink : w.tip;
    float t = b.y > a.y ? clamp01((y - a.y) / (b.y - a.y)) : 0;
    return {y, a.le + (b.le - a.le) * t, a.te + (b.te - a.te) * t, a.z + (b.z - a.z) * t,
            a.thick + (b.thick - a.thick) * t};
}

// Its section at y on side s - 1 left, -1 right - from `from` to `to` of the
// chord, 0 the leading edge and 1 the trailing.
Section chordAt(const Wing &w, float y, float s, float from = 0, float to = 1, float thick = -1) {
    WingStation e = wingAt(w, y);
    float c = e.te - e.le;
    return {{e.le + c * from, s * y, e.z}, {e.le + c * to, s * y, e.z}, thick >= 0 ? thick : e.thick};
}

// A hinge along a line, pointing towards +y - or +z - whichever end it is
// given from, so that a control's travel turns both sides alike.
Hinge hingeAlong(Vec3 a, Vec3 b, float travel, Vec3 towards) {
    Vec3 axis = normalize(b - a);
    if (dot(axis, towards) < 0) {
        axis = axis * -1;
    }
    return {a, axis, travel};
}

// The two wings, in pieces at the kink and at the ailerons' ends - each
// aileron the back of the chord there, hinged - with flaps and spoilers on
// an airliner's. Simply: one piece a side.
void wings(Vector<ModelTri> &out, int level, const Wing &w, Design &d) {
    Using pen(painted(PAINT_WINGS));
    if (level >= DETAIL_SIMPLE) {
        for (float s : {1.0f, -1.0f}) {
            if (w.kink.y > w.root.y + 0.01f) {
                slab(out, chordAt(w, w.root.y, s), chordAt(w, w.kink.y, s), AXIS_Z, WING);
                slab(out, chordAt(w, w.kink.y, s), chordAt(w, w.tip.y, s), AXIS_Z, WING);
            } else {
                slab(out, chordAt(w, w.root.y, s), chordAt(w, w.tip.y, s), AXIS_Z, WING);
            }
        }
        return;
    }
    float span = w.tip.y - w.root.y;
    float a0 = std::max(w.kink.y, w.root.y + span * w.aileronFrom), a1 = w.root.y + span * w.aileronTo;
    float cut = 1 - w.aileronChord;
    for (float s : {1.0f, -1.0f}) {
        if (w.elevons) {
            // A delta: the leading part whole, the trailing edge elevons -
            // inboard pitching, outboard rolling.
            float mid = w.root.y + span * 0.48f;
            slab(out, chordAt(w, w.root.y, s, 0, cut), chordAt(w, w.tip.y, s, 0, cut), AXIS_Z, WING);
            {
                Using move(moving(s > 0 ? MOVE_ELEVATOR_L : MOVE_ELEVATOR_R));
                slab(out, chordAt(w, w.root.y + 0.2f, s, cut, 1, 0.08f), chordAt(w, mid, s, cut, 1, 0.06f), AXIS_Z,
                     WING);
            }
            {
                Using move(moving(s > 0 ? MOVE_AILERON_L : MOVE_AILERON_R));
                slab(out, chordAt(w, mid, s, cut, 1, 0.06f), chordAt(w, w.root.y + span * 0.96f, s, cut, 1, 0.04f),
                     AXIS_Z, WING);
            }
            Section e0 = chordAt(w, w.root.y + 0.2f, s, cut), e1 = chordAt(w, mid, s, cut),
                    e2 = chordAt(w, w.root.y + span * 0.96f, s, cut);
            d.hinge[s > 0 ? MOVE_ELEVATOR_L : MOVE_ELEVATOR_R] = hingeAlong(e0.le, e1.le, 0.35f, AXIS_Y);
            d.hinge[s > 0 ? MOVE_AILERON_L : MOVE_AILERON_R] = hingeAlong(e1.le, e2.le, s > 0 ? -0.3f : 0.3f, AXIS_Y);
            continue;
        }
        if (w.kink.y > w.root.y + 0.01f) {
            slab(out, chordAt(w, w.root.y, s), chordAt(w, w.kink.y, s), AXIS_Z, WING);
        }
        slab(out, chordAt(w, w.kink.y, s), chordAt(w, a0, s), AXIS_Z, WING);
        slab(out, chordAt(w, a0, s, 0, cut), chordAt(w, a1, s, 0, cut), AXIS_Z, WING);
        slab(out, chordAt(w, a1, s), chordAt(w, w.tip.y, s), AXIS_Z, WING);
        {
            Using move(moving(s > 0 ? MOVE_AILERON_L : MOVE_AILERON_R));
            float t0 = wingAt(w, a0).thick * 0.45f, t1 = wingAt(w, a1).thick * 0.45f;
            slab(out, chordAt(w, a0, s, cut, 1, t0), chordAt(w, a1, s, cut, 1, t1), AXIS_Z, WING);
        }
        // Rolling right: the left aileron's edge down, the right's up.
        d.hinge[s > 0 ? MOVE_AILERON_L : MOVE_AILERON_R] =
            hingeAlong(chordAt(w, a0, s, cut).le, chordAt(w, a1, s, cut).le, s > 0 ? -0.3f : 0.3f, AXIS_Y);

        // The flaps, out of the trailing edge and down 25 degrees, inboard
        // and out to flapsTo; and the spoilers, panels standing up off the
        // top of the wing ahead of them.
        if (w.flapsTo > 0) {
            Using flaps(shown(SHOW_FLAPS));
            float out1 = w.root.y + span * std::min(w.flapsTo, w.aileronFrom - 0.02f);
            auto flap = [&](float y) {
                WingStation e = wingAt(w, y);
                float chord = (e.te - e.le) * -0.28f;
                Vec3 te = {e.te, s * y, e.z - 0.05f};
                return Section{te, te + Vec3{-chord * 0.906f, 0, -chord * 0.423f}, std::max(0.06f, e.thick * 0.16f)};
            };
            float from = w.root.y + 0.1f;
            if (w.kink.y > from + 0.5f && w.kink.y < out1) {
                slab(out, flap(from), flap(w.kink.y), AXIS_Z, FLAP);
                slab(out, flap(w.kink.y), flap(out1), AXIS_Z, FLAP);
            } else {
                slab(out, flap(from), flap(out1), AXIS_Z, FLAP);
            }
            if (w.spoilers) {
                // Made lying on the wing, and raised on their hinge - their
                // front edge - as far as the touchdown has them: up to 50
                // degrees, the ground spoilers' travel.
                Using spoilers(shown(SHOW_SPOILERS));
                Using raised(moving(s > 0 ? MOVE_SPOILER_L : MOVE_SPOILER_R));
                auto hingeAt = [&](float y) {
                    WingStation e = wingAt(w, y);
                    return Vec3{e.te + (e.le - e.te) * 0.3f, s * y, e.z + e.thick * 0.3f};
                };
                auto spoiler = [&](float y) {
                    WingStation e = wingAt(w, y);
                    Vec3 hinge = hingeAt(y);
                    return Section{hinge, hinge + Vec3{-(e.le - e.te) / 4.5f, 0, 0}, 0.06f};
                };
                float sFrom = w.root.y + span * 0.05f, sTo = w.root.y + span * std::min(w.flapsTo, 0.6f);
                if (w.kink.y > sFrom && w.kink.y < sTo) {
                    slab(out, spoiler(sFrom), spoiler(w.kink.y), AXIS_Z, SPOILER);
                    slab(out, spoiler(w.kink.y), spoiler(sTo), AXIS_Z, SPOILER);
                } else {
                    slab(out, spoiler(sFrom), spoiler(sTo), AXIS_Z, SPOILER);
                }
                d.hinge[s > 0 ? MOVE_SPOILER_L : MOVE_SPOILER_R] = hingeAlong(hingeAt(sFrom), hingeAt(sTo), 0.87f, AXIS_Y);
            }
        }
    }
}

// What stands at a wingtip: a sharklet or a winglet turned up from it, as
// tall as `height`; a fence above and below; or nothing.
enum Tip : uint8_t { TIP_PLAIN, TIP_SHARKLET, TIP_FENCE };

void wingtips(Vector<ModelTri> &out, int level, const Wing &w, Tip tip, float height) {
    if (tip == TIP_PLAIN || level >= DETAIL_SIMPLE) {
        return;
    }
    Using pen(painted(PAINT_LIVERY));
    for (float s : {1.0f, -1.0f}) {
        Section t = chordAt(w, w.tip.y, s, 0, 1, 0.12f);
        if (tip == TIP_SHARKLET) {
            Vec3 lift = {0, s * 0.17f * height, height};
            slab(out, t, {t.le + lift + Vec3{-0.9f * height, 0, 0}, t.te + lift + Vec3{-0.5f * height, 0, 0}, 0.10f},
                 AXIS_Y, TAIL);
        } else {
            Vec3 up = {0, 0, height}, down = {0, 0, -height * 0.45f};
            float c = t.le.x - t.te.x;
            Section top = {t.le + up + Vec3{-0.35f * c, 0, 0}, t.te + up, 0.08f},
                    bottom = {t.le + down + Vec3{-0.2f * c, 0, 0}, t.te + down, 0.08f};
            slab(out, bottom, top, AXIS_Y, TAIL);
        }
    }
}

// A tailplane's half, or a fin, from root to tip - `across` its thickness -
// with its back `moving` of the chord hinged and moving with `move`, as far
// as `travel` either way. All of it moving, `moving` 1, it pivots a little
// ahead of the middle of its chord.
void surface(Vector<ModelTri> &out, int level, Section root, Section tip, Vec3 across, Rgb color, float movingPart,
             uint8_t move, float travel, Vec3 towards, Design &d) {
    if (level >= DETAIL_SIMPLE || move == MOVE_NONE) {
        slab(out, root, tip, across, color);
        return;
    }
    auto at = [](const Section &s, float f) { return lerp(s.le, s.te, f); };
    if (movingPart >= 0.99f) {
        {
            Using pen(moving(move));
            slab(out, root, tip, across, color);
        }
        d.hinge[move] = hingeAlong(at(root, 0.42f), at(tip, 0.42f), travel, towards);
        return;
    }
    float cut = 1 - movingPart;
    slab(out, {root.le, at(root, cut), root.thick}, {tip.le, at(tip, cut), tip.thick}, across, color);
    {
        Using pen(moving(move));
        slab(out, {at(root, cut), root.te, root.thick * 0.55f}, {at(tip, cut), tip.te, tip.thick * 0.55f}, across,
             color);
    }
    d.hinge[move] = hingeAlong(at(root, cut), at(tip, cut), travel, towards);
}

// The tailplane, both halves: from its root section on the left and its tip.
void tailplane(Vector<ModelTri> &out, int level, Section root, Section tip, float elevator, Design &d) {
    Using pen(painted(PAINT_WINGS));
    for (float s : {1.0f, -1.0f}) {
        auto side = [s](Section v) {
            v.le.y *= s;
            v.te.y *= s;
            return v;
        };
        surface(out, level, side(root), side(tip), AXIS_Z, WING, elevator, s > 0 ? MOVE_ELEVATOR_L : MOVE_ELEVATOR_R,
                0.35f, AXIS_Y, d);
    }
}

// A fin - the airline's colours - its rudder hinged on the back.
void fin(Vector<ModelTri> &out, int level, Section root, Section tip, float rudder, Design &d,
         uint8_t move = MOVE_RUDDER, Vec3 across = AXIS_Y) {
    Using pen(painted(PAINT_LIVERY));
    Vec3 up = normalize(lerp(tip.le, tip.te, 0.5f) - lerp(root.le, root.te, 0.5f));
    surface(out, level, root, tip, across, TAIL, rudder, move, 0.4f, up, d);
}

// --- engines ---------------------------------------------------------------------

// A turbofan's nacelle, the lip of its intake at `front`, pointing along +x:
// the cowl swelling from the lip and tapering to the exhaust - the A320's,
// in this one's length and radius - the fan a dark disc in the intake.
void fanNacelle(Vector<ModelTri> &out, int level, Vec3 front, float length, float radius) {
    Using pen(painted(PAINT_ENGINES));
    if (level <= DETAIL_MEDIUM) {
        int round = level == DETAIL_HERO ? 10 : 8;
        const float AT[] = {0, 0.075f, 0.472f, 0.849f, 1.0f}, R[] = {0.89f, 1.0f, 0.98f, 0.76f, 0.55f};
        Ring rings[5];
        for (int i = 0; i < 5; i++) {
            rings[i] = {-AT[i] * length, R[i] * radius};
        }
        Vec3 inside = front - AXIS_X * (length * 0.5f);
        tube(out, front, AXIS_X, AXIS_Z, AXIS_Y, rings, 5, round, WHITE);
        Using plain(painted(PAINT_AS_BUILT));
        disc(out, front, AXIS_X, AXIS_Z, AXIS_Y, rings[0], round, inside, FAN);
        disc(out, front, AXIS_X, AXIS_Z, AXIS_Y, rings[4], round, inside, EXHAUST);
        if (level == DETAIL_HERO) {
            const Ring SPINNER[] = {{-0.009f * length, 0.29f * radius}, {0.085f * length, 0.02f}};
            tube(out, front, AXIS_X, AXIS_Z, AXIS_Y, SPINNER, 2, 8, BELLY);
        }
    } else {
        const Ring RINGS[] = {{0, radius}, {-length, 0.68f * radius}};
        tube(out, front, AXIS_X, AXIS_Z, AXIS_Y, RINGS, 2, 4, WHITE);
        Using plain(painted(PAINT_AS_BUILT));
        disc(out, front, AXIS_X, AXIS_Z, AXIS_Y, RINGS[0], 4, front - AXIS_X * length, FAN);
    }
}

// Its pylon, up from the top of the nacelle into the wing at height `wingZ`.
void pylonUp(Vector<ModelTri> &out, int level, Vec3 front, float length, float radius, float wingZ) {
    if (level >= DETAIL_SIMPLE) {
        return;
    }
    Using pen(painted(PAINT_BODY));
    float z0 = front.z + 0.8f * radius, z1 = wingZ + 0.3f, k = radius / 1.1f;
    slab(out, {{front.x - 0.36f * length, front.y, z0}, {front.x - 1.04f * length, front.y, z0}, 0.35f * k},
         {{front.x - 0.74f * length, front.y, z1}, {front.x - 1.26f * length, front.y, z1}, 0.3f * k}, AXIS_Y, WHITE);
}

// A turboprop's nacelle, its front just behind the spinner at `front`: round
// at the front, deep enough to take the wing over it, tapering behind.
void propNacelle(Vector<ModelTri> &out, int level, Vec3 front, float length, float halfWidth, float halfHeight) {
    const float AT[] = {0, 0.06f, 0.22f, 0.62f, 1.0f}, W[] = {0.62f, 0.95f, 1.0f, 0.86f, 0.25f},
                H[] = {0.55f, 0.92f, 1.0f, 0.92f, 0.3f}, UP[] = {0, 0, 0.06f, 0.1f, 0.35f};
    Station st[5];
    for (int i = 0; i < 5; i++) {
        st[i] = {front.x - AT[i] * length, halfWidth * W[i], front.z + UP[i] * halfHeight, halfHeight * H[i]};
    }
    podAt(out, level, st, 5, front.y, WHITE, PAINT_ENGINES);
}

// A propeller or a rotor of `blades`, turning about `axis` through `hub` -
// `u` and `v` across it - `radius` to its tips; the spinner over its hub; and,
// while it turns, a blur: a disc of a quarter or half the pixels.
void propeller(Vector<ModelTri> &out, int level, Vec3 hub, Vec3 axis, Vec3 u, Vec3 v, float radius, int blades,
               float chord, float pitch, float rate, uint8_t blur, Design &d, float spinnerR = -1) {
    if (level == DETAIL_TINY || d.spinnerCount >= MAX_SPINNERS) {
        return;
    }
    int index = d.spinnerCount;
    if (level == DETAIL_HERO) {
        d.spinners[index] = {hub, axis, rate};
        d.spinnerCount++;
    } else {
        // Built from the hero's: the same spinners in the same order.
        for (int i = 0; i < d.spinnerCount; i++) {
            if (length(d.spinners[i].hub - hub) < 0.01f) {
                index = i;
            }
        }
    }
    Using pen(moving((uint8_t)(MOVE_SPIN + index)));
    float hubR = spinnerR >= 0 ? spinnerR : radius * 0.12f;
    if (hubR > 0) {
        if (fabsf(axis.z) > 0.9f) {
            Using metal(painted(PAINT_AS_BUILT));
            const Ring HUB[] = {{-0.4f * hubR, hubR}, {0.25f * hubR, hubR}, {0.6f * hubR, 0.35f * hubR}};
            tube(out, hub, axis, u, v, HUB, 3, level == DETAIL_HERO ? 8 : 6, GEAR);
        } else {
            Using body(painted(PAINT_ENGINES));
            const Ring SPINNER[] = {{-0.2f * hubR, hubR}, {1.5f * hubR, 0.03f}};
            tube(out, hub, axis, u, v, SPINNER, 2, level == DETAIL_HERO ? 8 : 6, WHITE);
        }
    }
    if (level <= DETAIL_MEDIUM) {
        for (int b = 0; b < blades; b++) {
            float a = 2 * PI * b / blades;
            Vec3 out1 = u * cosf(a) + v * sinf(a), side = cross(axis, out1);
            Vec3 along = normalize(side * cosf(pitch) + axis * sinf(pitch));
            Vec3 root = hub + out1 * (hubR * 0.8f + 0.02f), tip = hub + out1 * radius;
            slab(out, {root + along * (chord / 2), root - along * (chord / 2), chord * 0.25f},
                 {tip + along * (chord * 0.3f), tip - along * (chord * 0.3f), chord * 0.12f}, cross(out1, along),
                 BLADE);
        }
    }
    // The blur, two-sided: seen from either side, its faces aren't culled.
    Pen blurPen = g_pen;
    blurPen.show = SHOW_TURNING;
    blurPen.blur = blur;
    blurPen.move = MOVE_NONE;
    Using blurring(blurPen);
    int sides = level == DETAIL_HERO ? 20 : level == DETAIL_MEDIUM ? 14 : 8;
    for (int k = 0; k < sides; k++) {
        float a0 = 2 * PI * k / sides, a1 = 2 * PI * (k + 1) / sides;
        face(out, hub, hub + (u * cosf(a0) + v * sinf(a0)) * radius, hub + (u * cosf(a1) + v * sinf(a1)) * radius,
             hub - axis, BLADE);
    }
}

// --- gear ----------------------------------------------------------------------

void wheel(Vector<ModelTri> &out, int level, Vec3 hub, float r, float width) {
    int round = level == DETAIL_HERO ? 8 : 6;
    const Ring WHEEL[] = {{-width / 2, r}, {width / 2, r}};
    tube(out, hub, AXIS_Y, AXIS_X, AXIS_Z, WHEEL, 2, round, TYRE);
    disc(out, hub, AXIS_Y, AXIS_X, AXIS_Z, WHEEL[0], round, hub, TYRE);
    disc(out, hub, AXIS_Y, AXIS_X, AXIS_Z, WHEEL[1], round, hub, TYRE);
}

// A leg at (x, y) from `top` down to its wheels: two on an axle, or a bogie
// of four or six - `gap` from the leg either side, `wheelR` round.
void gearLeg(Vector<ModelTri> &out, int level, float x, float y, float top, float wheelR, int wheels, float gap,
             float width) {
    if (level >= DETAIL_SIMPLE) {
        return;
    }
    float leg = std::max(0.08f, wheelR * 0.26f);
    box(out, {x - leg, y - leg, wheelR}, {x + leg, y + leg, top}, GEAR, GEAR);
    int axles = wheels <= 2 ? 1 : wheels / 2;
    if (axles > 1) {
        float reach = (axles - 1) * wheelR * 1.15f;
        box(out, {x - reach, y - leg * 0.7f, wheelR - leg * 0.7f}, {x + reach, y + leg * 0.7f, wheelR + leg * 0.7f},
            GEAR, GEAR);
    }
    for (int a = 0; a < axles; a++) {
        float ax = x + (a - (axles - 1) / 2.0f) * wheelR * 2.3f;
        if (wheels == 1) {
            wheel(out, level, {ax, y, wheelR}, wheelR, width);
            continue;
        }
        for (float side : {-1.0f, 1.0f}) {
            wheel(out, level, {ax, y + side * gap, wheelR}, wheelR, width);
        }
    }
}

// --- lights and sizes --------------------------------------------------------------

// The lights an aeroplane carries: navigation lights and strobes at its
// wingtips, a white light and a strobe at its tail, a beacon over it and one
// under it, and landing lights at the roots of its wings.
void planeLights(Design &d, const Wing &w, Vec3 tail, float top, float bottom, float beaconX) {
    float s = w.tip.y + 0.3f;
    WingStation t = wingAt(w, w.tip.y);
    float size = std::max(0.12f, std::min(0.35f, w.tip.y / 50));
    d.lights.push_back({{t.le - 0.06f, s, t.z + 0.03f}, {255, 40, 40}, size, Lamp::NAV});
    d.lights.push_back({{t.le - 0.06f, -s, t.z + 0.03f}, {40, 255, 90}, size, Lamp::NAV});
    d.lights.push_back({tail + Vec3{-0.2f, 0, 0}, {255, 255, 255}, size, Lamp::NAV});
    d.lights.push_back({{beaconX, 0, top + 0.13f}, {255, 30, 30}, size * 1.15f, Lamp::BEACON_TOP});
    d.lights.push_back({{beaconX + 3 * size / 0.35f, 0, bottom - 0.08f}, {255, 30, 30}, size * 1.15f,
                        Lamp::BEACON_BELLY});
    d.lights.push_back({{t.le - 0.46f, s + 0.1f, t.z - 0.02f}, {255, 255, 255}, size * 1.3f, Lamp::STROBE});
    d.lights.push_back({{t.le - 0.46f, -s - 0.1f, t.z - 0.02f}, {255, 255, 255}, size * 1.3f, Lamp::STROBE});
    d.lights.push_back({tail + Vec3{-0.2f, 0, 0.2f}, {255, 255, 255}, size, Lamp::STROBE});
    WingStation r = wingAt(w, w.root.y);
    float landing = std::max(0.2f, size * 1.4f);
    d.lights.push_back({{r.le - 0.9f * size / 0.35f, w.root.y + 1.3f * size / 0.35f, r.z - 0.3f},
                        {255, 248, 230}, landing, Lamp::LANDING});
    d.lights.push_back({{r.le - 0.9f * size / 0.35f, -w.root.y - 1.3f * size / 0.35f, r.z - 0.3f},
                        {255, 248, 230}, landing, Lamp::LANDING});
}

// A shadow as three flat quads: the body from nose to tail, and each wing.
void blobFor(Design &d, const Wing &w, float nose, float noseW, float noseZ, float tail, float tailW, float tailZ) {
    Vec3 body[4] = {{nose, -noseW, noseZ}, {nose, noseW, noseZ}, {tail, tailW, tailZ}, {tail, -tailW, tailZ}};
    memcpy(d.blob[0], body, sizeof body);
    for (int s = 0; s < 2; s++) {
        float side = s == 0 ? 1.0f : -1.0f;
        WingStation r = wingAt(w, w.root.y), t = wingAt(w, w.tip.y);
        Vec3 q[4] = {{r.le, side * w.root.y, r.z}, {t.le, side * w.tip.y, t.z}, {t.te, side * w.tip.y, t.z},
                     {r.te, side * w.root.y, r.z}};
        memcpy(d.blob[1 + s], q, sizeof q);
    }
}

// --- transports -----------------------------------------------------------------
// Airliners, business jets, turboprops, airlifters and light aircraft: a body,
// a wing low or high, a tail low or T, and engines under the wings, at the
// tail, on the wings with propellers, or a propeller in the nose.

enum Engines : uint8_t { WING_FANS, TAIL_FANS, WING_PROPS, NOSE_PROP };

struct Transport {
    BodyShape shape{};
    // A body of its own, nose to tail, in place of an airliner's shape.
    const Station *stations = nullptr;
    int stationCount = 0;
    int screen = 2;  // the windscreen's station; -1 for none
    int roundness = 16;
    float windows[2] = {0, 0};  // the cabin's, from x to x; none where they are the same
    float windowsUp = 0.21f, windowsTall = 0.22f;
    float upperWindows[2] = {0, 0};  // a second deck's
    float upperUp = 0;
    // The 747's upper deck, a hump of its own, and its windows.
    const Station *hump = nullptr;
    int humpCount = 0, humpScreen = -1;
    float humpWindows[2] = {0, 0};
    float humpUp = 0;
    // The E-3's dish, on two struts over the body.
    float radomeX = 0, radomeZ = 0, radomeR = 0;
    Wing wing{};
    Tip tip = TIP_PLAIN;
    float tipHeight = 0;
    // A high wing's struts - from out along the wing to the body's side.
    float strutY = 0, strutX = 0, strutBodyY = 0, strutBodyZ = 0;
    Engines engines = WING_FANS;
    int engineCount = 1;  // a side
    float engineY[2] = {0, 0};
    // WING_FANS: the nacelle, how far its lip is ahead of the wing's leading
    // edge, of its length, and how far its axis hangs below the wing.
    float nacelleLength = 0, nacelleRadius = 0, engineDrop = 0, engineAhead = 0.81f;
    // TAIL_FANS: the left one's intake lip.
    Vec3 tailFan{};
    // WING_PROPS and NOSE_PROP: the propellers' hubs at x, z - and the
    // nacelles behind them.
    float propX = 0, propZ = 0, propRadius = 0;
    int blades = 0;
    float nacelleHalfWidth = 0, nacelleHalfHeight = 0;
    bool counterRotating = false;  // each wing's pair turning down between them, as the A400M's do
    Section tailRoot{}, tailTip{};  // the tailplane's left half
    bool allMovingTail = false;
    Section finRoot{}, finTip{};
    float gearX = 0, gearY = 0, gearTop = 0, wheelR = 0, wheelGap = 0.45f, wheelWidth = 0.38f;
    int wheels = 2;
    // A second pair of main gear under the body: the 747's and A380's; the
    // A340's in the middle, at y 0.
    float bodyGearX = 0, bodyGearY = 0;
    int bodyWheels = 0;
    float noseX = 0, noseTop = 0, noseWheelR = 0;
    int noseWheels = 2;
    bool fixedGear = false;
    // Fairings either side of the body for the main gear.
    const Station *sponson = nullptr;
    int sponsonCount = 0;
    float sponsonY = 0;
};

// Some thirty faces: a body of four sides with a point at each end, the wings
// and the fin as flat plates painted both sides.
void tinyTransport(Vector<ModelTri> &out, const Transport &t, const Station *st, int n) {
    const Station &nose = st[0], &tail = st[n - 1];
    float length = nose.x - tail.x;
    Station s[2];
    for (int i = 0; i < 2; i++) {
        float x = i == 0 ? nose.x - length * 0.1f : tail.x + length * 0.22f, w, z, h;
        bodyAt(st, n, x, w, z, h);
        s[i] = {x, w, z, h};
    }
    {
        Using pen(painted(PAINT_BODY));
        Vec3 inside = {(s[0].x + s[1].x) / 2, 0, (s[0].z + s[1].z) / 2};
        for (int k = 0; k < 4; k++) {
            faceQuad(out, stationPoint(s[0], k, 4), stationPoint(s[0], k + 1, 4), stationPoint(s[1], k + 1, 4),
                     stationPoint(s[1], k, 4), inside, WHITE);
            face(out, {nose.x, 0, nose.z}, stationPoint(s[0], k, 4), stationPoint(s[0], k + 1, 4), inside, WHITE);
            face(out, {tail.x, 0, tail.z}, stationPoint(s[1], k, 4), stationPoint(s[1], k + 1, 4), inside, WHITE);
        }
    }
    const Wing &w = t.wing;
    for (float side : {1.0f, -1.0f}) {
        WingStation r = wingAt(w, w.root.y), p = wingAt(w, w.tip.y);
        Vec3 a = {r.le, side * w.root.y, r.z}, b = {p.le, side * w.tip.y, p.z}, c = {p.te, side * w.tip.y, p.z},
             d = {r.te, side * w.root.y, r.z};
        Using pen(painted(PAINT_WINGS));
        faceQuad(out, a, b, c, d, {r.le, side * w.tip.y / 2, r.z - 4}, WING);  // seen from above
        faceQuad(out, a, b, c, d, {r.le, side * w.tip.y / 2, r.z + 4}, WING);  // and from below
    }
    Using pen(painted(PAINT_LIVERY));
    Vec3 f0 = t.finRoot.le, f1 = lerp(t.finTip.le, t.finTip.te, 0.5f), f2 = t.finRoot.te;
    face(out, f0, f1, f2, f0 + Vec3{0, 1, 0}, TAIL);
    face(out, f0, f1, f2, f0 + Vec3{0, -1, 0}, TAIL);
}

void buildTransport(Vector<ModelTri> &out, int level, const Transport &t, Design &d) {
    Station shaped[12];
    const Station *st = t.stations;
    int n = t.stationCount;
    if (st == nullptr) {
        n = stationsOf(t.shape, shaped);
        st = shaped;
    }
    if (level == DETAIL_TINY) {
        tinyTransport(out, t, st, n);
        return;
    }
    bool full = level <= DETAIL_MEDIUM;
    int sides = level == DETAIL_HERO ? t.roundness : level == DETAIL_MEDIUM ? std::max(6, (t.roundness * 5 / 8) & ~1) : 6;
    body(out, level, st, n, t.screen, t.roundness);
    if (full && t.windows[0] != t.windows[1]) {
        windows(out, st, n, sides, t.windows[0], t.windows[1], t.windowsUp, t.windowsTall);
    }
    if (full && t.upperWindows[0] != t.upperWindows[1]) {
        windows(out, st, n, sides, t.upperWindows[0], t.upperWindows[1], t.upperUp, t.windowsTall);
    }
    if (t.hump != nullptr) {
        body(out, level, t.hump, t.humpCount, t.humpScreen, t.roundness);
        if (full && t.humpWindows[0] != t.humpWindows[1]) {
            windows(out, t.hump, t.humpCount, sides, t.humpWindows[0], t.humpWindows[1], t.humpUp, t.windowsTall);
        }
    }
    if (t.radomeR > 0) {
        // The dish: a lens on its side, a dark band round its rim; and its
        // two struts, leaning in to it from the body's back.
        float r = t.radomeR, th = r * 0.2f;
        Vec3 c = {t.radomeX, 0, t.radomeZ};
        int round = level == DETAIL_HERO ? 20 : level == DETAIL_MEDIUM ? 14 : 8;
        Using pen(painted(PAINT_BODY));
        const Ring TOP[] = {{th * 0.4f, r}, {th, r * 0.8f}}, BAND[] = {{-th * 0.4f, r}, {th * 0.4f, r}},
                   BOTTOM[] = {{-th, r * 0.8f}, {-th * 0.4f, r}};
        tube(out, c, AXIS_Z, AXIS_X, AXIS_Y, TOP, 2, round, WHITE);
        tube(out, c, AXIS_Z, AXIS_X, AXIS_Y, BOTTOM, 2, round, WHITE);
        disc(out, c, AXIS_Z, AXIS_X, AXIS_Y, TOP[1], round, c, WHITE);
        disc(out, c, AXIS_Z, AXIS_X, AXIS_Y, BOTTOM[0], round, c, WHITE);
        {
            Using band(painted(PAINT_AS_BUILT));
            tube(out, c, AXIS_Z, AXIS_X, AXIS_Y, BAND, 2, round, DARK);
        }
        float w, z, h;
        bodyAt(st, n, t.radomeX, w, z, h);
        for (float s : {1.0f, -1.0f}) {
            Vec3 foot = {t.radomeX, s * w * 0.4f, z + h * 0.9f}, head = {t.radomeX, s * 0.25f, t.radomeZ - th};
            Section a = {foot + Vec3{1.6f, 0, 0}, foot + Vec3{-1.6f, 0, 0}, 0.3f},
                    b = {head + Vec3{0.9f, 0, 0}, head + Vec3{-0.9f, 0, 0}, 0.25f};
            slab(out, a, b, normalize(cross(AXIS_X, head - foot)), WHITE);
        }
    }

    wings(out, level, t.wing, d);
    wingtips(out, level, t.wing, t.tip, t.tipHeight);
    if (t.strutY > 0 && level <= DETAIL_SIMPLE) {
        Using pen(painted(PAINT_WINGS));
        WingStation e = wingAt(t.wing, t.strutY);
        for (float s : {1.0f, -1.0f}) {
            Vec3 top = {t.strutX, s * t.strutY, e.z - e.thick * 0.5f}, foot = {t.strutX - 0.2f, s * t.strutBodyY, t.strutBodyZ};
            Vec3 across = normalize(cross(AXIS_X, top - foot));
            slab(out, {foot + Vec3{0.12f, 0, 0}, foot - Vec3{0.12f, 0, 0}, 0.05f},
                 {top + Vec3{0.14f, 0, 0}, top - Vec3{0.14f, 0, 0}, 0.05f}, across, WING);
        }
    }

    // The engines - each jet's exhaust noted, for its trail at height.
    for (float s : {1.0f, -1.0f}) {
        switch (t.engines) {
        case WING_FANS:
            for (int i = 0; i < t.engineCount; i++) {
                float y = t.engineY[i];
                WingStation e = wingAt(t.wing, y);
                Vec3 front = {e.le + t.engineAhead * t.nacelleLength, s * y, e.z - t.engineDrop};
                fanNacelle(out, level, front, t.nacelleLength, t.nacelleRadius);
                pylonUp(out, level, front, t.nacelleLength, t.nacelleRadius, e.z);
                if (level == DETAIL_HERO && d.exhausts < 4) {
                    d.exhaust[d.exhausts++] = front - AXIS_X * t.nacelleLength;
                }
            }
            break;
        case TAIL_FANS: {
            Vec3 front = {t.tailFan.x, s * t.tailFan.y, t.tailFan.z};
            fanNacelle(out, level, front, t.nacelleLength, t.nacelleRadius);
            if (level <= DETAIL_MEDIUM) {
                // A stub out from the body to it.
                float w, z, h, x = front.x - 0.55f * t.nacelleLength;
                bodyAt(st, n, x, w, z, h);
                Using pen(painted(PAINT_BODY));
                float inner = t.tailFan.y - t.nacelleRadius * 0.5f, k = t.nacelleRadius;
                slab(out, {{x + 0.35f * k * 2, s * w * 0.8f, front.z}, {x - 0.9f * k, s * w * 0.8f, front.z}, 0.35f * k},
                     {{x + 0.2f * k * 2, s * inner, front.z}, {x - 0.8f * k, s * inner, front.z}, 0.3f * k}, AXIS_Z,
                     WHITE);
            }
            if (level == DETAIL_HERO && d.exhausts < 4) {
                d.exhaust[d.exhausts++] = front - AXIS_X * t.nacelleLength;
            }
            break;
        }
        case WING_PROPS:
            for (int i = 0; i < t.engineCount; i++) {
                float y = t.engineY[i];
                Vec3 hub = {t.propX, s * y, t.propZ};
                if (level <= DETAIL_SIMPLE) {
                    propNacelle(out, level, hub - AXIS_X * 0.1f, t.nacelleLength, t.nacelleHalfWidth,
                                t.nacelleHalfHeight);
                }
                // Turning clockwise seen from behind, as most do - or, on an
                // A400M, each wing's pair turning down between them.
                float turn = t.counterRotating ? (i == 0 ? s : -s) : 1.0f;
                propeller(out, level, hub, AXIS_X, AXIS_Z, AXIS_Y, t.propRadius, t.blades, t.propRadius * 0.16f, 0.45f,
                          -5.0f * turn, 3, d, t.propRadius * 0.19f);
            }
            break;
        case NOSE_PROP:
            if (s > 0) {
                propeller(out, level, {t.propX, 0, t.propZ}, AXIS_X, AXIS_Z, AXIS_Y, t.propRadius, t.blades,
                          t.propRadius * 0.13f, 0.35f, -6.0f, 3, d, t.propRadius * 0.17f);
            }
            break;
        }
        if (t.sponson != nullptr) {
            podAt(out, level, t.sponson, t.sponsonCount, s * t.sponsonY, WHITE, PAINT_BELLY);
        }
    }

    tailplane(out, level, t.tailRoot, t.tailTip, t.allMovingTail ? 1.0f : 0.32f, d);
    fin(out, level, t.finRoot, t.finTip, 0.3f, d);

    // The gear: down from the approach to the stand - or always, fixed.
    if (full) {
        Using pen(shown(t.fixedGear ? SHOW_ALWAYS : SHOW_GEAR));
        Using plain(painted(PAINT_AS_BUILT));
        for (float s : {1.0f, -1.0f}) {
            if (t.fixedGear) {
                // A light aircraft's sprung legs, splayed out to its wheels.
                Vec3 hub = {t.gearX, s * t.gearY, t.wheelR};
                Vec3 foot = {t.gearX + 0.05f, s * 0.42f, t.gearTop};
                slab(out, {foot + Vec3{0.08f, 0, 0}, foot - Vec3{0.08f, 0, 0}, 0.04f},
                     {hub + Vec3{0.08f, -s * 0.05f, 0.1f}, hub - Vec3{0.08f, s * 0.05f, -0.1f}, 0.04f},
                     normalize(cross(AXIS_X, hub - foot)), GEAR);
                wheel(out, level, hub, t.wheelR, t.wheelR * 0.55f);
            } else {
                gearLeg(out, level, t.gearX, s * t.gearY, t.gearTop, t.wheelR, t.wheels, t.wheelGap, t.wheelWidth);
            }
            if (t.bodyWheels > 0 && (s > 0 || t.bodyGearY > 0)) {
                gearLeg(out, level, t.bodyGearX, s * t.bodyGearY, t.gearTop, t.wheelR, t.bodyWheels, t.wheelGap,
                        t.wheelWidth);
            }
        }
        gearLeg(out, level, t.noseX, 0, t.noseTop, t.noseWheelR, t.noseWheels, t.noseWheelR * 0.58f,
                t.noseWheelR * 0.62f);
    }

    if (level == DETAIL_HERO) {
        float w, z, h;
        bodyAt(st, n, (t.wing.root.te + t.wing.root.le) / 2, w, z, h);
        const Station &tail = st[n - 1];
        planeLights(d, t.wing, {tail.x, 0, tail.z}, z + h, z - h, t.wing.root.te + 1.4f);
        blobFor(d, t.wing, st[0].x - (st[0].x - tail.x) * 0.01f, w * 0.96f, z, tail.x + 0.1f, tail.w + w * 0.4f,
                tail.z);
        d.gearY = t.gearY;
        d.noseGear = t.noseX;
    }
}

// --- fighters -------------------------------------------------------------------
// A pointed body, a canopy, intakes, a delta or a swept wing, canards or an
// all-moving tailplane, one fin or two canted out, and the nozzles.

struct Fighter {
    const Station *body = nullptr;
    int bodyCount = 0;
    const Station *canopy = nullptr;
    int canopyCount = 0;
    // A chin intake under it, at y 0, or a pair either side at y.
    const Station *intake = nullptr;
    int intakeCount = 0;
    float intakeY = 0;
    Wing wing{};
    bool canards = false;
    Section canardRoot{}, canardTip{};
    bool tailplane = false;
    Section tailRoot{}, tailTip{};
    int fins = 1;
    Section finRoot{}, finTip{};  // with two, the left one's, its tip out where its cant takes it
    int nozzles = 1;
    float nozzleX = 0, nozzleY = 0, nozzleZ = 0, nozzleR = 0;
    float gearX = 0, gearY = 0, gearTop = 0, wheelR = 0, noseX = 0, noseTop = 0, noseWheelR = 0;
};

void buildFighter(Vector<ModelTri> &out, int level, const Fighter &f, Design &d) {
    if (level == DETAIL_TINY) {
        // A dart: four sides, a point at each end, the wing a plate.
        Transport t;
        t.wing = f.wing;
        t.finRoot = f.finRoot;
        t.finTip = f.finTip;
        if (f.fins == 2) {
            t.finRoot.le.y = t.finRoot.te.y = t.finTip.le.y = t.finTip.te.y = 0;
        }
        tinyTransport(out, t, f.body, f.bodyCount);
        return;
    }
    bool full = level <= DETAIL_MEDIUM;
    body(out, level, f.body, f.bodyCount, -1, 12);
    if (f.canopy != nullptr) {
        pod(out, level, f.canopy, f.canopyCount, GLASS, PAINT_AS_BUILT);
    }
    if (f.intake != nullptr) {
        for (float s : {1.0f, -1.0f}) {
            if (f.intakeY == 0 && s < 0) {
                break;
            }
            podAt(out, level, f.intake, f.intakeCount, s * f.intakeY, WHITE, PAINT_BODY);
            if (full) {
                // Its mouth, dark.
                const Station &m = f.intake[0];
                Using pen(painted(PAINT_AS_BUILT));
                int sides = level == DETAIL_HERO ? 10 : 8;
                Vec3 c = {m.x + 0.01f, s * f.intakeY, m.z};
                for (int k = 0; k < sides; k++) {
                    float a0 = (k + 0.5f) * 2 * PI / sides, a1 = (k + 1.5f) * 2 * PI / sides;
                    face(out, c, c + Vec3{0, m.w * sinf(a0), m.h * cosf(a0)}, c + Vec3{0, m.w * sinf(a1), m.h * cosf(a1)},
                         c - AXIS_X, DARK);
                }
            }
        }
    }
    wings(out, level, f.wing, d);
    if (f.canards) {
        Using pen(painted(PAINT_WINGS));
        for (float s : {1.0f, -1.0f}) {
            Section r = f.canardRoot, t = f.canardTip;
            r.le.y *= s, r.te.y *= s, t.le.y *= s, t.te.y *= s;
            if (level <= DETAIL_MEDIUM) {
                Using pen(moving(MOVE_CANARD));
                slab(out, r, t, AXIS_Z, WING);
            } else {
                slab(out, r, t, AXIS_Z, WING);
            }
        }
        Vec3 pivot = lerp(f.canardRoot.le, f.canardRoot.te, 0.45f);
        d.hinge[MOVE_CANARD] = {{pivot.x, 0, pivot.z}, AXIS_Y, -0.25f};
    }
    if (f.tailplane) {
        tailplane(out, level, f.tailRoot, f.tailTip, 1.0f, d);
    }
    for (int i = 0; i < f.fins; i++) {
        float s = i == 0 ? 1.0f : -1.0f;
        Section r = f.finRoot, t = f.finTip;
        r.le.y *= s, r.te.y *= s, t.le.y *= s, t.te.y *= s;
        Vec3 up = normalize(lerp(t.le, t.te, 0.5f) - lerp(r.le, r.te, 0.5f));
        fin(out, level, r, t, 0.28f, d, i == 0 ? MOVE_RUDDER : MOVE_RUDDER_2, normalize(cross(AXIS_X, up)));
    }
    Using plain(painted(PAINT_AS_BUILT));
    for (int i = 0; i < f.nozzles; i++) {
        float y = f.nozzles == 1 ? 0 : (i == 0 ? f.nozzleY : -f.nozzleY);
        Vec3 c = {f.nozzleX, y, f.nozzleZ};
        int round = level == DETAIL_HERO ? 12 : level == DETAIL_MEDIUM ? 8 : 6;
        const Ring NOZZLE_RINGS[] = {{0.4f, f.nozzleR * 1.05f}, {0, f.nozzleR}, {-0.15f, f.nozzleR * 0.94f}};
        tube(out, c, AXIS_X, AXIS_Z, AXIS_Y, NOZZLE_RINGS, 3, round, NOZZLE);
        disc(out, c, AXIS_X, AXIS_Z, AXIS_Y, NOZZLE_RINGS[2], round, c + AXIS_X, EXHAUST);
        if (level == DETAIL_HERO && d.exhausts < 4) {
            d.exhaust[d.exhausts++] = c - AXIS_X * 0.15f;
        }
    }
    if (full) {
        Using pen(shown(SHOW_GEAR));
        for (float s : {1.0f, -1.0f}) {
            gearLeg(out, level, f.gearX, s * f.gearY, f.gearTop, f.wheelR, 1, 0, f.wheelR * 0.6f);
        }
        gearLeg(out, level, f.noseX, 0, f.noseTop, f.noseWheelR, 1, 0, f.noseWheelR * 0.6f);
    }
    if (level == DETAIL_HERO) {
        const Station &tail = f.body[f.bodyCount - 1];
        float w, z, h;
        bodyAt(f.body, f.bodyCount, 0, w, z, h);
        planeLights(d, f.wing, {tail.x, 0, tail.z}, z + h, z - h, f.wing.root.te + 1.0f);
        blobFor(d, f.wing, f.body[0].x, w * 0.5f, z, tail.x, tail.w, tail.z);
        d.gearY = f.gearY;
        d.noseGear = f.noseX;
    }
}

// --- rotorcraft -----------------------------------------------------------------
// A cabin and its tail boom, the engines and gearbox over it, the rotor - or
// two, fore and aft - a fin and a tail rotor, and wheels or skids.

struct Rotorcraft {
    const Station *body = nullptr;
    int bodyCount = 0, screen = 2;
    const Station *fairing = nullptr;  // over the cabin
    int fairingCount = 0;
    float windows[2] = {0, 0};
    float windowsUp = 0, windowsTall = 0;
    Vec3 rotor{};
    float rotorR = 0;
    int blades = 0;
    bool tandem = false;
    Vec3 rearRotor{};
    const Station *pylon = nullptr;  // the Chinook's, up to its rear rotor
    int pylonCount = 0;
    const Station *engine = nullptr;  // and its engines, either side of it
    int engineCount = 0;
    float engineY = 0;
    Vec3 tailRotor{};
    float tailRotorR = 0;
    int tailBlades = 0;
    Section finRoot{}, finTip{}, stabRoot{}, stabTip{};
    const Station *sponson = nullptr;
    int sponsonCount = 0;
    float sponsonY = 0;
    bool skids = false;
    float skidY = 0, skidFrom = 0, skidTo = 0;
    // Wheels: the mains at (gearX, ±gearY), the nose's - or, on a Chinook, a
    // second pair - at noseX.
    float gearX = 0, gearY = 0, gearTop = 0, wheelR = 0, noseX = 0, noseY = 0, noseTop = 0, noseWheelR = 0;
    bool fixedGear = false;
};

void buildRotorcraft(Vector<ModelTri> &out, int level, const Rotorcraft &r, Design &d) {
    if (level == DETAIL_TINY) {
        // The body as four sides and the rotor's disc, see-through.
        Transport t;
        t.wing.root = {0.1f, 0.2f, -0.2f, r.body[0].z, 0.05f};
        t.wing.kink = t.wing.root;
        t.wing.tip = {0.3f, 0.2f, -0.2f, r.body[0].z, 0.05f};
        t.finRoot = r.finRoot;
        t.finTip = r.finTip;
        tinyTransport(out, t, r.body, r.bodyCount);
        Pen blur = g_pen;
        blur.blur = 2;
        Using pen(blur);
        for (int k = 0; k < 6; k++) {
            float a0 = 2 * PI * k / 6, a1 = 2 * PI * (k + 1) / 6;
            face(out, r.rotor, r.rotor + Vec3{cosf(a0), sinf(a0), 0} * r.rotorR,
                 r.rotor + Vec3{cosf(a1), sinf(a1), 0} * r.rotorR, r.rotor - AXIS_Z, BLADE);
        }
        return;
    }
    bool full = level <= DETAIL_MEDIUM;
    body(out, level, r.body, r.bodyCount, r.screen, 12);
    int sides = level == DETAIL_HERO ? 12 : level == DETAIL_MEDIUM ? 8 : 6;
    if (full && r.windows[0] != r.windows[1]) {
        windows(out, r.body, r.bodyCount, sides, r.windows[0], r.windows[1], r.windowsUp, r.windowsTall);
    }
    if (r.fairing != nullptr) {
        pod(out, level, r.fairing, r.fairingCount, WHITE, PAINT_BODY);
    }
    if (r.pylon != nullptr) {
        pod(out, level, r.pylon, r.pylonCount, WHITE, PAINT_BODY);
    }
    if (r.engine != nullptr) {
        for (float s : {1.0f, -1.0f}) {
            podAt(out, level, r.engine, r.engineCount, s * r.engineY, WHITE, PAINT_BODY);
        }
    }
    if (r.sponson != nullptr) {
        for (float s : {1.0f, -1.0f}) {
            podAt(out, level, r.sponson, r.sponsonCount, s * r.sponsonY, WHITE, PAINT_BODY);
        }
    }
    // The rotors: the mast up to each hub, and the blades. A Chinook's turn
    // opposite ways.
    {
        Using pen(painted(PAINT_AS_BUILT));
        float mast = r.rotorR * 0.025f;
        for (int i = 0; i < (r.tandem ? 2 : 1); i++) {
            Vec3 hub = i == 0 ? r.rotor : r.rearRotor;
            if (full) {
                box(out, hub + Vec3{-mast, -mast, -1.0f}, hub + Vec3{mast, mast, 0.1f}, GEAR, GEAR);
            }
            propeller(out, level, hub, AXIS_Z, AXIS_X, AXIS_Y, r.rotorR, r.blades, r.rotorR * 0.075f, 0.12f,
                      i == 0 ? 2.4f : -2.4f, 2, d, r.rotorR * 0.045f);
        }
        if (r.tailRotorR > 0) {
            propeller(out, level, r.tailRotor, AXIS_Y, AXIS_X, AXIS_Z, r.tailRotorR, r.tailBlades,
                      r.tailRotorR * 0.2f, 0.3f, 7.0f, 3, d, r.tailRotorR * 0.08f);
        }
    }
    fin(out, level, r.finRoot, r.finTip, 0, d, MOVE_NONE);
    if (r.stabTip.le.y > 0) {
        Using pen(painted(PAINT_WINGS));
        for (float s : {1.0f, -1.0f}) {
            Section a = r.stabRoot, b = r.stabTip;
            a.le.y *= s, a.te.y *= s, b.le.y *= s, b.te.y *= s;
            slab(out, a, b, AXIS_Z, WING);
        }
    }
    if (full) {
        Using pen(shown(r.fixedGear ? SHOW_ALWAYS : SHOW_GEAR));
        Using plain(painted(PAINT_AS_BUILT));
        if (r.skids) {
            Using always(shown(SHOW_ALWAYS));
            for (float s : {1.0f, -1.0f}) {
                const Ring SKID[] = {{0, 0.05f}, {r.skidFrom - r.skidTo, 0.05f}};
                tube(out, {r.skidTo, s * r.skidY, 0.06f}, AXIS_X, AXIS_Z, AXIS_Y, SKID, 2, 4, GEAR);
                for (float x : {r.skidFrom - 0.9f, r.skidTo + 0.6f}) {
                    Vec3 foot = {x, s * r.skidY, 0.06f}, top = {x, s * 0.45f, r.body[3].z - r.body[3].h * 0.8f};
                    slab(out, {foot + Vec3{0.04f, 0, 0}, foot - Vec3{0.04f, 0, 0}, 0.06f},
                         {top + Vec3{0.04f, 0, 0}, top - Vec3{0.04f, 0, 0}, 0.06f}, normalize(cross(AXIS_X, top - foot)),
                         GEAR);
                }
            }
        } else {
            for (float s : {1.0f, -1.0f}) {
                gearLeg(out, level, r.gearX, s * r.gearY, r.gearTop, r.wheelR, 1, 0, r.wheelR * 0.6f);
                if (r.noseY > 0) {
                    gearLeg(out, level, r.noseX, s * r.noseY, r.noseTop, r.noseWheelR, 1, 0, r.noseWheelR * 0.6f);
                }
            }
            if (r.noseY == 0) {
                gearLeg(out, level, r.noseX, 0, r.noseTop, r.noseWheelR, 1, 0, r.noseWheelR * 0.6f);
            }
        }
    }
    if (level == DETAIL_HERO) {
        // Its lights: red and green either side of the cabin, white at the
        // tail, a beacon over the boom and under the belly, strobes on the
        // fin, and a landing light under the nose.
        const Station &nose = r.body[2], &tail = r.body[r.bodyCount - 1], &cabin = r.body[r.bodyCount / 2 - 1];
        float size = 0.18f;
        d.lights.push_back({{cabin.x, cabin.w + 0.05f, cabin.z + cabin.h * 0.4f}, {255, 40, 40}, size, Lamp::NAV});
        d.lights.push_back({{cabin.x, -cabin.w - 0.05f, cabin.z + cabin.h * 0.4f}, {40, 255, 90}, size, Lamp::NAV});
        d.lights.push_back({{tail.x - 0.1f, 0, tail.z}, {255, 255, 255}, size, Lamp::NAV});
        d.lights.push_back({{cabin.x - 1.5f, 0, cabin.z + cabin.h + 0.6f}, {255, 30, 30}, size, Lamp::BEACON_TOP});
        d.lights.push_back({{cabin.x, 0, cabin.z - cabin.h - 0.05f}, {255, 30, 30}, size, Lamp::BEACON_BELLY});
        d.lights.push_back({r.finTip.te + Vec3{0, 0, 0.05f}, {255, 255, 255}, size * 1.3f, Lamp::STROBE});
        d.lights.push_back({{nose.x, 0.3f, nose.z - nose.h}, {255, 248, 230}, 0.25f, Lamp::LANDING});
        d.lights.push_back({{nose.x, -0.3f, nose.z - nose.h}, {255, 248, 230}, 0.25f, Lamp::LANDING});
        float w = cabin.w;
        Vec3 bodyBlob[4] = {{r.body[0].x, -w * 0.6f, cabin.z}, {r.body[0].x, w * 0.6f, cabin.z},
                            {tail.x, tail.w, tail.z}, {tail.x, -tail.w, tail.z}};
        memcpy(d.blob[0], bodyBlob, sizeof bodyBlob);
        // The rotor's shadow, a square of its disc's middle.
        float k = r.rotorR * 0.7f;
        Vec3 disc1[4] = {r.rotor + Vec3{k, -k, 0}, r.rotor + Vec3{k, k, 0}, r.rotor + Vec3{-k, k, 0},
                         r.rotor + Vec3{-k, -k, 0}};
        memcpy(d.blob[1], disc1, sizeof disc1);
        Vec3 rear = r.tandem ? r.rearRotor : r.rotor;
        Vec3 disc2[4] = {rear + Vec3{k, -k, 0}, rear + Vec3{k, k, 0}, rear + Vec3{-k, k, 0}, rear + Vec3{-k, -k, 0}};
        memcpy(d.blob[2], disc2, sizeof disc2);
        d.gearY = r.skids ? r.skidY : r.gearY;
        d.noseGear = r.skids ? 0 : r.noseX;
    }
}

// --- the families ------------------------------------------------------------------
// Each one's numbers from its type's: length, span and height as published,
// the rest as near as drawings of it show.

Transport narrowbody() {  // the A320
    Transport t;
    t.shape = {16.75f, -20.9f, 1.97f, 2.07f, 3.6f, 3.85f, 11.4f, 1.15f};
    t.windows[0] = 11.4f, t.windows[1] = -8.6f;
    t.wing.root = {1.9f, 3.6f, -3.4f, 2.30f, 0.75f};
    t.wing.kink = {6.3f, 1.55f, -3.4f, 2.68f, 0.50f};
    t.wing.tip = {17.0f, -3.44f, -5.2f, 3.62f, 0.20f};
    t.wing.flapsTo = 0.70f;
    t.wing.spoilers = true;
    t.tip = TIP_SHARKLET, t.tipHeight = 2.38f;
    t.engineY[0] = 5.75f;
    t.nacelleLength = 5.3f, t.nacelleRadius = 1.10f, t.engineDrop = 1.01f;
    t.tailRoot = {{-15.2f, 1.2f, 4.25f}, {-19.0f, 1.2f, 4.25f}, 0.35f};
    t.tailTip = {{-18.6f, 6.2f, 4.60f}, {-20.2f, 6.2f, 4.60f}, 0.12f};
    t.finRoot = {{-12.8f, 0, 5.2f}, {-19.2f, 0, 5.2f}, 0.45f};
    t.finTip = {{-18.4f, 0, 11.5f}, {-20.6f, 0, 11.5f}, 0.15f};
    t.gearY = 3.8f, t.gearTop = 2.3f, t.wheelR = 0.58f;
    t.noseX = 12.4f, t.noseTop = 2.0f, t.noseWheelR = 0.38f;
    return t;
}

Transport narrowbody4() {  // the KC-135R, a 707
    Transport t;
    t.shape = {17.0f, -24.5f, 1.83f, 1.98f, 3.75f, 4.2f, 12.5f, 1.4f};
    t.windows[0] = 13.0f, t.windows[1] = -12.0f;
    t.wing.root = {1.85f, 5.0f, -4.6f, 2.55f, 0.9f};
    t.wing.kink = {6.0f, 2.09f, -4.6f, 2.85f, 0.6f};
    t.wing.tip = {19.94f, -7.67f, -10.07f, 4.77f, 0.22f};
    t.wing.flapsTo = 0.62f;
    t.wing.spoilers = true;
    t.engineCount = 2, t.engineY[0] = 7.2f, t.engineY[1] = 12.6f;
    t.nacelleLength = 4.6f, t.nacelleRadius = 1.0f, t.engineDrop = 1.55f, t.engineAhead = 0.95f;
    t.tailRoot = {{-19.0f, 1.0f, 4.9f}, {-23.3f, 1.0f, 4.9f}, 0.35f};
    t.tailTip = {{-22.6f, 6.9f, 5.6f}, {-24.6f, 6.9f, 5.6f}, 0.12f};
    t.finRoot = {{-13.5f, 0, 5.5f}, {-23.5f, 0, 5.1f}, 0.5f};
    t.finTip = {{-22.8f, 0, 12.7f}, {-25.2f, 0, 12.7f}, 0.15f};
    t.gearY = 3.3f, t.gearTop = 2.4f, t.wheelR = 0.5f, t.wheels = 4;
    t.noseX = 14.5f, t.noseTop = 2.2f, t.noseWheelR = 0.4f;
    return t;
}

Transport awacs() {  // the E-3 Sentry: a 707 under its radar's dish
    Transport t = narrowbody4();
    t.windows[0] = t.windows[1] = 0;
    t.radomeX = -9.0f, t.radomeZ = 9.5f, t.radomeR = 4.08f;
    return t;
}

Transport widebody() {  // the 787-9
    Transport t;
    t.shape = {29.5f, -33.3f, 2.885f, 2.985f, 5.1f, 5.6f, 15.5f, 1.9f};
    t.windows[0] = 25.5f, t.windows[1] = -22.0f;
    t.windowsUp = 0.4f, t.windowsTall = 0.34f;
    t.wing.root = {2.9f, 8.0f, -5.5f, 3.9f, 1.3f};
    t.wing.kink = {10.5f, 3.25f, -5.5f, 4.7f, 0.85f};
    t.wing.tip = {30.05f, -8.97f, -11.47f, 6.75f, 0.25f};
    t.wing.flapsTo = 0.65f;
    t.wing.spoilers = true;
    t.wing.aileronFrom = 0.74f;
    t.engineY[0] = 9.75f;
    t.nacelleLength = 6.6f, t.nacelleRadius = 1.75f, t.engineDrop = 2.2f;
    t.tailRoot = {{-24.3f, 2.1f, 6.6f}, {-30.8f, 2.1f, 6.6f}, 0.55f};
    t.tailTip = {{-30.3f, 9.8f, 7.5f}, {-32.6f, 9.8f, 7.5f}, 0.18f};
    t.finRoot = {{-21.5f, 0, 8.0f}, {-32.0f, 0, 7.4f}, 0.7f};
    t.finTip = {{-31.0f, 0, 17.0f}, {-34.6f, 0, 17.0f}, 0.2f};
    t.gearY = 4.9f, t.gearTop = 3.6f, t.wheelR = 0.66f, t.wheels = 4, t.wheelGap = 0.55f, t.wheelWidth = 0.45f;
    t.noseX = 25.6f, t.noseTop = 3.0f, t.noseWheelR = 0.5f;
    return t;
}

Transport widebody4() {  // the A340-300
    Transport t;
    t.shape = {29.0f, -34.7f, 2.82f, 2.96f, 4.95f, 5.2f, 15.0f, 1.7f};
    t.windows[0] = 25.5f, t.windows[1] = -21.5f;
    t.windowsUp = 0.4f, t.windowsTall = 0.3f;
    t.wing.root = {2.85f, 7.0f, -4.8f, 3.75f, 1.2f};
    t.wing.kink = {9.0f, 3.3f, -4.8f, 4.3f, 0.8f};
    t.wing.tip = {29.0f, -8.7f, -10.7f, 6.3f, 0.22f};
    t.wing.flapsTo = 0.62f;
    t.wing.spoilers = true;
    t.tip = TIP_SHARKLET, t.tipHeight = 2.7f;
    t.engineCount = 2, t.engineY[0] = 9.5f, t.engineY[1] = 17.2f;
    t.nacelleLength = 4.8f, t.nacelleRadius = 1.15f, t.engineDrop = 1.55f;
    t.tailRoot = {{-24.6f, 2.0f, 6.4f}, {-31.1f, 2.0f, 6.4f}, 0.5f};
    t.tailTip = {{-30.6f, 9.6f, 7.2f}, {-33.0f, 9.6f, 7.2f}, 0.18f};
    t.finRoot = {{-21.6f, 0, 7.8f}, {-32.1f, 0, 7.2f}, 0.7f};
    t.finTip = {{-30.6f, 0, 16.85f}, {-34.4f, 0, 16.85f}, 0.2f};
    t.gearY = 5.3f, t.gearTop = 3.4f, t.wheelR = 0.63f, t.wheels = 4, t.wheelGap = 0.55f, t.wheelWidth = 0.42f;
    t.bodyGearX = -1.5f, t.bodyGearY = 0, t.bodyWheels = 2;
    t.noseX = 25.4f, t.noseTop = 2.8f, t.noseWheelR = 0.48f;
    return t;
}

// The 747's upper deck: rising out of the nose behind the radome, as wide as
// its cabin, and faired back into the roof over the wing.
const Station HUMP_747[] = {{30.0f, 0.3f, 6.6f, 0.3f},    {29.0f, 1.1f, 7.35f, 1.0f}, {27.5f, 1.7f, 8.0f, 1.6f},
                            {25.5f, 1.95f, 8.55f, 1.85f}, {9.0f, 1.95f, 8.6f, 1.85f},  {4.5f, 1.55f, 8.65f, 1.45f},
                            {1.5f, 0.6f, 8.8f, 0.4f}};

Transport jumbo() {  // the 747-400
    Transport t;
    t.shape = {31.5f, -39.1f, 3.25f, 3.45f, 5.5f, 6.0f, 17.0f, 1.7f};
    t.screen = -1;  // its flight deck is up on the hump
    t.windows[0] = 27.0f, t.windows[1] = -28.0f;
    t.windowsUp = 0.4f, t.windowsTall = 0.32f;
    t.hump = HUMP_747, t.humpCount = 7, t.humpScreen = 1;
    t.humpWindows[0] = 26.5f, t.humpWindows[1] = 8.5f;
    t.humpUp = 0.2f;
    t.wing.root = {3.25f, 11.0f, -6.0f, 4.3f, 1.5f};
    t.wing.kink = {11.5f, 4.7f, -6.0f, 4.95f, 0.9f};
    t.wing.tip = {31.0f, -10.25f, -14.1f, 7.75f, 0.25f};
    t.wing.flapsTo = 0.62f;
    t.wing.spoilers = true;
    t.tip = TIP_SHARKLET, t.tipHeight = 1.8f;
    t.engineCount = 2, t.engineY[0] = 12.0f, t.engineY[1] = 21.0f;
    t.nacelleLength = 6.6f, t.nacelleRadius = 1.42f, t.engineDrop = 2.0f;
    t.tailRoot = {{-28.0f, 2.3f, 7.6f}, {-35.0f, 2.3f, 7.6f}, 0.6f};
    t.tailTip = {{-36.2f, 11.0f, 8.7f}, {-38.4f, 11.0f, 8.7f}, 0.2f};
    t.finRoot = {{-24.5f, 0, 9.0f}, {-36.0f, 0, 8.2f}, 0.8f};
    t.finTip = {{-36.0f, 0, 19.4f}, {-39.4f, 0, 19.4f}, 0.25f};
    t.gearY = 5.6f, t.gearTop = 3.9f, t.wheelR = 0.62f, t.wheels = 4, t.wheelGap = 0.55f, t.wheelWidth = 0.45f;
    t.bodyGearX = -3.0f, t.bodyGearY = 1.9f, t.bodyWheels = 4;
    t.noseX = 25.6f, t.noseTop = 3.2f, t.noseWheelR = 0.5f;
    return t;
}

Transport superjumbo() {  // the A380
    Transport t;
    t.shape = {33.4f, -39.3f, 3.57f, 4.2f, 6.55f, 6.8f, 19.0f, 2.6f};
    t.windows[0] = 29.0f, t.windows[1] = -26.0f;
    t.windowsUp = -1.15f, t.windowsTall = 0.34f;
    t.upperWindows[0] = 27.0f, t.upperWindows[1] = -21.0f;
    t.upperUp = 1.75f;
    t.wing.root = {3.6f, 14.0f, -3.7f, 4.1f, 1.8f};
    t.wing.kink = {12.5f, 8.1f, -4.6f, 4.85f, 1.1f};
    t.wing.tip = {39.0f, -9.4f, -13.4f, 7.5f, 0.3f};
    t.wing.flapsTo = 0.6f;
    t.wing.spoilers = true;
    t.tip = TIP_FENCE, t.tipHeight = 1.5f;
    t.engineCount = 2, t.engineY[0] = 15.5f, t.engineY[1] = 25.7f;
    t.nacelleLength = 7.0f, t.nacelleRadius = 1.75f, t.engineDrop = 2.3f;
    t.tailRoot = {{-28.6f, 3.0f, 8.6f}, {-35.6f, 3.0f, 8.6f}, 0.7f};
    t.tailTip = {{-36.6f, 15.0f, 9.8f}, {-39.0f, 15.0f, 9.8f}, 0.25f};
    t.finRoot = {{-26.0f, 0, 10.6f}, {-37.4f, 0, 9.6f}, 0.9f};
    t.finTip = {{-36.6f, 0, 24.1f}, {-40.0f, 0, 24.1f}, 0.3f};
    t.gearY = 6.1f, t.gearTop = 4.3f, t.wheelR = 0.68f, t.wheels = 4, t.wheelGap = 0.6f, t.wheelWidth = 0.48f;
    t.bodyGearX = -3.6f, t.bodyGearY = 2.0f, t.bodyWheels = 6;
    t.noseX = 30.0f, t.noseTop = 3.5f, t.noseWheelR = 0.55f;
    return t;
}

Transport regional() {  // the CRJ900
    Transport t;
    t.shape = {19.3f, -16.9f, 1.345f, 1.42f, 2.55f, 3.0f, 8.0f, 0.95f};
    t.roundness = 14;
    t.windows[0] = 16.0f, t.windows[1] = -6.5f;
    t.windowsUp = 0.18f, t.windowsTall = 0.2f;
    t.wing.root = {1.3f, 2.6f, -2.2f, 1.55f, 0.55f};
    t.wing.kink = {4.0f, 1.24f, -2.2f, 1.74f, 0.4f};
    t.wing.tip = {11.6f, -2.6f, -3.85f, 2.27f, 0.14f};
    t.wing.flapsTo = 0.66f;
    t.wing.spoilers = true;
    t.tip = TIP_SHARKLET, t.tipHeight = 1.3f;
    t.engines = TAIL_FANS;
    t.tailFan = {-8.4f, 2.3f, 3.15f};
    t.nacelleLength = 3.6f, t.nacelleRadius = 0.74f;
    t.tailRoot = {{-14.3f, 0.25f, 7.35f}, {-16.7f, 0.25f, 7.35f}, 0.3f};
    t.tailTip = {{-16.3f, 4.25f, 7.45f}, {-17.6f, 4.25f, 7.45f}, 0.12f};
    t.finRoot = {{-10.5f, 0, 3.75f}, {-15.8f, 0, 3.5f}, 0.45f};
    t.finTip = {{-14.1f, 0, 7.4f}, {-17.3f, 0, 7.4f}, 0.32f};
    t.gearY = 2.0f, t.gearTop = 1.4f, t.wheelR = 0.47f, t.wheelGap = 0.3f, t.wheelWidth = 0.28f;
    t.noseX = 17.3f, t.noseTop = 1.3f, t.noseWheelR = 0.33f;
    return t;
}

Transport bizjet() {  // the Gulfstream G650
    Transport t;
    t.shape = {16.4f, -14.0f, 1.3f, 1.37f, 2.35f, 3.3f, 7.2f, 0.95f};
    t.roundness = 14;
    t.windows[0] = 12.2f, t.windows[1] = 0.8f;
    t.windowsUp = 0.12f, t.windowsTall = 0.34f;
    t.wing.root = {1.3f, 3.4f, -2.1f, 1.25f, 0.6f};
    t.wing.kink = {4.2f, 1.3f, -2.1f, 1.45f, 0.4f};
    t.wing.tip = {14.0f, -5.8f, -7.1f, 2.15f, 0.12f};
    t.wing.flapsTo = 0.6f;
    t.tip = TIP_SHARKLET, t.tipHeight = 2.0f;
    t.engines = TAIL_FANS;
    t.tailFan = {-6.1f, 2.15f, 3.15f};
    t.nacelleLength = 4.4f, t.nacelleRadius = 0.82f;
    t.tailRoot = {{-12.0f, 0.25f, 7.65f}, {-14.2f, 0.25f, 7.65f}, 0.26f};
    t.tailTip = {{-14.6f, 5.0f, 7.65f}, {-15.6f, 5.0f, 7.65f}, 0.1f};
    t.finRoot = {{-7.6f, 0, 3.6f}, {-12.6f, 0, 3.4f}, 0.4f};
    t.finTip = {{-12.1f, 0, 7.82f}, {-14.6f, 0, 7.82f}, 0.28f};
    t.gearY = 2.1f, t.gearTop = 1.2f, t.wheelR = 0.42f, t.wheelGap = 0.28f, t.wheelWidth = 0.25f;
    t.noseX = 13.7f, t.noseTop = 1.2f, t.noseWheelR = 0.3f;
    return t;
}

// The fairings either side over a high-winged aircraft's main gear.
const Station ATR_SPONSON[] = {{1.6f, 0.15f, 0.95f, 0.2f}, {1.0f, 0.45f, 0.95f, 0.5f}, {-1.8f, 0.45f, 0.98f, 0.5f},
                               {-2.8f, 0.15f, 1.05f, 0.2f}};

Transport turboprop() {  // the ATR 72-600
    Transport t;
    t.shape = {13.0f, -14.2f, 1.285f, 1.43f, 2.35f, 2.7f, 8.5f, 1.25f};
    t.roundness = 14;
    t.windows[0] = 9.5f, t.windows[1] = -5.0f;
    t.windowsUp = 0.25f, t.windowsTall = 0.25f;
    t.wing.root = {0.0f, 1.3f, -1.27f, 4.0f, 0.45f};
    t.wing.kink = {6.0f, 1.3f, -1.27f, 4.0f, 0.4f};
    t.wing.tip = {13.5f, 0.95f, -0.45f, 4.33f, 0.15f};
    t.wing.flapsTo = 0.5f;
    t.wing.aileronFrom = 0.7f, t.wing.aileronTo = 0.95f, t.wing.aileronChord = 0.3f;
    t.engines = WING_PROPS;
    t.engineY[0] = 4.05f;
    t.propX = 3.4f, t.propZ = 3.35f, t.propRadius = 1.965f, t.blades = 6;
    t.nacelleLength = 6.3f, t.nacelleHalfWidth = 0.5f, t.nacelleHalfHeight = 0.72f;
    t.tailRoot = {{-11.0f, 0.15f, 7.4f}, {-13.2f, 0.15f, 7.4f}, 0.25f};
    t.tailTip = {{-11.6f, 3.7f, 7.42f}, {-13.0f, 3.7f, 7.42f}, 0.12f};
    t.finRoot = {{-8.4f, 0, 3.6f}, {-12.6f, 0, 3.5f}, 0.35f};
    t.finTip = {{-11.3f, 0, 7.45f}, {-13.6f, 0, 7.45f}, 0.26f};
    t.gearY = 1.4f, t.gearTop = 0.9f, t.wheelR = 0.43f, t.wheelGap = 0.22f, t.wheelWidth = 0.24f;
    t.noseX = 10.8f, t.noseTop = 1.3f, t.noseWheelR = 0.3f;
    t.sponson = ATR_SPONSON, t.sponsonCount = 4, t.sponsonY = 1.15f;
    return t;
}

Transport turbopropLow() {  // the Saab 340
    Transport t;
    t.shape = {9.2f, -10.5f, 1.155f, 1.2f, 2.35f, 2.4f, 6.0f, 0.8f};
    t.roundness = 14;
    t.windows[0] = 6.5f, t.windows[1] = -3.5f;
    t.windowsUp = 0.2f, t.windowsTall = 0.24f;
    t.wing.root = {1.0f, 1.45f, -1.25f, 1.6f, 0.45f};
    t.wing.kink = {3.7f, 1.45f, -1.25f, 1.73f, 0.38f};
    t.wing.tip = {10.7f, 0.85f, -0.2f, 2.45f, 0.13f};
    t.wing.flapsTo = 0.55f;
    t.engines = WING_PROPS;
    t.engineY[0] = 3.7f;
    t.propX = 3.5f, t.propZ = 2.4f, t.propRadius = 1.675f, t.blades = 4;
    t.nacelleLength = 5.4f, t.nacelleHalfWidth = 0.45f, t.nacelleHalfHeight = 0.62f;
    t.tailRoot = {{-7.4f, 0.55f, 2.95f}, {-9.4f, 0.55f, 2.95f}, 0.2f};
    t.tailTip = {{-8.5f, 4.5f, 3.6f}, {-9.6f, 4.5f, 3.6f}, 0.1f};
    t.finRoot = {{-5.6f, 0, 3.35f}, {-9.9f, 0, 3.15f}, 0.3f};
    t.finTip = {{-9.0f, 0, 6.97f}, {-10.5f, 0, 6.97f}, 0.2f};
    t.gearY = 3.7f, t.gearTop = 1.65f, t.wheelR = 0.38f, t.wheelGap = 0.24f, t.wheelWidth = 0.22f;
    t.noseX = 7.1f, t.noseTop = 1.3f, t.noseWheelR = 0.3f;
    return t;
}

const Station C172[] = {{2.80f, 0.30f, 1.33f, 0.30f},  {2.30f, 0.50f, 1.30f, 0.46f},  {1.50f, 0.56f, 1.36f, 0.58f},
                        {0.90f, 0.57f, 1.42f, 0.64f},  {-0.40f, 0.56f, 1.44f, 0.62f}, {-1.20f, 0.46f, 1.48f, 0.52f},
                        {-3.00f, 0.24f, 1.58f, 0.30f}, {-4.80f, 0.10f, 1.68f, 0.17f}, {-5.25f, 0.05f, 1.72f, 0.08f}};

Transport lightHigh() {  // the Cessna 172
    Transport t;
    t.stations = C172, t.stationCount = 9, t.screen = 2, t.roundness = 10;
    t.windows[0] = 0.75f, t.windows[1] = -1.05f;
    t.windowsUp = 0.3f, t.windowsTall = 0.34f;
    t.wing.root = {0.0f, 1.2f, -0.43f, 2.08f, 0.22f};
    t.wing.kink = {2.6f, 1.2f, -0.43f, 2.1f, 0.2f};
    t.wing.tip = {5.5f, 1.02f, -0.12f, 2.2f, 0.12f};
    t.wing.flapsTo = 0.45f;
    t.wing.aileronFrom = 0.5f, t.wing.aileronTo = 0.95f, t.wing.aileronChord = 0.25f;
    t.strutY = 2.6f, t.strutX = 0.62f, t.strutBodyY = 0.52f, t.strutBodyZ = 0.92f;
    t.engines = NOSE_PROP;
    t.propX = 2.95f, t.propZ = 1.33f, t.propRadius = 0.95f, t.blades = 2;
    t.tailRoot = {{-4.05f, 0.1f, 1.66f}, {-5.15f, 0.1f, 1.66f}, 0.1f};
    t.tailTip = {{-4.35f, 1.73f, 1.66f}, {-5.0f, 1.73f, 1.66f}, 0.06f};
    t.finRoot = {{-3.7f, 0, 1.85f}, {-5.2f, 0, 1.78f}, 0.12f};
    t.finTip = {{-4.85f, 0, 2.72f}, {-5.35f, 0, 2.72f}, 0.08f};
    t.fixedGear = true;
    t.gearX = -0.1f, t.gearY = 1.25f, t.gearTop = 0.85f, t.wheelR = 0.28f;
    t.noseX = 2.05f, t.noseTop = 0.95f, t.noseWheelR = 0.22f, t.noseWheels = 1;
    return t;
}

const Station PA28[] = {{2.40f, 0.30f, 1.10f, 0.28f},  {1.95f, 0.48f, 1.08f, 0.42f},  {1.20f, 0.55f, 1.13f, 0.50f},
                        {0.55f, 0.58f, 1.22f, 0.60f},  {-0.90f, 0.55f, 1.25f, 0.56f}, {-1.60f, 0.42f, 1.28f, 0.46f},
                        {-3.20f, 0.22f, 1.36f, 0.28f}, {-4.55f, 0.09f, 1.42f, 0.15f}, {-4.85f, 0.04f, 1.44f, 0.07f}};

Transport lightLow() {  // the Piper Cherokee
    Transport t;
    t.stations = PA28, t.stationCount = 9, t.screen = 2, t.roundness = 10;
    t.windows[0] = 0.4f, t.windows[1] = -1.4f;
    t.windowsUp = 0.28f, t.windowsTall = 0.3f;
    t.wing.root = {0.55f, 1.0f, -0.6f, 0.72f, 0.2f};
    t.wing.kink = {2.6f, 1.0f, -0.6f, 0.97f, 0.19f};
    t.wing.tip = {5.33f, 0.85f, -0.25f, 1.3f, 0.1f};
    t.wing.flapsTo = 0.45f;
    t.wing.aileronFrom = 0.55f, t.wing.aileronTo = 0.95f;
    t.engines = NOSE_PROP;
    t.propX = 2.5f, t.propZ = 1.1f, t.propRadius = 0.94f, t.blades = 2;
    t.tailRoot = {{-3.75f, 0.1f, 1.42f}, {-4.6f, 0.1f, 1.42f}, 0.09f};
    t.tailTip = {{-3.85f, 1.85f, 1.42f}, {-4.6f, 1.85f, 1.42f}, 0.06f};
    t.allMovingTail = true;
    t.finRoot = {{-3.3f, 0, 1.6f}, {-4.75f, 0, 1.5f}, 0.1f};
    t.finTip = {{-4.25f, 0, 2.22f}, {-4.85f, 0, 2.22f}, 0.07f};
    t.fixedGear = true;
    t.gearX = 0.1f, t.gearY = 1.55f, t.gearTop = 0.7f, t.wheelR = 0.28f;
    t.noseX = 1.85f, t.noseTop = 0.75f, t.noseWheelR = 0.22f, t.noseWheels = 1;
    return t;
}

const Station C17_SPONSON[] = {{4.0f, 0.3f, 2.1f, 0.4f}, {2.8f, 1.0f, 2.0f, 1.0f}, {-5.0f, 1.0f, 2.0f, 1.0f},
                               {-7.0f, 0.3f, 2.3f, 0.4f}};

Transport airlifter() {  // the C-17 Globemaster III
    Transport t;
    t.shape = {22.5f, -30.5f, 3.15f, 3.3f, 4.9f, 5.5f, 15.5f, 3.3f};
    t.wing.root = {0.0f, 6.5f, -3.6f, 8.55f, 1.1f};
    t.wing.kink = {3.3f, 4.96f, -3.6f, 8.5f, 0.95f};
    t.wing.tip = {25.9f, -5.58f, -8.6f, 7.37f, 0.3f};
    t.wing.flapsTo = 0.6f;
    t.tip = TIP_SHARKLET, t.tipHeight = 2.9f;
    t.engineCount = 2, t.engineY[0] = 8.3f, t.engineY[1] = 15.2f;
    t.nacelleLength = 5.6f, t.nacelleRadius = 1.3f, t.engineDrop = 2.5f, t.engineAhead = 0.85f;
    t.tailRoot = {{-24.0f, 0.3f, 16.2f}, {-29.3f, 0.3f, 16.2f}, 0.5f};
    t.tailTip = {{-28.4f, 9.9f, 16.2f}, {-31.0f, 9.9f, 16.2f}, 0.2f};
    t.finRoot = {{-17.0f, 0, 8.1f}, {-27.0f, 0, 8.0f}, 0.8f};
    t.finTip = {{-25.3f, 0, 16.4f}, {-30.2f, 0, 16.4f}, 0.45f};
    t.gearX = -1.5f, t.gearY = 3.6f, t.gearTop = 1.8f, t.wheelR = 0.62f, t.wheels = 6, t.wheelGap = 0.45f,
    t.wheelWidth = 0.42f;
    t.noseX = 20.0f, t.noseTop = 2.5f, t.noseWheelR = 0.5f;
    t.sponson = C17_SPONSON, t.sponsonCount = 4, t.sponsonY = 3.0f;
    return t;
}

const Station C130_SPONSON[] = {{2.8f, 0.25f, 1.15f, 0.35f}, {2.0f, 0.8f, 1.1f, 0.8f}, {-2.4f, 0.8f, 1.1f, 0.8f},
                                {-3.4f, 0.25f, 1.3f, 0.35f}};

Transport hercules() {  // the C-130 Hercules
    Transport t;
    t.shape = {13.6f, -16.2f, 2.15f, 2.2f, 3.05f, 3.6f, 10.0f, 2.4f};
    t.wing.root = {0.0f, 2.25f, -2.7f, 5.65f, 0.75f};
    t.wing.kink = {10.0f, 2.25f, -2.7f, 5.85f, 0.65f};
    t.wing.tip = {20.2f, 1.0f, -1.25f, 6.1f, 0.2f};
    t.wing.flapsTo = 0.45f;
    t.wing.aileronFrom = 0.6f, t.wing.aileronTo = 0.95f;
    t.engines = WING_PROPS;
    t.engineCount = 2, t.engineY[0] = 4.9f, t.engineY[1] = 9.9f;
    t.propX = 4.0f, t.propZ = 4.95f, t.propRadius = 2.055f, t.blades = 4;
    t.nacelleLength = 7.4f, t.nacelleHalfWidth = 0.55f, t.nacelleHalfHeight = 0.85f;
    t.tailRoot = {{-11.6f, 0.3f, 5.6f}, {-15.4f, 0.3f, 5.6f}, 0.4f};
    t.tailTip = {{-13.9f, 8.0f, 5.6f}, {-15.6f, 8.0f, 5.6f}, 0.15f};
    t.finRoot = {{-8.6f, 0, 5.1f}, {-15.6f, 0, 5.4f}, 0.5f};
    t.finTip = {{-14.2f, 0, 11.84f}, {-16.4f, 0, 11.84f}, 0.2f};
    t.gearY = 2.15f, t.gearTop = 1.2f, t.wheelR = 0.6f, t.wheelGap = 0.35f, t.wheelWidth = 0.35f;
    t.noseX = 9.8f, t.noseTop = 1.4f, t.noseWheelR = 0.45f;
    t.sponson = C130_SPONSON, t.sponsonCount = 4, t.sponsonY = 2.0f;
    return t;
}

const Station A400_SPONSON[] = {{3.0f, 0.3f, 1.65f, 0.4f}, {2.0f, 0.9f, 1.6f, 0.9f}, {-4.2f, 0.9f, 1.6f, 0.9f},
                                {-5.5f, 0.3f, 1.8f, 0.4f}};

Transport atlas() {  // the A400M Atlas
    Transport t;
    t.shape = {19.5f, -25.6f, 2.85f, 2.95f, 4.15f, 4.4f, 13.5f, 3.2f};
    t.wing.root = {0.0f, 3.6f, -2.9f, 7.5f, 0.9f};
    t.wing.kink = {3.0f, 2.8f, -2.9f, 7.5f, 0.85f};
    t.wing.tip = {21.2f, -2.1f, -4.6f, 7.4f, 0.25f};
    t.wing.flapsTo = 0.5f;
    t.engines = WING_PROPS;
    t.engineCount = 2, t.engineY[0] = 6.8f, t.engineY[1] = 12.4f;
    t.propX = 5.4f, t.propZ = 6.6f, t.propRadius = 2.665f, t.blades = 8;
    t.nacelleLength = 7.2f, t.nacelleHalfWidth = 0.62f, t.nacelleHalfHeight = 0.95f;
    t.counterRotating = true;
    t.tailRoot = {{-21.6f, 0.3f, 14.25f}, {-25.6f, 0.3f, 14.25f}, 0.4f};
    t.tailTip = {{-24.8f, 9.75f, 14.25f}, {-26.6f, 9.75f, 14.25f}, 0.15f};
    t.finRoot = {{-15.8f, 0, 7.0f}, {-24.2f, 0, 7.0f}, 0.6f};
    t.finTip = {{-22.6f, 0, 14.7f}, {-26.4f, 0, 14.7f}, 0.42f};
    t.gearX = -1.0f, t.gearY = 3.0f, t.gearTop = 1.6f, t.wheelR = 0.6f, t.wheels = 6, t.wheelGap = 0.38f,
    t.wheelWidth = 0.38f;
    t.noseX = 16.5f, t.noseTop = 2.2f, t.noseWheelR = 0.48f;
    t.sponson = A400_SPONSON, t.sponsonCount = 4, t.sponsonY = 2.6f;
    return t;
}

const Station C295_SPONSON[] = {{1.8f, 0.2f, 0.98f, 0.3f}, {1.2f, 0.62f, 0.95f, 0.6f}, {-2.2f, 0.62f, 0.95f, 0.6f},
                                {-3.0f, 0.2f, 1.1f, 0.3f}};

Transport tacticalTwin() {  // the C295
    Transport t;
    t.shape = {10.0f, -14.45f, 1.45f, 1.5f, 2.3f, 2.9f, 8.3f, 1.9f};
    t.roundness = 14;
    t.wing.root = {0.0f, 1.6f, -1.4f, 4.05f, 0.5f};
    t.wing.kink = {4.6f, 1.6f, -1.4f, 4.05f, 0.45f};
    t.wing.tip = {12.9f, 1.1f, -0.4f, 4.3f, 0.15f};
    t.wing.flapsTo = 0.55f;
    t.tip = TIP_SHARKLET, t.tipHeight = 1.0f;
    t.engines = WING_PROPS;
    t.engineY[0] = 4.1f;
    t.propX = 3.9f, t.propZ = 3.4f, t.propRadius = 1.965f, t.blades = 6;
    t.nacelleLength = 6.2f, t.nacelleHalfWidth = 0.5f, t.nacelleHalfHeight = 0.78f;
    t.tailRoot = {{-10.1f, 0.3f, 3.85f}, {-12.6f, 0.3f, 3.85f}, 0.25f};
    t.tailTip = {{-11.3f, 5.15f, 3.95f}, {-12.9f, 5.15f, 3.95f}, 0.12f};
    t.finRoot = {{-8.2f, 0, 3.7f}, {-13.1f, 0, 3.9f}, 0.35f};
    t.finTip = {{-12.2f, 0, 8.66f}, {-14.3f, 0, 8.66f}, 0.2f};
    t.gearY = 1.55f, t.gearTop = 0.95f, t.wheelR = 0.45f, t.wheelGap = 0.25f, t.wheelWidth = 0.28f;
    t.noseX = 8.0f, t.noseTop = 1.2f, t.noseWheelR = 0.32f;
    t.sponson = C295_SPONSON, t.sponsonCount = 4, t.sponsonY = 1.4f;
    return t;
}

const Station BAE146_SPONSON[] = {{1.2f, 0.2f, 1.05f, 0.3f}, {0.6f, 0.55f, 1.0f, 0.55f}, {-1.8f, 0.55f, 1.0f, 0.55f},
                                  {-2.6f, 0.2f, 1.15f, 0.3f}};

Transport quadHigh() {  // the BAe 146-300
    Transport t;
    t.shape = {14.5f, -16.5f, 1.78f, 1.78f, 2.6f, 3.0f, 8.5f, 1.1f};
    t.windows[0] = 11.5f, t.windows[1] = -7.5f;
    t.windowsUp = 0.2f;
    t.wing.root = {0.0f, 2.3f, -2.0f, 4.5f, 0.55f};
    t.wing.kink = {3.0f, 1.5f, -2.0f, 4.45f, 0.5f};
    t.wing.tip = {13.1f, -1.2f, -2.4f, 3.97f, 0.15f};
    t.wing.flapsTo = 0.7f, t.wing.aileronFrom = 0.75f;
    t.wing.spoilers = true;
    t.engineCount = 2, t.engineY[0] = 3.8f, t.engineY[1] = 6.4f;
    t.nacelleLength = 3.4f, t.nacelleRadius = 0.72f, t.engineDrop = 1.3f, t.engineAhead = 0.7f;
    t.tailRoot = {{-13.2f, 0.2f, 8.45f}, {-16.3f, 0.2f, 8.45f}, 0.3f};
    t.tailTip = {{-15.3f, 5.7f, 8.45f}, {-16.8f, 5.7f, 8.45f}, 0.12f};
    t.finRoot = {{-9.6f, 0, 4.25f}, {-14.9f, 0, 4.0f}, 0.4f};
    t.finTip = {{-13.5f, 0, 8.61f}, {-16.6f, 0, 8.61f}, 0.3f};
    t.gearY = 2.35f, t.gearTop = 1.0f, t.wheelR = 0.43f, t.wheelGap = 0.25f, t.wheelWidth = 0.26f;
    t.noseX = 11.4f, t.noseTop = 1.2f, t.noseWheelR = 0.3f;
    t.sponson = BAE146_SPONSON, t.sponsonCount = 4, t.sponsonY = 1.75f;
    return t;
}

const Station TYPHOON[] = {{9.40f, 0.03f, 2.05f, 0.03f},  {8.90f, 0.16f, 2.05f, 0.16f},  {7.90f, 0.40f, 2.02f, 0.40f},
                           {6.50f, 0.55f, 2.00f, 0.55f},  {5.20f, 0.62f, 2.00f, 0.64f},  {4.00f, 0.90f, 1.92f, 0.80f},
                           {2.00f, 1.10f, 1.86f, 0.86f},  {-1.00f, 1.15f, 1.84f, 0.80f}, {-4.40f, 1.05f, 1.86f, 0.70f},
                           {-6.30f, 0.92f, 1.86f, 0.62f}};
const Station TYPHOON_CANOPY[] = {{6.90f, 0.05f, 2.48f, 0.05f}, {6.25f, 0.32f, 2.62f, 0.28f},
                                  {5.20f, 0.40f, 2.70f, 0.34f}, {4.10f, 0.33f, 2.66f, 0.28f},
                                  {3.30f, 0.08f, 2.58f, 0.06f}};
const Station TYPHOON_INTAKE[] = {{4.30f, 0.62f, 1.12f, 0.36f}, {3.00f, 0.72f, 1.18f, 0.42f},
                                  {0.50f, 0.70f, 1.30f, 0.40f}, {-0.80f, 0.30f, 1.45f, 0.20f}};

Fighter typhoon() {  // the Eurofighter Typhoon
    Fighter f;
    f.body = TYPHOON, f.bodyCount = 10;
    f.canopy = TYPHOON_CANOPY, f.canopyCount = 5;
    f.intake = TYPHOON_INTAKE, f.intakeCount = 4;
    f.wing.root = {1.05f, 2.4f, -5.2f, 1.55f, 0.4f};
    f.wing.kink = f.wing.root;
    f.wing.tip = {5.47f, -3.5f, -4.85f, 1.55f, 0.1f};
    f.wing.elevons = true;
    f.wing.aileronChord = 0.2f;
    f.canards = true;
    f.canardRoot = {{4.75f, 0.85f, 2.05f}, {3.45f, 0.85f, 2.05f}, 0.1f};
    f.canardTip = {{3.35f, 2.4f, 2.1f}, {2.85f, 2.4f, 2.1f}, 0.05f};
    f.finRoot = {{-1.6f, 0, 2.55f}, {-5.9f, 0, 2.45f}, 0.3f};
    f.finTip = {{-5.0f, 0, 5.28f}, {-6.4f, 0, 5.28f}, 0.12f};
    f.nozzles = 2, f.nozzleX = -6.3f, f.nozzleY = 0.45f, f.nozzleZ = 1.78f, f.nozzleR = 0.42f;
    f.gearX = -0.3f, f.gearY = 1.55f, f.gearTop = 1.2f, f.wheelR = 0.38f;
    f.noseX = 5.4f, f.noseTop = 1.0f, f.noseWheelR = 0.26f;
    return f;
}

const Station F35[] = {{9.20f, 0.03f, 1.92f, 0.03f},  {8.60f, 0.34f, 1.92f, 0.28f},  {7.40f, 0.62f, 1.95f, 0.48f},
                       {6.00f, 0.85f, 1.95f, 0.60f},  {4.40f, 1.30f, 1.90f, 0.72f},  {2.40f, 1.48f, 1.86f, 0.78f},
                       {0.00f, 1.52f, 1.82f, 0.74f},  {-3.00f, 1.38f, 1.82f, 0.68f}, {-5.00f, 0.95f, 1.78f, 0.60f},
                       {-6.40f, 0.66f, 1.74f, 0.58f}};
const Station F35_CANOPY[] = {{7.60f, 0.05f, 2.38f, 0.05f}, {6.80f, 0.38f, 2.56f, 0.30f}, {5.50f, 0.44f, 2.64f, 0.34f},
                              {4.40f, 0.34f, 2.58f, 0.26f}, {3.60f, 0.06f, 2.48f, 0.05f}};
const Station F35_INTAKE[] = {{4.90f, 0.30f, 2.00f, 0.42f}, {4.00f, 0.36f, 1.98f, 0.46f}, {2.20f, 0.32f, 1.95f, 0.40f},
                              {1.00f, 0.12f, 1.92f, 0.20f}};

Fighter f35() {  // the F-35A Lightning II
    Fighter f;
    f.body = F35, f.bodyCount = 10;
    f.canopy = F35_CANOPY, f.canopyCount = 5;
    f.intake = F35_INTAKE, f.intakeCount = 4, f.intakeY = 1.25f;
    f.wing.root = {1.45f, 2.0f, -3.6f, 1.9f, 0.35f};
    f.wing.kink = f.wing.root;
    f.wing.tip = {5.35f, -0.77f, -2.62f, 1.9f, 0.1f};
    f.wing.aileronFrom = 0.4f, f.wing.aileronTo = 0.95f, f.wing.aileronChord = 0.28f;
    f.tailplane = true;
    f.tailRoot = {{-3.9f, 1.1f, 1.82f}, {-6.3f, 1.1f, 1.82f}, 0.15f};
    f.tailTip = {{-5.5f, 3.45f, 1.82f}, {-6.7f, 3.45f, 1.82f}, 0.06f};
    f.fins = 2;
    f.finRoot = {{-2.3f, 1.05f, 2.45f}, {-5.0f, 1.05f, 2.4f}, 0.18f};
    f.finTip = {{-4.5f, 1.85f, 4.38f}, {-5.8f, 1.85f, 4.38f}, 0.08f};
    f.nozzles = 1, f.nozzleX = -6.4f, f.nozzleZ = 1.74f, f.nozzleR = 0.6f;
    f.gearX = -0.5f, f.gearY = 2.15f, f.gearTop = 1.3f, f.wheelR = 0.4f;
    f.noseX = 5.6f, f.noseTop = 1.3f, f.noseWheelR = 0.25f;
    return f;
}

const Station F16[] = {{8.50f, 0.03f, 1.98f, 0.03f},  {8.00f, 0.30f, 1.98f, 0.30f},  {6.80f, 0.52f, 2.02f, 0.50f},
                       {5.60f, 0.60f, 2.02f, 0.60f},  {4.00f, 0.65f, 1.98f, 0.70f},  {2.00f, 0.85f, 1.94f, 0.72f},
                       {0.00f, 0.95f, 1.92f, 0.68f},  {-3.00f, 0.85f, 1.94f, 0.64f}, {-5.20f, 0.62f, 1.90f, 0.55f},
                       {-6.55f, 0.50f, 1.88f, 0.50f}};
const Station F16_CANOPY[] = {{6.70f, 0.05f, 2.50f, 0.05f}, {6.00f, 0.36f, 2.68f, 0.30f}, {4.90f, 0.40f, 2.76f, 0.36f},
                              {3.70f, 0.30f, 2.68f, 0.26f}, {2.90f, 0.06f, 2.56f, 0.05f}};
const Station F16_INTAKE[] = {{4.30f, 0.52f, 1.15f, 0.42f}, {3.20f, 0.55f, 1.18f, 0.45f}, {0.80f, 0.50f, 1.32f, 0.40f},
                              {-0.60f, 0.20f, 1.50f, 0.20f}};

Fighter f16() {  // the F-16 Fighting Falcon
    Fighter f;
    f.body = F16, f.bodyCount = 10;
    f.canopy = F16_CANOPY, f.canopyCount = 5;
    f.intake = F16_INTAKE, f.intakeCount = 4;
    f.wing.root = {0.95f, 1.1f, -3.95f, 1.92f, 0.3f};
    f.wing.kink = f.wing.root;
    f.wing.tip = {4.73f, -2.07f, -3.25f, 1.92f, 0.08f};
    f.wing.aileronFrom = 0.25f, f.wing.aileronTo = 0.9f, f.wing.aileronChord = 0.25f;
    f.tailplane = true;
    f.tailRoot = {{-4.25f, 0.9f, 1.86f}, {-6.35f, 0.9f, 1.86f}, 0.12f};
    f.tailTip = {{-5.45f, 2.85f, 1.62f}, {-6.6f, 2.85f, 1.62f}, 0.05f};
    f.finRoot = {{-2.5f, 0, 2.5f}, {-5.95f, 0, 2.4f}, 0.25f};
    f.finTip = {{-4.95f, 0, 4.88f}, {-6.45f, 0, 4.88f}, 0.1f};
    f.nozzles = 1, f.nozzleX = -6.55f, f.nozzleZ = 1.88f, f.nozzleR = 0.5f;
    f.gearX = -0.6f, f.gearY = 1.18f, f.gearTop = 1.3f, f.wheelR = 0.36f;
    f.noseX = 3.9f, f.noseTop = 0.9f, f.noseWheelR = 0.26f;
    return f;
}

const Station HAWK[] = {{6.00f, 0.02f, 1.55f, 0.02f},  {5.50f, 0.27f, 1.55f, 0.27f},  {4.40f, 0.45f, 1.62f, 0.45f},
                        {3.00f, 0.53f, 1.70f, 0.58f},  {1.50f, 0.74f, 1.66f, 0.62f},  {0.00f, 0.80f, 1.62f, 0.60f},
                        {-2.50f, 0.65f, 1.66f, 0.50f}, {-4.50f, 0.43f, 1.70f, 0.42f}, {-5.70f, 0.33f, 1.72f, 0.33f}};
const Station HAWK_CANOPY[] = {{4.40f, 0.05f, 2.08f, 0.05f}, {3.80f, 0.38f, 2.24f, 0.30f}, {2.50f, 0.42f, 2.34f, 0.35f},
                               {1.00f, 0.40f, 2.32f, 0.33f}, {0.00f, 0.25f, 2.22f, 0.20f}, {-0.60f, 0.05f, 2.12f, 0.04f}};
const Station HAWK_INTAKE[] = {{1.90f, 0.28f, 1.78f, 0.38f}, {1.20f, 0.30f, 1.76f, 0.38f}, {-0.20f, 0.18f, 1.72f, 0.25f}};

Fighter hawk() {  // the BAE Hawk
    Fighter f;
    f.body = HAWK, f.bodyCount = 9;
    f.canopy = HAWK_CANOPY, f.canopyCount = 6;
    f.intake = HAWK_INTAKE, f.intakeCount = 3, f.intakeY = 0.72f;
    f.wing.root = {0.75f, 1.4f, -1.6f, 1.22f, 0.32f};
    f.wing.kink = f.wing.root;
    f.wing.tip = {4.7f, -0.53f, -1.53f, 1.36f, 0.1f};
    f.wing.flapsTo = 0.5f, f.wing.aileronFrom = 0.55f, f.wing.aileronTo = 0.95f;
    f.tailplane = true;
    f.tailRoot = {{-4.0f, 0.4f, 1.95f}, {-5.25f, 0.4f, 1.95f}, 0.1f};
    f.tailTip = {{-4.95f, 2.2f, 1.72f}, {-5.65f, 2.2f, 1.72f}, 0.05f};
    f.finRoot = {{-2.7f, 0, 2.12f}, {-5.45f, 0, 2.1f}, 0.22f};
    f.finTip = {{-4.75f, 0, 3.98f}, {-5.8f, 0, 3.98f}, 0.1f};
    f.nozzles = 1, f.nozzleX = -5.7f, f.nozzleZ = 1.72f, f.nozzleR = 0.3f;
    f.gearX = 0.0f, f.gearY = 1.75f, f.gearTop = 1.1f, f.wheelR = 0.32f;
    f.noseX = 3.4f, f.noseTop = 0.95f, f.noseWheelR = 0.22f;
    return f;
}

const Station AW139[] = {{5.50f, 0.10f, 1.30f, 0.10f},  {5.10f, 0.55f, 1.32f, 0.52f},  {4.30f, 0.88f, 1.48f, 0.82f},
                         {3.00f, 1.06f, 1.60f, 0.96f},  {0.80f, 1.08f, 1.62f, 0.98f},  {-1.10f, 0.95f, 1.72f, 0.88f},
                         {-2.50f, 0.50f, 2.02f, 0.50f}, {-6.60f, 0.28f, 2.32f, 0.28f}, {-8.30f, 0.22f, 2.42f, 0.24f}};
const Station AW139_FAIRING[] = {{2.60f, 0.20f, 2.45f, 0.10f}, {2.10f, 0.62f, 2.62f, 0.40f},
                                 {-1.60f, 0.66f, 2.66f, 0.42f}, {-2.80f, 0.30f, 2.58f, 0.20f}};
const Station AW139_SPONSON[] = {{0.9f, 0.15f, 0.95f, 0.2f}, {0.5f, 0.35f, 0.95f, 0.32f}, {-0.9f, 0.35f, 0.95f, 0.32f},
                                 {-1.3f, 0.15f, 1.0f, 0.2f}};

Rotorcraft aw139() {  // the AW139
    Rotorcraft r;
    r.body = AW139, r.bodyCount = 9, r.screen = 2;
    r.fairing = AW139_FAIRING, r.fairingCount = 4;
    r.windows[0] = 2.8f, r.windows[1] = -0.6f, r.windowsUp = 0.32f, r.windowsTall = 0.5f;
    r.rotor = {0.5f, 0, 3.55f}, r.rotorR = 6.9f, r.blades = 5;
    r.tailRotor = {-8.25f, 0.42f, 3.15f}, r.tailRotorR = 1.35f, r.tailBlades = 4;
    r.finRoot = {{-7.4f, 0, 2.4f}, {-8.55f, 0, 2.45f}, 0.18f};
    r.finTip = {{-8.25f, 0, 3.72f}, {-8.95f, 0, 3.72f}, 0.1f};
    r.stabRoot = {{-6.2f, 0.2f, 2.33f}, {-7.0f, 0.2f, 2.33f}, 0.1f};
    r.stabTip = {{-6.3f, 1.45f, 2.33f}, {-7.0f, 1.45f, 2.33f}, 0.06f};
    r.sponson = AW139_SPONSON, r.sponsonCount = 4, r.sponsonY = 1.12f;
    r.gearX = 0, r.gearY = 1.25f, r.gearTop = 0.9f, r.wheelR = 0.3f;
    r.noseX = 4.0f, r.noseTop = 0.9f, r.noseWheelR = 0.2f;
    return r;
}

const Station H145[] = {{4.10f, 0.10f, 1.22f, 0.10f},  {3.70f, 0.50f, 1.24f, 0.48f},  {3.00f, 0.76f, 1.42f, 0.74f},
                        {1.80f, 0.83f, 1.54f, 0.84f},  {0.00f, 0.83f, 1.58f, 0.83f},  {-1.20f, 0.70f, 1.72f, 0.72f},
                        {-2.20f, 0.34f, 2.06f, 0.36f}, {-6.00f, 0.20f, 2.32f, 0.20f}, {-6.90f, 0.18f, 2.38f, 0.20f}};
const Station H145_FAIRING[] = {{2.00f, 0.15f, 2.32f, 0.08f}, {1.50f, 0.52f, 2.45f, 0.32f},
                                {-1.40f, 0.55f, 2.48f, 0.34f}, {-2.40f, 0.25f, 2.40f, 0.16f}};

Rotorcraft h145() {  // the H145
    Rotorcraft r;
    r.body = H145, r.bodyCount = 9, r.screen = 2;
    r.fairing = H145_FAIRING, r.fairingCount = 4;
    r.windows[0] = 1.6f, r.windows[1] = -0.8f, r.windowsUp = 0.28f, r.windowsTall = 0.45f;
    r.rotor = {0.25f, 0, 3.35f}, r.rotorR = 5.5f, r.blades = 4;
    r.tailRotor = {-6.85f, 0.32f, 2.85f}, r.tailRotorR = 0.98f, r.tailBlades = 2;
    r.finRoot = {{-6.05f, 0, 2.3f}, {-6.95f, 0, 2.35f}, 0.15f};
    r.finTip = {{-6.7f, 0, 3.55f}, {-7.2f, 0, 3.55f}, 0.08f};
    r.stabRoot = {{-5.4f, 0.15f, 2.27f}, {-6.0f, 0.15f, 2.27f}, 0.08f};
    r.stabTip = {{-5.45f, 1.2f, 2.27f}, {-6.0f, 1.2f, 2.27f}, 0.05f};
    r.skids = true, r.skidY = 1.1f, r.skidFrom = 2.3f, r.skidTo = -1.9f;
    return r;
}

const Station CHINOOK[] = {{8.10f, 0.20f, 1.80f, 0.25f},  {7.60f, 0.95f, 1.95f, 1.05f},  {6.60f, 1.35f, 2.15f, 1.40f},
                           {5.20f, 1.45f, 2.22f, 1.52f},  {-5.20f, 1.45f, 2.22f, 1.52f}, {-7.20f, 1.32f, 2.42f, 1.36f},
                           {-7.90f, 1.00f, 2.75f, 0.95f}};
const Station CHINOOK_PYLON[] = {{-3.6f, 0.45f, 3.85f, 0.25f}, {-4.6f, 0.85f, 4.3f, 0.75f}, {-7.4f, 0.85f, 4.5f, 0.85f},
                                 {-8.0f, 0.4f, 4.85f, 0.4f}};
const Station CHINOOK_FRONT[] = {{6.6f, 0.3f, 3.75f, 0.2f}, {6.0f, 0.5f, 3.92f, 0.35f}, {4.9f, 0.5f, 3.92f, 0.35f},
                                 {4.4f, 0.2f, 3.8f, 0.15f}};
const Station CHINOOK_ENGINE[] = {{-3.6f, 0.2f, 3.65f, 0.2f}, {-4.0f, 0.5f, 3.65f, 0.5f}, {-6.4f, 0.48f, 3.65f, 0.48f},
                                  {-6.9f, 0.3f, 3.65f, 0.3f}};
const Station CHINOOK_SPONSON[] = {{4.4f, 0.2f, 1.1f, 0.3f}, {3.8f, 0.42f, 1.05f, 0.52f}, {-3.6f, 0.42f, 1.05f, 0.52f},
                                   {-4.2f, 0.2f, 1.1f, 0.3f}};

Rotorcraft chinook() {  // the CH-47 Chinook
    Rotorcraft r;
    r.body = CHINOOK, r.bodyCount = 7, r.screen = 1;
    r.fairing = CHINOOK_FRONT, r.fairingCount = 4;
    r.pylon = CHINOOK_PYLON, r.pylonCount = 4;
    r.engine = CHINOOK_ENGINE, r.engineCount = 4, r.engineY = 1.3f;
    r.rotor = {5.55f, 0, 4.45f}, r.rotorR = 9.14f, r.blades = 3;
    r.tandem = true, r.rearRotor = {-5.95f, 0, 5.55f};
    r.sponson = CHINOOK_SPONSON, r.sponsonCount = 4, r.sponsonY = 1.62f;
    r.fixedGear = true;
    r.gearX = -3.9f, r.gearY = 1.35f, r.gearTop = 0.8f, r.wheelR = 0.45f;
    r.noseX = 3.9f, r.noseY = 1.2f, r.noseTop = 0.8f, r.noseWheelR = 0.45f;
    return r;
}

// Each family's model's length, nose to tail - a helicopter's without its
// rotors - which a type's own is given against.
const float FAMILY_LENGTH[] = {
    37.65f,  // NARROWBODY
    41.5f,   // NARROWBODY_4
    41.5f,   // AWACS
    62.8f,   // WIDEBODY
    63.7f,   // WIDEBODY_4
    70.6f,   // JUMBO
    72.7f,   // SUPERJUMBO
    36.2f,   // REGIONAL_JET
    30.4f,   // BUSINESS_JET
    27.2f,   // TURBOPROP
    19.7f,   // TURBOPROP_LOW
    8.3f,    // LIGHT_HIGH
    7.4f,    // LIGHT_LOW
    53.0f,   // AIRLIFTER
    29.8f,   // HERCULES
    45.1f,   // ATLAS
    24.45f,  // TACTICAL_TWIN
    31.0f,   // QUAD_HIGH
    15.7f,   // FIGHTER_DELTA
    15.6f,   // FIGHTER
    15.05f,  // FIGHTER_SINGLE
    11.7f,   // JET_TRAINER
    13.8f,   // HELICOPTER
    11.0f,   // HELICOPTER_LIGHT
    16.0f,   // CHINOOK
};
static_assert(sizeof(FAMILY_LENGTH) / sizeof(FAMILY_LENGTH[0]) == (size_t)Family::COUNT, "a length for each family");

// --- building them ------------------------------------------------------------------

// A model as its corners and faces: each corner once - two faces' corners
// one where they are the same point and move together - found through a
// hash of where it is.
Mesh indexed(const Vector<ModelTri> &tris) {
    Mesh m;
    size_t cap = 64;
    while (cap < tris.size() * 6) {
        cap <<= 1;
    }
    Vector<int32_t> table(cap, -1);
    auto bits = [](float v) {
        v += 0.0f;  // -0 as 0
        uint32_t u;
        memcpy(&u, &v, sizeof u);
        return u;
    };
    // Where each corner is, as found - it is kept to the centimetre.
    Vector<Vec3> found;
    auto centimetres = [](float v) { return (int16_t)std::max(-32767.0f, std::min(32767.0f, roundf(v * 100))); };
    auto of127 = [](float v) { return (int8_t)std::max(-127.0f, std::min(127.0f, roundf(v * 127))); };
    for (const ModelTri &t : tris) {
        for (const Vec3 &v : t.v) {
            uint32_t h = bits(v.x) * 73856093u ^ bits(v.y) * 19349663u ^ bits(v.z) * 83492791u ^ t.move * 2654435761u;
            size_t i = h & (cap - 1);
            while (table[i] >= 0) {
                const Vec3 &c = found[table[i]];
                if (c.x == v.x && c.y == v.y && c.z == v.z && m.moves[table[i]] == t.move) {
                    break;
                }
                i = (i + 1) & (cap - 1);
            }
            if (table[i] < 0) {
                table[i] = (int32_t)found.size();
                found.push_back(v);
                m.corners.push_back(centimetres(v.x));
                m.corners.push_back(centimetres(v.y));
                m.corners.push_back(centimetres(v.z));
                m.moves.push_back(t.move);
            }
            m.faces.push_back((uint16_t)table[i]);
        }
        Vec3 n = normalize(cross(t.v[1] - t.v[0], t.v[2] - t.v[0]));
        m.normals.push_back(of127(n.x));
        m.normals.push_back(of127(n.y));
        m.normals.push_back(of127(n.z));
        m.colors.push_back(paletteIndex(t.color));
        int blur = t.blur == 0 ? 0 : std::max(1, std::min(3, t.blur - 1));
        m.looks.push_back((uint8_t)(t.paint | t.show << 3 | blur << 6));
        m.moving = m.moving || t.move != MOVE_NONE;
    }
    m.corners.shrink_to_fit();
    m.moves.shrink_to_fit();
    m.faces.shrink_to_fit();
    m.normals.shrink_to_fit();
    m.colors.shrink_to_fit();
    m.looks.shrink_to_fit();
    return m;
}

void buildLevel(Family f, int level, Vector<ModelTri> &out, Design &d) {
    switch (f) {
    case Family::NARROWBODY: buildTransport(out, level, narrowbody(), d); break;
    case Family::NARROWBODY_4: buildTransport(out, level, narrowbody4(), d); break;
    case Family::AWACS: buildTransport(out, level, awacs(), d); break;
    case Family::WIDEBODY: buildTransport(out, level, widebody(), d); break;
    case Family::WIDEBODY_4: buildTransport(out, level, widebody4(), d); break;
    case Family::JUMBO: buildTransport(out, level, jumbo(), d); break;
    case Family::SUPERJUMBO: buildTransport(out, level, superjumbo(), d); break;
    case Family::REGIONAL_JET: buildTransport(out, level, regional(), d); break;
    case Family::BUSINESS_JET: buildTransport(out, level, bizjet(), d); break;
    case Family::TURBOPROP: buildTransport(out, level, turboprop(), d); break;
    case Family::TURBOPROP_LOW: buildTransport(out, level, turbopropLow(), d); break;
    case Family::LIGHT_HIGH: buildTransport(out, level, lightHigh(), d); break;
    case Family::LIGHT_LOW: buildTransport(out, level, lightLow(), d); break;
    case Family::AIRLIFTER: buildTransport(out, level, airlifter(), d); break;
    case Family::HERCULES: buildTransport(out, level, hercules(), d); break;
    case Family::ATLAS: buildTransport(out, level, atlas(), d); break;
    case Family::TACTICAL_TWIN: buildTransport(out, level, tacticalTwin(), d); break;
    case Family::QUAD_HIGH: buildTransport(out, level, quadHigh(), d); break;
    case Family::FIGHTER_DELTA: buildFighter(out, level, typhoon(), d); break;
    case Family::FIGHTER: buildFighter(out, level, f35(), d); break;
    case Family::FIGHTER_SINGLE: buildFighter(out, level, f16(), d); break;
    case Family::JET_TRAINER: buildFighter(out, level, hawk(), d); break;
    case Family::HELICOPTER: buildRotorcraft(out, level, aw139(), d); break;
    case Family::HELICOPTER_LIGHT: buildRotorcraft(out, level, h145(), d); break;
    case Family::CHINOOK: buildRotorcraft(out, level, chinook(), d); break;
    default: buildTransport(out, level, narrowbody(), d); break;
    }
}

// Runs `work` where there is room for it: building a model goes some 5KB
// deep - wings() alone takes 1.2KB - which a caller already deep in its own
// work, the loop task's 8KB, can't spare. So on the device it runs on a task
// of its own for the while, with 16KB, the caller waiting for it.
template <typename Work>
void withRoom(Work &&work) {
#ifdef ESP_PLATFORM
    struct Job {
        Work *work;
        SemaphoreHandle_t done;
    } job = {&work, xSemaphoreCreateBinary()};
    auto run = [](void *p) {
        Job *j = (Job *)p;
        (*j->work)();
        xSemaphoreGive(j->done);
        vTaskDelete(nullptr);
    };
    if (job.done != nullptr &&
        xTaskCreatePinnedToCore(run, "models", 16384, &job, uxTaskPriorityGet(nullptr), nullptr, xPortGetCoreID()) ==
            pdPASS) {
        xSemaphoreTake(job.done, portMAX_DELAY);
        vSemaphoreDelete(job.done);
        return;
    }
    if (job.done != nullptr) {
        vSemaphoreDelete(job.done);
    }
#endif
    work();
}

void buildDesign(Family f, Design &d) {
    d = Design{};
    for (int level = DETAIL_HERO; level <= DETAIL_TINY; level++) {
        Vector<ModelTri> tris;
        g_pen = Pen{};
        buildLevel(f, level, tris, d);
        d.lod[level] = indexed(tris);
    }
    g_pen = Pen{};
    d.lights.shrink_to_fit();
    // How big it is: the box its full model fits in. Its length is its
    // body's, nose to tail, as a type's is given.
    const Mesh &hero = d.lod[DETAIL_HERO];
    d.lo = {1e9f, 1e9f, 1e9f};
    d.hi = {-1e9f, -1e9f, -1e9f};
    for (size_t i = 0; i < hero.cornerCount(); i++) {
        Vec3 c = hero.corner(i);
        d.lo = {std::min(d.lo.x, c.x), std::min(d.lo.y, c.y), std::min(d.lo.z, c.z)};
        d.hi = {std::max(d.hi.x, c.x), std::max(d.hi.y, c.y), std::max(d.hi.z, c.z)};
    }
    d.middle = (d.lo + d.hi) * 0.5f;
    d.radius = length(d.hi - d.middle);
    d.halfLength = std::max(d.hi.x - d.lo.x, d.hi.y - d.lo.y) / 2;
    d.length = FAMILY_LENGTH[(int)f];
    // Only the one followed is drawn in full: its family's full model is
    // built again when it is wanted - see heroOf().
    d.lod[DETAIL_HERO] = Mesh{};
}

// A family's full model, built when one of it is followed and let go when
// one of another family is - only ever one aircraft is drawn in full.
// Called while a frame is begun, before the cores share out the aircraft.
const Mesh &heroOf(Family f) {
    static int built = -1;
    Design &d = designs()[(int)f];
    if (d.lod[DETAIL_HERO].faceCount() == 0) {
        if (built >= 0 && built != (int)f) {
            designs()[built].lod[DETAIL_HERO] = Mesh{};
        }
        // Built against a design of its own, so the one in use keeps its
        // spinners and hinges as they were counted - the same, in the same
        // order.
        withRoom([&] {
            static Design scratch;
            scratch = Design{};
            Vector<ModelTri> tris;
            g_pen = Pen{};
            buildLevel(f, DETAIL_HERO, tris, scratch);
            g_pen = Pen{};
            d.lod[DETAIL_HERO] = indexed(tris);
        });
        built = (int)f;
    }
    return d.lod[DETAIL_HERO];
}

// --- the airfield --------------------------------------------------------------
// Built from its runways - where each one's ends are, how wide it is, what its
// ends are called - painted and lit as ICAO has them, each with a taxiway
// alongside and ways on and off it; and, between the runways or beside the
// one, a terminal: a building with piers out from it and stands along them. A
// generic airport, but with its runways where they really are.

const Rgb ASPHALT = {66, 68, 72}, TAXIWAY = {84, 86, 88}, CONCRETE = {170, 170, 164}, PAINT = {236, 236, 236},
          YELLOW = {226, 184, 40}, GRASS = {94, 124, 66},  // the grass's drawn as the light has it: see PAINT_GRASS
          WATER = {44, 66, 90};                            // the sea's, a river's

// How far a runway's taxiway runs from its centreline, and how often a way
// off joins them.
constexpr float TAXIWAY_OFFSET = 180, CONNECTOR_EVERY = 500;

void rect(Vector<ModelTri> &out, Vec3 a, Vec3 b, Vec3 c, Vec3 d, Rgb color) {
    // The ground is seen only from above: no culling, no winding. How narrow
    // it is, for a marking to be left out where it would be lost.
    float fine = std::min(length(b - a), length(d - a));
    out.push_back({{a, b, c}, color, fine});
    out.push_back({{a, c, d}, color, fine});
}

// A strip of ground from a to b, `half` wide either side of the line between
// - a long thin one, a line, in pieces of 100m, so each piece is left out or
// not by how near it is.
void strip(Vector<ModelTri> &out, Vec3 a, Vec3 b, float half, Rgb color) {
    Vec3 d = normalize(b - a);
    Vec3 side = {-d.y * half, d.x * half, 0};
    float len = length(b - a);
    int pieces = half < 1 && len > 120 ? (int)ceilf(len / 100) : 1;
    for (int i = 0; i < pieces; i++) {
        Vec3 p0 = lerp(a, b, (float)i / pieces), p1 = lerp(a, b, (float)(i + 1) / pieces);
        rect(out, p0 - side, p1 - side, p1 + side, p0 + side, color);
    }
}

Vec3 leftOf(Vec3 along) { return {-along.y, along.x, 0}; }

// One end of a runway, as an aircraft landing there sees it: from its
// threshold, down the runway and to its right.
struct RunwayEnd {
    Vec3 origin, along, right;
    float length, halfWidth;
    int number;
    char letter;       // 'L', 'R', 'C', or 0
    float exitAlong;   // where, from here, the way off it the landing takes
    bool exitRight;    // and which side its taxiway is

    Vec3 at(float a, float r) const { return origin + along * a + right * r; }
    void paint(Vector<ModelTri> &out, float along0, float right0, float along1, float right1) const {
        rect(out, at(along0, right0), at(along1, right0), at(along1, right1), at(along0, right1), PAINT);
    }
    void paintBothSides(Vector<ModelTri> &out, float along0, float from, float along1, float to) const {
        paint(out, along0, from, along1, to);
        paint(out, along0, -from, along1, -to);
    }
};

// Runway numbers and letters, 9m tall, read by an aircraft on final: bars in
// a 3.6m-wide cell, u across it and v up it - the bars across the runway
// twice as thick as those along it, as foreshortening thins them.
constexpr float GLYPH_W = 3.6f, GLYPH_H = 9, BAR = 0.9f, CROSSBAR = 1.8f;

struct Bar {
    float u0, v0, u1, v1;
};

void paintBars(Vector<ModelTri> &out, const RunwayEnd &end, const Bar *bars, int n, float right, float along) {
    for (int i = 0; i < n; i++) {
        end.paint(out, along + bars[i].v0, right + bars[i].u0, along + bars[i].v1, right + bars[i].u1);
    }
}

void paintDigit(Vector<ModelTri> &out, const RunwayEnd &end, int digit, float right, float along) {
    static const uint8_t SEGMENTS[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
    constexpr float W = GLYPH_W, H = GLYPH_H, T = BAR, A = CROSSBAR;
    const Bar BARS[] = {{0, H - A, W, H}, {W - T, H / 2, W, H}, {W - T, 0, W, H / 2}, {0, 0, W, A},
                        {0, 0, T, H / 2}, {0, H / 2, T, H},     {0, H / 2 - A / 2, W, H / 2 + A / 2}};
    for (int s = 0; s < 7; s++) {
        if (SEGMENTS[digit] >> s & 1) {
            paintBars(out, end, &BARS[s], 1, right, along);
        }
    }
}

void paintLetter(Vector<ModelTri> &out, const RunwayEnd &end, char letter, float right, float along) {
    constexpr float W = GLYPH_W, H = GLYPH_H, T = BAR, A = CROSSBAR;
    const Bar L[] = {{0, 0, T, H}, {0, 0, W, A}};
    const Bar C[] = {{0, 0, T, H}, {0, 0, W, A}, {0, H - A, W, H}};
    const Bar R[] = {{0, 0, T, H}, {0, H - A, W, H}, {W - T, H / 2, W, H}, {0, H / 2 - A / 2, W, H / 2 + A / 2}};
    if (letter == 'L') {
        paintBars(out, end, L, 2, right, along);
    } else if (letter == 'C') {
        paintBars(out, end, C, 3, right, along);
    } else if (letter == 'R') {
        paintBars(out, end, R, 4, right, along);
        // The R's leg, from its middle down to the right.
        Vec3 a = end.at(along + H / 2, right + W / 2 - T / 2), b = end.at(along + H / 2, right + W / 2 + T / 2),
             c = end.at(along, right + W), d = end.at(along, right + W - T);
        rect(out, a, b, c, d, PAINT);
    }
}

void paintEnd(Vector<ModelTri> &out, const RunwayEnd &end) {
    // The threshold's piano keys, as many a side as its width takes.
    int keys = std::max(3, std::min(8, (int)((end.halfWidth - 2.7f) / 3.3f)));
    for (int i = 0; i < keys; i++) {
        end.paintBothSides(out, 6, 2.7f + 3.3f * i, 36, 4.5f + 3.3f * i);
    }
    // Its name: a letter, nearest the threshold, then the number beyond.
    float numberAt = 48;
    if (end.letter != 0) {
        paintLetter(out, end, end.letter, -GLYPH_W / 2, 48);
        numberAt = 63;
    }
    paintDigit(out, end, end.number / 10, -4.35f, numberAt);
    paintDigit(out, end, end.number % 10, 0.75f, numberAt);
    // The touchdown zone, in threes, twos and ones, and the aiming point -
    // as far as the runway's length allows.
    const struct {
        float along;
        int bars;
    } ZONE[] = {{150, 3}, {300, 3}, {600, 2}, {750, 2}, {900, 1}};
    for (auto z : ZONE) {
        if (z.along + 22.5f > end.length / 2 - 50) {
            break;
        }
        for (int i = 0; i < z.bars; i++) {
            end.paintBothSides(out, z.along, 9 + 3.3f * i, z.along + 22.5f, 10.8f + 3.3f * i);
        }
    }
    if (end.length > 1500) {
        end.paintBothSides(out, 400, 9, 460, 17);
    }
}

const Rgb TERMINAL = {204, 196, 182}, ROOF = {120, 122, 128}, PIER = {196, 198, 202};

// A box turned to lie along `axis`, standing on the ground.
void orientedBox(Vector<ModelTri> &out, Vec3 centre, Vec3 axis, float halfLength, float halfWidth, float height,
                 Rgb sides, Rgb top) {
    Vec3 across = leftOf(axis), up = {0, 0, height};
    Vec3 c[4] = {centre - axis * halfLength - across * halfWidth, centre + axis * halfLength - across * halfWidth,
                 centre + axis * halfLength + across * halfWidth, centre - axis * halfLength + across * halfWidth};
    Vec3 inside = centre + Vec3{0, 0, height / 2};
    faceQuad(out, c[0] + up, c[1] + up, c[2] + up, c[3] + up, inside, top);
    for (int i = 0; i < 4; i++) {
        faceQuad(out, c[i], c[(i + 1) % 4], c[(i + 1) % 4] + up, c[i] + up, inside, sides);
    }
}

// The wireframe: what is all frame anyway, and too fine to fill - a line a
// strut costs next to nothing to draw.
const Rgb STEEL = {226, 230, 236}, LIGHT_FRAME = {150, 156, 162};

void line(Vector<ModelLine> &out, Vec3 a, Vec3 b, Rgb color) { out.push_back({a, b, color}); }

void circle(Vector<ModelLine> &out, Vec3 centre, float r, int sides, Rgb color, float turn = 0) {
    for (int k = 0; k < sides; k++) {
        float a0 = (k + turn) * 2 * PI / sides, a1 = (k + 1 + turn) * 2 * PI / sides;
        line(out, centre + Vec3{r * cosf(a0), r * sinf(a0), 0}, centre + Vec3{r * cosf(a1), r * sinf(a1), 0},
             color);
    }
}

// A Doppler VOR with its DME: a counterpoise disc 30m across on legs, a ring
// of antennas round its rim, the hut and the VOR's own antenna in the middle,
// and the DME's mast standing over them.
void buildVorDme(Vector<ModelLine> &out, Vec3 at) {
    constexpr float R = 15, DECK = 4.5f;
    Vec3 deck = at + Vec3{0, 0, DECK};
    circle(out, deck, R, 24, STEEL);
    circle(out, deck, R * 0.55f, 16, STEEL);
    for (int k = 0; k < 8; k++) {
        float a = k * 2 * PI / 8;
        Vec3 rim = {R * cosf(a), R * sinf(a), 0};
        line(out, at + rim * 0.8f, deck + rim * 0.8f, STEEL);  // the legs
        line(out, deck, deck + rim, STEEL);                    // the spokes
    }
    for (int k = 0; k < 24; k++) {
        float a = (k + 0.5f) * 2 * PI / 24;
        Vec3 rim = deck + Vec3{R * cosf(a), R * sinf(a), 0};
        line(out, rim, rim + Vec3{0, 0, 1.6f}, STEEL);  // the antennas round the rim
    }
    // The hut under the deck, and the cone of the antenna on it.
    for (int k = 0; k < 4; k++) {
        float a0 = (k + 0.5f) * PI / 2, a1 = (k + 1.5f) * PI / 2;
        Vec3 c0 = {3.5f * cosf(a0), 3.5f * sinf(a0), 0}, c1 = {3.5f * cosf(a1), 3.5f * sinf(a1), 0};
        line(out, at + c0, at + c0 + Vec3{0, 0, DECK}, STEEL);
        line(out, at + c0 + Vec3{0, 0, DECK}, at + c1 + Vec3{0, 0, DECK}, STEEL);
    }
    circle(out, deck + Vec3{0, 0, 0.4f}, 1.6f, 8, STEEL);
    for (int k = 0; k < 8; k++) {
        float a = k * 2 * PI / 8;
        line(out, deck + Vec3{1.6f * cosf(a), 1.6f * sinf(a), 0.4f}, deck + Vec3{0, 0, 3.2f}, STEEL);
    }
    // The DME: a mast, and its radome a ring at the top.
    line(out, deck + Vec3{0, 0, 3.2f}, deck + Vec3{0, 0, 8.5f}, STEEL);
    circle(out, deck + Vec3{0, 0, 7.6f}, 0.5f, 6, STEEL);
    circle(out, deck + Vec3{0, 0, 8.5f}, 0.5f, 6, STEEL);
}

// The control tower: an octagonal shaft tapering up 30m, the cab on top
// leaning out, its glass between mullions, a roof overhanging it and a mast.
void buildTower(Vector<ModelLine> &out, Vec3 foot) {
    constexpr int SIDES = 8;
    auto corner = [&](float r, int k, float z) {
        float a = (k + 0.5f) * 2 * PI / SIDES;
        return foot + Vec3{r * cosf(a), r * sinf(a), z};
    };
    for (int k = 0; k < SIDES; k++) {
        line(out, corner(3.5f, k, 0), corner(3.0f, k, 29.5f), STEEL);     // the shaft
        line(out, corner(5.4f, k, 30), corner(6.6f, k, 35), STEEL);       // the mullions
        line(out, corner(6.6f, k, 35), foot + Vec3{0, 0, 36.6f}, STEEL);  // the roof
    }
    for (float z : {0.0f, 10.0f, 20.0f}) {
        circle(out, foot + Vec3{0, 0, z}, 3.5f - 0.5f * z / 29.5f, SIDES, STEEL, 0.5f);
    }
    circle(out, foot + Vec3{0, 0, 29.5f}, 6.0f, SIDES, STEEL, 0.5f);  // the gallery under the cab
    circle(out, foot + Vec3{0, 0, 30}, 5.4f, SIDES, STEEL, 0.5f);
    circle(out, foot + Vec3{0, 0, 35}, 6.6f, SIDES, STEEL, 0.5f);
    line(out, foot + Vec3{0, 0, 36.6f}, foot + Vec3{0, 0, 42}, STEEL);
}

// The approach lights' frames: a bar every 30m out to 900m on the centreline,
// on poles - a little taller the further out - and the crossbar at 300m.
// A windsock beside a runway end, on its left past the aiming point: a mast
// and a striped cone, flying down the runway - without a wind to go by, as it
// would in the headwind an aircraft lands into.
void buildWindsock(Vector<ModelTri> &solids, Vector<ModelLine> &wires, const RunwayEnd &end) {
    Vec3 foot = end.at(300, -(end.halfWidth + 70)), top = foot + Vec3{0, 0, 6.5f};
    line(wires, foot, top, LIGHT_FRAME);
    const Rgb ORANGE = {240, 110, 30}, STRIPE = {236, 236, 232};
    Vec3 u = end.along, a = {0, 0, 1}, b = leftOf(end.along);
    for (int seg = 0; seg < 3; seg++) {
        const Ring PART[] = {{1.2f * seg, 0.45f - 0.08f * seg}, {1.2f * (seg + 1), 0.45f - 0.08f * (seg + 1)}};
        tube(solids, top, u, a, b, PART, 2, 6, seg % 2 ? STRIPE : ORANGE);
    }
}

void buildApproachFrames(Vector<ModelLine> &out, const RunwayEnd &end) {
    for (float along = -30; along >= -900; along -= 30) {
        float height = 0.6f - along * 0.004f;
        float half = along == -300 ? 15 : 2.1f;
        Vec3 foot = end.at(along, 0);
        line(out, foot, foot + Vec3{0, 0, height}, LIGHT_FRAME);
        line(out, end.at(along, -half) + Vec3{0, 0, height}, end.at(along, half) + Vec3{0, 0, height}, STEEL);
    }
}

// The lights, as they are by night: white along the edges, turning yellow in
// the last 600m; the centreline's white, alternating with red and then all red
// towards the end; across each threshold, green to an aircraft coming in and
// red to one rolling out towards it; the approach lights on their bars; and
// the taxiways' green centrelines.
const Rgb LAMP_WHITE = {255, 246, 224}, LAMP_YELLOW = {255, 200, 60}, LAMP_RED = {255, 50, 40},
          LAMP_GREEN = {60, 255, 120};

struct ModelLight {
    Vec3 at;
    Rgb color;
    float size;      // across, in metres; never drawn smaller than a pixel
    Vec3 facing;     // a light showing another colour behind it: which way its own faces; zero for one that doesn't
    Rgb behind;
    int end = -1;    // an approach light: the runway end it leads to, lit only while that end is in use
};

// An airfield's light as a frame reads it - a big airport has thousands, all
// read every frame after dark - a quarter the size: on the grid the ground
// is (see GroundTri), its facing in 127ths, its colours and its size from
// Models.lightColors and lightSizes, which hold the few there are exactly.
struct LightPoint {
    int16_t x, y, z;
    int8_t facing[3];
    uint8_t color, behind, size;
    int8_t end;
};

void buildRunwayLights(Vector<ModelLight> &out, const RunwayEnd &end, int endIndex) {
    const Vec3 NONE = {0, 0, 0};
    // Edges and centreline down the half of the runway this end lands on,
    // so that the two ends together light it once over.
    for (float a = 0; a <= end.length / 2; a += 60) {
        Rgb c = a > end.length - 600 ? LAMP_YELLOW : LAMP_WHITE;
        out.push_back({end.at(a, end.halfWidth + 1) + Vec3{0, 0, 0.4f}, c, 0.5f, NONE, c});
        out.push_back({end.at(a, -end.halfWidth - 1) + Vec3{0, 0, 0.4f}, c, 0.5f, NONE, c});
    }
    for (float a = end.length - 15; a > end.length / 2; a -= 30) {
        float left = end.length - a;
        Rgb c = left < 300 ? LAMP_RED : left < 900 && (int)(a / 30) % 2 ? LAMP_RED : LAMP_WHITE;
        out.push_back({end.at(a, 0) + Vec3{0, 0, 0.1f}, c, 0.35f, NONE, c});
    }
    for (float r = -end.halfWidth; r <= end.halfWidth; r += 3) {
        out.push_back({end.at(-1, r) + Vec3{0, 0, 0.3f}, LAMP_GREEN, 0.45f, end.along * -1, LAMP_RED});
    }
    // The approach lights - on only for the end being landed on: see
    // ModelLight::end.
    for (float along = -30; along >= -900; along -= 30) {
        float height = 0.6f - along * 0.004f;
        int across = along == -300 ? 7 : 2;
        for (int i = -across; i <= across; i++) {
            ModelLight l = {end.at(along, i * (along == -300 ? 2.1f : 1.05f)) + Vec3{0, 0, height + 0.15f}, LAMP_WHITE,
                            0.5f, NONE, LAMP_WHITE};
            l.end = endIndex;
            out.push_back(l);
        }
    }
}

// The PAPI beside each runway end at its aiming point, on the left: four
// lights, each white seen from above its angle and red from below - the one
// nearest the runway set at 3.5 degrees, the furthest at 2.5 - so that on
// the glide path two show white and two red.
struct Papi {
    Vec3 at;
    float angle;
    Vec3 facing;  // down the approach
};

// Towns about the airfield by night - a light here and there, gathered in
// places - and the stars over them. Made up, but the same every time.
uint32_t g_seed = 12345;
float random01() {
    g_seed = g_seed * 1664525 + 1013904223;
    return (g_seed >> 8) / 16777216.0f;
}

void buildTownLights(Vector<ModelLight> &out, Vec3 middle) {
    const Rgb WARM[] = {{255, 196, 120}, {255, 226, 180}, {255, 170, 90}, {210, 225, 255}};
    g_seed = 12345;
    for (int town = 0; town < 14; town++) {
        float bearing = random01() * 2 * PI, distance = 4500 + random01() * 9000;
        Vec3 centre = middle + Vec3{cosf(bearing), sinf(bearing), 0} * distance;
        int lights = 20 + (int)(random01() * 50);
        float spread = 300 + random01() * 700;
        for (int i = 0; i < lights; i++) {
            float a = random01() * 2 * PI, r = spread * sqrtf(random01());
            out.push_back({centre + Vec3{r * cosf(a), r * sinf(a), 2}, WARM[i % 4], 4, {0, 0, 0}, WARM[i % 4]});
        }
    }
}

struct Star {
    Vec3 direction;
    float brightness;
};

void buildStars(Vector<Star> &out) {
    for (int i = 0; i < 260; i++) {
        float bearing = random01() * 2 * PI, elevation = asinf(0.04f + 0.96f * random01());
        out.push_back({{cosf(elevation) * cosf(bearing), cosf(elevation) * sinf(bearing), sinf(elevation)},
                       0.25f + 0.75f * random01() * random01()});
    }
}

// A marking on the ground too thin to draw as a strip far off - a taxiway's
// centreline, a runway's edge: its middle from a to b, how wide it is, and
// its paint.
struct PaintLine {
    Vec3 a, b;
    float width;
    Rgb color;
    // A short one - a dash of a holding point's line - drawn only near: from
    // further off they are under a pixel each, and a big airport has
    // thousands.
    bool near = false;
};
// The paint's runs, in its cells' `lod`: see buildAirfield().
constexpr uint8_t PAINT_BROAD = 0, PAINT_THIN = 1, PAINT_DASHES = 2;
// The airfield lights' cells: the taxiways' marked so.
constexpr uint8_t LIGHTS_TAXIWAY = 1;

// A run of a list whose items are near one another, and a ball round them
// all: for a frame to pass over a whole run out of view at once, rather than
// look at each of its items - most of an airfield's thousands, or of the
// country's coasts, being out of view in any one.
struct Cell {
    Vec3 middle;
    float radius;
    uint32_t begin, end;
    // Drawn at any distance (LOD_ANY), only near (LOD_NEAR) or only far
    // (LOD_FAR): two versions of the same ground, fine and coarse; or only
    // where there are coasts about (LOD_GRASS): the grass under an airfield,
    // which otherwise is the ground's own colour.
    uint8_t lod = 0;
};
constexpr uint8_t LOD_ANY = 0, LOD_NEAR = 1, LOD_FAR = 2, LOD_GRASS = 3;
// Where the ground's coarse version takes over from the fine, metres from the
// eye - both drawn for this much either side, so there is never a gap.
constexpr float COARSE_FROM = 2000, COARSE_OVERLAP = 250;

// Items [from, to) of a list put in order by the square of a grid, `size`
// metres across, that each one's middle is in - and in each square in the
// order they were, as the ground's must stay, being painted in order - and a
// Cell for each square's run added to `cells`. bounds(item, lo, hi) widens
// lo and hi to take in the item.
template <typename T, typename Bounds>
void intoCells(Vector<T> &items, size_t from, size_t to, float size, Vector<Cell> &cells, Bounds bounds) {
    struct Keyed {
        int64_t key;
        uint32_t index;
    };
    Vector<Keyed> keyed;
    keyed.reserve(to - from);
    for (size_t i = from; i < to; i++) {
        Vec3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
        bounds(items[i], lo, hi);
        int32_t cx = (int32_t)floorf((lo.x + hi.x) / 2 / size), cy = (int32_t)floorf((lo.y + hi.y) / 2 / size);
        keyed.push_back({(int64_t)cy << 32 | (uint32_t)cx, (uint32_t)i});
    }
    std::stable_sort(keyed.begin(), keyed.end(), [](const Keyed &a, const Keyed &b) { return a.key < b.key; });
    Vector<T> sorted;
    sorted.reserve(to - from);
    for (const Keyed &k : keyed) {
        sorted.push_back(items[k.index]);
    }
    std::copy(sorted.begin(), sorted.end(), items.begin() + from);
    for (size_t i = 0; i < keyed.size();) {
        size_t j = i;
        Vec3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
        for (; j < keyed.size() && keyed[j].key == keyed[i].key; j++) {
            bounds(items[from + j], lo, hi);
        }
        Vec3 middle = (lo + hi) * 0.5f;
        cells.push_back({middle, length(hi - middle), (uint32_t)(from + i), (uint32_t)(from + j)});
        i = j;
    }
    cells.shrink_to_fit();
}

inline void widen(Vec3 p, float reach, Vec3 &lo, Vec3 &hi) {
    lo = {std::min(lo.x, p.x - reach), std::min(lo.y, p.y - reach), std::min(lo.z, p.z - reach)};
    hi = {std::max(hi.x, p.x + reach), std::max(hi.y, p.y + reach), std::max(hi.z, p.z + reach)};
}

constexpr float AIRFIELD_CELL = 200, COUNTRY_CELL = 20000;  // metres
// High up, how far off the country's lines and rivers are drawn - and the
// land told from the sea and the towns by them: see groundAndCoasts().
constexpr float LINES_REACH = 250000;

// A stand: its stop - where the nose wheel stops, as OpenStreetMap maps them
// - the way an aircraft there faces, and how far on from the stop it is to
// the building ahead. One drawn as its outline: its middle, `middle` set.
struct StandStop {
    Vec3 at;
    float heading;
    float room;
    bool middle;
};

// A building's box: the extent of its footprint east to west and south to
// north, and how high its roof is.
struct Block {
    float x0, y0, x1, y1, top;
};

// The box about a building orientedBox() made.
Block blockOf(Vec3 centre, Vec3 axis, float halfLength, float halfWidth, float height) {
    float dx = fabsf(axis.x) * halfLength + fabsf(axis.y) * halfWidth,
          dy = fabsf(axis.y) * halfLength + fabsf(axis.x) * halfWidth;
    return {centre.x - dx, centre.y - dy, centre.x + dx, centre.y + dy, height};
}


struct Models {
    Vector<ModelTri> ground, solids;
    // Small buildings: drawn only near - see prepareEnd().
    Vector<ModelTri> detail;
    Vector<PaintLine> paintLines;
    Vector<ModelLine> wires;
    Vector<ModelLight> airfieldLights, townLights;
    // Those in cells, each list in its cells' order: the ground's a layer at
    // a time - see buildAirfield() - so its cells, taken in order, paint each
    // layer over the one before.
    Vector<Cell> groundCells, paintCells, solidCells, wireCells, lightCells, townCells, detailCells;
    // The solids' and the detail's facing - each triangle's unit normal, in
    // their cells' order - worked out once, not every frame.
    Vector<Vec3> solidNormals, detailNormals;
    // The ground, the solids and the detail as a frame reads them, in the
    // same order - see GroundTri - made from those once their cells are,
    // which then go; and the grid's unit, in metres.
    Vector<GroundTri> groundTris;
    Vector<SolidTri> solidTris, detailTris;
    float unit = 1.0f / 64;
    // The airfield's lights the same - made from airfieldLights, which go -
    // and their colours.
    Vector<LightPoint> lightPoints;
    Vector<Rgb> lightColors;
    Vector<float> lightSizes;
    // High up: lines on the ground, and airfields far off as runways and
    // their edge lights.
    Vector<GroundLine> groundLines;
    Vector<Cell> lineCells;
    bool towns = false;  // whether any of groundLines are a town's
    Vector<PaintLine> rivers;
    Vector<Cell> riverCells;
    Vector<ModelTri> farGround;
    Vector<ModelLight> farLights;
    Vector<Star> stars;
    Vector<Papi> papis;
    Vector<RunwayEnd> ends;  // two to a runway, in its order
    Vector<StandStop> stands;
    // The buildings as boxes: for what a camera on the ground would see past
    // them - see seenPast().
    Vector<Block> blocks;
    Vec3 middle{}, tower{};
    float towerEyeZ = 33;  // the tower's cab, how high: where a camera in it looks out from
    // And how far over that its camera is raised, for the buildings between
    // not to hide the runways from it - see liftTower().
    float towerLift = 0;
    bool built = false;
};

Models g_models;

// Which runway end the landing is onto, and which the departures leave
// from: an index into g_models.ends, two to a runway; -1 for none.
int g_landingEnd = 0, g_departureEnd = -1;

// The airfield from its runways - see setAirfield().
// A real layout's point, on the ground.
Vec3 at(const airfield::Point &p) { return {p.x, p.y, 0}; }

// A way's centreline, junction to junction.
void lineOf(const airfield::Layout &l, const airfield::Way &w, Vector<Vec3> &out) {
    out.clear();
    out.push_back(at(l.junctions[w.from]));
    for (const airfield::Point &p : w.between) {
        out.push_back(at(p));
    }
    out.push_back(at(l.junctions[w.to]));
}

// `line` with the points left out that are within `tolerance` of the line
// between those kept either side - Douglas-Peucker - into `out`: a taxiway's
// pavement, 20m wide, needs its bends to a metre or two, not as finely as its
// centreline paint is drawn.
void simplified(const Vector<Vec3> &line, float tolerance, Vector<Vec3> &out) {
    out.clear();
    if (line.size() <= 2) {
        out = line;
        return;
    }
    static Vector<uint8_t> keep;
    static Vector<std::pair<uint16_t, uint16_t>> spans;
    keep.assign(line.size(), 0);
    keep.front() = keep.back() = 1;
    spans.clear();
    spans.push_back({0, (uint16_t)(line.size() - 1)});
    while (!spans.empty()) {
        std::pair<uint16_t, uint16_t> span = spans.back();
        spans.pop_back();
        Vec3 a = line[span.first], b = line[span.second];
        float dx = b.x - a.x, dy = b.y - a.y, len = sqrtf(dx * dx + dy * dy);
        float worst = tolerance;
        uint16_t at = 0;
        for (uint16_t i = span.first + 1; i < span.second; i++) {
            float off = len < 1e-3f ? length(line[i] - a) : fabsf((line[i].x - a.x) * dy - (line[i].y - a.y) * dx) / len;
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
    for (size_t i = 0; i < line.size(); i++) {
        if (keep[i]) {
            out.push_back(line[i]);
        }
    }
}

// The taxiways' centreline lights, green: this far apart on the straight, and
// none this near the end of a run of taxiway that meets others - see
// taxiwayLine().
constexpr float TAXI_LIGHTS_EVERY = 30, TAXI_LIGHTS_CLEAR = 15;

// Whether `p` is on one of the runways, or within 5m of one's edge.
bool onRunway(Vec3 p, const RunwaySpec *runways, int count) {
    for (int i = 0; i < count; i++) {
        Vec3 d = runways[i].end[1] - runways[i].end[0];
        float len = length(d);
        if (len < 1) {
            continue;
        }
        Vec3 u = d * (1 / len), q = p - runways[i].end[0];
        float a = dot(q, u);
        if (a > -30 && a < len + 30 && fabsf(dot(q, leftOf(u))) < runways[i].width / 2 + 5) {
            return true;
        }
    }
    return false;
}

// Lights down the middle of a taxiway along `line`, from `from` metres after
// its start to `to` metres before its end, into `out`: evenly, about
// TAXI_LIGHTS_EVERY apart on the straight and half that round a bend; just
// one in its middle if it is too short for both ends' room; none on a
// runway.
void taxiwayLine(const Vector<Vec3> &line, float from, float to, const RunwaySpec *runways, int count,
                 Vector<ModelLight> &out) {
    if (line.size() < 2) {
        return;
    }
    Vector<float> along(line.size(), 0);
    for (size_t i = 1; i < line.size(); i++) {
        along[i] = along[i - 1] + length(line[i] - line[i - 1]);
    }
    float total = along.back();
    if (total < 10) {
        return;
    }
    // Every STEP along it: where, and the way it runs.
    constexpr float STEP = 2;
    int n = (int)ceilf(total / STEP);
    Vector<Vec3> at(n + 1), way(n + 1);
    for (int k = 0, i = 0; k <= n; k++) {
        float s = std::min(total, k * STEP);
        while (i + 2 < (int)line.size() && along[i + 1] < s) {
            i++;
        }
        float len = along[i + 1] - along[i];
        at[k] = line[i] + (line[i + 1] - line[i]) * (len > 1e-3f ? (s - along[i]) / len : 0);
        way[k] = len > 1e-3f ? (line[i + 1] - line[i]) * (1 / len) : k > 0 ? way[k - 1] : Vec3{1, 0, 0};
    }
    // How far along it by a measure that counts a metre of bend - where it
    // turns by more than 10 degrees in 20m - as two: lights evenly spaced by
    // that are twice as close round its bends.
    Vector<float> measure(n + 1, 0);
    auto bend = [&](int k) {
        return dot(way[std::max(0, k - 5)], way[std::min(n, k + 5)]) < 0.985f ? 2.0f : 1.0f;
    };
    for (int k = 1; k <= n; k++) {
        float s = std::min(total, k * STEP), was = (k - 1) * STEP;
        measure[k] = measure[k - 1] + (s - was) * (bend(k - 1) + bend(k)) / 2;
    }
    auto measured = [&](float s) {
        int k = std::max(0, std::min(n - 1, (int)(s / STEP)));
        float span = std::min(total, (k + 1) * STEP) - k * STEP;
        return measure[k] + (measure[k + 1] - measure[k]) * (span > 1e-3f ? (s - k * STEP) / span : 0);
    };
    auto add = [&](float s) {
        int k = std::max(0, std::min(n - 1, (int)(s / STEP)));
        float span = std::min(total, (k + 1) * STEP) - k * STEP;
        Vec3 p = at[k] + (at[k + 1] - at[k]) * (span > 1e-3f ? (s - k * STEP) / span : 0);
        if (!onRunway(p, runways, count)) {
            out.push_back({p + Vec3{0, 0, 0.1f}, LAMP_GREEN, 0.3f, {0, 0, 0}, LAMP_GREEN});
        }
    };
    if (total - to - from < 1) {
        add(total / 2);
        return;
    }
    float m0 = measured(from), m1 = measured(total - to);
    int lights = std::max(1, (int)lrintf((m1 - m0) / TAXI_LIGHTS_EVERY));
    for (int j = 0, k = 0; j <= lights; j++) {
        float want = m0 + (m1 - m0) * j / lights;
        while (k + 1 < n && measure[k + 1] < want) {
            k++;
        }
        float span = measure[k + 1] - measure[k];
        float s = k * STEP + (std::min(total, (k + 1) * STEP) - k * STEP) * (span > 1e-6f ? (want - measure[k]) / span : 0);
        add(std::max(0.0f, std::min(total, s)));
    }
}

// Where along runway `r` the real taxiways leave it - each junction of one
// that lies on the runway's pavement, give or take - and the side, left of
// its first end looking down it (1) or right (-1), most of them go to.
void exitsOf(const airfield::Layout &l, const RunwaySpec &r, Vector<float> &along, float &side) {
    Vec3 d = r.end[1] - r.end[0];
    float len = length(d), half = r.width / 2;
    Vec3 u = d * (1 / len), left = leftOf(u);
    float lean = 0;
    for (const airfield::Way &w : l.ways) {
        if (w.kind != airfield::WayKind::TAXIWAY) {
            continue;
        }
        for (int e = 0; e < 2; e++) {
            Vec3 j = at(l.junctions[e == 0 ? w.from : w.to]), other = at(l.junctions[e == 0 ? w.to : w.from]);
            float a = dot(j - r.end[0], u), off = dot(j - r.end[0], left);
            if (a < -60 || a > len + 60 || fabsf(off) > half + 30) {
                continue;
            }
            along.push_back(a);
            lean += dot(other - j, left) >= 0 ? 1 : -1;
        }
    }
    if (lean != 0) {
        side = lean > 0 ? 1.0f : -1.0f;
    }
}

// The real layout: aprons, then taxiways, painted in that order; their
// centrelines, holding points and stands' lines in yellow; the buildings,
// to their heights; stands and the tower where they are; and the taxiways'
// lights, green down their middles.
void buildLayout(Models &m, const airfield::Layout &l, const RunwaySpec *runways, int count, Vector<ModelTri> &apron,
                 Vector<ModelTri> &taxiways, Vector<ModelTri> &taxiwaysFar, Vector<ModelTri> &paint,
                 Vector<ModelLight> &taxiwayLights) {
    for (const airfield::Area &a : l.surfaces) {
        Rgb color = a.kind == 1 ? CONCRETE : TAXIWAY;
        for (size_t t = 0; t + 2 < a.triangles.size(); t += 3) {
            apron.push_back({{at(a.corners[a.triangles[t]]), at(a.corners[a.triangles[t + 1]]),
                              at(a.corners[a.triangles[t + 2]])},
                             color});
        }
    }
    // The taxiways as runs, each joined end to end through the junctions
    // where it goes straight on - near enough as wide - its branches runs of
    // their own; with the junctions each ends at, and how many ways of any
    // kind meet at each junction. Far off, their pavement coarser: along
    // each run's line to within 5m - past COARSE_FROM that is a pixel or two,
    // and a big airport's pavement far fewer triangles.
    struct TaxiRun {
        Vector<Vec3> line;
        uint16_t ends[2];
    };
    Vector<TaxiRun> runs;
    Vector<uint8_t> meeting(l.junctions.size(), 0);
    {
        Vector<Vector<uint16_t>> meets(l.junctions.size());
        for (size_t i = 0; i < l.ways.size(); i++) {
            const airfield::Way &w = l.ways[i];
            meeting[w.from] = (uint8_t)std::min(255, meeting[w.from] + 1);
            if (w.to != w.from) {
                meeting[w.to] = (uint8_t)std::min(255, meeting[w.to] + 1);
            }
            if (w.kind == airfield::WayKind::TAXIWAY) {
                meets[w.from].push_back((uint16_t)i);
                if (w.to != w.from) {
                    meets[w.to].push_back((uint16_t)i);
                }
            }
        }
        Vector<uint8_t> used(l.ways.size(), 0);
        Vector<Vec3> chain, piece, far;
        // On from the end of `chain` - at junction `j`, along way `last` -
        // the straightest way on: where just two meet, turning by less than
        // 60 degrees; where more do, by less than 30 - the main line through
        // a junction, its branches runs of their own. Returns the junction it
        // ends at.
        auto extend = [&](uint16_t j, uint16_t last, float width) {
            for (int guard = 0; guard < 400 && chain.size() >= 2; guard++) {
                Vec3 in = chain.back() - chain[chain.size() - 2];
                float li = length(in);
                if (li < 0.01f) {
                    return j;
                }
                int best = -1;
                float straightest = meets[j].size() == 2 ? 0.5f : 0.866f;
                for (uint16_t k : meets[j]) {
                    const airfield::Way &w = l.ways[k];
                    if (k == last || used[k] || fabsf(w.width - width) > width * 0.3f) {
                        continue;
                    }
                    lineOf(l, w, piece);
                    if (w.to == j && w.from != j) {
                        std::reverse(piece.begin(), piece.end());
                    }
                    Vec3 out = piece[1] - piece[0];
                    float lo = length(out);
                    float c = lo < 0.01f ? -1 : dot(in, out) / (li * lo);
                    if (c > straightest) {
                        straightest = c;
                        best = k;
                    }
                }
                if (best < 0) {
                    return j;
                }
                const airfield::Way &w = l.ways[best];
                lineOf(l, w, piece);
                if (w.to == j && w.from != j) {
                    std::reverse(piece.begin(), piece.end());
                }
                used[best] = 1;
                chain.insert(chain.end(), piece.begin() + 1, piece.end());
                last = (uint16_t)best;
                j = w.to == j ? w.from : w.to;
            }
            return j;
        };
        for (size_t i = 0; i < l.ways.size(); i++) {
            const airfield::Way &w = l.ways[i];
            if (used[i] || w.kind != airfield::WayKind::TAXIWAY) {
                continue;
            }
            used[i] = 1;
            lineOf(l, w, chain);
            uint16_t end = extend(w.to, (uint16_t)i, w.width);
            std::reverse(chain.begin(), chain.end());
            uint16_t start = extend(w.from, (uint16_t)i, w.width);
            float half = std::max(4.0f, w.width / 2);
            simplified(chain, 5, far);
            for (size_t p = 0; p + 1 < far.size(); p++) {
                Vec3 a = far[p], b = far[p + 1];
                float len = length(b - a);
                if (len >= 0.01f) {
                    Vec3 dir = (b - a) * (1 / len);
                    strip(taxiwaysFar, a - dir * half, b + dir * half, half, TAXIWAY);
                }
            }
            runs.push_back({chain, {end, start}});
        }
    }
    Vector<Vec3> line, coarse;
    for (const airfield::Way &w : l.ways) {
        if (w.kind == airfield::WayKind::RUNWAY) {
            continue;  // the runway itself is drawn from the runway table
        }
        lineOf(l, w, line);
        float half = std::max(4.0f, w.width / 2);
        // A taxiway's pavement - each piece run on half its width past its
        // ends, so a bend has no notch - along its line to within a tenth of
        // its width: a third of the triangles at a big airport. A taxilane
        // is paint on an apron, no pavement of its own.
        if (w.kind == airfield::WayKind::TAXIWAY) {
            simplified(line, std::max(0.8f, half * 0.2f), coarse);
            for (size_t i = 0; i + 1 < coarse.size(); i++) {
                Vec3 a = coarse[i], b = coarse[i + 1];
                float len = length(b - a);
                if (len >= 0.01f) {
                    Vec3 dir = (b - a) * (1 / len);
                    strip(taxiways, a - dir * half, b + dir * half, half, TAXIWAY);
                }
            }
        }
        for (size_t i = 0; i + 1 < line.size(); i++) {
            Vec3 a = line[i], b = line[i + 1];
            if (length(b - a) >= 0.01f) {
                strip(paint, a, b, 0.15f, YELLOW);
            }
        }
    }
    // The lights: green down the middle of each run of taxiway, as a big
    // airport has them - and no blue along its edges as well, which made
    // thousands a scatter - evenly, every 30m or so on the straight and
    // twice as often round a bend; none within 15m of a junction its run
    // ends at, where the way on has its own, nor on a runway, which has its
    // own.
    for (const TaxiRun &r : runs) {
        taxiwayLine(r.line, meeting[r.ends[0]] > 1 ? TAXI_LIGHTS_CLEAR : 0,
                    meeting[r.ends[1]] > 1 ? TAXI_LIGHTS_CLEAR : 0, runways, count, taxiwayLights);
    }
    // Holding points: across the way, at the point - on a runway's, two solid
    // lines on the taxiway's side and two dashed on the runway's; elsewhere
    // one dashed line.
    auto toRunway = [&](Vec3 p) {
        float best = 1e9f;
        for (int i = 0; i < count; i++) {
            Vec3 d = runways[i].end[1] - runways[i].end[0];
            float t = clamp01(dot(p - runways[i].end[0], d) / std::max(1.0f, dot(d, d)));
            best = std::min(best, length(p - (runways[i].end[0] + d * t)));
        }
        return best;
    };
    for (const airfield::Hold &h : l.holds) {
        const airfield::Way &w = l.ways[h.way];
        lineOf(l, w, line);
        Vec3 p = at(h.at), dir = {1, 0, 0};
        float nearest = 1e9f;
        for (size_t i = 0; i + 1 < line.size(); i++) {
            Vec3 a = line[i], b = line[i + 1], d = b - a;
            float t = clamp01(dot(p - a, d) / std::max(0.01f, dot(d, d))), gap = length(p - (a + d * t));
            if (gap < nearest && length(d) > 0.01f) {
                nearest = gap;
                dir = normalize(d);
            }
        }
        if (toRunway(p + dir * 30) > toRunway(p - dir * 30)) {
            dir = dir * -1;  // dir towards the runway
        }
        Vec3 across = leftOf(dir);
        float half = std::max(4.0f, w.width / 2);
        int bars = h.kind == airfield::HoldKind::RUNWAY ? 4 : 1;
        for (int bar = 0; bar < bars; bar++) {
            Vec3 c = p + dir * (bar * 0.9f - (bars - 1) * 0.45f);
            bool dashed = bars == 1 || bar >= 2;
            if (!dashed) {
                strip(paint, c - across * half, c + across * half, 0.15f, YELLOW);
            } else {
                for (float s = -half; s < half; s += 2.4f) {
                    strip(paint, c + across * s, c + across * std::min(half, s + 1.2f), 0.15f, YELLOW);
                }
            }
        }
    }
    // Stands: the lines in, and where an aircraft parks - its nose wheel
    // where the line stops, facing the way it comes in; with no line, its
    // nose to the nearest building; one drawn as its outline, the middle of
    // the aircraft on the middle of it.
    for (const airfield::Stand &st : l.stands) {
        for (size_t i = 0; i + 1 < st.lead.size(); i++) {
            strip(paint, at(st.lead[i]), at(st.lead[i + 1]), 0.15f, YELLOW);
        }
        bool outline = st.lead.size() >= 4 && length(at(st.lead.front()) - at(st.lead.back())) < 5;
        Vec3 p = at(st.at);
        if (outline) {
            p = {0, 0, 0};
            for (size_t i = 0; i + 1 < st.lead.size(); i++) {
                p = p + at(st.lead[i]) * (1.0f / (st.lead.size() - 1));
            }
        }
        float heading = 0;
        if (st.lead.size() >= 2 && !outline) {
            Vec3 d = at(st.lead.back()) - at(st.lead[st.lead.size() - 2]);
            heading = atan2f(d.y, d.x);
        } else {
            float best = 1e9f;
            for (const airfield::Area &b : l.buildings) {
                if (b.corners.empty()) {
                    continue;
                }
                Vec3 c = at(b.corners[0]);
                if (length(c - p) < best) {
                    best = length(c - p);
                    heading = atan2f(c.y - p.y, c.x - p.x);
                }
            }
        }
        m.stands.push_back({p, heading, l.roomAhead({p.x, p.y}, heading, 80), outline});
    }
    // The buildings: the roof's triangles at its height, and a wall up each
    // edge round it - each edge only one triangle has - facing out. The tower
    // the one marked so - its cab looked out of from just below its top - or
    // where the layout says it is.
    bool towerBuilt = false;
    float towerHeight = 0;
    for (const airfield::Area &b : l.buildings) {
        if (!b.corners.empty()) {
            Block box = {b.corners[0].x, b.corners[0].y, b.corners[0].x, b.corners[0].y, std::max(3.0f, b.height)};
            for (const airfield::Point &c : b.corners) {
                box = {std::min(box.x0, c.x), std::min(box.y0, c.y), std::max(box.x1, c.x), std::max(box.y1, c.y),
                       box.top};
            }
            m.blocks.push_back(box);
        }
        // A small one - not the airport's own - drawn only near.
        float area = 0;
        for (size_t t = 0; t + 2 < b.triangles.size(); t += 3) {
            Vec3 p0 = at(b.corners[b.triangles[t]]), p1 = at(b.corners[b.triangles[t + 1]]),
                 p2 = at(b.corners[b.triangles[t + 2]]);
            area += fabsf(cross(p1 - p0, p2 - p0).z) / 2;
        }
        Vector<ModelTri> &into = b.kind == 4 && area < 600 ? m.detail : m.solids;
        Rgb wall = b.kind == 1 ? TERMINAL : b.kind == 2 ? Rgb{176, 182, 188} : b.kind == 3 ? Rgb{226, 226, 222}
                                                                                           : Rgb{190, 180, 166};
        Vec3 up = {0, 0, std::max(3.0f, b.height)};
        struct Edge {
            uint16_t a, b;
        };
        Vector<Edge> edges;
        for (size_t t = 0; t + 2 < b.triangles.size(); t += 3) {
            uint16_t i[3] = {b.triangles[t], b.triangles[t + 1], b.triangles[t + 2]};
            Vec3 p0 = at(b.corners[i[0]]), p1 = at(b.corners[i[1]]), p2 = at(b.corners[i[2]]);
            if (cross(p1 - p0, p2 - p0).z < 0) {
                std::swap(i[1], i[2]);  // anticlockwise from above: the roof facing up
                std::swap(p1, p2);
            }
            into.push_back({{p0 + up, p1 + up, p2 + up}, ROOF});
            for (int k = 0; k < 3; k++) {
                edges.push_back({i[k], i[(k + 1) % 3]});
            }
        }
        // An edge two triangles share comes once each way: sorted by its two
        // corners, those pairs sit together.
        auto key = [](const Edge &e) { return (uint32_t)std::min(e.a, e.b) << 16 | std::max(e.a, e.b); };
        std::sort(edges.begin(), edges.end(), [&](const Edge &x, const Edge &y) { return key(x) < key(y); });
        for (size_t i = 0; i < edges.size(); i++) {
            if ((i > 0 && key(edges[i - 1]) == key(edges[i])) ||
                (i + 1 < edges.size() && key(edges[i + 1]) == key(edges[i]))) {
                continue;
            }
            Vec3 a = at(b.corners[edges[i].a]), c = at(b.corners[edges[i].b]);
            rect(into, a, c, c + up, a + up, wall);
        }
        if (b.kind == 3 && !towerBuilt && !b.corners.empty()) {
            Vec3 middle{};
            for (const airfield::Point &q : b.corners) {
                middle = middle + at(q) * (1.0f / b.corners.size());
            }
            m.tower = middle;
            towerBuilt = true;
            towerHeight = b.height;
        }
    }
    if (!towerBuilt && l.hasTower) {
        m.tower = at(l.tower);
        towerBuilt = true;
        towerHeight = l.towerHeight;
    }
    if (!towerBuilt) {
        m.tower = m.middle;
    }
    // A control tower stands 30m and more; one mapped lower - Glasgow's as
    // its two-storey base - is taken for the foot of one, the tower put up
    // on it, and the cab looked out of from 30m.
    if (towerHeight < 20) {
        buildTower(m.wires, m.tower + Vec3{0, 0, towerHeight});
        m.towerEyeZ = 33;
    } else {
        m.towerEyeZ = towerHeight - 3;
    }
}

// The ground, the solids and the detail made into what a frame reads - see
// GroundTri - and the ModelTris they were made from let go.
void compact(Models &m) {
    float most = 1;
    for (const ModelTri &t : m.ground) {
        for (const Vec3 &v : t.v) {
            most = std::max({most, fabsf(v.x), fabsf(v.y)});
        }
    }
    for (const Vector<ModelTri> *tris : {&m.solids, &m.detail}) {
        for (const ModelTri &t : *tris) {
            for (const Vec3 &v : t.v) {
                most = std::max({most, fabsf(v.x), fabsf(v.y), fabsf(v.z)});
            }
        }
    }
    for (const ModelLight &l : m.airfieldLights) {
        most = std::max({most, fabsf(l.at.x), fabsf(l.at.y), fabsf(l.at.z)});
    }
    m.unit = std::max(1.0f / 256, most / 32000);
    auto at = [&](float v) { return (int16_t)lrintf(v / m.unit); };
    auto level = [](float c) { return (uint8_t)lrintf(std::min(255.0f, std::max(0.0f, c))); };
    m.groundTris.resize(m.ground.size());
    for (size_t i = 0; i < m.ground.size(); i++) {
        const ModelTri &t = m.ground[i];
        GroundTri &o = m.groundTris[i];
        for (int k = 0; k < 3; k++) {
            o.x[k] = at(t.v[k].x);
            o.y[k] = at(t.v[k].y);
        }
        o.rgb[0] = level(t.color.r);
        o.rgb[1] = level(t.color.g);
        o.rgb[2] = level(t.color.b);
        o.grass = t.paint == PAINT_GRASS;
        o.fine = t.fine;
    }
    auto solids = [&](const Vector<ModelTri> &tris, const Vector<Vec3> &normals, Vector<SolidTri> &out) {
        out.resize(tris.size());
        for (size_t i = 0; i < tris.size(); i++) {
            const ModelTri &t = tris[i];
            SolidTri &o = out[i];
            for (int k = 0; k < 3; k++) {
                o.v[k][0] = at(t.v[k].x);
                o.v[k][1] = at(t.v[k].y);
                o.v[k][2] = at(t.v[k].z);
            }
            o.n[0] = (int16_t)lrintf(normals[i].x * 32767);
            o.n[1] = (int16_t)lrintf(normals[i].y * 32767);
            o.n[2] = (int16_t)lrintf(normals[i].z * 32767);
            o.rgb[0] = level(t.color.r);
            o.rgb[1] = level(t.color.g);
            o.rgb[2] = level(t.color.b);
            o.spare = 0;
        }
    };
    solids(m.solids, m.solidNormals, m.solidTris);
    solids(m.detail, m.detailNormals, m.detailTris);
    m.lightColors.clear();
    auto colorIndex = [&](Rgb c) {
        for (size_t i = 0; i < m.lightColors.size(); i++) {
            if (m.lightColors[i].r == c.r && m.lightColors[i].g == c.g && m.lightColors[i].b == c.b) {
                return (uint8_t)i;
            }
        }
        if (m.lightColors.size() >= 255) {
            return (uint8_t)0;
        }
        m.lightColors.push_back(c);
        return (uint8_t)(m.lightColors.size() - 1);
    };
    m.lightSizes.clear();
    auto sizeIndex = [&](float size) {
        for (size_t i = 0; i < m.lightSizes.size(); i++) {
            if (m.lightSizes[i] == size) {
                return (uint8_t)i;
            }
        }
        if (m.lightSizes.size() >= 255) {
            return (uint8_t)0;
        }
        m.lightSizes.push_back(size);
        return (uint8_t)(m.lightSizes.size() - 1);
    };
    m.lightPoints.resize(m.airfieldLights.size());
    for (size_t i = 0; i < m.airfieldLights.size(); i++) {
        const ModelLight &l = m.airfieldLights[i];
        LightPoint &o = m.lightPoints[i];
        o.x = at(l.at.x);
        o.y = at(l.at.y);
        o.z = at(l.at.z);
        o.facing[0] = (int8_t)lrintf(l.facing.x * 127);
        o.facing[1] = (int8_t)lrintf(l.facing.y * 127);
        o.facing[2] = (int8_t)lrintf(l.facing.z * 127);
        o.color = colorIndex(l.color);
        o.behind = colorIndex(l.behind);
        o.size = sizeIndex(l.size);
        o.end = (int8_t)l.end;
    }
    m.airfieldLights.clear();
    m.airfieldLights.shrink_to_fit();
    m.lightPoints.shrink_to_fit();
    for (Vector<ModelTri> *tris : {&m.ground, &m.solids, &m.detail}) {
        tris->clear();
        tris->shrink_to_fit();
    }
    for (Vector<Vec3> *normals : {&m.solidNormals, &m.detailNormals}) {
        normals->clear();
        normals->shrink_to_fit();
    }
    m.groundTris.shrink_to_fit();
    m.solidTris.shrink_to_fit();
    m.detailTris.shrink_to_fit();
}

// Grass over all of an airfield where no water is near, under its paving:
// the outline round everything of it - a convex hull - as a fan of a few
// triangles. A town's colour isn't drawn across the airfield it is round, and
// a strip under each taxiway, as by the sea, was thousands of triangles
// (Heathrow's ground took 10ms a frame).
void grassHull(Vector<ModelTri> &grass, std::initializer_list<const Vector<ModelTri> *> layers) {
    using P = std::pair<float, float>;
    static Vector<P> points, hull;
    points.clear();
    for (const Vector<ModelTri> *layer : layers) {
        for (const ModelTri &t : *layer) {
            for (const Vec3 &v : t.v) {
                points.push_back({v.x, v.y});
            }
        }
    }
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() < 3) {
        return;
    }
    // Andrew's monotone chain: the lower side left to right, the upper back.
    auto turn = [](P o, P a, P b) {
        return (a.first - o.first) * (b.second - o.second) - (a.second - o.second) * (b.first - o.first);
    };
    hull.assign(2 * points.size(), P{});
    size_t k = 0;
    for (size_t i = 0; i < points.size(); i++) {
        while (k >= 2 && turn(hull[k - 2], hull[k - 1], points[i]) <= 0) {
            k--;
        }
        hull[k++] = points[i];
    }
    for (size_t i = points.size() - 1, low = k + 1; i-- > 0;) {
        while (k >= low && turn(hull[k - 2], hull[k - 1], points[i]) <= 0) {
            k--;
        }
        hull[k++] = points[i];
    }
    hull.resize(k - 1);
    auto at = [](P p) { return Vec3{p.first, p.second, 0}; };
    for (size_t i = 1; i + 1 < hull.size(); i++) {
        grass.push_back({{at(hull[0]), at(hull[i]), at(hull[i + 1])}, GRASS});
    }
}

// The gaps in an airfield's ground narrower than twice `reach`, between its
// grass and its paving, made grass too: on a grid of the ground they cover,
// grown by `reach` and shrunk back - a closing - and what that adds put into
// `grass` as rectangles, a row of the grid at a time, each reaching a cell
// under what it fills between, for no seam to show. Hong Kong's airport, an
// island made of the sea, had the water show between its taxiways.
void fillBetween(Vector<ModelTri> &grass, std::initializer_list<const Vector<ModelTri> *> layers, float reach) {
    float x0 = 1e30f, y0 = 1e30f, x1 = -1e30f, y1 = -1e30f;
    for (const Vector<ModelTri> *layer : layers) {
        for (const ModelTri &t : *layer) {
            for (const Vec3 &v : t.v) {
                x0 = std::min(x0, v.x), x1 = std::max(x1, v.x), y0 = std::min(y0, v.y), y1 = std::max(y1, v.y);
            }
        }
    }
    if (x0 > x1) {
        return;
    }
    x0 -= reach, y0 -= reach, x1 += reach, y1 += reach;
    // Cells of 20m - under the strips' edges, which hide them - or as many as
    // make 200,000 of them.
    float cell = std::max(20.0f, sqrtf((x1 - x0) * (y1 - y0) / 200000));
    int w = (int)ceilf((x1 - x0) / cell), h = (int)ceilf((y1 - y0) / cell), r = (int)ceilf(reach / cell);
    Vector<uint8_t> had((size_t)w * h, 0);
    for (const Vector<ModelTri> *layer : layers) {
        for (const ModelTri &t : *layer) {
            // Each row of cells whose middle the triangle spans, from where
            // its edges cross the row.
            float lo = std::min({t.v[0].y, t.v[1].y, t.v[2].y}), hi = std::max({t.v[0].y, t.v[1].y, t.v[2].y});
            for (int j = std::max(0, (int)floorf((lo - y0) / cell)); j < std::min(h, (int)ceilf((hi - y0) / cell)); j++) {
                float y = y0 + (j + 0.5f) * cell, a = 1e30f, b = -1e30f;
                for (int e = 0; e < 3; e++) {
                    Vec3 p = t.v[e], q = t.v[(e + 1) % 3];
                    if ((p.y <= y) != (q.y <= y)) {
                        float x = p.x + (y - p.y) * (q.x - p.x) / (q.y - p.y);
                        a = std::min(a, x), b = std::max(b, x);
                    }
                }
                for (int i = std::max(0, (int)floorf((a - x0) / cell)); i < std::min(w, (int)ceilf((b - x0) / cell)); i++) {
                    had[(size_t)j * w + i] = 1;
                }
            }
        }
    }
    // Grown, then shrunk: each a square 2r + 1 across, along the rows and
    // then down the columns, by running counts.
    Vector<uint8_t> a = had, b((size_t)w * h);
    auto pass = [&](const Vector<uint8_t> &from, Vector<uint8_t> &to, bool grow) {
        Vector<uint8_t> mid((size_t)w * h);
        for (int along = 0; along < 2; along++) {
            const Vector<uint8_t> &src = along == 0 ? from : mid;
            Vector<uint8_t> &dst = along == 0 ? mid : to;
            int lines = along == 0 ? h : w, n = along == 0 ? w : h;
            for (int k = 0; k < lines; k++) {
                auto at = [&](int i) -> size_t { return along == 0 ? (size_t)k * w + i : (size_t)i * w + k; };
                int sum = 0;
                for (int i = 0; i < std::min(n, r); i++) {
                    sum += src[at(i)];
                }
                for (int i = 0; i < n; i++) {
                    if (i + r < n) {
                        sum += src[at(i + r)];
                    }
                    if (i - r - 1 >= 0) {
                        sum -= src[at(i - r - 1)];
                    }
                    int span = std::min(n - 1, i + r) - std::max(0, i - r) + 1;
                    dst[at(i)] = grow ? sum > 0 : sum == span && i - r >= 0 && i + r < n;
                }
            }
        }
    };
    pass(a, b, true);
    pass(b, a, false);
    // What closing added, and a cell round it, within the closing.
    for (size_t k = 0; k < a.size(); k++) {
        b[k] = a[k] && !had[k];
    }
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            size_t k = (size_t)j * w + i;
            if (!a[k] || b[k]) {
                continue;
            }
            bool next = (i > 0 && b[k - 1] == 1) || (i + 1 < w && b[k + 1] == 1) || (j > 0 && b[k - w] == 1) ||
                        (j + 1 < h && b[k + w] == 1);
            if (next) {
                b[k] = 2;
            }
        }
    }
    // Each row's runs, a run the same as one in the row before carried on
    // down from it rather than begun again.
    struct Run {
        int from, to, top;
    };
    static Vector<Run> open, next;
    open.clear();
    auto put = [&](const Run &run, int bottom) {
        float ya = y0 + run.top * cell, yb = y0 + bottom * cell, xa = x0 + run.from * cell, xb = x0 + run.to * cell;
        rect(grass, {xa, ya, 0}, {xb, ya, 0}, {xb, yb, 0}, {xa, yb, 0}, GRASS);
    };
    for (int j = 0; j <= h; j++) {
        next.clear();
        for (int i = 0; j < h && i < w;) {
            if (!b[(size_t)j * w + i]) {
                i++;
                continue;
            }
            int from = i;
            while (i < w && b[(size_t)j * w + i]) {
                i++;
            }
            next.push_back({from, i, j});
        }
        // Runs the same as an open one carry it on; the open ones not
        // carried on end here.
        for (Run &run : next) {
            for (Run &o : open) {
                if (o.from == run.from && o.to == run.to && o.top >= 0) {
                    run.top = o.top;
                    o.top = -1;
                    break;
                }
            }
        }
        for (const Run &o : open) {
            if (o.top >= 0) {
                put(o, j);
            }
        }
        std::swap(open, next);
    }
}

void buildAirfield(Models &m, const RunwaySpec *runways, int count, const Vec3 *beacons, int beaconCount,
                   const airfield::Layout *layout, bool madeUp) {
    // Real taxiways and the rest, from the layout, where there is one with
    // anything in it; where there isn't, the runways alone - or, `madeUp`,
    // with taxiways and a terminal made up about them.
    bool real = layout != nullptr && !layout->empty();
    madeUp = madeUp && !real;
    m.towerEyeZ = 33;
    m.detail.clear();
    m.ground.clear();
    m.solids.clear();
    m.wires.clear();
    m.airfieldLights.clear();
    m.townLights.clear();
    m.papis.clear();
    m.ends.clear();
    m.stands.clear();
    m.blocks.clear();

    // Its middle: among the runways' middles. The terminal goes there - when
    // that is clear of them; beside the longest when it isn't.
    Vec3 middle = {0, 0, 0};
    int longest = 0;
    for (int i = 0; i < count; i++) {
        middle = middle + (runways[i].end[0] + runways[i].end[1]) * (0.5f / count);
        if (length(runways[i].end[1] - runways[i].end[0]) > length(runways[longest].end[1] - runways[longest].end[0])) {
            longest = i;
        }
    }
    auto distanceToRunway = [&](Vec3 p, const RunwaySpec &r) {
        Vec3 d = r.end[1] - r.end[0];
        float t = clamp01(dot(p - r.end[0], d) / dot(d, d));
        Vec3 q = r.end[0] + d * t - p;
        return sqrtf(q.x * q.x + q.y * q.y);
    };
    float clearance = 1e9f;
    for (int i = 0; i < count; i++) {
        clearance = std::min(clearance, distanceToRunway(middle, runways[i]));
    }
    const RunwaySpec &main = runways[longest];
    Vec3 axis = normalize(main.end[1] - main.end[0]), across = leftOf(axis);
    float mainLength = length(main.end[1] - main.end[0]);
    float halfLength = std::max(300.0f, std::min(900.0f, mainLength * 0.3f));
    float halfDepth = std::max(120.0f, std::min(450.0f, clearance - TAXIWAY_OFFSET - 120));
    if (clearance < TAXIWAY_OFFSET + 240) {
        halfDepth = 160;
        middle = (main.end[0] + main.end[1]) * 0.5f + across * (TAXIWAY_OFFSET + 70 + halfDepth);
    }
    // Shrunk until no runway comes within 200m of it.
    for (int tries = 0; tries < 12; tries++) {
        bool clear = true;
        for (int i = 0; i < count && clear; i++) {
            Vec3 d = runways[i].end[1] - runways[i].end[0];
            float len = length(d);
            for (float s = 0; s <= len && clear; s += 40) {
                Vec3 p = runways[i].end[0] + d * (s / len) - middle;
                clear = fabsf(dot(p, axis)) > halfLength + 200 || fabsf(dot(p, across)) > halfDepth + 200;
            }
        }
        if (clear) {
            break;
        }
        halfLength *= 0.85f;
        halfDepth *= 0.85f;
    }
    m.middle = middle;
    // Whether the water comes near - a coast's or a lake's shore within 9km of
    // the middle, set before the airfield is. Where it does, the grass is
    // laid along each runway and taxiway and the gaps between filled in, for
    // the airfield not to stand in the sea where Natural Earth's coast runs
    // through it; inland, an outline round it all.
    bool shore = std::any_of(m.groundLines.begin(), m.groundLines.end(), [&](const GroundLine &l) {
        return (l.kind == GroundKind::COAST || l.kind == GroundKind::LAKE) &&
               (length(l.a - middle) < 9000 || length(l.b - middle) < 9000);
    });

    // The runways: their taxiways and ways on, then their asphalt, then their
    // paint - the ground is painted in that order, with no depth to it. The
    // taxiways' lights apart from the rest: see their cells below. And
    // under all of it, where there is a coast about, grass: each runway's
    // strip, 75m beyond its edges and 90m past its ends, each taxiway's
    // shoulders, 30m - for an airfield the sea's coast runs through - its
    // runways out on land made from the sea, as Hong Kong's are and Nice's,
    // or Natural Earth's coast a few hundred metres off - to stand on land,
    // not in the water.
    Vector<ModelTri> grass, taxiways, asphalt, rubberMarks, paint;
    Vector<ModelLight> taxiwayLights;
    for (int i = 0; i < count; i++) {
        const RunwaySpec &r = runways[i];
        Vec3 d = r.end[1] - r.end[0];
        float len = length(d);
        Vec3 u = d * (1 / len), left = leftOf(u);
        float half = r.width / 2;
        strip(grass, r.end[0] - u * 90, r.end[1] + u * 90, half + 75, GRASS);
        // Its taxiway on the side towards the middle.
        Vec3 mid = (r.end[0] + r.end[1]) * 0.5f;
        float side = dot(middle - mid, left) >= 0 ? 1.0f : -1.0f;
        Vector<float> connectors = {-48, len + 48};
        if (real) {
            // The real ones' ways off it, and the side most of them go.
            connectors.clear();
            exitsOf(*layout, r, connectors, side);
        } else {
            // Ways off it every 500m, for where a landing turns off - drawn
            // only on the made-up airfield, with the taxiway they join.
            for (float c = CONNECTOR_EVERY / 2; c < len - 100; c += CONNECTOR_EVERY) {
                connectors.push_back(c);
            }
        }
        if (madeUp) {
            // From the runway out past its taxiway, all grass.
            strip(grass, r.end[0] - u * 90 + left * (side * (TAXIWAY_OFFSET + 42) / 2),
                  r.end[1] + u * 90 + left * (side * (TAXIWAY_OFFSET + 42) / 2), (TAXIWAY_OFFSET + 42) / 2, GRASS);
            strip(taxiways, r.end[0] - u * 60 + left * (side * TAXIWAY_OFFSET),
                  r.end[1] + u * 60 + left * (side * TAXIWAY_OFFSET), 12, TAXIWAY);
            for (float c : connectors) {
                Vec3 base = r.end[0] + u * c;
                strip(taxiways, base + left * (side * half), base + left * (side * (TAXIWAY_OFFSET - 12)), 12, TAXIWAY);
            }
            // The taxiways' yellow centrelines, and across each way on, the
            // holding point: two solid lines on the taxiway's side, two dashed
            // on the runway's.
            strip(paint, r.end[0] - u * 60 + left * (side * TAXIWAY_OFFSET),
                  r.end[1] + u * 60 + left * (side * TAXIWAY_OFFSET), 0.15f, YELLOW);
            // And their lights, green down the middles of the taxiway and its
            // ways on.
            Vector<Vec3> lit = {r.end[0] - u * 60 + left * (side * TAXIWAY_OFFSET),
                                r.end[1] + u * 60 + left * (side * TAXIWAY_OFFSET)};
            taxiwayLine(lit, 0, 0, runways, count, taxiwayLights);
            for (float c : connectors) {
                Vec3 base = r.end[0] + u * c;
                strip(paint, base + left * (side * half), base + left * (side * TAXIWAY_OFFSET), 0.15f, YELLOW);
                lit = {base + left * (side * half), base + left * (side * TAXIWAY_OFFSET)};
                taxiwayLine(lit, 0, TAXI_LIGHTS_CLEAR, runways, count, taxiwayLights);
                for (int bar = 0; bar < 4; bar++) {
                    Vec3 at = base + left * (side * (half + 55 + bar * 0.9f));
                    if (bar < 2) {
                        strip(paint, at - u * 11.5f, at + u * 11.5f, 0.15f, YELLOW);
                    } else {
                        for (float d = -11.5f; d < 11.5f; d += 2.4f) {
                            strip(paint, at + u * d, at + u * std::min(11.5f, d + 1.2f), 0.15f, YELLOW);
                        }
                    }
                }
            }
        }
        strip(asphalt, r.end[0] - u * 30, r.end[1] + u * 30, half, ASPHALT);
        // Rubber laid down in each touchdown zone, darkest where the main
        // wheels meet it, in streaks of different lengths.
        for (int e = 0; e < 2 && len > 1500; e++) {
            Vec3 from = r.end[e], way = e == 0 ? u : u * -1;
            Vec3 right = {way.y, -way.x, 0};
            for (int k = 0; k < 6; k++) {
                float a0 = 260 + 40 * k, a1 = 620 + 70 * (k % 3), off = (k % 2 ? 1 : -1) * (2.5f + 0.6f * (k / 2));
                Rgb rubber = k < 2 ? Rgb{46, 47, 50} : Rgb{54, 55, 58};
                strip(rubberMarks, from + way * a0 + right * off, from + way * a1 + right * off, 1.6f, rubber);
            }
        }

        for (int e = 0; e < 2; e++) {
            RunwayEnd end;
            end.origin = r.end[e];
            end.along = e == 0 ? u : u * -1;
            end.right = Vec3{end.along.y, -end.along.x, 0};
            end.length = len;
            end.halfWidth = half;
            end.number = atoi(r.name[e]);
            end.letter = r.name[e][strlen(r.name[e]) - 1];
            if (end.letter < 'A') {
                end.letter = 0;
            }
            // The way off its landing takes: the first a decent rollout away.
            end.exitAlong = 0;
            for (float c : connectors) {
                float from = e == 0 ? c : len - c;
                if (from >= 1300 && (end.exitAlong == 0 || from < end.exitAlong)) {
                    end.exitAlong = from;
                }
            }
            if (end.exitAlong == 0) {
                end.exitAlong = len - 48;
            }
            end.exitRight = dot(left * side, end.right) > 0;
            paintEnd(paint, end);
            buildApproachFrames(m.wires, end);
            buildRunwayLights(m.airfieldLights, end, (int)m.ends.size());
            buildWindsock(m.solids, m.wires, end);
            for (int k = 0; k < 4; k++) {
                m.papis.push_back({end.at(350, -(half + 15 + 9.0f * k)) + Vec3{0, 0, 0.9f}, (3.5f - k / 3.0f) * DEG,
                                   end.along * -1});
            }
            m.ends.push_back(end);
        }
        // The edges, and the centreline's dashes: 30m painted, 20m not.
        const RunwayEnd &a = m.ends[m.ends.size() - 2];
        a.paint(paint, 0, half - 0.9f, len, half);
        a.paint(paint, 0, -half, len, -half + 0.9f);
        for (float x = 70; x + 30 <= len - 70; x += 50) {
            a.paint(paint, x, -0.45f, x + 30, 0.45f);
        }
    }

    // The terminal: the apron, a building down its middle, piers out from it
    // either side, and stands along both sides of each pier, nose in - or
    // the real ones.
    Vector<ModelTri> apron, taxiwaysFar;
    if (real) {
        buildLayout(m, *layout, runways, count, apron, taxiways, taxiwaysFar, paint, taxiwayLights);
        // Each taxiway's line to within 20m, its grass 40m either side, and
        // none for the short stubs between others' - Hong Kong's were 7,000
        // triangles a frame, 8m.
        Vector<Vec3> line, coarse;
        for (const airfield::Way &w : layout->ways) {
            if (!shore || w.kind != airfield::WayKind::TAXIWAY) {
                continue;
            }
            lineOf(*layout, w, line);
            simplified(line, 20, coarse);
            for (size_t k = 0; k + 1 < coarse.size(); k++) {
                if (length(coarse[k + 1] - coarse[k]) > 40) {
                    strip(grass, coarse[k], coarse[k + 1], w.width / 2 + 40, GRASS);
                }
            }
        }
    } else if (madeUp) {
    rect(grass, middle - axis * (halfLength + 60) - across * (halfDepth + 60),
         middle + axis * (halfLength + 60) - across * (halfDepth + 60),
         middle + axis * (halfLength + 60) + across * (halfDepth + 60),
         middle - axis * (halfLength + 60) + across * (halfDepth + 60), GRASS);
    rect(apron, middle - axis * halfLength - across * halfDepth, middle + axis * halfLength - across * halfDepth,
         middle + axis * halfLength + across * halfDepth, middle - axis * halfLength + across * halfDepth, CONCRETE);
    orientedBox(m.solids, middle, axis, halfLength - 40, 28, 16, TERMINAL, ROOF);
    m.blocks.push_back(blockOf(middle, axis, halfLength - 40, 28, 16));
    // An A320's nose is this far ahead of its nose wheel.
    constexpr float PIER_EVERY = 170, STAND_EVERY = 50, NOSE_PAST = 4.4f;
    for (float a = -halfLength + 90; a <= halfLength - 90; a += PIER_EVERY) {
        for (float s : {1.0f, -1.0f}) {
            float from = 28, to = halfDepth - 50;
            if (to - from < 60) {
                continue;
            }
            Vec3 pierMiddle = middle + axis * a + across * (s * (from + to) / 2);
            orientedBox(m.solids, pierMiddle, across, (to - from) / 2, 10, 9, PIER, ROOF);
            m.blocks.push_back(blockOf(pierMiddle, across, (to - from) / 2, 10, 9));
            for (float c = from + 30; c <= to - 10; c += STAND_EVERY) {
                for (float side : {1.0f, -1.0f}) {
                    // The stop far enough from the pier for an A320's nose to
                    // stop 5m short of it.
                    Vec3 stop = middle + axis * (a + side * (10 + 5 + NOSE_PAST)) + across * (s * c);
                    Vec3 nose = axis * -side;
                    m.stands.push_back({stop, atan2f(nose.y, nose.x), 5 + NOSE_PAST, false});
                }
            }
        }
    }
    m.tower = middle + axis * (halfLength - 40) + across * (halfDepth - 30);
    buildTower(m.wires, m.tower);
    } else {
        // The runways alone - the airfield pack not on the card, or this
        // airport not in it: nothing made up where it isn't known. The TOWER
        // camera has a place still, where the made-up one's tower would be:
        // a vantage, with no tower drawn under it.
        m.tower = middle + axis * (halfLength - 40) + across * (halfDepth - 30);
    }

    // The ground in layers, each painted over the one before: the grass, the
    // aprons, the taxiways, the runways, the rubber on them, and then the
    // broad paint below. Within a layer the order is nothing - it is all one
    // surface - so each is grouped by the square it is in, and a frame
    // visits only the squares in view, layer by layer.
    if (!shore) {
        grassHull(grass, {&grass, &apron, &taxiways, &asphalt});
    } else {
#if !defined(ESP_PLATFORM)
        auto filled0 = std::chrono::steady_clock::now();
#endif
        fillBetween(grass, {&grass, &apron, &taxiways, &asphalt}, 250);
#if !defined(ESP_PLATFORM)
        if (std::getenv("PROFILE")) {
            std::printf("  the gaps filled in %.2f ms\n",
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - filled0).count());
        }
#endif
    }
    for (ModelTri &t : grass) {
        t.paint = PAINT_GRASS;
    }
    m.ground = grass;
    for (const Vector<ModelTri> *layer : {&apron, &taxiways, &taxiwaysFar, &asphalt, &rubberMarks}) {
        m.ground.insert(m.ground.end(), layer->begin(), layer->end());
    }
    constexpr int LAYERS = 7;
    size_t layerEnds[LAYERS] = {grass.size(), 0, 0, 0, 0, 0, 0};
    layerEnds[1] = layerEnds[0] + apron.size();
    layerEnds[2] = layerEnds[1] + taxiways.size();
    layerEnds[3] = layerEnds[2] + taxiwaysFar.size();
    layerEnds[4] = layerEnds[3] + asphalt.size();
    layerEnds[5] = m.ground.size();
    // The grass only where there are coasts about; the fine taxiways only
    // near, when there is a coarse version for far.
    const uint8_t layerLod[LAYERS] = {LOD_GRASS, LOD_ANY, (uint8_t)(taxiwaysFar.empty() ? LOD_ANY : LOD_NEAR), LOD_FAR,
                                      LOD_ANY, LOD_ANY, LOD_ANY};

    // The paint, each piece made by rect() as a pair of triangles, {a, b, c}
    // and {a, c, d}: the broad as it is, the thin - taxiways' centrelines,
    // runways' edges - as lines down their middles, for addPaint() to draw
    // as the view has them.
    m.paintLines.clear();
    for (size_t i = 0; i + 1 < paint.size(); i += 2) {
        const ModelTri &t = paint[i];
        if (t.fine >= 1.5f) {
            m.ground.push_back(t);
            m.ground.push_back(paint[i + 1]);
            continue;
        }
        Vec3 a = t.v[0], b = t.v[1], c = t.v[2], d = paint[i + 1].v[2];
        bool alongAB = length(b - a) >= length(d - a);
        Vec3 from = alongAB ? (a + d) * 0.5f : (a + b) * 0.5f, to = alongAB ? (b + c) * 0.5f : (d + c) * 0.5f;
        m.paintLines.push_back({from, to, t.fine, t.color, length(to - from) < 3});
    }
    m.paintLines.shrink_to_fit();

    for (int i = 0; i < beaconCount; i++) {
        buildVorDme(m.wires, beacons[i]);
    }
    buildTownLights(m.townLights, middle);

    auto tri = [](const ModelTri &t, Vec3 &lo, Vec3 &hi) {
        for (const Vec3 &v : t.v) {
            widen(v, 0, lo, hi);
        }
    };
    auto light = [](const ModelLight &l, Vec3 &lo, Vec3 &hi) { widen(l.at, l.size, lo, hi); };
    layerEnds[6] = m.ground.size();  // the broad paint, last
#if !defined(ESP_PLATFORM)
    if (std::getenv("GROUND_STATS")) {
        const char *names[LAYERS] = {"grass", "apron", "taxiways", "taxiways far", "asphalt", "rubber", "broad paint"};
        for (int k = 0, from = 0; k < LAYERS; from = (int)layerEnds[k], k++) {
            double area = 0;
            for (size_t i = from; i < layerEnds[k]; i++) {
                const ModelTri &t = m.ground[i];
                Vec3 u = t.v[1] - t.v[0], v = t.v[2] - t.v[0];
                area += fabsf(u.x * v.y - u.y * v.x) / 2;
            }
            std::printf("  ground %s: %zu triangles, %.0f m2 each on average\n", names[k], layerEnds[k] - from,
                   area / std::max<size_t>(1, layerEnds[k] - from));
        }
    }
#endif
    m.groundCells.clear();
    for (int k = 0, from = 0; k < LAYERS; from = (int)layerEnds[k], k++) {
        size_t first = m.groundCells.size();
        intoCells(m.ground, from, layerEnds[k], AIRFIELD_CELL, m.groundCells, tri);
        for (size_t c = first; c < m.groundCells.size(); c++) {
            m.groundCells[c].lod = layerLod[k];
        }
    }
    // The paint in three runs, each in cells of its own - the broad, drawn at
    // any distance; the thin, within 1.5km; the dashes, within 600m - so a
    // cell too far off for its run is passed over unread: from an approach,
    // nine in ten of a big airport's lines in view.
    auto kindOf = [](const PaintLine &l) { return l.near ? PAINT_DASHES : l.width < 0.5f ? PAINT_THIN : PAINT_BROAD; };
    std::stable_sort(m.paintLines.begin(), m.paintLines.end(),
                     [&](const PaintLine &x, const PaintLine &y) { return kindOf(x) < kindOf(y); });
    m.paintCells.clear();
    for (size_t from = 0; from < m.paintLines.size();) {
        uint8_t kind = kindOf(m.paintLines[from]);
        size_t to = from;
        while (to < m.paintLines.size() && kindOf(m.paintLines[to]) == kind) {
            to++;
        }
        size_t first = m.paintCells.size();
        intoCells(m.paintLines, from, to, AIRFIELD_CELL, m.paintCells, [](const PaintLine &l, Vec3 &lo, Vec3 &hi) {
            widen(l.a, l.width, lo, hi);
            widen(l.b, l.width, lo, hi);
        });
        for (size_t c = first; c < m.paintCells.size(); c++) {
            m.paintCells[c].lod = kind;
        }
        from = to;
    }
    m.solidCells.clear();
    intoCells(m.solids, 0, m.solids.size(), AIRFIELD_CELL, m.solidCells, tri);
    m.detailCells.clear();
    intoCells(m.detail, 0, m.detail.size(), AIRFIELD_CELL, m.detailCells, tri);
    auto normalsOf = [](const Vector<ModelTri> &tris, Vector<Vec3> &normals) {
        normals.clear();
        normals.reserve(tris.size());
        for (const ModelTri &t : tris) {
            normals.push_back(normalize(cross(t.v[1] - t.v[0], t.v[2] - t.v[0])));
        }
    };
    normalsOf(m.solids, m.solidNormals);
    normalsOf(m.detail, m.detailNormals);
    m.wireCells.clear();
    intoCells(m.wires, 0, m.wires.size(), AIRFIELD_CELL, m.wireCells, [](const ModelLine &l, Vec3 &lo, Vec3 &hi) {
        widen(l.a, 0, lo, hi);
        widen(l.b, 0, lo, hi);
    });
    // The taxiways' lights after the rest, in cells of their own: far dimmer
    // than the runways', they fade with distance - see prepareEnd().
    size_t firstTaxiway = m.airfieldLights.size();
    m.airfieldLights.insert(m.airfieldLights.end(), taxiwayLights.begin(), taxiwayLights.end());
    m.lightCells.clear();
    intoCells(m.airfieldLights, 0, firstTaxiway, AIRFIELD_CELL, m.lightCells, light);
    size_t taxiwayCells = m.lightCells.size();
    intoCells(m.airfieldLights, firstTaxiway, m.airfieldLights.size(), AIRFIELD_CELL, m.lightCells, light);
    for (size_t c = taxiwayCells; c < m.lightCells.size(); c++) {
        m.lightCells[c].lod = LIGHTS_TAXIWAY;
    }
    m.townCells.clear();
    intoCells(m.townLights, 0, m.townLights.size(), AIRFIELD_CELL * 5, m.townCells, light);
    compact(m);
}

// The made-up airfield the preview and bench start with: one runway, 09/27,
// 3000m long and 45 wide, east from the origin, and a VOR/DME beside it.
void buildDefaultAirfield(Models &m) {
    RunwaySpec one = {{{0, 0, 0}, {3000, 0, 0}}, 45, {"09", "27"}};
    Vec3 vor = {420, -360, 0};
    buildAirfield(m, &one, 1, &vor, 1, nullptr, true);
}

Models &models() {
    if (!g_models.built) {
        g_models.built = true;
        withRoom([] {
            for (int f = 0; f < (int)Family::COUNT; f++) {
                buildDesign((Family)f, designs()[f]);
            }
            buildStars(g_models.stars);
            buildDefaultAirfield(g_models);
        });
    }
    return g_models;
}


// --- a frame -------------------------------------------------------------------

// A triangle on the screen: its corners top to bottom, as A, B and C, and the
// slopes of its edges; 1/z as a plane, a * x + b * y + c; and the rows
// [top, bottom) whose pixel centres it may cover.
struct ScreenTri {
    float ax, ay, bx, by;
    float dAC, dAB, dBC;
    float za, zb, zc;
    int16_t top, bottom;
    uint16_t color;  // a solid's, lit, hazed and packed
    uint8_t lit[3];  // the ground's, lit, hazed a row at a time
};

struct ScreenLine {
    float x0, y0, iz0, x1, y1, iz1;
    int top, bottom;
    uint16_t color;
};

// A light or a puff of smoke: a disc facing the eye, at one depth. A light's
// core is solid; its glow is added to what is behind it, fading to its rim;
// smoke is see-through, drawn in one, two or three of each two by two pixels.
enum class Blend : uint8_t { SOLID, GLOW, STIPPLE, TINT };

struct ScreenSprite {
    float x, y, r, iz;
    int top, bottom;
    uint16_t color;  // SOLID, STIPPLE and TINT
    float add[3];    // GLOW: what it adds at its middle, in 565's levels
    Blend blend;
    uint8_t coverage;  // STIPPLE: of 4; TINT: eighths of its colour at its middle, fading to none at its rim
};

// A light far enough off to be a single pixel with no glow round it - most of
// a big airport's after dark, seen from the approach: where it is, how near
// and its colour, a quarter the size of a ScreenSprite, drawn as its one
// pixel. Its row is `top`; `bottom` the one after, for its List.
struct ScreenDot {
    int16_t x, top, bottom;
    uint16_t color;
    float iz;
};

// A list of what is to be drawn, and beside it each one's rows, top and
// bottom in a word: sorting them into bands then reads four bytes apiece
// rather than the whole of each, which is out in PSRAM - read twice over,
// that was a quarter of a busy frame's preparing.
template <typename T>
struct List {
    Vector<T> items;
    Vector<uint32_t> rows;

    void push_back(const T &t) {
        items.push_back(t);
        rows.push_back((uint32_t)t.top | (uint32_t)t.bottom << 16);
    }
    void clear() {
        items.clear();
        rows.clear();
    }
    size_t size() const { return items.size(); }
    const T &operator[](size_t i) const { return items[i]; }
};

// Each band's share of some lists of what is to be drawn - triangles, lines
// or sprites - in their order, the first list's first: band b's are
// index[start[b]] up to index[start[b + 1]], each the list in its top two
// bits and the place in it below.
constexpr int LIST_SHIFT = 30;
constexpr uint32_t PLACE_MASK = (1u << LIST_SHIFT) - 1;

struct Bins {
    Vector<uint32_t> start, index, cursor;

    template <typename T>
    void build(const List<T> *const *lists, int count, int bandRows, int bands) {
        start.assign(bands + 1, 0);
        for (int l = 0; l < count; l++) {
            for (uint32_t r : lists[l]->rows) {
                for (int b = (int)(r & 0xFFFF) / bandRows; b <= (int)((r >> 16) - 1) / bandRows; b++) {
                    start[b + 1]++;
                }
            }
        }
        for (int b = 0; b < bands; b++) {
            start[b + 1] += start[b];
        }
        index.resize(start[bands]);
        cursor.assign(start.begin(), start.end() - 1);
        for (int l = 0; l < count; l++) {
            const Vector<uint32_t> &rows = lists[l]->rows;
            for (uint32_t i = 0; i < rows.size(); i++) {
                for (int b = (int)(rows[i] & 0xFFFF) / bandRows; b <= (int)((rows[i] >> 16) - 1) / bandRows; b++) {
                    index[cursor[b]++] = (uint32_t)l << LIST_SHIFT | i;
                }
            }
        }
    }
};

struct Projected {
    float x, y, iz;
};

// The aircraft in the frame: each placed, nearest first, at its detail.
struct Placed {
    Pose pose;
    Look look;
    Config config;
    int index;       // in the scene's traffic; -1 the one followed
    float distance;  // from the eye to its middle
    int detail;      // DETAIL_HERO down to DETAIL_LIGHTS
    bool followed;
    bool whole;  // wholly in view, nothing of it to clip
    // Its middle on the screen and how far round it it covers, for
    // labelAnchors() - reach 0 if the middle is behind the eye.
    float screenX, screenY, reach;
    // Its middle and its top on the screen - off it, -1 - and how near its
    // nearest could be, 1/z: as the frame is drawn, whether something nearer
    // than that is drawn at each - a building, another aircraft - for the
    // next frame's labelAnchors() to leave its label off.
    int16_t probeX[2], probeY[2];
    float probeIz;
    bool covered[2];
};

// Where a point of an aircraft is, it being where `pose` says.
struct Placer {
    float cp, sp, ch, sh, cr, sr, scale;
    Vec3 at;
    explicit Placer(const Pose &pose, float size = 1)
        : cp(cosf(pose.pitch)), sp(sinf(pose.pitch)), ch(cosf(pose.heading)), sh(sinf(pose.heading)),
          cr(cosf(pose.roll)), sr(sinf(pose.roll)), scale(size), at(pose.at) {}
    Vec3 turn(Vec3 v) const {  // rolled about its length, pitched about its wheels, then turned to its heading
        float y = v.y * cr - v.z * sr, z0 = v.y * sr + v.z * cr;
        float x = v.x * cp - z0 * sp, z = v.x * sp + z0 * cp;
        return {x * ch - y * sh, x * sh + y * ch, z};
    }
    Vec3 place(Vec3 v) const { return at + turn(v * scale); }
};

// The aircraft are shared out in parts, for each core to take one: what a part
// makes of them, and its scratch for their corners.
constexpr int PARTS = 2;

struct Part {
    // clear: see-through, the blur of turning blades - drawn over the
    // solids, tinting what is behind it so many eighths of its colour.
    List<ScreenTri> solid, shadow, clear;
    List<ScreenSprite> sprites;
    Vector<Vec3> world, cam;
    Vector<Projected> proj;
    float nearest;  // the least depth of any corner of its aircraft
    Counts counts;
};

struct Frame {
    int w = 0, h = 0, bandRows = 1, bands = 0;
    bool swap = false;
    Palette pal = KEYS[4].palette;
    Vec3 sun{};      // where things are lit from: the sun while it is up, the sky's last light after
    Vec3 sunDir{};   // where the sun itself is
    float sunElevationDeg = 40;
    bool shadows = true;
    int shadow256 = 256;
    alignas(16) uint16_t shadowLanes[4][8] = {};  // scene3d_darken8's constants: k, then red's, green's and blue's masks
    float cx = 0, cy = 0, f = 1;
    // The view's sides, guard band and all, as slopes - x or y over depth -
    // and how much a point's distance in from them is to be scaled for a
    // ball's: worked out once a frame, not for every corner of every
    // triangle, two divisions each.
    float gx = 1, gy = 1, sx = 1, sy = 1;
    float wx = 1, wy = 1;  // the same for WIDE
    Vec3 eye{}, right{}, down{}, forward{};
    // A step of the grid the ground and the buildings are kept on - see
    // GroundTri - east, north and up, as the camera has it.
    Vec3 gridX{}, gridY{}, gridZ{};
    Scene scene;
    Vector<Placed> placed;
    Part parts[PARTS];
    List<ScreenTri> ground, solid;  // solid: what stands that isn't an aircraft
    List<ScreenLine> wires;
    List<ScreenSprite> sprites;
    List<ScreenDot> dots;
    const List<ScreenTri> *solidLists[PARTS + 1], *shadowLists[PARTS], *clearLists[PARTS];
    const List<ScreenSprite> *spriteLists[PARTS + 1];
    Bins groundBins, shadowBins, solidBins, clearBins, wireBins, spriteBins, dotBins;
    Vector<uint32_t> rowColor;  // the sky or the grass behind each row, a dithered pair
    Vector<uint16_t> rowHaze;   // how much haze the ground in each row is under, of 256
    // Over coasts and towns: each row's land and towns as well as its sea -
    // their colours, and where along the row each begins and ends,
    // rowCross[rowStart[y]] up to rowCross[rowStart[y + 1]], left to right,
    // with what the row is before the first of them: in rowFirst, land the
    // first bit, a town the second.
    struct Crossing {
        int16_t x;
        uint8_t layer;  // the land's (0) or the towns' (1)
        bool on;        // after it
    };
    bool landSea = false;
    Vector<uint32_t> rowLand, rowTown;
    Vector<uint8_t> rowGround, rowFirst;
    Vector<uint32_t> rowStart;
    Vector<Crossing> rowCross;
    // What covers the land, where the view has the ground pack - see
    // GroundCover. For a row of ground, where five points along it are on
    // the pack's grid, in tiles - its left end, a quarter, half and three
    // quarters of the way, its right end - and the level its cells are read
    // at, the finest whose cells are still four pixels or more; -1 for a row
    // the land is one colour in. `straight` where the five are on a line - the
    // ground flat, or the earth's curve across the row a twentieth of a cell
    // or less - for its cells to be read along the one line, end to end.
    // Flat, worked out as each row is drawn, in a few dozen cycles; round,
    // here, for each even row - the odd row under it takes its cells - a
    // thousand or two each.
    // A row's cover: its level - NO_LEVEL where the row has no ground.
    static constexpr int8_t NO_LEVEL = -128;
    struct RowCover {
        int8_t level;
        bool straight;
        bool tall;  // its cells eight rows or more tall: four rows can share a walk along it
        bool wide;  // and sixteen pixels or more wide: a sample of them each eight pixels
        float u[5], v[5];
    };
    GroundCover cover;
    bool coverOn = false;
    // Towns in the cover as well, by day: its own town middles, suburbs and
    // parks say more than one grey. After dark, the towns' own colour, with
    // the glow of their lights far off, over a cover all but black.
    bool coverTowns = false;
    Vector<RowCover> rowCover;
    // Flat, a row's ends on the grid from y in a dozen multiply-adds and a
    // divide, these worked out once a frame - see coverFlatEnds(): the row's
    // slope, sy = y/f + sy0; the ray's height, dz = fz + dz1 sy; how far
    // along it the ground is, t = ez / -dz; and its left end, u = au + t (bu +
    // cu sy), stepping t ku a pixel - v the same.
    struct CoverFlat {
        float perF, sy0, fz, dz1, ez, au, bu, cu, ku, av, bv, cv, kv;
    } coverFlat;
    uint8_t coverLit[16][3] = {};  // each cover's colour, lit as the land is
    uint8_t coverRgb[256][3] = {};  // each cell's: its cover's colour, lit, by its shade
    // A round cover's origin, in degrees, and the sine and cosine of its
    // latitude: as floats once a frame, not converted from double - in
    // software, the P4 having no double in its FPU - at every point.
    float coverLat0 = 0, coverLon0 = 0, coverSin = 0, coverCos = 1;
    float perW = 1;  // 1/w
    // The hills along the skyline, a range at a time - see
    // groundpack::Horizon - the furthest drawn first and each nearer over
    // it: each column's first row of the range's hills and the row after
    // its last - the same for none - and its colour, the hill there hazed as
    // far off as it is, dithered for even rows and odd, and how far off it
    // is, as 1/z and then, once the frame's scale for it is known, as the
    // depth buffer has it; in PSRAM, 16-byte aligned, as PIE reads them (see
    // scene3d_hills8); and the rows they reach between.
    struct HillRange {
        bool on = false;
        int top = 0, bottom = 0;
        int16_t *from = nullptr, *to = nullptr;
        uint16_t *pixels[2] = {};  // each column's colour, for even rows and odd
        float *iz = nullptr;
        uint16_t *depth = nullptr;
    };
    HillRange hills[groundpack::Horizon::RANGES];
    Vector<int16_t> hillRows;
    Vector<uint16_t> hillColors, hillDepths;
    Vector<float> hillIz;
    // The land at its height - see terrainMarch(): for each fourth column,
    // 1 / how deep along the view each row meets the land, bfloat16 - the
    // top half of a float - a column's rows together, as the march writes them
    // and a band reads them; and the column's highest row that does: above it, sky or the land past reach. And what a row and a
    // column each add to where on the cover's grid that depth puts the land:
    // u = tUe + z (tRowU[y] + tColU[c]), and v likewise; the depth each
    // level of the cover is read out to, its cells a sample or more; and how
    // much haze a depth puts over the land, a row at a time - tRowFar[y]
    // the distance a metre of depth comes to along its ray.
    bool terrainOn = false;
    int tCols = 0;
    Vector<uint16_t> tz;
    Vector<int16_t> tTop;
    float tUe = 0, tVe = 0;
    Vector<float> tRowU, tRowV, tRowFar, tColU, tColV;
    // And for the assembly, a column's at once: cu and cv side by side, and
    // its highest row of land with its flat top above it - top | flatTop << 16.
    Vector<float> tColUV, tColXY;
    Vector<uint32_t> tTopFlat;
    // Round, high up: where on the grid a place dx east and dy north of the
    // eye is, as u = U[0] + U[1] dx + U[2] dy + U[3] dx^2 + U[4] dx dy +
    // U[5] dy^2, and v likewise - the meridians closing in, the earth
    // curving - and a depth's dx and dy, z (tRowX[y] + tColX[c]) and so on.
    bool tRound = false;
    float tMapU[6] = {}, tMapV[6] = {};
    Vector<float> tRowX, tRowY, tColX, tColY;
    // At an airfield, its ground held flat round it: the rows each column
    // sees it in - from tFlatTop[c] down - its depth there each row's alone,
    // tFlatZ[y]; and the row from which every column does, tFlatAll - those
    // rows drawn as the flat ground is, a straight walk along each.
    Vector<int16_t> tFlatTop;
    Vector<float> tFlatZ;
    int tFlatAll = 1 << 30;
    float tLevelZ[groundpack::LEVELS - groundpack::FINEST] = {};
    float tLevelPer = 0;  // 1 / tLevelZ[FINEST]: a depth's share of the finest level's reach
    // The rows a sample of the land's colours covers, from a multiple of it:
    // 2, or 4 where the governor has the land skimping - its cells a level
    // coarser then, so that one is four rows tall where it is four pixels
    // wide, and the fewer samples miss none.
    int tUnit = 2;
    // The haze over the land at a distance, of 32: by the distance's float
    // bits' top eleven - its exponent and two bits more, four steps an
    // octave - all of them, a positive float's, so that nothing need be
    // clamped to read it.
    uint8_t tHaze[1024] = {};
    uint32_t serial = 0;  // which frame - for the colours' caches
    // Depth is kept in 16 bits - half the memory of a float, and eight to a
    // PIE vector: 1/z scaled so that the nearest thing in the frame comes to
    // DEPTH_TOP, which spends all 16 bits' precision on what is there to draw.
    float depthK = 1;
    Counts counts{};
};

// Two frames: `g` the one being prepared, `d` the one being drawn - so one
// core can work out the next frame while the other draws this one. flip()
// makes the one prepared the one to draw.
Frame g, d;

constexpr float DEPTH_TOP = 32000;

// A depth as kept, a little nearer than it is: for a strut or a light, so
// one along a surface isn't lost in it.
inline uint16_t depthOf(float iz) { return (uint16_t)std::min(65535.0f, iz * d.depthK * 1.002f); }

uint32_t (*g_clock)() = nullptr;
uint32_t now() { return g_clock != nullptr ? g_clock() : 0; }

uint16_t swapIf(uint16_t v) { return g.swap ? (uint16_t)(v >> 8 | v << 8) : v; }

uint16_t pack(Rgb c) {
    int r = std::min(255, std::max(0, (int)c.r)), gr = std::min(255, std::max(0, (int)c.g)),
        b = std::min(255, std::max(0, (int)c.b));
    return swapIf((uint16_t)((r >> 3) << 11 | (gr >> 2) << 5 | b >> 3));
}

// RGB565 has 32 levels of red and blue: a gentle gradient - the sky, or haze
// coming over the grass - steps in bands across the screen. So those are
// dithered, two by two: each row a pair of colours, for its even and odd
// pixels, whose levels average out to the colour between. A pair is what one
// 32-bit write puts down, so it costs no more than a plain colour.
uint16_t packDithered(Rgb c, float d) {
    // Times top/255, not times top over 255: a multiply, where the division
    // was a float divide a channel - six to a pair, three pairs to a row.
    auto level = [d](float v, float scale, int top) { return std::min(top, std::max(0, (int)(v * scale + d))); };
    return swapIf((uint16_t)(level(c.r, 31.0f / 255, 31) << 11 | level(c.g, 63.0f / 255, 63) << 5 |
                             level(c.b, 31.0f / 255, 31)));
}

uint32_t pair(Rgb c, int y) {
    static const float BAYER[2][2] = {{0, 0.5f}, {0.75f, 0.25f}};
    return packDithered(c, BAYER[y & 1][0]) | (uint32_t)packDithered(c, BAYER[y & 1][1]) << 16;
}

// The same in whole numbers, for the ground's spans, of which a frame has
// thousands: a colour of 0-255 a channel, hazed `haze` of 256 towards
// `towards`.
uint32_t pairInt(const uint8_t lit[3], const int towards[3], int haze, int y) {
    // The Bayer thresholds, as a fraction of 255.
    static const int BAYER[2][2] = {{0, 127}, {191, 63}};
    int v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = lit[i] + ((towards[i] - lit[i]) * haze >> 8);
    }
    auto pack = [&](int dither) {
        // x / 255, exactly, for x under 65536.
        auto level = [dither](int c, int top) {
            int x = c * top + dither;
            return std::min(top, (x + 1 + (x >> 8)) >> 8);
        };
        return swapIf((uint16_t)(level(v[0], 31) << 11 | level(v[1], 63) << 5 | level(v[2], 31)));
    };
    return pack(BAYER[y & 1][0]) | (uint32_t)pack(BAYER[y & 1][1]) << 16;
}

Rgb hazed(Rgb c, float distance) { return mix(c, g.pal.haze, 1 - expf(-distance / g.pal.visibility)); }

// How high a point is above the ground: above the round earth's surface,
// near enough, where the scene has one.
float heightOf(Vec3 p) {
    float r = g.scene.earthRadius;
    return r > 0 ? p.z + (p.x * p.x + p.y * p.y) / (2 * r) : p.z;
}

// Whether the earth's curve hides a point from the eye: further than the
// horizon is from the eye, and the point's own from it, together.
bool overHorizon(Vec3 p) {
    float r = g.scene.earthRadius;
    if (r <= 0) {
        return false;
    }
    float dx = p.x - g.eye.x, dy = p.y - g.eye.y;
    float reach = sqrtf(2 * r * std::max(1.0f, heightOf(g.eye))) + sqrtf(2 * r * std::max(0.0f, heightOf(p)));
    return dx * dx + dy * dy > reach * reach;
}

// The same for a point on the ground, squared and worked out once a frame -
// no root to take for each of thousands of coastline points.
float g_groundReach2 = 0;
bool groundOverHorizon(Vec3 p) {
    float dx = p.x - g.eye.x, dy = p.y - g.eye.y;
    return g.scene.earthRadius > 0 && dx * dx + dy * dy > g_groundReach2;
}

// The ground under a point: on the round earth, straight down towards its
// middle.
Vec3 groundUnder(Vec3 p) {
    float r = g.scene.earthRadius;
    if (r <= 0) {
        return {p.x, p.y, 0};
    }
    Vec3 up = normalize(Vec3{p.x, p.y, p.z + r});
    return p - up * heightOf(p);
}

// What every corner of every triangle goes through, thousands of times a
// frame, put in place where it is used. Left to itself the compiler calls
// them - and on the P4 a Vec3, three floats, goes to a function and back
// through memory.
#define HOT_INLINE __attribute__((always_inline)) inline

// (int)ceilf(v) and (int)floorf(v). The library's are calls of thirty-odd
// instructions on the P4, and drawing takes two a row of every triangle -
// where the P4 converts a float to an int rounding either way in one.
HOT_INLINE int ceilInt(float v) {
#if defined(ESP_PLATFORM)
    int i;
    asm("fcvt.w.s %0, %1, rup" : "=r"(i) : "f"(v));
    return i;
#else
    return (int)ceilf(v);
#endif
}
HOT_INLINE int floorInt(float v) {
#if defined(ESP_PLATFORM)
    int i;
    asm("fcvt.w.s %0, %1, rdn" : "=r"(i) : "f"(v));
    return i;
#else
    return (int)floorf(v);
#endif
}

HOT_INLINE Vec3 toCamera(Vec3 p) {
    Vec3 d = p - g.eye;
    return {dot(d, g.right), dot(d, g.down), dot(d, g.forward)};
}

// The planes things are clipped to, in the camera's space, each as how far
// inside it a point is: the near plane, and the four sides of the view.
HOT_INLINE float inside(int plane, Vec3 p) {
    switch (plane) {
    case 0: return p.z - NEAR;
    case 1: return p.x + p.z * g.gx;
    case 2: return p.z * g.gx - p.x;
    case 3: return p.y + p.z * g.gy;
    default: return p.z * g.gy - p.y;
    }
}

// Which of those planes a point is outside of, a bit for each.
HOT_INLINE unsigned outside(Vec3 p) {
    float x = p.z * g.gx, y = p.z * g.gy;
    return (p.z < NEAR) | (p.x < -x) << 1 | (p.x > x) << 2 | (p.y < -y) << 3 | (p.y > y) << 4;
}

// The same with the sides out at WIDE: where a triangle is cut.
HOT_INLINE float insideWide(int plane, Vec3 p) {
    switch (plane) {
    case 0: return p.z - NEAR;
    case 1: return p.x + p.z * g.wx;
    case 2: return p.z * g.wx - p.x;
    case 3: return p.y + p.z * g.wy;
    default: return p.z * g.wy - p.y;
    }
}
HOT_INLINE unsigned outsideWide(Vec3 p) {
    float x = p.z * g.wx, y = p.z * g.wy;
    return (p.z < NEAR) | (p.x < -x) << 1 | (p.x > x) << 2 | (p.y < -y) << 3 | (p.y > y) << 4;
}

// How far inside the view a ball's middle is, in the camera's space, as the
// least of its distances in from each side: more than its radius, and all of
// it is in view; less than minus that, and none of it is.
float ballInside(Vec3 c) {
    return std::min({c.z - NEAR, inside(1, c) * g.sx, inside(2, c) * g.sx, inside(3, c) * g.sy, inside(4, c) * g.sy});
}

// Sutherland-Hodgman, cutting each crossing edge from its inside end, so an
// edge two triangles share is cut at the same point by both - at the near
// plane, or the sides out at WIDE.
int clip(Vec3 *poly, int n, int plane) {
    Vec3 out[12];
    int m = 0;
    for (int i = 0; i < n; i++) {
        Vec3 a = poly[i], b = poly[(i + 1) % n];
        float da = insideWide(plane, a), db = insideWide(plane, b);
        if (da >= 0) {
            out[m++] = a;
        }
        if ((da >= 0) != (db >= 0)) {
            out[m++] = da >= 0 ? a + (b - a) * (da / (da - db)) : b + (a - b) * (db / (db - da));
        }
    }
    memcpy(poly, out, sizeof(Vec3) * m);
    return m;
}

HOT_INLINE Projected project(Vec3 c) {
    float iz = 1 / c.z;
    return {g.cx + g.f * c.x * iz, g.cy + g.f * c.y * iz, iz};
}

HOT_INLINE bool setup(ScreenTri &t, Projected p0, Projected p1, Projected p2) {
    float ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.iz - p0.iz;
    float vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.iz - p0.iz;
    float nz = ux * vy - uy * vx;
    if (fabsf(nz) < 1e-6f) {
        return false;
    }
    float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz;
    t.za = -nx / nz;
    t.zb = -ny / nz;
    t.zc = p0.iz - t.za * p0.x - t.zb * p0.y;
    if (p1.y < p0.y) std::swap(p0, p1);
    if (p2.y < p0.y) std::swap(p0, p2);
    if (p2.y < p1.y) std::swap(p1, p2);
    int top = std::max(0, ceilInt(p0.y - 0.5f)), bottom = std::min(g.h, ceilInt(p2.y - 0.5f));
    if (top >= bottom) {
        return false;
    }
    t.top = (int16_t)top;
    t.bottom = (int16_t)bottom;
    t.ax = p0.x, t.ay = p0.y, t.bx = p1.x, t.by = p1.y;
    t.dAC = (p2.x - p0.x) / (p2.y - p0.y);
    t.dAB = p1.y > p0.y ? (p1.x - p0.x) / (p1.y - p0.y) : 0;
    t.dBC = p2.y > p1.y ? (p2.x - p1.x) / (p2.y - p1.y) : 0;
    return true;
}

// A triangle, its corners in the camera's space, into screen triangles -
// none, one, or the several that clipping makes of it.
#ifdef BENCH3D
uint32_t g_seen[4];  // for the bench: triangles seen, wholly outside a plane, wholly inside, cut
#endif

template <typename Make>
void addSeen(Vec3 c0, Vec3 c1, Vec3 c2, List<ScreenTri> &out, Make &&make) {
    // None of it drawn where every corner is outside the same side of the
    // view; drawn as it is where none is behind the eye or far off a side -
    // the drawing stops at the screen's edges - and otherwise cut, by only
    // the planes a corner is outside of: what is cut off one lies between
    // corners, inside the rest.
    unsigned o0 = outside(c0), o1 = outside(c1), o2 = outside(c2);
    unsigned w = (o0 | o1 | o2) == 0 ? 0 : outsideWide(c0) | outsideWide(c1) | outsideWide(c2);
#ifdef BENCH3D
    g_seen[0]++;
    g_seen[(o0 & o1 & o2) ? 1 : w == 0 ? 2 : 3]++;
#endif
    if (o0 & o1 & o2) {
        return;
    }
    if (w == 0) {
        ScreenTri t;
        if (setup(t, project(c0), project(c1), project(c2))) {
            make(t);
            out.push_back(t);
        }
        return;
    }
    Vec3 poly[12] = {c0, c1, c2};
    int n = 3;
    for (int plane = 0; plane < 5 && n >= 3; plane++) {
        if (w >> plane & 1) {
            n = clip(poly, n, plane);
        }
    }
    if (n < 3) {
        return;
    }
    Projected p[12];
    for (int i = 0; i < n; i++) {
        p[i] = project(poly[i]);
    }
    for (int i = 1; i + 1 < n; i++) {
        ScreenTri t;
        if (setup(t, p[0], p[i], p[i + 1])) {
            make(t);
            out.push_back(t);
        }
    }
}

template <typename Make>
void addTriangle(Vec3 a, Vec3 b, Vec3 c, List<ScreenTri> &out, Make &&make) {
    addSeen(toCamera(a), toCamera(b), toCamera(c), out, make);
}

// One known to be wholly in view - its cell is - straight to the screen,
// none of the clipping's tests: most of a big airfield's ground, seen from
// above.
template <typename Make>
void addWhole(Vec3 a, Vec3 b, Vec3 c, List<ScreenTri> &out, Make &&make) {
    ScreenTri t;
    if (setup(t, project(toCamera(a)), project(toCamera(b)), project(toCamera(c)))) {
        make(t);
        out.push_back(t);
    }
}

// Where a cell's corners on the grid are: from its middle's nearest point
// on the grid - in the camera's space and from the eye - each corner a few
// whole steps of it on, worked out exactly, rather than kilometres of them
// from the airfield's middle, which in a float would shake the ground by the
// eye by a fraction of a pixel.
struct GridBase {
    Vec3 seen, fromEye;
    int32_t qx, qy, qz;
};
HOT_INLINE GridBase gridBase(Vec3 middle, float unit) {
    GridBase b;
    b.qx = (int32_t)lrintf(middle.x / unit);
    b.qy = (int32_t)lrintf(middle.y / unit);
    b.qz = (int32_t)lrintf(middle.z / unit);
    Vec3 at = {b.qx * unit, b.qy * unit, b.qz * unit};
    b.fromEye = at - g.eye;
    b.seen = toCamera(at);
    return b;
}
HOT_INLINE Vec3 seenOnGrid(const GridBase &b, int32_t qx, int32_t qy, int32_t qz) {
    return b.seen + g.gridX * (float)(qx - b.qx) + g.gridY * (float)(qy - b.qy) + g.gridZ * (float)(qz - b.qz);
}

// A cell's triangle - its corners in the camera's space - put in place here
// for the thousands a frame: wholly in view, as its cell is, straight to the
// screen; in a cell only partly in view, first its corners tested against
// the view's sides - nothing more done where it is all off one of them - and
// cut where it crosses one. `make` - its colour - only for those kept.
template <typename Make>
HOT_INLINE void addInCell(Vec3 c0, Vec3 c1, Vec3 c2, bool whole, List<ScreenTri> &out, Make &&make) {
    if (!whole) {
        unsigned o0 = outside(c0), o1 = outside(c1), o2 = outside(c2);
        if (o0 & o1 & o2) {
            return;
        }
        if ((o0 | o1 | o2) && (outsideWide(c0) | outsideWide(c1) | outsideWide(c2))) {
            addSeen(c0, c1, c2, out, make);
            return;
        }
    }
    ScreenTri t;
    if (setup(t, project(c0), project(c1), project(c2))) {
        make(t);
        out.push_back(t);
    }
}

void addSeenLine(Vec3 a, Vec3 b, Rgb color, float distance, bool lit = false);

void addLine(const ModelLine &l) {
    addSeenLine(toCamera(l.a), toCamera(l.b), l.color, length(lerp(l.a, l.b, 0.5f) - g.eye));
}

// A line, its ends in the camera's space, hazed for being `distance` off - a
// strut unlit after dark, but `lit` paint as the light has it already.
void addSeenLine(Vec3 a, Vec3 b, Rgb lineColor, float distance, bool lit) {
    float t0 = 0, t1 = 1;
    for (int plane = 0; plane < 5; plane++) {
        float da = inside(plane, a), db = inside(plane, b);
        if (da < 0 && db < 0) {
            return;
        }
        if (da < 0) {
            t0 = std::max(t0, da / (da - db));
        } else if (db < 0) {
            t1 = std::min(t1, da / (da - db));
        }
    }
    if (t0 >= t1) {
        return;
    }
    Projected p0 = project(lerp(a, b, t0)), p1 = project(lerp(a, b, t1));
    ScreenLine s = {p0.x, p0.y, p0.iz, p1.x, p1.y, p1.iz, 0, 0, 0};
    s.top = std::max(0, floorInt(std::min(p0.y, p1.y)));
    s.bottom = std::min(g.h, floorInt(std::max(p0.y, p1.y)) + 1);
    if (s.top >= s.bottom) {
        return;
    }
    Rgb color = g.pal.lights > 0.5f && !lit ? scale(lineColor, g.pal.ambient * 1.6f) : lineColor;  // unlit, after dark
    s.color = pack(hazed(color, distance));
    g.wires.push_back(s);
}

// A disc on the screen round a point in the world, `size` metres across but
// never less than `minPx` pixels.
// The same from where it is in the camera's space.
bool addSpriteSeen(Vec3 c, float size, float minPx, ScreenSprite &s) {
    if (c.z < NEAR || ballInside(c) < -size) {
        return false;
    }
    Projected p = project(c);
    s.x = p.x;
    s.y = p.y;
    s.iz = p.iz;
    s.r = std::max(minPx, g.f * size / 2 / c.z);
    s.top = std::max(0, floorInt(p.y - s.r));
    s.bottom = std::min(g.h, ceilInt(p.y + s.r) + 1);
    return s.top < s.bottom && p.x + s.r >= 0 && p.x - s.r < g.w;
}

bool addSprite(Vec3 world, float size, float minPx, ScreenSprite &s) {
    return addSpriteSeen(toCamera(world), size, minPx, s);
}

// A light: a core of its colour, a pixel across at least, and a glow round it
// - wider when it shines at you - that haze dims less than it does colours,
// as lights carry further than what they light.
// How much haze a light `distance` off is under: less than a colour is.
float lightHaze(float distance) { return (1 - expf(-distance / (3 * g.pal.visibility))) * 0.8f; }
// A taxiway's lights as bright as they are near within TAXI_LIGHTS_FULL,
// metres; dimmer as the distance grows past it, fading out over the last
// TAXI_LIGHTS_FADE before TAXI_LIGHTS_GONE.
constexpr float TAXI_LIGHTS_FULL = 1200, TAXI_LIGHTS_FADE = 1500, TAXI_LIGHTS_GONE = 6000;

// From where it is in the camera's space, its haze worked out already - an
// airfield's lights, thousands of them, each a cell's.
void addLightSeen(Vec3 c, Rgb color, float size, float glow, float haze, List<ScreenSprite> &out,
                  float glowCapPx = 1e9f) {
    // Behind the eye or off to the side: nothing more to work out. Thousands
    // of a busy airfield's lights go this way.
    if (c.z < NEAR || fabsf(c.x) > c.z * g.gx + size || fabsf(c.y) > c.z * g.gy + size) {
        return;
    }
    Rgb seen = mix(color, g.pal.haze, haze);
    ScreenSprite s{};
    if (!addSpriteSeen(c, size, 0.5f, s)) {
        return;
    }
    // A pixel's worth, with no glow round it, as drawSprite() would draw it:
    // a dot - the airfield's, not an aircraft's, which go with its part.
    if (s.r < 0.75f && !(glow > 0 && s.r * glow >= 1.6f) && &out == &g.sprites) {
        int x = floorInt(s.x), y = floorInt(s.y);
        if (x >= 0 && x < g.w && y >= 0 && y < g.h) {
            g.dots.push_back({(int16_t)x, (int16_t)y, (int16_t)(y + 1), pack(seen), s.iz});
        }
        return;
    }
    s.blend = Blend::SOLID;
    s.color = pack(seen);
    out.push_back(s);
    // A glow round a light only where the light is big enough to have one -
    // a far one is a point, and the glows of a thousand points cost more
    // than they show.
    if (glow > 0 && s.r * glow >= 1.6f) {
        s.r = std::min(glowCapPx, s.r * glow);
        s.top = std::max(0, floorInt(s.y - s.r));
        s.bottom = std::min(g.h, ceilInt(s.y + s.r) + 1);
        s.blend = Blend::GLOW;
        float k = 0.55f * (1 - haze);
        s.add[0] = seen.r * k * 31 / 255;
        s.add[1] = seen.g * k * 63 / 255;
        s.add[2] = seen.b * k * 31 / 255;
        out.push_back(s);
    }
}

// An airfield's or a town's light from where it is in the camera's space: a
// dot where it comes to a pixel with no glow round it - most of them after
// dark, from any way off - with no more worked out for it than where its
// pixel is, and `packed()` its colour hazed; otherwise as addLightSeen().
template <typename Packed>
HOT_INLINE void addFieldLight(Vec3 c, Rgb color, Packed &&packed, float size, float glow, float haze) {
    if (c.z < NEAR || fabsf(c.x) > c.z * g.gx + size || fabsf(c.y) > c.z * g.gy + size) {
        return;
    }
    float r = std::max(0.5f, g.f * size / 2 / c.z);
    if (r < 0.75f && !(glow > 0 && r * glow >= 1.6f)) {
        float iz = 1 / c.z;
        int x = floorInt(g.cx + g.f * c.x * iz), y = floorInt(g.cy + g.f * c.y * iz);
        if (x >= 0 && x < g.w && y >= 0 && y < g.h) {
            g.dots.push_back({(int16_t)x, (int16_t)y, (int16_t)(y + 1), packed(), iz});
        }
        return;
    }
    addLightSeen(c, color, size, glow, haze, g.sprites);
}

// The same from where it is in the world - its haze, `haze` under nought,
// worked out for it.
void addLight(Vec3 world, Rgb color, float size, float glow, List<ScreenSprite> &out, float glowCapPx = 1e9f,
              float haze = -1) {
    Vec3 d = world - g.eye;
    Vec3 c = {dot(d, g.right), dot(d, g.down), dot(d, g.forward)};
    if (c.z < NEAR || fabsf(c.x) > c.z * g.gx + size || fabsf(c.y) > c.z * g.gy + size) {
        return;  // before any haze is worked out
    }
    addLightSeen(c, color, size, glow, haze < 0 ? lightHaze(length(d)) : haze, out, glowCapPx);
}

// --- drawing a band ---------------------------------------------------------------

// Calls span(y, x0, x1) for each row of the triangle in [y0, y1), for the
// pixels [x0, x1) whose centres it covers.
template <typename Span>
inline void raster(const ScreenTri &t, int y0, int y1, Span &&span) {
    int ys = std::max(y0, (int)t.top), ye = std::min(y1, (int)t.bottom);
    for (int y = ys; y < ye; y++) {
        float sy = y + 0.5f;
        float xa = t.ax + (sy - t.ay) * t.dAC;
        float xb = sy < t.by ? t.ax + (sy - t.ay) * t.dAB : t.bx + (sy - t.by) * t.dBC;
        if (xa > xb) {
            std::swap(xa, xb);
        }
        int xs = std::max(0, ceilInt(xa - 0.5f)), xe = std::min(d.w, ceilInt(xb - 0.5f));
        if (xs < xe) {
            span(y, xs, xe);
        }
    }
}

#if defined(ESP_PLATFORM)
// The innermost loops with the P4's PIE vector instructions - see
// scene3d_pie.S - eight pixels to an instruction. scene3d_span8 is span()'s:
// `c` and `z` are where pixel x0 is, which is a multiple of 8; lane i's depth
// is (at >> 16) + ramp[i] each chunk, `at` going on by eight steps a chunk.
// scene3d_fill8 is fill()'s, from a 16-byte boundary.
extern "C" void scene3d_span8(uint16_t *c, uint16_t *z, int chunks, int32_t at, int32_t step8,
                              const int16_t *ramp, const uint16_t *color);
extern "C" void scene3d_fill8(uint16_t *row, int chunks, const uint32_t *pattern);
extern "C" void scene3d_darken8(uint16_t *c, const uint16_t *m, int chunks, const uint16_t *constants);
extern "C" void scene3d_tint8(uint16_t *c, const uint16_t *z, int chunks, int32_t at, int32_t step8,
                              const uint16_t *constants, const uint16_t *head, const uint16_t *tail);
bool g_simd = true;
#endif

// Pixels [xs, xe) of a row - which starts on a 16-byte boundary - in a pair
// of colours, the low half for even x.
inline void fill(uint16_t *row, int xs, int xe, uint32_t pair) {
    if (xs >= xe) {
        return;
    }
    if (xs & 1) {
        row[xs++] = (uint16_t)(pair >> 16);
    }
#if defined(ESP_PLATFORM)
    // Long enough to be worth it: two at a time to a 16-byte boundary, then
    // eight at a time.
    if (g_simd && xe - xs >= 32) {
        while ((xs & 7) != 0) {
            *(uint32_t *)(row + xs) = pair;
            xs += 2;
        }
        int chunks = (xe - xs) / 8;
        alignas(16) uint32_t pattern[4] = {pair, pair, pair, pair};
        scene3d_fill8(row + xs, chunks, pattern);
        xs += chunks * 8;
    }
#endif
    uint32_t *q = (uint32_t *)(row + xs);
    int words = (xe - xs) / 2;
    for (int i = 0; i < words; i++) {
        q[i] = pair;
    }
    if ((xe - xs) & 1) {
        row[xe - 1] = (uint16_t)pair;
    }
}

inline uint16_t darken(uint16_t c) {
    c = swapIf(c);
    int k = d.shadow256;
    c = (uint16_t)(((c >> 11) * k >> 8) << 11 | ((c >> 5 & 63) * k >> 8) << 5 | ((c & 31) * k >> 8));
    return swapIf(c);
}

// Pixels [xs, xe) of a row darkened where its mask row `m` isn't nought - with
// PIE, high byte first, eight at a time from a multiple of 8, the sums the
// same: see scene3d_pie.S.
inline void darkenRow(uint16_t *c, const uint16_t *m, int xs, int xe) {
#if defined(ESP_PLATFORM)
    if (g_simd && d.swap && xe - xs >= 16) {
        for (int x8 = (xs + 7) & ~7; xs < x8; xs++) {
            if (m[xs]) {
                c[xs] = darken(c[xs]);
            }
        }
        int chunks = (xe - xs) / 8;
        scene3d_darken8(c + xs, m + xs, chunks, &d.shadowLanes[0][0]);
        xs += chunks * 8;
    }
#endif
    for (; xs < xe; xs++) {
        if (m[xs]) {
            c[xs] = darken(c[xs]);
        }
    }
}

void drawLine(const ScreenLine &l, int y0, int y1, uint16_t *color, uint16_t *depth) {
    int w = d.w;
    float dx = l.x1 - l.x0, dy = l.y1 - l.y0;
    auto plot = [&](int y, int x, float iz) {
        if (x < 0 || x >= w || y < y0 || y >= y1) {
            return;
        }
        size_t i = (size_t)(y - y0) * w + x;
        if (depthOf(iz) >= depth[i]) {
            color[i] = l.color;
        }
    };
    if (fabsf(dx) >= fabsf(dy)) {
        if (dx == 0) {
            plot(floorInt(l.y0), floorInt(l.x0), l.iz0);
            return;
        }
        // A pixel a column - only the columns where it is in these rows.
        float t0 = 0, t1 = 1;
        if (dy != 0) {
            float ta = (y0 - 1 - l.y0) / dy, tb = (y1 + 1 - l.y0) / dy;
            t0 = std::max(0.0f, std::min(ta, tb));
            t1 = std::min(1.0f, std::max(ta, tb));
        }
        float xa = l.x0 + t0 * dx, xb = l.x0 + t1 * dx;
        if (xa > xb) {
            std::swap(xa, xb);
        }
        int cs = std::max(0, ceilInt(xa - 0.5f)), ce = std::min(w, ceilInt(xb - 0.5f));
        for (int x = cs; x < ce; x++) {
            float t = (x + 0.5f - l.x0) / dx;
            plot(floorInt(l.y0 + t * dy), x, l.iz0 + t * (l.iz1 - l.iz0));
        }
    } else {
        int ys = std::max(y0, ceilInt(std::min(l.y0, l.y1) - 0.5f));
        int ye = std::min(y1, ceilInt(std::max(l.y0, l.y1) - 0.5f));
        for (int y = ys; y < ye; y++) {
            float t = (y + 0.5f - l.y0) / dy;
            plot(y, floorInt(l.x0 + t * dx), l.iz0 + t * (l.iz1 - l.iz0));
        }
    }
}

void drawSprite(const ScreenSprite &s, int y0, int y1, uint16_t *color, uint16_t *depth) {
    int w = d.w;
    // Which of each two by two pixels see-through smoke covers, in the order
    // it takes them: of 4, the first `coverage`.
    static const uint8_t RANK[4] = {0, 2, 3, 1};
    if (s.r < 0.75f) {
        int x = floorInt(s.x), y = floorInt(s.y);  // a pixel's worth: just the one
        if (x >= 0 && x < w && y >= y0 && y < y1) {
            if (depthOf(s.iz) >= depth[(size_t)(y - y0) * w + x]) {
                color[(size_t)(y - y0) * w + x] = s.color;
            }
        }
        return;
    }
    int ys = std::max(y0, s.top), ye = std::min(y1, s.bottom);
    float r2 = s.r * s.r;
    uint16_t d = depthOf(s.iz);
    for (int y = ys; y < ye; y++) {
        float dy = y + 0.5f - s.y, h2 = r2 - dy * dy;
        if (h2 <= 0) {
            continue;
        }
        float half = sqrtf(h2);
        int xs = std::max(0, ceilInt(s.x - half - 0.5f)), xe = std::min(w, ceilInt(s.x + half - 0.5f));
        uint16_t *c = color + (size_t)(y - y0) * w;
        uint16_t *z = depth + (size_t)(y - y0) * w;
        for (int x = xs; x < xe; x++) {
            if (d < z[x]) {
                continue;
            }
            switch (s.blend) {
            case Blend::SOLID:
                c[x] = s.color;
                break;
            case Blend::STIPPLE:
                if (RANK[(y & 1) * 2 + (x & 1)] < s.coverage) {
                    c[x] = s.color;
                }
                break;
            case Blend::TINT: {
                float dx = x + 0.5f - s.x;
                int k = (int)(s.coverage * 32 * (1 - (dx * dx + dy * dy) / r2));
                uint16_t v = swapIf(c[x]), t = swapIf(s.color);
                int r = v >> 11, gr = v >> 5 & 63, b = v & 31;
                r += ((t >> 11) - r) * k >> 8;
                gr += ((t >> 5 & 63) - gr) * k >> 8;
                b += ((t & 31) - b) * k >> 8;
                c[x] = swapIf((uint16_t)(r << 11 | gr << 5 | b));
                break;
            }
            case Blend::GLOW: {
                float dx = x + 0.5f - s.x, a = 1 - (dx * dx + dy * dy) / r2;
                a *= a;
                uint16_t v = swapIf(c[x]);
                int r = std::min(31, (v >> 11) + (int)(s.add[0] * a)),
                    gr = std::min(63, (v >> 5 & 63) + (int)(s.add[1] * a)),
                    b = std::min(31, (v & 31) + (int)(s.add[2] * a));
                c[x] = swapIf((uint16_t)(r << 11 | gr << 5 | b));
                break;
            }
            }
        }
    }
}

size_t colorBytes(int w, int rows) { return ((size_t)w * rows * 2 + 63) & ~(size_t)63; }

// A solid's span: pixels [xs, xe) of a row, its depth starting at `at` and
// going `step` a pixel, both 16.16; where it is nearer than what is there, its
// depth and colour written. Summed unsigned: on a steep, near face the sum a
// step past the span's end - never used - can pass what an int holds.
inline void spanC(uint16_t *c, uint16_t *z, int xs, int xe, int32_t at, int32_t step, uint16_t color) {
    uint32_t sum = (uint32_t)at, by = (uint32_t)step;
    for (int x = xs; x < xe; x++, sum += by) {
        uint16_t d = (uint16_t)(sum >> 16);
        if (d > z[x]) {
            z[x] = d;
            c[x] = color;
        }
    }
}


// A see-through triangle's spans' constants, worked out once for the
// triangle: its depth's step along a row, as a solid's, and in each of a
// chunk of eight pixels, the ramp; the eighths of what is behind kept,
// 8 - e; and the tint's share of each channel, e/8 of it rounded down, in
// place - see tintC(). In lanes, for PIE: the ramp, 8 - e, the masks of
// red, green and blue, and the share.
struct Tinter {
    int32_t step;
    uint16_t keep, share;
    alignas(16) uint16_t constants[6][8];
    Tinter(int32_t step, uint16_t tint, int eighths) : step(step) {
        keep = (uint16_t)(8 - eighths);
        // The tint's share of each channel rounded so that, with what the
        // pixel keeps of itself rounded down, the two come out right on
        // average - rounded down too, they came out three quarters of a step
        // dark, and over a night sky that took the five-bit red and blue to
        // nothing before the six-bit green: a trail tinted it green. Never
        // past a channel's top, so nothing carries into the next.
        auto part = [eighths](uint32_t c) { return (2 * c * (uint32_t)eighths + 15) >> 4; };
        share = (uint16_t)(part(tint >> 11) << 11 | part((tint >> 5) & 63) << 5 | part(tint & 31));
        for (int i = 0; i < 8; i++) {
            constants[0][i] = (uint16_t)(int16_t)((int64_t)step * i >> 16);
            constants[1][i] = keep;
            constants[2][i] = 0xF800;
            constants[3][i] = 0x07E0;
            constants[4][i] = 0x001F;
            constants[5][i] = share;
        }
    }
};

// For a span's first chunk of eight, the lanes before it - all ones - and
// for its last, those after it: left as they are.
struct ChunkEdges {
    alignas(16) uint16_t before[8][8], after[9][8];
    ChunkEdges() {
        for (int s = 0; s < 8; s++) {
            for (int i = 0; i < 8; i++) {
                before[s][i] = i < s ? 0xFFFF : 0;
            }
        }
        for (int e = 0; e <= 8; e++) {
            for (int i = 0; i < 8; i++) {
                after[e][i] = i >= e ? 0xFFFF : 0;
            }
        }
    }
};
const ChunkEdges g_chunkEdges;

// Pixels [xs, xe) of a row seen through a see-through triangle where it
// isn't behind what is drawn: each channel keeps (8 - e)/8 of itself, in
// place, rounded down, and takes the tint's share; the depth left as it
// was. Its depth as scene3d_tint8 has it, a chunk of eight at a time from
// x8, xs's multiple of 8 - `base` there in 16.16 fixed point, going on by
// eight steps a chunk, and the ramp's share in each - so the two are the
// same, pixel for pixel.
inline void tintC(uint16_t *c, const uint16_t *z, int xs, int xe, int x8, int32_t base, const Tinter &t) {
    for (int x = xs; x < xe; x++) {
        int chunk = (x - x8) >> 3, lane = (x - x8) & 7;
        int32_t at = (int32_t)((uint32_t)base + (uint32_t)t.step * 8u * (uint32_t)chunk);
        int depth = std::max(-32768, std::min(32767, (int)(int16_t)(at >> 16) + (int16_t)t.constants[0][lane]));
        if (depth < (int16_t)z[x]) {
            continue;
        }
        uint32_t v = d.swap ? (uint16_t)(c[x] >> 8 | c[x] << 8) : c[x], k = t.keep;
        uint16_t tinted = (uint16_t)(((((v & 0xF800) * k) >> 3) & 0xF800) + ((((v & 0x07E0) * k) >> 3) & 0x07E0) +
                                     ((((v & 0x001F) * k) >> 3) & 0x001F) + t.share);
        c[x] = d.swap ? (uint16_t)(tinted >> 8 | tinted << 8) : tinted;
    }
}

// The same, `at` the depth at xs: every chunk of it with PIE, the lanes
// outside the span left as they are.
inline void tint(uint16_t *c, const uint16_t *z, int xs, int xe, int32_t at, const Tinter &t) {
    if (xs >= xe) {
        return;
    }
    int x8 = xs & ~7;
    int32_t base = (int32_t)((uint32_t)at - (uint32_t)t.step * (uint32_t)(xs - x8));
#if defined(ESP_PLATFORM)
    if (g_simd && d.swap) {
        int chunks = (xe - x8 + 7) >> 3;
        scene3d_tint8(c + x8, z + x8, chunks, base, (int32_t)((uint32_t)t.step * 8u), &t.constants[0][0],
                      g_chunkEdges.before[xs - x8], g_chunkEdges.after[xe - (x8 + 8 * (chunks - 1))]);
        return;
    }
#endif
    tintC(c, z, xs, xe, x8, base, t);
}

// A solid triangle's spans' constants, worked out once for the triangle
// rather than once a span - a close-up's tens of thousands of them a frame:
// its colour, its depth's step along a row and, for PIE, those in each of
// eight lanes.
struct Spanner {
    int32_t step;
    uint16_t color;
#if defined(ESP_PLATFORM)
    alignas(16) int16_t ramp[8];
    alignas(16) uint16_t colors[8];
#endif
    Spanner(int32_t step, uint16_t color) : step(step), color(color) {
#if defined(ESP_PLATFORM)
        for (int i = 0; i < 8; i++) {
            ramp[i] = (int16_t)((int64_t)step * i >> 16);
            colors[i] = color;
        }
#endif
    }
};

#ifdef BENCH3D
// For the bench, a core's own: solids' spans counted by how long they are -
// under 8 pixels, 8 to 23, 24 and more, which PIE draws - and their pixels.
uint32_t g_spanStats[2][6];
#endif

inline void span(uint16_t *c, uint16_t *z, int xs, int xe, int32_t at, const Spanner &s) {
#ifdef BENCH3D
    {
        int n = xe - xs, bucket = n < 8 ? 0 : n < 24 ? 1 : 2;
        uint32_t *st = g_spanStats[xPortGetCoreID() & 1];
        st[bucket]++;
        st[3 + bucket] += (uint32_t)n;
    }
#endif
#if defined(ESP_PLATFORM)
    // Long enough to be worth it: the pixels to the next multiple of 8 one
    // at a time, the rest eight at a time, and what is left over.
    if (g_simd && xe - xs >= 24) {
        int x8 = (xs + 7) & ~7;
        spanC(c, z, xs, x8, at, s.step, s.color);
        at = (int32_t)((uint32_t)at + (uint32_t)s.step * (uint32_t)(x8 - xs));
        int chunks = (xe - x8) / 8;
        scene3d_span8(c + x8, z + x8, chunks, at, (int32_t)((uint32_t)s.step * 8u), s.ramp, s.colors);
        int done = x8 + chunks * 8;
        spanC(c, z, done, xe, (int32_t)((uint32_t)at + (uint32_t)s.step * (uint32_t)(done - x8)), s.step, s.color);
        return;
    }
#endif
    spanC(c, z, xs, xe, at, s.step, s.color);
}

// A solid triangle's rows in a band, as a trapezoid - its top corner to
// its middle one, or the middle to the bottom: a left and a right edge
// stepped down a row at a time in 16.16 fixed point, the half pixel and the
// rounding up folded in, so a row's first pixel is xl >> 16 and the one
// after its last xr >> 16; and its depth at x nought, in float, stepped
// down too, the depth at a row's first pixel fused-multiplied from it.
// scene3d_solid's - see scene3d_pie.S, which has the fields where they are.
struct Trapezoid {
    uint16_t *c, *z;  // row 0's colours and depths, at x nought
    int32_t rows, w;
    int32_t xl, dl, xr, dr;
    float zrow, dzrow, zx;
    int32_t step;  // the depth's a pixel, 16.16
    uint32_t color;
    const int16_t *ramp;  // PIE's - see Spanner
    const uint16_t *colors;
    int32_t simd;
};
#if defined(ESP_PLATFORM)  // the PC's pointers are twice as long, and it has no scene3d_solid
static_assert(offsetof(Trapezoid, xl) == 16 && offsetof(Trapezoid, zrow) == 32 && offsetof(Trapezoid, step) == 44 &&
                  offsetof(Trapezoid, ramp) == 52 && offsetof(Trapezoid, simd) == 60,
              "scene3d_solid has the fields where they were");
#endif

// The C of scene3d_solid, which is this exactly.
void solidRowsC(const Trapezoid &tr) {
    uint16_t *c = tr.c, *z = tr.z;
    int32_t xl = tr.xl, xr = tr.xr;
    float zrow = tr.zrow;
    for (int r = 0; r < tr.rows; r++, c += tr.w, z += tr.w) {
        int xs = (int)std::max<int32_t>(0, xl >> 16), xe = (int)std::min<int32_t>(tr.w, xr >> 16);
        if (xs < xe) {
            int32_t at = std::max<int32_t>(0, (int32_t)fmaf(tr.zx, (float)xs, zrow));
#if defined(ESP_PLATFORM)
            if (tr.simd && xe - xs >= 24) {
                int x8 = (xs + 7) & ~7;
                spanC(c, z, xs, x8, at, tr.step, (uint16_t)tr.color);
                at = (int32_t)((uint32_t)at + (uint32_t)tr.step * (uint32_t)(x8 - xs));
                int chunks = (xe - x8) / 8;
                scene3d_span8(c + x8, z + x8, chunks, at, (int32_t)((uint32_t)tr.step * 8u), tr.ramp, tr.colors);
                int done = x8 + chunks * 8;
                spanC(c, z, done, xe, (int32_t)((uint32_t)at + (uint32_t)tr.step * 8u * (uint32_t)chunks), tr.step,
                      (uint16_t)tr.color);
            } else
#endif
            {
                spanC(c, z, xs, xe, at, tr.step, (uint16_t)tr.color);
            }
        }
        xl += tr.dl;
        xr += tr.dr;
        zrow += tr.dzrow;
    }
}

#if defined(ESP_PLATFORM)
extern "C" void scene3d_solid(const Trapezoid *t);
#endif

// A triangle's rows from y0 to y1 as its two trapezoids - its top corner to
// its middle one, and the middle to the bottom - each passed to `trapezoid`
// with its first row, its rows, and its left and right edges in 16.16 fixed
// point, the half pixel and the rounding up folded in, so a row's first
// pixel is xl >> 16 and the one after its last xr >> 16, stepping dl and dr
// a row; or, an edge so far off the screen or so near flat that 16.16 can't
// hold it, to `floating` with its first row and the one after its last, to
// be drawn as raster() has it.
template <typename Trap, typename Floating>
void trapezoids(const ScreenTri &t, int y0, int y1, Trap &&trapezoid, Floating &&floating) {
    int ys = std::max(y0, (int)t.top), ye = std::min(y1, (int)t.bottom);
    if (ys >= ye) {
        return;
    }
    int ymid = std::max(ys, std::min(ye, ceilInt(t.by - 0.5f)));
    for (int part = 0; part < 2; part++) {
        int ya = part == 0 ? ys : ymid, yb = part == 0 ? ymid : ye;
        if (ya >= yb) {
            continue;
        }
        float sy = ya + 0.5f, sa = t.dAC, sb = part == 0 ? t.dAB : t.dBC;
        float xa = t.ax + (sy - t.ay) * sa, xb = part == 0 ? t.ax + (sy - t.ay) * sb : t.bx + (sy - t.by) * sb;
        // Which side is which: the long edge's or the other's, half way down.
        float half = (yb - ya - 1) * 0.5f;
        bool aLeft = xa + sa * half <= xb + sb * half;
        float xl = aLeft ? xa : xb, xr = aLeft ? xb : xa, dl = aLeft ? sa : sb, dr = aLeft ? sb : sa;
        float last = (float)(yb - ya);
        if (!(fabsf(xl) < 30000 && fabsf(xr) < 30000 && fabsf(dl) < 16384 && fabsf(dr) < 16384 &&
              fabsf(xl + dl * last) < 30000 && fabsf(xr + dr * last) < 30000)) {
            floating(ya, yb);
            continue;
        }
        trapezoid(ya, yb - ya, floorInt((xl - 0.5f) * 65536.0f) + 65535, (int32_t)lroundf(dl * 65536.0f),
                  floorInt((xr - 0.5f) * 65536.0f) + 65535, (int32_t)lroundf(dr * 65536.0f));
    }
}

// Solid triangle t's rows in the band from y0 to y1, into its colours and
// depths - see trapezoids() and scene3d_solid.
void solidRows(const ScreenTri &t, uint16_t *color, uint16_t *depth, int y0, int y1, float fixed,
               const Spanner &spanner) {
    trapezoids(
        t, y0, y1,
        [&](int ya, int rows, int32_t xl, int32_t dl, int32_t xr, int32_t dr) {
            Trapezoid tr;
            tr.c = color + (size_t)(ya - y0) * d.w;
            tr.z = depth + (size_t)(ya - y0) * d.w;
            tr.rows = rows;
            tr.w = d.w;
            tr.xl = xl;
            tr.dl = dl;
            tr.xr = xr;
            tr.dr = dr;
            tr.zrow = (t.za * 0.5f + t.zb * (ya + 0.5f) + t.zc) * fixed;
            tr.dzrow = t.zb * fixed;
            tr.zx = t.za * fixed;
            tr.step = spanner.step;
            tr.color = spanner.color;
#if defined(ESP_PLATFORM)
            tr.ramp = spanner.ramp;
            tr.colors = spanner.colors;
            tr.simd = g_simd;
            if (g_simd) {
                scene3d_solid(&tr);
                return;
            }
#else
            tr.ramp = nullptr;
            tr.colors = nullptr;
            tr.simd = 0;
#endif
            solidRowsC(tr);
        },
        [&](int ya, int yb) {
            raster(t, ya, yb, [&](int y, int xs, int xe) {
                int32_t from = std::max<int32_t>(0, (int32_t)((t.za * (xs + 0.5f) + t.zb * (y + 0.5f) + t.zc) * fixed));
                span(color + (size_t)(y - y0) * d.w, depth + (size_t)(y - y0) * d.w, xs, xe, from, spanner);
            });
        });
}

// The ground's triangle t's rows in the band from y0 to y1, each in its
// colour hazed as the row is - worked out again only where the row's haze
// isn't the last row's of the same parity, which down most of a triangle
// it is.
void groundRows(const ScreenTri &t, uint16_t *color, int y0, int y1, const int haze[3]) {
    int lastHaze[2] = {-1, -1};
    uint32_t lastPair[2] = {0, 0};
    auto row = [&](int y, int xs, int xe) {
        int hz = d.rowHaze[y], p = y & 1;
        if (hz != lastHaze[p]) {
            lastHaze[p] = hz;
            lastPair[p] = pairInt(t.lit, haze, hz, y);
        }
        fill(color + (size_t)(y - y0) * d.w, xs, xe, lastPair[p]);
    };
    trapezoids(
        t, y0, y1,
        [&](int ya, int rows, int32_t xl, int32_t dl, int32_t xr, int32_t dr) {
            for (int y = ya; y < ya + rows; y++, xl += dl, xr += dr) {
                int xs = (int)std::max<int32_t>(0, xl >> 16), xe = (int)std::min<int32_t>(d.w, xr >> 16);
                if (xs < xe) {
                    row(y, xs, xe);
                }
            }
        },
        [&](int ya, int yb) { raster(t, ya, yb, row); });
}

const Design &designOf(const Look &look) { return designs()[std::min((int)look.family, (int)Family::COUNT - 1)]; }

Rgb rgbOf(uint32_t c) { return {(float)(c >> 16 & 255), (float)(c >> 8 & 255), (float)(c & 255)}; }

// v turned about the line through `origin` along `axis`, by the angle whose
// cosine and sine are c and s.
#if defined(BENCH3D) && defined(ESP_PLATFORM)
// The one followed, each core's: cycles placing its corners, and lighting
// and setting up its faces; then its corners, its faces, and the triangles
// they made.
uint64_t g_aircraftStats[2][6];
#endif

Vec3 turnAbout(Vec3 v, Vec3 origin, Vec3 axis, float c, float s) {
    Vec3 p = v - origin;
    return origin + p * c + cross(axis, p) * s + axis * (dot(axis, p) * (1 - c));
}

// An aircraft into a part's lists: its faces at its detail, its shadow, its
// lights.
void addAircraft(const Placed &p, Part &out) {
    const Design &dsn = designOf(p.look);
    const Palette &pal = g.pal;
    const Scene &scene = g.scene;
    Placer at(p.pose, p.look.scale);
    Rgb livery = rgbOf(p.look.tail), body = rgbOf(p.look.body), belly = rgbOf(p.look.belly),
        engines = rgbOf(p.look.engines), wings = rgbOf(p.look.wings);
    out.counts.aircraftFull += p.detail <= DETAIL_MEDIUM;
    out.counts.aircraftSimple += p.detail == DETAIL_SIMPLE;
    out.counts.aircraftTiny += p.detail == DETAIL_TINY;
    out.counts.aircraftDots += p.detail == DETAIL_LIGHTS;
    if (p.detail < DETAIL_LIGHTS) {
        // Each corner placed and seen once - those of a control turned on its
        // hinge, and of a propeller or rotor round its shaft, first; then each
        // face facing the eye lit - the haze, which is all but the same over
        // the whole of it, worked out once for the lot.
        const Mesh &mesh = dsn.lod[p.detail];
        size_t n = mesh.cornerCount();
        out.world.resize(n);
        out.cam.resize(n);
        out.proj.resize(n);
        float angle[MOVE_SPIN + MAX_SPINNERS] = {};
        // Each angle's cosine and sine, once: not again for each of the
        // hundreds of corners a control or a propeller has.
        float cosine[MOVE_SPIN + MAX_SPINNERS], sine[MOVE_SPIN + MAX_SPINNERS];
        if (mesh.moving) {
            const Config &c = p.config;
            const float command[HINGES] = {0,         c.aileron, c.aileron, c.elevator,      c.elevator,
                                           c.rudder,  c.rudder,  c.elevator, c.spoilerDeploy, c.spoilerDeploy};
            for (int h = 1; h < HINGES; h++) {
                angle[h] = dsn.hinge[h].travel * command[h];
            }
            // Each aircraft's propellers somewhere of their own in their turn,
            // so a row of them parked doesn't stand in step.
            float phase = (p.index + 2) * 1.7f;
            for (int i = 0; i < dsn.spinnerCount; i++) {
                angle[MOVE_SPIN + i] =
                    p.config.turning ? fmodf(scene.seconds * dsn.spinners[i].rate + phase + i, 2 * PI) : phase + i;
            }
            for (int m = 1; m < MOVE_SPIN + dsn.spinnerCount; m++) {
                cosine[m] = cosf(angle[m]);
                sine[m] = sinf(angle[m]);
            }
        }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
        uint64_t *astats = g_aircraftStats[xPortGetCoreID() & 1];
        uint32_t aclock = esp_cpu_get_cycle_count();
        if (p.followed) {
            astats[3] += n;
            astats[4] += mesh.faceCount();
        }
#endif
        for (size_t v = 0; v < n; v++) {
            Vec3 corner = mesh.corner(v);
            uint8_t move = mesh.moves[v];
            if (move != MOVE_NONE && angle[move] != 0) {
                if (move >= MOVE_SPIN) {
                    const Spinner &s = dsn.spinners[move - MOVE_SPIN];
                    corner = turnAbout(corner, s.hub, s.axis, cosine[move], sine[move]);
                } else {
                    const Hinge &h = dsn.hinge[move];
                    corner = turnAbout(corner, h.origin, h.axis, cosine[move], sine[move]);
                }
            }
            out.world[v] = at.place(corner);
            out.cam[v] = toCamera(out.world[v]);
            out.nearest = std::min(out.nearest, out.cam[v].z);
            if (p.whole) {
                out.proj[v] = project(out.cam[v]);
            }
        }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
        size_t madeBefore = out.solid.size() + out.clear.size();
        if (p.followed) {
            uint32_t c = esp_cpu_get_cycle_count();
            astats[0] += c - aclock;
            aclock = c;
        }
#endif
        float haze = 1 - expf(-p.distance / pal.visibility);
        for (size_t f = 0; f < mesh.faceCount(); f++) {
            uint8_t show = mesh.show(f);
            if ((show == SHOW_GEAR && !p.config.gear) || (show == SHOW_FLAPS && !p.config.flaps) ||
                (show == SHOW_SPOILERS && !p.config.spoilers) || (show == SHOW_TURNING && !p.config.turning)) {
                continue;  // stowed, or stopped
            }
            const uint16_t *i = &mesh.faces[f * 3];
            uint8_t blur = mesh.blur(f);
            // A blade's facing from where its corners now are: it turns right
            // round. Any other's as built - a control's few degrees of travel
            // don't change which way it faces.
            uint8_t moved = mesh.moves[i[0]];
            Vec3 normal = moved >= MOVE_SPIN || moved == MOVE_SPOILER_L || moved == MOVE_SPOILER_R
                              ? normalize(cross(out.world[i[1]] - out.world[i[0]], out.world[i[2]] - out.world[i[0]]))
                              : at.turn(mesh.normal(f));
            float facing = dot(normal, out.world[i[0]] - g.eye);
            if (blur == 0 && facing >= 0) {
                continue;  // facing away
            }
            Rgb base;
            switch (mesh.paint(f)) {
            case PAINT_BODY: base = body; break;
            case PAINT_BELLY: base = belly; break;
            case PAINT_LIVERY: base = livery; break;
            case PAINT_ENGINES: base = engines; break;
            case PAINT_WINGS: base = wings; break;
            case PAINT_CABIN: base = p.look.windows ? DARK : body; break;
            default: base = g_palette[mesh.colors[f]]; break;
            }
            // A blur so near it is a good part of the view - the eye in a
            // rotor's disc, or as good as - left out: it would be a haze over
            // everything, and its blending, pixel by pixel, took a Chinook's
            // frames to a quarter of a second each.
            if (blur > 0) {
                if (!p.whole) {
                    continue;
                }
                const Projected &a = out.proj[i[0]], &b = out.proj[i[1]], &c = out.proj[i[2]];
                if (fabsf((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) > 0.08f * g.w * g.h) {
                    continue;
                }
            }
            // A see-through face is lit on whichever side the eye sees.
            float sunward = dot(normal, g.sun) * (blur > 0 && facing > 0 ? -1.0f : 1.0f);
            float light = pal.ambient + pal.diffuse * std::max(0.0f, sunward);
            uint16_t color = pack(mix(scale(base, std::max(light, pal.floodlit)), pal.haze, haze));
            List<ScreenTri> &list = blur > 0 ? out.clear : out.solid;
            if (p.whole) {
                ScreenTri t;
                if (setup(t, out.proj[i[0]], out.proj[i[1]], out.proj[i[2]])) {
                    t.color = color;
                    t.lit[0] = blur;
                    list.push_back(t);
                }
            } else {
                addSeen(out.cam[i[0]], out.cam[i[1]], out.cam[i[2]], list, [&](ScreenTri &s) {
                    s.color = color;
                    s.lit[0] = blur;
                });
            }
        }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
        if (p.followed) {
            uint32_t c = esp_cpu_get_cycle_count();
            astats[1] += c - aclock;
            aclock = c;
            astats[5] += out.solid.size() + out.clear.size() - madeBefore;
        }
#endif
        // Its shadow, while it is near enough the ground to see it by. The
        // one followed: the simple model's faces towards the sun, flattened
        // onto the ground along its rays - together, the outline the sun
        // can't get through. Any other: a blob, its body and wings as three
        // flat quads laid down the same way - six faces, where a busy
        // apron's worth of outlines would be thousands.
        if (g.shadows && p.pose.at.z < 250 && !p.followed) {
            for (const auto &quad : dsn.blob) {
                Vec3 c[4];
                for (int k = 0; k < 4; k++) {
                    Vec3 w = at.place(quad[k]);
                    c[k] = toCamera(w - g.sun * (w.z / g.sun.z));
                }
                addSeen(c[0], c[1], c[2], out.shadow, [](ScreenTri &) {});
                addSeen(c[0], c[2], c[3], out.shadow, [](ScreenTri &) {});
            }
        }
        if (g.shadows && p.pose.at.z < 250 && p.followed) {
            uint32_t s0 = now();
            const Mesh &simple = dsn.lod[DETAIL_SIMPLE];
            out.cam.resize(simple.cornerCount());
            for (size_t v = 0; v < simple.cornerCount(); v++) {
                Vec3 w = at.place(simple.corner(v));
                out.cam[v] = toCamera(w - g.sun * (w.z / g.sun.z));
            }
            for (size_t f = 0; f < simple.faceCount(); f++) {
                // Not a blur's: the light comes through it.
                if (simple.blur(f) == 0 && dot(at.turn(simple.normal(f)), g.sun) > 0) {
                    const uint16_t *i = &simple.faces[f * 3];
                    addSeen(out.cam[i[0]], out.cam[i[1]], out.cam[i[2]], out.shadow, [](ScreenTri &) {});
                }
            }
            out.counts.stepUs[STEP_SHADOWS] += now() - s0;
        }
    }
    // Its trails, high up where the air is cold enough to freeze its
    // exhaust: from each engine back along its path, a ribbon facing the eye
    // that spreads and thins over three kilometres or so - a core and a wider,
    // fainter edge, tinting what is behind them. How a jet at height is seen
    // from far off, before the jet itself is.
    if (scene.trailsAbove >= 0 && dsn.exhausts > 0 && heightOf(p.pose.at) > scene.trailsAbove) {
        Vec3 back = at.turn({-1, 0, 0});
        constexpr int MOST = 14;
        int segments = p.followed ? MOST : p.detail <= DETAIL_SIMPLE ? 11 : 8;
        float light = pal.ambient + pal.diffuse * std::max(0.0f, g.sun.z);
        Rgb ice = scale({246, 248, 252}, std::min(1.25f, std::max(light, 0.25f)));
        float k = std::max(0.6f, p.look.scale);
        // How far back each joint between its sections is.
        float along[MOST + 1];
        along[0] = 6 * k;
        for (int i = 0; i < segments; i++) {
            along[i + 1] = along[i] * 1.45f + 10 * k;
        }
        for (int e = 0; e < dsn.exhausts; e++) {
            Vec3 from = at.place(dsn.exhaust[e]);
            // One strip, its sections sharing their joints, each joint
            // turned across the eye where it is. Each section turned on its
            // own, they twisted apart looking along the trail - the sky
            // through the gaps between them, a dark line, and their edges
            // stepped.
            Vec3 joint[MOST + 1], across[MOST + 1];
            for (int i = 0; i <= segments; i++) {
                joint[i] = from + back * along[i] + Vec3{0, 0, -along[i] * 0.004f};
                across[i] = normalize(cross(back, g.eye - joint[i]));
            }
            // Each section's colour, hazed by how far off it is - and none
            // where the eye is in it, the veil all round. Going by close, it
            // is drawn, cut at the eye.
            uint16_t colors[MOST];
            bool inside[MOST];
            for (int i = 0; i < segments; i++) {
                Vec3 a = joint[i], ab = joint[i + 1] - joint[i];
                float u = clamp01(dot(g.eye - a, ab) / std::max(1e-6f, dot(ab, ab)));
                float wide = (0.25f + (along[i] + (along[i + 1] - along[i]) * u) * 0.008f) * 2.2f;
                inside[i] = length(a + ab * u - g.eye) < wide * 1.5f;
                colors[i] = pack(hazed(ice, length(a + ab * 0.5f - g.eye)));
            }
            for (int layer = 0; layer < 2; layer++) {
                Vec3 left[MOST + 1], right[MOST + 1];
                for (int i = 0; i <= segments; i++) {
                    float w = (0.25f + along[i] * 0.008f) * (layer ? 2.2f : 1.0f);
                    left[i] = toCamera(joint[i] - across[i] * w);
                    right[i] = toCamera(joint[i] + across[i] * w);
                }
                for (int i = 0; i < segments; i++) {
                    if (inside[i]) {
                        continue;
                    }
                    uint16_t color = colors[i];
                    uint8_t eighths = (uint8_t)std::max(1, (layer ? 1 : 4) - i * 4 / segments);
                    auto tint = [&](ScreenTri &t) {
                        t.color = color;
                        t.lit[0] = eighths;
                    };
                    addSeen(left[i], left[i + 1], right[i + 1], out.clear, tint);
                    addSeen(left[i], right[i + 1], right[i], out.clear, tint);
                }
            }
        }
    }

    // Its lights - none on one parked.
    if (!p.followed && p.pose.at.z == 0) {
        return;
    }
    Vec3 nose = at.turn({1, 0, 0});
    // No glow wider than a tenth of the aircraft is long on the screen, so a
    // telephoto close-up's lights stay lights rather than floodlights.
    float aircraftPx = g.f * dsn.halfLength * 1.06f * p.look.scale / std::max(p.distance, NEAR),
          glowCap = std::max(2.0f, aircraftPx * 0.1f);
    for (const PlaneLight &l : dsn.lights) {
        Vec3 world = at.place(l.at);
        float glow = 1.8f;
        switch (l.lamp) {
        case Lamp::NAV:
            if (pal.lights < 0.5f && p.detail < DETAIL_LIGHTS) {
                continue;  // not to be seen by day
            }
            break;
        case Lamp::BEACON_TOP:
        case Lamp::BEACON_BELLY:
            if (fmodf(scene.seconds + (l.lamp == Lamp::BEACON_BELLY ? 0.5f : 0), 1.0f) > 0.12f) {
                continue;
            }
            glow = 2.5f;
            break;
        case Lamp::STROBE: {
            float u = fmodf(scene.seconds, 1.3f);
            if (!(u < 0.05f || (u > 0.18f && u < 0.23f))) {
                continue;
            }
            glow = 3;
            break;
        }
        case Lamp::LANDING: {
            if (!p.followed || !scene.landingLights) {
                continue;
            }
            // A glare as the beam comes round to you.
            float k = clamp01((dot(nose, normalize(g.eye - world)) - 0.75f) / 0.25f);
            glow = 1.8f + 5 * k * k;
            break;
        }
        }
        addLight(world, l.color, l.size * 0.7f * p.look.scale, glow, out.sprites, glowCap);
    }
}

}  // namespace

// --- the frame -----------------------------------------------------------------

void setClock(uint32_t (*micros)()) { g_clock = micros; }

// The NOAA's approximation of where the sun is - good to a fraction of a
// degree, which is all a sky needs - worked in double for the date's sake.
Sun sunAt(int64_t utc, double lat, double lon) {
    constexpr double R = M_PI / 180;
    double julian = utc / 86400.0 + 2440587.5, t = (julian - 2451545.0) / 36525.0;
    double meanLong = fmod(280.46646 + t * (36000.76983 + t * 0.0003032), 360);
    double anomaly = 357.52911 + t * (35999.05029 - 0.0001537 * t);
    double ecc = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
    double centre = sin(anomaly * R) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                    sin(2 * anomaly * R) * (0.019993 - 0.000101 * t) + sin(3 * anomaly * R) * 0.000289;
    double omega = 125.04 - 1934.136 * t;
    double lambda = meanLong + centre - 0.00569 - 0.00478 * sin(omega * R);
    double obliquity = 23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60 +
                       0.00256 * cos(omega * R);
    double decl = asin(sin(obliquity * R) * sin(lambda * R));
    double y = tan(obliquity * R / 2) * tan(obliquity * R / 2);
    double eqTime = 4 / R *
                    (y * sin(2 * meanLong * R) - 2 * ecc * sin(anomaly * R) +
                     4 * ecc * y * sin(anomaly * R) * cos(2 * meanLong * R) - 0.5 * y * y * sin(4 * meanLong * R) -
                     1.25 * ecc * ecc * sin(2 * anomaly * R));
    double minutes = fmod((double)utc, 86400.0) / 60;
    double hourAngle = (fmod(minutes + eqTime + 4 * lon + 1440, 1440) / 4 - 180) * R, phi = lat * R;
    double elevation = asin(sin(phi) * sin(decl) + cos(phi) * cos(decl) * cos(hourAngle)) / R;
    double azimuth = atan2(sin(hourAngle), cos(hourAngle) * sin(phi) - tan(decl) * cos(phi)) / R + 180;
    return {(float)fmod(azimuth, 360), (float)elevation};
}

Sun sunFor(TimeOfDay time) {
    switch (time) {
    case TimeOfDay::DUSK: return {268, 2.5f};
    case TimeOfDay::NIGHT: return {300, -20};
    default: return {200, 40};
    }
}

void setSimd(bool on) {
#if defined(ESP_PLATFORM)
    g_simd = on;
#else
    (void)on;
#endif
}

// Whether all of a cell is out of view: behind the eye, or past an edge of
// the screen and its guard.
bool outOfView(const Cell &c) { return ballInside(toCamera(c.middle)) < -c.radius; }

// Each of `items` in the cells `seen` lets through.
template <typename T, typename Seen, typename Each>
void inCells(const Vector<Cell> &cells, const Vector<T> &items, Seen seen, Each each) {
    for (const Cell &c : cells) {
        if (seen(c)) {
            for (uint32_t i = c.begin; i < c.end; i++) {
                each(items[i]);
            }
        }
    }
}

const auto inView = [](const Cell &c) { return !outOfView(c); };

// A thin marking, as a strip where it is a pixel or more across, and as a
// line a pixel wide where it is less: a strip that thin falls on a pixel's
// middle here and there and breaks up into dashes, which crawl along it as
// the view moves. The line's paint is faded towards the ground's by how much
// of the pixel the marking would really cover - steady at any distance, and
// gone where it is too thin to see.
void addPaint(const PaintLine &p, float groundLight, const Rgb *under = nullptr) {
    Vec3 across = normalize(Vec3{p.a.y - p.b.y, p.b.x - p.a.x, 0}) * p.width;
    Vec3 a = toCamera(p.a), b = toCamera(p.b), side = toCamera(p.a + across) - a;
    if (a.z < NEAR && b.z < NEAR) {
        return;
    }
    // From u0 to u1 of the way from a to b: what of it is in front of the eye.
    float u0 = a.z < NEAR ? (NEAR - a.z) / (b.z - a.z) : 0, u1 = b.z < NEAR ? (NEAR - a.z) / (b.z - a.z) : 1;
    Vec3 c0 = lerp(a, b, u0), c1 = lerp(a, b, u1);
    Projected p0 = project(c0), p1 = project(c1);
    float dx = p1.x - p0.x, dy = p1.y - p0.y, run = sqrtf(dx * dx + dy * dy);
    // How thick it looks at a point: the gap between its edges on the
    // screen, square to the line - not the length of its width put on the
    // screen, which, seen low down and at a slant, lies almost along the
    // line, pixels long, when the edges are a fraction of one apart. Both
    // edges running to the same point on the horizon, the gap closes
    // steadily along the line: its ends say where it is under a pixel.
    auto thickness = [&](Vec3 c) {
        float iz = 1 / c.z;
        float sx = g.f * (side.x - c.x * side.z * iz) * iz, sy = g.f * (side.y - c.y * side.z * iz) * iz;
        return fabsf(dx * sy - dy * sx) / run;
    };
    float thick0 = run < 0.5f ? 1 : thickness(c0), thick1 = run < 0.5f ? 1 : thickness(c1);  // end on: a speck

    Rgb paint = scale(p.color, groundLight);
    auto strip = [&](float ua, float ub) {
        Vec3 e0 = lerp(p.a, p.b, ua), e1 = lerp(p.a, p.b, ub), half = across * 0.5f;
        auto fill = [&](ScreenTri &s) {
            s.lit[0] = (uint8_t)std::min(255.0f, paint.r);
            s.lit[1] = (uint8_t)std::min(255.0f, paint.g);
            s.lit[2] = (uint8_t)std::min(255.0f, paint.b);
        };
        addTriangle(e0 - half, e1 - half, e1 + half, g.ground, fill);
        addTriangle(e0 - half, e1 + half, e0 + half, g.ground, fill);
    };
    // The line in a few pieces, each faded for how thick it is along it:
    // from thickA at ua to thickB at ub, evenly on the screen - and so at
    // `s` of the way along on the screen, v of the way along in depth.
    auto line = [&](float ua, float ub, float thickA, float thickB) {
        // White on a runway, yellow on a taxiway; a river on the land.
        Rgb ground = under != nullptr ? *under : scale(p.color.r == PAINT.r ? ASPHALT : TAXIWAY, groundLight);
        Vec3 ca = lerp(a, b, ua), cb = lerp(a, b, ub), lift = toCamera(p.a + Vec3{0, 0, 0.05f}) - a;
        // A river - kilometres long, all of it under a pixel from high up -
        // in fewer: a few hundred of them were a thousand lines a frame.
        int pieces = std::min(under != nullptr ? 3 : 10, ceilInt(fabsf(thickA - thickB) * 8) + 1);
        Vec3 from = ca;
        for (int i = 1; i <= pieces; i++) {
            float s = (float)i / pieces, v = s / cb.z / ((1 - s) / ca.z + s / cb.z);
            Vec3 to = lerp(ca, cb, v);
            // A little more than its share of the pixel: a thin bright line
            // shows to the eye more than its width says, and on a small
            // screen it is what marks out a runway far off.
            float cover = std::min(1.0f, (thickA + (thickB - thickA) * (i - 0.5f) / pieces) * 1.5f);
            if (cover > 0.06f) {
                // Faded as light adds up, not as the numbers do - squared,
                // near enough - or a thin bright line looks dull. And just
                // over the ground, for the depth to keep it in front.
                auto fade = [&](float under, float over) { return sqrtf(under * under + (over * over - under * under) * cover); };
                Rgb color = {fade(ground.r, paint.r), fade(ground.g, paint.g), fade(ground.b, paint.b)};
                addSeenLine(from + lift, to + lift, color, length(lerp(from, to, 0.5f)), true);
            }
            from = to;
        }
    };
    if (thick0 >= 1 && thick1 >= 1) {
        strip(0, 1);
    } else if (thick0 < 1 && thick1 < 1) {
        line(u0, u1, thick0, thick1);
    } else {
        // A pixel thick part way along: `s` of the way along on the screen.
        float s = (thick0 - 1) / (thick0 - thick1), u = u0 + (u1 - u0) * (s / c1.z / ((1 - s) / c0.z + s / c1.z));
        if (thick0 >= 1) {
            strip(0, u);
            line(u, u1, 1, thick1);
        } else {
            line(u0, u, thick0, 1);
            strip(u, 1);
        }
    }
}

namespace {
// The frame's buildings, as prepareSolids() has them: ready to do once the
// camera is placed, being done by one core, then done.
enum class Solids : uint8_t { DONE, READY, DOING };
std::atomic<Solids> g_solids{Solids::DONE};
void (*g_solidsReady)() = nullptr;
}  // namespace

// --- what covers the land -----------------------------------------------------

#if defined(BENCH3D) && defined(ESP_PLATFORM)
// Where the cover's time goes, each core's - see coverStats() - and the
// see-through's: its cycles and pixels.
uint64_t g_coverStats[2][11];
uint64_t g_seeStats[2][7];
#endif

GroundCover flatCover(const groundpack::Window &window, double lat0, double lon0) {
    GroundCover c;
    c.window = &window;
    c.ux = (float)(groundpack::TILES_PER_DEG / (111320.0 * cos(lat0 * M_PI / 180)));
    c.u0 = (float)((lon0 + 180) * groundpack::TILES_PER_DEG - window.west);
    c.vy = (float)(-groundpack::TILES_PER_DEG / 110574.0);
    c.v0 = (float)((90 - lat0) * groundpack::TILES_PER_DEG - window.north);
    return c;
}

GroundCover roundCover(const groundpack::Window &window, double lat0, double lon0) {
    // The sphere's radius over the ellipsoid's two there: north and south,
    // and east and west.
    constexpr double A = 6378137.0, E2 = 6.69437999014e-3, R = 6371000.0;
    double sp = sin(lat0 * M_PI / 180), q = 1 - E2 * sp * sp;
    GroundCover c;
    c.window = &window;
    c.round = true;
    c.lat0 = lat0;
    c.lon0 = lon0;
    c.kLat = (float)(R / (A * (1 - E2) / (q * sqrt(q))));
    c.kLon = (float)(R / (A / sqrt(q)));
    return c;
}

// The angle whose tangent is y / x, x positive, for one a fifth or less -
// eleven degrees, further than the view sees from its origin - from the
// series, t - t³/3 + t⁵/5 - t⁷/7, good to 6e-8 of a radian, or 40cm:
// a division and four multiplies, where atan2f takes a hundred instructions
// and more, and asinf as many. atan2f itself past that.
inline float atanSmall(float y, float x) {
    float t = y / x;
    if (!(x > 0 && fabsf(t) <= 0.2f)) {
        return atan2f(y, x);
    }
    float t2 = t * t;
    return t * (1 + t2 * (-1.0f / 3 + t2 * (1.0f / 5 + t2 * (-1.0f / 7))));
}

// Where point `p` on the ground is on the cover's grid, in tiles: flat,
// straight from its x and y; round, from its latitude and longitude, worked
// out from its direction from the earth's middle about the origin.
void coverPoint(Vec3 p, float er, float &u, float &v) {
    const GroundCover &c = g.cover;
    const groundpack::Window &W = *c.window;
    if (!c.round) {
        u = c.ux * p.x + c.u0;
        v = c.vy * p.y + c.v0;
        return;
    }
    // Its direction from the earth's middle, turned for the origin's
    // meridian to be the X-Z plane: its latitude atan2(Z, √(X² + Y²)), its
    // longitude from the origin's atan2(Y, X) - and its latitude's from the
    // origin's, by tan(a - b)'s formula, atan2(Z cos φ0 - h sin φ0, h cos φ0 +
    // Z sin φ0), h being √(X² + Y²). Both small, for atanSmall().
    float nx = p.x, ny = p.y, nz = p.z + er, s0 = g.coverSin, c0 = g.coverCos;
    float X = -ny * s0 + nz * c0, Y = nx, Z = ny * c0 + nz * s0, h = sqrtf(X * X + Y * Y);
    float latDeg = g.coverLat0 + atanSmall(Z * c0 - h * s0, h * c0 + Z * s0) * (1 / DEG) * c.kLat;
    float lonDeg = g.coverLon0 + atanSmall(Y, X) * (1 / DEG) * c.kLon;
    v = (90 - latDeg) * groundpack::TILES_PER_DEG - W.north;
    u = (lonDeg + 180) * groundpack::TILES_PER_DEG - W.west;
}

// Row y's five points on the cover's grid, from its ray down the middle,
// `dir`, swung to each across the row - and the level its cells are read
// at: the finest whose cells are four pixels or more, along the row or
// down from the row before.
// Round: row y's five points - see Frame::RowCover - from its ray down the
// middle, `dir`, swung to each across the row. The ends and the middle
// first: where the middle is on the line between the ends, within a
// twentieth of a cell, the earth's curve across the row is too slight to
// matter - the near ground's - and the other two aren't worked out. The
// level from how fast the points go along the row, and down the screen
// from the even row before.
void coverAnchors(int y, Vec3 dir, float er, Frame::RowCover &rc) {
    float ex = g.eye.x, ey = g.eye.y, ez = g.eye.z;
    float above = ex * ex + ey * ey + ez * (ez + 2 * er);
    auto point = [&](int i) {
        Vec3 d = dir + g.right * ((i * g.w * 0.25f - g.cx) / g.f);
        float l = length(d), b = (ex * d.x + ey * d.y + (ez + er) * d.z) / l, disc = b * b - above;
        if (!(disc >= 0 && b < 0 && above > 0)) {
            return false;
        }
        coverPoint(g.eye + d * (above / (-b + sqrtf(disc)) / l), er, rc.u[i], rc.v[i]);
        return true;
    };
    rc.level = Frame::NO_LEVEL;
    if (!point(0) || !point(4) || !point(2)) {
        return;
    }
    float along = std::max(fabsf(rc.u[4] - rc.u[0]), fabsf(rc.v[4] - rc.v[0])) / g.w, rate = along, down = 1e9f;
    if (y >= 2 && g.rowCover[(y >> 1) - 1].level != Frame::NO_LEVEL) {
        const Frame::RowCover &p = g.rowCover[(y >> 1) - 1];
        down = std::max(fabsf(rc.u[2] - p.u[2]), fabsf(rc.v[2] - p.v[2])) * 0.5f;
        rate = std::max(rate, down);
    }
    float cellsPerPx = rate * (groundpack::ROWS0 << -groundpack::FINEST);
    int level = groundpack::FINEST;
    while (level < groundpack::LEVELS - 1 && cellsPerPx * 4 > 1) {
        cellsPerPx *= 0.5f;
        level++;
    }
    float cell = 1.0f / (level < 0 ? groundpack::ROWS0 << -level : groundpack::ROWS0 >> level), off = 0.05f * cell;
    rc.straight = fabsf(rc.u[2] - (rc.u[0] + rc.u[4]) * 0.5f) <= off && fabsf(rc.v[2] - (rc.v[0] + rc.v[4]) * 0.5f) <= off;
    rc.tall = down * 8 <= cell;
    rc.wide = along * 16 <= cell;
    if (!rc.straight && (!point(1) || !point(3))) {
        return;
    }
    rc.level = (int8_t)level;
}

// Flat: row y's two ends on the pack's grid - every point between on the
// line from one to the other, the ground being a plane - straight from the
// camera: the row's rays all meet the ground the same way along them, the
// eye's height over how steeply the row looks down, the camera having no
// roll. False where the row doesn't look down at the ground.
inline bool coverFlatEnds(const Frame &f, int y, float &u0, float &v0, float &u4, float &v4) {
    const Frame::CoverFlat &k = f.coverFlat;
    float sy = y * k.perF + k.sy0, dz = k.fz + k.dz1 * sy;
    if (!(dz < 0 && k.ez > 0)) {
        return false;
    }
    float t = k.ez / -dz, du = t * k.ku, dv = t * k.kv;  // a pixel's worth
    u0 = k.au + t * (k.bu + k.cu * sy);
    v0 = k.av + t * (k.bv + k.cv * sy);
    u4 = u0 + du * f.w;
    v4 = v0 + dv * f.w;
    return true;
}

// The same, as Frame::RowCover has it, with its level - from how fast the
// ends go along the row, and how fast the middle goes down the screen to
// the row two under it.
bool coverFlatRow(const Frame &f, int y, Frame::RowCover &rc) {
    float u2, v2, u6, v6;
    if (!coverFlatEnds(f, y, rc.u[0], rc.v[0], rc.u[4], rc.v[4]) || !coverFlatEnds(f, y + 2, u2, v2, u6, v6)) {
        rc.level = Frame::NO_LEVEL;
        return false;
    }
    float along = std::max(fabsf(rc.u[4] - rc.u[0]), fabsf(rc.v[4] - rc.v[0])) * f.perW;
    float down = std::max(fabsf(u2 + u6 - rc.u[0] - rc.u[4]), fabsf(v2 + v6 - rc.v[0] - rc.v[4])) * 0.25f;
    float rate = std::max(along, down);
    // The finest level whose cells are four pixels or more: a cell of level
    // L is 2^L / 288 of a tile, from level -2's 1/1152.
    constexpr float FINE = 1.0f / (4 * (groundpack::ROWS0 << -groundpack::FINEST));
    rc.level = (int8_t)(groundpack::FINEST + (rate > FINE) + (rate > 2 * FINE) + (rate > 4 * FINE) +
                        (rate > 8 * FINE) + (rate > 16 * FINE) + (rate > 32 * FINE));
    rc.straight = true;
    // A cell of level L is 4 FINE 2^(L - FINEST) of a tile.
    float cell = 4 * FINE * (float)(1 << (rc.level - groundpack::FINEST));
    rc.tall = down * 8 <= cell;
    rc.wide = along * 16 <= cell;
    return true;
}

template <typename T>
T *align16(T *p) {
    return (T *)(((uintptr_t)p + 15) & ~(uintptr_t)15);
}

// The hills along the skyline, from the horizon worked out round a place
// near the eye - see Scene::horizon - the highest land each way. Each bearing's
// highest point put where it is from there, at the angle and distance the
// horizon has, and seen from where the eye is now: as the eye comes closer
// or goes by, the hills move and grow as they would, rather than standing
// still till the next horizon. Along the screen, each column between two
// such points, a bearing apart - eight pixels or so - has the row of the
// hill's top on the line between theirs, and the row of its foot, where the
// ground the view draws is as far off; and its colour that of the nearer
// of the two: the dark green of wet moorland and plantations, lit as the
// land is, in a damp haze - cooler than the sky's, and thicker than the
// view's, as Scotland's air mostly is - as far off as that hill is, so a
// hill comes out of the haze as it is neared - and as far off as the
// bearings either side have theirs, so a near ridge's end against a far one
// is no stripe of another haze. Down to the eye's horizon only: below it is
// the land before the hills, or the earth's curve hiding their feet, and a
// ridge the eye is above isn't against the sky at all - the cover's light and
// shade on the ground shows it. Drawn down to their feet, from a few
// thousand feet the hills lay across the land as pale arcs. None at all
// above 4,000m, where the hills are seen from above.
void skyline(const Scene &scene, float groundLight) {
    constexpr int R = groundpack::Horizon::RANGES, B = groundpack::Horizon::BEARINGS;
    for (Frame::HillRange &range : g.hills) {
        range.on = false;
    }
    // None down among the hills: where land nearer than the skyline's 5km
    // stands over the eye a sixth of the way round, the skyline - with
    // those fells left out of it - stood as walls over them and broke off
    // in blocks where they crossed the 5km, a telephoto making the blocks
    // a screen high. Only real terrain can draw that; the cover's light and
    // shade on the ground shows them meanwhile.
    const groundpack::Horizon *hz = scene.horizon;
    if (hz == nullptr || scene.eyeHeightM > 4000 || hz->nearAbove > 1.0f / 6) {
        return;
    }
    int w = g.w, h = g.h;
    size_t n = ((size_t)w + 7) & ~(size_t)7;
    g.hillRows.resize(2 * R * n + 8);
    g.hillColors.resize(2 * R * n + 8);
    g.hillDepths.resize(R * n + 8);
    g.hillIz.resize(R * n + 4);
    int16_t *rows = align16(g.hillRows.data());
    uint16_t *colors = align16(g.hillColors.data()), *depths = align16(g.hillDepths.data());
    for (int r = 0; r < R; r++) {
        g.hills[r].from = rows + 2 * r * n;
        g.hills[r].to = rows + (2 * r + 1) * n;
        g.hills[r].pixels[0] = colors + 2 * r * n;
        g.hills[r].pixels[1] = colors + (2 * r + 1) * n;
        g.hills[r].depth = depths + r * n;
        g.hills[r].iz = g.hillIz.data() + r * n;
    }
    float er = g.scene.earthRadius;
    Vec3 centre = {0, 0, -er}, at = scene.horizonAt, eye = g.eye;
    // Up, north and east where the horizon was worked out: flat, the view's
    // own; round, the sphere's there.
    Vec3 up = {0, 0, 1}, north = {0, 1, 0}, east = {1, 0, 0};
    if (er > 0) {
        up = normalize(at - centre);
        north = normalize(Vec3{0, 1, 0} - up * up.y);
        east = cross(north, up);
    }
    // The row the eye's horizon is at: flat, its level; round, below it by
    // its height over the sphere - as straight across the middle of the
    // screen, the camera having no roll.
    float overGround = er > 0 ? length(eye - centre) - er : eye.z;
    float dip = er > 0 ? sqrtf(2 * std::max(0.0f, overGround) / er) : 0;
    Vec3 level = {g.forward.x, g.forward.y, 0};
    level = normalize(Vec3{level.x, level.y, -dip * length(level)});
    float horizonRow = dot(level, g.forward) > 1e-3f ? g.cy + g.f * dot(level, g.down) / dot(level, g.forward) - 0.5f
                                                     : (dot(level, g.down) > 0 ? (float)h : 0.0f);
    // The colours: a damp haze - the view's, cooled towards a blue grey as
    // bright, as wet air scatters - thicker than the view's, 30km taking 63%
    // of a colour; over the dark green of wet hills, lit.
    const Rgb &haze = g.pal.haze;
    float bright = 0.3f * haze.r + 0.59f * haze.g + 0.11f * haze.b;
    Rgb damp = mix(haze, {bright * 0.88f, bright * 0.97f, bright * 1.08f}, 0.5f);
    Rgb hills = scale({52, 72, 50}, groundLight);
    float perVisibility = 1 / std::min(g.pal.visibility, 30000.0f);
    // A bearing's point as the screen has it: where across, the rows of
    // its top and foot, and its colour's four pixels - by row and column
    // parity - or none.
    struct Seen {
        bool on = false;
        float x = 0, top = 0, foot = 0, iz = 0;
        uint16_t pixels[2][2] = {};
    };
    auto see = [&](int r, int b, Vec3 dir, Seen &out) {
        out.on = false;
        float far = hz->distance[r][b], tanUp = hz->tanUp[r][b];
        if (!(tanUp > -0.5f)) {
            return;
        }
        Vec3 top = at + dir * far + up * (far * tanUp);
        Vec3 foot = er > 0 ? centre + normalize(top - centre) * er : Vec3{top.x, top.y, 0};
        Vec3 ct = top - eye, cf = foot - eye;
        float zt = dot(ct, g.forward), zf = dot(cf, g.forward);
        if (zt < 1 || zf < 1) {
            return;  // behind the eye, or beside it
        }
        float pt = g.f / zt, pf = g.f / zf;
        out.iz = 1 / zt;
        out.x = g.cx + dot(ct, g.right) * pt - 0.5f;
        out.top = g.cy + dot(ct, g.down) * pt - 0.5f;
        out.foot = g.cy + dot(cf, g.down) * pf - 0.5f;
        // Its haze by how far off the ridges are four bearings either side
        // too, the nearer the more - two degrees' worth - so a ridge at 9km
        // here and the next at 24km shade into each other along the hills
        // rather than standing in bands of two hazes.
        static const float WEIGHT[5] = {5, 4, 3, 2, 1};
        float sum = WEIGHT[0] * far, weight = WEIGHT[0];
        for (int k = 1; k <= 4; k++) {
            for (int side : {-k, k}) {
                int nb = (b + side + B) % B;
                if (hz->tanUp[r][nb] > -0.5f) {
                    sum += WEIGHT[k] * hz->distance[r][nb];
                    weight += WEIGHT[k];
                }
            }
        }
        float x = length(ct) * (sum / weight / far) * perVisibility;
        float clear = 1 / (1 + x * (1 + x * (0.5f + x * (1.0f / 6 + x * (1.0f / 24)))));
        Rgb color = mix(hills, damp, 1 - clear);
        // Dithered to the screen's 5, 6 and 5 bits by an ordered 2 by 2's
        // quarters, in whole quarters: floor(c + d) is (floor(4c) + 4d) / 4.
        int qr = (int)(color.r * (4 * 31.0f / 255)), qg = (int)(color.g * (4 * 63.0f / 255)),
            qb = (int)(color.b * (4 * 31.0f / 255));
        static const int BAYER4[2][2] = {{0, 2}, {3, 1}};
        for (int p = 0; p < 2; p++) {
            for (int q = 0; q < 2; q++) {
                int d = BAYER4[p][q];
                int lr = std::min(31, (qr + d) >> 2), lg = std::min(63, (qg + d) >> 2), lb = std::min(31, (qb + d) >> 2);
                out.pixels[p][q] = swapIf((uint16_t)(lr << 11 | lg << 5 | lb));
            }
        }
        out.on = true;
    };
    // The bearings in view, left to right: those the screen's edges reach -
    // as each of its top, the horizon's and its bottom rows has them, the
    // camera having no roll - and two more each side, for the spans to reach
    // the edges. Looking straight up or down, half a turn about where it
    // looks, as all of them might be.
    float lookingAt = atan2f(g.forward.x, g.forward.y), spread = 0;
    {
        float rowsAt[3] = {0, std::min((float)h, std::max(0.0f, horizonRow)), (float)h};
        for (float y : rowsAt) {
            Vec3 d = g.forward + g.down * ((y - g.cy) / g.f);
            float along = d.x * g.forward.x + d.y * g.forward.y, flat = length(Vec3{g.forward.x, g.forward.y, 0});
            along = flat > 1e-4f ? along / flat : 0;
            spread = along > 1e-3f ? std::max(spread, atanf((g.cx + 1) / g.f / along)) : PI / 2;
        }
        spread = std::min(spread, PI / 2);
    }
    float looking = lookingAt + scene.horizonTurn, step = 2 * PI / B;
    // And as many more each side as the eye's being off from where the
    // horizon was worked out can turn a hill 5km off: a fast jet's second
    // ahead of it, a fly past's camera set down 2km on - a hill there seen
    // 7 to 22 degrees round from where its bearing is, its range cut off by
    // the screen's edge, a block, or not there.
    float offM = hypotf(eye.x - at.x, eye.y - at.y);
    int extra = (int)ceilf(atanf(offM / 5000) / step);
    int first = (int)floorf((looking - spread) / step) - 2 - extra,
        last = (int)ceilf((looking + spread) / step) + 2 + extra;
    last = std::min(last, first + std::min(B - 1, B / 2 + 2 + 2 * extra));
    // Each bearing's direction, turned on from the last's by a bearing's
    // angle - two multiplies and an add each way, where sinf and cosf were a
    // hundred cycles and more each.
    float a0 = first * step - scene.horizonTurn, sa = sinf(a0), ca = cosf(a0);
    const float SS = sinf(step), CS = cosf(step);
    Seen left[R];
    int top[R], bottom[R], lowest[R];
    for (int r = 0; r < R; r++) {
        memset(g.hills[r].from, 0, n * sizeof(int16_t));
        memset(g.hills[r].to, 0, n * sizeof(int16_t));
        top[r] = h;
        bottom[r] = 0;
        // Down to the horizon at most - see above.
        lowest[r] = std::max(0, std::min(h, ceilInt(horizonRow)));
    }
    for (int k = first; k <= last; k++) {
        int b = (k % B + B) % B;
        Vec3 dir = east * sa + north * ca;
        float s2 = sa * CS + ca * SS;
        ca = ca * CS - sa * SS;
        sa = s2;
        for (int r = 0; r < R; r++) {
            Frame::HillRange &range = g.hills[r];
            Seen right;
            see(r, b, dir, right);
            const Seen &l = left[r];
            if (l.on && right.on && right.x > l.x) {
                // The columns between the two, on the lines between.
                int x0 = std::max(0, ceilInt(l.x)), x1 = std::min(w, ceilInt(right.x));
                if (x0 < x1) {
                    float perSpan = 1 / (right.x - l.x), t = (x0 - l.x) * perSpan;
                    float dTop = (right.top - l.top) * perSpan, dFoot = (right.foot - l.foot) * perSpan;
                    float *iz = range.iz;
                    float rowTop = l.top + (right.top - l.top) * t, rowFoot = l.foot + (right.foot - l.foot) * t;
                    for (int x = x0; x < x1; x++, t += perSpan, rowTop += dTop, rowFoot += dFoot) {
                        int rt = std::max(0, std::min(h, ceilInt(rowTop))),
                            rf = std::max(0, std::min(lowest[r], ceilInt(rowFoot)));
                        if (rt >= rf) {
                            continue;
                        }
                        range.from[x] = (int16_t)rt;
                        range.to[x] = (int16_t)rf;
                        iz[x] = l.iz + (right.iz - l.iz) * t;  // 1/z goes straight across the screen
                        top[r] = std::min(top[r], rt);
                        bottom[r] = std::max(bottom[r], rf);
                        // The colour of one or the other, by how far across
                        // it is against an ordered dither's thresholds: the
                        // two blended over the span between them, two hills'
                        // hazes a bearing apart with no seam between, for
                        // nothing.
                        static const float THRESHOLD[2][4] = {{0.125f, 0.625f, 0.375f, 0.875f},
                                                              {0.875f, 0.375f, 0.625f, 0.125f}};
                        for (int p = 0; p < 2; p++) {
                            range.pixels[p][x] = (t < THRESHOLD[p][x & 3] ? l : right).pixels[p][x & 1];
                        }
                    }
                }
            }
            left[r] = right;
        }
    }
    for (int r = 0; r < R; r++) {
        Frame::HillRange &range = g.hills[r];
        range.top = top[r];
        range.bottom = bottom[r];
        range.on = top[r] < bottom[r];
    }
}

// --- the terrain -----------------------------------------------------------------------

// The terrain's colour pairs, a few of them at a time: by cell and haze -
// a sample's mostly the last's, or one a few back - each core and parity
// of row its own, in the chip's own memory: 1KB each, where a table of
// them all, 33KB, had to be in the PSRAM and was read from it at random.
struct TerrainPairs {
    uint32_t serial = 0xFFFFFFFF;
    uint16_t key[128];
    uint32_t pair[128];
};
TerrainPairs g_terrainPairs[4];

// The terrain's costs, for the log: the march's cycles, samples, rows filled
// and tiles' heights looked up; the rows' cycles, samples, cells found the
// long way and colours worked out - each core's, the rows being both's.
uint64_t g_terrainStats[2][8];
// And the caches' counts over them, on the bench: the march's and the
// drawing's L1 and L2 data hits and misses, each core's own bus.
uint64_t g_terrainCache[2][8];
// On the bench, the instructions the land's bands took, each core's; and
// the samples the assembly left for C.
uint64_t g_bandInstret[2], g_bandHanded[2];
#if defined(ESP_PLATFORM) && defined(BENCH3D)
}  // namespace scene3d
#include "soc/cache_reg.h"
#include "soc/soc.h"
namespace scene3d {
struct CacheCounts {
    uint32_t l1Hit, l1Miss, l2Hit, l2Miss;
};
// This core's data bus's counts - on first use the counters set going.
CacheCounts cacheCounts(int core) {
    static bool on = false;
    if (!on) {
        on = true;
        REG_SET_BIT(CACHE_L1_CACHE_ACS_CNT_CTRL_REG, CACHE_L1_DBUS0_CNT_ENA | CACHE_L1_DBUS1_CNT_ENA);
        REG_SET_BIT(CACHE_L2_CACHE_ACS_CNT_CTRL_REG, CACHE_L2_DBUS0_CNT_ENA | CACHE_L2_DBUS1_CNT_ENA);
    }
    CacheCounts c;
    c.l1Hit = REG_READ(core ? CACHE_L1_DBUS1_ACS_HIT_CNT_REG : CACHE_L1_DBUS0_ACS_HIT_CNT_REG);
    c.l1Miss = REG_READ(core ? CACHE_L1_DBUS1_ACS_MISS_CNT_REG : CACHE_L1_DBUS0_ACS_MISS_CNT_REG);
    c.l2Hit = REG_READ(core ? CACHE_L2_DBUS1_ACS_HIT_CNT_REG : CACHE_L2_DBUS0_ACS_HIT_CNT_REG);
    c.l2Miss = REG_READ(core ? CACHE_L2_DBUS1_ACS_MISS_CNT_REG : CACHE_L2_DBUS0_ACS_MISS_CNT_REG);
    return c;
}
void cacheAdd(uint64_t *into, const CacheCounts &a, const CacheCounts &b) {
    into[0] += b.l1Hit - a.l1Hit;
    into[1] += b.l1Miss - a.l1Miss;
    into[2] += b.l2Hit - a.l2Hit;
    into[3] += b.l2Miss - a.l2Miss;
}
#define CACHE_BEGIN(name) CacheCounts name = cacheCounts(coreNow())
#define CACHE_END(name, into) cacheAdd(into, name, cacheCounts(coreNow()))
#else
#define CACHE_BEGIN(name)
#define CACHE_END(name, into)
#endif
HOT_INLINE uint32_t cycles() {
#if defined(ESP_PLATFORM)
    return esp_cpu_get_cycle_count();
#else
    // On the PC, nanoseconds - for the preview's TSTATS.
    return (uint32_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
#endif
}
HOT_INLINE int coreNow() {
#if defined(ESP_PLATFORM)
    return xPortGetCoreID() & 1;
#else
    return 0;
#endif
}

// A float's top half - its sign, exponent and seven bits of its mantissa -
// and back: the terrain's depths, to under 1% (at 10km, 80m: a cell there is
// 600m), in half the memory a float would take.
HOT_INLINE uint16_t toBf16(float v) {
    uint32_t b;
    memcpy(&b, &v, 4);
    return (uint16_t)((b + 0x8000) >> 16);
}
HOT_INLINE float fromBf16(uint16_t v) {
    uint32_t b = (uint32_t)v << 16;
    float f;
    memcpy(&f, &b, 4);
    return f;
}

// The land's height at (u, v) on the cover's grid - tiles east and south of
// the window's corner - metres above the sea: between the four nearest of
// the finest heights the window has there, every 15 seconds where a tile's
// mid block is in, else its coarse heights; nought off the tiles.
HOT_INLINE float packHeight(const groundpack::Window &W, float u, float v) {
    int tu = floorInt(u), tv = floorInt(v);
    if ((unsigned)tu >= (unsigned)W.cols || (unsigned)tv >= (unsigned)W.rows) {
        return 0;
    }
    int slot = tv * W.cols + tu, c0 = groundpack::cols0At(W.north + tv);
    const int16_t *hs;
    int rows, cols;
    if (const uint8_t *mid = W.mid[slot]) {
        rows = groundpack::ROWS0 / 6;
        cols = c0 / 6;
        hs = (const int16_t *)(mid + groundpack::midHeightsAt(c0));
    } else if (const uint8_t *coarse = W.coarse[slot]) {
        rows = groundpack::ROWS0 / 16;
        cols = c0 / 16;
        hs = (const int16_t *)(coarse + rows * cols);
    } else {
        return 0;
    }
    if (cols <= 0) {
        return 0;
    }
    float y = (v - tv) * rows - 0.5f, x = (u - tu) * cols - 0.5f;
    int r0 = floorInt(y), q0 = floorInt(x);
    float wy = y - r0, wx = x - q0;
    int r1 = std::min(rows - 1, r0 + 1), q1 = std::min(cols - 1, q0 + 1);
    r0 = std::max(0, r0);
    q0 = std::max(0, q0);
    float top = hs[r0 * cols + q0] + (hs[r0 * cols + q1] - hs[r0 * cols + q0]) * wx;
    float bottom = hs[r1 * cols + q0] + (hs[r1 * cols + q1] - hs[r1 * cols + q0]) * wx;
    return top + (bottom - top) * wy;
}

// The land at its height, as the view sees it: down each fourth column of
// the screen, from the eye outwards along the ground the column looks over,
// the land sampled - its height from the pack - and put where it is on the
// screen; each sample standing above all those nearer fills the column's
// rows from theirs up to its own, with how deep along the view it is there,
// between the depths of it and the sample before. Nearer than the last
// filled row, a sample is hidden, and is told so by a multiply and a
// compare, the division only for one seen. The samples a fiftieth of the
// way further each, from a metre or so out to 100km - their heights between
// samples 460m apart, nearer than that adds nothing - and a column given up
// once nothing further could rise above what it has: the highest land
// there is, 1,350m, put further off, under its filled rows. Out only as far
// as the haze lets anything be seen - where 95% of a hill is haze - and
// 100km at most: hills culled further out, which would be the most samples
// for the least. The haze the view's over the first couple of kilometres,
// as the airfield and the aircraft have it, then thinning to 30km's - the
// skyline's - as hills stand out of the low haze: at the view's 7km all
// through, Ben Lomond from Glasgow, 35km off, would be gone. The earth's curve taken off the
// land with distance, a seventh back for the air's bending the light, as
// the skyline had it. An airfield's ground held at the field's height out
// to 2.5km, easing to the land's by 5km: its runways and buildings stand on
// it.
// A column's march - see terrainMarch() - what is alike for every column:
// the steps, the grid, the eye and the screen.
struct MarchFrame {
    const float *far, *base;  // each step's distance; flat, what its height is set against
    int steps;
    const int16_t *heights, *cellMax, *blockMax;
    int gcols, bcols, rowMask;
    uint32_t xLim, yLim;  // a sample's place on the grid under these, 256ths of a cell, has its neighbours
    float ez, groundUp, e2, fall;  // the earth's fall: flat, s^2 fall; round, (r^2) fall from the origin
    float Fz, Dz, f, cy, zHigh;
    float skipTo;  // blocks skipped only nearer than this: further, a step or two crosses one
    uint16_t *tz;
    int cols, bandRows;
    struct MarchState *asmState;  // the assembly's, its frame's part filled in
    const float *zHighAt;         // the column's highest land, from the eye, from each sixteenth step on
};
// And a column's own: its ray over the grid, 256ths of a cell - x = X0 +
// s (XS + s XS2), y likewise - how deep and how far down a metre along it
// puts the land, where it crosses the ring an airfield's flat ground eases
// into the land's, the sample before, and where it starts.
struct MarchRay {
    float X0, XS, XS2, Y0, YS, YS2;
    float A, B, eh2;
    float easeLo, easeHi;
    float lastS, lastH;
    int c, first, filled;
};
struct MarchCounts {
    uint32_t samples = 0, rows = 0, skips = 0;
    // On the bench, the samples by the way they went: hidden by their
    // cell's highest corner, hidden worked out, seen with no rows, seen
    // filling rows, the previous sample's height worked out again, blocks
    // tested, and samples outside the grid.
    uint32_t paths[7] = {};
};
MarchCounts g_marchPaths;
// On the bench, ##i: what the terrain's memory costs it - 1 the march's
// heights read from four rows of the grid, 2 the drawing's cells all one -
// for its cycles with and without; 4 the drawing in C alone, for the
// assembly to be checked against.
int g_terrainProbe = 0;

constexpr int MARCH_MOST = 400;  // steps a column's march takes at most

// The first of the steps' distances, from step `from` on, not under s - by
// halves, where walking them one at a time took hundreds a column.
HOT_INLINE int stepFrom(const float *far, int steps, int from, float s) {
    int lo = from, hi = steps;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (far[mid] < s) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

// A column's march where scene3d_march_flat (scene3d_terrain.S) runs it:
// what it needs of the frame and the column, and where the march is, which
// it and marchColumn() hand back and forth. Its offsets are the assembly's.
struct MarchState {
    const float *far;        // 0: the steps' distances; their bases MARCH_MOST floats on
    int steps;               // 4
    const int16_t *heights;  // 8
    int cellMaxBytes;        // 12: the cells' highest corners, this many bytes on from their heights
    int rowBytes;            // 16: a row of the grid's
    uint32_t xLim, yLim;     // 20, 24
    int bcols;               // 28
    float Fz, Dz, f, cyLess; // 32, 36, 40, 44: cyLess = cy - 0.5
    float cyMore;            // 48: cy + 0.5
    float zHigh, skipTo;     // 52, 56
    uint16_t *tz;            // 60
    int bandRows1, bandBack; // 64, 68: bandRows - 1; the row above a band's first, in the band before
    float X0, XS, Y0, YS;    // 72, 76, 80, 84
    float A, B;              // 88, 92
    int iEaseLo, iEaseHi;    // 96, 100: the steps in the ring an airfield's ground eases over
    float e2, eh2, groundUp; // 104, 108, 112
    float flatIn, perRing;   // 116, 120: 2500, 1 / 2500
    float tiny, notSloped;   // 124, 128: 1e-3, -1e-4
    float minusInf, three;   // 132, 136
    int i, filled, at, off;  // 140, 144, 148, 152
    float k, gk, gkLine;     // 156, 160, 164
    int sloped;              // 168
    int prev, known, shown;  // 172, 176, 180
    float knownH, shownRho, shownW;  // 184, 188, 192
    int curBlk;              // 196
    uint32_t samples;        // 200
    float rXS, rYS;          // 204, 208: 1 / XS and 1 / YS, nought for none
    float gez, fall, twoFall;  // 212, 216, 220: groundUp + ez; the earth's fall, and twice it
    float easeLoF, easeHiF;  // 224, 228: the ring's distances
    const int16_t *blockMax; // 232
    int iEaseLo0;            // 236: the ring's first step, kept
    uint32_t skips;          // 240
    float farOff;            // 244: 1e9, a block not left
    const float *zHighAt;    // 248: the column's highest land, from the eye, from each sixteenth step on
    float XS2, YS2;          // 252, 256: round, the ray's curve over the grid
    float eh2f;              // 260: round, the column's part of the earth's fall, s eh2f
    float tinyStep;          // 264: 1e-9, a ray too near square to the grid's x or y for 1 / it
};
#if defined(ESP_PLATFORM)
static_assert(offsetof(MarchState, X0) == 72 && offsetof(MarchState, iEaseLo) == 96 &&
                  offsetof(MarchState, minusInf) == 132 && offsetof(MarchState, i) == 140 &&
                  offsetof(MarchState, k) == 156 && offsetof(MarchState, prev) == 172 &&
                  offsetof(MarchState, knownH) == 184 && offsetof(MarchState, samples) == 200,
              "scene3d_terrain.S has MarchState's fields where these are");
static_assert(offsetof(MarchState, rXS) == 204 && offsetof(MarchState, blockMax) == 232 &&
                  offsetof(MarchState, farOff) == 244 && offsetof(MarchState, zHighAt) == 248 &&
                  offsetof(MarchState, eh2f) == 260 && offsetof(MarchState, tinyStep) == 264,
              "scene3d_terrain.S has MarchState's fields where these are");
// The march of one column over flat ground from step m->i, the common ways:
// 0 when the column is done, 1 at a step it leaves for C - see
// scene3d_terrain.S. And over the round earth.
extern "C" int scene3d_march_flat(MarchState *m);
extern "C" int scene3d_march_round(MarchState *m);
#endif

// One column's march, out from its first step, everything it needs in
// registers: its rows filled from `filled` up, and the top one returned. A
// sample's place on the grid as an integer, 256ths of a cell - its cell and
// where in it at once - and its cell's highest corner set against the line
// of sight through the top filled row, a multiply-add and a compare: under
// it, the sample is hidden, the most of them. Only one that may not be has
// its height worked out between its cell's corners, and only one seen is
// put on the screen, with a division - and the sample before it, for the
// rows between. Into another block of the grid near the eye, the block
// skipped whole where its highest can't reach the line of sight anywhere
// across it.
template <bool ROUND>
int marchColumn(const MarchFrame &K, const MarchRay &R, MarchCounts &counts) {
    constexpr float PER_Q = 1.0f / 65536, FLAT_IN = 2500, FLAT_OUT = 5000;
    constexpr int BLOCK_BITS = 11;  // a block, 8 cells, 2048 256ths of one
    const float *far = K.far, *base = K.base;
    const int steps = K.steps;
    const int16_t *heights = K.heights, *cellMax = K.cellMax, *blockMax = K.blockMax;
    const int gcols = K.gcols, bcols = K.bcols, cols = K.cols, bandRows = K.bandRows;
    const uint32_t xLim = K.xLim, yLim = K.yLim;
    const float ez = K.ez, groundUp = K.groundUp, e2 = K.e2, fall = K.fall;
    const float Fz = K.Fz, Dz = K.Dz, f = K.f, cy = K.cy, skipTo = K.skipTo;
    const float *zHighAt = K.zHighAt;
    // Fused as scene3d_march_flat fuses them, so that the two agree to the
    // bit: a row from cy - 0.5, a line of sight's k from cy + 0.5.
    const float cyLess = cy - 0.5f, cyMore = cy + 0.5f;
    // What a sample's height is set against, the base, is base[i] + s eh2f
    // at step i, and fma(s s, fall, gez) + s eh2f anywhere: flat, the eye's
    // height above the sea and the earth's fall, eh2f nought; round, the
    // fall from the origin, eh2f the column's part of it.
    const float eh2f = ROUND ? R.eh2 * fall : 0, gez = ROUND ? fmaf(e2, fall, ez) : groundUp + ez;
    const float X0 = R.X0, XS = R.XS, XS2 = R.XS2, Y0 = R.Y0, YS = R.YS, YS2 = R.YS2;
    const float A = R.A, B = R.B, eh2 = R.eh2, easeLo = R.easeLo, easeHi = R.easeHi;
    const float rXS = ROUND || fabsf(XS) < 1e-9f ? 0 : 1 / XS, rYS = ROUND || fabsf(YS) < 1e-9f ? 0 : 1 / YS;
    uint16_t *tz = K.tz;
    // What the sample's height is set against, but for the line of sight's
    // slope: the eye's height above the sea and the earth's fall that far.
    auto underOf = [&](float s) { return fmaf(s, eh2f, fmaf(s * s, fall, gez)); };
    // An airfield's ground eased into the land's, out to FLAT_OUT from it.
    auto eased = [&](float h, float s) {
        if (ROUND || s >= easeHi || s < easeLo) {
            return h;
        }
        float t = std::min(1.0f, std::max(0.0f, (sqrtf(fmaf(s, eh2 + s, e2)) - FLAT_IN) * (1 / (FLAT_OUT - FLAT_IN))));
        return fmaf(h - groundUp, t * t * (3 - (t + t)), groundUp);
    };
    // The land's height at grid place (ix, iy), between its cell's corners.
    auto between = [&](int ix, int iy) {
        int fx = ix & 255, fy = iy & 255;
#ifdef BENCH3D
        const int16_t *p = heights + (size_t)((iy >> 8) & K.rowMask) * gcols + (ix >> 8);
#else
        const int16_t *p = heights + (size_t)(iy >> 8) * gcols + (ix >> 8);
#endif
        int a0 = p[0], b0 = p[1], c0 = p[gcols], d0 = p[gcols + 1];
        int top = (a0 << 8) + (b0 - a0) * fx, bottom = (c0 << 8) + (d0 - c0) * fx;
        return ((top << 8) + (bottom - top) * fy) * PER_Q;
    };
    // A step's height, all of it worked out: for the sample before one seen.
    auto heightAt = [&](float s) {
        float x = ROUND ? fmaf(s, fmaf(s, XS2, XS), X0) : fmaf(s, XS, X0);
        float y = ROUND ? fmaf(s, fmaf(s, YS2, YS), Y0) : fmaf(s, YS, Y0);
        int ix = (int)x, iy = (int)y;
        return eased((uint32_t)ix < xLim && (uint32_t)iy < yLim ? between(ix, iy) : 0.0f, s);
    };
    int filled = R.filled;
    float k = filled - cyMore;
    float den = fmaf(-k, Fz, f * Dz);
    bool sloped = den < -1e-4f;
    float gk = sloped ? fmaf(k, A, -(f * B)) / den : 0;
    float gkLine = sloped ? gk : -INFINITY;  // not sloped: every sample to the full test
    // The sample before: a step, -2 the flat ground's edge, -1 none; the
    // last whose height was worked out; and the last put on the screen,
    // its row and 1 / depth.
    int prev = R.lastS >= 0 ? -2 : -1, known = -3, shown = -3;
    float knownH = 0, shownRho = 0, shownW = 0;
    // Row filled - 1 of the column: where it is kept, and its row in its band.
    int off = (filled - 1) % bandRows;
    int at = (((filled - 1) / bandRows) * cols + R.c) * bandRows + off;
    const int bandBack = bandRows - 1 - cols * bandRows;  // the row above a band's first, in the band before
    const bool everyRow = bandRows & 1;                    // bands an odd number of rows: every row kept
    int curBlk = -1;
    uint32_t samples = 0, skips = 0;
#ifdef BENCH3D
    uint32_t path0 = 0, path1 = 0, path2 = 0, path3 = 0, path4 = 0, path5 = 0, path6 = 0;
#endif
    // Sample i - its next step i + 1, or where a block skipped puts it -
    // true once the column is done.
    auto step = [&](int &i) __attribute__((always_inline)) {
        float s = far[i];
        samples++;
        float x = ROUND ? fmaf(s, fmaf(s, XS2, XS), X0) : fmaf(s, XS, X0);
        float y = ROUND ? fmaf(s, fmaf(s, YS2, YS), Y0) : fmaf(s, YS, Y0);
        float line = fmaf(s, gkLine + eh2f, base[i]);
        int ix = (int)x, iy = (int)y;
        float hgt = 0;
        if ((uint32_t)ix < xLim && (uint32_t)iy < yLim) {
            int blk = (iy >> BLOCK_BITS) * bcols + (ix >> BLOCK_BITS);
            if (blk != curBlk) {
                curBlk = blk;
                if (sloped && s < skipTo) {
#ifdef BENCH3D
                    path5++;
#endif
                    int bx = ix >> BLOCK_BITS, by = iy >> BLOCK_BITS;
                    float dX = ROUND ? fmaf(s + s, XS2, XS) : XS, dY = ROUND ? fmaf(s + s, YS2, YS) : YS;
                    float rX = ROUND ? (fabsf(dX) < 1e-9f ? 0 : 1 / dX) : rXS;
                    float rY = ROUND ? (fabsf(dY) < 1e-9f ? 0 : 1 / dY) : rYS;
                    float outX = rX > 0 ? ((bx + 1) * 2048 - x) * rX : rX < 0 ? (bx * 2048 - x) * rX : 1e9f;
                    float outY = rY > 0 ? ((by + 1) * 2048 - y) * rY : rY < 0 ? (by * 2048 - y) * rY : 1e9f;
                    float sOut = s + std::max(0.0f, std::min(outX, outY));
                    float low = std::min(line, fmaf(sOut, gk + eh2f, fmaf(sOut * sOut, fall, gez)));
                    // Where the line, bent by the fall, is lowest.
                    float sLow = -(gk + eh2f) / (2 * fall);
                    if (sLow > s && sLow < sOut) {
                        low = std::min(low, fmaf(sLow, gk + eh2f, fmaf(sLow * sLow, fall, gez)));
                    }
                    // Its highest - or the airfield's ground, eased over it.
                    float most = blockMax[blk];
                    if (!ROUND && sOut >= easeLo && s < easeHi) {
                        most = std::max(most, groundUp);
                    }
                    if (most < low) {
                        int j = i;
                        while (j + 8 < steps && far[j + 8] < sOut) {
                            j += 8;
                        }
                        while (j + 1 < steps && far[j + 1] < sOut) {
                            j++;
                        }
                        if (j > i + 1) {
                            // On from the block's last step, as the sample
                            // before the next.
                            skips++;
                            i = j;
                            return false;
                        }
                    }
                }
            }
#ifdef BENCH3D
            size_t cell = (size_t)((iy >> 8) & K.rowMask) * gcols + (ix >> 8);
#else
            size_t cell = (size_t)(iy >> 8) * gcols + (ix >> 8);
#endif
            float most = cellMax[cell];
            if (!ROUND && s < easeHi && s >= easeLo) {
                most = std::max(most, groundUp);
            }
            if (most < line) {
                // Under the line of sight, all its cell.
#ifdef BENCH3D
                path0++;
#endif
                prev = i;
                if ((i & 15) == 15) {
                    float zh = zHighAt[i >> 4];
                    float zcHigh = fmaf(s, A, zh * Fz), ycHigh = fmaf(s, B, zh * Dz);
                    if (zh > 0 ? (zcHigh > 0.5f && f * ycHigh > k * zcHigh) : (A > 0 && f * B > k * A)) {
                        return true;
                    }
                }
                i++;
                return false;
            }
            hgt = between(ix, iy);
        }
#ifdef BENCH3D
        else {
            path6++;
        }
#endif
        hgt = eased(hgt, s);
        if (hgt >= line) {
            // Above the line of sight, or the line not sloped: put on the
            // screen, and the rows between it and the sample before filled.
            float dz = hgt - fmaf(s, eh2f, base[i]);
            float zc = fmaf(s, A, dz * Fz), yc = fmaf(s, B, dz * Dz);
            if (zc > 0.5f && (sloped || f * yc <= k * zc)) {
                float w = 1 / zc, rho = fmaf(yc * w, f, cyLess);
                int top = std::max(0, ceilInt(rho));
                if (top < filled) {
                    // Rows [top, filled): between the sample before and this,
                    // 1 / depth straight between theirs - as a plane's is.
                    float slopeW = 0, lastRho = 0, lastW = 0;
                    bool before = false;
                    if (prev == shown) {
                        lastRho = shownRho;
                        lastW = shownW;
                        before = true;
                    } else if (prev >= 0 || prev == -2) {
                        float lastS = prev >= 0 ? far[prev] : R.lastS, lastH = R.lastH;
                        float lastUnder = prev < 0 ? underOf(lastS) : fmaf(lastS, eh2f, base[prev]);
                        if (prev >= 0) {
#ifdef BENCH3D
                            path4 += prev != known;
#endif
                            lastH = prev == known ? knownH : heightAt(lastS);
                        }
                        float ldz = lastH - lastUnder;
                        float lzc = fmaf(lastS, A, ldz * Fz), lyc = fmaf(lastS, B, ldz * Dz);
                        if (lzc > 0.5f) {
                            lastW = 1 / lzc;
                            lastRho = fmaf(lyc * lastW, f, cyLess);
                            before = true;
                        }
                    }
                    if (before && lastRho - rho > 1e-3f) {
                        slopeW = (lastW - w) / (lastRho - rho);
                    }
                    float wr = fmaf((float)(filled - 1 - top), slopeW, std::min(fmaf((float)top - rho, slopeW, w), w));
#ifdef BENCH3D
                    path3++;
#endif
                    // The rows the drawing reads: even ones - every band's
                    // first among them, bands being an even number of rows -
                    // and the top; every one where bands are odd. Two rows
                    // up at a time over the even, as scene3d_march_flat goes.
                    auto up = [&] {
                        if (off == 0) {
                            off = bandRows - 1;
                            at += bandBack;
                        } else {
                            off--;
                            at--;
                        }
                    };
                    int r = filled - 1;
                    if (everyRow) {
                        for (; r >= top; r--) {
                            tz[at] = toBf16(wr);
                            wr -= slopeW;
                            up();
                        }
                    } else {
                        if (r & 1) {
                            if (r == top) {
                                tz[at] = toBf16(wr);
                            }
                            wr -= slopeW;
                            up();
                            r--;
                        }
                        while (r >= top) {
                            tz[at] = toBf16(wr);
                            wr -= slopeW;
                            if (r == top) {
                                up();
                                break;
                            }
                            if (r - 1 == top) {
                                up();
                                tz[at] = toBf16(wr);
                                up();
                                break;
                            }
                            wr -= slopeW;
                            if (off == 0) {
                                off = bandRows - 2;
                                at += bandBack - 1;
                            } else {
                                off -= 2;
                                at -= 2;
                            }
                            r -= 2;
                        }
                    }
                    filled = top;
                    if (top == 0) {
                        return true;
                    }
                    k = top - cyMore;
                    den = fmaf(-k, Fz, f * Dz);
                    sloped = den < -1e-4f;
                    gk = sloped ? fmaf(k, A, -(f * B)) / den : 0;
                    gkLine = sloped ? gk : -INFINITY;
                }
                shown = i;
                shownRho = rho;
                shownW = w;
            }
        }
#ifdef BENCH3D
        path2 += hgt >= line;
        path1 += hgt < line;
#endif
        prev = known = i;
        knownH = hgt;
        // Given up once the highest land about, this far off or further,
        // would be under the rows filled.
        if ((i & 15) == 15) {
            float zh = zHighAt[i >> 4];
            float zcHigh = fmaf(s, A, zh * Fz), ycHigh = fmaf(s, B, zh * Dz);
            if (zh > 0 ? (zcHigh > 0.5f && f * ycHigh > k * zcHigh) : (A > 0 && f * B > k * A)) {
                return true;
            }
        }
        i++;
        return false;
    };
    int i = R.first;
#if defined(ESP_PLATFORM)
#ifdef BENCH3D
    bool inC = g_terrainProbe & 8;  // the bench's check: C alone, to compare
#else
    constexpr bool inC = false;
#endif
    if (!inC && i < steps) {
        // The common ways in assembly: what it leaves at
        // step m.i - a block to test for skipping, a sample before to be
        // worked out again - done here a step at a time, then on in it.
        MarchState &m = *K.asmState;
        m.X0 = X0;
        m.XS = XS;
        m.Y0 = Y0;
        m.YS = YS;
        m.A = A;
        m.B = B;
        m.e2 = e2;
        m.eh2 = eh2;
        m.groundUp = groundUp;
        m.rXS = rXS;
        m.rYS = rYS;
        m.XS2 = XS2;
        m.YS2 = YS2;
        m.eh2f = eh2f;
        m.gez = gez;
        m.fall = fall;
        m.twoFall = 2 * fall;
        m.easeLoF = easeLo;
        m.easeHiF = easeHi;
        int iLo = stepFrom(far, steps, 0, easeLo), iHi = stepFrom(far, steps, iLo, easeHi);
        m.iEaseLo = m.iEaseLo0 = iLo < iHi ? iLo : 0x7FFFFFFF;
        m.iEaseHi = iHi;
        for (;;) {
            m.i = i;
            m.filled = filled;
            m.at = at;
            m.off = off;
            m.k = k;
            m.gk = gk;
            m.gkLine = gkLine;
            m.sloped = sloped;
            m.prev = prev;
            m.known = known;
            m.shown = shown;
            m.knownH = knownH;
            m.shownRho = shownRho;
            m.shownW = shownW;
            m.curBlk = curBlk;
            m.samples = samples;
            m.skips = skips;
            int more = ROUND ? scene3d_march_round(&m) : scene3d_march_flat(&m);
            i = m.i;
            filled = m.filled;
            at = m.at;
            off = m.off;
            k = m.k;
            gk = m.gk;
            gkLine = m.gkLine;
            sloped = m.sloped;
            prev = m.prev;
            known = m.known;
            shown = m.shown;
            knownH = m.knownH;
            shownRho = m.shownRho;
            shownW = m.shownW;
            curBlk = m.curBlk;
            samples = m.samples;
            skips = m.skips;
            if (!more || step(i) || i >= steps) {
                break;
            }
        }
    } else
#endif
    {
        while (i < steps && !step(i)) {
        }
    }
    counts.samples += samples;
    counts.rows += R.filled - filled;
    counts.skips += skips;
#ifdef BENCH3D
    counts.paths[0] += path0;
    counts.paths[1] += path1;
    counts.paths[2] += path2;
    counts.paths[3] += path3;
    counts.paths[4] += path4;
    counts.paths[5] += path5;
    counts.paths[6] += path6;
#endif
    return filled;
}

void terrainMarch() {
    uint64_t *stats = g_terrainStats[coreNow()];
    uint32_t marchClock = cycles();
    CACHE_BEGIN(cacheAtStart);
    const GroundCover &cover = g.cover;
    const groundpack::Window &W = *cover.window;
    const groundpack::TerrainGrid &G = *g.scene.terrainGrid;
    int w = g.w, h = g.h, cols = (w + 3) / 4, bandRows = g.bandRows;
    g.tCols = cols;
    // Each band's rows of every column together - band by band - as a band
    // reads them: column c's rows of band b from ((b cols) + c) bandRows on.
    g.tz.resize((size_t)((h + bandRows - 1) / bandRows) * cols * bandRows);
    g.tTop.resize(cols);
    float ex = g.eye.x, ey = g.eye.y, ez = g.eye.z, perF = 1 / g.f;
    const Vec3 F = g.forward, D = g.down, R = g.right;
    bool round = cover.round;
    float er = g.scene.earthRadius, groundUp = round ? 0 : g.scene.groundUpM;
    // The grid's place about the eye: flat, straight lines from it; round,
    // worked out from nine places 20km round it, to the second order.
    float U[6] = {}, V6[6] = {};
    if (!round) {
        U[0] = cover.ux * ex + cover.u0;
        U[1] = cover.ux;
        V6[0] = cover.vy * ey + cover.v0;
        V6[2] = cover.vy;
    } else {
        constexpr float H = 20000;
        float u[3][3], v[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float x = ex + (i - 1) * H, y = ey + (j - 1) * H;
                coverPoint({x, y, -(x * x + y * y) / (2 * er)}, er, u[i][j], v[i][j]);
            }
        }
        float(*grid[2])[3] = {u, v};
        float *out[2] = {U, V6};
        for (int k = 0; k < 2; k++) {
            float(*q)[3] = grid[k];
            out[k][0] = q[1][1];
            out[k][1] = (q[2][1] - q[0][1]) / (2 * H);
            out[k][2] = (q[1][2] - q[1][0]) / (2 * H);
            out[k][3] = (q[2][1] - 2 * q[1][1] + q[0][1]) / (2 * H * H);
            out[k][4] = (q[2][2] - q[2][0] - q[0][2] + q[0][0]) / (4 * H * H);
            out[k][5] = (q[1][2] - 2 * q[1][1] + q[1][0]) / (2 * H * H);
        }
    }
    g.tRound = round;
    memcpy(g.tMapU, U, sizeof U);
    memcpy(g.tMapV, V6, sizeof V6);
    float ue = U[0], ve = V6[0];
    // Haze: tau = d / 30km + (2km / V)(1 - e^(-d / 2km)), its share 1 - e^-tau.
    const float V = g.pal.visibility, NEAR_HAZE = 2000, FAR_V = std::max(30000.0f, V);
    auto tau = [&](float dist) { return dist / FAR_V + NEAR_HAZE / V * (1 - expf(-dist / NEAR_HAZE)); };
    // The governor's skimping: fewer samples, less far, as frames run long.
    const float skimp = clamp01(g.scene.terrainEffort);
    float reach = std::min(100000.0f * (1 - 0.3f * skimp), (3 - NEAR_HAZE / V) * FAR_V);
    {
        float one = 1;
        uint32_t bits;
        memcpy(&bits, &one, 4);
        int from = (int)(bits >> 21) + 4 * 3;  // from 8m
        uint8_t steps[64];
        for (int i = 0; i < 64; i++) {
            float dist = 8 * exp2f((i + 0.5f) / 4);
            steps[i] = (uint8_t)std::min(31, (int)(32 * (1 - expf(-tau(dist)))));
        }
        for (int i = 0; i < 1024; i++) {
            g.tHaze[i] = steps[std::min(63, std::max(0, i - from))];
        }
    }
    // The samples' distances, alike for every column, a fortieth further each
    // - and half a cell at least: the land's heights are straight between
    // its cells' corners, curved only by a cell's twist, two chords a cell
    // within a thirty-second of that - under a pixel 3km off - and its rows'
    // depths straight between samples as a plane's are, so that nearer than
    // that adds nothing but samples. And, flat, what a sample's height is set against at each, but
    // for the line of sight's slope: the eye's height above the sea and the
    // earth's fall that far, a seventh back for the light's bending.
    // Kept together: the assembly reads a step's base 1600 bytes on from its
    // distance.
    static float stepTables[2][MARCH_MOST];
    float *far = stepTables[0], *base = stepTables[1];
    constexpr int MOST = MARCH_MOST;
    constexpr float PER_TWO_R = 1 / (2 * 6371000.0f * 7 / 6);
    // A cell's least width, metres.
    float cellSize = std::min(1 / std::max(1e-9f, fabsf(U[1]) * groundpack::TerrainGrid::TILE_COLS),
                              1 / std::max(1e-9f, fabsf(V6[2]) * groundpack::TerrainGrid::TILE_ROWS));
    float eyeUp = std::max(1.0f, ez), leastStep = cellSize * (0.5f + 0.5f * skimp), eTwo = ex * ex + ey * ey;
    const float growth = 0.025f * (1 + skimp);
    int steps = 0;
    for (float s = std::min(20.0f, 0.5f * eyeUp); s < reach && steps < MOST; s += std::max(leastStep, s * growth)) {
        far[steps] = s;
        base[steps] = round ? fmaf(s * s + eTwo, 1 / (2 * g.scene.earthRadius), ez) : groundUp + ez + s * s * PER_TWO_R;
        steps++;
    }
    // The highest land about, from the eye; and the highest near it, for
    // where a column's land could first be on the screen.
    float zHigh = std::max((float)G.highest, groundUp) - groundUp - ez;
    const float FLAT_IN = 2500, FLAT_OUT = 5000;
    float e2 = ex * ex + ey * ey;
    // The grid's cells: x = (u + gu) 32 - 1/2, y = (v + gv) 48 - 1/2.
    constexpr int TC = groundpack::TerrainGrid::TILE_COLS, TR = groundpack::TerrainGrid::TILE_ROWS,
                  BLOCK = groundpack::TerrainGrid::BLOCK;
    float gu = (float)(W.west - G.west), gv = (float)(W.north - G.north);
    float zNear = zHigh, nearReach = 0;
    {
        float eyeX = (ue + gu) * TC - 0.5f, eyeY = (ve + gv) * TR - 0.5f;
        int bx = floorInt(eyeX / BLOCK), by = floorInt(eyeY / BLOCK), highest = -32768;
        bool all = true;
        for (int j = by - 2; j <= by + 2; j++) {
            for (int i = bx - 2; i <= bx + 2; i++) {
                if (i < 0 || j < 0 || i >= G.blockCols || j >= G.blockRows) {
                    all = false;
                } else {
                    highest = std::max(highest, (int)G.blockMax[j * G.blockCols + i]);
                }
            }
        }
        // Two blocks each way of the eye's: that far round it at least.
        if (all) {
            zNear = std::max((float)highest, groundUp) - groundUp - ez;
            nearReach = 2 * BLOCK * cellSize;
        }
    }
    uint16_t *tz = g.tz.data();
    // The eye over an airfield's flat ground: the rows below where each
    // column's ray leaves it see the plane, their depth the row's alone -
    // no march needed for them, the most of a column's samples at an
    // airfield, nearest first.
    bool onFlat = !round && e2 < FLAT_IN * FLAT_IN && ez > 0;
    g.tFlatTop.assign(cols, (int16_t)h);
    g.tFlatZ.resize(h);
    for (int y = 0; y < h; y++) {
        float dirZ = F.z + D.z * (y + 0.5f - g.cy) * perF;
        g.tFlatZ[y] = onFlat && dirZ < -1e-6f ? -ez / dirZ : 0;
    }
    MarchFrame K;
    K.far = far;
    K.base = base;
    K.steps = steps;
    K.heights = G.heights;
    K.cellMax = G.cellMax;
    K.blockMax = G.blockMax;
    K.gcols = G.cols;
    K.bcols = G.blockCols;
    K.rowMask = g_terrainProbe & 1 ? 3 : -1;
    K.xLim = (uint32_t)(G.cols - 1) << 8;
    K.yLim = (uint32_t)(G.rows - 1) << 8;
    K.ez = ez;
    K.groundUp = groundUp;
    K.e2 = e2;
    K.fall = round ? 1 / (2 * er) : PER_TWO_R;
    K.Fz = F.z;
    K.Dz = D.z;
    K.f = g.f;
    K.cy = g.cy;
    K.zHigh = zHigh;
    K.skipTo = 10 * BLOCK * cellSize;
    K.tz = tz;
    K.cols = cols;
    K.bandRows = bandRows;
    static float zHighAt[MARCH_MOST / 16 + 1];
    K.zHighAt = zHighAt;
    int profiled = 0;  // the column the profile next wanted for
    static MarchState asmState;
    {
        MarchState &m = asmState;
        m.far = far;
        m.steps = steps;
        m.heights = G.heights;
        m.cellMaxBytes = (int)((const uint8_t *)G.cellMax - (const uint8_t *)G.heights);
        m.rowBytes = G.cols * 2;
        m.xLim = K.xLim;
        m.yLim = K.yLim;
        m.bcols = G.blockCols;
        m.Fz = F.z;
        m.Dz = D.z;
        m.f = g.f;
        m.cyLess = g.cy - 0.5f;
        m.cyMore = g.cy + 0.5f;
        m.zHigh = zHigh;
        m.skipTo = K.skipTo;
        m.tz = tz;
        m.bandRows1 = bandRows - 1;
        m.bandBack = bandRows - 1 - cols * bandRows;
        m.flatIn = 2500;
        m.perRing = 1.0f / 2500;
        m.tiny = 1e-3f;
        m.notSloped = -1e-4f;
        m.minusInf = -INFINITY;
        m.three = 3;
        m.blockMax = G.blockMax;
        m.farOff = 1e9f;
        m.tinyStep = 1e-9f;
        m.zHighAt = zHighAt;
    }
    K.asmState = &asmState;
    MarchCounts counts;
    // Every column: its ray, where its march starts, the flat ground under
    // it, and its march.
    for (int c = 0; c < cols; c++) {
        MarchRay m;
        m.c = c;
        float a = (4 * c + 2 - g.cx) * perF;
        float dx = F.x + R.x * a, dy = F.y + R.y * a, lh = sqrtf(dx * dx + dy * dy);
        g.tTop[c] = (int16_t)h;
        if (lh < 1e-5f) {
            continue;
        }
        float hx = dx / lh, hy = dy / lh;
        m.A = hx * F.x + hy * F.y;
        m.B = hx * D.x + hy * D.y;
        float u1 = U[1] * hx + U[2] * hy, u2 = U[3] * hx * hx + U[4] * hx * hy + U[5] * hy * hy;
        float v1 = V6[1] * hx + V6[2] * hy, v2 = V6[3] * hx * hx + V6[4] * hx * hy + V6[5] * hy * hy;
        m.X0 = ((ue + gu) * TC - 0.5f) * 256;
        m.XS = u1 * TC * 256;
        m.XS2 = u2 * TC * 256;
        m.Y0 = ((ve + gv) * TR - 0.5f) * 256;
        m.YS = v1 * TR * 256;
        m.YS2 = v2 * TR * 256;
        m.eh2 = 2 * (ex * hx + ey * hy);
        m.lastS = -1;
        m.lastH = 0;
        // Where its ray crosses the ring the airfield's ground eases into
        // the land's, |eye + s h| < FLAT_OUT.
        m.easeLo = m.easeHi = -1;
        if (!round) {
            float disc = m.eh2 * m.eh2 - 4 * (e2 - FLAT_OUT * FLAT_OUT);
            if (disc > 0) {
                float root = sqrtf(disc);
                m.easeLo = (-m.eh2 - root) * 0.5f;
                m.easeHi = (-m.eh2 + root) * 0.5f;
            }
        }
        int first = 0;
        // From where the bottom row's ray could first meet the land: the
        // highest near the eye out to the near reach, the highest there is
        // beyond - nearer, all of it under the screen.
        float b = (h - g.cy) * perF;
        float bz = F.z + D.z * b, bx = F.x + R.x * a + D.x * b, by = F.y + R.y * a + D.y * b;
        if (bz < -1e-4f) {
            float along = sqrtf(bx * bx + by * by) / bz * 0.9f;
            float sStart = zHigh < 0 ? zHigh * along : 0;
            if (zNear < 0) {
                sStart = std::max(sStart, std::min(nearReach, zNear * along));
            }
            first = stepFrom(far, steps, first, sStart);
        }
        int filled = h;
        if (onFlat) {
            // Where its ray leaves the flat ground, |eye + s h| = FLAT_IN.
            float b2 = m.eh2 * m.eh2 - 4 * (e2 - FLAT_IN * FLAT_IN), sExit = (-m.eh2 + sqrtf(std::max(0.0f, b2))) * 0.5f;
            float zcExit = sExit * m.A - ez * F.z, ycExit = sExit * m.B - ez * D.z;
            if (zcExit > 0.5f) {
                // The plane's rows: not written, the drawing has their depths
                // from tFlatZ - a plane's own, not straight between two as
                // the land's are.
                filled = std::min(filled, std::max(0, ceilInt(g.cy + g.f * ycExit / zcExit - 0.5f)));
                g.tFlatTop[c] = (int16_t)filled;
                m.lastS = sExit;
                m.lastH = groundUp;
                first = stepFrom(far, steps, first, sExit);
            }
        }
        m.first = first;
        m.filled = filled;
        if (filled > 0 && first < steps && c >= profiled) {
            // The highest land along its ray from each sixteenth step on, from
            // the eye: what it gives up against - the land's own along it,
            // not the highest there is - for it and the three columns after:
            // their rays within a block of its, 100km out. Read a block
            // apart, from the far end in, each block with those round it:
            // none passed by.
            profiled = c + 4;
            constexpr int j0 = 0;  // from the first: the three after may start nearer
            int most = -32768;
            float sAt = far[steps - 1], aBlock = BLOCK * cellSize;
            for (int j = (steps - 1) >> 4; j >= j0; j--) {
                float sTo = far[std::min(steps - 1, 16 * j + 15)];
                for (;; sAt -= aBlock) {
                    float sNow = std::max(sAt, sTo);
                    float x = round ? m.X0 + sNow * (m.XS + sNow * m.XS2) : m.X0 + sNow * m.XS;
                    float y = round ? m.Y0 + sNow * (m.YS + sNow * m.YS2) : m.Y0 + sNow * m.YS;
                    int bx = (int)x >> 11, by = (int)y >> 11;  // just off the grid's west or north, its first: higher, if anything
                    most = std::max(most, (unsigned)bx < (unsigned)G.blockCols && (unsigned)by < (unsigned)G.blockRows
                                              ? (int)G.wideMax[by * G.blockCols + bx]
                                              : 0);
                    if (sNow <= sTo) {
                        break;
                    }
                }
                zHighAt[j] = std::max((float)most, groundUp) - groundUp - ez;
            }
        }
        if (filled > 0 && first < steps) {
            filled = round ? marchColumn<true>(K, m, counts) : marchColumn<false>(K, m, counts);
        }
        g.tTop[c] = (int16_t)filled;
    }
    stats[1] += counts.samples;
    stats[2] += counts.rows;
    stats[3] += counts.skips;
    g_marchPaths = counts;
    g.tFlatAll = h;
    for (int c = 0; c < cols && onFlat; c++) {
        g.tFlatAll = c == 0 ? g.tFlatTop[c] : std::max(g.tFlatAll, (int)g.tFlatTop[c]);
    }
    if (!onFlat) {
        g.tFlatAll = 1 << 30;
    }
    // For the drawing: each row's and column's part in where a depth puts
    // the land on the grid; how far a metre of depth goes along each row's
    // ray; and the depth each level is read to - its cells four pixels wide
    // or more there, a sample's.
    g.tUe = ue;
    g.tVe = ve;
    g.tRowU.resize(h + 1);  // and one over: the assembly reads a pair's odd row's with its even's
    g.tRowV.resize(h + 1);
    g.tRowFar.resize(h + 1);
    g.tColU.resize(cols);
    g.tColV.resize(cols);
    g.tRowX.resize(h + 1);
    g.tRowY.resize(h + 1);
    g.tColX.resize(cols);
    g.tColY.resize(cols);
    g.tFlatZ.resize(h + 1);
    for (int y = 0; y <= h; y++) {
        float b = (y + 0.5f - g.cy) * perF;
        g.tRowU[y] = U[1] * (F.x + D.x * b);
        g.tRowV[y] = V6[2] * (F.y + D.y * b);
        g.tRowX[y] = F.x + D.x * b;
        g.tRowY[y] = F.y + D.y * b;
        g.tRowFar[y] = sqrtf(1 + b * b);
    }
    g.tFlatZ[h] = 0;
    g.tColUV.resize(2 * cols);
    g.tColXY.resize(2 * cols);
    g.tTopFlat.resize(cols);
    for (int c = 0; c < cols; c++) {
        float a = (4 * c + 2 - g.cx) * perF;
        g.tColU[c] = U[1] * R.x * a;
        g.tColV[c] = V6[2] * R.y * a;
        g.tColX[c] = R.x * a;
        g.tColY[c] = R.y * a;
        g.tColUV[2 * c] = g.tColU[c];
        g.tColUV[2 * c + 1] = g.tColV[c];
        g.tColXY[2 * c] = g.tColX[c];
        g.tColXY[2 * c + 1] = g.tColY[c];
        g.tTopFlat[c] = (uint16_t)g.tTop[c] | (uint32_t)(uint16_t)g.tFlatTop[c] << 16;
    }
    // A cell of level L is 2^L of level 0's, 2.5 seconds of latitude, some
    // 77m; a sample is four pixels, z * 4 / f metres at depth z.
    for (int L = groundpack::FINEST; L < groundpack::LEVELS; L++) {
        g.tLevelZ[L - groundpack::FINEST] = 77.0f * ldexpf(1.0f, L) * g.f / 4;
    }
    g.tUnit = skimp > 0.6f ? 4 : 2;
    g.tLevelPer = (g.tUnit == 4 ? 2.0f : 1.0f) / g.tLevelZ[0];
    stats[0] += cycles() - marchClock;
    CACHE_END(cacheAtStart, g_terrainCache[coreNow()]);
}

void prepareBegin(int w, int h, int bandRows, bool swapBytes, const Scene &scene, const Camera &camera) {
    uint32_t t0 = now();
    const Models &m = models();
    g.scene = scene;
    g.pal = paletteFor(scene.sun.elevationDeg);
    if (scene.visibility > 0) {
        g.pal.visibility = scene.visibility;
    }
    if (scene.earthRadius > 0) {
        // Up where the air is thin the sky overhead is a deeper blue; and
        // the ground, sea and land as one at this distance, a grey-green.
        float k = clamp01(camera.eye.z / 14000);
        g.pal.skyTop = scale(g.pal.skyTop, 1 - 0.35f * k);
        g.pal.grass = mix(g.pal.grass, {62, 84, 96}, 0.7f);
    }
    const Palette &pal = g.pal;
    float az = scene.sun.azimuthDeg * DEG, el = scene.sun.elevationDeg * DEG;
    g.sunDir = {cosf(el) * sinf(az), cosf(el) * cosf(az), sinf(el)};
    g.sunElevationDeg = scene.sun.elevationDeg;
    // Lit from the sun while it is up; after, by what is left of the sky's
    // light, from high on the side it went down.
    g.sun = scene.sun.elevationDeg > 0 ? g.sunDir : normalize({sinf(az) * 0.6f, cosf(az) * 0.6f, 0.8f});
    // Shadows fade as the sun gets low - long, faint, then gone.
    float shadowing = clamp01((scene.sun.elevationDeg - 3) / 5);
    g.shadows = shadowing > 0;
    g.shadow256 = (int)((1 - (1 - pal.shadow) * shadowing) * 256);
    for (int i = 0; i < 8; i++) {
        g.shadowLanes[0][i] = (uint16_t)g.shadow256;
        g.shadowLanes[1][i] = 0xF800;
        g.shadowLanes[2][i] = 0x07E0;
        g.shadowLanes[3][i] = 0x001F;
    }
    g.w = w;
    g.h = h;
    g.bandRows = bandRows;
    g.bands = (h + bandRows - 1) / bandRows;
    g.swap = swapBytes;
    g.cx = w / 2.0f;
    g.cy = h / 2.0f;
    g.f = g.cx / tanf(camera.fovDeg * DEG / 2);
    g.gx = (g.cx + GUARD) / g.f;
    g.gy = (g.cy + GUARD) / g.f;
    g.wx = (g.cx + WIDE) / g.f;
    g.wy = (g.cy + WIDE) / g.f;
    g.sx = 1 / sqrtf(1 + g.gx * g.gx);
    g.sy = 1 / sqrtf(1 + g.gy * g.gy);
    g.eye = camera.eye;
    g.forward = normalize(camera.target - camera.eye);
    g.right = cross(g.forward, {0, 0, 1});
    g.right = length(g.right) > 1e-4f ? normalize(g.right) : Vec3{0, -1, 0};
    g.down = cross(g.forward, g.right);
    g.gridX = Vec3{g.right.x, g.down.x, g.forward.x} * m.unit;
    g.gridY = Vec3{g.right.y, g.down.y, g.forward.y} * m.unit;
    g.gridZ = Vec3{g.right.z, g.down.z, g.forward.z} * m.unit;
    g.counts = {};
    uint32_t mark = t0;
    auto lap = [&](Step step) {
        uint32_t n = now();
        g.counts.stepUs[step] += n - mark;
        mark = n;
    };
    // The haze warmest looking towards a low sun, cooler looking away: a sky
    // can't vary across a row here, but it can with which way the view faces.
    float facing = std::max(0.0f, cosf(atan2f(g.forward.x, g.forward.y) - az));
    float glow = clamp01(1 - fabsf(scene.sun.elevationDeg - 2) / 14);
    g.pal.haze = mix(mix(pal.haze, pal.skyTop, 0.4f), pal.haze, facing * glow + (1 - glow));
    // The camera placed and the light worked out: the buildings ready for
    // another core to do while this one gets on.
    g_solids.store(Solids::READY, std::memory_order_release);
    if (g_solidsReady != nullptr) {
        g_solidsReady();
    }

    // Each row's sky or grass: the grass hazed by how far off the ground is
    // that the row looks at, the sky bluer the higher the row looks.
    g.rowColor.resize(h);
    g.rowHaze.resize(h);
    g.rowGround.assign(h, 0);
    g.rowLand.resize(h);
    g.rowTown.resize(h);
    float groundLight = pal.ambient + pal.diffuse * std::max(0.0f, g.sun.z);
    Rgb grass = scale(pal.grass, groundLight);
    // High up, the sea a deep blue-grey, the land - where the coasts say it
    // is - a muted green, autumn's brown in it. Low down at an airfield by
    // the sea or a loch, the land its grass and the water the same blue-grey.
    Rgb sea = scale({44, 66, 90}, groundLight), land = scale({86, 98, 70}, groundLight);
    // A town, the grey of its roofs and roads with the brown of the older
    // ones - as near as one colour can say it from a few thousand feet; and
    // after dark, far off, the glow of its lights, sodium's orange, where
    // they run together - near, the lights themselves, strewn over it (see
    // lightTowns()), on the dark.
    Rgb town = scale({124, 118, 108}, groundLight), townGlow = {72, 46, 22};
    // What covers the land, each cover's colour lit as the land is - see
    // GroundCover: none, trees, shrubs, grass, crops, built over, bare, snow
    // and ice, water, marsh, mangroves, moss and lichen; and the mixes -
    // suburbs, towns' middles, moor, rocky ground. A wet country's: the
    // grass a deep green, fields more green than gold, the shrubs the brown
    // green of heather, bare ground grey rock; the suburbs' roofs among
    // gardens and trees a grey green, a town's middle a darker grey, the
    // moor's deer grass and bracken a russet brown.
    static const Rgb COVER_RGB[groundpack::COVERS] = {
        {86, 98, 70},   {40, 72, 42},    {94, 90, 64},   {80, 118, 58},   {126, 128, 76},  {124, 118, 108},
        {128, 124, 112}, {236, 240, 244}, {44, 66, 90},   {70, 90, 70},    {40, 84, 54},    {102, 110, 88},
        {100, 106, 84}, {112, 106, 100}, {108, 100, 64}, {118, 116, 100}};
    g.cover = scene.cover;
    bool coverOn = g.cover.window != nullptr && g.cover.window->rows > 0;
    g.coverLat0 = (float)g.cover.lat0;
    g.coverLon0 = (float)g.cover.lon0;
    g.coverSin = sinf(g.coverLat0 * DEG);
    g.coverCos = cosf(g.coverLat0 * DEG);
    for (int c = 0; c < groundpack::COVERS; c++) {
        Rgb lit = scale(COVER_RGB[c], groundLight);
        g.coverLit[c][0] = (uint8_t)std::min(255.0f, std::max(0.0f, lit.r));
        g.coverLit[c][1] = (uint8_t)std::min(255.0f, std::max(0.0f, lit.g));
        g.coverLit[c][2] = (uint8_t)std::min(255.0f, std::max(0.0f, lit.b));
    }
    g.coverOn = coverOn;
    g.terrainOn = coverOn && scene.terrain && scene.terrainGrid != nullptr && scene.terrainGrid->heights != nullptr;
    g.perW = 1.0f / w;
    if (coverOn && !g.cover.round) {
        Frame::CoverFlat &k = g.coverFlat;
        const GroundCover &c = g.cover;
        k.perF = 1 / g.f;
        k.sy0 = (0.5f - g.cy) * k.perF;
        k.fz = g.forward.z;
        k.dz1 = g.down.z;
        k.ez = g.eye.z > 0 ? g.eye.z : 0;  // under the ground: no row reaches it
        k.ku = c.ux * g.right.x * k.perF;
        k.kv = c.vy * g.right.y * k.perF;
        k.au = c.ux * g.eye.x + c.u0;
        k.av = c.vy * g.eye.y + c.v0;
        k.bu = c.ux * g.forward.x - k.ku * g.cx;
        k.bv = c.vy * g.forward.y - k.kv * g.cx;
        k.cu = c.ux * g.down.x;
        k.cv = c.vy * g.down.y;
    }
    g.rowCover.resize(coverOn && g.cover.round ? (h + 1) / 2 : 0);
    for (int cell = 0; coverOn && cell < 256; cell++) {
        int k = (cell & 15) * 256 / 11;  // shade 11 is flat ground
        for (int i = 0; i < 3; i++) {
            g.coverRgb[cell][i] = (uint8_t)std::min(255, g.coverLit[cell >> 4][i] * k >> 8);
        }
    }
    float dark = clamp01((0.36f - groundLight) / 0.18f);
    g.coverTowns = coverOn && dark < 0.5f;
    bool landRows = scene.landAndSea && !m.groundLines.empty(), townRows = landRows && m.towns;
    if (scene.earthRadius <= 0) {
        land = grass;
    }
    if (scene.earthRadius > 0 || landRows) {
        grass = sea;
    }
    // High up, where a row meets the round earth: a ray from p, the eye from
    // the earth's middle, meets it t along where t² + 2bt + c = 0 - b the
    // ray's length along p, c = |p|² - r². Not worked out as that, which in
    // a float loses the metres to the radius squared, but as what it comes
    // to, x² + y² + z(z + 2r); and the nearer root as c / (-b + √(b² - c)),
    // which keeps them too. In float, which the P4 does in its FPU: in
    // double, done in software, the rows took 8ms a frame.
    // The land's colour too, a row at a time, only where the land is drawn
    // apart from the sea - see groundAndCoasts(): the dithering of each is
    // a good part of the rows' time.
    float er = scene.earthRadius, ex = g.eye.x, ey = g.eye.y, ez = g.eye.z;
    float above = ex * ex + ey * ey + ez * (ez + 2 * er), perVisibility = 1 / pal.visibility;
    for (int y = 0; y < h; y++) {
        Vec3 dir = g.forward + g.down * ((y + 0.5f - g.cy) / g.f);
        float l = length(dir), distance = -1;
        if (er > 0) {
            float b = (ex * dir.x + ey * dir.y + (ez + er) * dir.z) / l, disc = b * b - above;
            if (disc >= 0 && b < 0 && above > 0) {
                distance = above / (-b + sqrtf(disc));
            }
        } else if (dir.z < 0 && g.eye.z > 0) {
            distance = g.eye.z / -dir.z * l;
        }
        if (coverOn && g.cover.round && !g.terrainOn && !(y & 1)) {
            Frame::RowCover rc;
            rc.level = Frame::NO_LEVEL;
#if defined(BENCH3D) && defined(ESP_PLATFORM)
            uint64_t *stats = g_coverStats[xPortGetCoreID() & 1];
            uint32_t clock = esp_cpu_get_cycle_count();
#endif
            if (distance > 0) {
                coverAnchors(y, dir, er, rc);
#if defined(BENCH3D) && defined(ESP_PLATFORM)
                stats[8]++;
#endif
            }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
            uint32_t mid = esp_cpu_get_cycle_count();
            stats[7] += mid - clock;
            g.rowCover[y >> 1] = rc;
            stats[9] += esp_cpu_get_cycle_count() - mid;
#else
            g.rowCover[y >> 1] = rc;
#endif
        }
        if (distance > 0) {
            float haze = 1 - expf(-distance * perVisibility);
            g.rowColor[y] = pair(mix(grass, pal.haze, haze), y);
            if (landRows) {
                g.rowLand[y] = pair(mix(land, pal.haze, haze), y);
            }
            if (townRows) {
                Rgb lit = mix(town, townGlow, dark * clamp01((distance - 3000) / 20000));
                g.rowTown[y] = pair(mix(lit, pal.haze, haze), y);
            }
            g.rowHaze[y] = (uint16_t)(haze * 256);
            g.rowGround[y] = 1;
        } else {
            // Bluer the higher the row looks: its elevation, as near as a
            // gradient needs, from its sine - arcsine's series to the fifth.
            float sine = std::max(0.0f, dir.z / l);
            float elevation = sine + sine * sine * sine * (1.0f / 6 + sine * sine * (3.0f / 40));
            g.rowColor[y] = pair(mix(pal.haze, pal.skyTop, sqrtf(std::min(1.0f, elevation / (40 * DEG)))), y);
            g.rowHaze[y] = 256;
        }
    }
    // The land at its height where the view has it, flat at an airfield for
    // now - and then no skyline, the hills the land's own - or the hills
    // along the skyline.
    // A frame's serial from the one count, not its own: the two frames
    // flip, and each counting for itself had the drawing see 1, 1, 2, 2 -
    // the colours' caches kept from the frame before every second frame.
    static uint32_t s_serial = 0;
    g.serial = ++s_serial;
    if (g.terrainOn) {
        for (Frame::HillRange &range : g.hills) {
            range.on = false;
        }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
        uint32_t marchClock = esp_cpu_get_cycle_count();
        terrainMarch();
        g_coverStats[xPortGetCoreID() & 1][10] += esp_cpu_get_cycle_count() - marchClock;
#else
        terrainMarch();
#endif
    } else {
#if defined(BENCH3D) && defined(ESP_PLATFORM)
        uint32_t skylineClock = esp_cpu_get_cycle_count();
        skyline(scene, groundLight);
        g_coverStats[xPortGetCoreID() & 1][10] += esp_cpu_get_cycle_count() - skylineClock;
#else
        skyline(scene, groundLight);
#endif
    }
    lap(STEP_ROWS);

    // The ground, lit all alike, hazed a row at a time as it is drawn.
    g.ground.clear();
    // A marking narrower than half a pixel at its nearest corner is left out:
    // a runway's numbers and bars, a mile off. (The thin ones - taxiway
    // lines, runway edges - are addPaint's, after.)
    for (const ModelTri &t : m.farGround) {
        if (overHorizon(t.v[0])) {
            continue;
        }
        Rgb lit = scale(t.color, groundLight);
        addTriangle(t.v[0], t.v[1], t.v[2], g.ground, [&](ScreenTri &s) {
            s.lit[0] = (uint8_t)std::min(255.0f, lit.r);
            s.lit[1] = (uint8_t)std::min(255.0f, lit.g);
            s.lit[2] = (uint8_t)std::min(255.0f, lit.b);
        });
    }
    lap(STEP_FAR_GROUND);
    // The rivers, under the airfield's ground - a river by a runway runs
    // under its grass, as through a culvert - and the colour of the sea.
    float riverReach = scene.earthRadius > 0 ? LINES_REACH : 1e30f;
    for (const Cell &c : m.riverCells) {
        float dx = c.middle.x - g.eye.x, dy = c.middle.y - g.eye.y;
        if (outOfView(c) || sqrtf(dx * dx + dy * dy) - c.radius > riverReach) {
            continue;
        }
        for (uint32_t i = c.begin; i < c.end; i++) {
            const PaintLine &river = m.rivers[i];
            if (!overHorizon(river.a) || !overHorizon(river.b)) {
                addPaint(river, groundLight, &land);
            }
        }
    }
    lap(STEP_RIVERS);
    // A cell wholly in view - most of a big airfield's, from the approach -
    // its triangles straight to the screen, unclipped.
    for (const Cell &c : m.groundCells) {
        if (c.lod == LOD_GRASS && !landRows) {
            continue;
        }
        float in = ballInside(toCamera(c.middle));
        if (in < -c.radius) {
            continue;
        }
        if (c.lod == LOD_NEAR || c.lod == LOD_FAR) {
            float away = length(c.middle - g.eye);
            if (c.lod == LOD_NEAR ? away - c.radius > COARSE_FROM + COARSE_OVERLAP
                                  : away + c.radius < COARSE_FROM - COARSE_OVERLAP) {
                continue;
            }
        }
        bool whole = in > c.radius;
        GridBase base = gridBase(c.middle, m.unit);
        for (uint32_t i = c.begin; i < c.end; i++) {
            const GroundTri &t = m.groundTris[i];
            Vec3 c0 = seenOnGrid(base, t.x[0], t.y[0], 0), c1 = seenOnGrid(base, t.x[1], t.y[1], 0),
                 c2 = seenOnGrid(base, t.x[2], t.y[2], 0);
            if (t.fine < 100) {
                float reach = g.f * t.fine * 2, nearest = std::min({dot(c0, c0), dot(c1, c1), dot(c2, c2)});
                if (nearest > reach * reach) {
                    continue;
                }
            }
            addInCell(c0, c1, c2, whole, g.ground, [&](ScreenTri &s) {
                Rgb lit = t.grass ? land : scale({(float)t.rgb[0], (float)t.rgb[1], (float)t.rgb[2]}, groundLight);
                s.lit[0] = (uint8_t)std::min(255.0f, lit.r);
                s.lit[1] = (uint8_t)std::min(255.0f, lit.g);
                s.lit[2] = (uint8_t)std::min(255.0f, lit.b);
            });
        }
    }
    lap(STEP_GROUND);
    g.wires.clear();  // the frame's lines: the thin paint's first
    // The squares in view - the short pieces only in those near, and the
    // thin lines, a taxiway's centre, only within 1.5km: past that they are a
    // tenth of a pixel, faded almost to nothing, and a big airport has miles
    // of them. The runways' broader lines, any distance.
    for (const Cell &c : m.paintCells) {
        float away = length(c.middle - g.eye) - c.radius;
        if ((c.lod == PAINT_DASHES && away >= 600) || (c.lod == PAINT_THIN && away >= 1500) || outOfView(c)) {
            continue;
        }
        for (uint32_t i = c.begin; i < c.end; i++) {
            addPaint(m.paintLines[i], groundLight);
        }
    }
    lap(STEP_PAINT);

    // The aircraft, nearest first - so that where one hides another, the one
    // behind fails the depth test pixel by pixel rather than being drawn and
    // then drawn over - each at the detail its size on the screen calls for,
    // and none at all that is out of view.
    g.placed.clear();
    auto consider = [&](const Aircraft &aircraft, bool followed, int index) {
        const Pose &pose = aircraft.pose;
        const Design &dsn = designOf(aircraft.look);
        float size = aircraft.look.scale;
        Vec3 middle = Placer(pose, size).place(dsn.middle);
        Vec3 c = toCamera(middle);
        float radius = dsn.radius * size;
        float in = ballInside(c);
        if (in < -radius) {
            return;
        }
        float px = g.f * dsn.halfLength * 1.17f * size / std::max(c.z, NEAR);
        // The one being followed drawn in full where it is big enough on the
        // screen for the detail to show - it is what the view is of - and a
        // step down where it is small, a tower's or a long lens's: the full
        // model cost 3-5ms a frame there for nothing seen. With a margin each
        // way, so it doesn't flick between them.
        static int followedDetail = DETAIL_HERO;
        if (followed) {
            if (followedDetail == DETAIL_HERO && px < 130) {
                followedDetail = DETAIL_MEDIUM;
            } else if (followedDetail == DETAIL_MEDIUM && px >= 170) {
                followedDetail = DETAIL_HERO;
            } else if (followedDetail == DETAIL_MEDIUM && px < 45) {
                followedDetail = DETAIL_SIMPLE;
            } else if (followedDetail == DETAIL_SIMPLE && px >= 60) {
                followedDetail = DETAIL_MEDIUM;
            }
        }
        int detail = px < DOT_BELOW_PX     ? DETAIL_LIGHTS
                     : followed               ? followedDetail
                     : px >= scene.detailPx   ? DETAIL_MEDIUM
                     : px >= scene.tinyPx     ? DETAIL_SIMPLE
                                              : DETAIL_TINY;
        float iz = c.z > NEAR ? 1 / c.z : 0;
        Vec3 top = toCamera(Placer(pose, size).place({dsn.middle.x, 0, dsn.hi.z}));
        int16_t px0 = -1, py0 = -1, px1 = -1, py1 = -1;
        if (c.z > NEAR) {
            px0 = (int16_t)std::max(-1.0f, std::min(32000.0f, g.cx + g.f * c.x * iz));
            py0 = (int16_t)std::max(-1.0f, std::min(32000.0f, g.cy + g.f * c.y * iz));
        }
        if (top.z > NEAR) {
            px1 = (int16_t)std::max(-1.0f, std::min(32000.0f, g.cx + g.f * top.x / top.z));
            py1 = (int16_t)std::max(-1.0f, std::min(32000.0f, g.cy + g.f * top.y / top.z));
        }
        g.placed.push_back({pose, aircraft.look, aircraft.config, index, length(middle - g.eye), detail, followed,
                            in > radius, g.cx + g.f * c.x * iz, g.cy + g.f * c.y * iz,
                            g.f * dsn.halfLength * 0.96f * size * iz, {px0, px1}, {py0, py1},
                            1 / std::max(NEAR, c.z - radius), {false, false}});
    };
    consider(scene.followed, true, -1);
    heroOf(scene.followed.look.family);
    for (int i = 0; i < scene.trafficCount; i++) {
        consider(scene.traffic[i], false, i);
    }
    lap(STEP_PLACE);
    std::sort(g.placed.begin(), g.placed.end(),
              [](const Placed &a, const Placed &b) { return a.distance < b.distance; });
    for (Part &p : g.parts) {
        p.solid.clear();
        p.shadow.clear();
        p.clear.clear();
        p.sprites.clear();
        p.nearest = 1e9f;
        p.counts = {};
    }
    lap(STEP_SORT);
    g.counts.beginUs = now() - t0;
}

void prepareAircraft(int part, int parts) {
    uint32_t t0 = now();
    Part &out = g.parts[part];
    uint32_t followed = 0;
    for (size_t i = part; i < g.placed.size(); i += parts) {
        if (g.placed[i].followed) {
            uint32_t f0 = now();
            addAircraft(g.placed[i], out);
            followed = now() - f0;
        } else {
            addAircraft(g.placed[i], out);
        }
    }
    out.counts.aircraftUs[0] = now() - t0;
    out.counts.stepUs[STEP_FOLLOWED] += followed;
    out.counts.stepUs[STEP_TRAFFIC] += out.counts.aircraftUs[0] - followed;
}

// The country below high up: its lines drawn - coasts pale, as surf and sand
// show them from high up; lakes' shores the same; borders and state lines
// fainter; none the earth's curve hides - and, from the coasts and shores,
// where the land is along each ground row, and from the towns' outlines,
// where the towns are. Each is a layer of its own, worked out alike. A coast
// or a shore has the land on its left, going along it, unless it says
// otherwise, and a town's outline the town: so crossing a row to the right,
// one going down the screen leads onto it and one going up off it. Before a
// row's first crossing it is whatever that crossing leaves; a row none of a
// layer's lines cross, what the row below it is in the middle, unless one
// crosses the middle between the two - and the lowest, what is under the
// nearest point of the nearest line.
namespace {
constexpr float TOUCHING = 0.25f;  // pixels: two crossings nearer than this are a touch

// Which layer of the ground a line of it is a side of: the land's, the
// towns', or neither.
constexpr uint8_t SHORES = 0, TOWNS = 1, NO_LAYER = 255;
uint8_t layerOf(GroundKind kind) {
    return kind == GroundKind::COAST || kind == GroundKind::LAKE ? SHORES : kind == GroundKind::TOWN ? TOWNS : NO_LAYER;
}

// What is under a point of the ground in a layer, by the side of the nearest
// of its lines the point is on - kept, and looked for again only once the
// point could have crossed one: has moved as far as the nearest was from it,
// whether the eye moved or the view turned. (It was once only when the eye
// had moved a couple of kilometres: a view dragged round near a coast kept
// the side it had been on, and showed land and sea swapped.) And afresh for
// a new country's lines.
struct Side {
    Vec3 at{1e30f, 0, 0};
    float safe = 0;
    bool inside = false;
    const GroundLine *linesWere = nullptr;
    size_t countWas = 0;
};

bool sideOf(Side &side, uint8_t layer, Vec3 p, bool none) {
    const Models &m = models();
    if (m.groundLines.data() != side.linesWere || m.groundLines.size() != side.countWas) {
        side.linesWere = m.groundLines.data();
        side.countWas = m.groundLines.size();
        side.at = {1e30f, 0, 0};
    }
    float mx = p.x - side.at.x, my = p.y - side.at.y;
    if (mx * mx + my * my < side.safe * side.safe) {
        return side.inside;
    }
    side.at = p;
    float best = 1e30f;
    // The nearest point of a line can be a corner of it, the end of one line
    // and the start of the next - and which side of it p is can't be told
    // from either line alone: off a narrow headland, p is on the land's side
    // of one of them. So each line's inside is kept as a normal, and at a
    // corner they are added, the lines' ends being the same points to the
    // bit.
    bool corner = false;
    float nearX = 0, nearY = 0, inX = 0, inY = 0;
    auto scan = [&](const Cell &c) {
        for (uint32_t i = c.begin; i < c.end; i++) {
            const GroundLine &l = m.groundLines[i];
            if (layerOf(l.kind) != layer) {
                continue;
            }
            float ex = l.b.x - l.a.x, ey = l.b.y - l.a.y, len2 = ex * ex + ey * ey;
            if (len2 <= 0) {
                continue;
            }
            // How far along from a the nearest point is, and how far off the
            // line p is - and the nearest point taken as an end when it is
            // within a hair of one: 20km off, a float's squared distance
            // can't tell a point 10m short of a corner from the corner, and
            // one line's corner said a town and the other line, nearer by
            // nothing it could see, was passed over (off the Clyde).
            float len = sqrtf(len2), rx = p.x - l.a.x, ry = p.y - l.a.y;
            float along = (rx * ex + ry * ey) / len, off = fabsf(rx * ey - ry * ex) / len;
            float hair = std::max(1.0f, off * 5e-4f);
            bool atA = along <= hair && along <= len - along, atB = !atA && along >= len - hair;
            bool end = atA || atB;
            float u = along / len;
            float qx = atA ? l.a.x : atB ? l.b.x : l.a.x + ex * u;
            float qy = atA ? l.a.y : atB ? l.b.y : l.a.y + ey * u;
            float d2 = (qx - p.x) * (qx - p.x) + (qy - p.y) * (qy - p.y);
            float k = (l.landRight ? -1 : 1) / len, nx = -ey * k, ny = ex * k;
            if (d2 < best) {
                best = d2;
                corner = end;
                nearX = qx;
                nearY = qy;
                inX = nx;
                inY = ny;
            } else if (end && corner && qx == nearX && qy == nearY) {
                inX += nx;
                inY += ny;
            }
        }
    };
    // How near p a cell's lines can come.
    auto gap = [&](const Cell &c) {
        float dx = c.middle.x - p.x, dy = c.middle.y - p.y;
        return std::max(0.0f, sqrtf(dx * dx + dy * dy) - c.radius);
    };
    // The nearest cell first, so the rest - all but a few - are passed over
    // by the line found in it; then those as near, a metre over, for a
    // corner shared with a cell at the same distance.
    const Cell *first = nullptr;
    float firstGap = 1e30f;
    for (const Cell &c : m.lineCells) {
        float gc = gap(c);
        if (gc < firstGap) {
            firstGap = gc;
            first = &c;
        }
    }
    if (first != nullptr) {
        scan(*first);
    }
    for (const Cell &c : m.lineCells) {
        if (&c != first && gap(c) <= sqrtf(best) + 1) {
            scan(c);
        }
    }
    // None of the layer's lines at all: all land, inland, or no town.
    side.inside = best < 1e30f ? (p.x - nearX) * inX + (p.y - nearY) * inY > 0 : none;
    side.safe = best < 1e30f ? sqrtf(best) : 1e30f;
    return side.inside;
}

void groundAndCoasts() {
    const Models &m = models();
    int h = g.h;
    g.landSea = false;
    if (m.groundLines.empty()) {
        return;
    }
    float r = g.scene.earthRadius, eyeUp = std::max(1.0f, heightOf(g.eye));
    g_groundReach2 = r > 0 ? 2 * r * eyeUp : 0;
    bool fill = g.scene.landAndSea;
    // Each line's crossing of the rows on the screen, once: where it is on
    // the first row it crosses, how far along a row it moves a row, its rows,
    // which layer, and which way it leads.
    struct Crosser {
        float x, slope;
        int16_t top, bottom;
        uint8_t layer;
        bool onto;
    };
    static Vector<Crosser> crossers;
    crossers.clear();
    // And where each crosses the middle of the screen, top to bottom, and
    // whether its layer's inside is above it there: what a row none of a
    // layer's lines cross takes from the middle of the row below it can be
    // the end of an island, the row above it past its tip - the island's
    // coast crossing that row twice, either side of the middle, and the row
    // above not at all - and the sea beyond painted land (the Isle of Lewis,
    // from 220km).
    struct Upward {
        float y;
        uint8_t layer;
        bool on;
    };
    static Vector<Upward> middle;
    middle.clear();
    const float middleX = (float)(g.w / 2);
    static const Rgb KINDS[] = {{222, 216, 196}, {196, 206, 214}, {150, 150, 158}, {128, 130, 138}};
    static const Rgb LAND_LINE = {86, 98, 70};  // the land's own colour, for a line seen along it
    // Cells none of whose lines can be on the screen passed over: all of one
    // behind the eye, off a side of the view, or past the horizon. (Those off
    // to the sides were once read for where a coast crossed a row out of
    // sight, which said what the row began as; now what is in the middle of
    // each row comes up the middle of the screen, and a row is worked out
    // from there by the crossings on the screen - and a coast hundreds of
    // kilometres off to the side, thinned or crossed over, can't paint the
    // view.)
    // And none further off than LINES_REACH: past it the haze has taken
    // four fifths of the colour, the land and sea and towns are near enough
    // one, and from FL350 it was over half of the ground in view.
    float reach = r > 0 ? std::min(sqrtf(g_groundReach2), LINES_REACH) : 0, reach2 = reach * reach;
    auto onScreen = [&](const Cell &c) {
        if (ballInside(toCamera(c.middle)) < -c.radius) {
            return false;
        }
        float dx = c.middle.x - g.eye.x, dy = c.middle.y - g.eye.y;
        return reach <= 0 || sqrtf(dx * dx + dy * dy) - c.radius <= reach;
    };
    auto beyond = [&](Vec3 p) {
        float dx = p.x - g.eye.x, dy = p.y - g.eye.y;
        return reach > 0 && dx * dx + dy * dy > reach2;
    };
    // A line's start is mostly the end of the one before it, as they come:
    // put in the camera's space once.
    Vec3 lastEnd{1e30f, 0, 0}, lastEndSeen{};
    inCells(m.lineCells, m.groundLines, onScreen, [&](const GroundLine &l) {
        if (beyond(l.a) && beyond(l.b)) {
            return;
        }
        Vec3 a = l.a.x == lastEnd.x && l.a.y == lastEnd.y && l.a.z == lastEnd.z ? lastEndSeen : toCamera(l.a),
             b = toCamera(l.b);
        lastEnd = l.b;
        lastEndSeen = b;
        if ((a.z < NEAR && b.z < NEAR) || (inside(1, a) < 0 && inside(1, b) < 0) ||
            (inside(2, a) < 0 && inside(2, b) < 0)) {
            return;
        }
        uint8_t layer = layerOf(l.kind);
        // A town's edge isn't drawn: it is where the town is drawn to. Nor,
        // over the ground pack's cover, a shore: the cover's cells draw it -
        // land meeting water - and this line, from other data, ran a
        // kilometre out over the sea from it at Lossiemouth, a pale thread
        // across the water.
        if (layer != TOWNS && !(g.coverOn && layer == SHORES)) {
            // Pale where it is seen looking well down on it, as surf and sand
            // are; seen low along the ground - a coast twenty miles off from
            // 1,500ft - the strip of beach a hundredth of a pixel, and the
            // line the land's colour, not a white thread along the horizon.
            Vec3 mid = lerp(l.a, l.b, 0.5f) - g.eye;
            float far = sqrtf(dot(mid, mid)), steep = clamp01((eyeUp / far - 0.02f) * (1 / 0.06f));
            addSeenLine(a, b, mix(LAND_LINE, KINDS[(int)l.kind & 3], steep), far);
        }
        if (!fill || layer == NO_LAYER) {
            return;
        }
        if (a.z < NEAR) {
            a = lerp(a, b, (NEAR - a.z) / (b.z - a.z));
        } else if (b.z < NEAR) {
            b = lerp(b, a, (NEAR - b.z) / (a.z - b.z));
        }
        Projected pa = project(a), pb = project(b);
        // Crossing the middle going right, it has its inside above it.
        if ((pa.x < middleX) != (pb.x < middleX)) {
            float y = pa.y + (middleX - pa.x) * (pb.y - pa.y) / (pb.x - pa.x);
            if (y > -1 && y < h + 1) {
                middle.push_back({y, layer, (pb.x > pa.x) == !l.landRight});
            }
        }
        float dy = pb.y - pa.y;
        if (fabsf(dy) < 1e-4f) {
            return;
        }
        int y0 = std::max(0, ceilInt(std::min(pa.y, pb.y) - 0.5f));
        int y1 = std::min(h, ceilInt(std::max(pa.y, pb.y) - 0.5f));
        if (y0 >= y1) {
            return;
        }
        float slope = (pb.x - pa.x) / dy;
        crossers.push_back({pa.x + (y0 + 0.5f - pa.y) * slope, slope, (int16_t)y0, (int16_t)y1, layer,
                            (dy > 0) == !l.landRight});
    });
    if (!fill) {
        return;
    }
    g.landSea = true;
    // Bucketed by row - counted, then placed - rather than sorted all
    // together; and each row's few put in order along it. Only those on the
    // screen: off it, past a side, the lines wholly out there weren't read,
    // and a row's crossings there are only some of them - one left behind
    // said sea, and the next, passed over, would have said land (Finland's
    // lakes). Before the first on the screen is what it leaves.
    g.rowStart.assign(h + 1, 0);
    const float w = (float)g.w;
    for (const Crosser &c : crossers) {
        float x = c.x;
        for (int y = c.top; y < c.bottom; y++, x += c.slope) {
            g.rowStart[y + 1] += g.rowGround[y] && x >= 0 && x < w;
        }
    }
    for (int y = 0; y < h; y++) {
        g.rowStart[y + 1] += g.rowStart[y];
    }
    static Vector<float> xs;
    xs.resize(g.rowStart[h]);
    g.rowCross.resize(g.rowStart[h]);
    static Vector<uint32_t> at;
    at.assign(g.rowStart.begin(), g.rowStart.end() - 1);
    for (const Crosser &c : crossers) {
        float x = c.x;
        for (int y = c.top; y < c.bottom; y++, x += c.slope) {
            if (g.rowGround[y] && x >= 0 && x < w) {
                uint32_t i = at[y]++;
                xs[i] = x;
                g.rowCross[i] = {(int16_t)x, c.layer, c.onto};
            }
        }
    }
    uint32_t kept = 0;
    for (int y = 0; y < h; y++) {
        uint32_t from = g.rowStart[y], to = g.rowStart[y + 1];
        // Insertion sort by where along the row, on the true place rather
        // than the whole pixel, so two in the one pixel keep their order.
        for (uint32_t i = from + 1; i < to; i++) {
            float x = xs[i];
            Frame::Crossing c = g.rowCross[i];
            uint32_t j = i;
            for (; j > from && xs[j - 1] > x; j--) {
                xs[j] = xs[j - 1];
                g.rowCross[j] = g.rowCross[j - 1];
            }
            xs[j] = x;
            g.rowCross[j] = c;
        }
        // Two of a layer a hair apart, one onto it and one off, neither
        // kept: the tip of an island the row just touches, or all but - put
        // in either order by the rounding, and the wrong one said the rest of
        // the row was land (an island off the Virgin Islands, end on). What
        // is either side of the two is the same.
        g.rowStart[y] = kept;
        for (uint32_t i = from; i < to; i++) {
            if (i + 1 < to && xs[i + 1] - xs[i] < TOUCHING && g.rowCross[i].layer == g.rowCross[i + 1].layer &&
                g.rowCross[i].on != g.rowCross[i + 1].on) {
                i++;
                continue;
            }
            xs[kept] = xs[i];
            g.rowCross[kept++] = g.rowCross[i];
        }
    }
    g.rowStart[h] = kept;
    g.counts.crossings = (int)kept;
    g.rowFirst.assign(h, 0);
    int lowest = h - 1;
    while (lowest >= 0 && !g.rowGround[lowest]) {
        lowest--;
    }
    if (lowest < 0) {
        return;
    }
    // What is under the middle of the lowest row of ground, in each layer:
    // where that row's middle meets the round earth, as the rows have it -
    // not where it would meet a flat one: looking out near the horizon, that
    // was hundreds of kilometres further on, past the horizon, often over
    // some other coast, and the whole of the ground was painted the other
    // way (the Bay of Biscay, from below an aircraft). At an airfield, the
    // ground flat.
    Vec3 dir = g.forward + g.down * ((lowest + 0.5f - g.cy) / g.f);
    float dl = length(dir), reachOut = 0;
    if (r > 0) {
        float eyeX = g.eye.x, eyeY = g.eye.y, eyeZ = g.eye.z;
        float above = eyeX * eyeX + eyeY * eyeY + eyeZ * (eyeZ + 2 * r);
        float b = (eyeX * dir.x + eyeY * dir.y + (eyeZ + r) * dir.z) / dl, disc = b * b - above;
        reachOut = disc >= 0 && b < 0 && above > 0 ? above / (-b + sqrtf(disc)) : 0;
    } else if (dir.z < 0 && g.eye.z > 0) {
        reachOut = g.eye.z / -dir.z * dl;
    }
    Vec3 p = g.eye + dir * (reachOut / dl);
    static Side shoreSide, townSide;
    bool carried[2] = {sideOf(shoreSide, SHORES, p, true), sideOf(townSide, TOWNS, p, false)};
    // Up the rows from the lowest, what is in the middle of each - each
    // crossing of the middle between one row's and the next's saying what is
    // above it.
    std::sort(middle.begin(), middle.end(), [](const Upward &a, const Upward &b) { return a.y > b.y; });
    // The same for two of a layer a hair apart up the middle, between the
    // same two rows: a row between them is inside what they bound, and the
    // second says what is above it.
    size_t keptUp = 0;
    for (size_t i = 0; i < middle.size(); i++) {
        if (i + 1 < middle.size() && middle[i].y - middle[i + 1].y < TOUCHING && middle[i].layer == middle[i + 1].layer &&
            middle[i].on != middle[i + 1].on && floorf(middle[i].y - 0.5f) == floorf(middle[i + 1].y - 0.5f)) {
            i++;
            continue;
        }
        middle[keptUp++] = middle[i];
    }
    middle.resize(keptUp);
    size_t up = 0;
    while (up < middle.size() && middle[up].y > lowest + 0.5f) {
        up++;
    }
    for (int y = lowest; y >= 0; y--) {
        if (!g.rowGround[y]) {
            continue;
        }
        for (; up < middle.size() && middle[up].y > y + 0.5f; up++) {
            carried[middle[up].layer] = middle[up].on;
        }
        // Each layer: before the row's first crossing of it, what that
        // leaves - or, none crossing it, what came up the middle - and in
        // the row's middle, for the rows above.
        uint8_t first = 0;
        for (uint8_t layer = SHORES; layer <= TOWNS; layer++) {
            bool crossed = false, state = carried[layer], inMiddle = carried[layer];
            for (uint32_t i = g.rowStart[y]; i < g.rowStart[y + 1]; i++) {
                const Frame::Crossing &c = g.rowCross[i];
                if (c.layer != layer) {
                    continue;
                }
                if (!crossed) {
                    crossed = true;
                    state = inMiddle = !c.on;
                }
                if (c.x < g.w / 2) {
                    inMiddle = c.on;
                }
            }
            first |= state << layer;
            carried[layer] = inMiddle;
        }
        g.rowFirst[y] = first;
    }
}
}  // namespace

namespace {
// What stands about the airfield, into g.solid.
void addSolids() {
    const Models &m = models();
    const Palette &pal = g.pal;
    g.solid.clear();
    // Each cell's haze once - its buildings much of a distance - and each
    // triangle's facing from the normal worked out with the airfield; a
    // cell wholly in view, its triangles unclipped.
    auto solids = [&](const Vector<Cell> &cells, const Vector<SolidTri> &items, float within) {
        for (const Cell &c : cells) {
            float in = ballInside(toCamera(c.middle)), away = length(c.middle - g.eye);
            if (in < -c.radius || away - c.radius > within) {
                continue;
            }
            bool whole = in > c.radius;
            float haze = 1 - expf(-away / pal.visibility);
            GridBase base = gridBase(c.middle, m.unit);
            for (uint32_t i = c.begin; i < c.end; i++) {
                const SolidTri &t = items[i];
                // Facing away - from where its first corner is from the eye -
                // nothing more worked out.
                Vec3 n = {(float)t.n[0], (float)t.n[1], (float)t.n[2]};
                Vec3 first = base.fromEye + Vec3{(float)(t.v[0][0] - base.qx), (float)(t.v[0][1] - base.qy),
                                                 (float)(t.v[0][2] - base.qz)} * m.unit;
                if (dot(n, first) >= 0) {
                    continue;
                }
                Vec3 c0 = seenOnGrid(base, t.v[0][0], t.v[0][1], t.v[0][2]),
                     c1 = seenOnGrid(base, t.v[1][0], t.v[1][1], t.v[1][2]),
                     c2 = seenOnGrid(base, t.v[2][0], t.v[2][1], t.v[2][2]);
                addInCell(c0, c1, c2, whole, g.solid, [&](ScreenTri &s) {
                    float light = pal.ambient + pal.diffuse * std::max(0.0f, dot(n, g.sun) * (1.0f / 32767));
                    Rgb color = {(float)t.rgb[0], (float)t.rgb[1], (float)t.rgb[2]};
                    s.color = pack(mix(scale(color, std::max(light, pal.floodlit)), pal.haze, haze));
                });
            }
        }
    };
    solids(m.solidCells, m.solidTris, 1e30f);
    // The small buildings only within 1.5km: past that they are a few pixels
    // each, and an RAF station has a hundred of them.
    solids(m.detailCells, m.detailTris, 1500);
}
}  // namespace

void setSolidsReady(void (*ready)()) { g_solidsReady = ready; }

SolidsBench benchSolids() {
    const Models &m = models();
    const Palette &pal = g.pal;
    SolidsBench out{};
    volatile float sink = 0;
    for (int upTo = 0; upTo < 5; upTo++) {
        uint32_t t0 = now();
        uint32_t tested = 0, facing = 0;
        for (int rep = 0; rep < 20; rep++) {
            if (upTo == 4) {
                addSolids();
                continue;
            }
            for (int which = 0; which < 2; which++) {
                const Vector<Cell> &cells = which == 0 ? m.solidCells : m.detailCells;
                const Vector<SolidTri> &items = which == 0 ? m.solidTris : m.detailTris;
                float within = which == 0 ? 1e30f : 1500;
                for (const Cell &c : cells) {
                    float in = ballInside(toCamera(c.middle)), away = length(c.middle - g.eye);
                    if (in < -c.radius || away - c.radius > within) {
                        continue;
                    }
                    float haze = 1 - expf(-away / pal.visibility);
                    GridBase base = gridBase(c.middle, m.unit);
                    for (uint32_t i = c.begin; i < c.end; i++) {
                        const SolidTri &t = items[i];
                        tested++;
                        Vec3 n = {(float)t.n[0], (float)t.n[1], (float)t.n[2]};
                        Vec3 first = base.fromEye + Vec3{(float)(t.v[0][0] - base.qx), (float)(t.v[0][1] - base.qy),
                                                         (float)(t.v[0][2] - base.qz)} * m.unit;
                        if (dot(n, first) >= 0) {
                            continue;
                        }
                        facing++;
                        if (upTo == 0) {
                            continue;
                        }
                        float light = pal.ambient + pal.diffuse * std::max(0.0f, dot(n, g.sun) * (1.0f / 32767));
                        Rgb color = {(float)t.rgb[0], (float)t.rgb[1], (float)t.rgb[2]};
                        uint16_t packed = pack(mix(scale(color, std::max(light, pal.floodlit)), pal.haze, haze));
                        if (upTo == 1) {
                            sink = sink + packed;
                            continue;
                        }
                        Vec3 c0 = seenOnGrid(base, t.v[0][0], t.v[0][1], t.v[0][2]),
                             c1 = seenOnGrid(base, t.v[1][0], t.v[1][1], t.v[1][2]),
                             c2 = seenOnGrid(base, t.v[2][0], t.v[2][1], t.v[2][2]);
                        Projected p0 = project(c0), p1 = project(c1), p2 = project(c2);
                        if (upTo == 2) {
                            sink = sink + p0.x + p1.y + p2.iz;
                            continue;
                        }
                        ScreenTri st;
                        if (setup(st, p0, p1, p2)) {
                            sink = sink + st.top;
                        }
                    }
                }
            }
        }
        out.us[upTo] = (now() - t0) / 20;
        if (upTo == 0) {
            out.tested = tested / 20;
            out.facing = facing / 20;
        }
    }
    // The ground's cells in view, their triangles read and no more.
    uint32_t g0 = now(), tris = 0;
    for (int rep = 0; rep < 20; rep++) {
        for (const Cell &c : m.groundCells) {
            if (ballInside(toCamera(c.middle)) < -c.radius) {
                continue;
            }
            for (uint32_t i = c.begin; i < c.end; i++) {
                sink = sink + m.groundTris[i].x[0] + m.groundTris[i].fine;
                tris++;
            }
        }
    }
    out.groundReadUs = (now() - g0) / 20;
    out.groundTris = tris / 20;
    out.groundUs = g.counts.stepUs[STEP_GROUND];
#ifdef BENCH3D
    memset(g_seen, 0, sizeof g_seen);
    addSolids();
    memcpy(out.seen, g_seen, sizeof out.seen);
#endif
    return out;
}

bool prepareSolids() {
    Solids ready = Solids::READY;
    if (!g_solids.compare_exchange_strong(ready, Solids::DOING, std::memory_order_acquire)) {
        return false;
    }
    uint32_t t0 = now();
    addSolids();
    g.counts.stepUs[STEP_SOLIDS] = now() - t0;
    g_solids.store(Solids::DONE, std::memory_order_release);
    return true;
}

void prepareEnd() {
    uint32_t t0 = now(), mark = t0;
    auto lap = [&](Step step) {
        uint32_t n = now();
        g.counts.stepUs[step] += n - mark;
        mark = n;
    };
    const Models &m = models();
    const Palette &pal = g.pal;
    const Scene &scene = g.scene;

    // What stands about the airfield, after the aircraft - they are nearer -
    // unless the other core has them in hand.
    bool solidsHere = prepareSolids();
    mark = now();

    inCells(m.wireCells, m.wires, inView, [](const ModelLine &l) { addLine(l); });
    lap(STEP_WIRES);
    // The country below: its coasts pale, as surf and sand show them from
    // high up; lakes' shores the same; borders and state lines fainter - and
    // none of it the earth's curve hides.
    groundAndCoasts();
    lap(STEP_COUNTRY);
    // From each aircraft high enough, a line down to the ground under it: its
    // height, seen.
    if (scene.stemsAbove >= 0) {
        for (const Placed &p : g.placed) {
            if (p.followed || heightOf(p.pose.at) <= scene.stemsAbove) {
                continue;
            }
            Vec3 top = Placer(p.pose, p.look.scale).place(designOf(p.look).middle);
            addLine({top, groundUnder(top), {176, 184, 196}});
        }
    }
    lap(STEP_STEMS);

    // The airfield's lights after dark, the towns' about it, and the stars;
    // the PAPI all day.
    g.sprites.clear();
    g.dots.clear();
    if (pal.lights > 0.5f) {
        // Each cell's lights hazed alike, as far off as its middle: across a
        // cell the haze hardly changes, and a root and an exponential for
        // each of thousands of lights was most of the cost of them.
        for (const Cell &c : m.lightCells) {
            if (!inView(c)) {
                continue;
            }
            float distance = length(c.middle - g.eye), haze = lightHaze(distance);
            // A taxiway's lights a fraction of a runway's: dimmer the further
            // off, and none at all past TAXI_LIGHTS_GONE - a big airport's
            // thousands, seen from miles out, only a speckle.
            if (c.lod == LIGHTS_TAXIWAY) {
                if (distance > TAXI_LIGHTS_GONE) {
                    continue;
                }
                haze = 1 - (1 - haze) * std::min(1.0f, TAXI_LIGHTS_FULL / distance) *
                               std::min(1.0f, (TAXI_LIGHTS_GONE - distance) / TAXI_LIGHTS_FADE);
            }
            GridBase base = gridBase(c.middle, m.unit);
            // Its few colours hazed and packed once for the cell, as each is
            // first wanted.
            constexpr int CACHED = 16;
            uint16_t packed[CACHED];
            uint32_t done = 0;
            auto packedOf = [&](int k) {
                if (k >= CACHED) {
                    return pack(mix(m.lightColors[k], g.pal.haze, haze));
                }
                if (!(done >> k & 1)) {
                    packed[k] = pack(mix(m.lightColors[k], g.pal.haze, haze));
                    done |= 1u << k;
                }
                return packed[k];
            };
            for (uint32_t i = c.begin; i < c.end; i++) {
                const LightPoint &l = m.lightPoints[i];
                if (l.end >= 0 && l.end != g_landingEnd) {
                    continue;  // an approach not in use
                }
                // One showing a different colour behind it - a threshold's,
                // green to the approach and red to the runway - in the
                // colour it shows the eye.
                bool behind = false;
                if (l.facing[0] != 0 || l.facing[1] != 0) {
                    Vec3 fromEye = base.fromEye + Vec3{(float)(l.x - base.qx), (float)(l.y - base.qy),
                                                       (float)(l.z - base.qz)} * m.unit;
                    behind = dot(fromEye, Vec3{(float)l.facing[0], (float)l.facing[1], (float)l.facing[2]}) > 0;
                }
                int k = behind ? l.behind : l.color;
                addFieldLight(seenOnGrid(base, l.x, l.y, l.z), m.lightColors[k], [&] { return packedOf(k); },
                              m.lightSizes[l.size], 3, haze);
            }
        }
        for (const Cell &c : m.townCells) {
            if (!inView(c) || (scene.earthRadius > 0 && overHorizon(c.middle) &&
                               length(c.middle - g.eye) - c.radius > sqrtf(2 * scene.earthRadius * heightOf(g.eye)))) {
                continue;
            }
            float haze = lightHaze(length(c.middle - g.eye));
            for (uint32_t i = c.begin; i < c.end; i++) {
                const ModelLight &l = m.townLights[i];
                addFieldLight(toCamera(l.at), l.color, [&] { return pack(mix(l.color, g.pal.haze, haze)); }, l.size, 0,
                              haze);
            }
        }
        for (const ModelLight &l : m.farLights) {
            if (!overHorizon(l.at)) {
                addLight(l.at, l.color, l.size, 2.5f, g.sprites);
            }
        }
    }
    if (pal.stars > 0.02f) {
        for (const Star &s : m.stars) {
            ScreenSprite sp{};
            if (addSprite(g.eye + s.direction * 1e6f, 1, 0.5f, sp)) {
                sp.blend = Blend::SOLID;
                sp.iz = 0;
                sp.color = pack(mix(pal.skyTop, {220, 226, 255}, s.brightness * pal.stars));
                g.sprites.push_back(sp);
            }
        }
    }
    // The sun itself, while it is up: a disc half a degree across and a glow
    // round it, deep orange low down, white higher, behind everything.
    if (g.sunElevationDeg > 0.5f) {
        ScreenSprite sp{};
        if (addSprite(g.eye + g.sunDir * 1e6f, 1, 0.5f, sp)) {
            Rgb color = mix({255, 176, 96}, {255, 248, 232}, clamp01(g.sunElevationDeg / 20));
            sp.r = std::max(3.0f, g.f * tanf(0.27f * DEG));
            sp.iz = 0;
            sp.blend = Blend::SOLID;
            sp.color = pack(color);
            sp.top = std::max(0, floorInt(sp.y - sp.r * 8));
            sp.bottom = std::min(g.h, ceilInt(sp.y + sp.r * 8) + 1);
            if (sp.top < sp.bottom) {
                g.sprites.push_back(sp);
                sp.r *= 8;
                sp.blend = Blend::GLOW;
                sp.add[0] = color.r * 0.5f * 31 / 255;
                sp.add[1] = color.g * 0.5f * 63 / 255;
                sp.add[2] = color.b * 0.5f * 31 / 255;
                g.sprites.push_back(sp);
            }
        }
    }
    for (const Papi &papi : m.papis) {
        Vec3 towards = g.eye - papi.at;
        if (dot(towards, papi.facing) <= 0) {
            continue;  // seen from behind, it shows nothing
        }
        float angle = atan2f(towards.z, sqrtf(towards.x * towards.x + towards.y * towards.y));
        addLight(papi.at, angle > papi.angle ? LAMP_WHITE : LAMP_RED, 0.8f, pal.lights > 0.5f ? 5 : 3, g.sprites);
    }

    // The rabbit: after dark, the approach in use's sequenced flashers, a
    // white flash running up the centreline from 900m out to the crossbar at
    // 300, twice a second.
    if (pal.lights > 0.5f && !m.ends.empty()) {
        const RunwayEnd &end = m.ends[g_landingEnd];
        int station = (int)(fmodf(scene.seconds * 2, 1.0f) * 20);
        float along = -900 + 30.0f * station, height = 0.6f - along * 0.004f;
        addLight(end.at(along, 0) + Vec3{0, 0, height + 0.4f}, {255, 255, 255}, 0.9f, 6, g.sprites);
    }

    // Tyre smoke where the wheels touched: two puffs a side, swelling and
    // thinning as they drift up and back.
    if (scene.smokeAge >= 0 && scene.smokeAge < 3) {
        Placer at(scene.touchdown, scene.followed.look.scale);
        for (float side : {1.0f, -1.0f}) {
            for (int puff = 0; puff < 2; puff++) {
                float age = scene.smokeAge - puff * 0.12f;
                if (age < 0) {
                    continue;
                }
                Vec3 where = at.place({-2.0f - age * 2.5f - puff * 2,
                                       side * (designOf(scene.followed.look).gearY + puff * 0.5f), 0.6f + age * 0.9f});
                ScreenSprite s{};
                if (!addSprite(where, 2 + age * 5, 1, s)) {
                    continue;
                }
                s.blend = Blend::STIPPLE;
                s.coverage = age < 0.5f ? 3 : age < 1.4f ? 2 : age < 2.6f ? 1 : 0;
                s.color = pack(hazed(scale({214, 214, 208}, pal.ambient + pal.diffuse * 0.7f), length(where - g.eye)));
                if (s.coverage > 0) {
                    g.sprites.push_back(s);
                }
            }
        }
    }

    lap(STEP_LIGHTS);

    // The buildings, if the other core took them: waited for.
    if (!solidsHere) {
        while (g_solids.load(std::memory_order_acquire) != Solids::DONE) {
        }
        lap(STEP_SOLIDS_WAIT);
    }

    // Each band's share of it all.
    const List<ScreenTri> *ground = &g.ground;
    g.groundBins.build(&ground, 1, g.bandRows, g.bands);
    for (int p = 0; p < PARTS; p++) {
        g.solidLists[p] = &g.parts[p].solid;
        g.shadowLists[p] = &g.parts[p].shadow;
        g.clearLists[p] = &g.parts[p].clear;
        g.spriteLists[p + 1] = &g.parts[p].sprites;
    }
    g.solidLists[PARTS] = &g.solid;
    g.spriteLists[0] = &g.sprites;
    g.solidBins.build(g.solidLists, PARTS + 1, g.bandRows, g.bands);
    g.shadowBins.build(g.shadowLists, PARTS, g.bandRows, g.bands);
    g.clearBins.build(g.clearLists, PARTS, g.bandRows, g.bands);
    g.spriteBins.build(g.spriteLists, PARTS + 1, g.bandRows, g.bands);
    const List<ScreenDot> *dots = &g.dots;
    g.dotBins.build(&dots, 1, g.bandRows, g.bands);
    const List<ScreenLine> *wires = &g.wires;
    g.wireBins.build(&wires, 1, g.bandRows, g.bands);
    lap(STEP_BINS);

    // The depth scale: the nearest any aircraft's corner, anything else
    // standing, a line's end or a light comes, to DEPTH_TOP. (1/z: the
    // largest is the nearest.)
    float nearest = 1e-6f;
    for (const Part &p : g.parts) {
        nearest = std::max(nearest, 1 / std::max(NEAR, p.nearest));
    }
    for (const ScreenTri &t : g.solid.items) {
        nearest = std::max(nearest, t.za * t.ax + t.zb * t.ay + t.zc);  // its top corner: near enough
    }
    for (const ScreenLine &l : g.wires.items) {
        nearest = std::max({nearest, l.iz0, l.iz1});
    }
    for (const List<ScreenSprite> *list : g.spriteLists) {
        for (const ScreenSprite &sp : list->items) {
            nearest = std::max(nearest, sp.iz);
        }
    }
    for (const ScreenDot &dot : g.dots.items) {
        nearest = std::max(nearest, dot.iz);
    }
    g.depthK = DEPTH_TOP / nearest;
    // The hills' depths, at that scale - the depth buffer's own sums.
    for (Frame::HillRange &range : g.hills) {
        if (range.on) {
            float k = g.depthK * 1.002f;
            for (int x = 0; x < g.w; x++) {
                range.depth[x] = range.from[x] < range.to[x] ? (uint16_t)std::min(65535.0f, range.iz[x] * k) : 0;
            }
        }
    }
    lap(STEP_DEPTH);

    Counts &c = g.counts;
    c.triangles = (int)(g.ground.size() + g.solid.size());
    c.lights = (int)(g.sprites.size() + g.dots.size());
    for (int p = 0; p < PARTS; p++) {
        const Part &part = g.parts[p];
        c.triangles += (int)(part.solid.size() + part.shadow.size() + part.clear.size());
        c.lights += (int)part.sprites.size();
        c.aircraftFull += part.counts.aircraftFull;
        c.aircraftSimple += part.counts.aircraftSimple;
        c.aircraftTiny += part.counts.aircraftTiny;
        c.aircraftDots += part.counts.aircraftDots;
        c.aircraftUs[p] = part.counts.aircraftUs[0];
        for (int step = STEP_FOLLOWED; step <= STEP_SHADOWS; step++) {
            c.stepUs[step] += part.counts.stepUs[step];
        }
    }
    c.lines = (int)g.wires.size();
    c.endUs = now() - t0;
}

void prepare(int w, int h, int bandRows, bool swapBytes, const Scene &scene, const Camera &camera) {
    prepareBegin(w, h, bandRows, swapBytes, scene, camera);
    prepareAircraft(0, 1);
    prepareEnd();
    flip();
}

void flip() {
    std::swap(g, d);
    // The lists' places, taken from where they now are.
    for (int p = 0; p < PARTS; p++) {
        d.solidLists[p] = &d.parts[p].solid;
        d.shadowLists[p] = &d.parts[p].shadow;
        d.clearLists[p] = &d.parts[p].clear;
        d.spriteLists[p + 1] = &d.parts[p].sprites;
    }
    d.solidLists[PARTS] = &d.solid;
    d.spriteLists[0] = &d.sprites;
}

Counts counts() { return d.counts; }


void terrainCache(uint64_t out[8], bool reset) {
    for (int i = 0; i < 8; i++) {
        out[i] = g_terrainCache[0][i] + g_terrainCache[1][i];
        if (reset) {
            g_terrainCache[0][i] = g_terrainCache[1][i] = 0;
        }
    }
}

void terrainProbe(int which) { g_terrainProbe = which; }

void terrainPaths(uint32_t out[7]) { memcpy(out, g_marchPaths.paths, sizeof g_marchPaths.paths); }

uint64_t terrainBandInstructions() {
    uint64_t n = g_bandInstret[0] + g_bandInstret[1];
    g_bandInstret[0] = g_bandInstret[1] = 0;
    return n;
}

uint64_t terrainBandHanded() {
    uint64_t n = g_bandHanded[0] + g_bandHanded[1];
    g_bandHanded[0] = g_bandHanded[1] = 0;
    return n;
}

void terrainMarchCheck(uint32_t out[4]) {
#if defined(ESP_PLATFORM) && defined(BENCH3D)
    // The frame being prepared's march in assembly, then in C alone: their
    // columns' tops and rows' depths that differ, and the cycles each took.
    uint64_t st[8];
    terrainStats(st, true);
    std::vector<uint16_t> tz;
    std::vector<int16_t> tops;
    uint32_t c0 = cycles();
    terrainMarch();
    out[2] = cycles() - c0;
    tz.assign(g.tz.begin(), g.tz.end());
    tops.assign(g.tTop.begin(), g.tTop.end());
    g_terrainProbe = 8;
    c0 = cycles();
    terrainMarch();
    out[3] = cycles() - c0;
    g_terrainProbe = 0;
    uint32_t topsDiffer = 0, rowsDiffer = 0;
    for (size_t c = 0; c < tops.size(); c++) {
        topsDiffer += tops[c] != g.tTop[c];
    }
    // Only the rows filled count: the rest is left from frames before.
    int bandRows = g.bandRows, cols = g.tCols;
    for (int c = 0; c < cols; c++) {
        int from = std::max((int)tops[c], (int)g.tTop[c]), to = g.tFlatTop[c];
        for (int r = from; r < to; r++) {
            int b = r / bandRows;
            size_t at = ((size_t)b * cols + c) * bandRows + (r - b * bandRows);
            rowsDiffer += tz[at] != g.tz[at];
        }
    }
    out[0] = topsDiffer;
    out[1] = rowsDiffer;
    terrainStats(st, true);
#else
    memset(out, 0, 4 * sizeof(uint32_t));
#endif
}

void terrainEvents(uint32_t out[6]) {
#if defined(ESP_PLATFORM) && defined(BENCH3D)
    // The march as it is, then with its heights read from four rows of the
    // grid, all in the cache: cycles, instructions and samples of each.
    uint64_t st[8];
    terrainStats(st, true);
    for (int probe = 0; probe < 2; probe++) {
        g_terrainProbe = probe;
        uint32_t c0, c1, i0, i1;
        __asm__ volatile("csrr %0, mcycle" : "=r"(c0));
        __asm__ volatile("csrr %0, minstret" : "=r"(i0));
        terrainMarch();
        __asm__ volatile("csrr %0, minstret" : "=r"(i1));
        __asm__ volatile("csrr %0, mcycle" : "=r"(c1));
        terrainStats(st, true);
        out[3 * probe] = c1 - c0;
        out[3 * probe + 1] = i1 - i0;
        out[3 * probe + 2] = (uint32_t)st[1];
    }
    g_terrainProbe = 0;
#else
    memset(out, 0, 6 * sizeof(uint32_t));
#endif
}

void terrainStats(uint64_t out[8], bool reset) {
    for (int i = 0; i < 8; i++) {
        out[i] = g_terrainStats[0][i] + g_terrainStats[1][i];
        if (reset) {
            g_terrainStats[0][i] = g_terrainStats[1][i] = 0;
        }
    }
}

#ifdef BENCH3D
void spanStats(uint32_t out[6], bool reset) {
    for (int i = 0; i < 6; i++) {
        out[i] = g_spanStats[0][i] + g_spanStats[1][i];
        if (reset) {
            g_spanStats[0][i] = g_spanStats[1][i] = 0;
        }
    }
}
#endif

const char *stepName(int step) {
    static const char *const NAMES[STEPS] = {"rows",    "far ground", "ground",  "paint",   "placing", "sorting",
                                             "followed", "traffic",   "its shadow", "solids", "wires",  "country",
                                             "stems",   "lights",     "waiting for the solids", "bins", "depth",
                                             "rivers"};
    return step >= 0 && step < STEPS ? NAMES[step] : "";
}

int bandCount() { return d.bands; }

size_t scratchBytes(int w, int bandRows) { return colorBytes(w, bandRows) + (size_t)w * bandRows * 2; }  // bands of 64 rows at most

uint16_t *drawBand(int band, void *scratch, uint32_t *stageUs) {
    uint16_t *color = (uint16_t *)scratch;
    drawBandInto(band, color, (uint16_t *)((uint8_t *)scratch + colorBytes(d.w, d.bandRows)), stageUs);
    return color;
}

// The cover's innermost loops, in C - and on the device, in assembly: see
// scene3d_cover.S, which the bench sets against these.
//
// `n` of a tile's cells, `stride` to its row, along a line from (cu, cv) in
// 16.16 fixed point, stepping (dcu, dcv): into `out`, each marked in `seen`.
void coverCellsC(const uint8_t *cells, int stride, int32_t cu, int32_t cv, int32_t dcu, int32_t dcv, int n,
                 uint8_t *out, uint8_t *seen) {
    for (int i = 0; i < n; i++) {
        uint8_t c = cells[(cv >> 16) * stride + (cu >> 16)];
        out[i] = c;
        seen[c] = 1;
        cu += dcu;
        cv += dcv;
    }
}

// `n` samples' four pixels each, from a pixel whose x is a multiple of 4:
// each the colour pair of its cell.
void coverWriteC(uint16_t *row, const uint8_t *cells, const uint32_t *pairs, int n) {
    uint32_t *out = (uint32_t *)row;
    for (int i = 0; i < n; i++) {
        uint32_t p = pairs[cells[i]];
        out[2 * i] = p;
        out[2 * i + 1] = p;
    }
}

// The same for samples of eight pixels.
void coverWrite8C(uint16_t *row, const uint8_t *cells, const uint32_t *pairs, int n) {
    uint32_t *out = (uint32_t *)row;
    for (int i = 0; i < n; i++) {
        uint32_t p = pairs[cells[i]];
        out[4 * i] = p;
        out[4 * i + 1] = p;
        out[4 * i + 2] = p;
        out[4 * i + 3] = p;
    }
}

#if defined(ESP_PLATFORM)
extern "C" void scene3d_cover_cells(const uint8_t *cells, int stride, int32_t cu, int32_t cv, int32_t dcu,
                                    int32_t dcv, int n, uint8_t *out, uint8_t *seen);
extern "C" void scene3d_cover_write(uint16_t *row, const uint8_t *cells, const uint32_t *pairs, int n);
extern "C" void scene3d_cover_write8(uint16_t *row, const uint8_t *cells, const uint32_t *pairs, int n);
#endif

// A run of `m` samples kept in a tile `cells` wide, 16.16 fixed point: the
// first put inside it, and the step cut where the last would go out of it -
// by rounding, only ever just - and every one between with them.
inline void keepIn(int32_t &at, int32_t &step, int m, int cells) {
    int32_t top = cells * 65536 - 1;
    at = std::max<int32_t>(0, std::min(at, top));
    if (m > 1) {
        int32_t end = at + step * (m - 1);
        if (end > top) {
            step -= (end - top + m - 2) / (m - 1);
        } else if (end < 0) {
            step += (m - 2 - end) / (m - 1);
        }
    }
}

// The pack's cells along a stretch of a row of ground: `n` samples from
// (u, v), in tiles, stepping (du, dv) a sample, read at `level` - or, where
// a tile's cells at that level aren't in yet, the finest it has - into
// `out`, 0 where there is no tile, each marked in `seen`. A tile at a time:
// the tile worked out again only as one is crossed, and its cells stepped
// through. At level 0 and finer, a part of a tile at a time - see
// groundpack::Window - as many parts each way as the level asked for has,
// the walk's units that many to a tile.
void coverWalk(float u, float v, float du, float dv, int n, int level, uint8_t *out, uint8_t *seen) {
    const groundpack::Window &W = *d.cover.window;
    level = std::max(groundpack::FINEST, std::min(groundpack::LEVELS - 1, level));  // never a table past the ends
    int sBits = level <= 0 ? 1 - level : 0;  // the walk's units, 2^sBits to a tile each way
    if (sBits != 0) {
        float split = (float)(1 << sBits);
        u *= split;
        v *= split;
        du *= split;
        dv *= split;
    }
    float iu = du != 0 ? 1 / du : 0, iv = dv != 0 ? 1 / dv : 0;
    while (n > 0) {
        int tu = floorInt(u), tv = floorInt(v);
        int m = n;
        if (du > 0) {
            m = std::min(m, ceilInt((tu + 1 - u) * iu));
        } else if (du < 0) {
            m = std::min(m, ceilInt((tu - u) * iu));
        }
        if (dv > 0) {
            m = std::min(m, ceilInt((tv + 1 - v) * iv));
        } else if (dv < 0) {
            m = std::min(m, ceilInt((tv - v) * iv));
        }
        m = std::max(m, 1);
        // The cells: the finest the tile has at `level` or coarser, and the
        // corner (ou, ov) of their part, and how many units it spans, 2^k.
        const uint8_t *cells = nullptr;
        int stride = 0, rows = 0, ou = 0, ov = 0, k = 0;
        int su = tu >> sBits, sv = tv >> sBits;  // the tile
        if ((unsigned)su < (unsigned)W.cols && (unsigned)sv < (unsigned)W.rows) {
            int slot = sv * W.cols + su, c0 = groundpack::cols0At(W.north + sv);
            int at = level;
            for (;; at++) {
                if (at < 0) {
                    cells = W.finer[-1 - at][slot];
                } else if (at == 0) {
                    cells = W.detail[slot];
                } else if (at < groundpack::LEVELS - 1) {
                    if (const uint8_t *mid = W.mid[slot]) {
                        constexpr int R1 = groundpack::ROWS0 >> 1;
                        int c1 = c0 >> 1;
                        cells = mid + (at >= 2 ? R1 * c1 : 0) + (at >= 3 ? (R1 >> 1) * (c1 >> 1) : 0);
                    }
                } else {
                    cells = W.coarse[slot];
                }
                if (cells != nullptr || at >= groundpack::LEVELS - 1) {
                    break;
                }
            }
            if (cells != nullptr) {
                int pBits = at <= 0 ? 1 - at : 0;  // its parts, 2^pBits each way
                stride = at <= 0 ? c0 >> 1 : c0 >> at;
                rows = at <= 0 ? groundpack::ROWS0 >> 1 : groundpack::ROWS0 >> at;
                k = sBits - pBits;
                int pu = (tu >> k) & ((1 << pBits) - 1), pv = (tv >> k) & ((1 << pBits) - 1);
                cells += (size_t)((pv << pBits) + pu) * rows * stride;
                ou = (tu >> k) << k;
                ov = (tv >> k) << k;
            }
        }
        if (cells == nullptr || stride == 0) {
            memset(out, 0, m);
            seen[0] = 1;
        } else {
            float part = 1.0f / (1 << k), ku = part * stride * 65536.0f, kv = part * rows * 65536.0f;
            int32_t cu = (int32_t)((u - ou) * ku), cv = (int32_t)((v - ov) * kv);
            int32_t dcu = (int32_t)(du * ku), dcv = (int32_t)(dv * kv);
            keepIn(cu, dcu, m, stride);
            keepIn(cv, dcv, m, rows);
#if defined(BENCH3D) && defined(ESP_PLATFORM)
            uint64_t *stats = g_coverStats[xPortGetCoreID() & 1];
            uint32_t clock = esp_cpu_get_cycle_count();
#endif
#if defined(ESP_PLATFORM)
            if (g_simd) {
                scene3d_cover_cells(cells, stride, cu, cv, dcu, dcv, m, out, seen);
            } else
#endif
            {
                coverCellsC(cells, stride, cu, cv, dcu, dcv, m, out, seen);
            }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
            stats[5] += esp_cpu_get_cycle_count() - clock;
            stats[6] += m;
#endif
        }
        out += m;
        u += du * m;
        v += dv * m;
        n -= m;
    }
}

// What covers a row of ground: the cell at each four pixels - read at the
// middle of the four - and which cells there are among them, worked out for
// a row and kept for the row under it, which takes them as they are: a cell
// is some rows tall wherever it is fewer pixels across than four. Each core
// its own.
constexpr int COVER_SAMPLE = 4;
constexpr int COVER_SAMPLES_MOST = 1280 / COVER_SAMPLE;
struct CoverCells {
    int y = -100, share = 2;  // the first row it is for, and how many from there
    int shift = 2;            // a sample each 4 pixels, or - 3 - each 8
    int count = 0;  // in `list`
    alignas(8) uint8_t cell[COVER_SAMPLES_MOST];
    uint8_t list[256];
    alignas(4) uint8_t seen[256];
};
CoverCells g_coverCells[2];

// The colour pairs of the cells for a row's haze, to 32 levels, one for each
// parity of row, as the dithering has it - shared by the rows the haze is
// the same in, which on a core are most of a band's in turn: each worked out
// as its cell is first wanted. `checked` the row whose cells it has every
// one of.
struct CoverPairs {
    int haze = -1, checked = -100;
    uint32_t valid[8];
    alignas(16) uint32_t pair[256];
};
CoverPairs g_coverPairs[2][2];


// Row y's place on the cover's grid, and its level: false where it looks at
// no ground. Round, the even row's of its pair.
bool rowCoverOf(int y, Frame::RowCover &rc) {
    if (d.cover.round) {
        rc = d.rowCover[y >> 1];
        return rc.level != Frame::NO_LEVEL;
    }
    return coverFlatRow(d, y, rc);
}

// The cells along a row of ground, `rc`, into `out`.
void coverCells(const Frame::RowCover &rc, CoverCells &out) {
    int sample = 1 << out.shift, w = d.w, n = (w + sample - 1) >> out.shift;
    // Four stretches between the five points - or, straight, one from end
    // to end, which crosses fewer tiles' edges.
    int stretches = rc.straight ? 1 : 4, step = 4 / stretches;
    for (int seg = 0; seg < 4; seg += step) {
        int i0 = seg * n / 4, i1 = (seg + step) * n / 4;
        float span = w * 0.25f * step, du = (rc.u[seg + step] - rc.u[seg]) / span,
              dv = (rc.v[seg + step] - rc.v[seg]) / span;
        float at = i0 * sample + sample * 0.5f - seg * w * 0.25f;  // the first sample's middle, from the point
        coverWalk(rc.u[seg] + du * at, rc.v[seg] + dv * at, du * sample, dv * sample, i1 - i0, rc.level, out.cell + i0,
                  out.seen);
    }
    // The cells marked, gathered into the list, and the marks cleared - a
    // word at a time, most of them nought.
    out.count = 0;
    uint32_t *words = (uint32_t *)out.seen;
    for (int i = 0; i < 64; i++) {
        if (words[i] != 0) {
            for (int b = 0; b < 4; b++) {
                if (out.seen[4 * i + b] != 0) {
                    out.list[out.count++] = (uint8_t)(4 * i + b);
                }
            }
            words[i] = 0;
        }
    }
}

// Pixels [xa, xb) of row y, land, in what covers it - see GroundCover - each
// four the colour of its cell, lit, shaded and hazed as the row is; `land`
// where there is no tile, or the pack has nothing.
void coverSpan(uint16_t *row, int y, int xa, int xb, uint32_t land, const int haze[3]) {
#if defined(ESP_PLATFORM)
    int core = xPortGetCoreID() & 1;
#else
    int core = 0;
#endif
#if defined(BENCH3D) && defined(ESP_PLATFORM)
    uint64_t *stats = g_coverStats[core];
    uint32_t clock = esp_cpu_get_cycle_count();
    auto lap = [&](int i) {
        uint32_t c = esp_cpu_get_cycle_count();
        stats[i] += c - clock;
        clock = c;
    };
#else
    auto lap = [](int) {};
#endif
    CoverCells &cells = g_coverCells[core];
    if (!(y >= cells.y && y < cells.y + cells.share)) {
        // Four rows' alike, read along the third of them - near the middle -
        // where its cells are tall enough for them to be: eight rows or more,
        // the near land's, close to, a cell's edge then never more than two
        // rows out. Otherwise two rows', read along the first.
        // Or - flat, the row above it just over the horizon, looking at no
        // ground - its own, alone.
        int g4 = y & ~3, g2 = y & ~1;
        Frame::RowCover rc;
        if (g4 + 2 < d.h && rowCoverOf(g4 + 2, rc) && rc.tall) {
            cells.y = g4;
            cells.share = 4;
        } else if (rowCoverOf(g2, rc)) {
            cells.y = g2;
            cells.share = 2;
        } else if (rowCoverOf(y, rc)) {
            cells.y = y;
            cells.share = 1;
        } else {
            cells.y = -100;
            fill(row, xa, xb, land);
            return;
        }
        // A sample each eight pixels where the cells are sixteen or more wide.
        cells.shift = rc.wide ? 3 : 2;
        coverCells(rc, cells);
#if defined(BENCH3D) && defined(ESP_PLATFORM)
        stats[3]++;
#endif
    }
    lap(0);
    CoverPairs &pairs = g_coverPairs[core][y & 1];
    int h = d.rowHaze[y] >> 3;
    if (pairs.haze != h) {
        pairs.haze = h;
        pairs.checked = -100;
        memset(pairs.valid, 0, sizeof pairs.valid);
        pairs.valid[0] = 0xFFFF;  // nothing, set below
    }
    for (int i = 0; i < 16; i++) {
        pairs.pair[i] = land;
    }
    if (pairs.checked != cells.y) {
        pairs.checked = cells.y;
        for (int i = 0; i < cells.count; i++) {
            int c = cells.list[i];
            if (!(pairs.valid[c >> 5] >> (c & 31) & 1)) {
                pairs.pair[c] = pairInt(d.coverRgb[c], haze, h << 3, y);
                pairs.valid[c >> 5] |= 1u << (c & 31);
#if defined(BENCH3D) && defined(ESP_PLATFORM)
                stats[4]++;
#endif
            }
        }
    }
    lap(1);
    // A pixel at a time to a sample's edge, a sample at a time - four
    // pixels or eight - to the last whole one, then a pixel at a time.
    int x = xa, shift = cells.shift, mask = (1 << shift) - 1;
    for (; x < xb && (x & mask) != 0; x++) {
        uint32_t p = pairs.pair[cells.cell[x >> shift]];
        row[x] = (uint16_t)(x & 1 ? p >> 16 : p);
    }
    int n = (xb - x) >> shift;
    if (n > 0) {
#if defined(ESP_PLATFORM)
        if (g_simd) {
            (shift == 3 ? scene3d_cover_write8 : scene3d_cover_write)(row + x, cells.cell + (x >> shift), pairs.pair, n);
        } else
#endif
        {
            (shift == 3 ? coverWrite8C : coverWriteC)(row + x, cells.cell + (x >> shift), pairs.pair, n);
        }
        x += n << shift;
    }
    for (; x < xb; x++) {
        uint32_t p = pairs.pair[cells.cell[x >> shift]];
        row[x] = (uint16_t)(x & 1 ? p >> 16 : p);
    }
    lap(2);
}

// Where a sample's cell is: the part of a tile it was last read from - its
// level, where it lies in the walk's units, and its cells - for the next
// sample, mostly in the same part, to be read straight from them.
struct CellPart {
    int level = 1 << 20;
    float u0 = 0, v0 = 0, u1 = -1, v1 = -1;  // the part, in units of the level
    float split = 1, ku = 0, kv = 0;         // units a tile, and cells a unit
    const uint8_t *cells = nullptr;
    int stride = 0, cols1 = 0, rows1 = 0;
};

// The pack's cell at (u, v) on the cover's grid, read at `level` - or, where
// the tile hasn't that level in, the finest it has that is coarser: as
// coverWalk() reads its cells, for one point - from `part` when it holds
// the place, else found and kept there. -1 off the tiles.
HOT_INLINE int cellAt(const groundpack::Window &W, float u, float v, int level, CellPart &part) {
    if (level == part.level) {
        float uu = u * part.split, vv = v * part.split;
        if (uu >= part.u0 && uu < part.u1 && vv >= part.v0 && vv < part.v1) {
            int col = std::min(part.cols1, (int)((uu - part.u0) * part.ku)),
                row = std::min(part.rows1, (int)((vv - part.v0) * part.kv));
            return part.cells[row * part.stride + col];
        }
    }
    int sBits = level <= 0 ? 1 - level : 0;  // units, 2^sBits to a tile each way
    float split = (float)(1 << sBits), uu = u * split, vv = v * split;
    int tu = floorInt(uu), tv = floorInt(vv), su = tu >> sBits, sv = tv >> sBits;
    if ((unsigned)su >= (unsigned)W.cols || (unsigned)sv >= (unsigned)W.rows) {
        return -1;
    }
    int slot = sv * W.cols + su, c0 = groundpack::cols0At(W.north + sv);
    const uint8_t *cells = nullptr;
    int at = level;
    for (;; at++) {
        if (at < 0) {
            cells = W.finer[-1 - at][slot];
        } else if (at == 0) {
            cells = W.detail[slot];
        } else if (at < groundpack::LEVELS - 1) {
            if (const uint8_t *mid = W.mid[slot]) {
                constexpr int R1 = groundpack::ROWS0 >> 1;
                int c1 = c0 >> 1;
                cells = mid + (at >= 2 ? R1 * c1 : 0) + (at >= 3 ? (R1 >> 1) * (c1 >> 1) : 0);
            }
        } else {
            cells = W.coarse[slot];
        }
        if (cells != nullptr || at >= groundpack::LEVELS - 1) {
            break;
        }
    }
    if (cells == nullptr || c0 == 0) {
        return -1;
    }
    int pBits = at <= 0 ? 1 - at : 0;  // its parts, 2^pBits each way
    int stride = at <= 0 ? c0 >> 1 : c0 >> at, rows = at <= 0 ? groundpack::ROWS0 >> 1 : groundpack::ROWS0 >> at;
    int k = sBits - pBits;
    int pu = (tu >> k) & ((1 << pBits) - 1), pv = (tv >> k) & ((1 << pBits) - 1);
    cells += (size_t)((pv << pBits) + pu) * rows * stride;
    float span = (float)(1 << k);
    part.level = level;
    part.split = split;
    part.u0 = (float)((tu >> k) << k);
    part.v0 = (float)((tv >> k) << k);
    part.u1 = part.u0 + span;
    part.v1 = part.v0 + span;
    part.ku = stride / span;
    part.kv = rows / span;
    part.cells = cells;
    part.stride = stride;
    part.cols1 = stride - 1;
    part.rows1 = rows - 1;
    int col = std::min(stride - 1, std::max(0, (int)((uu - part.u0) * part.ku))),
        row = std::min(rows - 1, std::max(0, (int)((vv - part.v0) * part.kv)));
    return cells[row * stride + col];
}

// The level a sample at depth z is read at: the finest whose cells are four
// pixels or more - a sample - wide there, from z's own exponent: level L's
// cells reach a sample at depth tLevelZ[L].
HOT_INLINE int levelFor(float z) {
    float q = z * d.tLevelPer;  // 1 at the finest level's reach
    uint32_t b;
    memcpy(&b, &q, 4);
    int e = (int)(b >> 23) - 127 + ((b & 0x7FFFFF) != 0);  // ceil(log2 q)
    return std::min(groundpack::LEVELS - 1, std::max(groundpack::FINEST, groundpack::FINEST + e));
}

// Rows [y0, y1) of a band's land at its height - see terrainMarch(): column
// by column, each column's rows from where it first meets the land, its
// depths read in order; the cell its depth puts there read every second
// row and kept for the row under it - a cell is two rows or more tall
// wherever it is four pixels wide or more - lit and hazed for its
// distance, four pixels of it a row; the row's own colours where the pack
// has nothing. Rows an airfield's flat ground fills right across are the
// flat ground's, drawn as it is.
// Where a column's samples are reading the cover: a part of a tile at one
// level, its cells as straight lines from the grid - col = u au + bu, row =
// v av + bv, a 1024 added so that a cast is a floor - and the depths that
// level is read over: a sample there, and inside, is a cell in two
// multiply-adds, two casts and two compares.
struct CellMap {
    float zLo = INFINITY, zHi = 0;  // (zLo, zHi]: none yet
    float au = 0, bu = 0, av = 0, bv = 0;
    const uint8_t *cells = nullptr;
    int cols = 0, rows = 0, stride = 0;
};

// The cell at (u, v) at depth z found the long way - cellAt() - and the map
// made from where it was; -1 off the tiles.
__attribute__((noinline)) int cellSlow(const groundpack::Window &W, float u, float v, float z, CellMap &map,
                                       CellPart &part) {
    int level = levelFor(z);
    int cell = cellAt(W, u, v, level, part);
    if (cell < 0 || part.level != level || part.cells == nullptr) {
        map.zLo = INFINITY;
        return cell;
    }
    // The depths levelFor() reads this level over, to the float: z
    // tLevelPer up to 2^e, e its level's from the finest, the largest z
    // whose product is no more - found by ulps from 2^e / tLevelPer - so
    // that every depth the map takes is one levelFor() puts at its level.
    // A ulp out, a bfloat16 depth shared by a patch of the far land read
    // the map's level by one route and the next level by the other.
    const float per = d.tLevelPer;
    auto edge = [per](int e) {
        float q = ldexpf(1.0f, e), z = q / per;
        while (z * per > q) {
            z = nextafterf(z, 0.0f);
        }
        while (nextafterf(z, INFINITY) * per <= q) {
            z = nextafterf(z, INFINITY);
        }
        return z;
    };
    map.zLo = level == groundpack::FINEST ? 0 : edge(level - groundpack::FINEST - 1);
    map.zHi = level == groundpack::LEVELS - 1 ? INFINITY : edge(level - groundpack::FINEST);
    map.au = part.split * part.ku;
    map.bu = 1024 - part.u0 * part.ku;
    map.av = part.split * part.kv;
    map.bv = 1024 - part.v0 * part.kv;
    map.cells = part.cells;
    map.cols = part.cols1 + 1;
    map.rows = part.rows1 + 1;
    map.stride = part.stride;
    // And the cell as the map reads it, where it reads this one: the same
    // place one cell or the other, whichever sample came to the part first.
    int col = (int)fmaf(u, map.au, map.bu) - 1024, row = (int)fmaf(v, map.av, map.bv) - 1024;
    if (z > map.zLo && z <= map.zHi && (unsigned)col < (unsigned)map.cols && (unsigned)row < (unsigned)map.rows) {
        return map.cells[row * map.stride + col];
    }
    return cell;
}

// A column of a band's land as terrainBand() draws it, and as
// scene3d_terrain_rows (scene3d_terrain.S) does the most of it: the column,
// the band's rows, where on the cover's grid its depths put the land, the
// cell map, and each parity of row's last pair of pixels and what they
// were for. Its offsets are the assembly's.
struct TerrainCol {
    uint32_t *px;            // 0: row y's four pixels, two pairs
    const uint16_t *zc;      // 4: 1 / depth by row, bfloat16 - zc[y]
    int y, yb, flatTop;      // 8, 12, 16: the next row, the one after the last, the airfield's flat ground's first
    int stride;              // 20: a row's bytes
    float cu, cv, ue, ve;    // 24, 28, 32, 36: u = ue + z (rowU[y] + cu), v likewise
    const float *rowU;       // 40
    const float *rowV;       // 44
    const float *rowFar;     // 48
    const float *flatZ;      // 52
    const uint8_t *haze;     // 56: by the distance's float bits >> 21
    float zLo, zHi;          // 60, 64: the cell map - see CellMap
    float au, bu, av, bv;    // 68, 72, 76, 80
    const uint8_t *cells;    // 84
    int cols, rows, mapStride;  // 88, 92, 96
    uint32_t keyE, pairE, keyO, pairO;  // 100, 104, 108, 112: cell | haze << 8, and its pair, each parity's
    uint32_t samples;        // 116
    TerrainPairs *pairs[2];  // 120, 124: each parity's cache of pairs
    // The band's, for the assembly to go from column to column itself.
    int c, bandCols;         // 128, 132: the column it goes on from, and how many
    int y0;                  // 136: the band's first row
    const uint32_t *topFlat; // 140: each column's highest row of land | its flat top << 16
    const float *colUV;      // 144: each column's cu and cv
    int rowAt;               // 148: the row - a pair's even, or an odd one alone - it goes on from
    const uint16_t *zs;      // 152: the band's 1 / depths, column by column
    int bandBytes;           // 156: a column's of them, bandRows * 2
    uint32_t *color;         // 160: the band's row y0, x nought
    // A pair of pixels not in the cache, worked out as pairInt() does.
    const uint8_t *coverRgb; // 164: each cell's colour, lit
    const int *hazeColor;    // 168: the haze's
    int swap;                // 172: bytes swapped for the panel
    uint32_t misses;         // 176: pairs worked out
    float mapU[6], mapV[6];  // 180, 204: round, where on the grid a place dx east and dy north is
};
#if defined(ESP_PLATFORM)
static_assert(offsetof(TerrainCol, flatZ) == 52 && offsetof(TerrainCol, haze) == 56 &&
                  offsetof(TerrainCol, cells) == 84 && offsetof(TerrainCol, keyE) == 100 &&
                  offsetof(TerrainCol, samples) == 116 && offsetof(TerrainCol, pairs) == 120,
              "scene3d_terrain.S has TerrainCol's fields where these are");
static_assert(offsetof(TerrainPairs, key) == 4 && offsetof(TerrainPairs, pair) == 260,
              "scene3d_terrain.S has TerrainPairs' fields where these are");
static_assert(offsetof(TerrainCol, c) == 128 && offsetof(TerrainCol, topFlat) == 140 &&
                  offsetof(TerrainCol, rowAt) == 148 && offsetof(TerrainCol, color) == 160 &&
                  offsetof(TerrainCol, coverRgb) == 164 && offsetof(TerrainCol, misses) == 176 &&
                  offsetof(TerrainCol, mapU) == 180 && offsetof(TerrainCol, mapV) == 204,
              "scene3d_terrain.S has TerrainCol's fields where these are");
#endif

// The map's fields in t, from `map`, and back.
HOT_INLINE void mapInto(TerrainCol &t, const CellMap &map) {
    t.zLo = map.zLo;
    t.zHi = map.zHi;
    t.au = map.au;
    t.bu = map.bu;
    t.av = map.av;
    t.bv = map.bv;
    t.cells = map.cells;
    t.cols = map.cols;
    t.rows = map.rows;
    t.mapStride = map.stride;
}

// Row y's pair of pixels for `key`, from the parity's cache - worked out and
// kept there if it isn't in it - and remembered as the parity's last.
HOT_INLINE uint32_t pairFor(TerrainCol &t, uint32_t key, int y, const int haze[3], uint32_t &misses) {
    uint32_t &lastKey = y & 1 ? t.keyO : t.keyE, &lastPair = y & 1 ? t.pairO : t.pairE;
    if (key != lastKey) {
        TerrainPairs &tp = *t.pairs[y & 1];
        int cell = key & 255, hz = key >> 8;
        int slot = (cell ^ hz * 37) & 127;
        if (tp.key[slot] != key) {
            tp.key[slot] = (uint16_t)key;
            tp.pair[slot] = pairInt(d.coverRgb[cell], haze, hz << 3, y);
            misses++;
        }
        lastKey = key;
        lastPair = tp.pair[slot];
    }
    return lastPair;
}

// The rows of t's column from t.y the plain way: the sample at t.y - its
// depth, where on the cover that puts it, its cell and haze - drawn on
// `rows` rows from t.y. What the assembly hands back where it can't go on:
// another part of the cover.
template <bool ROUND>
HOT_INLINE void terrainSampleC(TerrainCol &t, int rows, CellMap &map, CellPart &part, const groundpack::Window &W,
                               const int haze[3], uint32_t &longWay, uint32_t &misses) {
    int y = t.y;
    float z = y >= t.flatTop ? t.flatZ[y] : 1 / fromBf16(t.zc[y]), u, v;
    if (!ROUND) {
        u = t.ue + z * (t.rowU[y] + t.cu);
        v = t.ve + z * (t.rowV[y] + t.cv);
    } else {
        // Fused as scene3d_terrain_band_round fuses it, to the bit.
        float dx = z * (t.rowU[y] + t.cu), dy = z * (t.rowV[y] + t.cv);
        const float *U = t.mapU, *V = t.mapV;
        u = fmaf(dy, fmaf(U[5], dy, U[2]), fmaf(dx, fmaf(U[4], dy, fmaf(U[3], dx, U[1])), U[0]));
        v = fmaf(dy, fmaf(V[5], dy, V[2]), fmaf(dx, fmaf(V[4], dy, fmaf(V[3], dx, V[1])), V[0]));
    }
    int col = (int)fmaf(u, t.au, t.bu) - 1024, row = (int)fmaf(v, t.av, t.bv) - 1024, cell;
    if (z > t.zLo && z <= t.zHi && (unsigned)col < (unsigned)t.cols && (unsigned)row < (unsigned)t.rows) {
        cell = t.cells[row * t.mapStride + col];
    } else {
        longWay++;
        cell = cellSlow(W, u, v, z, map, part);
        mapInto(t, map);
    }
#ifdef BENCH3D
    if (g_terrainProbe & 2) {
        cell = 0x35 + ((int)(u * 64) & 0x40);
    }
#endif
    float dist = z * t.rowFar[y];
    uint32_t bits;
    memcpy(&bits, &dist, 4);
    uint32_t key = (uint32_t)cell | (uint32_t)t.haze[bits >> 21] << 8;
    t.samples++;
    if (cell >= 16) {
        uint32_t *px = t.px;
        for (int k = 0; k < rows; k++, px = (uint32_t *)((uint8_t *)px + t.stride)) {
            uint32_t p = pairFor(t, key, y + k, haze, misses);
            px[0] = p;
            px[1] = p;
        }
    }
    t.px = (uint32_t *)((uint8_t *)t.px + rows * t.stride);
    t.y += rows;
}

#if defined(ESP_PLATFORM)
// The band's land in assembly, a pair of rows at a time across every
// column, from row t.rowAt and column t.c: 0 once it is all done, 1 at a
// sample it can't do - another part of the cover, a pair of pixels not in
// the cache - with t.y, t.px and the column's own values set for
// terrainSampleC(), and t.c the column after it. See scene3d_terrain.S.
extern "C" int scene3d_terrain_band(TerrainCol *t);
// The same for the round cover, its grid's place worked out to the second
// order. And both four rows a sample.
extern "C" int scene3d_terrain_band_round(TerrainCol *t);
extern "C" int scene3d_terrain_quad(TerrainCol *t);
extern "C" int scene3d_terrain_quad_round(TerrainCol *t);
#endif

// Rows [y0, y1) of a band's land at its height - see terrainMarch(): column
// by column, each column's rows from where it first meets the land, its
// depths read in order; the cell its depth puts there read every second
// row and kept for the row under it - a cell is two rows or more tall
// wherever it is four pixels wide or more - lit and hazed for its
// distance, four pixels of it a row; the row's own colours where the pack
// has nothing. Rows an airfield's flat ground fills right across are the
// flat ground's, drawn as it is; above them, a column's own rows of it have
// their depths from the plane. The pairs of rows in assembly, flat; what
// it can't do, and round, here.
void terrainBand(uint16_t *color, int y0, int y1, const int haze[3]) {
    int yb = std::min(y1, d.tFlatAll);
    if (y0 >= yb) {
        return;
    }
    int core = coreNow();
    uint64_t *stats = g_terrainStats[core];
    uint32_t clock = cycles();
#if defined(ESP_PLATFORM) && defined(BENCH3D)
    uint32_t instret0;
    __asm__ volatile("csrr %0, minstret" : "=r"(instret0));
#endif
    CACHE_BEGIN(cacheAtStart);
    TerrainCol t;
    t.pairs[0] = &g_terrainPairs[core * 2];
    t.pairs[1] = &g_terrainPairs[core * 2 + 1];
    for (TerrainPairs *tp : t.pairs) {
        if (tp->serial != d.serial) {
            tp->serial = d.serial;
            memset(tp->key, 0xFF, sizeof tp->key);
        }
    }
    const groundpack::Window &W = *d.cover.window;
    const bool round = d.tRound;
    int cols = d.tCols, bandRows = d.bandRows;
    t.stride = d.w * 2;
    t.ue = d.tUe;
    t.ve = d.tVe;
    t.rowU = d.tRowU.data();
    t.rowV = d.tRowV.data();
    t.rowFar = d.tRowFar.data();
    t.flatZ = d.tFlatZ.data();
    t.haze = d.tHaze;
    t.samples = 0;
    const int16_t *tops = d.tTop.data();
    const uint16_t *zs = d.tz.data() + (size_t)(y0 / bandRows) * cols * bandRows;  // this band's
    uint32_t longWay = 0, misses = 0;
    CellPart part;
    CellMap map;
    mapInto(t, map);
    // The rows a sample on row y covers: to the next unit's first row - a
    // multiple of the unit, whatever band it is in - or the band's end. The
    // unit 2 or 4, a mask rather than a division.
    const int unit = d.tUnit;
    auto rowsFrom = [&](int y) { return std::min((y & -unit) + unit, yb) - y; };
    if (round) {
        // Round, the rows' and columns' parts of dx and dy, and the map.
        t.rowU = d.tRowX.data();
        t.rowV = d.tRowY.data();
        memcpy(t.mapU, d.tMapU, sizeof t.mapU);
        memcpy(t.mapV, d.tMapV, sizeof t.mapV);
    }
#if defined(ESP_PLATFORM)
#ifdef BENCH3D
    bool inC = g_terrainProbe & 4;  // the bench's check: C alone, to compare
#else
    const bool inC = false;
#endif
    if (!inC) {
        // Every column in assembly; what it can't do here, a sample at a
        // time, and on.
        t.c = 0;
        t.bandCols = cols;
        t.rowAt = y0;
        t.y0 = y0;
        t.yb = yb;
        t.topFlat = d.tTopFlat.data();
        t.colUV = round ? d.tColXY.data() : d.tColUV.data();
        t.zs = zs;
        t.bandBytes = bandRows * 2;
        t.color = (uint32_t *)color;
        t.coverRgb = &d.coverRgb[0][0];
        t.hazeColor = haze;
        t.swap = g.swap;
        t.misses = 0;
        t.keyE = t.keyO = ~0u;
        uint32_t handed = 0;
        int (*fn)(TerrainCol *) = unit == 4 ? (round ? scene3d_terrain_quad_round : scene3d_terrain_quad)
                                            : (round ? scene3d_terrain_band_round : scene3d_terrain_band);
        while (fn(&t)) {
            if (round) {
                terrainSampleC<true>(t, rowsFrom(t.y), map, part, W, haze, longWay, misses);
            } else {
                terrainSampleC<false>(t, rowsFrom(t.y), map, part, W, haze, longWay, misses);
            }
            handed++;
        }
#ifdef BENCH3D
        g_bandHanded[core] += handed;
#endif
        misses += t.misses;
        cols = 0;
    }
#endif
#if defined(BENCH3D) || !defined(ESP_PLATFORM)
    // The bench's check of the check: C in the assembly's order - row by
    // row-pair across every column - for what the order alone changes.
    bool rowMajor = g_terrainProbe & 16;
#if !defined(ESP_PLATFORM)
    rowMajor = rowMajor || getenv("ROW_MAJOR") != nullptr;
#endif
    if (rowMajor && unit == 2) {
        auto at = [&](int c, int y, int rows) {
            t.zc = zs + (size_t)c * bandRows - y0;
            t.px = (uint32_t *)color + (size_t)(y - y0) * (d.w / 2) + 2 * c;
            t.y = y;
            t.yb = yb;
            t.flatTop = d.tFlatTop[c];
            t.cu = round ? d.tColX[c] : d.tColU[c];
            t.cv = round ? d.tColY[c] : d.tColV[c];
            if (round) {
                terrainSampleC<true>(t, rows, map, part, W, haze, longWay, misses);
            } else {
                terrainSampleC<false>(t, rows, map, part, W, haze, longWay, misses);
            }
        };
        t.keyE = t.keyO = ~0u;
        for (int y = y0; y < yb;) {
            if (y & 1) {
                for (int c = 0; c < cols; c++) {
                    if (tops[c] <= y) {
                        at(c, y, 1);
                    }
                }
                y++;
                continue;
            }
            for (int c = 0; c < cols; c++) {
                int top = tops[c];
                if (top > y + 1) {
                    continue;
                }
                if (top == y + 1) {
                    if (y + 1 < yb) {
                        at(c, y + 1, 1);
                    }
                    continue;
                }
                at(c, y, y + 1 < yb ? 2 : 1);
            }
            y += 2;
        }
        cols = 0;
    }
#endif
    for (int c = 0; c < cols; c++) {
        int ya = std::max(y0, (int)tops[c]);
        if (ya >= yb) {
            continue;
        }
        t.zc = zs + (size_t)c * bandRows - y0;  // this band's rows of column c, by row
        t.px = (uint32_t *)color + (size_t)(ya - y0) * (d.w / 2) + 2 * c;
        t.y = ya;
        t.yb = yb;
        t.flatTop = d.tFlatTop[c];
        t.cu = round ? d.tColX[c] : d.tColU[c];
        t.cv = round ? d.tColY[c] : d.tColV[c];
        t.keyE = t.keyO = ~0u;
        // A sample on the column's first row, then on each unit's first.
        while (t.y < yb) {
            if (round) {
                terrainSampleC<true>(t, rowsFrom(t.y), map, part, W, haze, longWay, misses);
            } else {
                terrainSampleC<false>(t, rowsFrom(t.y), map, part, W, haze, longWay, misses);
            }
        }
    }
    stats[4] += cycles() - clock;
    stats[5] += t.samples;
    stats[6] += longWay;
    stats[7] += misses;
#if defined(ESP_PLATFORM) && defined(BENCH3D)
    uint32_t instret1;
    __asm__ volatile("csrr %0, minstret" : "=r"(instret1));
    g_bandInstret[core] += instret1 - instret0;
#endif
    CACHE_END(cacheAtStart, g_terrainCache[core] + 4);
}

// The terrain's depth over rows [y0, y1) of a band, for what stands up from
// it to be hidden behind it: each fourth pixel's column's, nearer the
// larger, and nought - infinitely far - where a column has no land. A row
// all on an airfield's flat ground one depth across.
void terrainDepth(uint16_t *depth, int y0, int y1) {
    int w = d.w, cols = d.tCols, stride = w / 2;
    int yb = std::min(y1, d.tFlatAll);
    for (int y = std::max(y0, yb); y < y1; y++) {
        float z = d.tFlatZ[y];
        uint16_t v = z > 0.5f ? depthOf(1 / z) : 0;
        fill(depth + (size_t)(y - y0) * w, 0, w, v | (uint32_t)v << 16);
    }
    if (y0 >= yb) {
        return;
    }
    fill(depth, 0, w * (yb - y0), 0);  // infinitely far
    const int16_t *tops = d.tTop.data();
    int bandRows = d.bandRows;
    const uint16_t *zs = d.tz.data() + (size_t)(y0 / bandRows) * cols * bandRows;  // this band's
    float k = d.depthK * 1.002f;
    // The airfield's flat ground's, a row's alone.
    uint16_t flat[64];
    for (int y = y0; y < yb && y - y0 < 64; y++) {
        float z = d.tFlatZ[y];
        flat[y - y0] = z > 0.5f ? (uint16_t)std::min(65535.0f, k / z) : 0;
    }
    // Bands an even number of rows, the march keeps only the even rows' and
    // each column's top's: an odd row under it has the even row above's -
    // a row out at most, nearer by under a row's depth.
    const bool everyRow = bandRows & 1;
    for (int c = 0; c < cols; c++) {
        int ya = std::max(y0, (int)tops[c]), flatTop = d.tFlatTop[c];
        const uint16_t *zc = zs + (size_t)c * bandRows - y0;
        uint32_t *px = (uint32_t *)depth + (size_t)(ya - y0) * stride + 2 * c;
        uint32_t v = 0;
        for (int y = ya; y < yb; y++, px += stride) {
            if (y >= flatTop) {
                v = flat[y - y0];
                v |= v << 16;
            } else if (everyRow || !(y & 1) || y == ya) {
                v = (uint32_t)std::min(65535.0f, k * fromBf16(zc[y]));
                v |= v << 16;
            }
            px[0] = v;
            px[1] = v;
        }
    }
}

#ifdef BENCH3D
void coverMicro(int w, int h, const Scene &scene, const Camera &camera, uint32_t out[4]) {
    for (int i = 0; i < 4; i++) {
        out[i] = 0;
    }
#if defined(ESP_PLATFORM)
    prepareBegin(w, h, 16, true, scene, camera);
    prepareAircraft(0, 1);
    prepareEnd();
    // An even row of ground three quarters of the way down, its points
    // worked out over and over - in the cache, all of it, after the first.
    int y = h * 3 / 4 & ~1;
    bool round = g.cover.round;
    Frame::RowCover rc;
    Vec3 dir = g.forward + g.down * ((y + 0.5f - g.cy) / g.f);
    float er = g.scene.earthRadius;
    if (!g.coverOn || !g.rowGround[y]) {
        flip();
        return;
    }
    auto row = [&](int r, Vec3 rdir) {
        if (round) {
            coverAnchors(r, rdir, er, rc);
        } else {
            coverFlatRow(g, r, rc);
        }
    };
    row(y, dir);
    uint32_t c0 = esp_cpu_get_cycle_count();
    for (int i = 0; i < 256; i++) {
        row(y, dir);
        __asm__ volatile("" ::: "memory");
    }
    out[0] = (esp_cpu_get_cycle_count() - c0) / 256;
    float u = 0, v = 0;
    Vec3 p = g.eye + dir * (g.eye.z > 0 ? 1.0f : 0.0f);
    c0 = esp_cpu_get_cycle_count();
    for (int i = 0; i < 256; i++) {
        coverPoint(p, er, u, v);
        __asm__ volatile("" ::: "memory");
    }
    out[1] = (esp_cpu_get_cycle_count() - c0) / 256;
    if (round) {
        c0 = esp_cpu_get_cycle_count();
        for (int i = 0; i < 256; i++) {
            g.rowCover[y >> 1] = rc;
            __asm__ volatile("" ::: "memory");
        }
        out[2] = (esp_cpu_get_cycle_count() - c0) / 256;
    }
    // And every even row's, once, as the frame has them.
    c0 = esp_cpu_get_cycle_count();
    for (int r = 0; r < h; r += 2) {
        if (g.rowGround[r]) {
            row(r, g.forward + g.down * ((r + 0.5f - g.cy) / g.f));
        }
    }
    out[3] = (esp_cpu_get_cycle_count() - c0);
    flip();
#endif
}

void aircraftStats(uint64_t out[6], bool reset) {
#if defined(ESP_PLATFORM)
    for (int i = 0; i < 6; i++) {
        out[i] = g_aircraftStats[0][i] + g_aircraftStats[1][i];
        if (reset) {
            g_aircraftStats[0][i] = g_aircraftStats[1][i] = 0;
        }
    }
#else
    (void)reset;
    for (int i = 0; i < 6; i++) {
        out[i] = 0;
    }
#endif
}

void seeStats(uint64_t out[7], bool reset) {
#if defined(ESP_PLATFORM)
    for (int i = 0; i < 7; i++) {
        out[i] = g_seeStats[0][i] + g_seeStats[1][i];
        if (reset) {
            g_seeStats[0][i] = g_seeStats[1][i] = 0;
        }
    }
#else
    (void)reset;
    for (int i = 0; i < 7; i++) {
        out[i] = 0;
    }
#endif
}

void coverStats(uint64_t out[11], bool reset) {
#if defined(ESP_PLATFORM)
    for (int i = 0; i < 11; i++) {
        out[i] = g_coverStats[0][i] + g_coverStats[1][i];
        if (reset) {
            g_coverStats[0][i] = g_coverStats[1][i] = 0;
        }
    }
#else
    (void)reset;
    for (int i = 0; i < 11; i++) {
        out[i] = 0;
    }
#endif
}
#endif

// Row y's pixels where a range's hill is - its column's top at or above the
// row, its foot below it - in the hill's colour, and with its depth where
// the row has depth, `depth` not nullptr: scene3d_hills8's sums.
void hillsRowC(uint16_t *row, uint16_t *depth, int y, int from, int to, const Frame::HillRange &range) {
    const uint16_t *pixels = range.pixels[y & 1];
    for (int x = from; x < to; x++) {
        if (range.from[x] <= y && y < range.to[x]) {
            row[x] = pixels[x];
            if (depth != nullptr) {
                depth[x] = range.depth[x];
            }
        }
    }
}

#if defined(ESP_PLATFORM)
extern "C" void scene3d_hills8(uint16_t *row, const int16_t *top, const int16_t *foot, const uint16_t *hill, int y,
                               int chunks);
extern "C" void scene3d_hills8d(uint16_t *row, uint16_t *depth, const int16_t *top, const int16_t *foot,
                                const uint16_t *hill, const uint16_t *hillDepth, int y, int chunks);
#endif

void hillsRow(uint16_t *row, uint16_t *depth, int y, const Frame::HillRange &range) {
#if defined(ESP_PLATFORM)
    if (g_simd) {
        int chunks = d.w / 8;
        if (depth != nullptr) {
            scene3d_hills8d(row, depth, range.from, range.to, range.pixels[y & 1], range.depth, y, chunks);
        } else {
            scene3d_hills8(row, range.from, range.to, range.pixels[y & 1], y, chunks);
        }
        hillsRowC(row, depth, y, chunks * 8, d.w, range);
        return;
    }
#endif
    hillsRowC(row, depth, y, 0, d.w, range);
}

void drawBandInto(int band, uint16_t *color, uint16_t *depth, uint32_t *stageUs) {
    uint32_t mark = stageUs != nullptr ? now() : 0;
    auto stage = [&](int i) {
        if (stageUs != nullptr) {
            uint32_t t = now();
            stageUs[i] += t - mark;
            mark = t;
        }
    };
    int w = d.w, y0 = band * d.bandRows, rows = std::min(d.bandRows, d.h - y0), y1 = y0 + rows;
    const int haze[3] = {(int)d.pal.haze.r, (int)d.pal.haze.g, (int)d.pal.haze.b};

    for (int y = y0; y < y1; y++) {
        uint16_t *row = color + (size_t)(y - y0) * w;
        // Land in what covers it, where the view has the ground pack - and
        // has it flat: at its height, the terrain's, over the row below.
        bool covered = d.coverOn && (!d.terrainOn || y >= d.tFlatAll) && d.rowGround[y] &&
                       (!d.cover.round || d.rowCover[y >> 1].level != Frame::NO_LEVEL);
        if (!d.landSea || !d.rowGround[y]) {
            if (covered && d.rowGround[y] && !d.cover.round) {
                coverSpan(row, y, 0, w, d.rowColor[y], haze);  // an airfield inland: all of it land
            } else {
                fill(row, 0, w, d.rowColor[y]);
            }
            continue;
        }
        bool on[2] = {(d.rowFirst[y] & 1) != 0, (d.rowFirst[y] & 2) != 0};  // land; a town
        // Each stretch drawn as one - in the cover, or in one colour - not
        // broken where a crossing changes neither: a town's edge, where the
        // cover is drawn for towns too.
        auto covers = [&]() { return covered && on[0] && (!on[1] || d.coverTowns); };
        auto color = [&]() { return !on[0] ? d.rowColor[y] : on[1] ? d.rowTown[y] : d.rowLand[y]; };
        int x = 0;
        bool cover = covers();
        uint32_t flat = cover ? 0 : color();
        auto stretch = [&](int to) {
            if (to <= x) {
                return;
            }
            if (cover) {
                coverSpan(row, y, x, to, d.rowLand[y], haze);
            } else {
                fill(row, x, to, flat);
            }
            x = to;
        };
        for (uint32_t i = d.rowStart[y]; i < d.rowStart[y + 1]; i++) {
            int to = std::max(x, std::min(w, (int)d.rowCross[i].x));
            on[d.rowCross[i].layer] = d.rowCross[i].on;
            bool nowCover = covers();
            uint32_t nowFlat = nowCover ? 0 : color();
            if (nowCover != cover || nowFlat != flat) {
                stretch(to);
                cover = nowCover;
                flat = nowFlat;
            }
        }
        stretch(w);
    }
    if (d.terrainOn) {
        terrainBand(color, y0, y1, haze);
    }
    stage(0);

    // The ground, painted in order - the runway over the grass, its markings
    // over the runway - with no depth to it: it is all at the same height.
    const Bins &gb = d.groundBins;
    for (uint32_t k = gb.start[band]; k < gb.start[band + 1]; k++) {
        groundRows(d.ground[gb.index[k] & PLACE_MASK], color, y0, y1, haze);
    }

    stage(1);

    // Shadows: where any falls, marked - in the depth buffer, not yet in use
    // - then each pixel marked darkened once.
    // Only between the leftmost and rightmost any shadow reaches in a row is
    // looked at again - where the whole band was, a busy apron spent 14ms
    // of a frame on it.
    const Bins &sb = d.shadowBins;
    if (sb.start[band] < sb.start[band + 1]) {
        constexpr int MAX_ROWS = 64;
        int16_t left[MAX_ROWS], right[MAX_ROWS];
        for (int r = 0; r < rows; r++) {
            left[r] = (int16_t)w;
            right[r] = 0;
        }
        fill(depth, 0, w * rows, 0);  // eight to a PIE store, not memset's one word
        for (uint32_t k = sb.start[band]; k < sb.start[band + 1]; k++) {
            uint32_t at = sb.index[k];
            raster((*d.shadowLists[at >> LIST_SHIFT])[at & PLACE_MASK], y0, y1, [&](int y, int xs, int xe) {
                fill(depth + (size_t)(y - y0) * w, xs, xe, 0x00010001);
                left[y - y0] = (int16_t)std::min<int>(left[y - y0], xs);
                right[y - y0] = (int16_t)std::max<int>(right[y - y0], xe);
            });
        }
        for (int r = 0; r < rows; r++) {
            darkenRow(color + (size_t)r * w, depth + (size_t)r * w, left[r], right[r]);
        }
    }

    stage(2);

    // What stands up from it, nearest wins; then the wireframe and the
    // lights, behind or in front of that as they are.
    const Bins &ob = d.solidBins, &wb = d.wireBins, &lb = d.spriteBins, &cb = d.clearBins, &db = d.dotBins;
    bool deep = ob.start[band] < ob.start[band + 1] || wb.start[band] < wb.start[band + 1] ||
                lb.start[band] < lb.start[band + 1] || cb.start[band] < cb.start[band + 1] ||
                db.start[band] < db.start[band + 1];
    if (deep) {
        if (d.terrainOn) {
            terrainDepth(depth, y0, y1);  // the land's, where it has a height
        } else {
            fill(depth, 0, w * rows, 0);  // infinitely far - eight to a PIE store, not memset's one word
        }
    }
    // The hills along the skyline, over the sky and the far land - and over
    // the ground's paint and
    // shadows, which are behind a hill wherever one is drawn over them: a
    // point of the ground as far off as a hill's foot is on the foot's row,
    // and further off, above it. With their depth, so that what is drawn
    // after is behind them where it is further off - a coast's pale line
    // twenty miles away, an aircraft beyond a hill.
    for (int r = groundpack::Horizon::RANGES - 1; r >= 0; r--) {
        const Frame::HillRange &range = d.hills[r];
        if (range.on && y1 > range.top && y0 < range.bottom) {
            for (int y = std::max(y0, range.top); y < std::min(y1, range.bottom); y++) {
                hillsRow(color + (size_t)(y - y0) * w, deep ? depth + (size_t)(y - y0) * w : nullptr, y, range);
            }
        }
    }
    stage(3);
    // Depth across a row in 16.16 fixed point, in whole-number steps.
    float fixed = d.depthK * 65536.0f;
#if defined(BENCH3D) && defined(ESP_PLATFORM)
    uint64_t *solidStats = g_seeStats[xPortGetCoreID() & 1];
    uint32_t solidClock = esp_cpu_get_cycle_count();
    uint32_t solidUs = now();
    solidStats[4] += ob.start[band + 1] - ob.start[band];
#endif
    for (uint32_t k = ob.start[band]; k < ob.start[band + 1]; k++) {
        uint32_t at = ob.index[k];
        const ScreenTri &t = (*d.solidLists[at >> LIST_SHIFT])[at & PLACE_MASK];
        Spanner spanner((int32_t)(t.za * fixed), t.color);
#if defined(BENCH3D) && defined(ESP_PLATFORM)
        solidStats[3] += std::max(0, std::min(y1, (int)t.bottom) - std::max(y0, (int)t.top));
#endif
        solidRows(t, color, depth, y0, y1, fixed, spanner);
    }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
    solidStats[2] += esp_cpu_get_cycle_count() - solidClock;
#endif
#if defined(BENCH3D) && defined(ESP_PLATFORM)
    uint64_t *seeStats = g_seeStats[xPortGetCoreID() & 1];
    uint32_t seeClock = esp_cpu_get_cycle_count();
#endif
    // See-through: contrails, and turning blades' blur, where nothing nearer
    // is - what is behind tinted so many eighths of the way to its colour,
    // the depth left as it was. The depth scale is set by what stands, not
    // by these: a trail going by the eye is nearer than any of it, past the
    // top of 16 bits - which went round to nought, behind the sky, and
    // dropped each span's first pixel, a dark line down the trail. Nearer
    // than the scale reaches, its depth is held at the top, in front of
    // everything, as it is; and steep across the screen - edge on, close -
    // held level, so no chunk of eight runs it past 16 bits either.
    constexpr float FRONT = DEPTH_TOP, STEEP = 64;
    for (uint32_t k = cb.start[band]; k < cb.start[band + 1]; k++) {
        uint32_t at = cb.index[k];
        const ScreenTri &t = (*d.clearLists[at >> LIST_SHIFT])[at & PLACE_MASK];
        uint16_t tc = d.swap ? (uint16_t)(t.color >> 8 | t.color << 8) : t.color;
        float perPixel = t.za * d.depthK;
        bool steep = fabsf(perPixel) > STEEP;
        Tinter tinter(steep ? 0 : (int32_t)(t.za * fixed), tc, t.lit[0]), level(0, tc, t.lit[0]);
        raster(t, y0, y1, [&](int y, int xs, int xe) {
#if defined(BENCH3D) && defined(ESP_PLATFORM)
            seeStats[1] += xe - xs;
#endif
            uint16_t *c = color + (size_t)(y - y0) * w, *z = depth + (size_t)(y - y0) * w;
            // Its depth at xs, and where along the span it reaches FRONT -
            // [xs, xf) below it and [xf, xe) at it, or the other way round
            // coming nearer leftwards. A step nearer than it is, so the
            // chunks' sums - rounded down once and rounded down again -
            // never come out under nought where it is right on it.
            float d0 = (t.za * (xs + 0.5f) + t.zb * (y + 0.5f) + t.zc) * d.depthK + 1;
            auto fixedAt = [&](float depthThere) { return (int32_t)(std::max(0.0f, depthThere) * 65536.0f); };
            if (steep) {
                float middle = d0 + perPixel * (xe - xs - 1) * 0.5f;
                tint(c, z, xs, xe, fixedAt(std::min(FRONT, middle)), level);
                return;
            }
            float d1 = d0 + perPixel * (xe - xs - 1);
            if (std::max(d0, d1) < FRONT) {
                tint(c, z, xs, xe, fixedAt(d0), tinter);
            } else if (std::min(d0, d1) >= FRONT) {
                tint(c, z, xs, xe, fixedAt(FRONT), level);
            } else if (perPixel > 0) {
                int xf = std::min(xe, xs + (int)ceilf((FRONT - d0) / perPixel));
                tint(c, z, xs, xf, fixedAt(d0), tinter);
                tint(c, z, xf, xe, fixedAt(FRONT), level);
            } else {
                int xf = std::min(xe, xs + (int)floorf((FRONT - d0) / perPixel) + 1);
                tint(c, z, xs, xf, fixedAt(FRONT), level);
                tint(c, z, xf, xe, fixedAt(d0 + perPixel * (xf - xs)), tinter);
            }
        });
    }
#if defined(BENCH3D) && defined(ESP_PLATFORM)
    seeStats[0] += esp_cpu_get_cycle_count() - seeClock;
    solidStats[5] += now() - solidUs;
    solidStats[6] += esp_cpu_get_cycle_count() - solidClock;
#endif
    stage(4);
    for (uint32_t k = wb.start[band]; k < wb.start[band + 1]; k++) {
        drawLine(d.wires[wb.index[k] & PLACE_MASK], y0, y1, color, depth);
    }
    // The dots before the lights' glows, which add to what is under them.
    for (uint32_t k = db.start[band]; k < db.start[band + 1]; k++) {
        const ScreenDot &dot = d.dots[db.index[k] & PLACE_MASK];
        size_t at = (size_t)(dot.top - y0) * w + dot.x;
        if (depthOf(dot.iz) >= depth[at]) {
            color[at] = dot.color;
        }
    }
    for (uint32_t k = lb.start[band]; k < lb.start[band + 1]; k++) {
        uint32_t at = lb.index[k];
        drawSprite((*d.spriteLists[at >> LIST_SHIFT])[at & PLACE_MASK], y0, y1, color, depth);
    }
    // Each aircraft's middle and top that are in this band: whether what is
    // drawn there is nearer than any of it could be. Each the one band's,
    // so the two cores drawing bands never write the same one.
    if (deep) {
        for (Placed &p : d.placed) {
            for (int k = 0; k < 2; k++) {
                int x = p.probeX[k], y = p.probeY[k];
                if (y >= y0 && y < y1 && x >= 0 && x < w) {
                    p.covered[k] = depth[(size_t)(y - y0) * w + x] > depthOf(p.probeIz);
                }
            }
        }
    }
    stage(5);
}

int labelAnchors(Label *out, int max) {
    // Where a point is in the frame drawn, and how far.
    auto seen = [](Vec3 p, float &x, float &y, float &z) {
        Vec3 v = p - d.eye;
        z = dot(v, d.forward);
        if (z < NEAR) {
            return false;
        }
        x = d.cx + d.f * dot(v, d.right) / z;
        y = d.cy + d.f * dot(v, d.down) / z;
        return true;
    };
    // The nearest few that are big on the screen, as the boxes they fill
    // there, and inside the box as their simplest models do: a circle round
    // one seen from behind, as the one followed is, takes in as much sky
    // above and below its wings again - and so does its box, banked - and
    // hides what is plainly in view beyond it.
    struct Big {
        size_t index;
        float x0, y0, x1, y1;
    } big[8];
    int bigCount = 0;
    for (size_t j = 0; j < d.placed.size() && bigCount < 8; j++) {
        const Placed &q = d.placed[j];
        if (q.reach < 40 * 0.96f) {
            continue;
        }
        const Design &qd = designOf(q.look);
        Placer placer(q.pose, q.look.scale);
        Big b = {j, 1e9f, 1e9f, -1e9f, -1e9f};
        bool whole = true;  // no corner behind the eye
        for (int k = 0; k < 8 && whole; k++) {
            Vec3 corner = {k & 1 ? qd.hi.x : qd.lo.x, k & 2 ? qd.hi.y : qd.lo.y, k & 4 ? qd.hi.z : qd.lo.z};
            float cx = 0, cy = 0, cz = 0;
            whole = seen(placer.place(corner), cx, cy, cz);
            b.x0 = std::min(b.x0, cx), b.y0 = std::min(b.y0, cy), b.x1 = std::max(b.x1, cx), b.y1 = std::max(b.y1, cy);
        }
        if (whole) {
            big[bigCount++] = b;
        }
    }
    auto covers = [&](const Big &b, float x, float y) {
        const Placed &q = d.placed[b.index];
        const Mesh &mesh = designOf(q.look).lod[DETAIL_TINY];
        Placer placer(q.pose, q.look.scale);
        for (size_t f = 0; f < mesh.faceCount(); f++) {
            float px[3], py[3], pz;
            bool whole = true;
            for (int k = 0; k < 3 && whole; k++) {
                whole = seen(placer.place(mesh.corner(mesh.faces[f * 3 + k])), px[k], py[k], pz);
            }
            if (!whole) {
                continue;
            }
            float e0 = (px[1] - px[0]) * (y - py[0]) - (py[1] - py[0]) * (x - px[0]);
            float e1 = (px[2] - px[1]) * (y - py[1]) - (py[2] - py[1]) * (x - px[1]);
            float e2 = (px[0] - px[2]) * (y - py[2]) - (py[0] - py[2]) * (x - px[2]);
            if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) {
                return true;
            }
        }
        return false;
    };
    int n = 0;
    for (size_t i = 0; i < d.placed.size() && n < max; i++) {
        const Placed &p = d.placed[i];
        if (p.followed) {
            continue;
        }
        // Well off the screen, by where its middle is: no further.
        if (p.reach <= 0 || p.screenX < -p.reach || p.screenX > d.w + p.reach || p.screenY < -p.reach ||
            p.screenY > d.h + p.reach) {
            continue;
        }
        float x, y, z;
        const Design &dsn = designOf(p.look);
        if (!seen(Placer(p.pose, p.look.scale).place({dsn.middle.x, 0, dsn.hi.z + 1.5f}), x, y, z) || x < 0 ||
            x >= d.w || y < 0 || y >= d.h) {
            continue;
        }
        // Behind a building, or anything else drawn, as the frame before
        // found - its middle and its top both covered there - is hidden: it
        // as it was then, by where it was and its kind.
        bool hidden = false;
        for (const Placed &q : g.placed) {
            Vec3 moved = q.pose.at - p.pose.at;
            if (!q.followed && q.look.family == p.look.family && dot(moved, moved) < 40 * 40) {
                hidden = q.covered[0] && q.covered[1];
                break;
            }
        }
        // Behind a nearer aircraft - one placed before it, the list being
        // nearest first - is hidden: inside the box a big one fills, or the
        // circle a small one covers.
        for (size_t j = 0; j < i && !hidden; j++) {
            const Big *b = nullptr;
            for (int k = 0; k < bigCount && b == nullptr; k++) {
                b = big[k].index == j ? &big[k] : nullptr;
            }
            if (b != nullptr) {
                hidden = x > b->x0 && x < b->x1 && y > b->y0 && y < b->y1 && covers(*b, x, y);
                continue;
            }
            const Placed &q = d.placed[j];
            if (q.reach > 0) {
                hidden = (x - q.screenX) * (x - q.screenX) + (y - q.screenY) * (y - q.screenY) < q.reach * q.reach;
            }
        }
        if (!hidden) {
            out[n++] = {x, y, z, p.index, d.f * dsn.halfLength * 2 * p.look.scale / z};
        }
    }
    return n;
}

bool followedOnScreen(float &x, float &y, float &reach) {
    for (const Placed &p : d.placed) {
        if (!p.followed) {
            continue;
        }
        const Design &dsn = designOf(p.look);
        Vec3 v = Placer(p.pose, p.look.scale).place(dsn.middle) - d.eye;
        float z = dot(v, d.forward);
        if (z < NEAR) {
            return false;
        }
        x = d.cx + d.f * dot(v, d.right) / z;
        y = d.cy + d.f * dot(v, d.down) / z;
        reach = d.f * dsn.halfLength * p.look.scale / z;
        return true;
    }
    return false;
}

bool projectPoint(Vec3 p, float &x, float &y) {
    Vec3 v = p - d.eye, c = {dot(v, d.right), dot(v, d.down), dot(v, d.forward)};
    if (c.z < NEAR) {
        return false;
    }
    x = d.cx + d.f * c.x / c.z;
    y = d.cy + d.f * c.y / c.z;
    return true;
}

bool skylineDrawn(Vec3 &eye, Vec3 &right, Vec3 &down, Vec3 &forward, float &f, float &cx, float &cy,
                  const int16_t *&top, const int16_t *&foot, int &width) {
    eye = d.eye;
    right = d.right;
    down = d.down;
    forward = d.forward;
    f = d.f;
    cx = d.cx;
    cy = d.cy;
    width = d.w;
    const Frame::HillRange &range = d.hills[0];
    top = range.from;
    foot = range.to;
    return range.on && top != nullptr && foot != nullptr;
}

// --- the landing and the traffic ---------------------------------------------------

namespace {

constexpr float GLIDE = 3 * DEG, AIM_X = 350, FLARE_HEIGHT = 9, START_X = -1200;
// The slope it touches down at: 0.8 m/s down at 70 along, 160 feet a minute.
constexpr float TOUCH_SLOPE = 0.0114f;
constexpr float APPROACH_SPEED = 70, TAXI_SPEED = 12;  // m/s: 136 knots, and 23
constexpr float TURN_R = 60;
constexpr float APPROACH_PITCH = 3 * DEG, FLARE_PITCH = 6 * DEG, ROLLING_PITCH = 0.2f * DEG, DEROTATE_S = 4;

// The landing on a runway's own terms - x down it from the threshold, y
// towards its taxiway - worked out for where its way off is.
struct Path {
    float exitX, rolloutEndX, turnX;
    float flareX, flareLength, touchX, decel;
    float tFlare, tTouch, tRolled, tTurn, tTurned, tEnd;
};

Path pathFor(float exitX) {
    Path q;
    q.exitX = exitX;
    q.rolloutEndX = exitX - 100;
    q.turnX = exitX - TURN_R;
    float tanGlide = tanf(GLIDE);
    q.flareX = AIM_X - FLARE_HEIGHT / tanGlide;
    // A flare that eases the descent from the glide path's slope to the
    // touchdown's, over the distance an even easing takes.
    q.flareLength = 2 * FLARE_HEIGHT / (tanGlide + TOUCH_SLOPE);
    q.touchX = q.flareX + q.flareLength;
    q.decel = (APPROACH_SPEED * APPROACH_SPEED - TAXI_SPEED * TAXI_SPEED) /
              (2 * std::max(300.0f, q.rolloutEndX - q.touchX));
    q.tFlare = (q.flareX - START_X) / APPROACH_SPEED;
    q.tTouch = q.tFlare + q.flareLength / APPROACH_SPEED;
    q.tRolled = q.tTouch + (APPROACH_SPEED - TAXI_SPEED) / q.decel;
    q.tTurn = q.tRolled + (q.turnX - q.rolloutEndX) / TAXI_SPEED;
    q.tTurned = q.tTurn + PI / 2 * TURN_R / TAXI_SPEED;
    q.tEnd = q.tTurned + (TAXIWAY_OFFSET - TURN_R) / TAXI_SPEED;
    return q;
}

Path g_path;
bool g_pathReady = false;

const RunwayEnd &landingEnd() { return models().ends[g_landingEnd]; }

const Path &path() {
    if (!g_pathReady) {
        g_path = pathFor(landingEnd().exitAlong);
        g_pathReady = true;
    }
    return g_path;
}

// A pose on a runway end's own terms in the world - its y mirrored where the
// taxiway is on the right.
Pose onRunway(const RunwayEnd &end, Pose local) {
    if (end.exitRight) {
        local.at.y = -local.at.y;
        local.heading = -local.heading;
    }
    Vec3 at = end.origin + end.along * local.at.x - end.right * local.at.y + Vec3{0, 0, local.at.z};
    return {at, atan2f(end.along.y, end.along.x) + local.heading, local.pitch};
}

Vec3 onRunway(const RunwayEnd &end, Vec3 local) { return onRunway(end, Pose{local, 0, 0}).at; }

// The landing at `t` - before 0, further out on the approach.
Pose localLanding(const Path &p, float t) {
    t = std::min(t, p.tEnd);
    if (t < p.tFlare) {
        float x = START_X + APPROACH_SPEED * t;
        return {{x, 0, (AIM_X - x) * tanf(GLIDE)}, 0, APPROACH_PITCH};
    }
    if (t < p.tTouch) {
        // A cubic from the flare's height and slope to the ground at the
        // touchdown's slope.
        float s = APPROACH_SPEED * (t - p.tFlare), u = s / p.flareLength, L = p.flareLength;
        float h = FLARE_HEIGHT * (2 * u * u * u - 3 * u * u + 1) - tanf(GLIDE) * L * (u * u * u - 2 * u * u + u) -
                  TOUCH_SLOPE * L * (u * u * u - u * u);
        return {{p.flareX + s, 0, std::max(0.0f, h)}, 0, APPROACH_PITCH + (FLARE_PITCH - APPROACH_PITCH) * u};
    }
    // On the ground: the nose let down onto its wheel over the first seconds.
    float pitch = std::max(ROLLING_PITCH, FLARE_PITCH - (FLARE_PITCH - ROLLING_PITCH) * (t - p.tTouch) / DEROTATE_S);
    if (t < p.tRolled) {
        float u = t - p.tTouch;
        return {{p.touchX + APPROACH_SPEED * u - p.decel * u * u / 2, 0, 0}, 0, pitch};
    }
    if (t < p.tTurn) {
        return {{p.rolloutEndX + TAXI_SPEED * (t - p.tRolled), 0, 0}, 0, pitch};
    }
    if (t < p.tTurned) {
        float a = TAXI_SPEED * (t - p.tTurn) / TURN_R;
        return {{p.turnX + TURN_R * sinf(a), TURN_R - TURN_R * cosf(a), 0}, a, pitch};
    }
    return {{p.turnX + TURN_R, TURN_R + TAXI_SPEED * (t - p.tTurned), 0}, PI / 2, pitch};
}

// A departure: lined up on the threshold until 0, then the take-off roll,
// rotating at 75 m/s - 146 knots - lifting off at 80 and climbing away.
Pose localTakeoff(float t) {
    constexpr float LINED_UP = 40, ACCEL = 2.1f, ROTATE = 75, LIFTOFF = 80, CLIMB = 7 * DEG, CLIMB_PITCH = 10 * DEG;
    if (t <= 0) {
        return {{LINED_UP, 0, 0}, 0, ROLLING_PITCH};
    }
    float tRotate = ROTATE / ACCEL, tLift = LIFTOFF / ACCEL;
    if (t < tLift) {
        float pitch = t > tRotate ? (t - tRotate) / (tLift - tRotate) * CLIMB_PITCH : ROLLING_PITCH;
        return {{LINED_UP + ACCEL * t * t / 2, 0, 0}, 0, pitch};
    }
    float u = t - tLift, s = LINED_UP + ACCEL * tLift * tLift / 2 + LIFTOFF * u;
    return {{s, 0, LIFTOFF * u * tanf(CLIMB)}, 0, CLIMB_PITCH};
}

}  // namespace

void liftTower(Models &m);

void setAirfield(const RunwaySpec *runways, int count, const Vec3 *beacons, int beaconCount,
                 const airfield::Layout *layout, bool madeUp) {
    Models &m = models();
    withRoom([&] {
        if (count <= 0) {
            buildDefaultAirfield(m);
        } else {
            buildAirfield(m, runways, count, beacons, beaconCount, layout, madeUp);
        }
        liftTower(m);
    });
    g_landingEnd = 0;
    g_departureEnd = -1;
    g_pathReady = false;
}

void clearAirfield() {
    Models &m = models();
    m.ground.clear();
    m.paintLines.clear();
    m.solids.clear();
    m.detail.clear();
    m.solidNormals.clear();
    m.detailNormals.clear();
    m.groundTris.clear();
    m.solidTris.clear();
    m.detailTris.clear();
    m.lightPoints.clear();
    m.lightColors.clear();
    m.lightSizes.clear();
    m.wires.clear();
    m.airfieldLights.clear();
    m.townLights.clear();
    for (Vector<Cell> *cells :
         {&m.groundCells, &m.paintCells, &m.solidCells, &m.detailCells, &m.wireCells, &m.lightCells, &m.townCells}) {
        cells->clear();
    }
    m.papis.clear();
    m.ends.clear();
    m.stands.clear();
    m.blocks.clear();
    g_landingEnd = 0;
    g_departureEnd = -1;
    g_pathReady = false;
}

bool towerEye(Vec3 &eye) {
    const Models &m = models();
    if (m.ends.empty()) {
        return false;
    }
    // Just outside the cab, on the airfield's side - not on the tower's own
    // mast, which would stand up through the middle of the view.
    Vec3 towards = m.middle - m.tower;
    towards.z = 0;
    float away = length(towards);
    eye = m.tower + (away > 1 ? towards * (8 / away) : Vec3{0, 0, 0}) + Vec3{0, 0, m.towerEyeZ + m.towerLift};
    return true;
}

bool seenPast(Vec3 eye, Vec3 at, bool outside) {
    const Models &m = models();
    const float from[3] = {eye.x, eye.y, eye.z}, way[3] = {at.x - eye.x, at.y - eye.y, at.z - eye.z};
    for (const Block &b : m.blocks) {
        if (outside && eye.x >= b.x0 && eye.x <= b.x1 && eye.y >= b.y0 && eye.y <= b.y1) {
            continue;
        }
        // The part of the line from the eye to it between each pair of the
        // box's sides - west and east, south and north, the ground and its
        // roof - and of that, what is between all three: none, it is clear.
        const float lo[3] = {b.x0, b.y0, 0}, hi[3] = {b.x1, b.y1, b.top};
        float t0 = 0, t1 = 1;
        for (int k = 0; k < 3 && t0 <= t1; k++) {
            if (fabsf(way[k]) < 1e-6f) {
                if (from[k] < lo[k] || from[k] > hi[k]) {
                    t1 = -1;
                }
                continue;
            }
            float a = (lo[k] - from[k]) / way[k], c = (hi[k] - from[k]) / way[k];
            t0 = std::max(t0, std::min(a, c));
            t1 = std::min(t1, std::max(a, c));
        }
        if (t0 <= t1) {
            return false;
        }
    }
    return true;
}

// The tower's camera raised - five metres at a time, sixty at most - until
// it sees nine in ten of the points along its runways, an aircraft's height
// up, past the buildings: a landing watched from the cab had been behind a
// terminal.
void liftTower(Models &m) {
    m.towerLift = 0;
    if (m.ends.empty() || m.blocks.empty()) {
        return;
    }
    Vec3 towards = m.middle - m.tower;
    towards.z = 0;
    float away = length(towards);
    Vec3 eye = m.tower + (away > 1 ? towards * (8 / away) : Vec3{0, 0, 0}) + Vec3{0, 0, m.towerEyeZ};
    for (float lift = 0; lift <= 60; lift += 5) {
        int seen = 0, all = 0;
        for (size_t e = 0; e + 1 < m.ends.size(); e += 2) {
            const RunwayEnd &r = m.ends[e];
            for (float a = 0; a <= r.length; a += 150) {
                all++;
                seen += seenPast(eye + Vec3{0, 0, lift}, r.at(a, 0) + Vec3{0, 0, 4}, true) ? 1 : 0;
            }
        }
        m.towerLift = lift;
        if (all == 0 || seen * 10 >= all * 9) {
            break;
        }
    }
}

float towerRaised() { return models().towerLift; }

void setGroundLines(const GroundLine *lines, int count) {
    Models &m = models();
    m.groundLines.assign(lines, lines + std::max(0, count));
    m.groundLines.shrink_to_fit();
    m.towns = std::any_of(m.groundLines.begin(), m.groundLines.end(),
                          [](const GroundLine &l) { return l.kind == GroundKind::TOWN; });
    m.lineCells.clear();
    intoCells(m.groundLines, 0, m.groundLines.size(), COUNTRY_CELL, m.lineCells,
              [](const GroundLine &l, Vec3 &lo, Vec3 &hi) {
                  widen(l.a, 0, lo, hi);
                  widen(l.b, 0, lo, hi);
              });
}

void lightTowns(int most, float closest, float falloff, bool round) {
    Models &m = models();
    // The towns' outlines, and the ground they span.
    float x0 = 1e30f, y0 = 1e30f, x1 = -1e30f, y1 = -1e30f;
    size_t outlines = 0;
    for (const GroundLine &l : m.groundLines) {
        if (l.kind == GroundKind::TOWN) {
            x0 = std::min({x0, l.a.x, l.b.x}), x1 = std::max({x1, l.a.x, l.b.x});
            y0 = std::min({y0, l.a.y, l.b.y}), y1 = std::max({y1, l.a.y, l.b.y});
            outlines++;
        }
    }
    if (outlines == 0) {
        return;  // none here: the airfield's made-up ones, or none
    }
    // No further out than three times `falloff`, where a place is a tenth as
    // likely a light: the work is the grid's, and a city's was most of a
    // second on the device.
    float reach = 3 * falloff;
    x0 = std::max(x0, -reach), x1 = std::min(x1, reach), y0 = std::max(y0, -reach), y1 = std::min(y1, reach);
    if (x0 >= x1 || y0 >= y1) {
        return;
    }
    // Along rows of a grid `step` apart, where each row is inside a town -
    // as a row of the screen is told it: each outline crossing it leads in
    // going one way and out going the other - its crossings bucketed by row,
    // counted then placed; `visit` given each row's and its runs inside.
    static Vector<uint32_t> starts;
    static Vector<std::pair<float, bool>> crossings;
    auto rows = [&](float step, auto visit) {
        int n = std::max(1, (int)ceilf((y1 - y0) / step));
        starts.assign(n + 1, 0);
        auto each = [&](auto put) {
            for (const GroundLine &l : m.groundLines) {
                if (l.kind != GroundKind::TOWN || l.a.y == l.b.y) {
                    continue;
                }
                float lo = std::min(l.a.y, l.b.y), hi = std::max(l.a.y, l.b.y);
                int j0 = std::max(0, ceilInt((lo - y0) / step - 0.5f)), j1 = std::min(n, ceilInt((hi - y0) / step - 0.5f));
                for (int j = j0; j < j1; j++) {
                    float y = y0 + (j + 0.5f) * step;
                    put(j, l.a.x + (y - l.a.y) * (l.b.x - l.a.x) / (l.b.y - l.a.y), (l.b.y < l.a.y) != l.landRight);
                }
            }
        };
        each([&](int j, float, bool) { starts[j + 1]++; });
        for (int j = 0; j < n; j++) {
            starts[j + 1] += starts[j];
        }
        crossings.resize(starts[n]);
        static Vector<uint32_t> at;
        at.assign(starts.begin(), starts.end() - 1);
        each([&](int j, float x, bool in) { crossings[at[j]++] = {x, in}; });
        for (int j = 0; j < n; j++) {
            std::sort(crossings.begin() + starts[j], crossings.begin() + starts[j + 1]);
            for (uint32_t i = starts[j]; i + 1 < starts[j + 1]; i++) {
                if (crossings[i].second && !crossings[i + 1].second) {
                    visit(y0 + (j + 0.5f) * step, crossings[i].first, crossings[i + 1].first);
                }
            }
        }
    };
    // On a grid `closest` apart, each place in a town a light by chance -
    // the chance falling off with how far it is from the middle, the
    // airfield's or the aircraft's, `falloff` out a half - and all of them
    // scaled for `most` lights in all: first weighed, then strewn. Each a
    // little out of line: street lights and windows, the sodium's orange
    // and the newer white.
    float step = std::max(closest, std::max(x1 - x0, y1 - y0) / 4000);  // a grid of 16 million at most
    auto weight = [&](float x, float y) { return 1 / (1 + (x * x + y * y) / (falloff * falloff)); };
    // Weighed on a grid four times as coarse, each place there for sixteen.
    double weighed = 0;
    rows(step * 4, [&](float y, float a, float b) {
        for (float x = a + step * 2; x < b; x += step * 4) {
            weighed += 16 * weight(x, y);
        }
    });
    float chance = (float)std::min(1.0, most / std::max(1.0, weighed));
    const Rgb WARM[] = {{255, 196, 120}, {255, 226, 180}, {255, 170, 90}, {210, 225, 255}};
    float r = 6371000;  // high up, on the round earth, as the outlines are
    m.townLights.clear();
    g_seed = 54321;
    rows(step, [&](float y, float a, float b) {
        for (float x = a + step * 0.5f; x < b && (int)m.townLights.size() < most * 5 / 4; x += step) {
            if (random01() >= chance * weight(x, y)) {
                continue;
            }
            float px = x + (random01() - 0.5f) * step, py = y + (random01() - 0.5f) * step;
            float z = round ? -(px * px + py * py) / (2 * r) + 2 : 2;
            const Rgb &color = WARM[(int)(random01() * 4) & 3];
            m.townLights.push_back({{px, py, z}, color, 2, {0, 0, 0}, color});
        }
    });
    m.townLights.shrink_to_fit();
    m.townCells.clear();
    intoCells(m.townLights, 0, m.townLights.size(), round ? COUNTRY_CELL / 4 : AIRFIELD_CELL * 5, m.townCells,
              [](const ModelLight &l, Vec3 &lo, Vec3 &hi) { widen(l.at, l.size, lo, hi); });
}

void setRivers(const RiverLine *rivers, int count) {
    Models &m = models();
    m.rivers.clear();
    for (int i = 0; i < count; i++) {
        m.rivers.push_back({rivers[i].a, rivers[i].b, rivers[i].width, WATER});
    }
    m.rivers.shrink_to_fit();
    m.riverCells.clear();
    intoCells(m.rivers, 0, m.rivers.size(), COUNTRY_CELL / 4, m.riverCells, [](const PaintLine &l, Vec3 &lo, Vec3 &hi) {
        widen(l.a, l.width, lo, hi);
        widen(l.b, l.width, lo, hi);
    });
}

void setFarRunways(const RunwaySpec *runways, int count) {
    Models &m = models();
    m.farGround.clear();
    m.farLights.clear();
    for (int i = 0; i < count; i++) {
        const RunwaySpec &r = runways[i];
        Vec3 d = r.end[1] - r.end[0];
        float len = length(d);
        if (len < 1) {
            continue;
        }
        Vec3 u = d * (1 / len), side = Vec3{-u.y, u.x, 0} * (r.width / 2);
        rect(m.farGround, r.end[0] - side, r.end[1] - side, r.end[1] + side, r.end[0] + side, {70, 72, 76});
        // Its edges, a light every 300m, and a green bar across each end.
        for (float a = 0; a <= len; a += 300) {
            for (float s : {1.0f, -1.0f}) {
                m.farLights.push_back({r.end[0] + u * a + side * (s * 1.1f) + Vec3{0, 0, 1}, LAMP_WHITE, 6,
                                       {0, 0, 0}, LAMP_WHITE});
            }
        }
        for (int e = 0; e < 2; e++) {
            m.farLights.push_back({r.end[e] + Vec3{0, 0, 1}, LAMP_GREEN, 9, {0, 0, 0}, LAMP_GREEN});
        }
    }
}

void setLanding(int runway, int end) {
    g_landingEnd = std::max(0, std::min((int)models().ends.size() - 1, runway * 2 + end));
    g_pathReady = false;
}

void setDeparture(int runway, int end) {
    g_departureEnd = runway < 0 ? -1 : std::min((int)models().ends.size() - 1, runway * 2 + end);
}

float landingSeconds() { return path().tEnd; }
float landingTouchdown() { return path().tTouch; }

Pose landingPose(float t) { return onRunway(landingEnd(), localLanding(path(), t)); }

namespace {

using F = Family;

// ICAO types, each with its family and its length in metres.
struct TypeShape {
    const char *type;
    Family family;
    float length;
};

const TypeShape TYPES[] = {
    // Airbus and Boeing narrowbodies, and their kin
    {"A318", F::NARROWBODY, 31.44f}, {"A319", F::NARROWBODY, 33.84f}, {"A19N", F::NARROWBODY, 33.84f},
    {"A320", F::NARROWBODY, 37.57f}, {"A20N", F::NARROWBODY, 37.57f}, {"A321", F::NARROWBODY, 44.51f},
    {"A21N", F::NARROWBODY, 44.51f}, {"B731", F::NARROWBODY, 28.6f},  {"B732", F::NARROWBODY, 30.5f},
    {"B733", F::NARROWBODY, 33.4f},  {"B734", F::NARROWBODY, 36.4f},  {"B735", F::NARROWBODY, 31.0f},
    {"B736", F::NARROWBODY, 31.2f},  {"B737", F::NARROWBODY, 33.6f},  {"B738", F::NARROWBODY, 39.5f},
    {"B739", F::NARROWBODY, 42.1f},  {"B37M", F::NARROWBODY, 35.56f}, {"B38M", F::NARROWBODY, 39.52f},
    {"B39M", F::NARROWBODY, 42.16f}, {"B3XM", F::NARROWBODY, 43.8f},  {"B752", F::NARROWBODY, 47.32f},
    {"B753", F::NARROWBODY, 54.43f}, {"BCS1", F::NARROWBODY, 35.0f},  {"BCS3", F::NARROWBODY, 38.7f},
    {"E170", F::NARROWBODY, 29.9f},  {"E75L", F::NARROWBODY, 31.7f},  {"E75S", F::NARROWBODY, 31.7f},
    {"E190", F::NARROWBODY, 36.2f},  {"E195", F::NARROWBODY, 38.7f},  {"E290", F::NARROWBODY, 36.2f},
    {"E295", F::NARROWBODY, 41.5f},  {"C919", F::NARROWBODY, 38.9f},  {"SU95", F::NARROWBODY, 29.94f},
    {"T204", F::NARROWBODY, 46.14f}, {"P8", F::NARROWBODY, 39.47f},   {"E737", F::NARROWBODY, 33.6f},
    // The 707s, and the DC-8
    {"B703", F::NARROWBODY_4, 46.61f}, {"K35R", F::NARROWBODY_4, 41.53f}, {"K35E", F::NARROWBODY_4, 41.53f},
    {"C135", F::NARROWBODY_4, 41.53f}, {"R135", F::NARROWBODY_4, 41.53f}, {"E6", F::NARROWBODY_4, 46.61f},
    {"E8", F::NARROWBODY_4, 46.61f},   {"DC86", F::NARROWBODY_4, 57.1f},  {"DC87", F::NARROWBODY_4, 57.1f},
    {"E3TF", F::AWACS, 46.61f},        {"E3CF", F::AWACS, 46.61f},
    // Widebodies
    {"A306", F::WIDEBODY, 54.08f}, {"A30B", F::WIDEBODY, 53.62f}, {"A310", F::WIDEBODY, 46.66f},
    {"A332", F::WIDEBODY, 58.82f}, {"A333", F::WIDEBODY, 63.66f}, {"A338", F::WIDEBODY, 58.82f},
    {"A339", F::WIDEBODY, 63.66f}, {"A359", F::WIDEBODY, 66.8f},  {"A35K", F::WIDEBODY, 73.79f},
    {"B762", F::WIDEBODY, 48.51f}, {"B763", F::WIDEBODY, 54.94f}, {"B764", F::WIDEBODY, 61.37f},
    {"B772", F::WIDEBODY, 63.73f}, {"B773", F::WIDEBODY, 73.86f}, {"B77L", F::WIDEBODY, 63.73f},
    {"B77W", F::WIDEBODY, 73.86f}, {"B778", F::WIDEBODY, 70.86f}, {"B779", F::WIDEBODY, 76.72f},
    {"B788", F::WIDEBODY, 56.72f}, {"B789", F::WIDEBODY, 62.81f}, {"B78X", F::WIDEBODY, 68.28f},
    {"MD11", F::WIDEBODY, 61.62f}, {"DC10", F::WIDEBODY, 55.5f},  {"L101", F::WIDEBODY, 54.17f},
    {"A3ST", F::WIDEBODY, 56.15f}, {"A337", F::WIDEBODY, 63.1f},
    {"A342", F::WIDEBODY_4, 59.4f}, {"A343", F::WIDEBODY_4, 63.69f}, {"A345", F::WIDEBODY_4, 67.93f},
    {"A346", F::WIDEBODY_4, 75.36f}, {"IL86", F::WIDEBODY_4, 60.21f}, {"IL96", F::WIDEBODY_4, 55.35f},
    {"B741", F::JUMBO, 70.6f}, {"B742", F::JUMBO, 70.6f}, {"B743", F::JUMBO, 70.6f}, {"B744", F::JUMBO, 70.66f},
    {"B748", F::JUMBO, 76.25f}, {"B74R", F::JUMBO, 70.6f}, {"B74S", F::JUMBO, 56.31f}, {"BLCF", F::JUMBO, 71.68f},
    {"A388", F::SUPERJUMBO, 72.72f},
    // Engines at the tail
    {"CRJ1", F::REGIONAL_JET, 26.77f}, {"CRJ2", F::REGIONAL_JET, 26.77f}, {"CRJ7", F::REGIONAL_JET, 32.51f},
    {"CRJ9", F::REGIONAL_JET, 36.2f},  {"CRJX", F::REGIONAL_JET, 39.1f},  {"E135", F::REGIONAL_JET, 26.33f},
    {"E145", F::REGIONAL_JET, 29.87f}, {"E35L", F::REGIONAL_JET, 26.33f}, {"F70", F::REGIONAL_JET, 30.91f},
    {"F100", F::REGIONAL_JET, 35.53f}, {"F28", F::REGIONAL_JET, 27.4f},   {"MD81", F::REGIONAL_JET, 45.06f},
    {"MD82", F::REGIONAL_JET, 45.06f}, {"MD83", F::REGIONAL_JET, 45.06f}, {"MD87", F::REGIONAL_JET, 39.75f},
    {"MD88", F::REGIONAL_JET, 45.06f}, {"MD90", F::REGIONAL_JET, 46.5f},  {"B712", F::REGIONAL_JET, 37.8f},
    {"DC93", F::REGIONAL_JET, 36.4f},  {"DC95", F::REGIONAL_JET, 40.7f},  {"B722", F::REGIONAL_JET, 46.7f},
    {"T154", F::REGIONAL_JET, 47.9f},  {"YK42", F::REGIONAL_JET, 36.4f},  {"IL62", F::REGIONAL_JET, 53.12f},
    {"VC10", F::REGIONAL_JET, 48.36f},
    {"GLF4", F::BUSINESS_JET, 26.92f}, {"GLF5", F::BUSINESS_JET, 29.4f},  {"GLF6", F::BUSINESS_JET, 30.41f},
    {"G280", F::BUSINESS_JET, 20.37f}, {"GA5C", F::BUSINESS_JET, 30.66f}, {"GA6C", F::BUSINESS_JET, 30.41f},
    {"GA7C", F::BUSINESS_JET, 33.48f}, {"GLEX", F::BUSINESS_JET, 30.3f},  {"GL5T", F::BUSINESS_JET, 29.5f},
    {"GL7T", F::BUSINESS_JET, 33.8f},  {"CL30", F::BUSINESS_JET, 20.92f}, {"CL35", F::BUSINESS_JET, 20.92f},
    {"CL60", F::BUSINESS_JET, 20.85f}, {"C25A", F::BUSINESS_JET, 13.0f},  {"C25B", F::BUSINESS_JET, 15.6f},
    {"C25C", F::BUSINESS_JET, 16.3f},  {"C25M", F::BUSINESS_JET, 12.9f},  {"C510", F::BUSINESS_JET, 12.37f},
    {"C525", F::BUSINESS_JET, 12.98f}, {"C550", F::BUSINESS_JET, 14.39f}, {"C560", F::BUSINESS_JET, 14.9f},
    {"C56X", F::BUSINESS_JET, 15.79f}, {"C650", F::BUSINESS_JET, 16.9f},  {"C680", F::BUSINESS_JET, 19.35f},
    {"C68A", F::BUSINESS_JET, 19.4f},  {"C700", F::BUSINESS_JET, 22.3f},  {"C750", F::BUSINESS_JET, 22.04f},
    {"LJ31", F::BUSINESS_JET, 14.83f}, {"LJ35", F::BUSINESS_JET, 14.83f}, {"LJ40", F::BUSINESS_JET, 16.43f},
    {"LJ45", F::BUSINESS_JET, 17.68f}, {"LJ60", F::BUSINESS_JET, 17.88f}, {"LJ75", F::BUSINESS_JET, 17.68f},
    {"H25B", F::BUSINESS_JET, 15.6f},  {"H25C", F::BUSINESS_JET, 15.6f},  {"HA4T", F::BUSINESS_JET, 21.08f},
    {"E50P", F::BUSINESS_JET, 12.82f}, {"E55P", F::BUSINESS_JET, 15.64f}, {"E545", F::BUSINESS_JET, 19.68f},
    {"E550", F::BUSINESS_JET, 20.74f}, {"FA7X", F::BUSINESS_JET, 23.19f}, {"FA8X", F::BUSINESS_JET, 24.46f},
    {"F900", F::BUSINESS_JET, 20.21f}, {"F2TH", F::BUSINESS_JET, 20.23f}, {"FA50", F::BUSINESS_JET, 18.52f},
    {"FA6X", F::BUSINESS_JET, 25.68f}, {"FA10", F::BUSINESS_JET, 13.86f}, {"FA20", F::BUSINESS_JET, 17.15f},
    {"PC24", F::BUSINESS_JET, 16.85f}, {"HDJT", F::BUSINESS_JET, 12.99f}, {"ASTR", F::BUSINESS_JET, 16.94f},
    {"GALX", F::BUSINESS_JET, 18.97f}, {"BE40", F::BUSINESS_JET, 14.75f}, {"PRM1", F::BUSINESS_JET, 14.02f},
    {"EA50", F::BUSINESS_JET, 10.12f}, {"SF50", F::BUSINESS_JET, 9.42f},
    // Propellers
    {"AT43", F::TURBOPROP, 22.67f}, {"AT44", F::TURBOPROP, 22.67f}, {"AT45", F::TURBOPROP, 22.67f},
    {"AT46", F::TURBOPROP, 22.67f}, {"AT72", F::TURBOPROP, 27.17f}, {"AT73", F::TURBOPROP, 27.17f},
    {"AT75", F::TURBOPROP, 27.17f}, {"AT76", F::TURBOPROP, 27.17f}, {"DH8A", F::TURBOPROP, 22.25f},
    {"DH8B", F::TURBOPROP, 22.25f}, {"DH8C", F::TURBOPROP, 25.68f}, {"DH8D", F::TURBOPROP, 32.83f},
    {"D328", F::TURBOPROP, 21.11f}, {"F50", F::TURBOPROP, 25.25f},  {"F27", F::TURBOPROP, 23.56f},
    {"DHC7", F::TURBOPROP, 24.58f}, {"L410", F::TURBOPROP, 14.42f}, {"AN24", F::TURBOPROP, 23.53f},
    {"SH36", F::TURBOPROP, 21.58f}, {"SH33", F::TURBOPROP, 17.69f},
    {"SF34", F::TURBOPROP_LOW, 19.73f}, {"SB20", F::TURBOPROP_LOW, 27.28f}, {"JS31", F::TURBOPROP_LOW, 14.37f},
    {"JS32", F::TURBOPROP_LOW, 14.37f}, {"JS41", F::TURBOPROP_LOW, 19.25f}, {"E110", F::TURBOPROP_LOW, 15.1f},
    {"E120", F::TURBOPROP_LOW, 20.0f},  {"SW4", F::TURBOPROP_LOW, 18.09f},  {"BE20", F::TURBOPROP_LOW, 13.34f},
    {"BE30", F::TURBOPROP_LOW, 14.22f}, {"B350", F::TURBOPROP_LOW, 14.22f}, {"BE9L", F::TURBOPROP_LOW, 10.82f},
    {"BE9T", F::TURBOPROP_LOW, 10.82f}, {"B190", F::TURBOPROP_LOW, 17.63f}, {"BE99", F::TURBOPROP_LOW, 13.58f},
    {"P180", F::TURBOPROP_LOW, 14.41f}, {"PA34", F::TURBOPROP_LOW, 8.72f},  {"PA31", F::TURBOPROP_LOW, 9.94f},
    {"PA44", F::TURBOPROP_LOW, 8.41f},  {"BE58", F::TURBOPROP_LOW, 9.09f},  {"BE55", F::TURBOPROP_LOW, 8.53f},
    {"BE76", F::TURBOPROP_LOW, 8.86f},  {"DA42", F::TURBOPROP_LOW, 8.56f},  {"DA62", F::TURBOPROP_LOW, 9.19f},
    {"C310", F::TURBOPROP_LOW, 9.74f},  {"C340", F::TURBOPROP_LOW, 10.46f}, {"C414", F::TURBOPROP_LOW, 11.09f},
    {"C421", F::TURBOPROP_LOW, 11.09f},
    {"C150", F::LIGHT_HIGH, 7.3f},   {"C152", F::LIGHT_HIGH, 7.3f},  {"C162", F::LIGHT_HIGH, 6.95f},
    {"C170", F::LIGHT_HIGH, 7.6f},   {"C172", F::LIGHT_HIGH, 8.28f}, {"C177", F::LIGHT_HIGH, 8.4f},
    {"C182", F::LIGHT_HIGH, 8.84f},  {"C185", F::LIGHT_HIGH, 7.85f}, {"C206", F::LIGHT_HIGH, 8.61f},
    {"C207", F::LIGHT_HIGH, 9.8f},   {"C208", F::LIGHT_HIGH, 11.46f}, {"C210", F::LIGHT_HIGH, 8.59f},
    {"PA18", F::LIGHT_HIGH, 6.88f},  {"DHC2", F::LIGHT_HIGH, 9.24f}, {"PC6", F::LIGHT_HIGH, 11.0f},
    {"GA8", F::LIGHT_HIGH, 8.95f},
    {"P28A", F::LIGHT_LOW, 7.25f}, {"P28B", F::LIGHT_LOW, 7.25f}, {"P28R", F::LIGHT_LOW, 7.6f},
    {"P32R", F::LIGHT_LOW, 8.4f},  {"PA32", F::LIGHT_LOW, 8.4f},  {"PA46", F::LIGHT_LOW, 8.81f},
    {"PA38", F::LIGHT_LOW, 7.0f},  {"SR20", F::LIGHT_LOW, 7.9f},  {"SR22", F::LIGHT_LOW, 7.92f},
    {"S22T", F::LIGHT_LOW, 7.92f}, {"DA40", F::LIGHT_LOW, 8.06f}, {"DA20", F::LIGHT_LOW, 7.16f},
    {"M20P", F::LIGHT_LOW, 7.5f},  {"BE33", F::LIGHT_LOW, 8.38f}, {"BE35", F::LIGHT_LOW, 8.38f},
    {"BE36", F::LIGHT_LOW, 8.38f}, {"AA5", F::LIGHT_LOW, 6.86f},  {"RV7", F::LIGHT_LOW, 6.2f},
    {"RV8", F::LIGHT_LOW, 6.2f},   {"TB10", F::LIGHT_LOW, 7.7f},  {"TB20", F::LIGHT_LOW, 7.7f},
    {"DR40", F::LIGHT_LOW, 6.96f}, {"G115", F::LIGHT_LOW, 7.4f},  {"G120", F::LIGHT_LOW, 8.4f},
    {"TBM7", F::LIGHT_LOW, 10.7f}, {"TBM8", F::LIGHT_LOW, 10.7f}, {"TBM9", F::LIGHT_LOW, 10.7f},
    {"PC12", F::LIGHT_LOW, 14.4f}, {"TEX2", F::LIGHT_LOW, 10.2f}, {"PC21", F::LIGHT_LOW, 11.2f},
    {"PC7", F::LIGHT_LOW, 9.78f},  {"PC9", F::LIGHT_LOW, 10.18f}, {"TUCA", F::LIGHT_LOW, 9.86f},
    {"E314", F::LIGHT_LOW, 11.38f},
    // Military transports
    {"C17", F::AIRLIFTER, 53.04f}, {"C5", F::AIRLIFTER, 75.31f}, {"C5M", F::AIRLIFTER, 75.31f},
    {"U2", F::BUSINESS_JET, 31.4f},  // a slim single-seat jet of a glider's span: the nearest there is
    {"IL76", F::AIRLIFTER, 46.59f}, {"A124", F::AIRLIFTER, 68.96f}, {"B52", F::AIRLIFTER, 48.5f},
    {"C130", F::HERCULES, 29.79f}, {"C30J", F::HERCULES, 34.37f}, {"L100", F::HERCULES, 34.37f},
    {"AN12", F::HERCULES, 33.1f},  {"A400", F::ATLAS, 45.1f},
    {"C295", F::TACTICAL_TWIN, 24.45f}, {"CN35", F::TACTICAL_TWIN, 21.4f},  {"C27J", F::TACTICAL_TWIN, 22.7f},
    {"C160", F::TACTICAL_TWIN, 32.4f},  {"G222", F::TACTICAL_TWIN, 22.7f},  {"AN26", F::TACTICAL_TWIN, 23.8f},
    {"AN32", F::TACTICAL_TWIN, 23.68f}, {"D228", F::TACTICAL_TWIN, 16.56f}, {"DHC6", F::TACTICAL_TWIN, 15.77f},
    {"BN2P", F::TACTICAL_TWIN, 10.86f}, {"BN2T", F::TACTICAL_TWIN, 10.86f}, {"TRIS", F::TACTICAL_TWIN, 15.01f},
    {"P68", F::TACTICAL_TWIN, 9.43f},   {"C212", F::TACTICAL_TWIN, 16.15f},
    {"B461", F::QUAD_HIGH, 26.2f}, {"B462", F::QUAD_HIGH, 28.6f}, {"B463", F::QUAD_HIGH, 31.0f},
    {"RJ70", F::QUAD_HIGH, 26.2f}, {"RJ85", F::QUAD_HIGH, 28.6f}, {"RJ1H", F::QUAD_HIGH, 31.0f},
    // Fast jets
    {"EUFI", F::FIGHTER_DELTA, 15.96f}, {"RFAL", F::FIGHTER_DELTA, 15.27f}, {"MIR2", F::FIGHTER_DELTA, 14.36f},
    {"F35", F::FIGHTER, 15.67f}, {"F22", F::FIGHTER, 18.92f},  {"F15", F::FIGHTER, 19.43f},
    {"F18H", F::FIGHTER, 17.07f}, {"F18S", F::FIGHTER, 18.31f}, {"F14", F::FIGHTER, 19.1f},
    {"SU27", F::FIGHTER, 21.94f}, {"SU30", F::FIGHTER, 21.94f}, {"SU35", F::FIGHTER, 21.9f},
    {"MG29", F::FIGHTER, 17.32f}, {"A10", F::FIGHTER, 16.26f},  {"BT7", F::FIGHTER, 14.1f},
    {"F16", F::FIGHTER_SINGLE, 15.06f}, {"TOR", F::FIGHTER_SINGLE, 16.72f}, {"JAGR", F::FIGHTER_SINGLE, 16.83f},
    {"F4", F::FIGHTER_SINGLE, 19.2f},   {"F5", F::FIGHTER_SINGLE, 14.45f},  {"MG21", F::FIGHTER_SINGLE, 14.7f},
    {"B1", F::FIGHTER_SINGLE, 44.5f},
    {"HAWK", F::JET_TRAINER, 11.95f}, {"T38", F::JET_TRAINER, 14.14f}, {"M346", F::JET_TRAINER, 11.49f},
    {"L39", F::JET_TRAINER, 12.13f},  {"L159", F::JET_TRAINER, 12.72f}, {"AJET", F::JET_TRAINER, 13.23f},
    {"T50", F::JET_TRAINER, 13.14f},
    // Helicopters
    {"A139", F::HELICOPTER, 13.77f}, {"A169", F::HELICOPTER, 12.5f},  {"A189", F::HELICOPTER, 14.6f},
    {"S92", F::HELICOPTER, 17.1f},   {"S76", F::HELICOPTER, 13.22f},  {"EC25", F::HELICOPTER, 16.79f},
    {"EC75", F::HELICOPTER, 15.68f}, {"AS32", F::HELICOPTER, 15.53f}, {"PUMA", F::HELICOPTER, 14.06f},
    {"H60", F::HELICOPTER, 15.26f},  {"EH10", F::HELICOPTER, 19.53f}, {"NH90", F::HELICOPTER, 16.13f},
    {"LYNX", F::HELICOPTER, 13.5f},  {"MI8", F::HELICOPTER, 18.17f},  {"H64", F::HELICOPTER, 15.0f},
    {"AS65", F::HELICOPTER, 11.63f}, {"A109", F::HELICOPTER, 11.45f}, {"B412", F::HELICOPTER, 12.92f},
    {"H160", F::HELICOPTER, 12.6f},  {"H53", F::HELICOPTER, 22.35f},  {"H53S", F::HELICOPTER, 22.35f},
    {"EC35", F::HELICOPTER_LIGHT, 10.2f},  {"EC45", F::HELICOPTER_LIGHT, 11.0f}, {"EC30", F::HELICOPTER_LIGHT, 10.68f},
    {"EC20", F::HELICOPTER_LIGHT, 9.6f},   {"AS50", F::HELICOPTER_LIGHT, 10.93f}, {"AS55", F::HELICOPTER_LIGHT, 10.93f},
    {"A119", F::HELICOPTER_LIGHT, 11.14f}, {"GAZL", F::HELICOPTER_LIGHT, 9.53f}, {"R22", F::HELICOPTER_LIGHT, 6.3f},
    {"R44", F::HELICOPTER_LIGHT, 8.9f},    {"R66", F::HELICOPTER_LIGHT, 9.0f},   {"B06", F::HELICOPTER_LIGHT, 9.5f},
    {"B407", F::HELICOPTER_LIGHT, 10.6f},  {"B429", F::HELICOPTER_LIGHT, 10.4f}, {"B505", F::HELICOPTER_LIGHT, 9.7f},
    {"UH1", F::HELICOPTER_LIGHT, 12.7f},   {"UH1Y", F::HELICOPTER_LIGHT, 13.6f}, {"SUCO", F::HELICOPTER_LIGHT, 13.9f},
    {"H47", F::CHINOOK, 15.8f},
};

// What a type not known is drawn as, by its emitter category: [0] for
// "A0" or none.
const TypeShape BY_CATEGORY[] = {
    {"A0", F::NARROWBODY, 37.6f},   {"A1", F::LIGHT_HIGH, 8.3f},       {"A2", F::BUSINESS_JET, 18.0f},
    {"A3", F::NARROWBODY, 37.6f},   {"A4", F::NARROWBODY, 47.3f},       {"A5", F::WIDEBODY, 63.0f},
    {"A6", F::FIGHTER_SINGLE, 15.0f}, {"A7", F::HELICOPTER_LIGHT, 11.0f}, {"B1", F::LIGHT_LOW, 7.4f},
    {"B4", F::LIGHT_HIGH, 6.0f},    {"B6", F::JET_TRAINER, 11.0f},
};

// The military's paint: grey for most, darker for the airlifters and the
// fighters' stealthier kind, black for the trainers, a green-grey for the
// helicopters.
uint32_t militaryGrey(Family f) {
    switch (f) {
    case F::AIRLIFTER:
    case F::HERCULES:
    case F::ATLAS:
    case F::TACTICAL_TWIN: return 0x7E848A;
    case F::FIGHTER: return 0x5E646A;
    case F::FIGHTER_DELTA:
    case F::FIGHTER_SINGLE: return 0x8D959B;
    case F::JET_TRAINER:
    case F::LIGHT_LOW: return 0x2C2E32;
    case F::HELICOPTER:
    case F::HELICOPTER_LIGHT:
    case F::CHINOOK: return 0x555D54;
    default: return 0x9AA0A6;
    }
}

uint32_t darker(uint32_t c, float k) {
    auto ch = [&](int s) { return (uint32_t)std::min(255.0f, (c >> s & 255) * k) << s; };
    return ch(16) | ch(8) | ch(0);
}

// Airlines' basic colours: body above and below, tail, engines - and a
// freighter, with no windows. Not their liveries, which no model this simple
// could carry: what tells one from another across an apron.
struct AirlinePaint {
    const char *airline;
    uint32_t body, belly, tail, engines;
    bool freighter;
};

constexpr uint32_t W = 0xF2F3F5, G = 0xCDD1D6;

const AirlinePaint AIRLINE_PAINT[] = {
    {"BAW", W, 0x1B2A55, 0x1B2A55, 0xC8CCD2, false}, {"SHT", W, 0x1B2A55, 0x1B2A55, 0xC8CCD2, false},
    {"CFE", W, 0x1B2A55, 0x1B2A55, 0xC8CCD2, false}, {"EZY", W, G, 0xFF6600, 0xFF6600, false},
    {"EJU", W, G, 0xFF6600, 0xFF6600, false},        {"EZS", W, G, 0xFF6600, 0xFF6600, false},
    {"RYR", W, 0x073590, 0x073590, 0x073590, false}, {"RUK", W, 0x073590, 0x073590, 0x073590, false},
    {"WZZ", W, G, 0xC6007E, 0x5B1E5E, false},        {"WUK", W, G, 0xC6007E, 0x5B1E5E, false},
    {"EXS", W, G, 0xE2231A, 0xE2231A, false},        {"LOG", W, G, 0x1F3F7F, W, false},
    {"TOM", W, G, 0x6CC4EE, 0x0B2A5B, false},        {"VIR", W, G, 0xE10A17, 0xB8BCC2, false},
    {"DLH", W, G, 0x05164D, 0x05164D, false},        {"CLH", W, G, 0x05164D, 0x05164D, false},
    {"KLM", 0x00A1DE, W, 0x00A1DE, 0x00A1DE, false}, {"KLC", 0x00A1DE, W, 0x00A1DE, 0x00A1DE, false},
    {"AFR", W, G, 0x002157, G, false},               {"IBE", W, G, 0xD7192D, G, false},
    {"SAS", W, G, 0x003057, 0x003057, false},        {"FIN", W, G, 0x0F1689, W, false},
    {"NOZ", W, G, 0xD91C24, W, false},               {"NSZ", W, G, 0xD91C24, W, false},
    {"SWR", W, G, 0xCC0000, W, false},               {"AUA", W, G, 0xCC0000, W, false},
    {"EIN", W, G, 0x006272, 0x006272, false},        {"EWG", W, G, 0x8A1538, W, false},
    {"ICE", W, 0x0E2C5A, 0x0E2C5A, W, false},        {"THY", W, G, 0xC70A0C, W, false},
    {"UAE", W, G, 0xD71921, W, false},               {"QTR", 0xD9DADC, 0xC4C6C9, 0x5C0632, 0x5C0632, false},
    {"ETD", 0xD8CCB4, 0xC8BBA0, 0x8C7853, 0xD8CCB4, false}, {"SIA", W, G, 0x1D4886, W, false},
    {"CPA", W, 0xC4C8CC, 0x006564, W, false},        {"QFA", W, G, 0xE40000, W, false},
    {"ANA", W, G, 0x13448F, 0x13448F, false},        {"JAL", W, G, 0xCC0000, W, false},
    {"AAL", 0xC9CED3, 0xB8BEC4, 0x0078D2, 0xC9CED3, false}, {"DAL", W, 0x00305F, 0xC01933, 0x00305F, false},
    {"UAL", W, 0x0033A0, 0x002244, 0x0033A0, false}, {"SWA", 0x304CB2, 0xC8102E, 0x304CB2, 0xB8BCC2, false},
    {"ACA", W, 0x231F20, 0x231F20, 0x231F20, false}, {"JBU", W, 0x003876, 0x003876, 0xB8BCC2, false},
    {"FDX", W, 0xA3A8AD, 0x4D148C, W, true},         {"UPS", 0xEBEBEB, 0x351C15, 0x351C15, 0x351C15, true},
    {"DHK", 0xFFCC00, 0xFFCC00, 0xFFCC00, 0xD40511, true}, {"BCS", 0xFFCC00, 0xFFCC00, 0xFFCC00, 0xD40511, true},
    {"DHX", 0xFFCC00, 0xFFCC00, 0xFFCC00, 0xD40511, true}, {"DAE", 0xFFCC00, 0xFFCC00, 0xFFCC00, 0xD40511, true},
    {"CLX", W, 0xC4C8CC, 0x1C3F94, W, true},
};

}  // namespace

Look lookFor(const char *type, uint32_t tail, bool military, const char *category) {
    models();
    const TypeShape *shape = nullptr;
    for (const TypeShape &t : TYPES) {
        if (type != nullptr && strcmp(type, t.type) == 0) {
            shape = &t;
            break;
        }
    }
    if (shape == nullptr) {
        shape = &BY_CATEGORY[0];
        for (const TypeShape &t : BY_CATEGORY) {
            if (category != nullptr && strcmp(category, t.type) == 0) {
                shape = &t;
            }
        }
        // A military aircraft of a type not known: a heavy one an airlifter
        // rather than an airliner, a rotorcraft one with wheels.
        if (military && shape->family == F::WIDEBODY) {
            static const TypeShape HEAVY = {"", F::AIRLIFTER, 53.0f};
            shape = &HEAVY;
        } else if (military && shape->family == F::HELICOPTER_LIGHT) {
            static const TypeShape ROTOR = {"", F::HELICOPTER, 15.0f};
            shape = &ROTOR;
        }
    }
    Look look;
    look.family = shape->family;
    look.scale = shape->length / designs()[(int)shape->family].length;
    look.tail = tail;
    if (military) {
        uint32_t grey = militaryGrey(look.family);
        look.body = look.engines = grey;
        look.belly = darker(grey, 0.9f);
        look.wings = darker(grey, 0.97f);
        look.tail = darker(grey, 0.94f);
        look.windows = false;
    }
    return look;
}

void paintAirline(Look &look, const char *callsign) {
    if (callsign == nullptr || strlen(callsign) < 3) {
        return;
    }
    for (const AirlinePaint &a : AIRLINE_PAINT) {
        if (strncmp(callsign, a.airline, 3) == 0) {
            look.body = a.body;
            look.belly = a.belly;
            look.tail = a.tail;
            look.engines = a.engines;
            look.windows = !a.freighter;
            return;
        }
    }
}

size_t buildModels() {
    models();
    size_t bytes = g_palette.capacity() * sizeof(Rgb);
    for (const Design &d : designs()) {
        for (const Mesh &m : d.lod) {
            bytes += m.corners.capacity() * sizeof(int16_t) + m.moves.capacity() +
                     m.faces.capacity() * sizeof(uint16_t) + m.normals.capacity() + m.colors.capacity() +
                     m.looks.capacity();
        }
        bytes += d.lights.capacity() * sizeof(PlaneLight) + sizeof(Design);
    }
    return bytes;
}

const char *typeOf(Family family) {
    static const char *const TYPES[] = {"A320", "K35R", "E3TF", "B789", "A343", "B744", "A388", "CRJ9", "GLF6",
                                        "AT76", "SF34", "C172", "P28A", "C17",  "C130", "A400", "C295", "B463",
                                        "EUFI", "F35",  "F16",  "HAWK", "A139", "EC45", "H47"};
    static_assert(sizeof TYPES / sizeof TYPES[0] == (size_t)Family::COUNT, "a type for each family");
    return (int)family < (int)Family::COUNT ? TYPES[(int)family] : "";
}

float sizeOf(const Look &look) {
    models();
    return designs()[std::min((int)look.family, (int)Family::COUNT - 1)].length * look.scale / 37.6f;
}

float noseWheelAhead(const Look &look) {
    models();
    return designs()[std::min((int)look.family, (int)Family::COUNT - 1)].noseGear * look.scale;
}

float noseAhead(const Look &look) {
    models();
    return designs()[std::min((int)look.family, (int)Family::COUNT - 1)].hi.x * look.scale;
}

// Below this over the ground, coming in, it flares; and the nose as its
// wheels touch: an airliner's, a light aircraft's, a fast jet's.
constexpr float FLARE_M = 15, TOUCH_PITCH = 5.5f * DEG, LIGHT_TOUCH_PITCH = 6.5f * DEG, FAST_TOUCH_PITCH = 10 * DEG;
// The steepest approach flown: London City's.
constexpr float STEEPEST_GLIDE = 5.5f * DEG;
// Its main wheels down, the nose held up this long, then lowered onto its
// wheel over this.
constexpr float NOSE_HELD_S = 1.5f, NOSE_LOWERED_S = 3;
// Taking off, the nose raised no quicker than a pilot rotates, to the
// attitude it lifts off at: an airliner's, a fast jet's.
constexpr float ROTATE_RATE = 3 * DEG, LIFT_PITCH = 8 * DEG, FAST_LIFT_PITCH = 11 * DEG;

namespace {
// The speed it rotates at, by its kind, over the ground as the feed has it -
// a little under the airspeed, into the wind: a light aircraft's 55 knots,
// a narrowbody's 135, a jumbo's 155. The runway simulation lifts every one
// off at 145, by when a narrowbody, rotating at three degrees a second, has
// its nose up.
float rotateKt(Family family) {
    switch (family) {
    case F::LIGHT_HIGH:
    case F::LIGHT_LOW: return 55;
    case F::TURBOPROP_LOW: return 95;
    case F::TURBOPROP:
    case F::TACTICAL_TWIN:
    case F::HERCULES:
    case F::ATLAS: return 105;
    case F::BUSINESS_JET:
    case F::JET_TRAINER:
    case F::QUAD_HIGH: return 115;
    case F::REGIONAL_JET:
    case F::AIRLIFTER: return 125;
    case F::FIGHTER_DELTA:
    case F::FIGHTER:
    case F::FIGHTER_SINGLE: return 140;
    case F::WIDEBODY:
    case F::WIDEBODY_4:
    case F::SUPERJUMBO: return 145;
    case F::JUMBO: return 155;
    default: return 135;
    }
}

// Coming down to land, by its descent - or, in the last few metres, by its not
// climbing: the flare itself all but stops the descent, and a report in it -
// 180ft a minute at 8m - would put the nose down onto the runway.
bool comingIn(float climbFpm, float heightM) { return climbFpm <= -250 || (heightM < 2 * FLARE_M && climbFpm <= 0); }
}  // namespace

float landingPitch(Family family, float speedKt, float climbFpm, float heightM, float pitch) {
    if (!comingIn(climbFpm, heightM) || speedKt > 220 || !(heightM < 900)) {
        return pitch;
    }
    bool light = family == F::LIGHT_HIGH || family == F::LIGHT_LOW;
    bool fast = family == F::FIGHTER_DELTA || family == F::FIGHTER || family == F::FIGHTER_SINGLE ||
                family == F::JET_TRAINER;
    // No lower for coming down more steeply than an approach is flown: where
    // the day's pressure isn't known, the feed's heights read a few hundred
    // feet high on a day of low pressure, and the runway simulation brings
    // that down in the last seconds - the nose would go down with it, onto
    // the runway.
    float glide = atanf(climbFpm * 0.00508f / (std::max(speedKt, 30.0f) * 0.5144f));
    pitch += std::max(0.0f, -STEEPEST_GLIDE - glide);
    // Slowed for the approach, its flaps out: an airliner's nose two or
    // three degrees up on a three-degree glide.
    pitch += light ? 1 * DEG : clamp01((220 - speedKt) / 60) * (fast ? 6 : 3) * DEG;
    float u = clamp01(1 - heightM / FLARE_M);
    float touch = light ? LIGHT_TOUCH_PITCH : fast ? FAST_TOUCH_PITCH : TOUCH_PITCH;
    return pitch + (std::max(pitch, touch) - pitch) * u * u * (3 - 2 * u);
}

float liftOffPitch(Family family) {
    bool fast = family == F::FIGHTER_DELTA || family == F::FIGHTER || family == F::FIGHTER_SINGLE;
    return fast ? FAST_LIFT_PITCH : LIFT_PITCH;
}

void Pilot::fly(float dt, Family family, float turnRate, float speedKt, float climbFpm, bool onGround,
                float heightM, bool departing) {
    if (dt <= 0) {
        return;
    }
    dt = std::min(dt, 0.5f);
    // How steeply its kind banks and how quickly it rolls: an airliner to 30
    // degrees at 7 a second; regional and business aircraft a little
    // quicker; light aircraft and helicopters further and faster; a fast jet
    // as it likes.
    float maxBank = 30 * DEG, maxRate = 7 * DEG, maxAccel = 6 * DEG;
    bool rotor = false;
    switch (family) {
    case F::LIGHT_HIGH:
    case F::LIGHT_LOW:
        maxBank = 45 * DEG, maxRate = 25 * DEG, maxAccel = 40 * DEG;
        break;
    case F::HELICOPTER:
    case F::HELICOPTER_LIGHT:
    case F::CHINOOK:
        maxBank = 35 * DEG, maxRate = 18 * DEG, maxAccel = 30 * DEG, rotor = true;
        break;
    case F::FIGHTER_DELTA:
    case F::FIGHTER:
    case F::FIGHTER_SINGLE:
    case F::JET_TRAINER:
        maxBank = 80 * DEG, maxRate = 120 * DEG, maxAccel = 300 * DEG;
        break;
    case F::REGIONAL_JET:
    case F::BUSINESS_JET:
    case F::TURBOPROP:
    case F::TURBOPROP_LOW:
    case F::TACTICAL_TWIN:
        maxBank = 35 * DEG, maxRate = 10 * DEG, maxAccel = 10 * DEG;
        break;
    default:
        break;
    }
    float v = speedKt * 0.5144f;
    // The bank a coordinated turn at this speed takes: tan(bank) = v x rate
    // / g. Turning left - the rate positive, as headings go - is banking
    // left, which is a roll the other way to a Pose's.
    float target = onGround || v < 15 ? 0 : -atanf(v * turnRate / 9.81f);
    target = std::max(-maxBank, std::min(maxBank, target));
    // Rolling towards it no faster than its kind can, the rate easing in and
    // out as a pilot's hands do.
    float want = std::max(-maxRate, std::min(maxRate, (target - roll) * 1.2f));
    float step = maxAccel * dt;
    rollRate_ += std::max(-step, std::min(step, want - rollRate_));
    roll += rollRate_ * dt;
    // The ailerons as far over as the roll it is making, back in the middle
    // once it is banked; and a touch of rudder with them.
    aileron = std::max(-1.0f, std::min(1.0f, rollRate_ / maxRate));
    rudder = 0.3f * aileron;
    // The nose along its climb and a few degrees above it, as a wing needs -
    // more in a turn, to hold its height. A helicopter's not along its path,
    // which it can climb or come down any way it likes, but down as it goes
    // faster and up as it slows - its flare, slowing to a hover or to set
    // down. The elevator moving it there, and held back in a turn.
    // Its speeding up as the estimate has it, a report's correction to its
    // speed - a step - taken as no more than ten knots a second.
    float speeding = std::max(-10.0f, std::min(10.0f, (speedKt - lastKt_) / dt));
    accel_ += (speeding - accel_) * std::min(1.0f, dt / 1.5f);
    lastKt_ = speedKt;
    float gamma = v > 5 && !rotor ? atanf(climbFpm * 0.00508f / v) : 0;
    float load = 1 / std::max(0.5f, cosf(roll));
    float flare = std::max(-6.0f, std::min(10.0f, -accel_ * 3)) * DEG;
    float alpha = onGround ? 0 : rotor ? -std::min(9.0f, speedKt / 15) * DEG + flare : 2.5f * DEG * load;
    pitchTarget_ = gamma + alpha;
    if (!onGround && !rotor) {
        pitchTarget_ = landingPitch(family, speedKt, climbFpm, heightM, pitchTarget_);
    }
    // Taking off: rotated at its kind's speed for it, the nose raised to the
    // attitude it lifts off at while its main wheels still roll - it stayed
    // level till the feed had it off the ground - and held up there at
    // least as it climbs away, till its climb as reported catches up: a
    // climb's first reports lag it, and the nose dipped as it left.
    bool rotating = departing && !rotor && onGround && speedKt >= rotateKt(family) && accel_ > -1;
    if (rotating || (departing && !rotor && !onGround)) {
        pitchTarget_ = std::max(pitchTarget_, liftOffPitch(family));
    }
    float was = pitch;
    // Landed: its main wheels on, the nose held up, then lowered onto its
    // wheel - not dropped with the rest of its pitch.
    if (onGround && !wasGround_ && !rotor && speedKt > 40) {
        touchPitch_ = std::max(0.0f, pitch);
        sinceTouch_ = 0;
    }
    wasGround_ = onGround;
    if (!onGround) {
        sinceTouch_ = -1;
    }
    if (sinceTouch_ >= 0) {
        sinceTouch_ += dt;
        float u = clamp01((sinceTouch_ - NOSE_HELD_S) / NOSE_LOWERED_S);
        pitch = touchPitch_ * (1 - u * u * (3 - 2 * u));
        if (u >= 1) {
            sinceTouch_ = -1;
        }
    } else {
        // Quicker in the flare: it has only a few seconds.
        bool flaring = !onGround && heightM < FLARE_M && comingIn(climbFpm, heightM);
        float eased = pitch + (pitchTarget_ - pitch) * (1 - expf(-dt / (flaring ? 0.6f : 1.2f)));
        pitch = rotating ? std::min(eased, pitch + ROTATE_RATE * dt) : eased;
    }
    float pitchRate = (pitch - was) / dt;
    elevator = std::max(-1.0f, std::min(1.0f, pitchRate / (4 * DEG) + (load - 1) * 1.5f));
}

namespace {
// A made-up mix of traffic for the busy airport: narrowbodies, regional jets
// and turboprops on the stands - sized for them - and the widebodies and
// jumbos among those moving, each in one of a dozen airlines' colours, the
// same dozen the preview's labels give them.
const char *const MIX_AIRLINES[] = {"BAW", "EZY", "RYR", "DLH", "AFR", "KLM", "UAE", "QTR", "AAL", "UAL", "EIN", "VIR"};
const char *const NARROW[] = {"A320", "A20N", "A319", "A321", "B738", "B38M", "E190", "A21N", "BCS3", "AT76", "CRJ9", "B737"};
const char *const ANY[] = {"A320", "B77W", "A21N", "B789", "A388", "A359", "B738", "B744", "A333", "E190", "A343", "DH8D"};

Look mixLook(int i, bool narrow) {
    uint32_t mixed = (uint32_t)i * 2654435761u;
    const char *type = narrow ? NARROW[mixed % 12] : ANY[mixed % 12];
    Look look = lookFor(type, 0x1B2A55);
    paintAirline(look, MIX_AIRLINES[i % 12]);
    return look;
}
}  // namespace

Config configFor(float heightM, float speedKt, bool onGround, bool departing) {
    Config c;
    if (departing) {
        // Flaps set for take-off, the gear coming up as it climbs away.
        c.flaps = onGround || heightM < 900;
        c.gear = onGround || heightM < 15;
    } else if (onGround) {
        // Spoilers up and flaps out from the touchdown until the roll-out
        // slows - the spoilers coming down as it slows from 60kt to 40; cleaned
        // up to taxi in.
        float k = clamp01((speedKt - 40) / 20);
        c.spoilerDeploy = k * k * (3 - 2 * k);
        c.spoilers = c.spoilerDeploy > 0.01f;
        c.flaps = speedKt > 40;
    } else {
        // Coming in: the gear down below 2,000ft or so and the flaps out below
        // 4,000 - slow enough for them, as an airliner on its approach is and
        // a fighter running in low at 280kt isn't.
        c.gear = heightM < 600 && speedKt < 210;
        c.flaps = heightM < 1200 && speedKt < 230;
    }
    return c;
}

// An aircraft parked at a stand: its nose wheel on the stop - or its middle
// on the middle of one drawn as its outline - and further back if that would
// bring its nose within 3m of the building ahead.
Pose parkedOn(const StandStop &s, const Look &look) {
    float back = std::max(s.middle ? 0.0f : noseWheelAhead(look), noseAhead(look) - (s.room - 3));
    return {s.at - Vec3{cosf(s.heading), sinf(s.heading), 0} * back, s.heading, 0};
}

int parkedAtStands(Aircraft *out, int max) {
    const Models &m = models();
    int n = std::min(max, (int)m.stands.size());
    for (int i = 0; i < n; i++) {
        Look look = mixLook(i, true);
        out[i] = {parkedOn(m.stands[i], look), look, {}};
    }
    return n;
}

int busyTraffic(float t, Aircraft *out, int max) {
    const Models &m = models();
    int n = 0;
    auto add = [&](const Pose &p, Config config = Config{}) {
        if (n < max) {
            out[n] = {p, mixLook(n, n < (int)m.stands.size()), config};
            n++;
        }
    };
    for (const StandStop &s : m.stands) {
        if (n < max) {
            Look look = mixLook(n, true);
            out[n] = {parkedOn(s, look), look, {}};
            n++;
        }
    }
    // Three more arrivals behind the one landing, 75 seconds apart - a
    // little under three miles, at 136 knots.
    for (int k = 1; k <= 3; k++) {
        Pose p = onRunway(landingEnd(), localLanding(path(), t - 75 * k));
        add(p, configFor(p.at.z, 136, p.at.z <= 0, false));
    }
    // A departure every 90 seconds, each lined up for 25 before it goes.
    if (g_departureEnd >= 0) {
        const RunwayEnd &dep = m.ends[g_departureEnd];
        for (int k = 0; k < 2; k++) {
            Pose p = localTakeoff(fmodf(t + 45 * k + 1000, 90) - 25);
            p.at.y = 0;
            Vec3 at = dep.origin + dep.along * p.at.x + Vec3{0, 0, p.at.z};
            add({at, atan2f(dep.along.y, dep.along.x), p.pitch}, configFor(p.at.z, 150, p.at.z <= 0, true));
        }
    }
    // Two taxiing each way along each runway's taxiway, at 20 knots.
    for (size_t e = 0; e + 1 < m.ends.size(); e += 2) {
        const RunwayEnd &a = m.ends[e];
        float r = a.exitRight ? TAXIWAY_OFFSET : -TAXIWAY_OFFSET, len = a.length + 120;
        float heading = atan2f(a.along.y, a.along.x);
        for (int k = 0; k < 2; k++) {
            float s = fmodf(t * 10 + k * len / 2 + e * 300, len);
            add({a.at(-60 + s, r), heading, ROLLING_PITCH});
            add({a.at(len - 60 - s, r + (a.exitRight ? 0.0f : 0.0f)), heading + PI, ROLLING_PITCH});
        }
    }
    return n;
}

// --- cameras ---------------------------------------------------------------------

namespace {
// The middle of the aircraft, which a camera looks at.
Vec3 middleOf(const Pose &pose, float size = 1) { return pose.at + Vec3{0, 0, 3.5f * size}; }

float ease(float u) {
    u = clamp01(u);
    return u * u * (3 - 2 * u);
}
}  // namespace

Camera orbitCamera(const Pose &pose, float azimuthDeg, float elevationDeg, float distanceM, float fovDeg) {
    float bearing = pose.heading + PI + azimuthDeg * DEG, el = elevationDeg * DEG;
    Vec3 middle = middleOf(pose);
    Vec3 eye = middle + Vec3{cosf(el) * cosf(bearing), cosf(el) * sinf(bearing), sinf(el)} * distanceM;
    eye.z = std::max(eye.z, 1.5f);
    return {eye, middle, fovDeg};
}

Camera fixedCamera(const Pose &pose, Vec3 eye, float fill, float size) {
    Vec3 middle = middleOf(pose, size);
    float distance = length(middle - eye);
    float fov = 2 * atanf(40 * size / fill / 2 / distance) / DEG;
    return {eye, middle, std::max(1.5f, std::min(65.0f, fov))};
}

Camera cinemaCamera(float t, const char **shot, float size) {
    const Path &p = path();
    const RunwayEnd &end = landingEnd();
    Pose pose = landingPose(t);
    auto named = [&](const char *name) {
        if (shot) {
            *shot = name;
        }
    };
    float chaseFrom = p.tFlare - 9, touchFrom = p.tFlare - 1, rollFrom = p.tTouch + 3.5f;
    if (t < chaseFrom) {
        // The long lens from the fence by the threshold, up the approach: the
        // aircraft hanging in the haze, pushed in on slowly as it comes.
        named("long lens");
        return fixedCamera(pose, onRunway(end, Vec3{-30, -60, 1.7f}), 0.26f + 0.14f * ease(t / chaseFrom), size);
    }
    if (t < touchFrom) {
        // Low behind it on short final, swinging from one quarter to the other.
        named("chase");
        float u = ease((t - chaseFrom) / (touchFrom - chaseFrom));
        return orbitCamera(pose, -40 + 75 * u, 4 + 4 * u, 58 * size, 42);
    }
    if (t < rollFrom) {
        // Side-on and low, just past where the wheels will touch: the flare,
        // the smoke, the nose coming down.
        named("touchdown");
        return fixedCamera(pose, onRunway(end, Vec3{p.touchX + 70, -62, 1.2f}), 0.5f, size);
    }
    if (t < p.tRolled) {
        // Behind it down the runway, craning up and away as it slows.
        named("rollout");
        float u = ease((t - rollFrom) / (p.tRolled - rollFrom));
        return orbitCamera(pose, 30 - 50 * u, 6 + 18 * u, (70 + 40 * u) * size, 46);
    }
    if (t < p.tTurned) {
        // From the tower, just outside the glass on the runway's side.
        named("tower");
        const Models &m = models();
        Vec3 towards = normalize(onRunway(end, Vec3{p.exitX, 0, 0}) - m.tower);
        return fixedCamera(pose, m.tower + Vec3{towards.x * 8, towards.y * 8, m.towerEyeZ + m.towerLift}, 0.32f, size);
    }
    // Low on the taxiway ahead of it as it turns in towards the stands.
    named("taxi in");
    return fixedCamera(pose, onRunway(end, Vec3{p.exitX + 26, TAXIWAY_OFFSET + 56, 2.2f}), 0.7f, size);
}

// --- the director ------------------------------------------------------------------

Pose departurePose(float t) {
    if (g_departureEnd < 0) {
        return {};
    }
    const RunwayEnd &dep = models().ends[g_departureEnd];
    Pose p = localTakeoff(t);
    return {dep.origin + dep.along * p.at.x + Vec3{0, 0, p.at.z}, atan2f(dep.along.y, dep.along.x), p.pitch};
}

namespace {

enum class Shot { CHASE, LONG_LENS, TOUCHDOWN, REPLAY, ROLLOUT, TOWER, TAXI_IN, LINED_UP, ROLL, CLIMB, CLIMB_AWAY };
const char *const SHOT_NAMES[] = {"chase",  "long lens", "touchdown", "replay",    "rollout",    "tower",
                                  "taxi in", "lined up",  "rolling",   "climb out", "climbing away"};

// Where the replay picks up from, how long it lasts on the screen, and how
// much slower than life.
constexpr float REPLAY_FROM = 3.5f, REPLAY_SLOW = 0.5f, REPLAY_LIVE_S = 4.5f;
// The least a shot is held, unless something happens.
constexpr float HOLD_S = 3;

}  // namespace

struct Director::State {
    Shot shot = Shot::CHASE;
    float shotFrom = 0;
    // Where the camera is, for the shots that settle it as they start: held
    // there, or - on the ground - carried along at `groundOffset` from it.
    Vec3 groundEye{}, groundOffset{};
    bool eyeSet = false;
    // How far the shot's camera is raised over a building in its way - see
    // raised() - and when the last frame was.
    float raise = 0, lastSeconds = -1;
    bool wasOnGround = false, haveTouchdown = false;
    float touchdownAt = -1, liftoffAt = -1;
    // The last dozen seconds of where it was, for a replay.
    struct Then {
        float seconds;
        Pose pose;
    };
    Vector<Then> history;
};

Director::Director() : s(new State) {}
Director::~Director() { delete s; }

Camera Director::next(const Flight &flight, float seconds, const char **shot, Pose *shown, bool *replaying) {
    State &st = *s;
    const Pose &pose = flight.aircraft.pose;
    float size = sizeOf(flight.aircraft.look);
    if (replaying) {
        *replaying = false;
    }
    st.history.push_back({seconds, pose});
    while (!st.history.empty() && st.history.front().seconds < seconds - 12) {
        st.history.erase(st.history.begin());
    }
    // The moments that make a cut whatever the hold: wheels on, wheels off.
    if (flight.onGround && !st.wasOnGround && !flight.departing && st.history.size() > 1) {
        st.touchdownAt = seconds;
        st.haveTouchdown = true;
    }
    if (!flight.onGround && st.wasOnGround && flight.departing) {
        st.liftoffAt = seconds;
    }
    st.wasOnGround = flight.onGround;

    // Where it is on its runway, if it has one.
    const Models &m = models();
    bool onRunwayLine = flight.runway >= 0 && flight.runway * 2 + flight.end < (int)m.ends.size();
    const RunwayEnd *end = onRunwayLine ? &m.ends[flight.runway * 2 + flight.end] : nullptr;
    float along = 0, height = pose.at.z;
    if (end) {
        along = dot(pose.at - end->origin, end->along);
    }

    // The shot what it is doing calls for.
    Shot want = Shot::CHASE;
    float sinceTouchdown = st.haveTouchdown ? seconds - st.touchdownAt : 1e9f;
    if (!end) {
        want = flight.onGround ? Shot::TAXI_IN : Shot::CHASE;
    } else if (!flight.departing) {
        if (!flight.onGround) {
            want = along < -2000 ? Shot::CHASE : height > 25 ? Shot::LONG_LENS : Shot::TOUCHDOWN;
        } else if (sinceTouchdown < REPLAY_FROM) {
            want = Shot::TOUCHDOWN;
        } else if (sinceTouchdown < REPLAY_FROM + REPLAY_LIVE_S) {
            want = Shot::REPLAY;
        } else if (flight.speedKt > 35) {
            want = Shot::ROLLOUT;
        } else {
            want = st.shot == Shot::TOWER && seconds - st.shotFrom > 12 ? Shot::TAXI_IN
                   : st.shot == Shot::TAXI_IN                         ? Shot::TAXI_IN
                                                                      : Shot::TOWER;
        }
    } else {
        float sinceLiftoff = st.liftoffAt >= 0 ? seconds - st.liftoffAt : -1;
        if (flight.onGround) {
            want = flight.speedKt < 30 ? Shot::LINED_UP : Shot::ROLL;
        } else if (sinceLiftoff >= 0 && sinceLiftoff < 3) {
            want = Shot::ROLL;
        } else {
            want = height < 350 ? Shot::CLIMB : Shot::CLIMB_AWAY;
        }
    }
    // Cut to it - at once for a touchdown, a replay or a lift-off, or from a
    // shot placed by the runway once there is no runway, the movement over;
    // otherwise only once the shot showing has had its few seconds.
    bool byRunway = st.shot != Shot::CHASE && st.shot != Shot::TAXI_IN && st.shot != Shot::CLIMB;
    bool event = want == Shot::TOUCHDOWN || want == Shot::REPLAY || st.shot == Shot::REPLAY ||
                 (st.shot == Shot::ROLL && want == Shot::CLIMB) || (byRunway && end == nullptr);
    if (want != st.shot && (event || seconds - st.shotFrom >= HOLD_S)) {
        st.shot = want;
        st.shotFrom = seconds;
        st.eyeSet = false;
        st.raise = 0;
    }
    if (shot) {
        *shot = SHOT_NAMES[(int)st.shot];
    }
    float in = seconds - st.shotFrom;  // how long the shot has been up

    // A point on the runway's own terms - along it, out to its right - with
    // the taxiway's side kept clear.
    auto at = [&](float a, float right, float z) {
        float r = end->exitRight ? -right : right;
        return end->origin + end->along * a + end->right * r + Vec3{0, 0, z};
    };
    // A camera on the ground raised - five metres at a time, forty at most,
    // quickly, and down again slowly once clear - while a building stands
    // between it and `target`: by a runway, or carried beside it as it taxied
    // in among the terminals, it was watched through their walls.
    float dt = st.lastSeconds >= 0 ? std::max(0.0f, seconds - st.lastSeconds) : 0;
    st.lastSeconds = seconds;
    auto raised = [&](Vec3 eye, Vec3 target) {
        float need = 0;
        while (need < 40 && !seenPast(eye + Vec3{0, 0, need}, target)) {
            need += 5;
        }
        st.raise += (need - st.raise) * std::min(1.0f, dt / (need > st.raise ? 0.5f : 3.0f));
        return eye + Vec3{0, 0, st.raise};
    };
    // On the ground, carried along with it and never turned: `distance` off
    // it, `azimuthDeg` round from behind it - to its right - and
    // `elevationDeg` up, as it points when the shot starts, and kept there
    // in the airfield's terms. What is behind it slides by as it goes,
    // rather than the picture swinging round after it; turning, it is seen
    // to turn; stopped, nothing moves.
    auto carried = [&](float azimuthDeg, float elevationDeg, float distance, float fill) {
        Vec3 middle = middleOf(pose, size);
        if (!st.eyeSet) {
            float bearing = pose.heading + PI + azimuthDeg * DEG, el = elevationDeg * DEG;
            st.groundOffset = Vec3{cosf(el) * cosf(bearing), cosf(el) * sinf(bearing), sinf(el)} * distance;
            st.eyeSet = true;
        }
        Vec3 eye = middle + st.groundOffset;
        eye.z = std::max(eye.z, 1.5f);
        eye = raised(eye, middle);
        float fov = 2 * atanf(40 * size / fill / 2 / length(eye - middle)) / DEG;
        return Camera{eye, middle, std::max(1.5f, std::min(65.0f, fov))};
    };
    switch (st.shot) {
    case Shot::LONG_LENS:
        return fixedCamera(pose, raised(at(-30, 60, 1.7f), middleOf(pose, size)),
                           0.26f + 0.14f * clamp01((along + 2000) / 2000), size);
    case Shot::TOUCHDOWN:
        return fixedCamera(pose, raised(at(590, 62, 1.2f), middleOf(pose, size)), 0.5f, size);
    case Shot::REPLAY: {
        // The flare again, half speed, from low on the far side and behind.
        float then = st.touchdownAt - 2.5f + in * REPLAY_SLOW;
        Pose was = pose;
        for (size_t i = 0; i + 1 < st.history.size(); i++) {
            const State::Then &a = st.history[i], &b = st.history[i + 1];
            if (a.seconds <= then && b.seconds >= then) {
                float u = b.seconds > a.seconds ? (then - a.seconds) / (b.seconds - a.seconds) : 0;
                was = {lerp(a.pose.at, b.pose.at, u), a.pose.heading + (b.pose.heading - a.pose.heading) * u,
                       a.pose.pitch + (b.pose.pitch - a.pose.pitch) * u};
                break;
            }
        }
        if (shown) {
            *shown = was;
        }
        if (replaying) {
            *replaying = true;
        }
        return fixedCamera(was, raised(at(420, -48, 0.9f), middleOf(was, size)), 0.62f, size);
    }
    case Shot::ROLLOUT:
        // Alongside it, low, on the side away from the taxiways: slowing,
        // with the runway's lights streaming by.
        return carried(end->exitRight ? -75.0f : 75.0f, 5, 95 * size, 0.42f);
    case Shot::TOWER: {
        // From the tower, held where it was looking as the shot started -
        // ahead of it, by where it is heading - and wide enough to see it
        // taxi there: the airfield still, and the aircraft moving across it.
        if (!st.eyeSet) {
            Vec3 towards = normalize(at(end->length / 2, 0, 0) - m.tower);
            st.groundEye = m.tower + Vec3{towards.x * 8, towards.y * 8, m.towerEyeZ + m.towerLift};
            // Higher yet if a building is in the way of it, as it is now.
            for (int k = 0; k < 8 && !seenPast(st.groundEye, middleOf(pose, size), true); k++) {
                st.groundEye.z += 5;
            }
            Vec3 ahead = {cosf(pose.heading), sinf(pose.heading), 0};
            st.groundOffset = middleOf(pose, size) + ahead * (flight.speedKt * 0.514f * 6) - st.groundEye;
            st.eyeSet = true;
        }
        float distance = std::max(1.0f, length(st.groundOffset));
        float across = 40 * size / 0.3f + flight.speedKt * 0.514f * 12 * 1.3f;
        float fov = 2 * atanf(across / 2 / distance) / DEG;
        return {st.groundEye, st.groundEye + st.groundOffset, std::max(1.5f, std::min(65.0f, fov))};
    }
    case Shot::TAXI_IN:
        // Off its quarter and above, carried along with it.
        return carried(35, 8, 100 * size, 0.45f);
    case Shot::LINED_UP:
        // Ahead of it, low, a little to one side: what it is about to roll
        // towards.
        return carried(end->exitRight ? 160.0f : -160.0f, 4, 120 * size, 0.4f);
    case Shot::ROLL:
        // Side-on where it will rotate, watching it come.
        return fixedCamera(pose, raised(at(1450, 70, 1.2f), middleOf(pose, size)), 0.45f, size);
    case Shot::CLIMB:
        return orbitCamera(pose, 12, -6, 75 * size, 42);
    case Shot::CLIMB_AWAY:
        return fixedCamera(pose, raised(at(end->length + 150, 80, 1.7f), middleOf(pose, size)), 0.3f, size);
    default:
        // Behind and a little above, off one quarter.
        return orbitCamera(pose, 20, 9, 85 * size, 44);
    }
}

}  // namespace scene3d
