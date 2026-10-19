#pragma once

#include <stddef.h>
#include <stdint.h>

#include <vector>

#ifdef ESP_PLATFORM
#include "psram_alloc.h"
#endif

// An airport's real layout - its taxiways and taxilanes, holding points,
// stands, aprons, buildings, tower and windsocks - from OpenStreetMap, as
// tools/gen_airfields.py packs them into airfields.bin, which goes in the
// overhead folder on the SD card. In metres east and north of the middle of
// the airport's open runways, as the 3D view's buildAirfield() takes it from
// the runway table: the frame the 3D view draws the airfield in. The layouts
// are (c) OpenStreetMap contributors, under the Open Database License.
namespace airfield {

#ifdef ESP_PLATFORM
template <typename T>
using List = std::vector<T, PsramAllocator<T>>;
#else
template <typename T>
using List = std::vector<T>;
#endif

struct Point {
    float x, y;
};

enum class WayKind : uint8_t { TAXIWAY = 1, TAXILANE = 2, RUNWAY = 3 };

// A taxiway, taxilane or runway between two junctions - the network the
// aircraft move on - by its centreline: the junction it starts at, the points
// between, and the junction it ends at.
struct Way {
    WayKind kind;
    float width;  // metres
    char ref[8];  // "A1", "" where it has none
    uint16_t from, to;
    List<Point> between;
};

enum class HoldKind : uint8_t { RUNWAY = 1, OTHER = 2 };

// A holding point: where on which way an aircraft waits.
struct Hold {
    uint16_t way;
    Point at;
    HoldKind kind;
    char ref[8];
};

// A stand: where an aircraft parks, and the line painted in to it - from the
// taxilane to where it stops - if it has one.
struct Stand {
    Point at;
    char ref[8];
    List<Point> lead;
};

// An area - an apron, a taxiway drawn as one, a building's roof - as its
// corners and the triangles that fill it, three corners each.
struct Area {
    uint8_t kind;  // a surface's: 1 apron, 2 taxiway; a building's: 1 terminal, 2 hangar, 3 tower, 4 other
    float height;  // a building's, metres; 0 for a surface
    List<Point> corners;
    List<uint16_t> triangles;
};

struct Layout {
    double lat0 = 0, lon0 = 0;  // where (0, 0) is
    List<Point> junctions;
    List<Way> ways;
    List<Hold> holds;
    List<Stand> stands;
    List<Area> surfaces, buildings;
    bool hasTower = false;
    Point tower{};
    float towerHeight = 0;
    List<Point> windsocks;

    void clear();
    bool empty() const { return ways.empty() && surfaces.empty() && buildings.empty(); }
    // Every point moved by (dx, dy): to a frame whose (0, 0) is elsewhere.
    void moveBy(float dx, float dy);
    // How far from `from`, going `heading` - radians anticlockwise from east
    // - before a building: `reach` if none is nearer, 0 if `from` is in one.
    // For an aircraft parked on a stand to keep its nose out of the terminal.
    float roomAhead(Point from, float heading, float reach) const;
};

// One airport's block of the file, unpacked into `out`: false, and `out` left
// as far as it got, if the block is cut short or not what it should be.
bool parse(const uint8_t *block, size_t size, Layout &out);

#ifdef ESP_PLATFORM
// Why there was no layout from the card - for the 3D view, drawing the
// airport's runways alone, to say so.
enum class Missing : uint8_t {
    NOTHING,        // it loaded
    NO_CARD,        // no SD card in, or none that mounts
    NO_PACK,        // no airfields.bin in the card's overhead folder
    OTHER_VERSION,  // one this firmware can't read: another version's, or not one at all
    NOT_IN_PACK,    // nothing in it for that airport
    DAMAGED,        // its block of it cut short or wrong
};

// The layout of the airport `code` - as the app's tables have it: "GLA",
// "EGXE" - from the card: false, `out` empty and `why` saying why, if there's
// no card, no file, or nothing in it for that airport.
bool load(const char *code, Layout &out, Missing *why = nullptr);

// What airfield pack the card has: how many airfields and how big - or
// false, and why there is none to read.
bool packInfo(uint16_t &airfields, uint32_t &bytes, Missing &why);

#ifdef BENCH3D
// The pack taken as not there, or there again - for the basic view to be
// seen with the card as it is.
void benchHidePack(bool hide);
#endif

// Where the airport `code`'s control tower is, from the card - its tower
// alone, the rest of its layout passed over unread: for the airport zoom to
// mark it. False if there's no card, no file, nothing in it for that airport,
// or no tower in OpenStreetMap there.
bool towerOf(const char *code, double &lat, double &lon);
#endif

}  // namespace airfield
