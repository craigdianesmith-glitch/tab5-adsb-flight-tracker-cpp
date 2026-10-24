#pragma once

#include <stddef.h>
#include <stdint.h>

// The ground pack - the card's /overhead/ground.bin, made by
// tools/gen_ground.py: what covers the land, the hills' light and shade, and
// their heights, for the 3D view to stamp onto the ground it draws and to
// draw its skyline from. From ESA WorldCover (land cover) and Copernicus
// DEM GLO-90 (heights) - see that tool for what each is and its credit.
//
// The pack is tiles of 0.2 degrees. A tile's cells are 2.5 seconds of
// latitude - some 75m - and 2.5k of longitude - k 1, 1.5, 2 or 3 by how far
// from the equator, so a cell stays near square - 288 rows of 288/k, north
// first; a cell a byte, its cover in the top four bits and its shade in the
// bottom four. Each tile has levels 0 to 4, each a quarter of the one
// before, and heights. Level 4 of every tile, and its heights, are held
// from the moment the pack is opened - a megabyte or so for a country, the
// far country and the skyline; levels 1 to 3 and the finer heights are read
// in, a tile at a time, round wherever the view asks; and level 0, the
// finest, for the tiles round the eye only, where its cells are big enough
// to be seen. A pack of version 1 has no level 0. Finer still, levels -1
// and -2 - 37.5m and 18.75m - are worked out here from level 0, for the few
// tiles nearest the eye, where even level 0's cells are many pixels across.
namespace groundpack {

constexpr int ROWS0 = 288;  // a tile's rows at level 0
constexpr int LEVELS = 5;   // 0 to 4, the pack's

// Where a tile's heights every 15 seconds start in its mid block: after its
// levels 1, 2 and 3 - 48 rows of cols0/6 of them, int16 metres above the sea.
inline size_t midHeightsAt(int cols0) {
    int r1 = ROWS0 / 2, c1 = cols0 / 2;
    return (size_t)r1 * c1 + (size_t)(r1 / 2) * (c1 / 2) + (size_t)(r1 / 4) * (c1 / 4);
}
constexpr int FINEST = -2;  // the finest worked out here
constexpr int TILES_PER_DEG = 5;

// A tile's columns at level 0, by its row of tiles from the north pole -
// 288, or fewer the further from the equator; nought past 75 degrees.
inline int cols0At(int northRow) {
    float a = 90.0f - (northRow + 0.5f) * (1.0f / TILES_PER_DEG);
    a = a < 0 ? -a : a;
    return a < 50 ? ROWS0 : a < 60 ? ROWS0 * 2 / 3 : a < 70 ? ROWS0 / 2 : a < 75 ? ROWS0 / 3 : 0;
}

// The covers, as the pack has them: WorldCover's, and four mixes of them -
// see tools/gen_ground.py.
enum Cover : uint8_t {
    NONE = 0, TREES, SHRUBS, GRASS, CROPS, BUILT, BARE, SNOW, WATER, MARSH, MANGROVES, MOSS,
    SUBURB, TOWN, MOOR, ROCKY, COVERS
};

// The tiles the 3D view reads from, a window of them round where it asked:
// each tile's level 4 and coarse heights, and - near - its levels 1 to 3
// and its finer heights, and - nearer - its level 0, and nearest, levels -1
// and -2; or nullptr where there is no tile or it isn't in yet. Rows of
// tiles north first, as the cells are; tile (0, 0) at north edge `north`,
// west edge `west`, in tiles from the north pole and from 180 west. Levels
// 0 and finer are in parts of 144 rows by cols0/2, as many as each needs -
// 2 by 2, 4 by 4, 8 by 8 - north-west first, a row of them at a time: the
// 3D view's walk along them counts places in a part in 16 bits.
struct Window {
    int north = 0, west = 0, rows = 0, cols = 0;
    const uint8_t **coarse = nullptr;  // rows*cols: level 4, 18 by cols0/16, then its heights (int16)
    const uint8_t **mid = nullptr;     // rows*cols: levels 1, 2, 3, then heights every 15 seconds (int16)
    const uint8_t **detail = nullptr;  // rows*cols: level 0, in parts
    const uint8_t **finer[2] = {};     // rows*cols: levels -1 and -2, in parts
};

// Opens the card's pack and reads in every tile's level 3 - false, and
// nothing held, where there isn't one or it can't be read. Once, from the
// view as it opens; close() gives the memory back.
bool open();
void close();
bool isOpen();

// The tiles round (lat, lon) the view will want: a window `fineKm` round
// read in at levels 1 to 3, in the background, the tiles round it at level
// 0, and the window of every tile's level 4 round it. Levels -1 and -2 too,
// for the eye `upM` over the ground or lower than that: -2 within 4,000m,
// -1 within 8,000 - higher, even straight down they come to under four
// pixels a cell, never drawn - and those worked out given back on climbing
// past. Called each frame; only a move of a tile or more changes anything.
// False until the pack is open.
bool want(double lat, double lon, float fineKm, float upM = 0);

// The window as it stands, for this frame - valid until the next want().
const Window &window();

// The land's heights about a place, for the 3D view's terrain: one regular
// grid over a block of tiles - rows of them north first - 48 rows by 32
// columns of heights a tile, the pack's own every 15 seconds of latitude and
// 22.5 of longitude between 50 and 60 north and spread to the same
// elsewhere, its coarse heights spread over a tile whose mid block isn't
// in; int16 metres above the sea, nought off the pack. A sample at the
// middle of its cell, as the pack's are. And the highest of each 8 by 8 of
// them with the row and the column after - all a sample between them can
// reach - for a march over them to skip what can't be seen. One grid, not
// the tiles: a sample is four loads from it, where from the tiles it was a
// tile found, its kind of heights picked and its edges kept to.
struct TerrainGrid {
    static constexpr int TILE_ROWS = 48, TILE_COLS = 32, BLOCK = 8;
    int north = 0, west = 0;  // its first tile, from the north pole and 180 west
    int tilesDown = 0, tilesAcross = 0;
    int rows = 0, cols = 0;  // tilesDown * 48, tilesAcross * 32
    int blockRows = 0, blockCols = 0;
    int16_t *heights = nullptr;   // rows * cols
    int16_t *cellMax = nullptr;   // rows * cols: each cell's highest corner, from (x, y) to (x + 1, y + 1)
    int16_t *blockMax = nullptr;  // blockRows * blockCols
    int16_t *wideMax = nullptr;   // blockRows * blockCols: the highest of each block and those round it
    int16_t highest = 0;
};
// Fills `grid` - its corner and size already set, and its memory there -
// from the window's tiles, its block maxima too.
void fillTerrainGrid(const Window &window, TerrainGrid &grid);
// The grid the 3D view's terrain is drawn from, about where it last asked
// (see want()): nullptr while there is none.
const TerrainGrid *terrainGrid();

// The height of the ground at (lat, lon), metres, from the tiles held - or
// 0 where there are none.
float heightAt(double lat, double lon);

// For the log and the bench: tiles with their levels 1 to 3 in, with their
// level 0, and with levels -1 and -2 worked out; reads made of them so far,
// and how long the levels finer than the pack's have taken to work out.
void stats(int &midHeld, int &detailHeld, int &finerHeld, int &reads, uint32_t &readUs, uint32_t &finerUs);

// Level `level`, -1 or -2, of a tile, worked out from its level 0, `detail`,
// as the window has them - the tile `northRow` and `westCol` tiles from the
// north pole and 180 west: the cover's edges wandering as a field's or a
// wood's do, its light and shade smooth between level 0's. What the ground task does for the tiles nearest the eye,
// and tools/finer_preview draws. Its memory the caller's to free (with
// free() off the device), or nullptr for none.
uint8_t *finerLevel(const uint8_t *detail, int cols0, int northRow, int westCol, int level);

// The horizon round a point, for the 3D view's skyline: along each of 720
// bearings - half a degree each, from north round by east - how high the
// land stands above level from the point, at its highest, as the tangent
// of the angle up to it, the earth's curve and the air's bending of the
// light both taken off, and how far off that is, metres: the skyline, the
// highest anywhere from 5km out to 100km. (Three ranges, the hills behind one
// another a range at a time, gave each bearing its own pick of near and far
// ridges, and the ranges took turns bearing by bearing - stepped blocks of
// two hazes along the hills.) From the tiles' heights, their finest held.
struct Horizon {
    static constexpr int BEARINGS = 720, RANGES = 1;
    double lat = 0, lon = 0;
    float heightM = 0;  // the point's, above the sea
    // How many of the bearings, as a share, have land within the 5km
    // the skyline leaves out standing above the point's level: down among
    // the hills, none drawn - see scene3d's skyline().
    float nearAbove = 0;
    float tanUp[RANGES][BEARINGS];
    float distance[RANGES][BEARINGS];
};

// The horizon wanted round (lat, lon), `heightM` above the sea - each frame,
// as the eye moves. Worked out afresh, in the background, only once it is
// 300m from where the last was, or 30m higher or lower.
void wantHorizon(double lat, double lon, float heightM);

// The horizon last worked out, nullptr until there is one. Valid till the
// one after next is worked out - which is only begun once the eye has gone
// 300m further, long after any frame drawing from this one is done.
const Horizon *horizon();

}  // namespace groundpack
