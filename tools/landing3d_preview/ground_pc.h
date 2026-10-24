// The ground pack, ground.bin, read on the PC into a groundpack::Window round
// a place, as the Tab5's ground task fills the window it keeps - every tile's
// level 4 and coarse heights, its levels 1 to 3 and finer heights, its level
// 0, and levels -1 and -2 worked out for the tiles nearest - for the preview
// to draw the land as the 3D view does. All of it at once: the device reads
// them in the background as it is wanted.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zlib.h>

#include <memory>
#include <vector>

#include "ground_pack.h"

struct PcGround {
    groundpack::Window window;
    std::vector<const uint8_t *> coarse, mid, detail, finer1, finer2;
    std::vector<std::unique_ptr<uint8_t[]>> keep;
    std::vector<uint8_t *> finers;  // from finerLevel(), malloc'd
    // The terrain's heights over all of it, as the device's ground task
    // keeps a grid of them about the eye.
    groundpack::TerrainGrid grid;
    std::vector<int16_t> gridHeights, gridCellMax, gridMax, gridWideMax;

    ~PcGround() {
        for (uint8_t *p : finers) {
            free(p);
        }
    }

    static uint32_t u32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

    static size_t midCells(int c0) {
        int r1 = groundpack::ROWS0 / 2, c1 = c0 / 2;
        return (size_t)r1 * c1 + (r1 / 2) * (c1 / 2) + (r1 / 4) * (c1 / 4);
    }

    static bool inflated(const uint8_t *in, uint32_t size, uint8_t *out, size_t n) {
        z_stream z = {};
        if (inflateInit2(&z, -15) != Z_OK) {
            return false;
        }
        z.next_in = (Bytef *)in;
        z.avail_in = size;
        z.next_out = out;
        z.avail_out = (uInt)n;
        int r = inflate(&z, Z_FINISH);
        inflateEnd(&z);
        return r == Z_STREAM_END && z.avail_out == 0;
    }

    // The tiles `radius` round (lat, lon), each way; levels -1 and -2 for
    // those `finerRadius` round. False if the file isn't a version 2 pack, or
    // is one the Tab5 would refuse - too big for it, or damaged: checked as
    // its ground task checks it, by the same code.
    bool load(const char *path, double lat, double lon, int radius, int finerRadius = 1) {
        FILE *f = fopen(path, "rb");
        if (f == nullptr) {
            return false;
        }
        std::vector<uint8_t> file;
        uint8_t buf[1 << 16];
        size_t n;
        while ((n = fread(buf, 1, sizeof buf, f)) > 0) {
            file.insert(file.end(), buf, buf + n);
        }
        fclose(f);
        const uint8_t *h = file.data();
        using namespace groundpack;
        PackCheck check;
        std::vector<int16_t> entryAt;
        bool ok = file.size() <= UINT32_MAX && checkHead(h, (uint32_t)file.size(), check);
        if (ok) {
            entryAt.resize((size_t)check.rows * check.cols);
            ok = checkIndex(h + check.indexAt, check, nullptr, entryAt.data());
        }
        if (!ok) {
            fprintf(stderr, "ground: %s refused, as the Tab5 would refuse it: %s\n", path, check.why);
            return false;
        }
        if (check.version != 2) {
            return false;
        }
        int south = (int16_t)(h[8] | h[9] << 8), west = (int16_t)(h[10] | h[11] << 8);
        int count = (int)u32(h + 16);
        uint32_t indexAt = u32(h + 20);
        int side = 2 * radius + 1;
        int centreNorth = (int)floor((90 - lat) * TILES_PER_DEG), centreWest = (int)floor((lon + 180) * TILES_PER_DEG);
        window.north = centreNorth - radius;
        window.west = centreWest - radius;
        window.rows = window.cols = side;
        coarse.assign(side * side, nullptr);
        mid.assign(side * side, nullptr);
        detail.assign(side * side, nullptr);
        finer1.assign(side * side, nullptr);
        finer2.assign(side * side, nullptr);
        for (int i = 0; i < count; i++) {
            const uint8_t *e = h + indexAt + 28 * i;
            int row = e[0] | e[1] << 8, col = e[2] | e[3] << 8;
            int northRow = 90 * TILES_PER_DEG - (south + row) - 1, westCol = west + col + 180 * TILES_PER_DEG;
            int r = northRow - window.north, c = westCol - window.west;
            if (r < 0 || r >= side || c < 0 || c >= side) {
                continue;
            }
            int c0 = cols0At(northRow);
            if (c0 == 0) {
                continue;
            }
            int slot = r * side + c;
            size_t coarseN = 3 * (size_t)(ROWS0 / 16) * (c0 / 16);
            size_t midN = midCells(c0) + 2 * (size_t)(ROWS0 / 6) * (c0 / 6);
            size_t detailN = (size_t)ROWS0 * c0;
            struct Block {
                uint32_t at, size;
                size_t n;
                std::vector<const uint8_t *> *into;
            } blocks[3] = {{u32(e + 4), u32(e + 8), coarseN, &coarse},
                           {u32(e + 12), u32(e + 16), midN, &mid},
                           {u32(e + 20), u32(e + 24), detailN, &detail}};
            for (const Block &b : blocks) {
                if (b.size == 0) {
                    continue;
                }
                std::unique_ptr<uint8_t[]> cells(new uint8_t[b.n]);
                if (!inflated(h + b.at, b.size, cells.get(), b.n)) {
                    fprintf(stderr, "ground: tile %d,%d didn't inflate\n", northRow, westCol);
                    return false;
                }
                (*b.into)[slot] = cells.get();
                keep.push_back(std::move(cells));
            }
            if (abs(r - radius) <= finerRadius && abs(c - radius) <= finerRadius && detail[slot] != nullptr) {
                uint8_t *a = finerLevel(detail[slot], c0, northRow, westCol, -1);
                uint8_t *b = finerLevel(detail[slot], c0, northRow, westCol, -2);
                finer1[slot] = a;
                finer2[slot] = b;
                finers.push_back(a);
                finers.push_back(b);
            }
        }
        window.coarse = coarse.data();
        window.mid = mid.data();
        window.detail = detail.data();
        window.finer[0] = finer1.data();
        window.finer[1] = finer2.data();
        // No tile of the pack anywhere in it: no grid of heights - the land
        // flat, as the Tab5's ground task leaves it.
        grid = groundpack::TerrainGrid();
        if (!groundpack::hasTiles(window, window.north, window.west, window.rows, window.cols)) {
            return true;
        }
        using G = groundpack::TerrainGrid;
        grid.north = window.north;
        grid.west = window.west;
        grid.tilesDown = window.rows;
        grid.tilesAcross = window.cols;
        grid.rows = grid.tilesDown * G::TILE_ROWS;
        grid.cols = grid.tilesAcross * G::TILE_COLS;
        grid.blockRows = (grid.rows + G::BLOCK - 1) / G::BLOCK;
        grid.blockCols = (grid.cols + G::BLOCK - 1) / G::BLOCK;
        gridHeights.assign((size_t)grid.rows * grid.cols, 0);
        gridCellMax.assign((size_t)grid.rows * grid.cols, 0);
        gridMax.assign((size_t)grid.blockRows * grid.blockCols, 0);
        gridWideMax.assign((size_t)grid.blockRows * grid.blockCols, 0);
        grid.wideMax = gridWideMax.data();
        grid.heights = gridHeights.data();
        grid.cellMax = gridCellMax.data();
        grid.blockMax = gridMax.data();
        groundpack::fillTerrainGrid(window, grid);
        return true;
    }
};
