// Levels -1 and -2 of the ground pack, as the ground task works them out for
// the tiles nearest the eye (ground_pack.cpp's finerLevel()), next to the
// pack's own level 0 - a picture of each round a place, each cell coloured
// by its cover and shaded as the 3D view does, for a look at what they
// make of the land close to.
//
//   g++ -O2 -std=c++17 -Iinclude tools/finer_preview/preview.cpp src/ground_pack.cpp -lz -o finer_preview
//   ./finer_preview ground.bin LAT LON OUT_PREFIX [CELLS]
//
// Writes OUT_PREFIX-0.ppm, -1.ppm and -2.ppm: CELLS of level 0's cells each
// way round (LAT, LON) - 48 unless given - at 8, 4 and 2 pixels a cell.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#include <vector>

#include "ground_pack.h"

using namespace groundpack;

static const unsigned char RGB[16][3] = {{86, 98, 70},   {40, 72, 42},    {94, 90, 64},   {80, 118, 58},
                                         {126, 128, 76}, {124, 118, 108}, {128, 124, 112}, {236, 240, 244},
                                         {44, 66, 90},   {70, 90, 70},    {40, 84, 54},   {102, 110, 88},
                                         {100, 106, 84}, {112, 106, 100}, {108, 100, 64}, {118, 116, 100}};

static uint32_t u32(const unsigned char *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

int main(int argc, char **argv) {
    if (argc < 5) {
        fprintf(stderr, "usage: %s ground.bin LAT LON OUT_PREFIX [CELLS]\n", argv[0]);
        return 2;
    }
    double lat = atof(argv[2]), lon = atof(argv[3]);
    int around = argc > 5 ? atoi(argv[5]) : 48;
    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        perror(argv[1]);
        return 1;
    }
    std::vector<unsigned char> pack;
    unsigned char buf[1 << 16];
    size_t n;
    while ((n = fread(buf, 1, sizeof buf, f)) > 0) {
        pack.insert(pack.end(), buf, buf + n);
    }
    fclose(f);
    const unsigned char *h = pack.data();
    if (memcmp(h, "OVGR", 4) != 0 || h[4] != 2) {
        fprintf(stderr, "not a version 2 ground pack\n");
        return 1;
    }
    int south = (int16_t)(h[8] | h[9] << 8), west = (int16_t)(h[10] | h[11] << 8);
    int count = (int)u32(h + 16);
    uint32_t indexAt = u32(h + 20);
    int row = (int)floor(lat * TILES_PER_DEG) - south, col = (int)floor(lon * TILES_PER_DEG) - west;
    const unsigned char *e = nullptr;
    for (int i = 0; i < count; i++) {
        const unsigned char *r = h + indexAt + 28 * i;
        if ((r[0] | r[1] << 8) == row && (r[2] | r[3] << 8) == col) {
            e = r;
        }
    }
    if (e == nullptr) {
        fprintf(stderr, "no tile there\n");
        return 1;
    }
    int northRow = 90 * TILES_PER_DEG - (south + row) - 1, westCol = west + col + 180 * TILES_PER_DEG;
    int cols0 = cols0At(northRow);
    std::vector<uint8_t> detail((size_t)ROWS0 * cols0);
    z_stream z = {};
    inflateInit2(&z, -15);
    z.next_in = (Bytef *)h + u32(e + 20);
    z.avail_in = u32(e + 24);
    z.next_out = detail.data();
    z.avail_out = (uInt)detail.size();
    if (inflate(&z, Z_FINISH) != Z_STREAM_END) {
        fprintf(stderr, "the detail block didn't inflate\n");
        return 1;
    }
    inflateEnd(&z);
    // Where (lat, lon) is in the tile, in level 0's cells.
    double v = ((90 - lat) * TILES_PER_DEG - northRow) * ROWS0, u = ((lon + 180) * TILES_PER_DEG - westCol) * cols0;
    for (int level = 0; level >= FINEST; level--) {
        int s = 1 << -level, parts = 2 << -level, pr = ROWS0 / 2, pc = cols0 / 2;
        uint8_t *cells = level == 0 ? detail.data() : finerLevel(detail.data(), cols0, northRow, westCol, level);
        int px = 8 >> -level;  // pixels a cell
        int side = 2 * around * s, r0 = (int)(v * s) - around * s, c0 = (int)(u * s) - around * s;
        std::vector<unsigned char> img((size_t)side * px * side * px * 3);
        for (int y = 0; y < side; y++) {
            for (int x = 0; x < side; x++) {
                int i = std::min(ROWS0 * s - 1, std::max(0, r0 + y)), j = std::min(cols0 * s - 1, std::max(0, c0 + x));
                uint8_t c = cells[(size_t)((i / pr) * parts + j / pc) * pr * pc + (i % pr) * pc + j % pc];
                float k = (c & 15) / 11.0f;
                for (int dy = 0; dy < px; dy++) {
                    for (int dx = 0; dx < px; dx++) {
                        unsigned char *o = &img[(((size_t)y * px + dy) * side * px + x * px + dx) * 3];
                        for (int ch = 0; ch < 3; ch++) {
                            o[ch] = (unsigned char)std::min(255.0f, RGB[c >> 4][ch] * k);
                        }
                    }
                }
            }
        }
        char name[512];
        snprintf(name, sizeof name, "%s-%d.ppm", argv[4], -level);
        FILE *o = fopen(name, "wb");
        fprintf(o, "P6 %d %d 255\n", side * px, side * px);
        fwrite(img.data(), 1, img.size(), o);
        fclose(o);
        printf("%s\n", name);
        if (level != 0) {
            free(cells);
        }
    }
    return 0;
}
