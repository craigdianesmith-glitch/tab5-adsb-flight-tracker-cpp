// Checks the ground pack's checks - checkHead() and checkIndex(), in
// src/ground_pack.cpp, which the Tab5's ground task puts the card's
// ground.bin through before its tiles take any memory, and the PC
// preview's loader too - against packs made here: a good one, one at each of
// the Tab5's limits and one just past each, and one damaged each way a pack
// could be. Then the preview's loader on them, taking and refusing them as
// the Tab5 would, and leaving the terrain's grid unbuilt where the pack has
// no tile. Build and run, from the repo's root:
//
//   g++ -std=c++17 -O1 -Iinclude tools/ground_pack_test/test.cpp src/ground_pack.cpp -lz -o /tmp/ground_pack_test
//   /tmp/ground_pack_test [ground.bin]
//
// (all one line). A pack named is checked as well, and must pass. Each check
// that fails says so; the last line says how many did.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#include <string>
#include <utility>
#include <vector>

#include "../landing3d_preview/ground_pc.h"

using namespace groundpack;

int g_checks = 0, g_failed = 0;

void expect(bool ok, const std::string &what) {
    g_checks++;
    if (!ok) {
        g_failed++;
        printf("FAILED: %s\n", what.c_str());
    }
}

const char *name(Status s) {
    switch (s) {
    case Status::UNKNOWN: return "UNKNOWN";
    case Status::NO_CARD: return "NO_CARD";
    case Status::NO_PACK: return "NO_PACK";
    case Status::LOADING: return "LOADING";
    case Status::READY: return "READY";
    case Status::OTHER_VERSION: return "OTHER_VERSION";
    case Status::TOO_BIG: return "TOO_BIG";
    case Status::DAMAGED: return "DAMAGED";
    case Status::NO_ROOM: return "NO_ROOM";
    }
    return "?";
}

void put16(std::vector<uint8_t> &b, size_t at, uint32_t v) {
    b[at] = v & 0xFF;
    b[at + 1] = v >> 8 & 0xFF;
}
void put32(std::vector<uint8_t> &b, size_t at, uint32_t v) {
    put16(b, at, v & 0xFFFF);
    put16(b, at + 2, v >> 16);
}

// zlib's deflate without its header, as tools/gen_ground.py writes a block.
std::vector<uint8_t> deflated(const std::vector<uint8_t> &raw) {
    z_stream z = {};
    deflateInit2(&z, 9, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY);
    std::vector<uint8_t> out(deflateBound(&z, raw.size()) + 16);
    z.next_in = (Bytef *)raw.data();
    z.avail_in = (uInt)raw.size();
    z.next_out = out.data();
    z.avail_out = (uInt)out.size();
    deflate(&z, Z_FINISH);
    out.resize(z.total_out);
    deflateEnd(&z);
    return out;
}

// What a pack is made of: its header's numbers and its tiles, (row, column)
// from its south-west corner. At 55 north unless said - k 1.5, 192 columns.
// With `shared`, every tile's row of the index points at the first tile's
// blocks: thousands of tiles in a file of a few kilobytes, which the checks
// can't tell from the real thing.
struct Spec {
    int version = 2, perDegree = 5;
    int south = 275, west = -25, rows = 4, cols = 4;
    std::vector<std::pair<int, int>> tiles = {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
    bool shared = false;
    int16_t height = 120;  // every height in it, metres
};

constexpr size_t HEAD = 32;

// A tile's cells: grass at the flat's shade, then heights - as many bytes as
// the block inflates to.
std::vector<uint8_t> cells(size_t covers, size_t bytes, int16_t height) {
    std::vector<uint8_t> raw(bytes, 0x3B);
    for (size_t at = covers; at + 1 < bytes; at += 2) {
        raw[at] = height & 0xFF;
        raw[at + 1] = (height >> 8) & 0xFF;
    }
    return raw;
}

std::vector<uint8_t> make(const Spec &s) {
    int count = (int)s.tiles.size(), rowBytes = s.version == 2 ? 28 : 20;
    int northEdge = 90 * TILES_PER_DEG - (s.south + s.rows);
    std::vector<std::vector<uint8_t>> coarse, mid, detail;
    for (int i = 0; i < count; i++) {
        if (s.shared && i > 0) {
            break;
        }
        int c0 = cols0At(northEdge + s.rows - 1 - s.tiles[i].first);
        coarse.push_back(deflated(cells((size_t)(ROWS0 / 16) * (c0 / 16), coarseBytes(c0), s.height)));
        mid.push_back(deflated(cells(midHeightsAt(c0), midBytes(c0), s.height)));
        detail.push_back(deflated(cells(detailBytes(c0), detailBytes(c0), s.height)));
    }
    size_t indexAt = HEAD, coarseAt = indexAt + (size_t)rowBytes * count, coarseSize = 0, midSize = 0;
    for (auto &b : coarse) {
        coarseSize += b.size();
    }
    for (auto &b : mid) {
        midSize += b.size();
    }
    size_t midAt = coarseAt + coarseSize, detailAt = midAt + midSize;
    std::vector<uint8_t> out(coarseAt);
    memcpy(out.data(), "OVGR", 4);
    put16(out, 4, s.version);
    put16(out, 6, s.perDegree);
    put16(out, 8, (uint16_t)(int16_t)s.south);
    put16(out, 10, (uint16_t)(int16_t)s.west);
    put16(out, 12, s.rows);
    put16(out, 14, s.cols);
    put32(out, 16, count);
    put32(out, 20, indexAt);
    put32(out, 24, coarseAt);
    put32(out, 28, coarseSize);
    size_t co = coarseAt, mo = midAt, de = detailAt;
    for (int i = 0; i < count; i++) {
        int b = s.shared ? 0 : i;
        size_t row = indexAt + (size_t)rowBytes * i;
        put16(out, row, s.tiles[i].first);
        put16(out, row + 2, s.tiles[i].second);
        put32(out, row + 4, s.shared ? coarseAt : co);
        put32(out, row + 8, coarse[b].size());
        put32(out, row + 12, s.shared ? midAt : mo);
        put32(out, row + 16, mid[b].size());
        if (rowBytes == 28) {
            put32(out, row + 20, s.shared ? detailAt : de);
            put32(out, row + 24, detail[b].size());
        }
        if (!s.shared) {
            co += coarse[b].size();
            mo += mid[b].size();
            de += detail[b].size();
        }
    }
    for (auto *blocks : {&coarse, &mid, &detail}) {
        if (blocks == &detail && rowBytes == 20) {
            break;
        }
        for (auto &b : *blocks) {
            out.insert(out.end(), b.begin(), b.end());
        }
    }
    return out;
}

// The pack's checks, as the ground task runs them: the header, then - if it
// passes - the index, into a grid of places as the Tab5 holds it.
PackCheck checked(const std::vector<uint8_t> &pack) {
    PackCheck c;
    if (checkHead(pack.data(), (uint32_t)pack.size(), c)) {
        std::vector<int16_t> entryAt((size_t)c.rows * c.cols);
        std::vector<Entry> entries(c.count);
        checkIndex(pack.data() + c.indexAt, c, entries.data(), entryAt.data());
    }
    return c;
}

// `pack` checked, and expected to come to `want` - its words with `says` in
// them, where given.
void expectStatus(const char *what, const std::vector<uint8_t> &pack, Status want, const char *says = nullptr) {
    PackCheck c = checked(pack);
    bool ok = c.status == want && (says == nullptr || strstr(c.why, says) != nullptr);
    printf("  %-58s %-13s %s\n", what, name(c.status), c.why);
    expect(ok, std::string(what) + ": wanted " + name(want) + (says ? std::string(" saying \"") + says + "\"" : "") +
                   ", got " + name(c.status) + " \"" + c.why + "\"");
}

// The tiles of a grid `rows` by `cols`, the first `count` of them, row by row.
std::vector<std::pair<int, int>> fill(int rows, int cols, int count) {
    std::vector<std::pair<int, int>> out;
    for (int r = 0; r < rows && (int)out.size() < count; r++) {
        for (int c = 0; c < cols && (int)out.size() < count; c++) {
            out.push_back({r, c});
        }
    }
    return out;
}

size_t rowAt(const Spec &s, int i) { return HEAD + (size_t)(s.version == 2 ? 28 : 20) * i; }

void headerAndIndex() {
    printf("The checks, on packs made here:\n");
    Spec good;
    std::vector<uint8_t> pack = make(good);
    expectStatus("a good pack", pack, Status::READY);
    PackCheck c = checked(pack);
    expect(c.count == 4 && c.rows == 4 && c.cols == 4 && c.south == 275 && c.west == -25 && c.version == 2,
           "a good pack's header read back");
    expect(c.coarseBytes == 4 * coarseBytes(192), "a good pack's level 4: 4 tiles of 648 bytes at k 1.5");
    {
        Spec v1 = good;
        v1.version = 1;
        expectStatus("a version 1 pack, without level 0", make(v1), Status::READY);
    }

    // Not one this firmware reads.
    {
        auto p = pack;
        p[0] = 'X';
        expectStatus("not a ground pack", p, Status::OTHER_VERSION, "isn't a ground pack");
        expectStatus("shorter than a header", std::vector<uint8_t>(pack.begin(), pack.begin() + 20),
                     Status::OTHER_VERSION, "isn't a ground pack");
        p = pack;
        put16(p, 4, 3);
        expectStatus("version 3", p, Status::OTHER_VERSION, "version 3");
        p = pack;
        put16(p, 6, 4);
        expectStatus("4 tiles a degree", p, Status::OTHER_VERSION, "4 tiles a degree");
    }

    // The Tab5's limits: at each, and past it.
    {
        Spec most;  // 6,000 tiles between 50 and 60 north, k 1.5: 3.9MB held
        most.south = 255;
        most.rows = 45;
        most.cols = 134;
        most.tiles = fill(most.rows, most.cols, MAX_TILES);
        most.shared = true;
        expectStatus("6000 tiles at 51 to 60 north - the most", make(most), Status::READY);
        Spec over = most;
        over.tiles = fill(over.rows, over.cols, MAX_TILES + 1);
        expectStatus("6001 tiles", make(over), Status::TOO_BIG, "6001 tiles, the most 6000");
        auto p = make(most);
        put32(p, 16, 70000);
        expectStatus("70,000 tiles, said in the header alone", p, Status::TOO_BIG, "70000 tiles, the most 6000");
    }
    {
        Spec south;  // below 50 degrees, k 1, a tile's level 4 is 972 bytes: 4MB comes at 4,315
        south.south = 100;
        south.rows = 66;
        south.cols = 66;
        south.shared = true;
        south.tiles = fill(66, 66, 4315);
        expectStatus("4315 tiles at 20 north - the most there", make(south), Status::READY);
        south.tiles = fill(66, 66, 4316);
        expectStatus("4316 tiles at 20 north", make(south), Status::TOO_BIG, "need 4097KB of memory, the most 4096KB");
    }
    {
        Spec wide;  // a grid of 200 by 200 places, from 20 to 60 north: the most
        wide.rows = 200;
        wide.cols = 200;
        wide.south = 100;
        wide.tiles = {{0, 0}, {199, 199}};
        expectStatus("a grid of 200 by 200 tiles - the most", make(wide), Status::READY);
        wide.rows = 201;
        expectStatus("a grid of 201 by 200 tiles", make(wide), Status::TOO_BIG,
                     "spans 201 by 200 tiles, the most 40000");
        auto p = make(good);
        put16(p, 12, 65535);
        put16(p, 14, 65535);
        expectStatus("a grid of 65,535 by 65,535 - its places past 32 bits' int", p, Status::TOO_BIG, "spans 65535");
    }
    {
        auto p = pack;
        put32(p, 28, MAX_COARSE_BYTES + 1);
        expectStatus("coarse blocks of 4MB and a byte", p, Status::TOO_BIG, "4097KB, the most 4096KB");
    }

    // Damaged.
    {
        auto p = pack;
        put32(p, 16, 0);
        expectStatus("no tiles", p, Status::DAMAGED, "no tiles");
        p = pack;
        put16(p, 12, 0);
        expectStatus("no rows", p, Status::DAMAGED, "no tiles");
        p = pack;
        put32(p, 16, 17);
        expectStatus("17 tiles on a grid of 16", p, Status::DAMAGED, "17 tiles on a grid of 16");
        Spec pole = good;
        pole.south = 440;
        pole.rows = 20;
        pole.tiles = {{0, 0}};
        expectStatus("a grid past the north pole", make(pole), Status::DAMAGED, "off the world");
        Spec east = good;
        east.west = 890;
        east.cols = 20;
        east.tiles = {{0, 0}};
        expectStatus("a grid past 180 east", make(east), Status::DAMAGED, "off the world");
        p = pack;
        put32(p, 20, (uint32_t)pack.size() - 10);
        expectStatus("the index past the end of the file", p, Status::DAMAGED, "index runs past");
        p = pack;
        put32(p, 20, 8);
        expectStatus("the index in the header", p, Status::DAMAGED, "index runs past");
        p = pack;
        put32(p, 28, (uint32_t)pack.size());
        expectStatus("the coarse blocks past the end of the file", p, Status::DAMAGED, "coarse blocks run past");
        p = pack;
        put32(p, 24, 0xFFFFFFF0u);
        put32(p, 28, 0x20);
        expectStatus("the coarse blocks wrapping 32 bits", p, Status::DAMAGED, "coarse blocks run past");
        p = std::vector<uint8_t>(pack.begin(), pack.end() - 100);
        expectStatus("the file cut short", p, Status::DAMAGED, "detail block runs past");
    }
    {
        Spec far = good;  // 75.2 north: past where the pack has tiles
        far.south = 376;
        far.rows = 2;
        far.tiles = {{0, 0}};
        expectStatus("a tile past 75 degrees", make(far), Status::DAMAGED, "past 75 degrees");
    }
    {
        auto p = pack;
        put16(p, rowAt(good, 2), 4);
        expectStatus("a tile off its grid", p, Status::DAMAGED, "tile 2 is off its grid");
        p = pack;
        put16(p, rowAt(good, 3), 0);
        put16(p, rowAt(good, 3) + 2, 0);
        expectStatus("a tile on the grid twice", p, Status::DAMAGED, "tile 3 is on its grid twice");
        uint32_t coarseAt = pack[24] | pack[25] << 8 | pack[26] << 16 | (uint32_t)pack[27] << 24;
        p = pack;
        put32(p, rowAt(good, 1) + 4, coarseAt - 1);
        expectStatus("a coarse block before the coarse blocks", p, Status::DAMAGED, "isn't among the coarse");
        p = pack;
        put32(p, rowAt(good, 1) + 8, 0);
        expectStatus("a coarse block of nought", p, Status::DAMAGED, "isn't among the coarse");
        p = pack;
        put32(p, rowAt(good, 0) + 12, (uint32_t)pack.size() - 5);
        expectStatus("a mid block past the end of the file", p, Status::DAMAGED, "tile 0's mid block runs past");
        p = pack;
        put32(p, rowAt(good, 0) + 12, 0xFFFFFF00u);
        put32(p, rowAt(good, 0) + 16, 0x200);
        expectStatus("a mid block wrapping 32 bits", p, Status::DAMAGED, "tile 0's mid block runs past");
        p = pack;
        put32(p, rowAt(good, 1) + 24, 0x7FFFFFFF);
        expectStatus("a detail block of 2GB", p, Status::DAMAGED, "tile 1's detail block runs past");
        // Inside the file, but bigger than its cells could pack to.
        p = pack;
        p.resize(pack.size() + 200000);
        put32(p, rowAt(good, 2) + 16, (uint32_t)(2 * midBytes(192) + 65));
        expectStatus("a mid block bigger than its cells could make", p, Status::DAMAGED, "bigger than its cells");
    }
}

// A pack written to a file, for the preview's loader.
std::string written(const std::vector<uint8_t> &pack, const char *name) {
    const char *dir = getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp";
    std::string path = std::string(dir) + "/ground_pack_test_" + name + ".bin";
    FILE *f = fopen(path.c_str(), "wb");
    fwrite(pack.data(), 1, pack.size(), f);
    fclose(f);
    return path;
}

void previewLoader() {
    printf("The preview's loader, as the Tab5 takes them:\n");
    Spec good;
    // The middle of tile (1, 1): 55.3 north, 4.7 west.
    double lat = (good.south + 1.5) / TILES_PER_DEG, lon = (good.west + 1.5) / TILES_PER_DEG;
    std::string path = written(make(good), "good");
    {
        PcGround g;
        bool ok = g.load(path.c_str(), lat, lon, 7);
        expect(ok, "a good pack loads");
        expect(ok && g.grid.heights != nullptr && g.grid.highest == good.height,
               "and its terrain grid is built, its heights the pack's");
        printf("  a good pack round its tiles: %s, grid %s, highest %dm\n", ok ? "loaded" : "refused",
               g.grid.heights ? "built" : "none", g.grid.highest);
    }
    {
        PcGround g;
        bool ok = g.load(path.c_str(), 40.0, 20.0, 7);
        expect(ok, "a good pack loads far from its tiles");
        expect(ok && g.grid.heights == nullptr, "but no terrain grid is built there - the land flat");
        printf("  the same, at 40N 20E, no tile of it near: %s, grid %s\n", ok ? "loaded" : "refused",
               g.grid.heights ? "built" : "none");
    }
    remove(path.c_str());
    {
        Spec over;
        over.south = 255;
        over.rows = 45;
        over.cols = 134;
        over.tiles = fill(45, 134, MAX_TILES + 1);
        over.shared = true;
        std::string big = written(make(over), "big");
        PcGround g;
        expect(!g.load(big.c_str(), lat, lon, 7), "a pack of 6001 tiles is refused");
        remove(big.c_str());
    }
    {
        auto p = make(good);
        put32(p, rowAt(good, 1) + 12, 0xFFFFFF00u);
        std::string bad = written(p, "damaged");
        PcGround g;
        expect(!g.load(bad.c_str(), lat, lon, 7), "a damaged pack is refused");
        remove(bad.c_str());
    }
}

void tilesAbout() {
    printf("Tiles about a place:\n");
    const uint8_t cell = 0;
    const uint8_t *coarse[9] = {nullptr, nullptr, nullptr, nullptr, &cell, nullptr, nullptr, nullptr, nullptr};
    Window w;
    w.north = 100;
    w.west = 200;
    w.rows = w.cols = 3;
    w.coarse = coarse;  // a tile at (101, 201) alone
    expect(hasTiles(w, 101, 201, 1, 1), "the tile itself");
    expect(hasTiles(w, 96, 196, 11, 15), "a block round it");
    expect(!hasTiles(w, 102, 202, 5, 5), "a block beside it, partly over the window");
    expect(!hasTiles(w, 0, 0, 11, 15), "a block far off");
    expect(!hasTiles(Window(), 0, 0, 1000, 1000), "no window");
}

int main(int argc, char **argv) {
    headerAndIndex();
    previewLoader();
    tilesAbout();
    if (argc > 1) {
        FILE *f = fopen(argv[1], "rb");
        std::vector<uint8_t> pack;
        if (f != nullptr) {
            uint8_t buf[1 << 16];
            size_t n;
            while ((n = fread(buf, 1, sizeof buf, f)) > 0) {
                pack.insert(pack.end(), buf, buf + n);
            }
            fclose(f);
        }
        PackCheck c = checked(pack);
        printf("%s: %s %s - %d tiles on a grid of %d by %d, %zuKB of level 4 held, the file %.1fMB\n", argv[1],
               name(c.status), c.why, c.count, c.rows, c.cols, c.coarseBytes / 1024, pack.size() / 1048576.0);
        expect(c.status == Status::READY, std::string(argv[1]) + " passes");
    }
    printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed != 0;
}
