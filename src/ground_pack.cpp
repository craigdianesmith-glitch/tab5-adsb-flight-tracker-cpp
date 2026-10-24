#include "ground_pack.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <SD.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <miniz.h>
#include "recorder.h"
#endif

namespace groundpack {
namespace {

struct Entry {
    uint16_t row, col;  // in tiles, from the pack's south-west corner
    uint32_t coarseAt, coarseSize, midAt, midSize, detailAt, detailSize;  // no detail block: size nought
};

// A tile's blocks' sizes as inflated, by its columns at level 0 - see the
// header: levels 1 to 3, and with them the heights every 15 seconds; level
// 0; level 4, and the heights at its cells.
constexpr int ROWS1 = ROWS0 / 2;
size_t midCells(int cols0) {
    int c1 = cols0 / 2;
    return (size_t)ROWS1 * c1 + (ROWS1 / 2) * (c1 / 2) + (ROWS1 / 4) * (c1 / 4);
}
size_t midBytes(int cols0) { return midCells(cols0) + 2 * (size_t)(ROWS0 / 6) * (cols0 / 6); }
size_t detailBytes(int cols0) { return (size_t)ROWS0 * cols0; }
size_t coarseBytes(int cols0) { return 3 * (size_t)(ROWS0 / 16) * (cols0 / 16); }

bool g_open = false;
int g_south = 0, g_westTile = 0, g_rows = 0, g_cols = 0;  // the pack's, south-based rows
Entry *g_entries = nullptr;
int g_count = 0;
int16_t *g_entryAt = nullptr;  // rows*cols, north-based row: the entry there, or -1
Window g_window;
const uint8_t **g_coarse = nullptr, **g_mid = nullptr, **g_detail = nullptr, **g_finer[2] = {};
uint8_t *g_coarseData = nullptr;

// The tiles read in at levels 1 to 3, and at level 0, and those with levels
// -1 and -2 worked out: each one's memory, and which tile - and those taken
// out of the window, kept a few frames for any band still being drawn from
// them before their memory goes back.
struct Held {
    int slot = -1;  // north-based row * cols + col, -1 none
    uint8_t *data = nullptr;
    uint32_t outFrame = 0;  // 0 while in the window
};
struct Pool {
    Held *held;
    int size, most;  // its places, and how many of them may be in the window at once
    const uint8_t ***table;  // g_mid, g_detail or g_finer's
};
constexpr int MOST_MID = 64, MOST_DETAIL = 12, MOST_FINER = 4;
// The tiles read in at level 0: the eye's, and those round it - a tile from
// it or less - where its cells are big enough to see. Those with levels -1
// and -2 worked out: the two by two of them nearest the eye.
constexpr int DETAIL_RING = 1;
Held g_midHeld[MOST_MID + 16], g_detailHeld[MOST_DETAIL + 8], g_finerHeld[2][MOST_FINER + 4];
Pool g_midPool = {g_midHeld, MOST_MID + 16, MOST_MID, &g_mid};
Pool g_detailPool = {g_detailHeld, MOST_DETAIL + 8, MOST_DETAIL, &g_detail};
Pool g_finerPool[2] = {{g_finerHeld[0], MOST_FINER + 4, MOST_FINER, &g_finer[0]},
                       {g_finerHeld[1], MOST_FINER + 4, MOST_FINER, &g_finer[1]}};
uint32_t g_frame = 0;

// What the view wants read, newest first, for the task: round the eye's
// tile, and the two by two tiles nearest it - their north-west one's row and
// column.
volatile int g_wantRow = -1, g_wantCol = -1, g_wantRadius = 0, g_blockRow = -1, g_blockCol = -1;
volatile int g_finerWanted = 2;  // how many of levels -1 and -2 the eye's height has use for
uint32_t g_finerUs = 0;

// The horizon: three, in PSRAM - the latest done, the one before, and one
// being worked out - and where the view last wanted one, under g_lock.
Horizon *g_horizons = nullptr;
// The terrain's grids: the one published for the view - see terrainGrid() -
// and the next, built into the other, about the eye: 11 tiles down by 15
// across, some 245km by 210km at 55 north, past the furthest hills drawn.
// Built afresh once the eye is two tiles from the last one's middle, or a
// second after more mid blocks came in - never into the one a frame may be
// reading.
constexpr int GRID_DOWN = 11, GRID_ACROSS = 15;
TerrainGrid g_grids[2];
const TerrainGrid *volatile g_terrainGridNow = nullptr;
int g_gridBuilt = -1, g_gridMids = -1;
uint32_t g_gridAt = 0;
volatile int g_midsIn = 0;  // mid blocks read in so far
volatile int g_horizonAt = -1;
bool g_horizonWanted = false;
double g_horizonLat = 0, g_horizonLon = 0;
float g_horizonHeight = 0;
volatile bool g_coarseReady = false;
int g_reads = 0;
uint32_t g_readUs = 0;

// Memory for cells: the PSRAM's, on the device.
void *cellMemory(size_t bytes) {
#ifdef ESP_PLATFORM
    return heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
#else
    return malloc(bytes);
#endif
}
void freeCells(void *p) {
#ifdef ESP_PLATFORM
    heap_caps_free(p);
#else
    free(p);
#endif
}

// Every tile of `pool` out of the window - the eye too high for them to be
// drawn - each one's memory given back once no band can still be drawing
// from it. Under g_lock.
void retireAll(Pool &pool) {
    for (int i = 0; i < pool.size; i++) {
        Held &h = pool.held[i];
        if (h.slot >= 0 && h.outFrame == 0) {
            (*pool.table)[h.slot] = nullptr;
            h.outFrame = g_frame == 0 ? 1 : g_frame;
            h.slot = -1;
        } else if (h.data != nullptr && h.outFrame != 0 && g_frame - h.outFrame > 6) {
            freeCells(h.data);
            h = Held();
        }
    }
}

// A 32-bit hash of a cell's place on a level's whole grid - as like chance
// as the roughening needs, and the same for a cell whichever tile asks.
inline uint32_t cellHash(uint32_t r, uint32_t c, uint32_t salt) {
    uint32_t h = (r * 0x9E3779B1u) ^ (c * 0x85EBCA77u) ^ salt;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return h;
}

#ifdef ESP_PLATFORM
TaskHandle_t g_task = nullptr;
SemaphoreHandle_t g_lock = nullptr;  // the pools' places and the window's pointers to them

bool readAt(File &f, uint32_t at, void *out, size_t size) { return f.seek(at) && f.read((uint8_t *)out, size) == size; }

// A block, inflated into `out` of `size` bytes exactly. The inflater's state
// - some 11KB of tables - in the PSRAM, once, rather than on the task's stack
// in the chip's own memory, which tinfl_decompress_mem_to_mem() puts it on.
bool inflate(const uint8_t *in, size_t inSize, uint8_t *out, size_t size) {
    static tinfl_decompressor *state = nullptr;
    if (state == nullptr) {
        state = (tinfl_decompressor *)heap_caps_malloc(sizeof(tinfl_decompressor), MALLOC_CAP_SPIRAM);
        if (state == nullptr) {
            return false;
        }
    }
    tinfl_init(state);
    size_t inLeft = inSize, outLeft = size;
    tinfl_status status = tinfl_decompress(state, in, &inLeft, out, out, &outLeft,
                                           TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    return status == TINFL_STATUS_DONE && outLeft == size;
}

// Every tile's level 4, in one read of them all. `northEdge` the pack's
// first row of tiles, from the north pole.
bool loadCoarse(File &f, uint32_t at, uint32_t size, int northEdge) {
    uint8_t *all = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (all == nullptr) {
        return false;
    }
    bool ok;
    {
        recorder::CardLock lock;
        ok = readAt(f, at, all, size);
    }
    size_t need = 0;
    for (int i = 0; i < g_count; i++) {
        need += coarseBytes(cols0At(northEdge + g_rows - 1 - g_entries[i].row));
    }
    g_coarseData = ok ? (uint8_t *)heap_caps_malloc(need, MALLOC_CAP_SPIRAM) : nullptr;
    ok = ok && g_coarseData != nullptr;
    size_t out = 0;
    for (int i = 0; ok && i < g_count; i++) {
        const Entry &e = g_entries[i];
        int northRow = g_rows - 1 - e.row;
        int c0 = cols0At(northEdge + northRow);
        size_t n = coarseBytes(c0);
        if (c0 == 0 || e.coarseAt < at || e.coarseAt + e.coarseSize > at + size ||
            !inflate(all + (e.coarseAt - at), e.coarseSize, g_coarseData + out, n)) {
            ok = false;
            break;
        }
        g_coarse[northRow * g_cols + e.col] = g_coarseData + out;
        out += n;
        if ((i & 31) == 31) {
            vTaskDelay(1);  // the inflating is the core's other tasks' time too
        }
    }
    heap_caps_free(all);
    return ok;
}

// A tile's block at `at`, `size` bytes, read in and inflated to `n` - or
// nullptr where it can't be.
uint8_t *loadBlock(File &f, uint32_t at, uint32_t size, size_t n) {
    uint8_t *packed = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    uint8_t *cells = (uint8_t *)heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    bool ok = packed != nullptr && cells != nullptr;
    uint32_t t0 = micros();
    if (ok) {
        recorder::CardLock lock;
        ok = readAt(f, at, packed, size);
    }
    ok = ok && inflate(packed, size, cells, n);
    g_readUs += micros() - t0;
    g_reads++;
    heap_caps_free(packed);
    if (!ok) {
        heap_caps_free(cells);
        return nullptr;
    }
    return cells;
}



// A grid of heights, `rows` by `cols` over a tile, at (fu, fv) of the way
// across and down it: between its four nearest samples, by how near each
// is - kept to the tile at its edges.
inline float bilinear(const int16_t *h, int rows, int cols, float fu, float fv) {
    float y = fv * rows - 0.5f, x = fu * cols - 0.5f;
    int r0 = (int)floorf(y), c0 = (int)floorf(x);
    float wy = y - r0, wx = x - c0;
    int r1 = std::min(rows - 1, r0 + 1), c1 = std::min(cols - 1, c0 + 1);
    r0 = std::max(0, r0);
    c0 = std::max(0, c0);
    float top = h[r0 * cols + c0] + (h[r0 * cols + c1] - h[r0 * cols + c0]) * wx;
    float bottom = h[r1 * cols + c0] + (h[r1 * cols + c1] - h[r1 * cols + c0]) * wx;
    return top + (bottom - top) * wy;
}

// The height of the land at (u, v) - in tiles, east from the window's west
// edge and south from its north - from the finest a tile holds, between
// its samples; and its level 4's cover there. Nought, and no cover, off
// the tiles.
inline float heightAtTile(float u, float v, uint8_t &cover) {
    int tu = (int)floorf(u), tv = (int)floorf(v);
    cover = 0;
    if ((unsigned)tu >= (unsigned)g_window.cols || (unsigned)tv >= (unsigned)g_window.rows) {
        return 0;
    }
    int slot = tv * g_window.cols + tu, c0 = cols0At(g_window.north + tv);
    float fu = u - tu, fv = v - tv;
    const uint8_t *coarse = g_window.coarse[slot];
    if (coarse == nullptr || c0 == 0) {
        return 0;
    }
    int rows4 = ROWS0 / 16, cols4 = c0 / 16;
    int r4 = std::min(rows4 - 1, (int)(fv * rows4)), q4 = std::min(cols4 - 1, (int)(fu * cols4));
    cover = coarse[r4 * cols4 + q4] >> 4;
    const uint8_t *mid = g_window.mid[slot];
    if (mid != nullptr) {
        return bilinear((const int16_t *)(mid + midCells(c0)), ROWS0 / 6, c0 / 6, fu, fv);
    }
    return bilinear((const int16_t *)(coarse + rows4 * cols4), rows4, cols4, fu, fv);
}

// The horizon round (lat, lon), heightM above the sea, into `out`: along
// each bearing, the land sampled every 100m from 5km out to 10km, then every
// 1% of the way, to 100km - the distances, their reciprocals and the earth's
// fall at each worked out once for all 720 - the earth's curve taken off
// with a seventh of it back for the light's bending. Some 200,000 samples:
// a few hundred milliseconds, here in the background.
void workOutHorizon(Horizon &out, double lat, double lon, float heightM) {
    constexpr int MOST = 400;
    static float far[MOST], perFar[MOST], fall[MOST];
    static int steps = 0;
    if (steps == 0) {
        constexpr float R = 6371000.0f * 7 / 6;
        for (float d = 600; d < 100000 && steps < MOST; d += std::max(100.0f, d * 0.01f)) {
            far[steps] = d;
            perFar[steps] = 1 / d;
            fall[steps] = d * d / (2 * R);
            steps++;
        }
    }
    out.lat = lat;
    out.lon = lon;
    out.heightM = heightM;
    float cosLat = (float)cos(lat * M_PI / 180);
    float u0 = (float)((lon + 180) * TILES_PER_DEG - g_window.west), v0 = (float)((90 - lat) * TILES_PER_DEG - g_window.north);
    float uPerM = TILES_PER_DEG / (111320.0f * cosLat), vPerM = -TILES_PER_DEG / 110574.0f;
    // From 5km out: a hill nearer than that, a fell close by flying low among
    // them, stood twenty degrees up as a wall over the whole sky, one colour
    // - a skyline is for the hills far enough off to be a strip along the
    // horizon. Nearer, the cover's light and shade on the ground shows them.
    static int first = -1;
    if (first < 0) {
        first = 0;
        while (first < steps - 1 && far[first] < 5000) {
            first++;
        }
    }
    // And the land nearer than that, whether it stands above the point's
    // level, for how far down among the hills the point is.
    static float nearFar[50], perNearFar[50];
    constexpr int NEAR_STEPS = 44;  // 600m to 4.9km, 100m apart
    if (nearFar[0] == 0) {
        for (int i = 0; i < NEAR_STEPS; i++) {
            nearFar[i] = 600 + 100 * i;
            perNearFar[i] = 1 / nearFar[i];
        }
    }
    int above = 0;
    for (int b = 0; b < Horizon::BEARINGS; b++) {
        float a = b * (float)(2 * M_PI / Horizon::BEARINGS), du = sinf(a) * uPerM, dv = cosf(a) * vPerM;
        for (int i = 0; i < NEAR_STEPS; i++) {
            uint8_t cover;
            if (heightAtTile(u0 + du * nearFar[i], v0 + dv * nearFar[i], cover) > heightM) {
                above++;
                break;
            }
        }
        float best = -1e9f, bestFar = far[first];
        for (int i = first; i < steps; i++) {
            uint8_t cover;
            float h = heightAtTile(u0 + du * far[i], v0 + dv * far[i], cover);
            float up = (h - heightM - fall[i]) * perFar[i];
            if (up > best) {
                best = up;
                bestFar = far[i];
            }
        }
        out.tanUp[0][b] = best;
        out.distance[0][b] = bestFar;
        if ((b & 63) == 63) {
            vTaskDelay(1);  // the core's other tasks their turn
        }
    }
    out.nearAbove = (float)above / Horizon::BEARINGS;
}

// Whether the horizon wanted is far enough from the last worked out to work
// out again - and if so, the one to work it into.
Horizon *horizonDue(double &lat, double &lon, float &heightM) {
    if (g_horizons == nullptr) {
        g_horizons = (Horizon *)heap_caps_calloc(3, sizeof(Horizon), MALLOC_CAP_SPIRAM);
        if (g_horizons == nullptr) {
            return nullptr;
        }
    }
    xSemaphoreTake(g_lock, portMAX_DELAY);
    bool wanted = g_horizonWanted;
    lat = g_horizonLat;
    lon = g_horizonLon;
    heightM = g_horizonHeight;
    xSemaphoreGive(g_lock);
    int at = g_horizonAt;
    if (!wanted) {
        return nullptr;
    }
    if (at >= 0) {
        const Horizon &last = g_horizons[at];
        double dn = (lat - last.lat) * 110574.0, de = (lon - last.lon) * 111320.0 * cos(lat * M_PI / 180);
        if (dn * dn + de * de < 300.0 * 300.0 && fabsf(heightM - last.heightM) < 30) {
            return nullptr;
        }
    }
    return &g_horizons[(at + 1) % 3];
}

// Room in `pool` for one more tile: a place free, or one freed of a tile
// taken out of the window more than a few frames ago - and, with as many in
// the window as it may hold, the one furthest from the tiles wanted - rows
// r0 to r1, columns c0 to c1, from the north pole and 180 west - taken out
// of it, unless that is one of them. The place, -1 for none yet, or -2 for
// none while every one is wanted. Under g_lock.
int room(Pool &pool, int northEdge, int r0, int r1, int c0, int c1) {
    int freeAt = -1, inWindow = 0, far = -1, farDist = -1;
    for (int i = 0; i < pool.size; i++) {
        Held &h = pool.held[i];
        if (h.slot < 0 && h.data == nullptr) {
            freeAt = freeAt < 0 ? i : freeAt;
            continue;
        }
        if (h.outFrame != 0) {
            if (g_frame - h.outFrame > 6) {
                heap_caps_free(h.data);
                h = Held();
                freeAt = freeAt < 0 ? i : freeAt;
            }
            continue;
        }
        inWindow++;
        int r = h.slot / g_cols + northEdge, c = h.slot % g_cols + g_window.west;
        int d = std::max({r0 - r, r - r1, c0 - c, c - c1, 0});
        if (d > farDist) {
            farDist = d;
            far = i;
        }
    }
    if (inWindow >= pool.most && far >= 0) {
        if (farDist == 0) {
            return -2;
        }
        (*pool.table)[pool.held[far].slot] = nullptr;
        pool.held[far].outFrame = g_frame == 0 ? 1 : g_frame;
        pool.held[far].slot = -1;
    }
    return freeAt;
}

// Reads the pack's head and index; then, as asked, tiles' levels 1 to 3, and
// near, level 0: the nearest of those wanted not yet in, one at a time, a
// yield between.
void task(void *) {
    File f;
    {
        recorder::CardLock lock;
        f = SD.open("/overhead/ground.bin", FILE_READ);
    }
    uint8_t head[32];
    bool ok = f && readAt(f, 0, head, sizeof head) && memcmp(head, "OVGR", 4) == 0 && head[5] == 0 &&
              (head[4] == 1 || head[4] == 2) && head[6] == 5;
    int rowBytes = ok && head[4] == 2 ? 28 : 20;  // version 1's without the detail block's place
    uint32_t indexAt = 0, coarseAt = 0, coarseSize = 0;
    if (ok) {
        g_south = (int16_t)(head[8] | head[9] << 8);
        g_westTile = (int16_t)(head[10] | head[11] << 8);
        g_rows = head[12] | head[13] << 8;
        g_cols = head[14] | head[15] << 8;
        g_count = (int)(head[16] | head[17] << 8 | head[18] << 16 | (uint32_t)head[19] << 24);
        indexAt = head[20] | head[21] << 8 | head[22] << 16 | (uint32_t)head[23] << 24;
        coarseAt = head[24] | head[25] << 8 | head[26] << 16 | (uint32_t)head[27] << 24;
        coarseSize = head[28] | head[29] << 8 | head[30] << 16 | (uint32_t)head[31] << 24;
        ok = g_rows > 0 && g_cols > 0 && g_count > 0 && g_count <= g_rows * g_cols;
    }
    if (ok) {
        g_entries = (Entry *)heap_caps_malloc(sizeof(Entry) * g_count, MALLOC_CAP_SPIRAM);
        g_entryAt = (int16_t *)heap_caps_malloc(sizeof(int16_t) * g_rows * g_cols, MALLOC_CAP_SPIRAM);
        g_coarse = (const uint8_t **)heap_caps_calloc(g_rows * g_cols, sizeof(uint8_t *), MALLOC_CAP_SPIRAM);
        g_mid = (const uint8_t **)heap_caps_calloc(g_rows * g_cols, sizeof(uint8_t *), MALLOC_CAP_SPIRAM);
        g_detail = (const uint8_t **)heap_caps_calloc(g_rows * g_cols, sizeof(uint8_t *), MALLOC_CAP_SPIRAM);
        for (auto &finer : g_finer) {
            finer = (const uint8_t **)heap_caps_calloc(g_rows * g_cols, sizeof(uint8_t *), MALLOC_CAP_SPIRAM);
        }
        uint8_t *rows = (uint8_t *)heap_caps_malloc(rowBytes * (size_t)g_count, MALLOC_CAP_SPIRAM);
        ok = g_entries && g_entryAt && g_coarse && g_mid && g_detail && g_finer[0] && g_finer[1] && rows;
        if (ok) {
            recorder::CardLock lock;
            ok = readAt(f, indexAt, rows, rowBytes * (size_t)g_count);
        }
        if (ok) {
            for (int i = 0; i < g_rows * g_cols; i++) {
                g_entryAt[i] = -1;
            }
            for (int i = 0; i < g_count; i++) {
                const uint8_t *r = rows + rowBytes * i;
                auto u32 = [&](int at) { return r[at] | r[at + 1] << 8 | r[at + 2] << 16 | (uint32_t)r[at + 3] << 24; };
                Entry &e = g_entries[i];
                e.row = r[0] | r[1] << 8;
                e.col = r[2] | r[3] << 8;
                e.coarseAt = u32(4);
                e.coarseSize = u32(8);
                e.midAt = u32(12);
                e.midSize = u32(16);
                e.detailAt = rowBytes == 28 ? u32(20) : 0;
                e.detailSize = rowBytes == 28 ? u32(24) : 0;
                if (e.row >= g_rows || e.col >= g_cols) {
                    ok = false;
                    break;
                }
                g_entryAt[(g_rows - 1 - e.row) * g_cols + e.col] = (int16_t)i;
            }
        }
        heap_caps_free(rows);
    }
    uint32_t t0 = millis();
    // The pack's north-west tile, from the north pole and 180 west.
    int northEdge = 90 * TILES_PER_DEG - (g_south + g_rows);
    ok = ok && loadCoarse(f, coarseAt, coarseSize, northEdge);
    if (!ok) {
        Serial.println("[ground] ground.bin isn't there, or isn't one this firmware reads - the land as it was");
        if (f) {
            recorder::CardLock lock;
            f.close();
        }
        g_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    xSemaphoreTake(g_lock, portMAX_DELAY);
    g_window.north = northEdge;
    g_window.west = g_westTile + 180 * TILES_PER_DEG;
    g_window.rows = g_rows;
    g_window.cols = g_cols;
    g_window.coarse = g_coarse;
    g_window.mid = g_mid;
    g_window.detail = g_detail;
    g_window.finer[0] = g_finer[0];
    g_window.finer[1] = g_finer[1];
    xSemaphoreGive(g_lock);
    __sync_synchronize();
    g_coarseReady = true;
    Serial.printf("[ground] ground.bin, version %d: %d tiles, every one's level 4 in %lums (%uKB)\n", head[4],
                  g_count, (unsigned long)(millis() - t0), (unsigned)(coarseSize / 1024));

    for (;;) {
        // The horizon first, when the eye has moved far enough for it.
        double hLat, hLon;
        float hHeight;
        if (Horizon *h = horizonDue(hLat, hLon, hHeight)) {
            uint32_t h0 = millis();
            workOutHorizon(*h, hLat, hLon, hHeight);
            __sync_synchronize();  // the horizon before the index to it
            g_horizonAt = (int)(h - g_horizons);
            static uint32_t loggedAt = 0;
            if (millis() - loggedAt > 30000) {
                loggedAt = millis();
                Serial.printf("[ground] the horizon round %.3f,%.3f at %.0fm, in %lums\n", hLat, hLon, hHeight,
                              (unsigned long)(millis() - h0));
            }
        }
        // The terrain's grid, about the eye.
        if (g_wantRow >= 0) {
            int wr = g_wantRow, wc = g_wantCol;
            const TerrainGrid *now = g_gridBuilt >= 0 ? &g_grids[g_gridBuilt] : nullptr;
            bool moved = now == nullptr || abs(wr - (now->north + GRID_DOWN / 2)) >= 2 ||
                         abs(wc - (now->west + GRID_ACROSS / 2)) >= 2;
            bool more = g_midsIn != g_gridMids && millis() - g_gridAt > 1000;
            if (moved || more) {
                int next = g_gridBuilt == 0 ? 1 : 0;
                TerrainGrid &G = g_grids[next];
                if (G.heights == nullptr) {
                    constexpr int ROWS = GRID_DOWN * TerrainGrid::TILE_ROWS, COLS = GRID_ACROSS * TerrainGrid::TILE_COLS;
                    constexpr int B = TerrainGrid::BLOCK;
                    G.tilesDown = GRID_DOWN;
                    G.tilesAcross = GRID_ACROSS;
                    G.rows = ROWS;
                    G.cols = COLS;
                    G.blockRows = (ROWS + B - 1) / B;
                    G.blockCols = (COLS + B - 1) / B;
                    G.heights = (int16_t *)heap_caps_malloc(sizeof(int16_t) * ROWS * COLS, MALLOC_CAP_SPIRAM);
                    G.cellMax = (int16_t *)heap_caps_malloc(sizeof(int16_t) * ROWS * COLS, MALLOC_CAP_SPIRAM);
                    G.blockMax = (int16_t *)heap_caps_malloc(sizeof(int16_t) * G.blockRows * G.blockCols,
                                                             MALLOC_CAP_SPIRAM);
                    G.wideMax = (int16_t *)heap_caps_malloc(sizeof(int16_t) * G.blockRows * G.blockCols,
                                                            MALLOC_CAP_SPIRAM);
                    if (G.heights == nullptr || G.cellMax == nullptr || G.blockMax == nullptr || G.wideMax == nullptr) {
                        heap_caps_free(G.heights);
                        heap_caps_free(G.cellMax);
                        heap_caps_free(G.blockMax);
                        heap_caps_free(G.wideMax);
                        G.heights = nullptr;
                        G.cellMax = nullptr;
                        G.blockMax = nullptr;
                        G.wideMax = nullptr;
                    }
                }
                if (G.heights != nullptr) {
                    uint32_t t0 = millis();
                    int mids = g_midsIn;
                    G.north = wr - GRID_DOWN / 2;
                    G.west = wc - GRID_ACROSS / 2;
                    fillTerrainGrid(g_window, G);
                    __sync_synchronize();  // the grid before the pointer to it
                    g_terrainGridNow = &G;
                    g_gridBuilt = next;
                    g_gridMids = mids;
                    g_gridAt = millis();
                    static uint32_t loggedAt = 0;
                    if (moved || millis() - loggedAt > 30000) {
                        loggedAt = millis();
                        Serial.printf("[ground] the terrain's heights round tile %d,%d, %d mid blocks in, in %lums; the "
                                      "highest %dm\n",
                                      wr, wc, mids, (unsigned long)(millis() - t0), G.highest);
                    }
                }
            }
        }
        // The nearest tile wanted with something not in, a ring at a time
        // outwards: its levels 1 to 3, then - a ring or less from the eye's -
        // its level 0, then - one of the two by two nearest the eye - its
        // levels -1 and -2, worked out from its level 0. Those only with room
        // to spare in the PSRAM, a few megabytes, for everything else.
        int wr = g_wantRow, wc = g_wantCol, radius = g_wantRadius, br = g_blockRow, bc = g_blockCol;
        int best = -1, bestSlot = -1, bestCols0 = 0, bestRow = 0, kind = 0;  // kind: 0 mid, 1 detail, 2 and 3 finer
        bool roomy = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > 6u << 20;
        int finer = g_finerWanted;
        if (finer < 2) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            retireAll(g_finerPool[1]);
            if (finer < 1) {
                retireAll(g_finerPool[0]);
            }
            xSemaphoreGive(g_lock);
        }
        for (int ring = 0; wr >= 0 && ring <= radius && best < 0; ring++) {
            for (int pass = 0; pass < 4 && best < 0 && (pass == 0 || ring <= DETAIL_RING); pass++) {
                if (pass >= 2 && (!roomy || pass - 2 >= finer)) {
                    break;
                }
                const uint8_t **table = pass == 0 ? g_mid : pass == 1 ? g_detail : g_finer[pass - 2];
                for (int dr = -ring; dr <= ring && best < 0; dr++) {
                    for (int dc = -ring; dc <= ring && best < 0; dc++) {
                        if (std::max(abs(dr), abs(dc)) != ring) {
                            continue;
                        }
                        int r = wr + dr - northEdge, c = wc + dc - g_window.west;
                        if (r < 0 || r >= g_rows || c < 0 || c >= g_cols) {
                            continue;
                        }
                        int slot = r * g_cols + c, e = g_entryAt[slot];
                        if (e < 0 || table[slot] != nullptr) {
                            continue;
                        }
                        if (pass == 1 && g_entries[e].detailSize == 0) {
                            continue;
                        }
                        if (pass >= 2 && (g_detail[slot] == nullptr || wr + dr < br || wr + dr > br + 1 ||
                                          wc + dc < bc || wc + dc > bc + 1)) {
                            continue;
                        }
                        best = e;
                        bestSlot = slot;
                        bestRow = r;
                        bestCols0 = cols0At(northEdge + r);
                        kind = pass;
                    }
                }
            }
        }
        if (best < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // Room for it: the held tile furthest from what is wanted, out of the
        // window, and freed once no band can still be drawing from it.
        Pool &pool = kind == 0 ? g_midPool : kind == 1 ? g_detailPool : g_finerPool[kind - 2];
        xSemaphoreTake(g_lock, portMAX_DELAY);
        int at = kind == 0   ? room(pool, northEdge, wr - radius, wr + radius, wc - radius, wc + radius)
                 : kind == 1 ? room(pool, northEdge, wr - DETAIL_RING, wr + DETAIL_RING, wc - DETAIL_RING,
                                    wc + DETAIL_RING)
                             : room(pool, northEdge, br, br + 1, bc, bc + 1);
        xSemaphoreGive(g_lock);
        if (at < 0) {
            vTaskDelay(pdMS_TO_TICKS(at == -2 ? 100 : 50));  // -2: all wanted already, none to make room with
            continue;
        }
        const Entry &e = g_entries[best];
        uint8_t *cells = nullptr;
        if (kind >= 2) {
            uint32_t t0 = micros();
            cells = finerLevel(g_detail[bestSlot], bestCols0, northEdge + bestRow, g_window.west + bestSlot % g_cols,
                               -1 - (kind - 2));
            g_finerUs += micros() - t0;
            Serial.printf("[ground] level %d of tile %d,%d worked out in %lums%s\n", -1 - (kind - 2),
                          northEdge + bestRow, g_window.west + bestSlot % g_cols, (unsigned long)((micros() - t0) / 1000),
                          cells == nullptr ? " - no room for it" : "");
        } else {
            cells = kind == 1 ? loadBlock(f, e.detailAt, e.detailSize, detailBytes(bestCols0))
                              : loadBlock(f, e.midAt, e.midSize, midBytes(bestCols0));
        }
        xSemaphoreTake(g_lock, portMAX_DELAY);
        bool kept = cells != nullptr && pool.held[at].data == nullptr;
        if (kept) {
            pool.held[at].slot = bestSlot;
            pool.held[at].data = cells;
            pool.held[at].outFrame = 0;
            __sync_synchronize();  // the cells before the pointer to them
            (*pool.table)[bestSlot] = cells;
            if (kind == 0) {
                g_midsIn = g_midsIn + 1;
            }
        } else {
            heap_caps_free(cells);
        }
        xSemaphoreGive(g_lock);
        if (!kept) {
            vTaskDelay(pdMS_TO_TICKS(500));  // no room, or the card failing: not round again at once
        }
        vTaskDelay(1);  // the card's reads busy-wait: the core's others their turn
    }
}
#endif

}  // namespace

// The terrain grid's heights for one tile, from the window: its mid block's
// every 15 seconds, spread across to 32 columns where it has another
// number of them; else its coarse heights spread over the whole; else
// nought - written into `out`, `stride` to a row.
static void terrainTile(const Window &W, int northRow, int westCol, int16_t *out, int stride) {
    constexpr int TR = TerrainGrid::TILE_ROWS, TC = TerrainGrid::TILE_COLS;
    int r = northRow - W.north, c = westCol - W.west, c0 = cols0At(northRow);
    const uint8_t *mid = nullptr, *coarse = nullptr;
    if (r >= 0 && r < W.rows && c >= 0 && c < W.cols && c0 > 0) {
        mid = W.mid[r * W.cols + c];
        coarse = W.coarse[r * W.cols + c];
    }
    if (mid != nullptr) {
        const int16_t *hs = (const int16_t *)(mid + midHeightsAt(c0));
        int cols = c0 / 6;  // rows: ROWS0 / 6, 48, as the grid's
        for (int y = 0; y < TR; y++) {
            const int16_t *row = hs + y * cols;
            int16_t *o = out + y * stride;
            if (cols == TC) {
                memcpy(o, row, TC * sizeof(int16_t));
                continue;
            }
            for (int x = 0; x < TC; x++) {
                float q = (x + 0.5f) * cols / TC - 0.5f;
                int q0 = std::max(0, std::min(cols - 1, (int)floorf(q))), q1 = std::min(cols - 1, q0 + 1);
                float t = std::max(0.0f, std::min(1.0f, q - q0));
                o[x] = (int16_t)lroundf(row[q0] + (row[q1] - row[q0]) * t);
            }
        }
        return;
    }
    if (coarse != nullptr) {
        int rows = ROWS0 / 16, cols = c0 / 16;
        const int16_t *hs = (const int16_t *)(coarse + rows * cols);
        for (int y = 0; y < TR; y++) {
            float p = (y + 0.5f) * rows / TR - 0.5f;
            int p0 = std::max(0, std::min(rows - 1, (int)floorf(p))), p1 = std::min(rows - 1, p0 + 1);
            float ty = std::max(0.0f, std::min(1.0f, p - p0));
            int16_t *o = out + y * stride;
            for (int x = 0; x < TC; x++) {
                float q = (x + 0.5f) * cols / TC - 0.5f;
                int q0 = std::max(0, std::min(cols - 1, (int)floorf(q))), q1 = std::min(cols - 1, q0 + 1);
                float tx = std::max(0.0f, std::min(1.0f, q - q0));
                float top = hs[p0 * cols + q0] + (hs[p0 * cols + q1] - hs[p0 * cols + q0]) * tx;
                float bottom = hs[p1 * cols + q0] + (hs[p1 * cols + q1] - hs[p1 * cols + q0]) * tx;
                o[x] = (int16_t)lroundf(top + (bottom - top) * ty);
            }
        }
        return;
    }
    for (int y = 0; y < TR; y++) {
        memset(out + y * stride, 0, TC * sizeof(int16_t));
    }
}

void fillTerrainGrid(const Window &W, TerrainGrid &grid) {
    constexpr int TR = TerrainGrid::TILE_ROWS, TC = TerrainGrid::TILE_COLS, B = TerrainGrid::BLOCK;
    for (int r = 0; r < grid.tilesDown; r++) {
        for (int c = 0; c < grid.tilesAcross; c++) {
            terrainTile(W, grid.north + r, grid.west + c, grid.heights + (size_t)r * TR * grid.cols + c * TC, grid.cols);
        }
    }
    // Each cell's highest corner.
    for (int y = 0; y < grid.rows; y++) {
        const int16_t *row = grid.heights + (size_t)y * grid.cols, *below = y + 1 < grid.rows ? row + grid.cols : row;
        int16_t *out = grid.cellMax + (size_t)y * grid.cols;
        for (int x = 0; x < grid.cols; x++) {
            int x1 = std::min(grid.cols - 1, x + 1);
            out[x] = std::max(std::max(row[x], row[x1]), std::max(below[x], below[x1]));
        }
    }
    // Each block's highest, over its cells and the row and column after.
    int16_t most = 0;
    for (int br = 0; br < grid.blockRows; br++) {
        for (int bc = 0; bc < grid.blockCols; bc++) {
            int16_t m = -32768;
            int y1 = std::min(grid.rows, br * B + B + 1), x1 = std::min(grid.cols, bc * B + B + 1);
            for (int y = br * B; y < y1; y++) {
                const int16_t *row = grid.heights + (size_t)y * grid.cols;
                for (int x = bc * B; x < x1; x++) {
                    m = std::max(m, row[x]);
                }
            }
            grid.blockMax[br * grid.blockCols + bc] = m;
            most = std::max(most, m);
        }
    }
    grid.highest = most;
    // And each block's with those round it: the highest a ray going by it
    // could meet, sampled every half a block.
    for (int br = 0; br < grid.blockRows; br++) {
        for (int bc = 0; bc < grid.blockCols; bc++) {
            int16_t m = -32768;
            for (int r = std::max(0, br - 1); r <= std::min(grid.blockRows - 1, br + 1); r++) {
                for (int c = std::max(0, bc - 1); c <= std::min(grid.blockCols - 1, bc + 1); c++) {
                    m = std::max(m, grid.blockMax[r * grid.blockCols + c]);
                }
            }
            grid.wideMax[br * grid.blockCols + bc] = m;
        }
    }
}

// Level `level`, -1 or -2, of a tile from its level 0, `detail` - in parts,
// as the window has them: each of level 0's cells 2 or 4 of them each way.
//  - What covers each: level 0's cell nearest a place moved off it by a
//    noise smooth over a cell or so - a field's edge, a wood's, the shore,
//    wandering as they do, not in steps of 75m.
//  - Its light and shade: between level 0's four nearest - the hills
//    smooth, not in 75m facets - to the nearest of the shade's sixteen
//    levels. Not roughened, nor dithered, cell by cell: these levels are
//    the ones drawn nearest the eye, their cells there tens of pixels
//    across, and a cell a shade lighter or darker than the next, which a
//    few pixels away is texture, there is a checkerboard.
// The noise is from where each cell is in the world: the same each time it
// is worked out, and either side of where tiles meet. `northRow` and
// `westCol` the tile's, in tiles from the north pole and from 180 west. In
// the PSRAM, or nullptr where there is no room.
uint8_t *finerLevel(const uint8_t *detail, int cols0, int northRow, int westCol, int level) {
    const int sh = -level, s = 1 << sh, parts = 2 << sh;
    const int pr = ROWS0 / 2, pc = cols0 / 2;  // a part's rows and columns
    const int rows = ROWS0 * s, cols = cols0 * s, bw = cols0 + 2;
    // Level 0 whole, and a cell more all round - the edges' own again.
    uint8_t *whole = (uint8_t *)cellMemory((size_t)(ROWS0 + 2) * bw);
    // Along a row of the cells being worked out, interpolated once for the
    // s rows that share it - bilinear's two ways apart: the noise's random
    // pushes at the corners of level 0's cells, along the corners' row above
    // and below; and level 0's shade along its rows above, at and below.
    float *lines = (float *)cellMemory(sizeof(float) * cols * 7);
    uint8_t *out = (uint8_t *)cellMemory((size_t)rows * cols);
    if (whole == nullptr || lines == nullptr || out == nullptr) {
        freeCells(whole);
        freeCells(lines);
        freeCells(out);
        return nullptr;
    }
    for (int r = -1; r <= ROWS0; r++) {
        int rr = std::min(ROWS0 - 1, std::max(0, r));
        uint8_t *line = whole + (size_t)(r + 1) * bw;
        for (int c = -1; c <= cols0; c++) {
            int cc = std::min(cols0 - 1, std::max(0, c));
            line[c + 1] = detail[(size_t)((rr >= pr) * 2 + (cc >= pc)) * pr * pc + (rr % pr) * pc + cc % pc];
        }
    }
    // Along a level 0 cell, for each of its s: where its middle is, as a
    // share of the way across, eased for the noise; and for the shade, the
    // neighbour it is between it and - before or after - and how far on.
    float at[4], eased[4], toward[4];
    int before[4];
    for (int a = 0; a < s; a++) {
        at[a] = (a + 0.5f) / s;
        eased[a] = at[a] * at[a] * (3 - 2 * at[a]);
        before[a] = at[a] < 0.5f ? -1 : 0;
        toward[a] = at[a] - 0.5f - before[a];
    }
    constexpr float WANDER = 0.45f;  // how far an edge wanders, in level 0's cells
    const uint32_t gRow = (uint32_t)northRow * ROWS0, gCol = (uint32_t)westCol * cols0;
    float *pushes[2] = {lines, lines + 2 * cols}, *shades[3] = {lines + 4 * cols, lines + 5 * cols, lines + 6 * cols};
    // The pushes along corner row r, to every column being worked out.
    auto pushRow = [&](float *line, int r) {
        uint32_t h1 = cellHash(gRow + r, gCol, 0x51ED270Bu);
        for (int C = 0; C < cols0; C++) {
            uint32_t h0 = h1;
            h1 = cellHash(gRow + r, gCol + C + 1, 0x51ED270Bu);
            float x0 = (int32_t)(h0 & 0xFFFF) * (2.0f / 65536) - 1, y0 = (int32_t)(h0 >> 16) * (2.0f / 65536) - 1;
            float x1 = (int32_t)(h1 & 0xFFFF) * (2.0f / 65536) - 1, y1 = (int32_t)(h1 >> 16) * (2.0f / 65536) - 1;
            for (int b = 0; b < s; b++) {
                float *o = line + 2 * (C * s + b);
                o[0] = x0 + (x1 - x0) * eased[b];
                o[1] = y0 + (y1 - y0) * eased[b];
            }
        }
    };
    // Level 0's shade along its row r, from -1 to ROWS0, to every column.
    auto shadeRow = [&](float *line, int r) {
        const uint8_t *src = whole + (size_t)(r + 1) * bw + 1;
        for (int C = 0; C < cols0; C++) {
            for (int b = 0; b < s; b++) {
                int c0 = C + before[b];
                float v0 = src[c0] & 15, v1 = src[c0 + 1] & 15;
                line[C * s + b] = v0 + (v1 - v0) * toward[b];
            }
        }
    };
    pushRow(pushes[1], 0);
    shadeRow(shades[1], -1);
    shadeRow(shades[2], 0);
    for (int R = 0; R < ROWS0; R++) {
        std::swap(pushes[0], pushes[1]);
        pushRow(pushes[1], R + 1);
        float *oldest = shades[0];
        shades[0] = shades[1];
        shades[1] = shades[2];
        shades[2] = oldest;
        shadeRow(shades[2], R + 1);
#ifdef ESP_PLATFORM
        if ((R & 7) == 7) {
            // A breath every eight rows - a tick - the work spread out, not a
            // burst of a megabyte through the cache the drawing shares.
            vTaskDelay(1);
        }
#endif
        const uint8_t *here = whole + (size_t)(R + 1) * bw + 1;  // row R, from column 0
        for (int a = 0; a < s; a++) {
            int i = R * s + a;
            uint8_t *dst = out + (size_t)(i / pr) * parts * pr * pc + (size_t)(i % pr) * pc;
            const float *upper = pushes[0], *lower = pushes[1];
            const float *s0 = shades[1 + before[a]], *s1 = shades[2 + before[a]];
            float ty = toward[a], ey = eased[a], y = at[a];
            for (int j = 0, jj = 0; j < cols; j++) {
                // Where it is moved to, and what covers that.
                float wy = upper[2 * j + 1] + (lower[2 * j + 1] - upper[2 * j + 1]) * ey;
                float wx = upper[2 * j] + (lower[2 * j] - upper[2 * j]) * ey;
                float my = y + WANDER * wy, mx = at[j & (s - 1)] + WANDER * wx;
                int dr = my < 0 ? -bw : my >= 1 ? bw : 0, dc = mx < 0 ? -1 : mx >= 1 ? 1 : 0;
                int cover = here[dr + (j >> sh) + dc] >> 4;
                // Its shade, between the four nearest middles.
                float shade = s0[j] + (s1[j] - s0[j]) * ty + 0.5f;
                int q = std::min(15, std::max(0, (int)shade));
                *dst++ = (uint8_t)(cover << 4 | q);
                if (++jj == pc) {
                    jj = 0;
                    dst += (size_t)(pr - 1) * pc;  // on to the next part's
                }
            }
        }
    }
    freeCells(whole);
    freeCells(lines);
    return out;
}

bool open() {
#ifdef ESP_PLATFORM
    if (g_open) {
        return true;
    }
    if (!recorder::mount()) {
        return false;
    }
    {
        recorder::CardLock lock;
        if (!SD.exists("/overhead/ground.bin")) {
            return false;
        }
    }
    if (g_lock == nullptr) {
        g_lock = xSemaphoreCreateMutex();
    }
    g_open = xTaskCreatePinnedToCore(task, "ground", 5120, nullptr, 1, &g_task, 0) == pdPASS;
    return g_open;
#else
    return false;
#endif
}

void close() {
    // Held for the session: reading every tile's level 4 again is half a
    // second each time the view opens, and the memory is the PSRAM's.
}

bool isOpen() { return g_open && g_coarseReady; }

bool want(double lat, double lon, float fineKm, float upM) {
    if (!g_coarseReady) {
        return false;
    }
    g_frame++;
    double v = (90.0 - lat) * TILES_PER_DEG, u = (lon + 180.0) * TILES_PER_DEG;
    int row = (int)floor(v), col = (int)floor(u);
    float tileKm = 111.32f / TILES_PER_DEG;
    int radius = std::max(0, std::min(4, (int)ceilf(fineKm / tileKm)));
    // The two by two tiles with the eye nearest their middle - moved on only
    // once it is within a third of a tile of their edge, not to and fro as
    // it crosses a tile's middle.
    int br = g_blockRow, bc = g_blockCol;
    if (br < 0 || v < br + 0.3 || v > br + 1.7) {
        br = (int)floor(v - 0.5);
    }
    if (bc < 0 || u < bc + 0.3 || u > bc + 1.7) {
        bc = (int)floor(u - 0.5);
    }
    g_blockRow = br;
    g_blockCol = bc;
    g_finerWanted = upM < 4000 ? 2 : upM < 8000 ? 1 : 0;
    g_wantRadius = radius;
    g_wantCol = col;
    g_wantRow = row;
    return true;
}

const Window &window() {
    static const Window none;
    return g_coarseReady ? g_window : none;
}

float heightAt(double lat, double lon) {
    if (!g_coarseReady) {
        return 0;
    }
    double v = (90.0 - lat) * TILES_PER_DEG, u = (lon + 180.0) * TILES_PER_DEG;
    int r = (int)floor(v) - g_window.north, c = (int)floor(u) - g_window.west;
    if (r < 0 || r >= g_window.rows || c < 0 || c >= g_window.cols) {
        return 0;
    }
    int slot = r * g_window.cols + c, c0 = cols0At(r + g_window.north);
    float fv = (float)(v - floor(v)), fu = (float)(u - floor(u));
    const uint8_t *mid = g_window.mid[slot];
    if (mid != nullptr) {
        int rows = ROWS0 / 6, cols = c0 / 6;
        const int16_t *h = (const int16_t *)(mid + midCells(c0));
        return h[std::min(rows - 1, (int)(fv * rows)) * cols + std::min(cols - 1, (int)(fu * cols))];
    }
    const uint8_t *coarse = g_window.coarse[slot];
    if (coarse != nullptr) {
        int rows = ROWS0 / 16, cols = c0 / 16;
        const int16_t *h = (const int16_t *)(coarse + rows * cols);
        return h[std::min(rows - 1, (int)(fv * rows)) * cols + std::min(cols - 1, (int)(fu * cols))];
    }
    return 0;
}

void wantHorizon(double lat, double lon, float heightM) {
#ifdef ESP_PLATFORM
    if (!g_coarseReady || g_lock == nullptr) {
        return;
    }
    xSemaphoreTake(g_lock, portMAX_DELAY);
    g_horizonWanted = true;
    g_horizonLat = lat;
    g_horizonLon = lon;
    g_horizonHeight = heightM;
    xSemaphoreGive(g_lock);
#else
    (void)lat;
    (void)lon;
    (void)heightM;
#endif
}

const TerrainGrid *terrainGrid() { return g_terrainGridNow; }

const Horizon *horizon() {
    int at = g_horizonAt;
    return at >= 0 && g_horizons != nullptr ? &g_horizons[at] : nullptr;
}

void stats(int &midHeld, int &detailHeld, int &finerHeld, int &reads, uint32_t &readUs, uint32_t &finerUs) {
    midHeld = detailHeld = finerHeld = 0;
    for (const Held &h : g_midHeld) {
        midHeld += h.slot >= 0;
    }
    for (const Held &h : g_detailHeld) {
        detailHeld += h.slot >= 0;
    }
    for (const Held &h : g_finerHeld[1]) {
        finerHeld += h.slot >= 0;
    }
    reads = g_reads;
    readUs = g_readUs;
    finerUs = g_finerUs;
}

}  // namespace groundpack
