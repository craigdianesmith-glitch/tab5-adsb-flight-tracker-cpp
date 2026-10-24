#!/usr/bin/env python3
"""Builds ground.bin, the 3D view's ground pack: what covers the land - trees,
fields, moor, towns, water - and the hills' light and shade, for the 3D view
to stamp onto the ground it draws, and their heights for the land it draws
at its height.

    tools/gen_ground.py OUT.bin [--south 49.8 --north 61 --west -10.8 --east 2] [--jobs 8]

This is the pack the releases carry, the United Kingdom and Ireland, made in
one go over a box. For anywhere else - a country, a state, the country round
a town - tools/make_ground_pack.py makes one by name, with these same tiles:
it imports them from here.

Needs numpy and rasterio (pip install numpy rasterio), and reads straight
from the datasets' open archives on AWS, a window at a time - nothing to
download first:

  ESA WorldCover 2021 v200, 10m land cover - (c) ESA WorldCover project 2021 /
  Contains modified Copernicus Sentinel data (2021) processed by ESA
  WorldCover consortium, CC BY 4.0;
  Copernicus DEM GLO-30 and GLO-90 - (c) DLR e.V. 2010-2014 and (c) Airbus
  Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European
  Union and ESA; all rights reserved.

The pack - all numbers little-endian:

  header, 32 bytes: "OVGR", version (u16) 2, tiles a degree (u16) 5, the
  south and west edges in tiles (i16 each), rows and columns of tiles
  (u16 each), the tile count (u32), the index's offset (u32), where the
  coarse blocks start and how long they are together (u32 each)
  index: a row for each tile there is, by latitude then longitude - its row
  and column (u16 each), the offsets and lengths of its coarse, mid and
  detail blocks (u32 each)
  every tile's coarse block, one after another, in the index's order - to
  be read in one go - then every tile's mid block, then every tile's detail
  block; each zlib's deflate without a header (raw, as the ESP32's ROM
  inflates it).

A tile is 0.2 degrees square. Its cells are 2.5 seconds of latitude - some
75m - and 2.5 times k of longitude - k 1 below 50 degrees, 1.5 to 60, 2 to
70, 3 to 75, nothing past that - so they stay near square: 288 rows of
288/k. A cell is a byte: its cover, as below, in the top four bits, and in
the bottom four its shade, the hills lit from the north-west at 45 degrees -
11 for flat ground, down to 0 facing away and up to 15 facing the light,
from the GLO-30's heights. There are five levels of them, 0 to 4, each a
cell of every 2 by 2 of the one before: its cover from the shares of
WorldCover's there - sixteen samples of it in each cell of level 0, and so
4^L times sixteen in a cell of level L - and its shade their mean.

A tile's detail block is level 0, in quarters - north-west, north-east,
south-west, south-east - each 144 rows of 144/k: the 3D view reads it only
for the tiles round the eye. Its mid block is levels 1, 2 and 3, and its
heights every 15 seconds (48 rows of 48/k, i16 metres, from the GLO-90),
read for the tiles within some 45km. Its coarse block is level 4, 18 rows of
18/k, and the heights at the same cells - small enough for the 3D view to
hold every tile's, for the country far off and its heights.

Version 1 was version 2 without the detail blocks - its levels 0 to 3 are
version 2's 1 to 4 - the index's rows 20 bytes, without the detail block's
offset and length; the firmware reads both.

The covers: 0 none (the sea, or none given), 1 trees, 2 shrubs, 3 grass,
4 crops, 5 built over, 6 bare, 7 snow and ice, 8 water, 9 marsh, 10
mangroves, 11 moss and lichen - each WorldCover's own, where it has most of
the cell - and, mixed, 12 suburb, a fifth to three fifths of it built over,
gardens, trees and grass the rest; 13 town, nine tenths and more built
over; 14 moor, grass up where the hills' rough grazing is - above a line
from 400m on the south coast to 150m in Shetland, ragged by up to 100m
either side of it in patches some 600m across, as a head dyke is, or on
slopes steeper than 22 degrees - which WorldCover counts as grass, as it does the lowland's
pasture; 15 rocky, a fifth or more of it bare rock and scree, but not most.
The moor's line is the British Isles' own: tools/make_ground_pack.py draws
it there only, and leaves grass as grass elsewhere.
"""

import argparse
import math
import multiprocessing
import struct
import sys
import time
import zlib

import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.merge import merge
from rasterio.windows import from_bounds

TILES_PER_DEG = 5
TILE_DEG = 1.0 / TILES_PER_DEG
ROWS0 = 288  # 2.5 seconds of latitude a cell
LEVELS = 5
SUB = 4  # WorldCover's samples along a cell of level 0
WORLDCOVER = ("/vsicurl/https://esa-worldcover.s3.eu-central-1.amazonaws.com/v200/2021/map/"
              "ESA_WorldCover_10m_2021_v200_{ns}{lat:02d}{ew}{lon:03d}_Map.tif")
DEM90 = ("/vsicurl/https://copernicus-dem-90m.s3.amazonaws.com/Copernicus_DSM_COG_30_{ns}{lat:02d}_00_{ew}{lon:03d}_00_DEM/"
         "Copernicus_DSM_COG_30_{ns}{lat:02d}_00_{ew}{lon:03d}_00_DEM.tif")
DEM30 = ("/vsicurl/https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_{ns}{lat:02d}_00_{ew}{lon:03d}_00_DEM/"
         "Copernicus_DSM_COG_10_{ns}{lat:02d}_00_{ew}{lon:03d}_00_DEM.tif")
NONE, TREES, SHRUBS, GRASS, CROPS, BUILT, BARE, SNOW, WATER, MARSH, MANGROVES, MOSS, SUBURB, TOWN, MOOR, ROCKY = \
    range(16)
WORLDCOVER_CLASSES = 12  # the covers WorldCover has: NONE to MOSS
# WorldCover's classes to the pack's.
COVER = {10: TREES, 20: SHRUBS, 30: GRASS, 40: CROPS, 50: BUILT, 60: BARE, 70: SNOW, 80: WATER, 90: MARSH,
         95: MANGROVES, 100: MOSS}
LOOKUP = np.zeros(256, np.uint8)
for _wc, _code in COVER.items():
    LOOKUP[_wc] = _code
# A tile's bytes as this file makes them: one more each time a change here
# changes them, so that tools/make_ground_pack.py doesn't take a tile it built
# before - and kept - for one built now.
TILE_VERSION = 1
# Every tile the archives have - the paths dataset() is asked for, as
# WORLDCOVER, DEM30 and DEM90 make them - or None, as here, to find out by
# trying each one. tools/make_ground_pack.py sets it from the archives' own
# lists of their tiles: then a tile not on its list is the sea, with nothing
# asked of the network, and one on it that can't be read is the network
# failing - an error - not the sea.
KNOWN = None


def k_for(lat):
    a = abs(lat)
    return 1 if a < 50 else 1.5 if a < 60 else 2 if a < 70 else 3 if a < 75 else None


def ns_ew(lat, lon):
    return ("N" if lat >= 0 else "S", abs(lat), "E" if lon >= 0 else "W", abs(lon))


_open = {}


def dataset(path):
    """The archive's tile at `path`, open - or None where it has none: kept
    open, the 24 used last, the one used longest ago closed for another."""
    if path in _open:
        _open[path] = _open.pop(path)  # the newest now
        return _open[path]
    while len(_open) >= 24:
        oldest = next(iter(_open))
        if _open[oldest] is not None:
            _open[oldest].close()
        del _open[oldest]
    if KNOWN is not None and path not in KNOWN:
        _open[path] = None  # not on the archive's list: the sea
        return None
    try:
        _open[path] = rasterio.open(path)
    except rasterio.errors.RasterioIOError:
        if KNOWN is not None:
            raise  # one the archive has, not read
        _open[path] = None  # the sea: the archive has no tile there
    return _open[path]


def cover_counts(south, west, rows, cols):
    """How many of each cell's SUB by SUB samples of WorldCover are each of the
    pack's covers NONE to MOSS: (rows, cols, 12) - all NONE where there is no
    WorldCover tile."""
    counts = np.zeros((rows, cols, WORLDCOVER_CLASSES), np.uint8)
    lat3 = int(math.floor(south / 3)) * 3
    lon3 = int(math.floor(west / 3)) * 3
    ns, la, ew, lo = ns_ew(lat3, lon3)
    ds = dataset(WORLDCOVER.format(ns=ns, lat=la, ew=ew, lon=lo))
    if ds is None:
        counts[:, :, NONE] = SUB * SUB
        return counts
    win = from_bounds(west, south, west + TILE_DEG, south + TILE_DEG, ds.transform)
    a = ds.read(1, window=win, out_shape=(rows * SUB, cols * SUB), resampling=Resampling.nearest, boundless=True,
                fill_value=0)
    codes = LOOKUP[a].reshape(rows, SUB, cols, SUB)
    for code in range(WORLDCOVER_CLASSES):
        counts[:, :, code] = (codes == code).sum(axis=(1, 3))
    return counts


def heights(south, west, rows, cols, border=0, template=DEM90, resampling=Resampling.bilinear, fallback=None,
            used=None):
    """Heights over a tile, metres - on `rows` by `cols` cells and `border`
    more round them, for the slopes at its edges, from the neighbouring
    tiles of the archive where the border crosses into them; nought where
    the sea is. Where `template`'s archive has no tile, `fallback`'s, if
    given and it has one - a degree it stood in for added to `used`."""
    dy, dx = TILE_DEG / rows, TILE_DEG / cols
    s, w = south - border * dy, west - border * dx
    n, e = south + TILE_DEG + border * dy, west + TILE_DEG + border * dx
    shape = (rows + 2 * border, cols + 2 * border)
    sources = []
    for la in range(int(math.floor(s)), int(math.floor(n - 1e-9)) + 1):
        for lo in range(int(math.floor(w)), int(math.floor(e - 1e-9)) + 1):
            ns, a, ew, o = ns_ew(la, lo)
            ds = dataset(template.format(ns=ns, lat=a, ew=ew, lon=o))
            if ds is None and fallback is not None:
                ds = dataset(fallback.format(ns=ns, lat=a, ew=ew, lon=o))
                if ds is not None and used is not None:
                    used.append((la, lo))
            if ds is not None:
                sources.append(ds)
    if not sources:
        return np.zeros(shape, np.float32)
    if len(sources) == 1:
        win = from_bounds(w, s, e, n, sources[0].transform)
        z = sources[0].read(1, window=win, out_shape=shape, resampling=resampling, boundless=True, fill_value=0)
    else:
        # Merged over a few cells more each way, which are cut off again: at
        # its own edges the merge leaves nought, which the shading took for a
        # cliff along every tile beside a degree's line.
        m = 4
        z = merge(sources, bounds=(w - m * dx, s - m * dy, e + m * dx, n + m * dy), res=(dx, dy),
                  resampling=resampling, nodata=0)[0][0]
        z = z[m:m + shape[0], m:m + shape[1]]
        if z.shape != shape:
            z = np.pad(z, ((0, shape[0] - z.shape[0]), (0, shape[1] - z.shape[1])), mode="edge")
    return np.maximum(z.astype(np.float32), 0)


def shade(z, cell_y, cell_x):
    """Horn's hillshade, lit from the north-west at 45 degrees: the ground's
    normal against the light, 0 to 15 - 11 for flat ground, as a float, to
    be averaged - and how steep it is, the tangent. `z` has a cell's border
    all round, which the answers don't; rows run north to south."""
    p = z
    east = ((p[:-2, 2:] + 2 * p[1:-1, 2:] + p[2:, 2:]) - (p[:-2, :-2] + 2 * p[1:-1, :-2] + p[2:, :-2])) / (8 * cell_x)
    south = ((p[2:, :-2] + 2 * p[2:, 1:-1] + p[2:, 2:]) - (p[:-2, :-2] + 2 * p[:-2, 1:-1] + p[:-2, 2:])) / (8 * cell_y)
    north = -south
    # The light, east-north-up: from azimuth 315 at 45 degrees up.
    lx, ly, lz = -0.5, 0.5, math.sqrt(0.5)
    hs = (-east * lx - north * ly + lz) / np.sqrt(east * east + north * north + 1)
    return np.clip(hs, 0, 1) * 15.5, np.sqrt(east * east + north * north)


def blocks(a, f):
    """`a`, (rows, cols, ...), summed over each f by f block of cells."""
    r, c = a.shape[0] // f, a.shape[1] // f
    return a.reshape(r, f, c, f, *a.shape[2:]).sum(axis=(1, 3))


def lattice_hash(r, c):
    """A number from 0 to 1 for each point (r, c) of a lattice - uint32 arrays,
    broadcast together."""
    h = (r * np.uint32(2654435761)) ^ (c * np.uint32(2246822519))
    h ^= h >> np.uint32(15)
    h *= np.uint32(2654435761)
    h ^= h >> np.uint32(13)
    return (h & np.uint32(0xFFFF)).astype(np.float32) / 65536


def ragged(r0, c0, rows, cols, f):
    """A number from 0 to 1 for each cell, wandering smoothly from one to the
    next over some eight cells of level 0 - 600m - with a little of each
    cell's own; the same for a cell whichever tile asks. `r0` and `c0` the
    first's row and column across the pack's whole grid at its level, `f`
    its cells' size in level 0's."""
    y = ((np.arange(rows) + r0) * f + f / 2) / 8.0
    x = ((np.arange(cols) + c0) * f + f / 2) / 8.0
    iy, ix = np.floor(y), np.floor(x)
    fy, fx = y - iy, x - ix
    fy, fx = (fy * fy * (3 - 2 * fy))[:, None], (fx * fx * (3 - 2 * fx))[None, :]
    iy, ix = iy.astype(np.uint32)[:, None], ix.astype(np.uint32)[None, :]
    one = np.uint32(1)
    smooth = ((lattice_hash(iy, ix) * (1 - fx) + lattice_hash(iy, ix + one) * fx) * (1 - fy) +
              (lattice_hash(iy + one, ix) * (1 - fx) + lattice_hash(iy + one, ix + one) * fx) * fy)
    r = (np.arange(rows, dtype=np.uint32) + np.uint32(r0))[:, None]
    c = (np.arange(cols, dtype=np.uint32) + np.uint32(c0))[None, :]
    own = lattice_hash(r ^ np.uint32(0x5BD1E995), c ^ np.uint32(0x1B873593))
    return 0.75 * smooth + 0.25 * own


def classify(counts, z, steep, moorline, dither):
    """Each cell's cover - see the top - from how many of its samples are
    each of WorldCover's: the most of them, or the mixes. The moor's line,
    `moorline`, up or down by `dither`, 0 to 1, each cell's - or no moor at
    all, the grass left grass, with `moorline` None."""
    n = np.maximum(counts.sum(axis=2), 1).astype(np.float32)
    most = counts.astype(np.float32)
    most[:, :, NONE] -= 0.5  # nought, none, loses to anything - ties to the lower code
    out = most.argmax(axis=2).astype(np.uint8)
    built, bare = counts[:, :, BUILT] / n, counts[:, :, BARE] / n
    land = (out != NONE) & (out != WATER) & (out != SNOW)
    if moorline is not None:
        up = z >= moorline + (dither - 0.5) * 200
        out[land & (out == GRASS) & (up | (steep >= 0.4))] = MOOR
    out[land & (bare >= 0.2) & (out != BARE)] = ROCKY
    out[land & (built >= 0.2)] = SUBURB
    out[land & (built >= 0.6)] = BUILT
    out[land & (built >= 0.9)] = TOWN
    return out


def deflate(data):
    raw = zlib.compressobj(9, zlib.DEFLATED, -15)
    return raw.compress(data) + raw.flush()


def make_tile(ty, tx, moor=True, dem_fallback=False):
    """The coarse, mid and detail blocks of the tile (ty, tx) - in tiles from
    the south pole and 180 west - or None for one that is all sea; and
    whether the GLO-90 stood in for the GLO-30 for its light and shade.
    With `moor` False, no moor - the grass left grass; with `dem_fallback`,
    the GLO-90's heights for the shade where the GLO-30 has no tile of its
    own, as over Armenia and Azerbaijan, which it leaves out - else flat
    there, as the sea is."""
    south, west = ty / TILES_PER_DEG - 90, tx / TILES_PER_DEG - 180
    k = k_for(south + TILE_DEG / 2)
    if k is None:
        return None, False
    rows, cols = ROWS0, int(ROWS0 / k)
    counts = cover_counts(south, west, rows, cols)
    stood_in = []
    z = heights(south, west, rows, cols, border=1, template=DEM30, resampling=Resampling.average,
                fallback=DEM90 if dem_fallback else None, used=stood_in)
    if counts[:, :, NONE].all() and not (z > 0).any():
        return None, False
    mid = math.radians(south + TILE_DEG / 2)
    cell_y = TILE_DEG / rows * 111320.0
    cell_x = TILE_DEG / cols * 111320.0 * math.cos(mid)
    light, steep = shade(z, cell_y, cell_x)  # rows north first, as a picture's are
    ground = z[1:-1, 1:-1]
    moorline = min(400.0, max(150.0, 400 - 25 * (abs(south + TILE_DEG / 2) - 50))) if moor else None
    north_row = (180 * TILES_PER_DEG - ty - 1) * rows  # the tile's first row of cells, from the north pole
    west_col = tx * cols
    levels = []
    for level in range(LEVELS):
        f = 1 << level
        r, c = rows // f, cols // f
        cell_counts = counts if f == 1 else blocks(counts.astype(np.uint16), f)
        mean = lambda a: blocks(a, f) / (f * f)
        dither = ragged(north_row // f, west_col // f, r, c, f) if moor else None
        covers = classify(cell_counts, mean(ground), mean(steep), moorline, dither)
        shades = np.clip(np.rint(mean(light)), 0, 15).astype(np.uint8)
        levels.append((covers << 4) | shades)
    fine_h = heights(south, west, rows // 6, cols // 6)
    coarse_h = heights(south, west, rows // 16, cols // 16)
    h, w = rows // 2, cols // 2
    level0 = levels[0]
    detail = b"".join(level0[a:a + h, b:b + w].tobytes() for a in (0, h) for b in (0, w))
    middle = levels[1].tobytes() + levels[2].tobytes() + levels[3].tobytes() + fine_h.astype("<i2").tobytes()
    coarse = levels[4].tobytes() + coarse_h.astype("<i2").tobytes()
    return (deflate(coarse), deflate(middle), deflate(detail)), bool(stood_in)


def tile(job):
    """The coarse, mid and detail blocks of the tile `job` - (row, col) in
    tiles from the south pole and 180 west - or None for one that is all
    sea: the pack's own, with the moor's line, from the GLO-30 alone."""
    return make_tile(*job)[0]


def write_pack(f, south, west, rows, cols, tiles):
    """The pack, written to the open file `f`: its grid `rows` by `cols`
    tiles from `south` and `west` - in tiles from the equator and the
    meridian - and `tiles` on it, (row, col, coarse, mid, detail) each, by
    latitude then longitude, as the index has them. Its size, where its
    detail blocks start, and how long its coarse blocks are together."""
    header_size, row = 32, struct.calcsize("<HHIIIIII")
    index_offset = header_size
    coarse_offset = index_offset + row * len(tiles)
    coarse_length = sum(len(t[2]) for t in tiles)
    mid_offset = coarse_offset + coarse_length
    detail_offset = mid_offset + sum(len(t[3]) for t in tiles)
    f.write(struct.pack("<4sHHhhHHIIII", b"OVGR", 2, TILES_PER_DEG, south, west, rows, cols, len(tiles),
                        index_offset, coarse_offset, coarse_length))
    co, mo, do = coarse_offset, mid_offset, detail_offset
    for r, c, cb, mb, db in tiles:
        f.write(struct.pack("<HHIIIIII", r, c, co, len(cb), mo, len(mb), do, len(db)))
        co += len(cb)
        mo += len(mb)
        do += len(db)
    for part in (2, 3, 4):
        for t in tiles:
            f.write(t[part])
    return f.tell(), detail_offset, coarse_length


def start_worker():
    # Each process its own GDAL settings and caches, kept for its life.
    global _env
    _env = rasterio.Env(GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR", CPL_VSIL_CURL_ALLOWED_EXTENSIONS=".tif",
                        GDAL_HTTP_MULTIRANGE="YES", GDAL_CACHEMAX=128, VSI_CACHE="TRUE", VSI_CACHE_SIZE=64 << 20,
                        GDAL_HTTP_MAX_RETRY=4, GDAL_HTTP_RETRY_DELAY=2)
    _env.__enter__()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--south", type=float, default=49.8)
    ap.add_argument("--north", type=float, default=61.0)
    ap.add_argument("--west", type=float, default=-10.8)
    ap.add_argument("--east", type=float, default=2.0)
    ap.add_argument("--jobs", type=int, default=8, help="tiles read at once")
    args = ap.parse_args()
    s0 = int(math.floor(args.south * TILES_PER_DEG))
    n0 = int(math.ceil(args.north * TILES_PER_DEG))
    w0 = int(math.floor(args.west * TILES_PER_DEG))
    e0 = int(math.ceil(args.east * TILES_PER_DEG))
    jobs = [(ty + 90 * TILES_PER_DEG, tx + 180 * TILES_PER_DEG) for ty in range(s0, n0) for tx in range(w0, e0)]
    tiles = []
    started = time.time()
    total = len(jobs)
    size = 0
    with multiprocessing.Pool(args.jobs, initializer=start_worker) as pool:
        for done, ((ty, tx), got) in enumerate(zip(jobs, pool.imap(tile, jobs, chunksize=2)), 1):
            if got is not None:
                tiles.append((ty - 90 * TILES_PER_DEG - s0, tx - 180 * TILES_PER_DEG - w0) + got)
                size += sum(map(len, got))
            if done % 50 == 0:
                rate = done / max(1e-6, time.time() - started)
                print(f"{done}/{total} tiles, {len(tiles)} with land, {size / 1e6:.1f}MB, "
                      f"{(total - done) / rate / 60:.1f} min to go", flush=True)
    with open(args.out, "wb") as f:
        size, detail_offset, coarse_length = write_pack(f, s0, w0, n0 - s0, e0 - w0, tiles)
    print(f"{args.out}: {len(tiles)} tiles of {total}, {size / 1e6:.1f}MB - coarse {coarse_length / 1e3:.0f}KB, "
          f"detail {(size - detail_offset) / 1e6:.1f}MB - in {(time.time() - started) / 60:.1f} min")


if __name__ == "__main__":
    sys.exit(main())
