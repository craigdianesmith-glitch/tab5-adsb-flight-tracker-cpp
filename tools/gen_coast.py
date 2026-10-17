#!/usr/bin/env python3
"""Generates src/coast_table.inc from the Natural Earth dataset.

The outlines the radar draws under everything else, to say where its traffic
is: the world's coastlines and lakes, its countries' borders, and the state
and province lines of the big federations, where a plot can be a long way
from any of the rest. Natural Earth (naturalearthdata.com) is public domain;
this is its 1:10m set, the finest it has.

    tools/gen_coast.py          # downloads the ten layers
    tools/gen_coast.py DIR      # from the ne_10m_*.geojson files in DIR

All of the work that can be done ahead is done here, so that the device only
has to look up the few cells under its plot and add up steps:

- The lines are cut into cells a degree square, and only the cells with
  something in them are listed, in order - by latitude, then longitude - for
  a search to go straight to.
- Within its cell a line is a start and then steps of a signed byte each way,
  on a grid of 54m - a pixel at the 10nm range. A step too long for a byte is
  taken in several.

There is one level of detail, not several: Natural Earth's points are a mile
or so apart, which is already further than a pixel at the widest range the
radar has, so a second level with every bend under 250m taken out still had
two thirds of the points - two thirds the flash again, to spare the widest
ranges a third of their strokes. The other way, close in, the device draws a
coast as a curve through its points - see src/coast.cpp.

Every coast and lake shore runs with the land on its left. Natural Earth's
don't all: of its coastlines about one in seven has the land on its right,
and nearly all of its minor islands' do. Each is turned round where it
needs to be - a coastline by testing a point a few metres either side of it
against Natural Earth's land polygons, a lake's shore by whether it is the
lake's outline or an island's in it. The radar doesn't care which way a line
runs; a 3D view high up does, telling land from sea by which way a coast
crosses each row of it.

Left out: Natural Earth's own notes on a border - an overlay or a lease's
limit - and every country's states but those of STATES_OF below.
"""

import json
import math
import re
import struct
import sys
import urllib.request
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
# A release rather than master, so a table made later is the same table.
DATA_URL = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/v5.1.2/geojson/"

# As OutlineKind in include/coast.h numbers them.
COAST, LAKE, BORDER, STATE = range(4)
LAYERS = [
    ("ne_10m_coastline", COAST),
    ("ne_10m_minor_islands_coastline", COAST),
    ("ne_10m_lakes", LAKE),
    # The smaller lakes it has for the parts of the world mapped more closely.
    ("ne_10m_lakes_europe", LAKE),
    ("ne_10m_lakes_north_america", LAKE),
    ("ne_10m_lakes_australia", LAKE),
    ("ne_10m_admin_0_boundary_lines_land", BORDER),
    ("ne_10m_admin_1_states_provinces_lines", STATE),
]
BORDER_CLASSES = {
    "International boundary (verify)",
    "Disputed (please verify)",
    "Indefinite (please verify)",
    "Line of control (please verify)",
}
# Whose states and provinces are drawn: the federations a plot can sit in the
# middle of with no coast or border anywhere on it.
STATES_OF = {"USA", "CAN", "MEX", "BRA", "AUS"}
# All of the classes those five have. A "statistical" one is a state line that
# a census region's edge runs along as well - Colorado's with Nebraska is - and
# an "indicator" one carries a state line on across water, as down the
# Chesapeake.
STATE_CLASSES = {
    "Admin-1 boundary",
    "Admin-1 statistical boundary",
    "Admin-1 statistical meta bounds",
    "Admin-1 boundary indicator",
}

# How many steps of the grid there are to a degree: a power of two, and no
# more than 2048, for a cell's width to fit the twelve bits it has.
UNIT = 2048
MAX_STEP = 127   # a signed byte
MAX_STEPS = 255  # a line's count of them is one byte
CONTINUES = 4    # with the kind: the line goes on from the one before


def load(layer, directory):
    if directory:
        with open(Path(directory) / f"{layer}.geojson", encoding="utf-8") as f:
            return json.load(f)
    with urllib.request.urlopen(DATA_URL + layer + ".geojson") as r:
        return json.load(r)


def wanted(layer, props):
    if layer == "ne_10m_coastline":
        return props["featurecla"] == "Coastline"  # the other is Null Island, a joke at 0, 0
    if layer == "ne_10m_admin_0_boundary_lines_land":
        return props["FEATURECLA"] in BORDER_CLASSES
    if layer == "ne_10m_admin_1_states_provinces_lines":
        return props["ADM0_A3"] in STATES_OF and props["FEATURECLA"] in STATE_CLASSES
    return True


# The land the coasts are tested against: Natural Earth's land polygons for
# its coastlines, its minor islands' for theirs.
LAND_OF = {"ne_10m_coastline": "ne_10m_land", "ne_10m_minor_islands_coastline": "ne_10m_minor_islands"}


class Land:
    """Whether a point is on land: a ray cast east from it, its crossings of
    the polygons' edges counted - the edges kept in rows a tenth of a degree
    of latitude deep, so a test looks at only those its row could cross."""

    def __init__(self, geojson):
        self.rows = defaultdict(list)
        for feature in geojson["features"]:
            geometry = feature["geometry"]
            if not geometry:
                continue
            polygons = [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
            for polygon in polygons:
                for ring in polygon:
                    for (x0, y0), (x1, y1) in zip(ring, ring[1:]):
                        if y0 != y1:
                            for row in range(math.floor(min(y0, y1) * 10), math.floor(max(y0, y1) * 10) + 1):
                                self.rows[row].append((x0, y0, x1, y1))

    def has(self, x, y):
        crossings = 0
        for x0, y0, x1, y1 in self.rows[math.floor(y * 10)]:
            if (y0 <= y) != (y1 <= y) and x0 + (y - y0) * (x1 - x0) / (y1 - y0) > x:
                crossings += 1
        return crossings % 2 == 1


def area(ring):
    """Twice a ring's signed area: more than nothing, anticlockwise."""
    return sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(ring, ring[1:]))


def land_on_left(line, land):
    """Which side of a coastline the land is: tested a few metres either side
    of segments along it - a quarter, half and three quarters of the way, and
    on through the rest until one says - +1 left, -1 right, 0 not known."""
    n = len(line)
    order = [n // 4, n // 2, (3 * n) // 4] + list(range(n - 1))
    for i in order:
        i = min(max(i, 0), n - 2)
        (x0, y0), (x1, y1) = line[i][:2], line[i + 1][:2]
        length = math.hypot(x1 - x0, y1 - y0)
        if length == 0:
            continue
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        ox, oy = -(y1 - y0) / length * 1e-4, (x1 - x0) / length * 1e-4
        left, right = land.has(mx + ox, my + oy), land.has(mx - ox, my - oy)
        if left != right:
            return 1 if left else -1
    return 0


def oriented(layer, geometry, land, tally):
    """A feature's lines, each coast and shore turned so the land is on its
    left: see the docstring at the top."""
    if layer in LAND_OF:
        out = []
        for line in lines_of(geometry):
            side = land_on_left(line, land)
            if side == 0 and line[0] == line[-1]:
                side = 1 if area(line) > 0 else -1  # an island it couldn't tell: land inside
            tally["turned" if side < 0 else "kept"] += 1
            out.append(line[::-1] if side < 0 else line)
        return out
    if geometry["type"] in ("Polygon", "MultiPolygon"):
        # A lake: its outline clockwise, water on the right; an island in it
        # anticlockwise - the land on the left of both.
        polygons = [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
        out = []
        for polygon in polygons:
            for i, ring in enumerate(polygon):
                wanted = area(ring) < 0 if i == 0 else area(ring) > 0
                tally["kept" if wanted else "turned"] += 1
                out.append(ring if wanted else ring[::-1])
        return out
    return lines_of(geometry)


def lines_of(geometry):
    kind, coords = geometry["type"], geometry["coordinates"]
    if kind == "LineString":
        return [coords]
    if kind in ("MultiLineString", "Polygon"):
        return coords
    if kind == "MultiPolygon":
        return [ring for polygon in coords for ring in polygon]
    return []


# Where a step from (x0, y0) to (x1, y1) crosses from one cell into the next,
# in order - on the grid, whose cells are `unit` across.
def crossings(x0, y0, x1, y1, unit):
    ts = []
    for a0, a1 in ((x0, x1), (y0, y1)):
        lo, hi = min(a0, a1), max(a0, a1)
        edge = (lo // unit + 1) * unit
        while edge < hi:
            ts.append((edge - a0) / (a1 - a0))
            edge += unit
    return [(round(x0 + t * (x1 - x0)), round(y0 + t * (y1 - y0))) for t in sorted(ts)]


# A line as the pieces of it in each cell it passes through, added to
# out[cell], the points of each from its cell's south-west corner.
def cut_into_cells(points, unit, out, kind):
    grid = []
    for lon, lat in points:
        p = (round((lon + 180) * unit), round((lat + 90) * unit))
        if not grid or grid[-1] != p:
            grid.append(p)
    pieces = []
    for (x0, y0), (x1, y1) in zip(grid, grid[1:]):
        stops = [(x0, y0)]
        for p in crossings(x0, y0, x1, y1, unit) + [(x1, y1)]:
            if p != stops[-1]:
                stops.append(p)
        for a, b in zip(stops, stops[1:]):
            # By its middle, which is inside the one cell even where both its
            # ends are on an edge - and a line along an edge, as the 49th
            # parallel is, goes to the cell above or to the right of it.
            cell = (min((a[0] + b[0]) // (2 * unit), 359), min((a[1] + b[1]) // (2 * unit), 179))
            if not pieces or pieces[-1][0] != cell:
                pieces.append((cell, [a]))
            pieces[-1][1].append(b)
    # A ring - an island, a lake - that leaves the cell it starts in ends in
    # it too, as two pieces that are one: joined, so that where Natural Earth
    # happened to start the ring is a point like any other on it, for the
    # device to curve through.
    if len(pieces) > 1 and grid[0] == grid[-1] and pieces[0][0] == pieces[-1][0]:
        pieces[0] = (pieces[0][0], pieces.pop()[1] + pieces[0][1][1:])
    for (cx, cy), piece in pieces:
        out[cy * 360 + cx].append((kind, [(x - cx * unit, y - cy * unit) for x, y in piece]))


# A piece as the device reads it: where it starts, with what it is in the top
# four bits; how many steps follow; and the steps. One with more steps than
# the count holds goes on in another, marked as going on.
def encode(kind, points):
    path = [points[0]]
    for x, y in points[1:]:
        px, py = path[-1]
        dx, dy = x - px, y - py
        n = max(1, -(-max(abs(dx), abs(dy)) // MAX_STEP))
        for i in range(1, n + 1):
            path.append((px + round(dx * i / n), py + round(dy * i / n)))
    out = bytearray()
    for first in range(0, len(path) - 1, MAX_STEPS):
        run = path[first:first + MAX_STEPS + 1]
        flags = kind | (CONTINUES if first else 0)
        out += struct.pack("<HHB", run[0][0] | flags << 12, run[0][1], len(run) - 1)
        for a, b in zip(run, run[1:]):
            out += struct.pack("bb", b[0] - a[0], b[1] - a[1])
    return out


def array(name, ctype, values, per_row):
    rows = [f"const {ctype} {name}[] = {{"]
    for i in range(0, len(values), per_row):
        rows.append("    " + ", ".join(map(str, values[i:i + per_row])) + ",")
    rows.append("};")
    return rows


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else None
    cells = defaultdict(list)
    lines = points = 0
    for layer, kind in LAYERS:
        land = Land(load(LAND_OF[layer], directory)) if layer in LAND_OF else None
        tally = defaultdict(int)
        for feature in load(layer, directory)["features"]:
            if feature["geometry"] and wanted(layer, feature["properties"]):
                for line in oriented(layer, feature["geometry"], land, tally):
                    cut_into_cells([(p[0], p[1]) for p in line], UNIT, cells, kind)
                    lines += 1
                    points += len(line)
        if tally:
            print(f"  {layer}: {tally['kept']} the right way round, {tally['turned']} turned round")

    ids = sorted(cells)
    starts, data = [], bytearray()
    steps = {}
    for cell in ids:
        starts.append(len(data))
        # The coast last, over a border that runs down to it.
        for kind, piece in sorted(cells[cell], key=lambda p: -p[0]):
            data += encode(kind, piece)
        steps[cell] = (len(data) - starts[-1]) // 2
    starts.append(len(data))

    out = [
        "// Generated by tools/gen_coast.py from Natural Earth's 1:10m coastline, lakes,",
        "// boundary lines and state and province lines - do not edit.",
        f"constexpr int COAST_UNIT = {UNIT};  // steps of the grid to a degree",
        "// The cells with anything in them, as latitude * 360 + longitude from 90S 180W.",
    ]
    out += array("COAST_CELLS", "uint16_t", ids, 20)
    out.append("// Where each cell's lines start in COAST_LINES, and the last one's end.")
    out += array("COAST_STARTS", "uint32_t", starts, 16)
    out.append("// The lines: start x with the kind above it, start y, the number of steps, the steps.")
    out += array("COAST_LINES", "uint8_t", list(data), 32)
    (ROOT / "src" / "coast_table.inc").write_text("\n".join(out) + "\n")
    size = len(data) + 2 * len(ids) + 4 * len(starts)
    print(f"{points} points in {lines} lines, {len(ids)} cells, {size / 1024:.0f}KB")

    # The most there is to draw on any airport's plot: what the radar has to
    # get through in a frame, at the airports in its table.
    airports = [(code, float(lat), float(lon)) for code, lat, lon in
                re.findall(r'\{"([A-Z]{3})", (-?[\d.]+)f, (-?[\d.]+)f\}', (ROOT / "src" / "airports.cpp").read_text())]
    for nm in (20, 60, 150):
        loads = []
        for code, lat, lon in airports:
            dlat = nm / 60
            dlon = nm / (60 * max(math.cos(math.radians(lat)), 0.01))
            count = 0
            for cy in range(math.floor(lat - dlat + 90), math.floor(lat + dlat + 90) + 1):
                for cx in range(math.floor(lon - dlon + 180), math.floor(lon + dlon + 180) + 1):
                    count += steps.get(cy * 360 + cx % 360, 0)
            loads.append((count, code))
        loads.sort(reverse=True)
        print(f"  at {nm}nm, steps in the cells under an airport's plot: most " +
              ", ".join(f"{code} {count}" for count, code in loads[:5]) + f"; median {loads[len(loads) // 2][0]}")


if __name__ == "__main__":
    main()
