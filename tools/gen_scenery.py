#!/usr/bin/env python3
"""Generates src/scenery_table.inc from the Natural Earth dataset.

What the 3D view draws of the country besides its coasts: the towns, as the
land built over, and the rivers. Natural Earth (naturalearthdata.com) is
public domain; its 1:10m urban areas - the built-up land as seen from orbit,
the outlines of 500m squares - and its rivers, the world's and the more
closely mapped Europe's, North America's and Australia's, without the lines
it draws down the middles of the lakes they run through.

    tools/gen_scenery.py        # downloads the five layers
    tools/gen_scenery.py DIR    # from the ne_10m_*.geojson files in DIR

Needs shapely (pip install shapely), for thinning a town's outline without
its edges crossing: the view tells a town from the country round it as it
tells the land from the sea - by which way each outline crosses a row of the
screen - and an outline crossed over in the thinning paints a band across
the view (as the coasts' thinned copies did).

Laid out as the coast table is (see tools/gen_coast.py, whose cutting into
cells this uses): a degree square to a cell, the cells with anything in them
listed in order, and in each a line as a start and steps of a signed byte on
a grid of 54m. A line's start has its kind in the four bits above x: three
for what it is - a town's outline, with the town on its left, or a river
by how wide it is drawn - and one for a line going on from the one before.
"""

import json
import sys
import urllib.request
from collections import defaultdict
from pathlib import Path

from shapely.geometry import shape
from shapely.geometry.polygon import orient

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gen_coast import DATA_URL, MAX_STEP, MAX_STEPS, ROOT, UNIT, array, cut_into_cells  # noqa: E402

# As SceneryKind in include/scenery.h numbers them: a town's outline, then
# rivers, narrowest first - drawn 25, 60, 150 and 400m wide.
TOWN, RIVER_SMALL, RIVER, RIVER_BIG, RIVER_HUGE = range(5)
CONTINUES = 8  # with the kind: the line goes on from the one before

# A town's outline kept to within this of where it ran, in degrees - 110m
# north and south, less east and west. From the 3D view's lowest, a few
# hundred feet up, a town's edge a kilometre off, that is a few pixels.
TOWN_TOLERANCE = 0.001
RIVER_TOLERANCE = 0.0004

RIVERS = ["ne_10m_rivers_lake_centerlines", "ne_10m_rivers_europe", "ne_10m_rivers_north_america",
          "ne_10m_rivers_australia"]


def load(layer, directory):
    if directory:
        with open(Path(directory) / f"{layer}.geojson", encoding="utf-8") as f:
            return json.load(f)
    with urllib.request.urlopen(DATA_URL + layer + ".geojson") as r:
        return json.load(r)


def river_kind(props):
    """How wide a river is drawn, from how early Natural Earth shows it - its
    scale rank, nought for the Amazon - or, in the regional layers, from the
    stroke it is drawn with."""
    weight = props.get("strokeweig")
    if weight:
        return RIVER_HUGE if weight >= 1.5 else RIVER_BIG if weight >= 0.8 else RIVER if weight >= 0.4 else RIVER_SMALL
    rank = props.get("scalerank") or 10
    return RIVER_HUGE if rank <= 2 else RIVER_BIG if rank <= 5 else RIVER if rank <= 8 else RIVER_SMALL


def encode(kind, points):
    """A piece as the device reads it - as gen_coast's, with the kind in
    three bits and going-on in the fourth."""
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
        out += (run[0][0] | flags << 12).to_bytes(2, "little") + run[0][1].to_bytes(2, "little")
        out.append(len(run) - 1)
        for a, b in zip(run, run[1:]):
            out += (b[0] - a[0]).to_bytes(1, "little", signed=True) + (b[1] - a[1]).to_bytes(1, "little", signed=True)
    return out


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else None
    cells = defaultdict(list)
    tally = defaultdict(int)

    for feature in load("ne_10m_urban_areas", directory)["features"]:
        if not feature["geometry"]:
            continue
        thin = shape(feature["geometry"]).simplify(TOWN_TOLERANCE, preserve_topology=True)
        for polygon in getattr(thin, "geoms", [thin]):
            if polygon.is_empty or polygon.geom_type != "Polygon":
                continue
            # The outline anticlockwise and any hole in it clockwise: the
            # town on the left of both.
            polygon = orient(polygon, 1.0)
            for ring in [polygon.exterior, *polygon.interiors]:
                cut_into_cells(list(ring.coords), UNIT, cells, TOWN)
                tally["town points"] += len(ring.coords)

    for layer in RIVERS:
        for feature in load(layer, directory)["features"]:
            props = feature["properties"]
            if not feature["geometry"] or props.get("featurecla") == "Lake Centerline":
                continue
            kind = river_kind(props)
            thin = shape(feature["geometry"]).simplify(RIVER_TOLERANCE)
            for line in getattr(thin, "geoms", [thin]):
                if line.is_empty or len(line.coords) < 2:
                    continue
                cut_into_cells(list(line.coords), UNIT, cells, kind)
                tally["river points"] += len(line.coords)

    ids = sorted(cells)
    starts, data = [], bytearray()
    for cell in ids:
        starts.append(len(data))
        for kind, piece in cells[cell]:
            if len(piece) >= 2:
                data += encode(kind, piece)
    starts.append(len(data))

    out = [
        "// Generated by tools/gen_scenery.py from Natural Earth's 1:10m urban areas and",
        "// rivers - do not edit.",
        f"constexpr int SCENERY_UNIT = {UNIT};  // steps of the grid to a degree",
        "// The cells with anything in them, as latitude * 360 + longitude from 90S 180W.",
    ]
    out += array("SCENERY_CELLS", "uint16_t", ids, 20)
    out.append("// Where each cell's lines start in SCENERY_LINES, and the last one's end.")
    out += array("SCENERY_STARTS", "uint32_t", starts, 16)
    out.append("// The lines: start x with the kind above it, start y, the number of steps, the steps.")
    out += array("SCENERY_LINES", "uint8_t", list(data), 32)
    (ROOT / "src" / "scenery_table.inc").write_text("\n".join(out) + "\n")
    size = len(data) + 2 * len(ids) + 4 * len(starts)
    print(f"{tally['town points']} points of towns' outlines, {tally['river points']} of rivers; "
          f"{len(ids)} cells, {size / 1024:.0f}KB")


if __name__ == "__main__":
    main()
