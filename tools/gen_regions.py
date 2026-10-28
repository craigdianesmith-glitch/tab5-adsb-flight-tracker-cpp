#!/usr/bin/env python3
"""Generates src/regions_table.inc from the Natural Earth dataset.

The places the firmware treats apart: Scotland and England, where home keeps
the UK's time (src/clock_zone.cpp); and the United States, where it keeps one
of the mainland's four zones, and a height read aloud is a flight level from
18,000ft rather than 6,000ft (src/sound.cpp). Scotland and England are apart
from when each had a voice of its own, until 1.5.1. From Natural Earth's
1:10m map units, which have Scotland and England apart where its countries
have the United Kingdom:

- Which of them each airport in src/airports.cpp is in, worked out here from
  the outlines as Natural Earth has them: an airport at the water's edge can
  be a few hundred metres off its coast, and is taken to be in the country
  whose land is nearest it, within 5nm.
- Their outlines, simplified to a couple of kilometres, for where an aircraft
  is with no airport near it.

    tools/gen_regions.py          # downloads the one layer
    tools/gen_regions.py DIR      # from ne_10m_admin_0_map_units.geojson in DIR
"""

import json
import math
import re
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA_URL = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/v5.1.2/geojson/"
LAYER = "ne_10m_admin_0_map_units"

# The map unit each is, by its NAME, and how far a point may be left out
# from where it was, in degrees: the United States is drawn a little more
# coarsely, being the bigger by far.
REGIONS = [
    ("SCOTLAND", "Scotland", 0.02),
    ("ENGLAND", "England", 0.02),
    ("USA", "United States of America", 0.05),
]
# An airport on none of Natural Earth's land is in the country nearest it, as
# far off as this.
SHORE_NM = 5.0


def load(directory):
    if directory:
        with open(Path(directory) / f"{LAYER}.geojson", encoding="utf-8") as f:
            return json.load(f)
    with urllib.request.urlopen(DATA_URL + LAYER + ".geojson") as r:
        return json.load(r)


# Douglas-Peucker, on longitude and latitude as they are.
def simplify(ring, tol):
    keep = [False] * len(ring)
    keep[0] = keep[-1] = True
    stack = [(0, len(ring) - 1)]
    while stack:
        i, j = stack.pop()
        (ax, ay), (bx, by) = ring[i], ring[j]
        dx, dy = bx - ax, by - ay
        n = (dx * dx + dy * dy) ** 0.5
        far, at = -1.0, -1
        for k in range(i + 1, j):
            px, py = ring[k]
            d = abs(dx * (py - ay) - dy * (px - ax)) / n if n else ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
            if d > far:
                far, at = d, k
        if far > tol:
            keep[at] = True
            stack += [(i, at), (at, j)]
    return [p for p, k in zip(ring, keep) if k]


def rings_of(geometry):
    polygons = geometry["coordinates"] if geometry["type"] == "MultiPolygon" else [geometry["coordinates"]]
    return [ring for polygon in polygons for ring in polygon]


# Each ring with its box, for a point to be tested against quickly.
def boxed(rings):
    out = []
    for ring in rings:
        lons, lats = [p[0] for p in ring], [p[1] for p in ring]
        out.append((ring, min(lons), min(lats), max(lons), max(lats)))
    return out


def inside(boxed_rings, lon, lat):
    c = False
    for ring, w, s, e, n in boxed_rings:
        if lon < w or lon > e or lat < s or lat > n:
            continue
        for (ax, ay), (bx, by) in zip(ring, ring[1:]):
            if (ay > lat) != (by > lat) and lon < ax + (bx - ax) * (lat - ay) / (by - ay):
                c = not c
    return c


# How far (lon, lat) is from the nearest edge of the rings, in nm, looking no
# further than `limit`.
def distance_nm(boxed_rings, lon, lat, limit):
    k = math.cos(math.radians(lat))
    reach = limit / 60.0
    best = limit
    for ring, w, s, e, n in boxed_rings:
        if lon < w - reach / k or lon > e + reach / k or lat < s - reach or lat > n + reach:
            continue
        for (ax, ay), (bx, by) in zip(ring, ring[1:]):
            x0, y0 = (ax - lon) * k * 60, (ay - lat) * 60
            dx, dy = (bx - ax) * k * 60, (by - ay) * 60
            t = max(0.0, min(1.0, -(x0 * dx + y0 * dy) / (dx * dx + dy * dy))) if dx or dy else 0.0
            best = min(best, math.hypot(x0 + t * dx, y0 + t * dy))
    return best


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else None
    features = load(directory)["features"]
    units = {f["properties"]["NAME"]: f["geometry"] for f in features}
    points, rings, shapes = [], [], []
    for region, name, tol in REGIONS:
        first = len(rings)
        for ring in rings_of(units[name]):
            ring = simplify([(p[0], p[1]) for p in ring], tol)
            if len(ring) < 4:
                continue  # an islet, gone to a line - its airport, if it has one, is in the list
            lons, lats = [p[0] for p in ring], [p[1] for p in ring]
            rings.append((len(points) // 2, len(ring), min(lats), min(lons), max(lats), max(lons)))
            for lon, lat in ring:
                points += [lat, lon]
        shapes.append((region, first, len(rings) - first))

    # Every airport's, from the outlines in full: in one of the regions, or on
    # no land at all and nearest one of them.
    full = {region: boxed(rings_of(units[name])) for region, name, _ in REGIONS}
    everyone = [(f["properties"]["NAME"], boxed(rings_of(f["geometry"]))) for f in features if f["geometry"]]
    airports = re.findall(r'\{"([A-Z]{3})", (-?[\d.]+)f, (-?[\d.]+)f\}', (ROOT / "src" / "airports.cpp").read_text())
    placed = []
    for code, lat, lon in airports:
        lat, lon = float(lat), float(lon)
        region = next((r for r, b in full.items() if inside(b, lon, lat)), None)
        if region is None and not any(inside(b, lon, lat) for _, b in everyone):
            nearest = min(((distance_nm(b, lon, lat, SHORE_NM), name) for name, b in everyone), default=(SHORE_NM, None))
            region = next((r for r, n, _ in REGIONS if n == nearest[1] and nearest[0] < SHORE_NM), None)
        if region:
            placed.append((code, region))

    out = [
        "// Generated by tools/gen_regions.py from Natural Earth's 1:10m map units - do not edit.",
        "// Latitude and longitude, a point at a time, ring after ring.",
        "const float REGION_POINTS[] = {",
    ]
    for i in range(0, len(points), 16):
        out.append("    " + ", ".join(f"{v:.4f}f" for v in points[i:i + 16]) + ",")
    out.append("};")
    out.append("// Each ring: its first point, how many, and the box round it.")
    out.append("const RegionRing REGION_RINGS[] = {")
    for first, count, s, w, n, e in rings:
        out.append(f"    {{{first}, {count}, {s:.4f}f, {w:.4f}f, {n:.4f}f, {e:.4f}f}},")
    out.append("};")
    out.append("// The airports in each, by code, in order.")
    out.append("const AirportRegion AIRPORT_REGIONS[] = {")
    for i in range(0, len(placed), 6):
        out.append("    " + " ".join(f'{{"{c}", Region::{r}}},' for c, r in placed[i:i + 6]))
    out.append("};")
    out.append("// Each place: its rings, the first of them and how many.")
    out.append("const RegionShape REGION_SHAPES[] = {")
    for region, first, count in shapes:
        out.append(f"    {{Region::{region}, {first}, {count}}},")
    out.append("};")
    (ROOT / "src" / "regions_table.inc").write_text("\n".join(out) + "\n")
    print(f"{len(points) // 2} points in {len(rings)} rings: " +
          ", ".join(f"{region} {count} rings" for region, _, count in shapes))
    print(f"{len(placed)} of {len(airports)} airports in them: " +
          ", ".join(f"{r} {sum(1 for _, x in placed if x == r)}" for r, _, _ in REGIONS))


if __name__ == "__main__":
    main()
