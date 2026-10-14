#!/usr/bin/env python3
"""Generates src/names_table.inc from the Natural Earth dataset.

The names the radar writes on its map, faintly, under everything else - see
src/names.cpp: its countries; their regions, the states, provinces, counties
and council areas Natural Earth calls first-order divisions; its seas, bays
and straits; and its lakes. Natural Earth (naturalearthdata.com) is public
domain; this is its 1:10m set, as tools/gen_coast.py uses for the outlines.

    tools/gen_names.py          # downloads the eight layers
    tools/gen_names.py DIR      # from the ne_10m_*.geojson files in DIR

Each name is given the spot inside its outline with the most room for it:
the middle of the biggest box, the shape of the name as the radar writes it,
that fits inside - rather than the outline's centroid, which for a bay or a
horseshoe is off it, and for a country with a long coast can be in a corner
of it. With it goes how big that box is, as the most a pixel of the name at
size 1 can stand for, in nautical miles: at any range, the device sizes the
name to its box with a multiplication. A name of two words or more can go
on two lines, where they make more of the room.

A name made of several outlines - a country with its islands, a lake Natural
Earth has in two parts - goes on the one with the most room for it.

Takes a few minutes, all of it finding the boxes; it uses every core.
"""

import json
import heapq
import math
import multiprocessing
import re
import sys
import unicodedata
import urllib.request
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA_URL = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/v5.1.2/geojson/"

# As NameKind in include/names.h numbers them.
COUNTRY, SEA, REGION, LAKE = range(4)
LAYERS = [
    ("ne_10m_admin_0_map_units", COUNTRY, "NAME"),
    ("ne_10m_geography_marine_polys", SEA, "name"),
    ("ne_10m_admin_1_states_provinces", REGION, "name"),
    ("ne_10m_lakes", LAKE, "name"),
    ("ne_10m_lakes_europe", LAKE, "name"),
    ("ne_10m_lakes_north_america", LAKE, "name"),
    ("ne_10m_lakes_australia", LAKE, "name"),
]
# The marine areas that are areas of water, rather than a reef.
SEA_CLASSES = {"ocean", "sea", "gulf", "bay", "strait", "sound", "channel", "fjord", "lagoon", "inlet", "river",
               "generic"}

# Natural Earth's own slips; names it gives in another country's language,
# where its neighbours' are their own; and a lake it has in parts by name.
RENAMED = {
    "North Ayshire": "North Ayrshire",
    "Perthshire and Kinross": "Perth and Kinross",
    "Sjaælland": "Sjælland",
    "Laoighis": "Laois",
    "Aoste": "Aosta",
    "Bozen": "Bolzano",
    "Loch Lomond North Basin": "Loch Lomond",
}
# Countries its map units split into the parts of their politics - Belgium
# into Flemish and Walloon - named as the one country. The United Kingdom's
# four are left apart, being countries of their own.
UNSPLIT = {
    "Belgium": "Belgium",
    "Bosnia and Herzegovina": "Bosnia and Herz.",
    "Republic of Serbia": "Serbia",
    "Georgia": "Georgia",
    "Iraq": "Iraq",
    "Somalia": "Somalia",
}

# How the device writes a name, in pixels of Font0 at size 1 - as
# src/names.cpp has them: a letter every 6 and the letters 8 high, with this
# much more between letters for each kind, and this between two lines.
ADVANCE, HEIGHT, LINE_GAP = 6, 8, 3
SPACING = {COUNTRY: 2, SEA: 2, REGION: 1, LAKE: 1}
# Two lines only where they give it this much more room than one.
TWO_LINES_GAIN = 1.25


def load(layer, directory):
    if directory:
        with open(Path(directory) / f"{layer}.geojson", encoding="utf-8") as f:
            return json.load(f)
    with urllib.request.urlopen(DATA_URL + layer + ".geojson") as r:
        return json.load(r)


# Letters Unicode doesn't take apart into a plain one and an accent.
SPELLED = {"ß": "SS", "æ": "AE", "Æ": "AE", "œ": "OE", "Œ": "OE", "ø": "O", "Ø": "O", "ł": "L", "Ł": "L",
           "đ": "D", "Đ": "D", "ð": "D", "Ð": "D", "þ": "TH", "Þ": "TH", "ı": "I", "ə": "A", "Ə": "A",
           "–": "-", "—": "-", "’": "'", "‘": "'"}


# A name as the radar writes it: in capitals, in the plain letters of its
# font, without what Natural Earth adds in brackets - "Homs (Hims)" - or the
# apostrophe it ends a word with for a letter the plain ones have none for,
# Arabic's or Russian's - "Karbala'", "Tver'" - though not a short word's own,
# "Lake O' the Pines"; and with its "Lac d' Annecy" closed up.
def clean(name):
    name = RENAMED.get(name, name)
    name = re.sub(r"\s*\([^)]*\)", "", name)
    out = []
    for ch in unicodedata.normalize("NFKD", name):
        if unicodedata.combining(ch):
            continue
        ch = SPELLED.get(ch, ch)
        if all(" " <= c <= "~" for c in ch):
            out.append(ch)
    name = " ".join("".join(out).upper().split())
    name = re.sub(r"\b([DL])' ", r"\1'", name)
    return re.sub(r"(\w{3,})'(?=\s|$)", r"\1", name)


def polygons_of(geometry):
    if geometry["type"] == "Polygon":
        return [geometry["coordinates"]]
    if geometry["type"] == "MultiPolygon":
        return geometry["coordinates"]
    return []


def bbox(rings):
    xs = [p[0] for r in rings for p in r]
    ys = [p[1] for r in rings for p in r]
    return min(xs), min(ys), max(xs), max(ys)


# --- the biggest box ---------------------------------------------------------
# Measured with the east-west distance divided by the box's width over its
# height, a box of that shape is a square; and the biggest square centred on
# a point that fits inside an outline reaches just as far as the outline's
# nearest point in the L-infinity sense - the greater of the two distances,
# across and up. So the spot is polylabel's search (Mapbox's: the cells over
# the outline, best first, each split until none can beat the best found by
# more than the precision), with that distance in place of the usual one.


# The L-infinity distance from (px, py) to the segment from (ax, ay) by (dx, dy):
# the least, along it, of the greater of the two distances - which is at an
# end, or where one distance is nothing, or where the two are equal.
def seg_dist(px, py, ax, ay, dx, dy):
    ex, ey = ax - px, ay - py
    best = max(abs(ex), abs(ey))
    v = max(abs(ex + dx), abs(ey + dy))
    if v < best:
        best = v
    for num, den in ((-ex, dx), (-ey, dy), (ey - ex, dx - dy), (-(ex + ey), dx + dy)):
        if den:
            t = num / den
            if 0 < t < 1:
                v = max(abs(ex + t * dx), abs(ey + t * dy))
                if v < best:
                    best = v
    return best


class Outline:
    def __init__(self, rings):
        self.segs = []
        for ring in rings:
            for (ax, ay), (bx, by) in zip(ring, ring[1:]):
                self.segs.append((ax, ay, bx - ax, by - ay, min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)))

    def inside(self, x, y):
        c = False
        for ax, ay, dx, dy, *_ in self.segs:
            by = ay + dy
            if (ay > y) != (by > y) and x < ax + dx * (y - ay) / dy:
                c = not c
        return c

    # Positive inside, negative outside.
    def dist(self, x, y):
        best = math.inf
        for ax, ay, dx, dy, x0, y0, x1, y1 in self.segs:
            # No nearer than its bounding box.
            if max(x0 - x, x - x1, y0 - y, y - y1) >= best:
                continue
            v = seg_dist(x, y, ax, ay, dx, dy)
            if v < best:
                best = v
        return best if self.inside(x, y) else -best


def best_spot(outline, x0, y0, x1, y1, beat):
    size = min(x1 - x0, y1 - y0)
    if size <= 0 or size / 2 <= beat:
        return None
    precision = max(size * 0.004, 0.005)
    h = size / 2
    heap = []
    best = (-math.inf, 0, 0)

    def push(x, y, h):
        d = outline.dist(x, y)
        heapq.heappush(heap, (-(d + h), d, x, y, h))

    y = y0
    while y < y1:
        x = x0
        while x < x1:
            push(x + h, y + h, h)
            x += size
        y += size
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    d = outline.dist(cx, cy)
    best = (d, cx, cy)
    while heap:
        neg_max, d, x, y, h = heapq.heappop(heap)
        if d > best[0]:
            best = (d, x, y)
        if -neg_max - best[0] <= precision or -neg_max <= beat:
            break
        h /= 2
        for sx in (-h, h):
            for sy in (-h, h):
                push(x + sx, y + sy, h)
    return best if best[0] > beat else None


# Douglas-Peucker: the ring with every point less than `tol` off the line
# through its neighbours left out - the search's time goes on the segments.
def simplify(ring, tol):
    if len(ring) < 5:
        return ring
    keep = [False] * len(ring)
    keep[0] = keep[-1] = True
    stack = [(0, len(ring) - 1)]
    while stack:
        i, j = stack.pop()
        ax, ay = ring[i]
        bx, by = ring[j]
        dx, dy = bx - ax, by - ay
        n = math.hypot(dx, dy)
        far, at = -1.0, -1
        for k in range(i + 1, j):
            px, py = ring[k]
            d = abs(dx * (py - ay) - dy * (px - ax)) / n if n else math.hypot(px - ax, py - ay)
            if d > far:
                far, at = d, k
        if far > tol:
            keep[at] = True
            stack += [(i, at), (at, j)]
    return [p for p, k in zip(ring, keep) if k]


# The lines a name goes on: as one, or split at the space that leaves the
# longer line shortest.
def layouts(name):
    out = [[name]]
    spaces = [i for i, c in enumerate(name) if c == " "]
    if spaces:
        i = min(spaces, key=lambda i: max(i, len(name) - i - 1))
        out.append([name[:i], name[i + 1:]])
    return out


def box_shape(lines, kind):
    width = max(len(l) * (ADVANCE + SPACING[kind]) - SPACING[kind] - 1 for l in lines)
    height = len(lines) * HEIGHT + (len(lines) - 1) * LINE_GAP
    return width, height


# A name's spot and the nm a pixel of it can stand for, for the lines it is
# best on - or None, with no room for it anywhere.
def place(job):
    kind, name, polygons = job
    found = []
    for lines in layouts(name):
        width, height = box_shape(lines, kind)
        aspect = width / height
        best = None  # (d, lat, lon)
        parts = []
        for rings in polygons:
            lon0 = sum(p[0] for p in rings[0]) / len(rings[0])
            lat0 = sum(p[1] for p in rings[0]) / len(rings[0])
            k = 60 * math.cos(math.radians(lat0)) / aspect
            xy = [[((lon - lon0) * k, (lat - lat0) * 60) for lon, lat in ring] for ring in rings]
            bx0, by0, bx1, by1 = bbox(xy)
            parts.append((min(bx1 - bx0, by1 - by0), xy, (bx0, by0, bx1, by1), lat0, lon0, k))
        parts.sort(key=lambda p: -p[0])
        for size, xy, (bx0, by0, bx1, by1), lat0, lon0, k in parts:
            beat = best[0] if best else 0
            if size / 2 <= beat:
                break
            tol = max(size * 0.002, 0.01)
            spot = best_spot(Outline([simplify(r, tol) for r in xy]), bx0, by0, bx1, by1, beat)
            if spot:
                d, x, y = spot
                best = (d, lat0 + y / 60, lon0 + x / k)
        if best:
            found.append((2 * best[0] / height, best[1], best[2], lines))
    if not found:
        return None
    one = found[0]
    two = found[1] if len(found) > 1 else None
    nm, lat, lon, lines = two if two and two[0] > one[0] * TWO_LINES_GAIN else one
    return kind, "\n".join(lines), lat, lon, nm


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else None
    # By kind and name: the outlines with that name, near enough to each other
    # to be the one place.
    named = defaultdict(list)
    countries = set()
    for layer, kind, field in LAYERS:
        for feature in load(layer, directory)["features"]:
            props, geometry = feature["properties"], feature["geometry"]
            if not geometry or not props.get(field):
                continue
            if kind == SEA and props["featurecla"] not in SEA_CLASSES:
                continue
            name = clean(UNSPLIT.get(props["ADMIN"], props[field]) if kind == COUNTRY else props[field])
            if not name:
                continue
            if kind == COUNTRY:
                countries.add(name)
            # A region that is all of its country, as a small island's is.
            if kind == REGION and name in (clean(props["admin"] or ""), clean(props["geonunit"] or "")):
                continue
            polygons = [p for p in polygons_of(geometry) if p and len(p[0]) >= 4]
            if polygons:
                named[(kind, name)].append(polygons)

    jobs = []
    for (kind, name), features in named.items():
        # Together where within half a degree of each other.
        boxes = [bbox([r for p in polys for r in p]) for polys in features]
        group = list(range(len(features)))

        def root(i):
            while group[i] != i:
                group[i] = group[group[i]]
                i = group[i]
            return i

        for i in range(len(features)):
            for j in range(i):
                a, b = boxes[i], boxes[j]
                if a[0] - 0.5 <= b[2] and b[0] - 0.5 <= a[2] and a[1] - 0.5 <= b[3] and b[1] - 0.5 <= a[3]:
                    group[root(i)] = root(j)
        together = defaultdict(list)
        for i, polys in enumerate(features):
            together[root(i)] += polys
        for polys in together.values():
            jobs.append((kind, name, polys))

    with multiprocessing.Pool() as pool:
        results = [r for r in pool.imap_unordered(place, jobs, chunksize=4) if r]
    results.sort(key=lambda r: (r[2], r[3]))

    def literal(text):
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'

    out = [
        "// Generated by tools/gen_names.py from Natural Earth's 1:10m map units, first-order",
        "// divisions, marine areas and lakes - do not edit.",
        "// By latitude: where each name goes, the nm a pixel of it at size 1 can stand for",
        "// there, its kind, and the name, its lines apart at a newline.",
        "const PlaceName NAMES[] = {",
    ]
    for kind, text, lat, lon, nm in results:
        out.append(f"    {{{lat:.4f}f, {lon:.4f}f, {nm:.4e}f, {kind}, {literal(text)}}},")
    out.append("};")
    (ROOT / "src" / "names_table.inc").write_text("\n".join(out) + "\n")
    counts = defaultdict(int)
    for r in results:
        counts[r[0]] += 1
    print(f"{len(results)} names of {len(jobs)}: {counts[COUNTRY]} countries, {counts[SEA]} seas, "
          f"{counts[REGION]} regions, {counts[LAKE]} lakes")


if __name__ == "__main__":
    main()
