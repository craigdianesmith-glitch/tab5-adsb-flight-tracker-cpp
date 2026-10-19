#!/usr/bin/env python3
"""Airfield layouts from OpenStreetMap, for the 3D view - the real taxiways,
aprons, stands, holding points and buildings of the app's airports, where the
view otherwise makes up a layout from the runways alone.

    tools/gen_airfields.py fetch [--box S,W,N,E | --codes GLA,EDI] [--cache DIR]
    tools/gen_airfields.py extract FILE.osm.pbf [--box ... | --codes ...] [--cache DIR] [--nodes STORAGE]
    tools/gen_airfields.py build [--box ... | --codes ...] [--cache DIR] [--out airfields.bin]

fetch: each airport's - the civil table's in src/airports.cpp and the
military's in src/military_airports.inc - from the Overpass API: its
aerodrome's boundary, and everything inside it tagged aeroway (taxiways,
taxilanes, runways, aprons, holding positions, stands, gates, the tower),
its buildings, its windsocks. One airport a query, the answer kept in the
cache - by default ~/.cache/overhead-airfields - so a fetch that stops goes
on where it left off, and an airport is only asked for again once its file is
deleted. The public server takes half a minute or so an airport; it is asked
politely, one at a time, waiting between, and backing off when it is busy.
The default box is the UK and Ireland.

extract: the same answers, every airport's in the box that it has, from an
OpenStreetMap extract on disk - Geofabrik's, a country's or a continent's, or
the planet - rather than from the Overpass API, which would take days for
the world. Two passes over the file: the aerodromes within 2km of the app's
points, then everything inside them that fetch asks for. Needs pyosmium and
shapely; a continent's node locations want --nodes dense_file_array,FILE
(a sparse file on disk) rather than the default, in memory.

build: the cache made into the one file the Tab5 reads from its SD card,
airfields.bin - copied to the card's overhead folder. Each airport's layout in
the 3D view's own frame for it: metres east and north of the middle of its
open runways, as src/view3d.cpp's buildAirfield() works it out from the
runway table, so the two line up. Simplified - points dropped where the line
stays within half a metre - and packed as small differences from point to
point. Needs mapbox_earcut, shapely and numpy (pip install, in a venv).

The file, little-endian throughout:

    "OVAF", version 1, 0, the airport count (u16), then an index of them
    sorted by code: code (4 bytes, NUL padded), offset and length (u32 each).

    Each airport: its origin, latitude and longitude in 1e-7 degrees (i32
    each), then sections - a kind byte, a length (varint), and that many
    bytes, so a reader can step over one it doesn't know:

    1 junctions   n, then each point
    2 ways        n, then each: kind (1 taxiway, 2 taxilane, 3 runway),
                  width (u8, half metres), ref, from and to junction (varint
                  each), the count of points between and each of them
    3 holds       n, then each: way (varint), point, kind (1 runway, 2 other),
                  ref
    4 stands      n, then each: point, ref, and its lead-in line: count and
                  points (0 if none)
    5 surfaces    n, then each: kind (1 apron, 2 taxiway), the count of
                  corners and each, the count of triangles and each's three
                  corners (varint each)
    6 buildings   n, then each: kind (1 terminal, 2 hangar, 3 tower, 4 other),
                  height (u8, half metres), corners and triangles - the roof's
                  - as a surface's
    7 tower       point, height (u8, half metres); none if OSM hasn't one
    8 windsocks   n, then each point

    A point: its east and north from the point before it in its section, in
    quarter metres, zigzag varints - a section's first from (0, 0). A ref:
    its length (u8) and its bytes.

The layouts are (c) OpenStreetMap contributors, under the Open Database
License (ODbL 1.0): so is the file this makes, and the About screen says so.
"""

import argparse
import json
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OVERPASS = "https://overpass-api.de/api/interpreter"
UK_IRELAND = (49.8, -11.0, 61.0, 2.0)

QUERY = """[out:json][timeout:180];
(
  way["aeroway"="aerodrome"](around:{reach},{lat},{lon});
  relation["aeroway"="aerodrome"](around:{reach},{lat},{lon});
)->.field;
.field out geom;
.field map_to_area->.inside;
(
  nwr["aeroway"]["aeroway"!="aerodrome"](area.inside);
  nwr["building"](area.inside);
  node["man_made"~"^(tower|windsock)$"](area.inside);
  way["man_made"="tower"](area.inside);
);
out geom;"""


def airports():
    """Every airport the app knows: code -> (lat, lon, military)."""
    found = {}
    pattern = re.compile(r'\{"([A-Z0-9]{3,4})", (-?[\d.]+)f, (-?[\d.]+)f\}')
    for path, military in ((ROOT / "src" / "airports.cpp", False), (ROOT / "src" / "military_airports.inc", True)):
        for code, lat, lon in pattern.findall(path.read_text()):
            found.setdefault(code, (float(lat), float(lon), military))
    return found


def chosen(args):
    every = airports()
    if args.codes:
        codes = [c.strip().upper() for c in args.codes.split(",") if c.strip()]
        missing = [c for c in codes if c not in every]
        if missing:
            sys.exit(f"not in the app's tables: {', '.join(missing)}")
        return {c: every[c] for c in codes}
    s, w, n, e = (float(v) for v in args.box.split(",")) if args.box else UK_IRELAND
    return {c: v for c, v in sorted(every.items()) if s <= v[0] <= n and w <= v[1] <= e}


def fetch_one(code, lat, lon):
    # The aerodrome found within 2km of the app's point for it - the point
    # being the airport's reference, which is most often on the field.
    body = urllib.parse.urlencode({"data": QUERY.format(reach=2000, lat=lat, lon=lon)}).encode()
    request = urllib.request.Request(OVERPASS, data=body,
                                     headers={"User-Agent": "overhead-cpp airfield layouts (tools/gen_airfields.py)"})
    with urllib.request.urlopen(request, timeout=240) as response:
        return response.read()


def fetch(args):
    cache = Path(args.cache).expanduser()
    cache.mkdir(parents=True, exist_ok=True)
    todo = chosen(args)
    print(f"{len(todo)} airfields; cache {cache}", flush=True)
    done = failed = 0
    for i, (code, (lat, lon, military)) in enumerate(todo.items()):
        path = cache / f"{code}.json"
        if path.exists():
            continue
        for attempt in range(4):
            started = time.time()
            try:
                data = fetch_one(code, lat, lon)
                json.loads(data)  # a whole answer, or it is asked again
                path.write_bytes(data)
                elements = len(json.loads(data).get("elements", []))
                print(f"[{i + 1}/{len(todo)}] {code}{' (military)' if military else ''}: {elements} elements, "
                      f"{len(data) // 1024}KB in {time.time() - started:.0f}s", flush=True)
                done += 1
                break
            except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
                wait = 60 * (attempt + 1)
                print(f"[{i + 1}/{len(todo)}] {code}: {e} - again in {wait}s", flush=True)
                time.sleep(wait)
        else:
            failed += 1
        time.sleep(3)  # politely
    print(f"fetched {done}, failed {failed}, the rest already cached", flush=True)


# --- extract ------------------------------------------------------------------------

def extract(args):
    import math
    import osmium
    from shapely.affinity import affine_transform
    from shapely.geometry import LineString, Point, Polygon
    from shapely.strtree import STRtree

    cache = Path(args.cache).expanduser()
    cache.mkdir(parents=True, exist_ok=True)
    pbf = str(Path(args.pbf).expanduser())
    todo = chosen(args)
    started = time.time()

    def latlon(points):
        return [{"lat": round(p[1], 7), "lon": round(p[0], 7)} for p in points]

    def rings(area):
        out = []
        for outer in area.outer_rings():
            out.append(("outer", [(n.lon, n.lat) for n in outer]))
            for inner in area.inner_rings(outer):
                out.append(("inner", [(n.lon, n.lat) for n in inner]))
        return out

    def polygon(rings_):
        outers = [Polygon(r) for role, r in rings_ if role == "outer" and len(r) >= 4]
        inners = [Polygon(r) for role, r in rings_ if role == "inner" and len(r) >= 4]
        shape = None
        for o in outers:
            shape = o if shape is None else shape.union(o)
        for i in inners:
            shape = shape.difference(i) if shape is not None else None
        return shape if shape is not None and shape.is_valid and not shape.is_empty else (
            shape.buffer(0) if shape is not None else None)

    # 1. The aerodromes - a few thousand of the planet's billions of objects -
    # found, and their outlines made from their own nodes alone, without
    # every node's location kept: three quick reads, each filtered as the file
    # is decoded. Each as Overpass's .field has it - its element - and its
    # shape, from a closed way or a multipolygon's member ways.
    from shapely.ops import polygonize, unary_union
    aero_ways, aero_rels = {}, {}
    for o in (osmium.FileProcessor(pbf, osmium.osm.WAY | osmium.osm.RELATION)
              .with_filter(osmium.filter.TagFilter(("aeroway", "aerodrome")))):
        if o.is_way():
            aero_ways[o.id] = (dict(o.tags), [n.ref for n in o.nodes])
        elif o.is_relation():
            aero_rels[o.id] = (dict(o.tags), [(m.type, m.ref, m.role) for m in o.members])
    member_ids = {ref for _, members in aero_rels.values() for kind, ref, _ in members if kind == "w"}
    member_nodes = {}
    if member_ids:
        for o in osmium.FileProcessor(pbf, osmium.osm.WAY).with_filter(osmium.filter.IdFilter(member_ids)):
            member_nodes[o.id] = [n.ref for n in o.nodes]
    node_ids = {n for _, nodes in aero_ways.values() for n in nodes}
    node_ids.update(n for nodes in member_nodes.values() for n in nodes)
    where = {}
    if node_ids:
        for o in osmium.FileProcessor(pbf, osmium.osm.NODE).with_filter(osmium.filter.IdFilter(node_ids)):
            if o.location.valid():
                where[o.id] = (o.location.lon, o.location.lat)
    print(f"{len(aero_ways)} aerodromes as ways, {len(aero_rels)} as relations, {len(where)} of their nodes "
          f"({time.time() - started:.0f}s)", flush=True)

    def ring_shapes(lines):
        merged = unary_union([LineString(line) for line in lines if len(line) >= 2])
        return [p for p in polygonize(merged) if p.is_valid and p.area > 0]

    fields = []  # (element, shape)
    for way_id, (tags, nodes) in aero_ways.items():
        points = [where[n] for n in nodes if n in where]
        if len(points) < 4 or nodes[0] != nodes[-1] or len(points) != len(nodes):
            continue
        shape = Polygon(points)
        if not shape.is_valid:
            shape = shape.buffer(0)
        if shape.is_empty:
            continue
        fields.append(({"type": "way", "id": way_id, "nodes": nodes, "tags": tags, "geometry": latlon(points)},
                       shape))
    for rel_id, (tags, members) in aero_rels.items():
        lines = {"outer": [], "inner": []}
        for kind, ref, role in members:
            if kind != "w" or ref not in member_nodes:
                continue
            line = [where[n] for n in member_nodes[ref] if n in where]
            if len(line) >= 2:
                lines["inner" if role == "inner" else "outer"].append(line)
        outer = ring_shapes(lines["outer"])
        if not outer:
            continue
        shape = unary_union(outer)
        for hole in ring_shapes(lines["inner"]):
            shape = shape.difference(hole)
        if shape.is_empty:
            continue
        fields.append(({"type": "relation", "id": rel_id, "tags": tags,
                        "members": [{"type": "way", "ref": 0, "role": role, "geometry": latlon(line)}
                                    for role in ("outer", "inner") for line in lines[role]]}, shape))
    print(f"{len(fields)} aerodromes in {pbf} ({time.time() - started:.0f}s)", flush=True)

    # Each of the app's airports' aerodromes: those within 2km of its point,
    # measured in metres about it - as fetch's around:2000.
    tree = STRtree([shape for _, shape in fields])
    mine = {}  # field index -> codes
    found = {}  # code -> field indexes
    for code, (lat, lon, military) in todo.items():
        k = 111320.0 * math.cos(math.radians(lat))
        near = tree.query(Point(lon, lat).buffer(2500 / k))
        hits = []
        for i in near:
            # The distance in metres: the shape's corners taken about the point.
            metres = affine_transform(fields[i][1], [k, 0, 0, 110574.0, -lon * k, -lat * 110574.0])
            if metres.distance(Point(0, 0)) <= 2000:
                hits.append(int(i))
        if hits:
            found[code] = hits
            for i in hits:
                mine.setdefault(i, []).append(code)
    print(f"{len(found)} of the {len(todo)} airports have an aerodrome here", flush=True)
    if not found:
        return
    wanted = sorted(mine)
    shapes = [fields[i][1] for i in wanted]
    wanted_tree = STRtree(shapes)
    elements = {code: [fields[i][0] for i in found[code]] for code in found}

    def owners(geometry):
        return {c for j in wanted_tree.query(geometry, predicate="intersects") for c in mine[wanted[int(j)]]}

    # Cells of a fiftieth of a degree over each wanted aerodrome and one more
    # round it: most of the world's buildings are passed over by the cell of
    # their first corner, before any shape is made for them.
    STEP = 50
    near = set()
    for shape in shapes:
        w, s_, e, n = shape.bounds
        for y in range(math.floor(s_ * STEP) - 1, math.floor(n * STEP) + 2):
            for x in range(math.floor(w * STEP) - 1, math.floor(e * STEP) + 2):
                near.add((y, x))

    def nearby(lon, lat):
        return (math.floor(lat * STEP), math.floor(lon * STEP)) in near

    if args.near:
        extract_near(args, pbf, shapes, owners, elements, cache, started)
        return

    # 2. Everything inside them that fetch asks for: aeroway other than the
    # aerodrome, buildings, towers and windsocks - a multipolygon's as its
    # rings, which build joins as it would Overpass's member ways.
    fp = (osmium.FileProcessor(pbf).with_locations(args.nodes)
          .with_areas(osmium.filter.KeyFilter("aeroway", "building"))
          .with_filter(osmium.filter.KeyFilter("aeroway", "building", "man_made")))
    kept = seen = 0
    shown = time.time()
    for o in fp:
        seen += 1
        if seen % 1000000 == 0 and time.time() - shown > 60:
            shown = time.time()
            print(f"  {seen // 1000000}M looked at, {kept} kept ({time.time() - started:.0f}s)", flush=True)
        if o.is_area() and o.from_way():
            continue  # its way is taken as it is
        tags = o.tags
        aeroway, man_made = tags.get("aeroway"), tags.get("man_made")
        if aeroway == "aerodrome":
            continue
        if o.is_area():
            if not (aeroway or "building" in tags):
                continue
            first = next(iter(o.outer_rings()), None)
            if first is None or len(first) == 0 or not nearby(first[0].lon, first[0].lat):
                continue
            rs = rings(o)
            shape = polygon(rs)
            if shape is None:
                continue
            codes = owners(shape)
            if codes:
                element = {"type": "relation", "id": o.orig_id(), "tags": dict(tags),
                           "members": [{"type": "way", "ref": 0, "role": role, "geometry": latlon(r)}
                                       for role, r in rs]}
        elif o.is_node():
            if not (aeroway or "building" in tags or man_made in ("tower", "windsock")):
                continue
            if not o.location.valid() or not nearby(o.location.lon, o.location.lat):
                continue
            codes = owners(Point(o.location.lon, o.location.lat))
            if codes:
                element = {"type": "node", "id": o.id, "lat": round(o.location.lat, 7),
                           "lon": round(o.location.lon, 7), "tags": dict(tags)}
        elif o.is_way():
            if not (aeroway or "building" in tags or man_made == "tower"):
                continue
            try:
                if len(o.nodes) == 0 or not nearby(o.nodes[0].lon, o.nodes[0].lat):
                    continue
                points = [(n.lon, n.lat) for n in o.nodes]
            except osmium.InvalidLocationError:
                continue
            if len(points) < 2:
                continue
            codes = owners(LineString(points) if points[0] != points[-1] or len(points) < 4 else Polygon(points))
            if codes:
                element = {"type": "way", "id": o.id, "nodes": [n.ref for n in o.nodes], "tags": dict(tags),
                           "geometry": latlon(points)}
        else:
            continue
        if codes:
            kept += 1
            for code in codes:
                elements[code].append(element)
    for code, items in elements.items():
        (cache / f"{code}.json").write_text(json.dumps({"elements": items}))
    print(f"{kept} elements for {len(elements)} airports, cached in {cache} ({time.time() - started:.0f}s)",
          flush=True)


def extract_near(args, pbf, shapes, owners, elements, cache, started):
    """extract's second step for the planet, by tools/airfield_near: the cells
    over and round each aerodrome wanted - a 200th of a degree square, and one
    more about it - written for it to keep the nodes of; then each object it
    finds there shared out among the airports whose aerodromes it lies in."""
    import math
    import subprocess
    import tempfile
    from shapely.geometry import LineString, MultiLineString, Point, box

    cells = set()
    for shape in shapes:
        w, s, e, n = shape.bounds
        grown = shape.buffer(1 / 200)
        for y in range(math.floor(s * 200) - 1, math.floor(n * 200) + 2):
            for x in range(math.floor(w * 200) - 1, math.floor(e * 200) + 2):
                if grown.intersects(box(x / 200, y / 200, (x + 1) / 200, (y + 1) / 200)):
                    cells.add((y, x))
    with tempfile.NamedTemporaryFile("w", suffix=".cells", delete=False) as f:
        f.write("".join(f"{y} {x}\n" for y, x in cells))
        cells_path = f.name
    print(f"{len(cells)} cells for the near tool ({time.time() - started:.0f}s)", flush=True)
    kept = read = 0
    proc = subprocess.Popen([str(Path(args.near).expanduser()), pbf, cells_path], stdout=subprocess.PIPE)
    for line in proc.stdout:
        e = json.loads(line)
        read += 1
        if e["type"] == "node":
            geometry = Point(e["lon"], e["lat"])
        elif e["type"] == "way":
            geometry = LineString([(p["lon"], p["lat"]) for p in e["geometry"]])
        else:
            lines = [[(p["lon"], p["lat"]) for p in m["geometry"]] for m in e["members"] if len(m["geometry"]) >= 2]
            if not lines:
                continue
            geometry = MultiLineString(lines)
        codes = owners(geometry)
        if codes:
            kept += 1
            for code in codes:
                elements[code].append(e)
    if proc.wait() != 0:
        sys.exit(f"the near tool failed: {proc.returncode}")
    Path(cells_path).unlink()
    for code, items in elements.items():
        (cache / f"{code}.json").write_text(json.dumps({"elements": items}))
    print(f"{kept} of {read} found kept, for {len(elements)} airports, cached in {cache} "
          f"({time.time() - started:.0f}s)", flush=True)


# --- build --------------------------------------------------------------------------

UNIT = 0.25  # metres a step, for points
SIMPLIFY_M = 0.5  # a line's points dropped while it stays within this
WIDTH = {"taxiway": 18.0, "taxilane": 12.0, "runway": 45.0}
KIND = {"taxiway": 1, "taxilane": 2, "runway": 3}
MIN_BUILDING_M2 = 150  # smaller than this - sheds, kiosks - left out, unless an airport building
MOST_BUILDINGS = 150  # the largest, for the 3D view to keep up: an RAF station maps 450
CLUTTER = {"house", "detached", "semidetached_house", "residential", "terrace", "garage", "garages", "shed",
           "hut", "cabin", "roof", "carport", "kiosk", "toilets", "bunker", "container"}


def runway_origins():
    """Each airport's origin as the 3D view takes it: the mean of its open
    runways' middles, from the float32 values the firmware's tables hold."""
    import numpy as np
    pattern = re.compile(r'\{"([A-Z0-9]{3,4})", \d+, -?\d+, (true|false), \{\{"[^"]*", (-?[\d.]+)f, (-?[\d.]+)f\}, '
                         r'\{"[^"]*", (-?[\d.]+)f, (-?[\d.]+)f\}\}\}')
    sums = {}
    for path in (ROOT / "src" / "runways_table.inc", ROOT / "src" / "military_runways.inc"):
        for code, closed, a, b, c, d in pattern.findall(path.read_text()):
            if closed == "true":
                continue
            f = [float(np.float32(v)) for v in (a, b, c, d)]
            lat, lon, n = sums.get(code, (0.0, 0.0, 0))
            sums[code] = (lat + (f[0] + f[2]) / 2.0, lon + (f[1] + f[3]) / 2.0, n + 1)
    return {code: (lat / n, lon / n) for code, (lat, lon, n) in sums.items()}


class Frame:
    """The 3D view's frame for an airport: as view3d.cpp's local()."""

    def __init__(self, lat0, lon0):
        import math
        self.lat0, self.lon0 = lat0, lon0
        self.kx = 111320.0 * math.cos(lat0 * math.pi / 180)

    def __call__(self, lat, lon):
        return ((lon - self.lon0) * self.kx, (lat - self.lat0) * 110574.0)


def varint(n):
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def zigzag(n):
    return (n << 1) ^ (n >> 63)


class Points:
    """A section's points, each as its difference from the one before."""

    def __init__(self):
        self.x = self.y = 0

    def __call__(self, p):
        x, y = round(p[0] / UNIT), round(p[1] / UNIT)
        out = varint(zigzag(x - self.x)) + varint(zigzag(y - self.y))
        self.x, self.y = x, y
        return out


def ref_bytes(tags):
    ref = (tags.get("ref") or "").encode("ascii", "replace")[:15]
    return bytes([len(ref)]) + ref


# "Control tower" in its name, in the languages most often met: Fukuoka's is
# a point named 福岡空港管制塔 alone.
CONTROL_TOWER_NAMES = ("control tower", "traffic control", "tour de contr", "torre de control", "torre di controllo",
                       "kontrollturm", "flugsicherung", "管制塔", "塔台", "관제탑", "диспетчерск")


def control_tower(tags, building=False):
    """How surely OpenStreetMap's tags are the airport's control tower: 0 not
    at all. 3 as an aeroway or a building's kind, or by its name - Las Vegas's
    "FAA Airport Traffic Control Tower", tagged a communication tower; 2 a
    tower whose kind says so - air_traffic_control, aircraft_control (as
    Heathrow's and Edinburgh's are), airport_control and the rest; 1 an
    observation tower, or a building that is a tower of no kind or of one of
    its own (Newcastle's newcastle_tower). Never the radar's mast, a
    communication, lighting or cooling tower - which a tower alone took in -
    nor a disused or old one."""
    kind = (tags.get("tower:type") or "").lower()
    name = (tags.get("name") or "").lower()
    words = set(re.split(r"[^a-z]+", kind + " " + name))
    if words & {"disused", "old", "former", "abandoned", "historic", "demolished"}:
        return 0
    if tags.get("aeroway") in ("control_tower", "tower") or tags.get("building") in ("control_tower", "tower"):
        return 3
    if tags.get("man_made") != "tower":
        return 0
    if any(n in name for n in CONTROL_TOWER_NAMES) or "atc" in words:
        return 3
    if words & {"control", "atc", "aircraft", "airport"}:
        return 2
    if kind == "observation":
        return 1
    if building and (not kind or (kind.endswith("_tower") and kind not in ("bell_tower", "water_tower"))):
        return 1
    return 0


def height_of(tags, kind):
    for key in ("height", "building:height"):
        m = re.match(r"\s*([\d.]+)", tags.get(key, ""))
        if m:
            return float(m.group(1))
    m = re.match(r"\s*(\d+)", tags.get("building:levels", ""))
    if m:
        return int(m.group(1)) * 3.5 + 1
    return {1: 16.0, 2: 14.0, 3: 30.0}.get(kind, 8.0)


def rings_of(element):
    """A closed way's ring, or a multipolygon's outer and inner rings, as
    lists of (lat, lon)."""
    if element["type"] == "way":
        g = [(p["lat"], p["lon"]) for p in element.get("geometry", []) if p]
        return [g], []
    outer, inner = [], []
    for m in element.get("members", []):
        g = [(p["lat"], p["lon"]) for p in m.get("geometry", []) or [] if p]
        if len(g) >= 2:
            (inner if m.get("role") == "inner" else outer).append(g)
    return join(outer), join(inner)


def join(pieces):
    """A multipolygon's ways, which may each be only part of a ring, joined
    end to end into whole rings."""
    rings, open_ = [], [list(p) for p in pieces]
    while open_:
        ring = open_.pop()
        while ring[0] != ring[-1]:
            for i, p in enumerate(open_):
                if p[0] == ring[-1]:
                    ring += p[1:]
                elif p[-1] == ring[-1]:
                    ring += p[::-1][1:]
                else:
                    continue
                open_.pop(i)
                break
            else:
                break  # can't close it: dropped
        if ring[0] == ring[-1] and len(ring) >= 4:
            rings.append(ring)
    return rings


def triangulated(polygon):
    """A shapely polygon's corners - outer ring then holes - and triangles."""
    import mapbox_earcut as earcut
    import numpy as np
    from shapely.geometry.polygon import orient
    polygon = orient(polygon)
    rings = [list(polygon.exterior.coords)[:-1]] + [list(r.coords)[:-1] for r in polygon.interiors]
    rings = [r for r in rings if len(r) >= 3]
    if not rings:
        return [], []
    corners = [p for r in rings for p in r]
    ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
    tri = earcut.triangulate_float64(np.array(corners, dtype=np.float64), ends)
    return corners, [tuple(int(v) for v in tri[i:i + 3]) for i in range(0, len(tri), 3)]


def polygons(element, frame, tolerance):
    """An area's polygons in the frame, made valid and simplified."""
    from shapely.geometry import MultiPolygon, Polygon
    from shapely.validation import make_valid
    outer, inner = rings_of(element)
    found = []
    for ring in outer:
        holes = [[frame(*p) for p in h] for h in inner]
        try:
            poly = make_valid(Polygon([frame(*p) for p in ring], [h for h in holes if len(h) >= 4]))
        except ValueError:
            continue
        for g in getattr(poly, "geoms", [poly]):
            if isinstance(g, Polygon) and g.area > 1:
                g = g.simplify(tolerance, preserve_topology=True)
                if isinstance(g, (Polygon,)) and not g.is_empty:
                    found.append(g)
                elif isinstance(g, MultiPolygon):
                    found += list(g.geoms)
    return found


def surface_bytes(corners, triangles, points):
    out = varint(len(corners)) + b"".join(points(p) for p in corners)
    out += varint(len(triangles)) + b"".join(varint(a) + varint(b) + varint(c) for a, b, c in triangles)
    return out


def section(kind, payload, count):
    body = varint(count) + payload
    return bytes([kind]) + varint(len(body)) + body


def layout(code, data, frame):
    """One airport's block, and a few numbers about it for the log."""
    from shapely.geometry import LineString, Point
    elements = data.get("elements", [])
    nodes = {e["id"]: e for e in elements if e["type"] == "node"}

    # The network: taxiways, taxilanes and runways, split at every node two
    # of them share - and at holding points and their ends - into ways
    # between junctions.
    lines = [e for e in elements if e["type"] == "way" and e.get("tags", {}).get("aeroway") in KIND
             and e.get("tags", {}).get("area") != "yes" and len(e.get("nodes", [])) >= 2]
    used = {}
    for w in lines:
        for n in w["nodes"]:
            used[n] = used.get(n, 0) + 1
    holds = {e["id"]: e for e in elements if e["type"] == "node"
             and e.get("tags", {}).get("aeroway") == "holding_position"}
    position = {}
    for w in lines:
        for n, g in zip(w["nodes"], w.get("geometry", [])):
            if g:
                position[n] = frame(g["lat"], g["lon"])
    junction_of, junctions = {}, []

    def junction(n):
        if n not in junction_of:
            junction_of[n] = len(junctions)
            junctions.append(position[n])
        return junction_of[n]

    ways, hold_rows = [], []
    for w in lines:
        tags = w["tags"]
        kind = tags["aeroway"]
        m = re.match(r"\s*([\d.]+)", tags.get("width", ""))
        width = float(m.group(1)) if m else WIDTH[kind]
        ids = [n for n in w["nodes"] if n in position]
        cut = [i for i, n in enumerate(ids) if i == 0 or i == len(ids) - 1 or used[n] > 1 or n in holds]
        for a, b in zip(cut, cut[1:]):
            run = ids[a:b + 1]
            pts = [position[n] for n in run]
            if len(pts) > 2:
                kept = LineString(pts).simplify(SIMPLIFY_M, preserve_topology=False)
                pts = list(kept.coords)
            index = len(ways)
            ways.append((KIND[kind], width, tags, junction(run[0]), junction(run[-1]), pts[1:-1]))
            for n in (run[0], run[-1]):
                if n in holds and not any(h[0] == n for h in hold_rows):
                    hold_rows.append((n, index))

    out = bytearray()
    pts = Points()
    out += section(1, b"".join(pts(p) for p in junctions), len(junctions))
    pts, body = Points(), bytearray()
    for kind, width, tags, a, b, between in ways:
        body += bytes([kind, min(255, round(width * 2))]) + ref_bytes(tags) + varint(a) + varint(b)
        body += varint(len(between)) + b"".join(pts(p) for p in between)
    out += section(2, bytes(body), len(ways))
    pts, body = Points(), bytearray()
    for n, way in hold_rows:
        tags = holds[n].get("tags", {})
        kind = 1 if tags.get("holding_position:type", "runway") in ("runway", "ILS") else 2
        body += varint(way) + pts(position[n]) + bytes([kind]) + ref_bytes(tags)
    out += section(3, bytes(body), len(hold_rows))

    # Stands: a node, or a way - the lead-in line, ending at the stand.
    pts, body, stands = Points(), bytearray(), 0
    for e in elements:
        tags = e.get("tags", {})
        if tags.get("aeroway") != "parking_position":
            continue
        if e["type"] == "node":
            at, lead = frame(e["lat"], e["lon"]), []
        elif e["type"] == "way" and e.get("geometry"):
            line = [frame(g["lat"], g["lon"]) for g in e["geometry"] if g]
            if len(line) < 2:
                continue
            if len(line) > 2:
                line = list(LineString(line).simplify(SIMPLIFY_M).coords)
            at, lead = line[-1], line
        else:
            continue
        body += pts(at) + ref_bytes(tags) + varint(len(lead)) + b"".join(pts(p) for p in lead)
        stands += 1
    out += section(4, bytes(body), stands)

    # Surfaces: aprons, and taxiways drawn as areas.
    pts, body, surfaces = Points(), bytearray(), 0
    for e in elements:
        tags = e.get("tags", {})
        kind = 1 if tags.get("aeroway") == "apron" else 2 if (
            tags.get("aeroway") in ("taxiway", "taxilane") and tags.get("area") == "yes") else 0
        if not kind or e["type"] not in ("way", "relation"):
            continue
        for poly in polygons(e, frame, 1.0):
            corners, triangles = triangulated(poly)
            if triangles:
                body += bytes([kind]) + surface_bytes(corners, triangles, pts)
                surfaces += 1
    out += section(5, bytes(body), surfaces)

    # Buildings - and the tower, if one is a building. The airport's own -
    # terminals, hangars, the tower - all of them; the rest only if they are
    # big enough to see, and not houses, garages or sheds; and of those only
    # the largest, as many as the 3D view can draw.
    tower, sure = None, 0
    found = []
    for e in elements:
        tags = e.get("tags", {})
        aeroway = tags.get("aeroway", "")
        if e["type"] not in ("way", "relation"):
            continue
        # A control tower mapped as a tower's outline and not a building -
        # SeaTac's - is one too.
        score = control_tower(tags, building=True)
        if not ("building" in tags or aeroway in ("terminal", "hangar", "control_tower", "tower") or score >= 2):
            continue
        kind = (1 if aeroway == "terminal" or tags.get("building") == "terminal" else
                2 if aeroway == "hangar" or tags.get("building") == "hangar" else
                3 if score else 4)
        if kind == 4 and tags.get("building") in CLUTTER:
            continue
        height = height_of(tags, kind)
        for poly in polygons(e, frame, 0.5):
            if kind == 4 and poly.area < MIN_BUILDING_M2:
                continue
            # The bigger the building the coarser: a terminal 300m long has
            # its outline traced to the metre, which nobody sees from the air
            # and which cost Heathrow's 3D view 9,000 triangles.
            tolerance = max(0.5, poly.area ** 0.5 / 80)
            if tolerance > 0.5:
                poly = poly.simplify(tolerance, preserve_topology=True)
                if poly.is_empty or poly.geom_type != "Polygon":
                    continue
            found.append((kind, height, poly, score))
    found.sort(key=lambda b: (b[0] == 4, -b[2].area))
    pts, body, buildings = Points(), bytearray(), 0
    for kind, height, poly, score in found[:max(MOST_BUILDINGS, sum(1 for b in found if b[0] != 4))]:
        corners, triangles = triangulated(poly)
        if not triangles:
            continue
        body += bytes([kind, min(255, round(height * 2))]) + surface_bytes(corners, triangles, pts)
        buildings += 1
        # The tower: the surest of them, the largest of those - the list
        # being largest first.
        if kind == 3 and score > sure:
            c = poly.centroid
            tower, sure = ((c.x, c.y), height), score
    out += section(6, bytes(body), buildings)

    # Or a point, where that is surer: Heathrow's is a point alone.
    for e in elements:
        tags = e.get("tags", {})
        score = control_tower(tags) if e["type"] == "node" else 0
        if score > sure:
            tower, sure = (frame(e["lat"], e["lon"]), height_of(tags, 3)), score
    if tower is not None:
        (tx, ty), th = tower
        body = Points()((tx, ty)) + bytes([min(255, round(th * 2))])
        out += bytes([7]) + varint(len(body)) + body

    pts, body, socks = Points(), bytearray(), 0
    for e in elements:
        tags = e.get("tags", {})
        if e["type"] == "node" and (tags.get("aeroway") == "windsock" or tags.get("man_made") == "windsock"):
            body += pts(frame(e["lat"], e["lon"]))
            socks += 1
    out += section(8, bytes(body), socks)

    head = int(round(frame.lat0 * 1e7)).to_bytes(4, "little", signed=True) + \
        int(round(frame.lon0 * 1e7)).to_bytes(4, "little", signed=True)
    counts = dict(junctions=len(junctions), ways=len(ways), holds=len(hold_rows), stands=stands,
                  surfaces=surfaces, buildings=buildings, tower=tower is not None, windsocks=socks)
    return head + bytes(out), counts


def build(args):
    cache = Path(args.cache).expanduser()
    origins = runway_origins()
    blocks, skipped = {}, []
    for code in sorted(chosen(args)):
        path = cache / f"{code}.json"
        if not path.exists():
            skipped.append(f"{code} (not fetched)")
            continue
        if code not in origins:
            skipped.append(f"{code} (no open runway in the table)")
            continue
        data = json.loads(path.read_text())
        if not any(e.get("tags", {}).get("aeroway") for e in data.get("elements", [])):
            skipped.append(f"{code} (nothing mapped)")
            continue
        block, counts = layout(code, data, Frame(*origins[code]))
        blocks[code] = block
        print(f"{code}: {len(block) / 1024:.1f}KB - " + ", ".join(f"{k} {v}" for k, v in counts.items()), flush=True)
    index_at = 16
    data_at = index_at + 12 * len(blocks)
    head = b"OVAF" + bytes([1, 0]) + len(blocks).to_bytes(2, "little") + index_at.to_bytes(4, "little") + \
        data_at.to_bytes(4, "little")
    index, body = bytearray(), bytearray()
    for code, block in sorted(blocks.items()):
        index += code.encode().ljust(4, b"\0") + (data_at + len(body)).to_bytes(4, "little") + \
            len(block).to_bytes(4, "little")
        body += block
    Path(args.out).write_bytes(head + bytes(index) + bytes(body))
    print(f"{len(blocks)} airfields, {(16 + len(index) + len(body)) / 1024:.0f}KB -> {args.out}")
    if skipped:
        print("skipped: " + ", ".join(skipped))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("what", choices=["fetch", "extract", "build"])
    ap.add_argument("pbf", nargs="?", help="extract: the OpenStreetMap extract, .osm.pbf")
    ap.add_argument("--near", help="extract: tools/airfield_near's program, built - for the planet, whose every "
                                   "node's location pyosmium can't keep in memory")
    ap.add_argument("--nodes", default="flex_mem",
                    help="extract: where node locations are kept - flex_mem, or dense_file_array,FILE for a "
                         "continent or the planet")
    ap.add_argument("--box", help="S,W,N,E in degrees; the UK and Ireland if neither this nor --codes")
    ap.add_argument("--codes", help="airport codes, comma separated, as the app's tables have them")
    ap.add_argument("--cache", default="~/.cache/overhead-airfields")
    ap.add_argument("--out", default=str(ROOT / "airfields.bin"))
    args = ap.parse_args()
    if args.what == "fetch":
        fetch(args)
    elif args.what == "extract":
        if not args.pbf:
            sys.exit("extract needs the .osm.pbf file")
        extract(args)
    else:
        build(args)


if __name__ == "__main__":
    main()
