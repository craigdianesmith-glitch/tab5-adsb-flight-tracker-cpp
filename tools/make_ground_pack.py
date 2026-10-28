#!/usr/bin/env python3
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Craig Smith. Overhead's terrain engine: PolyForm Noncommercial 1.0.0
# (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.
"""Makes a ground pack - ground.bin, the land and its hills for Overhead's 3D
view - for the part of the world you choose: your country, your state or
region, or the country round your town.

The pack each release carries is the United Kingdom and Ireland's. With one
of your own on the Tab5's card instead, the 3D view draws your land in what
covers it - woods, fields, towns, water, bare rock and snow - lit and shaded
as its hills lie, and at its height - its hills and mountains. Without a
pack the land is flat and one colour.

Run it with nothing after it, and it asks where, shows what that will take,
and asks before it starts:

    python make_ground_pack.py

Or say where on the command line: a country, by its name or its two- or
three-letter code; a state, province or region of one; several of them; a
circle round a place; or a box of latitude and longitude:

    python make_ground_pack.py Luxembourg
    python make_ground_pack.py France Belgium
    python make_ground_pack.py Texas
    python make_ground_pack.py Scotland
    python make_ground_pack.py "Limburg, Belgium"
    python make_ground_pack.py --around 46.02,7.75 --radius 25
    python make_ground_pack.py --around Zermatt --radius 25
    python make_ground_pack.py --south 54.4 --north 54.8 --west -3.6 --east -2.6
    python make_ground_pack.py Malta --out malta.bin
    python make_ground_pack.py Texas --estimate      (what it would take, and stop)
    python make_ground_pack.py --list                (the countries it knows)
    python make_ground_pack.py --list Germany        (a country's states and regions)
    python make_ground_pack.py --list york           (every name with "york" in it)

On Windows, type py where these say python.


GETTING READY - once

It needs Python 3.8 or newer, and three free add-ons for it: numpy, rasterio
and Pillow. The file requirements.txt beside this one lists them; and
gen_ground.py, beside it too, makes the tiles.

  Windows
    1. Install Python from https://www.python.org/downloads/ - on its first
       screen, tick "Add python.exe to PATH".
    2. Get this project's files: on its GitHub page, Code, then Download ZIP;
       unzip it, and open its tools folder.
    3. Click the folder's address bar, type cmd and press Enter: a black
       window opens, in that folder.
    4. Type   py -m pip install -r requirements.txt   and press Enter.
    5. Then   py make_ground_pack.py   - and answer its questions.

  macOS
    1. Install Python from https://www.python.org/downloads/ (it also needs
       "Install Certificates.command", in its folder under Applications, run
       once, for downloads to work).
    2. Get this project's files, as above, and open Terminal in its tools
       folder: type cd and a space, drag the folder onto Terminal, Enter.
    3. python3 -m venv ~/groundpack
       ~/groundpack/bin/pip install -r requirements.txt
    4. ~/groundpack/bin/python make_ground_pack.py

  Linux
    The same as macOS, with your system's python3 - on Debian and Ubuntu,
    sudo apt install python3-venv first.

It reads the maps from the internet as it goes - nothing to download first -
so it needs a connection the whole time, the faster the better.


WHAT IT MAKES

  ground.bin             the pack, for the card (or the name given with --out)
  ground-NOTICE.txt      the data's credits - keep it with the pack if you
                         pass the pack on
  ground-preview.png     a picture of the pack: what covers the land, shaded
                         by its hills - to see what you made

Then copy ground.bin to the Tab5's microSD card, into a folder named
overhead at the top of the card - overhead/ground.bin - the card FAT32, as
for the airfield pack. The Tab5 reads one ground pack: this one takes the
place of any that was there. To have two countries, name both in one pack.


HOW BIG

The pack is made of tiles 0.2 degrees square - some 22km north to south, and
narrower further from the equator. Only the tiles your areas touch are made,
and a margin of some 30km round them (--margin), so the hills just over a
border or across the water are there too; tiles of nothing but open sea are
left out - the 3D view draws the sea the same without them. A country's
parts far from the rest - France's overseas regions, say - are left out, and
said so: name one on its own to make a pack of it.

The Tab5 holds every tile's coarsest level in its memory while the 3D view
is open - for the far country and its heights - so a pack can have at most
6,000 tiles (MAX_TILES), those coarse levels can come to at most 4MB
(MAX_COARSE_BYTES), and the rectangle of tiles round it all at most 40,000
places (MAX_GRID): the firmware's own numbers, in include/ground_pack.h,
which refuses a pack past them. That is room for a country the size of
France, Germany, Spain or Sweden, or a circle some 1,200km across. It says
before it starts, and won't make one too big - try a region, or --around
your home with a --radius.

Luxembourg and some 30km round it is 84 tiles and 3MB, made in a couple of
minutes; Switzerland and the land round it, 337 tiles and 18MB, in seven
minutes on a computer busy with other work. The release's pack of the United
Kingdom and Ireland - 2,068 tiles and 27MB, made by tools/gen_ground.py,
whose tiles this makes too - took twelve minutes on a quiet one with a fast
connection.


IF IT STOPS

Each tile, as it is made, is kept in a cache folder - on Windows under
%LOCALAPPDATA%, on a Mac under ~/Library/Caches, on Linux under ~/.cache, in
overhead-ground-pack (or --cache) - so if the connection drops, the laptop
is closed or you stop it with Ctrl+C, the same command again carries on
where it stopped. A pack is only written once every tile is in, to a
temporary file first, then renamed: never half a pack. The cache can be
deleted once you have the pack you want; it only saves time.


THE DATA

  - What covers the land: ESA WorldCover 2021 v200, 10m - © ESA WorldCover
    project 2021 / Contains modified Copernicus Sentinel data (2021)
    processed by ESA WorldCover consortium, CC BY 4.0.
  - The hills: the Copernicus DEM GLO-30 and GLO-90 - the pack is produced
    using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence
    and Space GmbH 2014-2018 provided under COPERNICUS by the European Union
    and ESA; all rights reserved, and produced using Copernicus WorldDEM™-90
    © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018
    provided under COPERNICUS by the European Union and ESA; all rights
    reserved. Where the GLO-30 isn't public - Armenia and Azerbaijan - the
    hills' light and shade come from the GLO-90 instead, coarser, and the
    summary says so.
  - The countries and regions, to choose by: Natural Earth
    (naturalearthdata.com), public domain.
  - A town's place, for --around by name: Open-Meteo's geocoding API
    (open-meteo.com), its data CC BY 4.0.

The tiles themselves are tools/gen_ground.py's, which this imports: see that
file for the pack's format and how a tile is made. One thing differs: the
moor - grass above a line from 400m on the south coast to 150m in Shetland,
or steep - is the British Isles' hill grazing, so it is drawn only on tiles
in or near the United Kingdom, Ireland, the Isle of Man and the Channel
Islands, and grass is grass everywhere else (--moor on or off to choose).
And a box - --south, --north, --west and --east - is gen_ground.py's own pack
of it, byte for byte, its open sea kept as that keeps it: with --moor on
where the box reaches past the British Isles, as the release's does into
France.
"""

import argparse
import concurrent.futures
import datetime
import difflib
import gzip
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import signal
import ssl
import struct
import sys
import textwrap
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request
import zlib
from pathlib import Path

if sys.version_info < (3, 8):
    sys.exit("make_ground_pack.py needs Python 3.8 or newer - see the top of this file for how to get it.")

try:
    import numpy as np
    import rasterio
    import rasterio.features
    from rasterio.transform import from_origin
    from PIL import Image, ImageDraw, ImageFont
except ImportError as missing:
    # Into this very Python - a venv's, say - not whichever "python" is first.
    pip = ("py" if os.name == "nt" else shlex.quote(sys.executable)) + " -m pip install -r requirements.txt"
    sys.exit(f"make_ground_pack.py needs {missing.name}, which isn't installed yet.\n"
             f"In this folder, run:  {pip}\n"
             "- then try again.")

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_ground as gg  # noqa: E402 - the tiles, made as the release's pack's are

# What the Tab5 can hold: include/ground_pack.h's MAX_TILES,
# MAX_COARSE_BYTES and MAX_GRID, the same numbers - change one, change both.
# A pack past any of them is refused as it opens, and the land drawn flat.
# Its tiles; every tile's coarse block, inflated into its memory as the pack
# opens - level 4 and its heights, three bytes a cell of 18 rows by cols0/16,
# 650 bytes a tile between 50 and 60 degrees, 970 nearer the equator; and its
# grid - every place of the rectangle of tiles round them all, a tile there
# or not - 22 bytes each, its index to the tiles (an int16) and five
# pointers. One country's grid is a few thousand places; areas a continent
# apart, a hundred thousand.
MAX_TILES = 6000
MAX_COARSE_BYTES = 4 << 20
MAX_GRID = 40000
GRID_BYTES = 22


def firmware_limits():
    """The firmware's own limits, read from include/ground_pack.h where this
    file is in the project beside it - so that the two can't part company.
    Where it isn't, or doesn't say, the numbers above."""
    global MAX_TILES, MAX_COARSE_BYTES, MAX_GRID
    try:
        text = (Path(__file__).resolve().parent.parent / "include" / "ground_pack.h").read_text(encoding="utf-8")
    except OSError:
        return

    def value(name, default):
        m = re.search(rf"constexpr\s+[\w:]+\s+{name}\s*=\s*(\d+)[uU]?\s*(?:<<\s*(\d+))?\s*;", text)
        return int(m[1]) << int(m[2] or 0) if m else default

    MAX_TILES = value("MAX_TILES", MAX_TILES)
    MAX_COARSE_BYTES = value("MAX_COARSE_BYTES", MAX_COARSE_BYTES)
    MAX_GRID = value("MAX_GRID", MAX_GRID)
ENTRY_BYTES = 28  # a tile's place in the index, as the Tab5 keeps it

MARGIN_KM = 30  # round the areas chosen
LINK_KM = 1000  # a country's parts further than this from the rest are left out
RADIUS_KM = 150  # round a place, unless told
# For the estimates: the pack's bytes for each cell of a tile's level 0 -
# the United Kingdom and Ireland's pack has 0.3 for each of its tiles with
# land, its sea and moor plain; Luxembourg's, fields and woods all through,
# 0.5; the Alps', 0.7 - and a tile's seconds, one worker, read and made from
# the archives on AWS, for each 192 columns it has at level 0 - most of it
# reading WorldCover: the release's pack, 2,950 tiles with something on the
# archives, most of 192 columns, took 12 minutes eight at a time.
BYTES_PER_CELL = 0.4
SECONDS_PER_TILE = 2.0

TPD = gg.TILES_PER_DEG
TILE_DEG = gg.TILE_DEG
ROWS, COLS = 180 * TPD, 360 * TPD  # the world's tiles: from the north pole down, from 180 west east
KM_LAT, KM_LON = 110.574, 111.320  # a degree's kilometres, north and east at the equator

NE_URL = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/v5.1.2/geojson/"
ADMIN0, ADMIN1 = "ne_10m_admin_0_countries", "ne_10m_admin_1_states_provinces"
LISTS = {
    "glo30": "https://copernicus-dem-30m.s3.amazonaws.com/tileList.txt",
    "glo90": "https://copernicus-dem-90m.s3.amazonaws.com/tileList.txt",
    "worldcover": "https://esa-worldcover.s3.eu-central-1.amazonaws.com/v200/2021/esa_worldcover_grid.geojson",
}
GEOCODER = "https://geocoding-api.open-meteo.com/v1/search"
PROJECT = "https://github.com/craigdianesmith-glitch/tab5-adsb-flight-tracker-cpp"
# Where the moor is drawn: the British Isles, by Natural Earth's codes.
BRITISH_ISLES = ("GBR", "IRL", "IMN", "JEY", "GGY")
AGENT = "overhead-make-ground-pack/1"

# The 3D view's colours for the covers, as src/scene3d.cpp's COVER_RGB has
# them - none, trees, shrubs, grass, crops, built over, bare, snow and ice,
# water, marsh, mangroves, moss and lichen, suburb, town, moor, rocky - for
# the preview. Its sea for none, as the view draws the sea.
COVER_RGB = [(44, 66, 90), (40, 72, 42), (94, 90, 64), (80, 118, 58), (126, 128, 76), (124, 118, 108),
             (128, 124, 112), (236, 240, 244), (44, 66, 90), (70, 90, 70), (40, 84, 54), (102, 110, 88),
             (100, 106, 84), (112, 106, 100), (108, 100, 64), (118, 116, 100)]
COVER_NAMES = ["sea", "trees", "shrubs", "grass", "fields", "built over", "bare ground", "snow and ice", "water",
               "marsh", "mangroves", "moss and lichen", "suburbs", "town", "moor", "rocky ground"]
OFF_PACK = (24, 30, 40)  # the preview's tiles not in the pack


class Trouble(Exception):
    """Something the person running it can put right - said plainly, with
    what to do."""


class Stopped(Exception):
    """Stopped part way, the work so far kept."""


# --- Saying and asking ---------------------------------------------------

def say(text=""):
    print(text, flush=True)


def ask(prompt, default=""):
    try:
        answer = input(prompt)
    except EOFError:
        raise Trouble("No answer came (the input closed), so nothing was made.")
    answer = answer.strip()
    return answer if answer else default


def yes(prompt, default=True):
    answer = ask(prompt).lower()
    if not answer:
        return default
    return answer[0] in "yj"


def megabytes(n):
    if n <= 0:
        return "0 KB"
    if n < 1e6:
        return f"{max(1, round(n / 1e3))} KB"
    return f"{n / 1e6:.1f} MB" if n < 10e6 else f"{n / 1e6:.0f} MB"


def memory(n):
    """Memory, in the binary megabytes and kilobytes the limits are in."""
    if n < 1024:
        return "under 1 KB"
    return f"{n / 1024:.0f} KB" if n < 1 << 20 else f"{n / (1 << 20):.1f} MB"


def minutes(seconds):
    if seconds < 45:
        return "under a minute"
    if seconds < 90:
        return "about a minute"
    if seconds < 3600:
        return f"about {round(seconds / 60)} minutes"
    return f"about {seconds / 3600:.1f} hours"


def took(seconds):
    """How long something took: "12 seconds", "4 minutes"."""
    if seconds < 1.5:
        return "a second"
    if seconds < 90:
        return f"{round(seconds)} seconds"
    if seconds < 3600:
        return f"{round(seconds / 60)} minutes"
    return f"{seconds / 3600:.1f} hours"


def grouped(n):
    return f"{n:,}"


def listed(names):
    names = list(names)
    return names[0] if len(names) == 1 else ", ".join(names[:-1]) + " and " + names[-1]


# --- The cache, and downloads --------------------------------------------

def default_cache():
    if os.name == "nt":
        base = Path(os.environ.get("LOCALAPPDATA") or Path.home() / "AppData" / "Local")
    elif sys.platform == "darwin":
        base = Path.home() / "Library" / "Caches"
    else:
        base = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return base / "overhead-ground-pack"


def ssl_context():
    """The certificates rasterio brings with it - certifi's - where the
    system's own are missing, as on a Mac's python.org Python."""
    try:
        import certifi
        return ssl.create_default_context(cafile=certifi.where())
    except Exception:
        return ssl.create_default_context()


def fetch(url, path, what):
    """`url` downloaded to `path`, once - kept there, compressed - and its
    path; asked for compressed, as GitHub and S3 send it. Said as it starts,
    unless `what` is empty."""
    if path.exists():
        return path
    if what:
        say(f"Downloading {what} - once; it is kept for next time...")
    path.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(url, headers={"Accept-Encoding": "gzip", "User-Agent": AGENT})
    for attempt in range(4):
        try:
            with urllib.request.urlopen(request, timeout=60, context=ssl_context()) as r:
                data = r.read()
                if r.headers.get("Content-Encoding") != "gzip":
                    data = gzip.compress(data, 6)
            part = path.with_name(path.name + ".part")
            part.write_bytes(data)
            os.replace(part, path)
            return path
        except ssl.SSLError as e:
            raise Trouble(f"Couldn't download {what}: the connection's certificate couldn't be checked ({e}).\n"
                          "On a Mac, run \"Install Certificates.command\" in the Python folder under Applications, "
                          "then try again.")
        except (urllib.error.URLError, OSError) as e:
            if attempt == 3:
                raise Trouble(f"Couldn't download {what}: {getattr(e, 'reason', e)}.\n"
                              "Check the computer is online, then run it again.")
            time.sleep(3 * (attempt + 1))


def read_json(path):
    with gzip.open(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def lists_dir(cache):
    return Path(cache) / "lists"


def fetch_lists(cache):
    """The archives' lists of their tiles, in the cache."""
    d = lists_dir(cache)
    if not all((d / f"{name}.gz").exists() for name in LISTS):
        say("Downloading the maps' lists of their tiles - once; they are kept for next time...")
    for name, url in LISTS.items():
        fetch(url, d / f"{name}.gz", "")


def known_tiles(cache):
    """Every tile the archives have, as gen_ground.dataset() is asked for
    them - see gen_ground.KNOWN."""
    d = lists_dir(cache)
    known = set()
    pattern = re.compile(r"_([NS])(\d\d)_00_([EW])(\d\d\d)_00_DEM")
    for name, template in (("glo30", gg.DEM30), ("glo90", gg.DEM90)):
        with gzip.open(d / f"{name}.gz", "rt") as f:
            for line in f:
                m = pattern.search(line)
                if m:
                    known.add(template.format(ns=m[1], lat=int(m[2]), ew=m[3], lon=int(m[4])))
    for feature in read_json(d / "worldcover.gz")["features"]:
        m = re.fullmatch(r"([NS])(\d\d)([EW])(\d\d\d)", feature["properties"]["ll_tile"])
        if m:
            known.add(gg.WORLDCOVER.format(ns=m[1], lat=int(m[2]), ew=m[3], lon=int(m[4])))
    return known


def dem_degrees(cache):
    """The degree squares each of the GLO-30 and GLO-90 has, as (lat, lon)
    of their south-west corners."""
    d = lists_dir(cache)
    pattern = re.compile(r"_([NS])(\d\d)_00_([EW])(\d\d\d)_00_DEM")
    out = {}
    for name in ("glo30", "glo90"):
        got = set()
        with gzip.open(d / f"{name}.gz", "rt") as f:
            for line in f:
                m = pattern.search(line)
                if m:
                    got.add((int(m[2]) * (1 if m[1] == "N" else -1), int(m[4]) * (1 if m[3] == "E" else -1)))
        out[name] = got
    return out


def worldcover_squares(cache):
    """WorldCover's 3-degree squares, by their south-west corners."""
    got = set()
    for feature in read_json(lists_dir(cache) / "worldcover.gz")["features"]:
        m = re.fullmatch(r"([NS])(\d\d)([EW])(\d\d\d)", feature["properties"]["ll_tile"])
        if m:
            got.add((int(m[2]) * (1 if m[1] == "N" else -1), int(m[4]) * (1 if m[3] == "E" else -1)))
    return got


# --- Names ----------------------------------------------------------------

_LETTERS = str.maketrans({"ø": "o", "Ø": "o", "æ": "ae", "Æ": "ae", "œ": "oe", "Œ": "oe", "ß": "ss", "đ": "d",
                          "Đ": "d", "ł": "l", "Ł": "l", "þ": "th", "Þ": "th", "ð": "d", "ı": "i"})


def norm(text):
    """A name as it is matched: no accents or capitals, no punctuation, no
    "the" in front - "Côte d'Ivoire" as "cote divoire"."""
    text = unicodedata.normalize("NFKD", str(text).translate(_LETTERS))
    text = "".join(ch for ch in text if not unicodedata.combining(ch)).casefold().replace("&", " and ")
    text = re.sub(r"[.'’`]", "", text)
    words = [_WORDS.get(w, w) for w in re.sub(r"[^0-9a-z]+", " ", text).split()]
    if words[:1] == ["the"]:
        words = words[1:]
    return " ".join(words)


# Words Natural Earth shortens - "St. Lucia", "Canary Is." - shortened the
# same in what is typed.
_WORDS = {"saint": "st", "sainte": "ste", "islands": "is", "island": "i", "isles": "is", "mount": "mt"}


# Names people use that Natural Earth hasn't: to its countries' codes.
ALIASES = {"uk": "GBR", "great britain": "GBR", "britain": "GBR", "usa": "USA", "us": "USA", "america": "USA",
           "united states": "USA", "holland": "NLD", "turkiye": "TUR", "ivory coast": "CIV", "burma": "MMR",
           "swaziland": "SWZ", "macedonia": "MKD", "east timor": "TLS", "south korea": "KOR",
           "north korea": "PRK", "vatican": "VAT", "czech republic": "CZE", "the gambia": "GMB",
           "cape verde": "CPV", "republic of ireland": "IRL", "eire": "IRL"}
COUNTRY_FIELDS = (("NAME_EN", 0), ("ADMIN", 0), ("NAME", 0), ("NAME_LONG", 0), ("GEOUNIT", 1), ("SUBUNIT", 1),
                  ("BRK_NAME", 1), ("FORMAL_EN", 1), ("NAME_SORT", 1), ("NAME_ALT", 1), ("NAME_CIAWF", 1),
                  ("ISO_A2_EH", 0), ("ISO_A3_EH", 0), ("ADM0_A3", 0))
REGION_FIELDS = (("name", 0), ("name_en", 0), ("iso_3166_2", 0), ("woe_name", 1), ("gn_name", 1),
                 ("name_alt", 1))


class Choice:
    """An area asked for: a country, a region - one of a country's states,
    provinces or regions, or several together - a circle or a box."""

    def __init__(self, kind, name, what="", a3="", features=(), circle=None, box=None):
        self.kind = kind  # "country", "region", "circle" or "box"
        self.name = name  # as said back: "Texas"
        self.what = what  # and what it is: "a state of the United States of America"
        self.a3 = a3      # its country's code, Natural Earth's
        self.features = list(features)
        self.circle = circle  # (lat, lon, km)
        self.box = box        # (south, north, west, east)
        self.others = []      # a town's: the other places with its name

    def label(self):
        return f"{self.name}, {self.what}" if self.what else self.name


class Ambiguous(Trouble):
    def __init__(self, text, options):
        super().__init__(text)
        self.options = options  # Choices

    def full(self):
        lines = [str(self)] + [f"  {c.label()}" for c in self.options]
        example = next((c for c in self.options if c.kind == "region"), None)
        if example is not None:
            country = example.what.split(" of ", 1)[-1]
            lines.append(f'Say which, with its country: "{example.name}, {country}".')
        else:
            lines.append("Say which by its code - see --list.")
        return "\n".join(lines)


class Places:
    """Natural Earth's countries, and their states, provinces and regions -
    downloaded once, kept in the cache, read when first wanted."""

    def __init__(self, cache):
        self.cache = Path(cache)
        self._countries = self._regions = None
        self._country_names = self._region_names = None

    def _read(self, layer, what):
        path = fetch(NE_URL + layer + ".geojson", self.cache / "naturalearth" / f"{layer}.geojson.gz", what)
        try:
            return read_json(path)["features"]
        except (OSError, ValueError, EOFError):
            path.unlink(missing_ok=True)
            raise Trouble(f"The kept copy of {what} was damaged, so it has been deleted - run it again to download "
                          "it afresh.")

    def countries(self):
        if self._countries is None:
            self._countries = self._read(ADMIN0, "the list of countries (5 MB)")
            names = {}
            for i, f in enumerate(self._countries):
                p = f["properties"]
                for field, rank in COUNTRY_FIELDS:
                    value = p.get(field)
                    if value and value != "-99":
                        # A code is its own country's first: "AU" Australia,
                        # not one of its islands with the same code.
                        if field.startswith("ISO") and p["ADM0_A3"] != p.get("ISO_A3_EH"):
                            rank = 2
                        names.setdefault(norm(value), set()).add((rank, i))
            for alias, a3 in ALIASES.items():
                for i, f in enumerate(self._countries):
                    if f["properties"]["ADM0_A3"] == a3:
                        names.setdefault(alias, set()).add((0, i))
            self._country_names = names
        return self._countries

    def regions(self):
        if self._regions is None:
            self._regions = self._read(ADMIN1, "the list of states, provinces and regions (14 MB)")
            countries = {f["properties"]["ADM0_A3"]: f["properties"]["ADMIN"] for f in self.countries()}
            names = {}
            for i, f in enumerate(self._regions):
                p = f["properties"]
                for field, rank in REGION_FIELDS:
                    for value in str(p.get(field) or "").split("|"):
                        if value and value != "-99":
                            names.setdefault(norm(value), set()).add(("one", rank, i))
                # Together: a country's regions - Lombardia, Bretagne - and its
                # parts - Scotland, Wales, Flanders.
                for field in ("region", "geonunit"):
                    value = p.get(field)
                    if value and norm(value) != norm(countries.get(p["adm0_a3"], "")) and norm(value) != norm(
                            p.get("admin") or ""):
                        names.setdefault(norm(value), set()).add(("group", 2, (field, value, p["adm0_a3"])))
            self._region_names = names
        return self._regions

    def country_choice(self, i):
        f = self.countries()[i]
        p = f["properties"]
        name = p.get("NAME_EN") or p["NAME"]
        return Choice("country", name, "", p["ADM0_A3"], [f])

    def countries_named(self, key):
        """The countries `key` - a name, as norm() has it - names, the
        likeliest first."""
        self.countries()
        hits = self._country_names.get(key, set())
        if not hits:
            return []
        best = min(rank for rank, _ in hits)
        return sorted({i for rank, i in hits if rank == best})

    def _region_choice(self, entry):
        regions = self.regions()
        if entry[0] == "one":
            f = regions[entry[2]]
            p = f["properties"]
            kind = (p.get("type_en") or "region").lower()
            return Choice("region", region_name(p), f"{article(kind)} {kind} of {p['admin']}", p["adm0_a3"], [f])
        field, value, a3 = entry[2]
        members = [f for f in regions if f["properties"].get(field) == value and f["properties"]["adm0_a3"] == a3]
        admin = members[0]["properties"]["admin"] if members else a3
        what = f"a region of {admin}" if field == "region" else f"part of {admin}"
        return Choice("region", value, what, a3, members)

    def resolve(self, text):
        """The area `text` names - a country, or a region of one, "Limburg,
        Belgium" to say which - or Trouble saying why not."""
        text = text.strip()
        whole = self.countries_named(norm(text))
        if len(whole) == 1:
            return self.country_choice(whole[0])
        place, country = text, None
        m = re.fullmatch(r"(.+?)\s*\((.+)\)\s*", text) or re.fullmatch(r"(.+),\s*([^,]+)", text)
        if m:
            place, country = m[1], m[2]
        key = norm(place)
        if not key:
            raise Trouble("That was empty - type a country, a state or region, or a place.")
        if country is None:
            hits = self.countries_named(key)
            if len(hits) == 1:
                return self.country_choice(hits[0])
            if len(hits) > 1:
                options = [self.country_choice(i) for i in hits]
                raise Ambiguous(f'"{text}" could be more than one country.', options)
        self.regions()
        entries = self._region_names.get(key, set())
        if country is not None:
            wanted = {self.countries()[i]["properties"]["ADM0_A3"] for i in self.countries_named(norm(country))}
            if wanted:
                entries = {e for e in entries if (e[2][2] if e[0] == "group" else
                                                  self._regions[e[2]]["properties"]["adm0_a3"]) in wanted}
            else:  # not a country after the comma: the whole of it a name, "Korea, Republic of"
                key, place = norm(text), text
                entries = self._region_names.get(key, set())
        if entries:
            best = min(e[1] for e in entries)
            choices, seen = [], set()
            for e in sorted((e for e in entries if e[1] == best), key=str):
                c = self._region_choice(e)
                ids = frozenset(id(f) for f in c.features)
                if ids and ids not in seen:
                    seen.add(ids)
                    choices.append(c)
            if len(choices) > 1 and len({c.a3 for c in choices}) == 1:
                # All in one country: a region its provinces all name -
                # Lombardy, Brittany - as one.
                features = list({id(f): f for c in choices for f in c.features}.values())
                admin = features[0]["properties"]["admin"]
                return Choice("region", typed_name(features, key, place), f"a region of {admin}", choices[0].a3,
                              features)
            if len(choices) == 1:
                return choices[0]
            if choices:
                raise Ambiguous(f'"{place}" is the name of more than one place.', choices)
        raise Trouble(self.not_found(text, key))

    def not_found(self, text, key):
        names = {}
        for k, hits in self._country_names.items():
            names.setdefault(k, self.countries()[min(hits)[1]]["properties"].get("NAME_EN"))
        if self._region_names is not None:
            for k, hits in self._region_names.items():
                e = min(hits, key=str)
                names.setdefault(k, e[2][1] if e[0] == "group" else
                                 self._regions[e[2]]["properties"]["name"])
        close = difflib.get_close_matches(key, list(names), n=4, cutoff=0.75)
        hint = ""
        if close:
            hint = " Did you mean " + listed([f'"{names[c]}"' for c in close]).replace(" and ", " or ") + "?"
        return (f'"{text}" isn\'t a country, state or region I know.{hint}\n'
                f"For a town, use --around \"{text}\" --radius 150; to see the names I know, --list.")

    def name_parts(self, a3, far):
        """Names for a country's far-off parts, `far`: their regions' - or
        "islands of" one where most of it is kept, as Alaska's Aleutians
        past the 180th meridian."""
        mine = [(region_name(f["properties"]), [poly_box(q) for q in polygons(f["geometry"])])
                for f in self.regions() if f["properties"]["adm0_a3"] == a3]
        sizes = {}
        for f in self.regions():
            q = f["properties"]
            if q["adm0_a3"] == a3:
                area = sum(poly_km2(p) for p in polygons(f["geometry"]))
                sizes[region_name(q)] = sizes.get(region_name(q), 0.0) + area

        def region_of(poly):
            w, s, e, n = poly_box(poly)
            best, most = None, 0.0
            for name, boxes in mine:
                for fw, fs, fe, fn in boxes:
                    overlap = max(0.0, min(e, fe) - max(w, fw)) * max(0.0, min(n, fn) - max(s, fs))
                    if overlap > most:
                        best, most = name, overlap
            if best is None:
                lat, lon = (s + n) / 2, (w + e) / 2
                return (f"the part near {abs(lat):.1f}{'N' if lat >= 0 else 'S'} "
                        f"{abs(lon):.1f}{'E' if lon >= 0 else 'W'}")
            return best

        gone = {}
        for p in far:
            name = region_of(p)
            gone[name] = gone.get(name, 0.0) + poly_km2(p)
        return sorted(name if gone[name] >= 0.5 * sizes.get(name, 0.0) else f"islands of {name}" for name in gone)


def region_name(p):
    """A state's or province's name, in English where Natural Earth has one
    that is a name rather than a word for its kind."""
    en = p.get("name_en")
    if en and norm(en) not in norm(p.get("type_en") or ""):
        return en
    return p["name"]


def typed_name(features, key, typed):
    """The name as Natural Earth spells it that `key` matched, among the
    features' names - else as it was typed."""
    for f in features:
        p = f["properties"]
        for field, _ in REGION_FIELDS:
            for value in str(p.get(field) or "").split("|"):
                if value and norm(value) == key:
                    return value
    return typed.strip()


def article(word):
    w = word.lower()
    return "a" if w[:1] not in "aeiou" or w.startswith(("uni", "use", "eu", "one")) else "an"


# --- Shapes ---------------------------------------------------------------

def polygons(geometry):
    """A GeoJSON geometry's polygons, each a list of rings."""
    if geometry is None:
        return []
    if geometry["type"] == "Polygon":
        return [geometry["coordinates"]]
    if geometry["type"] == "MultiPolygon":
        return list(geometry["coordinates"])
    return []


def poly_box(poly):
    ring = np.asarray(poly[0], dtype=float)
    return float(ring[:, 0].min()), float(ring[:, 1].min()), float(ring[:, 0].max()), float(ring[:, 1].max())


def poly_km2(poly):
    ring = np.asarray(poly[0], dtype=float)
    lat = math.radians(float(ring[:, 1].mean()))
    x, y = ring[:, 0] * KM_LON * math.cos(lat), ring[:, 1] * KM_LAT
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))) / 2


def home_parts(polys):
    """The polygons that go together - the biggest, and every one within
    LINK_KM of one already in, by the gap between their boxes - and those
    left out, too far off: a country's overseas parts. Not across the 180th
    meridian: the far side is as far off as the earth is round."""
    if len(polys) <= 1:
        return list(polys), []
    boxes = np.array([poly_box(p) for p in polys])
    areas = np.array([poly_km2(p) for p in polys])
    kept = np.zeros(len(polys), bool)
    start = int(areas.argmax())
    kept[start] = True
    frontier = [start]
    while frontier:
        i = frontier.pop()
        w, s, e, n = boxes[i]
        dlat = np.maximum(0, np.maximum(boxes[:, 1] - n, s - boxes[:, 3]))
        dlon = np.maximum(0, np.maximum(boxes[:, 0] - e, w - boxes[:, 2]))
        lat = np.radians((s + n) / 2 + (boxes[:, 1] + boxes[:, 3]) / 2) / 2
        gap = np.hypot(dlat * KM_LAT, dlon * KM_LON * np.cos(lat))
        near = np.where(~kept & (gap <= LINK_KM))[0]
        kept[near] = True
        frontier.extend(int(j) for j in near)
    return [p for p, k in zip(polys, kept) if k], [p for p, k in zip(polys, kept) if not k]


# --- Tiles ----------------------------------------------------------------
#
# The world's tiles as a picture's pixels: rows from the north pole down,
# columns from 180 west east - row i the tile gen_ground calls ty = ROWS - 1
# - i, from the south pole.

def cols0_of(ty):
    """A tile's columns at level 0, by its row from the south pole - 288,
    fewer further from the equator, nought past 75 degrees."""
    k = gg.k_for(ty / TPD - 90 + TILE_DEG / 2)
    return 0 if k is None else int(gg.ROWS0 / k)


def coarse_bytes(cols0):
    """A tile's coarse block, inflated, as ground_pack.cpp's coarseBytes()."""
    return 3 * (gg.ROWS0 // 16) * (cols0 // 16)


def tile_width_km(ty):
    return TILE_DEG * KM_LON * math.cos(math.radians(ty / TPD - 90 + TILE_DEG / 2))


def burn(polys, mask):
    """Every tile any of `polys` touches, set in `mask`."""
    if not polys:
        return
    boxes = np.array([poly_box(p) for p in polys])
    w, s, e, n = boxes[:, 0].min(), boxes[:, 1].min(), boxes[:, 2].max(), boxes[:, 3].max()
    c0 = max(0, int(math.floor((w + 180) * TPD)) - 1)
    c1 = min(COLS, int(math.ceil((e + 180) * TPD)) + 1)
    r0 = max(0, int(math.floor((90 - n) * TPD)) - 1)
    r1 = min(ROWS, int(math.ceil((90 - s) * TPD)) + 1)
    transform = from_origin(-180 + c0 / TPD, 90 - r0 / TPD, TILE_DEG, TILE_DEG)
    shapes = [({"type": "Polygon", "coordinates": p}, 1) for p in polys]
    got = rasterio.features.rasterize(shapes, out_shape=(r1 - r0, c1 - c0), transform=transform, all_touched=True,
                                      fill=0, dtype="uint8")
    mask[r0:r1, c0:c1] |= got.astype(bool)


def widen(mask, km):
    """`mask`, and every tile within some `km` of one in it."""
    if km <= 0:
        return mask.copy()
    out = mask.copy()
    for d in range(1, int(math.ceil(km / (TILE_DEG * KM_LAT))) + 1):
        out[d:] |= mask[:-d]
        out[:-d] |= mask[d:]
    lat = 90 - (np.arange(ROWS) + 0.5) / TPD
    across = np.minimum(60, np.ceil(km / np.maximum(TILE_DEG * KM_LON * np.cos(np.radians(lat)), 1e-3))).astype(int)
    busy = out.any(axis=1)
    result = out.copy()
    for reach in sorted(set(across[busy])):
        rows = np.where(busy & (across == reach))[0]
        part = out[rows]
        acc = part.copy()
        for d in range(1, reach + 1):
            acc[:, d:] |= part[:, :-d]
            acc[:, :-d] |= part[:, d:]
        result[rows] = acc
    return result


def circle(lat, lon, km, mask):
    """Every tile with a corner or an edge within `km` of (lat, lon), set in
    `mask`."""
    dlat = km / KM_LAT + TILE_DEG
    r0 = max(0, int(math.floor((90 - (lat + dlat)) * TPD)))
    r1 = min(ROWS, int(math.ceil((90 - (lat - dlat)) * TPD)))
    widest = max(0.02, math.cos(math.radians(min(89.0, abs(lat) + dlat))))
    dlon = min(180.0, km / (KM_LON * widest) + TILE_DEG)
    c0 = max(0, int(math.floor((lon - dlon + 180) * TPD)))
    c1 = min(COLS, int(math.ceil((lon + dlon + 180) * TPD)))
    north = 90 - np.arange(r0, r1) / TPD
    west = np.arange(c0, c1) / TPD - 180
    near_lat = np.clip(lat, north - TILE_DEG, north)[:, None]
    near_lon = np.clip(lon, west, west + TILE_DEG)[None, :]
    p1, p2 = math.radians(lat), np.radians(near_lat)
    a = (np.sin((p2 - p1) / 2) ** 2 +
         math.cos(p1) * np.cos(p2) * np.sin(np.radians(near_lon - lon) / 2) ** 2)
    mask[r0:r1, c0:c1] |= 2 * 6371.0 * np.arcsin(np.sqrt(np.minimum(1, a))) <= km


def box_grid(south, north, west, east):
    """A box's tiles as gen_ground.py takes them: from the tile with its
    south-west corner on to the one with its north-east - (south, west,
    rows, cols) in tiles from the equator and the meridian."""
    s0, n0 = int(math.floor(south * TPD)), int(math.ceil(north * TPD))
    w0, e0 = int(math.floor(west * TPD)), int(math.ceil(east * TPD))
    return s0, w0, n0 - s0, e0 - w0


def polar_rows():
    """The rows past 75 degrees, which have no tiles."""
    return np.array([cols0_of(ROWS - 1 - i) == 0 for i in range(ROWS)])


# --- The plan, and what it will take ---------------------------------------

class Plan:
    """The tiles to make for the areas chosen, and what the pack will take."""

    def __init__(self):
        self.choices = []
        self.tiles = []        # (ty, tx), by latitude then longitude
        self.moor = set()      # the tiles with the moor drawn
        self.grid = None       # a box's own grid, as gen_ground.py's; else the tiles'
        self.keep_sea = False  # tiles of nothing but sea kept, as gen_ground.py keeps them
        self.left_out = []     # (area, [parts]) too far off
        self.outlines = []     # polygons chosen, for the preview
        self.circles = []      # (lat, lon, km)
        self.polar = False     # some of it past 75 degrees, which the pack can't have
        self.land = []         # the tiles likely to be in the pack: land in them, or sea kept
        self.solid = []        # those of them with land, for the pack's size
        self.work = []         # the tiles with something on the archives, not made before
        self.stood_in = 0      # tiles where the GLO-90 will stand in for the GLO-30
        self.cached = 0        # tiles made before, and kept
        self.margin_km = 0     # taken round the countries and regions


def plan_for(choices, places, cache, margin_km, moor_mode):
    plan = Plan()
    plan.choices = choices
    plan.margin_km = margin_km
    mask = np.zeros((ROWS, COLS), bool)
    for c in choices:
        if c.kind in ("country", "region"):
            polys = [p for f in c.features for p in polygons(f["geometry"])]
            kept, far = home_parts(polys)
            if far:
                big = [p for p in far if poly_km2(p) >= 5]
                plan.left_out.append((c.name, places.name_parts(c.a3, big) if big else []))
            area = np.zeros((ROWS, COLS), bool)
            burn(kept, area)
            mask |= widen(area, margin_km)
            plan.outlines.extend(kept)
        elif c.kind == "circle":
            circle(*c.circle, mask)
            plan.circles.append(c.circle)
        elif c.kind == "box":
            s0, w0, rows, cols = box_grid(*c.box)
            r0, c0 = 90 * TPD - (s0 + rows), w0 + 180 * TPD
            mask[max(0, r0):max(0, r0 + rows), max(0, c0):max(0, c0 + cols)] = True
    polar = polar_rows()
    plan.polar = bool(mask[polar].any())
    mask[polar] = False
    rows, cols = np.nonzero(mask)
    plan.tiles = sorted((ROWS - 1 - int(r), int(c)) for r, c in zip(rows, cols))
    # A box alone: its own grid, and every tile with something on the
    # archives kept, sea or not - gen_ground.py's pack, byte for byte.
    if len(choices) == 1 and choices[0].kind == "box":
        plan.grid = box_grid(*choices[0].box)
        plan.keep_sea = True
    if moor_mode == "on":
        plan.moor = set(plan.tiles)
    elif moor_mode == "auto":
        isles = np.zeros((ROWS, COLS), bool)
        burn([p for f in places.countries() if f["properties"]["ADM0_A3"] in BRITISH_ISLES
              for p in polygons(f["geometry"])], isles)
        isles = widen(isles, max(margin_km, MARGIN_KM))
        plan.moor = {t for t in plan.tiles if isles[ROWS - 1 - t[0], t[1]]}
    # What it will take: the tiles likely to have land - those Natural
    # Earth's countries touch, or near them for a box, which keeps the sea
    # by the coast too - and those with anything on the archives at all.
    land = land_mask(places, cache)
    plan.solid = [t for t in plan.tiles if land[ROWS - 1 - t[0], t[1]]]
    if plan.keep_sea:
        land = widen(land, TILE_DEG * KM_LAT)
    plan.land = [t for t in plan.tiles if land[ROWS - 1 - t[0], t[1]]]
    squares = worldcover_squares(cache)
    degrees = dem_degrees(cache)
    for ty, tx in plan.tiles:
        lat, lon = ty / TPD - 90, tx / TPD - 180
        square = (int(math.floor(lat / 3)) * 3, int(math.floor(lon / 3)) * 3)
        degree = (int(math.floor(lat + 1e-9)), int(math.floor(lon + 1e-9)))
        made = tile_file(cache, ty, tx, (ty, tx) in plan.moor).exists()
        plan.cached += made
        if not made and (square in squares or degree in degrees["glo30"] or degree in degrees["glo90"]):
            plan.work.append((ty, tx))
        plan.stood_in += degree in degrees["glo90"] and degree not in degrees["glo30"]
    return plan


def land_mask(places, cache):
    """Every tile one of Natural Earth's countries touches - kept in the
    cache once worked out."""
    path = Path(cache) / "naturalearth" / "land-tiles.npy"
    try:
        got = np.load(path)
        if got.shape == (ROWS, COLS):
            return got
    except (OSError, ValueError):
        pass
    mask = np.zeros((ROWS, COLS), bool)
    shapes = [(f["geometry"], 1) for f in places.countries() if f["geometry"]]
    got = rasterio.features.rasterize(shapes, out_shape=(ROWS, COLS), all_touched=True, fill=0, dtype="uint8",
                                      transform=from_origin(-180, 90, TILE_DEG, TILE_DEG))
    mask |= got.astype(bool)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.save(path, mask)
    except OSError:
        pass
    return mask


class Estimate:
    """What a pack of `tiles` - (ty, tx) each - takes: on the Tab5, on the
    card, and to make; `solid` the tiles of them with land, for its size."""

    def __init__(self, tiles, grid=None, jobs=8, work=(), solid=None):
        self.tiles = len(tiles)
        self.coarse = sum(coarse_bytes(cols0_of(ty)) for ty, _ in tiles)
        if grid is not None:
            self.rows, self.cols = grid[2], grid[3]
        elif tiles:
            tys, txs = [t[0] for t in tiles], [t[1] for t in tiles]
            self.rows, self.cols = max(tys) - min(tys) + 1, max(txs) - min(txs) + 1
        else:
            self.rows = self.cols = 0
        self.cells = self.rows * self.cols
        self.index = GRID_BYTES * self.cells + ENTRY_BYTES * self.tiles
        self.size = 32 + 28 * self.tiles + int(sum(BYTES_PER_CELL * gg.ROWS0 * cols0_of(ty)
                                                   for ty, _ in (tiles if solid is None else solid)))
        self.todo = len(work)
        busy = sum(SECONDS_PER_TILE * cols0_of(ty) / 192 for ty, _ in work)
        self.seconds = 20 + busy / max(1, min(jobs, self.todo))

    def too_big(self):
        """What is over the Tab5's limits, said plainly - empty if nothing."""
        over = []
        if self.tiles > MAX_TILES:
            over.append(f"{grouped(self.tiles)} tiles, and the Tab5 takes at most {grouped(MAX_TILES)}")
        if self.coarse > MAX_COARSE_BYTES:
            over.append(f"{memory(self.coarse)} of the Tab5's memory for the far country, and it has "
                        f"{memory(MAX_COARSE_BYTES)} for it")
        if self.cells > MAX_GRID:
            over.append(f"a rectangle of {grouped(self.cells)} tiles round it all, and the Tab5 keeps an index of "
                        f"at most {grouped(MAX_GRID)}")
        return over


def fitting_radius(lat):
    """The widest circle whose pack the Tab5 can hold, round a place at
    `lat`, in kilometres - rounded down, with room to spare."""
    ty = int(math.floor((lat + 90) * TPD))
    cols0 = cols0_of(ty) or gg.ROWS0 // 3
    tiles = min(MAX_TILES, MAX_COARSE_BYTES / coarse_bytes(cols0)) * 0.85
    area = tiles * (TILE_DEG * KM_LAT) * max(1.0, tile_width_km(ty))
    return max(50, int(math.sqrt(area / math.pi) / 50) * 50)


def middle_of(plan):
    """A place near the middle of the plan's tiles, to suggest a circle
    round."""
    tys = sorted(t[0] for t in plan.land or plan.tiles)
    txs = sorted(t[1] for t in plan.land or plan.tiles)
    ty, tx = tys[len(tys) // 2], txs[len(txs) // 2]
    return ty / TPD - 90 + TILE_DEG / 2, tx / TPD - 180 + TILE_DEG / 2


def advice(plan, est, program):
    """Why it won't make this one, and what to try instead."""
    lat, lon = middle_of(plan)
    radius = fitting_radius(lat)
    suggest = min(300, radius)
    names = listed([c.name for c in plan.choices])
    lines = [f"That is more than the Tab5 can hold. {names} would come to:"]
    lines += [f"  - {reason}" for reason in est.too_big()]
    lines += ["", "Make a smaller pack instead:",
             f"  - round your home, with a radius of up to some {radius} km there - say",
             f"      {program} --around {lat:.2f},{lon:.2f} --radius {suggest}",
             "    with your own latitude and longitude, or a town's name, after --around;"]
    if est.cells > MAX_GRID and len(plan.choices) > 1:
        lines.insert(1 + len(est.too_big()), "Its areas are too far apart for one pack: the Tab5 keeps an index of "
                                             "every tile in the rectangle round them all.")
    countries = [c for c in plan.choices if c.kind == "country"]
    if countries:
        lines.append(f"  - or some of its states or regions - {program} --list \"{countries[0].name}\" shows them;")
    if len(plan.choices) > 1:
        lines.append("  - or fewer of the areas at once.")
    else:
        lines[-1] = lines[-1].rstrip(";") + "."
    return "\n".join(lines)


def describe(plan, est, jobs, out):
    """What the plan will make, for the person to say yes to."""
    say()
    say("What it would make:")
    for c in plan.choices:
        say(f"  Area:         {c.label()}")
    for name, parts in plan.left_out:
        if parts:
            say(f"  Left out:     {listed(parts)} - far from the rest of {name}. Name one on its own to make "
                "a pack of it.")
        else:
            say(f"  Left out:     some small islands far from the rest of {name}.")
    if plan.polar:
        say("  Left out:     the land past 75 degrees north or south, which the pack has no tiles for.")
    if any(c.kind in ("country", "region") for c in plan.choices) and plan.margin_km > 0:
        say(f"  Margin:       {plan.margin_km:g} km round it, for the hills just over its borders and coasts")
    sea = "" if plan.keep_sea else " - the open sea round it is left out"
    say(f"  Tiles:        about {grouped(est.tiles)} with land (each 0.2 degrees square){sea}")
    say(f"  Pack size:    about {megabytes(est.size)}")
    say(f"  Tab5 memory:  about {memory(est.coarse)} of the {memory(MAX_COARSE_BYTES)} it allows for the far "
        f"country, and {memory(est.index)} for its index")
    if plan.cached:
        say(f"  Already made: {grouped(plan.cached)} of its tiles, kept from before")
    if est.todo or not plan.cached:
        say(f"  Time:         {minutes(est.seconds)} with a fast connection ({jobs} tiles at a time)")
    else:
        say("  Time:         a few seconds - its tiles are all made already")
    if plan.stood_in:
        say("  Hills:        from the coarser GLO-90 (90 m) where the GLO-30 (30 m) isn't public - "
            f"{grouped(plan.stood_in)} of its tiles")
    moor = len(plan.moor.intersection(plan.land))
    if moor == len(plan.land) and moor:
        say("  Moor:         drawn - the British Isles' rough hill grazing, on grass high up or steep")
    elif moor:
        say(f"  Moor:         drawn on its {grouped(moor)} tiles in or near the British Isles; elsewhere grass is "
            "grass")
    say(f"  Saved as:     {out}")


# --- Making the tiles ------------------------------------------------------

TILE_HEAD = struct.Struct("<4sBBHHHIIII")  # "OVGT", format, flags, 0, ty, tx, crc32, the blocks' lengths
TILE_FORMAT = 1
HAS_BLOCKS, STOOD_IN, SEA = 1, 2, 4


def tile_file(cache, ty, tx, moor):
    """Where a tile made is kept: by gen_ground's TILE_VERSION, and with the
    moor or without."""
    kind = f"v{gg.TILE_VERSION}-{'moor' if moor else 'grass'}"
    return Path(cache) / "tiles" / kind / f"{ty:03d}" / f"{tx:04d}.tile"


def replace(part, path):
    """`part` renamed to `path`, over any there - tried again a moment later
    if the file is held, as a virus checker on Windows can hold one just
    written."""
    for attempt in range(5):
        try:
            os.replace(part, path)
            return
        except PermissionError:
            if attempt == 4:
                raise
            time.sleep(0.5)


def save_tile(path, ty, tx, blocks, flags):
    data = b"".join(blocks) if blocks else b""
    sizes = [len(b) for b in blocks] if blocks else [0, 0, 0]
    head = TILE_HEAD.pack(b"OVGT", TILE_FORMAT, flags, 0, ty, tx, zlib.crc32(data), *sizes)
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    with open(part, "wb") as f:
        f.write(head + data)
    replace(part, path)


def load_tile(path):
    """A tile kept: its flags and blocks - None if it isn't there, or isn't
    whole."""
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    if len(raw) < TILE_HEAD.size:
        return None
    magic, form, flags, _, ty, tx, crc, a, b, c = TILE_HEAD.unpack_from(raw)
    data = raw[TILE_HEAD.size:]
    if magic != b"OVGT" or form != TILE_FORMAT or len(data) != a + b + c or zlib.crc32(data) != crc:
        return None
    return flags, ((data[:a], data[a:a + b], data[a + b:]) if flags & HAS_BLOCKS else None)


def only_sea(blocks, cols0):
    """Whether a tile is nothing but water - or nothing - and no height
    above the sea: the open sea, which the 3D view draws the same without
    it."""
    detail = np.frombuffer(zlib.decompress(blocks[2], -15), np.uint8) >> 4
    if not np.isin(detail, (gg.NONE, gg.WATER)).all():
        return False
    r1, c1 = gg.ROWS0 // 2, cols0 // 2
    cells = r1 * c1 + (r1 // 2) * (c1 // 2) + (r1 // 4) * (c1 // 4)
    fine = np.frombuffer(zlib.decompress(blocks[1], -15)[cells:], "<i2")
    coarse = np.frombuffer(zlib.decompress(blocks[0], -15)[(gg.ROWS0 // 16) * (cols0 // 16):], "<i2")
    return bool((fine <= 0).all() and (coarse <= 0).all())


def start_worker(cache):
    """Each worker: Ctrl+C left to the main process, GDAL's settings, and
    the archives' lists."""
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except (ValueError, OSError):
        pass
    gg.start_worker()
    gg.KNOWN = known_tiles(cache)


def make_tiles(chunk, cache):
    """Makes the tiles of `chunk` - (ty, tx, moor) each - and keeps each in
    the cache as it is done: (ty, tx, flags, bytes, error) for each."""
    out = []
    for ty, tx, moor in chunk:
        try:
            blocks, stood_in = gg.make_tile(ty, tx, moor=moor, dem_fallback=True)
            flags = (HAS_BLOCKS if blocks else 0) | (STOOD_IN if stood_in else 0)
            if blocks and only_sea(blocks, cols0_of(ty)):
                flags |= SEA
            save_tile(tile_file(cache, ty, tx, moor), ty, tx, blocks, flags)
            out.append((ty, tx, flags, sum(map(len, blocks)) if blocks else 0, None))
        except Exception as e:  # the network, mostly: the tile is made again next time
            message = " ".join(str(e).split())[:300] or type(e).__name__
            out.append((ty, tx, 0, 0, message))
    return out


def stop_workers(pool):
    for p in list(getattr(pool, "_processes", {}).values()):
        try:
            p.terminate()
        except Exception:
            pass
    try:
        pool.shutdown(wait=False, cancel_futures=True)
    except TypeError:
        pool.shutdown(wait=False)


def run(todo, plan, cache, jobs):
    """Makes the tiles of `todo` not yet made, `jobs` at a time, saying how
    it goes - those that failed, (ty, tx, error) each."""
    if not todo:
        return []
    chunks, run_ = [], []
    for ty, tx in todo:  # two neighbours together: their reads overlap
        if run_ and (run_[-1][0] != ty or run_[-1][1] != tx - 1 or len(run_) == 2):
            chunks.append(run_)
            run_ = []
        run_.append((ty, tx, (ty, tx) in plan.moor))
    if run_:
        chunks.append(run_)
    total = len(todo)
    done = made = size = 0
    failed = []
    started = time.monotonic()
    said = 0.0
    tty = sys.stdout.isatty()
    pool = concurrent.futures.ProcessPoolExecutor(max_workers=max(1, min(jobs, 61, len(chunks))),
                                                  initializer=start_worker, initargs=(str(cache),))
    try:
        futures = [pool.submit(make_tiles, chunk, str(cache)) for chunk in chunks]
        for future in concurrent.futures.as_completed(futures):
            for ty, tx, flags, n, error in future.result():
                done += 1
                if error:
                    failed.append((ty, tx, error))
                elif flags & HAS_BLOCKS and (plan.keep_sea or not flags & SEA):
                    made += 1
                    size += n
            now = time.monotonic()
            if now - said > (1 if tty else 15) or done == total:
                said = now
                rate = done / max(1e-6, now - started)
                left = (total - done) / max(rate, 1e-6)
                line = (f"  {grouped(done)} of {grouped(total)} tiles ({100 * done // total}%), {grouped(made)} with "
                        f"land, {megabytes(size)} so far" +
                        (f" - {minutes(left)} to go" if done < total and done >= 8 else ""))
                if tty:
                    print("\r" + line.ljust(100), end="", flush=True)
                else:
                    say(line)
        if tty:
            print(flush=True)
    except KeyboardInterrupt:
        if tty:
            print(flush=True)
        stop_workers(pool)
        raise Stopped()
    except concurrent.futures.process.BrokenProcessPool:
        stop_workers(pool)
        raise Trouble("A worker stopped unexpectedly - most often for want of memory. The tiles made so far are "
                      "kept: run it again with --jobs 4 to carry on with fewer at once.")
    pool.shutdown(wait=True)
    return failed


def build(plan, cache, jobs):
    """Every tile of the plan made and kept - Trouble if some can't be,
    even on a second try."""
    todo = [t for t in plan.tiles if load_tile(tile_file(cache, t[0], t[1], t in plan.moor)) is None]
    if len(todo) < len(plan.tiles):
        say(f"  {grouped(len(plan.tiles) - len(todo))} of the {grouped(len(plan.tiles))} tiles were made before, "
            "and are kept - carrying on with the rest.")
    failed = run(todo, plan, cache, jobs)
    if failed:
        say(f"  {len(failed)} tiles couldn't be read - trying them again in a moment...")
        time.sleep(10)
        failed = run([(ty, tx) for ty, tx, _ in failed], plan, cache, max(1, jobs // 2))
    if failed:
        ty, tx, error = failed[0]
        raise Trouble(f"{len(failed)} tiles couldn't be read from the maps' archives - the connection may have "
                      f"dropped, or the archives be busy. Everything else is kept: run the same command again to "
                      f"finish.\n(The first, at {ty / TPD - 90:.1f},{tx / TPD - 180:.1f}: {error})")


# --- The pack, and what goes with it -------------------------------------

class Made:
    def __init__(self):
        self.tiles = []     # (ty, tx, coarse, mid, detail)
        self.sea = 0        # tiles of open sea left out
        self.stood_in = 0   # tiles shaded from the GLO-90
        self.grid = None    # (south, west, rows, cols), in tiles from the equator and the meridian
        self.size = 0
        self.trimmed = 0
        self.sha256 = ""


def assemble(plan, cache):
    made = Made()
    for ty, tx in plan.tiles:
        got = load_tile(tile_file(cache, ty, tx, (ty, tx) in plan.moor))
        if got is None:
            raise Trouble("A tile went missing from the cache while the pack was being made - run it again.")
        flags, blocks = got
        if blocks is None:
            continue
        if flags & SEA and not plan.keep_sea:
            made.sea += 1
            continue
        made.stood_in += bool(flags & STOOD_IN)
        made.tiles.append((ty, tx) + blocks)
    if not made.tiles:
        raise Trouble("There is no land in that area - nothing to make a pack of. Check the place, or make the "
                      "radius bigger.")
    # Over the limits after all - the estimate's land a little short of the
    # maps' - the tiles furthest from the middle left off, rather than a pack
    # the Tab5 won't open.
    est = Estimate([(t[0], t[1]) for t in made.tiles], plan.grid)
    if est.too_big():
        lat, lon = middle_of(plan)
        my, mx = (lat + 90) * TPD, (lon + 180) * TPD
        made.tiles.sort(key=lambda t: (t[0] + 0.5 - my) ** 2 +
                        ((t[1] + 0.5 - mx) * tile_width_km(t[0]) / (TILE_DEG * KM_LAT)) ** 2)
        while made.tiles and Estimate([(t[0], t[1]) for t in made.tiles], plan.grid).too_big():
            drop = max(1, len(made.tiles) // 100)
            made.tiles = made.tiles[:-drop]
            made.trimmed += drop
        made.tiles.sort()
    if plan.grid is not None:
        made.grid = plan.grid
    else:
        tys, txs = [t[0] for t in made.tiles], [t[1] for t in made.tiles]
        made.grid = (min(tys) - 90 * TPD, min(txs) - 180 * TPD, max(tys) - min(tys) + 1, max(txs) - min(txs) + 1)
    return made


def write(made, out):
    """The pack, to a temporary file beside `out`, then renamed: never half
    a pack where the last was."""
    south, west, rows, cols = made.grid
    tiles = [(ty - 90 * TPD - south, tx - 180 * TPD - west, c, m, d) for ty, tx, c, m, d in made.tiles]
    out.parent.mkdir(parents=True, exist_ok=True)
    part = out.with_name(out.name + ".part")
    with open(part, "wb") as f:
        made.size, _, _ = gg.write_pack(f, south, west, rows, cols, tiles)
        f.flush()
        os.fsync(f.fileno())
    made.sha256 = hashlib.sha256(part.read_bytes()).hexdigest()
    try:
        replace(part, out)
    except PermissionError:
        raise Trouble(f"Couldn't replace {out} - is it open in another program, or on a card that is locked? The new "
                      f"pack is saved as {part}: rename it to {out.name} yourself.")


def notice_path(out):
    return out.with_name(f"{out.stem}-NOTICE.txt")


def preview_path(out):
    return out.with_name(f"{out.stem}-preview.png")


def write_notice(plan, made, out):
    areas = listed([c.label() for c in plan.choices])
    if any(c.kind in ("country", "region") for c in plan.choices) and plan.margin_km > 0:
        areas += f", and some {plan.margin_km:g} km round it"
    mixes = ("four mixes WorldCover has no class for - suburb, town, moor and rocky ground - worked out from its "
             "classes and the heights" if plan.moor else
             "three mixes WorldCover has no class for - suburb, town and rocky ground - worked out from its classes")
    glo90 = " Where the GLO-30 isn't public, the GLO-90 was used for the hillshade instead." if made.stood_in else ""
    fill = lambda text: textwrap.fill(text, 76)
    chosen = ("\n" + fill("The area was chosen with Natural Earth's countries and regions "
                           "(https://www.naturalearthdata.com), which are in the public domain.") + "\n"
              if any(c.kind in ("country", "region") for c in plan.choices) else "")
    # Word for word as the licences give them: the Copernicus DEM's notices
    # for data that has been changed - the WorldDEM-90's with its trade mark -
    # and the sentence on liability its licences have whoever passes it on
    # add; and WorldCover's citation as ESA gives it.
    text = f"""{out.name} - a ground pack for Overhead's 3D view

{fill(f"{out.name} holds what covers the land of {areas} - woods, fields, towns, water, bare rock and snow - and "
      f"its hills' light and shade and heights, for Overhead's 3D view to cover the land with and to draw it at its "
      f"height: {grouped(len(made.tiles))} tiles of 0.2 degrees.")}

It contains modified data from:

ESA WorldCover 2021 v200, 10m land cover (https://esa-worldcover.org):
© ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data
(2021) processed by ESA WorldCover consortium. Licensed under Creative
Commons Attribution 4.0 International (CC BY 4.0):
https://creativecommons.org/licenses/by/4.0/
Citation: Zanaga, D., Van De Kerchove, R., Daems, D., De Keersmaecker, W.,
Brockmann, C., Kirches, G., Wevers, J., Cartus, O., Santoro, M., Fritz, S.,
Lesiv, M., Herold, M., Tsendbazar, N.E., Xu, P., Ramoino, F., Arino, O.,
2022. ESA WorldCover 10 m 2021 v200. https://doi.org/10.5281/zenodo.7254221

The Copernicus DEM GLO-30 and GLO-90
(https://spacedata.copernicus.eu/collections/copernicus-digital-elevation-model):
produced using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus
Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European
Union and ESA; all rights reserved;
and produced using Copernicus WorldDEM™-90 © DLR e.V. 2010-2014 and © Airbus
Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European
Union and ESA; all rights reserved.

The organisations in charge of the Copernicus programme by law or by
delegation do not incur any liability for any use of the Copernicus
WorldDEM-30. The organisations in charge of the Copernicus programme by law
or by delegation do not incur any liability for any use of the Copernicus
WorldDEM™-90.

{fill(f"What was changed: the land cover was sampled into cells of 2.5 seconds of latitude, some 75m, at five "
      f"levels of detail, with {mixes}. The heights were made into a hillshade, lit from the north-west at 45 "
      f"degrees, and sampled every 15 seconds for the land's heights.{glo90}")}
{chosen}
{fill("Passing it on: whoever passes this pack on, changed or not, must pass this notice on with it, and is "
      "bound as its maker is - to give the credits and the sentences on liability above, not to suggest that "
      "Airbus Defence and Space, the European Union, ESA or anyone else in charge of the Copernicus programme "
      "endorses what they do, and to bind whoever they pass it on to in the same way. None of them endorses this "
      "pack or Overhead.")}

It was made by tools/make_ground_pack.py, in
{PROJECT}
on {datetime.date.today().isoformat()}.

SHA-256: {made.sha256}
"""
    notice_path(out).write_text(text, encoding="utf-8")


def level_cells(blocks, cols0, level):
    """A tile's cells at `level`, 0 to 4, rows north first."""
    if level == 0:
        d = np.frombuffer(zlib.decompress(blocks[2], -15), np.uint8)
        h, w = gg.ROWS0 // 2, cols0 // 2
        q = d.reshape(4, h, w)
        return np.vstack([np.hstack([q[0], q[1]]), np.hstack([q[2], q[3]])])
    if level == 4:
        rows, cols = gg.ROWS0 // 16, cols0 // 16
        return np.frombuffer(zlib.decompress(blocks[0], -15)[:rows * cols], np.uint8).reshape(rows, cols)
    m = np.frombuffer(zlib.decompress(blocks[1], -15), np.uint8)
    at = 0
    for lv in range(1, 4):
        rows, cols = gg.ROWS0 >> lv, cols0 >> lv
        if lv == level:
            return m[at:at + rows * cols].reshape(rows, cols)
        at += rows * cols


def font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # an older Pillow: its one small font
        return ImageFont.load_default()


def write_preview(plan, made, out):
    """A picture of the pack: each cell the 3D view's colour for what covers
    it, shaded as its hills lie; the areas chosen outlined; a key."""
    tys, txs = [t[0] for t in made.tiles], [t[1] for t in made.tiles]
    ty0, ty1, tx0, tx1 = min(tys), max(tys), min(txs), max(txs)
    rows, cols = ty1 - ty0 + 1, tx1 - tx0 + 1
    # Every tile as wide as it is on the ground, at the pack's middle: a
    # kilometre the same either way, a circle round.
    aspect = math.cos(math.radians((ty0 + ty1 + 1) / 2 / TPD - 90))
    most = 1600
    level = 4
    for lv in range(5):
        if rows * (gg.ROWS0 >> lv) <= most and cols * (gg.ROWS0 >> lv) * aspect <= most:
            level = lv
            break
    th = gg.ROWS0 >> level
    tw = max(1, round(th * aspect))
    picture = np.empty((rows * th, cols * tw, 3), np.uint8)
    picture[:] = OFF_PACK
    lut = np.zeros((256, 3), np.uint8)
    for v in range(256):
        rgb = np.array(COVER_RGB[v >> 4], float)
        light = 1.0 if v >> 4 in (gg.NONE, gg.WATER) else (v & 15) / 11
        lut[v] = np.clip(rgb * light * 1.3, 0, 255)
    counts = np.zeros(16, np.int64)
    for ty, tx, c, m, d in made.tiles:
        cells = level_cells((c, m, d), cols0_of(ty), level)
        cells = cells[:, (np.arange(tw) * cells.shape[1]) // tw]
        counts += np.bincount((cells >> 4).ravel(), minlength=16)
        r, q = (ty1 - ty) * th, (tx - tx0) * tw
        picture[r:r + th, q:q + tw] = lut[cells]
    image = Image.fromarray(picture)
    scale = 1.0
    if max(image.size) > 2400:
        scale = 2400 / max(image.size)
        box = getattr(Image, "Resampling", Image).BOX
        image = image.resize((max(1, int(image.width * scale)), max(1, int(image.height * scale))), box)
    draw = ImageDraw.Draw(image)
    north, west = (ty1 + 1) / TPD - 90, tx0 / TPD - 180
    px_lon, px_lat = image.width / (cols * TILE_DEG), image.height / (rows * TILE_DEG)

    def xy(lon, lat):
        return (lon - west) * px_lon, (north - lat) * px_lat

    for poly in plan.outlines:
        ring = np.asarray(poly[0], float)
        pts = np.column_stack([(ring[:, 0] - west) * px_lon, (north - ring[:, 1]) * px_lat])
        keep = np.ones(len(pts), bool)
        keep[1:] = np.hypot(*(np.diff(pts, axis=0).T)) >= 0.7
        pts = pts[keep]
        if len(pts) >= 2:
            draw.line([tuple(p) for p in pts], fill=(250, 226, 140), width=1)
    for lat, lon, km in plan.circles:
        pts = []
        for b in range(0, 361, 3):
            a = math.radians(b)
            dlat = km / KM_LAT * math.cos(a)
            dlon = km / (KM_LON * max(0.05, math.cos(math.radians(lat + dlat)))) * math.sin(a)
            pts.append(xy(lon + dlon, lat + dlat))
        draw.line(pts, fill=(250, 226, 140), width=1)
    # The key, and a line saying what it is, under the picture.
    small = font(14)
    shown = [i for i in range(16) if counts[i] > 0 and i != gg.NONE]
    key_h = 26 + 20 * ((len(shown) + 3) // 4)
    width = max(image.width, 640)
    sheet = Image.new("RGB", (width, image.height + key_h), (16, 20, 26))
    sheet.paste(image, ((width - image.width) // 2, 0))
    d = ImageDraw.Draw(sheet)
    title = (f"{out.name}: {listed([c.name for c in plan.choices])} - {grouped(len(made.tiles))} tiles, "
             f"{megabytes(made.size)}")
    d.text((8, image.height + 5), title, fill=(230, 230, 230), font=small)
    for n, i in enumerate(shown):
        x, y = 8 + (n % 4) * (width // 4), image.height + 26 + 20 * (n // 4)
        d.rectangle([x, y + 2, x + 14, y + 14], fill=tuple(int(v) for v in lut[(i << 4) | 11]))
        d.text((x + 20, y), COVER_NAMES[i], fill=(210, 210, 210), font=small)
    sheet.save(preview_path(out))


def about_line(made):
    """The ground pack's line in the Tab5's Settings > About, as
    src/info_screen.cpp words it for this pack."""
    south, west, rows, cols = made.grid

    def edge(tiles, plus, minus):
        return f"{abs(tiles) / TPD:.1f}{minus if tiles < 0 else plus}"

    return (f"Ground pack: ground.bin, {grouped(len(made.tiles))} tiles ({made.size / 1048576:.1f}MB), between "
            f"{edge(south, 'N', 'S')} and {edge(south + rows, 'N', 'S')}, {edge(west, 'E', 'W')} and "
            f"{edge(west + cols, 'E', 'W')}.")


def summary(plan, made, out, cache, seconds, program):
    est = Estimate([(t[0], t[1]) for t in made.tiles], made.grid)
    say()
    say(f"Done: {out.name} is ready - {grouped(len(made.tiles))} tiles, {megabytes(made.size)}, made in "
        f"{took(seconds)}.")
    say(f"  Area:         {listed([c.label() for c in plan.choices])}")
    if made.sea:
        say(f"  Open sea:     {grouped(made.sea)} tiles of nothing but sea left out - the 3D view draws the sea "
            "the same")
    if made.trimmed:
        say(f"  Trimmed:      {grouped(made.trimmed)} tiles furthest from the middle left off, to fit the Tab5")
    say(f"  Tab5 memory:  about {memory(est.coarse + est.index)} while the 3D view is open "
        f"(its limit: {memory(MAX_COARSE_BYTES)} for the far country, {grouped(MAX_TILES)} tiles)")
    if made.stood_in:
        say(f"  Hills:        {grouped(made.stood_in)} tiles shaded from the coarser GLO-90 (90 m), as the GLO-30 "
            "(30 m) isn't public there")
    else:
        say("  Hills:        shaded from the GLO-30 (30 m) everywhere")
    say(f"  The pack:     {out}")
    say(f"  Its credits:  {notice_path(out)} - keep it with the pack if you pass it on")
    say(f"  A picture:    {preview_path(out)} - open it to see what you made")
    say()
    say("Next:")
    say(f"  1. Copy {out.name} onto the Tab5's microSD card, into a folder named overhead at the top of the")
    say("     card, named ground.bin - so it is overhead/ground.bin, beside airfields.bin if you have it.")
    say("     The card must be FAT32, as for the airfield pack. The Tab5 reads one ground pack: this one")
    say("     takes the place of any that was there.")
    say("  2. Put the card in the Tab5 and turn it on, or restart it.")
    say("  3. Check it found the pack: on the Tab5, Settings > About. Under its first paragraph it says")
    say(f"     \"{about_line(made)}\"")
    say("     - or, if it can't use the pack, why, and what to do. Then follow an aircraft over it and tap 3D.")
    say()
    try:
        kept = sum(p.stat().st_size for p in (Path(cache) / "tiles").rglob("*.tile"))
        say(f"The tiles it made are kept in {cache} ({megabytes(kept)}), so that making it again, or a")
        say("bigger pack round it, is quicker. Delete that folder whenever you like.")
    except OSError:
        pass


# --- Choosing --------------------------------------------------------------

def coordinates(text):
    m = re.fullmatch(r"\s*([-+]?\d+(?:\.\d+)?)\s*[, ]\s*([-+]?\d+(?:\.\d+)?)\s*", text)
    if not m:
        return None
    lat, lon = float(m[1]), float(m[2])
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise Trouble(f"{text} isn't a latitude and longitude: latitude goes from -90 to 90, longitude from -180 to "
                      "180 (west and south negative).")
    return lat, lon


def geocode(text):
    """A town or place by name - its latitude, longitude and full name - from
    Open-Meteo's geocoding API; "Springfield, Illinois" to say which."""
    name, within = text, ""
    if "," in text:
        name, within = [s.strip() for s in text.split(",", 1)]
    query = urllib.parse.urlencode({"name": name, "count": 10, "language": "en", "format": "json"})
    request = urllib.request.Request(f"{GEOCODER}?{query}", headers={"User-Agent": AGENT})
    try:
        with urllib.request.urlopen(request, timeout=30, context=ssl_context()) as r:
            results = json.load(r).get("results") or []
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise Trouble(f"Couldn't look up \"{text}\": {getattr(e, 'reason', e)}. Check the computer is online, or give "
                      "its latitude and longitude instead, as 46.02,7.75.")
    if within:
        w = norm(within)
        results = [r for r in results if w in (norm(r.get("admin1", "")), norm(r.get("country", "")),
                                               norm(r.get("country_code", "")), norm(r.get("admin2", "")))]
    if not results:
        raise Trouble(f"Couldn't find a place called \"{text}\". Check the spelling, add its country - "
                      "\"Springfield, Illinois\" - or give its latitude and longitude, as 46.02,7.75.")
    r = results[0]
    label = lambda o: ", ".join(x for x in (o.get("name"), o.get("admin1"), o.get("country")) if x)
    full = label(r)
    others = []
    for o in results[1:]:
        # Others of the same name, somewhere else - not the town's own
        # heliport, nor a village with a name like it.
        far = math.hypot(o["latitude"] - r["latitude"], (o["longitude"] - r["longitude"]) *
                         math.cos(math.radians(r["latitude"]))) * KM_LAT > 20
        if norm(o.get("name", "")) == norm(r.get("name", "")) and far and label(o) not in others + [full]:
            others.append(label(o))
    others = others[:3]
    return float(r["latitude"]), float(r["longitude"]), full, others


def locate(text):
    """A place - "46.02,7.75", or a town's name - as its latitude,
    longitude and name, and the other places with that name."""
    at = coordinates(text)
    if at is not None:
        lat, lon = at
        return lat, lon, f"{abs(lat):.3f}{'N' if lat >= 0 else 'S'} {abs(lon):.3f}{'E' if lon >= 0 else 'W'}", []
    lat, lon, name, others = geocode(text)
    return lat, lon, f"{name} ({lat:.3f}, {lon:.3f})", others


def around(place, km):
    """A circle `km` round a place located."""
    if not km > 0:
        raise Trouble("The radius must be more than nought kilometres.")
    lat, lon, name, others = place
    c = Choice("circle", f"{km:g} km round {name}", "", "", circle=(lat, lon, km))
    c.others = others
    return c


def show_list(places, text):
    """The names it knows: the countries; a country's regions; or every
    name with `text` in it."""
    if not text:
        countries = places.countries()
        say("The countries it knows - type the name, or the two-letter code after it:")
        rows = sorted((p.get("NAME_EN") or p["NAME"], p.get("ISO_A2_EH") if p["ADM0_A3"] == p.get("ISO_A3_EH") else "")
                      for p in (f["properties"] for f in countries))
        cells = [f"{name} ({code})" if code and code != "-99" else name for name, code in rows]
        width = min(36, max(len(c) for c in cells)) + 2
        per = max(1, 110 // width)
        for i in range(0, len(cells), per):
            say("  " + "".join(c.ljust(width) for c in cells[i:i + per]).rstrip())
        say()
        say("For a country's states, provinces and regions: --list followed by its name, as --list Germany.")
        return
    key = norm(text)
    hits = places.countries_named(key)
    regions = places.regions()
    if len(hits) == 1:
        a3 = places.countries()[hits[0]]["properties"]["ADM0_A3"]
        mine = [f["properties"] for f in regions if f["properties"]["adm0_a3"] == a3]
        name = places.country_choice(hits[0]).name
        groups = {}
        for field, what in (("region", "Its regions, each several of the ones below"), ("geonunit", "Its parts")):
            values = sorted({p[field] for p in mine if p.get(field) and norm(p[field]) != norm(name) and
                             norm(p[field]) != norm(p.get("admin") or "")})
            if values:
                groups[what] = values
        for what, values in groups.items():
            say(f"{what}:")
            say("  " + "; ".join(values))
            say()
        kinds = sorted({(p.get("type_en") or "region") for p in mine})
        say(f"The {', '.join(k.lower() + 's' for k in kinds)} of {name}:")
        say("  " + "; ".join(sorted(p.get("name_en") or p["name"] for p in mine)))
        say()
        say("Type one as it is here - with its country if the name is used elsewhere too, as \"Limburg, Belgium\".")
        return
    found = []
    for f in places.countries():
        p = f["properties"]
        if any(key in norm(p.get(k) or "") for k in ("NAME_EN", "ADMIN", "NAME_LONG")):
            found.append(f"{p.get('NAME_EN') or p['NAME']} - a country")
    groups = set()
    for f in regions:
        p = f["properties"]
        names = [p.get("name"), p.get("name_en")] + str(p.get("name_alt") or "").split("|")
        if any(n and key in norm(n) for n in names):
            kind = (p.get("type_en") or "region").lower()
            found.append(f"{p.get('name_en') or p['name']}, {p['admin']} - {article(kind)} {kind}")
        for field in ("region", "geonunit"):
            v = p.get(field)
            if v and key in norm(v) and norm(v) != norm(p.get("admin") or ""):
                groups.add(f"{v}, {p['admin']} - {'a region' if field == 'region' else 'a part of the country'}")
    found += sorted(groups)
    if not found:
        say(f"No country, state or region with \"{text}\" in its name. For a town, use --around \"{text}\".")
        return
    say(f"Names with \"{text}\" in them:")
    for line in sorted(set(found))[:200]:
        say(f"  {line}")


def choose_interactively(places, program):
    """Asks where, until there is somewhere - the Choices."""
    say("Where would you like the land for? Type one of:")
    say("  a country                    France, or its code: FR")
    say("  a state, province or region  Texas, Bavaria, Scotland, Limburg, Belgium")
    say("  a town or city               Zermatt     - for a circle round it")
    say("  a latitude and longitude     46.02, 7.75 - for a circle round it")
    say("Type ? to see the names it knows.")
    choices = []
    while True:
        text = ask("Where? " if not choices else "Anywhere else? Type it, or press Enter to go on: ")
        if not text:
            if choices:
                return choices
            continue
        if text.startswith("?"):
            show_list(places, ask("Type part of a name, or a country for its regions (or Enter for every "
                                  "country): "))
            continue
        try:
            choice = pick_area(places, text) if coordinates(text) is None else None
            if choice is None:
                place = locate(text)
                say(f"  Found {place[2]}.")
                if place[3]:
                    say(f"  (Others with that name: {'; '.join(place[3])} - add the region or country to pick "
                        "one, as Springfield, Illinois.)")
                km = ask(f"  How far round it, in kilometres? [{RADIUS_KM}] ", str(RADIUS_KM))
                try:
                    km = float(km)
                except ValueError:
                    say("  That isn't a number of kilometres - try again.")
                    continue
                choice = around(place, km)
            say(f"  Got it: {choice.label()}.")
            choices.append(choice)
        except Trouble as t:
            say("  " + str(t).replace("\n", "\n  "))


def pick_area(places, text):
    """The country or region `text` names, asking which where it could be
    more than one - or None for one it doesn't, to look up as a town."""
    try:
        return places.resolve(text)
    except Ambiguous as a:
        say(f"  {a}")
        for n, c in enumerate(a.options, 1):
            say(f"    {n}. {c.label()}")
        pick = ask("  Which one? Type its number: ")
        if pick.isdigit() and 1 <= int(pick) <= len(a.options):
            return a.options[int(pick) - 1]
        raise Trouble("None picked - try again.")
    except Trouble as t:
        miss = str(t).split("\n")[0]
    say(f"  {text} isn't a country or region I know - looking it up as a town or place...")
    try:
        locate(text)
    except Trouble:
        raise Trouble(miss)
    return None


# --- The command ------------------------------------------------------------

def parse(argv):
    ap = argparse.ArgumentParser(
        prog="make_ground_pack.py",
        description="Makes ground.bin - the land and hills for Overhead's 3D view - for the area you choose. "
                    "With nothing after it, it asks.",
        epilog="Examples: Luxembourg | France Belgium | Texas | \"Limburg, Belgium\" | "
               "--around 46.02,7.75 --radius 25 | --around Zermatt --radius 25 | --list Germany")
    ap.add_argument("areas", nargs="*", help="countries, states, provinces or regions, by name or code")
    ap.add_argument("--around", metavar="PLACE", help="a circle round a place: LAT,LON or a town's name")
    ap.add_argument("--radius", type=float, default=None, metavar="KM",
                    help=f"the circle's radius in kilometres (default {RADIUS_KM})")
    ap.add_argument("--south", type=float)
    ap.add_argument("--north", type=float)
    ap.add_argument("--west", type=float)
    ap.add_argument("--east", type=float, help="with --south, --north and --west: a box of latitude and longitude")
    ap.add_argument("--out", "-o", default="ground.bin", metavar="FILE", help="where to save it (default ground.bin)")
    ap.add_argument("--margin", type=float, default=MARGIN_KM, metavar="KM",
                    help=f"land taken round the areas, for the hills over their borders (default {MARGIN_KM})")
    ap.add_argument("--moor", choices=("auto", "on", "off"), default="auto",
                    help="the British Isles' moor on high or steep grass: auto - in and near them only (default)")
    ap.add_argument("--jobs", type=int, default=8, help="tiles made at once (default 8)")
    ap.add_argument("--cache", metavar="DIR", help=f"where the tiles made are kept (default {default_cache()})")
    ap.add_argument("--estimate", action="store_true", help="say what it would take, and stop")
    ap.add_argument("--list", nargs="?", const="", metavar="TEXT",
                    help="the names it knows: countries; a country's regions; or names containing TEXT")
    return ap.parse_args(argv)


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    argv = sys.argv[1:] if argv is None else argv
    program = how_to_run()
    args = parse(argv)
    # Nowhere said: it asks - as when it is run with nothing after it, or
    # opened with a double click.
    interactive = not (args.areas or args.around or args.list is not None or
                       any(v is not None for v in (args.south, args.north, args.west, args.east)))
    try:
        status = make(args, interactive, program)
    except Trouble as t:
        say()
        say(str(t))
        status = 1
    except Stopped:
        say()
        say("Stopped. The tiles made so far are kept - run the same command again to carry on from there.")
        status = 130
    except KeyboardInterrupt:
        say()
        say("Stopped - nothing was made.")
        status = 130
    if interactive:
        try:
            input("\nPress Enter to close.")
        except (EOFError, KeyboardInterrupt):
            pass
    return status


def how_to_run():
    """How to run the tool again, as it was run: with Windows's py; else by
    the name that finds this very Python on the PATH - a venv's, once it is
    activated - or else its whole path."""
    if os.name == "nt":
        return "py make_ground_pack.py"
    for name in ("python3", "python"):
        found = shutil.which(name)
        if found and os.path.abspath(found) == os.path.abspath(sys.executable):
            return f"{name} make_ground_pack.py"
    return f"{shlex.quote(sys.executable)} make_ground_pack.py"


def make(args, interactive, program):
    firmware_limits()
    cache = Path(args.cache).expanduser() if args.cache else default_cache()
    places = Places(cache)
    if args.list is not None:
        show_list(places, args.list)
        return 0
    out = Path(args.out).expanduser()
    if out.is_dir():
        out = out / "ground.bin"
    out = out.resolve()
    if interactive:
        say("Overhead's ground pack maker")
        say("This makes ground.bin: the land and its hills for the Tab5's 3D view, for the area you choose.")
        say("It reads free maps from the internet as it goes, so keep the computer online.")
        say()
    box = [args.south, args.north, args.west, args.east]
    if any(v is not None for v in box) and not all(v is not None for v in box):
        raise Trouble("A box needs all four of --south, --north, --west and --east.")
    if args.margin < 0:
        raise Trouble("The margin can't be less than nought.")
    jobs = max(1, args.jobs)
    while True:
        if interactive:
            choices = choose_interactively(places, program)
        else:
            choices = []
            for a in args.areas:
                try:
                    choices.append(places.resolve(a))
                except Ambiguous as e:
                    raise Trouble(e.full())
            if args.around:
                c = around(locate(args.around), args.radius if args.radius is not None else RADIUS_KM)
                if getattr(c, "others", None):
                    say(f"Taking {c.name} - there are others with that name: {'; '.join(c.others)}. Add the region "
                        "or country, as \"Springfield, Illinois\", to pick another.")
                choices.append(c)
            elif args.radius is not None:
                raise Trouble("--radius goes with --around: --around LAT,LON --radius KM, or a town's name.")
            if all(v is not None for v in box):
                if not (args.south < args.north and args.west < args.east):
                    raise Trouble("The box is the wrong way round: --south below --north, --west below --east.")
                choices.append(Choice("box", f"{args.south:g} to {args.north:g} north, {args.west:g} to {args.east:g} "
                                             "east", "", "", box=tuple(box)))
            if not choices:
                raise Trouble("Say where: a country, a region, --around a place, or a box - or run it with nothing "
                              "after it to be asked.")
        say()
        say("Working out the tiles...")
        fetch_lists(cache)
        plan = plan_for(choices, places, cache, args.margin, args.moor)
        est = Estimate(plan.land, plan.grid, jobs, plan.work, plan.solid)
        describe(plan, est, jobs, out)
        if est.too_big():
            say()
            say(advice(plan, est, program))
            if interactive:
                say()
                say("Let's try again, smaller.")
                continue
            return 1
        if args.estimate:
            return 0
        if interactive:
            if not yes("\nMake it now? [Y/n] "):
                if yes("Choose somewhere else? [Y/n] "):
                    continue
                return 0
        break
    # The lists of names - some hundreds of megabytes read in - not wanted
    # while the tiles are made.
    places._regions = places._region_names = places._countries = places._country_names = None
    say()
    say(f"Making {out.name} - stop it with Ctrl+C whenever you like; the same command again carries on.")
    started = time.monotonic()
    build(plan, cache, jobs)
    made = assemble(plan, cache)
    write(made, out)
    write_notice(plan, made, out)
    try:
        write_preview(plan, made, out)
    except Exception as e:  # the picture is a nicety: the pack is made
        say(f"(The picture couldn't be drawn: {e})")
    summary(plan, made, out, cache, time.monotonic() - started, program)
    return 0


if __name__ == "__main__":
    sys.exit(main())
