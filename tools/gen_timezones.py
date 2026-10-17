#!/usr/bin/env python3
"""Generates src/timezone_table.inc: which time zone each part of the world
keeps, and each zone's rules - for the clock to show the local time where an
aircraft followed is, anywhere.

    pip install timezonefinder        # in a venv of its own
    tools/gen_timezones.py [step]     # degrees a cell, 0.2 unless given

Where: the world in cells `step` degrees square, each the zone
timezonefinder puts its middle in - the zones' boundaries being Evan
Siroky's timezone-boundary-builder, from OpenStreetMap, with the seas: a
country's coastal waters its own, the open sea in fifteen-degree bands of
whole hours. Each row of cells written as runs of one zone, a zone where it
starts and how many cells it goes on for.

What: each zone's clock as the IANA time zone database has it, worked out by
Python's zoneinfo from /usr/share/zoneinfo: where it starts in 2025, and each
change from then to 2037 - summer time in and out, Morocco's hour back for
Ramadan, a country's new rules - as the moment and the offset and name after
it; and then, for the years after, the POSIX TZ string each zone's compiled
file ends with, "CET-1CEST,M3.5.0,M10.5.0/3", which says its rules for any
year from there. (Not that string alone: Morocco's, for one, holds only once
its Ramadans are past, in 2087.) Zones whose clocks agree all through are
one: half of Europe a single run.

The boundaries are © OpenStreetMap contributors and timezone-boundary-builder,
under the Open Database License (ODbL 1.0): so is the table this makes, and
the About screen says so. The IANA database is public domain.
"""

import sys
from datetime import datetime, timezone
from pathlib import Path
from zoneinfo import ZoneInfo

from timezonefinder import TimezoneFinder

ROOT = Path(__file__).resolve().parent.parent
ZONEINFO = Path("/usr/share/zoneinfo")
FROM = int(datetime(2025, 1, 1, tzinfo=timezone.utc).timestamp())
UNTIL = int(datetime(2037, 1, 1, tzinfo=timezone.utc).timestamp())


def posix_rule(zone):
    """The POSIX TZ string at the end of a zone's compiled file."""
    data = (ZONEINFO / zone).read_bytes()
    return data.rstrip(b"\n").rsplit(b"\n", 1)[-1].decode()


def clock_of(zone):
    """A zone's offset in minutes and name at FROM, and each change after."""
    z = ZoneInfo(zone)

    def at(t):
        d = datetime.fromtimestamp(t, z)
        return int(d.utcoffset().total_seconds()) // 60, d.tzname()

    first = now = at(FROM)
    changes, t = [], FROM
    while t < UNTIL:
        day = min(t + 86400, UNTIL)
        if at(day) == now:
            t = day
            continue
        low, high = t, day  # the change is in (low, high]
        while high - low > 1:
            middle = (low + high) // 2
            low, high = (middle, high) if at(middle) == now else (low, middle)
        now = at(high)
        changes.append((high,) + now)
        t = high
    return first, tuple(changes)


def tzdata_version():
    for name in ("tzdata.zi", "+VERSION"):
        path = ZONEINFO / name
        if path.exists():
            first = path.read_text().splitlines()[0]
            return first.split()[-1] if first.startswith("#") else first.strip()
    return "unknown"


def main():
    step = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
    rows, cols = round(180 / step), round(360 / step)
    finder = TimezoneFinder()
    groups, group_of, zones_in = [], {}, {}

    def group_index(zone):
        zone = zone or "Etc/UTC"
        key = clock_of(zone) + (posix_rule(zone),)
        if key not in group_of:
            assert len(groups) < 256, "more zones than a byte can say"
            group_of[key] = len(groups)
            groups.append(key)
            zones_in[key] = []
        zones_in[key].append(zone)
        return group_of[key]

    cache = {}
    data, offsets = bytearray(), []
    for r in range(rows):
        lat = 90 - (r + 0.5) * step
        offsets.append(len(data))
        run_group, run_length = None, 0
        for c in range(cols):
            lon = -180 + (c + 0.5) * step
            zone = finder.timezone_at(lng=lon, lat=lat)
            if zone not in cache:
                cache[zone] = group_index(zone)
            group = cache[zone]
            if group == run_group:
                run_length += 1
                continue
            if run_group is not None:
                data += encode(run_length, run_group)
            run_group, run_length = group, 1
        data += encode(run_length, run_group)
        if r % 300 == 0:
            print(f"  row {r} of {rows}: {len(data)} bytes so far", file=sys.stderr)
    offsets.append(len(data))

    names = sorted({first[1] for first, _, _ in groups} | {c[2] for _, changes, _ in groups for c in changes})
    name_of = {n: i for i, n in enumerate(names)}
    changes_out, groups_out = [], []
    for (offset, name), changes, rule in groups:
        groups_out.append((offset, name_of[name], len(changes_out), len(changes), rule))
        changes_out += [(t, o, name_of[n]) for t, o, n in changes]

    lines = [
        "// Generated by tools/gen_timezones.py - do not edit.",
        "// The world's time zones: their boundaries (c) OpenStreetMap contributors and",
        "// timezone-boundary-builder, under the Open Database License (ODbL 1.0), which",
        "// this table is under too; their clocks from the IANA time zone database",
        f"// ({tzdata_version()}), public domain.",
        f"constexpr float ZONE_STEP = {step}f;  // degrees a cell",
        f"constexpr int ZONE_ROWS = {rows}, ZONE_COLS = {cols};",
        f"constexpr int64_t ZONE_FROM = {FROM}, ZONE_UNTIL = {UNTIL};  // 2025 to 2037, UTC",
        "// What the zones call their times: \"CEST\", \"+03\".",
        "const char *const ZONE_NAMES[] = {",
    ]
    lines += ["    " + " ".join(f'"{n}",' for n in names[i:i + 10]) for i in range(0, len(names), 10)]
    lines += [
        "};",
        "// Each change to a zone's clock from 2025 to 2037: when, the offset after it in",
        "// minutes east of UTC, and its name after it.",
        "const ZoneChange ZONE_CHANGES[] = {",
    ]
    lines += ["    " + " ".join(f"{{{t}, {o}, {n}}}," for t, o, n in changes_out[i:i + 4])
              for i in range(0, len(changes_out), 4)]
    lines += [
        "};",
        "// Each zone - and the places keeping its time: its offset and name in 2025, its",
        "// changes after, and its rules from 2037 as a POSIX TZ string.",
        "const ZoneClock ZONE_CLOCKS[] = {",
    ]
    for (offset, name, first, count, rule), key in zip(groups_out, groups):
        places = ", ".join(zones_in[key][:4]) + (", ..." if len(zones_in[key]) > 4 else "")
        lines.append(f'    {{{offset}, {name}, {first}, {count}, "{rule}"}},  // {places}')
    lines += [
        "};",
        "// Where each row of cells starts in ZONE_RUNS, north to south, and its end.",
        "const uint32_t ZONE_ROW_AT[] = {",
    ]
    lines += ["    " + ", ".join(str(o) for o in offsets[i:i + 12]) + "," for i in range(0, len(offsets), 12)]
    lines += [
        "};",
        "// The runs, west to east from 180: how many cells - a byte, or two with the top bit",
        "// of the first set - and the zone.",
        "const uint8_t ZONE_RUNS[] = {",
    ]
    lines += ["    " + ", ".join(str(b) for b in data[i:i + 24]) + "," for i in range(0, len(data), 24)]
    lines += ["};"]
    (ROOT / "src" / "timezone_table.inc").write_text("\n".join(lines) + "\n")
    print(f"{len(groups)} clocks for {len(cache)} zones, {len(changes_out)} changes, {len(names)} names; "
          f"{rows}x{cols} cells in {len(data)} bytes of runs -> src/timezone_table.inc")


def encode(length, rule):
    if length < 128:
        return bytes([length, rule])
    return bytes([0x80 | length >> 8, length & 0xFF, rule])


if __name__ == "__main__":
    main()
