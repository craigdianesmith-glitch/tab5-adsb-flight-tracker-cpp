#!/usr/bin/env python3
"""Generates src/military_*.inc from the OurAirports dataset.

The military's airfields, which the airport table leaves out - it has only
the airports with scheduled service - so that a military flight's base is on
the radar, in the airport zoom and in the 3D view: RAF Lossiemouth, Brize
Norton, Lakenheath, Mildenhall, Ramstein, and some five hundred more round
the world. OurAirports (ourairports.com) is public domain, and the same
source as the airport, runway and airport card tables.

    tools/gen_military.py                                            # downloads the three CSVs
    tools/gen_military.py airports.csv runways.csv airport-frequencies.csv

An airfield is taken to be the military's by its name - "RAF", "Air Base",
"Naval Air Station", "Army Airfield" and the same in a dozen languages - or
by OurAirports' keywords saying so; and is kept only with at least one open
runway whose ends are both placed, there being nothing to draw it by
otherwise. One the airport table already has - a field civil and military
alike, as Bodo or Eindhoven - is left to it. Its code is its IATA code where
it has one no other airport in either table has, and otherwise its four
letters from ICAO: EGXE for Leeming.

Written as the airport, runway and card tables are, sorted by code, in three
files - the airfields, their runways, and their frequencies and cards - for
src/airports.cpp, src/runways.cpp and src/airport_info.cpp to look in after
their own.
"""

import csv
import importlib.util
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# The airport card generator's sorting of frequencies, used as it is.
_spec = importlib.util.spec_from_file_location("gen_airport_info", ROOT / "tools" / "gen_airport_info.py")
info = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(info)

MILITARY_NAME = re.compile(
    r"(?i)\b(air base|air force base|afb|raf|royal air force|rnas|naval air|nas|naval station|navy|marine corps air"
    r"|mcas|army air ?field|army heliport|aaf|military|air station|airbase|ang base|air national guard|joint base"
    r"|jrb|base a[eé]rea|base a[eé]rienne|fliegerhorst|heeresflugplatz|marinefliegerhorst|vliegbasis"
    r"|luchtmachtbasis|flygflottilj|flygbase|aeroporto militare|aeroporto da for[cç]a a[eé]rea|lotnisko wojskowe"
    r"|baza lotnicza|letecka zakladna|jasdf|jmsdf|jgsdf|plaaf|iaf station|air force station)\b"
)
# Proper ICAO codes only: OurAirports' own idents for unnamed fields - US-1234,
# GB-0123 - aren't codes anyone would know a field by.
ICAO = re.compile(r"[A-Z]{4}")
NEAR_CIVIL_KM = 3  # an airport the table already has, under another name


def distance_km(lat0, lon0, lat1, lon1):
    p0, p1 = math.radians(lat0), math.radians(lat1)
    a = math.sin((p1 - p0) / 2) ** 2 + math.cos(p0) * math.cos(p1) * math.sin(math.radians(lon1 - lon0) / 2) ** 2
    return 12742 * math.asin(math.sqrt(a))


def main():
    args = sys.argv[1:] + [None, None, None]
    civil = {code: (float(lat), float(lon)) for code, lat, lon in
             re.findall(r'\{"([A-Z]{3})", (-?[\d.]+)f, (-?[\d.]+)f\}', (ROOT / "src" / "airports.cpp").read_text())}

    # Every IATA code in the dataset, to keep a base's own from clashing with
    # another airport's - one closed, or not in the table.
    candidates, taken = {}, set(civil)
    with info.open_csv(args[0], "airports.csv") as f:
        for r in csv.DictReader(f):
            if r["type"] not in ("large_airport", "medium_airport", "small_airport"):
                continue
            military = MILITARY_NAME.search(r["name"]) or "military" in (r.get("keywords") or "").lower()
            if not military or r["iata_code"] in civil:
                continue
            lat, lon = float(r["latitude_deg"]), float(r["longitude_deg"])
            if any(abs(lat - clat) < 0.1 and distance_km(lat, lon, clat, clon) < NEAR_CIVIL_KM
                   for clat, clon in civil.values()):
                continue
            icao = next((c for c in (r.get("icao_code", ""), r.get("gps_code", ""), r["ident"])
                         if ICAO.fullmatch(c or "")), "")
            candidates[r["ident"]] = {
                "iata": r["iata_code"], "icao": icao, "lat": lat, "lon": lon,
                "elevation": int(float(r["elevation_ft"] or 0)), "name": info.ascii_text(r["name"])[:info.NAME_MAX],
                "large": r["type"] == "large_airport", "runways": [], "frequencies": {u: [] for u in info.USES},
            }

    with info.open_csv(args[1], "runways.csv") as f:
        for r in csv.DictReader(f):
            field = candidates.get(r["airport_ident"])
            if field is None or r["closed"] == "1":
                continue
            le, he = r["le_ident"].strip(), r["he_ident"].strip()
            if not (le[:1].isdigit() and he[:1].isdigit()):
                continue
            try:
                ends = [(le, float(r["le_latitude_deg"]), float(r["le_longitude_deg"])),
                        (he, float(r["he_latitude_deg"]), float(r["he_longitude_deg"]))]
            except ValueError:
                continue  # an end not placed
            width = int(float(r["width_ft"])) if r["width_ft"] else 150
            field["runways"].append((min(max(width, 30), 400), ends))

    with info.open_csv(args[2], "airport-frequencies.csv") as f:
        for r in csv.DictReader(f):
            field = candidates.get(r["airport_ident"])
            use = info.use_of(r["type"], r["description"])
            try:
                khz = round(float(r["frequency_mhz"]) * 1000)
            except ValueError:
                continue
            if field and use and (108000 <= khz < 137000 or 225000 <= khz <= 400000):
                if khz not in field["frequencies"][use]:
                    field["frequencies"][use].append(khz)

    # Codes: IATA where it is the base's alone, else ICAO; a base with
    # neither, or with no runways, is left out.
    iata_count = {}
    for field in candidates.values():
        if field["iata"]:
            iata_count[field["iata"]] = iata_count.get(field["iata"], 0) + 1
    fields = {}
    for ident, field in candidates.items():
        if not field["runways"]:
            continue
        if field["iata"] and iata_count[field["iata"]] == 1 and field["iata"] not in taken:
            code = field["iata"]
        elif field["icao"] and field["icao"] not in taken:
            code = field["icao"]
        else:
            continue
        taken.add(code)
        fields[code] = field

    airports, runways, cards, frequencies = [], [], [], []
    for code in sorted(fields):
        field = fields[code]
        airports.append(f'{{"{code}", {field["lat"]:.4f}f, {field["lon"]:.4f}f}},')
        for width, ((e0, a0, o0), (e1, a1, o1)) in sorted(field["runways"], key=lambda r: r[1][0][0]):
            # The low-numbered end first, as the runway table has them.
            if int(re.match(r"\d+", e0).group()) > int(re.match(r"\d+", e1).group()):
                (e0, a0, o0), (e1, a1, o1) = (e1, a1, o1), (e0, a0, o0)
            runways.append(f'{{"{code}", {width}, {field["elevation"]}, false, '
                           f'{{{{"{e0}", {a0:.6f}f, {o0:.6f}f}}, {{"{e1}", {a1:.6f}f, {o1:.6f}f}}}}}},')
        first = len(frequencies)
        for i, use in enumerate(info.USES):
            frequencies += [(khz, i) for khz in sorted(field["frequencies"][use])[:info.MAX_PER_USE]]
        cards.append(f'{{"{code}", "{field["icao"]}", {field["elevation"]}, {first}, {len(frequencies) - first}, '
                     f'{str(field["large"]).lower()}, "{field["name"]}"}},')

    head = ["// Generated by tools/gen_military.py from OurAirports' airports.csv, runways.csv",
            "// and airport-frequencies.csv - do not edit."]
    files = {
        "military_airports.inc": head + [
            "// The military's airfields: code (IATA, or ICAO where it has none), latitude, longitude.",
            "const AirportEntry MILITARY_AIRPORTS[] = {",
        ] + ["    " + " ".join(airports[i:i + 3]) for i in range(0, len(airports), 3)] + ["};"],
        "military_runways.inc": head + [
            "// Their runways, as the runway table has them.",
            "const Runway MILITARY_RUNWAYS[] = {",
        ] + ["    " + line for line in runways] + ["};"],
        "military_info.inc": head + [
            "// Their frequencies and cards, as the airport card table has them.",
            "const AirportFrequency MILITARY_FREQUENCIES[] = {",
        ] + ["    " + " ".join(f"{{{khz}, {use}}}," for khz, use in frequencies[i:i + 8])
             for i in range(0, len(frequencies), 8)] + [
            "};",
            "const AirportInfo MILITARY_INFO[] = {",
        ] + ["    " + line for line in cards] + ["};"],
    }
    for name, lines in files.items():
        (ROOT / "src" / name).write_text("\n".join(lines) + "\n")
    print(f"{len(fields)} military airfields, {len(runways)} runways, {len(frequencies)} frequencies -> "
          f"src/military_{{airports,runways,info}}.inc")
    for code in ("LMO", "BZZ", "LKZ", "MHZ", "QCY", "WTN", "KNF", "EGXE", "EGOS", "RMS"):
        field = fields.get(code)
        print(f"  {code}: " + (f'{field["name"]}, {len(field["runways"])} runways' if field else "-"))


if __name__ == "__main__":
    main()
