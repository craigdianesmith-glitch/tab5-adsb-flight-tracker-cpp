#!/usr/bin/env python3
"""Generates src/airport_info_table.inc from the OurAirports dataset.

What an airport's card says of it, for every airport in src/airports.cpp's
table: its name, its ICAO code, how high it is, and the radio frequencies
someone with a scanner would tune to hear it - the tower, the approach, the
radar and the control centre over it, with the ground, the clearance
delivery, the departures, the ATIS and, at a field without a tower, the
information or traffic frequency. OurAirports (ourairports.com) is public
domain, and the same source the airport and runway tables came from.

    tools/gen_airport_info.py                                   # downloads the two CSVs
    tools/gen_airport_info.py airports.csv airport-frequencies.csv

A frequency there belongs to an airport and has no place of its own - there
is nothing to say where its aerial stands - so these are not marked on the
radar as the navaids are, but listed on the airport's card.

The dataset's own types are a few dozen spellings, typed in by hand over the
years, and are sorted here into the nine the card has. Where the type alone
doesn't say, the description does: an "A/D" - approach and departure - that
describes itself as "SAN JUAN CNTR" is the centre.

Left out: anything outside the VHF airband and the military UHF one - the
dataset has HF, typing slips and a television channel among them - flight
service and company frequencies, the automatic weather stations, and what
is past the fourth frequency for any one use.
"""

import csv
import io
import re
import sys
import unicodedata
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA_URL = "https://davidmegginson.github.io/ourairports-data/"

# As AtcUse in include/airport_info.h numbers them, and the order a card
# lists them in.
USES = ["TOWER", "GROUND", "DELIVERY", "APPROACH", "DEPARTURE", "RADAR", "CONTROL", "ATIS", "INFO"]
TYPES = {
    "TOWER": "TWR TOWER",
    "GROUND": "GND GROUND SMC GCCD",
    "DELIVERY": "CLD DEL DELIVERY CLNC",
    "APPROACH": "APP APPR APP/DEP ARR A/D TCA",
    "DEPARTURE": "DEP",
    "RADAR": "RDR RADAR DIR GCA",
    "CONTROL": "CNTR ACC CTR ARTC PAL",
    "INFO": "AFIS INFO FIS AAS CTAF UNIC ATF MF A/G",
}
USE_OF_TYPE = {kind: use for use, kinds in TYPES.items() for kind in kinds.split()}
# A description with one of these in it is a control centre's, whatever type
# it was filed under - but a tower's or a ground controller's is taken at its
# word, "GROUND CONTROL" being no centre.
CONTROL_WORDS = {"CNTR", "CENTER", "CENTRE", "CONTROL", "CON", "ACC", "ARTCC"}
SURE = {"TOWER", "GROUND", "DELIVERY", "DEPARTURE"}
MAX_PER_USE = 4
NAME_MAX = 66  # two lines of the card


def open_csv(arg, name):
    if arg:
        return open(arg, encoding="utf-8", newline="")
    with urllib.request.urlopen(DATA_URL + name) as r:
        return io.StringIO(r.read().decode("utf-8"), newline="")


# The display's font is plain ASCII: accents dropped rather than drawn as junk.
def ascii_text(s):
    s = unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode("ascii")
    return s.replace("\\", "/").replace('"', "'").strip()


def use_of(kind, description):
    kind = kind.strip().upper()
    if kind.startswith("ATIS"):
        return "ATIS"
    use = USE_OF_TYPE.get(kind)
    if use in SURE:
        return use
    if (use or kind == "FSS") and set(re.split(r"[^A-Z]+", description.upper())) & CONTROL_WORDS:
        return "CONTROL"
    return use


def main():
    args = sys.argv[1:] + [None, None]
    codes = re.findall(r'\{"([A-Z]{3})",', (ROOT / "src" / "airports.cpp").read_text())

    # The same filter the airport table was made with, so each code is the
    # one airport it names there.
    airports = {}
    idents = {}
    with open_csv(args[0], "airports.csv") as f:
        for r in csv.DictReader(f):
            if (r["iata_code"] in codes and r["type"] in ("large_airport", "medium_airport")
                    and r["scheduled_service"] == "yes"):
                icao = next((c for c in (r.get("icao_code", ""), r["ident"]) if re.fullmatch(r"[A-Z]{4}", c or "")), "")
                idents[r["ident"]] = r["iata_code"]
                airports[r["iata_code"]] = (icao, int(float(r["elevation_ft"] or 0)), ascii_text(r["name"])[:NAME_MAX],
                                            r["type"] == "large_airport")

    frequencies = {code: {use: [] for use in USES} for code in airports}
    with open_csv(args[1], "airport-frequencies.csv") as f:
        for r in csv.DictReader(f):
            code = idents.get(r["airport_ident"])
            use = use_of(r["type"], r["description"])
            try:
                khz = round(float(r["frequency_mhz"]) * 1000)
            except ValueError:
                continue
            if code and use and (108000 <= khz < 137000 or 225000 <= khz <= 400000):
                if khz not in frequencies[code][use]:
                    frequencies[code][use].append(khz)

    rows, info = [], []
    for code in sorted(airports):
        icao, elevation, name, large = airports[code]
        first = len(rows)
        for i, use in enumerate(USES):
            rows += [(khz, i) for khz in sorted(frequencies[code][use])[:MAX_PER_USE]]
        info.append(f'{{"{code}", "{icao}", {elevation}, {first}, {len(rows) - first}, {str(large).lower()}, "{name}"}},')

    out = [
        "// Generated by tools/gen_airport_info.py from OurAirports' airports.csv and",
        "// airport-frequencies.csv - do not edit.",
        "// Each frequency in kHz, and what it is for, as AtcUse numbers them: an airport's",
        "// together, by use and then by frequency.",
        "const AirportFrequency AIRPORT_FREQUENCIES[] = {",
    ]
    for i in range(0, len(rows), 8):
        out.append("    " + " ".join(f"{{{khz}, {use}}}," for khz, use in rows[i:i + 8]))
    out += [
        "};",
        "// IATA code, ICAO code, elevation in feet, its first frequency above and how many,",
        "// whether OurAirports has it as a large airport, name - by IATA code.",
        "const AirportInfo AIRPORT_INFO[] = {",
    ]
    out += ["    " + line for line in info]
    out.append("};")
    path = ROOT / "src" / "airport_info_table.inc"
    path.write_text("\n".join(out) + "\n")
    listed = sum(1 for code in airports if any(frequencies[code].values()))
    print(f"{len(rows)} frequencies at {listed} of {len(airports)} airports -> {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
