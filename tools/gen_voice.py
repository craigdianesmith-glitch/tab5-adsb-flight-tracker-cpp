#!/usr/bin/env python3
"""Generate the spoken words into src/voice_english.inc.

    python tools/gen_voice.py --piper PATH/TO/piper --model PATH/TO/en_GB-cori-medium.onnx

Speaks each word with Piper (https://github.com/rhasspy/piper), a free offline
neural text-to-speech engine - `pip install piper-tts` in a virtualenv, and a
voice from https://huggingface.co/rhasspy/piper-voices - trims the silence off
either end, brings it up to a common peak, and writes it at 16kHz - resampled
from the voice's own 22.05kHz, a quarter less flash for speech the Tab5's
small speaker plays no differently - as 4-bit IMA ADPCM, a quarter of what
16-bit samples would take, encoded by the search in tools/adpcm_search.c (see
tools/ima_adpcm.py), for the firmware to turn back into samples as each word
is said. The firmware has one voice, every word said in it - see
src/sound.cpp:

    en_GB-cori-medium  English: LibriVox recordings, public domain

as its model card on Hugging Face says - credited on the About screen,
src/info_screen.cpp. Until 1.5.1 it had two more, a Scottish voice
(en_GB-alba-medium) and an American one (en_US-amy-medium): both were taken
out, having been fine-tuned from Piper's lessac voice, whose Blizzard 2013
data is licensed for research purposes only. A voice is only as free as what
it was made from, so before another goes in, read on its model card what it
was trained - or fine-tuned - from, and that data's licence.
"""
import argparse
import array
import io
import math
import os

sys_path = os.path.dirname(os.path.abspath(__file__))
import re
import subprocess
import sys
import wave

# The name each clip has in the firmware, and what is said.
WORDS = [
    ("EMERGENCY", "Emergency."),
    ("MILITARY", "Military."),
    ("RARE", "Rare aircraft."),
    ("WATCHLIST", "Watchlist."),
]
# A callsign's letters and digits, as a pilot or controller reads them out:
# the ICAO phonetic alphabet - spelt as Piper says them right - and niner.
LETTERS = ["Alpha", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot", "Golf", "Hotel", "India", "Juliet", "Kilo",
           "Lima", "Mike", "November", "Oscar", "Papa", "Quebec", "Romeo", "Sierra", "Tango", "Uniform", "Victor",
           "Whiskey", "X-ray", "Yankee", "Zulu"]
DIGITS = ["Zero", "One", "Two", "Three", "Four", "Five", "Six", "Seven", "Eight", "Niner"]
# The makers in src/aircraft_db.cpp's type names, as each name begins, and
# how each is said - its model is then spelt from the letters and digits
# above, "Airbus... Alpha three eight zero". Longer names go before the ones
# they begin with, Lockheed Martin before Lockheed, for the firmware to find
# the longest. Every type name there has to begin with one of these: the
# script stops if one doesn't.
MAKERS = [
    ("Bell Boeing", "Bell Boeing"), ("Lockheed Martin", "Lockheed Martin"), ("Northrop Grumman", "Northrop Grumman"),
    ("McDonnell Douglas", "McDonnell Douglas"), ("General Dynamics", "General Dynamics"),
    ("General Atomics", "General Atomics"), ("De Havilland", "De Havilland"), ("DG Flugzeugbau", "D G Flugzeugbau"),
    ("Air Tractor", "Air Tractor"), ("BAE Systems", "B A E Systems"), ("AgustaWestland", "Agusta Westland"),
    ("Aerospatiale", "Aerospatiale"), ("Agusta", "Agusta"), ("Airbus", "Airbus"), ("ATR", "A T R"),
    ("Beechcraft", "Beechcraft"), ("Bell", "Bell"), ("Boeing", "Boeing"), ("Bombardier", "Bombardier"),
    ("Cessna", "Cessna"), ("Cirrus", "Cirrus"), ("Dassault", "Dassault"), ("Embraer", "Embraer"),
    ("Eurocopter", "Eurocopter"), ("Eurofighter", "Eurofighter"), ("Fairchild", "Fairchild"),
    ("Grumman", "Grumman"), ("Gulfstream", "Gulfstream"), ("Hawker", "Hawker"), ("Kawasaki", "Kawasaki"),
    ("Learjet", "Learjet"), ("Leonardo", "Leonardo"), ("Lockheed", "Lockheed"), ("Mil", "Mil"),
    ("NHIndustries", "N H Industries"), ("Northrop", "Northrop"), ("Panavia", "Panavia"), ("Piaggio", "Piaggio"),
    ("Piper", "Piper"), ("Robinson", "Robinson"), ("Rockwell", "Rockwell"), ("SEPECAT", "Sepecat"),
    ("Sikorsky", "Sikorsky"), ("Socata", "Socata"), ("Swearingen", "Swearingen"), ("Transall", "Transall"),
]
# Variants worth hearing, said after the model where a name has them.
VARIANTS = [("NEO", "neo"), ("MAX", "max"), ("DREAMLINER", "Dreamliner")]
# The airport zoom's announcements: "easyJet five three Tango Hotel, runway
# two three, cleared for take-off" - the airline by its name in
# src/aircraft_db.cpp's table, every one of which is said, and how a few of
# them are said where Piper would get the name itself wrong.
AIRLINE_SAID = {
    "easyJet": "easyJet", "Jet2.com": "Jet two", "KLM": "K L M", "TUI Airways": "Tooey Airways",
    "LOT Polish Airlines": "Lot Polish Airlines", "TAP Air Portugal": "Tap Air Portugal",
    "ITA Airways": "I T A Airways", "Flybe": "Fly bee", "JetBlue": "Jet Blue", "Aer Lingus": "Air Lingus",
    "EgyptAir": "Egypt Air", "Wideroe": "Veederer",
    # The cargo operators - FedEx and the rest, spelt where they are said so.
    "FedEx": "Fed Ex", "UPS": "U P S", "DHL": "D H L", "AeroLogic": "Aero Logic", "Cargolux": "Cargo lux",
    "ABX Air": "A B X Air", "ASL Airlines": "A S L Airlines", "Cargojet": "Cargo jet", "Amerijet": "Ameri jet",
}
PHRASES = [
    ("RUNWAY", "Runway,"),
    ("SIDE_L", "left,"), ("SIDE_R", "right,"), ("SIDE_C", "centre,"),
    ("CLEARED_TAKEOFF", "cleared for take-off."),
    ("CLEARED_LAND", "cleared to land."),
]
# Read aloud - see src/sound.cpp's soundReadAircraft() and the cards': an
# aircraft's height, speed and heading as a controller says them - "four
# thousand four hundred feet", "flight level three five zero", "one seven
# five knots", "heading two four zero" - and a frequency, "one one eight
# decimal eight".
READ = [
    ("DECIMAL", "decimal,"), ("FEET", "feet,"), ("THOUSAND", "thousand,"), ("HUNDRED", "hundred,"),
    ("FLIGHT_LEVEL", "flight level,"), ("KNOTS", "knots,"), ("HEADING", "heading,"), ("CLIMBING", "climbing,"),
    ("DESCENDING", "descending,"), ("LEVEL", "level,"), ("ON_GROUND", "on the ground."), ("CHANNEL", "channel,"),
]
# A navaid's kind, in NavaidType's order, as a pilot says it.
NAVAID_TYPES = [("NAV_VOR", "V O R,"), ("NAV_VOR_DME", "V O R D M E,"), ("NAV_VORTAC", "Vortac,"),
                ("NAV_TACAN", "Tacan,"), ("NAV_DME", "D M E,"), ("NAV_NDB_DME", "N D B D M E,")]
# Who is on an airport's frequency, in AtcUse's order.
ATC_USES = [("ATC_TOWER", "Tower,"), ("ATC_GROUND", "Ground,"), ("ATC_DELIVERY", "Delivery,"),
            ("ATC_APPROACH", "Approach,"), ("ATC_DEPARTURE", "Departure,"), ("ATC_RADAR", "Radar,"),
            ("ATC_CONTROL", "Control,"), ("ATC_ATIS", "Atis,"), ("ATC_INFO", "Information,")]
# Words a voice says wrongly as written, and how to write them for it to say
# them right - found by ear, a few spellings tried for each, by the voice's
# --name. The English voice says every one right as it is written.
RESPELT = {}
PEAK = 0.89  # of full scale: about -1dB, loud on the Tab5's small speaker without clipping
# Speech peaks only now and then, a dozen dB over most of a word, where the
# alert's chime is a square wave at full level throughout - so at the same
# peak the voice sounded much the quieter. It is squeezed, as broadcast
# speech is: driven this far into a soft limiter, which brings the quiet of
# each word up by the same and leaves its peaks where they were. 1 for none.
DRIVE = 3.0
SILENCE = 0.02  # of the clip's own peak: quieter than this at either end is trimmed
MARGIN_MS = 30  # kept either side of the speech, so its first and last sounds aren't clipped

ap = argparse.ArgumentParser()
ap.add_argument("--piper", required=True, help="the piper executable")
ap.add_argument("--model", required=True, help="the voice's .onnx file, with its .onnx.json beside it")
ap.add_argument("--name", default="english", help="the voice's name, for its file and namespace: english, the one")
ap.add_argument("--out", help="default src/voice_<name>.inc")
ap.add_argument("--drive", type=float, default=DRIVE, help="how hard the words are squeezed: louder, up to harsh")
ap.add_argument("--rate", type=int, default=16000, help="the sample rate the words are stored at")
args = ap.parse_args()
if not args.out:
    args.out = os.path.join(os.path.dirname(__file__), "..", "src", f"voice_{args.name}.inc")


# Band-limited resampling: a Kaiser-windowed sinc, cut off a little under the
# lower rate's Nyquist, evaluated at each output sample's place in the input.
def resample(samples, rate_in, rate_out, half_taps=16, beta=8.0):
    if rate_in == rate_out:
        return list(samples)
    step = rate_in / rate_out
    cutoff = min(1.0, rate_out / rate_in) * 0.95

    def i0(x):
        total, term, k = 1.0, 1.0, 1
        while term > 1e-10 * total:
            term *= (x / (2 * k)) ** 2
            total += term
            k += 1
        return total

    norm = i0(beta)
    n = len(samples)
    out = []
    for j in range(int(n / step)):
        t = j * step
        c = int(t)
        acc = 0.0
        for k in range(max(0, c - half_taps + 1), min(n, c + half_taps + 1)):
            x = t - k
            w = i0(beta * math.sqrt(max(0.0, 1 - (x / half_taps) ** 2))) / norm
            arg = math.pi * x * cutoff
            acc += samples[k] * cutoff * (math.sin(arg) / arg if arg else 1.0) * w
        out.append(int(round(max(-32768, min(32767, acc)))))
    return out


def speak(text):
    out = subprocess.run([args.piper, "--model", args.model, "--output_file", "/dev/stdout"], input=text.encode(),
                         capture_output=True, check=True).stdout
    w = wave.open(io.BytesIO(out))
    if w.getnchannels() != 1 or w.getsampwidth() != 2:
        sys.exit("expected 16-bit mono from piper")
    samples = array.array("h", w.readframes(w.getnframes()))
    if sys.byteorder != "little":
        samples.byteswap()
    return args.rate, resample(samples, w.getframerate(), args.rate)


def trim_and_level(rate, samples):
    peak = max(abs(s) for s in samples) or 1
    loud = [i for i, s in enumerate(samples) if abs(s) > peak * SILENCE]
    margin = rate * MARGIN_MS // 1000
    start = max(0, loud[0] - margin)
    end = min(len(samples), loud[-1] + margin)
    if args.drive <= 1.0:
        gain = PEAK * 32767 / peak
        return [max(-32768, min(32767, int(round(s * gain)))) for s in samples[start:end]]
    # tanh: straight through near silence, rounding off smoothly towards the
    # peak - louder without the buzz of hard clipping.
    scale = PEAK * 32767 / math.tanh(args.drive)
    return [int(round(math.tanh(s / peak * args.drive) * scale)) for s in samples[start:end]]


# Every type name the firmware might say has a maker here.
db = open(os.path.join(os.path.dirname(__file__), "..", "src", "aircraft_db.cpp")).read()
types = db[db.index("TypeEntry TYPES[]"):db.index("AirlineEntry AIRLINES[]")]
type_names = re.findall(r'\{"[^"]+",\s*"([^"]+)"\}', types)
orphans = sorted({n for n in type_names if not any(n == m or n.startswith(m + " ") for m, _ in MAKERS)})
if orphans:
    sys.exit("no maker in MAKERS for: " + ", ".join(orphans))
# And every airline name, once - two prefixes can share one.
airlines_src = db[db.index("AirlineEntry AIRLINES[]"):]
airlines_src = airlines_src[:airlines_src.index("};")]
AIRLINES = list(dict.fromkeys(re.findall(r'\{"[A-Z0-9]{3}",\s*"([^"]+)"', airlines_src)))

# Spoken as one of a list, with a comma rather than a full stop, so each
# doesn't fall away as if the callsign had ended.
WORDS += [(f"LETTER_{chr(65 + i)}", w + ",") for i, w in enumerate(LETTERS)]
WORDS += [(f"DIGIT_{i}", w + ",") for i, w in enumerate(DIGITS)]
WORDS += [(f"MAKER_{i}", spoken + ",") for i, (_, spoken) in enumerate(MAKERS)]
WORDS += [(f"VARIANT_{name}", spoken + ",") for name, spoken in VARIANTS]
WORDS += [(f"AIRLINE_{i}", AIRLINE_SAID.get(name, name) + ",") for i, name in enumerate(AIRLINES)]
WORDS += PHRASES + READ + NAVAID_TYPES + ATC_USES
WORDS = [(name, RESPELT.get(args.name, {}).get(text, text)) for name, text in WORDS]

sys.path.insert(0, sys_path)
import ima_adpcm  # noqa: E402

lines = [
    "// Generated by tools/gen_voice.py from " + os.path.basename(args.model) + " - do not edit.",
    f"// The spoken words, as 4-bit IMA ADPCM - see tools/ima_adpcm.py - squeezed with a drive of {args.drive:g}.",
    f"namespace voice_{args.name} {{",
]
rate = None
spoken = []
for name, text in WORDS:
    r, samples = speak(text)
    if rate is not None and r != rate:
        sys.exit("piper changed its rate between words")
    rate = r
    spoken.append((name, text, trim_and_level(r, samples)))
    print(f"{name}: {len(spoken[-1][2]) / r:.2f}s")

codes = ima_adpcm.encode_search([s for _, _, s in spoken])
counts = {}
total = 0
for (name, text, samples), data in zip(spoken, codes):
    counts[name] = len(samples)
    total += len(data)
    lines.append(f"const uint8_t {name}[] = {{  // \"{text}\", {len(samples) / rate:.2f}s, {len(samples)} samples")
    for i in range(0, len(data), 32):
        lines.append("    " + ",".join(str(b) for b in data[i:i + 32]) + ",")
    lines.append("};")


def clip(name):
    return f"{{{name}, {counts[name]}}}"


# The makers, by the name a type name begins with, longest first; and the
# airlines, by their name in the airline table.
lines.append("const NamedClip MAKERS[] = {")
for i, (name, _) in enumerate(MAKERS):
    lines.append(f'    {{"{name}", {clip(f"MAKER_{i}")}}},')
lines.append("};")
lines.append("const NamedClip AIRLINES[] = {")
for i, name in enumerate(AIRLINES):
    lines.append(f'    {{"{name}", {clip(f"AIRLINE_{i}")}}},')
lines.append("};")
# All of it, in the order src/sound.cpp's Voice has it.
lines.append("const Voice VOICE = {")
lines.append(f"    {rate},")
lines.append("    " + ", ".join(clip(w) for w in ("EMERGENCY", "MILITARY", "RARE", "WATCHLIST")) + ",")
lines.append("    {" + ", ".join(clip(f"LETTER_{chr(65 + i)}") for i in range(26)) + "},")
lines.append("    {" + ", ".join(clip(f"DIGIT_{i}") for i in range(10)) + "},")
lines.append("    MAKERS, sizeof(MAKERS) / sizeof(MAKERS[0]),")
lines.append("    " + ", ".join(clip(f"VARIANT_{v}") for v, _ in VARIANTS) + ",")
lines.append("    AIRLINES, sizeof(AIRLINES) / sizeof(AIRLINES[0]),")
lines.append("    " + ", ".join(clip(w) for w, _ in PHRASES) + ",")
lines.append("    " + ", ".join(clip(w) for w, _ in READ) + ",")
lines.append("    {" + ", ".join(clip(w) for w, _ in NAVAID_TYPES) + "},")
lines.append("    {" + ", ".join(clip(w) for w, _ in ATC_USES) + "},")
lines.append("};")
lines.append(f"}}  // namespace voice_{args.name}")
print(f"{len(type_names)} type names, every one with a maker; {len(AIRLINES)} airlines")
with open(args.out, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {args.out}: {total // 1024}KB of IMA ADPCM, at {rate}Hz")
