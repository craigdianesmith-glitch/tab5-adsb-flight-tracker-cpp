#!/usr/bin/env python3
"""Generate the spoken alert words into src/voice_table.inc.

    python tools/gen_voice.py --piper PATH/TO/piper --model PATH/TO/voice.onnx

Speaks each word with Piper (https://github.com/rhasspy/piper), a free offline
neural text-to-speech engine - `pip install piper-tts` in a virtualenv, and a
voice from https://huggingface.co/rhasspy/piper-voices - trims the silence off
either end, brings it up to a common peak, and writes it as 16-bit samples at
the voice's own rate for the firmware to play from flash. The default words
are spoken by en_GB-alba-medium, a Scottish voice built from the University
of Edinburgh's CSTR data (CC BY 4.0).
"""
import argparse
import array
import io
import math
import os
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
ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "src", "voice_table.inc"))
ap.add_argument("--drive", type=float, default=DRIVE, help="how hard the words are squeezed: louder, up to harsh")
args = ap.parse_args()


def speak(text):
    out = subprocess.run([args.piper, "--model", args.model, "--output_file", "/dev/stdout"], input=text.encode(),
                         capture_output=True, check=True).stdout
    w = wave.open(io.BytesIO(out))
    if w.getnchannels() != 1 or w.getsampwidth() != 2:
        sys.exit("expected 16-bit mono from piper")
    samples = array.array("h", w.readframes(w.getnframes()))
    if sys.byteorder != "little":
        samples.byteswap()
    return w.getframerate(), samples


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

# Spoken as one of a list, with a comma rather than a full stop, so each
# doesn't fall away as if the callsign had ended.
WORDS += [(f"LETTER_{chr(65 + i)}", w + ",") for i, w in enumerate(LETTERS)]
WORDS += [(f"DIGIT_{i}", w + ",") for i, w in enumerate(DIGITS)]
WORDS += [(f"MAKER_{i}", spoken + ",") for i, (_, spoken) in enumerate(MAKERS)]
WORDS += [(f"VARIANT_{name}", spoken + ",") for name, spoken in VARIANTS]

lines = [
    "// Generated by tools/gen_voice.py from " + os.path.basename(args.model) + " - do not edit.",
    f"// The spoken alert words, as 16-bit mono samples, squeezed with a drive of {args.drive:g}.",
]
rate = None
total = 0
for name, text in WORDS:
    r, samples = speak(text)
    if rate is not None and r != rate:
        sys.exit("piper changed its rate between words")
    rate = r
    clip = trim_and_level(r, samples)
    total += len(clip) * 2
    lines.append(f"const int16_t VOICE_{name}[] = {{  // \"{text}\", {len(clip) / r:.2f}s")
    for i in range(0, len(clip), 16):
        lines.append("    " + ", ".join(str(s) for s in clip[i:i + 16]) + ",")
    lines.append("};")
    print(f"{name}: {len(clip) / r:.2f}s")
lines.insert(2, f"constexpr uint32_t VOICE_RATE = {rate};")
# And the letters and digits indexed, for a callsign to be spelt from.
for kind, count, first in (("LETTER", 26, 65), ("DIGIT", 10, 48)):
    names = [f"VOICE_{kind}_{chr(first + i) if kind == 'LETTER' else i}" for i in range(count)]
    lines.append(f"const int16_t *const VOICE_{kind}S[] = {{" + ", ".join(names) + "};")
    lines.append(f"const size_t VOICE_{kind}_LENS[] = {{" + ", ".join(f"sizeof({n}) / 2" for n in names) + "};")
# And the makers, by the name a type name begins with, longest first.
lines.append("struct VoiceMaker {\n    const char *name;\n    const int16_t *samples;\n    size_t count;\n};")
lines.append("const VoiceMaker VOICE_MAKERS[] = {")
for i, (name, _) in enumerate(MAKERS):
    lines.append(f'    {{"{name}", VOICE_MAKER_{i}, sizeof(VOICE_MAKER_{i}) / 2}},')
lines.append("};")
print(f"{len(type_names)} type names, every one with a maker")
with open(args.out, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {args.out}: {total // 1024}KB of samples at {rate}Hz")
