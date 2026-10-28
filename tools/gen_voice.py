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

Piper says a word a little differently each time, and the words were chosen
by ear, so a word or two can be said again on their own, the rest left as
they are:

    --only NAV_VOR,MAKER_22    say just these again, by their names below, and
                               put them in src/voice_english.inc in place of
                               what was there - or, new to it, in with the
                               rest, a type said whole in the table of them -
                               every other clip as it was
    --say 'MAKER_22=Embrayer,' say one as this rather than as the tables below
                               have it, to try a spelling by ear
    --wav DIR                  write each one said as DIR/NAME.wav, as the
                               Tab5 plays it - to IMA ADPCM and back, at
                               16kHz - with Piper's own take of it beside it,
                               NAME.take.wav, and what it said, NAME.txt; the
                               .inc is left alone
    --takes DIR                make each from its take in DIR, written there by
                               --wav, rather than saying it afresh - the one
                               heard is the one that goes in
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
import tempfile
import wave

# The name each clip has in the firmware, and what is said.
WORDS = [
    ("EMERGENCY", "Emergency."),
    ("MILITARY", "Military."),
    ("RARE", "Rare aircraft."),
    ("WATCHLIST", "Watchlist."),
]
# A callsign's letters and digits, as a pilot or controller reads them out:
# the ICAO phonetic alphabet - spelt as Piper says them right, Romeo as
# RESPELT below has it - and niner.
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
    ("Cessna", "Cessna"), ("Cirrus", "Cirrus"), ("Dassault", "Dassault"), ("Embraer", "Embrayer"),
    ("Eurocopter", "Eurocopter"), ("Eurofighter", "Eurofighter"), ("Fairchild", "Fairchild"),
    ("Grumman", "Grumman"), ("Gulfstream", "Gulfstream"), ("Hawker", "Hawker"), ("Kawasaki", "Kawasaki"),
    ("Learjet", "Learjet"), ("Leonardo", "Leonardo"), ("Lockheed", "Lockheed"), ("Mil", "Mil"),
    ("NHIndustries", "N H Industries"), ("Northrop", "Northrop"), ("Panavia", "Panavia"), ("Piaggio", "Piaggio"),
    ("Piper", "Piper"), ("Robinson", "Robinson"), ("Rockwell", "Rockwell"), ("SEPECAT", "Sepecat"),
    ("Sikorsky", "Sikorsky"), ("Socata", "Socata"), ("Swearingen", "Swearingen"), ("Transall", "Transall"),
]
# Variants worth hearing, said after the model where a name has them.
VARIANTS = [("NEO", "neo"), ("MAX", "max"), ("DREAMLINER", "Dreamliner")]
# Type names said whole, in one breath: put together a clip at a time,
# "Embrayer... Echo, One, Niner, Five", a pause after each, the Embraers'
# were hard to make out. Each by its name in src/aircraft_db.cpp, which
# src/sound.cpp's sayType() looks for whole before it looks for a maker; the
# name of its clip; and what is said - the model in the radio's alphabet, as a
# controller reads it out, in the words a callsign is spelt in. The comma
# after the maker is a short break, and keeps espeak-ng from running an r into
# the Echo, "Embrayer-r-Echo".
TYPE_PHRASES = [
    ("Embraer E170", "TYPE_E170", "Embrayer, Echo One Seven Zero."),
    ("Embraer E175", "TYPE_E175", "Embrayer, Echo One Seven Five."),
    ("Embraer E190", "TYPE_E190", "Embrayer, Echo One Niner Zero."),
    ("Embraer E195", "TYPE_E195", "Embrayer, Echo One Niner Five."),
    ("Embraer E190-E2", "TYPE_E190_E2", "Embrayer, Echo One Niner Zero, Echo Two."),
    ("Embraer E195-E2", "TYPE_E195_E2", "Embrayer, Echo One Niner Five, Echo Two."),
    ("Embraer ERJ-135", "TYPE_ERJ_135", "Embrayer, Echo Romeo Juliet One Three Five."),
    ("Embraer ERJ-145", "TYPE_ERJ_145", "Embrayer, Echo Romeo Juliet One Four Five."),
]
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
# A navaid's kind, in NavaidType's order, as a pilot says it: "vee oh AR, dee
# em EE" - every letter on its own but at a speaker's pace, each group's last
# letter the stressed one, as an initialism is said, and a short break between
# the groups. Given to Piper as phonemes - piper-tts 1.3 and later read [[...]]
# as them: written as letters, "V O R D M E", espeak-ng links them with
# glides, vˈiːʲ ˈəʊ, a "y" into the O and the M, and Piper ran them together;
# with a comma after each, "V, O, R", they were clear but slow, a pause after
# every letter.
NAVAID_TYPES = [("NAV_VOR", "[[vˌiː ˌəʊ ˈɑː,]]"), ("NAV_VOR_DME", "[[vˌiː ˌəʊ ˈɑː, dˌiː ˌɛm ˈiː,]]"),
                ("NAV_VORTAC", "Vortac,"), ("NAV_TACAN", "Tacan,"), ("NAV_DME", "[[dˌiː ˌɛm ˈiː,]]"),
                ("NAV_NDB_DME", "[[ˌɛn dˌiː bˈiː, dˌiː ˌɛm ˈiː,]]")]
# Who is on an airport's frequency, in AtcUse's order.
ATC_USES = [("ATC_TOWER", "Tower,"), ("ATC_GROUND", "Ground,"), ("ATC_DELIVERY", "Delivery,"),
            ("ATC_APPROACH", "Approach,"), ("ATC_DEPARTURE", "Departure,"), ("ATC_RADAR", "Radar,"),
            ("ATC_CONTROL", "Control,"), ("ATC_ATIS", "Atis,"), ("ATC_INFO", "Information,")]
# Words a voice says wrongly as written, and how to write them for it to say
# them right - found by ear, a few spellings tried for each, by the voice's
# --name. The English voice says every one right as it is written but Romeo:
# espeak-ng reads it ɹˈəʊmɪʲˌəʊ, a short i gliding into the last "oh", which
# Piper squashed to the glide alone in a third of its takes - the clip in
# until 1.5.1 among them, "Rome-yo" - and every plain spelling tried kept the
# glide. So it is given as phonemes, as "Rome" and then "ee, oh" with no
# glide, which was what the ear chose of a dozen ways tried - said a little
# slower and steadier than the rest (SAID_AS).
RESPELT = {"english": {"Romeo,": "[[ɹˈəʊm iː ˌəʊ,]]"}}
# Piper's settings for a word that needs other than its defaults, by its text.
SAID_AS = {"[[ɹˈəʊm iː ˌəʊ,]]": ["--length-scale", "1.15", "--noise-scale", "0.5", "--noise-w-scale", "0.5"]}
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
ap.add_argument("--only", help="say just these clips again, by name - NAV_VOR,MAKER_22 - and put them in --out in "
                               "place of what was there, or in with the rest if new to it, every other clip left as "
                               "it is")
ap.add_argument("--say", action="append", default=[], metavar="NAME=TEXT",
                help="say a clip as TEXT rather than as the tables have it, to try a spelling by ear")
ap.add_argument("--wav", metavar="DIR", help="write each clip said as DIR/NAME.wav, as the Tab5 plays it, with "
                                             "Piper's take of it beside it, and leave --out alone")
ap.add_argument("--takes", action="append", default=[], metavar="DIR",
                help="make a clip from its take in DIR, written there by --wav, rather than saying it afresh")
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


# Piper's take of `text`, as the WAV it writes. Into a file rather than down a
# pipe: given more than one sentence - "V. O. R." - it goes back to the
# header to say how long the WAV is, which it can't do on a pipe.
def speak(text):
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, "take.wav")
        subprocess.run([args.piper, "--model", args.model, "--output_file", path] + SAID_AS.get(text, []),
                       input=text.encode(), capture_output=True, check=True)
        with open(path, "rb") as f:
            return f.read()


# A take's samples, at the rate they are stored at.
def heard(take):
    w = wave.open(io.BytesIO(take))
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
# And every type said whole is one of them, spelt as there.
unknown = sorted({name for name, _, _ in TYPE_PHRASES} - set(type_names))
if unknown:
    sys.exit("no type in src/aircraft_db.cpp called: " + ", ".join(unknown))
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
WORDS += [(name, text) for _, name, text in TYPE_PHRASES]
WORDS = [(name, RESPELT.get(args.name, {}).get(text, text)) for name, text in WORDS]
# A clip said otherwise for now - see --say - and just the ones asked for -
# see --only.
if any("=" not in s for s in args.say):
    sys.exit("--say NAME=TEXT")
said = dict(s.split("=", 1) for s in args.say)
only = args.only.split(",") if args.only else [name for name, _ in WORDS]
unknown = sorted((set(said) | set(only)) - {name for name, _ in WORDS})
if unknown:
    sys.exit("no clip called " + ", ".join(unknown))
WORDS = [(name, said.get(name, text)) for name, text in WORDS]

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
    if name not in only:
        continue
    # Its take from --takes, if it has one there, of the same words; else
    # said afresh.
    take, took = None, ""
    for folder in args.takes:
        path = os.path.join(folder, name + ".take.wav")
        if os.path.exists(path):
            was = open(os.path.join(folder, name + ".txt"), encoding="utf-8").read()
            if was != text:
                sys.exit(f"{path} says \"{was}\", not \"{text}\" - --say it, or change it in the tables")
            take, took = open(path, "rb").read(), f", from {path}"
            break
    if take is None:
        take = speak(text)
    r, samples = heard(take)
    if rate is not None and r != rate:
        sys.exit("piper changed its rate between words")
    rate = r
    spoken.append((name, text, take, trim_and_level(r, samples)))
    print(f"{name}: {len(spoken[-1][3]) / r:.2f}s{took}")

codes = ima_adpcm.encode_search([s for _, _, _, s in spoken])


# A clip's codes as the .inc has them, 32 bytes a line - the .inc in UTF-8,
# as the phonemes some clips are said from are in their comments.
def clip_lines(name, text, samples, data):
    out = [f"const uint8_t {name}[] = {{  // \"{text}\", {len(samples) / rate:.2f}s, {len(samples)} samples"]
    for i in range(0, len(data), 32):
        out.append("    " + ",".join(str(b) for b in data[i:i + 32]) + ",")
    return out + ["};"]


# The types said whole, by their whole name, as the .inc has them: those of
# TYPE_PHRASES with a clip in `counts`, its count of samples.
def types_table(counts):
    out = ["const NamedClip TYPES[] = {"]
    for name, clip_name, _ in TYPE_PHRASES:
        if clip_name in counts:
            out.append(f'    {{"{name}", {{{clip_name}, {counts[clip_name]}}}}},')
    return out + ["};"]


if args.wav:
    # Each as the Tab5 plays it - its codes turned back into samples as
    # src/sound.cpp does - with the take it was made from, for --takes.
    os.makedirs(args.wav, exist_ok=True)
    for (name, text, take, samples), data in zip(spoken, codes):
        played = array.array("h", ima_adpcm.decode(data, len(samples)))
        if sys.byteorder != "little":
            played.byteswap()
        with wave.open(os.path.join(args.wav, name + ".wav"), "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(played.tobytes())
        with open(os.path.join(args.wav, name + ".take.wav"), "wb") as f:
            f.write(take)
        with open(os.path.join(args.wav, name + ".txt"), "w", encoding="utf-8") as f:
            f.write(text)
    print(f"wrote {len(spoken)} clips to {args.wav} as the Tab5 plays them, at {rate}Hz; {args.out} left alone")
    sys.exit()
if args.only:
    # Just these: each one's codes, and its count wherever the tables give it,
    # in place of what was there - every other clip as it was, byte for byte.
    # One not there yet goes in after the one before it in the order above,
    # where a whole .inc would have it.
    inc = open(args.out, encoding="utf-8").read()
    stored = re.search(r"const Voice VOICE = \{\n    (\d+),", inc)
    if not stored or int(stored.group(1)) != rate:
        sys.exit(f"{args.out} isn't at {rate}Hz")
    order = [name for name, _ in WORDS]
    again, added = [], []
    for (name, text, _, samples), data in zip(spoken, codes):
        block = "\n".join(clip_lines(name, text, samples, data))
        inc, found = re.subn(r"const uint8_t " + name + r"\[\] = \{.*?\n\};", lambda m: block, inc, count=1,
                             flags=re.S)
        if found:
            again.append(name)
        else:
            before = [b for b in order[:order.index(name)] if f"const uint8_t {b}[] = {{" in inc]
            if before:
                at = re.search(r"const uint8_t " + before[-1] + r"\[\] = \{.*?\n\};\n", inc, re.S).end()
            else:
                at = inc.index("\n", inc.index(f"namespace voice_{args.name} {{")) + 1
            inc = inc[:at] + block + "\n" + inc[at:]
            added.append(name)
        inc = re.sub(r"\{" + name + r", \d+\}", "{" + name + f", {len(samples)}}}", inc)
    # A type said whole: the table of them made again from the clips there
    # now - put in after the makers', and in the voice after them, the first
    # time.
    if any(name in {c for _, c, _ in TYPE_PHRASES} for name, _, _, _ in spoken):
        counts = {m.group(1): int(m.group(2))
                  for m in re.finditer(r"const uint8_t (\w+)\[\] = \{  // .*, (\d+) samples\n", inc)}
        table = "\n".join(types_table(counts)) + "\n"
        inc, found = re.subn(r"const NamedClip TYPES\[\] = \{\n.*?\n\};\n", lambda m: table, inc, count=1, flags=re.S)
        if not found:
            makers = "    MAKERS, sizeof(MAKERS) / sizeof(MAKERS[0]),\n"
            if "const NamedClip AIRLINES[] = {" not in inc or makers not in inc:
                sys.exit(f"no makers' and airlines' tables in {args.out} to put the types' with")
            inc = inc.replace("const NamedClip AIRLINES[] = {", table + "const NamedClip AIRLINES[] = {", 1)
            inc = inc.replace(makers, makers + "    TYPES, sizeof(TYPES) / sizeof(TYPES[0]),\n", 1)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(inc)
    done = ([", ".join(again) + " said again"] if again else []) + ([", ".join(added) + " put in"] if added else [])
    print(f"wrote {args.out}: " + "; ".join(done) + "; the rest as they were")
    sys.exit()
counts = {}
total = 0
for (name, text, _, samples), data in zip(spoken, codes):
    counts[name] = len(samples)
    total += len(data)
    lines += clip_lines(name, text, samples, data)


def clip(name):
    return f"{{{name}, {counts[name]}}}"


# The makers, by the name a type name begins with, longest first; the types
# said whole; and the airlines, by their name in the airline table.
lines.append("const NamedClip MAKERS[] = {")
for i, (name, _) in enumerate(MAKERS):
    lines.append(f'    {{"{name}", {clip(f"MAKER_{i}")}}},')
lines.append("};")
lines += types_table(counts)
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
lines.append("    TYPES, sizeof(TYPES) / sizeof(TYPES[0]),")
lines.append("    " + ", ".join(clip(f"VARIANT_{v}") for v, _ in VARIANTS) + ",")
lines.append("    AIRLINES, sizeof(AIRLINES) / sizeof(AIRLINES[0]),")
lines.append("    " + ", ".join(clip(w) for w, _ in PHRASES) + ",")
lines.append("    " + ", ".join(clip(w) for w, _ in READ) + ",")
lines.append("    {" + ", ".join(clip(w) for w, _ in NAVAID_TYPES) + "},")
lines.append("    {" + ", ".join(clip(w) for w, _ in ATC_USES) + "},")
lines.append("};")
lines.append(f"}}  // namespace voice_{args.name}")
print(f"{len(type_names)} type names, every one with a maker; {len(AIRLINES)} airlines")
with open(args.out, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"wrote {args.out}: {total // 1024}KB of IMA ADPCM, at {rate}Hz")
