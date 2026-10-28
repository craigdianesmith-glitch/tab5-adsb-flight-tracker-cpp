#!/usr/bin/env python3
"""Soak-test the 3D view on the Tab5: made-up flights followed in 3D, for as
long as asked, with every camera, zoom, sweeps of the orbit camera, taps for
an aircraft's details and Back, and the view closed and opened again - and
the firmware's log read all the while, for how it fared.

    python tools/soak3d.py [--minutes 60] [--scenarios 0123456789ab] [--sun real|day|dusk|night] [--traffic]
                           [--speech] [--port /dev/ttyACM0]

Needs firmware built with the 3D bench in it, which makes up the flights and
takes the commands (see main.cpp's "##k" and the rest):

    PLATFORMIO_BUILD_FLAGS=-DBENCH3D pio run -e tab5 -t upload

The flights, by number - and from 10 by letter - as main.cpp's startSoak()
has them:

    0  a Typhoon circling RAF Lossiemouth, 1,800ft - a military airfield
    1  an F-35 high over the North Sea - the country view, high
    2  a Hawk low over the Lake District - the country view, low, with lakes
    3  a Chinook circling RAF Odiham, 1,000ft
    4  a police EC135 hovering over Glasgow
    5  an air ambulance H145 circling Edinburgh, 800ft
    6  an A400M orbiting high over Perthshire
    7  an A320 landing on 27L at Heathrow, then parking
    8  a 737-800 taking off from 24 at Edinburgh
    9  a C-17 landing at RAF Brize Norton, then parking
    a  an air ambulance H145 setting down at Glasgow and off again - its
       pressure altitude 270ft low, its GNSS height right, never saying it
       is on the ground: the spot camera, it set down, lifting off
    b  a coastguard AW139 setting down at Edinburgh - no GNSS height, its
       pressure altitude put right by the day's pressure as the aircraft
       about have it, 40ft out; saying when it is down - then its
       transponder off: it settling onto the ground, and kept there
    c  an Emirates A380 landing on 27L at Heathrow - for pictures, not in
       the soak's default round
    d  a British Airways 777 taking off from 27R at Heathrow - likewise
    e  a Typhoon running in at 400kt and 1,000ft to break into the circuit at
       Lossiemouth, then landing on 23 - one landing begun, not two: the run
       in is no landing
    f  a Cessna joining overhead at Glasgow at 2,000ft, then landing on 23 -
       likewise, the overhead no landing
    g  an A320 landing on 23 at Glasgow on a day of low pressure - its
       pressure altitude out by the day's pressure, put right by it: its nose
       up as it touches down
    h  an A320 landing on 23 at Lossiemouth on a day of high pressure - its
       pressure altitude 500ft under its height, under nought on short final;
       its GNSS height right
    i  an alert for an A320 landing at Glasgow, auto-followed into 3D: no
       alert banner left round the view (##I looks)
    j  a U-2 circling RAF Fairford at 65,000ft and 410kt - twice the
       cruise's height, the horizon 500km off
    k  a Typhoon at 10,000ft and 480kt circling the Cheviots - the hills
       seen from above, laid across the land no longer; its fly past fast,
       its sound arriving all at once as it passes
    l  an E190 taking off from 23 at Glasgow - its nose rotated for on the
       runway, its APU off as it lines up, the hills beyond
    m  a 767 at FL300 circling the Southern Uplands - its trails whole and
       grey, a widebody's fly past
    q  a Typhoon taking off from 23 at Lossiemouth - full dry power on its
       brakes as it waits lined up, its reheat lit as it rolls, held in its
       climb and cancelled at 3,000ft over the field
    R  not a flight: the newest recording that follows an aircraft, replayed
       in 3D - played, 4x, a minute back and on, its bar, another aircraft
       picked, Back and 3D again

Each touchdown in 3D is said with its nose's angle, and each landing begun
by its runway simulation; the summary counts them, and the banner's pixels.

Everything the firmware says goes to soak-<when>/log.txt, with the time; at
the end - or on Ctrl-C - each flight's frame rates, the most the preparing
took, the longest core 0 went without its idle task (the task watchdog
resets the board at five seconds), the least internal RAM, slow frames,
errors and resets, to the screen and soak-<when>/summary.md.

The Tab5 must be idle - nothing followed, nothing recording - and this takes
it over for the while. Needs pyserial; close any serial monitor first.
"""
import argparse
import re
import sys
import threading
import time
from pathlib import Path

import serial

ap = argparse.ArgumentParser()
ap.add_argument("--minutes", type=float, default=60)
ap.add_argument("--scenarios", default="0123456789abefghijklmnoqR")
ap.add_argument("--port", default="/dev/ttyACM0")
ap.add_argument("--out", default=time.strftime("soak-%Y%m%d-%H%M"))
ap.add_argument("--sun", choices=["real", "day", "dusk", "night"], default="real",
                help="the 3D view's sun: as it is, or held at a time of day, for runs to compare like with like")
ap.add_argument("--speech", action="store_true",
                help="speech all the while: an alert, a runway announcement or a card read out every 25 seconds or "
                     "so, and every fourth time a whole airport's worth at Glasgow, Heathrow or JFK - for the words "
                     "to be seen not to stall the 3D view, nor it them")
ap.add_argument("--traffic", action="store_true",
                help="made-up traffic about each airfield - parked on its stands, taxiing its taxiways - as the "
                     "polls would bring it, for the traffic to be tried with the network down")
args = ap.parse_args()

out = Path(args.out)
out.mkdir(parents=True, exist_ok=True)
log = open(out / "log.txt", "a", buffering=1)

# DTR and RTS are left as the kernel raises them on open: setting both low is
# the ESP32's reset signal.
port = serial.Serial(args.port, 115200, timeout=0.2)

lines = []  # (time, text) of everything said, for the summary
lock = threading.Lock()
stop = threading.Event()


def reader():
    buffer = b""
    while not stop.is_set():
        try:
            chunk = port.read(8192)
        except serial.SerialException as e:
            log.write(f"{time.strftime('%H:%M:%S')} [soak3d] port error: {e}\n")
            time.sleep(1)
            continue
        if not chunk:
            continue
        buffer += chunk
        while b"\n" in buffer:
            raw, buffer = buffer.split(b"\n", 1)
            text = raw.decode("utf-8", "replace").rstrip("\r")
            now = time.time()
            log.write(f"{time.strftime('%H:%M:%S', time.localtime(now))} {text}\n")
            with lock:
                lines.append((now, text))


threading.Thread(target=reader, daemon=True).start()


write_lock = threading.Lock()


def send(command):
    with write_lock:  # the speech thread sends too: one command at a time
        port.write(("##" + command).encode())
    log.write(f"{time.strftime('%H:%M:%S')} [soak3d] >> ##{command}\n")


# Speech, with --speech: the alerts, the runway announcements and the cards,
# short ones every 25 seconds or so and every fourth a long one - an alert, a
# landing and an aircraft's details at Glasgow, Heathrow or JFK.
SHORT_SPEECH = ["E", "W", "R", "M", "T", "L", "N", "A"]
LONG_SPEECH = ["C", "G", "U"]


def speaker():
    n = 0
    while not stop.is_set():
        wait(25)
        if stop.is_set():
            break
        send(LONG_SPEECH[(n // 4) % len(LONG_SPEECH)] if n % 4 == 3 else SHORT_SPEECH[n % len(SHORT_SPEECH)])
        n += 1


def wait(seconds):
    end = time.time() + seconds
    while time.time() < end and not stop.is_set():
        time.sleep(0.2)


# What a finger does to it, in order, after the flight is up: each command and
# the seconds to leave it.
STEPS = [
    ("Kh", 12),  # CHASE
    ("Ko", 1), ("o", 46),  # ORBIT, swept every way
    ("Kt", 10),  # TOWER - greyed high up
    ("K+", 2), ("K+", 6), ("K-", 2), ("K-", 2), ("K-", 6), ("K+", 1),
    ("Kc", 15),  # CINEMA
    ("Ks", 4), ("Ks", 1),  # SOUND off, the engines fading out, and on again
    ("Kf", 30), ("K+", 10), ("K-", 1),  # FLY PAST, passes nearer, and back
    ("d", 5), ("B", 10),  # an aircraft tapped for its details, and Back into 3D
    ("V", 4), ("2", 12),  # Back to the radar, and its 3D button
    ("Ko", 1), ("o", 46),
    ("Kc", 20),
]
# Every other round, at the end: FOLLOW on another aircraft's details, into
# 3D following it.
FOLLOW_STEPS = [("d", 5), ("F", 20)]
# A circuit takes longer than the steps to land: each kept up at least this
# long, for its landing to be seen.
MIN_SECONDS = {"e": 270, "f": 420}
# The replay in 3D, R: the newest recording that follows an aircraft opened,
# its 3D, and every control - then another aircraft picked, Back, and in again.
REPLAY_STEPS = [
    ("y", 6), ("j3", 15),
    ("jp", 30), ("js", 30), ("jb", 10), ("jf", 10), ("jm", 20),
    ("Kh", 10), ("Ko", 1), ("o", 30), ("Kc", 15), ("Kf", 20),
    ("d", 5), ("F", 15),
    ("d", 5), ("B", 10),
    ("jp", 8), ("jB", 5), ("j3", 15), ("jB", 5), ("jx", 4),
]

runs = []  # (scenario, start, end)
started = time.time()
deadline = started + args.minutes * 60
round_no = 0
try:
    print(f"soak: {args.minutes:g} minutes of {args.scenarios}, log in {out}/log.txt", flush=True)
    send("Z" + str(["real", "day", "dusk", "night"].index(args.sun)))
    send("r1" if args.traffic else "r0")
    if args.speech:
        threading.Thread(target=speaker, daemon=True).start()
    while time.time() < deadline and not stop.is_set():
        for n in args.scenarios:
            if time.time() >= deadline:
                break
            t0 = time.time()
            print(f"{time.strftime('%H:%M:%S')} scenario {n}", flush=True)
            if n == "R":
                send("x")
                wait(2)
                for command, seconds in REPLAY_STEPS:
                    send(command)
                    wait(seconds)
                runs.append((n, t0, time.time()))
                continue
            send("k" + n)
            if n == "i":
                wait(8)  # the alert, auto-follow, and the 3D view opened
                send("I")
                wait(17)
            else:
                wait(25)  # CINEMA, as it opens
            steps = STEPS + (FOLLOW_STEPS if round_no % 2 == 1 else [])
            for command, seconds in steps:
                send(command)
                wait(seconds)
            wait(MIN_SECONDS.get(n, 0) - (time.time() - t0))
            send("x")
            wait(4)
            runs.append((n, t0, time.time()))
        round_no += 1
except KeyboardInterrupt:
    print("stopped", flush=True)
finally:
    send("x")
    send("r0")
    time.sleep(2)
    stop.set()
    time.sleep(0.5)
    port.close()

# --- the summary ----------------------------------------------------------------------

FPS = re.compile(r"\[3d\] ([\d.]+) fps; preparing ([\d.]+)ms, (?:drawing [\d.]+ms, )?waiting for the drawing ([\d.]+)ms; "
                 r".*internal RAM "
                 r"(\d+)KB free(?:; core 0 idle at most (\d+)ms apart)?")
OPEN = re.compile(r"\[3d\] open, (\d+)-row bands: internal RAM (\d+)KB free, the largest block (\d+)KB")
TROUBLE = ("[hang]", "[screen]", "no room for the bands", "Guru", "panic", "abort", "Backtrace", "assert",
           "banner left behind", "no recording that follows", "isn't in the recording")
TOUCHDOWN = re.compile(r"\[3d\] (\S+) down on (\S+) at (\S*), its nose ([\d.-]+) degrees up")
LIFTOFF = re.compile(r"\[3d\] (\S+) off (\S+) at (\S*) at (\d+) kt, its nose ([\d.-]+) degrees up")
ENGINES = re.compile(r"\[3d\] (\S+): engines (running|shut down), APU (on|off)")
LANDING = re.compile(r"\[follow\] (\S+): [\w ]+ -> landing")
BANNER = re.compile(r"\[soak\] banner check on (.*?): (\d+) of its pixels")
LATE = re.compile(r"\[sound\] word \d+ of \d+ (\d+)ms late")


def summarise(n, t0, t1):
    with lock:
        said = [text for when, text in lines if t0 <= when < t1]
    fps, prep, gaps, ram, opens, slow, trouble, resets, taps = [], [], [], [], [], [], [], 0, []
    said_count, late, dropped = 0, [], []
    noses, landings, banner, lifts, engines = [], 0, None, [], []
    for text in said:
        m = TOUCHDOWN.search(text)
        if m:
            noses.append(float(m.group(4)))
        m = LIFTOFF.search(text)
        if m:
            lifts.append((int(m.group(4)), float(m.group(5))))
        m = ENGINES.search(text)
        if m:
            engines.append(("off" if m.group(2) == "shut down" else "on") + "/" + m.group(3))
        if LANDING.search(text):
            landings += 1
        m = BANNER.search(text)
        if m:
            banner = int(m.group(2))
        m = FPS.search(text)
        if m:
            fps.append(float(m.group(1)))
            prep.append(float(m.group(2)))
            ram.append(int(m.group(4)))
            if m.group(5):
                gaps.append(int(m.group(5)))
        m = OPEN.search(text)
        if m:
            opens.append((int(m.group(1)), int(m.group(2)), int(m.group(3))))
        if "[3d] slow frame" in text:
            slow.append(text)
        if any(k in text for k in TROUBLE):
            trouble.append(text)
        if "ESP-ROM" in text or text.startswith("rst:"):
            resets += 1
        if "nothing to tap" in text:
            taps.append(text)
        if text.startswith("[sound] ") and (": " in text and " spoken" in text):
            said_count += 1
        m = LATE.search(text)
        if m:
            late.append(int(m.group(1)))
        if text.startswith("[sound] ") and ("dropped" in text or "not sounded" in text or "failed" in text):
            dropped.append(text)
    return {
        "n": n,
        "minutes": (t1 - t0) / 60,
        "fps": (min(fps), sum(fps) / len(fps)) if fps else None,
        "prep": max(prep) if prep else None,
        "gap": max(gaps) if gaps else None,
        "ram": min(ram) if ram else None,
        "opens": opens,
        "slow": slow,
        "trouble": trouble,
        "resets": resets,
        "untapped": len(taps),
        "said": said_count,
        "late": late,
        "dropped": dropped,
        "noses": noses,
        "lifts": lifts,
        "engines": engines,
        "landings": landings,
        "banner": banner,
    }


rows = [summarise(*r) for r in runs]
lines_out = [
    f"# 3D soak, {time.strftime('%Y-%m-%d %H:%M', time.localtime(started))}, "
    f"{(time.time() - started) / 60:.0f} minutes",
    "",
    "| # | min | fps lowest / average | preparing, most | core 0 idle gap, most | internal RAM, least | "
    "3D opens (bands, KB free, largest) | slow frames | resets | trouble | said | words late, most | "
    "landings begun | touchdowns, nose | lift-offs, kt and nose | engines/APU | banner pixels |",
    "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
]
for r in rows:
    fps = f"{r['fps'][0]:.1f} / {r['fps'][1]:.1f}" if r["fps"] else "-"
    prep = f"{r['prep']:.1f}ms" if r["prep"] is not None else "-"
    gap = f"{r['gap']}ms" if r["gap"] is not None else "-"
    ram = f"{r['ram']}KB" if r["ram"] is not None else "-"
    opens = ", ".join(f"{b}r {f}/{l}" for b, f, l in r["opens"]) or "-"
    late = f"{len(r['late'])}, {max(r['late'])}ms" if r["late"] else "0"
    noses = ", ".join(f"{x:.1f}" for x in r["noses"]) or "-"
    lifts = ", ".join(f"{kt} {nose:.1f}" for kt, nose in r["lifts"]) or "-"
    engines = " ".join(r["engines"]) or "-"
    banner = "-" if r["banner"] is None else str(r["banner"])
    lines_out.append(f"| {r['n']} | {r['minutes']:.1f} | {fps} | {prep} | {gap} | {ram} | {opens} | "
                     f"{len(r['slow'])} | {r['resets']} | {len(r['trouble'])} | {r['said']} | {late} | "
                     f"{r['landings']} | {noses} | {lifts} | {engines} | {banner} |")
details = [r for r in rows if r["slow"] or r["trouble"] or r["resets"] or r["dropped"]]
if details:
    lines_out += ["", "## What went wrong", ""]
    for r in details:
        for text in (r["trouble"] + r["slow"] + r["dropped"])[:12]:
            lines_out.append(f"- scenario {r['n']}: `{text}`")
summary = "\n".join(lines_out) + "\n"
(out / "summary.md").write_text(summary)
print(summary)
