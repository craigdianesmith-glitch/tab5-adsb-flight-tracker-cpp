#!/usr/bin/env python3
"""Play the 3D view's landings on the Tab5, and print how fast it drew them.

    python tools/bench3d.py <command> [--port /dev/ttyACM0]

Needs firmware built with the 3D bench in it:

    PLATFORMIO_BUILD_FLAGS=-DBENCH3D pio run -e tab5 -t upload

The commands, as that firmware takes them over serial after "##":

    0   busy airports at 960x544, pipelined: Heathrow by day landing 27L,
        then Kennedy at dusk landing 22L - about two and a half minutes
    1   the director at Heathrow, as though live: a landing on 27L with its
        touchdown replayed in slow motion, then a 787 departing 27R
    4   the landing through the cinema's shots, full screen, by day
    5   the same at dusk
    6   the same at night
    9   the busiest half minute at four sizes of view
    3   single frames timed every way they could reach the screen
    7   a showcase frame left on the screen - the next of five each time -
        for tools/screenshot.py to take
    8   put back the screen that was up
    h   high up: a minute of the cruise over the Firth of Clyde, by day
    n   half a minute of it at night
    l   the traffic's labels stamped into a frame's bands, in C and with PIE,
        against how the bench wrote them before

It prints the firmware's report - frames a second, and where the time went -
and returns when the firmware says it has finished. Needs pyserial; close any
serial monitor first.
"""
import argparse
import sys
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("command", choices=list("013456789hnl"))
ap.add_argument("--port", default="/dev/ttyACM0")
args = ap.parse_args()

# DTR and RTS are left as the kernel raises them on open: setting both low is
# the ESP32's reset signal.
port = serial.Serial(args.port, 115200, timeout=0.5)
port.reset_input_buffer()
port.write(("##" + args.command).encode())

# Done when the firmware says so - every command ends "[3d] done" - or after
# ten minutes regardless.
deadline = time.time() + 600
buffer = b""
while time.time() < deadline:
    chunk = port.read(4096)
    if not chunk:
        continue
    buffer += chunk
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        text = line.decode(errors="replace").rstrip()
        if text.endswith("[3d] done"):
            sys.exit(0)
        if "[3d]" in text or "[screen]" in text:
            print(text)
sys.exit("no word from the firmware for ten minutes - is it the bench build, and the port free?")
