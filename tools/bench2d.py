#!/usr/bin/env python3
"""Time the rest of the app on the Tab5 - the 3D view has tools/bench3d.py -
a part at a time, and print what the firmware says.

    python tools/bench2d.py <letters> [--port /dev/ttyACM0]

Needs firmware built with the bench in it, from the table:

    PLATFORMIO_BUILD_FLAGS=-DBENCH3D pio run -e tab5 -t upload

The letters, as include/bench2d.h has them: m the map's first draw, n the
names, r the radar's plot, z the airport zoom, t the table, x the whole
screen's push, p a reply from the feed parsed (bench-*.json put on the card
first with tools/sd_upload.py), k the tracks, s the voices and the engines.
"mnrztxpks" runs them all, one after another.

Needs pyserial; close any serial monitor first.
"""
import argparse
import sys
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("letters")
ap.add_argument("--port", default="/dev/ttyACM0")
args = ap.parse_args()

# DTR and RTS are left as the kernel raises them on open: setting both low is
# the ESP32's reset signal.
port = serial.Serial(args.port, 115200, timeout=0.5)
for letter in args.letters:
    port.reset_input_buffer()
    port.write(("##=" + letter).encode())
    deadline = time.time() + 300
    buffer = b""
    done = False
    while time.time() < deadline and not done:
        chunk = port.read(4096)
        if not chunk:
            continue
        buffer += chunk
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            text = line.decode(errors="replace").rstrip()
            if text.endswith("[bench] done"):
                done = True
                break
            if "[bench]" in text:
                print(text, flush=True)
    if not done:
        sys.exit(f"no word from the firmware on '{letter}' for five minutes - is it the bench build, and the port free?")
