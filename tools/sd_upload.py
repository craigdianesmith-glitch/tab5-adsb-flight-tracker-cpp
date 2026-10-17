#!/usr/bin/env python3
"""Put a file on the Tab5's SD card over USB, without taking the card out -
for now the airfield layouts tools/gen_airfields.py builds:

    python tools/sd_upload.py airfields.bin [--port /dev/ttyACM0]

Needs firmware built with the 3D bench in it (main.cpp's "##u"). It goes in
blocks of 4KB, each answered, and is checked by its CRC-32 at the end; the
card keeps the old file until the new one has all come. Needs pyserial; close
any serial monitor first.
"""
import argparse
import sys
import time
import zlib
from pathlib import Path

import serial

ap = argparse.ArgumentParser()
ap.add_argument("file")
ap.add_argument("--port", default="/dev/ttyACM0")
args = ap.parse_args()

data = Path(args.file).read_bytes()
name = Path(args.file).name
crc = zlib.crc32(data) & 0xFFFFFFFF

# DTR and RTS are left as the kernel raises them on open: setting both low is
# the ESP32's reset signal.
port = serial.Serial(args.port, 115200, timeout=0.2)


def answer(seconds=10):
    end = time.time() + seconds
    while time.time() < end:
        line = port.readline().decode("utf-8", "replace").strip()
        if line.startswith("UPLOAD"):
            return line
    sys.exit("no answer from the Tab5 - is the bench firmware on it?")


port.reset_input_buffer()
port.write(f"##u{name} {len(data)} {crc:08x}\n".encode())
line = answer()
if line != "UPLOAD READY":
    sys.exit(line)
started = time.time()
for at in range(0, len(data), 4096):
    # A few hundred bytes at a time: the Tab5's USB receive buffer is small,
    # and a block sent all at once overflows it before it is read.
    block = data[at:at + 4096]
    for piece in range(0, len(block), 256):
        port.write(block[piece:piece + 256])
        port.flush()
        time.sleep(0.002)
    line = answer()
    if line.startswith("UPLOAD FAILED") or line.startswith("UPLOAD REFUSED"):
        sys.exit(line)
    print(f"\r{at + len(data[at:at + 4096])} of {len(data)} bytes", end="", flush=True)
line = answer()
print(f"\n{line}, in {time.time() - started:.1f}s")
port.close()
sys.exit(0 if line.startswith("UPLOAD DONE") else 1)
