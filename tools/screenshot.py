#!/usr/bin/env python3
"""Save what the Tab5 is showing as a PNG.

    python tools/screenshot.py [output.png] [--port /dev/ttyACM0]

Asks the running firmware for its canvas over USB serial and converts the
1280x720 RGB565 pixels it sends back. Needs pyserial and Pillow
(pip install pyserial pillow). Close any serial monitor first - only one
program can hold the port.
"""
import argparse
import sys
import time

import serial
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument("output", nargs="?", default=time.strftime("tab5-%Y%m%d-%H%M%S.png"))
ap.add_argument("--port", default="/dev/ttyACM0")
args = ap.parse_args()

# DTR and RTS are left as the kernel raises them on open. Dropping DTR while
# RTS is still up is the ESP32's USB-serial reset signal, which is exactly
# what setting both low does - pyserial clears DTR first.
port = serial.Serial(args.port, 115200, timeout=5)


def grab():
    port.reset_input_buffer()
    port.write(b"##S")
    # Log lines may arrive ahead of the header; skip until it turns up.
    deadline = time.time() + 10
    while True:
        if time.time() > deadline:
            sys.exit("no screenshot header - is the firmware running and the port free?")
        line = port.readline().decode(errors="replace").strip()
        if line.startswith("SCREENSHOT ") and line != "SCREENSHOT END":
            _, w, h = line.split()
            w, h = int(w), int(h)
            break
    size = w * h * 2
    data = bytearray()
    while len(data) < size:
        chunk = port.read(size - len(data))
        if not chunk:
            sys.exit(f"timed out after {len(data)} of {size} bytes")
        data += chunk
    # The end marker has to follow the last pixel exactly. If another task
    # logged a line mid-transfer, the pixels are shifted and it won't.
    tail = port.read(len(b"\nSCREENSHOT END\n"))
    return w, h, data, tail == b"\nSCREENSHOT END\n"


for attempt in range(3):
    w, h, data, ok = grab()
    if ok:
        break
    print("transfer interleaved with log output, retrying")
else:
    sys.exit("could not get a clean transfer")
port.close()

# The canvas stores RGB565 high byte first.
pixels = bytearray(w * h * 3)
for i in range(w * h):
    v = (data[2 * i] << 8) | data[2 * i + 1]
    r, g, b = (v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F
    pixels[3 * i] = (r << 3) | (r >> 2)
    pixels[3 * i + 1] = (g << 2) | (g >> 4)
    pixels[3 * i + 2] = (b << 3) | (b >> 2)
Image.frombytes("RGB", (w, h), bytes(pixels)).save(args.output)
print(f"saved {args.output} ({w}x{h})")
