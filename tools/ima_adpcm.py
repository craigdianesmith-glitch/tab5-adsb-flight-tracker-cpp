"""IMA ADPCM, as the firmware decodes it (src/sound.cpp): four bits a sample,
the first of each pair in a byte's low nibble, from a predictor of 0 and a step
index of 0 at the start of each clip - a word's clip starts in its own quiet.

    encode_search(clips) -> [bytes]  each clip of 16-bit samples, by the search
                                     in tools/adpcm_search.c (compiled with cc)
    encode(samples) -> bytes         the standard encoder, for comparison
    decode(data, count) -> list      and back, as the firmware hears them

The search picks each code for the least error over that sample and the next
two, rather than the nearest alone: on the voices' squeezed words, 26 to 28dB
of signal to the noise it adds, where the standard encoder had 20 to 23.
"""
import os
import struct
import subprocess
import tempfile

STEP = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97,
    107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871,
    5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
    27086, 29794, 32767,
]
INDEX = [-1, -1, -1, -1, 2, 4, 6, 8]


def _next(code, pred, index):
    step = STEP[index]
    delta = step >> 3
    if code & 4:
        delta += step
    if code & 2:
        delta += step >> 1
    if code & 1:
        delta += step >> 2
    pred = pred - delta if code & 8 else pred + delta
    pred = max(-32768, min(32767, pred))
    index = max(0, min(88, index + INDEX[code & 7]))
    return pred, index


def encode(samples):
    pred, index = 0, 0
    out = bytearray()
    for i, s in enumerate(samples):
        step = STEP[index]
        diff = s - pred
        code = 0
        if diff < 0:
            code = 8
            diff = -diff
        if diff >= step:
            code |= 4
            diff -= step
        if diff >= step >> 1:
            code |= 2
            diff -= step >> 1
        if diff >= step >> 2:
            code |= 1
        pred, index = _next(code, pred, index)
        if i % 2 == 0:
            out.append(code)
        else:
            out[-1] |= code << 4
    return bytes(out)


def decode(data, count):
    pred, index = 0, 0
    out = []
    for i in range(count):
        code = (data[i >> 1] >> (4 * (i & 1))) & 15
        pred, index = _next(code, pred, index)
        out.append(pred)
    return out


def encode_search(clips):
    encoder = os.path.join(tempfile.gettempdir(), "adpcm_search")
    source = os.path.join(os.path.dirname(os.path.abspath(__file__)), "adpcm_search.c")
    subprocess.run(["cc", "-O2", "-o", encoder, source], check=True)
    stdin = b"".join(struct.pack(f"<I{len(c)}h", len(c), *c) for c in clips)
    out = subprocess.run([encoder], input=stdin, capture_output=True, check=True).stdout
    codes, at = [], 0
    for c in clips:
        size = (len(c) + 1) // 2
        codes.append(out[at:at + size])
        at += size
    return codes
