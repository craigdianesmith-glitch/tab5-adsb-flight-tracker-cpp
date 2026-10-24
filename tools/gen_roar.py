"""Cut a fighter's roar from real recordings into src/roar_table.inc.

    python tools/gen_roar.py [--source rear.wav|.mp3] [--front front.wav|.mp3] [--wav loop.wav]

The 3D view's fighters - low-bypass jets - roar with this loop rather than
with noise shaped to look like one: shaped noise, however its spectrum was
set, came out a gale on the Tab5's small speaker, which plays nothing much
under 300Hz, where a jet's roar has its weight. A recording keeps what that
speaker can carry: the roar's churning, its tearing, its steepened edges.

The recording: "Eurofighter Typhoon.wav" by johnnyguitar01 on Freesound,
https://freesound.org/people/johnnyguitar01/sounds/416788/ - Creative
Commons 0, the public domain - a Typhoon at Dunsfold the day before Wings
and Wheels 2015, holding its roar steady for nine seconds: no pass in it,
so no Doppler shift of its own - the 3D view puts the one its flight makes.
Without --source its preview is fetched from Freesound and decoded with
ffmpeg.

From 1s to 10s of it: under 150Hz taken off, which the speaker can't play;
its loudness held level over 0.6s - its surges slower than that out, the
3D view making its own, which never come round again: one 4dB swell, 5s
in, came round with the stretches it was in, a landmark the ear knew the
loop by - and set to -16dBFS; and its last 0.3s laid over its first, so it
runs round without a seam. Mono 16-bit at 16kHz, nine seconds, 281KB of
flash; --wav writes the loop out too, to listen to.

And its front, which that roar - its side and back - is not: heard ahead of
it, quiet, it was a dull rush, a gale, where a jet coming at you screams of
its intake. From "eurofighter.wav" by joly1 on Freesound,
https://freesound.org/people/joly1/sounds/69878/ - Creative Commons 0 -
evolutions at a hundred yards: 224.5s to 228.25s, one coming at the
microphone, its loudness held level through its coming, and slowed by the
half again its approach raised it by - some 300 knots, a little off the
line to the microphone - to the pitch it makes itself, for the 3D view to
put its own flight's on. Under 150Hz off, -16dBFS, and seamless likewise:
5.6 seconds, 176KB more.

And an airliner's, to set against its synthesised voice - its take-off roar
and its approach, by its engines' power:
 - "Passenger jet liftoff" by PostProdDog, Freesound 536510, CC0: 4s to 16s,
   climbing away at full power, its loudness steady - sped up by the fifth
   its going away lowered it, some 80m/s;
 - "Commercial jet flies overhead.wav" by C-V, Freesound 509046, CC0: 22s to
   29s, overhead at 1,500ft on its approach, side on, so as its own pitch.

And a turboprop's and a light aircraft's, likewise against their voices -
each pass's Doppler shift taken out exactly, by when each moment's sound
left the aircraft, from the line it flew: its speed, how near it came and
when, fitted to its propellers' tone through the pass:
 - "ATR 72 (AT72) plane flyby at 300 m altitude landing towards airport" by
   Hoscalegeek, Freesound 315660, CC0: 39s to 46s, coming in at 95kt 612m
   off, its six blades' tone 119.3Hz - its propellers at their 1,200rpm;
 - "King Air 350 Takeoff" by highflyer500, Freesound 502670, CC0: 5s to
   10.5s, climbing away past at 132kt 156m off at full power, its four
   blades' 113.1Hz - 1,700rpm;
 - "CESSNA Flyby 150Mts close" by MarioV360, Freesound 828766, CC0: 3.5s to
   10.5s, on its approach, its power coming off as it went by - its
   strongest tone, Doppler and throttle together, a cubic through its track,
   held at 171Hz: the third of its firing, at 1,710rpm.
"""
import argparse
import math
import os
import struct
import subprocess
import tempfile
import urllib.request
import wave

RATE = 16000
SOURCE_URL = "https://cdn.freesound.org/previews/416/416788_4844309-hq.mp3"
FRONT_URL = "https://cdn.freesound.org/previews/69/69878_466559-hq.mp3"
FROM_S, LOOP_S, BLEND_S = 1.0, 9.0, 0.3
FRONT_FROM_S, FRONT_TO_S, FRONT_DOPPLER = 224.5, 228.25, 1.5
TAKEOFF_URL = "https://cdn.freesound.org/previews/536/536510_6911631-hq.mp3"
APPROACH_URL = "https://cdn.freesound.org/previews/509/509046_8956746-hq.mp3"
TAKEOFF_FROM_S, TAKEOFF_TO_S, TAKEOFF_DOPPLER = 4.0, 16.3, 1 / 1.23
APPROACH_FROM_S, APPROACH_TO_S = 22.0, 29.3
TURBOPROP_URL = "https://cdn.freesound.org/previews/315/315660_2506497-hq.mp3"
TURBOPROP_FROM_S, TURBOPROP_TO_S, TURBOPROP_PASS = 39.0, 46.0, (48.7, 612.0, 47.98)
KINGAIR_URL = "https://cdn.freesound.org/previews/502/502670_10785049-hq.mp3"
KINGAIR_FROM_S, KINGAIR_TO_S, KINGAIR_PASS = 5.0, 10.5, (68.15, 156.0, 6.74)
PISTON_URL = "https://cdn.freesound.org/previews/828/828766_17943094-hq.mp3"
PISTON_FROM_S, PISTON_TO_S, PISTON_HZ = 3.5, 10.5, 171.0
PISTON_TONE = (223.254, -11.3689, 0.898037, -0.0323762)  # Hz, by seconds into it
SOUND_MS = 343.0
HIGH_PASS_HZ = 150
LEVEL = 10 ** (-16 / 20)  # its loudness, RMS against full scale


def decoded(path):
    """The recording as 16kHz mono samples, decoded by ffmpeg."""
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "source.wav")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", path, "-ac", "1", "-ar", str(RATE),
                        "-c:a", "pcm_s16le", out], check=True)
        with wave.open(out, "rb") as w:
            data = w.readframes(w.getnframes())
    return list(struct.unpack(f"<{len(data) // 2}h", data))


def high_pass(x, hz):
    """A two-pole Butterworth high-pass, the bilinear transform's."""
    k = math.tan(math.pi * hz / RATE)
    norm = 1 / (1 + math.sqrt(2) * k + k * k)
    b0, b1, b2 = norm, -2 * norm, norm
    a1, a2 = 2 * (k * k - 1) * norm, (1 - math.sqrt(2) * k + k * k) * norm
    y, x1, x2, y1, y2 = [], 0.0, 0.0, 0.0, 0.0
    for v in x:
        out = b0 * v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1, y2, y1 = x1, v, y1, out
        y.append(out)
    return y


def level_out(x, window_s=0.4):
    """Its loudness held level: each sample over the RMS of the window round it."""
    w = int(window_s * RATE)
    sq = [0.0]
    for v in x:
        sq.append(sq[-1] + v * v)
    out = []
    for i, v in enumerate(x):
        a, b = max(0, i - w // 2), min(len(x), i + w // 2)
        out.append(v / max(1e-6, math.sqrt((sq[b] - sq[a]) / (b - a))))
    return out


def slowed(x, factor):
    """Played `factor` times slower, between samples a straight line."""
    out, n = [], int((len(x) - 1) * factor)
    for i in range(n):
        p = i / factor
        k = int(p)
        out.append(x[k] + (x[k + 1] - x[k]) * (p - k))
    return out


def at_times(x, times):
    """x at each of `times` - seconds into it - between samples a straight line."""
    out = []
    for t in times:
        p = t * RATE
        k = int(p)
        out.append(x[k] + (x[k + 1] - x[k]) * (p - k))
    return out


def unshifted(x, pass_, from_s, to_s):
    """x from from_s to to_s heard as the aircraft made it: a pass along a line,
    at v m/s, d metres off at its nearest at t0 - by when each moment left it -
    each sample of its own time taken from when that moment was heard."""
    v, d, t0 = pass_

    def heard(tau):
        return tau + math.sqrt(d * d + (v * (tau - t0)) ** 2) / SOUND_MS

    def made(t):  # when what is heard at t left it
        tau = t - d / SOUND_MS
        for _ in range(40):
            tau = t - math.sqrt(d * d + (v * (tau - t0)) ** 2) / SOUND_MS
        return tau

    a, b = made(from_s), made(to_s)
    return at_times(x, [heard(a + i / RATE) for i in range(int((b - a) * RATE))])


def retuned(x, tone, hz, from_s, to_s):
    """x from from_s to to_s with its strongest tone - `tone`'s cubic, by the
    second - held at `hz`: read the slower the higher it was heard."""
    times, t = [], from_s
    while t < to_s:
        times.append(t)
        t += hz / (tone[0] + t * (tone[1] + t * (tone[2] + t * tone[3]))) / RATE
    return at_times(x, times)


def looped(x, n, blend):
    """x's first n samples at -16dBFS, its continuation laid over its start."""
    rms = math.sqrt(sum(v * v for v in x[:n + blend]) / (n + blend))
    x = [v * LEVEL * 32768 / rms for v in x[:n + blend]]
    loop = x[:n]
    for i in range(blend):
        t = i / blend
        loop[i] = x[i] * math.sin(t * math.pi / 2) + x[n + i] * math.cos(t * math.pi / 2)
    return [max(-32768, min(32767, int(round(v)))) for v in loop]


def fetched(url):
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "source.mp3")
        urllib.request.urlretrieve(url, path)
        return decoded(path)


def write_table(f, name, samples):
    f.write(f"const int16_t {name}[] = {{  // {len(samples) / RATE:.2f}s\n")
    for i in range(0, len(samples), 16):
        f.write("    " + ", ".join(str(s) for s in samples[i:i + 16]) + ",\n")
    f.write("};\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", help="the recording of its roar; fetched from Freesound if not given")
    ap.add_argument("--front", help="the recording of its front; fetched from Freesound if not given")
    ap.add_argument("--takeoff", help="an airliner's take-off; fetched from Freesound if not given")
    ap.add_argument("--approach", help="an airliner's approach; fetched from Freesound if not given")
    ap.add_argument("--turboprop", help="a turboprop's approach; fetched from Freesound if not given")
    ap.add_argument("--kingair", help="a turboprop's take-off; fetched from Freesound if not given")
    ap.add_argument("--piston", help="a light aircraft's pass; fetched from Freesound if not given")
    ap.add_argument("--out", default="src/roar_table.inc")
    ap.add_argument("--wav", help="also write the loop here, to listen to")
    args = ap.parse_args()

    samples = decoded(args.source) if args.source else fetched(SOURCE_URL)
    a, n, blend = int(FROM_S * RATE), int(LOOP_S * RATE), int(BLEND_S * RATE)
    out = looped(level_out(high_pass([float(v) for v in samples], HIGH_PASS_HZ)[a:a + n + blend], 0.6), n, blend)

    # Its front: levelled, slowed by its approach's Doppler shift, looped.
    samples = decoded(args.front) if args.front else fetched(FRONT_URL)
    x = high_pass([float(v) for v in samples], HIGH_PASS_HZ)[int(FRONT_FROM_S * RATE):int(FRONT_TO_S * RATE)]
    x = slowed(level_out(x), FRONT_DOPPLER)
    front = looped(x, len(x) - blend, blend)

    # An airliner's: its take-off roar levelled and sped back up by its going
    # away's Doppler shift; its approach overhead as it is.
    samples = decoded(args.takeoff) if args.takeoff else fetched(TAKEOFF_URL)
    x = high_pass([float(v) for v in samples], HIGH_PASS_HZ)[int(TAKEOFF_FROM_S * RATE):int(TAKEOFF_TO_S * RATE)]
    x = slowed(level_out(x, 1.0), TAKEOFF_DOPPLER)
    takeoff = looped(x, len(x) - blend, blend)
    samples = decoded(args.approach) if args.approach else fetched(APPROACH_URL)
    x = high_pass([float(v) for v in samples], HIGH_PASS_HZ)[int(APPROACH_FROM_S * RATE):int(APPROACH_TO_S * RATE)]
    approach = looped(level_out(x, 1.0), len(x) - blend, blend)

    # A turboprop's, at low power and at full, and a light aircraft's: each
    # as its aircraft made it, levelled.
    samples = decoded(args.turboprop) if args.turboprop else fetched(TURBOPROP_URL)
    x = unshifted(high_pass([float(v) for v in samples], HIGH_PASS_HZ), TURBOPROP_PASS, TURBOPROP_FROM_S,
                  TURBOPROP_TO_S)
    turboprop = looped(level_out(x, 1.0), len(x) - blend, blend)
    samples = decoded(args.kingair) if args.kingair else fetched(KINGAIR_URL)
    x = unshifted(high_pass([float(v) for v in samples], HIGH_PASS_HZ), KINGAIR_PASS, KINGAIR_FROM_S, KINGAIR_TO_S)
    kingair = looped(level_out(x, 0.4), len(x) - blend, blend)
    samples = decoded(args.piston) if args.piston else fetched(PISTON_URL)
    x = retuned(high_pass([float(v) for v in samples], HIGH_PASS_HZ), PISTON_TONE, PISTON_HZ, PISTON_FROM_S,
                PISTON_TO_S)
    piston = looped(level_out(x, 1.0), len(x) - blend, blend)

    with open(args.out, "w") as f:
        f.write("// Generated by tools/gen_roar.py - do not edit.\n")
        f.write("// A fighter's roar, loops cut from recordings of Eurofighter Typhoons, as\n")
        f.write("// 16-bit mono: its side and back at Dunsfold by johnnyguitar01 (Freesound\n")
        f.write("// 416788, CC0); its front coming at the microphone by joly1 (Freesound\n")
        f.write("// 69878, CC0).\n")
        f.write(f"constexpr uint32_t ROAR_RATE = {RATE};\n")
        write_table(f, "ROAR_LOOP", out)
        write_table(f, "ROAR_FRONT", front)
        f.write("// An airliner's take-off climbing away (PostProdDog, Freesound 536510, CC0)\n")
        f.write("// and its approach overhead (C-V, Freesound 509046, CC0).\n")
        write_table(f, "AIRLINER_TAKEOFF", takeoff)
        write_table(f, "AIRLINER_APPROACH", approach)
        f.write("// A turboprop's approach (an ATR 72, Hoscalegeek, Freesound 315660, CC0) and\n")
        f.write("// take-off (a King Air 350, highflyer500, Freesound 502670, CC0), and a\n")
        f.write("// light aircraft's (a Cessna, MarioV360, Freesound 828766, CC0), each with\n")
        f.write("// its pass's Doppler shift taken out: their tones 119.3Hz, 113.1Hz, 171Hz.\n")
        write_table(f, "TURBOPROP_LOW", turboprop)
        write_table(f, "TURBOPROP_HIGH", kingair)
        write_table(f, "PISTON_LOOP", piston)
    if args.wav:
        for path, loop in ((args.wav, out), (args.wav.replace(".wav", "-front.wav"), front),
                           (args.wav.replace(".wav", "-turboprop.wav"), turboprop),
                           (args.wav.replace(".wav", "-kingair.wav"), kingair),
                           (args.wav.replace(".wav", "-piston.wav"), piston)):
            with wave.open(path, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(RATE)
                w.writeframes(struct.pack(f"<{len(loop)}h", *loop))
    for name, loop in (("its roar", out), ("its front", front), ("an airliner's take-off", takeoff),
                       ("an airliner's approach", approach), ("a turboprop's approach", turboprop),
                       ("a turboprop's take-off", kingair), ("a light aircraft's", piston)):
        peak = max(abs(v) for v in loop)
        print(f"{args.out}, {name}: {len(loop)} samples, {len(loop) / RATE:.2f}s, {len(loop) * 2 // 1024}KB, "
              f"peak {20 * math.log10(peak / 32768):.1f}dBFS")


if __name__ == "__main__":
    main()
