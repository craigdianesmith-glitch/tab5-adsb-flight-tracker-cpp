#!/usr/bin/env python3
"""Turns a voice table of 8-bit samples - src/voice_<name>.inc as
tools/gen_voice.py wrote it before it wrote IMA ADPCM - into IMA ADPCM, the
same words as they were rather than spoken afresh: Piper says a word a little
differently each time, and these were chosen by ear. Encoded by
tools/adpcm_search.c, compiled here with cc, which looks a couple of samples
ahead for each code it picks - 26dB of signal to the noise it adds, where
the standard encoder in tools/ima_adpcm.py had 20.

    python3 tools/voice_adpcm.py src/voice_scottish.inc [...]

Rewrites each file in place, and says how far each voice's words came out
from what they were.
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(__file__))
import ima_adpcm  # noqa: E402

CLIP = re.compile(r'const int8_t (\w+)\[\] = \{  // (.*?)\n(.*?)\n\};', re.S)

for path in sys.argv[1:]:
    text = open(path).read()
    if "const int8_t" not in text:
        print(f"{path}: no 8-bit clips in it - converted already?")
        continue
    counts = {}
    noise = signal = 0.0
    out = []
    pos = 0
    found = list(CLIP.finditer(text))
    clips = [[int(v) * 256 for v in re.findall(r'-?\d+', m.group(3))] for m in found]
    codes = ima_adpcm.encode_search(clips)
    for m, samples, data in zip(found, clips, codes):
        name, comment = m.group(1), m.group(2)
        # Decoded as the firmware will, to say what was lost.
        back = ima_adpcm.decode(data, len(samples))
        signal += sum(s * s for s in samples)
        noise += sum((a - b) ** 2 for a, b in zip(samples, back))
        counts[name] = len(samples)
        out.append(text[pos:m.start()])
        lines = [f"const uint8_t {name}[] = {{  // {comment}, {len(samples)} samples"]
        for i in range(0, len(data), 32):
            lines.append("    " + ",".join(str(b) for b in data[i:i + 32]) + ",")
        lines.append("};")
        out.append("\n".join(lines))
        pos = m.end()
    out.append(text[pos:])
    text = "".join(out)
    text = re.sub(r'\{(\w+), sizeof\(\1\)\}', lambda m: f"{{{m.group(1)}, {counts[m.group(1)]}}}", text)
    text = text.replace("// The spoken words, as 8-bit mono samples,",
                        "// The spoken words, as 4-bit IMA ADPCM - see tools/ima_adpcm.py - from 8-bit mono samples")
    open(path, "w").write(text)
    total = sum(counts.values())
    print(f"{path}: {len(counts)} clips, {total} samples, {(total + 1) // 2 // 1024}KB "
          f"(was {total // 1024}KB), {10 * math.log10(signal / noise):.1f}dB signal to added noise")
