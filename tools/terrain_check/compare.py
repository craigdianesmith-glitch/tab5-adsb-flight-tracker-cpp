# Two of run.sh's sets of frames set against each other, folder by folder,
# from their logs alone: each frame's image hash ("image NAME HASH") and
# march digest ("tstats ... digest HASH"), and the march's own counts beside
# them. Exit 1 if any image or digest differs, or a frame is missing.
#
#   python3 compare.py <set> <other set>
import os
import re
import sys


def read(folder):
    images, digests = {}, []
    us = samples = rows = 0
    for line in open(os.path.join(folder, 'log.txt'), errors='replace'):
        m = re.match(r'image (\S+) ([0-9a-f]{8})', line)
        if m:
            images[m.group(1)] = m.group(2)
        m = re.match(r'tstats \d+: march (\d+)us (\d+) samples (\d+) rows', line)
        if m:
            us += int(m.group(1))
            samples += int(m.group(2))
            rows += int(m.group(3))
            g = re.search(r'digest ([0-9a-f]{8})', line)
            digests.append(g.group(1) if g else None)
    return images, digests, us, samples, rows


def main():
    a, b = sys.argv[1], sys.argv[2]
    bad = False
    for name in sorted(os.listdir(a)):
        if not os.path.isdir(os.path.join(a, name)):
            continue
        ia, da, ua, na, ra = read(os.path.join(a, name))
        ib, db, ub, nb, rb = read(os.path.join(b, name))
        differ = [k for k in ia if k in ib and ia[k] != ib[k]]
        missing = [k for k in ia if k not in ib] if ia else ['(no frames)']
        digests = sum(1 for x, y in zip(da, db) if x != y) + abs(len(da) - len(db))
        bad = bad or bool(differ or missing or digests)
        first = f' first {differ[0]}' if differ else ''
        gone = f', {len(missing)} missing' if missing else ''
        print(f'{name:9s} {len(ia):3d} frames: {len(differ)} images differ{first}, {digests} digests differ{gone}'
              f' | march {ua / 1000:7.1f}ms -> {ub / 1000:7.1f}ms, samples {na} -> {nb}, rows {ra} -> {rb}')
    sys.exit(1 if bad else 0)


main()
