#!/bin/bash
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
# (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.
# render_same.sh <worktree> <reference> <outdir>: whether every sound the
# reference makes - the A320's, the Typhoon's, the 777's, the boot splash's -
# is still made byte for byte by <worktree>'s synth and preview. <reference>
# is a tree, or a commit of <worktree>'s repository (git archive'd into
# <outdir>/ref-src): for the turboprop's builds, atr72-base or 28518ba, the
# 777's own. Both previews are built as the preview's own header has it, and
# every one of their scenes rendered in turn - with no ENGINE= and with each
# turbofan, the fuel vents' whistle, the transients on the picture, the
# device's frames, another airframe and eufi-low-pass at 0.85 - then each WAV
# the reference made (full range and -tab5) compared with the worktree's of
# the same name: the worktree's own new scenes, a turboprop's, aren't in the
# count. Writes <outdir>/same.txt, one line a file, and the WAVs' checksums;
# the WAVs themselves deleted unless KEEP=1 (some 4GB). Exits non-zero unless
# all are identical.
set -e
W=$(cd "$1" && pwd); REF=$2; OUT=$(mkdir -p "$3" && cd "$3" && pwd)
if [ -d "$REF" ]; then
  R=$(cd "$REF" && pwd)
else
  R=$OUT/ref-src
  rm -rf "$R"; mkdir -p "$R"
  git -C "$W" archive "$REF" include src tools/engine_preview | tar -x -C "$R"
fi
mkdir -p "$OUT/bin"
g++ -std=c++17 -O2 -I"$W/include" "$W/tools/engine_preview/preview.cpp" "$W/src/engine_noise.cpp" -o "$OUT/bin/new" &
g++ -std=c++17 -O2 -I"$R/include" "$R/tools/engine_preview/preview.cpp" "$R/src/engine_noise.cpp" -o "$OUT/bin/ref" &
wait
RUNS=("default|" "cfm56|ENGINE=cfm56" "v2500|ENGINE=v2500" "leap|ENGINE=leap" "pw|ENGINE=pw" "ge90|ENGINE=ge90"
      "ge90-94|ENGINE=ge90-94" "trent|ENGINE=trent" "cfm56-fopp|ENGINE=cfm56 FOPP=1" "v2500-fopp|ENGINE=v2500 FOPP=1"
      "cfm56-sync|ENGINE=cfm56 SYNC=1" "leap-sync|ENGINE=leap SYNC=1" "device|DEVICE=1" "who7|WHO=7"
      "power085|POWER=0.85")
for run in "${RUNS[@]}"; do
  name=${run%%|*}; vars=${run#*|}
  for side in ref new; do
    mkdir -p "$OUT/$side/$name"
    (env $vars "$OUT/bin/$side" "$OUT/$side/$name" > "$OUT/$side/$name.log" 2>&1) &
  done
done
wait
same=0; differ=0; missing=0
: > "$OUT/same.txt"
for run in "${RUNS[@]}"; do
  name=${run%%|*}
  (cd "$OUT/ref/$name" && md5sum *.wav) > "$OUT/ref/$name.md5"
  (cd "$OUT/new/$name" && md5sum *.wav) > "$OUT/new/$name.md5"
  while read -r sum file; do
    other=$(awk -v f="$file" '$2 == f {print $1}' "$OUT/new/$name.md5")
    if [ -z "$other" ]; then missing=$((missing + 1)); echo "missing $name/$file" >> "$OUT/same.txt"
    elif [ "$other" = "$sum" ]; then same=$((same + 1)); echo "same    $name/$file $sum" >> "$OUT/same.txt"
    else differ=$((differ + 1)); echo "DIFFERS $name/$file $sum $other" >> "$OUT/same.txt"; fi
  done < "$OUT/ref/$name.md5"
  # the dBFS lines too - stdout, every scene's loudness every 2s
  grep -v '^atr' "$OUT/new/$name.log" > "$OUT/new/$name.levels" || true
  cp "$OUT/ref/$name.log" "$OUT/ref/$name.levels"
  if ! cmp -s "$OUT/ref/$name.levels" "$OUT/new/$name.levels"; then
    differ=$((differ + 1)); echo "DIFFERS $name: the loudness lines" >> "$OUT/same.txt"
  fi
  if [ -z "$KEEP" ]; then rm -f "$OUT/ref/$name"/*.wav "$OUT/new/$name"/*.wav; fi
done
grep -v '^same' "$OUT/same.txt" || true
echo "$same identical, $differ differ, $missing missing ($(cd "$OUT/ref" && cat *.md5 | wc -l) WAVs from the reference in ${#RUNS[@]} runs)" | tee -a "$OUT/same.txt"
[ "$differ" = 0 ] && [ "$missing" = 0 ]
