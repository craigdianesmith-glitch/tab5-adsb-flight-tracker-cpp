#!/bin/bash
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
# (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.
# render_turboprops.sh <worktree> <outdir> [engine ...]: the other turboprops'
# rows on the ATR's voice, each rendered from <worktree>'s synth and preview
# into <outdir>/<engine>/ in three of the ATR's scenes - the take-off beside
# the runway (atr72-takeoff, 700m along and 120m off), the approach from
# beside (atr72-approach-side, 300m up and 300m off) and the taxi past
# (atr72-taxi-past, 35m off at 8m/s) - each as <scene>.wav and
# <scene>-tab5.wav, the scenes' paths and timings the ATR's, its engines,
# propellers, power by phase and level its row's (ENGINE=, preview.cpp). With
# no engines named, every row's. And the ATR 72-600's same three in
# <outdir>/atr72-600/, to set them beside. Built with ENGINE_NOISE_DEBUG, so
# each scene's log (<outdir>/logs/) says how many samples the soft limiter
# rounded off - it should be under 0.1%; summary.txt gathers the loudness
# lines (dBFS every 2s) and the limiter's.
set -e
W=$(cd "$1" && pwd); OUT=$(mkdir -p "$2" && cd "$2" && pwd)
shift 2
ENGINES=${*:-"q400 saab340 twinotter kingair350 kingair200 pc12 caravan beech1900 tbm700 tbm900 c130j a400m
              dash8-100 dash8-300 c295 cn235 fokker50 do328 emb120 saab2000 kingair90 kodiak pc6"}
BIN=$OUT/bin/engine_preview
mkdir -p "$OUT/bin" "$OUT/logs"
g++ -std=c++17 -O2 -Wall -Wextra -DENGINE_NOISE_DEBUG -I"$W/include" "$W/tools/engine_preview/preview.cpp" \
    "$W/src/engine_noise.cpp" -o "$BIN"
SCENES="atr72-takeoff atr72-approach-side atr72-taxi-past"
jobs=0
for e in atr72-600 $ENGINES; do
  mkdir -p "$OUT/$e"
  for s in $SCENES; do
    ENGINE=$e "$BIN" "$OUT/$e" "$s" > "$OUT/logs/$e-$s.log" 2>&1 &
    jobs=$((jobs + 1))
    if [ $((jobs % 16)) = 0 ]; then wait; fi
  done
done
wait
: > "$OUT/summary.txt"
n=0
for e in atr72-600 $ENGINES; do
  for s in $SCENES; do
    echo "$e $s: $(grep -E "limiter rounded" "$OUT/logs/$e-$s.log" | head -1)" >> "$OUT/summary.txt"
    grep -E "dBFS every" "$OUT/logs/$e-$s.log" >> "$OUT/summary.txt" || true
    [ -f "$OUT/$e/$s.wav" ] && [ -f "$OUT/$e/$s-tab5.wav" ] && n=$((n + 2))
  done
done
want=$(( ($(echo $ENGINES | wc -w) + 1) * 6 ))
echo "$n WAVs in $OUT ($want expected)"
[ "$n" = "$want" ]
