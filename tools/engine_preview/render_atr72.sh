#!/bin/bash
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Craig Smith. Overhead's sound engine: PolyForm Noncommercial 1.0.0
# (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.
# render_atr72.sh <worktree> <outdir>: the ATR's preview scenes (preview.cpp's
# atr72-* and atr42-*, the phase tables of atr72-sources.md 14) rendered from
# <worktree>'s synth and preview into <outdir>, each as <scene>.wav and
# <scene>-tab5.wav, all side by side; again with four blades - the ATR 72-200's
# take-off and taxi (-atr72-200), the ATR 42-300's take-off (-atr42-300). Built
# with ENGINE_NOISE_DEBUG, so each scene's log (<outdir>/logs/) says how many
# samples the soft limiter rounded off - it should be under 0.1% - beside
# the airliner's own debug lines; summary.txt gathers the loudness lines
# (dBFS every 2s) and the limiter's.
# And the A320's take-off (CFM56), the level its own are set against, in
# <outdir>/ref/. TRACE=1 on the preview prints what each scene gives the
# synth, once a second, the propellers' speed asked among it.
set -e
W=$(cd "$1" && pwd); OUT=$(mkdir -p "$2" && cd "$2" && pwd)
BIN=$OUT/bin/engine_preview
mkdir -p "$OUT/bin" "$OUT/logs" "$OUT/ref"
g++ -std=c++17 -O2 -Wall -Wextra -DENGINE_NOISE_DEBUG -I"$W/include" "$W/tools/engine_preview/preview.cpp" \
    "$W/src/engine_noise.cpp" -o "$BIN"
SCENES="atr72-start-idle atr72-taxi-past atr72-takeoff atr72-climbout-flyover atr72-cruise-flyover
        atr72-approach-side atr72-approach-behind atr72-touchdown-reverse atr42-takeoff atr72-cruise-cinema"
for s in $SCENES; do
  "$BIN" "$OUT" "$s" > "$OUT/logs/$s.log" 2>&1 &
done
ENGINE=atr72-200 SUFFIX=-atr72-200 "$BIN" "$OUT" atr72-takeoff > "$OUT/logs/atr72-takeoff-atr72-200.log" 2>&1 &
ENGINE=atr72-200 SUFFIX=-atr72-200 "$BIN" "$OUT" atr72-taxi-past > "$OUT/logs/atr72-taxi-past-atr72-200.log" 2>&1 &
ENGINE=atr42-300 SUFFIX=-atr42-300 "$BIN" "$OUT" atr42-takeoff > "$OUT/logs/atr42-takeoff-atr42-300.log" 2>&1 &
ENGINE=cfm56 "$BIN" "$OUT/ref" a320-takeoff > "$OUT/logs/a320-takeoff.log" 2>&1 &
wait
: > "$OUT/summary.txt"
for s in $SCENES atr72-takeoff-atr72-200 atr72-taxi-past-atr72-200 atr42-takeoff-atr42-300 a320-takeoff; do
  grep -E "dBFS every|limiter rounded" "$OUT/logs/$s.log" >> "$OUT/summary.txt"
done
cat "$OUT/summary.txt"
n=$(ls "$OUT"/*.wav | wc -l)
echo "$n WAVs in $OUT (26 expected)"
[ "$n" = 26 ]
