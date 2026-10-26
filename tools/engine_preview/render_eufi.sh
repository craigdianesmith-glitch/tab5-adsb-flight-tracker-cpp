#!/bin/bash
# render_eufi.sh <worktree> <outdir>: the Typhoon's preview scenes (preview.cpp's
# eufi-* and x-eufi-*, eufi-sources.md section 12) rendered from <worktree>'s
# synth and preview into <outdir>, each as <scene>.wav and <scene>-tab5.wav,
# all side by side; and eufi-low-pass again at POWER=0.85 as eufi-low-pass-085.
# Built with ENGINE_NOISE_DEBUG, so each scene's log (<outdir>/logs/) says how
# many samples the soft limiter rounded off - it should be under 0.1%.
# summary.txt gathers the loudness lines (dBFS every 2 s) and the limiter's.
# And the A320's take-off (CFM56), the level the Typhoon's loudest scenes are
# set against, in <outdir>/ref/.
set -e
W=$(cd "$1" && pwd); OUT=$(mkdir -p "$2" && cd "$2" && pwd)
BIN=$OUT/bin/engine_preview
mkdir -p "$OUT/bin" "$OUT/logs" "$OUT/ref"
g++ -std=c++17 -O2 -Wall -Wextra -DENGINE_NOISE_DEBUG -I"$W/include" "$W/tools/engine_preview/preview.cpp" \
    "$W/src/engine_noise.cpp" -o "$BIN"
SCENES="eufi-stand-start eufi-taxi-past eufi-takeoff eufi-climbout-overhead eufi-display-pass eufi-low-pass
        eufi-flypast eufi-chase-cruise eufi-approach-chase eufi-landing x-eufi-zeltweg x-eufi-close-pass
        x-eufi-display-100m"
for s in $SCENES; do
  "$BIN" "$OUT" "$s" > "$OUT/logs/$s.log" 2>&1 &
done
POWER=0.85 SUFFIX=-085 "$BIN" "$OUT" eufi-low-pass > "$OUT/logs/eufi-low-pass-085.log" 2>&1 &
ENGINE=cfm56 "$BIN" "$OUT/ref" a320-takeoff > "$OUT/logs/a320-takeoff.log" 2>&1 &
wait
: > "$OUT/summary.txt"
for s in $SCENES eufi-low-pass-085 a320-takeoff; do cat "$OUT/logs/$s.log" >> "$OUT/summary.txt"; done
cat "$OUT/summary.txt"
n=$(ls "$OUT"/*.wav | wc -l)
echo "$n WAVs in $OUT (28 expected)"
[ "$n" = 28 ]
