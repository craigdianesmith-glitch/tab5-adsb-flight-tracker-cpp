#!/bin/sh
# Builds tools/landing_soak/landing_soak.cpp against the repo it is in - or
# SRC=another checkout - into OUT (/tmp/landing_soak). The view's placement
# is cut out of $SRC/src/view3d.cpp here, each build, not copied: prepareNext()
# from its first line down to "int shownKt", and the block from
# "constexpr float STILL_KT" to the end of estimateOf().
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=${SRC:-$(cd "$HERE/../.." && pwd)}
OUT=${OUT:-/tmp/landing_soak}
GEN=$(mktemp -d)
trap 'rm -rf "$GEN"' EXIT
awk '/^void prepareNext\(/{on=1; next} on{print} on && /int shownKt = /{exit}' "$SRC/src/view3d.cpp" > "$GEN/placement.inc"
awk '/^constexpr float STILL_KT/{on=1} on{print} on && /^FollowEstimate estimateOf/{est=1} est && /^}/{exit}' \
    "$SRC/src/view3d.cpp" > "$GEN/still.inc"
grep -q "int shownKt" "$GEN/placement.inc" || { echo "prepareNext() not found in $SRC/src/view3d.cpp"; exit 1; }
grep -q "FollowEstimate estimateOf" "$GEN/still.inc" || { echo "estimateOf() not found in $SRC/src/view3d.cpp"; exit 1; }
g++ -std=c++17 -O2 -w -I"$HERE/stub" -I"$GEN" -I"$SRC/include" "$HERE/landing_soak.cpp" \
    "$SRC/src/follow.cpp" "$SRC/src/telemetry.cpp" "$SRC/src/runways.cpp" "$SRC/src/airports.cpp" \
    "$SRC/src/geoid.cpp" "$SRC/src/ground_route.cpp" "$SRC/src/ground_taxi.cpp" "$SRC/src/ground_follow.cpp" \
    "$SRC/src/airfield_pack.cpp" "$SRC/src/scene3d.cpp" -o "$OUT"
echo "built $OUT from $SRC"
