#!/bin/bash
# The terrain's assembly checked against its C on the PC, without the Tab5.
# The 3D preview is built twice for the Tab5's ESP32-P4 - the C alone, and
# with the assembly (scene3d_terrain.S, scene3d_march.S; -DTERRAIN_ASM) -
# and each is run under qemu-riscv32-static over the same views, and every
# frame's pixels and march set against the other's:
#
#   GROUND=ground.bin AIRFIELDS=airfields.bin tools/terrain_check/qcheck.sh <work dir>
#
# It takes qemu-user-static, PlatformIO's riscv32-esp-elf toolchain, the
# firmware built once and python3 - see build.sh. Exit 0 only if nothing
# differs anywhere in:
#  - the governor's efforts 0 and 0.8 - a sample every two rows, and every
#    four - with bands of 16 and 15 rows at the preview's 1280 by 720;
#  - the Tab5's 1008 by 560 at each band height it opens with: 16 rows with
#    room, 14, 12, 10 and 8 without;
#  - the panel's bytes swapped, and the bands shared by two cores, each on
#    its own caches;
#  - the eye out on the cover, 12km and 28km from the airfield.
# The C runs in the assembly's order (ROW_MAJOR): in column order a sample
# a hair from a part's edge can read the cell on the other side of it.
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
WORK=$(realpath -m "${1:?a work folder}")
: "${GROUND:?GROUND= a ground.bin}" "${AIRFIELDS:?AIRFIELDS= an airfields.bin}"
command -v qemu-riscv32-static > /dev/null || { echo "no qemu-riscv32-static: install qemu-user-static" >&2; exit 2; }
mkdir -p "$WORK"
cd "$ROOT"
("$HERE/build.sh" "$WORK/rv32-c" > "$WORK/build-c.log" 2>&1) &
("$HERE/build.sh" "$WORK/rv32-asm" -DTERRAIN_ASM src/scene3d_terrain.S src/scene3d_march.S > "$WORK/build-asm.log" 2>&1) &
wait
for k in c asm; do
  if [ ! -x "$WORK/rv32-$k" ]; then
    echo "BUILD FAILED ($k):"
    grep -E "error|undefined" "$WORK/build-$k.log" | head -20
    exit 2
  fi
done

TAB5="PREVIEW_W=1008 PREVIEW_H=560"
CONFIGS=(
  "TERRAIN_EFFORT=0 BAND_ROWS=16"
  "TERRAIN_EFFORT=0.8 BAND_ROWS=16"
  "TERRAIN_EFFORT=0 BAND_ROWS=15"
  "TERRAIN_EFFORT=0 BAND_ROWS=16 $TAB5"
  "TERRAIN_EFFORT=0.8 BAND_ROWS=14 $TAB5"
  "TERRAIN_EFFORT=0 BAND_ROWS=12 $TAB5"
  "TERRAIN_EFFORT=0.8 BAND_ROWS=10 $TAB5"
  "TERRAIN_EFFORT=0 BAND_ROWS=8 $TAB5"
  "TERRAIN_EFFORT=0 BAND_ROWS=16 $TAB5 PREVIEW_SWAP=1"
  "TERRAIN_EFFORT=0 BAND_ROWS=16 $TAB5 BAND_SPLIT=1"
  "TERRAIN_EFFORT=0.8 BAND_ROWS=16 $TAB5 BAND_SPLIT=1"
  "TERRAIN_EFFORT=0 BAND_ROWS=16 $TAB5 VIEWS=chase EYE_AT=12000,0,600,0,0,0,60"
  "TERRAIN_EFFORT=0 BAND_ROWS=16 $TAB5 VIEWS=chase EYE_AT=28000,0,1500,0,0,0,40"
)
status=0
n=0
for cfg in "${CONFIGS[@]}"; do
  n=$((n + 1))
  tag=$(printf "cfg%02d" $n)
  (env $cfg ROW_MAJOR=1 "$HERE/run.sh" "qemu-riscv32-static $WORK/rv32-c" "$WORK/$tag-c") &
  (env $cfg "$HERE/run.sh" "qemu-riscv32-static $WORK/rv32-asm" "$WORK/$tag-asm") &
  wait
  echo "== $tag: $cfg (C -> assembly)"
  python3 -I "$HERE/compare.py" "$WORK/$tag-c" "$WORK/$tag-asm" || status=1
done
if [ $status = 0 ]; then
  echo "QCHECK PASS: the assembly matches the C, every pixel and digest"
else
  echo "QCHECK FAIL"
fi
exit $status
