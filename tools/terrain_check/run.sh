#!/bin/bash
# The terrain check's frames: one preview binary's views, all at once, each
# into a folder of its own - its log holding each frame's hash and march
# digest (TERRAIN, TSTATS and NOSAVE set) - for compare.py:
#
#   run.sh "<runner> <binary>" <output dir>
#
# GROUND= and AIRFIELDS=, the packs, from the environment, as the preview's
# other settings - BAND_ROWS, TERRAIN_EFFORT, PREVIEW_W ...; VIEWS= some of
# the views alone.
BIN=$1
OUT=$2
: "${GROUND:?GROUND= a ground.bin}" "${AIRFIELDS:?AIRFIELDS= an airfields.bin}"
export TERRAIN=1 TSTATS=1 NOSAVE=1 GROUND AIRFIELDS
VIEWS=${VIEWS:-"spotter tower orbit chase cinema dep enr4000 enr10000 enr35000"}
mkdir -p "$OUT"
one() {
  local v=$1 d=$OUT/$1
  rm -rf "$d"
  mkdir -p "$d"
  case $v in
    dep) set -- departure "$d" 1 - day GLA 23 05 ;;
    enr*) export FEET=${v#enr}; set -- enroute "$d" 0.5 - day ;;
    *) set -- "$v" "$d" 0.25 - day GLA 23 05 ;;
  esac
  (cd "$d" && $BIN "$@" > log.txt 2>&1)
}
for v in $VIEWS; do
  one "$v" &
done
wait
