#!/bin/bash
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Craig Smith. Overhead's terrain engine: PolyForm Noncommercial 1.0.0
# (LICENSE-POLYFORM.md), not the MIT licence of the rest of the repository.
# The 3D preview (tools/landing3d_preview) built for the Tab5's ESP32-P4 -
# RV32IMAFC, with PlatformIO's riscv32-esp-elf toolchain - to run on the PC
# under qemu-riscv32-static, for qcheck.sh:
#
#   build.sh <output> [-D flags...] [more sources...]
#
# The firmware built once first, for the font the preview takes from M5GFX.
# zlib, which the preview reads the ground pack with, is built for it on
# first use from zlib 1.3.1's source, into ~/.cache/overhead-terrain-check;
# the newlib hooks it runs on Linux with are linux_shim.c's.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
TC=${RISCV_TOOLCHAIN:-$HOME/.platformio/packages/toolchain-riscv32-esp/bin}
CACHE=${XDG_CACHE_HOME:-$HOME/.cache}/overhead-terrain-check
OUT=$(realpath -m "$1")
shift
FLAGS=()
SRCS=()
for a in "$@"; do
  case "$a" in
    -*) FLAGS+=("$a") ;;
    *) SRCS+=("$a") ;;
  esac
done
ARCH=(-march=rv32imafc_zicsr_zifencei -mabi=ilp32f)
FONTS=$ROOT/.pio/libdeps/tab5/M5GFX/src/lgfx/Fonts
if [ ! -d "$FONTS" ]; then  # a worktree: the main checkout's
  FONTS=$(cd "$(git -C "$ROOT" rev-parse --git-common-dir)/.." && pwd)/.pio/libdeps/tab5/M5GFX/src/lgfx/Fonts
fi
[ -d "$FONTS" ] || { echo "no M5GFX fonts at $FONTS: build the firmware once first" >&2; exit 1; }

mkdir -p "$CACHE"
ZLIB=$CACHE/zlib-1.3.1
if [ ! -f "$ZLIB/libz.a" ]; then
  # Only what reading the pack takes: inflating.
  curl -sSL -o "$CACHE/zlib.tar.gz" https://github.com/madler/zlib/releases/download/v1.3.1/zlib-1.3.1.tar.gz
  echo "9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23  $CACHE/zlib.tar.gz" | sha256sum -c --quiet
  tar -xzf "$CACHE/zlib.tar.gz" -C "$CACHE"
  for f in adler32 crc32 inffast inflate inftrees zutil; do
    "$TC/riscv32-esp-elf-gcc" -O2 "${ARCH[@]}" -c "$ZLIB/$f.c" -o "$ZLIB/$f.o"
  done
  (cd "$ZLIB" && "$TC/riscv32-esp-elf-ar" rcs libz.a adler32.o crc32.o inffast.o inflate.o inftrees.o zutil.o)
fi
if [ ! -f "$CACHE/linux_shim.o" ] || [ "$HERE/linux_shim.c" -nt "$CACHE/linux_shim.o" ]; then
  "$TC/riscv32-esp-elf-gcc" -O2 "${ARCH[@]}" -c "$HERE/linux_shim.c" -o "$CACHE/linux_shim.o"
fi

cd "$ROOT"
COMMON=(-O2 -fno-math-errno "${ARCH[@]}" -Iinclude -I"$FONTS" -I"$ZLIB" "${FLAGS[@]}")
OBJ=$(mktemp -d)
trap 'rm -rf "$OBJ"' EXIT
# The preview's main() renamed - the shim's own sets up the environment and
# calls it - and no locks round statics' first use: newlib's want threads.
"$TC/riscv32-esp-elf-g++" -std=gnu++17 -fno-threadsafe-statics "${COMMON[@]}" -Dmain=preview_main \
  -c tools/landing3d_preview/preview.cpp -o "$OBJ/preview.o"
"$TC/riscv32-esp-elf-g++" -std=gnu++17 -fno-threadsafe-statics "${COMMON[@]}" src/scene3d.cpp src/airfield_pack.cpp \
  src/labels3d.cpp src/coast.cpp src/scenery.cpp src/ground_pack.cpp "${SRCS[@]}" "$OBJ/preview.o" \
  "$CACHE/linux_shim.o" -L"$ZLIB" -lz -o "$OUT"
