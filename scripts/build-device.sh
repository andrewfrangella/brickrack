#!/bin/sh
# Builds BrickRack for TrimUI Brick / Brick Pro (NextUI, tg5040) inside Docker
# and packages dist/Tools/tg5040/BrickRack.pak (copy the Tools folder to the SD card).
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=brickrack-tg5040
# Cached after the first run; rebuilds automatically when the Dockerfile changes.
docker build -q -t "$IMAGE" "$ROOT/docker" >/dev/null
docker run --rm --platform linux/arm64 -v "$ROOT":/src -w /src "$IMAGE" sh -c '
  cmake -S . -B build-device -DCMAKE_BUILD_TYPE=Release -DBRICKRACK_DEVICE=ON >/dev/null &&
  cmake --build build-device -j"$(nproc)" --target brickrack &&
  strip build-device/brickrack &&
  file build-device/brickrack'
"$ROOT/scripts/package.sh"
