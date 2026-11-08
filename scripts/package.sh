#!/bin/sh
# Assembles dist/Tools/tg5040/BrickRack.pak from build-device/ and pak/.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PAK="$ROOT/dist/Tools/tg5040/BrickRack.pak"
rm -rf "$PAK"
mkdir -p "$PAK"
cp "$ROOT/build-device/brickrack" "$PAK/brickrack"
cp "$ROOT/pak/launch.sh" "$ROOT/pak/pak.json" "$PAK/"
cp "$ROOT/README.md" "$PAK/README.md" 2>/dev/null || true
cp "$ROOT/LICENSE" "$PAK/LICENSE"
cp "$ROOT/THIRD_PARTY_NOTICES" "$PAK/THIRD_PARTY_NOTICES"
cp "$ROOT/third_party/fonts/OFL.txt" "$PAK/OFL.txt"
mkdir -p "$PAK/patches"
cp "$ROOT"/tests/patches/*.brp "$PAK/patches/" 2>/dev/null || true
chmod +x "$PAK/launch.sh" "$PAK/brickrack"
(cd "$ROOT/dist" && rm -f BrickRack-tg5040.zip && zip -qr BrickRack-tg5040.zip Tools)
echo "packaged $PAK"
echo "zip: $ROOT/dist/BrickRack-tg5040.zip"
