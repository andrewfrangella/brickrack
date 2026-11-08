#!/bin/sh
# BrickRack launcher for NextUI (tg5040: TrimUI Brick / Brick Pro / Smart Pro).
PAK_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

export PLATFORM="${PLATFORM:-tg5040}"
export SDCARD_PATH="${SDCARD_PATH:-/mnt/SDCARD}"
export SYSTEM_PATH="${SYSTEM_PATH:-$SDCARD_PATH/.system/$PLATFORM}"
export USERDATA_PATH="${USERDATA_PATH:-$SDCARD_PATH/.userdata/$PLATFORM}"
export LOGS_PATH="${LOGS_PATH:-$USERDATA_PATH/logs}"

# Patches live outside the pak so updates never touch them.
export BRICKRACK_HOME="$USERDATA_PATH/BrickRack"
mkdir -p "$BRICKRACK_HOME/patches" "$LOGS_PATH"
# Seed example patches on first run.
for p in "$PAK_DIR"/patches/*.brp; do
  [ -f "$p" ] || continue
  dst="$BRICKRACK_HOME/patches/example-$(basename "$p")"
  [ -e "$dst" ] || cp "$p" "$dst"
done

LOG="$LOGS_PATH/BrickRack.txt"

# Audio DSP wants the CPU at full speed; restore the previous governor on exit.
GOV=/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
MINF=/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
SAVED_GOV="$(cat "$GOV" 2>/dev/null)"
SAVED_MIN="$(cat "$MINF" 2>/dev/null)"
restore() {
  [ -n "$SAVED_GOV" ] && echo "$SAVED_GOV" > "$GOV" 2>/dev/null
  [ -n "$SAVED_MIN" ] && echo "$SAVED_MIN" > "$MINF" 2>/dev/null
}
trap restore EXIT INT TERM
echo performance > "$GOV" 2>/dev/null || true
echo 1608000 > "$MINF" 2>/dev/null || true

export LD_LIBRARY_PATH="/usr/trimui/lib:$SYSTEM_PATH/lib:/usr/lib:/lib:$LD_LIBRARY_PATH"
export PATH="/usr/trimui/bin:$SYSTEM_PATH/bin:$PATH"

cd "$PAK_DIR" || exit 1
echo "=== BrickRack $(date) ===" > "$LOG"
"$PAK_DIR/brickrack" "$@" >> "$LOG" 2>&1
RC=$?
echo "exit code $RC" >> "$LOG"
exit $RC
