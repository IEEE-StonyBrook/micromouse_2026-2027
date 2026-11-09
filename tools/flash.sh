#!/usr/bin/env bash
# Configure (if needed), build and flash the firmware.
#
#   tools/flash.sh            competition build, flash
#   tools/flash.sh --bench    bench build (-DBENCH=ON), flash
#   tools/flash.sh --no-flash build only
#
# BENCH is passed on every run so a bench value cached in build/ never leaks
# into a competition flash.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build}"  # override to build elsewhere
BENCH=OFF
FLASH=1
for arg in "$@"; do
    case "$arg" in
        --bench) BENCH=ON ;;
        --no-flash) FLASH=0 ;;
        *) echo "usage: tools/flash.sh [--bench] [--no-flash]" >&2; exit 2 ;;
    esac
done

# A build/ copied or moved from another checkout still points at the old
# source dir ("The source directory ... does not exist"); a non-Ninja cache
# breaks `ninja`. Either way, drop the CMake cache and configure fresh.
CACHE="$BUILD/CMakeCache.txt"
if [ -f "$CACHE" ]; then
    home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$CACHE")"
    gen="$(sed -n 's/^CMAKE_GENERATOR:INTERNAL=//p' "$CACHE")"
    if [ "$home" != "$ROOT" ] || [ "$gen" != "Ninja" ]; then
        echo "flash.sh: build/ is stale (source='$home', generator='$gen'); reconfiguring"
        rm -rf "$CACHE" "$BUILD/CMakeFiles"
    fi
fi

# The Pico SDK, toolchain and picotool come from ~/.pico-sdk via the
# pico-vscode.cmake include in CMakeLists.txt (SDK 2.3.0).
cmake -S "$ROOT" -B "$BUILD" -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBENCH="$BENCH" >/dev/null
ninja -C "$BUILD"

UF2="$BUILD/Jurababa.uf2"
echo "flash.sh: built $UF2 (BENCH=$BENCH)"
[ "$FLASH" = 1 ] || exit 0

PICOTOOL="$HOME/.pico-sdk/picotool/2.3.0/picotool/picotool"
[ -x "$PICOTOOL" ] || PICOTOOL="$(command -v picotool || true)"
if [ -z "$PICOTOOL" ]; then
    echo "flash.sh: picotool not found (expected ~/.pico-sdk/picotool/2.3.0)" >&2
    exit 1
fi

# -f asks running firmware to reboot into BOOTSEL over USB; -x runs it after.
if ! "$PICOTOOL" load -fx "$UF2"; then
    cat >&2 <<'EOF'

flash.sh: no Pico found. Hold BOOTSEL while plugging in USB (an RPI-RP2
drive appears), check the cable carries data, then run this again.
EOF
    exit 1
fi
