#!/bin/sh
# Host compile check: every platform-neutral header compiles on its own,
# control/ and common/ build into a static library against hal/host, and the
# platform-neutral app/ sources (the CLI and mouse logic) compile against it,
# all with no Pico SDK on the include path. Run from tests/ (make -C tests does).
set -eu

CXX=${CXX:-g++}
FLAGS="-std=c++17 -Wall -Wextra -Werror -I../firmware -isystem ../third_party"
OUT=build-host

mkdir -p "$OUT"
rm -f "$OUT"/*.o "$OUT"/libfirmware_host.a

headers=0
for h in ../firmware/control/*.h ../firmware/maze/*.h ../firmware/navigation/*.h \
    ../firmware/common/*.h ../firmware/drivers/*.h ../firmware/driver_lab/*.h \
    ../firmware/app/*.h ../firmware/hal/*.h; do
    [ -e "$h" ] || continue
    echo "#include \"${h#../firmware/}\"" | $CXX $FLAGS -x c++ -fsyntax-only - ||
        { echo "FAIL: $h does not compile on the host"; exit 1; }
    headers=$((headers + 1))
done

sources=0
for s in ../firmware/control/*.cpp ../firmware/common/*.cpp ../firmware/hal/host/*.cpp; do
    [ -e "$s" ] || continue
    o="$OUT/$(echo "${s#../firmware/}" | tr / _).o"
    $CXX $FLAGS -c "$s" -o "$o" ||
        { echo "FAIL: $s does not compile on the host"; exit 1; }
    sources=$((sources + 1))
done
ar rcs "$OUT/libfirmware_host.a" "$OUT"/*.o

# app/ wires drivers and boards together, so its sources only need to compile
# here; app/pico/ holds the board-specific halves and is not built.
apps=0
for s in ../firmware/app/*.cpp; do
    [ -e "$s" ] || continue
    $CXX $FLAGS -fsyntax-only "$s" ||
        { echo "FAIL: $s does not compile on the host"; exit 1; }
    apps=$((apps + 1))
done

echo "PASS: $headers headers, $sources sources and $apps app sources compile against hal/host"
