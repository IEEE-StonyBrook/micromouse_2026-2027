#!/bin/sh
# Host compile check: every platform-neutral header compiles on its own, and
# control/ and common/ build into a static library against hal/host, with no
# Pico SDK on the include path. Run from tests/ (make -C tests does).
set -eu

CXX=${CXX:-g++}
FLAGS="-std=c++17 -Wall -Wextra -Werror -I../firmware -isystem ../third_party"
OUT=build-host

mkdir -p "$OUT"
rm -f "$OUT"/*.o "$OUT"/libfirmware_host.a

headers=0
for h in ../firmware/control/*.h ../firmware/maze/*.h ../firmware/navigation/*.h \
    ../firmware/common/*.h ../firmware/drivers/*.h ../firmware/driver_lab/*.h \
    ../firmware/hal/*.h; do
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

echo "PASS: $headers headers and $sources sources compile against hal/host"
