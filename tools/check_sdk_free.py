#!/usr/bin/env python3
"""Fail if platform-neutral firmware code includes a Pico SDK or libopencm3 header.

Every file in control/, maze/, navigation/, common/, hal/ and hal/host/, plus
the driver and DriverLab headers, must compile without the Pico SDK so they
can build against hal/host. Platform code lives in hal/pico/, hal/stm32/,
the driver .cpp files and the main_*.cpp entry points.

Includes are followed transitively through files in firmware/ and
third_party/, so a header that pulls the SDK in through a third-party header
is caught too. The scan is textual: an include inside `#ifdef PICO_BUILD`
still counts, because the rule is about the source, not one configuration.

Usage: python3 tools/check_sdk_free.py   (exit 0 = clean, 1 = violations)
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
FIRMWARE = REPO / "firmware"
SEARCH_ROOTS = [FIRMWARE, REPO / "third_party"]

# Every file in these directories must be SDK-free.
WHOLE_DIRS = ["control", "maze", "navigation", "common", "hal", "hal/host"]
# Only the headers in these directories must be SDK-free.
HEADER_DIRS = ["drivers", "driver_lab"]

SDK_PREFIXES = ("pico/", "hardware/", "boards/", "tusb", "libopencm3/", "libopencmsis/")
SDK_FILES = {"pico.h"}

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]', re.MULTILINE)
SOURCE_SUFFIXES = {".h", ".hpp", ".cpp", ".c"}


def is_sdk(name):
    return name.startswith(SDK_PREFIXES) or name in SDK_FILES


def targets():
    files = []
    for d in WHOLE_DIRS:
        root = FIRMWARE / d
        if root.is_dir():
            files += [p for p in root.iterdir() if p.suffix in SOURCE_SUFFIXES]
    for d in HEADER_DIRS:
        files += list((FIRMWARE / d).glob("*.h"))
    return sorted(files)


def resolve(name, including_file):
    for base in [including_file.parent] + SEARCH_ROOTS:
        candidate = (base / name).resolve()
        if candidate.is_file():
            return candidate
    return None


def sdk_chain(path, seen):
    """Return the include chain to the first SDK header reachable from path."""
    if path in seen:
        return None
    seen.add(path)
    for name in INCLUDE_RE.findall(path.read_text(errors="replace")):
        if is_sdk(name):
            return [name]
        child = resolve(name, path)
        if child is not None:
            chain = sdk_chain(child, seen)
            if chain:
                return [str(child.relative_to(REPO))] + chain
    return None


def main():
    violations = []
    for path in targets():
        chain = sdk_chain(path, set())
        if chain:
            violations.append(" -> ".join([str(path.relative_to(REPO))] + chain))
    if violations:
        print("Platform SDK includes in platform-neutral code:")
        for v in violations:
            print(f"  {v}")
        return 1
    print(f"PASS: {len(targets())} files free of platform SDK includes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
