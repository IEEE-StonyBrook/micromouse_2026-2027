#!/usr/bin/env python3
"""Fail if the STM32 firmware ELF breaks the flash and RAM layout.

STM32F405 flash sectors 0-3 are 16 KB each. The layout reserves:
  - sector 0 (0x08000000) for the vector table only,
  - sectors 1-2 (0x08004000-0x0800BFFF) for the flash store, which must stay
    empty in the image so a reflash never overwrites saved settings,
  - sector 3 onwards (0x0800C000+) for code and constants.
SRAM (0x20000000, 128 KB) holds .data, .bss, .dma, .noinit, heap and stack;
CCM (0x10000000, 64 KB) holds only explicitly marked symbols in .ccm. CCM is
not reachable by DMA and is never loaded from flash.

Reads the section headers with `arm-none-eabi-objdump -h`.

Usage: python3 tools/check_stm32_layout.py <firmware.elf> [objdump]
Exit 0 = layout holds, 1 = violations, 2 = usage or tool error.
"""

import re
import subprocess
import sys

FLASH_START = 0x08000000
STORE_START = 0x08004000  # sector 1
STORE_END = 0x0800C000  # sector 3, exclusive
FLASH_END = 0x08100000  # 1 MB part
SRAM = (0x20000000, 0x20020000)
CCM = (0x10000000, 0x10010000)

# Idx Name Size VMA LMA File-off Algn, then a line of flags.
HEADER_RE = re.compile(
    r"^\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+[0-9a-f]+\s+\S+\s*$"
)


def read_sections(elf, objdump):
    out = subprocess.run(
        [objdump, "-h", elf], check=True, capture_output=True, text=True
    ).stdout
    lines = out.splitlines()
    sections = {}
    for i, line in enumerate(lines):
        m = HEADER_RE.match(line)
        if not m:
            continue
        flags = lines[i + 1] if i + 1 < len(lines) else ""
        sections[m.group(1)] = {
            "size": int(m.group(2), 16),
            "vma": int(m.group(3), 16),
            "lma": int(m.group(4), 16),
            "alloc": "ALLOC" in flags,
            "load": "LOAD" in flags,
        }
    return sections


def within(addr, size, region):
    return region[0] <= addr and addr + size <= region[1]


def check(sections):
    errors = []
    allocated = {n: s for n, s in sections.items() if s["alloc"] and s["size"]}

    vec = sections.get(".isr_vector")
    if vec is None or not vec["size"]:
        errors.append(".isr_vector missing or empty")
    elif vec["vma"] != FLASH_START:
        errors.append(f".isr_vector at {vec['vma']:#010x}, want {FLASH_START:#010x}")
    elif vec["vma"] + vec["size"] > STORE_START:
        errors.append(".isr_vector overflows sector 0")

    text = sections.get(".text")
    if text is None or not text["size"]:
        errors.append(".text missing or empty")
    elif text["vma"] < STORE_END:
        errors.append(f".text at {text['vma']:#010x}, want >= {STORE_END:#010x}")

    for name, s in allocated.items():
        # Flash bytes come from the load address of anything loaded from the image.
        if s["load"]:
            lo, hi = s["lma"], s["lma"] + s["size"]
            if lo < STORE_END and hi > STORE_START:
                errors.append(
                    f"{name} occupies {lo:#010x}-{hi - 1:#010x}, inside the flash store"
                )
            if FLASH_START <= lo and hi > FLASH_END:
                errors.append(f"{name} runs past the end of flash")
            if name != ".isr_vector" and FLASH_START <= lo < STORE_START:
                errors.append(f"{name} is in sector 0, which holds only the vectors")
        if within(s["vma"], s["size"], CCM):
            if name != ".ccm":
                errors.append(f"{name} placed in CCM; only .ccm may live there")
            elif s["load"]:
                errors.append(".ccm must be NOLOAD (nothing copied from flash)")
        if name == ".ccm" and not within(s["vma"], s["size"], CCM):
            errors.append(f".ccm at {s['vma']:#010x}, outside CCM")

    for name in (".data", ".bss", ".dma", ".noinit"):
        s = sections.get(name)
        if s and s["size"] and not within(s["vma"], s["size"], SRAM):
            errors.append(f"{name} at {s['vma']:#010x}, outside SRAM")

    return errors


def main(argv):
    if len(argv) < 2:
        print(__doc__.strip().splitlines()[-2])
        return 2
    objdump = argv[2] if len(argv) > 2 else "arm-none-eabi-objdump"
    try:
        sections = read_sections(argv[1], objdump)
    except (OSError, subprocess.CalledProcessError) as e:
        print(f"ERROR: could not read {argv[1]}: {e}")
        return 2
    errors = check(sections)
    if errors:
        print("STM32 memory layout violations:")
        for e in errors:
            print(f"  {e}")
        return 1
    text = sections[".text"]
    print(
        f"PASS: vectors at {FLASH_START:#010x}, flash store "
        f"{STORE_START:#010x}-{STORE_END - 1:#010x} empty, "
        f".text at {text['vma']:#010x}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
