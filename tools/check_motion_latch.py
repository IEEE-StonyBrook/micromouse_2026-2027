#!/usr/bin/env python3
"""Fail if main-loop code in Motion writes state the control tick owns.

Motion::update() runs from a timer interrupt. The main loop changes the
tick's state only by queueing a command (Motion::issue); the tick applies the
queue at the top of update(). This script checks firmware/control/motion.cpp:
every Motion member function that is not on the tick side (TICK_FUNCTIONS)
must not write a tick-owned member or call a mutating drivetrain method, and
must not call a tick-side helper (which would do the write for it).

Best effort, by regex, with these limits:
- It reads motion.cpp only. Other files can't reach the private members, so
  the compiler covers them, but a public Drivetrain, Profile or PID reached
  through another object is not checked.
- It sees writes by member name: `x_ = ...`, `++x_`, `x_.start(...)`. A write
  through a pointer or reference alias (other than applyCommanded's `state.`
  references, which are its purpose) slips past.
- It does not check that the lock is held; issue() and view() take it.

Usage: python3 tools/check_motion_latch.py   (exit 0 = clean, 1 = violations)
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_isr_logging import matching, strip_comments_and_strings  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
MOTION_CPP = REPO / "firmware/control/motion.cpp"

# Functions that run in the control tick (or, for the constructor and the
# latch's overflow path in issue(), with the tick provably not running).
TICK_FUNCTIONS = {
    "Motion",  # constructor: no tick runs yet
    "update",
    "checkCrash",
    "runPositionControl",
    "applyPendingCommands",
    "applyCommand",
    "resetControlHistory",
    "clearWallSteeringStats",
    "wallSteeringAdjustment",
    "recordWallSteeringStats",
    "headingHoldAdjustment",
}

# Main-loop functions allowed to call a tick-side helper, and which one.
ALLOWED_CALLS = {
    ("issue", "applyPendingCommands"),  # overflow: applied under hal::irq::Lock
}

# Members the tick owns. A trailing * matches a prefix.
TICK_MEMBERS = [
    "forward_",
    "rotation_",
    "forward_controller_",
    "rotation_controller_",
    "prev_left_cmd_vel_mmps_",
    "prev_right_cmd_vel_mmps_",
    "motion_sequence_active_",
    "steering_mode_",
    "encoder_rotation_ticks_",
    "latest_*",
    "side_error_*",
    "wall_steering_*",
    "line_steering_*",
    "heading_hold_*",
    "trace_*",
]

# The latch itself: only issue() and the drain may write it.
LATCH_MEMBERS = ["latch_*"]
LATCH_WRITERS = {"issue", "applyPendingCommands"}

MUTATORS = r"(?:start|reset|stop|update|set\w*|adjust\w*|extend\w*)"
DRIVETRAIN_WRITE = re.compile(r"\bdrivetrain_->(?:stop|reset|setVoltage|update)\s*\(")


def member_regex(names):
    alts = [re.escape(n[:-1]) + r"\w*" if n.endswith("*") else re.escape(n) for n in names]
    return r"(?<![.>\w])(?:%s)" % "|".join(alts)


def write_patterns(names):
    m = member_regex(names)
    return [
        re.compile(m + r"(?:\[[^\]]*\])?\s*(?:[-+*/|&]?=(?!=)|\+\+|--)"),
        re.compile(r"(?:\+\+|--)\s*" + m),
        re.compile(m + r"\." + MUTATORS + r"\s*\("),
    ]


def definitions(src):
    """Yield (name, body_start, body_end) for each Motion:: member definition."""
    for m in re.finditer(r"(?<![\w:])Motion::(~?\w+)\s*\(", src):
        paren = m.end() - 1
        after = matching(src, paren, "(", ")")
        rest = re.match(r"\s*(?:const\s*)?(?:noexcept\s*)?(?::[^{;]*)?\{", src[after:], re.S)
        if rest:
            brace = after + rest.end() - 1
            yield m.group(1), brace, matching(src, brace, "{", "}")


def main():
    src = strip_comments_and_strings(MOTION_CPP.read_text())
    tick_writes = write_patterns(TICK_MEMBERS)
    latch_writes = write_patterns(LATCH_MEMBERS)
    helper_call = re.compile(
        r"(?<![\w.>:])(%s)\s*\(" % "|".join(sorted(TICK_FUNCTIONS - {"Motion", "update"}))
    )

    violations = []
    seen = set()
    for name, start, end in definitions(src):
        seen.add(name)
        body = src[start:end]

        def report(offset, what):
            line = src.count("\n", 0, start + offset) + 1
            violations.append(f"{MOTION_CPP.relative_to(REPO)}:{line}: Motion::{name}: {what}")

        if name not in LATCH_WRITERS:
            for pattern in latch_writes:
                for hit in pattern.finditer(body):
                    report(hit.start(), f"writes the latch ({hit.group(0).strip()})")
        if name in TICK_FUNCTIONS:
            continue
        for pattern in tick_writes:
            for hit in pattern.finditer(body):
                report(hit.start(), f"writes tick-owned state ({hit.group(0).strip()})")
        for hit in DRIVETRAIN_WRITE.finditer(body):
            report(hit.start(), f"drives the drivetrain ({hit.group(0).strip()})")
        for hit in helper_call.finditer(body):
            if (name, hit.group(1)) not in ALLOWED_CALLS:
                report(hit.start(), f"calls tick-side {hit.group(1)}()")

    missing = TICK_FUNCTIONS - seen
    for name in sorted(missing):
        violations.append(f"{MOTION_CPP.relative_to(REPO)}: Motion::{name}: not found (stale list)")

    for v in violations:
        print(v)
    if violations:
        print(f"FAIL: {len(violations)} write(s) to tick-owned Motion state outside the latch")
        return 1
    print("OK: main-loop Motion functions change tick state only through the latch")
    return 0


if __name__ == "__main__":
    sys.exit(main())
