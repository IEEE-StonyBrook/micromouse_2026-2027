#!/usr/bin/env python3
"""Fail if code reachable from the control interrupt logs or touches the heap.

The 500 Hz control tick (Motion::update, run from a hardware-timer alarm in
main_pico.cpp) and the IMU UART interrupt must not format strings or allocate:
LOG_* builds an std::ostringstream, which allocates and can block on stdio.
This script checks the body of every function listed in ISR_FUNCTIONS for
forbidden calls. Add a function here when the control path starts calling it.

Usage: python3 tools/check_isr_logging.py   (exit 0 = clean, 1 = violations)
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# (file, function) pairs reachable from an interrupt. A function name checks
# its definition's body; "call:<name>" checks the argument list of every call
# to <name> (used for the lambda handed to the repeating timer).
ISR_FUNCTIONS = [
    ("firmware/main_pico.cpp", "call:add_repeating_timer_us"),
    ("firmware/control/motion.cpp", "Motion::update"),
    ("firmware/control/motion.cpp", "Motion::runPositionControl"),
    ("firmware/control/motion.cpp", "Motion::resetControlHistory"),
    ("firmware/control/motion.cpp", "Motion::resetWallSteeringStats"),
    ("firmware/control/motion.cpp", "Motion::wallSteeringAdjustment"),
    ("firmware/control/motion.cpp", "Motion::recordWallSteeringStats"),
    ("firmware/control/motion.cpp", "Motion::headingHoldAdjustment"),
    ("firmware/control/motion.cpp", "Motion::yaw_deg"),
    ("firmware/control/run_log.cpp", "RunLog::due"),
    ("firmware/control/run_log.cpp", "RunLog::record"),
    ("firmware/control/run_log.cpp", "RunLog::pack"),
    ("firmware/control/run_log.cpp", "packField"),
    ("firmware/control/profile.cpp", "Profile::update"),
    ("firmware/control/profile.cpp", "Profile::brakingDistance"),
    ("firmware/control/profile.cpp", "Profile::remaining"),
    ("firmware/control/profile.cpp", "Profile::carryingVelocity"),
    ("firmware/control/pid.cpp", "PID::update"),
    ("firmware/control/pid.cpp", "PID::reset"),
    ("firmware/control/drivetrain.cpp", "Drivetrain::update"),
    ("firmware/control/drivetrain.cpp", "Drivetrain::feedforward"),
    ("firmware/control/drivetrain.cpp", "Drivetrain::setVoltage"),
    ("firmware/control/drivetrain.cpp", "Drivetrain::batteryVoltage"),
    ("firmware/control/drivetrain.cpp", "Drivetrain::velocity"),
    ("firmware/control/drivetrain.cpp", "Drivetrain::stop"),
    ("firmware/drivers/motor.cpp", "Motor::set_motor_volts"),
    ("firmware/drivers/motor.cpp", "Motor::set_motor_pwm"),
    ("firmware/drivers/motor.cpp", "Motor::stop"),
    ("firmware/drivers/encoder.cpp", "Encoder::ticks"),
    ("firmware/drivers/battery.cpp", "Battery::voltage"),
    ("firmware/drivers/battery.cpp", "Battery::voltsFromAdc"),
    ("firmware/drivers/imu.cpp", "IMU::update"),
    ("firmware/drivers/imu.cpp", "IMU::robot_angle"),
    ("firmware/drivers/imu.cpp", "IMU::robot_omega"),
    ("firmware/drivers/imu.cpp", "IMU::robot_rot_change"),
    ("firmware/drivers/imu.cpp", "IMU::uart_interrupt_handler"),
    ("firmware/drivers/imu.cpp", "IMU::process_receive_data"),
    ("firmware/drivers/imu.cpp", "IMU::parse_packet_and_extract_yaw"),
    ("firmware/common/tof_wall_utils.h", "evaluate"),
    ("firmware/common/tof_wall_utils.h", "steeringAdjustmentDegps"),
    ("firmware/common/utils.h", "wrapAngle180"),
    ("firmware/common/utils.h", "clampAbs"),
    ("firmware/hal/stm32/tick.cpp", "tim7_isr"),
    ("firmware/hal/stm32/console.cpp", "dma1_stream3_isr"),
    ("firmware/hal/stm32/console.cpp", "usart3_isr"),
    ("firmware/hal/stm32/console.cpp", "kickTx"),
    ("firmware/main_stm32.cpp", "onTick"),
    ("firmware/hal/stm32/quad_counter.cpp", "QuadCounter::update"),
    ("firmware/drivers/lsm6dsr.cpp", "Lsm6dsr::readFifo"),
    ("firmware/drivers/lsm6dsr.cpp", "decodeFifoWord"),
    ("firmware/hal/stm32/spi.cpp", "SpiBus::transfer"),
]

FORBIDDEN = [
    (r"\bLOG_[A-Z]+\b", "LOG_ macro"),
    (r"\bLog::", "Log call"),
    (r"\b(?:v?s?n?printf|puts|putchar)\s*\(", "stdio print"),
    (r"\bstd::(?:cout|cerr|clog)\b", "iostream"),
    (r"\b(?:o|i)?stringstream\b", "stringstream"),
    (r"\bstd::(?:string|to_string)\b", "std::string"),
    (r"\bnew\b", "operator new"),
    (r"\bdelete\b", "operator delete"),
    (r"\b(?:malloc|calloc|realloc|free)\s*\(", "C heap"),
    (r"\.(?:push_back|emplace_back|resize|reserve|insert|append)\s*\(", "container growth"),
]


def strip_comments_and_strings(src):
    """Blank comments and string literals, keeping offsets and newlines."""
    pattern = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"', re.S)
    return pattern.sub(lambda m: re.sub(r"[^\n]", " ", m.group(0)), src)


def matching(src, start, open_ch, close_ch):
    """Index just past the bracket that closes the one at src[start]."""
    depth = 0
    for i in range(start, len(src)):
        if src[i] == open_ch:
            depth += 1
        elif src[i] == close_ch:
            depth -= 1
            if depth == 0:
                return i + 1
    raise ValueError("unbalanced brackets")


def spans_for(src, name):
    """(start, end) offsets of each span to check for one manifest entry."""
    if name.startswith("call:"):
        callee = name[len("call:"):]
        spans = []
        for m in re.finditer(r"\b%s\s*\(" % re.escape(callee), src):
            paren = m.end() - 1
            spans.append((paren, matching(src, paren, "(", ")")))
        return spans

    # A definition: name(...) [const] [noexcept] { ... }, not a call via . or ->.
    for m in re.finditer(r"(?<![.>\w:])%s\s*\(" % re.escape(name), src):
        paren = m.end() - 1
        after = matching(src, paren, "(", ")")
        rest = re.match(r"\s*(?:const\s*)?(?:noexcept\s*)?(?:override\s*)?\{", src[after:])
        if rest:
            brace = after + rest.end() - 1
            return [(brace, matching(src, brace, "{", "}"))]
    return []


def main():
    violations = []
    for rel, name in ISR_FUNCTIONS:
        src = strip_comments_and_strings((REPO / rel).read_text())
        spans = spans_for(src, name)
        if not spans:
            violations.append(f"{rel}: {name}: definition not found (stale manifest)")
            continue
        for start, end in spans:
            body = src[start:end]
            for regex, label in FORBIDDEN:
                for hit in re.finditer(regex, body):
                    line = src.count("\n", 0, start + hit.start()) + 1
                    violations.append(f"{rel}:{line}: {name}: {label} ({hit.group(0).strip()})")

    for v in violations:
        print(v)
    if violations:
        print(f"FAIL: {len(violations)} forbidden call(s) reachable from an interrupt")
        return 1
    print(f"OK: {len(ISR_FUNCTIONS)} ISR-reachable functions are free of logging and heap use")
    return 0


if __name__ == "__main__":
    sys.exit(main())
