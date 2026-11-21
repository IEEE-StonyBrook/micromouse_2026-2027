/**
 * @file reset_report.h
 * @brief The next-boot verdict on why the chip reset: the hardware reset
 *        cause plus the breadcrumb the previous session left, as text lines.
 *
 * Platform-neutral and I/O-free, so tests/reset_report_test.cpp checks it on
 * the host. Breadcrumb::report() prints the lines through whatever the
 * platform has (Log on the Pico, the console on the STM32).
 */
#ifndef COMMON_RESET_REPORT_H
#define COMMON_RESET_REPORT_H

#include <cstdint>

#include "common/breadcrumb.h"
#include "hal/system.h"

enum class ResetSeverity : uint8_t
{
    kInfo,
    kWarning,
    kError,
};

struct ResetReport
{
    struct Line
    {
        ResetSeverity severity;
        char          text[200];
    };
    uint8_t count;
    Line    lines[2];
};

/**
 * @brief Explains a reset in plain English, most specific cause first.
 *
 * Software and watchdog resets, then the reset pin, then power. POR alone
 * cannot separate a cold boot from a dip (the Pico latches it for good), so
 * a surviving breadcrumb means the rail dipped rather than went away. A
 * brownout flag (STM32 BOR) is a power fault even when no breadcrumb survived.
 *
 * @param sag_warn_v rail voltage below which the pack failed to deliver current
 */
ResetReport resetReport(const hal::system::ResetCause& cause, const Breadcrumb::Snapshot& prev,
                        float sag_warn_v);

#endif // COMMON_RESET_REPORT_H
