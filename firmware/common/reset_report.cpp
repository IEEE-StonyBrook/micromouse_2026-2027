#include "common/reset_report.h"

#include <cstdarg>
#include <cstdio>

namespace
{
void add(ResetReport& r, ResetSeverity severity, const char* format, ...)
    __attribute__((format(printf, 3, 4)));

void add(ResetReport& r, ResetSeverity severity, const char* format, ...)
{
    if (r.count >= sizeof r.lines / sizeof r.lines[0])
        return;
    ResetReport::Line& line = r.lines[r.count++];
    line.severity           = severity;
    va_list args;
    va_start(args, format);
    std::vsnprintf(line.text, sizeof line.text, format, args);
    va_end(args);
}
} // namespace

ResetReport resetReport(const hal::system::ResetCause& cause, const Breadcrumb::Snapshot& prev,
                        float sag_warn_v)
{
    ResetReport       r{};
    const char*       activity = (prev.last_log[0] != '\0') ? prev.last_log : "(nothing logged yet)";
    const unsigned long up_ms  = static_cast<unsigned long>(prev.uptime_ms);
    const double      v_min    = static_cast<double>(prev.voltage_min);
    const char*       fault    = cause.brownout ? "POWER FAULT (brownout)" : "POWER FAULT";
    constexpr auto    kInfo    = ResetSeverity::kInfo;
    constexpr auto    kWarning = ResetSeverity::kWarning;
    constexpr auto    kError   = ResetSeverity::kError;

    if (cause.debugger)
        add(r, kWarning, "Reset cause: debugger restart.");
    else if (cause.software)
        add(r, kInfo, "Reset cause: software reboot (reflash or bootloader). Not a power fault.");
    else if (cause.watchdog_timeout)
    {
        if (prev.valid)
            add(r, kError, "WATCHDOG RESET: the main loop hung during: %s (begun at %lu ms)",
                activity, up_ms);
        else
            add(r, kError, "WATCHDOG RESET: the main loop hung; no breadcrumb survived.");
        add(r, kError, "  -> The firmware stopped feeding the watchdog for at least 3 s.");
    }
    else if (cause.window_watchdog)
        add(r, kError, "Reset cause: window watchdog.");
    else if (cause.low_power)
        add(r, kWarning, "Reset cause: low-power management reset.");
    else if (cause.run_pin)
        add(r, kWarning, "Reset cause: external reset (RUN pin or reset button).");
    else if (!prev.valid && cause.brownout)
    {
        add(r, kError, "%s: the supply fell below the brownout threshold; no breadcrumb survived.",
            fault);
        add(r, kError, "  -> Check the battery charge and the power wiring.");
    }
    else if (!prev.valid)
        add(r, kInfo, "Reset cause: cold power-up. No history from a previous run.");
    else if (prev.voltage_min <= 0.0f)
    {
        add(r, kError, "%s: reset after %lu ms, during: %s", fault, up_ms, activity);
        add(r, kError, "  -> No battery readings were taken, so the rail cannot be assessed.");
    }
    else if (prev.voltage_min < sag_warn_v)
    {
        add(r, kError,
            "%s: the battery rail collapsed under load. It fell to %.2f V after %lu ms, during: %s",
            fault, v_min, up_ms, activity);
        add(r, kError, "  -> The pack cannot deliver stall current. Check its charge and health, "
                       "and the motor power wiring.");
    }
    else
    {
        add(r, kError, "%s: reset while the rail still read %.2f V, after %lu ms, during: %s",
            fault, v_min, up_ms, activity);
        add(r, kError, "  -> Voltage never sagged, so suspect an intermittent power or ground "
                       "connection rather than the battery.");
    }
    return r;
}
