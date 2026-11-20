#include "common/breadcrumb.h"

#include <cstdio>
#include <cstring>

#include "common/log.h"
#include "common/reset_report.h"

#ifdef PICO_BUILD
#include "config/config.h"
#include "hal/system.h"
#include "hal/time.h"
#endif

namespace
{
#ifdef PICO_BUILD
constexpr uint32_t kMagic = 0x4A555242; // 'JURB'

struct Live
{
    uint32_t magic;
    uint32_t uptime_ms;
    float    voltage_min;
    float    voltage_last;
    char     last_log[48];
};

// Must not land in .bss: crt0 zeroes that, which would erase the history of the
// very reset we are trying to explain.
Live HAL_NOINIT(live_crumb);
#endif
} // namespace

Breadcrumb::Snapshot Breadcrumb::previous_ = {};

const Breadcrumb::Snapshot& Breadcrumb::previous()
{
    return previous_;
}

void Breadcrumb::begin()
{
#ifdef PICO_BUILD
    previous_.valid = (live_crumb.magic == kMagic);
    if (previous_.valid)
    {
        previous_.uptime_ms    = live_crumb.uptime_ms;
        previous_.voltage_min  = live_crumb.voltage_min;
        previous_.voltage_last = live_crumb.voltage_last;
        std::memcpy(previous_.last_log, live_crumb.last_log, sizeof(previous_.last_log));
        previous_.last_log[sizeof(previous_.last_log) - 1] = '\0';
    }

    live_crumb.magic        = kMagic;
    live_crumb.uptime_ms    = 0;
    live_crumb.voltage_min  = 0.0f; // 0 means "no reading taken yet"
    live_crumb.voltage_last = 0.0f;
    live_crumb.last_log[0]  = '\0';
#endif
}

void Breadcrumb::noteVoltage(float volts)
{
#ifdef PICO_BUILD
    live_crumb.voltage_last = volts;
    if (live_crumb.voltage_min <= 0.0f || volts < live_crumb.voltage_min)
        live_crumb.voltage_min = volts;
#else
    (void)volts;
#endif
}

void Breadcrumb::noteActivity(const char* text)
{
#ifdef PICO_BUILD
    if (text == nullptr)
        return;

    std::strncpy(live_crumb.last_log, text, sizeof(live_crumb.last_log) - 1);
    live_crumb.last_log[sizeof(live_crumb.last_log) - 1] = '\0';
    // Stamped here rather than in noteVoltage() so the 500 Hz path never reads the clock.
    live_crumb.uptime_ms = hal::time::nowMs();
#else
    (void)text;
#endif
}

void Breadcrumb::report()
{
#ifdef PICO_BUILD
    const hal::system::ResetCause cause = hal::system::resetCause();
    const Snapshot&               prev  = previous_;
    const ResetReport             r     = resetReport(cause, prev, BATTERY_SAG_WARN_V);
    for (uint8_t i = 0; i < r.count; ++i)
    {
        switch (r.lines[i].severity)
        {
        case ResetSeverity::kInfo:
            LOG_INFO(r.lines[i].text);
            break;
        case ResetSeverity::kWarning:
            LOG_WARNING(r.lines[i].text);
            break;
        case ResetSeverity::kError:
            LOG_ERROR(r.lines[i].text);
            break;
        }
    }

    char line[200];
    std::snprintf(line, sizeof(line),
                  "chip_reset=0x%08lx por=%d run=%d psm=%d wd=%d crumb=%s uptime_ms=%lu "
                  "v_min=%.3f v_last=%.3f",
                  static_cast<unsigned long>(cause.raw), cause.power_on ? 1 : 0,
                  cause.run_pin ? 1 : 0, cause.debugger ? 1 : 0, cause.software ? 1 : 0,
                  prev.valid ? "valid" : "decayed", static_cast<unsigned long>(prev.uptime_ms),
                  prev.voltage_min, prev.voltage_last);
    LOG_DIAG(line);
#endif
}
