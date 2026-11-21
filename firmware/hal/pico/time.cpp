#include "hal/time.h"

#include "pico/time.h"

namespace hal::time
{
uint32_t nowMs()
{
    return to_ms_since_boot(get_absolute_time());
}

void sleepMs(uint32_t duration_ms)
{
    sleep_ms(duration_ms);
}

uint64_t nowUs()
{
    return to_us_since_boot(get_absolute_time());
}

void sleepUntilUs(uint64_t deadline_us)
{
    sleep_until(from_us_since_boot(deadline_us));
}
} // namespace hal::time
