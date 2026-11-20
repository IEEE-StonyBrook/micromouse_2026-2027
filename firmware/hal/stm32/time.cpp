#include "hal/time.h"

#include "hal/stm32/tick.h"

namespace hal::time
{
uint64_t nowUs()
{
    return hal::stm32::tick::nowUs();
}

uint32_t nowMs()
{
    return static_cast<uint32_t>(nowUs() / 1000);
}

// Both sleeps wake on every tick, so they overshoot by at most 1 ms. They
// need the tick running (tick::start()); before that the clock reads 0.
void sleepUntilUs(uint64_t deadline_us)
{
    while (nowUs() < deadline_us)
        __asm__ volatile("wfi");
}

void sleepMs(uint32_t duration_ms)
{
    sleepUntilUs(nowUs() + 1000ull * duration_ms);
}
} // namespace hal::time
