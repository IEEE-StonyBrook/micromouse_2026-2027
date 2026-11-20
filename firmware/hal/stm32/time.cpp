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

void sleepMs(uint32_t duration_ms)
{
    // Wakes on every tick, so the wait overshoots by at most 1 ms.
    const uint64_t end_us = nowUs() + 1000ull * duration_ms;
    while (nowUs() < end_us)
        __asm__ volatile("wfi");
}
} // namespace hal::time
