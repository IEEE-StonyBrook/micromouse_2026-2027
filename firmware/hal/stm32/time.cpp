#include "hal/time.h"

#include <libopencm3/cm3/dwt.h>

#include "hal/stm32/clock.h"
#include "hal/stm32/spin_wait.h"
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

// Both sleeps wake on every tick, so they overshoot by at most 1 ms.
void sleepUntilUs(uint64_t deadline_us)
{
    if (!hal::stm32::tick::running())
    {
        // Before tick::start() the clock reads 0 and no interrupt would end a
        // WFI, so spin on the cycle counter for the time left instead. It
        // assumes the 168 MHz core clock; before setupClocks() the core runs
        // on the 16 MHz HSI and the wait lasts longer, never shorter.
        const uint64_t now_us = nowUs();
        dwt_enable_cycle_counter();
        if (deadline_us > now_us)
            hal::stm32::spinForUs(deadline_us - now_us, dwt_read_cycle_counter,
                                  hal::stm32::BUSES.hclk_hz / 1000000);
        return;
    }
    while (nowUs() < deadline_us)
        __asm__ volatile("wfi");
}

void sleepMs(uint32_t duration_ms)
{
    sleepUntilUs(nowUs() + 1000ull * duration_ms);
}
} // namespace hal::time
