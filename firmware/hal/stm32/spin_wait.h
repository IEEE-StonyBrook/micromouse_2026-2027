#ifndef HAL_STM32_SPIN_WAIT_H
#define HAL_STM32_SPIN_WAIT_H

/**
 * @file spin_wait.h
 * @brief A busy-wait on a free-running 32-bit cycle counter (DWT_CYCCNT on
 *        target). No libopencm3: the counter is injected, so
 *        tests/stm32_spin_wait_test.cpp runs it on the host.
 */

#include <cstdint>

namespace hal::stm32
{
/**
 * Spins until at least duration_us * cycles_per_us cycles of now_cycles()
 * pass. Unsigned 32-bit deltas are summed into 64 bits, so the counter may
 * wrap any number of times as long as it is read at least once per wrap.
 */
template <typename NowFn>
void spinForUs(uint64_t duration_us, NowFn now_cycles, uint32_t cycles_per_us)
{
    const uint64_t target  = duration_us * cycles_per_us;
    uint64_t       elapsed = 0;
    uint32_t       last    = target > 0 ? now_cycles() : 0;
    while (elapsed < target)
    {
        const uint32_t now = now_cycles();
        elapsed += static_cast<uint32_t>(now - last);
        last = now;
    }
}
} // namespace hal::stm32

#endif // HAL_STM32_SPIN_WAIT_H
