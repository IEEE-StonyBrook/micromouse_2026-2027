#ifndef HAL_STM32_CLOCK_STARTUP_H
#define HAL_STM32_CLOCK_STARTUP_H

/**
 * @file clock_startup.h
 * @brief Bounded HSE start-up and the clock-fault blink pattern. No
 *        libopencm3: the ready flag and the clock are injected, so
 *        tests/stm32_clock_test.cpp runs the state machine on the host.
 */

#include <cstdint>

namespace hal::stm32
{
enum class ClockSource : uint8_t
{
    kHse, ///< External crystal started
    kHsi, ///< Crystal never came up; running from the internal RC
};

/**
 * Polls hse_ready() until it returns true or timeout_cycles of now_cycles()
 * pass. libopencm3's rcc_clock_setup_pll() spins on HSERDY forever, so a
 * missing or dead crystal would hang the board with no sign of life; this
 * bounds the wait and lets the caller fall back to the HSI.
 *
 * now_cycles() is a free-running 32-bit counter (DWT_CYCCNT on target);
 * unsigned subtraction keeps the timeout correct across its wrap.
 */
template <typename ReadyFn, typename NowFn>
ClockSource waitForHse(ReadyFn hse_ready, NowFn now_cycles, uint32_t timeout_cycles)
{
    const uint32_t start = now_cycles();
    while (true)
    {
        if (hse_ready())
            return ClockSource::kHse;
        if (now_cycles() - start >= timeout_cycles)
            return ClockSource::kHsi;
    }
}

/// Clock-fault LED pattern: three 100 ms flashes, then dark until the 2 s period repeats.
constexpr bool clockFaultLedOn(uint32_t uptime_ms)
{
    const uint32_t phase_ms = uptime_ms % 2000;
    return phase_ms < 600 && (phase_ms / 100) % 2 == 0;
}
} // namespace hal::stm32

#endif // HAL_STM32_CLOCK_STARTUP_H
