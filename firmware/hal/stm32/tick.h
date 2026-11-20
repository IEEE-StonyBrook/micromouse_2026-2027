#ifndef HAL_STM32_TICK_H
#define HAL_STM32_TICK_H

#include <cstdint>

/**
 * @brief The STM32 time base and control tick.
 *
 * TIM5 counts microseconds in 32 bits; the tick widens it to 64 (see
 * hal::time::nowUs()). TIM7 interrupts at 1 kHz in every operating mode: it
 * extends the time base, then runs the registered handler. Both timers stop
 * while a debugger halts the core, so a breakpoint does not leave a burst of
 * ticks or a time jump behind it.
 */
namespace hal::stm32::tick
{
constexpr uint32_t TICK_HZ = 1000;

/// Runs in the TIM7 interrupt. No logging, printf or heap (see CLAUDE.md).
using Handler = void (*)();

/// Starts TIM5 and TIM7. Call once, after setupClocks().
void start(Handler handler);

/// Timing of the tick interrupt, in core cycles (168 per microsecond).
struct Stats
{
    uint32_t count;             ///< Ticks since start() or resetStats()
    uint32_t last_cycles;       ///< Length of the latest tick
    uint32_t max_cycles;        ///< Longest tick
    uint32_t min_period_cycles; ///< Shortest entry-to-entry interval
    uint32_t max_period_cycles; ///< Longest entry-to-entry interval
};

/// A consistent copy of the counters, taken with interrupts masked.
Stats stats();

void resetStats();

/// The widened TIM5 count; hal::time::nowUs() calls this.
uint64_t nowUs();
} // namespace hal::stm32::tick

#endif // HAL_STM32_TICK_H
