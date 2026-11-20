// STM32F405 bring-up entry point: blinks the Feather's red LED at 1 Hz, or
// the clock-fault pattern (three quick flashes every 2 s) when the 12 MHz
// crystal failed to start and the core runs from the HSI instead.
//
// There is no console yet, so memory use and tick timing are left in
// `memory_usage` and `tick_stats` for a debugger to read.

#include <libopencm3/stm32/gpio.h>

#include "config/board_feather.h"
#include "hal/stm32/clock.h"
#include "hal/stm32/gpio.h"
#include "hal/stm32/memory.h"
#include "hal/stm32/tick.h"
#include "hal/time.h"

namespace
{
constexpr uint32_t BLINK_HALF_MS = 500;
} // namespace

// Read with `print memory_usage` and `print tick_stats` in gdb.
volatile hal::stm32::MemoryUsage memory_usage;
volatile hal::stm32::tick::Stats tick_stats;

int main()
{
    const hal::stm32::ClockSource clock = hal::stm32::setupClocks();
    hal::stm32::gpioOutput(board::LED);
    hal::stm32::tick::start(nullptr);

    const uint32_t led_port       = hal::stm32::gpioPort(board::LED);
    const uint16_t led_bit        = hal::stm32::gpioBit(board::LED);
    uint32_t       last_report_ms = 0;
    while (true)
    {
        __asm__ volatile("wfi");
        const uint32_t now_ms = hal::time::nowMs();
        const bool     led_on = clock == hal::stm32::ClockSource::kHse
                                    ? (now_ms / BLINK_HALF_MS) % 2 == 0
                                    : hal::stm32::clockFaultLedOn(now_ms);
        if (led_on)
            gpio_set(led_port, led_bit);
        else
            gpio_clear(led_port, led_bit);

        if (now_ms - last_report_ms >= BLINK_HALF_MS)
        {
            last_report_ms += BLINK_HALF_MS;
            const hal::stm32::MemoryUsage usage = hal::stm32::memoryUsage();
            memory_usage.heap_peak_bytes        = usage.heap_peak_bytes;
            memory_usage.heap_limit_bytes       = usage.heap_limit_bytes;
            memory_usage.stack_peak_bytes       = usage.stack_peak_bytes;
            memory_usage.stack_limit_bytes      = usage.stack_limit_bytes;

            const hal::stm32::tick::Stats ticks = hal::stm32::tick::stats();
            tick_stats.count                    = ticks.count;
            tick_stats.last_cycles              = ticks.last_cycles;
            tick_stats.max_cycles               = ticks.max_cycles;
            tick_stats.min_period_cycles        = ticks.min_period_cycles;
            tick_stats.max_period_cycles        = ticks.max_period_cycles;
        }
    }
}
