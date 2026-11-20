// STM32F405 bring-up entry point: blinks the Feather's red LED at 1 Hz, or
// the clock-fault pattern (three quick flashes every 2 s) when the 12 MHz
// crystal failed to start and the core runs from the HSI instead.
//
// There is no console yet, so memory use is left in `memory_usage` for a
// debugger to read.

#include <libopencm3/cm3/systick.h>

#include "config/board_feather.h"
#include "hal/stm32/clock.h"
#include "hal/stm32/gpio.h"
#include "hal/stm32/memory.h"

namespace
{
constexpr uint32_t TICK_HZ       = 1000;
constexpr uint32_t BLINK_HALF_MS = 500;

volatile uint32_t uptime_ms = 0;
} // namespace

// Read with `print memory_usage` in gdb.
volatile hal::stm32::MemoryUsage memory_usage;

extern "C" void sys_tick_handler()
{
    uptime_ms = uptime_ms + 1;
}

int main()
{
    const hal::stm32::ClockSource clock = hal::stm32::setupClocks();
    hal::stm32::gpioOutput(board::LED);

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(hal::stm32::BUSES.hclk_hz / TICK_HZ - 1);
    systick_interrupt_enable();
    systick_counter_enable();

    const uint32_t led_port       = hal::stm32::gpioPort(board::LED);
    const uint16_t led_bit        = hal::stm32::gpioBit(board::LED);
    uint32_t       last_memory_ms = 0;
    while (true)
    {
        __asm__ volatile("wfi");
        const uint32_t now_ms = uptime_ms;
        const bool     led_on = clock == hal::stm32::ClockSource::kHse
                                    ? (now_ms / BLINK_HALF_MS) % 2 == 0
                                    : hal::stm32::clockFaultLedOn(now_ms);
        if (led_on)
            gpio_set(led_port, led_bit);
        else
            gpio_clear(led_port, led_bit);

        if (now_ms - last_memory_ms >= BLINK_HALF_MS)
        {
            last_memory_ms += BLINK_HALF_MS;
            const hal::stm32::MemoryUsage usage = hal::stm32::memoryUsage();
            memory_usage.heap_peak_bytes        = usage.heap_peak_bytes;
            memory_usage.heap_limit_bytes       = usage.heap_limit_bytes;
            memory_usage.stack_peak_bytes       = usage.stack_peak_bytes;
            memory_usage.stack_limit_bytes      = usage.stack_limit_bytes;
        }
    }
}
