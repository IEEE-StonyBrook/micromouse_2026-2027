// STM32F405 bring-up entry point: blinks the Feather's red LED at 1 Hz.
//
// Runs on the 16 MHz HSI the chip resets into; the PLL and HSE come with the
// clock bring-up. There is no console yet, so memory use is left in
// `memory_usage` for a debugger to read.

#include <libopencm3/cm3/systick.h>

#include "hal/stm32/board_feather.h"
#include "hal/stm32/memory.h"

namespace
{
constexpr uint32_t HSI_HZ        = 16000000;
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
    rcc_periph_clock_enable(board::LED_RCC);
    gpio_mode_setup(board::LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, board::LED_PIN);

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(HSI_HZ / TICK_HZ - 1);
    systick_interrupt_enable();
    systick_counter_enable();

    uint32_t last_toggle_ms = 0;
    while (true)
    {
        __asm__ volatile("wfi");
        if (uptime_ms - last_toggle_ms >= BLINK_HALF_MS)
        {
            last_toggle_ms += BLINK_HALF_MS;
            gpio_toggle(board::LED_PORT, board::LED_PIN);

            const hal::stm32::MemoryUsage usage = hal::stm32::memoryUsage();
            memory_usage.heap_peak_bytes        = usage.heap_peak_bytes;
            memory_usage.heap_limit_bytes       = usage.heap_limit_bytes;
            memory_usage.stack_peak_bytes       = usage.stack_peak_bytes;
            memory_usage.stack_limit_bytes      = usage.stack_limit_bytes;
        }
    }
}
