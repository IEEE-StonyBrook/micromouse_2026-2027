#ifndef HAL_STM32_CLOCK_H
#define HAL_STM32_CLOCK_H

#include "hal/stm32/clock_plan.h"
#include "hal/stm32/clock_startup.h"

namespace hal::stm32
{
/// The one clock plan the firmware runs: 168 MHz SYSCLK, APB1 42 MHz, APB2 84 MHz.
constexpr BusPlan BUSES = planBuses(MAX_SYSCLK_HZ);

/**
 * Starts the crystal with a 100 ms timeout, then runs the PLL at 168 MHz from
 * it, or from the HSI if the crystal never came up. Either way the buses end
 * at BUSES; only the accuracy differs (HSI is ±1 % over temperature).
 * Call first thing in main(). Also starts the DWT cycle counter.
 */
ClockSource setupClocks();

/// Source chosen by setupClocks().
ClockSource clockSource();
} // namespace hal::stm32

#endif // HAL_STM32_CLOCK_H
