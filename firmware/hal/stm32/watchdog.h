#ifndef HAL_STM32_WATCHDOG_H
#define HAL_STM32_WATCHDOG_H

/**
 * @brief The independent watchdog (IWDG): resets the chip when the main loop
 *        stops feeding it for IWDG_MIN_TIMEOUT_MS or more (watchdog_plan.h).
 *
 * Feed it from the main loop only, never from an interrupt: the 1 kHz tick
 * keeps running when the main loop hangs, so a tick-fed watchdog would never
 * fire. Once started it cannot be stopped short of a reset. It is frozen
 * while a debugger halts the core.
 */
namespace hal::stm32::watchdog
{
void start();
void feed();
} // namespace hal::stm32::watchdog

#endif // HAL_STM32_WATCHDOG_H
