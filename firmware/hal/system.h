#ifndef HAL_SYSTEM_H
#define HAL_SYSTEM_H

#include <cstdint>

namespace hal::system
{
/// Why the chip last reset, as far as the hardware recorded it.
struct ResetCause
{
    uint32_t raw;      ///< Platform reset register, for diagnostics only
    bool     power_on; ///< Power-on reset latched (never cleared on the Pico)
    bool     run_pin;  ///< External reset: RUN pin or reset button
    bool     debugger; ///< Restart requested by a debugger
    bool     watchdog; ///< Software reboot through the watchdog (reflash, bootloader)
};

ResetCause resetCause();
} // namespace hal::system

/**
 * Declares a variable that survives a warm reset: startup code neither zeroes
 * nor initialises it. Use as `Type HAL_NOINIT(name);`. On the host it is an
 * ordinary zero-initialised variable.
 */
#ifdef PICO_BUILD
// Same section the SDK's __uninitialized_ram() uses; its linker script keeps it out of .bss.
#define HAL_NOINIT(name) __attribute__((section(".uninitialized_data." #name))) name
#elif defined(STM32_BUILD)
// hal/stm32/stm32f405.ld places .noinit past _ebss, where startup never writes.
#define HAL_NOINIT(name) __attribute__((section(".noinit." #name))) name
#else
#define HAL_NOINIT(name) name
#endif

/**
 * Places a variable in the STM32's 64 KB core-coupled RAM: zeroed at startup,
 * CPU-only (never a DMA buffer). Use as `Type HAL_CCM(name);`. Nothing is
 * loaded from flash, so a constant non-zero initialiser is lost; constructors
 * still run. Elsewhere it is an ordinary variable.
 */
#ifdef STM32_BUILD
#define HAL_CCM(name) __attribute__((section(".ccm." #name))) name
#else
#define HAL_CCM(name) name
#endif

#endif // HAL_SYSTEM_H
