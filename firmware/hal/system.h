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
#else
#define HAL_NOINIT(name) name
#endif

#endif // HAL_SYSTEM_H
