#ifndef HAL_STM32_RESET_FLAGS_H
#define HAL_STM32_RESET_FLAGS_H

/**
 * @file reset_flags.h
 * @brief Decodes the STM32F405's RCC_CSR reset flags into hal::system::ResetCause.
 *        No libopencm3, so tests/stm32_reset_test.cpp runs it on the host.
 *
 * The flags are sticky across resets until RMVF clears them, so the boot
 * code reads them once, then clears them (hal/stm32/reset.cpp). Every reset
 * source drives NRST low, so PINRSTF rides along with all of them, and a
 * power-up (POR/PDR) also sets BORRSTF (RM0090 6.3.21). The decode keeps
 * one cause: the most specific flag wins.
 */

#include <cstdint>

#include "hal/system.h"

namespace hal::stm32
{
namespace csr
{
constexpr uint32_t BORRSTF  = 1u << 25;
constexpr uint32_t PINRSTF  = 1u << 26;
constexpr uint32_t PORRSTF  = 1u << 27;
constexpr uint32_t SFTRSTF  = 1u << 28;
constexpr uint32_t IWDGRSTF = 1u << 29;
constexpr uint32_t WWDGRSTF = 1u << 30;
constexpr uint32_t LPWRRSTF = 1u << 31;
} // namespace csr

constexpr hal::system::ResetCause decodeResetFlags(uint32_t rcc_csr)
{
    hal::system::ResetCause c{};
    c.raw = rcc_csr;
    if (rcc_csr & csr::IWDGRSTF)
        c.watchdog_timeout = true;
    else if (rcc_csr & csr::WWDGRSTF)
        c.window_watchdog = true;
    else if (rcc_csr & csr::SFTRSTF)
        c.software = true;
    else if (rcc_csr & csr::LPWRRSTF)
        c.low_power = true;
    else if (rcc_csr & csr::PORRSTF)
        c.power_on = true;
    else if (rcc_csr & csr::BORRSTF)
        c.brownout = true;
    else if (rcc_csr & csr::PINRSTF)
        c.run_pin = true;
    return c;
}
} // namespace hal::stm32

#endif // HAL_STM32_RESET_FLAGS_H
