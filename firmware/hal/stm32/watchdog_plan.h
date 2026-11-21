#ifndef HAL_STM32_WATCHDOG_PLAN_H
#define HAL_STM32_WATCHDOG_PLAN_H

/**
 * @file watchdog_plan.h
 * @brief The independent watchdog's (IWDG) timing, as plain arithmetic with
 *        no libopencm3, so tests/stm32_reset_test.cpp checks it on the host.
 *
 * The IWDG counts the LSI, which DS8626 allows anywhere from 17 to 47 kHz.
 * libopencm3's iwdg_set_period_ms() assumes 32 kHz, so a "2 s" period could
 * fire after 1.4 s on a fast part. The reload here is sized for the fastest
 * LSI instead: at least 3 s there, about 4.4 s at 32 kHz, at most 8.3 s at
 * 17 kHz. That clears a flash-store save (one 16 KB sector erase, up to
 * 500 ms with every flash fetch stalled) with room to spare.
 */

#include <cstdint>

namespace hal::stm32
{
constexpr uint32_t LSI_MAX_HZ                = 47000;
constexpr uint32_t IWDG_MIN_TIMEOUT_MS       = 3000;
constexpr uint32_t IWDG_PRESCALER            = 256; ///< IWDG_PR = 6
constexpr uint32_t FLASH_SECTOR_ERASE_MAX_MS = 500; ///< DS8626 t_ERASE16KB

/// Smallest reload that reaches IWDG_MIN_TIMEOUT_MS on the fastest LSI.
constexpr uint32_t IWDG_RELOAD =
    (IWDG_MIN_TIMEOUT_MS * LSI_MAX_HZ + IWDG_PRESCALER * 1000 - 1) / (IWDG_PRESCALER * 1000);
static_assert(IWDG_RELOAD <= 0x0FFFu, "IWDG reload is a 12-bit register");

/// Time from the last feed to the reset, for a given LSI frequency.
constexpr double iwdgTimeoutMs(uint32_t lsi_hz)
{
    return IWDG_RELOAD * static_cast<double>(IWDG_PRESCALER) * 1000.0 / lsi_hz;
}
} // namespace hal::stm32

#endif // HAL_STM32_WATCHDOG_PLAN_H
