// Host tests for the STM32F405 reset-cause decode and the independent
// watchdog's timing. Bit positions are RM0090 6.3.21 (RCC_CSR); the LSI range
// and erase time are DS8626 (LSI 17-47 kHz, t_ERASE16KB up to 500 ms).
#include <cstdint>

#include "doctest/doctest.h"
#include "hal/stm32/reset_flags.h"
#include "hal/stm32/watchdog_plan.h"

using namespace hal::stm32;

namespace
{
// RCC_CSR reset flags, written out from RM0090 rather than taken from the code.
constexpr uint32_t BOR  = 1u << 25;
constexpr uint32_t PIN  = 1u << 26;
constexpr uint32_t POR  = 1u << 27;
constexpr uint32_t SFT  = 1u << 28;
constexpr uint32_t IWDG = 1u << 29;
constexpr uint32_t WWDG = 1u << 30;
constexpr uint32_t LPWR = 1u << 31;

int flagsSet(const hal::system::ResetCause& c)
{
    return c.power_on + c.run_pin + c.debugger + c.software + c.brownout + c.watchdog_timeout +
           c.window_watchdog + c.low_power;
}
} // namespace

TEST_CASE("A power-up sets POR, BOR and PIN and decodes as power-on only")
{
    const auto c = decodeResetFlags(POR | BOR | PIN | 0x3u); // LSION/LSIRDY ignored
    CHECK(c.raw == (POR | BOR | PIN | 0x3u));
    CHECK(c.power_on);
    CHECK(flagsSet(c) == 1);
}

TEST_CASE("A brownout sets BOR and PIN and decodes as a brownout")
{
    const auto c = decodeResetFlags(BOR | PIN);
    CHECK(c.brownout);
    CHECK(flagsSet(c) == 1);
}

TEST_CASE("PIN alone is the reset button")
{
    const auto c = decodeResetFlags(PIN);
    CHECK(c.run_pin);
    CHECK(flagsSet(c) == 1);
}

TEST_CASE("Internal resets also pulse NRST, so PIN does not count as the button")
{
    CHECK(decodeResetFlags(IWDG | PIN).watchdog_timeout);
    CHECK(flagsSet(decodeResetFlags(IWDG | PIN)) == 1);
    CHECK(decodeResetFlags(WWDG | PIN).window_watchdog);
    CHECK(flagsSet(decodeResetFlags(WWDG | PIN)) == 1);
    CHECK(decodeResetFlags(SFT | PIN).software);
    CHECK(flagsSet(decodeResetFlags(SFT | PIN)) == 1);
    CHECK(decodeResetFlags(LPWR | PIN).low_power);
    CHECK(flagsSet(decodeResetFlags(LPWR | PIN)) == 1);
}

TEST_CASE("No flags decode to nothing")
{
    CHECK(flagsSet(decodeResetFlags(0)) == 0);
}

TEST_CASE("The independent watchdog waits at least 2 s even on the fastest LSI")
{
    // IWDG timeout = reload * prescaler / f_LSI (RM0090 19.3). The LSI is
    // 17-47 kHz across parts and temperature (DS8626).
    const double fastest_s = IWDG_RELOAD * double(IWDG_PRESCALER) / 47000.0;
    const double slowest_s = IWDG_RELOAD * double(IWDG_PRESCALER) / 17000.0;
    CHECK(fastest_s >= 2.0);
    CHECK(slowest_s <= 10.0); // a real hang still resets within seconds
    CHECK(IWDG_RELOAD <= 0x0FFFu); // 12-bit reload register
    CHECK(iwdgTimeoutMs(47000) == doctest::Approx(fastest_s * 1000.0).epsilon(0.001));
}

TEST_CASE("A flash-store save, fed just before its erase, cannot outlast the watchdog")
{
    // One save erases one 16 KB sector (up to 500 ms, DS8626 t_ERASE16KB) and
    // programs under a KB of words; the timeout must clear it with room to spare.
    CHECK(iwdgTimeoutMs(47000) >= 2 * FLASH_SECTOR_ERASE_MAX_MS);
}
