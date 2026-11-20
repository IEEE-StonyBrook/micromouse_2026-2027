// Host tests for the STM32F405 clock-tree arithmetic and the HSE start-up
// fallback. Expected values are worked from RM0090 (RCC, USART, SPI and
// flash chapters), not from the code.
#include <cstdint>

#include "doctest/doctest.h"
#include "hal/stm32/clock_plan.h"
#include "hal/stm32/clock_startup.h"

using namespace hal::stm32;

TEST_CASE("PLL plan turns the Feather's 12 MHz crystal into exactly 168 MHz and 48 MHz USB")
{
    constexpr PllPlan plan = planPll(12000000, 168000000);
    static_assert(plan.valid, "12 MHz must reach 168 MHz");
    CHECK(plan.sysclk_hz == 168000000);
    CHECK(plan.usb_hz == 48000000);
    // 12 MHz / 7 = 1.714 MHz into the VCO, x196 = 336 MHz, /2 = 168 MHz, /7 = 48 MHz.
    CHECK(plan.m == 7);
    CHECK(plan.n == 196);
    CHECK(plan.p == 2);
    CHECK(plan.q == 7);
    // RM0090 limits: VCO input 1-2 MHz, VCO output 192-432 MHz on the F405.
    CHECK(plan.vco_in_hz >= 1000000);
    CHECK(plan.vco_in_hz <= 2000000);
    CHECK(plan.vco_out_hz == 336000000);
}

TEST_CASE("PLL plan reaches exactly 168 MHz from the 16 MHz HSI fallback")
{
    constexpr PllPlan plan = planPll(16000000, 168000000);
    static_assert(plan.valid, "HSI must reach 168 MHz");
    CHECK(plan.sysclk_hz == 168000000);
    CHECK(plan.usb_hz == 48000000);
    CHECK(plan.m == 10);
    CHECK(plan.n == 210);
}

TEST_CASE("PLL plan refuses a target no legal divider set reaches exactly")
{
    CHECK_FALSE(planPll(12000000, 167000001).valid);
    CHECK_FALSE(planPll(12000000, 200000000).valid); // above the 168 MHz part limit
}

TEST_CASE("Bus plan runs APB1 at 42 MHz and APB2 at 84 MHz, timers at twice that")
{
    constexpr BusPlan bus = planBuses(168000000);
    CHECK(bus.hclk_hz == 168000000);
    CHECK(bus.apb1_div == 4);
    CHECK(bus.apb2_div == 2);
    CHECK(bus.pclk1_hz == 42000000);
    CHECK(bus.pclk2_hz == 84000000);
    // A prescaled APB doubles its timer clock (RM0090 RCC clock tree).
    CHECK(bus.tim_apb1_hz == 84000000);
    CHECK(bus.tim_apb2_hz == 168000000);
}

TEST_CASE("Flash wait states follow RM0090's 2.7-3.6 V table")
{
    CHECK(flashWaitStates(30000000) == 0);
    CHECK(flashWaitStates(30000001) == 1);
    CHECK(flashWaitStates(84000000) == 2);
    CHECK(flashWaitStates(150000000) == 4);
    CHECK(flashWaitStates(168000000) == 5);
}

TEST_CASE("Timer divisors give a 1 MHz count and a 1 kHz tick from 84 MHz")
{
    constexpr TimerDivisors tick = timerDivisors(84000000, 1000000, 1000);
    static_assert(tick.valid, "1 kHz from 84 MHz must divide exactly");
    CHECK(tick.psc == 83);
    CHECK(tick.arr == 999);

    CHECK_FALSE(timerDivisors(84000000, 1000000, 3000).valid); // 333.33 counts
    CHECK_FALSE(timerDivisors(84000000, 5000000, 1000).valid); // 84 / 5 = 16.8
}

TEST_CASE("USART BRR is exact for 1 Mbaud on either APB")
{
    constexpr UsartBrr apb1 = usartBrr(42000000, 1000000);
    CHECK(apb1.brr == 0x2A); // USARTDIV 2.625: mantissa 2, fraction 10/16
    CHECK(apb1.exact);
    constexpr UsartBrr apb2 = usartBrr(84000000, 1000000);
    CHECK(apb2.brr == 0x54); // USARTDIV 5.25
    CHECK(apb2.exact);

    const UsartBrr slow = usartBrr(42000000, 115200);
    CHECK(slow.brr == 365); // 364.58 rounds up
    CHECK_FALSE(slow.exact);
}

TEST_CASE("SPI divider is the smallest power of two that keeps under the device limit")
{
    CHECK(spiDivider(42000000, 10000000) == 8); // 5.25 MHz
    CHECK(spiDivider(42000000, 21000000) == 2);
    CHECK(spiDivider(42000000, 100000) == 256); // clamped at the slowest
}

TEST_CASE("HSE start-up returns the crystal as soon as it reports ready")
{
    uint32_t now   = 0;
    int      polls = 0;
    auto     ready = [&] { return ++polls >= 3; };
    auto     clock = [&] { return now += 100; };
    CHECK(waitForHse(ready, clock, 10000) == ClockSource::kHse);
    CHECK(polls == 3);
}

TEST_CASE("HSE start-up falls back to the HSI once the timeout passes")
{
    uint32_t now   = 0;
    int      polls = 0;
    auto     never = [&]
    {
        ++polls;
        return false;
    };
    auto clock = [&] { return now += 100; };
    CHECK(waitForHse(never, clock, 10000) == ClockSource::kHsi);
    // Bounded: about timeout / step polls, never a hang.
    CHECK(polls >= 99);
    CHECK(polls <= 102);
}

TEST_CASE("HSE start-up timeout survives the cycle counter wrapping")
{
    uint32_t now   = 0xFFFFFF00u;
    int      polls = 0;
    auto     never = [&]
    {
        ++polls;
        return false;
    };
    auto clock = [&] { return now += 100; };
    CHECK(waitForHse(never, clock, 10000) == ClockSource::kHsi);
    CHECK(polls <= 102);
}

TEST_CASE("Clock fault blink flashes three times every two seconds")
{
    int  rising_edges = 0;
    bool last         = false;
    for (uint32_t ms = 0; ms < 2000; ++ms)
    {
        const bool on = clockFaultLedOn(ms);
        if (on && !last)
            ++rising_edges;
        last = on;
    }
    CHECK(rising_edges == 3);
    CHECK(clockFaultLedOn(0) == clockFaultLedOn(2000));
    CHECK_FALSE(clockFaultLedOn(1999));
}
