#ifndef HAL_STM32_CLOCK_PLAN_H
#define HAL_STM32_CLOCK_PLAN_H

/**
 * @file clock_plan.h
 * @brief STM32F405 clock-tree arithmetic: PLL, bus prescalers, flash wait
 *        states and peripheral divisors, from RM0090. Pure constexpr with no
 *        libopencm3, so tests/stm32_clock_test.cpp checks it on the host and
 *        the firmware can static_assert its own clock plan.
 */

#include <cstdint>

namespace hal::stm32
{
/// Fastest SYSCLK the STM32F405 is rated for.
constexpr uint32_t MAX_SYSCLK_HZ = 168000000;
/// USB OTG FS needs exactly 48 MHz from the PLL's Q output.
constexpr uint32_t USB_HZ = 48000000;

/// Main PLL dividers: SYSCLK = source / m * n / p, USB = source / m * n / q.
struct PllPlan
{
    bool     valid;
    uint32_t m;
    uint32_t n;
    uint32_t p;
    uint32_t q;
    uint32_t vco_in_hz;
    uint32_t vco_out_hz;
    uint32_t sysclk_hz;
    uint32_t usb_hz;
};

/**
 * Finds dividers that give exactly sysclk_hz and exactly 48 MHz USB from a
 * source_hz PLL input, or valid = false if none exist.
 *
 * Limits (RM0090, RCC_PLLCFGR): VCO input 1-2 MHz, VCO output 192-432 MHz,
 * M 2-63, P 2/4/6/8, Q 2-15. N is held to 192-432: older RM0090 revisions
 * give that range for the F405/F407, newer ones widen it, so 192 is legal
 * under both. Among valid plans the smallest M wins, because the highest VCO
 * input frequency gives the least PLL jitter (RM0090 recommends 2 MHz).
 */
constexpr PllPlan planPll(uint32_t source_hz, uint32_t sysclk_hz)
{
    PllPlan none{};
    if (sysclk_hz == 0 || sysclk_hz > MAX_SYSCLK_HZ)
        return none;
    for (uint32_t m = 2; m <= 63; ++m)
    {
        if (source_hz / m > 2000000 || source_hz < 1000000 * m)
            continue;
        for (uint32_t p = 2; p <= 8; p += 2)
        {
            // source / m * n == sysclk * p, exactly.
            const uint64_t vco_x_m = static_cast<uint64_t>(sysclk_hz) * p * m;
            if (vco_x_m % source_hz != 0)
                continue;
            const uint64_t n = vco_x_m / source_hz;
            if (n < 192 || n > 432)
                continue;
            const uint64_t vco_out = static_cast<uint64_t>(sysclk_hz) * p;
            if (vco_out < 192000000 || vco_out > 432000000)
                continue;
            if (vco_out % USB_HZ != 0)
                continue;
            const uint64_t q = vco_out / USB_HZ;
            if (q < 2 || q > 15)
                continue;
            PllPlan plan{};
            plan.valid      = true;
            plan.m          = m;
            plan.n          = static_cast<uint32_t>(n);
            plan.p          = p;
            plan.q          = static_cast<uint32_t>(q);
            plan.vco_in_hz  = source_hz / m;
            plan.vco_out_hz = static_cast<uint32_t>(vco_out);
            plan.sysclk_hz  = sysclk_hz;
            plan.usb_hz     = static_cast<uint32_t>(vco_out / q);
            return plan;
        }
    }
    return none;
}

/// Bus clocks. APB1 is capped at 42 MHz and APB2 at 84 MHz (DS8626).
struct BusPlan
{
    uint32_t apb1_div;
    uint32_t apb2_div;
    uint32_t hclk_hz;
    uint32_t pclk1_hz;
    uint32_t pclk2_hz;
    uint32_t tim_apb1_hz; ///< TIM2-7, TIM12-14
    uint32_t tim_apb2_hz; ///< TIM1, TIM8-11
};

constexpr uint32_t MAX_PCLK1_HZ = 42000000;
constexpr uint32_t MAX_PCLK2_HZ = 84000000;

/// Smallest APB prescaler (1, 2, 4, 8 or 16) that keeps the bus at or under max_hz.
constexpr uint32_t apbDivider(uint32_t hclk_hz, uint32_t max_hz)
{
    uint32_t div = 1;
    while (div < 16 && hclk_hz / div > max_hz)
        div *= 2;
    return div;
}

/// A timer on a prescaled APB runs at twice the bus clock (RM0090 clock tree).
constexpr uint32_t timerClockHz(uint32_t pclk_hz, uint32_t apb_div)
{
    return apb_div == 1 ? pclk_hz : 2 * pclk_hz;
}

/// AHB undivided, each APB as fast as its limit allows.
constexpr BusPlan planBuses(uint32_t sysclk_hz)
{
    BusPlan bus{};
    bus.hclk_hz     = sysclk_hz;
    bus.apb1_div    = apbDivider(sysclk_hz, MAX_PCLK1_HZ);
    bus.apb2_div    = apbDivider(sysclk_hz, MAX_PCLK2_HZ);
    bus.pclk1_hz    = sysclk_hz / bus.apb1_div;
    bus.pclk2_hz    = sysclk_hz / bus.apb2_div;
    bus.tim_apb1_hz = timerClockHz(bus.pclk1_hz, bus.apb1_div);
    bus.tim_apb2_hz = timerClockHz(bus.pclk2_hz, bus.apb2_div);
    return bus;
}

/// Flash wait states at 2.7-3.6 V: one per started 30 MHz of HCLK beyond the first.
constexpr uint32_t flashWaitStates(uint32_t hclk_hz)
{
    return hclk_hz == 0 ? 0 : (hclk_hz - 1) / 30000000;
}

/// Timer prescaler and auto-reload: PSC + 1 = clock / count, ARR + 1 = count / rate.
/// Sized for a 16-bit timer such as TIM7.
struct TimerDivisors
{
    bool     valid; ///< Both ratios divide exactly and PSC and ARR fit 16 bits
    uint32_t psc;
    uint32_t arr;
};

constexpr TimerDivisors timerDivisors(uint32_t timer_clk_hz, uint32_t count_hz,
                                      uint32_t overflow_hz)
{
    TimerDivisors d{};
    if (count_hz == 0 || overflow_hz == 0 || timer_clk_hz % count_hz != 0 ||
        count_hz % overflow_hz != 0)
        return d;
    const uint32_t psc_plus_1 = timer_clk_hz / count_hz;
    const uint32_t arr_plus_1 = count_hz / overflow_hz;
    if (psc_plus_1 > 65536 || arr_plus_1 > 65536)
        return d;
    d.valid = true;
    d.psc   = psc_plus_1 - 1;
    d.arr   = arr_plus_1 - 1;
    return d;
}

/// USART_BRR with 16x oversampling: USARTDIV * 16 = pclk / baud, in 12.4 fixed point.
struct UsartBrr
{
    uint16_t brr;
    bool     exact;    ///< The real baud rate equals the requested one
    bool     in_range; ///< USARTDIV is at least 1 and BRR fits 16 bits
};

constexpr UsartBrr usartBrr(uint32_t pclk_hz, uint32_t baud)
{
    UsartBrr       r{};
    const uint32_t div = (pclk_hz + baud / 2) / baud;
    r.brr              = static_cast<uint16_t>(div);
    r.exact            = pclk_hz % baud == 0;
    r.in_range         = div >= 16 && div <= 0xFFFF;
    return r;
}

/// SPI baud divider (2..256, powers of two): the fastest clock at or under max_hz.
constexpr uint32_t spiDivider(uint32_t pclk_hz, uint32_t max_hz)
{
    uint32_t div = 2;
    while (div < 256 && pclk_hz / div > max_hz)
        div *= 2;
    return div;
}
} // namespace hal::stm32

#endif // HAL_STM32_CLOCK_PLAN_H
