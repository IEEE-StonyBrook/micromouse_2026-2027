#include "hal/stm32/clock.h"

#include <libopencm3/cm3/dwt.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>

#include "config/board_feather.h"

namespace hal::stm32
{
namespace
{
constexpr uint32_t HSI_HZ = 16000000;
// The crystal's typical start-up is 2 ms (DS8626); 100 ms leaves margin for a
// slow or marginal crystal while still booting promptly without one.
constexpr uint32_t HSE_TIMEOUT_CYCLES = HSI_HZ / 10;

constexpr PllPlan HSE_PLL = planPll(board::HSE_HZ, MAX_SYSCLK_HZ);
constexpr PllPlan HSI_PLL = planPll(HSI_HZ, MAX_SYSCLK_HZ);
static_assert(HSE_PLL.valid && HSE_PLL.sysclk_hz == MAX_SYSCLK_HZ, "no exact PLL from the HSE");
static_assert(HSI_PLL.valid && HSI_PLL.sysclk_hz == MAX_SYSCLK_HZ, "no exact PLL from the HSI");
static_assert(BUSES.apb1_div == 4 && BUSES.apb2_div == 2,
              "prescaler constants below assume /4, /2");

ClockSource source = ClockSource::kHsi;

rcc_clock_scale scaleFor(const PllPlan& pll, uint8_t pll_source)
{
    rcc_clock_scale s{};
    s.pllm       = static_cast<uint8_t>(pll.m);
    s.plln       = static_cast<uint16_t>(pll.n);
    s.pllp       = static_cast<uint8_t>(pll.p);
    s.pllq       = static_cast<uint8_t>(pll.q);
    s.pllr       = 0; // the F405 has no R output
    s.pll_source = pll_source;
    s.flash_config =
        FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY(flashWaitStates(BUSES.hclk_hz));
    s.hpre           = RCC_CFGR_HPRE_NODIV;
    s.ppre1          = RCC_CFGR_PPRE_DIV4;
    s.ppre2          = RCC_CFGR_PPRE_DIV2;
    s.voltage_scale  = PWR_SCALE1;
    s.ahb_frequency  = BUSES.hclk_hz;
    s.apb1_frequency = BUSES.pclk1_hz;
    s.apb2_frequency = BUSES.pclk2_hz;
    return s;
}
} // namespace

ClockSource setupClocks()
{
    // Out of reset the core runs on the HSI, so the cycle counter ticks at 16 MHz here.
    dwt_enable_cycle_counter();

    rcc_osc_on(RCC_HSE);
    source = waitForHse([] { return rcc_is_osc_ready(RCC_HSE); }, dwt_read_cycle_counter,
                        HSE_TIMEOUT_CYCLES);

    rcc_clock_scale scale;
    if (source == ClockSource::kHse)
    {
        // HSERDY is already set, so the wait inside rcc_clock_setup_pll returns at once.
        scale = scaleFor(HSE_PLL, RCC_CFGR_PLLSRC_HSE_CLK);
    }
    else
    {
        rcc_osc_off(RCC_HSE);
        scale = scaleFor(HSI_PLL, RCC_CFGR_PLLSRC_HSI_CLK);
    }
    rcc_clock_setup_pll(&scale);
    flash_prefetch_enable();
    return source;
}

ClockSource clockSource()
{
    return source;
}
} // namespace hal::stm32
