#include "hal/stm32/quad_counter.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "hal/stm32/gpio.h"

namespace hal::stm32
{
namespace
{
struct TimerUnit
{
    uint32_t         base;
    rcc_periph_clken clock;
    rcc_periph_rst   reset;
};

// The timers the Feather table may use for encoders.
TimerUnit timerUnit(uint8_t number)
{
    switch (number)
    {
        case 3:
            return {TIM3, RCC_TIM3, RST_TIM3};
        case 4:
            return {TIM4, RCC_TIM4, RST_TIM4};
        default:
            // A board table error: stop here, where a debugger shows why.
            while (true)
                __asm__ volatile("bkpt #0");
    }
}
} // namespace

void QuadCounter::init(const board::TimerPin& a, const board::TimerPin& b)
{
    const TimerUnit unit = timerUnit(a.timer);
    timer_               = unit.base;

    // Encoders often have open-collector outputs; the pull-ups are harmless on push-pull ones.
    gpioAlternate(a.pin, a.af, GPIO_PUPD_PULLUP);
    gpioAlternate(b.pin, b.af, GPIO_PUPD_PULLUP);

    rcc_periph_clock_enable(unit.clock);
    rcc_periph_reset_pulse(unit.reset);
    timer_ic_set_input(timer_, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(timer_, TIM_IC2, TIM_IC_IN_TI2);
    timer_ic_set_filter(timer_, TIM_IC1, TIM_IC_CK_INT_N_8);
    timer_ic_set_filter(timer_, TIM_IC2, TIM_IC_CK_INT_N_8);
    timer_slave_set_mode(timer_, TIM_SMCR_SMS_EM3);
    timer_set_period(timer_, 0xFFFF);
    timer_enable_counter(timer_);

    extended_.reset(raw16());
}

uint16_t QuadCounter::raw16() const
{
    return static_cast<uint16_t>(TIM_CNT(timer_));
}

void QuadCounter::update()
{
    extended_.update(raw16());
}

int32_t QuadCounter::count() const
{
    // Masked so the read cannot land inside an update() from the tick.
    const uint32_t mask  = cm_mask_interrupts(1);
    const int32_t  value = extended_.value();
    cm_mask_interrupts(mask);
    return value;
}
} // namespace hal::stm32
