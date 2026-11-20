#include "hal/stm32/h_bridge.h"

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "config/board_stm32.h"
#include "hal/stm32/clock.h"
#include "hal/stm32/gpio.h"

namespace hal::stm32
{
namespace
{
constexpr uint32_t TIMER_HZ = BUSES.tim_apb2_hz; // TIM8 is on APB2

constexpr tim_oc_id OC_IDS[4] = {TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4};

// The bring-up only knows TIM8: anything else is a board table error.
void requireTim8(const board::TimerPin& p)
{
    if (p.timer != 8)
        while (true)
            __asm__ volatile("bkpt #0");
}
} // namespace

void HBridge::init()
{
    // Driver asleep and inputs low before the pins leave their reset state.
    gpioOutput(board::MOTOR_NSLEEP);
    gpio_clear(gpioPort(board::MOTOR_NSLEEP), gpioBit(board::MOTOR_NSLEEP));

    rcc_periph_clock_enable(RCC_TIM8);
    rcc_periph_reset_pulse(RST_TIM8);
    period_ = TIMER_HZ / board::MOTOR_PWM_HZ;
    timer_set_prescaler(TIM8, 0);
    timer_set_period(TIM8, period_ - 1);
    timer_enable_preload(TIM8);
    for (const board::TimerPin& p : board::MOTOR_INPUTS)
    {
        requireTim8(p);
        const tim_oc_id oc = OC_IDS[p.channel - 1];
        // PWM mode 1: the input is high while CNT < CCR, so CCR = period is 100%.
        timer_set_oc_mode(TIM8, oc, TIM_OCM_PWM1);
        timer_enable_oc_preload(TIM8, oc);
        timer_set_oc_value(TIM8, oc, 0);
        timer_enable_oc_output(TIM8, oc);
    }
    timer_generate_event(TIM8, TIM_EGR_UG); // load ARR and CCRs now
    timer_enable_break_main_output(TIM8);   // an advanced timer's outputs need MOE
    timer_enable_counter(TIM8);
    for (const board::TimerPin& p : board::MOTOR_INPUTS)
        gpioAlternate(p.pin, p.af);

    // ADC1 at APB2 / 4 = 21 MHz, under the 36 MHz maximum. 144 cycles of
    // sampling (about 7 us) settles a few-kOhm R_IPROPI source easily.
    for (const board::Pin& pin : board::IPROPI_PINS)
    {
        rcc_periph_clock_enable(gpioRcc(pin));
        gpio_mode_setup(gpioPort(pin), GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpioBit(pin));
    }
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY4);
    adc_enable_scan_mode(ADC1); // convert the whole injected group per start
    uint8_t channels[2] = {board::IPROPI_ADC_CHANNELS[0], board::IPROPI_ADC_CHANNELS[1]};
    for (uint8_t ch : channels)
        adc_set_sample_time(ADC1, ch, ADC_SMPR_SMP_144CYC);
    adc_set_injected_sequence(ADC1, 2, channels);
    adc_disable_external_trigger_injected(ADC1);
    adc_power_on(ADC1);
    ready_ = true;
}

void HBridge::setInputs(int motor, float in1_high, float in2_high)
{
    if (!ready_)
        return;
    const float levels[2] = {in1_high, in2_high};
    for (int i = 0; i < 2; i++)
    {
        const board::TimerPin& p = board::MOTOR_INPUTS[2 * motor + i];
        timer_set_oc_value(TIM8, OC_IDS[p.channel - 1],
                           static_cast<uint32_t>(levels[i] * static_cast<float>(period_) + 0.5f));
    }
}

void HBridge::setAwake(bool awake)
{
    if (!ready_)
        return;
    if (awake)
        gpio_set(gpioPort(board::MOTOR_NSLEEP), gpioBit(board::MOTOR_NSLEEP));
    else
        gpio_clear(gpioPort(board::MOTOR_NSLEEP), gpioBit(board::MOTOR_NSLEEP));
}

void HBridge::sample()
{
    if (!ready_)
        return;
    // Two 144-cycle conversions take about 15 us: long done a tick later.
    if (adc_eoc_injected(ADC1))
    {
        counts_[0] = static_cast<uint16_t>(adc_read_injected(ADC1, 1));
        counts_[1] = static_cast<uint16_t>(adc_read_injected(ADC1, 2));
        adc_clear_flag(ADC1, ADC_SR_JEOC);
    }
    adc_start_conversion_injected(ADC1);
}
} // namespace hal::stm32
