#include "drivers/motor.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

std::atomic<bool> Motor::battery_fallback_{false};

Motor::Motor(int dir_pin, int pwm_pin, bool invert_direction)
    : dir_pin_(dir_pin), pwm_pin_(pwm_pin), invert_direction_(invert_direction)
{
    setup_pins();
    setup_pwm();
}

void Motor::setup_pins()
{
    // Direction pin is a plain GPIO output
    gpio_init(dir_pin_);
    gpio_set_dir(dir_pin_, GPIO_OUT);
    gpio_put(dir_pin_, 0);

    // PWM pin uses hardware PWM
    gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);
}

void Motor::setup_pwm()
{
    pwm_slice_   = pwm_gpio_to_slice_num(pwm_pin_);
    pwm_channel_ = pwm_gpio_to_channel(pwm_pin_);

    pwm_config config = pwm_get_default_config();
    pwm_init(pwm_slice_, &config, false);
    pwm_set_wrap(pwm_slice_, PWM_WRAP);
    pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
    pwm_set_enabled(pwm_slice_, true);
}

void Motor::set_motor_pwm(float duty_cycle)
{
    duty_cycle = std::clamp(duty_cycle, -1.0f, 1.0f);

    // Set direction via CTRL pin (inverter makes IN1/IN2 pair)
    bool forward = (duty_cycle >= 0.0f);
    if (invert_direction_)
        forward = !forward;
    gpio_put(dir_pin_, forward ? 1 : 0);

    // Set speed via PWM (low PWM = short brake)
    float pwm_level = std::fabs(duty_cycle) * PWM_WRAP;
    pwm_level       = std::clamp(pwm_level, 0.0f, static_cast<float>(PWM_WRAP));
    pwm_set_chan_level(pwm_slice_, pwm_channel_, static_cast<uint16_t>(pwm_level));
}

void Motor::set_motor_volts(float desired_volts, float battery_volts)
{
    // Motors run from the battery rail, not USB, so a sub-1 V reading means
    // the pack is off. This runs in the 500 Hz control interrupt, so it only
    // raises a flag; reportBatteryFallback() logs it from the main loop.
    const bool fallback = battery_volts < 1.0f;
    battery_fallback_.store(fallback, std::memory_order_relaxed);
    if (fallback)
        battery_volts = DEFAULT_BATTERY_VOLTAGE;
    set_motor_pwm(desired_volts / battery_volts);
}

void Motor::reportBatteryFallback()
{
    // Edge-triggered: one line per transition, not one per poll.
    static bool reported = false;
    const bool  fallback = battery_fallback_.load(std::memory_order_relaxed);
    if (fallback == reported)
        return;
    reported = fallback;
    if (fallback)
    {
        LOG_DEBUG("Battery <1V; using default voltage (USB only? motors will not move)");
    }
    else
    {
        LOG_DEBUG("Battery voltage restored");
    }
}

void Motor::stop()
{
    pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
}
