// STM32 Motor: one wheel of a DRV8874 pair, through MotorBridge. The bridge
// holds the drive, brake, sleep and arm-gate logic; this only applies the
// wheel's direction and the battery scaling, as the Pico's Motor does.
#include <algorithm>
#include <cstdio>

#include "drivers/motor.h"
#include "drivers/motor_bridge.h"

std::atomic<bool> Motor::battery_fallback_{false};

Motor::Motor(MotorBridge& bridge, int motor, bool invert_direction)
    : bridge_(&bridge), motor_(motor), invert_direction_(invert_direction)
{
}

void Motor::set_motor_pwm(float duty_cycle)
{
    duty_cycle = std::clamp(duty_cycle, -1.0f, 1.0f);
    if (invert_direction_)
        duty_cycle = -duty_cycle;
    bridge_->set_motor_duty(motor_, duty_cycle);
}

void Motor::set_motor_volts(float desired_volts, float battery_volts)
{
    // As on the Pico: under 1 V the pack is off or unread, so scale against
    // the nominal pack and raise a flag for the main loop to report.
    const bool fallback = battery_volts < 1.0f;
    battery_fallback_.store(fallback, std::memory_order_relaxed);
    if (fallback)
        battery_volts = DEFAULT_BATTERY_VOLTAGE;
    set_motor_pwm(desired_volts / battery_volts);
}

void Motor::stop()
{
    // Zero duty is a DRV8874 brake (both inputs high), like the Pico's PWM 0.
    bridge_->set_motor_duty(motor_, 0.0f);
}

void Motor::reportBatteryFallback()
{
    static bool reported = false;
    const bool  fallback = battery_fallback_.load(std::memory_order_relaxed);
    if (fallback == reported)
        return;
    reported = fallback;
    std::printf(fallback ? "battery <1V; scaling motors to the nominal pack\n"
                         : "battery voltage restored\n");
}
