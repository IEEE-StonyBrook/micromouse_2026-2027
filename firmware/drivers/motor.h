#ifndef DRIVERS_MOTOR_H
#define DRIVERS_MOTOR_H

#include <algorithm>
#include <atomic>
#include <cmath>

#include "common/log.h"
#include "config/config.h"

#if defined(STM32_BUILD)
class MotorBridge;
#endif

/**
 * @brief DC Motor driver using TB6552FNG with hardware inverter
 *
 * Each motor is controlled by two pins:
 *   - DIR pin (GPIO): sets direction via hardware inverter → IN1/IN2
 *   - PWM pin (hardware PWM): controls speed
 *
 * STBY is hardwired to 5V. Short brake occurs when PWM is low.
 */
class Motor
{
  public:
#if defined(STM32_BUILD)
    /**
     * @brief STM32: one motor of a DRV8874 pair (drivers/stm32/motor.cpp).
     * @param bridge The pair's MotorBridge, which gates on arming
     * @param motor 0 for the left motor, 1 for the right
     * @param invert_direction If true, flips the direction logic
     */
    Motor(MotorBridge& bridge, int motor, bool invert_direction);
#else
    /**
     * @brief Constructs motor controller with pin configuration
     * @param dir_pin GPIO pin for direction control (CTRL → inverter → IN1/IN2)
     * @param pwm_pin GPIO pin for PWM speed control
     * @param invert_direction If true, flips the direction logic
     */
    Motor(int dir_pin, int pwm_pin, bool invert_direction = false);
#endif

    /**
     * @brief Sets PWM command to motor
     * @param duty_cycle PWM in range [-1.0, 1.0] (positive = forward)
     */
    void set_motor_pwm(float duty_cycle);

    /**
     * @brief Sets voltage command to motor (scaled by battery voltage)
     * @param desired_volts Desired motor voltage
     * @param battery_volts Current battery voltage for scaling
     */
    void set_motor_volts(float desired_volts, float battery_volts);

    /**
     * @brief Immediately stops motor (short brake via PWM low)
     */
    void stop();

    /**
     * @brief Logs a change in the battery-fallback state. Call from the main
     *        loop only: set_motor_volts() runs in the control interrupt, where
     *        logging (string formatting, heap) is not allowed.
     */
    static void reportBatteryFallback();

  private:
#if defined(STM32_BUILD)
    MotorBridge* bridge_;
    int          motor_;
#else
    void setup_pins();
    void setup_pwm();

    int      dir_pin_;
    int      pwm_pin_;
    unsigned pwm_slice_;
    unsigned pwm_channel_;
#endif
    bool invert_direction_;

    // Set by set_motor_volts() in the control interrupt when the battery reads
    // under 1 V. Shared by both motors; plain load/store is lock-free on M0+.
    static std::atomic<bool> battery_fallback_;
};

#endif
