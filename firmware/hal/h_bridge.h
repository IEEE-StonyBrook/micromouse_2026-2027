#ifndef HAL_H_BRIDGE_H
#define HAL_H_BRIDGE_H

#include <cstdint>

namespace hal
{
/**
 * @brief The pins of two H-bridge motor drivers in IN/IN (PWM) mode that
 *        share one nSLEEP line, plus their current-sense ADC readings.
 *
 * MotorBridge (drivers/motor_bridge.h) holds the logic and talks to the
 * board through this, so it builds and tests on the host against a fake.
 * Backends: hal/stm32/h_bridge.h (DRV8874s on TIM PWM and ADC1).
 *
 * Motor 0 is the left motor, 1 the right.
 */
class HBridgePort
{
  public:
    /// Fraction of each PWM period that IN1 and IN2 are high, each in [0, 1].
    virtual void setInputs(int motor, float in1_high, float in2_high) = 0;

    /// Drives nSLEEP: high (awake) or low (both drivers asleep, outputs Hi-Z).
    virtual void setAwake(bool awake) = 0;

    /// The latest current-sense (IPROPI) ADC reading, in raw counts.
    virtual uint16_t senseCounts(int motor) = 0;

  protected:
    ~HBridgePort() = default;
};
} // namespace hal

#endif // HAL_H_BRIDGE_H
