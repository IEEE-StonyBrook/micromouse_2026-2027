#ifndef TESTS_FAKES_FAKE_ROBOT_H
#define TESTS_FAKES_FAKE_ROBOT_H

/**
 * @brief Host test doubles for the Motor, Encoder, IMU and Battery drivers.
 *
 * fake_drivers.cpp defines those classes' member functions in place of the
 * Pico ones in drivers/, so Drivetrain and Motion run unchanged on the host.
 * The real headers fix each class's layout, so per-instance fake state lives
 * in tables keyed by the instance. Directions are logical: positive duty and
 * positive counts mean the wheel drives forward; invert_direction is ignored.
 */

#include "drivers/encoder.h"
#include "drivers/imu.h"
#include "drivers/motor.h"

namespace fake
{
/// Last duty cycle the motor was commanded, in [-1, 1]; 0 after stop().
float motorDuty(const Motor& motor);

/// Sets the encoder's raw count; ticks() reports it relative to the last reset().
void setEncoderCount(const Encoder& encoder, int count);

/// Sets the IMU's absolute yaw (deg, + = CCW) and yaw rate (deg/s).
void setImu(const IMU& imu, float yaw_deg, float omega_degps);

/**
 * @brief Two wheels on a first-order motor model, wired to fake drivers.
 *
 * Each wheel's speed approaches km * volts with time constant tm_s:
 *   dv/dt = (km * volts - v) / tm_s
 * where volts = duty * DEFAULT_BATTERY_VOLTAGE (Drivetrain's scale with no
 * Battery). Wheel travel feeds the encoders in whole counts and the wheel
 * difference feeds the IMU yaw. No slip, no friction, no noise.
 */
class Robot
{
  public:
    Robot(float km_mmps_per_volt, float tm_s);

    /// Advances the plant by one control tick (LOOP_INTERVAL_S).
    void step();

    float wheelMm(bool left) const { return left ? left_mm_ : right_mm_; }
    float wheelMmps(bool left) const { return left ? left_mmps_ : right_mmps_; }
    float distanceMm() const { return 0.5f * (left_mm_ + right_mm_); }
    float yawDeg() const;

    Motor   left_motor;
    Motor   right_motor;
    Encoder left_encoder;
    Encoder right_encoder;
    IMU     imu;

  private:
    float km_;
    float tm_s_;
    float left_mm_    = 0.0f;
    float right_mm_   = 0.0f;
    float left_mmps_  = 0.0f;
    float right_mmps_ = 0.0f;
};
} // namespace fake

#endif // TESTS_FAKES_FAKE_ROBOT_H
