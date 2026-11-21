// Host definitions of the Motor, Encoder, IMU and Battery member functions,
// standing in for drivers/*.cpp, plus the fake::Robot plant. See fake_robot.h.
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include "config/config.h"
#include "config/geometry.h"
#include "config/motion.h"
#include "drivers/battery.h"
#include "fakes/fake_robot.h"

namespace
{
struct ImuState
{
    float yaw_deg     = 0.0f;
    float omega_degps = 0.0f;
    float offset_deg  = 0.0f;
};

std::map<const Motor*, float>  g_duty;
std::map<const Encoder*, int>  g_counts;
std::map<const IMU*, ImuState> g_imu;
std::function<void()>          g_imu_update_hook;
} // namespace

// --- Motor ---------------------------------------------------------------

Motor::Motor(int dir_pin, int pwm_pin, bool invert_direction)
    : dir_pin_(dir_pin), pwm_pin_(pwm_pin), pwm_slice_(0), pwm_channel_(0),
      invert_direction_(invert_direction)
{
    g_duty[this] = 0.0f;
}

void Motor::set_motor_pwm(float duty_cycle)
{
    g_duty[this] = std::clamp(duty_cycle, -1.0f, 1.0f);
}

void Motor::set_motor_volts(float desired_volts, float battery_volts)
{
    if (battery_volts < 1.0f)
        battery_volts = DEFAULT_BATTERY_VOLTAGE;
    set_motor_pwm(desired_volts / battery_volts);
}

void Motor::stop()
{
    g_duty[this] = 0.0f;
}

void Motor::reportBatteryFallback()
{
}

// --- Encoder -------------------------------------------------------------

Encoder::Encoder(hal::PioBlock pio_block, int gpio_pin, bool invert_direction)
    : pio_block_(pio_block), state_machine_(0), offset_ticks_(0),
      invert_direction_(invert_direction), gpio_pin_(gpio_pin)
{
    g_counts[this] = 0;
}

void Encoder::reset()
{
    offset_ticks_ = g_counts[this];
}

int Encoder::ticks() const
{
    return g_counts.at(this) - offset_ticks_;
}

// --- IMU -----------------------------------------------------------------

IMU::IMU(int uart_rx_pin) : uart_rx_pin_(uart_rx_pin)
{
    g_imu[this] = ImuState{};
}

void IMU::update()
{
    if (g_imu_update_hook)
        g_imu_update_hook();
}

float IMU::robot_angle()
{
    const ImuState& s = g_imu.at(this);
    float           a = std::fmod(s.yaw_deg - s.offset_deg + 180.0f, 360.0f);
    if (a < 0.0f)
        a += 360.0f;
    return a - 180.0f;
}

float IMU::robot_omega()
{
    return g_imu.at(this).omega_degps;
}

float IMU::robot_rot_change()
{
    return g_imu.at(this).omega_degps * LOOP_INTERVAL_S;
}

uint32_t IMU::packet_seq() const
{
    return 0;
}

void IMU::reset()
{
    ImuState& s  = g_imu.at(this);
    s.offset_deg = s.yaw_deg;
}

// --- Battery: only reached if a test wires one; none do yet. -------------

float Battery::voltage() const
{
    return DEFAULT_BATTERY_VOLTAGE;
}

// --- Test API and plant --------------------------------------------------

namespace fake
{
float motorDuty(const Motor& motor)
{
    return g_duty.at(&motor);
}

void setImuUpdateHook(std::function<void()> hook)
{
    g_imu_update_hook = std::move(hook);
}

void setEncoderCount(const Encoder& encoder, int count)
{
    g_counts.at(&encoder) = count;
}

void setImu(const IMU& imu, float yaw_deg, float omega_degps)
{
    ImuState& s   = g_imu.at(&imu);
    s.yaw_deg     = yaw_deg;
    s.omega_degps = omega_degps;
}

Robot::Robot(float km_mmps_per_volt, float tm_s)
    : left_motor(0, 1), right_motor(2, 3), left_encoder(hal::PioBlock::kPio0, 4),
      right_encoder(hal::PioBlock::kPio0, 6), imu(8), km_(km_mmps_per_volt), tm_s_(tm_s)
{
}

float Robot::yawDeg() const
{
    return (right_mm_ - left_mm_) * DEG_PER_MM_DIFFERENCE;
}

void Robot::step()
{
    const float dt       = LOOP_INTERVAL_S;
    const float vl       = motorDuty(left_motor) * DEFAULT_BATTERY_VOLTAGE;
    const float vr       = motorDuty(right_motor) * DEFAULT_BATTERY_VOLTAGE;
    const float prev_yaw = yawDeg();
    left_mmps_ += (km_ * vl - left_mmps_) * dt / tm_s_;
    right_mmps_ += (km_ * vr - right_mmps_) * dt / tm_s_;
    left_mm_ += left_mmps_ * dt;
    right_mm_ += right_mmps_ * dt;

    setEncoderCount(left_encoder, static_cast<int>(std::lround(left_mm_ / MM_PER_TICK_LEFT)));
    setEncoderCount(right_encoder, static_cast<int>(std::lround(right_mm_ / MM_PER_TICK_RIGHT)));
    setImu(imu, yawDeg(), (yawDeg() - prev_yaw) / dt);
}
} // namespace fake
