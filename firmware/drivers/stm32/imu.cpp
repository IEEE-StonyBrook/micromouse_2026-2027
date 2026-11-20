// STM32 IMU: the LSM6DSR's integrated yaw and its yaw rate. The tick drains
// the FIFO (Lsm6dsr::readFifo) before anything here reads it.
#include "drivers/imu.h"

#include "common/utils.h"
#include "drivers/lsm6dsr.h"

IMU::IMU(const Lsm6dsr& gyro) : gyro_(&gyro)
{
}

void IMU::update()
{
    // Nothing per tick: the gyro is read and integrated by readFifo().
}

float IMU::robot_angle()
{
    return utils::wrapAngle180(gyro_->yawDeg() - yaw_offset_deg_);
}

float IMU::robot_omega()
{
    return gyro_->yawRateDegps();
}

float IMU::robot_rot_change()
{
    return gyro_->yawRateDegps() * LOOP_INTERVAL_S;
}

uint32_t IMU::packet_seq() const
{
    // One "packet" per gyro sample: DriverLab TURN-STEP gates on new data.
    return gyro_->gyroSamples();
}

void IMU::reset()
{
    yaw_offset_deg_ = gyro_->yawDeg();
}
