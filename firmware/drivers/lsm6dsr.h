#ifndef DRIVERS_LSM6DSR_H
#define DRIVERS_LSM6DSR_H

#include <cstdint>

#include "hal/spi.h"

/**
 * @brief Register-level pieces of the ST LSM6DSR 6-axis IMU, from its
 *        datasheet (DS13254). Pure functions, host-tested in
 *        tests/lsm6dsr_test.cpp.
 */
namespace lsm6dsr
{
constexpr uint8_t WHO_AM_I_VALUE = 0x6B;

/// Gyro sensitivity at FS = 2000 dps: 70 mdps/LSB.
constexpr float GYRO_DEGPS_PER_LSB = 0.070f;
/// Accel sensitivity at FS = +/-8 g: 0.244 mg/LSB.
constexpr float ACCEL_G_PER_LSB = 0.000244f;

constexpr float gyroDegps(int16_t raw)
{
    return static_cast<float>(raw) * GYRO_DEGPS_PER_LSB;
}

constexpr float accelG(int16_t raw)
{
    return static_cast<float>(raw) * ACCEL_G_PER_LSB;
}

/// FIFO_DATA_OUT_TAG sensor codes for uncompressed data.
constexpr uint8_t TAG_GYRO  = 0x01;
constexpr uint8_t TAG_ACCEL = 0x02;

/// One 7-byte FIFO word: the tag byte, then X, Y, Z as little-endian int16.
struct FifoWord
{
    uint8_t sensor;    ///< TAG_SENSOR, bits 7:3 of the tag byte
    uint8_t tag_count; ///< TAG_CNT, bits 2:1
    int16_t x;
    int16_t y;
    int16_t z;
};

FifoWord decodeFifoWord(const uint8_t bytes[7]);

/// Words waiting in the FIFO: DIFF_FIFO, 10 bits across FIFO_STATUS1 and 2.
constexpr uint16_t fifoLevel(uint8_t status1, uint8_t status2)
{
    return static_cast<uint16_t>(status1 | ((status2 & 0x03) << 8));
}

/// FIFO_OVR_IA in FIFO_STATUS2: the FIFO filled and the oldest data was lost.
constexpr bool fifoOverrun(uint8_t status2)
{
    return (status2 & 0x40) != 0;
}
} // namespace lsm6dsr

/**
 * @brief LSM6DSR over SPI: gyro fixed at 2000 dps, accel at +/-8 g, both
 *        batched into the FIFO in continuous mode and drained every tick.
 *
 * The gyro runs at 1.66 kHz and the accel at 833 Hz, so a 1 kHz tick reads
 * one or two gyro words; yawRateDegps() is their mean, which filters rather
 * than samples the gyro and keeps the ODR from aliasing against the tick.
 *
 * Yaw is positive counter-clockwise seen from above. The gyro's Z axis points
 * out of the package top (right-hand rule), so a chip mounted face up needs
 * yaw_sign +1 and one mounted face down -1.
 *
 * init() runs in the main loop; readFifo() and the getters in the tick. No
 * logging or heap anywhere, so both are safe in interrupt context.
 */
class Lsm6dsr
{
  public:
    enum class Status : uint8_t
    {
        kOk,
        kWrongId,      ///< WHO_AM_I is not 0x6B: wrong chip, wiring or SPI mode
        kResetTimeout, ///< SW_RESET never cleared
    };

    explicit Lsm6dsr(hal::SpiDevice& spi, int yaw_sign = 1);

    /// Checks WHO_AM_I, resets the chip (bounded wait) and configures it.
    /// Call at least 10 ms after power-up (the chip's boot time).
    Status init();

    uint8_t whoAmI();

    /// Drains up to max_words FIFO words and returns how many it read.
    uint32_t readFifo(uint32_t max_words = 8);

    /// Mean yaw rate over the last read that held gyro data, deg/s, + = CCW.
    float yawRateDegps() const { return yaw_rate_degps_; }

    /// Latest acceleration along each chip axis, in g.
    float accelXG() const { return lsm6dsr::accelG(accel_raw_[0]); }
    float accelYG() const { return lsm6dsr::accelG(accel_raw_[1]); }
    float accelZG() const { return lsm6dsr::accelG(accel_raw_[2]); }

    uint32_t gyroSamples() const { return gyro_samples_; }
    uint32_t accelSamples() const { return accel_samples_; }
    /// Reads that found the FIFO had overrun (data lost: the tick fell behind).
    uint32_t fifoOverruns() const { return fifo_overruns_; }

  private:
    uint8_t readRegister(uint8_t reg);
    void    writeRegister(uint8_t reg, uint8_t value);

    hal::SpiDevice& spi_;
    int             yaw_sign_;
    float           yaw_rate_degps_ = 0.0f;
    int16_t         accel_raw_[3]   = {0, 0, 0};
    uint32_t        gyro_samples_   = 0;
    uint32_t        accel_samples_  = 0;
    uint32_t        fifo_overruns_  = 0;
};

#endif // DRIVERS_LSM6DSR_H
