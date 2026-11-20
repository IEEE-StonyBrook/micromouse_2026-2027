#include "drivers/lsm6dsr.h"

namespace
{
// Register map (DS13254, section 8).
constexpr uint8_t REG_FIFO_CTRL3   = 0x09;
constexpr uint8_t REG_FIFO_CTRL4   = 0x0A;
constexpr uint8_t REG_WHO_AM_I     = 0x0F;
constexpr uint8_t REG_CTRL1_XL     = 0x10;
constexpr uint8_t REG_CTRL2_G      = 0x11;
constexpr uint8_t REG_CTRL3_C      = 0x12;
constexpr uint8_t REG_CTRL4_C      = 0x13;
constexpr uint8_t REG_FIFO_STATUS1 = 0x3A;
constexpr uint8_t REG_FIFO_TAG     = 0x78; // then 6 data bytes at 0x79-0x7E

constexpr uint8_t SPI_READ = 0x80;

// Output data rate codes, shared by CTRLx ODR fields and FIFO_CTRL3 batch rates.
constexpr uint8_t ODR_833_HZ  = 0x7;
constexpr uint8_t ODR_1666_HZ = 0x8;

constexpr uint8_t FS_G_2000_DPS = 0x3 << 2;
constexpr uint8_t FS_XL_8_G     = 0x3 << 2; // FS_XL 11 = +/-8 g (00 2 g, 10 4 g, 01 16 g)

constexpr uint8_t CTRL3_SW_RESET    = 1 << 0;
constexpr uint8_t CTRL3_IF_INC      = 1 << 2;
constexpr uint8_t CTRL3_BDU         = 1 << 6;
constexpr uint8_t CTRL4_I2C_DISABLE = 1 << 2;

constexpr uint8_t FIFO_MODE_CONTINUOUS = 0x6;

// SW_RESET takes tens of microseconds; each poll is one SPI read (a few us at
// 5 MHz), so this allows several milliseconds before giving up.
constexpr int RESET_POLL_LIMIT = 1000;

int16_t le16(const uint8_t* p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0] | (p[1] << 8)));
}
} // namespace

namespace lsm6dsr
{
FifoWord decodeFifoWord(const uint8_t bytes[7])
{
    FifoWord w;
    w.sensor    = static_cast<uint8_t>(bytes[0] >> 3);
    w.tag_count = static_cast<uint8_t>((bytes[0] >> 1) & 0x03);
    w.x         = le16(bytes + 1);
    w.y         = le16(bytes + 3);
    w.z         = le16(bytes + 5);
    return w;
}
} // namespace lsm6dsr

Lsm6dsr::Lsm6dsr(hal::SpiDevice& spi, int yaw_sign) : spi_(spi), yaw_sign_(yaw_sign < 0 ? -1 : 1)
{
}

uint8_t Lsm6dsr::readRegister(uint8_t reg)
{
    const uint8_t tx[2] = {static_cast<uint8_t>(reg | SPI_READ), 0};
    uint8_t       rx[2] = {0, 0};
    spi_.transfer(tx, rx, 2);
    return rx[1];
}

void Lsm6dsr::writeRegister(uint8_t reg, uint8_t value)
{
    const uint8_t tx[2] = {reg, value};
    spi_.transfer(tx, nullptr, 2);
}

uint8_t Lsm6dsr::whoAmI()
{
    return readRegister(REG_WHO_AM_I);
}

Lsm6dsr::Status Lsm6dsr::init()
{
    // Check identity before writing anything, so a wrong chip is left alone.
    if (whoAmI() != lsm6dsr::WHO_AM_I_VALUE)
        return Status::kWrongId;

    writeRegister(REG_CTRL3_C, CTRL3_SW_RESET | CTRL3_IF_INC);
    int polls = 0;
    while (readRegister(REG_CTRL3_C) & CTRL3_SW_RESET)
    {
        if (++polls >= RESET_POLL_LIMIT)
            return Status::kResetTimeout;
    }

    writeRegister(REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC);
    // SPI only: stop the I2C block from misreading SPI traffic as I2C.
    writeRegister(REG_CTRL4_C, CTRL4_I2C_DISABLE);
    writeRegister(REG_FIFO_CTRL3, static_cast<uint8_t>((ODR_1666_HZ << 4) | ODR_833_HZ));
    writeRegister(REG_FIFO_CTRL4, FIFO_MODE_CONTINUOUS);
    writeRegister(REG_CTRL2_G, static_cast<uint8_t>((ODR_1666_HZ << 4) | FS_G_2000_DPS));
    writeRegister(REG_CTRL1_XL, static_cast<uint8_t>((ODR_833_HZ << 4) | FS_XL_8_G));
    return Status::kOk;
}

uint32_t Lsm6dsr::readFifo(uint32_t max_words)
{
    const uint8_t status_tx[3] = {static_cast<uint8_t>(REG_FIFO_STATUS1 | SPI_READ), 0, 0};
    uint8_t       status_rx[3] = {0, 0, 0};
    spi_.transfer(status_tx, status_rx, 3);
    if (lsm6dsr::fifoOverrun(status_rx[2]))
        ++fifo_overruns_;

    uint32_t words = lsm6dsr::fifoLevel(status_rx[1], status_rx[2]);
    if (words > max_words)
        words = max_words;

    int32_t  gyro_z_sum = 0;
    uint32_t gyro_count = 0;
    for (uint32_t i = 0; i < words; ++i)
    {
        // One transaction per word: tag plus six data bytes, auto-incremented.
        const uint8_t tx[8] = {static_cast<uint8_t>(REG_FIFO_TAG | SPI_READ)};
        uint8_t       rx[8] = {};
        spi_.transfer(tx, rx, 8);
        const lsm6dsr::FifoWord w = lsm6dsr::decodeFifoWord(rx + 1);
        if (w.sensor == lsm6dsr::TAG_GYRO)
        {
            gyro_z_sum += w.z;
            ++gyro_count;
        }
        else if (w.sensor == lsm6dsr::TAG_ACCEL)
        {
            accel_raw_[0] = w.x;
            accel_raw_[1] = w.y;
            accel_raw_[2] = w.z;
            ++accel_samples_;
        }
    }

    if (gyro_count > 0)
    {
        const float mean_raw = static_cast<float>(gyro_z_sum) / static_cast<float>(gyro_count);
        yaw_rate_degps_ = static_cast<float>(yaw_sign_) * mean_raw * lsm6dsr::GYRO_DEGPS_PER_LSB;
        gyro_samples_ += gyro_count;
    }
    return words;
}
