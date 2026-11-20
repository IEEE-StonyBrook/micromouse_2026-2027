// Host tests for the LSM6DSR driver against a fake register map on a fake SPI
// bus. Register addresses, bit fields and sensitivities are from ST's LSM6DSR
// datasheet (DS13254); expected values are worked from it, not from the code.
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#include "doctest/doctest.h"
#include "drivers/lsm6dsr.h"
#include "hal/spi.h"

namespace
{
// Datasheet register addresses, spelled out here so the test does not lean on
// the driver's own table.
constexpr uint8_t REG_FIFO_CTRL3   = 0x09;
constexpr uint8_t REG_FIFO_CTRL4   = 0x0A;
constexpr uint8_t REG_WHO_AM_I     = 0x0F;
constexpr uint8_t REG_CTRL1_XL     = 0x10;
constexpr uint8_t REG_CTRL2_G      = 0x11;
constexpr uint8_t REG_CTRL3_C      = 0x12;
constexpr uint8_t REG_CTRL4_C      = 0x13;
constexpr uint8_t REG_FIFO_STATUS1 = 0x3A;
constexpr uint8_t REG_FIFO_STATUS2 = 0x3B;
constexpr uint8_t REG_FIFO_TAG     = 0x78;

/// An LSM6DSR in SPI mode: bit 7 of the first byte selects read, and the
/// address auto-increments (CTRL3_C.IF_INC, set out of reset).
class FakeLsm6dsr : public hal::SpiDevice
{
  public:
    FakeLsm6dsr()
    {
        regs[REG_WHO_AM_I] = 0x6B;
        regs[REG_CTRL3_C]  = 0x04; // IF_INC
    }

    void transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) override
    {
        ++transactions;
        REQUIRE(len >= 1);
        const bool read = (tx[0] & 0x80) != 0;
        uint8_t    addr = tx[0] & 0x7F;
        if (rx != nullptr)
            rx[0] = 0;
        if (addr == REG_FIFO_TAG && read)
        {
            REQUIRE(len == 8); // one whole FIFO word per transaction
            std::vector<uint8_t> word(7, 0);
            if (!fifo.empty())
            {
                word = fifo.front();
                fifo.pop_front();
            }
            std::memcpy(rx + 1, word.data(), 7);
            return;
        }
        for (std::size_t i = 1; i < len; ++i, ++addr)
        {
            if (read)
                rx[i] = readReg(addr);
            else
                writeReg(addr, tx[i]);
        }
    }

    void pushWord(uint8_t tag, int16_t x, int16_t y, int16_t z)
    {
        fifo.push_back({static_cast<uint8_t>(tag << 3), static_cast<uint8_t>(x & 0xFF),
                        static_cast<uint8_t>((x >> 8) & 0xFF), static_cast<uint8_t>(y & 0xFF),
                        static_cast<uint8_t>((y >> 8) & 0xFF), static_cast<uint8_t>(z & 0xFF),
                        static_cast<uint8_t>((z >> 8) & 0xFF)});
    }

    uint8_t                          regs[128] = {};
    std::deque<std::vector<uint8_t>> fifo;
    int                              reset_reads_left = 3; // SW_RESET clears after this many reads
    bool                             reset_sticks     = false;
    bool                             overrun          = false;
    int                              transactions     = 0;
    int                              writes           = 0;

  private:
    uint8_t readReg(uint8_t addr)
    {
        if (addr == REG_CTRL3_C && (regs[REG_CTRL3_C] & 0x01) && !reset_sticks &&
            --reset_reads_left <= 0)
            regs[REG_CTRL3_C] = 0x04; // reset done: defaults back
        if (addr == REG_FIFO_STATUS1)
            return static_cast<uint8_t>(fifo.size() & 0xFF);
        if (addr == REG_FIFO_STATUS2)
            return static_cast<uint8_t>(((fifo.size() >> 8) & 0x03) | (overrun ? 0x40 : 0));
        return regs[addr];
    }

    void writeReg(uint8_t addr, uint8_t value)
    {
        ++writes;
        regs[addr] = value;
    }
};
} // namespace

TEST_CASE("LSM6DSR raw counts convert at the datasheet sensitivities")
{
    // FS = 2000 dps: 70 mdps/LSB. FS = +/-8 g: 0.244 mg/LSB.
    CHECK(lsm6dsr::gyroDegps(1000) == doctest::Approx(70.0f));
    CHECK(lsm6dsr::gyroDegps(-32768) == doctest::Approx(-2293.76f));
    CHECK(lsm6dsr::accelG(4096) == doctest::Approx(0.999424f));
    CHECK(lsm6dsr::accelG(-4096) == doctest::Approx(-0.999424f));
}

TEST_CASE("LSM6DSR FIFO word decodes its tag and little-endian axes")
{
    // Tag byte: sensor in bits 7:3, tag counter in 2:1, parity in 0.
    const uint8_t bytes[7]    = {(0x01 << 3) | (2 << 1) | 1, 0x34, 0x12, 0xFF, 0xFF, 0x00, 0x80};
    const lsm6dsr::FifoWord w = lsm6dsr::decodeFifoWord(bytes);
    CHECK(w.sensor == lsm6dsr::kTagGyro);
    CHECK(w.tag_count == 2);
    CHECK(w.x == 0x1234);
    CHECK(w.y == -1);
    CHECK(w.z == -32768);
}

TEST_CASE("LSM6DSR FIFO level is the 10-bit DIFF_FIFO count")
{
    CHECK(lsm6dsr::fifoLevel(0x00, 0x00) == 0);
    CHECK(lsm6dsr::fifoLevel(0xFF, 0x03) == 1023);
    CHECK(lsm6dsr::fifoLevel(0x10, 0x41) == 0x110); // overrun flag is not part of the level
    CHECK(lsm6dsr::fifoOverrun(0x40));
    CHECK_FALSE(lsm6dsr::fifoOverrun(0x03));
}

TEST_CASE("LSM6DSR init resets, then sets 2000 dps, accel on, FIFO continuous")
{
    FakeLsm6dsr fake;
    Lsm6dsr     imu(fake);
    CHECK(imu.init() == Lsm6dsr::Status::kOk);
    // CTRL2_G: FS_G = 11 (2000 dps) in bits 3:2, a non-zero gyro ODR in 7:4.
    CHECK((fake.regs[REG_CTRL2_G] & 0x0F) == 0x0C);
    CHECK((fake.regs[REG_CTRL2_G] >> 4) != 0);
    // CTRL1_XL: FS_XL = 11 (+/-8 g), a non-zero accel ODR.
    CHECK((fake.regs[REG_CTRL1_XL] & 0x0C) == 0x0C);
    CHECK((fake.regs[REG_CTRL1_XL] >> 4) != 0);
    // CTRL3_C keeps IF_INC and adds BDU; CTRL4_C turns the I2C block off.
    CHECK((fake.regs[REG_CTRL3_C] & 0x44) == 0x44);
    CHECK((fake.regs[REG_CTRL4_C] & 0x04) == 0x04);
    // Both sensors batched; FIFO_MODE = 110 (continuous).
    CHECK((fake.regs[REG_FIFO_CTRL3] & 0x0F) != 0);
    CHECK((fake.regs[REG_FIFO_CTRL3] >> 4) != 0);
    CHECK((fake.regs[REG_FIFO_CTRL4] & 0x07) == 0x06);
}

TEST_CASE("LSM6DSR init refuses a device that is not an LSM6DSR, writing nothing")
{
    FakeLsm6dsr fake;
    fake.regs[REG_WHO_AM_I] = 0x6C; // an LSM6DSO32X, say
    Lsm6dsr imu(fake);
    CHECK(imu.init() == Lsm6dsr::Status::kWrongId);
    CHECK(fake.writes == 0);
}

TEST_CASE("LSM6DSR init gives up on a reset that never finishes")
{
    FakeLsm6dsr fake;
    fake.reset_sticks = true;
    Lsm6dsr imu(fake);
    CHECK(imu.init() == Lsm6dsr::Status::kResetTimeout);
    CHECK(fake.transactions < 2000); // bounded, not a hang
}

TEST_CASE("LSM6DSR FIFO read averages this tick's gyro words into a yaw rate")
{
    FakeLsm6dsr fake;
    Lsm6dsr     imu(fake);
    REQUIRE(imu.init() == Lsm6dsr::Status::kOk);

    fake.pushWord(lsm6dsr::kTagGyro, 0, 0, 1000);  // 70 dps
    fake.pushWord(lsm6dsr::kTagAccel, 0, 0, 4096); // 1 g on Z
    fake.pushWord(lsm6dsr::kTagGyro, 0, 0, 2000);  // 140 dps
    fake.pushWord(0x03, 1, 2, 3);                  // temperature: skipped
    CHECK(imu.readFifo() == 4);
    CHECK(imu.yawRateDegps() == doctest::Approx(105.0f)); // mean of 70 and 140
    CHECK(imu.accelZG() == doctest::Approx(0.999424f));
    CHECK(imu.gyroSamples() == 2);
    CHECK(imu.accelSamples() == 1);

    // An empty FIFO keeps the last rate rather than reporting zero.
    CHECK(imu.readFifo() == 0);
    CHECK(imu.yawRateDegps() == doctest::Approx(105.0f));
}

TEST_CASE("LSM6DSR yaw sign follows how the board is mounted")
{
    FakeLsm6dsr fake;
    Lsm6dsr     upside_down(fake, -1);
    REQUIRE(upside_down.init() == Lsm6dsr::Status::kOk);
    fake.pushWord(lsm6dsr::kTagGyro, 0, 0, 1000);
    upside_down.readFifo();
    CHECK(upside_down.yawRateDegps() == doctest::Approx(-70.0f));
}

TEST_CASE("LSM6DSR FIFO read is capped per tick and counts overruns")
{
    FakeLsm6dsr fake;
    Lsm6dsr     imu(fake);
    REQUIRE(imu.init() == Lsm6dsr::Status::kOk);
    for (int i = 0; i < 10; ++i)
        fake.pushWord(lsm6dsr::kTagGyro, 0, 0, 100);
    fake.overrun = true;
    CHECK(imu.readFifo(4) == 4);
    CHECK(fake.fifo.size() == 6);
    CHECK(imu.fifoOverruns() == 1);
}
