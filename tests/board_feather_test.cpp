// Host checks on the Feather STM32F405 pin table: no pin is claimed twice, and
// nothing lands on a pin the board already wires to something else.
#include <vector>

#include "config/board_feather.h"
#include "doctest/doctest.h"

namespace
{
std::vector<board::Pin> claimedPins()
{
    using namespace board;
    std::vector<Pin> pins = {LED, TICK_PROBE, IMU_SPI.cs};
    for (const TimerPin& p : ENCODER_PINS)
        pins.push_back(p.pin);
    pins.push_back(CONSOLE.tx.pin);
    pins.push_back(CONSOLE.rx.pin);
    pins.push_back(IMU_SPI.sck.pin);
    pins.push_back(IMU_SPI.miso.pin);
    pins.push_back(IMU_SPI.mosi.pin);
    return pins;
}

bool samePin(board::Pin a, board::Pin b)
{
    return a.port == b.port && a.number == b.number;
}
} // namespace

TEST_CASE("Feather pin table claims every pin at most once")
{
    const std::vector<board::Pin> pins = claimedPins();
    for (std::size_t i = 0; i < pins.size(); ++i)
        for (std::size_t j = i + 1; j < pins.size(); ++j)
            CHECK_FALSE(samePin(pins[i], pins[j]));
}

TEST_CASE("Feather pin table avoids the on-board SD card, SPI flash and battery monitor")
{
    // Adafruit's Feather STM32F405 Express wiring: SDIO on PC8-PC12 and PD2,
    // the SPI flash on PB3-PB5 and PA15, the battery divider on PA3,
    // and the NeoPixel on PC0.
    const board::Pin reserved[] = {{'C', 8}, {'C', 9}, {'C', 10}, {'C', 11}, {'C', 12}, {'D', 2},
                                   {'B', 3}, {'B', 4}, {'B', 5},  {'A', 15}, {'A', 3},  {'C', 0}};
    for (const board::Pin& used : claimedPins())
        for (const board::Pin& r : reserved)
            CHECK_FALSE(samePin(used, r));
}

TEST_CASE("Each encoder uses channels 1 and 2 of one timer")
{
    // Encoder mode counts only TIx1 and TIx2.
    for (int e = 0; e < 2; ++e)
    {
        const board::TimerPin& a = board::ENCODER_PINS[2 * e];
        const board::TimerPin& b = board::ENCODER_PINS[2 * e + 1];
        CHECK(a.timer == b.timer);
        CHECK(a.channel == 1);
        CHECK(b.channel == 2);
    }
    CHECK(board::ENCODER_PINS[0].timer != board::ENCODER_PINS[2].timer);
}
