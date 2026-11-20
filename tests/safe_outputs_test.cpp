// SafeOutputs: the power-on order that holds every actuator output (motor
// driver inputs and nSLEEP, IR emitters, fan) at its off level before any
// other start-up step, and keeps it off until the board arms. Checked against
// a fake port that logs each GPIO call, plus the STM32 board table.
#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include "common/safe_outputs.h"
#include "config/board_stm32.h"
#include "doctest/doctest.h"
#include "fakes/fake_output_port.h"

namespace
{
// Two outputs off low (a driver input, nSLEEP) and one off high (an
// active-low enable), so the test sees each level used.
constexpr board::SafeOutput OUTPUTS[] = {
    {{'B', 12}, false},
    {{'C', 6}, false},
    {{'A', 8}, true},
};

std::ptrdiff_t indexOf(const std::vector<std::string>& log, const std::string& entry)
{
    const auto it = std::find(log.begin(), log.end(), entry);
    REQUIRE_MESSAGE(it != log.end(), "missing: " << entry);
    return it - log.begin();
}

// Stands in for the boot steps that follow power-on: logs, returns a value.
struct ClockSetup
{
    std::vector<std::string>& log;
    int                       operator()()
    {
        log.push_back("clock setup");
        return 168;
    }
};

bool samePin(board::Pin a, board::Pin b)
{
    return a.port == b.port && a.number == b.number;
}

bool heldLow(board::Pin pin)
{
    for (const board::SafeOutput& out : board::REV1_SAFE_OUTPUTS)
        if (samePin(out.pin, pin))
            return !out.off_high;
    return false;
}
} // namespace

TEST_CASE("power-on drives every safe output off before any other start-up step")
{
    std::vector<std::string> log;
    fake::OutputPort         port(log);
    SafeOutputs              safe(port, OUTPUTS, std::size(OUTPUTS));

    const int clock = powerOn(safe, ClockSetup{log});

    CHECK(clock == 168); // the start-up step's result comes back
    const std::ptrdiff_t init = indexOf(log, "clock setup");
    CHECK(indexOf(log, "level B12 low") < init);
    CHECK(indexOf(log, "output B12") < init);
    CHECK(indexOf(log, "level C6 low") < init);
    CHECK(indexOf(log, "output C6") < init);
    CHECK(indexOf(log, "level A8 high") < init);
    CHECK(indexOf(log, "output A8") < init);
}

TEST_CASE("each safe output's off level is set before the pin starts driving")
{
    // A pin made an output first would drive whatever its data register held
    // at reset (low) for a moment: an on pulse for an active-low enable.
    std::vector<std::string> log;
    fake::OutputPort         port(log);
    SafeOutputs              safe(port, OUTPUTS, std::size(OUTPUTS));
    safe.holdOff();

    CHECK(indexOf(log, "level B12 low") < indexOf(log, "output B12"));
    CHECK(indexOf(log, "level C6 low") < indexOf(log, "output C6"));
    CHECK(indexOf(log, "level A8 high") < indexOf(log, "output A8"));
}

TEST_CASE("the board may arm only once the outputs are held off")
{
    std::vector<std::string> log;
    fake::OutputPort         port(log);
    SafeOutputs              safe(port, OUTPUTS, std::size(OUTPUTS));

    CHECK_FALSE(safe.held()); // arming refused: the outputs were never held off
    safe.holdOff();
    CHECK(safe.held());
}

TEST_CASE("a board with no safe outputs touches no pin")
{
    std::vector<std::string> log;
    fake::OutputPort         port(log);
    SafeOutputs              safe(port, nullptr, 0);
    safe.holdOff();

    CHECK(log.empty());
    CHECK(safe.held()); // nothing to hold, so arming is still allowed
}

TEST_CASE("the rev1 table holds both motor drivers, the emitters and the fan off")
{
    // DRV8874: nSLEEP low sleeps the driver; IN1 = IN2 = low coasts.
    CHECK(heldLow(board::MOTOR_NSLEEP));
    for (const board::TimerPin& in : board::MOTOR_INPUTS)
        CHECK(heldLow(in.pin));
    for (const board::Pin& emitter : board::EMITTER_PINS)
        CHECK(heldLow(emitter));
    CHECK(heldLow(board::FAN_PIN));
    CHECK(std::size(board::REV1_SAFE_OUTPUTS) ==
          1 + std::size(board::MOTOR_INPUTS) + std::size(board::EMITTER_PINS) + 1);
}

TEST_CASE("the rev1 safe outputs claim each pin once")
{
    const auto& outs = board::REV1_SAFE_OUTPUTS;
    for (std::size_t i = 0; i < std::size(outs); ++i)
        for (std::size_t j = i + 1; j < std::size(outs); ++j)
            CHECK_FALSE(samePin(outs[i].pin, outs[j].pin));
}

TEST_CASE("a board without the rev1 actuators drives no safe output")
{
    // The Feather uses PC6/PC7 for an encoder and PA3 for its battery monitor:
    // driving the rev1 pins there would fight those peripherals.
    if (!board::MOTOR_DRIVERS_FITTED)
        CHECK(board::SAFE_OUTPUT_COUNT == 0);
    else
        CHECK(board::SAFE_OUTPUT_COUNT == std::size(board::REV1_SAFE_OUTPUTS));
}
