// SafeOutputs: the power-on order that holds every actuator output (motor
// driver inputs and nSLEEP, IR emitters, fan) at its off level before any
// other start-up step, and keeps it off until the board arms. Checked against
// a fake port that logs each GPIO call.
#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include "common/safe_outputs.h"
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

TEST_CASE("nothing may turn on until the board arms after power-on")
{
    std::vector<std::string> log;
    fake::OutputPort         port(log);
    SafeOutputs              safe(port, OUTPUTS, std::size(OUTPUTS));

    CHECK_FALSE(safe.arm()); // refused: the outputs were never held off
    CHECK_FALSE(safe.armed());

    safe.holdOff();
    CHECK_FALSE(safe.armed());

    CHECK(safe.arm());
    CHECK(safe.armed());
    safe.disarm();
    CHECK_FALSE(safe.armed());
}

TEST_CASE("a board with no safe outputs touches no pin")
{
    std::vector<std::string> log;
    fake::OutputPort         port(log);
    SafeOutputs              safe(port, nullptr, 0);
    safe.holdOff();

    CHECK(log.empty());
    CHECK(safe.arm()); // nothing to hold, so arming is still allowed
}
