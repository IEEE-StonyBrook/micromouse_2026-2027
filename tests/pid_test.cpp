// PID: a position-error PD. Setpoint is a rate, measured is a per-tick change.
#include "control/pid.h"
#include "doctest/doctest.h"

namespace
{
constexpr float kHz = 500.0f;
constexpr float kDt = 1.0f / kHz;
} // namespace

TEST_CASE("pid output pushes toward the setpoint")
{
    PID behind(2.0f, 0.0f, kHz);
    // Commanded 100 mm/s, moved nothing: 0.2 mm behind, so drive forward.
    CHECK(behind.update(100.0f, 0.0f) > 0.0f);

    PID ahead(2.0f, 0.0f, kHz);
    // Commanded 100 mm/s, moved 1 mm in one tick: ahead, so drive back.
    CHECK(ahead.update(100.0f, 1.0f) < 0.0f);

    PID reverse(2.0f, 0.0f, kHz);
    CHECK(reverse.update(-100.0f, 0.0f) < 0.0f);
}

TEST_CASE("pid proportional term scales with the position error")
{
    PID pid(2.0f, 0.0f, kHz);
    CHECK(pid.update(100.0f, 0.0f) == doctest::Approx(2.0f * 100.0f * kDt));
    CHECK(pid.error() == doctest::Approx(100.0f * kDt));

    // Doubling the gain doubles the output for the same error.
    PID stiffer(4.0f, 0.0f, kHz);
    CHECK(stiffer.update(100.0f, 0.0f) == doctest::Approx(2.0f * 2.0f * 100.0f * kDt));
}

TEST_CASE("pid integrates position error across ticks")
{
    PID   pid(1.0f, 0.0f, kHz);
    float out = 0.0f;
    for (int i = 0; i < 10; i++)
        out = pid.update(100.0f, 0.0f);
    CHECK(pid.error() == doctest::Approx(10.0f * 100.0f * kDt));
    CHECK(out == doctest::Approx(10.0f * 100.0f * kDt));
}

TEST_CASE("pid tracking the setpoint exactly holds zero output")
{
    PID pid(3.0f, 0.5f, kHz);
    for (int i = 0; i < 50; i++)
        CHECK(pid.update(250.0f, 250.0f * kDt) == doctest::Approx(0.0f));
}

TEST_CASE("pid derivative term acts on the per-tick change in error")
{
    PID pid(0.0f, 0.01f, kHz);
    // First tick: error jumps from 0 to 0.2 mm, so D = kd * 0.2 * 500.
    CHECK(pid.update(100.0f, 0.0f) == doctest::Approx(0.01f * 0.2f * kHz));
    // Error unchanged on the next tick: D falls to zero.
    CHECK(pid.update(0.0f, 0.0f) == doctest::Approx(0.0f));
}

TEST_CASE("pid steering adjustment adds to the setpoint rate")
{
    PID with_steer(2.0f, 0.0f, kHz);
    PID plain(2.0f, 0.0f, kHz);
    CHECK(with_steer.update(100.0f, 0.0f, 50.0f) == doctest::Approx(plain.update(150.0f, 0.0f)));
}

TEST_CASE("pid output saturates symmetrically and reset clears history")
{
    PID pid(1000.0f, 0.0f, kHz);
    pid.setOutputLimit(6.0f);
    CHECK(pid.update(1000.0f, 0.0f) == doctest::Approx(6.0f));
    pid.reset();
    CHECK(pid.error() == 0.0f);
    CHECK(pid.update(-1000.0f, 0.0f) == doctest::Approx(-6.0f));
}
