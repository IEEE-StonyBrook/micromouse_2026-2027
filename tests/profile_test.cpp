// Profile: trapezoidal accelerate / cruise / brake, observed tick by tick.
#include <cmath>

#include "config/motion.h"
#include "control/profile.h"
#include "doctest/doctest.h"

namespace
{
struct Run
{
    int   ticks         = 0;
    int   cruise_ticks  = 0; // ticks spent at top speed
    float peak_speed    = 0.0f;
    float max_step      = 0.0f; // largest per-tick speed change before the last tick
    bool  sped_up_first = false;
};

// Steps the profile until it finishes (or a 10 s guard trips).
Run runToFinish(Profile& p, float top_speed)
{
    Run   r;
    float prev = p.velocity();
    while (!p.finished() && r.ticks < 10 * static_cast<int>(LOOP_FREQUENCY_HZ))
    {
        p.update();
        r.ticks++;
        const float v = p.velocity();
        if (r.ticks == 1)
            r.sped_up_first = std::fabs(v) > std::fabs(prev);
        // The finishing tick snaps to the final speed, so leave it out.
        if (!p.finished())
            r.max_step = std::fmax(r.max_step, std::fabs(v - prev));
        r.peak_speed = std::fmax(r.peak_speed, std::fabs(v));
        if (std::fabs(std::fabs(v) - top_speed) < 1e-3f)
            r.cruise_ticks++;
        prev = v;
    }
    return r;
}
} // namespace

TEST_CASE("profile accelerates, cruises and brakes to rest at the target")
{
    Profile p;
    p.start(1000.0f, 800.0f, 0.0f, 4000.0f);
    CHECK(p.active());

    const Run r = runToFinish(p, 800.0f);
    REQUIRE(p.finished());
    CHECK(r.sped_up_first);
    CHECK(r.peak_speed == doctest::Approx(800.0f));
    // 1000 mm at 800 mm/s with 80 mm of ramp each end: most of it is cruise.
    CHECK(r.cruise_ticks > static_cast<int>(0.9f * LOOP_FREQUENCY_HZ));
    // Never changes speed faster than the commanded acceleration.
    CHECK(r.max_step <= 4000.0f * LOOP_INTERVAL_S + 1e-3f);
    CHECK(p.position() == doctest::Approx(1000.0f).epsilon(0.001));
    CHECK(p.velocity() == 0.0f);
    // Trapezoid time is d/v + v/a = 1.25 s + 0.2 s; braking creeps the last bit.
    CHECK(r.ticks * LOOP_INTERVAL_S == doctest::Approx(1.45f).epsilon(0.05));
}

TEST_CASE("profile too short to cruise peaks below top speed")
{
    Profile p;
    p.start(50.0f, 2000.0f, 0.0f, 4000.0f);
    const Run r = runToFinish(p, 2000.0f);
    REQUIRE(p.finished());
    CHECK(r.cruise_ticks == 0);
    // Triangle peak sqrt(a * d) = sqrt(4000 * 50) ~ 447 mm/s.
    CHECK(r.peak_speed == doctest::Approx(447.0f).epsilon(0.05));
    CHECK(p.position() == doctest::Approx(50.0f).epsilon(0.01));
    CHECK(p.velocity() == 0.0f);
}

TEST_CASE("profile with a final speed finishes moving at that speed")
{
    Profile p;
    p.start(180.0f, 800.0f, 300.0f, 4000.0f);
    const Run r = runToFinish(p, 800.0f);
    REQUIRE(p.finished());
    CHECK(r.peak_speed == doctest::Approx(800.0f));
    CHECK(p.velocity() == doctest::Approx(300.0f));
    // Overshoot is at most one tick of travel at the final speed.
    CHECK(p.position() >= 180.0f - 0.125f);
    CHECK(p.position() <= 180.0f + 300.0f * LOOP_INTERVAL_S);
}

TEST_CASE("profile with a start speed carries it into the move")
{
    Profile p;
    p.start(180.0f, 300.0f, 800.0f, 300.0f, 4000.0f);
    p.update();
    CHECK(p.velocity() > 300.0f);
    runToFinish(p, 800.0f);
    CHECK(p.finished());
    CHECK(p.velocity() == doctest::Approx(300.0f));
}

TEST_CASE("profile runs backwards for a negative distance")
{
    Profile p;
    p.start(-90.0f, 500.0f, 0.0f, 3000.0f);
    const Run r = runToFinish(p, 500.0f);
    REQUIRE(p.finished());
    CHECK(r.sped_up_first);
    CHECK(r.peak_speed == doctest::Approx(500.0f));
    CHECK(p.position() == doctest::Approx(-90.0f).epsilon(0.01));
    CHECK(p.velocity() == 0.0f);
}

TEST_CASE("profile stop brakes to rest from cruise")
{
    Profile p;
    p.start(1000.0f, 800.0f, 0.0f, 4000.0f);
    for (int i = 0; i < 200; i++)
        p.update();
    REQUIRE(p.velocity() == doctest::Approx(800.0f));
    const float at_stop = p.position();

    p.stop(4000.0f);
    runToFinish(p, 0.0f);
    CHECK(p.finished());
    CHECK(p.velocity() == 0.0f);
    // Braking distance v^2 / 2a = 80 mm.
    CHECK(p.position() - at_stop == doctest::Approx(80.0f).epsilon(0.02));
}

TEST_CASE("profile with no distance or no acceleration finishes at once")
{
    Profile p;
    p.start(0.0f, 800.0f, 0.0f, 4000.0f);
    CHECK(p.finished());
    p.start(100.0f, 800.0f, 0.0f, 0.0f);
    CHECK(p.finished());
}
