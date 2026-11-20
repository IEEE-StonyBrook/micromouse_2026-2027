// Host test for the speed-run time estimate.
#include <cmath>
#include <string>

#include "doctest/doctest.h"
#include "navigation/run_time.h"

static bool near(float actual, float expected)
{
    return std::fabs(actual - expected) < 1e-3f;
}

// Fixed numbers, not config/motion.h, so retuning the mouse leaves the worked
// examples below valid: 1500 mm/s at 2500 mm/s^2, spins at 270 deg/s, 1200 deg/s^2.
static RunTimeModel model()
{
    RunTimeModel m;
    m.cell_mm           = 180.0f;
    m.speed_mmps        = 1500.0f;
    m.accel_mmps2       = 2500.0f;
    m.turn_omega_degps  = 270.0f;
    m.turn_alpha_degps2 = 1200.0f;
    return m;
}

TEST_CASE("run time: one cell never reaches top speed")
{
    // Triangle profile: t = 2 sqrt(d / a) = 2 sqrt(180 / 2500) = 0.5367 s.
    CHECK(near(straightTimeS(1, model()), 0.5367f));
}

TEST_CASE("run time: a long straight cruises")
{
    // 15 cells = 2700 mm: 0.6 s up and 0.6 s down cover 900 mm, 1800 mm at
    // 1500 mm/s takes 1.2 s, so 2.4 s in all.
    CHECK(near(straightTimeS(15, model()), 2.4f));
}

TEST_CASE("run time: a spin turn is a trapezoid in angle")
{
    // 0.225 s ramps cover 2 x 30.375 deg; the other 29.25 deg at 270 deg/s
    // take 0.1083 s: 0.5583 s for a 90.
    CHECK(near(turnTimeS(90.0f, model()), 0.5583f));
    CHECK(near(turnTimeS(-90.0f, model()), 0.5583f));
}

TEST_CASE("run time: a longer straight costs less per cell than turns")
{
    CHECK(straightTimeS(8, model()) / 8.0f < straightTimeS(1, model()));
    // The same four cells, straight or broken by turns.
    const float straight = estimatedRunTimeS("F#F#F#F#", model());
    const float zigzag   = estimatedRunTimeS("F#R#F#L#F#R#F#", model());
    CHECK(near(straight, straightTimeS(4, model())));
    CHECK(near(zigzag, 4.0f * straightTimeS(1, model()) + 3.0f * turnTimeS(90.0f, model())));
    CHECK(straight < zigzag);
}

TEST_CASE("run time: an about-turn is two spins")
{
    CHECK(near(estimatedRunTimeS("R#R#F#", model()),
               2.0f * turnTimeS(90.0f, model()) + straightTimeS(1, model())));
}
