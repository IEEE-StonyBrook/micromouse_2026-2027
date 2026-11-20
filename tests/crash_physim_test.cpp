// Crash watchdog end to end: the real Motion and CrashDetector in the physics
// sim. A wheel pinned against a wall must cut the motors within 150 ms; hard
// but healthy moves must not trip. make -C tests runs these at 500 Hz and 1 kHz.
#include <cmath>

#include "config/motion.h"
#include "control/crash_detector.h"
#include "doctest/doctest.h"
#include "sim/physim.h"

namespace
{
constexpr float kPinAtS = 0.6f; // mid-cruise on a search-speed straight

physim::Params pinnedLeft()
{
    physim::Params p       = physim::nominal();
    p.pinned_wheel.enabled = true;
    p.pinned_wheel.left    = true;
    p.pinned_wheel.at_s    = kPinAtS;
    return p;
}

physim::RunOptions search(bool watchdog = true)
{
    physim::RunOptions o;
    o.speed_mmps     = ROBOT_MAX_SEARCH_SPEED_MMPS;
    o.accel_mmps2    = ROBOT_SEARCH_ACCEL_MMPS2;
    o.crash_watchdog = watchdog;
    return o;
}

int  cut_calls = 0;
void countCut()
{
    cut_calls++;
}
} // namespace

TEST_CASE("crash sim: a pinned wheel cuts the motors within 150 ms and stays cut")
{
    cut_calls              = 0;
    physim::RunOptions o   = search();
    o.on_crash_cut         = countCut;
    const physim::Result r = physim::run(physim::Scenario::kStraight10, pinnedLeft(), o);
    REQUIRE(r.crash_trip_s >= 0.0f);
    CHECK(r.crash_trip_s - kPinAtS > 0.0f);
    CHECK(r.crash_trip_s - kPinAtS <= 0.150f);
    CHECK(r.crash_cause != CrashCause::kStall); // no current sensing by default
    CHECK(r.crash_cause != CrashCause::kNone);
    // Latched: through the rest of the run the profile still wants to move,
    // but no tick after the trip drives a motor.
    CHECK(r.peak_volts_after_trip == 0.0f);
    CHECK(cut_calls == 1);
}

TEST_CASE("crash sim: with current sensing a pinned wheel also trips within 150 ms")
{
    physim::RunOptions o   = search();
    o.sense_current        = true;
    const physim::Result r = physim::run(physim::Scenario::kStraight10, pinnedLeft(), o);
    REQUIRE(r.crash_trip_s >= 0.0f);
    CHECK(r.crash_trip_s - kPinAtS <= 0.150f);
    CHECK(r.peak_volts_after_trip == 0.0f);
}

TEST_CASE("crash sim: the inhibit flag lets a pinned wheel push")
{
    physim::RunOptions o   = search();
    o.crash_inhibit        = true;
    const physim::Result r = physim::run(physim::Scenario::kStraight10, pinnedLeft(), o);
    CHECK(r.crash_trip_s < 0.0f);
    CHECK(r.crash_cause == CrashCause::kNone);
}

TEST_CASE("crash sim: the watchdog off (the Pico default) never trips")
{
    const physim::Result r =
        physim::run(physim::Scenario::kStraight10, pinnedLeft(), search(false));
    CHECK(r.crash_trip_s < 0.0f);
}

TEST_CASE("crash sim: healthy moves do not trip")
{
    physim::RunOptions sense = search();
    sense.sense_current      = true;

    SUBCASE("full search-speed straight")
    {
        const physim::Result r =
            physim::run(physim::Scenario::kStraight10, physim::nominal(), sense);
        CHECK(r.finished);
        CHECK(r.crash_trip_s < 0.0f);
    }
    SUBCASE("hard acceleration to fast cruise")
    {
        // 4000 mm/s^2, well past ROBOT_BASE_ACCEL_MMPS2, to 1500 mm/s: the
        // command touches the 6 V clamp for a few ms only. (2000 mm/s does
        // trip: the motor cannot reach it on 6 V, so it saturates for good.)
        physim::RunOptions o   = sense;
        o.speed_mmps           = ROBOT_MAX_FAST_SPEED_MMPS;
        o.accel_mmps2          = 4000.0f;
        const physim::Result r = physim::run(physim::Scenario::kStraight10, physim::nominal(), o);
        CHECK(r.finished);
        CHECK(r.crash_trip_s < 0.0f);
    }
    SUBCASE("in-place 90")
    {
        const physim::Result r =
            physim::run(physim::Scenario::kInPlace90, physim::nominal(), sense);
        CHECK(r.finished);
        CHECK(r.crash_trip_s < 0.0f);
    }
    SUBCASE("smooth 90")
    {
        const physim::Result r = physim::run(physim::Scenario::kSmooth90, physim::nominal(), sense);
        CHECK(r.finished);
        CHECK(r.crash_trip_s < 0.0f);
    }
}
