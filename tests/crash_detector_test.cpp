// CrashDetector: the crash watchdog's pure trip logic, fed one tick at a time.
// Thresholds are times (config/motion.h), so the tick counts here come from
// LOOP_TICKS and the same cases hold at 500 Hz and 1 kHz.
#include "config/config.h"
#include "config/motion.h"
#include "control/crash_detector.h"
#include "doctest/doctest.h"

namespace
{
// Cruising straight at 300 mm/s on a full battery: nothing near a limit.
CrashInputs cruising()
{
    CrashInputs in;
    in.left_volts    = 1.8f;
    in.right_volts   = 1.8f;
    in.battery_volts = 8.0f;
    in.left_mmps     = 300.0f;
    in.right_mmps    = 300.0f;
    in.fwd_error_mm  = 0.5f;
    return in;
}

CrashInputs saturated()
{
    CrashInputs in = cruising();
    in.left_volts  = MAX_VOLTAGE; // 6 V clamp on an 8 V battery: at the limit
    return in;
}

// Runs n ticks of the same inputs; returns the tick (1-based) it tripped, or 0.
int feed(CrashDetector& d, const CrashInputs& in, int n)
{
    for (int i = 1; i <= n; i++)
        if (d.update(in))
            return i;
    return 0;
}
} // namespace

TEST_CASE("crash: sustained saturation trips after about 100 ms, not before")
{
    CrashDetector d(true);
    const int     n = LOOP_TICKS(CRASH_SATURATION_TIME_S);
    CHECK(feed(d, saturated(), n - 1) == 0);
    CHECK_FALSE(d.tripped());
    CHECK(d.update(saturated()));
    CHECK(d.tripped());
    CHECK(d.cause() == CrashCause::kSaturation);
}

TEST_CASE("crash: the saturation limit follows a battery below MAX_VOLTAGE")
{
    // 5 V battery: 4.9 V is 0.98 of what the motor can get, so saturated,
    // although well under the 6 V clamp.
    CrashDetector d(true);
    CrashInputs   in = cruising();
    in.battery_volts = 5.0f;
    in.right_volts   = -4.9f;
    CHECK(feed(d, in, LOOP_TICKS(0.2f)) > 0);

    // 4.7 V of 5 V is 0.94: below the 0.97 line.
    CrashDetector d2(true);
    in.right_volts = 4.7f;
    CHECK(feed(d2, in, LOOP_TICKS(0.5f)) == 0);
}

TEST_CASE("crash: a saturation burst shorter than the window does not trip")
{
    CrashDetector d(true);
    for (int burst = 0; burst < 10; burst++)
    {
        CHECK(feed(d, saturated(), LOOP_TICKS(0.08f)) == 0);
        CHECK(feed(d, cruising(), 1) == 0);
    }
    CHECK_FALSE(d.tripped());
}

TEST_CASE("crash: stall current at near-zero speed trips; without current sensing it cannot")
{
    CrashInputs in = cruising();
    in.left_mmps   = 0.0f;
    in.left_amps   = CRASH_STALL_AMPS * 1.2f;
    in.right_amps  = 0.2f;

    CrashDetector blind(true);
    CHECK(feed(blind, in, LOOP_TICKS(0.5f)) == 0); // has_current false: input ignored

    in.has_current = true;
    CrashDetector d(true);
    const int     tick = feed(d, in, LOOP_TICKS(0.5f));
    CHECK(tick == LOOP_TICKS(CRASH_STALL_TIME_S));
    CHECK(d.cause() == CrashCause::kStall);

    // The same current while the wheel turns is acceleration, not a stall.
    in.left_mmps = 300.0f;
    CrashDetector moving(true);
    CHECK(feed(moving, in, LOOP_TICKS(0.5f)) == 0);
}

TEST_CASE("crash: forward error that keeps growing trips; a large steady one does not")
{
    CrashDetector d(true);
    CrashInputs   in  = cruising();
    int           hit = 0;
    for (int i = 1; i <= LOOP_TICKS(1.0f) && hit == 0; i++)
    {
        in.fwd_error_mm = 0.2f * static_cast<float>(i); // 0.2 mm per tick
        if (d.update(in))
            hit = i;
    }
    CHECK(hit > 0);
    CHECK(d.cause() == CrashCause::kRunaway);

    CrashDetector steady(true);
    in.fwd_error_mm = CRASH_RUNAWAY_ERROR_MM + 5.0f;
    CHECK(feed(steady, in, LOOP_TICKS(1.0f)) == 0);
}

TEST_CASE("crash: a trip stays latched until cleared")
{
    CrashDetector d(true);
    REQUIRE(feed(d, saturated(), LOOP_TICKS(0.5f)) > 0);
    CHECK(feed(d, cruising(), LOOP_TICKS(0.5f)) == 0); // reports the edge only once
    CHECK(d.tripped());
    CHECK(d.cause() == CrashCause::kSaturation);
    CHECK(d.tripCount() == 1);

    d.clear(); // takes effect on the next update(), in the control tick
    d.update(cruising());
    CHECK_FALSE(d.tripped());
    CHECK(d.cause() == CrashCause::kNone);
    CHECK(d.tripCount() == 1);
}

TEST_CASE("crash: inhibit and disable suppress every cause")
{
    CrashInputs stall = saturated();
    stall.has_current = true;
    stall.left_mmps   = 0.0f;
    stall.left_amps   = CRASH_STALL_AMPS * 2.0f;

    CrashDetector inhibited(true);
    inhibited.set_inhibit(true);
    CHECK(feed(inhibited, stall, LOOP_TICKS(0.5f)) == 0);
    // Lifting the inhibit starts the window afresh rather than tripping at once.
    inhibited.set_inhibit(false);
    CHECK_FALSE(inhibited.update(stall));
    CHECK(feed(inhibited, stall, LOOP_TICKS(0.5f)) > 0);

    CrashDetector disabled(false);
    CHECK(feed(disabled, stall, LOOP_TICKS(0.5f)) == 0);
    disabled.set_enabled(true);
    CHECK(feed(disabled, stall, LOOP_TICKS(0.5f)) > 0);
}
