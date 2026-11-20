// Loop rate: set per platform in config/board.h. make -C tests runs the
// whole suite twice, at the Pico's 500 Hz and, built with
// -DHOST_LOOP_FREQUENCY_HZ=1000.0f, at the STM32's 1 kHz.
#include "config/motion.h"
#include "control/pid.h"
#include "doctest/doctest.h"

TEST_CASE("the loop rate is the one this build asked for")
{
#ifdef HOST_LOOP_FREQUENCY_HZ
    CHECK(LOOP_FREQUENCY_HZ == HOST_LOOP_FREQUENCY_HZ);
#else
    CHECK(LOOP_FREQUENCY_HZ == 500.0f); // the Pico board, the host default
#endif
    CHECK(LOOP_INTERVAL_S * LOOP_FREQUENCY_HZ == doctest::Approx(1.0f));
}

TEST_CASE("pid derivative response is the same at 500 Hz and 1 kHz")
{
    // An error that grows at 100 mm/s (setpoint 100 mm/s, nothing moved)
    // has derivative 100 mm/s whatever the rate: D = kd * 100 = 1.0.
    for (float hz : {500.0f, 1000.0f})
    {
        PID pid(0.0f, 0.01f, hz);
        for (int i = 0; i < 5; i++)
            CHECK(pid.update(100.0f, 0.0f) == doctest::Approx(1.0f));
    }
    // A 1 mm error step lasts one tick, so its D kick scales with the rate.
    PID slow(0.0f, 0.01f, 500.0f);
    PID fast(0.0f, 0.01f, 1000.0f);
    CHECK(slow.update(0.0f, -1.0f) == doctest::Approx(5.0f));
    CHECK(fast.update(0.0f, -1.0f) == doctest::Approx(10.0f));
}
