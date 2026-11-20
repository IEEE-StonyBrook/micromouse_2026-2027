// Loop rate: set per platform in config/board.h. make -C tests runs the
// whole suite twice, at the Pico's 500 Hz and, built with
// -DHOST_LOOP_FREQUENCY_HZ=1000.0f, at the STM32's 1 kHz.
#include <memory>

#include "config/motion.h"
#include "config/tuning.h"
#include "control/drivetrain.h"
#include "control/line_follower.h"
#include "control/motion.h"
#include "control/pid.h"
#include "control/run_log.h"
#include "doctest/doctest.h"
#include "fakes/fake_robot.h"

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

TEST_CASE("tick counts derive from a time, not a rate")
{
    // The encoder averager spans 16 ms: 8 taps on the Pico, 16 on the STM32.
    CHECK(ENCODER_AVERAGER_LENGTH * LOOP_INTERVAL_S == doctest::Approx(0.016f));
#ifndef HOST_LOOP_FREQUENCY_HZ
    CHECK(ENCODER_AVERAGER_LENGTH == 8);
#endif
    CHECK(LOOP_TICKS(0.1f) == static_cast<int>(LOOP_FREQUENCY_HZ / 10.0f));
}

TEST_CASE("motion records the run log every 8 ms by default at any rate")
{
    struct Rig
    {
        fake::Robot robot{MOTOR_KM, MOTOR_TM};
        Drivetrain  dt{&robot.left_motor, &robot.right_motor, &robot.left_encoder,
                       &robot.right_encoder};
        Motion      motion{&dt, &robot.imu};
        RunLog      log;
    };
    auto r = std::make_unique<Rig>(); // the run log is ~56 KB
    r->motion.set_run_log(&r->log);
    r->motion.start_move(180.0f, 300.0f, 0.0f, 2000.0f);
    r->motion.start_trace(); // latched: the next tick starts the log
    CHECK_FALSE(r->log.active());
    for (int i = 0; i < LOOP_TICKS(0.080f); i++)
        r->motion.update();
    CHECK(r->log.dtMs() == 8);
    CHECK(r->log.count() == 10); // 80 ms of 8 ms samples
    const RunLog::Sample last = r->log.sample(9);
    CHECK((last.events & RunLog::FORWARD_ACTIVE) != 0);
    CHECK((last.events & RunLog::ROTATION_ACTIVE) == 0);
    CHECK(last.fwd_mmps > 0.0f); // accelerating at 2000 mm/s^2
    CHECK(last.pos_mm > 0.0f);

    r->motion.stop_trace();
    r->motion.update();
    CHECK_FALSE(r->log.active());
    for (int i = 0; i < LOOP_TICKS(0.080f); i++)
        r->motion.update();
    CHECK(r->log.count() == 10); // frozen for the dump
}

TEST_CASE("motion without a run log records nothing")
{
    struct Rig
    {
        fake::Robot robot{MOTOR_KM, MOTOR_TM};
        Drivetrain  dt{&robot.left_motor, &robot.right_motor, &robot.left_encoder,
                       &robot.right_encoder};
        Motion      motion{&dt, &robot.imu};
        RunLog      log;
    };
    auto r = std::make_unique<Rig>();
    r->log.start(); // never handed to Motion
    r->motion.start_move(180.0f, 300.0f, 0.0f, 2000.0f);
    for (int i = 0; i < LOOP_TICKS(0.080f); i++)
        r->motion.update();
    CHECK(r->log.count() == 0);
}

TEST_CASE("line error filter settles in the same time at 500 Hz and 1 kHz")
{
    // The tuned 500 Hz filter keeps its exact gain on the Pico.
    CHECK(lineErrorFilterAlpha(500.0f) == LINE_ERROR_FILTER_ALPHA_500HZ);

    // Step response 10 ms in: 1 - 0.55^5 at 500 Hz, the same at 1 kHz.
    for (float hz : {500.0f, 1000.0f})
    {
        const float alpha    = lineErrorFilterAlpha(hz);
        float       filtered = 0.0f;
        for (int i = 0; i < static_cast<int>(0.010f * hz + 0.5f); i++)
            filtered += alpha * (1.0f - filtered);
        CHECK(filtered == doctest::Approx(0.94967f).epsilon(1e-4));
    }
}
