// Motion closed loop: profile + PD + feedforward driving the fake plant.
#include <cmath>
#include <memory>

#include "config/motion.h"
#include "config/tuning.h"
#include "control/drivetrain.h"
#include "control/motion.h"
#include "doctest/doctest.h"
#include "fakes/fake_robot.h"

namespace
{
struct Rig
{
    fake::Robot robot;
    Drivetrain  dt;
    Motion      motion;

    Rig(float km, float tm)
        : robot(km, tm),
          dt(&robot.left_motor, &robot.right_motor, &robot.left_encoder, &robot.right_encoder),
          motion(&dt, &robot.imu)
    {
    }

    // One control tick, then the plant moves for one tick on those volts.
    void tick()
    {
        motion.update();
        robot.step();
    }

    // Runs until the move finishes, then lets the robot coast to rest.
    int runMove(int guard_ticks)
    {
        int ticks = 0;
        while (!motion.move_finished() && ticks < guard_ticks)
        {
            tick();
            ticks++;
        }
        for (int i = 0; i < static_cast<int>(LOOP_FREQUENCY_HZ); i++)
            tick();
        return ticks;
    }
};

constexpr int GUARD_TICKS = 10 * static_cast<int>(LOOP_FREQUENCY_HZ);
} // namespace

TEST_CASE("motion drives a straight move to rest within tolerance")
{
    // Plant matched to the tuned model: feedforward is nearly right on its own.
    auto r = std::make_unique<Rig>(MOTOR_KM, MOTOR_TM);
    r->motion.start_move(540.0f, 600.0f, 0.0f, 3000.0f);
    const int ticks = r->runMove(GUARD_TICKS);

    REQUIRE(ticks < GUARD_TICKS);
    CHECK(std::fabs(r->robot.distanceMm() - 540.0f) < 3.0f);
    CHECK(std::fabs(r->robot.yawDeg()) < 1.0f);
    CHECK(std::fabs(r->robot.wheelMmps(true)) < 1.0f);
    CHECK(std::fabs(r->robot.wheelMmps(false)) < 1.0f);
    CHECK(fake::motorDuty(r->robot.left_motor) == 0.0f);
    CHECK(fake::motorDuty(r->robot.right_motor) == 0.0f);
}

TEST_CASE("motion feedback corrects a plant weaker than its model")
{
    // 20% less gain and a slower motor than the feedforward assumes: the
    // PD has to make up the difference.
    auto r = std::make_unique<Rig>(0.8f * MOTOR_KM, 1.25f * MOTOR_TM);
    r->motion.start_move(360.0f, 500.0f, 0.0f, 2500.0f);
    const int ticks = r->runMove(GUARD_TICKS);

    REQUIRE(ticks < GUARD_TICKS);
    CHECK(std::fabs(r->robot.distanceMm() - 360.0f) < 3.0f);
    CHECK(std::fabs(r->robot.yawDeg()) < 1.0f);
}

TEST_CASE("motion reverses for a negative move")
{
    auto r = std::make_unique<Rig>(MOTOR_KM, MOTOR_TM);
    r->motion.start_move(-180.0f, 400.0f, 0.0f, 2500.0f);
    r->runMove(GUARD_TICKS);
    CHECK(std::fabs(r->robot.distanceMm() + 180.0f) < 3.0f);
}

TEST_CASE("motion tracks the profile while moving")
{
    auto r = std::make_unique<Rig>(MOTOR_KM, MOTOR_TM);
    r->motion.start_move(540.0f, 600.0f, 0.0f, 3000.0f);
    float worst_lag = 0.0f;
    while (!r->motion.move_finished())
    {
        r->tick();
        worst_lag = std::fmax(worst_lag, std::fabs(r->motion.position() - r->robot.distanceMm()));
    }
    // True position trails the profile by the encoder averaging delay and the
    // motor lag, a few millimetres at 600 mm/s, never more.
    CHECK(worst_lag < 8.0f);
}
