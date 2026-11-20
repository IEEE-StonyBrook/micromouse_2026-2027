// Drivetrain on fake encoders and motors: what it reports and commands.
#include "config/config.h"
#include "config/geometry.h"
#include "config/motion.h"
#include "config/tuning.h"
#include "control/drivetrain.h"
#include "doctest/doctest.h"
#include "fakes/fake_robot.h"

namespace
{
struct Rig
{
    fake::Robot robot{MOTOR_KM, MOTOR_TM};
    Drivetrain dt{&robot.left_motor, &robot.right_motor, &robot.left_encoder, &robot.right_encoder};
    int        left  = 0;
    int        right = 0;

    Rig() { dt.reset(); }

    // Moves each wheel by the given counts, then runs one Drivetrain tick.
    void tick(int d_left, int d_right)
    {
        left += d_left;
        right += d_right;
        fake::setEncoderCount(robot.left_encoder, left);
        fake::setEncoderCount(robot.right_encoder, right);
        dt.update();
    }
};
} // namespace

TEST_CASE("drivetrain reports wheel travel in counts and millimetres")
{
    Rig r;
    r.tick(100, -40);
    CHECK(r.dt.ticks(WheelSide::LEFT) == 100);
    CHECK(r.dt.ticks(WheelSide::RIGHT) == -40);
    CHECK(r.dt.position(WheelSide::LEFT) == doctest::Approx(100 * MM_PER_TICK_LEFT));
    CHECK(r.dt.position(WheelSide::RIGHT) == doctest::Approx(-40 * MM_PER_TICK_RIGHT));

    r.dt.reset();
    CHECK(r.dt.ticks(WheelSide::LEFT) == 0);
    CHECK(r.dt.position(WheelSide::RIGHT) == 0.0f);
}

TEST_CASE("drivetrain velocity settles over the averaging window")
{
    Rig         r;
    const float full = 10 * MM_PER_TICK_LEFT * LOOP_FREQUENCY_HZ;
    r.tick(10, 10);
    // One tick in: only 1/8 of the window holds motion.
    CHECK(r.dt.velocity(WheelSide::LEFT) == doctest::Approx(full / ENCODER_AVERAGER_LENGTH));
    for (int i = 1; i < ENCODER_AVERAGER_LENGTH; i++)
        r.tick(10, 10);
    CHECK(r.dt.velocity(WheelSide::LEFT) == doctest::Approx(full));
    CHECK(r.dt.fwdChangeMm() == doctest::Approx(10 * MM_PER_TICK));
    CHECK(r.dt.rotChangeDeg() == doctest::Approx(0.0f));
}

TEST_CASE("drivetrain velocity is exact at +-200 counts per tick")
{
    // ~200 counts/tick is full speed; the history must not wrap there.
    for (int counts : {200, -200})
    {
        Rig r;
        for (int i = 0; i < 2 * ENCODER_AVERAGER_LENGTH; i++)
            r.tick(counts, counts);
        const float expected = counts * MM_PER_TICK_LEFT * LOOP_FREQUENCY_HZ;
        CHECK(r.dt.velocity(WheelSide::LEFT) == doctest::Approx(expected));
        CHECK(r.dt.velocity(WheelSide::RIGHT) == doctest::Approx(expected));
        CHECK(r.dt.fwdChangeMm() == doctest::Approx(counts * MM_PER_TICK));
    }
}

TEST_CASE("drivetrain yaw change is positive when the right wheel leads")
{
    Rig r;
    for (int i = 0; i < ENCODER_AVERAGER_LENGTH; i++)
        r.tick(-5, 5);
    CHECK(r.dt.rotChangeDeg() > 0.0f);
    CHECK(r.dt.fwdChangeMm() == doctest::Approx(0.0f));
    CHECK(r.dt.rotChangeDeg() == doctest::Approx(10 * MM_PER_TICK * DEG_PER_MM_DIFFERENCE));
}

TEST_CASE("drivetrain feedforward follows speed sign and adds friction")
{
    Rig r;
    r.dt.setFeedforward(WheelSide::LEFT, 0.01f, 0.5f, 0.001f);
    CHECK(r.dt.feedforward(WheelSide::LEFT, 0.0f) == 0.0f);
    CHECK(r.dt.feedforward(WheelSide::LEFT, 100.0f) == doctest::Approx(1.0f + 0.5f));
    CHECK(r.dt.feedforward(WheelSide::LEFT, -100.0f) == doctest::Approx(-1.0f - 0.5f));
    CHECK(r.dt.feedforward(WheelSide::LEFT, 100.0f, 1000.0f) == doctest::Approx(2.5f));
}

TEST_CASE("drivetrain scales volts by the battery and stops the motors")
{
    Rig r;
    r.dt.setVoltage(DEFAULT_BATTERY_VOLTAGE / 2.0f, -DEFAULT_BATTERY_VOLTAGE / 4.0f);
    CHECK(fake::motorDuty(r.robot.left_motor) == doctest::Approx(0.5f));
    CHECK(fake::motorDuty(r.robot.right_motor) == doctest::Approx(-0.25f));
    r.dt.stop();
    CHECK(fake::motorDuty(r.robot.left_motor) == 0.0f);
    CHECK(fake::motorDuty(r.robot.right_motor) == 0.0f);
}
