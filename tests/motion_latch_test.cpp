// Motion's command latch: main-loop mutators queue commands that the control
// tick applies, in issue order, at the top of the next Motion::update().
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
// Motion holds a ~100 KB trace buffer, so tests keep the rig on the heap.
struct Rig
{
    fake::Robot robot{MOTOR_KM, MOTOR_TM};
    Drivetrain dt{&robot.left_motor, &robot.right_motor, &robot.left_encoder, &robot.right_encoder};
    Motion     motion{&dt, &robot.imu};

    void tick()
    {
        motion.update();
        robot.step();
    }
};

// Clears the mid-tick hook even when a CHECK throws.
struct HookGuard
{
    ~HookGuard() { fake::setImuUpdateHook({}); }
};
} // namespace

TEST_CASE("a command issued during a tick takes effect whole on the next tick")
{
    // Two identical robots cruise at 400 mm/s. Robot A is told to stop while a
    // tick is in progress (as when the timer interrupt preempts the main loop
    // mid-call); robot B is told between ticks, after that same tick ends.
    // With the latch both apply the stop at the top of the following tick, so
    // their motor commands match on every tick. Without it, A's tick in
    // progress would run on a half-applied or early stop.
    HookGuard guard;
    auto      a = std::make_unique<Rig>();
    auto      b = std::make_unique<Rig>();
    a->motion.start_move(2000.0f, 400.0f, 400.0f, 3000.0f);
    b->motion.start_move(2000.0f, 400.0f, 400.0f, 3000.0f);

    const int cruise_ticks = static_cast<int>(LOOP_FREQUENCY_HZ / 2.0f);
    for (int i = 0; i < cruise_ticks; i++)
    {
        a->tick();
        b->tick();
    }

    bool issued = false;
    fake::setImuUpdateHook(
        [&]
        {
            if (!issued)
            {
                issued = true;
                a->motion.stop_after(50.0f);
            }
        });
    a->tick();
    fake::setImuUpdateHook({});
    REQUIRE(issued);
    b->tick();
    b->motion.stop_after(50.0f);

    CHECK(fake::motorDuty(a->robot.left_motor) == fake::motorDuty(b->robot.left_motor));
    CHECK(fake::motorDuty(a->robot.right_motor) == fake::motorDuty(b->robot.right_motor));
    CHECK(a->motion.position() == b->motion.position());

    for (int i = 0; i < static_cast<int>(LOOP_FREQUENCY_HZ); i++)
    {
        a->tick();
        b->tick();
        REQUIRE(fake::motorDuty(a->robot.left_motor) == fake::motorDuty(b->robot.left_motor));
        REQUIRE(fake::motorDuty(a->robot.right_motor) == fake::motorDuty(b->robot.right_motor));
    }
    // Both stopped about 50 mm after the command, not at the 2000 mm target.
    CHECK(a->motion.move_finished());
    CHECK(a->motion.velocity() == 0.0f);
    CHECK(a->robot.distanceMm() < 400.0f);
}

TEST_CASE("commands issued before one tick apply in issue order")
{
    // FIFO, not last-wins: every command issued between two ticks applies,
    // oldest first.
    auto r = std::make_unique<Rig>();
    r->motion.set_position(100.0f);
    r->motion.adjust_forward_position(-30.0f);
    r->tick();
    CHECK(r->motion.position() == doctest::Approx(70.0f));

    // begin_motion_sequence() then start_move(): the reset must not erase the
    // move queued behind it.
    r->motion.begin_motion_sequence();
    r->motion.start_move(90.0f, 300.0f, 0.0f, 2000.0f);
    r->tick();
    CHECK_FALSE(r->motion.move_finished());
    for (int i = 0; i < 2 * static_cast<int>(LOOP_FREQUENCY_HZ); i++)
        r->tick();
    CHECK(r->motion.move_finished());
    CHECK(std::fabs(r->robot.distanceMm() - 90.0f) < 3.0f);
    r->motion.end_motion_sequence();
}

TEST_CASE("main-loop reads see the loop's own commands before the tick applies them")
{
    // Search code polls position() and move_finished() straight after a
    // command; those reads must already include it.
    auto r = std::make_unique<Rig>();
    r->motion.start_move(180.0f, 300.0f, 0.0f, 2000.0f);
    CHECK_FALSE(r->motion.move_finished());
    r->motion.start_turn(90.0f, 180.0f, 0.0f, 1800.0f);
    CHECK_FALSE(r->motion.turn_finished());

    r->motion.set_position(250.0f);
    CHECK(r->motion.position() == 250.0f);
    r->motion.adjust_forward_position(-180.0f);
    CHECK(r->motion.position() == doctest::Approx(70.0f));

    for (int i = 0; i < 20; i++)
        r->tick();
    REQUIRE(r->motion.velocity() > 0.0f);
    r->motion.reset_drive_control();
    CHECK(r->motion.move_finished());
    CHECK(r->motion.turn_finished());
    CHECK(r->motion.velocity() == 0.0f);
    CHECK(r->motion.position() == 0.0f);
    r->tick();
    CHECK(r->motion.velocity() == 0.0f);
    CHECK(fake::motorDuty(r->robot.left_motor) == 0.0f);
}

TEST_CASE("a burst of commands longer than the latch still applies every one in order")
{
    auto r = std::make_unique<Rig>();
    r->motion.set_position(0.0f);
    for (int i = 0; i < 100; i++)
        r->motion.adjust_forward_position(1.0f);
    CHECK(r->motion.position() == doctest::Approx(100.0f));
    r->tick();
    CHECK(r->motion.position() == doctest::Approx(100.0f));
}

TEST_CASE("setting reads see queued commands, and resets clear what they always cleared")
{
    auto r = std::make_unique<Rig>();
    r->motion.set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
    r->motion.set_heading_hold(190.0f);
    r->motion.set_line_steering_adjustment_degps(4.0f, true);
    CHECK(r->motion.steeringMode() == tof_wall::SteeringMode::STEER_NORMAL);
    CHECK(r->motion.headingHoldActive());
    CHECK(r->motion.headingHoldTargetYawDeg() == doctest::Approx(-170.0f));
    CHECK(r->motion.lineSteeringValid());
    CHECK(r->motion.lineSteeringAdjustmentDegps() == 4.0f);

    // end_motion_sequence() keeps the steering mode; the other resets do not.
    r->motion.end_motion_sequence();
    CHECK(r->motion.steeringMode() == tof_wall::SteeringMode::STEER_NORMAL);
    CHECK_FALSE(r->motion.headingHoldActive());
    CHECK_FALSE(r->motion.lineSteeringValid());
    r->tick();

    r->motion.set_heading_hold(10.0f);
    r->motion.emergency_stop();
    CHECK(r->motion.steeringMode() == tof_wall::SteeringMode::STEERING_OFF);
    CHECK_FALSE(r->motion.headingHoldActive());
}

TEST_CASE("encoder reads straight after a queued reset start from zero")
{
    // The encoder calibration reads its start count right after emergency_stop().
    auto r = std::make_unique<Rig>();
    r->motion.start_move(100.0f, 300.0f, 0.0f, 2000.0f);
    for (int i = 0; i < static_cast<int>(LOOP_FREQUENCY_HZ); i++)
        r->tick();
    REQUIRE(r->motion.encoder_ticks(WheelSide::LEFT) > 100);

    r->motion.emergency_stop();
    CHECK(r->motion.encoder_ticks(WheelSide::LEFT) == 0);
    CHECK(r->motion.encoder_ticks(WheelSide::RIGHT) == 0);
    CHECK(r->motion.wheel_position(WheelSide::LEFT) == 0.0f);
    r->tick();
    CHECK(r->motion.encoder_ticks(WheelSide::LEFT) == 0);
}
