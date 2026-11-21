// MotorBridge: DRV8874 drive, brake, sleep and wake, and IPROPI current, against
// a fake port. Times come from the datasheet (tWAKE 1 ms) and config, so the
// same cases hold at 500 Hz and 1 kHz.
#include <memory>

#include "config/config.h"
#include "control/drivetrain.h"
#include "control/motion.h"
#include "doctest/doctest.h"
#include "drivers/motor_bridge.h"
#include "fakes/fake_h_bridge.h"
#include "fakes/fake_robot.h"

namespace
{
constexpr float TICK_S = 1.0f / LOOP_FREQUENCY_HZ;

// 3.3 V 12-bit ADC across 2.2 kOhm: 1 A of motor current is 450 uA of
// IPROPI, 0.99 V, 1228.8 counts.
constexpr float AMPS_PER_COUNT = ipropiAmpsPerCount(3.3f, 4096.0f, 2200.0f, 450.0f);

bool coasting(const fake::HBridge::Inputs& in)
{
    return in.in1_high == 0.0f && in.in2_high == 0.0f;
}

bool braking(const fake::HBridge::Inputs& in)
{
    return in.in1_high == 1.0f && in.in2_high == 1.0f;
}

void run(MotorBridge& b, int ticks)
{
    for (int i = 0; i < ticks; i++)
        b.update();
}

// Wakes the bridge and waits out tWAKE; returns with it braking.
void wakeUp(MotorBridge& b)
{
    b.wake();
    run(b, MotorBridge::WAKE_TICKS + 1);
}
} // namespace

TEST_CASE("motor bridge: starts asleep with nSLEEP low and the inputs off")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    CHECK(b.state() == MotorBridge::State::kAsleep);
    CHECK_FALSE(port.awake);
    CHECK(coasting(port.inputs[0]));
    CHECK(coasting(port.inputs[1]));
}

TEST_CASE("motor bridge: a command given asleep waits out tWAKE, then lands whole")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    b.set_duty(0.5f, -0.25f); // given once, never repeated
    b.update();
    CHECK(port.awake); // nSLEEP rises on the first tick
    int ticks = 1;
    while (b.state() != MotorBridge::State::kAwake && ticks < 100)
    {
        // The driver ignores its inputs until tWAKE: keep them quiet.
        CHECK(coasting(port.inputs[0]));
        CHECK(coasting(port.inputs[1]));
        b.update();
        ticks++;
    }
    // Lands strictly after tWAKE (1 ms), at most one tick later.
    const float waited_s = static_cast<float>(ticks - 1) * TICK_S;
    CHECK(waited_s > 0.001f);
    CHECK(waited_s <= 0.001f + TICK_S + 1e-6f);
    CHECK(port.inputs[0].in1_high == 1.0f);
    CHECK(port.inputs[0].in2_high == doctest::Approx(0.5f));
    CHECK(port.inputs[1].in1_high == doctest::Approx(0.75f));
    CHECK(port.inputs[1].in2_high == 1.0f);
}

TEST_CASE("motor bridge: wake() on arming brakes once awake, so the first move need not wait")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    wakeUp(b);
    CHECK(b.state() == MotorBridge::State::kAwake);
    CHECK(port.awake);
    CHECK(braking(port.inputs[0])); // both inputs high: low sides on, the wheel held
    CHECK(braking(port.inputs[1]));

    b.set_duty(1.0f, -1.0f); // applied on the very next tick
    b.update();
    CHECK(port.inputs[0].in1_high == 1.0f);
    CHECK(port.inputs[0].in2_high == 0.0f);
    CHECK(port.inputs[1].in1_high == 0.0f);
    CHECK(port.inputs[1].in2_high == 1.0f);
}

TEST_CASE(
    "motor bridge: braked with no command for the idle time, it sleeps; driving keeps it awake")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    b.set_duty(0.2f, 0.2f);
    run(b, LOOP_TICKS(3.0f)); // a long move: never idle
    CHECK(b.state() == MotorBridge::State::kAwake);

    b.set_duty(0.0f, 0.0f);
    run(b, LOOP_TICKS(MOTOR_IDLE_SLEEP_TIME_S) - 1);
    CHECK(b.state() == MotorBridge::State::kAwake);
    CHECK(braking(port.inputs[0]));
    b.update();
    CHECK(b.state() == MotorBridge::State::kAsleep);
    CHECK_FALSE(port.awake);

    // A zero command keeps it asleep; the next real one wakes it.
    run(b, LOOP_TICKS(1.0f));
    CHECK_FALSE(port.awake);
    b.set_duty(0.0f, -0.4f);
    run(b, MotorBridge::WAKE_TICKS + 1);
    CHECK(b.state() == MotorBridge::State::kAwake);
    CHECK(port.inputs[1].in1_high == doctest::Approx(0.6f));
}

TEST_CASE("motor bridge: sleep() cuts at once and drops the command (disarm, crash, flash save)")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    b.set_duty(0.8f, 0.8f);
    run(b, MotorBridge::WAKE_TICKS + 1);
    REQUIRE(port.awake);

    b.sleep(); // no tick needed
    CHECK_FALSE(port.awake);
    CHECK_FALSE(b.awake());
    CHECK(coasting(port.inputs[0]));
    CHECK(coasting(port.inputs[1]));
    run(b, LOOP_TICKS(0.5f));
    CHECK_FALSE(port.awake); // the old 0.8 does not come back
}

TEST_CASE("motor bridge: IPROPI counts become amps, less the offset learned asleep")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    float l = -1.0f, r = -1.0f;

    // Asleep: the ADC reads its own offset, 30 counts; no current is reported.
    port.counts[0] = 30;
    port.counts[1] = 30;
    run(b, LOOP_TICKS(0.2f));
    CHECK_FALSE(MotorBridge::readAmps(&b, &l, &r));
    CHECK(b.amps(0) == 0.0f);

    // Awake: 1 A is 1228.8 counts above the offset; 0.5 A is 614.4.
    b.set_duty(0.5f, 0.5f);
    port.counts[0] = 30 + 1229;
    port.counts[1] = 30 + 614;
    run(b, MotorBridge::WAKE_TICKS + 1);
    REQUIRE(MotorBridge::readAmps(&b, &l, &r));
    CHECK(l == doctest::Approx(1.0f).epsilon(0.005));
    CHECK(r == doctest::Approx(0.5f).epsilon(0.005));

    // Below the offset (noise) reads as zero, never negative.
    port.counts[0] = 20;
    b.update();
    CHECK(b.amps(0) == 0.0f);
}

TEST_CASE("motor bridge: the offset is learned only after tSLEEP, not from a still-active driver")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    b.set_duty(0.5f, 0.5f);
    run(b, MotorBridge::WAKE_TICKS + 1);
    port.counts[0] = 900; // still flowing as nSLEEP falls
    b.sleep();
    b.update(); // within tSLEEP of nSLEEP low
    port.counts[0] = 30;
    run(b, LOOP_TICKS(0.005f));
    b.set_duty(0.5f, 0.5f);
    port.counts[0] = 30 + 1229;
    run(b, MotorBridge::WAKE_TICKS + 1);
    CHECK(b.amps(0) == doctest::Approx(1.0f).epsilon(0.005));
}

namespace
{
// Motion's crash cut hook takes no context, so a board keeps its bridge in a
// file-scope pointer, as this does.
MotorBridge* cut_bridge = nullptr;
void         sleepDrivers()
{
    cut_bridge->sleep();
}
} // namespace

TEST_CASE("motor bridge: measured stall current trips the crash watchdog, which sleeps the drivers")
{
    // A mouse at rest (fake plant, no motion) whose wheels the drivers push
    // against something: IPROPI reads 1.5 A, above CRASH_STALL_AMPS.
    fake::Robot robot(MOTOR_KM, MOTOR_TM);
    Drivetrain dt(&robot.left_motor, &robot.right_motor, &robot.left_encoder, &robot.right_encoder);
    auto       motion = std::make_unique<Motion>(&dt, &robot.imu);
    fake::HBridge port;
    MotorBridge   bridge(port, AMPS_PER_COUNT);
    bridge.arm();
    cut_bridge = &bridge;
    motion->set_motor_current_reader(MotorBridge::readAmps, &bridge);
    motion->set_crash_cut_hook(sleepDrivers);
    motion->crashDetector().set_enabled(true);

    bridge.set_duty(0.3f, 0.3f);
    port.counts[0] = static_cast<uint16_t>(1.5f / AMPS_PER_COUNT);
    int ticks      = 0;
    while (!motion->crashDetector().tripped() && ticks < LOOP_TICKS(1.0f))
    {
        bridge.update();
        motion->update();
        robot.step();
        ticks++;
    }
    REQUIRE(motion->crashDetector().tripped());
    CHECK(motion->crashDetector().cause() == CrashCause::kStall);
    // Waking takes tWAKE, then the stall must hold for its whole window.
    CHECK(ticks <= MotorBridge::WAKE_TICKS + LOOP_TICKS(CRASH_STALL_TIME_S) + 1);
    CHECK_FALSE(port.awake);
    CHECK(coasting(port.inputs[0]));
    CHECK(coasting(port.inputs[1]));
    cut_bridge = nullptr;
}

TEST_CASE("motor bridge: wake() while already awake does not undo the next idle sleep")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    wakeUp(b);
    b.wake(); // armed again mid-run
    run(b, LOOP_TICKS(MOTOR_IDLE_SLEEP_TIME_S) + LOOP_TICKS(0.05f));
    CHECK(b.state() == MotorBridge::State::kAsleep);
    CHECK_FALSE(port.awake);
}

TEST_CASE("motor bridge: duty past full scale is clamped to it")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    b.set_duty(1.5f, -3.0f);
    run(b, MotorBridge::WAKE_TICKS + 1);
    CHECK(port.inputs[0].in2_high == 0.0f);
    CHECK(port.inputs[1].in1_high == 0.0f);
}

TEST_CASE("motor bridge: disarmed, it stays asleep and drops every command")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT); // built disarmed
    b.set_duty(0.5f, 0.5f);
    b.set_motor_duty(1, -0.5f);
    b.wake();
    run(b, LOOP_TICKS(0.1f));
    CHECK(b.state() == MotorBridge::State::kAsleep);
    CHECK_FALSE(port.awake);
    CHECK(coasting(port.inputs[0]));
    CHECK(coasting(port.inputs[1]));

    // Armed and woken, the drivers brake: the commands given disarmed are gone.
    CHECK_FALSE(b.armed());
    b.arm();
    CHECK(b.armed());
    b.wake();
    run(b, MotorBridge::WAKE_TICKS + 1);
    CHECK(b.state() == MotorBridge::State::kAwake);
    CHECK(braking(port.inputs[0]));
    CHECK(braking(port.inputs[1]));
}

TEST_CASE("motor bridge: disarm() cuts at once and refuses commands until arm()")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    b.set_duty(0.8f, 0.8f);
    run(b, MotorBridge::WAKE_TICKS + 1);
    REQUIRE(port.awake);

    b.disarm(); // no tick needed
    CHECK_FALSE(b.armed());
    CHECK_FALSE(port.awake);
    CHECK(coasting(port.inputs[0]));
    b.set_duty(0.3f, 0.3f);
    run(b, LOOP_TICKS(0.1f));
    CHECK_FALSE(port.awake);
}

TEST_CASE("motor bridge: one motor's duty is set without touching the other's")
{
    fake::HBridge port;
    MotorBridge   b(port, AMPS_PER_COUNT);
    b.arm();
    b.set_duty(0.5f, -0.25f);
    b.set_motor_duty(1, 0.4f);
    b.set_motor_duty(0, 2.0f); // clamped like set_duty
    run(b, MotorBridge::WAKE_TICKS + 1);
    CHECK(port.inputs[0].in1_high == 1.0f);
    CHECK(port.inputs[0].in2_high == 0.0f);
    CHECK(port.inputs[1].in1_high == 1.0f);
    CHECK(port.inputs[1].in2_high == doctest::Approx(0.6f));
}
