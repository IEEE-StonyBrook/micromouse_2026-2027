// Loop rate: the same move must come out the same at 500 Hz (Pico) and 1 kHz
// (STM32). make -C tests runs this suite twice: the 500 Hz run writes its
// trajectories to RATE_TRAJ_OUT, and the 1 kHz run (built with
// -DHOST_LOOP_FREQUENCY_HZ=1000.0f) reads them back from RATE_TRAJ_REF and
// compares sample by sample.
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <string>

#include "config/motion.h"
#include "config/tuning.h"
#include "control/drivetrain.h"
#include "control/motion.h"
#include "control/profile.h"
#include "doctest/doctest.h"
#include "fakes/fake_robot.h"

namespace
{
// Trajectories are compared every 10 ms, a whole number of ticks at both rates.
constexpr float kSamplePeriodS = 0.010f;

struct Sample
{
    float value;
    float tolerance;
};

// Key: "<series> <sample index>".
using Trajectory = std::map<std::string, Sample>;

void record(Trajectory& traj, const char* series, int index, float value, float tolerance)
{
    traj[std::string(series) + " " + std::to_string(index)] = {value, tolerance};
}

int ticksPerSample()
{
    return static_cast<int>(std::lround(kSamplePeriodS * LOOP_FREQUENCY_HZ));
}

void recordProfile(Trajectory& traj)
{
    Profile p;
    p.start(1000.0f, 800.0f, 0.0f, 4000.0f);
    const int per = ticksPerSample();
    for (int tick = 1; !p.finished() && tick < 10 * static_cast<int>(LOOP_FREQUENCY_HZ); tick++)
    {
        p.update();
        if (tick % per == 0)
        {
            // One tick of travel at 800 mm/s is 1.6 mm at 500 Hz.
            record(traj, "profile_pos_mm", tick / per, p.position(), 2.0f);
            record(traj, "profile_vel_mmps", tick / per, p.velocity(), 10.0f);
        }
    }
}

// Motion holds a ~100 KB trace buffer, so the rig lives on the heap.
struct Rig
{
    fake::Robot robot{MOTOR_KM, MOTOR_TM};
    Drivetrain dt{&robot.left_motor, &robot.right_motor, &robot.left_encoder, &robot.right_encoder};
    Motion     motion{&dt, &robot.imu};
};

void recordStraight(Trajectory& traj)
{
    auto r = std::make_unique<Rig>();
    r->motion.start_move(540.0f, 600.0f, 0.0f, 3000.0f);
    const int per   = ticksPerSample();
    const int total = static_cast<int>(1.5f * LOOP_FREQUENCY_HZ); // move takes ~1.1 s
    for (int tick = 1; tick <= total; tick++)
    {
        r->motion.update();
        r->robot.step();
        if (tick % per == 0)
        {
            record(traj, "straight_pos_mm", tick / per, r->robot.distanceMm(), 2.0f);
            record(traj, "straight_vel_mmps", tick / per,
                   0.5f * (r->robot.wheelMmps(true) + r->robot.wheelMmps(false)), 25.0f);
            record(traj, "straight_yaw_deg", tick / per, r->robot.yawDeg(), 0.5f);
        }
    }
}

void write(const Trajectory& traj, const char* path)
{
    std::ofstream out(path);
    for (const auto& [key, s] : traj)
        out << key << " " << s.value << "\n";
}

Trajectory read(const char* path)
{
    Trajectory    traj;
    std::ifstream in(path);
    std::string   series;
    int           index = 0;
    float         value = 0.0f;
    while (in >> series >> index >> value)
        record(traj, series.c_str(), index, value, 0.0f);
    return traj;
}
} // namespace

TEST_CASE("profile and closed-loop straight trajectories match across loop rates")
{
    Trajectory traj;
    recordProfile(traj);
    recordStraight(traj);
    REQUIRE(traj.size() > 300);

    if (const char* out = std::getenv("RATE_TRAJ_OUT"))
        write(traj, out);

    const char* ref_path = std::getenv("RATE_TRAJ_REF");
    if (ref_path == nullptr)
    {
        // Only the reference run may skip the comparison.
        CHECK(LOOP_FREQUENCY_HZ == 500.0f);
        return;
    }
    const Trajectory ref = read(ref_path);
    REQUIRE(ref.size() > 300);
    int compared = 0;
    for (const auto& entry : traj)
    {
        const std::string& key = entry.first;
        const Sample&      s   = entry.second;
        const auto         it  = ref.find(key);
        if (it == ref.end())
            continue; // a series that ran a sample longer at one rate
        INFO(key << ": " << s.value << " vs 500 Hz " << it->second.value);
        CHECK(std::fabs(s.value - it->second.value) <= s.tolerance);
        compared++;
    }
    CHECK(compared > 300);
}
