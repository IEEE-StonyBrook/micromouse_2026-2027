// physim: runs one physics-sim scenario and prints a CSV trace, one row per
// control tick, then a summary line on stderr. Build with `make -C tests physim`.
//
//   physim --scenario straight10|ip90|ss90 [knobs...]
//
// Knobs (defaults are the nominal model; see sim/physim.h):
//   --seed N            random seed (default 1)
//   --speed MMPS        straight cruise speed     --accel MMPS2   straight accel
//   --battery V         battery voltage
//   --friction V        motor/gearbox Coulomb friction as volts, both wheels (default kS)
//   --mu X              traction coefficient, both wheels (--mu-left/--mu-right)
//   --slip-rate HZ      random slip events per wheel per second
//   --slip-duration S   --slip-traction X   (grip left during an event)
//   --counts-per-mm X   encoder resolution, both wheels (62.6 = next mouse)
//   --gyro-noise DEGPS_RTHZ (noise density)  --gyro-bias DEGPS
//   --seam-at MM        enable a seam this far along the path
//   --seam-skew MM  --seam-duration S  --seam-traction X  --seam-encoder MM
//   --seam-pitch DEGPS  --seam-accel MPS2
//   --every N           print every Nth tick (default 1)
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "sim/physim.h"

namespace
{
int usage()
{
    std::fprintf(stderr, "usage: physim --scenario straight10|ip90|ss90 [knobs...]; see "
                         "tests/sim/physim_main.cpp\n");
    return 2;
}
} // namespace

int main(int argc, char** argv)
{
    physim::Params     p = physim::nominal();
    physim::RunOptions opt;
    physim::Scenario   scenario{};
    bool               have_scenario = false;
    int                every         = 1;

    for (int i = 1; i < argc; i++)
    {
        const char* flag = argv[i];
        if (i + 1 >= argc)
            return usage();
        const char* arg = argv[++i];
        const float v   = std::strtof(arg, nullptr);
        if (!std::strcmp(flag, "--scenario"))
        {
            if (!physim::parseScenario(arg, &scenario))
                return usage();
            have_scenario = true;
        }
        else if (!std::strcmp(flag, "--seed"))
            p.seed = std::strtoull(arg, nullptr, 10);
        else if (!std::strcmp(flag, "--speed"))
            opt.speed_mmps = v;
        else if (!std::strcmp(flag, "--accel"))
            opt.accel_mmps2 = v;
        else if (!std::strcmp(flag, "--battery"))
            p.battery_v = v;
        else if (!std::strcmp(flag, "--friction"))
            p.friction_v_left = p.friction_v_right = v;
        else if (!std::strcmp(flag, "--mu"))
            p.mu_left = p.mu_right = v;
        else if (!std::strcmp(flag, "--mu-left"))
            p.mu_left = v;
        else if (!std::strcmp(flag, "--mu-right"))
            p.mu_right = v;
        else if (!std::strcmp(flag, "--slip-rate"))
            p.slip.rate_hz = v;
        else if (!std::strcmp(flag, "--slip-duration"))
            p.slip.duration_s = v;
        else if (!std::strcmp(flag, "--slip-traction"))
            p.slip.traction_scale = v;
        else if (!std::strcmp(flag, "--counts-per-mm"))
            p.counts_per_mm_left = p.counts_per_mm_right = v;
        else if (!std::strcmp(flag, "--gyro-noise"))
            p.gyro_noise_degps_rthz = v;
        else if (!std::strcmp(flag, "--gyro-bias"))
            p.gyro_bias_degps = v;
        else if (!std::strcmp(flag, "--seam-at"))
        {
            p.seam.enabled = true;
            p.seam.at_mm   = v;
        }
        else if (!std::strcmp(flag, "--seam-skew"))
            p.seam.skew_mm = v;
        else if (!std::strcmp(flag, "--seam-duration"))
            p.seam.duration_s = v;
        else if (!std::strcmp(flag, "--seam-traction"))
            p.seam.traction_scale = v;
        else if (!std::strcmp(flag, "--seam-encoder"))
            p.seam.encoder_extra_mm = v;
        else if (!std::strcmp(flag, "--seam-pitch"))
            p.seam.pitch_rate_peak_degps = v;
        else if (!std::strcmp(flag, "--seam-accel"))
            p.seam.accel_z_peak_mps2 = v;
        else if (!std::strcmp(flag, "--every"))
            every = std::atoi(arg) > 0 ? std::atoi(arg) : 1;
        else
            return usage();
    }
    if (!have_scenario)
        return usage();

    std::printf("time_s,target_mm,target_deg,distance_mm,yaw_deg,enc_left_mm,enc_right_mm,"
                "imu_yaw_deg,wheel_left_mmps,wheel_right_mmps,ground_left_mmps,"
                "ground_right_mmps,volts_left,volts_right,slip_left,slip_right,"
                "pitch_rate_degps,accel_z_mps2\n");
    long                 tick = 0;
    const physim::Result r    = physim::run(
        scenario, p, opt,
        [&](const physim::Sample& s)
        {
            if (tick++ % every != 0)
                return;
            std::printf(
                "%.4f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%d,%d,"
                "%.1f,%.2f\n",
                s.time_s, s.target_mm, s.target_deg, s.distance_mm, s.yaw_deg, s.encoder_left_mm,
                s.encoder_right_mm, s.imu_yaw_deg, s.wheel_left_mmps, s.wheel_right_mmps,
                s.ground_left_mmps, s.ground_right_mmps, s.volts_left, s.volts_right, s.slip_left,
                s.slip_right, s.pitch_rate_degps, s.accel_z_mps2);
        });
    std::fprintf(stderr,
                 "finished=%d duration_s=%.3f distance_mm=%.3f yaw_deg=%.3f imu_yaw_deg=%.3f "
                 "encoder_mm=%.3f peak_pitch_rate_degps=%.1f peak_accel_z_mps2=%.2f "
                 "seam_crossings=%d\n",
                 r.finished, r.duration_s, r.distance_mm, r.yaw_deg, r.imu_yaw_deg, r.encoder_mm,
                 r.peak_pitch_rate_degps, r.peak_accel_z_mps2, r.seam_crossings);
    return r.finished ? 0 : 1;
}
