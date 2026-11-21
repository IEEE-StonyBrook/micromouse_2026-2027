// physim: runs one physics-sim scenario and prints a CSV trace, one row per
// control tick, then a summary line on stderr. Build with `make -C tests physim`.
//
//   physim --scenario straight10|ip90|ss90 [knobs...]
//   physim --script "<PATH args>" [knobs...]    the Pico's PATH on an open floor,
//                       e.g. --script "FFRFFLF 600 3000 SMOOTH" or a TRACE cmd= text
//   physim --replay-volts FILE [--run N] [knobs...]
//                       open loop: hold each logged left_v/right_v on the plant
//                       (battery from the header's batt_v, else --battery) and
//                       print a TRACE of the plant's wheel speeds; stderr gives
//                       their RMS gap to the logged ones
//
//   --format csv|trace  csv (default): one row per tick; trace: the firmware's
//                       TRACE dump (RunLog rows plus true_dist_mm,true_yaw_deg)
//   --emit-pico-trace   trace, wrapped as a serial_term capture of the Pico
//                       ("[ms] [INFO] TRACE ...") for tools/tracelog.py
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
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "sim/physim.h"

namespace
{
int usage()
{
    std::fprintf(stderr, "usage: physim --scenario straight10|ip90|ss90 | --script \"<PATH "
                         "args>\" | --replay-volts FILE [knobs...]; see "
                         "tests/sim/physim_main.cpp\n");
    return 2;
}

// Prints a TRACE dump, raw or as serial_term saves a Pico session: "[ms] "
// stamps, the typed commands echoed, "[INFO] " on the dump's lines, and the
// rows paced as the Pico paces them over Bluetooth (~8 ms a row).
void printTrace(const std::vector<std::string>& trace, bool pico, const std::string& typed,
                float run_s)
{
    if (!pico)
    {
        for (const std::string& line : trace)
            std::printf("%s\n", line.c_str());
        return;
    }
    double t_ms = 1000.0;
    std::printf("[%10.1f] >>> [serial] %s\n", t_ms, typed.c_str());
    t_ms += run_s * 1000.0 + 500.0;
    std::printf("[%10.1f] >>> [serial] TRACE\n", t_ms);
    for (const std::string& line : trace)
    {
        t_ms += 8.0;
        std::printf("[%10.1f] [INFO] %s\n", t_ms, line.c_str());
    }
}
} // namespace

int main(int argc, char** argv)
{
    physim::Params     p = physim::nominal();
    physim::RunOptions opt;
    physim::Scenario   scenario{};
    bool               have_scenario = false;
    int                every         = 1;
    bool               trace_format  = false;
    bool               pico          = false;
    const char*        script        = nullptr;
    const char*        replay_file   = nullptr;
    int                replay_run    = 0;
    bool               battery_set   = false;

    for (int i = 1; i < argc; i++)
    {
        const char* flag = argv[i];
        if (!std::strcmp(flag, "--emit-pico-trace"))
        {
            trace_format = pico = true;
            continue;
        }
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
        else if (!std::strcmp(flag, "--script"))
            script = arg;
        else if (!std::strcmp(flag, "--replay-volts"))
            replay_file = arg;
        else if (!std::strcmp(flag, "--run"))
            replay_run = std::atoi(arg);
        else if (!std::strcmp(flag, "--format"))
        {
            if (!std::strcmp(arg, "trace"))
                trace_format = true;
            else if (std::strcmp(arg, "csv") != 0)
                return usage();
        }
        else if (!std::strcmp(flag, "--seed"))
            p.seed = std::strtoull(arg, nullptr, 10);
        else if (!std::strcmp(flag, "--speed"))
            opt.speed_mmps = v;
        else if (!std::strcmp(flag, "--accel"))
            opt.accel_mmps2 = v;
        else if (!std::strcmp(flag, "--battery"))
        {
            p.battery_v = v;
            battery_set = true;
        }
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
    if (have_scenario + (script != nullptr) + (replay_file != nullptr) != 1)
        return usage();

    if (replay_file != nullptr)
    {
        std::ifstream      in(replay_file);
        std::ostringstream text;
        text << in.rdbuf();
        physim::TraceRun logged;
        std::string      error;
        if (!in || !physim::parseTrace(text.str(), replay_run, &logged, &error))
        {
            std::fprintf(stderr, "physim: %s: %s\n", replay_file,
                         in ? error.c_str() : "unreadable");
            return 2;
        }
        if (logged.wrapped)
            std::fprintf(stderr, "physim: warning: the log wrapped; the replay starts the plant "
                                 "at rest mid-run\n");
        // --battery overrides the header's batt_v.
        if (battery_set)
            logged.batt_v = p.battery_v;
        std::vector<std::string>   trace;
        const physim::ReplayResult r = physim::replayVolts(logged, p, &trace);
        printTrace(trace, pico, "REPLAY-VOLTS", r.samples * logged.dt_ms / 1000.0f);
        std::fprintf(stderr, "samples=%d rms_left_mmps=%.2f rms_right_mmps=%.2f\n", r.samples,
                     r.rms_left_mmps, r.rms_right_mmps);
        return 0;
    }

    physim::PathCommand path;
    if (script != nullptr)
    {
        std::string error;
        if (!physim::parsePathCommand(script, &path, &error))
        {
            std::fprintf(stderr, "physim: %s\n", error.c_str());
            return 2;
        }
    }
    const auto simulate = [&](const std::function<void(const physim::Sample&)>& on_sample)
    {
        return script != nullptr ? physim::runPath(path, p, opt, on_sample)
                                 : physim::run(scenario, p, opt, on_sample);
    };

    physim::Result r{};
    if (trace_format)
    {
        std::vector<std::string> trace;
        opt.trace = &trace;
        r         = simulate(nullptr);
        printTrace(trace, pico, script != nullptr ? physim::describe(path) : "SCENARIO",
                   r.duration_s);
    }
    else
    {
        std::printf("time_s,target_mm,target_deg,distance_mm,yaw_deg,enc_left_mm,enc_right_mm,"
                    "imu_yaw_deg,wheel_left_mmps,wheel_right_mmps,ground_left_mmps,"
                    "ground_right_mmps,volts_left,volts_right,slip_left,slip_right,"
                    "pitch_rate_degps,accel_z_mps2\n");
        long tick = 0;
        r         = simulate(
            [&](const physim::Sample& s)
            {
                if (tick++ % every != 0)
                    return;
                std::printf(
                    "%.4f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%d,%d,"
                    "%.1f,%.2f\n",
                    s.time_s, s.target_mm, s.target_deg, s.distance_mm, s.yaw_deg,
                    s.encoder_left_mm, s.encoder_right_mm, s.imu_yaw_deg, s.wheel_left_mmps,
                    s.wheel_right_mmps, s.ground_left_mmps, s.ground_right_mmps, s.volts_left,
                    s.volts_right, s.slip_left, s.slip_right, s.pitch_rate_degps, s.accel_z_mps2);
            });
    }
    std::fprintf(stderr,
                 "finished=%d duration_s=%.3f distance_mm=%.3f yaw_deg=%.3f imu_yaw_deg=%.3f "
                 "encoder_mm=%.3f peak_pitch_rate_degps=%.1f peak_accel_z_mps2=%.2f "
                 "seam_crossings=%d x_mm=%.1f y_mm=%.1f\n",
                 r.finished, r.duration_s, r.distance_mm, r.yaw_deg, r.imu_yaw_deg, r.encoder_mm,
                 r.peak_pitch_rate_degps, r.peak_accel_z_mps2, r.seam_crossings, r.x_mm, r.y_mm);
    return r.finished ? 0 : 1;
}
