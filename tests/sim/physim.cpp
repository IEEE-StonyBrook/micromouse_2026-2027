// Physics sim plant and scenario runner. See physim.h.
#include "sim/physim.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "config/config.h"
#include "config/geometry.h"
#include "config/motion.h"
#include "config/sensors.h"
#include "config/smooth_turn.h"
#include "config/tuning.h"
#include "control/drivetrain.h"
#include "control/motion.h"
#include "control/run_log.h"
#include "fakes/fake_robot.h"

namespace physim
{
namespace
{
constexpr float PI           = 3.14159265f;
constexpr float GRAVITY_MPS2 = 9.81f;
constexpr float RAD_TO_DEG   = 180.0f / PI;
// The tyre is a stiff spring in slip speed, so the plant integrates in
// substeps well under its ~0.4 ms slip time constant.
constexpr float MAX_SUBSTEP_S = 20e-6f;
// Settle after the last move, and give up on a move after this long.
constexpr float SETTLE_S = 1.0f;
constexpr float GUARD_S  = 10.0f;
} // namespace

Params nominal()
{
    Params p;
    // Assumed, not measured: a ~110 g mouse, its footprint as a uniform slab.
    p.mass_kg          = 0.110f;
    p.yaw_inertia_kgm2 = 1.4e-4f;
    p.wheel_base_mm    = WHEEL_BASE_MM;
    p.wheel_radius_mm  = WHEEL_DIAMETER_MM / 2.0f;

    p.battery_v   = DEFAULT_BATTERY_VOLTAGE;
    p.winding_ohm = 4.0f; // small gearmotor class; only the R / ke^2 ratio matters below
    p.gear_ratio  = GEAR_RATIO;

    // Solve the motor so a straight reproduces the identified first-order model.
    // Steady state: tyre speed = V r / (G ke), so ke = r / (G Km).
    // Time constant: Tm = m_eff R r^2 / (G^2 ke^2), with m_eff the rotating
    // mass plus the wheel's half of the body.
    const float r_m       = p.wheel_radius_mm / 1000.0f;
    const float km_mps_v  = MOTOR_KM / 1000.0f;
    p.back_emf_vs_per_rad = r_m / (p.gear_ratio * km_mps_v);
    const float ke_g      = p.back_emf_vs_per_rad * p.gear_ratio;
    const float m_eff_kg  = MOTOR_TM * ke_g * ke_g / (p.winding_ohm * r_m * r_m);
    p.rotating_mass_kg    = m_eff_kg - p.mass_kg / 2.0f;

    // The identified model is V = v / Km + kS: the feedforward's kS cancels
    // this friction, so the plant must carry it or kS over-drives every move.
    p.friction_v_left  = FORWARD_KSL;
    p.friction_v_right = FORWARD_KSR;

    p.mu_left                  = 1.0f;
    p.mu_right                 = 1.0f;
    p.tyre_stiffness_n_per_mps = 100.0f;

    p.counts_per_mm_left  = 1.0f / MM_PER_TICK_LEFT;
    p.counts_per_mm_right = 1.0f / MM_PER_TICK_RIGHT;
    return p;
}

Robot::Robot(const Params& params)
    : left_motor(0, 1), right_motor(2, 3), left_encoder(hal::PioBlock::kPio0, 4),
      right_encoder(hal::PioBlock::kPio0, 6), imu(8), p_(params),
      rng_(params.seed * 0x9E3779B97F4A7C15ull + 1)
{
}

float Robot::yawDeg() const
{
    return yaw_rad_ * RAD_TO_DEG;
}

float Robot::wheelMmps(bool left) const
{
    return (left ? left_ : right_).tyre_mps * 1000.0f;
}

float Robot::groundMmps(bool left) const
{
    const float half_base_m = p_.wheel_base_mm / 2000.0f;
    return (v_mps_ + (left ? -1.0f : 1.0f) * omega_radps_ * half_base_m) * 1000.0f;
}

bool Robot::slipping(bool left) const
{
    return (left ? left_ : right_).at_limit;
}

float Robot::volts(bool left) const
{
    return (left ? left_ : right_).volts;
}

float Robot::amps(bool left) const
{
    const Wheel& w    = left ? left_ : right_;
    const float  r_m  = p_.wheel_radius_mm / 1000.0f;
    const float  ke_g = p_.back_emf_vs_per_rad * p_.gear_ratio;
    return (w.volts - ke_g * w.tyre_mps / r_m) / p_.winding_ohm;
}

float Robot::encoderMm(bool left) const
{
    const Wheel& w   = left ? left_ : right_;
    const float  cpm = left ? p_.counts_per_mm_left : p_.counts_per_mm_right;
    return std::round((w.tyre_m + w.extra_m) * 1000.0f * cpm) / cpm;
}

float Robot::imuYawDeg() const
{
    return imu_yaw_deg_;
}

// splitmix64: the same sequence on every platform, unlike <random>'s distributions.
float Robot::uniform()
{
    uint64_t z = (rng_ += 0x9E3779B97F4A7C15ull);
    z          = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z          = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return static_cast<float>((z >> 40) + 1) / static_cast<float>(1ull << 24); // (0, 1]
}

float Robot::gaussian()
{
    // Box-Muller; one value per call keeps the stream simple.
    return std::sqrt(-2.0f * std::log(uniform())) * std::cos(2.0f * PI * uniform());
}

float Robot::traction(Wheel& w, float mu, float ground_mps)
{
    float scale = 1.0f;
    if (w.slip_left_s > 0.0f)
        scale *= p_.slip.traction_scale;
    if (w.seam_left_s > 0.0f)
        scale *= p_.seam.traction_scale;
    const float limit_n = mu * scale * p_.mass_kg * GRAVITY_MPS2 / 2.0f;
    const float force_n = p_.tyre_stiffness_n_per_mps * (w.tyre_mps - ground_mps);
    w.at_limit          = std::fabs(force_n) > limit_n;
    return std::clamp(force_n, -limit_n, limit_n);
}

void Robot::substep(float h)
{
    const float r_m         = p_.wheel_radius_mm / 1000.0f;
    const float half_base_m = p_.wheel_base_mm / 2000.0f;
    const float ke_g        = p_.back_emf_vs_per_rad * p_.gear_ratio;
    const float ground_l    = v_mps_ - omega_radps_ * half_base_m;
    const float ground_r    = v_mps_ + omega_radps_ * half_base_m;
    const float t_l         = traction(left_, p_.mu_left, ground_l);
    const float t_r         = traction(right_, p_.mu_right, ground_r);

    for (Wheel* w : {&left_, &right_})
    {
        // Motor force at the tyre: (V - back EMF) / R through the gearbox.
        const float amps    = (w->volts - ke_g * w->tyre_mps / r_m) / p_.winding_ohm;
        const float drive_n = ke_g * amps / r_m;
        const float t_n     = (w == &left_) ? t_l : t_r;
        const float fric_v  = (w == &left_) ? p_.friction_v_left : p_.friction_v_right;
        // Friction force at the tyre: the stall current of fric_v through the gearbox.
        const float fric_n = ke_g * fric_v / (p_.winding_ohm * r_m);
        const float net_n  = drive_n - t_n;
        if (w->tyre_mps == 0.0f && std::fabs(net_n) <= fric_n)
        {
            // Stuck: friction holds the gearbox until the net force beats it.
        }
        else
        {
            const float dir   = (w->tyre_mps != 0.0f) ? std::copysign(1.0f, w->tyre_mps)
                                                      : std::copysign(1.0f, net_n);
            const float v_new = w->tyre_mps + (net_n - dir * fric_n) / p_.rotating_mass_kg * h;
            // Friction stops the wheel; it never drives it backwards.
            w->tyre_mps = (v_new * dir < 0.0f) ? 0.0f : v_new;
        }
        if (w == pinned_)
            w->tyre_mps = 0.0f;
        w->tyre_m += w->tyre_mps * h;
    }
    left_.ground_m += ground_l * h;
    right_.ground_m += ground_r * h;

    v_mps_ += (t_l + t_r) / p_.mass_kg * h;
    omega_radps_ += (t_r - t_l) * half_base_m / p_.yaw_inertia_kgm2 * h;
    yaw_rad_ += omega_radps_ * h;
    distance_m_ += v_mps_ * h;
    x_m_ += v_mps_ * std::cos(yaw_rad_) * h;
    y_m_ += v_mps_ * std::sin(yaw_rad_) * h;
}

void Robot::updateSensors(float dt)
{
    // Slip events and seam crossings, per wheel, at tick resolution.
    for (Wheel* w : {&left_, &right_})
    {
        if (w->slip_left_s > 0.0f)
            w->slip_left_s -= dt;
        else if (p_.slip.rate_hz > 0.0f && uniform() < p_.slip.rate_hz * dt)
            w->slip_left_s = p_.slip.duration_s;

        const Seam& s = p_.seam;
        if (w->seam_left_s > 0.0f)
        {
            // Spread the over-read across the crossing.
            const float share = std::min(dt / s.duration_s, 1.0f) * s.encoder_extra_mm / 1000.0f;
            w->extra_m        = std::min(w->extra_m + share, s.encoder_extra_mm / 1000.0);
            w->seam_left_s -= dt;
            if (w->seam_left_s <= 0.0f)
                w->extra_m = s.encoder_extra_mm / 1000.0f;
        }
        const float seam_m = (s.at_mm + (w == &right_ ? s.skew_mm : 0.0f)) / 1000.0f;
        if (s.enabled && !w->seam_crossed && w->ground_m >= seam_m)
        {
            w->seam_crossed = true;
            w->seam_left_s  = s.duration_s;
            seam_crossings_++;
            if (seam_t_s_ < 0.0f)
                seam_t_s_ = 0.0f;
        }
    }

    // Body pitch over the seam: nose up then down (one sine of pitch rate),
    // and one upward bump of z-accel.
    pitch_rate_degps_ = 0.0f;
    accel_z_mps2_     = 0.0f;
    if (seam_t_s_ >= 0.0f && seam_t_s_ < p_.seam.duration_s)
    {
        const float phase = seam_t_s_ / p_.seam.duration_s;
        pitch_rate_degps_ = p_.seam.pitch_rate_peak_degps * std::sin(2.0f * PI * phase);
        accel_z_mps2_     = p_.seam.accel_z_peak_mps2 * std::sin(PI * phase);
    }
    if (seam_t_s_ >= 0.0f)
        seam_t_s_ += dt;

    // Encoders report in the firmware's tick units: the plant quantises at its
    // own resolution, then Drivetrain multiplies by MM_PER_TICK.
    fake::setEncoderCount(left_encoder,
                          static_cast<int>(std::lround(encoderMm(true) / MM_PER_TICK_LEFT)));
    fake::setEncoderCount(right_encoder,
                          static_cast<int>(std::lround(encoderMm(false) / MM_PER_TICK_RIGHT)));

    // Gyro: true rate plus bias and white noise, integrated into yaw. A noise
    // density sampled every dt has RMS density * sqrt(1/dt), so the heading
    // random walk comes out the same at any loop rate.
    const float noise_rms_degps = p_.gyro_noise_degps_rthz * std::sqrt(1.0f / dt);
    const float omega_degps     = omega_radps_ * RAD_TO_DEG + p_.gyro_bias_degps +
                                  (noise_rms_degps > 0.0f ? noise_rms_degps * gaussian() : 0.0f);
    imu_yaw_deg_ += omega_degps * dt;
    fake::setImu(imu, imu_yaw_deg_, omega_degps);
}

void Robot::step()
{
    const float dt = LOOP_INTERVAL_S;
    if (p_.pinned_wheel.enabled && time_s_ >= p_.pinned_wheel.at_s)
        pinned_ = p_.pinned_wheel.left ? &left_ : &right_;
    left_.volts   = fake::motorDuty(left_motor) * p_.battery_v;
    right_.volts  = fake::motorDuty(right_motor) * p_.battery_v;
    const int   n = static_cast<int>(std::ceil(dt / MAX_SUBSTEP_S));
    const float h = dt / static_cast<float>(n);
    for (int i = 0; i < n; i++)
        substep(h);
    updateSensors(dt);
    time_s_ += dt;
}

bool parseScenario(const char* name, Scenario* out)
{
    if (std::strcmp(name, "straight10") == 0)
        *out = Scenario::kStraight10;
    else if (std::strcmp(name, "ip90") == 0)
        *out = Scenario::kInPlace90;
    else if (std::strcmp(name, "ss90") == 0)
        *out = Scenario::kSmooth90;
    else
        return false;
    return true;
}

namespace
{
// Motion holds a ~100 KB trace buffer, so the rig lives on the heap.
// Motion's current reader: the plant's winding currents, as IPROPI would give.
bool readPlantAmps(void* robot, float* left_amps, float* right_amps)
{
    const Robot* r = static_cast<const Robot*>(robot);
    *left_amps     = r->amps(true);
    *right_amps    = r->amps(false);
    return true;
}

// Control ticks per run log sample, as RunLog::start() rounds them.
int ticksPerSample(int dt_ms)
{
    const int ticks = static_cast<int>(std::lround(dt_ms * LOOP_FREQUENCY_HZ / 1000.0f));
    return ticks > 0 ? ticks : 1;
}

// The firmware's TRACE header (app/mouse.cpp print_turn_trace) with the
// extended keys, and RunLog::COLUMNS with the sim-only truth columns before ev.
std::string traceHeader(unsigned count, int dt_ms, float batt_v, bool wrapped,
                        const std::string& cmd)
{
    std::string cols = RunLog::COLUMNS;
    cols.insert(cols.rfind(','), ",true_dist_mm,true_yaw_deg");
    char head[96];
    std::snprintf(head, sizeof head, "TRACE n=%u dt_ms=%d cols=", count, dt_ms);
    char tail[96];
    std::snprintf(tail, sizeof tail, " run=1 loop_hz=%.0f batt_v=%.2f wrapped=%d cmd=\"",
                  static_cast<double>(LOOP_FREQUENCY_HZ), static_cast<double>(batt_v),
                  wrapped ? 1 : 0);
    return head + cols + tail + cmd + "\"";
}

// RunLog::formatRow() with the truth columns spliced in before ev, the last,
// and the PATH segment index in ev bits 8-13 (0 = start advance, then 1..n),
// as the Pico's RunLog::set_segment() numbers them.
std::string traceRow(uint16_t index, uint16_t dt_ms, RunLog::Sample sample, float true_dist_mm,
                     float true_yaw_deg, uint8_t segment = 0)
{
    sample.events = static_cast<uint16_t>(sample.events | (segment % 64u) << 8);
    char row[RunLog::ROW_BYTES];
    RunLog::formatRow(row, sizeof row, index, dt_ms, sample);
    char truth[48];
    std::snprintf(truth, sizeof truth, ",%.1f,%.2f", static_cast<double>(true_dist_mm),
                  static_cast<double>(true_yaw_deg));
    std::string out = row;
    out.insert(out.rfind(','), truth);
    return out;
}

float normalizeYawDelta(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg <= -180.0f)
        deg += 360.0f;
    return deg;
}

struct Truth
{
    float   distance_mm;
    float   yaw_deg;
    uint8_t segment;
};

struct Rig
{
    Robot                                     robot;
    Drivetrain                                dt;
    Motion                                    motion;
    RunLog                                    log;
    std::vector<Truth>                        truth; // one per sample the log recorded
    uint8_t                                   segment = 0; // the PATH segment driving
    float                                     battery_v;
    Result                                    result{};
    const std::function<void(const Sample&)>& on_sample;

    Rig(const Params& p, const std::function<void(const Sample&)>& cb)
        : robot(p),
          dt(&robot.left_motor, &robot.right_motor, &robot.left_encoder, &robot.right_encoder),
          motion(&dt, &robot.imu), battery_v(p.battery_v), on_sample(cb)
    {
        motion.set_run_log(&log);
        result.finished              = true;
        result.crash_cause           = CrashCause::kNone;
        result.crash_trip_s          = -1.0f;
        result.peak_volts_after_trip = 0.0f;
    }

    void tick()
    {
        const bool     was_tripped = motion.crashDetector().tripped();
        const uint16_t before      = log.count();
        // Motion skips the sample on an idle tick, so watch the log for a new
        // one: the count grows or, once the ring is full, its newest or oldest
        // sample changes. RunLog exposes no record counter, so a full ring of
        // a robot standing still (identical samples at both ends) can miss one.
        const bool     full = before == RunLog::CAPACITY;
        RunLog::Packed ends_before[2]{};
        if (full)
        {
            ends_before[0] = RunLog::pack(log.sample(0));
            ends_before[1] = RunLog::pack(log.sample(before - 1));
        }
        // Truth at the instant Motion samples: before this tick's plant step.
        const Truth now{robot.distanceMm(), robot.yawDeg(), segment};
        motion.update();
        if (log.count() < before)
            truth.clear(); // restarted
        bool recorded = log.count() > before;
        if (full && log.count() == before)
        {
            const RunLog::Packed ends_after[2] = {RunLog::pack(log.sample(0)),
                                                  RunLog::pack(log.sample(before - 1))};
            recorded = std::memcmp(ends_before, ends_after, sizeof ends_after) != 0;
        }
        if (recorded)
            truth.push_back(now);
        robot.step();
        if (was_tripped)
            result.peak_volts_after_trip =
                std::max({result.peak_volts_after_trip, std::fabs(robot.volts(true)),
                          std::fabs(robot.volts(false))});
        else if (motion.crashDetector().tripped())
        {
            result.crash_trip_s = robot.timeS();
            result.crash_cause  = motion.crashDetector().cause();
        }
        result.peak_pitch_rate_degps =
            std::max(result.peak_pitch_rate_degps, std::fabs(robot.pitchRateDegps()));
        result.peak_accel_z_mps2 =
            std::max(result.peak_accel_z_mps2, std::fabs(robot.accelZMps2()));
        if (on_sample)
            on_sample({robot.timeS(), motion.position(), motion.angle(), robot.distanceMm(),
                       robot.yawDeg(), robot.encoderMm(true), robot.encoderMm(false),
                       robot.imuYawDeg(), robot.wheelMmps(true), robot.wheelMmps(false),
                       robot.groundMmps(true), robot.groundMmps(false), robot.volts(true),
                       robot.volts(false), robot.slipping(true), robot.slipping(false),
                       robot.pitchRateDegps(), robot.accelZMps2()});
    }

    template <typename Done> void waitFor(Done done)
    {
        const int guard = LOOP_TICKS(GUARD_S);
        int       ticks = 0;
        // A trip freezes the move for good; stop waiting on it.
        while (!done() && !motion.crashDetector().tripped() && ticks++ < guard)
            tick();
        if (!done())
            result.finished = false;
    }

    void waitMove()
    {
        waitFor([this] { return motion.move_finished(); });
    }
    void waitTurn()
    {
        waitFor([this] { return motion.turn_finished(); });
    }
    // The firmware's wait_path_segment_motion() / Mouse::waitForMotion().
    void waitMotion()
    {
        waitFor([this] { return motion.move_finished() && motion.turn_finished(); });
    }
    // Motion::stop_forward() and spin_turn() poll for a settled forward
    // profile on the wall clock; step the plant to it first, so they return at once.
    void waitSettled()
    {
        waitFor([this] { return motion.move_settled(); });
    }
    void settle()
    {
        for (int i = 0; i < LOOP_TICKS(SETTLE_S); i++)
            tick();
    }

    // Stops the log and dumps it as the firmware does.
    void dumpTrace(const std::string& cmd, std::vector<std::string>* out)
    {
        log.stop();
        const uint16_t count = log.count();
        const uint16_t dt_ms = log.dtMs();
        const size_t   first = truth.size() > count ? truth.size() - count : 0; // overwritten
        out->push_back(traceHeader(count, dt_ms, battery_v, first > 0, cmd));
        for (uint16_t i = 0; i < count; ++i)
        {
            const Truth t = first + i < truth.size() ? truth[first + i] : Truth{NAN, NAN, 0};
            out->push_back(
                traceRow(i, dt_ms, log.sample(i), t.distance_mm, t.yaw_deg, t.segment));
        }
        out->push_back("TRACE end");
    }
};

void configure(Rig& r, const RunOptions& options)
{
    r.motion.crashDetector().set_enabled(options.crash_watchdog);
    r.motion.crashDetector().set_inhibit(options.crash_inhibit);
    if (options.sense_current)
        r.motion.set_motor_current_reader(readPlantAmps, &r.robot);
    r.motion.set_crash_cut_hook(options.on_crash_cut);
}

void fillResult(Rig& r)
{
    Result& res        = r.result;
    res.duration_s     = r.robot.timeS();
    res.distance_mm    = r.robot.distanceMm();
    res.yaw_deg        = r.robot.yawDeg();
    res.x_mm           = r.robot.xMm();
    res.y_mm           = r.robot.yMm();
    res.imu_yaw_deg    = r.robot.imuYawDeg();
    res.encoder_mm     = 0.5f * (r.robot.encoderMm(true) + r.robot.encoderMm(false));
    res.seam_crossings = r.robot.seamCrossings();
}

const char* scenarioName(Scenario scenario)
{
    switch (scenario)
    {
        case Scenario::kStraight10:
            return "straight10";
        case Scenario::kInPlace90:
            return "ip90";
        case Scenario::kSmooth90:
            return "ss90";
    }
    return "?";
}
} // namespace

Result run(Scenario scenario, const Params& params, const RunOptions& options,
           const std::function<void(const Sample&)>& on_sample)
{
    auto r = std::make_unique<Rig>(params, on_sample);
    configure(*r, options);
    if (options.trace != nullptr)
        r->motion.start_trace();
    switch (scenario)
    {
        case Scenario::kStraight10:
            // Maze runs hold the logical heading on straights with the IMU.
            r->motion.set_heading_hold(0.0f);
            r->motion.start_move(10.0f * CELL_SIZE_MM, options.speed_mmps, 0.0f,
                                 options.accel_mmps2);
            r->waitMove();
            break;
        case Scenario::kInPlace90:
            r->motion.start_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                                 ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
            r->waitTurn();
            break;
        case Scenario::kSmooth90:
        {
            // The firmware's 95.5 deg arc makes up for tyre scrub the plant does
            // not model; a scrub-free tyre needs exactly 90.
            Motion::SmoothTurnTuning tuning = r->motion.smoothTurnTuning();
            tuning.left.angle_deg           = 90.0f;
            r->motion.set_smooth_turn_tuning(tuning);
            // From the centre of a cell, turn left in the next one and stop at
            // the centre of the cell after that.
            r->motion.start_smooth_turn_entry(true, CELL_SIZE_MM);
            r->waitMove();
            r->motion.start_smooth_turn_arc(true);
            r->waitTurn();
            r->motion.start_move(r->motion.smoothTurnExitMm(true, CELL_SIZE_MM), tuning.speed_mmps,
                                 0.0f, ROBOT_SEARCH_ACCEL_MMPS2);
            r->waitMove();
            break;
        }
    }
    r->settle();
    if (options.trace != nullptr)
    {
        char cmd[96];
        std::snprintf(cmd, sizeof cmd, "SCENARIO %s speed=%g accel=%g", scenarioName(scenario),
                      static_cast<double>(options.speed_mmps),
                      static_cast<double>(options.accel_mmps2));
        r->dumpTrace(cmd, options.trace);
    }
    fillResult(*r);
    return r->result;
}

// ---------------------------------------------------------------- PATH --

namespace
{
enum class SegmentType
{
    kForward,
    kTurn,
    kSmoothTurn,
};

struct Segment
{
    SegmentType type;
    float       value;                   // forward: mm; turns: deg, + = left
    float       drive_mm         = 0.0f; // forward, after smooth-turn trims
    float       final_speed_mmps = 0.0f;
};

bool equalsIgnoreCase(const std::string& a, const char* b)
{
    if (a.size() != std::strlen(b))
        return false;
    for (size_t i = 0; i < a.size(); i++)
        if (std::toupper(static_cast<unsigned char>(a[i])) != b[i])
            return false;
    return true;
}

// app/cli.cpp parse_path_chunk(): F[n] cells, L/R/B turns, '#' ignored.
bool parseSequence(const std::string& text, std::vector<Segment>* out, std::string* error)
{
    out->clear();
    for (size_t i = 0; i < text.size();)
    {
        const char token = static_cast<char>(std::toupper(static_cast<unsigned char>(text[i++])));
        if (token == '#')
            continue;
        if (token == 'F')
        {
            int  cells      = 0;
            bool has_digits = false;
            while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i])))
            {
                has_digits = true;
                cells      = cells * 10 + (text[i++] - '0');
            }
            if (has_digits && cells <= 0)
            {
                *error = "PATH forward count must be positive";
                return false;
            }
            const float mm = (cells == 0 ? 1 : cells) * CELL_SIZE_MM;
            if (!out->empty() && out->back().type == SegmentType::kForward)
                out->back().value += mm;
            else
                out->push_back({SegmentType::kForward, mm});
            continue;
        }
        if (token == 'L' || token == 'R' || token == 'B')
        {
            if (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i])))
            {
                *error = "PATH only supports L/R/B as turns";
                return false;
            }
            out->push_back({SegmentType::kTurn, token == 'L'   ? 90.0f
                                                : token == 'R' ? -90.0f
                                                               : 180.0f});
            continue;
        }
        *error = std::string("invalid PATH token '") + token + "'";
        return false;
    }
    if (out->empty())
    {
        *error = "PATH expects a sequence, e.g. FFRFFLF";
        return false;
    }
    return true;
}

// app/cli.cpp run_path_segments() / resolve_path_segments(): SMOOTH turns a
// 90 with a straight after it (and no spin before) into an arc; a straight
// next to an arc gives it HALF_CELL_MM and hands over at turn speed.
bool resolveSegments(const PathCommand& c, float turn_speed_mmps, std::vector<Segment>* segments,
                     std::string* error)
{
    if (!parseSequence(c.sequence, segments, error))
        return false;
    std::vector<Segment>& s = *segments;
    if (c.smooth)
        for (size_t i = 1; i + 1 < s.size(); ++i)
            if (s[i].type == SegmentType::kTurn &&
                std::fabs(std::fabs(s[i].value) - 90.0f) < 0.125f &&
                s[i - 1].type != SegmentType::kTurn && s[i + 1].type == SegmentType::kForward)
                s[i].type = SegmentType::kSmoothTurn;
    for (size_t i = 0; i < s.size(); ++i)
    {
        if (s[i].type != SegmentType::kForward)
            continue;
        const bool turn_next = i + 1 < s.size() && s[i + 1].type == SegmentType::kSmoothTurn;
        const bool turn_prev = i > 0 && s[i - 1].type == SegmentType::kSmoothTurn;
        s[i].drive_mm =
            s[i].value - (turn_next ? HALF_CELL_MM : 0.0f) - (turn_prev ? HALF_CELL_MM : 0.0f);
        s[i].final_speed_mmps = turn_next ? turn_speed_mmps : 0.0f;
        if (s[i].drive_mm < 0.0f)
        {
            *error = "PATH rejected: a straight is shorter than a neighbouring smooth turn needs";
            return false;
        }
    }
    return true;
}

bool parseBool(const std::string& v)
{
    return v == "1" || equalsIgnoreCase(v, "TRUE");
}

// app/cli.cpp parsePathOption(): trailing flags, checked before numbers.
bool parsePathOption(const std::string& w, PathCommand* c)
{
    if (equalsIgnoreCase(w, "RAW"))
        c->raw = true;
    else if (equalsIgnoreCase(w, "POSE") || equalsIgnoreCase(w, "HOLD") ||
             equalsIgnoreCase(w, "HEADING"))
        c->raw = false;
    else if (w == "1" || equalsIgnoreCase(w, "TRUE") || equalsIgnoreCase(w, "SMOOTH"))
        c->smooth = true;
    else if (w == "0" || equalsIgnoreCase(w, "FALSE") || equalsIgnoreCase(w, "STILL") ||
             equalsIgnoreCase(w, "STATIONARY"))
        c->smooth = false;
    else
        return false;
    return true;
}

// app/cli.cpp startsNumericArg().
bool startsNumeric(const std::string& w)
{
    size_t i = (w[0] == '+' || w[0] == '-') ? 1 : 0;
    return i < w.size() && (std::isdigit(static_cast<unsigned char>(w[i])) || w[i] == '.');
}

// app/cli.cpp parseFloatArg(): the whole word is the number.
bool parseFloat(const std::string& w, float* out)
{
    char*       end = nullptr;
    const float v   = std::strtof(w.c_str(), &end);
    if (end == w.c_str() || *end != '\0')
        return false;
    *out = v;
    return true;
}
} // namespace

bool parsePathCommand(const std::string& args, PathCommand* out, std::string* error)
{
    PathCommand c;
    c.speed_mmps   = CLI_PATH_SPEED_MMPS;
    c.accel_mmps2  = CLI_PATH_ACCEL_MMPS2;
    c.omega_degps  = CLI_PATH_OMEGA_DEGPS;
    c.alpha_degps2 = CLI_PATH_ALPHA_DEGPS2;

    // Bounds as app/cli.cpp handle_path_command() checks them.
    struct Param
    {
        const char* key;
        float*      value;
        float       max;
    };
    const Param params[] = {{"speed", &c.speed_mmps, 1e9f},
                            {"accel", &c.accel_mmps2, 5000.0f},
                            {"omega", &c.omega_degps, 720.0f},
                            {"alpha", &c.alpha_degps2, 7200.0f}};

    // Split into words; key=value words are the TRACE header's cmd= form.
    std::vector<std::string> words;
    for (size_t i = 0; i < args.size();)
    {
        while (i < args.size() && std::isspace(static_cast<unsigned char>(args[i])))
            i++;
        const size_t start = i;
        while (i < args.size() && !std::isspace(static_cast<unsigned char>(args[i])))
            i++;
        if (i == start)
            continue;
        const std::string w  = args.substr(start, i - start);
        const size_t      eq = w.find('=');
        if (eq == std::string::npos)
        {
            words.push_back(w);
            continue;
        }
        const std::string key   = w.substr(0, eq);
        const std::string val   = w.substr(eq + 1);
        bool              known = key == "smooth" || key == "raw";
        for (const Param& p : params)
            if (key == p.key)
            {
                known = true;
                if (!parseFloat(val, p.value))
                {
                    *error = "PATH " + key + " must be a number";
                    return false;
                }
            }
        if (key == "smooth")
            c.smooth = parseBool(val);
        if (key == "raw")
            c.raw = parseBool(val);
        if (!known)
        {
            *error = "unknown PATH key " + key;
            return false;
        }
    }
    if (!words.empty() && equalsIgnoreCase(words[0], "PATH"))
        words.erase(words.begin());

    // As handle_path_command(): flags off the end (the sequence stays), then
    // the sequence up to the first numeric word, then up to four numbers.
    size_t limit = words.size();
    while (limit > 1 && parsePathOption(words[limit - 1], &c))
        --limit;
    size_t first_number = limit;
    for (size_t i = 0; i < limit; i++)
        if (startsNumeric(words[i]))
        {
            first_number = i;
            break;
        }
    for (size_t i = 0; i < first_number; i++)
        c.sequence += words[i];
    if (limit - first_number > 4)
    {
        *error = "PATH usage: <sequence> [speed accel omega alpha] [SMOOTH] [RAW]";
        return false;
    }
    for (size_t i = first_number; i < limit; i++)
        if (!parseFloat(words[i], params[i - first_number].value))
        {
            *error = std::string("PATH ") + params[i - first_number].key + " must be a number";
            return false;
        }
    for (const Param& p : params)
        if (!(*p.value >= 1.0f && *p.value <= p.max))
        {
            *error = std::string("PATH ") + p.key + " out of range";
            return false;
        }
    std::vector<Segment> segments;
    if (!resolveSegments(c, ROBOT_SEARCH_TURN_SPEED_MMPS, &segments, error))
        return false;
    *out = c;
    return true;
}

std::string describe(const PathCommand& c)
{
    char text[160];
    std::snprintf(text, sizeof text, "PATH %s speed=%g accel=%g omega=%g alpha=%g smooth=%d raw=%d",
                  c.sequence.c_str(), static_cast<double>(c.speed_mmps),
                  static_cast<double>(c.accel_mmps2), static_cast<double>(c.omega_degps),
                  static_cast<double>(c.alpha_degps2), c.smooth ? 1 : 0, c.raw ? 1 : 0);
    return text;
}

Result runPath(const PathCommand& c, const Params& params, const RunOptions& options,
               const std::function<void(const Sample&)>& on_sample)
{
    auto r = std::make_unique<Rig>(params, on_sample);
    configure(*r, options);
    Motion& m = r->motion;
    // An open floor: every ToF out of range, so wall steering stays zero.
    m.set_wall_distances(TOF_OUT_OF_RANGE_MM, TOF_OUT_OF_RANGE_MM, TOF_OUT_OF_RANGE_MM);

    std::vector<Segment> segments;
    std::string          error;
    if (!resolveSegments(c, m.smoothTurnTuning().speed_mmps, &segments, &error))
    {
        r->result.finished = false;
        fillResult(*r);
        return r->result;
    }

    // Armed before the start advance (segment 0), as handle_path_command() does.
    m.start_trace();
    // run_path_start_center(): from against the back wall to the first cell's centre.
    if (!c.raw)
    {
        m.begin_motion_sequence();
        m.set_heading_hold(m.yaw_deg());
        m.move(START_CENTER_DISTANCE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
        r->waitMotion();
        m.end_motion_sequence();
        if (!r->result.finished)
        {
            m.stop_trace();
            if (options.trace != nullptr)
                r->dumpTrace(describe(c), options.trace);
            fillResult(*r); // PATH stops here too
            return r->result;
        }
    }
    const float x0_mm    = r->robot.xMm();
    const float y0_mm    = r->robot.yMm();
    const float yaw0_rad = r->robot.yawDeg() * PI / 180.0f;

    m.begin_motion_sequence();
    float expected_yaw_deg = m.yaw_deg();
    for (const Segment& s : segments)
    {
        r->segment = static_cast<uint8_t>((&s - segments.data() + 1) % 64);
        if (s.type == SegmentType::kForward)
        {
            if (!c.raw)
                m.set_heading_hold(normalizeYawDelta(expected_yaw_deg));
            if (!c.raw && c.smooth)
                m.set_steering_mode(tof_wall::SteeringMode::STEER_NORMAL);
            if (s.drive_mm > 0.0f)
            {
                m.start_move(s.drive_mm, c.speed_mmps, s.final_speed_mmps, c.accel_mmps2);
                r->waitMotion();
            }
        }
        else if (s.type == SegmentType::kSmoothTurn)
        {
            // Mouse::smooth_turn(turn_left, HALF_CELL_MM, HALF_CELL_MM); no
            // front wall, so the entry ends by odometry.
            const bool turn_left = s.value > 0.0f;
            m.set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
            const bool  aim_arc     = m.headingHoldActive();
            const float aim_yaw_deg = m.headingHoldTargetYawDeg();
            m.clear_heading_hold();
            const float lead_mm = m.start_smooth_turn_entry(turn_left, HALF_CELL_MM);
            r->waitFor([&] { return m.position() >= lead_mm; });
            const float aim_error_deg =
                aim_arc ? std::clamp(normalizeYawDelta(aim_yaw_deg - m.yaw_deg()),
                                     -SMOOTH_TURN_MAX_AIM_DEG, SMOOTH_TURN_MAX_AIM_DEG)
                        : 0.0f;
            m.start_smooth_turn_arc(turn_left, aim_error_deg);
            r->waitMotion();
            const float speed_mmps = m.smoothTurnTuning().speed_mmps;
            m.move(m.smoothTurnExitMm(turn_left, HALF_CELL_MM), speed_mmps, speed_mmps,
                   ROBOT_SEARCH_ACCEL_MMPS2);
            r->waitMotion();
            expected_yaw_deg += turn_left ? 90.0f : -90.0f;
        }
        else
        {
            if (!c.raw)
                m.clear_heading_hold();
            r->waitSettled();
            m.spin_turn(s.value, std::fabs(c.omega_degps), std::fabs(c.alpha_degps2));
            r->waitMotion();
            expected_yaw_deg += s.value;
        }
        if (!r->result.finished)
            break;
    }
    // Brake to rest, as PATH does after its last segment.
    r->waitSettled();
    m.stop_forward(c.accel_mmps2, STOP_AT_CENTRE_PROFILE_TIMEOUT_MS);
    m.stop();
    m.end_motion_sequence();
    m.stop_trace(); // run_path_segments() stops it on the way out
    r->settle();
    if (options.trace != nullptr)
        r->dumpTrace(describe(c), options.trace);

    fillResult(*r);
    // Relative to the first cell's centre and the heading there.
    const float dx = r->robot.xMm() - x0_mm;
    const float dy = r->robot.yMm() - y0_mm;
    r->result.x_mm = dx * std::cos(yaw0_rad) + dy * std::sin(yaw0_rad);
    r->result.y_mm = -dx * std::sin(yaw0_rad) + dy * std::cos(yaw0_rad);
    return r->result;
}

// ------------------------------------------------------- volts replay --

int TraceRun::column(const char* name) const
{
    for (size_t i = 0; i < columns.size(); i++)
        if (columns[i] == name)
            return static_cast<int>(i);
    return -1;
}

namespace
{
// Drops serial_term's "[  123.4] " stamp and the Pico's "[INFO] " tag.
std::string stripPrefixes(const std::string& line)
{
    size_t i = 0;
    for (;;)
    {
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])))
            i++;
        if (i >= line.size() || line[i] != '[')
            break;
        const size_t close = line.find(']', i);
        if (close == std::string::npos)
            break;
        i = close + 1;
    }
    size_t end = line.size();
    while (end > i && std::isspace(static_cast<unsigned char>(line[end - 1])))
        end--;
    return line.substr(i, end - i);
}

// The value of " key=" in a header line, up to the next space.
std::string headerValue(const std::string& line, const char* key)
{
    const std::string k  = std::string(" ") + key + "=";
    const size_t      at = line.find(k);
    if (at == std::string::npos)
        return "";
    const size_t start = at + k.size();
    return line.substr(start, line.find(' ', start) - start);
}
} // namespace

bool parseTrace(const std::string& text, int index, TraceRun* out, std::string* error)
{
    int      seen = -1;
    TraceRun run;
    bool     in  = false;
    size_t   pos = 0;
    while (pos <= text.size())
    {
        size_t nl = text.find('\n', pos);
        if (nl == std::string::npos)
            nl = text.size();
        const std::string line = stripPrefixes(text.substr(pos, nl - pos));
        pos                    = nl + 1;
        if (line.rfind("TRACE ", 0) == 0 && line.find(" cols=") != std::string::npos &&
            line.find("n=") != std::string::npos)
        {
            if (in)
                break; // the dump we wanted ended without "TRACE end"
            if (++seen != index)
                continue;
            in                     = true;
            run.dt_ms              = std::atoi(headerValue(line, "dt_ms").c_str());
            run.wrapped            = headerValue(line, "wrapped") == "1";
            const std::string batt = headerValue(line, "batt_v");
            run.batt_v             = batt.empty() ? NAN : std::strtof(batt.c_str(), nullptr);
            const size_t cmd_at    = line.find(" cmd=\"");
            if (cmd_at != std::string::npos)
                run.cmd = line.substr(cmd_at + 6, line.rfind('"') - (cmd_at + 6));
            const std::string cols = headerValue(line, "cols");
            size_t            s    = 0;
            for (size_t i = 0; i <= cols.size(); i++)
                if (i == cols.size() || cols[i] == ',')
                {
                    run.columns.push_back(cols.substr(s, i - s));
                    s = i + 1;
                }
        }
        else if (in && line.rfind("T,", 0) == 0)
        {
            std::vector<float> row;
            const char*        p = line.c_str() + 2;
            for (;;)
            {
                char* end;
                row.push_back(std::strtof(p, &end));
                if (*end != ',')
                    break;
                p = end + 1;
            }
            // A row cut short on the link keeps its slot, so later rows keep
            // their times; replay holds the volts across it.
            if (row.size() != run.columns.size())
                row.assign(run.columns.size(), NAN);
            run.rows.push_back(row);
        }
        else if (in && line == "TRACE end")
            break;
    }
    if (!in)
    {
        *error = "no TRACE dump " + std::to_string(index);
        return false;
    }
    if (run.dt_ms <= 0 || run.column("left_v") < 0 || run.column("right_v") < 0)
    {
        *error = "TRACE dump lacks dt_ms or the left_v/right_v columns";
        return false;
    }
    *out = run;
    return true;
}

ReplayResult replayVolts(const TraceRun& logged, const Params& params,
                         std::vector<std::string>* trace)
{
    Params p = params;
    if (!std::isnan(logged.batt_v) && logged.batt_v > 0.0f)
        p.battery_v = logged.batt_v;
    // No Motion: the logged volts go straight to the motors.
    auto       robot = std::make_unique<Robot>(p);
    Drivetrain dt(&robot->left_motor, &robot->right_motor, &robot->left_encoder,
                  &robot->right_encoder);

    float      left_v = 0.0f, right_v = 0.0f; // held across a lost row
    const int  lv = logged.column("left_v"), rv = logged.column("right_v");
    const int  lm = logged.column("left_mmps"), rm = logged.column("right_mmps");
    const int  per     = ticksPerSample(logged.dt_ms);
    const auto dt_ms   = static_cast<uint16_t>(logged.dt_ms);
    double     sq_left = 0.0, sq_right = 0.0;
    int        compared = 0;
    if (trace != nullptr)
        trace->push_back(traceHeader(static_cast<unsigned>(logged.rows.size()), logged.dt_ms,
                                     p.battery_v, false, "REPLAY-VOLTS " + logged.cmd));
    for (size_t i = 0; i < logged.rows.size(); i++)
    {
        const std::vector<float>& row = logged.rows[i];
        for (int k = 0; k < per; k++)
        {
            // As in Motion::update(): measure, then command; the sample sees both.
            dt.update();
            if (k == 0)
            {
                if (!std::isnan(row[lv]) && !std::isnan(row[rv]))
                {
                    left_v  = row[lv];
                    right_v = row[rv];
                }
                const float left_mmps  = dt.velocity(WheelSide::LEFT);
                const float right_mmps = dt.velocity(WheelSide::RIGHT);
                if (lm >= 0 && rm >= 0 && !std::isnan(row[lm]) && !std::isnan(row[rm]))
                {
                    sq_left += (left_mmps - row[lm]) * (left_mmps - row[lm]);
                    sq_right += (right_mmps - row[rm]) * (right_mmps - row[rm]);
                    compared++;
                }
                if (trace != nullptr)
                {
                    RunLog::Sample s;
                    s.yaw_deg     = robot->imuYawDeg();
                    s.left_mmps   = left_mmps;
                    s.right_mmps  = right_mmps;
                    s.left_volts  = left_v;
                    s.right_volts = right_v;
                    trace->push_back(traceRow(static_cast<uint16_t>(i), dt_ms, s,
                                              robot->distanceMm(), robot->yawDeg()));
                }
                // The logged volts are what the wheels got: the firmware scales
                // duty by its battery reading. The pack caps them.
                robot->left_motor.set_motor_volts(left_v, p.battery_v);
                robot->right_motor.set_motor_volts(right_v, p.battery_v);
            }
            robot->step();
        }
    }
    if (trace != nullptr)
        trace->push_back("TRACE end");
    ReplayResult res;
    res.samples        = static_cast<int>(logged.rows.size());
    res.rms_left_mmps  = compared ? static_cast<float>(std::sqrt(sq_left / compared)) : NAN;
    res.rms_right_mmps = compared ? static_cast<float>(std::sqrt(sq_right / compared)) : NAN;
    return res;
}
} // namespace physim
