// Physics sim plant and scenario runner. See physim.h.
#include "sim/physim.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

#include "config/config.h"
#include "config/geometry.h"
#include "config/motion.h"
#include "config/tuning.h"
#include "control/drivetrain.h"
#include "control/motion.h"
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

struct Rig
{
    Robot                                     robot;
    Drivetrain                                dt;
    Motion                                    motion;
    Result                                    result{};
    const std::function<void(const Sample&)>& on_sample;

    Rig(const Params& p, const std::function<void(const Sample&)>& cb)
        : robot(p),
          dt(&robot.left_motor, &robot.right_motor, &robot.left_encoder, &robot.right_encoder),
          motion(&dt, &robot.imu), on_sample(cb)
    {
        result.finished              = true;
        result.crash_cause           = CrashCause::kNone;
        result.crash_trip_s          = -1.0f;
        result.peak_volts_after_trip = 0.0f;
    }

    void tick()
    {
        const bool was_tripped = motion.crashDetector().tripped();
        motion.update();
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
    void settle()
    {
        for (int i = 0; i < LOOP_TICKS(SETTLE_S); i++)
            tick();
    }
};
} // namespace

Result run(Scenario scenario, const Params& params, const RunOptions& options,
           const std::function<void(const Sample&)>& on_sample)
{
    auto r = std::make_unique<Rig>(params, on_sample);
    r->motion.crashDetector().set_enabled(options.crash_watchdog);
    r->motion.crashDetector().set_inhibit(options.crash_inhibit);
    if (options.sense_current)
        r->motion.set_motor_current_reader(readPlantAmps, &r->robot);
    r->motion.set_crash_cut_hook(options.on_crash_cut);
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

    Result& res        = r->result;
    res.duration_s     = r->robot.timeS();
    res.distance_mm    = r->robot.distanceMm();
    res.yaw_deg        = r->robot.yawDeg();
    res.imu_yaw_deg    = r->robot.imuYawDeg();
    res.encoder_mm     = 0.5f * (r->robot.encoderMm(true) + r->robot.encoderMm(false));
    res.seam_crossings = r->robot.seamCrossings();
    return res;
}
} // namespace physim
