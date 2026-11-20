#ifndef TESTS_SIM_PHYSIM_H
#define TESTS_SIM_PHYSIM_H

/**
 * @brief Physics sim: the real Motion, Profile, PID and Drivetrain at loop
 * rate against a DC-motor, two-wheel, slipping-tyre plant.
 *
 * The plant drives the same fake drivers the host tests use (fakes/), so the
 * firmware runs unchanged. Every knob defaults to the nominal model, which is
 * built to reproduce the identified MOTOR_KM / MOTOR_TM on a straight; the
 * knobs then add what that model leaves out: a traction limit and random slip
 * events per wheel, encoder resolution, gyro noise and bias, battery voltage
 * and a floor seam. Randomness comes from one seeded generator, so a run is
 * reproducible on any platform.
 *
 * Units: mm, s, deg at the interface; SI (m, kg, N, V, ohm) inside the plant.
 */

#include <cstdint>
#include <functional>

#include "control/crash_detector.h"
#include "drivers/encoder.h"
#include "drivers/imu.h"
#include "drivers/motor.h"

namespace physim
{
/// Random loss of grip: each wheel starts an event as a Poisson process.
struct SlipEvents
{
    float rate_hz        = 0.0f;  // mean events per second, per wheel; 0 = none
    float duration_s     = 0.05f; // how long grip stays reduced
    float traction_scale = 0.2f;  // fraction of the traction limit left during an event
};

/**
 * @brief A floor seam (a raised joint between maze tiles) the wheels cross.
 *
 * As each wheel's contact point reaches at_mm (the right wheel skew_mm later)
 * it loses grip for duration_s and its encoder over-reads encoder_extra_mm:
 * the tyre rolls up and over the step, turning further than the body moves.
 * The body also pitches over it: a pitch-rate swing (up, then down) and a
 * z-accel bump the IMU would see. The firmware IMU reads yaw only, so pitch
 * and z-accel are logged for a later seam detector, not fed back.
 */
struct Seam
{
    bool  enabled               = false;
    float at_mm                 = 0.0f;  // body-centre distance of the seam from the start
    float skew_mm               = 0.0f;  // right wheel crosses this much later (seam not square)
    float duration_s            = 0.02f; // crossing time per wheel
    float traction_scale        = 0.3f;  // grip left while a wheel is on the step
    float encoder_extra_mm      = 0.0f;  // extra encoder travel per wheel crossing
    float pitch_rate_peak_degps = 0.0f;  // peak of the pitch-rate swing
    float accel_z_peak_mps2     = 0.0f;  // peak of the z-accel bump
};

/**
 * @brief A wheel jammed from at_s on: against a wall or post, or a seized
 * gearbox. The tyre stops turning and skids on the floor, so the motor stalls
 * and the other wheel swings the body round it.
 */
struct PinnedWheel
{
    bool  enabled = false;
    bool  left    = true;
    float at_s    = 0.0f; // run time the wheel jams
};

struct Params
{
    // Body.
    float mass_kg          = 0.0f;
    float yaw_inertia_kgm2 = 0.0f;
    float wheel_base_mm    = 0.0f;
    float wheel_radius_mm  = 0.0f;

    // Motor and gearbox, per wheel. Inductance is ignored: its time constant
    // (well under 1 ms) is far below the mechanical one.
    float battery_v           = 0.0f;
    float winding_ohm         = 0.0f;
    float back_emf_vs_per_rad = 0.0f; // = torque constant, N m / A, at the motor shaft
    float gear_ratio          = 0.0f;
    float rotating_mass_kg    = 0.0f; // rotor and wheel inertia reflected to the tyre, per wheel

    // Traction: a stiff tyre up to a friction limit of mu * (wheel load).
    float      mu_left                  = 0.0f;
    float      mu_right                 = 0.0f;
    float      tyre_stiffness_n_per_mps = 0.0f;
    SlipEvents slip;

    // Sensors.
    float counts_per_mm_left    = 0.0f;
    float counts_per_mm_right   = 0.0f;
    float gyro_noise_degps_rthz = 0.0f; // white yaw-rate noise density, deg/s/sqrt(Hz)
    float gyro_bias_degps       = 0.0f; // constant yaw-rate offset (+ = CCW)

    Seam        seam;
    PinnedWheel pinned_wheel;

    uint64_t seed = 1;
};

/// The nominal model: matches MOTOR_KM, MOTOR_TM and the geometry config, no
/// slip events, encoders at the firmware's MM_PER_TICK, perfect gyro, no seam.
Params nominal();

/// Encoder resolution of the next (STM32) mouse, counts per mm of wheel travel.
constexpr float NEW_MOUSE_COUNTS_PER_MM = 62.6f;

/**
 * @brief The plant, wired to fake drivers. Drivetrain and Motion take the
 * driver members; step() once per control tick after Motion::update().
 */
class Robot
{
  public:
    explicit Robot(const Params& params);
    Robot(const Robot&)            = delete;
    Robot& operator=(const Robot&) = delete;

    /// Advances the plant one control tick (LOOP_INTERVAL_S) on the commanded duties.
    void step();

    // Truth.
    float timeS() const { return time_s_; }
    float distanceMm() const { return distance_m_ * 1000.0f; }
    float yawDeg() const;
    float wheelMmps(bool left) const;  // tyre surface speed
    float groundMmps(bool left) const; // contact-point ground speed
    bool  slipping(bool left) const;   // traction at its limit
    // Sensors.
    float encoderMm(bool left) const; // what the encoder's counts mean in mm
    float imuYawDeg() const;
    float pitchRateDegps() const { return pitch_rate_degps_; }
    float accelZMps2() const { return accel_z_mps2_; }
    int   seamCrossings() const { return seam_crossings_; }
    float volts(bool left) const;
    float amps(bool left) const; // motor winding current, + = driving forward

    Motor   left_motor;
    Motor   right_motor;
    Encoder left_encoder;
    Encoder right_encoder;
    IMU     imu;

  private:
    struct Wheel
    {
        float  tyre_mps     = 0.0f; // tyre surface speed
        double tyre_m       = 0.0;  // tyre surface travel (what the encoder turns with)
        double ground_m     = 0.0;  // contact-point travel along the path
        double extra_m      = 0.0;  // seam over-read already applied
        float  slip_left_s  = 0.0f; // time left in a random slip event
        float  seam_left_s  = 0.0f; // time left on the seam step
        bool   seam_crossed = false;
        bool   at_limit     = false;
        float  volts        = 0.0f;
    };

    void  substep(float dt_s);
    float traction(Wheel& w, float mu, float ground_mps);
    void  updateSensors(float dt_s);
    float uniform();
    float gaussian();

    // Positions add up ~10^5 substeps a run; double keeps rounding off the mm.
    Params   p_;
    Wheel    left_;
    Wheel    right_;
    float    v_mps_            = 0.0f;
    float    omega_radps_      = 0.0f;
    double   yaw_rad_          = 0.0;
    double   distance_m_       = 0.0;
    double   imu_yaw_deg_      = 0.0;
    double   time_s_           = 0.0;
    float    seam_t_s_         = -1.0f; // time since the body reached the seam; < 0 before
    float    pitch_rate_degps_ = 0.0f;
    float    accel_z_mps2_     = 0.0f;
    int      seam_crossings_   = 0;
    Wheel*   pinned_           = nullptr; // the jammed wheel, once PinnedWheel.at_s passes
    uint64_t rng_;
};

enum class Scenario
{
    kStraight10, // 10 cells straight from rest to rest, heading hold on
    kInPlace90,  // 90 deg CCW spin in place
    kSmooth90,   // roll into a 90 deg CCW smooth turn and stop a cell later
};

/// Parses "straight10", "ip90" or "ss90".
bool parseScenario(const char* name, Scenario* out);

struct RunOptions
{
    float speed_mmps  = 1000.0f; // straight cruise
    float accel_mmps2 = 2500.0f;
    // Crash watchdog: on, inhibited, fed the plant's motor current (as the
    // STM32's IPROPI would) or not, and a hook for the motor cut.
    bool crash_watchdog    = false;
    bool crash_inhibit     = false;
    bool sense_current     = false;
    void (*on_crash_cut)() = nullptr;
};

/// One control tick of a run, for traces.
struct Sample
{
    float time_s;
    float target_mm;   // forward profile position
    float target_deg;  // rotation profile position
    float distance_mm; // truth
    float yaw_deg;     // truth
    float encoder_left_mm;
    float encoder_right_mm;
    float imu_yaw_deg;
    float wheel_left_mmps;
    float wheel_right_mmps;
    float ground_left_mmps;
    float ground_right_mmps;
    float volts_left;
    float volts_right;
    bool  slip_left;
    bool  slip_right;
    float pitch_rate_degps;
    float accel_z_mps2;
};

struct Result
{
    bool       finished;    // every move ended before the guard time
    float      duration_s;  // including the settle time after the last move
    float      distance_mm; // true body-centre path length
    float      yaw_deg;     // true heading, + = CCW
    float      imu_yaw_deg;
    float      encoder_mm; // mean of the two encoders
    float      peak_pitch_rate_degps;
    float      peak_accel_z_mps2;
    int        seam_crossings;
    CrashCause crash_cause;           // kNone if the watchdog never tripped
    float      crash_trip_s;          // run time of the trip; < 0 if none
    float      peak_volts_after_trip; // largest |motor volts| on any tick after it
};

/// Runs a scenario; on_sample (if set) sees every control tick.
Result run(Scenario scenario, const Params& params, const RunOptions& options = {},
           const std::function<void(const Sample&)>& on_sample = nullptr);
} // namespace physim

#endif // TESTS_SIM_PHYSIM_H
