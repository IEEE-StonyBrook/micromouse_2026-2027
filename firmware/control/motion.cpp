#include "control/motion.h"

#include <cmath>

#include "common/log.h"
#include "common/utils.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "control/run_log.h"
#include "drivers/imu.h"
#include "hal/irq.h"
#include "hal/time.h"

namespace
{
constexpr float    kPositionToleranceMm        = 0.125f;
constexpr float    kForwardSettledVelocityMmps = 1.0f;
constexpr uint32_t kDefaultMotionWaitTimeoutMs = 5000;
constexpr uint32_t kSpinTurnPreStopTimeoutMs   = 1000;

bool timedOut(uint32_t start_ms, uint32_t timeout_ms)
{
    return timeout_ms > 0 && static_cast<uint32_t>(hal::time::nowMs() - start_ms) >= timeout_ms;
}
} // namespace

Motion::Motion(Drivetrain* drivetrain, IMU* imu)
    : drivetrain_(drivetrain), imu_(imu), forward_controller_(FWD_KP, FWD_KD, LOOP_FREQUENCY_HZ),
      rotation_controller_(ROT_KP, ROT_KD, LOOP_FREQUENCY_HZ)
{
    forward_controller_.setOutputLimit(MAX_VOLTAGE);
    rotation_controller_.setOutputLimit(MAX_VOLTAGE);

    reset();
    // No tick runs yet, so the reset applies now rather than on the first tick.
    hal::irq::Lock lock;
    applyPendingCommands();
}

void Motion::reset()
{
    reset_drive_system();
}

void Motion::reset_drive_system()
{
    issue(Command::Op::ResetDriveSystem);
    // IMU yaw belongs to the IMU's UART interrupt, not the control tick, and
    // IMU::reset() logs, so it zeroes here on the main loop. After the issue,
    // so a tick between the two never holds the old heading on a zeroed yaw.
    imu_->reset();
}

void Motion::reset_drive_control()
{
    issue(Command::Op::ResetDriveControl);
}

void Motion::resetControlHistory()
{
    forward_controller_.reset();
    rotation_controller_.reset();
    encoder_rotation_ticks_ = 0;

    prev_left_cmd_vel_mmps_               = 0.0f;
    prev_right_cmd_vel_mmps_              = 0.0f;
    side_error_prev_norm_                 = 0.0f;
    side_error_prev_valid_                = false;
    latest_wall_state_                    = {};
    latest_steering_adjustment_degps_     = 0.0f;
    latest_heading_hold_adjustment_degps_ = 0.0f;
    line_steering_adjustment_degps_       = 0.0f;
    line_steering_valid_                  = false;
    latest_line_rotation_volts_           = 0.0f;
    latest_line_differential_volts_       = 0.0f;
    latest_motor_differential_volts_      = 0.0f;
    clearWallSteeringStats();
}

void Motion::set_wall_distances(float left_mm, float front_mm, float right_mm)
{
    set_wall_distances(left_mm, front_mm, right_mm, left_mm, front_mm, right_mm);
}

void Motion::set_wall_distances(float left_raw_mm, float front_raw_mm, float right_raw_mm,
                                float left_filtered_mm, float front_filtered_mm,
                                float right_filtered_mm)
{
    // The main loop owns these and the tick only reads them; the lock keeps a
    // tick from steering on half old, half new distances.
    hal::irq::Lock lock;
    left_raw_wall_mm_  = left_raw_mm;
    front_raw_wall_mm_ = front_raw_mm;
    right_raw_wall_mm_ = right_raw_mm;
    left_wall_mm_      = left_filtered_mm;
    front_wall_mm_     = front_filtered_mm;
    right_wall_mm_     = right_filtered_mm;
}

void Motion::set_steering_mode(tof_wall::SteeringMode mode)
{
    issue(Command::Op::SetSteeringMode, static_cast<float>(static_cast<int>(mode)));
}

tof_wall::SteeringMode Motion::steeringMode() const
{
    return view().steering_mode;
}

bool Motion::wallLeft()
{
    return tof_wall::wallLeft(left_wall_mm_);
}

bool Motion::wallFront()
{
    return tof_wall::wallFront(front_wall_mm_);
}

bool Motion::wallRight()
{
    return tof_wall::wallRight(right_wall_mm_);
}

float Motion::leftDistance()
{
    return left_wall_mm_;
}

float Motion::frontDistance()
{
    return front_wall_mm_;
}

float Motion::rightDistance()
{
    return right_wall_mm_;
}

float Motion::leftRawDistance()
{
    return left_raw_wall_mm_;
}

float Motion::frontRawDistance()
{
    return front_raw_wall_mm_;
}

float Motion::rightRawDistance()
{
    return right_raw_wall_mm_;
}

tof_wall::WallState Motion::wallSteeringState() const
{
    return latest_wall_state_;
}

float Motion::wallSteeringAdjustmentDegps() const
{
    return latest_steering_adjustment_degps_;
}

Motion::WallSteeringStats Motion::wallSteeringStats() const
{
    WallSteeringStats stats;
    stats.peak_degps = wall_steering_peak_degps_;
    stats.average_degps =
        wall_steering_samples_ > 0
            ? wall_steering_sum_degps_ / static_cast<float>(wall_steering_samples_)
            : 0.0f;
    stats.samples        = wall_steering_samples_;
    stats.source_changes = wall_steering_source_changes_;
    stats.both_samples   = wall_steering_both_samples_;
    stats.single_samples = wall_steering_single_samples_;
    return stats;
}

void Motion::resetWallSteeringStats()
{
    issue(Command::Op::ResetWallSteeringStats);
}

void Motion::clearWallSteeringStats()
{
    wall_steering_peak_degps_        = 0.0f;
    wall_steering_sum_degps_         = 0.0f;
    wall_steering_samples_           = 0;
    wall_steering_source_changes_    = 0;
    wall_steering_both_samples_      = 0;
    wall_steering_single_samples_    = 0;
    wall_steering_prev_source_       = tof_wall::SteeringSource::None;
    wall_steering_prev_source_valid_ = false;
}

void Motion::set_line_steering_adjustment_degps(float adjustment_degps, bool valid)
{
    issue(Command::Op::SetLineSteering, adjustment_degps, valid ? 1.0f : 0.0f);
}

void Motion::clear_line_steering_adjustment()
{
    issue(Command::Op::ClearLineSteering);
}

float Motion::lineSteeringAdjustmentDegps() const
{
    return view().line_steering_adjustment_degps;
}

bool Motion::lineSteeringValid() const
{
    return view().line_steering_valid;
}

float Motion::lineRotationVolts() const
{
    return latest_line_rotation_volts_;
}

float Motion::lineDifferentialVolts() const
{
    return latest_line_differential_volts_;
}

float Motion::motorDifferentialVolts() const
{
    return latest_motor_differential_volts_;
}

void Motion::set_heading_hold(float target_yaw_deg)
{
    issue(Command::Op::SetHeadingHold, utils::wrapAngle180(target_yaw_deg));
}

void Motion::clear_heading_hold()
{
    issue(Command::Op::ClearHeadingHold);
}

bool Motion::headingHoldActive() const
{
    return view().heading_hold_enabled;
}

float Motion::headingHoldTargetYawDeg() const
{
    return view().heading_hold_target_yaw_deg;
}

float Motion::headingHoldErrorDeg() const
{
    return latest_heading_hold_error_deg_;
}

float Motion::headingHoldAdjustmentDegps() const
{
    return latest_heading_hold_adjustment_degps_;
}

float Motion::position() const
{
    return view().fwd.position();
}

float Motion::wheel_position(WheelSide side)
{
    hal::irq::Lock lock;
    return drivetrainResetQueued() ? 0.0f : drivetrain_->position(side);
}

int32_t Motion::encoder_ticks(WheelSide side) const
{
    hal::irq::Lock lock;
    return drivetrainResetQueued() ? 0 : drivetrain_->ticks(side);
}

float Motion::velocity() const
{
    return view().fwd.velocity();
}

float Motion::acceleration() const
{
    return view().fwd.acceleration();
}

float Motion::angle() const
{
    return view().rot.position();
}

float Motion::omega() const
{
    return view().rot.velocity();
}

float Motion::alpha() const
{
    return view().rot.acceleration();
}

float Motion::yaw_deg() const
{
    return imu_->robot_angle();
}

float Motion::yaw_rate_degps() const
{
    return imu_->robot_omega();
}

void Motion::set_target_velocity(float velocity_mmps)
{
    issue(Command::Op::SetTargetVelocity, velocity_mmps);
}

void Motion::set_final_velocity(float velocity_mmps)
{
    issue(Command::Op::SetFinalVelocity, velocity_mmps);
}

void Motion::extend_move(float distance_mm)
{
    issue(Command::Op::ExtendMove, distance_mm);
}

void Motion::begin_motion_sequence()
{
    issue(Command::Op::BeginSequence);
}

void Motion::end_motion_sequence()
{
    issue(Command::Op::EndSequence);
}

void Motion::start_move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                        float accel_mmps2)
{
    // The tick starts the profile from the velocity it has when it applies this.
    issue(Command::Op::StartMove, distance_mm, top_speed_mmps, final_speed_mmps, accel_mmps2);
}

bool Motion::move_finished() const
{
    return !view().fwd.active();
}

void Motion::move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                  float accel_mmps2)
{
    start_move(distance_mm, top_speed_mmps, final_speed_mmps, accel_mmps2);
}

bool Motion::move_settled(float velocity_tolerance_mmps) const
{
    return view().fwd.settled(velocity_tolerance_mmps);
}

bool Motion::turn_settled(float velocity_tolerance_degps) const
{
    return view().rot.settled(velocity_tolerance_degps);
}

bool Motion::stop_forward(float accel_mmps2, uint32_t timeout_ms)
{
    issue(Command::Op::StopForward, accel_mmps2);
    return wait_until_forward_settled(timeout_ms);
}

bool Motion::wait_until_forward_settled(uint32_t timeout_ms)
{
    const uint32_t start_ms = hal::time::nowMs();

    while (!move_settled(kForwardSettledVelocityMmps))
    {
        if (timedOut(start_ms, timeout_ms))
        {
            LOG_ERROR("Motion: forward stop timeout position_mm=" << position() << " velocity_mmps="
                                                                  << velocity());
            issue(Command::Op::ForwardAbort);
            return false;
        }
        hal::time::sleepMs(2);
    }

    issue(Command::Op::ForwardSettled);
    return true;
}

void Motion::start_turn(float degrees, float top_speed_degps, float final_speed_degps,
                        float accel_degps2)
{
    issue(Command::Op::StartTurn, degrees, top_speed_degps, final_speed_degps, accel_degps2);
}

bool Motion::turn_finished() const
{
    return !view().rot.active();
}

void Motion::turn(float degrees, float top_speed_degps, float final_speed_degps, float accel_degps2)
{
    start_turn(degrees, top_speed_degps, final_speed_degps, accel_degps2);
}

bool Motion::spin_turn(float degrees, float omega_degps, float alpha_degps2)
{
    if (!stop_forward(ROBOT_BASE_ACCEL_MMPS2, kSpinTurnPreStopTimeoutMs))
    {
        LOG_ERROR("Motion: spin_turn cancelled before turn degrees="
                  << degrees << " forward_velocity_mmps=" << velocity());
        issue(Command::Op::ResetRotation);
        return false;
    }

    issue(Command::Op::SpinTurn, degrees, omega_degps, alpha_degps2);
    return true;
}

bool Motion::turn_IP180()
{
    static int direction      = 1;
    const int  next_direction = -direction;
    if (!spin_turn(next_direction * 180.0f, ROBOT_MAX_TURN_SPEED_DEGPS,
                   ROBOT_BASE_ANGULAR_ACCEL_DEGPS2))
        return false;
    direction = next_direction;
    return true;
}

bool Motion::turn_IP90R()
{
    return spin_turn(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

bool Motion::turn_IP90L()
{
    return spin_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Motion::stop_at(float position_mm)
{
    issue(Command::Op::StopAt, position_mm);
}

void Motion::stop_after(float distance_mm)
{
    issue(Command::Op::StopAfter, distance_mm);
}

void Motion::set_position(float position_mm)
{
    issue(Command::Op::SetPosition, position_mm);
}

void Motion::adjust_forward_position(float delta_mm)
{
    issue(Command::Op::AdjustPosition, delta_mm);
}

float Motion::smoothTurnLeadMm(bool turn_left, float turn_centre_ahead_mm) const
{
    // mazerunner-core: turn_point = FULL_CELL + HALF_CELL - entry_offset.
    return turn_centre_ahead_mm - smooth_turn_.side(turn_left).entry_mm;
}

float Motion::smoothTurnExitMm(bool turn_left, float end_after_centre_mm) const
{
    // mazerunner-core: move(SENSING_POSITION - (HALF_CELL + exit_offset)).
    // Clamp so an exit offset larger than the room left never reverses the
    // robot; that case means the turn is too wide for the cell (raise ALPHA).
    const float exit_mm = end_after_centre_mm - smooth_turn_.side(turn_left).exit_mm;
    return (exit_mm > 0.0f) ? exit_mm : 0.0f;
}

float Motion::start_smooth_turn_entry(bool turn_left, float turn_centre_ahead_mm)
{
    float lead_mm = smoothTurnLeadMm(turn_left, turn_centre_ahead_mm);
    if (lead_mm < 0.0f)
    {
        // Already past the turn point (slow planning). Turn now, as
        // mazerunner-core's wait loop does: a late turn is recoverable, but
        // skipping it would leave the maze heading turned and the robot not.
        LOG_WARNING("Motion: smooth turn late by " << -lead_mm << " mm; turning now");
        lead_mm = 0.0f;
    }
    // final_speed == top_speed: the profile reaches the turn point still
    // rolling at turn speed and carries it through the arc (mazerunner sets
    // target velocity to SEARCH_TURN_SPEED). start_move() carries the current
    // velocity in as start_speed.
    start_move(lead_mm, smooth_turn_.speed_mmps, smooth_turn_.speed_mmps, ROBOT_SEARCH_ACCEL_MMPS2);
    return lead_mm;
}

void Motion::start_smooth_turn_arc(bool turn_left, float heading_error_deg)
{
    // mazerunner-core: motion.turn(params.angle, params.omega, 0, params.alpha).
    const float angle_deg = smooth_turn_.side(turn_left).angle_deg;
    issue(Command::Op::SmoothArc, (turn_left ? angle_deg : -angle_deg) + heading_error_deg,
          smooth_turn_.omega_degps, 0.0f, smooth_turn_.alpha_degps2);
}

void Motion::update()
{
    // Mazerunner systick.update() shape:
    //   encoders.update(); motion.update(); motors.update_controllers(...)
    // We add imu_->update() because IMU owns rotation tracking
    // (substitutes mazerunner's encoder-derived robot_rot_change).
    // Main-loop commands apply first, whole and in order, so the rest of the
    // tick never sees one half-written.
    {
        hal::irq::Lock lock;
        applyPendingCommands();
    }
    drivetrain_->update();
    imu_->update();
    // A tripped watchdog keeps the motors cut and the move frozen.
    if (!crash_.tripped())
    {
        forward_.update();

        rotation_.update();

        runPositionControl();
    }
    checkCrash();
}

void Motion::checkCrash()
{
    // Off and clear (the Pico default): skip the work entirely.
    if (!crash_.enabled() && !crash_.tripped())
        return;
    CrashInputs in;
    in.left_volts    = drivetrain_->leftVolts();
    in.right_volts   = drivetrain_->rightVolts();
    in.battery_volts = drivetrain_->batteryVoltage();
    in.left_mmps     = drivetrain_->velocity(WheelSide::LEFT);
    in.right_mmps    = drivetrain_->velocity(WheelSide::RIGHT);
    in.fwd_error_mm  = forward_controller_.error();
    if (current_reader_ != nullptr)
        in.has_current = current_reader_(current_context_, &in.left_amps, &in.right_amps);
    if (crash_.update(in))
    {
        drivetrain_->stop();
        if (crash_cut_hook_ != nullptr)
            crash_cut_hook_();
    }
}

void Motion::set_motor_current_reader(CurrentReader reader, void* context)
{
    current_reader_  = reader;
    current_context_ = context;
}

void Motion::clear_crash()
{
    reset_drive_system();
    crash_.clear();
}

void Motion::runPositionControl()
{
    const bool forward_commanded  = forward_.active() || std::fabs(forward_.velocity()) > 1e-3f;
    const bool rotation_commanded = rotation_.active() || std::fabs(rotation_.velocity()) > 1e-3f;

    if (!forward_commanded && !rotation_commanded)
    {
        drivetrain_->stop();
        if (motion_sequence_active_)
        {
            latest_wall_state_ =
                tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
            latest_steering_adjustment_degps_     = 0.0f;
            latest_heading_hold_adjustment_degps_ = 0.0f;
            latest_line_rotation_volts_           = 0.0f;
            latest_line_differential_volts_       = 0.0f;
            latest_motor_differential_volts_      = 0.0f;
            if (heading_hold_enabled_)
                latest_heading_hold_error_deg_ =
                    utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
            return;
        }
        resetControlHistory();
        latest_wall_state_ =
            tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
        latest_steering_adjustment_degps_     = 0.0f;
        latest_heading_hold_adjustment_degps_ = 0.0f;
        latest_line_rotation_volts_           = 0.0f;
        latest_line_differential_volts_       = 0.0f;
        latest_motor_differential_volts_      = 0.0f;
        if (heading_hold_enabled_)
            latest_heading_hold_error_deg_ =
                utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
        return;
    }

    float fwd_change_mm = drivetrain_->fwdChangeMm();
    // Smooth-turn arcs use encoder yaw; see SMOOTH_TURN_ENCODER_HOLDOVER_MS.
    if (encoder_rotation_ticks_ == UINT16_MAX && !rotation_.active())
        encoder_rotation_ticks_ =
            static_cast<uint16_t>(SMOOTH_TURN_ENCODER_HOLDOVER_MS * LOOP_FREQUENCY_HZ / 1000.0f);
    const bool encoder_rotation = ROT_FEEDBACK_ENCODER || encoder_rotation_ticks_ > 0;
    if (encoder_rotation && encoder_rotation_ticks_ != UINT16_MAX)
        --encoder_rotation_ticks_;
    float rot_change_deg = encoder_rotation ? drivetrain_->rotChangeDeg()
                                            : imu_->robot_rot_change(); // per-tick delta (deg)
    float fwd_velocity   = forward_.velocity();
    float rot_velocity   = rotation_.velocity();

    // Single-rate PD — both forward and rotation update every control tick,
    // exactly like UKMARS mazerunner-core. Rotation feedback comes from the
    // IMU as a held-flat per-tick delta; this removes the 10 ms staircase
    // that the old 100 Hz gate produced in rotation_output_ and stops the
    // step-input-driven oscillation on spin turns.
    // mazerunner-core runs both PDs every tick. During a spin the forward
    // setpoint is 0, so this holds position; skipping it let motor imbalance
    // walk the robot ~30 mm forward per 360 deg.
    const float forward_output      = forward_controller_.update(fwd_velocity, fwd_change_mm);
    const bool  straight_move       = forward_.active() && !rotation_.active();
    float       wall_adjustment     = 0.0f;
    float       heading_adjustment  = 0.0f;
    float       steering_adjustment = 0.0f;
    const bool  line_steering_used  = straight_move && line_steering_valid_;
    if (line_steering_used)
    {
        steering_adjustment                   = line_steering_adjustment_degps_;
        latest_heading_hold_adjustment_degps_ = 0.0f;
    }
    else
    {
        wall_adjustment =
            (TOF_STEERING_ENABLE && steering_mode_ != tof_wall::SteeringMode::STEERING_OFF)
                ? wallSteeringAdjustment(fwd_velocity, rot_velocity)
                : 0.0f;
        heading_adjustment  = headingHoldAdjustment(fwd_velocity, rot_velocity);
        steering_adjustment = wall_adjustment + heading_adjustment;
    }
#if !TOF_STEERING_ENABLE
    wallSteeringAdjustment(fwd_velocity, rot_velocity); // diagnostics only
#endif
    const float rotation_output =
        rotation_controller_.update(rot_velocity, rot_change_deg, steering_adjustment);
    // Diagnostic only: the direct one-tick voltage contribution of the line
    // steering term before the rotation controller's accumulated yaw error.
    latest_line_rotation_volts_ =
        line_steering_used
            ? utils::clampAbs((ROT_KP * line_steering_adjustment_degps_ * LOOP_INTERVAL_S) +
                                  (ROT_KD * line_steering_adjustment_degps_),
                              MAX_VOLTAGE)
            : 0.0f;
    latest_line_differential_volts_ = 2.0f * latest_line_rotation_volts_;

    // Mix forward + rotation outputs into per-wheel volts (mazerunner shape).
    float left_volts  = forward_output - rotation_output;
    float right_volts = forward_output + rotation_output;

    float wheelbase_radius = WHEEL_BASE_MM / 2.0f;
    float tangential_vel   = rot_velocity * (M_PI / 180.0f) * wheelbase_radius;

    float left_vel  = fwd_velocity - tangential_vel;
    float right_vel = fwd_velocity + tangential_vel;

    // Per-wheel acceleration via per-loop diff. Multiplying by LOOP_FREQUENCY
    // converts the per-loop velocity diff into mm/s².
    float left_accel         = (left_vel - prev_left_cmd_vel_mmps_) * LOOP_FREQUENCY_HZ;
    float right_accel        = (right_vel - prev_right_cmd_vel_mmps_) * LOOP_FREQUENCY_HZ;
    prev_left_cmd_vel_mmps_  = left_vel;
    prev_right_cmd_vel_mmps_ = right_vel;

    left_volts += drivetrain_->feedforward(WheelSide::LEFT, left_vel, left_accel);
    right_volts += drivetrain_->feedforward(WheelSide::RIGHT, right_vel, right_accel);

    // Clamp at hardware boundary — no slew limit (would blunt FF response).
    left_volts                       = utils::clampAbs(left_volts, MAX_VOLTAGE);
    right_volts                      = utils::clampAbs(right_volts, MAX_VOLTAGE);
    latest_motor_differential_volts_ = right_volts - left_volts;

    drivetrain_->setVoltage(left_volts, right_volts);

    // The run log's ring keeps the last samples of a long run, the ones
    // leading up to a crash or HALT.
    if (run_log_ != nullptr && run_log_->due())
    {
        RunLog::Sample s;
        s.target_deg    = rotation_.position();
        s.yaw_deg       = yaw_deg();
        s.rot_error_deg = rotation_controller_.error();
        s.rot_volts     = rotation_output;
        s.hold_err_deg  = latest_heading_hold_error_deg_;
        s.wall_degps    = wall_adjustment;
        s.hold_degps    = heading_adjustment;
        s.fwd_mmps      = fwd_velocity;
        s.left_mmps     = drivetrain_->velocity(WheelSide::LEFT);
        s.right_mmps    = drivetrain_->velocity(WheelSide::RIGHT);
        s.left_volts    = left_volts;
        s.right_volts   = right_volts;
        s.left_tof_mm   = left_wall_mm_;
        s.front_tof_mm  = front_wall_mm_;
        s.right_tof_mm  = right_wall_mm_;
        s.pos_mm        = forward_.position();
        const bool saturated =
            std::fabs(left_volts) >= MAX_VOLTAGE || std::fabs(right_volts) >= MAX_VOLTAGE;
        uint16_t events = 0;
        events |= encoder_rotation ? RunLog::ENCODER_ROTATION : 0;
        events |= latest_wall_state_.left_wall ? RunLog::WALL_LEFT : 0;
        events |= latest_wall_state_.front_wall ? RunLog::WALL_FRONT : 0;
        events |= latest_wall_state_.right_wall ? RunLog::WALL_RIGHT : 0;
        events |= saturated ? RunLog::VOLTS_SATURATED : 0;
        events |= forward_.active() ? RunLog::FORWARD_ACTIVE : 0;
        events |= rotation_.active() ? RunLog::ROTATION_ACTIVE : 0;
        s.events = events;
        run_log_->record(s);
    }
}

void Motion::start_trace(uint16_t period_ms)
{
    issue(Command::Op::StartTrace, static_cast<float>(period_ms));
}

void Motion::stop_trace()
{
    issue(Command::Op::StopTrace);
}

float Motion::wallSteeringAdjustment(float fwd_velocity_mmps, float rot_velocity_degps)
{
    // Not forward_.active(): a search straight rolls on a finished profile at
    // its final speed (mazerunner applies steering every tick regardless).
    const bool straight_move =
        !rotation_.active() && fwd_velocity_mmps > 1.0f && std::fabs(rot_velocity_degps) < 1.0f;
    if (!straight_move)
    {
        side_error_prev_valid_ = false;
        latest_wall_state_ =
            tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
        latest_steering_adjustment_degps_ = 0.0f;
        return 0.0f;
    }

    latest_wall_state_ =
        tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
    if (!latest_wall_state_.steering_allowed)
    {
        side_error_prev_valid_            = false;
        latest_steering_adjustment_degps_ = 0.0f;
        return 0.0f;
    }

    const float side_error_delta_norm_per_s =
        side_error_prev_valid_
            ? (latest_wall_state_.side_error_norm - side_error_prev_norm_) * LOOP_FREQUENCY_HZ
            : 0.0f;
    side_error_prev_norm_             = latest_wall_state_.side_error_norm;
    side_error_prev_valid_            = true;
    latest_steering_adjustment_degps_ = tof_wall::steeringAdjustmentDegps(
        latest_wall_state_.side_error_norm, side_error_delta_norm_per_s);
    recordWallSteeringStats(latest_steering_adjustment_degps_, latest_wall_state_.source);
    return latest_steering_adjustment_degps_;
}

void Motion::recordWallSteeringStats(float adjustment_degps, tof_wall::SteeringSource source)
{
    if (wall_steering_samples_ == 0 ||
        std::fabs(adjustment_degps) > std::fabs(wall_steering_peak_degps_))
        wall_steering_peak_degps_ = adjustment_degps;

    wall_steering_sum_degps_ += adjustment_degps;
    ++wall_steering_samples_;

    if (source == tof_wall::SteeringSource::Both)
        ++wall_steering_both_samples_;
    else if (source == tof_wall::SteeringSource::Left || source == tof_wall::SteeringSource::Right)
        ++wall_steering_single_samples_;

    if (wall_steering_prev_source_valid_ && source != wall_steering_prev_source_)
        ++wall_steering_source_changes_;
    wall_steering_prev_source_       = source;
    wall_steering_prev_source_valid_ = true;
}

float Motion::headingHoldAdjustment(float fwd_velocity_mmps, float rot_velocity_degps)
{
    // Not forward_.active(): a search straight rolls on a finished profile at
    // its final speed (mazerunner applies steering every tick regardless).
    const bool straight_move =
        !rotation_.active() && fwd_velocity_mmps > 1.0f && std::fabs(rot_velocity_degps) < 1.0f;
    if (!heading_hold_enabled_ || !straight_move)
    {
        latest_heading_hold_adjustment_degps_ = 0.0f;
        if (heading_hold_enabled_)
            latest_heading_hold_error_deg_ =
                utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
        return 0.0f;
    }

    latest_heading_hold_error_deg_ = utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
#if MAZE_HEADING_HOLD_ENABLE
    latest_heading_hold_adjustment_degps_ =
        utils::clampAbs(latest_heading_hold_error_deg_ * MAZE_HEADING_HOLD_KP_DEGPS_PER_DEG,
                        MAZE_HEADING_HOLD_MAX_DEGPS);
#else
    latest_heading_hold_adjustment_degps_ = 0.0f;
#endif
    return latest_heading_hold_adjustment_degps_;
}

void Motion::stop()
{
    issue(Command::Op::Stop);
}

void Motion::disable_drive()
{
    issue(Command::Op::DisableDrive);
}

void Motion::emergency_stop()
{
    issue(Command::Op::EmergencyStop);
    imu_->reset(); // main-loop side, as in reset_drive_system()
}

bool Motion::wait_until_position(float position_mm, uint32_t timeout_ms)
{
    const uint32_t start_ms = hal::time::nowMs();

    while (position() < position_mm - kPositionToleranceMm)
    {
        if (timedOut(start_ms, timeout_ms))
        {
            LOG_ERROR("Motion: wait_until_position timeout target_mm="
                      << position_mm << " position_mm=" << position()
                      << " velocity_mmps=" << velocity());
            issue(Command::Op::Stop);
            return false;
        }
        hal::time::sleepMs(2);
    }

    return true;
}

void Motion::wait_until_position(float position_mm)
{
    (void)wait_until_position(position_mm, kDefaultMotionWaitTimeoutMs);
}

bool Motion::wait_until_distance(float distance_mm, uint32_t timeout_ms)
{
    const float target_mm = position() + distance_mm;
    return wait_until_position(target_mm, timeout_ms);
}

void Motion::wait_until_distance(float distance_mm)
{
    (void)wait_until_distance(distance_mm, kDefaultMotionWaitTimeoutMs);
}

// === Command latch ===

void Motion::issue(Command::Op op, float a, float b, float c, float d)
{
    hal::irq::Lock lock;
    if (latch_count_ == LATCH_CAPACITY)
    {
        // More commands than one tick should ever need: the tick has stalled
        // or this is a burst. Apply the backlog here; the lock keeps the tick
        // out, so it still lands whole and in order.
        ++latch_overflows_;
        applyPendingCommands();
    }
    latch_[latch_count_++] = Command{op, a, b, c, d};
}

void Motion::applyPendingCommands()
{
    for (uint8_t i = 0; i < latch_count_; i++)
        applyCommand(latch_[i]);
    latch_count_ = 0;
}

bool Motion::drivetrainResetQueued() const
{
    for (uint8_t i = 0; i < latch_count_; i++)
    {
        const Command::Op op = latch_[i].op;
        if (op == Command::Op::ResetDriveSystem || op == Command::Op::EmergencyStop ||
            op == Command::Op::ResetDriveControl || op == Command::Op::BeginSequence)
            return true;
    }
    return false;
}

Motion::View Motion::view() const
{
    hal::irq::Lock  lock;
    View            v{forward_,
                      rotation_,
                      steering_mode_,
                      heading_hold_enabled_,
                      heading_hold_target_yaw_deg_,
                      line_steering_adjustment_degps_,
                      line_steering_valid_,
                      motion_sequence_active_};
    const Commanded state{v.fwd,
                          v.rot,
                          v.steering_mode,
                          v.heading_hold_enabled,
                          v.heading_hold_target_yaw_deg,
                          v.line_steering_adjustment_degps,
                          v.line_steering_valid,
                          v.motion_sequence_active};
    for (uint8_t i = 0; i < latch_count_; i++)
        applyCommanded(latch_[i], state);
    return v;
}

void Motion::applyCommanded(const Command& command, const Commanded& state)
{
    using Op = Command::Op;
    switch (command.op)
    {
        case Op::ResetDriveSystem:
        case Op::EmergencyStop:
        case Op::BeginSequence:
        case Op::EndSequence:
            state.motion_sequence_active      = command.op == Op::BeginSequence;
            state.heading_hold_enabled        = false;
            state.heading_hold_target_yaw_deg = 0.0f;
            if (command.op != Op::EndSequence)
                state.steering_mode = tof_wall::SteeringMode::STEERING_OFF;
            [[fallthrough]];
        case Op::ResetDriveControl:
            state.fwd.reset();
            state.rot.reset();
            state.line_steering_adjustment_degps = 0.0f;
            state.line_steering_valid            = false;
            break;
        case Op::StartMove:
            state.fwd.start(command.a, state.fwd.velocity(), command.b, command.c, command.d);
            break;
        case Op::StartTurn:
        case Op::SmoothArc:
            state.rot.start(command.a, state.rot.velocity(), command.b, command.c, command.d);
            break;
        case Op::SpinTurn:
            state.rot.reset();
            state.rot.start(command.a, command.b, 0.0f, command.c);
            break;
        case Op::ResetRotation:
            state.rot.reset();
            break;
        case Op::StopAt:
            state.fwd.start(command.a - state.fwd.position(), state.fwd.velocity(),
                            std::fabs(state.fwd.velocity()), 0.0f, state.fwd.acceleration());
            break;
        case Op::StopAfter:
            state.fwd.start(command.a, state.fwd.velocity(), std::fabs(state.fwd.velocity()), 0.0f,
                            state.fwd.acceleration());
            break;
        case Op::StopForward:
            state.fwd.stop(command.a);
            break;
        case Op::SetPosition:
            state.fwd.setPosition(command.a);
            break;
        case Op::AdjustPosition:
            state.fwd.adjustPosition(command.a);
            break;
        case Op::SetTargetVelocity:
            state.fwd.setTargetSpeed(command.a);
            break;
        case Op::SetFinalVelocity:
            state.fwd.setFinalSpeed(command.a);
            break;
        case Op::ExtendMove:
            state.fwd.extendTarget(command.a);
            break;
        case Op::ForwardAbort:
            state.fwd.reset();
            break;
        case Op::SetHeadingHold:
            state.heading_hold_enabled        = true;
            state.heading_hold_target_yaw_deg = command.a;
            break;
        case Op::ClearHeadingHold:
        case Op::DisableDrive:
            state.heading_hold_enabled        = false;
            state.heading_hold_target_yaw_deg = 0.0f;
            break;
        case Op::SetSteeringMode:
            state.steering_mode = static_cast<tof_wall::SteeringMode>(static_cast<int>(command.a));
            break;
        case Op::SetLineSteering:
            state.line_steering_valid            = command.b != 0.0f;
            state.line_steering_adjustment_degps = state.line_steering_valid ? command.a : 0.0f;
            break;
        case Op::ClearLineSteering:
            state.line_steering_adjustment_degps = 0.0f;
            state.line_steering_valid            = false;
            break;
        case Op::ForwardSettled:
        case Op::Stop:
        case Op::StartTrace:
        case Op::StopTrace:
        case Op::ResetWallSteeringStats:
            break;
    }
}

void Motion::applyCommand(const Command& command)
{
    applyCommanded(command, Commanded{forward_, rotation_, steering_mode_, heading_hold_enabled_,
                                      heading_hold_target_yaw_deg_, line_steering_adjustment_degps_,
                                      line_steering_valid_, motion_sequence_active_});

    // The rest of each command: the drivetrain, controllers and diagnostics
    // that main-loop reads never need to see ahead of the tick.
    using Op = Command::Op;
    switch (command.op)
    {
        case Op::ResetDriveSystem:
        case Op::EmergencyStop:
        case Op::BeginSequence:
            latest_heading_hold_error_deg_ = 0.0f;
            [[fallthrough]];
        case Op::ResetDriveControl:
            drivetrain_->stop();
            drivetrain_->reset();
            resetControlHistory();
            break;
        case Op::EndSequence:
            drivetrain_->stop();
            resetControlHistory();
            latest_heading_hold_error_deg_ = 0.0f;
            break;
        case Op::SmoothArc:
            encoder_rotation_ticks_ = UINT16_MAX;
            break;
        case Op::ForwardAbort:
            drivetrain_->stop();
            [[fallthrough]];
        case Op::ForwardSettled:
            forward_controller_.reset();
            prev_left_cmd_vel_mmps_  = 0.0f;
            prev_right_cmd_vel_mmps_ = 0.0f;
            break;
        case Op::SetHeadingHold:
            latest_heading_hold_error_deg_ =
                utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
            break;
        case Op::DisableDrive:
            drivetrain_->stop();
            [[fallthrough]];
        case Op::ClearHeadingHold:
            latest_heading_hold_error_deg_        = 0.0f;
            latest_heading_hold_adjustment_degps_ = 0.0f;
            break;
        case Op::SetSteeringMode:
            side_error_prev_norm_             = 0.0f;
            side_error_prev_valid_            = false;
            latest_steering_adjustment_degps_ = 0.0f;
            break;
        case Op::Stop:
            drivetrain_->stop();
            break;
        case Op::StartTrace:
            if (run_log_ != nullptr)
                run_log_->start(static_cast<uint16_t>(command.a));
            break;
        case Op::StopTrace:
            if (run_log_ != nullptr)
                run_log_->stop();
            break;
        case Op::ResetWallSteeringStats:
            clearWallSteeringStats();
            break;
        default:
            break;
    }
}
