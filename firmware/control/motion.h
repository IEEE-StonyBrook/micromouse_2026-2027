#ifndef CONTROL_MOTION_H
#define CONTROL_MOTION_H

#include <cstdint>

#include "common/tof_wall_utils.h"
#include "config/motion.h"
#include "control/crash_detector.h"
#include "control/pid.h"
#include "control/profile.h"

class Drivetrain;
class IMU;

/**
 * @brief High-level motion controller (mazerunner-core Motion style)
 *
 * Per-tick: drivetrain.update() → forward.update() → rotation.update()
 * → runPositionControl(). Motion complete when both profiles report finished().
 *
 * `angle()` and `omega()` intentionally match mazerunner-core profile semantics.
 * Physical yaw remains available separately through `yaw_deg()` / `yaw_rate_degps()`.
 *
 * Threading: update() runs in the control-timer interrupt; everything else is
 * main-loop code. Mutators queue a Command that update() applies first, in
 * issue order (FIFO, never last-wins), so the tick never sees half a command.
 * Profile and setting reads return the live state with the queue applied, so
 * a read straight after a command already reflects it. Diagnostics computed by
 * the tick (heading-hold error, steering stats, the trace) catch up on the
 * next tick; encoder reads report 0 while a drivetrain reset is queued.
 */
class Motion
{
  public:
    struct WallSteeringStats
    {
        float    peak_degps     = 0.0f;
        float    average_degps  = 0.0f;
        uint32_t samples        = 0;
        uint32_t source_changes = 0;
        uint32_t both_samples   = 0;
        uint32_t single_samples = 0;
    };

    /**
     * @brief Per-side smooth turn shape: mazerunner-core TurnParameters.
     *
     * entry_mm: how far before the turn cell centre the arc starts.
     * exit_mm: how far past the centre (on the new heading) the arc ends.
     * front_trigger_mm: front ToF distance that starts the arc early when a
     * wall is ahead (mazerunner's sensor trigger); 0 = distance only.
     * angle_deg: arc size the encoders drive (mazerunner TurnParameters.angle).
     * Tyre scrub makes the body turn less than the wheels say at speed, so a
     * robot can need slightly more than 90 to come out square.
     * Left and right stay independent: tyre loading and motor asymmetry make
     * the two sides differ (Orion: entry 70 L / 55 R).
     */
    struct SmoothTurnSide
    {
        float entry_mm;
        float exit_mm;
        float front_trigger_mm;
        float angle_deg;
    };

    /**
     * @brief Live smooth turn parameters, seeded from config/motion.h.
     *
     * A RAM shadow so a corner can be tuned over serial without a rebuild, same
     * pattern as LineFollower::RuntimeTuning.
     */
    struct SmoothTurnTuning
    {
        float          speed_mmps   = ROBOT_SEARCH_TURN_SPEED_MMPS;
        float          omega_degps  = ROBOT_SMOOTH_TURN_OMEGA_DEGPS;
        float          alpha_degps2 = ROBOT_SMOOTH_TURN_ALPHA_DEGPS2;
        SmoothTurnSide left{ROBOT_SMOOTH_TURN_ENTRY_LEFT_MM, ROBOT_SMOOTH_TURN_EXIT_LEFT_MM,
                            ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM, ROBOT_SMOOTH_TURN_ANGLE_LEFT_DEG};
        SmoothTurnSide right{ROBOT_SMOOTH_TURN_ENTRY_RIGHT_MM, ROBOT_SMOOTH_TURN_EXIT_RIGHT_MM,
                             ROBOT_SMOOTH_TURN_FRONT_TRIGGER_MM, ROBOT_SMOOTH_TURN_ANGLE_RIGHT_DEG};
        // mazerunner EXTRA_WALL_ADJUST: front trigger shift per side wall seen.
        float extra_wall_adjust_mm = 0.0f;

        const SmoothTurnSide& side(bool turn_left) const { return turn_left ? left : right; }
    };

    Motion(Drivetrain* drivetrain, IMU* imu);

    void reset();

    // === Wall sensing (ToF) ===
    void set_wall_distances(float left_mm, float front_mm, float right_mm);
    void set_wall_distances(float left_raw_mm, float front_raw_mm, float right_raw_mm,
                            float left_filtered_mm, float front_filtered_mm,
                            float right_filtered_mm);
    void set_steering_mode(tof_wall::SteeringMode mode);
    tof_wall::SteeringMode steeringMode() const;
    bool                   wallLeft();
    bool                   wallFront();
    bool                   wallRight();
    bool                   see_left_wall() { return wallLeft(); }
    bool                   see_front_wall() { return wallFront(); }
    bool                   see_right_wall() { return wallRight(); }
    float                  leftDistance();
    float                  frontDistance();
    float                  rightDistance();
    float                  leftRawDistance();
    float                  frontRawDistance();
    float                  rightRawDistance();
    tof_wall::WallState    wallSteeringState() const;
    float                  wallSteeringAdjustmentDegps() const;
    WallSteeringStats      wallSteeringStats() const;
    void                   resetWallSteeringStats();
    void                   set_line_steering_adjustment_degps(float adjustment_degps, bool valid);
    void                   clear_line_steering_adjustment();
    float                  lineSteeringAdjustmentDegps() const;
    bool                   lineSteeringValid() const;
    float                  lineRotationVolts() const;
    float                  lineDifferentialVolts() const;
    float                  motorDifferentialVolts() const;
    void                   set_heading_hold(float target_yaw_deg);
    void                   clear_heading_hold();
    bool                   headingHoldActive() const;
    float                  headingHoldTargetYawDeg() const;
    float                  headingHoldErrorDeg() const;
    float                  headingHoldAdjustmentDegps() const;

    // === Mazerunner-core compatible motion names ===
    void    reset_drive_system();
    void    reset_drive_control();
    void    stop();
    void    disable_drive();
    void    emergency_stop();
    float   position() const;
    float   wheel_position(WheelSide side);
    int32_t encoder_ticks(WheelSide side) const;
    float   velocity() const;
    float   acceleration() const;
    float   angle() const;
    float   omega() const;
    float   alpha() const;
    float   yaw_deg() const;
    float   yaw_rate_degps() const;
    void    set_target_velocity(float velocity_mmps);
    void    set_final_velocity(float velocity_mmps);
    void    extend_move(float distance_mm);
    bool    move_settled(float velocity_tolerance_mmps = 1.0f) const;
    bool    turn_settled(float velocity_tolerance_degps = 1.0f) const;
    bool    stop_forward(float accel_mmps2, uint32_t timeout_ms);
    bool    wait_until_forward_settled(uint32_t timeout_ms);
    void    begin_motion_sequence();
    void    end_motion_sequence();
    void    start_move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                       float accel_mmps2);
    bool    move_finished() const;
    void move(float distance_mm, float top_speed_mmps, float final_speed_mmps, float accel_mmps2);
    void start_turn(float degrees, float top_speed_degps, float final_speed_degps,
                    float accel_degps2);
    bool turn_finished() const;
    void turn(float degrees, float top_speed_degps, float final_speed_degps, float accel_degps2);
    bool spin_turn(float degrees, float omega_degps, float alpha_degps2);
    bool turn_IP180();
    bool turn_IP90R();
    bool turn_IP90L();
    void stop_at(float position_mm);
    void stop_after(float distance_mm);
    void set_position(float position_mm);
    void adjust_forward_position(float delta_mm);
    bool wait_until_position(float position_mm, uint32_t timeout_ms);
    void wait_until_position(float position_mm);
    bool wait_until_distance(float distance_mm, uint32_t timeout_ms);
    void wait_until_distance(float distance_mm);

    // === Smooth (arc) turns ===
    // The motion half of mazerunner-core Mouse::turn_smooth(); the caller
    // (Mouse::smooth_turn) owns the wait between the two steps, exactly as
    // mazerunner's wait loop does:
    //   1. start_smooth_turn_entry(): roll at turn speed to the turn point,
    //      ENTRY mm before the turn cell centre.
    //   2. start_smooth_turn_arc(): the 90 deg arc, still rolling.
    // then drive smoothTurnExitMm() with a normal move().
    const SmoothTurnTuning& smoothTurnTuning() const { return smooth_turn_; }
    void set_smooth_turn_tuning(const SmoothTurnTuning& tuning) { smooth_turn_ = tuning; }
    void reset_smooth_turn_tuning() { smooth_turn_ = SmoothTurnTuning{}; }
    // Forward distance from now to the turn point, given the distance from now
    // to the turn cell centre. Negative means the turn point is already behind.
    float smoothTurnLeadMm(bool turn_left, float turn_centre_ahead_mm) const;
    // Exit leg length so the robot ends end_after_centre_mm past the turn cell
    // centre on the new heading. Clamped at 0: a negative leg would reverse.
    float smoothTurnExitMm(bool turn_left, float end_after_centre_mm) const;
    // Starts the lead-in; returns its length (position() reaches it at the
    // turn point). A turn point already behind gives 0: turn now.
    float start_smooth_turn_entry(bool turn_left, float turn_centre_ahead_mm);
    // heading_error_deg (+ = CCW) is added so the arc ends on the intended heading.
    void start_smooth_turn_arc(bool turn_left, float heading_error_deg = 0.0f);

    // === Control Loop ===
    // Caller must invoke update() exactly once per tick from a deterministic
    // scheduler pinned to LOOP_FREQUENCY_HZ.
    void update();

    // === Crash watchdog ===
    // update() feeds the detector every tick. On a trip it cuts the motors
    // itself (Drivetrain::stop(), then the cut hook) and holds them cut,
    // with the move frozen, until clear_crash(). The main loop reports it.
    // Reads the motor currents in amps; false if no reading this tick.
    using CurrentReader = bool (*)(void* context, float* left_amps, float* right_amps);
    CrashDetector& crashDetector() { return crash_; }
    // Wire these at start-up, before the control tick runs.
    void set_motor_current_reader(CurrentReader reader, void* context);
    // Called from the control tick on the trip, after the PWM is cut: where a
    // board puts its drivers to sleep (STM32 DRV8874 nSLEEP). Null = none.
    void set_crash_cut_hook(void (*hook)()) { crash_cut_hook_ = hook; }
    // Main loop only: drops the frozen move, so clearing never resumes it,
    // then releases the latch on the next tick.
    void clear_crash();

    // === Turn trace (diagnostics) ===
    // Recorded on the control core every TRACE_DECIMATION ticks while armed;
    // read back on Core 0 only after the motion has finished.
    struct TraceSample
    {
        float target_deg;    // rotation profile position, relative to turn start
        float yaw_deg;       // IMU robot angle (absolute)
        float rot_error_deg; // rotation PD accumulated error
        float rot_volts;     // rotation PD output
        float hold_err_deg;  // heading hold error
        float wall_degps;    // ToF wall steering fed to the rotation PD
        float hold_degps;    // IMU heading hold fed to the rotation PD
        float fwd_mmps;      // forward profile velocity
        float left_mmps;     // measured wheel velocity
        float right_mmps;
        float left_volts;
        float right_volts;
        bool  encoder_rotation; // rotation PD fed from encoders, not the IMU
        float left_tof_mm;      // filtered ToF distances as last set by Core 0
        float front_tof_mm;
        float right_tof_mm;
        float pos_mm; // forward profile position (the search's cell frame)
    };
    static constexpr uint16_t TRACE_CAPACITY = 1500; // ~108 KB RAM
    // Default sample period 4 ms: 2 ticks at 500 Hz, 4 at 1 kHz.
    static constexpr uint16_t TRACE_DECIMATION = LOOP_TICKS(0.004f);

    void               start_trace(uint16_t decimation = TRACE_DECIMATION);
    void               stop_trace();
    uint16_t           traceCount() const;
    uint16_t           traceDtMs() const;
    const TraceSample& traceSample(uint16_t index) const;

    /// Times a burst of main-loop commands overflowed the latch and was
    /// applied from the main loop with interrupts masked (diagnostics).
    uint32_t latchOverflows() const { return latch_overflows_; }

  private:
    /**
     * @brief One main-loop request to change control-tick state.
     *
     * Every public mutator queues one of these instead of writing the
     * profiles, controllers or drivetrain itself; update() applies the queue
     * first thing, in issue order, so the tick never sees half a command.
     */
    struct Command
    {
        enum class Op : uint8_t
        {
            ResetDriveSystem,
            ResetDriveControl,
            BeginSequence,
            EndSequence,
            StartMove, // a distance_mm, b top_mmps, c final_mmps, d accel_mmps2
            StartTurn, // a deg, b top_degps, c final_degps, d accel_degps2
            SmoothArc, // StartTurn, and the rotation PD reads encoder yaw
            SpinTurn,  // a deg, b omega_degps, c alpha_degps2, from rest
            ResetRotation,
            StopAt,      // a position_mm
            StopAfter,   // a distance_mm
            StopForward, // a accel_mmps2
            SetPosition, // a position_mm
            AdjustPosition,
            SetTargetVelocity,
            SetFinalVelocity,
            ExtendMove,
            ForwardSettled, // forward controller history cleared
            ForwardAbort,   // forward profile and controller cleared, motors off
            SetHeadingHold, // a target_yaw_deg (already wrapped)
            ClearHeadingHold,
            SetSteeringMode, // a mode
            SetLineSteering, // a adjustment_degps, b valid
            ClearLineSteering,
            Stop,
            DisableDrive,
            EmergencyStop,
            StartTrace, // a decimation
            StopTrace,
            ResetWallSteeringStats,
        };
        Op    op;
        float a, b, c, d;
    };

    /**
     * @brief The state commands set, as references, so one function applies a
     * command both to the live members (in the tick) and to copies (for
     * main-loop reads that must already see queued commands).
     */
    struct Commanded
    {
        Profile&                fwd;
        Profile&                rot;
        tof_wall::SteeringMode& steering_mode;
        bool&                   heading_hold_enabled;
        float&                  heading_hold_target_yaw_deg;
        float&                  line_steering_adjustment_degps;
        bool&                   line_steering_valid;
        bool&                   motion_sequence_active;
    };

    /// The live state with every queued command applied, read atomically.
    struct View
    {
        Profile                fwd;
        Profile                rot;
        tof_wall::SteeringMode steering_mode;
        bool                   heading_hold_enabled;
        float                  heading_hold_target_yaw_deg;
        float                  line_steering_adjustment_degps;
        bool                   line_steering_valid;
        bool                   motion_sequence_active;
    };

    static constexpr uint8_t LATCH_CAPACITY = 16;

    // Out of line: inlined into every mutator and getter they cost ~50 KB of flash.
    [[gnu::noinline]] void issue(Command::Op op, float a = 0.0f, float b = 0.0f, float c = 0.0f,
                                 float d = 0.0f);
    void                   applyPendingCommands();
    // Caller holds hal::irq::Lock. Encoder reads report 0 until the tick resets them.
    bool                          drivetrainResetQueued() const;
    [[gnu::noinline]] void        applyCommand(const Command& command);
    [[gnu::noinline]] static void applyCommanded(const Command& command, const Commanded& state);
    [[gnu::noinline]] View        view() const;

    void  runPositionControl();
    float wallSteeringAdjustment(float fwd_velocity_mmps, float rot_velocity_degps);
    float headingHoldAdjustment(float fwd_velocity_mmps, float rot_velocity_degps);
    void  resetControlHistory();
    void  clearWallSteeringStats();
    void  recordWallSteeringStats(float adjustment_degps, tof_wall::SteeringSource source);
    void  checkCrash();

    // Hardware
    Drivetrain* drivetrain_;
    IMU*        imu_;

    // Motion profiles. Names intentionally match UKMARS Motion::forward/rotation.
    Profile forward_;
    Profile rotation_;

    // Controllers — both at LOOP_FREQUENCY_HZ (single-rate, UKMARS-faithful).
    // Rotation PD consumes a per-tick yaw delta from imu_->robot_rot_change()
    // (cached_omega * dt), so the IMU's true 100 Hz sample rate is presented
    // as a held-flat per-tick increment instead of a 100 Hz gate. Each PID
    // instance still carries its own loop rate so KD * diff * loop_frequency
    // scales correctly per axis if the rates ever diverge.
    PID forward_controller_;
    PID rotation_controller_;

    // Per-wheel commanded velocity from previous tick — feeds ACC_FF as
    // `(v - prev_v) * LOOP_FREQUENCY_HZ`.
    float prev_left_cmd_vel_mmps_  = 0.0f;
    float prev_right_cmd_vel_mmps_ = 0.0f;
    bool  motion_sequence_active_  = false;

    SmoothTurnTuning smooth_turn_{};

    // ToFs are physically read and filtered outside Motion. Filtered values
    // feed wall detection/steering; raw values are kept only for diagnostics.
    float left_raw_wall_mm_  = 0.0f;
    float front_raw_wall_mm_ = 0.0f;
    float right_raw_wall_mm_ = 0.0f;
    float left_wall_mm_      = 0.0f;
    float front_wall_mm_     = 0.0f;
    float right_wall_mm_     = 0.0f;

    tof_wall::WallState      latest_wall_state_{};
    tof_wall::SteeringMode   steering_mode_ = tof_wall::SteeringMode::STEERING_OFF;
    float                    latest_steering_adjustment_degps_ = 0.0f;
    float                    side_error_prev_norm_             = 0.0f;
    bool                     side_error_prev_valid_            = false;
    float                    wall_steering_peak_degps_         = 0.0f;
    float                    wall_steering_sum_degps_          = 0.0f;
    uint32_t                 wall_steering_samples_            = 0;
    uint32_t                 wall_steering_source_changes_     = 0;
    uint32_t                 wall_steering_both_samples_       = 0;
    uint32_t                 wall_steering_single_samples_     = 0;
    tof_wall::SteeringSource wall_steering_prev_source_        = tof_wall::SteeringSource::None;
    bool                     wall_steering_prev_source_valid_  = false;
    float                    line_steering_adjustment_degps_   = 0.0f;
    bool                     line_steering_valid_              = false;
    float                    latest_line_rotation_volts_       = 0.0f;
    float                    latest_line_differential_volts_   = 0.0f;
    float                    latest_motor_differential_volts_  = 0.0f;

    bool  heading_hold_enabled_                 = false;
    float heading_hold_target_yaw_deg_          = 0.0f;
    float latest_heading_hold_error_deg_        = 0.0f;
    float latest_heading_hold_adjustment_degps_ = 0.0f;

    TraceSample       trace_[TRACE_CAPACITY] = {};
    volatile uint16_t trace_count_           = 0;
    uint16_t          trace_next_            = 0; // ring write index
    uint16_t          trace_tick_            = 0;
    uint16_t          trace_decimation_      = TRACE_DECIMATION;
    volatile bool     trace_active_          = false;

    // Main-loop commands waiting for the next tick, oldest first. Written by
    // the main loop and drained by update(), both under hal::irq::Lock.
    Command  latch_[LATCH_CAPACITY] = {};
    uint8_t  latch_count_           = 0;
    uint32_t latch_overflows_       = 0;

    // Non-zero while the rotation PD reads encoder yaw: UINT16_MAX during a
    // smooth-turn arc, then a holdover countdown in ticks once it ends.
    uint16_t encoder_rotation_ticks_ = 0;

    CrashDetector crash_;
    CurrentReader current_reader_  = nullptr;
    void*         current_context_ = nullptr;
    void (*crash_cut_hook_)()      = nullptr;
};

#endif
