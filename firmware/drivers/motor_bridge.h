#ifndef DRIVERS_MOTOR_BRIDGE_H
#define DRIVERS_MOTOR_BRIDGE_H

#include <atomic>
#include <cstdint>

#include "config/motion.h"
#include "config/motor_driver.h"
#include "hal/h_bridge.h"

/**
 * @brief Two DRV8874 motor drivers in PWM (IN/IN) mode: slow-decay drive,
 *        brake, sleep after idle, wake without losing a command, and motor
 *        current from IPROPI.
 *
 * Drive (DRV8874 datasheet, Table 4): IN1 IN2 = 10 forward, 01 reverse,
 * 11 brake (both low sides on), 00 coast. Duty d > 0 holds IN1 high and
 * pulses IN2 low for d of each period, so the bridge alternates between
 * driving and braking (slow decay); d < 0 mirrors it; d = 0 is a brake.
 *
 * Sleep: braked with no command for MOTOR_IDLE_SLEEP_TIME_S, or on sleep(),
 * nSLEEP goes low and the outputs go Hi-Z. A non-zero command, or wake(),
 * raises nSLEEP; the driver ignores its inputs for tWAKE, so the bridge
 * holds the command and applies it on the first tick after tWAKE has surely
 * passed. The latest command is always kept, so none is lost.
 *
 * Current: IPROPI sources AIPROPI times the low-side current, which flows
 * in both the drive and the brake halves of a slow-decay period, so it is
 * continuous, not chopped by the PWM. amps() is valid only while awake.
 * The ADC's zero offset is learned while the drivers sleep, when IPROPI
 * carries no current; until the first sleep has settled (one tSLEEP after
 * boot) the offset is 0, so amps read high by the ADC's offset. IPROPI
 * cannot tell direction: amps are magnitudes.
 *
 * Threading: update() belongs to the control tick, once per tick.
 * set_duty(), wake() and sleep() may be called from the tick or the main
 * loop; they take hal::irq::Lock. sleep() acts at once, so a caller that
 * needs the drivers off (a flash save, the crash watchdog) has them off
 * when it returns.
 */
class MotorBridge
{
  public:
    enum class State : uint8_t
    {
        kAsleep, // nSLEEP low, outputs Hi-Z
        kWaking, // nSLEEP high, waiting out tWAKE
        kAwake,  // driving or braking
    };

    /// amps_per_count converts IPROPI ADC counts into motor amps; see
    /// ipropiAmpsPerCount(). Starts asleep; drives nSLEEP low.
    MotorBridge(hal::HBridgePort& port, float amps_per_count);

    /// Duty in [-1, 1] per motor, positive forward; 0 brakes.
    void set_duty(float left, float right);

    /// Wakes the drivers with a zero (brake) command, e.g. on arming, so
    /// the first move does not wait out tWAKE.
    void wake();

    /// Puts the drivers to sleep now and zeroes the command. A later
    /// non-zero command wakes them again.
    void sleep();

    /// Advances wake, idle and current sensing by one control tick.
    void update();

    State state() const { return static_cast<State>(state_.load()); }
    /// nSLEEP is high (waking or awake).
    bool awake() const { return state() != State::kAsleep; }

    /// Motor current magnitude in amps at the last update(); 0 unless awake.
    float amps(int motor) const { return amps_[motor].load(); }

    /// Motion::CurrentReader: both motors' amps; false unless awake.
    static bool readAmps(void* bridge, float* left_amps, float* right_amps);

    // nSLEEP rises inside one update(); the command lands WAKE_TICKS updates
    // later, the first tick strictly past tWAKE (2 ms at both 500 Hz and
    // 1 kHz), so tick jitter cannot cut it short.
    static constexpr int TICK_US    = static_cast<int>(1e6f / LOOP_FREQUENCY_HZ + 0.5f);
    static constexpr int WAKE_TICKS = DRV8874_WAKE_TIME_US / TICK_US + 1;
    // The offset is learned only once the driver has surely gone to sleep.
    static constexpr int SLEEP_SETTLE_TICKS = DRV8874_SLEEP_TIME_US / TICK_US + 1;
    static constexpr int IDLE_SLEEP_TICKS   = LOOP_TICKS(MOTOR_IDLE_SLEEP_TIME_S);

  private:
    void driveOutputs();
    void sleepNow();

    hal::HBridgePort& port_;
    const float       amps_per_count_;

    // Written under hal::irq::Lock or from the tick.
    float duty_[2]      = {0.0f, 0.0f};
    bool  wake_request_ = false;

    std::atomic<uint8_t> state_{static_cast<uint8_t>(State::kAsleep)};
    std::atomic<float>   amps_[2] = {0.0f, 0.0f};

    // Control-tick state; sleep() also resets asleep_ticks_, under the lock.
    int   wake_ticks_       = 0;
    int   idle_ticks_       = 0;
    int   asleep_ticks_     = 0;
    float offset_counts_[2] = {0.0f, 0.0f};
};

/// Amps per IPROPI ADC count: a count is adc_full_scale_volts / adc_counts
/// volts across r_ipropi_ohms, and IPROPI carries AIPROPI per motor amp.
constexpr float ipropiAmpsPerCount(float adc_full_scale_volts, float adc_counts,
                                   float r_ipropi_ohms, float aipropi_ua_per_a)
{
    return adc_full_scale_volts / adc_counts / (r_ipropi_ohms * aipropi_ua_per_a * 1e-6f);
}

#endif // DRIVERS_MOTOR_BRIDGE_H
