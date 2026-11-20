#ifndef CONTROL_CRASH_DETECTOR_H
#define CONTROL_CRASH_DETECTOR_H

#include <atomic>
#include <cstdint>

#include "config/motion.h"

/// Why the crash watchdog tripped. The value is the LED cause code.
enum class CrashCause : uint8_t
{
    kNone       = 0,
    kSaturation = 1, // a wheel's command pinned at the voltage limit
    kStall      = 2, // stall current at near-zero wheel speed
    kRunaway    = 3, // forward position error growing without bound
};

/// Short upper-case name for reports ("SATURATION"); no allocation.
const char* crashCauseName(CrashCause cause);

/// One control tick's view of the drive, in the units Motion works in.
struct CrashInputs
{
    float left_volts    = 0.0f; // commanded this tick, after the MAX_VOLTAGE clamp
    float right_volts   = 0.0f;
    float battery_volts = 0.0f;
    float left_mmps     = 0.0f; // measured wheel speeds
    float right_mmps    = 0.0f;
    float fwd_error_mm  = 0.0f; // forward PD accumulated position error
    // Motor current, when the board can sense it (STM32 IPROPI). The Pico
    // cannot, so it leaves has_current false and the stall cause is off.
    bool  has_current = false;
    float left_amps   = 0.0f;
    float right_amps  = 0.0f;
};

/**
 * @brief The crash watchdog's trip logic: platform-neutral, no I/O.
 *
 * update() runs once per control tick from Motion::update() and returns true
 * on the tick it trips. A trip latches until clear(). Every window is a time
 * (CRASH_*_TIME_S, config/motion.h) turned into ticks with LOOP_TICKS, so it
 * behaves the same at 500 Hz and 1 kHz.
 *
 * Threading: update() belongs to the control tick. set_enabled(),
 * set_inhibit() and clear() are for the main loop; they only store flags
 * that the next update() acts on, so neither side sees a half-updated state.
 */
class CrashDetector
{
  public:
    explicit CrashDetector(bool enabled = CRASH_WATCHDOG_DEFAULT_ENABLED);

    bool update(const CrashInputs& in);

    bool       tripped() const { return tripped_.load(); }
    CrashCause cause() const { return static_cast<CrashCause>(cause_.load()); }
    // Trips since boot; the main loop reports each new one once.
    uint32_t tripCount() const { return trip_count_.load(); }

    // Releases the latch on the next update().
    void clear() { clear_request_.store(true); }

    bool enabled() const { return enabled_.load(); }
    void set_enabled(bool enabled) { enabled_.store(enabled); }
    // Suppresses every cause while set, e.g. backing into a wall on purpose;
    // each window starts afresh once it lifts.
    bool inhibited() const { return inhibit_.load(); }
    void set_inhibit(bool inhibit) { inhibit_.store(inhibit); }

  private:
    void resetWindows();
    void trip(CrashCause cause);

    std::atomic<bool>     enabled_;
    std::atomic<bool>     inhibit_{false};
    std::atomic<bool>     clear_request_{false};
    std::atomic<bool>     tripped_{false};
    std::atomic<uint8_t>  cause_{static_cast<uint8_t>(CrashCause::kNone)};
    std::atomic<uint32_t> trip_count_{0};

    // Control-tick state only.
    int   saturation_ticks_ = 0;
    int   stall_ticks_      = 0;
    int   runaway_ticks_    = 0;
    float runaway_start_mm_ = 0.0f; // |error| when the current window began
};

#endif // CONTROL_CRASH_DETECTOR_H
