/**
 * @file breadcrumb.h
 * @brief Reset-surviving record so the next boot can explain an electrical fault.
 *
 * The Pico firmware has no watchdog, so a software crash hangs rather than
 * restarts, and a restart means power was lost or sagged below brownout. The
 * STM32 also resets on a brownout (BOR) or a watchdog timeout (a hung main
 * loop). Either way the reset-cause register alone cannot say how far the rail
 * fell or what the robot was doing, and a warning logged at that moment dies
 * unsent in the Bluetooth queue. So we keep those few facts in SRAM instead
 * (HAL_NOINIT), and resetReport() (common/reset_report.h) explains them.
 *
 * RP2040 and STM32F405 SRAM keep their contents through a brownout, watchdog
 * or software reset; only a power-off long enough for it to decay destroys it.
 * A magic word tells the two apart, so a genuine cold boot is never reported as
 * a brownout. On the host and in the simulator every call is a no-op.
 */
#ifndef COMMON_BREADCRUMB_H
#define COMMON_BREADCRUMB_H

#include <cstdint>

class Breadcrumb
{
  public:
    /// What the previous session recorded before it died.
    struct Snapshot
    {
        bool     valid;        // false when SRAM decayed, i.e. a true cold boot
        uint32_t uptime_ms;    // uptime at the last log line
        float    voltage_min;  // lowest instantaneous reading; <= 0 means none taken
        float    voltage_last; // most recent instantaneous reading
        char     last_log[48]; // last log line, i.e. what it was doing
    };

    /**
     * @brief Copies the previous session's record aside, then re-arms for this one.
     *
     * Must run before anything logs or samples the battery, otherwise this
     * session overwrites the history we are trying to read.
     */
    static void begin();

    /// Tracks the running minimum. Cheap enough for the 500 Hz path (two compares).
    static void noteVoltage(float volts);

    /// Records the activity text and the uptime at which it happened.
    static void noteActivity(const char* text);

    /**
     * @brief Prints the plain-English verdict plus a [DIAG] line of raw register bits.
     *
     * Call once Bluetooth is up so the verdict actually reaches the operator.
     */
    static void report();

    static const Snapshot& previous();

  private:
    static Snapshot previous_;
};

#endif // COMMON_BREADCRUMB_H
