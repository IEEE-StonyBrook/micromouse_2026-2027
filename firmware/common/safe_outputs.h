#ifndef COMMON_SAFE_OUTPUTS_H
#define COMMON_SAFE_OUTPUTS_H

#include <cstddef>

#include "config/board_feather.h" // board::SafeOutput
#include "hal/output_port.h"

/**
 * @brief The power-on order: every actuator output (motor driver inputs and
 *        nSLEEP, IR emitters, fan) is driven to its off level before any other
 *        start-up step, and the board does not arm until that has happened.
 *
 * Between reset and holdOff() the pins are high-impedance, so the board's
 * pull-downs (docs/rev1-bringup-checklist.md) are what keep the actuators off
 * for those few microseconds. The driver that later takes a pin over (e.g.
 * hal/stm32/h_bridge.h) must keep it off until the board arms.
 *
 * tests/safe_outputs_test.cpp checks the order on the host.
 */
class SafeOutputs
{
  public:
    /// No GPIO access here, so it may be a static object.
    SafeOutputs(hal::OutputPort& port, const board::SafeOutput* outputs, std::size_t count)
        : port_(port), outputs_(outputs), count_(count)
    {
    }

    /// Sets each output's off level, then makes it an output, so no pin
    /// glitches on. Disarms.
    void holdOff();

    /// Arms; refused (false) until holdOff() has run.
    bool arm();
    void disarm() { armed_ = false; }
    bool armed() const { return armed_; }

  private:
    hal::OutputPort&         port_;
    const board::SafeOutput* outputs_;
    std::size_t              count_;
    bool                     held_  = false;
    bool                     armed_ = false;
};

/// Boot: the safe outputs off first, then `startUp` (clock setup and the
/// rest). Returns what `startUp` returns.
template <typename StartUp> auto powerOn(SafeOutputs& safe, StartUp&& startUp)
{
    safe.holdOff();
    return startUp();
}

#endif // COMMON_SAFE_OUTPUTS_H
