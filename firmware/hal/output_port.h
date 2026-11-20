#ifndef HAL_OUTPUT_PORT_H
#define HAL_OUTPUT_PORT_H

#include "config/board_feather.h" // board::Pin

namespace hal
{
/**
 * @brief Plain GPIO outputs, split so a pin's level can be set before the pin
 *        starts driving it: no edge appears when the pin leaves reset.
 *
 * SafeOutputs (common/safe_outputs.h) uses it to hold the actuators off at
 * power-on. Backends: hal/stm32/output_port.h; tests/fakes/fake_output_port.h.
 */
class OutputPort
{
  public:
    /// Sets the level the pin drives once it is an output (the output data
    /// register); a pin still in reset state stays high-impedance.
    virtual void setLevel(board::Pin pin, bool high) = 0;

    /// Makes the pin a push-pull output at the level last set.
    virtual void makeOutput(board::Pin pin) = 0;

  protected:
    ~OutputPort() = default;
};
} // namespace hal

#endif // HAL_OUTPUT_PORT_H
