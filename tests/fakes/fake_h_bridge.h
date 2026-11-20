#ifndef TESTS_FAKES_FAKE_H_BRIDGE_H
#define TESTS_FAKES_FAKE_H_BRIDGE_H

#include <cstdint>

#include "hal/h_bridge.h"

namespace fake
{
/// Two motor drivers' pins as a test sees them: the last input fractions,
/// the nSLEEP level, and IPROPI readings the test sets.
class HBridge final : public hal::HBridgePort
{
  public:
    struct Inputs
    {
        float in1_high = 0.0f;
        float in2_high = 0.0f;
    };

    void setInputs(int motor, float in1_high, float in2_high) override
    {
        inputs[motor] = {in1_high, in2_high};
    }
    void     setAwake(bool on) override { awake = on; }
    uint16_t senseCounts(int motor) override { return counts[motor]; }

    Inputs   inputs[2] = {};
    bool     awake     = true; // unknown at power-up until the bridge drives it
    uint16_t counts[2] = {0, 0};
};
} // namespace fake

#endif // TESTS_FAKES_FAKE_H_BRIDGE_H
