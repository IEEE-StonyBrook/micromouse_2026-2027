#ifndef TESTS_FAKES_FAKE_OUTPUT_PORT_H
#define TESTS_FAKES_FAKE_OUTPUT_PORT_H

#include <string>
#include <vector>

#include "hal/output_port.h"

namespace fake
{
/// GPIO outputs as a test sees them: every call, in order, in a log the test
/// can share with other steps to check what happened first.
class OutputPort final : public hal::OutputPort
{
  public:
    explicit OutputPort(std::vector<std::string>& log) : log_(log) {}

    void setLevel(board::Pin pin, bool high) override
    {
        log_.push_back("level " + name(pin) + (high ? " high" : " low"));
    }
    void makeOutput(board::Pin pin) override { log_.push_back("output " + name(pin)); }

    static std::string name(board::Pin pin)
    {
        return std::string(1, pin.port) + std::to_string(pin.number);
    }

  private:
    std::vector<std::string>& log_;
};
} // namespace fake

#endif // TESTS_FAKES_FAKE_OUTPUT_PORT_H
