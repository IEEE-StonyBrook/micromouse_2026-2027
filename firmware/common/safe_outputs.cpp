#include "common/safe_outputs.h"

void SafeOutputs::holdOff()
{
    armed_ = false;
    for (std::size_t i = 0; i < count_; ++i)
        port_.setLevel(outputs_[i].pin, outputs_[i].off_high);
    for (std::size_t i = 0; i < count_; ++i)
        port_.makeOutput(outputs_[i].pin);
    held_ = true;
}

bool SafeOutputs::arm()
{
    armed_ = held_;
    return armed_;
}
