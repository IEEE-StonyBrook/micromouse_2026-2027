#include "hal/system.h"

namespace hal::system
{
ResetCause resetCause()
{
    // A host process always starts cold.
    ResetCause cause{};
    cause.power_on = true;
    return cause;
}
} // namespace hal::system
