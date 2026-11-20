#include "hal/system.h"

namespace hal::system
{
ResetCause resetCause()
{
    // A host process always starts cold.
    return ResetCause{0, true, false, false, false};
}
} // namespace hal::system
