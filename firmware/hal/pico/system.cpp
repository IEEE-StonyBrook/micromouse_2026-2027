#include "hal/system.h"

#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/watchdog.h"

namespace hal::system
{
ResetCause resetCause()
{
    const uint32_t bits = vreg_and_chip_reset_hw->chip_reset;

    ResetCause cause{};
    cause.raw      = bits;
    cause.power_on = (bits & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) != 0;
    cause.run_pin  = (bits & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) != 0;
    cause.debugger = (bits & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) != 0;
    cause.software = watchdog_caused_reboot();
    return cause;
}
} // namespace hal::system
