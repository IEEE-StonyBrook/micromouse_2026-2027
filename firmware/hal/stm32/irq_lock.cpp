// hal::irq::Lock on the STM32: masks every maskable interrupt (PRIMASK) on
// the one core and restores the previous mask, so it nests and is safe inside
// a handler.
#include <libopencm3/cm3/cortex.h>

#include "hal/irq.h"

namespace hal::irq
{
Lock::Lock() : saved_(cm_mask_interrupts(1))
{
}

Lock::~Lock()
{
    cm_mask_interrupts(saved_);
}
} // namespace hal::irq
