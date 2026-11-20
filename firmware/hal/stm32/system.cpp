#include "hal/system.h"

#include <libopencm3/stm32/rcc.h>

#include "hal/stm32/reset_flags.h"

static_assert(hal::stm32::csr::BORRSTF == RCC_CSR_BORRSTF &&
                  hal::stm32::csr::PINRSTF == RCC_CSR_PINRSTF &&
                  hal::stm32::csr::PORRSTF == RCC_CSR_PORRSTF &&
                  hal::stm32::csr::SFTRSTF == RCC_CSR_SFTRSTF &&
                  hal::stm32::csr::IWDGRSTF == RCC_CSR_IWDGRSTF &&
                  hal::stm32::csr::WWDGRSTF == RCC_CSR_WWDGRSTF &&
                  static_cast<uint32_t>(hal::stm32::csr::LPWRRSTF) ==
                      static_cast<uint32_t>(RCC_CSR_LPWRRSTF),
              "reset_flags.h disagrees with libopencm3's RCC_CSR bits");

namespace hal::system
{
ResetCause resetCause()
{
    // The flags are sticky until RMVF clears them, so read them once and
    // clear them: otherwise every later reset would still show this one.
    static bool       captured = false;
    static ResetCause cause{};
    if (!captured)
    {
        cause = hal::stm32::decodeResetFlags(RCC_CSR);
        RCC_CSR |= RCC_CSR_RMVF;
        captured = true;
    }
    return cause;
}
} // namespace hal::system
