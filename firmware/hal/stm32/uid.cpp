#include "hal/stm32/uid.h"

namespace hal::stm32
{
ChipUid chipUid()
{
    constexpr uint32_t UID_BASE = 0x1FFF7A10u; // system memory, read-only
    const auto*        id       = reinterpret_cast<const volatile uint32_t*>(UID_BASE);
    return {{id[0], id[1], id[2]}};
}
} // namespace hal::stm32
