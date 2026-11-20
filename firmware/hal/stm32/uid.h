#ifndef HAL_STM32_UID_H
#define HAL_STM32_UID_H

#include <cstdint>

namespace hal::stm32
{
/// The chip's 96-bit unique ID, factory-programmed at 0x1FFF7A10 (RM0090
/// 39.1), lowest address first.
struct ChipUid
{
    uint32_t words[3];
};

ChipUid chipUid();
} // namespace hal::stm32

#endif // HAL_STM32_UID_H
