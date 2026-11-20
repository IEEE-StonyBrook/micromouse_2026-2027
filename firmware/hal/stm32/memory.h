#ifndef HAL_STM32_MEMORY_H
#define HAL_STM32_MEMORY_H

#include <cstdint>

namespace hal::stm32
{
/// Peak heap and stack use since reset, in bytes, against their limits.
struct MemoryUsage
{
    uint32_t heap_peak_bytes;   ///< Highest program break reached through _sbrk
    uint32_t heap_limit_bytes;  ///< SRAM between the end of .noinit and the reserved stack
    uint32_t stack_peak_bytes;  ///< Deepest stack use seen in the reserved stack region
    uint32_t stack_limit_bytes; ///< Reserved stack size (_min_stack_size in the linker script)
};

/// Scans the painted stack, so it costs up to stack_limit_bytes / 4 reads.
MemoryUsage memoryUsage();
} // namespace hal::stm32

#endif // HAL_STM32_MEMORY_H
