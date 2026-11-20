#ifndef HAL_STM32_FLASH_LAYOUT_H
#define HAL_STM32_FLASH_LAYOUT_H

#include <cstdint>

/**
 * @file flash_layout.h
 * @brief The STM32F405's flash sector map (RM0090 table 5), as plain
 *        arithmetic with no libopencm3, so host tests can check it.
 *
 * Main memory starts at 0x08000000: sectors 0-3 are 16 KB, sector 4 is
 * 64 KB and sectors 5-11 are 128 KB, 1 MB in all.
 */
namespace hal::stm32
{
constexpr uint32_t FLASH_BASE_ADDR = 0x08000000u;
constexpr uint32_t FLASH_END_ADDR  = 0x08100000u;

struct FlashSector
{
    bool     valid;
    uint8_t  number;
    uint32_t start;
    uint32_t size_bytes;
    bool     starts_here; ///< The address is the sector's first byte
};

constexpr FlashSector flashSector(uint32_t addr)
{
    if (addr < FLASH_BASE_ADDR || addr >= FLASH_END_ADDR)
        return {false, 0, 0, 0, false};
    const uint32_t offset = addr - FLASH_BASE_ADDR;
    uint8_t        number = 0;
    uint32_t       start  = 0;
    uint32_t       size   = 0;
    if (offset < 0x10000u) // sectors 0-3
    {
        number = static_cast<uint8_t>(offset / 0x4000u);
        start  = number * 0x4000u;
        size   = 0x4000u;
    }
    else if (offset < 0x20000u) // sector 4
    {
        number = 4;
        start  = 0x10000u;
        size   = 0x10000u;
    }
    else // sectors 5-11
    {
        number = static_cast<uint8_t>(5 + (offset - 0x20000u) / 0x20000u);
        start  = 0x20000u + (number - 5u) * 0x20000u;
        size   = 0x20000u;
    }
    return {true, number, FLASH_BASE_ADDR + start, size, offset == start};
}

/// The flash store's sectors, from the linker script's region bounds.
struct StoreSectors
{
    bool     valid;
    uint8_t  first_sector;
    uint32_t count;
    uint32_t sector_bytes;
};

/// Valid only when [start, end) is a run of whole sectors of one size, which
/// is what an A/B store of equal slots needs.
constexpr StoreSectors storeSectors(uint32_t start, uint32_t end)
{
    const FlashSector first = flashSector(start);
    if (!first.valid || !first.starts_here || end <= start)
        return {false, 0, 0, 0};
    uint32_t count = 0;
    for (uint32_t addr = start; addr < end; addr += first.size_bytes)
    {
        const FlashSector s = flashSector(addr);
        if (!s.valid || !s.starts_here || s.size_bytes != first.size_bytes ||
            addr + s.size_bytes > end)
            return {false, 0, 0, 0};
        ++count;
    }
    return {true, first.number, count, first.size_bytes};
}
} // namespace hal::stm32

#endif // HAL_STM32_FLASH_LAYOUT_H
