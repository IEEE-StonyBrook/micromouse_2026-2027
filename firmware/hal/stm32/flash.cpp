#include "hal/stm32/flash.h"

#include <libopencm3/stm32/flash.h>

#include "hal/stm32/watchdog.h"

// Bounds of the store region, from hal/stm32/stm32f405.ld.
extern "C" const uint32_t _flash_store_start;
extern "C" const uint32_t _flash_store_end;

namespace hal::stm32
{
namespace
{
constexpr uint32_t SR_ERRORS =
    FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR;

void clearErrors()
{
    FLASH_SR = SR_ERRORS;
} // write 1 to clear

bool noErrors()
{
    return (FLASH_SR & SR_ERRORS) == 0;
}

/// The data cache may still hold words read before an erase or program
/// (RM0090 3.5.2): flush it so the store's read-back sees the cells.
void flushDataCache()
{
    flash_dcache_disable();
    flash_dcache_reset();
    flash_dcache_enable();
}
} // namespace

bool StoreFlash::init()
{
    base_   = reinterpret_cast<uint32_t>(&_flash_store_start);
    layout_ = storeSectors(base_, reinterpret_cast<uint32_t>(&_flash_store_end));
    return layout_.valid;
}

bool StoreFlash::erase(uint32_t sector)
{
    if (!layout_.valid || sector >= layout_.count)
        return false;
    // The erase stalls the CPU for up to 500 ms (FLASH_SECTOR_ERASE_MAX_MS);
    // a fresh feed leaves the watchdog its whole timeout, at least 3 s.
    watchdog::feed();
    flash_unlock();
    clearErrors();
    flash_erase_sector(static_cast<uint8_t>(layout_.first_sector + sector), FLASH_CR_PROGRAM_X32);
    const bool ok = noErrors();
    flash_lock();
    flushDataCache();
    return ok;
}

bool StoreFlash::program(uint32_t offset_bytes, const uint32_t* words, std::size_t count)
{
    const uint32_t size = layout_.count * layout_.sector_bytes;
    if (!layout_.valid || offset_bytes % 4 != 0 || offset_bytes > size ||
        count > (size - offset_bytes) / 4)
        return false;
    flash_unlock();
    clearErrors();
    bool ok = true;
    for (std::size_t i = 0; i < count && ok; ++i)
    {
        flash_program_word(base_ + offset_bytes + 4 * i, words[i]);
        ok = noErrors();
    }
    flash_lock();
    flushDataCache();
    return ok;
}

void StoreFlash::read(uint32_t offset_bytes, uint32_t* words, std::size_t count) const
{
    const auto* cells = reinterpret_cast<const volatile uint32_t*>(base_ + offset_bytes);
    for (std::size_t i = 0; i < count; ++i)
        words[i] = cells[i];
}
} // namespace hal::stm32
