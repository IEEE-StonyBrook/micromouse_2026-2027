// Host tests for the STM32F405 flash sector map the flash store relies on.
// Sector boundaries are from RM0090 table 5 (flash module organisation,
// STM32F40x/41x): sectors 0-3 are 16 KB, sector 4 64 KB, sectors 5-11 128 KB.
#include "doctest/doctest.h"
#include "hal/stm32/flash_layout.h"

using namespace hal::stm32;

TEST_CASE("Flash addresses map to RM0090's sector numbers")
{
    CHECK(flashSector(0x08000000u).number == 0);
    CHECK(flashSector(0x08004000u).number == 1);
    CHECK(flashSector(0x08007FFCu).number == 1);
    CHECK(flashSector(0x08008000u).number == 2);
    CHECK(flashSector(0x0800C000u).number == 3);
    CHECK(flashSector(0x08010000u).number == 4);
    CHECK(flashSector(0x08010000u).size_bytes == 64 * 1024);
    CHECK(flashSector(0x08020000u).number == 5);
    CHECK(flashSector(0x080E0000u).number == 11);
    CHECK(flashSector(0x080E0000u).size_bytes == 128 * 1024);
    CHECK_FALSE(flashSector(0x08100000u).valid);
    CHECK_FALSE(flashSector(0x07FFFFFCu).valid);
    // Mid-sector addresses are not a sector start.
    CHECK(flashSector(0x08004000u).starts_here);
    CHECK_FALSE(flashSector(0x08004004u).starts_here);
}

TEST_CASE("The linker's store region is two whole equal sectors, 1 and 2")
{
    const StoreSectors store = storeSectors(0x08004000u, 0x0800C000u);
    REQUIRE(store.valid);
    CHECK(store.first_sector == 1);
    CHECK(store.count == 2);
    CHECK(store.sector_bytes == 16 * 1024);
}

TEST_CASE("A store region that splits a sector or mixes sizes is rejected")
{
    CHECK_FALSE(storeSectors(0x08004000u, 0x0800A000u).valid); // ends mid-sector 2
    CHECK_FALSE(storeSectors(0x08004004u, 0x0800C000u).valid); // starts mid-sector 1
    CHECK_FALSE(storeSectors(0x0800C000u, 0x08020000u).valid); // 16 KB sector 3 + 64 KB sector 4
    CHECK_FALSE(storeSectors(0x08004000u, 0x08004000u).valid); // empty
}
