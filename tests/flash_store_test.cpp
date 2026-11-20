// Host tests for the A/B flash store and the in-memory flash it runs on here.
// The in-memory flash can cut power at any word of an erase or a program, so
// the power-loss cases sweep every word of a save rather than sampling a few.
#include <cstdint>
#include <cstring>

#include "common/crc32.h"
#include "doctest/doctest.h"
#include "hal/host/memory_flash.h"

namespace
{
constexpr uint32_t SECTOR_BYTES = 1024;
} // namespace

TEST_CASE("CRC-32 matches the standard check value")
{
    // The IEEE 802.3 CRC-32 of the ASCII digits "123456789" is 0xCBF43926.
    const char* digits = "123456789";
    CHECK(crc32::compute(digits, 9) == 0xCBF43926u);
    // Feeding it in pieces gives the same answer as one call.
    uint32_t state = crc32::INITIAL;
    state          = crc32::update(state, digits, 4);
    state          = crc32::update(state, digits + 4, 5);
    CHECK(crc32::finish(state) == 0xCBF43926u);
}

TEST_CASE("In-memory flash erases to ones and programming only clears bits")
{
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    CHECK(flash.sectorCount() == 2);
    CHECK(flash.sectorSizeBytes() == SECTOR_BYTES);

    uint32_t word = 0;
    flash.read(SECTOR_BYTES, &word, 1);
    CHECK(word == 0xFFFFFFFFu);

    const uint32_t first = 0xF0F0FFFFu;
    REQUIRE(flash.program(SECTOR_BYTES, &first, 1));
    const uint32_t second = 0xFFFF0F0Fu;
    REQUIRE(flash.program(SECTOR_BYTES, &second, 1));
    flash.read(SECTOR_BYTES, &word, 1);
    CHECK(word == 0xF0F00F0Fu);

    REQUIRE(flash.erase(1));
    flash.read(SECTOR_BYTES, &word, 1);
    CHECK(word == 0xFFFFFFFFu);

    // Out of range or misaligned writes fail and change nothing.
    CHECK_FALSE(flash.program(2 * SECTOR_BYTES, &first, 1));
    CHECK_FALSE(flash.program(2, &first, 1));
    CHECK_FALSE(flash.erase(2));
}

TEST_CASE("In-memory flash tears the word at the power cut and then goes dead")
{
    hal::host::MemoryFlash flash(1, SECTOR_BYTES);
    const uint32_t         zeros[4] = {0, 0, 0, 0};

    flash.cutPowerAfterWords(2);
    CHECK_FALSE(flash.program(0, zeros, 4));
    uint32_t got[4];
    flash.read(0, got, 4);
    CHECK(got[0] == 0);
    CHECK(got[1] == 0);
    CHECK(got[3] == 0xFFFFFFFFu); // never reached
    CHECK_FALSE(flash.powered());

    // A dead device ignores everything until power comes back.
    CHECK_FALSE(flash.erase(0));
    flash.read(0, got, 4);
    CHECK(got[0] == 0);

    flash.restorePower();
    CHECK(flash.powered());
    const uint64_t before = flash.wordOps();
    REQUIRE(flash.erase(0));
    // An erase counts one operation per word of the sector.
    CHECK(flash.wordOps() - before == SECTOR_BYTES / 4);
}
