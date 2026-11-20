// Host tests for identifying a board by its chip UID: a board loads the
// profile it saved under its own UID, and an unknown board must pick and save
// one before it may arm.
#include <cmath>
#include <cstdint>
#include <cstring>

#include "common/board_identity.h"
#include "common/flash_store.h"
#include "doctest/doctest.h"
#include "hal/host/memory_flash.h"

namespace
{
constexpr uint32_t SECTOR_BYTES = 16 * 1024;
constexpr uint32_t UID_A[3]     = {0x00470031u, 0x3438510Bu, 0x31323637u};
constexpr uint32_t UID_B[3]     = {0x00470031u, 0x3438510Bu, 0x31323638u};

class Guard : public SaveGuard
{
  public:
    SaveBlocker prepareForSave() override { return blocker; }
    SaveBlocker blocker = SaveBlocker::kNone;
};

/// One power-up of a board: a store and an identity built fresh from flash.
struct Boot
{
    Boot(hal::FlashDevice& flash, const uint32_t (&uid)[3])
        : store(flash, guard, SETTINGS_VERSION, uidHash(uid)), identity(store)
    {
        status = identity.boot();
    }
    Guard         guard;
    FlashStore    store;
    BoardIdentity identity;
    LoadStatus    status;
};

std::size_t presetNamed(const char* name)
{
    for (std::size_t i = 0; i < board::PROFILE_PRESET_COUNT; ++i)
        if (std::strcmp(board::PROFILE_PRESETS[i].name, name) == 0)
            return i;
    FAIL("no preset named " << name);
    return 0;
}
} // namespace

TEST_CASE("UID hashes tell boards apart and depend on word order")
{
    CHECK(uidHash(UID_A) == uidHash(UID_A));
    CHECK(uidHash(UID_A) != uidHash(UID_B));
    const uint32_t swapped[3] = {UID_A[1], UID_A[0], UID_A[2]};
    CHECK(uidHash(swapped) != uidHash(UID_A));
}

TEST_CASE("The presets include Mouse 1 with the fan and Mouse 2 without")
{
    const board::BoardProfile& m1 = board::PROFILE_PRESETS[presetNamed("mouse1")];
    const board::BoardProfile& m2 = board::PROFILE_PRESETS[presetNamed("mouse2")];
    CHECK(m1.has_fan == 1);
    CHECK(m2.has_fan == 0);
    for (std::size_t i = 0; i < board::PROFILE_PRESET_COUNT; ++i)
        CHECK(profileValid(board::PROFILE_PRESETS[i]));
}

TEST_CASE("An unknown board needs a profile and refuses to arm until one is saved")
{
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    {
        Boot first(flash, UID_A);
        CHECK(first.status == LoadStatus::kEmpty);
        CHECK_FALSE(first.identity.identified());
        CHECK_FALSE(first.identity.armAllowed());

        const SaveResult saved = first.identity.pick(presetNamed("mouse1"));
        REQUIRE(saved.status == SaveStatus::kOk);
        CHECK(first.identity.identified());
        CHECK(first.identity.armAllowed());
        CHECK(std::strcmp(first.identity.profile().name, "mouse1") == 0);
    }
    // Next power-up: the board recognises itself.
    Boot again(flash, UID_A);
    CHECK(again.status == LoadStatus::kOk);
    CHECK(again.identity.identified());
    CHECK(again.identity.armAllowed());
    CHECK(std::strcmp(again.identity.profile().name, "mouse1") == 0);
    CHECK(again.identity.profile().has_fan == 1);
}

TEST_CASE("Each board loads its own profile by UID")
{
    hal::host::MemoryFlash flash_a(2, SECTOR_BYTES);
    hal::host::MemoryFlash flash_b(2, SECTOR_BYTES);
    REQUIRE(Boot(flash_a, UID_A).identity.pick(presetNamed("mouse1")).status == SaveStatus::kOk);
    REQUIRE(Boot(flash_b, UID_B).identity.pick(presetNamed("mouse2")).status == SaveStatus::kOk);

    Boot a(flash_a, UID_A);
    Boot b(flash_b, UID_B);
    CHECK(std::strcmp(a.identity.profile().name, "mouse1") == 0);
    CHECK(std::strcmp(b.identity.profile().name, "mouse2") == 0);
    CHECK(b.identity.profile().has_fan == 0);
}

TEST_CASE("A profile saved by another chip is not trusted")
{
    // The store region survives a reflash, and a debugger can copy it from
    // one board to the other: board B must not take on board A's profile.
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    REQUIRE(Boot(flash, UID_A).identity.pick(presetNamed("mouse1")).status == SaveStatus::kOk);
    Boot other(flash, UID_B);
    CHECK(other.status == LoadStatus::kNoValidRecord);
    CHECK_FALSE(other.identity.identified());
    CHECK_FALSE(other.identity.armAllowed());
}

TEST_CASE("Picking a profile while armed is refused and the board stays unarmable")
{
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    Boot                   boot(flash, UID_A);
    boot.guard.blocker     = SaveBlocker::kArmed;
    const SaveResult saved = boot.identity.pick(presetNamed("mouse2"));
    CHECK(saved.status == SaveStatus::kRefused);
    CHECK(saved.blocker == SaveBlocker::kArmed);
    CHECK_FALSE(boot.identity.identified());
    CHECK_FALSE(boot.identity.armAllowed());
    CHECK_FALSE(Boot(flash, UID_A).identity.identified());
}

TEST_CASE("An out-of-range preset index saves nothing")
{
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    Boot                   boot(flash, UID_A);
    const uint64_t         before = flash.wordOps();
    CHECK(boot.identity.pick(board::PROFILE_PRESET_COUNT).status == SaveStatus::kRefused);
    CHECK(flash.wordOps() == before);
    CHECK_FALSE(boot.identity.armAllowed());
}

TEST_CASE("A saved profile that fails its sanity checks sends the board to the picker")
{
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    Boot                   boot(flash, UID_A);

    board::BoardProfile bad = board::PROFILE_PRESETS[0];
    SUBCASE("unknown IR set")
    {
        bad.ir_set = board::IR_SET_COUNT;
    }
    SUBCASE("name not terminated")
    {
        std::memset(bad.name, 'x', sizeof bad.name);
    }
    SUBCASE("empty name")
    {
        bad.name[0] = '\0';
    }
    SUBCASE("divider below 1")
    {
        bad.battery_divider = 0.5f;
    }
    SUBCASE("has_fan neither 0 nor 1")
    {
        bad.has_fan = 7;
    }
    CHECK_FALSE(profileValid(bad));

    // Saved straight through the store, as a corrupt-but-CRC-valid record would be.
    REQUIRE(boot.store.save(&bad, sizeof bad).status == SaveStatus::kOk);
    Boot again(flash, UID_A);
    CHECK_FALSE(again.identity.identified());
    CHECK_FALSE(again.identity.armAllowed());
}

TEST_CASE("A damaged newest record falls back to the previous profile")
{
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    REQUIRE(Boot(flash, UID_A).identity.pick(presetNamed("mouse1")).status == SaveStatus::kOk);
    REQUIRE(Boot(flash, UID_A).identity.pick(presetNamed("mouse2")).status == SaveStatus::kOk);
    // The second save went to slot 1; break its magic word.
    flash.poke(SECTOR_BYTES, 0);
    Boot boot(flash, UID_A);
    CHECK(boot.status == LoadStatus::kFellBack);
    CHECK(boot.identity.identified());
    CHECK(std::strcmp(boot.identity.profile().name, "mouse1") == 0);
}
