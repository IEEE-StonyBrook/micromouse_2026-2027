// Host tests for the A/B flash store and the in-memory flash it runs on here.
// The in-memory flash can cut power at any word of an erase or a program, so
// the power-loss cases sweep every word of a save rather than sampling a few.
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "common/crc32.h"
#include "common/flash_store.h"
#include "doctest/doctest.h"
#include "hal/host/memory_flash.h"

namespace
{
constexpr uint32_t SECTOR_BYTES = 1024;
// The STM32F405's sectors 1 and 2, which the real store lives in.
constexpr uint32_t STORE_SECTOR_BYTES = 16 * 1024;
constexpr uint32_t VERSION            = 3;
constexpr uint32_t UID_HASH           = 0x1234ABCDu;

// Record word offsets, from the layout documented in common/flash_store.h.
constexpr uint32_t MAGIC_OFFSET    = 0;
constexpr uint32_t VERSION_OFFSET  = 4;
constexpr uint32_t LEN_OFFSET      = 12;
constexpr uint32_t UID_HASH_OFFSET = 16;

struct Settings
{
    uint32_t counter;
    float    gains[6];
    char     name[12];
};

Settings makeSettings(uint32_t counter)
{
    Settings s{};
    s.counter = counter;
    for (int i = 0; i < 6; ++i)
        s.gains[i] = 0.5f * static_cast<float>(counter + i);
    std::snprintf(s.name, sizeof s.name, "rev%u", static_cast<unsigned>(counter));
    return s;
}

bool same(const Settings& a, const Settings& b)
{
    return std::memcmp(&a, &b, sizeof a) == 0;
}

/// Stands in for the app's guard: refuses with whatever blocker the test sets.
class TestGuard : public SaveGuard
{
  public:
    SaveBlocker prepareForSave() override
    {
        ++calls;
        return blocker;
    }
    SaveBlocker blocker = SaveBlocker::kNone;
    int         calls   = 0;
};

uint32_t slotOffset(int slot)
{
    return static_cast<uint32_t>(slot) * STORE_SECTOR_BYTES;
}

uint32_t readWord(const hal::host::MemoryFlash& flash, uint32_t offset)
{
    uint32_t word = 0;
    flash.read(offset, &word, 1);
    return word;
}

/// Loads into a buffer pre-filled with a sentinel, so "untouched" is checkable.
LoadResult loadFresh(hal::FlashDevice& flash, Settings& out, uint32_t uid_hash = UID_HASH,
                     uint32_t version = VERSION)
{
    TestGuard  guard;
    FlashStore store(flash, guard, version, uid_hash);
    out = makeSettings(0xDEFA017u);
    return store.load(&out, sizeof out);
}
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

TEST_CASE("In-memory flash leaves a whole sector part erased when power dies mid-erase")
{
    hal::host::MemoryFlash flash(1, SECTOR_BYTES);
    const uint32_t         zero = 0;
    for (uint32_t offset = 0; offset < SECTOR_BYTES; offset += 4)
        REQUIRE(flash.program(offset, &zero, 1));
    flash.cutPowerAfterWords(3);
    CHECK_FALSE(flash.erase(0));
    // Not a clean prefix: even the last word has some bits set and some not.
    uint32_t last = 0;
    flash.read(SECTOR_BYTES - 4, &last, 1);
    CHECK(last != 0);
    CHECK(last != 0xFFFFFFFFu);
}

TEST_CASE("A blank store loads nothing and leaves the defaults alone")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    Settings               got;
    const LoadResult       result = loadFresh(flash, got);
    CHECK(result.status == LoadStatus::kEmpty);
    CHECK(same(got, makeSettings(0xDEFA017u)));
    CHECK(ledFlashes(result.status) == 0);
}

TEST_CASE("A saved record loads back byte for byte")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    TestGuard              guard;
    FlashStore             store(flash, guard, VERSION, UID_HASH);
    const Settings         saved = makeSettings(7);
    const SaveResult       wrote = store.save(&saved, sizeof saved);
    REQUIRE(wrote.status == SaveStatus::kOk);
    CHECK(wrote.seq == 1);
    CHECK(guard.calls == 1);

    Settings         got;
    const LoadResult result = loadFresh(flash, got);
    CHECK(result.status == LoadStatus::kOk);
    CHECK(result.seq == 1);
    CHECK(result.len == sizeof saved);
    CHECK(same(got, saved));
}

TEST_CASE("A payload that is not a whole number of words round-trips")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    TestGuard              guard;
    FlashStore             store(flash, guard, VERSION, UID_HASH);
    const char             text[7] = {'a', 'b', 'c', 'd', 'e', 'f', 'g'};
    REQUIRE(store.save(text, sizeof text).status == SaveStatus::kOk);
    char             got[8] = {};
    const LoadResult result = store.load(got, sizeof got);
    CHECK(result.status == LoadStatus::kOk);
    CHECK(result.len == 7);
    CHECK(std::memcmp(got, text, 7) == 0);
}

TEST_CASE("Saves alternate slots and the highest sequence number wins")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    TestGuard              guard;
    FlashStore             store(flash, guard, VERSION, UID_HASH);
    for (uint32_t n = 1; n <= 5; ++n)
    {
        const Settings s = makeSettings(n);
        REQUIRE(store.save(&s, sizeof s).status == SaveStatus::kOk);
    }
    // Five saves: 1, 3, 5 in slot 0 and 2, 4 in slot 1, so the newest is in slot 0.
    CHECK(readWord(flash, slotOffset(0) + 8) == 5);
    CHECK(readWord(flash, slotOffset(1) + 8) == 4);

    Settings         got;
    const LoadResult result = loadFresh(flash, got);
    CHECK(result.status == LoadStatus::kOk);
    CHECK(result.seq == 5);
    CHECK(same(got, makeSettings(5)));
}

TEST_CASE("A bad newest record falls back to the other slot, with an LED code")
{
    struct Corruption
    {
        const char* what;
        uint32_t    offset;
    };
    // The payload starts after the five header words; the CRC follows it.
    const Corruption cases[] = {
        {"magic", MAGIC_OFFSET},
        {"version", VERSION_OFFSET},
        {"uid_hash", UID_HASH_OFFSET},
        {"payload", 20 + 8},
        {"crc", 20 + static_cast<uint32_t>((sizeof(Settings) + 3) / 4 * 4)},
    };
    for (const Corruption& c : cases)
    {
        CAPTURE(c.what);
        hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
        TestGuard              guard;
        FlashStore             store(flash, guard, VERSION, UID_HASH);
        const Settings         older = makeSettings(1);
        const Settings         newer = makeSettings(2);
        REQUIRE(store.save(&older, sizeof older).status == SaveStatus::kOk);
        REQUIRE(store.save(&newer, sizeof newer).status == SaveStatus::kOk);

        // The newer record is in slot 1; flip one bit of the chosen word.
        const uint32_t at = slotOffset(1) + c.offset;
        flash.poke(at, readWord(flash, at) ^ 0x00010000u);

        Settings         got;
        const LoadResult result = loadFresh(flash, got);
        CHECK(result.status == LoadStatus::kFellBack);
        CHECK(result.seq == 1);
        CHECK(same(got, older));
        CHECK(ledFlashes(result.status) == 2);
    }
}

TEST_CASE("With no valid record the store keeps the defaults and shows a fault code")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    TestGuard              guard;
    FlashStore             store(flash, guard, VERSION, UID_HASH);
    const Settings         a = makeSettings(1);
    REQUIRE(store.save(&a, sizeof a).status == SaveStatus::kOk);
    REQUIRE(store.save(&a, sizeof a).status == SaveStatus::kOk);

    Settings got;
    SUBCASE("both CRCs bad")
    {
        const uint32_t crc_at = 20 + static_cast<uint32_t>((sizeof(Settings) + 3) / 4 * 4);
        flash.poke(slotOffset(0) + crc_at, 0);
        flash.poke(slotOffset(1) + crc_at, 0);
        const LoadResult result = loadFresh(flash, got);
        CHECK(result.status == LoadStatus::kNoValidRecord);
        CHECK(ledFlashes(result.status) == 4);
    }
    SUBCASE("records from an older settings version")
    {
        const LoadResult result = loadFresh(flash, got, UID_HASH, VERSION + 1);
        CHECK(result.status == LoadStatus::kNoValidRecord);
        CHECK(ledFlashes(result.status) == 4);
    }
    SUBCASE("records written by another board")
    {
        const LoadResult result = loadFresh(flash, got, UID_HASH ^ 1u);
        CHECK(result.status == LoadStatus::kNoValidRecord);
        CHECK(ledFlashes(result.status) == 4);
    }
    SUBCASE("a length field past the end of the slot")
    {
        flash.poke(slotOffset(0) + LEN_OFFSET, STORE_SECTOR_BYTES);
        flash.poke(slotOffset(1) + LEN_OFFSET, 0xFFFFFFF0u);
        const LoadResult result = loadFresh(flash, got);
        CHECK(result.status == LoadStatus::kNoValidRecord);
        CHECK(ledFlashes(result.status) == 4);
    }
    CHECK(same(got, makeSettings(0xDEFA017u)));
}

TEST_CASE("A newest record larger than the caller's buffer is not loaded, nor the older one")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    TestGuard              guard;
    FlashStore             store(flash, guard, VERSION, UID_HASH);
    const uint32_t         fits = 0xAAAA5555u;
    REQUIRE(store.save(&fits, sizeof fits).status == SaveStatus::kOk);
    const Settings big = makeSettings(1);
    REQUIRE(store.save(&big, sizeof big).status == SaveStatus::kOk);

    // The older record would fit, but the next save erases its slot, so
    // loading it would only hide the mismatch until then.
    uint32_t small[2] = {};
    CHECK(store.load(small, sizeof small).status == LoadStatus::kNoValidRecord);
    CHECK(small[0] == 0);

    // A save still keeps the newest record and writes over the older slot.
    REQUIRE(store.save(&fits, sizeof fits).status == SaveStatus::kOk);
    CHECK(store.load(small, sizeof small).status == LoadStatus::kOk);
    CHECK(small[0] == fits);
}

namespace
{
/// Writes a record straight into a slot, laid out as common/flash_store.h
/// documents it, so the test does not lean on the store's own writer.
void writeRawRecord(hal::host::MemoryFlash& flash, int slot, uint32_t seq, uint32_t value)
{
    const uint32_t words[6] = {FlashStore::RECORD_MAGIC, VERSION, seq, 4, UID_HASH, value};
    uint8_t        bytes[24];
    for (int i = 0; i < 24; ++i)
        bytes[i] = static_cast<uint8_t>(words[i / 4] >> (8 * (i % 4)));
    const uint32_t crc = crc32::compute(bytes, sizeof bytes);
    REQUIRE(flash.erase(static_cast<uint32_t>(slot)));
    REQUIRE(flash.program(slotOffset(slot), words, 6));
    REQUIRE(flash.program(slotOffset(slot) + 24, &crc, 1));
}
} // namespace

TEST_CASE("The sequence number wraps: 0 is newer than 0xFFFFFFFF")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    writeRawRecord(flash, 0, 0xFFFFFFFFu, 111);
    writeRawRecord(flash, 1, 0, 222);
    TestGuard  guard;
    FlashStore store(flash, guard, VERSION, UID_HASH);
    uint32_t   got = 0;
    CHECK(store.load(&got, sizeof got).status == LoadStatus::kOk);
    CHECK(got == 222);

    // The next save goes over slot 0, the older record, as seq 1.
    const uint32_t   next  = 333;
    const SaveResult wrote = store.save(&next, sizeof next);
    CHECK(wrote.status == SaveStatus::kOk);
    CHECK(wrote.seq == 1);
    CHECK(readWord(flash, slotOffset(1) + 20) == 222);
}

namespace
{
/// Flash whose programs report success but never reach the cells.
class SilentFlash : public hal::FlashDevice
{
  public:
    explicit SilentFlash(hal::host::MemoryFlash& inner) : inner_(inner) {}
    uint32_t sectorCount() const override { return inner_.sectorCount(); }
    uint32_t sectorSizeBytes() const override { return inner_.sectorSizeBytes(); }
    bool     erase(uint32_t sector) override { return inner_.erase(sector); }
    bool     program(uint32_t, const uint32_t*, std::size_t) override { return true; }
    void     read(uint32_t offset_bytes, uint32_t* words, std::size_t count) const override
    {
        inner_.read(offset_bytes, words, count);
    }

  private:
    hal::host::MemoryFlash& inner_;
};
} // namespace

TEST_CASE("A save that does not read back reports a flash error and the old record survives")
{
    hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
    TestGuard              guard;
    {
        FlashStore     store(flash, guard, VERSION, UID_HASH);
        const Settings old = makeSettings(1);
        REQUIRE(store.save(&old, sizeof old).status == SaveStatus::kOk);
    }
    SilentFlash    silent(flash);
    FlashStore     store(silent, guard, VERSION, UID_HASH);
    const Settings changed = makeSettings(2);
    CHECK(store.save(&changed, sizeof changed).status == SaveStatus::kFlashError);

    Settings got;
    CHECK(loadFresh(flash, got).status == LoadStatus::kOk);
    CHECK(same(got, makeSettings(1)));
}

TEST_CASE("A payload that does not fit a slot is refused before anything is erased")
{
    hal::host::MemoryFlash flash(2, SECTOR_BYTES);
    TestGuard              guard;
    FlashStore             store(flash, guard, VERSION, UID_HASH);
    // Five header words and one CRC word share the sector with the payload.
    CHECK(store.maxPayloadBytes() == SECTOR_BYTES - 24);
    static uint8_t big[SECTOR_BYTES] = {};
    const uint64_t before            = flash.wordOps();
    CHECK(store.save(big, store.maxPayloadBytes() + 1).status == SaveStatus::kTooLarge);
    CHECK(flash.wordOps() == before);
    CHECK(store.save(big, store.maxPayloadBytes()).status == SaveStatus::kOk);
}

TEST_CASE("Saving is refused while armed or with any output on, and flash is untouched")
{
    const SaveBlocker blockers[] = {SaveBlocker::kArmed, SaveBlocker::kMotorsAwake,
                                    SaveBlocker::kFanOn, SaveBlocker::kEmittersOn};
    for (SaveBlocker blocker : blockers)
    {
        CAPTURE(static_cast<int>(blocker));
        hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
        TestGuard              guard;
        FlashStore             store(flash, guard, VERSION, UID_HASH);
        const Settings         old = makeSettings(1);
        REQUIRE(store.save(&old, sizeof old).status == SaveStatus::kOk);

        guard.blocker            = blocker;
        const uint64_t   before  = flash.wordOps();
        const Settings   changed = makeSettings(2);
        const SaveResult result  = store.save(&changed, sizeof changed);
        CHECK(result.status == SaveStatus::kRefused);
        CHECK(result.blocker == blocker);
        CHECK(flash.wordOps() == before);

        Settings got;
        CHECK(loadFresh(flash, got).status == LoadStatus::kOk);
        CHECK(same(got, old));
    }
}

TEST_CASE("The guard's verdict follows the arm state and every output")
{
    CHECK(saveBlocker({false, false, false, false}) == SaveBlocker::kNone);
    // Armed is checked first: an armed mouse may legitimately have its outputs on.
    CHECK(saveBlocker({true, true, true, true}) == SaveBlocker::kArmed);
    CHECK(saveBlocker({false, true, false, false}) == SaveBlocker::kMotorsAwake);
    CHECK(saveBlocker({false, false, true, false}) == SaveBlocker::kFanOn);
    CHECK(saveBlocker({false, false, false, true}) == SaveBlocker::kEmittersOn);
}

namespace
{
/// Cuts power at every word of one save and checks each outcome. prior is the
/// number of saves made before the one that is cut.
void sweepPowerCuts(uint32_t prior)
{
    // Measure how many word operations one save takes from this starting state.
    uint64_t ops = 0;
    {
        hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
        TestGuard              guard;
        FlashStore             store(flash, guard, VERSION, UID_HASH);
        for (uint32_t n = 1; n <= prior; ++n)
        {
            const Settings s = makeSettings(n);
            REQUIRE(store.save(&s, sizeof s).status == SaveStatus::kOk);
        }
        const uint64_t before = flash.wordOps();
        const Settings next   = makeSettings(prior + 1);
        REQUIRE(store.save(&next, sizeof next).status == SaveStatus::kOk);
        ops = flash.wordOps() - before;
    }
    // One erase of a 16 KB sector, then the record.
    REQUIRE(ops > STORE_SECTOR_BYTES / 4);

    const Settings old_settings = makeSettings(prior);
    const Settings new_settings = makeSettings(prior + 1);
    for (uint64_t cut = 0; cut < ops; ++cut)
    {
        hal::host::MemoryFlash flash(2, STORE_SECTOR_BYTES);
        TestGuard              guard;
        FlashStore             store(flash, guard, VERSION, UID_HASH);
        for (uint32_t n = 1; n <= prior; ++n)
        {
            const Settings s = makeSettings(n);
            REQUIRE(store.save(&s, sizeof s).status == SaveStatus::kOk);
        }
        flash.cutPowerAfterWords(cut);
        const bool saved = store.save(&new_settings, sizeof new_settings).status == SaveStatus::kOk;
        flash.restorePower();

        // Reboot: a new store object reads what the flash holds now.
        Settings         got;
        const LoadResult result = loadFresh(flash, got);
        const bool       is_new = same(got, new_settings);
        const bool       is_old =
            prior > 0 ? same(got, old_settings) : same(got, makeSettings(0xDEFA017u));
        if (!(is_new || is_old) || saved)
        {
            // Report only the failing cut, not one line per cut.
            CAPTURE(cut);
            CAPTURE(saved);
            CHECK((is_new || is_old));
            CHECK_FALSE(saved); // power died, so the save cannot claim success
        }
        // Saves alternate slots, so this one went to slot prior % 2. A cut
        // there shows the damage LED code, unless the new record made it in
        // whole or the cut erase happened to leave every cell erased.
        const uint32_t target       = prior % 2;
        bool           target_blank = true;
        for (uint32_t offset = 0; offset < STORE_SECTOR_BYTES; offset += 4)
            target_blank = target_blank && readWord(flash, slotOffset(static_cast<int>(target)) +
                                                               offset) == 0xFFFFFFFFu;
        LoadStatus expected = LoadStatus::kOk;
        if (!is_new && prior > 0)
            expected = target_blank ? LoadStatus::kOk : LoadStatus::kFellBack;
        if (!is_new && prior == 0)
            expected = target_blank ? LoadStatus::kEmpty : LoadStatus::kNoValidRecord;
        if (result.status != expected)
        {
            CAPTURE(cut);
            CAPTURE(static_cast<int>(result.status));
            FAIL("unexpected load status after a cut save");
        }

        // The next save after the reboot must work and win.
        TestGuard      guard2;
        FlashStore     store2(flash, guard2, VERSION, UID_HASH);
        const Settings after = makeSettings(100);
        if (store2.save(&after, sizeof after).status != SaveStatus::kOk)
        {
            CAPTURE(cut);
            FAIL("save after recovery failed");
        }
        Settings final_got;
        if (loadFresh(flash, final_got).status != LoadStatus::kOk || !same(final_got, after))
        {
            CAPTURE(cut);
            FAIL("save after recovery did not load back cleanly");
        }
    }
}
} // namespace

TEST_CASE("A power cut at any word of the first save leaves defaults or the new record")
{
    sweepPowerCuts(0);
}

TEST_CASE("A power cut at any word of a save leaves the old record or the new one")
{
    sweepPowerCuts(1);
}

TEST_CASE("A power cut at any word of a save over two slots leaves the old record or the new one")
{
    sweepPowerCuts(2);
}

TEST_CASE("The LED code shows its flash count as 100 ms blinks every 2 s")
{
    // Two flashes: on 0-99 ms, off 100-199, on 200-299, then dark to 2 s.
    CHECK(ledCodeOn(0, 2));
    CHECK(ledCodeOn(99, 2));
    CHECK_FALSE(ledCodeOn(100, 2));
    CHECK(ledCodeOn(250, 2));
    CHECK_FALSE(ledCodeOn(300, 2));
    CHECK_FALSE(ledCodeOn(1999, 2));
    CHECK(ledCodeOn(2000, 2));
    // Four flashes reach a fourth blink at 600-699 ms.
    CHECK(ledCodeOn(650, 4));
    CHECK_FALSE(ledCodeOn(750, 4));
    // No code, no light.
    CHECK_FALSE(ledCodeOn(0, 0));
}
