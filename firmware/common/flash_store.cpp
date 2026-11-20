#include "common/flash_store.h"

#include <cstring>

#include "common/crc32.h"

namespace
{
constexpr uint32_t HEADER_WORDS = 5;
constexpr uint32_t HEADER_BYTES = HEADER_WORDS * 4;
// Payload is read and written through a small stack buffer, never all at once.
constexpr uint32_t CHUNK_WORDS = 16;

uint32_t wordsFor(std::size_t bytes)
{
    return static_cast<uint32_t>((bytes + 3) / 4);
}

/// Little-endian bytes of a word, so the CRC does not depend on the host's byte order.
void wordBytes(uint32_t word, uint8_t (&out)[4])
{
    for (int i = 0; i < 4; ++i)
        out[i] = static_cast<uint8_t>(word >> (8 * i));
}

uint32_t crcWords(uint32_t state, const uint32_t* words, std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i)
    {
        uint8_t bytes[4];
        wordBytes(words[i], bytes);
        state = crc32::update(state, bytes, 4);
    }
    return state;
}

/// Packs payload bytes [first, first + 4 * count) into words, zero-padding past len.
void packWords(const uint8_t* payload, std::size_t len, std::size_t first, uint32_t* words,
               std::size_t count)
{
    for (std::size_t w = 0; w < count; ++w)
    {
        uint32_t word = 0;
        for (std::size_t b = 0; b < 4; ++b)
        {
            const std::size_t i = first + 4 * w + b;
            if (i < len)
                word |= static_cast<uint32_t>(payload[i]) << (8 * b);
        }
        words[w] = word;
    }
}

void unpackWords(const uint32_t* words, std::size_t count, std::size_t first, uint8_t* payload,
                 std::size_t len)
{
    for (std::size_t w = 0; w < count; ++w)
        for (std::size_t b = 0; b < 4; ++b)
        {
            const std::size_t i = first + 4 * w + b;
            if (i < len)
                payload[i] = static_cast<uint8_t>(words[w] >> (8 * b));
        }
}

/// CRC of a stored record's header and payload, streamed from flash.
uint32_t storedCrc(const hal::FlashDevice& flash, uint32_t base, const uint32_t (&header)[5],
                   uint32_t len)
{
    uint32_t state = crcWords(crc32::INITIAL, header, HEADER_WORDS);
    uint32_t chunk[CHUNK_WORDS];
    uint8_t  bytes[CHUNK_WORDS * 4];
    for (uint32_t done = 0; done < len; done += CHUNK_WORDS * 4)
    {
        const uint32_t n = wordsFor(len - done) < CHUNK_WORDS ? wordsFor(len - done) : CHUNK_WORDS;
        flash.read(base + HEADER_BYTES + done, chunk, n);
        const uint32_t chunk_len = len - done < n * 4 ? len - done : n * 4;
        unpackWords(chunk, n, 0, bytes, chunk_len);
        state = crc32::update(state, bytes, chunk_len);
    }
    return crc32::finish(state);
}

/// True when a minus b is positive in wrapping 32-bit arithmetic.
bool newer(uint32_t a, uint32_t b)
{
    return static_cast<int32_t>(a - b) > 0;
}
} // namespace

SaveBlocker saveBlocker(const OutputState& outputs)
{
    if (outputs.armed)
        return SaveBlocker::kArmed;
    if (outputs.motors_awake)
        return SaveBlocker::kMotorsAwake;
    if (outputs.fan_on)
        return SaveBlocker::kFanOn;
    if (outputs.emitters_on)
        return SaveBlocker::kEmittersOn;
    return SaveBlocker::kNone;
}

uint8_t ledFlashes(LoadStatus status)
{
    switch (status)
    {
        case LoadStatus::kFellBack:
            return 2;
        case LoadStatus::kNoValidRecord:
            return 4;
        default:
            return 0;
    }
}

FlashStore::FlashStore(hal::FlashDevice& flash, SaveGuard& guard, uint32_t version,
                       uint32_t uid_hash)
    : flash_(flash), guard_(guard), version_(version), uid_hash_(uid_hash)
{
}

std::size_t FlashStore::maxPayloadBytes() const
{
    return flash_.sectorSizeBytes() - HEADER_BYTES - 4;
}

FlashStore::SlotInfo FlashStore::inspect(uint32_t slot) const
{
    const uint32_t base = slot * flash_.sectorSizeBytes();
    uint32_t       header[HEADER_WORDS];
    flash_.read(base, header, HEADER_WORDS);

    SlotInfo info{false, sectorErased(base), header[2], header[3]};
    if (info.blank)
        return info;

    if (header[0] != RECORD_MAGIC || header[1] != version_ || header[4] != uid_hash_ ||
        header[3] > maxPayloadBytes())
        return info;

    uint32_t crc = 0;
    flash_.read(base + HEADER_BYTES + wordsFor(header[3]) * 4, &crc, 1);
    info.valid = crc == storedCrc(flash_, base, header, header[3]);
    return info;
}

bool FlashStore::sectorErased(uint32_t base) const
{
    // The whole sector, not just the header: an erase cut off part way can
    // leave the header erased and old payload behind it, which is damage.
    uint32_t chunk[CHUNK_WORDS];
    for (uint32_t offset = 0; offset < flash_.sectorSizeBytes(); offset += CHUNK_WORDS * 4)
    {
        flash_.read(base + offset, chunk, CHUNK_WORDS);
        for (uint32_t word : chunk)
            if (word != hal::FlashDevice::ERASED_WORD)
                return false;
    }
    return true;
}

int FlashStore::newest(const SlotInfo (&slots)[2]) const
{
    if (slots[0].valid && slots[1].valid)
        return newer(slots[1].seq, slots[0].seq) ? 1 : 0;
    if (slots[0].valid)
        return 0;
    if (slots[1].valid)
        return 1;
    return -1;
}

LoadResult FlashStore::load(void* payload, std::size_t capacity_bytes) const
{
    const SlotInfo slots[2] = {inspect(0), inspect(1)};
    const int      pick     = newest(slots);
    const bool     damaged =
        (!slots[0].valid && !slots[0].blank) || (!slots[1].valid && !slots[1].blank);
    if (pick < 0)
        return {damaged ? LoadStatus::kNoValidRecord : LoadStatus::kEmpty, 0, 0};
    // A newest record too big for the caller's buffer is from another
    // settings layout that kept the version number by mistake. Neither
    // half-load it nor fall back to the older slot: the next save erases
    // that one, so loading it now would only hide the problem until then.
    if (slots[pick].len > capacity_bytes)
        return {LoadStatus::kNoValidRecord, 0, 0};

    const uint32_t base = static_cast<uint32_t>(pick) * flash_.sectorSizeBytes();
    const uint32_t len  = slots[pick].len;
    auto*          out  = static_cast<uint8_t*>(payload);
    uint32_t       chunk[CHUNK_WORDS];
    for (uint32_t done = 0; done < len; done += CHUNK_WORDS * 4)
    {
        const uint32_t n = wordsFor(len - done) < CHUNK_WORDS ? wordsFor(len - done) : CHUNK_WORDS;
        flash_.read(base + HEADER_BYTES + done, chunk, n);
        unpackWords(chunk, n, done, out, len);
    }
    return {damaged ? LoadStatus::kFellBack : LoadStatus::kOk, slots[pick].seq, len};
}

SaveResult FlashStore::save(const void* payload, std::size_t len)
{
    const SaveBlocker blocker = guard_.prepareForSave();
    if (blocker != SaveBlocker::kNone)
        return {SaveStatus::kRefused, blocker, 0};
    if (len > maxPayloadBytes())
        return {SaveStatus::kTooLarge, SaveBlocker::kNone, 0};

    const SlotInfo slots[2] = {inspect(0), inspect(1)};
    const int      keep     = newest(slots);
    const uint32_t target   = keep == 0 ? 1 : 0;
    const uint32_t seq      = keep < 0 ? 1 : slots[keep].seq + 1;
    const uint32_t base     = target * flash_.sectorSizeBytes();

    const uint32_t header[HEADER_WORDS] = {RECORD_MAGIC, version_, seq, static_cast<uint32_t>(len),
                                           uid_hash_};
    const auto*    bytes                = static_cast<const uint8_t*>(payload);
    const uint32_t crc =
        crc32::finish(crc32::update(crcWords(crc32::INITIAL, header, HEADER_WORDS), bytes, len));

    const SaveResult failed{SaveStatus::kFlashError, SaveBlocker::kNone, 0};
    if (!flash_.erase(target) || !flash_.program(base, header, HEADER_WORDS))
        return failed;
    const uint32_t payload_words = wordsFor(len);
    uint32_t       chunk[CHUNK_WORDS];
    for (uint32_t w = 0; w < payload_words; w += CHUNK_WORDS)
    {
        const uint32_t n = payload_words - w < CHUNK_WORDS ? payload_words - w : CHUNK_WORDS;
        packWords(bytes, len, w * 4, chunk, n);
        if (!flash_.program(base + HEADER_BYTES + w * 4, chunk, n))
            return failed;
    }
    // The CRC goes in last: until it lands, the record is invalid and the
    // other slot's record is still the newest.
    if (!flash_.program(base + HEADER_BYTES + payload_words * 4, &crc, 1))
        return failed;

    const SlotInfo written = inspect(target);
    if (!written.valid || written.seq != seq)
        return failed;
    return {SaveStatus::kOk, SaveBlocker::kNone, seq};
}
