#ifndef TESTS_FAKES_FAKE_IR_BANK_H
#define TESTS_FAKES_FAKE_IR_BANK_H

/**
 * @brief Host stand-in for an IR emitter/receiver bank (common/emitter_check.h).
 *
 * Each receiver reads its ambient (dark) level plus, when an emitter is lit,
 * that emitter's coupling into it. Records which emitters each read lit, so a
 * test can check that the self-test fires them one at a time.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common/emitter_check.h"

namespace fake
{
class IrBank final : public ::IrBank
{
  public:
    explicit IrBank(std::size_t channels) : channels_(channels) {}

    /// Ambient counts receiver r reads with every emitter off.
    uint16_t dark[IrBank::kMaxChannels] = {};
    /// coupling[e][r]: counts emitter e adds to receiver r when lit alone.
    uint16_t coupling[IrBank::kMaxChannels][IrBank::kMaxChannels] = {};
    /// The emitter lit for each readAll() call, in order (kAllOff for none).
    std::vector<int> lit;

    std::size_t channels() const override { return channels_; }

    void readAll(int emitter, uint16_t* counts) override
    {
        lit.push_back(emitter);
        for (std::size_t r = 0; r < channels_; ++r)
        {
            uint32_t value = dark[r];
            if (emitter != kAllOff)
                value += coupling[emitter][r];
            counts[r] = static_cast<uint16_t>(value);
        }
    }

  private:
    std::size_t channels_;
};
} // namespace fake

#endif // TESTS_FAKES_FAKE_IR_BANK_H
