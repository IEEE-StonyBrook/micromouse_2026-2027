/**
 * @file emitter_check.h
 * @brief IR emitter isolation self-test: fire each emitter alone, read every
 *        receiver with the dark level subtracted, and judge the resulting
 *        crosstalk matrix.
 *
 * Platform-neutral: the hardware sits behind IrBank, so
 * tests/emitter_check_test.cpp runs it on the host against
 * tests/fakes/fake_ir_bank.h. Main loop only (readAll() blocks while the
 * receivers settle).
 *
 * Bench setup: the mouse sits in a cell with walls left, right and ahead (or
 * a white card in front of each sensor), so every emitter has a reflector.
 * Without one the diagonal reads near zero and the check fails as weak.
 */
#ifndef COMMON_EMITTER_CHECK_H
#define COMMON_EMITTER_CHECK_H

#include <cstddef>
#include <cstdint>

/**
 * @brief The IR emitters and receivers, paired by index: emitter i is the one
 *        aimed at receiver i's field of view.
 */
class IrBank
{
  public:
    static constexpr std::size_t kMaxChannels = 4;
    static constexpr int         kAllOff      = -1;

    virtual ~IrBank() = default;

    /// Emitter/receiver pairs on this board, 0..kMaxChannels.
    virtual std::size_t channels() const = 0;

    /// Lights emitter `emitter` alone (kAllOff: none), waits for the receivers
    /// to settle, reads every receiver's raw ADC counts into
    /// counts[0..channels()), and leaves every emitter off on return.
    virtual void readAll(int emitter, uint16_t* counts) = 0;
};

/// Dark/lit read pairs averaged per emitter.
constexpr int CROSSTALK_SAMPLES = 8;

/// counts[e][r]: receiver r's mean lit-minus-dark reading (raw ADC counts)
/// with emitter e lit alone. Negative values are noise.
struct CrosstalkMatrix
{
    std::size_t channels = 0;
    int32_t     counts[IrBank::kMaxChannels][IrBank::kMaxChannels] = {};
};

/// Fires each emitter alone CROSSTALK_SAMPLES times, each lit read preceded
/// by a dark read, and averages the differences.
CrosstalkMatrix captureCrosstalk(IrBank& bank);

#endif // COMMON_EMITTER_CHECK_H
