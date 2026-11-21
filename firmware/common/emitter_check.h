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

    /// Emitter/receiver pairs on this board, 0..kMaxChannels.
    virtual std::size_t channels() const = 0;

    /// Lights emitter `emitter` alone (kAllOff: none), waits for the receivers
    /// to settle, reads every receiver's raw ADC counts into
    /// counts[0..channels()), and leaves every emitter off on return.
    virtual void readAll(int emitter, uint16_t* counts) = 0;

  protected:
    // Non-virtual and trivial, so a static bank needs no exit-time destructor
    // (the STM32 link has no __dso_handle). Never deleted through IrBank*.
    ~IrBank() = default;
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

/// A receiver whose own emitter lifts it by fewer counts than this is weak: a
/// dead emitter or receiver, or no reflector in front of it. About 2.5 % of
/// a 12-bit ADC's 4095, well clear of its few-count noise. A first guess,
/// to tune on the rev1 board.
constexpr int32_t CROSSTALK_MIN_DIAGONAL_COUNTS = 100;

/// Emitter e leaks into receiver r (e != r) when it lifts r by more than this
/// percentage of r's own diagonal reading. At 25 % the three other emitters
/// together stay under 75 % of the diagonal, so a passing matrix is strictly
/// diagonal-dominant down every column.
constexpr int32_t CROSSTALK_MAX_LEAK_PERCENT = 25;

struct CrosstalkVerdict
{
    struct Leak
    {
        uint8_t emitter;
        uint8_t receiver;
    };

    /// True only with at least one channel, no weak receiver and no leak.
    bool        pass = false;
    /// Bit r set: receiver r is weak (below CROSSTALK_MIN_DIAGONAL_COUNTS).
    /// A weak receiver's column is not judged for leaks.
    uint8_t     weak_mask  = 0;
    std::size_t leak_count = 0;
    Leak        leaks[IrBank::kMaxChannels * (IrBank::kMaxChannels - 1)] = {};
};

/// Applies the weak and leak thresholds above.
CrosstalkVerdict judgeCrosstalk(const CrosstalkMatrix& m);

/// Captures and judges the matrix, then prints it through hal::console: one
/// row per lit emitter (E0..), one column per receiver (R0..), then each weak
/// receiver and leaking pair and a final "emitters: PASS" or "emitters: FAIL".
CrosstalkVerdict runEmitterCheck(IrBank& bank);

#endif // COMMON_EMITTER_CHECK_H
