#ifndef HAL_STM32_CONSOLE_RINGS_H
#define HAL_STM32_CONSOLE_RINGS_H

/**
 * @file console_rings.h
 * @brief Buffers behind the STM32 console: a transmit ring that a DMA stream
 *        drains, and a cursor over a circular DMA receive buffer. No
 *        libopencm3; host-tested in tests/console_ring_test.cpp.
 */

#include <atomic>
#include <cstdint>
#include <cstring>

namespace hal::stm32
{
/**
 * Single-producer, single-consumer byte ring for DMA transmit.
 *
 * The main loop pushes; the DMA-complete interrupt consumes. push() never
 * blocks: a write that does not fit is dropped whole and its bytes counted,
 * so a flood costs output rather than control-loop time and never leaves half
 * a line queued. Indices run free and wrap at 2^32; N must be a power of two.
 */
template <uint32_t N> class TxRing
{
    static_assert(N > 0 && (N & (N - 1)) == 0, "TxRing size must be a power of two");

  public:
    /// A contiguous run of queued bytes, ready to hand to the DMA.
    struct Span
    {
        const char* data;
        uint32_t    len;
    };

    /// Queues all len bytes, or none of them (counting them as dropped).
    bool push(const char* data, uint32_t len)
    {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        if (len > N - (head - tail))
        {
            dropped_.store(dropped_.load(std::memory_order_relaxed) + len,
                           std::memory_order_relaxed);
            return false;
        }
        const uint32_t start = head & (N - 1);
        const uint32_t first = len < N - start ? len : N - start;
        std::memcpy(buf_ + start, data, first);
        std::memcpy(buf_, data + first, len - first);
        head_.store(head + len, std::memory_order_release);
        return true;
    }

    /// The oldest queued bytes up to the end of storage; len 0 when empty.
    Span nextSpan() const
    {
        const uint32_t tail   = tail_.load(std::memory_order_relaxed);
        const uint32_t head   = head_.load(std::memory_order_acquire);
        const uint32_t start  = tail & (N - 1);
        const uint32_t queued = head - tail;
        const uint32_t to_end = N - start;
        return Span{buf_ + start, queued < to_end ? queued : to_end};
    }

    /// Releases len bytes from the front, once the DMA has sent them.
    void consume(uint32_t len)
    {
        tail_.store(tail_.load(std::memory_order_relaxed) + len, std::memory_order_release);
    }

    uint32_t pending() const
    {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }

    uint32_t free() const { return N - pending(); }

    /// Bytes refused by push() since boot or the last resetDropped().
    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

    void resetDropped() { dropped_.store(0, std::memory_order_relaxed); }

    /// Empties the ring with both indices at `index`; lets tests cross the 2^32 wrap.
    void resetIndicesForTest(uint32_t index)
    {
        head_.store(index);
        tail_.store(index);
    }

  private:
    char                  buf_[N] = {};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    std::atomic<uint32_t> dropped_{0};
};

/**
 * Read cursor over a buffer that a DMA stream fills in circular mode.
 *
 * The DMA's remaining-count register (NDTR) counts down from N to 1 and
 * reloads, so the write position is N - NDTR. A reader that falls a whole
 * buffer behind loses data without notice; at 1 Mbaud a 256-byte buffer
 * gives the main loop 2.5 ms between polls.
 */
template <uint32_t N> class DmaRxRing
{
  public:
    static constexpr uint32_t SIZE = N; ///< The DMA stream's transfer count

    uint8_t* buffer() { return buf_; }

    /// Bytes waiting, given the stream's current NDTR.
    uint32_t available(uint32_t ndtr) const { return (writePos(ndtr) + N - read_) % N; }

    /// Next byte (0..255), or -1 when the reader has caught up with the DMA.
    int getc(uint32_t ndtr)
    {
        if (read_ == writePos(ndtr))
            return -1;
        // Volatile read: the DMA, not this code, writes the buffer.
        const uint8_t c = static_cast<volatile uint8_t*>(buf_)[read_];
        read_           = (read_ + 1) % N;
        return c;
    }

  private:
    static uint32_t writePos(uint32_t ndtr) { return (N - ndtr) % N; }

    uint8_t  buf_[N] = {};
    uint32_t read_   = 0;
};
} // namespace hal::stm32

#endif // HAL_STM32_CONSOLE_RINGS_H
