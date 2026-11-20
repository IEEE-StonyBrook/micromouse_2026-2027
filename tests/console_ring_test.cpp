// Host tests for the STM32 console buffers: the DMA transmit ring that drops
// and counts instead of blocking, and the circular DMA receive cursor.
#include <cstring>
#include <string>

#include "doctest/doctest.h"
#include "hal/stm32/console_rings.h"

using hal::stm32::DmaRxRing;
using hal::stm32::TxRing;

namespace
{
template <uint32_t N> bool push(TxRing<N>& ring, const char* text)
{
    return ring.push(text, static_cast<uint32_t>(std::strlen(text)));
}

// Drains the ring the way the DMA does, one contiguous span at a time.
template <uint32_t N> std::string drain(TxRing<N>& ring)
{
    std::string out;
    while (true)
    {
        const auto span = ring.nextSpan();
        if (span.len == 0)
            return out;
        out.append(span.data, span.len);
        ring.consume(span.len);
    }
}
} // namespace

TEST_CASE("TX ring hands back queued bytes in order")
{
    TxRing<16> ring;
    CHECK(push(ring, "hello "));
    CHECK(push(ring, "world"));
    CHECK(ring.pending() == 11);
    CHECK(drain(ring) == "hello world");
    CHECK(ring.pending() == 0);
    CHECK(ring.dropped() == 0);
}

TEST_CASE("TX ring drops a whole write that does not fit, and counts its bytes")
{
    TxRing<16> ring;
    CHECK(push(ring, "0123456789"));
    CHECK_FALSE(push(ring, "abcdefgh")); // 8 > 6 free
    CHECK(ring.dropped() == 8);
    CHECK(push(ring, "abcdef")); // exactly fills it
    CHECK(ring.free() == 0);
    CHECK_FALSE(push(ring, "x"));
    CHECK(ring.dropped() == 9);
    // What was queued survives untouched: no partial line from the dropped writes.
    CHECK(drain(ring) == "0123456789abcdef");

    ring.resetDropped();
    CHECK(ring.dropped() == 0);
}

TEST_CASE("TX ring drops a write longer than the whole ring")
{
    TxRing<8> ring;
    CHECK_FALSE(push(ring, "123456789"));
    CHECK(ring.dropped() == 9);
    CHECK(ring.pending() == 0);
}

TEST_CASE("TX ring splits a wrapped write into two DMA spans")
{
    TxRing<8> ring;
    CHECK(push(ring, "abcdef"));
    ring.consume(6); // DMA sent them; head and tail now sit at 6
    CHECK(push(ring, "WXYZ"));

    const auto first = ring.nextSpan();
    CHECK(std::string(first.data, first.len) == "WX"); // up to the end of storage
    ring.consume(first.len);
    const auto second = ring.nextSpan();
    CHECK(std::string(second.data, second.len) == "YZ");
    ring.consume(second.len);
    CHECK(ring.nextSpan().len == 0);
}

TEST_CASE("TX ring keeps working after its indices wrap 2^32")
{
    TxRing<8> ring;
    ring.resetIndicesForTest(0xFFFFFFFCu);
    CHECK(push(ring, "abcdef"));
    CHECK(ring.pending() == 6);
    CHECK(drain(ring) == "abcdef");
}

TEST_CASE("DMA RX ring returns bytes up to the DMA write position, then -1")
{
    DmaRxRing<8> rx;
    std::memcpy(rx.buffer(), "abc", 3);
    // NDTR counts down from 8 as the DMA writes; 5 left means 3 written.
    CHECK(rx.available(8) == 0);
    CHECK(rx.available(5) == 3);
    CHECK(rx.getc(5) == 'a');
    CHECK(rx.getc(5) == 'b');
    CHECK(rx.getc(5) == 'c');
    CHECK(rx.getc(5) == -1);
}

TEST_CASE("DMA RX ring follows the circular wrap, and high bytes are not -1")
{
    DmaRxRing<4> rx;
    rx.buffer()[0] = 'x';
    rx.buffer()[1] = 'y';
    rx.buffer()[2] = 'z';
    rx.buffer()[3] = 0xFF;
    CHECK(rx.getc(1) == 'x');
    CHECK(rx.getc(1) == 'y');
    CHECK(rx.getc(1) == 'z');
    // The DMA wrote index 3 and wrapped: NDTR reloaded to 4.
    rx.buffer()[0] = 'q';
    CHECK(rx.available(3) == 2);
    CHECK(rx.getc(3) == 0xFF);
    CHECK(rx.getc(3) == 'q');
    CHECK(rx.getc(3) == -1);
}
