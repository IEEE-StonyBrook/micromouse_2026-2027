// Fixed-capacity byte FIFO. No heap, so it is safe in the UART and BLE
// callbacks' calling context.
#pragma once

#include <cstddef>
#include <cstdint>

namespace bridge
{

template <size_t N> class Ring
{
  public:
    size_t size() const { return count_; }
    size_t free() const { return N - count_; }
    bool   empty() const { return count_ == 0; }

    bool push(uint8_t byte)
    {
        if (count_ == N)
        {
            return false;
        }
        buf_[(head_ + count_) % N] = byte;
        ++count_;
        return true;
    }

    // Byte i places from the oldest; i < size().
    uint8_t at(size_t i) const { return buf_[(head_ + i) % N]; }

    void drop(size_t n)
    {
        n     = n < count_ ? n : count_;
        head_ = (head_ + n) % N;
        count_ -= n;
    }

    void clear() { drop(count_); }

  private:
    uint8_t buf_[N] = {};
    size_t  head_   = 0;
    size_t  count_  = 0;
};

} // namespace bridge
