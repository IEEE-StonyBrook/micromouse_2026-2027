#ifndef NAVIGATION_STATIC_HEAP_H
#define NAVIGATION_STATIC_HEAP_H

#include <algorithm>
#include <array>
#include <cstddef>

/**
 * @brief A max-heap on `operator<` in a fixed array: std::priority_queue
 * without the heap allocation.
 *
 * push() and pop() run std::push_heap and std::pop_heap exactly as
 * std::priority_queue does, so the same pushes pop in the same order, ties
 * included, and a planner moved onto it finds the same routes.
 */
template <typename T, size_t N> class StaticHeap
{
  public:
    bool     empty() const { return size_ == 0; }
    size_t   size() const { return size_; }
    const T& top() const { return items_[0]; }

    /// False (and unchanged) when full.
    bool push(const T& item)
    {
        if (size_ >= N)
            return false;
        items_[size_++] = item;
        std::push_heap(items_.begin(), items_.begin() + size_);
        return true;
    }

    void pop()
    {
        std::pop_heap(items_.begin(), items_.begin() + size_);
        --size_;
    }

    void clear() { size_ = 0; }

    /// Drops every entry `stale` picks and rebuilds the heap. A lazy-deletion
    /// search calls it when full: its stale entries can never be expanded.
    template <typename Stale> void prune(Stale stale)
    {
        size_ = static_cast<size_t>(std::remove_if(items_.begin(), items_.begin() + size_, stale) -
                                    items_.begin());
        std::make_heap(items_.begin(), items_.begin() + size_);
    }

  private:
    std::array<T, N> items_{};
    size_t           size_ = 0;
};

#endif
