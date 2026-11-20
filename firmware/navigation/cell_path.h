#ifndef NAVIGATION_CELL_PATH_H
#define NAVIGATION_CELL_PATH_H

#include <algorithm>
#include <array>
#include <vector>

#include "config/geometry.h"

class Cell;

/**
 * @brief A route of cells in a fixed buffer, so planning never touches the heap.
 *
 * Holds up to one entry per cell of the largest maze: a shortest route never
 * enters a cell twice.
 */
class CellPath
{
  public:
    static constexpr int CAPACITY = MAZE_SIZE * MAZE_SIZE;

    void clear() { size_ = 0; }
    /// Appends a cell; false (and unchanged) when full.
    bool push(Cell* cell)
    {
        if (size_ >= CAPACITY)
            return false;
        cells_[size_++] = cell;
        return true;
    }
    void reverse() { std::reverse(cells_.begin(), cells_.begin() + size_); }

    int   size() const { return size_; }
    bool  empty() const { return size_ == 0; }
    Cell* operator[](int i) const { return cells_[i]; }

    std::vector<Cell*> toVector() const
    {
        return std::vector<Cell*>(cells_.begin(), cells_.begin() + size_);
    }

  private:
    std::array<Cell*, CAPACITY> cells_{};
    int                         size_ = 0;
};

#endif
