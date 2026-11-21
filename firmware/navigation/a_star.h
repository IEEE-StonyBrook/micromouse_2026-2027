#ifndef NAVIGATION_A_STAR_H
#define NAVIGATION_A_STAR_H

#include <array>
#include <string>
#include <vector>

#include "common/log.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "navigation/cell_path.h"

class Mouse;

/**
 * @brief A* pathfinding algorithm for maze navigation
 *
 * Finds optimal paths using A* with configurable heuristics.
 * Supports both cardinal and diagonal movement modes. The search works in
 * static buffers shared by every AStar, so it never touches the heap; only
 * the std::vector and std::string results allocate.
 */
class AStar
{
  public:
    explicit AStar(MazeMouse* mouse);

    /**
     * @brief Get LFR path string to goal cells
     * @param goals Target cell coordinates
     * @param diagonals Allow diagonal movement
     * @param pass_goals Allow path through goal cells
     * @return LFR-formatted path string (e.g., "F#L#F#F#R#F")
     */
    std::string pathTo(const std::vector<std::array<int, 2>>& goals, bool diagonals = false,
                       bool pass_goals = false);

    /**
     * @brief Get cell-based path to goal cells
     * @return Vector of Cell pointers representing the path
     */
    std::vector<Cell*> cellPath(const std::vector<std::array<int, 2>>& goals,
                                bool diagonals = false, bool pass_goals = false);

    /**
     * @brief The same path as cellPath(), with no heap use. The buffer is
     * shared by every AStar and holds until the next search.
     */
    const CellPath& route(const std::vector<std::array<int, 2>>& goals, bool diagonals = false,
                          bool pass_goals = false);

    float lastPathCost() const { return total_cost_; }
    // Extra cost per explored cell, to prefer unexplored routes. 0 = shortest.
    void set_explored_cell_cost(float cost) { explored_cell_cost_ = cost; }
    // A caller that only probes whether a route exists (and has a fallback)
    // can silence the no-path error.
    void set_quiet(bool quiet) { quiet_ = quiet; }

  private:
    // False when there is no path; `out` is empty when end is the start.
    bool findPathTo(Cell* end, bool diagonals, bool pass_goals, CellPath& out);
    void reconstructPath(Cell* start, Cell* end, CellPath& out);

    static std::string pathToString(const std::vector<Cell*>& path);
    static float       heuristic(Cell* from, Cell* to);

    MazeMouse* mouse_;
    float      total_cost_ = 0.0f;

    float explored_cell_cost_ = 0.0f;
    bool  quiet_              = false;
};

#endif
