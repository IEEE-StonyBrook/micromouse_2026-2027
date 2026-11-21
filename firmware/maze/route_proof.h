#ifndef MAZE_ROUTE_PROOF_H
#define MAZE_ROUTE_PROOF_H

#include <array>
#include <cstdint>
#include <vector>

class MazeMouse;

/**
 * @brief Length of the best start-to-goal route under two views of the
 *        unknown walls, in cells entered (the cost the speed run minimises).
 *
 * `known_cells` is the route the speed run can take now: explored cells only,
 * through known exits. `optimistic_cells` treats every unknown wall as open,
 * so no route the maze could hide is shorter. When the two agree, no
 * unexplored cell can shorten the speed run and the search may stop.
 */
struct RouteBounds
{
    static constexpr int NO_ROUTE = -1;

    int known_cells      = NO_ROUTE;
    int optimistic_cells = NO_ROUTE;

    bool proven() const { return known_cells != NO_ROUTE && known_cells == optimistic_cells; }
};

/// Limits on exploring to prove the route. A leg is one cell driven while
/// proving; the time is the elapsed proof time in ms; a stall is the legs
/// driven since the known route last got shorter. Any cap ends the proof with
/// the best known route.
struct RouteProofCaps
{
    int      max_legs;
    uint32_t max_ms;
    int      max_stall_legs;
};

RouteBounds routeBounds(MazeMouse& maze, const std::array<int, 2>& start,
                        const std::vector<std::array<int, 2>>& goals);

/// Unexplored cells on the optimistic best route that has the fewest of them:
/// the cells still worth visiting. Empty once the route is proven or no route
/// exists.
std::vector<std::array<int, 2>> unprovenRouteCells(MazeMouse& maze, const std::array<int, 2>& start,
                                                   const std::vector<std::array<int, 2>>& goals);

/// Mark explored every unexplored cell whose four walls are already known
/// from its neighbours: it has nothing left to sense, and the speed run, which
/// plans over explored cells only, may then use it. Returns how many changed.
int markKnownCellsExplored(MazeMouse& maze);

/// True while the route is unproven, an optimistic route exists, and no cap
/// is spent.
bool keepProving(const RouteBounds& bounds, int legs, uint32_t elapsed_ms, int stall_legs,
                 const RouteProofCaps& caps);

#endif
