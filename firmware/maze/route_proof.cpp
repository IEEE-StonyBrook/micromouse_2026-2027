#include "maze/route_proof.h"

#include <queue>

#include "maze/maze.h"
#include "maze/maze_mouse.h"

namespace
{
enum class Unknown
{
    Closed, // explored cells and known exits only, as the speed run plans
    Open,   // any cell; only a known wall blocks
};

using Grid = std::vector<std::vector<int>>;

char sideToward(const Cell* from, const Cell* to)
{
    const int dx = to->x() - from->x();
    const int dy = to->y() - from->y();
    if (dy == 1)
        return 'N';
    if (dx == 1)
        return 'E';
    if (dy == -1)
        return 'S';
    return 'W';
}

bool passable(const Cell* from, const Cell* to, Unknown unknown)
{
    const WallState side = from->wallState(sideToward(from, to));
    if (unknown == Unknown::Closed)
        return side == EXIT && from->explored() && to->explored();
    return side == EXIT || side == UNKNOWN;
}

// Breadth-first cells-entered distance from every source; walls are shared,
// so the same grid serves either direction of travel.
Grid distances(MazeMouse& maze, const std::vector<std::array<int, 2>>& sources, Unknown unknown)
{
    const int width  = maze.mazeWidth();
    const int height = maze.mazeHeight();
    Grid      dist(width, std::vector<int>(height, RouteBounds::NO_ROUTE));

    std::queue<Cell*> queue;
    for (const auto& source : sources)
    {
        Cell* cell = maze.cellAt(source[0], source[1]);
        if (cell == nullptr || dist[source[0]][source[1]] == 0)
            continue;
        if (unknown == Unknown::Closed && !cell->explored())
            continue;
        dist[source[0]][source[1]] = 0;
        queue.push(cell);
    }

    while (!queue.empty())
    {
        Cell* cell = queue.front();
        queue.pop();
        for (Cell* next : maze.cellNeighbors(cell, /*include_diagonal=*/false))
        {
            if (dist[next->x()][next->y()] != RouteBounds::NO_ROUTE)
                continue;
            if (!passable(cell, next, unknown))
                continue;
            dist[next->x()][next->y()] = dist[cell->x()][cell->y()] + 1;
            queue.push(next);
        }
    }
    return dist;
}

int nearestGoal(const Grid& from_start, const std::vector<std::array<int, 2>>& goals)
{
    int best = RouteBounds::NO_ROUTE;
    for (const auto& goal : goals)
    {
        if (goal[0] < 0 || goal[0] >= static_cast<int>(from_start.size()) || goal[1] < 0 ||
            goal[1] >= static_cast<int>(from_start[0].size()))
            continue;
        const int d = from_start[goal[0]][goal[1]];
        if (d != RouteBounds::NO_ROUTE && (best == RouteBounds::NO_ROUTE || d < best))
            best = d;
    }
    return best;
}
} // namespace

RouteBounds routeBounds(MazeMouse& maze, const std::array<int, 2>& start,
                        const std::vector<std::array<int, 2>>& goals)
{
    RouteBounds bounds;
    bounds.known_cells      = nearestGoal(distances(maze, {start}, Unknown::Closed), goals);
    bounds.optimistic_cells = nearestGoal(distances(maze, {start}, Unknown::Open), goals);
    return bounds;
}

std::vector<std::array<int, 2>> unprovenRouteCells(MazeMouse& maze, const std::array<int, 2>& start,
                                                   const std::vector<std::array<int, 2>>& goals)
{
    const Grid from_start = distances(maze, {start}, Unknown::Open);
    const int  best       = nearestGoal(from_start, goals);
    if (best == RouteBounds::NO_ROUTE)
        return {};
    const Grid to_goal = distances(maze, goals, Unknown::Open);

    // A cell lies on some best route when its distance from the start plus
    // its distance to the nearest goal equals the best route length.
    std::vector<std::array<int, 2>> cells;
    for (int x = 0; x < maze.mazeWidth(); ++x)
    {
        for (int y = 0; y < maze.mazeHeight(); ++y)
        {
            const int ds = from_start[x][y];
            const int dg = to_goal[x][y];
            if (ds == RouteBounds::NO_ROUTE || dg == RouteBounds::NO_ROUTE || ds + dg != best)
                continue;
            Cell* cell = maze.cellAt(x, y);
            if (cell != nullptr && !cell->explored())
                cells.push_back({x, y});
        }
    }
    return cells;
}

bool keepProving(const RouteBounds& bounds, int legs, uint32_t elapsed_ms,
                 const RouteProofCaps& caps)
{
    if (bounds.optimistic_cells == RouteBounds::NO_ROUTE || bounds.proven())
        return false;
    return legs < caps.max_legs && elapsed_ms < caps.max_ms;
}
