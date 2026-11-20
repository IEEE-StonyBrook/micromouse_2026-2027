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

// Breadth-first cells-entered distance from every source.
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

    // Of the tied best routes, follow the one with the fewest unexplored
    // cells: it is the cheapest to confirm, and confirming it proves the
    // route. Walk the shortest-route layers outward from the start, keeping
    // each cell's fewest unexplored cells so far and the parent that gave it.
    const int                       width  = maze.mazeWidth();
    const int                       height = maze.mazeHeight();
    std::vector<std::vector<Cell*>> layers(best + 1);
    for (int x = 0; x < width; ++x)
        for (int y = 0; y < height; ++y)
            if (from_start[x][y] != RouteBounds::NO_ROUTE && from_start[x][y] <= best)
                layers[from_start[x][y]].push_back(maze.cellAt(x, y));

    constexpr int                   kUnreached = 1 << 30;
    Grid                            unexplored(width, std::vector<int>(height, kUnreached));
    std::vector<std::vector<Cell*>> parent(width, std::vector<Cell*>(height, nullptr));
    unexplored[start[0]][start[1]] = maze.cellAt(start[0], start[1])->explored() ? 0 : 1;
    for (int d = 1; d <= best; ++d)
    {
        for (Cell* cell : layers[d])
        {
            for (Cell* prev : maze.cellNeighbors(cell, /*include_diagonal=*/false))
            {
                if (from_start[prev->x()][prev->y()] != d - 1 ||
                    !passable(prev, cell, Unknown::Open))
                    continue;
                const int count = unexplored[prev->x()][prev->y()] + (cell->explored() ? 0 : 1);
                if (count < unexplored[cell->x()][cell->y()])
                {
                    unexplored[cell->x()][cell->y()] = count;
                    parent[cell->x()][cell->y()]     = prev;
                }
            }
        }
    }

    Cell* end = nullptr;
    for (const auto& goal : goals)
    {
        Cell* cell = maze.cellAt(goal[0], goal[1]);
        if (cell == nullptr || from_start[goal[0]][goal[1]] != best)
            continue;
        if (end == nullptr || unexplored[goal[0]][goal[1]] < unexplored[end->x()][end->y()])
            end = cell;
    }

    std::vector<std::array<int, 2>> cells;
    for (Cell* cell = end; cell != nullptr; cell = parent[cell->x()][cell->y()])
        if (!cell->explored())
            cells.push_back({cell->x(), cell->y()});
    return cells;
}

int markKnownCellsExplored(MazeMouse& maze)
{
    int marked = 0;
    for (int x = 0; x < maze.mazeWidth(); ++x)
    {
        for (int y = 0; y < maze.mazeHeight(); ++y)
        {
            Cell* cell = maze.cellAt(x, y);
            if (cell == nullptr || cell->explored() || cell->hasUnknownWalls())
                continue;
            cell->markExplored();
            ++marked;
        }
    }
    return marked;
}

bool keepProving(const RouteBounds& bounds, int legs, uint32_t elapsed_ms, int stall_legs,
                 const RouteProofCaps& caps)
{
    if (bounds.optimistic_cells == RouteBounds::NO_ROUTE || bounds.proven())
        return false;
    return legs < caps.max_legs && elapsed_ms < caps.max_ms && stall_legs < caps.max_stall_legs;
}
