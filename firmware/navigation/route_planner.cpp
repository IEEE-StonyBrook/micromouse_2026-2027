#include "navigation/route_planner.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <limits>
#include <queue>
#include <string>

#include "common/log.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"

namespace
{
struct ExploredNode
{
    Cell* cell;
    float g_cost;
    float f_cost;

    bool operator<(const ExploredNode& other) const { return f_cost > other.f_cost; }
};

float manhattan(Cell* from, Cell* to)
{
    return static_cast<float>(std::abs(from->x() - to->x()) + std::abs(from->y() - to->y()));
}

std::vector<Cell*> reconstructExploredPath(const std::vector<std::vector<Cell*>>& parents,
                                           Cell* start, Cell* end)
{
    std::vector<Cell*> path;
    Cell*              node = end;

    while (node != nullptr && node != start)
    {
        path.push_back(node);
        node = parents[node->x()][node->y()];
    }

    if (node != start)
    {
        LOG_ERROR("Explored speed path reconstruction failed.");
        return {};
    }

    std::reverse(path.begin(), path.end());

    for (Cell* cell : path)
    {
        if (!cell->explored())
        {
            LOG_ERROR("Explored speed path rejected unexplored cell (" + std::to_string(cell->x()) +
                      "," + std::to_string(cell->y()) + ")");
            return {};
        }
    }

    return path;
}

std::vector<Cell*> exploredPathTo(MazeMouse* maze_mouse, Cell* end)
{
    Cell* start = maze_mouse->currentCell();
    if (start == nullptr || end == nullptr)
        return {};

    if (!start->explored() || !end->explored())
    {
        LOG_ERROR("Explored speed path requires explored start and goal cells.");
        return {};
    }

    if (start == end)
        return {};

    const int width  = maze_mouse->mazeWidth();
    const int height = maze_mouse->mazeHeight();

    std::vector<std::vector<float>> g_costs(
        width, std::vector<float>(height, std::numeric_limits<float>::infinity()));
    std::vector<std::vector<Cell*>>   parents(width, std::vector<Cell*>(height, nullptr));
    std::vector<std::vector<bool>>    closed(width, std::vector<bool>(height, false));
    std::priority_queue<ExploredNode> open;

    g_costs[start->x()][start->y()] = 0.0f;
    open.push({start, 0.0f, manhattan(start, end)});

    while (!open.empty())
    {
        ExploredNode current = open.top();
        open.pop();

        if (current.cell == end)
            return reconstructExploredPath(parents, start, end);

        if (closed[current.cell->x()][current.cell->y()])
            continue;
        closed[current.cell->x()][current.cell->y()] = true;

        for (Cell* neighbor : maze_mouse->cellNeighbors(current.cell, /*include_diagonal=*/false))
        {
            if (!neighbor->explored())
                continue;
            if (closed[neighbor->x()][neighbor->y()])
                continue;
            if (!maze_mouse->canMoveBetween(current.cell, neighbor, /*diagonals=*/false))
                continue;

            const float new_g = current.g_cost + 1.0f;
            if (new_g < g_costs[neighbor->x()][neighbor->y()])
            {
                g_costs[neighbor->x()][neighbor->y()] = new_g;
                parents[neighbor->x()][neighbor->y()] = current.cell;
                open.push({neighbor, new_g, new_g + manhattan(neighbor, end)});
            }
        }
    }

    LOG_ERROR("No explored-only speed path found.");
    return {};
}
} // namespace

std::vector<Cell*> shortestExploredPath(MazeMouse*                             maze_mouse,
                                        const std::vector<std::array<int, 2>>& goals)
{
    std::vector<Cell*> best_path;
    float              best_cost = std::numeric_limits<float>::infinity();

    for (const auto& goal : goals)
    {
        Cell* goal_cell = maze_mouse->cellAt(goal[0], goal[1]);
        if (goal_cell == nullptr || !goal_cell->explored())
            continue;

        std::vector<Cell*> path = exploredPathTo(maze_mouse, goal_cell);
        if (path.empty())
            continue;

        float cost = static_cast<float>(path.size());
        if (cost < best_cost)
        {
            best_path = path;
            best_cost = cost;
        }
    }

    return best_path;
}

namespace
{
// Cardinal headings in turn order, so +1 is a right turn: N, E, S, W.
constexpr std::array<std::array<int, 2>, 4> kHeadings = {{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};

int headingIndex(const std::array<int, 2>& heading)
{
    for (int i = 0; i < 4; ++i)
    {
        if (kHeadings[i] == heading)
            return i;
    }
    return -1;
}

struct TimedNode
{
    int   state;
    float time_s;

    bool operator<(const TimedNode& other) const { return time_s > other.time_s; }
};
} // namespace

std::vector<Cell*> fastestExploredPath(MazeMouse*                             maze_mouse,
                                       const std::vector<std::array<int, 2>>& goals,
                                       const RunTimeModel&                    model)
{
    Cell* start = maze_mouse->currentCell();
    if (start == nullptr || !start->explored())
        return {};
    const int start_heading = headingIndex(maze_mouse->currentDirectionArray());
    if (start_heading < 0)
        return {}; // The speed run starts on a cardinal heading.

    const int w = maze_mouse->mazeWidth();
    const int h = maze_mouse->mazeHeight();

    std::vector<bool> goal(static_cast<size_t>(w * h), false);
    for (const auto& g : goals)
    {
        Cell* cell = maze_mouse->cellAt(g[0], g[1]);
        if (cell != nullptr && cell->explored())
            goal[static_cast<size_t>(g[1] * w + g[0])] = true;
    }

    // A state is a cell and a heading, with the mouse at rest there: the
    // cardinal speed run stops after every straight and spins in place. Edges
    // are a whole straight of k cells (one merged move) or a spin, so the cost
    // of a route is exactly its estimated run time.
    auto         state_of = [w](int x, int y, int heading) { return (y * w + x) * 4 + heading; };
    const size_t states   = static_cast<size_t>(w * h * 4);
    std::vector<float> best(states, std::numeric_limits<float>::infinity());
    std::vector<int>   parent(states, -1);
    std::vector<bool>  closed(states, false);

    const float turn_90_s  = turnTimeS(90.0f, model);
    const float turn_180_s = 2.0f * turn_90_s; // An about-turn is two 90s.

    std::priority_queue<TimedNode> open;
    const int                      first = state_of(start->x(), start->y(), start_heading);
    best[first]                          = 0.0f;
    open.push({first, 0.0f});

    int end = -1;
    while (!open.empty())
    {
        const TimedNode node = open.top();
        open.pop();
        if (closed[node.state])
            continue;
        closed[node.state] = true;

        const int heading = node.state % 4;
        const int x       = (node.state / 4) % w;
        const int y       = (node.state / 4) / w;
        if (goal[static_cast<size_t>(y * w + x)])
        {
            end = node.state;
            break;
        }

        auto relax = [&](int next, float time_s)
        {
            if (!closed[next] && time_s < best[next])
            {
                best[next]   = time_s;
                parent[next] = node.state;
                open.push({next, time_s});
            }
        };

        relax(state_of(x, y, (heading + 1) % 4), node.time_s + turn_90_s);
        relax(state_of(x, y, (heading + 3) % 4), node.time_s + turn_90_s);
        relax(state_of(x, y, (heading + 2) % 4), node.time_s + turn_180_s);

        Cell* from = maze_mouse->cellAt(x, y);
        for (int k = 1;; ++k)
        {
            const int nx = x + kHeadings[heading][0] * k;
            const int ny = y + kHeadings[heading][1] * k;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                break;
            Cell* to = maze_mouse->cellAt(nx, ny);
            if (!to->explored() || !maze_mouse->canMoveBetween(from, to, /*diagonals=*/false))
                break;
            relax(state_of(nx, ny, heading), node.time_s + straightTimeS(k, model));
            from = to;
        }
    }

    if (end < 0)
    {
        LOG_ERROR("No explored-only speed path found.");
        return {};
    }

    // Walk the states back to the start, then lay out every cell each
    // straight passes through.
    std::vector<int> chain;
    for (int s = end; s != -1; s = parent[s])
        chain.push_back(s);
    std::reverse(chain.begin(), chain.end());

    std::vector<Cell*> path;
    for (size_t i = 1; i < chain.size(); ++i)
    {
        int       x       = (chain[i - 1] / 4) % w;
        int       y       = (chain[i - 1] / 4) / w;
        const int to_x    = (chain[i] / 4) % w;
        const int to_y    = (chain[i] / 4) / w;
        const int heading = chain[i] % 4;
        while (x != to_x || y != to_y)
        {
            x += kHeadings[heading][0];
            y += kHeadings[heading][1];
            path.push_back(maze_mouse->cellAt(x, y));
        }
    }
    return path;
}

std::vector<Cell*> plannedExploredPath(MazeMouse*                             maze_mouse,
                                       const std::vector<std::array<int, 2>>& goals,
                                       RoutePlanner                           planner)
{
    if (planner == RoutePlanner::RunTime)
        return fastestExploredPath(maze_mouse, goals);
    return shortestExploredPath(maze_mouse, goals);
}

const char* routePlannerName(RoutePlanner planner)
{
    return planner == RoutePlanner::RunTime ? "time" : "length";
}
