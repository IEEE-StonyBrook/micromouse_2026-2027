#include "navigation/route_planner.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

#include "common/log.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "navigation/static_heap.h"

namespace
{
struct ExploredNode
{
    Cell* cell;
    float g_cost;
    float f_cost;

    bool operator<(const ExploredNode& other) const { return f_cost > other.f_cost; }
};

struct TimedNode
{
    int   state;
    float time_s;

    bool operator<(const TimedNode& other) const { return time_s > other.time_s; }
};

constexpr int MAX_CELLS  = CellPath::CAPACITY;
constexpr int MAX_STATES = MAX_CELLS * 4; // a cell and a cardinal heading

// The planners' working memory, sized for the largest maze. They run on the
// main loop only, one at a time. In both searches a push only ever lowers a
// node's cost, so each node holds at most one live open entry: one slot per
// node is enough once stale entries are pruned.
struct ExploredScratch
{
    float                               g_costs[MAX_CELLS];
    Cell*                               parents[MAX_CELLS];
    bool                                closed[MAX_CELLS];
    StaticHeap<ExploredNode, MAX_CELLS> open;
    CellPath                            candidate;
} explored;

struct TimedScratch
{
    float                             best[MAX_STATES];
    int16_t                           parent[MAX_STATES];
    bool                              closed[MAX_STATES];
    bool                              goal[MAX_CELLS];
    StaticHeap<TimedNode, MAX_STATES> open;
} timed;

float manhattan(Cell* from, Cell* to)
{
    return static_cast<float>(std::abs(from->x() - to->x()) + std::abs(from->y() - to->y()));
}

bool fitsScratch(MazeMouse* maze_mouse)
{
    if (maze_mouse->mazeWidth() * maze_mouse->mazeHeight() <= MAX_CELLS)
        return true;
    LOG_ERROR("Speed path planner: maze larger than MAZE_SIZE x MAZE_SIZE.");
    return false;
}

bool reconstructExploredPath(int height, Cell* start, Cell* end, CellPath& out)
{
    out.clear();
    Cell* node = end;

    while (node != nullptr && node != start)
    {
        if (!out.push(node))
            break;
        node = explored.parents[node->x() * height + node->y()];
    }

    if (node != start)
    {
        LOG_ERROR("Explored speed path reconstruction failed.");
        out.clear();
        return false;
    }

    out.reverse();

    for (int i = 0; i < out.size(); ++i)
    {
        Cell* cell = out[i];
        if (!cell->explored())
        {
            LOG_ERROR("Explored speed path rejected unexplored cell (" + std::to_string(cell->x()) +
                      "," + std::to_string(cell->y()) + ")");
            out.clear();
            return false;
        }
    }

    return true;
}

// Fewest-cells route to `end` into `out`; empty when there is none.
void exploredPathTo(MazeMouse* maze_mouse, Cell* end, CellPath& out)
{
    out.clear();
    Cell* start = maze_mouse->currentCell();
    if (start == nullptr || end == nullptr)
        return;

    if (!start->explored() || !end->explored())
    {
        LOG_ERROR("Explored speed path requires explored start and goal cells.");
        return;
    }

    if (start == end || !fitsScratch(maze_mouse))
        return;

    const int width  = maze_mouse->mazeWidth();
    const int height = maze_mouse->mazeHeight();
    auto      index  = [height](Cell* cell) { return cell->x() * height + cell->y(); };

    for (int i = 0; i < width * height; ++i)
    {
        explored.g_costs[i] = std::numeric_limits<float>::infinity();
        explored.parents[i] = nullptr;
        explored.closed[i]  = false;
    }
    explored.open.clear();
    auto push = [&](const ExploredNode& node)
    {
        if (explored.open.push(node))
            return true;
        explored.open.prune(
            [&](const ExploredNode& e)
            {
                return explored.closed[index(e.cell)] || e.g_cost > explored.g_costs[index(e.cell)];
            });
        return explored.open.push(node);
    };

    explored.g_costs[index(start)] = 0.0f;
    push({start, 0.0f, manhattan(start, end)});

    while (!explored.open.empty())
    {
        ExploredNode current = explored.open.top();
        explored.open.pop();

        if (current.cell == end)
        {
            reconstructExploredPath(height, start, end, out);
            return;
        }

        if (explored.closed[index(current.cell)])
            continue;
        explored.closed[index(current.cell)] = true;

        std::array<Cell*, 8> neighbors;
        const int            count =
            maze_mouse->cellNeighbors(current.cell, /*include_diagonal=*/false, neighbors);
        for (int i = 0; i < count; ++i)
        {
            Cell* neighbor = neighbors[i];
            if (!neighbor->explored())
                continue;
            if (explored.closed[index(neighbor)])
                continue;
            if (!maze_mouse->canMoveBetween(current.cell, neighbor, /*diagonals=*/false))
                continue;

            const float new_g = current.g_cost + 1.0f;
            if (new_g < explored.g_costs[index(neighbor)])
            {
                explored.g_costs[index(neighbor)] = new_g;
                explored.parents[index(neighbor)] = current.cell;
                if (!push({neighbor, new_g, new_g + manhattan(neighbor, end)}))
                {
                    LOG_ERROR("Explored speed path open set full.");
                    return;
                }
            }
        }
    }

    LOG_ERROR("No explored-only speed path found.");
}

// Cardinal headings in turn order, so +1 is a right turn: N, E, S, W.
constexpr std::array<std::array<int, 2>, 4> HEADING_STEPS = {{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};

int headingIndex(const std::array<int, 2>& heading)
{
    for (int i = 0; i < 4; ++i)
    {
        if (HEADING_STEPS[i] == heading)
            return i;
    }
    return -1;
}
} // namespace

bool shortestExploredRoute(MazeMouse* maze_mouse, const std::vector<std::array<int, 2>>& goals,
                           CellPath& out)
{
    out.clear();
    float best_cost = std::numeric_limits<float>::infinity();

    for (const auto& goal : goals)
    {
        Cell* goal_cell = maze_mouse->cellAt(goal[0], goal[1]);
        if (goal_cell == nullptr || !goal_cell->explored())
            continue;

        exploredPathTo(maze_mouse, goal_cell, explored.candidate);
        if (explored.candidate.empty())
            continue;

        float cost = static_cast<float>(explored.candidate.size());
        if (cost < best_cost)
        {
            out       = explored.candidate;
            best_cost = cost;
        }
    }

    return !out.empty();
}

std::vector<Cell*> shortestExploredPath(MazeMouse*                             maze_mouse,
                                        const std::vector<std::array<int, 2>>& goals)
{
    static CellPath path;
    shortestExploredRoute(maze_mouse, goals, path);
    return path.toVector();
}

bool fastestExploredRoute(MazeMouse* maze_mouse, const std::vector<std::array<int, 2>>& goals,
                          CellPath& out, const RunTimeModel& model)
{
    out.clear();
    Cell* start = maze_mouse->currentCell();
    if (start == nullptr || !start->explored())
        return false;
    const int start_heading = headingIndex(maze_mouse->currentDirectionArray());
    if (start_heading < 0)
        return false; // The speed run starts on a cardinal heading.
    if (!fitsScratch(maze_mouse))
        return false;

    const int w = maze_mouse->mazeWidth();
    const int h = maze_mouse->mazeHeight();

    for (int i = 0; i < w * h; ++i)
        timed.goal[i] = false;
    for (const auto& g : goals)
    {
        Cell* cell = maze_mouse->cellAt(g[0], g[1]);
        if (cell != nullptr && cell->explored())
            timed.goal[g[1] * w + g[0]] = true;
    }

    // A state is a cell and a heading, with the mouse at rest there: the
    // cardinal speed run stops after every straight and spins in place. Edges
    // are a whole straight of k cells (one merged move) or a spin, so the cost
    // of a route is exactly its estimated run time.
    auto      state_of = [w](int x, int y, int heading) { return (y * w + x) * 4 + heading; };
    const int states   = w * h * 4;
    for (int i = 0; i < states; ++i)
    {
        timed.best[i]   = std::numeric_limits<float>::infinity();
        timed.parent[i] = -1;
        timed.closed[i] = false;
    }
    timed.open.clear();

    const float turn_90_s  = turnTimeS(90.0f, model);
    const float turn_180_s = 2.0f * turn_90_s; // An about-turn is two 90s.

    bool open_full = false;
    auto push      = [&](const TimedNode& node)
    {
        if (timed.open.push(node))
            return;
        timed.open.prune([&](const TimedNode& e)
                         { return timed.closed[e.state] || e.time_s > timed.best[e.state]; });
        if (!timed.open.push(node))
            open_full = true;
    };

    const int first   = state_of(start->x(), start->y(), start_heading);
    timed.best[first] = 0.0f;
    push({first, 0.0f});

    int end = -1;
    while (!timed.open.empty() && !open_full)
    {
        const TimedNode node = timed.open.top();
        timed.open.pop();
        if (timed.closed[node.state])
            continue;
        timed.closed[node.state] = true;

        const int heading = node.state % 4;
        const int x       = (node.state / 4) % w;
        const int y       = (node.state / 4) / w;
        if (timed.goal[y * w + x])
        {
            end = node.state;
            break;
        }

        auto relax = [&](int next, float time_s)
        {
            if (!timed.closed[next] && time_s < timed.best[next])
            {
                timed.best[next]   = time_s;
                timed.parent[next] = static_cast<int16_t>(node.state);
                push({next, time_s});
            }
        };

        relax(state_of(x, y, (heading + 1) % 4), node.time_s + turn_90_s);
        relax(state_of(x, y, (heading + 3) % 4), node.time_s + turn_90_s);
        relax(state_of(x, y, (heading + 2) % 4), node.time_s + turn_180_s);

        Cell* from = maze_mouse->cellAt(x, y);
        for (int k = 1;; ++k)
        {
            const int nx = x + HEADING_STEPS[heading][0] * k;
            const int ny = y + HEADING_STEPS[heading][1] * k;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                break;
            Cell* to = maze_mouse->cellAt(nx, ny);
            if (!to->explored() || !maze_mouse->canMoveBetween(from, to, /*diagonals=*/false))
                break;
            relax(state_of(nx, ny, heading), node.time_s + straightTimeS(k, model));
            from = to;
        }
    }

    if (open_full)
    {
        LOG_ERROR("Timed speed path open set full.");
        return false;
    }
    if (end < 0)
    {
        LOG_ERROR("No explored-only speed path found.");
        return false;
    }

    // Walk the states back to the start, laying out every cell each straight
    // passes through, last cell first; then put them in driving order.
    for (int s = end; timed.parent[s] != -1; s = timed.parent[s])
    {
        const int from_x  = (timed.parent[s] / 4) % w;
        const int from_y  = (timed.parent[s] / 4) / w;
        int       x       = (s / 4) % w;
        int       y       = (s / 4) / w;
        const int heading = s % 4;
        while (x != from_x || y != from_y)
        {
            if (!out.push(maze_mouse->cellAt(x, y)))
            {
                LOG_ERROR("Timed speed path longer than the maze.");
                out.clear();
                return false;
            }
            x -= HEADING_STEPS[heading][0];
            y -= HEADING_STEPS[heading][1];
        }
    }
    out.reverse();
    return true;
}

std::vector<Cell*> fastestExploredPath(MazeMouse*                             maze_mouse,
                                       const std::vector<std::array<int, 2>>& goals,
                                       const RunTimeModel&                    model)
{
    static CellPath path;
    fastestExploredRoute(maze_mouse, goals, path, model);
    return path.toVector();
}

bool plannedExploredRoute(MazeMouse* maze_mouse, const std::vector<std::array<int, 2>>& goals,
                          RoutePlanner planner, CellPath& out)
{
    if (planner == RoutePlanner::RunTime)
        return fastestExploredRoute(maze_mouse, goals, out);
    return shortestExploredRoute(maze_mouse, goals, out);
}

std::vector<Cell*> plannedExploredPath(MazeMouse*                             maze_mouse,
                                       const std::vector<std::array<int, 2>>& goals,
                                       RoutePlanner                           planner)
{
    static CellPath path;
    plannedExploredRoute(maze_mouse, goals, planner, path);
    return path.toVector();
}

const char* routePlannerName(RoutePlanner planner)
{
    return planner == RoutePlanner::RunTime ? "time" : "length";
}
