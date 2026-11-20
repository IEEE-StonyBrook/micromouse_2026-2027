#include "navigation/a_star.h"

#include <cmath>
#include <limits>

#include "navigation/path_converter.h"
#include "navigation/static_heap.h"

namespace
{
struct AStarNode
{
    Cell* cell;
    float g_cost, f_cost;
    bool  operator<(const AStarNode& other) const { return f_cost > other.f_cost; }
};

// The search's working memory, sized for the largest maze. Planning runs on
// the main loop only, one search at a time. A cell holds at most one live
// open entry (a push only lowers its cost), so one slot per cell is enough
// once stale entries are pruned.
float                                     g_costs[CellPath::CAPACITY];
StaticHeap<AStarNode, CellPath::CAPACITY> open_set;
CellPath                                  candidate_path;
CellPath                                  best_path;
} // namespace

AStar::AStar(MazeMouse* mouse) : mouse_(mouse)
{
}

std::string AStar::pathTo(const std::vector<std::array<int, 2>>& goals, bool diagonals,
                          bool pass_goals)
{
    std::vector<Cell*> path = cellPath(goals, diagonals, pass_goals);
    LOG_DEBUG(pathToString(path));

    std::string lfr =
        PathConverter::buildLFR(mouse_->currentCell(), mouse_->currentDirectionArray(), path);
    LOG_DEBUG(lfr);
    return lfr;
}

std::vector<Cell*> AStar::cellPath(const std::vector<std::array<int, 2>>& goals, bool diagonals,
                                   bool pass_goals)
{
    return route(goals, diagonals, pass_goals).toVector();
}

const CellPath& AStar::route(const std::vector<std::array<int, 2>>& goals, bool diagonals,
                             bool pass_goals)
{
    best_path.clear();
    float best_cost = std::numeric_limits<float>::infinity();

    for (const auto& goal : goals)
    {
        if (!findPathTo(mouse_->cellAt(goal[0], goal[1]), diagonals, pass_goals, candidate_path))
            continue;
        if (candidate_path.empty())
            continue;

        if (total_cost_ < best_cost)
        {
            best_path = candidate_path;
            best_cost = total_cost_;
        }
    }

    return best_path;
}

bool AStar::findPathTo(Cell* end, bool diagonals, bool pass_goals, CellPath& out)
{
    out.clear();
    mouse_->resetPathfinding();

    Cell* start = mouse_->currentCell();
    int   w     = mouse_->mazeWidth();
    int   h     = mouse_->mazeHeight();
    if (w * h > CellPath::CAPACITY)
    {
        LOG_ERROR("AStar: maze larger than MAZE_SIZE x MAZE_SIZE.");
        return false;
    }

    auto index = [h](Cell* cell) { return cell->x() * h + cell->y(); };
    for (int i = 0; i < w * h; ++i)
        g_costs[i] = std::numeric_limits<float>::infinity();
    open_set.clear();

    // A full heap still holds entries for processed cells or superseded
    // costs; dropping them leaves at most one entry per unprocessed cell.
    auto push = [&](const AStarNode& node)
    {
        if (open_set.push(node))
            return true;
        open_set.prune([&](const AStarNode& e)
                       { return e.cell->processed || e.g_cost > g_costs[index(e.cell)]; });
        return open_set.push(node);
    };

    g_costs[index(start)] = 0.0f;
    push({start, 0.0f, heuristic(start, end)});

    while (!open_set.empty())
    {
        AStarNode current = open_set.top();
        open_set.pop();

        if (current.cell == end)
        {
            total_cost_ = current.f_cost;
            reconstructPath(start, end, out);
            return true;
        }

        if (current.cell->processed)
            continue;
        current.cell->processed = true;

        std::array<Cell*, 8> neighbors;
        const int            count = mouse_->cellNeighbors(current.cell, diagonals, neighbors);
        for (int i = 0; i < count; ++i)
        {
            Cell* neighbor = neighbors[i];
            bool  is_end   = (neighbor == end);
            if (neighbor->processed || (!pass_goals && mouse_->isGoal(neighbor) && !is_end) ||
                !mouse_->canMoveBetween(current.cell, neighbor, diagonals))
            {
                continue;
            }

            float new_g = current.g_cost + heuristic(current.cell, neighbor);
            if (neighbor->explored())
                new_g += explored_cell_cost_;

            if (new_g < g_costs[index(neighbor)])
            {
                g_costs[index(neighbor)] = new_g;
                neighbor->parent         = current.cell;
                if (!push({neighbor, new_g, new_g + heuristic(neighbor, end)}))
                {
                    LOG_ERROR("AStar: open set full.");
                    mouse_->resetPathfinding();
                    return false;
                }
            }
        }
    }

    LOG_ERROR("AStar: No path found!");
    mouse_->resetPathfinding();
    return false;
}

float AStar::heuristic(Cell* from, Cell* to)
{
    float dx = std::fabs(from->x() - to->x());
    float dy = std::fabs(from->y() - to->y());
    // Octile distance
    return (dx + dy) + (std::sqrt(2.0f) - 2.0f) * std::fmin(dx, dy);
}

void AStar::reconstructPath(Cell* start, Cell* end, CellPath& out)
{
    Cell* node = end;

    while (node && node != start && out.push(node))
        node = node->parent;

    out.reverse();
    mouse_->resetPathfinding();
}

std::string AStar::pathToString(const std::vector<Cell*>& path)
{
    std::string s;
    for (size_t i = 0; i < path.size(); i++)
    {
        s += "(" + std::to_string(path[i]->x()) + "," + std::to_string(path[i]->y()) + ")";
        if (i < path.size() - 1)
            s += " -> ";
    }
    return s;
}
