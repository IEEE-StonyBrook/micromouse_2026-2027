#ifndef NAVIGATION_ROUTE_PLANNER_H
#define NAVIGATION_ROUTE_PLANNER_H

#include <array>
#include <vector>

#include "navigation/run_time.h"

class Cell;
class MazeMouse;

// How a speed run picks its route through explored cells.
enum class RoutePlanner
{
    PathLength, // fewest cells (the default)
    RunTime,    // least estimated run time under RunTimeModel
};

// Fewest-cells cardinal route from the mouse's cell to the nearest explored
// goal, through explored cells only. Empty if there is none.
std::vector<Cell*> shortestExploredPath(MazeMouse*                             maze_mouse,
                                        const std::vector<std::array<int, 2>>& goals);

// Least estimated run time cardinal route from the mouse's cell and heading to
// any explored goal, through explored cells only: straights merge, so a longer
// route with fewer turns can win. Empty if there is none.
std::vector<Cell*> fastestExploredPath(MazeMouse*                             maze_mouse,
                                       const std::vector<std::array<int, 2>>& goals,
                                       const RunTimeModel&                    model = {});

// The route the given planner picks.
std::vector<Cell*> plannedExploredPath(MazeMouse*                             maze_mouse,
                                       const std::vector<std::array<int, 2>>& goals,
                                       RoutePlanner                           planner);

const char* routePlannerName(RoutePlanner planner);

#endif
