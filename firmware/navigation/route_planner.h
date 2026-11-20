#ifndef NAVIGATION_ROUTE_PLANNER_H
#define NAVIGATION_ROUTE_PLANNER_H

#include <array>
#include <vector>

class Cell;
class MazeMouse;

// Fewest-cells cardinal route from the mouse's cell to the nearest explored
// goal, through explored cells only. Empty if there is none.
std::vector<Cell*> shortestExploredPath(MazeMouse*                             maze_mouse,
                                        const std::vector<std::array<int, 2>>& goals);

#endif
