/**
 * @file main.cpp
 * @brief Micromouse Three-Phase Competition Simulator
 *
 * Simulates a competition run with mms (Micromouse Simulator):
 *   1. Search: iterative A* without diagonals to the goal
 *   2. Return: iterative A* without diagonals back to start
 *   3. Speed Run: diagonalized A* for optimal time
 *
 * Usage: Micromouse.out [--planner length|time] ["<size> <goals>"]. The
 * setting is the same text as the firmware's MAZE command, e.g. "16 7,7..8,8".
 * Without it the bot assumes a 16x16 maze with goal (2, 2). mms cannot report
 * goal cells, so the caller passes them. --planner picks how the speed run
 * chooses its route: fewest cells (length, the default) or least estimated
 * run time (time).
 */

#include <array>
#include <cstring>
#include <string>
#include <vector>

#include "app/mouse.h"
#include "common/log.h"
#include "config/geometry.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "maze/maze_setting.h"
#include "navigation/path_utils.h"

int main(int argc, char** argv)
{
    LOG_INFO("=== MICROMOUSE COMPETITION SIMULATOR ===\n");

    MazeSetting setting;
    setting.size  = MAZE_SIZE;
    setting.goals = {{2, 2}};
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--planner") == 0)
        {
            const std::string name = i + 1 < argc ? argv[++i] : "";
            if (name == "length")
                PathUtils::set_route_planner(RoutePlanner::PathLength);
            else if (name == "time")
                PathUtils::set_route_planner(RoutePlanner::RunTime);
            else
            {
                LOG_ERROR("--planner takes length or time, not \"" + name + "\"");
                return 2;
            }
            continue;
        }
        std::string error;
        if (!parseMazeSetting(argv[i], MAZE_SIZE, setting, error))
        {
            LOG_ERROR("Bad maze setting \"" + std::string(argv[i]) + "\": " + error);
            return 2;
        }
    }
    LOG_INFO("Maze " + mazeSettingText(setting));
    LOG_INFO(std::string("Route planner: ") + routePlannerName(PathUtils::routePlanner()));

    std::array<int, 2>                     start = {0, 0};
    const std::vector<std::array<int, 2>>& goals = setting.goals;

    Maze      maze(setting.size, setting.size);
    MazeMouse maze_mouse(start, "n", goals, &maze);
    Mouse     mouse(&maze_mouse);
    mouse.run_on_simulator = true;
    mouse.setUp(start, goals);

    // Phase 1: SEARCH - Iterative A* to the goal, sensing walls and replanning
    // whenever the path reaches an unexplored cell.
    LOG_INFO("Phase 1: Search to Goal (Iterative A* without diagonals)");
    mouse.setPhaseColor('y');
    PathUtils::traversePath(&mouse, &maze_mouse, goals, /*diagonals=*/false,
                            /*all_explored=*/false, /*avoid_goals=*/false);
    LOG_INFO("Reached goal\n");

    // Phase 2: RETURN - Go back to start using iterative A* with no diagonals.
    LOG_INFO("Phase 2: Return to Start (Iterative A* without diagonals)");
    mouse.clearAllColor();
    mouse.setPhaseColor('c');
    PathUtils::traversePath(&mouse, &maze_mouse, {start}, /*diagonals=*/false,
                            /*all_explored=*/false, /*avoid_goals=*/false);
    LOG_INFO("Returned to start\n");

    // Phase 3: SPEED RUN - Diagonalized A* over the discovered wall map.
    LOG_INFO("Phase 3: Speed Run (Diagonalized A*)");
    mouse.setPhaseColor('G');
    PathUtils::traverseExploredDiagonalPath(&mouse, &maze_mouse, goals);
    LOG_INFO("Speed run complete\n");

    LOG_INFO("=== SIMULATION COMPLETE ===");
    return 0;
}
