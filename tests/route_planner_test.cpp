// Host test for the speed-run route planners.
#include <array>
#include <string>
#include <vector>

#include "doctest/doctest.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "navigation/route_planner.h"
#include "navigation/run_time.h"

// Fixed numbers, not config/motion.h, so retuning the mouse leaves the worked
// examples below valid: 1500 mm/s at 2500 mm/s^2, spins at 270 deg/s, 1200 deg/s^2.
static RunTimeModel model()
{
    RunTimeModel m;
    m.cell_mm           = 180.0f;
    m.speed_mmps        = 1500.0f;
    m.accel_mmps2       = 2500.0f;
    m.turn_omega_degps  = 270.0f;
    m.turn_alpha_degps2 = 1200.0f;
    return m;
}

// A 5x5 maze, every cell explored, walls everywhere except two routes from
// (0,0) facing north to the goal (3,3):
//   staircase: N E N E N E, 6 cells and 5 turns;
//   detour:    north up x=0 to y=4, east to x=4, south to y=3, west to (3,3),
//              10 cells and 3 turns.
struct TwoRouteMaze
{
    Maze      maze{5, 5};
    MazeMouse mouse{{0, 0}, "n", {{3, 3}}, &maze};

    TwoRouteMaze()
    {
        for (int x = 0; x < 5; ++x)
            for (int y = 0; y < 5; ++y)
            {
                Cell* c = maze.cell(x, y);
                for (char d : {'N', 'E', 'S', 'W'})
                    c->setWall(d);
                c->markExplored();
            }
        const char* staircase = "NENENE";
        const char* detour    = "NNNNEEEESW";
        open(staircase);
        open(detour);
        mouse.setMazeMask(MASK_CLOSED);
    }

    void open(const char* moves)
    {
        int x = 0, y = 0;
        for (const char* m = moves; *m; ++m)
        {
            maze.cell(x, y)->setExit(*m);
            x += *m == 'E' ? 1 : *m == 'W' ? -1 : 0;
            y += *m == 'N' ? 1 : *m == 'S' ? -1 : 0;
        }
    }
};

static std::string cellsText(const std::vector<Cell*>& path)
{
    std::string s;
    for (Cell* c : path)
        s += "(" + std::to_string(c->x()) + "," + std::to_string(c->y()) + ")";
    return s;
}

TEST_CASE("route planner: time planner prefers fewer turns")
{
    TwoRouteMaze             m;
    const std::vector<Cell*> path = fastestExploredPath(&m.mouse, {{3, 3}}, model());
    CHECK(cellsText(path) == "(0,1)(0,2)(0,3)(0,4)(1,4)(2,4)(3,4)(4,4)(4,3)(3,3)");
}

TEST_CASE("route planner: length planner takes the short zigzag")
{
    TwoRouteMaze             m;
    const std::vector<Cell*> path = shortestExploredPath(&m.mouse, {{3, 3}});
    CHECK(cellsText(path) == "(0,1)(1,1)(1,2)(2,2)(2,3)(3,3)");
}

TEST_CASE("route planner: time route is estimated faster")
{
    TwoRouteMaze             m;
    const std::array<int, 2> heading = m.mouse.currentDirectionArray();
    Cell*                    start   = m.mouse.currentCell();
    const float              by_time =
        pathRunTimeS(start, heading, fastestExploredPath(&m.mouse, {{3, 3}}, model()), model());
    const float by_length =
        pathRunTimeS(start, heading, shortestExploredPath(&m.mouse, {{3, 3}}), model());
    CHECK(by_time < by_length);
}

TEST_CASE("route planner: unreachable goal gives an empty path")
{
    TwoRouteMaze m;
    CHECK(fastestExploredPath(&m.mouse, {{2, 0}}, model()).empty());
}
