// Host test for the search-stop proof. Build and run: make -C tests
#include <algorithm>
#include <array>
#include <cstdio>
#include <vector>

#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "maze/route_proof.h"

static int failures = 0;

#define CHECK(cond)                                                                                \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                            \
            ++failures;                                                                            \
        }                                                                                          \
    } while (0)

using Coord = std::array<int, 2>;

// Visit a cell as the search does: every side becomes a known wall or exit.
static void visit(Maze& maze, int x, int y, const char* walls)
{
    Cell*             cell     = maze.cell(x, y);
    static const char kSides[] = {'N', 'E', 'S', 'W'};
    for (int i = 0; i < 4; ++i)
    {
        if (walls[i] == '1')
            cell->setWall(kSides[i]);
        else
            cell->setExit(kSides[i]);
    }
    cell->markExplored();
}

static bool contains(const std::vector<Coord>& cells, int x, int y)
{
    return std::find(cells.begin(), cells.end(), Coord{x, y}) != cells.end();
}

// 3 wide, 2 tall, start (0,0), goal (2,0). The short route runs east along
// the bottom row (2 cells); the long one goes up, across and down (4 cells).
//
//   +---+---+---+
//   | 1   1   1 |
//   +   +   +   +      (the wall under (1,1) is closed)
//   | S   ?   G |
//   +---+---+---+
static void unexploredCellWithKnownExitsIsNotProof()
{
    Maze      maze(2, 3);
    MazeMouse mm({0, 0}, "n", {{2, 0}}, &maze);
    // N E S W, '1' = wall.
    visit(maze, 0, 0, "0011");
    visit(maze, 0, 1, "1001");
    visit(maze, 1, 1, "1010");
    visit(maze, 2, 1, "1100");
    visit(maze, 2, 0, "0110");

    // (1,0) has every side known from its neighbours but was never entered,
    // and the speed run only plans over explored cells.
    RouteBounds bounds = routeBounds(mm, {0, 0}, {{2, 0}});
    CHECK(bounds.known_cells == 4);
    CHECK(bounds.optimistic_cells == 2);
    CHECK(!bounds.proven());

    std::vector<Coord> todo = unprovenRouteCells(mm, {0, 0}, {{2, 0}});
    CHECK(todo.size() == 1);
    CHECK(contains(todo, 1, 0));

    visit(maze, 1, 0, "1010");
    bounds = routeBounds(mm, {0, 0}, {{2, 0}});
    CHECK(bounds.known_cells == 2);
    CHECK(bounds.optimistic_cells == 2);
    CHECK(bounds.proven());
    CHECK(unprovenRouteCells(mm, {0, 0}, {{2, 0}}).empty());
}

// Only the start explored: unknown walls closed give no route at all, while
// unknown walls open give a 4-cell corner-to-corner staircase.
static void freshMazeIsUnproven()
{
    Maze      maze(3, 3);
    MazeMouse mm({0, 0}, "n", {{2, 2}}, &maze);
    visit(maze, 0, 0, "0111");

    RouteBounds bounds = routeBounds(mm, {0, 0}, {{2, 2}});
    CHECK(bounds.known_cells == RouteBounds::NO_ROUTE);
    CHECK(bounds.optimistic_cells == 4);
    CHECK(!bounds.proven());

    // One staircase's four unexplored cells. The start's east wall is known,
    // so the route leaves north and never touches the bottom row.
    std::vector<Coord> todo = unprovenRouteCells(mm, {0, 0}, {{2, 2}});
    CHECK(todo.size() == 4);
    CHECK(!contains(todo, 0, 0));
    CHECK(!contains(todo, 1, 0));
    CHECK(!contains(todo, 2, 0));
    CHECK(contains(todo, 0, 1));
    CHECK(contains(todo, 2, 2));
}

// Of the tied best routes, the one with the fewest unexplored cells is the
// one worth checking: here east along the bottom, then north, leaving only
// the goal itself.
static void fewestUnexploredRouteIsChosen()
{
    Maze      maze(3, 3);
    MazeMouse mm({0, 0}, "n", {{2, 2}}, &maze);
    visit(maze, 0, 0, "0011");
    visit(maze, 1, 0, "1010");
    visit(maze, 2, 0, "0110");
    visit(maze, 2, 1, "0100");

    const std::vector<Coord> todo = unprovenRouteCells(mm, {0, 0}, {{2, 2}});
    CHECK(todo.size() == 1);
    CHECK(contains(todo, 2, 2));
}

// Any goal cell of a block ends the route.
static void nearestGoalCellCounts()
{
    Maze      maze(2, 3);
    MazeMouse mm({0, 0}, "n", {{2, 0}, {2, 1}}, &maze);
    visit(maze, 0, 0, "0011");
    visit(maze, 0, 1, "1001");
    visit(maze, 1, 1, "1010");
    visit(maze, 2, 1, "1100");
    visit(maze, 2, 0, "0110");
    visit(maze, 1, 0, "1110"); // closed to the east now

    const RouteBounds bounds = routeBounds(mm, {0, 0}, {{2, 0}, {2, 1}});
    CHECK(bounds.known_cells == 3);
    CHECK(bounds.proven());
}

// A cell whose four sides were all seen from its neighbours has nothing left
// to sense, so it counts as explored without a visit, and the proof that
// failed for the unentered cell above now holds.
static void fullyWalledCellNeedsNoVisit()
{
    Maze      maze(2, 3);
    MazeMouse mm({0, 0}, "n", {{2, 0}}, &maze);
    visit(maze, 0, 0, "0011");
    visit(maze, 0, 1, "1001");
    visit(maze, 1, 1, "1010");
    visit(maze, 2, 1, "1100");
    // (2,0)'s south and east are the maze edge, known from the start.
    maze.cell(2, 0)->setWall('S');
    maze.cell(2, 0)->setWall('E');

    // (1,0): N from (1,1), W from (0,0), but S and E unseen. (2,0): N, E, S
    // known, W unseen. Neither is complete yet.
    CHECK(markKnownCellsExplored(mm) == 0);

    maze.cell(1, 0)->setWall('S');
    maze.cell(1, 0)->setExit('E');
    CHECK(markKnownCellsExplored(mm) == 2);
    CHECK(maze.cell(1, 0)->explored());
    CHECK(maze.cell(2, 0)->explored());
    CHECK(routeBounds(mm, {0, 0}, {{2, 0}}).proven());
    CHECK(markKnownCellsExplored(mm) == 0);
}

static void capsStopTheProof()
{
    const RouteProofCaps caps{/*max_legs=*/10, /*max_ms=*/1000};
    RouteBounds          open{/*known_cells=*/6, /*optimistic_cells=*/4};
    RouteBounds          done{/*known_cells=*/4, /*optimistic_cells=*/4};

    CHECK(keepProving(open, 0, 0, caps));
    CHECK(keepProving(open, 9, 999, caps));
    CHECK(!keepProving(done, 0, 0, caps));
    CHECK(!keepProving(open, 10, 0, caps));
    CHECK(!keepProving(open, 0, 1000, caps));
    // No optimistic route at all means nothing left to explore.
    CHECK(!keepProving(RouteBounds{}, 0, 0, caps));
}

int main()
{
    unexploredCellWithKnownExitsIsNotProof();
    freshMazeIsUnproven();
    fewestUnexploredRouteIsChosen();
    nearestGoalCellCounts();
    fullyWalledCellNeedsNoVisit();
    capsStopTheProof();
    if (failures == 0)
        std::printf("PASS: route proof\n");
    return failures == 0 ? 0 : 1;
}
