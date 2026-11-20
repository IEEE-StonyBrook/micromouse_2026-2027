// The A* and speed-run planners on the sim maze corpus: their routes match a
// golden file recorded from the original std::vector/priority_queue planners,
// and planning allocates nothing but the path it returns.
//
// Rewrite the golden file (only after a planner change meant to change routes):
//   PLANNER_GOLDEN_OUT=planner_golden.txt ./host_tests -tc="planner*golden*"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "doctest/doctest.h"
#include "maze/maze.h"
#include "maze/maze_mouse.h"
#include "maze_text.h"
#include "navigation/a_star.h"
#include "navigation/route_planner.h"
#include "navigation/run_time.h"

// Counts every global operator new in this test binary.
static size_t g_allocations = 0;

void* operator new(std::size_t size)
{
    ++g_allocations;
    if (void* p = std::malloc(size ? size : 1))
        return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept
{
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept
{
    std::free(p);
}

namespace
{
const char* const CORPUS_DIR  = "../sim/mazes";
const char* const GOLDEN_FILE = "planner_golden.txt";

// Fixed numbers, as in route_planner_test.cpp, so retuning the mouse leaves
// the golden routes valid.
RunTimeModel model()
{
    RunTimeModel m;
    m.cell_mm           = 180.0f;
    m.speed_mmps        = 1500.0f;
    m.accel_mmps2       = 2500.0f;
    m.turn_omega_degps  = 270.0f;
    m.turn_alpha_degps2 = 1200.0f;
    return m;
}

std::vector<std::string> corpusFiles()
{
    std::vector<std::string> files;
    for (const char* sub : {"classic", "small"})
        for (const auto& entry :
             std::filesystem::directory_iterator(std::string(CORPUS_DIR) + "/" + sub))
            if (entry.path().extension() == ".txt")
                files.push_back(std::string(sub) + "/" + entry.path().filename().string());
    std::sort(files.begin(), files.end());
    return files;
}

// How much of the maze the mouse knows.
enum class Knowledge
{
    Full,    // every wall known, every cell explored
    None,    // nothing known: the first search plan
    Partial, // a fixed scatter of cells known and explored
};

// A maze loaded into the firmware's Maze with the chosen knowledge.
struct Loaded
{
    MazeText                   text;
    std::unique_ptr<Maze>      maze;
    std::unique_ptr<MazeMouse> mouse;

    Loaded(const std::string& name, Knowledge knowledge, MazeMask mask)
    {
        REQUIRE(loadMazeText(std::string(CORPUS_DIR) + "/" + name, text));
        maze  = std::make_unique<Maze>(text.height, text.width);
        mouse = std::make_unique<MazeMouse>(text.start, "n", text.goals, maze.get());
        for (int x = 0; x < text.width; ++x)
            for (int y = 0; y < text.height; ++y)
            {
                const bool known = knowledge == Knowledge::Full ||
                                   (knowledge == Knowledge::Partial && (x * 7 + y * 3) % 5 != 0);
                if (!known)
                    continue;
                Cell* c = maze->cell(x, y);
                for (char d : {'N', 'E', 'S', 'W'})
                {
                    if (text.wall(x, y, d))
                        c->setWall(d);
                    else
                        c->setExit(d);
                }
                c->markExplored();
            }
        mouse->setMazeMask(mask);
    }
};

std::string cellsText(const std::vector<Cell*>& path)
{
    std::string s;
    for (Cell* c : path)
        s += "(" + std::to_string(c->x()) + "," + std::to_string(c->y()) + ")";
    return s;
}

// One golden line: the route's length and a 64-bit FNV-1a hash of its cells.
std::string digest(const std::string& name, const char* scenario, const std::vector<Cell*>& path)
{
    uint64_t hash = 1469598103934665603ull;
    for (char c : cellsText(path))
        hash = (hash ^ static_cast<uint8_t>(c)) * 1099511628211ull;
    char line[256];
    std::snprintf(line, sizeof line, "%s %s %zu %016llx", name.c_str(), scenario, path.size(),
                  static_cast<unsigned long long>(hash));
    return line;
}

std::vector<Cell*> aStar(Loaded& m, bool diagonals, bool pass_goals, float explored_cost)
{
    AStar a_star(m.mouse.get());
    a_star.set_explored_cell_cost(explored_cost);
    return a_star.cellPath(m.text.goals, diagonals, pass_goals);
}

// Every planner scenario on one maze, one golden line each.
std::vector<std::string> mazeDigests(const std::string& name)
{
    std::vector<std::string> out;
    {
        Loaded m(name, Knowledge::Full, MASK_CLOSED);
        out.push_back(digest(name, "astar_full", aStar(m, false, false, 0.0f)));
        out.push_back(digest(name, "astar_full_pass", aStar(m, false, true, 0.0f)));
        out.push_back(digest(name, "astar_full_diag", aStar(m, true, false, 0.0f)));
        out.push_back(
            digest(name, "length_full", shortestExploredPath(m.mouse.get(), m.text.goals)));
        out.push_back(
            digest(name, "time_full", fastestExploredPath(m.mouse.get(), m.text.goals, model())));
        // The return trip: from the first goal back to the start.
        m.mouse->setPosition(m.maze->cell(m.text.goals[0][0], m.text.goals[0][1]));
        AStar back(m.mouse.get());
        out.push_back(digest(name, "astar_return", back.cellPath({m.text.start}, false, true)));
    }
    {
        Loaded m(name, Knowledge::None, MASK_OPEN);
        out.push_back(digest(name, "astar_none", aStar(m, false, false, 0.0f)));
        out.push_back(digest(name, "astar_none_diag", aStar(m, true, false, 0.0f)));
    }
    {
        Loaded m(name, Knowledge::Partial, MASK_OPEN);
        out.push_back(digest(name, "astar_partial", aStar(m, false, false, 0.0f)));
        out.push_back(digest(name, "astar_partial_unexplored", aStar(m, false, false, 0.5f)));
    }
    return out;
}
} // namespace

TEST_CASE("planner routes on the maze corpus match the golden file")
{
    const std::vector<std::string> files = corpusFiles();
    REQUIRE(files.size() >= 45);
    std::vector<std::string> lines;
    for (const std::string& name : files)
        for (const std::string& line : mazeDigests(name))
            lines.push_back(line);

    if (const char* out_path = std::getenv("PLANNER_GOLDEN_OUT"))
    {
        std::ofstream out(out_path);
        for (const std::string& line : lines)
            out << line << "\n";
        MESSAGE("wrote " << lines.size() << " golden lines to " << std::string(out_path));
        return;
    }

    std::ifstream            golden(GOLDEN_FILE);
    std::vector<std::string> expected;
    std::string              line;
    while (std::getline(golden, line))
        expected.push_back(line);
    REQUIRE(expected.size() == lines.size());
    for (size_t i = 0; i < lines.size(); ++i)
        CHECK(lines[i] == expected[i]);
}

TEST_CASE("planning allocates only the path it returns")
{
    Loaded                                m("classic/APEC2017.txt", Knowledge::Full, MASK_CLOSED);
    const std::vector<std::array<int, 2>> goals = m.text.goals;
    AStar                                 a_star(m.mouse.get());

    size_t before = g_allocations;
    {
        const std::vector<Cell*> path = a_star.cellPath(goals, false, true);
        CHECK(!path.empty());
    }
    CHECK(g_allocations - before == 1);

    before = g_allocations;
    {
        const std::vector<Cell*> path = shortestExploredPath(m.mouse.get(), goals);
        CHECK(!path.empty());
    }
    CHECK(g_allocations - before == 1);

    before = g_allocations;
    {
        const std::vector<Cell*> path = fastestExploredPath(m.mouse.get(), goals, model());
        CHECK(!path.empty());
    }
    CHECK(g_allocations - before == 1);
}

TEST_CASE("planning into a CellPath never allocates")
{
    Loaded                                m("classic/APEC2017.txt", Knowledge::Full, MASK_CLOSED);
    const std::vector<std::array<int, 2>> goals = m.text.goals;
    AStar                                 a_star(m.mouse.get());
    CellPath                              path;

    const size_t before = g_allocations;
    CHECK(!a_star.route(goals, false, true).empty());
    CHECK(shortestExploredRoute(m.mouse.get(), goals, path));
    CHECK(fastestExploredRoute(m.mouse.get(), goals, path, model()));
    CHECK(g_allocations == before);
}
