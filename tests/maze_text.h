// Host-test loader for the sim corpus's text mazes (sim/mazes/README.md):
// posts every 4 columns and 2 rows, north at the top, any non-space at a wall
// position is a wall, G marks a goal and S the start. Without G the goal is
// the centre. Mirrors sim/runner/maze_file.py's parse_text.
#ifndef TESTS_MAZE_TEXT_H
#define TESTS_MAZE_TEXT_H

#include <array>
#include <fstream>
#include <string>
#include <vector>

struct MazeText
{
    int                             width  = 0;
    int                             height = 0;
    std::array<int, 2>              start  = {0, 0};
    std::vector<std::array<int, 2>> goals;
    // walls[x][y] holds the sides of cell (x, y) that are walls, as "NESW" letters.
    std::vector<std::vector<std::string>> walls;

    bool wall(int x, int y, char side) const { return walls[x][y].find(side) != std::string::npos; }
};

inline bool loadMazeText(const std::string& path, MazeText& out)
{
    std::ifstream            file(path);
    std::vector<std::string> lines;
    std::string              line;
    while (std::getline(file, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.find_first_not_of(' ') != std::string::npos)
            lines.push_back(line);
    }
    if (lines.size() < 3 || lines.size() % 2 == 0 || (lines[0].size() - 1) % 4 != 0)
        return false;
    const size_t post_len = lines[0].size();
    for (std::string& l : lines)
        l.resize(post_len, ' ');

    out.height = static_cast<int>(lines.size() - 1) / 2;
    out.width  = static_cast<int>(post_len - 1) / 4;
    out.walls.assign(out.width, std::vector<std::string>(out.height));
    out.goals.clear();
    auto setWall = [&](int x, int y, char side)
    {
        out.walls[x][y] += side;
        const int  dx       = side == 'E' ? 1 : side == 'W' ? -1 : 0;
        const int  dy       = side == 'N' ? 1 : side == 'S' ? -1 : 0;
        const char opposite = side == 'N' ? 'S' : side == 'S' ? 'N' : side == 'E' ? 'W' : 'E';
        const int  nx = x + dx, ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < out.width && ny < out.height)
            out.walls[nx][ny] += opposite;
    };
    for (int x = 0; x < out.width; ++x)
    {
        setWall(x, 0, 'S');
        setWall(x, out.height - 1, 'N');
    }
    for (int y = 0; y < out.height; ++y)
    {
        setWall(0, y, 'W');
        setWall(out.width - 1, y, 'E');
    }
    for (int y = 0; y < out.height; ++y)
    {
        const size_t row = 2 * (out.height - 1 - y) + 1;
        for (int x = 0; x < out.width; ++x)
        {
            const size_t col = 4 * x + 2;
            if (lines[row - 1][col] != ' ')
                setWall(x, y, 'N');
            if (lines[row][col - 2] != ' ')
                setWall(x, y, 'W');
            const char mark = lines[row][col];
            if (mark == 'G' || mark == 'g')
                out.goals.push_back({x, y});
            else if (mark == 'S' || mark == 's')
                out.start = {x, y};
        }
    }
    if (out.goals.empty())
    {
        auto middle = [](int n)
        { return n % 2 == 0 ? std::vector<int>{n / 2 - 1, n / 2} : std::vector<int>{n / 2}; };
        for (int x : middle(out.width))
            for (int y : middle(out.height))
                out.goals.push_back({x, y});
    }
    return true;
}

#endif
