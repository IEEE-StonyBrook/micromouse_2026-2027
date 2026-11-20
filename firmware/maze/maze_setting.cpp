#include "maze/maze_setting.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <sstream>

namespace
{

// Whole-string base-10 integer: rejects empty text, trailing junk and overflow.
bool parseInt(const std::string& text, int& value)
{
    if (text.empty())
        return false;
    char* end = nullptr;
    errno     = 0;
    long v    = std::strtol(text.c_str(), &end, 10);
    if (errno != 0 || *end != '\0' || v < -1000000 || v > 1000000)
        return false;
    value = static_cast<int>(v);
    return true;
}

bool parseCell(const std::string& text, std::array<int, 2>& cell)
{
    const size_t comma = text.find(',');
    if (comma == std::string::npos)
        return false;
    return parseInt(text.substr(0, comma), cell[0]) && parseInt(text.substr(comma + 1), cell[1]);
}

} // namespace

bool parseMazeSetting(const std::string& text, int max_size, MazeSetting& out, std::string& error)
{
    std::istringstream in(text);
    std::string        size_text, goals_text, extra;
    in >> size_text >> goals_text;
    if (size_text.empty() || goals_text.empty() || (in >> extra))
    {
        error = "usage: MAZE <size> <x,y> or MAZE <size> <x1,y1..x2,y2>";
        return false;
    }

    int size = 0;
    if (!parseInt(size_text, size) || size < 1 || size > max_size)
    {
        error = "size must be 1.." + std::to_string(max_size);
        return false;
    }

    std::array<int, 2> first{}, last{};
    const size_t       dots = goals_text.find("..");
    bool               ok   = false;
    if (dots == std::string::npos)
    {
        ok   = parseCell(goals_text, first);
        last = first;
    }
    else
    {
        ok = parseCell(goals_text.substr(0, dots), first) &&
             parseCell(goals_text.substr(dots + 2), last);
    }
    if (!ok)
    {
        error = "goals must be x,y or x1,y1..x2,y2";
        return false;
    }

    const int x_lo = std::min(first[0], last[0]), x_hi = std::max(first[0], last[0]);
    const int y_lo = std::min(first[1], last[1]), y_hi = std::max(first[1], last[1]);
    if (x_lo < 0 || y_lo < 0 || x_hi >= size || y_hi >= size)
    {
        error = "goals must lie in 0.." + std::to_string(size - 1);
        return false;
    }

    // x outer, y inner: the same order main_pico.cpp has always listed goal cells in.
    MazeSetting setting;
    setting.size = size;
    for (int x = x_lo; x <= x_hi; ++x)
        for (int y = y_lo; y <= y_hi; ++y)
            setting.goals.push_back({x, y});
    out = setting;
    return true;
}

std::string mazeSettingText(const MazeSetting& setting)
{
    std::string text =
        std::to_string(setting.size) + "x" + std::to_string(setting.size) + " goals=";
    for (size_t i = 0; i < setting.goals.size(); ++i)
    {
        if (i > 0)
            text += ",";
        text += "(" + std::to_string(setting.goals[i][0]) + "," +
                std::to_string(setting.goals[i][1]) + ")";
    }
    if (setting.goals.empty())
        text += "(none)";
    return text;
}
