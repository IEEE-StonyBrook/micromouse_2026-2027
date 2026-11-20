#ifndef MAZE_MAZE_SETTING_H
#define MAZE_MAZE_SETTING_H

#include <array>
#include <string>
#include <vector>

/**
 * @brief Maze size and goal cells chosen at runtime by the `MAZE` command.
 *
 * Cells use the maze's (x, y) convention: (0, 0) is the start corner, x grows
 * east and y grows north.
 */
struct MazeSetting
{
    int                             size = 0;
    std::vector<std::array<int, 2>> goals;
};

/**
 * @brief Parse `<size> <goals>` from the `MAZE` command, with no hardware.
 *
 * `goals` is one cell `x,y` or an inclusive rectangle `x1,y1..x2,y2`.
 * Size must be 1..max_size and every goal must lie inside the size.
 * On failure returns false, leaves `out` untouched and sets `error`.
 */
bool parseMazeSetting(const std::string& text, int max_size, MazeSetting& out, std::string& error);

/** @brief One line such as `16x16 goals=(7,7),(7,8),(8,7),(8,8)`. */
std::string mazeSettingText(const MazeSetting& setting);

#endif
