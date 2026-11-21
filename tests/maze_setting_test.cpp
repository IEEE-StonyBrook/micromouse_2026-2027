// Host test for the MAZE command parser.
#include <string>

#include "doctest/doctest.h"
#include "maze/maze_setting.h"

static bool parses(const char* text, MazeSetting& out)
{
    std::string error;
    return parseMazeSetting(text, 16, out, error);
}

static bool rejected(const char* text)
{
    MazeSetting before;
    before.size     = 99;
    MazeSetting out = before;
    std::string error;
    const bool  ok = parseMazeSetting(text, 16, out, error);
    // A rejection explains itself and leaves the old setting alone.
    return !ok && !error.empty() && out.size == 99 && out.goals.empty();
}

TEST_CASE("maze setting: contest centre goals")
{
    MazeSetting s;
    CHECK(parses("16 7,7..8,8", s));
    CHECK(s.size == 16);
    CHECK(s.goals.size() == 4);
    CHECK(mazeSettingText(s) == "16x16 goals=(7,7),(7,8),(8,7),(8,8)");
}

TEST_CASE("maze setting: practice maze centre goals")
{
    MazeSetting s;
    CHECK(parses("8 3,3..4,4", s));
    CHECK(mazeSettingText(s) == "8x8 goals=(3,3),(3,4),(4,3),(4,4)");
}

TEST_CASE("maze setting: single goal cell")
{
    MazeSetting s;
    CHECK(parses("5 4,2", s));
    CHECK(s.size == 5);
    CHECK(s.goals.size() == 1);
    CHECK(mazeSettingText(s) == "5x5 goals=(4,2)");
}

TEST_CASE("maze setting: reversed rectangle is normalised")
{
    MazeSetting s;
    CHECK(parses("16 8,8..7,7", s));
    CHECK(mazeSettingText(s) == "16x16 goals=(7,7),(7,8),(8,7),(8,8)");
}

TEST_CASE("maze setting: out-of-range input is rejected")
{
    CHECK(rejected("17 7,7"));     // larger than the maze the firmware allocates
    CHECK(rejected("0 0,0"));      // no cells
    CHECK(rejected("-4 0,0"));     // negative size
    CHECK(rejected("8 8,8"));      // goal just outside an 8x8
    CHECK(rejected("8 3,3..4,8")); // rectangle corner outside
    CHECK(rejected("16 -1,3"));    // negative coordinate
}

TEST_CASE("maze setting: malformed input is rejected")
{
    CHECK(rejected(""));
    CHECK(rejected("16"));               // goals missing
    CHECK(rejected("16 7"));             // half a cell
    CHECK(rejected("16 7,7,8"));         // three numbers
    CHECK(rejected("16 7,7.."));         // rectangle missing its end
    CHECK(rejected("16 7,7...8,8"));     // three dots
    CHECK(rejected("16 a,b"));           // not numbers
    CHECK(rejected("16x 7,7"));          // trailing junk on size
    CHECK(rejected("16 7,7 extra"));     // trailing token
    CHECK(rejected("16 7,7..8,8..9,9")); // two ranges
    CHECK(rejected("99999999999 1,1"));  // overflow
}
