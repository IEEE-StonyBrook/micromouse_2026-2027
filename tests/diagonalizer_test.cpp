// Host test for the cardinal-to-diagonal path converter.
#include <sstream>
#include <string>
#include <vector>

#include "doctest/doctest.h"
#include "navigation/diagonalizer.h"

static std::vector<std::string> tokens(const std::string& path)
{
    std::vector<std::string> out;
    std::stringstream        ss(path);
    std::string              token;
    while (std::getline(ss, token, '#'))
        out.push_back(token);
    return out;
}

// Swap left and right turns: a mirrored path must diagonalize to the mirror.
static std::string mirror(const std::string& path)
{
    std::string out;
    for (const std::string& t : tokens(path))
    {
        std::string m = t;
        if (m[0] == 'L')
            m[0] = 'R';
        else if (m[0] == 'R')
            m[0] = 'L';
        out += (out.empty() ? "" : "#") + m;
    }
    return out;
}

TEST_CASE("Diagonalizer leaves an L-F-L diagonal exit with the ghost move GMF")
{
    // R F L F opens a diagonal; the F L F that follows exits it to the left.
    const std::string out = Diagonalizer::diagonalize("R#F#L#F#F#L#F");
    CHECK(out == "R#FH#L45#FH#GMF#L45#F#L45#FH#L45#FH#GMF");
}

TEST_CASE("Diagonalizer emits only tokens the speed run executes")
{
    const char* paths[] = {"R#F#L#F#F#L#F", "L#F#R#F#F#R#F",   "F#R#F#L#F#R#F#L#F",
                           "F#F#R#F#R#F#F", "L#F#L#F#L#F#L#F", "R#F#R#F#F#L#F#L#F"};
    for (const char* p : paths)
    {
        for (const std::string& t : tokens(Diagonalizer::diagonalize(p)))
        {
            INFO(std::string(p) << " -> " << t);
            CHECK((t == "F" || t == "FH" || t == "L" || t == "R" || t == "L45" || t == "R45" ||
                   t == "GMF"));
        }
        INFO(std::string(p));
        CHECK(Diagonalizer::diagonalize(mirror(p)) == mirror(Diagonalizer::diagonalize(p)));
    }
}
