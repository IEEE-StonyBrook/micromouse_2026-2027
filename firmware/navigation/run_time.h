#ifndef NAVIGATION_RUN_TIME_H
#define NAVIGATION_RUN_TIME_H

#include <array>
#include <string>
#include <vector>

#include "config/geometry.h"
#include "config/motion.h"

class Cell;

// How the cardinal speed run drives a route (PathUtils::traverseExploredPath):
// each merged straight is one rest-to-rest move at the fast cruise speed and
// base acceleration, and each 90 deg turn is a spin in place at the spin-turn
// rate limits. Defaults are the firmware's own speed-run constants.
struct RunTimeModel
{
    float cell_mm           = CELL_SIZE_MM;
    float speed_mmps        = ROBOT_MAX_FAST_SPEED_MMPS;
    float accel_mmps2       = ROBOT_BASE_ACCEL_MMPS2;
    float turn_omega_degps  = ROBOT_MAX_TURN_SPEED_DEGPS;
    float turn_alpha_degps2 = ROBOT_BASE_ANGULAR_ACCEL_DEGPS2;
};

// Time for a rest-to-rest trapezoidal (or, if too short to reach the peak,
// triangular) profile over distance at peak rate and acceleration.
float trapezoidTimeS(float distance, float peak_rate, float accel);

// Time for one straight of `cells` cells, from rest to rest.
float straightTimeS(int cells, const RunTimeModel& model);

// Time for one spin turn of angle_deg (either sign).
float turnTimeS(float angle_deg, const RunTimeModel& model);

// Estimated time to drive an LFR path ("F#R#F#..."): consecutive F tokens
// merge into one straight, as the speed run merges them.
float estimatedRunTimeS(const std::string& lfr, const RunTimeModel& model);

// Estimated time of a cell path from start facing heading.
float pathRunTimeS(Cell* start, std::array<int, 2> heading, const std::vector<Cell*>& path,
                   const RunTimeModel& model);

#endif
