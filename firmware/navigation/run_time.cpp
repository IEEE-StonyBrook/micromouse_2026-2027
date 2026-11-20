#include "navigation/run_time.h"

#include <cmath>

#include "navigation/path_converter.h"

float trapezoidTimeS(float distance, float peak_rate, float accel)
{
    distance = std::fabs(distance);
    if (distance <= 0.0f || peak_rate <= 0.0f || accel <= 0.0f)
        return 0.0f;

    // Each ramp to peak_rate covers peak_rate^2 / (2 accel). If both ramps
    // need more than the whole distance, the profile is a triangle.
    const float ramp_distance = peak_rate * peak_rate / (2.0f * accel);
    if (2.0f * ramp_distance >= distance)
        return 2.0f * std::sqrt(distance / accel);
    return 2.0f * peak_rate / accel + (distance - 2.0f * ramp_distance) / peak_rate;
}

float straightTimeS(int cells, const RunTimeModel& model)
{
    return trapezoidTimeS(static_cast<float>(cells) * model.cell_mm, model.speed_mmps,
                          model.accel_mmps2);
}

float turnTimeS(float angle_deg, const RunTimeModel& model)
{
    return trapezoidTimeS(angle_deg, model.turn_omega_degps, model.turn_alpha_degps2);
}

float estimatedRunTimeS(const std::string& lfr, const RunTimeModel& model)
{
    float time_s        = 0.0f;
    int   pending_cells = 0;
    for (char c : lfr)
    {
        if (c == '#')
            continue;
        if (c == 'F')
        {
            ++pending_cells;
            continue;
        }
        time_s += straightTimeS(pending_cells, model);
        pending_cells = 0;
        if (c == 'L' || c == 'R')
            time_s += turnTimeS(90.0f, model);
    }
    return time_s + straightTimeS(pending_cells, model);
}

float pathRunTimeS(Cell* start, std::array<int, 2> heading, const std::vector<Cell*>& path,
                   const RunTimeModel& model)
{
    return estimatedRunTimeS(PathConverter::buildLFR(start, heading, path), model);
}
