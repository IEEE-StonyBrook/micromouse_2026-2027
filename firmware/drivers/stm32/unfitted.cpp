// STM32: the sensors the rev1 board does not carry. DriverLab's ToF and line
// sensor commands are compiled for every board; main_stm32.cpp hands
// DriverLab null pointers for these, so it reports "not fitted" and none of
// these definitions ever runs. They exist so the image links.
#include "drivers/line_sensor.h"
#include "drivers/tof.h"

float ToF::get_distance()
{
    return 0.0f;
}

bool LineSensor::read()
{
    return false;
}

float LineSensor::get_position() const
{
    return 0.0f;
}

bool LineSensor::on_line() const
{
    return false;
}

LineSensor::IntersectionEvent LineSensor::intersectionEvent()
{
    return {};
}

uint8_t LineSensor::currentPathsMask() const
{
    return PATH_NONE;
}

uint8_t LineSensor::rawByte() const
{
    return 0;
}

uint8_t LineSensor::activeMask() const
{
    return 0;
}
