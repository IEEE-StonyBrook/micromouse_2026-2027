// STM32 Battery: a stand-in until the rev1 board's VBAT divider and ADC pin
// are known (TODO(rev1)). It reads nothing and reports the nominal pack, as
// Drivetrain does with no battery wired, so motor volts scale against
// DEFAULT_BATTERY_VOLTAGE and a sagging pack goes unseen.
#include "drivers/battery.h"

#include "config/config.h"

Battery::Battery(uint8_t adc_pin, float r1_ohms, float r2_ohms)
    : adc_pin_(adc_pin), adc_channel_(0), r1_ohms_(r1_ohms), r2_ohms_(r2_ohms),
      divider_ratio_((r1_ohms + r2_ohms) / r2_ohms), raw_readings_{}, reading_index_(0),
      reading_sum_(0), buffer_filled_(false), sagging_(false)
{
}

void Battery::begin()
{
}

void Battery::update()
{
}

float Battery::voltage() const
{
    return DEFAULT_BATTERY_VOLTAGE;
}

uint16_t Battery::raw_adc() const
{
    return 0;
}
