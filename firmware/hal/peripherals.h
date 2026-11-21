#ifndef HAL_PERIPHERALS_H
#define HAL_PERIPHERALS_H

#include <cstdint>

/**
 * @brief Platform-neutral handles for peripheral instances.
 *
 * Drivers take these instead of SDK pointers (PIO, i2c_inst_t*) so their
 * headers stay SDK-free. hal/pico/peripherals.h maps them to SDK instances.
 */
namespace hal
{
enum class PioBlock : uint8_t
{
    kPio0 = 0,
    kPio1 = 1,
};

enum class I2cBus : uint8_t
{
    kI2c0 = 0,
    kI2c1 = 1,
};
} // namespace hal

#endif // HAL_PERIPHERALS_H
