#ifndef HAL_PICO_PERIPHERALS_H
#define HAL_PICO_PERIPHERALS_H

#include "hal/peripherals.h"

#include "hardware/i2c.h"
#include "hardware/pio.h"

/**
 * @brief Maps hal peripheral handles to Pico SDK instances. Include only from
 *        Pico-side code (driver .cpp files, hal/pico/).
 */
namespace hal::pico
{
inline PIO sdkPio(PioBlock block)
{
    return pio_get_instance(static_cast<uint>(block));
}

inline i2c_inst_t* sdkI2c(I2cBus bus)
{
    return i2c_get_instance(static_cast<uint>(bus));
}
} // namespace hal::pico

#endif // HAL_PICO_PERIPHERALS_H
