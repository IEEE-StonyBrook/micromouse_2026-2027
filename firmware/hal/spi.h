#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <cstddef>
#include <cstdint>

namespace hal
{
/**
 * @brief One device on an SPI bus, with its chip select.
 *
 * Drivers talk to their chip through this, so they build and test on the
 * host against a fake. Backends: hal/stm32/spi.h.
 */
class SpiDevice
{
  public:
    /// Clocks len bytes out of tx while filling rx (which may be nullptr),
    /// with chip select held active for the whole transfer. Blocks until done.
    virtual void transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) = 0;

  protected:
    ~SpiDevice() = default;
};
} // namespace hal

#endif // HAL_SPI_H
