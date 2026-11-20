#ifndef HAL_STM32_SPI_H
#define HAL_STM32_SPI_H

#include "config/board_feather.h"
#include "hal/spi.h"

namespace hal::stm32
{
/**
 * @brief A polled SPI master (mode 3, 8-bit, MSB first) with a GPIO chip select.
 *
 * Polled, not DMA: an LSM6DSR FIFO read is a few dozen bytes per tick, about
 * 60 us at 5.25 MHz, and polling keeps it simple to call from the tick.
 * One user per bus: nothing arbitrates between contexts.
 */
class SpiBus final : public hal::SpiDevice
{
  public:
    /// Clock is the fastest APB1 / 2^k at or under config.max_clock_hz.
    void init(const board::SpiConfig& config);

    void transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) override;

  private:
    uint32_t cs_port_ = 0;
    uint16_t cs_bit_  = 0;
};
} // namespace hal::stm32

#endif // HAL_STM32_SPI_H
