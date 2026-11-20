#include "hal/stm32/spi.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "hal/stm32/clock.h"
#include "hal/stm32/gpio.h"

namespace hal::stm32
{
namespace
{
// Register names below are SPI2's; fail the build if the board asks for another bus.
static_assert(board::IMU_SPI.spi == 2, "SPI glue is written for SPI2");
constexpr uint32_t SPI = SPI2;

/// SPI_CR1.BR for a divider of 2^(BR + 1).
constexpr uint32_t baudBits(uint32_t divider)
{
    uint32_t br = 0;
    while ((2u << br) < divider)
        ++br;
    return br << 3;
}
static_assert(baudBits(8) == SPI_CR1_BAUDRATE_FPCLK_DIV_8, "BR encoding");
} // namespace

void SpiBus::init(const board::SpiConfig& config)
{
    gpioAlternate(config.sck.pin, config.sck.af);
    gpioAlternate(config.miso.pin, config.miso.af);
    gpioAlternate(config.mosi.pin, config.mosi.af);

    cs_port_ = gpioPort(config.cs);
    cs_bit_  = gpioBit(config.cs);
    gpioOutput(config.cs);
    gpio_set(cs_port_, cs_bit_); // deselected

    rcc_periph_clock_enable(RCC_SPI2);
    rcc_periph_reset_pulse(RST_SPI2);
    const uint32_t divider = spiDivider(BUSES.pclk1_hz, config.max_clock_hz);
    spi_init_master(SPI, baudBits(divider), SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    // Chip select is a GPIO, so the peripheral's own NSS input is tied high in software.
    spi_enable_software_slave_management(SPI);
    spi_set_nss_high(SPI);
    spi_enable(SPI);
}

void SpiBus::transfer(const uint8_t* tx, uint8_t* rx, std::size_t len)
{
    gpio_clear(cs_port_, cs_bit_);
    for (std::size_t i = 0; i < len; ++i)
    {
        const uint8_t in = static_cast<uint8_t>(spi_xfer(SPI, tx != nullptr ? tx[i] : 0));
        if (rx != nullptr)
            rx[i] = in;
    }
    // spi_xfer returns on RXNE, after the last clock edge, but wait out BSY
    // anyway before releasing chip select (RM0090 SPI disable procedure).
    while (SPI_SR(SPI) & SPI_SR_BSY)
    {
    }
    gpio_set(cs_port_, cs_bit_);
}
} // namespace hal::stm32
