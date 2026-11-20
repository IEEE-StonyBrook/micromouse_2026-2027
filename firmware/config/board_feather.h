#ifndef CONFIG_BOARD_FEATHER_H
#define CONFIG_BOARD_FEATHER_H

/**
 * @file board_feather.h
 * @brief Pins, timers and buses of the Adafruit Feather STM32F405 Express,
 *        the P1B bring-up board. The rev1 mouse board gets its own file.
 *
 * Plain data, no libopencm3, so host tests can check the table;
 * hal/stm32/gpio.h turns a Pin into libopencm3 port and pin values.
 *
 * Verification status:
 * - Feather labels -> STM32 pins: checked against Adafruit's board definition
 *   (CircuitPython ports/stm/boards/feather_stm32f405_express, pins.c and
 *   mpconfigboard.h): LED/D13 = PC1, D12 = PC2, D5 = PC7, D6 = PC6, D10 = PB9,
 *   SCL = PB6, SDA = PB7, SCK/MISO/MOSI = PB13/PB14/PB15, TX/RX = PB10/PB11,
 *   12 MHz HSE. Reserved: SDIO PC8-PC12 + PD2, SPI flash PB3-PB5 + PA15,
 *   battery monitor PA3, NeoPixel PC0.
 * - Alternate-function numbers come from the STM32F405 datasheet (DS8626)
 *   alternate function table. UNVERIFIED ON HARDWARE.
 * - DMA streams come from RM0090's DMA1 request mapping. UNVERIFIED ON HARDWARE.
 * - UNVERIFIED: whether the Feather fits I2C pull-ups on SCL/SDA. The right
 *   encoder uses those pins; external pull-ups there only add to the internal
 *   ones the encoder setup enables, but check the edges on a scope.
 */

#include <cstdint>

namespace board
{
/// A GPIO pin: port letter 'A'..'D' and pin number 0..15.
struct Pin
{
    char    port;
    uint8_t number;
};

/// A pin in alternate-function mode.
struct AfPin
{
    Pin     pin;
    uint8_t af;
};

/// One timer input or output channel: {timer, channel, pin, AF}.
struct TimerPin
{
    uint8_t timer;   ///< TIMx number
    uint8_t channel; ///< 1..4
    Pin     pin;
    uint8_t af;
};

struct UartConfig
{
    uint8_t  usart; ///< USARTx number
    AfPin    tx;
    AfPin    rx;
    uint32_t baud;
    uint8_t  dma;         ///< DMA controller number
    uint8_t  dma_channel; ///< Request channel shared by both streams
    uint8_t  tx_stream;
    uint8_t  rx_stream;
};

struct SpiConfig
{
    uint8_t  spi; ///< SPIx number
    AfPin    sck;
    AfPin    miso;
    AfPin    mosi;
    Pin      cs; ///< Plain GPIO output, active low
    uint32_t max_clock_hz;
};

// ================= Clock ================= //
constexpr uint32_t HSE_HZ    = 12000000;  // Feather crystal
constexpr uint32_t SYSCLK_HZ = 168000000; // STM32F405 maximum

// ================= LED ================= //
// Red user LED on Feather pin D13, active high.
constexpr Pin LED = {'C', 1};

// ================= Tick probe ================= //
// Feather pin D12: high for the duration of each TIM7 control tick, so a
// scope shows the 1 kHz rate, its jitter and the ISR's length.
constexpr Pin TICK_PROBE = {'C', 2};

// ================= Encoders ================= //
// Quadrature inputs in encoder mode. Entries 0-1 are the left encoder (A, B),
// 2-3 the right. Left/right is arbitrary on the Feather: swap rows to match
// the wiring. Both timers run from the 84 MHz APB1 timer clock.
constexpr TimerPin ENCODER_PINS[4] = {
    {3, 1, {'C', 6}, 2}, // left A:  D6,  TIM3_CH1, AF2
    {3, 2, {'C', 7}, 2}, // left B:  D5,  TIM3_CH2, AF2
    {4, 1, {'B', 6}, 2}, // right A: SCL, TIM4_CH1, AF2
    {4, 2, {'B', 7}, 2}, // right B: SDA, TIM4_CH2, AF2
};

// ================= Console ================= //
// USART3 sits on APB1 (42 MHz), where 1 Mbaud divides exactly. DMA1 channel 4
// carries USART3_TX on stream 3 and USART3_RX on stream 1.
constexpr UartConfig CONSOLE = {
    3,
    {{'B', 10}, 7}, // TX: Feather TX, USART3_TX, AF7
    {{'B', 11}, 7}, // RX: Feather RX, USART3_RX, AF7
    1000000,
    1,
    4,
    3,
    1,
};

// ================= IMU ================= //
// LSM6DSR breakout on the Feather's SPI header (SPI2, APB1 42 MHz). The
// LSM6DSR's SPI clock tops out at 10 MHz.
constexpr SpiConfig IMU_SPI = {
    2,
    {{'B', 13}, 5}, // SCK:  SPI2_SCK,  AF5
    {{'B', 14}, 5}, // MISO: SPI2_MISO, AF5
    {{'B', 15}, 5}, // MOSI: SPI2_MOSI, AF5
    {'B', 9},       // CS:   D10
    10000000,
};
} // namespace board

#endif // CONFIG_BOARD_FEATHER_H
