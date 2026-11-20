#ifndef HAL_STM32_IRQ_H
#define HAL_STM32_IRQ_H

#include <cstdint>

/**
 * NVIC priorities for every STM32 interrupt, in one place so their order is
 * visible. Lower is more urgent; the F405 implements the top 4 bits.
 */
namespace hal::stm32::irq
{
constexpr uint8_t TICK    = 2 << 4; ///< TIM7 1 kHz control tick
constexpr uint8_t CONSOLE = 8 << 4; ///< USART3 and its DMA streams
} // namespace hal::stm32::irq

#endif // HAL_STM32_IRQ_H
