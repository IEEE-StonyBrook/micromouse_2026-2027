/**
 * @file board_stm32.h
 * @brief STM32 board config
 */
#ifndef CONFIG_BOARD_STM32_H
#define CONFIG_BOARD_STM32_H

// Control loop rate. The STM32 has an FPU and a hardware tick timer, so it
// runs the controllers at 1 kHz. Every per-tick quantity derives from this
// rate (config/motion.h).
#define LOOP_FREQUENCY_HZ 1000.0f

#endif // CONFIG_BOARD_STM32_H
