/**
 * @file board_pico.h
 * @brief Raspberry Pi Pico (RP2040) board config
 */
#ifndef CONFIG_BOARD_PICO_H
#define CONFIG_BOARD_PICO_H

// Control loop rate. 500 Hz matches mazerunner-core / motorlab and leaves the
// FPU-less RP2040 headroom in the control ISR. Every per-tick quantity derives
// from this rate (config/motion.h), so changing it does not retune the mouse.
#define LOOP_FREQUENCY_HZ 500.0f

#endif // CONFIG_BOARD_PICO_H
