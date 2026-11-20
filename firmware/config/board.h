/**
 * @file board.h
 * @brief Selects the per-platform board config
 *
 * Each board header sets what differs between platforms; today that is the
 * control loop rate. The Pico board is the default, so the simulator and the
 * host tests build as the Pico unless told otherwise. Host tests may force a
 * rate with -DHOST_LOOP_FREQUENCY_HZ=<Hz> to check that behaviour does not
 * depend on it.
 */
#ifndef CONFIG_BOARD_H
#define CONFIG_BOARD_H

#if defined(HOST_LOOP_FREQUENCY_HZ)
#if defined(PICO_BUILD) || defined(STM32_BUILD)
#error "HOST_LOOP_FREQUENCY_HZ is for host builds only"
#endif
#define LOOP_FREQUENCY_HZ HOST_LOOP_FREQUENCY_HZ
#elif defined(STM32_BUILD)
#include "config/board_stm32.h"
#else
#include "config/board_pico.h"
#endif

#endif // CONFIG_BOARD_H
