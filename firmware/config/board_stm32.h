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

// Crash watchdog (config/motion.h) on by default; CRASH OFF in the CLI disarms it.
#define CRASH_WATCHDOG_DEFAULT_ENABLED 1

#include "config/board_feather.h" // board::Pin, board::TimerPin

namespace board
{
// ================= Motor drivers ================= //
// Two DRV8874s in PWM (IN/IN) mode, PMODE strapped high, one shared nSLEEP,
// IPROPI to ADC1 (drivers/motor_bridge.h, hal/stm32/h_bridge.h).
//
// UNVERIFIED PLACEHOLDERS: the rev1 schematic is not final. Every pin,
// channel and resistor below is a guess from the integration plan (motor
// PWM on TIM8, IPROPI on PA5/PA6) and must be replaced with the schematic's
// values before the drivers are fitted. The Feather has no motor drivers
// and uses PC6/PC7 for an encoder, so MOTOR_DRIVERS_FITTED stays false
// there and nothing below is touched.
constexpr bool MOTOR_DRIVERS_FITTED = false;

// IN1/IN2 of each driver on TIM8 CH1-4, AF3 (DS8626 alternate function
// table). Entries 0-1 are the left driver's IN1, IN2; 2-3 the right's.
// UNVERIFIED placeholder.
constexpr TimerPin MOTOR_INPUTS[4] = {
    {8, 1, {'C', 6}, 3}, // left IN1:  TIM8_CH1
    {8, 2, {'C', 7}, 3}, // left IN2:  TIM8_CH2
    {8, 3, {'C', 8}, 3}, // right IN1: TIM8_CH3
    {8, 4, {'C', 9}, 3}, // right IN2: TIM8_CH4
};
// 20 kHz: above hearing, and a 168 MHz timer still gives 8400 steps.
constexpr uint32_t MOTOR_PWM_HZ = 20000;

// nSLEEP of both drivers, active low. UNVERIFIED placeholder pin.
constexpr Pin MOTOR_NSLEEP = {'B', 12};

// IPROPI of the left and right driver: PA5 = ADC12_IN5, PA6 = ADC12_IN6.
// UNVERIFIED placeholder pins and channels.
constexpr Pin     IPROPI_PINS[2]         = {{'A', 5}, {'A', 6}};
constexpr uint8_t IPROPI_ADC_CHANNELS[2] = {5, 6};

// R_IPROPI. UNVERIFIED placeholder: 2.2 kOhm puts 3.3 V full scale at
// 3.3 A of motor current (450 uA/A); size it so the measured stall current
// stays under 3.3 V.
constexpr float IPROPI_OHMS = 2200.0f;
// ADC1 reference: VDDA on the Feather; check rev1's VREF+.
constexpr float ADC_FULL_SCALE_VOLTS = 3.3f;
} // namespace board

#endif // CONFIG_BOARD_STM32_H
