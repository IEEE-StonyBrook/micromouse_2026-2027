/**
 * @file motor_driver.h
 * @brief DRV8874 motor driver constants (STM32 boards; the Pico's TB6552
 *        has no sleep or current sense). Used by drivers/motor_bridge.h.
 */
#ifndef CONFIG_MOTOR_DRIVER_H
#define CONFIG_MOTOR_DRIVER_H

// Datasheet SLVSF66A, Electrical Characteristics:
//   tWAKE   nSLEEP high to active, max 1 ms: inputs are ignored until then.
//   tSLEEP  nSLEEP low to sleep mode, max 1 ms.
//   AIPROPI current mirror gain, typ 450 uA per A of low-side current.
#define DRV8874_WAKE_TIME_US     1000
#define DRV8874_SLEEP_TIME_US    1000
#define DRV8874_AIPROPI_UA_PER_A 450.0f
// Braked with no command this long, MotorBridge puts the drivers to sleep.
// Long enough that the pauses inside a run (a wall read at a cell, the
// start-cell wait) stay braked, short enough that a parked mouse sleeps soon.
#define MOTOR_IDLE_SLEEP_TIME_S 1.0f

#endif // CONFIG_MOTOR_DRIVER_H
