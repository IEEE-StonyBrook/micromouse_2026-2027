/**
 * @file mouse_stm32.h
 * @brief The STM32 mouse's wheels, encoders and motor model: what DriverLab
 *        measures on the rev1 board (#38). geometry.h and tuning.h take these
 *        in STM32 builds; the Pico mouse keeps its own values there.
 *
 * UNVERIFIED PLACEHOLDERS, every one. Nothing here has been measured on the
 * new mouse:
 * - The wheel diameter is the integration plan's working figure (37 mm, still
 *   to be measured), and 7273 counts per wheel turn its reference figure.
 * - The wheel base and the whole motor model are the Pico mouse's values,
 *   copied only so the controllers start from a sane shape. The new motors,
 *   gearing and mass differ: run DriverLab OL, then STEP, then TURN-OL and
 *   TURN-STEP on the board, Q-push 500 mm for MM_PER_TICK_LEFT/RIGHT, and
 *   replace these before trusting a closed-loop move.
 */
#ifndef CONFIG_MOUSE_STM32_H
#define CONFIG_MOUSE_STM32_H

// ===================== Wheels ===================== //
#define WHEEL_DIAMETER_MM 37.0f // UNVERIFIED: plan figure, not measured
#define WHEEL_BASE_MM     80.4f // UNVERIFIED: the Pico mouse's value

// ===================== Encoder ===================== //
// Counts per wheel turn, all four edges counted by the timer (QuadCounter).
#define TICKS_PER_REVOLUTION 7273.0f // UNVERIFIED: plan reference figure
#define MM_PER_TICK_NOMINAL  ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REVOLUTION)

// Per-wheel mm per count, from a 500 mm hand push. UNVERIFIED: nominal until
// measured on the board.
#define MM_PER_TICK_LEFT  MM_PER_TICK_NOMINAL
#define MM_PER_TICK_RIGHT MM_PER_TICK_NOMINAL

// ===================== Motor Model ===================== //
// UNVERIFIED: all the Pico mouse's values (see the file comment).
#define MOTOR_KM 317.06f // mm/s per volt, from OL
#define MOTOR_TM 0.080f  // seconds, from STEP
#define ROT_KM   385.0f  // deg/s per volt of differential drive, from TURN-OL
#define ROT_TM   0.097f  // seconds, from TURN-STEP

// ================ Feedforward (per motor) ============== //
// V = kV * speed + kS + kA * accel. UNVERIFIED: the Pico mouse's values.
#define FORWARD_KVL (1.0f / MOTOR_KM) // V/(mm/s)
#define FORWARD_KVR (1.0f / MOTOR_KM)
#define FORWARD_KSL 0.6907f // V
#define FORWARD_KSR 0.5701f
#define FORWARD_KAL 0.0002194f // V/(mm/s^2)
#define FORWARD_KAR 0.0002194f

#endif // CONFIG_MOUSE_STM32_H
