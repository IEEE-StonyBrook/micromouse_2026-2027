#ifndef DRIVER_LAB_BOARD_IO_H
#define DRIVER_LAB_BOARD_IO_H

/**
 * @file board_io.h
 * @brief The two DriverLab diagnostics that bypass the drivers and touch pins
 *        directly (GPIO DIAG and DIRTEST). Board-specific: the Pico half is
 *        driver_lab/pico/board_io.cpp. Main loop only.
 */
namespace driver_lab_board
{
/// Makes the right encoder's A and B pins plain pulled-up inputs.
void setupRightEncoderPinsRaw();

/// Raw logic levels of the right encoder's A and B pins.
void readRightEncoderPins(bool& pin_a, bool& pin_b);

/// Sets both DIR pins to dir_level and both PWM outputs to duty (0..1 of
/// PWM_WRAP), without going through Motor (no polarity or battery scaling).
void driveMotorsRaw(bool dir_level, float duty);
} // namespace driver_lab_board

#endif // DRIVER_LAB_BOARD_IO_H
