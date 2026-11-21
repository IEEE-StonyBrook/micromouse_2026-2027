#include "driver_lab/board_io.h"

#include <cstdint>

#include "config/config.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

namespace driver_lab_board
{
void setupRightEncoderPinsRaw()
{
    gpio_init(PIN_ENCODER_R_A);
    gpio_set_dir(PIN_ENCODER_R_A, GPIO_IN);
    gpio_pull_up(PIN_ENCODER_R_A);
    gpio_init(PIN_ENCODER_R_B);
    gpio_set_dir(PIN_ENCODER_R_B, GPIO_IN);
    gpio_pull_up(PIN_ENCODER_R_B);
}

void readRightEncoderPins(bool& pin_a, bool& pin_b)
{
    pin_a = gpio_get(PIN_ENCODER_R_A);
    pin_b = gpio_get(PIN_ENCODER_R_B);
}

void driveMotorsRaw(bool dir_level, float duty)
{
    const uint16_t level = static_cast<uint16_t>(PWM_WRAP * duty);
    gpio_put(PIN_MOTOR_L_DIR, dir_level);
    gpio_put(PIN_MOTOR_R_DIR, dir_level);
    pwm_set_chan_level(pwm_gpio_to_slice_num(PIN_MOTOR_L_PWM), pwm_gpio_to_channel(PIN_MOTOR_L_PWM),
                       level);
    pwm_set_chan_level(pwm_gpio_to_slice_num(PIN_MOTOR_R_PWM), pwm_gpio_to_channel(PIN_MOTOR_R_PWM),
                       level);
}
} // namespace driver_lab_board
