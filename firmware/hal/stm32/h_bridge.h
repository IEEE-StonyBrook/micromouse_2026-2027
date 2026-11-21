#ifndef HAL_STM32_H_BRIDGE_H
#define HAL_STM32_H_BRIDGE_H

#include <cstdint>

#include "hal/h_bridge.h"

namespace hal::stm32
{
/**
 * @brief Two DRV8874s on the board's pins (config/board_stm32.h): IN1/IN2
 *        as TIM8 PWM, one nSLEEP GPIO, IPROPI on ADC1's injected group.
 *
 * IPROPI is not chopped by slow-decay PWM (see drivers/motor_bridge.h), so
 * it needs no sync to the PWM. sample() reads the pair the last call started
 * and starts the next, so each reading is one tick old and the tick never
 * waits on the ADC. Call sample() once per tick, before MotorBridge::update().
 */
class HBridge final : public hal::HBridgePort
{
  public:
    /// nSLEEP low and every input low first, then the PWM and the ADC.
    void init();

    void     setInputs(int motor, float in1_high, float in2_high) override;
    void     setAwake(bool awake) override;
    uint16_t senseCounts(int motor) override { return counts_[motor]; }

    /// Tick context: collects the last IPROPI conversion, starts the next.
    void sample();

  private:
    bool              ready_     = false;
    uint32_t          period_    = 0; // timer counts per PWM period
    volatile uint16_t counts_[2] = {0, 0};
};
} // namespace hal::stm32

#endif // HAL_STM32_H_BRIDGE_H
