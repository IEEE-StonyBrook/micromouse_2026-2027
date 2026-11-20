#include "drivers/encoder.h"

#include "hal/pico/peripherals.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "quadrature_encoder/QuadratureEncoder.h"

Encoder::Encoder(hal::PioBlock pio_block, int gpio_pin, bool invert_direction)
    : pio_block_(pio_block), gpio_pin_(gpio_pin), offset_ticks_(0),
      invert_direction_(invert_direction)
{
    const PIO pio  = hal::pico::sdkPio(pio_block_);
    state_machine_ = pio_claim_unused_sm(pio, true);
    load_pio_program();
    quadrature_encoder_program_init(pio, state_machine_, gpio_pin_, 0);
}

void Encoder::load_pio_program()
{
    static bool program_loaded = false;
    if (!program_loaded)
    {
        pio_add_program_at_offset(hal::pico::sdkPio(pio_block_), &quadrature_encoder_program, 0);
        program_loaded = true;
    }
}

int Encoder::ticks() const
{
    int raw_ticks = quadrature_encoder_get_count(hal::pico::sdkPio(pio_block_), state_machine_);
    int direction_corrected = raw_ticks * (invert_direction_ ? -1 : 1);
    return direction_corrected - offset_ticks_;
}

void Encoder::reset()
{
    LOG_DEBUG("Resetting encoder tick count to zero");
    offset_ticks_ += ticks();
}
