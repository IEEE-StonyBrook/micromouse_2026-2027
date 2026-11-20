#include "hal/stm32/tick.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "config/board_feather.h"
#include "hal/stm32/clock.h"
#include "hal/stm32/counter_extender.h"
#include "hal/stm32/gpio.h"
#include "hal/stm32/irq.h"

namespace hal::stm32::tick
{
namespace
{
constexpr uint32_t COUNT_HZ = 1000000; // both timers count microseconds

constexpr TimerDivisors DIVISORS = timerDivisors(BUSES.tim_apb1_hz, COUNT_HZ, TICK_HZ);
static_assert(DIVISORS.valid, "TIM7 cannot make an exact 1 kHz tick");

// RM0090 38.16.4: DBGMCU_APB1_FZ, which libopencm3 only names for the F1.
#define DBGMCU_APB1_FZ MMIO32(DBGMCU_BASE + 0x08)
constexpr uint32_t DBG_TIM5_STOP = 1u << 3;
constexpr uint32_t DBG_TIM7_STOP = 1u << 5;

Handler       handler = nullptr;
TimeExtender  time_base;
volatile bool started = false;

volatile uint32_t tick_count        = 0;
volatile uint32_t last_cycles       = 0;
volatile uint32_t max_cycles        = 0;
volatile uint32_t min_period_cycles = UINT32_MAX;
volatile uint32_t max_period_cycles = 0;
uint32_t          last_entry_cycles = 0;
bool              have_last_entry   = false;

uint32_t probe_port;
uint16_t probe_bit;
} // namespace

void start(Handler tick_handler)
{
    handler = tick_handler;

    gpioOutput(board::TICK_PROBE);
    probe_port = gpioPort(board::TICK_PROBE);
    probe_bit  = gpioBit(board::TICK_PROBE);

    // Encoder timers are left running under the debugger: freezing them would
    // drop real wheel motion while halted.
    DBGMCU_APB1_FZ |= DBG_TIM5_STOP | DBG_TIM7_STOP;

    // TIM5: free-running 32-bit microsecond counter.
    rcc_periph_clock_enable(RCC_TIM5);
    rcc_periph_reset_pulse(RST_TIM5);
    TIM_PSC(TIM5) = DIVISORS.psc;
    TIM_ARR(TIM5) = 0xFFFFFFFFu;
    TIM_EGR(TIM5) = TIM_EGR_UG; // load PSC now, not at the first overflow
    TIM_CR1(TIM5) = TIM_CR1_CEN;
    time_base.reset(TIM_CNT(TIM5));

    // TIM7: 1 kHz update interrupt.
    rcc_periph_clock_enable(RCC_TIM7);
    rcc_periph_reset_pulse(RST_TIM7);
    TIM_PSC(TIM7)  = DIVISORS.psc;
    TIM_ARR(TIM7)  = DIVISORS.arr;
    TIM_CR1(TIM7)  = TIM_CR1_URS; // only overflow raises UIF, not the UG below
    TIM_EGR(TIM7)  = TIM_EGR_UG;
    TIM_SR(TIM7)   = 0;
    TIM_DIER(TIM7) = TIM_DIER_UIE;
    nvic_set_priority(NVIC_TIM7_IRQ, irq::TICK);
    nvic_enable_irq(NVIC_TIM7_IRQ);
    started = true;
    TIM_CR1(TIM7) |= TIM_CR1_CEN;
}

uint64_t nowUs()
{
    if (!started)
        return 0;
    const uint32_t mask = cm_mask_interrupts(1);
    const uint64_t now  = time_base.peek(TIM_CNT(TIM5));
    cm_mask_interrupts(mask);
    return now;
}

Stats stats()
{
    const uint32_t mask = cm_mask_interrupts(1);
    Stats          s;
    s.count             = tick_count;
    s.last_cycles       = last_cycles;
    s.max_cycles        = max_cycles;
    s.min_period_cycles = min_period_cycles;
    s.max_period_cycles = max_period_cycles;
    cm_mask_interrupts(mask);
    return s;
}

void resetStats()
{
    const uint32_t mask = cm_mask_interrupts(1);
    tick_count          = 0;
    last_cycles         = 0;
    max_cycles          = 0;
    min_period_cycles   = UINT32_MAX;
    max_period_cycles   = 0;
    have_last_entry     = false;
    cm_mask_interrupts(mask);
}
} // namespace hal::stm32::tick

using namespace hal::stm32::tick;

extern "C" void tim7_isr()
{
    const uint32_t entry  = dwt_read_cycle_counter();
    GPIO_BSRR(probe_port) = probe_bit;
    // Clear first: clearing UIF as the last write can let the ISR re-enter
    // because the write reaches the timer after the exception returns.
    TIM_SR(TIM7) = ~TIM_SR_UIF;

    if (have_last_entry)
    {
        const uint32_t period = entry - last_entry_cycles;
        if (period < min_period_cycles)
            min_period_cycles = period;
        if (period > max_period_cycles)
            max_period_cycles = period;
    }
    last_entry_cycles = entry;
    have_last_entry   = true;

    time_base.update(TIM_CNT(TIM5));
    if (handler != nullptr)
        handler();

    const uint32_t cycles = dwt_read_cycle_counter() - entry;
    last_cycles           = cycles;
    if (cycles > max_cycles)
        max_cycles = cycles;
    tick_count            = tick_count + 1;
    GPIO_BSRR(probe_port) = static_cast<uint32_t>(probe_bit) << 16;
}
