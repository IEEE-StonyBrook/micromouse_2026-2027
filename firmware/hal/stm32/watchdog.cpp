#include "hal/stm32/watchdog.h"

#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/iwdg.h>

#include "hal/stm32/watchdog_plan.h"

namespace hal::stm32::watchdog
{
namespace
{
// DBGMCU_APB1_FZ (0xE0042008) and its DBG_IWDG_STOP bit, from RM0090; libopencm3
// names only the F1's DBGMCU_CR layout.
constexpr uint32_t DBG_IWDG_STOP = 1u << 12;
volatile uint32_t& dbgmcuApb1Fz()
{
    return MMIO32(DBGMCU_BASE + 0x08);
}

static_assert(IWDG_PRESCALER == 256, "IWDG_PR below is the /256 divider");
} // namespace

void start()
{
    dbgmcuApb1Fz() |= DBG_IWDG_STOP;
    IWDG_KR = IWDG_KR_START; // also starts the LSI
    IWDG_KR = IWDG_KR_UNLOCK;
    IWDG_PR  = IWDG_PR_DIV256;
    IWDG_RLR = IWDG_RELOAD;
    while (IWDG_SR & (IWDG_SR_PVU | IWDG_SR_RVU))
    {
    }
    IWDG_KR = IWDG_KR_RESET;
}

void feed()
{
    IWDG_KR = IWDG_KR_RESET;
}
} // namespace hal::stm32::watchdog
