#include "hal/critical_section.h"

#include "pico/critical_section.h"

namespace hal
{
namespace
{
critical_section_t* sdk(unsigned char* storage)
{
    return reinterpret_cast<critical_section_t*>(storage);
}
} // namespace

void CriticalSection::init()
{
    static_assert(sizeof(critical_section_t) <= sizeof(storage_),
                  "critical_section_t outgrew hal::CriticalSection storage");
    static_assert(alignof(critical_section_t) <= 4, "critical_section_t needs wider alignment");
    critical_section_init(sdk(storage_));
}

void CriticalSection::enter()
{
    critical_section_enter_blocking(sdk(storage_));
}

void CriticalSection::exit()
{
    critical_section_exit(sdk(storage_));
}
} // namespace hal
